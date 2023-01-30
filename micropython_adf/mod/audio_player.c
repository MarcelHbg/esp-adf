/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2019 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <stdio.h>
#include <string.h>

#include "py/objstr.h"
#include "py/runtime.h"

#include "esp_audio.h"

#include "audio_hal.h"
#include "board.h"

#include "mp3_decoder.h" // http/ file mp3 stream
#include "aac_decoder.h" // http aac stream

#include "pcm_decoder.h" // decoder for bt stream

#include "http_stream.h"
#include "i2s_stream.h"
#include "vfs_stream.h"

#include "a2dp_stream.h" // bt audio stream

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_peripherals.h"

static const char *TAG = "MPY_AUDIO";

typedef struct _audio_player_obj_t {
    mp_obj_base_t base;
    mp_obj_t callback;

    esp_audio_handle_t player;
    esp_audio_state_t state;

    bool bt_connected;
    bool bt_enabled;
    bool bt_scanning;
    esp_bd_addr_t peer_bda;
    esp_bd_addr_t scan_bda;
    esp_periph_handle_t bt_periph;
    esp_periph_set_handle_t periph_set;
    bool bt_block_mp_cb;
    mp_obj_t bt_callback;
} audio_player_obj_t;

STATIC audio_player_obj_t *g_audio_player = NULL;

STATIC const qstr player_info_fields[] = {
    MP_QSTR_input, MP_QSTR_codec
};

STATIC const MP_DEFINE_STR_OBJ(player_info_input_obj, "http|file|bluetooth stream");
STATIC const MP_DEFINE_STR_OBJ(player_info_codec_obj, "mp3|aac|pcm");

STATIC MP_DEFINE_ATTRTUPLE(
    player_info_obj,
    player_info_fields,
    2,
    (mp_obj_t)&player_info_input_obj,
    (mp_obj_t)&player_info_codec_obj);

STATIC mp_obj_t player_info(void)
{
    return (mp_obj_t)&player_info_obj;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(audio_player_info_obj, player_info);

/********************************************************************************************************/

STATIC void audio_state_cb(esp_audio_state_t *state, void *ctx)
{
    audio_player_obj_t *self = (audio_player_obj_t *)ctx;
    memcpy(&self->state, state, sizeof(esp_audio_state_t));
    if (self->callback != mp_const_none) {
        mp_obj_dict_t *dict = mp_obj_new_dict(3);

        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_status), MP_OBJ_TO_PTR(mp_obj_new_int(state->status)));
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_err_msg), MP_OBJ_TO_PTR(mp_obj_new_int(state->err_msg)));
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_media_src), MP_OBJ_TO_PTR(mp_obj_new_int(state->media_src)));

        mp_sched_schedule(self->callback, dict);
    }
}

/********************************************************************************************************/

STATIC int _http_stream_event_handle(http_stream_event_msg_t *msg)
{
    if (msg->event_id == HTTP_STREAM_RESOLVE_ALL_TRACKS) {
        return ESP_OK;
    }

    if (msg->event_id == HTTP_STREAM_FINISH_TRACK) {
        return http_stream_next_track(msg->el);
    }
    if (msg->event_id == HTTP_STREAM_FINISH_PLAYLIST) {
        return http_stream_restart(msg->el);
    }
    return ESP_OK;
}

/********************************************************************************************************/
// Bluetooth helper functions

#define BD_ADDR_STR_LEN 18

STATIC const char *_bda_to_str(esp_bd_addr_t bda){
    static char _str[BD_ADDR_STR_LEN] = {0};
    snprintf(_str, BD_ADDR_STR_LEN, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));
    _str[BD_ADDR_STR_LEN - 1] = '\0';
    return _str;
}

STATIC uint8_t *_str_to_bda(const char *str){
    unsigned int _raw_bda[ESP_BD_ADDR_LEN];
    sscanf(str, "%x:%x:%x:%x:%x:%x", ESP_BD_ADDR_HEX(&_raw_bda));
    static uint8_t _bda[ESP_BD_ADDR_LEN]; 
    for(int i = 0; i<ESP_BD_ADDR_LEN; i++){
        _bda[i] = _raw_bda[i];
    }
    ESP_LOGI(TAG, "[%s] str=%s addr=%s", __func__, str, _bda_to_str(_bda));
    return _bda;
}

STATIC esp_err_t audio_player_bt_connectable(bool enable){
    esp_err_t err = ESP_OK;

    if(enable)
        err = esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
    else
        err = esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_NONE);
    
    if(err){
        ESP_LOGE(TAG, "[%s] err=%d",__func__, err);
    }
    else{
        ESP_LOGI(TAG, "Bluetooth %s", enable ? "connectable" : "NOT connectable");
    }

    return err;
}

/********************************************************************************************************/
// Bluetooth event callbacks

STATIC void call_mp_callback(audio_player_obj_t *self, int event){
    // run mp callback if set
    if (!self->bt_block_mp_cb && self->bt_callback != mp_const_none) {
        mp_obj_dict_t *dict = mp_obj_new_dict(2);

        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_cmd), MP_OBJ_TO_PTR(mp_obj_new_int(event)));
        mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_bda), MP_OBJ_TO_PTR(mp_obj_new_str(_bda_to_str(self->peer_bda), BD_ADDR_STR_LEN)));

        mp_sched_schedule(self->bt_callback, dict);
    }
}

STATIC esp_err_t periph_event_cb(audio_event_iface_msg_t *msg, void *ctx){
    audio_player_obj_t *self = (audio_player_obj_t *)ctx;

    if (msg->source_type == PERIPH_ID_BLUETOOTH 
        && msg->source == (void *)self->bt_periph) {
        // save address
        if(msg->data && msg->data_len == sizeof(esp_bd_addr_t)){
            memcpy(self->peer_bda, msg->data, sizeof(esp_bd_addr_t));
        }
        else{
            esp_bd_addr_t _bda = {0,0,0,0,0,0};
            memcpy(self->peer_bda, _bda, sizeof(esp_bd_addr_t));
        }
        // free data if needed
        if(msg->need_free_data){
            free(msg->data);
        }

        // handle connection state
        if(msg->cmd == PERIPH_BLUETOOTH_CONNECTED){
            self->bt_connected = true;
            audio_player_bt_connectable(false);
        } 
        if(msg->cmd == PERIPH_BLUETOOTH_DISCONNECTED){
            self->bt_connected = false;
            if(self->bt_enabled) audio_player_bt_connectable(true);
        }

        // run mp callback if set
        call_mp_callback(self, msg->cmd);
    }

    return ESP_OK;
}

STATIC void gap_event_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param){
    switch (event){
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED){
            g_audio_player->bt_scanning = true;
            ESP_LOGI(TAG, "bluetooth scanning ...");
        }
        else {
            g_audio_player->bt_scanning = false;
            ESP_LOGI(TAG, "bluetooth scanning stopped");
        }
        break;
    
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(TAG, "discoverd bda: %s", _bda_to_str(param->disc_res.bda));
        memcpy(g_audio_player->scan_bda, param->disc_res.bda, sizeof(esp_bd_addr_t));
        break;
    
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "confirmation bda: %s", _bda_to_str(param->cfm_req.bda));
        break;

    case ESP_BT_GAP_AUTH_CMPL_EVT:
        ESP_LOGI(TAG, "authentication bda: %s", _bda_to_str(param->auth_cmpl.bda));
        break;

    default:
        ESP_LOGI(TAG, "GAP event: %d [%s]",event , _bda_to_str(param->auth_cmpl.bda));
        break;
    }
}

/********************************************************************************************************/
// constructor functions

STATIC esp_audio_handle_t audio_player_create(const char *device_name){
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Create Audio player");

    // init bluetooth
    ESP_LOGI(TAG, "Initialize Bluetooth");
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret |= esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    ret |= esp_bt_controller_init(&bt_cfg);
    ret |= esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    ret |= esp_bluedroid_init();
    ret |= esp_bluedroid_enable();
    ret |= esp_bt_gap_register_callback(gap_event_cb);
    if (device_name) ret |= esp_bt_dev_set_device_name(device_name);
    else ret |= esp_bt_dev_set_device_name("ESP_MPY_SPEAKER");
    ESP_LOGI(TAG, "BT controller startet (return: %d)", ret);

    // init audio board
    //ESP_LOGI(TAG, "Initialize Audio Board");
    //audio_board_handle_t board_handle = audio_board_init();
    //audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    // init player
    ESP_LOGI(TAG, "Initialize Audio Player");
    esp_audio_cfg_t cfg = DEFAULT_ESP_AUDIO_CONFIG();
    //cfg.vol_handle = board_handle->audio_hal;
    //cfg.vol_set = (audio_volume_set)audio_hal_set_volume;
    //cfg.vol_get = (audio_volume_get)audio_hal_get_volume;
    cfg.resample_rate = 48000;
    cfg.prefer_type = ESP_AUDIO_PREFER_MEM;
    esp_audio_handle_t player = esp_audio_create(&cfg);

    // add input stream
    // fatfs stream
    ESP_LOGI(TAG, "Initialize input File Stream");
    vfs_stream_cfg_t fs_reader = VFS_STREAM_CFG_DEFAULT();
    fs_reader.type = AUDIO_STREAM_READER;
    fs_reader.task_core = 1;
    esp_audio_input_stream_add(player, vfs_stream_init(&fs_reader));
    // http stream
    ESP_LOGI(TAG, "Initialize input Http Stream");
    http_stream_cfg_t http_cfg = HTTP_STREAM_CFG_DEFAULT();
    http_cfg.event_handle = _http_stream_event_handle;
    http_cfg.type = AUDIO_STREAM_READER;
    http_cfg.enable_playlist_parser = true;
    http_cfg.task_core = 1;
    audio_element_handle_t http_stream_reader = http_stream_init(&http_cfg);
    esp_audio_input_stream_add(player, http_stream_reader);

    // bt stream
    ESP_LOGI(TAG, "Initialize input a2dp Stream");
    a2dp_stream_config_t a2dp_config = {
        .type = AUDIO_STREAM_READER,
        .user_callback = {0},
    };
    audio_element_handle_t bt_stream_reader = a2dp_stream_init(&a2dp_config);
    esp_audio_input_stream_add(player, bt_stream_reader);

    // add decoder
    // mp3 for file/http stream
    ESP_LOGI(TAG, "Initialize mp3 Decoder");
    mp3_decoder_cfg_t mp3_dec_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_dec_cfg.task_core = 1;
    esp_audio_codec_lib_add(player, AUDIO_CODEC_TYPE_DECODER, mp3_decoder_init(&mp3_dec_cfg));
    // aac for http stream
    ESP_LOGI(TAG, "Initialize aac Decoder");
    aac_decoder_cfg_t aac_dec_cfg = DEFAULT_AAC_DECODER_CONFIG();
    aac_dec_cfg.task_core = 1;
    esp_audio_codec_lib_add(player, AUDIO_CODEC_TYPE_DECODER, aac_decoder_init(&aac_dec_cfg));
    // pcm for bt stream
    ESP_LOGI(TAG, "Initialize pcm Decoder");
    pcm_decoder_cfg_t pcm_dec_cfg = DEFAULT_PCM_DECODER_CONFIG();
    pcm_dec_cfg.task_core = 1;
    esp_audio_codec_lib_add(player, AUDIO_CODEC_TYPE_DECODER, pcm_decoder_init(&pcm_dec_cfg));

    // Create writers and add to esp_audio
    ESP_LOGI(TAG, "Initialize output I2S Stream");
    i2s_stream_cfg_t i2s_writer = I2S_STREAM_CFG_DEFAULT();
    i2s_writer.type = AUDIO_STREAM_WRITER;
    i2s_writer.i2s_config.sample_rate = 48000;
    i2s_writer.task_core = 1;
    esp_audio_output_stream_add(player, i2s_stream_init(&i2s_writer));

    ESP_LOGI(TAG, "Create Audio player done");
    return player;
}

STATIC mp_obj_t audio_player_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 2, 2, false);

    const char *device_name = mp_obj_str_get_str(args[0]);

    static esp_audio_handle_t basic_player = NULL;

    audio_player_obj_t *self = m_new_obj_with_finaliser(audio_player_obj_t);
    self->base.type = type;
    self->callback = args[1];
    if (basic_player == NULL) {
        basic_player = audio_player_create(device_name);
    }
    self->player = basic_player;

    // init bt peripheral
    ESP_LOGI(TAG, "Initialize Bluetooth peripheral");
    esp_periph_config_t esp_periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    self->periph_set = esp_periph_set_init(&esp_periph_cfg);
    self->bt_periph = bt_create_periph();
    esp_err_t err = esp_periph_set_register_callback(self->periph_set, periph_event_cb, (void *)self);
    err |= esp_periph_start(self->periph_set, self->bt_periph);
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    evt_cfg.external_queue_size = 10;
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);
    err |= audio_event_iface_set_listener(esp_periph_set_get_event_iface(self->periph_set), evt);
    if (err){
        ESP_LOGE(TAG, "[%s] BT init periph failed err=%d", __func__, err);
    }
    ESP_LOGI(TAG, "BT peripheral started");

    self->bt_callback = mp_const_none;
    esp_bd_addr_t _bda = {0,0,0,0,0,0};
    memcpy(self->peer_bda, _bda, sizeof(esp_bd_addr_t));
    memcpy(self->scan_bda, _bda, sizeof(esp_bd_addr_t));
    self->bt_connected = false;
    self->bt_enabled = false;
    self->bt_scanning = false;
    self->bt_block_mp_cb = false;

    g_audio_player = self;
    return MP_OBJ_FROM_PTR(self);
}

/********************************************************************************************************/
// Bluetooth methods

STATIC mp_obj_t audio_player_bt_callback(mp_obj_t self_in, mp_obj_t callback){
    audio_player_obj_t *self = self_in;
    
    self->bt_callback = callback;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(audio_player_bt_callback_obj, audio_player_bt_callback);

STATIC mp_obj_t audio_player_bt_enable(mp_obj_t self_in, mp_obj_t enable){
    audio_player_obj_t *self = self_in;

    if(!self->bt_periph){
        mp_raise_ValueError("Bluetooth peripheral not initialized");
        return mp_const_none;
    }

    bool _enable = mp_obj_is_true(enable);

    self->bt_enabled = _enable;

    esp_err_t err = ESP_OK;

    if(!_enable && self->bt_connected){
        err |= esp_a2d_sink_disconnect(self->peer_bda);
        if(err){
            ESP_LOGE(TAG, "[%s] err=%d",__func__, err);
        }
    }
    else
        err = audio_player_bt_connectable(_enable);

    return mp_obj_new_int(err);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(audio_player_bt_enable_obj, audio_player_bt_enable);

STATIC mp_obj_t audio_player_bt_connect_to(mp_obj_t self_in, mp_obj_t bda_str){
    audio_player_obj_t *self = self_in;
    const char *_bda_str = mp_obj_str_get_str(bda_str);

    if(!self->bt_enabled){
        mp_raise_TypeError("Bluetooth not enabled");
        return mp_const_none;
    }

    if(self->bt_connected){
        mp_raise_TypeError("Bluetooth allready connected");
        return mp_const_none;
    }

    if(!_bda_str){
        mp_raise_TypeError("Bluetooth address string invalid");
        return mp_const_none;
    }
    
    // block mp callback shedule to handle connection state intern
    self->bt_block_mp_cb = true;

    esp_err_t err = esp_a2d_sink_connect(_str_to_bda(_bda_str));
    if(err){
        ESP_LOGE(TAG, "%s esp_a2d_sink_connect err=%d",__func__, err);
    }

    int loop_cnt = 0;
    const int max_loop = 50;
    TickType_t xDelay = 100 / portTICK_PERIOD_MS; // delay in ms
    while (!self->bt_connected && loop_cnt < max_loop){
        // wait 5s on connection
        vTaskDelay(xDelay);
        loop_cnt++;
    }

    // wait 1s on potential disconnect for peer
    xDelay = 1000 / portTICK_PERIOD_MS; 
    vTaskDelay(xDelay);
    
    self->bt_block_mp_cb = false;

    // connection successfull
    if (self->bt_connected){
        call_mp_callback(self, PERIPH_BLUETOOTH_CONNECTED);
        return mp_obj_new_int(ESP_OK);
    }

    // timeout or disconnect from peer
    return mp_obj_new_int(ESP_FAIL);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(audio_player_bt_connect_to_obj, audio_player_bt_connect_to);

STATIC mp_obj_t audio_player_bt_disconnect(mp_obj_t self_in){
    audio_player_obj_t *self = self_in;

    if(!self->bt_enabled){
        mp_raise_TypeError("Bluetooth not enabled");
        return mp_const_none;
    }

    // not connected
    if(!self->bt_connected) return mp_obj_new_int(ESP_OK);

    esp_err_t err = esp_a2d_sink_disconnect(self->peer_bda);
    if(err){
        ESP_LOGE(TAG, "[%s] esp_a2d_sink_disconnect err=%d",__func__, err);
    }

    int loop_cnt = 0;
    const int max_loop = 50;
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS; // delay in ms
    while (self->bt_connected && loop_cnt < max_loop){
        // wait 5s on disconnection
        vTaskDelay(xDelay);
        loop_cnt++;
    }
    
    // disconnection successfull
    if (!self->bt_connected) return mp_obj_new_int(ESP_OK);

    // timeout
    return mp_obj_new_int(ESP_FAIL);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_player_bt_disconnect_obj, audio_player_bt_disconnect);

STATIC mp_obj_t audio_player_bt_scan_for(mp_obj_t self_in, mp_obj_t bda_str){
    audio_player_obj_t *self = self_in;
    const char *_bda_str = mp_obj_str_get_str(bda_str);

    if(!_bda_str){
        mp_raise_TypeError("Bluetooth address string invalid");
        return mp_const_none;
    }

    esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
    if(err){
        ESP_LOGE(TAG, "%s err=%d",__func__, err);
        return mp_obj_new_int(err);
    }

    esp_bd_addr_t search_bda = {0,0,0,0,0,0};
    memcpy(search_bda, _str_to_bda(_bda_str), sizeof(esp_bd_addr_t));

    // wait on scanning started
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);

    while (self->bt_scanning){
        if (memcmp(search_bda, self->scan_bda, sizeof(esp_bd_addr_t)) == 0) {
            // address found
            esp_bt_gap_cancel_discovery();
            return mp_obj_new_int(ESP_OK);
        }
        const TickType_t xDelay = 50 / portTICK_PERIOD_MS;
        vTaskDelay(xDelay);
    }

    return mp_obj_new_int(ESP_FAIL);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(audio_player_bt_scan_for_obj, audio_player_bt_scan_for);

#define AP_BT_CMD_PLAY 1
#define AP_BT_CMD_STOP 0
#define AP_BT_CMD_PAUSE 2

STATIC mp_obj_t audio_player_bt_periph_cmd(mp_obj_t self_in, mp_obj_t cmd){
    audio_player_obj_t *self = self_in;
    int _cmd = mp_obj_get_int(cmd);

    if (AP_BT_CMD_PLAY == _cmd)
        return mp_obj_new_int(periph_bt_play(self->bt_periph));
    else if (AP_BT_CMD_PAUSE == _cmd)
        return mp_obj_new_int(periph_bt_pause(self->bt_periph));
    else if (AP_BT_CMD_STOP == _cmd)
        return mp_obj_new_int(periph_bt_stop(self->bt_periph));
    else
        return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(audio_player_bt_periph_cmd_obj, audio_player_bt_periph_cmd);

/********************************************************************************************************/

STATIC mp_obj_t audio_player_play_helper(audio_player_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum {
        ARG_uri,
        ARG_pos,
        ARG_sync,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_uri, MP_ARG_REQUIRED | MP_ARG_OBJ, { .u_obj = mp_const_none } },
        { MP_QSTR_pos, MP_ARG_INT, { .u_int = 0 } },
        { MP_QSTR_sync, MP_ARG_BOOL, { .u_obj = mp_const_false } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (args[ARG_uri].u_obj != mp_const_none) {
        const char *uri = mp_obj_str_get_str(args[ARG_uri].u_obj);
        int pos = args[ARG_pos].u_int;

        esp_audio_state_t state = { 0 };
        esp_audio_state_get(self->player, &state);
        if (state.status == AUDIO_STATUS_RUNNING || state.status == AUDIO_STATUS_PAUSED) {
            esp_audio_stop(self->player, TERMINATION_TYPE_NOW);
            int wait = 20;
            esp_audio_state_get(self->player, &state);
            while (wait-- && (state.status == AUDIO_STATUS_RUNNING || state.status == AUDIO_STATUS_PAUSED)) {
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_audio_state_get(self->player, &state);
            }
        }
        esp_audio_callback_set(self->player, audio_state_cb, self);
        if (args[ARG_sync].u_obj == mp_const_false) {
            self->state.status = AUDIO_STATUS_RUNNING;
            self->state.err_msg = ESP_ERR_AUDIO_NO_ERROR;
            return mp_obj_new_int(esp_audio_play(self->player, AUDIO_CODEC_TYPE_DECODER, uri, pos));
        } else {
            return mp_obj_new_int(esp_audio_sync_play(self->player, uri, pos));
        }
    } else {
        return mp_obj_new_int(ESP_ERR_AUDIO_INVALID_PARAMETER);
    }
}

STATIC mp_obj_t audio_player_play(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
    return audio_player_play_helper(args[0], n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_player_play_obj, 1, audio_player_play);

/********************************************************************************************************/

STATIC mp_obj_t audio_player_stop_helper(audio_player_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum {
        ARG_termination,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_termination, MP_ARG_INT, { .u_int = TERMINATION_TYPE_NOW } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    return mp_obj_new_int(esp_audio_stop(self->player, args[ARG_termination].u_int));
}

STATIC mp_obj_t audio_player_stop(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
    return audio_player_stop_helper(args[0], n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_player_stop_obj, 1, audio_player_stop);

STATIC mp_obj_t audio_player_pause(mp_obj_t self_in)
{
    audio_player_obj_t *self = self_in;
    return mp_obj_new_int(esp_audio_pause(self->player));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_player_pause_obj, audio_player_pause);

STATIC mp_obj_t audio_player_resume(mp_obj_t self_in)
{
    audio_player_obj_t *self = self_in;
    return mp_obj_new_int(esp_audio_resume(self->player));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_player_resume_obj, audio_player_resume);

STATIC mp_obj_t audio_player_vol_helper(audio_player_obj_t *self, mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum {
        ARG_vol,
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_vol, MP_ARG_INT, { .u_int = 0xffff } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if (args[ARG_vol].u_int == 0xffff) {
        int vol = 0;
        esp_audio_vol_get(self->player, &vol);
        return mp_obj_new_int(vol);
    } else {
        if (args[ARG_vol].u_int >= 0 && args[ARG_vol].u_int <= 100) {
            return mp_obj_new_int(esp_audio_vol_set(self->player, args[ARG_vol].u_int));
        } else {
            return mp_obj_new_int(ESP_ERR_AUDIO_INVALID_PARAMETER);
        }
    }
}

STATIC mp_obj_t audio_player_vol(mp_uint_t n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
    return audio_player_vol_helper(args[0], n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(audio_player_vol_obj, 1, audio_player_vol);

STATIC mp_obj_t audio_player_get_vol(mp_obj_t self_in)
{
    audio_player_obj_t *self = self_in;
    int vol = 0;
    esp_audio_vol_get(self->player, &vol);
    return mp_obj_new_int(vol);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_player_get_vol_obj, audio_player_get_vol);

STATIC mp_obj_t audio_player_set_vol(mp_obj_t self_in, mp_obj_t vol)
{
    audio_player_obj_t *self = self_in;
    int volume = mp_obj_get_int(vol);
    return mp_obj_new_int(esp_audio_vol_set(self->player, volume));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(audio_player_set_vol_obj, audio_player_set_vol);

STATIC mp_obj_t audio_player_state(mp_obj_t self_in)
{
    audio_player_obj_t *self = self_in;
    mp_obj_dict_t *dict = mp_obj_new_dict(3);

    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_status), MP_OBJ_TO_PTR(mp_obj_new_int(self->state.status)));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_err_msg), MP_OBJ_TO_PTR(mp_obj_new_int(self->state.err_msg)));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_media_src), MP_OBJ_TO_PTR(mp_obj_new_int(self->state.media_src)));

    return dict;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_player_state_obj, audio_player_state);

STATIC mp_obj_t audio_player_pos(mp_obj_t self_in)
{
    audio_player_obj_t *self = self_in;
    int pos = -1;
    int err = esp_audio_pos_get(self->player, &pos);
    if (err == ESP_ERR_AUDIO_NO_ERROR) {
        return mp_obj_new_int(pos);
    } else {
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_player_pos_obj, audio_player_pos);

STATIC mp_obj_t audio_player_time(mp_obj_t self_in)
{
    audio_player_obj_t *self = self_in;
    int time = 0;
    int err = esp_audio_time_get(self->player, &time);
    if (err == ESP_ERR_AUDIO_NO_ERROR) {
        return mp_obj_new_int(time);
    } else {
        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_player_time_obj, audio_player_time);

STATIC const mp_rom_map_elem_t player_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&audio_player_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_play), MP_ROM_PTR(&audio_player_play_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&audio_player_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_pause), MP_ROM_PTR(&audio_player_pause_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume), MP_ROM_PTR(&audio_player_resume_obj) },
    { MP_ROM_QSTR(MP_QSTR_vol), MP_ROM_PTR(&audio_player_vol_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_vol), MP_ROM_PTR(&audio_player_get_vol_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_vol), MP_ROM_PTR(&audio_player_set_vol_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_state), MP_ROM_PTR(&audio_player_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_pos), MP_ROM_PTR(&audio_player_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_time), MP_ROM_PTR(&audio_player_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_bt_callback), MP_ROM_PTR(&audio_player_bt_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_bt_enable), MP_ROM_PTR(&audio_player_bt_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_bt_connect_to), MP_ROM_PTR(&audio_player_bt_connect_to_obj) },
    { MP_ROM_QSTR(MP_QSTR_bt_disconnect), MP_ROM_PTR(&audio_player_bt_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_bt_scan_for), MP_ROM_PTR(&audio_player_bt_scan_for_obj) },
    { MP_ROM_QSTR(MP_QSTR_bt_send_cmd), MP_ROM_PTR(&audio_player_bt_periph_cmd_obj) },

    // BT COMMANDS
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_CMD_PLAY         ), MP_ROM_INT(AP_BT_CMD_PLAY)},
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_CMD_PAUSE         ), MP_ROM_INT(AP_BT_CMD_PAUSE)}, 
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_CMD_STOP         ), MP_ROM_INT(AP_BT_CMD_STOP)}, 
    
    // bt_event_t
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_UNKNOWN         ), MP_ROM_INT(PERIPH_BLUETOOTH_UNKNOWN) },                   /*!< No event */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_CONNECTED       ), MP_ROM_INT(PERIPH_BLUETOOTH_CONNECTED) },               /*!< A bluetooth device was connected */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_DISCONNECTED    ), MP_ROM_INT(PERIPH_BLUETOOTH_DISCONNECTED) },         /*!< Last connection was disconnected */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_AUDIO_STARTED   ), MP_ROM_INT(PERIPH_BLUETOOTH_AUDIO_STARTED) },       /*!< The audio session has been started */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_AUDIO_SUSPENDED ), MP_ROM_INT(PERIPH_BLUETOOTH_AUDIO_SUSPENDED) },   /*!< The audio session has been suspended */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_AUDIO_STOPPED   ), MP_ROM_INT(PERIPH_BLUETOOTH_AUDIO_STOPPED) },       /*!< The audio session has been stopped */

    // esp_audio_status_t
    { MP_ROM_QSTR(MP_QSTR_STATUS_UNKNOWN), MP_ROM_INT(AUDIO_STATUS_UNKNOWN) },
    { MP_ROM_QSTR(MP_QSTR_STATUS_RUNNING), MP_ROM_INT(AUDIO_STATUS_RUNNING) },
    { MP_ROM_QSTR(MP_QSTR_STATUS_PAUSED), MP_ROM_INT(AUDIO_STATUS_PAUSED) },
    { MP_ROM_QSTR(MP_QSTR_STATUS_STOPPED), MP_ROM_INT(AUDIO_STATUS_STOPPED) },
    { MP_ROM_QSTR(MP_QSTR_STATUS_FINISHED), MP_ROM_INT(AUDIO_STATUS_FINISHED) },
    { MP_ROM_QSTR(MP_QSTR_STATUS_ERROR), MP_ROM_INT(AUDIO_STATUS_ERROR) },

    // audio_termination_type
    { MP_ROM_QSTR(MP_QSTR_TERMINATION_NOW), MP_ROM_INT(TERMINATION_TYPE_NOW) },
    { MP_ROM_QSTR(MP_QSTR_TERMINATION_DONE), MP_ROM_INT(TERMINATION_TYPE_DONE) },
};

STATIC MP_DEFINE_CONST_DICT(player_locals_dict, player_locals_dict_table);

const mp_obj_type_t audio_player_type = {
    { &mp_type_type },
    .name = MP_QSTR_player,
    .make_new = audio_player_make_new,
    .locals_dict = (mp_obj_dict_t *)&player_locals_dict,
};

