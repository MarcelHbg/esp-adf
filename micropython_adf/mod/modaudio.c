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

#include "modaudio.h"

#include "py/objstr.h"
#include "py/runtime.h"

#include "esp_audio.h"
#include "esp_log.h"

#include "audio_mem.h"

#include "bt_audio_handler.h"

const char *verno = "0.5-beta1";

static const char *TAG = "MPY_AUDIO";

STATIC mp_obj_t audio_mem_info(void)
{
#ifdef CONFIG_SPIRAM_BOOT_INIT
    mp_obj_dict_t *dict = mp_obj_new_dict(3);

    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_mem_total), MP_OBJ_TO_PTR(mp_obj_new_int(esp_get_free_heap_size())));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_inter), MP_OBJ_TO_PTR(mp_obj_new_int(heap_caps_get_free_size(MALLOC_CAP_INTERNAL))));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_dram), MP_OBJ_TO_PTR(mp_obj_new_int(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT))));
#else
    mp_obj_dict_t *dict = mp_obj_new_dict(1);
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_mem_total), MP_OBJ_TO_PTR(mp_obj_new_int(esp_get_free_heap_size())));
#endif
    return dict;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(audio_mem_info_obj, audio_mem_info);

STATIC mp_obj_t audio_mod_verno(void)
{
    return mp_obj_new_str(verno, strlen(verno));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(audio_mod_verno_obj, audio_mod_verno);

STATIC mp_obj_t audio_enable_log(mp_obj_t enable){
    if(enable == mp_const_true)
        esp_log_level_set("*", ESP_LOG_VERBOSE);
    else if(enable == mp_const_false)
        esp_log_level_set("*", ESP_LOG_NONE);
    else
        mp_raise_TypeError("enable type must be boolean");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_enable_log_obj, audio_enable_log);

STATIC mp_obj_t audio_bt_init(mp_obj_t device_name){
    const char *_device_name = mp_obj_str_get_str(device_name);
    return mp_obj_new_int(bt_audio_init(_device_name));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_bt_init_obj, audio_bt_init);

STATIC mp_obj_t audio_bt_delete(void){
    bt_audio_deinit();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(audio_bt_delete_obj, audio_bt_delete);

STATIC mp_obj_t audio_bt_start(mp_obj_t sink){
    if(sink == mp_const_true)
        return mp_obj_new_int(bt_audio_start(true));
    else if(sink == mp_const_false)
        return mp_obj_new_int(bt_audio_start(false));
    else
        mp_raise_TypeError("sink is not boolean");
        return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_bt_start_obj, audio_bt_start);

STATIC mp_obj_t audio_bt_stop(void){
    return mp_obj_new_int(bt_audio_stop());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(audio_bt_stop_obj, audio_bt_stop);

STATIC mp_obj_t audio_bt_set_connectable(mp_obj_t connectable){
    if(connectable == mp_const_true)
        return mp_obj_new_int(bt_audio_set_connectable(true));
    else if(connectable == mp_const_false)
        return mp_obj_new_int(bt_audio_set_connectable(false));
    else
        mp_raise_TypeError("connectable is not boolean");
        return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_bt_set_connectable_obj, audio_bt_set_connectable);

STATIC mp_obj_t audio_bt_connect_to(mp_obj_t peer){
    if (!mp_obj_is_type(peer, &mp_type_list)) {
        ESP_LOGE(TAG, "[%s] bda is not list type", __func__);
        mp_raise_TypeError("peer - is not List type");
        return mp_obj_new_int(ESP_ERR_INVALID_ARG);
    }

    size_t len;
    mp_obj_t *_obj_peer_addr;
    mp_obj_list_get(peer, &len, &_obj_peer_addr);

    if (len != ESP_BD_ADDR_LEN){
        ESP_LOGE(TAG, "[%s] list length not 6", __func__);
        mp_raise_TypeError("peer - List is not of length 6");
        return mp_obj_new_int(ESP_ERR_INVALID_SIZE);
    }

    esp_bd_addr_t _peer_addr;
    for (int i = 0; i < len; i++){
        if (!MP_OBJ_IS_SMALL_INT(_obj_peer_addr[i])) {
            ESP_LOGE(TAG, "[%s] bda[%d] is no int", __func__, i);
            mp_raise_TypeError("peer - List item is not Int type");
            return mp_obj_new_int(ESP_ERR_INVALID_ARG);
        }
        _peer_addr[i] = MP_OBJ_SMALL_INT_VALUE(_obj_peer_addr[i]);
    }

    return mp_obj_new_int(bt_audio_connect_to(_peer_addr));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_bt_connect_to_obj, audio_bt_connect_to);

STATIC mp_obj_t audio_bt_disconnect(void){
    return mp_obj_new_int(bt_audio_disconnect());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(audio_bt_disconnect_obj, audio_bt_disconnect);

STATIC mp_obj_t audio_bt_get_remote_addr(void){
    esp_bd_addr_t bda = {0,0,0,0,0,0};
    esp_err_t err = bt_audio_get_remote_addr(&bda);
    if (err != ESP_OK){
        return mp_obj_new_int(err);
    }

    mp_obj_t _bda_list[ESP_BD_ADDR_LEN];
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++){
        _bda_list[i] = mp_obj_new_int(bda[i]);
    }

    return mp_obj_new_list(ESP_BD_ADDR_LEN, _bda_list);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(audio_bt_get_remote_addr_obj, audio_bt_get_remote_addr);

STATIC mp_obj_t audio_bt_is_connectable(void){
    if (bt_audio_is_connectable()) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(audio_bt_is_connectable_obj, audio_bt_is_connectable);

extern const mp_obj_type_t audio_player_type;
extern const mp_obj_type_t audio_recorder_type;

STATIC const mp_rom_map_elem_t audio_module_globals_table[] = {
    // module init
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_BT_AUDIO) },
    // module functions
    { MP_ROM_QSTR(MP_QSTR_enableLog), MP_ROM_PTR(&audio_enable_log_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem_info), MP_ROM_PTR(&audio_mem_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_verno), MP_ROM_PTR(&audio_mod_verno_obj) },
    { MP_ROM_QSTR(MP_QSTR_btInit), MP_ROM_PTR(&audio_bt_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_btDeinit), MP_ROM_PTR(&audio_bt_delete_obj) },
    { MP_ROM_QSTR(MP_QSTR_btStart), MP_ROM_PTR(&audio_bt_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_btStop), MP_ROM_PTR(&audio_bt_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_btSetConnectable), MP_ROM_PTR(&audio_bt_set_connectable_obj) },
    { MP_ROM_QSTR(MP_QSTR_btConnectTo), MP_ROM_PTR(&audio_bt_connect_to_obj) },
    { MP_ROM_QSTR(MP_QSTR_btDisconnect), MP_ROM_PTR(&audio_bt_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_btGetRemoteAddr), MP_ROM_PTR(&audio_bt_get_remote_addr_obj) },
    { MP_ROM_QSTR(MP_QSTR_btIsConnectable), MP_ROM_PTR(&audio_bt_is_connectable_obj) },
    
    // classes
    { MP_ROM_QSTR(MP_QSTR_player), MP_ROM_PTR(&audio_player_type) },
    { MP_ROM_QSTR(MP_QSTR_recorder), MP_ROM_PTR(&audio_recorder_type) },

    // bt_event_t
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_UNKNOWN         ), MP_ROM_INT(PERIPH_BLUETOOTH_UNKNOWN) },                   /*!< No event */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_CONNECTED       ), MP_ROM_INT(PERIPH_BLUETOOTH_CONNECTED) },               /*!< A bluetooth device was connected */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_DISCONNECTED    ), MP_ROM_INT(PERIPH_BLUETOOTH_DISCONNECTED) },         /*!< Last connection was disconnected */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_AUDIO_STARTED   ), MP_ROM_INT(PERIPH_BLUETOOTH_AUDIO_STARTED) },       /*!< The audio session has been started */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_AUDIO_SUSPENDED ), MP_ROM_INT(PERIPH_BLUETOOTH_AUDIO_SUSPENDED) },   /*!< The audio session has been suspended */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_AUDIO_STOPPED   ), MP_ROM_INT(PERIPH_BLUETOOTH_AUDIO_STOPPED) },       /*!< The audio session has been stopped */
    { MP_ROM_QSTR(MP_QSTR_BLUETOOTH_CONNECTION_LOST ), MP_ROM_INT(PERIPH_BLUETOOTH_CONNECTION_LOST) },       /*!< The audio session has been stopped */

    // audio_err_t
    { MP_ROM_QSTR(MP_QSTR_AUDIO_OK), MP_ROM_INT(ESP_ERR_AUDIO_NO_ERROR) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_FAIL), MP_ROM_INT(ESP_ERR_AUDIO_FAIL) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_NO_INPUT_STREAM), MP_ROM_INT(ESP_ERR_AUDIO_NO_INPUT_STREAM) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_NO_OUTPUT_STREAM), MP_ROM_INT(ESP_ERR_AUDIO_NO_OUTPUT_STREAM) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_NO_CODEC), MP_ROM_INT(ESP_ERR_AUDIO_NO_CODEC) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_HAL_FAIL), MP_ROM_INT(ESP_ERR_AUDIO_HAL_FAIL) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_MEMORY_LACK), MP_ROM_INT(ESP_ERR_AUDIO_MEMORY_LACK) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_INVALID_URI), MP_ROM_INT(ESP_ERR_AUDIO_INVALID_URI) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_INVALID_PATH), MP_ROM_INT(ESP_ERR_AUDIO_INVALID_PATH) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_INVALID_PARAMETER), MP_ROM_INT(ESP_ERR_AUDIO_INVALID_PARAMETER) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_NOT_READY), MP_ROM_INT(ESP_ERR_AUDIO_NOT_READY) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_NOT_SUPPORT), MP_ROM_INT(ESP_ERR_AUDIO_NOT_SUPPORT) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_TIMEOUT), MP_ROM_INT(ESP_ERR_AUDIO_TIMEOUT) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_ALREADY_EXISTS), MP_ROM_INT(ESP_ERR_AUDIO_ALREADY_EXISTS) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_LINK_FAIL), MP_ROM_INT(ESP_ERR_AUDIO_LINK_FAIL) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_UNKNOWN), MP_ROM_INT(ESP_ERR_AUDIO_UNKNOWN) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_OPEN), MP_ROM_INT(ESP_ERR_AUDIO_OPEN) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_INPUT), MP_ROM_INT(ESP_ERR_AUDIO_INPUT) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_PROCESS), MP_ROM_INT(ESP_ERR_AUDIO_PROCESS) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_OUTPUT), MP_ROM_INT(ESP_ERR_AUDIO_OUTPUT) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_CLOSE), MP_ROM_INT(ESP_ERR_AUDIO_CLOSE) },
};

STATIC MP_DEFINE_CONST_DICT(audio_module_globals, audio_module_globals_table);

const mp_obj_module_t audio_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&audio_module_globals,
};
