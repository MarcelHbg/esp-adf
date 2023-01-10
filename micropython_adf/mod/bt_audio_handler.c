#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_peripherals.h" // needed for a2dp_stream.h
#include "a2dp_stream.h" // bt audio stream

#include "audio_common.h" // audio_stream_type_t

#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"

#include "bt_audio_handler.h"

static const char *AV_TAG = "BT_AUDIO_HDL";

typedef struct {
    char device_name[BT_AUDIO_DEVICENAME_MAX_LEN + 1];
    bt_audio_state_t state;
    esp_bd_addr_t peer_bd_addr;
    bool is_connectable;
    audio_stream_type_t stream_type;
    audio_element_handle_t a2dp_stream;
    esp_periph_handle_t bt_periph;
    bt_audio_periph_hdl_t bt_periph_evt_cb;
    void * bt_periph_evt_ctx;
}bt_audio_hdl_t;

#define DEFAULT_BT_AUDIO_HDL_CONFIG() {\
    .device_name = "ESP_A2DP_SINK",\
    .state = BT_AUDIO_STATE_IDLE,\
    .peer_bd_addr = {0,0,0,0,0,0},\
    .is_connectable = false,\
    .stream_type = AUDIO_STREAM_NONE,\
    .a2dp_stream = NULL,\
    .bt_periph = NULL,\
    .bt_periph_evt_cb = NULL,\
    .bt_periph_evt_ctx = NULL,\
}

/*******************************************************************************************/
/* global obj obj pointer */

static bt_audio_hdl_t *g_bt_audio_hdl = NULL;

/*******************************************************************************************/
/* global helper functions */

/**
 * @brief call vTaskDelay to deley for the indicated number of milliseconds
 * 
 */
void delay(long millis) {
    const TickType_t xDelay = millis / portTICK_PERIOD_MS; 
    vTaskDelay(xDelay);
}

unsigned long millis() {
    return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

const char *_bd_addr_to_str(esp_bd_addr_t bda){
    static char bda_str[18];
    sprintf(bda_str, ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(bda));
    return (const char*)bda_str;
}

const char *_bool_to_str(bool x){ 
    return x ? "true" : "false";
}

void _log_free_heap(){
    ESP_LOGI(AV_TAG, "Available Heap: %zu", esp_get_free_heap_size());
}

/*******************************************************************************************/
/* event handler helper funktions*/

void handle_connection_state(esp_a2d_connection_state_t conn_state, esp_bd_addr_t bda){
    if(conn_state == ESP_A2D_CONNECTION_STATE_DISCONNECTED){
        esp_bd_addr_t _bda = {0,0,0,0,0,0};
        memcpy(g_bt_audio_hdl->peer_bd_addr, _bda, ESP_BD_ADDR_LEN);
        g_bt_audio_hdl->state = BT_AUDIO_STATE_STARTED; //????
        ESP_LOGI(AV_TAG, "[%s] A2DP disconnected bda=%s",__func__, _bd_addr_to_str(bda));
        return;
    }
    if(conn_state == ESP_A2D_CONNECTION_STATE_CONNECTED){
        memcpy(g_bt_audio_hdl->peer_bd_addr, bda, ESP_BD_ADDR_LEN);
        g_bt_audio_hdl->state = BT_AUDIO_STATE_CONNECTED;
        ESP_LOGI(AV_TAG, "[%s] A2DP connected bda=%s",__func__, _bd_addr_to_str(bda));
    }
}

/*******************************************************************************************/
/* bluetooth callbacks */

void bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            } else {
                ESP_LOGE(AV_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
            }
            break;
        }

        case ESP_BT_GAP_PIN_REQ_EVT: {
                memcpy(g_bt_audio_hdl->peer_bd_addr, param->pin_req.bda, ESP_BD_ADDR_LEN);
                ESP_LOGI(AV_TAG, "partner address: %s", _bd_addr_to_str(g_bt_audio_hdl->peer_bd_addr));
            }
            break;

        case ESP_BT_GAP_CFM_REQ_EVT: {
                memcpy(g_bt_audio_hdl->peer_bd_addr, param->cfm_req.bda, ESP_BD_ADDR_LEN);
                ESP_LOGI(AV_TAG, "partner address: %s", _bd_addr_to_str(g_bt_audio_hdl->peer_bd_addr));

                ESP_LOGI(AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please confirm the passkey: %d", param->cfm_req.num_val);
                //pin_code_int = param->key_notif.passkey;
                //pin_code_request = Confirm;
            }
            break;

        case ESP_BT_GAP_KEY_NOTIF_EVT: {
                ESP_LOGI(AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
                //pin_code_int = param->key_notif.passkey;
                //pin_code_request = Reply;
            }
            break;

        case ESP_BT_GAP_KEY_REQ_EVT: {
                ESP_LOGI(AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
                //pin_code_request = Reply;
            } 
            break;

        default: {
            ESP_LOGI(AV_TAG, "event: %d", event);
            break;
        }
    }
    return;
}

void a2dp_user_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    if(event == ESP_A2D_CONNECTION_STATE_EVT)
        handle_connection_state(param->conn_stat.state, param->conn_stat.remote_bda);
}

esp_err_t bt_periph_evt_hdl(audio_event_iface_msg_t *event, void *context){
    esp_err_t err = ESP_OK;
    if (event->source_type == PERIPH_ID_BLUETOOTH 
        && event->source == (void *)g_bt_audio_hdl->bt_periph)
    {
        switch (event->cmd)
        {
        case PERIPH_BLUETOOTH_CONNECTED:
            g_bt_audio_hdl->state = BT_AUDIO_STATE_CONNECTED;
            break;
        case PERIPH_BLUETOOTH_DISCONNECTED:
            g_bt_audio_hdl->state = BT_AUDIO_STATE_STARTED;
            break;
        default:
            break;
        }
        // run external callback
        if(g_bt_audio_hdl->bt_periph_evt_cb){
            err = g_bt_audio_hdl->bt_periph_evt_cb(event->cmd, g_bt_audio_hdl->bt_periph_evt_ctx);
            if(err){
                ESP_LOGE(AV_TAG, "[%s] bt_periph_evt_cb event=%d err=%d",__func__, event->cmd, err);
            }
        }
    }
    return err;
}

/*******************************************************************************************/
/* public bt_a2dp_sink ADT */

esp_err_t bt_audio_init(const char *device_name){
    if(g_bt_audio_hdl){
        ESP_LOGE(AV_TAG,"[%s] allready initialized", __func__);
        return ESP_ERR_ADF_ALREADY_EXISTS;
    }

    bt_audio_hdl_t *bt_audio_hdl = (bt_audio_hdl_t *)malloc(sizeof(bt_audio_hdl_t));
    if(!bt_audio_hdl){
        ESP_LOGE(AV_TAG,"[%s] malloc bt_audio_hdl_t failed", __func__);
        return ESP_ERR_NO_MEM;
    }
    
    bt_audio_hdl_t default_hdl = DEFAULT_BT_AUDIO_HDL_CONFIG();
    memcpy(bt_audio_hdl, &default_hdl, sizeof(default_hdl));
    
    if (device_name){
        strncpy(bt_audio_hdl->device_name, device_name, BT_AUDIO_DEVICENAME_MAX_LEN);
        bt_audio_hdl->device_name[BT_AUDIO_DEVICENAME_MAX_LEN] = '\0'; // enshure nulltermination
        ESP_LOGI(AV_TAG, "[%s] device name is set to: %s", __func__, bt_audio_hdl->device_name);
    }

    // save mem -> BLE not needed
    if(esp_bt_controller_mem_release(ESP_BT_MODE_BLE) != ESP_OK){
        ESP_LOGE(AV_TAG,"[%s] esp_bt_controller_mem_release BLE failed", __func__);
    }

    esp_err_t err = ESP_OK;

    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE){
        ESP_LOGI(AV_TAG, "init bt controller ...");
        esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        err = esp_bt_controller_init(&cfg);
        if (err){
            ESP_LOGE(AV_TAG, "[%s] esp_bt_controller_init err=%d", __func__, err);
            free(bt_audio_hdl);
            return err;
        }
        while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE){}
    }
    ESP_LOGI(AV_TAG, "bt controller initialized");

    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED){
        ESP_LOGI(AV_TAG, "enable bt controller ...");
        err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
        if (err) {
            ESP_LOGE(AV_TAG, "[%s] bt controller enable err=%d", __func__, err);
            free(bt_audio_hdl);
            return err;
        }
        while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED){}
    }
    ESP_LOGI(AV_TAG, "bt controller enabled");
    
    if(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED){
        ESP_LOGI(AV_TAG, "init bluedroid ...");
        err = esp_bluedroid_init();
        if (err) {
            ESP_LOGE(AV_TAG,"[%s] esp_bluedroid_init err=%d", __func__, err);
            free(bt_audio_hdl);
            return err;
        }
        while(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED){}
    }
    ESP_LOGI(AV_TAG,"bluedroid initialized");

    g_bt_audio_hdl = bt_audio_hdl;
    bt_audio_hdl->state = BT_AUDIO_STATE_INITED;
    return err;
}

void bt_audio_deinit(){
    // not initialized -> no deinit needed
    if(g_bt_audio_hdl->state == BT_AUDIO_STATE_IDLE) return;

    if(g_bt_audio_hdl->state >= BT_AUDIO_STATE_STARTED){
        bt_audio_stop();
    }

    esp_err_t err = esp_bluedroid_deinit();
    if (err){
        ESP_LOGE(AV_TAG, "[%s] esp_bluedroid_deinit err=%d", __func__, err);
    }
    ESP_LOGI(AV_TAG, "bluedroid deinitialized");
    _log_free_heap();

    // deinit controller
    err = esp_bt_controller_disable();
    if (err){
        ESP_LOGE(AV_TAG, "[%s] esp_bt_controller_disable err=%d", __func__, err);
    }

    // waiting for status change
    while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
        delay(50);
    
    err = esp_bt_controller_deinit();
    if (err){
        ESP_LOGE(AV_TAG, "[%s] esp_bt_controller_deinit err=%d", __func__, err);
    }
    ESP_LOGI(AV_TAG, "bt controller deinitialized");
    _log_free_heap();

    free(g_bt_audio_hdl);
    g_bt_audio_hdl = NULL;
}

esp_err_t bt_audio_start(bool sink){
    esp_err_t err = ESP_OK;

    if(g_bt_audio_hdl->state < BT_AUDIO_STATE_INITED){
        ESP_LOGI(AV_TAG, "[%s] not initialized", __func__);
        return err;
    }

    if(g_bt_audio_hdl->state > BT_AUDIO_STATE_INITED){
        ESP_LOGI(AV_TAG, "[%s] allready started", __func__);
        return err;
    }

    if(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED){
        ESP_LOGI(AV_TAG, "enable bluedroid ...");
        err = esp_bluedroid_enable();
        if (err) {
            ESP_LOGE(AV_TAG, "[%s] bluedroid enable err=%d", __func__, err);
            return err;
        }
        while(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED){}
    }
    ESP_LOGI(AV_TAG,"bluedroid enabled"); 

    err = esp_bt_gap_register_callback(bt_gap_callback);
    if (err) {
        ESP_LOGE(AV_TAG,"[%s] esp_bt_gap_register_callback err=%d", __func__, err);
        return err;
    }

    err = esp_bt_dev_set_device_name(g_bt_audio_hdl->device_name);
    if(err){
        ESP_LOGE(AV_TAG,"[%s] esp_bt_dev_set_device_name err=%d", __func__, err);
        return err;
    }

    g_bt_audio_hdl->stream_type = sink ? AUDIO_STREAM_READER : AUDIO_STREAM_WRITER;
    a2dp_stream_config_t a2dp_config = {
        .type = g_bt_audio_hdl->stream_type,
        .user_callback = { //0
            .user_a2d_cb = a2dp_user_callback,
            .user_a2d_sink_data_cb = NULL,
            .user_a2d_source_data_cb = NULL,
        },
    };
    g_bt_audio_hdl->a2dp_stream = a2dp_stream_init(&a2dp_config);
    if(!g_bt_audio_hdl->a2dp_stream){
        ESP_LOGE(AV_TAG,"[%s] a2dp_stream_init failed", __func__);
        return ESP_FAIL;
    }
    ESP_LOGI(AV_TAG, "[%s] stream inited streamtype=%d audioele=%p",
                        __func__,
                        g_bt_audio_hdl->stream_type,
                        g_bt_audio_hdl->a2dp_stream);
    
    esp_periph_config_t esp_periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&esp_periph_cfg);
    g_bt_audio_hdl->bt_periph = bt_create_periph();
    ESP_ERROR_CHECK(esp_periph_set_register_callback(set, bt_periph_evt_hdl, (void *)g_bt_audio_hdl->bt_periph_evt_ctx));
    ESP_ERROR_CHECK(esp_periph_start(set, g_bt_audio_hdl->bt_periph));
    ESP_LOGI(AV_TAG, "bt_periph started");
    
    g_bt_audio_hdl->state = BT_AUDIO_STATE_STARTED;

    err = bt_audio_set_connectable(true);

    return err;
}

esp_err_t bt_audio_stop(){
    esp_err_t err = ESP_OK;

    // state check not needed here allready done in a2dp_sink_disconnect
    err = bt_audio_disconnect();
    if(err) return err;
    while(g_bt_audio_hdl->state == BT_AUDIO_STATE_CONNECTED){
        delay(100);
    }
    	
    err = a2dp_destroy();
    if(err){
        ESP_LOGE(AV_TAG, "[%s] esp_a2d_sink_deinit err=%d", __func__, err);
    }
    g_bt_audio_hdl->a2dp_stream = NULL;
    ESP_LOGI(AV_TAG, "A2DP deinitialized");
    _log_free_heap();

    err = esp_bluedroid_disable();
    if (err){
        ESP_LOGE(AV_TAG, "[%s] esp_bluedroid_disable err=%d", __func__, err);
    }
    ESP_LOGI(AV_TAG, "bluedroid disabled");
    _log_free_heap();
    
    g_bt_audio_hdl->state = BT_AUDIO_STATE_INITED;
    return err;
}

esp_err_t bt_audio_set_connectable(bool connectable) {
    if(g_bt_audio_hdl->state < BT_AUDIO_STATE_STARTED){
        ESP_LOGE(AV_TAG, "[%s] not started", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(AV_TAG, "[%s] set to: %s",__func__, _bool_to_str(connectable));    
    esp_bt_scan_mode_t mode = connectable ? ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE : ESP_BT_SCAN_MODE_NONE;
    esp_err_t err = esp_bt_gap_set_scan_mode(mode);

    if (err != ESP_OK) {
        ESP_LOGE(AV_TAG,"[%s] esp_bt_gap_set_scan_mode err=%d",__func__, err);
        g_bt_audio_hdl->is_connectable = false;
    }
    else {
        g_bt_audio_hdl->is_connectable = true;
    }
    
    return err;
}

esp_err_t bt_audio_connect_to(esp_bd_addr_t bda){
    // state check not needed here allready done in bt_audio_set_connectable
    esp_err_t err = bt_audio_set_connectable(true);
    if (err) return err;
    delay(100);

    ESP_LOGI(AV_TAG, "[%s] %s",__func__, _bd_addr_to_str(bda));

    if(g_bt_audio_hdl->stream_type == AUDIO_STREAM_READER)
        err = esp_a2d_sink_connect(bda);
    else
        err = esp_a2d_source_connect(bda);

    if (err){
        ESP_LOGE(AV_TAG, "esp_a2d_source_connect err=%d", err);
        return err;
    }

    return ESP_OK;
}

esp_err_t bt_audio_disconnect(){
    if(g_bt_audio_hdl->state < BT_AUDIO_STATE_INITED){
        ESP_LOGE(AV_TAG, "[%s] not initialized", __func__);
        return ESP_ERR_INVALID_STATE;
    }
    // disconnect possible when not in connected state
    esp_err_t err = ESP_OK;

    ESP_LOGI(AV_TAG, "disconnect a2dp: %s", _bd_addr_to_str(g_bt_audio_hdl->peer_bd_addr));
    
    if(g_bt_audio_hdl->stream_type == AUDIO_STREAM_READER)
        err = esp_a2d_sink_disconnect(g_bt_audio_hdl->peer_bd_addr);
    else
        err = esp_a2d_source_disconnect(g_bt_audio_hdl->peer_bd_addr);

    if (err) {
        ESP_LOGE(AV_TAG, "[%s] disconnecting err=%d",__func__, err);
    }

    return err;
}

esp_err_t bt_audio_get_remote_addr(esp_bd_addr_t *remote_bda){
    if(g_bt_audio_hdl->state < BT_AUDIO_STATE_INITED){
        ESP_LOGE(AV_TAG, "[%s] not initialized", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    // in every state possible default return = {0,0,0,0,0,0}

    ESP_LOGI(AV_TAG, "[%s] bda: %s -> [%d,%d,%d,%d,%d,%d]",
                    __func__,
                    _bd_addr_to_str(g_bt_audio_hdl->peer_bd_addr),
                    ESP_BD_ADDR_HEX(g_bt_audio_hdl->peer_bd_addr));

    if (!remote_bda) {
        ESP_LOGE(AV_TAG, "[%s] remote_bda is NULL", __func__);
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(remote_bda, g_bt_audio_hdl->peer_bd_addr, ESP_BD_ADDR_LEN);
    return ESP_OK;
}

bool bt_audio_is_connectable(){
    if(g_bt_audio_hdl->state < BT_AUDIO_STATE_INITED){
        ESP_LOGE(AV_TAG, "[%s] not initialized", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    return g_bt_audio_hdl->is_connectable;
}

esp_err_t bt_audio_set_event_handler(bt_audio_periph_hdl_t handler, void *context){
    if(g_bt_audio_hdl->state < BT_AUDIO_STATE_INITED){
        ESP_LOGE(AV_TAG, "[%s] not initialized", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    if(!handler){
        ESP_LOGW(AV_TAG, "[%s] handler is NULL", __func__);
    }

    if(g_bt_audio_hdl->bt_periph_evt_cb){
        ESP_LOGW(AV_TAG, "[%s] connection_event_cb overwritten", __func__);
    }
    g_bt_audio_hdl->bt_periph_evt_cb = handler;
    g_bt_audio_hdl->bt_periph_evt_ctx = context;

    return ESP_OK;
}

audio_element_handle_t bt_audio_get_stream(){
    if(g_bt_audio_hdl->state < BT_AUDIO_STATE_INITED){
        ESP_LOGE(AV_TAG, "[%s] not initialized", __func__);
        return NULL;
    }

    return g_bt_audio_hdl->a2dp_stream;
}