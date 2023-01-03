#include <stdio.h>
#include <string.h>

#include "py/objstr.h"
#include "py/runtime.h"

#include "a2dp_stream.h" // bt audio stream

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_peripherals.h"

#include "freertos/FreeRTOSConfig.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#define APP_SIG_WORK_DISPATCH (0x01)

static const char *AV_TAG = "MPY_A2DP";
static const char *APP_TAG = "MPY_A2DP_TASK";

/**
 * @brief     handler for the dispatched work
 */
typedef void (* app_callback_t) (uint16_t event, void *param);

/** @brief Internal message to be sent for BluetoothA2DPSink and BluetoothA2DPSource */
typedef struct {
    uint16_t             sig;      /*!< signal to app_task */
    uint16_t             event;    /*!< message event id */
    app_callback_t       cb;       /*!< context switch callback */
    void                 *param;   /*!< parameter area needs to be last */
} app_msg_t;

typedef struct {
    UBaseType_t task_priority = configMAX_PRIORITIES - 10;
    BaseType_t task_core = 1;
    xQueueHandle app_task_queue = NULL;
    xTaskHandle app_task_handle = NULL;
    int event_queue_size = 20;
    int event_stack_size = 3072;
} app_task_t

typedef struct _bt_a2dp_obj_t {
    mp_obj_base_t base;
    mp_obj_t callback;

    esp_periph_handle_t bt_periph;
    esp_periph_set_handle_t periph_set;
    audio_event_iface_msg_t bt_event;
    mp_obj_t bt_callback;

    esp_bd_addr_t peer_bd_addr;
    esp_bd_addr_t last_connection = {0,0,0,0,0,0};
    bool autoreconnect_on = true;
    bool autoreconnect_allowed = true;

    app_task_t app_task;
} bt_a2dp_obj_t;

/*******************************************************************************************/
/* global self obj pointer */

static bt_a2dp_obj_t *self_g = NULL;

/*******************************************************************************************/
/* callback functions */

void ccall_app_a2d_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
void ccall_app_rc_ct_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
void ccall_app_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

/*******************************************************************************************/
/* global private functions */

const char *_bd_addr_to_str(esp_bd_addr_t bda){
    static char bda_str[18];
    sprintf(bda_str, "%02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return (const char*)bda_str;
}

const char *_bool_to_str(bool x){ 
    return x ? "true" : "false";
}

void _log_free_heap(){
    ESP_LOGD(AV_TAG, "Available Heap: %zu", esp_get_free_heap_size());
}

/*******************************************************************************************/
/* private init functions */

bool _init_bt_controller(){
    ESP_LOGI(AV_TAG, "init_bt_controller");
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    ESP_LOGI(AV_TAG,"esp_bt_controller_mem_release BLE");
    if(esp_bt_controller_mem_release(ESP_BT_MODE_BLE) != ESP_OK){
        ESP_LOGE(AV_TAG,"esp_bt_controller_mem_release BLE failed");
    }

    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED){
        ESP_LOGI(AV_TAG,"BT enabled"); 
        return true;
    }
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE){
        esp_bt_controller_init(&cfg);
        while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE){}
    }
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED){
        if (esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) {
            ESP_LOGE(AV_TAG, "BT Enable failed");
            return false;
        }
    }
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED){
        ESP_LOGI(AV_TAG,"BT enabled"); 
        return true;
    }

    ESP_LOGE(AV_TAG, "BT Start failed");
    return false;
}

bool _init_bluedroid(){
    esp_bluedroid_status_t bt_stack_status = esp_bluedroid_get_status();

    if(bt_stack_status == ESP_BLUEDROID_STATUS_UNINITIALIZED){
        if (esp_bluedroid_init() != ESP_OK) {
            ESP_LOGE(AV_TAG,"Failed to initialize bluedroid");
            return false;
        }
        ESP_LOGI(AV_TAG,"bluedroid initialized");
    }

    while(bt_stack_status != ESP_BLUEDROID_STATUS_ENABLED){
        if (esp_bluedroid_enable() != ESP_OK) {
            ESP_LOGE(AV_TAG,"Failed to enable bluedroid");
            delay(100);
            //return false;
        } else {
            ESP_LOGI(AV_TAG,"bluedroid enabled"); 
        }
        bt_stack_status = esp_bluedroid_get_status();
    }

    if (esp_bt_gap_register_callback(ccall_app_gap_callback) != ESP_OK) {
        ESP_LOGE(AV_TAG,"gap register failed");
        return false;
    }
    /*
    if ((esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(AV_TAG,"esp_spp_init failed");
        return false;
    }*/

    return true;
}

/*******************************************************************************************/
/* bt_a2dp class */

bt_a2dp_obj_t *init(const char *device_name){
    bt_a2dp_obj_t self = m_malloc(sizeof(_bt_a2dp_obj_t));
    if(!self) {
        ESP_LOGE(AV_TAG,"m_malloc failed");
        return NULL;
    }
    // save address global
    self_g = &self;

    const char *_name = device_name != NULL ? device_name : "ESP_MPY_AUDIO";

    if(!_init_bt_controller()){
        return NULL;
    }

    if(!_init_bluedroid()){
        return NULL;
    }

    // create application task 
    app_task_start_up(&self.app_task);

    ESP_LOGI(AV_TAG,"esp_bt_dev_set_device_name: %s", _name);
    if(esp_bt_dev_set_device_name(_name) != ESP_OK){
        ESP_LOGE(AV_TAG,"esp_bt_dev_set_device_name failed");
    }
}

void deinit(bt_a2dp_obj_t *self){
    disconnect(self);
    while(is_connected()){
        delay(100);
    }

    ESP_LOGI(AV_TAG,"deinit avrc");
    if (esp_avrc_ct_deinit() != ESP_OK){
         ESP_LOGE(AV_TAG,"Failed to deinit avrc");
    }
    _log_free_heap();

    ESP_LOGI(AV_TAG,"disable bluetooth");
    if (esp_bluedroid_disable() != ESP_OK){
        ESP_LOGE(AV_TAG,"Failed to disable bluetooth");
    }
    _log_free_heap();

    ESP_LOGI(AV_TAG,"deinit bluetooth");
    if (esp_bluedroid_deinit() != ESP_OK){
        ESP_LOGE(AV_TAG,"Failed to deinit bluetooth");
    }
    _log_free_heap();

    ESP_LOGI(AV_TAG,"esp_bt_controller_disable");
    if (esp_bt_controller_disable() != ESP_OK){
        ESP_LOGE(AV_TAG,"esp_bt_controller_disable failed");
    }
    _log_free_heap();

    // waiting for status change
    while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
        delay(50);

    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED){
        ESP_LOGI(AV_TAG,"esp_bt_controller_deinit");
        if (esp_bt_controller_deinit() != ESP_OK){
            ESP_LOGE(AV_TAG,"esp_bt_controller_deinit failed");
        }
        _log_free_heap();
    }

    m_free(self);
    self = NULL;
    self_g = NULL;
}

// static method
void set_scan_mode_connectable(bool connectable) {
    ESP_LOGI(AV_TAG, "set_scan_mode_connectable %s", _bool_to_str(connectable));             
    if (connectable){
        if (esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE) != ESP_OK){
            ESP_LOGE(AV_TAG,"esp_bt_gap_set_scan_mode");            
        } 
    } else {
        if (esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_NONE) != ESP_OK){
            ESP_LOGE(AV_TAG,"esp_bt_gap_set_scan_mode");            
        }    
    }
}

bool connect_to(bt_a2dp_obj_t *self, esp_bd_addr_t peer){
    ESP_LOGW(AV_TAG, "connect_to to %s", _bd_addr_to_str(peer));
    set_scan_mode_connectable(true);
    delay(100);
    esp_err_t err = esp_a2d_connect(peer);

    if (err!=ESP_OK){
        ESP_LOGE(AV_TAG, "esp_a2d_source_connect:%d", err);
    }

    self.last_connection = peer;
    return err==ESP_OK;
}

void disconnect(bt_a2dp_obj_t *self){
    ESP_LOGI(AV_TAG, "disconnect a2dp: %s", _bd_addr_to_str(self->last_connection));

    // Prevent automatic reconnect
    self->autoreconnect_allowed = false;

    esp_err_t status = esp_a2d_sink_disconnect(self->last_connection);
    if (status == ESP_FAIL) {
        ESP_LOGE(AV_TAG, "Failed disconnecting to device!");
    }
}

esp_bd_addr_t get_connected_addr(bt_a2dp_obj_t *self){
    return self->peer_bd_addr;
}

bool is_connected(bt_a2dp_obj_t *self){
    return true;
}

void set_autoreconnect(bt_a2dp_obj_t *self, bool state){
    self.autoreconnect_on = state;
    ESP_LOGI(AV_TAG, "set_autoreconnect: %s", _bool_to_str(state));
}

/*******************************************************************************************/
/* app task methods */

bool app_work_dispatch(app_task_t *app_task, app_callback_t p_cback, uint16_t event, void *p_params, int param_len)
{
    ESP_LOGD(APP_TAG, "%s event 0x%x, param len %d", __func__, event, param_len);
    
    app_msg_t msg;
    memset(&msg, 0, sizeof(app_msg_t));

    msg.sig = APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return app_send_msg(app_task, &msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            return app_send_msg(app_task, &msg);
        }
    }

    return false;
}

void app_work_dispatched(app_msg_t *msg)
{
    ESP_LOGD(AV_TAG, "%s", __func__);
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}


bool app_send_msg(app_task_t *app_task, app_msg_t *msg)
{
    ESP_LOGD(AV_TAG, "%s", __func__);
    if (msg == NULL || app_task->app_task_queue == NULL) {
        ESP_LOGE(APP_TAG, "%s app_send_msg failed", __func__);
        return false;
    }

    if (xQueueSend(app_task->app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(APP_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}

void app_task_handler(void *arg){
    ESP_LOGD(AV_TAG, "%s", __func__);
    
    app_task_t *app_task = (app_task_t *)arg;

    app_msg_t msg;
    while (true) {
        if (!app_task->app_task_queue){
            ESP_LOGE(APP_TAG, "%s, app_task_queue is null", __func__);
            delay(100);
        } else if (pdTRUE == xQueueReceive(app_task->app_task_queue, &msg, (portTickType)portMAX_DELAY)) {
            ESP_LOGD(APP_TAG, "%s, sig 0x%x, 0x%x", __func__, msg.sig, msg.event);
            switch (msg.sig) {
            case APP_SIG_WORK_DISPATCH:
                ESP_LOGI(APP_TAG, "%s, APP_SIG_WORK_DISPATCH sig: %d", __func__, msg.sig);
                app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(APP_TAG, "%s, unhandled sig: %d", __func__, msg.sig);
                break;
            } // switch (msg.sig)

            if (msg.param) {
                free(msg.param);
            }
        } else {
            ESP_LOGI(APP_TAG, "%s, xQueueReceive -> no data", __func__);
            delay(10);
        }
    }
}

void app_task_start_up(app_task_t *app_task){
    ESP_LOGD(AV_TAG, "%s", __func__);
    if (app_task->app_task_queue==NULL) 
        app_task->app_task_queue = xQueueCreate(app_task->event_queue_size, sizeof(app_msg_t));

    if (app_task->app_task_handle==NULL) {
        if (xTaskCreatePinnedToCore(app_task_handler, "BtAppT", app_task->event_stack_size, (void *)app_task, app_task->task_priority, &app_task->app_task_handle, app_task->task_core) != pdPASS){
            ESP_LOGE(AV_TAG, "%s failed", __func__);
        }
    }
}

void app_task_shut_down(app_task_t *app_task){
    ESP_LOGD(AV_TAG, "%s", __func__);
    if (app_task->app_task_handle!=NULL) {
        vTaskDelete(app_task->app_task_handle);
        app_task->app_task_handle = NULL;
    }
    if (app_task->app_task_queue!=NULL) {
        vQueueDelete(app_task->app_task_queue);
        app_task->app_task_queue = NULL;
    }
}

void app_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
              //  esp_log_buffer_hex(AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);

            } else {
                ESP_LOGE(AV_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
                // reset pin_code data to "undefined" after authentication failure
                // just like when in disconnected state
                //pin_code_int = 0;
                //pin_code_request = Undefined;
            }
            break;
        }

        case ESP_BT_GAP_PIN_REQ_EVT: {
                memcpy(self_g->peer_bd_addr, param->pin_req.bda, ESP_BD_ADDR_LEN);
                ESP_LOGI(AV_TAG, "partner address: %s", to_str(self_g->peer_bd_addr));
            }
            break;

        case ESP_BT_GAP_CFM_REQ_EVT: {
                memcpy(self_g->peer_bd_addr, param->cfm_req.bda, ESP_BD_ADDR_LEN);
                ESP_LOGI(AV_TAG, "partner address: %s", to_str(self_g->peer_bd_addr));

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

