#include <stdio.h>
#include <string.h>

#include "py/objstr.h"
#include "py/runtime.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_peripherals.h"

#include "freertos/timers.h"
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "bt_a2dp_sink_handler.h"

static const char *AV_TAG = "MPY_A2DP";
static const char *APP_TAG = "MPY_A2DP_TASK";

#define APP_SIG_WORK_DISPATCH (0x01)

/**
 * @brief     handler for the dispatched work
 */
typedef void (* app_callback_t) (uint16_t event, void *param);

/* @brief event for handler "av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/** @brief Internal message to be sent for BluetoothA2DPSink and BluetoothA2DPSource */
typedef struct {
    uint16_t             sig;      /*!< signal to app_task */
    uint16_t             event;    /*!< message event id */
    app_callback_t       cb;       /*!< context switch callback */
    void                 *param;   /*!< parameter area needs to be last */
} app_msg_t;

struct _a2dp_task_t {
    UBaseType_t task_priority;
    BaseType_t task_core;
    xQueueHandle app_task_queue;
    xTaskHandle app_task_handle;
    int event_queue_size;
    int event_stack_size;
};
typedef struct _a2dp_task_t a2dp_task_t;
#define DEFAULT_A2DP_TASK_CONFIG() {\
    .task_priority = configMAX_PRIORITIES - 10,\
    .task_core = 1,\
    .app_task_queue = NULL,\
    .app_task_handle = NULL,\
    .event_queue_size = 20,\
    .event_stack_size = 3072,\
}

struct _bt_a2dp_obj_t {
    char *device_name;
    esp_periph_handle_t bt_periph;
    esp_periph_set_handle_t periph_set;
    //audio_event_iface_msg_t bt_event;
    esp_bd_addr_t peer_bd_addr;
    esp_bd_addr_t last_connection;
    bool is_connected;
    bool autoreconnect_on;
    bool autoreconnect_allowed;
    a2dp_task_t app_task;
};
#define DEFAULT_A2DP_OBJ_CONFIG() {\
    .device_name = "ESP_MP_A2DP_SINK",\
    .bt_periph = NULL,\
    .periph_set = NULL,\
    .peer_bd_addr = {0,0,0,0,0,0},\
    .last_connection = {0,0,0,0,0,0},\
    .is_connected = false,\
    .autoreconnect_on = true,\
    .autoreconnect_allowed = false,\
    .app_task = DEFAULT_A2DP_TASK_CONFIG(),\
}

/*******************************************************************************************/
/* global obj obj pointer */

static bt_a2dp_obj_t *obj_g = NULL;

/*******************************************************************************************/
/* private function declarations*/

// task functions
void app_task_start_up(a2dp_task_t *app_task);
void app_task_shut_down(a2dp_task_t *app_task);
bool app_work_dispatch(a2dp_task_t *app_task, app_callback_t p_cback, uint16_t event, void *p_params, int param_len);

// event handler
void av_hdl_stack_evt(uint16_t event, void *p_param);

// bt callbacks
void bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

/*******************************************************************************************/
/* global private functions */

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

    if (esp_bt_gap_register_callback(bt_gap_callback) != ESP_OK) {
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

bt_a2dp_obj_t *a2dp_init_bt(const char *device_name){
    
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set(APP_TAG, ESP_LOG_DEBUG);
    esp_log_level_set(AV_TAG, ESP_LOG_DEBUG);

    ESP_LOGD(AV_TAG, "%s called", __func__);

    bt_a2dp_obj_t *obj = NULL;
    /*bt_a2dp_obj_t *obj = m_malloc(sizeof(bt_a2dp_obj_t));
    if(!obj) {
        ESP_LOGE(AV_TAG,"%s m_malloc failed", __func__);
        return NULL;
    }

    ESP_LOGD(AV_TAG, "init default obj");
    bt_a2dp_obj_t a2dp_obj = DEFAULT_A2DP_OBJ_CONFIG();
    memcpy(obj, &a2dp_obj, sizeof(bt_a2dp_obj_t));

    ESP_LOGD(AV_TAG, "device name is set: %s", device_name);
    strcpy(obj->device_name, device_name);

    // save address global
    obj_g = obj;

    if(!_init_bt_controller()){
        return NULL;
    }

    if(!_init_bluedroid()){
        return NULL;
    }

    // create application task 
    ESP_LOGI(AV_TAG, "start_app_task");
    app_task_start_up(&obj->app_task);

    // Bluetooth device name, connection mode and profile set up
    ESP_LOGD(AV_TAG, "dispatch app stackup event");
    app_work_dispatch(&obj->app_task, av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0);

    */
    return obj;
}

void a2dp_deinit_bt(bt_a2dp_obj_t *obj){
    if (!obj) {
        ESP_LOGE(AV_TAG, "%s:bt_a2dp_obj_t is NULL", __func__);
        return;
    }

    a2dp_disconnect(obj);
    while(a2dp_is_connected(obj)){
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

    app_task_shut_down(&obj->app_task);

    m_free(obj);
    obj = NULL;
    obj_g = NULL;
}

// static method
void a2dp_set_bt_connectable(bool connectable) {
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

bool a2dp_connect_to(bt_a2dp_obj_t *obj, int *peer){
    if (!obj) {
        ESP_LOGE(AV_TAG, "%s:bt_a2dp_obj_t is NULL", __func__);
        return false;
    }

    if (!obj->autoreconnect_allowed){
        ESP_LOGE(AV_TAG, "reconnect not allowed");
        return false;
    } 

    esp_bd_addr_t _peer_addr = {peer[0], peer[1], peer[2], peer[3], peer[4], peer[5]};

    ESP_LOGW(AV_TAG, "connect_to to %s", _bd_addr_to_str(_peer_addr));
    a2dp_set_bt_connectable(true);
    delay(100);
    esp_err_t err = esp_a2d_sink_connect(_peer_addr);

    if (err!=ESP_OK){
        ESP_LOGE(AV_TAG, "esp_a2d_source_connect:%d", err);
    }

    obj->is_connected = true;
    memcpy(obj->last_connection, _peer_addr, sizeof(esp_bd_addr_t));
    return err==ESP_OK;
}

void a2dp_disconnect(bt_a2dp_obj_t *obj){
    if (!obj) {
        ESP_LOGE(AV_TAG, "%s:bt_a2dp_obj_t is NULL", __func__);
        return;
    }

    ESP_LOGI(AV_TAG, "disconnect a2dp: %s", _bd_addr_to_str(obj->last_connection));

    // Prevent automatic reconnect
    obj->autoreconnect_allowed = false;

    esp_err_t status = esp_a2d_sink_disconnect(obj->last_connection);
    if (status == ESP_FAIL) {
        ESP_LOGE(AV_TAG, "Failed disconnecting to device!");
    }
}

int *a2dp_get_connected_addr(bt_a2dp_obj_t *obj){
    if (!obj) {
        ESP_LOGE(AV_TAG, "%s:bt_a2dp_obj_t is NULL", __func__);
        return NULL;
    }
    return (int *)obj->peer_bd_addr;
}

bool a2dp_is_connected(bt_a2dp_obj_t *obj){
    if (!obj) {
        ESP_LOGE(AV_TAG, "%s:bt_a2dp_obj_t is NULL", __func__);
        return false;
    }
    return obj->is_connected;
}

void a2dp_set_autoreconnect(bt_a2dp_obj_t *obj, bool state){
    if (!obj) {
        ESP_LOGE(AV_TAG, "%s:bt_a2dp_obj_t is NULL", __func__);
        return;
    }
    obj->autoreconnect_on = state;
    ESP_LOGI(AV_TAG, "set_autoreconnect: %s", _bool_to_str(state));
}

/*******************************************************************************************/
/* app task methods */

bool app_send_msg(a2dp_task_t *app_task, app_msg_t *msg)
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

bool app_work_dispatch(a2dp_task_t *app_task, app_callback_t p_cback, uint16_t event, void *p_params, int param_len)
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

void app_task_handler(void *arg){
    ESP_LOGD(AV_TAG, "%s", __func__);
    
    a2dp_task_t *app_task = (a2dp_task_t *)arg;

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

void app_task_start_up(a2dp_task_t *app_task){
    ESP_LOGD(AV_TAG, "%s", __func__);
    if (app_task->app_task_queue==NULL) 
        app_task->app_task_queue = xQueueCreate(app_task->event_queue_size, sizeof(app_msg_t));

    if (app_task->app_task_handle==NULL) {
        if (xTaskCreatePinnedToCore(app_task_handler, "BtAppT", app_task->event_stack_size, (void *)app_task, app_task->task_priority, &app_task->app_task_handle, app_task->task_core) != pdPASS){
            ESP_LOGE(AV_TAG, "%s failed", __func__);
        }
    }
}

void app_task_shut_down(a2dp_task_t *app_task){
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

void av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(AV_TAG, "%s evt %d", __func__, event);
    //esp_err_t result;

    switch (event) {
        case BT_APP_EVT_STACK_UP: {
            ESP_LOGD(AV_TAG, "%s av_hdl_stack_evt %s", __func__, "BT_APP_EVT_STACK_UP");

            /* set up device name */
            ESP_LOGI(AV_TAG,"esp_bt_dev_set_device_name: %s", obj_g->device_name);
            esp_bt_dev_set_device_name(obj_g->device_name);  

            // initialize AVRCP controller 
            /*result = esp_avrc_ct_init();
            if (result == ESP_OK){
                result = esp_avrc_ct_register_callback(ccall_app_rc_ct_callback);
                if (result == ESP_OK){
                    ESP_LOGD(AV_TAG, "AVRCP controller initialized!");
                } else {
                    ESP_LOGE(AV_TAG,"esp_avrc_ct_register_callback: %d",result);
                }
            } else {
                ESP_LOGE(AV_TAG,"esp_avrc_ct_init: %d",result);
            }
            */
#ifdef ESP_IDF_4
            
            /* initialize AVRCP target */
            if (esp_avrc_tg_init() == ESP_OK){
                esp_avrc_tg_register_callback(ccall_app_rc_tg_callback);
                esp_avrc_rn_evt_cap_mask_t evt_set = {0};
                esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
                if(esp_avrc_tg_set_rn_evt_cap(&evt_set) != ESP_OK){
                    ESP_LOGE(AV_TAG,"esp_avrc_tg_set_rn_evt_cap failed");
                }
            } else {
                ESP_LOGE(AV_TAG,"esp_avrc_tg_init failed");
            }

#endif
            
            /* initialize A2DP sink */
            /*if (esp_a2d_register_callback(ccall_app_a2d_callback)!=ESP_OK){
                ESP_LOGE(AV_TAG,"esp_a2d_register_callback");
            }*/
            if (esp_a2d_sink_init()!=ESP_OK){
                ESP_LOGE(AV_TAG,"esp_a2d_sink_init");            
            }
            obj_g->autoreconnect_allowed = true;

            // start automatic reconnect if relevant and stack is up
            /*if (reconnect_status==AutoReconnect && has_last_connection() ) {
                ESP_LOGD(AV_TAG, "reconnect");
                memcpy(peer_bd_addr, last_connection, ESP_BD_ADDR_LEN);
                reconnect();
            }*/

            /* set discoverable and connectable mode, wait to be connected */
            ESP_LOGD(AV_TAG, "set_scan_mode_connectable(true)");
            a2dp_set_bt_connectable(true);
            break;
        }

        default:
            ESP_LOGE(AV_TAG, "%s unhandled evt %d", __func__, event);
            break;

    }
}

void bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
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
                memcpy(obj_g->peer_bd_addr, param->pin_req.bda, ESP_BD_ADDR_LEN);
                ESP_LOGI(AV_TAG, "partner address: %s", _bd_addr_to_str(obj_g->peer_bd_addr));
            }
            break;

        case ESP_BT_GAP_CFM_REQ_EVT: {
                memcpy(obj_g->peer_bd_addr, param->cfm_req.bda, ESP_BD_ADDR_LEN);
                ESP_LOGI(AV_TAG, "partner address: %s", _bd_addr_to_str(obj_g->peer_bd_addr));

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

