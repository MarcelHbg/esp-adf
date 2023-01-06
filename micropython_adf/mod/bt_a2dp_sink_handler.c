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

static const char *AV_TAG = "a2dp_sink_adt";
static const char *APP_TAG = "a2dp_sink_app";

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

typedef enum {
    NoReconnect,
    AutoReconnect,
    IsReconnecting
}reconnect_status_t;

const char *m_a2d_conn_state_str[4] = {"Disconnected",
                                        "Connecting",
                                        "Connected",
                                        "Disconnecting"};

const char *m_a2d_audio_state_str[3] = {"Suspended",
                                        "Stopped",
                                        "Started"};

#define RECONNECT_TIMEOUT 5000 // in ms

struct _bt_a2dp_obj_t {
    char device_name[A2DP_OBJ_DEVICENAME_MAX_LEN + 1];
    bt_a2dp_obj_state_t state;
    a2dp_task_t app_task;
    a2dp_connection_hdl_t connection_hdl_cb;
    void * connection_hdl_ctx;
    a2dp_audio_hdl_t audio_hdl_cb;
    void * audio_hdl_ctx;
    esp_a2d_audio_state_t audio_state;
    esp_bd_addr_t peer_bd_addr;
    esp_bd_addr_t last_connection;
    esp_a2d_connection_state_t connection_state;
    bool is_connectable;
    bool autoreconnect_allowed;
    uint8_t retry_count;
    uint8_t retry_max_count;
    reconnect_status_t reconnect_status;
    unsigned long reconnect_timout;
};
#define DEFAULT_A2DP_OBJ_CONFIG() {\
    .device_name = "ESP_A2DP_SINK",\
    .state = A2DP_OBJ_STATE_IDLE,\
    .app_task = DEFAULT_A2DP_TASK_CONFIG(),\
    .connection_hdl_cb = NULL,\
    .connection_hdl_ctx = NULL,\
    .audio_hdl_cb = NULL,\
    .audio_hdl_ctx = NULL,\
    .audio_state = 0,\
    .peer_bd_addr = {0,0,0,0,0,0},\
    .last_connection = {0,0,0,0,0,0},\
    .connection_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED,\
    .is_connectable = false,\
    .autoreconnect_allowed = false,\
    .retry_count = 0,\
    .retry_max_count = 5,\
    .reconnect_status = AutoReconnect,\
    .reconnect_timout = 0,\
}

/*******************************************************************************************/
/* global obj obj pointer */

static bt_a2dp_obj_t *obj_g = NULL;

/*******************************************************************************************/
/* private function declarations*/

// task functions
void app_task_start_up();
void app_task_shut_down();
bool app_work_dispatch(app_callback_t p_cback, uint16_t event, void *p_params, int param_len);

// event handler
void hdl_stack_evt(uint16_t event, void *p_param);
void hdl_avrc_evt(uint16_t event, void *p_param);
void hdl_a2dp_evt(uint16_t event, void *p_param);

// bt callbacks
void bt_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void avrc_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
void a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

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

const char *_a2dp_con_state_to_str(esp_a2d_connection_state_t state) {
    return m_a2d_conn_state_str[state];
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
/* public bt_a2dp_sink ADT */

bt_a2dp_obj_t *a2dp_init_bt(const char *device_name){
    
    esp_log_level_set("*", ESP_LOG_DEBUG);
    esp_log_level_set(APP_TAG, ESP_LOG_DEBUG);
    esp_log_level_set(AV_TAG, ESP_LOG_DEBUG);

    bt_a2dp_obj_t *obj = malloc(sizeof(bt_a2dp_obj_t));
    if(!obj) {
        ESP_LOGE(AV_TAG,"[%s] malloc failed", __func__);
        return NULL;
    }

    ESP_LOGI(AV_TAG, "[%s] init default obj", __func__);
    bt_a2dp_obj_t a2dp_obj = DEFAULT_A2DP_OBJ_CONFIG();
    memcpy(obj, &a2dp_obj, sizeof(bt_a2dp_obj_t));

    if (device_name){
        strncpy(obj->device_name, device_name, A2DP_OBJ_DEVICENAME_MAX_LEN);
        obj->device_name[A2DP_OBJ_DEVICENAME_MAX_LEN] = '\0'; // enshure nulltermination
        ESP_LOGI(AV_TAG, "[%s] device name is set to: %s", __func__, obj->device_name);
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
            free(obj);
            return NULL;
        }
        while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE){}
    }
    ESP_LOGI(AV_TAG, "bt controller initialized");

    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED){
        ESP_LOGI(AV_TAG, "enable bt controller ...");
        err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
        if (err) {
            ESP_LOGE(AV_TAG, "[%s] bt controller enable err=%d", __func__, err);
            free(obj);
            return NULL;
        }
    }
    ESP_LOGI(AV_TAG, "bt controller enabled");

    obj->state = A2DP_OBJ_STATE_INITED;

    // save address global
    obj_g = obj;

    ESP_LOGI(AV_TAG, "a2dp sink obj initialized");
    return obj;
}

void a2dp_deinit_bt(bt_a2dp_obj_t *obj){
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return;
    }

    esp_err_t err = ESP_OK;

    if(obj->state >= A2DP_OBJ_STATE_STARTED){
        a2dp_sink_stop(obj);
    }

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

    free(obj);
    obj = NULL;
    obj_g = NULL;

    ESP_LOGI(AV_TAG, "a2dp sink obj deinitialized");
}

esp_err_t a2dp_sink_start(bt_a2dp_obj_t *obj){
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ESP_OK;

    if(obj->state >= A2DP_OBJ_STATE_STARTED){
        ESP_LOGI(AV_TAG, "[%s] allready started", __func__);
        return err;
    }

    if(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED){
        ESP_LOGI(AV_TAG, "init bluedroid ...");
        err = esp_bluedroid_init();
        if (err) {
            ESP_LOGE(AV_TAG,"[%s] esp_bluedroid_init err=%d", __func__, err);
            return err;
        }
    }
    ESP_LOGI(AV_TAG,"bluedroid initialized");

    if(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED){
        ESP_LOGI(AV_TAG, "enable bluedroid ...");
        err = esp_bluedroid_enable();
        if (err) {
            ESP_LOGE(AV_TAG, "[%s] bluedroid enable err=%d", __func__, err);
            return err;
        }
        while(esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED){}
    }
    ESP_LOGI(AV_TAG,"bluedroid enabled"); 

    err = esp_bt_gap_register_callback(bt_gap_callback);
    if (err) {
        ESP_LOGE(AV_TAG,"[%s] gap register callback err=%d", __func__, err);
        return false;
    }

    // create application task 
    ESP_LOGI(AV_TAG, "[%s] start app task", __func__);
    app_task_start_up(obj->app_task);

    // Bluetooth device name, connection mode and profile set up
    ESP_LOGI(AV_TAG, "[%s] dispatch app stackup event", __func__);
    app_work_dispatch(hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0);

    return err;
}

esp_err_t a2dp_sink_stop(bt_a2dp_obj_t *obj){
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ESP_OK;

    // state check not needed here allready done in a2dp_sink_disconnect
    err = a2dp_sink_disconnect(obj);
    if(err) return err;
    while(a2dp_is_connected(obj)){
        delay(100);
    }

    err = esp_a2d_sink_deinit();
    if(err){
        ESP_LOGE(AV_TAG, "[%s] esp_a2d_sink_deinit err=%d", __func__, err);
    }
    ESP_LOGI(AV_TAG, "A2DP deinitialized");
    _log_free_heap();

    err = esp_avrc_ct_deinit();
    if (err){
        ESP_LOGE(AV_TAG, "[%s] esp_avrc_ct_deinit err=%d", __func__, err);
    }
    ESP_LOGI(AV_TAG, "AVRC deinitialized");
    _log_free_heap();

    err = esp_bluedroid_disable();
    if (err){
        ESP_LOGE(AV_TAG, "[%s] esp_bluedroid_disable err=%d", __func__, err);
    }
    ESP_LOGI(AV_TAG, "bluedroid disabled");
    _log_free_heap();

    err = esp_bluedroid_deinit();
    if (err){
        ESP_LOGE(AV_TAG, "[%s] esp_bluedroid_deinit err=%d", __func__, err);
    }
    ESP_LOGI(AV_TAG, "bluedroid deinitialized");
    _log_free_heap();

    // shut down app task
    app_task_shut_down(&obj_g->app_task);
    
    obj->state = A2DP_OBJ_STATE_INITED;

    return err;
}

esp_err_t a2dp_set_bt_connectable(bt_a2dp_obj_t *obj, bool connectable) {
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    if(obj->state == A2DP_OBJ_STATE_INITED){
        ESP_LOGE(AV_TAG, "[%s] not started", __func__);
        return ESP_ERR_INVALID_STATE;
    }
    if(obj->state == A2DP_OBJ_STATE_CONNECTED){
        ESP_LOGE(AV_TAG, "[%s] allready connected", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(AV_TAG, "[%s] set to: %s",__func__, _bool_to_str(connectable));    
    esp_bt_scan_mode_t mode = connectable ? ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE : ESP_BT_SCAN_MODE_NONE;
    esp_err_t err = esp_bt_gap_set_scan_mode(mode);

    if (err != ESP_OK) {
        ESP_LOGE(AV_TAG,"esp_bt_gap_set_scan_mode err=%d", err);
        obj->is_connectable = false;
    }
    else {
        obj->is_connectable = true;
    }
    
    return err;
}

esp_err_t a2dp_sink_connect_to_source(bt_a2dp_obj_t *obj, esp_bd_addr_t source_bda){
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    // state check not needed here allready done in a2dp_set_bt_connectable
    esp_err_t err = a2dp_set_bt_connectable(obj, true);
    if (err) return err;
    delay(100);

    ESP_LOGI(AV_TAG, "[%s] %s",__func__, _bd_addr_to_str(source_bda));
    err = esp_a2d_sink_connect(source_bda);
    if (err){
        ESP_LOGE(AV_TAG, "esp_a2d_source_connect err=%d", err);
        return err;
    }

    return ESP_OK;
}

esp_err_t a2dp_sink_disconnect(bt_a2dp_obj_t *obj){
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    if(obj->state == A2DP_OBJ_STATE_INITED){
        ESP_LOGE(AV_TAG, "[%s] not started", __func__);
        return ESP_ERR_INVALID_STATE;
    }
    // disconnect possible when not in connected state

    ESP_LOGI(AV_TAG, "disconnect a2dp: %s", _bd_addr_to_str(obj->last_connection));

    // Prevent automatic reconnect
    //obj->autoreconnect_allowed = false;

    esp_err_t err = esp_a2d_sink_disconnect(obj->last_connection);
    if (err) {
        ESP_LOGE(AV_TAG, "[%s] disconnecting err=%d",__func__, err);
    }
    return err;
}

esp_err_t a2dp_get_connected_addr(bt_a2dp_obj_t *obj, esp_bd_addr_t *remote_bda){
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    // in every state possible default return = {0,0,0,0,0,0}

    ESP_LOGI(AV_TAG, "[%s] bda: %s -> [%d,%d,%d,%d,%d,%d]",
                    __func__,
                    _bd_addr_to_str(obj->peer_bd_addr),
                    ESP_BD_ADDR_HEX(obj->peer_bd_addr));

    if (!remote_bda) {
        ESP_LOGE(AV_TAG, "[%s] remote_bda is NULL", __func__);
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(remote_bda, obj->peer_bd_addr, ESP_BD_ADDR_LEN);
    return ESP_OK;
}

bool a2dp_is_connected(bt_a2dp_obj_t *obj){
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return false;
    }

    // in every state possible default connection_state = ESP_A2D_CONNECTION_STATE_DISCONNECTED
    return obj->connection_state == ESP_A2D_CONNECTION_STATE_CONNECTED;
}

esp_err_t a2dp_sink_register_connection_handler(bt_a2dp_obj_t *obj, a2dp_connection_hdl_t handler, void *context){
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    if(!handler){
        ESP_LOGW(AV_TAG, "[%s] handler is NULL", __func__);
    }

    if(obj->connection_hdl_cb){
        ESP_LOGE(AV_TAG, "[%s] connection_hdl_cb overwritten", __func__);
    }
    obj->connection_hdl_cb = handler;
    obj->connection_hdl_ctx = context;

    return ESP_OK;
}

esp_err_t a2dp_sink_register_audio_handler(bt_a2dp_obj_t *obj, a2dp_audio_hdl_t handler, void *context){
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return ESP_ERR_INVALID_STATE;
    }

    if(!handler){
        ESP_LOGW(AV_TAG, "[%s] handler is NULL", __func__);
    }

    if(obj->audio_hdl_cb){
        ESP_LOGE(AV_TAG, "[%s] audio_hdl_cb overwritten", __func__);
    }
    obj->audio_hdl_cb = handler;
    obj->audio_hdl_ctx = context;

    return ESP_OK;
}

// not defined in API for now
void a2dp_set_autoreconnect(bt_a2dp_obj_t *obj, bool state){
    if(!obj) { // determines in IDLE state (not initalized)
        ESP_LOGE(AV_TAG, "[%s] bt_a2dp_obj_t is NULL", __func__);
        return;
    }
    obj->reconnect_status = state ? AutoReconnect : NoReconnect;
    ESP_LOGI(AV_TAG, "set_autoreconnect: %s", _bool_to_str(state));
}

/*******************************************************************************************/
/* app task functions */

bool app_send_msg(app_msg_t *msg)
{
    ESP_LOGI(AV_TAG, "%s", __func__);
    if (msg == NULL || obj_g->app_task.app_task_queue == NULL) {
        ESP_LOGE(APP_TAG, "%s app_send_msg failed", __func__);
        return false;
    }

    if (xQueueSend(obj_g->app_task.app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        ESP_LOGE(APP_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}

bool app_work_dispatch(app_callback_t p_cback, uint16_t event, void *p_params, int param_len)
{
    ESP_LOGI(APP_TAG, "%s event 0x%x, param len %d", __func__, event, param_len);
    
    app_msg_t msg;
    memset(&msg, 0, sizeof(app_msg_t));

    msg.sig = APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            return app_send_msg(&msg);
        }
    }

    return false;
}

void app_work_dispatched(app_msg_t *msg)
{
    ESP_LOGI(AV_TAG, "%s", __func__);
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

void app_task_handler(void *arg){
    ESP_LOGI(AV_TAG, "%s", __func__);
    
    app_msg_t msg;
    while (true) {
        if (!obj_g->app_task.app_task_queue){
            ESP_LOGE(APP_TAG, "%s, app_task_queue is null", __func__);
            delay(100);
        } else if (pdTRUE == xQueueReceive(obj_g->app_task.app_task_queue, &msg, (portTickType)portMAX_DELAY)) {
            ESP_LOGI(APP_TAG, "%s, sig 0x%x, 0x%x", __func__, msg.sig, msg.event);
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

void app_task_start_up(){
    ESP_LOGI(AV_TAG, "%s", __func__);
    if (obj_g->app_task.app_task_queue ==NULL) 
        obj_g->app_task.app_task_queue = xQueueCreate(obj_g->app_task.event_queue_size, sizeof(app_msg_t));

    if (obj_g->app_task.app_task_handle==NULL) {
        if (xTaskCreatePinnedToCore(app_task_handler,
                                    "BtAppT",
                                    obj_g->app_task.event_stack_size,
                                    NULL,
                                    obj_g->app_task.task_priority,
                                    &obj_g->app_task.app_task_handle,
                                    obj_g->app_task.task_core) != pdPASS){
            ESP_LOGE(AV_TAG, "%s failed", __func__);
        }
    }
}

void app_task_shut_down(){
    ESP_LOGI(AV_TAG, "%s", __func__);
    if (obj_g->app_task.app_task_handle!=NULL) {
        vTaskDelete(obj_g->app_task.app_task_handle);
        obj_g->app_task.app_task_handle = NULL;
    }
    if (obj_g->app_task.app_task_queue!=NULL) {
        vQueueDelete(obj_g->app_task.app_task_queue);
        obj_g->app_task.app_task_queue = NULL;
    }
}

/*******************************************************************************************/
/* (re)connection helper funktions*/

bool has_last_connection() {  
    esp_bd_addr_t empty_connection = {0,0,0,0,0,0};
    int result = memcmp(obj_g->last_connection, empty_connection, ESP_BD_ADDR_LEN);
    return result!=0;
}

void get_last_connection() {
    return;
}

void set_last_connection(esp_bd_addr_t bda) {
     ESP_LOGI(AV_TAG, "set_last_connection: %s", _bd_addr_to_str(bda));

    //same value, nothing to store
    if ( memcmp(bda, obj_g->last_connection, ESP_BD_ADDR_LEN) == 0 ) {
        ESP_LOGI(AV_TAG, "no change!");
        return; 
    }

    memcpy(obj_g->last_connection, bda, ESP_BD_ADDR_LEN);
}

void clean_last_connection() {
    ESP_LOGI(AV_TAG, "%s", __func__);
    esp_bd_addr_t cleanBda = { 0 };
    set_last_connection(cleanBda);
}

bool reconnect() {
    obj_g->autoreconnect_allowed = true;
    obj_g->reconnect_status = IsReconnecting;
    obj_g->reconnect_timout = millis() + RECONNECT_TIMEOUT;
    return a2dp_sink_connect_to_source(obj_g, obj_g->peer_bd_addr);
}

 bool is_reconnect(esp_a2d_disc_rsn_t type) {
    bool result = obj_g->autoreconnect_allowed && (obj_g->reconnect_status==AutoReconnect || obj_g->reconnect_status==IsReconnecting) && has_last_connection();
    ESP_LOGI(AV_TAG,"is_reconnect: %s", _bool_to_str(result));
    return result;
}

/*******************************************************************************************/
/* event handler helper funktions*/

bt_a2dp_obj_connect_state_t connect_state_convert(esp_a2d_connection_state_t state, esp_a2d_disc_rsn_t disc_rsn){
    switch (state){
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
            if(disc_rsn == ESP_A2D_DISC_RSN_NORMAL) return A2DP_OBJ_CONNECTION_DISCONNECTED;
            else return A2DP_OBJ_CONNECTION_LOST;
        default:
            return state;
    }
}

void handle_connection_state(uint16_t event, void *p_param){
    ESP_LOGI(AV_TAG, "%s evt %d", __func__, event);
    esp_a2d_cb_param_t* a2d = (esp_a2d_cb_param_t *)(p_param);

    // determine remote BDA
    memcpy(obj_g->peer_bd_addr, a2d->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
    ESP_LOGI(AV_TAG, "partner address: %s", _bd_addr_to_str(obj_g->peer_bd_addr));

    obj_g->connection_state = a2d->conn_stat.state;
    // callback
    //if (connection_state_callback!=nullptr){
    //    connection_state_callback(connection_state, connection_state_obj);
    //}

    ESP_LOGI(AV_TAG, "A2DP connection state: %s, [%s]", _a2dp_con_state_to_str(a2d->conn_stat.state), _bd_addr_to_str(a2d->conn_stat.remote_bda));
    switch (a2d->conn_stat.state) {

        case ESP_A2D_CONNECTION_STATE_DISCONNECTING:
            ESP_LOGI(AV_TAG, "ESP_A2D_CONNECTION_STATE_DISCONNECTING");
            if (a2d->conn_stat.disc_rsn==ESP_A2D_DISC_RSN_NORMAL){
                obj_g->autoreconnect_allowed = false;
            }
            break;

        case ESP_A2D_CONNECTION_STATE_DISCONNECTED:
            ESP_LOGI(AV_TAG, "ESP_A2D_CONNECTION_STATE_DISCONNECTED");
            obj_g->state = A2DP_OBJ_STATE_STARTED;
            if (a2d->conn_stat.disc_rsn==ESP_A2D_DISC_RSN_NORMAL){
                obj_g->autoreconnect_allowed = false;
            }

            // reset pin code
            //pin_code_int = 0;
            //pin_code_request = Undefined;
            
            // RECONNECTION MGMT
            // do not auto reconnect when disconnect was requested from device
            if (obj_g->autoreconnect_allowed) {
                if (is_reconnect(a2d->conn_stat.disc_rsn)) {
                    if (obj_g->retry_count < obj_g->retry_max_count ){
                        ESP_LOGI(AV_TAG,"Connection try number: %d", obj_g->retry_count);
                        // make sure that any open connection is timing out on the target
                        memcpy(obj_g->peer_bd_addr, obj_g->last_connection, ESP_BD_ADDR_LEN);
                        reconnect();
                        // when we lost the connection we do allow any others to connect after 2 trials
                        if (obj_g->retry_count==2) a2dp_set_bt_connectable(obj_g, true);

                    } else {
                        ESP_LOGI(AV_TAG, "Reconect retry limit reached");
                        if ( has_last_connection() && a2d->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_NORMAL ){
                            clean_last_connection();
                        }
                        a2dp_set_bt_connectable(obj_g, true);
                    }
                } else {
                    a2dp_set_bt_connectable(obj_g, true);   
                }
            } else {
                a2dp_set_bt_connectable(obj_g, true);   
            }
            break;

        case ESP_A2D_CONNECTION_STATE_CONNECTING:
            ESP_LOGI(AV_TAG, "ESP_A2D_CONNECTION_STATE_CONNECTING");
            obj_g->retry_count++;
            break;

        case ESP_A2D_CONNECTION_STATE_CONNECTED:
            ESP_LOGI(AV_TAG, "ESP_A2D_CONNECTION_STATE_CONNECTED");
            obj_g->state = A2DP_OBJ_STATE_CONNECTED;
            // stop reconnect retries in event loop
            if (obj_g->reconnect_status==IsReconnecting){
                obj_g->reconnect_status = AutoReconnect;
            }

            // checks if the address is valid
            bool is_valid = true;
            /*if(address_validator!=nullptr){
                uint8_t *bda = a2d->conn_stat.remote_bda;
                if (!address_validator(bda)){
                    ESP_LOGI(AV_TAG,"esp_a2d_sink_disconnect: %s", (char*)bda );
                    esp_a2d_sink_disconnect(bda);
                    is_valid = false;
                }
            }*/

            if (is_valid){
                a2dp_set_bt_connectable(obj_g, false);   
                obj_g->retry_count = 0;

                // record current connection
                if (obj_g->reconnect_status==AutoReconnect && is_valid) {
                    set_last_connection(a2d->conn_stat.remote_bda);
                }
#ifdef ESP_IDF_4
                // ask for the remote name
                esp_err_t esp_err = esp_bt_gap_read_remote_name(a2d->conn_stat.remote_bda);
                if (esp_err!=ESP_OK){
                    ESP_LOGE(AV_TAG,"esp_bt_gap_read_remote_name");
                }
#endif     
            }

            break;

    } 
}

/*******************************************************************************************/
/* event handler */

void hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGI(AV_TAG, "%s evt %d", __func__, event);
    esp_err_t result;

    switch (event) {
        case BT_APP_EVT_STACK_UP: {
            ESP_LOGI(AV_TAG, "%s hdl_stack_evt %s", __func__, "BT_APP_EVT_STACK_UP");

            /* set up device name */
            ESP_LOGI(AV_TAG,"esp_bt_dev_set_device_name: %s", obj_g->device_name);
            esp_bt_dev_set_device_name(obj_g->device_name);  

            // initialize AVRCP controller 
            result = esp_avrc_ct_init();
            if (result == ESP_OK){
                result = esp_avrc_ct_register_callback(avrc_callback);
                if (result == ESP_OK){
                    ESP_LOGI(AV_TAG, "AVRCP controller initialized!");
                } else {
                    ESP_LOGE(AV_TAG,"esp_avrc_ct_register_callback: %d",result);
                }
            } else {
                ESP_LOGE(AV_TAG,"esp_avrc_ct_init: %d",result);
            }
            
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
            if (esp_a2d_register_callback(a2dp_callback)!=ESP_OK){
                ESP_LOGE(AV_TAG,"esp_a2d_register_callback");
            }
            if (esp_a2d_sink_init()!=ESP_OK){
                ESP_LOGE(AV_TAG,"esp_a2d_sink_init");            
            }

            // start automatic reconnect if relevant and stack is up
            if (obj_g->reconnect_status==AutoReconnect && has_last_connection() ) {
                ESP_LOGI(AV_TAG, "reconnect");
                memcpy(obj_g->peer_bd_addr, obj_g->last_connection, ESP_BD_ADDR_LEN);
                reconnect();
            }

            /* set discoverable and connectable mode, wait to be connected */
            ESP_LOGI(AV_TAG, "a2dp_set_bt_connectable(true)");
            a2dp_set_bt_connectable(obj_g, true);
            break;

            obj_g->state = A2DP_OBJ_STATE_STARTED;
        }

        default:
            ESP_LOGE(AV_TAG, "%s unhandled evt %d", __func__, event);
            break;

    }
}

void hdl_avrc_evt(uint16_t event, void *p_param) {
    ESP_LOGI(AV_TAG, "%s evt %d", __func__, event);
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        ESP_LOGI(AV_TAG, "AVRC conn_state evt: state %d, [%s]", rc->conn_stat.connected, _bd_addr_to_str(rc->conn_stat.remote_bda));

#ifdef ESP_IDF_4
        if (rc->conn_stat.connected) {
            av_new_track();
             // get remote supported event_ids of peer AVRCP Target
            esp_avrc_ct_send_get_rn_capabilities_cmd(APP_RC_CT_TL_GET_CAPS);
        } else {
            // clear peer notification capability record
            s_avrc_peer_rn_cap.bits = 0;
        }        
#else
        if (rc->conn_stat.connected) {
            //av_new_track();
        }
#endif
        break;

    }
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        ESP_LOGI(AV_TAG, "AVRC passthrough rsp: key_code 0x%x, key_state %d", rc->psth_rsp.key_code, rc->psth_rsp.key_state);
        break;
    }
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        ESP_LOGI(AV_TAG, "AVRC metadata rsp: attribute id 0x%x, %s", rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
        // call metadata callback if available
        //if (avrc_metadata_callback != nullptr){
        //    avrc_metadata_callback(rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
        //}

        free(rc->meta_rsp.attr_text);
        break;
    }
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
        ESP_LOGI(AV_TAG, "AVRC event notification: %d, param: %d", (int)rc->change_ntf.event_id, (int)rc->change_ntf.event_parameter);
        //av_notify_evt_handler(rc->change_ntf.event_id, rc->change_ntf.event_parameter);
        break;
    }
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        ESP_LOGI(AV_TAG, "AVRC remote features %x", rc->rmt_feats.feat_mask);
        break;
    }

#ifdef ESP_IDF_4

    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        ESP_LOGI(AV_TAG, "remote rn_cap: count %d, bitmask 0x%x", rc->get_rn_caps_rsp.cap_count,
                 rc->get_rn_caps_rsp.evt_set.bits);
        s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;
        av_new_track();
        //bt_av_playback_changed();
        //bt_av_play_pos_changed();
        break;
    }

#endif

    default:
        ESP_LOGE(AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}

void hdl_a2dp_evt(uint16_t event, void *p_param) {
    ESP_LOGI(AV_TAG, "%s evt %d", __func__, event);
    esp_a2d_cb_param_t* param = (esp_a2d_cb_param_t *)(p_param);
    esp_err_t err = ESP_OK;
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT: {
            ESP_LOGI(AV_TAG, "%s ESP_A2D_CONNECTION_STATE_EVT", __func__);
            handle_connection_state(event, p_param); // internal handler
            if(obj_g->connection_hdl_cb){ // external handler
                bt_a2dp_obj_connect_state_t conn_state = connect_state_convert(param->conn_stat.state, param->conn_stat.disc_rsn);
                err = obj_g->connection_hdl_cb(conn_state, obj_g->connection_hdl_ctx);
                if(err){
                    ESP_LOGE(AV_TAG, "[%s] connection_hdl_cb state=%d err=%d",__func__, conn_state, err);
                }
            }
        } break;

        case ESP_A2D_AUDIO_STATE_EVT: {
            ESP_LOGI(AV_TAG, "%s ESP_A2D_AUDIO_STATE_EVT", __func__);
            //handle_audio_state(event, p_param); // internal handler
            if(obj_g->audio_hdl_cb){ // external handler
                err = obj_g->audio_hdl_cb(param->audio_stat.state, obj_g->audio_hdl_ctx);
                if(err){
                    ESP_LOGE(AV_TAG, "[%s] connection_hdl_cb err=%d",__func__, err);
                }
            }
        } break;
        case ESP_A2D_AUDIO_CFG_EVT: {
            ESP_LOGI(AV_TAG, "%s ESP_A2D_AUDIO_CFG_EVT", __func__);
            //handle_audio_cfg(event, p_param);
            
        } break;

#ifdef ESP_IDF_4

        case ESP_A2D_PROF_STATE_EVT: {
            esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(p_param);
            if (ESP_A2D_INIT_SUCCESS == a2d->a2d_prof_stat.init_state) {
                ESP_LOGI(AV_TAG,"A2DP PROF STATE: Init Compl\n");
            } else {
                ESP_LOGI(AV_TAG,"A2DP PROF STATE: Deinit Compl\n");
            }
        } break;

#endif

        default:
            ESP_LOGE(AV_TAG, "%s unhandled evt %d", __func__, event);
            break;
    }
}

/*******************************************************************************************/
/* bluetooth callbacks */

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

void avrc_callback(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    ESP_LOGI(AV_TAG, "%s", __func__);

    switch (event) {
        case ESP_AVRC_CT_METADATA_RSP_EVT:
            ESP_LOGI(AV_TAG, "%s ESP_AVRC_CT_METADATA_RSP_EVT", __func__);
            //app_alloc_meta_buffer(param);
            //app_work_dispatch(hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        case ESP_AVRC_CT_CONNECTION_STATE_EVT:
            ESP_LOGI(AV_TAG, "%s ESP_AVRC_CT_CONNECTION_STATE_EVT", __func__);
            app_work_dispatch(hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
            ESP_LOGI(AV_TAG, "%s ESP_AVRC_CT_PASSTHROUGH_RSP_EVT", __func__);
            app_work_dispatch(hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
            ESP_LOGI(AV_TAG, "%s ESP_AVRC_CT_CHANGE_NOTIFY_EVT", __func__);
            app_work_dispatch(hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
            ESP_LOGI(AV_TAG, "%s ESP_AVRC_CT_REMOTE_FEATURES_EVT", __func__);
            app_work_dispatch(hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        }

#ifdef ESP_IDF_4

        case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
            ESP_LOGI(AV_TAG, "%s ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT", __func__);
            app_work_dispatch(hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        }
        
#endif

        default:
            ESP_LOGE(AV_TAG, "Invalid AVRC event: %d", event);
            break;
        }
}

void a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    ESP_LOGI(AV_TAG, "%s", __func__);

    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGI(AV_TAG, "%s ESP_A2D_CONNECTION_STATE_EVT", __func__);
        app_work_dispatch(hdl_a2dp_evt, event, param, sizeof(esp_a2d_cb_param_t));
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(AV_TAG, "%s ESP_A2D_AUDIO_STATE_EVT", __func__);
        obj_g->audio_state = param->audio_stat.state;
        app_work_dispatch(hdl_a2dp_evt,event, param, sizeof(esp_a2d_cb_param_t));
        break;
    case ESP_A2D_AUDIO_CFG_EVT: {
        ESP_LOGI(AV_TAG, "%s ESP_A2D_AUDIO_CFG_EVT", __func__);
        app_work_dispatch(hdl_a2dp_evt, event, param, sizeof(esp_a2d_cb_param_t));
        break;
    }
    
#ifdef ESP_IDF_4
    case ESP_A2D_PROF_STATE_EVT: {
        ESP_LOGI(AV_TAG, "%s ESP_A2D_AUDIO_CFG_EVT", __func__);
        app_work_dispatch(hdl_a2dp_evt, event, param, sizeof(esp_a2d_cb_param_t));
        break;
    }
#endif    
    
    default:
        ESP_LOGE(AV_TAG, "Invalid A2DP event: %d", event);
        break;
    }
}
