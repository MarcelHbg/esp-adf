#ifndef _BT_A2DP_SINK_HANDLER
#define _BT_A2DP_SINK_HANDLER

#define A2DP_OBJ_DEVICENAME_MAX_LEN 15

typedef enum {
    A2DP_OBJ_STATE_IDLE = 0,
    A2DP_OBJ_STATE_INITED,
    A2DP_OBJ_STATE_STARTED,
    A2DP_OBJ_STATE_CONNECTED,
    A2DP_OBJ_STATE_NUM
}bt_a2dp_obj_state_t;

typedef enum {
    A2DP_OBJ_CONNECTION_DISCONNECTED = 0,   /*!< connection released */
    A2DP_OBJ_CONNECTION_CONNECTING,         /*!< connecting remote device */
    A2DP_OBJ_CONNECTION_CONNECTED,          /*!< connection established */
    A2DP_OBJ_CONNECTION_DISCONNECTING,      /*!< disconnecting remote device */
    A2DP_OBJ_CONNECTION_LOST,               /*!< connection lost */
    A2DP_OBJ_CONNECTION_NUM                 /*!< amount of connection states */
}bt_a2dp_obj_connect_state_t;

typedef esp_err_t (*a2dp_connection_hdl_t)(bt_a2dp_obj_connect_state_t state, void *context);
typedef esp_err_t (*a2dp_audio_hdl_t)(esp_a2d_audio_state_t state, void *context);

/**
 * @brief Object datatype for a2dp sink handler
 */
typedef struct _bt_a2dp_obj_t bt_a2dp_obj_t;

/**
 * @brief constructor of a2dp sink obj and init bluetooth
 * 
 * @param device_name set device name with max length of A2DP_OBJ_DEVICENAME_MAX_LEN
 * @return bt_a2dp_obj_t* a2dp sink obj
 */
bt_a2dp_obj_t *a2dp_init_bt(const char *device_name);

/**
 * @brief destructor of a2dp sink obj and deinit bluetooth
 * 
 * @param obj a2dp sink obj
 */
void a2dp_deinit_bt(bt_a2dp_obj_t *obj);

/**
 * @brief enable Bluetooth and start a2dp sink task
 * 
 * @param obj a2dp sink obj
 * @return esp_err_t 
 */
esp_err_t a2dp_sink_start(bt_a2dp_obj_t *obj);

/**
 * @brief disable Bluetooth and stop a2dp sink task
 * 
 * @param obj a2dp sink obj
 * @return esp_err_t 
 */
esp_err_t a2dp_sink_stop(bt_a2dp_obj_t *obj);

/**
 * @brief set bluetooth connectable and discoverable
 * 
 * @param obj a2dp sink obj
 * @param connectable set/unset
 * @return esp_err_t
 */
esp_err_t a2dp_set_bt_connectable(bt_a2dp_obj_t *obj, bool connectable);

/**
 * @brief connect to bt a2dp source
 * 
 * @param obj a2dp sink obj
 * @param source_bda bt address of source device
 * @return esp_err_t 
 */
esp_err_t a2dp_sink_connect_to_source(bt_a2dp_obj_t *obj, esp_bd_addr_t source_bda);

/**
 * @brief disconnect from bt a2dp source
 * 
 * @param obj a2dp sink obj
 * @return esp_err_t
 */
esp_err_t a2dp_sink_disconnect(bt_a2dp_obj_t *obj);

/**
 * @brief returns the connected bt address into remote_bda
 * 
 * @param obj a2dp sink obj
 * @param remote_bda connected bt address
 * @return esp_err_t 
 */
esp_err_t a2dp_get_connected_addr(bt_a2dp_obj_t *obj, esp_bd_addr_t *remote_bda);

/**
 * @brief check if a2dp sink is connected to source
 * 
 * @param obj a2dp sink obj
 * @return true 
 * @return false 
 */
bool a2dp_is_connected(bt_a2dp_obj_t *obj);

/**
 * @brief register callback function for external a2dp audio state handling
 * 
 * @param obj a2dp sink obj
 * @param handler 
 * @param context 
 * @return esp_err_t 
 */
esp_err_t a2dp_sink_register_audio_handler(bt_a2dp_obj_t *obj, a2dp_audio_hdl_t handler, void *context);

/**
 * @brief register callback function for external a2dp connection state handling
 * 
 * @param obj a2dp sink obj
 * @param handler 
 * @param context 
 * @return esp_err_t 
 */
esp_err_t a2dp_sink_register_connection_handler(bt_a2dp_obj_t *obj, a2dp_connection_hdl_t handler, void *context);

#endif