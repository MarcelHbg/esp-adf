#ifndef _BT_A2DP_SINK_HANDLER
#define _BT_A2DP_SINK_HANDLER

#define A2DP_OBJ_DEVICENAME_MAX_LEN 15

typedef enum {
    A2DP_OBJ_IDLE,
    A2DP_OBJ_INITED,
    A2DP_OBJ_STARTED,
    A2DP_OBJ_CONNECTED,
    A2DP_OBJ_NUM
}bt_a2dp_obj_state_t;

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

#endif