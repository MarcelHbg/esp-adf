#ifndef _BT_AUDIO_HANDLER
#define _BT_AUDIO_HANDLER

#include "esp_err.h" // esp_err_t
#include "audio_element.h" // audio_element_handle_t
#include "esp_a2dp_api.h" // esp_a2d_audio_state_t
#include "bt_keycontrol.h" // periph_bluetooth_event_id_t

#define BT_AUDIO_DEVICENAME_MAX_LEN 15

typedef enum {
    BT_AUDIO_STATE_IDLE = 0,
    BT_AUDIO_STATE_INITED,
    BT_AUDIO_STATE_STARTED,
    BT_AUDIO_STATE_CONNECTED,
    BT_AUDIO_STATE_NUM
}bt_audio_state_t;

typedef esp_err_t (*bt_audio_periph_hdl_t)(periph_bluetooth_event_id_t evt, void *context);

/**
 * @brief init bluetooth
 * 
 * @param device_name set device name with max length of BT_AUDIO_DEVICENAME_MAX_LEN
 * @return esp_err_t
 */
esp_err_t bt_audio_init(const char *device_name);

/**
 * @brief deinit bluetooth
 */
void bt_audio_deinit();

/**
 * @brief enable Bluetooth and start a2dp
 * 
 * @param sink if true a2dp sink will be started else a2dp source
 * @return esp_err_t 
 */
esp_err_t bt_audio_start(bool sink);

/**
 * @brief disable Bluetooth and stop a2dp
 * 
 * @return esp_err_t 
 */
esp_err_t bt_audio_stop();

/**
 * @brief set bluetooth connectable and discoverable
 * 
 * @param connectable set/unset
 * @return esp_err_t
 */
esp_err_t bt_audio_set_connectable(bool connectable);

/**
 * @brief connect to bt a2dp sink or source
 * 
 * @param source_bda bt address of remote device
 * @return esp_err_t 
 */
esp_err_t bt_audio_connect_to(esp_bd_addr_t bda);

/**
 * @brief disconnect from bt a2dp sink or source
 * 
 * @return esp_err_t
 */
esp_err_t bt_audio_disconnect();

/**
 * @brief returns the remote bt address into remote_bda
 * 
 * @param remote_bda remote bt address
 * @return esp_err_t 
 */
esp_err_t bt_audio_get_remote_addr(esp_bd_addr_t *remote_bda);

/**
 * @brief check if bt is connectable
 * 
 * @return true 
 * @return false 
 */
bool bt_audio_is_connectable();

/**
 * @brief set function wich handles bluetooth periph events
 * 
 * @param handler 
 * @param context 
 * @return esp_err_t 
 */
esp_err_t bt_audio_set_event_handler(bt_audio_periph_hdl_t handler, void *context);

/**
 * @brief get audio stream for audio player
 * 
 * @return audio_element_t 
 */
audio_element_handle_t bt_audio_get_stream();

#endif