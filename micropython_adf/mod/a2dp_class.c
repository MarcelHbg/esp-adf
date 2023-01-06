#include <stdio.h>
#include <string.h>

#include "py/objstr.h"
#include "py/runtime.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt_defs.h"
#include "esp_a2dp_api.h"

#include "bt_a2dp_sink_handler.h"

static const char *TAG = "MPY_A2DPSink";

typedef struct _mp_a2dp_sink_obj_t {
    mp_obj_base_t base;
    mp_obj_t connection_callback;
    mp_obj_t audio_callback;
    bt_a2dp_obj_t *a2dp_sink;
} mp_a2dp_sink_obj_t;

STATIC esp_err_t a2dp_connection_hdl_cb(bt_a2dp_obj_connect_state_t state, void *context){
    mp_a2dp_sink_obj_t *self = (mp_a2dp_sink_obj_t *)context;
    if (self->connection_callback != mp_const_none) {
        mp_sched_schedule(self->connection_callback, mp_obj_new_int(state));
    }
    return ESP_OK;
}

STATIC esp_err_t a2dp_audio_hdl_cb(esp_a2d_audio_state_t state, void *context){
    mp_a2dp_sink_obj_t *self = (mp_a2dp_sink_obj_t *)context;
    if (self->audio_callback != mp_const_none) {
        mp_sched_schedule(self->audio_callback, mp_obj_new_int(state));
    }
    return ESP_OK;
}

static bt_a2dp_obj_t *a2dp_sink_obj = NULL;

STATIC mp_obj_t mp_a2dp_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args){
    // set log level for mp a2dp_class wrapper
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    const char *device_name = mp_obj_str_get_str(args[0]);

    // only init once if no obj exists
    if (!a2dp_sink_obj) a2dp_sink_obj = a2dp_init_bt(device_name);

    // mp multiple objects possible but they reference all the same bt_a2dp_obj_t
    mp_a2dp_sink_obj_t *self = m_new_obj_with_finaliser(mp_a2dp_sink_obj_t);
    self->base.type = type;
    self->connection_callback = mp_const_none;
    self->audio_callback = mp_const_none;
    self->a2dp_sink = a2dp_sink_obj;

    a2dp_sink_register_connection_handler(a2dp_sink_obj, a2dp_connection_hdl_cb, self);
    a2dp_sink_register_audio_handler(a2dp_sink_obj, a2dp_audio_hdl_cb, self);

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_a2dp_delete(mp_obj_t self_obj){
    mp_a2dp_sink_obj_t *self = self_obj;
    a2dp_deinit_bt(self->a2dp_sink);
    self->a2dp_sink = NULL;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_a2dp_delete_obj, mp_a2dp_delete);

STATIC mp_obj_t mp_a2dp_sink_start(mp_obj_t self_obj){
    mp_a2dp_sink_obj_t *self = self_obj;
    return mp_obj_new_int(a2dp_sink_start(self->a2dp_sink));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_a2dp_sink_start_obj, mp_a2dp_sink_start);

STATIC mp_obj_t mp_a2dp_sink_stop(mp_obj_t self_obj){
    mp_a2dp_sink_obj_t *self = self_obj;
    return mp_obj_new_int(a2dp_sink_stop(self->a2dp_sink));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_a2dp_sink_stop_obj, mp_a2dp_sink_stop);

STATIC mp_obj_t mp_a2dp_set_connectable(mp_obj_t self_obj, mp_obj_t connectable){
    mp_a2dp_sink_obj_t *self = self_obj;
    bool _connectable = connectable == mp_const_true ? true : false;
    return mp_obj_new_int(a2dp_set_bt_connectable(self->a2dp_sink, _connectable));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_a2dp_set_connectable_obj, mp_a2dp_set_connectable);

STATIC mp_obj_t mp_a2dp_connect_to(mp_obj_t self_obj, mp_obj_t peer){
    mp_a2dp_sink_obj_t *self = self_obj;

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

    return mp_obj_new_int(a2dp_sink_connect_to_source(self->a2dp_sink, _peer_addr));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_a2dp_connect_to_obj, mp_a2dp_connect_to);

STATIC mp_obj_t mp_a2dp_disconnect(mp_obj_t self_obj){
    mp_a2dp_sink_obj_t *self = self_obj;
    return mp_obj_new_int(a2dp_sink_disconnect(self->a2dp_sink));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_a2dp_disconnect_obj, mp_a2dp_disconnect);

STATIC mp_obj_t mp_a2dp_get_connected_addr(mp_obj_t self_obj){
    mp_a2dp_sink_obj_t *self = self_obj;

    esp_bd_addr_t bda = {0,0,0,0,0,0};
    esp_err_t err = a2dp_get_connected_addr(self->a2dp_sink, &bda);
    if (err != ESP_OK){
        return mp_obj_new_int(err);
    }

    mp_obj_t _bda_list[ESP_BD_ADDR_LEN];
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++){
        _bda_list[i] = mp_obj_new_int(bda[i]);
    }

    return mp_obj_new_list(ESP_BD_ADDR_LEN, _bda_list);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_a2dp_get_connected_addr_obj, mp_a2dp_get_connected_addr);

STATIC mp_obj_t mp_a2dp_is_connected(mp_obj_t self_obj){
    mp_a2dp_sink_obj_t *self = self_obj;

    if (a2dp_is_connected(self->a2dp_sink)) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_a2dp_is_connected_obj, mp_a2dp_is_connected);

STATIC mp_obj_t mp_a2dp_set_connection_cb(mp_obj_t self_obj, mp_obj_t callback){
    mp_a2dp_sink_obj_t *self = self_obj;
    self->connection_callback = callback;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_a2dp_set_connection_cb_obj, mp_a2dp_set_connection_cb);

STATIC mp_obj_t mp_a2dp_set_audio_cb(mp_obj_t self_obj, mp_obj_t callback){
    mp_a2dp_sink_obj_t *self = self_obj;
    self->audio_callback = callback;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_a2dp_set_audio_cb_obj, mp_a2dp_set_audio_cb);

STATIC const mp_rom_map_elem_t a2dp_sink_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&mp_a2dp_sink_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&mp_a2dp_sink_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_setConnectable), MP_ROM_PTR(&mp_a2dp_set_connectable_obj) },
    { MP_ROM_QSTR(MP_QSTR_connectToSource), MP_ROM_PTR(&mp_a2dp_connect_to_obj) },
    { MP_ROM_QSTR(MP_QSTR_disconnectSource), MP_ROM_PTR(&mp_a2dp_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_getRemoteAddr), MP_ROM_PTR(&mp_a2dp_get_connected_addr_obj) },
    { MP_ROM_QSTR(MP_QSTR_isConnected), MP_ROM_PTR(&mp_a2dp_is_connected_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&mp_a2dp_delete_obj) }, 
    { MP_ROM_QSTR(MP_QSTR_setConnectionStateCb), MP_ROM_PTR(&mp_a2dp_set_connection_cb_obj) }, 
    { MP_ROM_QSTR(MP_QSTR_setAudioStateCb), MP_ROM_PTR(&mp_a2dp_set_audio_cb_obj) }, 

    // connection states
    { MP_ROM_QSTR(MP_QSTR_CONNECTION_CONNECTED), MP_ROM_INT(A2DP_OBJ_CONNECTION_CONNECTED) },
    { MP_ROM_QSTR(MP_QSTR_CONNECTION_CONNECTING), MP_ROM_INT(A2DP_OBJ_CONNECTION_CONNECTING) }, 
    { MP_ROM_QSTR(MP_QSTR_CONNECTION_DISCONNECTED), MP_ROM_INT(A2DP_OBJ_CONNECTION_DISCONNECTED) }, 
    { MP_ROM_QSTR(MP_QSTR_CONNECTION_DISCONNECTING), MP_ROM_INT(A2DP_OBJ_CONNECTION_DISCONNECTING) }, 
    { MP_ROM_QSTR(MP_QSTR_CONNECTION_CONNECTION_LOST), MP_ROM_INT(A2DP_OBJ_CONNECTION_LOST) },

    // audio states
    { MP_ROM_QSTR(MP_QSTR_AUDIO_REMOTE_SUSPEND), MP_ROM_INT(ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_STARTED), MP_ROM_INT(ESP_A2D_AUDIO_STATE_STARTED) },
    { MP_ROM_QSTR(MP_QSTR_AUDIO_STOPPED), MP_ROM_INT(ESP_A2D_AUDIO_STATE_STOPPED) },
 
};

STATIC MP_DEFINE_CONST_DICT(a2dp_sink_locals_dict, a2dp_sink_locals_dict_table);

const mp_obj_type_t a2dp_sink_type = {
    { &mp_type_type },
    .name = MP_QSTR_A2DPSink,
    .make_new = mp_a2dp_make_new,
    .locals_dict = (mp_obj_dict_t*)&a2dp_sink_locals_dict,
};