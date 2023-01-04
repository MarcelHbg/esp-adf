#include <stdio.h>
#include <string.h>

#include "py/objstr.h"
#include "py/runtime.h"

#include "esp_log.h"

#include "bt_a2dp_sink_handler.h"

static const char *A2DP_CLASS_TAG = "A2DPSink";

typedef struct _mp_a2dp_sink_obj_t {
    mp_obj_base_t base;
    //mp_obj_t callback;

    bt_a2dp_obj_t *a2dp_sink;
} mp_a2dp_sink_obj_t;

STATIC mp_obj_t mp_a2dp_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args){
    esp_log_level_set(A2DP_CLASS_TAG, ESP_LOG_VERBOSE);

    mp_arg_check_num(n_args, n_kw, 1, 1, false);

    const char *device_name = mp_obj_str_get_str(args[0]);
    ESP_LOGI(A2DP_CLASS_TAG, "name: %s",device_name);

    static bt_a2dp_obj_t *basic_sink = NULL;

    mp_a2dp_sink_obj_t *self = m_new_obj_with_finaliser(mp_a2dp_sink_obj_t);
    self->base.type = type;
    if (basic_sink == NULL){
        ESP_LOGI(A2DP_CLASS_TAG, "call a2dp_handler init");
        basic_sink = a2dp_init_bt(device_name);
    }
    self->a2dp_sink = basic_sink;

    ESP_LOGD(A2DP_CLASS_TAG, "log debug test");
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t mp_a2dp_delete(mp_obj_t self_obj){
    mp_a2dp_sink_obj_t *self = self_obj;
    a2dp_deinit_bt(self->a2dp_sink);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_a2dp_delete_obj, mp_a2dp_delete);

STATIC mp_obj_t mp_a2dp_set_connectable(mp_obj_t self_obj, mp_obj_t connectable){
    mp_a2dp_sink_obj_t *self = self_obj;
    bool _connectable = connectable == mp_const_true ? true : false;
    a2dp_set_bt_connectable(_connectable);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_a2dp_set_connectable_obj, mp_a2dp_set_connectable);

STATIC mp_obj_t mp_a2dp_connect_to(mp_obj_t self_obj, mp_obj_t peer){
    mp_a2dp_sink_obj_t *self = self_obj;

    if (!mp_obj_is_type(peer, &mp_type_list)) return mp_const_false;

    size_t len;
    mp_obj_t *_obj_peer_addr;
    mp_obj_list_get(peer, &len, &_obj_peer_addr);

    if (len != 6) return mp_const_false;

    int _peer_addr[len];
    for (int i = 0; i < len; i++){
        if (!MP_OBJ_IS_SMALL_INT(_obj_peer_addr[i])) return mp_const_false;
        _peer_addr[i] = MP_OBJ_SMALL_INT_VALUE(_obj_peer_addr[i]);
    }

    if (a2dp_connect_to(self->a2dp_sink, _peer_addr)) return mp_const_true;

    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_a2dp_connect_to_obj, mp_a2dp_connect_to);

STATIC mp_obj_t mp_a2dp_disconnect(mp_obj_t self_obj){
    mp_a2dp_sink_obj_t *self = self_obj;

    a2dp_disconnect(self->a2dp_sink);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_a2dp_disconnect_obj, mp_a2dp_disconnect);

STATIC mp_obj_t mp_a2dp_get_connected_addr(mp_obj_t self_obj){
    mp_a2dp_sink_obj_t *self = self_obj;

    size_t len = 6;
    int *_peer_addr = NULL;
    _peer_addr = a2dp_get_connected_addr(self->a2dp_sink);

    mp_obj_t _obj_peer_addr[len];
    for (int i = 0; i < len; i++){
        _obj_peer_addr[i] = mp_obj_new_int(_peer_addr[i]);
    }

    return mp_obj_new_list(len, _obj_peer_addr);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_a2dp_get_connected_addr_obj, mp_a2dp_get_connected_addr);

STATIC mp_obj_t mp_a2dp_is_connected(mp_obj_t self_obj){
    mp_a2dp_sink_obj_t *self = self_obj;

    if (a2dp_is_connected(self->a2dp_sink)) return mp_const_true;
    return mp_const_false;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_a2dp_is_connected_obj, mp_a2dp_is_connected);

STATIC const mp_rom_map_elem_t a2dp_sink_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_setConnectable), MP_ROM_PTR(&mp_a2dp_set_connectable_obj) },
    { MP_ROM_QSTR(MP_QSTR_connectTo), MP_ROM_PTR(&mp_a2dp_connect_to_obj) },
    { MP_ROM_QSTR(MP_QSTR_disconnect), MP_ROM_PTR(&mp_a2dp_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_getPeerAddr), MP_ROM_PTR(&mp_a2dp_get_connected_addr_obj) },
    { MP_ROM_QSTR(MP_QSTR_isConnected), MP_ROM_PTR(&mp_a2dp_is_connected_obj) },
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&mp_a2dp_delete_obj) },
    
};

STATIC MP_DEFINE_CONST_DICT(a2dp_sink_locals_dict, a2dp_sink_locals_dict_table);

const mp_obj_type_t a2dp_sink_type = {
    { &mp_type_type },
    .name = MP_QSTR_A2DPSink,
    .make_new = mp_a2dp_make_new,
    .locals_dict = (mp_obj_dict_t*)&a2dp_sink_locals_dict,
};