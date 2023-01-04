#ifndef _BT_A2DP_SINK_HANDLER
#define _BT_A2DP_SINK_HANDLER

typedef struct _bt_a2dp_obj_t bt_a2dp_obj_t;

bt_a2dp_obj_t *a2dp_init_bt(const char *device_name);

void a2dp_deinit_bt(bt_a2dp_obj_t *obj);

void a2dp_set_bt_connectable(bt_a2dp_obj_t *obj, bool connectable);

bool a2dp_connect_to(bt_a2dp_obj_t *obj, int *peer);

void a2dp_disconnect(bt_a2dp_obj_t *obj);

int *a2dp_get_connected_addr(bt_a2dp_obj_t *obj);

bool a2dp_is_connected(bt_a2dp_obj_t *obj);

void a2dp_set_autoreconnect(bt_a2dp_obj_t *obj, bool state);

#endif