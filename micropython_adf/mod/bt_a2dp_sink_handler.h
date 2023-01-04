#ifndef _BT_A2DP_SINK_HANDLER
#define _BT_A2DP_SINK_HANDLER

typedef struct _bt_a2dp_obj_t bt_a2dp_obj_t;

bt_a2dp_obj_t *a2dp_init_bt(const char *device_name);

void a2dp_deinit_bt(bt_a2dp_obj_t *self);

void a2dp_set_bt_connectable(bool connectable);

bool a2dp_connect_to(bt_a2dp_obj_t *self, int *peer);

void a2dp_disconnect(bt_a2dp_obj_t *self);

int *a2dp_get_connected_addr(bt_a2dp_obj_t *self);

bool a2dp_is_connected(bt_a2dp_obj_t *self);

void a2dp_set_autoreconnect(bt_a2dp_obj_t *self, bool state);

#endif