/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/settings/settings.h>
#include <zephyr/init.h>

#include <limits.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

#include <zmk/ble.h>
#include <zmk/endpoints_types.h>
#include <zmk/hog.h>
#include <zmk/hid.h>
#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
#include <zmk/pointing/resolution_multipliers.h>
#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
#include <zmk/hid_indicators.h>
#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

enum {
    HIDS_REMOTE_WAKE = BIT(0),
    HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
    uint16_t version; /* version number of base USB HID Specification */
    uint8_t code;     /* country HID Device hardware is localized for. */
    uint8_t flags;
} __packed;

struct hids_report {
    uint8_t id;   /* report id */
    uint8_t type; /* report type */
} __packed;

static struct hids_info info = {
    .version = 0x0000,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE | HIDS_REMOTE_WAKE,
};

enum {
    HIDS_INPUT = 0x01,
    HIDS_OUTPUT = 0x02,
    HIDS_FEATURE = 0x03,
};

static struct hids_report input = {
    .id = ZMK_HID_REPORT_ID_KEYBOARD,
    .type = HIDS_INPUT,
};

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

static struct hids_report led_indicators = {
    .id = ZMK_HID_REPORT_ID_LEDS,
    .type = HIDS_OUTPUT,
};

#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

static struct hids_report consumer_input = {
    .id = ZMK_HID_REPORT_ID_CONSUMER,
    .type = HIDS_INPUT,
};

#if IS_ENABLED(CONFIG_ZMK_POINTING)

static struct hids_report mouse_input = {
    .id = ZMK_HID_REPORT_ID_MOUSE,
    .type = HIDS_INPUT,
};

#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

static struct hids_report mouse_feature = {
    .id = ZMK_HID_REPORT_ID_MOUSE,
    .type = HIDS_FEATURE,
};

#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

static bool host_requests_notification = false;
static uint8_t ctrl_point;
// static uint8_t proto_mode;

static ssize_t read_hids_info(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct hids_info));
}

static ssize_t read_hids_report_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data,
                             sizeof(struct hids_report));
}

static ssize_t read_hids_report_map(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, zmk_hid_report_desc,
                             sizeof(zmk_hid_report_desc));
}

static ssize_t read_hids_input_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      void *buf, uint16_t len, uint16_t offset) {
    struct zmk_hid_keyboard_report_body *report_body = &zmk_hid_get_keyboard_report()->body;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, report_body,
                             sizeof(struct zmk_hid_keyboard_report_body));
}

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
static ssize_t write_hids_leds_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset,
                                      uint8_t flags) {
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != sizeof(struct zmk_hid_led_report_body)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    struct zmk_hid_led_report_body *report = (struct zmk_hid_led_report_body *)buf;
    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct zmk_endpoint_instance endpoint = {.transport = ZMK_TRANSPORT_BLE,
                                             .ble = {
                                                 .profile_index = profile,
                                             }};
    zmk_hid_indicators_process_report(report, endpoint);

    return len;
}

#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

static ssize_t read_hids_consumer_input_report(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr, void *buf,
                                               uint16_t len, uint16_t offset) {
    struct zmk_hid_consumer_report_body *report_body = &zmk_hid_get_consumer_report()->body;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, report_body,
                             sizeof(struct zmk_hid_consumer_report_body));
}

#if IS_ENABLED(CONFIG_ZMK_POINTING)

static ssize_t read_hids_mouse_input_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                            void *buf, uint16_t len, uint16_t offset) {
    struct zmk_hid_mouse_report_body *report_body = &zmk_hid_get_mouse_report()->body;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, report_body,
                             sizeof(struct zmk_hid_mouse_report_body));
}

#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

static ssize_t read_hids_mouse_feature_report(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                              void *buf, uint16_t len, uint16_t offset) {

    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        LOG_DBG("   BT_ATT_ERR_UNLIKELY");
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct zmk_endpoint_instance endpoint = {
        .transport = ZMK_TRANSPORT_BLE,
        .ble = {.profile_index = profile},
    };

    struct zmk_pointing_resolution_multipliers mult =
        zmk_pointing_resolution_multipliers_get_profile(endpoint);

    struct zmk_hid_mouse_resolution_feature_report_body report = {
        .wheel_res = mult.wheel,
        .hwheel_res = mult.hor_wheel,
    };

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &report,
                             sizeof(struct zmk_hid_mouse_resolution_feature_report_body));
}

static ssize_t write_hids_mouse_feature_report(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr, const void *buf,
                                               uint16_t len, uint16_t offset, uint8_t flags) {
    if (offset != 0) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }
    if (len != sizeof(struct zmk_hid_mouse_resolution_feature_report_body)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    struct zmk_hid_mouse_resolution_feature_report_body *report =
        (struct zmk_hid_mouse_resolution_feature_report_body *)buf;
    int profile = zmk_ble_profile_index(bt_conn_get_dst(conn));
    if (profile < 0) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    struct zmk_endpoint_instance endpoint = {.transport = ZMK_TRANSPORT_BLE,
                                             .ble = {
                                                 .profile_index = profile,
                                             }};
    zmk_pointing_resolution_multipliers_process_report(report, endpoint);

    return len;
}

#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

// static ssize_t write_proto_mode(struct bt_conn *conn,
//                                 const struct bt_gatt_attr *attr,
//                                 const void *buf, uint16_t len, uint16_t offset,
//                                 uint8_t flags)
// {
//     printk("PROTO CHANGED\n");
//     return 0;
// }

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    host_requests_notification = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t write_ctrl_point(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    uint8_t *value = attr->user_data;

    if (offset + len > sizeof(ctrl_point)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    memcpy(value + offset, buf, len);

    return len;
}

/* HID Service Declaration */
BT_GATT_SERVICE_DEFINE(
    hog_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
    //    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_PROTOCOL_MODE, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
    //                           BT_GATT_PERM_WRITE, NULL, write_proto_mode, &proto_mode),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, read_hids_info,
                           NULL, &info),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ, BT_GATT_PERM_READ_ENCRYPT,
                           read_hids_report_map, NULL, NULL),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_hids_input_report, NULL, NULL),
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &input),

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_hids_consumer_input_report, NULL, NULL),
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &consumer_input),

#if IS_ENABLED(CONFIG_ZMK_POINTING)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_ENCRYPT, read_hids_mouse_input_report, NULL, NULL),
    BT_GATT_CCC(input_ccc_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &mouse_input),

#if IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
                           read_hids_mouse_feature_report, write_hids_mouse_feature_report, NULL),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &mouse_feature),
#endif // IS_ENABLED(CONFIG_ZMK_POINTING_SMOOTH_SCROLLING)

#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

#if IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT, NULL,
                           write_hids_leds_report, NULL),
    BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ_ENCRYPT, read_hids_report_ref,
                       NULL, &led_indicators),
#endif // IS_ENABLED(CONFIG_ZMK_HID_INDICATORS)

    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, write_ctrl_point, &ctrl_point));

K_THREAD_STACK_DEFINE(hog_q_stack, CONFIG_ZMK_BLE_THREAD_STACK_SIZE);

struct k_work_q hog_work_q;

K_MSGQ_DEFINE(zmk_hog_keyboard_msgq, sizeof(struct zmk_hid_keyboard_report_body),
              CONFIG_ZMK_BLE_KEYBOARD_REPORT_QUEUE_SIZE, 4);

void send_keyboard_report_callback(struct k_work *work) {
    struct zmk_hid_keyboard_report_body report;

    while (k_msgq_get(&zmk_hog_keyboard_msgq, &report, K_NO_WAIT) == 0) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            return;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &hog_svc.attrs[5],
            .data = &report,
            .len = sizeof(report),
        };

        int err = bt_gatt_notify_cb(conn, &notify_params);
        if (err == -EPERM) {
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else if (err) {
            LOG_DBG("Error notifying %d", err);
        }

        bt_conn_unref(conn);
    }
}

K_WORK_DEFINE(hog_keyboard_work, send_keyboard_report_callback);

int zmk_hog_send_keyboard_report(struct zmk_hid_keyboard_report_body *report) {
    int err = k_msgq_put(&zmk_hog_keyboard_msgq, report, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Keyboard message queue full, popping first message and queueing again");
            struct zmk_hid_keyboard_report_body discarded_report;
            k_msgq_get(&zmk_hog_keyboard_msgq, &discarded_report, K_NO_WAIT);
            return zmk_hog_send_keyboard_report(report);
        }
        default:
            LOG_WRN("Failed to queue keyboard report to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&hog_work_q, &hog_keyboard_work);

    return 0;
};

K_MSGQ_DEFINE(zmk_hog_consumer_msgq, sizeof(struct zmk_hid_consumer_report_body),
              CONFIG_ZMK_BLE_CONSUMER_REPORT_QUEUE_SIZE, 4);

void send_consumer_report_callback(struct k_work *work) {
    struct zmk_hid_consumer_report_body report;

    while (k_msgq_get(&zmk_hog_consumer_msgq, &report, K_NO_WAIT) == 0) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            return;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &hog_svc.attrs[9],
            .data = &report,
            .len = sizeof(report),
        };

        int err = bt_gatt_notify_cb(conn, &notify_params);
        if (err == -EPERM) {
            bt_conn_set_security(conn, BT_SECURITY_L2);
        } else if (err) {
            LOG_DBG("Error notifying %d", err);
        }

        bt_conn_unref(conn);
    }
};

K_WORK_DEFINE(hog_consumer_work, send_consumer_report_callback);

int zmk_hog_send_consumer_report(struct zmk_hid_consumer_report_body *report) {
    int err = k_msgq_put(&zmk_hog_consumer_msgq, report, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Consumer message queue full, popping first message and queueing again");
            struct zmk_hid_consumer_report_body discarded_report;
            k_msgq_get(&zmk_hog_consumer_msgq, &discarded_report, K_NO_WAIT);
            return zmk_hog_send_consumer_report(report);
        }
        default:
            LOG_WRN("Failed to queue consumer report to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&hog_work_q, &hog_consumer_work);

    return 0;
};

#if IS_ENABLED(CONFIG_ZMK_POINTING)

static int notify_mouse_report(struct bt_conn *conn,
                               const struct zmk_hid_mouse_report_body *report,
                               bt_gatt_complete_func_t complete, void *user_data) {
    struct bt_gatt_notify_params notify_params = {
        .attr = &hog_svc.attrs[13],
        .data = report,
        .len = sizeof(*report),
        .func = complete,
        .user_data = user_data,
    };

    int err = bt_gatt_notify_cb(conn, &notify_params);
    if (err == -EPERM) {
        bt_conn_set_security(conn, BT_SECURITY_L2);
    } else if (err) {
        LOG_DBG("Error notifying %d", err);
    }

    return err;
}

#if defined(CONFIG_ZMK_BLE_MOUSE_REPORT_INTERVAL_MS) &&                                    \
    CONFIG_ZMK_BLE_MOUSE_REPORT_INTERVAL_MS > 0

BUILD_ASSERT(CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE > 1,
             "Mouse report queue size must be greater than one");

struct mouse_report_segment {
    int64_t d_x;
    int64_t d_y;
    int64_t d_scroll_y;
    int64_t d_scroll_x;
    zmk_mouse_button_flags_t buttons;
    uint32_t sequence;
    bool button_edge_pending;
};

struct mouse_report_snapshot {
    struct zmk_hid_mouse_report_body report;
    uint32_t generation;
    uint32_t sequence;
    bool button_edge_pending;
};

struct mouse_report_coalescer {
    struct mouse_report_segment segments[CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE];
    struct bt_conn *conn;
    size_t head;
    size_t count;
    zmk_mouse_button_flags_t latest_buttons;
    uint32_t generation;
    uint32_t next_sequence;
    uint32_t next_notify_token;
    uint32_t in_flight_token;
    int64_t retry_not_before;
    struct mouse_report_snapshot in_flight_snapshot;
    bool in_flight;
};

static struct mouse_report_coalescer mouse_coalescer;
K_MUTEX_DEFINE(mouse_coalescer_mutex);

K_THREAD_STACK_DEFINE(hog_mouse_q_stack, CONFIG_ZMK_BLE_THREAD_STACK_SIZE);
static struct k_work_q hog_mouse_work_q;

static void send_mouse_report_callback(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(hog_mouse_work, send_mouse_report_callback);

static int16_t clamp_mouse_delta(int64_t value) {
    if (value > INT16_MAX) {
        return INT16_MAX;
    }
    if (value < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static bool mouse_segment_has_work(const struct mouse_report_segment *segment) {
    return segment->button_edge_pending || segment->d_x != 0 || segment->d_y != 0 ||
           segment->d_scroll_y != 0 || segment->d_scroll_x != 0;
}

static struct mouse_report_segment *mouse_segment_at(size_t offset) {
    size_t index = (mouse_coalescer.head + offset) % CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE;
    return &mouse_coalescer.segments[index];
}

static struct mouse_report_segment *mouse_head_segment(void) { return mouse_segment_at(0); }

static struct mouse_report_segment *mouse_tail_segment(void) {
    return mouse_segment_at(mouse_coalescer.count - 1);
}

static void mouse_coalescer_ensure_segment(void) {
    if (mouse_coalescer.count != 0) {
        return;
    }

    mouse_coalescer.head = 0;
    mouse_coalescer.count = 1;
    mouse_coalescer.segments[0] = (struct mouse_report_segment){
        .buttons = mouse_coalescer.latest_buttons,
        .sequence = ++mouse_coalescer.next_sequence,
    };
}

static void mouse_coalescer_reset_locked(void) {
    if (mouse_coalescer.conn != NULL) {
        bt_conn_unref(mouse_coalescer.conn);
        mouse_coalescer.conn = NULL;
    }

    memset(mouse_coalescer.segments, 0, sizeof(mouse_coalescer.segments));
    mouse_coalescer.head = 0;
    mouse_coalescer.count = 0;
    mouse_coalescer.latest_buttons = 0;
    mouse_coalescer.in_flight_token = 0;
    mouse_coalescer.retry_not_before = 0;
    memset(&mouse_coalescer.in_flight_snapshot, 0,
           sizeof(mouse_coalescer.in_flight_snapshot));
    mouse_coalescer.in_flight = false;
    mouse_coalescer.generation++;
    mouse_coalescer_ensure_segment();
}

static void mouse_coalescer_prune_locked(void) {
    while (mouse_coalescer.count > 1 && !mouse_segment_has_work(mouse_head_segment())) {
        memset(mouse_head_segment(), 0, sizeof(*mouse_head_segment()));
        mouse_coalescer.head =
            (mouse_coalescer.head + 1) % CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE;
        mouse_coalescer.count--;
    }
}

static bool mouse_coalescer_next_delay_locked(int64_t now, int64_t *delay_ms) {
    if (mouse_coalescer.in_flight) {
        return false;
    }

    mouse_coalescer_prune_locked();
    struct mouse_report_segment *segment = mouse_head_segment();
    if (!mouse_segment_has_work(segment)) {
        return false;
    }

    *delay_ms = mouse_coalescer.retry_not_before > now
                    ? mouse_coalescer.retry_not_before - now
                    : 0;
    return true;
}

static void schedule_mouse_report(int64_t delay_ms) {
    int err = k_work_reschedule_for_queue(&hog_mouse_work_q, &hog_mouse_work,
                                          delay_ms > 0 ? K_MSEC(delay_ms) : K_NO_WAIT);
    if (err < 0) {
        LOG_WRN("Failed to schedule mouse report (%d)", err);
    }
}

static bool mouse_conn_is_connected(struct bt_conn *conn) {
    struct bt_conn_info info;
    return conn != NULL && bt_conn_get_info(conn, &info) == 0 &&
           info.state == BT_CONN_STATE_CONNECTED;
}

static uint32_t mouse_next_notify_token_locked(void) {
    mouse_coalescer.next_notify_token++;
    if (mouse_coalescer.next_notify_token == 0) {
        mouse_coalescer.next_notify_token++;
    }
    return mouse_coalescer.next_notify_token;
}

static void mouse_report_notify_complete(struct bt_conn *conn, void *user_data) {
    uint32_t token = (uint32_t)(uintptr_t)user_data;
    int64_t delay_ms = 0;
    bool should_schedule = false;

    k_mutex_lock(&mouse_coalescer_mutex, K_FOREVER);

    if (!mouse_coalescer.in_flight || token != mouse_coalescer.in_flight_token) {
        k_mutex_unlock(&mouse_coalescer_mutex);
        return;
    }

    struct mouse_report_snapshot snapshot = mouse_coalescer.in_flight_snapshot;
    bool snapshot_matches = mouse_coalescer.conn == conn &&
                            mouse_coalescer.generation == snapshot.generation &&
                            mouse_head_segment()->sequence == snapshot.sequence;

    mouse_coalescer.in_flight = false;
    mouse_coalescer.in_flight_token = 0;
    mouse_coalescer.retry_not_before = 0;

    if (snapshot_matches) {
        struct mouse_report_segment *segment = mouse_head_segment();
        segment->d_x -= snapshot.report.d_x;
        segment->d_y -= snapshot.report.d_y;
        segment->d_scroll_y -= snapshot.report.d_scroll_y;
        segment->d_scroll_x -= snapshot.report.d_scroll_x;
        if (snapshot.button_edge_pending) {
            segment->button_edge_pending = false;
        }
    }

    should_schedule = mouse_coalescer_next_delay_locked(k_uptime_get(), &delay_ms);
    k_mutex_unlock(&mouse_coalescer_mutex);

    if (should_schedule) {
        schedule_mouse_report(delay_ms);
    }
}

static void send_mouse_report_callback(struct k_work *work) {
    struct mouse_report_snapshot snapshot;
    struct bt_conn *conn = NULL;
    int64_t delay_ms = 0;
    bool should_schedule = false;

    k_mutex_lock(&mouse_coalescer_mutex, K_FOREVER);

    if (!mouse_coalescer_next_delay_locked(k_uptime_get(), &delay_ms)) {
        k_mutex_unlock(&mouse_coalescer_mutex);
        return;
    }

    if (delay_ms > 0) {
        k_mutex_unlock(&mouse_coalescer_mutex);
        schedule_mouse_report(delay_ms);
        return;
    }

    struct mouse_report_segment *segment = mouse_head_segment();
    snapshot = (struct mouse_report_snapshot){
        .report =
            {
                .buttons = segment->buttons,
                .d_x = clamp_mouse_delta(segment->d_x),
                .d_y = clamp_mouse_delta(segment->d_y),
                .d_scroll_y = clamp_mouse_delta(segment->d_scroll_y),
                .d_scroll_x = clamp_mouse_delta(segment->d_scroll_x),
            },
        .generation = mouse_coalescer.generation,
        .sequence = segment->sequence,
        .button_edge_pending = segment->button_edge_pending,
    };
    uint32_t notify_token = mouse_next_notify_token_locked();
    mouse_coalescer.in_flight_snapshot = snapshot;
    mouse_coalescer.in_flight_token = notify_token;
    mouse_coalescer.in_flight = true;
    if (mouse_coalescer.conn != NULL) {
        conn = bt_conn_ref(mouse_coalescer.conn);
    }

    k_mutex_unlock(&mouse_coalescer_mutex);

    struct bt_conn *active_conn = zmk_ble_active_profile_conn();
    bool connection_matches = active_conn == conn && mouse_conn_is_connected(conn);
    if (active_conn != NULL) {
        bt_conn_unref(active_conn);
    }

    k_mutex_lock(&mouse_coalescer_mutex, K_FOREVER);
    bool notify_still_current = mouse_coalescer.in_flight &&
                                mouse_coalescer.in_flight_token == notify_token &&
                                mouse_coalescer.generation == snapshot.generation;
    k_mutex_unlock(&mouse_coalescer_mutex);
    connection_matches = connection_matches && notify_still_current;

    int err = connection_matches
                  ? notify_mouse_report(conn, &snapshot.report, mouse_report_notify_complete,
                                        (void *)(uintptr_t)notify_token)
                  : -ENOTCONN;
    if (conn != NULL) {
        bt_conn_unref(conn);
    }

    if (err != 0) {
        k_mutex_lock(&mouse_coalescer_mutex, K_FOREVER);
        bool notify_matches = mouse_coalescer.in_flight &&
                              mouse_coalescer.in_flight_token == notify_token &&
                              mouse_coalescer.generation == snapshot.generation;

        if (notify_matches && (!connection_matches || err == -ENOTCONN)) {
            mouse_coalescer_reset_locked();
        } else if (notify_matches) {
            mouse_coalescer.in_flight = false;
            mouse_coalescer.in_flight_token = 0;
            bool transient_error = err == -ENOMEM || err == -ENOBUFS || err == -EAGAIN ||
                                   err == -EPERM;
            int64_t retry_delay = transient_error
                                      ? CONFIG_ZMK_BLE_MOUSE_REPORT_INTERVAL_MS
                                      : MAX(CONFIG_ZMK_BLE_MOUSE_REPORT_INTERVAL_MS, 250);
            mouse_coalescer.retry_not_before = k_uptime_get() + retry_delay;
            should_schedule =
                mouse_coalescer_next_delay_locked(k_uptime_get(), &delay_ms);
        }
        k_mutex_unlock(&mouse_coalescer_mutex);

        if (should_schedule) {
            schedule_mouse_report(delay_ms);
        }
    }
}

void zmk_hog_reset_mouse_reports(void) {
    k_mutex_lock(&mouse_coalescer_mutex, K_FOREVER);
    mouse_coalescer_reset_locked();
    k_mutex_unlock(&mouse_coalescer_mutex);
}

static void mouse_segment_add_report(struct mouse_report_segment *segment,
                                     const struct zmk_hid_mouse_report_body *report) {
    segment->d_x += report->d_x;
    segment->d_y += report->d_y;
    segment->d_scroll_y += report->d_scroll_y;
    segment->d_scroll_x += report->d_scroll_x;
}

int zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *report) {
    struct bt_conn *conn = zmk_ble_active_profile_conn();
    if (!mouse_conn_is_connected(conn)) {
        if (conn != NULL) {
            bt_conn_unref(conn);
        }
        k_mutex_lock(&mouse_coalescer_mutex, K_FOREVER);
        bool needs_reset = mouse_coalescer.conn != NULL;
        k_mutex_unlock(&mouse_coalescer_mutex);
        if (needs_reset) {
            zmk_hog_reset_mouse_reports();
        }
        return 0;
    }

    int64_t delay_ms = 0;
    bool should_schedule = false;

    k_mutex_lock(&mouse_coalescer_mutex, K_FOREVER);

    if (mouse_coalescer.conn != conn) {
        mouse_coalescer_reset_locked();
        mouse_coalescer.conn = conn;
        conn = NULL;
    }
    mouse_coalescer_ensure_segment();

    bool buttons_changed = report->buttons != mouse_coalescer.latest_buttons;
    struct mouse_report_segment *segment = mouse_tail_segment();

    if (buttons_changed) {
        if (mouse_segment_has_work(segment)) {
            if (mouse_coalescer.count >= CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE) {
                // Preserve all relative movement and the final button state.
                // Only intermediate transitions are collapsed if a host has
                // stalled through the entire bounded transition queue.
                LOG_WRN("Mouse button transition queue full; collapsing latest transition");
                segment->buttons = report->buttons;
                segment->sequence = ++mouse_coalescer.next_sequence;
                segment->button_edge_pending = true;
            } else {
                size_t next_index =
                    (mouse_coalescer.head + mouse_coalescer.count) %
                    CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE;
                segment = &mouse_coalescer.segments[next_index];
                *segment = (struct mouse_report_segment){
                    .buttons = report->buttons,
                    .sequence = ++mouse_coalescer.next_sequence,
                    .button_edge_pending = true,
                };
                mouse_coalescer.count++;
            }
        } else {
            segment->buttons = report->buttons;
            segment->sequence = ++mouse_coalescer.next_sequence;
            segment->button_edge_pending = true;
        }

        mouse_coalescer.latest_buttons = report->buttons;
        mouse_segment_add_report(segment, report);
    } else {
        mouse_segment_add_report(segment, report);
    }

    should_schedule = mouse_coalescer_next_delay_locked(k_uptime_get(), &delay_ms);
    k_mutex_unlock(&mouse_coalescer_mutex);

    if (conn != NULL) {
        bt_conn_unref(conn);
    }
    if (should_schedule) {
        schedule_mouse_report(delay_ms);
    }

    return 0;
}

#else

K_MSGQ_DEFINE(zmk_hog_mouse_msgq, sizeof(struct zmk_hid_mouse_report_body),
              CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE, 4);

static void send_mouse_report_callback(struct k_work *work) {
    struct zmk_hid_mouse_report_body report;
    while (k_msgq_get(&zmk_hog_mouse_msgq, &report, K_NO_WAIT) == 0) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            k_msgq_purge(&zmk_hog_mouse_msgq);
            return;
        }

        notify_mouse_report(conn, &report, NULL, NULL);
        bt_conn_unref(conn);
    }
}

K_WORK_DEFINE(hog_mouse_work, send_mouse_report_callback);

static int queue_mouse_report(const struct zmk_hid_mouse_report_body *report) {
    int err = k_msgq_put(&zmk_hog_mouse_msgq, report, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Mouse message queue full, popping first message and queueing again");
            struct zmk_hid_mouse_report_body discarded_report;
            k_msgq_get(&zmk_hog_mouse_msgq, &discarded_report, K_NO_WAIT);
            return queue_mouse_report(report);
        }
        default:
            LOG_WRN("Failed to queue mouse report to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&hog_work_q, &hog_mouse_work);
    return 0;
}

void zmk_hog_reset_mouse_reports(void) { k_msgq_purge(&zmk_hog_mouse_msgq); }

int zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *report) {
    if (!zmk_ble_active_profile_is_connected()) {
        zmk_hog_reset_mouse_reports();
        return 0;
    }
    return queue_mouse_report(report);
}

#endif
#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

static int zmk_hog_init(void) {
    static const struct k_work_queue_config queue_config = {.name = "HID Over GATT Send Work"};
    k_work_queue_start(&hog_work_q, hog_q_stack, K_THREAD_STACK_SIZEOF(hog_q_stack),
                       CONFIG_ZMK_BLE_THREAD_PRIORITY, &queue_config);

#if IS_ENABLED(CONFIG_ZMK_POINTING) &&                                                     \
    defined(CONFIG_ZMK_BLE_MOUSE_REPORT_INTERVAL_MS) &&                                    \
    CONFIG_ZMK_BLE_MOUSE_REPORT_INTERVAL_MS > 0
    static const struct k_work_queue_config mouse_queue_config = {.name = "BLE Mouse Send Work"};
    k_work_queue_start(&hog_mouse_work_q, hog_mouse_q_stack,
                       K_THREAD_STACK_SIZEOF(hog_mouse_q_stack),
                       CONFIG_ZMK_BLE_THREAD_PRIORITY + 1, &mouse_queue_config);
#endif

    return 0;
}

SYS_INIT(zmk_hog_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
