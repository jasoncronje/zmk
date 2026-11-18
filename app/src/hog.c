/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/settings/settings.h>
#include <zephyr/init.h>

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

#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_REPORT_COALESCE)

/*
 * At most one mouse notification per connection interval. Reports that arrive
 * in between are summed into the newest waiting report (same buttons), so each
 * connection event carries one report holding all movement since the last one,
 * and a fast sensor can never build a backlog. A button change starts a new
 * report so no click is merged away.
 *
 * Paced by time, not by the sent callback: a notification is only acknowledged
 * in the following connection event, so waiting for it left every other event
 * unused (30 ms updates on a 15 ms link).
 */

#define MOUSE_PENDING_MAX CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE
// Used until the connection reports its interval
#define MOUSE_DEFAULT_INTERVAL_US 15000

static struct k_spinlock mouse_lock;
static struct zmk_hid_mouse_report_body mouse_pending[MOUSE_PENDING_MAX];
static uint8_t mouse_pending_head;
static uint8_t mouse_pending_count;
static int64_t mouse_next_send_us;

void send_mouse_report_callback(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(hog_mouse_work, send_mouse_report_callback);

static bool add_fits(int16_t a, int16_t b) {
    int32_t sum = (int32_t)a + b;
    return sum >= INT16_MIN && sum <= INT16_MAX;
}

static bool mouse_report_merge(struct zmk_hid_mouse_report_body *into,
                               const struct zmk_hid_mouse_report_body *report) {
    if (into->buttons != report->buttons || !add_fits(into->d_x, report->d_x) ||
        !add_fits(into->d_y, report->d_y) || !add_fits(into->d_scroll_x, report->d_scroll_x) ||
        !add_fits(into->d_scroll_y, report->d_scroll_y)) {
        return false;
    }

    into->d_x += report->d_x;
    into->d_y += report->d_y;
    into->d_scroll_x += report->d_scroll_x;
    into->d_scroll_y += report->d_scroll_y;
    return true;
}

static int64_t mouse_interval_us(struct bt_conn *conn) {
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) == 0 && info.le.interval > 0) {
        return (int64_t)info.le.interval * 1250;
    }
    return MOUSE_DEFAULT_INTERVAL_US;
}

static int64_t now_us(void) { return k_ticks_to_us_floor64(k_uptime_ticks()); }

// Caller holds mouse_lock
static void mouse_queue_push(const struct zmk_hid_mouse_report_body *report) {
    uint8_t tail = (mouse_pending_head + mouse_pending_count + MOUSE_PENDING_MAX - 1) %
                   MOUSE_PENDING_MAX;
    if (mouse_pending_count > 0 && mouse_report_merge(&mouse_pending[tail], report)) {
        return;
    }
    if (mouse_pending_count == MOUSE_PENDING_MAX) {
        LOG_WRN("Mouse report queue full, dropping the oldest report");
        mouse_pending_head = (mouse_pending_head + 1) % MOUSE_PENDING_MAX;
        mouse_pending_count--;
    }
    tail = (mouse_pending_head + mouse_pending_count) % MOUSE_PENDING_MAX;
    mouse_pending[tail] = *report;
    mouse_pending_count++;
}

// Caller holds mouse_lock
static void mouse_queue_pop(struct zmk_hid_mouse_report_body *report) {
    *report = mouse_pending[mouse_pending_head];
    mouse_pending_head = (mouse_pending_head + 1) % MOUSE_PENDING_MAX;
    mouse_pending_count--;
}

// Sends one report and, if more is waiting, comes back after one interval
static void mouse_notify(struct zmk_hid_mouse_report_body *report, bool more) {
    struct bt_conn *conn = zmk_ble_active_profile_conn();
    if (conn == NULL) {
        return;
    }

    int64_t interval = mouse_interval_us(conn);
    k_spinlock_key_t key = k_spin_lock(&mouse_lock);
    mouse_next_send_us = now_us() + interval;
    k_spin_unlock(&mouse_lock, key);

    struct bt_gatt_notify_params notify_params = {
        .attr = &hog_svc.attrs[13],
        .data = report,
        .len = sizeof(*report),
    };

    int err = bt_gatt_notify_cb(conn, &notify_params);
    if (err == -EPERM) {
        bt_conn_set_security(conn, BT_SECURITY_L2);
    } else if (err) {
        LOG_DBG("Error notifying %d", err);
    }

    bt_conn_unref(conn);

    if (more) {
        k_work_schedule_for_queue(&hog_work_q, &hog_mouse_work, K_USEC(interval));
    }
}

#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_REPORT_RESAMPLE)

/*
 * Even time slices. Each sensor reading is movement spread evenly over the time
 * since the previous reading. A report sent at time T carries exactly the
 * movement that falls before T - one sample period, with fractions of a count
 * kept for the next report. At a steady ball speed every report then holds the
 * same counts, however the sensor's readings line up with the connection
 * interval (otherwise a 4 ms sensor on a 15 ms link alternates 4 and 3 readings
 * per report, and macOS accelerates the 4s more). Costs one sample period of
 * latency. Only x/y movement is resampled; buttons and scroll go through the
 * queue, and a button change first flushes all movement so clicks land where
 * the pointer was.
 */

#define RS_SEGMENTS 8
#define RS_SAMPLE_US CONFIG_ZMK_BLE_MOUSE_REPORT_RESAMPLE_SAMPLE_US
#define RS_ONE 256 // fixed point: 1/256 count

struct rs_segment {
    int64_t start_us; // movement before this has been sent
    int64_t end_us;
    int32_t x, y; // movement left in [start_us, end_us], 1/256 counts
};

static struct rs_segment rs_seg[RS_SEGMENTS];
static uint8_t rs_head;
static uint8_t rs_count;
static int64_t rs_last_us;
static int32_t rs_acc_x, rs_acc_y;
static zmk_mouse_button_flags_t rs_buttons;

static int16_t clamp16(int32_t v) {
    return v > INT16_MAX ? INT16_MAX : (v < INT16_MIN ? INT16_MIN : (int16_t)v);
}

// Caller holds mouse_lock
static void rs_add(int16_t dx, int16_t dy, int64_t now) {
    if (dx == 0 && dy == 0) {
        return;
    }
    int64_t dur = now - rs_last_us;
    if (rs_count > 0 && dur < RS_SAMPLE_US / 4) {
        // Same reading split over several reports: add it to the newest slice
        struct rs_segment *s = &rs_seg[(rs_head + rs_count - 1) % RS_SEGMENTS];
        s->x += dx * RS_ONE;
        s->y += dy * RS_ONE;
        return;
    }
    if (dur <= 0 || dur > 2 * RS_SAMPLE_US) {
        dur = RS_SAMPLE_US; // first reading after a pause
    }
    if (rs_count == RS_SEGMENTS) {
        // Link stalled: hand the oldest slice over whole
        rs_acc_x += rs_seg[rs_head].x;
        rs_acc_y += rs_seg[rs_head].y;
        rs_head = (rs_head + 1) % RS_SEGMENTS;
        rs_count--;
    }
    rs_seg[(rs_head + rs_count) % RS_SEGMENTS] = (struct rs_segment){
        .start_us = now - dur, .end_us = now, .x = dx * RS_ONE, .y = dy * RS_ONE};
    rs_count++;
    rs_last_us = now;
}

static int16_t rs_out(int32_t *acc, bool last) {
    int32_t q = last ? (*acc + (*acc >= 0 ? RS_ONE / 2 : -RS_ONE / 2)) / RS_ONE : *acc / RS_ONE;
    int16_t out = clamp16(q);
    *acc -= (int32_t)out * RS_ONE;
    if (last && *acc > -RS_ONE && *acc < RS_ONE) {
        *acc = 0; // pointer stopped: no leftover fraction to creep later
    }
    return out;
}

// Movement before cutoff. Caller holds mouse_lock.
static void rs_take(int64_t cutoff, int16_t *x, int16_t *y) {
    while (rs_count > 0) {
        struct rs_segment *s = &rs_seg[rs_head];
        if (s->end_us <= cutoff) {
            rs_acc_x += s->x;
            rs_acc_y += s->y;
            rs_head = (rs_head + 1) % RS_SEGMENTS;
            rs_count--;
            continue;
        }
        if (s->start_us < cutoff) {
            int64_t span = s->end_us - s->start_us;
            int32_t px = (int32_t)((int64_t)s->x * (cutoff - s->start_us) / span);
            int32_t py = (int32_t)((int64_t)s->y * (cutoff - s->start_us) / span);
            rs_acc_x += px;
            rs_acc_y += py;
            s->x -= px;
            s->y -= py;
            s->start_us = cutoff;
        }
        break;
    }
    bool last = rs_count == 0;
    *x = rs_out(&rs_acc_x, last);
    *y = rs_out(&rs_acc_y, last);
}

static bool rs_pending(void) { return rs_count > 0 || rs_acc_x != 0 || rs_acc_y != 0; }

void send_mouse_report_callback(struct k_work *work) {
    struct zmk_hid_mouse_report_body report;

    k_spinlock_key_t key = k_spin_lock(&mouse_lock);
    if (mouse_pending_count == 0 && !rs_pending()) {
        k_spin_unlock(&mouse_lock, key);
        return;
    }
    int64_t now = now_us();
    int64_t wait = mouse_next_send_us - now;
    if (wait > 0) {
        k_spin_unlock(&mouse_lock, key);
        k_work_schedule_for_queue(&hog_work_q, &hog_mouse_work, K_USEC(wait));
        return;
    }
    if (mouse_pending_count > 1) {
        // Older button states first; resampled movement comes after them
        mouse_queue_pop(&report);
    } else {
        int16_t rx, ry;
        rs_take(now - RS_SAMPLE_US, &rx, &ry);
        if (mouse_pending_count == 1) {
            mouse_queue_pop(&report);
            report.d_x = clamp16((int32_t)report.d_x + rx);
            report.d_y = clamp16((int32_t)report.d_y + ry);
        } else if (rx != 0 || ry != 0) {
            report = (struct zmk_hid_mouse_report_body){
                .buttons = rs_buttons, .d_x = rx, .d_y = ry};
        } else {
            // Nothing due yet (first reading after a pause): look again shortly
            bool pending = rs_pending();
            k_spin_unlock(&mouse_lock, key);
            if (pending) {
                k_work_schedule_for_queue(&hog_work_q, &hog_mouse_work, K_USEC(1000));
            }
            return;
        }
    }
    bool more = mouse_pending_count > 0 || rs_pending();
    k_spin_unlock(&mouse_lock, key);

    mouse_notify(&report, more);
}

#else // IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_REPORT_RESAMPLE)

void send_mouse_report_callback(struct k_work *work) {
    struct zmk_hid_mouse_report_body report;

    k_spinlock_key_t key = k_spin_lock(&mouse_lock);
    if (mouse_pending_count == 0) {
        k_spin_unlock(&mouse_lock, key);
        return;
    }
    int64_t wait = mouse_next_send_us - now_us();
    if (wait > 0) {
        k_spin_unlock(&mouse_lock, key);
        k_work_schedule_for_queue(&hog_work_q, &hog_mouse_work, K_USEC(wait));
        return;
    }
    mouse_queue_pop(&report);
    bool more = mouse_pending_count > 0;
    k_spin_unlock(&mouse_lock, key);

    mouse_notify(&report, more);
}

#endif // IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_REPORT_RESAMPLE)

int zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *report) {
    k_spinlock_key_t key = k_spin_lock(&mouse_lock);
#if IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_REPORT_RESAMPLE)
    if (report->buttons != rs_buttons) {
        int16_t fx, fy;
        rs_take(INT64_MAX, &fx, &fy);
        if (fx != 0 || fy != 0) {
            struct zmk_hid_mouse_report_body moved = {
                .buttons = rs_buttons, .d_x = fx, .d_y = fy};
            mouse_queue_push(&moved);
        }
        mouse_queue_push(report);
        rs_buttons = report->buttons;
    } else {
        rs_add(report->d_x, report->d_y, now_us());
        if (report->d_scroll_x != 0 || report->d_scroll_y != 0) {
            struct zmk_hid_mouse_report_body scroll = {.buttons = rs_buttons,
                                                       .d_scroll_x = report->d_scroll_x,
                                                       .d_scroll_y = report->d_scroll_y};
            mouse_queue_push(&scroll);
        }
    }
#else
    mouse_queue_push(report);
#endif
    k_spin_unlock(&mouse_lock, key);

    k_work_schedule_for_queue(&hog_work_q, &hog_mouse_work, K_NO_WAIT);

    return 0;
};

#else // IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_REPORT_COALESCE)

K_MSGQ_DEFINE(zmk_hog_mouse_msgq, sizeof(struct zmk_hid_mouse_report_body),
              CONFIG_ZMK_BLE_MOUSE_REPORT_QUEUE_SIZE, 4);

void send_mouse_report_callback(struct k_work *work) {
    struct zmk_hid_mouse_report_body report;
    while (k_msgq_get(&zmk_hog_mouse_msgq, &report, K_NO_WAIT) == 0) {
        struct bt_conn *conn = zmk_ble_active_profile_conn();
        if (conn == NULL) {
            return;
        }

        struct bt_gatt_notify_params notify_params = {
            .attr = &hog_svc.attrs[13],
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

K_WORK_DEFINE(hog_mouse_work, send_mouse_report_callback);

int zmk_hog_send_mouse_report(struct zmk_hid_mouse_report_body *report) {
    int err = k_msgq_put(&zmk_hog_mouse_msgq, report, K_MSEC(100));
    if (err) {
        switch (err) {
        case -EAGAIN: {
            LOG_WRN("Consumer message queue full, popping first message and queueing again");
            struct zmk_hid_mouse_report_body discarded_report;
            k_msgq_get(&zmk_hog_mouse_msgq, &discarded_report, K_NO_WAIT);
            return zmk_hog_send_mouse_report(report);
        }
        default:
            LOG_WRN("Failed to queue mouse report to send (%d)", err);
            return err;
        }
    }

    k_work_submit_to_queue(&hog_work_q, &hog_mouse_work);

    return 0;
};
#endif // IS_ENABLED(CONFIG_ZMK_BLE_MOUSE_REPORT_COALESCE)

#endif // IS_ENABLED(CONFIG_ZMK_POINTING)

static int zmk_hog_init(void) {
    static const struct k_work_queue_config queue_config = {.name = "HID Over GATT Send Work"};
    k_work_queue_start(&hog_work_q, hog_q_stack, K_THREAD_STACK_SIZEOF(hog_q_stack),
                       CONFIG_ZMK_BLE_THREAD_PRIORITY, &queue_config);

    return 0;
}

SYS_INIT(zmk_hog_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);
