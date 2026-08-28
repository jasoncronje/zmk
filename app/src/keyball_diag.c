/*
 * RAM-only Keyball Bluetooth timing capture.
 *
 * No event is formatted, logged, or written to flash while capture is active.
 * The circular buffer is frozen when USB is attached and emitted only after a
 * serial client asserts CDC DTR.
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <stdarg.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include <zmk/activity.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/keyball_diag.h>
#include <zmk/usb.h>

#define DIAG_UART_NODE DT_CHOSEN(zmk_studio_rpc_uart)
#define DIAG_EVENT_CAPACITY 1024
#define DIAG_TRANSITION_CAPACITY 128
#define DIAG_NOTIFY_SLOTS 256
#define DIAG_LINE_SIZE 512
#define DIAG_SLOW_HOG_US 1000U
#define DIAG_SLOW_NOTIFY_RETURN_US 1000U
#define DIAG_SLOW_WORK_US 2000U
#define DIAG_SPLIT_RX_SAMPLE_INTERVAL 32U
#define DIAG_SPLIT_RX_LONG_GAP_US 100000U

BUILD_ASSERT(DT_NODE_HAS_STATUS(DIAG_UART_NODE, okay),
             "Keyball diagnostics require the Studio USB UART snippet");
BUILD_ASSERT(IS_POWER_OF_TWO(DIAG_EVENT_CAPACITY));
BUILD_ASSERT(IS_POWER_OF_TWO(DIAG_TRANSITION_CAPACITY));

struct diag_event {
    atomic_t sequence;
    uint32_t timestamp_us;
    uint8_t type;
    uint8_t reserved[3];
    int32_t a;
    int32_t b;
    int32_t c;
    int32_t d;
};

static struct diag_event events[DIAG_EVENT_CAPACITY];
static struct diag_event transition_events[DIAG_TRANSITION_CAPACITY];
static atomic_t counters[ZMK_KEYBALL_DIAG_EVENT_COUNT];
static atomic_t next_sequence;
static atomic_t next_transition_sequence;
static atomic_t capture_frozen;
static atomic_t usb_present;
static atomic_t dump_complete;
static atomic_t uart_claimed;

static atomic_t last_input_us;
static atomic_t max_input_gap_us;
static atomic_t hog_started_us;
static atomic_t max_hog_duration_us;
static atomic_t hog_disconnected;
static atomic_t last_notify_us;
static atomic_t max_notify_gap_us;
static atomic_t scheduled_due_us;
static atomic_t max_work_late_us;
static atomic_t next_notify_id;
static atomic_t notify_started_us[DIAG_NOTIFY_SLOTS];
static atomic_t notify_slot_ids[DIAG_NOTIFY_SLOTS];
static atomic_t notify_deltas[DIAG_NOTIFY_SLOTS];
static atomic_t notify_outstanding;
static atomic_t max_notify_outstanding;
static atomic_t max_notify_complete_us;
static atomic_t max_notify_return_us;
static atomic_t notify_unmatched;
static atomic_t notify_underflows;
static atomic_t schedule_errors;
static atomic_t schedule_last_error;

static atomic_t host_connected;
static atomic_t host_last_connect_error;
static atomic_t host_last_disconnect_reason;
static atomic_t host_security_level;
static atomic_t host_security_error;
static atomic_t host_interval;
static atomic_t host_latency;
static atomic_t host_timeout;

static atomic_t split_enabled;
static atomic_t split_scanning;
static atomic_t split_connections;
static atomic_t split_available;
static atomic_t split_last_scan_error;
static atomic_t split_last_create_error;
static atomic_t split_last_connect_error;
static atomic_t split_last_disconnect_reason;
static atomic_t split_last_security_error;
static atomic_t split_interval;
static atomic_t split_latency;
static atomic_t split_timeout;
static atomic_t split_last_discovery_error;
static atomic_t split_last_subscribe_error;
static atomic_t split_ready;
static atomic_t split_last_slot;
static atomic_t split_service_found_count;
static atomic_t split_scan_retry_count;
static atomic_t split_rx_count;
static atomic_t split_last_rx_us;
static atomic_t split_max_rx_gap_us;
static atomic_t split_rx_ready;

K_SEM_DEFINE(dump_request, 0, 1);

static const struct device *const diag_uart = DEVICE_DT_GET(DIAG_UART_NODE);

static const char *const event_names[ZMK_KEYBALL_DIAG_EVENT_COUNT] = {
    [ZMK_KEYBALL_DIAG_BOOT] = "boot",
    [ZMK_KEYBALL_DIAG_ACTIVITY] = "activity",
    [ZMK_KEYBALL_DIAG_USB] = "usb",
    [ZMK_KEYBALL_DIAG_INPUT] = "input",
    [ZMK_KEYBALL_DIAG_HOG_ENQUEUE] = "hog_enqueue",
    [ZMK_KEYBALL_DIAG_HOG_RESULT] = "hog_result",
    [ZMK_KEYBALL_DIAG_SCHEDULE] = "schedule",
    [ZMK_KEYBALL_DIAG_WORK] = "work",
    [ZMK_KEYBALL_DIAG_NOTIFY_CALL] = "notify_call",
    [ZMK_KEYBALL_DIAG_NOTIFY_RETURN] = "notify_return",
    [ZMK_KEYBALL_DIAG_NOTIFY_COMPLETE] = "notify_complete",
    [ZMK_KEYBALL_DIAG_HOST_CONNECTED] = "host_connected",
    [ZMK_KEYBALL_DIAG_HOST_DISCONNECTED] = "host_disconnected",
    [ZMK_KEYBALL_DIAG_HOST_SECURITY] = "host_security",
    [ZMK_KEYBALL_DIAG_HOST_PARAMS] = "host_params",
    [ZMK_KEYBALL_DIAG_SPLIT_SCAN_RETRY] = "split_scan_retry",
    [ZMK_KEYBALL_DIAG_SPLIT_SCAN_START] = "split_scan_start",
    [ZMK_KEYBALL_DIAG_SPLIT_SCAN_STOP] = "split_scan_stop",
    [ZMK_KEYBALL_DIAG_SPLIT_SERVICE_FOUND] = "split_service_found",
    [ZMK_KEYBALL_DIAG_SPLIT_CREATE] = "split_create",
    [ZMK_KEYBALL_DIAG_SPLIT_CONNECTED] = "split_connected",
    [ZMK_KEYBALL_DIAG_SPLIT_DISCONNECTED] = "split_disconnected",
    [ZMK_KEYBALL_DIAG_SPLIT_SECURITY] = "split_security",
    [ZMK_KEYBALL_DIAG_SPLIT_PARAMS] = "split_params",
    [ZMK_KEYBALL_DIAG_SPLIT_DISCOVER] = "split_discover",
    [ZMK_KEYBALL_DIAG_SPLIT_SUBSCRIBE] = "split_subscribe",
    [ZMK_KEYBALL_DIAG_SPLIT_SUBSCRIBE_COMPLETE] = "split_subscribe_complete",
    [ZMK_KEYBALL_DIAG_SPLIT_READY] = "split_ready",
    [ZMK_KEYBALL_DIAG_SPLIT_RX] = "split_rx",
    [ZMK_KEYBALL_DIAG_SPLIT_ENABLED] = "split_enabled",
    [ZMK_KEYBALL_DIAG_SPLIT_STATUS] = "split_status",
};

static uint32_t now_us(void) { return k_cyc_to_us_floor32(k_cycle_get_32()); }

static void atomic_update_max(atomic_t *maximum, uint32_t value) {
    atomic_val_t old = atomic_get(maximum);
    while (value > (uint32_t)old && !atomic_cas(maximum, old, (atomic_val_t)value)) {
        old = atomic_get(maximum);
    }
}

static void atomic_dec_nonnegative(atomic_t *value) {
    atomic_val_t old = atomic_get(value);
    while (old > 0 && !atomic_cas(value, old, old - 1)) {
        old = atomic_get(value);
    }
    if (old <= 0) {
        atomic_inc(&notify_underflows);
    }
}

static void update_state_snapshot(enum zmk_keyball_diag_event_type type, int32_t a, int32_t b,
                                  int32_t c, int32_t d) {
    switch (type) {
    case ZMK_KEYBALL_DIAG_HOST_CONNECTED:
        atomic_set(&host_last_connect_error, a);
        atomic_set(&host_connected, a == 0);
        break;
    case ZMK_KEYBALL_DIAG_HOST_DISCONNECTED:
        atomic_set(&host_last_disconnect_reason, a);
        atomic_set(&host_connected, 0);
        break;
    case ZMK_KEYBALL_DIAG_HOST_SECURITY:
        atomic_set(&host_security_level, a);
        atomic_set(&host_security_error, b);
        break;
    case ZMK_KEYBALL_DIAG_HOST_PARAMS:
        atomic_set(&host_interval, a);
        atomic_set(&host_latency, b);
        atomic_set(&host_timeout, c);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_SCAN_RETRY:
        atomic_inc(&split_scan_retry_count);
        if (a < 0) {
            atomic_set(&split_last_scan_error, a);
        }
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_SCAN_START:
        atomic_set(&split_last_scan_error, a);
        atomic_set(&split_scanning, b);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_SCAN_STOP:
        atomic_set(&split_last_scan_error, a);
        atomic_set(&split_scanning, b);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_SERVICE_FOUND:
        atomic_set(&split_last_slot, a);
        atomic_inc(&split_service_found_count);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_CREATE:
        atomic_set(&split_last_create_error, a);
        atomic_set(&split_last_slot, b);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_CONNECTED:
        atomic_set(&split_last_connect_error, a);
        atomic_set(&split_last_slot, b);
        atomic_set(&split_ready, 0);
        atomic_set(&split_last_discovery_error, 0);
        atomic_set(&split_last_subscribe_error, 0);
        atomic_set(&split_rx_ready, 0);
        atomic_set(&split_interval, 0);
        atomic_set(&split_latency, 0);
        atomic_set(&split_timeout, 0);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_DISCONNECTED:
        atomic_set(&split_last_disconnect_reason, a);
        atomic_set(&split_last_slot, b);
        atomic_set(&split_ready, 0);
        atomic_set(&split_rx_ready, 0);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_SECURITY:
        atomic_set(&split_last_security_error, b);
        atomic_set(&split_last_slot, c);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_PARAMS:
        atomic_set(&split_interval, a);
        atomic_set(&split_latency, b);
        atomic_set(&split_timeout, c);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_DISCOVER:
        if (a != 0) {
            atomic_set(&split_last_discovery_error, a);
        }
        atomic_set(&split_last_slot, c);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_SUBSCRIBE:
        if (a != 0 && a != -EALREADY) {
            atomic_set(&split_last_subscribe_error, a);
        }
        atomic_set(&split_last_slot, b);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_SUBSCRIBE_COMPLETE:
        if (a != 0) {
            atomic_set(&split_last_subscribe_error, a);
        }
        atomic_set(&split_last_slot, b);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_READY:
        atomic_set(&split_ready, a);
        atomic_set(&split_last_slot, b);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_ENABLED:
        atomic_set(&split_enabled, a);
        break;
    case ZMK_KEYBALL_DIAG_SPLIT_STATUS:
        atomic_set(&split_enabled, a);
        atomic_set(&split_connections, b);
        atomic_set(&split_available, c);
        atomic_set(&split_scanning, d);
        break;
    default:
        break;
    }
}

static void write_event(struct diag_event *ring, uint32_t capacity, atomic_t *next,
                        uint32_t timestamp, enum zmk_keyball_diag_event_type type, int32_t a,
                        int32_t b, int32_t c, int32_t d) {
    uint32_t sequence = (uint32_t)atomic_inc(next) + 1U;
    struct diag_event *event = &ring[(sequence - 1U) & (capacity - 1U)];
    atomic_set(&event->sequence, 0);
    event->timestamp_us = timestamp;
    event->type = (uint8_t)type;
    event->a = a;
    event->b = b;
    event->c = c;
    event->d = d;
    atomic_set(&event->sequence, (atomic_val_t)sequence);
}

static bool is_transition_event(enum zmk_keyball_diag_event_type type) {
    return type == ZMK_KEYBALL_DIAG_BOOT || type == ZMK_KEYBALL_DIAG_ACTIVITY ||
           type == ZMK_KEYBALL_DIAG_USB ||
           (type >= ZMK_KEYBALL_DIAG_HOST_CONNECTED && type != ZMK_KEYBALL_DIAG_SPLIT_RX);
}

static void record_transition_only(enum zmk_keyball_diag_event_type type, int32_t a, int32_t b,
                                   int32_t c, int32_t d) {
    if (atomic_get(&capture_frozen)) {
        return;
    }

    write_event(transition_events, DIAG_TRANSITION_CAPACITY, &next_transition_sequence, now_us(),
                type, a, b, c, d);
}

static void record_event(enum zmk_keyball_diag_event_type type, int32_t a, int32_t b, int32_t c,
                         int32_t d, bool increment_counter) {
    if (type <= 0 || type >= ZMK_KEYBALL_DIAG_EVENT_COUNT || atomic_get(&capture_frozen)) {
        return;
    }

    update_state_snapshot(type, a, b, c, d);
    uint32_t timestamp = now_us();
    write_event(events, DIAG_EVENT_CAPACITY, &next_sequence, timestamp, type, a, b, c, d);
    if (is_transition_event(type)) {
        write_event(transition_events, DIAG_TRANSITION_CAPACITY, &next_transition_sequence,
                    timestamp, type, a, b, c, d);
    }
    if (increment_counter) {
        atomic_inc(&counters[type]);
    }
}

void zmk_keyball_diag_record(enum zmk_keyball_diag_event_type type, int32_t a, int32_t b, int32_t c,
                             int32_t d) {
    record_event(type, a, b, c, d, true);
}

void zmk_keyball_diag_input(int16_t dx, int16_t dy, int16_t scroll_y, int16_t scroll_x) {
    if (atomic_get(&capture_frozen)) {
        return;
    }
    uint32_t timestamp = now_us();
    uint32_t previous = (uint32_t)atomic_set(&last_input_us, (atomic_val_t)timestamp);
    if (previous != 0U) {
        atomic_update_max(&max_input_gap_us, timestamp - previous);
    }
    zmk_keyball_diag_record(ZMK_KEYBALL_DIAG_INPUT, dx, dy, scroll_y, scroll_x);
}

void zmk_keyball_diag_hog_enqueue(int16_t dx, int16_t dy, int16_t scroll_y, int16_t scroll_x) {
    ARG_UNUSED(dx);
    ARG_UNUSED(dy);
    ARG_UNUSED(scroll_y);
    ARG_UNUSED(scroll_x);
    if (atomic_get(&capture_frozen)) {
        return;
    }
    atomic_set(&hog_started_us, (atomic_val_t)now_us());
    atomic_inc(&counters[ZMK_KEYBALL_DIAG_HOG_ENQUEUE]);
}

void zmk_keyball_diag_hog_result(int result, bool connected) {
    if (atomic_get(&capture_frozen)) {
        return;
    }
    uint32_t elapsed = now_us() - (uint32_t)atomic_get(&hog_started_us);
    atomic_inc(&counters[ZMK_KEYBALL_DIAG_HOG_RESULT]);
    atomic_update_max(&max_hog_duration_us, elapsed);
    if (!connected) {
        atomic_inc(&hog_disconnected);
    }
    if (result != 0 || !connected || elapsed > DIAG_SLOW_HOG_US) {
        record_event(ZMK_KEYBALL_DIAG_HOG_RESULT, result, connected, (int32_t)elapsed, 0, false);
    }
}

void zmk_keyball_diag_schedule_due(int64_t delay_ms) {
    if (atomic_get(&capture_frozen)) {
        return;
    }
    uint32_t due = now_us() + (uint32_t)MAX(delay_ms, 0) * 1000U;
    atomic_set(&scheduled_due_us, (atomic_val_t)due);
    atomic_inc(&counters[ZMK_KEYBALL_DIAG_SCHEDULE]);
}

void zmk_keyball_diag_schedule_result(int64_t delay_ms, int result) {
    if (atomic_get(&capture_frozen)) {
        return;
    }
    atomic_set(&schedule_last_error, result);
    if (result < 0) {
        atomic_inc(&schedule_errors);
        record_event(ZMK_KEYBALL_DIAG_SCHEDULE, (int32_t)delay_ms, result,
                     atomic_get(&scheduled_due_us), 0, false);
    }
}

void zmk_keyball_diag_work(void) {
    if (atomic_get(&capture_frozen)) {
        return;
    }
    uint32_t timestamp = now_us();
    uint32_t due = (uint32_t)atomic_get(&scheduled_due_us);
    int32_t late = due == 0U ? 0 : (int32_t)(timestamp - due);
    if (late > 0) {
        atomic_update_max(&max_work_late_us, (uint32_t)late);
    }
    atomic_inc(&counters[ZMK_KEYBALL_DIAG_WORK]);
    if (late > (int32_t)DIAG_SLOW_WORK_US) {
        record_event(ZMK_KEYBALL_DIAG_WORK, (int32_t)due, late, 0, 0, false);
    }
}

uint32_t zmk_keyball_diag_notify_begin(int16_t dx, int16_t dy) {
    if (atomic_get(&capture_frozen)) {
        return 0;
    }
    uint32_t id = (uint32_t)atomic_inc(&next_notify_id) + 1U;
    uint32_t timestamp = now_us();
    uint32_t previous = (uint32_t)atomic_set(&last_notify_us, (atomic_val_t)timestamp);
    if (previous != 0U) {
        atomic_update_max(&max_notify_gap_us, timestamp - previous);
    }
    atomic_set(&notify_started_us[id % DIAG_NOTIFY_SLOTS], (atomic_val_t)timestamp);
    atomic_set(&notify_slot_ids[id % DIAG_NOTIFY_SLOTS], (atomic_val_t)id);
    uint32_t packed_delta = (uint32_t)(uint16_t)dx | ((uint32_t)(uint16_t)dy << 16);
    atomic_set(&notify_deltas[id % DIAG_NOTIFY_SLOTS], (atomic_val_t)packed_delta);
    uint32_t outstanding = (uint32_t)atomic_inc(&notify_outstanding) + 1U;
    atomic_update_max(&max_notify_outstanding, outstanding);
    atomic_inc(&counters[ZMK_KEYBALL_DIAG_NOTIFY_CALL]);
    return id;
}

void zmk_keyball_diag_notify_result(uint32_t id, int result) {
    if (id == 0U || atomic_get(&capture_frozen)) {
        return;
    }
    uint32_t started = (uint32_t)atomic_get(&notify_started_us[id % DIAG_NOTIFY_SLOTS]);
    uint32_t elapsed = now_us() - started;
    uint32_t packed_delta = (uint32_t)atomic_get(&notify_deltas[id % DIAG_NOTIFY_SLOTS]);
    atomic_inc(&counters[ZMK_KEYBALL_DIAG_NOTIFY_RETURN]);
    atomic_update_max(&max_notify_return_us, elapsed);
    if (result != 0) {
        if ((uint32_t)atomic_get(&notify_slot_ids[id % DIAG_NOTIFY_SLOTS]) == id) {
            atomic_set(&notify_slot_ids[id % DIAG_NOTIFY_SLOTS], 0);
            atomic_set(&notify_deltas[id % DIAG_NOTIFY_SLOTS], 0);
            atomic_dec_nonnegative(&notify_outstanding);
        } else {
            atomic_inc(&notify_unmatched);
        }
    }
    if (result != 0 || elapsed > DIAG_SLOW_NOTIFY_RETURN_US) {
        record_event(ZMK_KEYBALL_DIAG_NOTIFY_RETURN, (int32_t)id, result, (int32_t)elapsed,
                     (int32_t)packed_delta, false);
    }
}

void zmk_keyball_diag_notify_complete(uint32_t id) {
    if (id == 0U || atomic_get(&capture_frozen)) {
        return;
    }
    if ((uint32_t)atomic_get(&notify_slot_ids[id % DIAG_NOTIFY_SLOTS]) != id) {
        atomic_inc(&notify_unmatched);
        return;
    }
    uint32_t packed_delta = (uint32_t)atomic_get(&notify_deltas[id % DIAG_NOTIFY_SLOTS]);
    atomic_set(&notify_slot_ids[id % DIAG_NOTIFY_SLOTS], 0);
    atomic_set(&notify_deltas[id % DIAG_NOTIFY_SLOTS], 0);
    uint32_t started = (uint32_t)atomic_get(&notify_started_us[id % DIAG_NOTIFY_SLOTS]);
    uint32_t elapsed = now_us() - started;
    atomic_update_max(&max_notify_complete_us, elapsed);
    atomic_dec_nonnegative(&notify_outstanding);
    zmk_keyball_diag_record(ZMK_KEYBALL_DIAG_NOTIFY_COMPLETE, (int32_t)id, (int32_t)elapsed,
                            atomic_get(&notify_outstanding), (int32_t)packed_delta);
}

void zmk_keyball_diag_split_rx(int slot, uint16_t length) {
    if (atomic_get(&capture_frozen)) {
        return;
    }

    uint32_t timestamp = now_us();
    uint32_t previous = (uint32_t)atomic_set(&split_last_rx_us, (atomic_val_t)timestamp);
    uint32_t gap = previous == 0U ? 0U : timestamp - previous;
    uint32_t count = (uint32_t)atomic_inc(&split_rx_count) + 1U;
    bool first_for_link = atomic_cas(&split_rx_ready, 0, 1);
    atomic_set(&split_last_slot, slot);
    atomic_inc(&counters[ZMK_KEYBALL_DIAG_SPLIT_RX]);
    if (previous != 0U) {
        atomic_update_max(&split_max_rx_gap_us, gap);
    }

    if (count == 1U || first_for_link || (count % DIAG_SPLIT_RX_SAMPLE_INTERVAL) == 0U ||
        gap >= DIAG_SPLIT_RX_LONG_GAP_US) {
        record_event(ZMK_KEYBALL_DIAG_SPLIT_RX, slot, length, (int32_t)gap, (int32_t)count, false);
    }
    if (count == 1U || first_for_link) {
        record_transition_only(ZMK_KEYBALL_DIAG_SPLIT_RX, slot, length, (int32_t)gap,
                               (int32_t)count);
    }
}

static void reset_capture(bool reset_link_state) {
    atomic_set(&capture_frozen, 1);
    memset(events, 0, sizeof(events));
    memset(transition_events, 0, sizeof(transition_events));
    memset(counters, 0, sizeof(counters));
    memset(notify_started_us, 0, sizeof(notify_started_us));
    memset(notify_slot_ids, 0, sizeof(notify_slot_ids));
    memset(notify_deltas, 0, sizeof(notify_deltas));
    atomic_set(&next_sequence, 0);
    atomic_set(&next_transition_sequence, 0);
    atomic_set(&last_input_us, 0);
    atomic_set(&max_input_gap_us, 0);
    atomic_set(&hog_started_us, 0);
    atomic_set(&max_hog_duration_us, 0);
    atomic_set(&hog_disconnected, 0);
    atomic_set(&last_notify_us, 0);
    atomic_set(&max_notify_gap_us, 0);
    atomic_set(&scheduled_due_us, 0);
    atomic_set(&max_work_late_us, 0);
    atomic_set(&next_notify_id, 0);
    atomic_set(&notify_outstanding, 0);
    atomic_set(&max_notify_outstanding, 0);
    atomic_set(&max_notify_complete_us, 0);
    atomic_set(&max_notify_return_us, 0);
    atomic_set(&notify_unmatched, 0);
    atomic_set(&notify_underflows, 0);
    atomic_set(&schedule_errors, 0);
    atomic_set(&schedule_last_error, 0);
    atomic_set(&split_service_found_count, 0);
    atomic_set(&split_scan_retry_count, 0);
    atomic_set(&split_rx_count, 0);
    atomic_set(&split_last_rx_us, 0);
    atomic_set(&split_max_rx_gap_us, 0);
    if (reset_link_state) {
        atomic_set(&host_connected, 0);
        atomic_set(&host_last_connect_error, 0);
        atomic_set(&host_last_disconnect_reason, 0);
        atomic_set(&host_security_level, 0);
        atomic_set(&host_security_error, 0);
        atomic_set(&host_interval, 0);
        atomic_set(&host_latency, 0);
        atomic_set(&host_timeout, 0);
        atomic_set(&split_enabled, 0);
        atomic_set(&split_scanning, 0);
        atomic_set(&split_connections, 0);
        atomic_set(&split_available, 0);
        atomic_set(&split_last_scan_error, 0);
        atomic_set(&split_last_create_error, 0);
        atomic_set(&split_last_connect_error, 0);
        atomic_set(&split_last_disconnect_reason, 0);
        atomic_set(&split_last_security_error, 0);
        atomic_set(&split_interval, 0);
        atomic_set(&split_latency, 0);
        atomic_set(&split_timeout, 0);
        atomic_set(&split_last_discovery_error, 0);
        atomic_set(&split_last_subscribe_error, 0);
        atomic_set(&split_ready, 0);
        atomic_set(&split_rx_ready, 0);
        atomic_set(&split_last_slot, -1);
    }
    atomic_set(&dump_complete, 0);
    atomic_set(&capture_frozen, 0);
    zmk_keyball_diag_record(ZMK_KEYBALL_DIAG_BOOT, 1, 0, 0, 0);
}

static bool serial_link_ready(void) {
    uint32_t dtr = 0;
    return atomic_get(&usb_present) &&
           uart_line_ctrl_get(diag_uart, UART_LINE_CTRL_DTR, &dtr) == 0 && dtr != 0U;
}

static bool serial_write(const char *text) {
    size_t remaining = strlen(text);
    while (remaining > 0U) {
        if (!serial_link_ready()) {
            return false;
        }

        int written = uart_fifo_fill(diag_uart, (const uint8_t *)text, remaining);
        if (written <= 0) {
            k_sleep(K_MSEC(1));
            continue;
        }

        text += written;
        remaining -= written;
    }
    return true;
}

static bool serial_line(const char *format, ...) {
    char line[DIAG_LINE_SIZE];
    va_list args;
    va_start(args, format);
    int length = vsnprintk(line, sizeof(line) - 3, format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= sizeof(line) - 3U) {
        return false;
    }
    size_t end = (size_t)length;
    line[end++] = '\r';
    line[end++] = '\n';
    line[end] = '\0';
    return serial_write(line);
}

static bool dump_capture(void) {
    uint32_t total = (uint32_t)atomic_get(&next_sequence);
    uint32_t first = total > DIAG_EVENT_CAPACITY ? total - DIAG_EVENT_CAPACITY + 1U : 1U;
    uint32_t transition_total = (uint32_t)atomic_get(&next_transition_sequence);
    uint32_t transition_first = transition_total > DIAG_TRANSITION_CAPACITY
                                    ? transition_total - DIAG_TRANSITION_CAPACITY + 1U
                                    : 1U;

    if (!serial_line("KEYBALL_DIAG_BEGIN,2")) {
        return false;
    }
    if (!serial_line("META,uptime_ms,%lld,interval_ms,%d,event_capacity,%d,"
                     "transition_capacity,%d,cpi,%d",
                     (long long)k_uptime_get(), CONFIG_ZMK_BLE_MOUSE_REPORT_INTERVAL_MS,
                     DIAG_EVENT_CAPACITY, DIAG_TRANSITION_CAPACITY, CONFIG_PMW3610_CPI)) {
        return false;
    }
    if (!serial_line(
            "SUMMARY,total,%u,first,%u,overwritten,%u,input_gap_max_us,%u,"
            "hog_duration_max_us,%u,hog_disconnected,%d,notify_gap_max_us,%u,"
            "notify_return_max_us,%u,work_late_max_us,%u,notify_complete_max_us,%u,"
            "outstanding,%d,outstanding_max,%d,notify_unmatched,%d,"
            "notify_underflows,%d,schedule_errors,%d,schedule_last_error,%d",
            total, first, total > DIAG_EVENT_CAPACITY ? total - DIAG_EVENT_CAPACITY : 0,
            (uint32_t)atomic_get(&max_input_gap_us), (uint32_t)atomic_get(&max_hog_duration_us),
            atomic_get(&hog_disconnected), (uint32_t)atomic_get(&max_notify_gap_us),
            (uint32_t)atomic_get(&max_notify_return_us), (uint32_t)atomic_get(&max_work_late_us),
            (uint32_t)atomic_get(&max_notify_complete_us), atomic_get(&notify_outstanding),
            atomic_get(&max_notify_outstanding), atomic_get(&notify_unmatched),
            atomic_get(&notify_underflows), atomic_get(&schedule_errors),
            atomic_get(&schedule_last_error))) {
        return false;
    }
    if (!serial_line("HOST_STATE,connected,%d,last_connect_error,%d,last_disconnect_reason,%d,"
                     "security_level,%d,security_error,%d,interval,%d,latency,%d,timeout,%d",
                     atomic_get(&host_connected), atomic_get(&host_last_connect_error),
                     atomic_get(&host_last_disconnect_reason), atomic_get(&host_security_level),
                     atomic_get(&host_security_error), atomic_get(&host_interval),
                     atomic_get(&host_latency), atomic_get(&host_timeout))) {
        return false;
    }
    if (!serial_line(
            "SPLIT_STATE,enabled,%d,scanning,%d,connections,%d,available,%d,"
            "last_scan_error,%d,last_create_error,%d,last_connect_error,%d,"
            "last_disconnect_reason,%d,last_security_error,%d,last_discovery_error,%d,"
            "last_subscribe_error,%d,gatt_ready,%d,last_slot,%d,"
            "service_found_count,%d,scan_retry_count,%d",
            atomic_get(&split_enabled), atomic_get(&split_scanning), atomic_get(&split_connections),
            atomic_get(&split_available), atomic_get(&split_last_scan_error),
            atomic_get(&split_last_create_error), atomic_get(&split_last_connect_error),
            atomic_get(&split_last_disconnect_reason), atomic_get(&split_last_security_error),
            atomic_get(&split_last_discovery_error), atomic_get(&split_last_subscribe_error),
            atomic_get(&split_ready), atomic_get(&split_last_slot),
            atomic_get(&split_service_found_count), atomic_get(&split_scan_retry_count))) {
        return false;
    }
    if (!serial_line("SPLIT_RX_STATE,functional_ready,%d,count,%d,last_us,%u,max_gap_us,%u",
                     atomic_get(&split_rx_ready), atomic_get(&split_rx_count),
                     (uint32_t)atomic_get(&split_last_rx_us),
                     (uint32_t)atomic_get(&split_max_rx_gap_us))) {
        return false;
    }
    if (!serial_line("SPLIT_LINK_STATE,interval,%d,latency,%d,timeout,%d",
                     atomic_get(&split_interval), atomic_get(&split_latency),
                     atomic_get(&split_timeout))) {
        return false;
    }

    for (int type = 1; type < ZMK_KEYBALL_DIAG_EVENT_COUNT; type++) {
        if (!serial_line("COUNT,%d,%s,%d", type, event_names[type], atomic_get(&counters[type]))) {
            return false;
        }
    }

    if (!serial_line("STATE_SUMMARY,total,%u,first,%u,overwritten,%u", transition_total,
                     transition_first,
                     transition_total > DIAG_TRANSITION_CAPACITY
                         ? transition_total - DIAG_TRANSITION_CAPACITY
                         : 0)) {
        return false;
    }
    if (!serial_line("STATE_FIELDS,sequence,timestamp_us,type,name,a,b,c,d")) {
        return false;
    }
    for (uint32_t sequence = transition_first; sequence <= transition_total; sequence++) {
        const struct diag_event *event =
            &transition_events[(sequence - 1U) & (DIAG_TRANSITION_CAPACITY - 1U)];
        if ((uint32_t)atomic_get(&event->sequence) != sequence) {
            if (!serial_line("STATE_LOST,%u", sequence)) {
                return false;
            }
            continue;
        }
        if (!serial_line("STATE_EVENT,%u,%u,%u,%s,%d,%d,%d,%d", sequence, event->timestamp_us,
                         event->type, event_names[event->type], event->a, event->b, event->c,
                         event->d)) {
            return false;
        }
    }

    if (!serial_line("FIELDS,sequence,timestamp_us,type,name,a,b,c,d")) {
        return false;
    }
    for (uint32_t sequence = first; sequence <= total; sequence++) {
        const struct diag_event *event = &events[(sequence - 1U) % DIAG_EVENT_CAPACITY];
        if ((uint32_t)atomic_get(&event->sequence) != sequence) {
            if (!serial_line("LOST,%u", sequence)) {
                return false;
            }
            continue;
        }
        if (!serial_line("EVENT,%u,%u,%u,%s,%d,%d,%d,%d", sequence, event->timestamp_us,
                         event->type, event_names[event->type], event->a, event->b, event->c,
                         event->d)) {
            return false;
        }
    }
    return serial_line("KEYBALL_DIAG_END");
}

static void dump_thread(void *p1, void *p2, void *p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    while (true) {
        k_sem_take(&dump_request, K_FOREVER);
        while (atomic_get(&usb_present)) {
            uint32_t dtr = 0;
            while (atomic_get(&usb_present) &&
                   (uart_line_ctrl_get(diag_uart, UART_LINE_CTRL_DTR, &dtr) != 0 || dtr == 0U)) {
                k_sleep(K_MSEC(100));
            }
            if (!atomic_get(&usb_present)) {
                break;
            }

            /* Hold the shared Studio UART for the complete DTR-open extraction session. */
            atomic_set(&uart_claimed, 1);
            uart_irq_tx_disable(diag_uart);
            uart_irq_rx_disable(diag_uart);
            k_sleep(K_MSEC(20));
            bool complete = dump_capture();
            if (complete && serial_link_ready()) {
                /* CDC ACM has no drain API. Allow its TX ring and final USB transfer to empty. */
                k_sleep(K_MSEC(500));
            }
            atomic_set(&dump_complete, complete);

            /* A DTR close/reopen retries the same frozen capture, even after a complete attempt. */
            while (atomic_get(&usb_present) && serial_link_ready()) {
                k_sleep(K_MSEC(20));
            }
            atomic_set(&uart_claimed, 0);
            uart_irq_rx_enable(diag_uart);
        }
    }
}

bool zmk_keyball_diag_uart_claimed(void) { return atomic_get(&uart_claimed) != 0; }

K_THREAD_DEFINE(keyball_diag_dump_thread, 2048, dump_thread, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

static int diag_event_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *activity = as_zmk_activity_state_changed(eh);
    if (activity != NULL) {
        zmk_keyball_diag_record(ZMK_KEYBALL_DIAG_ACTIVITY, activity->state, 0, 0, 0);
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct zmk_usb_conn_state_changed *usb = as_zmk_usb_conn_state_changed(eh);
    if (usb == NULL) {
        return -ENOTSUP;
    }

    if (usb->conn_state == ZMK_USB_CONN_NONE) {
        atomic_set(&usb_present, 0);
        if (atomic_get(&capture_frozen) || atomic_get(&dump_complete)) {
            reset_capture(false);
        }
        zmk_keyball_diag_record(ZMK_KEYBALL_DIAG_USB, usb->conn_state, 0, 0, 0);
    } else if (!atomic_get(&capture_frozen)) {
        zmk_keyball_diag_record(ZMK_KEYBALL_DIAG_USB, usb->conn_state, 0, 0, 0);
        atomic_set(&usb_present, 1);
        atomic_set(&capture_frozen, 1);
        k_sem_give(&dump_request);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(keyball_diag, diag_event_listener);
ZMK_SUBSCRIPTION(keyball_diag, zmk_activity_state_changed);
ZMK_SUBSCRIPTION(keyball_diag, zmk_usb_conn_state_changed);

static int keyball_diag_init(void) {
    if (!device_is_ready(diag_uart)) {
        return -ENODEV;
    }
    reset_capture(true);
    return 0;
}

SYS_INIT(keyball_diag_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
