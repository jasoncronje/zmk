/*
 * Keyball Bluetooth diagnostic capture.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/sys/util.h>
#include <zephyr/types.h>

enum zmk_keyball_diag_event_type {
    ZMK_KEYBALL_DIAG_BOOT = 1,
    ZMK_KEYBALL_DIAG_ACTIVITY,
    ZMK_KEYBALL_DIAG_USB,
    ZMK_KEYBALL_DIAG_INPUT,
    ZMK_KEYBALL_DIAG_HOG_ENQUEUE,
    ZMK_KEYBALL_DIAG_HOG_RESULT,
    ZMK_KEYBALL_DIAG_SCHEDULE,
    ZMK_KEYBALL_DIAG_WORK,
    ZMK_KEYBALL_DIAG_NOTIFY_CALL,
    ZMK_KEYBALL_DIAG_NOTIFY_RETURN,
    ZMK_KEYBALL_DIAG_NOTIFY_COMPLETE,
    ZMK_KEYBALL_DIAG_HOST_CONNECTED,
    ZMK_KEYBALL_DIAG_HOST_DISCONNECTED,
    ZMK_KEYBALL_DIAG_HOST_SECURITY,
    ZMK_KEYBALL_DIAG_HOST_PARAMS,
    ZMK_KEYBALL_DIAG_SPLIT_SCAN_RETRY,
    ZMK_KEYBALL_DIAG_SPLIT_SCAN_START,
    ZMK_KEYBALL_DIAG_SPLIT_SCAN_STOP,
    ZMK_KEYBALL_DIAG_SPLIT_SERVICE_FOUND,
    ZMK_KEYBALL_DIAG_SPLIT_CREATE,
    ZMK_KEYBALL_DIAG_SPLIT_CONNECTED,
    ZMK_KEYBALL_DIAG_SPLIT_DISCONNECTED,
    ZMK_KEYBALL_DIAG_SPLIT_SECURITY,
    ZMK_KEYBALL_DIAG_SPLIT_PARAMS,
    ZMK_KEYBALL_DIAG_SPLIT_DISCOVER,
    ZMK_KEYBALL_DIAG_SPLIT_SUBSCRIBE,
    ZMK_KEYBALL_DIAG_SPLIT_SUBSCRIBE_COMPLETE,
    ZMK_KEYBALL_DIAG_SPLIT_READY,
    ZMK_KEYBALL_DIAG_SPLIT_RX,
    ZMK_KEYBALL_DIAG_SPLIT_ENABLED,
    ZMK_KEYBALL_DIAG_SPLIT_STATUS,
    ZMK_KEYBALL_DIAG_EVENT_COUNT,
};

#if IS_ENABLED(CONFIG_ZMK_KEYBALL_DIAGNOSTICS)

void zmk_keyball_diag_record(enum zmk_keyball_diag_event_type type, int32_t a, int32_t b, int32_t c,
                             int32_t d);
void zmk_keyball_diag_input(int16_t dx, int16_t dy, int16_t scroll_y, int16_t scroll_x);
void zmk_keyball_diag_hog_enqueue(int16_t dx, int16_t dy, int16_t scroll_y, int16_t scroll_x);
void zmk_keyball_diag_hog_result(int result, bool connected);
void zmk_keyball_diag_schedule_due(int64_t delay_ms);
void zmk_keyball_diag_schedule_result(int64_t delay_ms, int result);
void zmk_keyball_diag_work(void);
uint32_t zmk_keyball_diag_notify_begin(int16_t dx, int16_t dy);
void zmk_keyball_diag_notify_result(uint32_t id, int result);
void zmk_keyball_diag_notify_complete(uint32_t id);
void zmk_keyball_diag_split_rx(int slot, uint16_t length);
bool zmk_keyball_diag_uart_claimed(void);

#else

#define zmk_keyball_diag_record(...)                                                               \
    do {                                                                                           \
    } while (false)
#define zmk_keyball_diag_input(...)                                                                \
    do {                                                                                           \
    } while (false)
#define zmk_keyball_diag_hog_enqueue(...)                                                          \
    do {                                                                                           \
    } while (false)
#define zmk_keyball_diag_hog_result(...)                                                           \
    do {                                                                                           \
    } while (false)
#define zmk_keyball_diag_schedule_due(...)                                                         \
    do {                                                                                           \
    } while (false)
#define zmk_keyball_diag_schedule_result(...)                                                      \
    do {                                                                                           \
    } while (false)
#define zmk_keyball_diag_work(...)                                                                 \
    do {                                                                                           \
    } while (false)
static inline uint32_t zmk_keyball_diag_notify_begin(int16_t dx, int16_t dy) {
    ARG_UNUSED(dx);
    ARG_UNUSED(dy);
    return 0;
}
#define zmk_keyball_diag_notify_result(...)                                                        \
    do {                                                                                           \
    } while (false)
#define zmk_keyball_diag_notify_complete(...)                                                      \
    do {                                                                                           \
    } while (false)
#define zmk_keyball_diag_split_rx(...)                                                             \
    do {                                                                                           \
    } while (false)
static inline bool zmk_keyball_diag_uart_claimed(void) { return false; }

#endif
