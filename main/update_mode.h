/*
 * Update mode: one boot with BLE off and the Wi-Fi access point up, so the
 * Numen app can upload firmware to /upload/ota.bin.
 *
 * Normally the first BLE connection turns Wi-Fi off for good (ble.c,
 * CONNECT_EVT), and a bonded phone reconnects the moment the WiCAN is in
 * range, so the access point is never there to update through. ATXOTA asks
 * for one boot without BLE instead.
 */
#ifndef __UPDATE_MODE_H__
#define __UPDATE_MODE_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UPDATE_MODE_PASS_LEN 12

// ATXOTA: picks a one-boot access-point password, schedules a reboot into
// update mode 500 ms out (time for the reply to reach the phone) and returns
// the password.
const char *update_mode_request(void);

// The access point's SSID, as wifi_network_init() names it.
void update_mode_ap_ssid(char *out, size_t len);

// Early in app_main: true when this boot is the one ATXOTA asked for. Arms a
// 5-minute timeout that reboots back to normal if no upload starts.
bool update_mode_begin(void);

bool update_mode_active(void);

// The one-boot password while in update mode, NULL otherwise.
const char *update_mode_password(void);

// An upload has started: cancel the timeout. The upload handler reboots
// when it finishes either way.
void update_mode_hold(void);

// How this boot went, from main.c's boot heap line. ATXHC reports it so the
// app can check a build it just installed.
typedef struct {
    bool can_tasks_ok;
    uint32_t free_before;     // heap before the CAN tasks
    uint32_t free_after;      // heap after them
    uint32_t largest_after;   // largest free block after them
} update_mode_boot_t;

void update_mode_note_boot(const update_mode_boot_t *boot);
void update_mode_get_boot(update_mode_boot_t *out);

#endif
