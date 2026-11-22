// Parked BLE standby (experimental branch). See standby.c and
// docs/plans/wican-parked-ble-standby-plan-2026-11-16.md in the app repo.
//
// After a departure lock WiCAN stays reachable over BLE for the phone's
// approach unlock, with no OBD adapter link and no vehicle queries, for at
// most 48 h or until the 12 V battery stays under the floor for 10 min. Then it falls back
// to the existing protective sleep (BLE off).
//
// Contract with the app (BLE only):
//   ATXPK        query -> XPK:<en>,<state>,<reason>,<since_s>,<budget_s>,
//                <vbat_mV>,<floor_mV>,<home>,<lease>,<pm>,<reset>
//                state: active | parked | protect; vbat -1 = no reading
//   ATXPKE1|0    standby on | off. Volatile, off at boot, like ATXAUm:
//                the app reapplies it on every connect.
//   ATXPKH1|0    the car is parked inside the owner's Home zone, where the
//                app never approach-unlocks | not. Parked only: slows RSSI
//                sampling to 2 s. Grants nothing. Any disconnect clears it;
//                the app resends it on connect. (It no longer asks for a slow
//                link: the phone's commands must stay prompt.)
//   ATXPKP1|0    reserved: power management is not built in (2026-11-16)
//   ATXPKV<mV>   battery floor, 11000-12800 (volatile; default 11600)
//   ATXPKX       leave parked standby now: the owner asked the phone to
//                resume its OBD adapter. Grants no lease; the next drive does.
//   $XPK:...     pushed on every state change while the phone is connected
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    STANDBY_ACTIVE = 0,
    STANDBY_PARKED,
    STANDBY_PROTECT,
} standby_state_t;

void standby_set_tx_queue(QueueHandle_t *queue);

// Vehicle and lock lifecycle, from autolock.c / obd_handoff.c.
void standby_note_drive(void);                 // an on session (READY or the app's ATXALP1)
void standby_note_off(void);                   // fresh off after that session
void standby_note_locked(const char *why);     // a departure lock: WiCAN's or the owner's
void standby_note_unlocked(void);              // an accepted approach unlock

// One OBD adapter lease per departure. A phone reconnect never grants one.
bool standby_lease_allowed(void);
void standby_note_lease_started(void);
// touched_vehicle: the lease got as far as sending diagnostic requests.
void standby_note_lease_ended(bool touched_vehicle);

// Phone link, from ble.c via autolock.c.
void standby_note_connected(void);
void standby_note_disconnected(void);

// Battery, from sleep_mode.c's ADC task (about once a second).
void standby_note_voltage(float volts, bool valid);

// From autolock_tick (40 ms). Applies transitions and their side effects.
void standby_tick(void);

standby_state_t standby_state(void);
bool standby_holds_awake(void);        // parked: suppress the voltage sleep timer
bool standby_protect_requested(void);  // budget or battery: sleep now, BLE off
int64_t standby_rssi_period_us(int64_t normal_us);

bool standby_command(const char *args, char *response, size_t response_len);

// Platform side effects (standby_platform.c on the device, stubs in tests).
void standby_platform_apply(standby_state_t state, bool pm_allowed);
int standby_platform_reset_reason(void);
