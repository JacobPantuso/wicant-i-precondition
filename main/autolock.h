// Disconnect auto-lock for the Numen firmware; see autolock.c.
//
// The myIONIQ app decides walk-away locks from harness RSSI, but executes
// them over the same BLE link that just died — in a concrete hallway with
// no cell service the lock fails exactly where it fires. The WiCAN is in
// the car, on the bus, so it locks on the disconnect itself.
//
// Contract with the app (docs/plans/wican-disconnect-auto-lock.md):
//   ATXAL        query -> XAL:<armed>,<last>,<count>
//   ATXAL0|1     disable | enable (persists; default enabled)
#pragma once

#include <stdbool.h>
#include <stddef.h>

// NVS + state. Call once after nvs_flash_init().
void autolock_init(void);

// BLE session edges, from ble.c. Only the key-holder's phone arms the
// logic; with a single client slot that means any client, gated on a key
// being stored.
void autolock_note_connected(void);
void autolock_note_disconnected(void);

// 40 ms housekeeping from the main precondition tick: READY-edge arming,
// the disconnect hold timer, and burst verification.
void autolock_tick(void);

// ATXAL handler for elm327.c. Fills `response` and returns true for the
// query and both sets; returns false for anything else (the caller
// answers '?').
bool autolock_command(const char *args, char *response, size_t response_len);
