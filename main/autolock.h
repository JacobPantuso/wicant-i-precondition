// Walk-away auto-lock for the Numen firmware; see autolock.c.
//
// The WiCAN is in the car and always running, so it makes the walk-away
// decision itself: from the phone's link signal, and from the link dropping.
// The app can be suspended by iOS at any moment; this can't.
//
// Contract with the app (docs/plans/wican-disconnect-auto-lock.md):
//   ATXAL        query -> XAL:<armed>,<last>,<count>,<threshold>,<signal>
//                (signal: smoothed dBm, 0 when there's no reading)
//   ATXAL0|1     disable | enable (persists; default enabled)
//   ATXALT<dBm>  the walk-away line, -100 to -40 (persists; default -85)
//   ATXALB1|0    hold for 30 s | release: the app's OBD reading says a door
//                is open or someone is seated, which this bus can't see
//   $XAL:...     pushed after a lock, while the phone is connected
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// NVS + state. Call once after nvs_flash_init().
void autolock_init(void);

// The BLE TX queue, for the $XAL line after a lock. main.c, once at boot.
void autolock_set_tx_queue(QueueHandle_t *queue);

// BLE session edges and link RSSI readings, from ble.c.
void autolock_note_connected(void);
void autolock_note_disconnected(void);
void autolock_note_rssi(int rssi);

// From the main loop's 40 ms tick.
void autolock_tick(void);

// ATXAL<args>, with args the text after "xal".
bool autolock_command(const char *args, char *response, size_t response_len);
