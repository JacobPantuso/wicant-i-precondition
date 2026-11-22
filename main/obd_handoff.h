// Experimental direct BLE ELM departure checks. No vehicle commands are sent
// until VIN and fresh safety status have been validated.
#pragma once
#include <stdbool.h>
#include <stddef.h>
#ifdef OBD_HANDOFF_HOST_TEST
#include "obd_handoff_test_platform.h"
#else
#include "esp_gap_ble_api.h"
#endif
void obd_handoff_init(void);
void obd_handoff_gap(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
bool obd_handoff_command(const char *args, char *response, size_t capacity);
bool obd_handoff_blocks(void);
void obd_handoff_tick(void);
void obd_handoff_stop(void);

// Wait for the bounded worker to stop before tearing down the BLE host.
bool obd_handoff_shutdown(void);
