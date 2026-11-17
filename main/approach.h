#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Experimental branch only. No standing unlock permission is stored.
void approach_set_tx_queue(QueueHandle_t *queue);
void approach_departed(void);
void approach_reset_session(void);
void approach_disconnected(void);
// Returns true only when a prior departure permits a short reconnect probe.
bool approach_connected(void);
// Raw, timestamped RSSI; this module owns its responsive approach filter.
void approach_sample(int signal, bool parked_off, int64_t now);
// How often autolock reads the phone's RSSI now; gaps and freshness scale with it.
void approach_set_sample_period(int64_t us);
bool approach_command(const char *args, char *response, size_t length);
