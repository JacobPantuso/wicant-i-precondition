/*
 * Firmware update over BLE, for the Numen app.
 *
 * With the car on, the head unit keeps the phone on its own Wi-Fi for
 * wireless CarPlay, so the access-point update (update_mode.c) can't hold
 * its connection. This takes the image over the BLE link the app already
 * has instead:
 *
 *   ATXOB          -> XOB:1          the command exists (the app's probe)
 *   ATXOB<size>    -> XOB:READY      or XOB:ERR,<reason>
 *   <size> bytes   written to the command characteristic, with response,
 *                  so the link paces itself
 *   ATXOBEND       -> XOB:DONE       image checked and set to boot; reboots
 *                                    500 ms later
 *                  -> XOB:ERR,IMAGE  the image failed its check
 *
 * A write that fails mid-transfer sends $XOB:ERR,<reason> at once. A dropped
 * link abandons the transfer. The image goes through esp_ota_* as the web
 * page's upload does (config_server.c), so esp_ota_end checks it before it
 * can boot and main.c's rollback still applies. Nothing here is heap until
 * a transfer starts.
 */
#include "ble_ota.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "types.h"

static const char *TAG = "ble_ota";

static QueueHandle_t *s_tx_queue;
static esp_ota_handle_t s_handle;
static const esp_partition_t *s_partition;
static uint32_t s_expected;
static uint32_t s_received;
// Image bytes expected. Set by ble_ota_command (can_tx_task) once the handle
// is open, cleared by the BTC task on the last byte, a failed write or a drop.
static volatile bool s_active;
// Every byte is written and the handle is open, waiting for ATXOBEND.
static volatile bool s_complete;
static esp_timer_handle_t s_reboot_timer;

void ble_ota_set_tx_queue(QueueHandle_t *queue)
{
	s_tx_queue = queue;
}

bool ble_ota_active(void)
{
	return s_active;
}

static void ble_ota_notify(const char *line)
{
	static xdev_buffer buffer;
	if (s_tx_queue == NULL || *s_tx_queue == NULL) return;
	int len = snprintf((char *)buffer.ucElement, sizeof(buffer.ucElement), "$%s\r", line);
	if (len <= 0) return;
	if (len >= (int)sizeof(buffer.ucElement)) len = sizeof(buffer.ucElement) - 1;
	buffer.usLen = len;
	buffer.dev_channel = DEV_BLE;
	xQueueSend(*s_tx_queue, &buffer, pdMS_TO_TICKS(100));
}

static void reboot_cb(void *arg)
{
	(void)arg;
	esp_restart();
}

static void schedule_reboot(void)
{
	const esp_timer_create_args_t args = { .callback = reboot_cb, .name = "ble_ota_reboot" };
	if (s_reboot_timer == NULL && esp_timer_create(&args, &s_reboot_timer) != ESP_OK)
	{
		esp_restart();
	}
	// Long enough for the reply to go out first.
	esp_timer_start_once(s_reboot_timer, 500 * 1000);
}

static const char *ble_ota_begin(const char *size_str)
{
	char *end = NULL;
	unsigned long size = strtoul(size_str, &end, 10);
	if (end == size_str || *end != 0) return NULL;

	// A finished transfer nobody ended: let it go.
	if (s_complete)
	{
		esp_ota_abort(s_handle);
		s_complete = false;
	}
	s_partition = esp_ota_get_next_update_partition(NULL);
	if (s_partition == NULL || size < 1024 || size > s_partition->size)
	{
		return "XOB:ERR,SIZE";
	}
	// Sequential writes erase each sector as it's reached, instead of the
	// whole partition up front, which would hold up this task for seconds.
	esp_err_t err = esp_ota_begin(s_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
		return "XOB:ERR,BEGIN";
	}
	s_expected = size;
	s_received = 0;
	s_active = true;
	ESP_LOGW(TAG, "receiving %lu bytes", size);
	return "XOB:READY";
}

static const char *ble_ota_finish(void)
{
	if (!s_complete) return "XOB:ERR,INCOMPLETE";
	s_complete = false;
	// Reads the whole image back to check it: about a second.
	esp_err_t err = esp_ota_end(s_handle);
	if (err == ESP_OK)
	{
		err = esp_ota_set_boot_partition(s_partition);
	}
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "finishing failed (%s)", esp_err_to_name(err));
		return err == ESP_ERR_OTA_VALIDATE_FAILED ? "XOB:ERR,IMAGE" : "XOB:ERR,FINISH";
	}
	ESP_LOGW(TAG, "image accepted; rebooting");
	schedule_reboot();
	return "XOB:DONE";
}

const char *ble_ota_command(const char *args)
{
	if (args[0] == 0) return "XOB:1";
	if (strcmp(args, "end") == 0) return ble_ota_finish();
	return ble_ota_begin(args);
}

void ble_ota_feed(const uint8_t *data, uint16_t len)
{
	if (!s_active) return;
	if (len > s_expected - s_received)
	{
		esp_ota_abort(s_handle);
		s_active = false;
		ble_ota_notify("XOB:ERR,LENGTH");
		return;
	}
	esp_err_t err = esp_ota_write(s_handle, data, len);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
		esp_ota_abort(s_handle);
		s_active = false;
		ble_ota_notify("XOB:ERR,WRITE");
		return;
	}
	s_received += len;
	if (s_received == s_expected)
	{
		// Back to commands: the app sends ATXOBEND next.
		s_complete = true;
		s_active = false;
	}
}

void ble_ota_cancel(void)
{
	if (!s_active) return;
	ESP_LOGW(TAG, "link dropped after %lu of %lu bytes; abandoned",
			 (unsigned long)s_received, (unsigned long)s_expected);
	esp_ota_abort(s_handle);
	s_active = false;
}
