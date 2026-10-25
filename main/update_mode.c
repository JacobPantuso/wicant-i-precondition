/*
 * Update mode; see update_mode.h.
 */
#include "update_mode.h"

#include <stdint.h>
#include <stdio.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"

#define UPDATE_MODE_MAGIC   0x55504454u                 // "UPDT"
#define UPDATE_WINDOW_US    (5LL * 60 * 1000 * 1000)

static const char *TAG = "update_mode";

// RTC memory survives esp_restart() but not a power cut, so a WiCAN unplugged
// mid-update simply comes back in normal mode. Not heap: costs the CAN tasks
// nothing.
static RTC_NOINIT_ATTR uint32_t s_magic;
static RTC_NOINIT_ATTR char s_pass[UPDATE_MODE_PASS_LEN + 1];

static bool s_active;
static esp_timer_handle_t s_reboot_timer;

static void reboot_cb(void *arg)
{
    (void)arg;
    esp_restart();
}

static void schedule_reboot(int64_t after_us)
{
    if (s_reboot_timer == NULL)
    {
        const esp_timer_create_args_t args = { .callback = reboot_cb, .name = "update_reboot" };
        if (esp_timer_create(&args, &s_reboot_timer) != ESP_OK)
        {
            esp_restart();
        }
    }
    esp_timer_stop(s_reboot_timer);
    esp_timer_start_once(s_reboot_timer, after_us);
}

const char *update_mode_request(void)
{
    // No 0/o, 1/l/i: the password is never typed, but it shows in logs.
    static const char alphabet[] = "abcdefghjkmnpqrstuvwxyz23456789";
    for (int i = 0; i < UPDATE_MODE_PASS_LEN; i++)
    {
        s_pass[i] = alphabet[esp_random() % (sizeof(alphabet) - 1)];
    }
    s_pass[UPDATE_MODE_PASS_LEN] = 0;
    s_magic = UPDATE_MODE_MAGIC;
    ESP_LOGW(TAG, "update mode requested; rebooting");
    schedule_reboot(500 * 1000);
    return s_pass;
}

void update_mode_ap_ssid(char *out, size_t len)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(out, len, "WiCAN_%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool update_mode_begin(void)
{
    s_active = (s_magic == UPDATE_MODE_MAGIC);
    // One boot only: the reboot after an upload, or after the timeout, is a
    // normal one.
    s_magic = 0;
    if (s_active)
    {
        s_pass[UPDATE_MODE_PASS_LEN] = 0;
        ESP_LOGW(TAG, "update mode: BLE off, access point up for 5 minutes");
        schedule_reboot(UPDATE_WINDOW_US);
    }
    return s_active;
}

bool update_mode_active(void)
{
    return s_active;
}

const char *update_mode_password(void)
{
    return s_active ? s_pass : NULL;
}

void update_mode_hold(void)
{
    if (s_active && s_reboot_timer != NULL)
    {
        esp_timer_stop(s_reboot_timer);
    }
}
