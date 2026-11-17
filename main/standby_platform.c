// Device side effects of parked BLE standby (see standby.c). Host tests
// stub these.
//
// Parked: Wi-Fi AP and web server off, power LED off, and, when the build
// has power management, dynamic frequency scaling down to the 40 MHz XTAL
// plus automatic light sleep where the build has BLE modem sleep and
// tickless idle. Drivers that hold PM locks (the TWAI driver while CAN is
// running) keep the chip out of light sleep; only measurement says what
// this saves. Active: back to a fixed CPU frequency and the power LED on.
// Wi-Fi is not restarted, as after any BLE connect (ble.c).

#include "standby.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#if CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#include "ble.h"
#include "config_server.h"
#include "hw_config.h"
#include "wifi_network.h"

static const char *TAG = "standby";

static void configure_pm(bool low)
{
#if CONFIG_PM_ENABLE
    esp_pm_config_t cfg = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = low ? 40 : CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE && CONFIG_BT_CTRL_MODEM_SLEEP
        .light_sleep_enable = low,
#else
        .light_sleep_enable = false,
#endif
    };
    esp_err_t err = esp_pm_configure(&cfg);
    ESP_LOGI(TAG, "power management %s: %s", low ? "low" : "full", esp_err_to_name(err));
#else
    (void)low;
#endif
}

void standby_platform_apply(standby_state_t state, bool pm_allowed)
{
    if (state == STANDBY_PARKED) {
        config_server_stop();
        wifi_network_deinit();
        gpio_set_level(PWR_LED_GPIO_NUM, PWR_LED_OFF);
        configure_pm(pm_allowed);
    } else {
        // PROTECT also returns to full power: sleep_mode.c takes it from here.
        configure_pm(false);
        gpio_set_level(PWR_LED_GPIO_NUM, PWR_LED_ON);
    }
}

int standby_platform_reset_reason(void)
{
    return (int)esp_reset_reason();
}
