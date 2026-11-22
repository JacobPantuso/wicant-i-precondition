// Parked BLE standby; see standby.h for the app contract.
//
// Lifecycle:
//   ACTIVE  - driving, or parked without a departure lock. Normal behaviour.
//   PARKED  - a departure lock happened (WiCAN's walk-away lock, or the owner
//             locking while WiCAN held the OBD lease). The OBD lease is over,
//             BLE stays up for the approach unlock, Wi-Fi and the power LED
//             are off, power management is on, and the voltage sleep timer
//             is held off. Ends on a new drive or an accepted approach unlock.
//   PROTECT - 48 h in PARKED, or the battery under 11.6 V for 10 min. Sleep
//             now with BLE off (sleep_mode.c); the next drive restarts WiCAN.
//
// The 48 h budget starts at PARKED entry and nothing but a drive or an
// accepted unlock starts it again: a phone reconnecting from the house,
// another lock event or link churn never extends it.
//
// OBD leases: one per departure. A drive's on edge or an accepted approach
// unlock grants one; starting a lease uses it. A lease that failed before
// sending any diagnostic request gives it back (at most 3 times), because it
// never touched the car. A phone reconnect never grants one.
//
// The battery reading counts as low when missing or stale: an ADC that stops
// reporting must not keep BLE up on a flat battery.

#include "standby.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "types.h"

#define BUDGET_US         (48LL * 3600 * 1000 * 1000)
#define FLOOR_DWELL_US    (10LL * 60 * 1000 * 1000)
#define VOLTAGE_STALE_US  (10LL * 1000 * 1000)
#define FLOOR_MV_DEFAULT  11600
#define FLOOR_MV_MIN      11000
#define FLOOR_MV_MAX      12800
#define HOME_RSSI_US      (2LL * 1000 * 1000)
#define LEASE_REFUNDS     3

static const char *TAG = "standby";

static volatile bool enabled;
static volatile standby_state_t state = STANDBY_ACTIVE;
static standby_state_t applied = STANDBY_ACTIVE;
static bool applied_pm = true;
static int64_t entered_at;
static const char *volatile reason = "boot";
static volatile bool driving;
static volatile bool lease_token;
static volatile unsigned lease_refunds;
static volatile bool home_idle;
static volatile bool connected;
static volatile int vbat_mv = -1;
static volatile int64_t vbat_at;
static int64_t low_since;
static volatile int floor_mv = FLOOR_MV_DEFAULT;
static volatile bool pm_allowed = true;
static QueueHandle_t *tx_queue;

static const char *state_name(standby_state_t s)
{
    switch (s) {
    case STANDBY_PARKED:  return "parked";
    case STANDBY_PROTECT: return "protect";
    default:              return "active";
    }
}

void standby_set_tx_queue(QueueHandle_t *queue)
{
    tx_queue = queue;
}

static void go(standby_state_t next, const char *why)
{
    if (state == next) {
        return;
    }
    state = next;
    reason = why;
    if (next == STANDBY_PARKED) {
        entered_at = esp_timer_get_time();
        low_since = 0;
    }
    ESP_LOGI(TAG, "%s (%s)", state_name(next), why);
}

void standby_note_drive(void)
{
    // The on edge, not every READY tick: a lease started while READY is
    // still fresh must not get its token back.
    if (!driving) {
        driving = true;
        lease_token = true;
        lease_refunds = 0;
    }
    if (state != STANDBY_ACTIVE) {
        go(STANDBY_ACTIVE, "drive");
    }
}

void standby_note_off(void)
{
    driving = false;
}

void standby_note_locked(const char *why)
{
    driving = false;
    // A second lock event while parked keeps the original budget.
    if (enabled && state == STANDBY_ACTIVE) {
        go(STANDBY_PARKED, why);
    }
}

void standby_note_unlocked(void)
{
    // The owner is back at the car: the next walk-away is a new departure.
    driving = false;
    lease_token = true;
    lease_refunds = 0;
    if (state != STANDBY_ACTIVE) {
        go(STANDBY_ACTIVE, "unlocked");
    }
}

bool standby_lease_allowed(void)
{
    return lease_token && state != STANDBY_PROTECT;
}

void standby_note_lease_started(void)
{
    lease_token = false;
}

void standby_note_lease_ended(bool touched_vehicle)
{
    if (!touched_vehicle && state == STANDBY_ACTIVE && lease_refunds < LEASE_REFUNDS) {
        lease_refunds++;
        lease_token = true;
    }
}

void standby_note_connected(void)
{
    connected = true;
}

void standby_note_disconnected(void)
{
    connected = false;
    home_idle = false;
}

void standby_note_voltage(float volts, bool valid)
{
    vbat_mv = valid ? (int)(volts * 1000.0f + 0.5f) : -1;
    vbat_at = esp_timer_get_time();
}

standby_state_t standby_state(void)
{
    return state;
}

bool standby_holds_awake(void)
{
    return state == STANDBY_PARKED;
}

bool standby_protect_requested(void)
{
    return state == STANDBY_PROTECT;
}

int64_t standby_rssi_period_us(int64_t normal_us)
{
    return state == STANDBY_PARKED && home_idle ? HOME_RSSI_US : normal_us;
}

static void report(char *response, size_t len)
{
    int64_t now = esp_timer_get_time();
    long since = state == STANDBY_PARKED ? (long)((now - entered_at) / 1000000) : 0;
    long budget = state == STANDBY_PARKED
        ? (long)((BUDGET_US - (now - entered_at)) / 1000000) : 0;
    if (budget < 0) {
        budget = 0;
    }
    snprintf(response, len, "XPK:%u,%s,%s,%ld,%ld,%d,%d,%u,%u,%u,%d",
             enabled ? 1 : 0, state_name(state), reason, since, budget,
             vbat_mv, floor_mv, home_idle ? 1 : 0,
             standby_lease_allowed() ? 1 : 0, pm_allowed ? 1 : 0,
             standby_platform_reset_reason());
}

static void push_report(void)
{
    static xdev_buffer buffer;
    if (tx_queue == NULL || *tx_queue == NULL || !connected) {
        return;
    }
    char line[96];
    report(line, sizeof(line));
    int len = snprintf((char *)buffer.ucElement, sizeof(buffer.ucElement), "$%s\r", line);
    if (len <= 0) {
        return;
    }
    if (len >= (int)sizeof(buffer.ucElement)) {
        len = sizeof(buffer.ucElement) - 1;
    }
    buffer.usLen = len;
    buffer.dev_channel = DEV_BLE;
    xQueueSend(*tx_queue, &buffer, 0);
}

void standby_tick(void)
{
    int64_t now = esp_timer_get_time();

    if (state == STANDBY_PARKED) {
        if (!enabled) {
            go(STANDBY_ACTIVE, "disabled");
        } else if (now - entered_at >= BUDGET_US) {
            go(STANDBY_PROTECT, "budget");
        } else {
            int mv = vbat_mv;
            bool low = mv < 0 || mv < floor_mv || now - vbat_at > VOLTAGE_STALE_US;
            if (!low) {
                low_since = 0;
            } else if (low_since == 0) {
                low_since = now;
            } else if (now - low_since >= FLOOR_DWELL_US) {
                go(STANDBY_PROTECT, mv < 0 || now - vbat_at > VOLTAGE_STALE_US
                                        ? "no-voltage" : "battery");
            }
        }
    }

    standby_state_t s = state;
    bool pm = pm_allowed;
    if (s != applied || (s == STANDBY_PARKED && pm != applied_pm)) {
        bool changed = s != applied;
        applied = s;
        applied_pm = pm;
        standby_platform_apply(s, pm);
        if (changed) {
            push_report();
        }
    }

}

bool standby_command(const char *args, char *response, size_t response_len)
{
    if (args[0] == 0) {
        report(response, response_len);
        return true;
    }
    if ((args[0] == 'e' || args[0] == 'h' || args[0] == 'p')
            && (args[1] == '0' || args[1] == '1') && args[2] == 0) {
        bool on = args[1] == '1';
        if (args[0] == 'e') {
            if (on != enabled) {
                ESP_LOGI(TAG, "standby %s by ATXPKE", on ? "enabled" : "disabled");
            }
            enabled = on;
        } else if (args[0] == 'h') {
            home_idle = on;
        } else {
            pm_allowed = on;
        }
        snprintf(response, response_len, "OK");
        return true;
    }
    if (args[0] == 'x' && args[1] == 0) {
        if (state == STANDBY_PARKED) {
            go(STANDBY_ACTIVE, "resumed");
        }
        snprintf(response, response_len, "OK");
        return true;
    }
    if (args[0] == 'v') {
        char *end = NULL;
        long value = strtol(&args[1], &end, 10);
        if (end == &args[1] || *end != 0 || value < FLOOR_MV_MIN || value > FLOOR_MV_MAX) {
            return false;
        }
        floor_mv = (int)value;
        snprintf(response, response_len, "OK");
        return true;
    }
    return false;
}
