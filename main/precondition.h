#ifndef __PRECONDITION_H__
#define __PRECONDITION_H__

#include <stdbool.h>
#include <stdint.h>
#include "hsm.h"

void precondition_init(void);
void precondition_can_rx_hook(twai_message_t *to_push, can_bus_t rx_bus);
fwd_result_t precondition_fwd_hook(twai_message_t *to_send, can_bus_t fwd_bus);
void precondition_tick(void);

typedef struct {
    int8_t min_c;
    int8_t max_c;
    int64_t updated_at_us;
} precondition_temperature_t;

bool precondition_get_battery_temperature(precondition_temperature_t *out);

// ********************* remote activation *********************
// A second way in, alongside the harness button: a request from another task
// (today the ELM327 `ATXPC` command arriving over BLE). The request is queued,
// not applied, so the state machine keeps being driven from the CAN task only.

typedef enum {
    PRECON_REMOTE_TOGGLE = 0,  // exactly what an activation button press does
    PRECON_REMOTE_START,       // start; no-op if a request is already in flight
    PRECON_REMOTE_STOP,        // stop; no-op if nothing is running
} precon_remote_action_t;

// Safe to call from any task.
void precondition_request_remote(precon_remote_action_t action);

// Coarse public view of the state machine's leaf, for remote status queries.
// Deliberately narrower than the internal states: a caller wants to know what
// to show a driver, not which burst phase is in flight.
typedef enum {
    PRECON_STATE_IDLE = 0,
    PRECON_STATE_REQUESTED,  // request owned, no burst in flight yet
    PRECON_STATE_STARTING,   // start burst sent, waiting on the car
    PRECON_STATE_ACTIVE,     // once-mode session running
    PRECON_STATE_MANAGED,    // repeating-mode session handed to the BMU
    PRECON_STATE_STOPPING,
} precon_state_t;

// What the car itself last reported, independent of what we asked for. Stays
// UNKNOWN on platforms where no status frame is ever seen.
typedef enum {
    PRECON_CAR_UNKNOWN = 0,
    PRECON_CAR_IDLE,
    PRECON_CAR_STARTING,
    PRECON_CAR_STARTED,
} precon_car_status_t;

#define PRECON_FLAG_CAR_READY     0x01U
#define PRECON_FLAG_STATUS_FRAME  0x02U
#define PRECON_FLAG_BUTTON_HELD   0x04U
#define PRECON_FLAG_TEMP_VALID    0x08U

typedef struct {
    uint8_t  state;              // precon_state_t
    uint8_t  car_status;         // precon_car_status_t
    uint16_t seconds_remaining;  // until the next start/stop retry; 0 when none
    int8_t   batt_min_c;
    int8_t   batt_max_c;
    uint8_t  flags;              // PRECON_FLAG_*
} precondition_status_t;

// False until the first tick has published a snapshot.
bool precondition_get_status(precondition_status_t *out);

#endif
