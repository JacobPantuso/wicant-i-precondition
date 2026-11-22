# Approach unlock experiment

This branch only: `feature/wican-approach-unlock`. Do not merge the approach
files into the normal firmware release. The app's normal bundler rejects them.

WiCAN detects sustained filtered RSSI at least -65 dBm for two seconds after
its own successful walk-away lock. Reconnect alone cannot unlock. Near events
are one-shot and expire after five seconds. RSSI must still be fresh within
1.5 seconds at execution. READY, lost proximity, disconnect, mode changes,
manual locking and consumed/failed authentication invalidate eligibility.

BLE-only commands:

- `ATXAUM0`: Off; `ATXAUM1`: Detect Only; `ATXAUM2`: Unlock mode.
- `$XAU:near,<sequence>,<signal>`: a new near event, including in Detect Only.
- `ATXAU`: `XAU:<mode>,<sequence>,<signal>,<nonce-or-dash>`.
- `ATXAU<mac>`: HMAC-SHA256(key, `numen-approach` || nonce), accepted only
  for the current eligible event after its challenge was requested.

Modes are RAM-only and default Off on boot. No reusable unlock permission is
stored. The Debug phone sets its selected mode after authenticated pairing.
The phone signs each event using its existing Keychain unlock key, and firmware
rechecks eligibility before queuing its local body burst. The normal unlock
API and cloud command queue are not used.

Default WiCAN config disables sleep. This branch does not change sleep policy;
installed NVS configuration may differ. If BLE is asleep, approach cannot work.
RSSI is not relay-resistant distance measurement. Detect Only is the default
phone setting; test unwanted near-home detections before choosing Unlock.

Build v300 with ESP-IDF, then use the app's explicit
`scripts/bundle-experimental-adapter-firmware.sh <checkout>` for local Debug
resources. These image names are ignored by Git and excluded from Release.
Installing the phone app does not flash the WiCAN: use the in-app adapter
updater to install the local experimental image when ready to test on the car.
