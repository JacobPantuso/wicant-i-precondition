# wicant-i-precondition (Numen fork)

Firmware for a WiCAN-OBD-C3 on the [Ioniq5_CAN harness](https://github.com/tylerharvey/Ioniq5_CAN)
([build](https://github.com/tylerharvey/Ioniq5_CAN/wiring_harness/) or [buy](https://electroniqbuttons.com))
in a first-generation E-GMP car: 2021-2024 IONIQ 5 and EV6, 2023-2025 IONIQ 6.
It's a fork of [L1Z3/wicant-i-precondition](https://github.com/L1Z3/wicant-i-precondition),
which adds a manual battery preconditioning button to the
[WiCAN firmware](https://github.com/meatpiHQ/wican-fw).

You're on **`feature/ble-remote-trigger`**.

| Branch | Adds |
|--------|------|
| `main` | upstream wicant-i-precondition, unchanged |
| **`feature/ble-remote-trigger`** | `ATXPC`: start and stop battery preconditioning over BLE |
| `feature/can-sniffer` | `ATXSN`: stream CAN frames over BLE (builds on the branch above) |
| `feature/cluster-nav` | `ATXNV`/`ATXNT`: turn-by-turn on the cluster and HUD (builds on the branch above) |
| `firmware/Numen` | all of the above plus seat, lock and sleep features. **Flash this one for the Numen app.** |

## Install

You need macOS or Linux, git, and a GitHub SSH key (the ESP-IDF submodule clones over SSH).

```sh
git clone --recursive -b feature/ble-remote-trigger https://github.com/JacobPantuso/wicant-i-precondition.git
cd wicant-i-precondition
./esp-idf/install.sh esp32c3   # first time only
. ./esp-idf/export.sh          # every new shell
./build.sh v300
```

Then flash it either way:

- **Wi-Fi:** join the WiCAN's network, open http://192.168.80.1, and upload
  `build.v300/wican-fw_*.bin` as a firmware update.
- **USB:** `idf.py -B build.v300 -p <port> -b 115200 flash`, where `<port>` is
  `/dev/cu.usbmodem*` on macOS or `/dev/ttyACM0` on Linux. The slow baud rate
  keeps a flaky cable from stopping the transfer part way.

After a USB flash, run `idf.py -B build.v300 -p <port> monitor` and check the
boot log's heap line: both CAN tasks must say `created: 1` (see *BLE notes*).

Host tests run without ESP-IDF: `make -C test/host`.

## Preconditioning over BLE (`ATXPC`)

Starts and stops preconditioning over the same BLE connection an OBD app
already uses for the WiCAN's ELM327 emulation. An adapter without this firmware
answers `?`.

| Command  | Effect | Reply |
|----------|--------|-------|
| `ATXPC`  | status | `XPC:<state>,<car>,<secs>,<tmin>,<tmax>,<flags>` |
| `ATXPC0` | stop (no-op if idle) | `OK` |
| `ATXPC1` | start (no-op if running) | `OK` |
| `ATXPC2` | toggle, same as the button | `OK` |

- `state`: `0` idle, `1` requested, `2` starting, `3` active, `4` managed by the BMU, `5` stopping
- `car`: the car's own status, `0` unknown, `1` idle, `2` starting, `3` started
- `secs`: countdown to the next start/stop retry, `0` when nothing is pending
- `tmin` / `tmax`: pack temperature extremes in °C, valid when flag bit 3 is set
- `flags`: bit 0 car in READY, bit 1 status frame seen, bit 2 button held, bit 3 pack temperature valid

Commands apply on the next 40 ms CAN tick, and the car takes tens of seconds to
start, so poll the status rather than treating `OK` as done.

BLE is on by default on this branch, because the WiCAN's access point is
unreliable. The AP is still there for a while after power-up.

## BLE notes

- Keep each BLE write to 65 bytes or fewer. Longer writes are truncated and
  logged. The parser joins writes until `\r`, so send long commands in pieces.
- The ESP32-C3 heap is close to its limit, and a CAN task that can't be created
  fails silently. Every boot logs:

      W app_main: heap before CAN tasks: free 15696, largest 7680, min ever 15604
      W app_main: can_rx_task created: 1, can_tx_task created: 1, heap after: free 8880, largest 7680

  Check it after any change that adds a static buffer or a queue.

---

[WiCAN docs](https://meatpihq.github.io/wican-fw/) ·
[Upstream releases](https://github.com/L1Z3/wicant-i-precondition/releases/) ·
[Project background](https://github.com/tylerharvey/Ioniq5_CAN) ·
[Fluxer server](https://fluxer.gg/w0OpDJjG)
