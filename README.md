# wicant-i-precondition (Numen fork)

Firmware for a WiCAN-OBD-C3 on the [Ioniq5_CAN harness](https://github.com/tylerharvey/Ioniq5_CAN)
([build](https://github.com/tylerharvey/Ioniq5_CAN/wiring_harness/) or [buy](https://electroniqbuttons.com))
in a first-generation E-GMP car: 2021-2024 IONIQ 5 and EV6, 2023-2025 IONIQ 6.
It's a fork of [L1Z3/wicant-i-precondition](https://github.com/L1Z3/wicant-i-precondition),
which adds a manual battery preconditioning button to the
[WiCAN firmware](https://github.com/meatpiHQ/wican-fw).

You're on **`main`**.

| Branch | Adds |
|--------|------|
| **`main`** | upstream wicant-i-precondition, unchanged |
| `feature/ble-remote-trigger` | `ATXPC`: start and stop battery preconditioning over BLE |
| `feature/can-sniffer` | `ATXSN`: stream CAN frames over BLE (builds on the branch above) |
| `feature/cluster-nav` | `ATXNV`/`ATXNT`: turn-by-turn on the cluster and HUD (builds on the branch above) |
| `firmware/Numen` | all of the above plus seat, lock and sleep features. **Flash this one for the Numen app.** |

This branch matches upstream. For the Numen app, use [`firmware/Numen`](https://github.com/JacobPantuso/wicant-i-precondition/tree/firmware/Numen).

## Install

You need macOS or Linux, git, and a GitHub SSH key (the ESP-IDF submodule clones over SSH).

```sh
git clone --recursive -b main https://github.com/JacobPantuso/wicant-i-precondition.git
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

Host tests run without ESP-IDF: `make -C test/host`.

---

[WiCAN docs](https://meatpihq.github.io/wican-fw/) ·
[Upstream releases](https://github.com/L1Z3/wicant-i-precondition/releases/) ·
[Project background](https://github.com/tylerharvey/Ioniq5_CAN) ·
[Fluxer server](https://fluxer.gg/w0OpDJjG)
