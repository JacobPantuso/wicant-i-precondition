# What is this?

This is a fork of the [WiCAN firmware repository](https://github.com/meatpihq/wican-fw) designed to add custom CAN-based buttons. For now, the button added is a manual preconditioning button for first-generation E-GMP cars (2021-2024 Hyundai Ioniq 5 and EV6, 2023-2025 Ioniq 6). This is only possible through direct access to CAN busses with a [custom harness](https://github.com/tylerharvey/Ioniq5_CAN/wiring_harness/) that you can build or [buy](https://electroniqbuttons.com). See [our project coordination repository](https://github.com/tylerharvey/Ioniq5_CAN) for more background.


# [WiCAN Documentation](https://meatpihq.github.io/wican-fw/) | [Firmware updates](https://github.com/L1Z3/wicant-i-precondition/releases/) | [Fluxer server](https://fluxer.gg/w0OpDJjG)

# Building

1. Clone this repo recursively: `git clone --recursive https://github.com/L1Z3/wicant-i-precondition.git`
2. Install ESP-IDF: `cd wicant-i-precondition/esp-idf && ./install.sh`
3. Export the environment: `. ./export.sh`
4. Open project and build: `cd .. && ./build.sh v300`
5. Flash: use http interface or if using USB: `idf.py flash`

# WiCAN-OBD-C3 Information

The WiCAN original shipped with the beta preconditioning kit is a powerful ESP32-C3-based CAN adapter for car hacking and general CAN-bus development. WiCAN connects to your existing Wi-Fi network and any device on that network, where it allows you to configure Wi-Fi and CAN settings through a built-in web interface. The WiCAN has a power-saving mode that detects when the voltage drops under 13 V or some other preset value. When this power-saving mode is engaged, WiCAN is capable of entering sleep mode, which drops current consumption below 1 mA.

WiCAN is a simple, ready-to-use solution for CAN-bus development and hacking. It accelerates development by providing vehicle-diagnostic APIs and libraries in various languages and for various operating systems. WiCAN works with a large array of pre-existing vehicle-diagnostic libraries, including RealDash, SavvyCAN, BUSmaster, python-can/SocketCA, and more. APIs are also available for LabView, C#, VB.Net, Delphi, and Python in case you’re writing your own software.

## Description
![image](https://user-images.githubusercontent.com/94690098/231444160-08842087-55ad-4165-8291-b379da63aeeb.png)

WiCAN-OBD will be of great interest to car enthusiasts and tinkers who want to modernize or customize the head-unit displays in their cars using RealDash. Check out some examples of the available graphic interfaces, which are supported by a robust collection of Manuals & Tutorials to get you started with RealDash.

Another great feature of WiCAN-OBD is its MQTT battery alerts. It can monitor your battery voltage and send an alert if that voltage drops under a set threshold. This feature is especially important for users who own multiple cars they do not use regularly.

## WiCAN-OBD2 Pinout

<p align="center">
<img src="https://user-images.githubusercontent.com/94690098/182854687-911bae04-9bdd-4947-8363-e088e278b3b8.png" >
</p>

## [**Programming Examples**](https://github.com/meatpiHQ/programming_examples/tree/master/CAN)

### **Features and Specifications**:

- Supports CAN2.0A/B up to 1Mbits.
- Works with Realdash, based on "realdash 66"
- Supports SocketCAN and works with BUSMaster
- Supports TCP and UDP
- WiFi can be used in AP and station mode
- WiFi and CAN configured using web interface.
- Diode protection for the USB port

### Remote activation over BLE (`ATXPC`)

Alongside the harness button, preconditioning can be started and stopped by a
BLE client talking to the WiCAN's ELM327 emulation — the same connection an OBD
app already uses, so it needs no second transport and does not disturb normal
PID reads.

| Command  | Effect                                       | Reply |
|----------|----------------------------------------------|-------|
| `ATXPC`  | query status                                 | `XPC:<state>,<car>,<secs>,<tmin>,<tmax>,<flags>` |
| `ATXPC0` | stop (no-op if nothing is running)           | `OK`  |
| `ATXPC1` | start (no-op if already running)             | `OK`  |
| `ATXPC2` | toggle — identical to pressing the button    | `OK`  |

An adapter without this firmware answers `?`, which is how a client can tell the
feature apart from a stock ELM327 without guessing.

Status fields:

- `state` — `0` idle, `1` requested, `2` starting, `3` active, `4` managed by the
  BMU (repeating mode), `5` stopping
- `car` — what the car reports in its own status frame: `0` unknown, `1` idle,
  `2` starting, `3` started
- `secs` — countdown to the next start/stop retry; `0` when nothing is pending
- `tmin` / `tmax` — pack temperature extremes in °C, valid only when the
  temperature flag is set
- `flags` — bit 0 car in READY, bit 1 status frame seen, bit 2 button held,
  bit 3 pack temperature valid

Requests are queued and applied on the next 40 ms CAN tick, and the status
snapshot is republished on that same tick, so a query issued immediately after a
command can still report the previous state. Clients should poll rather than
treat the `OK` as confirmation — the car itself takes tens of seconds to
acknowledge a start.

Due to this implementation, the firmware associated on this branch enables BLE by default. The AP is still accessible on initial startup after power cycling the device but sometimes the AP cannot be accessed.  

### CAN sniffer over BLE (`ATXSN`)

A second vendor command streams received CAN frames to the BLE client, for
reverse-engineering traffic (the myIONIQ app records it while driving). It is
receive-only: nothing is transmitted.

| Command               | Effect                                                   | Reply |
|-----------------------|----------------------------------------------------------|-------|
| `ATXSN`               | query status                                             | `XSN:<on>,<npins>,<rx>,<sent>,<dropped>,<suppressed>,<ids>` |
| `ATXSN0`              | stop                                                     | `OK`  |
| `ATXSN1`              | start, change-only for every identifier                  | `OK`  |
| `ATXSN1 63E,4CC,...`  | start, and pass every frame of the listed identifiers    | `OK`  |
| `ATXSN1P 63E,4CC,...` | the listed identifiers only, nothing else                | `OK`  |

While running, frames arrive as `'$'`-prefixed lines between ordinary ELM327
responses, one per line, each ending in `\r`:

    $0000 0001A2F3 63E 8 0322FF0000000000   line number, timestamp (low 32 bits of µs), id, DLC, data
    $0001 ! 1532 211 0 1321 94              once a second: rx, sent, dropped, suppressed, ids

Line numbers count the lines the firmware handed to BLE, from 0000 at each
start; a gap on the receiving side means lines were lost in transit.

A whole bus does not fit through BLE, so an unlisted identifier is only sent
when its payload changes, at most 5 times a second. Listed ("pinned")
identifiers bypass that, so multi-frame ISO-TP messages arrive intact. On a busy
bus even the change-only stream saturates BLE (about 30% of lines were dropped
on a drive), and one lost frame ruins a multi-frame message, so `ATXSN1P`
streams the pinned identifiers alone. Lines are dropped rather than let them
crowd out command replies, and the `dropped` counter says how many. The sniffer
stops when BLE disconnects. The command is capped at 126 characters, which
bounds the pin list (16 at most).

The change-only table (512 slots, ~16 KB) is allocated when a session starts and
freed when it stops, in 1 KB chunks: at boot the heap can't spare it, and a
single 16 KB block is rarely free once the device is up.

### BLE client notes

- **Keep each BLE write to 65 bytes or fewer.** The receive buffer is
  `DEV_BUFFER_LENGTH` (65) bytes. A longer write is now truncated (and logged)
  instead of overrunning the buffer, which used to wedge command handling until
  reboot. The ELM327 parser joins writes until the terminating `\r`, so a long
  command can be sent in pieces.
- **The boot log reports heap headroom.** The ESP32-C3 heap is close to its
  limit, and `xTaskCreate` fails silently when it runs out: at one point neither
  CAN task started, so no ELM327 command was answered and the harness button did
  nothing. Every boot now logs (at warning level, so it survives the runtime log
  filter):

      W app_main: heap before CAN tasks: free 15696, largest 7680, min ever 15604
      W app_main: can_rx_task created: 1, can_tx_task created: 1, heap after: free 8880, largest 7680

  Check that line after any change that adds a static buffer or a queue.
- The Wi-Fi TCP reply queue is 32 slots, down from 128, to leave room for the
  CAN tasks' stacks.

---
![Modes](https://user-images.githubusercontent.com/94690098/222961571-bd137341-808a-4f0a-9528-789fe24d640e.png "Connection Mode")
---

Images of WiCANs © 2026 meatPi Electronics | www.meatpi.com | PO Box 5005 Clayton, VIC 3168, Australia
