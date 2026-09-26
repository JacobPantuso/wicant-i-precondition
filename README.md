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
stops when BLE disconnects. The pin list holds 16 identifiers at most.

The change-only table (512 slots, ~16 KB) is allocated when a session starts and
freed when it stops, in 1 KB chunks: at boot the heap can't spare it, and a
single 16 KB block is rarely free once the device is up.

### Cluster and HUD turn-by-turn (`ATXNV`, `ATXNT`)

Lets a phone app put its own navigation (the myIONIQ app uses it for CarPlay
routes) on the instrument cluster and the head-up display. The head unit keeps
broadcasting "no guidance" nav frames while CarPlay navigates; the firmware
rewrites those on their way to the cluster, and sends the multi-frame text and
lane messages the head unit leaves silent under CarPlay.

**The firmware knows no frame layouts.** The app sends each frame's bytes and a
mask of which to overwrite, so decoding more of the cluster is an app update,
not a reflash.

| Command              | Effect | Reply |
|----------------------|--------|-------|
| `ATXNV`              | query status | `XNV:<active>,<updates>,<injected>` |
| `ATXNV0`             | stop, and forget every multi-frame message | `OK` |
| `ATXNV1<frames>`     | set the frames to rewrite: 1–5 × 22 hex digits, each `<id:4><mask:2><d0..d7:16>` | `OK` |
| `ATXNT`              | query status | `XNT:<messages>,<frames>` |
| `ATXNT<id:4><hex>`   | set the ISO-TP message for that CAN id and SID, up to 96 bytes | `OK` |

- **Frames `ATXNV1` may rewrite:** `4CC`, `63E`, `640`, `641`, `4E8`, `4EA`, and
  nothing else. The hook sees every frame bound for the cluster side, powertrain
  included, so a bad payload must not be able to rewrite one of those. A byte
  whose mask bit is clear keeps the head unit's value.
- **Messages `ATXNT` may originate:** text `F0` (next street), `F2` (HUD maneuver
  list) and `F4` (destination) on both `6E7` (cluster) and `6DF` (HUD), and the
  81-byte `F1` lane graphic on `680` and `681`. Frames are paced 5 ms apart with
  `AA` padding and no flow control, as the head unit sends them. Each is
  re-sent only when it changes.
- **Safety:** injection stops 4 s after the last `ATXNV1`, and at once when BLE
  disconnects, so a dropped link can't leave a frozen arrow. When it lapses,
  every message the firmware drew is replaced with the head unit's blank form
  (`SID 00 00`, or `F1` and 80 zeros), and `ATXNV0` or a disconnect forgets
  them.
- **Preconditioning wins:** its countdown display owns `4E8`/`4CC` while a start
  or stop is in flight (`fwd_hooks()` in `main.c`).
- **Single-bus vs MITM:** on the single-bus V300 the rewritten frame is sent
  right after the head unit's own, so the cluster may flicker between them. On
  a dual-bus board in bridge mode the same hook replaces the frame outright.
- Commands can be up to 254 characters (the ELM327 command buffer is 256 bytes
  on this branch). Send them in BLE writes of 65 bytes or fewer; see below.

The layouts, all from IONIQ 6 captures of the built-in nav, live in the app
(`ClusterNavFeed.swift`):

| Frame | Layout |
|-------|--------|
| `4CC` | `0D 00 00 <arrow> <m LE> 00 00`: arrow = clockwise angle ÷ 7.5°. `46` follow road, `09` calculating, `0A` recalculating |
| `63E` | `00 00 <m to turn LE> <m from the car to the turn after, LE> 00 00` |
| `640` | head unit's idle bytes, `d7` = approach bar in % |
| `4E8` | `<tenths<<4 \| unit> <ETA h> <ETA m> <arrow−1> <destination LE> 5<flag> 00` |
| `6E7`/`6DF` `F2` | `F2 <n> <HUD arrow ×3, 00-padded> <street> 00 00 00 00`: HUD arrows `41` straight, `42` slight R, `43` R, `44` sharp R, `45` sharp L, `46` L, `47` slight L, `48`/`49` U-turn L/R, `70` destination |
| `680`/`681` `F1` | `[1]` lane count, `[3+i]` lane colours (overlay<<2 \| underlay: 0 none, 1 white, 2 blue, 3 grey), `[19+3i]`/`[20+3i]` underlay/overlay arrow (`01` straight … `08` slight L) |

Host tests: `test/host/test_clusternav.c`.

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
