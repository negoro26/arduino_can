# MKR1000 CAN bring-up probe

Diagnostic firmware for an **Arduino MKR1000 + Arduino MKR CAN shield (ASX00005)**,
written to answer one question about a **Renault Megane II (2006-2008)**:

> Which of this car's ECUs can be reached over CAN, and which are K-line only?

That answer decides whether a DIY ELM327-to-CAN bridge can drive
[ddt4all](https://github.com/cedricp/ddt4all) against this car at all. An MCP2515
can never reach a K-line module, no matter what the firmware does.

## Status

The firmware here is the **diagnostic probe**, not the ELM327 emulator. The
emulator has not been written yet; it is waiting on the measurement this probe
produces.

## Hardware facts

Verified on the bench, or read from the official
[ASX00005 schematic](https://docs.arduino.cc/resources/schematics/ASX00005-schematics.pdf):

| Item | Value | How known |
|---|---|---|
| CAN controller | MCP2515 | probed over SPI, `CANSTAT=0x80` / `CANCTRL=0x87` |
| Chip select | **D3** (fixed by the shield PCB) | schematic + confirmed working |
| Interrupt | D7 | schematic |
| Crystal | 16 MHz | schematic (`Y1`, OSC1/OSC2 column) |
| Transceiver | TJA1049T/3, `STB` tied low via `R8 = 0R` | schematic |
| Termination | onboard **120R** (`R3`), no jumper found | schematic |
| Transceiver power | header `5V` -> net `+5V` -> `VDD` | schematic |

Because the transceiver is fed from the header `5V` pin, **USB power alone is
enough — the 12 V screw terminal is optional.**

Note the onboard 120R: a vehicle bus is already terminated at both ends, so this
adds a third terminator in parallel. Usually fine for a short session; suspect it
first if frames arrive but erratically.

## Safety model

| Stage | Transmits? | Notes |
|---|---|---|
| 1 - MCP2515 presence | no | raw SPI register probe |
| 2 - loopback self-test | no | never reaches the wire |
| 3 - listen-only survey | **no** | MCP2515 listen-only sends no ACKs and no error frames |
| 4 - active ECU discovery | **YES** | armed by hand, plus two interlocks |

Stages 1-3 run automatically and cannot disturb a vehicle bus.

Stage 4 sends exactly one payload, and nothing else exists in the firmware:

```c
{0x02, 0x10, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00}   // ISO-TP SF: UDS StartDiagnosticSession
```

Service `0x10` opens a diagnostic session and writes nothing. It is in ddt4all's
own `safe_commands` whitelist. Absent by design: `2E`/`3B`/`3D` (write),
`34`/`36` (flash), `31` (routine control), `11` (ECU reset), `27` (security
access), `14`/`04` (clear DTC). A session also self-expires without a `3E`
keepalive, which this firmware never sends.

### Stage 4 interlocks

1. **Refuses to transmit unless stage 3 saw real traffic.** This proves the
   bitrate passively before anything is transmitted. A wrong bit rate would
   otherwise flood the bus with error frames.
2. **Skips every CAN ID that stage 3 observed.** It cannot collide with a
   module's operational messages, because the exclusion list is measured from
   this car rather than assumed from convention.

## Wiring

DB9 on the shield to the car's OBD2 (SAE J1962) socket:

| OBD2 pin | Signal |
|---|---|
| 6 | CAN-H |
| 14 | CAN-L |
| 4 or 5 | GND |

Do **not** connect pin 16 (+12 V). It is not needed and it is the one miswire
that can do real damage: +12 V onto a ground line shorts the battery through the
USB cable.

If stage 3 reports `0 frames` at both bitrates, **swap CAN-H and CAN-L** — the
MKR CAN shield's silkscreen labels are reported swapped. Stage 3 is listen-only,
so retrying costs nothing.

## Reading the output

Nothing needs to be installed on the laptop: the firmware is already in flash,
so the laptop is only a screen. The MKR1000 is native USB CDC and appears as
**"USB Serial Device (COMx)"** on Windows 10/11 with the built-in driver.

**115200 baud, 8-N-1, no flow control.** DTR/RTS state is irrelevant (measured).

### Windows

[Tera Term](https://ttssh2.osdn.jp/) lists serial ports in its opening dialog:

1. Open Tera Term -> **Serial** -> pick the COM port -> OK
2. **Setup -> Serial port** -> Speed `115200` -> New setting
3. **File -> Log...** -> filename, tick **Plain text** -> Save

PuTTY works too: Connection type Serial, Speed 115200, logging under
Session -> Logging -> "All session output".

### Linux

```sh
./monitor.sh          # wraps `pio device monitor`, logs to ./logs/
```

Output repeats every ~3 seconds. Opening a port does not reset a native-USB
board, so a one-shot report at boot would be missed — the sweep loops instead.

## Running it on the car

Laptop on battery, charger unplugged. Ignition to position II, engine off
(modules sleep otherwise).

Watch stage 3. A live bus looks like:

```
[3] Listen-only bus survey (cannot disturb the vehicle)
    500 kbit: 1247 frames, ids: 0x1F6 0x181 0x5DE 0x354 ...
    bus alive at 500 kbit; 14 distinct ids, 0 of them inside 0x700-0x7FF (will be skipped)
```

Only then press `s` to arm stage 4. Each responder is reported as:

```
ECU: request 0x745 -> response 0x765  [03 50 C0 00 ...]  (session opened)
```

A `7F` negative response still counts — a refusing ECU is a present ECU.

## Build

```sh
pio run -t upload
```

PlatformIO installs its own venv and does not add itself to `PATH`; it lives at
`~/.platformio/penv/bin/pio`. Serial devices are `root:uucp 0660`, so the user
must be in the `uucp` group (`sudo usermod -aG uucp $USER`, then re-login).

Library: `autowp/autowp-mcp2515` — chosen over `sandeepmistry/arduino-CAN`
because the latter's `endPacket()` busy-waits on TXREQ until a frame is ACKed,
which never returns on a bus with no other node.
