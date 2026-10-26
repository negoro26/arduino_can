# MKR1000 ELM327 adapter for Renault and Dacia

Firmware that turns an **Arduino MKR1000 + Arduino MKR CAN shield (ASX00005)** into an ELM327 v1.5-compatible diagnostic adapter on USB. It speaks the CAN protocols only (ISO 15765-4, 11-bit and 29-bit, 500 and 250 kbit). Diagnostic programs that drive an ELM327 use it unchanged: [ddt4all](https://github.com/cedricp/ddt4all), [pyren](https://gitlab.com/py_ren/pyren), [python-OBD](https://github.com/brendan-w/python-OBD), and generic OBD-II apps.

The repo also has `tools/canscan.py`, a scanner that reads identification and fault codes from every Renault/Dacia module without the proprietary databases that ddt4all and pyren need.

By default the adapter refuses to send anything that writes to an ECU. See [Write guard](#write-guard).

## Which cars work

The MCP2515 on the shield talks CAN only. It can never reach a module that is diagnosed over K-line (OBD pin 7).

| Car | Diagnostics | Result |
|---|---|---|
| Dacia Sandero II (2012-2020), 1.2 16V | every module on CAN except the SVT tracker ([RenCOM list](https://www.obdtester.com/rencom-eculist/dacia/sandero_ii_[2012_2020])) | engine, ABS, airbag, UCH, cluster, EPS, climate are reachable |
| Renault Megane II (2002-2008), 1.5 dCi | most modules on CAN; parking assist, some headlamp-levelling units, the LPG unit and one automatic gearbox on K-line ([RenCOM list](https://www.obdtester.com/rencom-eculist/renault/megane_ii_[2002_2008])) | the CAN modules are reachable |
| Skoda Fabia II (5J) 2008 | K-line until 12/2010 ([blafusel](https://www.blafusel.de/obd/vag_compatibility.html)) | does not work |
| VW Golf IV (1J) | K-line | does not work |

Nothing in this README has been tried on a car yet. The firmware passes the client checks against a simulated Sandero and against the same simulated car running on the real board (see [Develop and test](#develop-and-test)).

## Hardware

Verified on the bench, read from the official [ASX00005 schematic](https://docs.arduino.cc/resources/schematics/ASX00005-schematics.pdf), or taken from the [Arduino store page](https://store.arduino.cc/products/arduino-mkr-can-shield):

| Item | Value | How known |
|---|---|---|
| CAN controller | MCP2515 | probed over SPI, `CANSTAT=0x80` / `CANCTRL=0x87` |
| Chip select | **D3** (fixed by the shield PCB) | schematic + confirmed working |
| Interrupt | D7 (not used: the firmware polls over SPI) | schematic |
| Crystal | 16 MHz | schematic (`Y1`, OSC1/OSC2 column) |
| Transceiver | TJA1049T/3, `STB` tied low via `R8 = 0R` | schematic |
| Termination | 120R (`R3`) in series with the slide switch `SW1` | schematic; store page: "Switchable onboard termination resistor" |
| Transceiver power | header `5V` -> net `+5V` -> `VDD` | schematic |
| Power input | terminal `VIN` -> TPS54232 buck converter (`U3`) -> the MKR's `VIN` pin; rated 7-24 V | schematic, store page |

The transceiver is fed from the header `5V` pin, so USB power alone is enough. The terminal's `VIN` input is optional.

`SW1` switches the 120R across CAN-H and CAN-L. A car bus is already terminated at both ends, so set `SW1` to off for a car. The schematic does not say which slide position is off. Check with a multimeter on the shield alone: about 120 ohms between the `CANH` and `CANL` terminals means the terminator is on, and an open circuit means it is off.

## Wire it to the car

The shield has one 4-position terminal block. In the schematic its positions are, in order, `GND`, `CANL`, `CANH` and `VIN`.

Read the labels on the top side of the board. Arduino's store page warns: "On some boards, the bottom silk is mirrored. Please refer to the top silk for guidance." The bottom labels in Arduino's own product photo read `VIN`, `CANL`, `CANH`, `GND`, the reverse of the schematic. So `GND` and `VIN` are the two positions that are easy to swap. Before you connect the car, check with a multimeter that the position you use as `GND` beeps against the metal shell of the MKR1000's USB socket.

The OBD2 connector pinout (SAE J1962) is fixed by the standard, so check an untrustworthy aftermarket cable against it:

| OBD2 pin | Signal | Terminal position |
|---|---|---|
| 6 | CAN-H | `CANH` |
| 14 | CAN-L | `CANL` |
| 4 or 5 | GND | `GND` |

Do **not** connect pin 16 (+12 V). The board runs from USB.

### What a miswire can damage

CAN-H and CAN-L cannot be damaged by a miswire. The NXP TJA1049 datasheet gives `VCANH` and `VCANL` an absolute maximum of -58 V to +58 V and an operating common-mode range of -12 V to +12 V, so battery voltage on a bus pin is within normal operation. ESD rating is +-8 kV. The MCP2515 sits behind the transceiver and never sees the wires.

GND can. If +12 V lands on the `GND` position, the battery shorts through the USB cable. Check that connection twice.

## Flash the firmware

```sh
pio run -t upload            # the adapter (env mkr1000USB)
```

PlatformIO may not be on `PATH`: it lives at `~/.platformio/penv/bin/pio` (IDE install) or `~/.local/bin/pio` (`uv tool install platformio`). Serial devices are `root:uucp 0660`, so your user must be in the `uucp` group (`sudo usermod -aG uucp $USER`, then log out and back in, or run the command inside `newgrp uucp`).

The board appears as `/dev/ttyACM0` on Linux and as "USB Serial Device (COMx)" on Windows 10/11. The baud rate does not matter on native USB.

## Check the wiring first

Ignition to position II, engine off. Open a terminal (`./monitor.sh`, or Tera Term or PuTTY on Windows), send `ATL1` so lines do not overwrite each other, then send `AT#SURVEY`. The survey listens at 500, 250, 125 and 100 kbit and transmits nothing:

| Frames | Errors | Verdict |
|---|---|---|
| > 0 | any | pins correct, bitrate found |
| 0 | > 0 | pins on a live bus but nothing decodes: swap CAN-H and CAN-L, or the bus runs at an untried rate |
| 0 | 0 | not connected: wrong OBD pins, broken cable, or bus asleep |

If a rate prints `SETUP FAILED`, the fault is the board, not the car. Run `AT#SELFTEST`.

## Read fault codes without a database

ddt4all cannot scan or read codes until the DDT2000 `ecus` database is installed, and pyren exits without the CLIP database. Both are Renault-proprietary and not part of this repo. `tools/canscan.py` works without either:

```sh
uv run tools/canscan.py /dev/ttyACM0                  # Windows: COM5
uv run --with obd tools/canscan.py /dev/ttyACM0       # adds descriptions for generic OBD-II codes
uv run tools/canscan.py /dev/ttyACM0 --json report.json
```

The scan runs in three steps:

1. It listens for 1.5 s (`ATMA`) and records every CAN id in use. It never transmits on a request id that the car already uses for normal traffic.
2. It reads generic OBD-II on 7DF: MIL, VIN, and stored, pending and permanent codes.
3. It probes the 114 Renault/Dacia request ids from ddt4all's address table. Each module gets a UDS probe (`10 03`) and then a KWP2000 probe (`10 C0`). Each module that answers is identified (`22 F1 xx` or `21 80`) and its codes are read (`19 02 AF` or `17 FF 00`).

This is the output against the simulated Sandero, shortened:

```
OBD-II (7DF):
  7E8 engine: MIL on, 2 stored DTCs (0101), VIN UU15SDAG551234567
    stored     P0335
    stored     P0115
Renault/Dacia modules:
  UCH                                      745->765  KWP
      supplier: 213
      DTC 9323 status 0xE0 (B1323 if SAE)
  ECM                                      7E0->7E8  UDS
      supplier: CONTINENTAL
      P0335-1C  DTC 03351C status 0x2F active, stored, pending
  not probed, request id seen as live traffic: 622->484 Steering Column
4 modules answered, 6 DTCs
```

Renault names its codes ("DF001") in the databases, so the scanner shows the raw code plus its SAE reading. A full scan takes about 40 s.

To clear codes, add `--clear`. After the report the scanner lists what it will clear and waits until you type `CLEAR`. It raises the guard to `CLEAR`, sends `14 FF FF FF` (UDS), `14 FF 00` (KWP) and OBD mode `04`, drops the guard back to `READ`, and reads the codes again.

## Use ddt4all, pyren or python-OBD

- **ddt4all.** Pick adapter type **USB**, leave **Change UART speed** at **No**, and type the port (`/dev/ttyACM0` or `COMx`). ddt4all needs the DDT2000 `ecus` folder for scanning and fault codes.
- **pyren.** Run `pyren3.py -p /dev/ttyACM0`. pyren needs the CLIP database.
- **python-OBD.** Pass the port explicitly, `obd.OBD("/dev/ttyACM0")`. Its auto-scan on Linux looks only at `ttyUSB` and `rfcomm` devices.

The firmware answers `?` to `AT BRD` (USB has no baud rate). That is why ddt4all's UART speed must stay at **No**.

## Write guard

Every frame the adapter transmits is classified by its diagnostic service before it reaches the bus. A frame above the current tier is not sent, and the adapter prints `BLOCKED` (ddt4all and pyren show that as an empty answer).

Below `WRITE` the adapter transmits only on diagnostic CAN ids. Those are the 11-bit ids `600` to `7FF` and the 29-bit ids `18DAxxxx` and `18DBxxxx` (ISO 15765-4 normal fixed addressing, for example `18DA10F1` and `18DB33F1`). A frame on any other id needs `WRITE`, whatever its bytes are.

| Tier | Allows |
|---|---|
| `READ` (default) | OBD-II modes 01, 02, 03, 05, 06, 07, 09, 0A; services 10 (except programming sessions 02, 82, 85), 12, 17, 18, 19, 1A, 20, 21, 22, 23, 24, 3E, 81, 82, 83; flow-control frames |
| `CLEAR` | `READ` plus 14 (clear DTCs) and OBD-II mode 04 |
| `WRITE` | everything, including 2E, 3B, 2F, 30, 31, 27, 11, 28, 85 and 34-37 |

- `AT#GUARD` prints the tier. `AT#GUARD READ`, `AT#GUARD CLEAR` and `AT#GUARD WRITE` set it.
- The tier lives in RAM. ATZ, ATWS and ATD do not touch it. Unplugging the board resets it to `READ`.
- To clear codes from ddt4all, open a terminal first, send `AT#GUARD CLEAR`, close the terminal, then connect ddt4all.
- In raw mode (`ATCAF0`, which ddt4all and pyren use), the guard reads the service from the ISO-TP frame itself. A consecutive frame gets the verdict of the first frame before it.

## How the adapter behaves on the bus

- Between requests the MCP2515 is in listen-only mode. It does not ACK frames and cannot send error frames.
- Before the first transmission at a bitrate, it listens for 60 ms. If it sees errors but no frames, the bitrate is wrong. It then prints `CAN ERROR` and sends nothing.
- Every transmission is one-shot. A frame that hits a bus error is never retried by the chip. On this board, measured with nothing connected: in one-shot mode the attempt ended after 93 us, while without one-shot the chip retried for the full 20 ms test window.
- A frame is retried only after a clean arbitration loss, at most 2 times (3 attempts in all). The attempt must end with lost arbitration and no transmit error, and the receive error counter and the message error flag must show no bus error during it. Such a loss means another node won the bus, so nothing was corrupted.
- Any other failed attempt is not retried. In one-shot mode the chip also reports lost arbitration after a bus error. On this board with nothing connected at 500 kbit, an attempt ends that way and the receive error counter rises by 9.

## Commands

The firmware implements the ELM327 AT commands that ddt4all, pyren and python-OBD send, plus the K-line parameter commands that their adapter checkers test (they answer `OK` and do nothing). Three commands are specific to this adapter:

| Command | Does |
|---|---|
| `AT#GUARD [READ\|CLEAR\|WRITE]` | prints or sets the write-guard tier |
| `AT#SURVEY` | listen-only wiring check at 500, 250, 125 and 100 kbit |
| `AT#SELFTEST` | MCP2515 presence check, then a loopback frame at each bitrate |

Differences from a real ELM327:

- Protocols 1-5 (K-line, J1850) can be selected, but requests answer `UNABLE TO CONNECT`. `ATSI` and `ATFI` answer `BUS INIT: ...ERROR`.
- `ATRV` always answers `12.0V`. Pin 16 is not wired, so nothing is measured. python-OBD refuses to connect below 6 V.
- `AT BRD` and `STI` answer `?`.
- A bare CR repeats only the last CAN request, not an AT command.
- `ATMA` keeps monitoring when the chip drops a frame, instead of stopping with `BUFFER FULL`.
- After the last frame of an answer the adapter waits 50 ms (`AT AT1`, default) or 25 ms (`AT AT2`) for more, capped by `AT ST`. While consecutive frames are still owed after its own flow control, it waits the full `AT ST`.
- One answer can hold at most 600 frames, which covers a 4095-byte ISO-TP message. Above that the adapter prints `BUFFER FULL`.
- `ATCS` prints `T:xx R:xx` without the frequency field.

## Develop and test

| Environment | Builds |
|---|---|
| `mkr1000USB` (default) | the adapter |
| `mkr1000USB_demo` | the adapter plus a virtual Renault/Dacia car that answers through the MCP2515 in loopback mode. Use it to try the PC tools at a desk. |
| `native` | the host test suites |
| `sim` | a PTY simulator of the same virtual car for real PC clients |

Code layout:

- `lib/elm` is the portable ELM327 core. It has no Arduino dependency, no heap and no STL.
- `lib/vecu` holds the virtual car: ISO-TP ECUs and a simulated bus that models the MCP2515's two receive buffers.
- `src/firmware` holds the MCP2515 adapter, and `src/sim` holds the PTY host.

```sh
pio test -e native                                         # 98 cases, no board, no car
pio run -e sim && .pio/build/sim/program --link /tmp/elm327-sim
uv run tools/compat/check_clients.py --sim .pio/build/sim/program
uv run tools/compat/check_clients.py --port /dev/ttyACM0   # board running mkr1000USB_demo
uv run --with pytest --with pyserial pytest tools          # scanner decoders
```

`tools/compat/check_clients.py` runs the real client code (ddt4all and pyren at pinned commits, python-OBD 0.7.3, and `canscan.py`) against the simulator or the demo board. It passes 80 of 80 checks on both.

**Always pass `-e native` to `pio test`.** `default_envs` applies to `pio test` too. A bare `pio test` resolves to `mkr1000USB`, finds every suite ignored there, and reports `0 test cases` with exit status 0. Without that `test_ignore`, `pio test` would instead upload a Unity runner to the board and hang waiting for results.

Library: `autowp/autowp-mcp2515`, chosen over `sandeepmistry/arduino-CAN` because the latter's `endPacket()` busy-waits on TXREQ until a frame is ACKed, which never returns on a bus with no other node. The firmware still writes CANCTRL itself for one-shot mode, because the library's `setNormalOneShotMode()` always reports failure (its self-check compares a bit that CANSTAT never carries).

### Editor and LSP

```sh
pio run -t compiledb    # regenerate compile_commands.json
```

Regenerate the database after a fresh clone. It is gitignored because it embeds absolute toolchain paths. Without it clangd falls back to a bare `clang file.cpp` with no include paths and reports `'Arduino.h' file not found`. Restart the language server afterwards (`:LspRestart` in Neovim).

`.clangd` handles two clang-versus-GCC disagreements, explained in that file. The build itself is GCC (`arm-none-eabi-g++`). When clangd and `pio run` disagree, `pio run` is right.
