# jeep-aux-and-can-gateway

An ATmega328 board that lives inside the radio of my 2005 Jeep Grand Cherokee WH, spliced
into the interior **CAN-B bus at 83.333 kbps**.

It has been doing four things for ten years, and still does:

1. Announces a VES entertainment module on `0x3DD`, so the head unit offers its AUX
   input.
2. Powers the OVC3860 Bluetooth A2DP module, but only while AUX/VES is the selected
   source.
3. Maps radio panel buttons (`0x394`) to Bluetooth transport commands -- seek left/right
   to prev/next track, RND/PTY to play/pause.
4. Optionally runs **bench mode** so a head unit removed from the car powers up and stays
   unlocked on the desk: it fakes ignition on `0x000` and, if a VIN has been provisioned
   (`AT@VIN`, stored in EEPROM), broadcasts it on `0x01B` (a short burst -- the radio
   latches it once and would otherwise demand its SKREEM anti-theft code). With no VIN set
   it sends power-on only. Bench mode listens for 2 s before transmitting and stays silent
   if it hears real bus traffic, so a board left in bench mode and reinstalled in the car
   never spams the live bus.

New in v2: an **ELM327-style AT command surface** on the hardware UART, so a laptop --
later an ESP32-S3 with a touch display -- can send raw CAN frames and read the replies.
The board stays a dumb frame pipe: no ISO-TP reassembly, no KWP2000 parsing, no PID
decoding on the AVR. There are 2 KB of RAM in there; the host can do the thinking.

## The autonomy guarantee

The four features above are **autonomous and take priority**. They are driven only by
what the modules see on the bus, they never wait on the serial link, and the AT surface
cannot command them -- there is no AT command to change the radio source, toggle the VES
announce, or power the Bluetooth module. Unplug the link, or plug in a laptop running
buggy software, and the car behaves exactly as it did before. The one exception is
`AT@BENCH`, which toggles bench mode at runtime, and that exists only because it used to
be a `#define` and a reflash.

## Hardware

* Arduino Pro Mini, 5 V / 16 MHz (ATmega328)
* MCP2515 + MCP2551 CAN module, **16 MHz** crystal (`mcp_can` is patched for 83.3 kbps)
* OVC3860-based Bluetooth A2DP module -- generic XS3868 / H166
* USB-TTL adapter for the AT link, **switched to 5 V logic**
* An ICSP programmer -- an **Arduino running the stock ArduinoISP sketch** (Mega 2560 or a
  CH340 Uno/Nano both work here); see `platformio.ini` for the exact `avrdude` invocation

Wiring, from `lib/Core/Config.h`:

| Signal | Pin | Note |
|---|---|---|
| MCP2515 CS | D10 (PB2) | SPI otherwise on the usual pins |
| BT module power | D3 (PD3) | LOW = on (active low); was PC1 pre-rework |
| BT module RX | A2 (PC2) | through a Schottky diode -- the module is 3.3 V, the board is 5 V |
| BT module TX | A4 (PC4) | wired but never read |
| AT link | D0 / D1 | hardware UART, 115200 8N1 |

## Build and flash

The board is flashed **over the ICSP header**. There is no serial bootloader, which is
why pins 0/1 can be dedicated to the AT link and why the board is not reset when you open
the serial port.

```sh
pio run -e promini                 # build
pio run -e promini -t upload       # flash over ICSP
```

Flashing uses a custom `upload_command` in `platformio.ini`: an Arduino-as-ISP
(`-c arduino_as_isp`, `-p m328p`) on a serial port -- **that `-P` is the *programmer's*
port, not the target's**. The pin table and wiring notes, and the board's ICSP-RESET
errata (the ISP header's RESET does not reach the MCU -- a recurring `00 00 00` signature
if that wire is loose), are commented inline in `platformio.ini`. Use **avrdude 8.2 from
scoop**; the Arduino IDE's copy has no `arduino_as_isp` definition.

Host tests for the parts that do not touch hardware (line parsing, ISO-TP flow control):

```sh
pio test -e native                 # needs gcc/g++ on PATH
sh scripts/run-host-tests.sh       # or this, if you only have clang
```

The `native` environment builds nothing from `src/`; it is there so the fiddly logic can
be tested on a real machine instead of by flashing a car. PlatformIO's native platform
hard-codes the names `gcc`/`g++`, and an `extra_scripts` override does not stick for
`pio test`, so `scripts/run-host-tests.sh` compiles and runs the same two suites with
whatever compiler it can find (honouring `CC`/`CXX`). Both paths run the identical
tests -- 38 of them at the time of writing.

## Folders

```
src/                main.cpp -- wiring up, the module array, the main loop
lib/Core/           contracts and constants: CanFrame, ICanBus, Module, Events,
                    Config (pins/tuning), JeepCan (CAN ids and payloads), Version
lib/CanDriver/      MCP2515 driver (patched mcp_can) behind ICanBus
lib/AtServer/       the AT command surface: line parsing, responses, events
lib/IsoTpFc/        ISO-TP frame classification and the auto flow-control frame
lib/Modules/        the autonomous features: VES/AUX, BT control, bench mode
test/               host-side unit tests (pio test -e native)
scripts/            run-host-tests.sh (hosts without gcc);
                    bench_vin_test.py (replay power-on + VIN over the AT link to
                    validate a radio on the bench without flashing);
                    send_frame.py (send one raw CAN frame through the gateway)
docs/               see below
```

## Docs

* [docs/AT-COMMANDS.md](docs/AT-COMMANDS.md) -- the protocol reference. Wiring, the full
  command table, response conventions, why `ATCAF0` matters, and how flow control works
  (with a worked multi-frame example). Enough to drive the board from a plain serial
  terminal.
* [docs/CAN-B-CAPABILITIES.md](docs/CAN-B-CAPABILITIES.md) and
  [docs/CAN-IHS-MAP.md](docs/CAN-IHS-MAP.md) -- what the interior bus exposes and the
  decoded id map.
* ICSP flashing notes -- inline in `platformio.ini` (pin table, the ICSP-RESET errata,
  the avrdude 8.2 requirement, and reading the device signature to verify wiring before a
  flash).
* [docs/PROBE-LOG.md](docs/PROBE-LOG.md) -- worksheet for the first bring-up session in
  the car: an ordered probe script, the module map, and empty tables to fill in by hand.

## No serial debugging

Worth repeating, because it catches everyone: the UART is a protocol port now, so there is
no `Serial.print`. Diagnostics come out as `AT@EV` event lines (`AT@EV1` to enable) and
from `AT@ST` on demand.

## Credits and licence

Written by Anton Viktorov, <https://github.com/latonita/jeep-canbus>. This replaces the
2015-2017 `jeep-aux-and-extra-features` sketch by the same author, which is still in the
parent repo and is where every CAN id and payload in `lib/Core/JeepCan.h` was worked out
in the first place.

Free software under the MIT licence -- see [LICENSE](../LICENSE) in the parent repo.
