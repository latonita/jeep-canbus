# jeep-carputer — working notes

Car-puter for a 2005 Jeep Grand Cherokee WH (3.0 CRD / OM642, Bosch EDC16CP31),
on a Waveshare ESP32-S3-Touch-LCD-2.8 (240×320 portrait, **V1 board → CST328
touch**). See also `../README.md` and `../ARCHITECTURE.md`.

## Two repos, one system

```
jeep-carputer/            ESP32-S3 dashboard  (this repo)
jeep-aux-and-can-gateway/ ATmega328 in the radio: VES/AUX + CAN-B gateway
```

The 328 is a **dumb frame pipe** with an ELM327-style AT surface
(`../jeep-aux-and-can-gateway/docs/AT-COMMANDS.md`). ISO-TP, KWP and decoding
all happen here. Bus is **interior CAN-B, 83.3 kbps**, measured at **151
frames/s**, ~22 % of the 115200 link.

## Build / flash

```bash
# firmware: build in WSL, flash from Windows (WSL2 has no COM ports)
wsl -- bash -lc 'cd /mnt/d/Repos/car/jeep-canbus/jeep-carputer && pio run -e s3'
powershell -File scripts\flash.ps1              # flash + monitor, COM18
powershell -File scripts\flash.ps1 -MonitorOnly # just watch

bash scripts/run-host-tests.sh                  # all of core/, on the laptop

# gateway (ATmega328, ICSP via Arduino-as-ISP on COM19, radio must come out)
cd ../jeep-aux-and-can-gateway && pio run -e promini
"C:\Users\latonita\scoop\apps\avrdude\current\avrdude.exe" -c arduino_as_isp \
  -p m328p -P COM19 -U flash:w:.pio\build\promini\firmware.hex:i
```

`powershell`, **not `pwsh`** (5.1 only). `not in sync: resp=0x00` on the first
avrdude try is the programmer resetting — just run it again.

**Desk testing without a car:** `python scripts/replay-lawicel.py capture.log
--port COM5 --loop` answers the AT handshake and streams a LAWICEL capture into
GPIO44 at 115200. Polled values (fuel %, TPMS, lamps) stay dashed — they are
request/response, not broadcast.

## Layout

| | |
|---|---|
| `core/` | **pure C++**, no ESP-IDF, host-tested. Broadcast, IsoTp, Kwp, Trip, SignalDb, LinkLine |
| `src/` | ESP-IDF: At328Link, Display, Touch, Ui, Settings, UiTheme, main |
| `design/` | `.dc.html` artboards at true 240×320, `canvas.json`, `build.py` |
| `test/` | auto-discovered by the runner — a new `test_*/` dir registers itself |

Keep logic in `core/` and it gets tested without a car. That is the whole point.

## Gotchas that cost real time

- **CRLF here, LF in the gateway repo.** Read with universal newlines, write
  with the repo's own ending, or every patch silently misses.
- **Heredoc → Python mangles `\0` into a real NUL byte.** Bitten twice, once
  producing a "binary file" that still compiled. Use raw strings or `chr(92)`,
  and assert `count(b'\x00') == 0` after every write.
- **`src/CMakeLists.txt` globs.** A NEW file in `core/` or `src/` needs
  `touch src/CMakeLists.txt` or it will not be compiled and you get a confusing
  undefined reference.
- **Touching a header = 8–12 min full ESP-IDF rebuild.** A `.cpp` is ~60 s.
- **Decode panics with addr2line**, do not guess:
  `~/.platformio/packages/toolchain-xtensa-esp-elf/bin/xtensa-esp32s3-elf-addr2line -pfiaC -e .pio/build/s3/firmware.elf <addr>`

## Things that are true and non-obvious

- **`0x1A7` is already a RATE.** `Q = b0<<8|b1`, `L/h = Q/160`. Integrate
  time-weighted per frame; never multiply by rpm, never sum samples. `0xFFFF` =
  unavailable (drop the interval), `0` = valid overrun cut-off.
- **Distance is the `0x014` odometer differenced**, 0.1 km/bit — not integrated
  speed.
- **LVGL has a FIXED 64 KB pool** (`CONFIG_LV_MEM_SIZE_KILOBYTES`, expand 0) and
  `LV_USE_ASSERT_MALLOC=y`, whose handler is a bare `while(1)`. Allocation
  failure is a **silent freeze**, not an error. When rebuilding the UI: free the
  old tree *before* building the new one, and watch the `lvgl heap` log line.
- **Never `ESP_LOGx` while holding `lvgl_port_lock`.** USB-Serial-JTAG blocks
  until the host drains; it stalls the renderer and the instrument lowers the
  measurement it is taking.
- **The link is single-requester by construction** (only `carTask` transacts).
  No mutex — a lock inside each method would not make a *transaction* atomic.
- **`ready()` is not liveness.** It stays true after the cable is pulled; the
  watchdog uses silence + an `AT@ST` probe, and re-runs `begin()` to recover.
- **Palettes are GENERATED**, not hand-picked:
  `cd design && python daypalette.py && cp palettes.inc ../src/UiPalettes.inc`.
  Same script renders `daylight-local.html`, so firmware and design cannot
  disagree. Teal sits at **OKLCH hue 151°** (an old comment said 178 — wrong).
- **`0x210` byte 1 reads `0x00` for BOTH** the bottom of the dimmer wheel and
  all-lights-off, which want opposite brightness. `0x1C8` separates them, and is
  wired to only ever *brighten* so an absent signal costs nothing.
- Fonts are Barlow Condensed (digits) / Barlow Semi Condensed (labels), matching
  the artboards. Regeneration commands are in `src/fonts/fonts.h`.

## Product decisions, settled — do not relitigate

- Driver-glanceable, **not** a parked diagnostic tool.
- **TPMS is the hero feature.** Tyre pressure in **bar**. Corner order is still
  UNMAPPED and naming them would be a guess worse than no name.
- The radio is invisible; the AT surface cannot touch VES/AUX or Bluetooth.
- Brightness from the CAN dash dimmer, **fail bright**.
- Read the maximum from broadcasts; poll only what does not broadcast.
- Hold-and-dim missing data; show dashes only for explicit unavailable
  sentinels. A frozen-but-plausible number is the failure mode that gets
  believed.
- **There is deliberately NO idle timeout.** A page changes only when a finger
  changes it.
- Screens: TRIP, DRIVE, TYRES, HEALTH, SETTINGS — boot settles on **DRIVE**
  (`UI_HOME`), reachable from either direction in one swipe.

## Open / unverified

- Gateway **2.1** (`AT@SM` duplex streaming) is built and flashed; the carputer
  probes it and falls back to ATMA windows against 2.0.
- `0x1C8 == 0x02` for "lights on" is a **one-sample guess**; the backlight log
  prints every input so one lights-off toggle settles it.
- Q/160 has never been checked against fuel actually paid for — a fill-to-fill
  would settle it.
- `avg render ~66 ms` during page scrolls, not yet investigated.
- No link-failure screen; HEALTH lamp pills drawn but not driven; FAULTS and
  ALERT designed but not built (no vertical axis).
- `docs/ISP-WIRING.md` is referenced by the gateway's `platformio.ini` and does
  not exist.
