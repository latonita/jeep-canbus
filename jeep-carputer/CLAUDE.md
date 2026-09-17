# jeep-carputer

Car-puter for a 2005 Jeep Grand Cherokee WH (3.0 CRD / OM642, Bosch EDC16CP31)
on a Waveshare ESP32-S3-Touch-LCD-2.8 (240×320 portrait, **V1 board → CST328
touch**). Six screens, portrait and landscape toggled at runtime.

The companion repo `../jeep-aux-and-can-gateway/` is an ATmega328 in the radio:
a **dumb frame pipe** with an ELM327-style AT surface
(`../jeep-aux-and-can-gateway/docs/AT-COMMANDS.md`). ISO-TP, KWP and all
decoding happen here. Bus is interior CAN-B, 83.3 kbps, ~151 frames/s, ~30 % of
the 115200 link. See also `../README.md` and `ARCHITECTURE.md`.

## Layout

| | |
|---|---|
| `core/` | **pure C++**, no ESP-IDF, host-tested. Logic AND policy both live here |
| `src/` | ESP-IDF only: At328Link, Display, Touch, Ui, Settings, UiTheme, FaultStore, main |
| `design/` | `.dc.html` artboards at true size, `canvas.json`, `build.py` |
| `test/` | auto-discovered by the runner — a new `test_*/` dir registers itself |

Keep logic in `core/` and it gets tested without a car. That is the whole point.

## Build / flash / test

```bash
# firmware: Windows PlatformIO, in the project root
pio run -e s3
powershell -File scripts\flash.ps1              # flash + monitor, COM18
powershell -File scripts\flash.ps1 -MonitorOnly # just watch

bash scripts/run-host-tests.sh                  # all of core/, on the laptop
HOSTTEST_SAN=0 bash scripts/run-host-tests.sh   # ...without the sanitizers

# desk testing with no car — emulates the 328 AND the modules behind it
python scripts/emulate-car.py --port COM5                      # requests only
python scripts/emulate-car.py --port COM5 --log playback --loop
python scripts/emulate-car.py --port COM5 --log drive          # the real drive

# gateway (ATmega328, ICSP via Arduino-as-ISP on COM19, radio must come out)
cd ../jeep-aux-and-can-gateway && pio run -e promini
"C:\Users\latonita\scoop\apps\avrdude\current\avrdude.exe" -c arduino_as_isp \
  -p m328p -P COM19 -U flash:w:.pio\build\promini\firmware.hex:i
```

`powershell`, **not `pwsh`** (5.1 only).

## Hard rules

- **`lib_ldf_mode = off` is load-bearing.** Dependencies come from
  `src/idf_component.yml` into `managed_components/`; with the LDF on,
  PlatformIO also scans `~/.platformio/lib`, finds an LVGL 8 there and compiles
  it alongside 9.5.
- **Do NOT use `sed -i` on `src/`.** MSYS sed reads text and writes binary, so it
  silently strips every CR from a CRLF file. `core.autocrlf=true` means **git
  will not show you this happening**, and `cat -A` / `grep -c $"\r$"` both lie
  about it for the same reason — `file src/Ui.cpp` is the reliable check, and
  `perl -pi -e "s/\r?\n/\r\n/"` puts it back. **Patch with Python**, reading the
  bytes and writing the file's own ending back. This repo is CRLF and the gateway
  repo is LF; some `core/` files are LF even here, so check per file.
- **Heredoc → Python mangles `\0` into a real NUL byte.** Bitten twice, once
  producing a "binary file" that still compiled. Use raw strings or `chr(92)`,
  and assert `count(b'\x00') == 0` after every write.
- **A NEW file in `core/` or `src/` needs `touch src/CMakeLists.txt`** — it globs,
  and the symptom is a confusing undefined reference.
- **NEVER `uart_flush_input()` once the reader task owns the port.** It deadlocks
  the link inside `begin()` before `ATZ`, with a silent console and a dashed
  panel — indistinguishable from an unplugged gateway. `drain()` enforces the
  rule; do not add a fourth call site.
- **Never `ESP_LOGx` while holding `lvgl_port_lock`** — USB-Serial-JTAG blocks
  until the host drains and stalls the renderer.
- **LVGL's pool is fixed and its allocation failure is a bare `while(1)`** — a
  silent white-screen freeze, not an error. Free the old tree before building a
  new one and watch the `lvgl heap` line.
- **Hold-and-dim missing data.** Dashes are only for explicit unavailable
  sentinels; a frozen-but-plausible number is the failure mode that gets
  believed.
- Touching a header = 8–12 min full ESP-IDF rebuild; a `.cpp` is ~60 s. Decode
  panics with `addr2line`, do not guess.

## Where the rest is written down

`.claude/rules/*.md` load automatically when you touch matching files —
`build-and-toolchain`, `core-architecture`, `can-signals`, `link-and-gateway`,
`ui-lvgl`, `ui-layout`, `faults-and-dtc`, `design-artboards`.

- **Before changing what the instrument shows or how it behaves, read
  `docs/product-decisions.md`** — settled calls, several of them the owner's, not
  to be relitigated.
- **`docs/open-questions.md`** — what is unverified, unmeasured, or waiting on a
  drive, a receipt or an answer from the owner. Check it before claiming
  something is broken.
- **`D:\Repos\car\jeepdiag` is a KNOWN-WORKING diagnostic reference** against this
  vehicle over this same gateway. When something diagnostic fails here, diff
  against it before theorising.
