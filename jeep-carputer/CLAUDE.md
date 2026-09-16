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
# firmware: Windows PlatformIO, in the project root
pio run -e s3
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

**`lib_ldf_mode = off` is load-bearing.** Every dependency comes from
`src/idf_component.yml` into `managed_components/` — LVGL **9.5**,
esp_lvgl_port, esp_lcd_touch_cst328. PlatformIO's Library Dependency Finder
knows nothing about that and also scans `~/.platformio/lib`; this machine has an
LVGL **8** there, so with the LDF on it compiles v8 alongside and dies on
`lv_btnmatrix_ctrl_t` and a wall of v8-only symbols. The build used to live in
WSL, whose PlatformIO home happens to have no such library — which is the only
reason this never showed up before the move to Windows.

**The move also bumped ESP-IDF, silently.** `dependencies.lock` went 5.3.2 →
5.5.4, because the Windows PlatformIO ships a newer `framework-espidf` than the
WSL install did. LVGL did not move — same 9.5.0, same component hash. The build
is clean and the board boots on it, but that is a whole IDF minor arriving as a
side effect of changing machines, so suspect it first if something that used to
work stops.

**Desk testing without a car:** `python scripts/replay-lawicel.py capture.log
--port COM5 --loop` answers the AT handshake and streams a LAWICEL capture into
GPIO44 at 115200. Polled values (fuel %, TPMS, lamps) stay dashed — they are
request/response, not broadcast.

## Layout

| | |
|---|---|
| `core/` | **pure C++**, no ESP-IDF, host-tested. Broadcast, IsoTp, Kwp, Trip, Accel, SignalDb, LinkLine |
| `src/` | ESP-IDF: At328Link, Display, Touch, Ui, Settings, UiTheme, main |
| `design/` | `.dc.html` artboards at true size — 240×320, plus a `*.landscape.dc.html` band at 320×240 — `canvas.json`, `build.py` |
| `test/` | auto-discovered by the runner — a new `test_*/` dir registers itself |

Keep logic in `core/` and it gets tested without a car. That is the whole point.

## Render speed and swipe feel — measured, not guessed

A full-screen frame went **65 ms -> 29 ms**, idle **4.5 -> 2.8 ms**, page scrolls
**73-78 -> ~43 ms**. Benchmark without touching the glass: the splash-to-root
fade forces full-screen frames, and `avg render` for them is the first two
`ui:` lines after boot.

The panel was never the problem. 240x320x16bpp at 80 MHz SPI is 15.4 ms, under a
quarter of the old frame; the rest was the memory system and the scheduler.

- **`CONFIG_LV_ATTRIBUTE_FAST_MEM_USE_IRAM=y`** and **`CONFIG_ESPTOOLPY_FLASHMODE_QIO=y`**
  are most of the win. LVGL's blend/mask inner loops were executing XIP from
  DIO flash at ~20 MB/s, alongside 69 KB of uncompressed 4 bpp glyph `.rodata`
  re-streamed every frame. Idle frames were fast only because one small label
  stayed cache-resident — that 4.5/65 cliff is a cache signature, not a draw
  cost.
- **`CONFIG_LV_DRAW_SW_DRAW_UNIT_CNT=1`.** LVGL creates draw threads with plain
  `xTaskCreate` — **no affinity** — at prio 3, under `carTask`'s 5 on core 0. So
  a worker lands on core 0 and the link task's long blocking UART reads preempt
  it. `uiInit()` pins the esp_lvgl_port task to core 1, but that is the
  coordinator; with `LV_USE_OS` the pixels are drawn on those threads. Pinning
  the wrong task is why the inversion survived, and it is what the 11 ms
  frame-to-frame variance was.
- **NOT `CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB`.** It links, it flashes, and the
  board then boot-loops — silently, because the console is native
  USB-Serial-JTAG so every reset tears the port down before anything prints.
  The cache is carved from internal SRAM at startup, so nothing catches it at
  link time. Symptom to recognise: `esptool` can still read the flash ID, but a
  console capture returns zero bytes and the COM port keeps dropping.

**The swipe snap-back was the touch driver, not the scroller.** LVGL builds
throw velocity from the last eight per-poll deltas, weighted `1 - t/99ms` and
zero beyond that. `TOUCH_RELEASE_HOLD_MS` kept reporting PRESSED at *unchanged*
coordinates after the lift, injecting ~4 zero-motion samples and ageing every
real one 60 ms into that decay — so a flick's measured velocity collapsed 5-10x,
LVGL's predicted landing fell short of the half-page it needs, and the page went
back. Once INT has proven itself a level the hold drops to one poll
(`TOUCH_RELEASE_HOLD_INT_MS`), and `lv_indev_set_scroll_throw(indev, 4)` covers
what the controller's sub-poll frame rate still eats.

- **`LV_OBJ_FLAG_SCROLL_ONE` replaced a hand-rolled clamp** on the throw
  animation. The clamp limited only the throw, so a long drag still slid past
  two pages under the finger; and it anchored on a *rounded* `scroll_x` sampled
  at SCROLL_BEGIN, which is an intermediate value when a new drag starts during
  an in-flight settle — so a quick second flick got clamped back to the page it
  was leaving. A second snap-back, of our own making.
- **Every page settle was a fixed 400 ms.** LVGL derives it from a speed
  encoding that always saturates `SCROLL_ANIM_TIME_MAX` for a full-page move,
  and there is no Kconfig for it. `UI_PAGE_ANIM_MS` is written straight onto the
  animation in `onScrollBegin`; `resolve_time()` runs after that callback, so it
  wins.
- **`LV_EVENT_SCROLL_END` fires TWICE**, the first from the same call that only
  *starts* the animation, before `scroll_x` has moved. So the page dot showed
  the page being left for the whole settle and then corrected — which reads as
  the swipe not having taken. The dot is now set to the DESTINATION in
  `onScrollBegin`.

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

- **The flow bar is 50 L/h full scale and turns critical above 35 L/100km.**
  The threshold is on the PER-DISTANCE figure and only while the readout is
  actually in L/100 — stopped there is no distance to divide by and an idling
  engine would sit permanently in alarm. It is a rate of spending, not a fault.
  `UI_FLOW_FULL_LPH` is a hard CLAMP, not an auto-range — 3.125 L/h a segment,
  sixteen lit at 50 and nothing above it — because a bar whose full scale moves
  means something different every time the driver looks at it. The NUMBER
  beside it is not clamped and must not be: the bar is the glance, the number
  is the fact.
- **The flow bar's pitch is DERIVED from the band, and the gap is not.** Both
  bands keep portrait's 2px gap; portrait is 12 wide on 14 (222px) and
  landscape 10 on 12 (190px). Landscape shipped at 8 on 10 = 158px — four
  fifths of the width where portrait fills it — because its comment sized it
  against a **165px** left column that `UI_LS_RIGHT_W` going 130 → 112 had
  already made **187px**. `16w + 15g = 187` has no usable integer answer, so the
  bar deliberately overhangs the content edge by 3px to x=200, exactly as
  portrait's overhangs 230 to 232, and still clears the column rule at 207 by
  7px. **If a landscape width ever moves again, this bar does not follow on its
  own.**
- **`0x1A7` is already a RATE.** `Q = b0<<8|b1`, `L/h = Q/160`. Integrate
  time-weighted per frame; never multiply by rpm, never sum samples. `0xFFFF` =
  unavailable (drop the interval), `0` = valid overrun cut-off.
- **Distance is the `0x014` odometer differenced**, 0.1 km/bit — not integrated
  speed.
- **The LVGL pool is 96 KB, and BOTH ends of that are a hard wall.** It was 64,
  which built the six-page tree and then could not allocate a DRAW TASK on top
  of it: `lv_draw_add_task` -> `lv_malloc_zeroed` -> `LV_ASSERT_MALLOC` ->
  `while(1)`, on taskLVGL, with the panel still white and no message. The
  symptom is a WHITE SCREEN, no touch, carTask still decoding frames happily,
  and `task_wdt` naming `CPU 1: taskLVGL` every five seconds forever. Decode the
  backtrace; it lands in `lv_draw.c`. Raising it to **128 went too far in the
  other direction** -- the pool is `.bss`, so it comes out of the same internal
  SRAM the two 38 KB draw buffers are allocated from, and `esp_lvgl_port` then
  logs `Not enough memory for LVGL buffer (buf2) allocation!` at boot and there
  is no display at all. That failure then shows up as `main` wedged in
  `uiSplash` -> `lv_screen_load_anim`, which is misleading: the splash is fine,
  the display was never created. 96 clears both. Check `RAM:` in the build
  output and that boot line before trusting any change here.
- **The FAULTS drill-down is built ON DEMAND and deleted on the way out.** It
  used to be built with the tree and merely hidden -- ~42 objects resident for a
  page opened deliberately, from a standstill, maybe once a month. A hidden page
  does not draw but it still OCCUPIES, and what ran out was what RENDER needs,
  not what build needs. `faultPageShow()` owns both halves and nulls
  `W.faultTitle` / `W.faultRow[]` / `W.faultState[]` on the way out, because a
  dangling pointer that is only safe because of a guard somewhere else is the
  next person's crash.
- **LVGL has a FIXED pool** (`CONFIG_LV_MEM_SIZE_KILOBYTES`, expand 0) and
  `LV_USE_ASSERT_MALLOC=y`, whose handler is a bare `while(1)`. Allocation
  failure is a **silent freeze**, not an error. When rebuilding the UI: free the
  old tree *before* building the new one, and watch the `lvgl heap` log line.
- **The touch release hold must follow the GESTURE, not a constant.** It
  bridges the controller's slow reporting so a press survives; shorten it and a
  single contact is chopped into several. Flat 20 ms broke both gestures on the
  TRIP page at once — two CLICKEDs inside the double-tap window so one tap
  rotated the meter, and no press ever reached the 400 ms `LV_EVENT_LONG_PRESSED`
  needs, so reset became unreachable. `s_intProven` is no defence: it takes ONE
  observation of INT low, which can happen during motion and says nothing about
  a still fingertip. So: moved in the last 200 ms → release promptly (a swipe
  ending, and every held-still sample is a zero-motion entry in LVGL's throw
  history); otherwise → bridge with the full 60 ms.
- **LVGL suppresses `LV_EVENT_CLICKED` when a press becomes a scroll**
  (`lv_indev.c`: `if(scroll_obj == NULL)`), and every page here lives inside a
  horizontal scroller. So any latch cleared only on CLICKED gets stuck: a
  fingertip that drifts past the scroll limit on the way up produces the long
  press and then no click at all. Clear those on `LV_EVENT_PRESSED`, which
  always arrives — `s_priceHeld` and `s_tripLongFired` both do. For the same
  reason the fuel price commits from `uiTask` on a dirty flag rather than from
  the click, or a held adjustment that ends in a swipe would never reach NVS
  and would silently revert on the next boot.
- **Never `ESP_LOGx` while holding `lvgl_port_lock`.** USB-Serial-JTAG blocks
  until the host drains; it stalls the renderer and the instrument lowers the
  measurement it is taking.
- **The link is single-requester by construction** (only `carTask` transacts).
  No mutex — a lock inside each method would not make a *transaction* atomic.
- **`ready()` is not liveness.** It stays true after the cable is pulled; the
  watchdog uses silence + an `AT@ST` probe, and re-runs `begin()` to recover.
- **`Tyres*.dc.html` is the ASPIRATIONAL board** (LEFT FRONT / RIGHT FRONT …);
  `TyresUnmapped*.dc.html` is what ships (SENSOR 1-4). They are meant to
  disagree with each other — do not "fix" the mapped one against the firmware.
  Both landscape boards also draw a per-cell `LOW` chip that the firmware does
  not have; the firmware colours the value instead. The two boards now differ
  only in the NAMES — both are 2×2 in both bands.
- **Palettes are GENERATED**, not hand-picked:
  `cd design && python daypalette.py && cp palettes.inc ../src/UiPalettes.inc`.
  Same script renders `daylight-local.html`, so firmware and design cannot
  disagree. Teal sits at **OKLCH hue 151°** (an old comment said 178 — wrong).
- **`0x210` byte 1 reads `0x00` for BOTH** the bottom of the dimmer wheel and
  all-lights-off, which want opposite brightness. `0x1C8` separates them, and is
  wired to only ever *brighten* so an absent signal costs nothing.
- Fonts are Barlow Condensed (digits) / Barlow Semi Condensed (labels), matching
  the artboards. Regeneration commands are in `src/fonts/fonts.h`.

## Landscape 320×240 — built, and how it rotates

`SETTINGS → ORIENTATION` flips it at runtime and NVS remembers it. Same five
peers, same order, same one-flick-one-page rule, same absence of an idle
timeout — a settings row, **not a second product**. FAULTS and ALERT are still
artboards only, in both bands.

**The rotation is one call.** `lv_display_set_rotation(s_disp, ROTATION_90)` in
`uiApplyOrientation()` does all of it:

- LVGL swaps the resolution it reports, which is what `UI_W` / `UI_H` read.
  Below the geometry block in `Ui.cpp`, `LCD_H_RES` / `LCD_V_RES` mean the
  *panel* and appear only in the buffer sizing and the port config.
- `esp_lvgl_port` hears `LV_EVENT_RESOLUTION_CHANGED` and writes
  `swap_xy`/`mirror` into the ST7789 — composed against `disp_cfg.rotation`,
  which is all-false, so landscape resolves to `swap_xy(true) + mirror(false,
  true)`.
- `lv_indev` runs every touch point through `lv_display_rotate_point()` before
  a widget sees it.

**So `Touch.cpp` is not part of the rotation and must not be made part of it.**
The old worry that the panel mirror and the touch flags "must move together"
is closed by construction, in the opposite direction to the one expected: only
one of the two is ever told, because the digitiser is glued to the glass and
does not turn. Adding a `swap_xy` there to "match" would rotate the input
twice.

**Corrected while doing this:** `disp_cfg.rotation` all-false is NOT a no-op
that "cancels out" Display.cpp's `mirror(true, false)`.
`lvgl_port_add_disp()` calls its rotation update unconditionally, so those
three zeroes are written into MADCTL and *replace* what Display.cpp set. They
are the panel's orientation. Behaviour is unchanged — only the comment was
wrong.

**Free before build, still.** `rootTeardown()` / `rootRebuild()` are now the
one path for both the palette swap and the flip; the rotation happens in the
gap between them, with one bare screen alive. That gap is the only safe moment
for it — `lv_display_set_rotation()` resizes every screen on the display and
dirties every layout.

**Measured on the board**, which is what the `lvgl heap` line is for: portrait
`39360/59984 used, frag 8%, largest free 18984`; landscape `39308/60024`. Those
numbers are from the 64 KB pool and are the reason it is now 96 -- ~20 KB of
headroom looked healthy and was not, because draw tasks come out of the same
pool at render time and never appear in a figure logged at BUILD time.
Landscape is marginally *cheaper* — the dot rail folded away. Six rebuilds
back to back (two flips, four palette cycles) oscillate inside a 76-byte band
with no monotonic growth, so the teardown leaks nothing. ~20 KB headroom is
healthy; if that line ever shows `largest free` dropping toward a page's worth
of widgets, the freeze is coming.

Reasoning for the layout lives in the `landscape-*` notes in `canvas.json`; the
short version:

- **The dot rail folds into the status bar.** 26px is 8 % of portrait's height
  and 11 % of landscape's — off the only axis that is short. One 22px bar
  carries clock, dots and link, and content gets 218px instead of 192.
- **Two columns replace the stack**: hero on the left, the same values in a
  112px right stack, insets of 10px. It was 189/130 at 12px insets until TRIP's
  unit came down out of the header. **The two columns share a fixed total**:
  content = band + stack = `(UI_W - 1 - 2*PAD) - PAD`, which is 271px at PAD 12
  and 279 at PAD 10 *whatever the divider does* — so the only way to give the
  hero band more is to take it from the inset. TRIP needs 187 (a 164.2px
  `247.8` plus an 18.6px `KM`) and the stack needs 92 (`4163` + `rpm` = 90.1);
  279 covers both, 271 does not. Every narrowed stack string was measured, and
  DRIVE's distances being whole km is what keeps its worst at `9999 km` = 88.9. Nothing dropped, added or reordered — that is what
  makes a runtime toggle safe rather than a second design to keep in sync.
- **No new type.** Every numeral is `font_digits_64` or `font_digits_30`, every
  word `font_label_14`. Enforced, not lucky: four tyre pressures at hero size
  want 238px of the 218 available, so they were drawn and then thrown out.
- **SETTINGS is the one page that cannot take two columns.** With a single 14px
  label face, ORIENTATION plus its chip is ~205px and ADAPTER plus its value
  ~106px — 351px of a 320px board. Controls keep full width; INFO became a
  bottom strip, which portrait's 240px could not have held.
- **TYRES landscape is a 2×2**, reading order: `SENSOR 1` `SENSOR 2` on top,
  `SENSOR 3` `SENSOR 4` beneath. It was four columns in index order, which was
  the more conservative arrangement — a grid implies front/rear and left/right,
  and `21 3B` gives four bytes in a fixed order with nothing saying which wheel
  is which, so four across made no claim where a 2×2 invites one. **Changed on
  the owner's explicit instruction after that was put to them**, which makes it
  their call and not drift. The cells still read `SENSOR`, never LF/RF/LR/RR, so
  the layout suggests a shape while the labels refuse to name a corner — and
  nothing here is a mapping until one is measured.
- **FAULTS goes to one line per fault.** Two-line rows fit three in 148px, and
  16 stored codes would then need six screens. State moved to a 3px coloured
  left edge to buy back the width; the longest descriptions ellipsise.
  *(Designed, not built — no vertical axis in either band.)*

### The artboards are 30 % narrower than the firmware

The `.dc.html` boards are CSS Barlow Condensed; Barlow figures are ~0.727 em,
so `font-size:66px` renders 48px digits while `font_digits_64` renders 64. Do
not take a coordinate off an artboard without re-measuring the string. Three
places on the landscape band only fit once measured, and would have shipped
overlapping:

- DRIVE's hero unit is **`L/100`, not `L/100km`**. The long form right-aligns
  to x=122 and the hero reaches that at four glyphs — which FROM START really
  produces, since a cold crawl out of a driveway reads 40-something until the
  first kilometre is behind it.
- TRIP has **NO HEADER ROW** — the only page without one. It held the page name
  and a `KM` key, both sitting on top of the hero. The name went redundant the
  moment the selector arrived (three meters listed with one lit says which card
  you are on better than a word does), and the unit belongs to the number. The
  row's space went to the selector. `KM` now sits on the **hero's baseline**,
  right-aligned to the column edge so it holds still while the number's width
  changes, and it is drawn with **`mkUnit` (font_label_14), not `mkUnitKey`** —
  `font_label_20` exists for unit KEYS parked in a header, and this is no
  longer one. Baselines are computed, not eyeballed: `font_digits_64` is
  line_height 73 / base_line 1 so its baseline is 72px down, `font_label_14` is
  16/3 so its own is 13, hence `hero_y + 72 - 13`.
- TRIP's **selector row is sized by the LANDSCAPE band**: three 54px cells are
  162px of the left column. Portrait has 218px before the `KM` key and would
  have taken 60px cells happily; sizing to the narrow band and using the result
  unchanged in both is what keeps the rotation a toggle. (This replaced a
  two-line gesture hint, which stacked because side by side it was 182px of
  that same band.) **The 165px this was measured against is STALE** — that was
  PAD 12 with `UI_LS_RIGHT_W` 130; the shipped column is **187px** (PAD 10,
  stack 112). Nothing here breaks, since everything cited fits a wider band
  than it was measured in, but 165 is not a number to size anything new
  against — it is what left the flow bar short.

`scripts/` has no checker for this; the measurement was done by parsing
`adv_w` out of `src/fonts/*.c` (1/16 px units, plus `letter_space` between
glyphs but not after the last).

## Product decisions, settled — do not relitigate

- Driver-glanceable, **not** a parked diagnostic tool.
- **TPMS is the hero feature.** Tyre pressure in **bar**. Corner order is still
  UNMAPPED and naming them would be a guess worse than no name.
- **Tyre pressure is polled ONCE A MINUTE** (corners and spare, on the owner's
  instruction; it was 5 s and 30 s). A tyre loses pressure over days and the
  sensors themselves only transmit every ~60 s parked, so a 5 s request was
  mostly re-reading the WCM's held copy — four of the five requests in `kPlan`
  were buying nothing. **`UI_TYRE_TTL_MS` / `UI_SPARE_TTL_MS` in `Ui.cpp` are
  twice the poll interval and must move with it**, or the page dims a reading
  that is as fresh as it will ever be — the readouts and the low-corner colour
  both use it. And **`PollItem::periodMs` is a `uint16_t`**: 65535 ms is the
  longest period `kPlan` can express, and 120000 there is a `-Wnarrowing`
  build error, which is why the spare shares the corners' rate rather than
  staying slower than them.
- The radio is invisible; the AT surface cannot touch VES/AUX or Bluetooth.
- Brightness from the CAN dash dimmer, **fail bright**.
- Read the maximum from broadcasts; poll only what does not broadcast.
- Hold-and-dim missing data; show dashes only for explicit unavailable
  sentinels. A frozen-but-plausible number is the failure mode that gets
  believed.
- **There is deliberately NO idle timeout.** A page changes only when a finger
  changes it.
- Screens: **DRIVE, TRIP, TYRES, HEALTH, SETTINGS** — DRIVE is index 0 and
  where the boot settles (`UI_HOME`). It used to sit second so it was reachable
  from either direction in one swipe; that was given up deliberately, because
  the card wanted most often should cost NO flicks, and a hard end cannot be
  overshot. SETTINGS stays last and now costs four flicks, which is the point.
- **Three trip meters**: TRIP A and TRIP B both persist (two NVS keys), FROM
  START clears on every boot. Two resettable ones because a single meter cannot
  answer two questions — keep a tank on it and you cannot zero it for a leg.
- **The meter is chosen from a SELECTOR ROW, not a cycle** — `TRIP A`, `TRIP B`,
  `START` across the top of the hero, active one lit and underlined, one tap to
  select. It replaced a double tap that stepped A→B→START plus a faint
  `DBL SWAP; HOLD RESET` line. The cycle was wrong in two ways a hint could not
  fix: it was MODAL (reaching TRIP B from START meant two taps and knowing
  which way the ring ran) and INVISIBLE (the only way to learn the other two
  meters existed was to read the hint). Three names with one lit is the state
  of the page STATED rather than described, and it costs nothing — it took
  exactly the row the hint was using. The double tap was also guarding the
  wrong gesture: swapping which meter shows is harmless and undone by tapping
  the one you wanted. **Holding a NAME switches to it first, then asks to zero
  it** — otherwise holding `TRIP B` while looking at `TRIP A` would offer to
  clear TRIP A, the one mistake this page must not make. The tap target is the
  54×36 CELL, not the 41×16 glyphs.
- **Fuel price lives in SETTINGS**, default 2.01 €, adjusted with `-` / `+`
  rather than a cycle. Cost = litres × price, and it is the one readout whose
  accuracy is the driver's rather than the bus's.
- **ORIENTATION is a settings row**, sitting *below* INVERT COLOURS. Invert is
  reached squinting into low sun with the truck moving; orientation is set once,
  when the bracket is chosen. The row needed in a hurry stays first. It took one
  of the two placeholder rows — the other stays empty, because a page that
  invents options to look finished teaches the driver to distrust it.

## Open / unverified

- Gateway **2.1** (`AT@SM` duplex streaming) is built and flashed; the carputer
  probes it and falls back to ATMA windows against 2.0.
- `0x1C8 == 0x02` for "lights on" is a **one-sample guess**; the backlight log
  prints every input so one lights-off toggle settles it.
- Q/160 has never been checked against fuel actually paid for — a fill-to-fill
  would settle it.
- No link-failure screen; HEALTH lamp pills drawn but not driven; FAULTS and
  ALERT designed but not built (no vertical axis).
- **Which 90° is a guess.** `LV_DISPLAY_ROTATION_90` was picked over `_270`
  without glass. If landscape comes up upside down, that one token in
  `uiApplyOrientation()` is the whole fix and the touch follows it
  automatically — do not "correct" it in `Touch.cpp` or `Display.cpp`.
- **TRIP's hero crosses the column rule above 999.9 km.** `1247.8` is 188px of
  a 177px band. Portrait has the same ceiling one digit further out (it
  overflows the panel above 9999.9). Neither is fixable without a font the
  band is not allowed to add.
- `docs/ISP-WIRING.md` is referenced by the gateway's `platformio.ini` and does
  not exist.
- **A trip does not run while the ENGINE does not**, and rpm has THREE states,
  not two. Live and above `TRIP_RUNNING_RPM` accumulates; live and below it
  pauses the clock *and* carries the odometer anchor forward, so a car rolled or
  towed adds no distance either. **Stale rpm is the third**: pause the clock but
  leave the anchor alone. Treating stale as engine-off means a link dropout
  while driving silently deletes the kilometres covered during it — the anchor
  walks forward and the distance never appears. The host tests caught exactly
  that.
- **DTC reading is `core/Faults.h` plus a scan driver in main.cpp.** Everything
  underneath already existed: `kwpBuildReadDtc`, `kwpParseDtcList`,
  `kwpDtcToSae` (verified against all 1152 pairs) and 1152 descriptions in
  SignalDb. The scan walks ONE module per pass of carTask — twelve modules, half
  on CAN-C where they may never answer, so doing it in one go would cost five
  seconds of not listening. A module that **did not answer is not a module with
  no faults**: `answered` and `unreachable` are separate bitmasks, and HEALTH
  says `none (7 asked)` rather than a bare `none`.
- **The scan is AUTOMATIC: once when the link comes up, then every 5 minutes**
  (`FAULT_SCAN_PERIOD_MS`), on the owner's instruction — it used to be manual
  and nothing else. The automatic start is gated on `gLinkUp`, not on boot: with
  the link still down `faultScanStep()` marks every module unreachable *without
  transacting*, so a boot-time scan completes having asked nothing, and does it
  in the seconds before the answer becomes obtainable. Waiting for the link is
  what makes "once on link" and "every five minutes" one code path. A walk cut
  short by the link dropping sets the due stamp to *now* rather than +5 min, so
  it is re-owed the moment the link returns and not five minutes later. The
  first scan also re-asks the three modules `readDtcs()` just probed at
  bring-up — three redundant transactions, once per boot, and the probe stays
  because it is what proves the link end to end.
- **RE-READ still exists and only brings the next scan forward.** The control
  sits on its own result, which is the point — you read `3 stored`, you want to
  know if it still holds, and the thing you press is the thing you were already
  looking at. A single tap is enough because the read is harmless; the TRIP
  reset two pages over destroys something, which is what earns *it* a long press
  and a confirmation.
- **`faultScanStep()` runs on BOTH sides of the link-retry loop**, exactly like
  `tripHousekeeping()` and for exactly the reason its comment gives. Called only
  past the loop, a tap on FAULTS with the link down sits unconsumed until the
  link happens to return — it looks like a dead control. With no link it marks
  each module unreachable *without transacting*, so the walk costs nothing
  instead of twelve 800 ms timeouts inside the loop that is trying to reconnect.
- **FAULTS is the first DRILL-DOWN** — the vertical axis the canvas notes
  describe and nothing had ever used. It is NOT a peer on the swipe axis: a
  fault list is read deliberately, from a standstill, so it should cost a flick
  from nobody who is not going there. It covers the page area only, leaving the
  status bar and the dots showing, because the dots still say HEALTH — you are
  *inside* health, not somewhere else. Tap FAULTS to enter, BACK to leave,
  RE-READ to scan. The BACK / RE-READ bar is **1.5× its original height** (60px
  portrait, 39 landscape, on the owner's instruction): two words on a 16px line
  had hit targets only as tall as the bar, and nothing else on the page competes
  for the height — the list just scrolls one row less.
- **Muting is a HARD mute, and it is the driver's decision, not a finding.**
  Matched on `(ecu, code)` so it follows the fault rather than its list
  position; preserved across `faultsBeginScan()`; and persisted to NVS by
  `src/FaultStore` so it survives a key cycle. This car carries faults that are
  permanent and known, and without muting the count never changes — which means
  nobody reads it, and the one genuinely new fault is lost in the noise.
  `faultsActiveCount()` is what HEALTH shows.
- **A muted fault stays in the list.** It reads `not important`, dimmed, and
  UNMUTE in its dialog puts it back. Removing it would make the list a lie; the
  only thing muting changes is the COUNT.
- **`faultStoreLoadMutes()` must run AFTER `tripStoreInit()`**, which is what
  brings NVS up. Called earlier it returns an empty list, which looks exactly
  like a driver who never muted anything.
- **One tap handler for sixteen rows.** The FAULTS list works the row out from
  the touch y plus the scroll offset (`y / UI_FAULT_ROW_H`) rather than carrying
  sixteen invisible tap targets — the pool is fixed and the page already costs
  sixty-odd objects. LVGL only sends CLICKED when the press did not become a
  scroll, which is what makes tap-to-open safe on a list that also scrolls.
- **LVGL's click suppression only works if the list CAN scroll** — and that is
  what made the DTC list feel un-draggable. `lv_indev.c` drops `CLICKED` when a
  press became a scroll (`if(scroll_obj == NULL)`), but `lv_indev_scroll.c`
  will not even consider an object a scroll candidate unless
  `scroll_top > 0 || scroll_bottom > 0`. Five faults at 40px is 200px of
  portrait's 206, so there was nothing to scroll, so every drag arrived as a
  click and opened a dialog. The list now judges the gesture itself:
  `UI_FAULT_TAP_SLOP` (12px, just past `LV_INDEV_DEF_SCROLL_LIMIT`'s 10) against
  the press point recorded on `LV_EVENT_PRESSED`, plus a scroll-offset
  comparison for a press that lands while an inertial scroll is still settling.
  **`SCROLL_ELASTIC` / `SCROLL_MOMENTUM` are not the fix** — they are LVGL
  defaults, `stripChrome()` only strips `SCROLLABLE` and `CLICKABLE`, and with
  nothing off-screen there is no scroll to be elastic about. A short list
  answers the drag by doing nothing; the header count and the auto scrollbar
  are what say whether there is more below.
- **A list row is 40px and carries the DESCRIPTION.** `UI_FAULT_ROW_H` was 20 —
  one line, and half the 44px a fingertip needs, on the page read with a thumb.
  The second line is what the height is for: the description out of
  `dtcLookup()`, dotted at one line (220px ≈ 34 chars against a median 40), with
  a rule under each row so the pair reads as one target. The descriptions were
  ALREADY compiled in — 1152 of them in `core/SignalDb.cpp`, ~61KB of the image,
  binary-searched by `(ecu, code)` — and the scan log had been printing them all
  along; only the screen had never shown them.
- **Tapping a row OPENS it; it no longer mutes.** Muting from the list put the
  decision to ignore a fault behind the same gesture used to read one, on a
  target showing a code and a state and no reason. The dialog (`buildFaultDlg`)
  shows code, module, status and the full description over three wrapped lines,
  with CLOSE and MUTE/UNMUTE — the button's word is what it will DO, not what
  has been done. MUTE posts the same request the row used to, so the dialog
  stays open and flips to UNMUTE when carTask's change lands: the undo is the
  same button. Nothing here destroys anything, which is why neither gesture
  needs a confirmation — unlike the TRIP reset two pages over.
- **The dialog is a CHILD of the drill-down, scrim included.** BACK deletes the
  page and takes the dialog with it, so `faultDlgForget()` only drops pointers;
  `faultDlgShow(false)` is the path that deletes. The scrim is translucent
  (`LV_OPA_70`) so the list stays visible behind the card — *inside* the list,
  not somewhere else, the same argument as the dots staying visible — and it is
  CLICKABLE so it absorbs the taps the card does not. Without that flag
  `lv_indev_search_obj()` carries on to the siblings underneath, which is the
  exact bug that made the TRIP confirm dialog dismiss itself wherever it was
  touched.
- **The dialog's three controls are arranged PER BAND, like SETTINGS.**
  Landscape has the width and not the height — 218px of page cannot take two
  46px rows plus the description, but 280−24 of card divides into three 78px
  buttons on an 11px gap, and `UNMUTE` is 55px at `font_label_14`. Portrait has
  the height and not the width — three 88s want 288px of 192 — so CLOSE and
  MUTE keep their row and CLEAR goes underneath, full width, in `UI_CRIT`. That
  is the better arrangement anyway: the destructive control is not adjacent to
  the one that means "never mind".
- **CLEAR erases ONE code, and it is the only destructive control here.**
  `kwpBuildClearDtc()` sends `14 <hi> <lo>` — clearDiagnosticInformation with
  the group field carrying the specific code instead of the all-groups
  `FF 00`. **Whether these modules honour the single-code form is UNVERIFIED**;
  a refusal is logged and the row stays, which is what the button's `FAILED`
  state means. The row leaves our list only on a POSITIVE reply
  (`faultsRemove()`), never optimistically — showing a cleared fault that the
  car still holds, and that reappears at the next read, is the
  frozen-but-plausible failure applied to the one screen whose whole job is
  saying what is wrong.
- **CLEAR arms on the first tap and acts on the second** (`UI_CLEAR_ARM_MS`,
  4 s), and the button says which of its four states it is in: `CLEAR`,
  `CLEAR?`, `CLEARING`, `FAILED`. No second dialog — a dialog on top of a
  dialog is worse than the problem it solves — and the arming lapses on its
  own, so a dialog left open cannot be finished by a bump on a rough road.
  `FAILED` after `UI_CLEAR_WAIT_MS` is not a guess: the request went to
  carTask, carTask transacted, and the row is still here. A successful clear
  takes the row with it and the dialog closes itself.
- **The clear request travels as `(ecu, code)`, not as a list index** — unlike
  the mute. A scan finishing between the tap and carTask consuming it re-sorts
  the list, and an index would then name a different fault. For a reversible
  mute that is a wrong row; for an erase it is the one mistake that cannot be
  undone. `requestFaultClear()` writes the code FIRST and the ecu LAST, so the
  pair is complete whenever the reader sees a valid ecu.
- **The dialog tracks its fault by `(ecu, code)` AND index.** A RE-READ while it
  is open renumbers the list, and an index alone would quietly start describing
  a different fault — the one mistake this page must not make. It closes
  instead.
- **The list is SORTED by the code as DRAWN, in `faultsEndScan()`.** Not by the
  raw `uint16_t`: `kwpDtcToSae` takes its letter from `"PCBU"` (bits 15..14), so
  raw ascending puts every P before every C before every B and reads as no order
  at all. Sorting the rendered five characters gives B, C, P, U. Stable, so two
  modules reporting the same code keep scan order between them. Sorted at
  END-of-scan only — per record would shuffle rows under a finger mid-walk, and
  the open dialog holds an index.
- **`faultPoolSample()` logs the pool when the drill-down opens.** `faults page:
  lvgl heap …` — sampled under the lock, printed by `uiTask` outside it, exactly
  like the frame-rate line, because `ESP_LOGx` under `lvgl_port_lock` blocks on
  the USB-Serial-JTAG host and stalls the renderer. This is the page that grew;
  it is also the one whose allocation failure is a bare `while(1)`. **Look at
  that line after touching this page.**
- **There is ALSO a pre-existing `readDtcs()`** run at link bring-up for RADIO,
  CCN and WCM. It only logs — it does not feed the fault store — and it is kept
  because a module answering (even with zero faults) is what proves ISO-TP, KWP
  and the adapter end to end. Two paths now build the same `18 00 FF 00`; change
  one and look at the other.
- **`0x2C0` byte 2 is the ACCELERATOR PEDAL.** Dual-track APPS relayed onto
  CAN-B at 100 ms: byte 3 is the half-scale second track, byte 6 a byte-identical
  copy of byte 2, bytes 0/1/5 always `0xFF`. Both ends are anchored, not
  guessed — `0x17` released is proven by a hardware idle switch (bit 2 or 3 of
  byte 7 set ⟹ byte 2 == `0x17`, in 3767 of 3767 capture frames), `0xEA` is the
  floor.
- **The KICKDOWN DETENT is visible in the data.** Between `0xC0` and `0xE9`
  there is not one frame in 8089: the pedal sweeps to the hump (single frames at
  `0xB6`, `0xBF` passing through) and past it is only ever observed on the floor
  — 54 consecutive frames at `0xEA` through a 1509→4158 rpm pull. Nobody rests
  in a detent. That makes `>= 0xDF` a **binary, driver-intentional** event that
  needs no calibration, which is worth more than any percentage — and it is why
  the pedal, not a time or an rpm threshold, is what now gates a run.
- **Reject the engine-off sentinel on 0x2C0.** With the engine stopped the frame
  is all `0xFF` but a trailing `0x00`, and byte 2 then reads as a pedal pressed
  harder than the floor. Gate on bit 7 of byte 7 (engine running) and on byte 6
  agreeing with byte 2.
- **0-60 and 0-100 come off the speed broadcast alone** (`core/Accel.h`), and
  the limits are all consequences of that being the only clock and ruler there
  is. A run ARMS only by having genuinely seen the car below 0.5 km/h — joining
  a motorway at 90 must never produce a figure. It times from the LAST
  STATIONARY sample, not the first moving one, so the bias can only ever be
  slow. Splits are INTERPOLATED across the sample that crosses the line: 0x002
  lands every ~50 ms and a hard launch gains several km/h in that gap, which is
  a bias that flatters exactly the runs people care about. Over 15 s is not a
  run and is not recorded, though a 60 reached inside it still counts.
- **A run must be LAUNCHED, not merely started from rest** —
  `ACCEL_LAUNCH_MS` (1500 ms), and this was a REPORTED DEFECT: pulling gently
  out of a junction started the stopwatch and DRIVE's sub-line flipped away
  from the journey. Rest-then-motion was the entire trigger; the kickdown flag
  was decoration on the result (`floored`, shown dimmer). Now the pedal has to
  reach the detent within 1500 ms of the car moving or the run is **thrown
  away** — `dropUnlaunched()`, not `finish()`, so nothing is published however
  quick it was. Measured on `scripts/playback.log`, one 841 s drive: **14 runs
  started under the old trigger, 0 under the new, and 0 were ever published
  either way** — fourteen false stopwatches, no real figure lost. 1500 ms
  because 0x2C0 is a 100 ms frame and a foot is not simultaneous with the first
  millimetre; long enough for the real order of events, far too short for "pull
  away, then decide to overtake".
- **`accelLaunched()` is what the DISPLAY follows, `accelRunning()` is not.**
  Timing still starts at the last stationary sample — that is what keeps the
  figure honest — so `accelRunning()` is true for up to 1500 ms on an ordinary
  pull-away, before the run is dropped. Draw from that and the sub-line flips
  and flips back, which is the defect as seen from the driver's seat rather
  than in the data.
- **The kickdown flag is fed only while the pedal is FRESH** —
  `BC_PEDAL_TTL_MS` (500 ms) in `main.cpp`, where `accelPedal()` is called.
  `pedalRaw` is HELD when 0x2C0 stops, like every value here, and the flag is
  now a GATE rather than a decoration: a held `0xEA` would confirm the launch
  of every pull-away for as long as the signal stayed away. Frozen, plausible
  and wrong, on the feature that had just been fixed.
- **`scripts/playback.log` replays through core on the HOST**, and the harness
  is now checked in as **`scripts/replay-accel.cpp`** (build line in its header)
  rather than being rewritten from scratch each time. 126183 frames, 841 s of a
  real drive: `core/Broadcast.cpp` + `core/Accel.cpp` against a small main that
  parses LAWICEL and calls them in `onBroadcastFrame`'s order, synthesising time
  from the measured 151 frames/s — no board, no wiring. It is what measured the
  launch gate (14 starts → 0), and its printed peaks double as a check that the
  replay is faithful. It found: peak 82.9 km/h / 4163 rpm, pedal max
  `0xEA`, 7624 of 8089 pedal frames accepted, **zero acceleration runs**. That
  last one is the point: 15 standing starts, only 3 ever reach 60, the quickest
  in 19.2 s. Ordinary driving records nothing, which is what the cut-offs are
  for. It also proved the SPEED sentinel matters — four frames late in the log
  read `0xFFFF` = 512.0 km/h, and each would have produced a fake 0.1 s 0-60
  had Broadcast not rejected them.
- **Accel is fed from `onBroadcastFrame`, not carTask** — the one derived value
  that is. carTask samples `gBcast` around a loop containing a 700 ms monitor
  window, and a launch timed off that would start its clock up to 700 ms early:
  a fifth of a brisk 0-60, all of it flattering. Safe because it keeps ONE
  writer; the trips cannot move there because carTask also resets and commits
  them.
- **DRIVE's hero sub-line is CONTEXTUAL** — the only readout on the instrument
  that changes meaning. It shows the journey's distance and cost, and switches
  to the acceleration stopwatch while a LAUNCHED run is timing plus 20 s after.
  It is allowed to because the trigger is the driver flooring it from a
  standstill — literally that, since `ACCEL_LAUNCH_MS`, rather than nearly that
  — and because an acceleration figure is interesting for ten seconds a year
  while a permanent cell would have cost something useful on every drive.
- **The hero carries a SUB-LINE.** `font_digits_64` is 73px in a column that
  has 172, so a hero alone left ~87px of air. TRIP puts `IN` (the whole window)
  and `RIDE` (the moving part) there; DRIVE puts this drive's distance and cost.
  That is what freed the grid cell COST occupies — nothing was dropped for it.
- **`font_label_20` exists for the hero UNIT KEYS** (`KM`, `L/100`, `V`, `BAR`),
  right-aligned at the top of the page rather than trailing the numeral. Forced,
  not stylistic: at 20px an inline `L/100` starts where DRIVE's hero ends. It
  also hands every hero its full column width, which TRIP needs — `247.8` alone
  is 164px of 165. Regenerating `font_label_14` for the € did **not** move a
  single existing advance width (same `--size 16`, same 16/3 metrics).
- **Numeric precision is a layout constraint, not a taste.** The stack cell is
  106px (portrait's pair 110), and the unit eats the right end of it. COST is
  **1 decimal** (clean to 999.9; two decimals ran off the panel past €1000) and
  DRIVE's TRIP A/B distances are **whole km** (clean to 9999; `km` is a 19px
  unit and 999.9 only just fitted). TRIP's own hero keeps its decimal.
- **The status bar shows clock, dots and link. No ignition word.** It was on
  every artboard and the widget existed unwritten for months; driving it was a
  mistake and it is gone from both. A driver looking at a running instrument in
  a running truck already knows the ignition is on, and a 22px bar has no room
  for a field that is constant whenever anyone can read it. `bcast.ignition` is
  still decoded and still useful for logic; it is just not a readout.
- **A low tyre corner is coloured, but obeys hold-and-dim.** `UI_CRIT` when
  fresh, `UI_CRIT_DIM` when stale — a held reading must never be the loudest
  thing on the panel. It runs *after* `setValScaled` in `refresh()` and only
  ever paints low corners, so a recovered corner is restored by that call;
  swap the order and it stays red for good.
- **`font_digits_*` has no letters** (`0x20`, `0x2D-0x2E`, `0x30-0x3A`, `0xB0`).
  The FAULTS codes (`P213A`) are drawn in the condensed face on the artboards
  and cannot render in it. The SETTINGS info values (`STREAM`, `v2.1`) hit the
  same thing and use `font_label_14` in both bands, which is why they look
  lighter than the artboard.
- **The LVGL flow-bar cache had to be reset on rebuild.** `s_flowLit` survives
  a tree teardown and `setFlow()` only repaints on a change, so a palette swap
  (pre-existing) or a flip left a freshly-dark bar agreeing with a stale count
  and never lighting again. `buildRoot()` now clears it. Any future
  "only-update-on-change" cache that lives outside a widget needs the same.
