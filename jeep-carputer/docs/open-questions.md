# Open / unverified

Things that are not settled. Everything here is either unmeasured, unverified on
the car, or a decision waiting on the owner.

## Needs a drive or a receipt

- **ONE TANK RUN SETTLES TWO NUMBERS.** Brim to brim, logging raw `sum(Q*dt)` in
  count-seconds alongside raw `0x159 b5`:
  - **`0x1A7`'s divisor** — litres actually added falls straight out of the
    count-second total — see
    `../../jeep-can-b-logs/fuel-analysis/1A7-fuel-rate.md`. 160 is anchored at
    idle (~0.76 L/h), rated power (40.0 vs 40.3 L/h), urban consumption (12-13
    L/100 km moving) and coarsely by the 0x159 cross-check, but never against
    fuel paid for.
  - **`0x159`'s 0.5 L/count** — the difference between two brim readings should
    equal the litres the pump says, exactly, with no capacity assumption. Litres
    are what a pump prints, so this is the cheapest number here to settle.
- **`0x1C8 == 0x02` for "lights on" is a one-sample guess.** The backlight log
  prints every input, so one lights-off toggle settles it.

## Waiting on hardware or a decision

- **Neither link fallback rung has been taken on real hardware** since the ladder
  grew its middle rung — the board in the radio always accepts `AT@SML2`. Only
  that rung has been exercised on the emulator.
- **`TRIP_MAX_GAP_MS` should be 300 ms on the LAWICEL path and is 2000.**
  Measured over 14,478 `0x1A7` frames on the gateway's clock: median 100 ms, p99
  108, worst 206, none above 300. All of 2000 / 1000 / 500 / 300 integrate the
  identical 1.1055 L; 150 ms starts discarding 0.18 %. So 300 is the honest
  threshold — and it is not what the constant says, because the constant is
  compile-time in pure code that cannot know which clock the stamps arrived on,
  and the `AT@SM1` and ATMA rungs really do need seconds of slack (a 200 ms host
  stall at a 300 ms threshold discards up to 9.5 % of the fuel). **Tightening it
  means making it FOLLOW THE LINK MODE**, a deliberate change to a
  persisted-totals path. Cost of leaving it is bounded: on the LAWICEL path 2000
  drops nothing 300 would keep, it merely fails to notice a dropout between
  300 ms and 2 s.
- **The response-pending loop is not implemented**, and it is why CLEAR fails on
  the car (see `.claude/rules/faults-and-dtc.md` for the full diff against
  jeepdiag). `IsoTpClient::transact()` completes on the first whole PDU, so
  `7F xx 78` is taken as the answer. Any module that defers will never be read —
  this silently costs every polled signal on such a module, not just the clear.
  `emulate-car.py`'s `pending once` reproduces it.
- **Open question for the owner**: once pending is fixed, chase a single-code
  clear, or accept `14 FF 00`, make the button mean "clear this module", and let
  MUTING carry the permanent faults? `14 FF 00` wipes a module's whole fault
  memory, which is exactly what the single-code form was chosen to avoid, and the
  single-code form has never been observed working anywhere.
- **Which 90° is a guess.** `LV_DISPLAY_ROTATION_90` was picked over `_270`
  without glass. If landscape comes up upside down, that one token in
  `uiApplyOrientation()` is the whole fix and the touch follows it automatically
  — do not "correct" it in `Touch.cpp` or `Display.cpp`.

## Known gaps

- No link-failure screen.
- HEALTH lamp pills are drawn but not driven.
- FAULTS landscape and ALERT are designed but not built.
- **TRIP's hero crosses the column rule above 999.9 km** (`1247.8` is 188px of a
  177px band). Portrait has the same ceiling one digit further out (it overflows
  the panel above 9999.9). Neither is fixable without a font the band is not
  allowed to add.
- `docs/ISP-WIRING.md` is referenced by the gateway's `platformio.ini` and does
  not exist.
- **`scripts/replay-accel.cpp` synthesises time at 151 f/s** while
  `playback.log` carries its own 4-digit stamps that `core/StreamClock` could
  unwrap. The header says so; the code still does not. Nothing in the counts it
  produced is timing-sensitive at that resolution, but a split measured off it is
  measured off a metronome.
