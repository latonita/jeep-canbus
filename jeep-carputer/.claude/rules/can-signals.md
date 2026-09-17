---
paths:
  - "core/Broadcast.*"
  - "core/Trip.*"
  - "core/Accel.*"
  - "core/Compass.*"
  - "core/VehicleState.*"
  - "core/SignalDb.*"
  - "core/TpmsSignals.*"
  - "core/Decoder.*"
  - "core/PollDecode.*"
  - "tools/**/*"
  - "scripts/replay-accel.cpp"
---

# What the signals mean, and what is actually measured

Numbers here were measured against captures in `../../jeep-can-b-logs/` and
`scripts/playback.log`. Do not "improve" one without re-measuring — a stale
comment on a verified number costs somebody an afternoon.

## Broadcast ids

- **`0x1A7` is already a RATE.** `Q = b0<<8|b1`, `L/h = Q/160`. Integrate
  time-weighted per frame; never multiply by rpm, never sum samples. `0xFFFF` =
  unavailable (drop the interval), `0` = valid overrun cut-off.
- **Distance is the `0x014` odometer differenced**, 0.1 km/bit — not integrated
  speed. Its quantisation is 0.94 % of the fuel error budget and dominates every
  other term by thirty times.
- **`0x159 b5` is 0.5 LITRES per count, not a percentage**, and `b4` is its
  validity flag. Three independent things say litres: the largest valid raw in
  any log is 154 = 77.0 L against this truck's 77.6 L tank; the owner reports
  ~90 % where `/2.55` insisted on 54 %; and over the 2026 drive the tank fell
  2 counts = 1.0 L while the 0x1A7 integral accumulated 1.1065 L. `b4==0x01` ⇔
  `b5==0xFF` ⇔ no reading, with no exceptions in any log — the usual all-ones
  test cannot work here because `b0=FE, b1:b3=FF FF FF` is the NORMAL payload.
  `b5==0xFF` is deliberately NOT also rejected: b4 is the flag the module sets,
  and blanking the gauge the one moment a tank could legitimately read high is
  the worse lie. The frame keeps transmitting through ignition-off. **There is
  no CCN fuel poll** — it was a slower copy of this, and deleting it also stopped
  the bench reading back `emulate-car.py`'s own hard-coded constant.
- **`0x2C0` byte 2 is the ACCELERATOR PEDAL.** Dual-track APPS at 100 ms: byte 3
  is the half-scale second track, byte 6 a byte-identical copy of byte 2, bytes
  0/1/5 always `0xFF`. Both ends are anchored: `0x17` released is proven by the
  hardware idle switch (bit 2 or 3 of byte 7 set ⟹ byte 2 == `0x17`, 3767 of
  3767 frames), `0xEA` is the floor, held flat through a 1509 → 4158 rpm pull.
  1631 accepted frames sit above `0x37` across 69 distinct values.
- **The KICKDOWN DETENT is visible in the data**: not one frame in 8089 between
  `0xC0` and `0xE9`. Nobody rests in a detent, so `>= 0xDF` is a **binary,
  driver-intentional** event needing no calibration — which is why the pedal,
  not a time or an rpm threshold, gates an acceleration run.
- **Reject the engine-off sentinel on 0x2C0.** Engine stopped, the frame is all
  `0xFF` but a trailing `0x00`, and byte 2 then reads as pressed harder than the
  floor. Gate on bit 7 of byte 7 (engine running) and on byte 6 agreeing with
  byte 2.
- **`0x1B4` heading has no unavailable sentinel and does NOT stop at key-off.**
  0xFF never appears for b1 in any of ten logs, and the frame keeps sending a
  valid unchanged heading straight through the 0x000 ignition-off transition and
  for ~1000 lines after. So nothing may gate key-off on it, and `headingMs` can
  only be zero before the first frame.
- **The octant enum bins by ROUND, not floor.** `round(b1*2/45) mod 8` agrees
  with the module's own byte 0 in 99.46 % of 2953 frames (99.75 % on an
  independent 2019 session); floor agrees 41.65 %. All 16 residual disagreements
  put the module one octant AHEAD and never behind — evidence that b1 is
  `floor(trueDeg/2)`, and the reason NOT to "fix" the residual with a +0.5 nudge,
  which would break the other 2937.
- **Speed is rejected only at EXACTLY `0xFFFF`.** `0xFFFE` is 511.98 km/h and is
  accepted as a reading. `playback.log` contains four real `0xFFFF` speed frames,
  each of which would have produced a fake 0.1 s 0-60.
- **`0x210` byte 1 reads `0x00` for BOTH** the bottom of the dimmer wheel and
  all-lights-off, which want opposite brightness. `0x1C8` separates them and is
  wired to only ever *brighten*, so an absent signal costs nothing.

## Measured loss and timing

Overall frame loss is **0.22 % across 43 clean periodic ids**. `0x002` loses
nothing (28,991 frames, median 50 ms, max 56 ms, no gap above 75 ms); `0x1A7`
median 100 ms, p99 108 ms, worst 206 ms; `0x2C0` loses 2.1 % of frames plus
1.85 % failing the validity gate — 3.9 % absent overall. With adapter stamps a
150 ms gap detector on any 100 ms id separates real loss perfectly (largest
legitimate interval 119 ms); on host receive times the same detector produces
17–80× more false alarms than real losses.

## Trip

- **A trip does not run while the ENGINE does not, and rpm has THREE states.**
  Live and above `TRIP_RUNNING_RPM` accumulates. Live and below pauses the clock
  *and* carries the odometer anchor forward, so a car rolled or towed adds no
  distance. **Stale rpm is the third**: pause the clock but leave the anchor
  alone — treating stale as engine-off means a link dropout while driving
  silently deletes the kilometres covered during it.
- **`tripRangeKm(t, litresInTank)`** projects the SELECTED meter's average over
  what 0x159 says is in the tank, so switching tabs changes it. Same
  negative-means-unavailable convention as `tripL100()`, `tripCost()`,
  `tripAvgKph()`; caller passes -1 for an absent tank. **`TRIP_RANGE_MIN_KM` is
  1.0, ten times `tripL100()`'s own floor**, because this multiplies an average
  across ~700 km: a meter reset at the bottom of a hill reads a few hundred
  millilitres per 100 km and promises several thousand kilometres — exact, and a
  lie. No upper clamp: capping would make a wrong average look reasonable. Zero
  litres is a REAL reading and returns 0 km, not unavailable.
- **`TRIP_MAX_GAP_MS` is 2000 and should be 300 on the LAWICEL path** — see
  `docs/open-questions.md`; it is not a number to simply edit.

## Accel (`core/Accel.h`)

0-60 and 0-100 come off the speed broadcast alone, and every limit is a
consequence of that being the only clock and ruler:

- A run ARMS only by having genuinely seen the car below 0.5 km/h — joining a
  motorway at 90 must never produce a figure.
- It times from the LAST STATIONARY sample, so the bias can only ever be slow.
- Splits are INTERPOLATED across the sample that crosses the line: 0x002 lands
  every ~50 ms and a hard launch gains several km/h in that gap, a bias that
  would flatter exactly the runs people care about.
- Over 15 s is not a run and is not recorded, though a 60 reached inside it
  still counts.
- **A run must be LAUNCHED**: the pedal has to reach the detent within
  `ACCEL_LAUNCH_MS` (1500 ms) of the car moving, or the run is thrown away by
  `dropUnlaunched()` rather than published. Rest-then-motion alone started
  the
  stopwatch when pulling gently out of a junction, which flipped DRIVE's
  sub-line away from the journey.
- **The gate is ALSO in `finish()`, and the temporal test alone was not
  enough.** `dropUnlaunched()` can only discard a run still running at 1500 ms; a
  run that *completed* inside the window was published with `floored` false and
  — because `best` only ever moves faster and `accelInit()` runs once a boot —
  pinned the BEST row for the session. Two samples do it: `(0 km/h, 600 rpm,
  t=1000)` then `(110 km/h, 600 rpm, t=1100)` publishes a 55 ms 0-60. `finish()`
  is the only writer of `hist` and `best`, so the check belongs there and covers
  all four callers.
- **The kickdown flag is fed only while the pedal is FRESH** — `BC_PEDAL_TTL_MS`
  (500 ms) in `main.cpp`, where `accelPedal()` is called. `pedalRaw` is HELD
  when 0x2C0 stops, and the flag is a GATE rather than a decoration: a held
  `0xEA` would confirm the launch of every pull-away for as long as the signal stayed away.
- **`accelLaunched()` is what the DISPLAY follows, not `accelRunning()`** —
  timing still starts at the last stationary sample, so `accelRunning()` is true
  for up to 1500 ms on an ordinary pull-away and the sub-line would flip and
  flip back.

## Replaying captures on the host

`scripts/replay-accel.cpp` (build line in its header) runs `core/Broadcast.cpp`
+ `core/Accel.cpp` over `scripts/playback.log` — 126,183 frames, 841 s of real
driving, no board. It measured the launch gate (14 starts → 0 under the new
rule, and 0 were ever published either way: 15 standing starts, only 3 reach 60,
quickest 19.2 s). Peaks 82.9 km/h / 4163 rpm, pedal max `0xEA`, 7624 of 8089
pedal frames accepted.

**Its time base is wrong and the header says so**: it synthesises time from
151 frames/s while the log carries 4-digit rolling stamps of its own.
`core/StreamClock` is the module that unwraps them. Nothing in the counts above
is timing-sensitive at that resolution, but a split measured off this harness is
measured off a metronome.

SAE codes round-trip exactly over all 65536 values (`kwpDtcToSae`, verified
against all 1152 pairs), so anything typed as `P213A` or `B2102` reaches the
dashboard as itself.
