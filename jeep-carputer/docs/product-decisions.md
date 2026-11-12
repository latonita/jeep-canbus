# Product decisions, settled — do not relitigate

Read this before changing what the instrument shows or how it behaves. Items
marked "owner's instruction" are the owner's preference about their own
dashboard and that settles them.

## What this thing is

- Driver-glanceable, **not** a parked diagnostic tool.
- **TPMS is the hero feature.** Tyre pressure in **bar**. Corner order is still
  UNMAPPED and naming the corners would be a guess worse than no name.
- The radio is invisible; the AT surface cannot touch VES/AUX or Bluetooth.
- Read the maximum from broadcasts; poll only what does not broadcast.
- Hold-and-dim missing data; show dashes only for explicit unavailable
  sentinels. **A frozen-but-plausible number is the failure mode that gets
  believed.**
- Brightness from the CAN dash dimmer, **fail bright**.
- **There is deliberately NO idle timeout.** A page changes only when a finger
  changes it.

## The eight screens

**DRIVE, TRIP, COMPASS, HEALTH, TYRES, ACCEL, HISTORY, SETTINGS** — eight. This
heading has said five, then six; COMPASS and HISTORY are what made it eight, and
the order is the one in `UI_SCREEN_COUNT` (`Ui.h`), which is the only place it
is written down as code.

`UI_ACCEL` and `UI_HISTORY` both sit between the deliberate-reading cards and
SETTINGS.

DRIVE is index 0 and where the boot settles (`UI_HOME`). It used to sit second
so it was reachable from either direction in one swipe; that was given up
deliberately, because the card wanted most often should cost NO flicks and a
hard end cannot be overshot. SETTINGS stays last and costs five flicks, which is
the point.

**HEALTH comes BEFORE TYRES, on the owner's instruction.** They were the other
way round and no reason was given, so none is invented here. Written down because
it cuts against something else: TPMS is the hero feature, and this puts it one
flick further from DRIVE than the page that is mostly a voltage and two
temperatures. If the order ever looks wrong to somebody reading the rationale
instead of driving the truck, this is the answer.

Reordering is ONE edit — `s_page[]` is indexed by the enum symbol everywhere
(`buildTyresLs(s_page[UI_TYRES])`) and the dot rail is a plain loop over
`UI_SCREEN_COUNT`. Nothing assumes a literal index except one log line about
`s_page[0]`, which is DRIVE and does not move.

## Orientation

**THREE orientations, not two.** `Settings.h`: 0 portrait, 1 landscape, 2
landscape turned end for end. The flip between the two landscapes needs no
rebuild — only a different panel mirror — which is why it is easy to forget the
third exists.

**ORIENTATION is a settings row**, sitting *below* INVERT COLOURS: invert is
reached squinting into low sun with the truck moving, orientation is set once
when the bracket is chosen, and the row needed in a hurry stays first. It took
one of the two placeholder rows — the other stays empty, because a page that
invents options to look finished teaches the driver to distrust it.

Landscape is **a settings row, not a second product**: same five peers, same
order, same one-flick-one-page rule, same absence of an idle timeout.

## Tyres

**Polled ONCE A MINUTE** — corners and spare, on the owner's instruction; it was
5 s and 30 s. A tyre loses pressure over days and the sensors themselves only
transmit every ~60 s parked, so a 5 s request was mostly re-reading the WCM's
held copy: four of the five requests in `kPlan` were buying nothing.

`UI_TYRE_TTL_MS` / `UI_SPARE_TTL_MS` are twice the poll interval and must move
with it, or the page dims a reading as fresh as it will ever be. They live in
`core/Snapshot.h`. Note `PollItem::periodMs` is a `uint16_t` — 65535 ms is the
longest period `kPlan` can express (120000 there is a `-Wnarrowing` build error),
which is why the spare shares the corners' rate rather than staying slower.

## Trips

**Three meters**: TRIP A and TRIP B both persist (two NVS keys), FROM START
clears on every boot. Two resettable ones because a single meter cannot answer
two questions — keep a tank on it and you cannot zero it for a leg.

**The meter is chosen from a SELECTOR ROW, not a cycle** — `TRIP A`, `TRIP B`,
`START` across the top of the hero, active one lit and underlined, one tap to
select. It replaced a double tap that stepped A→B→START plus a faint
`DBL SWAP; HOLD RESET` hint. The cycle was wrong in two ways a hint could not
fix: MODAL (reaching TRIP B from START meant two taps and knowing which way the
ring ran) and INVISIBLE (the only way to learn the other two meters existed was
to read the hint). Three names with one lit is the state of the page STATED
rather than described, and it cost nothing — it took the row the hint was using.

**Holding a NAME switches to it first, then asks to zero it** — otherwise holding
`TRIP B` while looking at `TRIP A` would offer to clear TRIP A, the one mistake
this page must not make. The tap target is the 54×36 CELL, not the 41×16 glyphs.

**Fuel price lives in SETTINGS**, default 2.01 €, adjusted with `-` / `+` rather
than a cycle. Cost = litres × price, and it is the one readout whose accuracy is
the driver's rather than the bus's.

## History

**FOUR PARAMETERS, ONE PLOTTED: FUEL FLOW, SPEED, COOLANT, BATTERY.** Chosen for
having a TREND that says something the number cannot — coolant creeping up a
climb, volts sagging across a drive, what the last quarter of an hour actually
drank. Speed is there because it is what explains the shape of the other three.
Ambient and tank level were both considered and dropped: neither moves enough in
fifteen minutes to draw, and a chart that is always a flat line teaches the
driver the page is broken.

**FIFTEEN MINUTES, AS 45 BUCKETS OF 20 SECONDS.** The point count is set by
PIXELS, not by time: the chart has to be 179 px wide because that is what the
landscape left column affords, and at a 4 px pitch that is 45 columns. The
window is then just 45 × the bucket length, and 20 s was chosen over 10 s
(7.5 min, too short for a commute) and 40 s (30 min, more than one needs to
answer "what just happened"). Storage is 720 bytes either way.

**THE CHART IS THE SAME WIDTH IN BOTH BANDS.** Same argument as the compass
tape, applied to time rather than heading: a chart that got wider when the
bracket turned would change how much history a glance buys. Landscape is the
narrow band, so it sets the width and portrait uses the result unchanged —
carrying ~51 px of margin to its right, deliberately, with a full-width floor
rule under the chart so it reads as placed rather than cut off.

**EACH COLUMN IS A MIN-MAX RANGE, NOT A MEAN.** A mean hides the spike, and on
speed and flow the spike is the information: a junction reads as a bar spanning
0 to 60 rather than a flat 30. It also makes a coarse bucket cheap — what 20 s
costs is *when inside those 20 s* something happened, not *whether* it did —
which is what let the window be 15 minutes instead of 7.5 at the same width.

**FIXED BANDS, PRINTED, NEVER AUTO-RANGED** (`kHistBand[]` in `core/History.h`):
flow 0-50 L/h, speed 0-160 km/h, coolant 40-120 °C, volts 11-15 V. Same rule as
the flow bar — a chart whose full scale moves means something different every
time it is looked at. Coolant's floor is **40 and not 0** knowingly: the
interesting band is 70-105 and starting at zero squeezes it into a third of the
height, at the price of a cold start clipping to the floor for the first
minutes.

**ONLY FRESH READINGS ARE RECORDED, AND A GAP DRAWS NOTHING.** A held reading
folded into a bucket would draw a flat line at the last known value for as long
as the dropout lasted — hold-and-dim's failure mode, and worse on a chart than
on a readout because there is no dimming to carry the doubt. An empty bucket
draws no column at all, not a floor tick: zero km/h and zero L/h are both
ordinary readings, so a mark on the floor could not be told from one.

**IT CHARTS THE READOUT, NOT THE BUS.** The ring is fed from the resolved
`Snapshot`, last thing in `snapshotBuild()`, so it inherits broadcast-beats-
polled, the TTLs and the freshness rule without restating any of them. The
history rides on the snapshot rather than through a second reader with a second
lock, because five live pointers is exactly what that structure was built to
replace.

**FLOW IS CHARTED IN L/h, NOT IN THE SWITCHING UNIT.** `Snapshot::flow` changes
between L/h and L/100km on a speed hysteresis, which is right for a hero readout
that says what it is showing and useless for a fifteen-minute trace — half the
window would be one quantity and half the other, against one scale, with no
visible seam. `Snapshot::flowLph` is the same float taken one step earlier.

**The history survives a palette swap and an orientation flip, and not a
reboot.** It lives in `core/`, outside LVGL, so a tree teardown cannot touch it;
it is never written to NVS, because a fifteen-minute window across an ignition
cycle describes nothing.
