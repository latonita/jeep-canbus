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

## The six screens

**DRIVE, TRIP, HEALTH, TYRES, ACCEL, SETTINGS** — six, not five. `UI_ACCEL` sits
between the deliberate-reading cards and SETTINGS (`Ui.h`).

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
