---
paths:
  - "src/Ui.*"
  - "src/fonts/**/*"
---

# Layout and typography, both bands

Portrait 240×320 and landscape 320×240 are ONE product toggled at runtime, not
two designs to keep in sync. Same six peers, same order, same one-flick-one-page
rule. Nothing is dropped, added or reordered between them — that is what makes
the toggle safe.

**Measure before you place anything.** Widths come from `adv_w` in
`src/fonts/*.c` (1/16 px units, plus `letter_space` between glyphs but not after
the last). There is no checker script for this. The `.dc.html` artboards are
~30 % narrower than the firmware (CSS Barlow figures are ~0.727 em, so
`font-size:66px` renders 48px digits while `font_digits_64` renders 64) — never
take a coordinate off an artboard without re-measuring the string.

## Fonts

- **`font_digits_*` has no letters** (`0x20`, `0x2D-0x2E`, `0x30-0x3A`, `0xB0`).
  DTC codes (`P213A`) and the SETTINGS info values (`STREAM`, `v2.1`) therefore
  use `font_label_14` in both bands, which is why they look lighter than the
  artboard.
- **`font_label_20` is NO LONGER A UNIT FACE** but is still compiled in and
  drawn, in two places that are not units: the `-`/`+` glyphs on the SETTINGS
  fuel-price buttons, and the code line in the fault dialog (`UI_FONT_UNIT`,
  now a misleading name). **Do not delete `src/fonts/font_label_20.c`**, and
  note that `src/fonts/fonts.h` still describes it as the hero unit key face —
  that is history, not the rule. (Fonts are generated; the regeneration command
  lines are in that header.) Regenerating `font_label_14` for the € did not move
  a single existing advance width.
- **`mkUnitKey` and `UI_UNIT_KEY_DY` are GONE**, zero call sites.
- `font_digits_64` is line_height 73 / base_line 1, so its baseline is 72px
  down; `font_label_14` is 16/3, so its own is 13. `font_digits_30` has
  **`base_line = 0`** — alone among these faces its baseline IS the bottom of
  its line box, with no descender space to absorb anything.

## The one typographic rule

**The hero's unit sits on the hero's baseline, right-aligned to the column edge,
in `font_label_14`, in both bands.** No header unit key exists anywhere on the
instrument. `UI_HERO_UNIT_Y(heroY) = heroY + 72 - 13`.

Portrait heroes start at x=8 and units right-align to `UI_W - (UI_PAD - 2)` =
230:

| portrait hero | unit w | unit left | hero worst case | hero right | clear |
|---|---|---|---|---|---|
| DRIVE `L/100` | 35.3 | 194.7 | `99.9` = 127.1 | 135.1 | **59.6** |
| DRIVE if `100.0` ever rendered | 35.3 | 194.7 | 159.1 | 167.1 | **27.6** |
| HEALTH `V` | 8.4 | 221.6 | absurd `888.8` = 168.8 | 176.8 | **44.8** |

`refreshDrive()`'s **decimal rule** (drop the tenth at three integer digits)
is what makes this work in LANDSCAPE, where `L/100` right-aligns to the column
edge at x=162 and `font_digits_64` renders `100.0` in 159px against a hero
starting at x=8 — 45px of clearance once the tenth goes. Portrait clears by 27.6
even with the tenth, but the rule is shared and STAYS: a hero that renders
differently depending on which way the bracket faces is worse than a tenth
nobody can use. DRIVE really does reach three digits — a 0.1 km cold start reads
150.

**ACCEL has NO unit at all, in either band**, and that is a different decision:
it is a 3 × 2 table of times, so a single key qualifies none of the six and six
inline `s` units would be six pieces of furniture repeating one fact. The row
labels carry it, and `setAccelCell()` draws a bare `--` where a stranded unit
would sit looking broken.

**Landscape's hero block is DERIVED** so the three hero pages cannot drift apart
again: `UI_LS_HERO_Y` 24, header 5, `UNIT_Y = HERO_Y + 72 - 13` = 83,
`SUB_Y = HERO_Y + 73 + 11` = 108. The 11 of gap (portrait uses 6) was set by EYE
against the panel and is the one number in the block that is not derived, because
`font_digits_64`'s glyphs reach the very bottom of the line box so the measured
gap and the seen gap are the same number here. DRIVE's third acceleration figure
is `SUB_Y + 18`, which IS derived and had to be — it overlapped the moment SUB_Y
went 103 → 108.

## Sub-lines

`font_digits_64` is 73px in a column with 172, so a hero alone left ~87px of
air. All three hero pages carry a sub-line:

- **DRIVE's is CONTEXTUAL** — the only readout on the instrument that changes
  meaning. It shows the journey's distance and cost and switches to the
  acceleration stopwatch while a LAUNCHED run is timing, plus 20 s after. It is
  allowed to because the trigger is the driver flooring it from a standstill,
  and an acceleration figure is interesting for ten seconds a year while a
  permanent cell would cost something useful on every drive.
- **TRIP's are `IN` (the whole window) / `RIDE` (the moving part), plus a second
  line `233 km to empty`.** The range line is the SELECTED meter's average over
  what 0x159 says is in the tank, which is why it lives here and not on HEALTH
  beside the litres it is computed from — HEALTH has no notion of which meter is
  selected. **The label TRAILS the number**, lower case: `setSubSuffix()` exists
  for it because `setSubValue()` puts its word in FRONT ("IN 24:15"), right for
  those since the word names which figure follows. This one reads as a sentence
  and the digits land at the left edge where the eye already is. Its unavailable
  form keeps the phrase and dashes only the number — a bare `--` with the label
  dropped would name nothing.
- **HEALTH's carries the TANK, in litres** — a state-of-the-vehicle fact read
  before setting off, like the voltage above it, not a thing to watch while
  moving. Landscape drops it into `UI_LS_HERO_SUB_Y` for free; portrait cost 20px
  and everything from the pair down moved by exactly 20 (pair 92→112, rules
  150→170 and 214→234, lamp pills 176→196, the FAULTS tap with them), with the
  sub-line at **95, not DRIVE's 96** — the page has precisely 20px of slack above
  the dot rail and the one pixel buys the last of it. The TRIP range line cost
  portrait another 18px (pairs 130→148 and 190→208, closing rule 250→268).
- **`setSubSnap()` is `setSubValue()` with three states**, and the split is not
  cosmetic: trip figures are DERIVED from meters we own so two states is honest,
  while a POLLED reading has three. Its colours step DOWN the palette (`UI_DIM`
  live, `UI_LABEL` held, `UI_FAINT` absent) rather than reusing `setSnap()`'s
  `UI_PRIMARY`/`UI_DIM`, because this line is subordinate to the hero above it —
  so live must be `UI_DIM`, and held needs a rung below that or held and absent
  are the same grey on the one readout where "slightly old" is the interesting
  answer.

## The compass tape

**A TAPE THAT SCROLLS UNDER A FIXED MARKER, identical geometry in both bands**:
152px at x=58 showing a 90° window — 1.69px to the degree, octant names 76px
apart, 15° ticks 25px apart. It was 156 until the link dot became an 11px icon;
the 4px came off the TAPE rather than the margin, because the tape is the element
with slack and the margin keeps the bar off the panel edge. The alternative — a
pointer travelling along a painted 0-360 scale — puts 0.67px on a degree, so a
junction turn would move the pointer 60px and a lane change nothing at all. One
geometry for both bands because a tape that changed width with the orientation
would change how much horizon a glance buys.

**Heading increases RIGHTWARD, so at 005° north sits LEFT of the marker.** This
reads as backwards and is not: pointing 5° east of north, north is 5°
anticlockwise of where the truck is aimed. `compassArc(5,0)` is -5 and the mark
lands 9px left of centre. A tape scrolling the wrong way looks almost right at
every non-cardinal heading, which is why `test_compass` asserts that exact pixel.

**The tape INTERPOLATES between frames**, and that is only defensible because
this signal has no jitter: across all 196 consecutive stationary sample pairs in
the drive capture the heading delta was exactly zero, so the glide can only
smooth real motion. 0x1B4 lands every 500 ms against a 10 Hz redraw, so without
it the tape steps five times a second through a turn. `UI_COMPASS_GLIDE` closes
45 % of the remaining arc per refresh (~90 % in 400 ms), and `compassGlide()`
SNAPS past `COMPASS_SNAP_DEG` because a hairpin apex really does turn 96° in one
sample.

**The tape BLANKS rather than dashing when the heading is absent** — every other
readout dashes because a number has an obvious empty form, a compass does not.
The pip stays, faint, so the bar does not look like it lost a widget.
`UI_HEADING_TTL_MS` is 2 s, three missed frames: the generic 5 s default is
wrong in the DANGEROUS direction here, because ten stale frames of a compass
still point somewhere entirely plausible and no other readout contradicts them,
so the dimming is the only thing that can.

## The flow bar

Scaled in **L/100km, 50 is full**, red above 35 (`UI_FLOW_FULL_L100`) — a hard
CLAMP, not an auto-range: 3.125 L/100 a segment, sixteen lit at 50, nothing
above, because a bar whose full scale moves means something different every time
the driver looks at it. **The NUMBER beside it is not clamped and must not be** —
the bar is the glance, the number is the fact. Rules and thresholds live in
`core/Snapshot.h`; only the GEOMETRY is here.

It was 50 L/h, and that was a REPORTED DEFECT — "the scale is jumping up and
down". There were TWO scales and one was invisible: segments off the L/h rate
while the readout switched to L/100 the moment the car moved. At 39 km/h and
11.7 L/100 the rate is only 4.6 L/h, so the number read 11.7 and the bar lit one
segment of sixteen. The fix is one quantity for both, which meant computing the
bar AFTER the unit decision rather than before it.

- **`flowSegs` is ZERO whenever the readout is in L/h** — the honest consequence
  of a fixed L/100 scale: stopped or crawling there is no distance to divide by.
  The readout says `1.2 L/h` and the bar says nothing.
- **L/100 is volatile at a crawl, and that is the QUANTITY, not the bar.** One
  litre an hour at 3 km/h is 33 L/100. The lever is `UI_FLOW_TO_L100_KPH`
  (3 km/h); do not "fix" it by putting the bar back on a second scale.
- **The pitch is DERIVED from the band, the gap is not** — both bands keep
  portrait's 2px gap; portrait is 12 wide on 14 (222px), landscape 10 on 12
  (190px). Landscape shipped at 8 on 10 = 158px because its comment sized it
  against a 165px left column that `UI_LS_RIGHT_W` 130 → 112 had already made
  187px. `16w + 15g = 187` has no usable integer answer, so the bar deliberately
  overhangs the content edge by 3px to x=200 (portrait's overhangs 230 to 232)
  and still clears the column rule at 207 by 7px. **If a landscape width ever
  moves again, this bar does not follow on its own.**

## The status bar

Clock, COMPASS, dots and link — **no ignition word and no `LINK` legend**.
`bcast.ignition` is still decoded and useful for logic; a driver looking at a
running instrument in a running truck already knows it is on, and a 22px bar has
no room for a field that is constant whenever anyone can read it.

**The link indicator is an 11px ring-and-dash icon**, crossed out in `UI_CRIT`
when down. A 6px dot carrying its one bit in colour alone was legible only to
somebody who already knew what it meant, and the word that explained it had just
gone. Two interlocking rings were tried and were wrong: at 9px a chain reads as a
blurred figure-8, because the interlock is the one feature carrying the metaphor
and the first thing to go at that size. One ring with a bar through it survives
the shrink. Drawn from an `mkOutline` ring and a staircase slash because there IS
no glyph — `lv_font_montserrat_14`'s FontAwesome subset has a WiFi fan and a USB
trident, neither of which says "the cable to the gateway".

**The slash is drawn TWICE**: a 4px HALO pass in `UI_BG` first, then a 2px LINE
pass in `UI_CRIT` on top. A bare red diagonal across a red ring shares its colour
and touches it at four points, so the eye reads one blob rather than a thing and
a cancellation. The halo must be a SEPARATE PASS rather than per-block borders,
because the blocks overlap (which is what makes the diagonal solid) so per-block
borders would paint each halo over its neighbour's core and chop the line into
dashes. **Ten blocks, and the count is derived**: the core travels
`UI_LINK_D - UI_LINK_CORE` = 9px, so ten positions step exactly 1px and
consecutive cores overlap by one. Eight stepped 9/7 and left two visible nicks.

## Landscape specifics

- **The dot rail folds into the status bar.** 26px is 8 % of portrait's height
  and 11 % of landscape's — off the only axis that is short. One 22px bar
  carries clock, dots and link, and content gets 218px instead of 192. **The dots
  are not CENTRED in it** — the compass tape took the middle, so they sit right,
  35px off the edge. That is the one place the two bands disagree about the dots,
  and it is forced.
- **Two columns replace the stack**: hero left, the same values in a 112px right
  stack, insets of 10px. **The two columns share a fixed total**: content =
  `(UI_W - 1 - 2*PAD) - PAD`, which is 271px at PAD 12 and 279 at PAD 10
  *whatever the divider does* — so the only way to give the hero band more is to
  take it from the inset. TRIP needs 187 (a 164.2px `247.8` plus an 18.6px `KM`)
  and the stack needs 92 (`4163` + `rpm` = 90.1); 279 covers both, 271 does not.
  **The 165px some older notes cite is STALE** (PAD 12 with `UI_LS_RIGHT_W` 130);
  the shipped column is 187.
- **No new type.** Every numeral is `font_digits_64` or `font_digits_30`, every
  word `font_label_14`. Enforced, not lucky: four tyre pressures at hero size
  want 238px of the 218 available, so they were drawn and thrown out.
- **SETTINGS is the one page that cannot take two columns.** ORIENTATION plus its
  chip is ~205px and ADAPTER plus its value ~106px — 351px of a 320px board.
  Controls keep full width; INFO became a bottom strip, which portrait's 240px
  could not have held.
- **TRIP has NO HEADER ROW** — the only page without one. The page name went
  redundant when the selector arrived (three meters listed with one lit says
  which card you are on better than a word does) and the unit belongs to the
  number; the row's space went to the selector. **The selector row is sized by
  the LANDSCAPE band**: three 54px cells = 162px of the left column. Portrait
  would have taken 60px cells happily; sizing to the narrow band and using the
  result unchanged in both is what keeps the rotation a toggle.
- **DRIVE's hero unit is `L/100`, not `L/100km`** — the long form right-aligns to
  x=122 and the hero reaches that at four glyphs, which a cold crawl out of a
  driveway really does produce.

## TYRES

**2×2 in both bands**, reading order `SENSOR 1` `SENSOR 2` on top, `SENSOR 3`
`SENSOR 4` beneath. Four across made no claim where a 2×2 invites one; changed on
the owner's explicit instruction after that was put to them. The cells read
`SENSOR`, never LF/RF/LR/RR — the layout suggests a shape while the labels refuse
to name a corner, and nothing here is a mapping until one is measured.

**`bar` goes in all four cells, in both bands** — a key belongs to a HERO, the
one number a page is about; this page has four equal numbers so a single key at
the top qualifies none of them, and it sits closest to `SENSOR 2`. It fits on
measured clearance: `bar` is 20.9px right-aligned inside the 159px cell and the
widest string the formatter can emit still leaves 10.6px (a raw 255 through
`UI_PSI_TO_BAR` caps at `17.58`, so two digits always start with `1`).
Right-aligned IN the cell, not trailed after the value, because the value is
centred and its width changes with the reading (`--` 27px, `2.34` 62px, `17.58`
71.3px) — trailed, it would slide sideways on four cells at once.

**A low corner is coloured but obeys hold-and-dim**: `UI_CRIT` fresh,
`UI_CRIT_DIM` stale. It runs *after* `setValScaled` in `refresh()` and only ever
paints low corners, so a recovered corner is restored by that call — swap the
order and it stays red for good.

## Other measured constraints

- **The right-hand stack values hang off the CAPTION'S BASELINE, not its line
  box.** They were 1px off the rule beneath them; `font_digits_30`'s
  `base_line = 0` is why. Taking back the 3px of descender an all-caps caption
  never uses shortens the block 51 → 48 and spends it on the gap that was wrong:
  2px above, 3px below. The stack units sit on the value's baseline too — they
  were 10px above it, which made the stack and the hero read as two different
  typographic systems.
- **Numeric precision is a layout constraint, not a taste.** The stack cell is
  106px (portrait's pair 110) and the unit eats the right end. COST is 1 decimal
  (two ran off the panel past €1000); DRIVE's TRIP A/B distances are whole km
  (`km` is a 19px unit and 999.9 only just fitted; DRIVE worst case `9999 km` =
  88.9). TRIP's own hero keeps its decimal.
- **FAULTS landscape goes to one line per fault** — two-line rows fit three in
  148px and 16 codes would need six screens. State moved to a 3px coloured left
  edge to buy back width; longest descriptions ellipsise. *(Designed, not built.)*
