#pragma once
// Compass tape arithmetic: heading normalisation, octant naming, the shortest
// arc between two headings, the glide that carries a tape between them, and
// the marks that tape wants drawn.
//
// This is presentation arithmetic, and it lives in core/ for exactly the
// reason core/Snapshot.h sets out at length: psi -> bar, the L/h <-> L/100km
// hysteresis and the low-corner test were all vehicle logic wearing a
// presentation hat, trapped in a translation unit bound to LVGL and therefore
// impossible to test without a board and a car. The wrap at 0/360 is the same
// kind of thing, only sharper -- one boundary, one degree either side of it,
// and the tape draws the wrong half of the horizon. It is testable on a laptop
// or it is not testable at all.
//
// THE SIGNAL THIS SERVES: 0x1B4 on CAN-B, DLC 6, every 500 ms.
//
//   byte 0   8-way octant enum: 0=N 1=NE 2=E 3=SE 4=S 5=SW 6=W 7=NW
//   byte 1   magnetic heading, 2 deg/count
//   byte 2   status, 0x07
//   3..5     zero
//
// Nothing in this file decodes that frame. Broadcasts are decoded in
// core/Broadcast.cpp and that is where 0x1B4 belongs when it is wired up; this
// is the arithmetic that sits above the decode, kept apart so that both halves
// can be written and tested on their own.
//
// EVERY FIGURE BELOW IS MEASURED, and was re-measured while writing this file,
// over `jeep-can-b-logs/drive log - becici to school on top of the steep hill
// and back - 2026-09-17_074126.log` -- 2953 0x1B4 frames of one real drive --
// plus an independent 1584-frame session from 2019 that agrees with all of it.
//
//  * DLC is 6 in 2953 of 2953. Period mean 502.8 ms, median 500 ms, and
//    99.83 % of the gaps fall between 480 and 520 ms.
//  * byte 2 is 0x07 in 2953 of 2953. Bytes 3..5 are 00 00 00 in 2953 of 2953.
//  * byte 1 spans 0..178 observed, against a documented ceiling of 179
//    (-> 358 deg). 0xFF IS NEVER SEEN: THERE IS NO UNAVAILABLE SENTINEL ON
//    THIS FRAME. That decides how a stale heading has to be drawn. SnapAge's
//    three states still apply, but SNAP_ABSENT here can only ever mean "we
//    have never heard the frame" -- never "the module says it has no value",
//    because this module has no way of saying that. A compass reading is held
//    and dimmed, or it is nothing at all.
//  * AND IT MUST BE DIMMED, because 0x1B4 GOES ON TRANSMITTING A VALID,
//    UNCHANGED HEADING STRAIGHT THROUGH AN IGNITION-OFF TRANSITION -- visible
//    in scripts/playback.log. It is not a key-off indicator and it does not
//    stop when the car does, so nothing in the frame itself will ever tell a
//    presenter that the number has gone stale. Only a TTL can. A held heading
//    still drawn as a live one is the frozen-but-plausible failure this whole
//    instrument is written around, aimed at the one readout a driver would
//    turn a wheel because of.
//  * The heading is BIT-FOR-BIT STATIC WHEN STOPPED. Across 196 consecutive
//    stationary (<= 2 km/h) sample pairs the delta was exactly 0.0 deg, every
//    single time. There is no jitter here to filter, which is worth stating
//    plainly: compassGlide() below exists for the EYE's sake and not for the
//    signal's, and anyone tempted to add a filter should know first that there
//    is nothing to filter.
//  * Real slew is 18-26 deg/s through an ordinary intersection turn. The peak
//    across the whole drive is 192 deg/s, one 500 ms sample at the apex of a
//    tight hairpin at 15-19 km/h -- genuine, not a data error, and the entire
//    reason COMPASS_SNAP_DEG exists.
//
// PURE: no ESP-IDF, no Arduino, no LVGL.
#include <stdint.h>

// byte 1 scaling and its ceiling. 2 deg/count is the whole reason the octant
// binning below is worth an argument: the finest distinction this signal can
// draw is 2 degrees, and the sector boundaries sit on half degrees.
#define COMPASS_DEG_PER_COUNT 2.0f
#define COMPASS_RAW_MAX 179u        // byte1 ceiling -> 358 deg

// Minor tick spacing on the tape. 15 divides 45, so every third tick is an
// octant and the major/minor test is arithmetic rather than a table.
#define COMPASS_TICK_DEG 15u

// How many marks a caller must have room for. The shipped tape -- 156 px over
// a 90 deg span -- can never produce more than seven, so this is headroom
// rather than a measurement. A much wider span WOULD want more (a full 360 at
// 15 deg ticks is 24 marks), and compassMarks() truncates at `max` rather than
// overrunning, so this constant bounds the array while the parameter bounds
// the writes.
#define COMPASS_MAX_MARKS 16

// 0..360, wrapping negatives and >= 360 correctly. Exactly 360 comes back as
// 0, not as 360: the range is half open, which is what lets everything else
// here subtract two normalised headings and know the difference is strictly
// inside -360..+360.
float compassNorm(float deg);

// "N","NE","E","SE","S","SW","W","NW" for 0..7; "" for anything else.
//
// NEVER INVENT A DIRECTION. Same rule and the same reasoning as
// ignitionFrom() in core/Broadcast.cpp returning IGN_UNKNOWN for an unmapped
// key byte: byte 0 of 0x1B4 has only ever been observed at 0..7, so a 9 means
// we are looking at something that is not this frame, and the honest answer to
// that is nothing at all rather than a plausible compass point.
const char* compassOctantName(uint8_t octant);

// Which octant a heading falls in.
//
// ROUND BINNING, NOT FLOOR, and this is the one decision in the file that
// could have been got wrong silently. Sector CENTRES are at 0/45/90/...,
// boundaries at 22.5/67.5/..., so the rule is round(deg/45) mod 8.
//
// THE CAR SETTLES IT. byte 0 and byte 1 are two views of the same heading and
// the capture holds 2953 of them, so the two candidate rules can simply be
// scored against the module's own answer:
//
//     round(byte1 * 2 / 45) mod 8   agrees in 2937/2953 =  99.46 %
//     floor(byte1 * 2 / 45) mod 8   agrees in 1230/2953 =  41.65 %
//
// and the 2019 session gives 99.75 % for round, independently. Floor is not a
// near miss wanting a tie-break; it is wrong three times in five.
//
// ALL SIXTEEN DISAGREEMENTS ARE BOUNDARY QUANTISATION, not a flaw in the rule.
// Every one is a raw count whose doubled value sits half a degree BELOW a
// sector boundary that the module -- which holds the heading finer than
// 2 deg -- has already crossed: byte1=146 -> 292.0 deg against the 292.5
// boundary (8 frames; the module says NW, this says W), byte1=101 -> 202.0
// against 202.5 (4 frames), byte1=11 -> 22.0 against 22.5 (4 frames). In all
// sixteen the module is one octant AHEAD and never behind, which is exactly
// what truncating a finer heading to 2 deg/count does, and is the strongest
// evidence available that byte 1 is floor(trueDeg / 2). The residual is the
// +/-1 deg the scale itself costs and nothing here can recover it.
uint8_t compassOctantFor(float deg);

// Shortest signed arc from `from` to `to`, in -180..+180. Positive is
// clockwise, the direction a heading increases.
//
// Exactly antipodal resolves to +180 rather than -180. The tie has to break
// somewhere, and breaking it at a fixed sign means a glide across the far side
// of the rose picks a direction and holds it, rather than flipping between two
// equally short answers as the target wobbles by a hundredth of a degree.
float compassArc(float from, float to);

// Ease `shown` toward `target` along the shortest arc for one UI tick.
// `alpha` is the fraction of the remaining arc to close (0..1).
// Returns a normalised 0..360 -- always, for every input, including the ones
// a caller should not have passed.
//
// TWO SNAPS, AT OPPOSITE ENDS, and both are measured rather than felt:
//
//  * Above COMPASS_SNAP_DEG the glide is abandoned and the target taken whole.
//    The hairpin in the capture moves 96 deg in ONE 500 ms sample (192 deg/s,
//    at 15-19 km/h, genuine). Easing that at any alpha worth having takes
//    seconds to arrive, so the tape would be reading out a corner the driver
//    has already finished -- lagging, not smoothing, which is the
//    frozen-but-plausible failure wearing a smooth animation.
//  * Below COMPASS_SETTLE_DEG the target is taken whole as well, because a
//    proportional glide is asymptotic and otherwise never arrives. A quarter
//    of a degree is an eighth of one raw count, so snapping there cannot cross
//    a sector boundary that the signal was able to resolve in the first place.
//
// Both comparisons are inclusive. At exactly a threshold either behaviour is
// defensible to within a fraction of a pixel, and inclusive is one fewer edge
// to reason about.
float compassGlide(float shown, float target, float alpha);
#define COMPASS_SNAP_DEG   60.0f
#define COMPASS_SETTLE_DEG 0.25f

// One thing the tape wants drawn.
struct CompassMark {
  int16_t  x;      // pixel centre within the tape, 0..w
  uint16_t deg;    // the heading it marks, a multiple of COMPASS_TICK_DEG
  bool     major;  // deg % 45 == 0 -- draw the octant NAME, not a tick
};

// Every mark visible in a tape `w` px wide showing `spanDeg` degrees centred
// on `deg`. Returns how many were written, never more than `max`.
//
// The tape scrolls under a FIXED CENTRE MARKER, so the current heading is
// always at x = w/2 and a mark at heading m lands at
//
//     x = w/2 + compassArc(deg, m) * (w / spanDeg)
//
// emitted only when 0 <= x <= w. Marks come out ordered by x and never
// decreasing -- they are walked in ascending heading across the window and
// that map is monotonic, so the ordering is a property of the walk and not of
// a sort. (Two marks can round to the same pixel on an absurdly wide span;
// none can ever come out backwards.)
//
// IT IS CORRECT ACROSS THE WRAP, which is the only hard part here. A heading
// of 5 deg shows 330/345/0/15/30/45 left to right, because the window is
// walked in UNWRAPPED degrees -- -30, -15, 0, 15, 30, 45 relative to the
// heading -- and each mark is normalised only to label it. Walking a 0..345
// table and wrapping the comparison instead is how this gets written wrong:
// the window straddles the seam, so the visible set is not a contiguous run of
// that table and any loop that assumes it is drops half the tape.
uint8_t compassMarks(CompassMark* out, uint8_t max, float deg,
                     uint16_t w, uint16_t spanDeg);

// ---------------------------------------------------------------------------
// THE DIAL: the same rose, lying flat on the ground and seen in PERSPECTIVE.
//
// The tape above and the dial below are two instruments and they answer two
// different questions. The tape shows a WINDOW -- 90 deg of horizon at 1.73 px
// to the degree, which is what makes a lane change visible -- and says nothing
// at all about the other 270. The dial shows the WHOLE ring at once, so south
// is somewhere the eye can find without scrolling there, and pays for that in
// resolution. Both are heading-up, both turn under a fixed marker; only the
// projection differs.
//
// The ring is drawn as a disc on the ground, viewed from BEHIND AND ABOVE, so
// the driver's own heading is at the TOP of the screen and therefore the
// FARTHEST away. Bearings behind the truck come round the near side, close to
// the viewer, big.
//
//     a = markBearing - heading        // 0 = the heading, at the top
//     u = sin(a),  v = cos(a)          // v = +1 farthest, -1 nearest
//     f = 1 / (1 + v * COMPASS_DIAL_E) // the perspective divide
//     x = cx + rx * u * f
//     y = cy - ry * v * f
//
// THE DIVIDE IS THE WHOLE POINT AND MUST NOT BE SIMPLIFIED AWAY. Drop `f` and
// what is left is an orthographic ellipse -- a circle squashed vertically --
// which compresses the far half and the near half by EXACTLY THE SAME AMOUNT
// and therefore reads as an oval seen edge-on rather than as a disc lying on
// the ground. With e = 0.30 the ring reaches ry/1.3 above the centre and
// ry/0.7 below it: at ry = 50 that is 38 px up against 71 px down, and that
// ASYMMETRY is the entire visual effect. It is also the only thing an
// automated check can hold on to, which is why test_compass asserts those two
// numbers by name -- an orthographic implementation puts both at exactly ry
// and looks plausible in every other respect.
//
// NOTHING HERE DRAWS ANYTHING. Ui.cpp has no line, arc or circle primitive --
// the whole instrument is rectangles, dots and labels at absolute positions --
// so a dial is a SET OF COMPUTED POSITIONS for small dots and the arithmetic
// that produces them is ordinary pure code. That is the same argument that put
// the tape here and not in the renderer, only more so: there is more of it.
#define COMPASS_DIAL_E 0.30f

// Tick spacing on the dial, and the count that follows from it. Same 15 deg as
// the tape, for the same reason -- 15 divides 45, so every third mark is an
// octant and major/minor stays arithmetic -- but the dial shows the closed ring
// rather than a window, so the count is fixed at 360/15 and is not headroom.
#define COMPASS_DIAL_TICK_DEG 15u
#define COMPASS_DIAL_MARKS 24          // 360 / 15

// One dot of the ring.
struct CompassDialMark {
  int16_t  x;       // pixel centre
  int16_t  y;
  uint8_t  size;    // dot diameter, px -- larger when nearer
  uint8_t  depth;   // 0 = nearest, 255 = farthest; for colour mixing
  uint16_t deg;     // the bearing this mark names, a multiple of 15
  bool     major;   // deg % 45 == 0 -> the renderer draws the octant NAME
};

// Project the whole ring for `heading`. Returns how many marks were written,
// never more than `max`.
//
// ALL 24 ARE ALWAYS EMITTED and nothing is culled: the ring is closed, every
// mark is on screen by construction, and a near mark is the most prominent
// thing on the dial rather than the least. There is no visibility test here at
// all, which is the one structural difference from compassMarks() -- that one
// walks a window and drops what falls outside it.
//
// EMITTED FARTHEST FIRST, by descending v, so a caller that assigns widgets in
// order gets a painter's ordering for free: the near dots are written last and
// so are drawn last, over the far ones they overlap. `depth` is derived from v
// monotonically, so far-first is the same statement as `depth` never
// increasing, and that is how it is asserted.
//
// THE ORDER IS RIGID UNDER ROTATION -- the k-th mark out is always the same arc
// from the heading, so a caller holding one widget per slot sees the ring turn
// and nothing else. Eleven pairs are TIED at any heading (the ring is symmetric
// about the screen's vertical), and they are broken by u, which puts the
// clockwise half first. Leaving them to a stable sort instead would tie their
// order to where the 0/360 seam falls in the walk, and two dots would trade
// places as the truck turned through north. See the comment on the sort itself.
//
// `depth` is (v + 1) / 2 scaled to 0..255 and rounded: 0 at the nearest mark,
// 255 at the farthest, half way at the two that sit level with the centre.
// `size` interpolates dotMax at depth 0 down to dotMin at depth 255, and is
// CLAMPED into the caller's own two numbers whichever order they arrive in --
// a reversed pair gets a ring that grows with distance, which is wrong and
// visible, rather than a zero-diameter or runaway dot, which is a crash or a
// smear on a panel nobody can attach a debugger to.
//
// CORRECT ACROSS THE WRAP, and it gets there the same way everything else in
// this file does: compassArc() supplies the signed offset from the heading to
// each bearing, so there is no modular arithmetic open-coded here to get wrong
// at 0/360. sin and cos do not care that the arc is signed.
//
// ROUNDED TO NEAREST, not truncated. Truncation is toward zero on the way out
// of a float, so every x left of centre and every y above it would move half a
// pixel the wrong way while the other halves stayed put -- the ring would come
// out biased up and left, by a whole pixel across a diameter, which on a 100 px
// radius is visible as an off-centre dial rather than as rounding.
uint8_t compassDial(CompassDialMark* out, uint8_t max, float heading,
                    int16_t cx, int16_t cy, int16_t rx, int16_t ry,
                    uint8_t dotMin, uint8_t dotMax);
