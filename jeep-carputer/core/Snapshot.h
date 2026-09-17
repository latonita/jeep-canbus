#pragma once
// THE ONE PUBLICATION BOUNDARY between the vehicle and the screen.
//
// Everything the driver looks at arrives here, resolved, and the renderer reads
// nothing else. That is a change of architecture, not a convenience wrapper,
// and it settles three problems that were each being solved separately and
// slightly differently.
//
// 1. STALENESS WAS REIMPLEMENTED FIVE TIMES. `VS_DEFAULT_TTL_MS`,
//    `UI_TYRE_TTL_MS`, `UI_SPARE_TTL_MS`, `BACKLIGHT_SIGNAL_TTL_MS` and
//    `BC_PEDAL_TTL_MS` each encoded the same rule -- hold a reading, dim it
//    once it is older than its own poll rate, show dashes only for an explicit
//    unavailable -- against a different clock read at a different moment, in a
//    translation unit bound to LVGL and therefore impossible to host-test. Two
//    of them had to be hand-synchronised with `kPlan` in main.cpp, which is a
//    maintenance obligation written down in CLAUDE.md rather than enforced.
//    The TTL decision now happens ONCE, here, in code the laptop can run.
//
// 2. THE RENDERER SAW LIVE STRUCTURES BEING MUTATED UNDER IT. The fault list
//    is insertion-sorted at end-of-scan while the drill-down walks it, which is
//    a torn STRUCTURE rather than a torn word -- rows could duplicate or vanish
//    for a frame. A snapshot is copied whole, so a rendered frame is internally
//    consistent by construction: every number on the glass belongs to the same
//    instant.
//
// 3. PRESENTATION ARITHMETIC WAS TRAPPED IN THE UI. psi -> bar, the
//    L/h <-> L/100km switch and its hysteresis, the low-corner test and the
//    flow bar's segment count all lived inside `Ui.cpp` and could not be
//    tested without a board and a car. They are vehicle logic wearing a
//    presentation hat, and they are now here with the rest of it.
//
// What did NOT move: anything about layout, colour, fonts or geometry. This
// header decides WHAT to say and HOW MUCH to believe it. `Ui.cpp` still owns
// entirely how it looks.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "Accel.h"
#include "Broadcast.h"
#include "Faults.h"
#include "Trip.h"
#include "VehicleState.h"

// ---------------------------------------------------------------------------
// How much a readout can be believed. THREE states, and the third is the one
// projects get wrong.
//
// A value that stops arriving must be HELD, not blanked: a gap in a poll cycle
// is normal, and blinking dashes are worse than a number one second old. But a
// held value must never pass for a live one, so it goes dim. Dashes are
// reserved for the case where the module said, explicitly, that it has no
// value -- an all-ones sentinel -- because that is the only time we actually
// know nothing rather than knowing something slightly old.
//
// Collapsing HELD into FRESH is the frozen-but-plausible failure this whole
// instrument is written to avoid. Collapsing HELD into ABSENT throws away a
// perfectly good reading and makes the panel flicker.
enum SnapAge : uint8_t {
  SNAP_ABSENT = 0,   // nothing known, or an explicit unavailable -> dashes
  SNAP_HELD,         // a real reading, past its TTL -> show it, dimmed
  SNAP_FRESH,        // within its TTL -> show it plainly
};

// A number and whether to trust it. `v` is meaningless when age is SNAP_ABSENT
// and is deliberately left at 0 there, so a caller that forgets to check does
// not render an old value as a current one.
struct SnapVal {
  float   v;
  SnapAge age;
};

static inline bool snapShow(const SnapVal& s) { return s.age != SNAP_ABSENT; }
static inline bool snapLive(const SnapVal& s) { return s.age == SNAP_FRESH; }

// ---------------------------------------------------------------------------
// Product rules that used to live in Ui.cpp.
//
// These are here rather than there because they are statements about the CAR,
// not about the screen: what counts as a soft tyre does not change when the
// panel rotates. Ui.cpp still uses UI_FLOW_SEGS for its widget array, which is
// why the name is unchanged -- this is a move, not a redesign.

// WCM 21 3B reports one psi per bit, with no offset and no scaling. The cabin
// reads in bar, so the conversion lives here rather than in the decoder: the
// wire value stays the wire value, and only the presentation changes.
//
// For scale: the owner's stated target band of 2.1-2.3 bar is 30.5-33.4 psi,
// and the recorded corners (30-34 psi) are 2.07-2.34 bar.
#define UI_PSI_TO_BAR 0.0689476f

// Bottom of the owner's stated cold band. Below this a road wheel is called
// low; the spare is excluded, since it reads about 0.5 bar and would otherwise
// hold the warning on permanently.
#define UI_TYRE_LOW_BAR 2.1f

// How long a tyre reading counts as fresh. TWICE the poll interval in kPlan,
// which is a minute for the corners and for the spare, so both tolerate one
// missed request and no more.
//
// These are a property of the POLL RATE, not of the signal: judged against the
// default 5 s TTL a value asked for once a minute is stale fifty-five seconds
// out of every sixty, and a reading as fresh as its poll rate allows is not a
// stale reading. It just makes the dimming mean nothing, on the page where the
// dimming is the guard against believing a frozen number.
#define UI_TYRE_TTL_MS  120000u
#define UI_SPARE_TTL_MS 120000u

// How long a heading counts as fresh, and like the tyres it is a property of
// THE SIGNAL'S OWN RATE. 0x1B4 lands every 500 ms -- mean 502.8, median 500,
// 99.83 % of gaps inside 480-520 -- so 2 s tolerates three missed frames and
// no more.
//
// The generic VS_DEFAULT_TTL_MS of 5 s is wrong here, and wrong in the
// dangerous direction: ten missed frames of a compass still points somewhere
// entirely plausible. No arithmetic gives it away and no other readout
// contradicts it, so the dimming is the only thing that can -- which is
// precisely the frozen-but-plausible failure this instrument is built around.
#define UI_HEADING_TTL_MS 2000u

// How long a tank reading counts as fresh. 0x159 lands about once a second, so
// 4 s tolerates three missed frames -- the same rule as the heading above and
// the tyres below: a TTL is a property of THE SIGNAL'S OWN RATE.
//
// It is generous against the quantity itself, and that is the point of deriving
// it from the rate rather than from the reading: a tank does not move in four
// seconds, so this dims only when the frame has genuinely stopped arriving.
#define UI_FUEL_TTL_MS 4000u

// ---- The flow readout ----------------------------------------------------
//
// 0x1A7 is a rate in known units -- L/h = Q/160 -- so unlike the first version
// of this screen the bar CAN be scaled to something real and CAN carry a number
// beside it.
//
// Full scale is 50 L/h. An OM642 peaks higher than that, but almost no driving
// happens up there: idle is under 1 L/h and a cruise is around 10, so a bar
// scaled to the engine's true peak would spend its life in the bottom sixth and
// show nothing useful. 50 puts a cruise at a fifth lit, leaves room for a hard
// overtake, and pegs on full throttle -- which is information in itself.
//
// It was 40 until the bar gained a critical state. The colour now carries the
// alarm, so the LENGTH is free to describe a wider range without the top of it
// being somewhere the driver never sees.
// It CLAMPS rather than auto-ranging: a bar whose full scale moves means
// something different every time the driver looks at it.
// THE BAR IS SCALED IN L/100km, AND ITS SCALE NEVER MOVES.
//
// It used to be scaled in L/h, which was the bug the owner reported as "the
// scale is jumping up and down". The bar and the number beside it were on
// DIFFERENT QUANTITIES: segments came off the L/h rate while the readout
// switched to L/100 the moment the car was moving. At 39 km/h and 11.7 L/100
// the rate is only 4.6 L/h, so the number read 11.7 and the bar lit ONE
// segment of sixteen -- and as speed changed the per-distance figure moved a
// long way while the rate barely moved, so the two drifted apart in front of
// the driver. Nothing was auto-ranging; it was two scales, one of them
// invisible.
//
// One quantity now. 50 L/100km is full, 3.125 per segment, sixteen lit at 50
// and NOTHING above it -- a hard clamp, not an auto-range, because a bar whose
// full scale moves means something different every time it is looked at. The
// NUMBER beside it is not clamped and must not be: the bar is the glance, the
// number is the fact.
//
// KNOWN CONSEQUENCE, and it is a property of the quantity rather than of the
// bar: L/100 is a rate DIVIDED BY SPEED, so at a crawl it is genuinely
// volatile -- one litre an hour at 3 km/h is 33 L/100, which is nearly
// critical. The readout has always done this; the bar now does it too, because
// they are finally the same number. The lever, if it grates in traffic, is
// UI_FLOW_TO_L100_KPH below -- raising it keeps the readout in L/h until the
// per-distance figure has a speed worth dividing by.
#define UI_FLOW_SEGS 16
#define UI_FLOW_FULL_L100 50.0f

// Above this the bar goes critical.
//
// The threshold is on the PER-DISTANCE figure, not on L/h, and it only applies
// while the readout is actually in L/100 -- stopped or crawling there is no
// distance to divide by, and an idling engine would otherwise sit permanently
// in alarm for burning under a litre an hour. See Snapshot::flowPerDistance.
//
// It is a rate of SPENDING, not a fault. Nothing is wrong with the car at
// 40 L/100km; you are just paying for it, which is exactly the thing a fuel
// card exists to say out loud.
#define UI_FLOW_CRIT_L100 35.0f

// Where the instantaneous readout changes unit.
//
// Stopped, or crawling below the speed this firmware already calls stopped,
// litres-per-100-km is arithmetic on a number approaching zero: an idling
// engine at 1 km/h reads 80 L/100km, which is arithmetically true and useless.
// Sitting in a jam or parked and running, litres per HOUR is the figure that
// means something -- it is what tells you what an hour of waiting costs.
//
// Two thresholds, not one, and the gap between them is deliberate. A crawling
// car crosses a single threshold over and over, and a readout that flips
// between "0.9 L/h" and "62.0 L/100" every few seconds cannot be read -- which
// is precisely the traffic-jam case this exists for. Below 2 km/h it is always
// L/h; above 3 km/h always L/100; in between it keeps whatever it was showing.
#define UI_FLOW_TO_LPH_KPH TRIP_MOVING_KPH
#define UI_FLOW_TO_L100_KPH 3.0f

// ---------------------------------------------------------------------------
// Everything the screen may read, as of one instant.
//
// The three vehicle structures are copied WHOLESALE rather than re-modelled
// field by field. They are small (376 bytes between them), they are already
// plain data, and a copy is what buys the consistency in point 2 above. A
// hand-picked projection would have to be extended every time a page wants one
// more field, and would be one more place to forget.
struct Snapshot {
  // The clock this was resolved against. Every age in here was decided against
  // this one value, which is the point: two readouts can no longer disagree
  // about what "now" was.
  uint32_t builtMs;

  TripSet  trips;
  AccelSet accel;
  FaultSet faults;

  // --- readouts that exist on BOTH paths ----------------------------------
  // Already resolved: the broadcast copy wins when it is showable, and the
  // polled copy is the fallback. That choice used to be an if/else in the
  // renderer, which meant the rule "anything on the bus beats a request" was
  // stated in a comment in Ui.h and implemented in one branch of one refresh
  // function. It is a statement about the VEHICLE -- broadcasts arrive at 151
  // frames/s for free while a poll costs a transaction -- so it belongs here,
  // where it can be tested and where a second page cannot decide differently.
  SnapVal speedKph;
  SnapVal rpm;
  SnapVal volts;
  SnapVal coolantC;
  SnapVal ambientC;

  // --- broadcast only -----------------------------------------------------
  // No snapshotPrefer() here: nothing answers a KWP request with a heading, so
  // there is no polled copy to fall back to.
  //
  // `headingOctant` sits immediately beside `headingDeg` and may be read ONLY
  // when snapShow(headingDeg) says the heading is showable. It is the module's
  // own enum off the same frame and has NO AGE OF ITS OWN, so pairing it with
  // the SnapVal is what stops the two disagreeing about whether we know
  // anything at all -- a bare byte still reading "NE" beside a set of dashes
  // is exactly the confident wrong direction the decoder refuses to invent.
  // This is the tyreLow[]/tyreBar[] precedent: a bare value sitting NEXT TO a
  // SnapVal, never inside one.
  SnapVal headingDeg;       // 0..358, already scaled from the 2 deg/count wire value
  uint8_t headingOctant;    // the module's own enum, meaningless unless snapShow(headingDeg)

  // --- polled only --------------------------------------------------------
  // LITRES, not a percentage, and the rename is the finding.
  //
  // 0x159 b5 is 0.5 L per count. The map offered `% = raw/2.55` and the CCN
  // poll this replaced used x0.5 out of a vendor table, and BOTH were wrong
  // about the unit -- the transform x0.5 was right, what it produces is litres.
  //
  // THE EVIDENCE IS THE CEILING. Across every log in the repository the largest
  // valid raw ever sent is 154, which is 77.0 L against this truck's 77.6 L
  // (20.5 US gal) tank. A percentage scale has no reason to stop at 154 of 255;
  // a litre scale stops exactly where the tank does. It also settles an owner
  // observation that /2.55 could not: a drive logged the morning after a fill
  // reads 138-142 raw, which is 69-71 L or about 90 % -- "almost full", where
  // /2.55 insisted on 54 %.
  //
  // Still uncalibrated in the strict sense -- nobody has put a receipt against
  // it. That is one fill away and is the cheapest of the three numbers listed
  // under "ONE TANK RUN" in CLAUDE.md, because litres are what a pump prints.
  SnapVal fuelLitres;
  SnapVal tyreBar[4];    // already psi -> bar
  SnapVal spareBar;

  // The low-corner test, decided here so the page cannot disagree with the
  // count at the foot of it. A corner is only `low` if it is also showable --
  // an absent reading is not a warning.
  bool    tyreLow[4];
  uint8_t tyreLowCount;

  // --- fuel flow ----------------------------------------------------------
  // `flow` is in whichever unit `flowPerDistance` names, already switched and
  // already through the hysteresis. `flowCrit` is the 35 L/100km rule, which
  // can only ever be true while `flowPerDistance` is.
  SnapVal flow;
  bool    flowPerDistance;
  bool    flowCrit;
  // 0..UI_FLOW_SEGS. ZERO whenever flowPerDistance is false: a fixed L/100
  // scale can only show L/100, and stopped there is no distance to divide by.
  // The readout says "1.2 L/h" and the bar says nothing, which is the honest
  // pair -- the alternative was drawing an L/h value against an L/100 scale,
  // which is exactly what was wrong before.
  uint8_t flowSegs;

  // --- link / bus ---------------------------------------------------------
  uint32_t frames;       // cumulative frame counter, for the bus-rate readout
  bool     linkUp;
};

// State the builder has to carry BETWEEN builds.
//
// Only the flow unit needs it, and it needs it because hysteresis is memory by
// definition: the answer depends on which side the readout came from. Keeping
// it here rather than as a static inside the builder is what lets a host test
// drive a whole approach-and-stop sequence and assert the readout does not
// flicker.
struct SnapshotSource {
  bool flowPerDistance;
};

void snapshotSourceInit(SnapshotSource& src);

// Resolve one frame's worth of truth.
//
// `state` may be null (nothing polled yet); the polled readouts then come back
// SNAP_ABSENT rather than zero. `slotXxx` are the VehicleState slot indices the
// caller resolved at startup, -1 for "not registered" -- passing them in keeps
// this function free of any opinion about signal NAMES, which belong to
// SignalDb and change with the vehicle.
struct SnapshotSlots {
  int8_t coolant;
  int8_t volts;
  int8_t ambient;
  int8_t speed;
  int8_t rpm;
  int8_t tyre[4];
  int8_t spare;
};

void snapshotSlotsInit(SnapshotSlots& s);

void snapshotBuild(Snapshot& out, SnapshotSource& src, const SnapshotSlots& slots,
                   const BroadcastState& bc, const TripSet& trips,
                   const AccelSet& accel, const FaultSet& faults,
                   const VehicleState* state, bool linkUp, uint32_t nowMs);

// Resolve one polled slot on its own. Exposed because the TTL argument is the
// whole subtlety and it deserves its own tests.
SnapVal snapshotSlot(const VehicleState* state, int8_t slot, uint32_t nowMs,
                     uint32_t ttlMs, float scale);

// Resolve one broadcast value from its stamp. `stampMs == 0` means the module
// said it has no value -- Broadcast.cpp zeroes the stamp for an explicit
// sentinel and HOLDS it for a dropped frame, and that distinction is the
// difference between dashes and a dimmed number.
SnapVal snapshotBcast(float value, uint32_t stampMs, uint32_t nowMs, uint32_t ttlMs);

// `first` if it is showable at all, otherwise `second`.
//
// Note it prefers a HELD broadcast over a FRESH poll, and that is deliberate:
// the two are readings of the same quantity, so the honest thing is to keep
// showing the source the driver has been reading and let hold-and-dim say how
// old it is. Switching sources mid-glance would change the number for a reason
// that has nothing to do with the car.
SnapVal snapshotPrefer(const SnapVal& first, const SnapVal& second);
