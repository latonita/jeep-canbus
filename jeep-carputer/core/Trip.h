#pragma once
// Trip meters.
//
// Three of them, all accumulating the same four quantities:
//
//   TRIP_A      -- the driver's own meter. Survives a key cycle (see TripStore)
//                  and is only cleared by a long press.
//   TRIP_B      -- a second one, identical in every respect. Two exist so a
//                  long-running total (a tank, a month) can be kept while a
//                  short one (a leg, an errand) is reset freely. One meter
//                  cannot do both: whichever question you ask it, resetting
//                  destroys the answer to the other.
//   TRIP_START  -- since this box powered up. Cleared on every boot, which is
//                  what makes it different from the other two and the reason
//                  all three exist.
//
// DISTANCE comes from the odometer broadcast (0x014), never from integrating
// speed. The odometer is 100 m per count and is the car's own number, so a trip
// built from its differences cannot drift, cannot double-count a stop, and
// agrees with the dash to the last digit. Integrating speed would accumulate
// error for the whole life of the trip.
//
// FUEL comes from 0x1A7, and is the one number here that carries an unpinned
// constant. See tripLitres().
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

// ORDER IS THE UI's CYCLE ORDER. The TRIP page steps through these on a double
// tap, so TRIP_START sits last: the two resettable meters are adjacent, and the
// one that cannot be reset at all is not between them.
enum TripId : uint8_t {
  TRIP_A = 0,
  TRIP_B,
  TRIP_START,
  TRIP_COUNT,
};

// Below this the car counts as stopped, for "moving time" and average speed.
// Not zero: a stationary car still reports a fraction of a km/h now and then.
#define TRIP_MOVING_KPH 2.0f

// Below this the engine is not running, and a trip does not accumulate at all.
//
// The case this exists for is the ignition ON with the engine OFF -- accessory,
// or sitting with the radio on. The bus is alive and broadcasting, so without
// this a trip would quietly rack up elapsed time for as long as anyone sat
// there, and FROM START would report a journey mostly made of not driving.
//
// It stops DISTANCE too, not just the clock: the odometer anchor is carried
// forward across the pause, so a car rolled or towed with the engine off adds
// nothing -- there is no fuel to attribute to it, and counting the distance
// alone would drag L/100 down by an amount nobody could explain.
//
// But only when rpm is LIVE and below this. A stale rpm means we do not know,
// and an unknown engine must not cost the trip its distance -- see tripUpdate.
#define TRIP_RUNNING_RPM 300u

// Longest gap a single fuel sample is allowed to represent. Beyond this the
// interval is DROPPED, not clamped: a dropped interval under-reads by a known,
// bounded amount, whereas a clamped one asserts a flow rate nobody measured.
//
// 2000 ms, AND IT IS SIZED FOR OUR OWN CLOCK RATHER THAN FOR THE BUS. That
// distinction is the whole of this comment, and an earlier version of it had
// the reason wrong: it argued the slack was needed because ATMA is modal -- the
// loop listens for a window, then goes deaf to run KWP requests, and 0x1A7 stops
// arriving for the whole poll phase -- and concluded that "if the loop is ever
// made to listen continuously, 1000 ms becomes the right number".
//
// The loop IS continuous now, on any 2.1 adapter, and 1000 ms is not the
// interesting number. Measured over a 1449.6 s drive, 14478 0x1A7 frames timed
// by the gateway's own clock:
//
//   median 100 ms, p99 108 ms, WORST 206 ms. Eighteen intervals above 150 ms,
//   NONE above 300 ms. Eighteen single frames lost in twenty-four minutes.
//
// So on the ADAPTER's clock the honest threshold is about 300 ms -- thirty per
// cent of headroom over the worst interval the bus actually produced -- and
// every one of 2000, 1000, 500 and 300 integrates the identical 1.1055 L.
// Tightening to 150 ms would start discarding real fuel (0.18 %).
//
// WHAT KEEPS IT AT 2000 IS THE FALLBACK PATHS, not the bus. This threshold is a
// compile-time constant in pure code that cannot know which clock the stamps
// came in on, and there are three possibilities:
//
//   AT@SML2  broadcast stamps are the GATEWAY's, mapped by core/StreamClock.
//            300 ms would be right.
//   AT@SM1   host receive times. A reader stalled 100 ms behind an NVS commit
//            inflates the worst 0x1A7 interval from 206 ms to 299 ms and, at
//            300 ms, a 200 ms stall starts dropping intervals -- up to 9.5 % of
//            the fuel discarded in the measured worst case.
//   ATMA     the modal windows the old comment described, which really do need
//            seconds of slack.
//
// 2000 ms is therefore the value that is SAFE ON ALL THREE, and the cost of that
// safety is precisely bounded: on the LAWICEL path it drops nothing that 300 ms
// would keep, and it merely fails to notice a dropout between 300 ms and 2 s.
// Tightening it means making it a runtime value that follows the link mode, and
// that is a deliberate change to a persisted-totals path rather than a constant
// to edit -- see the note in CLAUDE.md.
//
// Genuine dropouts are longer than this and still get dropped.
#define TRIP_MAX_GAP_MS 2000u

// One trip's running totals. Plain data on purpose: TripStore writes this
// struct to NVS as a blob.
struct Trip {
  uint32_t odoStartDm;   // odometer at the reset, in 100 m counts. 0 = unanchored
  uint32_t odoNowDm;     // most recent odometer reading
  uint64_t fuelAccum;    // sum of raw x rpm x ms; see tripLitres()
  uint32_t movingMs;
  uint32_t elapsedMs;
};

// Everything the accumulator needs to remember between samples. Kept out of
// Trip so that what gets persisted is only the totals, not the plumbing.
struct TripSet {
  Trip     t[TRIP_COUNT];
  uint32_t lastTickMs;
  uint32_t lastFlowStampMs;   // the 0x1A7 stamp we last integrated
  uint32_t lastOdoStampMs;

  // Smoothed instantaneous flow, for the DRIVE readout. Lives here rather than
  // in the presenter because it has to be fed on every frame: the UI refreshes
  // at 10 Hz but 0x1A7 only arrives inside the monitor window, so a presenter
  // sampling it on its own clock would smooth an aliased signal.
  float    flowLph;
  uint32_t flowLphMs;
};

void tripSetInit(TripSet& s);

// Wipe one trip back to zero and re-anchor it to the current odometer.
void tripReset(TripSet& s, uint8_t which);

struct BroadcastState;

// Fold whatever is new in `bc` into both trips. Safe to call as often as you
// like -- it works off the broadcast timestamps, so calling it twice with
// nothing new in between accumulates nothing twice.
void tripUpdate(TripSet& s, const BroadcastState& bc, uint32_t nowMs);

// --- readouts. All return a negative value when there is nothing to show yet,
// which the presenter renders as dashes rather than a confident zero. ---

float tripKm(const Trip& t);
float tripLitres(const Trip& t);
float tripL100(const Trip& t);
float tripAvgKph(const Trip& t);

// What the fuel this trip burned cost, at `pricePerLitre`.
//
// Negative whenever tripLitres() is, and for the same reason: a cost of 0.00
// on a trip that has not integrated any fuel yet is a confident lie, where a
// dash is not. A non-positive price is also treated as "cannot say" -- there
// is no sensible cost to quote when nobody has told the instrument what fuel
// costs.
//
// The only input here that is not measured is the price, which the driver
// types in. That makes this the one readout on the instrument whose accuracy
// is the driver's responsibility rather than the bus's.
float tripCost(const Trip& t, float pricePerLitre);

// How far the fuel in the tank goes at THIS meter's average, in km. Negative
// when it cannot be said, exactly like tripL100() and tripCost().
//
// Same shape as tripCost(): a trip plus one scalar the trip cannot know. The
// litres come from 0x159 on the bus and the consumption from whichever meter
// the driver is looking at, so the answer changes when they switch tabs -- and
// that is the point. TRIP A carrying a tankful of motorway and TRIP B a week of
// town SHOULD project differently.
//
// THE MINIMUM DISTANCE IS ONE KILOMETRE, ten times tripL100()'s own floor, and
// the difference is deliberate. tripL100() only has to survive the odometer's
// 100 m quantisation to state a fact about distance already covered. This
// PROJECTS that fact over a whole tankful -- roughly 700 km of it -- so an
// average taken over 100 m is multiplied by seven thousand. A meter reset at
// the bottom of a hill would read a few hundred millilitres per 100 km and
// promise several thousand kilometres, which is arithmetically exact and a lie.
//
// There is deliberately NO upper clamp. Past this floor the number is the
// meter's own honest projection, and capping it would make a wrong average look
// reasonable rather than obviously wrong.
#define TRIP_RANGE_MIN_KM 1.0f
float tripRangeKm(const Trip& t, float litresInTank);

// Smoothed instantaneous flow in L/h, or negative when 0x1A7 has gone quiet --
// which is what an engine that is not running looks like.
float tripFlowLph(const TripSet& s, uint32_t nowMs);

// Fuel.
//
// 0x1A7 carries INSTANTANEOUS FLOW, directly. DLC 6, every 100 ms, bytes 2..5
// always zero:
//
//   Q = (b0 << 8) | b1          unsigned, big-endian
//   L/h = Q / 160.0
//
// So litres is just the time integral of that rate, and nothing else is needed:
//
//   litres = SUM( Q x dt_ms ) / (160 x 3600 x 1000)
//          = SUM( Q x dt_ms ) / TRIP_FUEL_DIVISOR
//
// AN EARLIER VERSION OF THIS FILE WAS WRONG. It read 0x1A7 as injection
// quantity per stroke -- a vendor-database cross-reference for a PCM PID, not
// something confirmed against this broadcast -- and therefore multiplied by rpm
// and carried a measured 3.00x correction on top to make a real drive come out
// right. Both are gone. Multiplying an already-rate signal by rpm implies a
// peak near 390 L/h, which is the tell. A signal that needs a 3x fudge to agree
// with the dashboard was being modelled wrongly, not measured wrongly.
#define TRIP_FUEL_DIVISOR 5.76e8

// Q == 0xFFFF is the module saying it has no value. It is NOT 409 L/h, and it
// is not zero either -- the interval it spans must be dropped rather than
// integrated at any rate. Rejected in Broadcast.cpp, so the stamp never moves
// and this file only ever sees real readings.
#define TRIP_FLOW_UNAVAILABLE 0xFFFFu

// Q == 0 is a REAL reading: overrun fuel cut-off, a closed throttle on a
// trailing engine, which genuinely burns nothing. It must be integrated as zero
// rather than skipped, or every downhill stretch would hold the last cruising
// rate and invent fuel.

// Time constant for the instantaneous readout on DRIVE. Raw Q is very spiky --
// it is a per-stroke control signal, not something meant to be looked at -- and
// an unsmoothed number at 10 Hz is unreadable from a moving vehicle. This is
// display smoothing ONLY; the integral above is fed raw samples.
#define TRIP_FLOW_TAU_MS 3000u

// Still worth confirming against a pump. Q/160 is a documented scaling, not one
// derived here, but nothing in this firmware has been checked against fuel
// actually paid for. A fill-to-fill over a few hundred km would settle it.
//
// IT IS NO LONGER UNCHECKED AGAINST A DRIVE, THOUGH, and the result is
// plausible enough to be worth recording: replaying a 1449.6 s capture
// (14478 0x1A7 frames, timed by the gateway's own clock) through this arithmetic
// gives 1.1055 L over a 10.600 km odometer delta -- 10.43 L/100 km, for a
// 3.0 CRD doing 26 km/h average up and down a steep hill. Peak flow 13.1 L/h,
// 1.2 % of samples at exactly zero (overrun cut-off on the descent). A wrong
// model would not land there by accident.
//
// AND THE ERROR BUDGET IS NOW KNOWN, which matters more than the figure:
//
//   integration scheme      0.003 %  (forward / backward / trapezoid, all three)
//   host-clock jitter       0.03 %   (worst of four reader-stall models)
//   ODOMETER QUANTISATION   0.94 %   (+/- one 100 m tick over 10.6 km)
//
// The distance term dominates by a factor of thirty. So no refinement of the
// time integral -- not the timestamps, not the rectangle rule -- can improve
// the L/100 km figure on a short trip: only a longer trip can, and on a tankful
// the odometer term falls away and Q/160 itself becomes the thing to check.
