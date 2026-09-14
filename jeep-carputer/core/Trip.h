#pragma once
// Trip meters.
//
// Two of them, both accumulating the same four quantities:
//
//   TRIP_A      -- the driver's own meter. Survives a key cycle (see TripStore)
//                  and is only cleared by a long press.
//   TRIP_START  -- since this box powered up. Cleared on every boot, which is
//                  what makes it different from TRIP_A and the reason both
//                  exist.
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

enum TripId : uint8_t {
  TRIP_A = 0,
  TRIP_START,
  TRIP_COUNT,
};

// Below this the car counts as stopped, for "moving time" and average speed.
// Not zero: a stationary car still reports a fraction of a km/h now and then.
#define TRIP_MOVING_KPH 2.0f

// Longest gap a single fuel sample is allowed to represent. Beyond this the
// interval is DROPPED, not clamped: a dropped interval under-reads by a known,
// bounded amount, whereas a clamped one asserts a flow rate nobody measured.
//
// 2000 ms, not the 1000 ms that suits a continuous logger. This firmware is not
// one: ATMA is modal, so the loop listens for MONITOR_WINDOW_MS and then leaves
// monitor mode to run KWP requests, and 0x1A7 stops arriving for the whole poll
// phase. That gap is not lost frames -- we know exactly why we stopped
// listening and the engine plainly kept running -- so holding the last rate
// across it is the correct estimate, and the threshold has to clear a poll
// phase or every one of them would be discarded and the total would run low.
//
// Genuine dropouts are longer than this and still get dropped. If the loop is
// ever made to listen continuously, 1000 ms becomes the right number.
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
