#include "Snapshot.h"

#include <string.h>

void snapshotSourceInit(SnapshotSource& src) {
  // Starts per-hour. A cold start from a standstill has no speed to divide by,
  // and L/h is the one of the two readings that is still true.
  src.flowPerDistance = false;
}

void snapshotSlotsInit(SnapshotSlots& s) {
  s.coolant = -1;
  s.volts = -1;
  s.ambient = -1;
  s.speed = -1;
  s.rpm = -1;
  for (uint8_t i = 0u; i < 4u; ++i) {
    s.tyre[i] = -1;
  }
  s.spare = -1;
}

// An all-ones raw is the module saying "no value", not a reading.
//
// Scaled, 0xFFFF/128 is a perfectly plausible 512 km/h -- exactly the kind of
// number that gets believed. This is judged on the RAW value against the
// signal's own bit width, because the sentinel is a property of the wire
// format and the scaled float has already lost it.
//
// Widths of 0 and >= 32 are refused rather than shifted: `1u << 32` is
// undefined behaviour, and a 32-bit signal has no spare code point to use as a
// sentinel anyway.
static bool rawIsSentinel(uint32_t raw, uint8_t bitLength) {
  if (bitLength == 0u || bitLength >= 32u) {
    return false;
  }
  return raw == ((1u << bitLength) - 1u);
}

SnapVal snapshotSlot(const VehicleState* state, int8_t slot, uint32_t nowMs,
                     uint32_t ttlMs, float scale) {
  SnapVal out;
  out.v = 0.0f;
  out.age = SNAP_ABSENT;

  if (state == nullptr || slot < 0) {
    return out;
  }
  const StateSlot* sl = state->slot(slot);
  if (sl == nullptr || !sl->everSet || sl->def == nullptr) {
    return out;
  }
  if (rawIsSentinel(sl->raw, sl->def->bitLength)) {
    return out;
  }

  out.v = sl->value * scale;
  out.age = state->fresh(slot, nowMs, ttlMs) ? SNAP_FRESH : SNAP_HELD;
  return out;
}

SnapVal snapshotBcast(float value, uint32_t stampMs, uint32_t nowMs, uint32_t ttlMs) {
  SnapVal out;
  out.v = 0.0f;
  out.age = SNAP_ABSENT;

  // A zeroed stamp is Broadcast.cpp reporting an explicit sentinel. It HOLDS
  // the stamp for a merely dropped frame, so this test separates "the module
  // says it has nothing" from "we missed a frame" -- dashes from a dimmed
  // number.
  if (stampMs == 0u) {
    return out;
  }

  out.v = value;
  // Wrap-safe, like every other clock comparison in core/. nowMs truncates a
  // 64-bit timer and rolls over every 49.7 days; an absolute comparison here
  // would mark everything stale for the whole TTL after the roll.
  out.age = ((nowMs - stampMs) < ttlMs) ? SNAP_FRESH : SNAP_HELD;
  return out;
}

SnapVal snapshotPrefer(const SnapVal& first, const SnapVal& second) {
  return snapShow(first) ? first : second;
}

void snapshotBuild(Snapshot& out, SnapshotSource& src, const SnapshotSlots& slots,
                   const BroadcastState& bc, const TripSet& trips,
                   const AccelSet& accel, const FaultSet& faults,
                   const VehicleState* state, bool linkUp, uint32_t nowMs) {
  out.builtMs = nowMs;
  out.linkUp = linkUp;
  out.frames = bc.frames;

  // Wholesale copies. See the header: this is what makes one rendered frame
  // internally consistent, and it is what stops the drill-down walking a fault
  // list that end-of-scan is re-sorting underneath it.
  out.trips = trips;
  out.accel = accel;
  out.faults = faults;

  // --- the five that exist on both paths ----------------------------------
  // Broadcast first, polled as the fallback. See snapshotPrefer().
  out.speedKph =
      snapshotPrefer(snapshotBcast(bc.speedKph, bc.speedMs, nowMs, VS_DEFAULT_TTL_MS),
                     snapshotSlot(state, slots.speed, nowMs, VS_DEFAULT_TTL_MS, 1.0f));
  out.rpm =
      snapshotPrefer(snapshotBcast((float)bc.rpm, bc.rpmMs, nowMs, VS_DEFAULT_TTL_MS),
                     snapshotSlot(state, slots.rpm, nowMs, VS_DEFAULT_TTL_MS, 1.0f));
  out.volts =
      snapshotPrefer(snapshotBcast(bc.volts, bc.voltsMs, nowMs, VS_DEFAULT_TTL_MS),
                     snapshotSlot(state, slots.volts, nowMs, VS_DEFAULT_TTL_MS, 1.0f));
  out.coolantC = snapshotPrefer(
      snapshotBcast((float)bc.coolantC, bc.coolantMs, nowMs, VS_DEFAULT_TTL_MS),
      snapshotSlot(state, slots.coolant, nowMs, VS_DEFAULT_TTL_MS, 1.0f));
  out.ambientC = snapshotPrefer(
      snapshotBcast(bc.ambientC, bc.ambientMs, nowMs, VS_DEFAULT_TTL_MS),
      snapshotSlot(state, slots.ambient, nowMs, VS_DEFAULT_TTL_MS, 1.0f));

  // --- broadcast only ------------------------------------------------------
  // The x2 happens HERE and not at decode: the wire value stays the wire value,
  // exactly as with psi -> bar, and only the presentation changes. No
  // snapshotPrefer -- nothing polls a heading.
  out.headingDeg = snapshotBcast((float)bc.headingRaw * 2.0f, bc.headingMs, nowMs,
                                 UI_HEADING_TTL_MS);
  out.headingOctant = bc.headingOctant;

  // --- polled only --------------------------------------------------------
  // BROADCAST now, not a poll: 0x159 b5 / 2.55, straight off the bus.
  //
  // No snapshotPrefer() and no polled fallback, because the CCN 0x2103 request
  // is gone from kPlan entirely on the owner's instruction -- there is no second
  // source to fall back TO. Deliberate rather than an omission: two readings of
  // one quantity, NEITHER of them calibrated, would only make it harder to say
  // which one a wrong number came from.
  out.fuelLitres = snapshotBcast(static_cast<float>(bc.fuelRaw) * 0.5f,
                                 bc.fuelMs, nowMs, UI_FUEL_TTL_MS);
  for (uint8_t i = 0u; i < 4u; ++i) {
    out.tyreBar[i] =
        snapshotSlot(state, slots.tyre[i], nowMs, UI_TYRE_TTL_MS, UI_PSI_TO_BAR);
  }
  out.spareBar =
      snapshotSlot(state, slots.spare, nowMs, UI_SPARE_TTL_MS, UI_PSI_TO_BAR);

  // --- the low-corner test ------------------------------------------------
  // Road wheels only; the spare has no placard pressure worth warning about
  // and a low spare is not a driving hazard. An ABSENT corner is not a
  // warning -- we do not know it is low, we know we cannot see it.
  out.tyreLowCount = 0u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    out.tyreLow[i] = snapShow(out.tyreBar[i]) && out.tyreBar[i].v < UI_TYRE_LOW_BAR;
    if (out.tyreLow[i]) {
      ++out.tyreLowCount;
    }
  }

  // --- fuel flow ----------------------------------------------------------
  // The SMOOTHED rate, not the last raw frame: Q is a per-stroke control
  // signal and is very spiky, so an unsmoothed readout at 10 Hz is a blur and
  // an unsmoothed bar is a flicker.
  const float lph = tripFlowLph(trips, nowMs);

  out.flowSegs = 0u;
  out.flowCrit = false;
  out.flow.v = 0.0f;
  out.flow.age = SNAP_ABSENT;

  if (lph >= 0.0f) {
    // Which unit FIRST, because the bar is scaled off the answer now -- it
    // used to be computed up here from the rate, against a scale the readout
    // had already left. See UI_FLOW_FULL_L100.
    //
    // Two thresholds with a gap between them, because one
    // threshold makes the readout flip units at walking pace in traffic.
    //
    // No live speed means per-hour as well, and that is not a fallback: with
    // no speed there is no distance to divide by, so L/100 would be a number
    // invented out of a stale reading.
    const bool speedLive = snapLive(out.speedKph);
    if (!speedLive || out.speedKph.v < UI_FLOW_TO_LPH_KPH) {
      src.flowPerDistance = false;
    } else if (out.speedKph.v >= UI_FLOW_TO_L100_KPH) {
      src.flowPerDistance = true;
    }
    out.flowPerDistance = src.flowPerDistance;

    if (out.flowPerDistance) {
      // Guarded: the hysteresis cannot latch per-distance below
      // UI_FLOW_TO_LPH_KPH (2 km/h), and it only ever latches ON above
      // UI_FLOW_TO_L100_KPH (3), so the divisor here is at least 2 km/h. The
      // test stays anyway, because a divide-by-zero on the hero readout is not
      // something to leave resting on a threshold two branches away -- and
      // both of those thresholds are one #define from being edited.
      const float kph = out.speedKph.v;
      if (kph > 0.0f) {
        out.flow.v = lph * 100.0f / kph;
        out.flow.age = SNAP_FRESH;
        out.flowCrit = out.flow.v > UI_FLOW_CRIT_L100;

        // THE BAR, off the same figure the number shows. Clamped at the top
        // and at the bottom: sixteen lit at 50 L/100 and never seventeen, and
        // a negative can only arrive from a negative rate, which tripFlowLph
        // uses for "nothing known" and the `lph >= 0` test above has already
        // taken out.
        const float f =
            out.flow.v * (float)UI_FLOW_SEGS / UI_FLOW_FULL_L100 + 0.5f;
        out.flowSegs = (f >= (float)UI_FLOW_SEGS)
                           ? (uint8_t)UI_FLOW_SEGS
                           : (uint8_t)((f > 0.0f) ? f : 0.0f);
      } else {
        out.flowPerDistance = false;
        src.flowPerDistance = false;
        out.flow.v = lph;
        out.flow.age = SNAP_FRESH;
      }
    } else {
      out.flow.v = lph;
      out.flow.age = SNAP_FRESH;
    }
  } else {
    out.flowPerDistance = src.flowPerDistance;
  }
}
