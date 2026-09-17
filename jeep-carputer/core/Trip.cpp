#include "Trip.h"

#include "Broadcast.h"

void tripSetInit(TripSet& s) {
  for (uint8_t i = 0u; i < TRIP_COUNT; ++i) {
    s.t[i].odoStartDm = 0u;
    s.t[i].odoNowDm = 0u;
    s.t[i].fuelAccum = 0u;
    s.t[i].movingMs = 0u;
    s.t[i].elapsedMs = 0u;
  }
  s.lastTickMs = 0u;
  s.lastFlowStampMs = 0u;
  s.lastOdoStampMs = 0u;
  s.flowLph = 0.0f;
  s.flowLphMs = 0u;
}

void tripReset(TripSet& s, uint8_t which) {
  if (which >= TRIP_COUNT) {
    return;
  }
  Trip& t = s.t[which];
  // Re-anchor to wherever the odometer is NOW rather than to zero. Anchoring to
  // zero and waiting for the next 0x014 would make the trip read the whole
  // odometer -- a quarter of a million km -- for the second or so in between.
  const uint32_t odo = t.odoNowDm;
  t.odoStartDm = odo;
  t.fuelAccum = 0u;
  t.movingMs = 0u;
  t.elapsedMs = 0u;
}

void tripUpdate(TripSet& s, const BroadcastState& bc, uint32_t nowMs) {
  // --- distance: odometer differences, not integrated speed ---
  //
  // Only act on a reading we have not already taken. The stamp moves when a
  // 0x014 frame lands, so this is "is there a new odometer reading", not "has
  // the value changed" -- a car standing still reports the same value over and
  // over and must not re-anchor a trip on each one.
  // Is the engine actually running? A stale rpm is treated as NOT running, the
  // same way a stale speed is treated as stopped below: if the bus has gone
  // quiet we do not know the engine is turning, and a trip that keeps counting
  // on a silent bus is the failure nobody notices until the numbers are wrong.
  // THREE states, not two, and the third is why this is not one boolean.
  //
  //   running    -- rpm is live and above idle-crank. Everything accumulates.
  //   engineOff  -- rpm is live and BELOW it. We positively know the engine is
  //                 stopped, so the clock pauses and the odometer anchor is
  //                 carried forward, costing the trip the distance too.
  //   neither    -- rpm is STALE. We know nothing. The clock pauses, but the
  //                 anchor is LEFT ALONE.
  //
  // That last distinction matters more than it looks. Treating a stale rpm as
  // engine-off would mean a link dropout while driving silently deletes the
  // kilometres covered during it -- the anchor would walk forward and the
  // distance would simply never appear. Pausing the clock is recoverable and
  // visible; destroying distance is neither.
  const bool rpmLive = (bc.rpmMs != 0u) && ((nowMs - bc.rpmMs) < 5000u);
  const bool running = rpmLive && (bc.rpm >= TRIP_RUNNING_RPM);
  const bool engineOff = rpmLive && (bc.rpm < TRIP_RUNNING_RPM);

  if (bc.odometerMs != 0u && bc.odometerMs != s.lastOdoStampMs) {
    s.lastOdoStampMs = bc.odometerMs;
    for (uint8_t i = 0u; i < TRIP_COUNT; ++i) {
      Trip& t = s.t[i];
      // Engine off: carry the ANCHOR forward by the same amount the odometer
      // moved, so the distance lands nowhere. A car rolled, towed or pushed
      // with the engine off has no fuel behind its kilometres, and counting
      // them would drag L/100 down by an amount nobody could account for.
      if (engineOff && t.odoStartDm != 0u && t.odoNowDm != 0u &&
          bc.odometerDm > t.odoNowDm) {
        t.odoStartDm += (bc.odometerDm - t.odoNowDm);
      }
      t.odoNowDm = bc.odometerDm;
      if (t.odoStartDm == 0u || t.odoStartDm > bc.odometerDm) {
        // Unanchored, or the odometer went backwards (a restored trip from a
        // different car, or a corrupt read). Re-anchor rather than show a
        // nonsense distance.
        t.odoStartDm = bc.odometerDm;
      }
    }
  }

  // --- time ---
  if (s.lastTickMs == 0u) {
    s.lastTickMs = nowMs;
  }
  uint32_t dt = nowMs - s.lastTickMs;
  if (dt > TRIP_MAX_GAP_MS) {
    dt = TRIP_MAX_GAP_MS;
  }
  s.lastTickMs = nowMs;

  // NOTHING accumulates while the engine is not turning. The case this exists
  // for is the ignition on with the engine off -- accessory, or sitting with
  // the radio playing. The bus is alive and broadcasting, so without this a
  // trip racks up elapsed time for as long as anyone sits there, and FROM
  // START reports a journey mostly made of not driving.
  //
  // After lastTickMs is updated, so the pause does not arrive as one huge dt
  // the moment the engine starts again.
  if (!running) {
    return;
  }

  // Moving vs stopped comes from the last live speed reading. A stale one is
  // treated as stopped: if the bus has gone quiet we do not know the car is
  // moving, and inventing moving time would flatter the average speed.
  const bool speedLive = (bc.speedMs != 0u) && ((nowMs - bc.speedMs) < 5000u);
  const bool moving = speedLive && (bc.speedKph >= TRIP_MOVING_KPH);

  for (uint8_t i = 0u; i < TRIP_COUNT; ++i) {
    s.t[i].elapsedMs += dt;
    if (moving) {
      s.t[i].movingMs += dt;
    }
  }

  // --- fuel ---
  //
  // Integrate only on a NEW 0x1A7 sample, and integrate over the time since the
  // previous one -- not since the last tick. That is what makes the total
  // independent of our monitor duty cycle: we are measuring the area under the
  // flow curve, so what matters is the width of each interval, not how many
  // samples happened to land in it. A plain sum of samples would scale with
  // however many frames we caught, and 21-30% of 0x1A7 frames drop, by a
  // fraction that varies from session to session.
  if (bc.flowMs != 0u && bc.flowMs != s.lastFlowStampMs) {
    const uint32_t prev = s.lastFlowStampMs;
    s.lastFlowStampMs = bc.flowMs;
    if (prev != 0u && bc.flowMs > prev) {
      const uint32_t fdt = bc.flowMs - prev;
      if (fdt <= TRIP_MAX_GAP_MS) {
        // Q is already a rate, so this is rate x time and nothing else. Q == 0
        // needs no special case: it is a real reading of "burning nothing" and
        // contributes exactly zero through the same multiply.
        const uint64_t add = (uint64_t)bc.flowRaw * (uint64_t)fdt;
        for (uint8_t i = 0u; i < TRIP_COUNT; ++i) {
          s.t[i].fuelAccum += add;
        }

        // Display smoothing, over the same interval. Weighting by fdt rather
        // than treating every sample equally keeps the time constant honest
        // when frames drop: ten samples 100 ms apart and two 500 ms apart
        // should decay the average by the same amount.
        float a = (float)fdt / (float)TRIP_FLOW_TAU_MS;
        if (a > 1.0f) {
          a = 1.0f;
        }
        s.flowLph += ((float)bc.flowRaw / 160.0f - s.flowLph) * a;
        s.flowLphMs = bc.flowMs;
      }
    }
  }
}

float tripKm(const Trip& t) {
  if (t.odoStartDm == 0u || t.odoNowDm < t.odoStartDm) {
    return -1.0f;
  }
  return (float)(t.odoNowDm - t.odoStartDm) * 0.1f;   // 100 m per count
}

float tripLitres(const Trip& t) {
  if (t.fuelAccum == 0u) {
    return -1.0f;
  }
  return (float)((double)t.fuelAccum / TRIP_FUEL_DIVISOR);
}

float tripL100(const Trip& t) {
  const float km = tripKm(t);
  const float l = tripLitres(t);
  // Below 100 m the quotient is dominated by the odometer's own quantisation:
  // one 100 m count either way swings the figure by a factor of two. Show
  // nothing rather than a number that swings like that.
  if (km < 0.1f || l < 0.0f) {
    return -1.0f;
  }
  return l * 100.0f / km;
}

float tripFlowLph(const TripSet& s, uint32_t nowMs) {
  // Nothing heard recently means the engine is not running -- 0x1A7 stops
  // entirely rather than settling at zero -- so this is "no reading", not
  // "0.0 L/h". Generous against TRIP_MAX_GAP_MS so the poll phase, during which
  // no frame can possibly arrive, does not blank the readout every loop.
  if (s.flowLphMs == 0u || (nowMs - s.flowLphMs) > (TRIP_MAX_GAP_MS + 1000u)) {
    return -1.0f;
  }
  return s.flowLph;
}

float tripAvgKph(const Trip& t) {
  const float km = tripKm(t);
  if (km < 0.0f || t.movingMs < 1000u) {
    return -1.0f;
  }
  return km * 3600000.0f / (float)t.movingMs;
}

float tripRangeKm(const Trip& t, float litresInTank) {
  if (litresInTank < 0.0f) {
    return -1.0f;          // no tank reading; the caller passes -1 for absent
  }
  if (tripKm(t) < TRIP_RANGE_MIN_KM) {
    return -1.0f;          // too little data to project a tankful from
  }
  const float l100 = tripL100(t);
  if (l100 <= 0.0f) {
    return -1.0f;          // already unavailable, or a meter that burnt nothing
  }
  return litresInTank * 100.0f / l100;
}

float tripCost(const Trip& t, float pricePerLitre) {
  if (pricePerLitre <= 0.0f) {
    return -1.0f;
  }
  const float litres = tripLitres(t);
  if (litres < 0.0f) {
    return -1.0f;
  }
  return litres * pricePerLitre;
}
