// Host tests for core/Trip.cpp.
//
// Two things here are worth more than the rest: that distance comes from
// odometer differences and nothing else, and that fuel integration is
// independent of how often we happen to sample the bus. The second is what
// separates this from the sum-the-samples formula it replaces -- our monitor
// window is a fraction of wall-clock time, so a sum would scale with the duty
// cycle rather than with the fuel actually burnt.
#include "doctest_min.h"

#include "Broadcast.h"
#include "Trip.h"

// A bus state with the engine running: rpm and speed live, flow settable.
static void running(BroadcastState& bc, uint32_t t, uint16_t rpm, uint16_t flow) {
  bc.rpm = rpm;
  bc.rpmMs = t;
  bc.speedKph = 60.0f;
  bc.speedMs = t;
  bc.flowRaw = flow;
  bc.flowMs = t;
}

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // Nothing known yet reads as nothing, not as zero
  // -------------------------------------------------------------------------
  {
    TripSet s;
    tripSetInit(s);
    CHECK(tripKm(s.t[TRIP_A]) < 0.0f);
    CHECK(tripLitres(s.t[TRIP_A]) < 0.0f);
    CHECK(tripL100(s.t[TRIP_A]) < 0.0f);
    CHECK(tripAvgKph(s.t[TRIP_A]) < 0.0f);
  }

  // -------------------------------------------------------------------------
  // Distance is odometer differences. 100 m per count.
  // -------------------------------------------------------------------------
  {
    TripSet s;
    tripSetInit(s);
    BroadcastState bc;
    broadcastClear(bc);

    bc.odometerDm = 2462785u;   // 246278.5 km, the owner's real reading
    bc.odometerMs = 1000u;
    tripUpdate(s, bc, 1000u);
    // First reading anchors; a trip is zero long, not a quarter million km.
    CHECK_NEAR(tripKm(s.t[TRIP_A]), 0.0f, 0.001f);

    bc.odometerDm = 2462885u;   // +100 counts = +10.0 km
    bc.odometerMs = 2000u;
    tripUpdate(s, bc, 2000u);
    CHECK_NEAR(tripKm(s.t[TRIP_A]), 10.0f, 0.001f);
    CHECK_NEAR(tripKm(s.t[TRIP_START]), 10.0f, 0.001f);

    // A repeated odometer frame at a standstill must not re-anchor.
    bc.odometerMs = 3000u;
    tripUpdate(s, bc, 3000u);
    CHECK_NEAR(tripKm(s.t[TRIP_A]), 10.0f, 0.001f);
  }

  // -------------------------------------------------------------------------
  // Fuel: the same drive sampled at two different rates must give the same
  // litres. This is the property a plain sum of samples does NOT have.
  // -------------------------------------------------------------------------
  {
    // 60 seconds at a constant flow of Q=500 (3.125 L/h), sampled first every
    // 100 ms and then every 500 ms.
    float litres[2];
    const uint32_t kStep[2] = {100u, 500u};
    for (uint8_t k = 0u; k < 2u; ++k) {
      TripSet s;
      tripSetInit(s);
      BroadcastState bc;
      broadcastClear(bc);
      for (uint32_t t = kStep[k]; t <= 60000u; t += kStep[k]) {
        running(bc, t, 1000u, 500u);
        tripUpdate(s, bc, t);
      }
      litres[k] = tripLitres(s.t[TRIP_A]);
    }
    CHECK(litres[0] > 0.0f);
    // Within one sample's worth: the only difference is the leading interval
    // that has no predecessor to integrate against.
    CHECK_NEAR(litres[0], litres[1], litres[0] * 0.02f);

    // And the magnitude is the rate times the time, with no free parameters:
    //   Q/160 L/h = 3.125 L/h, for 60 s = 0.0521 L
    //   == Q x ms / 5.76e8 = 500 x 60000 / 5.76e8
    CHECK_NEAR(litres[0], 0.05208f, 0.0015f);
  }

  // -------------------------------------------------------------------------
  // Q == 0 is a real reading and burns exactly nothing
  //
  // Overrun fuel cut-off: a trailing engine on a closed throttle. This must not
  // be mistaken for "no data" and hold the previous cruising rate, which is how
  // a long descent would otherwise invent fuel.
  // -------------------------------------------------------------------------
  {
    TripSet s;
    tripSetInit(s);
    BroadcastState bc;
    broadcastClear(bc);
    // Ten seconds of real flow, then twenty of overrun.
    for (uint32_t t = 100u; t <= 10000u; t += 100u) {
      running(bc, t, 2000u, 1600u);
      tripUpdate(s, bc, t);
    }
    const float afterCruise = tripLitres(s.t[TRIP_A]);
    for (uint32_t t = 10100u; t <= 30000u; t += 100u) {
      running(bc, t, 2000u, 0u);
      tripUpdate(s, bc, t);
    }
    CHECK_NEAR(tripLitres(s.t[TRIP_A]), afterCruise, 1e-6f);
  }

  // -------------------------------------------------------------------------
  // 0xFFFF means "no value" and must never reach the integrator
  // -------------------------------------------------------------------------
  {
    BroadcastState bc;
    broadcastClear(bc);
    CanFrame f;
    f.id = 0x1A7u;
    f.len = 6u;
    for (uint8_t i = 0u; i < 8u; ++i) {
      f.data[i] = 0u;
    }
    f.data[0] = 0xFFu;
    f.data[1] = 0xFFu;
    CHECK(!broadcastApply(bc, f, 1000u));
    CHECK(bc.flowMs == 0u);      // the stamp must not move, so the interval drops

    // A real reading of zero, by contrast, is accepted.
    f.data[0] = 0u;
    f.data[1] = 0u;
    CHECK(broadcastApply(bc, f, 2000u));
    CHECK(bc.flowMs == 2000u);
    CHECK(bc.flowRaw == 0u);
  }

  // -------------------------------------------------------------------------
  // A blackout longer than the cap does not invent fuel
  //
  // The interval is DROPPED, not clamped: nothing at all is integrated across
  // it, because holding a cruising rate over a gap of unknown content would be
  // asserting a measurement nobody made.
  // -------------------------------------------------------------------------
  {
    TripSet s;
    tripSetInit(s);
    BroadcastState bc;
    broadcastClear(bc);
    running(bc, 100u, 2000u, 1000u);
    tripUpdate(s, bc, 100u);
    // 10 minutes of silence, then one more sample.
    running(bc, 600100u, 2000u, 1000u);
    tripUpdate(s, bc, 600100u);
    CHECK(tripLitres(s.t[TRIP_A]) < 0.0f);
  }

  // -------------------------------------------------------------------------
  // The smoothed instantaneous readout converges on the true rate, and goes
  // away entirely when 0x1A7 stops
  // -------------------------------------------------------------------------
  {
    TripSet s;
    tripSetInit(s);
    BroadcastState bc;
    broadcastClear(bc);
    CHECK(tripFlowLph(s, 0u) < 0.0f);          // nothing heard yet

    // Q = 1600 is 10.0 L/h. Well past the time constant it should be there.
    for (uint32_t t = 100u; t <= 20000u; t += 100u) {
      running(bc, t, 2000u, 1600u);
      tripUpdate(s, bc, t);
    }
    CHECK_NEAR(tripFlowLph(s, 20000u), 10.0f, 0.1f);

    // And it is a reading with an age: long after the last frame, nothing.
    CHECK(tripFlowLph(s, 20000u + TRIP_MAX_GAP_MS + 2000u) < 0.0f);
  }

  // -------------------------------------------------------------------------
  // Reset clears one trip and re-anchors it where the car is now
  // -------------------------------------------------------------------------
  {
    TripSet s;
    tripSetInit(s);
    BroadcastState bc;
    broadcastClear(bc);

    bc.odometerDm = 1000u;
    bc.odometerMs = 1000u;
    tripUpdate(s, bc, 1000u);
    bc.odometerDm = 1050u;      // +5.0 km
    bc.odometerMs = 2000u;
    tripUpdate(s, bc, 2000u);
    CHECK_NEAR(tripKm(s.t[TRIP_A]), 5.0f, 0.001f);

    tripReset(s, TRIP_A);
    CHECK_NEAR(tripKm(s.t[TRIP_A]), 0.0f, 0.001f);
    // The other meter is untouched -- that is the point of having two.
    CHECK_NEAR(tripKm(s.t[TRIP_START]), 5.0f, 0.001f);

    bc.odometerDm = 1070u;      // a further +2.0 km
    bc.odometerMs = 3000u;
    tripUpdate(s, bc, 3000u);
    CHECK_NEAR(tripKm(s.t[TRIP_A]), 2.0f, 0.001f);
    CHECK_NEAR(tripKm(s.t[TRIP_START]), 7.0f, 0.001f);
  }

  // -------------------------------------------------------------------------
  // Average speed uses MOVING time, so traffic lights do not drag it down
  // -------------------------------------------------------------------------
  {
    TripSet s;
    tripSetInit(s);
    BroadcastState bc;
    broadcastClear(bc);

    bc.odometerDm = 100u;
    bc.odometerMs = 0u + 1u;
    tripUpdate(s, bc, 1u);

    // 60 s moving at 60 km/h, then 60 s stopped -- ENGINE RUNNING throughout.
    // Idling at a standstill still counts as elapsed time; it is the engine
    // being OFF that stops the clock, not the car being stationary.
    for (uint32_t t = 100u; t <= 60000u; t += 100u) {
      bc.speedKph = 60.0f;
      bc.speedMs = t;
      bc.rpm = 1800u;
      bc.rpmMs = t;
      tripUpdate(s, bc, t);
    }
    bc.odometerDm = 110u;       // 1.0 km covered
    bc.odometerMs = 60000u;
    tripUpdate(s, bc, 60000u);
    for (uint32_t t = 60100u; t <= 120000u; t += 100u) {
      bc.speedKph = 0.0f;
      bc.speedMs = t;
      bc.rpm = 750u;          // idling at the lights
      bc.rpmMs = t;
      tripUpdate(s, bc, t);
    }

    CHECK_NEAR(tripKm(s.t[TRIP_A]), 1.0f, 0.001f);
    // 1 km in ~60 s of movement = 60 km/h, not the 30 that total time gives.
    CHECK_NEAR(tripAvgKph(s.t[TRIP_A]), 60.0f, 1.0f);
    CHECK(s.t[TRIP_A].elapsedMs > s.t[TRIP_A].movingMs);
  }

  // -------------------------------------------------------------------------
  // A trip does not run while the ENGINE does not
  //
  // Ignition on, engine off: the bus is alive and broadcasting, so without this
  // a trip racks up time for as long as anyone sits there with the radio on.
  // And a car rolled or towed with the engine off must not add distance either
  // -- there is no fuel behind those kilometres.
  // -------------------------------------------------------------------------
  {
    TripSet s;
    tripSetInit(s);
    BroadcastState bc;

    // Drive 1 km with the engine running.
    bc.rpm = 1500u; bc.rpmMs = 1u;
    bc.speedKph = 60.0f; bc.speedMs = 1u;
    bc.odometerDm = 1000u; bc.odometerMs = 1u;
    tripUpdate(s, bc, 1u);
    for (uint32_t t = 100u; t <= 10000u; t += 100u) {
      bc.rpmMs = t; bc.speedMs = t;
      tripUpdate(s, bc, t);
    }
    bc.odometerDm = 1010u; bc.odometerMs = 10000u;
    tripUpdate(s, bc, 10000u);
    const float km = tripKm(s.t[TRIP_A]);
    const uint32_t elapsed = s.t[TRIP_A].elapsedMs;
    CHECK_NEAR(km, 1.0f, 0.001f);
    CHECK(elapsed > 9000u);

    // Now switch the engine off and sit there for a minute, ROLLING 2 km.
    for (uint32_t t = 10100u; t <= 70000u; t += 100u) {
      bc.rpm = 0u; bc.rpmMs = t;
      bc.speedKph = 0.0f; bc.speedMs = t;
      tripUpdate(s, bc, t);
    }
    bc.odometerDm = 1030u; bc.odometerMs = 70000u;
    tripUpdate(s, bc, 70000u);

    CHECK_NEAR(tripKm(s.t[TRIP_A]), km, 0.001f);        // no distance
    CHECK(s.t[TRIP_A].elapsedMs == elapsed);            // and no time
  }

  // -------------------------------------------------------------------------
  // A STALE rpm is not the same as an engine that is off
  //
  // A link dropout while driving must pause the clock but must never delete the
  // distance covered during it -- the anchor stays put, so the kilometres
  // appear as soon as the odometer is heard again.
  // -------------------------------------------------------------------------
  {
    TripSet s;
    tripSetInit(s);
    BroadcastState bc;
    bc.rpm = 1500u; bc.rpmMs = 1u;
    bc.odometerDm = 2000u; bc.odometerMs = 1u;
    tripUpdate(s, bc, 1u);

    // Ten seconds with nothing heard at all, then the odometer reappears
    // 1 km further on. rpmMs is long stale by now.
    bc.odometerDm = 2010u; bc.odometerMs = 20000u;
    tripUpdate(s, bc, 20000u);
    CHECK_NEAR(tripKm(s.t[TRIP_A]), 1.0f, 0.001f);      // distance survived
  }

  // -------------------------------------------------------------------------
  // L/100 km is withheld until the odometer has something to divide by
  // -------------------------------------------------------------------------
  {
    TripSet s;
    tripSetInit(s);
    BroadcastState bc;
    broadcastClear(bc);
    bc.odometerDm = 500u;
    bc.odometerMs = 1u;
    tripUpdate(s, bc, 1u);
    for (uint32_t t = 100u; t <= 5000u; t += 100u) {
      running(bc, t, 2000u, 800u);
      tripUpdate(s, bc, t);
    }
    CHECK(tripLitres(s.t[TRIP_A]) > 0.0f);
    CHECK(tripL100(s.t[TRIP_A]) < 0.0f);   // still parked on the same 100 m count
  }

TEST_MAIN_END()
