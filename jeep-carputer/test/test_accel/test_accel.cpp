// Host tests for core/Accel.cpp.
//
// The ones that matter most: that a run can only be armed by having SEEN the
// car stopped AND idling -- joining a motorway at speed must never produce a
// figure -- that arming survives a brake-torque launch, and that the split is
// interpolated rather than taken from the sample that first reads over the
// line. At 50 ms between samples a hard launch gains several km/h per sample,
// so snapping to the sample flatters exactly the runs people care about.
#include "doctest_min.h"

#include <string.h>

#include "Accel.h"
#include "Broadcast.h"

#define IDLE_RPM   700u
#define DRIVE_RPM 2000u

// Feed a constant-acceleration ramp, one sample every `stepMs`, starting at
// `t0` with speed `v0` and gaining `accelKphPerS`. Returns the next timestamp.
//
// `floored` defaults to TRUE: a ramp away from a standstill means a LAUNCH
// here, and since ACCEL_LAUNCH_MS a launch is a pedal through the kickdown
// detent, not merely motion. Pass false to simulate pulling gently out of a
// junction -- which is what must not be timed.
static uint32_t ramp(AccelSet& a, uint32_t t0, uint32_t stepMs, float v0,
                     float accelKphPerS, uint32_t forMs, bool floored = true) {
  uint32_t t = t0;
  const uint32_t end = t0 + forMs;
  while (t <= end) {
    const float v = v0 + accelKphPerS * (float)(t - t0) / 1000.0f;
    accelUpdate(a, v, DRIVE_RPM, t);
    if (floored) {
      accelPedal(a, true);
    }
    t += stepMs;
  }
  return t;
}

// Hold a steady speed for a while, so a run can run out of road.
static uint32_t hold(AccelSet& a, uint32_t t0, uint32_t stepMs, float v,
                     uint32_t forMs, bool floored = true) {
  uint32_t t = t0;
  const uint32_t end = t0 + forMs;
  while (t <= end) {
    accelUpdate(a, v, DRIVE_RPM, t);
    if (floored) {
      accelPedal(a, true);
    }
    t += stepMs;
  }
  return t;
}

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // Moving without ever having been seen stopped is not a run
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    ramp(a, 1000u, 50u, 90.0f, 40.0f, 2000u);   // join at 90, push past 100
    CHECK(!accelRunning(a));
    CHECK(accelLast(a) == nullptr);
    CHECK(a.best.ms[ACCEL_SPLIT_100] == 0u);
  }

  // -------------------------------------------------------------------------
  // Stopped, but the engine is not running: nothing to arm
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, 0u, 1000u);            // ignition on, engine off
    CHECK(a.state != ACCEL_ARMED);
    ramp(a, 1050u, 50u, 1.0f, 20.0f, 6000u);
    CHECK(accelLast(a) == nullptr);
  }

  // -------------------------------------------------------------------------
  // Stopped but already wound up: not a standing start
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, 2500u, 1000u);         // rolled to a stop revving
    CHECK(a.state != ACCEL_ARMED);
    ramp(a, 1050u, 50u, 1.0f, 20.0f, 6000u);
    CHECK(accelLast(a) == nullptr);
  }

  // -------------------------------------------------------------------------
  // A BRAKE-TORQUE launch still counts: rpm is checked when arming, not after
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);      // arrive at the line idling
    CHECK(a.state == ACCEL_ARMED);
    accelUpdate(a, 0.0f, 2200u, 1500u);         // wind it up against the brake
    CHECK(a.state == ACCEL_ARMED);              // still armed
    ramp(a, 1550u, 50u, 1.0f, 20.0f, 6000u);
    CHECK(accelLast(a) != nullptr);
    CHECK(accelLast(a)->ms[ACCEL_SPLIT_60] != 0u);
  }

  // -------------------------------------------------------------------------
  // A clean run records both splits, and 60-100 falls out of them
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    ramp(a, 1050u, 50u, 1.0f, 20.0f, 6000u);    // 60 at 3 s, 100 at 5 s

    const AccelRun* r = accelLast(a);
    CHECK(r != nullptr);
    CHECK(r->ms[ACCEL_SPLIT_60] > 2900u);
    CHECK(r->ms[ACCEL_SPLIT_60] < 3200u);
    CHECK(r->ms[ACCEL_SPLIT_100] > 4900u);
    CHECK(r->ms[ACCEL_SPLIT_100] < 5200u);
    CHECK(accelMidMs(*r) > 1800u);              // 60-100 is the difference
    CHECK(accelMidMs(*r) < 2200u);
    CHECK(a.best.ms[ACCEL_SPLIT_60] == r->ms[ACCEL_SPLIT_60]);
    CHECK(!accelRunning(a));                    // both in: the run ended itself
  }

  // -------------------------------------------------------------------------
  // The split is INTERPOLATED, not snapped to the sample that first exceeds it
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1u);
    // Two samples a second apart straddling 60: 40 at t=1001, 80 at t=2001.
    // 60 is exactly halfway, so ~t=1501 -> ~1500 ms from the start at t=1.
    accelUpdate(a, 40.0f, DRIVE_RPM, 1001u);
    accelPedal(a, true);                        // a launch, or the gate drops
                                                // it before the second sample
    accelUpdate(a, 80.0f, DRIVE_RPM, 2001u);
    CHECK(a.live.ms[ACCEL_SPLIT_60] > 1400u);   // snapping would give 2000
    CHECK(a.live.ms[ACCEL_SPLIT_60] < 1600u);
  }

  // -------------------------------------------------------------------------
  // Reached 60 in time but never 100: the 60 still counts on its own
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    uint32_t t = ramp(a, 1050u, 50u, 1.0f, 15.0f, 4000u);   // 60 at ~4 s
    hold(a, t, 50u, 70.0f, 12000u);                          // then sit at 70

    const AccelRun* r = accelLast(a);
    CHECK(r != nullptr);
    CHECK(r->ms[ACCEL_SPLIT_60] > 3500u);
    CHECK(r->ms[ACCEL_SPLIT_60] < 4500u);
    CHECK(r->ms[ACCEL_SPLIT_100] == 0u);        // never got there
    CHECK(accelMidMs(*r) == 0u);                // so no 60-100 either
    CHECK(!accelRunning(a));
  }

  // -------------------------------------------------------------------------
  // Ordinary driving is NOT a run: 60 outside its own cut-off records nothing
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    ramp(a, 1050u, 100u, 1.0f, 6.0f, 12000u);   // 60 at ~9.8 s, past the 6 s bar
    CHECK(accelLast(a) == nullptr);
    CHECK(a.best.ms[ACCEL_SPLIT_60] == 0u);
    CHECK(!accelRunning(a));
  }

  // -------------------------------------------------------------------------
  // Coasting back to rest ends the run, keeps what it reached, and re-arms
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    ramp(a, 1050u, 50u, 1.0f, 20.0f, 3200u);    // through 60, short of 100
    CHECK(a.live.ms[ACCEL_SPLIT_60] != 0u);
    accelUpdate(a, 0.0f, IDLE_RPM, 6000u);      // back to a standstill
    CHECK(a.state == ACCEL_ARMED);
    CHECK(accelLast(a) != nullptr);
    CHECK(accelLast(a)->ms[ACCEL_SPLIT_60] != 0u);
    CHECK(accelLast(a)->ms[ACCEL_SPLIT_100] == 0u);
  }

  // -------------------------------------------------------------------------
  // BEST keeps the quickest per split; a slower but still-valid run does not
  // drag it back, and DOES become the most recent
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    ramp(a, 1050u, 50u, 1.0f, 20.0f, 6000u);       // brisk: 60 at ~3 s
    const uint32_t quick = a.best.ms[ACCEL_SPLIT_60];
    CHECK(quick != 0u);

    accelUpdate(a, 0.0f, IDLE_RPM, 20000u);        // stop, re-arm
    ramp(a, 20050u, 50u, 1.0f, 12.0f, 9000u);      // slower, but still valid
    CHECK(accelLast(a)->ms[ACCEL_SPLIT_60] > quick);   // the new run was slower
    CHECK(a.best.ms[ACCEL_SPLIT_60] == quick);         // best is unmoved
    CHECK(a.histCount == 2u);
  }

  // -------------------------------------------------------------------------
  // History keeps the last five, newest first, and stops growing
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    uint32_t t = 1000u;
    for (uint32_t run = 0; run < 7u; ++run) {
      accelUpdate(a, 0.0f, IDLE_RPM, t);
      // Each run a little slower than the last, so they are distinguishable.
      t = ramp(a, t + 50u, 50u, 1.0f, 20.0f - (float)run, 6000u);
      t += 2000u;
    }
    CHECK(a.histCount == ACCEL_HISTORY);
    // Newest first: hist[0] is the slowest, being the last one run.
    CHECK(a.hist[0].ms[ACCEL_SPLIT_60] > a.hist[1].ms[ACCEL_SPLIT_60]);
    CHECK(a.hist[1].ms[ACCEL_SPLIT_60] > a.hist[2].ms[ACCEL_SPLIT_60]);
    // And the best is the very first run, which is no longer in the history.
    CHECK(a.best.ms[ACCEL_SPLIT_60] < a.hist[ACCEL_HISTORY - 1u].ms[ACCEL_SPLIT_60]);
  }

  // -------------------------------------------------------------------------
  // A repeated broadcast stamp is not a new sample
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    accelUpdate(a, 30.0f, DRIVE_RPM, 1050u);
    const uint32_t start = a.startMs;
    for (uint32_t i = 0; i < 50u; ++i) {
      accelUpdate(a, 99.0f, DRIVE_RPM, 1050u);   // same stamp, wild value
    }
    CHECK(a.startMs == start);
    CHECK(a.live.ms[ACCEL_SPLIT_60] == 0u);      // the 99 was never folded in
    CHECK(accelRunning(a));
  }

  // -------------------------------------------------------------------------
  // The clock expires a run even if the speed broadcasts stop mid-launch
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    accelUpdate(a, 20.0f, DRIVE_RPM, 1050u);
    CHECK(accelRunning(a));
    accelTick(a, 1000u + kAccelLimitMs[ACCEL_SPLIT_100] + 500u);
    CHECK(!accelRunning(a));
    CHECK(accelLast(a) == nullptr);              // it reached nothing
  }

  // -------------------------------------------------------------------------
  // 0x2C0 decodes to a pedal, and rejects the engine-off sentinel
  //
  // Frames lifted verbatim from scripts/playback.log. The sentinel matters
  // most: with the engine stopped every byte is 0xFF, which reads as a pedal
  // pressed harder than the floor.
  // -------------------------------------------------------------------------
  {
    BroadcastState bc;
    CanFrame f;
    f.id = 0x2C0u;
    f.len = 8u;

    const uint8_t released[8] = {0xFF, 0xFF, 0x17, 0x0B, 0x7E, 0xFF, 0x17, 0x82};
    memcpy(f.data, released, 8);
    CHECK(broadcastApply(bc, f, 1000u));
    CHECK(bc.pedalRaw == 0x17u);
    CHECK(broadcastPedalPercent(bc.pedalRaw) == 0u);
    CHECK(bc.pedalRaw < BC_PEDAL_KICKDOWN);

    const uint8_t floored[8] = {0xFF, 0xFF, 0xEA, 0x75, 0xFE, 0xFF, 0xEA, 0x82};
    memcpy(f.data, floored, 8);
    CHECK(broadcastApply(bc, f, 2000u));
    CHECK(bc.pedalRaw == 0xEAu);
    CHECK(broadcastPedalPercent(bc.pedalRaw) == 100u);
    CHECK(bc.pedalRaw >= BC_PEDAL_KICKDOWN);

    // Engine off: all 0xFF. Must be refused, and must not move the reading.
    const uint8_t sentinel[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    memcpy(f.data, sentinel, 8);
    CHECK(!broadcastApply(bc, f, 3000u));
    CHECK(bc.pedalRaw == 0xEAu);          // unchanged
    CHECK(bc.pedalMs == 2000u);

    // The redundant copy in byte 6 disagreeing is a corrupt frame.
    const uint8_t mismatch[8] = {0xFF, 0xFF, 0x40, 0x20, 0x90, 0xFF, 0x41, 0x82};
    memcpy(f.data, mismatch, 8);
    CHECK(!broadcastApply(bc, f, 4000u));
    CHECK(bc.pedalRaw == 0xEAu);

    // Part throttle, from the 2026 drive log.
    const uint8_t part[8] = {0xFF, 0xFF, 0x33, 0x18, 0x8C, 0xFF, 0x33, 0x82};
    memcpy(f.data, part, 8);
    CHECK(broadcastApply(bc, f, 5000u));
    CHECK(broadcastPedalPercent(bc.pedalRaw) > 5u);
    CHECK(broadcastPedalPercent(bc.pedalRaw) < 20u);
  }

  // -------------------------------------------------------------------------
  // Kickdown is latched onto the run, and only onto a run
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelPedal(a, true);                       // nothing running: ignored
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    accelUpdate(a, 5.0f, DRIVE_RPM, 1050u);    // run starts
    CHECK(!a.live.floored);
    accelPedal(a, true);
    CHECK(a.live.floored);
    ramp(a, 1100u, 50u, 5.0f, 20.0f, 6000u);
    CHECK(accelLast(a)->floored);
    CHECK(a.best.floored);
  }

  // A run made without the detent is DISCARDED, not kept as a lesser one.
  //
  // This assertion used to be the other way round -- the flag was decoration
  // and a quick-but-unfloored run was published and shown dimmer. Pulling out
  // of a junction therefore registered as an acceleration run, which is the
  // defect ACCEL_LAUNCH_MS answers.
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    ramp(a, 1050u, 50u, 1.0f, 20.0f, 6000u, false);   // quick, never floored
    CHECK(accelLast(a) == nullptr);
    CHECK(a.best.ms[ACCEL_SPLIT_60] == 0u);
  }

  // -------------------------------------------------------------------------
  // A GENTLE pull-away is not a run, however quick the car turns out to be
  //
  // This is the defect the launch gate exists for: rest-then-motion used to be
  // the entire trigger, so every set of lights started the stopwatch and the
  // DRIVE sub-line flipped away from the journey for no reason.
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);       // stopped and idling: armed
    CHECK(a.state == ACCEL_ARMED);
    // Moves, and keeps moving hard -- but the pedal never reaches the detent.
    const uint32_t t = ramp(a, 1050u, 50u, 0.0f, 40.0f, 5000u, false);
    CHECK(!accelLaunched(a));
    CHECK(!accelRunning(a));                     // dropped, not timing
    CHECK(accelLast(a) == nullptr);              // and nothing published
    CHECK(a.best.ms[ACCEL_SPLIT_60] == 0u);
    CHECK(a.best.ms[ACCEL_SPLIT_100] == 0u);
    (void)t;
  }

  // -------------------------------------------------------------------------
  // The same launch WITH the pedal records normally
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    ramp(a, 1050u, 50u, 0.0f, 40.0f, 5000u, true);
    CHECK(accelLast(a) != nullptr);
    CHECK(accelLast(a)->ms[ACCEL_SPLIT_60] != 0u);
    CHECK(accelLast(a)->floored);
  }

  // -------------------------------------------------------------------------
  // The pedal may arrive a beat AFTER the car moves, but not much later
  //
  // 0x2C0 is a 100 ms frame and a foot is not simultaneous with the first
  // millimetre, which is what the window is for. Past it the run is gone even
  // if the driver then floors it.
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    accelUpdate(a, 3.0f, DRIVE_RPM, 1050u);      // moving: timing starts
    CHECK(accelRunning(a));
    CHECK(!accelLaunched(a));                    // not yet a launch
    accelUpdate(a, 12.0f, DRIVE_RPM, 1300u);
    accelPedal(a, true);                         // 250 ms in: still a launch
    CHECK(accelLaunched(a));
    ramp(a, 1350u, 50u, 12.0f, 40.0f, 4000u, true);
    CHECK(accelLast(a) != nullptr);
    CHECK(accelLast(a)->ms[ACCEL_SPLIT_60] != 0u);
  }
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    // Crawls for two seconds, then decides to overtake. Not a 0-60.
    uint32_t t = hold(a, 1050u, 50u, 5.0f, 2000u, false);
    CHECK(!accelRunning(a));
    accelPedal(a, true);                         // too late; nothing running
    t = ramp(a, t, 50u, 5.0f, 40.0f, 4000u, true);
    CHECK(accelLast(a) == nullptr);
    (void)t;
  }

  // -------------------------------------------------------------------------
  // A run that goes off the bus unlaunched is dropped by the clock alone
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    accelUpdate(a, 4.0f, DRIVE_RPM, 1050u);      // moves, then silence
    CHECK(accelRunning(a));
    accelTick(a, 1600u);                         // inside the window still
    CHECK(accelRunning(a));
    accelTick(a, 2700u);                         // past it
    CHECK(!accelRunning(a));
    CHECK(accelLast(a) == nullptr);
  }

  // -------------------------------------------------------------------------
  // A run that FINISHES inside the launch window is still thrown away
  //
  // This was a live bug, and the gate it needed is in finish() rather than in
  // dropUnlaunched(). dropUnlaunched() is purely TEMPORAL -- it fires only
  // once ACCEL_LAUNCH_MS has passed -- so a run that reaches every split
  // before that never met it, went straight through the "all splits in the
  // bag" branch and was published without a launch.
  //
  // Two samples 100 ms apart do it, which is not a contrived input: the 0x002
  // speed word is rejected by core/Broadcast.cpp only at EXACTLY 0xFFFF, so
  // 0xFFFE decodes as a real 511.98 km/h, and playback.log carries four
  // near-sentinel speed frames in one 841 s drive. The published figure was
  // 0-60 in 55 ms and 0-100 in 91 ms.
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, 600u, 1000u);        // at rest, idling: ARMED
    accelUpdate(a, 110.0f, 600u, 1100u);      // ONE sample, 100 ms later
    CHECK(accelLast(a) == nullptr);           // was: 55 ms / 91 ms, floored=0
    CHECK(a.histCount == 0u);
    CHECK(a.best.ms[ACCEL_SPLIT_60] == 0u);
    CHECK(a.best.ms[ACCEL_SPLIT_100] == 0u);
    CHECK(!a.best.floored);
    CHECK(a.live.ms[ACCEL_SPLIT_60] == 0u);   // and nothing left in `live` for
    CHECK(a.live.ms[ACCEL_SPLIT_100] == 0u);  // the next arming to inherit
  }

  // -------------------------------------------------------------------------
  // ... and BEST is what makes that permanent, so it is asserted separately
  //
  // `best` only ever moves FASTER and accelInit() runs once per boot, so a
  // bogus 55 ms 0-60 could never be displaced by a genuine 3 s launch: it
  // would sit on the page beside every real run for the rest of the key
  // cycle. The second half of this case is the one that used to fail.
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, 600u, 1000u);
    accelUpdate(a, 110.0f, 600u, 1100u);      // the corrupt sample
    CHECK(a.best.ms[ACCEL_SPLIT_60] == 0u);

    // A genuine floored launch, 20 s later: 60 at ~3 s, 100 at ~5 s. It must
    // publish, and it must OWN `best` -- there is nothing quicker to beat.
    accelUpdate(a, 0.0f, IDLE_RPM, 20000u);
    CHECK(a.state == ACCEL_ARMED);
    ramp(a, 20050u, 50u, 1.0f, 20.0f, 6000u, true);
    const AccelRun* r = accelLast(a);
    CHECK(r != nullptr);
    CHECK(r->floored);
    CHECK(r->ms[ACCEL_SPLIT_60] > 2900u);
    CHECK(r->ms[ACCEL_SPLIT_60] < 3200u);
    CHECK(a.histCount == 1u);                 // the bogus run is not in there
    CHECK(a.best.ms[ACCEL_SPLIT_60] == r->ms[ACCEL_SPLIT_60]);
    CHECK(a.best.ms[ACCEL_SPLIT_100] == r->ms[ACCEL_SPLIT_100]);
    CHECK(a.best.floored);
  }

  // -------------------------------------------------------------------------
  // The CAME-TO-REST path reaches the gate too
  //
  // finish() has four callers and the two that can be hit inside the launch
  // window are "all splits in the bag" and this one. Coasting back to a
  // standstill publishes whatever the run reached -- correctly, for a launched
  // run -- so an unlaunched one must be refused here as well, or a corrupt
  // sample followed by a stop would publish a single split instead of two.
  // -------------------------------------------------------------------------
  {
    AccelSet a;
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    accelUpdate(a, 70.0f, DRIVE_RPM, 1100u);  // one sample past 60, no pedal
    CHECK(a.live.ms[ACCEL_SPLIT_60] != 0u);   // the split IS recorded...
    CHECK(accelRunning(a));                   // ...and the run is still in
                                              // flight: 100 ms < 1500 ms
    accelUpdate(a, 0.0f, IDLE_RPM, 1400u);    // back to rest, still inside the
                                              // window
    CHECK(accelLast(a) == nullptr);           // nothing published
    CHECK(a.best.ms[ACCEL_SPLIT_60] == 0u);
    CHECK(a.state == ACCEL_ARMED);            // and it re-arms as before
  }

  // -------------------------------------------------------------------------
  // The same three paths for a LAUNCHED run all still publish
  //
  // The gate must not have cost anything. One run per finish() caller that can
  // carry a launch: every split in the bag, the car coming to rest, and the
  // 12 s cut-off expiring a run that is still moving.
  // -------------------------------------------------------------------------
  {
    AccelSet a;                                // all splits in the bag
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    ramp(a, 1050u, 50u, 1.0f, 20.0f, 6000u, true);
    CHECK(accelLast(a) != nullptr);
    CHECK(a.best.ms[ACCEL_SPLIT_100] != 0u);
  }
  {
    AccelSet a;                                // came to rest with one split
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    ramp(a, 1050u, 50u, 1.0f, 20.0f, 3200u, true);
    accelUpdate(a, 0.0f, IDLE_RPM, 6000u);
    CHECK(accelLast(a) != nullptr);
    CHECK(accelLast(a)->ms[ACCEL_SPLIT_60] != 0u);
    CHECK(accelLast(a)->ms[ACCEL_SPLIT_100] == 0u);
    CHECK(a.best.ms[ACCEL_SPLIT_60] != 0u);
  }
  {
    AccelSet a;                                // the cut-off, off the bus
    accelInit(a);
    accelUpdate(a, 0.0f, IDLE_RPM, 1000u);
    accelUpdate(a, 70.0f, DRIVE_RPM, 1050u);
    accelPedal(a, true);                       // launched, then silence
    CHECK(accelLaunched(a));
    accelTick(a, 1000u + kAccelLimitMs[ACCEL_SPLIT_100] + 500u);
    CHECK(!accelRunning(a));
    CHECK(accelLast(a) != nullptr);            // the 60 it reached still counts
    CHECK(accelLast(a)->ms[ACCEL_SPLIT_60] != 0u);
    CHECK(a.best.ms[ACCEL_SPLIT_60] != 0u);
  }

TEST_MAIN_END()
