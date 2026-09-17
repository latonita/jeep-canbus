// Host tests for core/Broadcast.cpp.
//
// core/Broadcast.cpp is THE primary data path of this instrument: passive
// CAN-B at a measured 151 frames/s, and the origin of almost everything the
// driver reads. It is also the only core module that had no suite of its own.
// It was exercised sideways -- test_trip and test_accel hand-build
// BroadcastState structs and never call broadcastApply() at all, and
// scripts/replay-accel.cpp proves the decode is self-consistent across one
// real 841 s drive but asserts nothing about any signal that drive did not
// happen to carry. So the scaling, the sentinel handling and the short-frame
// guards of the busiest file in core/ were all unverified.
//
// What this suite is mainly for is THE SENTINEL SEMANTICS, because there are
// two of them and conflating them is the exact failure this whole project is
// written around: "hold the last value and dim it" is the right answer for a
// MISSED frame and the wrong answer for a module that has just told us it has
// no value. Broadcast.cpp implements three distinct policies and they are not
// interchangeable:
//
//   * ZEROES THE STAMP, keeps the value -- rpm, speed, coolant, ambient,
//     volts. These are DISPLAYED numbers. Stamp 0 is how the UI is told to
//     draw dashes rather than hold-and-dim. Every one of these sentinels
//     scales into something believable, which is the whole problem: 0xFFFF
//     speed is 512.0 km/h and CLAUDE.md records four such frames late in one
//     real capture -- each would have produced a fake 0.1 s 0-60 had it leaked
//     through.
//   * REJECTS THE WHOLE FRAME, stamp untouched -- odometer (0x014) and fuel
//     flow (0x1A7). These are INTEGRATOR inputs, and what the consumer needs
//     to see is a GAP: Trip then drops the interval instead of integrating,
//     and the trip's odometer anchor stays where it was. Storing 0xFFFFFF
//     would re-anchor every trip to 1 677 721.5 km, a distance no trip can
//     reach, so the kilometres would never appear again.
//   * NO SENTINEL AT ALL -- the dimmer (0x210) and the lights byte (0x1C8).
//     Every value is a legal reading there, including the 0x00 that means two
//     opposite things (bottom of the wheel, and every light off). That
//     ambiguity is resolved in broadcastBacklightPermille(), not at decode.
//
// Every frame built here carries all EIGHT data bytes even when f.len is 1, on
// purpose. The short-frame sweeps then test the length GUARD and only that --
// and any state change one of them causes is positive proof the guard was
// skipped, rather than a sanitizer report about the harness's own
// uninitialised memory.
#include "doctest_min.h"

#include "Broadcast.h"

// A frame with all eight bytes written, whatever `len` says. See the header
// comment: this is what makes a short-frame rejection provable instead of
// merely un-sanitized.
static CanFrame mk(uint16_t id, uint8_t len,
                   uint8_t b0 = 0u, uint8_t b1 = 0u, uint8_t b2 = 0u,
                   uint8_t b3 = 0u, uint8_t b4 = 0u, uint8_t b5 = 0u,
                   uint8_t b6 = 0u, uint8_t b7 = 0u) {
  CanFrame f;
  f.id = id;
  f.len = len;
  f.data[0] = b0;
  f.data[1] = b1;
  f.data[2] = b2;
  f.data[3] = b3;
  f.data[4] = b4;
  f.data[5] = b5;
  f.data[6] = b6;
  f.data[7] = b7;
  return f;
}

// Compare every VALUE field, ignoring the two counters -- which is exactly the
// question "did this frame change anything it was not supposed to?".
//
// Field by field rather than memcmp, and that is deliberate: BroadcastState
// has interior padding, so a memcmp would compare bytes no member owns and
// would answer a different question depending on the compiler's mood. It also
// has to name every field, which is the property that makes the same helper
// useful for the broadcastClear() walk below -- adding a field to the struct
// and forgetting it here is a compile-time-visible omission in one place
// instead of a silent hole in twelve tests.
static bool sameValues(const BroadcastState& a, const BroadcastState& b) {
  return a.rpm == b.rpm && a.rpmMs == b.rpmMs &&
         a.speedKph == b.speedKph && a.speedMs == b.speedMs &&
         a.coolantC == b.coolantC && a.coolantMs == b.coolantMs &&
         a.ambientC == b.ambientC && a.ambientMs == b.ambientMs &&
         a.volts == b.volts && a.voltsMs == b.voltsMs &&
         a.odometerDm == b.odometerDm && a.odometerMs == b.odometerMs &&
         a.ignition == b.ignition && a.ignitionMs == b.ignitionMs &&
         a.dimmerRaw == b.dimmerRaw && a.dimmerMs == b.dimmerMs &&
         a.lightsOn == b.lightsOn && a.lightsRaw == b.lightsRaw &&
         a.lightsMs == b.lightsMs && a.dimmerByte0 == b.dimmerByte0 &&
         a.pedalRaw == b.pedalRaw && a.pedalMs == b.pedalMs &&
         a.flowRaw == b.flowRaw && a.flowMs == b.flowMs &&
         a.fuelRaw == b.fuelRaw && a.fuelMs == b.fuelMs &&
         a.headingOctant == b.headingOctant && a.headingRaw == b.headingRaw &&
         a.headingStatus == b.headingStatus && a.headingMs == b.headingMs;
}

// Sweep f.len from 0 up to (but not including) the minimum this id needs, and
// require every one of them to be rejected AND to change nothing. `frames`
// must still count them -- a malformed frame was still a frame on the wire, and
// the frames/decoded ratio in the boot log is how a mis-set adapter filter is
// noticed at all.
static void expectShortRejected(const CanFrame& good, uint8_t minLen) {
  for (uint8_t len = 0u; len < minLen; ++len) {
    BroadcastState st;
    broadcastClear(st);
    // Pre-load a plausible history so a guard-skipping decode has something to
    // overwrite and be caught doing it.
    st.rpm = 1234u;      st.rpmMs = 500u;
    st.speedKph = 42.0f; st.speedMs = 500u;
    st.coolantC = 88;    st.coolantMs = 500u;
    st.ambientC = 11.5f; st.ambientMs = 500u;
    st.volts = 14.1f;    st.voltsMs = 500u;
    st.odometerDm = 2462785u; st.odometerMs = 500u;
    st.ignition = IGN_RUN;    st.ignitionMs = 500u;
    st.dimmerRaw = 0x40u;     st.dimmerMs = 500u;
    st.dimmerByte0 = 0x03u;
    st.lightsOn = false;      st.lightsRaw = 0x01u; st.lightsMs = 500u;
    st.pedalRaw = 0x17u;      st.pedalMs = 500u;
    st.flowRaw = 1600u;       st.flowMs = 500u;
    // A NON-ZERO fuel stamp, and it is load-bearing rather than decorative:
    // 0x159's unavailable path ZEROES this stamp, so a guard-skipping decode
    // of a short frame that happened to carry a non-zero b4 would write the
    // zero that was already there and change nothing visible. Pre-loading 500
    // is what makes that skip provable.
    st.fuelRaw = 140u;        st.fuelMs = 500u;
    st.headingOctant = 2u;    st.headingRaw = 45u;
    st.headingStatus = 0x07u; st.headingMs = 500u;

    BroadcastState before = st;
    CanFrame f = good;
    f.len = len;
    const bool ok = broadcastApply(st, f, 9000u);
    CHECK_MSG(!ok, "id 0x%03X len %u must be rejected", (unsigned)good.id,
              (unsigned)len);
    CHECK_MSG(sameValues(st, before),
              "id 0x%03X len %u changed state", (unsigned)good.id,
              (unsigned)len);
    CHECK_MSG(st.frames == 1u && st.decoded == 0u,
              "id 0x%03X len %u counters: frames=%u decoded=%u",
              (unsigned)good.id, (unsigned)len, (unsigned)st.frames,
              (unsigned)st.decoded);
  }
}

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // broadcastClear() clears EVERYTHING
  //
  // Walked field by field, not spot-checked, and this is the one test here
  // that is about maintenance rather than about the car. broadcastClear() is a
  // hand-written list of assignments, so the failure mode is not a wrong value
  // but an OMITTED line -- and an omitted line is invisible in review and
  // invisible at runtime on the firmware, because the one real caller clears a
  // file-scope static that the C runtime had already zeroed. It only bites
  // somewhere else: a stack-local BroadcastState (every suite in test/ has
  // one), or a second instance, or a re-clear after the bus has been running.
  //
  // Every field is poisoned with a pattern that is both non-zero and
  // ACTIVELY WRONG -- stamps far in the future, a pedal on the floor, an
  // odometer at the sentinel -- so a survivor is not merely stale, it is the
  // sort of value that arms things.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    st.rpm = 4163u;            st.rpmMs = 0xDEADBEEFu;
    st.speedKph = 82.9f;       st.speedMs = 0xDEADBEEFu;
    st.coolantC = -1234;       st.coolantMs = 0xDEADBEEFu;
    st.ambientC = 87.0f;       st.ambientMs = 0xDEADBEEFu;
    st.volts = 25.4f;          st.voltsMs = 0xDEADBEEFu;
    st.odometerDm = 0xFFFFFFu; st.odometerMs = 0xDEADBEEFu;
    st.ignition = IGN_CRANK;   st.ignitionMs = 0xDEADBEEFu;
    st.dimmerRaw = 0xC8u;      st.dimmerMs = 0xDEADBEEFu;
    st.lightsOn = false;       st.lightsMs = 0xDEADBEEFu;
    st.lightsRaw = 0x7Fu;      st.dimmerByte0 = 0x03u;
    st.pedalRaw = BC_PEDAL_FLOORED; st.pedalMs = 0xDEADBEEFu;
    st.flowRaw = 0xFFFFu;      st.flowMs = 0xDEADBEEFu;
    // A tank reading that is wrong in the one direction that matters: raw 255
    // is 127.5 L against a 77.6 L tank, and the largest VALID raw any log in
    // this repository contains is 154. A survivor here is an over-full tank
    // with a plausible age on a bus that has said nothing yet.
    st.fuelRaw = 0xFFu;        st.fuelMs = 0xDEADBEEFu;
    // Due WEST, on a heading of 356 deg, which is very nearly due north: the
    // octant and the heading contradict each other and neither is where the
    // truck is pointing. A status of 0xFF is a value neither capture contains.
    st.headingOctant = 6u;     st.headingRaw = 178u;
    st.headingStatus = 0xFFu;  st.headingMs = 0xDEADBEEFu;
    st.frames = 123456u;       st.decoded = 65432u;

    broadcastClear(st);

    CHECK_EQ(st.rpm, 0);
    CHECK_EQ(st.rpmMs, 0);
    CHECK_NEAR(st.speedKph, 0.0f, 0.0f);
    CHECK_EQ(st.speedMs, 0);
    CHECK_EQ(st.coolantC, 0);
    CHECK_EQ(st.coolantMs, 0);
    CHECK_NEAR(st.ambientC, 0.0f, 0.0f);
    CHECK_EQ(st.ambientMs, 0);
    CHECK_NEAR(st.volts, 0.0f, 0.0f);
    CHECK_EQ(st.voltsMs, 0);
    CHECK_EQ(st.odometerDm, 0);
    CHECK_EQ(st.odometerMs, 0);
    CHECK_EQ(st.ignition, IGN_UNKNOWN);
    CHECK_EQ(st.ignitionMs, 0);
    CHECK_EQ(st.dimmerRaw, 0);
    CHECK_EQ(st.dimmerMs, 0);
    CHECK_EQ(st.dimmerByte0, 0);
    CHECK_EQ(st.lightsRaw, 0);
    CHECK_EQ(st.lightsMs, 0);
    // NOT false. "Lights on" is the deliberate default, because an absent
    // 0x1C8 must leave the dimmer wheel in full control rather than pin the
    // panel bright and take the wheel away from the driver.
    CHECK(st.lightsOn);
    // The pedal pair. pedalRaw surviving a clear at BC_PEDAL_FLOORED is not a
    // cosmetic stale value: main.cpp arms an acceleration run on
    // `pedalRaw >= BC_PEDAL_KICKDOWN` gated on pedalMs freshness, so a
    // survivor of 0xEA with a plausible stamp confirms the launch of the first
    // pull-away after the clear. Frozen, plausible and wrong, on the feature
    // the launch gate was added to fix.
    CHECK_EQ(st.pedalRaw, 0);
    CHECK_EQ(st.pedalMs, 0);
    CHECK_EQ(st.flowRaw, 0);
    CHECK_EQ(st.flowMs, 0);
    CHECK_EQ(st.fuelRaw, 0);
    CHECK_EQ(st.fuelMs, 0);
    // The compass trio and their shared stamp. 0x1B4 carries no unavailable
    // sentinel, so headingMs is zero ONLY before the first frame -- which makes
    // this clear the one and only thing that can ever put the readout back into
    // ABSENT. A surviving stamp here is a compass that is showable, dimmed and
    // pointing wherever the last instance happened to leave it.
    CHECK_EQ(st.headingOctant, 0);
    CHECK_EQ(st.headingRaw, 0);
    CHECK_EQ(st.headingStatus, 0);
    CHECK_EQ(st.headingMs, 0);
    CHECK_EQ(st.frames, 0);
    CHECK_EQ(st.decoded, 0);

    // And the same thing again from the other direction: a freshly cleared
    // state must equal a freshly cleared state, whatever was in it before.
    // This is the assertion that keeps catching a NEW field added to the
    // struct and not to broadcastClear().
    BroadcastState pristine;
    broadcastClear(pristine);
    CHECK(sameValues(st, pristine));
  }

  // -------------------------------------------------------------------------
  // Unknown ids: counted, not decoded, and nothing touched
  //
  // The bus carries far more than the nine ids below -- CLAUDE.md's own
  // measurement is 151 frames/s of which only a fraction is understood -- so
  // the default arm of that switch is the hottest path in the file. It must be
  // inert, and `frames` must still count, because `frames`/`decoded` in the
  // boot log is the only thing that distinguishes "the bus is quiet" from "the
  // adapter is filtering us out".
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    BroadcastState before = st;
    // A spread of ids either side of and between the handled ones, including
    // one-bit neighbours of each -- the kind a mis-typed constant produces.
    const uint16_t kUnknown[] = {0x001u, 0x003u, 0x004u, 0x010u, 0x013u,
                                 0x016u, 0x02Du, 0x1A6u, 0x1A8u, 0x1C9u,
                                 0x1CCu, 0x200u, 0x211u, 0x2C1u, 0x3FFu,
                                 0x7FFu};
    bool anyDecoded = false;
    for (unsigned i = 0u; i < sizeof(kUnknown) / sizeof(kUnknown[0]); ++i) {
      // Every byte 0xFF: if any of these ever starts being decoded, an
      // all-ones payload is the loudest possible wrong answer.
      CanFrame f = mk(kUnknown[i], 8u, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
                      0xFFu, 0xFFu, 0xFFu, 0xFFu);
      if (broadcastApply(st, f, 1000u)) {
        anyDecoded = true;
      }
    }
    CHECK(!anyDecoded);
    CHECK(sameValues(st, before));
    CHECK_EQ(st.frames, sizeof(kUnknown) / sizeof(kUnknown[0]));
    CHECK_EQ(st.decoded, 0);
  }

  // -------------------------------------------------------------------------
  // 0x000 -- key position. Never invented.
  //
  // The four raw codes were observed walking a real key cycle, and anything
  // else is REFUSED rather than mapped to a nearest neighbour. An ignition
  // state is a latch that other logic keys off; guessing one is worse than
  // admitting ignorance, which is why the unknown arm returns false and leaves
  // both the value and its stamp exactly where they were.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    CHECK(broadcastApply(st, mk(0x000u, 8u, 0x01u), 1000u));
    CHECK_EQ(st.ignition, IGN_OFF);
    CHECK_EQ(st.ignitionMs, 1000);
    CHECK(broadcastApply(st, mk(0x000u, 1u, 0x63u), 2000u));
    CHECK_EQ(st.ignition, IGN_ACC);
    CHECK(broadcastApply(st, mk(0x000u, 1u, 0x85u), 3000u));
    CHECK_EQ(st.ignition, IGN_RUN);
    CHECK(broadcastApply(st, mk(0x000u, 1u, 0xA9u), 4000u));
    CHECK_EQ(st.ignition, IGN_CRANK);
    CHECK_EQ(st.ignitionMs, 4000);

    // An unrecognised code holds the last KNOWN state and its stamp. Note the
    // asymmetry with the display signals: it does NOT zero the stamp, because
    // an unmapped byte is our ignorance of the encoding rather than the
    // module reporting that it has no key position.
    const uint32_t heldMs = st.ignitionMs;
    bool anyInvented = false;
    for (unsigned raw = 0u; raw <= 255u; ++raw) {
      if (raw == 0x01u || raw == 0x63u || raw == 0x85u || raw == 0xA9u) {
        continue;
      }
      if (broadcastApply(st, mk(0x000u, 1u, (uint8_t)raw), 5000u)) {
        anyInvented = true;
      }
    }
    CHECK(!anyInvented);
    CHECK_EQ(st.ignition, IGN_CRANK);
    CHECK_EQ(st.ignitionMs, heldMs);
    // 252 rejects, and not one of them counted as decoded.
    CHECK_EQ(st.decoded, 4);
    CHECK_EQ(st.frames, 4 + 252);

    expectShortRejected(mk(0x000u, 1u, 0x85u), 1u);
  }

  // -------------------------------------------------------------------------
  // 0x002 -- rpm and speed, the 50 ms frame everything else is timed against
  //
  // Big-endian pairs, rpm raw and speed at 1/128 km/h. The numbers used here
  // are the peaks scripts/replay-accel.cpp measured over a real 841 s drive
  // (4163 rpm, 82.9 km/h), so the scaling is anchored to something that
  // actually happened rather than to a table.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    CHECK(broadcastApply(st, mk(0x002u, 8u, 0x10u, 0x43u, 0x29u, 0x73u), 1000u));
    CHECK_EQ(st.rpm, 4163);                     // 0x1043
    CHECK_EQ(st.rpmMs, 1000);
    CHECK_NEAR(st.speedKph, 82.8984f, 0.001f);  // 0x2973 / 128
    CHECK_EQ(st.speedMs, 1000);
    CHECK_EQ(st.decoded, 1);

    // The low end. 0x0040 is 0.5 km/h, which is the threshold core/Accel.h
    // calls stationary -- so this particular quantum has to land exactly or a
    // run arms one count early or late.
    CHECK(broadcastApply(st, mk(0x002u, 4u, 0x00u, 0x00u, 0x00u, 0x40u), 1100u));
    CHECK_EQ(st.rpm, 0);                        // engine stopped IS a reading
    CHECK_EQ(st.rpmMs, 1100);                   // ...and it is a fresh one
    CHECK_NEAR(st.speedKph, 0.5f, 1e-6f);
    CHECK(broadcastApply(st, mk(0x002u, 4u, 0x00u, 0x01u, 0x00u, 0x80u), 1200u));
    CHECK_EQ(st.rpm, 1);
    CHECK_NEAR(st.speedKph, 1.0f, 1e-6f);

    // Bytes 4..7 carry something we do not decode; a change in them must not
    // move a single value we do.
    BroadcastState before = st;
    CHECK(broadcastApply(st, mk(0x002u, 8u, 0x00u, 0x01u, 0x00u, 0x80u,
                                0xAAu, 0x55u, 0xAAu, 0x55u), 1200u));
    CHECK(sameValues(st, before));
  }

  // -------------------------------------------------------------------------
  // 0x002 sentinels -- THE 512 km/h LIE
  //
  // 0xFFFF speed scales to 511.99 km/h and 0xFFFF rpm to 65535. CLAUDE.md
  // records FOUR frames late in scripts/playback.log reading exactly that, and
  // each one would have produced a fake 0.1 s 0-60 in core/Accel: the run was
  // already armed from a standstill, and a single 512 km/h sample crosses both
  // splits inside one 50 ms frame. That is not hypothetical, it is a thing
  // that happened and that this guard is the reason nobody saw.
  //
  // The policy is ZERO THE STAMP, KEEP THE VALUE. Zeroing the stamp is how the
  // UI is told to draw dashes instead of hold-and-dim; keeping the value costs
  // nothing because no consumer may read it with the stamp at 0.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    // A live reading first, so "held" is distinguishable from "never set".
    CHECK(broadcastApply(st, mk(0x002u, 4u, 0x07u, 0xD0u, 0x10u, 0x00u), 1000u));
    CHECK_EQ(st.rpm, 2000);
    CHECK_NEAR(st.speedKph, 32.0f, 1e-6f);

    // rpm sentinel alone: rpm blanks, speed carries on. The frame is still
    // decoded -- half of it was real.
    CHECK(broadcastApply(st, mk(0x002u, 4u, 0xFFu, 0xFFu, 0x08u, 0x00u), 2000u));
    CHECK_EQ(st.rpmMs, 0);
    CHECK_EQ(st.rpm, 2000);                     // held, but unreadable
    CHECK_NEAR(st.speedKph, 16.0f, 1e-6f);
    CHECK_EQ(st.speedMs, 2000);
    CHECK_EQ(st.decoded, 2);

    // speed sentinel alone: the 512 must not appear anywhere.
    CHECK(broadcastApply(st, mk(0x002u, 4u, 0x07u, 0xD0u, 0xFFu, 0xFFu), 3000u));
    CHECK_EQ(st.speedMs, 0);
    CHECK_NEAR(st.speedKph, 16.0f, 1e-6f);      // still the last real one
    CHECK(st.speedKph < 500.0f);                // the assertion that matters
    CHECK_EQ(st.rpm, 2000);
    CHECK_EQ(st.rpmMs, 3000);
    CHECK_EQ(st.decoded, 3);

    // Both sentinels: both stamps zeroed AND the frame reported as carrying
    // nothing, so `decoded` does not move. The engine-off frame on this id is
    // exactly this, and counting it as understood would make the
    // frames/decoded ratio in the log claim a healthy bus at rest.
    CHECK(!broadcastApply(st, mk(0x002u, 4u, 0xFFu, 0xFFu, 0xFFu, 0xFFu), 4000u));
    CHECK_EQ(st.rpmMs, 0);
    CHECK_EQ(st.speedMs, 0);
    CHECK_EQ(st.decoded, 3);
    CHECK_EQ(st.frames, 4);

    // 0xFFFE is NOT the sentinel and must decode: the guard is exact-match, and
    // a `>=` there would silently throw away 511.99 km/h and 65534 rpm --
    // unreachable on this truck, but the wrong shape of test.
    CHECK(broadcastApply(st, mk(0x002u, 4u, 0xFFu, 0xFEu, 0xFFu, 0xFEu), 5000u));
    CHECK_EQ(st.rpm, 65534);
    CHECK_EQ(st.rpmMs, 5000);
    CHECK_EQ(st.speedMs, 5000);

    expectShortRejected(mk(0x002u, 4u, 0x07u, 0xD0u, 0x10u, 0x00u), 4u);
  }

  // -------------------------------------------------------------------------
  // 0x014 -- the odometer, and the one sentinel that HOLDS instead of blanking
  //
  // A uint24 in tenths of a km. CLAUDE.md: distance is the odometer
  // DIFFERENCED, never integrated speed -- which is what makes this the one
  // signal where a leaked sentinel is unrecoverable rather than ugly. Stored,
  // 0xFFFFFF is 1 677 721.5 km and it becomes the trip's anchor; every
  // subsequent reading is then hundreds of thousands of km SMALLER, so the
  // kilometres do not merely glitch, they never appear again for the life of
  // that trip.
  //
  // So the policy here is the other one: reject the whole frame and leave
  // odometerMs alone. Trip sees no new reading, keeps its anchor, and picks up
  // where it was when a real odometer frame returns.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    // 0x259441 = 2 462 785 tenths = 246 278.5 km, the owner's real reading.
    CHECK(broadcastApply(st, mk(0x014u, 8u, 0x25u, 0x94u, 0x41u), 1000u));
    CHECK_EQ(st.odometerDm, 2462785);
    CHECK_EQ(st.odometerMs, 1000);

    CHECK(broadcastApply(st, mk(0x014u, 3u, 0x00u, 0x00u, 0x00u), 1100u));
    CHECK_EQ(st.odometerDm, 0);                 // a zero odometer is a value
    CHECK_EQ(st.odometerMs, 1100);

    // Back to something real, then the sentinel.
    CHECK(broadcastApply(st, mk(0x014u, 3u, 0x25u, 0x94u, 0x41u), 2000u));
    const uint32_t held = st.odometerDm;
    const uint32_t heldMs = st.odometerMs;
    CHECK(!broadcastApply(st, mk(0x014u, 8u, 0xFFu, 0xFFu, 0xFFu), 3000u));
    CHECK_EQ(st.odometerDm, held);              // value untouched
    CHECK_EQ(st.odometerMs, heldMs);            // AND the stamp: this one holds
    CHECK_EQ(st.decoded, 3);

    // One count short of all-ones is a legal reading and must survive -- the
    // guard is exact-match on 0xFFFFFF, and only on the low 24 bits.
    CHECK(broadcastApply(st, mk(0x014u, 3u, 0xFFu, 0xFFu, 0xFEu), 4000u));
    CHECK_EQ(st.odometerDm, 0xFFFFFEu);
    CHECK_EQ(st.odometerMs, 4000);
    // And the top byte of the uint24 is byte 0, not byte 2: a byte-order slip
    // would read 0x4194_25 here instead.
    CHECK(broadcastApply(st, mk(0x014u, 3u, 0x01u, 0x00u, 0x00u), 4100u));
    CHECK_EQ(st.odometerDm, 65536);

    expectShortRejected(mk(0x014u, 3u, 0x25u, 0x94u, 0x41u), 3u);
  }

  // -------------------------------------------------------------------------
  // 0x015 -- ambient temperature and battery volts
  //
  // Ambient is 0.5 C/bit with a -40 offset; volts is 0.1 V/bit with NO OFFSET.
  // That second one is the interesting assertion: the extracted signal table
  // this project started from carries a +5.0 V offset, which reads 17.6 V where
  // the truck is sitting at 12.6 V. A plausible-looking overvoltage on the one
  // readout a driver uses to decide whether the alternator has died.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    CHECK(broadcastApply(st, mk(0x015u, 8u, 0x50u, 0x7Eu), 1000u));
    CHECK_NEAR(st.ambientC, 0.0f, 1e-6f);       // 80 * 0.5 - 40
    CHECK_EQ(st.ambientMs, 1000);
    CHECK_NEAR(st.volts, 12.6f, 0.001f);        // 126 * 0.1, and NOT 17.6
    CHECK(st.volts < 15.0f);
    CHECK_EQ(st.voltsMs, 1000);

    CHECK(broadcastApply(st, mk(0x015u, 2u, 0x00u, 0x8Cu), 1100u));
    CHECK_NEAR(st.ambientC, -40.0f, 1e-6f);     // the bottom of the scale
    CHECK_NEAR(st.volts, 14.0f, 0.001f);        // charging
    CHECK(broadcastApply(st, mk(0x015u, 2u, 0x64u, 0xFEu), 1200u));
    CHECK_NEAR(st.ambientC, 10.0f, 1e-6f);
    CHECK_NEAR(st.volts, 25.4f, 0.001f);        // implausible, but not a sentinel
    // 0xFE ambient is 87.0 C -- absurd for outside air and still not the
    // sentinel. Exact-match again; a `>= 0xFE` guard would blank a hot day in
    // a country this truck has been to.
    CHECK(broadcastApply(st, mk(0x015u, 2u, 0xFEu, 0x7Eu), 1300u));
    CHECK_NEAR(st.ambientC, 87.0f, 1e-6f);
    CHECK_EQ(st.ambientMs, 1300);

    // Sentinels, independently: stamp zeroed, value held, other half live.
    CHECK(broadcastApply(st, mk(0x015u, 2u, 0xFFu, 0x7Eu), 2000u));
    CHECK_EQ(st.ambientMs, 0);
    CHECK_NEAR(st.ambientC, 87.0f, 1e-6f);      // held; 0xFF would be 87.5
    CHECK_NEAR(st.volts, 12.6f, 0.001f);
    CHECK_EQ(st.voltsMs, 2000);

    CHECK(broadcastApply(st, mk(0x015u, 2u, 0x50u, 0xFFu), 3000u));
    CHECK_EQ(st.voltsMs, 0);
    CHECK_NEAR(st.volts, 12.6f, 0.001f);        // held; 0xFF would be 25.5
    CHECK_EQ(st.ambientMs, 3000);

    // BOTH sentinels still returns TRUE, unlike 0x002. Documented rather than
    // asserted-as-good: on 0x002 both-unavailable is the engine-off frame and
    // worth not counting, whereas nothing on this id distinguishes
    // both-absent from a frame we simply have no other field of. Recorded here
    // so a change to it is a deliberate change and not a surprise.
    const uint32_t decodedBefore = st.decoded;
    CHECK(broadcastApply(st, mk(0x015u, 2u, 0xFFu, 0xFFu), 4000u));
    CHECK_EQ(st.ambientMs, 0);
    CHECK_EQ(st.voltsMs, 0);
    CHECK_EQ(st.decoded, decodedBefore + 1);

    expectShortRejected(mk(0x015u, 2u, 0x50u, 0x7Eu), 2u);
  }

  // -------------------------------------------------------------------------
  // 0x02C -- coolant temperature, in BYTE 1
  //
  // Byte 0 is not ours, and the sentinel here is the cheapest plausible lie in
  // the file: 0xFF - 40 is 215 C, which is a number a driver would believe and
  // act on. It has to blank.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    CHECK(broadcastApply(st, mk(0x02Cu, 8u, 0xFFu, 0x78u), 1000u));
    CHECK_EQ(st.coolantC, 80);                  // 120 - 40, normal running
    CHECK_EQ(st.coolantMs, 1000);
    // Byte 0 all-ones changed nothing above, which is the point of putting it
    // there: an off-by-one in the byte index reads 215 C instead of 80.
    CHECK(broadcastApply(st, mk(0x02Cu, 2u, 0x00u, 0x28u), 1100u));
    CHECK_EQ(st.coolantC, 0);
    CHECK(broadcastApply(st, mk(0x02Cu, 2u, 0x00u, 0x00u), 1200u));
    CHECK_EQ(st.coolantC, -40);                 // signed, and the offset proves it
    CHECK(broadcastApply(st, mk(0x02Cu, 2u, 0x00u, 0xFEu), 1300u));
    CHECK_EQ(st.coolantC, 214);                 // not the sentinel; a real 214
    CHECK_EQ(st.coolantMs, 1300);

    CHECK(broadcastApply(st, mk(0x02Cu, 2u, 0x00u, 0xFFu), 2000u));
    CHECK_EQ(st.coolantMs, 0);
    CHECK_EQ(st.coolantC, 214);                 // held, never the 215
    CHECK(st.coolantC != 215);

    expectShortRejected(mk(0x02Cu, 2u, 0x00u, 0x78u), 2u);
  }

  // -------------------------------------------------------------------------
  // 0x1A7 -- instantaneous fuel flow. Already a RATE.
  //
  // L/h = raw / 160, per CLAUDE.md, and the two things that must hold are that
  // 0xFFFF never becomes 409 L/h and that ZERO is accepted as the real reading
  // it is. Zero is overrun fuel cut-off on a trailing engine: it burns nothing,
  // and if it were treated as "no data" the integrator would hold the previous
  // cruising rate all the way down a mountain pass and invent the fuel.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    CHECK(broadcastApply(st, mk(0x1A7u, 8u, 0x00u, 0xA0u), 1000u));
    CHECK_EQ(st.flowRaw, 160);                  // exactly 1.0 L/h
    CHECK_EQ(st.flowMs, 1000);
    CHECK(broadcastApply(st, mk(0x1A7u, 2u, 0x06u, 0x40u), 1100u));
    CHECK_EQ(st.flowRaw, 1600);                 // 10.0 L/h
    // UI_FLOW_FULL_LPH is 50 L/h and it is a hard CLAMP on the bar, so the raw
    // count at full scale is worth stating: 50 * 160 = 8000 = 0x1F40.
    CHECK(broadcastApply(st, mk(0x1A7u, 2u, 0x1Fu, 0x40u), 1200u));
    CHECK_EQ(st.flowRaw, 8000);
    CHECK_NEAR((float)st.flowRaw / 160.0f, 50.0f, 1e-6f);

    // Zero: accepted, stamped, integrated as the zero it is.
    CHECK(broadcastApply(st, mk(0x1A7u, 2u, 0x00u, 0x00u), 1300u));
    CHECK_EQ(st.flowRaw, 0);
    CHECK_EQ(st.flowMs, 1300);

    // The sentinel: whole frame rejected, stamp AND value untouched. Same
    // policy as the odometer and for the same reason -- this feeds an
    // integrator, so the consumer must see a gap and drop the interval rather
    // than be handed a blanked stamp it would read as "never seen".
    CHECK(broadcastApply(st, mk(0x1A7u, 2u, 0x06u, 0x40u), 1400u));
    const uint32_t heldMs = st.flowMs;
    CHECK(!broadcastApply(st, mk(0x1A7u, 2u, 0xFFu, 0xFFu), 2000u));
    CHECK_EQ(st.flowMs, heldMs);
    CHECK_EQ(st.flowRaw, 1600);
    CHECK(st.flowRaw != 0xFFFFu);               // 409.6 L/h never got in

    // 0xFFFE decodes: exact-match guard, third time. 409.5 L/h is nonsense on
    // a 3.0 CRD, but it is the module's nonsense and not ours to filter.
    CHECK(broadcastApply(st, mk(0x1A7u, 2u, 0xFFu, 0xFEu), 3000u));
    CHECK_EQ(st.flowRaw, 0xFFFEu);
    CHECK_EQ(st.flowMs, 3000);

    expectShortRejected(mk(0x1A7u, 2u, 0x06u, 0x40u), 2u);
  }

  // -------------------------------------------------------------------------
  // 0x210 -- the panel dimmer. NO SENTINEL, and that is deliberate.
  //
  // Byte 1 is the wheel, byte 0 is kept because it is 0x03 in every recorded
  // sample and a day when it is not is a day somebody wants to know about.
  // There is no unavailable marker: every byte value is a legal wheel
  // position, 0xFF included, and it is broadcastBacklightPermille() that
  // decides what to do with the ones above the wheel's top. Blanking a dimmer
  // reading would fail DARK, which is the one direction this instrument is not
  // allowed to fail in.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    CHECK(broadcastApply(st, mk(0x210u, 8u, 0x03u, 0x40u), 1000u));
    CHECK_EQ(st.dimmerRaw, 0x40);
    CHECK_EQ(st.dimmerByte0, 0x03);
    CHECK_EQ(st.dimmerMs, 1000);

    // Every one of the 256 byte values is accepted and stored verbatim,
    // including 0x00 (bottom of the wheel AND all-lights-off, the ambiguity
    // 0x1C8 exists to resolve) and 0xFF.
    bool allAccepted = true;
    bool allVerbatim = true;
    for (unsigned v = 0u; v <= 255u; ++v) {
      if (!broadcastApply(st, mk(0x210u, 2u, 0xA5u, (uint8_t)v), 2000u + v)) {
        allAccepted = false;
      }
      if (st.dimmerRaw != (uint8_t)v || st.dimmerMs != 2000u + v) {
        allVerbatim = false;
      }
    }
    CHECK(allAccepted);
    CHECK(allVerbatim);
    CHECK_EQ(st.dimmerByte0, 0xA5);             // captured whatever it is
    CHECK_EQ(st.dimmerRaw, 0xFF);

    expectShortRejected(mk(0x210u, 2u, 0x03u, 0x40u), 2u);
  }

  // -------------------------------------------------------------------------
  // 0x1C8 -- exterior lights, and the escape hatch for a one-sample guess
  //
  // `lightsOn = (byte0 == 0x02)` is explicitly an UNVERIFIED guess in
  // CLAUDE.md's open list, which is why lightsRaw is stored alongside the
  // verdict: if 0x02 is the wrong test, the backlight log says so in numbers
  // instead of the screen misbehaving quietly on a bright afternoon. Test the
  // verdict AND the raw, because the raw is the thing that makes the guess
  // cheap to settle.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    CHECK(st.lightsOn);                         // the fail-safe default
    CHECK_EQ(st.lightsMs, 0);

    CHECK(broadcastApply(st, mk(0x1C8u, 8u, 0x02u), 1000u));
    CHECK(st.lightsOn);
    CHECK_EQ(st.lightsRaw, 0x02);
    CHECK_EQ(st.lightsMs, 1000);

    // Exactly one value means on; all 255 others mean off, and every one of
    // them is still recorded and still counted as decoded -- "lights off" is
    // an answer, not an absence.
    bool allAccepted = true;
    bool verdictRight = true;
    bool rawKept = true;
    for (unsigned v = 0u; v <= 255u; ++v) {
      if (!broadcastApply(st, mk(0x1C8u, 1u, (uint8_t)v), 2000u + v)) {
        allAccepted = false;
      }
      if (st.lightsOn != (v == 0x02u)) {
        verdictRight = false;
      }
      if (st.lightsRaw != (uint8_t)v || st.lightsMs != 2000u + v) {
        rawKept = false;
      }
    }
    CHECK(allAccepted);
    CHECK(verdictRight);
    CHECK(rawKept);
    CHECK(!st.lightsOn);                        // finished on 0xFF
    CHECK_EQ(st.decoded, 257);

    expectShortRejected(mk(0x1C8u, 1u, 0x02u), 1u);
  }

  // -------------------------------------------------------------------------
  // 0x159 -- the tank level, and a VALIDITY FLAG where every other id has a
  // sentinel.
  //
  // DLC 6 at about a second: 1450 frames in the 2026 drive log, DLC 6 in 100 %
  // of them, period mean 1.0039 s and median 1.000. byte 5 is the level at
  // 0.5 L per count, and BYTE 4 IS WHAT SAYS WHETHER TO BELIEVE IT.
  //
  // THE ALL-ONES TEST USED EVERYWHERE ELSE IN THIS FILE COULD NOT WORK HERE,
  // and that is the whole reason this block exists. On this id `b0 = FE` with
  // `b1:b3 = FF FF FF` is the NORMAL payload, so the usual sentinel rule would
  // reject every frame the module has ever sent. Measured across every log in
  // the repository, with no exceptions anywhere: `b4 == 0x01` <=> `b5 == 0xFF`,
  // and `b4 == 0x00` <=> b5 is a real reading.
  //
  // This REPLACED the CCN 0x2103 poll rather than joining it, so there is no
  // second source to fall back to and no second opinion to notice a wrong
  // answer. The gate is the only thing between the driver and a frozen tank.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);

    // A real frame out of the 2026 drive, logged the morning after a fill:
    // raw 140, which is 70.0 L of a 77.6 L tank.
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0xFEu, 0xFFu, 0xFFu, 0xFFu,
                                0x00u, 0x8Cu), 1000u));
    CHECK_EQ(st.fuelRaw, 140);
    CHECK_EQ(st.fuelMs, 1000);
    CHECK_EQ(st.decoded, 1);
    // THE x0.5 IS THE CALLER'S, exactly as the compass's x2 is: the wire value
    // stays the wire value and Snapshot scales it on the way to the glass. A
    // decoder that helpfully stored 70 here would halve the tank twice.
    CHECK(st.fuelRaw != 70u);

    expectShortRejected(mk(0x159u, 6u, 0xFEu, 0xFFu, 0xFFu, 0xFFu,
                           0x00u, 0x8Cu), 6u);

    // THE B4 GATE, FIRST DIRECTION. `FE FF FF FF 01 FF` is the real key-off
    // payload -- 0x159 keeps transmitting straight through ignition-off and
    // simply flips b4 -- and it must ZERO THE STAMP while LEAVING THE VALUE.
    //
    // Both halves are asserted because that pairing IS the convention: a
    // zeroed stamp is how Snapshot is told to draw dashes rather than
    // hold-and-dim, and leaving the value alone is what distinguishes an
    // explicit unavailable from a dropped frame -- the one costs the reading,
    // the other must not.
    const uint32_t decodedBefore = st.decoded;
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0xFEu, 0xFFu, 0xFFu, 0xFFu,
                                0x01u, 0xFFu), 2000u));
    CHECK_EQ(st.fuelMs, 0);
    CHECK_EQ(st.fuelRaw, 140);
    // Still a frame we UNDERSTOOD. "The module says it has no tank reading" is
    // an answer, not noise, so it counts as decoded -- unlike the all-sentinel
    // 0x002 frame, which carries nothing at all and returns false.
    CHECK_EQ(st.decoded, decodedBefore + 1u);

    // ...AND BACK. A valid frame restores the stamp, so the dashes are a state
    // the readout can leave again rather than a latch.
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0xFEu, 0xFFu, 0xFFu, 0xFFu,
                                0x00u, 0x8Cu), 3000u));
    CHECK_EQ(st.fuelMs, 3000);
    CHECK_EQ(st.fuelRaw, 140);

    // THE GATE IS `!= 0`, NOT `== 1`. 0x01 is the only unavailable code any
    // log contains, but the decoder refuses every non-zero b4 and that is the
    // right shape: an unknown flag value is not a licence to believe byte 5.
    // Each iteration re-arms with a valid frame first, so a gate that failed
    // OPEN shows up as a stamp that SURVIVED rather than as one that was never
    // set -- and b5 here is 0x63, which would be visible if it were stored.
    bool gateHolds = true;
    for (unsigned b4 = 1u; b4 <= 255u; ++b4) {
      if (!broadcastApply(st, mk(0x159u, 6u, 0xFEu, 0xFFu, 0xFFu, 0xFFu,
                                 0x00u, 0x8Cu), 4000u + b4)) {
        gateHolds = false;
      }
      if (st.fuelMs != 4000u + b4) {
        gateHolds = false;
      }
      if (!broadcastApply(st, mk(0x159u, 6u, 0xFEu, 0xFFu, 0xFFu, 0xFFu,
                                 (uint8_t)b4, 0x63u), 4500u + b4)) {
        gateHolds = false;
      }
      if (st.fuelMs != 0u || st.fuelRaw != 140u) {
        gateHolds = false;
      }
    }
    CHECK(gateHolds);

    // B5 == 0xFF WITH B4 == 0x00 IS ACCEPTED, AND THAT IS THE POINT OF THIS
    // TEST. Under 0.5 L per count raw 255 is 127.5 L against a 77.6 L tank, so
    // it is out of range -- and it is still stored, because b4 is the flag the
    // module actually sets and range is not what this decoder is for. Adding a
    // `b5 == 0xFF` rejection of its own would blank the gauge on the strength
    // of an assumption about the tank, and a future full tank on a re-scaled
    // or different truck is entitled to a high raw.
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0xFEu, 0xFFu, 0xFFu, 0xFFu,
                                0x00u, 0xFFu), 6000u));
    CHECK_EQ(st.fuelRaw, 255);
    CHECK_EQ(st.fuelMs, 6000);

    // Every raw is stored VERBATIM when b4 says it is real. Valid raws
    // observed across the repository's logs are 25-30 (2019), 77, 138-142 (the
    // 2026 drive) and 152-154 (2015); 154 is the largest anywhere and is
    // 77.0 L against a 77.6 L tank, which is what named the unit.
    bool allVerbatim = true;
    for (unsigned v = 0u; v <= 255u; ++v) {
      if (!broadcastApply(st, mk(0x159u, 6u, 0xFEu, 0xFFu, 0xFFu, 0xFFu,
                                 0x00u, (uint8_t)v), 7000u + v)) {
        allVerbatim = false;
      }
      if (st.fuelRaw != (uint8_t)v || st.fuelMs != 7000u + v) {
        allVerbatim = false;
      }
    }
    CHECK(allVerbatim);

    // THE TWO REAL TRANSITIONS, replayed frame by frame in the order the logs
    // hold them.
    //
    // First, ignition-off out of scripts/playback.log. b0 changes across it
    // too -- 0xFE running, 0x00 off -- which is exactly why nothing here reads
    // it: b4 is the only byte that decides anything.
    broadcastClear(st);
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0xFEu, 0xFFu, 0xFFu, 0xFFu,
                                0x00u, 0x8Cu), 10000u));
    CHECK_EQ(st.fuelRaw, 140);
    CHECK_EQ(st.fuelMs, 10000);
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0x00u, 0xFFu, 0xFFu, 0xFFu,
                                0x01u, 0xFFu), 11000u));
    CHECK_EQ(st.fuelMs, 0);
    CHECK_EQ(st.fuelRaw, 140);
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0x00u, 0xFFu, 0xFFu, 0xFFu,
                                0x01u, 0xFFu), 12000u));
    CHECK_EQ(st.fuelMs, 0);

    // And the reverse, out of the radio-ignition-on capture: three unavailable
    // frames, then `05 FF FF FF 00 4D` the moment the key reaches ON. Raw 77
    // is 38.5 L, about half a tank.
    for (unsigned k = 0u; k < 3u; ++k) {
      CHECK(broadcastApply(st, mk(0x159u, 6u, 0x05u, 0xFFu, 0xFFu, 0xFFu,
                                  0x01u, 0xFFu), 13000u + k * 1000u));
      CHECK_EQ(st.fuelMs, 0);
    }
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0x05u, 0xFFu, 0xFFu, 0xFFu,
                                0x00u, 0x4Du), 16000u));
    CHECK_EQ(st.fuelRaw, 77);
    CHECK_EQ(st.fuelMs, 16000);

    // BYTES 0-3 ARE INERT. They are not constant on the wire -- b0 alone is
    // 0xFE, 0x00 and 0x05 across the three captures -- so a decode that read
    // any of them would drift with the key position. Replayed at the SAME
    // nowMs, so the stamp is identical too and sameValues() covers everything.
    BroadcastState untouched = st;
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0x00u, 0x00u, 0x00u, 0x00u,
                                0x00u, 0x4Du), 16000u));
    CHECK(sameValues(st, untouched));
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0xA5u, 0x5Au, 0xA5u, 0x5Au,
                                0x00u, 0x4Du), 16000u));
    CHECK(sameValues(st, untouched));
    // Bytes 6 and 7 are past this frame's DLC of 6 and must be inert for the
    // same reason, whatever the harness leaves in them.
    CHECK(broadcastApply(st, mk(0x159u, 6u, 0x05u, 0xFFu, 0xFFu, 0xFFu,
                                0x00u, 0x4Du, 0xFFu, 0xFFu), 16000u));
    CHECK(sameValues(st, untouched));
  }

  // -------------------------------------------------------------------------
  // 0x1B4 -- the compass. NO SENTINEL AT ALL, and one nonsense value refused.
  //
  // byte 0 is the module's own 8-way octant enum, byte 1 the magnetic heading
  // at 2 deg/count, byte 2 a status that reads 0x07 in all 2953 frames of the
  // timestamped drive log AND all 1584 of an independent 2019 session. byte 1
  // is observed 0..178 and 0xFF NEVER APPEARS, so unlike 0x002, 0x014 and
  // 0x1A7 this frame carries no unavailable marker and there is nothing here
  // to blank.
  //
  // So the only thing refusable is a nonsensical OCTANT, and it is refused on
  // ignitionFrom()'s rule rather than on a measurement: no value above 7 has
  // ever been seen, and a confidently wrong "NE" on the glass is worse than
  // dashes. Both halves get asserted, because "accepts everything except one
  // impossible byte" is a claim that fails silently in either direction.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    // Due east. Raw 45 is 90 deg, and round(90/45) mod 8 is 2, which is E --
    // the octant is ROUNDED on this module, not floored (99.46 % agreement
    // against floor binning's 41.65 %).
    CHECK(broadcastApply(st, mk(0x1B4u, 6u, 0x02u, 45u, 0x07u), 1000u));
    CHECK_EQ(st.headingOctant, 2);
    CHECK_EQ(st.headingRaw, 45);
    CHECK_EQ(st.headingStatus, 0x07);
    CHECK_EQ(st.headingMs, 1000);
    CHECK_EQ(st.decoded, 1);

    expectShortRejected(mk(0x1B4u, 6u, 0x02u, 45u, 0x07u), 3u);

    // Bytes 3-5 are 0x00 in every frame of both sessions and are not decoded.
    // A day when they are not must not move a value we do read.
    BroadcastState untouched = st;
    CHECK(broadcastApply(st, mk(0x1B4u, 6u, 0x02u, 45u, 0x07u,
                                0xFFu, 0xFFu, 0xFFu), 1000u));
    CHECK(sameValues(st, untouched));

    // An octant above 7 is refused OUTRIGHT and touches nothing -- not the
    // heading that arrived beside it on the same frame, not the shared stamp.
    // 8 is the off-by-one an enum grows into; 0xFF is what a module with
    // nothing to say would send.
    BroadcastState before = st;
    CHECK(!broadcastApply(st, mk(0x1B4u, 6u, 8u, 90u, 0x07u), 2000u));
    CHECK(sameValues(st, before));
    CHECK(!broadcastApply(st, mk(0x1B4u, 6u, 0xFFu, 90u, 0x07u), 2000u));
    CHECK(sameValues(st, before));
    CHECK_EQ(st.decoded, 2);
    CHECK_EQ(st.frames, before.frames + 2u);

    // byte 1 == 0xFF is ACCEPTED, and that is the point of this block. There
    // is no sentinel on this frame, so 0xFF is simply a raw above the observed
    // ceiling -- out of range, not "no value" -- and the decoder's job is to
    // report the wire value rather than to second-guess it. The x2 is the
    // CALLER's: Snapshot scales it, exactly as it does psi -> bar.
    bool allAccepted = true;
    bool allVerbatim = true;
    for (unsigned v = 0u; v <= 255u; ++v) {
      const uint8_t oct = (uint8_t)(v & 7u);
      if (!broadcastApply(st, mk(0x1B4u, 6u, oct, (uint8_t)v, 0x07u),
                          4000u + v)) {
        allAccepted = false;
      }
      if (st.headingRaw != (uint8_t)v || st.headingOctant != oct ||
          st.headingMs != 4000u + v) {
        allVerbatim = false;
      }
    }
    CHECK(allAccepted);
    CHECK(allVerbatim);
    CHECK_EQ(st.headingRaw, 0xFF);

    // Whatever byte 2 turns out to mean, it is stored. lightsRaw's reason: a
    // wrong guess about 0x07 then shows up in the log instead of misbehaving
    // quietly on the glass.
    CHECK(broadcastApply(st, mk(0x1B4u, 6u, 0x00u, 0u, 0x5Au), 8000u));
    CHECK_EQ(st.headingStatus, 0x5A);

    // A SECOND FRAME MOVES THE STAMP even when nothing else changed. The
    // heading is bit-for-bit static when the car is stopped -- 196 of 196
    // consecutive stationary sample pairs showed exactly 0.0 deg of delta --
    // so an unchanged value with a moving stamp is the normal case at a set of
    // lights, and the stamp is the only thing separating a parked compass from
    // a dead one.
    CHECK(broadcastApply(st, mk(0x1B4u, 6u, 0x02u, 45u, 0x07u), 9000u));
    CHECK_EQ(st.headingMs, 9000);
    CHECK(broadcastApply(st, mk(0x1B4u, 6u, 0x02u, 45u, 0x07u), 9500u));
    CHECK_EQ(st.headingRaw, 45);
    CHECK_EQ(st.headingOctant, 2);
    CHECK_EQ(st.headingMs, 9500);
  }

  // -------------------------------------------------------------------------
  // 0x2C0 -- the accelerator pedal, and the worst lie this decoder could tell
  //
  // With the engine stopped this frame is all 0xFF but for a trailing 0x00, and
  // byte 2 of that reads as a pedal pressed HARDER THAN THE FLOOR. Since
  // ACCEL_LAUNCH_MS the kickdown flag is what ARMS an acceleration run rather
  // than what decorates one, so a leaked 0xFF would arm a launch on a truck
  // sitting on a driveway with the key out.
  //
  // Three guards, and each is tested in ISOLATION as well as together --
  // because the all-0xFF frame trips all three at once, so testing only that
  // frame would leave two of them as dead code nobody would notice losing.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    // A good frame. byte7 bit 7 set = engine running (7624 capture frames at
    // 569-4158 rpm), byte 6 the byte-identical copy of byte 2, byte 3 the
    // half-scale second track, bytes 0/1/5 always 0xFF and carrying nothing.
    CHECK(broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, 0x5Au, 0x2Du,
                                0x00u, 0xFFu, 0x5Au, 0x80u), 1000u));
    CHECK_EQ(st.pedalRaw, 0x5A);
    CHECK_EQ(st.pedalMs, 1000);

    // The released anchor. 0x17 is proven by a hardware idle switch: across
    // 3767 capture frames with bit 2 or 3 of byte 7 set, byte 2 was exactly
    // 0x17, 3767 times out of 3767. Here with the idle switch bit also set,
    // which is what those frames look like.
    CHECK(broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, BC_PEDAL_RELEASED,
                                0x0Bu, 0x00u, 0xFFu, BC_PEDAL_RELEASED,
                                0x84u), 1100u));
    CHECK_EQ(st.pedalRaw, BC_PEDAL_RELEASED);
    CHECK_EQ(broadcastPedalPercent(st.pedalRaw), 0);

    // The floor, held flat through a 1509-4158 rpm pull for 54 consecutive
    // frames.
    CHECK(broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, BC_PEDAL_FLOORED,
                                0x75u, 0x00u, 0xFFu, BC_PEDAL_FLOORED,
                                0x80u), 1200u));
    CHECK_EQ(st.pedalRaw, BC_PEDAL_FLOORED);
    CHECK_EQ(broadcastPedalPercent(st.pedalRaw), 100);
    const uint32_t goodMs = st.pedalMs;
    const uint8_t goodRaw = st.pedalRaw;

    // Guard 1 alone -- engine flag clear, everything else valid and a
    // perfectly ordinary pedal value. This is the frame the car sends with the
    // ignition on and the engine off (335 capture frames at 0-1039 rpm).
    CHECK(!broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, 0x5Au, 0x2Du,
                                 0x00u, 0xFFu, 0x5Au, 0x00u), 2000u));
    CHECK_EQ(st.pedalRaw, goodRaw);
    CHECK_EQ(st.pedalMs, goodMs);

    // Guard 2 alone -- the redundant copy disagrees. A dual-track APPS whose
    // two tracks differ is a frame nobody should act on, whatever it says.
    CHECK(!broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, 0x5Au, 0x2Du,
                                 0x00u, 0xFFu, 0x5Bu, 0x80u), 2100u));
    CHECK_EQ(st.pedalRaw, goodRaw);
    CHECK_EQ(st.pedalMs, goodMs);

    // Guard 3 alone -- byte 2 is 0xFF with byte 6 agreeing and the engine flag
    // set. Nothing but the explicit 0xFF test catches this one, and 0xFF is
    // 21 counts past the floor.
    CHECK(!broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, 0xFFu, 0x7Fu,
                                 0x00u, 0xFFu, 0xFFu, 0x80u), 2200u));
    CHECK_EQ(st.pedalRaw, goodRaw);
    CHECK_EQ(st.pedalMs, goodMs);
    CHECK(st.pedalRaw != 0xFFu);

    // The 0xFF test is EXACT-MATCH, like every other sentinel guard here, and
    // that is worth stating because on this one id the consequence is
    // different. 0xEB..0xFE are all past the floor and all accepted, and every
    // one of them reads as kickdown -- so what keeps a corrupt frame from
    // arming a launch is not the value test but the other two guards: a single
    // flipped bit lands in byte 2 or byte 6, not both, and the copies then
    // disagree. Pinned as CURRENT BEHAVIOUR rather than endorsed: if two
    // agreeing tracks ever read 0xFE, this decoder believes them.
    CHECK(broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, 0xFEu, 0x7Fu,
                                0x00u, 0xFFu, 0xFEu, 0x80u), 2300u));
    CHECK_EQ(st.pedalRaw, 0xFE);
    CHECK_EQ(broadcastPedalPercent(st.pedalRaw), 100);
    // Put a real reading back, so the rejections below are provably rejections.
    CHECK(broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, goodRaw, 0x75u,
                                0x00u, 0xFFu, goodRaw, 0x80u), goodMs));
    CHECK_EQ(st.pedalRaw, goodRaw);
    CHECK_EQ(st.pedalMs, goodMs);

    // And the real engine-off frame: all 0xFF but a trailing 0x00.
    CHECK(!broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, 0xFFu, 0xFFu,
                                 0xFFu, 0xFFu, 0xFFu, 0x00u), 3000u));
    CHECK_EQ(st.pedalRaw, goodRaw);
    CHECK_EQ(st.pedalMs, goodMs);

    // Byte 3, the half-scale partner, is NOT decoded and not cross-checked.
    // Garbage there must change nothing -- stated so that a future decision to
    // start validating it is a decision rather than a discovery.
    BroadcastState before = st;
    CHECK(broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, goodRaw, 0x00u,
                                0xFFu, 0xFFu, goodRaw, 0xFFu), goodMs));
    CHECK(sameValues(st, before));

    // The DETENT GAP is not filtered, and should not be. Between 0xC0 and 0xE9
    // there is not one frame in 8089 -- the pedal sweeps to the hump and past
    // it is only ever observed on the floor. That emptiness is a property of
    // the CAR, so the decoder accepts those values verbatim; inventing a
    // rejection would hide a real change in the pedal map behind a guard.
    bool gapVerbatim = true;
    for (unsigned v = 0xC0u; v <= 0xE9u; ++v) {
      if (!broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, (uint8_t)v,
                                 0x00u, 0x00u, 0xFFu, (uint8_t)v, 0x80u),
                          4000u + v) ||
          st.pedalRaw != (uint8_t)v) {
        gapVerbatim = false;
      }
    }
    CHECK(gapVerbatim);

    // The kickdown threshold sits INSIDE that empty gap, which is what makes it
    // an uncalibrated binary event rather than a percentage with a cut-off. No
    // reading the car has ever produced lies near it, so no reading can
    // straddle it and chatter.
    CHECK(BC_PEDAL_RELEASED < BC_PEDAL_DETENT);
    CHECK(BC_PEDAL_DETENT < BC_PEDAL_KICKDOWN);
    CHECK(BC_PEDAL_KICKDOWN <= BC_PEDAL_FLOORED);

    // This is the ONLY id needing all eight bytes, so its sweep is the widest.
    expectShortRejected(mk(0x2C0u, 8u, 0xFFu, 0xFFu, 0x5Au, 0x2Du,
                           0x00u, 0xFFu, 0x5Au, 0x80u), 8u);
  }

  // -------------------------------------------------------------------------
  // broadcastPedalPercent() across its whole input space
  //
  // Clamped at both ends because the top anchor comes from one wide-open run in
  // a 2015 capture and this car has never been logged past 0x37. The property
  // that matters is that it is monotone and bounded, so nothing downstream can
  // be handed 137%.
  // -------------------------------------------------------------------------
  {
    bool bounded = true;
    bool monotone = true;
    uint8_t prev = 0u;
    for (unsigned raw = 0u; raw <= 255u; ++raw) {
      const uint8_t pc = broadcastPedalPercent((uint8_t)raw);
      if (pc > 100u) {
        bounded = false;
      }
      if (pc < prev) {
        monotone = false;
      }
      prev = pc;
    }
    CHECK(bounded);
    CHECK(monotone);

    // Everything at or below the idle-switch anchor is zero, not a fraction of
    // a percent of travel that would make a resting foot look applied.
    bool zeroBelowAnchor = true;
    for (unsigned raw = 0u; raw <= BC_PEDAL_RELEASED; ++raw) {
      if (broadcastPedalPercent((uint8_t)raw) != 0u) {
        zeroBelowAnchor = false;
      }
    }
    CHECK(zeroBelowAnchor);
    bool hundredAtTop = true;
    for (unsigned raw = BC_PEDAL_FLOORED; raw <= 255u; ++raw) {
      if (broadcastPedalPercent((uint8_t)raw) != 100u) {
        hundredAtTop = false;
      }
    }
    CHECK(hundredAtTop);

    // Linear between the anchors, rounded to nearest: the midpoint of the
    // travel is 50, not 49 or 51.
    CHECK_EQ(broadcastPedalPercent(0x80u), 50);
    CHECK_EQ(broadcastPedalPercent(0x18u), 0);   // one count up is still 0
    CHECK_EQ(broadcastPedalPercent(0x19u), 1);
    // And this is why the percentage is not what gates a launch: the detent is
    // 80% and through it is 95%, so a threshold expressed as a percentage would
    // have to be placed inside a 20-point band that not one frame in 8089 lies
    // in -- pick 90% and you have picked a raw value the pedal never reports.
    // The binary >= BC_PEDAL_KICKDOWN test needs no such choice, which is the
    // whole argument for it. 0x37, which Broadcast.h gives as the most this car
    // had been logged at when the clamp was written, is only 15%.
    CHECK_EQ(broadcastPedalPercent(BC_PEDAL_DETENT), 80);
    CHECK_EQ(broadcastPedalPercent(BC_PEDAL_KICKDOWN), 95);
    CHECK_EQ(broadcastPedalPercent(0x37u), 15);
  }

  // -------------------------------------------------------------------------
  // broadcastBacklightPermille() -- swept over its ENTIRE input space
  //
  // 256 dimmer bytes x haveSignal x lightsOn x haveLights = 2048 combinations,
  // aggregated into a handful of property checks so the count stays readable.
  // The properties, in order of how much they would cost to get wrong:
  //
  //   1. FAIL BRIGHT. Nothing heard means full duty. An unreadable dash in
  //      daylight is worse than a bright one at night, and this is the only
  //      branch that runs when the link is down.
  //   2. BOUNDED. 0..1000 per mille, and never 0 -- the panel is never turned
  //      off by a dimmer reading, only dimmed to barely visible.
  //   3. ONE-DIRECTIONAL. The lights signal can only ever BRIGHTEN. A bus that
  //      never carries 0x1C8 must lose nothing, and a wrong guess about 0x02
  //      must not be able to darken the screen.
  //   4. SQUARE LAW. Fine control at the bottom of the wheel, which is the end
  //      that exists.
  // -------------------------------------------------------------------------
  {
    // 1. No signal: full, regardless of anything else the caller passes.
    bool failsBright = true;
    for (unsigned d = 0u; d <= 255u; ++d) {
      for (unsigned on = 0u; on < 2u; ++on) {
        for (unsigned hl = 0u; hl < 2u; ++hl) {
          if (broadcastBacklightPermille((uint8_t)d, false, on != 0u,
                                         hl != 0u) != 1000u) {
            failsBright = false;
          }
        }
      }
    }
    CHECK(failsBright);

    // 2. Bounded, and never off, across all 2048.
    bool bounded = true;
    for (unsigned d = 0u; d <= 255u; ++d) {
      for (unsigned hs = 0u; hs < 2u; ++hs) {
        for (unsigned on = 0u; on < 2u; ++on) {
          for (unsigned hl = 0u; hl < 2u; ++hl) {
            const uint16_t pm = broadcastBacklightPermille(
                (uint8_t)d, hs != 0u, on != 0u, hl != 0u);
            if (pm == 0u || pm > 1000u) {
              bounded = false;
            }
          }
        }
      }
    }
    CHECK(bounded);

    // 3. The lights signal brightens or does nothing, never dims -- and with
    //    the lights ON it is exactly equivalent to having no lights signal at
    //    all, which is what "the wheel keeps full control" means in code.
    bool brightenOnly = true;
    bool onEqualsAbsent = true;
    for (unsigned d = 0u; d <= 255u; ++d) {
      const uint16_t absent =
          broadcastBacklightPermille((uint8_t)d, true, false, false);
      const uint16_t absentOn =
          broadcastBacklightPermille((uint8_t)d, true, true, false);
      const uint16_t lightsOn =
          broadcastBacklightPermille((uint8_t)d, true, true, true);
      const uint16_t lightsOff =
          broadcastBacklightPermille((uint8_t)d, true, false, true);
      if (absent != absentOn || lightsOn != absent) {
        onEqualsAbsent = false;
      }
      if (lightsOn < absent || lightsOff < absent) {
        brightenOnly = false;
      }
    }
    CHECK(brightenOnly);
    CHECK(onEqualsAbsent);

    // Lights off means daylight, whatever the wheel says -- including at the
    // 0x00 that is ALSO the bottom of the wheel. That single byte reading two
    // opposite things is the entire reason 0x1C8 is decoded at all, so it gets
    // its own assertion rather than living inside the sweep above.
    CHECK_EQ(broadcastBacklightPermille(0x00u, true, false, true), 1000);
    CHECK_EQ(broadcastBacklightPermille(0x00u, true, true, true), 3);
    CHECK_EQ(broadcastBacklightPermille(0x00u, true, true, false), 3);

    // 4. Landmarks along the wheel, with the lights on so the daylight branch
    //    is out of the way. 0x00 is the bottom (barely visible, not off),
    //    0xA4 the top, and the top of the NIGHT range stops at 550 because a
    //    full-brightness panel at night mirrors in the windscreen.
    CHECK_EQ(broadcastBacklightPermille(0x00u, true, true, true), 3);
    CHECK_EQ(broadcastBacklightPermille(0x01u, true, true, true), 3);
    CHECK_EQ(broadcastBacklightPermille(0xA4u, true, true, true), 550);
    // Above the wheel's top but below the cabin landmark: clamped, not
    // extrapolated, so an unexpected byte cannot exceed the night ceiling.
    CHECK_EQ(broadcastBacklightPermille(0xA5u, true, true, true), 550);
    CHECK_EQ(broadcastBacklightPermille(0xC7u, true, true, true), 550);
    // 0xC8 and up is believed to be cabin lights on, which is brighter again
    // but still short of full.
    CHECK_EQ(broadcastBacklightPermille(0xC8u, true, true, true), 750);
    CHECK_EQ(broadcastBacklightPermille(0xFFu, true, true, true), 750);

    // Monotone non-decreasing over the whole byte range, so turning the wheel
    // up never makes the panel darker.
    bool monotone = true;
    uint16_t prev = 0u;
    for (unsigned d = 0u; d <= 255u; ++d) {
      const uint16_t pm = broadcastBacklightPermille((uint8_t)d, true, true,
                                                     true);
      if (pm < prev) {
        monotone = false;
      }
      prev = pm;
    }
    CHECK(monotone);

    // The curve is a CURVE. At the wheel's midpoint (0x52 of 0xA4) a linear
    // map would give the middle of the 3..550 band, about 276. The square law
    // gives 139, and the first quarter of the travel spans under an eighth of
    // the output -- which is the fine control at the dim end that the whole
    // mapping exists for.
    CHECK_EQ(broadcastBacklightPermille(0x52u, true, true, true), 139);
    CHECK(broadcastBacklightPermille(0x52u, true, true, true) < 276u);
    CHECK(broadcastBacklightPermille(0x29u, true, true, true) < 3u + 547u / 8u);
  }

  // -------------------------------------------------------------------------
  // The headline, in one place: HELD vs BLANKED, on the same signal
  //
  // These two cases are one line apart in the decoder and they are the reason
  // this suite exists. A DROPPED frame leaves both the value and its stamp
  // alone, so the UI holds the reading and dims it. An explicit sentinel zeroes
  // the stamp, so the UI shows dashes. Get them the same way round and you
  // either dash a perfectly good number or -- far worse -- display a
  // frozen-but-plausible one, which is the failure mode that gets BELIEVED.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    CHECK(broadcastApply(st, mk(0x002u, 4u, 0x07u, 0xD0u, 0x10u, 0x00u), 1000u));
    CHECK(broadcastApply(st, mk(0x02Cu, 2u, 0x00u, 0x78u), 1000u));
    CHECK(broadcastApply(st, mk(0x015u, 2u, 0x50u, 0x7Eu), 1000u));

    // Frames simply stop. Nothing is called; every stamp stays at 1000, so a
    // presenter comparing it against `now` sees an age and dims. The values
    // are all still there to be dimmed.
    CHECK_EQ(st.rpmMs, 1000);
    CHECK_EQ(st.speedMs, 1000);
    CHECK_EQ(st.coolantMs, 1000);
    CHECK_EQ(st.ambientMs, 1000);
    CHECK_EQ(st.voltsMs, 1000);
    CHECK_EQ(st.rpm, 2000);
    CHECK_EQ(st.coolantC, 80);

    // Now the modules speak up and say they have nothing. Same values, but
    // every stamp goes to 0 -- and 0 is not an age, it is "never", which is
    // what makes the presenter draw dashes instead of a dimmed 2000 rpm.
    CHECK(!broadcastApply(st, mk(0x002u, 4u, 0xFFu, 0xFFu, 0xFFu, 0xFFu), 5000u));
    CHECK(broadcastApply(st, mk(0x02Cu, 2u, 0x00u, 0xFFu), 5000u));
    CHECK(broadcastApply(st, mk(0x015u, 2u, 0xFFu, 0xFFu), 5000u));
    CHECK_EQ(st.rpmMs, 0);
    CHECK_EQ(st.speedMs, 0);
    CHECK_EQ(st.coolantMs, 0);
    CHECK_EQ(st.ambientMs, 0);
    CHECK_EQ(st.voltsMs, 0);
    // The values are untouched, which is fine and is the point: no consumer
    // may read them while the stamp says never.
    CHECK_EQ(st.rpm, 2000);
    CHECK_EQ(st.coolantC, 80);
  }

  // -------------------------------------------------------------------------
  // A short plausible burst of real bus traffic, for the counters
  //
  // 151 frames/s is what this thing actually sees, most of it ids we do not
  // decode. `frames` counts the wire and `decoded` counts the understood part,
  // and the ratio between them is printed at boot -- it is how a mis-set
  // adapter filter or a dead branch of the switch gets noticed at all, so the
  // two must not drift into counting the same thing.
  // -------------------------------------------------------------------------
  {
    BroadcastState st;
    broadcastClear(st);
    uint32_t expectDecoded = 0u;
    uint32_t expectFrames = 0u;
    for (uint32_t t = 100u; t <= 1000u; t += 100u) {
      // Nine understood frames...
      CHECK(broadcastApply(st, mk(0x000u, 1u, 0x85u), t));
      CHECK(broadcastApply(st, mk(0x002u, 4u, 0x07u, 0xD0u, 0x10u, 0x00u), t));
      CHECK(broadcastApply(st, mk(0x014u, 3u, 0x25u, 0x94u, 0x41u), t));
      CHECK(broadcastApply(st, mk(0x015u, 2u, 0x50u, 0x7Eu), t));
      CHECK(broadcastApply(st, mk(0x02Cu, 2u, 0x00u, 0x78u), t));
      CHECK(broadcastApply(st, mk(0x1A7u, 2u, 0x06u, 0x40u), t));
      CHECK(broadcastApply(st, mk(0x210u, 2u, 0x03u, 0x40u), t));
      CHECK(broadcastApply(st, mk(0x1C8u, 1u, 0x02u), t));
      CHECK(broadcastApply(st, mk(0x2C0u, 8u, 0xFFu, 0xFFu, 0x5Au, 0x2Du,
                                  0x00u, 0xFFu, 0x5Au, 0x80u), t));
      expectDecoded += 9u;
      expectFrames += 9u;
      // ...and four that are not ours, which is the realistic ratio.
      for (unsigned k = 0u; k < 4u; ++k) {
        CHECK(!broadcastApply(st, mk((uint16_t)(0x300u + k), 8u), t));
        ++expectFrames;
      }
    }
    CHECK_EQ(st.decoded, expectDecoded);
    CHECK_EQ(st.frames, expectFrames);
    CHECK(st.decoded < st.frames);

    // And the whole state is coherent at the end of it.
    CHECK_EQ(st.ignition, IGN_RUN);
    CHECK_EQ(st.rpm, 2000);
    CHECK_NEAR(st.speedKph, 32.0f, 1e-6f);
    CHECK_EQ(st.odometerDm, 2462785);
    CHECK_NEAR(st.ambientC, 0.0f, 1e-6f);
    CHECK_NEAR(st.volts, 12.6f, 0.001f);
    CHECK_EQ(st.coolantC, 80);
    CHECK_EQ(st.flowRaw, 1600);
    CHECK_EQ(st.dimmerRaw, 0x40);
    CHECK(st.lightsOn);
    CHECK_EQ(st.pedalRaw, 0x5A);
    CHECK_EQ(st.pedalMs, 1000);

    // One clear puts every one of those back to nothing. Same assertion as the
    // first block, but reached through the decoder instead of by hand -- this
    // is the shape the firmware's own link-recovery path takes.
    BroadcastState pristine;
    broadcastClear(pristine);
    broadcastClear(st);
    CHECK(sameValues(st, pristine));
    CHECK_EQ(st.frames, 0);
    CHECK_EQ(st.decoded, 0);
  }

TEST_MAIN_END()
