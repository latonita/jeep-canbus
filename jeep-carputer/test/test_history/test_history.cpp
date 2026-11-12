// Host tests for core/History.cpp.
//
// The arithmetic here is a min, a max and a modulo, and none of those is worth
// a suite on its own. What is worth one is the four ways this structure can be
// wrong while every bar on the chart still looks plausible:
//
//  1. THE EMPTY MARKER. A bucket nothing arrived in is lo > hi, and there is
//     no flag byte to cross-check it against. Get the clear wrong -- or
//     "repair" the inverted pair -- and the chart draws a full-height bar from
//     INT16_MIN to INT16_MAX, or a zero bar at the bottom of the band, which
//     on speed and flow reads as a perfectly ordinary stationary truck.
//
//  2. THE BOUNDARY. One bucket must open per HIST_BUCKET_MS, no more and no
//     fewer. Off by one and the window is 14 minutes or 16, which nothing on
//     the panel would ever contradict.
//
//  3. THE 49-DAY WRAP. Every timestamp here is a 32-bit uptime. A `<`
//     comparison anywhere in the advance freezes the ring for a full window
//     either side of the rollover -- a chart that stops moving while the
//     numbers beside it keep changing, once every seven weeks, on a device
//     nobody can attach a debugger to. The rollover test below is what earns
//     the unsigned arithmetic.
//
//  4. THE CAP. A gap of a window or more clears the ring rather than stepping
//     to it. Stepping reaches the same state, so the state alone cannot tell
//     the two apart -- the cheap path is identified here by headMs landing on
//     `nowMs` exactly rather than on a bucket-aligned multiple.
//
// Nothing here needs a car or a board.
#include "doctest_min.h"

#include "History.h"

// Every bucket of every parameter, asserted empty. Used wherever the claim is
// "nothing is in the ring" -- which is a claim about 180 buckets, not about
// the one the eye would check.
static void checkAllEmpty(const History& h) {
  for (unsigned p = 0u; p < (unsigned)HIST_COUNT; ++p) {
    for (unsigned a = 0u; a < HIST_BUCKETS; ++a) {
      float lo = -999.0f;
      float hi = -999.0f;
      CHECK(!historyRange(h, (uint8_t)p, (uint8_t)a, lo, hi));
      // And the out-params really were left alone, everywhere, not just in
      // the one spot-check further down.
      CHECK_EQ((int)lo, -999);
      CHECK_EQ((int)hi, -999);
    }
  }
}

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // The bands are the product decision this module carries, so they are
  // asserted by value. Changing one is meant to be a deliberate act that
  // breaks a test, not a quiet edit.
  // -------------------------------------------------------------------------
  {
    CHECK_EQ((int)HIST_COUNT, 4);
    CHECK_EQ((int)HIST_BUCKETS, 45);
    CHECK_EQ((int)HIST_BUCKET_MS, 20000);
    CHECK_EQ((int)HIST_WINDOW_MS, 900000);

    CHECK_NEAR(kHistBand[HIST_FLOW].lo, 0.0, 1e-6);
    CHECK_NEAR(kHistBand[HIST_FLOW].hi, 50.0, 1e-6);
    CHECK_NEAR(kHistBand[HIST_SPEED].lo, 0.0, 1e-6);
    CHECK_NEAR(kHistBand[HIST_SPEED].hi, 160.0, 1e-6);
    // 40, not 0 -- the deliberate trade documented in the header: a cold start
    // clips to the floor so that 70-105 gets the height it deserves.
    CHECK_NEAR(kHistBand[HIST_COOLANT].lo, 40.0, 1e-6);
    CHECK_NEAR(kHistBand[HIST_COOLANT].hi, 120.0, 1e-6);
    CHECK_NEAR(kHistBand[HIST_VOLTS].lo, 11.0, 1e-6);
    CHECK_NEAR(kHistBand[HIST_VOLTS].hi, 15.0, 1e-6);
  }

  // -------------------------------------------------------------------------
  // 1. A fresh ring is empty, and a push before the first tick is a no-op
  // -------------------------------------------------------------------------
  {
    History h;
    historyInit(h);
    CHECK(!h.started);
    checkAllEmpty(h);

    // Pushed into a ring with no clock. There is no bucket to put it in --
    // headMs is meaningless until a caller supplies one -- so it must be
    // dropped rather than landing in bucket 0 and being dated from whenever
    // the first tick happens to arrive.
    historyPush(h, HIST_SPEED, 42.0f);
    historyPush(h, HIST_FLOW, 7.0f);
    checkAllEmpty(h);

    historyTick(h, 1000u);
    CHECK(h.started);
    CHECK_EQ(h.head, 0);
    CHECK_EQ(h.headMs, 1000u);
    // Still empty: the first tick opened the bucket, it did not retrospectively
    // accept the samples that arrived before there was one.
    checkAllEmpty(h);
  }

  // -------------------------------------------------------------------------
  // 2. Several pushes into one bucket fold to the min and the max
  // -------------------------------------------------------------------------
  {
    History h;
    historyInit(h);
    historyTick(h, 1000u);

    historyPush(h, HIST_SPEED, 30.0f);
    historyPush(h, HIST_SPEED, 0.0f);
    historyPush(h, HIST_SPEED, 61.5f);
    historyPush(h, HIST_SPEED, 12.0f);

    float lo = -999.0f;
    float hi = -999.0f;
    CHECK(historyRange(h, HIST_SPEED, 0u, lo, hi));
    // The junction from the header comment: a mean would have said 26 km/h
    // and hidden both ends of it.
    CHECK_NEAR(lo, 0.0, 0.001);
    CHECK_NEAR(hi, 61.5, 0.001);

    // One parameter filling does not fill the others -- the arrays are
    // separate and the clear walks all of them.
    float f = -999.0f;
    float g = -999.0f;
    CHECK(!historyRange(h, HIST_FLOW, 0u, f, g));
    CHECK_EQ((int)f, -999);

    // A single sample is the degenerate range, not an empty bucket.
    historyPush(h, HIST_VOLTS, 14.4f);
    CHECK(historyRange(h, HIST_VOLTS, 0u, lo, hi));
    CHECK_NEAR(lo, 14.4, 0.001);
    CHECK_NEAR(hi, 14.4, 0.001);
  }

  // -------------------------------------------------------------------------
  // 3. Exactly one bucket boundary
  // -------------------------------------------------------------------------
  {
    History h;
    historyInit(h);
    historyTick(h, 1000u);
    historyPush(h, HIST_COOLANT, 88.0f);
    historyPush(h, HIST_COOLANT, 92.0f);

    // One millisecond short of the boundary is still the same bucket. This is
    // the half of the off-by-one that a "> " instead of a ">=" would break.
    historyTick(h, 1000u + HIST_BUCKET_MS - 1u);
    CHECK_EQ(h.head, 0);
    float lo = -999.0f;
    float hi = -999.0f;
    CHECK(historyRange(h, HIST_COOLANT, 0u, lo, hi));
    CHECK_NEAR(lo, 88.0, 0.001);
    CHECK_NEAR(hi, 92.0, 0.001);

    // And exactly on it is the next bucket.
    historyTick(h, 1000u + HIST_BUCKET_MS);
    CHECK_EQ(h.head, 1);
    CHECK_EQ(h.headMs, 1000u + HIST_BUCKET_MS);

    lo = -999.0f;
    hi = -999.0f;
    CHECK(!historyRange(h, HIST_COOLANT, 0u, lo, hi));   // the new head
    CHECK_EQ((int)lo, -999);

    CHECK(historyRange(h, HIST_COOLANT, 1u, lo, hi));    // what was the head
    CHECK_NEAR(lo, 88.0, 0.001);
    CHECK_NEAR(hi, 92.0, 0.001);

    // A push now lands in the NEW bucket and does not reach back into the old
    // one, which is the whole point of clearing on the way in.
    historyPush(h, HIST_COOLANT, 95.0f);
    CHECK(historyRange(h, HIST_COOLANT, 0u, lo, hi));
    CHECK_NEAR(lo, 95.0, 0.001);
    CHECK_NEAR(hi, 95.0, 0.001);
    CHECK(historyRange(h, HIST_COOLANT, 1u, lo, hi));
    CHECK_NEAR(hi, 92.0, 0.001);
  }

  // -------------------------------------------------------------------------
  // 4. A walk longer than the ring pushes the oldest sample out of the window
  // -------------------------------------------------------------------------
  {
    History h;
    historyInit(h);
    uint32_t t = 5000u;
    historyTick(h, t);
    // The sample that must not survive. 1.0 L/h is chosen because it is a
    // value no later bucket holds, so "it is gone" is checkable by exhaustion
    // rather than by trusting one index.
    historyPush(h, HIST_FLOW, 1.0f);

    for (unsigned k = 1u; k <= HIST_BUCKETS; ++k) {
      t += HIST_BUCKET_MS;
      historyTick(h, t);
      CHECK_EQ(h.head, k % HIST_BUCKETS);
      historyPush(h, HIST_FLOW, (float)k + 1.0f);
    }

    // HIST_BUCKETS + 1 buckets have been opened and the ring holds
    // HIST_BUCKETS of them, so every bucket in the window is accounted for by
    // the loop above and the 1.0 has nowhere left to be.
    for (unsigned a = 0u; a < HIST_BUCKETS; ++a) {
      float lo = -999.0f;
      float hi = -999.0f;
      CHECK(historyRange(h, HIST_FLOW, (uint8_t)a, lo, hi));
      const double want = (double)(HIST_BUCKETS - a) + 1.0;
      CHECK_NEAR(lo, want, 0.001);
      CHECK_NEAR(hi, want, 0.001);
      CHECK(lo > 1.5f);   // ...and none of them is the evicted sample
    }
  }

  // -------------------------------------------------------------------------
  // 5. A gap of a window or more clears the ring instead of stepping to it
  // -------------------------------------------------------------------------
  {
    History h;
    historyInit(h);
    historyTick(h, 1000u);
    historyPush(h, HIST_FLOW, 9.0f);
    historyPush(h, HIST_SPEED, 90.0f);
    historyPush(h, HIST_COOLANT, 90.0f);
    historyPush(h, HIST_VOLTS, 14.0f);

    // Exactly a window is the boundary case for the cap. It cannot distinguish
    // the two implementations by itself -- HIST_WINDOW_MS is a whole number of
    // buckets, so stepping 45 times lands on the same head and the same
    // headMs -- but the ring must be empty either way.
    historyTick(h, 1000u + HIST_WINDOW_MS);
    checkAllEmpty(h);
    CHECK_EQ(h.head, 0);

    // THIS is the one that names the cheap path. A gap of a window plus five
    // seconds would leave a stepping implementation at headMs = 1000 +
    // HIST_WINDOW_MS with five seconds still owed; the cap restarts the clock
    // at `nowMs` outright. Asserting headMs is the only observable difference
    // between "cleared the ring" and "spun 45 times to reach an empty ring",
    // and the reason it matters is cost, not correctness: an hour parked is
    // 180 iterations, and a nowMs that has come round the 49-day clock is
    // millions, on the UI tick.
    historyInit(h);
    historyTick(h, 1000u);
    historyPush(h, HIST_SPEED, 77.0f);
    const uint32_t late = 1000u + HIST_WINDOW_MS + 5000u;
    historyTick(h, late);
    checkAllEmpty(h);
    CHECK_EQ(h.head, 0);
    CHECK_EQ(h.headMs, late);

    // An hour parked, which is the case the cap was written for.
    historyInit(h);
    historyTick(h, 1000u);
    historyPush(h, HIST_SPEED, 77.0f);
    historyTick(h, 1000u + 3600000u);
    checkAllEmpty(h);
    CHECK_EQ(h.headMs, 1000u + 3600000u);

    // And the ring is usable immediately afterwards rather than needing
    // another window to recover.
    historyPush(h, HIST_SPEED, 5.0f);
    float lo = -999.0f;
    float hi = -999.0f;
    CHECK(historyRange(h, HIST_SPEED, 0u, lo, hi));
    CHECK_NEAR(lo, 5.0, 0.001);
  }

  // -------------------------------------------------------------------------
  // 6. THE 49-DAY WRAP
  //
  // The ring is started just short of the rollover and walked across it one
  // bucket at a time. Exactly one bucket must open per HIST_BUCKET_MS, before
  // the wrap and after it, and a sample pushed before the wrap must still be
  // readable at its age afterwards.
  // -------------------------------------------------------------------------
  {
    History h;
    historyInit(h);

    const uint32_t t0 = 0xFFFFFF00u;   // 256 ms short of the rollover
    historyTick(h, t0);
    CHECK_EQ(h.headMs, t0);
    CHECK_EQ(h.head, 0);
    // The marker that has to survive the wrap.
    historyPush(h, HIST_SPEED, 100.0f);

    uint32_t t = t0;
    bool wrapped = false;
    for (unsigned k = 1u; k <= 8u; ++k) {
      const uint32_t prev = t;
      // One millisecond short first: the boundary test must be as exact on the
      // far side of the rollover as it is anywhere else.
      historyTick(h, prev + HIST_BUCKET_MS - 1u);
      CHECK_EQ(h.head, (k - 1u) % HIST_BUCKETS);

      t = prev + HIST_BUCKET_MS;       // wraps through zero on the first turn
      if (t < prev) {
        wrapped = true;
      }
      historyTick(h, t);
      // Exactly one bucket per interval, across the seam. A signed or `<`
      // comparison anywhere in the advance stops the ring dead here.
      CHECK_EQ(h.head, k % HIST_BUCKETS);
      CHECK_EQ(h.headMs, t);
      historyPush(h, HIST_SPEED, (float)(60u + k));
    }
    // The arithmetic really did cross zero -- otherwise this whole block is
    // testing an ordinary walk with big numbers.
    CHECK(wrapped);

    // The pre-wrap marker, eight buckets back, still reads as itself.
    float lo = -999.0f;
    float hi = -999.0f;
    CHECK(historyRange(h, HIST_SPEED, 8u, lo, hi));
    CHECK_NEAR(lo, 100.0, 0.001);
    CHECK_NEAR(hi, 100.0, 0.001);
    // And the buckets written after it are where they should be.
    CHECK(historyRange(h, HIST_SPEED, 0u, lo, hi));
    CHECK_NEAR(lo, 68.0, 0.001);
    CHECK(historyRange(h, HIST_SPEED, 7u, lo, hi));
    CHECK_NEAR(lo, 61.0, 0.001);
  }

  // -------------------------------------------------------------------------
  // 7. Clamping. A wild value must not wrap round into a plausible one
  // -------------------------------------------------------------------------
  {
    History h;
    historyInit(h);
    historyTick(h, 1000u);

    historyPush(h, HIST_FLOW, 1e9f);
    historyPush(h, HIST_FLOW, -1e9f);

    float lo = -999.0f;
    float hi = -999.0f;
    CHECK(historyRange(h, HIST_FLOW, 0u, lo, hi));
    // INT16_MIN/INT16_MAX tenths. The failure this guards is not the clipping
    // -- a caller sending 1e9 L/h is already wrong -- it is the WRAPPED
    // reading: a 1e9 truncated into an int16_t can come out as any value at
    // all, including a completely ordinary 12.3 L/h that nothing downstream
    // would question.
    CHECK_NEAR(lo, -3276.8, 0.05);
    CHECK_NEAR(hi, 3276.7, 0.05);
    CHECK(lo < hi);   // still a range, and still the right way round

    // The ordinary value alongside them is unaffected, so the clamp did not
    // reach past the sample it was applied to.
    historyPush(h, HIST_SPEED, 511.98f);   // 0xFFFE, a legitimate reading
    CHECK(historyRange(h, HIST_SPEED, 0u, lo, hi));
    CHECK_NEAR(lo, 512.0, 0.06);
    CHECK_NEAR(hi, 512.0, 0.06);
  }

  // -------------------------------------------------------------------------
  // 8. Rounding to the stored tenth, both ways
  // -------------------------------------------------------------------------
  {
    History h;
    historyInit(h);
    historyTick(h, 1000u);
    historyPush(h, HIST_VOLTS, 12.34f);

    float lo = -999.0f;
    float hi = -999.0f;
    CHECK(historyRange(h, HIST_VOLTS, 0u, lo, hi));
    CHECK_NEAR(lo, 12.3, 0.001);
    CHECK_NEAR(hi, 12.3, 0.001);

    historyTick(h, 1000u + HIST_BUCKET_MS);
    historyPush(h, HIST_VOLTS, 12.36f);
    CHECK(historyRange(h, HIST_VOLTS, 0u, lo, hi));
    CHECK_NEAR(lo, 12.4, 0.001);
    CHECK_NEAR(hi, 12.4, 0.001);

    // Negative readings round away from zero the same way, which a truncating
    // cast would not do -- it rounds toward zero and would bias every
    // below-freezing coolant reading upward by a tenth.
    historyTick(h, 1000u + 2u * HIST_BUCKET_MS);
    historyPush(h, HIST_COOLANT, -12.36f);
    CHECK(historyRange(h, HIST_COOLANT, 0u, lo, hi));
    CHECK_NEAR(lo, -12.4, 0.001);
  }

  // -------------------------------------------------------------------------
  // 9. Out-of-range arguments return false and touch nothing
  // -------------------------------------------------------------------------
  {
    History h;
    historyInit(h);
    historyTick(h, 1000u);
    historyPush(h, HIST_SPEED, 50.0f);

    float lo = -999.0f;
    float hi = -999.0f;

    CHECK(!historyRange(h, (uint8_t)HIST_COUNT, 0u, lo, hi));
    CHECK_EQ((int)lo, -999);
    CHECK_EQ((int)hi, -999);

    CHECK(!historyRange(h, 200u, 0u, lo, hi));
    CHECK_EQ((int)lo, -999);

    CHECK(!historyRange(h, HIST_SPEED, (uint8_t)HIST_BUCKETS, lo, hi));
    CHECK_EQ((int)lo, -999);

    CHECK(!historyRange(h, HIST_SPEED, 255u, lo, hi));
    CHECK_EQ((int)lo, -999);

    // The last valid age is still valid -- the bound is exclusive, and a test
    // that only checked the rejection would pass on an off-by-one that ate the
    // oldest bucket.
    CHECK(!historyRange(h, HIST_SPEED, (uint8_t)(HIST_BUCKETS - 1u), lo, hi));
    CHECK_EQ((int)lo, -999);   // empty, not out of range

    // A push with a bad param must not scribble on the bucket next door.
    historyPush(h, (uint8_t)HIST_COUNT, 999.0f);
    historyPush(h, 200u, 999.0f);
    CHECK(historyRange(h, HIST_SPEED, 0u, lo, hi));
    CHECK_NEAR(lo, 50.0, 0.001);
    CHECK_NEAR(hi, 50.0, 0.001);
  }

TEST_MAIN_END()
