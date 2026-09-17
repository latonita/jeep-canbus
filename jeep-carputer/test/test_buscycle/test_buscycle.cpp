// Host tests for core/BusCycle.cpp.
//
// Small module, and the tests are here for two specific rules rather than for
// coverage.
//
// The first is that STREAMING WINS and monitoring is not even attempted
// alongside it. On a 2.1 gateway ATMA would make the adapter deaf to requests
// in exchange for frames that are already arriving -- so the bug this guards
// against is not a wrong number, it is the poll path going silent on the one
// adapter that supports everything.
//
// The second is the rolling-worst report. It is the loop's own health metric,
// and the loop's period is what bounds how stale anything on the panel can be.
// A worst-case figure that silently resets at the wrong moment, or that is
// poisoned for a whole key cycle by one clock rollover, is an instrument that
// lies about itself -- which is worse than not having it.
#include "doctest_min.h"

#include "BusCycle.h"

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // The listen decision. Three inputs' worth of truth table.
  // -------------------------------------------------------------------------
  {
    // Streaming: pace only, and the pace is a SAMPLING RATE -- it must stay
    // comfortably under 0x002's 50 ms or tripUpdate() starts missing flow
    // intervals and loses fuel silently.
    BusListenPlan p = busListenDecide(true, false);
    CHECK_EQ((int)p.mode, (int)BUS_LISTEN_STREAM);
    CHECK_EQ(p.delayMs, BUS_STREAM_PACE_MS);
    CHECK(p.delayMs < 50u);

    // Streaming WINS even when the caller says monitoring is available. This
    // is the case worth asserting: the caller should not have tried, but if it
    // does, the decision must not take it up on the offer.
    p = busListenDecide(true, true);
    CHECK_EQ((int)p.mode, (int)BUS_LISTEN_STREAM);
    CHECK_EQ(p.delayMs, BUS_STREAM_PACE_MS);

    // Not streaming, monitor available: the ATMA window.
    p = busListenDecide(false, true);
    CHECK_EQ((int)p.mode, (int)BUS_LISTEN_MONITOR);
    CHECK_EQ(p.delayMs, BUS_MONITOR_WINDOW_MS);

    // Neither: poll alone rather than stall. A bench link, or an adapter that
    // refused ATMA.
    p = busListenDecide(false, false);
    CHECK_EQ((int)p.mode, (int)BUS_LISTEN_NONE);
    CHECK_EQ(p.delayMs, BUS_POLL_ONLY_PACE_MS);
  }

  // -------------------------------------------------------------------------
  // Only the MONITOR plan needs closing, and that is what tells the caller
  // whether to call monitorEnd(). Calling it without having begun is what
  // would leave the adapter in a mode nobody asked for.
  // -------------------------------------------------------------------------
  {
    CHECK(busListenDecide(false, true).mode == BUS_LISTEN_MONITOR);
    CHECK(busListenDecide(true, true).mode != BUS_LISTEN_MONITOR);
    CHECK(busListenDecide(false, false).mode != BUS_LISTEN_MONITOR);
  }

  // -------------------------------------------------------------------------
  // The window has to leave room for the poll burst inside the link's quiet
  // timeout, and that relationship is arithmetic rather than taste. Three
  // polls at their 400 ms worst is 1.2 s of deafness on top of the window;
  // LINK_QUIET_MS is 3000. If either number moves this is the check that
  // notices.
  // -------------------------------------------------------------------------
  {
    const uint32_t worstBurst = (uint32_t)BUS_POLLS_PER_PASS * 400u;
    CHECK_EQ(worstBurst, 1200u);
    CHECK((BUS_MONITOR_WINDOW_MS + worstBurst) < 3000u);
  }

  // -------------------------------------------------------------------------
  // The loop metric: a rolling worst that describes ITS OWN window.
  // -------------------------------------------------------------------------
  {
    BusLoopStats s;
    busLoopInit(s, 1000u);
    CHECK_EQ(s.worstMs, 0u);
    CHECK_EQ(s.reportedWorstMs, 0u);

    // Twenty-millisecond passes, nothing due yet.
    uint32_t t = 1000u;
    for (uint8_t i = 0u; i < 10u; ++i) {
      t += 20u;
      CHECK(!busLoopTick(s, t));
    }
    CHECK_EQ(s.periodMs, 20u);
    CHECK_EQ(s.worstMs, 20u);

    // One excursion. The WORST is the whole point of the figure -- one 900 ms
    // pass among fifty 20 ms ones vanishes in a mean and is the entire story.
    t += 900u;
    CHECK(!busLoopTick(s, t));
    CHECK_EQ(s.periodMs, 900u);
    CHECK_EQ(s.worstMs, 900u);

    // Quieter again: the worst must HOLD, not decay.
    t += 20u;
    CHECK(!busLoopTick(s, t));
    CHECK_EQ(s.worstMs, 900u);

    // Run on in REALISTIC passes until the report comes due. Jumping the
    // clock straight to the deadline would manufacture one enormous pass and
    // that pass would legitimately become the worst -- which is the module
    // working, not failing, and is worth saying out loud because the first
    // version of this test made exactly that mistake and blamed the code.
    bool due = false;
    for (uint16_t i = 0u; i < 2000u && !due; ++i) {
      t += 20u;
      due = busLoopTick(s, t);
    }
    CHECK(due);
    CHECK_EQ(s.reportedWorstMs, 900u);
    CHECK_EQ(s.worstMs, 0u);

    // The NEXT window is its own. A report that carried uptime's worst for
    // ever would say nothing about now.
    t += 20u;
    CHECK(!busLoopTick(s, t));
    CHECK_EQ(s.worstMs, 20u);
    CHECK_EQ(s.reportedWorstMs, 900u);   // held until the next report replaces it
  }

  // -------------------------------------------------------------------------
  // MILLISECOND WRAPAROUND. nowMs truncates a 64-bit timer and rolls every
  // 49.7 days. An absolute compare would report a period of nearly 2^32 ms
  // once per roll -- and because the worst only ever goes UP between reports,
  // that single bogus figure would poison the metric for the rest of the key
  // cycle.
  // -------------------------------------------------------------------------
  {
    BusLoopStats s;
    const uint32_t before = 0xFFFFFF00u;
    busLoopInit(s, before);

    // Walk straight through zero in 20 ms steps.
    uint32_t t = before;
    for (uint8_t i = 0u; i < 30u; ++i) {
      t += 20u;                       // wraps partway through
      (void)busLoopTick(s, t);
      CHECK_EQ(s.periodMs, 20u);      // never 4 294 967 xxx
      CHECK(s.worstMs <= 20u);
    }

    // And the report interval survives the roll: a report stamped before the
    // wrap must still come due after it.
    BusLoopStats r;
    busLoopInit(r, 0xFFFFF000u);
    CHECK(!busLoopTick(r, 0xFFFFF000u + 5000u));
    CHECK(busLoopTick(r, 0xFFFFF000u + BUS_LOOP_REPORT_MS + 1u));
  }

  // -------------------------------------------------------------------------
  // A pass that takes exactly the report interval is due, and the boundary is
  // not off by one in the direction that would make the report never fire.
  // -------------------------------------------------------------------------
  {
    BusLoopStats s;
    busLoopInit(s, 0u);
    CHECK(!busLoopTick(s, BUS_LOOP_REPORT_MS - 1u));
    CHECK(busLoopTick(s, BUS_LOOP_REPORT_MS));
  }

TEST_MAIN_END()
