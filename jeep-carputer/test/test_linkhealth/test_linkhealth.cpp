// Host tests for core/LinkHealth.cpp.
//
// Two properties here are worth more than the rest, and neither can be
// observed on the bench the firmware is developed on.
//
// The first is that SILENCE IS NOT DEATH: an adapter sitting on a desk with no
// car attached delivers nothing at all, forever, and must still read as a live
// link the moment it answers AT@ST. Get that wrong and the instrument shows a
// permanent link fault in the only configuration anyone develops in -- and,
// worse, teaches whoever sees it to ignore the indicator.
//
// The second is the WRAP. The clock is a 64-bit timer truncated to uint32_t,
// so every comparison here has ~49.7 days to be wrong in and one instant in
// which to be wrong at all. An absolute `stamp + LIMIT < now` test passes
// every test anyone would write by hand and then, once, reports a live adapter
// as dead or a dead one as live until the next reboot. The only way that gets
// caught is by driving the timestamps over the rollover on a laptop, which is
// what the last block does.
#include "doctest_min.h"

#include "LinkHealth.h"

// One loop pass with no probe expected: assert the action and hand back `up`.
static bool quietPass(LinkHealth& h, uint32_t lastRxMs, uint32_t nowMs) {
  CHECK_EQ(linkHealthStep(h, lastRxMs, nowMs), LH_NOTHING);
  return h.up;
}

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // A fresh state is DOWN and says so. Not "up until proven otherwise": a
  // board that has never heard an adapter must not draw a healthy link.
  // -------------------------------------------------------------------------
  {
    LinkHealth h;
    linkHealthInit(h);
    CHECK(!h.up);
    CHECK(!h.everProbed);
  }

  // -------------------------------------------------------------------------
  // Traffic inside the quiet window is the answer. No probe, ever, however
  // many times the loop comes round -- the adapter is demonstrably talking and
  // asking it to confirm that would be a transaction bought with nothing.
  // -------------------------------------------------------------------------
  {
    LinkHealth h;
    linkHealthInit(h);
    for (uint32_t t = 1000u; t <= 60000u; t += 100u) {
      CHECK(quietPass(h, t, t));      // a line arriving every pass
    }
    CHECK(!h.everProbed);             // not one probe in a minute of traffic
  }

  // -------------------------------------------------------------------------
  // lastRxMs == 0 means NEVER, and is not traffic at the epoch.
  //
  // This is the one that would look right while being wrong: left as a
  // timestamp, a link that has not delivered a single byte reads as traffic
  // 0 ms ago -- perfect health, sustained for the first LINK_QUIET_MS of
  // uptime, on an adapter that is not plugged in.
  // -------------------------------------------------------------------------
  {
    LinkHealth h;
    linkHealthInit(h);
    CHECK_EQ(linkHealthStep(h, 0u, 5u), LH_PROBE);   // 5 ms in, still asks
    linkHealthNoteProbe(h, false, 10u);
    CHECK(!h.up);
  }

  // -------------------------------------------------------------------------
  // The quiet window's edge. `< LINK_QUIET_MS`, so an age of exactly the limit
  // is already quiet.
  // -------------------------------------------------------------------------
  {
    LinkHealth h;
    linkHealthInit(h);
    const uint32_t rx = 10000u;
    CHECK(quietPass(h, rx, rx + LINK_QUIET_MS - 1u));
    LinkHealth g;
    linkHealthInit(g);
    CHECK_EQ(linkHealthStep(g, rx, rx + LINK_QUIET_MS), LH_PROBE);
  }

  // -------------------------------------------------------------------------
  // Going quiet costs exactly ONE probe per LINK_PROBE_MS.
  //
  // The loop turns over several times a second and a probe against a dead
  // adapter blocks for its whole timeout, so a missing spacing test would make
  // the watchdog the reason nothing else runs -- on exactly the fault it
  // exists to report.
  // -------------------------------------------------------------------------
  {
    LinkHealth h;
    linkHealthInit(h);
    const uint32_t rx = 1000u;
    CHECK(quietPass(h, rx, 1500u));                   // traffic: up

    CHECK_EQ(linkHealthStep(h, rx, 4100u), LH_PROBE); // 3100 ms quiet
    linkHealthNoteProbe(h, true, 4150u);              // answered
    CHECK(h.up);

    // Every pass for the next interval: still quiet, still no second probe.
    for (uint32_t t = 4200u; t < 4150u + LINK_PROBE_MS; t += 100u) {
      CHECK(quietPass(h, rx, t));                     // verdict LATCHES up
    }
    // Spacing runs from when the probe FINISHED (4150), not from the pass that
    // issued it (4100) -- the timeout is time already spent on this link.
    CHECK_EQ(linkHealthStep(h, rx, 4150u + LINK_PROBE_MS), LH_PROBE);
  }

  // -------------------------------------------------------------------------
  // THE BENCH CASE. Silent bus, answering adapter: UP, indefinitely.
  //
  // No frames at all is what a desk with no car looks like, and it is also
  // what a live 83.3 kbps bus looks like through a streaming adapter with the
  // ignition off. Neither is a dead link.
  // -------------------------------------------------------------------------
  {
    LinkHealth h;
    linkHealthInit(h);
    uint32_t t = 0u;
    for (uint8_t probes = 0u; probes < 20u; ++probes) {
      // Grind the loop forward until the spacing lets the next probe out.
      LinkHealthAct act = LH_NOTHING;
      while (act == LH_NOTHING) {
        t += 100u;
        act = linkHealthStep(h, 0u, t);   // nothing has EVER arrived
      }
      linkHealthNoteProbe(h, true, t + 20u);
      CHECK(h.up);
    }
    // 20 probes over the whole run, not one per pass: ~60 s of loop time at
    // the 3 s spacing.
    CHECK(t >= 19u * LINK_PROBE_MS);
  }

  // -------------------------------------------------------------------------
  // A failed probe is DOWN, and stays down between probes rather than flapping
  // -- there is no new evidence, so there is no new verdict.
  // -------------------------------------------------------------------------
  {
    LinkHealth h;
    linkHealthInit(h);
    CHECK_EQ(linkHealthStep(h, 0u, 1000u), LH_PROBE);
    linkHealthNoteProbe(h, false, 1800u);              // 800 ms timeout
    CHECK(!h.up);
    for (uint32_t t = 1900u; t < 1800u + LINK_PROBE_MS; t += 100u) {
      CHECK(!quietPass(h, 0u, t));
    }
    CHECK_EQ(linkHealthStep(h, 0u, 1800u + LINK_PROBE_MS), LH_PROBE);
  }

  // -------------------------------------------------------------------------
  // Recovery: traffic returning is proof enough on its own.
  //
  // It must not have to wait for the next probe to be due. The cable going
  // back in produces frames immediately, and a link that reads DOWN for up to
  // three more seconds after it is plainly working is the same lie as ready()
  // reading UP after the cable came out, pointed the other way.
  // -------------------------------------------------------------------------
  {
    LinkHealth h;
    linkHealthInit(h);
    CHECK_EQ(linkHealthStep(h, 0u, 1000u), LH_PROBE);
    linkHealthNoteProbe(h, false, 1500u);
    CHECK(!h.up);
    // One line arrives at 2000, well inside the probe spacing.
    CHECK(quietPass(h, 2000u, 2000u));
    CHECK(h.up);
  }

  // -------------------------------------------------------------------------
  // The RECONNECT path: a verdict reached out of band.
  //
  // main.cpp answers a failed probe by re-running begin(), and a reconnect
  // that succeeds proves the adapter as well as an answered AT@ST does. There
  // is deliberately no "was a probe outstanding?" guard for that reason.
  // -------------------------------------------------------------------------
  {
    LinkHealth h;
    linkHealthInit(h);
    CHECK_EQ(linkHealthStep(h, 0u, 1000u), LH_PROBE);
    linkHealthNoteProbe(h, false, 1800u);          // probe failed
    CHECK(!h.up);
    linkHealthNoteProbe(h, true, 1900u);           // begin() brought it back
    CHECK(h.up);
    // And the recovery did not buy a free probe: the spacing moved with it.
    CHECK(quietPass(h, 0u, 1900u + LINK_PROBE_MS - 1u));
    CHECK_EQ(linkHealthStep(h, 0u, 1900u + LINK_PROBE_MS), LH_PROBE);
  }

  // -------------------------------------------------------------------------
  // MILLISECOND WRAPAROUND.
  //
  // nowMs() truncates a 64-bit timer to uint32_t, so the clock steps from
  // 0xFFFFFFFF to 0 once every 49.7 days. Unsigned subtraction is modulo 2^32
  // and comes out right across that step; `stamp + LIMIT` does not, and the
  // numbers below are chosen so an absolute comparison fails in both
  // directions.
  // -------------------------------------------------------------------------
  {
    const uint32_t kNearEnd = 0xFFFFFF00u;   // 256 ms before the rollover

    // Traffic straddling the wrap is still traffic. An absolute test computes
    // kNearEnd + 3000, which overflows to 0xB98, and then declares 0x100 to be
    // BEFORE it -- reporting a healthy link for the wrong reason, right up
    // until the moment it reports a dead one.
    {
      LinkHealth h;
      linkHealthInit(h);
      CHECK(quietPass(h, kNearEnd, 0x100u));   // age 0x200 = 512 ms
      CHECK(!h.everProbed);
    }

    // Quiet straddling the wrap is still quiet, and must still probe.
    {
      LinkHealth h;
      linkHealthInit(h);
      const uint32_t now = kNearEnd + LINK_QUIET_MS;   // wraps to 0xBFC
      CHECK_EQ(linkHealthStep(h, kNearEnd, now), LH_PROBE);
      linkHealthNoteProbe(h, true, now + 20u);
      CHECK(h.up);
    }

    // Probe SPACING across the wrap: one probe before it, none for the
    // interval after it, then exactly one more.
    {
      LinkHealth h;
      linkHealthInit(h);
      CHECK_EQ(linkHealthStep(h, 0u, kNearEnd), LH_PROBE);
      linkHealthNoteProbe(h, false, kNearEnd);
      CHECK(!h.up);

      // Walk the clock over the rollover in 100 ms steps. Everything inside
      // the interval must be refused, wrap or no wrap.
      for (uint32_t d = 100u; d < LINK_PROBE_MS; d += 100u) {
        CHECK(!quietPass(h, 0u, kNearEnd + d));
      }
      CHECK_EQ(linkHealthStep(h, 0u, kNearEnd + LINK_PROBE_MS), LH_PROBE);
      linkHealthNoteProbe(h, true, kNearEnd + LINK_PROBE_MS);
      CHECK(h.up);
    }

    // And a full minute of loop passes straight through the wrap with real
    // traffic: the whole point is that nothing anywhere in here notices.
    {
      LinkHealth h;
      linkHealthInit(h);
      for (uint32_t d = 0u; d < 60000u; d += 100u) {
        const uint32_t t = kNearEnd + d;
        CHECK(quietPass(h, t, t));
      }
      CHECK(!h.everProbed);
    }
  }

TEST_MAIN_END()
