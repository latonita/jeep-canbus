#pragma once
// Is the adapter still there?
//
// This is the decision half of what used to live entirely inside
// linkWatchdog() in main.cpp, where it could not be tested: the function it
// sat in owned a link object, a UART, a logger and two UI calls, so the only
// way to exercise "the cable was pulled" was to pull the cable.
//
// THE PROBLEM IT SOLVES. CarLink::ready() only reports that begin() once
// succeeded. It stays true forever afterwards -- including with the connector
// hanging out of the radio -- so ready() is not liveness and must never be
// drawn as it. Silence is the real signal.
//
// SILENCE ALONE IS NOT ENOUGH EITHER, and that is the whole reason this file
// exists rather than a single timeout. A bench with the adapter on a desk and
// no car attached produces no frames at all, indefinitely, and that must not
// read as a dead link -- it is the configuration the firmware is developed in.
// So quiet triggers a QUESTION (AT@ST) rather than a verdict: an adapter that
// answers is alive on a quiet bus, and one that does not is gone.
//
// The I/O stays with the caller. This returns "you should probe now" and is
// told the answer, which is what makes every branch below reachable from a
// host test -- including the millisecond wrap, which no bench session is ever
// going to sit through.
//
// The caller's shape, which is main.cpp's linkWatchdog():
//
//   if (linkHealthStep(gHealth, gLink.lastRxMs(), now) == LH_PROBE) {
//     char st[96];
//     const bool ok = gLink.status(st, sizeof(st)) == LINK_OK;
//     linkHealthNoteProbe(gHealth, ok, nowMs());
//     if (!ok && gLink.begin() == LINK_OK) {      // try to bring it back
//       reportLinkIdentity();
//       linkHealthNoteProbe(gHealth, true, nowMs());
//     }
//   }
//   setLinkUp(gHealth.up);
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

// No line of any kind off the adapter for this long and we stop believing it
// is there. Generous against the worst legitimate quiet: in the ATMA fallback
// the adapter goes silent for the whole poll burst, bounded by three polls at
// 400 ms plus the window, so about 2 s. 3 s clears that with room.
//
// Note what this measures: ANY line, not any FRAME. A streaming adapter with a
// genuinely silent bus is still quiet by this test, which is what the probe
// below is for.
#define LINK_QUIET_MS 3000u

// Minimum spacing between liveness probes, so a genuinely dead link is not
// hammered with AT@ST once per loop.
//
// The loop this runs in turns over several times a second. Without a floor, an
// absent adapter would mean a blocking status() call and its timeout on every
// pass -- the watchdog would then be the reason nothing else got done, on
// exactly the fault it exists to report.
#define LINK_PROBE_MS 3000u

// Everything remembered between calls. Plain data: no clock, no link, nothing
// that cannot be constructed in a test.
struct LinkHealth {
  // When the last probe was ISSUED or REPORTED, whichever happened later.
  // Meaningless until everProbed -- see linkHealthStep for why that flag is
  // not the same thing as lastProbeMs == 0.
  uint32_t lastProbeMs;

  // The current verdict, and the only thing a caller should draw. It is a
  // LATCH, not a computation: while the probe interval forbids asking again
  // there is no new evidence, so the previous answer stands. A link that reads
  // DOWN stays DOWN until something says otherwise, rather than flickering
  // between "quiet" and "dead" at loop rate.
  bool up;

  // Whether a probe has ever been issued.
  //
  // NOT redundant with lastProbeMs == 0, and not cosmetic. It is what lets the
  // FIRST probe go out immediately: a bare zero stamp would make the boot
  // epoch look like "probed at t=0", and the spacing test would then swallow
  // the first LINK_PROBE_MS of uptime -- three seconds during which a missing
  // adapter reads as whatever `up` was initialised to. That is the one
  // deliberate difference from the `static uint32_t lastProbe = 0` this
  // replaces, where the grace period was an accident of zero-initialisation
  // rather than a decision. It also disambiguates the same collision at the
  // far end of the clock, where now == 0 comes round again for real.
  bool everProbed;
};

enum LinkHealthAct : uint8_t {
  LH_NOTHING = 0,   // nothing to do; `up` already reflects the truth
  LH_PROBE,         // caller should run its AT@ST probe and report back
};

void linkHealthInit(LinkHealth& h);

// Decide what to do now.
//
// `lastRxMs` is the link's last-received-anything stamp, where 0 means NEVER.
// Zero is not a timestamp here and must not be treated as one: a link that has
// never delivered a byte would otherwise look like traffic at the epoch, which
// near boot is recent traffic and reads as a healthy link that has in fact
// never spoken.
//
// Returning LH_PROBE stamps the attempt, so the spacing holds even if the
// caller never reports back -- a caller that drops the answer on the floor
// must not be able to turn this into a probe per loop.
LinkHealthAct linkHealthStep(LinkHealth& h, uint32_t lastRxMs, uint32_t nowMs);

// Feed the probe's outcome back in. `ok` is the verdict: an adapter that
// answered is up whatever the bus is doing, and one that did not is down.
//
// Also the way to report a verdict reached OUT OF BAND, which the caller needs
// for exactly one case: after a failed probe it re-runs begin(), and a
// reconnect that succeeds is as good a proof of life as an answered AT@ST.
// Hence no "was a probe outstanding?" guard -- there is a legitimate caller
// with an answer this file never asked for.
//
// `nowMs` re-stamps the spacing from when the probe FINISHED rather than from
// when it was issued, the same argument as PollScheduler::noteResult: a probe
// against a dead adapter blocks the caller for its whole timeout, and that
// time was spent on this link, not waiting to bother it again.
void linkHealthNoteProbe(LinkHealth& h, bool ok, uint32_t nowMs);
