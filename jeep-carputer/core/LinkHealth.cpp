// Link liveness: traffic, or a probe, or a verdict that stands.
//
// PURE: no ESP-IDF, no Arduino. Owns no link and no clock -- the caller does
// the talking and hands the answer back.
#include "LinkHealth.h"

#include <stdint.h>

void linkHealthInit(LinkHealth& h) {
  h.lastProbeMs = 0u;
  // Starts DOWN, deliberately. The alternative -- assume up until proven
  // otherwise -- would show a confident link indicator on a board that has
  // never heard from an adapter, which is the frozen-but-plausible failure
  // this instrument avoids everywhere else. One pass of the loop either
  // confirms it or probes.
  h.up = false;
  h.everProbed = false;
}

LinkHealthAct linkHealthStep(LinkHealth& h, uint32_t lastRxMs, uint32_t nowMs) {
  // Traffic within the quiet window: the adapter is demonstrably talking, so
  // there is nothing to ask and nothing to wait for.
  //
  // lastRxMs == 0 is NEVER, not "at the epoch". Left as a timestamp it would
  // read as traffic 0 ms ago at boot, i.e. a link in perfect health that has
  // not delivered a single byte.
  //
  // ROLLOVER SAFETY: unsigned subtraction is modulo 2^32, so the age is
  // correct across the ~49.7 day wrap of the truncated clock. Never compare
  // lastRxMs + LINK_QUIET_MS against nowMs -- that addition overflows near the
  // wrap and makes live traffic look ancient. Same rule as
  // VehicleState::fresh, and the same reason.
  if (lastRxMs != 0u && (uint32_t)(nowMs - lastRxMs) < LINK_QUIET_MS) {
    h.up = true;
    return LH_NOTHING;
  }

  // Quiet. Ask -- but not more often than LINK_PROBE_MS, and not at all until
  // the previous answer has had its turn.
  if (h.everProbed && (uint32_t)(nowMs - h.lastProbeMs) < LINK_PROBE_MS) {
    // No new evidence, so no new verdict: `up` keeps whatever the last probe
    // or the last traffic established. Returning a computed "down" here
    // instead would make a quiet bus flap DOWN for the gap between probes and
    // UP again on each answer, several times a minute, on a link that is fine.
    return LH_NOTHING;
  }

  // Stamped at ISSUE, before the caller has said anything. The spacing is a
  // floor on how often a dead adapter is bothered, and it has to hold for a
  // caller that never reports back at all -- a probe that throws its own
  // result away must still cost only one probe.
  h.lastProbeMs = nowMs;
  h.everProbed = true;
  return LH_PROBE;
}

void linkHealthNoteProbe(LinkHealth& h, bool ok, uint32_t nowMs) {
  // An answered probe is UP even with a dead silent bus. That is the entire
  // distinction this module exists to make: no frames means nothing about the
  // adapter, and a bench with no car is the case that proves it.
  h.up = ok;
  // Re-stamp from completion. See the header: the probe's own timeout is time
  // already spent on this link.
  h.lastProbeMs = nowMs;
  h.everProbed = true;
}
