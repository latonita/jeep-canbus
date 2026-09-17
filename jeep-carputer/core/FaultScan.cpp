#include "FaultScan.h"

void faultScanPlanInit(FaultScanPlan& p) {
  p.dueMs = 0u;
  p.requested = false;
}

void faultScanRequest(FaultScanPlan& p) {
  p.requested = true;
}

bool faultScanDue(const FaultScanPlan& p, uint32_t nowMs) {
  // SIGNED delta, not `nowMs >= p.dueMs`.
  //
  // The absolute form is right for 49 days and then wrong for the rest of the
  // key cycle: a deadline stamped at 0xFFFFF000 + 300000 lands at 0x00048BE7,
  // and an unsigned compare calls that "already past" the instant it is
  // written -- so the five-minute period collapses to no period at all for the
  // one pass that straddles the wrap, and every earlier now looks due. The
  // difference of two uint32 read as int32 is correct across the rollover for
  // any interval under 24.8 days, which five minutes comfortably is.
  return (int32_t)(nowMs - p.dueMs) >= 0;
}

FaultScanAct faultScanDecide(FaultScanPlan& p, bool linkUp, bool scanning,
                             uint8_t nextEcu, uint8_t ecuCount,
                             uint32_t nowMs, uint8_t& ecu) {
  // Nothing to walk. A zero-length walk would finish the instant it began,
  // without ever reaching a module and -- since the deadline only moves on a
  // completion the caller observes -- would be owed again on the very next
  // pass, forever. Real firmware cannot get here (ECU_COUNT is an enum with
  // modules in it); a test or a future table built at runtime can.
  if (ecuCount == 0u) {
    return FSCAN_NOTHING;
  }

  // Self-start, gated on the link. Expressed as "the timer presses RE-READ"
  // rather than as a second way into faultsBeginScan(), because there must be
  // exactly one path that starts a walk: two of them is how the request ends
  // up being honoured on one and dropped on the other.
  if (linkUp && !scanning && faultScanDue(p, nowMs)) {
    p.requested = true;
  }

  // A request only takes effect BETWEEN walks. Restarting one mid-flight would
  // throw away the modules already asked and ask them again, which from the
  // driver's side is the scan going backwards -- the count drops to nothing and
  // climbs back, on the page they are watching precisely because they want to
  // know whether the number changed.
  if (!scanning) {
    if (p.requested) {
      p.requested = false;
      return FSCAN_BEGIN;
    }
    return FSCAN_NOTHING;
  }

  // In flight. 0xFF is faultsNextEcu()'s "finished", which cannot happen while
  // `scanning` is true -- both come off the same cursor -- so it is the
  // caller's two arguments disagreeing rather than a state of the walk. Say
  // nothing owed rather than pick one to believe.
  if (nextEcu == 0xFFu) {
    return FSCAN_NOTHING;
  }

  // Past the caller's table. Defensive: the walk was begun with more modules
  // than the caller can name, so there is no request to build and the only
  // honest move is to end it. Not a transaction and not a skip -- a module
  // that does not exist did not fail to answer.
  if (nextEcu >= ecuCount) {
    return FSCAN_FINISH;
  }

  ecu = nextEcu;

  // No link: unreachable WITHOUT transacting.
  //
  // Asking anyway would cost twelve 800 ms timeouts for an answer already
  // known, and this runs inside the bring-up retry loop -- so it would delay
  // the reconnect that is the only thing that can make the walk useful. The
  // walk still completes and still reports honestly: every module quiet,
  // nothing found.
  return linkUp ? FSCAN_ASK : FSCAN_SKIP;
}

void faultScanNoteFinished(FaultScanPlan& p, bool answeredNothing,
                           uint32_t nowMs) {
  // `nowMs` and not `p.dueMs + FAULT_SCAN_PERIOD_MS`. The walk takes twelve
  // passes of carTask and the period is measured from its END, so the scans
  // are five minutes APART rather than every five minutes on a fixed grid that
  // the walk's own duration would eat into. It also means a walk that was held
  // up -- by the link, by a module taking its full timeout twelve times over
  // -- does not come back owed the moment it finishes.
  p.dueMs = answeredNothing ? nowMs : (nowMs + FAULT_SCAN_PERIOD_MS);
}
