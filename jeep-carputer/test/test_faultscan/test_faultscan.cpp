// Host tests for core/FaultScan.cpp.
//
// The rule worth protecting: a walk abandoned because the link went down is
// owed AGAIN IMMEDIATELY, not in five minutes. Nothing on the panel says which
// of those two happened -- the fault list looks the same either way -- so the
// only place the distinction can be caught is here, and getting it wrong means
// a dropout costs the driver a five-minute hole with an all-quiet fault list
// sitting in it.
//
// Second: the deadline is compared SIGNED. Every test below that straddles
// 0xFFFFFFFF is aimed at the absolute comparison an author reaches for first,
// which is right for 49 days and then silently wrong.
#include "doctest_min.h"

#include "FaultScan.h"
#include "Faults.h"
#include "SignalDb.h"

static DtcRecord rec(uint16_t code, uint8_t status) {
  DtcRecord r;
  r.code = code;
  r.status = status;
  return r;
}

// One pass of carTask, wired the way main.cpp's faultScanStep() is: decide,
// then execute, and for FSCAN_BEGIN decide AGAIN in the same pass -- which is
// what makes the pass that starts a walk also ask module 0, exactly as the
// original straight-line code did.
//
// The real Faults.h walk is driven here rather than a stub, because the pair
// only works if FaultScan's view of `scanning` / `nextEcu` and the cursor
// Faults.cpp actually keeps agree. A stub would agree with itself.
struct Pass {
  FaultScanAct act;
  uint8_t      ecu;
  bool         begun;    // this pass started a walk
  bool         ended;    // this pass ended one
};

static Pass runPass(FaultScanPlan& p, FaultSet& s, bool linkUp, uint32_t nowMs,
                    bool moduleAnswers) {
  Pass r;
  r.ecu = 0xFFu;
  r.begun = false;
  r.ended = false;

  r.act = faultScanDecide(p, linkUp, faultsScanning(s), faultsNextEcu(s),
                          (uint8_t)ECU_COUNT, nowMs, r.ecu);
  if (r.act == FSCAN_BEGIN) {
    faultsBeginScan(s, (uint8_t)ECU_COUNT);
    r.begun = true;
    r.act = faultScanDecide(p, linkUp, faultsScanning(s), faultsNextEcu(s),
                            (uint8_t)ECU_COUNT, nowMs, r.ecu);
  }

  if (r.act == FSCAN_ASK) {
    if (moduleAnswers) {
      const DtcRecord one[1] = {rec((uint16_t)(0x0100u + r.ecu), 0x21u)};
      faultsRecord(s, r.ecu, one, 1u, false);
    } else {
      faultsEcuFailed(s, r.ecu);        // silent, or a negative reply
    }
  } else if (r.act == FSCAN_SKIP) {
    faultsEcuFailed(s, r.ecu);          // not asked at all: no link
  } else if (r.act == FSCAN_FINISH) {
    faultsEndScan(s, nowMs);
    faultScanNoteFinished(p, false, nowMs);
    r.ended = true;
  }

  if ((r.act == FSCAN_ASK || r.act == FSCAN_SKIP) && !faultsScanning(s)) {
    faultsEndScan(s, nowMs);
    // Abandoned exactly when the last module was one we never asked.
    faultScanNoteFinished(p, r.act == FSCAN_SKIP, nowMs);
    r.ended = true;
  }
  return r;
}

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // A fresh plan is owed a walk, but only once the link is up
  //
  // dueMs 0 is "as soon as the link allows", not "at boot": with the link
  // still down every module would be marked unreachable without being asked,
  // and the driver would be handed a completed scan that questioned nothing
  // seconds before the answer became obtainable.
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);
    CHECK(!p.requested);
    CHECK(faultScanDue(p, 0u));
    CHECK(faultScanDue(p, 1234u));

    FaultSet s;
    faultsInit(s);
    uint8_t ecu = 0xFFu;

    // Link down, nobody asked: nothing happens, and nothing is armed for
    // later either -- the self-start gate is not a deferred request.
    CHECK_EQ(faultScanDecide(p, false, false, 0xFFu, (uint8_t)ECU_COUNT, 500u, ecu),
             FSCAN_NOTHING);
    CHECK(!p.requested);
    CHECK(!faultsScanning(s));

    // Link up: owed.
    CHECK_EQ(faultScanDecide(p, true, false, 0xFFu, (uint8_t)ECU_COUNT, 500u, ecu),
             FSCAN_BEGIN);
    CHECK(!p.requested);                 // consumed by the BEGIN
  }

  // -------------------------------------------------------------------------
  // A full walk: every module asked exactly once, in index order
  //
  // Twelve today. The count is taken from ECU_COUNT rather than written out,
  // so adding a module extends the walk instead of breaking this test -- what
  // is being locked down is one pass per module and no module asked twice.
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);
    FaultSet s;
    faultsInit(s);

    uint8_t asked = 0u;
    uint32_t now = 1000u;
    // The first pass begins the walk AND asks module 0.
    Pass r = runPass(p, s, true, now, true);
    CHECK(r.begun);
    CHECK_EQ(r.act, FSCAN_ASK);
    CHECK_EQ(r.ecu, 0);
    CHECK(!r.ended);
    ++asked;

    while (faultsScanning(s)) {
      now += 200u;                        // one pass of carTask
      r = runPass(p, s, true, now, true);
      CHECK(!r.begun);                    // never restarted mid-flight
      CHECK_EQ(r.act, FSCAN_ASK);
      CHECK_EQ(r.ecu, asked);             // strictly index order
      ++asked;
    }
    CHECK_EQ(asked, (int)ECU_COUNT);
    CHECK(r.ended);                       // the last ASK closed the walk
    CHECK_EQ(faultsAnsweredCount(s), (int)ECU_COUNT);
    CHECK_EQ(faultsUnreachableCount(s), 0);
    CHECK_EQ(s.count, (int)ECU_COUNT);    // one code each
    CHECK_EQ(s.doneMs, now);

    // And the next one is five minutes out, measured from the END of the walk
    // rather than from when it was owed -- twelve passes of a bus that was
    // being listened to should not be deducted from the gap.
    CHECK_EQ(p.dueMs, now + FAULT_SCAN_PERIOD_MS);
    CHECK(!p.requested);
    CHECK(!faultScanDue(p, now));
  }

  // -------------------------------------------------------------------------
  // The five minutes are honoured, to the millisecond
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);
    faultScanNoteFinished(p, false, 100000u);
    CHECK_EQ(p.dueMs, 100000u + FAULT_SCAN_PERIOD_MS);

    uint8_t ecu = 0xFFu;
    const uint32_t due = p.dueMs;
    CHECK(!faultScanDue(p, due - 1u));
    CHECK_EQ(faultScanDecide(p, true, false, 0xFFu, (uint8_t)ECU_COUNT, due - 1u, ecu),
             FSCAN_NOTHING);
    CHECK(faultScanDue(p, due));
    CHECK_EQ(faultScanDecide(p, true, false, 0xFFu, (uint8_t)ECU_COUNT, due, ecu),
             FSCAN_BEGIN);
  }

  // -------------------------------------------------------------------------
  // A walk with the link down costs no transactions, and is re-owed AT ONCE
  //
  // This is the behaviour the module exists for. Only a RE-READ can start a
  // walk with the link down (the self-start gate refuses to), every module is
  // marked unreachable WITHOUT being asked -- twelve 800 ms timeouts inside
  // the loop that is trying to reconnect is the opposite of helping -- and the
  // resulting empty answer must not be allowed to stand for five minutes.
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);
    faultScanNoteFinished(p, false, 10000u);   // not due for five minutes
    FaultSet s;
    faultsInit(s);

    faultScanRequest(p);                        // RE-READ, link down
    uint32_t now = 20000u;
    Pass r = runPass(p, s, false, now, false);
    CHECK(r.begun);
    CHECK_EQ(r.act, FSCAN_SKIP);                // not ASK: nothing transacted
    CHECK_EQ(r.ecu, 0);

    uint8_t skipped = 1u;
    while (faultsScanning(s)) {
      now += 200u;
      r = runPass(p, s, false, now, false);
      CHECK_EQ(r.act, FSCAN_SKIP);
      ++skipped;
    }
    CHECK_EQ(skipped, (int)ECU_COUNT);
    CHECK(r.ended);
    CHECK_EQ(faultsUnreachableCount(s), (int)ECU_COUNT);
    CHECK_EQ(faultsAnsweredCount(s), 0);
    CHECK_EQ(s.count, 0);

    // Owed NOW, not at now + five minutes. Five minutes was the defect: the
    // one walk that learned nothing would have held the list for as long as a
    // successful one.
    CHECK_EQ(p.dueMs, now);
    CHECK(faultScanDue(p, now));

    // Still gated, though -- with the link down it stays owed and asks
    // nothing, and the moment the link returns it begins.
    uint8_t ecu = 0xFFu;
    CHECK_EQ(faultScanDecide(p, false, false, 0xFFu, (uint8_t)ECU_COUNT, now, ecu),
             FSCAN_NOTHING);
    CHECK_EQ(faultScanDecide(p, true, false, 0xFFu, (uint8_t)ECU_COUNT, now, ecu),
             FSCAN_BEGIN);
  }

  // -------------------------------------------------------------------------
  // The link dropping MID-walk re-owes it too, and keeps what it had
  //
  // Half the modules answered before the cable was pulled. Those answers are
  // real and stay in the set; the rest are unreachable, which is the honest
  // word for them.
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);
    FaultSet s;
    faultsInit(s);

    uint32_t now = 1000u;
    Pass r = runPass(p, s, true, now, true);     // begins, asks 0
    CHECK_EQ(r.act, FSCAN_ASK);
    for (uint8_t i = 1u; i < 4u; ++i) {
      now += 200u;
      r = runPass(p, s, true, now, true);
      CHECK_EQ(r.act, FSCAN_ASK);
    }
    CHECK_EQ(faultsAnsweredCount(s), 4);

    while (faultsScanning(s)) {                  // link gone from here on
      now += 200u;
      r = runPass(p, s, false, now, false);
      CHECK_EQ(r.act, FSCAN_SKIP);
    }
    CHECK(r.ended);
    CHECK_EQ(faultsAnsweredCount(s), 4);         // kept
    CHECK_EQ(faultsUnreachableCount(s), (int)ECU_COUNT - 4);
    CHECK_EQ(s.count, 4);
    CHECK_EQ(p.dueMs, now);                      // re-owed at once
  }

  // -------------------------------------------------------------------------
  // A RE-READ mid-walk does not restart it -- and is not thrown away either
  //
  // Restarting would re-ask the modules already done, which from the driver's
  // side is the scan going backwards: the count they are watching drops to
  // nothing and climbs back. Dropping the press would be worse -- they pressed
  // it because they want a reading taken AFTER the press, and the walk in
  // flight was already half-taken before it. So it waits, and the walk that
  // follows starts immediately regardless of the five minutes.
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);
    FaultSet s;
    faultsInit(s);

    uint32_t now = 1000u;
    Pass r = runPass(p, s, true, now, true);
    CHECK(r.begun);

    now += 200u;
    r = runPass(p, s, true, now, true);
    CHECK_EQ(r.ecu, 1);
    faultScanRequest(p);                         // RE-READ, two modules in

    now += 200u;
    r = runPass(p, s, true, now, true);
    CHECK(!r.begun);                             // NOT restarted
    CHECK_EQ(r.act, FSCAN_ASK);
    CHECK_EQ(r.ecu, 2);                          // carried on where it was
    CHECK(p.requested);                          // and still owed

    while (faultsScanning(s)) {
      now += 200u;
      r = runPass(p, s, true, now, true);
    }
    CHECK(r.ended);
    CHECK_EQ(p.dueMs, now + FAULT_SCAN_PERIOD_MS);   // completion moved it

    // The held request wins over the deadline: the very next pass begins a
    // fresh walk even though five minutes have not passed.
    now += 200u;
    r = runPass(p, s, true, now, true);
    CHECK(r.begun);
    CHECK_EQ(r.act, FSCAN_ASK);
    CHECK_EQ(r.ecu, 0);
    CHECK(!p.requested);                         // consumed once, not twice
  }

  // -------------------------------------------------------------------------
  // MILLISECOND WRAPAROUND
  //
  // esp_timer's millisecond count rolls over every 49.7 days, and this
  // instrument has no reason to be power-cycled. A deadline stamped just
  // before the rollover lands just after it, where `nowMs >= dueMs` reads as
  // "long past due" for every timestamp LEADING UP to the wrap -- so an
  // absolute compare would scan on every pass for the rest of that cycle and
  // then, once past the wrap, be correct again with nothing to show that
  // anything had ever been wrong. The signed delta is right on both sides.
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);

    const uint32_t finished = 0xFFFFFF00u;       // 256 ms before the rollover
    faultScanNoteFinished(p, false, finished);
    const uint32_t due = finished + FAULT_SCAN_PERIOD_MS;   // wrapped
    CHECK_EQ(p.dueMs, due);
    CHECK(due < finished);                       // it really did wrap

    uint8_t ecu = 0xFFu;
    // Before the rollover: NOT due, though unsigned it looks long past.
    CHECK(!faultScanDue(p, finished));
    CHECK(!faultScanDue(p, 0xFFFFFFFEu));
    CHECK(finished > p.dueMs);                   // what the naive test sees
    CHECK_EQ(faultScanDecide(p, true, false, 0xFFu, (uint8_t)ECU_COUNT,
                             0xFFFFFFFEu, ecu),
             FSCAN_NOTHING);

    // Across the rollover, still not due -- only 256 ms of the five minutes
    // has actually elapsed.
    CHECK(!faultScanDue(p, 0u));
    CHECK(!faultScanDue(p, due - 1u));

    // And it fires on time, on the far side.
    CHECK(faultScanDue(p, due));
    CHECK_EQ(faultScanDecide(p, true, false, 0xFFu, (uint8_t)ECU_COUNT, due, ecu),
             FSCAN_BEGIN);
  }

  // -------------------------------------------------------------------------
  // A whole walk straddling the rollover
  //
  // The same arithmetic, but with the real Faults.h cursor running through it:
  // twelve passes whose timestamps cross 0xFFFFFFFF, ending in a deadline that
  // is itself on the other side of the wrap from the pass that set it.
  //
  // The plan is seeded from a COMPLETED walk rather than from init, because a
  // fresh plan's dueMs of 0 is deliberately not due this late in the cycle --
  // the signed delta reads 0 as 24.8 days in the future from here. That is the
  // one documented cost of using 0 for "as soon as the link allows", it is
  // checked below, and it is not the case under test here.
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);
    CHECK(!faultScanDue(p, 0xFFFFFF00u));        // the quirk, stated
    faultScanNoteFinished(p, true, 0xFFFFFEF0u); // ... so: owed from here
    FaultSet s;
    faultsInit(s);

    uint32_t now = 0xFFFFFF00u;
    Pass r = runPass(p, s, true, now, true);
    CHECK(r.begun);
    CHECK_EQ(r.act, FSCAN_ASK);
    uint8_t asked = 1u;
    while (faultsScanning(s)) {
      now += 100u;                               // sails through zero
      r = runPass(p, s, true, now, true);
      CHECK_EQ(r.act, FSCAN_ASK);
      CHECK_EQ(r.ecu, asked);
      ++asked;
    }
    CHECK_EQ(asked, (int)ECU_COUNT);
    CHECK(r.ended);
    CHECK(now < 0xFFFFFF00u);                    // wrapped mid-walk
    CHECK_EQ(faultsAnsweredCount(s), (int)ECU_COUNT);
    CHECK_EQ(p.dueMs, (uint32_t)(now + FAULT_SCAN_PERIOD_MS));
    CHECK(!faultScanDue(p, now));
    CHECK(faultScanDue(p, now + FAULT_SCAN_PERIOD_MS));
  }

  // -------------------------------------------------------------------------
  // A link-down abandonment at the rollover is still owed at once
  //
  // dueMs = now, and `now` here is a value an absolute compare would call
  // "not yet" for the whole of the next cycle.
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);
    faultScanNoteFinished(p, true, 0xFFFFFFFFu);
    CHECK_EQ(p.dueMs, 0xFFFFFFFFu);
    CHECK(faultScanDue(p, 0xFFFFFFFFu));
    CHECK(faultScanDue(p, 0u));                  // one ms later, after the wrap
    CHECK(faultScanDue(p, 5000u));
  }

  // -------------------------------------------------------------------------
  // Degenerate inputs decide nothing rather than spinning
  //
  // An empty table would otherwise produce a walk that finished the instant it
  // began -- and since the deadline moves only on a completion the CALLER
  // observes, it would be owed again on the very next pass, forever.
  // `scanning` true with nextEcu 0xFF is the caller's two arguments
  // disagreeing (both come off the same cursor), not a state of the walk, so
  // neither one is believed.
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);
    uint8_t ecu = 0xFFu;
    CHECK_EQ(faultScanDecide(p, true, false, 0xFFu, 0u, 1000u, ecu), FSCAN_NOTHING);
    CHECK(!p.requested);                         // not even armed
    CHECK_EQ(faultScanDecide(p, true, true, 0xFFu, (uint8_t)ECU_COUNT, 1000u, ecu),
             FSCAN_NOTHING);

    // A walk begun over a table that has since shrunk ends instead of asking
    // for a module nobody can name. Not SKIP: a module that does not exist did
    // not fail to answer.
    CHECK_EQ(faultScanDecide(p, true, true, 5u, 4u, 1000u, ecu), FSCAN_FINISH);
    CHECK_EQ(faultScanDecide(p, false, true, 5u, 4u, 1000u, ecu), FSCAN_FINISH);
  }

  // -------------------------------------------------------------------------
  // ASK is the only act that touches the bus, and it names the module
  // -------------------------------------------------------------------------
  {
    FaultScanPlan p;
    faultScanPlanInit(p);
    uint8_t ecu = 0xFFu;
    CHECK_EQ(faultScanDecide(p, true, true, 7u, (uint8_t)ECU_COUNT, 1000u, ecu),
             FSCAN_ASK);
    CHECK_EQ(ecu, 7);
    ecu = 0xFFu;
    CHECK_EQ(faultScanDecide(p, false, true, 7u, (uint8_t)ECU_COUNT, 1000u, ecu),
             FSCAN_SKIP);
    CHECK_EQ(ecu, 7);                            // named either way
  }

TEST_MAIN_END()
