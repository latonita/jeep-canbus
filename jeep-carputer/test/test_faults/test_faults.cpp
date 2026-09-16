// Host tests for core/Faults.cpp.
//
// The distinction worth protecting: a module that DID NOT ANSWER is not a
// module with no faults. Half of these live on CAN-C and may never reply, and
// reporting "0 faults" for a bus we could not reach would be the most
// confidently wrong thing this instrument could say.
#include "doctest_min.h"

#include "Faults.h"

static DtcRecord rec(uint16_t code, uint8_t status) {
  DtcRecord r;
  r.code = code;
  r.status = status;
  return r;
}

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // A fresh set has never scanned, and is not scanning
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    CHECK(!faultsScanning(s));
    CHECK(s.doneMs == 0u);
    CHECK(s.count == 0u);
    CHECK(faultsNextEcu(s) == 0xFFu);
  }

  // -------------------------------------------------------------------------
  // A scan walks the modules in order and collects what they report
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 3u);
    CHECK(faultsScanning(s));
    CHECK(faultsNextEcu(s) == 0u);

    const DtcRecord a[2] = {rec(0x0119u, 0x21u), rec(0x0234u, 0x21u)};
    faultsRecord(s, 0u, a, 2u, false);
    CHECK(faultsNextEcu(s) == 1u);
    CHECK(s.count == 2u);

    faultsEcuFailed(s, 1u);                 // silent module
    CHECK(faultsNextEcu(s) == 2u);

    const DtcRecord c[1] = {rec(0x1234u, 0x2Fu)};
    faultsRecord(s, 2u, c, 1u, false);
    CHECK(!faultsScanning(s));              // walked them all
    CHECK(faultsNextEcu(s) == 0xFFu);

    faultsEndScan(s, 50000u);
    CHECK(s.doneMs == 50000u);
    CHECK(s.count == 3u);
    CHECK(faultsAnsweredCount(s) == 2u);
    CHECK(faultsUnreachableCount(s) == 1u);
    // Which module each came from is kept -- the same code means different
    // things from different modules.
    CHECK(s.f[0].ecu == 0u);
    CHECK(s.f[0].code == 0x0119u);
    CHECK(s.f[2].ecu == 2u);
  }

  // -------------------------------------------------------------------------
  // Answering with NOTHING is a clean module; answering not at all is not
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 2u);
    faultsRecord(s, 0u, nullptr, 0u, false);   // replied, no faults
    faultsEcuFailed(s, 1u);                    // never replied
    faultsEndScan(s, 1000u);
    CHECK(s.count == 0u);
    CHECK(faultsAnsweredCount(s) == 1u);
    CHECK(faultsUnreachableCount(s) == 1u);
    CHECK(s.answered != s.unreachable);
  }

  // -------------------------------------------------------------------------
  // The same code from the same module twice is one fault
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 2u);
    const DtcRecord dup[3] = {rec(0x0119u, 0x21u), rec(0x0119u, 0x21u),
                              rec(0x0234u, 0x21u)};
    faultsRecord(s, 0u, dup, 3u, false);
    CHECK(s.count == 2u);
    // ...but the same code from a DIFFERENT module is a different fault.
    const DtcRecord same[1] = {rec(0x0119u, 0x21u)};
    faultsRecord(s, 1u, same, 1u, false);
    CHECK(s.count == 3u);
  }

  // -------------------------------------------------------------------------
  // Overflow is reported, not hidden
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 1u);
    DtcRecord many[FAULT_MAX + 4];
    for (uint8_t i = 0u; i < FAULT_MAX + 4; ++i) {
      many[i] = rec((uint16_t)(0x1000u + i), 0x21u);
    }
    faultsRecord(s, 0u, many, FAULT_MAX + 4, false);
    CHECK(s.count == FAULT_MAX);
    CHECK(s.full);                          // "at least 16", not "16"
  }

  // The module's own truncation flag sets it too, even when we had room.
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 1u);
    const DtcRecord one[1] = {rec(0x0119u, 0x21u)};
    faultsRecord(s, 0u, one, 1u, true);
    CHECK(s.count == 1u);
    CHECK(s.full);
  }

  // -------------------------------------------------------------------------
  // A re-scan keeps the previous stamp until it finishes
  //
  // Otherwise the display blanks for the twelve seconds a walk takes, and the
  // driver is told "never scanned" about a car that was scanned a minute ago.
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 1u);
    const DtcRecord one[1] = {rec(0x0119u, 0x21u)};
    faultsRecord(s, 0u, one, 1u, false);
    faultsEndScan(s, 10000u);
    CHECK(s.doneMs == 10000u);

    faultsBeginScan(s, 1u);
    CHECK(s.doneMs == 10000u);              // still the old stamp
    CHECK(s.count == 0u);                   // but the findings are cleared
    faultsEcuFailed(s, 0u);
    faultsEndScan(s, 60000u);
    CHECK(s.doneMs == 60000u);
  }

  // -------------------------------------------------------------------------
  // A scan stamped at tick 0 still reads as "scanned"
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 1u);
    faultsEcuFailed(s, 0u);
    faultsEndScan(s, 0u);
    CHECK(s.doneMs != 0u);
  }

  // -------------------------------------------------------------------------
  // Muting takes a fault out of the COUNT without taking it out of the list
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 1u);
    const DtcRecord r3[3] = {rec(0x0119u, 0x21u), rec(0x0234u, 0x21u),
                             rec(0x1234u, 0x21u)};
    faultsRecord(s, 0u, r3, 3u, false);
    faultsEndScan(s, 1000u);
    CHECK(faultsActiveCount(s) == 3u);

    CHECK(faultsToggleMute(s, 1u));          // now muted
    CHECK(s.f[1].muted);
    CHECK(s.count == 3u);                    // still listed
    CHECK(faultsActiveCount(s) == 2u);       // but not counted

    CHECK(!faultsToggleMute(s, 1u));         // and back again
    CHECK(!s.f[1].muted);
    CHECK(faultsActiveCount(s) == 3u);
  }

  // -------------------------------------------------------------------------
  // A mute SURVIVES a re-scan, and follows the code rather than the position
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 1u);
    const DtcRecord a[2] = {rec(0x0119u, 0x21u), rec(0x0234u, 0x21u)};
    faultsRecord(s, 0u, a, 2u, false);
    faultsEndScan(s, 1000u);
    faultsToggleMute(s, 0u);                 // mute P0119
    CHECK(faultsActiveCount(s) == 1u);

    // Re-scan. The same code comes back in a DIFFERENT position -- the new
    // fault is a B-code, which SORTS ahead of it, so the muted P0119 moves
    // from index 0 to index 1 whatever order the module reported them in.
    faultsBeginScan(s, 1u);
    CHECK(s.muteCount == 1u);                // the decision survived the wipe
    const DtcRecord b[3] = {rec(0x9851u, 0x21u), rec(0x0234u, 0x21u),
                            rec(0x0119u, 0x21u)};
    faultsRecord(s, 0u, b, 3u, false);
    faultsEndScan(s, 2000u);
    CHECK(s.count == 3u);
    CHECK(s.f[0].code == 0x9851u);           // B1851 sorts first
    CHECK(!s.f[0].muted);                    // new code, not muted
    CHECK(s.f[1].code == 0x0119u);
    CHECK(s.f[1].muted);                     // moved, still muted
    CHECK(faultsActiveCount(s) == 2u);
  }

  // The same code from a DIFFERENT module is a different decision.
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 2u);
    const DtcRecord one[1] = {rec(0x0119u, 0x21u)};
    faultsRecord(s, 0u, one, 1u, false);
    faultsRecord(s, 1u, one, 1u, false);
    faultsEndScan(s, 1000u);
    faultsToggleMute(s, 0u);
    CHECK(s.f[0].muted);
    CHECK(!s.f[1].muted);
    CHECK(faultsActiveCount(s) == 1u);
  }

  // A full mute table refuses rather than silently dropping the oldest.
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 1u);
    DtcRecord many[FAULT_MUTE_MAX + 1];
    for (uint8_t i = 0u; i < FAULT_MUTE_MAX + 1; ++i) {
      many[i] = rec((uint16_t)(0x2000u + i), 0x21u);
    }
    faultsRecord(s, 0u, many, FAULT_MUTE_MAX + 1, false);
    for (uint8_t i = 0u; i < FAULT_MUTE_MAX; ++i) {
      CHECK(faultsToggleMute(s, i));
    }
    CHECK(faultsMuteFull(s));
    // The 17th record does not exist to mute: FAULT_MAX == FAULT_MUTE_MAX, so
    // a fault beyond the mute table cannot be stored in the first place. The
    // list says so instead. (This used to index s.f[FAULT_MUTE_MAX], which is
    // one past the end of the array -- a read the compiler warned about.)
    CHECK(s.count == FAULT_MAX);
    CHECK(s.full);
    CHECK(!faultsToggleMute(s, FAULT_MAX));        // out of range, refused
  }

  // -------------------------------------------------------------------------
  // The finished list is ordered by the code as DRAWN, across modules
  //
  // Raw order would be P, C, B, U -- kwpDtcToSae takes its letter from "PCBU"
  // -- so this is also what proves the sort is on the rendered string and not
  // on the uint16_t.
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 3u);
    DtcRecord a[2] = {rec(0x0119u, 0x20u),    // P0119
                      rec(0x213Au, 0x20u)};   // P213A
    faultsRecord(s, 0u, a, 2u, false);
    DtcRecord b[1] = {rec(0x9851u, 0x20u)};   // B1851
    faultsRecord(s, 1u, b, 1u, false);
    DtcRecord c[1] = {rec(0xD504u, 0x20u)};   // U1504
    faultsRecord(s, 2u, c, 1u, false);
    // Still scan order while the walk is running: the list must not reorder
    // under a finger.
    CHECK(s.f[0].code == 0x0119u);
    CHECK(s.f[2].code == 0x9851u);
    faultsEndScan(s, 1000u);
    CHECK(s.count == 4u);
    CHECK(s.f[0].code == 0x9851u);   // B1851
    CHECK(s.f[1].code == 0x0119u);   // P0119
    CHECK(s.f[2].code == 0x213Au);   // P213A
    CHECK(s.f[3].code == 0xD504u);   // U1504
    // The module each row came from travels with it.
    CHECK(s.f[0].ecu == 1u);
    CHECK(s.f[1].ecu == 0u);
    CHECK(s.f[3].ecu == 2u);
  }

  // -------------------------------------------------------------------------
  // Sorting carries the muted flag with the row it belongs to
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 2u);
    DtcRecord a[1] = {rec(0x213Au, 0x20u)};   // P213A, recorded first
    faultsRecord(s, 0u, a, 1u, false);
    DtcRecord b[1] = {rec(0x9851u, 0x20u)};   // B1851, sorts above it
    faultsRecord(s, 1u, b, 1u, false);
    CHECK(faultsToggleMute(s, 0u));           // mute P213A at index 0
    faultsEndScan(s, 1000u);
    CHECK(s.f[0].code == 0x9851u);
    CHECK(!s.f[0].muted);
    CHECK(s.f[1].code == 0x213Au);
    CHECK(s.f[1].muted);
    CHECK(faultsActiveCount(s) == 1u);
  }


  // -------------------------------------------------------------------------
  // Removing a row PACKS the list and keeps the sorted order
  //
  // This is the half of the drill-down that erases: muting only stops a fault
  // being counted, removing is what actually makes the list shorter. The
  // survivors must stay in the order faultsEndScan() put them in, because the
  // list is read off the touch y and a row that moved would hand the driver a
  // different fault than the one they were pointing at.
  // -------------------------------------------------------------------------
  {
    // B1851, P0119, P213A, U1504 once sorted -- the same four as the sort case.
    const DtcRecord all[4] = {rec(0x0119u, 0x20u), rec(0x213Au, 0x20u),
                              rec(0x9851u, 0x20u), rec(0xD504u, 0x20u)};

    // The FIRST row.
    {
      FaultSet s;
      faultsInit(s);
      faultsBeginScan(s, 1u);
      faultsRecord(s, 0u, all, 4u, false);
      faultsEndScan(s, 1000u);
      faultsRemove(s, 0u);
      CHECK(s.count == 3u);
      CHECK(s.f[0].code == 0x0119u);
      CHECK(s.f[1].code == 0x213Au);
      CHECK(s.f[2].code == 0xD504u);
    }

    // A MIDDLE row -- the one a swap-with-last would get wrong.
    {
      FaultSet s;
      faultsInit(s);
      faultsBeginScan(s, 1u);
      faultsRecord(s, 0u, all, 4u, false);
      faultsEndScan(s, 1000u);
      faultsRemove(s, 1u);
      CHECK(s.count == 3u);
      CHECK(s.f[0].code == 0x9851u);
      CHECK(s.f[1].code == 0x213Au);
      CHECK(s.f[2].code == 0xD504u);
    }

    // The LAST row.
    {
      FaultSet s;
      faultsInit(s);
      faultsBeginScan(s, 1u);
      faultsRecord(s, 0u, all, 4u, false);
      faultsEndScan(s, 1000u);
      faultsRemove(s, 3u);
      CHECK(s.count == 3u);
      CHECK(s.f[0].code == 0x9851u);
      CHECK(s.f[1].code == 0x0119u);
      CHECK(s.f[2].code == 0x213Au);
    }
  }

  // -------------------------------------------------------------------------
  // An index past the end does nothing at all
  //
  // The drill-down holds an index, and a re-read can shorten the list under
  // it, so a stale index must be survivable rather than a crash.
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 1u);
    const DtcRecord two[2] = {rec(0x0119u, 0x20u), rec(0x213Au, 0x20u)};
    faultsRecord(s, 0u, two, 2u, false);
    faultsEndScan(s, 1000u);
    faultsRemove(s, 2u);                     // one past the end
    faultsRemove(s, 0xFFu);                  // and well past it
    CHECK(s.count == 2u);
    CHECK(s.f[0].code == 0x0119u);
    CHECK(s.f[1].code == 0x213Au);
  }

  // -------------------------------------------------------------------------
  // Removing a MUTED row leaves the mute behind on purpose
  //
  // The code was erased, but if it comes back it is still the fault the driver
  // already decided was not important -- being asked again is how a mute stops
  // being trusted. So muteCount does not move, and a re-scan that finds the
  // code again finds it already muted.
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 1u);
    const DtcRecord r3[3] = {rec(0x9851u, 0x20u), rec(0x0119u, 0x20u),
                             rec(0x213Au, 0x20u)};
    faultsRecord(s, 0u, r3, 3u, false);
    faultsEndScan(s, 1000u);
    CHECK(faultsToggleMute(s, 1u));          // mute P0119
    CHECK(faultsActiveCount(s) == 2u);
    CHECK(s.muteCount == 1u);

    faultsRemove(s, 1u);                     // and erase it
    CHECK(s.count == 2u);
    CHECK(s.muteCount == 1u);                // the decision stays
    CHECK(faultsIsMuted(s, 0u, 0x0119u));
    CHECK(faultsActiveCount(s) == 2u);       // both survivors were unmuted
    CHECK(s.f[0].code == 0x9851u);
    CHECK(s.f[1].code == 0x213Au);

    // Removing an UNMUTED row moves the active count, which is the whole
    // difference between erasing and muting.
    faultsRemove(s, 0u);
    CHECK(s.count == 1u);
    CHECK(faultsActiveCount(s) == 1u);
  }

  // -------------------------------------------------------------------------
  // An emptied list still remembers the WALK
  //
  // `full`, answered/unreachable and doneMs are facts about the scan, not
  // about the rows it produced. "at least 16 were found" stays true after the
  // rows are cleared out, and a module that never replied is still unknown.
  // -------------------------------------------------------------------------
  {
    FaultSet s;
    faultsInit(s);
    faultsBeginScan(s, 3u);
    const DtcRecord a[2] = {rec(0x0119u, 0x20u), rec(0x213Au, 0x20u)};
    faultsRecord(s, 0u, a, 2u, true);        // module truncated its own reply
    faultsEcuFailed(s, 1u);
    const DtcRecord c[1] = {rec(0xD504u, 0x20u)};
    faultsRecord(s, 2u, c, 1u, false);
    faultsEndScan(s, 70000u);
    CHECK(s.count == 3u);

    faultsRemove(s, 0u);
    faultsRemove(s, 0u);
    faultsRemove(s, 0u);
    CHECK(s.count == 0u);
    CHECK(faultsActiveCount(s) == 0u);
    CHECK(faultsAnsweredCount(s) == 2u);
    CHECK(faultsUnreachableCount(s) == 1u);
    CHECK(s.ecuCount == 3u);
    CHECK(s.doneMs == 70000u);
    CHECK(s.full);                           // still true about that scan
    CHECK(!faultsScanning(s));
    faultsRemove(s, 0u);                     // and empty stays empty
    CHECK(s.count == 0u);
  }

TEST_MAIN_END()
