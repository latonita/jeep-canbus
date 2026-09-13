// Host tests for core/PollScheduler.cpp.
//
// PollScheduler owns no link and no clock, so every test here is just "given
// this plan and this nowMs, what does next() hand out".
//
// One behaviour worth stating up front, because several tests depend on it:
// PollEntry has no "never attempted" flag, so lastAttemptMs starts at 0 and
// due-ness is purely (now - lastAttempt) >= effectivePeriod. An item is
// therefore first polled one period after the epoch, not immediately at t = 0.
#include "doctest_min.h"

#include "PollScheduler.h"

static PollItem mkItem(uint8_t ecu, uint16_t localId, uint16_t periodMs) {
  PollItem it;
  it.ecu = ecu;
  it.localId = localId;
  it.periodMs = periodMs;
  return it;
}

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // An empty scheduler answers safely to everything
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    CHECK_EQ(ps.count(), 0);
    CHECK(!ps.suspended());
    CHECK(ps.next(0) == 0);
    CHECK(ps.next(1000000) == 0);
    CHECK(ps.entry(0) == 0);
    CHECK(ps.entry(255) == 0);
    CHECK_EQ(ps.effectivePeriodMs(0), 0);
    CHECK_EQ(ps.effectivePeriodMs(255), 0);
    // noteResult with nothing in flight must be a no-op, not a write.
    ps.noteResult(false, 100);
    ps.noteResult(true, 100);
    CHECK_EQ(ps.count(), 0);
  }

  // -------------------------------------------------------------------------
  // add() validation
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    CHECK(ps.add(mkItem(ECU_CCN, 0x213E, 1000)));
    CHECK_EQ(ps.count(), 1);
    const PollEntry* e = ps.entry(0);
    CHECK(e != 0);
    CHECK_EQ(e->item.ecu, ECU_CCN);
    CHECK_EQ(e->item.localId, 0x213E);
    CHECK_EQ(e->item.periodMs, 1000);
    CHECK_EQ(e->lastAttemptMs, 0);
    CHECK_EQ(e->lastOkMs, 0);
    CHECK_EQ(e->failures, 0);
    CHECK(!e->everOk);
    CHECK_EQ(ps.effectivePeriodMs(0), 1000);

    // A zero period is rejected: backoff doubles the period, and doubling 0
    // is still 0, so a dead module set up that way would starve everything.
    CHECK(!ps.add(mkItem(ECU_CCN, 0x2000, 0)));
    // An ecu id past the end of kEcus is rejected here rather than crashing
    // downstream.
    CHECK(!ps.add(mkItem((uint8_t)ECU_COUNT, 0x2000, 100)));
    CHECK(!ps.add(mkItem(200, 0x2000, 100)));
    CHECK_EQ(ps.count(), 1);
  }

  // -------------------------------------------------------------------------
  // add() beyond POLL_MAX_ITEMS
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    for (unsigned i = 0; i < POLL_MAX_ITEMS; ++i) {
      CHECK(ps.add(mkItem(ECU_CCN, (uint16_t)(0x2100u + i), 1000)));
    }
    CHECK_EQ(ps.count(), (int)POLL_MAX_ITEMS);
    CHECK(!ps.add(mkItem(ECU_CCN, 0x9999, 1000)));
    CHECK(!ps.add(mkItem(ECU_RADIO, 0x9998, 1000)));
    CHECK_EQ(ps.count(), (int)POLL_MAX_ITEMS);
    CHECK(ps.entry((uint8_t)(POLL_MAX_ITEMS - 1u)) != 0);
    CHECK(ps.entry((uint8_t)POLL_MAX_ITEMS) == 0);
  }

  // -------------------------------------------------------------------------
  // Nothing due yet; then due; then not handed out twice
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    CHECK(ps.add(mkItem(ECU_CCN, 0x213E, 1000)));
    CHECK(ps.next(0) == 0);
    CHECK(ps.next(500) == 0);
    CHECK(ps.next(999) == 0);

    const PollItem* it = ps.next(1000);
    CHECK(it != 0);
    CHECK_EQ(it->localId, 0x213E);
    CHECK_EQ(ps.entry(0)->lastAttemptMs, 1000);
    // Marked attempted, so a second call at the same instant gets nothing.
    CHECK(ps.next(1000) == 0);
    CHECK(ps.next(1999) == 0);
    CHECK(ps.next(2000) != 0);
  }

  // -------------------------------------------------------------------------
  // Several due at once: fair rotation, everyone served, nobody twice
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    CHECK(ps.add(mkItem(ECU_CCN, 0x0001, 1000)));
    CHECK(ps.add(mkItem(ECU_RADIO, 0x0002, 1000)));
    CHECK(ps.add(mkItem(ECU_ORC, 0x0003, 1000)));

    // All three come due on the same tick with identical ages.
    const PollItem* a = ps.next(1000);
    const PollItem* b = ps.next(1000);
    const PollItem* c = ps.next(1000);
    CHECK(a != 0 && b != 0 && c != 0);
    CHECK_EQ(a->localId, 0x0001);
    CHECK_EQ(b->localId, 0x0002);
    CHECK_EQ(c->localId, 0x0003);
    CHECK(ps.next(1000) == 0);      // budget for this tick is spent

    // ...and the cycle repeats one period later, in the same order.
    CHECK_EQ(ps.next(2000)->localId, 0x0001);
    CHECK_EQ(ps.next(2000)->localId, 0x0002);
    CHECK_EQ(ps.next(2000)->localId, 0x0003);
    CHECK(ps.next(2000) == 0);
  }

  // -------------------------------------------------------------------------
  // Most-overdue-first when periods differ
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    CHECK(ps.add(mkItem(ECU_CCN, 0x00FA, 1000)));    // slow
    CHECK(ps.add(mkItem(ECU_RADIO, 0x00FB, 100)));   // fast

    // At t = 1000 both have age 1000. Overdue amounts are 0 and 900, so the
    // fast item goes first even though it was added second.
    CHECK_EQ(ps.next(1000)->localId, 0x00FB);
    CHECK_EQ(ps.next(1000)->localId, 0x00FA);
    CHECK(ps.next(1000) == 0);

    // The fast item then gets served repeatedly without the slow one
    // starving: at t = 1100..1900 only the fast one is due.
    for (uint32_t t = 1100; t <= 1900; t += 100) {
      const PollItem* f = ps.next(t);
      CHECK(f != 0);
      if (f != 0) CHECK_EQ(f->localId, 0x00FB);
      CHECK(ps.next(t) == 0);
    }
    // ...and at t = 2000 the slow one comes back round.
    const PollItem* first = ps.next(2000);
    const PollItem* second = ps.next(2000);
    CHECK(first != 0 && second != 0);
    // Both are served on this tick; order is by overdue amount.
    const bool bothServed =
        (first->localId == 0x00FA && second->localId == 0x00FB) ||
        (first->localId == 0x00FB && second->localId == 0x00FA);
    CHECK(bothServed);
  }

  // -------------------------------------------------------------------------
  // Backoff growth and its cap
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    CHECK(ps.add(mkItem(ECU_PCM, 0x1234, 100)));

    CHECK_EQ(ps.effectivePeriodMs(0), 100);

    uint32_t t = 100;
    const PollItem* it = ps.next(t);
    CHECK(it != 0);
    ps.noteResult(false, t);
    CHECK_EQ(ps.entry(0)->failures, 1);
    CHECK_EQ(ps.effectivePeriodMs(0), 200);
    CHECK(!ps.entry(0)->everOk);

    // The doubled period really is respected.
    CHECK(ps.next(t + 199) == 0);
    CHECK(ps.next(t + 200) != 0);
    t += 200;
    ps.noteResult(false, t);
    CHECK_EQ(ps.entry(0)->failures, 2);
    CHECK_EQ(ps.effectivePeriodMs(0), 400);

    // Keep failing and watch the shift saturate at POLL_MAX_BACKOFF_SHIFT.
    const uint32_t expected[] = {800u, 1600u, 3200u, 3200u, 3200u, 3200u};
    for (unsigned k = 0; k < 6u; ++k) {
      t += ps.effectivePeriodMs(0);
      const PollItem* dead = ps.next(t);
      CHECK(dead != 0);
      ps.noteResult(false, t);
      CHECK_EQ(ps.effectivePeriodMs(0), (long long)expected[k]);
    }
    // 100 << 5 = 3200 is the cap; failures keep counting past the shift limit.
    CHECK_EQ(ps.effectivePeriodMs(0), (long long)(100u << POLL_MAX_BACKOFF_SHIFT));
    CHECK_EQ(ps.entry(0)->failures, 8);

    // Recovery resets straight back to the configured period.
    t += 3200;
    CHECK(ps.next(t) != 0);
    ps.noteResult(true, t);
    CHECK_EQ(ps.entry(0)->failures, 0);
    CHECK_EQ(ps.entry(0)->lastOkMs, (long long)t);
    CHECK(ps.entry(0)->everOk);
    CHECK_EQ(ps.effectivePeriodMs(0), 100);
    CHECK(ps.next(t + 99) == 0);
    CHECK(ps.next(t + 100) != 0);
  }

  // -------------------------------------------------------------------------
  // A dead module does not starve a healthy one
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    CHECK(ps.add(mkItem(ECU_CCN, 0x00AA, 100)));   // healthy
    CHECK(ps.add(mkItem(ECU_PCM, 0x00BB, 100)));   // dead

    unsigned healthy = 0;
    unsigned dead = 0;
    for (uint32_t t = 100; t <= 4000; t += 20) {
      const PollItem* it = ps.next(t);
      while (it != 0) {
        if (it->localId == 0x00AA) {
          ++healthy;
          ps.noteResult(true, t);
        } else {
          ++dead;
          ps.noteResult(false, t);
        }
        it = ps.next(t);
      }
    }
    CHECK(healthy > 30u);
    CHECK(dead > 0u);
    // The dead module has been backed off to at most 3200 ms, so over ~4 s it
    // must have been tried far fewer times than the healthy one.
    CHECK(dead * 4u < healthy);
  }

  // -------------------------------------------------------------------------
  // noteResult with nothing in flight
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    CHECK(ps.add(mkItem(ECU_CCN, 0x213E, 100)));
    // Before anything was handed out.
    ps.noteResult(false, 50);
    CHECK_EQ(ps.entry(0)->failures, 0);
    CHECK_EQ(ps.entry(0)->lastAttemptMs, 0);

    CHECK(ps.next(100) != 0);
    ps.noteResult(false, 110);
    CHECK_EQ(ps.entry(0)->failures, 1);
    CHECK_EQ(ps.entry(0)->lastAttemptMs, 110);   // re-stamped at completion

    // The same outcome reported twice must not double-count.
    ps.noteResult(false, 120);
    CHECK_EQ(ps.entry(0)->failures, 1);
    CHECK_EQ(ps.entry(0)->lastAttemptMs, 110);
    ps.noteResult(true, 130);
    CHECK_EQ(ps.entry(0)->failures, 1);
    CHECK(!ps.entry(0)->everOk);
  }

  // -------------------------------------------------------------------------
  // suspend / resume
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    CHECK(ps.add(mkItem(ECU_CCN, 0x213E, 100)));
    CHECK(ps.add(mkItem(ECU_RADIO, 0x214E, 100)));

    ps.setSuspended(true);
    CHECK(ps.suspended());
    CHECK(ps.next(100) == 0);
    CHECK(ps.next(100000) == 0);          // however overdue
    CHECK_EQ(ps.entry(0)->lastAttemptMs, 0);   // and nothing was marked

    ps.setSuspended(false);
    CHECK(!ps.suspended());
    CHECK(ps.next(100000) != 0);
    CHECK(ps.next(100000) != 0);
    CHECK(ps.next(100000) == 0);
  }

  // -------------------------------------------------------------------------
  // millis() rollover
  // -------------------------------------------------------------------------
  {
    PollScheduler ps;
    CHECK(ps.add(mkItem(ECU_CCN, 0x213E, 100)));

    // 64 ms before the wrap. An entry with lastAttemptMs == 0 is colossally
    // overdue here, so it is handed out and stamped just short of 2^32.
    const uint32_t base = 0xFFFFFFC0u;
    CHECK(ps.next(base) != 0);
    ps.noteResult(true, base);
    CHECK_EQ(ps.entry(0)->lastAttemptMs, (long long)base);

    // 50 ms later, still before the wrap: age 50 < 100, not due.
    CHECK(ps.next((uint32_t)(base + 50u)) == 0);
    // 99 ms later the clock has already wrapped to 0x23, but the age is still
    // 99, so it must STILL not be due. A naive lastAttempt + period compare
    // (0xFFFFFFC0 + 100 wraps to 0x24) would get this wrong in both
    // directions.
    CHECK_EQ((uint32_t)(base + 99u), 0x00000023u);
    CHECK(ps.next((uint32_t)(base + 99u)) == 0);
    // 150 ms later, wrapped to 0x56: age 150 >= 100, due.
    const uint32_t wrapped = (uint32_t)(base + 150u);
    CHECK_EQ(wrapped, 0x00000056u);
    CHECK(ps.next(wrapped) != 0);
    CHECK_EQ(ps.entry(0)->lastAttemptMs, 0x00000056);
    ps.noteResult(false, wrapped);
    CHECK_EQ(ps.effectivePeriodMs(0), 200);

    // The backed-off period is honoured on the far side of the wrap too:
    // lastAttempt is 0x56 = 86.
    CHECK(ps.next(0x000000C0u) == 0);   // age 192 - 86 = 106 < 200
    CHECK(ps.next(0x0000011Du) == 0);   // age 285 - 86 = 199 < 200
    CHECK(ps.next(0x0000011Eu) != 0);   // age 286 - 86 = 200 >= 200
  }

TEST_MAIN_END()
