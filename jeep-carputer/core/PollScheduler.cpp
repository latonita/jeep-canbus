// Decides what to ask for next: round-robin over a static plan with
// per-item periods and exponential backoff on failure.
//
// PURE: no ESP-IDF, no Arduino. Owns no link and no clock.
#include "PollScheduler.h"

#include <stdint.h>
#include <string.h>

// POLL_MAX_ITEMS is 24, so every valid index fits in the int8_t inFlight_ with
// -1 left over as the "nothing handed out" sentinel.
#if POLL_MAX_ITEMS > 127u
#error "POLL_MAX_ITEMS must stay <= 127: inFlight_ is an int8_t"
#endif

PollScheduler::PollScheduler() : count_(0), inFlight_(-1), suspended_(false) {
  memset(entries_, 0, sizeof(entries_));
}

bool PollScheduler::add(const PollItem& item) {
  if (count_ >= POLL_MAX_ITEMS) return false;
  // A zero period would mean "poll as fast as possible", and because backoff
  // works by doubling the period, doubling zero is still zero -- a dead module
  // configured that way would eat the entire budget forever, which is exactly
  // what the backoff exists to prevent. Reject it at configuration time.
  if (item.periodMs == 0u) return false;
  // The ecu field indexes kEcus downstream. Refuse an id that would run off
  // the end of that table rather than handing it out and crashing later.
  if (item.ecu >= (uint8_t)ECU_COUNT) return false;

  PollEntry& e = entries_[count_];
  memset(&e, 0, sizeof(e));
  e.item = item;
  // lastAttemptMs starts at 0, and PollEntry has no "never attempted" flag, so
  // due-ness is purely (now - lastAttempt) >= effectivePeriod. Consequence,
  // documented rather than worked around: an item added before the clock has
  // run for one period is first polled one period after the epoch, not
  // immediately. That is a few hundred milliseconds at boot and nobody notices.
  ++count_;
  return true;
}

const PollEntry* PollScheduler::entry(uint8_t index) const {
  if (index >= count_) return 0;
  return &entries_[index];
}

uint32_t PollScheduler::effectivePeriodMs(uint8_t index) const {
  if (index >= count_) return 0;
  const PollEntry& e = entries_[index];
  uint8_t shift = e.failures;
  if (shift > POLL_MAX_BACKOFF_SHIFT) shift = (uint8_t)POLL_MAX_BACKOFF_SHIFT;
  // periodMs <= 65535 and shift <= 5, so the product is at most 2 097 120 --
  // no uint32_t overflow, no need to clamp.
  return (uint32_t)e.item.periodMs << shift;
}

const PollItem* PollScheduler::next(uint32_t nowMs) {
  // Suspended: hand out nothing at all. Used with the ignition off or the link
  // down, so we neither crowd the 83.3 kbps bus the radio and cluster depend
  // on nor spin on a dead UART.
  if (suspended_) return 0;

  int8_t best = -1;
  uint32_t bestOverdue = 0;

  for (uint8_t i = 0; i < count_; ++i) {
    const PollEntry& e = entries_[i];
    // Rollover-safe: unsigned subtraction is modulo 2^32, so the age is right
    // across the millis() wrap. See the note in VehicleState::fresh.
    const uint32_t age = nowMs - e.lastAttemptMs;
    const uint32_t period = effectivePeriodMs(i);
    if (age < period) continue;                 // not due yet

    const uint32_t overdue = age - period;      // >= 0 by the test above
    // Most-overdue-first, measured in absolute milliseconds (not as a multiple
    // of the period). Ties -- the common case when several items share a
    // period and all come due on the same tick -- go to the lowest index; the
    // rotation still comes out fair because handing an item out stamps its
    // lastAttemptMs, which takes it out of contention until its next period.
    if (best < 0 || overdue > bestOverdue) {
      best = (int8_t)i;
      bestOverdue = overdue;
    }
  }

  if (best < 0) return 0;

  PollEntry& e = entries_[(uint8_t)best];
  e.lastAttemptMs = nowMs;   // marks it attempted: next() will not repeat it
  inFlight_ = best;
  return &e.item;
}

void PollScheduler::noteResult(bool ok, uint32_t nowMs) {
  // Nothing was handed out (or this outcome has already been reported): drop
  // it. Counting a result against whatever happened to be polled last would
  // corrupt the backoff of an innocent entry.
  if (inFlight_ < 0 || (uint8_t)inFlight_ >= count_) {
    inFlight_ = -1;
    return;
  }

  PollEntry& e = entries_[(uint8_t)inFlight_];
  // Re-stamp the attempt at completion, not at request time. A failing module
  // costs us a full transaction timeout, and if the period were still measured
  // from when we sent the request that timeout would already have consumed it
  // -- the backoff would double a period that had elapsed anyway and change
  // nothing. Callers should therefore pass a nowMs at or after the one they
  // gave next().
  e.lastAttemptMs = nowMs;

  if (ok) {
    e.failures = 0;       // recovery: straight back to the configured period
    e.lastOkMs = nowMs;
    e.everOk = true;
  } else if (e.failures != 0xFFu) {
    ++e.failures;         // saturate; effectivePeriodMs caps the shift anyway
  }

  inFlight_ = -1;
}
