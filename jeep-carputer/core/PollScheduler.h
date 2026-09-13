#pragma once
// Decides what to ask for next.
//
// Round-robin over a static plan, each entry with its own period, plus
// exponential backoff on failure so one dead module cannot eat the whole
// budget. Deliberately owns no link and no clock: next() is given the time and
// returns what to poll, which makes the whole policy unit-testable.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "SignalDb.h"

#define POLL_MAX_ITEMS 24u

// Failure backoff: period doubles per consecutive failure up to this many
// times, then stays there until something succeeds.
#define POLL_MAX_BACKOFF_SHIFT 5u

struct PollItem {
  uint8_t  ecu;         // EcuId
  uint16_t localId;
  uint16_t periodMs;
};

struct PollEntry {
  PollItem item;
  uint32_t lastAttemptMs;
  uint32_t lastOkMs;
  uint8_t  failures;     // consecutive
  bool     everOk;
};

class PollScheduler {
 public:
  PollScheduler();

  bool add(const PollItem& item);

  // The next item due at nowMs, or NULL when nothing is. Marks the returned
  // entry as attempted, so calling twice does not hand out the same item.
  const PollItem* next(uint32_t nowMs);

  // Report the outcome of the item most recently handed out.
  void noteResult(bool ok, uint32_t nowMs);

  // Stop handing anything out -- used while the ignition is off or the link is
  // down, so we neither crowd the bus the radio depends on nor spin on a dead
  // UART.
  void setSuspended(bool suspended) { suspended_ = suspended; }
  bool suspended() const { return suspended_; }

  uint8_t count() const { return count_; }
  const PollEntry* entry(uint8_t index) const;

  // Effective period for an entry, including any backoff. Exposed for tests
  // and for a diagnostics screen.
  uint32_t effectivePeriodMs(uint8_t index) const;

 private:
  PollEntry entries_[POLL_MAX_ITEMS];
  uint8_t   count_;
  int8_t    inFlight_;   // index handed out by next(), -1 when none
  bool      suspended_;
};
