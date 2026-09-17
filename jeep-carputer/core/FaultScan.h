#pragma once
// WHEN to walk the modules for stored codes, and whether to ask at all.
//
// The walk itself lives in Faults.h -- which module is next, what each one
// answered, and the difference between a module with no faults and a module
// that said nothing. What was still untested was the POLICY around it: when a
// walk becomes owed, what an explicit RE-READ is allowed to interrupt, and how
// the deadline moves once a walk ends. That lived in main.cpp's faultScanStep()
// wrapped around an ISO-TP transaction and four ESP_LOGs, so none of it could
// be exercised without a board, an adapter and a car -- and the one rule in it
// that matters most (a walk abandoned with the link down is owed AGAIN
// IMMEDIATELY, not in five minutes) is invisible from the driver's seat until
// the day the link drops mid-walk and the fault list silently stops updating
// for five minutes at a time.
//
// So this file is the decision and nothing else. DECIDE HERE, EXECUTE THERE:
// faultScanDecide() is handed the clock, the link state and the walk's own
// answer to "which module next", and returns one act. The caller does the
// transaction, the logging, the mute and the clear -- all of which need the
// bus, flash or a console, and none of which are decisions.
//
// It owns no clock. nowMs is passed in, exactly like PollScheduler::next(),
// because a module that reads the clock itself can only be tested in real
// time and the interesting cases here are five minutes and 49 days apart.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

// How often the scan repeats itself once the link is up.
//
// Five minutes, on the owner's instruction; it used to be manual and nothing
// else. It is affordable because the walk is spread one module per pass of
// carTask -- twelve modules is about twelve seconds of a bus that is otherwise
// being listened to, so repeating it costs nothing anyone can perceive.
//
// Token-identical to the definition this replaced in main.cpp, so the two can
// coexist through the wiring without a redefinition warning. Delete that one.
#define FAULT_SCAN_PERIOD_MS (5u * 60u * 1000u)

struct FaultScanPlan {
  // When the next automatic walk is owed. Compared SIGNED against nowMs, so
  // the 49-day millisecond wrap needs no special case and 0 means "as soon as
  // the link allows".
  //
  // Moved ONLY by faultScanNoteFinished(). That is the whole mechanism behind
  // the re-owed-immediately rule: a walk that is cut short does not reach a
  // completion path, so the deadline it was started against still stands.
  uint32_t dueMs;

  // An explicit RE-READ is waiting. Set by faultScanRequest() from whichever
  // task the control lives on, consumed by faultScanDecide() when it hands out
  // FSCAN_BEGIN.
  //
  // It is NOT cleared when it cannot be honoured. A RE-READ pressed during a
  // walk stays owed and starts a fresh walk the moment this one ends, which is
  // what the driver asked for -- they pressed it because they want a reading
  // taken AFTER the press, and the walk in flight was already half-taken
  // before it.
  bool requested;
};

enum FaultScanAct : uint8_t {
  FSCAN_NOTHING = 0,  // nothing owed; do not touch the bus
  FSCAN_BEGIN,        // caller should faultsBeginScan(ecuCount), then decide again
  FSCAN_ASK,          // caller should transact with module `ecu`
  FSCAN_SKIP,         // caller should faultsEcuFailed(ecu) WITHOUT transacting
  FSCAN_FINISH,       // the cursor is past the table; end the walk
};

// dueMs 0, nothing requested: the first walk is owed as soon as the link comes
// up, and not before.
//
// NOT `requested = true`. A request is honoured whether or not the link is up,
// by design -- RE-READ with a dead link has to do something visible rather
// than sit unconsumed -- so pre-arming one here would run a full walk at boot
// with the link still down, mark all twelve modules unreachable without asking
// any of them, and hand the driver a completed scan that questioned nothing.
// Worse, it would do it in the seconds before the answer became obtainable.
// Waiting on the link is what makes "once when the link comes up" and "every
// five minutes" one code path.
//
// The one cost of dueMs 0 is honest and bounded: a board that runs 24.8 days
// without ever seeing the link go up has a deadline that reads as being in the
// FUTURE under the signed compare, and self-start waits out the rest of the
// wrap. RE-READ still works throughout, and a link that has been dead for
// three weeks is not a scan-scheduling problem.
void faultScanPlanInit(FaultScanPlan& p);

// RE-READ. Safe to call from another task: one bool, written once, and losing
// a race with the consumer costs one delayed walk rather than a corrupt one.
void faultScanRequest(FaultScanPlan& p);

// What to do on this pass of carTask.
//
// `scanning` is faultsScanning() and `nextEcu` is faultsNextEcu()'s answer
// (0xFF when no walk is in flight) -- passed in rather than reached for, so
// this stays independent of the FaultSet and testable against states the walk
// cannot easily be put into. `ecuCount` is how many modules the caller's table
// holds. `ecu` is written only for FSCAN_ASK and FSCAN_SKIP.
//
// ONE act per call, and FSCAN_BEGIN does NOT consume the pass: the caller
// begins the walk and calls this again, which is how the pass that starts a
// scan also asks module 0 -- exactly what faultScanStep() did as one straight
// line of code. Beginning is the caller's job because faultsBeginScan() owns
// the mute table, the previous scan's stamp and the ecuCount clamp, none of
// which belong to a scheduler.
//
// Returning FSCAN_BEGIN consumes `requested`, so the caller MUST begin the
// walk. Dropping it is not silently fatal -- the deadline has not moved, so
// self-start owes another one on the next pass -- but the RE-READ that asked
// for it is gone.
FaultScanAct faultScanDecide(FaultScanPlan& p, bool linkUp, bool scanning,
                             uint8_t nextEcu, uint8_t ecuCount,
                             uint32_t nowMs, uint8_t& ecu);

// Call once the caller has seen the walk end -- faultsScanning() has gone
// false after recording the last module, or FSCAN_FINISH came back.
//
// `answeredNothing` is the abandoned case: the walk ended on the link-down
// skip path, so every module in it is unreachable and nothing was asked.
// Then the next walk is owed NOW rather than in five minutes, because the
// result is worthless and the self-start gate holds it off until the link is
// genuinely back anyway. Leave it false and a link that drops mid-walk costs
// the driver a five-minute hole with an all-quiet fault list in it, which is
// the frozen-but-plausible failure on the one screen whose job is saying what
// is wrong.
void faultScanNoteFinished(FaultScanPlan& p, bool answeredNothing,
                           uint32_t nowMs);

// Is a walk owed right now? Pure predicate, no state change -- for a caller
// that wants to say so on screen, and for the tests.
bool faultScanDue(const FaultScanPlan& p, uint32_t nowMs);
