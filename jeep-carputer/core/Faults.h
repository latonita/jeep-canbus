#pragma once
// Stored diagnostic trouble codes, gathered one module at a time.
//
// Everything needed to READ a DTC already existed before this file: Kwp builds
// the 18 00 FF 00 request, parses the 58 <count> reply, and renders a raw code
// as its SAE string against 1152 verified pairs. What was missing was somewhere
// to put the answers and a way to walk the modules without blocking the bus.
//
// THE SCAN IS DELIBERATELY SLOW. This is a driver's instrument, not a parked
// diagnostic tool: a fault is read once and then looked at, so one module per
// pass of carTask costs nothing anyone can perceive and never competes with the
// monitor window. An unreachable module costs one 400 ms timeout and is
// recorded as unreachable rather than as clean -- the difference matters, since
// half these modules live on CAN-C and may not answer at all.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "Kwp.h"

// How many faults are kept.
//
// This car is known to hold sixteen; beyond that the set is marked full rather
// than silently dropping the rest, because "16 faults" and "at least 16 faults"
// are different statements and only one of them is true.
#define FAULT_MAX 16

struct Fault {
  uint16_t code;     // raw two-byte code as the module sent it
  uint8_t  status;   // raw status byte
  uint8_t  ecu;      // which module answered with it
  bool     muted;    // marked NOT IMPORTANT; see FaultMute
};

// A code the driver has decided not to care about.
//
// This vehicle has faults that are permanent and known -- a deleted EGR, a
// trailer module that is not fitted, a sensor whose circuit was cut years ago.
// They are real, they are stored, and they will never clear. Without a way to
// set them aside the fault count is a number that never changes and therefore
// never means anything, and the one time a NEW fault appears it is lost in the
// noise.
//
// Muting hides nothing: the fault stays in the list, still shows its code and
// its state, and can be unmuted with the same tap. What it does is take the
// code out of the COUNT, so the count goes back to meaning "something I have
// not already looked at".
//
// Matched on (ecu, code), not on the list position, because the position moves
// with every scan and the fault does not.
struct FaultMute {
  uint16_t code;
  uint8_t  ecu;
};

#define FAULT_MUTE_MAX 16

struct FaultSet {
  Fault    f[FAULT_MAX];
  uint8_t  count;
  bool     full;          // a module reported more than would fit

  FaultMute mute[FAULT_MUTE_MAX];
  uint8_t  muteCount;

  uint8_t  cursor;        // next module to ask; >= ecuCount when the scan is done
  uint8_t  ecuCount;      // how many modules this scan walks
  uint16_t answered;      // bit per module that replied
  uint16_t unreachable;   // bit per module that did not
  uint32_t doneMs;        // when the last scan finished; 0 = never scanned
};

void faultsInit(FaultSet& s);

// Start over. `ecuCount` is how many modules to walk, asked in index order.
void faultsBeginScan(FaultSet& s, uint8_t ecuCount);

bool faultsScanning(const FaultSet& s);

// Which module to ask next, or 0xFF when the scan is finished.
uint8_t faultsNextEcu(const FaultSet& s);

// Record one module's answer and advance. `n` records were parsed; `truncated`
// is what kwpParseDtcList reported.
void faultsRecord(FaultSet& s, uint8_t ecu, const DtcRecord* recs, uint8_t n,
                  bool truncated);

// Record that a module did not answer, and advance. NOT the same as a module
// answering with no faults -- a silent module is unknown, not clean.
void faultsEcuFailed(FaultSet& s, uint8_t ecu);

// Call when the cursor runs out, to stamp the scan.
void faultsEndScan(FaultSet& s, uint32_t nowMs);

// How many modules replied, and how many did not.
uint8_t faultsAnsweredCount(const FaultSet& s);
uint8_t faultsUnreachableCount(const FaultSet& s);

// Faults that have NOT been muted. This is the number HEALTH shows, and the
// only count anyone should act on.
uint8_t faultsActiveCount(const FaultSet& s);

bool faultsIsMuted(const FaultSet& s, uint8_t ecu, uint16_t code);

// Toggle the fault at `index`. Returns its new muted state. Silently does
// nothing when the mute table is full, which is reported by faultsMuteFull().
bool faultsToggleMute(FaultSet& s, uint8_t index);
bool faultsMuteFull(const FaultSet& s);

// Drop the fault at `index` from the list, keeping the rest packed and in the
// order faultsEndScan() put them in, and take one off `count`. `index` past
// the end is a silent no-op, like faultsToggleMute().
//
// This is the ERASE half of the drill-down, called once a module has been told
// to clear the code -- so it is the one thing that makes the count go down for
// real. Muting only takes a fault out of faultsActiveCount(); the row stays,
// because a list that hid it would be a lie. Removing takes the row itself.
//
// The MUTE for that (ecu, code) deliberately SURVIVES. A code that comes back
// after being erased is the same fault the driver has already looked at and
// called not important, and making them say so a second time is how a mute
// stops being trusted. The table is keyed on (ecu, code), not on a position,
// exactly so it can outlive the row.
//
// `full` is left alone too: it records that a SCAN overflowed, which stays
// true about that scan however many rows are removed afterwards. Clearing it
// on the way down would turn "at least 16 faults were found" into "15 faults
// were found", and the first of those is the true one. Same for `answered`,
// `unreachable`, `ecuCount` and `doneMs` -- all facts about the walk, not
// about the list it produced.
void faultsRemove(FaultSet& s, uint8_t index);

// Find a fault by (ecu, code). Returns its index, or 0xFF if it is not here.
//
// This is how every REQUEST from the screen names a fault, and it exists as a
// function because both the mute and the clear path had grown their own copy
// of the same four-line loop -- which is three lines of agreement away from
// the two controls disagreeing about which fault they are looking at.
//
// By (ecu, code) rather than by list position because the list is re-sorted at
// the end of every scan, and the renderer reads a published snapshot that can
// be a pass behind the live set on top of that. An index names whatever has
// since moved into that row; for the erase, that is the one mistake that
// cannot be undone.
uint8_t faultsFind(const FaultSet& s, uint8_t ecu, uint16_t code);
