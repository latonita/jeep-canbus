#pragma once
// Tyre pressures, hand-written because the vendor database does not have them.
//
// The WCM answers an undocumented service-21 range that no generated table can
// describe: `21 3B` returns all four road wheels, and `21 20`..`21 24` return
// one sensor each (the fifth being the spare). Since tools/gen_signals.py can
// only emit what the database contains, these live here instead -- same
// SignalDef shape, so they register in VehicleState and decode through the same
// path as everything else.
//
// Decode, confirmed three independent ways (owner's expected range, one
// genuinely soft corner, and a stored C0077 "Low Tire Pressure" on the WCM):
// each byte is ONE PSI PER BIT, no offset, no scaling.
//
// Corner order is UNMAPPED. `21 3B` byte k matches `21 (0x1F+k)`'s pressure
// byte in 11 of 11 recorded pairs, so the four bytes are sensors 1..4 in index
// order -- but which physical corner each sensor sits on is unknown, and
// nothing in the captures reveals it. They are therefore named by sensor index,
// not by corner. Deflating one wheel and watching which value moves is the only
// way to map them; do not guess.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "SignalDb.h"

// Four road wheels from one request. Prefer this over polling 21 20..23
// separately: it is the same data in a quarter of the bus traffic.
extern const SignalDef kTpmsSignals[];
extern const uint8_t kTpmsSignalCount;

// The local identifier the above are read from, in the service-qualified form
// the poll plan uses.
#define TPMS_ROAD_WHEELS_PID 0x213Bu

// Reported pressure is psi; the cabin reads bar.
#define TPMS_PSI_TO_BAR 0.0689476f
