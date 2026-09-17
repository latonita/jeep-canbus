#pragma once
// One response, every signal in it.
//
// A KWP `21 xx` reply is not one value. The WCM's `21 3B` carries four tyre
// pressures; the CCN's `21 3F` carries a wall of warning lamps. So a poll is
// "ask for a PID, then publish every signal defined against that PID" -- a
// fan-out across the signal tables, not a single decode.
//
// This lived in `pollOne()` in main.cpp, which is the reason it is here now.
// Every part of it was already pure -- `decodeSignal` is core, `VehicleState`
// is core, both signal tables are core -- and it was sitting in an ESP-IDF
// translation unit only because the transaction that fetches the PDU sits next
// to it. The transaction is I/O and stays there; the fan-out is a rule and
// belongs where it can be tested.
//
// The rule that actually needs testing is the TWO TABLES. `kSignals` is
// generated from the vendor database; `kTpmsSignals` is hand-written, because
// the WCM's per-wheel PID is undocumented and a generator can only emit what
// its source contains. A fan-out that walked one table and not the other would
// lose the hero feature of this instrument and lose it SILENTLY -- the poll
// would succeed, the response would be correct, and the tyre page would simply
// stay dashed. That is exactly the failure this project is written to avoid,
// and it is one `for` loop away at all times.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "Decoder.h"
#include "IsoTp.h"
#include "SignalDb.h"
#include "VehicleState.h"

// Called for every signal the fan-out touched, decoded or not, in table order.
// `v` is valid only when `ok` is true. This exists so main.cpp can keep its
// logging -- which is the whole reason the fan-out used to have to live there.
typedef void (*PollDecodeSink)(const SignalDef& def, const DecodedValue& v, bool ok);

// Decode and publish every signal defined against (`ecu`, `localId`).
//
// Returns how many were published. ZERO is the caller's "this poll achieved
// nothing" answer, and it is deliberately not the same as a transport failure:
// the module answered, the response parsed, and either no signal is defined
// against that PID or every one of them failed to decode. The poll scheduler
// wants to back off on both, but the log lines are different and a future
// caller may want to tell them apart.
//
// A signal that fails to decode gets `noteError()` rather than being skipped
// silently, so the error counters on the HEALTH page mean something.
//
// `sink` may be null.
uint8_t pollDecodeResponse(uint8_t ecu, uint16_t localId, const IsoTpPdu& pdu,
                           VehicleState& state, uint32_t nowMs,
                           PollDecodeSink sink);
