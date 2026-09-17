#include "PollDecode.h"

#include "TpmsSignals.h"

uint8_t pollDecodeResponse(uint8_t ecu, uint16_t localId, const IsoTpPdu& pdu,
                           VehicleState& state, uint32_t nowMs,
                           PollDecodeSink sink) {
  // BOTH tables, every time. See the header: the generated table cannot
  // contain the hand-written TPMS definitions, and walking only one of them
  // would leave the tyre page dashed with every other part of the system
  // reporting success.
  const SignalDef* tables[2] = {kSignals, kTpmsSignals};
  const uint16_t counts[2] = {kSignalCount, (uint16_t)kTpmsSignalCount};

  uint8_t published = 0u;

  for (uint8_t tbl = 0u; tbl < 2u; ++tbl) {
    for (uint16_t i = 0u; i < counts[tbl]; ++i) {
      const SignalDef& def = tables[tbl][i];
      if (def.ecu != ecu || def.localId != localId) {
        continue;
      }
      // Not registered means not in the poll plan, which is a normal
      // condition rather than an error: the tables describe every signal this
      // vehicle has, and `kPlan` asks for a handful of them.
      const int8_t slot = state.indexOf(&def);
      if (slot < 0) {
        continue;
      }

      DecodedValue v;
      const bool ok = (decodeSignal(def, pdu, v) == DECODE_OK) && v.valid;
      if (ok) {
        state.publish(slot, v.value, v.raw, v.label, nowMs);
        ++published;
      } else {
        // Counted, not skipped. A signal that is defined, asked for, answered
        // and then fails to decode is the interesting kind of broken -- it
        // means the definition disagrees with the module -- and a silent
        // `continue` here is how that stays invisible for months.
        state.noteError(slot);
      }
      if (sink != 0) {
        sink(def, v, ok);
      }
    }
  }

  return published;
}
