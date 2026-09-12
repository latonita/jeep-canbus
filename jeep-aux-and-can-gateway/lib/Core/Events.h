#pragma once
// Modules report state changes as (code, value) pairs rather than strings.
// Rationale: on AVR a string literal passed at runtime lands in RAM, and we
// have 2 KB. AtServer owns the single code->tag table and does the formatting.
#include <stdint.h>

enum EventCode : uint8_t {
  EV_NONE = 0,
  EV_RADIO_MODE,   // value = raw mode nibble from 0x09F
  EV_AUX_STATE,    // value = 1 when the radio has AUX/VES selected
  EV_BT_POWER,     // value = 1 when the BT module is powered
  EV_BUTTON,       // value = button bitmask from 0x394 byte 3
  EV_BT_COMMAND,   // value = OvcCommand actually sent to the BT module
  EV_CAN_STATE,    // value = CanStatus
  EV_BENCH_MODE,   // value = 1 when bench announce is active
  EV_SAFE_STATE,   // value = 1 on entry, 0 on recovery
};

class IEventSink {
 public:
  virtual void emit(EventCode code, uint16_t value) = 0;

 protected:
  ~IEventSink() = default;
};

// Null sink so modules always have a valid target and never test for NULL.
class NullEventSink : public IEventSink {
 public:
  void emit(EventCode, uint16_t) override {}
};
