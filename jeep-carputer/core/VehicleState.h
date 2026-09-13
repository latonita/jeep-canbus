#pragma once
// The hub. Sources publish here, presenters read from here and from nowhere
// else, so a new screen never touches protocol code.
//
// Fixed-size, no allocation, no locking: on the ESP32 a single poll task
// writes and the UI task reads. Slots are plain scalars, so a torn read is at
// worst one stale value, which is why every slot carries its own timestamp.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "SignalDb.h"

#define VS_MAX_SLOTS 48u

// A published value is only trustworthy for so long. Presenters should show a
// value as stale rather than pretending a five-minute-old RPM is current.
#define VS_DEFAULT_TTL_MS 5000u

struct StateSlot {
  const SignalDef* def;      // NULL for an unused slot
  float            value;
  uint32_t         raw;
  uint32_t         stampMs;  // when it was last published
  const char*      label;    // enum label, NULL otherwise
  uint16_t         updates;
  uint16_t         errors;
  bool             everSet;
};

class VehicleState {
 public:
  VehicleState();

  // Reserve a slot for a signal. Returns the slot index, or -1 when full or
  // already registered. Call at setup; the set of slots is fixed at runtime.
  int8_t registerSignal(const SignalDef* def);

  int8_t indexOf(const SignalDef* def) const;
  int8_t indexOfName(const char* name) const;

  void publish(int8_t slot, float value, uint32_t raw, const char* label,
               uint32_t nowMs);
  void noteError(int8_t slot);

  const StateSlot* slot(int8_t index) const;
  uint8_t count() const { return count_; }

  // Fresh = set at least once and published within ttlMs.
  bool fresh(int8_t index, uint32_t nowMs, uint32_t ttlMs = VS_DEFAULT_TTL_MS) const;

 private:
  StateSlot slots_[VS_MAX_SLOTS];
  uint8_t   count_;
};
