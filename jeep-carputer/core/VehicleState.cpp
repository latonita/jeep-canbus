// The hub: fixed-slot registry of published signal values.
//
// PURE: no ESP-IDF, no Arduino. Static allocation, no locking.
#include "VehicleState.h"

#include <stdint.h>
#include <string.h>

// VS_MAX_SLOTS is 48, so every valid index fits in int8_t and -1 stays free as
// the "no such slot" sentinel. Static-assert it so a future bump to 128+ slots
// fails the build instead of silently wrapping the index type.
#if VS_MAX_SLOTS > 127u
#error "VS_MAX_SLOTS must stay <= 127: registerSignal/indexOf return int8_t"
#endif

VehicleState::VehicleState() : count_(0) {
  // Zeroing gives def = NULL, label = NULL, everSet = false for every slot.
  memset(slots_, 0, sizeof(slots_));
}

// Slot must be registered (index < count_) AND actually carry a def. The
// second half is belt-and-braces: count_ is only ever advanced by
// registerSignal, which always sets def first.
static bool vsValidIndex(int8_t index, uint8_t count) {
  return index >= 0 && (uint8_t)index < count;
}

int8_t VehicleState::registerSignal(const SignalDef* def) {
  if (def == 0) return -1;
  const int8_t existing = indexOf(def);
  if (existing >= 0) return -1;             // already registered
  if (count_ >= VS_MAX_SLOTS) return -1;    // full

  StateSlot& s = slots_[count_];
  memset(&s, 0, sizeof(s));
  s.def = def;
  const int8_t idx = (int8_t)count_;
  ++count_;
  return idx;
}

int8_t VehicleState::indexOf(const SignalDef* def) const {
  if (def == 0) return -1;
  for (uint8_t i = 0; i < count_; ++i) {
    if (slots_[i].def == def) return (int8_t)i;
  }
  return -1;
}

int8_t VehicleState::indexOfName(const char* name) const {
  if (name == 0) return -1;
  for (uint8_t i = 0; i < count_; ++i) {
    const SignalDef* d = slots_[i].def;
    if (d == 0 || d->name == 0) continue;
    if (strcmp(d->name, name) == 0) return (int8_t)i;
  }
  return -1;
}

void VehicleState::publish(int8_t slot, float value, uint32_t raw,
                           const char* label, uint32_t nowMs) {
  // An unregistered or out-of-range slot is dropped, not indexed. A caller
  // that ignored a -1 from registerSignal must not be able to scribble over
  // the array or over count_.
  if (!vsValidIndex(slot, count_)) return;

  StateSlot& s = slots_[(uint8_t)slot];
  s.value = value;
  s.raw = raw;
  s.label = label;      // may be NULL: not an enum
  s.stampMs = nowMs;
  s.everSet = true;
  // Saturate rather than wrap: a diagnostics screen showing 65535 updates is
  // honest, one that has rolled back to 0 looks like a signal that never
  // arrived.
  if (s.updates != 0xFFFFu) ++s.updates;
}

void VehicleState::noteError(int8_t slot) {
  if (!vsValidIndex(slot, count_)) return;

  StateSlot& s = slots_[(uint8_t)slot];
  // Deliberately touches nothing else: value, raw, label, stampMs and everSet
  // all survive. A presenter keeps showing the last known RPM (and can flag it
  // stale via fresh()) instead of blanking the screen on one dropped poll.
  if (s.errors != 0xFFFFu) ++s.errors;
}

const StateSlot* VehicleState::slot(int8_t index) const {
  if (!vsValidIndex(index, count_)) return 0;
  return &slots_[(uint8_t)index];
}

bool VehicleState::fresh(int8_t index, uint32_t nowMs, uint32_t ttlMs) const {
  if (!vsValidIndex(index, count_)) return false;
  const StateSlot& s = slots_[(uint8_t)index];
  if (!s.everSet) return false;

  // ROLLOVER SAFETY: millis() is uint32_t and wraps after ~49.7 days. Never
  // compare stamp + ttl against now (that addition overflows near the wrap and
  // makes a fresh value look ancient, or vice versa). Instead subtract in
  // unsigned arithmetic, which is defined to be modulo 2^32, so the age is
  // correct across the wrap as long as it is genuinely less than 2^32 ms.
  //
  // Side effect, and the right one: if the clock ever jumps backwards, the age
  // computes as an enormous number and the slot reads stale. Erring towards
  // stale is the safe direction for a dashboard.
  const uint32_t age = nowMs - s.stampMs;
  return age < ttlMs;   // ttlMs == 0 therefore means "never fresh"
}
