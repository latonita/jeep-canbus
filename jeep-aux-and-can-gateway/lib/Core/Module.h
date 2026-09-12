#pragma once
// A feature module. Instances live in a static array in main.cpp; each is
// wrapped in its own #if FEATURE_* so a disabled module costs nothing.
#include "CanFrame.h"

class Module {
 public:
  virtual void begin() {}

  // Called every loop iteration with a single sampled millis() value, so all
  // modules agree on "now" within an iteration.
  virtual void tick(uint32_t nowMs) { (void)nowMs; }

  // Called for every accepted frame. Implementations MUST validate frame.len
  // before indexing frame.data -- a short frame on a known id is legal CAN.
  virtual void onFrame(const CanFrame& frame) { (void)frame; }

  // Drop to a defined safe state: outputs off, stop transmitting.
  virtual void safeState() {}

 protected:
  ~Module() = default;
};
