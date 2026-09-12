#pragma once
// Wiring types for the AT@ custom commands. These are NOT part of lib/Core --
// they exist only so main.cpp can hand AtServer the few values that live in
// other modules (AT@ST) and route the bench toggle (AT@BENCH) back to them.
//
// PURE: no Arduino, no AVR headers.
#include <stdint.h>

// Snapshot filled on demand for one AT@ST line. Values a provider cannot
// supply must be left at the defaults set by the constructor.
struct AtStatusInfo {
  uint8_t radioMode;   // raw nibble from 0x09F, 0xFF = unknown
  bool    auxActive;   // radio has AUX/VES selected
  bool    btPower;     // BT module powered
  bool    benchMode;   // bench announce active
  uint16_t freeRam;    // bytes between heap top and stack pointer, 0 = unknown
  uint8_t  resetFlags; // MCUSR latched at boot: why we came up last time
  uint16_t reinits;    // CAN controller re-initialisations since boot

  AtStatusInfo()
      : radioMode(0xFFu), auxActive(false), btPower(false), benchMode(false),
        freeRam(0u), resetFlags(0u), reinits(0u) {}
};

// Implemented by main.cpp (or a tiny adapter next to the module instances).
class IStatusProvider {
 public:
  virtual void fillStatus(AtStatusInfo& out) = 0;

 protected:
  ~IStatusProvider() = default;
};

// Implemented by whatever owns bench mode, so AT@BENCH0/1 reaches it without
// AtServer knowing about the bench module.
class IBenchControl {
 public:
  // Returns false when bench mode is not available in this build.
  virtual bool setBenchMode(bool on) = 0;
  virtual bool benchMode() const = 0;

  // Runtime VIN provisioning (AT@VIN). setVin/clearVin return false when bench
  // mode is compiled out or the operation is rejected/failed. getVin copies the
  // active VIN (stored or default) into out, NOT NUL-terminated, and returns the
  // length (0 if it will not fit or bench is unavailable). vinStored() is true
  // when the active VIN came from EEPROM rather than the compiled default.
  virtual bool setVin(const char* s, uint8_t n) = 0;
  virtual bool clearVin() = 0;
  virtual uint8_t getVin(char* out, uint8_t cap) const = 0;
  virtual bool vinStored() const = 0;

 protected:
  ~IBenchControl() = default;
};
