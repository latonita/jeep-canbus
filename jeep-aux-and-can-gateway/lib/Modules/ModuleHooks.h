#pragma once
// Dependency-injection seams owned by lib/Modules. Deliberately tiny and
// dependency-free (stdint only) so the feature modules never have to know about
// AtServer, the link, or each other's concrete types.
//
// PFS: every hook has a defined behaviour when it is not wired at all. A module
// that is handed nothing must still run the car correctly.
#include <stdint.h>

// Result of a module operation that can fail. CAN sends keep returning
// CanStatus; this covers everything else (EEPROM, deferred UART sends).
enum ModStatus : uint8_t {
  MOD_OK = 0,
  MOD_ERR_PARAM,      // caller passed something impossible
  MOD_ERR_NOT_READY,  // begin() not called yet
  MOD_ERR_BUSY,       // deferred: try again on a later tick
  MOD_ERR_HW,         // hardware/EEPROM refused
  MOD_ERR_NO_OP,      // nothing to do (not an error)
};

// "Is the hardware UART quiet right now?" SoftwareSerial disables interrupts
// for the whole duration of a byte, so a 7-character command at 115200 blocks
// ~610 us -- long enough to lose bytes arriving on the hardware UART. BtControl
// asks this before it flushes a queued command.
//
// Implemented by whoever owns the link (AtServer). When nothing implements it
// the module behaves as if the link is permanently idle, which is exactly right
// for a board with no host attached.
class ILinkIdle {
 public:
  virtual bool linkIdle() const = 0;

 protected:
  // Non-virtual, protected: implementations live in static storage for the
  // whole program lifetime and a virtual dtor would drag in operator delete.
  ~ILinkIdle() = default;
};

// Default: always idle. Safe because a board with no link has no bytes to lose.
class AlwaysIdleLink : public ILinkIdle {
 public:
  bool linkIdle() const override { return true; }
};

// "Is the BT module powered?" VesAux owns the power pin and implements this;
// BtControl only consumes it, so it never depends on the VesAux type.
class IBtPowerSource {
 public:
  virtual bool btPowered() const = 0;

 protected:
  ~IBtPowerSource() = default;
};
