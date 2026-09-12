#pragma once
// MCP2515 driver: the only place in the firmware that talks to the vendored
// Seeed mcp_can library. Everything else sees ICanBus.
//
// The vendored library (mcp_can.cpp/.h/mcp_can_dfs.h) is the author's patched
// copy -- upstream Seeed only carries 16 MHz bit-timing tables, this one also
// carries the added 8 MHz timing set. This board is 16 MHz. Do not replace
// it with a package from the registry.
//
// Notes on what this wrapper has to work around, all verified in mcp_can.cpp:
//
//  * begin() is the only blocking call. mcp2515_init() contains four or five
//    delay(10) calls on the DEBUG_MODE==0 path plus one in mcp2515_reset(), so
//    a single attempt costs roughly 50 ms. That is bounded and unavoidable
//    without editing the library, so begin() takes at most CAN_BEGIN_ATTEMPTS
//    shots and hands the retry cadence (CAN_INIT_RETRY_MS) back to the caller.
//    Nothing else in this class blocks or delays.
//
//  * init_Mask()/init_Filt() DO handle the configuration-mode transition
//    themselves: each one calls mcp2515_setCANCTRL_Mode(MODE_CONFIG), writes
//    the register, then puts the controller back in MODE_NORMAL. So we must
//    NOT try to switch modes around them -- but every such call also carries
//    two or three delay(10)s and leaves the node off the bus for that window.
//    Acceptance programming is therefore a setup-time operation only.
//
//  * mcp2515_initCANBuffers() has its mask/filter zeroing commented out, and
//    the MCP2515 does not reset RXMn/RXFn. After begin() the masks hold
//    whatever was in them, while RXBnCTRL is set to MCP_RXB_RX_STDEXT, i.e.
//    filtering IS active. begin() therefore always programs the acceptance
//    state explicitly so it is never left undefined.
#include "CanBus.h"
#include "CanFrame.h"
#include "Config.h"
#include "mcp_can.h"

// RXF0..RXF5: the MCP2515 has exactly six acceptance filters, so six is a hard
// ceiling, not a tuning knob. It covers every id in JeepCan.h with one spare.
#define CAN_ACCEPT_MAX      6u

// Attempts per begin()/reinit() call. Bounded on purpose: the AT link and the
// watchdog need the main loop to keep turning, so we never spin here.
#define CAN_BEGIN_ATTEMPTS  2u

// receive() skips frames it cannot represent (extended id, remote request).
// Bounded so a burst of them cannot stall the loop; two hardware RX buffers
// plus slack.
#define CAN_RX_SKIP_MAX     4u

#define CAN_STD_ID_MAX      0x7FFu

class CanDriver : public ICanBus {
 public:
  // Only stores the CS pin in the MCP_CAN member. Note that MCP_CAN's own
  // constructor calls pinMode()/digitalWrite(), which for a file-scope
  // instance runs before Arduino's init(); that is harmless for plain digital
  // I/O, but nothing else may be added to this constructor.
  CanDriver();

  // Bounded init at CAN_BITRATE_ID / CAN_CRYSTAL_ID, then re-applies the
  // acceptance table. Returns CAN_ST_OK, or CAN_ST_NOT_READY if the caller
  // should try again later (after CAN_INIT_RETRY_MS). Never loops forever.
  CanStatus begin() override;

  // Full controller reset for the Supervisor to call after bus silence. Also
  // the only way to clear the sticky RXnOVR flags -- the library exposes no
  // register write to clear EFLG.
  CanStatus reinit();

  CanStatus send(const CanFrame& frame) override;
  CanStatus receive(CanFrame& out) override;

  // Programs one more hardware filter. Setup-time only: each accepted id
  // rewrites all six filters and both masks, and every one of those library
  // calls drops the node out of normal mode for ~20 ms.
  CanStatus setAcceptance(uint16_t id, uint16_t mask) override;
  CanStatus acceptAll() override;

  bool healthy() override;

  uint16_t rxCount() const override { return rxCount_; }
  uint16_t txCount() const override { return txCount_; }
  uint16_t errCount() const override { return errCount_; }

  // True once the controller is initialised, so main.cpp can decide whether to
  // retry begin() this iteration instead of calling into a dead chip.
  bool ready() const { return ready_; }

 private:
  // Saturating: AT@ST reports these, and a wrapped counter reads as healthy.
  static void bump(uint16_t& counter);

  CanStatus writeMasks(uint16_t mask);
  CanStatus applyAcceptance();

  MCP_CAN  mcp_;

  uint16_t acceptId_[CAN_ACCEPT_MAX];
  uint16_t acceptMask_;
  uint8_t  acceptCount_;

  uint8_t  lastEflg_;   // previous EFLG, so healthy() counts each fault once
  bool     ready_;

  uint16_t rxCount_;
  uint16_t txCount_;
  uint16_t errCount_;
};
