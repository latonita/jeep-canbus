#pragma once
// Emulates the presence of a VES (rear-seat entertainment) unit so the RAR head
// unit offers the AUX/VES source, and switches the BT A2DP module's power to
// follow that source selection.
//
// AUTONOMOUS: nothing in this module can be reached, driven or suppressed by
// the AT link. It needs only a bus and a tick.
#include "CanBus.h"
#include "CanFrame.h"
#include "Config.h"
#include "Events.h"
#include "JeepCan.h"
#include "Module.h"
#include "ModuleHooks.h"

class VesAux : public Module, public IBtPowerSource {
 public:
  VesAux(ICanBus& bus, IEventSink& sink);

  // Configures PIN_BT_POWER as an output driven LOW, and (re-)arms announcing.
  // Also used by Supervisor to resume the module after a safe state.
  void begin() override;

  // Transmits the VES announce every VES_ANNOUNCE_PERIOD_MS. Never blocks.
  void tick(uint32_t nowMs) override;

  // Watches CANID_RADIO_MODE only. Validates frame.len before indexing.
  void onFrame(const CanFrame& frame) override;

  // BT power LOW, stop announcing. The head unit drops the AUX source on its
  // own once the announce stops, which is the intended safe behaviour.
  void safeState() override;

  // ---- status, for AT@ST -------------------------------------------------
  bool auxSelected() const { return auxSelected_; }
  bool btPowered() const override { return btPowered_; }
  bool announcing() const { return announcing_; }
  uint8_t radioMode() const { return radioMode_; }
  CanStatus lastTxStatus() const { return lastTx_; }
  uint16_t txFailures() const { return txFail_; }

 private:
  CanStatus announce();
  void applyBtPower(bool on);

  ICanBus& bus_;
  IEventSink& sink_;
  uint32_t lastAnnounceMs_;
  uint16_t txFail_;
  CanStatus lastTx_;
  uint8_t radioMode_;   // raw low nibble of 0x09F byte 0, 0xFF = not seen yet
  bool auxSelected_;
  bool btPowered_;
  bool announcing_;
};
