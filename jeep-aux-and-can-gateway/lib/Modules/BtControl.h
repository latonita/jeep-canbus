#pragma once
// Maps the head unit's seek / RND-PTY buttons (CANID_RADIO_CONTROLS) onto
// OVC3860 AT commands over a transmit-only SoftwareSerial.
//
// AUTONOMOUS: the AT link cannot inject, block or observe a command here. The
// only thing the link may contribute is the "are you busy?" hint via ILinkIdle,
// and if that is absent or lies, button mapping still works.
#include <SoftwareSerial.h>

#include "CanFrame.h"
#include "Config.h"
#include "Events.h"
#include "JeepCan.h"
#include "Module.h"
#include "ModuleHooks.h"

class BtControl : public Module {
 public:
  // power: who owns the BT power pin (VesAux implements IBtPowerSource). NULL
  //        means "use setBtPowered()"; with neither, nothing is ever sent.
  // link:  hardware-UART idle hint. NULL means "always idle".
  BtControl(IEventSink& sink, const IBtPowerSource* power = 0, const ILinkIdle* link = 0);

  // Opens the SoftwareSerial and immediately stops listening: this module is
  // transmit-only (the original never read the OVC3860 either), and not
  // listening keeps SoftwareSerial's pin-change interrupt off our back.
  void begin() override;

  // Flushes at most one pending command, only while the link is idle.
  void tick(uint32_t nowMs) override;

  // Watches CANID_RADIO_CONTROLS only. Validates frame.len before indexing --
  // the original read data[3] unconditionally.
  void onFrame(const CanFrame& frame) override;

  // Drop the pending command and send nothing.
  void safeState() override;

  // Fallback power gate for orchestrators that prefer a per-loop setter over
  // the injected source. Ignored when a source was injected.
  void setBtPowered(bool powered) { manualPowered_ = powered; }

  // ---- status, for AT@ST -------------------------------------------------
  bool btPowered() const;
  OvcCommand pending() const { return pending_; }
  uint8_t lastButtonMask() const { return lastMask_; }
  OvcCommand lastSent() const { return lastSent_; }
  uint16_t sentCount() const { return sentCount_; }
  uint16_t droppedCount() const { return droppedCount_; }

 private:
  // Single-slot queue: a newer press deliberately supersedes an unsent older
  // one. Nobody wants a backlog of track skips replayed at them.
  ModStatus queue(OvcCommand cmd);
  ModStatus flush();
  static OvcCommand decode(uint8_t mask);

  SoftwareSerial bt_;
  IEventSink& sink_;
  const IBtPowerSource* power_;
  const ILinkIdle* link_;
  uint32_t lastPressMs_;
  uint16_t sentCount_;
  uint16_t droppedCount_;
  OvcCommand pending_;
  OvcCommand lastSent_;
  uint8_t lastMask_;
  bool manualPowered_;
  bool started_;
};
