#pragma once
// Keeps the board alive without help: hardware watchdog, CAN bus liveness, and
// a defined safe state for every other module when the bus goes away.
//
// AUTONOMOUS: no part of this is reachable from the AT link. It only reports.
#include "CanBus.h"
#include "CanFrame.h"
#include "Config.h"
#include "Events.h"
#include "Module.h"
#include "ModuleHooks.h"

// How often we are willing to re-ask ICanBus::begin(). The MCP2515 init costs
// ~60 ms of blocking driver delays, so retrying it at CAN_INIT_RETRY_MS (the
// cold-boot interval) would eat a quarter of the loop on a dead bus.
#ifndef SUP_REINIT_RETRY_MS
#define SUP_REINIT_RETRY_MS CAN_SILENCE_REINIT_MS
#endif

enum SupState : uint8_t {
  SUP_BOOT = 0,   // no frame seen yet
  SUP_LIVE,       // frames arriving
  SUP_SILENT,     // silent past CAN_SILENCE_REINIT_MS, re-initialising
  SUP_SAFE,       // silent past CAN_SILENCE_SAFE_MS, modules parked
};

class Supervisor : public Module {
 public:
  // modules: the same static array main.cpp ticks, this instance included.
  //          NULL entries are tolerated; this instance is skipped when
  //          broadcasting safeState()/begin() so it never parks itself.
  Supervisor(ICanBus& bus, IEventSink& sink, Module* const* modules, uint8_t count);

  // Enables the watchdog at WDT_PERIOD and starts the silence timer.
  void begin() override;

  // Refreshes the watchdog, accumulates uptime, runs the bus-health state
  // machine. Must be called every loop iteration.
  void tick(uint32_t nowMs) override;

  // Any accepted frame counts as proof of life; this is how the silence timer
  // is fed by default.
  void onFrame(const CanFrame& frame) override;

  // No-op: the supervisor is what recovers everyone else.
  void safeState() override {}

  // Alternative/extra feed for frames that main.cpp consumes itself and does
  // not dispatch to modules (AT monitor traffic, ISO-TP responses).
  void noteRx();   // records arrival; the timestamp is taken in tick()

  // ---- status, for AT@ST -------------------------------------------------
  SupState state() const { return state_; }
  bool inSafeState() const { return state_ == SUP_SAFE; }
  uint32_t uptimeSeconds() const { return uptimeSec_; }
  uint32_t msSinceRx() const;
  uint16_t reinitCount() const { return reinits_; }
  uint8_t resetFlags() const { return resetFlags_; }

  // Classic AVR trick: distance between the top of the heap and the current
  // stack pointer. Static so AtServer can call it without a Supervisor.
  static uint16_t freeRam();

 private:
  ModStatus reinitBus(uint32_t nowMs);
  void enterSafe();
  void recover();
  void setState(SupState next, CanStatus reported);

  ICanBus& bus_;
  IEventSink& sink_;
  Module* const* modules_;
  uint32_t lastRxMs_;
  uint32_t lastReinitMs_;
  uint32_t lastSecMs_;
  uint32_t uptimeSec_;
  uint16_t reinits_;
  uint8_t count_;
  uint8_t resetFlags_;
  SupState state_;
  bool busErrLatched_;
  // Set by onFrame(), consumed by tick(). Deliberately not a timestamp: see
  // the comment in Supervisor::tick().
  bool sawFrame_;
};
