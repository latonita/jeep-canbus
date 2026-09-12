#pragma once
// Bench mode: tells the head unit the ignition is on, so a radio sitting on the
// desk powers up. In the original this was the compile-time #ifdef
// BENCH_MODE_ON; here it is a runtime flag kept in one EEPROM byte so it
// survives a reset, and it defaults to OFF on an uninitialised chip.
//
// AUTONOMOUS: it needs no host. setEnabled() is the single deliberate exception
// to "the link may not command a module" -- it is a bench convenience, and the
// worst it can do is make a radio think the key is on.
#include "CanBus.h"
#include "CanFrame.h"
#include "Config.h"
#include "Events.h"
#include "JeepCan.h"
#include "Module.h"
#include "ModuleHooks.h"

// One byte of the 1 KB EEPROM. Override with -D if it ever collides.
#ifndef BENCH_EEPROM_ADDR
#define BENCH_EEPROM_ADDR 0
#endif

// Runtime VIN store: [magic][len][VIN_MAX_LEN chars]. Kept clear of the bench
// flag at BENCH_EEPROM_ADDR (0). Override with -D if it ever collides.
#ifndef BENCH_VIN_EEPROM_ADDR
#define BENCH_VIN_EEPROM_ADDR 8
#endif

class BenchMode : public Module {
 public:
  BenchMode(ICanBus& bus, IEventSink& sink, uint16_t eepromAddr = BENCH_EEPROM_ADDR);

  // Reads the EEPROM byte exactly once. Anything that is not the "enabled"
  // magic -- including the 0xFF of a fresh chip -- means OFF, and is NOT
  // written back, so a virgin chip stays virgin.
  void begin() override;

  // While enabled, transmits the power-on frame every BENCH_ANNOUNCE_PERIOD_MS
  // for as long as bench mode is on, and -- only for the first
  // BENCH_VIN_BURST_CYCLES cycles after activation -- one VIN frame every
  // BENCH_VIN_PERIOD_MS (round-robin over the 3 frames). Never blocks. The VIN
  // burst is what stops the desk radio from demanding its anti-theft code; the
  // radio latches it once, so we stop after the burst. Both are bench-only.
  void tick(uint32_t nowMs) override;

  // Safety guard: while bench mode is on we listen for the first
  // BENCH_DETECT_WINDOW_MS before transmitting. A genuine 0x000 power frame in
  // that window means another node is driving the bus -- i.e. this board is in
  // a car (it powers up with the radio, by which time the interior bus is
  // alive) -- so we suppress bench for the rest of this power cycle. We never
  // receive our own frames, so a received 0x000 is always someone else's.
  void onFrame(const CanFrame& frame) override;

  void safeState() override;

  // Writes EEPROM only on an actual change of value.
  ModStatus setEnabled(bool on);
  bool enabled() const { return enabled_; }

  // ---- runtime VIN (AT@VIN) ----------------------------------------------
  // The VIN broadcast in bench mode. Provisioned at runtime and stored in
  // EEPROM; when none is stored the compiled-in VEHICLE_VIN default is used.
  // setVin() validates (1..VIN_MAX_LEN printable ASCII chars) and persists;
  // clearVin() forgets the stored VIN and reverts to the default. Both take
  // effect on the next announce -- no reboot needed.
  ModStatus setVin(const char* s, uint8_t n);
  ModStatus clearVin();
  // Copies the active VIN (stored or default) into out (NOT NUL-terminated);
  // returns the number of chars, 0 if it would not fit in cap.
  uint8_t getVin(char* out, uint8_t cap) const;
  bool vinStored() const { return vinStored_; }

  // ---- status, for AT@ST -------------------------------------------------
  // True only while we are actually transmitting: enabled, armed, past the
  // listen window, and not suppressed by detected vehicle traffic.
  bool announcing() const {
    return enabled_ && announcing_ && !detecting_ && !vehicleSeen_;
  }
  bool vehicleSeen() const { return vehicleSeen_; }
  CanStatus lastTxStatus() const { return lastTx_; }
  uint16_t eepromWrites() const { return writes_; }

 private:
  CanStatus announce();
  CanStatus announceVin(uint8_t frameIdx);
  void loadVin();   // fill vin_/vinLen_ from EEPROM, else the compiled default

  ICanBus& bus_;
  IEventSink& sink_;
  uint32_t lastAnnounceMs_;
  uint32_t lastVinMs_;
  uint32_t detectStartMs_;  // millis() at the start of the listen window
  uint16_t eepromAddr_;
  uint16_t writes_;
  uint16_t vinLeft_;  // VIN frames still to send in this burst (0 = burst done)
  CanStatus lastTx_;
  uint8_t vinIdx_;   // next VIN frame to send, 0..VIN_FRAME_COUNT-1
  uint8_t vinLen_;   // length of the active VIN in vin_
  bool enabled_;
  bool announcing_;
  bool detecting_;     // in the listen-before-transmit window
  bool detectArmed_;   // detectStartMs_ not yet captured (first tick will)
  bool vehicleSeen_;   // real 0x000 heard in-window -> suppress this power cycle
  bool vinStored_;     // active VIN came from EEPROM (vs the compiled default)
  char vin_[VIN_MAX_LEN];  // active VIN, not NUL-terminated (vinLen_ is the len)
};
