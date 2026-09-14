// jeep-aux-and-can-gateway
//
// In-radio ATmega328 for a 2006 Jeep Grand Cherokee WH, spliced into the
// 83.3 kbps interior CAN-B bus.
//
// Two jobs, in this order of importance:
//
//  1. Keep the car working, autonomously. Announce VES presence so the head
//     unit offers its AUX source, power the Bluetooth module when AUX is
//     selected, and map the radio's panel buttons to BT transport commands.
//     None of this depends on anything being connected to the UART. Unplug the
//     link and the board behaves exactly as the 2015 firmware did.
//
//  2. Act as a CAN gateway for a host on the hardware UART (a laptop now, an
//     ESP32-S3 later), via an ELM327-style AT command set. The board stays a
//     dumb frame pipe: no ISO-TP reassembly, no KWP parsing, no PID decoding
//     here. The only exception is auto-sending the ISO-TP Flow Control frame,
//     which the adapter has to do itself or every multi-frame reply stalls.
//
// The AT surface is deliberately observe-only -- it cannot command VES, the
// radio or Bluetooth, and it cannot reprogram or reset the CAN controller -- so
// no host-side bug can reach the car's audio. Bench mode is the sole exception,
// and only because it used to need a recompile.
//
// Flashed over the ICSP header. There is no serial bootloader, so pins 0/1
// belong to the link and there is NO Serial.print debugging: diagnostics leave
// as AT@EV event lines and AT@ST status.
//
// Copyright (C) 2015-2026 Anton Viktorov <latonita@yandex.ru>
//                         https://github.com/latonita/jeep-canbus
// MIT licensed, as the original.

#include <Arduino.h>

#include "CanDriver.h"
#include "Config.h"
#include "Events.h"
#include "Module.h"
#include "Version.h"

#include "BenchMode.h"
#include "BtControl.h"
#include "ModuleHooks.h"
#include "Supervisor.h"
#include "VesAux.h"

#if FEATURE_AT_SERVER
#include "AtServer.h"
#include "AtStatus.h"
#include "ByteIo.h"
#endif

// ---------------------------------------------------------------------------
// Adapters. These exist so the libraries never name each other's types: the AT
// server does not know what a VesAux is, and the modules do not know the link
// is a HardwareSerial.
// ---------------------------------------------------------------------------

#if FEATURE_AT_SERVER

// The link. Serial.read() already returns -1 when empty, which is exactly the
// IByteIo contract.
class SerialByteIo : public IByteIo {
 public:
  int readByte() override { return Serial.read(); }

  void writeBytes(const char* s) override {
    if (s != 0) {
      Serial.write(s);
    }
  }

  void writeByte(char c) override { Serial.write(c); }

  // Free space in the TX ring. HardwareSerial::write() busy-waits once the
  // ring is full, so the stream path asks first and drops instead.
  int writableBytes() const override { return Serial.availableForWrite(); }
};

// BtControl defers its SoftwareSerial writes until the link is quiet, because
// SoftwareSerial transmits with interrupts disabled and would otherwise drop
// bytes arriving on the UART.
class AtLinkIdle : public ILinkIdle {
 public:
  explicit AtLinkIdle(const AtServer& at) : at_(at) {}
  bool linkIdle() const override { return at_.idle(); }

 private:
  const AtServer& at_;
};

#endif  // FEATURE_AT_SERVER

// ---------------------------------------------------------------------------
// Statics. Everything is file-scope and fixed-size: no heap on a 2 KB part.
// ---------------------------------------------------------------------------

static CanDriver gBus;

#if FEATURE_AT_SERVER
static SerialByteIo gLinkIo;
static AtServer gAtServer(gLinkIo, gBus);
static AtLinkIdle gLinkIdle(gAtServer);

// Module events become AT@EV lines (only when the host has asked for them).
static IEventSink& gSink = gAtServer;
static const ILinkIdle* const gIdleSource = &gLinkIdle;
#else
// No link: events go nowhere, and there is nothing for the BT UART to collide
// with, so "always idle" is the correct answer rather than a stub.
static NullEventSink gNullSink;
static IEventSink& gSink = gNullSink;
static AlwaysIdleLink gAlwaysIdle;
static const ILinkIdle* const gIdleSource = &gAlwaysIdle;
#endif

#if FEATURE_VES_AUX
static VesAux gVesAux(gBus, gSink);
#endif

#if FEATURE_BT_CONTROL
// VesAux implements IBtPowerSource, so BtControl never names its type and will
// not send a command to an unpowered module.
static BtControl gBtControl(gSink,
#if FEATURE_VES_AUX
                            &gVesAux,
#else
                            0,
#endif
                            gIdleSource);
#endif

#if FEATURE_BENCH_MODE
static BenchMode gBenchMode(gBus, gSink);
#endif

// Supervisor needs the module array at construction and must appear in it (it
// skips itself). Declared with external linkage so its address is a link-time
// constant usable in the array initialiser below, above the definition.
extern Supervisor gSupervisor;

Module* const kModules[] = {
#if FEATURE_VES_AUX
    &gVesAux,
#endif
#if FEATURE_BT_CONTROL
    &gBtControl,
#endif
#if FEATURE_BENCH_MODE
    &gBenchMode,
#endif
    &gSupervisor,
};

static const uint8_t kModuleCount =
    static_cast<uint8_t>(sizeof(kModules) / sizeof(kModules[0]));

Supervisor gSupervisor(gBus, gSink, kModules, kModuleCount);

#if FEATURE_AT_SERVER

// AT@ST reads live values out of the modules that own them.
class ModuleStatus : public IStatusProvider {
 public:
  void fillStatus(AtStatusInfo& out) override {
#if FEATURE_VES_AUX
    out.radioMode = gVesAux.radioMode();
    out.auxActive = gVesAux.auxSelected();
    out.btPower = gVesAux.btPowered();
#endif
#if FEATURE_BENCH_MODE
    out.benchMode = gBenchMode.enabled();
#endif
    out.freeRam = Supervisor::freeRam();
    // Why the board last came up, and whether the bus has been re-initialised
    // since. An unexplained reboot in a car is worth being able to name.
    out.resetFlags = gSupervisor.resetFlags();
    out.reinits = gSupervisor.reinitCount();
  }
};

// AT@BENCH0/1. The only host command in the whole protocol that changes
// anything on the board, and it is irrelevant in a car.
class BenchToggle : public IBenchControl {
 public:
  bool setBenchMode(bool on) override {
#if FEATURE_BENCH_MODE
    return gBenchMode.setEnabled(on) == MOD_OK;
#else
    (void)on;
    return false;
#endif
  }

  bool benchMode() const override {
#if FEATURE_BENCH_MODE
    return gBenchMode.enabled();
#else
    return false;
#endif
  }

  bool setVin(const char* s, uint8_t n) override {
#if FEATURE_BENCH_MODE
    return gBenchMode.setVin(s, n) == MOD_OK;
#else
    (void)s; (void)n;
    return false;
#endif
  }

  bool clearVin() override {
#if FEATURE_BENCH_MODE
    return gBenchMode.clearVin() == MOD_OK;
#else
    return false;
#endif
  }

  uint8_t getVin(char* out, uint8_t cap) const override {
#if FEATURE_BENCH_MODE
    return gBenchMode.getVin(out, cap);
#else
    (void)out; (void)cap;
    return 0u;
#endif
  }

  bool vinStored() const override {
#if FEATURE_BENCH_MODE
    return gBenchMode.vinStored();
#else
    return false;
#endif
  }
};

static ModuleStatus gModuleStatus;
static BenchToggle gBenchToggle;

#endif  // FEATURE_AT_SERVER

// ---------------------------------------------------------------------------

void setup() {
  // The link, not a console. Nothing may print to it outside the AT protocol.
  Serial.begin(LINK_BAUD);

  // Deliberately not retried in a loop here. begin() is bounded and the main
  // loop re-attempts it; the original firmware's
  //   while (CAN_OK != CAN.begin(...)) delay(250);
  // would starve the watchdog and the link on a dead bus, and a dead bus is
  // exactly when we still want VES announce and the AT server alive.
  //
  // Hardware acceptance is left wide open on purpose -- the host's ATCRA/ATCF
  // filter is applied in software inside AtServer, so a diagnostic session can
  // never filter away the 0x09F and 0x394 frames the modules depend on.
  (void)gBus.begin();

  for (uint8_t i = 0; i < kModuleCount; ++i) {
    if (kModules[i] != 0) {
      kModules[i]->begin();
    }
  }

#if FEATURE_AT_SERVER
  gAtServer.setStatusProvider(&gModuleStatus);
  gAtServer.setBenchControl(&gBenchToggle);
  gAtServer.begin();
#endif
}

void loop() {
  // One millis() sample per iteration so every module agrees on "now".
  const uint32_t now = millis();

  // Retry CAN init out here rather than blocking in setup(). Supervisor owns
  // the re-init cadence once running; this covers a cold start with the bus
  // down (e.g. powered up before the radio).
  if (!gBus.ready()) {
    (void)gBus.begin();
  }

  // Drain a bounded number of frames per iteration: the loop must keep turning
  // for the watchdog and the link, and CAN-B at 83.3 kbps cannot outrun this.
  for (uint8_t drained = 0; drained < CAN_RX_DRAIN_MAX; ++drained) {
    CanFrame frame;
    if (gBus.receive(frame) != CAN_ST_OK) {
      break;
    }
    for (uint8_t i = 0; i < kModuleCount; ++i) {
      if (kModules[i] != 0) {
        kModules[i]->onFrame(frame);
      }
    }
#if FEATURE_AT_SERVER
    // AtServer never calls receive() itself -- this is its only source of
    // frames, and without it every request would answer NO DATA.
    gAtServer.onFrame(frame);
#endif
  }

  for (uint8_t i = 0; i < kModuleCount; ++i) {
    if (kModules[i] != 0) {
      kModules[i]->tick(now);
    }
  }

#if FEATURE_AT_SERVER
  gAtServer.tick(now);
#endif
}
