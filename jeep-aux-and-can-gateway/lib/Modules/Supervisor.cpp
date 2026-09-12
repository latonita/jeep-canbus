#include "Supervisor.h"

#include <Arduino.h>
#include <avr/wdt.h>

// Provided by avr-libc / the linker script; the boundary between the heap and
// free RAM. C linkage: both symbols are defined by C code and by the linker.
extern "C" {
extern char* __brkval;   // first byte not yet handed out by malloc, 0 if unused
extern char  __heap_start;
}

Supervisor::Supervisor(ICanBus& bus, IEventSink& sink, Module* const* modules, uint8_t count)
    : bus_(bus),
      sink_(sink),
      modules_(modules),
      lastRxMs_(0),
      lastReinitMs_(0),
      lastSecMs_(0),
      uptimeSec_(0),
      reinits_(0),
      count_((modules != 0) ? count : 0),
      resetFlags_(0),
      state_(SUP_BOOT),
      busErrLatched_(false),
      sawFrame_(false) {}

void Supervisor::begin() {
  // Why we came up last time -- watchdog, brown-out, external. Still intact at
  // this point because the Arduino core does not clear it.
  resetFlags_ = MCUSR;
  MCUSR = 0;

  wdt_disable();
  wdt_enable(WDT_PERIOD);
  wdt_reset();

  uint32_t now = millis();
  lastRxMs_ = now;       // the silence window starts at boot, not at epoch 0
  lastReinitMs_ = now;
  lastSecMs_ = now;
  state_ = SUP_BOOT;
  busErrLatched_ = false;
  sawFrame_ = false;
}

void Supervisor::noteRx() {
  // NOT lastRxMs_ = millis(). loop() drains frames before calling tick(), and
  // tick() compares against a `now` sampled at the top of that iteration. A
  // second millis() read here can land one millisecond LATER than that `now`,
  // and then `nowMs - lastRxMs_` underflows to ~49 days of apparent silence --
  // which threw the board straight into safe state, where recover() re-ran
  // every module's begin() and dropped the Bluetooth power pin. Observed in
  // the car as SAFE 1 / SAFE 0 thrashing at ~13 Hz with the bus fully alive.
  sawFrame_ = true;
}

void Supervisor::onFrame(const CanFrame& frame) {
  (void)frame;  // any accepted frame, on any id, is proof the bus is alive
  noteRx();
}

uint32_t Supervisor::msSinceRx() const {
  return millis() - lastRxMs_;
}

uint16_t Supervisor::freeRam() {
  char top;
  // &top is the current stack pointer; __brkval is 0 until the heap is used.
  if (__brkval == 0) {
    return (uint16_t)(&top - &__heap_start);
  }
  return (uint16_t)(&top - __brkval);
}

void Supervisor::tick(uint32_t nowMs) {
  // First and most important thing every iteration.
  wdt_reset();

  // Stamp arrivals with this tick's clock sample, so lastRxMs_ can never be
  // ahead of nowMs. See noteRx().
  if (sawFrame_) {
    sawFrame_ = false;
    lastRxMs_ = nowMs;
  }

  // Uptime without a loop, so a late tick cannot spin here.
  uint32_t sinceSec = nowMs - lastSecMs_;
  if (sinceSec >= 1000u) {
    uint32_t whole = sinceSec / 1000u;
    uptimeSec_ += whole;
    lastSecMs_ += whole * 1000u;
  }

  // Bus-off / high error counters are their own reason to re-init, regardless
  // of the silence timer.
  bool healthy = bus_.healthy();
  if (!healthy) {
    if (!busErrLatched_) {
      busErrLatched_ = true;
      sink_.emit(EV_CAN_STATE, (uint16_t)CAN_ST_BUS_ERROR);
    }
    (void)reinitBus(nowMs);
  } else if (busErrLatched_) {
    busErrLatched_ = false;
    sink_.emit(EV_CAN_STATE, (uint16_t)CAN_ST_OK);
  }

  // Signed compare, so a lastRxMs_ that somehow still leads nowMs reads as no
  // silence rather than as a wrap. The unsigned form is only wrap-safe while
  // lastRxMs_ <= nowMs, which is exactly the assumption that failed here.
  const int32_t elapsed = static_cast<int32_t>(nowMs - lastRxMs_);
  const uint32_t silence = (elapsed < 0) ? 0u : static_cast<uint32_t>(elapsed);

  if (silence >= CAN_SILENCE_SAFE_MS) {
    if (state_ != SUP_SAFE) {
      enterSafe();
    }
    (void)reinitBus(nowMs);  // rate-limited; keep trying forever
    return;
  }

  if (silence >= CAN_SILENCE_REINIT_MS) {
    if (state_ == SUP_SAFE) {
      recover();  // cannot happen: silence only shrinks when a frame arrives
    }
    if (state_ != SUP_SILENT) {
      setState(SUP_SILENT, CAN_ST_TIMEOUT);
    }
    (void)reinitBus(nowMs);
    return;
  }

  // Frames are flowing.
  switch (state_) {
    case SUP_SAFE:
      recover();
      break;
    case SUP_BOOT:
    case SUP_SILENT:
      setState(SUP_LIVE, CAN_ST_OK);
      break;
    case SUP_LIVE:
      break;
    default:
      setState(SUP_LIVE, CAN_ST_OK);
      break;
  }
}

void Supervisor::setState(SupState next, CanStatus reported) {
  if (state_ == next) {
    return;
  }
  state_ = next;
  sink_.emit(EV_CAN_STATE, (uint16_t)reported);
}

ModStatus Supervisor::reinitBus(uint32_t nowMs) {
  if (nowMs - lastReinitMs_ < SUP_REINIT_RETRY_MS) {
    return MOD_ERR_BUSY;
  }
  lastReinitMs_ = nowMs;
  if (reinits_ < 0xFFFFu) {
    reinits_++;
  }
  CanStatus st = bus_.begin();
  // begin() can block for tens of ms inside the MCP2515 driver; refresh the
  // watchdog on the way out rather than waiting for the next iteration.
  wdt_reset();
  return (st == CAN_ST_OK) ? MOD_OK : MOD_ERR_HW;
}

void Supervisor::enterSafe() {
  state_ = SUP_SAFE;
  for (uint8_t i = 0; i < count_; i++) {
    Module* m = modules_[i];
    if (m == 0 || m == this) {
      continue;  // never park the supervisor itself
    }
    m->safeState();
  }
  sink_.emit(EV_SAFE_STATE, 1u);
}

void Supervisor::recover() {
  // begin() is the modules' resume path: it re-arms announcing and re-asserts
  // the output pins from a known state. It is idempotent by design.
  for (uint8_t i = 0; i < count_; i++) {
    Module* m = modules_[i];
    if (m == 0 || m == this) {
      continue;
    }
    m->begin();
  }
  state_ = SUP_LIVE;
  sink_.emit(EV_SAFE_STATE, 0u);
  sink_.emit(EV_CAN_STATE, (uint16_t)CAN_ST_OK);
}
