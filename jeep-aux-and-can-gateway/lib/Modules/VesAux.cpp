#include "VesAux.h"

#if FEATURE_VES_AUX

#include <Arduino.h>

// 0x0F is a real mode value, so "never seen a mode frame" needs its own marker.
#define RADIO_MODE_UNKNOWN 0xFFu

VesAux::VesAux(ICanBus& bus, IEventSink& sink)
    : bus_(bus),
      sink_(sink),
      lastAnnounceMs_(0),
      txFail_(0),
      lastTx_(CAN_ST_OK),
      radioMode_(RADIO_MODE_UNKNOWN),
      auxSelected_(false),
      btPowered_(false),
      announcing_(false) {}

void VesAux::begin() {
  // Level BEFORE direction. Writing to a pin that is still an input enables the
  // pull-up, which for an active-low switch is the off state, so the pin never
  // passes through a low pulse on its way to being driven. With the sense
  // inverted that pulse would briefly power the BT module at every boot -- and
  // at every safe-state recovery, which can repeat.
  digitalWrite(PIN_BT_POWER, BT_POWER_OFF_LEVEL);
  pinMode(PIN_BT_POWER, OUTPUT);
  digitalWrite(PIN_BT_POWER, BT_POWER_OFF_LEVEL);
  btPowered_ = false;
  auxSelected_ = false;
  radioMode_ = RADIO_MODE_UNKNOWN;
  // 0 means "due immediately" on the first tick after a resume; at cold boot
  // millis() is still small, so the first announce lands about one period in,
  // exactly as the original firmware did.
  lastAnnounceMs_ = 0;
  announcing_ = true;
}

void VesAux::tick(uint32_t nowMs) {
  if (!announcing_) {
    return;
  }
  // Wrap-safe: unsigned subtraction survives the millis() rollover at ~49 days.
  if (nowMs - lastAnnounceMs_ < VES_ANNOUNCE_PERIOD_MS) {
    return;
  }
  lastAnnounceMs_ = nowMs;

  CanStatus st = announce();
  if (st != lastTx_) {
    lastTx_ = st;
    sink_.emit(EV_CAN_STATE, (uint16_t)st);
  }
  if (st != CAN_ST_OK && txFail_ < 0xFFFFu) {
    txFail_++;
  }
  // No delay() here on purpose: the original slept 20 ms after every send,
  // which starves the watchdog refresh and the AT link.
}

CanStatus VesAux::announce() {
  CanFrame f;
  f.id = CANID_VES_UNIT;
  f.len = VES_ANNOUNCE_LEN;
  f.data[0] = VES_ANNOUNCE_BYTE0;
  for (uint8_t i = 1; i < CAN_MAX_DLC; i++) {
    f.data[i] = 0;
  }
  return bus_.send(f);
}

void VesAux::onFrame(const CanFrame& frame) {
  switch (frame.id) {
    case CANID_RADIO_MODE: {
      // A short frame on a known id is legal CAN: validate before indexing.
      if (frame.len < LEN_RADIO_MODE_MIN) {
        return;
      }
      uint8_t mode = (uint8_t)(frame.data[0] & RADIO_MODE_MASK);
      if (mode != radioMode_) {
        radioMode_ = mode;
        sink_.emit(EV_RADIO_MODE, mode);
      }
      bool aux = (mode == RADIO_MODE_AUX);
      if (aux != auxSelected_) {
        auxSelected_ = aux;
        sink_.emit(EV_AUX_STATE, aux ? 1u : 0u);
        applyBtPower(aux);
      }
      break;
    }
    default:
      break;
  }
}

// Only touches the pin and emits on an actual change.
void VesAux::applyBtPower(bool on) {
  if (on == btPowered_) {
    return;
  }
  btPowered_ = on;
  digitalWrite(PIN_BT_POWER, on ? BT_POWER_ON_LEVEL : BT_POWER_OFF_LEVEL);
  sink_.emit(EV_BT_POWER, on ? 1u : 0u);
}

void VesAux::safeState() {
  announcing_ = false;
  auxSelected_ = false;
  radioMode_ = RADIO_MODE_UNKNOWN;
  // Unconditional, unlike applyBtPower(): a safe state must not depend on our
  // cached idea of the pin being correct, or on begin() having run first.
  digitalWrite(PIN_BT_POWER, BT_POWER_OFF_LEVEL);
  pinMode(PIN_BT_POWER, OUTPUT);
  digitalWrite(PIN_BT_POWER, BT_POWER_OFF_LEVEL);
  if (btPowered_) {
    btPowered_ = false;
    sink_.emit(EV_BT_POWER, 0u);
  }
}

#endif  // FEATURE_VES_AUX
