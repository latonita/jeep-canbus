#include "BtControl.h"

#if FEATURE_BT_CONTROL

#include <Arduino.h>

BtControl::BtControl(IEventSink& sink, const IBtPowerSource* power, const ILinkIdle* link)
    : bt_(PIN_BT_UART_RX, PIN_BT_UART_TX),
      sink_(sink),
      power_(power),
      link_(link),
      lastPressMs_(0),
      sentCount_(0),
      droppedCount_(0),
      pending_(OVC_NONE),
      lastSent_(OVC_NONE),
      lastMask_(0),
      manualPowered_(false),
      started_(false) {}

void BtControl::begin() {
  bt_.begin(BT_BAUD);
  // Transmit only. stopListening() releases the pin-change interrupt that
  // SoftwareSerial::begin() claims for the RX pin; we never read the OVC3860,
  // and that interrupt would only add jitter to the hardware UART.
  bt_.stopListening();
  pending_ = OVC_NONE;
  started_ = true;
}

bool BtControl::btPowered() const {
  return (power_ != 0) ? power_->btPowered() : manualPowered_;
}

void BtControl::tick(uint32_t nowMs) {
  (void)nowMs;
  if (pending_ == OVC_NONE) {
    return;
  }
  // The BT module is unpowered: a stale skip must not fire later when the
  // driver returns to the AUX source.
  if (!btPowered()) {
    pending_ = OVC_NONE;
    if (droppedCount_ < 0xFFFFu) {
      droppedCount_++;
    }
    return;
  }
  // Deferred send: SoftwareSerial masks interrupts for the whole transmission
  // (~610 us for 7 chars at 115200), so it only ever runs from tick(), and only
  // while the hardware UART has nothing in flight.
  if (link_ != 0 && !link_->linkIdle()) {
    return;
  }
  (void)flush();
}

ModStatus BtControl::flush() {
  if (!started_) {
    return MOD_ERR_NOT_READY;
  }
  OvcCommand cmd = pending_;
  pending_ = OVC_NONE;

  switch (cmd) {
    case OVC_NEXT_TRACK:
      bt_.print(F("AT#MD\n\r"));
      break;
    case OVC_PREV_TRACK:
      bt_.print(F("AT#ME\n\r"));
      break;
    case OVC_PLAY_PAUSE:
      bt_.print(F("AT#MA\n\r"));
      break;
    case OVC_NONE:
      return MOD_ERR_NO_OP;
    default:
      return MOD_ERR_PARAM;
  }

  lastSent_ = cmd;
  if (sentCount_ < 0xFFFFu) {
    sentCount_++;
  }
  sink_.emit(EV_BT_COMMAND, (uint16_t)cmd);
  return MOD_OK;
}

ModStatus BtControl::queue(OvcCommand cmd) {
  if (cmd == OVC_NONE) {
    return MOD_ERR_NO_OP;
  }
  if (!btPowered()) {
    // Buttons only reach us while the head unit is on AUX, so this is rare;
    // still, never hold a command for a module that is off.
    if (droppedCount_ < 0xFFFFu) {
      droppedCount_++;
    }
    return MOD_ERR_NOT_READY;
  }
  if (pending_ != OVC_NONE && droppedCount_ < 0xFFFFu) {
    droppedCount_++;  // one slot: the newer press wins
  }
  pending_ = cmd;
  return MOD_OK;
}

// Bit priority matches the original if/else chain exactly, including the two
// recognised-but-unmapped fast-forward bits, which swallow the press.
OvcCommand BtControl::decode(uint8_t mask) {
  if (mask & BTN_SEEK_RIGHT) {
    return OVC_NEXT_TRACK;
  }
  if (mask & BTN_SEEK_LEFT) {
    return OVC_PREV_TRACK;
  }
  if (mask & BTN_FF_RIGHT) {
    return OVC_NONE;
  }
  if (mask & BTN_FF_LEFT) {
    return OVC_NONE;
  }
  if (mask & BTN_RND_PTY) {
    return OVC_PLAY_PAUSE;
  }
  return OVC_NONE;
}

void BtControl::onFrame(const CanFrame& frame) {
  switch (frame.id) {
    case CANID_RADIO_CONTROLS: {
      // The original indexed data[3] with no length check at all.
      if (frame.len < LEN_RADIO_CONTROLS_MIN) {
        return;
      }
      uint8_t mask = frame.data[3];
      if (mask == 0) {
        return;  // release / idle report
      }
      uint32_t nowMs = millis();
      if (nowMs - lastPressMs_ < BUTTON_DEBOUNCE_MS) {
        return;
      }
      lastPressMs_ = nowMs;
      lastMask_ = mask;
      sink_.emit(EV_BUTTON, mask);
      (void)queue(decode(mask));
      break;
    }
    default:
      break;
  }
}

void BtControl::safeState() {
  pending_ = OVC_NONE;
}

#endif  // FEATURE_BT_CONTROL
