#include "Buttons.h"

void buttonsInit(ButtonState& b) {
  b.held = 0u;
  b.firedRight = 0u;
  b.firedLeft = 0u;
  // `ever*` exists so the very first press is never judged against a zero
  // stamp. Without it, a button pressed within BTN_LOCKOUT_MS of boot would be
  // compared against firedMs = 0, and on a wrap-safe unsigned subtraction that
  // reads as "fired a long time ago" only by luck of where the clock happens to
  // be. Stating it is cheaper than reasoning about it.
  b.everRight = false;
  b.everLeft = false;
}

int8_t buttonsStep(ButtonState& b, uint8_t raw, uint32_t nowMs) {
  const uint8_t both = BTN_RIGHT_MASK | BTN_LEFT_MASK;
  uint8_t now = static_cast<uint8_t>(raw & both);

  // Both down is not a gesture; see the header. Treated as fully released so
  // the next single press is a clean rising edge rather than a level that was
  // already set.
  if (now == both) {
    now = 0u;
  }

  const uint8_t rising = static_cast<uint8_t>(now & ~b.held);
  b.held = now;

  if ((rising & BTN_RIGHT_MASK) != 0u) {
    if (!b.everRight || (nowMs - b.firedRight) >= BTN_LOCKOUT_MS) {
      b.firedRight = nowMs;
      b.everRight = true;
      return 1;
    }
    // Inside the lockout: swallowed, and the stamp is deliberately NOT moved.
    // Advancing it would let a bit chattering faster than the lockout push the
    // window along for ever and never fire again -- the guard would become a
    // mute rather than a floor.
  }
  if ((rising & BTN_LEFT_MASK) != 0u) {
    if (!b.everLeft || (nowMs - b.firedLeft) >= BTN_LOCKOUT_MS) {
      b.firedLeft = nowMs;
      b.everLeft = true;
      return -1;
    }
  }
  return 0;
}
