#pragma once
// External page controls: the head unit's transport buttons, debounced.
//
// 0x394 byte 3 is the RAR's button bitfield. Two of its bits drive the page
// scroller: **bit 2 steps RIGHT, bit 3 steps LEFT**. They are fast-forward and
// rewind on the head unit, which is why they are the right pair to borrow --
// they sit under the thumb, they do nothing useful on a radio nobody is
// controlling from here, and they are already a left/right pair, so the
// mapping needs no explaining to the person pressing them.
//
// TWO PHYSICAL BUTTONS ARE COMING and this is written for that. Nothing below
// knows the bits came off CAN: `buttonsStep()` takes a bitmask and a clock, so
// wiring real buttons later means sampling two GPIOs into the same two bit
// positions and deleting nothing.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

// Bit positions within 0x394 b3.
#define BTN_RIGHT_BIT 2u
#define BTN_LEFT_BIT  3u
#define BTN_RIGHT_MASK (1u << BTN_RIGHT_BIT)
#define BTN_LEFT_MASK  (1u << BTN_LEFT_BIT)

// THE SIGNAL IS EVENT-DRIVEN, which is what makes this simple.
//
// Confirmed by the owner against the real head unit: **0x394 is sent the moment
// a button goes down, and sent again the moment it comes up**, carrying the new
// state each time. The ~1 s cadence in the byte map is the idle HEARTBEAT, not
// the report rate.
//
// That matters more than any constant below. Had presses only been visible in a
// 1 Hz heartbeat, a 200 ms tap could fall entirely between two frames and be
// missed, and no debounce setting could have fixed it -- it would have been a
// sampling problem wearing a debouncing costume. With an edge on each
// transition there is nothing to miss.
//
// A classic integrator is therefore the wrong tool as well as an unnecessary
// one: it waits for a level to hold steady because it is filtering an
// ELECTRICAL contact, and nothing here is electrical. The bit arrives inside a
// CRC-protected CAN frame, so a level that changed between two frames changed
// for real, and averaging it would only add latency to a clean signal.
//
// Three real risks remain, and the design answers each:
//
//   * ONE PRESS MUST BE ONE PAGE. Fire on the RISING EDGE only, and require the
//     bit to be seen CLEAR before it can fire again -- which the release frame
//     delivers immediately.
//   * CHATTER, if the head unit's own switch scanning ever reports a bit
//     flickering. BTN_LOCKOUT_MS is the floor between two accepted steps on the
//     same button.
//   * A STUCK BIT. If the radio faults with a bit held, the edge rule gives
//     exactly ONE spurious page change and then silence, because it never goes
//     clear to re-arm. Visible once, harmless, cannot run away. (A DISCONNECTED
//     head unit is not this case: unplugging it removes the whole ID family --
//     09F, 1AB, 1BB, 394, 3D0, 416 -- from the bus, so absence never reads as a
//     held button.)
//
// 150 ms, not 300. Switch bounce is a few milliseconds and 150 buries it, while
// a deliberate double-press to move two pages lands 150-300 ms apart and still
// gets through. 300 was chosen while the report rate was unknown and would have
// swallowed the second press of a fast double.
#define BTN_LOCKOUT_MS 150u

struct ButtonState {
  uint8_t  held;        // debounced level, the two mask bits only
  uint32_t firedRight;  // when each button last produced a step
  uint32_t firedLeft;
  bool     everRight;   // false until the first step, so t=0 is not a lockout
  bool     everLeft;
};

void buttonsInit(ButtonState& b);

// Fold in one 0x394 byte 4. Returns the page step it implies: +1 right,
// -1 left, 0 for nothing.
//
// BOTH BITS AT ONCE IS NOT A PRESS. Nobody holds fast-forward and rewind
// together, so it is either a fault or a byte that is not what we think it is;
// either way the honest answer is to move nothing and to treat both as
// released, so whichever one is let go of first can re-arm cleanly.
int8_t buttonsStep(ButtonState& b, uint8_t raw, uint32_t nowMs);
