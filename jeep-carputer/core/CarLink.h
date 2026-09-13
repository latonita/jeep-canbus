#pragma once
// Frame-level transport to the vehicle.
//
// The only path to the bus is the ATmega328 board in the radio
// (jeep-canbus/jeep-aux-and-can-gateway), reached over UART with an
// ELM327-style AT command set. That board is a dumb frame pipe: it auto-sends
// the ISO-TP Flow Control frame and nothing else. Everything above this
// interface -- reassembly, KWP, decoding -- is ours.
//
// Blocking with an explicit timeout, on purpose. On the ESP32 the poll loop
// owns its own task, so a bounded block is far simpler to reason about than a
// state machine, and there is no watchdog-starvation argument here the way
// there was on the AVR.
//
// PURE: no ESP-IDF, no Arduino. Implementations live outside core/.
#include <stdint.h>

#define CAN_MAX_DLC 8u

struct CanFrame {
  uint16_t id;                 // 11-bit standard identifier
  uint8_t  len;                // 0..CAN_MAX_DLC
  uint8_t  data[CAN_MAX_DLC];
};

enum LinkStatus : uint8_t {
  LINK_OK = 0,
  LINK_NO_DATA,       // nothing arrived before the deadline (not always an error)
  LINK_TIMEOUT,       // the adapter itself stopped answering
  LINK_CAN_ERROR,     // adapter reported CAN ERROR
  LINK_OVERFLOW,      // adapter reported BUFFER FULL
  LINK_PROTOCOL,      // unparseable adapter output
  LINK_NOT_READY,     // begin() not called, or the adapter is not responding
  LINK_BAD_PARAM,
};

class ICarLink {
 public:
  // Open the link and put the adapter into a known state: reset, echo off,
  // headers on, CAN auto-formatting OFF (we build our own PCI bytes).
  virtual LinkStatus begin() = 0;

  // Request/response addressing (ATSH / ATCRA). Cheap to call repeatedly; an
  // implementation should skip the round trip when the value is unchanged.
  virtual LinkStatus setTarget(uint16_t requestId, uint16_t responseId) = 0;

  // Transmit one raw frame with the current request id.
  virtual LinkStatus sendFrame(const CanFrame& frame) = 0;

  // Next frame from the adapter, or LINK_NO_DATA once the deadline passes.
  virtual LinkStatus readFrame(CanFrame& out, uint32_t timeoutMs) = 0;

  // Is the link usable at all? False makes the scheduler back off instead of
  // hammering a dead UART.
  virtual bool ready() = 0;

  // Passive monitoring. Between these two calls the adapter streams EVERY frame
  // on the bus and accepts no commands, so requests are impossible until
  // monitorEnd() returns -- the two modes cannot overlap. readFrame() is the
  // way to collect what arrives.
  //
  // Not pure: a replay or bench link that cannot monitor simply says so, and
  // the caller falls back to polling for everything.
  virtual LinkStatus monitorBegin() { return LINK_NOT_READY; }
  virtual LinkStatus monitorEnd() { return LINK_NOT_READY; }

 protected:
  ~ICarLink() = default;
};
