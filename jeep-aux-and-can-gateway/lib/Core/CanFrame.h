#pragma once
// Dependency-free CAN types. No Arduino, no AVR headers -- so every consumer of
// this header can also be compiled and unit-tested on the host.
#include <stdint.h>

#define CAN_MAX_DLC 8u

struct CanFrame {
  uint16_t id;                 // 11-bit standard identifier
  uint8_t  len;                // 0..CAN_MAX_DLC
  uint8_t  data[CAN_MAX_DLC];
};

// PFS: anything touching hardware returns a status, never void.
enum CanStatus : uint8_t {
  CAN_ST_OK = 0,
  CAN_ST_NO_MSG,      // nothing to receive (not an error)
  CAN_ST_TX_FAIL,     // controller refused / no free TX buffer
  CAN_ST_TIMEOUT,     // bounded wait expired
  CAN_ST_BUS_ERROR,   // error counters high / bus-off
  CAN_ST_NOT_READY,   // begin() not called or re-init in progress
  CAN_ST_BAD_PARAM,
};
