#pragma once
// Abstract bus, so modules and the AT server never see the MCP2515 directly and
// a fake can be substituted in host tests.
#include "CanFrame.h"

class ICanBus {
 public:
  virtual CanStatus begin() = 0;

  virtual CanStatus send(const CanFrame& frame) = 0;

  // Returns CAN_ST_OK and fills `out`, or CAN_ST_NO_MSG. Never blocks.
  virtual CanStatus receive(CanFrame& out) = 0;

  // Hardware acceptance filtering. mask bit 1 = "this bit must match".
  virtual CanStatus setAcceptance(uint16_t id, uint16_t mask) = 0;
  virtual CanStatus acceptAll() = 0;

  virtual bool healthy() = 0;

  virtual uint16_t rxCount() const = 0;
  virtual uint16_t txCount() const = 0;
  virtual uint16_t errCount() const = 0;

 protected:
  // Non-virtual and protected: these live in static storage for the whole
  // program lifetime, and a virtual dtor would drag in operator delete.
  ~ICanBus() = default;
};
