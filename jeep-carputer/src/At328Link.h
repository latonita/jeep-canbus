#pragma once
// ICarLink over the UART to the ATmega328 board in the radio.
//
// That board presents an ELM327-style AT surface (see
// ../jeep-aux-and-can-gateway/docs/AT-COMMANDS.md). It is observe-only: there
// is no command here that can touch the car's VES announce, radio source or
// Bluetooth, by design.
//
// Session settings chosen here, and why:
//   ATE0   echo off      -- we would otherwise have to skip our own commands
//   ATL0   no linefeeds  -- lines end with a bare CR, one delimiter to handle
//   ATS0   no spaces     -- a frame line becomes exactly 3 id nibbles + 2n
//                           data nibbles, which parses deterministically
//   ATH1   headers on    -- we must know which id answered
//   ATCAF0 no formatting -- we build the ISO-TP PCI byte ourselves
#include <stdint.h>

#include "CarLink.h"
#include "driver/uart.h"

#define AT328_LINE_MAX      64u
#define AT328_CMD_TIMEOUT_MS 300u
#define AT328_RESET_TIMEOUT_MS 1500u

class At328Link : public ICarLink {
 public:
  At328Link(uart_port_t port, int txPin, int rxPin, uint32_t baud);

  LinkStatus begin() override;
  LinkStatus setTarget(uint16_t requestId, uint16_t responseId) override;
  LinkStatus sendFrame(const CanFrame& frame) override;
  LinkStatus readFrame(CanFrame& out, uint32_t timeoutMs) override;
  bool ready() override { return ready_; }
  LinkStatus monitorBegin() override;
  LinkStatus monitorEnd() override;

  // Adapter status line (AT@ST) into the caller's buffer, for a diagnostics
  // screen. Does not parse it -- the format is the adapter's business.
  LinkStatus status(char* out, uint16_t cap);

 private:
  // Reads one CR-terminated line, or returns as soon as the '>' prompt
  // arrives (reported as an empty line with promptSeen set). Never blocks
  // past the deadline.
  LinkStatus readLine(char* out, uint16_t cap, uint16_t& len,
                      bool& promptSeen, uint32_t deadlineMs);

  // Writes a command and consumes lines until OK / an error / the prompt.
  LinkStatus command(const char* cmd, uint32_t timeoutMs);

  void writeLine(const char* s);
  void drain();

  static uint32_t nowMs();
  static bool parseFrameLine(const char* line, uint16_t len, CanFrame& out);

  uart_port_t port_;
  int         txPin_;
  int         rxPin_;
  uint32_t    baud_;

  uint16_t    reqId_;
  uint16_t    respId_;
  bool        targetValid_;
  bool        ready_;
  bool        monitoring_;
};
