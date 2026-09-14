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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define AT328_LINE_MAX      64u
#define AT328_CMD_TIMEOUT_MS 300u
#define AT328_RESET_TIMEOUT_MS 1500u

// Depth of the response-frame queue. The worst legal ISO-TP burst is one First
// Frame carrying 6 bytes plus ceil((ISOTP_MAX_PDU - 6) / 7) = 18 Consecutive
// Frames, so 19. Rounded up; it costs 24 x sizeof(CanFrame).
#define AT328_FRAME_Q_DEPTH  24u

// Depth of the command-reply queue. A command produces at most a couple of
// lines plus a prompt; the slack is for AT@EV event lines arriving between.
#define AT328_REPLY_Q_DEPTH  8u

// One classified non-frame line, on its way back to whoever sent a command.
struct At328Reply {
  char text[AT328_LINE_MAX];
  bool prompt;                 // '>' seen; text may be empty
};

// Where broadcast frames go. Called from the reader task, so it must be quick
// and must not block -- broadcastApply() and tripUpdate() qualify, an NVS
// write does not.
typedef void (*At328FrameSink)(const CanFrame& frame, uint32_t nowMs);

// SINGLE REQUESTER BY CONSTRUCTION. There is no mutex here, deliberately.
// A request is setTarget() + sendFrame() + repeated readFrame(), and only the
// whole sequence is meaningful -- a lock inside each method would look safe
// while leaving two callers free to interleave their transactions. Making the
// sequence atomic means changing ICarLink, which would cost core/ its host
// tests for a problem nobody has: carTask is the only task that ever issues a
// request. If a second one ever does (a diagnostics screen polling AT@ST, say)
// the lock belongs around transact(), not in here.
//
// The reader task is not a second requester: it only ever receives.
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

  // Unsolicited frames -- everything that is not the response currently being
  // waited for. Set once, before begin(); without it broadcasts are dropped.
  void setFrameSink(At328FrameSink sink) { sink_ = sink; }

  // True when the adapter accepted AT@SM1: it streams frames in the background
  // and requests no longer take turns with monitoring. False against a 2.0
  // adapter, which still needs the ATMA alternation.
  bool streaming() const { return streaming_; }

  // When the reader last pulled ANY complete line off the UART -- a frame, a
  // command reply, a prompt. Zero until the first one arrives.
  //
  // This is the only honest liveness signal we have. ready() only says begin()
  // once succeeded, which stays true forever after the cable is pulled, and
  // the poll scheduler can go quiet for perfectly good reasons. Silence on the
  // wire is the thing that actually means the adapter is gone.
  uint32_t lastRxMs() const { return lastRxMs_; }

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

  // The one task that ever reads the UART. Everything else waits on a queue.
  static void readerTrampoline(void* self);
  void readerLoop();

  static uint32_t nowMs();
  static bool parseFrameLine(const char* line, uint16_t len, CanFrame& out);

  uart_port_t port_;
  int         txPin_;
  int         rxPin_;
  uint32_t    baud_;

  uint16_t    reqId_;

  // Written by the requesting task, read by the reader task. Plain volatile is
  // enough: both are naturally aligned single words, the reader only compares
  // them, and the worst a torn read can do is route one frame to the wrong
  // place for one iteration.
  volatile uint16_t respId_;
  volatile bool     targetValid_;
  volatile uint32_t lastRxMs_;

  bool        ready_;
  bool        monitoring_;
  bool        streaming_;

  At328FrameSink sink_;
  QueueHandle_t  frameQ_;      // responses: id == respId_
  QueueHandle_t  replyQ_;      // OK / ? / NO DATA / status text / prompt
  TaskHandle_t   reader_;
};
