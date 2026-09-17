#pragma once
// ICarLink over the UART to the ATmega328 board in the radio.
//
// That board presents an ELM327-style AT surface (see
// ../jeep-aux-and-can-gateway/docs/AT-COMMANDS.md). It is observe-only: there
// is no command here that can touch the car's VES announce, radio source or
// Bluetooth, by design.
//
// Session settings chosen here, and why:
//   AT@SML0 / AT@SM0     -- SILENCE FIRST. Sent before anything else, because
//                           the 328 is not reset by us opening the port and may
//                           be streaming right now from a previous session, in
//                           a format we have not agreed to yet. See begin().
//   ATE0   echo off      -- we would otherwise have to skip our own commands
//   ATL0   no linefeeds  -- lines end with a bare CR, one delimiter to handle
//   ATS0   no spaces     -- a RESPONSE line becomes exactly 3 id nibbles + 2n
//                           data nibbles, which parses deterministically. It no
//                           longer governs the broadcast shape: LAWICEL lines
//                           ignore ATS and ATL entirely.
//   ATH1   headers on    -- we must know which id answered
//   ATCAF0 no formatting -- we build the ISO-TP PCI byte ourselves
//   AT@SML2 stream       -- LAWICEL/SLCAN frames in the background, each with
//                           four hex digits of the adapter's millis() % 60000.
//                           See setStreamFormat() for the ladder this falls
//                           down when an older adapter refuses it.
//
// WHY LAWICEL AND NOT THE ELM SHAPE, since this class asked for AT@SM1 for its
// whole life before now. Two reasons, and the second is the one that mattered:
//
//   * A leading 't' marks a frame line outright, so demuxing the stream from the
//     command replies stops being an arithmetic argument about odd lengths and
//     becomes one character.
//   * THE STAMP WIDTH BECOMES MEASURABLE. In the ELM shape the data length is
//     only implied by the hex count, so a 19-character line is either 8 data
//     bytes unstamped or 6 plus a 4-digit stamp -- both valid, and the width had
//     to be TOLD to the parser with no way to check it. LAWICEL carries a DLC
//     nibble, so the width is arithmetic (core/LinkLine.h spells it out) and a
//     mismatch between what we asked for and what arrives is a complaint in the
//     log instead of a dashboard full of plausible wrong numbers.
//
// The stamp is what makes the change worth making rather than merely tidier.
// Broadcast frames now reach the sink with the time the GATEWAY saw them, mapped
// onto our clock by core/StreamClock.h, so core/Trip.cpp's fuel integral and
// core/Accel.cpp's splits are measured on the car's intervals rather than on
// whenever the reader task got round to the line.
#include <stdint.h>

#include "CarLink.h"
#include "LinkLine.h"
#include "StreamClock.h"
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

// Which shape the adapter has agreed to stream in.
//
// Not a cosmetic difference: it selects which parser the reader hands each line
// to, and the two disagree about where a frame's data ends. It is set once, by
// begin(), while the session is coming up -- the reader takes a single copy per
// line so a re-negotiation landing mid-line cannot have the classifier and the
// decoder read the same characters differently.
enum LinkWire : uint8_t {
  LINK_WIRE_NONE = 0,   // not streaming: the ATMA windows, or nothing at all
  LINK_WIRE_LAWICEL,    // AT@SML: 't' + id + DLC + data + stamp, bare CR
  LINK_WIRE_ELM,        // AT@SM: the ELM327 shape, the pre-2.2 arrangement
};

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

  // True when the adapter accepted one of our stream levels: it streams frames
  // in the background and requests no longer take turns with monitoring. False
  // against a 2.0 adapter, which still needs the ATMA alternation.
  bool streaming() const { return streaming_; }

  // Which shape the stream is actually in, LINK_WIRE_NONE when it is off.
  LinkWire wire() const { return (LinkWire)wire_; }

  // Which format and level to ask for, in begin(). The default is
  // LINK_WIRE_LAWICEL at level 2 -- AT@SML2, LAWICEL with the 4-digit rolling
  // stamp -- and that is what the whole of this class is now written around.
  //
  // Must be set BEFORE begin(), because begin() is what negotiates and the
  // reader has to know which parser to use before it classifies the first line.
  // Levels outside 1..3 are refused: 0 would turn the stream off through a door
  // that looks like configuration, and the way to run without a stream is an
  // adapter that refuses the command.
  //
  // THE FALLBACK LADDER IS NOT CONFIGURABLE, and it is three rungs because each
  // one is a real adapter:
  //
  //   AT@SML2  the 2.1 gateway in the radio. Stamped LAWICEL, both halves of the
  //            link live at once, and the reader can CHECK the stamp width.
  //   AT@SM1   a 2.1 gateway that somehow knows AT@SM and not AT@SML. Unstamped
  //            ELM-shaped streaming: the overlap of requests and broadcasts is
  //            kept, the timestamps are not. Strictly better than windows, so
  //            worth a rung rather than falling straight through.
  //   ATMA     a 2.0 adapter. Modal windows -- listen, go deaf, spend a few
  //            requests -- which is what this firmware did before 2.1.
  void setStreamFormat(LinkWire wire, uint8_t level);

  // The level the adapter actually ACCEPTED, 0 when it is not streaming. Not
  // necessarily the one asked for -- see the ladder above.
  uint8_t streamLevel() const { return streamLevel_; }

  // The stamp width in characters (0, 4 or 8) the accepted level produces.
  //
  // On the LAWICEL path this is what we ASKED FOR and the reader compares every
  // line's DERIVED width against it, so a disagreement is logged rather than
  // decoded. On the ELM path it is the only width there is, because that shape
  // cannot be measured -- which is the asymmetry the whole switch was about.
  uint8_t stampWidth() const { return stampChars_; }

  // Stop the stream: AT@SML0, or AT@SM0 when the ELM fallback is what is
  // running. Safe to call at any time and on any adapter -- a 2.0 board answers
  // '?' and there was nothing to stop.
  //
  // begin() uses it to get to a KNOWN QUIET STATE before configuring anything,
  // which is the one place it is load-bearing. The 328 is flashed over ICSP and
  // has no bootloader on these pins, so opening the port does not reset it: it
  // may be mid-stream from a previous session of ours, in a format this session
  // has not agreed to, and every one of those frame lines would then be
  // classified against the wrong parser and pile into replyQ_ -- which is eight
  // slots deep and is where the answer to ATZ has to arrive.
  LinkStatus streamStop();

  // The adapter's own clock, as read off the most recently decoded stream line:
  // RAW, rolling, in the gateway's epoch. False before the first stamped line.
  //
  // This is the unmapped sample, kept for diagnostics only -- what the sink gets
  // is the MAPPED value, see streamClock() below and readerLoop. Two clocks in
  // one accessor would be the kind of ambiguity this file exists to avoid.
  bool lastStampMs(uint32_t& out) const {
    if (!haveStamp_) {
      return false;
    }
    out = lastStampMs_;
    return true;
  }

  // The mapping from the adapter's rolling stamp onto our timebase, for a
  // diagnostics screen: sample count, rollovers crossed, offset resyncs, and
  // streamClockSkewMs() -- the smallest transport latency it has observed.
  //
  // Returned by const reference and read from another task while the reader
  // writes it. Every field is a naturally aligned word, nothing here decides
  // anything, and a torn read costs one wrong number in a log line -- which is
  // the same trade the volatile counters above make.
  const StreamClock& streamClock() const { return clock_; }

  // Stream lines whose DERIVED stamp width was not the one we asked for.
  //
  // Zero is the only healthy value and it is the whole reason for moving to a
  // self-describing wire: on the ELM shape this quantity could not be measured
  // at all, so the same disagreement showed up as two data bytes read as a time
  // and nothing to count. Non-zero means the adapter is in a mode we did not
  // set.
  uint32_t stampWidthMismatches() const { return stampMismatch_; }

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

  // NO frame parsing here: both decoders are core/LinkLine's, beside the
  // classifier whose accepted length sets and timestamp widths they have to
  // match exactly. As private statics of this class they sat in an ESP-IDF
  // translation unit that no host test could link, so that agreement was prose;
  // now one suite sweeps accept-iff-decode across both shapes. readerLoop()
  // still owns WHICH PARSER and WHICH WIDTH -- see the note there about taking
  // a single copy of wire_ and stampChars_ per line.

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

  // Same deal, and the same reason volatile is enough -- except that a stale
  // read here would not misroute a frame, it would MISREAD one. So the reader
  // takes a single copy per line and uses it for both the classification and
  // the decode; begin() is the only writer and only while bringing the session
  // up. stampChars_ is derived from streamLevel_ and never set independently.
  volatile uint8_t  wire_;         // LinkWire, the shape in force
  volatile uint8_t  streamLevel_;
  volatile uint8_t  stampChars_;
  volatile uint32_t lastStampMs_;
  volatile bool     haveStamp_;
  volatile uint32_t stampMismatch_;

  // Owned by the reader task, which is its only writer. Not volatile: it is a
  // structure, so a single read cannot be atomic whatever the qualifier says,
  // and the only cross-task reader is a diagnostics line.
  StreamClock clock_;

  uint8_t     wireReq_;            // what to ask for, LinkWire
  uint8_t     streamLevelReq_;     // ...and at which level

  bool        ready_;
  bool        monitoring_;
  bool        streaming_;

  At328FrameSink sink_;
  QueueHandle_t  frameQ_;      // responses: id == respId_
  QueueHandle_t  replyQ_;      // OK / ? / NO DATA / status text / prompt
  TaskHandle_t   reader_;
};
