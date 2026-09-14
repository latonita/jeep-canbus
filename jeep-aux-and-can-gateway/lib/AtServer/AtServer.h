#pragma once
// ELM327-style AT command server for the CAN-B gateway.
//
// The board is a dumb frame pipe: it transmits the bytes the host gives it and
// prints back the frames it hears. There is NO ISO-TP reassembly, NO KWP
// parsing and NO PID decoding here -- all of that lives on the laptop/ESP32.
// The single exception is the automatic Flow Control answer (see lib/IsoTpFc),
// which has to happen on-board because a module will not wait for a round trip
// to the host.
//
// tick() never blocks: a request moves the server into a collect state with a
// deadline and returns immediately, so the VES/BT modules and the CAN drain in
// main.cpp keep running while a module is answering.
//
// ---- stream mode (AT@SM, 2.1) --------------------------------------------
// Classic ELM327 is half duplex by convention, not by necessity: an idle
// adapter is silent, and ATMA streams only until a character arrives, so a
// host that wants both broadcasts and requests has to alternate between them
// and goes deaf for the whole request phase.
//
// Nothing on the wire requires that. main.cpp already drains every frame and
// hands it here on every loop iteration, and the MCP2515's hardware masks are
// wide open for the life of the firmware. AT@SM1 simply stops throwing the
// idle ones away: frames are printed continuously, in the background, INCLUDING
// while a request is collecting its response. The measured bus load is about
// 151 frames/s, near 2.5 kB/s with ATS0, about 22% of the 115200 link.
//
// The stream is a flag rather than a State on purpose -- see idle() below.
//
// ---- RAM budget ----------------------------------------------------------
// Of the 2048 bytes: HardwareSerial buffers 128, SoftwareSerial 64, stack
// ~350. One AtServer is 102 bytes of static data on AVR (no padding, 2-byte
// pointers):
//   AtLine         52  (AT_LINE_MAX 48 + NUL + len + 2 flags)  [measured]
//   IsoTpFcConfig  12  (header + 8 data + len + mode)          [measured]
//   io_/bus_        4  references
//   hook pointers   4  IStatusProvider*, IBenchControl*
//   deadline_/now_  8
//   header/filter/mask  6
//   mfTotal_/mfGot_ 4
//   stUnits_/state_/rxSeen_  3
//   streamDropped_  2  AT@SM frames lost to a full TX ring
//   8 bool flags    8
//   vtable pointer  2  (IEventSink)
// Nothing else is buffered: response frames are formatted straight out to the
// link. The largest transients are stack-only -- a 25-byte hex buffer in
// writeFrame() and a 17-byte flash-string chunk in writeP(), never nested,
// so under ~50 bytes of stack. No new/malloc/String/dynamic container
// anywhere, and no response frame queue: if the host stops reading, the UART
// applies the backpressure.
#include <stdint.h>

#include "CanBus.h"
#include "Events.h"
#include "Version.h"

#include "AtLine.h"
#include "AtStatus.h"
#include "ByteIo.h"
#include "IsoTpFc.h"

// Placeholder request header. There is no universal diagnostic id on the Jeep
// interior bus, so a session always starts with ATSH -- this only keeps the
// first ATSH-less transmit from having to be rejected.
#define AT_DEFAULT_HEADER  0x7E0u

// ELM ATST unit is 4 ms.
#define AT_ST_UNIT_MS      4u

// Bounded work per tick(), so the AT link can never starve the CAN drain.
#define AT_RX_BYTES_PER_TICK  32u

class AtServer : public IEventSink {
 public:
  AtServer(IByteIo& io, ICanBus& bus);

  // Prints the identity banner and the first prompt.
  void begin();

  // Call every loop iteration with the same sampled millis() as the modules.
  void tick(uint32_t nowMs);

  // main.cpp must hand every frame it drains from ICanBus::receive() to this.
  // AtServer deliberately never calls receive() itself: there is one drain
  // loop in main.cpp feeding all consumers, and two readers would race for
  // frames. Implementations MUST validate frame.len before indexing data --
  // this one does.
  void onFrame(const CanFrame& frame);

  // IEventSink. Formats one "@EV <tag> <value>" line, but only when AT@EV1 is
  // on (default off, so plain request/response traffic stays clean).
  void emit(EventCode code, uint16_t value) override;

  // True when no command is mid-flight and no response is pending. BtControl
  // uses this to defer its SoftwareSerial writes: SoftwareSerial disables
  // interrupts while shifting bits out and would corrupt bytes arriving on
  // the hardware UART.
  //
  // THIS IS WHY STREAM MODE IS A FLAG AND NOT A State. A fourth state would
  // make idle() false for as long as streaming is on, BtControl would defer
  // its writes forever, and the steering-wheel buttons would stop working --
  // which on the bench looks exactly like a hardware fault.
  bool idle() const;

  // Abort anything in flight and leave the link at a prompt. Called from the
  // supervisor's safe-state path.
  void safeState();

  // ---- main.cpp wiring (see AtStatus.h) ----------------------------------
  // Both are optional; NULL is handled (AT@ST prints the fields it can, and
  // AT@BENCH answers '?' when nothing is wired).
  void setStatusProvider(IStatusProvider* provider) { status_ = provider; }
  void setBenchControl(IBenchControl* bench) { bench_ = bench; }

 private:
  enum State : uint8_t {
    ST_IDLE = 0,      // waiting for a command line
    ST_COLLECT,       // request sent, printing replies until the deadline
    ST_MONITOR,       // ATMA: streaming every frame until a byte arrives
  };

  enum Reply : uint8_t {
    RP_NONE = 0,      // command already produced its own output
    RP_PENDING,       // moved to ST_COLLECT/ST_MONITOR; no prompt yet
    RP_OK,
    RP_QUERY,         // '?' -- unrecognised
    RP_NO_DATA,
    RP_CAN_ERROR,
    RP_BUFFER_FULL,
  };

  // ---- output ----
  // writeP/writeMsg take a pointer into PROGMEM (PGM_P is just const char*),
  // copied to a 17-byte stack buffer in chunks. All reply text lives in flash.
  void writeTerm();
  void writeP(const char* progmemStr);         // no terminator
  void writeMsg(const char* progmemStr);       // one PROGMEM line + terminator
  void writeReply(Reply r);
  void writePrompt();
  void writeFrame(const CanFrame& frame, bool forceHeader);
  void streamFrame(const CanFrame& frame);     // AT@SM, never blocks

  // ---- input ----
  void pumpInput();
  Reply dispatch(const char* s);
  Reply cmdAt(const char* s);                  // s points past "AT"
  Reply cmdCustom(const char* s);              // s points past "AT@"
  Reply cmdData(const char* s);                // bare hex line
  Reply statusLine();

  // ---- helpers ----
  bool frameAccepted(const CanFrame& frame) const;
  uint32_t timeoutMs() const;
  void resetDefaults();
  void finish(Reply r);                        // print reply + prompt, go idle
  void noteResponseFrame(const CanFrame& frame);

  IByteIo& io_;
  ICanBus& bus_;
  IStatusProvider* status_;
  IBenchControl* bench_;

  AtLine line_;
  IsoTpFcConfig fc_;

  uint16_t header_;      // ATSH
  uint16_t filter_;      // ATCF / ATCRA
  uint16_t mask_;        // ATCM / ATCRA (0 = accept everything)
  uint32_t deadline_;
  uint32_t nowMs_;

  uint16_t mfTotal_;     // FF_DL of the reply being collected
  uint16_t mfGot_;       // payload bytes seen so far

  uint16_t streamDropped_;  // AT@SM frames dropped, no TX room (saturating)

  uint8_t  stUnits_;     // ATST, 0 = use AT_RESP_TIMEOUT_MS
  State    state_;
  uint8_t  rxSeen_;      // frames printed for the current request (saturating)

  bool echo_;            // ATE
  bool linefeed_;        // ATL
  bool spaces_;          // ATS
  bool headers_;         // ATH
  bool autoFormat_;      // ATCAF
  bool events_;          // AT@EV
  bool stream_;          // AT@SM -- a flag, NOT a State; see idle()
  bool skipLf_;          // last input byte was CR: swallow one LF
};
