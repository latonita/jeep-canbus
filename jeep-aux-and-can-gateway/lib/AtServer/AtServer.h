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
// ---- frame timestamps: the AT@SM levels ----------------------------------
// AT@SM is a LEVEL, not a switch, the way ELM327 spells ATAT0/1/2:
//   0  stream off (default)
//   1  stream on, no timestamp     -- byte-identical to the original AT@SM1
//   2  stream on + 4 hex digits of millis() % 60000, bit-exact LAWICEL "Z1"
//   3  stream on + 8 hex digits of raw millis(), ~49.7 days, no unwrapping
// Out-of-range digits answer '?' rather than being clamped, so AT@SM4 reads as
// the typo it is instead of quietly behaving like AT@SM1.
//
// Folding the timestamp into AT@SM rather than giving it its own command is
// what keeps this backward compatible AND free: levels 0 and 1 mean exactly
// what they meant before, query and AT@ST field included, and stream_ going
// from bool to uint8_t costs ZERO bytes of RAM. AT@ST did gain one field for
// AT@SML (` SML=`), appended inside the stream group -- the alternative was
// letting ` SM=` print 0..6, which would have made the field a lie for any
// host that reads it as "the AT@SM level".
//
// The stamp goes at the END of the line, where SLCAN puts it, and that
// position is load-bearing: 3 + 2*n + 4 and 3 + 2*n + 8 are both still ODD, so
// the host demux rule "pure hex, no space, odd length => frame line" keeps
// working. A prefix would break every parser already deployed.
//
// ---- AT@SML: the same frames in LAWICEL/SLCAN wire format ----------------
// AT@SML switches the stream to the format an off-the-shelf slcan parser
// already speaks, so a host can point existing SLCAN tooling at this board
// instead of teaching it the ELM shape:
//   0  off
//   1  LAWICEL, NO timestamp -- plain slcan
//   2  LAWICEL + 4 hex digits of millis() % 60000
//   3  LAWICEL + 8 hex digits of raw millis()
// Exactly mirroring AT@SM, so one mental model covers both formats.
//
// Level 1 is the point of the whole command. python-can's `slcan` reader and
// can-utils' `slcand` both default to LAWICEL Z0 -- timestamps OFF -- so a
// stamped line hands them 4 or 8 hex digits they never asked for. AT@SML1 is
// therefore the off-the-shelf-compatible level and AT@SML2/3 are the opt-ins,
// precisely as AT@SM1 is the compatible level on the ELM-shaped side.
//
//   't' + 3 hex id + 1 hex DLC + 2 hex per data byte + stamp + CR
//   id 0x1A7, 6 bytes 02 41 00 00 00 00, stamp 0xB3A7:
//   t1A76024100000000B3A7
//
// Three deviations from the ELM path are deliberate, and each one is what makes
// a stock slcan parser work:
//   * NO SPACES, ever -- ATS1 is ignored. LAWICEL has no space variant.
//   * CR ONLY -- ATL1 is ignored, and these lines do NOT go through
//     writeTerm(). An slcan parser splits on CR, so a trailing LF becomes a
//     LEADING LF on the next line and eats a whole frame, not just a newline.
//   * lowercase 't' -- in LAWICEL 't' is an 11-bit standard DATA frame and 'T'
//     is 29-bit extended. CanFrame holds a uint16_t id with no extended and no
//     RTR flag, so this firmware can never emit anything but 't'. That is a
//     real limit of the struct, not a shortcut taken here.
//
// The format is strictly the easier one to demux. 't' is not a hex digit, so
// it is an unambiguous frame marker -- nothing else this firmware emits starts
// with 't' (the banner starts with 'l', replies are OK / ? / NO DATA /
// CAN ERROR / BUFFER FULL, and AT@EV/AT@ST lines start with their own text) --
// which beats the odd-length parity rule the ELM format has to rely on. And
// because the DLC nibble says how many data bytes follow, a LAWICEL host can
// DETECT the stamp from the line length instead of having to be told the
// width, the one ambiguity the ELM format cannot escape.
//
// ---- one field, so the two formats cannot both be on --------------------
// AT@SM and AT@SML write the SAME stream_ field, widened into the StreamMode
// enum below rather than given a second streamL_ of its own. Two independent
// variables would create an invariant ("never both non-zero") that every
// "is the stream on?" site would have to honour, and one missed site silently
// enables or disables the stream -- a bug that shows up either as a link that
// is mysteriously quiet, or as two renderings of every frame at double the
// bandwidth. With one field the exclusivity is structural: every existing
// `stream_ != 0u` test keeps working verbatim, and it costs zero extra RAM.
//
// ---- which lines get which format ---------------------------------------
// Both unsolicited producers -- the background stream and ATMA -- use the
// ACTIVE format and the ACTIVE width, so under AT@SML1 ATMA emits LAWICEL
// lines too: a host that declared slcan on this link must not suddenly get
// ELM-shaped lines back from ATMA. Request/response lines are the other way
// round -- ALWAYS the ELM format and NEVER stamped, because slcan has no
// request/response notion and those lines belong to the ELM327 surface (ATH,
// NO DATA, the ISO-TP flow) that host parsers already depend on.
//
// A consequence of stamping being part of the stream level rather than its own
// knob, and it is worth knowing before it surprises someone: stamped ATMA
// output cannot be had without the background stream also being armed. During
// ATMA nothing is printed twice (ST_MONITOR short-circuits the stream path),
// but the moment any byte ends ATMA the board is back at ST_IDLE with a live
// level-2/3 (or LAWICEL) stream, so a host that expected silence after ATMA
// keeps receiving frames. Send AT@SM0 if that is not wanted.
//
// Accuracy ceiling: frames are POLLED out of the MCP2515 by main.cpp's drain
// loop, so the stamp records when the loop reached the controller, not bus
// arrival. That is +-1-2 ms in practice -- one stamp tick -- which is exactly
// why there is no microsecond mode to promise precision we do not have.
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
//   8 one-byte flags  8  7 bools plus stream_, a uint8_t StreamMode 0..6 --
//                         which is why folding the AT@SM timestamp levels and
//                         the AT@SML format into it added nothing at all
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
  //
  // nowMs is the SAME millis() sample the modules get, taken at the top of
  // loop(). It cannot be left to tick(): tick() runs AFTER the drain loop, so
  // nowMs_ would still hold the previous iteration's sample and every AT@SM
  // stamp would be systematically one loop stale. This is not an override of
  // any interface -- AtServer inherits only IEventSink, and the IModule
  // onFrame() that kModules[] calls is a different, unrelated hierarchy.
  void onFrame(const CanFrame& frame, uint32_t nowMs);

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

  // The single stream field. ELM levels come first so AT@SM<n> maps to n
  // directly and every `stream_ != 0u` test predates and survives this enum.
  // Two formats x three widths, in the same order, which is what lets
  // lawicel() and tsChars() below be one expression each instead of a table.
  enum StreamMode : uint8_t {
    SM_OFF = 0,        // AT@SM0 / AT@SML0
    SM_ELM = 1,        // AT@SM1   ELM format, no stamp  (unchanged since 2.1)
    SM_ELM_TS16 = 2,   // AT@SM2   ELM format + 4 hex
    SM_ELM_TS32 = 3,   // AT@SM3   ELM format + 8 hex
    SM_LAW = 4,        // AT@SML1  LAWICEL, no stamp -- plain slcan
    SM_LAW_TS16 = 5,   // AT@SML2  LAWICEL + 4 hex
    SM_LAW_TS32 = 6,   // AT@SML3  LAWICEL + 8 hex
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
  // stamp=true appends the AT@SM2/3 timestamp (when enabled). Only the two
  // unsolicited producers pass true. Request/response lines are NEVER
  // stamped: ATH already exists for responses because the host knows what it
  // asked for, so a stamp there would buy nothing and would break every
  // deployed ISO-TP host parser that counts bytes off the end of the line.
  void writeFrame(const CanFrame& frame, bool forceHeader, bool stamp);

  // One unsolicited frame line in the ACTIVE format. Both producers -- the
  // background stream and ATMA -- go through here, which is what keeps ATMA
  // from answering in ELM shape on a link that asked for slcan.
  void writeStreamFrame(const CanFrame& frame);
  void writeLawicelFrame(const CanFrame& frame);

  void streamFrame(const CanFrame& frame);     // AT@SM/AT@SML, never blocks

  // Characters the timestamp occupies at the current AT@SM level: 4, 8 or
  // none. SINGLE source of truth on purpose -- streamFrame()'s byte
  // accounting and writeFrame()'s emitting code both read it, and if the two
  // ever drifted the writableBytes() test would under-count, writeBytes()
  // would block on a full ring and loop() would stall. That is the exact
  // failure the accounting exists to prevent.
  uint8_t tsChars() const {
    // Within either format the level offset IS the width: 1 and 4 unstamped,
    // 2 and 5 four nibbles, 3 and 6 eight. Derived rather than tabulated so a
    // width cannot be taught to one format and forgotten in the other. The
    // SM_OFF guard is what keeps the subtraction from underflowing.
    if (stream_ == SM_OFF) {
      return 0u;
    }
    const uint8_t slot = static_cast<uint8_t>((stream_ - 1u) % 3u);
    return (slot == 0u) ? 0u : ((slot == 1u) ? 4u : 8u);
  }

  // Same reason as tsChars(): derived once, so the byte accounting and the
  // emitter cannot disagree about which format is going out.
  bool lawicel() const { return stream_ >= SM_LAW; }

  // Per-format views of stream_ -- what AT@SM, AT@SML and AT@ST all report.
  uint8_t elmLevel() const;
  uint8_t lawicelLevel() const;

  // Value behind the stamp, already reduced to the level's width.
  uint32_t tsValue() const;

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
  uint8_t stream_;       // StreamMode -- a field, NOT a State; see idle()
  bool skipLf_;          // last input byte was CR: swallow one LF
};
