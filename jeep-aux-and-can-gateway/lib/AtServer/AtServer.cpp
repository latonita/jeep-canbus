// AT command server -- AVR side. Everything Arduino-specific in the AT layer
// lives in this one file, which is why it is skipped in the host test build:
// test/ compiles only the pure AtLine and IsoTpFc translation units, and the
// native env defines UNIT_TEST.
#ifndef UNIT_TEST

#include "AtServer.h"

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "JeepCan.h"  // VIN_MAX_LEN, for the AT@VIN buffer

// Every literal below is wrapped in PSTR()/PROGMEM. On AVR a plain
// const char* literal used at runtime is copied into RAM at startup, and we
// only have 2 KB, so nothing here costs RAM beyond small stack buffers.

// Longest flash-string chunk copied to the stack at a time.
#define AT_STR_CHUNK   16u
// Hard cap on a PROGMEM string, so a bad pointer cannot spin forever.
#define AT_STR_MAX     64u
// Frames printed for one request before we stop and hand back the prompt.
#define AT_FRAMES_MAX  255u

// ---- small helpers -------------------------------------------------------

// If s starts with the flash prefix, returns the rest of s, else NULL.
static const char* after(const char* s, PGM_P pfx) {
  const size_t n = strlen_P(pfx);
  if (strncmp_P(s, pfx, n) == 0) {
    return s + n;
  }
  return NULL;
}

// Exact "0"/"1" argument.
static bool onOff(const char* s, bool& out) {
  if (s == NULL || s[0] == '\0' || s[1] != '\0') {
    return false;
  }
  if (s[0] == '0') {
    out = false;
    return true;
  }
  if (s[0] == '1') {
    out = true;
    return true;
  }
  return false;
}

// Free RAM arrives through IStatusProvider rather than being measured here.
// Poking at __heap_start / __brkval is the platform's business, not the
// protocol layer's, and having two copies of it in two libraries meant two
// different declared types for the same linker symbols.

// ---- construction / defaults --------------------------------------------

AtServer::AtServer(IByteIo& io, ICanBus& bus)
    : io_(io),
      bus_(bus),
      status_(NULL),
      bench_(NULL),
      line_(),
      fc_(),
      header_(AT_DEFAULT_HEADER),
      filter_(0u),
      mask_(0u),
      deadline_(0u),
      nowMs_(0u),
      mfTotal_(0u),
      mfGot_(0u),
      stUnits_(0u),
      state_(ST_IDLE),
      rxSeen_(0u),
      echo_(true),
      linefeed_(true),
      spaces_(true),
      headers_(false),
      autoFormat_(true),
      events_(false),
      skipLf_(false) {
  isoTpFcDefaults(fc_);
}

void AtServer::resetDefaults() {
  line_.reset();
  isoTpFcDefaults(fc_);
  header_ = AT_DEFAULT_HEADER;
  filter_ = 0u;
  mask_ = 0u;
  stUnits_ = 0u;
  state_ = ST_IDLE;
  rxSeen_ = 0u;
  mfTotal_ = 0u;
  mfGot_ = 0u;
  echo_ = true;
  linefeed_ = true;
  spaces_ = true;
  headers_ = false;
  autoFormat_ = true;
  events_ = false;
  skipLf_ = false;
  // Deliberately does NOT re-initialise the CAN controller.
  //
  // ATZ resets the AT layer only. Re-initialising the MCP2515 takes the node
  // off the bus for ~100 ms (bounded begin() plus mask programming), during
  // which VES announce cannot transmit. A host that resets in a loop -- which
  // is exactly what a serial tool does when it cannot parse the greeting --
  // would then stall the announce, the head unit would drop its AUX source,
  // and the music would stop. The car must never depend on the host, so
  // controller health belongs to Supervisor alone.
}

void AtServer::begin() {
  writeMsg(PSTR(FW_IDENT));
  writePrompt();
}

uint32_t AtServer::timeoutMs() const {
  return (stUnits_ == 0u) ? static_cast<uint32_t>(AT_RESP_TIMEOUT_MS)
                          : static_cast<uint32_t>(stUnits_) * AT_ST_UNIT_MS;
}

bool AtServer::idle() const {
  return state_ == ST_IDLE && !line_.partial();
}

void AtServer::safeState() {
  if (state_ != ST_IDLE) {
    state_ = ST_IDLE;
    mfTotal_ = 0u;
    mfGot_ = 0u;
    writePrompt();
  }
  line_.reset();
}

// ---- output --------------------------------------------------------------

void AtServer::writeTerm() {
  io_.writeByte('\r');
  if (linefeed_) {
    io_.writeByte('\n');
  }
}

void AtServer::writeP(const char* progmemStr) {
  char buf[AT_STR_CHUNK + 1u];
  uint8_t n = 0u;
  for (uint8_t i = 0u; i < AT_STR_MAX; ++i) {
    const char c = static_cast<char>(pgm_read_byte(progmemStr + i));
    if (c == '\0') {
      break;
    }
    buf[n++] = c;
    if (n == AT_STR_CHUNK) {
      buf[n] = '\0';
      io_.writeBytes(buf);
      n = 0u;
    }
  }
  if (n != 0u) {
    buf[n] = '\0';
    io_.writeBytes(buf);
  }
}

void AtServer::writeMsg(const char* progmemStr) {
  writeP(progmemStr);
  writeTerm();
}

void AtServer::writeReply(Reply r) {
  switch (r) {
    case RP_OK:          writeMsg(PSTR("OK")); break;
    case RP_QUERY:       writeMsg(PSTR("?")); break;
    case RP_NO_DATA:     writeMsg(PSTR("NO DATA")); break;
    case RP_CAN_ERROR:   writeMsg(PSTR("CAN ERROR")); break;
    case RP_BUFFER_FULL: writeMsg(PSTR("BUFFER FULL")); break;
    case RP_NONE:
    case RP_PENDING:
    default:
      break;   // the command already printed its own output
  }
}

void AtServer::writePrompt() {
  writeTerm();
  io_.writeByte('>');
}

void AtServer::writeFrame(const CanFrame& frame, bool forceHeader) {
  if (forceHeader) {
    char idbuf[4];
    (void)atFormatId11(frame.id, idbuf, sizeof(idbuf));
    io_.writeBytes(idbuf);
    if (spaces_) {
      io_.writeByte(' ');
    }
  }
  // Never trust len: a driver bug or a bus glitch must not walk off data[].
  const uint8_t len = (frame.len > CAN_MAX_DLC) ? CAN_MAX_DLC : frame.len;
  if (len != 0u) {
    char buf[CAN_MAX_DLC * 3u + 1u];   // 25 stack bytes, worst case "AA .. FF"
    (void)atFormatHexBytes(frame.data, len, buf, sizeof(buf), spaces_);
    io_.writeBytes(buf);
  }
  writeTerm();
}

void AtServer::finish(Reply r) {
  writeReply(r);
  writePrompt();
  state_ = ST_IDLE;
  mfTotal_ = 0u;
  mfGot_ = 0u;
}

// ---- tick / input --------------------------------------------------------

void AtServer::tick(uint32_t nowMs) {
  nowMs_ = nowMs;
  pumpInput();

  if (state_ == ST_COLLECT) {
    // Signed difference, so the 49-day millis() rollover is a non-event.
    if (static_cast<int32_t>(nowMs_ - deadline_) >= 0) {
      finish((rxSeen_ == 0u) ? RP_NO_DATA : RP_NONE);
    }
  }
}

void AtServer::pumpInput() {
  for (uint8_t i = 0u; i < AT_RX_BYTES_PER_TICK; ++i) {
    const int b = io_.readByte();
    if (b < 0) {
      return;
    }
    const char c = static_cast<char>(b);

    // Swallow the LF of a CRLF pair. Without this a host that terminates its
    // lines with "\r\n" would have its own line ending abort the request it
    // just sent.
    if (skipLf_ && c == '\n') {
      skipLf_ = false;
      continue;
    }
    skipLf_ = (c == '\r');

    switch (state_) {
      case ST_MONITOR:
        // ELM stops AT MA on any received character, and consumes rather than
        // executes it. Acceptance was not altered on entry, so there is
        // nothing to restore.
        finish(RP_NONE);
        continue;
      case ST_COLLECT:
        // ELM aborts the operation in progress on any received character.
        finish(RP_NONE);
        continue;
      case ST_IDLE:
      default:
        break;
    }

    if (echo_) {
      if (c == '\r' || c == '\n') {
        writeTerm();
      } else {
        io_.writeByte(c);
      }
    }

    const AtLineStatus ls = line_.push(c);
    switch (ls) {
      case AT_LINE_READY: {
        const Reply r = dispatch(line_.text());
        if (r != RP_PENDING) {
          finish(r);
        }
        if (state_ != ST_IDLE) {
          // A request is in flight now; stop draining input this tick so the
          // next queued byte does not immediately abort it.
          return;
        }
        break;
      }
      case AT_LINE_OVERFLOW:
        finish(RP_BUFFER_FULL);
        break;
      case AT_LINE_EMPTY:
        // Bare Enter. ELM repeats the previous command here; we do nothing,
        // and print no prompt, so CRLF hosts see exactly one prompt per
        // command.
        break;
      case AT_LINE_PENDING:
      default:
        break;
    }
  }
}

// ---- CAN frames in ------------------------------------------------------

void AtServer::onFrame(const CanFrame& frame) {
  if (!frameAccepted(frame)) {
    return;
  }
  switch (state_) {
    case ST_MONITOR:
      writeFrame(frame, true);   // AT MA always shows the id
      break;
    case ST_COLLECT:
      noteResponseFrame(frame);
      break;
    case ST_IDLE:
    default:
      break;                     // nobody is listening on the link
  }
}

void AtServer::noteResponseFrame(const CanFrame& frame) {
  writeFrame(frame, headers_);
  if (rxSeen_ < AT_FRAMES_MAX) {
    ++rxSeen_;
  } else {
    // Iteration cap: a chatty bus cannot hang us. This is the other documented
    // meaning of BUFFER FULL -- frames arriving faster than they can be
    // printed, with the rest dropped.
    finish(RP_BUFFER_FULL);
    return;
  }

  switch (isoTpClassify(frame)) {
    case ISOTP_TYPE_SINGLE:
      // A single-frame answer is a complete message, so there is no reason to
      // sit out the rest of the ATST window. (ELM keeps listening for other
      // responders; with a filter set that only adds latency.)
      finish(RP_NONE);
      break;

    case ISOTP_TYPE_FIRST: {
      mfTotal_ = isoTpFirstFrameLen(frame);
      mfGot_ = (frame.len > 2u) ? static_cast<uint16_t>(frame.len - 2u) : 0u;

      CanFrame fc;
      const IsoTpFcStatus st = isoTpBuildFc(fc_, frame, header_, fc);
      switch (st) {
        case ISOTP_FC_OK:
          // This is the one bit of ISO-TP the adapter must do itself: without
          // it the module never sends its consecutive frames.
          if (bus_.send(fc) != CAN_ST_OK) {
            finish(RP_CAN_ERROR);
            return;
          }
          break;
        case ISOTP_FC_NOT_FIRST_FRAME:
        case ISOTP_FC_NO_HEADER:
        case ISOTP_FC_BAD_CONFIG:
        default:
          finish(RP_CAN_ERROR);
          return;
      }
      // Consecutive frames can take longer than one ATST window; every frame
      // that arrives buys another one. Bounded by FF_DL and AT_FRAMES_MAX.
      deadline_ = nowMs_ + timeoutMs();
      break;
    }

    case ISOTP_TYPE_CONSEC:
      if (mfTotal_ != 0u) {
        mfGot_ = static_cast<uint16_t>(mfGot_ + isoTpConsecPayloadLen(frame));
        if (mfGot_ >= mfTotal_) {
          finish(RP_NONE);
          return;
        }
        deadline_ = nowMs_ + timeoutMs();
      }
      break;

    case ISOTP_TYPE_FLOW:
    case ISOTP_TYPE_UNKNOWN:
    default:
      // Not ISO-TP (or our own FC echoed back). We cannot know when it is
      // complete, so let the ATST deadline decide.
      break;
  }
}

// ---- acceptance ---------------------------------------------------------

// The host's ATCRA/ATCF/ATCM filter is applied in SOFTWARE, here, and is
// never pushed into the MCP2515's hardware masks.
//
// Reason: the autonomous modules need 0x09F (radio mode) and 0x394 (panel
// buttons) at all times. A hardware filter set for a diagnostic session --
// say ATCRA 514 to poll the cluster, the normal state once the ESP32 is
// running -- would stop those frames reaching the controller at all, freezing
// the Bluetooth power pin and killing the seek buttons for the duration. The
// bus is 83.3 kbps and the original firmware ran with hardware filtering
// effectively disabled for ten years, so matching in software costs nothing
// we can measure and removes the coupling entirely.
bool AtServer::frameAccepted(const CanFrame& frame) const {
  if (mask_ == 0u) {
    return true;
  }
  return ((frame.id ^ filter_) & mask_) == 0u;
}

// ---- dispatch -----------------------------------------------------------

AtServer::Reply AtServer::dispatch(const char* s) {
  if (s[0] == 'A' && s[1] == 'T') {
    if (s[2] == '@') {
      return cmdCustom(s + 3);
    }
    return cmdAt(s + 2);
  }
  return cmdData(s);
}

AtServer::Reply AtServer::cmdAt(const char* s) {
  const char* a;
  bool flag = false;
  uint16_t id = 0u;

  if (s[0] == '\0') {
    return RP_OK;                        // bare "AT"
  }

  // ---- identity / reset ----
  if (s[0] == 'Z' && s[1] == '\0') {
    resetDefaults();
    writeMsg(PSTR(FW_IDENT));
    return RP_NONE;
  }
  if (s[0] == 'I' && s[1] == '\0') {
    writeMsg(PSTR(FW_IDENT));
    return RP_NONE;
  }

  // ---- multi-character names first: ATSH/ATSP/ATST all start with 'S' ----
  a = after(s, PSTR("SH"));
  if (a != NULL) {
    if (atParseId11(a, id) != AT_HEX_OK) {
      return RP_QUERY;
    }
    header_ = id;
    return RP_OK;
  }
  a = after(s, PSTR("ST"));
  if (a != NULL) {
    uint8_t v[1];
    uint8_t n = 0u;
    if (atParseHexBytes(a, v, 1u, n) != AT_HEX_OK || n != 1u) {
      return RP_QUERY;
    }
    stUnits_ = v[0];                     // ELM units of 4 ms; 00 = default
    return RP_OK;
  }
  a = after(s, PSTR("SP"));
  if (a != NULL) {
    // Protocol 6 (ISO 15765-4, 11-bit CAN) is what this bus speaks, and the
    // bit rate is compiled in via CAN_BITRATE_ID -- so 6 and A6 are accepted
    // and anything else is refused rather than silently ignored.
    if ((a[0] == '6' && a[1] == '\0') ||
        (a[0] == 'A' && a[1] == '6' && a[2] == '\0')) {
      return RP_OK;
    }
    return RP_QUERY;
  }

  a = after(s, PSTR("CAF"));
  if (a != NULL) {
    if (!onOff(a, flag)) {
      return RP_QUERY;
    }
    autoFormat_ = flag;                  // CAF0: the caller owns the PCI byte
    return RP_OK;
  }
  a = after(s, PSTR("CRA"));
  if (a != NULL) {
    if (a[0] == '\0') {                  // bare ATCRA clears the filter
      filter_ = 0u;
      mask_ = 0u;
      return RP_OK;
    }
    if (atParseId11(a, id) != AT_HEX_OK) {
      return RP_QUERY;
    }
    filter_ = id;
    mask_ = 0x7FFu;
    return RP_OK;   // software-side filter, nothing to program
  }
  a = after(s, PSTR("CF"));
  if (a != NULL) {
    if (atParseId11(a, id) != AT_HEX_OK) {
      return RP_QUERY;
    }
    filter_ = id;
    return RP_OK;   // software-side filter, nothing to program
  }
  a = after(s, PSTR("CM"));
  if (a != NULL) {
    if (atParseId11(a, id) != AT_HEX_OK) {
      return RP_QUERY;
    }
    mask_ = id;
    return RP_OK;   // software-side filter, nothing to program
  }

  // ---- flow control ----
  a = after(s, PSTR("FCSH"));
  if (a != NULL) {
    if (atParseId11(a, id) != AT_HEX_OK) {
      return RP_QUERY;
    }
    fc_.header = id;
    return RP_OK;
  }
  a = after(s, PSTR("FCSD"));
  if (a != NULL) {
    uint8_t buf[CAN_MAX_DLC];
    uint8_t n = 0u;
    if (atParseHexBytes(a, buf, CAN_MAX_DLC, n) != AT_HEX_OK || n == 0u) {
      return RP_QUERY;
    }
    for (uint8_t k = 0u; k < CAN_MAX_DLC; ++k) {
      fc_.data[k] = (k < n) ? buf[k] : 0x00u;
    }
    fc_.len = n;
    return RP_OK;
  }
  a = after(s, PSTR("FCSM"));
  if (a != NULL) {
    if (a[0] < '0' || a[0] > '2' || a[1] != '\0') {
      return RP_QUERY;
    }
    // Stored for the host's benefit; see the note in IsoTpFc.h on why the
    // adapter answers a First Frame in every mode.
    fc_.mode = static_cast<uint8_t>(a[0] - '0');
    return RP_OK;
  }

  // ---- protocol description ----
  a = after(s, PSTR("DPN"));
  if (a != NULL && a[0] == '\0') {
    writeMsg(PSTR("6"));                 // closest ELM number to 11-bit CAN
    return RP_NONE;
  }
  a = after(s, PSTR("DP"));
  if (a != NULL && a[0] == '\0') {
    writeMsg(PSTR("CAN-B 83.3K"));
    return RP_NONE;
  }
  a = after(s, PSTR("PB"));
  if (a != NULL) {
    // Programmable baud divisors: accepted and ignored, the bit timing comes
    // from Config.h. Still validated so typos are reported.
    uint8_t buf[2];
    uint8_t n = 0u;
    if (atParseHexBytes(a, buf, 2u, n) != AT_HEX_OK || n != 2u) {
      return RP_QUERY;
    }
    return RP_OK;
  }

  // ---- adaptive timing: accepted and ignored, one fixed ATST window ----
  a = after(s, PSTR("AT"));
  if (a != NULL) {
    if (a[0] < '0' || a[0] > '2' || a[1] != '\0') {
      return RP_QUERY;
    }
    return RP_OK;
  }

  // ---- monitor ----
  a = after(s, PSTR("MA"));
  if (a != NULL && a[0] == '\0') {
    // With no ATCF/ATCM/ATCRA set (the default) mask_ is 0 and every frame is
    // shown, so "monitor all" does mean all; with a filter set it is obeyed,
    // as documented -- clear it first to see everything. Matching happens in
    // frameAccepted(), so nothing is reprogrammed on the controller.
    state_ = ST_MONITOR;
    rxSeen_ = 0u;
    return RP_PENDING;
  }

  // ---- single-letter switches, checked last ----
  switch (s[0]) {
    case 'E':
      if (!onOff(s + 1, flag)) { return RP_QUERY; }
      echo_ = flag;
      return RP_OK;
    case 'L':
      if (!onOff(s + 1, flag)) { return RP_QUERY; }
      linefeed_ = flag;
      return RP_OK;
    case 'S':
      if (!onOff(s + 1, flag)) { return RP_QUERY; }
      spaces_ = flag;
      return RP_OK;
    case 'H':
      if (!onOff(s + 1, flag)) { return RP_QUERY; }
      headers_ = flag;
      return RP_OK;
    default:
      return RP_QUERY;
  }
}

AtServer::Reply AtServer::cmdCustom(const char* s) {
  const char* a;
  bool flag = false;

  if (s[0] == 'I' && s[1] == '\0') {
    writeP(PSTR(FW_NAME " "));
    writeMsg(PSTR(FW_VERSION));
    return RP_NONE;
  }
  a = after(s, PSTR("ST"));
  if (a != NULL && a[0] == '\0') {
    return statusLine();
  }
  a = after(s, PSTR("EV"));
  if (a != NULL) {
    if (!onOff(a, flag)) {
      return RP_QUERY;
    }
    events_ = flag;
    return RP_OK;
  }
  a = after(s, PSTR("BENCH"));
  if (a != NULL) {
    if (!onOff(a, flag)) {
      return RP_QUERY;
    }
    if (bench_ == NULL) {
      return RP_QUERY;                   // not wired, or compiled out
    }
    return bench_->setBenchMode(flag) ? RP_OK : RP_QUERY;
  }

  // AT@VIN            -> query: print the active VIN + "STORED"/"DEFAULT"
  // AT@VIN<chars>     -> store that VIN in EEPROM (used by bench mode)
  // AT@VIN-           -> remove the stored VIN, revert to the compiled default
  // Spaces are stripped by the line reader, and a VIN never starts with '-',
  // so a lone '-' is an unambiguous "remove".
  a = after(s, PSTR("VIN"));
  if (a != NULL) {
    if (bench_ == NULL) {
      return RP_QUERY;                   // not wired, or compiled out
    }
    if (a[0] == '\0') {
      char buf[VIN_MAX_LEN + 1u];
      uint8_t n = bench_->getVin(buf, (uint8_t)VIN_MAX_LEN);
      if (n == 0u) {
        // No VIN provisioned: bench mode sends power-on only, no VIN frames.
        writeMsg(PSTR("NONE"));
        return RP_NONE;
      }
      buf[n] = '\0';
      io_.writeBytes(buf);
      io_.writeByte(' ');
      writeMsg(bench_->vinStored() ? PSTR("STORED") : PSTR("DEFAULT"));
      return RP_NONE;
    }
    if (a[0] == '-' && a[1] == '\0') {
      return bench_->clearVin() ? RP_OK : RP_QUERY;
    }
    return bench_->setVin(a, (uint8_t)strlen(a)) ? RP_OK : RP_QUERY;
  }

  return RP_QUERY;
}

AtServer::Reply AtServer::statusLine() {
  char num[8];

  AtStatusInfo info;
  if (status_ != NULL) {
    status_->fillStatus(info);
  }
  if (bench_ != NULL) {
    info.benchMode = bench_->benchMode();
  }

  writeP(PSTR("FW=" FW_VERSION " MODE="));
  if (info.radioMode == 0xFFu) {
    writeP(PSTR("--"));
  } else {
    const uint8_t m = info.radioMode;
    (void)atFormatHexBytes(&m, 1u, num, sizeof(num), false);
    io_.writeBytes(num);
  }

  writeP(PSTR(" AUX="));
  io_.writeByte(info.auxActive ? '1' : '0');
  writeP(PSTR(" BT="));
  io_.writeByte(info.btPower ? '1' : '0');
  writeP(PSTR(" BENCH="));
  io_.writeByte(info.benchMode ? '1' : '0');

  writeP(PSTR(" CAN="));
  writeP(bus_.healthy() ? PSTR("OK") : PSTR("ERR"));

  writeP(PSTR(" RX="));
  (void)atFormatDec(bus_.rxCount(), num, sizeof(num));
  io_.writeBytes(num);
  writeP(PSTR(" TX="));
  (void)atFormatDec(bus_.txCount(), num, sizeof(num));
  io_.writeBytes(num);
  writeP(PSTR(" ER="));
  (void)atFormatDec(bus_.errCount(), num, sizeof(num));
  io_.writeBytes(num);

  // Addressing state, so a session can be reconstructed from one line.
  char idbuf[4];
  writeP(PSTR(" SH="));
  (void)atFormatId11(header_, idbuf, sizeof(idbuf));
  io_.writeBytes(idbuf);
  writeP(PSTR(" CF="));
  (void)atFormatId11(filter_, idbuf, sizeof(idbuf));
  io_.writeBytes(idbuf);
  writeP(PSTR(" CM="));
  (void)atFormatId11(mask_, idbuf, sizeof(idbuf));
  io_.writeBytes(idbuf);

  // Compiled-in feature letters: V = VES/AUX, T = BT control, B = bench.
  writeP(PSTR(" F="));
#if FEATURE_VES_AUX
  io_.writeByte('V');
#endif
#if FEATURE_BT_CONTROL
  io_.writeByte('T');
#endif
#if FEATURE_BENCH_MODE
  io_.writeByte('B');
#endif

  writeP(PSTR(" UP="));
  (void)atFormatDec(static_cast<uint16_t>(nowMs_ / 1000UL), num, sizeof(num));
  io_.writeBytes(num);
  writeP(PSTR(" RAM="));
  (void)atFormatDec(info.freeRam, num, sizeof(num));
  io_.writeBytes(num);

  // MCUSR as latched at boot. Bit 0 PORF power-on, 1 EXTRF external,
  // 2 BORF brown-out, 3 WDRF watchdog -- so RST=02 is the reset line, RST=08
  // is the watchdog biting, and RST=01 is a real power cycle.
  writeP(PSTR(" RST="));
  (void)atFormatHexBytes(&info.resetFlags, 1u, num, sizeof(num), false);
  io_.writeBytes(num);
  writeP(PSTR(" REI="));
  (void)atFormatDec(info.reinits, num, sizeof(num));
  io_.writeBytes(num);

  writeTerm();
  return RP_NONE;
}

AtServer::Reply AtServer::cmdData(const char* s) {
  uint8_t buf[CAN_MAX_DLC];
  uint8_t n = 0u;
  if (atParseHexBytes(s, buf, CAN_MAX_DLC, n) != AT_HEX_OK || n == 0u) {
    return RP_QUERY;                     // odd digits, junk, or over 8 bytes
  }

  CanFrame f;
  f.id = header_;
  if (autoFormat_) {
    // ATCAF1: like ELM, build the single-frame PCI byte and pad to DLC 8.
    if (n > CAN_MAX_DLC - 1u) {
      return RP_QUERY;
    }
    f.data[0] = n;
    for (uint8_t k = 0u; k < CAN_MAX_DLC - 1u; ++k) {
      f.data[k + 1u] = (k < n) ? buf[k] : 0x00u;
    }
    f.len = CAN_MAX_DLC;
  } else {
    // ATCAF0: exactly the bytes the host typed, nothing added, nothing
    // stripped. The host owns the PCI byte -- the point of this adapter.
    for (uint8_t k = 0u; k < CAN_MAX_DLC; ++k) {
      f.data[k] = (k < n) ? buf[k] : 0x00u;
    }
    f.len = n;
  }

  if (bus_.send(f) != CAN_ST_OK) {
    return RP_CAN_ERROR;
  }

  state_ = ST_COLLECT;
  deadline_ = nowMs_ + timeoutMs();
  rxSeen_ = 0u;
  mfTotal_ = 0u;
  mfGot_ = 0u;
  return RP_PENDING;
}

// ---- events -------------------------------------------------------------

void AtServer::emit(EventCode code, uint16_t value) {
  if (!events_) {
    return;                              // AT@EV0 (the default): stay quiet
  }

  // One PROGMEM tag per code, as a switch rather than a pointer table so no
  // array of RAM-resident pointers is generated.
  PGM_P tag;
  switch (code) {
    case EV_RADIO_MODE: tag = PSTR("RADIO_MODE"); break;
    case EV_AUX_STATE:  tag = PSTR("AUX");        break;
    case EV_BT_POWER:   tag = PSTR("BTPWR");      break;
    case EV_BUTTON:     tag = PSTR("BUTTON");     break;
    case EV_BT_COMMAND: tag = PSTR("BTCMD");      break;
    case EV_CAN_STATE:  tag = PSTR("CAN");        break;
    case EV_BENCH_MODE: tag = PSTR("BENCH");      break;
    case EV_SAFE_STATE: tag = PSTR("SAFE");       break;
    case EV_NONE:
    default:
      return;                            // unknown code: nothing to say
  }

  // "AT@EV <TAG> <value>" -- the prefix a host parser keys on to tell an
  // asynchronous event apart from a command response. Values are decimal for
  // every code, including the EV_BUTTON bitmask.
  char num[8];
  writeP(PSTR("AT@EV "));
  writeP(tag);
  io_.writeByte(' ');
  (void)atFormatDec(value, num, sizeof(num));
  io_.writeBytes(num);
  writeTerm();
}

#else   // UNIT_TEST

// The host test build compiles only the pure translation units (AtLine,
// IsoTpFc). This keeps the file from being an empty translation unit.
typedef int AtServerNotBuiltOnHost;

#endif  // UNIT_TEST
