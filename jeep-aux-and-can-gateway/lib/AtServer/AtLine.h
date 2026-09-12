#pragma once
// Line accumulation and hex conversion for the AT link.
//
// PURE: only <stdint.h>/<stddef.h> and the dependency-free Core/Config.h, so
// this translation unit compiles on the host and test/test_at_line can run it.
// Nothing here allocates, nothing here does I/O.
//
// RAM: one AtLine is AT_LINE_MAX + 1 + 2 = 51 bytes. It is the single largest
// buffer the AT layer owns; see the budget note in AtServer.h.
#include <stdint.h>
#include <stddef.h>

#include "Config.h"   // AT_LINE_MAX

// ---- Line accumulator ----------------------------------------------------

enum AtLineStatus : uint8_t {
  AT_LINE_PENDING = 0,   // no terminator seen yet, keep feeding
  AT_LINE_READY,         // complete non-empty line available in text()
  AT_LINE_EMPTY,         // terminator with nothing before it -- ignore it
  AT_LINE_OVERFLOW,      // line exceeded AT_LINE_MAX; DISCARDED, not truncated
};

// ELM327 input normalisation is done as characters arrive: spaces and tabs are
// dropped, lowercase is folded to uppercase, other control characters are
// ignored. So "at sh 7e0" and "ATSH7E0" reach the dispatcher identically.
class AtLine {
 public:
  AtLine();

  // Feed one received character. Returns what the line became. After a READY,
  // EMPTY or OVERFLOW result the buffer is recycled automatically on the next
  // push(), so text() stays valid until then.
  AtLineStatus push(char c);

  void reset();

  const char* text() const { return buf_; }
  uint8_t     length() const { return len_; }

  // True while a line is half-typed: characters accumulated (or being
  // discarded after an overflow) but no terminator yet. A line that has
  // already been reported is NOT partial, even though text() still holds it.
  // AtServer::idle() uses this.
  bool partial() const { return !done_ && (len_ != 0u || overflow_); }

 private:
  char    buf_[AT_LINE_MAX + 1u];
  uint8_t len_;
  bool    overflow_;
  bool    done_;
};

// ---- Hex helpers ---------------------------------------------------------

enum AtHexStatus : uint8_t {
  AT_HEX_OK = 0,
  AT_HEX_EMPTY,        // no digits at all
  AT_HEX_BAD_CHAR,     // something that is not 0-9A-F
  AT_HEX_ODD_DIGITS,   // byte strings must be whole bytes
  AT_HEX_TOO_LONG,     // more bytes than the caller's buffer holds
  AT_HEX_RANGE,        // parsed but out of range (e.g. id > 0x7FF)
};

// Nibble value of an already-normalised (uppercase) hex digit, -1 otherwise.
int8_t atHexVal(char c);

// True when s is non-empty and made only of hex digits.
bool atIsHexString(const char* s);

// "0221 30" is already space-stripped by AtLine, so this expects "022130".
// On success outLen holds the number of bytes written to out.
AtHexStatus atParseHexBytes(const char* s, uint8_t* out, uint8_t cap, uint8_t& outLen);

// 1..3 hex digits -> 11-bit identifier. 3 digits can express 0xFFF, so values
// above 0x7FF are rejected rather than silently masked.
AtHexStatus atParseId11(const char* s, uint16_t& outId);

// Uppercase hex out. Returns the number of characters written, 0 when cap is
// too small; out is always NUL-terminated when cap >= 1.
uint8_t atFormatHexBytes(const uint8_t* in, uint8_t len, char* out, uint8_t cap, bool spaces);

// Always exactly 3 nibbles, ELM style. Returns characters written.
uint8_t atFormatId11(uint16_t id, char* out, uint8_t cap);

// Unsigned decimal, no padding. Returns characters written.
uint8_t atFormatDec(uint16_t v, char* out, uint8_t cap);
