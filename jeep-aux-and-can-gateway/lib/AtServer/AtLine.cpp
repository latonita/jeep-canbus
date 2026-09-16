// PURE implementation -- see AtLine.h. No Arduino, no AVR, no allocation.
#include "AtLine.h"

// ---- Line accumulator ----------------------------------------------------

AtLine::AtLine() {
  reset();
}

void AtLine::reset() {
  len_ = 0u;
  overflow_ = false;
  done_ = false;
  buf_[0] = '\0';
}

AtLineStatus AtLine::push(char c) {
  // The previous line has been reported; recycle the buffer now so text()
  // stayed valid for the caller right up to this point.
  if (done_) {
    reset();
  }

  if (c == '\r' || c == '\n') {
    if (overflow_) {
      // Discard rather than execute half a command: a truncated "ATSH7E" is a
      // different, possibly harmful, command from what the host typed.
      reset();
      done_ = true;
      return AT_LINE_OVERFLOW;
    }
    if (len_ == 0u) {
      // Second half of a CRLF pair, or a bare Enter. ELM would repeat the last
      // command here; we ignore it (see AtServer notes).
      done_ = true;
      return AT_LINE_EMPTY;
    }
    buf_[len_] = '\0';
    done_ = true;
    return AT_LINE_READY;
  }

  // ELM327 normalisation.
  if (c == ' ' || c == '\t') {
    return AT_LINE_PENDING;
  }
  if (c >= 'a' && c <= 'z') {
    c = static_cast<char>(c - 'a' + 'A');
  }
  if (static_cast<unsigned char>(c) < 0x20u || static_cast<unsigned char>(c) == 0x7Fu) {
    return AT_LINE_PENDING;   // stray control byte
  }

  if (len_ >= AT_LINE_MAX) {
    overflow_ = true;         // keep eating until the terminator
    return AT_LINE_PENDING;
  }
  buf_[len_++] = c;
  buf_[len_] = '\0';
  return AT_LINE_PENDING;
}

// ---- Hex helpers ---------------------------------------------------------

int8_t atHexVal(char c) {
  if (c >= '0' && c <= '9') {
    return static_cast<int8_t>(c - '0');
  }
  if (c >= 'A' && c <= 'F') {
    return static_cast<int8_t>(c - 'A' + 10);
  }
  if (c >= 'a' && c <= 'f') {
    return static_cast<int8_t>(c - 'a' + 10);
  }
  return -1;
}

bool atIsHexString(const char* s) {
  if (s == NULL || s[0] == '\0') {
    return false;
  }
  for (const char* p = s; *p != '\0'; ++p) {
    if (atHexVal(*p) < 0) {
      return false;
    }
  }
  return true;
}

AtHexStatus atParseHexBytes(const char* s, uint8_t* out, uint8_t cap, uint8_t& outLen) {
  outLen = 0u;
  if (s == NULL || out == NULL || cap == 0u) {
    return AT_HEX_TOO_LONG;
  }
  if (s[0] == '\0') {
    return AT_HEX_EMPTY;
  }

  uint8_t digits = 0u;
  uint8_t acc = 0u;
  // Bounded by the NUL; AT_LINE_MAX already caps the input length.
  for (const char* p = s; *p != '\0'; ++p) {
    const int8_t v = atHexVal(*p);
    if (v < 0) {
      outLen = 0u;
      return AT_HEX_BAD_CHAR;
    }
    acc = static_cast<uint8_t>((acc << 4) | static_cast<uint8_t>(v));
    ++digits;
    if ((digits & 1u) == 0u) {
      if (outLen >= cap) {
        outLen = 0u;
        return AT_HEX_TOO_LONG;
      }
      out[outLen++] = acc;
      acc = 0u;
    }
  }

  if ((digits & 1u) != 0u) {
    outLen = 0u;
    return AT_HEX_ODD_DIGITS;
  }
  return AT_HEX_OK;
}

AtHexStatus atParseId11(const char* s, uint16_t& outId) {
  outId = 0u;
  if (s == NULL || s[0] == '\0') {
    return AT_HEX_EMPTY;
  }

  uint16_t acc = 0u;
  uint8_t digits = 0u;
  for (const char* p = s; *p != '\0'; ++p) {
    const int8_t v = atHexVal(*p);
    if (v < 0) {
      return AT_HEX_BAD_CHAR;
    }
    if (digits >= 3u) {
      return AT_HEX_RANGE;    // more than 3 nibbles is not an 11-bit id
    }
    acc = static_cast<uint16_t>((acc << 4) | static_cast<uint16_t>(v));
    ++digits;
  }

  if (acc > 0x7FFu) {
    return AT_HEX_RANGE;
  }
  outId = acc;
  return AT_HEX_OK;
}

static char hexDigit(uint8_t nib) {
  return static_cast<char>(nib < 10u ? ('0' + nib) : ('A' + (nib - 10u)));
}

uint8_t atFormatHexBytes(const uint8_t* in, uint8_t len, char* out, uint8_t cap, bool spaces) {
  if (out == NULL || cap == 0u) {
    return 0u;
  }
  out[0] = '\0';
  if (in == NULL || len == 0u) {
    return 0u;
  }

  // 2 chars per byte, plus one separator between bytes, plus the NUL.
  const uint16_t need = static_cast<uint16_t>(len) * (spaces ? 3u : 2u) + 1u;
  if (need > cap) {
    return 0u;
  }

  uint8_t n = 0u;
  for (uint8_t i = 0u; i < len; ++i) {
    if (spaces && i != 0u) {
      out[n++] = ' ';
    }
    out[n++] = hexDigit(static_cast<uint8_t>(in[i] >> 4));
    out[n++] = hexDigit(static_cast<uint8_t>(in[i] & 0x0Fu));
  }
  out[n] = '\0';
  return n;
}

uint8_t atFormatId11(uint16_t id, char* out, uint8_t cap) {
  if (out == NULL || cap < 4u) {
    if (out != NULL && cap > 0u) {
      out[0] = '\0';
    }
    return 0u;
  }
  out[0] = hexDigit(static_cast<uint8_t>((id >> 8) & 0x07u));
  out[1] = hexDigit(static_cast<uint8_t>((id >> 4) & 0x0Fu));
  out[2] = hexDigit(static_cast<uint8_t>(id & 0x0Fu));
  out[3] = '\0';
  return 3u;
}

uint8_t atFormatHexFixed(uint32_t v, uint8_t digits, char* out, uint8_t cap) {
  if (out == NULL || cap == 0u) {
    return 0u;
  }
  out[0] = '\0';
  // digits + NUL must fit. 8 nibbles is the whole of a uint32_t; anything
  // wider would be a caller bug silently producing leading zeroes forever.
  if (digits == 0u || digits > 8u || static_cast<uint8_t>(digits + 1u) > cap) {
    return 0u;
  }
  for (uint8_t i = 0u; i < digits; ++i) {
    const uint8_t shift = static_cast<uint8_t>((digits - 1u - i) * 4u);
    out[i] = hexDigit(static_cast<uint8_t>((v >> shift) & 0x0Fu));
  }
  out[digits] = '\0';
  return digits;
}

uint8_t atFormatDec(uint16_t v, char* out, uint8_t cap) {
  if (out == NULL || cap < 2u) {
    if (out != NULL && cap > 0u) {
      out[0] = '\0';
    }
    return 0u;
  }
  char tmp[6];               // 65535 -> 5 digits
  uint8_t n = 0u;
  do {
    tmp[n++] = static_cast<char>('0' + (v % 10u));
    v = static_cast<uint16_t>(v / 10u);
  } while (v != 0u && n < sizeof(tmp));

  if (static_cast<uint8_t>(n + 1u) > cap) {
    out[0] = '\0';
    return 0u;
  }
  for (uint8_t i = 0u; i < n; ++i) {
    out[i] = tmp[n - 1u - i];
  }
  out[n] = '\0';
  return n;
}
