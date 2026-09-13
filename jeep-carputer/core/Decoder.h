#pragma once
// Raw response bytes -> a displayable value, using SignalDb's tables.
//
// PURE: no ESP-IDF, no Arduino, no floating-point formatting.
#include <stdint.h>

#include "IsoTp.h"
#include "SignalDb.h"

#define DECODED_TEXT_MAX 24u

struct DecodedValue {
  bool        valid;
  uint32_t    raw;
  float       value;                    // raw * factor + offset for RENDER_SCALED
  const char* unit;                     // never NULL, "" when unitless
  uint8_t     decimals;
  const char* label;                    // enum label, NULL when not an enum
  char        text[DECODED_TEXT_MAX];   // RENDER_ASCII payload, NUL-terminated
};

enum DecodeStatus : uint8_t {
  DECODE_OK = 0,
  DECODE_OUT_OF_RANGE,   // the field lies past the end of the response
  DECODE_NO_ENUM_MATCH,  // raw value not covered by its enum group
  DECODE_BAD_PARAM,
};

// Extract an unsigned field. Big-endian bytes, LSB-first bits, offset counted
// from byte 0 of the whole response. Sets ok=false when the field would read
// past `len` -- never reads out of bounds, and never trusts a module to have
// sent as much as the table expects.
uint32_t extractBits(const uint8_t* data, uint16_t len,
                     uint16_t bitOffset, uint8_t bitLength, bool& ok);

DecodeStatus decodeSignal(const SignalDef& def, const uint8_t* response,
                          uint16_t len, DecodedValue& out);

// Convenience over a reassembled PDU.
DecodeStatus decodeSignal(const SignalDef& def, const IsoTpPdu& pdu,
                          DecodedValue& out);

// Enum label for a raw value in a group; NULL when nothing matches.
const char* enumLabel(int16_t group, uint32_t raw);
