// Raw response bytes -> a displayable value.
//
// PURE: no ESP-IDF, no Arduino. <stdint.h> / <string.h> and project headers
// only. Static allocation, bounded loops, `default:` on every switch.
#include "Decoder.h"

#include <stdint.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Bit conventions -- fixed by SignalDb.h, established from real captures.
//
//   * bitOffset counts from bit 0 of byte 0 of the FULL response, so the
//     service byte and the local-identifier echo are included in the count.
//   * bits within a byte are numbered LSB-first: bit k of a byte has weight
//     1 << k. A run of `m` bits starting at bit k of one byte is therefore
//     just (byte >> k) & ((1 << m) - 1).
//   * bytes are big-endian: when a field spans bytes, the chunk taken from the
//     LOWER byte index is the MORE significant part of the result.
//
// Walk: start at (bitOffset / 8, bitOffset % 8), take bits with increasing bit
// number until the byte is exhausted, then move to the next byte starting at
// bit 0; shift each new chunk in from the right.
//
// Worked example -- bitOffset = 20, bitLength = 8, response = 61 21 A5 3C ...
//   step 1: byte 2 = 0xA5 = 1010_0101, bitInByte = 4, take = 4
//           chunk = (0xA5 >> 4) & 0xF = 0xA          <- bits 4..7 of byte 2
//           result = 0xA
//   step 2: byte 3 = 0x3C = 0011_1100, bitInByte = 0, take = 4
//           chunk = 0x3C & 0xF = 0xC                 <- bits 0..3 of byte 3
//           result = (0xA << 4) | 0xC = 0xAC = 172
// So the field high nibble is the HIGH nibble of byte 2 and its low nibble is
// the LOW nibble of byte 3.
//
// Sanity check of the byte-aligned case: bitOffset = 16, bitLength = 16 gives
// (data[2] << 8) | data[3] = 0xA53C -- big-endian, as required.
// ---------------------------------------------------------------------------

uint32_t extractBits(const uint8_t* data, uint16_t len,
                     uint16_t bitOffset, uint8_t bitLength, bool& ok) {
  ok = false;

  // Never trust the caller or the module. A NULL buffer, a zero-length
  // response, a nonsense field width and a field that runs off the end of a
  // short reply all degrade to "unknown" rather than reading adjacent memory.
  if (data == 0) return 0;
  if (bitLength == 0u || bitLength > 32u) return 0;

  const uint32_t firstBit = (uint32_t)bitOffset;
  const uint32_t lastBit = firstBit + (uint32_t)bitLength - 1u;  // 32-bit math
  if (lastBit >= (uint32_t)len * 8u) return 0;                   // covers len == 0

  uint32_t result = 0;
  uint32_t bit = firstBit;
  uint8_t remaining = bitLength;

  // Bounded: bitLength <= 32 means at most 5 iterations.
  while (remaining > 0u) {
    const uint32_t byteIdx = bit >> 3;
    const uint8_t bitInByte = (uint8_t)(bit & 7u);
    uint8_t take = (uint8_t)(8u - bitInByte);
    if (take > remaining) take = remaining;

    const uint8_t mask = (uint8_t)((take >= 8u) ? 0xFFu : ((1u << take) - 1u));
    const uint8_t chunk = (uint8_t)((data[byteIdx] >> bitInByte) & mask);

    // Big-endian across bytes: earlier bytes end up more significant.
    result = (result << take) | (uint32_t)chunk;

    remaining = (uint8_t)(remaining - take);
    bit += take;
  }

  ok = true;
  return result;
}

// ---------------------------------------------------------------------------
// Enum lookup
// ---------------------------------------------------------------------------

// Table-explicit form. Exposed (but deliberately NOT declared in Decoder.h,
// which is a fixed contract) so host suites can exercise the search against a
// table they own: the generated kEnums lives in SignalDb.cpp and cannot be
// redefined in a test without a duplicate symbol at link time.
//
// FIRST MATCH WINS. Ranges in the vendor source data do overlap, and there is
// no tie-break rule that recovers the intended label, so the earliest matching
// row in table order is authoritative. The generator must therefore emit rows
// in the order it wants them consulted.
const char* decoderEnumLabelIn(const EnumEntry* table, uint16_t count,
                               int16_t group, uint32_t raw) {
  if (table == 0 || group < 0) return 0;
  const uint16_t want = (uint16_t)group;
  for (uint16_t i = 0; i < count; ++i) {
    if (table[i].group != want) continue;
    if (raw >= table[i].from && raw <= table[i].to) return table[i].label;
  }
  return 0;
}

const char* enumLabel(int16_t group, uint32_t raw) {
  return decoderEnumLabelIn(kEnums, kEnumCount, group, raw);
}

// ---------------------------------------------------------------------------
// decodeSignal
// ---------------------------------------------------------------------------

static void decoderClear(DecodedValue& out) {
  out.valid = false;
  out.raw = 0;
  out.value = 0.0f;
  out.unit = "";  // contract: never NULL
  out.decimals = 0;
  out.label = 0;
  memset(out.text, 0, sizeof(out.text));
}

static DecodeStatus decoderExtract(const SignalDef& def, const uint8_t* response,
                                   uint16_t len, uint32_t& raw) {
  raw = 0;
  // Distinguished from OUT_OF_RANGE on purpose: a table row asking for 0 or
  // more than 32 bits is a generator bug, not a short reply from the module.
  if (def.bitLength == 0u || def.bitLength > 32u) return DECODE_BAD_PARAM;
  bool ok = false;
  raw = extractBits(response, len, def.bitOffset, def.bitLength, ok);
  return ok ? DECODE_OK : DECODE_OUT_OF_RANGE;
}

// RENDER_ASCII. Byte-aligned only: an unaligned text field has no sensible
// meaning and is reported as a table bug rather than silently shifted.
static DecodeStatus decoderAscii(const SignalDef& def, const uint8_t* response,
                                 uint16_t len, DecodedValue& out) {
  if ((def.bitOffset & 7u) != 0u || (def.bitLength & 7u) != 0u) return DECODE_BAD_PARAM;

  const uint32_t start = (uint32_t)def.bitOffset >> 3;
  const uint32_t nbytes = (uint32_t)def.bitLength >> 3;
  if (nbytes == 0u) return DECODE_BAD_PARAM;
  if (start + nbytes > (uint32_t)len) return DECODE_OUT_OF_RANGE;

  char tmp[DECODED_TEXT_MAX];
  memset(tmp, 0, sizeof(tmp));
  uint8_t n = 0;

  for (uint32_t i = 0; i < nbytes; ++i) {
    const uint8_t c = response[start + i];
    // 0x00 and 0xFF are how these modules pad a fixed-width text field; both
    // terminate the string rather than being rejected.
    if (c == 0x00u || c == 0xFFu) break;
    // Anything else outside printable ASCII means this is not text at all --
    // a wrong offset, or a corrupted reply. Reject it: "unknown" beats
    // rendering mojibake as a part number.
    if (c < 0x20u || c > 0x7Eu) return DECODE_BAD_PARAM;
    // Truncate at DECODED_TEXT_MAX - 1 characters, leaving room for the NUL.
    // A 17-byte VIN fits in the 24-byte buffer with room to spare.
    if (n < (uint8_t)(DECODED_TEXT_MAX - 1u)) {
      tmp[n] = (char)c;
      ++n;
    }
  }
  tmp[n] = '\0';

  // Strip trailing spaces (these fields are space-padded as often as they are
  // NUL-padded). An all-pad field decodes as a valid empty string: we read it
  // fine, it is simply blank -- which is not the same as failing to read it.
  while (n > 0u && tmp[n - 1u] == ' ') {
    --n;
    tmp[n] = '\0';
  }

  memcpy(out.text, tmp, sizeof(out.text));
  out.valid = true;
  return DECODE_OK;
}

// Table-explicit form; see the comment on decoderEnumLabelIn for why this
// seam exists. Not declared in Decoder.h.
DecodeStatus decodeSignalTbl(const SignalDef& def, const uint8_t* response,
                             uint16_t len,
                             const ScalingDef* scalings, uint8_t scalingCount,
                             const EnumEntry* enums, uint16_t enumCount,
                             DecodedValue& out) {
  decoderClear(out);
  if (response == 0) return DECODE_BAD_PARAM;

  switch (def.render) {
    case RENDER_ASCII:
      return decoderAscii(def, response, len, out);

    case RENDER_SCALED: {
      uint32_t raw = 0;
      const DecodeStatus st = decoderExtract(def, response, len, raw);
      if (st != DECODE_OK) return st;
      out.raw = raw;
      if (def.scaling < 0) {
        // No scaling row: pass the raw value through, unitless.
        out.value = (float)raw;
        out.valid = true;
        return DECODE_OK;
      }
      if (scalings == 0 || (uint8_t)def.scaling >= scalingCount) {
        out.value = (float)raw;
        return DECODE_BAD_PARAM;  // dangling scaling index = generator bug
      }
      const ScalingDef& sc = scalings[(uint8_t)def.scaling];
      out.value = (float)raw * sc.factor + sc.offset;
      out.unit = (sc.unit != 0) ? sc.unit : "";
      out.decimals = sc.decimals;
      out.valid = true;
      return DECODE_OK;
    }

    case RENDER_ENUM: {
      uint32_t raw = 0;
      const DecodeStatus st = decoderExtract(def, response, len, raw);
      if (st != DECODE_OK) return st;
      out.raw = raw;
      out.value = (float)raw;
      const char* lbl = decoderEnumLabelIn(enums, enumCount, def.enumGroup, raw);
      if (lbl == 0) {
        // raw is kept so a diagnostics screen can show the unmatched value,
        // but valid stays false: no label is better than a misleading one.
        return DECODE_NO_ENUM_MATCH;
      }
      out.label = lbl;
      out.valid = true;
      return DECODE_OK;
    }

    case RENDER_RAW: {
      uint32_t raw = 0;
      const DecodeStatus st = decoderExtract(def, response, len, raw);
      if (st != DECODE_OK) return st;
      out.raw = raw;
      out.value = (float)raw;
      out.valid = true;
      return DECODE_OK;
    }

    default:
      return DECODE_BAD_PARAM;
  }
}

DecodeStatus decodeSignal(const SignalDef& def, const uint8_t* response,
                          uint16_t len, DecodedValue& out) {
  return decodeSignalTbl(def, response, len,
                         kScalings, kScalingCount, kEnums, kEnumCount, out);
}

DecodeStatus decodeSignal(const SignalDef& def, const IsoTpPdu& pdu,
                          DecodedValue& out) {
  // Clamp a bogus length rather than trusting the PDU own length field.
  const uint16_t len = (pdu.len > ISOTP_MAX_PDU) ? (uint16_t)ISOTP_MAX_PDU : pdu.len;
  return decodeSignal(def, pdu.data, len, out);
}
