// Host tests for core/Decoder.cpp.
//
// All SignalDef / ScalingDef / EnumEntry instances used here are LOCAL to this
// file. The generated tables in core/SignalDb.cpp are owned by another part of
// the project and are deliberately not exercised: this suite must pass no
// matter what ends up in them.
//
// decodeSignal() and enumLabel() reach the generated tables by design (their
// signatures carry only an index / a group id), so the arithmetic and the
// range search are tested through the table-explicit seams in Decoder.cpp --
// decodeSignalTbl() and decoderEnumLabelIn() -- of which the two contract
// functions are one-line wrappers.
#include "doctest_min.h"

#include "Decoder.h"

// Seams in core/Decoder.cpp. Not in Decoder.h, which is a fixed contract.
const char* decoderEnumLabelIn(const EnumEntry* table, uint16_t count,
                               int16_t group, uint32_t raw);
DecodeStatus decodeSignalTbl(const SignalDef& def, const uint8_t* response,
                             uint16_t len,
                             const ScalingDef* scalings, uint8_t scalingCount,
                             const EnumEntry* enums, uint16_t enumCount,
                             DecodedValue& out);

// ---------------------------------------------------------------------------
// Local fixtures
// ---------------------------------------------------------------------------

// The canonical response used by the bit tests. 8 bytes = 64 bits.
//   byte 0 = 0x61 = 0110_0001
//   byte 1 = 0x21 = 0010_0001
//   byte 2 = 0xA5 = 1010_0101
//   byte 3 = 0x3C = 0011_1100
//   byte 4 = 0x0F   byte 5 = 0xF0   byte 6 = 0x12   byte 7 = 0x34
static const uint8_t kResp[8] = {0x61, 0x21, 0xA5, 0x3C, 0x0F, 0xF0, 0x12, 0x34};

static const ScalingDef kTestScalings[] = {
    {0.25f, 0.0f, "rpm", 0},   // 0
    {0.5f, -40.0f, "C", 1},    // 1
    {1.0f, 0.0f, "", 0},       // 2
};
static const uint8_t kTestScalingCount = 3;

static const EnumEntry kTestEnums[] = {
    {7, 0, 0, "Off"},
    {7, 1, 3, "Low"},
    {7, 2, 9, "MedOverlap"},   // overlaps [1,3]: 2 and 3 must resolve to "Low"
    {7, 10, 10, "Ten"},
    {8, 0, 255, "Group8Any"},
    {9, 5, 1, "Malformed"},    // from > to: matches nothing
};
static const uint16_t kTestEnumCount = 6;

static SignalDef mkDef(uint16_t bitOffset, uint8_t bitLength, uint8_t render,
                       int8_t scaling, int16_t enumGroup) {
  SignalDef d;
  d.ecu = (uint8_t)ECU_CCN;
  d.localId = 0x213E;
  d.bitOffset = bitOffset;
  d.bitLength = bitLength;
  d.render = render;
  d.scaling = scaling;
  d.enumGroup = enumGroup;
  d.name = "test.signal";
  return d;
}

static DecodeStatus dec(const SignalDef& d, const uint8_t* r, uint16_t len,
                        DecodedValue& out) {
  return decodeSignalTbl(d, r, len, kTestScalings, kTestScalingCount,
                         kTestEnums, kTestEnumCount, out);
}

// extractBits with the ok flag folded into the return for terser assertions.
static uint32_t xb(const uint8_t* d, uint16_t len, uint16_t off, uint8_t bits,
                   bool& ok) {
  return extractBits(d, len, off, bits, ok);
}

// Build "<hdr0> <hdr1> <text bytes>" into a caller-supplied buffer.
static uint16_t mkTextResp(uint8_t* buf, uint16_t cap, const char* body,
                           uint16_t bodyLen) {
  uint16_t n = 0;
  if (cap < 2u) return 0;
  buf[n++] = 0x61;
  buf[n++] = 0x90;
  for (uint16_t i = 0; i < bodyLen && n < cap; ++i) {
    buf[n++] = (uint8_t)body[i];
  }
  return n;
}

TEST_MAIN_BEGIN()

  bool ok = false;

  // -------------------------------------------------------------------------
  // extractBits: offset 0
  // -------------------------------------------------------------------------
  // byte 0 = 0x61 = 0110_0001, bits numbered LSB-first.
  CHECK_EQ(xb(kResp, 8, 0, 1, ok), 1);   CHECK(ok);   // bit 0 of 0x61
  CHECK_EQ(xb(kResp, 8, 1, 1, ok), 0);   CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 5, 1, ok), 1);   CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 6, 1, ok), 1);   CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 7, 1, ok), 0);   CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 0, 8, ok), 0x61); CHECK(ok);

  // -------------------------------------------------------------------------
  // extractBits: crossing a byte boundary
  // -------------------------------------------------------------------------
  // 4 bits at offset 6: byte 0 bits 6..7 = (0x61 >> 6) & 3 = 0b01, then byte 1
  // bits 0..1 = 0x21 & 3 = 0b01. Bytes are big-endian, so byte 0 contributes
  // the high half: (0b01 << 2) | 0b01 = 5.
  CHECK_EQ(xb(kResp, 8, 6, 4, ok), 5);   CHECK(ok);

  // 8 bits at offset 20 -- the worked example in Decoder.cpp.
  // byte 2 = 0xA5, bits 4..7 -> 0xA (high nibble of the value)
  // byte 3 = 0x3C, bits 0..3 -> 0xC (low nibble of the value)
  CHECK_EQ(xb(kResp, 8, 20, 8, ok), 0xAC); CHECK(ok);

  // 8 bits at offset 12: byte 1 bits 4..7 = 0x2, byte 2 bits 0..3 = 0x5.
  CHECK_EQ(xb(kResp, 8, 12, 8, ok), 0x25); CHECK(ok);

  // 12 bits at offset 12: byte 1 bits 4..7 = 0x2, then all of byte 2 = 0xA5.
  CHECK_EQ(xb(kResp, 8, 12, 12, ok), 0x2A5); CHECK(ok);

  // -------------------------------------------------------------------------
  // extractBits: 16- and 32-bit fields are big-endian
  // -------------------------------------------------------------------------
  CHECK_EQ(xb(kResp, 8, 16, 8, ok), 0xA5);        CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 16, 16, ok), 0xA53C);     CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 48, 16, ok), 0x1234);     CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 0, 32, ok), 0x6121A53Cu); CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 32, 32, ok), 0x0FF01234u); CHECK(ok);
  // Unaligned 16-bit: byte 2 bits 4..7 = 0xA, byte 3 = 0x3C, byte 4 bits 0..3
  // = 0xF  ->  0xA_3C_F.
  CHECK_EQ(xb(kResp, 8, 20, 16, ok), 0xA3CFu);    CHECK(ok);

  // -------------------------------------------------------------------------
  // extractBits: the field whose last bit is the response's last bit
  // -------------------------------------------------------------------------
  CHECK_EQ(xb(kResp, 8, 63, 1, ok), 0);        CHECK(ok);  // bit 7 of 0x34
  CHECK_EQ(xb(kResp, 8, 62, 2, ok), 0);        CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 60, 4, ok), 0x3);      CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 56, 8, ok), 0x34);     CHECK(ok);
  CHECK_EQ(xb(kResp, 8, 0, 8, ok), 0x61);      CHECK(ok);

  // -------------------------------------------------------------------------
  // extractBits: NEVER reads out of bounds
  // -------------------------------------------------------------------------
  // One bit past the end.
  ok = true; CHECK_EQ(xb(kResp, 8, 64, 1, ok), 0); CHECK(!ok);
  // Field starts inside but ends one bit past the end.
  ok = true; CHECK_EQ(xb(kResp, 8, 57, 8, ok), 0); CHECK(!ok);
  ok = true; CHECK_EQ(xb(kResp, 8, 56, 9, ok), 0); CHECK(!ok);
  // A module answered shorter than the table expects: 16-bit field at byte 2
  // of a 3-byte reply.
  ok = true; CHECK_EQ(xb(kResp, 3, 16, 16, ok), 0); CHECK(!ok);
  // ...but the same field in a 4-byte reply is fine.
  ok = false; CHECK_EQ(xb(kResp, 4, 16, 16, ok), 0xA53C); CHECK(ok);
  // Empty response.
  ok = true; CHECK_EQ(xb(kResp, 0, 0, 1, ok), 0); CHECK(!ok);
  ok = true; CHECK_EQ(xb(kResp, 0, 0, 8, ok), 0); CHECK(!ok);
  // NULL pointer.
  ok = true; CHECK_EQ(xb(0, 8, 0, 8, ok), 0); CHECK(!ok);
  ok = true; CHECK_EQ(xb(0, 0, 0, 1, ok), 0); CHECK(!ok);
  // Nonsense widths.
  ok = true; CHECK_EQ(xb(kResp, 8, 0, 0, ok), 0);  CHECK(!ok);
  ok = true; CHECK_EQ(xb(kResp, 8, 0, 33, ok), 0); CHECK(!ok);
  // bitOffset near the top of its uint16_t range must not overflow into a
  // false "in bounds".
  ok = true; CHECK_EQ(xb(kResp, 8, 65530, 8, ok), 0); CHECK(!ok);

  // -------------------------------------------------------------------------
  // decoderEnumLabelIn: first match wins, overlaps and all
  // -------------------------------------------------------------------------
  CHECK_STR(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 7, 0), "Off");
  CHECK_STR(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 7, 1), "Low");
  CHECK_STR(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 7, 2), "Low");
  CHECK_STR(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 7, 3), "Low");
  CHECK_STR(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 7, 4), "MedOverlap");
  CHECK_STR(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 7, 9), "MedOverlap");
  CHECK_STR(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 7, 10), "Ten");
  CHECK(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 7, 11) == 0);
  CHECK_STR(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 8, 255), "Group8Any");
  CHECK(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 8, 256) == 0);
  CHECK(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 6, 0) == 0);   // no group
  CHECK(decoderEnumLabelIn(kTestEnums, kTestEnumCount, 9, 5) == 0);   // from > to
  CHECK(decoderEnumLabelIn(kTestEnums, kTestEnumCount, -1, 0) == 0);  // no enum
  CHECK(decoderEnumLabelIn(kTestEnums, 0, 7, 0) == 0);                // empty
  CHECK(decoderEnumLabelIn(0, kTestEnumCount, 7, 0) == 0);            // NULL

  // enumLabel() reaches the generated kEnums, whose contents this suite must
  // not assume. Only the content-independent case is asserted.
  CHECK(enumLabel(-1, 0) == 0);

  // -------------------------------------------------------------------------
  // decodeSignal: RENDER_SCALED
  // -------------------------------------------------------------------------
  {
    DecodedValue v;
    // 16-bit at offset 16 = 0xA53C = 42300; * 0.25 = 10575 rpm.
    const SignalDef d = mkDef(16, 16, RENDER_SCALED, 0, -1);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_OK);
    CHECK(v.valid);
    CHECK_EQ(v.raw, 0xA53C);
    CHECK_NEAR(v.value, 10575.0, 0.001);
    CHECK_STR(v.unit, "rpm");
    CHECK_EQ(v.decimals, 0);
    CHECK(v.label == 0);
    CHECK_STR(v.text, "");
  }
  {
    DecodedValue v;
    // 8-bit at offset 16 = 0xA5 = 165; * 0.5 - 40 = 42.5 C, 1 decimal.
    const SignalDef d = mkDef(16, 8, RENDER_SCALED, 1, -1);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_OK);
    CHECK(v.valid);
    CHECK_EQ(v.raw, 165);
    CHECK_NEAR(v.value, 42.5, 0.001);
    CHECK_STR(v.unit, "C");
    CHECK_EQ(v.decimals, 1);
  }
  {
    DecodedValue v;
    // scaling < 0: raw straight through, no unit.
    const SignalDef d = mkDef(16, 8, RENDER_SCALED, -1, -1);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_OK);
    CHECK(v.valid);
    CHECK_EQ(v.raw, 165);
    CHECK_NEAR(v.value, 165.0, 0.001);
    CHECK_STR(v.unit, "");
    CHECK_EQ(v.decimals, 0);
  }
  {
    DecodedValue v;
    // Dangling scaling index.
    const SignalDef d = mkDef(16, 8, RENDER_SCALED, 3, -1);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_BAD_PARAM);
    CHECK(!v.valid);
  }
  {
    DecodedValue v;
    // Short reply: field runs off the end -> unknown, not adjacent memory.
    const SignalDef d = mkDef(56, 16, RENDER_SCALED, 0, -1);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_OUT_OF_RANGE);
    CHECK(!v.valid);
    CHECK_EQ(v.raw, 0);
    CHECK_STR(v.unit, "");
  }
  {
    DecodedValue v;
    const SignalDef d0 = mkDef(0, 0, RENDER_SCALED, 0, -1);
    CHECK_EQ(dec(d0, kResp, 8, v), DECODE_BAD_PARAM);
    const SignalDef d33 = mkDef(0, 33, RENDER_SCALED, 0, -1);
    CHECK_EQ(dec(d33, kResp, 8, v), DECODE_BAD_PARAM);
  }
  {
    DecodedValue v;
    // NULL response.
    const SignalDef d = mkDef(0, 8, RENDER_SCALED, 0, -1);
    CHECK_EQ(dec(d, 0, 8, v), DECODE_BAD_PARAM);
    CHECK(!v.valid);
    CHECK_STR(v.unit, "");
  }

  // -------------------------------------------------------------------------
  // decodeSignal: RENDER_ENUM
  // -------------------------------------------------------------------------
  {
    DecodedValue v;
    // 4 bits at offset 16 = 0xA5 & 0xF = 5 -> falls in [2,9] = "MedOverlap".
    const SignalDef d = mkDef(16, 4, RENDER_ENUM, -1, 7);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_OK);
    CHECK(v.valid);
    CHECK_EQ(v.raw, 5);
    CHECK_STR(v.label, "MedOverlap");
    CHECK_NEAR(v.value, 5.0, 0.001);
    CHECK_STR(v.unit, "");
  }
  {
    DecodedValue v;
    // 8 bits at offset 16 = 165: covered by no row in group 7.
    const SignalDef d = mkDef(16, 8, RENDER_ENUM, -1, 7);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_NO_ENUM_MATCH);
    CHECK(!v.valid);
    CHECK_EQ(v.raw, 165);      // kept for a diagnostics screen
    CHECK(v.label == 0);       // no misleading label
  }
  {
    DecodedValue v;
    // enumGroup = -1 on a RENDER_ENUM row: nothing can match.
    const SignalDef d = mkDef(16, 4, RENDER_ENUM, -1, -1);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_NO_ENUM_MATCH);
    CHECK(!v.valid);
  }
  {
    DecodedValue v;
    const SignalDef d = mkDef(56, 16, RENDER_ENUM, -1, 7);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_OUT_OF_RANGE);
    CHECK(!v.valid);
  }

  // -------------------------------------------------------------------------
  // decodeSignal: RENDER_RAW
  // -------------------------------------------------------------------------
  {
    DecodedValue v;
    const SignalDef d = mkDef(16, 16, RENDER_RAW, 0, 7);  // both indices ignored
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_OK);
    CHECK(v.valid);
    CHECK_EQ(v.raw, 0xA53C);
    CHECK_NEAR(v.value, 42300.0, 0.001);
    CHECK_STR(v.unit, "");
    CHECK_EQ(v.decimals, 0);
    CHECK(v.label == 0);
  }
  {
    DecodedValue v;
    const SignalDef d = mkDef(64, 8, RENDER_RAW, -1, -1);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_OUT_OF_RANGE);
    CHECK(!v.valid);
  }

  // -------------------------------------------------------------------------
  // decodeSignal: unknown render type
  // -------------------------------------------------------------------------
  {
    DecodedValue v;
    const SignalDef d = mkDef(0, 8, 99, -1, -1);
    CHECK_EQ(dec(d, kResp, 8, v), DECODE_BAD_PARAM);
    CHECK(!v.valid);
  }

  // -------------------------------------------------------------------------
  // decodeSignal: RENDER_ASCII
  // -------------------------------------------------------------------------
  {
    // A 17-byte VIN after a 2-byte header. 17 chars + NUL = 18 <= 24, so it
    // must survive whole.
    uint8_t buf[64];
    const char* vin = "1J4GZ58S2VC123456";
    const uint16_t n = mkTextResp(buf, sizeof(buf), vin, 17);
    CHECK_EQ(n, 19);
    DecodedValue v;
    const SignalDef d = mkDef(16, 17 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_OK);
    CHECK(v.valid);
    CHECK_STR(v.text, "1J4GZ58S2VC123456");
    CHECK_EQ((long long)std::strlen(v.text), 17);
    CHECK_STR(v.unit, "");
    CHECK(v.label == 0);
  }
  {
    // Exactly DECODED_TEXT_MAX - 1 = 23 characters: all of them kept.
    uint8_t buf[64];
    const char* s23 = "ABCDEFGHIJKLMNOPQRSTUVW";
    const uint16_t n = mkTextResp(buf, sizeof(buf), s23, 23);
    DecodedValue v;
    const SignalDef d = mkDef(16, 23 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_OK);
    CHECK_STR(v.text, "ABCDEFGHIJKLMNOPQRSTUVW");
    CHECK_EQ((long long)std::strlen(v.text), 23);
  }
  {
    // Exactly DECODED_TEXT_MAX = 24 characters: one must be dropped, and the
    // NUL must land inside the buffer.
    uint8_t buf[64];
    const char* s24 = "ABCDEFGHIJKLMNOPQRSTUVWX";
    const uint16_t n = mkTextResp(buf, sizeof(buf), s24, 24);
    DecodedValue v;
    const SignalDef d = mkDef(16, 24 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_OK);
    CHECK_STR(v.text, "ABCDEFGHIJKLMNOPQRSTUVW");
    CHECK_EQ((long long)std::strlen(v.text), 23);
    CHECK_EQ(v.text[DECODED_TEXT_MAX - 1], 0);
  }
  {
    // A 30-byte field truncates to 23 characters, no overrun.
    uint8_t buf[64];
    const char* s30 = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123";
    const uint16_t n = mkTextResp(buf, sizeof(buf), s30, 30);
    CHECK_EQ(n, 32);
    DecodedValue v;
    const SignalDef d = mkDef(16, 30 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_OK);
    CHECK_STR(v.text, "ABCDEFGHIJKLMNOPQRSTUVW");
    CHECK_EQ(v.text[DECODED_TEXT_MAX - 1], 0);
  }
  {
    // Trailing spaces stripped.
    uint8_t buf[32];
    const char* pn = "P05064033AA   ";
    const uint16_t n = mkTextResp(buf, sizeof(buf), pn, 14);
    DecodedValue v;
    const SignalDef d = mkDef(16, 14 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_OK);
    CHECK(v.valid);
    CHECK_STR(v.text, "P05064033AA");
  }
  {
    // NUL padding terminates.
    uint8_t buf[32];
    const char pad[6] = {'A', 'B', 'C', '\0', '\0', '\0'};
    const uint16_t n = mkTextResp(buf, sizeof(buf), pad, 6);
    DecodedValue v;
    const SignalDef d = mkDef(16, 6 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_OK);
    CHECK_STR(v.text, "ABC");
  }
  {
    // 0xFF padding terminates too.
    uint8_t buf[32];
    const char pad[6] = {'A', 'B', 'C', (char)0xFF, (char)0xFF, (char)0xFF};
    const uint16_t n = mkTextResp(buf, sizeof(buf), pad, 6);
    DecodedValue v;
    const SignalDef d = mkDef(16, 6 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_OK);
    CHECK_STR(v.text, "ABC");
  }
  {
    // All-blank field: read fine, simply empty. Distinct from a read failure.
    uint8_t buf[32];
    const char blank[4] = {' ', ' ', ' ', ' '};
    const uint16_t n = mkTextResp(buf, sizeof(buf), blank, 4);
    DecodedValue v;
    const SignalDef d = mkDef(16, 4 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_OK);
    CHECK(v.valid);
    CHECK_STR(v.text, "");
  }
  {
    // A control byte in the middle is not text: reject, do not render mojibake.
    uint8_t buf[32];
    const char bad[4] = {'A', 'B', (char)0x01, 'C'};
    const uint16_t n = mkTextResp(buf, sizeof(buf), bad, 4);
    DecodedValue v;
    const SignalDef d = mkDef(16, 4 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_BAD_PARAM);
    CHECK(!v.valid);
    CHECK_STR(v.text, "");   // nothing partial leaked out
  }
  {
    // High-bit byte likewise.
    uint8_t buf[32];
    const char bad[3] = {'A', (char)0x80, 'B'};
    const uint16_t n = mkTextResp(buf, sizeof(buf), bad, 3);
    DecodedValue v;
    const SignalDef d = mkDef(16, 3 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_BAD_PARAM);
    CHECK(!v.valid);
  }
  {
    // The field exactly fills the response.
    uint8_t buf[32];
    const uint16_t n = mkTextResp(buf, sizeof(buf), "XYZ", 3);
    CHECK_EQ(n, 5);
    DecodedValue v;
    const SignalDef d = mkDef(16, 3 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_OK);
    CHECK_STR(v.text, "XYZ");
    // One byte more than the module sent.
    const SignalDef d4 = mkDef(16, 4 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d4, buf, n, v), DECODE_OUT_OF_RANGE);
    CHECK(!v.valid);
  }
  {
    // A short reply to a VIN request.
    uint8_t buf[32];
    const uint16_t n = mkTextResp(buf, sizeof(buf), "1J4GZ58", 7);
    DecodedValue v;
    const SignalDef d = mkDef(16, 17 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(d, buf, n, v), DECODE_OUT_OF_RANGE);
    CHECK(!v.valid);
  }
  {
    // Unaligned offset / length on a text field is a table bug.
    uint8_t buf[32];
    const uint16_t n = mkTextResp(buf, sizeof(buf), "XYZ", 3);
    DecodedValue v;
    const SignalDef dOff = mkDef(17, 3 * 8, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(dOff, buf, n, v), DECODE_BAD_PARAM);
    const SignalDef dLen = mkDef(16, 12, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(dLen, buf, n, v), DECODE_BAD_PARAM);
    const SignalDef dZero = mkDef(16, 0, RENDER_ASCII, -1, -1);
    CHECK_EQ(dec(dZero, buf, n, v), DECODE_BAD_PARAM);
  }

  // -------------------------------------------------------------------------
  // The IsoTpPdu convenience overload. Uses RENDER_RAW / scaling = -1 so it
  // needs nothing from the generated tables.
  // -------------------------------------------------------------------------
  {
    IsoTpPdu pdu;
    std::memset(&pdu, 0, sizeof(pdu));
    pdu.data[0] = 0x61;
    pdu.data[1] = 0x21;
    pdu.data[2] = 0xA5;
    pdu.data[3] = 0x3C;
    pdu.len = 4;
    DecodedValue v;
    const SignalDef d = mkDef(16, 16, RENDER_RAW, -1, -1);
    CHECK_EQ(decodeSignal(d, pdu, v), DECODE_OK);
    CHECK(v.valid);
    CHECK_EQ(v.raw, 0xA53C);

    const SignalDef dScaled = mkDef(16, 16, RENDER_SCALED, -1, -1);
    CHECK_EQ(decodeSignal(dScaled, pdu, v), DECODE_OK);
    CHECK(v.valid);
    CHECK_NEAR(v.value, 42300.0, 0.001);
    CHECK_STR(v.unit, "");

    // Past the declared length.
    const SignalDef dPast = mkDef(32, 8, RENDER_RAW, -1, -1);
    CHECK_EQ(decodeSignal(dPast, pdu, v), DECODE_OUT_OF_RANGE);

    // A bogus len must be clamped to ISOTP_MAX_PDU, not trusted.
    pdu.len = 5000;
    const SignalDef dFar = mkDef(ISOTP_MAX_PDU * 8, 8, RENDER_RAW, -1, -1);
    CHECK_EQ(decodeSignal(dFar, pdu, v), DECODE_OUT_OF_RANGE);
    const SignalDef dNear = mkDef(0, 8, RENDER_RAW, -1, -1);
    CHECK_EQ(decodeSignal(dNear, pdu, v), DECODE_OK);
    CHECK_EQ(v.raw, 0x61);
  }

TEST_MAIN_END()
