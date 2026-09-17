// Line classifier host tests.
//
// TWO FRAME SHAPES ARRIVE ON ONE WIRE, and this suite is about telling all
// three kinds of line apart with nothing but the text of one line: a
// LAWICEL/SLCAN background STREAM frame, an ELM-shaped request RESPONSE, and a
// command reply. core/LinkLine.h argues the case; this file is the evidence for
// it, and the sweeps at the bottom are the part that would notice the argument
// stopping being true.
//
// THE ELM-SHAPED CASES (kFrames) are real lines off this vehicle, taken from
// ../jeep-aux-and-can-gateway/logs/inj-1A7.txt. That capture was recorded with
// ATS1 (spaces between bytes), so each line below is the same frame with the
// spaces stripped -- which is the ATS0 form the wire actually carries. Under
// AT@SML, the stream format this host now asks for, a line of this shape is
// only ever a request RESPONSE and is NEVER stamped, so width 0 is what the
// reader passes for it. The stamped widths stay under test because the AT@SM
// fallback still produces them, and because a width nothing exercises is a
// width nobody will notice breaking.
//
// THE LAWICEL CASES (kStreams) are real lines from a stamped AT@SML2 drive
// capture, ../../jeep-can-b-logs/drive log - becici to school on top of the
// steep hill and back - 2026-09-17_074126.log -- outside this repo, which is
// why every fixture is repeated in full here rather than referenced. Its
// corrupted lines are in the splice section further down, from the same file.
// The shape is 't' + 3 hex id nibbles + ONE HEX DLC NIBBLE + 2 hex per
// data byte + an optional 4- or 8-digit stamp, so
//
//   len == 5 + 2 * dlc + width
//
// and the marker is lowercase because in LAWICEL 't' is the 11-bit standard
// data frame and 'T' is the 29-bit extended one, which CanFrame has nowhere to
// put.
//
// THE REPLY CASES are the complete reply vocabulary of the gateway firmware:
// the five fixed strings src/At328Link.cpp matches with sameText(), the
// FW_IDENT from lib/Core/Version.h, the AT@EV event lines, and the AT@ST status
// line in BOTH its forms -- the pre-2.1 one carrying SM= alone and the 2.1 one
// carrying SM= and SML=, since the two stream formats are one field in the
// gateway and a host reading that line has to survive either.
//
// Every fixture is re-derived here rather than trusted. kFrames is checked to
// be odd-length and pure hex; kStreams is checked to carry the marker, to be
// pure hex after it, and to have exactly the length its OWN DLC nibble implies.
// The whole point of the LAWICEL table is that the line says how it splits, so
// a fixture with a stamp digit miscopied would assert the opposite of what it
// claims.
//
// STAMP WIDTHS, AND WHAT THE AMBIGUITY ARGUMENT BECOMES.
//
// This suite used to exist for one assertion, and it still makes it: on the ELM
// shape the data length is only IMPLIED by the hex count, so the same 19
// characters are a valid frame line under BOTH width 0 and width 4 -- 3 + 2*8
// and 3 + 2*6 + 4 -- and the two readings decode to different data. The width
// therefore cannot be recovered from the line: it is configuration, owned by
// the side that chose the stream level and passed down. Getting it wrong is
// silent in both directions, either filing a frame as a command reply or
// reading two data bytes as a time, and that assertion stays here because the
// AT@SM fallback still uses that shape.
//
// It does NOT carry over to the LAWICEL shape, and that is the whole reason
// this host moved. The DLC nibble is ON THE WIRE, so the stamp is whatever is
// left after the data --
//
//   width = len - 5 - 2 * dlc
//
// -- and the only judgement left is whether that remainder is a width the
// firmware can produce (0, 4 or 8). The ambiguity has not been removed from the
// format; it has been moved to the SENDER. `t1A76005E000000005A30` and
// `t1A74005E000000005A30` are the same 21 characters split two different ways,
// both legal, both pure hex, and the one character that decides which is IN THE
// LINE rather than in a setting the reader has to be told. So the LAWICEL cases
// below hand linkClassifyLine() a tsChars it is expected to IGNORE, and assert
// that it does, while the derived width is asserted as an OUTPUT of the parser.
//
// The routing rules that follow from the marker are asserted rather than
// assumed: a 't' line that fails the LAWICEL shape check is a REPLY and is
// never retried under the ELM rules, so prefixing a perfectly good ELM frame
// line with 't' TAKES ITS READING AWAY instead of adding one. That is what
// stops a spliced line -- and the gateway does splice them, echoing and
// rebooting mid-frame -- from being decoded with a plausible time built out of
// somebody else's data bytes.
//
// ACCEPT IFF DECODE, over both shapes, is what the two sweeps at the bottom are
// for. Classify and parse must be the same predicate or the failure is silent:
// a frame filed as a command's answer hands a transaction someone else's bytes,
// and a reply decoded as a frame puts a plausible wrong number on the
// dashboard. Both sweeps also sweep the REJECTS, because on this wire a
// rejected line is the common case and the interesting one.
#include "doctest_min.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "CarLink.h"
#include "LinkLine.h"

namespace {

// tsChars defaults to the unstamped width, which is what most of this suite is
// about and what AT@SM1 -- still the default level -- produces.
LinkLineKind classify(const char* s, uint8_t tsChars = LINK_LINE_TS_NONE) {
  return linkClassifyLine(s, (uint16_t)strlen(s), tsChars);
}

// Independent re-implementation of "what is this hex digit worth", used only to
// audit the fixtures and to mirror the decode below. Deliberately NOT the code
// in LinkLine.cpp or At328Link.cpp: if it were the same code the audit would be
// circular. -1 for anything that is not a hex digit.
int nibble(char c) {
  static const char kDigits[] = "0123456789ABCDEFabcdef";
  for (uint8_t i = 0u; kDigits[i] != '\0'; ++i) {
    if (kDigits[i] == c) {
      return i < 16u ? (int)i : (int)i - 6;   // 'a'..'f' fold onto 10..15
    }
  }
  return -1;
}

bool auditHex(char c) {
  return nibble(c) >= 0;
}

// An INDEPENDENT MIRROR of linkParseFrameLine, for the one thing a classifier
// cannot show on its own: that a line accepted under two widths MEANS two
// different things.
//
// It was written when the real parser was a private static of At328Link, in an
// ESP-IDF translation unit no host test could link. That is no longer true --
// linkParseFrameLine() is in core/LinkLine.cpp and the differential sweep at
// the bottom of this file calls it directly -- but this stays, and stays
// hand-written, for the same reason nibble() is not LinkLine.cpp's isHexDigit:
// the ambiguity below is evidence about what the FORMAT means, and asserting it
// with the code under test would make it circular. The sweep cross-checks the
// two against each other, so a drift between them fails rather than hides.
struct Decoded {
  uint16_t id;
  uint8_t  len;
  uint8_t  data[CAN_MAX_DLC];
  uint32_t stamp;
};

bool decode(const char* line, uint8_t tsChars, Decoded& out) {
  const uint16_t len = (uint16_t)strlen(line);
  if (len < (uint16_t)(LINK_LINE_FRAME_MIN + tsChars)) {
    return false;
  }
  const uint16_t dataChars = (uint16_t)(len - LINK_LINE_FRAME_MIN - tsChars);
  if ((dataChars & 1u) != 0u || (dataChars / 2u) > CAN_MAX_DLC) {
    return false;
  }
  out.id = 0u;
  out.stamp = 0u;
  for (uint16_t i = 0u; i < 3u; ++i) {
    const int v = nibble(line[i]);
    if (v < 0) {
      return false;
    }
    out.id = (uint16_t)((out.id << 4) | (uint16_t)v);
  }
  out.len = (uint8_t)(dataChars / 2u);
  for (uint8_t i = 0u; i < CAN_MAX_DLC; ++i) {
    out.data[i] = 0u;
  }
  for (uint8_t i = 0u; i < out.len; ++i) {
    const int hi = nibble(line[3u + i * 2u]);
    const int lo = nibble(line[4u + i * 2u]);
    if (hi < 0 || lo < 0) {
      return false;
    }
    out.data[i] = (uint8_t)((hi << 4) | lo);
  }
  for (uint8_t i = 0u; i < tsChars; ++i) {
    const int v = nibble(line[3u + dataChars + i]);
    if (v < 0) {
      return false;
    }
    out.stamp = (out.stamp << 4) | (uint32_t)v;
  }
  return true;
}

// The accepted set, stated once and independently of the implementation:
// 3 + 2n + W for n in 0..CAN_MAX_DLC. Used to sweep every length against every
// width, so that "exact set" is tested as a set rather than sampled at its
// ends.
bool lengthFits(uint16_t len, uint8_t tsChars) {
  for (uint8_t n = 0u; n <= CAN_MAX_DLC; ++n) {
    if (len == (uint16_t)(3u + 2u * n + tsChars)) {
      return true;
    }
  }
  return false;
}

// ---- the LAWICEL/SLCAN side ----------------------------------------------
//
// 't' + 3 hex id nibbles + ONE DLC NIBBLE + 2 hex per data byte + an optional
// 4- or 8-digit stamp. Everything in this section is written from that sentence
// and from the LAWICEL literals -- 5, 29, 4, 8 -- rather than from LinkLine's
// macros, for the same reason nibble() is not LinkLine.cpp's isHexDigit: a
// suite that borrows the implementation's constants cannot notice one of them
// moving, and the macro checks further down are then checking a macro against
// itself.

// Build one LAWICEL line from the format. GENERATED rather than copied, so a
// sweep case is evidence about the shape and not about a fixture and its
// generator sharing a typo.
//
// `dlc` is both the nibble written and the number of bytes taken, so it must be
// 0..CAN_MAX_DLC; the reject sweep makes the 9..F the firmware can never emit
// by patching buf[4] afterwards, rather than by letting this read past `data`.
// Returns the length written, which is what the caller then asserts against
// 5 + 2 * dlc + ts.
uint16_t buildSlcan(char* buf, uint16_t id, uint8_t dlc, const uint8_t* data,
                    uint8_t ts, uint32_t stamp) {
  static const char kHex[] = "0123456789ABCDEF";
  uint16_t n = 0u;
  buf[n++] = 't';
  buf[n++] = kHex[(id >> 8) & 0x0Fu];
  buf[n++] = kHex[(id >> 4) & 0x0Fu];
  buf[n++] = kHex[id & 0x0Fu];
  buf[n++] = kHex[dlc & 0x0Fu];
  for (uint8_t i = 0u; i < dlc; ++i) {
    buf[n++] = kHex[(data[i] >> 4) & 0x0Fu];
    buf[n++] = kHex[data[i] & 0x0Fu];
  }
  // Big-endian, most significant nibble first, which is how the gateway prints
  // millis(): a stamp written the other way round would still be pure hex and
  // still the right length, so the fixtures are what catch that, not the shape.
  for (uint8_t i = 0u; i < ts; ++i) {
    const uint8_t shift = (uint8_t)(4u * (ts - 1u - i));
    buf[n++] = kHex[(stamp >> shift) & 0x0Fu];
  }
  buf[n] = '\0';
  return n;
}

// The accepted LAWICEL length set, stated once and independently of the
// implementation: 5 + 2d + W for d in 0..CAN_MAX_DLC and W in {0, 4, 8} -- that
// is every ODD length from 5 to 29 and nothing else. Worth having as a set
// because the reject sweep walks 0..40 and this is what says which of those
// lengths no (dlc, width) pair can produce AT ALL, as opposed to the far larger
// number that this particular line's DLC nibble rules out.
bool slcanLengthPossible(uint16_t len) {
  for (uint8_t d = 0u; d <= CAN_MAX_DLC; ++d) {
    if (len == (uint16_t)(5u + 2u * d) || len == (uint16_t)(5u + 2u * d + 4u) ||
        len == (uint16_t)(5u + 2u * d + 8u)) {
      return true;
    }
  }
  return false;
}

// An INDEPENDENT MIRROR of linkParseStreamLine, in the same spirit as decode()
// above and for the same reason: the sweep asserts that the shipping parser and
// a reading of the format written from the format agree, byte for byte and
// digit for digit, so a drift between them fails rather than hides. `ts` is an
// output here too -- a mirror that took the width as an argument would not be
// mirroring the thing that matters.
struct StreamDecoded {
  uint16_t id;
  uint8_t  len;
  uint8_t  data[CAN_MAX_DLC];
  uint32_t stamp;
  uint8_t  ts;        // DERIVED from the DLC nibble and the length, never told
};

bool streamDecode(const char* line, uint16_t len, StreamDecoded& out) {
  if (line == 0 || len < 5u || len > 29u) {
    return false;
  }
  if (line[0] != 't') {
    return false;        // and 'T', the extended frame, lands here
  }
  for (uint16_t i = 1u; i < len; ++i) {
    if (nibble(line[i]) < 0) {
      return false;
    }
  }
  const int d = nibble(line[4]);
  if (d < 0 || d > (int)CAN_MAX_DLC) {
    return false;        // 9..F are CAN FD lengths this firmware never emits
  }
  const uint16_t head = (uint16_t)(5u + 2u * (uint16_t)d);
  if (len < head) {
    return false;        // the DLC claims more data than the line carries
  }
  const uint16_t ts = (uint16_t)(len - head);
  if (ts != 0u && ts != 4u && ts != 8u) {
    return false;        // a remainder no stream level can produce
  }

  out.id = 0u;
  for (uint16_t i = 1u; i < 4u; ++i) {
    out.id = (uint16_t)((out.id << 4) | (uint16_t)nibble(line[i]));
  }
  out.len = (uint8_t)d;
  for (uint8_t i = 0u; i < CAN_MAX_DLC; ++i) {
    out.data[i] = 0u;
  }
  for (uint8_t i = 0u; i < out.len; ++i) {
    out.data[i] = (uint8_t)((nibble(line[5u + i * 2u]) << 4) |
                            nibble(line[6u + i * 2u]));
  }
  out.stamp = 0u;
  for (uint16_t i = 0u; i < ts; ++i) {
    out.stamp = (out.stamp << 4) | (uint32_t)nibble(line[head + i]);
  }
  out.ts = (uint8_t)ts;
  return true;
}

struct FrameCase {
  const char* line;
  uint16_t    id;        // what the parser would extract, for documentation
  uint8_t     dataLen;
};

// Real frames, captured with the engine running. The id column is the decode
// core/Broadcast.cpp already knows: 002 RPM+speed, 014 odometer, 015 ambient
// and volts, 1A7 fuel flow, 210 panel dimmer, 1C8 exterior lights.
const FrameCase kFrames[] = {
  {"002041B0E1E4F264FB8", 0x002u, 8u},   // "002 04 1B 0E 1E 4F 26 4F B8"
  {"00204110E254F1A4FAB", 0x002u, 8u},   // "002 04 11 0E 25 4F 1A 4F AB"
  {"2C0FFFF170B95FF1782", 0x2C0u, 8u},   // "2C0 FF FF 17 0B 95 FF 17 82"
  {"0142596A9FF1B563A",   0x014u, 7u},   // "014 25 96 A9 FF 1B 56 3A"
  {"1A700E700000000",     0x1A7u, 6u},   // "1A7 00 E7 00 00 00 00"
  {"000850000000000",     0x000u, 6u},   // "000 85 00 00 00 00 00"
  {"0158F8B2BFF0000",     0x015u, 6u},   // "015 8F 8B 2B FF 00 00"
  {"3F8040000000000",     0x3F8u, 6u},   // "3F8 04 00 00 00 00 00"
  {"0138000004999",       0x013u, 5u},   // "013 80 00 00 49 99"
  {"11D000000",           0x11Du, 3u},   // "11D 00 00 00"
  {"21003C8",             0x210u, 2u},   // "210 03 C8"
  {"3A00000",             0x3A0u, 2u},   // "3A0 00 00"
  {"02E1F00",             0x02Eu, 2u},   // "02E 1F 00"
  {"0D00400",             0x0D0u, 2u},   // "0D0 04 00"
  {"1C802",               0x1C8u, 1u},   // "1C8 02"
};
const uint8_t kFrameCount = (uint8_t)(sizeof(kFrames) / sizeof(kFrames[0]));

// Real LAWICEL lines, verbatim from a stamped AT@SML2 drive capture, with the
// full reading each one claims: id, DLC, every data byte (zeros past the DLC,
// which is what the parser must zero-fill to), the stamp, and the width the
// line's own arithmetic implies. Every one of them is a 4-digit stamp because
// that is the level this host asks for; the 0- and 8-digit widths are generated
// from these same frames further down, where the sweep can cover all three
// against the same payloads.
//
// The ids are the ones core/Broadcast.cpp already decodes: 002 RPM+speed, 014
// odometer, 11D, 1A7 fuel flow, 1C8 exterior lights, 2C0, 3F8.
struct StreamCase {
  const char* line;
  uint16_t    id;
  uint8_t     dlc;
  uint8_t     data[CAN_MAX_DLC];
  uint32_t    stamp;
  uint8_t     ts;
};

const StreamCase kStreams[] = {
  {"t0147259C23FF18CE597816", 0x014u, 7u,
   {0x25u, 0x9Cu, 0x23u, 0xFFu, 0x18u, 0xCEu, 0x59u, 0x00u}, 0x7816u, 4u},
  {"t11D30000007809", 0x11Du, 3u,
   {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}, 0x7809u, 4u},
  {"t1C81025A51", 0x1C8u, 1u,
   {0x02u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}, 0x5A51u, 4u},
  {"t002800000000000000007819", 0x002u, 8u,
   {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}, 0x7819u, 4u},
  {"t2C08FFFF170B7DFF17027825", 0x2C0u, 8u,
   {0xFFu, 0xFFu, 0x17u, 0x0Bu, 0x7Du, 0xFFu, 0x17u, 0x02u}, 0x7825u, 4u},
  {"t1A76005E000000005A30", 0x1A7u, 6u,
   {0x00u, 0x5Eu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}, 0x5A30u, 4u},
  {"t3F860400000000007817", 0x3F8u, 6u,
   {0x04u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}, 0x7817u, 4u},
};
const uint8_t kStreamCount = (uint8_t)(sizeof(kStreams) / sizeof(kStreams[0]));

// The payloads the (dlc x width) sweep is generated from: the capture's own
// bytes, zero-extended to eight where the frame carried fewer. The byte VALUES
// are not what a shape sweep is about, but real ones keep it honest at the
// extremes -- 0xFF and 0x00 runs, and the 0x0B/0x0D nibbles a formatter that
// dropped a leading zero would mangle.
struct SweepFrame {
  uint16_t id;
  uint8_t  data[CAN_MAX_DLC];
};

const SweepFrame kSweep[] = {
  {0x002u, {0x04u, 0x1Bu, 0x0Eu, 0x1Eu, 0x4Fu, 0x26u, 0x4Fu, 0xB8u}},
  {0x014u, {0x25u, 0x9Cu, 0x23u, 0xFFu, 0x18u, 0xCEu, 0x59u, 0x00u}},
  {0x2C0u, {0xFFu, 0xFFu, 0x17u, 0x0Bu, 0x7Du, 0xFFu, 0x17u, 0x02u}},
  {0x1A7u, {0x00u, 0x5Eu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
  {0x3F8u, {0x04u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
  {0x11Du, {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
  {0x1C8u, {0x02u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u}},
};
const uint8_t kSweepCount = (uint8_t)(sizeof(kSweep) / sizeof(kSweep[0]));

// Every reply the firmware can put on the wire.
const char* const kReplies[] = {
  "OK",                    // AtServer RP_OK
  "?",                     // unknown command
  "NO DATA",               // request timed out with no answer
  "CAN ERROR",             // bus write failed
  "BUFFER FULL",           // TX ring or frame budget exhausted
  "latonita-jeep v2.1",    // FW_IDENT, answer to ATI and ATZ
  "AT@EV AUX 1",           // asynchronous event, AtServer::emit()
  "AT@EV BUTTON 0",
  "AT@EV RADIO_MODE 3",
  "@EV CAN 1",             // the shorter form the header comment documents
  // AT@ST, one line; parenthesised only to tell the compiler the split is
  // deliberate and not a missing comma.
  ("FW=2.1 MODE=01 AUX=0 BT=0 BENCH=0 CAN=OK RX=1024 TX=7 ER=0 SH=7E0 CF=000 "
   "CM=000 F=VTB UP=93 RAM=612 RST=01 REI=0 SM=1 SD=0"),
  "AT@EV CAN 0",           // the CAN health event, in its AT@-prefixed form
  // The 2.1 status line, which grew a field: SM= and SML= are the ELM and
  // LAWICEL stream levels, one field in the gateway and so mutually exclusive
  // -- SM=0 SML=2 is a session in the mode this file is about. Both vintages
  // of the line are kept, because a host has to survive an adapter of either.
  ("FW=2.1 MODE=01 AUX=0 BT=0 BENCH=0 CAN=OK RX=1024 TX=7 ER=0 SH=7E0 CF=000 "
   "CM=000 F=VTB UP=93 RAM=612 RST=01 REI=0 SM=0 SML=2 SD=0"),
  ">",                     // consumed by readLine(), but never a frame
};
const uint8_t kReplyCount = (uint8_t)(sizeof(kReplies) / sizeof(kReplies[0]));

}  // namespace

TEST_MAIN_BEGIN()

// ------------------------------------------------- the bound is the DLC bound
{
  // 19, but derived: 3 id chars plus two per data byte. If CAN_MAX_DLC ever
  // moved, the classifier would move with it rather than silently reject.
  CHECK_EQ(LINK_LINE_FRAME_MIN, 3);
  CHECK_EQ(LINK_LINE_FRAME_MAX, 19);

  // The stamp widths are the adapter's, not ours to choose: 4 digits of
  // millis() % 60000 at AT@SM2, 8 of raw millis() at AT@SM3. Both EVEN, which
  // is what keeps a frame line odd whether it is stamped or not -- the rule the
  // classifier leans on hardest.
  CHECK_EQ(LINK_LINE_TS_NONE, 0);
  CHECK_EQ(LINK_LINE_TS_SHORT, 4);
  CHECK_EQ(LINK_LINE_TS_LONG, 8);
  CHECK_EQ(LINK_LINE_TS_SHORT & 1u, 0);
  CHECK_EQ(LINK_LINE_TS_LONG & 1u, 0);
}

// -------------------------------------------------------------- fixture audit
// Before any of the frame fixtures is used as evidence, check it really has the
// shape claimed for it: odd length, in range, pure hex, and the data byte count
// the comment says.
{
  for (uint8_t i = 0u; i < kFrameCount; ++i) {
    const FrameCase& fc = kFrames[i];
    const uint16_t len = (uint16_t)strlen(fc.line);
    CHECK_MSG((len & 1u) == 1u, "%s: length %u is EVEN, not a frame line", fc.line,
              (unsigned)len);
    CHECK_MSG(len >= 3u && len <= 19u, "%s: length %u out of 3..19", fc.line, (unsigned)len);
    CHECK_MSG(len == (uint16_t)(3u + 2u * fc.dataLen), "%s: length %u does not match %u bytes",
              fc.line, (unsigned)len, (unsigned)fc.dataLen);
    CHECK_MSG(fc.dataLen <= CAN_MAX_DLC, "%s: %u data bytes exceeds the DLC", fc.line,
              (unsigned)fc.dataLen);
    for (uint16_t k = 0u; k < len; ++k) {
      CHECK_MSG(auditHex(fc.line[k]), "%s: char %u ('%c') is not a hex digit", fc.line,
                (unsigned)k, fc.line[k]);
    }
  }
  CHECK_EQ(kFrameCount, 15);
}

// ---------------------------------------------------- real frames are FRAME
{
  for (uint8_t i = 0u; i < kFrameCount; ++i) {
    const LinkLineKind k = classify(kFrames[i].line);
    CHECK_MSG(k == LINK_LINE_FRAME, "%s: expected FRAME, got %d", kFrames[i].line, (int)k);
  }
}

// ---------------------------------------------------- real replies are REPLY
{
  for (uint8_t i = 0u; i < kReplyCount; ++i) {
    const LinkLineKind k = classify(kReplies[i]);
    CHECK_MSG(k == LINK_LINE_REPLY, "\"%s\": expected REPLY, got %d", kReplies[i], (int)k);
  }
  CHECK_EQ(kReplyCount, 14);
}

// ------------------------------------------------------------------ boundaries
{
  // 3 is the shortest frame: a DLC-0 frame, printed as a bare id. None appears
  // in the capture -- nothing on this bus sends an empty frame -- but the
  // gateway would print one exactly like this, so the boundary is tested.
  CHECK_EQ(classify("002"), LINK_LINE_FRAME);
  CHECK_EQ(classify("7E8"), LINK_LINE_FRAME);
  CHECK_EQ(classify("000"), LINK_LINE_FRAME);

  // 19 is the longest: a full 8-byte frame, which is most of this bus.
  CHECK_EQ(strlen("002041B0E1E4F264FB8"), 19u);
  CHECK_EQ(classify("002041B0E1E4F264FB8"), LINK_LINE_FRAME);

  // 21 would be a 9-byte frame, which classical CAN cannot carry. Odd and pure
  // hex, so ONLY the length bound rejects it.
  CHECK_EQ(strlen("002041B0E1E4F264FB8AB"), 21u);
  CHECK_EQ(classify("002041B0E1E4F264FB8AB"), LINK_LINE_REPLY);

  // Below the id itself.
  CHECK_EQ(classify("0"), LINK_LINE_REPLY);
  CHECK_EQ(classify("00"), LINK_LINE_REPLY);
  CHECK_EQ(classify("AB"), LINK_LINE_REPLY);

  // Well past anything the protocol defines.
  CHECK_EQ(classify("0123456789ABCDEF0123456789ABCDEF0123456789"), LINK_LINE_REPLY);
}

// ------------------------------------------------------- even length is REPLY
// The rule that does most of the work, so it gets swept rather than sampled:
// every even length in range, all hex, must still be REPLY.
{
  char buf[32];
  for (uint16_t len = 4u; len <= 20u; len += 2u) {
    for (uint16_t i = 0u; i < len; ++i) {
      buf[i] = "0123456789ABCDEF"[i & 0x0Fu];
    }
    buf[len] = '\0';
    const LinkLineKind k = linkClassifyLine(buf, len, LINK_LINE_TS_NONE);
    CHECK_MSG(k == LINK_LINE_REPLY, "even length %u: expected REPLY, got %d", (unsigned)len,
              (int)k);
  }
  // ... and every odd length in range, all hex, must be FRAME.
  for (uint16_t len = 3u; len <= 19u; len += 2u) {
    for (uint16_t i = 0u; i < len; ++i) {
      buf[i] = "0123456789ABCDEF"[i & 0x0Fu];
    }
    buf[len] = '\0';
    const LinkLineKind k = linkClassifyLine(buf, len, LINK_LINE_TS_NONE);
    CHECK_MSG(k == LINK_LINE_FRAME, "odd length %u: expected FRAME, got %d", (unsigned)len,
              (int)k);
  }
}

// ------------------------------------------------------------- lowercase hex
// The gateway prints uppercase, but src/At328Link.cpp's hexVal() accepts a-f,
// so the classifier must as well -- otherwise a line the parser would decode
// gets filed as a reply.
{
  CHECK_EQ(classify("2c0ffff170b95ff1782"), LINK_LINE_FRAME);
  CHECK_EQ(classify("1a700e700000000"), LINK_LINE_FRAME);
  CHECK_EQ(classify("0d00400"), LINK_LINE_FRAME);
  CHECK_EQ(classify("abc"), LINK_LINE_FRAME);
  CHECK_EQ(classify("aBcDeF0"), LINK_LINE_FRAME);   // mixed case, as tolerated

  // Every hex digit in both cases is accepted somewhere in the line.
  static const char* const kAll = "0123456789ABCDEFabcdef";
  for (uint8_t i = 0u; kAll[i] != '\0'; ++i) {
    const char s[4] = {kAll[i], kAll[i], kAll[i], '\0'};
    CHECK_MSG(classify(s) == LINK_LINE_FRAME, "'%c' should be a hex digit", kAll[i]);
  }
  // 'g' is the first letter past the set, in both cases.
  CHECK_EQ(classify("ggg"), LINK_LINE_REPLY);
  CHECK_EQ(classify("GGG"), LINK_LINE_REPLY);
}

// -------------------------------------------------------------- empty / NULL
{
  CHECK_EQ(linkClassifyLine(0, 0u, LINK_LINE_TS_NONE), LINK_LINE_EMPTY);
  CHECK_EQ(linkClassifyLine(0, 19u, LINK_LINE_TS_NONE), LINK_LINE_EMPTY);  // NULL wins
  CHECK_EQ(linkClassifyLine("", 0u, LINK_LINE_TS_NONE), LINK_LINE_EMPTY);
  CHECK_EQ(linkClassifyLine("002", 0u, LINK_LINE_TS_NONE), LINK_LINE_EMPTY);  // len is
                                                        // the truth, not the text
  // A width does not rescue an empty line, in either direction.
  CHECK_EQ(linkClassifyLine(0, 23u, LINK_LINE_TS_SHORT), LINK_LINE_EMPTY);
  CHECK_EQ(linkClassifyLine("0021234", 0u, LINK_LINE_TS_SHORT), LINK_LINE_EMPTY);
}

// ------------------------------------------------- a space is always a REPLY
// The capture is ATS1, so its raw lines still carry the separators. Those must
// never classify as frames: under ATS0 a space cannot appear in a frame line,
// and a host that accepted one would decode the separator as a nibble.
{
  CHECK_EQ(classify("210 03 C8"), LINK_LINE_REPLY);
  CHECK_EQ(classify("1A7 00 E7"), LINK_LINE_REPLY);
  CHECK_EQ(classify("013 80 00 00 49"), LINK_LINE_REPLY);

  // Odd length, in range, and hex apart from the one space -- the case where
  // only the character scan can save us.
  CHECK_EQ(strlen("21 003C8"), 8u);
  CHECK_EQ(classify("002 41B0E1E4F264FB8"), LINK_LINE_REPLY);   // 19 chars, odd
  CHECK_EQ(strlen("002 41B0E1E4F264FB8"), 19u);
  CHECK_EQ(classify("00 204"), LINK_LINE_REPLY);

  // Leading and trailing whitespace likewise. readLine() strips neither.
  CHECK_EQ(classify(" 002"), LINK_LINE_REPLY);
  CHECK_EQ(classify("002 "), LINK_LINE_REPLY);
  CHECK_EQ(classify("0 2"), LINK_LINE_REPLY);
  CHECK_EQ(classify("\t002"), LINK_LINE_REPLY);
}

// -------------------------------------- near-misses that must not become frames
// Replies that are the right length and very nearly hex. These are the lines a
// looser classifier would get wrong, and each one would cost a real reading.
{
  CHECK_EQ(classify("OK"), LINK_LINE_REPLY);       // 2: too short
  CHECK_EQ(classify("NO DATA"), LINK_LINE_REPLY);  // 7: odd and in range, space saves it
  CHECK_EQ(classify("CAN ERROR"), LINK_LINE_REPLY);// 9: odd and in range, ditto
  CHECK_EQ(strlen("NO DATA"), 7u);
  CHECK_EQ(strlen("CAN ERROR"), 9u);

  // "DEC" and "BED" are odd, in range and pure hex -- they ARE frame lines, and
  // there is no reply in the firmware's vocabulary that collides with one.
  // Recorded here so that adding such a reply breaks this test on purpose.
  CHECK_EQ(classify("DEC"), LINK_LINE_FRAME);
  CHECK_EQ(classify("BED"), LINK_LINE_FRAME);

  // Anything with punctuation: the AT@ST and AT@EV shapes in miniature.
  CHECK_EQ(classify("SM=1"), LINK_LINE_REPLY);
  CHECK_EQ(classify("SD=0"), LINK_LINE_REPLY);
  CHECK_EQ(classify("@EV"), LINK_LINE_REPLY);
  CHECK_EQ(classify("v2.1"), LINK_LINE_REPLY);
  CHECK_EQ(classify("002+"), LINK_LINE_REPLY);
  CHECK_EQ(classify("00-2"), LINK_LINE_REPLY);

  // A NUL inside the buffer is not a terminator here: len is authoritative, and
  // a NUL is not a hex digit, so the line is a reply either way.
  CHECK_EQ(linkClassifyLine("00\0" "21", 5u, LINK_LINE_TS_NONE), LINK_LINE_REPLY);
}

// ------------------------------------------- an interleaved stream, end to end
// The shape the host actually has to survive: a command's reply buried in
// background frames. Exactly one line in this burst is a reply.
{
  static const char* const kBurst[] = {
    "002041B0E1E4F264FB8",
    "0142596A9FF1B563A",
    "1A700E700000000",
    "OK",
    "21003C8",
    "0158F8B2BFF0000",
    "11D000000",
  };
  uint8_t frames = 0u;
  uint8_t replies = 0u;
  for (uint8_t i = 0u; i < (uint8_t)(sizeof(kBurst) / sizeof(kBurst[0])); ++i) {
    switch (classify(kBurst[i])) {
      case LINK_LINE_FRAME: ++frames; break;
      case LINK_LINE_REPLY: ++replies; break;
      default: CHECK_MSG(false, "\"%s\" classified EMPTY", kBurst[i]); break;
    }
  }
  CHECK_EQ(frames, 6);
  CHECK_EQ(replies, 1);
}

// ===========================================================================
// Timestamps. Everything below is about the width being CONFIGURATION.
// ===========================================================================

// ------------------------------------- stamped lines are FRAME at their width
// Each real frame with four digits appended, as AT@SM2 would emit it. The
// stamp digits are hex like everything else, so only the length set separates
// these from unstamped lines -- and only if it is given the right width.
{
  char buf[40];
  for (uint8_t i = 0u; i < kFrameCount; ++i) {
    const FrameCase& fc = kFrames[i];
    snprintf(buf, sizeof(buf), "%s4FB8", fc.line);          // 20408 ms
    const uint16_t len = (uint16_t)strlen(buf);
    CHECK_MSG(classify(buf, LINK_LINE_TS_SHORT) == LINK_LINE_FRAME,
              "%s: expected FRAME at width 4", buf);

    // And eight, as AT@SM3 would.
    snprintf(buf, sizeof(buf), "%s0012D687", fc.line);      // 1234567 ms
    CHECK_MSG(classify(buf, LINK_LINE_TS_LONG) == LINK_LINE_FRAME,
              "%s: expected FRAME at width 8", buf);

    // At width 0 the stamped line is read as two MORE DATA BYTES, and that is
    // accepted for every DLC up to 6 -- 3+2n+4 == 3+2(n+2). Only the top two
    // DLCs escape, by pushing the total past the 19-char unstamped bound. That
    // is the ambiguity in the small, swept over every real frame on this bus.
    snprintf(buf, sizeof(buf), "%s4FB8", fc.line);
    const LinkLineKind atZero = classify(buf, LINK_LINE_TS_NONE);
    if (fc.dataLen <= (uint8_t)(CAN_MAX_DLC - 2u)) {
      CHECK_MSG(atZero == LINK_LINE_FRAME,
                "%s: width 0 should read this as %u data bytes", buf,
                (unsigned)(fc.dataLen + 2u));
    } else {
      // The silent drop the classifier change exists to prevent: a stamped
      // 7- or 8-byte frame is 21 or 23 characters, past the unstamped bound, so
      // a host still decoding at width 0 files it as a command reply and never
      // sees the frame at all.
      CHECK_MSG(atZero == LINK_LINE_REPLY,
                "%s: width 0 cannot hold %u data bytes", buf,
                (unsigned)(fc.dataLen + 2u));
      CHECK_MSG(len > LINK_LINE_FRAME_MAX, "%s: %u should be past the unstamped bound",
                buf, (unsigned)len);
    }
  }
}

// --------------------------------------------------- THE AMBIGUITY, asserted
// The reason the width must be configured rather than detected, written out as
// a test so that anyone tempted to sniff it has to delete this first.
{
  const char* const kLine = "002041B0E1E4F264FB8";    // a real 8-byte frame
  CHECK_EQ(strlen(kLine), 19u);

  // Both widths ACCEPT it. Neither is wrong about the shape: 3 + 2*8 and
  // 3 + 2*6 + 4 are the same 19 characters, all hex, odd either way, and there
  // is no DLC nibble on this wire to break the tie.
  CHECK_EQ(classify(kLine, LINK_LINE_TS_NONE), LINK_LINE_FRAME);
  CHECK_EQ(classify(kLine, LINK_LINE_TS_SHORT), LINK_LINE_FRAME);

  // ... and they mean DIFFERENT THINGS.
  Decoded plain;
  Decoded stamped;
  CHECK(decode(kLine, LINK_LINE_TS_NONE, plain));
  CHECK(decode(kLine, LINK_LINE_TS_SHORT, stamped));

  CHECK_EQ(plain.id, 0x002u);
  CHECK_EQ(stamped.id, 0x002u);       // the id is never in doubt: always 3 chars
  CHECK_EQ(plain.len, 8);
  CHECK_EQ(stamped.len, 6);           // two bytes fewer, and they are the time
  CHECK_EQ(plain.data[6], 0x4Fu);
  CHECK_EQ(plain.data[7], 0xB8u);
  CHECK_EQ(plain.stamp, 0u);
  CHECK_EQ(stamped.data[6], 0u);      // not data any more
  CHECK_EQ(stamped.data[7], 0u);
  CHECK_EQ(stamped.stamp, 0x4FB8u);

  // 0x4FB8 is 20408 ms, inside AT@SM2's 0..59999 window, so the wrong reading
  // is not even implausible. 0x002 is the RPM and speed broadcast: read at the
  // wrong width it would put two bytes of a millisecond counter where
  // core/Broadcast.cpp expects vehicle data, or drop the two bytes that carry
  // it. A plausible wrong number on the dashboard is the one failure mode this
  // project is written to avoid, and no amount of looking at this line can
  // tell you which reading is the right one.
  CHECK(stamped.stamp < 60000u);
}

// ----------------------------------------- the accepted set, swept exactly
// Not just "the cap moved": for each width, every length from 1 to 31 must be
// FRAME if and only if some n in 0..CAN_MAX_DLC gives 3 + 2n + W. That rejects
// 21 unstamped, 25 at width 4 and 29 at width 8 -- lengths that are odd, in
// range of the raised cap, and pure hex, so nothing else would catch them.
{
  static const uint8_t kWidths[] = {LINK_LINE_TS_NONE, LINK_LINE_TS_SHORT,
                                    LINK_LINE_TS_LONG};
  char buf[40];
  for (uint8_t w = 0u; w < 3u; ++w) {
    const uint8_t ts = kWidths[w];
    for (uint16_t len = 1u; len <= 31u; ++len) {
      for (uint16_t i = 0u; i < len; ++i) {
        buf[i] = "0123456789ABCDEF"[i & 0x0Fu];
      }
      buf[len] = '\0';
      const LinkLineKind k = linkClassifyLine(buf, len, ts);
      const LinkLineKind want = lengthFits(len, ts) ? LINK_LINE_FRAME : LINK_LINE_REPLY;
      CHECK_MSG(k == want, "width %u length %u: expected %d, got %d", (unsigned)ts,
                (unsigned)len, (int)want, (int)k);
    }
  }

  // The three that the exact set catches and a raised cap would not, spelled
  // out rather than left to the sweep.
  CHECK_EQ(classify("002041B0E1E4F264FB8AB", LINK_LINE_TS_NONE), LINK_LINE_REPLY);   // 21
  CHECK_EQ(strlen("002041B0E1E4F264FB8AB"), 21u);
  CHECK_EQ(classify("002041B0E1E4F264FB84FB8A", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK_EQ(strlen("002041B0E1E4F264FB84FB8A"), 24u);        // even, too
  CHECK_EQ(classify("002041B0E1E4F264FB8AB0012D687", LINK_LINE_TS_LONG), LINK_LINE_REPLY);
  CHECK_EQ(strlen("002041B0E1E4F264FB8AB0012D687"), 29u);
}

// ---------------------------------------------------- the stamped boundaries
{
  // DLC 0 stamped: a bare id plus the time. 3 + 0 + 4 = 7, 3 + 0 + 8 = 11.
  CHECK_EQ(classify("0020000", LINK_LINE_TS_SHORT), LINK_LINE_FRAME);
  CHECK_EQ(classify("002EA5F", LINK_LINE_TS_SHORT), LINK_LINE_FRAME);   // 59999 ms
  CHECK_EQ(classify("002FFFFFFFF", LINK_LINE_TS_LONG), LINK_LINE_FRAME);
  // Below the minimum, which the stamp raises with it: at width 4 a bare id is
  // no longer a frame line, because the adapter would have stamped it.
  CHECK_EQ(classify("002", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK_EQ(classify("002", LINK_LINE_TS_LONG), LINK_LINE_REPLY);
  CHECK_EQ(classify("0020000", LINK_LINE_TS_LONG), LINK_LINE_REPLY);

  // DLC 8 stamped: 23 at width 4, 27 at width 8 -- the longest line the
  // classifier can now accept, more than the 19 it was written for.
  CHECK_EQ(strlen("002041B0E1E4F264FB84FB8"), 23u);
  CHECK_EQ(classify("002041B0E1E4F264FB84FB8", LINK_LINE_TS_SHORT), LINK_LINE_FRAME);
  CHECK_EQ(strlen("002041B0E1E4F264FB80012D687"), 27u);
  CHECK_EQ(classify("002041B0E1E4F264FB80012D687", LINK_LINE_TS_LONG), LINK_LINE_FRAME);

  // 23 characters is ALSO a 6-byte frame with an 8-digit stamp (3+12+8), so the
  // ambiguity is not a width-4 quirk -- every pair of widths differing by 4
  // overlaps somewhere.
  CHECK_EQ(classify("002041B0E1E4F264FB84FB8", LINK_LINE_TS_LONG), LINK_LINE_FRAME);
  Decoded asFour;
  Decoded asEight;
  CHECK(decode("002041B0E1E4F264FB84FB8", LINK_LINE_TS_SHORT, asFour));
  CHECK(decode("002041B0E1E4F264FB84FB8", LINK_LINE_TS_LONG, asEight));
  CHECK_EQ(asFour.len, 8);
  CHECK_EQ(asEight.len, 6);
  CHECK_EQ(asFour.stamp, 0x4FB8u);
  CHECK_EQ(asEight.stamp, 0x4FB84FB8u);

  // 27 at width 4 would be 10 data bytes, which classical CAN cannot carry --
  // the DLC bound still does its job past the stamp, which is why it is
  // derived from CAN_MAX_DLC rather than written as a number.
  CHECK_EQ(classify("002041B0E1E4F264FB80012D687", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK_EQ(classify("002041B0E1E4F264FB80012D687", LINK_LINE_TS_NONE), LINK_LINE_REPLY);
}

// ------------------------------------- non-hex in the stamp is still a REPLY
// The stamp is scanned with the same accept set as the data, deliberately: a
// line whose time will not decode must not become a frame with a garbage time,
// and linkParseFrameLine fails such a line too. A reply that happens to land
// in the widened window is caught by exactly this.
{
  CHECK_EQ(classify("002041B0E1E4F264FB8-FB8", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK_EQ(classify("002041B0E1E4F264FB84FBG", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK_EQ(classify("002041B0E1E4F264FB8 FB8", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK_EQ(strlen("002041B0E1E4F264FB8 FB8"), 23u);

  // Every reply in the firmware's vocabulary, re-checked at both stamped
  // widths. The length test throws away fewer of them now -- "BUFFER FULL" is
  // 11 characters and width 8 accepts 11 -- so the hex scan is carrying more of
  // the load than it used to, and that has to be tested at the widths where it
  // is true rather than only at width 0.
  for (uint8_t i = 0u; i < kReplyCount; ++i) {
    CHECK_MSG(classify(kReplies[i], LINK_LINE_TS_SHORT) == LINK_LINE_REPLY,
              "\"%s\": expected REPLY at width 4", kReplies[i]);
    CHECK_MSG(classify(kReplies[i], LINK_LINE_TS_LONG) == LINK_LINE_REPLY,
              "\"%s\": expected REPLY at width 8", kReplies[i]);
  }
  // Spelled out for the one that the raised ceiling let through to the scan.
  CHECK_EQ(strlen("BUFFER FULL"), 11u);
  CHECK(lengthFits(11u, LINK_LINE_TS_LONG));
  CHECK_EQ(classify("BUFFER FULL", LINK_LINE_TS_LONG), LINK_LINE_REPLY);
}

// ------------------------------- an unmodified session is unchanged, checked
// The user-visible promise: AT@SM1 is still the default level, so width 0 is
// what the reader passes, and every line on this bus classifies exactly as it
// did before the width existed. The suite above is that test; this is the
// statement of it, on the interleaved burst.
{
  static const char* const kBurst[] = {
    "002041B0E1E4F264FB8",
    "OK",
    "21003C8",
    "1C802",
  };
  uint8_t frames = 0u;
  for (uint8_t i = 0u; i < 4u; ++i) {
    if (linkClassifyLine(kBurst[i], (uint16_t)strlen(kBurst[i]),
                         LINK_LINE_TS_NONE) == LINK_LINE_FRAME) {
      ++frames;
    }
  }
  CHECK_EQ(frames, 3);
}

// ===========================================================================
// ACCEPT IFF DECODE -- the coupling that used to be a paragraph
// ===========================================================================
//
// core/LinkLine.h argues at length that linkClassifyLine()'s accepted length
// set and linkParseFrameLine()'s `(dataChars / 2u) > CAN_MAX_DLC` check "cannot
// drift apart", and that both must agree about the timestamp width and about
// what a hex digit is. Until the parser moved into core/ that was PROSE: the
// classifier was host-tested and the parser was an At328Link private in an
// ESP-IDF translation unit, so nothing checked the pair.
//
// Both directions of a disagreement are silent, and both are the failure this
// project is written to avoid. FRAME that will not decode is a broadcast
// dropped -- a reading that simply never arrives, with the reader's `if` eating
// it and no counter anywhere. Decodes-but-classified-REPLY is worse: the line
// goes to replyQ_, so a frame is read as a command's answer and a transaction
// gets someone else's bytes.
//
// So: for all three legal widths, over a generated corpus, FRAME if and only if
// decoded. The corpus is systematic rather than random on purpose -- a 20
// million case fuzz over this found zero disagreements, which is why the job
// here is to KEEP it holding rather than to go looking again, and a sweep that
// runs identically every time is worth more in a suite than a sample that
// varies per run.
{
  // Hex digits in both cases, then the traps. 'G' is the first letter past the
  // set; ':' and '>' sit either side of the digits and of 'F' in ASCII, so a
  // range test written with the wrong comparison catches one of them; ' ' is
  // what ATS1 would leave in a line and the single most reliable mark of a
  // reply; '-' is the separator a LAWICEL-shaped format would use.
  static const char kAlpha[] = "09AFafG: ->";
  static const char kHexAlpha[] = "0123456789ABCDEFabcdef";
  static const uint8_t kWidths[] = {LINK_LINE_TS_NONE, LINK_LINE_TS_SHORT,
                                    LINK_LINE_TS_LONG};
  char buf[40];
  uint32_t cases = 0u;
  uint32_t framesSeen = 0u;      // the sweep must actually reach both answers
  uint32_t repliesSeen = 0u;

  // One case: classify and decode the same (line, len, width) and require the
  // two to agree. Everything the parser hands back is checked here too, so a
  // FRAME that decodes to nine data bytes or leaves a poisoned byte behind
  // fails at the case that produced it rather than somewhere downstream.
  struct Check {
    static void one(const char* line, uint16_t len, uint8_t ts, uint32_t& cases,
                    uint32_t& framesSeen, uint32_t& repliesSeen) {
      const LinkLineKind k = linkClassifyLine(line, len, ts);
      CanFrame f;
      memset(&f, 0xAA, sizeof(f));    // poison: a refused parse must not be
                                      // read, and a short frame must zero-fill
      uint32_t stamp = 0xDEADBEEFu;
      const bool ok = linkParseFrameLine(line, len, ts, f, stamp);
      ++cases;
      CHECK_MSG((k == LINK_LINE_FRAME) == ok,
                "width %u len %u \"%.*s\": classify=%d but parse=%d",
                (unsigned)ts, (unsigned)len, (int)len, line ? line : "(null)",
                (int)k, (int)ok);
      if (!ok) {
        ++repliesSeen;
        return;
      }
      ++framesSeen;
      // The DLC bound, restated on the OUTPUT. This is the assertion that
      // makes the classifier's EXACT length set worth having: nothing that
      // survives to here may claim more data than classical CAN can carry,
      // whatever width it was read at.
      CHECK_MSG(f.len <= CAN_MAX_DLC, "width %u len %u: decoded len %u > DLC",
                (unsigned)ts, (unsigned)len, (unsigned)f.len);
      CHECK_MSG((uint16_t)(3u + 2u * f.len + ts) == len,
                "width %u len %u: %u data bytes does not re-make the length",
                (unsigned)ts, (unsigned)len, (unsigned)f.len);
      CHECK_MSG(f.id <= 0xFFFu, "width %u len %u: id %u is not 3 nibbles",
                (unsigned)ts, (unsigned)len, (unsigned)f.id);
      if (ts == LINK_LINE_TS_NONE) {
        CHECK_MSG(stamp == 0u, "width 0 len %u: stamp %u should be 0",
                  (unsigned)len, (unsigned)stamp);
      }
      for (uint8_t i = f.len; i < CAN_MAX_DLC; ++i) {
        CHECK_MSG(f.data[i] == 0u, "width %u len %u: byte %u not zero-filled",
                  (unsigned)ts, (unsigned)len, (unsigned)i);
      }
      // And the independent mirror above must get the same answer. If it ever
      // does not, one of the two moved and the ambiguity evidence earlier in
      // this file has quietly stopped describing the shipping parser.
      Decoded m;
      CHECK_MSG(decode(line, ts, m),
                "width %u len %u: mirror refused what the parser accepted",
                (unsigned)ts, (unsigned)len);
      CHECK_EQ(m.id, f.id);
      CHECK_EQ(m.len, f.len);
      CHECK_EQ(m.stamp, stamp);
      CHECK_EQ(memcmp(m.data, f.data, CAN_MAX_DLC), 0);
    }
  };

  for (uint8_t w = 0u; w < 3u; ++w) {
    const uint8_t ts = kWidths[w];

    // (a) PURE HEX at every length either side of the accepted set. 1..31
    // covers the shortest line the reader can hand over through the longest
    // any width accepts (27) and four past it, so both bounds are crossed.
    for (uint16_t len = 1u; len <= 31u; ++len) {
      for (uint16_t i = 0u; i < len; ++i) {
        buf[i] = kHexAlpha[i % 22u];          // both cases, cycling
      }
      buf[len] = '\0';
      Check::one(buf, len, ts, cases, framesSeen, repliesSeen);
    }

    // (b) ONE TRAP CHARACTER, at every position of every length in range.
    // This is the half only the two implementations agreeing can settle: a
    // line rejected purely on its length is easy, while a line of exactly the
    // right shape carrying one bad nibble has to be refused by BOTH -- and the
    // interesting positions are the id (0..2), the data, and the one a looser
    // parser would skip, the stamp at the very end.
    for (uint16_t len = 3u; len <= 27u; ++len) {
      for (uint8_t t = 0u; kAlpha[t] != '\0'; ++t) {
        for (uint16_t pos = 0u; pos < len; ++pos) {
          for (uint16_t i = 0u; i < len; ++i) {
            buf[i] = kHexAlpha[i % 22u];
          }
          buf[pos] = kAlpha[t];
          buf[len] = '\0';
          Check::one(buf, len, ts, cases, framesSeen, repliesSeen);
        }
      }
    }

    // (c) The real frames off this vehicle, bare and with each stamp width
    // appended, read at THIS width -- so every fixture is put to all three
    // widths and the ambiguous readings are included rather than avoided.
    for (uint8_t i = 0u; i < kFrameCount; ++i) {
      Check::one(kFrames[i].line, (uint16_t)strlen(kFrames[i].line), ts, cases,
                 framesSeen, repliesSeen);
      snprintf(buf, sizeof(buf), "%s4FB8", kFrames[i].line);
      Check::one(buf, (uint16_t)strlen(buf), ts, cases, framesSeen,
                 repliesSeen);
      snprintf(buf, sizeof(buf), "%s0012D687", kFrames[i].line);
      Check::one(buf, (uint16_t)strlen(buf), ts, cases, framesSeen,
                 repliesSeen);
    }

    // (d) The firmware's whole reply vocabulary. Not one of these may decode at
    // any width -- the length test throws away fewer of them than it used to
    // ("BUFFER FULL" is 11 characters and width 8 accepts 11), so this is the
    // hex scan being asked the same question on both sides.
    for (uint8_t i = 0u; i < kReplyCount; ++i) {
      Check::one(kReplies[i], (uint16_t)strlen(kReplies[i]), ts, cases,
                 framesSeen, repliesSeen);
    }

    // (e) The degenerate inputs, which the two halves answer with DIFFERENT
    // words -- EMPTY is not FRAME, false is not true -- and must therefore
    // still agree. A null line carrying a plausible length is the one that
    // would crash rather than misfile.
    Check::one(0, 0u, ts, cases, framesSeen, repliesSeen);
    Check::one(0, 19u, ts, cases, framesSeen, repliesSeen);
    Check::one("", 0u, ts, cases, framesSeen, repliesSeen);
    Check::one("002041B0E1E4F264FB8", 0u, ts, cases, framesSeen, repliesSeen);
  }

  // The corpus size, pinned. A sweep that silently stopped generating would
  // otherwise "pass" -- and this is exactly the shape of test that fails that
  // way, since every assertion lives inside the loops. 4217 lines per width:
  // 31 pure-hex lengths, 4125 one-trap variants (11 traps over every position
  // of lengths 3..27), 45 real-frame readings, 14 replies and 4 degenerates.
  CHECK_EQ(cases, 3u * 4219u);
  // Both answers really occur, and not a handful of each. Without this the
  // whole sweep would be satisfied by a pair that rejected everything.
  CHECK(framesSeen > 200u);
  CHECK(repliesSeen > 200u);
  CHECK_EQ(framesSeen + repliesSeen, cases);
}


// ===========================================================================
// LAWICEL/SLCAN -- the second frame shape, which describes itself
// ===========================================================================

// ------------------------------------------------- the macros ARE the format
{
  // 't' and not 'T', and that is a fact about the gateway rather than a
  // convention: in LAWICEL the lowercase marker is an 11-bit standard data
  // frame and the uppercase one is the 29-bit extended frame, which a uint16_t
  // id has nowhere to put. The marker being a LETTER is what lets the STREAM
  // test be one character compare ahead of any counting -- it cannot collide
  // with a hex digit, so a frame line and a reply are told apart before either
  // is measured.
  CHECK_EQ(LINK_LINE_SLCAN_MARK, 't');
  CHECK(nibble(LINK_LINE_SLCAN_MARK) < 0);

  // 5 is marker + 3 id nibbles + the DLC nibble, which is a COMPLETE line for a
  // zero-length frame; 29 is the same head plus 8 data bytes and the widest
  // stamp. Both are derived from CAN_MAX_DLC and the stamp width in the header
  // rather than written as numbers, so the ends of the range cannot drift from
  // the DLC bound the parsers enforce -- and this pair of checks is what would
  // notice one of them being edited into a literal.
  CHECK_EQ(LINK_LINE_SLCAN_MIN, 5);
  CHECK_EQ(LINK_LINE_SLCAN_MAX, 29);
  CHECK_EQ(LINK_LINE_SLCAN_MAX,
           LINK_LINE_SLCAN_MIN + 2u * CAN_MAX_DLC + LINK_LINE_TS_LONG);

  // The LAWICEL window sits past the ELM one at both ends (5 vs 3, 29 vs 27),
  // so neither shape is contained in the other. Nothing depends on that -- the
  // marker decides, not the length -- but if it were ever otherwise, a reader
  // tempted to classify by length alone would look right for a while.
  CHECK(LINK_LINE_SLCAN_MIN > LINK_LINE_FRAME_MIN);
  CHECK(LINK_LINE_SLCAN_MAX > LINK_LINE_FRAME_MAX + LINK_LINE_TS_LONG);

  // FOUR kinds now, all distinct, and STREAM is a NEW answer rather than a
  // relabelled FRAME. A reader that switch()es on this and never grew a STREAM
  // case would send every background frame to its default branch, so the two
  // must not be confusable by value either.
  CHECK_EQ(LINK_LINE_EMPTY, 0);
  CHECK(LINK_LINE_STREAM != LINK_LINE_EMPTY);
  CHECK(LINK_LINE_STREAM != LINK_LINE_FRAME);
  CHECK(LINK_LINE_STREAM != LINK_LINE_REPLY);
  CHECK(LINK_LINE_FRAME != LINK_LINE_REPLY);
}

// ------------------------------------------------------ LAWICEL fixture audit
// Before any captured line is used as evidence: the marker, pure hex after it,
// and the length its own DLC nibble implies. The whole claim of this format is
// that the line says how it splits, so a fixture with one stamp digit miscopied
// would be asserting the opposite of what the table says it asserts.
{
  for (uint8_t i = 0u; i < kStreamCount; ++i) {
    const StreamCase& sc = kStreams[i];
    const uint16_t len = (uint16_t)strlen(sc.line);
    CHECK_MSG(sc.line[0] == LINK_LINE_SLCAN_MARK, "%s: does not start with the marker",
              sc.line);
    for (uint16_t k = 1u; k < len; ++k) {
      CHECK_MSG(auditHex(sc.line[k]), "%s: char %u (%c) is not a hex digit", sc.line,
                (unsigned)k, sc.line[k]);
    }
    CHECK_MSG(sc.dlc <= CAN_MAX_DLC, "%s: DLC %u exceeds the bound", sc.line,
              (unsigned)sc.dlc);
    CHECK_MSG(nibble(sc.line[4]) == (int)sc.dlc, "%s: the DLC nibble is not %u", sc.line,
              (unsigned)sc.dlc);
    CHECK_MSG(len == (uint16_t)(5u + 2u * sc.dlc + sc.ts),
              "%s: length %u is not 5 + 2*%u + %u", sc.line, (unsigned)len,
              (unsigned)sc.dlc, (unsigned)sc.ts);
    CHECK_MSG(sc.ts == LINK_LINE_TS_NONE || sc.ts == LINK_LINE_TS_SHORT ||
                  sc.ts == LINK_LINE_TS_LONG,
              "%s: %u is not a width the firmware emits", sc.line, (unsigned)sc.ts);
    CHECK_MSG(slcanLengthPossible(len), "%s: length %u is outside the accepted set",
              sc.line, (unsigned)len);
    // A 4-digit stamp is millis() % 60000, so a fixture claiming more than
    // 59999 ms in four digits would be a line no gateway at AT@SML2 could have
    // emitted -- which would make it a fixture about nothing.
    if (sc.ts == LINK_LINE_TS_SHORT) {
      CHECK_MSG(sc.stamp < 60000u, "%s: stamp %u is past the AT@SML2 window", sc.line,
                (unsigned)sc.stamp);
    }
  }
  CHECK_EQ(kStreamCount, 7);
  CHECK_EQ(kSweepCount, 7);
}

// --------------------------------------- real captured lines decode correctly
// The verdict is the easy half. This asserts the WHOLE reading: id, DLC, every
// data byte, the zero fill past it, the stamp, and the width -- which nobody
// told the parser. core/Broadcast.cpp reads these payloads positionally (byte 2
// of 0x1A7 is fuel flow), so a decoder off by one byte still returns true, still
// re-makes the length, and puts a wrong number on the dashboard rather than no
// number at all.
{
  for (uint8_t i = 0u; i < kStreamCount; ++i) {
    const StreamCase& sc = kStreams[i];
    const uint16_t len = (uint16_t)strlen(sc.line);
    const LinkLineKind k = classify(sc.line);
    CHECK_MSG(k == LINK_LINE_STREAM, "%s: expected STREAM, got %d", sc.line, (int)k);

    CanFrame f;
    memset(&f, 0xAAu, sizeof(f));     // poison: the zero fill has to be done by
                                      // the parser, not inherited from a caller
    uint32_t stamp = 0xDEADBEEFu;
    uint8_t ts = 0xFFu;
    CHECK_MSG(linkParseStreamLine(sc.line, len, f, stamp, ts),
              "%s: the parser refused a captured line", sc.line);
    CHECK_MSG(f.id == sc.id, "%s: id %03X, expected %03X", sc.line, (unsigned)f.id,
              (unsigned)sc.id);
    CHECK_MSG(f.len == sc.dlc, "%s: len %u, expected %u", sc.line, (unsigned)f.len,
              (unsigned)sc.dlc);
    CHECK_MSG(ts == sc.ts, "%s: derived width %u, expected %u", sc.line, (unsigned)ts,
              (unsigned)sc.ts);
    CHECK_MSG(stamp == sc.stamp, "%s: stamp %08X, expected %08X", sc.line,
              (unsigned)stamp, (unsigned)sc.stamp);
    for (uint8_t b = 0u; b < CAN_MAX_DLC; ++b) {
      CHECK_MSG(f.data[b] == sc.data[b], "%s: byte %u is %02X, expected %02X", sc.line,
                (unsigned)b, (unsigned)f.data[b], (unsigned)sc.data[b]);
    }
  }
}

// ---------------------------------------------- both ends of the length range
{
  CanFrame f;
  uint32_t stamp = 0u;
  uint8_t ts = 0xFFu;

  // THE MINIMUM: a line with nothing but a head -- marker, id, and a DLC of 0.
  // A zero-length frame is legal CAN and the gateway prints exactly this. It is
  // also the one line where the derivation has nothing to measure: the
  // remainder after no data at all is 0, which is a width.
  CHECK_EQ(strlen("t1A70"), 5u);
  CHECK_EQ(strlen("t1A70"), LINK_LINE_SLCAN_MIN);
  CHECK_EQ(classify("t1A70"), LINK_LINE_STREAM);
  memset(&f, 0xAAu, sizeof(f));
  CHECK(linkParseStreamLine("t1A70", 5u, f, stamp, ts));
  CHECK_EQ(f.id, 0x1A7u);
  CHECK_EQ(f.len, 0);
  CHECK_EQ(ts, LINK_LINE_TS_NONE);
  CHECK_EQ(stamp, 0u);
  for (uint8_t b = 0u; b < CAN_MAX_DLC; ++b) {
    CHECK_MSG(f.data[b] == 0u, "t1A70: byte %u not zero-filled", (unsigned)b);
  }
  // One character short of the minimum is a REPLY, not a frame with its DLC
  // read off the end of the line.
  CHECK_EQ(classify("t1A7"), LINK_LINE_REPLY);
  CHECK(!linkParseStreamLine("t1A7", 4u, f, stamp, ts));

  // DLC 0 STAMPED, at both widths: 9 and 13 characters. These are the lines
  // where the stamp is the only thing after the head, so the arithmetic is
  // doing all of the work and an off-by-one in it has nothing to hide behind.
  CHECK_EQ(strlen("t1A707816"), 9u);
  CHECK_EQ(classify("t1A707816"), LINK_LINE_STREAM);
  CHECK(linkParseStreamLine("t1A707816", 9u, f, stamp, ts));
  CHECK_EQ(f.len, 0);
  CHECK_EQ(ts, LINK_LINE_TS_SHORT);
  CHECK_EQ(stamp, 0x7816u);
  CHECK_EQ(strlen("t1A700012D687"), 13u);
  CHECK_EQ(classify("t1A700012D687"), LINK_LINE_STREAM);
  CHECK(linkParseStreamLine("t1A700012D687", 13u, f, stamp, ts));
  CHECK_EQ(f.len, 0);
  CHECK_EQ(ts, LINK_LINE_TS_LONG);
  CHECK_EQ(stamp, 0x0012D687u);      // 1234567 ms, raw millis() at AT@SML3

  // THE MAXIMUM: 8 data bytes and an 8-digit stamp. 29 characters, which is
  // what AT328_LINE_MAX has to clear, and the longest line either shape can put
  // on this wire.
  CHECK_EQ(strlen("t2C08FFFF170B7DFF17020012D687"), 29u);
  CHECK_EQ(strlen("t2C08FFFF170B7DFF17020012D687"), LINK_LINE_SLCAN_MAX);
  CHECK_EQ(classify("t2C08FFFF170B7DFF17020012D687"), LINK_LINE_STREAM);
  memset(&f, 0xAAu, sizeof(f));
  CHECK(linkParseStreamLine("t2C08FFFF170B7DFF17020012D687", 29u, f, stamp, ts));
  CHECK_EQ(f.id, 0x2C0u);
  CHECK_EQ(f.len, 8);
  CHECK_EQ(ts, LINK_LINE_TS_LONG);
  CHECK_EQ(stamp, 0x0012D687u);
  CHECK_EQ(f.data[0], 0xFFu);
  CHECK_EQ(f.data[7], 0x02u);

  // ... and 29 is GENUINELY the maximum. Thirty characters -- the same line
  // with one more hex digit -- is refused at every width the caller might pass,
  // and by the parser too. Two rules say no here (the length bound first, the
  // impossible remainder of 9 behind it) and that is on purpose: the bound is
  // what protects the buffer, the remainder is what protects the reading.
  CHECK_EQ(strlen("t2C08FFFF170B7DFF17020012D6877"), 30u);
  CHECK_EQ(classify("t2C08FFFF170B7DFF17020012D6877", LINK_LINE_TS_NONE),
           LINK_LINE_REPLY);
  CHECK_EQ(classify("t2C08FFFF170B7DFF17020012D6877", LINK_LINE_TS_SHORT),
           LINK_LINE_REPLY);
  CHECK_EQ(classify("t2C08FFFF170B7DFF17020012D6877", LINK_LINE_TS_LONG),
           LINK_LINE_REPLY);
  CHECK(!linkParseStreamLine("t2C08FFFF170B7DFF17020012D6877", 30u, f, stamp, ts));
  CHECK(!slcanLengthPossible(30u));

  // The widths are not merely accepted, they are DISTINGUISHED: the same
  // 8-byte frame at all three widths is 21, 25 and 29 characters, and each
  // reports its own width back.
  CHECK_EQ(strlen("t2C08FFFF170B7DFF1702"), 21u);
  CHECK(linkParseStreamLine("t2C08FFFF170B7DFF1702", 21u, f, stamp, ts));
  CHECK_EQ(ts, LINK_LINE_TS_NONE);
  CHECK_EQ(f.len, 8);
  CHECK_EQ(stamp, 0u);
  CHECK(linkParseStreamLine("t2C08FFFF170B7DFF17027825", 25u, f, stamp, ts));
  CHECK_EQ(ts, LINK_LINE_TS_SHORT);
  CHECK_EQ(f.len, 8);
  CHECK_EQ(stamp, 0x7825u);
}

// ------------------------------------- the ambiguity, MOVED TO THE SENDER
// The ELM shape has its ambiguity asserted further up: 19 characters that are
// two different frames depending on a width the reader has to be told. The
// LAWICEL shape does not delete that arithmetic -- it moves the decision into
// the line.
//
// These two differ in ONE character, the DLC nibble, and are the same 25
// characters otherwise. Both are legal. They decode to different data AND to a
// different time, and the wrong reading is not implausible: 0x17027825 is about
// 4.5 days of uptime, exactly what AT@SML3 prints. The difference from the ELM
// case is that nobody downstream has to CHOOSE -- the sender chose, and said so
// in the fifth character.
{
  CanFrame asEight;
  CanFrame asSix;
  uint32_t stampEight = 0u;
  uint32_t stampSix = 0u;
  uint8_t tsEight = 0xFFu;
  uint8_t tsSix = 0xFFu;
  CHECK_EQ(strlen("t2C08FFFF170B7DFF17027825"), 25u);
  CHECK_EQ(strlen("t2C06FFFF170B7DFF17027825"), 25u);

  CHECK_EQ(classify("t2C08FFFF170B7DFF17027825"), LINK_LINE_STREAM);
  CHECK_EQ(classify("t2C06FFFF170B7DFF17027825"), LINK_LINE_STREAM);
  CHECK(linkParseStreamLine("t2C08FFFF170B7DFF17027825", 25u, asEight, stampEight,
                            tsEight));
  CHECK(linkParseStreamLine("t2C06FFFF170B7DFF17027825", 25u, asSix, stampSix, tsSix));

  CHECK_EQ(asEight.id, 0x2C0u);
  CHECK_EQ(asSix.id, 0x2C0u);          // the id is never in doubt: always 3 nibbles
  CHECK_EQ(asEight.len, 8);
  CHECK_EQ(tsEight, LINK_LINE_TS_SHORT);
  CHECK_EQ(stampEight, 0x7825u);       // 30757 ms, inside the 60 s window
  CHECK_EQ(asSix.len, 6);              // two bytes fewer, and they are the time
  CHECK_EQ(tsSix, LINK_LINE_TS_LONG);
  CHECK_EQ(stampSix, 0x17027825u);
  CHECK_EQ(asEight.data[6], 0x17u);
  CHECK_EQ(asEight.data[7], 0x02u);
  CHECK_EQ(asSix.data[6], 0u);         // not data any more
  CHECK_EQ(asSix.data[7], 0u);

  // And the REPORTED widths differ, which is the part the reader acts on: it
  // compares this against the level it asked the gateway for and complains
  // about a mismatch, instead of quietly showing whichever reading it happened
  // to be configured to expect.
  CHECK(tsEight != tsSix);
}

// ------------------------------------------------------------- THE SPLICE
// The case that motivated deriving the width rather than being told it, and the
// only one here taken from a capture of the gateway misbehaving rather than
// working. A terminal was echoing frames back at it, and the capture contains
//
//   t4148FD153Ft0147259C8DFF192858443A
//
// -- a frame line cut short with the start of another inside it. Had the width
// been configuration the reader would have had a plausible reading available
// for pieces of this: any run of hex of the right parity is a frame, and the
// stamp would have been built out of somebody else's data bytes and handed to
// core/StreamClock.h as a time. Three independent rules refuse it instead, and
// the fragments it decomposes into are refused too.
{
  CanFrame f;
  uint32_t stamp = 0u;
  uint8_t ts = 0xFFu;

  // The spliced line itself: 34 characters, five past the longest line the
  // format can produce, so the length bound alone disposes of it -- and the
  // embedded marker is not a hex digit, so the character scan would have as
  // well. Being refused twice is the intent, not a redundancy: the length test
  // protects the buffer and the scan protects the reading.
  CHECK_EQ(strlen("t4148FD153Ft0147259C8DFF192858443A"), 34u);
  CHECK_EQ(classify("t4148FD153Ft0147259C8DFF192858443A", LINK_LINE_TS_NONE),
           LINK_LINE_REPLY);
  CHECK_EQ(classify("t4148FD153Ft0147259C8DFF192858443A", LINK_LINE_TS_SHORT),
           LINK_LINE_REPLY);
  CHECK_EQ(classify("t4148FD153Ft0147259C8DFF192858443A", LINK_LINE_TS_LONG),
           LINK_LINE_REPLY);
  CHECK(!linkParseStreamLine("t4148FD153Ft0147259C8DFF192858443A", 34u, f, stamp, ts));
  CHECK(!slcanLengthPossible(34u));

  // The first fragment ALONE -- what a reader that split the line on the inner
  // marker would be holding. Eleven characters, pure hex, in range, and still
  // refused: the DLC says eight data bytes and only three arrived. This is the
  // check that has no equivalent in the ELM shape, where a truncated line is
  // simply a shorter frame.
  CHECK_EQ(strlen("t4148FD153F"), 11u);
  CHECK(slcanLengthPossible(11u));          // the LENGTH is fine; the line is not
  CHECK_EQ(classify("t4148FD153F"), LINK_LINE_REPLY);
  CHECK(!linkParseStreamLine("t4148FD153F", 11u, f, stamp, ts));

  // The second fragment, the one that started inside the first: no marker at
  // all by the time it is on its own, so it is judged as an ELM line, and at
  // 22 characters it is too long and even to be one of those either.
  CHECK_EQ(strlen("0147259C8DFF192858443A"), 22u);
  CHECK_EQ(classify("0147259C8DFF192858443A", LINK_LINE_TS_NONE), LINK_LINE_REPLY);
  CHECK_EQ(classify("0147259C8DFF192858443A", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK(!linkParseStreamLine("0147259C8DFF192858443A", 22u, f, stamp, ts));

  // The other corruptions from that same capture, each a different way for a
  // line to arrive wrong and each refused by a different rule.
  //
  // A frame truncated by a reboot banner: the gateway reset mid-line and the
  // identity string landed on the end of it. Starts with the marker, so it is
  // judged purely as a LAWICEL candidate -- and it is 41 characters with
  // letters past 'F' and a space in it.
  CHECK_EQ(classify("t0028000000000000000087latonita-jeep v2.1"), LINK_LINE_REPLY);
  CHECK(!linkParseStreamLine("t0028000000000000000087latonita-jeep v2.1",
                             (uint16_t)strlen("t0028000000000000000087latonita-jeep v2.1"),
                             f, stamp, ts));

  // Line noise with a frame welded to the end of it. It does NOT start with the
  // marker, so the LAWICEL branch never sees it -- the embedded 't' cannot
  // rescue it, and must not: the classifier looks at position 0 and nowhere
  // else, which is what keeps the decision one compare.
  CHECK_EQ(classify("LN&5t210203C847BE"), LINK_LINE_REPLY);
  CHECK_EQ(classify("LN&5t210203C847BE", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK(!linkParseStreamLine("LN&5t210203C847BE", 17u, f, stamp, ts));

  // The tail of a frame whose head was lost. Six characters, pure hex, and the
  // ELM rules refuse it for being EVEN -- the oldest rule in this file, still
  // the one that catches a line cut at a byte boundary.
  CHECK_EQ(strlen("007C2D"), 6u);
  CHECK_EQ(classify("007C2D"), LINK_LINE_REPLY);
  CHECK_EQ(classify("007C2D", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK_EQ(classify("007C2D", LINK_LINE_TS_LONG), LINK_LINE_REPLY);
  CHECK(!linkParseStreamLine("007C2D", 6u, f, stamp, ts));

  // The same thing the other way round: a tail of digits with a whole stream
  // line spliced onto it. Pure hex apart from the marker in the middle, 33
  // characters, and judged as an ELM line because that is what its first
  // character makes it -- so the frame inside is LOST, and that is the correct
  // outcome. Half a frame decoded is a wrong number; a dropped line is a line
  // the next broadcast replaces 50 ms later.
  CHECK_EQ(strlen("333639000000t1B260002000000007C32"), 33u);
  CHECK_EQ(classify("333639000000t1B260002000000007C32"), LINK_LINE_REPLY);
  CHECK_EQ(classify("333639000000t1B260002000000007C32", LINK_LINE_TS_SHORT),
           LINK_LINE_REPLY);
  CHECK_EQ(classify("333639000000t1B260002000000007C32", LINK_LINE_TS_LONG),
           LINK_LINE_REPLY);
  CHECK(!linkParseStreamLine("333639000000t1B260002000000007C32", 33u, f, stamp, ts));
}

// ------------------------------------------------ THE TWO SHAPES COEXIST
// Under AT@SML both shapes arrive interleaved on one wire, so the routing has
// to keep them apart in both directions. An ISO-TP response filed as a
// broadcast is a transaction that never returns; a broadcast filed as a
// response hands a transaction someone else's bytes.
{
  CanFrame f;
  uint32_t stamp = 0xDEADBEEFu;
  uint8_t ts = 0xFFu;

  // The ELM-shaped RESPONSE from the worked example in core/LinkLine.h: odd,
  // pure hex, unstamped, no marker. Under AT@SML this is the ONLY ELM-shaped
  // line left on the wire, and it must still come out FRAME -- not STREAM, and
  // not REPLY.
  CHECK_EQ(strlen("5163B00E2E2E2E1FF"), 17u);
  CHECK_EQ(classify("5163B00E2E2E2E1FF", LINK_LINE_TS_NONE), LINK_LINE_FRAME);
  CHECK(classify("5163B00E2E2E2E1FF", LINK_LINE_TS_NONE) != LINK_LINE_STREAM);
  CHECK(!linkParseStreamLine("5163B00E2E2E2E1FF", 17u, f, stamp, ts));
  CHECK(linkParseFrameLine("5163B00E2E2E2E1FF", 17u, LINK_LINE_TS_NONE, f, stamp));
  CHECK_EQ(f.id, 0x516u);
  CHECK_EQ(f.len, 7);
  CHECK_EQ(f.data[0], 0x3Bu);
  CHECK_EQ(stamp, 0u);            // a response is never stamped, at any level

  // The SAME characters with a marker in front are judged PURELY as a LAWICEL
  // candidate, and fail: the fifth character of the result is a '3', so the DLC
  // says three data bytes, and the remainder after them is 7 -- a width no
  // level produces.
  // There is no second go under the ELM rules, and that is the point. Prefixing
  // a good frame line with 't' TAKES ITS READING AWAY rather than adding one,
  // which is exactly what should happen to a line that was spliced into.
  CHECK_EQ(strlen("t5163B00E2E2E2E1FF"), 18u);
  CHECK_EQ(classify("t5163B00E2E2E2E1FF", LINK_LINE_TS_NONE), LINK_LINE_REPLY);
  CHECK_EQ(classify("t5163B00E2E2E2E1FF", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK_EQ(classify("t5163B00E2E2E2E1FF", LINK_LINE_TS_LONG), LINK_LINE_REPLY);
  CHECK(!linkParseStreamLine("t5163B00E2E2E2E1FF", 18u, f, stamp, ts));

  // The same demonstration on a real broadcast line from the ELM capture: a
  // FRAME on its own, a REPLY the moment a marker is in front of it, at every
  // width.
  CHECK_EQ(classify("0142596A9FF1B563A", LINK_LINE_TS_NONE), LINK_LINE_FRAME);
  CHECK_EQ(classify("t0142596A9FF1B563A", LINK_LINE_TS_NONE), LINK_LINE_REPLY);
  CHECK_EQ(classify("t0142596A9FF1B563A", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK_EQ(classify("t0142596A9FF1B563A", LINK_LINE_TS_LONG), LINK_LINE_REPLY);

  // A LAWICEL line classifies STREAM whatever tsChars the caller passes. This
  // is worth its own case because it is the property that lets the reader hold
  // ONE width for the ELM path -- 0 under AT@SML, a real width on the AT@SM
  // fallback -- without that choice reaching the stream lines at all. If the
  // LAWICEL branch ever consulted the argument, a reader on the fallback path
  // would start losing background frames the moment it set a width.
  {
    static const uint8_t kAnyWidth[] = {LINK_LINE_TS_NONE, LINK_LINE_TS_SHORT,
                                        LINK_LINE_TS_LONG};
    for (uint8_t i = 0u; i < kStreamCount; ++i) {
      const uint16_t len = (uint16_t)strlen(kStreams[i].line);
      for (uint8_t w = 0u; w < 3u; ++w) {
        const LinkLineKind k = linkClassifyLine(kStreams[i].line, len, kAnyWidth[w]);
        CHECK_MSG(k == LINK_LINE_STREAM, "%s: expected STREAM with tsChars=%u, got %d",
                  kStreams[i].line, (unsigned)kAnyWidth[w], (int)k);
      }
    }
    // And the decode is identical whatever the caller believes, because it is
    // never asked: the parser takes no width at all.
    CanFrame a;
    CanFrame b;
    memset(&a, 0, sizeof(a));      // zeroed whole, padding included, so the
    memset(&b, 0, sizeof(b));      // memcmp below compares only what was parsed
    uint32_t sa = 0u;
    uint32_t sb = 0u;
    uint8_t ta = 0u;
    uint8_t tb = 0u;
    CHECK(linkParseStreamLine(kStreams[0].line, (uint16_t)strlen(kStreams[0].line), a,
                              sa, ta));
    CHECK(linkParseStreamLine(kStreams[0].line, (uint16_t)strlen(kStreams[0].line), b,
                              sb, tb));
    CHECK_EQ(memcmp(&a, &b, sizeof(a)), 0);
    CHECK_EQ(sa, sb);
    CHECK_EQ(ta, tb);
  }

  // The burst the reader actually has to survive: background LAWICEL frames,
  // one ELM-shaped response to a request, and replies, all on one wire and all
  // classified with the single width the reader holds for the ELM path.
  {
    static const char* const kMixed[] = {
      "t002800000000000000007819",   // stream: 002, the RPM and speed broadcast
      "t1A76005E000000005A30",       // stream: 1A7, fuel flow
      "5163B00E2E2E2E1FF",           // response to a request -- ELM, unstamped
      "OK",                          // reply
      "t1C81025A51",                 // stream: 1C8, exterior lights
      "AT@EV CAN 0",                 // reply: an asynchronous event
      "t0147259C23FF18CE597816",     // stream: 014, the odometer
    };
    uint8_t streams = 0u;
    uint8_t frames = 0u;
    uint8_t replies = 0u;
    for (uint8_t i = 0u; i < (uint8_t)(sizeof(kMixed) / sizeof(kMixed[0])); ++i) {
      switch (classify(kMixed[i], LINK_LINE_TS_NONE)) {
        case LINK_LINE_STREAM: ++streams; break;
        case LINK_LINE_FRAME:  ++frames;  break;
        case LINK_LINE_REPLY:  ++replies; break;
        default: CHECK_MSG(false, "%s classified EMPTY", kMixed[i]); break;
      }
    }
    CHECK_EQ(streams, 4);
    CHECK_EQ(frames, 1);
    CHECK_EQ(replies, 2);
  }
}

// ------------------------- the reply vocabulary, against the STREAM branch too
// Every reply the firmware can put on the wire, re-checked now that a leading
// 't' is a decision rather than a character. None of them starts with one --
// the banner starts with 'l', the events with 'A' or '@', the status line with
// 'F' -- so all of them still reach the ELM rules and all of them are still
// REPLY, and linkParseStreamLine refuses every one. If a reply were ever added
// that began with 't', this is the test that would fail, and it would fail
// loudly rather than costing a reading.
{
  CanFrame f;
  uint32_t stamp = 0xDEADBEEFu;
  uint8_t ts = 0xFFu;
  static const uint8_t kAnyWidth[] = {LINK_LINE_TS_NONE, LINK_LINE_TS_SHORT,
                                      LINK_LINE_TS_LONG};
  for (uint8_t i = 0u; i < kReplyCount; ++i) {
    const uint16_t len = (uint16_t)strlen(kReplies[i]);
    CHECK_MSG(kReplies[i][0] != LINK_LINE_SLCAN_MARK,
              "\"%s\": a reply starting with the stream marker", kReplies[i]);
    for (uint8_t w = 0u; w < 3u; ++w) {
      const LinkLineKind k = linkClassifyLine(kReplies[i], len, kAnyWidth[w]);
      CHECK_MSG(k == LINK_LINE_REPLY, "\"%s\": expected REPLY at width %u, got %d",
                kReplies[i], (unsigned)kAnyWidth[w], (int)k);
    }
    CHECK_MSG(!linkParseStreamLine(kReplies[i], len, f, stamp, ts),
              "\"%s\": the stream parser decoded a reply", kReplies[i]);
  }
  // The 2.1 status line is the long one, and the one that grew a field: SM= and
  // SML= are the two stream formats, which are ONE field in the gateway and
  // therefore mutually exclusive -- SM=0 SML=2 is what a session in the mode
  // this file is about reports. Both forms are in kReplies above, because a
  // host has to survive an adapter of either vintage.
  CHECK_EQ(classify("FW=2.1 MODE=01 AUX=0 BT=0 BENCH=0 CAN=OK RX=1024 TX=7 ER=0 "
                    "SH=7E0 CF=000 CM=000 F=VTB UP=93 RAM=612 RST=01 REI=0 SM=0 "
                    "SML=2 SD=0"),
           LINK_LINE_REPLY);
  CHECK_EQ(classify("AT@EV CAN 0"), LINK_LINE_REPLY);
  CHECK_EQ(classify("AT@EV CAN 0", LINK_LINE_TS_SHORT), LINK_LINE_REPLY);
  CHECK_EQ(classify(">"), LINK_LINE_REPLY);        // never reaches here, but one
                                                   // character cannot be a frame
  CHECK_EQ(kReplyCount, 14);
}

// ===========================================================================
// ACCEPT IFF DECODE, the LAWICEL half
// ===========================================================================
//
// The same coupling as the sweep above, over the other shape, and it is the
// reason this file exists rather than a list of examples. linkClassifyLine()
// decides which queue a line goes in and linkParseStreamLine() decides what it
// means, they share slcanShape() so that they cannot disagree, and this is what
// says they do not. Both directions of a disagreement are silent: STREAM that
// will not decode is a broadcast dropped with nothing counting it, and
// decodes-but-not-STREAM is worse, since the line goes to the reply queue and a
// command gets handed a frame.
//
// The equivalence is asserted as `classify == STREAM iff the parser accepts`,
// at ALL THREE widths per case, which also settles the other property this
// format is supposed to have: the LAWICEL branch ignores the tsChars argument
// entirely, so all three must give the same verdict on every line in the
// corpus, good or bad.
//
// TWO THINGS ARE SWEPT, and the rejects are the larger half on purpose -- on
// this wire a rejected line is the common case, because the gateway echoes,
// reboots and splices, and every one of those has to land somewhere safe.
//
// One caveat, stated because it is a real limit of the claim below rather than
// an oversight: a trap character at POSITION 0 destroys the marker, and the
// line is then judged by the ELM rules, where it may legitimately be FRAME. The
// equivalence still holds there (FRAME is not STREAM and the stream parser
// refuses it), and the stronger `must be REPLY` claim is made only where a
// non-hex character is involved -- which is every position including 0, since a
// line beginning with a non-hex character is not an ELM frame either.
{
  static const uint8_t kAskWidths[] = {LINK_LINE_TS_NONE, LINK_LINE_TS_SHORT,
                                       LINK_LINE_TS_LONG};
  // The same traps the ELM sweep uses, for the same reasons: 'G' is the first
  // letter past the set, ':' and '>' sit either side of the digits and of 'F'
  // in ASCII so a range test written with the wrong comparison catches one of
  // them, ' ' is what ATS1 would leave behind and the surest mark of a reply,
  // and '-' is the separator a LAWICEL-shaped format would plausibly grow. The
  // six hex characters in the list are not traps at all here -- they make a
  // DIFFERENT LEGAL LINE, or a different DLC -- and the sweep treats them that
  // way rather than pretending to catch something.
  static const char kTrapChars[] = "09AFafG: ->";
  static const char kHexCycle[] = "0123456789ABCDEFabcdef";
  char buf[48];
  uint32_t cases = 0u;
  uint32_t streamsSeen = 0u;
  uint32_t othersSeen = 0u;

  // One case: classify at every width, parse once, and require them to agree.
  // Everything the parser hands back is checked here too, so a STREAM that
  // decodes to nine data bytes, or leaves the caller's poison in a byte past
  // the DLC, or reports a width no level emits, fails at the case that produced
  // it rather than somewhere downstream.
  struct Check {
    static bool one(const char* line, uint16_t len, const uint8_t* widths,
                    CanFrame& f, uint32_t& stamp, uint8_t& ts, uint32_t& cases_,
                    uint32_t& streams_, uint32_t& others_) {
      memset(&f, 0xAAu, sizeof(f));   // poison: a refused parse must not be read
      stamp = 0xDEADBEEFu;            // and a short frame must zero-fill
      ts = 0xFFu;
      const bool ok = linkParseStreamLine(line, len, f, stamp, ts);
      ++cases_;
      for (uint8_t w = 0u; w < 3u; ++w) {
        const LinkLineKind k = linkClassifyLine(line, len, widths[w]);
        CHECK_MSG((k == LINK_LINE_STREAM) == ok,
                  "ask %u len %u \"%.*s\": classify=%d but parse=%d",
                  (unsigned)widths[w], (unsigned)len, (int)len,
                  line ? line : "(null)", (int)k, (int)ok);
      }
      if (!ok) {
        ++others_;
        // A REFUSED LINE LEAVES NO RESIDUE. The caller can tell a refusal from a
        // decode only by the bool, so a parser that wrote a width or a time on
        // its way to returning false would hand anyone who ignored the bool a
        // plausible number instead of an obviously poisoned one.
        CHECK_MSG(ts == 0u, "len %u: refused, but reported width %u", (unsigned)len,
                  (unsigned)ts);
        CHECK_MSG(stamp == 0u, "len %u: refused, but reported stamp %u", (unsigned)len,
                  (unsigned)stamp);
        return false;
      }
      ++streams_;
      CHECK_MSG(ts == LINK_LINE_TS_NONE || ts == LINK_LINE_TS_SHORT ||
                    ts == LINK_LINE_TS_LONG,
                "len %u: derived width %u is not one the firmware emits", (unsigned)len,
                (unsigned)ts);
      CHECK_MSG(f.len <= CAN_MAX_DLC, "len %u: decoded len %u exceeds the DLC",
                (unsigned)len, (unsigned)f.len);
      // The arithmetic identity the whole format rests on, restated on the
      // OUTPUT: the data and the stamp the parser claims to have found have to
      // add back up to the line it was given.
      CHECK_MSG((uint16_t)(LINK_LINE_SLCAN_MIN + 2u * f.len + ts) == len,
                "len %u: %u data bytes and a %u-digit stamp do not re-make the length",
                (unsigned)len, (unsigned)f.len, (unsigned)ts);
      CHECK_MSG(f.id <= 0xFFFu, "len %u: id %u is not 3 nibbles", (unsigned)len,
                (unsigned)f.id);
      if (ts == LINK_LINE_TS_NONE) {
        CHECK_MSG(stamp == 0u, "len %u: unstamped line reported stamp %u", (unsigned)len,
                  (unsigned)stamp);
      }
      for (uint8_t i = f.len; i < CAN_MAX_DLC; ++i) {
        CHECK_MSG(f.data[i] == 0u, "len %u: byte %u not zero-filled", (unsigned)len,
                  (unsigned)i);
      }
      // And the independent mirror must get the same answer, digit for digit.
      // If it ever does not, one of the two has moved and the evidence earlier
      // in this file has quietly stopped describing the shipping parser.
      StreamDecoded m;
      memset(&m, 0, sizeof(m));
      CHECK_MSG(streamDecode(line, len, m),
                "len %u: the mirror refused what the parser accepted", (unsigned)len);
      CHECK_EQ(m.id, f.id);
      CHECK_EQ(m.len, f.len);
      CHECK_EQ(m.ts, ts);
      CHECK_EQ(m.stamp, stamp);
      CHECK_EQ(memcmp(m.data, f.data, CAN_MAX_DLC), 0);
      return true;
    }
  };

  // (a) THE ACCEPTS: every (dlc 0..8) x (width 0, 4, 8) line, generated from
  // the real payloads above. 189 lines, and each is asserted as a ROUND TRIP --
  // the id, the DLC, the bytes and the stamp that went in come back out, and
  // the width comes back having never been told. A sweep that only checked the
  // verdict would pass with a parser that read the data backwards.
  for (uint8_t i = 0u; i < kSweepCount; ++i) {
    for (uint8_t dlc = 0u; dlc <= CAN_MAX_DLC; ++dlc) {
      for (uint8_t w = 0u; w < 3u; ++w) {
        const uint8_t want = kAskWidths[w];
        const uint32_t wantStamp = want == LINK_LINE_TS_SHORT
                                       ? 0x7816u
                                       : (want == LINK_LINE_TS_LONG ? 0x0012D687u : 0u);
        const uint16_t len =
            buildSlcan(buf, kSweep[i].id, dlc, kSweep[i].data, want, wantStamp);
        CHECK_MSG(len == (uint16_t)(5u + 2u * dlc + want),
                  "%s: the generator built %u characters, not %u", buf, (unsigned)len,
                  (unsigned)(5u + 2u * dlc + want));
        CanFrame f;
        uint32_t stamp = 0u;
        uint8_t got = 0u;
        const bool ok =
            Check::one(buf, len, kAskWidths, f, stamp, got, cases, streamsSeen,
                       othersSeen);
        CHECK_MSG(ok, "%s: a generated LAWICEL line was refused", buf);
        if (!ok) {
          continue;
        }
        CHECK_MSG(f.id == kSweep[i].id, "%s: id %03X, expected %03X", buf,
                  (unsigned)f.id, (unsigned)kSweep[i].id);
        CHECK_MSG(f.len == dlc, "%s: len %u, expected %u", buf, (unsigned)f.len,
                  (unsigned)dlc);
        CHECK_MSG(got == want, "%s: derived width %u, expected %u", buf, (unsigned)got,
                  (unsigned)want);
        CHECK_MSG(stamp == wantStamp, "%s: stamp %08X, expected %08X", buf,
                  (unsigned)stamp, (unsigned)wantStamp);
        for (uint8_t b = 0u; b < dlc; ++b) {
          CHECK_MSG(f.data[b] == kSweep[i].data[b], "%s: byte %u is %02X, expected %02X",
                    buf, (unsigned)b, (unsigned)f.data[b], (unsigned)kSweep[i].data[b]);
        }
      }
    }
  }

  // (b) EVERY LENGTH from 0 to 40, marker in place and hex filler behind it.
  // The lengths no (dlc, width) pair can produce -- 0 to 4, every even one, and
  // everything past 29 -- must be REPLY, since a length rejection is the
  // cheapest half of the predicate and the half a spliced line usually trips.
  //
  // Note what the possible ones do here, because it is the derivation working
  // rather than a gap: the filler puts a '4' in the DLC nibble, so out of the
  // odd lengths 5..29 only 13, 17 and 21 are accepted -- the three that 4 data
  // bytes plus a legal width can make. LENGTH ALONE NEVER DECIDES, which is
  // exactly the difference from the ELM shape.
  for (uint16_t len = 0u; len <= 40u; ++len) {
    buf[0] = LINK_LINE_SLCAN_MARK;
    for (uint16_t i = 1u; i < len; ++i) {
      buf[i] = kHexCycle[i % 22u];
    }
    buf[len] = '\0';
    CanFrame f;
    uint32_t stamp = 0u;
    uint8_t got = 0u;
    const bool ok =
        Check::one(buf, len, kAskWidths, f, stamp, got, cases, streamsSeen, othersSeen);
    if (!slcanLengthPossible(len) && len > 0u) {
      CHECK_MSG(!ok, "length %u should be impossible, but decoded", (unsigned)len);
      for (uint8_t w = 0u; w < 3u; ++w) {
        CHECK_MSG(linkClassifyLine(buf, len, kAskWidths[w]) == LINK_LINE_REPLY,
                  "length %u: expected REPLY at width %u", (unsigned)len,
                  (unsigned)kAskWidths[w]);
      }
    }
    if (len == 0u) {
      // The empty line is the one input where the two halves answer with
      // different words -- EMPTY is not REPLY, false is not true -- and must
      // still agree.
      CHECK_EQ(linkClassifyLine(buf, 0u, LINK_LINE_TS_NONE), LINK_LINE_EMPTY);
      CHECK(!ok);
    }
  }

  // (c) ONE TRAP CHARACTER at every position of a valid line, at three lengths
  // that between them cover every field: the 5-character minimum (marker, id,
  // DLC and nothing else), a real 21-character stamped frame, and the
  // 29-character maximum, where the last eight positions are stamp digits that
  // a looser parser would never look at.
  {
    static const char* const kTrapBases[] = {
      "t1A70",                          // the minimum: no data, no stamp
      "t1A76005E000000005A30",          // 6 data bytes, a 4-digit stamp
      "t2C08FFFF170B7DFF17020012D687",  // the maximum: 8 bytes, an 8-digit stamp
    };
    for (uint8_t b = 0u; b < 3u; ++b) {
      const uint16_t blen = (uint16_t)strlen(kTrapBases[b]);
      for (uint8_t t = 0u; kTrapChars[t] != '\0'; ++t) {
        for (uint16_t pos = 0u; pos < blen; ++pos) {
          memcpy(buf, kTrapBases[b], (size_t)blen + 1u);
          buf[pos] = kTrapChars[t];
          CanFrame f;
          uint32_t stamp = 0u;
          uint8_t got = 0u;
          const bool ok = Check::one(buf, blen, kAskWidths, f, stamp, got, cases,
                                     streamsSeen, othersSeen);
          if (nibble(kTrapChars[t]) >= 0) {
            continue;   // a hex character is a different legal line, not a trap
          }
          // A NON-HEX character anywhere is fatal wherever it lands: past the
          // marker the one scan refuses the line, and ON the marker the line is
          // not a LAWICEL candidate at all and the ELM rules refuse it for the
          // same reason. Either way it must never become a frame.
          CHECK_MSG(!ok, "%s: a non-hex character at %u decoded", buf, (unsigned)pos);
          for (uint8_t w = 0u; w < 3u; ++w) {
            const LinkLineKind k = linkClassifyLine(buf, blen, kAskWidths[w]);
            CHECK_MSG(k == LINK_LINE_REPLY, "%s: expected REPLY at width %u, got %d",
                      buf, (unsigned)kAskWidths[w], (int)k);
          }
        }
      }
    }
  }

  // (d) UPPERCASE 'T', the 29-bit EXTENDED frame, on every captured line. This
  // is not a corruption -- it is a perfectly well-formed LAWICEL line of a kind
  // CanFrame cannot hold, since a uint16_t id has nowhere to put 29 bits and
  // the struct has no extended flag to raise. Reading its first three nibbles
  // as the whole id would hand core/Broadcast.cpp a value belonging to a
  // different module, so it must be REJECTED rather than truncated.
  for (uint8_t i = 0u; i < kStreamCount; ++i) {
    const uint16_t len = (uint16_t)strlen(kStreams[i].line);
    memcpy(buf, kStreams[i].line, (size_t)len + 1u);
    buf[0] = 'T';
    CanFrame f;
    uint32_t stamp = 0u;
    uint8_t got = 0u;
    const bool ok =
        Check::one(buf, len, kAskWidths, f, stamp, got, cases, streamsSeen, othersSeen);
    CHECK_MSG(!ok, "%s: an extended-frame line decoded", buf);
    for (uint8_t w = 0u; w < 3u; ++w) {
      CHECK_MSG(linkClassifyLine(buf, len, kAskWidths[w]) == LINK_LINE_REPLY,
                "%s: expected REPLY at width %u", buf, (unsigned)kAskWidths[w]);
    }
  }

  // (e) A DLC NIBBLE OF 9..F on every captured line. LAWICEL proper uses those
  // for the CAN FD lengths 12, 16, 20, 24, 32, 48 and 64; this firmware emits
  // neither FD nor those nibbles, so a line carrying one did not come from the
  // gateway we think we are talking to, and the interesting part is that its
  // LENGTH is often still legal -- only the nibble says otherwise.
  {
    static const char kBadDlc[] = "9ABCDEFabcdef";
    for (uint8_t i = 0u; i < kStreamCount; ++i) {
      const uint16_t len = (uint16_t)strlen(kStreams[i].line);
      for (uint8_t d = 0u; kBadDlc[d] != '\0'; ++d) {
        memcpy(buf, kStreams[i].line, (size_t)len + 1u);
        buf[4] = kBadDlc[d];
        CanFrame f;
        uint32_t stamp = 0u;
        uint8_t got = 0u;
        const bool ok = Check::one(buf, len, kAskWidths, f, stamp, got, cases,
                                   streamsSeen, othersSeen);
        CHECK_MSG(!ok, "%s: DLC nibble %c decoded", buf, kBadDlc[d]);
        CHECK_MSG(slcanLengthPossible(len), "%s: the length was not the reason", buf);
        for (uint8_t w = 0u; w < 3u; ++w) {
          CHECK_MSG(linkClassifyLine(buf, len, kAskWidths[w]) == LINK_LINE_REPLY,
                    "%s: expected REPLY at width %u", buf, (unsigned)kAskWidths[w]);
        }
      }
    }
  }

  // (f) A DLC CLAIMING MORE DATA THAN THE LINE CARRIES -- the shape a line cut
  // short by a reboot or a full TX ring actually has. Every one of these is
  // pure hex, in range, and a legal length; the only thing wrong with them is
  // that the nibble promises bytes that never arrived, and the ELM shape has no
  // equivalent check because there a truncated line is simply a shorter frame.
  {
    static const char* const kOverclaim[] = {
      "t1A78",              // 8 bytes promised, none present
      "t1A78005E0000",      // 8 promised, 4 present
      "t1A7400",            // 4 promised, 1 present
      "t0028001122334455",  // 8 promised, 6 present
      "t2C0812",            // 8 promised, 1 present
      "t11D3",              // 3 promised, none present
    };
    for (uint8_t i = 0u; i < (uint8_t)(sizeof(kOverclaim) / sizeof(kOverclaim[0])); ++i) {
      const uint16_t len = (uint16_t)strlen(kOverclaim[i]);
      CanFrame f;
      uint32_t stamp = 0u;
      uint8_t got = 0u;
      const bool ok = Check::one(kOverclaim[i], len, kAskWidths, f, stamp, got, cases,
                                 streamsSeen, othersSeen);
      CHECK_MSG(!ok, "%s: an over-claiming DLC decoded", kOverclaim[i]);
      for (uint8_t w = 0u; w < 3u; ++w) {
        CHECK_MSG(linkClassifyLine(kOverclaim[i], len, kAskWidths[w]) == LINK_LINE_REPLY,
                  "%s: expected REPLY at width %u", kOverclaim[i],
                  (unsigned)kAskWidths[w]);
      }
    }
  }

  // (g) A REMAINDER NO LEVEL CAN PRODUCE. After the data there are 0, 4 or 8
  // characters and nothing else; 1, 2, 3, 5, 6 and 7 are widths no AT@SML level
  // emits. THIS is the check that makes deriving the width safe rather than
  // merely convenient -- without it a line with somebody else's digits glued on
  // would decode, with a plausible time built out of them, and the reader would
  // have no way to know.
  {
    static const uint8_t kBadRemainder[] = {1u, 2u, 3u, 5u, 6u, 7u};
    static const char* const kRemainderBases[] = {"t1A70", "t1A76005E00000000"};
    for (uint8_t b = 0u; b < 2u; ++b) {
      const uint16_t blen = (uint16_t)strlen(kRemainderBases[b]);
      for (uint8_t r = 0u; r < 6u; ++r) {
        memcpy(buf, kRemainderBases[b], (size_t)blen + 1u);
        for (uint8_t k = 0u; k < kBadRemainder[r]; ++k) {
          buf[blen + k] = kHexCycle[k];
        }
        const uint16_t len = (uint16_t)(blen + kBadRemainder[r]);
        buf[len] = '\0';
        CanFrame f;
        uint32_t stamp = 0u;
        uint8_t got = 0u;
        const bool ok = Check::one(buf, len, kAskWidths, f, stamp, got, cases,
                                   streamsSeen, othersSeen);
        CHECK_MSG(!ok, "%s: a %u-character remainder decoded", buf,
                  (unsigned)kBadRemainder[r]);
        for (uint8_t w = 0u; w < 3u; ++w) {
          CHECK_MSG(linkClassifyLine(buf, len, kAskWidths[w]) == LINK_LINE_REPLY,
                    "%s: expected REPLY at width %u", buf, (unsigned)kAskWidths[w]);
        }
      }
    }
  }

  // (h) THE OTHER SHAPE. Every real ELM frame line, put to the stream parser:
  // none may decode, and none may classify STREAM at any width. This is the
  // direction that would cost a transaction rather than a reading -- a response
  // recognised as a background frame never reaches the command that asked for
  // it.
  for (uint8_t i = 0u; i < kFrameCount; ++i) {
    CanFrame f;
    uint32_t stamp = 0u;
    uint8_t got = 0u;
    const bool ok = Check::one(kFrames[i].line, (uint16_t)strlen(kFrames[i].line),
                               kAskWidths, f, stamp, got, cases, streamsSeen,
                               othersSeen);
    CHECK_MSG(!ok, "%s: an ELM frame line decoded as a stream frame", kFrames[i].line);
  }

  // (i) The firmware's whole reply vocabulary, and the degenerate inputs. None
  // of these begins with the marker, so all of them are refused by one compare
  // -- but a reply that grew a leading 't' would be caught here rather than
  // silently decoded, and a null line carrying a plausible length is the one
  // that would crash rather than misfile.
  for (uint8_t i = 0u; i < kReplyCount; ++i) {
    CanFrame f;
    uint32_t stamp = 0u;
    uint8_t got = 0u;
    const bool ok = Check::one(kReplies[i], (uint16_t)strlen(kReplies[i]), kAskWidths, f,
                               stamp, got, cases, streamsSeen, othersSeen);
    CHECK_MSG(!ok, "\"%s\": a reply decoded as a stream frame", kReplies[i]);
  }
  {
    CanFrame f;
    uint32_t stamp = 0u;
    uint8_t got = 0u;
    Check::one(0, 0u, kAskWidths, f, stamp, got, cases, streamsSeen, othersSeen);
    Check::one(0, 21u, kAskWidths, f, stamp, got, cases, streamsSeen, othersSeen);
    Check::one("", 0u, kAskWidths, f, stamp, got, cases, streamsSeen, othersSeen);
    // len is the truth, not the text: a real line with a zero length is empty.
    Check::one("t1A76005E000000005A30", 0u, kAskWidths, f, stamp, got, cases,
               streamsSeen, othersSeen);
  }

  // The corpus size, pinned, for the same reason the ELM sweep pins its own: a
  // generator that silently stopped generating would otherwise "pass", since
  // every assertion in this block lives inside a loop. 984 cases --
  //
  //   189  (a) 7 frames x 9 DLCs x 3 widths, all accepted
  //    41  (b) every length 0..40
  //   605  (c) 11 traps over every position of lengths 5, 21 and 29
  //     7  (d) the captured lines with an extended-frame marker
  //    91  (e) 7 captured lines x 13 illegal DLC nibbles
  //     6  (f) a DLC claiming more than the line carries
  //    12  (g) 2 bases x 6 impossible remainders
  //    15  (h) the ELM frame fixtures
  //    18  (i) 14 replies and 4 degenerate inputs
  //
  // -- and each of those cases asks the classifier three times, so the
  // equivalence itself is asserted 2952 times.
  CHECK_EQ(cases, 984u);
  CHECK_EQ(streamsSeen + othersSeen, cases);
  // Both answers really occur, and not a handful of each: without this the
  // whole sweep would be satisfied by a pair that agreed to reject everything.
  CHECK(streamsSeen > 400u);
  CHECK(othersSeen > 400u);
}

TEST_MAIN_END()
