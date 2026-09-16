// Line classifier host tests.
//
// The FRAME cases are real lines off this vehicle, taken from
// ../jeep-aux-and-can-gateway/logs/inj-1A7.txt. That capture was recorded with
// ATS1 (spaces between bytes), so each line below is the same frame with the
// spaces stripped -- which is the ATS0 form stream mode actually emits, and the
// only form this classifier ever sees.
//
// The REPLY cases are the complete reply vocabulary of the gateway firmware:
// the five fixed strings src/At328Link.cpp matches with sameText(), the
// FW_IDENT from lib/Core/Version.h, the AT@EV event line and the AT@ST status
// line.
//
// Every fixture is re-derived here rather than trusted: kFrames is checked to
// be odd-length and pure hex before it is used, because a fixture that is
// secretly even or secretly contains a stray character would make the suite
// pass for the wrong reason.
//
// STAMP WIDTHS. AT@SM2 and AT@SM3 append 4 or 8 hex digits of millisecond
// timestamp to every streamed line, so the classifier takes the width it was
// configured with. The unstamped cases below therefore all pass 0 (the helper's
// default, and what an unmodified session sends), and the stamped ones derive
// their fixtures by appending digits to the same real frames. The assertion
// this suite exists for is the AMBIGUITY one: the same 19 characters are a
// valid frame line under BOTH width 0 and width 4, and decode to different
// data, which is the whole reason the width is configuration and not something
// to be sniffed off the wire.
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

// A MIRROR of src/At328Link.cpp's parseFrameLine, for the one thing a
// classifier cannot show on its own: that a line accepted under two widths
// MEANS two different things. The real parser lives in an ESP-IDF translation
// unit and cannot be linked on a laptop -- the same reason LinkLine.cpp carries
// its own copy of isHexDigit -- so this reproduces the split and nothing else.
// If parseFrameLine's split ever changes, this must change with it; the
// ambiguity assertion below is what would otherwise quietly stop being true.
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

struct FrameCase {
  const char* line;
  uint16_t    id;        // what parseFrameLine would extract, for documentation
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
  CHECK_EQ(kReplyCount, 12);
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
// and parseFrameLine fails such a line too. A reply that happens to land in the
// widened window is caught by exactly this.
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

TEST_MAIN_END()
