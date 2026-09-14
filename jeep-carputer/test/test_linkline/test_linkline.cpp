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
#include "doctest_min.h"

#include <stdint.h>
#include <string.h>

#include "CarLink.h"
#include "LinkLine.h"

namespace {

LinkLineKind classify(const char* s) {
  return linkClassifyLine(s, (uint16_t)strlen(s));
}

// Independent re-implementation of "is this a hex digit", used only to audit
// the fixtures. Deliberately NOT the one in LinkLine.cpp: if both were the same
// code the audit would be circular.
bool auditHex(char c) {
  static const char kDigits[] = "0123456789ABCDEFabcdef";
  for (uint8_t i = 0u; kDigits[i] != '\0'; ++i) {
    if (kDigits[i] == c) {
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
    const LinkLineKind k = linkClassifyLine(buf, len);
    CHECK_MSG(k == LINK_LINE_REPLY, "even length %u: expected REPLY, got %d", (unsigned)len,
              (int)k);
  }
  // ... and every odd length in range, all hex, must be FRAME.
  for (uint16_t len = 3u; len <= 19u; len += 2u) {
    for (uint16_t i = 0u; i < len; ++i) {
      buf[i] = "0123456789ABCDEF"[i & 0x0Fu];
    }
    buf[len] = '\0';
    const LinkLineKind k = linkClassifyLine(buf, len);
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
  CHECK_EQ(linkClassifyLine(0, 0u), LINK_LINE_EMPTY);
  CHECK_EQ(linkClassifyLine(0, 19u), LINK_LINE_EMPTY);   // NULL wins over length
  CHECK_EQ(linkClassifyLine("", 0u), LINK_LINE_EMPTY);
  CHECK_EQ(linkClassifyLine("002", 0u), LINK_LINE_EMPTY);  // len is the truth, not the text
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
  CHECK_EQ(linkClassifyLine("00\0" "21", 5u), LINK_LINE_REPLY);
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

TEST_MAIN_END()
