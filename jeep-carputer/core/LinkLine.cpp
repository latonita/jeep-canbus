#include "LinkLine.h"

// What a hex digit is WORTH: 0-9, A-F, a-f, and -1 for anything else.
//
// This used to be two functions in two translation units -- a predicate here
// and a value-returning hexVal() in src/At328Link.cpp, next to the parser --
// and the comment that stood here called that duplication the risk: change one
// accept set and a line the parser would decode gets filed as a command reply,
// losing the frame without a trace. That is now structural rather than
// promised. The parser moved in beside the classifier (linkParseFrameLine
// below) and the predicate is defined in terms of this, so there is ONE accept
// set and no way to move half of it.
//
// Lowercase is accepted even though the gateway emits uppercase only
// (AtServer's atFormatHexBytes uses an uppercase table). Both halves have to
// agree on that too: the alternative is a class of line that passes
// classification and then fails to decode, or vice versa.
static int hexVal(char c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  return -1;
}

static bool isHexDigit(char c) { return hexVal(c) >= 0; }

// ---- the LAWICEL shape check, shared by the classifier and the decoder ----
//
// 't' <3 id nibbles> <1 DLC nibble> <2 per data byte> [<stamp>]
//
// THE WIDTH IS AN OUTPUT, and that is the whole reason this host asked the
// gateway for AT@SML rather than staying on AT@SM. With the DLC on the wire the
// remainder of the line is the stamp by arithmetic:
//
//   tsChars = len - LINK_LINE_SLCAN_MIN - 2 * dlc
//
// and the only judgement left is whether that remainder is a width the firmware
// can actually produce -- 0, 4 or 8. There is no second reading of a LAWICEL
// line to choose between, so nothing here has to be told anything.
//
// Returns false on anything that is not a LAWICEL line, and only assigns the
// outputs on success. Both callers below ask exactly this one question, which is
// how ACCEPT-IFF-DECODE is kept structural rather than promised, and
// test_linkline sweeps it.
static bool slcanShape(const char* line, uint16_t len, uint8_t& dlc,
                       uint8_t& tsChars) {
  if (line == nullptr || len < LINK_LINE_SLCAN_MIN ||
      len > LINK_LINE_SLCAN_MAX) {
    return false;
  }
  // Lowercase only. In LAWICEL 't' is an 11-bit standard data frame and 'T' is
  // the 29-bit extended one, and CanFrame has no room for an extended id -- so
  // a 'T' line is a frame we are not equipped to hold, and reading its first
  // three nibbles as the whole id would hand the dashboard a value belonging to
  // a different module. Rejected, not truncated.
  if (line[0] != LINK_LINE_SLCAN_MARK) {
    return false;
  }

  // Every character after the marker is a hex digit, the stamp included. ONE
  // scan, so a bad digit anywhere fails the line rather than yielding a good
  // frame with a garbage time on it -- and it runs before the DLC is read, which
  // is what makes the read below safe.
  for (uint16_t i = 1u; i < len; ++i) {
    if (!isHexDigit(line[i])) {
      return false;
    }
  }

  const int d = hexVal(line[4]);
  if (d < 0 || static_cast<uint8_t>(d) > CAN_MAX_DLC) {
    // A DLC of 9..F is not a length this struct can hold. LAWICEL proper uses
    // those nibbles for CAN FD lengths; this firmware emits neither, so a line
    // carrying one did not come from the gateway we think we are talking to.
    return false;
  }

  const uint16_t headChars =
      static_cast<uint16_t>(LINK_LINE_SLCAN_MIN + 2u * static_cast<uint16_t>(d));
  if (len < headChars) {
    return false;          // the DLC claims more data than the line carries
  }
  const uint16_t ts = static_cast<uint16_t>(len - headChars);
  if (ts != LINK_LINE_TS_NONE && ts != LINK_LINE_TS_SHORT &&
      ts != LINK_LINE_TS_LONG) {
    // A remainder no level can produce. THIS is the check that makes deriving
    // the width safe rather than merely convenient: a truncated or spliced line
    // -- and the gateway does splice them, a capture of one taken while a
    // terminal echoed frames back at it contains lines like
    // `t4148FD153Ft0147259C8DFF192858443A` -- lands here instead of being
    // decoded with a plausible time built out of somebody else's data bytes.
    return false;
  }

  dlc = static_cast<uint8_t>(d);
  tsChars = static_cast<uint8_t>(ts);
  return true;
}

LinkLineKind linkClassifyLine(const char* line, uint16_t len, uint8_t tsChars) {
  if (line == 0 || len == 0u) {
    return LINK_LINE_EMPTY;
  }

  // STREAM FIRST, and it costs one character compare. 't' is not a hex digit
  // and nothing else the firmware emits begins with it, so a line starting with
  // the marker is either a LAWICEL frame or malformed -- there is no third
  // possibility to fall through to, and letting a spliced line have a second
  // go at the ELM rules would only invent a reading for a line that already
  // said what it was.
  if (line[0] == LINK_LINE_SLCAN_MARK) {
    uint8_t dlc = 0u;
    uint8_t ts = 0u;
    return slcanShape(line, len, dlc, ts) ? LINK_LINE_STREAM : LINK_LINE_REPLY;
  }

  // ---- and from here down, the ELM shape, unchanged ----
  //
  // Which under AT@SML is only ever a request RESPONSE, and only at width 0.
  // The stamped widths stay because the AT@SM fallback still produces them, and
  // because width 0 is the same code path either way.

  // Length first, because it is the cheapest test and it disposes of most of
  // the reply vocabulary outright: "OK" and "?" are too short to be a frame,
  // and the identity string and the AT@ST line are too long for any width.
  //
  // The accepted set is EXACT -- len == 3 + 2n + tsChars for n in
  // 0..CAN_MAX_DLC -- and not merely a raised cap. A stamp shifts BOTH ends of
  // the window by its own width, so a length no (n, tsChars) pair can produce
  // (21 unstamped, 25 with a 4-digit stamp) must be rejected here: it is odd
  // and it can be pure hex, so nothing downstream would catch it and
  // linkParseFrameLine would find a plausible DLC for it.
  const uint16_t minLen = (uint16_t)(LINK_LINE_FRAME_MIN + tsChars);
  const uint16_t maxLen = (uint16_t)(LINK_LINE_FRAME_MAX + tsChars);
  if (len < minLen || len > maxLen) {
    return LINK_LINE_REPLY;
  }

  // ODD length is still the load-bearing rule for THIS shape, stated against
  // the DATA rather than the whole line: two hex chars to the byte, and both
  // stamp widths are even, so a frame line is odd stamped or not. "NO DATA" (7)
  // and "CAN ERROR" (9) are odd too, which is precisely why the hex test below
  // still has to run.
  const uint16_t dataChars = (uint16_t)(len - minLen);
  if ((dataChars & 1u) != 0u) {
    return LINK_LINE_REPLY;
  }

  // Bounded by construction: len is at most LINK_LINE_FRAME_MAX + tsChars, and
  // the stamp digits are hex on the wire exactly like the data, so one scan
  // covers both. A non-hex digit anywhere -- id, data or stamp -- makes the
  // whole line a reply, which keeps this in step with linkParseFrameLine,
  // where a bad stamp digit fails the frame rather than yielding a good frame
  // with a garbage time on it.
  for (uint16_t i = 0u; i < len; ++i) {
    // A space lands here as well as every letter past 'F'. Under ATS0 the
    // gateway prints no separators at all, so a space is proof of a reply --
    // it is the single character that most reliably marks one, since every
    // multi-word reply and every AT@EV/AT@ST line carries at least one.
    if (!isHexDigit(line[i])) {
      return LINK_LINE_REPLY;
    }
  }

  return LINK_LINE_FRAME;
}

// `n` hex nibbles at `from`, big-endian, as one integer.
//
// Does not report failure, because it is never reached with an unchecked
// character: slcanShape() scans the whole line before the caller gets here. A
// predicate that could fail would have to be handled at four call sites and
// would suggest the scan had not happened.
static uint32_t hexRun(const char* line, uint16_t from, uint8_t n) {
  uint32_t v = 0u;
  for (uint8_t i = 0u; i < n; ++i) {
    v = (v << 4) | static_cast<uint32_t>(hexVal(line[from + i]));
  }
  return v;
}

// The LAWICEL decoder, and structurally the simpler of the two below for one
// reason: it is not TOLD a width, it measures one. There is nothing the caller
// can get wrong here and no reading of the line that depends on state
// elsewhere, which is the property the ELM shape could not offer.
//
// The stamp handed back is the ADAPTER'S OWN clock -- millis() % 60000 at width
// 4 -- so it rolls over every minute. It is not a host time and it is not
// monotonic. core/StreamClock.h is what makes it subtractable; mapping it is
// deliberately not done here, because this file's job is ONE line and a clock
// needs the history of all of them.
bool linkParseStreamLine(const char* line, uint16_t len, CanFrame& out,
                         uint32_t& stampMs, uint8_t& tsChars) {
  stampMs = 0u;
  tsChars = 0u;

  uint8_t dlc = 0u;
  uint8_t ts = 0u;
  if (!slcanShape(line, len, dlc, ts)) {
    return false;
  }

  out.id = static_cast<uint16_t>(hexRun(line, 1u, 3u));
  out.len = dlc;
  for (uint8_t i = 0u; i < dlc; ++i) {
    out.data[i] =
        static_cast<uint8_t>(hexRun(line, static_cast<uint16_t>(5u + i * 2u), 2u));
  }
  // Zero-filled past out.len, so a short frame carries no residue from whatever
  // the caller's CanFrame held before.
  for (uint8_t i = dlc; i < CAN_MAX_DLC; ++i) {
    out.data[i] = 0u;
  }

  if (ts != 0u) {
    stampMs = hexRun(
        line, static_cast<uint16_t>(LINK_LINE_SLCAN_MIN + 2u * dlc), ts);
  }
  tsChars = ts;
  return true;
}

// With ATH1 + ATS0 a frame line is exactly three id nibbles followed by an even
// number of data nibbles, no separators -- and then, at AT@SM2 or AT@SM3, a
// fixed-width millisecond stamp with no separator before it either.
//
// `tsChars` is the CONFIGURED width. It is not measurable: there is no DLC on
// this wire, so `3+2n` and `3+2n+4` overlap and the split has to be TOLD. It
// must be the value linkClassifyLine() accepted this line with, which is why
// the reader passes one copy to both.
//
// This lived in src/At328Link.cpp as a private static of At328Link, which put
// it in an ESP-IDF translation unit -- unlinkable on a laptop, so the coupling
// this header spends half its length on was asserted in PROSE and nothing
// else, and test_linkline had to carry a hand-written MIRROR of the split to
// say anything about it at all. Sitting here it is the same object file as the
// classifier, shares its one hexVal(), and test_linkline can now assert
// ACCEPT-IFF-DECODE directly over a generated corpus instead of describing it.
// That is the whole reason for the move: the two halves of this decision are
// only safe together, so they are now tested together.
bool linkParseFrameLine(const char* line, uint16_t len, uint8_t tsChars,
                        CanFrame& out, uint32_t& stampMs) {
  stampMs = 0u;
  if (line == nullptr || len < static_cast<uint16_t>(3u + tsChars)) {
    return false;
  }
  const uint16_t dataChars = static_cast<uint16_t>(len - 3u - tsChars);
  if ((dataChars & 1u) != 0u || (dataChars / 2u) > CAN_MAX_DLC) {
    return false;
  }

  uint16_t id = 0u;
  for (uint8_t i = 0u; i < 3u; ++i) {
    const int v = hexVal(line[i]);
    if (v < 0) {
      return false;
    }
    id = static_cast<uint16_t>((id << 4) | static_cast<uint16_t>(v));
  }

  out.id = id;
  out.len = static_cast<uint8_t>(dataChars / 2u);
  for (uint8_t i = 0u; i < out.len; ++i) {
    const int hi = hexVal(line[3u + i * 2u]);
    const int lo = hexVal(line[4u + i * 2u]);
    if (hi < 0 || lo < 0) {
      return false;
    }
    out.data[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  for (uint8_t i = out.len; i < CAN_MAX_DLC; ++i) {
    out.data[i] = 0u;
  }

  // The stamp last, and a bad digit in it fails the whole frame rather than
  // yielding a good frame with a garbage time on it -- the caller cannot tell
  // those apart from a bool. The classifier has already scanned every character
  // with the same accept set (hexVal's 0-9 A-F a-f), so this loop only ever
  // fails on a line that never went through it.
  for (uint8_t i = 0u; i < tsChars; ++i) {
    const int v = hexVal(line[3u + dataChars + i]);
    if (v < 0) {
      return false;
    }
    stampMs = (stampMs << 4) | static_cast<uint32_t>(v);
  }
  return true;
}
