#include "LinkLine.h"

// What counts as a hex digit.
//
// A verbatim copy of the accept set in src/At328Link.cpp's hexVal(): 0-9, A-F,
// a-f. It is written as a predicate rather than reusing hexVal() because that
// function lives in an anonymous namespace inside an ESP-IDF translation unit,
// and core/ must stay host-buildable. The DUPLICATION IS THE RISK here: if
// hexVal() ever changes -- gains a tolerance for 'x', say -- this must change
// with it, or a line the parser would accept gets routed to the command reader
// and the frame is lost without a trace.
//
// Lowercase is accepted even though the gateway emits uppercase only
// (AtServer's atFormatHexBytes uses an uppercase table). The parser accepts it,
// so the classifier must too: the alternative is a class of line that passes
// classification and then fails to decode, or vice versa.
static bool isHexDigit(char c) {
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

LinkLineKind linkClassifyLine(const char* line, uint16_t len, uint8_t tsChars) {
  if (line == 0 || len == 0u) {
    return LINK_LINE_EMPTY;
  }

  // Length first, because it is the cheapest test and it disposes of most of
  // the reply vocabulary outright: "OK" and "?" are too short to be a frame,
  // and the identity string and the AT@ST line are too long for any width.
  //
  // The accepted set is EXACT -- len == 3 + 2n + tsChars for n in
  // 0..CAN_MAX_DLC -- and not merely a raised cap. A stamp shifts BOTH ends of
  // the window by its own width, so a length no (n, tsChars) pair can produce
  // (21 unstamped, 25 with a 4-digit stamp) must be rejected here: it is odd
  // and it can be pure hex, so nothing downstream would catch it and
  // parseFrameLine would find a plausible DLC for it.
  const uint16_t minLen = (uint16_t)(LINK_LINE_FRAME_MIN + tsChars);
  const uint16_t maxLen = (uint16_t)(LINK_LINE_FRAME_MAX + tsChars);
  if (len < minLen || len > maxLen) {
    return LINK_LINE_REPLY;
  }

  // ODD length is still the load-bearing rule, now stated against the DATA
  // rather than the whole line: two hex chars to the byte, and both stamp
  // widths are even, so a frame line is odd stamped or not. "NO DATA" (7) and
  // "CAN ERROR" (9) are odd too, which is precisely why the hex test below
  // still has to run -- and at tsChars 8 the upper bound is 27, so several
  // longer replies now reach that scan as well.
  const uint16_t dataChars = (uint16_t)(len - minLen);
  if ((dataChars & 1u) != 0u) {
    return LINK_LINE_REPLY;
  }

  // Bounded by construction: len is at most LINK_LINE_FRAME_MAX + tsChars, and
  // the stamp digits are hex on the wire exactly like the data, so one scan
  // covers both. A non-hex digit anywhere -- id, data or stamp -- makes the
  // whole line a reply, which keeps this in step with parseFrameLine, where a
  // bad stamp digit fails the frame rather than yielding a good frame with a
  // garbage time on it.
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
