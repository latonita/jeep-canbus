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

LinkLineKind linkClassifyLine(const char* line, uint16_t len) {
  if (line == 0 || len == 0u) {
    return LINK_LINE_EMPTY;
  }

  // Length first, because it is the cheapest test and it disposes of most of
  // the reply vocabulary outright: "OK" and "?" are too short to be a frame,
  // and "BUFFER FULL", the identity string and the AT@ST line are all too long.
  if (len < LINK_LINE_FRAME_MIN || len > LINK_LINE_FRAME_MAX) {
    return LINK_LINE_REPLY;
  }

  // ODD length is the load-bearing rule. Data arrives two hex chars to the
  // byte on top of a three-char id, so a frame line can never be even -- and
  // "NO DATA" (7) and "CAN ERROR" (9) are odd, which is precisely why the hex
  // test below still has to run.
  if ((len & 1u) == 0u) {
    return LINK_LINE_REPLY;
  }

  // Bounded by construction: len is already known to be <= 19.
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
