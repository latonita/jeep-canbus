#pragma once
// Telling a streamed CAN frame apart from an AT command reply.
//
// The ATmega328 gateway has a "stream mode" (AT@SM1) in which it prints every
// frame the MCP2515 hands it, continuously, in the background -- while still
// answering AT commands on the same UART. So a host that sends `ATZ` gets its
// identity string back with an unknown number of frame lines wrapped around it,
// and the two have to be told apart with nothing but the text of one line.
//
// There is no marker to key on. The gateway deliberately does NOT prefix stream
// frames (see AtServer::streamFrame: "the header is always printed regardless
// of ATH", and nothing else is added) because a prefix would cost a byte per
// frame on a 9600-baud-era link and break every existing ELM327-shaped parser.
// What is left is the SHAPE of the line, and under the session settings this
// project configures -- ATH1 headers on, ATS0 no spaces, ATL0 no linefeeds --
// the shape is decisive:
//
//   frame line = 3 hex chars of 11-bit id, then exactly 2 hex chars per data
//                byte, no separators, 0..CAN_MAX_DLC bytes, then the optional
//                timestamp described below
//
// which makes every frame line PURE HEX, SPACE-FREE, and of ODD length: 3, 5,
// 7, ... 19 unstamped. Every reply the firmware can emit fails at least one of
// those:
//
//   OK, ?, NO DATA, CAN ERROR, BUFFER FULL   -- non-hex letters, or too short
//   latonita-jeep v2.1 (ATI/ATZ)             -- letters and a space
//   AT@EV <TAG> <value>                      -- '@' and spaces
//   FW=2.1 MODE=01 ... SM=1 SD=0 (AT@ST)     -- '=' and spaces
//
// TIMESTAMPS WIDEN THE WINDOW, AND THE WIDTH CANNOT BE SNIFFED.
//
// Stream mode is a LEVEL, not a flag (gateway 2.1): AT@SM0 off, AT@SM1 the
// unstamped stream this was written against, and AT@SM2 / AT@SM3 the same
// stream with a millisecond stamp APPENDED to every frame line -- 4 uppercase
// hex digits of millis() % 60000 (bit-exact LAWICEL Z1) or 8 of the raw
// millis(). ATMA lines are stamped to match; request/response lines never are.
// Under ATS0 nothing separates the stamp from the data, so the set of accepted
// lengths becomes
//
//   len == 3 + 2n + W,   n in 0..CAN_MAX_DLC,   W in {0, 4, 8}
//
// and W has to be passed IN. It cannot be recovered from the line. Real
// LAWICEL carries a DLC nibble (`t100211334D67`), so a LAWICEL host detects a
// stamp by comparing the payload length against the DLC it was handed; this
// adapter prints no 't' and no DLC, and the data length is only ever IMPLIED by
// the hex count. A 19-char line is therefore EITHER 8 data bytes unstamped OR
// 6 data bytes plus a 4-digit stamp -- both pure hex, both odd, both in range.
// Length-sniffing would have to guess between them, and a guess here is a wrong
// number on a dashboard. None of this is inherent, only inherent to THIS
// format: the gateway's other stream mode (AT@SML) emits true LAWICEL, where a
// leading 't' marks a frame line outright and the DLC nibble makes the stamp
// width measurable from the line itself -- but the two modes are mutually
// exclusive, and this host stays on AT@SM because everything here and in
// parseFrameLine is built around its shape.
//
// The host is the side that ASKED for a level, so it always knows the width:
// the level number declares both "stream on" and "this wide" in one token.
// src/At328Link.cpp maps its level to a width in exactly one place and hands
// that one value to both this classifier and its own parseFrameLine, which is
// what stops the two splitting the same line at different offsets.
//
// Both widths are EVEN, so ODD PARITY SURVIVES -- 3+2n+4 and 3+2n+8 are odd
// as well -- and every reply that used to be disposed of for being even still
// is. What the stamp costs is RANGE: at W=8 a frame line can be 27 characters,
// so "BUFFER FULL" (11) and the identity string are no longer rejected on
// length and lean entirely on the hex scan. That scan still separates them --
// nothing in the firmware's reply vocabulary is pure hex; each carries a letter
// past 'F', a space, '=', '@' or '.' -- but the margin is thinner than it was.
// Which is why the accepted set is kept EXACT rather than the cap merely
// raised: a 21- or a 25-char line matches no (n, W) pair and is still rejected
// outright instead of being handed to a parser that would find a DLC for it.
//
// This is a CLASSIFIER, not a parser: it answers "which queue does this line
// belong in", and src/At328Link.cpp's parseFrameLine still does the decoding.
// The two must agree on what a hex digit is, so the rules here are written to
// match that file's hexVal() exactly -- 0-9, A-F, a-f. A disagreement would
// show up as a frame silently routed to the command reader, i.e. a reading that
// never arrives and a command that never returns.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "CarLink.h"

// Shortest frame line: 3 id chars and no data at all. A DLC-0 frame is legal
// CAN and the gateway prints it as a bare id.
#define LINK_LINE_FRAME_MIN 3u

// Longest UNSTAMPED frame line: the same 3 id chars plus two per data byte.
// Derived from CAN_MAX_DLC rather than written as 19, so that the bound here
// and the `(dataChars / 2u) > CAN_MAX_DLC` check in parseFrameLine cannot drift
// apart. A timestamp shifts both this and the minimum up by its own width; the
// classifier adds it rather than either macro carrying it, because the width is
// a runtime setting and these two are the shape of a frame.
#define LINK_LINE_FRAME_MAX (LINK_LINE_FRAME_MIN + 2u * CAN_MAX_DLC)   // 19

// The stamp widths the AT@SM levels produce, in characters. These three are
// the only values linkClassifyLine() and parseFrameLine() are ever passed, and
// src/At328Link.cpp's stampCharsForLevel() is the one mapping from a level to
// a width.
#define LINK_LINE_TS_NONE  0u   // AT@SM0/AT@SM1 -- the default, nothing appended
#define LINK_LINE_TS_SHORT 4u   // AT@SM2 -- millis() % 60000, wraps every 60 s
#define LINK_LINE_TS_LONG  8u   // AT@SM3 -- raw millis(), wraps in ~49.7 days

enum LinkLineKind : uint8_t {
  LINK_LINE_EMPTY = 0,   // nothing to classify
  LINK_LINE_FRAME,       // parses as an id + data bytes (+ a stamp, if any)
  LINK_LINE_REPLY,       // anything else: a command reply, status or event
};

// Classify one already-terminated line. `len` is the length without the
// terminator, as readLine() reports it.
//
// `tsChars` is the stamp width the adapter was CONFIGURED for -- one of the
// LINK_LINE_TS_* values -- and never a width to be discovered from the line.
// Get it wrong in either direction and the failure is silent: too small and a
// stamped 8-byte frame exceeds every accepted length and is filed as a command
// reply, too large and a short frame line has two of its data bytes read as a
// time. That is why the width is owned in one place (At328Link, the side that
// chose the AT@SM level) and passed down, rather than defaulted or assumed
// here.
//
// Ambiguity is resolved towards REPLY on purpose. A misfiled reply costs one
// command that has to time out; a misfiled frame is a wrong number on a
// dashboard, which is the failure this whole project is written to avoid. Note
// that this cannot rescue a WRONG tsChars: a line that is valid under two
// widths is FRAME under both, and the only defence is that the width is
// configuration rather than inference.
//
// The '>' prompt never reaches here -- readLine() treats it as a terminator in
// its own right -- but it classifies as REPLY if it ever does, because it is
// one character and a frame line is at least three.
LinkLineKind linkClassifyLine(const char* line, uint16_t len, uint8_t tsChars);
