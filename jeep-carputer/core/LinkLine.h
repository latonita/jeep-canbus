#pragma once
// Telling a streamed CAN frame apart from an AT command reply.
//
// The ATmega328 gateway streams every frame the MCP2515 hands it, continuously,
// in the background -- while still answering AT commands on the same UART. So a
// host that sends `ATZ` gets its identity string back with an unknown number of
// frame lines wrapped around it, and the two have to be told apart with nothing
// but the text of one line.
//
// TWO FRAME SHAPES ARRIVE ON ONE WIRE, and which is which is not a setting --
// it is what the line IS.
//
// The gateway has two stream formats (2.1), and this host now asks for the
// second one:
//
//   AT@SM<n>   the ELM327 shape: 3 hex id nibbles, then 2 per data byte, no
//              't', no DLC, honouring ATS/ATL, with an optional stamp appended.
//   AT@SML<n>  true LAWICEL/SLCAN: 't' + 3 hex id + ONE HEX DLC NIBBLE + 2 hex
//              per data byte + an optional stamp, terminated by a bare CR,
//              ignoring ATS and ATL entirely.
//
// The levels mirror each other -- 0 off, 1 unstamped, 2 a 4-digit stamp of
// millis() % 60000 (bit-exact LAWICEL "Z1"), 3 an 8-digit raw millis() -- and
// the two are MUTUALLY EXCLUSIVE in the gateway, one field.
//
// But REQUEST/RESPONSE lines are always the ELM shape and are NEVER stamped,
// whatever the stream format is (AtServer.h, "which lines get which format").
// slcan has no request/response notion, and those lines belong to the ELM327
// surface -- ATH, NO DATA, the ISO-TP flow -- that host parsers depend on. So
// under AT@SML2, which is what src/At328Link.cpp asks for, this file sees
// THREE kinds of line and all three at once:
//
//   t1A76024100000000B3A7   a STREAM frame:   LAWICEL, stamped
//   5163B00E2E2E2E1FF       a RESPONSE frame: ELM shape, never stamped
//   OK / ? / NO DATA / >    a reply
//
// and that is a better arrangement than it sounds, because the two frame shapes
// now identify THEMSELVES rather than having to be told apart from a reply by
// arithmetic.
//
// WHY LAWICEL IS THE SAFER WIRE, and why this file grew a second parser rather
// than switching one over.
//
// 't' IS AN UNAMBIGUOUS MARKER. It is not a hex digit, and nothing else the
// firmware emits starts with it: the banner starts with 'l', the replies are
// OK / ? / NO DATA / CAN ERROR / BUFFER FULL, and AT@EV / AT@ST lines start
// with their own text. One character decides, before any counting.
//
// THE DLC NIBBLE MAKES THE STAMP WIDTH MEASURABLE. This is the important one.
// In the ELM shape the data length is only ever IMPLIED by the hex count, so
// `3 + 2n` and `3 + 2n + 4` overlap: a 19-char line is EITHER 8 data bytes
// unstamped OR 6 data bytes plus a 4-digit stamp, both pure hex, both odd, both
// in range. The width could not be recovered from the line and had to be passed
// in from the side that chose the level -- and getting it wrong was silent in
// both directions, either filing a frame as a command reply or reading two data
// bytes as a time. On a LAWICEL line the DLC says how many data bytes follow,
// so
//
//   width = len - 1 - 3 - 1 - 2 * dlc
//
// is an ARITHMETIC IDENTITY, not a guess. linkParseStreamLine() therefore
// DERIVES the width and hands it back, and the reader compares it against the
// level it asked for: a mismatch becomes a visible complaint instead of a
// dashboard full of plausible wrong numbers.
//
// What is left for the ELM shape is only the RESPONSE line, and only at width
// ZERO, which is the case that was never ambiguous: pure hex, space-free, odd
// length 3..19. The stamped-width machinery below stays because the AT@SM
// fallback still uses it (an adapter that refuses AT@SML gets AT@SM1, and one
// that refuses both gets the ATMA windows), and because width 0 is the same
// code path either way -- but nothing this host asks for depends any more on
// being told a width it cannot check.
//
// CLASSIFIER AND PARSERS, in that order and in this one file.
// linkClassifyLine() answers "which queue does this line belong in";
// linkParseStreamLine() and linkParseFrameLine() then decode what it accepted.
// They live together BECAUSE they must agree -- on the accepted length sets, on
// the timestamp width, and on what a hex digit is -- and a disagreement shows up
// as a frame silently routed to the command reader: a reading that never
// arrives and a command that never returns. One hexVal(), one shape check per
// format shared between classify and decode, and test_linkline sweeps
// ACCEPT-IFF-DECODE across both formats and all three widths.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "CarLink.h"

// Shortest frame line: 3 id chars and no data at all. A DLC-0 frame is legal
// CAN and the gateway prints it as a bare id.
#define LINK_LINE_FRAME_MIN 3u

// Longest UNSTAMPED frame line: the same 3 id chars plus two per data byte.
// Derived from CAN_MAX_DLC rather than written as 19, so that the bound here
// and the `(dataChars / 2u) > CAN_MAX_DLC` check in linkParseFrameLine() cannot
// drift apart -- and since both are now in one file with one test sweeping
// them together, that is enforced rather than hoped for. A timestamp shifts
// both this and the minimum up by its own width; the classifier adds it rather
// than either macro carrying it, because the width is a runtime setting and
// these two are the shape of a frame.
#define LINK_LINE_FRAME_MAX (LINK_LINE_FRAME_MIN + 2u * CAN_MAX_DLC)   // 19

// The stamp widths both stream formats produce, in characters. These three are
// the only values that reach the two parsers, and both directions are covered:
// linkParseFrameLine() is TOLD one (the ELM shape cannot measure it) while
// linkParseStreamLine() DERIVES one and reports which it found.
#define LINK_LINE_TS_NONE  0u   // level 1 -- nothing appended
#define LINK_LINE_TS_SHORT 4u   // level 2 -- millis() % 60000, wraps every 60 s
#define LINK_LINE_TS_LONG  8u   // level 3 -- raw millis(), wraps in ~49.7 days

// ---- the LAWICEL/SLCAN stream line ----------------------------------------
//
// 't' + 3 hex id + 1 hex DLC + 2 hex per data byte + an optional stamp.
//
// The marker is LOWERCASE and that is a fact about the gateway rather than a
// convention: in LAWICEL 't' is an 11-bit standard data frame and 'T' is the
// 29-bit extended one. CanFrame holds a uint16_t id with no extended and no RTR
// flag, so this firmware can never emit or accept anything but 't'. A 'T' line
// is therefore not a frame we are equipped to read, and it is REJECTED rather
// than decoded as if the first three nibbles were the whole id.
#define LINK_LINE_SLCAN_MARK 't'

// Shortest: the marker, the id, a DLC of 0 and nothing else. `t1A70` is a legal
// LAWICEL line for a zero-length frame, which is legal CAN.
#define LINK_LINE_SLCAN_MIN 5u

// Longest: the same head, 8 data bytes and the widest stamp. 29 characters,
// which is what AT328_LINE_MAX has to clear.
#define LINK_LINE_SLCAN_MAX   (LINK_LINE_SLCAN_MIN + 2u * CAN_MAX_DLC + LINK_LINE_TS_LONG)   // 29

enum LinkLineKind : uint8_t {
  LINK_LINE_EMPTY = 0,   // nothing to classify
  LINK_LINE_STREAM,      // LAWICEL: a background STREAM frame, self-describing
  LINK_LINE_FRAME,       // ELM shape: a request RESPONSE (+ a stamp, if any)
  LINK_LINE_REPLY,       // anything else: a command reply, status or event
};

// Classify one already-terminated line. `len` is the length without the
// terminator, as readLine() reports it.
//
// STREAM IS TESTED FIRST, and it costs one character compare: a leading 't'
// cannot begin any other line the firmware emits. A line that starts with 't'
// and then fails the LAWICEL shape check is a REPLY, never retried as an ELM
// frame -- 't' is not a hex digit, so it could not have been one anyway, and
// falling through would only invent a second reading of a line that already
// declared what it was.
//
// `tsChars` applies to the ELM shape ONLY, where the data length is implied by
// the hex count and the width therefore cannot be recovered from the line.
// Under AT@SML it is always LINK_LINE_TS_NONE, because the only ELM-shaped
// lines left are request responses and those are never stamped; it carries a
// real width only on the AT@SM fallback path. Get it wrong there and the
// failure is silent in both directions -- too small and a stamped 8-byte frame
// exceeds every accepted length and is filed as a command reply, too large and
// a short frame line has two of its data bytes read as a time -- which is why
// the width is owned in one place (At328Link, the side that chose the level)
// and passed down rather than defaulted here. The LAWICEL branch ignores it
// entirely and measures instead.
//
// Ambiguity is resolved towards REPLY on purpose. A misfiled reply costs one
// command that has to time out; a misfiled frame is a wrong number on a
// dashboard, which is the failure this whole project is written to avoid.
//
// The '>' prompt never reaches here -- readLine() treats it as a terminator in
// its own right -- but it classifies as REPLY if it ever does, because it is
// one character and either frame shape is at least three.
LinkLineKind linkClassifyLine(const char* line, uint16_t len, uint8_t tsChars);

// Decode a line the classifier has already accepted as LINK_LINE_STREAM.
//
// THE WIDTH IS AN OUTPUT, not an input. `tsChars` comes back as 0, 4 or 8 --
// whatever the DLC nibble and the line length say it must be -- and there is no
// third reading to choose between, which is the whole reason this host moved off
// the ELM shape. The caller is expected to COMPARE it against the level it asked
// the gateway for and complain about a mismatch: a stamp that arrived where none
// was ordered means the adapter is in a mode we did not set, and the frame is
// still decoded correctly while the discrepancy is said out loud.
//
// `stampMs` is 0 when the derived width is 0, and is otherwise the adapter's OWN
// rolling clock -- millis() % 60000 at width 4. It is NOT a host time and NOT
// monotonic across a 60 s boundary. core/StreamClock.h is what turns it into
// something a trip meter or an acceleration timer can subtract.
//
// Returns false on a line that is not a LAWICEL frame after all: no marker, a
// length no (dlc, width) pair can produce, a DLC above CAN_MAX_DLC, or a
// non-hex digit anywhere including the stamp. Those are the classifier's rules
// restated in decoding terms, which is the point -- ACCEPT and DECODE must be
// the same predicate, and test_linkline asserts they are over a generated
// corpus rather than trusting this paragraph.
//
// `out.data` is zero-filled past `out.len`, so a short frame carries no residue
// from whatever the caller's CanFrame held before.
bool linkParseStreamLine(const char* line, uint16_t len, CanFrame& out,
                         uint32_t& stampMs, uint8_t& tsChars);

// Decode a line the classifier has already accepted as LINK_LINE_FRAME: the
// ELM shape, which under AT@SML means a request RESPONSE.
//
// `tsChars` must be the SAME width linkClassifyLine() was given for this line
// -- it is the one thing the caller cannot be sloppy about, since the width is
// exactly what decides where the data stops and the time starts, and both
// readings of a 19-char line are internally consistent. Under AT@SML that width
// is always 0 and the question does not arise.
//
// Returns false on a line that is not a frame after all: a length no (n,
// tsChars) pair can produce, an odd data count, more than CAN_MAX_DLC bytes, or
// a non-hex digit anywhere including the stamp.
//
// `out.data` is zero-filled past `out.len`. `stampMs` is 0 at width 0 -- which
// is every line this host now sees through here.
bool linkParseFrameLine(const char* line, uint16_t len, uint8_t tsChars,
                        CanFrame& out, uint32_t& stampMs);
