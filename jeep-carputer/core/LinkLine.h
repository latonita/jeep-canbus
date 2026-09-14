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
//                byte, no separators, 0..CAN_MAX_DLC bytes
//
// which makes every frame line PURE HEX, SPACE-FREE, and of ODD length: 3, 5,
// 7, ... 19. Every reply the firmware can emit fails at least one of those:
//
//   OK, ?, NO DATA, CAN ERROR, BUFFER FULL   -- non-hex letters, or too short
//   latonita-jeep v2.1 (ATI/ATZ)             -- letters and a space
//   AT@EV <TAG> <value>                      -- '@' and spaces
//   FW=2.1 MODE=01 ... SM=1 SD=0 (AT@ST)     -- '=' and spaces
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

// Longest frame line: the same 3 id chars plus two per data byte. Derived from
// CAN_MAX_DLC rather than written as 19, so that the bound here and the
// `(dataChars / 2u) > CAN_MAX_DLC` check in parseFrameLine cannot drift apart.
#define LINK_LINE_FRAME_MAX (LINK_LINE_FRAME_MIN + 2u * CAN_MAX_DLC)   // 19

enum LinkLineKind : uint8_t {
  LINK_LINE_EMPTY = 0,   // nothing to classify
  LINK_LINE_FRAME,       // parses as an id + data bytes
  LINK_LINE_REPLY,       // anything else: a command reply, status or event
};

// Classify one already-terminated line. `len` is the length without the
// terminator, as readLine() reports it.
//
// Ambiguity is resolved towards REPLY on purpose. A misfiled reply costs one
// command that has to time out; a misfiled frame is a wrong number on a
// dashboard, which is the failure this whole project is written to avoid.
//
// The '>' prompt never reaches here -- readLine() treats it as a terminator in
// its own right -- but it classifies as REPLY if it ever does, because it is
// one character and a frame line is at least three.
LinkLineKind linkClassifyLine(const char* line, uint16_t len);
