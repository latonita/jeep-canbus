#pragma once
// Jeep Grand Cherokee WK/WH (2005-2006) interior CAN-B bus, 83.333 kbps.
// Everything here was established empirically by the original 2015 firmware and
// is unchanged. Newer models run the interior bus at 125 kbps and call it
// "high-speed"; do not assume these ids transfer.
#include <stdint.h>

// ---- Identifiers we act on ----------------------------------------------
#define CANID_POWER           0x000u   // ignition / power state
#define CANID_BLINKERS        0x006u   // turn signals (not yet used)
#define CANID_RADIO_MODE      0x09Fu   // head unit current source
#define CANID_RADIO_CONTROLS  0x394u   // head unit panel / steering buttons
#define CANID_VES_UNIT        0x3DDu   // VES entertainment module presence
#define CANID_VIN             0x01Bu   // vehicle VIN, 3-frame multiplex (see below)

// ---- Radio mode (CANID_RADIO_MODE, byte 0 low nibble) --------------------
#define RADIO_MODE_MASK       0x0Fu
#define RADIO_MODE_AUX        0x06u    // VES/AUX selected -> power the BT module

// ---- Panel buttons (CANID_RADIO_CONTROLS, byte 3 bitmask) ----------------
#define BTN_SEEK_RIGHT        0x01u
#define BTN_SEEK_LEFT         0x02u
#define BTN_FF_RIGHT          0x04u    // observed, unmapped
#define BTN_FF_LEFT           0x08u    // observed, unmapped
#define BTN_RND_PTY           0x20u    // reused as play/pause

// Minimum length required before indexing the byte each id is read at.
#define LEN_RADIO_MODE_MIN    1u
#define LEN_RADIO_CONTROLS_MIN 4u

// ---- Payloads we transmit -----------------------------------------------
// "A VES unit is present." Sent periodically; the head unit only offers the
// AUX/VES source while it keeps hearing this.
#define VES_ANNOUNCE_LEN      8u
#define VES_ANNOUNCE_BYTE0    0x03u

// Bench only: convinces a head unit removed from the car that the key is on.
#define BENCH_POWERON_LEN     6u
#define BENCH_POWERON_BYTE0   0x63u

// ---- VIN broadcast (CANID_VIN) ------------------------------------------
// BENCH ONLY. On a real car the body/cluster broadcasts the VIN on 0x01B and
// the radio compares it against the VIN it learned when it was paired; a
// mismatch -- or silence -- makes the radio lock and demand the SKREEM
// anti-theft code. A head unit on the desk hears no VIN, so it locks. We
// replay the car's own VIN to keep it unlocked.
//
// NEVER transmit this on a live bus: the genuine module is already sending
// 0x01B, and a second transmitter would collide with it. It is gated behind
// bench mode for exactly that reason.
//
// Wire format observed in jeep-can-b-logs (playback-long-wo-radio.log): DLC 8,
// byte 0 = frame index, bytes 1..7 = seven ASCII VIN characters. Three frames
// (index 00/01/02) carry the 17-char VIN; the tail of the last is zero-padded.
// The car spaces the frames evenly ~100 ms apart -- full VIN every ~300 ms
// (measured against capture A's 50 ms heartbeat; the raw synthetic timestamps
// in jeep.log are +1 ms/frame and must not be read as wall-clock). An index-03
// "not ready" marker appears only while a module boots; the working replay
// never sends it, so neither do we.
//
// Optional compile-time DEFAULT VIN. **Empty by default**: with no VIN stored
// in EEPROM (see BenchMode / AT@VIN), bench mode transmits the power-on frame
// only and NO VIN frames. Provision the VIN at runtime with AT@VIN, or bake a
// board-specific default in with -DVEHICLE_VIN=\"1J8HCE8M45Y588369\".
#ifndef VEHICLE_VIN
#define VEHICLE_VIN           ""
#endif
#define VIN_FRAME_LEN         8u   // DLC: 1 index byte + 7 ASCII chars
#define VIN_CHARS_PER_FRAME   7u
#define VIN_FRAME_COUNT       3u   // ceil(17 / 7)
// Longest VIN the 3 frames can carry (a real VIN is 17). Used to size the
// runtime VIN buffer that AT@VIN provisions -- see BenchMode.
#define VIN_MAX_LEN           (VIN_FRAME_COUNT * VIN_CHARS_PER_FRAME)  // 21

// ---- OVC3860 (XS3868 / H166) A2DP module, AT over UART -------------------
// Transmit-only in this firmware; the module's replies are never read.
enum OvcCommand : uint8_t {
  OVC_NONE = 0,
  OVC_PLAY_PAUSE,   // AT#MA
  OVC_NEXT_TRACK,   // AT#MD
  OVC_PREV_TRACK,   // AT#ME
};
