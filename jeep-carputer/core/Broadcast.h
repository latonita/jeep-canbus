#pragma once
// Decoding for the frames the bus emits unasked.
//
// Everything here is free: no request, no bus load, and far faster than a poll
// can be. 0x002 carries RPM and speed every 50 ms where a KWP read of the same
// values costs a whole transaction and lands once or twice a second. So the
// rule this implements is: take what the bus offers, and spend KWP requests
// only on what is never broadcast (tyre pressures, fuel level, lamp states,
// fault codes).
//
// Only CONFIRMED decodes are here. Signals the capture analysis marked
// PLAUSIBLE or UNKNOWN are deliberately absent -- a wrong number on a dashboard
// is worse than a missing one.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

#include "CarLink.h"

// Ignition positions, from 0x000 byte 0. Observed walking a real key cycle.
enum IgnitionState : uint8_t {
  IGN_UNKNOWN = 0,
  IGN_OFF,
  IGN_ACC,
  IGN_RUN,
  IGN_CRANK,
};

// One slot per value, each with the time it last arrived, so a presenter can
// tell a held reading from a live one.
struct BroadcastState {
  uint16_t rpm;
  uint32_t rpmMs;

  float    speedKph;
  uint32_t speedMs;

  int16_t  coolantC;
  uint32_t coolantMs;

  float    ambientC;
  uint32_t ambientMs;

  float    volts;
  uint32_t voltsMs;

  uint32_t odometerDm;      // decimetres... no: tenths of a km, as sent
  uint32_t odometerMs;

  uint8_t  ignition;        // IgnitionState
  uint32_t ignitionMs;

  uint8_t  dimmerRaw;       // 0x210 byte 1, as sent
  uint32_t dimmerMs;

  bool     lightsOn;        // 0x1C8 byte 0 == 0x02
  uint8_t  lightsRaw;       // the byte itself, so a wrong guess shows in the log
  uint8_t  dimmerByte0;     // 0x210 byte 0; 0x03 in every recorded sample

  uint8_t  pedalRaw;        // 0x2C0 byte 2, as sent
  uint32_t pedalMs;
  uint32_t lightsMs;

  // Instantaneous fuel flow from 0x1A7, raw: L/h = flowRaw / 160. The 0xFFFF
  // "no value" answer is rejected at decode, so flowMs only ever moves for a
  // real reading -- and zero IS a real reading. See core/Trip.h.
  uint16_t flowRaw;
  uint32_t flowMs;

  // ---- Compass, 0x1B4, DLC 6, ~500 ms -------------------------------------
  //
  // byte 0 is the module's own 8-way octant enum (0=N, 1=NE, 2=E, 3=SE, 4=S,
  // 5=SW, 6=W, 7=NW) and byte 1 the magnetic heading at 2 deg/count. MEASURED
  // over the 2953 frames of the timestamped drive log and cross-checked on an
  // independent 1584-frame 2019 session: DLC 6 in 2953 of 2953, period mean
  // 502.8 ms and median 500, 99.83 % of gaps inside 480-520 ms. byte 2 is 0x07
  // and bytes 3-5 are 0x00 in every frame of both sessions.
  //
  // The octant is ROUNDED, not floored. byte 0 agrees with round(byte1*2/45)
  // mod 8 in 99.46 % of frames where floor binning agrees in only 41.65 %, and
  // the 16 disagreements are boundary cases inside the +/-1 deg the 2 deg/count
  // scale allows. byte 1 is OBSERVED 0..178; 179 -- 358 deg, one count short of
  // north -- is the ASSUMED ceiling and has not been seen.
  //
  // THERE IS NO UNAVAILABLE SENTINEL. 0xFF never appears on any of these bytes
  // in either session, unlike 0x002, 0x014 and 0x1A7. The consequence is worth
  // stating plainly: headingMs can only ever be zero before the first frame, so
  // this signal reads ABSENT exactly once, at boot, and is HELD-or-FRESH for
  // ever after.
  //
  // It is also NOT a key-off indicator. 0x1B4 keeps sending a valid, unchanged
  // heading straight through an ignition-off transition -- ~1000 lines past the
  // 0x000 flip to 0x01 in scripts/playback.log -- so nothing may gate key-off
  // on it.
  //
  // The heading is bit-for-bit STATIC when the car is stopped: 196 of 196
  // consecutive stationary sample pairs showed exactly 0.0 deg of delta, no
  // jitter at all. So an unchanged value is the normal case at a set of lights
  // and the STAMP is the only thing separating a parked compass from a dead
  // one.
  //
  // ONE STAMP for both values, the dimmerRaw/dimmerByte0 precedent rather than
  // the rpm/speed one: direction and heading arrive on the SAME frame and
  // neither can be independently unavailable, so two stamps could only ever
  // disagree by being wrong. headingStatus is kept for exactly the reason
  // lightsRaw is kept -- so a wrong guess shows in the log rather than
  // misbehaving quietly on the glass.
  // 0x159 byte 5, the tank level as sent. ~1 s period.
  //
  // This REPLACED the CCN 0x2103 poll rather than joining it. The old comment
  // in kPlan said fuel was "not broadcast"; it is, once a second, and the poll
  // was buying a slower copy of it for a transaction every five seconds.
  //
  // THE SCALE IS NOT CALIBRATED. `% = raw/2.55` is the map's working
  // conversion and nothing has ever checked it against a tank -- but neither
  // had the poll it replaces, whose x0.5 came out of the same vendor table
  // that gave the battery voltage a +5.0 offset reading 17.6 V for a real
  // 12.6 V. So this trades nothing away. See CLAUDE.md, "ONE TANK RUN SETTLES
  // THREE OPEN NUMBERS".
  uint8_t  fuelRaw;
  uint32_t fuelMs;

  // 0x394 byte 3, the RAR head unit's button bitfield, as sent. Bit 2 is
  // fast-forward and bit 3 rewind; core/Buttons.h turns them into page steps.
  //
  // BYTE 3, NOT BYTE 4. The byte map assigns the button bits to b4 and calls b3
  // constant zero; the owner has it the other way round, and the captures
  // cannot arbitrate -- both bytes are 0x00 in every frame ever logged.
  //
  // Kept RAW and undecoded here on purpose. The byte carries five buttons and
  // this firmware acts on two of them, so a decode would be throwing away
  // three bits that cost nothing to keep and that the log then shows if a
  // mapping ever turns out to be wrong.
  uint8_t  btnRaw;
  uint32_t btnMs;

  uint8_t  headingOctant;   // 0x1B4 byte 0, the module's own 8-way enum
  uint8_t  headingRaw;      // 0x1B4 byte 1, 2 deg/count
  uint8_t  headingStatus;   // 0x1B4 byte 2; 0x07 in every frame of both captures
  uint32_t headingMs;

  uint32_t frames;          // everything seen, decoded or not
  uint32_t decoded;
};

void broadcastClear(BroadcastState& st);

// Fold one frame in. Returns true when it carried something we understand.
bool broadcastApply(BroadcastState& st, const CanFrame& f, uint32_t nowMs);

// Screen brightness for a dimmer reading, in PER MILLE of full duty.
//
// Per mille, not percent, because the useful range at the bottom is finer than
// 1%: on a 13-bit backlight, 1% is still 81 counts and reads as clearly lit.
//
// The mapping is NOT linear: it is square-law, so the bottom of the dimmer
// wheel gets fine control. A linear map wastes most of the travel across a
// range that all looks equally bright and leaves the useful dim end in the
// first few degrees.
//
// Byte 0x00 is the BOTTOM of the wheel on this car and means barely visible,
// not "lights off therefore daylight therefore bright" -- an earlier version
// had that backwards. The top of the range deliberately stops short of full,
// because a full-brightness panel at night mirrors in the windscreen.
//
// `haveSignal` false -- nothing heard for a while -- returns 1000 (full).
// Failing bright is the safe direction: an unreadable dash is worse than a
// bright one.
// Panel brightness for a dimmer reading, in per mille of full duty.
//
// `haveLights` / `lightsOn` come from 0x1C8 and exist to resolve one ambiguity:
// 0x210 reports 0x00 both for the BOTTOM OF THE DIMMER WHEEL and for EVERY
// LIGHT IN THE CAR BEING OFF, and those want opposite brightness -- barely
// visible at night, full brightness in daylight.
//
// They can only ever make the panel BRIGHTER. With no lights signal the dimmer
// wheel keeps full control, exactly as before, so a bus that never carries
// 0x1C8 loses nothing.
uint16_t broadcastBacklightPermille(uint8_t dimmerRaw, bool haveSignal,
                                    bool lightsOn, bool haveLights);

// ---- Accelerator pedal, 0x2C0 byte 2 --------------------------------------
//
// A dual-track APPS relayed onto CAN-B at 100 ms. Byte 2 is track one, byte 3
// its half-scale partner and byte 6 a byte-identical copy of byte 2 -- the
// usual three-way redundancy for a safety signal. Bytes 0, 1 and 5 are always
// 0xFF and carry nothing.
//
// The two ends are ANCHORED, not guessed. 0x17 is proven by a hardware idle
// switch: across 3767 capture frames with bit 2 or 3 of byte 7 set, byte 2 was
// exactly 0x17 every single time, no exceptions. The top is the KICKDOWN
// DETENT -- the hump you can feel underfoot, past which the box downshifts.
//
// And the capture shows the detent rather than merely implying it. Between
// 0xC0 and 0xE9 there is NOTHING: not one frame in 8089. The pedal sweeps up
// to the hump (single frames at 0xB6, 0xBF on the way through) and then the
// travel past it is fast enough that it is only ever observed at rest on the
// floor, 54 consecutive frames at 0xEA through a 1509-4158 rpm pull. You do
// not sit in the detent; you are either short of it or through it.
//
// That makes kickdown a genuinely BINARY, driver-intentional event, which is
// worth far more than a percentage: it cannot be reached by accident and needs
// no calibration to interpret.
// How long a pedal reading stays usable. 0x2C0 is a 100 ms frame, so five
// missed ones is not a live pedal.
//
// This matters more than it looks. pedalRaw is HELD when the frame stops --
// every value on this instrument is -- and the kickdown flag is now what
// ARMS an acceleration run rather than what decorates one. A held 0xEA would
// confirm the launch of every pull-away for as long as the signal stayed away:
// frozen, plausible, and wrong, which is the one failure mode this project is
// built around. Callers gate on this before feeding accelPedal().
#define BC_PEDAL_TTL_MS 500u

#define BC_PEDAL_RELEASED 0x17u   // hardware idle switch agrees, 3767/3767
#define BC_PEDAL_DETENT   0xC0u   // the hump underfoot
#define BC_PEDAL_KICKDOWN 0xDFu   // through it; only 0xEA is ever seen above
#define BC_PEDAL_FLOORED  0xEAu   // the floor, held flat under full throttle

// 0..100, linear between BC_PEDAL_RELEASED and BC_PEDAL_FLOORED, clamped at
// both ends.
//
// BOTH ANCHORS ARE MEASURED, and the correction is worth recording because
// this comment used to say the opposite. It claimed "this car has never been
// logged past 0x37 -- a sweep in Park would pin it properly", which described
// an early probe session and was never revisited once real driving data
// existed. Re-measured over `scripts/playback.log` (8089 0x2C0 frames, 7624
// accepted by the three gates below):
//
//   * 1631 accepted frames sit ABOVE 0x37, across 69 distinct values.
//   * The accepted maximum is 0xEA, with 54 frames -- held flat through a
//     1509 -> 4158 rpm pull, which is what "floored" looks like in the data.
//   * 0x17 released is the other end, and that one is anchored harder still:
//     the hardware idle switch agrees in 3767 of 3767 frames.
//
// So the range is pinned by the capture, not by a guess, and no sweep in Park
// is needed. The CLAMP stays, for the reason the wrong comment obscured: the
// decoder accepts 0xEB..0xFE (only 0xFF is a sentinel), so raws above the
// floor are representable even though none has been observed on two agreeing
// tracks. They read 100%, which is the only honest answer -- there is no more
// pedal.
uint8_t broadcastPedalPercent(uint8_t raw);
