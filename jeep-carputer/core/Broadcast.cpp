#include "Broadcast.h"

// Dimmer landmarks.
//
// 0x00 is the BOTTOM of the wheel, confirmed by the owner -- and it is ALSO
// what 0x210 reports with every light in the car switched off. One byte, two
// meanings, wanting opposite brightness. Told apart by 0x1C8 below.
#define DIM_LIGHTS_OFF 0x00u
#define DIM_MAX        0xA4u   // top of the dimmer wheel
#define DIM_CABIN      0xC8u   // above the wheel's top; believed cabin lights on

// Brightness range, in PER MILLE of full duty rather than percent.
//
// Percent is too coarse at the bottom: 1% of a 13-bit duty is still 81 counts
// and reads as a clearly lit panel. The floor here is 3/1000 -- about 25 counts
// -- which is genuinely "barely visible" in a dark cabin.
#define NIGHT_MIN_PM   3u      // barely visible, but not off
#define NIGHT_MAX_PM   550u    // a full-brightness panel at night mirrors in the glass
#define CABIN_PM       750u
#define FULL_PM        1000u

// "Signal not available" markers. Every one of these is a real reading the
// modules send with the engine off, and every one scales into something
// plausible: 0xFFFF speed becomes 512 km/h, 0xFF coolant becomes 215 C, 0xFFFF
// RPM becomes 65535. A dashboard that shows those has lied.
#define SENTINEL_8   0xFFu
#define SENTINEL_16  0xFFFFu

void broadcastClear(BroadcastState& st) {
  st.rpm = 0u;         st.rpmMs = 0u;
  st.speedKph = 0.0f;  st.speedMs = 0u;
  st.coolantC = 0;     st.coolantMs = 0u;
  st.ambientC = 0.0f;  st.ambientMs = 0u;
  st.volts = 0.0f;     st.voltsMs = 0u;
  st.odometerDm = 0u;  st.odometerMs = 0u;
  st.ignition = IGN_UNKNOWN; st.ignitionMs = 0u;
  st.dimmerRaw = 0u;   st.dimmerMs = 0u;
  // Defaults to "lights on", which is the safe default: an absent 0x1C8 then
  // leaves the dimmer wheel in full control exactly as before, rather than
  // pinning the panel bright and taking the wheel away.
  st.lightsOn = true;  st.lightsMs = 0u;
  st.lightsRaw = 0u;   st.dimmerByte0 = 0u;
  st.flowRaw = 0u;     st.flowMs = 0u;
  st.frames = 0u;      st.decoded = 0u;
}

static uint8_t ignitionFrom(uint8_t raw) {
  switch (raw) {
    case 0x01u: return IGN_OFF;
    case 0x63u: return IGN_ACC;
    case 0x85u: return IGN_RUN;
    case 0xA9u: return IGN_CRANK;
    default:    return IGN_UNKNOWN;
  }
}

bool broadcastApply(BroadcastState& st, const CanFrame& f, uint32_t nowMs) {
  ++st.frames;

  switch (f.id) {
    case 0x002u: {            // RPM + speed, every 50 ms
      if (f.len < 4u) return false;
      const uint16_t rpmRaw = (uint16_t)((f.data[0] << 8) | f.data[1]);
      const uint16_t spdRaw = (uint16_t)((f.data[2] << 8) | f.data[3]);

      // A sentinel ZEROES the stamp rather than leaving it alone. That is the
      // difference between "we missed a frame" and "the module says it has no
      // value": the first should hold the last reading and dim it, the second
      // must show nothing at all. Holding a stale RPM after the engine stops
      // would be exactly the plausible lie this exists to prevent.
      if (rpmRaw == SENTINEL_16) {
        st.rpmMs = 0u;
      } else {
        st.rpm = rpmRaw;
        st.rpmMs = nowMs;
      }
      if (spdRaw == SENTINEL_16) {
        st.speedMs = 0u;
      } else {
        st.speedKph = (float)spdRaw / 128.0f;
        st.speedMs = nowMs;
      }
      if (rpmRaw == SENTINEL_16 && spdRaw == SENTINEL_16) return false;
      break;
    }

    case 0x014u: {            // odometer, tenths of a km in a uint24
      if (f.len < 3u) return false;
      const uint32_t odo = ((uint32_t)f.data[0] << 16) | ((uint32_t)f.data[1] << 8) |
                           (uint32_t)f.data[2];
      // All-ones is "no value". Left alone it reads as 1 677 721 km and, worse,
      // would re-anchor every trip to a distance no trip can ever reach.
      if (odo == 0xFFFFFFu) return false;
      st.odometerDm = odo;
      st.odometerMs = nowMs;
      break;
    }

    case 0x015u:              // ambient temperature + battery voltage
      if (f.len < 2u) return false;
      if (f.data[0] == SENTINEL_8) {
        st.ambientMs = 0u;
      } else {
        st.ambientC = (float)f.data[0] * 0.5f - 40.0f;
        st.ambientMs = nowMs;
      }
      if (f.data[1] == SENTINEL_8) {
        st.voltsMs = 0u;
      } else {
        // x0.1 with NO offset. The extracted table's +5.0 offset is a known
        // extraction bug -- it reads 17.6 V where the truth is 12.6 V.
        st.volts = (float)f.data[1] * 0.1f;
        st.voltsMs = nowMs;
      }
      break;

    case 0x02Cu:              // coolant temperature in byte 1
      if (f.len < 2u) return false;
      if (f.data[1] == SENTINEL_8) {
        st.coolantMs = 0u;    // 0xFF would scale to a convincing 215 C
      } else {
        st.coolantC = (int16_t)f.data[1] - 40;
        st.coolantMs = nowMs;
      }
      break;

    case 0x000u: {            // key position
      if (f.len < 1u) return false;
      const uint8_t ign = ignitionFrom(f.data[0]);
      if (ign == IGN_UNKNOWN) return false;   // never invent a key state
      st.ignition = ign;
      st.ignitionMs = nowMs;
      break;
    }

    case 0x210u:              // panel dimmer level in byte 1
      if (f.len < 2u) return false;
      st.dimmerRaw = f.data[1];
      st.dimmerByte0 = f.data[0];
      st.dimmerMs = nowMs;
      break;

    case 0x1A7u: {            // instantaneous fuel flow, every 100 ms
      if (f.len < 2u) return false;
      const uint16_t q = (uint16_t)((f.data[0] << 8) | f.data[1]);
      // 0xFFFF is the module saying it has no value. Taken at face value it is
      // 409 L/h, which would be the single largest lie this decoder could tell.
      // Rejecting it here leaves flowMs where it was, so the integrator sees a
      // gap and drops the interval rather than integrating anything at all.
      if (q == 0xFFFFu) return false;
      // Zero, by contrast, is REAL: overrun fuel cut-off on a trailing engine
      // burns nothing, and it has to be integrated as the zero it is.
      st.flowRaw = q;
      st.flowMs = nowMs;
      break;
    }

    case 0x2C0u: {            // accelerator pedal, every 100 ms
      if (f.len < 8u) return false;
      // REJECT THE ENGINE-OFF SENTINEL. With the engine stopped this frame is
      // all 0xFF but for a trailing 0x00, and byte 2 of that reads as a pedal
      // pressed harder than the floor -- comfortably the worst lie this
      // decoder could tell. Bit 7 of byte 7 is the engine-running flag (set
      // across 7624 capture frames at 569-4158 rpm, clear across 335 at 0-1039)
      // and the redundant copy in byte 6 must agree with byte 2.
      if ((f.data[7] & 0x80u) == 0u || f.data[2] != f.data[6] ||
          f.data[2] == 0xFFu) {
        return false;
      }
      st.pedalRaw = f.data[2];
      st.pedalMs = nowMs;
      break;
    }

    case 0x1C8u:              // exterior lights
      // Restored: it is the only signal separating "wheel wound down" from
      // "lights off", and without it the panel goes dark in broad daylight.
      // The raw byte is kept alongside the verdict, so if 0x02 turns out to be
      // the wrong test the log says so instead of the screen misbehaving
      // quietly.
      if (f.len < 1u) return false;
      st.lightsRaw = f.data[0];
      st.lightsOn = (f.data[0] == 0x02u);
      st.lightsMs = nowMs;
      break;

    default:
      return false;
  }

  ++st.decoded;
  return true;
}

uint16_t broadcastBacklightPermille(uint8_t dimmerRaw, bool haveSignal,
                                    bool lightsOn, bool haveLights) {
  if (!haveSignal) {
    return FULL_PM;           // fail bright: an unreadable dash beats a dark one
  }
  // Every light in the car off means daylight, whatever the wheel says -- and
  // with the wheel wound down the wheel says 0x00, the same thing it says at
  // the bottom of its travel at night. This is the only test that tells those
  // two apart, and it is deliberately one-directional: it can brighten the
  // panel, never dim it.
  if (haveLights && !lightsOn) {
    return FULL_PM;
  }
  if (dimmerRaw >= DIM_CABIN) {
    return CABIN_PM;
  }
  if (dimmerRaw == DIM_LIGHTS_OFF) {
    return NIGHT_MIN_PM;      // bottom of the wheel: barely visible
  }

  uint16_t d = dimmerRaw;
  if (d > DIM_MAX) {
    d = DIM_MAX;
  }

  // SQUARE LAW, not linear.
  //
  // Perceived brightness goes roughly as the square root of duty, so a linear
  // map spends most of the wheel's travel across a range that all looks about
  // equally bright, and crams the useful dim end into the first few degrees.
  // Squaring the normalised position inverts that and gives fine control at the
  // bottom -- the end that actually matters in a dark cabin.
  const uint32_t n = ((uint32_t)d * 1000u) / DIM_MAX;   // 0..1000
  const uint32_t curved = (n * n) / 1000u;              // 0..1000, square law
  const uint32_t span = NIGHT_MAX_PM - NIGHT_MIN_PM;
  return (uint16_t)(NIGHT_MIN_PM + (curved * span) / 1000u);
}

uint8_t broadcastPedalPercent(uint8_t raw) {
  if (raw <= BC_PEDAL_RELEASED) {
    return 0u;
  }
  if (raw >= BC_PEDAL_FLOORED) {
    return 100u;
  }
  const uint32_t span = (uint32_t)(BC_PEDAL_FLOORED - BC_PEDAL_RELEASED);
  return (uint8_t)(((uint32_t)(raw - BC_PEDAL_RELEASED) * 100u + span / 2u) / span);
}
