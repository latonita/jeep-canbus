#include "BenchMode.h"

#if FEATURE_BENCH_MODE

#include <Arduino.h>
#include <EEPROM.h>

// Two magics rather than 0/1: a garbage byte then reads as "not enabled"
// instead of as "enabled", which is the direction that matters in a real car.
#define BENCH_MAGIC_ON  0x5Au
#define BENCH_MAGIC_OFF 0xA5u

// Compiled-in DEFAULT VIN, used only when nothing is stored in EEPROM. The
// active VIN lives in vin_/vinLen_ and is normally provisioned at runtime with
// AT@VIN. Frame k carries chars [k*7 .. k*7+6]; anything past the end is sent
// as 0x00, matching the car's padding on the last frame. sizeof-1 drops the NUL.
static const char kVehicleVin[] = VEHICLE_VIN;
#define VIN_CHARS (sizeof(kVehicleVin) - 1u)

// If a longer default VIN is ever -D'd in, bump VIN_FRAME_COUNT rather than
// truncate.
static_assert(VIN_CHARS <= VIN_MAX_LEN,
              "VEHICLE_VIN does not fit in VIN_FRAME_COUNT frames");

// Marks a valid stored VIN at BENCH_VIN_EEPROM_ADDR: [magic][len][chars...].
// Any other magic byte (0xFF on a fresh chip included) means "use the default".
#define BENCH_VIN_MAGIC  0xA7u

BenchMode::BenchMode(ICanBus& bus, IEventSink& sink, uint16_t eepromAddr)
    : bus_(bus),
      sink_(sink),
      lastAnnounceMs_(0),
      lastVinMs_(0),
      detectStartMs_(0),
      eepromAddr_(eepromAddr),
      writes_(0),
      vinLeft_(0),
      lastTx_(CAN_ST_OK),
      vinIdx_(0),
      vinLen_(0),
      enabled_(false),
      announcing_(false),
      detecting_(false),
      detectArmed_(false),
      vehicleSeen_(false),
      vinStored_(false) {}

// 0 cycles is the escape hatch for "send indefinitely, like the car".
#define BENCH_VIN_INFINITE     (BENCH_VIN_BURST_CYCLES == 0u)
#define BENCH_VIN_BURST_FRAMES ((uint16_t)(BENCH_VIN_BURST_CYCLES * VIN_FRAME_COUNT))

void BenchMode::begin() {
  loadVin();  // active VIN from EEPROM, or the compiled default
  uint8_t raw = EEPROM.read((int)eepromAddr_);
  // 0xFF (fresh chip), 0x00, or anything else unrecognised -> OFF.
  enabled_ = (raw == BENCH_MAGIC_ON);
  lastAnnounceMs_ = 0;
  lastVinMs_ = 0;
  vinIdx_ = 0;
  vinLeft_ = BENCH_VIN_BURST_FRAMES;  // arm the burst for a chip that boots enabled
  announcing_ = true;
  // Listen before transmitting, so a board that boots inside a car never spams
  // the live bus. The window is timed from the first tick (begin() has no now).
  detecting_ = enabled_;
  detectArmed_ = enabled_;
  vehicleSeen_ = false;
  sink_.emit(EV_BENCH_MODE, enabled_ ? 1u : 0u);
}

ModStatus BenchMode::setEnabled(bool on) {
  uint8_t want = on ? (uint8_t)BENCH_MAGIC_ON : (uint8_t)BENCH_MAGIC_OFF;
  uint8_t raw = EEPROM.read((int)eepromAddr_);
  if (raw != want) {
    // Only on a real change: the cell is good for ~100k writes.
    EEPROM.write((int)eepromAddr_, want);
    if (writes_ < 0xFFFFu) {
      writes_++;
    }
    if (EEPROM.read((int)eepromAddr_) != want) {
      return MOD_ERR_HW;
    }
  }
  if (on != enabled_) {
    enabled_ = on;
    lastAnnounceMs_ = 0;  // announce promptly once switched on
    lastVinMs_ = 0;
    vinIdx_ = 0;          // start the VIN cycle from frame 0
    vinLeft_ = on ? BENCH_VIN_BURST_FRAMES : 0u;  // (re)arm the VIN burst on enable
    // Re-run the listen-before-transmit guard whenever bench is switched on.
    detecting_ = on;
    detectArmed_ = on;
    vehicleSeen_ = false;
    sink_.emit(EV_BENCH_MODE, on ? 1u : 0u);
  }
  return MOD_OK;
}

void BenchMode::loadVin() {
  uint8_t magic = EEPROM.read((int)BENCH_VIN_EEPROM_ADDR);
  uint8_t len = EEPROM.read((int)BENCH_VIN_EEPROM_ADDR + 1);
  if (magic == BENCH_VIN_MAGIC && len >= 1u && len <= VIN_MAX_LEN) {
    for (uint8_t i = 0; i < len; i++) {
      vin_[i] = (char)EEPROM.read((int)BENCH_VIN_EEPROM_ADDR + 2 + i);
    }
    vinLen_ = len;
    vinStored_ = true;
    return;
  }
  // Nothing valid stored -> the compiled-in default, which is empty unless a
  // board baked one in with -DVEHICLE_VIN. Empty (vinLen_ == 0) means bench mode
  // sends power-on only and no VIN frames.
  vinLen_ = (uint8_t)VIN_CHARS;
  for (uint8_t i = 0; i < vinLen_; i++) {
    vin_[i] = kVehicleVin[i];
  }
  vinStored_ = false;
}

ModStatus BenchMode::setVin(const char* s, uint8_t n) {
  if (s == 0 || n == 0u || n > VIN_MAX_LEN) {
    return MOD_ERR_PARAM;
  }
  // VIN characters are printable ASCII (0x20..0x7E); reject controls and high
  // bytes so a stray line can never reach EEPROM or the bus.
  for (uint8_t i = 0; i < n; i++) {
    uint8_t c = (uint8_t)s[i];
    if (c < 0x20u || c > 0x7Eu) {
      return MOD_ERR_PARAM;
    }
  }
  // Payload first, magic last: a power loss mid-write never leaves a truncated
  // VIN marked valid. update() writes a cell only when it actually changes.
  EEPROM.update((int)BENCH_VIN_EEPROM_ADDR + 1, n);
  for (uint8_t i = 0; i < n; i++) {
    EEPROM.update((int)BENCH_VIN_EEPROM_ADDR + 2 + i, (uint8_t)s[i]);
  }
  EEPROM.update((int)BENCH_VIN_EEPROM_ADDR, BENCH_VIN_MAGIC);
  if (EEPROM.read((int)BENCH_VIN_EEPROM_ADDR) != BENCH_VIN_MAGIC) {
    return MOD_ERR_HW;
  }
  for (uint8_t i = 0; i < n; i++) {
    vin_[i] = s[i];
  }
  vinLen_ = n;
  vinStored_ = true;
  return MOD_OK;
}

ModStatus BenchMode::clearVin() {
  if (EEPROM.read((int)BENCH_VIN_EEPROM_ADDR) == BENCH_VIN_MAGIC) {
    EEPROM.update((int)BENCH_VIN_EEPROM_ADDR, 0xFFu);  // invalidate the store
    if (EEPROM.read((int)BENCH_VIN_EEPROM_ADDR) == BENCH_VIN_MAGIC) {
      return MOD_ERR_HW;
    }
  }
  loadVin();  // revert the active VIN to the compiled default
  return MOD_OK;
}

uint8_t BenchMode::getVin(char* out, uint8_t cap) const {
  if (out == 0 || cap < vinLen_) {
    return 0u;
  }
  for (uint8_t i = 0; i < vinLen_; i++) {
    out[i] = vin_[i];
  }
  return vinLen_;
}

void BenchMode::tick(uint32_t nowMs) {
  if (!enabled_ || !announcing_ || vehicleSeen_) {
    return;
  }

  // Listen-before-transmit guard. While detecting_ we send nothing; onFrame()
  // may flip vehicleSeen_ if it hears a real 0x000. Time the window from the
  // first tick, since begin() runs before millis() is meaningful here.
  if (detecting_) {
    if (detectArmed_) {
      detectStartMs_ = nowMs;
      detectArmed_ = false;
    }
    // Wrap-safe: unsigned subtraction survives the millis() rollover.
    if (nowMs - detectStartMs_ < BENCH_DETECT_WINDOW_MS) {
      return;  // still listening; do not transmit
    }
    detecting_ = false;  // window elapsed, no vehicle heard -> bench is safe
  }

  // VIN first: its absence is what triggers the anti-theft lock. The radio
  // latches it once, so we only send a bounded burst after activation (see
  // BENCH_VIN_BURST_CYCLES) rather than forever. One frame per tick keeps this
  // non-blocking. Wrap-safe subtraction survives the millis() rollover.
  // vinLen_ == 0 (no VIN provisioned) -> skip VIN entirely, power-on only.
  if (vinLen_ > 0u && (BENCH_VIN_INFINITE || vinLeft_ > 0u) &&
      nowMs - lastVinMs_ >= BENCH_VIN_PERIOD_MS) {
    lastVinMs_ = nowMs;
    CanStatus vst = announceVin(vinIdx_);
    if (vst != lastTx_) {
      lastTx_ = vst;
      sink_.emit(EV_CAN_STATE, (uint16_t)vst);
    }
    if (++vinIdx_ >= VIN_FRAME_COUNT) {
      vinIdx_ = 0;
    }
    if (!BENCH_VIN_INFINITE) {
      vinLeft_--;
    }
  }

  // Wrap-safe form; never `now > last + PERIOD`.
  if (nowMs - lastAnnounceMs_ < BENCH_ANNOUNCE_PERIOD_MS) {
    return;
  }
  lastAnnounceMs_ = nowMs;

  CanStatus st = announce();
  if (st != lastTx_) {
    lastTx_ = st;
    sink_.emit(EV_CAN_STATE, (uint16_t)st);
  }
  // No delay() after the send: the original's 20 ms would starve the watchdog
  // refresh and the AT link.
}

void BenchMode::onFrame(const CanFrame& frame) {
  // Only meaningful while enabled and still inside the listen window. A 0x000
  // power frame (len 6) there was transmitted by another node -- we never
  // receive our own -- so this board is on a live vehicle bus. Suppress bench
  // for the rest of this power cycle; the EEPROM flag is left on, so the same
  // board re-arms bench next power-up on a silent bench bus.
  if (!enabled_ || !detecting_ || vehicleSeen_) {
    return;
  }
  if (frame.id == CANID_POWER && frame.len == BENCH_POWERON_LEN) {
    vehicleSeen_ = true;
    detecting_ = false;
    sink_.emit(EV_BENCH_MODE, 0u);  // report bench announce suppressed
  }
}

CanStatus BenchMode::announce() {
  CanFrame f;
  f.id = CANID_POWER;
  f.len = BENCH_POWERON_LEN;
  f.data[0] = BENCH_POWERON_BYTE0;
  for (uint8_t i = 1; i < CAN_MAX_DLC; i++) {
    f.data[i] = 0;
  }
  return bus_.send(f);
}

// Builds one of the 3 VIN frames: byte 0 = index, bytes 1..7 = the seven VIN
// characters at that offset, zero-padded past the end of the string.
CanStatus BenchMode::announceVin(uint8_t frameIdx) {
  CanFrame f;
  f.id = CANID_VIN;
  f.len = VIN_FRAME_LEN;
  f.data[0] = frameIdx;
  uint8_t base = (uint8_t)(frameIdx * VIN_CHARS_PER_FRAME);
  for (uint8_t i = 0; i < VIN_CHARS_PER_FRAME; i++) {
    uint8_t pos = (uint8_t)(base + i);
    f.data[1 + i] = (pos < vinLen_) ? (uint8_t)vin_[pos] : 0u;
  }
  return bus_.send(f);
}

void BenchMode::safeState() {
  // The flag itself is untouched -- it is persistent configuration, not state.
  announcing_ = false;
}

#endif  // FEATURE_BENCH_MODE
