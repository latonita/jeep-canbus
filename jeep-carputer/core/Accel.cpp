#include "Accel.h"

#include <string.h>

const float kAccelSplitKph[ACCEL_SPLIT_COUNT] = {60.0f, 100.0f};
const uint32_t kAccelLimitMs[ACCEL_SPLIT_COUNT] = {6000u, 12000u};

// The run as a whole ends at the slowest split's cut-off.
#define ACCEL_RUN_MAX_MS (kAccelLimitMs[ACCEL_SPLIT_COUNT - 1])

void accelInit(AccelSet& a) {
  memset(&a, 0, sizeof(a));
  a.state = ACCEL_IDLE;
}

bool accelRunning(const AccelSet& a) { return a.state == ACCEL_RUNNING; }

bool accelLaunched(const AccelSet& a) {
  return a.state == ACCEL_RUNNING && a.live.floored;
}

// Throw away a run that was never launched.
//
// Not finish() -- finish() PUBLISHES what the run managed, and the whole point
// here is that an unlaunched run has nothing worth publishing however quick it
// was. IDLE rather than ARMED, because as far as we know the car is moving and
// a run may only ever arm from rest.
static void dropUnlaunched(AccelSet& a, uint32_t nowMs) {
  if (a.state == ACCEL_RUNNING && !a.live.floored &&
      (nowMs - a.startMs) > ACCEL_LAUNCH_MS) {
    memset(&a.live, 0, sizeof(a.live));
    a.state = ACCEL_IDLE;
  }
}

uint32_t accelElapsedMs(const AccelSet& a, uint32_t nowMs) {
  if (a.state != ACCEL_RUNNING || nowMs < a.startMs) {
    return 0u;
  }
  return nowMs - a.startMs;
}

uint32_t accelMidMs(const AccelRun& r) {
  if (r.ms[ACCEL_SPLIT_60] == 0u || r.ms[ACCEL_SPLIT_100] == 0u ||
      r.ms[ACCEL_SPLIT_100] <= r.ms[ACCEL_SPLIT_60]) {
    return 0u;
  }
  return r.ms[ACCEL_SPLIT_100] - r.ms[ACCEL_SPLIT_60];
}

const AccelRun* accelLast(const AccelSet& a) {
  return (a.histCount == 0u) ? static_cast<const AccelRun*>(0) : &a.hist[0];
}

// Close the run out: publish whatever it managed and take the best of it.
static void finish(AccelSet& a) {
  bool any = false;
  for (uint8_t i = 0; i < ACCEL_SPLIT_COUNT; ++i) {
    if (a.live.ms[i] != 0u) {
      any = true;
      if (a.best.ms[i] == 0u || a.live.ms[i] < a.best.ms[i]) {
        a.best.ms[i] = a.live.ms[i];
        a.best.floored = a.live.floored;
      }
    }
  }
  // A run that reached nothing is not published. Otherwise every pull-away out
  // of a junction would push a row of dashes through the history and bury the
  // last real one.
  if (any) {
    for (uint8_t i = ACCEL_HISTORY - 1u; i > 0u; --i) {
      a.hist[i] = a.hist[i - 1u];
    }
    a.hist[0] = a.live;
    if (a.histCount < ACCEL_HISTORY) {
      ++a.histCount;
    }
  }
  memset(&a.live, 0, sizeof(a.live));
}

void accelPedal(AccelSet& a, bool kickdown) {
  if (kickdown && a.state == ACCEL_RUNNING) {
    a.live.floored = true;
  }
}

void accelTick(AccelSet& a, uint32_t nowMs) {
  // Before the cut-off check, and on the same clock: a pull-away that stops
  // producing speed frames must still be dropped rather than sitting in
  // RUNNING until the 12 s cut-off publishes it.
  dropUnlaunched(a, nowMs);
  if (a.state == ACCEL_RUNNING && (nowMs - a.startMs) > ACCEL_RUN_MAX_MS) {
    finish(a);
    // IDLE, not ARMED: the car is still moving as far as we know, and a run may
    // only ever be armed by seeing it stationary.
    a.state = ACCEL_IDLE;
  }
}

void accelUpdate(AccelSet& a, float kph, uint16_t rpm, uint32_t stampMs) {
  if (stampMs == 0u || stampMs == a.lastStamp) {
    return;   // nothing new on the bus
  }
  const float prevKph = a.lastKph;
  const uint32_t prevStamp = a.lastStamp;
  const bool havePrev = (prevStamp != 0u);
  a.lastKph = kph;
  a.lastStamp = stampMs;

  if (kph < ACCEL_ZERO_KPH) {
    // Standing still. Any run in progress is over -- lifting off and coasting
    // back to rest is not a 0-100, and whatever it did reach still counts.
    if (a.state == ACCEL_RUNNING) {
      finish(a);
    }
    // ARM only from a genuine standing start: engine turning, and not already
    // wound up. Checked HERE and nowhere else, so a brake-torque launch that
    // revs after arming stays armed -- see ACCEL_IDLE_RPM.
    if (rpm >= ACCEL_RUNNING_RPM && rpm <= ACCEL_IDLE_RPM) {
      a.state = ACCEL_ARMED;
    } else if (a.state != ACCEL_ARMED) {
      a.state = ACCEL_IDLE;
    }
    return;
  }

  if (a.state == ACCEL_ARMED) {
    // The car has moved. Time from the LAST MOMENT WE KNOW IT WAS STOPPED,
    // not from this sample: the launch happened somewhere in between, and
    // starting the clock at the earlier of the two can only ever report a time
    // that is too LONG. Erring slow is the only honest direction for a figure
    // a driver might repeat to someone.
    a.startMs = havePrev ? prevStamp : stampMs;
    a.state = ACCEL_RUNNING;
    memset(&a.live, 0, sizeof(a.live));
  }

  // A run still waiting for its kickdown, past the window: gone, silently.
  dropUnlaunched(a, stampMs);

  if (a.state != ACCEL_RUNNING) {
    return;   // moving, but this motion never started from rest -- or was
              // never launched
  }

  if ((stampMs - a.startMs) > ACCEL_RUN_MAX_MS) {
    finish(a);
    a.state = ACCEL_IDLE;
    return;
  }

  for (uint8_t i = 0; i < ACCEL_SPLIT_COUNT; ++i) {
    if (a.live.ms[i] != 0u || kph < kAccelSplitKph[i]) {
      continue;
    }
    // INTERPOLATE the crossing rather than taking this sample's timestamp.
    // 0x002 arrives every 50 ms or so, and at a brisk launch the car covers
    // several km/h in that gap -- so the sample that first reads over 60 is
    // already past it, by an amount that depends on how hard you pulled. That
    // is a bias, not noise, and it flatters harder launches most.
    uint32_t crossed = stampMs;
    if (havePrev && kph > prevKph && prevKph < kAccelSplitKph[i]) {
      const float span = kph - prevKph;
      const float frac = (kAccelSplitKph[i] - prevKph) / span;
      const float dt = static_cast<float>(stampMs - prevStamp);
      crossed = prevStamp + static_cast<uint32_t>(frac * dt + 0.5f);
    }
    if (crossed < a.startMs) {
      crossed = a.startMs;
    }
    const uint32_t split = crossed - a.startMs;
    // Each split has its own cut-off, and missing one costs only itself.
    if (split <= kAccelLimitMs[i]) {
      // 0 is the "not achieved" sentinel, so a genuinely instantaneous split
      // has to round up to 1 ms rather than read as nothing.
      a.live.ms[i] = (split == 0u) ? 1u : split;
    }
  }

  // Every split in the bag: the run is done, no need to wait for the cut-off.
  bool all = true;
  for (uint8_t i = 0; i < ACCEL_SPLIT_COUNT; ++i) {
    if (a.live.ms[i] == 0u) {
      all = false;
      break;
    }
  }
  if (all) {
    finish(a);
    a.state = ACCEL_IDLE;
  }
}
