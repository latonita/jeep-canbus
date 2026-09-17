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

// Throw away a run that was never launched, ONCE ITS WINDOW HAS PASSED.
//
// This is the in-flight half of the launch gate and it is about the STATE, not
// about publishing: it takes the timer out of RUNNING the moment the run
// becomes hopeless, so accelRunning() stops being true and nothing goes on
// being timed. finish() carries the other half -- it refuses to publish an
// unlaunched run at all -- because this test can only ever fire after
// ACCEL_LAUNCH_MS and a run that reaches both splits inside that window never
// comes past here.
//
// IDLE rather than ARMED, because as far as we know the car is moving and a run
// may only ever arm from rest.
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

// Close the run out: publish what a LAUNCHED run managed and take the best of
// it. An unlaunched one is thrown away here -- see the gate below.
static void finish(AccelSet& a) {
  // THE LAUNCH GATE, stated where the publishing happens rather than only in
  // dropUnlaunched(). Both are needed, and they answer different questions.
  //
  // dropUnlaunched() is purely TEMPORAL -- it abandons a run only once
  // `now - startMs` has passed ACCEL_LAUNCH_MS -- so a run that FINISHES
  // inside that window never meets it at all. Two speed samples 100 ms apart
  // are enough: 0 km/h idling arms, one reading over 100 puts both splits in
  // the bag, the "every split in the bag" branch calls finish(), and an
  // unlaunched run is published with a 55 ms 0-60 on it.
  //
  // That is reachable, not theoretical. core/Broadcast.cpp rejects the 0x002
  // speed word only at EXACTLY 0xFFFF, so 0xFFFE reads as a real 511.98 km/h
  // -- and playback.log proves near-sentinel corruption happens on this bus,
  // four frames of it in one 841 s drive. One such sample is permanent damage
  // rather than a wrong row: `best` only ever moves FASTER and accelInit()
  // runs once per boot, so a 55 ms 0-60 sits there beside every genuine run
  // for the rest of the key cycle, and no later 5 s launch can displace it.
  // A frozen-but-plausible number that cannot be corrected is the exact
  // failure mode this project is written to avoid.
  //
  // Here, and not in the four callers. finish() is the ONLY writer of `hist`
  // and `best`, so one test at the choke point cannot be forgotten by the
  // fifth caller; guarding each site instead would have to be got right four
  // times over and again next time. And NOT by making dropUnlaunched()
  // ignore the elapsed time -- that would kill a real launch before its pedal
  // could arrive, since 0x2C0 is a 100 ms frame and the window exists
  // precisely because a foot is not simultaneous with the first millimetre.
  //
  // The run is still WIPED, exactly as dropUnlaunched() wipes it: an
  // unlaunched run has nothing worth keeping however quick it looked, and
  // leaving `live` populated would let the next arming inherit its splits.
  if (!a.live.floored) {
    memset(&a.live, 0, sizeof(a.live));
    return;
  }

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
