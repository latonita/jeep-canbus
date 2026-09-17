#pragma once
// 0-60, 60-100 and 0-100 km/h, timed from the speed broadcast and nothing else.
//
// 0x002 carries road speed several times a second, which is the only clock and
// the only ruler this has. There is no accelerometer on the board and no GPS
// yet, so every limit below comes from the sample rate rather than from taste.
//
// A run is: the car is STOPPED WITH THE ENGINE IDLING, the car is LAUNCHED --
// accelerator through the kickdown detent -- and a split is recorded the first
// time the speed crosses it. Nothing else arms it. Joining a motorway at
// 90 km/h is not the start of a run and this must never invent one, so the
// timer can only ever be armed by having genuinely seen the car at rest.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

// The measured splits, in km/h, ascending -- accelUpdate() relies on the order.
// 60-100 is not measured, it is the difference of these two.
#define ACCEL_SPLIT_60  0
#define ACCEL_SPLIT_100 1
#define ACCEL_SPLIT_COUNT 2
extern const float kAccelSplitKph[ACCEL_SPLIT_COUNT];

// PER-SPLIT cut-offs. Slower than this is not a run and is not recorded.
//
// One number could not do this job. A stock WH 3.0 CRD reaches 60 km/h in about
// 3.5-4 s and 100 in about 9.5, so a single 12 s window would catch 60 four
// times over -- every set of lights and every supermarket car park would
// register as an acceleration run and the figure on screen would mean nothing.
// 6 s and 12 s each sit roughly half again beyond the real figure: enough slack
// for a cold engine, a hill or a wet road, not enough for ordinary driving.
//
// A split missed only costs itself. A 0-60 inside 6 s still counts even when
// 100 never arrives.
extern const uint32_t kAccelLimitMs[ACCEL_SPLIT_COUNT];

// Below this the car counts as stationary.
//
// Not 0.0 exactly. The speed signal is quantised and a stopped car reports the
// occasional fraction, so an exact zero test would arm and disarm at random on
// a car standing still.
#define ACCEL_ZERO_KPH 0.5f

// Engine speed at or below which the car counts as idling, for ARMING only.
//
// Checked when the timer ARMS, never afterwards. That distinction is what keeps
// a brake-torque launch working: you arrive at the line at idle and arm there,
// and the engine is then free to climb against the brake without disarming --
// which is exactly the technique that produces the quickest run and would be
// absurd to exclude. What it does exclude is arming in the middle of something
// else, and arming with the engine stopped, where rpm is 0.
#define ACCEL_IDLE_RPM 800u
#define ACCEL_RUNNING_RPM 300u   // below this the engine is not turning

// How long after the car moves the accelerator has to reach the KICKDOWN
// DETENT for this to count as a launch. Miss it and the run is abandoned:
// nothing is timed, nothing is published, and the DRIVE sub-line never flips.
//
// This is the fix for the thing that made the feature useless in traffic --
// pulling gently out of a junction started the stopwatch, because rest-then-
// motion was the whole trigger and the pedal only decorated the result
// afterwards. Every set of lights read as an acceleration run.
//
// The pedal is the right gate precisely because it is not a threshold anyone
// has to calibrate: between the detent and the floor there is not one frame in
// 8089 captured, so nobody ever rests there and pushing past it is a decision.
// See BC_PEDAL_KICKDOWN.
//
// 1500 ms, not zero. 0x2C0 arrives every 100 ms, the launch happens somewhere
// inside the sample that first shows motion, and a driver's foot and the car's
// first millimetre are not simultaneous. Long enough for the real order of
// events, far too short for "pull away, then decide to overtake".
#define ACCEL_LAUNCH_MS 1500u

// How many finished runs are kept, newest first.
#define ACCEL_HISTORY 5

enum AccelState : uint8_t {
  ACCEL_IDLE = 0,   // moving, but this motion did not start from rest
  ACCEL_ARMED,      // stationary and idling, waiting for the car to move
  ACCEL_RUNNING,    // timing
};

// One set of split times in milliseconds. 0 means "not achieved".
struct AccelRun {
  uint32_t ms[ACCEL_SPLIT_COUNT];
  // Whether the accelerator went through the KICKDOWN DETENT during the run.
  //
  // This is what separates a launch from a brisk pull-away, and it is worth
  // more than any threshold on time or rpm because it is a physical event: you
  // feel the hump underfoot and you have to mean it to push past. The capture
  // bears that out -- between the detent and the floor there is not one frame
  // in 8089, so nobody ever rests there.
  //
  // It is REQUIRED, as of ACCEL_LAUNCH_MS. It used to be decoration -- a run
  // inside the cut-offs counted without it and was shown as the lesser thing
  // it was -- and that is exactly what made every pull-away from a junction
  // start the stopwatch. A run that never latches this is thrown away.
  //
  // ENFORCED, in two places that catch different runs. dropUnlaunched() takes
  // the timer out of RUNNING once ACCEL_LAUNCH_MS has passed without it; and
  // finish() -- the ONLY writer of `hist` and `best` -- refuses a run that
  // lacks it, which is what catches a run that ends INSIDE the window. It was
  // a comment here and nothing else for a while, and a single corrupt speed
  // sample was then enough to publish a two-sample 55 ms 0-60 and leave it in
  // `best` for the rest of the boot, since `best` only ever moves faster.
  bool floored;
};

struct AccelSet {
  uint8_t  state;
  uint32_t startMs;    // bus timestamp the run began at
  uint32_t lastStamp;  // the speed stamp last folded in; 0 = nothing yet
  float    lastKph;

  AccelRun live;                    // the run in progress
  AccelRun hist[ACCEL_HISTORY];     // finished runs, hist[0] most recent
  uint8_t  histCount;
  AccelRun best;                    // quickest ever seen, PER SPLIT
};

void accelInit(AccelSet& a);

// Fold in one speed reading. `stampMs` is the broadcast's own timestamp, and a
// repeat of the last one is ignored -- the UI refreshes far more often than
// 0x002 arrives, and re-folding a stale sample would stretch every run.
//
// `rpm` is only consulted to decide whether a stationary car may ARM.
void accelUpdate(AccelSet& a, float kph, uint16_t rpm, uint32_t stampMs);

// Ends a run that stopped arriving. Call with the current clock; a run whose
// splits were never reached expires even if the car has gone off the bus.
void accelTick(AccelSet& a, uint32_t nowMs);

// Tell the timer the accelerator is through the kickdown detent. Latches for
// the run in progress and is ignored at any other time. The pedal encoding
// stays in Broadcast, where the frame is -- see BC_PEDAL_KICKDOWN.
void accelPedal(AccelSet& a, bool kickdown);

// Timing: a launch is being measured right now. This is what the display
// should follow -- it goes true when the pedal confirms the launch, which is
// within a frame or two of the car moving on a real one, and never goes true
// at all for a gentle pull-away.
bool accelLaunched(const AccelSet& a);

// The raw state: a run is being timed, launched or not yet. A run that never
// gets its kickdown is dropped ACCEL_LAUNCH_MS after it started, so this is
// true for up to that long on an ordinary pull-away -- which is why it is not
// the predicate to draw anything from.
bool accelRunning(const AccelSet& a);

// How long the run in progress has been going, or 0 when none is.
uint32_t accelElapsedMs(const AccelSet& a, uint32_t nowMs);

// The rolling 60-100 of a run: the difference of its two splits, or 0 when
// either is missing. Derived rather than measured -- there is only one clock
// and both ends of it are already recorded.
//
// It is the figure that says most about the ENGINE rather than about traction
// and how the launch was made, which is why it is worth showing beside them.
uint32_t accelMidMs(const AccelRun& r);

// The most recently finished run, or nullptr when there is none yet.
const AccelRun* accelLast(const AccelSet& a);
