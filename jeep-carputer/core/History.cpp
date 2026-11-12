#include "History.h"

#include <math.h>

// The four fixed bands. The header carries the argument for each one, and in
// particular for why coolant starts at 40 and not at 0.
const HistBand kHistBand[HIST_COUNT] = {
  {  0.0f,  50.0f },   // HIST_FLOW,    L/h
  {  0.0f, 160.0f },   // HIST_SPEED,   km/h
  { 40.0f, 120.0f },   // HIST_COOLANT, degrees C
  { 11.0f,  15.0f },   // HIST_VOLTS,   V
};

// One bucket, emptied, across every parameter at once. lo > hi is the empty
// marker -- see the header. Clearing per BUCKET rather than per parameter is
// what makes the advance below one call per step instead of four.
static void clearBucket(History& h, uint8_t b) {
  for (unsigned p = 0u; p < (unsigned)HIST_COUNT; ++p) {
    h.lo[p][b] = INT16_MAX;
    h.hi[p][b] = INT16_MIN;
  }
}

static void clearAll(History& h) {
  for (unsigned b = 0u; b < HIST_BUCKETS; ++b) {
    clearBucket(h, (uint8_t)b);
  }
}

void historyInit(History& h) {
  clearAll(h);
  h.head = 0u;
  h.headMs = 0u;
  // NOT started: the first historyTick() is what opens the first bucket. See
  // the header for why a zero headMs is not a safe stand-in for a clock
  // nobody has supplied yet.
  h.started = false;
}

void historyTick(History& h, uint32_t nowMs) {
  if (!h.started) {
    clearAll(h);
    h.head = 0u;
    h.headMs = nowMs;
    h.started = true;
    return;
  }

  // UNSIGNED SUBTRACTION, NEVER A COMPARISON OF TIMESTAMPS. `nowMs < headMs`
  // is true for a whole window either side of the 49.7-day rollover and would
  // freeze the ring there -- a chart that stops moving while every number
  // beside it keeps changing. The difference of two unsigned values is right
  // straight through the wrap, because the wrap cancels.
  const uint32_t elapsed = (uint32_t)(nowMs - h.headMs);

  // THE CAP, and it is the case worth the comment.
  //
  // At or past a full window every bucket in the ring is older than the window
  // and there is nothing left to keep, so the whole thing is cleared and the
  // clock restarted at `nowMs`. Stepping instead would reach the same empty
  // ring the expensive way: a truck parked for an hour resumes by spinning 180
  // iterations, and a `nowMs` that has genuinely come back round the 49-day
  // clock -- or a caller that hands over a wild timestamp once -- would spin
  // millions of them, on the UI tick, for the same result.
  //
  // It is also why the loop below needs no iteration bound of its own: past
  // this test `elapsed` is strictly less than HIST_WINDOW_MS, so the loop can
  // step at most HIST_BUCKETS - 1 times.
  if (elapsed >= HIST_WINDOW_MS) {
    clearAll(h);
    h.head = 0u;
    h.headMs = nowMs;
    return;
  }

  // The condition is recomputed each turn and stays an unsigned difference, so
  // a headMs that wraps mid-advance is handled by the same arithmetic as one
  // that does not -- there is no special case at the rollover because there is
  // nothing there to special-case.
  while ((uint32_t)(nowMs - h.headMs) >= HIST_BUCKET_MS) {
    h.head = (uint8_t)((h.head + 1u) % HIST_BUCKETS);
    clearBucket(h, h.head);
    h.headMs += HIST_BUCKET_MS;   // wraps with the clock, which is the point
  }
}

void historyPush(History& h, uint8_t param, float v) {
  if (!h.started || param >= (uint8_t)HIST_COUNT) {
    return;
  }
  // A NaN IS NOT A READING. It cannot be clamped into range -- every
  // comparison against it is false, so it would fall through both bounds --
  // and converting one to an integer is undefined, which under the host
  // sanitizers aborts the suite rather than storing a wrong bar. Dropping it
  // is also the honest answer: a sample that was never a number is exactly
  // what an empty bucket already says.
  if (!(v == v)) {
    return;
  }

  // TENTHS, AND CLAMPED BEFORE THE CONVERSION RATHER THAN AFTER. Scaling first
  // and rounding the result is what actually overflows: lroundf returns a
  // `long`, which is 32 bits on the target, so a 1e9 reading scaled to 1e10
  // is undefined before any int16_t is in sight. The bounds are therefore
  // applied to the float, while it can still hold them.
  //
  // A tenth is enough resolution for all four bands and int16_t holds every
  // one of them with room to spare: flow 0..500, speed 0..5120 (0x002 can
  // legitimately carry 511.98 km/h -- only exactly 0xFFFF is rejected, so the
  // top of that range is a reading and not a sentinel), coolant -400..2150,
  // volts 0..255. The widest is an eighth of the available range, so this
  // clamp exists for a caller with a bug and never for the car.
  float t = v * 10.0f;
  if (t < (float)INT16_MIN) {
    t = (float)INT16_MIN;
  }
  if (t > (float)INT16_MAX) {
    t = (float)INT16_MAX;
  }
  const int16_t q = (int16_t)lroundf(t);

  int16_t& bLo = h.lo[param][h.head];
  int16_t& bHi = h.hi[param][h.head];
  // TWO INDEPENDENT TESTS, not an if/else. On an empty bucket -- lo at
  // INT16_MAX, hi at INT16_MIN -- the first sample satisfies both, which is
  // precisely how a bucket goes from empty to the degenerate range [q, q]
  // without a separate flag saying it did.
  if (q < bLo) {
    bLo = q;
  }
  if (q > bHi) {
    bHi = q;
  }
}

bool historyRange(const History& h, uint8_t param, uint8_t age,
                  float& lo, float& hi) {
  if (param >= (uint8_t)HIST_COUNT || age >= HIST_BUCKETS || !h.started) {
    return false;
  }
  // age 0 is the bucket now filling, HIST_BUCKETS-1 the oldest. The
  // + HIST_BUCKETS before the modulo is what keeps the whole expression in
  // unsigned arithmetic: head - age underflows for every age past head, which
  // is most of the chart most of the time.
  const unsigned i =
      ((unsigned)h.head + HIST_BUCKETS - (unsigned)age) % HIST_BUCKETS;

  const int16_t l = h.lo[param][i];
  const int16_t g = h.hi[param][i];
  if (l > g) {
    // Empty, and `lo`/`hi` are left exactly as the caller had them. See the
    // header: zeroing them would hand back a legitimate reading -- 0 km/h and
    // 0 L/h both mean stopped -- and a hole in the trace would become twenty
    // seconds of parked truck.
    return false;
  }
  lo = (float)l / 10.0f;
  hi = (float)g / 10.0f;
  return true;
}
