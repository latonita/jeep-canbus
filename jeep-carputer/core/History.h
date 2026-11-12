#pragma once
// Fifteen minutes of four numbers, in 720 bytes: the ring behind the history
// page.
//
// Everything else the renderer reads answers "what is it NOW" -- a snapshot
// resolved against one instant, a TTL, a held reading dimmed. This answers
// "what has the last quarter of an hour looked like", which is a different
// question and wants a different structure: 45 buckets of 20 s, each holding
// the MIN and the MAX seen inside it.
//
// A BUCKET HOLDS A MIN AND A MAX, NOT A MEAN, and the rest of the file follows
// from that. A mean HIDES THE SPIKE, and on speed and flow the spike is the
// information: twenty seconds through a junction averages to a flat 30 km/h,
// where the honest picture is a bar spanning 0 to 60. It is also what makes a
// bucket this coarse affordable -- what 20 s costs is knowing WHEN inside
// those 20 s something happened, not WHETHER it happened. The trace gives up
// timing resolution and keeps its range, which is the right way round for
// something read at a glance.
//
// ONLY FRESH READINGS ARE PUSHED, and this file cannot enforce that: it does
// not know what a SnapVal is, deliberately. core/Snapshot.h includes THIS
// header, so an include the other way is a cycle -- the caller holds the
// SnapAge and the caller decides. A held reading folded into bucket after
// bucket would draw a flat line for as long as the signal stayed away, which
// is the most confident-looking shape on the chart standing in for the one
// thing nobody knows. The honest mark for a gap is a HOLE IN THE TRACE. This
// is hold-and-dim applied to a chart, and an empty bucket is drawn as nothing
// rather than as the last number carried forward.
//
// THE BANDS ARE FIXED AND NOTHING AUTO-RANGES, for the same reason the flow
// bar in core/Snapshot.h clamps instead of scaling itself: a chart whose full
// scale moves means something different every time it is looked at. Two
// glances a minute apart at an identical shape would be two different stories,
// and there are no axis labels on this panel to catch it with.
//
// SIZE: 4 params x 45 buckets x 2 endpoints x int16_t = 720 bytes, plus the
// seven bytes of header fields. That budget is why the design is buckets at
// all -- a per-sample ring at 1 Hz over the same window is 3600 samples, and
// there is nowhere on this part to put it.
//
// PURE: no ESP-IDF, no Arduino, no allocation. Fixed arrays, and the whole
// thing is one POD -- it can be copied whole, published beside a Snapshot, or
// live on the stack of a host test.
#include <stdint.h>

// The window and how it is cut up. 45 x 20 s = 15 minutes.
//
// 20 s per bucket is the trade named above: min/max means a bucket that coarse
// still shows the spike, and the coarseness is what buys the whole quarter
// hour in 720 bytes. 45 is also about as many columns as a 240 px band can
// draw as separate bars with a gap between them, so the storage and the
// drawing agree on the number by construction rather than by coincidence.
#define HIST_BUCKETS   45u
#define HIST_BUCKET_MS 20000u
#define HIST_WINDOW_MS (HIST_BUCKETS * HIST_BUCKET_MS)   // 900000 ms = 15 min

// The four traces. This is also the drawing order and the order kHistBand is
// written in; nothing else depends on the values.
enum HistParam : uint8_t {
  HIST_FLOW = 0,   // litres per hour
  HIST_SPEED,      // km/h
  HIST_COOLANT,    // degrees C
  HIST_VOLTS,      // volts
  HIST_COUNT,
};

// The FIXED band a parameter is plotted against: `lo` is the bottom of the
// chart and `hi` the top, whatever the data does. Values outside it are the
// renderer's to clamp -- the storage keeps what was measured.
struct HistBand { float lo, hi; };

// FLOW     0..50 L/h    -- the same full scale as the flow bar, for the same
//                          measured reason: idle is under 1 L/h and a cruise
//                          about 10, so a scale drawn to the engine's true
//                          peak would spend its life in the bottom sixth.
// SPEED    0..160 km/h  -- everything this truck is ever actually driven at,
//                          with the top of the chart still reachable.
// VOLTS    11..15 V     -- the entire story sits between a rested 12.2 and a
//                          charging 14.4. On 0..15 that story is the top fifth
//                          of the chart and a failing alternator looks like a
//                          healthy one.
//
// COOLANT'S FLOOR IS 40 AND NOT 0, AND THAT IS A TRADE RATHER THAN AN
// OVERSIGHT. The band worth reading is 70-105 -- warmed up, and the first sign
// of not being -- and an axis starting at 0 squeezes it into a third of the
// height. The price is that a COLD START CLIPS TO THE FLOOR for the first few
// minutes: the trace sits on the bottom line saying "below 40" without saying
// how far below. Accepted knowingly, because warm-up is the one part of the
// coolant story the driver can already read off the gauge, and the part this
// chart exists for is the slow drift that has no other tell.
extern const HistBand kHistBand[HIST_COUNT];

// The ring. Storage is TENTHS of the unit in an int16_t -- see historyPush().
//
// AN EMPTY BUCKET IS lo > hi. Buckets are cleared to lo = INT16_MAX,
// hi = INT16_MIN, so the first sample to arrive satisfies both the min test
// and the max test and the bucket becomes the degenerate range [q, q]. No
// third field has to be kept in step to say "something is in here".
//
// That is the classic empty-range idiom and it is the one thing in this file a
// reader will trip over, so plainly: DO NOT "repair" a bucket whose lo is
// larger than its hi. That IS the representation of nothing having arrived,
// and historyRange() reports it by returning false rather than by returning a
// range.
struct History {
  int16_t  lo[HIST_COUNT][HIST_BUCKETS];
  int16_t  hi[HIST_COUNT][HIST_BUCKETS];
  uint8_t  head;      // the bucket currently filling
  uint32_t headMs;    // the uptime at which that bucket opened
  bool     started;   // false until the first historyTick() supplies a clock
};

// Empty every bucket and leave the clock UNSET.
//
// A History is not usable until the first historyTick(), and that is on
// purpose: there is no honest headMs until a caller hands one over, and zero
// is not a safe stand-in. A ring created at an uptime of several seconds would
// read its own first tick as a several-second gap, and one created at the far
// end of the 49-day clock would read it as a window and a half.
void historyInit(History& h);

// Advance the ring to `nowMs`, opening and clearing buckets as their turns
// come. Call it on the UI tick; it is cheap and idempotent within a bucket.
//
// WRAP-SAFE THROUGHOUT. `nowMs` is a 32-bit uptime in milliseconds and rolls
// over about every 49.7 days, so nothing here ever compares two timestamps
// with < -- the difference of two unsigned values is taken and compared
// against an interval, which stays correct straight through the rollover
// because the wrap cancels.
//
// AND THE ADVANCE IS CAPPED. At or past a full HIST_WINDOW_MS every bucket in
// the ring is older than the window, so there is nothing left to keep: the
// whole thing is cleared and the clock restarted at `nowMs`, instead of
// stepping. See the comment on the cap in History.cpp for why that is not
// merely an optimisation.
void historyTick(History& h, uint32_t nowMs);

// Fold one reading into the bucket currently filling: lo = min, hi = max.
// Does nothing before the first historyTick(), or for a param out of range.
//
// THE CALLER DECIDES WHAT IS WORTH PUSHING. This function has no idea whether
// the number it is given is fresh, held or invented -- see the header comment
// on why only SNAP_FRESH readings may reach it, and why that rule cannot live
// in this file.
void historyPush(History& h, uint8_t param, float v);

// Read one bucket back, in the parameter's own units.
//
// `age` counts backwards from the present: 0 is the bucket currently filling
// and HIST_BUCKETS-1 the oldest still inside the window.
//
// Returns FALSE -- leaving `lo` and `hi` exactly as the caller left them --
// for a param out of range, an age out of range, a ring that has never been
// ticked, and for a bucket nothing arrived in. The untouched out-params are
// deliberate: writing zeros into them would hand the renderer a legitimate
// reading, since 0 km/h and 0 L/h both mean "stopped", and twenty seconds with
// no data would then be indistinguishable from twenty seconds parked. The
// return value is the only signal there is, and nothing may shadow it with a
// plausible number.
bool historyRange(const History& h, uint8_t param, uint8_t age,
                  float& lo, float& hi);
