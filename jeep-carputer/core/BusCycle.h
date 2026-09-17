#pragma once
// How long to listen, in which mode, and how many requests to spend after.
//
// The car task alternates between listening and asking, because on an
// ELM327-style adapter the two CANNOT overlap: ATMA streams frames until any
// character arrives, and the adapter accepts no commands while it does. A 2.1
// gateway streaming in the background -- AT@SML, or AT@SM on the fallback rung
// -- removes that constraint, since frames arrive continuously on the reader
// task, so the same loop has to behave differently depending on what it is
// talking to.
//
// That choice was three branches inline in `carTask`, and each branch's delay
// carried a real argument in a comment with nothing checking it. The argument
// for the streaming pace is the one worth keeping honest: it is not a
// throttle, it is a SAMPLING RATE. `tripUpdate()` folds fuel flow in by
// integrating rate x interval, and it can only see the broadcast stamps that
// are present when it looks -- so the loop has to come round faster than the
// quickest signal it must not miss. 0x002 is the quickest at 50 ms, so 20 ms
// gives better than two looks per frame. Slow that down and the trips lose
// fuel silently, which is the worst way for this to fail: the number stays
// plausible.
//
// THE STAMPS IT SAMPLES ARE THE GATEWAY'S NOW, under AT@SML, and that does NOT
// weaken the argument above -- it is worth saying so, because it looks as though
// it should. Accurate intervals are no defence against a sample never being
// looked at: two 0x1A7 frames landing between two passes leave the second one's
// rate attributed to both their intervals, which is a real reading spread over
// twice its width, and no clock can undo that. The stamps make the interval
// HONEST; coming round faster than the signal is what makes it COMPLETE.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

// Pace the loop against the quickest broadcast on the bus (0x002, 50 ms).
// See above -- this is a sampling rate, not a throttle.
#define BUS_STREAM_PACE_MS 20u

// How long to sit in ATMA before dropping out to run requests. Long enough
// that the 50 ms signals arrive several times over, short enough that a 5 s
// poll period is still comfortably met.
//
// ONLY used against an adapter older than 2.1.
#define BUS_MONITOR_WINDOW_MS 700u

// No monitor mode at all -- a bench link, or an adapter that refused. Poll
// alone rather than stalling: a short pace keeps the loop responsive to the
// reset and fault requests it also services, and there is no window to
// protect.
#define BUS_POLL_ONLY_PACE_MS 50u

// Requests per pass, then back to listening.
//
// BOUNDED, and that is the point: a module may take its full 400 ms timeout,
// so an unbounded burst could hold the adapter out of monitor mode for
// seconds and starve every broadcast on the bus. Three at 400 ms worst case is
// 1.2 s of deafness against a 700 ms window -- already the larger half, and
// the reason LINK_QUIET_MS has to be as generous as it is.
#define BUS_POLLS_PER_PASS 3u

enum BusListenMode : uint8_t {
  // Frames are already arriving on the reader task; just pace the loop.
  BUS_LISTEN_STREAM = 0,
  // Enter ATMA for a window, then leave it.
  BUS_LISTEN_MONITOR,
  // No listening possible; poll only.
  BUS_LISTEN_NONE,
};

struct BusListenPlan {
  BusListenMode mode;
  uint16_t      delayMs;
};

// `monitorAvailable` is whether the caller's monitorBegin() succeeded. The
// caller has to try it to know, so this is told rather than asked -- keeping
// the I/O on the caller's side is the whole point of the split.
BusListenPlan busListenDecide(bool streaming, bool monitorAvailable);

// ---------------------------------------------------------------------------
// The loop's own health metric.
//
// Not decoration. This loop's period is what bounds how stale anything on the
// panel can be, and its WORST period is what a fuel-integration gap or a
// missed 0x002 would show up as first. It is reported as a rolling worst
// rather than an average because the average hides exactly the excursion that
// matters -- one 2 s pass among fifty 20 ms ones is invisible in a mean and is
// the whole story.
struct BusLoopStats {
  uint32_t lastMs;           // when the previous pass began
  uint32_t worstMs;          // longest period since the last report
  uint32_t reportMs;         // when the last report went out
  uint32_t periodMs;         // the period just measured
  // The worst of the window that just CLOSED, held for the caller to print.
  //
  // Separate from `worstMs` because the window has to be reset at the moment
  // it is reported, and the caller reads the figure after being told. Resetting
  // `worstMs` in place would hand the caller a zero -- the reset destroying the
  // one number the report exists to carry.
  uint32_t reportedWorstMs;
};

#define BUS_LOOP_REPORT_MS 10000u

void busLoopInit(BusLoopStats& s, uint32_t nowMs);

// Call once at the top of each pass. Returns true when a report is due; on
// that pass `reportedWorstMs` holds the worst period of the window just closed
// and the rolling worst starts again from zero -- so each report describes its
// own ten seconds rather than all of uptime.
bool busLoopTick(BusLoopStats& s, uint32_t nowMs);
