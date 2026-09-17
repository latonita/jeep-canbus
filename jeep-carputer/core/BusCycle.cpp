#include "BusCycle.h"

BusListenPlan busListenDecide(bool streaming, bool monitorAvailable) {
  BusListenPlan p;
  if (streaming) {
    // Streaming WINS over monitoring, and the caller must not even try
    // monitorBegin() in this case: ATMA would make the adapter deaf to the
    // requests the loop still wants to send, in exchange for frames it is
    // already receiving.
    p.mode = BUS_LISTEN_STREAM;
    p.delayMs = BUS_STREAM_PACE_MS;
    return p;
  }
  if (monitorAvailable) {
    p.mode = BUS_LISTEN_MONITOR;
    p.delayMs = BUS_MONITOR_WINDOW_MS;
    return p;
  }
  p.mode = BUS_LISTEN_NONE;
  p.delayMs = BUS_POLL_ONLY_PACE_MS;
  return p;
}

void busLoopInit(BusLoopStats& s, uint32_t nowMs) {
  s.lastMs = nowMs;
  s.worstMs = 0u;
  s.reportMs = nowMs;
  s.periodMs = 0u;
  s.reportedWorstMs = 0u;
}

bool busLoopTick(BusLoopStats& s, uint32_t nowMs) {
  // Wrap-safe, like every clock comparison in core/. nowMs truncates a 64-bit
  // timer and rolls over every 49.7 days; an absolute compare would report a
  // period of nearly 2^32 ms once per roll and then poison the rolling worst
  // for the rest of the key cycle, because the worst only ever goes UP between
  // reports.
  s.periodMs = nowMs - s.lastMs;
  s.lastMs = nowMs;
  if (s.periodMs > s.worstMs) {
    s.worstMs = s.periodMs;
  }

  if ((nowMs - s.reportMs) < BUS_LOOP_REPORT_MS) {
    return false;
  }
  s.reportMs = nowMs;
  s.reportedWorstMs = s.worstMs;
  s.worstMs = 0u;
  return true;
}
