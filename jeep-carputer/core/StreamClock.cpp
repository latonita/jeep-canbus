#include "StreamClock.h"

#include "LinkLine.h"

void streamClockInit(StreamClock& c, uint8_t tsChars) {
  // The modulus comes from the WIDTH, not from a level, so adding a level
  // cannot leave this behind -- the same reason the gateway's tsValue() is
  // keyed off tsChars() rather than off stream_.
  c.modulus = (tsChars == LINK_LINE_TS_SHORT) ? STREAM_CLOCK_MOD_SHORT : 0u;
  c.lastRaw = 0u;
  c.adapterMs = 0u;
  c.offset = 0u;
  c.creepRem = 0u;
  c.lastHostMs = 0u;
  c.lastMs = 0u;
  c.samples = 0u;
  c.wraps = 0u;
  c.resyncs = 0u;
  c.have = false;
}

uint32_t streamClockMap(StreamClock& c, uint32_t raw, uint32_t hostMs) {
  ++c.samples;

  if (!c.have) {
    // ANCHOR. adapterMs starts at the raw stamp -- there is no reason to
    // normalise it to zero, since only differences of it are ever used -- and
    // the offset is whatever makes this first sample map to hostMs exactly.
    // Both are modular, so a raw stamp larger than hostMs (a gateway that has
    // been up longer than we have, which is the normal case) wraps and still
    // adds back correctly.
    c.have = true;
    c.lastRaw = raw;
    c.adapterMs = raw;
    c.offset = hostMs - raw;
    c.creepRem = 0u;
    c.lastHostMs = hostMs;
    c.lastMs = hostMs;
    return hostMs;
  }

  const uint32_t hostAdvance = hostMs - c.lastHostMs;

  // ---- advance the adapter clock ----
  uint32_t step;
  if (c.modulus == 0u) {
    // The 8-digit width is the adapter's whole millis(), so unsigned
    // subtraction already does the unwrapping and there is no bucket to pick.
    step = raw - c.lastRaw;
  } else {
    step = (raw >= c.lastRaw) ? (raw - c.lastRaw)
                             : (c.modulus - (c.lastRaw - raw));
    if (raw < c.lastRaw) {
      ++c.wraps;
    }
    // WHOLE MODULI THE STAMP CANNOT REPORT. Sixty seconds of silence and
    // sixty-one look identical on a 4-digit stamp, so the host clock says how
    // many minutes to add and the stamp says where inside the last one we are.
    // Rounded to the nearest bucket, which is why the host only has to be right
    // to within thirty seconds -- and it is right to within a stall.
    if (hostAdvance > step + c.modulus / 2u) {
      const uint32_t k = (hostAdvance - step + c.modulus / 2u) / c.modulus;
      step += k * c.modulus;
      c.wraps += k;
    }
  }
  c.lastRaw = raw;
  c.adapterMs += step;

  // ---- track the offset: a MINIMUM, with slow upward creep ----
  //
  // host = adapter + offset + latency and latency >= 0, so the smallest
  // `host - adapter` yet seen is the best estimate and everything above it is
  // one line's queueing delay. Compared SIGNED because both are modular.
  const uint32_t cand = hostMs - c.adapterMs;
  const int32_t d = static_cast<int32_t>(cand - c.offset);

  if (d <= 0) {
    // A lower-latency sample, so a strictly better estimate: take it at once
    // rather than creeping toward it. This is also the whole of the
    // gateway-reboot recovery -- a restart makes adapterMs jump forward by up
    // to a modulus, cand falls by the same amount, and one sample puts the
    // mapping back.
    if (-d > static_cast<int32_t>(STREAM_CLOCK_RESYNC_MS)) {
      ++c.resyncs;
    }
    c.offset = cand;
    c.creepRem = 0u;
  } else if (d > static_cast<int32_t>(STREAM_CLOCK_RESYNC_MS)) {
    // Too far to be explained by queueing. Either the adapter's clock moved
    // under us or we mis-bucketed a wrap; in both cases the standing estimate
    // is the wrong one to keep, and taking the sample re-anchors at the cost of
    // this one frame carrying its own latency.
    ++c.resyncs;
    c.offset = cand;
    c.creepRem = 0u;
  } else {
    // Ordinary case: this sample was late, so it says nothing new about the
    // offset. Let the estimate rise only at the creep rate, and never past the
    // sample itself -- which is what keeps `mapped <= hostMs` true.
    c.creepRem += hostAdvance;
    const uint32_t creep = c.creepRem / STREAM_CLOCK_CREEP_DIV;
    if (creep != 0u) {
      c.creepRem -= creep * STREAM_CLOCK_CREEP_DIV;
      c.offset += (creep > static_cast<uint32_t>(d)) ? static_cast<uint32_t>(d)
                                                     : creep;
    }
  }

  c.lastHostMs = hostMs;

  // offset <= hostMs - adapterMs by construction above, so this cannot exceed
  // hostMs and every unsigned `nowMs() - stamp` age downstream stays a real
  // age rather than four billion.
  uint32_t mapped = c.adapterMs + c.offset;

  // MONOTONIC. A better offset estimate arriving is a step BACKWARDS in the
  // mapping, and an interval computed across it would be negative -- which,
  // unsigned, is an enormous positive one. Flattening it costs that single
  // interval its resolution; letting it through would cost a trip meter a
  // litre. Cannot break the bound above, because lastMs was itself <= the
  // previous hostMs, which is <= this one.
  if (static_cast<int32_t>(mapped - c.lastMs) < 0) {
    mapped = c.lastMs;
  }
  c.lastMs = mapped;
  return mapped;
}

int32_t streamClockSkewMs(const StreamClock& c) {
  if (!c.have) {
    return 0;
  }
  return static_cast<int32_t>(c.lastMs - c.lastHostMs);
}
