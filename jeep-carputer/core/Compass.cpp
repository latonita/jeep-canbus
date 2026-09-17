#include "Compass.h"

#include <math.h>

// The eight names, indexed by the octant enum the module itself sends in
// byte 0. The empty string is the ninth entry ON PURPOSE rather than a
// separate return: one table, one bounds test, and no path that can fall off
// the end of it.
static const char* const kOctantNames[9] = {
  "N", "NE", "E", "SE", "S", "SW", "W", "NW", ""
};

#define OCTANTS 8u
#define DEG_PER_OCTANT 45.0f

float compassNorm(float deg) {
  float d = fmodf(deg, 360.0f);
  if (d < 0.0f) {
    d += 360.0f;
  }
  // TWO EDGES THAT ONLY EXIST IN FLOAT, and both of them produce a heading
  // outside the half-open range this function promises.
  //
  //   * fmodf(-360, 360) is NEGATIVE ZERO, which is not < 0.0f, so it skips
  //     the branch above and comes out as -0.0f.
  //   * A tiny negative -- fmodf(-1e-7f, 360) -- plus 360.0f rounds to exactly
  //     360.0f in single precision, because 360 has no bits left to represent
  //     the difference. So the wrap can hand back the one value it exists to
  //     eliminate.
  //
  // Both land on the same test, and both must return a clean +0.0f: callers
  // compare headings for equality and index tables off them.
  if (d >= 360.0f || d == 0.0f) {
    d = 0.0f;
  }
  return d;
}

const char* compassOctantName(uint8_t octant) {
  if (octant >= OCTANTS) {
    return kOctantNames[OCTANTS];   // "" -- never invent a direction
  }
  return kOctantNames[octant];
}

uint8_t compassOctantFor(float deg) {
  const float d = compassNorm(deg);          // 0..360, exclusive at the top
  // ROUND, not floor -- the car's own byte 0 agrees with round 99.46 % of the
  // time and with floor 41.65 %. See the header for the full scoring and for
  // why all sixteen disagreements are quantisation rather than rule.
  //
  // + 0.5 and truncate is the round: d/45 is in [0, 8), so the sum is in
  // [0.5, 8.5) and the truncation lands on 0..8. The 8 is the top half of the
  // last sector -- 337.5..360, which is NORTH -- and the mask is what wraps it
  // home. That single case is the whole reason the sector containing 0 is the
  // only one split across the ends of the range.
  const unsigned o = (unsigned)(d / DEG_PER_OCTANT + 0.5f);
  return (uint8_t)(o & (OCTANTS - 1u));
}

float compassArc(float from, float to) {
  // Both normalised first, so the difference is strictly inside -360..+360 and
  // ONE correction in each direction is provably enough. Subtracting raw
  // inputs would need a loop, or would silently leave a 720 deg argument
  // three turns out.
  float d = compassNorm(to) - compassNorm(from);
  if (d > 180.0f) {
    d -= 360.0f;
  }
  if (d <= -180.0f) {
    // <=, not <, is what sends the antipodal case to +180 rather than -180.
    // See the header: a fixed sign is what stops a glide across the far side
    // of the rose flipping direction on a hundredth of a degree.
    d += 360.0f;
  }
  return d;
}

float compassGlide(float shown, float target, float alpha) {
  const float to = compassNorm(target);
  const float arc = compassArc(shown, to);
  const float mag = arc < 0.0f ? -arc : arc;

  // The far end: a genuine 96 deg hairpin sample. Gliding it lags.
  if (mag >= COMPASS_SNAP_DEG) {
    return to;
  }
  // The near end: a proportional glide is asymptotic and would never arrive.
  if (mag <= COMPASS_SETTLE_DEG) {
    return to;
  }
  // A caller outside 0..1 is a caller with a bug, but this function is on the
  // path that decides what a driver reads, so it is clamped rather than
  // trusted: alpha <= 0 holds still, alpha >= 1 arrives. Neither can throw the
  // tape past the target and back, which an unclamped 1.8 would do every tick.
  if (alpha <= 0.0f) {
    return compassNorm(shown);
  }
  if (alpha >= 1.0f) {
    return to;
  }
  return compassNorm(compassNorm(shown) + arc * alpha);
}

uint8_t compassMarks(CompassMark* out, uint8_t max, float deg,
                     uint16_t w, uint16_t spanDeg) {
  // A zero width or a zero span is a degenerate tape, not an error worth a
  // return code: there is nothing to draw and the division would be by zero.
  if (out == nullptr || max == 0u || w == 0u || spanDeg == 0u) {
    return 0u;
  }

  const float centreDeg = compassNorm(deg);
  const float half = (float)spanDeg * 0.5f;
  const float pxPerDeg = (float)w / (float)spanDeg;
  // INTEGER half-width: the centre marker is drawn on a pixel, so the tape's
  // middle is w/2 rounded down and not w/2.0. On an odd width that puts the
  // extreme left mark at -0.5 px, which the range test below then drops --
  // which is why that test is against the emitted integer and not the float.
  const float cx = (float)(w / 2u);

  const int tick = (int)COMPASS_TICK_DEG;

  // The walk is in UNWRAPPED degrees and `k` is a SIGNED tick index, so the
  // window that straddles the seam is a plain contiguous run: at a heading of
  // 5 deg over a 90 deg span the run is -30..45, and only the labels wrap.
  int k = (int)floorf((centreDeg - half) / (float)tick);
  // floorf is exact for these magnitudes, but the division that feeds it is
  // not, so one nudge covers a first tick that landed a ULP short of the left
  // edge. It can iterate at most once.
  while ((float)(k * tick) < centreDeg - half) {
    ++k;
  }

  uint8_t n = 0u;
  for (; n < max; ++k) {
    const float m = (float)(k * tick);
    if (m > centreDeg + half) {
      break;
    }
    // (m - centreDeg) IS compassArc(centreDeg, m): the walk is bounded to
    // +/- spanDeg/2 of the centre, so for any span up to 360 it is already the
    // shortest arc, and using it directly is what keeps x monotonic in k.
    // Going through compassArc() here would flip sign at the antipode and put
    // the last mark of a full-circle tape at the wrong end.
    const float x = cx + (m - centreDeg) * pxPerDeg;
    const int xi = (int)floorf(x + 0.5f);
    if (xi < 0 || xi > (int)w) {
      continue;
    }
    const float label = compassNorm(m);
    const uint16_t d = (uint16_t)(label + 0.5f);   // exact: always a 15 deg step
    out[n].x = (int16_t)xi;
    out[n].deg = d;
    out[n].major = ((d % 45u) == 0u);
    ++n;
  }
  return n;
}

// Degrees to radians. Spelled out rather than taken from M_PI, which is not in
// the C++ standard headers at all and arrives only by way of a POSIX feature
// macro -- a dependency this file has no other reason to have.
#define DEG_TO_RAD 0.017453292519943295f

// Round a float coordinate to the nearest pixel and land it in an int16_t.
//
// floorf(x + 0.5f) rather than a cast, and the difference is not pedantry: a
// cast truncates TOWARD ZERO, so it rounds down above the origin and UP below
// it. Every coordinate here is an offset from a centre that is itself hundreds
// of pixels from zero, so truncation would bias the whole ring consistently
// toward the top left by half a pixel -- a full pixel across a diameter, and
// plainly visible on a dial with a 100 px radius as a circle that is not where
// its centre says it is.
//
// The range clamp is for the cast and nothing else. An int16_t cannot hold
// every product of an int16_t radius and the 1.048 that the projection can
// reach, and a float-to-integer conversion that does not fit is undefined --
// which the host suites run UBSan specifically to catch, so it would abort the
// run rather than produce a wrong pixel. A caller passing a radius that big has
// a bug either way; this decides what the bug looks like.
static int16_t dialRound(float v) {
  const float r = floorf(v + 0.5f);
  if (r < -32768.0f) {
    return (int16_t)-32768;
  }
  if (r > 32767.0f) {
    return (int16_t)32767;
  }
  return (int16_t)r;
}

uint8_t compassDial(CompassDialMark* out, uint8_t max, float heading,
                    int16_t cx, int16_t cy, int16_t rx, int16_t ry,
                    uint8_t dotMin, uint8_t dotMax) {
  if (out == nullptr || max == 0u) {
    return 0u;
  }

  // The ring is computed WHOLE before any of it is written, because the
  // emission order is far-first and that is not the order the bearings are
  // walked in. Writing straight into `out` and sorting in place is not
  // available: `max` may be smaller than the ring, and the marks that get
  // dropped are the near ones at the END of the order, which cannot be known
  // until every v is in hand. 24 floats twice over and an index is ~200 bytes
  // of stack, once, on a task that has kilobytes.
  float uu[COMPASS_DIAL_MARKS];
  float vv[COMPASS_DIAL_MARKS];
  uint8_t order[COMPASS_DIAL_MARKS];

  const float head = compassNorm(heading);
  for (unsigned i = 0u; i < COMPASS_DIAL_MARKS; ++i) {
    const float bearing = (float)(i * COMPASS_DIAL_TICK_DEG);
    // compassArc() rather than (bearing - head), which is the whole of the
    // wrap handling. sin and cos would not mind an unwrapped argument, but the
    // SORT below and `depth` both read v, and a v computed from a 350 deg
    // difference is the same number as one computed from -10 -- so getting
    // this wrong would be invisible here and wrong nowhere in particular.
    const float a = compassArc(head, bearing) * DEG_TO_RAD;
    uu[i] = sinf(a);
    vv[i] = cosf(a);
    order[i] = (uint8_t)i;
  }

  // Insertion sort by DESCENDING v -- farthest first -- with u DESCENDING as
  // the tie-break, so the clockwise half of a tied pair comes out first.
  //
  // THE TIE-BREAK IS WHAT MAKES THE ORDER RIGID UNDER ROTATION, and the
  // obvious alternative is not. Eleven pairs are tied at any heading, because
  // a bearing and its mirror about the screen's vertical share a v exactly
  // (cos is even, and compassArc hands back exactly negated arcs for the
  // headings that land on a tick). Leaving those to a STABLE sort ties their
  // order to the position of the 0/360 seam in the walk above: at a heading of
  // 0 the pair is 15 and 345, which the walk meets at indexes 1 and 23, so 15
  // wins; at a heading of 15 the pair is 30 and 0, met at 2 and 0, so the
  // ANTI-clockwise one wins instead. The ring would then be emitted in an
  // order that flips as the truck turns through north -- and a caller holding
  // widgets in that order would see two dots swap places for no reason the
  // driver did anything to cause. Ordering on u instead is a property of the
  // geometry: the k-th mark out is always the same arc from the heading.
  //
  // 24 elements arriving as two already-sorted runs. There is no case here
  // worth a better algorithm and a good deal of value in an obvious one.
  for (unsigned i = 1u; i < COMPASS_DIAL_MARKS; ++i) {
    const uint8_t cur = order[i];
    unsigned j = i;
    while (j > 0u && (vv[order[j - 1u]] < vv[cur] ||
                      (vv[order[j - 1u]] == vv[cur] &&
                       uu[order[j - 1u]] < uu[cur]))) {
      order[j] = order[j - 1u];
      --j;
    }
    order[j] = cur;
  }

  // The size bounds, ORDERED. A caller that passes them the wrong way round
  // gets a ring whose dots grow with distance -- wrong, and obviously wrong on
  // the glass -- rather than a size that leaves the pair it was given.
  const float sizeLo = (float)(dotMin < dotMax ? dotMin : dotMax);
  const float sizeHi = (float)(dotMin < dotMax ? dotMax : dotMin);

  uint8_t n = 0u;
  while (n < max && n < COMPASS_DIAL_MARKS) {
    const uint8_t i = order[n];
    const float v = vv[i];
    // THE PERSPECTIVE DIVIDE. Without it this is an orthographic ellipse: the
    // far half and the near half squash equally and the ring reads flat. See
    // the header for the 38-up-against-71-down that this one term buys.
    const float f = 1.0f / (1.0f + v * COMPASS_DIAL_E);

    out[n].x = dialRound((float)cx + (float)rx * uu[i] * f);
    out[n].y = dialRound((float)cy - (float)ry * v * f);

    // v is a cosine, so (v + 1) / 2 is in 0..1 by construction and the scaled
    // value cannot leave 0..255. Clamped anyway, because what is being relied
    // on is a library function's range promise and the cost of being wrong
    // about it is an out-of-range float-to-integer conversion -- undefined,
    // and an abort rather than a wrong pixel under the sanitizers.
    float d = (v + 1.0f) * 0.5f * 255.0f;
    if (d < 0.0f) {
      d = 0.0f;
    }
    if (d > 255.0f) {
      d = 255.0f;
    }
    const uint8_t depth = (uint8_t)floorf(d + 0.5f);
    out[n].depth = depth;

    // dotMax at depth 0 (nearest) down to dotMin at depth 255 (farthest). The
    // interpolation is written against the ROUNDED depth rather than against v
    // directly, so the two fields cannot disagree about where a mark sits --
    // a caller mixing a colour by depth and a radius by size is drawing one
    // dot, and it would be a strange dot whose two halves were computed from
    // different numbers.
    float s = (float)dotMax + ((float)dotMin - (float)dotMax) *
                              ((float)depth / 255.0f);
    if (s < sizeLo) {
      s = sizeLo;
    }
    if (s > sizeHi) {
      s = sizeHi;
    }
    out[n].size = (uint8_t)floorf(s + 0.5f);

    const uint16_t deg = (uint16_t)(i * COMPASS_DIAL_TICK_DEG);
    out[n].deg = deg;
    out[n].major = ((deg % 45u) == 0u);
    ++n;
  }
  return n;
}
