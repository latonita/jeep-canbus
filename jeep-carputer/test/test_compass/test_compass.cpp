// Host tests for core/Compass.cpp.
//
// Three things here can be wrong while looking completely right on the glass,
// and this suite exists for those three:
//
//  1. THE OCTANT RULE. round(deg/45) and floor(deg/45) produce the same answer
//     for a bit over 40 % of headings, so a floor-binned compass is right
//     often enough to survive a test drive and a demo. The capture settles it
//     numerically -- the module sends its OWN octant in byte 0 of 0x1B4
//     alongside the heading in byte 1, so the two rules can simply be scored:
//     round agrees 2937/2953 (99.46 %) and floor 1230/2953 (41.65 %). The
//     regression below picks headings where the two rules DISAGREE, because a
//     test built on 50 deg -- where they agree -- would pass under either.
//
//  2. THE WRAP. Every function here is an arithmetic identity away from being
//     off by one turn, and the 0/360 seam is the only place it shows. The tape
//     is the worst of them: the visible window straddles the seam for a tenth
//     of all headings, and a loop that walks a 0..345 table assuming the
//     visible marks are contiguous in it drops half the tape at north.
//
//  3. THE RANGE PROMISE. compassGlide() returns a heading that is fed straight
//     back into itself on the next tick, so a single escape outside 0..360
//     does not self-heal -- it compounds. It is swept rather than spot-checked.
//
//  4. THE PERSPECTIVE DIVIDE. compassDial() projects the ring onto a disc lying
//     on the ground, and the `f = 1 / (1 + v*e)` term is the only thing that
//     makes it a disc rather than an oval. Delete it and every dot is still on
//     a closed curve, still ordered, still the right colour and the right size,
//     still symmetric left to right -- an orthographic ellipse passes every
//     obvious test. The ONE thing it cannot fake is the near/far ASYMMETRY:
//     38 px above the centre against 71 px below, where orthographic gives ry
//     both ways. That pair of numbers is asserted by name below, and it is the
//     whole regression test for the projection.
//
// Nothing here needs a car, a board or a log file: the log's verdict is
// already compiled into the constants and the comments, and what is asserted
// is that the code implements that verdict.
#include "doctest_min.h"

#include "Compass.h"

#include <math.h>   // sqrtf, for the dial's bounding box -- see checkDial()

// The shipped tape. 156 px of the 240 px portrait band across a 90 deg span,
// which makes the scale 1.7333 px/deg -- so an octant is 78 px and a minor
// tick 26 px. Both of those are asserted below as the geometry the layout was
// drawn against, not merely as whatever the arithmetic happens to produce.
#define TAPE_W    156u
#define TAPE_SPAN 90u

// Everything a caller may assume about one call, checked on every call the
// sweeps make: the count fits, the pixels are inside the tape, the order never
// goes backwards, the headings are real ticks, and `major` is the octant test
// and nothing else.
static void checkMarksInvariants(const CompassMark* m, uint8_t n, uint8_t cap,
                                 uint16_t w) {
  CHECK(n <= cap);
  for (uint8_t i = 0u; i < n; ++i) {
    CHECK(m[i].x >= 0);
    CHECK(m[i].x <= (int16_t)w);
    CHECK_EQ(m[i].deg % COMPASS_TICK_DEG, 0);
    CHECK(m[i].deg < 360u);
    // The whole of the major/minor rule. 15 divides 45, so every third tick is
    // an octant -- if this ever needs a table, the tick spacing has changed
    // and so has the layout.
    CHECK_EQ(m[i].major ? 1 : 0, (m[i].deg % 45u) == 0u ? 1 : 0);
    if (i > 0u) {
      CHECK(m[i].x >= m[i - 1].x);
    }
  }
}

// The dial geometry these tests are written against. 120/100 is the centre of
// the portrait band, and rx 100 / ry 50 is a 2:1 disc -- chosen here because
// every expected pixel below is then hand-computable to the tenth: 50/1.3 is
// 38.46 and 50/0.7 is 71.43, which round to 38 and 71 with no ambiguity about
// which way a half goes.
#define DIAL_CX 120
#define DIAL_CY 100
#define DIAL_RX 100
#define DIAL_RY 50
#define DIAL_DOT_MIN 2u
#define DIAL_DOT_MAX 8u

// Everything a caller may assume about one compassDial() call, checked on every
// call the sweeps make.
//
// THE BOUNDING BOX IS DERIVED, not eyeballed, and both halves of it are a
// statement about the projection rather than a guard rail:
//
//   y = cy - ry*v/(1 + e*v) is MONOTONIC in v (the derivative is
//   ry/(1 + e*v)^2, always positive), so y runs from cy - ry/(1+e) at v = +1
//   to cy + ry/(1-e) at v = -1. That box is ASYMMETRIC about cy, which is the
//   projection's whole point -- an orthographic ring fits inside +/- ry and
//   this one deliberately does not.
//
//   |x - cx| = rx * |sin a| / (1 + e*cos a) peaks at cos a = -e, where it is
//   rx / sqrt(1 - e^2) -- 1.0483 rx at e = 0.30. So the ring is slightly WIDER
//   than 2*rx, on the near side, which is another thing an orthographic
//   implementation cannot produce.
//
// One px of slack each way, for the rounding.
static void checkDial(const CompassDialMark* d, uint8_t n, uint8_t cap,
                      int16_t cx, int16_t cy, int16_t rx, int16_t ry,
                      uint8_t dotMin, uint8_t dotMax) {
  CHECK(n <= cap);
  const float e = COMPASS_DIAL_E;
  const float xLim = (float)rx / sqrtf(1.0f - e * e) + 1.0f;
  const float yUp  = (float)ry / (1.0f + e) + 1.0f;   // above cy: the FAR side
  const float yDn  = (float)ry / (1.0f - e) + 1.0f;   // below cy: the NEAR side
  const uint8_t lo = dotMin < dotMax ? dotMin : dotMax;
  const uint8_t hi = dotMin < dotMax ? dotMax : dotMin;

  uint32_t seen = 0u;
  for (uint8_t i = 0u; i < n; ++i) {
    CHECK_EQ(d[i].deg % COMPASS_DIAL_TICK_DEG, 0);
    CHECK(d[i].deg < 360u);
    // Each bearing at most once. The ring is a SET of 24 bearings, not a walk
    // that could revisit a tick after a wrap.
    const uint32_t bit = 1u << (d[i].deg / COMPASS_DIAL_TICK_DEG);
    CHECK_EQ(seen & bit, 0);
    seen |= bit;
    // Same major/minor rule as the tape: 15 divides 45, so it stays arithmetic.
    CHECK_EQ(d[i].major ? 1 : 0, (d[i].deg % 45u) == 0u ? 1 : 0);
    // Inside the derived box.
    CHECK((float)d[i].x >= (float)cx - xLim);
    CHECK((float)d[i].x <= (float)cx + xLim);
    CHECK((float)d[i].y >= (float)cy - yUp);
    CHECK((float)d[i].y <= (float)cy + yDn);
    // Size never leaves the caller's own two numbers, whichever order they
    // arrived in.
    CHECK(d[i].size >= lo);
    CHECK(d[i].size <= hi);
    if (i > 0u) {
      // FAR FIRST. depth is a monotonic function of v and the emission is
      // ordered by descending v, so this one comparison is the whole ordering
      // promise -- and it is what a caller assigning widgets in order relies on
      // for its painter's ordering.
      CHECK(d[i].depth <= d[i - 1].depth);
    }
  }
}

TEST_MAIN_BEGIN()

  // --- compassNorm -------------------------------------------------------
  {
    CHECK_NEAR(compassNorm(0.0f), 0.0f, 1e-4f);
    CHECK_NEAR(compassNorm(359.9f), 359.9f, 1e-3f);

    // Negatives wrap forward.
    CHECK_NEAR(compassNorm(-1.0f), 359.0f, 1e-4f);
    CHECK_NEAR(compassNorm(-90.0f), 270.0f, 1e-4f);
    CHECK_NEAR(compassNorm(-359.5f), 0.5f, 1e-3f);

    // >= 360 wraps back, including several turns out. 1085 = 3*360 + 5.
    CHECK_NEAR(compassNorm(361.0f), 1.0f, 1e-4f);
    CHECK_NEAR(compassNorm(1085.0f), 5.0f, 1e-3f);
    CHECK_NEAR(compassNorm(-1085.0f), 355.0f, 1e-3f);

    // EXACTLY 360 IS 0, not 360. The range is half-open, which is the property
    // every other function here leans on when it subtracts two normalised
    // headings. A 360 leaking out would put compassArc() one whole turn away
    // from the answer it corrects for, and its single correction each way is
    // only provably enough because this cannot happen.
    CHECK_NEAR(compassNorm(360.0f), 0.0f, 1e-6f);
    CHECK(compassNorm(360.0f) < 360.0f);

    // A large multiple, from both sides. These are the ones that go through
    // the float edges the implementation guards: fmodf(-720, 360) is negative
    // zero, and a tiny negative plus 360.0f rounds UP to exactly 360.0f.
    CHECK_NEAR(compassNorm(720.0f), 0.0f, 1e-6f);
    CHECK_NEAR(compassNorm(-720.0f), 0.0f, 1e-6f);
    CHECK_NEAR(compassNorm(36000.0f), 0.0f, 1e-3f);
    CHECK_NEAR(compassNorm(-36000.0f), 0.0f, 1e-3f);
    CHECK_NEAR(compassNorm(-1e-7f), 0.0f, 1e-6f);
    CHECK(compassNorm(-1e-7f) < 360.0f);
    CHECK(compassNorm(-720.0f) >= 0.0f);

    // And the promise in general, over a sweep that crosses the seam from
    // eight turns below to eight above.
    for (int i = -2880; i <= 2880; ++i) {
      const float d = compassNorm((float)i + 0.25f);
      CHECK(d >= 0.0f);
      CHECK(d < 360.0f);
    }
  }

  // --- compassOctantName -------------------------------------------------
  {
    CHECK_STR(compassOctantName(0u), "N");
    CHECK_STR(compassOctantName(1u), "NE");
    CHECK_STR(compassOctantName(2u), "E");
    CHECK_STR(compassOctantName(3u), "SE");
    CHECK_STR(compassOctantName(4u), "S");
    CHECK_STR(compassOctantName(5u), "SW");
    CHECK_STR(compassOctantName(6u), "W");
    CHECK_STR(compassOctantName(7u), "NW");

    // NEVER INVENT A DIRECTION. byte 0 of 0x1B4 has only ever been observed at
    // 0..7, so anything else means this is not the frame we think it is, and
    // the honest answer is the empty string -- exactly as ignitionFrom()
    // answers IGN_UNKNOWN rather than guessing a key position. An out-of-range
    // octant that silently read "N" would put a confident compass point on the
    // glass on the strength of a byte nobody understands.
    CHECK_STR(compassOctantName(8u), "");
    CHECK_STR(compassOctantName(9u), "");
    CHECK_STR(compassOctantName(255u), "");

    // Every value, so the bounds test cannot be off by one at either end.
    for (unsigned i = 0u; i < 256u; ++i) {
      const char* s = compassOctantName((uint8_t)i);
      CHECK(s != nullptr);
      CHECK_EQ(s[0] == '\0' ? 1 : 0, i >= 8u ? 1 : 0);
    }
  }

  // --- compassOctantFor: centres, boundaries, wrap ------------------------
  {
    // Sector CENTRES are on the octants themselves.
    for (unsigned o = 0u; o < 8u; ++o) {
      CHECK_EQ(compassOctantFor((float)o * 45.0f), o);
    }

    // Both sides of every boundary. The boundaries are the half-degrees
    // 22.5 / 67.5 / ... / 337.5, which is what "centres at 0/45/90" MEANS --
    // and is the whole difference from floor binning, which would put the
    // boundaries on the octants and the centres on the halves.
    CHECK_EQ(compassOctantFor(22.4f), 0);
    CHECK_EQ(compassOctantFor(22.6f), 1);
    CHECK_EQ(compassOctantFor(67.4f), 1);
    CHECK_EQ(compassOctantFor(67.6f), 2);
    CHECK_EQ(compassOctantFor(112.4f), 2);
    CHECK_EQ(compassOctantFor(112.6f), 3);
    CHECK_EQ(compassOctantFor(157.4f), 3);
    CHECK_EQ(compassOctantFor(157.6f), 4);
    CHECK_EQ(compassOctantFor(202.4f), 4);
    CHECK_EQ(compassOctantFor(202.6f), 5);
    CHECK_EQ(compassOctantFor(247.4f), 5);
    CHECK_EQ(compassOctantFor(247.6f), 6);
    CHECK_EQ(compassOctantFor(292.4f), 6);
    CHECK_EQ(compassOctantFor(292.6f), 7);

    // THE LAST BOUNDARY IS THE SEAM. 337.5 is the only sector edge whose
    // sector is split across the ends of the range, so this is the one that
    // needs the mod and the one a naive clamp gets wrong by returning 8.
    CHECK_EQ(compassOctantFor(337.4f), 7);
    CHECK_EQ(compassOctantFor(337.6f), 0);
    CHECK_EQ(compassOctantFor(358.0f), 0);   // the top of byte 1's range
    CHECK_EQ(compassOctantFor(359.9f), 0);
    CHECK_EQ(compassOctantFor(360.0f), 0);
    CHECK_EQ(compassOctantFor(-1.0f), 0);    // 359
    CHECK_EQ(compassOctantFor(-23.0f), 7);   // 337
    CHECK_EQ(compassOctantFor(722.0f), 0);   // 2

    // Nothing out of range, ever -- an octant is used to index a name table.
    for (int i = -720; i <= 1440; ++i) {
      CHECK(compassOctantFor((float)i * 0.5f) < 8u);
    }
  }

  // --- compassOctantFor: ROUND IS THE RULE, FLOOR IS NOT ------------------
  {
    // 50 deg is the case that proves nothing: round(50/45) = 1 and
    // floor(50/45) = 1, so a suite built on headings like this passes under
    // either rule. It is asserted anyway, as the control.
    CHECK_EQ(compassOctantFor(50.0f), 1);

    // THESE ARE THE DISCRIMINATING ONES. In each, round and floor give
    // DIFFERENT answers, and the answer asserted is round's:
    //
    //     30 deg -> round 1 (NE), floor 0 (N)
    //     70 deg -> round 2 (E),  floor 1 (NE)
    //
    // Headings of exactly this kind -- the top half of a sector -- are 50 % of
    // the circle, and they are precisely where the capture's 58 % floor
    // disagreement lives: floor matched the module's own byte 0 in only
    // 1230 of 2953 frames against round's 2937.
    CHECK_EQ(compassOctantFor(30.0f), 1);
    CHECK_EQ(compassOctantFor(70.0f), 2);
    CHECK_EQ(compassOctantFor(110.0f), 2);   // round 2, floor 2 -- lower half
    CHECK_EQ(compassOctantFor(115.0f), 3);   // round 3, floor 2
    CHECK_EQ(compassOctantFor(340.0f), 0);   // round 0 (wrapped from 8), floor 7
    CHECK_EQ(compassOctantFor(200.0f), 4);   // round 4, floor 4
    CHECK_EQ(compassOctantFor(250.0f), 6);   // round 6, floor 5

    // Stated as a property rather than as seven cases: across the whole
    // circle, round and floor must disagree on close to half of it. If this
    // ever falls to zero somebody has quietly changed the rule back and every
    // spot check above would still pass on the sector centres.
    unsigned differ = 0u;
    for (unsigned deg = 0u; deg < 360u; ++deg) {
      const unsigned flr = (deg / 45u) & 7u;
      if (compassOctantFor((float)deg) != flr) {
        ++differ;
      }
    }
    CHECK(differ > 150u);
    CHECK(differ < 200u);

    // THE CAPTURE'S SIXTEEN DISAGREEMENTS, reproduced deliberately. These are
    // the three raw counts where the module's byte 0 and round(byte1*2/45)
    // differ, and they are quantisation, not error: byte 1 is floor(true/2),
    // so each of these sits half a degree BELOW a boundary the module has
    // already crossed. The module says the octant one higher in all sixteen.
    // Asserting our side keeps the behaviour pinned and stops anyone "fixing"
    // it with a fudge factor -- the missing half degree is not recoverable
    // from a 2 deg/count signal, and a +0.5 nudge would break the other 2937.
    CHECK_EQ(compassOctantFor(146.0f * COMPASS_DEG_PER_COUNT), 6);  // 292.0, module 7
    CHECK_EQ(compassOctantFor(101.0f * COMPASS_DEG_PER_COUNT), 4);  // 202.0, module 5
    CHECK_EQ(compassOctantFor(11.0f * COMPASS_DEG_PER_COUNT), 0);   // 22.0,  module 1
    // One count further on and both agree again, which is what bounds the
    // error at the +/-1 deg the scale itself costs.
    CHECK_EQ(compassOctantFor(147.0f * COMPASS_DEG_PER_COUNT), 7);  // 294.0
    CHECK_EQ(compassOctantFor(12.0f * COMPASS_DEG_PER_COUNT), 1);   // 24.0

    // The documented ceiling, which byte 1 is allowed to reach even though the
    // drive only ever got to 178.
    CHECK_EQ(compassOctantFor((float)COMPASS_RAW_MAX * COMPASS_DEG_PER_COUNT), 0);
    CHECK_NEAR((float)COMPASS_RAW_MAX * COMPASS_DEG_PER_COUNT, 358.0f, 1e-4f);
  }

  // --- compassArc --------------------------------------------------------
  {
    CHECK_NEAR(compassArc(0.0f, 0.0f), 0.0f, 1e-4f);
    CHECK_NEAR(compassArc(90.0f, 90.0f), 0.0f, 1e-4f);

    // Sign: positive is clockwise, the direction a heading increases.
    CHECK_NEAR(compassArc(0.0f, 90.0f), 90.0f, 1e-4f);
    CHECK_NEAR(compassArc(90.0f, 0.0f), -90.0f, 1e-4f);
    CHECK_NEAR(compassArc(45.0f, 60.0f), 15.0f, 1e-4f);
    CHECK_NEAR(compassArc(60.0f, 45.0f), -15.0f, 1e-4f);

    // THE WRAP, which is the entire reason this function exists rather than a
    // subtraction. The naive difference is -340 and +340; the truth is 20 deg
    // of steering either way.
    CHECK_NEAR(compassArc(350.0f, 10.0f), 20.0f, 1e-4f);
    CHECK_NEAR(compassArc(10.0f, 350.0f), -20.0f, 1e-4f);
    CHECK_NEAR(compassArc(359.0f, 1.0f), 2.0f, 1e-4f);
    CHECK_NEAR(compassArc(1.0f, 359.0f), -2.0f, 1e-4f);

    // Unnormalised inputs are normalised first, so a caller that has not
    // wrapped its own arithmetic still gets the short answer rather than one
    // several turns out.
    CHECK_NEAR(compassArc(-10.0f, 10.0f), 20.0f, 1e-4f);
    CHECK_NEAR(compassArc(710.0f, 10.0f), 20.0f, 1e-4f);

    // Just inside 180 on each side, where the correction flips.
    CHECK_NEAR(compassArc(0.0f, 179.0f), 179.0f, 1e-4f);
    CHECK_NEAR(compassArc(0.0f, 181.0f), -179.0f, 1e-4f);

    // THE 180 EDGE. Antipodal is equally short both ways, so the tie is broken
    // at a FIXED sign: +180 whichever end you start from. The alternative is a
    // direction that flips as the target wobbles by a hundredth of a degree,
    // which on a glide reads as the tape twitching at a heading it has no
    // business caring about.
    CHECK_NEAR(compassArc(0.0f, 180.0f), 180.0f, 1e-4f);
    CHECK_NEAR(compassArc(180.0f, 0.0f), 180.0f, 1e-4f);
    CHECK_NEAR(compassArc(90.0f, 270.0f), 180.0f, 1e-4f);
    CHECK_NEAR(compassArc(270.0f, 90.0f), 180.0f, 1e-4f);

    // The range promise, and the identity that makes it useful: walking the
    // arc from `from` always lands on `to`.
    for (int a = 0; a < 360; a += 7) {
      for (int b = 0; b < 360; b += 11) {
        const float arc = compassArc((float)a, (float)b);
        CHECK(arc >= -180.0f);
        CHECK(arc <= 180.0f);
        CHECK_NEAR(compassNorm((float)a + arc), (float)b, 1e-2f);
      }
    }
  }

  // --- compassGlide ------------------------------------------------------
  {
    // CONVERGES, and actually ARRIVES. A proportional ease is asymptotic, so
    // without COMPASS_SETTLE_DEG this loop would approach 40 for ever and the
    // tape would never sit still -- which on a 2 deg/count signal is motion
    // the car cannot justify.
    float shown = 0.0f;
    for (int i = 0; i < 64; ++i) {
      const float next = compassGlide(shown, 40.0f, 0.25f);
      CHECK(next >= 0.0f);
      CHECK(next < 360.0f);
      // Monotonic toward the target, never past it.
      CHECK(next >= shown - 1e-4f);
      CHECK(next <= 40.0f + 1e-4f);
      shown = next;
    }
    CHECK_NEAR(shown, 40.0f, 1e-4f);
    // ...and having arrived, it stays.
    CHECK_NEAR(compassGlide(shown, 40.0f, 0.25f), 40.0f, 1e-4f);

    // CLOSES THE SHORT WAY ACROSS THE WRAP. 350 -> 10 is 20 deg clockwise; a
    // glide that subtracted instead of arcing would sweep 340 deg backwards,
    // through south, to arrive at the same place.
    CHECK_NEAR(compassGlide(350.0f, 10.0f, 0.25f), 355.0f, 1e-3f);
    CHECK_NEAR(compassGlide(350.0f, 10.0f, 0.5f), 0.0f, 1e-3f);
    CHECK_NEAR(compassGlide(10.0f, 350.0f, 0.5f), 0.0f, 1e-3f);
    CHECK_NEAR(compassGlide(5.0f, 355.0f, 0.5f), 0.0f, 1e-3f);
    {
      // The same thing as a walk: from 340, target 20, must pass through 0 and
      // never through 180.
      float s = 340.0f;
      for (int i = 0; i < 40; ++i) {
        s = compassGlide(s, 20.0f, 0.25f);
        const bool northHalf = (s >= 300.0f) || (s <= 60.0f);
        CHECK(northHalf);
      }
      CHECK_NEAR(s, 20.0f, 1e-3f);
    }

    // SNAPS ABOVE COMPASS_SNAP_DEG. 96 deg in one sample is the capture's
    // hairpin -- 192 deg/s at 15-19 km/h, real -- and easing it at 0.1 would
    // still be 30 deg out a second later, showing a corner the driver has
    // finished. One step, whole.
    CHECK_NEAR(compassGlide(0.0f, 96.0f, 0.1f), 96.0f, 1e-4f);
    CHECK_NEAR(compassGlide(0.0f, 264.0f, 0.1f), 264.0f, 1e-4f);   // -96 the short way
    CHECK_NEAR(compassGlide(350.0f, 86.0f, 0.1f), 86.0f, 1e-4f);   // 96 across the seam
    CHECK_NEAR(compassGlide(0.0f, COMPASS_SNAP_DEG + 1.0f, 0.1f),
               COMPASS_SNAP_DEG + 1.0f, 1e-4f);
    // Just below the snap it really does ease rather than jump.
    CHECK_NEAR(compassGlide(0.0f, COMPASS_SNAP_DEG - 1.0f, 0.5f),
               (COMPASS_SNAP_DEG - 1.0f) * 0.5f, 1e-3f);

    // SETTLES UNDER COMPASS_SETTLE_DEG -- takes the target whole instead of
    // closing a fraction of a fraction. A quarter of a degree is an eighth of
    // one raw count, so this can never cross a sector boundary the signal was
    // able to resolve.
    CHECK_NEAR(compassGlide(0.0f, 0.2f, 0.5f), 0.2f, 1e-4f);
    CHECK_NEAR(compassGlide(0.0f, COMPASS_SETTLE_DEG, 0.5f), COMPASS_SETTLE_DEG, 1e-4f);
    CHECK_NEAR(compassGlide(359.9f, 0.05f, 0.5f), 0.05f, 1e-4f);   // across the seam
    CHECK_NEAR(compassGlide(90.0f, 90.0f, 0.5f), 90.0f, 1e-4f);

    // alpha at and outside its stated range. A caller passing 1.8 has a bug,
    // but this feeds the glass every tick, so it clamps rather than launching
    // the tape past the target and back.
    CHECK_NEAR(compassGlide(0.0f, 40.0f, 0.0f), 0.0f, 1e-4f);
    CHECK_NEAR(compassGlide(0.0f, 40.0f, -1.0f), 0.0f, 1e-4f);
    CHECK_NEAR(compassGlide(0.0f, 40.0f, 1.0f), 40.0f, 1e-4f);
    CHECK_NEAR(compassGlide(0.0f, 40.0f, 2.0f), 40.0f, 1e-4f);

    // NEVER OUTSIDE 0..360, swept. This is the promise that cannot be spot
    // checked: the return is fed back in as `shown` on the next tick, so one
    // escape does not heal, it compounds -- and a heading of -0.3 fed to a
    // renderer that indexes a table by octant is a bad day.
    static const float kAlphas[] = { -1.0f, 0.0f, 0.05f, 0.25f, 0.5f,
                                     0.999f, 1.0f, 2.0f };
    for (int s = -400; s <= 760; s += 13) {
      for (int t = -400; t <= 760; t += 17) {
        for (unsigned ai = 0u; ai < sizeof(kAlphas) / sizeof(kAlphas[0]); ++ai) {
          const float r = compassGlide((float)s * 0.5f, (float)t * 0.5f, kAlphas[ai]);
          CHECK(r >= 0.0f);
          CHECK(r < 360.0f);
        }
      }
    }
  }

  // --- compassMarks: a mid-range heading, at the real geometry -------------
  {
    CompassMark m[COMPASS_MAX_MARKS];
    const uint8_t n = compassMarks(m, COMPASS_MAX_MARKS, 90.0f, TAPE_W, TAPE_SPAN);

    // 90 deg of span at 15 deg ticks is six intervals, so seven marks when the
    // heading sits on a tick -- both ends land exactly on the tape edges.
    CHECK_EQ(n, 7);
    checkMarksInvariants(m, n, COMPASS_MAX_MARKS, TAPE_W);

    const uint16_t wantDeg[7] = { 45u, 60u, 75u, 90u, 105u, 120u, 135u };
    const int16_t  wantX[7]   = { 0, 26, 52, 78, 104, 130, 156 };
    for (uint8_t i = 0u; i < 7u; ++i) {
      CHECK_EQ(m[i].deg, wantDeg[i]);
      CHECK_EQ(m[i].x, wantX[i]);
    }
    // The heading itself is dead centre, under the fixed marker.
    CHECK_EQ(m[3].deg, 90);
    CHECK_EQ(m[3].x, (int16_t)(TAPE_W / 2u));

    // THE TWO SPACINGS THE LAYOUT WAS DRAWN AGAINST. 156/90 is 1.7333 px/deg,
    // so an octant is 78 px and a minor tick 26 -- asserted here so that a
    // change to the tape's width or span shows up as a failing number rather
    // than as marks quietly crowding each other on the panel.
    CHECK_EQ(m[3].x - m[0].x, 78);      // 90 -> 45, one octant
    CHECK_EQ(m[6].x - m[3].x, 78);      // 90 -> 135, the other way
    for (uint8_t i = 1u; i < 7u; ++i) {
      CHECK_EQ(m[i].x - m[i - 1].x, 26);
    }

    // major is the octant test and nothing else: 45, 90 and 135 only.
    CHECK(m[0].major);
    CHECK(!m[1].major);
    CHECK(!m[2].major);
    CHECK(m[3].major);
    CHECK(!m[4].major);
    CHECK(!m[5].major);
    CHECK(m[6].major);
    CHECK_STR(compassOctantName(compassOctantFor((float)m[3].deg)), "E");
  }

  // --- compassMarks: ACROSS THE SEAM --------------------------------------
  {
    // The case the whole walk is shaped around. At a heading of 5 the window
    // is -40..+50, which in labels is 330 / 345 / 0 / 15 / 30 / 45 -- NOT a
    // contiguous run of a 0..345 table, which is what a loop that walks the
    // table and wraps its comparison would assume.
    CompassMark m[COMPASS_MAX_MARKS];
    const uint8_t n = compassMarks(m, COMPASS_MAX_MARKS, 5.0f, TAPE_W, TAPE_SPAN);

    CHECK_EQ(n, 6);
    checkMarksInvariants(m, n, COMPASS_MAX_MARKS, TAPE_W);

    const uint16_t wantDeg[6] = { 330u, 345u, 0u, 15u, 30u, 45u };
    for (uint8_t i = 0u; i < 6u; ++i) {
      CHECK_EQ(m[i].deg, wantDeg[i]);
    }
    // Left to right, in that order, with the seam in the middle of the tape
    // and no jump in the pixel spacing where it crosses.
    for (uint8_t i = 1u; i < 6u; ++i) {
      CHECK_EQ(m[i].x - m[i - 1].x, 26);
    }

    // Two majors in this window -- 0 and 45 -- and the interesting one is 0,
    // which the driver reads as "N". The window is 90 deg wide and octants are
    // 45 apart, so a tape this size always shows two or three of them.
    CHECK(m[2].major);
    CHECK_EQ(m[2].deg, 0);
    CHECK_STR(compassOctantName(compassOctantFor((float)m[2].deg)), "N");
    CHECK(m[5].major);
    CHECK_EQ(m[5].deg, 45);
    CHECK_STR(compassOctantName(compassOctantFor((float)m[5].deg)), "NE");
    CHECK(!m[0].major);
    CHECK(!m[1].major);
    CHECK(!m[3].major);
    CHECK(!m[4].major);

    // AND IT SITS JUST LEFT OF CENTRE, 9 px of the 1.7333 px/deg scale. That
    // is the contract's own arithmetic and it is worth spelling out because it
    // is easy to picture the other way round: pointing at 5 deg, north is
    // 5 deg ANTI-clockwise of where the truck is aimed, so
    // compassArc(5, 0) = -5 and the mark lands at 78 - 8.67 = 69.3 -> 69.
    // A tape that put it right of centre would be scrolling the wrong way, and
    // every heading except a cardinal one would look almost right.
    CHECK_EQ(m[2].x, 69);
    CHECK(m[2].x < (int16_t)(TAPE_W / 2u));
    CHECK_EQ(m[0].x, 17);
    CHECK_EQ(m[5].x, 147);

    // The mirror image, one tick the other side of the seam: heading 355 shows
    // 315..45 with 0 just RIGHT of centre.
    CompassMark m2[COMPASS_MAX_MARKS];
    const uint8_t n2 = compassMarks(m2, COMPASS_MAX_MARKS, 355.0f, TAPE_W, TAPE_SPAN);
    CHECK_EQ(n2, 6);
    checkMarksInvariants(m2, n2, COMPASS_MAX_MARKS, TAPE_W);
    const uint16_t wantDeg2[6] = { 315u, 330u, 345u, 0u, 15u, 30u };
    for (uint8_t i = 0u; i < 6u; ++i) {
      CHECK_EQ(m2[i].deg, wantDeg2[i]);
    }
    CHECK_EQ(m2[3].deg, 0);
    CHECK(m2[3].x > (int16_t)(TAPE_W / 2u));

    // Exactly on the seam: north dead centre, 315..45 either side of it.
    CompassMark m3[COMPASS_MAX_MARKS];
    const uint8_t n3 = compassMarks(m3, COMPASS_MAX_MARKS, 0.0f, TAPE_W, TAPE_SPAN);
    CHECK_EQ(n3, 7);
    checkMarksInvariants(m3, n3, COMPASS_MAX_MARKS, TAPE_W);
    const uint16_t wantDeg3[7] = { 315u, 330u, 345u, 0u, 15u, 30u, 45u };
    for (uint8_t i = 0u; i < 7u; ++i) {
      CHECK_EQ(m3[i].deg, wantDeg3[i]);
    }
    CHECK_EQ(m3[3].x, (int16_t)(TAPE_W / 2u));
    CHECK(m3[0].major);    // 315 = SW... NW
    CHECK(m3[3].major);    // 0
    CHECK(m3[6].major);    // 45
    CHECK_STR(compassOctantName(compassOctantFor((float)m3[0].deg)), "NW");
  }

  // --- compassMarks: `max` is honoured, and the degenerate cases -----------
  {
    CompassMark m[COMPASS_MAX_MARKS];

    // A full seven marks' worth of window, asked for three. The three returned
    // are the leftmost three -- the walk is in ascending x and simply stops --
    // and nothing beyond them is touched.
    for (uint8_t i = 0u; i < COMPASS_MAX_MARKS; ++i) {
      m[i].x = -1;
      m[i].deg = 9999u;
      m[i].major = true;
    }
    const uint8_t n = compassMarks(m, 3u, 90.0f, TAPE_W, TAPE_SPAN);
    CHECK_EQ(n, 3);
    checkMarksInvariants(m, n, 3u, TAPE_W);
    CHECK_EQ(m[0].deg, 45);
    CHECK_EQ(m[1].deg, 60);
    CHECK_EQ(m[2].deg, 75);
    CHECK_EQ(m[3].x, -1);            // untouched
    CHECK_EQ(m[3].deg, 9999);

    // A span wide enough to want more marks than COMPASS_MAX_MARKS. It
    // truncates at `max` rather than overrunning the caller's array -- which
    // is exactly why the bound is a parameter and not only a constant.
    const uint8_t nFull = compassMarks(m, COMPASS_MAX_MARKS, 180.0f, 240u, 360u);
    CHECK_EQ(nFull, COMPASS_MAX_MARKS);
    checkMarksInvariants(m, nFull, COMPASS_MAX_MARKS, 240u);

    // Degenerate geometry writes nothing and says so.
    CHECK_EQ(compassMarks(m, COMPASS_MAX_MARKS, 90.0f, 0u, TAPE_SPAN), 0);
    CHECK_EQ(compassMarks(m, COMPASS_MAX_MARKS, 90.0f, TAPE_W, 0u), 0);
    CHECK_EQ(compassMarks(m, 0u, 90.0f, TAPE_W, TAPE_SPAN), 0);
    CHECK_EQ(compassMarks(nullptr, COMPASS_MAX_MARKS, 90.0f, TAPE_W, TAPE_SPAN), 0);
  }

  // --- compassMarks: the whole circle, a tenth of a degree at a time -------
  {
    // Every invariant, at every heading the signal can produce and a great
    // many it cannot. What this is really guarding is the seam: the count must
    // not dip as the window crosses north, which is the shape a wrap bug takes
    // when it drops the marks on one side of it.
    CompassMark m[COMPASS_MAX_MARKS];
    for (int i = -3600; i < 7200; ++i) {
      const float deg = (float)i * 0.1f;
      const uint8_t n = compassMarks(m, COMPASS_MAX_MARKS, deg, TAPE_W, TAPE_SPAN);
      CHECK(n >= 6u);
      CHECK(n <= 7u);
      checkMarksInvariants(m, n, COMPASS_MAX_MARKS, TAPE_W);
      // The mark nearest the centre marker must be within half a tick of the
      // heading being shown -- the tape cannot be scrolled off by a whole tick
      // and still be internally consistent.
      float best = 999.0f;
      for (uint8_t j = 0u; j < n; ++j) {
        const float a = compassArc(deg, (float)m[j].deg);
        const float mag = a < 0.0f ? -a : a;
        if (mag < best) {
          best = mag;
        }
      }
      CHECK(best <= (float)COMPASS_TICK_DEG * 0.5f + 1e-3f);
    }
  }

  // --- the raw signal's own range, end to end ------------------------------
  {
    // Every value byte 1 is allowed to take, pushed through the whole chain.
    // 0xFF is deliberately absent: it never appears in 2953 frames and there
    // is no unavailable sentinel on this frame, so there is no such case to
    // test -- a heading is held and dimmed or it was never heard at all.
    CompassMark m[COMPASS_MAX_MARKS];
    for (unsigned raw = 0u; raw <= COMPASS_RAW_MAX; ++raw) {
      const float deg = (float)raw * COMPASS_DEG_PER_COUNT;
      CHECK(deg >= 0.0f);
      CHECK(deg <= 358.0f);
      CHECK_NEAR(compassNorm(deg), deg, 1e-4f);
      const uint8_t oct = compassOctantFor(deg);
      CHECK(oct < 8u);
      CHECK(compassOctantName(oct)[0] != '\0');
      const uint8_t n = compassMarks(m, COMPASS_MAX_MARKS, deg, TAPE_W, TAPE_SPAN);
      checkMarksInvariants(m, n, COMPASS_MAX_MARKS, TAPE_W);
      // And the glide is a no-op on a heading that has not moved, which is the
      // stationary case: 196 consecutive stopped sample pairs in the capture
      // differ by exactly 0.0 deg, so this is what the tape does most of the
      // time it is being looked at.
      CHECK_NEAR(compassGlide(deg, deg, 0.25f), deg, 1e-4f);
    }
  }

  // --- compassDial: THE FOUR QUADRANT ANCHORS, by hand ---------------------
  {
    // heading 0, centre (120,100), rx 100, ry 50. Every number below is the
    // projection worked through with a pencil:
    //
    //   deg 0    a = 0     u = 0   v = +1  f = 1/(1 + 0.30) = 0.76923
    //            x = 120 + 100*0*f       = 120
    //            y = 100 -  50*1*0.76923 = 100 - 38.46 = 61.54 -> 62
    //   deg 180  a = 180   u = 0   v = -1  f = 1/(1 - 0.30) = 1.42857
    //            x = 120
    //            y = 100 +  50*1.42857   = 100 + 71.43 = 171.43 -> 171
    //   deg 90   a = 90    u = +1  v = 0   f = 1/(1 + 0) = 1
    //            x = 120 + 100 = 220,  y = 100
    //   deg 270  a = -90   u = -1  v = 0   f = 1
    //            x = 120 - 100 = 20,   y = 100
    //
    // The two side marks sit EXACTLY on the centre line and exactly rx away,
    // because v = 0 is where the divide does nothing at all. They are the
    // control: an implementation with the divide dropped agrees with those two
    // and with nothing else on the ring.
    CompassDialMark d[COMPASS_DIAL_MARKS];
    const uint8_t n = compassDial(d, COMPASS_DIAL_MARKS, 0.0f,
                                  DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                  (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(n, COMPASS_DIAL_MARKS);
    checkDial(d, n, COMPASS_DIAL_MARKS, DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
              (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);

    // Far first, so index 0 IS the heading's own bearing and index 23 is its
    // antipode. The two sides fall wherever the sort puts them, so they are
    // found by bearing rather than assumed into a slot.
    CHECK_EQ(d[0].deg, 0);
    CHECK_EQ(d[0].x, 120);
    CHECK_EQ(d[0].y, 62);
    CHECK_EQ(d[COMPASS_DIAL_MARKS - 1].deg, 180);
    CHECK_EQ(d[COMPASS_DIAL_MARKS - 1].x, 120);
    CHECK_EQ(d[COMPASS_DIAL_MARKS - 1].y, 171);

    int i90 = -1;
    int i270 = -1;
    for (uint8_t i = 0u; i < n; ++i) {
      if (d[i].deg == 90u) {
        i90 = (int)i;
      }
      if (d[i].deg == 270u) {
        i270 = (int)i;
      }
    }
    CHECK(i90 >= 0);
    CHECK(i270 >= 0);
    CHECK_EQ(d[i90].x, 220);
    CHECK_EQ(d[i90].y, 100);
    CHECK_EQ(d[i270].x, 20);
    CHECK_EQ(d[i270].y, 100);

    // The ring is LEFT-RIGHT SYMMETRIC about the centre column and each mirror
    // pair shares a row. That is true of an orthographic ring too -- it is here
    // to pin the property the asymmetry test below deliberately does not cover.
    for (uint8_t i = 0u; i < n; ++i) {
      const uint16_t mirror = (uint16_t)((360u - d[i].deg) % 360u);
      for (uint8_t j = 0u; j < n; ++j) {
        if (d[j].deg == mirror) {
          CHECK_EQ(d[i].x - DIAL_CX, DIAL_CX - d[j].x);
          CHECK_EQ(d[i].y, d[j].y);
        }
      }
    }
  }

  // --- compassDial: THE ASYMMETRY IS THE REGRESSION TEST -------------------
  {
    // THIS IS THE CHECK THAT CATCHES THE DIVIDE BEING DROPPED, and it is the
    // only one here that can.
    //
    // With f the farthest mark sits ry/(1 + e) = 50/1.3 = 38.46 -> 38 px ABOVE
    // the centre and the nearest ry/(1 - e) = 50/0.7 = 71.43 -> 71 px BELOW it.
    // An ORTHOGRAPHIC implementation -- x = cx + rx*u, y = cy - ry*v, no divide
    // -- puts BOTH at exactly ry = 50, because it compresses the far half and
    // the near half by the same factor. Everything else about that ring is
    // still right: closed, ordered, symmetric, correctly coloured, correctly
    // sized, inside a sane box. It simply reads as an oval seen edge-on instead
    // of a disc lying on the ground, and nothing else in this file notices.
    //
    // So: 38 up, 71 down, 33 px of difference between them.
    CompassDialMark d[COMPASS_DIAL_MARKS];
    const uint8_t n = compassDial(d, COMPASS_DIAL_MARKS, 0.0f,
                                  DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                  (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(n, COMPASS_DIAL_MARKS);

    const int up = DIAL_CY - d[0].y;                         // farthest
    const int down = d[COMPASS_DIAL_MARKS - 1].y - DIAL_CY;  // nearest
    CHECK_EQ(up, 38);
    CHECK_EQ(down, 71);
    CHECK(down > up);
    CHECK_EQ(down - up, 33);
    // Stated as intent as well as arithmetic, so it survives a change of ry:
    // NEITHER half may be ry, and the near half must be the bigger one by a
    // wide margin rather than by a pixel of rounding.
    CHECK(up < DIAL_RY);
    CHECK(down > DIAL_RY);
    CHECK((float)down > (float)up * 1.5f);
    // And the exact ratio the projection promises: (1+e)/(1-e) = 1.857.
    CHECK_NEAR((float)down / (float)up,
               (1.0f + COMPASS_DIAL_E) / (1.0f - COMPASS_DIAL_E), 0.03f);

    // The same measurement on a DIFFERENT radius, so this is a test of the
    // projection and not of two memorised integers. ry = 100 gives
    // 100/1.3 = 76.9 -> 77 up and 100/0.7 = 142.9 -> 143 down.
    CompassDialMark d2[COMPASS_DIAL_MARKS];
    const uint8_t n2 = compassDial(d2, COMPASS_DIAL_MARKS, 0.0f,
                                   DIAL_CX, 200, DIAL_RX, 100,
                                   (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(n2, COMPASS_DIAL_MARKS);
    CHECK_EQ(200 - d2[0].y, 77);
    CHECK_EQ(d2[COMPASS_DIAL_MARKS - 1].y - 200, 143);

    // THE NEAR HALF IS ALSO WIDER, which is the same divide seen on the other
    // axis. |x - cx| peaks at cos a = -e rather than at the sides, reaching
    // rx/sqrt(1 - e^2) = 104.8 -> 105 at rx = 100. An orthographic ring's
    // widest point IS the side mark, at exactly rx. The true peak is at
    // 107.5 deg, so the 105 deg tick is the one that carries it.
    int widest = 0;
    for (uint8_t i = 0u; i < n; ++i) {
      const int dx = d[i].x > DIAL_CX ? d[i].x - DIAL_CX : DIAL_CX - d[i].x;
      if (dx > widest) {
        widest = dx;
      }
    }
    CHECK_EQ(widest, 105);
    CHECK(widest > DIAL_RX);
  }

  // --- compassDial: depth --------------------------------------------------
  {
    CompassDialMark d[COMPASS_DIAL_MARKS];
    const uint8_t n = compassDial(d, COMPASS_DIAL_MARKS, 0.0f,
                                  DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                  (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(n, COMPASS_DIAL_MARKS);

    // Both ends land exactly, because v is exactly +1 and exactly -1 there.
    CHECK_EQ(d[0].depth, 255);                          // farthest
    CHECK_EQ(d[COMPASS_DIAL_MARKS - 1].depth, 0);       // nearest

    // The two side marks are half way. NOT exactly 128: cosf(pi/2) is -4.4e-8
    // rather than zero, so (v+1)/2*255 comes to 127.4999 and rounds to 127. A
    // tolerance rather than a magic number, because which side of the half the
    // library's cosine falls on is not something this module gets to decide.
    for (uint8_t i = 0u; i < n; ++i) {
      if (d[i].deg == 90u || d[i].deg == 270u) {
        CHECK(d[i].depth >= 126u);
        CHECK(d[i].depth <= 129u);
      }
    }

    // MONOTONIC from near to far. checkDial() asserts non-increasing along the
    // emitted order; this walks it the other way and says the same thing from
    // the driver's end -- a mark further round the ring from the heading is
    // never darker than one closer to it.
    for (uint8_t i = (uint8_t)(n - 1u); i > 0u; --i) {
      CHECK(d[i - 1u].depth >= d[i].depth);
    }
    // And it spans the range rather than bunching at the ends.
    CHECK(d[1].depth < 255u);
    CHECK(d[1].depth > 200u);
    CHECK(d[COMPASS_DIAL_MARKS - 2].depth > 0u);
    CHECK(d[COMPASS_DIAL_MARKS - 2].depth < 50u);

    checkDial(d, n, COMPASS_DIAL_MARKS, DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
              (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
  }

  // --- compassDial: size, and the CLAMP ------------------------------------
  {
    CompassDialMark d[COMPASS_DIAL_MARKS];
    const uint8_t n = compassDial(d, COMPASS_DIAL_MARKS, 0.0f,
                                  DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                  (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(n, COMPASS_DIAL_MARKS);

    // dotMax at the nearest, dotMin at the farthest. The dot naming the
    // driver's own heading is the SMALLEST on the dial, which reads as wrong
    // until you remember the heading is at the top and the top is the far side.
    CHECK_EQ(d[0].size, DIAL_DOT_MIN);                        // depth 255
    CHECK_EQ(d[COMPASS_DIAL_MARKS - 1].size, DIAL_DOT_MAX);   // depth 0

    // Monotonically decreasing WITH DEPTH -- which, since the emission is far
    // first, means non-decreasing along the array.
    for (uint8_t i = 1u; i < n; ++i) {
      CHECK(d[i].size >= d[i - 1].size);
      if (d[i].depth == d[i - 1].depth) {
        CHECK_EQ(d[i].size, d[i - 1].size);   // one depth, one size
      }
    }
    for (uint8_t i = 0u; i < n; ++i) {
      CHECK(d[i].size >= DIAL_DOT_MIN);
      CHECK(d[i].size <= DIAL_DOT_MAX);
    }

    // THE PAIR PASSED THE WRONG WAY ROUND. There is no arrangement of two
    // uint8_t this function is able to reject, so the only defence available is
    // that the result stays inside the caller's own two numbers: the ring comes
    // out growing with distance -- wrong, and obviously wrong the moment anyone
    // looks at it -- rather than zero-diameter (invisible) or runaway (a smear
    // across the panel), which are the two failures nobody can diagnose from
    // the driver's seat.
    CompassDialMark r[COMPASS_DIAL_MARKS];
    const uint8_t nr = compassDial(r, COMPASS_DIAL_MARKS, 0.0f,
                                   DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                   (uint8_t)DIAL_DOT_MAX, (uint8_t)DIAL_DOT_MIN);
    CHECK_EQ(nr, COMPASS_DIAL_MARKS);
    checkDial(r, nr, COMPASS_DIAL_MARKS, DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
              (uint8_t)DIAL_DOT_MAX, (uint8_t)DIAL_DOT_MIN);
    for (uint8_t i = 0u; i < nr; ++i) {
      CHECK(r[i].size >= DIAL_DOT_MIN);
      CHECK(r[i].size <= DIAL_DOT_MAX);
      CHECK(r[i].size > 0u);
    }
    CHECK_EQ(r[0].size, DIAL_DOT_MAX);
    CHECK_EQ(r[COMPASS_DIAL_MARKS - 1].size, DIAL_DOT_MIN);
    // The GEOMETRY is untouched by the swap -- only the radii changed.
    for (uint8_t i = 0u; i < nr; ++i) {
      CHECK_EQ(r[i].deg, d[i].deg);
      CHECK_EQ(r[i].x, d[i].x);
      CHECK_EQ(r[i].y, d[i].y);
      CHECK_EQ(r[i].depth, d[i].depth);
    }

    // Equal bounds: one size everywhere, and nothing divides by the zero span.
    CompassDialMark f[COMPASS_DIAL_MARKS];
    const uint8_t nf = compassDial(f, COMPASS_DIAL_MARKS, 0.0f,
                                   DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY, 5u, 5u);
    CHECK_EQ(nf, COMPASS_DIAL_MARKS);
    for (uint8_t i = 0u; i < nf; ++i) {
      CHECK_EQ(f[i].size, 5);
    }
  }

  // --- compassDial: major, and the octant NAMES ----------------------------
  {
    CompassDialMark d[COMPASS_DIAL_MARKS];
    const uint8_t n = compassDial(d, COMPASS_DIAL_MARKS, 0.0f,
                                  DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                  (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(n, COMPASS_DIAL_MARKS);

    // Exactly eight majors out of twenty-four, on the multiples of 45 and
    // nowhere else -- the renderer draws a NAME at those and a dot at the rest,
    // so one extra would put "NE" half way to north.
    unsigned majors = 0u;
    for (uint8_t i = 0u; i < n; ++i) {
      CHECK_EQ(d[i].major ? 1 : 0, (d[i].deg % 45u) == 0u ? 1 : 0);
      if (d[i].major) {
        ++majors;
        // deg / 45 IS the octant index for a mark that lies on one, so the
        // renderer needs no binning here. compassOctantFor() is for arbitrary
        // headings; a dial mark is never arbitrary, and the two agree.
        CHECK_EQ((unsigned)(d[i].deg / 45u),
                 (unsigned)compassOctantFor((float)d[i].deg));
        CHECK(compassOctantName((uint8_t)(d[i].deg / 45u))[0] != '\0');
      }
    }
    CHECK_EQ(majors, 8);

    // The letters themselves, each looked up the way the renderer will.
    static const uint16_t kMajorDeg[8] = { 0u, 45u, 90u, 135u,
                                           180u, 225u, 270u, 315u };
    static const char* const kMajorName[8] = { "N", "NE", "E", "SE",
                                               "S", "SW", "W", "NW" };
    for (unsigned k = 0u; k < 8u; ++k) {
      bool found = false;
      for (uint8_t i = 0u; i < n; ++i) {
        if (d[i].deg == kMajorDeg[k]) {
          found = true;
          CHECK(d[i].major);
          CHECK_STR(compassOctantName((uint8_t)(d[i].deg / 45u)), kMajorName[k]);
        }
      }
      CHECK(found);
    }
  }

  // --- compassDial: ROTATION, and the rigid ring ---------------------------
  {
    // The dial is heading-up, so advancing the heading must move the FARTHEST
    // mark to the new heading's own bearing -- the ring turning under a fixed
    // marker, exactly as the tape scrolls under one.
    static const float kHead[8] = { 0.0f, 15.0f, 45.0f, 90.0f,
                                    180.0f, 270.0f, 345.0f, 360.0f };
    static const uint16_t kFar[8] = { 0u, 15u, 45u, 90u,
                                      180u, 270u, 345u, 0u };
    CompassDialMark d[COMPASS_DIAL_MARKS];
    for (unsigned k = 0u; k < 8u; ++k) {
      const uint8_t n = compassDial(d, COMPASS_DIAL_MARKS, kHead[k],
                                    DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                    (uint8_t)DIAL_DOT_MIN,
                                    (uint8_t)DIAL_DOT_MAX);
      CHECK_EQ(n, COMPASS_DIAL_MARKS);
      checkDial(d, n, COMPASS_DIAL_MARKS, DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
      CHECK_EQ(d[0].deg, kFar[k]);
      CHECK_EQ(d[0].y, 62);                                 // always farthest
      CHECK_EQ(d[0].x, DIAL_CX);
      CHECK_EQ(d[COMPASS_DIAL_MARKS - 1].deg,
               (uint16_t)(((unsigned)kFar[k] + 180u) % 360u));
    }

    // THE RING TURNS RIGIDLY. A mark at bearing b seen from heading h lands on
    // the same pixel as the mark at bearing b - h seen from heading 0, because
    // the projection depends on the ARC between the two and on nothing else.
    // Asserted as exact integer equality, which is available because
    // compassArc() returns arcs that are exactly equal rather than merely
    // close: 90 -> 0 gives -90.0f and 0 -> 270 gives -90.0f, the same float.
    //
    // (It is the mark at b MINUS h, not plus. Pointing at 90, east is dead
    // ahead and north is off to the left -- the same "it reads as backwards and
    // is not" that the tape's own seam test spells out.)
    CompassDialMark base[COMPASS_DIAL_MARKS];
    const uint8_t nb = compassDial(base, COMPASS_DIAL_MARKS, 0.0f,
                                   DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                   (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(nb, COMPASS_DIAL_MARKS);
    for (unsigned h = 0u; h < 360u; h += COMPASS_DIAL_TICK_DEG) {
      const uint8_t n = compassDial(d, COMPASS_DIAL_MARKS, (float)h,
                                    DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                    (uint8_t)DIAL_DOT_MIN,
                                    (uint8_t)DIAL_DOT_MAX);
      CHECK_EQ(n, COMPASS_DIAL_MARKS);
      for (uint8_t i = 0u; i < n; ++i) {
        const uint16_t want = (uint16_t)((d[i].deg + 360u - h) % 360u);
        for (uint8_t j = 0u; j < nb; ++j) {
          if (base[j].deg == want) {
            CHECK_EQ(d[i].x, base[j].x);
            CHECK_EQ(d[i].y, base[j].y);
            CHECK_EQ(d[i].depth, base[j].depth);
            CHECK_EQ(d[i].size, base[j].size);
          }
        }
      }
      // ...and the ORDER is rigid too: the k-th mark out is always the same
      // distance round the ring from whatever the heading is.
      for (uint8_t i = 0u; i < n; ++i) {
        CHECK_EQ((unsigned)((d[i].deg + 360u - h) % 360u),
                 (unsigned)base[i].deg);
      }
    }

    // AND IT IS CORRECT ACROSS THE WRAP AT A HEADING THAT IS NOT A TICK. At
    // 350 deg the farthest tick is 345 (5 deg back) and 0 is the next one round
    // -- a mark on the far side of the SEAM, which a naive bearing - heading
    // would place 350 deg away instead of 10 and fling to the bottom of the
    // dial.
    const uint8_t n350 = compassDial(d, COMPASS_DIAL_MARKS, 350.0f,
                                     DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                     (uint8_t)DIAL_DOT_MIN,
                                     (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(n350, COMPASS_DIAL_MARKS);
    checkDial(d, n350, COMPASS_DIAL_MARKS, DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
              (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(d[0].deg, 345);
    CHECK_EQ(d[1].deg, 0);          // 10 deg clockwise, still near the top
    CHECK_EQ(d[2].deg, 330);        // 20 deg anticlockwise
    CHECK(d[1].y < DIAL_CY - 30);   // still up in the far half, not across it
    CHECK(d[1].x > DIAL_CX);        // and to the RIGHT: 0 is clockwise of 350

    // Unnormalised headings are normalised first, like everything else here.
    CompassDialMark w[COMPASS_DIAL_MARKS];
    CHECK_EQ(compassDial(w, COMPASS_DIAL_MARKS, -10.0f,
                         DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                         (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX),
             COMPASS_DIAL_MARKS);
    for (uint8_t i = 0u; i < COMPASS_DIAL_MARKS; ++i) {
      CHECK_EQ(w[i].deg, d[i].deg);
      CHECK_EQ(w[i].x, d[i].x);
      CHECK_EQ(w[i].y, d[i].y);
    }
    CHECK_EQ(compassDial(w, COMPASS_DIAL_MARKS, 710.0f,
                         DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                         (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX),
             COMPASS_DIAL_MARKS);
    for (uint8_t i = 0u; i < COMPASS_DIAL_MARKS; ++i) {
      CHECK_EQ(w[i].x, d[i].x);
      CHECK_EQ(w[i].y, d[i].y);
    }
  }

  // --- compassDial: `max` is honoured, and the degenerate cases ------------
  {
    // Same idiom as the compassMarks test above: sentinel the whole array, ask
    // for fewer than the ring holds, and check the tail is untouched.
    CompassDialMark d[COMPASS_DIAL_MARKS];
    for (uint8_t i = 0u; i < COMPASS_DIAL_MARKS; ++i) {
      d[i].x = -1;
      d[i].y = -1;
      d[i].size = 99u;
      d[i].depth = 77u;
      d[i].deg = 9999u;
      d[i].major = true;
    }
    const uint8_t n = compassDial(d, 5u, 0.0f,
                                  DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                  (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(n, 5);
    checkDial(d, n, 5u, DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
              (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    // The five kept are the five FARTHEST, which is what far-first ordering
    // means for a truncated call: a caller short of widgets loses the near
    // marks, and those are the ones it would have drawn last anyway. Within a
    // tied pair the CLOCKWISE half comes first -- 15 before 345, 30 before 330
    // -- which is the tie-break that makes the order rigid as the ring turns,
    // and is asserted as such in the rotation block above.
    const uint16_t wantDeg[5] = { 0u, 15u, 345u, 30u, 330u };
    for (uint8_t i = 0u; i < 5u; ++i) {
      CHECK_EQ(d[i].deg, wantDeg[i]);
    }
    CHECK_EQ(d[5].x, -1);            // untouched, all six fields
    CHECK_EQ(d[5].y, -1);
    CHECK_EQ(d[5].deg, 9999);
    CHECK_EQ(d[5].size, 99);
    CHECK_EQ(d[5].depth, 77);
    CHECK_EQ(d[5].major ? 1 : 0, 1);
    CHECK_EQ(d[23].deg, 9999);

    // One mark: the heading's own, and nothing else written.
    const uint8_t n1 = compassDial(d, 1u, 90.0f,
                                   DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                   (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(n1, 1);
    CHECK_EQ(d[0].deg, 90);
    CHECK_EQ(d[0].depth, 255);
    CHECK_EQ(d[1].deg, 15);          // still what the five-mark call left there

    // A `max` larger than the ring returns the RING, not more: the count is
    // fixed at 360/15 and nothing is culled, so a caller with room for 200 gets
    // exactly 24.
    const uint8_t nBig = compassDial(d, 200u, 0.0f,
                                     DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                     (uint8_t)DIAL_DOT_MIN,
                                     (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(nBig, COMPASS_DIAL_MARKS);

    // Degenerate callers write nothing and say so.
    CHECK_EQ(compassDial(d, 0u, 0.0f, DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                         (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX), 0);
    CHECK_EQ(compassDial(nullptr, COMPASS_DIAL_MARKS, 0.0f,
                         DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                         (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX), 0);
    // A zero radius is a degenerate DIAL and not an error: every mark collapses
    // onto the centre, the ring is still emitted whole, and the renderer draws
    // twenty-four dots on top of each other rather than reading a count it was
    // never given.
    const uint8_t nz = compassDial(d, COMPASS_DIAL_MARKS, 0.0f,
                                   DIAL_CX, DIAL_CY, 0, 0,
                                   (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
    CHECK_EQ(nz, COMPASS_DIAL_MARKS);
    for (uint8_t i = 0u; i < nz; ++i) {
      CHECK_EQ(d[i].x, DIAL_CX);
      CHECK_EQ(d[i].y, DIAL_CY);
    }
  }

  // --- compassDial: the whole circle, half a degree at a time --------------
  {
    // Every invariant at every heading, three turns out in each direction, so
    // the seam is crossed from both sides and from several turns away.
    CompassDialMark d[COMPASS_DIAL_MARKS];
    for (int i = -720; i < 2160; ++i) {
      const float heading = (float)i * 0.5f;
      const uint8_t n = compassDial(d, COMPASS_DIAL_MARKS, heading,
                                    DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                    (uint8_t)DIAL_DOT_MIN,
                                    (uint8_t)DIAL_DOT_MAX);
      // ALWAYS the whole ring. A count that dips is a visibility test that
      // should not exist having crept in.
      CHECK_EQ(n, COMPASS_DIAL_MARKS);
      checkDial(d, n, COMPASS_DIAL_MARKS, DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
      // The first mark out is the one nearest the heading, within half a tick.
      // The dial cannot be rotated off by a whole tick and still be internally
      // consistent -- the same check the tape gets.
      const float a = compassArc(heading, (float)d[0].deg);
      const float mag = a < 0.0f ? -a : a;
      CHECK(mag <= (float)COMPASS_DIAL_TICK_DEG * 0.5f + 1e-3f);
      // ...and it is genuinely in the far half, above the centre, always --
      // with its antipode genuinely below it.
      CHECK(d[0].y < DIAL_CY);
      CHECK(d[COMPASS_DIAL_MARKS - 1].y > DIAL_CY);
      CHECK(d[0].depth > d[COMPASS_DIAL_MARKS - 1].depth);
    }
  }

  // --- compassDial: the raw signal's own range -----------------------------
  {
    // Every heading byte 1 can carry, projected. 2 deg/count means the dial
    // sees 180 distinct headings, and none of them may produce a ring that is
    // short, unordered or outside its box.
    CompassDialMark d[COMPASS_DIAL_MARKS];
    for (unsigned raw = 0u; raw <= COMPASS_RAW_MAX; ++raw) {
      const float deg = (float)raw * COMPASS_DEG_PER_COUNT;
      const uint8_t n = compassDial(d, COMPASS_DIAL_MARKS, deg,
                                    DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                                    (uint8_t)DIAL_DOT_MIN,
                                    (uint8_t)DIAL_DOT_MAX);
      CHECK_EQ(n, COMPASS_DIAL_MARKS);
      checkDial(d, n, COMPASS_DIAL_MARKS, DIAL_CX, DIAL_CY, DIAL_RX, DIAL_RY,
                (uint8_t)DIAL_DOT_MIN, (uint8_t)DIAL_DOT_MAX);
      CHECK_EQ(d[0].size, DIAL_DOT_MIN);
      CHECK_EQ(d[COMPASS_DIAL_MARKS - 1].size, DIAL_DOT_MAX);
    }
  }

TEST_MAIN_END()
