// Host tests for core/StreamClock.cpp.
//
// This module exists because the stream gives us a good TIMESTAMP and no
// clock. Four hex digits of the gateway's millis() % 60000: it rolls over
// every sixty seconds and it belongs to a machine the radio powered up long
// before we booted. Everything downstream wants the opposite -- one monotonic
// millisecond count on OUR timebase that can be subtracted -- and it wants the
// fine structure of it to be the CAR's rather than our reader task's.
//
// So there are two guarantees, and the cost of breaking either is not a wrong
// number on the glass. It is a plausible one.
//
//   mapped <= hostMs. Every staleness test in this firmware is
//   `nowMs() - someMs < TTL` in UNSIGNED arithmetic, so a mapped time one
//   millisecond in the future underflows to 4.29 billion and reads as
//   maximally stale. Not an error -- a pedal that blanks at random, a tyre
//   corner that dims for no reason, and nothing to tell it from a real dropout.
//
//   mapped never decreases. core/Trip.cpp integrates fuel as
//   `flowRaw * (flowMs - prevFlowMs)`, also unsigned, so one backwards step of
//   a few milliseconds is an interval of forty-nine days and a litre appears
//   in the tank meter out of nothing.
//
// Not one caller checks either of them. That is why they are asserted here
// after EVERY call in EVERY case, through driveFeed(), rather than at the end
// of a scenario: a suite that only inspected the last value would pass a
// module that stepped backwards in the middle of a run, which is exactly where
// it would step.
//
// The headline property is case 2, and it is what the module is FOR. A reader
// blocked behind an NVS commit or an LVGL tree rebuild dequeues its backlog a
// millisecond apart and then waits, so the host clock reports {D, 1, 1, 1} for
// frames the car produced 100 ms apart. The adapter's stamps do not care, and
// after mapping neither does anything downstream.
#include "doctest_min.h"

#include "StreamClock.h"
#include "LinkLine.h"

#define MODS STREAM_CLOCK_MOD_SHORT

// ---------------------------------------------------------------------------
// One clock plus the memory needed to police it. driveFeed() is the ONLY way
// this file calls streamClockMap(), so both guarantees are asserted on every
// value the module hands out here.
//
// Both tests are SIGNED differences of unsigned values, which is how
// StreamClock.cpp itself compares and the only form that survives the host
// clock's own 49.7-day rollover. Case 10 drives straight through it.
// ---------------------------------------------------------------------------
struct StreamDrive {
  StreamClock c;
  uint32_t    prev;
  bool        havePrev;
};

static void driveInit(StreamDrive& d, uint8_t tsChars) {
  streamClockInit(d.c, tsChars);
  d.prev = 0u;
  d.havePrev = false;
}

static uint32_t driveFeed(StreamDrive& d, uint32_t raw, uint32_t hostMs) {
  const uint32_t m = streamClockMap(d.c, raw, hostMs);
  CHECK_MSG(static_cast<int32_t>(m - hostMs) <= 0,
            "mapped %u <= hostMs %u  (raw %u, sample %u)", (unsigned)m,
            (unsigned)hostMs, (unsigned)raw, (unsigned)d.c.samples);
  if (d.havePrev) {
    CHECK_MSG(static_cast<int32_t>(m - d.prev) >= 0,
              "mapped %u >= previous %u  (hostMs %u, sample %u)", (unsigned)m,
              (unsigned)d.prev, (unsigned)hostMs, (unsigned)d.c.samples);
  }
  d.prev = m;
  d.havePrev = true;
  return m;
}

// ---------------------------------------------------------------------------
// CASE 2's body, run for five stall lengths.
//
// The frames are a metronome: stamps exactly kFrameMs apart, which is close to
// what 0x1A7 really looks like (the capture has its nominal 100 ms landing
// within a few milliseconds every time). The HOST times model a reader blocked
// for `stallMs`, during which lines queue in the UART FIFO and the FreeRTOS
// queue behind it, and which then drains them one millisecond at a time.
//
// readAt = produced + latency, floored one millisecond past the previous read:
// a reader cannot see a line before it exists and cannot read two in the same
// instant. That floor IS the queue, and it only ever RAISES latency -- which
// matters, because it keeps the first sample's kLatency the smallest latency
// in the run, so the minimum-tracking offset never has cause to move and every
// interval reported is the adapter's own.
//
// Host deltas across the stall are {stallMs, 1, 1, ...}; mapped deltas are
// asserted to be exactly kFrameMs throughout. The offset is separately allowed
// to creep up by 1 ms per STREAM_CLOCK_CREEP_DIV (1024) ms of host time, which
// would show here as a single 101. It does not fire even at stallMs=1000,
// because creepRem is zeroed on every sample whose latency returns to the
// minimum and the longest stretch here spends under 1024 ms away from it.
// ---------------------------------------------------------------------------
static void stalledReaderCase(uint32_t stallMs) {
  const uint32_t kFrameMs = 100u;
  const uint32_t kLatency = 5u;
  const uint32_t kEpoch   = 80000u;   // the gateway has been up 80 s longer
  const uint32_t kWarmup  = 3u;
  const uint32_t kTail    = 6u;

  StreamDrive d;
  driveInit(d, LINK_LINE_TS_SHORT);

  uint32_t a = 20000u;                       // adapter millis, unwrapped
  uint32_t read = a + kEpoch + kLatency;     // host clock at the read
  const uint32_t firstRead = read;
  uint32_t mapped = driveFeed(d, a % MODS, read);
  const uint32_t firstMapped = mapped;
  const uint32_t firstAdapter = a;
  CHECK_EQ(mapped, read);                    // the first sample is the host's

  for (uint32_t i = 0u; i < kWarmup; ++i) {
    a += kFrameMs;
    read = a + kEpoch + kLatency;
    const uint32_t m = driveFeed(d, a % MODS, read);
    CHECK_EQ(m - mapped, kFrameMs);          // steady latency: exact, always
    mapped = m;
  }

  // THE STALL, positioned to catch the next line: the reader goes away one
  // millisecond before that line is produced, so even a 20 ms block backs one
  // up. Everything produced before it comes back is then dequeued back to
  // back.
  const uint32_t drainAt = a + kFrameMs + kEpoch - 1u + stallMs;
  uint32_t queued = 0u;
  while (a + kFrameMs + kEpoch <= drainAt) {
    a += kFrameMs;
    const uint32_t prevRead = read;
    read = drainAt + queued;
    const uint32_t m = driveFeed(d, a % MODS, read);
    const uint32_t delta = m - mapped;
    // The adapter's interval, give or take the millisecond per 1024 ms of host
    // time the offset is allowed to creep upward -- and the millisecond it
    // gives straight back when an honest low-latency sample returns. Only the
    // 1000 ms stall is long enough to buy either. Nothing here is 1, and
    // nothing here is `stallMs`, which is what the host clock said.
    CHECK_MSG(delta >= kFrameMs - 1u && delta <= kFrameMs + 1u,
              "stall %u: mapped delta %u while the host moved %u",
              (unsigned)stallMs, (unsigned)delta, (unsigned)(read - prevRead));
    // ...and the host really did say that. The first queued line is the whole
    // stall late; every one after it arrives a millisecond behind the last.
    if (queued == 0u) {
      CHECK(read - prevRead >= stallMs);
    } else {
      CHECK_EQ(read - prevRead, 1u);
    }
    mapped = m;
    ++queued;
  }
  CHECK(queued >= 1u);          // the stall really did back lines up

  // And the reader catches up. The first frames here still carry the drain's
  // inflated latency, which is exactly the shape that would make an AVERAGED
  // offset hand out a time later than the host clock it was read at.
  for (uint32_t i = 0u; i < kTail; ++i) {
    a += kFrameMs;
    const uint32_t want = a + kEpoch + kLatency;
    read = (want > read) ? want : (read + 1u);
    const uint32_t m = driveFeed(d, a % MODS, read);
    const uint32_t delta = m - mapped;
    CHECK_MSG(delta >= kFrameMs - 1u && delta <= kFrameMs + 1u,
              "stall %u tail: mapped delta %u", (unsigned)stallMs,
              (unsigned)delta);
    mapped = m;
  }

  // Over the whole run the two clocks agree to within the creep, which is the
  // statement that matters to Trip.cpp: the stall cost no time and invented
  // none.
  const uint32_t adapterSpan = a - firstAdapter;
  const uint32_t mappedSpan = mapped - firstMapped;
  const uint32_t creepCap = (read - firstRead) / STREAM_CLOCK_CREEP_DIV + 1u;
  CHECK_MSG(mappedSpan + creepCap >= adapterSpan &&
                mappedSpan <= adapterSpan + creepCap,
            "stall %u: mapped span %u vs adapter span %u", (unsigned)stallMs,
            (unsigned)mappedSpan, (unsigned)adapterSpan);

  CHECK_EQ(d.c.wraps, 0u);      // no rollover anywhere in this case
  CHECK_EQ(d.c.resyncs, 0u);    // A STALL IS NOT A RESYNC -- see case 9
}

// ---------------------------------------------------------------------------
// REAL STAMPS, out of
//   jeep-can-b-logs/drive log - becici to school ... 2026-09-17_074126.log
//
// 288 consecutive frame stamps, records 8400..8687 of the clean prefix, a
// stretch of ordinary driving that happens to contain a genuine 60 s rollover
// (59986 -> 11). Real bus timing: 1874 ms of adapter time over 288 frames,
// deltas from 0 to 43 ms, nothing evenly spaced about any of it. Synthesised
// host times go on top, because the log records the gateway's clock and not
// ours -- the stamps are the real half.
// ---------------------------------------------------------------------------
static const uint16_t kStamps[] = {
    59438, 59439, 59440, 59442, 59443, 59444, 59449, 59487, 59511, 59521, 59522, 59537,
    59538, 59539, 59540, 59542, 59543, 59544, 59546, 59586, 59611, 59618, 59619, 59621,
    59622, 59623, 59624, 59630, 59636, 59637, 59638, 59640, 59641, 59642, 59643, 59644,
    59646, 59651, 59661, 59686, 59711, 59720, 59721, 59732, 59736, 59738, 59739, 59741,
    59742, 59743, 59744, 59746, 59747, 59748, 59749, 59751, 59787, 59811, 59821, 59822,
    59836, 59838, 59839, 59840, 59841, 59843, 59844, 59845, 59886, 59911, 59921, 59922,
    59934, 59936, 59937, 59939, 59940, 59941, 59942, 59944, 59986,    11,    20,    21,
       27,    36,    38,    39,    40,    42,    43,    44,    54,    69,    87,   111,
      121,   122,   123,   124,   124,   125,   126,   127,   129,   130,   131,   132,
      133,   136,   138,   139,   140,   141,   143,   144,   146,   186,   211,   221,
      221,   226,   232,   236,   237,   239,   240,   241,   242,   244,   246,   247,
      286,   311,   320,   321,   323,   336,   338,   339,   340,   342,   343,   344,
      387,   411,   413,   421,   422,   436,   437,   439,   440,   441,   442,   444,
      486,   497,   511,   520,   521,   536,   537,   539,   540,   541,   542,   545,
      587,   589,   611,   618,   619,   620,   621,   622,   623,   631,   636,   638,
      639,   640,   641,   643,   644,   645,   660,   686,   688,   711,   721,   722,
      732,   736,   737,   739,   740,   741,   743,   744,   745,   747,   748,   749,
      750,   752,   754,   756,   786,   788,   811,   820,   821,   836,   837,   839,
      840,   841,   844,   845,   887,   888,   911,   920,   921,   936,   938,   939,
      940,   942,   943,   944,   981,   987,  1011,  1021,  1022,  1036,  1037,  1039,
     1040,  1041,  1043,  1044,  1073,  1080,  1086,  1111,  1120,  1121,  1122,  1123,
     1124,  1125,  1126,  1127,  1129,  1130,  1131,  1132,  1136,  1137,  1138,  1140,
     1142,  1143,  1144,  1145,  1147,  1160,  1187,  1211,  1220,  1221,  1232,  1236,
     1238,  1239,  1240,  1242,  1243,  1244,  1245,  1247,  1248,  1249,  1287,  1312,
};

// THE GATEWAY REBOOT, from the same capture, records 597..660. The 328 is
// powered by the radio; the radio dropped out and its millis() came back at
// zero. The stamp goes 34742 -> 7709 at index 16 of this table -- the event
// the header comment points at, and 27033 ms wide, which is not a number a
// synthetic case would have thought to pick.
static const uint16_t kReboot[] = {
    34648, 34650, 34650, 34651, 34652, 34659, 34670, 34687, 34692, 34695, 34702, 34720,
    34728, 34730, 34740, 34742,  7709,  7711,  7721,  7730,  7735,  7736,  7737,  7739,
     7740,  7741,  7742,  7744,  7745,  7746,  7747,  7749,  7785,  7804,  7812,  7821,
     7830,  7834,  7836,  7837,  7838,  7840,  7841,  7842,  7884,  7901,  7912,  7921,
     7930,  7934,  7936,  7938,  7939,  7940,  7941,  7943,  7985,  7994,  8011,  8022,
     8030,  8034,  8036,  8037,
};
static const size_t kRebootAt = 16u;   // index of the first post-reboot stamp

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // CASE 1. A fresh clock, and the first sample.
  //
  // The first call returns hostMs EXACTLY, whatever the stamp says: there is
  // no offset estimate yet and inventing one would be worse than admitting the
  // first frame is only as good as our own clock. raw > hostMs is the NORMAL
  // case rather than an edge one -- the gateway has been running for as long
  // as the key has been in, and we booted seconds ago.
  // -------------------------------------------------------------------------
  {
    StreamClock c;
    streamClockInit(c, LINK_LINE_TS_SHORT);
    CHECK_EQ(c.modulus, STREAM_CLOCK_MOD_SHORT);
    CHECK(!c.have);
    CHECK_EQ(c.samples, 0u);
    CHECK_EQ(c.wraps, 0u);
    CHECK_EQ(c.resyncs, 0u);
    CHECK_EQ(streamClockSkewMs(c), 0);

    streamClockInit(c, LINK_LINE_TS_LONG);
    CHECK_EQ(c.modulus, 0u);          // the whole 32-bit millis, no bucket

    // Any other width leaves modulus 0 as well. The header calls that
    // "unusable" and forbids calling map() -- which is a CALLER contract, not
    // a state the module can tell apart from the long width. Asserted as it
    // behaves, not as the prose reads.
    streamClockInit(c, LINK_LINE_TS_NONE);
    CHECK_EQ(c.modulus, 0u);

    static const uint32_t kRaw[]  = {0u, 1u, 30000u, 59999u, 12345u};
    static const uint32_t kHost[] = {0u, 1u, 7u, 500u, 4000000000u};
    for (size_t i = 0u; i < sizeof(kRaw) / sizeof(kRaw[0]); ++i) {
      for (size_t j = 0u; j < sizeof(kHost) / sizeof(kHost[0]); ++j) {
        StreamDrive d;
        driveInit(d, LINK_LINE_TS_SHORT);
        CHECK_EQ(driveFeed(d, kRaw[i], kHost[j]), kHost[j]);
        CHECK(d.c.have);
        CHECK_EQ(d.c.samples, 1u);
        CHECK_EQ(d.c.wraps, 0u);
        CHECK_EQ(d.c.resyncs, 0u);
        CHECK_EQ(d.c.adapterMs, kRaw[i]);        // not normalised to zero
        CHECK_EQ(streamClockSkewMs(d.c), 0);     // mapped IS the host clock
      }
    }
  }

  // -------------------------------------------------------------------------
  // CASE 2. THE HEADLINE PROPERTY: the intervals are the ADAPTER'S, and
  // host-side jitter does not reach them.
  //
  // This is the entire reason the module exists. An NVS commit disables the
  // flash cache for tens of milliseconds and an LVGL tree rebuild is worse; a
  // reader blocked that long dequeues its backlog a millisecond apart, so the
  // host clock says {D, 1, 1, 1, ...} for a run of frames the car produced 100
  // ms apart. Hand those host deltas to tripUpdate() and four intervals of
  // fuel collapse to nothing while the fifth is integrated at whatever rate
  // happened to be in the last frame -- a trip meter that stays plausible and
  // is wrong, which is the failure this whole file is about.
  // -------------------------------------------------------------------------
  {
    stalledReaderCase(20u);
    stalledReaderCase(50u);
    stalledReaderCase(100u);
    stalledReaderCase(300u);
    stalledReaderCase(1000u);
  }

  // -------------------------------------------------------------------------
  // CASE 3. ROLLOVER. A backwards step in the stamp is a wrap -- at 151
  // frames/s the next frame is ~7 ms away, so there is nothing ambiguous about
  // it.
  // -------------------------------------------------------------------------
  {
    // The canonical one from the module's own comment: 59950 then 30 is 80 ms,
    // not a step back of 59920.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    const uint32_t m0 = driveFeed(d, 59950u, 1000u);
    const uint32_t m1 = driveFeed(d, 30u, 1080u);
    CHECK_EQ(m1 - m0, 80u);
    CHECK_EQ(d.c.wraps, 1u);
    CHECK_EQ(d.c.adapterMs, 60030u);      // unwrapped past the modulus
  }
  {
    // A wrap on the VERY SECOND sample, before the estimate has seen anything
    // but its own anchor.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    const uint32_t m0 = driveFeed(d, 59999u, 500u);
    const uint32_t m1 = driveFeed(d, 0u, 501u);
    CHECK_EQ(m1 - m0, 1u);                // 59999 -> 0 is one millisecond
    CHECK_EQ(d.c.wraps, 1u);
    CHECK_EQ(d.c.samples, 2u);
  }
  {
    // Walking up to the boundary and over it a millisecond at a time:
    // 59995 ... 59999, 0, 1 ... Both of the values the modulus makes special
    // are landed on exactly.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    uint32_t host = 100000u;
    uint32_t mapped = driveFeed(d, 59995u, host);
    for (uint32_t k = 1u; k <= 10u; ++k) {
      const uint32_t raw = (59995u + k) % MODS;
      ++host;
      const uint32_t m = driveFeed(d, raw, host);
      CHECK_EQ(m - mapped, 1u);
      mapped = m;
    }
    CHECK_EQ(d.c.wraps, 1u);              // exactly one crossing in the sweep
  }
  {
    // Forwards by very nearly the whole modulus is NOT a wrap, and a repeated
    // stamp is not one either -- the two shapes an over-eager test would call
    // rollovers.
    StreamDrive a;
    driveInit(a, LINK_LINE_TS_SHORT);
    driveFeed(a, 0u, 7000u);
    CHECK_EQ(driveFeed(a, 59999u, 7000u + 59999u) - 7000u, 59999u);
    CHECK_EQ(a.c.wraps, 0u);

    StreamDrive b;
    driveInit(b, LINK_LINE_TS_SHORT);
    driveFeed(b, 59999u, 7000u);
    CHECK_EQ(driveFeed(b, 59999u, 7000u) - 7000u, 0u);
    CHECK_EQ(b.c.wraps, 0u);
  }
  {
    // SEVERAL CONSECUTIVE WRAPS. Thirty samples 20 s apart is ten crossings,
    // and the mapped clock has to come out ten minutes further on.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    const uint32_t kStep = 20000u;
    uint32_t host = 5000u;
    uint32_t adapter = 40000u;
    const uint32_t m0 = driveFeed(d, adapter % MODS, host);
    uint32_t mapped = m0;
    for (uint32_t k = 0u; k < 30u; ++k) {
      adapter += kStep;
      host += kStep;
      const uint32_t m = driveFeed(d, adapter % MODS, host);
      CHECK_EQ(m - mapped, kStep);
      mapped = m;
    }
    CHECK_EQ(d.c.wraps, 10u);
    CHECK_EQ(mapped - m0, 30u * kStep);
    CHECK_EQ(d.c.resyncs, 0u);
  }

  // -------------------------------------------------------------------------
  // CASE 4. SILENCE LONGER THAN THE MODULUS, which the stamp cannot resolve on
  // its own: sixty seconds of nothing and sixty-one look identical on four
  // digits. The host clock picks the bucket -- it only has to be right to
  // within thirty seconds to do that -- and the stamp says where inside the
  // bucket we are. Get it wrong and the ignition coming back on after a
  // shopping trip hands Trip.cpp an interval a minute short, or three minutes
  // long.
  // -------------------------------------------------------------------------
  {
    // Three whole minutes of silence, the stamp returning 40 ms past where it
    // left.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    const uint32_t m0 = driveFeed(d, 30000u, 500000u);
    const uint32_t m1 = driveFeed(d, 30040u, 500000u + 180000u);
    // The BUCKETING is right: the adapter clock advanced three whole moduli
    // plus the 40 ms the stamp moved, which no amount of staring at four hex
    // digits could have told anyone.
    CHECK_EQ(d.c.wraps, 3u);
    CHECK_EQ(d.c.adapterMs, 30000u + 180040u);
    // The RETURNED value is nonetheless the host clock exactly. Over three
    // minutes of silence the two clocks disagreed by 40 ms, and a disagreement
    // in that direction is a smaller `host - adapter` -- a new minimum, which
    // is taken outright. Which is the division of labour the module is built
    // on: the stamp for the interval, our clock for the instant.
    CHECK_EQ(m1, 680000u);
    CHECK_EQ(m1 - m0, 180000u);
  }
  {
    // The same gap with the stamp returning 40 ms SHORT of where it left, so
    // the bucket rounds the other way and the mapped clock sits a little
    // behind the host's rather than on it.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    driveFeed(d, 30000u, 500000u);
    const uint32_t m1 = driveFeed(d, 29960u, 680000u);
    CHECK_EQ(d.c.wraps, 3u);
    CHECK(680000u - m1 <= 100u);          // within a small tolerance
  }
  {
    // Pushing the boundary: four gaps either side of the modulus, each on a
    // clock of its own. The gap is what the two clocks really elapsed, and the
    // stamp arithmetic has to agree with it.
    static const uint32_t kGap[] = {59500u, 60000u, 60500u, 119900u};
    static const uint32_t kWraps[] = {1u, 1u, 1u, 2u};
    for (size_t i = 0u; i < sizeof(kGap) / sizeof(kGap[0]); ++i) {
      StreamDrive d;
      driveInit(d, LINK_LINE_TS_SHORT);
      const uint32_t base = 30000u;
      const uint32_t m0 = driveFeed(d, base, 200000u);
      const uint32_t raw = (base + kGap[i]) % MODS;
      const uint32_t m1 = driveFeed(d, raw, 200000u + kGap[i]);
      CHECK_MSG(m1 - m0 == kGap[i], "gap %u: mapped delta %u",
                (unsigned)kGap[i], (unsigned)(m1 - m0));
      CHECK_MSG(d.c.wraps == kWraps[i], "gap %u: wraps %u", (unsigned)kGap[i],
                (unsigned)d.c.wraps);
    }
  }

  // -------------------------------------------------------------------------
  // CASE 5. GATEWAY REBOOT, and the point is that THERE IS NO SPECIAL CASE FOR
  // IT. The 328 is powered by the radio and restarts when the radio does; its
  // millis() begins again at zero, the stamp jumps backwards by an arbitrary
  // amount, and this module reads that as a wrap and over-advances its adapter
  // clock by up to a minute. It recovers on the NEXT SAMPLE, for free, because
  // `host - adapter` falls by that same amount and the offset tracks the
  // MINIMUM -- a better estimate is taken at once rather than crept toward.
  //
  // Driven off the real event in the capture: 34742 -> 7709, 613 frames in.
  // Note what the stamp itself says -- the gateway reports 7709 ms of uptime
  // on its first frame back, so the real gap cannot be shorter than that.
  // Nothing in the module needs to know it.
  // -------------------------------------------------------------------------
  {
    const uint32_t kGapMs = 8200u;       // radio off, 328 boots, 7709 ms in
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);

    uint32_t host = 400000u;
    uint32_t mapped = driveFeed(d, kReboot[0], host);
    for (size_t i = 1u; i < kRebootAt; ++i) {
      const uint32_t step = kReboot[i] - kReboot[i - 1];
      host += step;
      const uint32_t m = driveFeed(d, kReboot[i], host);
      CHECK_EQ(m - mapped, step);         // ordinary real frames, real deltas
      mapped = m;
    }
    CHECK_EQ(d.c.resyncs, 0u);
    CHECK_EQ(d.c.wraps, 0u);

    // THE REBOOT SAMPLE.
    host += kGapMs;
    const uint32_t mReboot = driveFeed(d, kReboot[kRebootAt], host);
    CHECK_EQ(d.c.resyncs, 1u);                     // one, and only one
    CHECK_EQ(d.c.wraps, 1u);                       // read as an ordinary wrap
    CHECK_EQ(mReboot, host);                       // back on the host clock
    CHECK(mReboot >= mapped);                      // and never backwards
    mapped = mReboot;

    // And the adapter's deltas are accurate again IMMEDIATELY -- not after a
    // settling period, and not after the creep has caught up.
    for (size_t i = kRebootAt + 1u; i < sizeof(kReboot) / sizeof(kReboot[0]); ++i) {
      const uint32_t step = kReboot[i] - kReboot[i - 1];
      host += step;
      const uint32_t m = driveFeed(d, kReboot[i], host);
      CHECK_MSG(m - mapped == step, "post-reboot frame %u: delta %u want %u",
                (unsigned)i, (unsigned)(m - mapped), (unsigned)step);
      mapped = m;
    }
    CHECK_EQ(d.c.resyncs, 1u);            // nothing after the event itself
    CHECK_EQ(d.c.samples, sizeof(kReboot) / sizeof(kReboot[0]));
    CHECK_EQ(streamClockSkewMs(d.c), 0);
  }
  {
    // The same thing synthesised, to pin the WORST reboot rather than the one
    // the capture happened to record: a restart that leaves the stamp almost a
    // whole modulus behind, which is the largest over-advance possible.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    uint32_t host = 1000000u;
    driveFeed(d, 59900u, host);
    host += 100u;
    driveFeed(d, 59999u, host);
    host += 12000u;                       // the radio cycles
    const uint32_t m = driveFeed(d, 3u, host);
    CHECK_EQ(m, host);                    // one sample, and it is right again
    CHECK_EQ(d.c.resyncs, 1u);
    uint32_t mapped = m;
    for (uint32_t k = 0u; k < 20u; ++k) {
      host += 100u;
      const uint32_t mm = driveFeed(d, 103u + k * 100u, host);
      CHECK_EQ(mm - mapped, 100u);
      mapped = mm;
    }
    CHECK_EQ(d.c.resyncs, 1u);
  }

  // -------------------------------------------------------------------------
  // CASE 6. CRYSTAL DRIFT, which is why the minimum is allowed to CREEP.
  //
  // STREAM_CLOCK_CREEP_DIV is 1024: the offset may rise one millisecond per
  // 1024 ms of host time, which is 1/1024 = 977 ppm. A cheap MCU crystal or
  // resonator is a few tens of ppm and a pair 100 ppm apart would be a bad
  // one, so 977 ppm is most of an order of magnitude past any plausible pair
  // -- genuine drift is always tracked -- while still being far slower than
  // the rate at which transport latency varies, so a burst of queueing cannot
  // drag the estimate up with it.
  //
  // A PURE minimum could never rise, so a host clock slower than the adapter's
  // would leave the estimate stuck at the first sample and the mapping would
  // walk away from the host clock by a few hundred milliseconds an hour, until
  // it crossed a TTL and readouts began dimming on a healthy bus. So the
  // assertion is that `hostMs - mapped` stays BOUNDED over a simulated hour
  // rather than growing.
  //
  // Stepped 1000 ms at a time to keep the suite quick; the ratio is exact,
  // 100 ppm as integer arithmetic -- one millisecond per 10 000.
  // -------------------------------------------------------------------------
  {
    // Host FAST relative to the adapter: host - adapter rises, so every sample
    // looks late by a hair and only the creep can follow it.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    const uint32_t kH0 = 2000000u;
    const uint32_t kA0 = 15000u;
    driveFeed(d, kA0 % MODS, kH0);
    uint32_t worst = 0u;
    for (uint32_t a = 1000u; a <= 3600000u; a += 1000u) {
      const uint32_t host = kH0 + a + a / 10000u;         // +100 ppm
      const uint32_t m = driveFeed(d, (kA0 + a) % MODS, host);
      const uint32_t behind = host - m;
      if (behind > worst) {
        worst = behind;
      }
    }
    CHECK_MSG(worst <= 8u, "host-fast drift: worst lag %u ms", (unsigned)worst);
    CHECK_EQ(d.c.resyncs, 0u);            // drift is tracked, never resynced
    CHECK_EQ(d.c.wraps, 60u);             // an hour of a 60 s modulus
  }
  {
    // Host SLOW relative to the adapter: host - adapter falls, so every sample
    // is a new minimum and the estimate follows it down at once. This is the
    // direction a pure minimum handles for free; it is here to show the creep
    // did not break it.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    const uint32_t kH0 = 2000000u;
    const uint32_t kA0 = 15000u;
    driveFeed(d, kA0 % MODS, kH0);
    uint32_t worst = 0u;
    for (uint32_t a = 1000u; a <= 3600000u; a += 1000u) {
      const uint32_t host = kH0 + a - a / 10000u;         // -100 ppm
      const uint32_t m = driveFeed(d, (kA0 + a) % MODS, host);
      const uint32_t behind = host - m;
      if (behind > worst) {
        worst = behind;
      }
    }
    CHECK_MSG(worst <= 8u, "host-slow drift: worst lag %u ms", (unsigned)worst);
    CHECK_EQ(d.c.resyncs, 0u);
    CHECK_EQ(d.c.wraps, 60u);
  }

  // -------------------------------------------------------------------------
  // CASE 7. THE 8-DIGIT WIDTH. LINK_LINE_TS_LONG is the adapter's whole 32-bit
  // millis(), where unsigned subtraction unwraps on its own: modulus 0, no
  // bucket to choose, no wrap to count. The host clock is not consulted about
  // the interval at all, which is the strongest form of what case 2 tests.
  // -------------------------------------------------------------------------
  {
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_LONG);
    CHECK_EQ(d.c.modulus, 0u);
    uint32_t adapter = 4000000000u;       // far past the 4-digit modulus
    uint32_t host = 60000u;
    uint32_t mapped = driveFeed(d, adapter, host);
    for (uint32_t k = 0u; k < 50u; ++k) {
      adapter += 100u;
      host += 100u;
      const uint32_t m = driveFeed(d, adapter, host);
      CHECK_EQ(m - mapped, 100u);
      mapped = m;
    }
    CHECK_EQ(d.c.wraps, 0u);

    // A gap longer than the SHORT modulus gets no bucket correction, because
    // there is nothing to correct -- the stamp already says three minutes.
    adapter += 180000u;
    host += 180000u;
    const uint32_t m = driveFeed(d, adapter, host);
    CHECK_EQ(m - mapped, 180000u);
    CHECK_EQ(d.c.wraps, 0u);
  }
  {
    // The 32-bit counter's own rollover, walked from 256 ms before 2^32 to
    // 344 ms after: unsigned subtraction gives the right small delta and
    // nothing here has to know it happened.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_LONG);
    uint32_t adapter = 0xFFFFFF00u;
    uint32_t host = 8000u;
    uint32_t mapped = driveFeed(d, adapter, host);
    for (uint32_t k = 0u; k < 20u; ++k) {
      adapter += 30u;                     // straight over 0xFFFFFFFF
      host += 30u;
      const uint32_t m = driveFeed(d, adapter, host);
      CHECK_EQ(m - mapped, 30u);
      mapped = m;
    }
    CHECK(adapter < 0x1000u);             // the counter really did wrap
    CHECK_EQ(d.c.wraps, 0u);
    CHECK_EQ(d.c.resyncs, 0u);
  }
  {
    // A GATEWAY REBOOT ON THE 8-DIGIT WIDTH, which is a different shape from
    // case 5's. With no modulus there is no wrap to mistake it for: the step
    // is `raw - lastRaw` and the adapter clock moves BACKWARDS by 27 s. That
    // lands far outside the resync window, so the estimate is replaced and the
    // mapping is the host clock again on the same sample -- no clamp needed.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_LONG);
    uint32_t host = 500000u;
    uint32_t mapped = driveFeed(d, 34740u, host);
    host += 2u;
    mapped = driveFeed(d, 34742u, host);
    host += 8200u;
    const uint32_t m = driveFeed(d, 7709u, host);
    CHECK_EQ(m, host);
    CHECK(m >= mapped);
    CHECK_EQ(d.c.resyncs, 1u);
    CHECK_EQ(d.c.wraps, 0u);
  }
  {
    // THE ONE PATH THAT REACHES THE MONOTONIC FLATTEN, and it is worth knowing
    // that it is the only one. On the 4-digit width the step is unwrapped into
    // [0, modulus) and whole moduli are only ever ADDED, so adapterMs cannot
    // move backwards and the raw mapping cannot either -- a fuzz of four
    // million short-width samples over arbitrary stamps and an arbitrary
    // non-decreasing host clock never once needed the clamp. On the 8-digit
    // width there is no unwrapping to do, so a stamp that steps backwards by
    // less than the resync window -- a corrupt or duplicated line, since a
    // reboot lands well outside it -- takes adapterMs with it.
    //
    // The interval loses its resolution and nothing else: the value is held,
    // not rolled back, which is what keeps `flowMs - prevFlowMs` out of the
    // 49-day range.
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_LONG);
    uint32_t host = 1000000u;
    const uint32_t m0 = driveFeed(d, 10000000u, host);
    host += 10u;
    const uint32_t held = driveFeed(d, 10000010u, host);
    CHECK_EQ(held, host);
    host += 1u;
    const uint32_t m = driveFeed(d, 10000008u, host);   // stamp steps back 2
    CHECK_EQ(m, held);                  // flattened to zero, not negative
    CHECK(m < host);
    CHECK_EQ(d.c.resyncs, 0u);          // 3 ms of disagreement is not a resync

    // It recovers on its own on the next sane stamp, and the two milliseconds
    // the flatten held back come off the FOLLOWING interval rather than being
    // conjured: 98, not 100. So the header's "costs that one interval its
    // resolution and costs nothing else" is true of the INTEGRAL, which is
    // what Trip.cpp cares about, and slightly generous about the interval
    // after -- the cumulative advance matches the adapter's exactly.
    host += 100u;
    const uint32_t m2 = driveFeed(d, 10000108u, host);
    CHECK_EQ(m2 - m, 98u);
    CHECK_EQ(m2 - m0, 10000108u - 10000000u);
  }

  // -------------------------------------------------------------------------
  // CASE 8. streamClockSkewMs(), a diagnostic and nothing else -- no decision
  // anywhere rests on it. Zero before the first sample and never positive
  // after, which is the `mapped <= hostMs` guarantee restated in a form a log
  // line can print.
  //
  // It CONVERGES DOWNWARD, which is the part worth asserting. The value on an
  // ordinary sample is (smallest latency seen) - (this sample's latency), so a
  // lower-latency frame arriving does not make the skew smaller in magnitude
  // -- it makes every ordinary sample after it read further behind. That is
  // the estimate getting BETTER, and a reading that moved the other way would
  // mean the minimum had been contaminated by a late frame.
  // -------------------------------------------------------------------------
  {
    StreamClock fresh;
    streamClockInit(fresh, LINK_LINE_TS_SHORT);
    CHECK_EQ(streamClockSkewMs(fresh), 0);

    const uint32_t kEpoch = 90000u;
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);

    // Anchor at a latency of 50 ms and hold it there. With every sample as
    // late as the first, the estimate is as good as it can see and the skew
    // reads zero -- it measures EXCESS latency, not absolute latency.
    uint32_t a = 10000u;
    driveFeed(d, a, a + kEpoch + 50u);
    for (uint32_t k = 0u; k < 5u; ++k) {
      a += 100u;
      driveFeed(d, a, a + kEpoch + 50u);
      CHECK_EQ(streamClockSkewMs(d.c), 0);
    }

    // One frame gets through in 10 ms. That is a strictly better estimate of
    // the offset, so it is taken at once rather than crept toward.
    a += 100u;
    driveFeed(d, a, a + kEpoch + 10u);
    CHECK_EQ(streamClockSkewMs(d.c), 0);

    // From here an ordinary 50 ms sample reads 40 ms behind. The skew has
    // converged DOWNWARD and is now telling the truth about how much of the
    // 50 ms is queueing.
    for (uint32_t k = 0u; k < 5u; ++k) {
      a += 100u;
      driveFeed(d, a, a + kEpoch + 50u);
      CHECK_EQ(streamClockSkewMs(d.c), -40);
    }
    // Better again: 4 ms through the link, and the ordinary sample after it
    // reads 46 behind.
    a += 100u;
    driveFeed(d, a, a + kEpoch + 4u);
    a += 100u;
    driveFeed(d, a, a + kEpoch + 50u);
    CHECK_EQ(streamClockSkewMs(d.c), -46);
    CHECK(streamClockSkewMs(d.c) <= 0);
    CHECK_EQ(d.c.resyncs, 0u);            // none of that is a resync
  }

  // -------------------------------------------------------------------------
  // CASE 9. THE COUNTERS, and one distinction inside them that matters.
  //
  // `resyncs` fires when the two clocks disagree by more than
  // STREAM_CLOCK_RESYNC_MS in EITHER direction, and must NOT fire on an
  // ordinary stall within it. A stall is not evidence that the offset changed
  // -- it is evidence that one frame was late -- and resyncing on it would
  // throw away a good estimate in favour of a sample already known to be
  // carrying queueing delay. Which is why the threshold sits above the worst
  // host-side stall (an NVS commit, an LVGL rebuild) and well below half the
  // modulus, where a mis-bucketed wrap would land.
  // -------------------------------------------------------------------------
  {
    const uint32_t kEpoch = 70000u;
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);

    // samples counts EVERY call, the anchor included. Latency is held at zero
    // through the clean frames so the offset estimate stands exactly still and
    // the disagreement being measured is the one being injected -- otherwise
    // the creep moves the estimate a millisecond or two under the test and the
    // threshold is no longer where the test thinks it is.
    uint32_t a = 1000u;
    driveFeed(d, a, a + kEpoch);
    CHECK_EQ(d.c.samples, 1u);
    a += 100u;
    driveFeed(d, a, a + kEpoch);
    a += 100u;
    driveFeed(d, a, a + kEpoch);
    CHECK_EQ(d.c.samples, 3u);
    CHECK_EQ(d.c.resyncs, 0u);

    // A stall of exactly STREAM_CLOCK_RESYNC_MS sits ON the threshold, and the
    // test is `>`, so it is not a resync. This is the case the whole constant
    // exists for: the frame is late, we know it is late, and the standing
    // estimate is better evidence about the offset than it is.
    a += 100u;
    driveFeed(d, a, a + kEpoch + STREAM_CLOCK_RESYNC_MS);
    CHECK_EQ(d.c.resyncs, 0u);

    // Back to an honest frame, far enough along that the host clock has moved
    // past the stalled read. The creep put a millisecond or two on the
    // estimate during the stall and this takes it straight off again -- a
    // correction, not a resync.
    a += 2100u;
    driveFeed(d, a, a + kEpoch);
    CHECK_EQ(d.c.resyncs, 0u);

    // One millisecond past the threshold and the estimate is replaced outright.
    a += 100u;
    driveFeed(d, a, a + kEpoch + STREAM_CLOCK_RESYNC_MS + 1u);
    CHECK_EQ(d.c.resyncs, 1u);

    // ...and the next honest frame is now more than the window BELOW the
    // estimate that resync just took, so it resyncs back down. Both directions
    // count, which is what makes this a diagnostic worth logging: a link that
    // keeps resyncing is a link whose stalls have outgrown the window.
    a += 2100u;
    driveFeed(d, a, a + kEpoch);
    CHECK_EQ(d.c.resyncs, 2u);
    CHECK_EQ(d.c.samples, 7u);
    CHECK_EQ(d.c.wraps, 0u);
  }
  {
    // A long run of ordinary jitter inside the window: not one resync, however
    // ugly it gets. 1900 ms is a worse stall than this firmware has ever been
    // measured taking, and it is still only a late frame -- the estimate that
    // survives it is worth more than the one that frame would give.
    //
    // Frames 2 s apart, so the jitter can swing the whole way down again
    // without the host clock having to move backwards. That crosses the
    // modulus three times over the run, which is the point of asserting wraps
    // here too: a run this jittery must not confuse a rollover for a stall.
    const uint32_t kEpoch = 70000u;
    static const uint32_t kJitter[] = {3u, 40u, 900u, 12u, 1900u, 7u, 1500u, 0u, 250u};
    const size_t kJitterN = sizeof(kJitter) / sizeof(kJitter[0]);
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    uint32_t a = 1000u;
    driveFeed(d, a % MODS, a + kEpoch + kJitter[0]);
    for (uint32_t pass = 0u; pass < 10u; ++pass) {
      for (size_t i = 0u; i < kJitterN; ++i) {
        a += 2000u;
        driveFeed(d, a % MODS, a + kEpoch + kJitter[i]);
      }
    }
    CHECK_EQ(d.c.resyncs, 0u);
    CHECK_EQ(d.c.samples, 1u + 10u * kJitterN);
    CHECK_EQ(d.c.wraps, 3u);
  }

  // -------------------------------------------------------------------------
  // CASE 10. A HOST CLOCK THAT WRAPS MID-RUN.
  //
  // The firmware's uptime clock is a 64-bit timer truncated to uint32_t, so it
  // steps from 0xFFFFFFFF to 0 once every 49.7 days -- and this instrument
  // lives in a vehicle that may well sit on that clock for weeks. Every
  // comparison in the module is a signed difference of unsigned values for
  // that reason, and the only way to find out whether one of them was written
  // the other way is to drive a run over the step on a laptop.
  // -------------------------------------------------------------------------
  {
    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);
    const uint32_t kStart = 0xFFFFF000u;          // ~4 s before the rollover
    uint32_t a = 27000u;
    uint32_t host = kStart;
    uint32_t mapped = driveFeed(d, a % MODS, host);
    CHECK_EQ(mapped, kStart);

    // Eight seconds of frames straight across the step, with a periodic stall
    // so the offset estimate is being worked at the same time.
    for (uint32_t k = 1u; k <= 80u; ++k) {
      a += 100u;
      const uint32_t lat = (k % 13u == 0u) ? 400u : 3u;
      host = kStart + k * 100u + lat - 3u;
      const uint32_t m = driveFeed(d, a % MODS, host);
      const uint32_t delta = m - mapped;          // unsigned, wraps correctly
      CHECK_MSG(delta == 100u, "host-wrap frame %u: delta %u", (unsigned)k,
                (unsigned)delta);
      mapped = m;
    }
    CHECK(host < kStart);                         // the host clock really wrapped
    CHECK_EQ(d.c.wraps, 0u);
    CHECK_EQ(d.c.resyncs, 0u);
    CHECK_EQ(mapped - kStart, 8000u);
  }

  // -------------------------------------------------------------------------
  // CASE 11. REAL STAMPS. 288 consecutive frames out of the drive capture,
  // containing a genuine rollover, driven through a synthesised reader that
  // stalls -- the log records the gateway's clock and not ours, so the stamps
  // are the real half and the host times are the modelled half.
  //
  // Worth more than any of the synthetic cases for one reason: the spacing is
  // what the bus and the gateway's polling loop actually produce, including
  // repeated stamps (two frames inside one millisecond), 43 ms holes, and
  // bursts of six frames a millisecond apart. Nothing here was chosen to be
  // convenient.
  //
  // The assertion is cumulative, and exact to the milliseconds the offset is
  // permitted to creep over this window: mapped advance == adapter advance,
  // plus at most `host span / STREAM_CLOCK_CREEP_DIV + 1`.
  // -------------------------------------------------------------------------
  {
    const size_t kN = sizeof(kStamps) / sizeof(kStamps[0]);
    const uint32_t kEpoch = 123456u;
    const uint32_t kMinLat = 3u;

    StreamDrive d;
    driveInit(d, LINK_LINE_TS_SHORT);

    uint32_t elapsed = 0u;                        // unwrapped adapter time
    uint32_t read = kStamps[0] + kEpoch + kMinLat;
    const uint32_t hostFirst = read;
    const uint32_t m0 = driveFeed(d, kStamps[0], read);
    uint32_t prevMapped = m0;

    for (size_t i = 1u; i < kN; ++i) {
      const uint32_t prevRaw = kStamps[i - 1];
      const uint32_t raw = kStamps[i];
      const uint32_t step = (raw >= prevRaw) ? (raw - prevRaw)
                                             : (MODS - (prevRaw - raw));
      elapsed += step;

      // The reader: latency wanders, and every 61st line comes out from behind
      // a flash write. Floored one millisecond past the previous read, which
      // is the queue -- and which only ever RAISES latency, so kMinLat stays
      // the minimum and the offset has no cause to move.
      const uint32_t lat = kMinLat + (uint32_t)((i * 37u) % 23u) +
                           ((i % 61u == 0u) ? 250u : 0u);
      const uint32_t want = kStamps[0] + elapsed + kEpoch + lat;
      read = (want > read) ? want : (read + 1u);

      const uint32_t m = driveFeed(d, raw, read);
      const uint32_t adv = m - m0;
      const uint32_t creepCap = (read - hostFirst) / STREAM_CLOCK_CREEP_DIV + 1u;
      CHECK_MSG(adv >= elapsed && adv <= elapsed + creepCap,
                "real frame %u: mapped advance %u, adapter advance %u",
                (unsigned)i, (unsigned)adv, (unsigned)elapsed);
      CHECK(m >= prevMapped);
      prevMapped = m;
    }

    CHECK_EQ(d.c.samples, kN);
    CHECK_EQ(d.c.wraps, 1u);                      // 59986 -> 11, once
    CHECK_EQ(d.c.resyncs, 0u);                    // no stall here is a resync
    CHECK_EQ(elapsed, 1874u);                     // 1.874 s of real bus time
    CHECK(prevMapped - m0 >= 1874u);
    CHECK(prevMapped - m0 <= 1874u + 4u);
  }

TEST_MAIN_END()
