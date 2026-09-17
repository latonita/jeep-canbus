#pragma once
// Turning the adapter's ROLLING stamp into a clock a trip meter can subtract.
//
// Under AT@SML2 every streamed frame line carries four hex digits of the
// gateway's `millis() % 60000` -- bit-exact LAWICEL "Z1". That is a good
// timestamp and a useless clock: it wraps every sixty seconds, it belongs to a
// different machine's epoch, and it says nothing about our own. Everything
// downstream wants the opposite -- one monotonic millisecond count, on roughly
// the host's timebase, that can be differenced.
//
// WHAT EACH CONSUMER ACTUALLY NEEDS, because the two halves pull in opposite
// directions and that is the whole design problem here:
//
//   DIFFERENCES must be the ADAPTER'S. core/Trip.cpp integrates fuel as
//   `flowRaw * (flowMs - prevFlowMs)`, and core/Accel.cpp times a 0-60 as
//   `stampMs - startMs`. Both want the interval the CAR produced, not the
//   interval our reader task happened to observe. A reader stalled behind an
//   NVS commit or an LVGL flush dequeues five queued lines a few hundred
//   microseconds apart and then waits; on the host clock four of those
//   intervals collapse to nothing and the fifth swallows the stall.
//
//   ABSOLUTE VALUES must be the HOST'S. Every staleness test in this firmware
//   is `nowMs() - someMs < TTL` against esp_timer -- BC_PEDAL_TTL_MS,
//   TRIP_MAX_GAP_MS, the 5 s rpm and speed windows in tripUpdate(),
//   UI_TYRE_TTL_MS. Hand those a stamp from a foreign epoch and every readout
//   on the panel reads stale for ever.
//
// So this maps one onto the other: the adapter's clock for the fine structure,
// anchored to the host's for the absolute value. Which is a one-way-delay
// problem, and is solved the way one-way delay always is.
//
// ---- THE OFFSET IS A MINIMUM, NOT AN AVERAGE ----------------------------
//
// host = adapter + offset + latency, and latency >= 0 -- a line cannot reach
// the reader before the gateway stamped it. So over any run of samples the
// SMALLEST observed `host - adapter` is the best estimate of the true offset,
// and every larger one is that estimate plus however long that particular line
// spent in a UART FIFO and a FreeRTOS queue.
//
// An average would be wrong in a way that matters rather than merely being
// looser. Take the mean and the mapped time sits in the MIDDLE of the latency
// distribution, so half of all frames map to a time LATER than the host clock
// reading at which we read them -- and `nowMs() - stampMs` is unsigned
// everywhere it is used, so a stamp one millisecond in the future underflows to
// 4.29 billion and reads as maximally stale. A pedal that blanks at random,
// a trip that drops intervals. With a minimum, `mapped <= hostMs` holds BY
// CONSTRUCTION (see streamClockMap), and no caller has to know that.
//
// ---- AND IT CREEPS, because two crystals are two crystals ---------------
//
// A pure minimum can never rise, so a host clock running slower than the
// adapter's would have the estimate stuck at the first sample for ever, drifting
// further out by a few hundred milliseconds an hour until something snapped it
// back. The offset is therefore allowed to rise by one millisecond per
// STREAM_CLOCK_CREEP_DIV milliseconds of host time -- about 1000 ppm, which is
// an order of magnitude past any plausible crystal pair and still slow enough
// that a burst of high latency cannot drag the estimate up with it.
//
// ---- WRAPS ARE COUNTED FROM THE STAMP, BUCKETED BY THE HOST -------------
//
// A backwards step in the raw stamp is a rollover, and at the measured 152
// frames/s on this bus the next frame is ~6.6 ms away, so the step is
// unambiguous -- nothing legitimate steps back. What is
// NOT recoverable from the stamp alone is a gap longer than the modulus: sixty
// seconds of silence and sixty-one look identical. The host clock picks the
// bucket -- it only has to be right to within thirty seconds to do that -- and
// the stamp supplies the millisecond inside it. Neither clock is trusted for
// the job the other one is better at, which is the whole trick.
//
// ---- AND A GATEWAY REBOOT NEEDS NO SPECIAL CASE ------------------------
//
// Worth stating because it looks like it would. The 328 is powered by the radio
// and restarts when the radio does; its millis() then begins again at zero and
// the stamp jumps backwards by an arbitrary amount, which this reads as a wrap
// and over-advances the adapter clock by up to a minute. The offset estimate
// then sees `host - adapter` fall by that same amount, and because it tracks the
// MINIMUM it takes the new value immediately rather than creeping to it. One
// sample, and the mapping is right again. The capture at
// `jeep-can-b-logs/drive log ... 2026-09-17_074126.log` has exactly this event
// in it, 613 frames in.
//
// ---- WHERE THE TWO RULES TRADE, and they do ----------------------------
//
// "Differences are the adapter's" and "absolute values are the host's" are not
// always both satisfiable, and it is worth knowing which one yields. AN INTERVAL
// ACROSS A NEW MINIMUM IS THE HOST'S, NOT THE ADAPTER'S. When a lower-latency
// sample arrives the offset takes it outright -- that is the whole point of a
// minimum filter -- and the mapping snaps by the size of the improvement, so the
// one interval spanning that snap is short by exactly that much.
//
// Measured on a 180 s silence where the adapter's clock ran 40 ms fast over the
// gap: the returned delta is 180000, the host's, and not the 180040 the adapter
// counted -- even though adapterMs really did advance 180040. The same happens
// on every reboot recovery.
//
// This is the right way round. The snap is bounded by the latency spread (a few
// hundred milliseconds at worst) and happens rarely, whereas letting the offset
// lag would put every SUBSEQUENT absolute value out by the same amount for as
// long as the creep took to catch up. One wrong interval beats a persistently
// wrong clock.
//
// NOT A PLL, and deliberately not. There is no frequency estimate and no
// filtering of the interval itself: the adapter's deltas are passed through
// untouched, which is the entire point of having them. Only the OFFSET is
// filtered, and only because the alternative is reading a stale dashboard.
//
// ACCURACY CEILING, so nobody hunts for precision that was never there: the
// gateway POLLS frames out of the MCP2515 in its main loop and stamps them when
// the loop reached the controller, not when the bus did. The capture above shows
// that loop turning over fast enough for 0x1A7's nominal 100 ms period to land
// within a few milliseconds each time, which is far better than the host clock
// manages -- but it is not bus time and this file cannot make it so.
//
// PURE: no ESP-IDF, no Arduino.
#include <stdint.h>

// The modulus of a 4-digit stamp. 59999 == 0xEA5F, so `millis() % 60000` always
// fits the four nibbles exactly -- which is why LAWICEL chose it and why this
// number is not ours to pick.
#define STREAM_CLOCK_MOD_SHORT 60000u

// How far the two clocks may disagree before the offset estimate is thrown away
// and replaced outright instead of being crept toward.
//
// It has to sit ABOVE the worst host-side stall -- an NVS commit disables the
// flash cache for tens of milliseconds, and a rebuild of the LVGL tree is worse
// -- because a stall is not evidence that the offset changed, and resyncing on
// one would discard a good estimate in favour of a sample we already know is
// late. And it has to sit WELL BELOW half the modulus, so that a mis-bucketed
// wrap (30 s out) is always caught. Two seconds clears the first by a factor of
// several and the second by fifteen.
#define STREAM_CLOCK_RESYNC_MS 2000u

// Milliseconds of host time per millisecond the offset is allowed to rise:
// 1/1024, about 977 ppm. Above any crystal pair by an order of magnitude, so
// genuine drift is always tracked; far below the rate at which latency varies,
// so a slow burst cannot pull the estimate along with it.
#define STREAM_CLOCK_CREEP_DIV 1024u

struct StreamClock {
  // The rolling modulus, in milliseconds. STREAM_CLOCK_MOD_SHORT for a 4-digit
  // stamp; ZERO means the stamp is the adapter's whole 32-bit millis() (the
  // 8-digit width), where unsigned subtraction wraps correctly on its own and
  // there is no bucket to choose.
  uint32_t modulus;

  uint32_t lastRaw;      // the stamp as it arrived, un-unwrapped
  uint32_t adapterMs;    // unwrapped, in the ADAPTER's epoch
  uint32_t offset;       // + adapterMs = our timebase. Modular; compare signed.
  uint32_t creepRem;     // host milliseconds not yet spent on creep
  uint32_t lastHostMs;   // host clock at the previous accepted sample
  uint32_t lastMs;       // the previous value handed out; never goes backwards

  uint32_t samples;
  uint32_t wraps;        // rollovers crossed, including host-bucketed ones
  uint32_t resyncs;      // times the offset estimate was replaced outright
  bool     have;         // false until the first sample anchors the clock
};

// Prepare a clock for a stamp of `tsChars` hex digits -- 4 or 8, the
// LINK_LINE_TS_* values.
//
// AND ANY OTHER WIDTH IS THE 8-DIGIT CONFIGURATION, not an unusable clock. This
// comment claimed the latter and a host suite proved otherwise: LINK_LINE_TS_NONE
// yields `modulus == 0`, which is bit-identical to LINK_LINE_TS_LONG, so a
// mistaken streamClockMap() call on an unstamped stream is undetectable here and
// would map the constant zero it was handed. Harmless as things stand -- the
// gateway only ever appends 4 or 8 digits, and At328Link only calls this with
// what the level implies -- but there is no trap here, so do not rely on one.
void streamClockInit(StreamClock& c, uint8_t tsChars);

// Fold in one stamp and get back our own monotonic millisecond count.
//
// `raw` is the stamp exactly as linkParseStreamLine() handed it over -- rolling,
// in the adapter's epoch. `hostMs` is our clock at the moment the line was read.
//
// Two guarantees the callers downstream rely on without checking:
//
//   mapped <= hostMs     -- so `hostMs - mapped` is a real age and every
//                           unsigned TTL test in this firmware keeps working.
//   mapped >= last mapped -- so an interval is never negative. A backwards step
//                           is FLATTENED to zero rather than allowed through.
//
// THE FLATTEN IS UNREACHABLE ON THE 4-DIGIT WIDTH, which is the one this host
// runs, and that is a fact about the arithmetic rather than a claim about the
// inputs: with a modulus the step is unwrapped into [0, modulus) and whole
// moduli are only ever ADDED, so adapterMs cannot move backwards, and a
// non-decreasing hostMs then makes the raw mapping non-decreasing too. Four
// million random short-width samples never fired it. It IS reachable at width 8,
// where `step = raw - lastRaw` and a corrupt or duplicated line stepping
// backwards by less than the resync window drags adapterMs with it -- so the
// guard stays, and the suite covers it there.
//
// What a flatten costs is TWO intervals, not one: the one it truncates and the
// one after it, which reads short by the same amount (a 2 ms flatten leaves the
// next interval at 98 rather than 100). The CUMULATIVE advance stays exact,
// which is what core/Trip.cpp integrates, so the total is untouched either way.
//
// The FIRST call returns `hostMs` exactly. There is no offset estimate yet and
// inventing one would be worse than admitting the first frame is only as good as
// the host clock.
uint32_t streamClockMap(StreamClock& c, uint32_t raw, uint32_t hostMs);

// How far behind our clock the mapped time runs, in milliseconds: `mapped -
// hostMs` for the most recent sample. Always <= 0, a diagnostic only, and zero
// before the first sample.
//
// IT IS NOT "the smallest latency seen", which is what this said until a host
// suite pinned the actual behaviour. It is (smallest latency seen) minus (THIS
// sample's latency) -- the current line's EXCESS queueing delay, negated. So it
// reads 0 whenever the current sample is itself the minimum: on the first
// sample, on every sample of a constant-latency link, and immediately after
// every resync. A run of zeroes here means the link is delivering evenly, not
// that the mapping is doing nothing.
int32_t streamClockSkewMs(const StreamClock& c);
