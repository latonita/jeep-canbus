// Host tests for core/Snapshot.cpp.
//
// These exist because the rules in here used to be UNTESTABLE. Staleness,
// psi -> bar, the low-corner test and the L/h <-> L/100km hysteresis all lived
// inside an LVGL-bound translation unit, written five slightly different ways
// against five different clocks, and the only way to check any of them was to
// flash a board and drive somewhere.
//
// So the emphasis here is deliberately NOT on arithmetic -- the conversions
// are one multiply each and could hardly be wrong. It is on the three-state
// age model, which is the part that carries a product decision (hold and dim a
// gap, dash only an explicit unavailable), and on the hysteresis, which is the
// part with memory and therefore the part that can flicker.
#include "doctest_min.h"

#include "Snapshot.h"

// Local SignalDefs, like test_state: VehicleState keys on the pointer, so a
// plain local struct is exactly the right fixture and the generated tables in
// core/SignalDb.cpp stay out of it.
static SignalDef g_def[8];
static const char* const kNames[8] = {
    "coolant", "volts", "fuel", "ambient", "tyre0", "tyre1", "tyre2", "spare",
};

static void buildDefs(void) {
  for (unsigned i = 0; i < 8u; ++i) {
    SignalDef& d = g_def[i];
    d.ecu = (uint8_t)ECU_CCN;
    d.localId = (uint16_t)(0x2100u + i);
    d.bitOffset = 0;
    d.bitLength = 8;      // an 8-bit signal: 0xFF is its "no value"
    d.render = RENDER_SCALED;
    d.scaling = -1;
    d.enumGroup = -1;
    d.name = kNames[i];
  }
}

// A bus with the engine running, so the trip meters do not veto everything.
static void running(BroadcastState& bc, uint32_t t, float kph) {
  bc.rpm = 1500u;
  bc.rpmMs = t;
  bc.speedKph = kph;
  bc.speedMs = t;
}

TEST_MAIN_BEGIN()

  buildDefs();

  // -------------------------------------------------------------------------
  // Nothing wired up at all must be safe, and must read ABSENT rather than 0
  // -------------------------------------------------------------------------
  {
    Snapshot s;
    SnapshotSource src;
    SnapshotSlots sl;
    snapshotSourceInit(src);
    snapshotSlotsInit(sl);

    BroadcastState bc;
    broadcastClear(bc);
    TripSet tr;
    tripSetInit(tr);
    AccelSet ac;
    accelInit(ac);
    FaultSet fs;
    faultsInit(fs);

    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, false, 1000u);

    CHECK_EQ(s.builtMs, 1000u);
    CHECK(!s.linkUp);
    // A null VehicleState is a legal state (nothing polled yet), and it must
    // not come back as a confident zero.
    CHECK_EQ((int)s.coolantC.age, (int)SNAP_ABSENT);
    CHECK_EQ((int)s.volts.age, (int)SNAP_ABSENT);
    CHECK_EQ((int)s.tyreBar[0].age, (int)SNAP_ABSENT);
    CHECK_EQ((int)s.speedKph.age, (int)SNAP_ABSENT);
    CHECK_EQ(s.tyreLowCount, 0);
    CHECK(!s.tyreLow[0]);
    CHECK(!snapShow(s.coolantC));
    CHECK(!snapLive(s.coolantC));
  }

  // -------------------------------------------------------------------------
  // THE THREE AGES. This is the product decision, so it gets asserted directly
  // rather than inferred from a readout.
  // -------------------------------------------------------------------------
  {
    // FRESH inside the TTL, HELD outside it, and the value SURVIVES going
    // stale -- that is the whole "hold and dim" contract. If HELD ever came
    // back with v == 0 the panel would blank a perfectly good reading.
    SnapVal a = snapshotBcast(88.5f, 1000u, 1000u, 5000u);
    CHECK_EQ((int)a.age, (int)SNAP_FRESH);
    CHECK_NEAR(a.v, 88.5f, 0.001f);

    a = snapshotBcast(88.5f, 1000u, 5999u, 5000u);
    CHECK_EQ((int)a.age, (int)SNAP_FRESH);

    a = snapshotBcast(88.5f, 1000u, 6000u, 5000u);
    CHECK_EQ((int)a.age, (int)SNAP_HELD);
    CHECK_NEAR(a.v, 88.5f, 0.001f);   // held, NOT blanked

    a = snapshotBcast(88.5f, 1000u, 9999999u, 5000u);
    CHECK_EQ((int)a.age, (int)SNAP_HELD);
    CHECK_NEAR(a.v, 88.5f, 0.001f);

    // A zeroed stamp is the module saying it has NO value. That is the only
    // thing that earns dashes, and Broadcast.cpp zeroes the stamp for an
    // explicit sentinel while HOLDING it for a dropped frame.
    a = snapshotBcast(88.5f, 0u, 1000u, 5000u);
    CHECK_EQ((int)a.age, (int)SNAP_ABSENT);
    CHECK_NEAR(a.v, 0.0f, 0.001f);    // and it does not leak the value
  }

  // -------------------------------------------------------------------------
  // MILLISECOND WRAPAROUND. nowMs truncates a 64-bit timer and rolls every
  // 49.7 days; an absolute comparison would mark everything stale for a whole
  // TTL after the roll, on a panel whose entire job is saying what it knows.
  // -------------------------------------------------------------------------
  {
    const uint32_t justBefore = 0xFFFFF000u;
    // 0x1000 ms past `justBefore` wraps to 0x00000000.
    SnapVal a = snapshotBcast(12.6f, justBefore, justBefore + 0x1000u, 5000u);
    CHECK_EQ((int)a.age, (int)SNAP_FRESH);

    a = snapshotBcast(12.6f, justBefore, justBefore + 6000u, 5000u);
    CHECK_EQ((int)a.age, (int)SNAP_HELD);

    // And the exact rollover instant is not special.
    a = snapshotBcast(12.6f, 0xFFFFFFFFu, 1000u, 5000u);
    CHECK_EQ((int)a.age, (int)SNAP_FRESH);
  }

  // -------------------------------------------------------------------------
  // A polled slot: the sentinel, the scale, and the per-signal TTL
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    const int8_t c = vs.registerSignal(&g_def[0]);
    CHECK(c >= 0);

    // Never published: ABSENT, not zero.
    CHECK_EQ((int)snapshotSlot(&vs, c, 1000u, 5000u, 1.0f).age, (int)SNAP_ABSENT);

    vs.publish(c, 88.0f, 88u, 0, 1000u);
    SnapVal a = snapshotSlot(&vs, c, 1000u, 5000u, 1.0f);
    CHECK_EQ((int)a.age, (int)SNAP_FRESH);
    CHECK_NEAR(a.v, 88.0f, 0.001f);

    // An all-ones raw against the signal's own bit width is the module saying
    // "not available". 0xFF scaled would be a plausible number, which is
    // exactly the kind that gets believed.
    vs.publish(c, 255.0f, 0xFFu, 0, 2000u);
    CHECK_EQ((int)snapshotSlot(&vs, c, 2000u, 5000u, 1.0f).age, (int)SNAP_ABSENT);

    // The scale is applied to the value and does NOT affect the sentinel test,
    // which is judged on the raw.
    vs.publish(c, 33.0f, 33u, 0, 3000u);
    a = snapshotSlot(&vs, c, 3000u, 5000u, UI_PSI_TO_BAR);
    CHECK_NEAR(a.v, 33.0f * UI_PSI_TO_BAR, 0.0001f);

    // A slot that was never registered, and a negative index, are legal
    // questions with a safe answer.
    CHECK_EQ((int)snapshotSlot(&vs, -1, 3000u, 5000u, 1.0f).age, (int)SNAP_ABSENT);
    CHECK_EQ((int)snapshotSlot(0, c, 3000u, 5000u, 1.0f).age, (int)SNAP_ABSENT);
  }

  // -------------------------------------------------------------------------
  // BROADCAST BEATS POLLED, and a HELD broadcast still beats a FRESH poll.
  //
  // The second half is the interesting one. They are two readings of the same
  // quantity, so switching source mid-glance would change the number on the
  // glass for a reason that has nothing to do with the car -- hold-and-dim is
  // there to say how old it is, and that is a better answer than a silent
  // substitution.
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    SnapshotSlots sl;
    snapshotSlotsInit(sl);
    sl.speed = vs.registerSignal(&g_def[0]);

    BroadcastState bc;
    broadcastClear(bc);
    TripSet tr;
    tripSetInit(tr);
    AccelSet ac;
    accelInit(ac);
    FaultSet fs;
    faultsInit(fs);
    Snapshot s;
    SnapshotSource src;
    snapshotSourceInit(src);

    // Polled only: that is what shows.
    vs.publish(sl.speed, 40.0f, 40u, 0, 1000u);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, &vs, true, 1000u);
    CHECK_EQ((int)s.speedKph.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.speedKph.v, 40.0f, 0.001f);

    // Broadcast arrives: it wins outright.
    bc.speedKph = 88.0f;
    bc.speedMs = 1000u;
    snapshotBuild(s, src, sl, bc, tr, ac, fs, &vs, true, 1000u);
    CHECK_NEAR(s.speedKph.v, 88.0f, 0.001f);

    // Broadcast goes STALE while the poll stays fresh. The broadcast still
    // wins, now marked HELD.
    vs.publish(sl.speed, 40.0f, 40u, 0, 20000u);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, &vs, true, 20000u);
    CHECK_EQ((int)s.speedKph.age, (int)SNAP_HELD);
    CHECK_NEAR(s.speedKph.v, 88.0f, 0.001f);

    // Only an ABSENT broadcast hands over. A zeroed stamp is the module
    // saying it has no value at all.
    bc.speedMs = 0u;
    snapshotBuild(s, src, sl, bc, tr, ac, fs, &vs, true, 20000u);
    CHECK_EQ((int)s.speedKph.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.speedKph.v, 40.0f, 0.001f);

    // Neither path has anything: ABSENT, not a confident zero.
    SnapshotSlots empty;
    snapshotSlotsInit(empty);
    snapshotBuild(s, src, empty, bc, tr, ac, fs, &vs, true, 20000u);
    CHECK_EQ((int)s.speedKph.age, (int)SNAP_ABSENT);

    // And the helper on its own.
    SnapVal absent;
    absent.v = 0.0f;
    absent.age = SNAP_ABSENT;
    SnapVal held;
    held.v = 7.0f;
    held.age = SNAP_HELD;
    SnapVal fresh;
    fresh.v = 9.0f;
    fresh.age = SNAP_FRESH;
    CHECK_NEAR(snapshotPrefer(held, fresh).v, 7.0f, 0.001f);
    CHECK_NEAR(snapshotPrefer(absent, fresh).v, 9.0f, 0.001f);
    CHECK_EQ((int)snapshotPrefer(absent, absent).age, (int)SNAP_ABSENT);
  }

  // -------------------------------------------------------------------------
  // THE COMPASS: the one readout that can only be ABSENT once.
  //
  // 0x1B4 carries no unavailable sentinel -- 0xFF appears on none of its bytes
  // in either capture -- so Broadcast.cpp never zeroes headingMs and the only
  // zero it can ever hold is the one broadcastClear() put there. ABSENT at
  // boot, HELD-or-FRESH for ever after.
  //
  // Which is exactly why the TTL is 2 s and not the generic 5. A compass ten
  // frames stale still points somewhere entirely plausible; no arithmetic
  // gives it away and no other readout contradicts it, so the dimming is the
  // only thing that can.
  // -------------------------------------------------------------------------
  {
    SnapshotSlots sl;
    snapshotSlotsInit(sl);
    BroadcastState bc;
    broadcastClear(bc);
    TripSet tr;
    tripSetInit(tr);
    AccelSet ac;
    accelInit(ac);
    FaultSet fs;
    faultsInit(fs);
    Snapshot s;
    SnapshotSource src;
    snapshotSourceInit(src);

    // Before the first frame, and this is the only moment it can happen.
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 1000u);
    CHECK_EQ((int)s.headingDeg.age, (int)SNAP_ABSENT);
    CHECK_NEAR(s.headingDeg.v, 0.0f, 0.001f);

    // Raw 89 is 178 deg, near enough due south; the module's own octant for it
    // is 4 = S.
    bc.headingOctant = 4u;
    bc.headingRaw = 89u;
    bc.headingStatus = 0x07u;
    bc.headingMs = 1000u;

    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 1000u);
    CHECK_EQ((int)s.headingDeg.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.headingDeg.v, 178.0f, 0.001f);
    // Carried through VERBATIM. Nothing here recomputes the octant from the
    // heading -- it is the module's own answer, and the rounding rule that
    // reproduces it is a measurement about this car rather than a definition.
    CHECK_EQ(s.headingOctant, 4);

    // Three missed frames is still fresh at a 500 ms period; the fourth is
    // not, and the value is HELD across that boundary rather than blanked.
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true,
                  1000u + UI_HEADING_TTL_MS - 1u);
    CHECK_EQ((int)s.headingDeg.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.headingDeg.v, 178.0f, 0.001f);

    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true,
                  1000u + UI_HEADING_TTL_MS);
    CHECK_EQ((int)s.headingDeg.age, (int)SNAP_HELD);
    CHECK_NEAR(s.headingDeg.v, 178.0f, 0.001f);
    CHECK_EQ(s.headingOctant, 4);

    // The top of the scale: 179 is the documented ceiling and scales to 358
    // deg, one count short of north.
    bc.headingOctant = 0u;
    bc.headingRaw = 179u;
    bc.headingMs = 5000u;
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 5000u);
    CHECK_EQ((int)s.headingDeg.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.headingDeg.v, 358.0f, 0.001f);
    CHECK_EQ(s.headingOctant, 0);

    // ZERO IS A REAL READING. Due north is raw 0, so a live stamp carrying a
    // zero must come back FRESH at 0.0 and showable -- not ABSENT. Same trap
    // as a zero fuel flow on overrun: the value that means "nothing" and the
    // state that means "we know nothing" are different things, and only the
    // stamp tells them apart.
    bc.headingRaw = 0u;
    bc.headingMs = 6000u;
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 6000u);
    CHECK_EQ((int)s.headingDeg.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.headingDeg.v, 0.0f, 0.001f);
    CHECK(snapShow(s.headingDeg));
  }

  // -------------------------------------------------------------------------
  // THE TANK: LITRES off the broadcast, and the ceiling that named the unit.
  //
  // 0x159 b5 is 0.5 L per count, so this readout is `bc.fuelRaw * 0.5f` and
  // nothing else -- no snapshotPrefer(), no polled fallback, because the CCN
  // 0x2103 request this replaced is gone from kPlan entirely. There is no
  // second opinion to notice a wrong answer here, which is what makes the
  // stamp semantics below carry the whole readout.
  //
  // THE EVIDENCE FOR LITRES IS A CEILING. Across every log in the repository
  // the largest valid raw ever sent is 154, which is 77.0 L against this
  // truck's 77.6 L tank. A percentage scale has no reason to stop at 154 of
  // 255; a litre scale stops exactly where the tank does. It also settles an
  // owner observation that the map's `% = raw/2.55` could not: a drive logged
  // the morning after a fill reads 138-142 raw, which is 69-71 L or about
  // 90 % -- where /2.55 insisted on 54 %.
  // -------------------------------------------------------------------------
  {
    SnapshotSlots sl;
    snapshotSlotsInit(sl);
    BroadcastState bc;
    broadcastClear(bc);
    TripSet tr;
    tripSetInit(tr);
    AccelSet ac;
    accelInit(ac);
    FaultSet fs;
    faultsInit(fs);
    Snapshot s;
    SnapshotSource src;
    snapshotSourceInit(src);

    // Before any frame at all. broadcastClear() leaves fuelMs at 0, and a zero
    // stamp is the ONLY thing that earns dashes -- so this must be ABSENT and
    // must not leak a confident 0.0 L onto a page read before setting off.
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 1000u);
    CHECK_EQ((int)s.fuelLitres.age, (int)SNAP_ABSENT);
    CHECK_NEAR(s.fuelLitres.v, 0.0f, 0.001f);
    CHECK(!snapShow(s.fuelLitres));

    // THE SCALING, first anchor: raw 140 is the morning-after-a-fill reading
    // in the 2026 drive, and it is 70.0 L.
    bc.fuelRaw = 140u;
    bc.fuelMs = 1000u;
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 1000u);
    CHECK_EQ((int)s.fuelLitres.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.fuelLitres.v, 70.0f, 0.001f);
    CHECK(snapShow(s.fuelLitres));
    CHECK(snapLive(s.fuelLitres));

    // Three missed frames at a 1 s period is still FRESH; the fourth is not,
    // and the value is HELD across that boundary rather than blanked. The TTL
    // is a property of THE SIGNAL'S OWN RATE, not of the quantity -- a tank
    // does not move in four seconds, so this dims only when the frame has
    // genuinely stopped arriving.
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true,
                  1000u + UI_FUEL_TTL_MS - 1u);
    CHECK_EQ((int)s.fuelLitres.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.fuelLitres.v, 70.0f, 0.001f);

    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true,
                  1000u + UI_FUEL_TTL_MS);
    CHECK_EQ((int)s.fuelLitres.age, (int)SNAP_HELD);
    CHECK_NEAR(s.fuelLitres.v, 70.0f, 0.001f);   // held, NOT blanked
    CHECK(snapShow(s.fuelLitres));
    CHECK(!snapLive(s.fuelLitres));

    // THE TANK-CAPACITY ANCHOR, and the reason this field is not called
    // fuelPct. Raw 154 is the largest valid value any log in the repository
    // contains, and x0.5 makes it 77.0 L against a 77.6 L (20.5 US gal) tank
    // -- the scale stopping exactly where the tank does is the whole evidence
    // that the unit is litres.
    bc.fuelRaw = 154u;
    bc.fuelMs = 5000u;
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 5000u);
    CHECK_EQ((int)s.fuelLitres.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.fuelLitres.v, 77.0f, 0.001f);

    // ZERO IS A REAL READING -- an empty tank, not an unknown one. Same trap
    // as a zero fuel flow on overrun: the value that means "nothing" and the
    // state that means "we know nothing" are different things, and only the
    // stamp tells them apart. Blanking it would hide the one tank reading a
    // driver most needs to see.
    bc.fuelRaw = 0u;
    bc.fuelMs = 6000u;
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 6000u);
    CHECK_EQ((int)s.fuelLitres.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.fuelLitres.v, 0.0f, 0.001f);
    CHECK(snapShow(s.fuelLitres));

    // AN EXPLICIT UNAVAILABLE READS ABSENT, NOT HELD -- and this half goes
    // through the real decoder rather than by hand, because what is being
    // asserted is the COUPLING: Broadcast.cpp zeroes fuelMs on a b4 the module
    // set, and snapshotBcast() has to turn that zero into dashes. Faked by
    // assigning bc.fuelMs = 0 directly, the test would pass with the gate in
    // Broadcast.cpp deleted.
    bc.fuelRaw = 140u;
    bc.fuelMs = 7000u;
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 7000u);
    CHECK_EQ((int)s.fuelLitres.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.fuelLitres.v, 70.0f, 0.001f);

    // The real key-off payload: 0x159 keeps transmitting and flips b4 to 0x01.
    CanFrame off;
    off.id = 0x159u;
    off.len = 6u;
    off.data[0] = 0xFEu;
    off.data[1] = 0xFFu;
    off.data[2] = 0xFFu;
    off.data[3] = 0xFFu;
    off.data[4] = 0x01u;
    off.data[5] = 0xFFu;
    off.data[6] = 0x00u;
    off.data[7] = 0x00u;
    CHECK(broadcastApply(bc, off, 8000u));
    CHECK_EQ(bc.fuelMs, 0);
    CHECK_EQ(bc.fuelRaw, 140);        // the decoder keeps the value...

    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 8000u);
    CHECK_EQ((int)s.fuelLitres.age, (int)SNAP_ABSENT);
    CHECK(!snapShow(s.fuelLitres));
    CHECK_NEAR(s.fuelLitres.v, 0.0f, 0.001f);   // ...and the snapshot does not
                                                // leak the 70.0 it still holds

    // ...and one valid frame puts it back. Dashes are a state this readout can
    // leave again, not a latch: the key comes back on and the tank returns.
    off.data[0] = 0x05u;
    off.data[4] = 0x00u;
    off.data[5] = 0x4Du;              // raw 77, about half a tank
    CHECK(broadcastApply(bc, off, 9000u));
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 9000u);
    CHECK_EQ((int)s.fuelLitres.age, (int)SNAP_FRESH);
    CHECK_NEAR(s.fuelLitres.v, 38.5f, 0.001f);
  }

  // -------------------------------------------------------------------------
  // TYRES: psi -> bar, and the low-corner test.
  //
  // The TTL here is the one that MUST track kPlan. Tyres are polled once a
  // minute, so a corner 90 s old is as fresh as it will ever be and must not
  // read stale -- that was a real defect when the poll rate moved and the TTL
  // did not.
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    SnapshotSlots sl;
    snapshotSlotsInit(sl);
    sl.tyre[0] = vs.registerSignal(&g_def[4]);
    sl.tyre[1] = vs.registerSignal(&g_def[5]);
    sl.tyre[2] = vs.registerSignal(&g_def[6]);

    BroadcastState bc;
    broadcastClear(bc);
    TripSet tr;
    tripSetInit(tr);
    AccelSet ac;
    accelInit(ac);
    FaultSet fs;
    faultsInit(fs);
    Snapshot s;
    SnapshotSource src;
    snapshotSourceInit(src);

    // 34 psi = 2.34 bar (fine), 28 psi = 1.93 bar (soft against a 2.1 floor).
    vs.publish(sl.tyre[0], 34.0f, 34u, 0, 1000u);
    vs.publish(sl.tyre[1], 28.0f, 28u, 0, 1000u);
    // tyre[2] registered but never published.

    snapshotBuild(s, src, sl, bc, tr, ac, fs, &vs, true, 1000u);
    CHECK_NEAR(s.tyreBar[0].v, 2.3442f, 0.001f);
    CHECK_NEAR(s.tyreBar[1].v, 1.9305f, 0.001f);
    CHECK(!s.tyreLow[0]);
    CHECK(s.tyreLow[1]);
    // An ABSENT corner is NOT a warning: we do not know it is low, we know we
    // cannot see it. Counting it would put a number on the page that no
    // reading supports.
    CHECK(!s.tyreLow[2]);
    CHECK(!s.tyreLow[3]);
    CHECK_EQ(s.tyreLowCount, 1);

    // 90 s on: past the 5 s default, well inside the tyres' own 120 s.
    snapshotBuild(s, src, sl, bc, tr, ac, fs, &vs, true, 91000u);
    CHECK_EQ((int)s.tyreBar[0].age, (int)SNAP_FRESH);
    CHECK(s.tyreLow[1]);
    CHECK_EQ(s.tyreLowCount, 1);

    // Past two minutes it is held and dimmed -- still a warning, visibly not a
    // live one.
    snapshotBuild(s, src, sl, bc, tr, ac, fs, &vs, true, 1000u + UI_TYRE_TTL_MS + 1u);
    CHECK_EQ((int)s.tyreBar[0].age, (int)SNAP_HELD);
    CHECK_EQ((int)s.tyreBar[1].age, (int)SNAP_HELD);
    CHECK(s.tyreLow[1]);        // still low
    CHECK_EQ(s.tyreLowCount, 1);
  }

  // -------------------------------------------------------------------------
  // FLOW: the unit switch, its hysteresis, and the clamp.
  //
  // The hysteresis is the point. One threshold makes the readout flip units at
  // walking pace in traffic, which is unreadable -- so between 5 and 15 km/h
  // it must STAY in whichever unit it arrived in.
  // -------------------------------------------------------------------------
  {
    SnapshotSlots sl;
    snapshotSlotsInit(sl);
    BroadcastState bc;
    broadcastClear(bc);
    AccelSet ac;
    accelInit(ac);
    FaultSet fs;
    faultsInit(fs);
    Snapshot s;
    SnapshotSource src;
    snapshotSourceInit(src);

    // Feed a steady 10 L/h (Q = 1600) so tripFlowLph has something smoothed.
    TripSet tr;
    tripSetInit(tr);
    uint32_t t = 1000u;
    for (; t <= 20000u; t += 100u) {
      running(bc, t, 0.0f);
      bc.odometerDm = 10000u;
      bc.odometerMs = t;
      bc.flowRaw = 1600u;
      bc.flowMs = t;
      tripUpdate(tr, bc, t);
    }

    // Stationary: per HOUR. There is no distance to divide by, and that is not
    // a fallback -- L/100 from a standstill would be invented.
    running(bc, t, 0.0f);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, t);
    CHECK(!s.flowPerDistance);
    CHECK(!s.flowCrit);
    CHECK(snapShow(s.flow));
    CHECK_NEAR(s.flow.v, 10.0f, 0.5f);

    // 2.5 km/h, inside the dead band, arriving from BELOW: stays per-hour.
    running(bc, t, 2.5f);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, t);
    CHECK(!s.flowPerDistance);

    // Past 3 km/h: switches to per-distance.
    running(bc, t, 20.0f);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, t);
    CHECK(s.flowPerDistance);
    CHECK_NEAR(s.flow.v, 10.0f * 100.0f / 20.0f, 0.5f);

    // Back into the dead band from ABOVE: STAYS per-distance. Same 2.5 km/h as
    // four lines up, opposite answer -- that is the hysteresis, and it is why
    // this state lives in SnapshotSource rather than being recomputed.
    running(bc, t, 2.5f);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, t);
    CHECK(s.flowPerDistance);

    // Below 2 km/h: back to per-hour.
    running(bc, t, 1.0f);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, t);
    CHECK(!s.flowPerDistance);

    // A STALE speed must not be divided by. Hold-and-dim applies to the
    // divisor too: 90 km/h from a minute ago is not a distance we are still
    // covering.
    running(bc, t, 90.0f);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, t);
    CHECK(s.flowPerDistance);
    bc.speedMs = t - (VS_DEFAULT_TTL_MS + 1000u);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, t);
    CHECK(!s.flowPerDistance);
  }

  // -------------------------------------------------------------------------
  // THE BAR IS SCALED IN L/100, and it is a CLAMP rather than an auto-range.
  //
  // This suite used to assert the bar against an L/h scale, which is the bug
  // the owner saw as a jumping scale: the bar and the readout were on two
  // different quantities. So the interesting assertions here are that the
  // segments track THE NUMBER SHOWN, that they stop at sixteen, and that they
  // are zero when there is no per-distance figure at all.
  // -------------------------------------------------------------------------
  {
    SnapshotSlots sl;
    snapshotSlotsInit(sl);
    BroadcastState bc;
    broadcastClear(bc);
    AccelSet ac;
    accelInit(ac);
    FaultSet fs;
    faultsInit(fs);
    Snapshot s;
    SnapshotSource src;
    snapshotSourceInit(src);
    TripSet tr;
    tripSetInit(tr);

    // Q = 160 per L/h, so 80 L/h is Q = 12800 -- well past full scale.
    uint32_t t = 1000u;
    for (; t <= 30000u; t += 100u) {
      running(bc, t, 0.0f);
      bc.odometerDm = 10000u;
      bc.odometerMs = t;
      bc.flowRaw = 12800u;
      bc.flowMs = t;
      tripUpdate(tr, bc, t);
    }
    // STOPPED: no per-distance figure exists, so no bar. The readout falls to
    // L/h and says the real thing; the bar says nothing rather than drawing an
    // L/h value against an L/100 scale.
    running(bc, t, 0.0f);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, t);
    CHECK(!s.flowPerDistance);
    CHECK_EQ(s.flowSegs, 0);
    CHECK(snapShow(s.flow));

    // MOVING at 20 km/h on 80 L/h is 400 L/100 -- far past full scale. The bar
    // pegs at sixteen and the NUMBER keeps telling the truth past the end of
    // it.
    running(bc, t, 20.0f);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, t);
    CHECK(s.flowPerDistance);
    CHECK_EQ(s.flowSegs, UI_FLOW_SEGS);
    CHECK(s.flow.v > UI_FLOW_FULL_L100);
    CHECK(s.flowCrit);

    // And the bar tracks the SHOWN number, segment for segment: 80 L/h at
    // 320 km/h is 25 L/100, which is half of 50, which is eight of sixteen.
    running(bc, t, 320.0f);
    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, t);
    CHECK_NEAR(s.flow.v, 25.0f, 0.6f);
    CHECK_EQ(s.flowSegs, 8);
    // 25 is under the 35 critical line, so the bar is not red at half full.
    CHECK(!s.flowCrit);
  }

  // -------------------------------------------------------------------------
  // The wholesale copies are a COPY. This is the bug the snapshot exists to
  // kill: the fault list is insertion-sorted at end-of-scan while the
  // drill-down walks it, so a live pointer could show a row twice or not at
  // all. A copy cannot.
  // -------------------------------------------------------------------------
  {
    SnapshotSlots sl;
    snapshotSlotsInit(sl);
    BroadcastState bc;
    broadcastClear(bc);
    TripSet tr;
    tripSetInit(tr);
    AccelSet ac;
    accelInit(ac);
    Snapshot s;
    SnapshotSource src;
    snapshotSourceInit(src);

    FaultSet fs;
    faultsInit(fs);
    DtcRecord r[2];
    r[0].code = 0x0119u;
    r[0].status = 0x60u;
    r[1].code = 0x0122u;
    r[1].status = 0x60u;
    faultsBeginScan(fs, 2u);
    faultsRecord(fs, 0u, r, 2u, false);
    faultsEndScan(fs, 1000u);
    CHECK_EQ(fs.count, 2);

    snapshotBuild(s, src, sl, bc, tr, ac, fs, 0, true, 1000u);
    CHECK_EQ(s.faults.count, 2);

    // Mutating the LIVE set after the build must not reach the snapshot.
    faultsInit(fs);
    CHECK_EQ(fs.count, 0);
    CHECK_EQ(s.faults.count, 2);
    CHECK_EQ(s.faults.f[0].code, 0x0119u);
  }

TEST_MAIN_END()
