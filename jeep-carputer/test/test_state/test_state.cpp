// Host tests for core/VehicleState.cpp.
//
// Every SignalDef here is local to this file; the generated tables in
// core/SignalDb.cpp are not consulted. VehicleState keys purely on the
// SignalDef pointer, so plain local structs are exactly the right fixture.
#include "doctest_min.h"

#include "VehicleState.h"

// A pool big enough to overflow the registry (VS_MAX_SLOTS + 2).
#define POOL_N (VS_MAX_SLOTS + 2u)
static SignalDef g_pool[POOL_N];
static char g_names[POOL_N][12];

static void buildPool() {
  for (unsigned i = 0; i < POOL_N; ++i) {
    g_names[i][0] = 's';
    g_names[i][1] = 'i';
    g_names[i][2] = 'g';
    g_names[i][3] = (char)('0' + (i / 10u) % 10u);
    g_names[i][4] = (char)('0' + i % 10u);
    g_names[i][5] = '\0';

    SignalDef& d = g_pool[i];
    d.ecu = (uint8_t)ECU_CCN;
    d.localId = (uint16_t)(0x2100u + i);
    d.bitOffset = 16;
    d.bitLength = 16;
    d.render = RENDER_SCALED;
    d.scaling = -1;
    d.enumGroup = -1;
    d.name = g_names[i];
  }
}

TEST_MAIN_BEGIN()

  buildPool();

  // -------------------------------------------------------------------------
  // A fresh registry answers safely to everything
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    CHECK_EQ(vs.count(), 0);
    CHECK(vs.slot(0) == 0);
    CHECK(vs.slot(-1) == 0);
    CHECK(vs.slot(127) == 0);
    CHECK_EQ(vs.indexOf(&g_pool[0]), -1);
    CHECK_EQ(vs.indexOf(0), -1);
    CHECK_EQ(vs.indexOfName("sig00"), -1);
    CHECK_EQ(vs.indexOfName(0), -1);
    CHECK(!vs.fresh(0, 0));
    CHECK(!vs.fresh(-1, 0));
    // Must not crash and must not disturb count_.
    vs.publish(0, 1.0f, 1, "x", 100);
    vs.publish(-1, 1.0f, 1, "x", 100);
    vs.noteError(0);
    vs.noteError(-1);
    CHECK_EQ(vs.count(), 0);
  }

  // -------------------------------------------------------------------------
  // registerSignal
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    CHECK_EQ(vs.registerSignal(&g_pool[0]), 0);
    CHECK_EQ(vs.registerSignal(&g_pool[1]), 1);
    CHECK_EQ(vs.registerSignal(&g_pool[2]), 2);
    CHECK_EQ(vs.count(), 3);

    CHECK_EQ(vs.registerSignal(&g_pool[1]), -1);   // already present
    CHECK_EQ(vs.registerSignal(0), -1);            // NULL def
    CHECK_EQ(vs.count(), 3);                       // neither consumed a slot

    CHECK_EQ(vs.indexOf(&g_pool[0]), 0);
    CHECK_EQ(vs.indexOf(&g_pool[2]), 2);
    CHECK_EQ(vs.indexOf(&g_pool[3]), -1);

    CHECK_EQ(vs.indexOfName("sig00"), 0);
    CHECK_EQ(vs.indexOfName("sig02"), 2);
    CHECK_EQ(vs.indexOfName("sig03"), -1);
    CHECK_EQ(vs.indexOfName(""), -1);

    const StateSlot* s = vs.slot(1);
    CHECK(s != 0);
    CHECK(s->def == &g_pool[1]);
    CHECK(!s->everSet);
    CHECK_EQ(s->updates, 0);
    CHECK_EQ(s->errors, 0);
    CHECK(s->label == 0);
    CHECK(vs.slot(3) == 0);                        // registered range only
  }

  // -------------------------------------------------------------------------
  // Registry full
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    for (unsigned i = 0; i < VS_MAX_SLOTS; ++i) {
      CHECK_EQ(vs.registerSignal(&g_pool[i]), (int)i);
    }
    CHECK_EQ(vs.count(), (int)VS_MAX_SLOTS);
    CHECK_EQ(vs.registerSignal(&g_pool[VS_MAX_SLOTS]), -1);
    CHECK_EQ(vs.registerSignal(&g_pool[VS_MAX_SLOTS + 1u]), -1);
    CHECK_EQ(vs.count(), (int)VS_MAX_SLOTS);
    // The last accepted slot is still readable, the rejected ones are not.
    CHECK(vs.slot((int8_t)(VS_MAX_SLOTS - 1u)) != 0);
    CHECK(vs.slot((int8_t)VS_MAX_SLOTS) == 0);
  }

  // -------------------------------------------------------------------------
  // publish
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    const int8_t rpm = vs.registerSignal(&g_pool[0]);
    const int8_t gear = vs.registerSignal(&g_pool[1]);
    CHECK_EQ(rpm, 0);
    CHECK_EQ(gear, 1);

    vs.publish(rpm, 812.5f, 3250, 0, 1000);
    const StateSlot* s = vs.slot(rpm);
    CHECK(s != 0);
    CHECK_NEAR(s->value, 812.5, 0.001);
    CHECK_EQ(s->raw, 3250);
    CHECK_EQ(s->stampMs, 1000);
    CHECK(s->label == 0);
    CHECK(s->everSet);
    CHECK_EQ(s->updates, 1);
    CHECK_EQ(s->errors, 0);

    vs.publish(gear, 4.0f, 4, "D4", 1010);
    const StateSlot* g = vs.slot(gear);
    CHECK_STR(g->label, "D4");
    CHECK_EQ(g->stampMs, 1010);
    // Publishing one slot leaves the other alone.
    CHECK_EQ(vs.slot(rpm)->stampMs, 1000);

    vs.publish(rpm, 900.0f, 3600, 0, 1100);
    CHECK_EQ(vs.slot(rpm)->updates, 2);
    CHECK_NEAR(vs.slot(rpm)->value, 900.0, 0.001);

    // Out-of-range publishes are dropped, not written anywhere.
    vs.publish(2, 1.0f, 1, "x", 9999);       // == count_, never registered
    vs.publish(47, 1.0f, 1, "x", 9999);
    vs.publish(-5, 1.0f, 1, "x", 9999);
    CHECK_EQ(vs.count(), 2);
    CHECK_EQ(vs.slot(rpm)->stampMs, 1100);
    CHECK_EQ(vs.slot(gear)->stampMs, 1010);
  }

  // -------------------------------------------------------------------------
  // noteError keeps the last good value
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    const int8_t i = vs.registerSignal(&g_pool[0]);
    vs.publish(i, 812.5f, 3250, "lbl", 1000);
    vs.noteError(i);
    const StateSlot* s = vs.slot(i);
    CHECK_EQ(s->errors, 1);
    CHECK_EQ(s->updates, 1);          // untouched
    CHECK_NEAR(s->value, 812.5, 0.001);
    CHECK_EQ(s->raw, 3250);
    CHECK_EQ(s->stampMs, 1000);       // untouched, so it can still go stale
    CHECK_STR(s->label, "lbl");
    CHECK(s->everSet);

    vs.noteError(i);
    CHECK_EQ(vs.slot(i)->errors, 2);
    // A dropped poll does not make the value un-fresh by itself; only time
    // does. A presenter keeps showing 812.5 and flags it when fresh() lapses.
    CHECK(vs.fresh(i, 1200));
    CHECK(!vs.fresh(i, 1200 + VS_DEFAULT_TTL_MS));

    // An error on a slot that was never published still leaves it not-fresh.
    const int8_t j = vs.registerSignal(&g_pool[1]);
    vs.noteError(j);
    CHECK_EQ(vs.slot(j)->errors, 1);
    CHECK(!vs.fresh(j, 0));
    CHECK(!vs.slot(j)->everSet);

    // Unregistered indices.
    vs.noteError(2);
    vs.noteError(-1);
    vs.noteError(100);
    CHECK_EQ(vs.count(), 2);
  }

  // -------------------------------------------------------------------------
  // fresh(): TTL boundary
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    const int8_t i = vs.registerSignal(&g_pool[0]);
    CHECK(!vs.fresh(i, 0));                    // never set
    CHECK(!vs.fresh(i, 1000000));

    vs.publish(i, 1.0f, 1, 0, 1000);
    CHECK(vs.fresh(i, 1000));                  // age 0
    CHECK(vs.fresh(i, 1000 + 4999));           // age 4999 < 5000
    CHECK(!vs.fresh(i, 1000 + 5000));          // age == ttl is stale
    CHECK(!vs.fresh(i, 1000 + 5001));

    CHECK(vs.fresh(i, 1500, 1000));            // explicit ttl
    CHECK(!vs.fresh(i, 2000, 1000));
    CHECK(!vs.fresh(i, 1000, 0));              // ttl 0 is never fresh

    // Out-of-range index is false, not an array access.
    CHECK(!vs.fresh(1, 1000));
    CHECK(!vs.fresh(-1, 1000));
    CHECK(!vs.fresh(120, 1000));
  }

  // -------------------------------------------------------------------------
  // fresh(): millis() rollover. Timestamps are uint32_t and wrap after ~49
  // days; the age must be computed as (now - stamp) in unsigned arithmetic.
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    const int8_t i = vs.registerSignal(&g_pool[0]);

    // Published just before the wrap.
    const uint32_t stamp = 0xFFFFFF00u;   // 256 ms before 2^32
    vs.publish(i, 812.5f, 3250, 0, stamp);
    CHECK_EQ(vs.slot(i)->stampMs, (long long)stamp);

    CHECK(vs.fresh(i, stamp));                     // age 0
    CHECK(vs.fresh(i, 0xFFFFFFFFu));               // age 255, still before wrap
    CHECK(vs.fresh(i, 0x00000000u));               // age 256, wrapped
    CHECK(vs.fresh(i, 0x00000040u));               // age 320
    CHECK(vs.fresh(i, (uint32_t)(stamp + 4999u))); // age 4999 -> 0x0000138F
    CHECK(!vs.fresh(i, (uint32_t)(stamp + 5000u)));
    CHECK(!vs.fresh(i, (uint32_t)(stamp + 60000u)));

    // The naive `stamp + ttl` comparison overflows here (0xFFFFFF00 + 5000
    // wraps to 0x1287) and would call every one of these fresh. Ages are
    // 256 + now, counted across the wrap:
    CHECK(vs.fresh(i, 0x00001000u));               // age 256 + 4096 = 4352
    CHECK(!vs.fresh(i, 0x000012FFu));              // age 256 + 4863 = 5119
    CHECK(!vs.fresh(i, 0x00001500u));              // age 256 + 5376 = 5632

    // Rollover the other way round: a clock that jumps backwards reads as a
    // huge age, i.e. stale. Erring towards stale is the safe direction.
    vs.publish(i, 1.0f, 1, 0, 10000);
    CHECK(vs.fresh(i, 10000));
    CHECK(!vs.fresh(i, 9000));
  }

  // -------------------------------------------------------------------------
  // Counters saturate instead of wrapping back through 0
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    const int8_t i = vs.registerSignal(&g_pool[0]);
    for (unsigned long n = 0; n < 70000ul; ++n) {
      vs.publish(i, 1.0f, 1, 0, 1000);
      vs.noteError(i);
    }
    CHECK_EQ(vs.slot(i)->updates, 0xFFFF);
    CHECK_EQ(vs.slot(i)->errors, 0xFFFF);
    CHECK(vs.slot(i)->everSet);
  }

TEST_MAIN_END()
