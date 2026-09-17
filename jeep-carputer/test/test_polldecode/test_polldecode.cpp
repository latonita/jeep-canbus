// Host tests for core/PollDecode.cpp.
//
// This code was inside `pollOne()` in main.cpp until it moved, and the reason
// it was worth moving is the reason this suite is short but not trivial: the
// fan-out walks TWO signal tables, and the second one exists only because the
// first cannot contain it.
//
// `kSignals` is generated from the vendor database. `kTpmsSignals` is
// hand-written, because the WCM's per-wheel pressure PID is undocumented and
// a generator can only emit what its source holds. A fan-out that walked one
// and not the other would take out TPMS -- the hero feature of this
// instrument -- and take it out SILENTLY: the request would succeed, the
// response would parse, the poll would report success, and the tyre page would
// simply stay dashed forever. That is one `for` loop away at all times, and
// it is what the first block below pins down.
#include "doctest_min.h"

#include "PollDecode.h"
#include "TpmsSignals.h"

// `21 3B` answers `61 3B <p1> <p2> <p3> <p4>` -- two bytes of echo, then one
// psi per bit per road wheel. Build that reply.
static void tpmsPdu(IsoTpPdu& pdu, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {
  pdu.data[0] = 0x61u;
  pdu.data[1] = 0x3Bu;
  pdu.data[2] = p1;
  pdu.data[3] = p2;
  pdu.data[4] = p3;
  pdu.data[5] = p4;
  pdu.len = 6u;
}

// Every signal the sink was handed, so a test can assert on what was WALKED
// rather than only on what was published.
static uint8_t g_seen = 0u;
static uint8_t g_seenOk = 0u;
static const char* g_lastName = 0;

static void sink(const SignalDef& def, const DecodedValue& v, bool ok) {
  (void)v;
  ++g_seen;
  if (ok) {
    ++g_seenOk;
  }
  g_lastName = def.name;
}

static void sinkReset(void) {
  g_seen = 0u;
  g_seenOk = 0u;
  g_lastName = 0;
}

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // THE HAND-WRITTEN TABLE IS REACHED. This is the whole point of the module.
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    // Register all four road wheels, exactly as main.cpp's kPlan walk does.
    int8_t slot[4];
    for (uint8_t i = 0u; i < 4u; ++i) {
      slot[i] = vs.registerSignal(&kTpmsSignals[i]);
      CHECK(slot[i] >= 0);
    }

    IsoTpPdu pdu;
    tpmsPdu(pdu, 33u, 28u, 34u, 30u);
    sinkReset();

    const uint8_t n = pollDecodeResponse(ECU_WCM, TPMS_ROAD_WHEELS_PID, pdu, vs,
                                         1000u, sink);
    // FOUR from one response. A fan-out that returned after the first match
    // would answer 1 here and leave three corners dashed -- and would still
    // report the poll a success.
    CHECK_EQ(n, 4);
    CHECK_EQ(g_seen, 4);
    CHECK_EQ(g_seenOk, 4);

    // Raw psi, not bar: the value published is the value the bus sent, and the
    // conversion is the presenter's job (core/Snapshot.h).
    CHECK_NEAR(vs.slot(slot[0])->value, 33.0f, 0.001f);
    CHECK_NEAR(vs.slot(slot[1])->value, 28.0f, 0.001f);
    CHECK_NEAR(vs.slot(slot[2])->value, 34.0f, 0.001f);
    CHECK_NEAR(vs.slot(slot[3])->value, 30.0f, 0.001f);
    // And the stamp is the one it was told, not one it read from a clock.
    CHECK_EQ(vs.slot(slot[0])->stampMs, 1000u);
    CHECK(vs.slot(slot[0])->everSet);
  }

  // -------------------------------------------------------------------------
  // bitOffset counts from byte 0 of the FULL response, echo included.
  //
  // TpmsSignals.cpp warns about exactly this: get it wrong and the echo reads
  // as a pressure and every corner is silently two bytes out. A test that only
  // checked "four values arrived" would pass through that bug, so this one
  // checks the ECHO cannot be mistaken for data.
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    const int8_t s0 = vs.registerSignal(&kTpmsSignals[0]);
    IsoTpPdu pdu;
    // Echo bytes deliberately set to plausible pressures.
    tpmsPdu(pdu, 33u, 28u, 34u, 30u);
    pdu.data[0] = 0x61u;
    pdu.data[1] = 0x3Bu;
    sinkReset();
    CHECK_EQ(pollDecodeResponse(ECU_WCM, TPMS_ROAD_WHEELS_PID, pdu, vs, 1u, sink), 1);
    // 33, the first real pressure -- not 0x61 (97) and not 0x3B (59).
    CHECK_NEAR(vs.slot(s0)->value, 33.0f, 0.001f);
  }

  // -------------------------------------------------------------------------
  // The spare comes from its OWN PID and must not answer to the corners'.
  // `21 3B` carries road wheels only, deliberately.
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    const int8_t sp = vs.registerSignal(&kTpmsSignals[4]);   // WCM.Tyre Spare
    CHECK(sp >= 0);

    IsoTpPdu pdu;
    tpmsPdu(pdu, 33u, 28u, 34u, 30u);
    sinkReset();
    // Asking for the road wheels must publish NOTHING for the spare.
    CHECK_EQ(pollDecodeResponse(ECU_WCM, TPMS_ROAD_WHEELS_PID, pdu, vs, 1u, sink), 0);
    CHECK_EQ(g_seen, 0);
    CHECK(!vs.slot(sp)->everSet);

    // Its own PID does.
    pdu.data[1] = 0x24u;
    pdu.data[2] = 7u;
    pdu.len = 6u;
    CHECK_EQ(pollDecodeResponse(ECU_WCM, TPMS_SPARE_PID, pdu, vs, 2u, sink), 1);
    CHECK_NEAR(vs.slot(sp)->value, 7.0f, 0.001f);
  }

  // -------------------------------------------------------------------------
  // A signal that is DEFINED but not REGISTERED is skipped, not an error.
  //
  // Normal condition, not a fault: the tables describe every signal this
  // vehicle has and kPlan asks for a handful. Counting these as errors would
  // make the HEALTH page's error counters meaningless.
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    const int8_t s0 = vs.registerSignal(&kTpmsSignals[0]);   // ONLY sensor 1
    CHECK(s0 >= 0);
    IsoTpPdu pdu;
    tpmsPdu(pdu, 33u, 28u, 34u, 30u);
    sinkReset();
    CHECK_EQ(pollDecodeResponse(ECU_WCM, TPMS_ROAD_WHEELS_PID, pdu, vs, 1u, sink), 1);
    // The sink sees only the registered one -- an unregistered signal is not
    // walked at all, so nothing can be logged about it either.
    CHECK_EQ(g_seen, 1);
    CHECK_EQ(vs.slot(s0)->errors, 0);
  }

  // -------------------------------------------------------------------------
  // A response too SHORT to carry the signal is an error, counted, not a
  // silent skip -- a definition that disagrees with a module is the
  // interesting kind of broken.
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    int8_t slot[4];
    for (uint8_t i = 0u; i < 4u; ++i) {
      slot[i] = vs.registerSignal(&kTpmsSignals[i]);
    }
    IsoTpPdu pdu;
    tpmsPdu(pdu, 33u, 28u, 34u, 30u);
    pdu.len = 4u;      // echo + two pressures only
    sinkReset();

    const uint8_t n = pollDecodeResponse(ECU_WCM, TPMS_ROAD_WHEELS_PID, pdu, vs,
                                         1u, sink);
    CHECK_EQ(n, 2);
    // All four were WALKED; two of them failed.
    CHECK_EQ(g_seen, 4);
    CHECK_EQ(g_seenOk, 2);
    CHECK_EQ(vs.slot(slot[0])->errors, 0);
    CHECK_EQ(vs.slot(slot[2])->errors, 1);
    CHECK_EQ(vs.slot(slot[3])->errors, 1);
    // And a failed decode must not leave a value behind to be believed.
    CHECK(!vs.slot(slot[2])->everSet);
  }

  // -------------------------------------------------------------------------
  // Degenerate inputs answer safely.
  // -------------------------------------------------------------------------
  {
    VehicleState vs;
    for (uint8_t i = 0u; i < 4u; ++i) {
      (void)vs.registerSignal(&kTpmsSignals[i]);
    }
    IsoTpPdu pdu;
    tpmsPdu(pdu, 33u, 28u, 34u, 30u);

    // A PID nothing is defined against.
    sinkReset();
    CHECK_EQ(pollDecodeResponse(ECU_WCM, 0x21FEu, pdu, vs, 1u, sink), 0);
    CHECK_EQ(g_seen, 0);

    // Right PID, wrong module. The (ecu, localId) pair is the key, and a PID
    // number is only unique WITHIN a module -- matching on the identifier
    // alone would cross-publish between ECUs.
    sinkReset();
    CHECK_EQ(pollDecodeResponse(ECU_CCN, TPMS_ROAD_WHEELS_PID, pdu, vs, 1u, sink), 0);
    CHECK_EQ(g_seen, 0);

    // A null sink is legal -- the decode is the job, the logging is not.
    CHECK_EQ(pollDecodeResponse(ECU_WCM, TPMS_ROAD_WHEELS_PID, pdu, vs, 1u, 0), 4);

    // An empty response.
    pdu.len = 0u;
    sinkReset();
    CHECK_EQ(pollDecodeResponse(ECU_WCM, TPMS_ROAD_WHEELS_PID, pdu, vs, 1u, sink), 0);
    CHECK_EQ(g_seen, 4);        // walked
    CHECK_EQ(g_seenOk, 0);      // none decoded
  }

  // -------------------------------------------------------------------------
  // The GENERATED table is reached too, by the same walk.
  //
  // Picked out of kSignals at runtime rather than hard-coded, so this keeps
  // working when the generator is re-run and the table shifts.
  // -------------------------------------------------------------------------
  {
    CHECK(kSignalCount > 0u);
    const SignalDef* pick = 0;
    for (uint16_t i = 0u; i < kSignalCount; ++i) {
      // A plain scaled 8-bit signal starting at the usual echo offset is the
      // easiest to build a believable response for.
      if (kSignals[i].bitLength == 8u && kSignals[i].bitOffset == 16u) {
        pick = &kSignals[i];
        break;
      }
    }
    CHECK(pick != 0);
    if (pick != 0) {
      VehicleState vs;
      const int8_t s = vs.registerSignal(pick);
      CHECK(s >= 0);
      IsoTpPdu pdu;
      pdu.data[0] = (uint8_t)(0x40u + (pick->localId >> 8));
      pdu.data[1] = (uint8_t)(pick->localId & 0xFFu);
      pdu.data[2] = 0x40u;
      pdu.len = 3u;
      sinkReset();
      const uint8_t n = pollDecodeResponse(pick->ecu, pick->localId, pdu, vs, 5u, sink);
      // At least this one; the PID may legitimately carry siblings.
      CHECK(n >= 1u);
      CHECK(g_seen >= 1u);
      CHECK(vs.slot(s)->everSet);
    }
  }

TEST_MAIN_END()
