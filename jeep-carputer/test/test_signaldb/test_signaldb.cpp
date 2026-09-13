// Host tests for the generated tables in core/SignalDb.cpp.
//
// These are structural tests plus a handful of hand-typed spot checks.  The
// point of the spot checks is that they were copied out of the vendor CSVs by
// hand, so a regeneration that silently moves a bit offset or changes a scale
// factor fails here instead of producing a plausible-but-wrong reading in the
// car.  If one of them fails, go and look at the CSV before "fixing" the test.
#include "SignalDb.h"
#include "doctest_min.h"

namespace {

// Scan kEnums for the entry covering `raw` in `group`.  Mirrors what the
// decoder has to do -- SignalDef::enumGroup is a group tag, not an index.
const EnumEntry* findEnum(int16_t group, uint32_t raw) {
  for (uint16_t i = 0; i < kEnumCount; ++i) {
    if ((int16_t)kEnums[i].group == group &&
        raw >= kEnums[i].from && raw <= kEnums[i].to) {
      return &kEnums[i];
    }
  }
  return NULL;
}

bool enumGroupExists(int16_t group) {
  for (uint16_t i = 0; i < kEnumCount; ++i) {
    if ((int16_t)kEnums[i].group == group) return true;
  }
  return false;
}

}  // namespace

TEST_MAIN_BEGIN()

  // -------------------------------------------------------------------------
  // kEcus: ids and bus assignment.  The request/response pairs are the ones
  // documented in SignalDb.h and observed in ecus.csv.
  // -------------------------------------------------------------------------
  CHECK_EQ(ECU_COUNT, 12);

  CHECK_EQ(kEcus[ECU_CCN].requestId,     0x6A0);
  CHECK_EQ(kEcus[ECU_CCN].responseId,    0x514);
  CHECK_EQ(kEcus[ECU_FCMCGW].requestId,  0x620);
  CHECK_EQ(kEcus[ECU_FCMCGW].responseId, 0x504);
  CHECK_EQ(kEcus[ECU_RADIO].requestId,   0x6B0);
  CHECK_EQ(kEcus[ECU_RADIO].responseId,  0x516);
  CHECK_EQ(kEcus[ECU_ORC].requestId,     0x6E0);
  CHECK_EQ(kEcus[ECU_ORC].responseId,    0x51C);
  CHECK_EQ(kEcus[ECU_WCM].requestId,     0x600);
  CHECK_EQ(kEcus[ECU_WCM].responseId,    0x500);
  CHECK_EQ(kEcus[ECU_SCM].requestId,     0x6A8);
  CHECK_EQ(kEcus[ECU_SCM].responseId,    0x515);
  CHECK_EQ(kEcus[ECU_EOM].requestId,     0x6F0);
  CHECK_EQ(kEcus[ECU_EOM].responseId,    0x51E);
  CHECK_EQ(kEcus[ECU_PCM].requestId,     0x7E0);
  CHECK_EQ(kEcus[ECU_PCM].responseId,    0x7E8);
  CHECK_EQ(kEcus[ECU_TCM].requestId,     0x7E1);
  CHECK_EQ(kEcus[ECU_TCM].responseId,    0x7E9);
  CHECK_EQ(kEcus[ECU_ABS].requestId,     0x784);
  CHECK_EQ(kEcus[ECU_ABS].responseId,    0x785);
  CHECK_EQ(kEcus[ECU_FDCM].requestId,    0x7B6);
  CHECK_EQ(kEcus[ECU_FDCM].responseId,   0x7B7);
  CHECK_EQ(kEcus[ECU_ESM].requestId,     0x788);
  CHECK_EQ(kEcus[ECU_ESM].responseId,    0x789);

  CHECK_STR(kEcus[ECU_CCN].name,    "CCN");
  CHECK_STR(kEcus[ECU_FCMCGW].name, "FCMCGW");
  CHECK_STR(kEcus[ECU_TCM].name,    "TCM");
  CHECK_STR(kEcus[ECU_ABS].name,    "ABS");
  CHECK_STR(kEcus[ECU_FDCM].name,   "FDCM");
  CHECK_STR(kEcus[ECU_ESM].name,    "ESM");

  // Interior CAN-B, and the five CAN-C strangers.
  CHECK(kEcus[ECU_CCN].interiorBus);
  CHECK(kEcus[ECU_FCMCGW].interiorBus);
  CHECK(kEcus[ECU_RADIO].interiorBus);
  CHECK(kEcus[ECU_ORC].interiorBus);
  CHECK(kEcus[ECU_WCM].interiorBus);
  CHECK(kEcus[ECU_SCM].interiorBus);
  CHECK(kEcus[ECU_EOM].interiorBus);
  CHECK(!kEcus[ECU_PCM].interiorBus);
  CHECK(!kEcus[ECU_TCM].interiorBus);
  CHECK(!kEcus[ECU_ABS].interiorBus);
  CHECK(!kEcus[ECU_FDCM].interiorBus);
  CHECK(!kEcus[ECU_ESM].interiorBus);

  for (int i = 0; i < ECU_COUNT; ++i) {
    CHECK_MSG(kEcus[i].name != NULL && kEcus[i].name[0] != '\0',
              "kEcus[%d].name is empty", i);
    CHECK_MSG(kEcus[i].requestId != kEcus[i].responseId,
              "kEcus[%d] request == response", i);
    CHECK_MSG(kEcus[i].requestId <= 0x7FF && kEcus[i].responseId <= 0x7FF,
              "kEcus[%d] is not an 11-bit id", i);
  }

  // -------------------------------------------------------------------------
  // Table sizes.  kSignalCount is the size of the curated set: 19 CCN fields
  // and 10 FCMCGW fields.
  // -------------------------------------------------------------------------
  CHECK_EQ(kSignalCount, 29);
  CHECK_EQ(kScalingCount, 11);
  CHECK_EQ(kEnumCount, 33);
  // Every row of dtc_codes.csv, now that EcuId can name ABS, FDCM and ESM.
  CHECK_EQ(kDtcCount, 1152);

  // The tables are declared `extern const X[]` -- incomplete types here, so
  // sizeof() is unavailable and the counts above are the only handle on their
  // length.  The exhaustive dtcLookup sweep at the end of this file is what
  // proves kDtcCount is not larger than the array actually is.

  // -------------------------------------------------------------------------
  // Every index a SignalDef carries must be in range for its table.  A
  // dangling index is not a crash, it is a wrong number on the dashboard.
  // -------------------------------------------------------------------------
  for (uint16_t i = 0; i < kSignalCount; ++i) {
    const SignalDef& s = kSignals[i];

    CHECK_MSG(s.ecu < ECU_COUNT, "kSignals[%u] ecu %u out of range", i, s.ecu);
    CHECK_MSG(s.name != NULL && s.name[0] != '\0', "kSignals[%u] has no name", i);
    CHECK_MSG(s.bitLength > 0, "kSignals[%u] (%s) has zero bitLength", i, s.name);
    CHECK_MSG(s.render <= RENDER_RAW, "kSignals[%u] (%s) bad render %u", i, s.name, s.render);

    CHECK_MSG(s.scaling >= -1 && s.scaling < (int)kScalingCount,
              "kSignals[%u] (%s) scaling %d not in [0,%u)", i, s.name, s.scaling, kScalingCount);
    CHECK_MSG(s.enumGroup == -1 || enumGroupExists(s.enumGroup),
              "kSignals[%u] (%s) enumGroup %d has no entries in kEnums",
              i, s.name, s.enumGroup);

    // A scaled field needs a scaling; an enum field needs a group.
    if (s.render == RENDER_SCALED) {
      CHECK_MSG(s.scaling >= 0, "kSignals[%u] (%s) is RENDER_SCALED with no scaling",
                i, s.name);
    }
    if (s.render == RENDER_ENUM) {
      CHECK_MSG(s.enumGroup >= 0, "kSignals[%u] (%s) is RENDER_ENUM with no group",
                i, s.name);
    }

    // bitOffset counts from byte 0 of the full response, so anything below
    // bit 16 would be reading the service byte or the identifier echo.
    CHECK_MSG(s.bitOffset >= 16, "kSignals[%u] (%s) bitOffset %u is in the header",
              i, s.name, s.bitOffset);
  }

  // Generator promises (ecu, localId, bitOffset) order.
  for (uint16_t i = 1; i < kSignalCount; ++i) {
    const SignalDef& a = kSignals[i - 1];
    const SignalDef& b = kSignals[i];
    const bool ordered =
        a.ecu < b.ecu ||
        (a.ecu == b.ecu && (a.localId < b.localId ||
                            (a.localId == b.localId && a.bitOffset <= b.bitOffset)));
    CHECK_MSG(ordered, "kSignals not sorted at %u: %s then %s", i, a.name, b.name);
  }

  // Every emitted scaling and enum group is actually used -- proves the prune
  // worked and nothing dead is being carried along.
  for (uint8_t si = 0; si < kScalingCount; ++si) {
    bool used = false;
    for (uint16_t i = 0; i < kSignalCount && !used; ++i) {
      used = kSignals[i].scaling == (int8_t)si;
    }
    CHECK_MSG(used, "kScalings[%u] is not referenced by any signal", si);
    CHECK_MSG(kScalings[si].unit != NULL, "kScalings[%u].unit is NULL", si);
  }
  for (uint16_t e = 0; e < kEnumCount; ++e) {
    bool used = false;
    for (uint16_t i = 0; i < kSignalCount && !used; ++i) {
      used = kSignals[i].enumGroup == (int16_t)kEnums[e].group;
    }
    CHECK_MSG(used, "kEnums[%u] group %u is not referenced by any signal", e, kEnums[e].group);
    CHECK_MSG(kEnums[e].label != NULL, "kEnums[%u].label is NULL", e);
    CHECK_MSG(kEnums[e].from <= kEnums[e].to, "kEnums[%u] range is inverted", e);
  }

  // -------------------------------------------------------------------------
  // Spot checks, hand-copied from the CSVs.
  //
  // The epsilon is 1e-6 rather than something tighter because ScalingDef holds
  // float, and 0.1f is 0.100000001490116..., which a 1e-9 tolerance rejects.
  // 1e-6 is still far tighter than any plausible wrong-row mix-up.
  // -------------------------------------------------------------------------

  // pids.csv row 150: CCN,6A0,514,213E,Vehicle Speed,32,16,0,12
  // pid_scaling.csv 12: met_factor 0.0078125, met_offset 0.0, met_unit km/h, 2
  {
    const SignalDef* s = signalByName("CCN.Vehicle Speed");
    CHECK(s != NULL);
    if (s) {
      CHECK_EQ(s->ecu,       ECU_CCN);
      CHECK_EQ(s->localId,   0x213E);
      CHECK_EQ(s->bitOffset, 32);
      CHECK_EQ(s->bitLength, 16);
      CHECK_EQ(s->render,    RENDER_SCALED);
      CHECK_EQ(s->enumGroup, -1);
      CHECK(s->scaling >= 0);
      if (s->scaling >= 0) {
        const ScalingDef& sc = kScalings[s->scaling];
        CHECK_NEAR(sc.factor, 0.0078125, 1e-6);
        CHECK_NEAR(sc.offset, 0.0,       1e-6);
        CHECK_STR(sc.unit, "km/h");
        CHECK_EQ(sc.decimals, 2);
      }
    }
  }

  // pids.csv row 532: FCMCGW,620,504,21A3,Odometer,16,24,0,20
  // pid_scaling.csv 20: met_factor 0.1, met_offset 0.0, met_unit km, 1
  {
    const SignalDef* s = signalByName("FCMCGW.Odometer");
    CHECK(s != NULL);
    if (s) {
      CHECK_EQ(s->ecu,       ECU_FCMCGW);
      CHECK_EQ(s->localId,   0x21A3);
      CHECK_EQ(s->bitOffset, 16);
      CHECK_EQ(s->bitLength, 24);
      CHECK_EQ(s->render,    RENDER_SCALED);
      if (s->scaling >= 0) {
        const ScalingDef& sc = kScalings[s->scaling];
        CHECK_NEAR(sc.factor, 0.1, 1e-6);
        CHECK_NEAR(sc.offset, 0.0, 1e-6);
        CHECK_STR(sc.unit, "km");
        CHECK_EQ(sc.decimals, 1);
      }
    }
  }

  // pids.csv row 175: CCN,6A0,514,2140,Engine Coolant Temperature,16,8,0,97
  // pid_scaling.csv 97: met_factor 1.0, met_offset -40.0, met_unit degC, 0
  {
    const SignalDef* s = signalByName("CCN.Engine Coolant Temperature");
    CHECK(s != NULL);
    if (s) {
      CHECK_EQ(s->localId,   0x2140);
      CHECK_EQ(s->bitOffset, 16);
      CHECK_EQ(s->bitLength, 8);
      CHECK_EQ(s->render,    RENDER_SCALED);
      if (s->scaling >= 0) {
        const ScalingDef& sc = kScalings[s->scaling];
        CHECK_NEAR(sc.factor,  1.0,   1e-6);
        CHECK_NEAR(sc.offset, -40.0,  1e-6);
        CHECK_STR(sc.unit, "\xc2\xb0" "C");   // UTF-8 U+00B0 DEGREE SIGN + 'C'
        CHECK_EQ(sc.decimals, 0);
        // 0x5A raw -> 90 - 40 = 50 degC
        CHECK_NEAR(0x5A * sc.factor + sc.offset, 50.0, 1e-6);
      }
    }
  }

  // pids.csv row 270: FCMCGW,620,504,1A90,VIN - Current,16,136,1  (ASCII, 17 bytes)
  {
    const SignalDef* s = signalByName("FCMCGW.VIN - Current");
    CHECK(s != NULL);
    if (s) {
      CHECK_EQ(s->localId,   0x1A90);
      CHECK_EQ(s->bitOffset, 16);
      CHECK_EQ(s->bitLength, 136);
      CHECK_EQ(s->bitLength / 8, 17);       // a VIN is 17 characters
      CHECK_EQ(s->render,    RENDER_ASCII);
      CHECK_EQ(s->scaling,   -1);
      CHECK_EQ(s->enumGroup, -1);
    }
  }

  // pids.csv row 93: CCN,6A0,514,2110,Parking Brake Input,21,1,0,,enum 213
  // pid_enum_values.csv group 213: 0 -> False, 1 -> True
  {
    const SignalDef* s = signalByName("CCN.Parking Brake Input");
    CHECK(s != NULL);
    if (s) {
      CHECK_EQ(s->localId,   0x2110);
      CHECK_EQ(s->bitOffset, 21);
      CHECK_EQ(s->bitLength, 1);
      CHECK_EQ(s->render,    RENDER_ENUM);
      CHECK_EQ(s->scaling,   -1);
      CHECK_EQ(s->enumGroup, 213);
      const EnumEntry* off = findEnum(s->enumGroup, 0);
      const EnumEntry* on  = findEnum(s->enumGroup, 1);
      CHECK(off != NULL && on != NULL);
      if (off) CHECK_STR(off->label, "False");
      if (on)  CHECK_STR(on->label,  "True");
    }
  }

  // pids.csv row 541: FCMCGW,620,504,21B1,PRND Status,16,8,0,,enum 54
  {
    const SignalDef* s = signalByName("FCMCGW.PRND Status");
    CHECK(s != NULL);
    if (s) {
      CHECK_EQ(s->bitOffset, 16);
      CHECK_EQ(s->bitLength, 8);
      CHECK_EQ(s->enumGroup, 54);
      const EnumEntry* park = findEnum(54, 0);
      const EnumEntry* rev  = findEnum(54, 1);
      const EnumEntry* drv  = findEnum(54, 4);
      CHECK(park && rev && drv);
      if (park) CHECK_STR(park->label, "Park");
      if (rev)  CHECK_STR(rev->label,  "Reverse");
      if (drv)  CHECK_STR(drv->label,  "Drive");
      // 3 and 6 are simply not in the vendor table for this group.
      CHECK(findEnum(54, 3) == NULL);
    }
  }

  // -------------------------------------------------------------------------
  // signalByName
  // -------------------------------------------------------------------------
  CHECK(signalByName("CCN.Fuel Level Percentage") != NULL);
  CHECK(signalByName("FCMCGW.Battery Voltage") != NULL);
  CHECK(signalByName("Fuel Level Percentage") != NULL);   // bare name accepted
  CHECK(signalByName("No Such Signal") == NULL);
  CHECK(signalByName("") == NULL);
  CHECK(signalByName("CCN.") == NULL);
  CHECK(signalByName(NULL) == NULL);

  // "Vehicle Speed" is the one colliding name_en; the bare form resolves to
  // the lowest EcuId, the qualified forms stay distinct.
  {
    const SignalDef* bare = signalByName("Vehicle Speed");
    const SignalDef* ccn  = signalByName("CCN.Vehicle Speed");
    const SignalDef* gw   = signalByName("FCMCGW.Vehicle Speed");
    CHECK(bare != NULL && ccn != NULL && gw != NULL);
    CHECK(ccn != gw);
    CHECK(bare == ccn);
    if (gw) {
      CHECK_EQ(gw->ecu,       ECU_FCMCGW);
      CHECK_EQ(gw->localId,   0x21B0);
      CHECK_EQ(gw->bitOffset, 24);   // pids.csv row 538
      CHECK_EQ(gw->bitLength, 16);
    }
  }

  // -------------------------------------------------------------------------
  // kDtcs really is sorted by (ecu, code) -- dtcLookup binary-searches it, so
  // a single out-of-order row would make some codes invisible.
  // -------------------------------------------------------------------------
  for (uint16_t i = 1; i < kDtcCount; ++i) {
    const DtcDef& a = kDtcs[i - 1];
    const DtcDef& b = kDtcs[i];
    const bool ordered = a.ecu < b.ecu || (a.ecu == b.ecu && a.code < b.code);
    CHECK_MSG(ordered, "kDtcs not strictly sorted at %u: (%u,0x%04X) then (%u,0x%04X)",
              i, a.ecu, a.code, b.ecu, b.code);
  }
  for (uint16_t i = 0; i < kDtcCount; ++i) {
    CHECK_MSG(kDtcs[i].ecu < ECU_COUNT, "kDtcs[%u] ecu %u out of range", i, kDtcs[i].ecu);
    CHECK_MSG(kDtcs[i].sae != NULL && kDtcs[i].sae[0] != '\0', "kDtcs[%u] has no SAE code", i);
    CHECK_MSG(kDtcs[i].text != NULL && kDtcs[i].text[0] != '\0', "kDtcs[%u] has no text", i);
  }

  // -------------------------------------------------------------------------
  // dtcLookup: first, last, middle, and misses.
  // -------------------------------------------------------------------------
  {
    // First row: dtc_codes.csv CCN 0461 -> P0461
    const DtcDef* d = dtcLookup(ECU_CCN, 0x0461);
    CHECK(d != NULL);
    if (d) {
      CHECK(d == &kDtcs[0]);
      CHECK_STR(d->sae, "P0461");
      CHECK_STR(d->text, "Fuel Level Sensor 1 Performance");
    }
  }
  {
    // Last row: dtc_codes.csv ESM C141 -> U0141.  ESM is the final EcuId, so
    // its highest code is the end of the table.
    const DtcDef* d = dtcLookup(ECU_ESM, 0xC141);
    CHECK(d != NULL);
    if (d) {
      CHECK(d == &kDtcs[kDtcCount - 1]);
      CHECK_STR(d->sae, "U0141");
      CHECK_STR(d->text, "Lost Communication with IPM (FCM/TIPM)");
    }
  }
  {
    // Middle row: kDtcs[575], WCM C169 -> U0169
    const DtcDef* d = dtcLookup(ECU_WCM, 0xC169);
    CHECK(d != NULL);
    if (d) {
      CHECK(d == &kDtcs[575]);
      CHECK_STR(d->sae, "U0169");
      CHECK_STR(d->text, "Lost Communication with Sunroof Control Module");
    }
  }
  {
    // Unmoved by the three appended modules: ORC C208 is still kDtcs[492],
    // which is what "append at the end" is supposed to guarantee.
    const DtcDef* d = dtcLookup(ECU_ORC, 0xC208);
    CHECK(d != NULL);
    if (d) {
      CHECK(d == &kDtcs[492]);
      CHECK_STR(d->sae, "U0208");
      CHECK_STR(d->text, "Lost Communication With Heated Seat Control Module");
    }
  }
  {
    // The three newly-nameable modules are reachable through the search.
    const DtcDef* abs  = dtcLookup(ECU_ABS,  0x5009);
    const DtcDef* fdcm = dtcLookup(ECU_FDCM, 0xC001);
    const DtcDef* esm  = dtcLookup(ECU_ESM,  0x0607);
    CHECK(abs != NULL && fdcm != NULL && esm != NULL);
    if (abs) {
      CHECK_EQ(abs->ecu, ECU_ABS);
      CHECK_STR(abs->sae, "C1009");          // dtc_codes.csv row 2
      CHECK_STR(abs->text, "Brake Fluid Level Low");
    }
    if (fdcm) CHECK_EQ(fdcm->ecu, ECU_FDCM);
    if (esm) {
      CHECK_EQ(esm->ecu, ECU_ESM);
      CHECK_STR(esm->sae, "P0607");
    }
  }
  {
    // Same raw code, eleven different modules: U0141 exists for everything
    // except FCMCGW.  This is the test that dtcLookup keys on BOTH fields --
    // a search that ignored `ecu` would return one arbitrary row for all of
    // them, and a wrong module's description is a wrong repair.
    const DtcDef* ccn = dtcLookup(ECU_CCN, 0xC141);
    const DtcDef* pcm = dtcLookup(ECU_PCM, 0xC141);
    const DtcDef* esm = dtcLookup(ECU_ESM, 0xC141);
    CHECK(ccn != NULL && pcm != NULL && esm != NULL);
    CHECK(ccn != pcm && pcm != esm && ccn != esm);
    if (ccn) CHECK_EQ(ccn->ecu, ECU_CCN);
    if (pcm) CHECK_EQ(pcm->ecu, ECU_PCM);
    if (esm) CHECK_EQ(esm->ecu, ECU_ESM);
    CHECK(dtcLookup(ECU_FCMCGW, 0xC141) == NULL);   // the one module without it
  }

  // Absent codes.
  CHECK(dtcLookup(ECU_CCN, 0xFFFF) == NULL);
  CHECK(dtcLookup(ECU_CCN, 0x0000) == NULL);
  CHECK(dtcLookup(ECU_ESM, 0xFFFF) == NULL);      // past the end of the table
  CHECK(dtcLookup(ECU_COUNT, 0x0461) == NULL);    // no such module
  CHECK(dtcLookup(200, 0x0461) == NULL);
  // Right code, wrong module: 0461 is a CCN code only, D50A a TCM code only.
  CHECK(dtcLookup(ECU_TCM, 0x0461) == NULL);
  CHECK(dtcLookup(ECU_RADIO, 0x0461) == NULL);
  CHECK(dtcLookup(ECU_ABS, 0x0461) == NULL);
  CHECK(dtcLookup(ECU_CCN, 0xD50A) == NULL);

  // Exhaustive: every row in the table must be findable through the search.
  for (uint16_t i = 0; i < kDtcCount; ++i) {
    const DtcDef* d = dtcLookup(kDtcs[i].ecu, kDtcs[i].code);
    CHECK_MSG(d == &kDtcs[i], "dtcLookup missed kDtcs[%u] (%u,0x%04X)",
              i, kDtcs[i].ecu, kDtcs[i].code);
  }

TEST_MAIN_END()
