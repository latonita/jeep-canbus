// Does the EMULATOR agree with the DASHBOARD? Decode its output with the
// firmware's own core/ and check the numbers that come out.
//
//   python scripts/emulate-car.py --port X --dump-frames frames.txt
//   clang++ -std=gnu++14 -Icore scripts/emu-verify.cpp core/LinkLine.cpp //     core/IsoTp.cpp core/Kwp.cpp core/PollDecode.cpp core/Decoder.cpp //     core/SignalDb.cpp core/TpmsSignals.cpp core/VehicleState.cpp //     core/Snapshot.cpp core/Broadcast.cpp core/Trip.cpp core/Accel.cpp //     core/Faults.cpp -o emu-verify && ./emu-verify frames.txt
//
// NOT part of run-host-tests.sh, on purpose: it needs Python to produce its
// input, and it tests a BENCH TOOL rather than the firmware. It is checked in
// for the same reason scripts/replay-accel.cpp is -- so it is not rewritten
// from scratch the next time somebody wonders whether the emulator is lying.
//
// It uses STL, so it will not link with -fsanitize=address on this MSVC
// target (stl_asan.lib is absent); build it plain. The firmware suites avoid
// STL, which is why they sanitize fine.
//
// WHAT IT IS FOR. An emulator that disagrees with the firmware about a
// response layout is worse than no emulator: the page stays dashed, or worse
// shows a wrong number, and every hour of the hunt is spent in the firmware.
// bitOffset in the signal tables counts from byte 0 of the FULL response,
// service echo included, so the first data byte of `61 3B ...` sits at bit 16.
// Get that wrong in the emulator and all four tyre pressures shift two bytes
// while looking entirely plausible. This is the check that catches it.

// Close the loop: take the EMULATOR's wire output and decode it with the
// FIRMWARE's own code -- linkParseFrameLine, IsoTpReassembler,
// kwpParseResponse, kwpParseDtcList, pollDecodeResponse. If the emulator and
// the dashboard ever disagree about a layout, this is where it shows, rather
// than on a bench with a dashed page and no clue why.
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <map>

#include "LinkLine.h"
#include "IsoTp.h"
#include "Kwp.h"
#include "PollDecode.h"
#include "TpmsSignals.h"
#include "Snapshot.h"

static int g_fail = 0;
#define OK(cond, ...) do { if (!(cond)) { ++g_fail; std::printf("  FAIL: "); \
  std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

static std::map<std::string, IsoTpPdu> g_pdus;
static std::map<std::string, std::string> g_reqOf;

static uint8_t reqBytes(const std::string& hex, uint8_t* out) {
  uint8_t n = 0;
  for (size_t i = 0; i + 1 < hex.size(); i += 2) {
    out[n++] = (uint8_t)std::strtoul(hex.substr(i, 2).c_str(), NULL, 16);
  }
  return n;
}

int main(int argc, char** argv) {
  if (argc < 2) { std::printf("need frames file\n"); return 2; }
  FILE* fh = std::fopen(argv[1], "r");
  if (!fh) { std::printf("cannot open %s\n", argv[1]); return 2; }

  char line[256];
  std::string scen, req;
  uint16_t respId = 0;
  std::vector<std::string> frames;

  // Local lambda would be neater but this stays C++14-plain on purpose.
  struct Finisher {
    static void run(std::string& scen, std::string& req, uint16_t respId,
                    std::vector<std::string>& frames) {
      if (scen.empty()) return;
      IsoTpReassembler r;
      IsoTpPdu pdu;
      IsoTpStatus last = ISOTP_INCOMPLETE;
      for (size_t i = 0; i < frames.size(); ++i) {
        const std::string& s = frames[i];
        CanFrame f;
        uint32_t stamp = 0;
        const bool okLine = linkParseFrameLine(s.c_str(), (uint16_t)s.size(),
                                               LINK_LINE_TS_NONE, f, stamp);
        OK(okLine, "%s: firmware REJECTED the emulator's line \"%s\"",
           scen.c_str(), s.c_str());
        if (!okLine) continue;
        OK(f.id == respId, "%s: id %03X, expected %03X",
           scen.c_str(), (unsigned)f.id, (unsigned)respId);
        last = r.feed(f, pdu);
      }
      OK(last == ISOTP_OK, "%s: reassembly ended %d, not ISOTP_OK",
         scen.c_str(), (int)last);
      if (last == ISOTP_OK) { g_pdus[scen] = pdu; g_reqOf[scen] = req; }
      frames.clear();
    }
  };

  while (std::fgets(line, sizeof(line), fh)) {
    char a[64], b[64], c[64];
    if (std::sscanf(line, "SCENARIO %63s %63s %63s", a, b, c) == 3) {
      Finisher::run(scen, req, respId, frames);
      scen = a; req = b; respId = (uint16_t)std::strtoul(c, NULL, 10);
    } else if (std::sscanf(line, "FRAME %63s", a) == 1) {
      frames.push_back(a);
    }
  }
  Finisher::run(scen, req, respId, frames);
  std::fclose(fh);

  // ---- TYRES: through the real fan-out, into a real VehicleState ---------
  if (g_pdus.count("tyres")) {
    VehicleState vs;
    int8_t slot[4];
    for (uint8_t i = 0; i < 4; ++i) slot[i] = vs.registerSignal(&kTpmsSignals[i]);
    uint8_t rq[8];
    const uint8_t rn = reqBytes(g_reqOf["tyres"], rq);
    KwpResponse kr;
    const KwpStatus ks = kwpParseResponse(rq, rn, g_pdus["tyres"], kr);
    OK(ks == KWP_OK, "tyres: kwp %d", (int)ks);
    const uint8_t n = pollDecodeResponse(ECU_WCM, TPMS_ROAD_WHEELS_PID,
                                         g_pdus["tyres"], vs, 1000u, NULL);
    OK(n == 4, "tyres: %u signals published, expected 4", (unsigned)n);
    const int want[4] = {33, 29, 34, 33};
    for (uint8_t i = 0; i < 4; ++i) {
      const float psi = vs.slot(slot[i])->value;
      OK((int)(psi + 0.5f) == want[i], "tyre %u: %.0f psi, expected %d",
         (unsigned)(i + 1), psi, want[i]);
      std::printf("  tyre %u = %.0f psi (%.2f bar)%s\n", (unsigned)(i + 1), psi,
                  psi * UI_PSI_TO_BAR,
                  (psi * UI_PSI_TO_BAR < UI_TYRE_LOW_BAR) ? "  LOW" : "");
    }
  } else { OK(false, "tyres: no PDU"); }

  // ---- SPARE -------------------------------------------------------------
  if (g_pdus.count("spare")) {
    VehicleState vs;
    const int8_t sp = vs.registerSignal(&kTpmsSignals[4]);
    const uint8_t n = pollDecodeResponse(ECU_WCM, TPMS_SPARE_PID, g_pdus["spare"],
                                         vs, 1000u, NULL);
    OK(n == 1, "spare: %u published", (unsigned)n);
    OK((int)(vs.slot(sp)->value + 0.5f) == 7, "spare: %.0f psi, expected 7",
       vs.slot(sp)->value);
    std::printf("  spare = %.0f psi\n", vs.slot(sp)->value);
  } else { OK(false, "spare: no PDU"); }

  // ---- FUEL: scaling x0.5 must land on 62% -------------------------------
  if (g_pdus.count("fuel")) {
    const SignalDef* def = NULL;
    for (uint16_t i = 0; i < kSignalCount; ++i) {
      if (kSignals[i].ecu == ECU_CCN && kSignals[i].localId == 0x2103) {
        def = &kSignals[i];
        break;
      }
    }
    OK(def != NULL, "fuel: no signal def for CCN 0x2103");
    if (def) {
      VehicleState vs;
      const int8_t s = vs.registerSignal(def);
      const uint8_t n = pollDecodeResponse(ECU_CCN, 0x2103u, g_pdus["fuel"], vs,
                                           1000u, NULL);
      OK(n == 1, "fuel: %u published", (unsigned)n);
      const float pct = vs.slot(s)->value;
      OK(pct > 61.5f && pct < 62.5f, "fuel: %.1f%%, expected 62", pct);
      std::printf("  fuel = %.1f%%\n", pct);
    }
  } else { OK(false, "fuel: no PDU"); }

  // ---- DTCs: the multi-frame path, and the codes as DRAWN ----------------
  const char* scens[2] = {"dtc_wcm", "dtc_ccn"};
  const int counts[2] = {2, 1};
  const char* codes[2][2] = {{"C0077", "B2102"}, {"U0184", NULL}};
  for (int w = 0; w < 2; ++w) {
    if (!g_pdus.count(scens[w])) { OK(false, "%s: no PDU", scens[w]); continue; }
    uint8_t rq[8];
    const uint8_t rn = reqBytes(g_reqOf[scens[w]], rq);
    KwpResponse kr;
    const KwpStatus ks = kwpParseResponse(rq, rn, g_pdus[scens[w]], kr);
    OK(ks == KWP_OK, "%s: kwp %d", scens[w], (int)ks);
    DtcRecord recs[16];
    bool trunc = false;
    const uint8_t n = kwpParseDtcList(kr, recs, 16, trunc);
    OK(n == counts[w], "%s: %u codes, expected %d", scens[w], (unsigned)n, counts[w]);
    OK(!trunc, "%s: reported truncated", scens[w]);
    for (uint8_t i = 0; i < n && i < 2; ++i) {
      char sae[6];
      OK(kwpDtcToSae(recs[i].code, sae, sizeof(sae)), "%s: unrenderable", scens[w]);
      if (codes[w][i]) {
        OK(std::strcmp(sae, codes[w][i]) == 0, "%s: got %s, expected %s",
           scens[w], sae, codes[w][i]);
      }
      const DtcDef* known = dtcLookup(
          (uint8_t)(std::strcmp(scens[w], "dtc_wcm") == 0 ? ECU_WCM : ECU_CCN),
          recs[i].code);
      std::printf("  %s: %s  %-16s  %s\n", scens[w], sae,
                  kwpDtcStatusText(recs[i].status),
                  (known && known->text) ? known->text : "(not in table)");
    }
  }

  // ---- IDENT: what probeEcu uses to tell absent from clean --------------
  if (g_pdus.count("ident")) {
    uint8_t rq[8];
    const uint8_t rn = reqBytes(g_reqOf["ident"], rq);
    KwpResponse kr;
    const KwpStatus ks = kwpParseResponse(rq, rn, g_pdus["ident"], kr);
    OK(ks == KWP_OK, "ident: kwp %d", (int)ks);
    std::printf("  ident = \"");
    for (uint16_t i = 2; i < kr.len; ++i) {
      if (kr.data[i] >= 0x20 && kr.data[i] < 0x7F) std::printf("%c", kr.data[i]);
    }
    std::printf("\"\n");
  } else { OK(false, "ident: no PDU"); }

  // ---- CLEAR: a positive reply is what lets a row leave the list ---------
  if (g_pdus.count("clear")) {
    uint8_t rq[8];
    const uint8_t rn = reqBytes(g_reqOf["clear"], rq);
    KwpResponse kr;
    const KwpStatus ks = kwpParseResponse(rq, rn, g_pdus["clear"], kr);
    OK(ks == KWP_OK, "clear: kwp %d, expected KWP_OK", (int)ks);
    std::printf("  clear -> KWP_OK (service %02X)\n", (unsigned)kr.service);
  } else { OK(false, "clear: no PDU"); }

  // ---- A REFUSAL must read as NEGATIVE, not as data ---------------------
  if (g_pdus.count("badpid")) {
    uint8_t rq[8];
    const uint8_t rn = reqBytes(g_reqOf["badpid"], rq);
    KwpResponse kr;
    const KwpStatus ks = kwpParseResponse(rq, rn, g_pdus["badpid"], kr);
    OK(ks == KWP_NEGATIVE, "badpid: kwp %d, expected KWP_NEGATIVE", (int)ks);
    OK(kr.nrc == 0x31, "badpid: nrc %02X, expected 31", (unsigned)kr.nrc);
    std::printf("  bad pid -> KWP_NEGATIVE nrc %02X\n", (unsigned)kr.nrc);
  } else { OK(false, "badpid: no PDU"); }

  std::printf("\n%s\n", g_fail ? "FAILURES ABOVE" :
              "emulator output decodes correctly through the firmware's own core/");
  return g_fail ? 1 : 0;
}
