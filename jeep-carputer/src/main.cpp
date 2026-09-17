// jeep-carputer -- ESP32-S3 in-car computer.
//
// Milestone 1: prove the data path. Bring up the link to the ATmega328 in the
// radio, read the cluster's fault codes once, then poll a handful of live
// signals and log the decoded values.
//
// Plus a display SMOKE TEST -- solid colour fills, no LVGL, no touch. That is
// there to verify pins, colour inversion and geometry, not to be a UI. The
// real interface gets designed before it gets built, and a throwaway screen
// quietly growing into a dashboard is how that decision gets made by accident.
//
// Everything interesting is in core/, which has no ESP-IDF dependency and is
// covered by host tests. This file is only wiring.

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "At328Link.h"
#include "Board.h"
#include "Display.h"
#include "Touch.h"
#include "Settings.h"
#include "Ui.h"
#include "UiTheme.h"
#include "UiLog.h"

#include "Decoder.h"
#include "IsoTp.h"
#include "LinkHealth.h"
#include "Kwp.h"
#include "PollDecode.h"
#include "PollScheduler.h"
#include "SignalDb.h"
#include "Broadcast.h"
#include "Buttons.h"
#include "esp_system.h"
#include "BusCycle.h"
#include "TpmsSignals.h"
#include "Accel.h"
#include "FaultStore.h"
#include "FaultScan.h"
#include "Faults.h"
#include "Trip.h"
#include "TripStore.h"
#include "Snapshot.h"
#include "VehicleState.h"

static const char* TAG = "carputer";

static At328Link   gLink(LINK_UART_PORT, LINK_UART_TX, LINK_UART_RX, LINK_UART_BAUD);
static IsoTpClient gIsoTp(gLink);
static VehicleState gState;
static PollScheduler gSched;
static BroadcastState gBcast;
static TripSet gTrips;

// Acceleration timing. Written by the LINK READER TASK, unlike the trips --
// see onBroadcastFrame() for why it has to be, and why that is safe.
static AccelSet gAccel;

// Stored trouble codes, walked one module per pass of carTask.
static FaultSet gFaults;

// A long press on the TRIP page asks for a reset. The request is a flag rather
// than a direct call because gTrips is written by carTask alone: letting the
// LVGL task clear it mid-integration is the kind of race that shows up once a
// month as a trip that lost a kilometre.
static volatile uint8_t gTripResetReq = 0xFFu;

static void requestTripReset(uint8_t which) {
  gTripResetReq = which;
}

// A fresh fault scan has been asked for. A flag rather than a direct call, for
// the same reason the trip reset is one: gFaults is walked by carTask, and
// twelve ISO-TP transactions started from the LVGL task would block the
// renderer for several seconds AND race the walk already in flight.
//
// Set by RE-READ on the FAULTS page, and by the timer below.
static volatile bool gFaultScanReq = false;

static void requestFaultScan(void) {
  gFaultScanReq = true;
}

// The walk's timing and gating policy, lifted into core/FaultScan.h so it can
// be host-tested. It is the rules that moved, not the work: the twelve ISO-TP
// transactions, the logging and the mute/clear handling all stay here, because
// they are I/O and this is the task that owns the link.
//
// Worth saying why it was worth moving. The rules are short, and every one of
// them is a decision that is wrong in a way no amount of reading catches:
// whether the deadline comparison survives the 49-day wrap, whether a walk cut
// short by the link dropping is owed again immediately or in five minutes,
// whether a RE-READ arriving mid-walk restarts it or is dropped. Those were
// three comments and no tests; they are now 167 checks.
static FaultScanPlan gFaultPlan;

// The scan also starts itself: once as soon as the link is up, and then every
// five minutes.
//
// NOT at boot. With the link still down every module is marked unreachable
// without being asked, so a boot-time scan would hand the driver a completed
// walk that questioned nothing -- and it would do it just as the link is coming
// up, when the answer is about to be obtainable. Waiting for gLinkUp is what
// makes "read once when the link is established" and "read again every five
// minutes" the same piece of code.
//
// FAULT_SCAN_PERIOD_MS and the deadline itself live in core/FaultScan.h now,
// with the wrap-safe comparison that reads it.

// Which fault the driver asked to mute, by list index. 0xFF is "nothing".
// A request rather than a direct call for the same reason as everything else
// here: carTask owns gFaults, and the mute is committed to flash, which the
// LVGL task must never do.
// By (ecu, code), the same shape as the clear below and for a weaker version
// of the same reason.
//
// It used to be a list index. A scan finishing between the tap and this being
// consumed re-sorts the list, and an index then names a different fault -- and
// since the renderer reads a published snapshot it can be looking at an order
// one carTask pass old on top of that. Muting the wrong row is undone by
// tapping the right one, so this was never the mistake CLEAR guards against;
// it is just not worth having two request shapes where one of them can be
// wrong. 0xFF ecu is "nothing asked".
static volatile uint8_t  gFaultMuteEcu = 0xFFu;
static volatile uint16_t gFaultMuteCode = 0u;

static void requestFaultMute(uint8_t ecu, uint16_t code) {
  gFaultMuteCode = code;
  gFaultMuteEcu = ecu;      // LAST, so the pair is complete when it is seen
}

// Which fault the driver asked to ERASE from its module, by (ecu, code).
//
// Not by list index, unlike the mute: a scan finishing between the tap and
// this being consumed re-sorts the list, and an index would then name a
// different fault -- which for a destructive operation is the one mistake
// that cannot be undone. 0xFF ecu is "nothing asked".
static volatile uint8_t  gFaultClearEcu = 0xFFu;
static volatile uint16_t gFaultClearCode = 0u;

static void requestFaultClear(uint8_t ecu, uint16_t code) {
  gFaultClearCode = code;
  gFaultClearEcu = ecu;     // LAST, so the pair is complete when it is seen
}

// What to ask for, and how often. Modest rates on purpose: this is an 83.3 kbps
// bus that the radio and instrument cluster depend on, and every request here
// crosses a UART to an ATmega328 first.
static const PollItem kPlan[] = {
    // ONLY what broadcasts cannot supply. RPM, speed, coolant, ambient,
    // voltage, odometer and ignition all arrive free on the bus and are read
    // in the monitor window instead -- polling them would spend a transaction
    // to get a slower copy of something already in hand.
    // FUEL LEVEL IS GONE FROM THIS PLAN. It was `{ECU_CCN, 0x2103u, 5000u}`
    // with the comment "-- not broadcast", and that comment was simply wrong:
    // 0x159 b5 carries the tank level on the bus about once a second, so the
    // poll was spending a transaction every five seconds to fetch a slower copy
    // of something already arriving free. Removed on the owner's instruction
    // and in line with the rule the rest of this plan is written to.
    //
    // It also made the BENCH lie. A polled value can only ever come back as
    // whatever emulate-car.py holds in `self.fuel_pct` (62.0, hard-coded), so
    // the panel round-tripped the emulator's own constant through our scaling
    // and looked like it was reading a tank. Off the broadcast it reads the
    // real capture instead.
    // 22 s and 30 s, on the owner's instruction -- they were 5 s and 10 s.
    //
    // Both are SLOW BOOLEANS and neither is on the glass. 0x213F carries the
    // ABS, MIL, oil and charging lamp requests; 0x2135 the washer, coolant and
    // brake fluid-low flags. A lamp that comes on stays on, and a reservoir
    // does not empty between two glances -- so the only thing the old rates
    // bought was noticing a latched flag fifteen seconds sooner, at a cost of
    // one transaction every five.
    //
    // It is the same argument that took the tyres from 5 s to a minute: this is
    // an 83.3 kbps bus the radio and cluster depend on, and every request here
    // crosses a UART to an ATmega328 first.
    //
    // NO TTL MOVES WITH THESE, and that is worth stating because it is the trap
    // the tyre change had to solve. UI_TYRE_TTL_MS exists because the tyre
    // page DIMS a reading past its age, so a rate change without it would dim a
    // value as fresh as it will ever be. Neither of these reaches Snapshot at
    // all -- the lamp pills are drawn but not driven -- so there is nothing
    // downstream judging their age.
    //
    // Both fit `PollItem::periodMs`, which is a uint16_t: 65535 ms is the
    // longest period this plan can express at all.
    {ECU_CCN, 0x213Fu, 22000u},  // ABS / MIL / oil / charge lamps -- not broadcast
    {ECU_CCN, 0x2135u, 30000u},  // washer / coolant / brake low   -- not broadcast
    // Tyre pressure once a MINUTE, on the owner's instruction -- it was every
    // 5 s. Nothing is lost: a tyre loses pressure over days, the sensors
    // themselves only transmit every 60 s or so parked and every ~15 s rolling,
    // so a 5 s request mostly re-read the WCM's own held copy. What it buys is
    // bus time -- four of the five requests in this plan are gone.
    //
    // The spare used to be six times slower than the corners and now shares
    // their rate, because `PollItem::periodMs` is a uint16_t: 65535 ms is the
    // longest period this plan can express, so "slower than a minute" is not
    // available without widening a core struct. A minute for both keeps the
    // ordering that actually matters -- the spare is never asked for MORE
    // often than the wheels the warning is about -- and costs one transaction
    // a minute for a value that reads ~0.5 bar and is excluded from the low
    // warning anyway.
    //
    // Both TTLs are tied to these numbers -- UI_TYRE_TTL_MS and
    // UI_SPARE_TTL_MS, twice the poll interval each. Change a rate here and
    // change the TTL there, or the page dims a reading that is as fresh as it
    // is ever going to be. They live in core/Snapshot.h now, which is at least
    // somewhere a host test can check them: test_snapshot asserts a corner
    // 90 s old still reads FRESH. The coupling is still manual -- this is a
    // uint16_t table of periods and that header is a set of #defines, with no
    // expression that could tie one to the other -- so it is still two edits.
    {ECU_WCM, TPMS_ROAD_WHEELS_PID, 60000u},  // tyre pressures -- request-only
    {ECU_WCM, TPMS_SPARE_PID, 60000u},        // the spare      -- own request
};

static const uint8_t kPlanCount = sizeof(kPlan) / sizeof(kPlan[0]);

static uint32_t nowMs() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

// Register every signal belonging to a polled (ecu, localId). Matching on the
// pair rather than on names avoids depending on the generated naming scheme.
static void registerPlanSignals() {
  for (uint16_t i = 0u; i < kSignalCount; ++i) {
    const SignalDef& def = kSignals[i];
    for (uint8_t p = 0u; p < kPlanCount; ++p) {
      if (def.ecu == kPlan[p].ecu && def.localId == kPlan[p].localId) {
        if (gState.registerSignal(&def) < 0) {
          ESP_LOGW(TAG, "state full, dropping %s", def.name);
        }
        break;
      }
    }
  }
  // The TPMS defs are not in the generated table -- the vendor database has no
  // per-wheel pressure PID -- so they are registered explicitly.
  for (uint8_t i = 0u; i < kTpmsSignalCount; ++i) {
    if (gState.registerSignal(&kTpmsSignals[i]) < 0) {
      ESP_LOGW(TAG, "state full, dropping %s", kTpmsSignals[i].name);
    }
  }
  ESP_LOGI(TAG, "registered %u signals", gState.count());
}

// Signal names are module-qualified ("CCN.Vehicle Speed"); 39 screen
// columns does not stretch to the prefix.
static const char* shortName(const char* name) {
  const char* dot = strchr(name, '.');
  return (dot != nullptr && dot[1] != '\0') ? dot + 1 : name;
}

static void logDecoded(const SignalDef& def, const DecodedValue& v) {
  if (!v.valid) {
    ESP_LOGW(TAG, "  %-40s <invalid> raw=%" PRIu32, def.name, v.raw);
    return;
  }
  if (v.label != nullptr) {
    ESP_LOGI(TAG, "  %-40s %s", def.name, v.label);
    uiLogPrintf("%.20s %.14s", shortName(def.name), v.label);
  } else if (v.text[0] != '\0') {
    ESP_LOGI(TAG, "  %-40s \"%s\"", def.name, v.text);
    uiLogPrintf("%.18s %.16s", shortName(def.name), v.text);
  } else {
    ESP_LOGI(TAG, "  %-40s %.*f %s", def.name, static_cast<int>(v.decimals),
             static_cast<double>(v.value), v.unit);
    uiLogPrintf("%.22s %.*f%.5s", shortName(def.name),
                static_cast<int>(v.decimals),
                static_cast<double>(v.value), v.unit);
  }
}

// What the fan-out calls back with, per signal.
//
// This is the reason pollDecodeResponse() takes a sink at all: the decode is
// pure and the logging is not, and logging every value as it lands is worth
// keeping -- it is how a signal definition that disagrees with a module gets
// noticed. A failed decode is logged too, through the same path it always was.
static void pollLogSink(const SignalDef& def, const DecodedValue& v, bool ok) {
  (void)ok;
  logDecoded(def, v);
}

// One request, then decode every signal that lives in its response.
static bool pollOne(const PollItem& item) {
  if (item.ecu >= ECU_COUNT) {
    return false;
  }
  const EcuDef& ecu = kEcus[item.ecu];

  uint8_t req[4];
  const uint8_t reqLen = kwpBuildReadLocalId(item.localId, req, sizeof(req));
  if (reqLen == 0u) {
    ESP_LOGE(TAG, "cannot build request for %04X", item.localId);
    return false;
  }

  IsoTpPdu pdu;
  const IsoTpStatus st =
      gIsoTp.transact(ecu.requestId, ecu.responseId, req, reqLen, pdu, 400u);
  if (st != ISOTP_OK) {
    ESP_LOGD(TAG, "%s %04X: isotp %d", ecu.name, item.localId,
             static_cast<int>(st));
    return false;
  }

  KwpResponse resp;
  const KwpStatus ks = kwpParseResponse(req, reqLen, pdu, resp);
  if (ks == KWP_PENDING) {
    // The module's real answer is already on its way. We do not re-send here:
    // At328Link::sendFrame() drains the adapter's input before writing, so the
    // stale reply cannot be mis-attributed to the next poll.
    ESP_LOGD(TAG, "%s %04X: response pending", ecu.name, item.localId);
    return false;
  }
  if (ks != KWP_OK) {
    ESP_LOGD(TAG, "%s %04X: kwp %d nrc %02X", ecu.name, item.localId,
             static_cast<int>(ks), resp.nrc);
    return false;
  }

  // The fan-out -- find every signal defined against this PID across BOTH
  // tables, decode it, publish it -- is core/PollDecode.h. Everything above
  // this line is the transaction, which is the only part that needs a link.
  const uint8_t decoded = pollDecodeResponse(item.ecu, item.localId, pdu, gState,
                                             nowMs(), pollLogSink);
  return decoded != 0u;
}

// Ask a module who it is. `1A 87` returns an ASCII part number and every
// module in the capture answers it, so it is the cheapest way to tell "module
// absent" apart from "module present but has nothing to report".
static void probeEcu(EcuId which) {
  const EcuDef& ecu = kEcus[which];
  uint8_t req[4];
  const uint8_t reqLen = kwpBuildReadEcuIdent(0x87u, req, sizeof(req));
  if (reqLen == 0u) {
    return;
  }

  IsoTpPdu pdu;
  if (gIsoTp.transact(ecu.requestId, ecu.responseId, req, reqLen, pdu, 800u) !=
      ISOTP_OK) {
    ESP_LOGW(TAG, "%-6s absent (no answer to 1A 87)", ecu.name);
    return;
  }

  KwpResponse resp;
  if (kwpParseResponse(req, reqLen, pdu, resp) != KWP_OK) {
    ESP_LOGW(TAG, "%-6s answered, but refused 1A 87 (nrc %02X)", ecu.name, resp.nrc);
    return;
  }

  // Payload is the identifier echo then printable ASCII.
  char pn[24];
  uint8_t n = 0u;
  for (uint16_t i = 1u; i < resp.len && n + 1u < sizeof(pn); ++i) {
    const char c = (char)resp.data[i];
    if (c >= 0x20 && c < 0x7F) {
      pn[n++] = c;
    }
  }
  pn[n] = '\0';
  ESP_LOGI(TAG, "%-6s PRESENT, part number \"%s\"", ecu.name, pn);
}

// Read one module's stored faults and name them from the generated table.
static void readDtcs(EcuId which) {
  const EcuDef& ecu = kEcus[which];
  uint8_t req[4];
  const uint8_t reqLen = kwpBuildReadDtc(req, sizeof(req));

  IsoTpPdu pdu;
  if (gIsoTp.transact(ecu.requestId, ecu.responseId, req, reqLen, pdu, 1200u) !=
      ISOTP_OK) {
    ESP_LOGW(TAG, "%s: no DTC response", ecu.name);
    return;
  }

  KwpResponse resp;
  if (kwpParseResponse(req, reqLen, pdu, resp) != KWP_OK) {
    ESP_LOGW(TAG, "%s: DTC read refused", ecu.name);
    return;
  }

  DtcRecord recs[16];
  bool truncated = false;
  const uint8_t n = kwpParseDtcList(resp, recs, sizeof(recs) / sizeof(recs[0]),
                                    truncated);
  ESP_LOGI(TAG, "%s: %u fault(s)%s", ecu.name, n, truncated ? " (truncated)" : "");
  uiLogPrintf("%s: %u fault(s)", ecu.name, n);

  for (uint8_t i = 0u; i < n; ++i) {
    char sae[6];
    if (!kwpDtcToSae(recs[i].code, sae, sizeof(sae))) {
      sae[0] = '\0';
    }
    const DtcDef* known = dtcLookup(which, recs[i].code);
    ESP_LOGI(TAG, "  %s  %-10s  %s", sae, kwpDtcStatusText(recs[i].status),
             known != nullptr ? known->text : "(not in table)");
    uiLogPrintf("%s %.32s", sae,
                known != nullptr ? known->text : "unknown");
  }
}

// Screen brightness from the dash dimmer. Fails BRIGHT: if nothing has been
// heard for five seconds the panel goes to 100% rather than staying dark.
#define BACKLIGHT_SIGNAL_TTL_MS 5000u

// Two callers, on two tasks -- so TWO LATCHES, one owned by each.
//
// The latch exists only to suppress a redundant LEDC write, and it used to be
// a single function-local static shared by both callers. That is a lost
// update: carTask writes `applied = old`, the higher-priority reader task
// preempts it with a newer dimmer frame, computes and writes `new`, and then
// carTask resumes and writes `old` over the top. The panel ends one step stale
// with the latch claiming otherwise.
//
// It self-heals -- carTask recomputes from the current gBcast every pass, so
// the next pass sees `pm != applied` and corrects it within 20 ms streaming or
// 700 ms on ATMA. That is why this was a blemish rather than a bug. But a
// correctness argument that rests on "something else fixes it shortly" is the
// kind that stops being true when the loop changes.
//
// Giving each task its own latch removes the sharing instead of guarding it.
// No lock, and in particular no LEDC call inside a critical section -- which
// is what a lock here would have meant, since the compare and the write have
// to be atomic together for a shared latch to work. The cost is that both
// tasks may write the same duty once on the same change, which is two register
// writes on a path that runs a few times a second when a driver is turning the
// wheel. `whose` selects the latch; it does not change the decision.
#define BACKLIGHT_OWNER_CAR    0u
#define BACKLIGHT_OWNER_READER 1u

static void applyBacklight(uint32_t now, uint8_t whose) {
  // Each index is touched by exactly one task, so neither needs protecting.
  static uint16_t applied[2] = {0xFFFFu, 0xFFFFu};
  const bool have = (gBcast.dimmerMs != 0u) &&
                    ((now - gBcast.dimmerMs) < BACKLIGHT_SIGNAL_TTL_MS);
  // 0x1C8 broadcasts at about 2 Hz, so the same 5 s window is generous.
  const bool haveLights = (gBcast.lightsMs != 0u) &&
                          ((now - gBcast.lightsMs) < BACKLIGHT_SIGNAL_TTL_MS);
  const uint16_t pm = broadcastBacklightPermille(gBcast.dimmerRaw, have,
                                                 gBcast.lightsOn, haveLights);
  if (pm == applied[whose]) {
    return;
  }
  applied[whose] = pm;
  (void)displayBacklightPermille(pm);

  // CARTASK ONLY, and that is the fix for a second finding rather than a
  // stylistic choice. USB-Serial-JTAG blocks until the host drains it, so
  // logging from the link reader stops the UART being read -- and At328Link
  // documents only ~1.2 s of buffer before the 328 back-pressures and the
  // MCP2515 starts dropping frames on the car side, uncounted. (It was 1.6 s;
  // the stamped LAWICEL line is six bytes longer than the ELM one it replaced,
  // which took the stream from 22 % of the link to 30 % and a quarter off the
  // cover the same 4096 bytes buy.) A brightness change is exactly when a bench
  // user has a monitor attached.
  //
  // Nothing is lost by moving it: carTask recomputes every pass, so a change
  // the reader applied first is still logged within one pass. Every input is
  // printed because 0x210 byte 1 reads 0x00 both at the bottom of the wheel
  // and with the lights off, and only this line can say which case the panel
  // just acted on.
  if (whose == BACKLIGHT_OWNER_CAR) {
    ESP_LOGI(TAG, "backlight %u.%u%% (0x210 %02X %02X, 0x1C8 %02X %s, lights %s)",
             pm / 10u, pm % 10u, gBcast.dimmerByte0, gBcast.dimmerRaw,
             gBcast.lightsRaw, haveLights ? "live" : "absent",
             gBcast.lightsOn ? "ON" : "OFF");
  }
}

// The listen window, the streaming pace, the poll-only fallback and the
// requests-per-pass bound all live in core/BusCycle.h now, with the reasoning
// that ties each number to a signal rate on the bus.

// Every broadcast frame, delivered from the link's reader task the moment its
// line is parsed.
//
// Runs on that task, so it must be quick and must never block: broadcastApply()
// is a switch over frame ids and applyBacklight() is an LEDC write, both
// microseconds. Anything that can block -- an NVS commit above all -- belongs
// in carTask, and stays there.
//
// tripUpdate() is deliberately NOT called here. gTrips is written by carTask
// alone (it also applies resets and saves to flash), and a second writer would
// be a race on accumulated state that no amount of dropped-frame tolerance
// makes acceptable. carTask samples gBcast every 20 ms instead, which is
// faster than the quickest broadcast on the bus (0x002 at 20 Hz).
// Debounced page buttons. SINGLE WRITER: only onBroadcastFrame() touches this,
// on the link reader task, exactly as gBcast itself is.
// HOW LONG A FROZEN SCREEN IS ALLOWED TO LAST.
//
// 2000 and not 1000, and the difference is a legitimate operation rather than
// caution: a theme swap or an orientation flip DELETES AND REBUILDS the whole
// seven-page tree while holding lvgl_port_lock, and uiTask cannot beat until it
// finishes. Rebooting the instrument in the middle of a rebuild the driver just
// asked for would be a worse fault than the freeze this guards against.
//
// carTask comes round every ~20 ms in the bus loop, so detection is prompt once
// the window expires; the window is the whole latency. Tighten it here if a
// rebuild is ever measured well under a second.
#define UI_WATCHDOG_MS 2000u

static ButtonState gButtons;

// THE FIRST BUTTON PRESS EVER SEEN, recorded here for carTask to print.
//
// 0x394 b3 is 0x00 in ALL 5510 frames of it across every capture in this repo --
// nobody has ever pressed one of these buttons while logging -- so the bit
// assignment (bit 2 fast-forward, bit 3 rewind) is the owner's word, and not
// something any data here confirms. Neither is the BYTE: the map says the bits
// live in b4 and the owner says b3, and since b3 and b4 are both constant zero
// in every captured frame, no log in this repo can settle which is right.
//
// The REPORT RATE question this was first written to answer is settled: the
// owner checked the real head unit and it sends on press and again on release.
// What is left is the assignment itself, and one press in the car prints it.
//
// Recorded on the link reader but PRINTED from carTask: ESP_LOGx blocks until
// the USB-JTAG host drains, and a stall on the reader costs frames.
static volatile uint8_t gBtnSeen = 0u;
static volatile uint32_t gBtnSeenMs = 0u;

static void onBroadcastFrame(const CanFrame& f, uint32_t t) {
  if (broadcastApply(gBcast, f, t) && f.id == 0x210u) {
    // Apply brightness the instant the dimmer frame lands. Deferring it to the
    // end of the loop added up to two seconds of lag to a control the driver is
    // actively turning, which reads as the screen ignoring them.
    applyBacklight(t, BACKLIGHT_OWNER_READER);
  }
  // Acted on HERE and not in carTask, for the same reason the backlight is:
  // carTask comes round a loop containing a 700 ms monitor window, so a page
  // that turns up to two thirds of a second after the thumb moved reads as the
  // button not having worked. uiPageStep() only accumulates one byte, so it is
  // safe to call from this task.
  if (f.id == 0x394u) {
    if (gBcast.btnRaw != 0u) {
      gBtnSeen = gBcast.btnRaw;
      gBtnSeenMs = t;
    }
    const int8_t step = buttonsStep(gButtons, gBcast.btnRaw, t);
    if (step != 0) {
      uiPageStep(step);
    }
  }

  // Acceleration timing IS fed from here, unlike the trips two paragraphs up.
  //
  // It has to be. carTask samples gBcast once around a loop that contains a
  // 700 ms monitor window, and an accelerometer figure timed off that would
  // start its clock up to 700 ms before the car actually moved -- a fifth of a
  // brisk 0-60, all of it in the flattering direction. accelUpdate() needs
  // every 0x002 it can get, at the stamp the frame carried.
  //
  // Safe for the same reason gBcast itself is: ONE writer. This task is the
  // only one that touches gAccel, and the UI only ever reads it. The trips are
  // different because carTask also resets and commits them, which is a second
  // writer and a real race.
  //
  // accelTick() rides along on every frame, not just 0x002, so a run whose
  // speed broadcasts stop still expires on the clock rather than timing for
  // ever. At 151 frames/s there is no shortage of chances.
  // rpm as well as speed: a run may only ARM from a genuine standing start,
  // engine turning and not already wound up. See ACCEL_IDLE_RPM.
  accelUpdate(gAccel, gBcast.speedKph, gBcast.rpm, gBcast.speedMs);
  // FRESH pedal only. pedalRaw is held when 0x2C0 stops, and this flag is what
  // arms a run now rather than what decorates one -- see BC_PEDAL_TTL_MS.
  const bool pedalLive =
      gBcast.pedalMs != 0u && (uint32_t)(t - gBcast.pedalMs) < BC_PEDAL_TTL_MS;
  accelPedal(gAccel, pedalLive && gBcast.pedalRaw >= BC_PEDAL_KICKDOWN);
  accelTick(gAccel, t);
}

// How often TRIP A is committed to flash. A minute of driving is at most ~1.5 km
// and a few hundred millilitres to lose to a power cut, against a write budget
// that has to last the life of the car.
#define TRIP_SAVE_INTERVAL_MS 60000u

// LINK_QUIET_MS and LINK_PROBE_MS moved to core/LinkHealth.h, with the
// wrap-safe comparisons that read them and with their reasoning. They were
// still defined here as well for a while, token-identically, which is why
// nothing warned -- and which is exactly the shape of the bug that keeping one
// definition is meant to prevent: the day one copy is tuned and the other is
// not, the tests agree with the wrong one.

static bool gLinkUp = false;

// ---------------------------------------------------------------------------
// PUBLISHING THE SNAPSHOT -- the one place vehicle state crosses to the screen.
//
// It used to cross as five raw pointers handed to the renderer at startup,
// which meant the UI task read structures that this task and the link reader
// were still mutating. Mostly that was benign: every field is four bytes or
// fewer and naturally aligned, so a torn VALUE was never the risk. Two things
// were real, though.
//
//   * A torn STRUCTURE. faultsEndScan() insertion-sorts the fault list while
//     the drill-down is walking it, so a row could be drawn twice or skipped
//     altogether -- not a wrong number, but a wrong LIST, on the page whose
//     whole job is saying what is wrong.
//   * Nothing agreed about "now". Each readout sampled the clock in its own
//     refresh function, so two numbers in the same frame could be judged
//     against different instants, and a value could be fresh for one and stale
//     for the next.
//
// So the crossing is now explicit, in one direction, through one structure,
// under a spinlock. A spinlock rather than a mutex because both sides are
// short and neither may sleep here: the writer is a task that must get back to
// the UART, and the reader is the LVGL task at the top of a frame. ~650 bytes
// is a couple of microseconds, which is far less than either side would lose
// blocking.
//
// gSnapLock protects gSnapLive and nothing else. Do NOT reach for it to guard
// gBcast or gFaults themselves -- those keep their single-writer discipline,
// and a lock around the READ of a structure that is written unlocked would buy
// nothing while looking like it bought something.
static portMUX_TYPE gSnapLock = portMUX_INITIALIZER_UNLOCKED;
static Snapshot gSnapLive;
static SnapshotSource gSnapSrc;
static SnapshotSlots gSnapSlots;

// Called from carTask. Builds OUTSIDE the lock and copies in, so the lock is
// held for a memcpy rather than for the whole resolve -- which reads the
// VehicleState, the trips and the fault list and is much the longer half.
static void publishSnapshot(uint32_t now) {
  static Snapshot staging;
  snapshotBuild(staging, gSnapSrc, gSnapSlots, gBcast, gTrips, gAccel, gFaults,
                &gState, gLinkUp, now);
  portENTER_CRITICAL(&gSnapLock);
  gSnapLive = staging;
  portEXIT_CRITICAL(&gSnapLock);
}

// Called from the LVGL task, once per frame. The renderer's own copy is its
// private memory; this only has to get it out from under the writer.
static void readSnapshot(Snapshot& out) {
  portENTER_CRITICAL(&gSnapLock);
  out = gSnapLive;
  portEXIT_CRITICAL(&gSnapLock);
}

// Asks the adapter who it is and hands the answer to the SETTINGS info block.
// Called after bring-up and again whenever the link comes back, because the
// mode can change across a reconnect -- a 2.0 adapter swapped for a 2.1 one is
// exactly the sort of thing that happens on a bench.
static void reportLinkIdentity(void) {
  char st[96];
  char adapter[12];
  snprintf(adapter, sizeof(adapter), "--");
  if (gLink.status(st, sizeof(st)) == LINK_OK) {
    ESP_LOGI(TAG, "adapter: %s", st);
    // AT@ST opens with "FW=2.1 MODE=..."; lift just the version for the
    // SETTINGS info block rather than showing the driver the whole line.
    const char* fw = strstr(st, "FW=");
    if (fw != nullptr) {
      unsigned n = 0u;
      adapter[n++] = 'v';
      for (const char* q = fw + 3; *q != '\0' && *q != ' ' && n + 1u < sizeof(adapter);
           ++q) {
        adapter[n++] = *q;
      }
      adapter[n] = '\0';
    }
  }
  // THREE modes, not two, because the difference between the first two is
  // visible in the numbers rather than only in a log line: under SLCAN the trip
  // and acceleration figures are timed by the GATEWAY's clock, and under STREAM
  // they are timed by ours. When a figure looks wrong, which of those was in
  // force is the first thing worth knowing -- so the row says which.
  const char* mode = "ATMA";
  if (gLink.wire() == LINK_WIRE_LAWICEL) {
    mode = "SLCAN";
  } else if (gLink.wire() == LINK_WIRE_ELM) {
    mode = "STREAM";
  }
  uiSetLinkInfo(mode, adapter);
}

static void setLinkUp(bool up) {
  if (up == gLinkUp) {
    return;
  }
  gLinkUp = up;
  uiLinkReady(up);
  if (!up) {
    uiSetLinkInfo("DOWN", "--");
    uiLogPrintf("link lost");
  }
  ESP_LOGW(TAG, "link %s", up ? "UP" : "DOWN");
}

// ready() only reports that begin() once succeeded, and stays true forever
// after the cable is pulled -- which is the bug this exists to fix. Silence is
// the real signal, and the probe is what tells "quiet bus" apart from "no
// adapter": a bench with no car produces no frames at all, and that must not
// read as a dead link.
// LINK_QUIET_MS, LINK_PROBE_MS and the decision itself are in
// core/LinkHealth.h. What is left here is the two things that cannot be pure:
// sending AT@ST, and calling begin() to recover.
//
// Note what moving it FIXED, which is the argument for having moved it. The
// probe spacing used to be a function-local `static uint32_t lastProbe = 0u`,
// so `now - lastProbe < LINK_PROBE_MS` was true for the first three seconds of
// uptime and a missing adapter could not be reported in that window. That
// grace period was an accident of zero-initialisation, not a decision -- and
// it came back round for real at the far end of the 49-day clock. The host
// tests name it; nothing in the old shape could.
static LinkHealth gLinkHealth;

static void linkWatchdog(uint32_t now) {
  if (linkHealthStep(gLinkHealth, gLink.lastRxMs(), now) != LH_PROBE) {
    setLinkUp(gLinkHealth.up);
    return;
  }

  char st[96];
  if (gLink.status(st, sizeof(st)) == LINK_OK) {
    linkHealthNoteProbe(gLinkHealth, true, nowMs());
    setLinkUp(true);          // quiet bus, live adapter
    return;
  }

  linkHealthNoteProbe(gLinkHealth, false, nowMs());
  setLinkUp(false);
  // Try to bring it back rather than waiting for a reboot. begin() is bounded
  // and safe to repeat; it reuses the existing reader task and re-probes
  // AT@SM, so a reconnect restores streaming on its own.
  if (gLink.begin() == LINK_OK) {
    reportLinkIdentity();
    linkHealthNoteProbe(gLinkHealth, true, nowMs());
    setLinkUp(true);
  }
}

// Ask ONE module for its stored codes, and advance the scan.
//
// Started by the timer above -- once when the link comes up, then every
// FAULT_SCAN_PERIOD_MS -- or by RE-READ on the FAULTS page, which only brings
// the next one forward. The automatic start is gated on gLinkUp, because a walk
// with no link marks every module unreachable and repeating THAT every five
// minutes would keep replacing a real answer with an empty one.
//
// Deliberately one per pass. A full walk is twelve modules, half of them on
// CAN-C where they may never answer, so a scan that tried to finish in one go
// would spend five seconds of bus time not listening. Spread one module per
// pass it is twelve seconds of nothing anyone can perceive, which is also what
// makes repeating it every five minutes free.
//
// A longer timeout than a normal poll: a module with a full fault table takes
// visibly longer to answer 18 00 FF 00 than it does to answer a data read.
#define DTC_TIMEOUT_MS 800u

// Erase ONE stored code from the module that reported it.
//
// KWP `14 <hi> <lo>` -- clearDiagnosticInformation with the group field set to
// the specific code rather than the all-groups `FF 00`. Whether these modules
// honour the single-code form is NOT established; a refusal is logged and the
// row stays, which is why the dialog's button can end up saying FAILED.
//
// The row is removed from our list only on a POSITIVE reply. Removing it
// optimistically would show the driver a fault that is still stored in the car
// and would come back at the next read -- the frozen-but-plausible failure,
// applied to the one screen whose whole job is saying what is wrong.
static void faultClearIfAsked(void) {
  const uint8_t ecu = gFaultClearEcu;
  if (ecu == 0xFFu) {
    return;
  }
  const uint16_t code = gFaultClearCode;
  gFaultClearEcu = 0xFFu;

  if (ecu >= ECU_COUNT) {
    return;
  }
  // It may have moved since the tap, or be gone already.
  const uint8_t idx = faultsFind(gFaults, ecu, code);
  char sae[8];
  if (!kwpDtcToSae(code, sae, sizeof(sae))) {
    snprintf(sae, sizeof(sae), "%04X", (unsigned)code);
  }
  if (idx == 0xFFu) {
    ESP_LOGW(TAG, "dtc clear %s: no longer in the list", sae);
    return;
  }
  if (!gLinkUp) {
    ESP_LOGW(TAG, "dtc clear %s: link down", sae);
    return;
  }

  const EcuDef& def = kEcus[ecu];
  uint8_t req[4];
  const uint8_t reqLen = kwpBuildClearDtc(code, req, sizeof(req));
  if (reqLen == 0u) {
    return;
  }
  IsoTpPdu pdu;
  KwpResponse resp;
  const IsoTpStatus st = gIsoTp.transact(def.requestId, def.responseId, req,
                                         reqLen, pdu, DTC_TIMEOUT_MS);
  if (st != ISOTP_OK) {
    ESP_LOGW(TAG, "dtc clear %s %s: no answer (isotp %d)", def.name, sae,
             (int)st);
    return;
  }
  const KwpStatus ks = kwpParseResponse(req, reqLen, pdu, resp);
  if (ks != KWP_OK) {
    ESP_LOGW(TAG, "dtc clear %s %s: refused (kwp %d nrc %02X)", def.name, sae,
             (int)ks, resp.nrc);
    return;
  }
  ESP_LOGI(TAG, "dtc clear %s %s: erased", def.name, sae);
  // The drill-down notices the row has gone and closes its dialog itself --
  // it tracks the fault it is showing by (ecu, code) for exactly this.
  faultsRemove(gFaults, idx);
}

static void faultScanStep(void) {
  // Hand the LVGL task's request over to the plan, then let the plan say what
  // this pass should do. Nothing about WHEN is decided here any more.
  if (gFaultScanReq) {
    gFaultScanReq = false;
    faultScanRequest(gFaultPlan);
  }

  const uint8_t muteEcu = gFaultMuteEcu;
  if (muteEcu != 0xFFu) {
    const uint16_t muteCode = gFaultMuteCode;
    gFaultMuteEcu = 0xFFu;

    // It may have moved since the tap, or be gone.
    const uint8_t muteIdx = faultsFind(gFaults, muteEcu, muteCode);
    if (muteIdx == 0xFFu) {
      ESP_LOGW(TAG, "mute %04X: no longer in the list", (unsigned)muteCode);
    } else if (faultsMuteFull(gFaults) && !gFaults.f[muteIdx].muted) {
      ESP_LOGW(TAG, "mute list full (%u)", (unsigned)FAULT_MUTE_MAX);
    } else {
      const bool now = faultsToggleMute(gFaults, muteIdx);
      faultStoreSaveMutes(gFaults);
      ESP_LOGI(TAG, "fault %04X %s", (unsigned)muteCode,
               now ? "muted" : "un-muted");
    }
  }

  faultClearIfAsked();

  // ONE act per pass, decided in core/. FSCAN_BEGIN deliberately does not
  // consume the pass -- beginning and then asking module 0 in the same call is
  // what the old straight-line code did, and the driver would notice a pass
  // spent doing nothing but starting.
  uint8_t ecuIdx = 0u;
  FaultScanAct act = faultScanDecide(gFaultPlan, gLinkUp, faultsScanning(gFaults),
                                     faultsNextEcu(gFaults), ECU_COUNT, nowMs(),
                                     ecuIdx);
  if (act == FSCAN_BEGIN) {
    faultsBeginScan(gFaults, ECU_COUNT);
    ESP_LOGI(TAG, "dtc scan requested: walking %u modules", (unsigned)ECU_COUNT);
    act = faultScanDecide(gFaultPlan, gLinkUp, faultsScanning(gFaults),
                          faultsNextEcu(gFaults), ECU_COUNT, nowMs(), ecuIdx);
  }

  if (act == FSCAN_NOTHING) {
    return;
  }

  if (act == FSCAN_FINISH) {
    faultsEndScan(gFaults, nowMs());
    faultScanNoteFinished(gFaultPlan, false, nowMs());
    return;
  }

  // No link: mark it unreachable WITHOUT transacting. Asking anyway would cost
  // twelve 800 ms timeouts for an answer we already know, and this runs inside
  // the bring-up retry loop where that would delay reconnecting. The scan still
  // completes and still reports honestly -- every module quiet, nothing found.
  if (act == FSCAN_SKIP) {
    faultsEcuFailed(gFaults, ecuIdx);
    if (!faultsScanning(gFaults)) {
      faultsEndScan(gFaults, nowMs());
      // `answeredNothing` -- owed again IMMEDIATELY rather than in five
      // minutes. This walk asked nothing, and the link gate holds it off until
      // the link is actually back anyway.
      faultScanNoteFinished(gFaultPlan, true, nowMs());
      ESP_LOGW(TAG, "dtc scan abandoned: link down");
    }
    return;
  }

  const EcuDef& ecu = kEcus[ecuIdx];

  uint8_t req[4];
  const uint8_t reqLen = kwpBuildReadDtc(req, sizeof(req));
  IsoTpPdu pdu;
  KwpResponse resp;
  DtcRecord recs[FAULT_MAX];
  bool truncated = false;

  const IsoTpStatus st =
      gIsoTp.transact(ecu.requestId, ecu.responseId, req, reqLen, pdu,
                      DTC_TIMEOUT_MS);
  if (st != ISOTP_OK ||
      kwpParseResponse(req, reqLen, pdu, resp) != KWP_OK) {
    // Silent, or a negative reply. NOT the same as clean -- see Faults.h.
    ESP_LOGI(TAG, "dtc %s: no answer", ecu.name);
    faultsEcuFailed(gFaults, ecuIdx);
  } else {
    const uint8_t n = kwpParseDtcList(resp, recs, FAULT_MAX, truncated);
    faultsRecord(gFaults, ecuIdx, recs, n, truncated);
    for (uint8_t i = 0u; i < n; ++i) {
      char sae[8];
      if (!kwpDtcToSae(recs[i].code, sae, sizeof(sae))) {
        snprintf(sae, sizeof(sae), "%04X", recs[i].code);
      }
      const DtcDef* def = dtcLookup(ecuIdx, recs[i].code);
      ESP_LOGW(TAG, "dtc %s %s %s -- %s", ecu.name, sae,
               kwpDtcStatusText(recs[i].status),
               (def != nullptr && def->text != nullptr) ? def->text : "(no description)");
    }
    ESP_LOGI(TAG, "dtc %s: %u code(s)", ecu.name, (unsigned)n);
  }

  if (!faultsScanning(gFaults)) {
    faultsEndScan(gFaults, nowMs());
    faultScanNoteFinished(gFaultPlan, false, nowMs());
    ESP_LOGI(TAG, "dtc scan done: %u stored, %u modules answered, %u quiet%s",
             (unsigned)gFaults.count, (unsigned)faultsAnsweredCount(gFaults),
             (unsigned)faultsUnreachableCount(gFaults),
             gFaults.full ? ", table full" : "");
  }
}

// Pulled out and called from two places, because it has to be. The bring-up
// below retries the link forever, and everything after it -- including the only
// code that acts on a reset request -- used to sit past that loop. On a bench
// with no 328 the driver could long-press, confirm, watch the dialog close, and
// have the request sit in gTripResetReq until the end of time. A dead link is
// exactly when someone is most likely to be prodding the screen.
static void faultScanStep(void);

// Everything the trip meters need that does NOT depend on the link being up.
//
// Pulled out and called from two places, because it has to be. The bring-up
// below retries the link forever, and everything after it -- including the only
// code that acts on a reset request -- would otherwise sit past that loop.
static void tripHousekeeping(void) {
  tripUpdate(gTrips, gBcast, nowMs());

  const uint8_t resetReq = gTripResetReq;
  if (resetReq < TRIP_COUNT) {
    gTripResetReq = 0xFFu;
    tripReset(gTrips, resetReq);
    // Both sides of the reset, because "it did nothing" and "it did something
    // invisible" look identical on the panel. odo 0 means no 0x014 has
    // arrived, in which case the trip re-anchors to nothing and the distance
    // legitimately stays as dashes.
    ESP_LOGI(TAG, "trip %u reset: odo=%u start=%u km=%.1f L=%.2f",
             (unsigned)resetReq, (unsigned)gTrips.t[resetReq].odoNowDm,
             (unsigned)gTrips.t[resetReq].odoStartDm,
             (double)tripKm(gTrips.t[resetReq]),
             (double)tripLitres(gTrips.t[resetReq]));
    if (resetReq < TRIP_STORE_COUNT) {
      tripStoreSaveNow(gTrips.t[resetReq], resetReq);
    }
  }
  for (uint8_t i = 0; i < TRIP_STORE_COUNT; ++i) {
    tripStoreSave(gTrips.t[i], i, nowMs(), TRIP_SAVE_INTERVAL_MS);
  }
}

static void carTask(void*) {
  // Announced before anything can block, so "the task never started" and "the
  // task is stuck in begin()" are told apart from the console alone. They look
  // identical otherwise, and both look like a missing gateway.
  ESP_LOGI(TAG, "car task up");

  // Before begin(), so the reader task has somewhere to put frames from its
  // very first line.
  gLink.setFrameSink(onBroadcastFrame);

  // Retry rather than give up: on a cold start the 328 may still be coming up
  // with the radio, and the link is the only path to the vehicle.
  unsigned tries = 0u;
  while (gLink.begin() != LINK_OK) {
    ESP_LOGW(TAG, "link not ready, retrying");
    uiSetLinkInfo("DOWN", "--");
    uiLogPrintf("link: no 328 (try %u, %us)", ++tries,
                static_cast<unsigned>(nowMs() / 1000u));
    // Wait in slices rather than one block. A reset asked for while the link is
    // down is then applied within a tenth of a second instead of up to two, and
    // the backlight failsafe keeps running -- a dead gateway must not also mean
    // a screen stuck at whatever brightness it last saw.
    for (uint8_t i = 0u; i < 20u; ++i) {
      tripHousekeeping();
      // Also here, for the reason above: a tap on FAULTS while the link is
      // down must not sit unanswered until the link happens to come back.
      faultScanStep();
      applyBacklight(nowMs(), BACKLIGHT_OWNER_CAR);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  gLinkUp = true;
  uiLinkReady(true);

  reportLinkIdentity();

  // Probe every module we might be able to reach, rather than only the cluster.
  // On a bench with just the radio, the CCN is not present and its silence says
  // nothing about whether the link works; the RADIO answering (even with zero
  // faults) proves ISO-TP, KWP and the adapter end to end.
  static const EcuId kProbe[] = {ECU_RADIO, ECU_CCN, ECU_WCM};
  for (uint8_t i = 0u; i < sizeof(kProbe) / sizeof(kProbe[0]); ++i) {
    probeEcu(kProbe[i]);
    readDtcs(kProbe[i]);
  }

  for (uint8_t i = 0u; i < kPlanCount; ++i) {
    if (!gSched.add(kPlan[i])) {
      ESP_LOGW(TAG, "poll plan full at %u", i);
    }
  }

  broadcastClear(gBcast);
  buttonsInit(gButtons);

  // Monitoring and requesting cannot overlap -- ATMA streams until a character
  // arrives and the adapter accepts no commands meanwhile -- so the loop
  // alternates: listen for a window, then spend a few requests on what
  // listening cannot provide.
  BusLoopStats loop;
  busLoopInit(loop, nowMs());

  for (;;) {
    if (busLoopTick(loop, nowMs())) {
      ESP_LOGI(TAG, "loop %ums (worst %ums)  frames=%u decoded=%u",
               (unsigned)loop.periodMs, (unsigned)loop.reportedWorstMs,
               (unsigned)gBcast.frames, (unsigned)gBcast.decoded);
    }

    // What to do about listening, decided in core/BusCycle.h. The I/O stays
    // here: only this side can try monitorBegin(), and trying it is the only
    // way to find out whether it works.
    //
    // Order matters and the decision encodes it -- streaming is asked FIRST
    // and monitorBegin() is not even attempted in that case, because ATMA
    // would make a 2.1 adapter deaf to the requests below in exchange for
    // frames already arriving.
    const bool streaming = gLink.streaming();
    const bool monitorOk = !streaming && (gLink.monitorBegin() == LINK_OK);
    const BusListenPlan plan = busListenDecide(streaming, monitorOk);
    vTaskDelay(pdMS_TO_TICKS(plan.delayMs));
    if (plan.mode == BUS_LISTEN_MONITOR) {
      gLink.monitorEnd();
    }

    linkWatchdog(nowMs());
    applyBacklight(nowMs(), BACKLIGHT_OWNER_CAR);   // also the failsafe when 0x210 goes quiet

    // Publish one resolved frame for the screen.
    //
    // HERE rather than from the renderer's own tick, because this is the task
    // that owns every input the snapshot reads. The UI pulls a copy whenever it
    // happens to draw; it never reaches into vehicle state to build one, which
    // is the whole direction of the boundary.
    publishSnapshot(nowMs());

    // ---- RENDERER WATCHDOG -------------------------------------------------
    //
    // Reboot if the UI stops refreshing. This is the SECOND of two watchdogs
    // and it catches what the first cannot.
    //
    // The ESP task watchdog (now set to panic rather than merely log) catches a
    // task that STARVES the idle tasks -- a busy loop, which is what
    // LV_ASSERT_MALLOC's bare while(1) is. That is the freeze we actually hit.
    //
    // This one catches the opposite shape: anything that leaves the renderer
    // BLOCKED rather than spinning -- a deadlock on lvgl_port_lock, a widget
    // tree that never returns from a layout, a draw unit waiting on a semaphore
    // nobody will give. Blocked tasks yield, so the idle tasks keep running and
    // the task watchdog is perfectly happy while the screen is stone dead.
    //
    // ARMED ONLY ONCE THE UI HAS BEATEN AT LEAST ONCE, so a slow boot or a
    // headless build can never reboot-loop the truck.
    {
      static uint32_t lastBeat = 0u;
      static uint32_t lastBeatMs = 0u;
      static bool armed = false;
      const uint32_t beat = uiHeartbeat();
      const uint32_t nowTick = nowMs();
      if (beat != lastBeat) {
        lastBeat = beat;
        lastBeatMs = nowTick;
        armed = true;
      } else if (armed && (nowTick - lastBeatMs) >= UI_WATCHDOG_MS) {
        ESP_LOGE(TAG, "UI STALLED %ums (beat stuck at %u) -- rebooting",
                 static_cast<unsigned>(nowTick - lastBeatMs),
                 static_cast<unsigned>(beat));
        // Let the message reach the host before the reset takes the port with
        // it. Without this the console shows a reboot with no reason.
        vTaskDelay(pdMS_TO_TICKS(120));
        esp_restart();
      }
    }

    // The button diagnostic. Every DISTINCT byte value prints once, so holding
    // a button prints one line and the gap to the next tells us the report rate.
    {
      static uint8_t lastBtn = 0u;
      const uint8_t seen = gBtnSeen;
      if (seen != lastBtn) {
        lastBtn = seen;
        ESP_LOGI(TAG, "0x394 b3=%02X at %ums  (bit2=right, bit3=left)",
                 static_cast<unsigned>(seen),
                 static_cast<unsigned>(gBtnSeenMs));
      }
    }

    // Trip housekeeping, outside the monitor window so an NVS write can never
    // stall ATMA. The reset is applied here, on the one task that writes the
    // trips, and is committed immediately -- the driver must see a cleared
    // meter stay cleared across a key cycle.
    tripHousekeeping();

    // One module's fault table per pass, until the walk is done.
    faultScanStep();

    // A few requests, then back to listening. Bounded so a slow module cannot
    // starve the monitor window -- see BUS_POLLS_PER_PASS for the arithmetic.
    for (uint8_t n = 0u; n < BUS_POLLS_PER_PASS; ++n) {
      const PollItem* item = gSched.next(nowMs());
      if (item == nullptr) {
        break;
      }
      const bool ok = pollOne(*item);
      gSched.noteResult(ok, nowMs());
      // A poll can block for up to 400 ms waiting on a module, during which
      // 0x1A7 broadcasts four more times. Folding in after each one bounds the
      // interval tripUpdate() has to attribute to a single flow reading.
      tripUpdate(gTrips, gBcast, nowMs());
    }
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "jeep-carputer: boot");
  ESP_LOGI(TAG, "%u signals and %u DTC descriptions compiled in",
           static_cast<unsigned>(kSignalCount), static_cast<unsigned>(kDtcCount));

  registerPlanSignals();

  // Trip meters. TRIP A and TRIP B are restored from flash; FROM START stays at
  // zero by definition -- that difference is the entire reason there are three.
  accelInit(gAccel);
  faultsInit(gFaults);
  tripSetInit(gTrips);
  if (tripStoreInit() == ESP_OK) {
    for (uint8_t i = 0; i < TRIP_STORE_COUNT; ++i) {
      (void)tripStoreLoad(gTrips.t[i], i);
    }
    // HERE, not beside faultsInit() -- tripStoreInit() is what brings NVS up,
    // and loading before it silently returns an empty mute list, which looks
    // exactly like the driver never muted anything.
    (void)faultStoreLoadMutes(gFaults);
  }

  // Display bring-up. A failure here must not stop the car link: the data path
  // is the point of the device, the screen is only how it is presented.
  // Settings before the panel is told anything: the stored colour polarity has
  // to be applied as part of bring-up, or the screen flashes the wrong way
  // round for as long as it takes the UI to come up.
  (void)settingsLoad();

  // Before uiInit(): every widget bakes in its colour when it is built, so the
  // stored palette has to be active before anything is built with it.
  uiThemeSet(settingsGet().theme);

  // Resolve the VehicleState slots the snapshot reads from.
  //
  // This used to live in uiInit(), which is why the renderer had to be handed
  // the hub at all. It belongs here: the names are a property of SignalDb and
  // of this vehicle, and every one of them is already registered a few lines
  // above by the same kPlan walk. The renderer never needed to know a signal
  // had a NAME.
  //
  // indexOfName wants the exact name; a signal that is not registered comes
  // back -1 and the snapshot reads that readout as ABSENT, which is the honest
  // answer and not a crash.
  faultScanPlanInit(gFaultPlan);
  linkHealthInit(gLinkHealth);

  snapshotSourceInit(gSnapSrc);
  snapshotSlotsInit(gSnapSlots);
  gSnapSlots.coolant = gState.indexOfName("CCN.Engine Coolant Temperature");
  gSnapSlots.volts = gState.indexOfName("CCN.System Voltage");
  gSnapSlots.speed = gState.indexOfName("CCN.Vehicle Speed");
  gSnapSlots.rpm = gState.indexOfName("CCN.Engine Revolutions Per Minute");
  gSnapSlots.ambient = gState.indexOfName("CCN.Ambient Temperature");
  gSnapSlots.tyre[0] = gState.indexOfName("WCM.Tyre Sensor 1");
  gSnapSlots.tyre[1] = gState.indexOfName("WCM.Tyre Sensor 2");
  gSnapSlots.tyre[2] = gState.indexOfName("WCM.Tyre Sensor 3");
  gSnapSlots.tyre[3] = gState.indexOfName("WCM.Tyre Sensor 4");
  gSnapSlots.spare = gState.indexOfName("WCM.Tyre Spare");

  // One publish before the UI exists, so the first frame draws a resolved
  // snapshot rather than the zeroed one. Both read the same dashes today --
  // nothing has arrived yet -- but it means the renderer is never looking at a
  // structure that no builder has touched.
  publishSnapshot(nowMs());

  if (displayInit() == ESP_OK) {
    // Brief flash so a blank screen can be told apart from a dead one before
    // LVGL takes the panel over.
    (void)displayFill(RGB565_BLACK);
    ESP_ERROR_CHECK(displayBacklight(100u));
    uiSetSnapshotReader(readSnapshot);
    uiSetTripReset(requestTripReset);
    uiSetFaultActions(requestFaultScan, requestFaultMute, requestFaultClear);
    if (uiInit() != ESP_OK) {
      ESP_LOGE(TAG, "dashboard unavailable");
    } else if (touchInit() != ESP_OK) {
      // Readable but not swipeable. Worth continuing: every screen still shows
      // its data, and DRIVE is where the UI parks itself anyway.
      ESP_LOGE(TAG, "touch unavailable, screens will not swipe");
    }
  } else {
    ESP_LOGE(TAG, "display unavailable, continuing headless");
  }

  // Its own task: the link blocks with timeouts, which is fine here but must
  // not hold up whatever else the system grows later. Started BEFORE the splash
  // hold so the handshake runs behind the welcome screen rather than after it.
  // Core 0, away from the renderer on core 1. Its long blocking UART reads are
  // then invisible to the display no matter how long a module takes to answer.
  //
  // THE RETURN IS CHECKED, and it was not. A failed xTaskCreate here is the one
  // failure in this file that is completely SILENT: no task, so no begin(), so
  // no "adapter ready" and no "link not ready, retrying" either -- the two logs
  // anyone debugging a dead link goes looking for. The panel comes up, renders
  // happily and shows dashes for ever, which is indistinguishable from a
  // gateway that is not plugged in.
  if (xTaskCreatePinnedToCore(carTask, "car", 6144, nullptr, 5, nullptr, 0) !=
      pdPASS) {
    ESP_LOGE(TAG, "car task creation FAILED -- no link, no vehicle data");
  }

  // Blocks app_main only, and app_main is allowed to return. The hold is what
  // covers the 328 handshake; the ceiling is what stops a dead gateway from
  // leaving the welcome screen up forever.
  uiSplash(1900u);
}
