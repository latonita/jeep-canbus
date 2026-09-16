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
#include "Kwp.h"
#include "PollScheduler.h"
#include "SignalDb.h"
#include "Broadcast.h"
#include "TpmsSignals.h"
#include "Accel.h"
#include "FaultStore.h"
#include "Faults.h"
#include "Trip.h"
#include "TripStore.h"
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

// The scan also starts itself: once as soon as the link is up, and then every
// five minutes.
//
// NOT at boot. With the link still down every module is marked unreachable
// without being asked, so a boot-time scan would hand the driver a completed
// walk that questioned nothing -- and it would do it just as the link is coming
// up, when the answer is about to be obtainable. Waiting for gLinkUp is what
// makes "read once when the link is established" and "read again every five
// minutes" the same piece of code.
#define FAULT_SCAN_PERIOD_MS (5u * 60u * 1000u)

// When the next automatic scan is owed, compared SIGNED against nowMs() so 0
// means "as soon as the link allows" and the 49-day wrap needs no special case.
// Written only by the completion paths in faultScanStep(), on carTask.
static uint32_t gFaultScanDueMs = 0u;

// Which fault the driver asked to mute, by list index. 0xFF is "nothing".
// A request rather than a direct call for the same reason as everything else
// here: carTask owns gFaults, and the mute is committed to flash, which the
// LVGL task must never do.
static volatile uint8_t gFaultMuteReq = 0xFFu;

static void requestFaultMute(uint8_t index) {
  gFaultMuteReq = index;
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
    {ECU_CCN, 0x2103u, 5000u},   // fuel level %      -- not broadcast
    {ECU_CCN, 0x213Fu, 5000u},   // warning lamps     -- not broadcast
    {ECU_CCN, 0x2135u, 10000u},  // fluid-low flags   -- not broadcast
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
    // Both TTLs in Ui.cpp are tied to these numbers -- UI_TYRE_TTL_MS and
    // UI_SPARE_TTL_MS, twice the poll interval each. Change a rate here and
    // change the TTL there, or the page dims a reading that is as fresh as it
    // is ever going to be.
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

  const uint32_t t = nowMs();
  uint8_t decoded = 0u;
  // Two tables: the generated one, and the hand-written TPMS defs that the
  // vendor database does not contain.
  const SignalDef* tables[2] = {kSignals, kTpmsSignals};
  const uint16_t counts[2] = {kSignalCount, kTpmsSignalCount};
  for (uint8_t tbl = 0u; tbl < 2u; ++tbl)
  for (uint16_t i = 0u; i < counts[tbl]; ++i) {
    const SignalDef& def = tables[tbl][i];
    if (def.ecu != item.ecu || def.localId != item.localId) {
      continue;
    }
    const int8_t slot = gState.indexOf(&def);
    if (slot < 0) {
      continue;
    }
    DecodedValue v;
    if (decodeSignal(def, pdu, v) == DECODE_OK && v.valid) {
      gState.publish(slot, v.value, v.raw, v.label, t);
      ++decoded;
    } else {
      gState.noteError(slot);
    }
    logDecoded(def, v);
  }
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

static void applyBacklight(uint32_t now) {
  static uint16_t applied = 0xFFFFu;
  const bool have = (gBcast.dimmerMs != 0u) &&
                    ((now - gBcast.dimmerMs) < BACKLIGHT_SIGNAL_TTL_MS);
  // 0x1C8 broadcasts at about 2 Hz, so the same 5 s window is generous.
  const bool haveLights = (gBcast.lightsMs != 0u) &&
                          ((now - gBcast.lightsMs) < BACKLIGHT_SIGNAL_TTL_MS);
  const uint16_t pm = broadcastBacklightPermille(gBcast.dimmerRaw, have,
                                                 gBcast.lightsOn, haveLights);
  if (pm == applied) {
    return;
  }
  applied = pm;
  (void)displayBacklightPermille(pm);
  // Every input to the decision, because 0x210 byte 1 reads 0x00 both at the
  // bottom of the wheel and with the lights off, and only this line can say
  // which case the panel just acted on.
  ESP_LOGI(TAG, "backlight %u.%u%% (0x210 %02X %02X, 0x1C8 %02X %s, lights %s)",
           pm / 10u, pm % 10u, gBcast.dimmerByte0, gBcast.dimmerRaw,
           gBcast.lightsRaw, haveLights ? "live" : "absent",
           gBcast.lightsOn ? "ON" : "OFF");
}

// How long to sit in ATMA before dropping out to run requests. Long enough that
// the 50 ms signals arrive several times over, short enough that a 5 s poll
// period is still comfortably met.
//
// ONLY used against an adapter older than 2.1. With AT@SM the frames arrive
// continuously on the link's reader task and there is no window at all.
#define MONITOR_WINDOW_MS 700u

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
static void onBroadcastFrame(const CanFrame& f, uint32_t t) {
  if (broadcastApply(gBcast, f, t) && f.id == 0x210u) {
    // Apply brightness the instant the dimmer frame lands. Deferring it to the
    // end of the loop added up to two seconds of lag to a control the driver is
    // actively turning, which reads as the screen ignoring them.
    applyBacklight(t);
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

// No line of any kind off the adapter for this long and we stop believing it
// is there. Generous against the worst legitimate quiet: in the ATMA fallback
// the adapter goes silent for the whole poll burst, bounded by three polls at
// 400 ms plus the window, so about 2 s. 3 s clears that with room.
#define LINK_QUIET_MS 3000u

// Minimum spacing between liveness probes, so a genuinely dead link is not
// hammered with AT@ST once per loop.
#define LINK_PROBE_MS 3000u

static bool gLinkUp = false;

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
  uiSetLinkInfo(gLink.streaming() ? "STREAM" : "ATMA", adapter);
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
static void linkWatchdog(uint32_t now) {
  static uint32_t lastProbe = 0u;

  if (gLink.lastRxMs() != 0u && (now - gLink.lastRxMs()) < LINK_QUIET_MS) {
    setLinkUp(true);
    return;
  }
  if ((now - lastProbe) < LINK_PROBE_MS) {
    return;
  }
  lastProbe = now;

  char st[96];
  if (gLink.status(st, sizeof(st)) == LINK_OK) {
    setLinkUp(true);          // quiet bus, live adapter
    return;
  }

  setLinkUp(false);
  // Try to bring it back rather than waiting for a reboot. begin() is bounded
  // and safe to repeat; it reuses the existing reader task and re-probes
  // AT@SM, so a reconnect restores streaming on its own.
  if (gLink.begin() == LINK_OK) {
    reportLinkIdentity();
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
  // Find it by (ecu, code). It may have moved, or be gone already.
  uint8_t idx = 0xFFu;
  for (uint8_t i = 0u; i < gFaults.count; ++i) {
    if (gFaults.f[i].ecu == ecu && gFaults.f[i].code == code) {
      idx = i;
      break;
    }
  }
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
  // Self-starting, gated on the link. The due stamp is moved only by the
  // completion paths below, so a walk cut short by the link dropping is owed
  // again the moment the link returns rather than five minutes later.
  if (gLinkUp && !faultsScanning(gFaults) &&
      (int32_t)(nowMs() - gFaultScanDueMs) >= 0) {
    gFaultScanReq = true;
  }

  // A request only takes effect between walks. Restarting one mid-flight would
  // throw away the modules already asked and re-ask them, which from the
  // driver's side looks like the scan going backwards.
  if (gFaultScanReq && !faultsScanning(gFaults)) {
    gFaultScanReq = false;
    faultsBeginScan(gFaults, ECU_COUNT);
    ESP_LOGI(TAG, "dtc scan requested: walking %u modules", (unsigned)ECU_COUNT);
  }

  const uint8_t muteReq = gFaultMuteReq;
  if (muteReq != 0xFFu) {
    gFaultMuteReq = 0xFFu;
    if (faultsMuteFull(gFaults) && !gFaults.f[muteReq].muted) {
      ESP_LOGW(TAG, "mute list full (%u)", (unsigned)FAULT_MUTE_MAX);
    } else {
      const bool now = faultsToggleMute(gFaults, muteReq);
      faultStoreSaveMutes(gFaults);
      ESP_LOGI(TAG, "fault %u %s", (unsigned)muteReq,
               now ? "muted" : "un-muted");
    }
  }

  faultClearIfAsked();

  const uint8_t ecuIdx = faultsNextEcu(gFaults);
  if (ecuIdx == 0xFFu) {
    return;
  }
  if (ecuIdx >= ECU_COUNT) {
    faultsEndScan(gFaults, nowMs());
    gFaultScanDueMs = nowMs() + FAULT_SCAN_PERIOD_MS;
    return;
  }

  // No link: mark it unreachable WITHOUT transacting. Asking anyway would cost
  // twelve 800 ms timeouts for an answer we already know, and this runs inside
  // the bring-up retry loop where that would delay reconnecting. The scan still
  // completes and still reports honestly -- every module quiet, nothing found.
  if (!gLinkUp) {
    faultsEcuFailed(gFaults, ecuIdx);
    if (!faultsScanning(gFaults)) {
      faultsEndScan(gFaults, nowMs());
      // Owed immediately, not in five minutes: this walk answered nothing, and
      // the gate above holds it off until the link is actually back anyway.
      gFaultScanDueMs = nowMs();
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
    gFaultScanDueMs = nowMs() + FAULT_SCAN_PERIOD_MS;
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
      applyBacklight(nowMs());
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

  // Monitoring and requesting cannot overlap -- ATMA streams until a character
  // arrives and the adapter accepts no commands meanwhile -- so the loop
  // alternates: listen for a window, then spend a few requests on what
  // listening cannot provide.
  uint32_t loopStart = nowMs();
  uint32_t loopWorst = 0u;
  uint32_t loopReport = nowMs();

  for (;;) {
    const uint32_t period = nowMs() - loopStart;
    loopStart = nowMs();
    if (period > loopWorst) {
      loopWorst = period;
    }
    if (nowMs() - loopReport > 10000u) {
      loopReport = nowMs();
      ESP_LOGI(TAG, "loop %ums (worst %ums)  frames=%u decoded=%u",
               (unsigned)period, (unsigned)loopWorst,
               (unsigned)gBcast.frames, (unsigned)gBcast.decoded);
      loopWorst = 0u;
    }

    if (gLink.streaming()) {
      // 2.1 adapter: frames are arriving continuously on the reader task and
      // reaching onBroadcastFrame() without this loop's help. All this delay
      // does is pace the loop fast enough that tripUpdate() below never misses
      // a broadcast stamp -- 20 ms against 0x002's 50 ms, the quickest on the
      // bus.
      vTaskDelay(pdMS_TO_TICKS(20));
    } else if (gLink.monitorBegin() == LINK_OK) {
      // Older adapter: monitoring and requesting cannot overlap, so listen for
      // a window and go deaf for the poll burst. Frames still arrive through
      // the same sink -- the window only decides when the adapter is willing
      // to send them.
      vTaskDelay(pdMS_TO_TICKS(MONITOR_WINDOW_MS));
      gLink.monitorEnd();
    } else {
      // No monitor mode (bench link, or the adapter refused): fall back to
      // polling alone rather than stalling.
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    linkWatchdog(nowMs());
    applyBacklight(nowMs());   // also covers the failsafe when 0x210 goes silent

    // Trip housekeeping, outside the monitor window so an NVS write can never
    // stall ATMA. The reset is applied here, on the one task that writes the
    // trips, and is committed immediately -- the driver must see a cleared
    // meter stay cleared across a key cycle.
    tripHousekeeping();

    // One module's fault table per pass, until the walk is done.
    faultScanStep();

    // A few requests, then back to listening. Bounded so a slow module cannot
    // starve the monitor window.
    for (uint8_t n = 0u; n < 3u; ++n) {
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

  if (displayInit() == ESP_OK) {
    // Brief flash so a blank screen can be told apart from a dead one before
    // LVGL takes the panel over.
    (void)displayFill(RGB565_BLACK);
    ESP_ERROR_CHECK(displayBacklight(100u));
    uiSetBroadcast(&gBcast);
    uiSetTrips(&gTrips, requestTripReset);
    uiSetAccel(&gAccel);
    uiSetFaults(&gFaults, requestFaultScan, requestFaultMute,
                requestFaultClear);
    if (uiInit(&gState) != ESP_OK) {
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
  xTaskCreatePinnedToCore(carTask, "car", 6144, nullptr, 5, nullptr, 0);

  // Blocks app_main only, and app_main is allowed to return. The hold is what
  // covers the 328 handshake; the ceiling is what stops a dead gateway from
  // leaving the welcome screen up forever.
  uiSplash(1900u);
}
