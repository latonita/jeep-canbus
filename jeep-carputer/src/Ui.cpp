#include "Ui.h"

#include <stdio.h>
#include <string.h>

#include "Board.h"
#include "Display.h"
#include "UiTheme.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "Broadcast.h"
#include "Trip.h"
#include "VehicleState.h"

static const char* TAG = "ui";

// Rows LVGL renders per flush. Two of these are double-buffered: 2 x 240 x 32 x
// 16 bpp is ~30 KB of internal DMA RAM, which buys tear-free scrolling and
// still leaves the bulk of the heap. A full framebuffer would be 150 KB and
// does not fit while PSRAM is off.
// Bigger flush chunks mean fewer SPI transactions per frame: 48 rows is 10
// chunks for a full screen instead of 10 at 32. Two of them is ~46 KB of
// internal DMA RAM, comfortable against ~248 KB free.
#define UI_BUF_ROWS 48

// WCM 21 3B reports one psi per bit, with no offset and no scaling. The cabin
// reads in bar, so the conversion lives here rather than in the decoder: the
// wire value stays the wire value, and only the presentation changes.
//
// For scale: the owner's stated target band of 2.1-2.3 bar is 30.5-33.4 psi,
// and the recorded corners (30-34 psi) are 2.07-2.34 bar.
#define UI_PSI_TO_BAR 0.0689476f

// Tank capacity. The WH 3.0 CRD carries 78.0 L, and CCN reports fuel level as a
// percentage, so litres is the percentage scaled -- the only number on DRIVE
// with no unpinned constant behind it.
#define UI_TANK_LITRES 78.0f

// Bottom of the owner's stated cold band. Below this a road wheel is called
// low; the spare is excluded, since it reads about 0.5 bar and would otherwise
// hold the warning on permanently.
#define UI_TYRE_LOW_BAR 2.1f

// ---- The flow readout ----------------------------------------------------
//
// 0x1A7 is a rate in known units -- L/h = Q/160 -- so unlike the first version
// of this screen the bar CAN be scaled to something real and CAN carry a number
// beside it.
//
// Full scale is 40 L/h. An OM642 peaks higher than that, but almost no driving
// happens up there: idle is under 1 L/h and a cruise is around 10, so a bar
// scaled to the engine's true peak would spend its life in the bottom sixth and
// show nothing useful. 40 puts a cruise at a quarter lit, leaves room for a
// hard overtake, and pegs on full throttle -- which is information in itself.
// It CLAMPS rather than auto-ranging: a bar whose full scale moves means
// something different every time the driver looks at it.
#define UI_FLOW_SEGS 16
#define UI_FLOW_FULL_LPH 40.0f

// Where the instantaneous readout changes unit.
//
// Stopped, or crawling below the speed this firmware already calls stopped,
// litres-per-100-km is arithmetic on a number approaching zero: an idling
// engine at 1 km/h reads 80 L/100km, which is arithmetically true and useless.
// Sitting in a jam or parked and running, litres per HOUR is the figure that
// means something -- it is what tells you what an hour of waiting costs.
//
// Two thresholds, not one, and the gap between them is deliberate. A crawling
// car crosses a single threshold over and over, and a readout that flips
// between "0.9 L/h" and "62.0 L/100" every few seconds cannot be read -- which
// is precisely the traffic-jam case this exists for. Below 2 km/h it is always
// L/h; above 3 km/h always L/100; in between it keeps whatever it was showing.
#define UI_FLOW_TO_LPH_KPH TRIP_MOVING_KPH
#define UI_FLOW_TO_L100_KPH 3.0f

// Chrome heights. Pages occupy what is left between them.
#define UI_PAGE_Y (UI_STATUSBAR_H)
#define UI_PAGE_H (LCD_V_RES - UI_STATUSBAR_H)

static VehicleState* s_state = nullptr;
static const BroadcastState* s_bcast = nullptr;
static const TripSet* s_trips = nullptr;
static void (*s_tripReset)(uint8_t) = nullptr;

// Which trip the TRIP page is currently showing. One tap swaps it.
static uint8_t s_tripShown = TRIP_A;
static lv_display_t* s_disp = nullptr;
static lv_obj_t* s_root = nullptr;
static lv_obj_t* s_pages = nullptr;   // the horizontal scroller
static lv_obj_t* s_page[UI_SCREEN_COUNT];
static lv_obj_t* s_dot[UI_SCREEN_COUNT];
static lv_obj_t* s_splash = nullptr;
static uint8_t s_current = UI_HOME;
static bool s_splashDone = false;

// Cached VehicleState slot indices, resolved once at init. -1 means the signal
// is not in the poll plan, which is a normal condition, not an error.
static struct {
  int8_t coolant, volts, speed, rpm, fuel, ambient;
  int8_t tyre[4], spare;
} s_slot;

// Live widgets.
static struct {
  lv_obj_t* linkDot;
  lv_obj_t* ign;
  lv_obj_t* clockLbl;
  lv_obj_t* l100;
  lv_obj_t* tank;
  lv_obj_t* range;
  lv_obj_t* flowSeg[UI_FLOW_SEGS];
  lv_obj_t* flowNow;
  lv_obj_t* speed;
  lv_obj_t* rpm;
  lv_obj_t* healthVolts;
  lv_obj_t* healthCoolant;
  lv_obj_t* healthAmbient;
  lv_obj_t* tyre[4];
  lv_obj_t* spare;
  lv_obj_t* tyreLow;
  lv_obj_t* tyreTarget;
  lv_obj_t* confirmTitle;
  lv_obj_t* confirmDetail;
  lv_obj_t* confirmBar;
  lv_obj_t* tripTitle;
  lv_obj_t* tripKm;
  lv_obj_t* tripUsed;
  lv_obj_t* tripL100;
  lv_obj_t* tripMoving;
  lv_obj_t* tripAvgKph;
  lv_obj_t* tripHint;
} W;

// ---------------------------------------------------------------------------
// Small builders. Absolute positioning throughout: at 240x320 a flex tree costs
// more in surprises than it saves in code, and these layouts are fixed.
// ---------------------------------------------------------------------------

// lv_obj_create() hands back a styled, clickable, scrollable container. All of
// that is unwanted here: a clickable decoration steals the drag from the
// scroller underneath it, which is what made swiping work only on bare
// background before.
static void stripChrome(lv_obj_t* o) {
  lv_obj_set_style_border_width(o, 0, 0);
  lv_obj_set_style_pad_all(o, 0, 0);
  lv_obj_set_style_radius(o, 0, 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, 0);
  lv_obj_remove_flag(
      o, static_cast<lv_obj_flag_t>(LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE));
}

static lv_obj_t* mkScreen() {
  lv_obj_t* scr = lv_obj_create(nullptr);
  stripChrome(scr);
  lv_obj_set_style_bg_color(scr, UI_BG, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  return scr;
}

static lv_obj_t* mkLabel(lv_obj_t* parent, const char* text, const lv_font_t* font,
                         lv_color_t colour, lv_coord_t x, lv_coord_t y) {
  lv_obj_t* l = lv_label_create(parent);
  lv_label_set_text(l, text);
  lv_obj_set_style_text_font(l, font, 0);
  lv_obj_set_style_text_color(l, colour, 0);
  lv_obj_set_pos(l, x, y);
  return l;
}

// Section legend: uppercase with generous tracking, the way an instrument
// cluster labels its fields.
static lv_obj_t* mkCaption(lv_obj_t* parent, const char* text, lv_coord_t x,
                           lv_coord_t y) {
  lv_obj_t* l = mkLabel(parent, text, UI_FONT_LABEL, UI_LABEL, x, y);
  lv_obj_set_style_text_letter_space(l, 2, 0);
  return l;
}

static lv_obj_t* mkRule(lv_obj_t* parent, lv_coord_t y, lv_coord_t x, lv_coord_t w) {
  lv_obj_t* r = lv_obj_create(parent);
  stripChrome(r);
  lv_obj_set_size(r, w, 1);
  lv_obj_set_pos(r, x, y);
  lv_obj_set_style_bg_color(r, UI_RULE, 0);
  lv_obj_set_style_bg_opa(r, LV_OPA_COVER, 0);
  return r;
}

static lv_obj_t* mkDot(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, lv_coord_t d,
                       lv_color_t colour) {
  lv_obj_t* o = lv_obj_create(parent);
  stripChrome(o);
  lv_obj_set_size(o, d, d);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_bg_color(o, colour, 0);
  lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
  return o;
}

static lv_obj_t* mkOutline(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, lv_coord_t w,
                           lv_coord_t h, lv_coord_t radius, lv_color_t colour,
                           lv_coord_t width) {
  lv_obj_t* o = lv_obj_create(parent);
  stripChrome(o);
  lv_obj_set_size(o, w, h);
  lv_obj_set_pos(o, x, y);
  lv_obj_set_style_border_color(o, colour, 0);
  lv_obj_set_style_border_width(o, width, 0);
  lv_obj_set_style_radius(o, radius, 0);
  return o;
}

// ---- chrome ---------------------------------------------------------------

// Status bar and page dots live on the ROOT, not inside the pages, so they stay
// put while the pages slide under them -- and so there is one of each rather
// than five.
static void buildStatusBar(lv_obj_t* root) {
  W.clockLbl = mkLabel(root, "--:--", UI_FONT_LABEL, UI_PRIMARY, UI_PAD - 2, 5);
  W.ign = mkLabel(root, "--", UI_FONT_LABEL, UI_DIM, 108, 5);
  lv_obj_t* lk = mkLabel(root, "LINK", UI_FONT_LABEL, UI_DIM, 176, 5);
  lv_obj_set_style_text_letter_space(lk, 1, 0);
  W.linkDot = mkDot(root, 222, 11, 6, UI_FAINT);
  mkRule(root, UI_STATUSBAR_H - 1, 0, LCD_H_RES);
}

static void buildDots(lv_obj_t* root) {
  const lv_coord_t y = LCD_V_RES - 16;
  const lv_coord_t step = 12;
  const lv_coord_t x0 = (LCD_H_RES - (UI_SCREEN_COUNT * step - (step - 5))) / 2;
  for (uint8_t i = 0; i < UI_SCREEN_COUNT; ++i) {
    s_dot[i] = mkDot(root, static_cast<lv_coord_t>(x0 + i * step), y, 5,
                     (i == UI_HOME) ? UI_PRIMARY : UI_DOT_OFF);
  }
}

static void setActiveDot(uint8_t active) {
  for (uint8_t i = 0; i < UI_SCREEN_COUNT; ++i) {
    if (s_dot[i] != nullptr) {
      lv_obj_set_style_bg_color(s_dot[i], (i == active) ? UI_PRIMARY : UI_DOT_OFF, 0);
    }
  }
}

// Page title plus an optional right-aligned freshness note. Returns the title
// label, for the one page whose title is not constant.
static lv_obj_t* buildHeader(lv_obj_t* page, const char* title, const char* note) {
  lv_obj_t* t = mkLabel(page, title, UI_FONT_LABEL, UI_PRIMARY, UI_PAD - 2, 5);
  lv_obj_set_style_text_letter_space(t, 3, 0);
  if (note != nullptr) {
    lv_obj_t* n = mkLabel(page, note, UI_FONT_LABEL, UI_LABEL, 0, 0);
    lv_obj_align(n, LV_ALIGN_TOP_RIGHT, -(UI_PAD - 2), 5);
  }
  return t;
}

// A two-up row of value cells, the pattern DRIVE and TRIP both use.
static void buildPair(lv_obj_t* page, lv_coord_t y, const char* capL, const char* capR,
                      lv_obj_t** outL, lv_obj_t** outR, const char* unitL,
                      const char* unitR) {
  mkRule(page, y, 0, LCD_H_RES);
  mkCaption(page, capL, UI_PAD - 2, y + 5);
  mkCaption(page, capR, LCD_H_RES / 2 + UI_PAD - 2, y + 5);
  *outL = mkLabel(page, "--", UI_FONT_MID, UI_PRIMARY, UI_PAD - 2, y + 20);
  *outR = mkLabel(page, "--", UI_FONT_MID, UI_PRIMARY, LCD_H_RES / 2 + UI_PAD - 2,
                  y + 20);
  if (unitL != nullptr) {
    mkLabel(page, unitL, UI_FONT_LABEL, UI_LABEL, UI_PAD + 76, y + 32);
  }
  if (unitR != nullptr) {
    mkLabel(page, unitR, UI_FONT_LABEL, UI_LABEL, LCD_H_RES / 2 + UI_PAD + 76, y + 32);
  }
}

// ---- pages ----------------------------------------------------------------
// Coordinates are page-relative: the status bar is no longer part of a page.

// DRIVE is about fuel, not about speed.
//
// The car already has a speedometer 30 cm away and a coolant gauge beside it.
// What it does NOT have is any idea what it is drinking, so that is what this
// card is for: the average since power-up as the headline, the instantaneous
// draw as a bar, and what is left in the tank underneath. Speed and rpm stay,
// small, at the bottom -- useful for reading the average in context, not worth
// the hero slot.
static void buildDrive(lv_obj_t* page) {
  mkCaption(page, "FROM START", UI_PAD - 2, 6);
  W.l100 = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_PAD - 4, 20);
  mkLabel(page, "L/100km", UI_FONT_LABEL, UI_DIM, 152, 64);

  mkCaption(page, "FLOW", UI_PAD - 2, 96);
  // The live figure shares the caption row, right-aligned. Value and unit in
  // one label because the unit itself changes -- L/h at a standstill, L/100km
  // once moving -- and two labels would have the number jumping sideways as the
  // unit beside it changed width.
  W.flowNow = mkLabel(page, "--", UI_FONT_LABEL, UI_FAINT, 0, 0);
  lv_obj_align(W.flowNow, LV_ALIGN_TOP_RIGHT, -(UI_PAD - 2), 96);

  for (uint8_t i = 0; i < UI_FLOW_SEGS; ++i) {
    lv_obj_t* s = lv_obj_create(page);
    stripChrome(s);
    lv_obj_set_size(s, 12, 12);
    lv_obj_set_pos(s, static_cast<lv_coord_t>(UI_PAD - 2 + i * 14), 112);
    lv_obj_set_style_bg_color(s, UI_DOT_OFF, 0);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, 0);
    W.flowSeg[i] = s;
  }

  buildPair(page, 136, "IN TANK", "RANGE", &W.tank, &W.range, "L", "km");
  buildPair(page, 194, "SPEED", "ENGINE", &W.speed, &W.rpm, "km/h", "rpm");
  mkRule(page, 252, 0, LCD_H_RES);
}

static void buildTyres(lv_obj_t* page) {
  buildHeader(page, "TYRES", nullptr);

  // Cells are labelled by sensor index because which physical corner each
  // sensor sits on is genuinely unknown -- `21 3B` gives four bytes in a fixed
  // order and nothing in the captures says which wheel is which. Naming them
  // LF/RF/LR/RR would be a guess, and a guess about which tyre is soft is worse
  // than no name at all. Deflating one wheel and watching which value moves is
  // the only way to map them.
  static const char* const kCaps[4] = {"SENSOR 1", "SENSOR 2", "SENSOR 3", "SENSOR 4"};
  mkRule(page, 32, 0, LCD_H_RES);
  for (uint8_t i = 0; i < 4; ++i) {
    const lv_coord_t x = (i & 1u) ? LCD_H_RES / 2 + UI_PAD - 2 : UI_PAD - 2;
    const lv_coord_t y = static_cast<lv_coord_t>(40 + (i >> 1) * 74);
    mkCaption(page, kCaps[i], x, y);
    W.tyre[i] =
        mkLabel(page, "--", UI_FONT_MID, UI_PRIMARY, x, static_cast<lv_coord_t>(y + 16));
    mkLabel(page, "bar", UI_FONT_LABEL, UI_LABEL, static_cast<lv_coord_t>(x + 72),
            static_cast<lv_coord_t>(y + 28));
  }
  mkRule(page, 106, 0, LCD_H_RES);
  mkRule(page, 180, 0, LCD_H_RES);

  // The spare sits below the rule, on its own, because it is not one of the
  // four: `21 3B` excludes it and it answers a different request entirely.
  // One line, label-sized, not a fifth cell. It is not a road wheel: you do not
  // check it at a glance and you cannot do anything about it while moving. Given
  // a cell the size of the corners it would read as equally urgent, and at 0.48
  // bar it is the lowest number on the screen -- the eye goes straight to it.
  mkCaption(page, "SPARE", UI_PAD - 2, 190);
  W.spare = mkLabel(page, "--", UI_FONT_LABEL, UI_DIM, 68, 190);
  mkLabel(page, "bar", UI_FONT_LABEL, UI_LABEL, 106, 190);

  // The target band appears ONLY when a road wheel is under it.
  //
  // A target printed permanently is a number the driver has to compare four
  // readings against every time, on a screen glanced at from a moving vehicle.
  // Printed only when something is actually low, it stops being reference
  // material and becomes the answer: what is wrong, and what it should be.
  mkRule(page, 244, 0, LCD_H_RES);
  W.tyreLow = mkLabel(page, "", UI_FONT_LABEL, UI_CRIT, UI_PAD - 2, 250);
  W.tyreTarget = mkLabel(page, "TARGET 2.1 - 2.3", UI_FONT_LABEL, UI_LABEL, 0, 0);
  lv_obj_align(W.tyreTarget, LV_ALIGN_TOP_RIGHT, -(UI_PAD - 2), 250);
  lv_obj_add_flag(W.tyreLow, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(W.tyreTarget, LV_OBJ_FLAG_HIDDEN);
}

static void buildHealth(lv_obj_t* page) {
  buildHeader(page, "HEALTH", nullptr);

  W.healthVolts = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_PAD - 4, 20);
  mkLabel(page, "V", UI_FONT_LABEL, UI_DIM, 158, 64);

  // Coolant and ambient moved here off DRIVE, which is now a fuel card. They
  // belong together: both are temperatures, neither changes fast enough to be
  // worth a glance while moving, and ambient had been decoded from 0x015 and
  // displayed nowhere at all.
  buildPair(page, 92, "COOLANT", "AMBIENT", &W.healthCoolant, &W.healthAmbient, "C",
            "C");

  mkRule(page, 150, 0, LCD_H_RES);
  mkCaption(page, "LAMPS", UI_PAD - 2, 156);
  // MIL is deliberately absent: CCN 213F bit24 reads true in every recorded
  // sample including at speed, so showing it would mean a permanent warning.
  static const char* const kLamps[3] = {"OIL", "ABS", "CHARGE"};
  for (uint8_t i = 0; i < 3; ++i) {
    lv_obj_t* p = mkOutline(page, static_cast<lv_coord_t>(UI_PAD - 2 + i * 73), 176, 68,
                            26, 0, UI_RULE, 1);
    lv_obj_t* t = mkLabel(p, kLamps[i], UI_FONT_LABEL, UI_FAINT, 0, 0);
    lv_obj_center(t);
  }

  mkRule(page, 214, 0, LCD_H_RES);
  mkCaption(page, "FAULTS", UI_PAD - 2, 220);
  mkLabel(page, "not scanned", UI_FONT_LABEL, UI_DIM, UI_PAD - 2, 238);
}

// The two meters share one page and one set of widgets; a tap swaps which trip
// feeds them. Two pages would have cost a swipe and a dot for what is the same
// five numbers measured over a different window.
static const char* const kTripTitle[TRIP_COUNT] = {"TRIP A", "FROM START"};

static void setTripTitle(void) {
  if (W.tripTitle != nullptr) {
    lv_label_set_text(W.tripTitle, kTripTitle[s_tripShown]);
  }
}

// The meter swap is a DOUBLE tap, not a single one.
//
// A single tap is what a thumb does to a panel it is only steadying itself
// against, and on the one page that also carries a destructive gesture that is
// too easy to fire by accident. Two taps is a deliberate act; it also leaves a
// single tap free to mean "dismiss whatever is on screen".
//
// The lower bound filters the digitiser rather than the driver: two CLICKED
// events closer together than a human hand can produce them are one contact
// reported twice, and must not read as a double tap.
#define UI_DOUBLE_TAP_MIN_MS 80u
#define UI_DOUBLE_TAP_MAX_MS 450u

static uint32_t s_lastTapMs = 0u;
static bool s_tripLongFired = false;

// Defined with the rest of the confirmation dialog, below.
static bool confirmOpen(void);
static void hideConfirm(void);

// A tap anywhere on the page swaps meters.
//
// LVGL only sends CLICKED when the press did not turn into a scroll, so this
// cannot fire on a swipe that happens to end over the page -- which is the
// whole reason the gesture is safe to put on a page inside a scroller.
static void onTripTap(lv_event_t* e) {
  (void)e;
  // A long press ends with a release, and a release sends CLICKED. Without
  // this, holding to reset would also swap the meter on the way out -- so the
  // driver would clear TRIP A and be looking at FROM START, which reads as the
  // reset having done nothing.
  if (s_tripLongFired) {
    s_tripLongFired = false;
    s_lastTapMs = 0u;
    return;
  }
  // Tapping the page while the dialog is up dismisses it and does nothing else.
  // Swapping meters out from under an open "RESET TRIP A" would leave the
  // driver confirming a reset of something other than what they asked for.
  if (confirmOpen()) {
    ESP_LOGI(TAG, "confirm: dismissed by a tap outside it");
    hideConfirm();
    s_lastTapMs = 0u;
    return;
  }
  const uint32_t now = lv_tick_get();
  const uint32_t since = now - s_lastTapMs;
  if (s_lastTapMs != 0u && since >= UI_DOUBLE_TAP_MIN_MS && since <= UI_DOUBLE_TAP_MAX_MS) {
    s_lastTapMs = 0u;   // consume the pair, so three taps are not two swaps
    s_tripShown = (uint8_t)((s_tripShown + 1u) % TRIP_COUNT);
    setTripTitle();
    return;
  }
  // Either the first tap of a possible pair, or one that came too fast to be
  // real. Both start the window again rather than doing anything.
  if (since >= UI_DOUBLE_TAP_MIN_MS) {
    s_lastTapMs = (now == 0u) ? 1u : now;
  }
}

// ---------------------------------------------------------------------------
// Reset confirmation
//
// A long press no longer clears the meter; it asks. Resetting is irreversible
// -- there is no undo for a trip, and TRIP A may hold weeks of driving -- and
// the gesture that triggers it is one a thumb resting on the panel over a bad
// road surface can produce by accident.
//
// It dismisses itself three ways: the CANCEL button, five seconds of being
// ignored, or a swipe. All three are the same decision, which is that the
// driver did not mean it. Only the RESET button means yes, and it is the one
// control on the panel drawn in the critical colour.
//
// The dialog lives on the ROOT, not on the TRIP page, so the pages keep sliding
// underneath it. That is deliberate: it makes a swipe read as "I am going
// somewhere else" rather than as a modal being fought with, and there is one
// dialog rather than one per page.
// ---------------------------------------------------------------------------

#define UI_CONFIRM_MS 5000u

static lv_obj_t* s_confirm = nullptr;
static uint32_t s_confirmStart = 0u;   // lv_tick when it opened; 0 = closed

static void hideConfirm(void) {
  if (s_confirm == nullptr || s_confirmStart == 0u) {
    return;
  }
  s_confirmStart = 0u;
  lv_obj_add_flag(s_confirm, LV_OBJ_FLAG_HIDDEN);
}

static void showConfirm(void) {
  if (s_confirm == nullptr) {
    return;
  }
  // Name the meter on the button-bearing dialog itself. "RESET" alone, on a
  // screen with two meters one tap apart, is not enough to act on.
  if (W.confirmTitle != nullptr) {
    char buf[32];
    snprintf(buf, sizeof(buf), "RESET %s", kTripTitle[s_tripShown]);
    lv_label_set_text(W.confirmTitle, buf);
  }
  // What is about to be destroyed, in the units it was read in. A driver who
  // opened this by accident recognises the numbers and cancels.
  if (W.confirmDetail != nullptr && s_trips != nullptr) {
    const Trip& t = s_trips->t[s_tripShown];
    const float km = tripKm(t);
    const float l = tripLitres(t);
    char buf[32];
    if (km < 0.0f) {
      snprintf(buf, sizeof(buf), "nothing recorded yet");
    } else if (l < 0.0f) {
      snprintf(buf, sizeof(buf), "%.1f km", static_cast<double>(km));
    } else {
      snprintf(buf, sizeof(buf), "%.1f km   %.2f L", static_cast<double>(km),
               static_cast<double>(l));
    }
    lv_label_set_text(W.confirmDetail, buf);
  }
  if (W.confirmBar != nullptr) {
    lv_obj_set_width(W.confirmBar, 184);   // refill before it is seen
  }
  s_confirmStart = lv_tick_get();
  if (s_confirmStart == 0u) {
    s_confirmStart = 1u;   // 0 is the closed sentinel
  }
  lv_obj_remove_flag(s_confirm, LV_OBJ_FLAG_HIDDEN);
}

static bool confirmOpen(void) { return s_confirmStart != 0u; }

// Clicks that land in the first moments after the dialog appears are ignored.
// The dialog opens under the finger that opened it, and a driver who lets go
// the instant it appears must not have destroyed a trip by doing so.
#define UI_CONFIRM_ARM_MS 400u

static void onConfirmYes(lv_event_t* e) {
  (void)e;
  if (s_confirmStart != 0u && lv_tick_elaps(s_confirmStart) < UI_CONFIRM_ARM_MS) {
    ESP_LOGI(TAG, "confirm: RESET ignored, still arming");
    return;
  }
  // Logged because the two halves of this live on different tasks: the button
  // records the intent here and carTask logs "trip N reset" when it acts on it.
  // Seeing one without the other says immediately which half is broken.
  ESP_LOGI(TAG, "confirm: RESET trip %u (fn=%p)", (unsigned)s_tripShown,
           (void*)s_tripReset);
  if (s_tripReset != nullptr) {
    s_tripReset(s_tripShown);
  }
  hideConfirm();
}

static void onConfirmNo(lv_event_t* e) {
  (void)e;
  ESP_LOGI(TAG, "confirm: CANCEL");
  hideConfirm();
}

// Long press asks. A trip you cannot zero is an odometer, and TRIP A in
// particular has to be clearable or it only ever grows.
static void onTripLongPress(lv_event_t* e) {
  (void)e;
  s_tripLongFired = true;
  showConfirm();
}

static void buildTrip(lv_obj_t* page) {
  W.tripTitle = buildHeader(page, kTripTitle[TRIP_A], nullptr);

  W.tripKm = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_PAD - 4, 30);
  mkLabel(page, "km", UI_FONT_LABEL, UI_DIM, 158, 74);

  buildPair(page, 114, "USED", "AVERAGE", &W.tripUsed, &W.tripL100, "L", "L/100");
  buildPair(page, 178, "MOVING", "AVG SPEED", &W.tripMoving, &W.tripAvgKph, nullptr,
            "km/h");

  // Bottom of the page, not the header row: the header already carries a title
  // that grows to "FROM START", and a right-aligned hint of this length would
  // run straight into it.
  mkRule(page, 242, 0, LCD_H_RES);
  W.tripHint = mkLabel(page, "2 TAPS SWAP", UI_FONT_LABEL, UI_FAINT, UI_PAD - 2, 250);
  lv_obj_set_style_text_letter_space(W.tripHint, 1, 0);
  // Two labels rather than one string: the pair only just fits the width, and
  // right-aligning the second guarantees it cannot run off the edge as the
  // wording changes.
  lv_obj_t* hint2 = mkLabel(page, "HOLD RESET", UI_FONT_LABEL, UI_FAINT, 0, 0);
  lv_obj_set_style_text_letter_space(hint2, 1, 0);
  lv_obj_align(hint2, LV_ALIGN_TOP_RIGHT, -(UI_PAD - 2), 250);

  lv_obj_add_event_cb(page, onTripTap, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(page, onTripLongPress, LV_EVENT_LONG_PRESSED, nullptr);
}

// A tap target with a border and a centred legend. Deliberately large: 88 x 44
// is past the 44 px a fingertip needs, on a panel that gets used over broken
// tarmac with one hand on the wheel.
static lv_obj_t* mkButton(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, lv_coord_t w,
                          const char* text, lv_color_t colour, lv_event_cb_t cb) {
  lv_obj_t* b = mkOutline(parent, x, y, w, 44, 2, colour, 2);
  lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_t* t = mkLabel(b, text, UI_FONT_LABEL, colour, 0, 0);
  lv_obj_set_style_text_letter_space(t, 2, 0);
  lv_obj_center(t);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
  return b;
}

static void buildConfirm(lv_obj_t* root) {
  // Opaque, not a translucent scrim. The page behind it is already pure black,
  // so a scrim would darken nothing; what separates the dialog from the page is
  // the border and the fact that the panel is filled at all.
  s_confirm = lv_obj_create(root);
  stripChrome(s_confirm);
  lv_obj_set_size(s_confirm, 208, 156);
  lv_obj_set_pos(s_confirm, 16, 88);
  lv_obj_set_style_bg_color(s_confirm, UI_BG, 0);
  lv_obj_set_style_bg_opa(s_confirm, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(s_confirm, UI_PRIMARY, 0);
  lv_obj_set_style_border_width(s_confirm, 2, 0);
  lv_obj_set_style_radius(s_confirm, 4, 0);

  // CLICKABLE, even though the panel itself does nothing when clicked.
  //
  // lv_indev_search_obj() walks children first and then asks lv_obj_hit_test()
  // about the object itself -- and hit_test returns false for anything without
  // this flag, so the search CARRIES ON to the next sibling. stripChrome()
  // removes the flag, which left a hole in the middle of the dialog: a tap on
  // its background fell straight through to the TRIP page underneath, where
  // onTripTap read it as a tap-outside and dismissed. That is why it closed no
  // matter where it was tapped. Being clickable makes the panel absorb its own
  // area, which is what a dialog is for.
  lv_obj_add_flag(s_confirm, LV_OBJ_FLAG_CLICKABLE);

  W.confirmTitle = mkCaption(s_confirm, "RESET", 14, 16);
  lv_obj_set_style_text_color(W.confirmTitle, UI_PRIMARY, 0);
  W.confirmDetail = mkLabel(s_confirm, "", UI_FONT_LABEL, UI_DIM, 14, 38);

  // CANCEL first and on the left, where a thumb sweeping in from the edge lands
  // first, and RESET in the critical colour -- the only control on this panel
  // that destroys anything.
  mkButton(s_confirm, 12, 74, 88, "CANCEL", UI_DIM, onConfirmNo);
  mkButton(s_confirm, 108, 74, 88, "RESET", UI_CRIT, onConfirmYes);

  // The timeout, drawn rather than spelled out. A rule that shortens says "this
  // is going away on its own" without asking the driver to read a number.
  W.confirmBar = mkRule(s_confirm, 140, 12, 184);
  lv_obj_set_style_bg_color(W.confirmBar, UI_FAINT, 0);

  lv_obj_add_flag(s_confirm, LV_OBJ_FLAG_HIDDEN);
}

// ---------------------------------------------------------------------------
// Frame-rate instrumentation
//
// LV_EVENT_REFR_START fires every refresh cycle whether or not anything was
// redrawn; RENDER_READY fires only after actual rendering. Counting both
// separates "LVGL is not being called often enough" from "LVGL is being called
// but the work is too slow", which are opposite problems with opposite fixes.
// ---------------------------------------------------------------------------

static uint32_t s_refrCycles = 0u;
static uint32_t s_renders = 0u;
static uint32_t s_renderUs = 0u;
static uint32_t s_renderStartUs = 0u;

static void onRefrStart(lv_event_t* e) {
  (void)e;
  ++s_refrCycles;
  s_renderStartUs = (uint32_t)esp_timer_get_time();
}

static void onRenderReady(lv_event_t* e) {
  (void)e;
  ++s_renders;
  s_renderUs += (uint32_t)esp_timer_get_time() - s_renderStartUs;
}

// ---------------------------------------------------------------------------
// Navigation
//
// The pages are children of ONE horizontally-scrolling container with snap
// points -- not five screens swapped by an animation.
//
// That distinction is the whole feel of the thing. Detecting a swipe on release
// and then playing a fixed animation gives a slideshow: nothing moves until you
// let go, and what happens next bears no relation to how you dragged. Binding
// the scroll offset to the finger -- which is what a scroll container does --
// puts the page under your thumb the whole way, and the snap only decides where
// it settles. Momentum and over-scroll resistance come along for free.
//
// The cost is wrapping: a scroller has ends. Five pages with hard ends is the
// same trade every phone launcher makes, and the ends give positional feedback
// that a wrap actively hides.
// ---------------------------------------------------------------------------

static uint8_t pageFromScroll() {
  if (s_pages == nullptr) {
    return s_current;
  }
  int32_t idx = (lv_obj_get_scroll_x(s_pages) + LCD_H_RES / 2) / LCD_H_RES;
  if (idx < 0) {
    idx = 0;
  }
  if (idx >= static_cast<int32_t>(UI_SCREEN_COUNT)) {
    idx = UI_SCREEN_COUNT - 1;
  }
  return static_cast<uint8_t>(idx);
}

// Where the finger landed when the current gesture began. A flick may travel
// any distance it likes; where it is ALLOWED to stop is measured from here.
static uint8_t s_dragFromPage = 0u;

// One swipe moves at most one page.
//
// Left alone, a scroller with snap points measures how fast the finger was
// going, predicts where that momentum would carry it, and snaps to whichever
// page is nearest THAT. A brisk flick therefore skips two or three pages, which
// is fine in a photo gallery and wrong here: the driver flicked to see the next
// screen, not to gamble on where five of them stop.
//
// LVGL makes this cheap to fix without fighting it. With snapping enabled the
// throw is not animated incrementally -- lv_indev_scroll_throw_handler()
// predicts the whole distance in one go, finds the snap point, and starts a
// SINGLE animation (see lv_indev_scroll.c). That animation is handed to
// LV_EVENT_SCROLL_BEGIN as a modifiable lv_anim_t*, so clamping its end value
// here changes where the page settles BEFORE a pixel of it is drawn. No
// overshoot, no visible snap-back, and the drag itself still tracks the finger
// exactly as before -- only the landing is constrained.
//
// The event fires with a NULL parameter for the drag, and with the animation
// for the release. Both are needed: the first is where the gesture started, the
// second is where it wants to end.
static void onScrollBegin(lv_event_t* e) {
  lv_anim_t* a = static_cast<lv_anim_t*>(lv_event_get_param(e));
  if (a == nullptr) {
    s_dragFromPage = pageFromScroll();
    // A swipe is an answer: the driver has decided to be somewhere else, which
    // is not a reset. Dismissed on the DRAG, not on the settle, so the dialog
    // is gone the instant the page starts moving under the finger.
    hideConfirm();
    return;
  }

  // The animation runs in NEGATED scroll coordinates: lv_obj_scroll_by() sets
  // its values to (-scroll_x, -scroll_x + dx), and scroll_x_anim() drives the
  // object to -v. So the position this animation will settle at is -end_value,
  // and writing a target back means negating it again.
  const int32_t pageW = LCD_H_RES;
  int32_t lo = (static_cast<int32_t>(s_dragFromPage) - 1) * pageW;
  int32_t hi = (static_cast<int32_t>(s_dragFromPage) + 1) * pageW;
  const int32_t maxX = static_cast<int32_t>(UI_SCREEN_COUNT - 1) * pageW;
  if (lo < 0) {
    lo = 0;
  }
  if (hi > maxX) {
    hi = maxX;
  }

  int32_t target = -a->end_value;
  if (target < lo) {
    target = lo;
  } else if (target > hi) {
    target = hi;
  }
  a->end_value = -target;
}

static void onScrollEnd(lv_event_t* e) {
  (void)e;
  const uint8_t now = pageFromScroll();
  if (now != s_current) {
    s_current = now;
    setActiveDot(s_current);
  }
}

// ---------------------------------------------------------------------------
// Live values
// ---------------------------------------------------------------------------

// Writes a slot's value, or a dimmed placeholder when it is stale or absent.
// Nothing here is allowed to hold a plausible-looking number past its TTL: a
// frozen reading is the failure mode that gets believed.
// Write a label ONLY when it would actually change.
//
// lv_label_set_text() and the style setters invalidate the widget
// unconditionally, so refreshing every value on every tick made LVGL redraw
// every readout ten times a second even on a parked car with nothing moving --
// fill and blend work competing with the scroll animation for the same draw
// unit. Comparing first is far cheaper than the redraw it avoids.
static void setTextIfChanged(lv_obj_t* lbl, const char* text) {
  const char* cur = lv_label_get_text(lbl);
  if (cur != nullptr && strcmp(cur, text) == 0) {
    return;
  }
  lv_label_set_text(lbl, text);
}

static void setColourIfChanged(lv_obj_t* lbl, lv_color_t c) {
  const lv_color_t cur = lv_obj_get_style_text_color(lbl, LV_PART_MAIN);
  if (lv_color_eq(cur, c)) {
    return;
  }
  lv_obj_set_style_text_color(lbl, c, 0);
}

// Modules answer 0xFF / 0xFFFF for "signal not available" -- the engine-off
// value for speed and RPM. Scaled, 0xFFFF/128 becomes a perfectly plausible
// 512 km/h, which is exactly the kind of number that gets believed. Treat an
// all-ones raw as no reading at all.
static bool rawIsSentinel(uint32_t raw, uint8_t bitLength) {
  if (bitLength == 0u || bitLength >= 32u) {
    return false;
  }
  return raw == ((1u << bitLength) - 1u);
}

static bool slotUsable(int8_t slot) {
  if (slot < 0 || s_state == nullptr) {
    return false;
  }
  const StateSlot* sl = s_state->slot(slot);
  return sl != nullptr && sl->everSet && sl->def != nullptr &&
         !rawIsSentinel(sl->raw, sl->def->bitLength);
}

static void setVal(lv_obj_t* lbl, int8_t slot, uint8_t decimals, uint32_t now) {
  if (lbl == nullptr) {
    return;
  }
  if (!slotUsable(slot)) {
    setTextIfChanged(lbl, "--");
    setColourIfChanged(lbl, UI_FAINT);
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%.*f", static_cast<int>(decimals),
           static_cast<double>(s_state->slot(slot)->value));
  setTextIfChanged(lbl, buf);
  // HOLD, then DIM. A value that stops arriving keeps its last reading rather
  // than blanking -- a gap in a 5 s poll cycle is normal and blinking dashes
  // would be worse than useless. But it goes dim once it is older than the TTL,
  // so a held number never passes for a live one.
  setColourIfChanged(lbl, s_state->fresh(slot, now) ? UI_PRIMARY : UI_DIM);
}

// As setVal, but scales on the way out. The published value stays the value the
// bus sent; only the presentation converts.
//
// `ttlMs` exists because the default 5 s is a property of the DEFAULT poll
// rate, not of every signal. A value polled every 30 s judged against a 5 s TTL
// is marked stale five sixths of the time, and a reading as fresh as its poll
// rate allows is not a stale reading -- it just makes the dimming meaningless.
//
// `live` and `stale` are passed as a pair so a row that is deliberately
// secondary can stay secondary AND still visibly dim: one colour alone would
// make the two states identical on any row that is not primary to begin with.
static void setValScaled(lv_obj_t* lbl, int8_t slot, uint8_t decimals, uint32_t now,
                         float scale, uint32_t ttlMs = VS_DEFAULT_TTL_MS,
                         lv_color_t live = UI_PRIMARY, lv_color_t stale = UI_DIM) {
  if (lbl == nullptr) {
    return;
  }
  if (!slotUsable(slot)) {
    setTextIfChanged(lbl, "--");
    setColourIfChanged(lbl, UI_FAINT);
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%.*f", static_cast<int>(decimals),
           static_cast<double>(s_state->slot(slot)->value * scale));
  setTextIfChanged(lbl, buf);
  setColourIfChanged(lbl, s_state->fresh(slot, now, ttlMs) ? live : stale);
}

// A value that came off the bus rather than out of a request. Same hold-and-dim
// contract as setVal: keep the last reading, dim it once it is stale.
static void setBcast(lv_obj_t* lbl, float value, uint32_t stampMs, uint8_t decimals,
                     uint32_t now) {
  if (lbl == nullptr) {
    return;
  }
  if (stampMs == 0u) {
    setTextIfChanged(lbl, "--");
    setColourIfChanged(lbl, UI_FAINT);
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%.*f", static_cast<int>(decimals),
           static_cast<double>(value));
  setTextIfChanged(lbl, buf);
  setColourIfChanged(lbl, ((now - stampMs) < VS_DEFAULT_TTL_MS) ? UI_PRIMARY : UI_DIM);
}

// A derived figure that is not always computable. The trip readouts return a
// negative when there is not yet enough to say -- no odometer anchor, no fuel
// integrated, under 100 m covered -- and that is a different thing from zero.
// "0.0 L/100" on a car that has not moved is a lie; a dash is not.
static void setOptional(lv_obj_t* lbl, float value, uint8_t decimals) {
  if (lbl == nullptr) {
    return;
  }
  if (value < 0.0f) {
    setTextIfChanged(lbl, "--");
    setColourIfChanged(lbl, UI_FAINT);
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%.*f", static_cast<int>(decimals),
           static_cast<double>(value));
  setTextIfChanged(lbl, buf);
  setColourIfChanged(lbl, UI_PRIMARY);
}

// Hours and minutes. Seconds would be one more digit changing every second on a
// screen the driver glances at, for information nobody uses.
static void setDuration(lv_obj_t* lbl, uint32_t ms) {
  if (lbl == nullptr) {
    return;
  }
  const uint32_t mins = ms / 60000u;
  char buf[12];
  snprintf(buf, sizeof(buf), "%u:%02u", static_cast<unsigned>(mins / 60u),
           static_cast<unsigned>(mins % 60u));
  setTextIfChanged(lbl, buf);
  setColourIfChanged(lbl, (mins == 0u) ? UI_DIM : UI_PRIMARY);
}

static void refreshTrip(void) {
  if (s_trips == nullptr || s_tripShown >= TRIP_COUNT) {
    return;
  }
  const Trip& t = s_trips->t[s_tripShown];
  setOptional(W.tripKm, tripKm(t), 1);
  setOptional(W.tripUsed, tripLitres(t), 2);
  setOptional(W.tripL100, tripL100(t), 1);
  setOptional(W.tripAvgKph, tripAvgKph(t), 0);
  setDuration(W.tripMoving, t.movingMs);
}

// The flow bar. Only the segments whose state actually changed are touched --
// recolouring sixteen objects ten times a second would invalidate a strip of
// the screen on every tick for a signal that moves in steps of a sixteenth.
static uint8_t s_flowLit = 0xFFu;

// Which unit the instantaneous readout is currently in. Latched rather than
// recomputed from speed alone, so the hysteresis band above has something to
// hold on to. Starts false: a car that has not moved yet is stopped.
static bool s_flowPerDistance = false;

static void setFlow(uint8_t lit) {
  if (lit == s_flowLit) {
    return;
  }
  for (uint8_t i = 0; i < UI_FLOW_SEGS; ++i) {
    if (W.flowSeg[i] != nullptr) {
      lv_obj_set_style_bg_color(W.flowSeg[i], (i < lit) ? UI_PRIMARY : UI_DOT_OFF, 0);
    }
  }
  s_flowLit = lit;
}

static void refreshDrive(uint32_t now) {
  // The headline is FROM START, not TRIP A: it answers "how is this journey
  // going", which is the question a driver actually has, and it needs no reset
  // to mean that.
  const float l100 = (s_trips != nullptr) ? tripL100(s_trips->t[TRIP_START]) : -1.0f;
  setOptional(W.l100, l100, 1);

  // Litres rather than percent, and through setValScaled so it keeps the
  // hold-and-dim contract -- fuel level is polled, so it goes stale like any
  // other requested value and must not pass for live when it does.
  setValScaled(W.tank, s_slot.fuel, 1, now, UI_TANK_LITRES / 100.0f);

  const bool haveFuel = slotUsable(s_slot.fuel);
  const float litres =
      haveFuel ? s_state->slot(s_slot.fuel)->value * (UI_TANK_LITRES / 100.0f) : -1.0f;

  // Range needs both, and says nothing until the average is real -- which is
  // honest: early in a journey there is no basis for the figure at all.
  setOptional(W.range, (haveFuel && l100 > 0.0f) ? (litres * 100.0f / l100) : -1.0f, 0);

  // Both the bar and the number come from the SMOOTHED rate, not from the last
  // raw frame. Q is a per-stroke control signal and is very spiky; an
  // unsmoothed readout at 10 Hz is a blur, and an unsmoothed bar is a flicker.
  const float lph = (s_trips != nullptr) ? tripFlowLph(*s_trips, now) : -1.0f;

  uint8_t lit = 0u;
  if (lph > 0.0f) {
    const float f = lph * UI_FLOW_SEGS / UI_FLOW_FULL_LPH + 0.5f;
    lit = (f >= (float)UI_FLOW_SEGS) ? UI_FLOW_SEGS : (uint8_t)f;
  }
  setFlow(lit);

  if (W.flowNow != nullptr) {
    if (lph < 0.0f) {
      setTextIfChanged(W.flowNow, "--");
      setColourIfChanged(W.flowNow, UI_FAINT);
    } else {
      // Moving, show what the journey is costing per distance; stopped or
      // crawling, show what the engine is drinking per hour.
      //
      // No live speed means L/h as well: without a speed there is no distance
      // to divide by, and L/h is the one of the two that is still true.
      const bool speedLive = (s_bcast != nullptr) && (s_bcast->speedMs != 0u) &&
                             ((now - s_bcast->speedMs) < VS_DEFAULT_TTL_MS);
      if (!speedLive || s_bcast->speedKph < UI_FLOW_TO_LPH_KPH) {
        s_flowPerDistance = false;
      } else if (s_bcast->speedKph >= UI_FLOW_TO_L100_KPH) {
        s_flowPerDistance = true;
      }

      char buf[20];
      if (s_flowPerDistance) {
        snprintf(buf, sizeof(buf), "%.1f L/100",
                 static_cast<double>(lph * 100.0f / s_bcast->speedKph));
      } else {
        snprintf(buf, sizeof(buf), "%.1f L/h", static_cast<double>(lph));
      }
      setTextIfChanged(W.flowNow, buf);
      setColourIfChanged(W.flowNow, UI_PRIMARY);
    }
  }
}

// The low-pressure line, road wheels only.
static void refreshTyreWarning(void) {
  if (W.tyreLow == nullptr || W.tyreTarget == nullptr) {
    return;
  }
  uint8_t low = 0u;
  for (uint8_t i = 0; i < 4; ++i) {
    if (slotUsable(s_slot.tyre[i]) &&
        s_state->slot(s_slot.tyre[i])->value * UI_PSI_TO_BAR < UI_TYRE_LOW_BAR) {
      ++low;
    }
  }
  if (low == 0u) {
    lv_obj_add_flag(W.tyreLow, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(W.tyreTarget, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  char buf[12];
  snprintf(buf, sizeof(buf), "%u LOW", static_cast<unsigned>(low));
  setTextIfChanged(W.tyreLow, buf);
  lv_obj_remove_flag(W.tyreLow, LV_OBJ_FLAG_HIDDEN);
  lv_obj_remove_flag(W.tyreTarget, LV_OBJ_FLAG_HIDDEN);
}

// Runs the dialog's five seconds down and closes it when they are gone.
//
// Driven off the UI tick rather than an lv_timer so there is one clock in this
// file and nothing to cancel: an open dialog is entirely described by the tick
// it opened at, and closing it is one assignment.
static void refreshConfirm(void) {
  if (s_confirmStart == 0u) {
    return;
  }
  const uint32_t elapsed = lv_tick_elaps(s_confirmStart);
  if (elapsed >= UI_CONFIRM_MS) {
    hideConfirm();
    return;
  }
  if (W.confirmBar != nullptr) {
    const int32_t full = 184;
    int32_t w = full - (int32_t)((uint64_t)full * elapsed / UI_CONFIRM_MS);
    if (w < 1) {
      w = 1;
    }
    if (lv_obj_get_width(W.confirmBar) != w) {
      lv_obj_set_width(W.confirmBar, w);
    }
  }
}

static void refresh(uint32_t now) {
  // Broadcast first: 0x002 lands every 50 ms where a poll of the same values
  // costs a transaction and arrives once or twice a second.
  if (s_bcast != nullptr) {
    setBcast(W.speed, s_bcast->speedKph, s_bcast->speedMs, 0, now);
    setBcast(W.rpm, (float)s_bcast->rpm, s_bcast->rpmMs, 0, now);
    setBcast(W.healthVolts, s_bcast->volts, s_bcast->voltsMs, 1, now);
    setBcast(W.healthCoolant, (float)s_bcast->coolantC, s_bcast->coolantMs, 0, now);
    setBcast(W.healthAmbient, s_bcast->ambientC, s_bcast->ambientMs, 0, now);
  } else {
    setVal(W.speed, s_slot.speed, 0, now);
    setVal(W.rpm, s_slot.rpm, 0, now);
    setVal(W.healthVolts, s_slot.volts, 1, now);
    setVal(W.healthCoolant, s_slot.coolant, 0, now);
    setVal(W.healthAmbient, s_slot.ambient, 0, now);
  }
  refreshDrive(now);
  refreshTrip();
  // Both against TTLs matched to their own poll rates: the corners are asked
  // for every 5 s and the spare every 30 s, so the default 5 s TTL would have
  // the corners flickering on the boundary and the spare permanently grey.
  for (uint8_t i = 0; i < 4; ++i) {
    setValScaled(W.tyre[i], s_slot.tyre[i], 2, now, UI_PSI_TO_BAR, 12000u);
  }
  setValScaled(W.spare, s_slot.spare, 2, now, UI_PSI_TO_BAR, 60000u, UI_DIM, UI_FAINT);
  refreshTyreWarning();
  refreshConfirm();

  // Uptime stands in for a clock until the RTC is wired.
  const uint32_t secs = now / 1000u;
  char t[8];
  snprintf(t, sizeof(t), "%02u:%02u", static_cast<unsigned>((secs / 60u) % 100u),
           static_cast<unsigned>(secs % 60u));
  if (W.clockLbl != nullptr) {
    setTextIfChanged(W.clockLbl, t);
  }
}

static void uiTask(void*) {
  for (;;) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    // Sampled inside the lock, PRINTED outside it. ESP_LOGI goes to
    // USB-Serial-JTAG, which blocks until the host drains its buffer -- so with
    // a monitor attached this line was holding the LVGL lock across a host
    // round trip and stalling the renderer. The instrument measuring the frame
    // rate was lowering it.
    uint32_t span = 0u, cycles = 0u, rends = 0u, us = 0u;

    if (lvgl_port_lock(100)) {
      if (s_splashDone) {
        refresh(now);
        static uint32_t lastDiag = 0u;
        if (now - lastDiag >= 2000u) {
          span = now - lastDiag;
          lastDiag = now;
          cycles = s_refrCycles;
          rends = s_renders;
          us = s_renderUs;
          s_refrCycles = 0u;
          s_renders = 0u;
          s_renderUs = 0u;
        }
        // NO idle timeout. The UI stays on whatever page it was left on.
        //
        // Snapping back to DRIVE after a pause sounds helpful and is not: the
        // driver put the screen on TYRES deliberately, looked at the road, and
        // found a different page waiting. A page only changes when a finger
        // changes it.
      }
      lvgl_port_unlock();
    }

    if (span != 0u) {
      ESP_LOGI(TAG, "refr %u/s  rendered %u/s  avg render %uus  page=%u",
               (unsigned)(cycles * 1000u / span), (unsigned)(rends * 1000u / span),
               (unsigned)(rends ? us / rends : 0u), (unsigned)s_current);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ---------------------------------------------------------------------------
// Welcome screen
// ---------------------------------------------------------------------------

static void buildSplash() {
  s_splash = mkScreen();

  // The WK's own face: seven short slots in a surround, between twin-circle
  // lamp clusters. Drawn with primitives rather than an image, so it costs no
  // flash and follows the palette.
  const lv_coord_t gx = 60;
  const lv_coord_t gy = 128;
  const lv_coord_t gw = 120;
  const lv_coord_t gh = 30;
  mkOutline(s_splash, gx, gy, gw, gh, 4, UI_PRIMARY, 2);
  for (uint8_t i = 0; i < 7; ++i) {
    lv_obj_t* slot = lv_obj_create(s_splash);
    stripChrome(slot);
    lv_obj_set_size(slot, 12, 22);
    lv_obj_set_pos(slot, static_cast<lv_coord_t>(gx + 5 + i * 16),
                   static_cast<lv_coord_t>(gy + 4));
    lv_obj_set_style_bg_color(slot, UI_PRIMARY, 0);
    lv_obj_set_style_bg_opa(slot, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(slot, 2, 0);
  }

  // Lamp clusters: outer circle slightly larger than inner, bottoms near-level.
  const lv_coord_t cy = static_cast<lv_coord_t>(gy + gh / 2);
  static const lv_coord_t kLampCx[4] = {18, 41, 199, 222};
  static const lv_coord_t kLampR[4] = {16, 14, 14, 16};
  for (uint8_t i = 0; i < 4; ++i) {
    const lv_coord_t d = static_cast<lv_coord_t>(kLampR[i] * 2);
    mkOutline(s_splash, static_cast<lv_coord_t>(kLampCx[i] - kLampR[i]),
              static_cast<lv_coord_t>(cy - kLampR[i]), d, d, LV_RADIUS_CIRCLE,
              UI_PRIMARY, 2);
  }

  mkRule(s_splash, 250, (LCD_H_RES - 48) / 2, 48);
  lv_obj_t* name = mkLabel(s_splash, "LATONITA-CARPUTER", UI_FONT_LABEL, UI_DIM, 0, 0);
  lv_obj_set_style_text_letter_space(name, 2, 0);
  lv_obj_align(name, LV_ALIGN_TOP_MID, 0, 262);
  lv_obj_t* ver = mkLabel(s_splash, "v1.0", UI_FONT_LABEL, UI_FAINT, 0, 0);
  lv_obj_align(ver, LV_ALIGN_TOP_MID, 0, 282);
}

void uiSetBroadcast(const BroadcastState* bcast) {
  s_bcast = bcast;
}

void uiSetTrips(const TripSet* trips, void (*resetFn)(uint8_t)) {
  s_trips = trips;
  s_tripReset = resetFn;
}

void uiLinkReady(bool ok) {
  if (!lvgl_port_lock(200)) {
    return;
  }
  if (W.linkDot != nullptr) {
    lv_obj_set_style_bg_color(W.linkDot, ok ? UI_PRIMARY : UI_CRIT, 0);
  }
  lvgl_port_unlock();
}

void uiSplash(uint32_t holdMs) {
  vTaskDelay(pdMS_TO_TICKS(holdMs));
  if (lvgl_port_lock(200)) {
    lv_screen_load_anim(s_root, LV_SCREEN_LOAD_ANIM_FADE_IN, 400, 0, false);
    s_current = UI_HOME;
    s_splashDone = true;
    ESP_LOGI(TAG, "splash -> root, active=%p", (void*)lv_screen_active());
    lvgl_port_unlock();
  }
}

// ---------------------------------------------------------------------------

esp_err_t uiInit(VehicleState* state) {
  s_state = state;
  memset(&W, 0, sizeof(W));

  lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
  // Rendering gets its own core. The link task blocks on UART reads for up to
  // 400 ms at a HIGHER priority than LVGL, so on a shared core a poll burst
  // stalls the display outright -- which reads as a low frame rate rather than
  // as the priority inversion it is. The S3 has two cores; use them.
  port_cfg.task_affinity = 1;
  port_cfg.timer_period_ms = 5;
  esp_err_t err = lvgl_port_init(&port_cfg);
  if (err != ESP_OK) {
    return err;
  }

  lvgl_port_display_cfg_t disp_cfg = {};
  disp_cfg.io_handle = displayPanelIo();
  disp_cfg.panel_handle = displayPanel();
  disp_cfg.buffer_size = static_cast<uint32_t>(LCD_H_RES) * UI_BUF_ROWS;
  disp_cfg.double_buffer = true;
  disp_cfg.hres = LCD_H_RES;
  disp_cfg.vres = LCD_V_RES;
  disp_cfg.monochrome = false;
  // Display.cpp already mirrored the panel; repeating it here cancels it out.
  disp_cfg.rotation.swap_xy = false;
  disp_cfg.rotation.mirror_x = false;
  disp_cfg.rotation.mirror_y = false;
  disp_cfg.flags.buff_dma = true;
  disp_cfg.flags.swap_bytes = true;

  s_disp = lvgl_port_add_disp(&disp_cfg);
  if (s_disp == nullptr) {
    return ESP_FAIL;
  }

  // Exact names: VehicleState::indexOfName does not accept the short form.
  if (state != nullptr) {
    s_slot.coolant = state->indexOfName("CCN.Engine Coolant Temperature");
    s_slot.volts = state->indexOfName("CCN.System Voltage");
    s_slot.speed = state->indexOfName("CCN.Vehicle Speed");
    s_slot.rpm = state->indexOfName("CCN.Engine Revolutions Per Minute");
    s_slot.fuel = state->indexOfName("CCN.Fuel Level Percentage");
    s_slot.ambient = state->indexOfName("CCN.Ambient Temperature");
    s_slot.tyre[0] = state->indexOfName("WCM.Tyre Sensor 1");
    s_slot.tyre[1] = state->indexOfName("WCM.Tyre Sensor 2");
    s_slot.tyre[2] = state->indexOfName("WCM.Tyre Sensor 3");
    s_slot.tyre[3] = state->indexOfName("WCM.Tyre Sensor 4");
    s_slot.spare = state->indexOfName("WCM.Tyre Spare");
  } else {
    s_slot.coolant = -1;
    s_slot.volts = -1;
    s_slot.speed = -1;
    s_slot.rpm = -1;
    s_slot.fuel = -1;
    s_slot.ambient = -1;
    for (uint8_t i = 0; i < 4; ++i) {
      s_slot.tyre[i] = -1;
    }
    s_slot.spare = -1;
  }

  if (!lvgl_port_lock(0)) {
    return ESP_FAIL;
  }

  s_root = mkScreen();

  // The scroller: a flex row of full-width pages with centre snapping, so a
  // drag moves the pages under the finger and release settles on the nearest.
  s_pages = lv_obj_create(s_root);
  stripChrome(s_pages);
  lv_obj_set_size(s_pages, LCD_H_RES, UI_PAGE_H);
  lv_obj_set_pos(s_pages, 0, UI_PAGE_Y);
  lv_obj_set_flex_flow(s_pages, LV_FLEX_FLOW_ROW);
  lv_obj_set_style_pad_column(s_pages, 0, 0);
  // CLICKABLE as well as SCROLLABLE, and the pages below are clickable too.
  // LVGL hit-tests for a clickable object to decide what was pressed, then
  // walks UP that object's parents to find something scrollable. Strip
  // clickability from everything and there is no press target at all, so the
  // scroller never learns a finger is on it -- the drag simply vanishes.
  lv_obj_add_flag(s_pages, static_cast<lv_obj_flag_t>(LV_OBJ_FLAG_SCROLLABLE |
                                                      LV_OBJ_FLAG_CLICKABLE));
  lv_obj_set_scroll_dir(s_pages, LV_DIR_HOR);
  lv_obj_set_scroll_snap_x(s_pages, LV_SCROLL_SNAP_CENTER);
  lv_obj_set_scrollbar_mode(s_pages, LV_SCROLLBAR_MODE_OFF);
  lv_obj_add_event_cb(s_pages, onScrollBegin, LV_EVENT_SCROLL_BEGIN, nullptr);
  lv_obj_add_event_cb(s_pages, onScrollEnd, LV_EVENT_SCROLL_END, nullptr);

  for (uint8_t i = 0; i < UI_SCREEN_COUNT; ++i) {
    s_page[i] = lv_obj_create(s_pages);
    stripChrome(s_page[i]);
    // OPAQUE, not transparent.
    //
    // stripChrome() leaves objects transparent, which is right for a 1 px rule
    // but wrong for a full-screen page: LVGL then has to paint the root and the
    // scroller underneath and blend the page over both, every frame. A scroll
    // dirties the whole screen, so that overdraw lands on exactly the frames
    // that can least afford it. An opaque page lets the renderer skip whatever
    // is behind it.
    lv_obj_set_style_bg_color(s_page[i], UI_BG, 0);
    lv_obj_set_style_bg_opa(s_page[i], LV_OPA_COVER, 0);
    lv_obj_set_size(s_page[i], LCD_H_RES, UI_PAGE_H);
    // The press target. Decorations stay non-clickable so they fall through to
    // here, and from here LVGL finds the scroller one level up.
    lv_obj_add_flag(s_page[i], LV_OBJ_FLAG_CLICKABLE);
  }
  buildTrip(s_page[UI_TRIP]);
  buildDrive(s_page[UI_DRIVE]);
  buildTyres(s_page[UI_TYRES]);
  buildHealth(s_page[UI_HEALTH]);

  // Chrome last, so it paints over the pages sliding beneath it.
  buildStatusBar(s_root);
  buildDots(s_root);
  buildConfirm(s_root);   // last of all: nothing may paint over a dialog

  lv_display_add_event_cb(s_disp, onRefrStart, LV_EVENT_REFR_START, nullptr);
  lv_display_add_event_cb(s_disp, onRenderReady, LV_EVENT_RENDER_READY, nullptr);

  buildSplash();
  lv_screen_load(s_splash);

  // Park the scroller on HOME before anything is shown.
  //
  // TRIP is index 0, so leaving the scroller at its natural resting place would
  // boot onto TRIP. The layout has to be resolved first or the scroll offset is
  // computed against zero-width pages and silently does nothing.
  lv_obj_update_layout(s_root);
  lv_obj_scroll_to_x(s_pages, UI_HOME * LCD_H_RES, LV_ANIM_OFF);
  s_current = UI_HOME;
  setActiveDot(UI_HOME);

  // Geometry diagnostics: a blank panel and a mislaid layout look identical
  // from the driver's seat, and these three numbers tell them apart.
  ESP_LOGI(TAG, "root %dx%d  scroller %dx%d at y=%d  page0 %dx%d  content_w=%d",
           (int)lv_obj_get_width(s_root), (int)lv_obj_get_height(s_root),
           (int)lv_obj_get_width(s_pages), (int)lv_obj_get_height(s_pages),
           (int)lv_obj_get_y(s_pages),
           (int)lv_obj_get_width(s_page[0]), (int)lv_obj_get_height(s_page[0]),
           (int)lv_obj_get_content_width(s_pages));
  ESP_LOGI(TAG, "scroll left=%d right=%d  active=%p splash=%p root=%p",
           (int)lv_obj_get_scroll_left(s_pages), (int)lv_obj_get_scroll_right(s_pages),
           (void*)lv_screen_active(), (void*)s_splash, (void*)s_root);
  lvgl_port_unlock();

  if (xTaskCreatePinnedToCore(uiTask, "ui", 4096, nullptr, 3, nullptr, 1) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }
  ESP_LOGI(TAG, "dashboard up: %u pages", static_cast<unsigned>(UI_SCREEN_COUNT));
  return ESP_OK;
}
