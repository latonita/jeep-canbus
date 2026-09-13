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

// Return to DRIVE after this long untouched, so a glance down always lands
// somewhere predictable.
#define UI_IDLE_HOME_MS 30000u

// WCM 21 3B reports one psi per bit, with no offset and no scaling. The cabin
// reads in bar, so the conversion lives here rather than in the decoder: the
// wire value stays the wire value, and only the presentation changes.
//
// For scale: the owner's stated target band of 2.1-2.3 bar is 30.5-33.4 psi,
// and the recorded corners (30-34 psi) are 2.07-2.34 bar.
#define UI_PSI_TO_BAR 0.0689476f

// Chrome heights. Pages occupy what is left between them.
#define UI_PAGE_Y (UI_STATUSBAR_H)
#define UI_PAGE_H (LCD_V_RES - UI_STATUSBAR_H)

static VehicleState* s_state = nullptr;
static const BroadcastState* s_bcast = nullptr;
static lv_display_t* s_disp = nullptr;
static lv_obj_t* s_root = nullptr;
static lv_obj_t* s_pages = nullptr;   // the horizontal scroller
static lv_obj_t* s_page[UI_SCREEN_COUNT];
static lv_obj_t* s_dot[UI_SCREEN_COUNT];
static lv_obj_t* s_splash = nullptr;
static uint8_t s_current = UI_DRIVE;
static bool s_splashDone = false;

// Cached VehicleState slot indices, resolved once at init. -1 means the signal
// is not in the poll plan, which is a normal condition, not an error.
static struct {
  int8_t coolant, volts, speed, rpm, fuel, ambient;
  int8_t tyre[4];
} s_slot;

// Live widgets.
static struct {
  lv_obj_t* linkDot;
  lv_obj_t* ign;
  lv_obj_t* clockLbl;
  lv_obj_t* coolant;
  lv_obj_t* volts;
  lv_obj_t* speed;
  lv_obj_t* rpm;
  lv_obj_t* fuel;
  lv_obj_t* healthVolts;
  lv_obj_t* tyre[4];
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
                     (i == UI_DRIVE) ? UI_PRIMARY : UI_DOT_OFF);
  }
}

static void setActiveDot(uint8_t active) {
  for (uint8_t i = 0; i < UI_SCREEN_COUNT; ++i) {
    if (s_dot[i] != nullptr) {
      lv_obj_set_style_bg_color(s_dot[i], (i == active) ? UI_PRIMARY : UI_DOT_OFF, 0);
    }
  }
}

// Page title plus an optional right-aligned freshness note.
static void buildHeader(lv_obj_t* page, const char* title, const char* note) {
  lv_obj_t* t = mkLabel(page, title, UI_FONT_LABEL, UI_PRIMARY, UI_PAD - 2, 5);
  lv_obj_set_style_text_letter_space(t, 3, 0);
  if (note != nullptr) {
    lv_obj_t* n = mkLabel(page, note, UI_FONT_LABEL, UI_LABEL, 0, 0);
    lv_obj_align(n, LV_ALIGN_TOP_RIGHT, -(UI_PAD - 2), 5);
  }
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

static void buildDrive(lv_obj_t* page) {
  mkCaption(page, "SPEED", UI_PAD - 2, 6);
  W.speed = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_PAD - 4, 22);
  mkLabel(page, "km/h", UI_FONT_LABEL, UI_DIM, 158, 66);

  buildPair(page, 106, "COOLANT", "BATTERY", &W.coolant, &W.volts, "C", "V");
  buildPair(page, 170, "ENGINE", "FUEL", &W.rpm, &W.fuel, "rpm", "%");
  mkRule(page, 234, 0, LCD_H_RES);
}

static void buildTyres(lv_obj_t* page) {
  buildHeader(page, "TYRES", "SENSOR ORDER UNMAPPED");

  // Corner order is unmapped, so cells are labelled by sensor index. The
  // mapping wizard rewrites these captions once it has been run.
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
  mkCaption(page, "TARGET 2.1 - 2.3 bar", UI_PAD - 2, 188);
}

static void buildHealth(lv_obj_t* page) {
  buildHeader(page, "HEALTH", nullptr);

  W.healthVolts = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_PAD - 4, 30);
  mkLabel(page, "V", UI_FONT_LABEL, UI_DIM, 158, 74);

  mkRule(page, 106, 0, LCD_H_RES);
  mkCaption(page, "LAMPS", UI_PAD - 2, 112);
  // MIL is deliberately absent: CCN 213F bit24 reads true in every recorded
  // sample including at speed, so showing it would mean a permanent warning.
  static const char* const kLamps[3] = {"OIL", "ABS", "CHARGE"};
  for (uint8_t i = 0; i < 3; ++i) {
    lv_obj_t* p = mkOutline(page, static_cast<lv_coord_t>(UI_PAD - 2 + i * 73), 132, 68,
                            26, 0, UI_RULE, 1);
    lv_obj_t* t = mkLabel(p, kLamps[i], UI_FONT_LABEL, UI_FAINT, 0, 0);
    lv_obj_center(t);
  }

  mkRule(page, 172, 0, LCD_H_RES);
  mkCaption(page, "FAULTS", UI_PAD - 2, 178);
  mkLabel(page, "not scanned", UI_FONT_LABEL, UI_DIM, UI_PAD - 2, 198);
}

static void buildTrip(lv_obj_t* page) {
  buildHeader(page, "TRIP A", nullptr);

  mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_PAD - 4, 30);
  mkLabel(page, "km", UI_FONT_LABEL, UI_DIM, 158, 74);

  lv_obj_t* used = nullptr;
  lv_obj_t* avg = nullptr;
  buildPair(page, 114, "USED", "AVERAGE", &used, &avg, "L", "L/100");
  lv_obj_t* moving = nullptr;
  lv_obj_t* avgSpeed = nullptr;
  buildPair(page, 178, "MOVING", "AVG SPEED", &moving, &avgSpeed, nullptr, "km/h");
}

static void buildTilt(lv_obj_t* page) {
  buildHeader(page, "TILT", "IMU NOT WIRED");

  mkRule(page, 40, 0, LCD_H_RES);
  mkCaption(page, "ROLL", 30, 48);
  mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, 14, 66);
  mkCaption(page, "PITCH", 150, 48);
  mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, 134, 66);
  mkRule(page, 144, 0, LCD_H_RES);
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

static void onScrollEnd(lv_event_t* e) {
  (void)e;
  const uint8_t now = pageFromScroll();
  if (now != s_current) {
    s_current = now;
    setActiveDot(s_current);
  }
}

static void goTo(uint8_t id) {
  if (s_pages == nullptr || id >= UI_SCREEN_COUNT) {
    return;
  }
  lv_obj_scroll_to_x(s_pages, static_cast<int32_t>(id) * LCD_H_RES, LV_ANIM_ON);
  s_current = id;
  setActiveDot(id);
}

// ---------------------------------------------------------------------------
// Live values
// ---------------------------------------------------------------------------

// Writes a slot's value, or a dimmed placeholder when it is stale or absent.
// Nothing here is allowed to hold a plausible-looking number past its TTL: a
// frozen reading is the failure mode that gets believed.
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
    lv_label_set_text(lbl, "--");
    lv_obj_set_style_text_color(lbl, UI_FAINT, 0);
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%.*f", static_cast<int>(decimals),
           static_cast<double>(s_state->slot(slot)->value));
  lv_label_set_text(lbl, buf);
  // HOLD, then DIM. A value that stops arriving keeps its last reading rather
  // than blanking -- a gap in a 5 s poll cycle is normal and blinking dashes
  // would be worse than useless. But it goes dim once it is older than the TTL,
  // so a held number never passes for a live one.
  lv_obj_set_style_text_color(lbl, s_state->fresh(slot, now) ? UI_PRIMARY : UI_DIM, 0);
}

// As setVal, but scales on the way out. The published value stays the value the
// bus sent; only the presentation converts.
static void setValScaled(lv_obj_t* lbl, int8_t slot, uint8_t decimals, uint32_t now,
                         float scale) {
  if (lbl == nullptr) {
    return;
  }
  if (!slotUsable(slot)) {
    lv_label_set_text(lbl, "--");
    lv_obj_set_style_text_color(lbl, UI_FAINT, 0);
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%.*f", static_cast<int>(decimals),
           static_cast<double>(s_state->slot(slot)->value * scale));
  lv_label_set_text(lbl, buf);
  lv_obj_set_style_text_color(lbl, s_state->fresh(slot, now) ? UI_PRIMARY : UI_DIM, 0);
}

// A value that came off the bus rather than out of a request. Same hold-and-dim
// contract as setVal: keep the last reading, dim it once it is stale.
static void setBcast(lv_obj_t* lbl, float value, uint32_t stampMs, uint8_t decimals,
                     uint32_t now) {
  if (lbl == nullptr) {
    return;
  }
  if (stampMs == 0u) {
    lv_label_set_text(lbl, "--");
    lv_obj_set_style_text_color(lbl, UI_FAINT, 0);
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%.*f", static_cast<int>(decimals),
           static_cast<double>(value));
  lv_label_set_text(lbl, buf);
  lv_obj_set_style_text_color(
      lbl, ((now - stampMs) < VS_DEFAULT_TTL_MS) ? UI_PRIMARY : UI_DIM, 0);
}

static void refresh(uint32_t now) {
  // Broadcast first: 0x002 lands every 50 ms where a poll of the same values
  // costs a transaction and arrives once or twice a second.
  if (s_bcast != nullptr) {
    setBcast(W.speed, s_bcast->speedKph, s_bcast->speedMs, 0, now);
    setBcast(W.rpm, (float)s_bcast->rpm, s_bcast->rpmMs, 0, now);
    setBcast(W.coolant, (float)s_bcast->coolantC, s_bcast->coolantMs, 0, now);
    setBcast(W.volts, s_bcast->volts, s_bcast->voltsMs, 1, now);
    setBcast(W.healthVolts, s_bcast->volts, s_bcast->voltsMs, 1, now);
  } else {
    setVal(W.speed, s_slot.speed, 0, now);
    setVal(W.coolant, s_slot.coolant, 0, now);
    setVal(W.volts, s_slot.volts, 1, now);
    setVal(W.healthVolts, s_slot.volts, 1, now);
    setVal(W.rpm, s_slot.rpm, 0, now);
  }
  setVal(W.fuel, s_slot.fuel, 0, now);
  for (uint8_t i = 0; i < 4; ++i) {
    setValScaled(W.tyre[i], s_slot.tyre[i], 2, now, UI_PSI_TO_BAR);
  }

  // Uptime stands in for a clock until the RTC is wired.
  const uint32_t secs = now / 1000u;
  char t[8];
  snprintf(t, sizeof(t), "%02u:%02u", static_cast<unsigned>((secs / 60u) % 100u),
           static_cast<unsigned>(secs % 60u));
  if (W.clockLbl != nullptr) {
    lv_label_set_text(W.clockLbl, t);
  }
}

static void uiTask(void*) {
  for (;;) {
    const uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    if (lvgl_port_lock(100)) {
      if (s_splashDone) {
        refresh(now);
        static uint32_t lastDiag = 0u;
        if (now - lastDiag > 5000u) {
          lastDiag = now;
          ESP_LOGI(TAG, "active=%p root=%p page=%u scroll_x=%d speed=\"%s\" tyre0slot=%d",
                   (void*)lv_screen_active(), (void*)s_root, (unsigned)s_current,
                   (int)lv_obj_get_scroll_x(s_pages),
                   W.speed ? lv_label_get_text(W.speed) : "(null)",
                   (int)s_slot.tyre[0]);
        }
        if (s_current != UI_DRIVE &&
            lv_display_get_inactive_time(s_disp) > UI_IDLE_HOME_MS) {
          goTo(UI_DRIVE);
        }
      }
      lvgl_port_unlock();
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
    s_current = UI_DRIVE;
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
  lv_obj_add_event_cb(s_pages, onScrollEnd, LV_EVENT_SCROLL_END, nullptr);

  for (uint8_t i = 0; i < UI_SCREEN_COUNT; ++i) {
    s_page[i] = lv_obj_create(s_pages);
    stripChrome(s_page[i]);
    lv_obj_set_size(s_page[i], LCD_H_RES, UI_PAGE_H);
    // The press target. Decorations stay non-clickable so they fall through to
    // here, and from here LVGL finds the scroller one level up.
    lv_obj_add_flag(s_page[i], LV_OBJ_FLAG_CLICKABLE);
  }
  buildDrive(s_page[UI_DRIVE]);
  buildTyres(s_page[UI_TYRES]);
  buildHealth(s_page[UI_HEALTH]);
  buildTrip(s_page[UI_TRIP]);
  buildTilt(s_page[UI_TILT]);

  // Chrome last, so it paints over the pages sliding beneath it.
  buildStatusBar(s_root);
  buildDots(s_root);

  buildSplash();
  lv_screen_load(s_splash);

  // Geometry diagnostics: a blank panel and a mislaid layout look identical
  // from the driver's seat, and these three numbers tell them apart.
  lv_obj_update_layout(s_root);
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
