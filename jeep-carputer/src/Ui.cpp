#include "Ui.h"

#include "Settings.h"

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

#include "Accel.h"
#include "Faults.h"
#include "SignalDb.h"
#include "Broadcast.h"
#include "Trip.h"
#include "VehicleState.h"

static const char* TAG = "ui";

// Rows LVGL renders per flush -- LCD_DRAW_ROWS, which Board.h owns because the
// SPI bus has to be sized to match it. Two of these are double-buffered, which
// is what lets a chunk render while the previous one is still on the wire. A
// full framebuffer would be 150 KB and does not fit while PSRAM is off.
#define UI_BUF_ROWS LCD_DRAW_ROWS

// How long a page takes to settle after the finger leaves.
//
// LVGL derives this from a speed encoding that resolves to 2000 ms for a move
// of a whole page and is then clamped to SCROLL_ANIM_TIME_MAX -- so EVERY page
// change took the full 400 ms, which is most of what "sluggish" was. There is
// no Kconfig for it; the duration is written directly onto the animation in
// onScrollBegin(), which LVGL resolves after that callback returns.
#define UI_PAGE_ANIM_MS 180

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

// U+20AC EURO SIGN as UTF-8, spelled in hex so this file stays pure ASCII. The
// patch tooling around this repo reads and writes source as bytes, and an
// encoding conversion that silently rewrites one character in one string is a
// class of bug nobody enjoys finding.
#define UI_EUR "\xE2\x82\xAC"

// U+00B0 DEGREE SIGN, same treatment. Both label faces carry it and always
// have; the temperature units simply never used it.
#define UI_DEG "\xC2\xB0"

// Fuel price band, in whole cents per litre. Not a guess about diesel: it is
// the range in which a price is still a price. Below 0.10 the cost readout is
// noise; above 9.99 it needs a fourth digit the row has no room for. Either end
// takes hundreds of taps to reach by accident.
#define UI_PRICE_MIN       10u
#define UI_PRICE_MAX      999u
#define UI_PRICE_STEP       1u   // one tap: a cent
#define UI_PRICE_STEP_FAST  5u   // one tick of a held press

// Bottom of the owner's stated cold band. Below this a road wheel is called
// low; the spare is excluded, since it reads about 0.5 bar and would otherwise
// hold the warning on permanently.
#define UI_TYRE_LOW_BAR 2.1f

// How long a tyre reading counts as fresh. TWICE the poll interval in kPlan,
// which is a minute for the corners and for the spare, so both tolerate one
// missed request and no more.
//
// These are a property of the POLL RATE, not of the signal: judged against the
// default 5 s TTL a value asked for once a minute is stale fifty-five seconds
// out of every sixty, and a reading as fresh as its poll rate allows is not a
// stale reading. It just makes the dimming mean nothing, on the page where the
// dimming is the guard against believing a frozen number.
#define UI_TYRE_TTL_MS  120000u
#define UI_SPARE_TTL_MS 120000u

// ---- The flow readout ----------------------------------------------------
//
// 0x1A7 is a rate in known units -- L/h = Q/160 -- so unlike the first version
// of this screen the bar CAN be scaled to something real and CAN carry a number
// beside it.
//
// Full scale is 50 L/h. An OM642 peaks higher than that, but almost no driving
// happens up there: idle is under 1 L/h and a cruise is around 10, so a bar
// scaled to the engine's true peak would spend its life in the bottom sixth and
// show nothing useful. 50 puts a cruise at a fifth lit, leaves room for a hard
// overtake, and pegs on full throttle -- which is information in itself.
//
// It was 40 until the bar gained a critical state. The colour now carries the
// alarm, so the LENGTH is free to describe a wider range without the top of it
// being somewhere the driver never sees.
// It CLAMPS rather than auto-ranging: a bar whose full scale moves means
// something different every time the driver looks at it.
#define UI_FLOW_SEGS 16
#define UI_FLOW_FULL_LPH 50.0f

// Above this the bar goes critical.
//
// The threshold is on the PER-DISTANCE figure, not on L/h, and it only applies
// while the readout is actually in L/100 -- stopped or crawling there is no
// distance to divide by, and an idling engine would otherwise sit permanently
// in alarm for burning under a litre an hour. See s_flowPerDistance.
//
// It is a rate of SPENDING, not a fault. Nothing is wrong with the car at
// 40 L/100km; you are just paying for it, which is exactly the thing a fuel
// card exists to say out loud.
#define UI_FLOW_CRIT_L100 35.0f

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

// ---- Orientation ---------------------------------------------------------
//
// The panel is 240x320 and always will be. LANDSCAPE is that panel turned a
// quarter turn, not a second device -- so UI_W / UI_H keep meaning
// the HARDWARE, and UI_W / UI_H mean whatever the layout is being drawn
// against right now. Below this line the layout code uses only UI_W / UI_H;
// the two places that still name the panel are the buffer sizing and the
// resolution handed to esp_lvgl_port, which are properties of the glass.
//
// The rotation itself is one call. lv_display_set_rotation() swaps the
// resolution LVGL reports, esp_lvgl_port hears LV_EVENT_RESOLUTION_CHANGED and
// writes swap_xy/mirror into the ST7789, and lv_indev runs every touch point
// through lv_display_rotate_point() on the way in.
//
// That last part is the one worth remembering: the panel and the digitiser
// cannot drift apart here, because only ONE of them is ever told about the
// rotation. Touch.cpp's flags describe the digitiser, which does not turn, and
// must not be changed to "match" a rotation -- doing so would rotate the input
// twice.
#define UI_W (s_w)
#define UI_H (s_h)

// The landscape band, from design/*.landscape.dc.html.
//
// The dot rail FOLDS INTO the status bar: 26 px of dedicated rail is 8 % of
// portrait's height and would be 11 % of landscape's, spent on the only axis
// that is short. One 22 px bar carries clock, dots and link, and the pages get
// 218 px instead of 192.
#define UI_LS_STATUSBAR_H 22
// 10, not 12. Two pixels off each inset, taken because the two columns are
// fighting over a total that does not move: content = band + stack is
// (UI_W - 1 - 2*PAD) - PAD, so 271 px at PAD 12 and 279 at PAD 10, whatever
// the divider does. TRIP needs 187 of it for a 64 px hero with its unit
// beside it, and the stack needs 92 for "4163 rpm" -- 279 exactly covers both
// and 271 does not.
#define UI_LS_PAD         10
// 112, not 130. The hero column had 165 px and "247.8" in font_digits_64 is
// 164.2, so a unit on the same line had nowhere to be -- which is why the unit
// used to be a KEY parked in the header. Moving it down to the baseline means
// the column has to hold both, and 187 px does. Every value the narrowed stack
// still has to show was MEASURED first, not estimated: the widest pair is
// "4163" + "rpm" at 90.1 px of the 92 available, and DRIVE's distances are
// whole km precisely so that "9999 km" (88.9) is the worst they get.
#define UI_LS_RIGHT_W     112   // the right-hand value stack
#define UI_LS_ROW_H       53    // one row of that stack

// Two columns replace the vertical stack: hero on the left, the same values in
// a narrow stack on the right. Nothing is dropped, added or reordered, which is
// what makes a runtime toggle safe rather than a second design to keep in sync.
#define UI_LS_DIV_X (UI_W - 1 - UI_LS_RIGHT_W)          // the 1 px column rule
#define UI_LS_L_R   (UI_LS_DIV_X - UI_LS_PAD)           // left column, content right edge
#define UI_LS_R_L   (UI_LS_DIV_X + 1 + UI_LS_PAD)       // right column, content left edge
#define UI_LS_R_R   (UI_W - UI_LS_PAD)                  // right column, content right edge

// Chrome heights. Pages occupy what is left between them.
#define UI_STATUS_H (s_landscape ? UI_LS_STATUSBAR_H : UI_STATUSBAR_H)
#define UI_PAGE_Y   (UI_STATUS_H)
#define UI_PAGE_H   (UI_H - UI_STATUS_H)

// Portrait info-block geometry.
//
// The floor is not arbitrary. The page dots live on the root at screen
// y = UI_H - 16, which is page-local 278 once UI_STATUSBAR_H is taken off, so
// the last row has to finish above that.
#define UI_SET_INFO_RULE  192
#define UI_SET_INFO_CAP   198
#define UI_SET_INFO_ROW0  216
#define UI_SET_INFO_STEP  21

// Where the three info VALUES are right-aligned to, and at what y. Filled by
// whichever SETTINGS builder ran -- portrait stacks them down the page,
// landscape lays them across a bottom strip -- because the refresh has to
// re-align on every change: a right-aligned label's x depends on its width, so
// lv_obj_align() is a one-shot position and the numbers have to survive until
// the next one. Two copies of them would drift.
static lv_coord_t s_infoX[3];
static lv_coord_t s_infoY[3];

// Which way round the instrument is drawn.
//
// Two landscapes, not one: the bracket can put the panel on either edge, and
// which of the two is upright is a property of the mount, not something this
// firmware can know. They are named for their relationship to each other --
// one is the other turned end for end -- rather than for a physical side,
// because naming a side would be a guess the driver would have to un-learn if
// it were wrong. The driver taps until it reads right way up; after that the
// row is never touched again.
//
// PORTRAIT_FLIP is deliberately absent. Portrait has a cable at one end and a
// bracket at the other, so there is only one sensible way up for it.
enum UiOrient : uint8_t {
  UI_ORIENT_PORTRAIT = 0,
  UI_ORIENT_LANDSCAPE,
  UI_ORIENT_LANDSCAPE_FLIP,
  UI_ORIENT_COUNT,
};

// The state text of the ORIENTATION row. Index with s_orient.
static const char* const kUiOrientName[UI_ORIENT_COUNT] = {
    "PORTRAIT", "LANDSCAPE", "LANDSCAPE FLIP"};

// The active orientation and the geometry that follows from it. Set before a
// single widget exists -- uiInit() reads the stored setting, and
// uiApplyOrientation() rewrites all four together. s_landscape is derived, and
// kept as its own variable only because the layout macros read it on every
// line; setOrientVars() is the one place any of them is written.
static uint8_t s_orient = UI_ORIENT_PORTRAIT;
static bool s_landscape = false;
static lv_coord_t s_w = LCD_H_RES;
static lv_coord_t s_h = LCD_V_RES;

static void setOrientVars(uint8_t orient) {
  s_orient = (orient < UI_ORIENT_COUNT) ? orient
                                        : static_cast<uint8_t>(UI_ORIENT_PORTRAIT);
  s_landscape = (s_orient != UI_ORIENT_PORTRAIT);
  s_w = s_landscape ? LCD_V_RES : LCD_H_RES;
  s_h = s_landscape ? LCD_H_RES : LCD_V_RES;
}

static lv_display_rotation_t orientRotation(uint8_t orient) {
  switch (orient) {
    case UI_ORIENT_LANDSCAPE:
      return LV_DISPLAY_ROTATION_90;
    case UI_ORIENT_LANDSCAPE_FLIP:
      return LV_DISPLAY_ROTATION_270;
    default:
      return LV_DISPLAY_ROTATION_0;
  }
}

static VehicleState* s_state = nullptr;
static const BroadcastState* s_bcast = nullptr;
static const TripSet* s_trips = nullptr;
static const AccelSet* s_accel = nullptr;
static const FaultSet* s_faults = nullptr;
static void (*s_faultRescan)(void) = nullptr;
static void (*s_faultMute)(uint8_t) = nullptr;
static void (*s_faultClear)(uint8_t, uint16_t) = nullptr;
static void (*s_tripReset)(uint8_t) = nullptr;

// The live fuel price, cents per litre. Mirrored out of Settings rather than
// read through it because a held +/- steps at LVGL's repeat rate and every step
// would otherwise be an NVS commit. The mirror moves freely; the commit waits
// for the finger to come off.
static uint16_t s_priceCents = 201u;

static float fuelPrice(void) { return (float)s_priceCents / 100.0f; }

// Which trip the TRIP page is currently showing. One tap swaps it.
static uint8_t s_tripShown = TRIP_A;
static lv_display_t* s_disp = nullptr;
static lv_obj_t* s_root = nullptr;
static lv_obj_t* s_pages = nullptr;   // the horizontal scroller
static lv_obj_t* s_page[UI_SCREEN_COUNT];
static lv_obj_t* s_dot[UI_SCREEN_COUNT];
static lv_obj_t* s_splash = nullptr;
static uint8_t s_current = UI_HOME;

// SETTINGS info block. Copied rather than pointed at, because carTask builds
// these on its stack.
static char s_linkMode[12] = "DOWN";
static char s_linkAdapter[12] = "--";

// The window the bus rate is measured over. File scope rather than function
// statics inside refreshSettings() so buildRoot() can clear them: frames is a
// free-running total and the rate is a difference, so a window left open
// across a rebuild leaves the BUS field blank until it closes on its own.
static uint32_t s_busLastMs = 0u;
static uint32_t s_busLastFrames = 0u;
static bool s_splashDone = false;

// Whether the link is up, as last reported by uiLinkReady().
//
// It starts FALSE, not "unknown", and the dot is correspondingly red rather
// than grey from the first frame. There is no third state to draw: before the
// handshake the link IS down, and a grey dot saying "no information" about
// something we have complete information about is the same class of lie as a
// held reading that has gone stale. The SETTINGS info block has always said
// DOWN in the critical colour at boot; the status bar now agrees with it.
//
// Kept across a rebuild because the handshake that set it may not happen again
// for as long as the cable stays plugged in -- so the state has to be replayed
// into the newly built dot, not forgotten.
static bool s_linkOk = false;

static lv_color_t linkDotColour(void) {
  return s_linkOk ? UI_PRIMARY : UI_CRIT;
}

// Defined down with buildRoot(), because they need it; used up in uiTask.
static void uiApplyTheme(uint8_t theme);
static void uiApplyOrientation(uint8_t orient);

// Defined with the other value-setters, far below; declared here because the
// SETTINGS page's fuel-price row writes its own label as it is built, long
// before the refresh machinery appears in the file.
static void setTextIfChanged(lv_obj_t* lbl, const char* text);

// A theme change requested from a tap, applied by uiTask.
//
// NOT applied in the event callback. The row that was tapped lives inside
// s_root, and applying a theme deletes s_root -- deleting the object that is
// currently dispatching an event is how LVGL is made to crash. 0xFF is "no
// request pending".
static volatile uint8_t s_themeReq = 0xFFu;

// An orientation change requested from a tap, applied by uiTask, for exactly
// the same reason as s_themeReq and rather more so: the flip deletes s_root,
// and the row that asked for it is inside s_root. 0xFF is "no request".
static volatile uint8_t s_orientReq = 0xFFu;

// Cached VehicleState slot indices, resolved once at init. -1 means the signal
// is not in the poll plan, which is a normal condition, not an error.
static struct {
  int8_t coolant, volts, speed, rpm, fuel, ambient;
  int8_t tyre[4], spare;
} s_slot;

// Live widgets.
static struct {
  lv_obj_t* linkDot;
  lv_obj_t* clockLbl;
  lv_obj_t* l100;
  lv_obj_t* driveDist;   // hero sub-line, left
  lv_obj_t* driveCost;   // hero sub-line, right
  lv_obj_t* driveAccel3; // the third acceleration figure; hidden otherwise
  lv_obj_t* accLast[3];
  lv_obj_t* accBest[3];
  lv_obj_t* tripADist;
  lv_obj_t* tripBDist;
  lv_obj_t* flowSeg[UI_FLOW_SEGS];
  lv_obj_t* flowNow;
  lv_obj_t* speed;
  lv_obj_t* rpm;
  lv_obj_t* healthVolts;
  lv_obj_t* healthCoolant;
  lv_obj_t* healthAmbient;
  lv_obj_t* faults;
  lv_obj_t* faultTitle;
  lv_obj_t* faultRow[FAULT_MAX];     // "P0119  CCN"
  lv_obj_t* faultState[FAULT_MAX];   // "stored, lamp on"
  lv_obj_t* faultDesc[FAULT_MAX];    // the compiled-in description, one line
  lv_obj_t* faultSep[FAULT_MAX];     // the rule that makes a row look like a target
  lv_obj_t* faultEmpty;
  // The detail dialog. Built with the dialog and destroyed with it, like the
  // page's own widgets -- see faultDlgShow().
  lv_obj_t* dlgCode;
  lv_obj_t* dlgMod;
  lv_obj_t* dlgStatus;
  lv_obj_t* dlgDesc;
  lv_obj_t* dlgMuteLbl;
  lv_obj_t* dlgClearLbl;
  lv_obj_t* tyre[4];
  lv_obj_t* spare;
  lv_obj_t* tyreLow;
  lv_obj_t* tyreTarget;
  lv_obj_t* confirmTitle;
  lv_obj_t* confirmDetail;
  lv_obj_t* confirmBar;
  lv_obj_t* tripKm;
  lv_obj_t* tripUsed;
  lv_obj_t* tripL100;
  lv_obj_t* tripCost;
  lv_obj_t* tripAvgKph;
  lv_obj_t* tripElapsed;  // hero sub-line, left  -- IN, the whole window
  lv_obj_t* tripMoving;   // hero sub-line, right -- RIDE, the moving part
  lv_obj_t* tripSeg[TRIP_COUNT];   // the TRIP A / TRIP B / START selector
  lv_obj_t* tripSegUl;             // the 2 px underline under the active one
  lv_obj_t* setInvertState;
  lv_obj_t* setOrientState;
  lv_obj_t* setPrice;
  lv_obj_t* setLink;
  lv_obj_t* setAdapter;
  lv_obj_t* setBus;
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

// A label pinned by its RIGHT edge to a given x. The parent must be a
// full-width page or the root, which is what lv_obj_align() measures against.
//
// LV_ALIGN_TOP_RIGHT alone measures from the PARENT's right edge, which is the
// panel edge everywhere in portrait and is exactly what a landscape column is
// not: a value belonging to the left column has to stop at the column rule.
// Passing the x it should end at keeps the call sites reading in page
// coordinates like everything else on the page.
static lv_obj_t* mkRightLabel(lv_obj_t* page, const char* text, const lv_font_t* font,
                              lv_color_t colour, lv_coord_t rightX, lv_coord_t y) {
  lv_obj_t* l = mkLabel(page, text, font, colour, 0, 0);
  lv_obj_align(l, LV_ALIGN_TOP_RIGHT, static_cast<lv_coord_t>(rightX - UI_W), y);
  return l;
}

// Centred inside a box of a known width. The four landscape tyre cells are the
// only place on the instrument where a value is centred rather than pinned to
// an edge -- four columns in index order, because a 2x2 would claim left/right
// and front/rear and the sensor mapping does not support that claim.
static lv_obj_t* mkCentreLabel(lv_obj_t* parent, const char* text,
                               const lv_font_t* font, lv_color_t colour,
                               lv_coord_t x, lv_coord_t y, lv_coord_t w) {
  lv_obj_t* l = mkLabel(parent, text, font, colour, x, y);
  lv_obj_set_width(l, w);
  lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
  return l;
}

// A rule down rather than across. Only the landscape column divider needs one.
static lv_obj_t* mkVRule(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, lv_coord_t h) {
  lv_obj_t* r = lv_obj_create(parent);
  stripChrome(r);
  lv_obj_set_size(r, 1, h);
  lv_obj_set_pos(r, x, y);
  lv_obj_set_style_bg_color(r, UI_RULE, 0);
  lv_obj_set_style_bg_opa(r, LV_OPA_COVER, 0);
  return r;
}

// ---- chrome ---------------------------------------------------------------

// Status bar and page dots live on the ROOT, not inside the pages, so they stay
// put while the pages slide under them -- and so there is one of each rather
// than five.
static void buildStatusBar(lv_obj_t* root) {
  if (s_landscape) {
    // Clock, page dots and link on one 22 px line, with buildDots() filling the
    // middle.
    W.clockLbl = mkLabel(root, "--:--", UI_FONT_LABEL, UI_PRIMARY, 10, 3);
    lv_obj_t* lk = mkRightLabel(root, "LINK", UI_FONT_LABEL, UI_DIM,
                                static_cast<lv_coord_t>(UI_W - 21), 3);
    lv_obj_set_style_text_letter_space(lk, 1, 0);
    W.linkDot = mkDot(root, static_cast<lv_coord_t>(UI_W - 16), 8, 6, linkDotColour());
    mkRule(root, UI_LS_STATUSBAR_H - 1, 0, UI_W);
    return;
  }
  W.clockLbl = mkLabel(root, "--:--", UI_FONT_LABEL, UI_PRIMARY, UI_PAD - 2, 5);
  lv_obj_t* lk = mkLabel(root, "LINK", UI_FONT_LABEL, UI_DIM, 176, 5);
  lv_obj_set_style_text_letter_space(lk, 1, 0);
  W.linkDot = mkDot(root, 222, 11, 6, linkDotColour());
  mkRule(root, UI_STATUSBAR_H - 1, 0, UI_W);
}

// Portrait gives the dots a rail of their own at the foot of the screen.
// Landscape puts the same five dots in the middle of the status bar: the
// vertical axis is the short one there, and that rail is the single largest
// thing that can be handed back to the content.
static void buildDots(lv_obj_t* root) {
  const lv_coord_t d = 5;
  const lv_coord_t step = 12;
  const lv_coord_t x0 = (UI_W - (UI_SCREEN_COUNT * step - (step - d))) / 2;
  const lv_coord_t y = s_landscape
                           ? static_cast<lv_coord_t>((UI_LS_STATUSBAR_H - d) / 2)
                           : static_cast<lv_coord_t>(UI_H - 16);
  for (uint8_t i = 0; i < UI_SCREEN_COUNT; ++i) {
    s_dot[i] = mkDot(root, static_cast<lv_coord_t>(x0 + i * step), y, d,
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
  // y = 5 in BOTH bands now. It was 3 in landscape, which centred a 16 px line
  // in a 22 px row -- but the unit key beside it is 21 px tall and has to share
  // the title's baseline, and from y=3 the key would have started above the row.
  // At 5 the shared baseline is 18 and the key sits at y=1, which fits.
  const lv_coord_t x = s_landscape ? UI_LS_PAD : (UI_PAD - 2);
  const lv_coord_t y = 5;
  lv_obj_t* t = mkLabel(page, title, UI_FONT_LABEL, UI_PRIMARY, x, y);
  lv_obj_set_style_text_letter_space(t, 3, 0);
  if (note != nullptr) {
    lv_obj_t* n = mkLabel(page, note, UI_FONT_LABEL, UI_LABEL, 0, 0);
    lv_obj_align(n, LV_ALIGN_TOP_RIGHT, -x, y);
  }
  return t;
}

// A two-up row of value cells, the pattern DRIVE and TRIP both use.
// A unit caption, pinned to the RIGHT EDGE OF ITS COLUMN rather than to a
// fixed offset from the value.
//
// Fixed offsets are what put "L/100" and "km/h" half off the panel: the unit
// was placed 76 px after the column start, which leaves 32 px, and "L/100" in
// the label face needs more than that. Right-aligning inside a box that ENDS at
// the column edge cannot overflow whatever the string or the font.
//
// It is still a one-shot placement -- the box does not move when the number
// changes, so there is no per-refresh realign and no layout work in the render
// path. The units simply line up in a column, which reads tidier than chasing
// a value whose width changes with every digit.
#define UI_UNIT_BOX 62

static void mkUnit(lv_obj_t* page, const char* text, lv_coord_t colRight,
                   lv_coord_t y) {
  lv_obj_t* u = mkLabel(page, text, UI_FONT_LABEL, UI_LABEL, 0, 0);
  lv_obj_set_width(u, UI_UNIT_BOX);
  lv_obj_set_style_text_align(u, LV_TEXT_ALIGN_RIGHT, 0);
  // The box may overlap the value's box; only the rendered glyphs matter, and
  // right-aligned text sits at the far end of it.
  lv_obj_set_pos(u, static_cast<lv_coord_t>(colRight - UI_UNIT_BOX), y);
}

static void buildPair(lv_obj_t* page, lv_coord_t y, const char* capL, const char* capR,
                      lv_obj_t** outL, lv_obj_t** outR, const char* unitL,
                      const char* unitR) {
  mkRule(page, y, 0, UI_W);
  mkCaption(page, capL, UI_PAD - 2, y + 5);
  mkCaption(page, capR, UI_W / 2 + UI_PAD - 2, y + 5);
  *outL = mkLabel(page, "--", UI_FONT_MID, UI_PRIMARY, UI_PAD - 2, y + 20);
  *outR = mkLabel(page, "--", UI_FONT_MID, UI_PRIMARY, UI_W / 2 + UI_PAD - 2,
                  y + 20);
  if (unitL != nullptr) {
    mkUnit(page, unitL, UI_W / 2 - (UI_PAD - 2), y + 32);
  }
  if (unitR != nullptr) {
    mkUnit(page, unitR, UI_W - (UI_PAD - 2), y + 32);
  }
}

// One row of the LANDSCAPE right-hand stack: caption, value, unit, and the
// hairline under it. These are the same values portrait shows as two-up pairs,
// stacked in a 130 px column instead -- nothing dropped, nothing added, which
// is what makes the orientation a toggle rather than a second product.
//
// The unit is right-aligned to the COLUMN's edge rather than trailing the
// value, exactly as buildPair() does it in portrait: a one-shot placement that
// cannot overflow whatever the number does, and units that line up with each
// other down the stack.
//
// `ruleUnder` is false for the last row of a stack, which is bounded by the
// edge of the page rather than by a rule.
// A page's UNIT KEY: the hero's unit, printed once at the top right in the
// larger label face instead of trailing the numeral.
//
// A key and not an inline unit for a measured reason. At 20 px "L/100"
// right-aligned to DRIVE's column starts at x=127 and the hero reaches x=137,
// so inline stops fitting the moment the unit grows. As a key it fits on every
// page in both bands, and it hands each hero the full width of its column --
// which TRIP needs, since "247.8" alone is 164 px of 165.
//
// `capY` is the CAPTION's y, not the key's. The faces have different line
// heights and base_lines (16/3 against 21/4), so matching their tops would
// leave the key sitting visibly low; matching baselines means -4.
#define UI_UNIT_KEY_DY (-4)

static lv_obj_t* mkUnitKey(lv_obj_t* page, const char* text, lv_coord_t rightX,
                           lv_coord_t capY) {
  lv_obj_t* k = mkRightLabel(page, text, UI_FONT_UNIT, UI_LABEL, rightX,
                             static_cast<lv_coord_t>(capY + UI_UNIT_KEY_DY));
  lv_obj_set_style_text_letter_space(k, 2, 0);
  return k;
}

// The hero's SUB-LINE: two short figures on one row, one pinned to each end of
// the column.
//
// This is what pays for the rest of the page. font_digits_64 is 73 px tall in a
// column that has 172, so a hero on its own leaves ~87 px of air -- and the
// figures put here are small, slow-moving and read in passing. Moving them off
// the grid is what freed the cell COST occupies.
//
// Each side is ONE label holding a word, a number and sometimes a unit, exactly
// as DRIVE's flowNow is: the number's width changes, and neighbours drawn as
// separate labels would shuffle sideways every time it did.
static void mkHeroSub(lv_obj_t* page, lv_coord_t y, lv_coord_t leftX,
                      lv_coord_t rightX, lv_obj_t** outL, lv_obj_t** outR) {
  *outL = mkLabel(page, "--", UI_FONT_LABEL, UI_DIM, leftX, y);
  *outR = mkRightLabel(page, "--", UI_FONT_LABEL, UI_DIM, rightX, y);
}

static void mkStackRow(lv_obj_t* page, lv_coord_t y, lv_coord_t h, const char* cap,
                       lv_obj_t** out, const char* unit, bool ruleUnder = true) {
  // 16 px of label plus 35 of digits is 51, and it is CENTRED in whatever
  // height the row is given rather than pinned to its top -- a stack of four
  // 53s and one whose last row is 56 both have to look evenly set.
  const lv_coord_t top = static_cast<lv_coord_t>(y + (h - 51) / 2);
  mkCaption(page, cap, UI_LS_R_L, top);
  *out = mkLabel(page, "--", UI_FONT_MID, UI_PRIMARY, UI_LS_R_L,
                 static_cast<lv_coord_t>(top + 16));
  if (unit != nullptr) {
    mkUnit(page, unit, UI_LS_R_R, static_cast<lv_coord_t>(top + 28));
  }
  if (ruleUnder) {
    mkRule(page, static_cast<lv_coord_t>(y + h), UI_LS_DIV_X + 1, UI_LS_RIGHT_W);
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
  mkUnitKey(page, "L/100", UI_W - (UI_PAD - 2), 6);
  W.l100 = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_PAD - 4, 20);

  // What this journey has covered and what its fuel cost -- the same window the
  // hero measures, so the whole top of the card is one drive.
  mkHeroSub(page, 96, UI_PAD - 2, UI_W - (UI_PAD - 2), &W.driveDist, &W.driveCost);
  // Portrait has 220 px on this line and 6 px of vertical air, so the third
  // acceleration figure goes in the MIDDLE of the same line rather than on one
  // of its own. Hidden until a run needs it.
  W.driveAccel3 = mkCentreLabel(page, "", UI_FONT_LABEL, UI_DIM, 70, 96, 100);
  lv_obj_add_flag(W.driveAccel3, LV_OBJ_FLAG_HIDDEN);

  mkCaption(page, "FLOW", UI_PAD - 2, 118);
  // The live figure shares the caption row, right-aligned. Value and unit in
  // one label because the unit itself changes -- L/h at a standstill, L/100km
  // once moving -- and two labels would have the number jumping sideways as the
  // unit beside it changed width.
  W.flowNow = mkLabel(page, "--", UI_FONT_LABEL, UI_FAINT, 0, 0);
  lv_obj_align(W.flowNow, LV_ALIGN_TOP_RIGHT, -(UI_PAD - 2), 118);

  for (uint8_t i = 0; i < UI_FLOW_SEGS; ++i) {
    lv_obj_t* s = lv_obj_create(page);
    stripChrome(s);
    lv_obj_set_size(s, 12, 12);
    lv_obj_set_pos(s, static_cast<lv_coord_t>(UI_PAD - 2 + i * 14), 136);
    lv_obj_set_style_bg_color(s, UI_DOT_OFF, 0);
    lv_obj_set_style_bg_opa(s, LV_OPA_COVER, 0);
    W.flowSeg[i] = s;
  }

  // IN TANK and RANGE used to be these two cells. The tank percentage is still
  // decoded and still polled; it is simply not drawn. What took their place
  // answers "how far on each meter" without a swipe to TRIP and two taps there.
  buildPair(page, 156, "TRIP A", "TRIP B", &W.tripADist, &W.tripBDist, "km", "km");
  buildPair(page, 214, "SPEED", "ENGINE", &W.speed, &W.rpm, "km/h", "rpm");
  mkRule(page, 272, 0, UI_W);
}

// The same card, turned. Average as the hero on the left with the flow bar
// under it, the four supporting figures stacked on the right -- the identical
// set portrait shows as two 2-up rows, in the identical order.
//
// The flow bar keeps all sixteen segments. It is the one element that had to
// be re-pitched rather than re-placed: portrait's 12 px segments on a 14 px
// pitch want 222 px and the left column has 187, so the PITCH comes off the
// column while the 2 px gap stays portrait's. Sixteen is the resolution of the
// readout and dropping to twelve would quietly change what a lit segment
// means.
static void buildDriveLs(lv_obj_t* page) {
  mkVRule(page, UI_LS_DIV_X, 0, UI_PAGE_H);

  mkCaption(page, "FROM START", UI_LS_PAD, 8);
  mkUnitKey(page, "L/100", UI_LS_L_R, 8);
  W.l100 = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_LS_PAD - 2, 24);
  mkHeroSub(page, 103, UI_LS_PAD, UI_LS_L_R, &W.driveDist, &W.driveCost);
  // Landscape's column is only 165 px wide but has 57 px of air under the
  // sub-line, so the third figure gets a SECOND line instead of being squeezed
  // onto the first. Same three figures, same words, different arrangement --
  // which is what the two bands do everywhere else.
  W.driveAccel3 = mkLabel(page, "", UI_FONT_LABEL, UI_DIM, UI_LS_PAD, 121);
  lv_obj_add_flag(W.driveAccel3, LV_OBJ_FLAG_HIDDEN);

  // The flow block sits on the floor of the column rather than under the hero:
  // the hero's width varies and the bar's does not, so anchoring the bar to
  // the bottom keeps it still while the number above it changes.
  mkCaption(page, "FLOW", UI_LS_PAD, 176);
  W.flowNow = mkRightLabel(page, "--", UI_FONT_LABEL, UI_FAINT, UI_LS_L_R, 176);

  // 10 wide on a 12 pitch, which is portrait's 2 px gap and 190 px of bar.
  //
  // It was 8 on 10 -- 158 px in a 187 px column, visibly four fifths of the
  // width where portrait's fills it, which is the defect this fixes. The old
  // comment blamed a 165 px column: that was UI_LS_RIGHT_W at 130, and the
  // stack narrowing to 112 moved the divider 18 px right without anything
  // re-deriving the bar.
  //
  // The arithmetic has no exact answer -- 16w + 15g = 187 is integral only at
  // w=7/g=5, which is a dotted line, not a bar -- so this lands 3 px PAST the
  // content edge at x=200, exactly as portrait's own bar overhangs its 230 px
  // edge to 232. The rule is at UI_LS_DIV_X (207), so the bar still clears it
  // by 7 px, the same air portrait leaves against the panel.
  //
  // The artboard draws this as sixteen flex-grow segments on a 2 px gap, i.e.
  // 9.8 px each: matching the GAP and keeping the bar full width is as close as
  // integer pixels get to it, and closer than staying short.
  for (uint8_t i = 0; i < UI_FLOW_SEGS; ++i) {
    lv_obj_t* sg = lv_obj_create(page);
    stripChrome(sg);
    lv_obj_set_size(sg, 10, 12);
    lv_obj_set_pos(sg, static_cast<lv_coord_t>(UI_LS_PAD + i * 12), 197);
    lv_obj_set_style_bg_color(sg, UI_DOT_OFF, 0);
    lv_obj_set_style_bg_opa(sg, LV_OPA_COVER, 0);
    W.flowSeg[i] = sg;
  }

  mkStackRow(page, 0, UI_LS_ROW_H, "TRIP A", &W.tripADist, "km");
  mkStackRow(page, 54, UI_LS_ROW_H, "TRIP B", &W.tripBDist, "km");
  mkStackRow(page, 108, UI_LS_ROW_H, "SPEED", &W.speed, "km/h");
  mkStackRow(page, 162, 56, "ENGINE", &W.rpm, "rpm", false);
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
  mkRule(page, 32, 0, UI_W);
  for (uint8_t i = 0; i < 4; ++i) {
    const lv_coord_t x = (i & 1u) ? UI_W / 2 + UI_PAD - 2 : UI_PAD - 2;
    const lv_coord_t y = static_cast<lv_coord_t>(40 + (i >> 1) * 74);
    mkCaption(page, kCaps[i], x, y);
    W.tyre[i] =
        mkLabel(page, "--", UI_FONT_MID, UI_PRIMARY, x, static_cast<lv_coord_t>(y + 16));
    mkLabel(page, "bar", UI_FONT_LABEL, UI_LABEL, static_cast<lv_coord_t>(x + 72),
            static_cast<lv_coord_t>(y + 28));
  }
  mkRule(page, 106, 0, UI_W);
  mkRule(page, 180, 0, UI_W);

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
  mkRule(page, 244, 0, UI_W);
  W.tyreLow = mkLabel(page, "", UI_FONT_LABEL, UI_CRIT, UI_PAD - 2, 250);
  W.tyreTarget = mkLabel(page, "TARGET 2.1 - 2.3", UI_FONT_LABEL, UI_LABEL, 0, 0);
  lv_obj_align(W.tyreTarget, LV_ALIGN_TOP_RIGHT, -(UI_PAD - 2), 250);
  lv_obj_add_flag(W.tyreLow, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(W.tyreTarget, LV_OBJ_FLAG_HIDDEN);
}

// TYRES is the one page where portrait and landscape deliberately say
// different things, and the divergence is the point.
//
// Portrait has to use a 2x2 because 240 px will not hold four columns. A 2x2
// makes a spatial claim -- left column is the left of the truck, top row is the
// front -- and that claim is exactly what `21 3B` does not support: it returns
// four bytes in a fixed order and nothing in the captures says which wheel is
// which. Landscape has the width to put the four sensors in a single row, in
// index order, implying no geometry at all. The layout finally says the same
// thing the caption has always said.
//
// The unit moves to the title as one BAR key. Printing "bar" four times is
// noise, and the width it costs is the width the cells need.
static void buildTyresLs(lv_obj_t* page) {
  buildHeader(page, "TYRES", nullptr);
  mkUnitKey(page, "BAR", UI_LS_R_R, 5);
  mkRule(page, 22, 0, UI_W);

  static const char* const kCaps[4] = {"SENSOR 1", "SENSOR 2", "SENSOR 3", "SENSOR 4"};

  // A 2 x 2, reading order: 1 2 on the top row, 3 4 on the bottom. It was four
  // columns in index order, and that was the more conservative arrangement --
  // a grid implies front/rear and left/right, and `21 3B` gives four bytes in
  // a fixed order with nothing in the captures saying which wheel is which.
  // Four across made no claim; a 2 x 2 of numbers invites one.
  //
  // Asked for anyway, and it is the driver's call to make: the cells still say
  // SENSOR, not LF/RF/LR/RR, so the layout suggests a shape while the labels
  // refuse to name a corner. Nothing here should be read as a mapping until
  // one is actually measured -- deflate a wheel, watch which index moves.
  const lv_coord_t cellW = (UI_W - 1) / 2;                // 159, one 1 px rule
  const lv_coord_t top = 23;
  const lv_coord_t regionH = 146;
  const lv_coord_t rowH = static_cast<lv_coord_t>(regionH / 2);   // 73
  // Caption and value are one 51 px block (16 + 35), centred in each ROW now
  // rather than in the whole region.
  const lv_coord_t capDY = static_cast<lv_coord_t>((rowH - 51) / 2);
  mkVRule(page, cellW, top, regionH);
  mkRule(page, static_cast<lv_coord_t>(top + rowH), 0, UI_W);
  for (uint8_t i = 0; i < 4; ++i) {
    const lv_coord_t x = (i & 1u) ? static_cast<lv_coord_t>(cellW + 1) : 0;
    const lv_coord_t capY =
        static_cast<lv_coord_t>(top + (i >> 1) * rowH + capDY);
    mkCentreLabel(page, kCaps[i], UI_FONT_LABEL, UI_LABEL, x, capY, cellW);
    W.tyre[i] = mkCentreLabel(page, "--", UI_FONT_MID, UI_PRIMARY, x,
                              static_cast<lv_coord_t>(capY + 16), cellW);
  }
  mkRule(page, 169, 0, UI_W);

  // The spare on its own line, label-sized, for the same reason as portrait:
  // `21 3B` excludes it, you cannot act on it while moving, and at 0.48 bar it
  // would be the lowest number on a screen of equally-sized cells.
  mkCaption(page, "SPARE", UI_LS_PAD, 174);
  W.spare = mkLabel(page, "--", UI_FONT_LABEL, UI_DIM, 74, 174);
  mkRule(page, 194, 0, UI_W);

  W.tyreLow = mkLabel(page, "", UI_FONT_LABEL, UI_CRIT, UI_LS_PAD, 198);
  W.tyreTarget = mkRightLabel(page, "TARGET 2.1 - 2.3", UI_FONT_LABEL, UI_LABEL,
                              UI_LS_R_R, 198);
  lv_obj_add_flag(W.tyreLow, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(W.tyreTarget, LV_OBJ_FLAG_HIDDEN);
}

// One tappable settings row: name, an explanatory sub-line, and its state on
// the right. Returns the state label so the caller can keep it in sync.
//
// CLICKABLE but NOT SCROLLABLE, which is the distinction that matters inside
// the swipe scroller. LVGL walks up from the pressed object looking for a
// scrollable ancestor, so a clickable child still lets a drag scroll the pages
// underneath it -- it is a child that is itself scrollable that swallows the
// drag, which is what once made swiping work only on bare background.
static lv_obj_t* mkSettingRow(lv_obj_t* page, lv_coord_t y, const char* name,
                              const char* detail, const char* state,
                              lv_event_cb_t cb) {
  lv_obj_t* row = lv_obj_create(page);
  stripChrome(row);
  lv_obj_set_size(row, UI_W, 44);
  lv_obj_set_pos(row, 0, y);
  lv_obj_add_flag(row, LV_OBJ_FLAG_CLICKABLE);
  if (cb != nullptr) {
    lv_obj_add_event_cb(row, cb, LV_EVENT_CLICKED, nullptr);
  }

  mkLabel(row, name, UI_FONT_LABEL, UI_PRIMARY, UI_PAD - 2, 4);
  mkLabel(row, detail, UI_FONT_LABEL, UI_FAINT, UI_PAD - 2, 23);

  // Level with the NAME, not floating between the two lines. Sitting 6 px
  // lower put its line box across the sub-line's, so on the longest pairing
  // -- ORIENTATION plus LANDSCAPE FLIP -- the two ran into each other, and the
  // state had to clear the sub-line's width rather than the name's much
  // shorter one.
  lv_obj_t* st = mkLabel(row, state, UI_FONT_LABEL, UI_DIM, 0, 0);
  lv_obj_align(st, LV_ALIGN_TOP_RIGHT, -(UI_PAD - 2), 4);

  mkRule(page, static_cast<lv_coord_t>(y + 44), 0, UI_W);
  return st;
}

// A read-only diagnostic line for the info block. `slot` says which of the
// three it is: the value is right-aligned, so every text change has to
// re-align it, and only the builder knows where it was put -- portrait stacks
// these down the page, landscape lays them across a strip along the bottom.
static lv_obj_t* mkInfoRow(lv_obj_t* page, uint8_t slot, const char* name,
                           lv_coord_t capX, lv_coord_t capY, lv_coord_t valRight) {
  mkCaption(page, name, capX, capY);
  // Same y as the caption. The value used to be lifted 3 px, which is a habit
  // from pairing a label face against a digit face -- and both of these are
  // font_label_14, so it simply read as two words that failed to line up. Most
  // obvious in landscape, where the pair sits side by side on one strip.
  s_infoX[slot] = valRight;
  s_infoY[slot] = capY;
  return mkRightLabel(page, "--", UI_FONT_LABEL, UI_DIM, valRight, s_infoY[slot]);
}

static void onThemeTap(lv_event_t* e) {
  (void)e;
  const uint8_t next = (uint8_t)((uiThemeGet() + 1u) % UI_THEME_COUNT);
  (void)settingsSetTheme(next);
  // Requested, not applied: see s_themeReq. Applying here would delete the
  // object currently dispatching this very event.
  s_themeReq = next;
}

// Cycles PORTRAIT -> LANDSCAPE -> LANDSCAPE FLIP -> PORTRAIT. A cycle rather
// than a pair of toggles for the same reason the palette is one: three states
// do not earn three rows on a page whose whole argument is that it does not
// invent options.
static void onOrientTap(lv_event_t* e) {
  (void)e;
  const uint8_t next = (uint8_t)((s_orient + 1u) % UI_ORIENT_COUNT);
  (void)settingsSetOrientation(next);
  // Requested, not applied, for the same reason as the theme and rather more
  // urgently: this deletes the whole tree, including the row being tapped.
  s_orientReq = next;
}

// Show the live price. Called from the builder and from every adjustment.
static void priceSetText(void) {
  if (W.setPrice == nullptr) {
    return;
  }
  char buf[16];
  snprintf(buf, sizeof(buf), "%u.%02u " UI_EUR, (unsigned)(s_priceCents / 100u),
           (unsigned)(s_priceCents % 100u));
  setTextIfChanged(W.setPrice, buf);
}

// True while a held button is repeating, so the CLICKED that arrives when the
// finger finally lifts does not add one more step on top of the repeat.
//
// CLEARED ON PRESS, not on click. LVGL only sends CLICKED when the press did
// not turn into a scroll (lv_indev.c: `if (scroll_obj == NULL)`), and these are
// 36 px targets sitting inside a horizontal scroller -- a fingertip that drifts
// past the scroll limit on the way up produces repeats and then no click at
// all. Clearing on the click alone left the latch stuck true, and the next
// genuine tap was swallowed clearing it. PRESSED always arrives.
static bool s_priceHeld = false;

// The price is committed to flash by uiTask, a moment after the last change --
// not from the event handler.
//
// Two reasons, and the first is a bug the second only tidies. A held button
// that ends in a scroll never delivers CLICKED, so a commit hung off the click
// is simply skipped: the label and the live value read 2.60 and NVS still holds
// 2.01, and the next boot silently reverts it. And committing per step would
// put a flash write inside the LVGL event dispatch at the repeat rate.
static bool s_priceDirty = false;
static uint32_t s_priceDirtyMs = 0u;

#define UI_PRICE_COMMIT_MS 1200u

static void priceStep(int16_t delta) {
  int32_t v = static_cast<int32_t>(s_priceCents) + delta;
  if (v < static_cast<int32_t>(UI_PRICE_MIN)) {
    v = UI_PRICE_MIN;
  } else if (v > static_cast<int32_t>(UI_PRICE_MAX)) {
    v = UI_PRICE_MAX;
  }
  s_priceCents = static_cast<uint16_t>(v);
  s_priceDirty = true;
  s_priceDirtyMs = lv_tick_get();
  priceSetText();
}

// Called from uiTask with the LVGL lock held.
static void priceCommitIfDue(void) {
  if (!s_priceDirty || lv_tick_elaps(s_priceDirtyMs) < UI_PRICE_COMMIT_MS) {
    return;
  }
  s_priceDirty = false;
  (void)settingsSetFuelPrice(s_priceCents);
}

// One handler for both buttons; `dir` is which one called it.
//
// The COMMIT happens on the click, never on a repeat tick. LVGL repeats at its
// own rate, and committing per tick would put a flash write behind every step
// of a held press -- so the value moves freely and only the release is stored.
static void priceEvent(lv_event_t* e, int16_t dir) {
  switch (lv_event_get_code(e)) {
    case LV_EVENT_PRESSED:
      s_priceHeld = false;    // a fresh press; whatever the last one did is done
      break;
    case LV_EVENT_LONG_PRESSED_REPEAT:
      s_priceHeld = true;
      priceStep(static_cast<int16_t>(dir * static_cast<int16_t>(UI_PRICE_STEP_FAST)));
      break;
    case LV_EVENT_CLICKED:
      // A long press ends in a release, and a release sends CLICKED too -- so
      // without the latch the finger coming off a held button would add one
      // more step on top of everything the repeat already did.
      if (!s_priceHeld) {
        priceStep(static_cast<int16_t>(dir * static_cast<int16_t>(UI_PRICE_STEP)));
      }
      break;
    default:
      break;
  }
}

static void onPriceMinus(lv_event_t* e) { priceEvent(e, -1); }
static void onPricePlus(lv_event_t* e) { priceEvent(e, 1); }

static void mkPriceButton(lv_obj_t* row, lv_coord_t x, lv_coord_t w,
                          const char* glyph, lv_event_cb_t cb) {
  // Full row height, so the target is as tall as a settings row even though it
  // is narrower than one. A price is adjusted with the truck stopped, but it is
  // still adjusted with a fingertip.
  lv_obj_t* b = mkOutline(row, x, 0, w, 44, 2, UI_DIM, 2);
  lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_t* t = mkLabel(b, glyph, UI_FONT_UNIT, UI_DIM, 0, 0);
  lv_obj_center(t);
  lv_obj_add_event_cb(b, cb, LV_EVENT_PRESSED, nullptr);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(b, cb, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);
}

// The FUEL PRICE row. Deliberately NOT a cycle like its two neighbours: a price
// is not a small set of choices, it is a number to land on exactly, and cycling
// from 2.01 to 1.50 a cent at a time would be fifty-one taps. A tap moves a
// cent, a held press moves five at a time.
static void mkPriceRow(lv_obj_t* page, lv_coord_t y) {
  lv_obj_t* row = lv_obj_create(page);
  stripChrome(row);
  lv_obj_set_size(row, UI_W, 44);
  lv_obj_set_pos(row, 0, y);

  // NO sub-line, unlike its two neighbours. Theirs explain a gesture that is not
  // visible -- that the row cycles, that the setting is a one-off. A minus and a
  // plus explain themselves, and a caption telling the driver to tap the button
  // they are already looking at is the kind of text that teaches people to stop
  // reading captions. So the name centres in the row instead, on the same line
  // as the value and the two targets.
  mkLabel(row, "FUEL PRICE", UI_FONT_LABEL, UI_PRIMARY, UI_PAD - 2, 14);

  // Laid out from the RIGHT edge in, because the name on the left is the part
  // that can afford to be crowded: at 240 px the buttons and value take 136 of
  // the 220, and "FUEL PRICE" needs 68 of what is left.
  const lv_coord_t right = static_cast<lv_coord_t>(UI_W - (UI_PAD - 2));
  const lv_coord_t bw = 36, gap = 6, vw = 52;
  const lv_coord_t plusX = static_cast<lv_coord_t>(right - bw);
  const lv_coord_t valX = static_cast<lv_coord_t>(plusX - gap - vw);
  const lv_coord_t minusX = static_cast<lv_coord_t>(valX - gap - bw);

  mkPriceButton(row, minusX, bw, "-", onPriceMinus);
  mkPriceButton(row, plusX, bw, "+", onPricePlus);

  // Centred in a FIXED box so the number does not shuffle between the buttons
  // as it changes width.
  W.setPrice = mkCentreLabel(row, "", UI_FONT_LABEL, UI_PRIMARY, valX, 14, vw);
  priceSetText();

  mkRule(page, static_cast<lv_coord_t>(y + 44), 0, UI_W);
}

// The one page a driver reads while stopped, so it sits last in the swipe
// order: no driving screen is ever behind it, and it costs nothing to be the
// furthest from where the boot settles.
static void buildSettings(lv_obj_t* page) {
  buildHeader(page, "SETTINGS", nullptr);

  mkRule(page, 24, 0, UI_W);

  // Cycles through kUiThemeName in order and wraps -- five of them now, not the
  // three this comment used to claim. A cycle rather than radio rows because
  // five of those would cost the page every other setting it might ever want.
  W.setInvertState = mkSettingRow(page, 30, "PALETTE", "tap to cycle",
                                  kUiThemeName[uiThemeGet()], onThemeTap);
  lv_obj_set_style_text_color(W.setInvertState, UI_PRIMARY, 0);

  // ORIENTATION sits BELOW the palette, not above, and the order is the whole
  // argument. The palette is reached while squinting into low sun with the
  // truck moving; orientation is set once, when the bracket is chosen, and
  // then never touched again. The row wanted in a hurry stays first.
  //
  // It took one of the two rows that were deliberately left empty. The other
  // stays empty: a page that invents options to look finished teaches the
  // driver to distrust it, and that still stands -- this is not an invented
  // option.
  //
  // "when fitted" rather than "at the bracket" because of 240 px, not taste:
  // the longer wording is 149 px from x=10 and the LANDSCAPE state label
  // starts at 154, so the two ran into each other on the portrait board. Same
  // string in both bands -- one row, one wording.
  W.setOrientState =
      mkSettingRow(page, 75, "ORIENTATION", "set once, when fitted",
                   kUiOrientName[s_orient], onOrientTap);
  lv_obj_set_style_text_color(W.setOrientState, UI_PRIMARY, 0);

  mkPriceRow(page, 120);

  // The info block is READ ONLY, deliberately. A settings page that can break
  // the link is a settings page that will, and the link is the entire device.
  // Captioned INFO, with the rows named LINK / ADAPTER / BUS -- the same three
  // words landscape lays across its bottom strip. They used to be a LINK
  // heading over a MODE row, which meant the two bands named the same readout
  // differently for no reason either could explain.
  mkRule(page, UI_SET_INFO_RULE, 0, UI_W);
  mkCaption(page, "INFO", UI_PAD - 2, UI_SET_INFO_CAP);

  const lv_coord_t vr = UI_W - (UI_PAD - 2);
  W.setLink = mkInfoRow(page, 0, "LINK", UI_PAD - 2, UI_SET_INFO_ROW0, vr);
  W.setAdapter = mkInfoRow(page, 1, "ADAPTER", UI_PAD - 2,
                           UI_SET_INFO_ROW0 + UI_SET_INFO_STEP, vr);
  W.setBus = mkInfoRow(page, 2, "BUS", UI_PAD - 2,
                       UI_SET_INFO_ROW0 + 2 * UI_SET_INFO_STEP, vr);
}

// SETTINGS is the one page that cannot take two columns, and it was drawn that
// way first and then measured. Every word on it is font_label_14 -- there is
// no smaller label face and there is not going to be one -- so ORIENTATION
// plus its chip is about 205 px and ADAPTER plus its value about 106 px. Two
// columns want 351 px of a 320 px board, and PALETTE reflowed onto two lines.
// A settings row that reflows is a settings row that looks broken.
//
// So the controls keep the full width and INFO becomes a strip along the
// bottom, three pairs read left to right -- which landscape is what makes
// possible at all: the three pairs need 262 px and portrait has 240.
static void buildSettingsLs(lv_obj_t* page) {
  buildHeader(page, "SETTINGS", nullptr);
  mkRule(page, 22, 0, UI_W);

  W.setInvertState = mkSettingRow(page, 23, "PALETTE", "tap to cycle",
                                  kUiThemeName[uiThemeGet()], onThemeTap);
  lv_obj_set_style_text_color(W.setInvertState, UI_PRIMARY, 0);

  W.setOrientState =
      mkSettingRow(page, 68, "ORIENTATION", "set once, when fitted",
                   kUiOrientName[s_orient], onOrientTap);
  lv_obj_set_style_text_color(W.setOrientState, UI_PRIMARY, 0);

  mkPriceRow(page, 113);

  // The remaining placeholder row is the gap between here and the strip. It is
  // left genuinely empty rather than filled with a disabled-looking control.
  mkRule(page, 186, 0, UI_W);
  W.setLink = mkInfoRow(page, 0, "LINK", UI_LS_PAD, 193, 104);
  W.setAdapter = mkInfoRow(page, 1, "ADAPTER", 112, 193, 212);
  W.setBus = mkInfoRow(page, 2, "BUS", 222, 193, UI_LS_R_R);
}

// Defined further down with the other value-setters; used here because the
// settings page is built alongside its peers rather than after them.
static void setColourIfChanged(lv_obj_t* lbl, lv_color_t c);

// Right-aligned text that only repaints when it changed.
//
// lv_label_set_text() invalidates unconditionally, so setting the same string
// every refresh would keep three labels redrawing at 10 Hz for nothing -- and
// the re-align afterwards is only needed because the width may have changed.
static void setInfoText(lv_obj_t* lbl, uint8_t slot, const char* txt) {
  if (lbl == nullptr) {
    return;
  }
  const char* cur = lv_label_get_text(lbl);
  if (cur != nullptr && strcmp(cur, txt) == 0) {
    return;
  }
  lv_label_set_text(lbl, txt);
  lv_obj_align(lbl, LV_ALIGN_TOP_RIGHT,
               static_cast<lv_coord_t>(s_infoX[slot] - UI_W), s_infoY[slot]);
}

// The info block. Cheap enough to run every cycle, but the bus rate needs a
// window: frames is a free-running total, so it is differenced against the
// previous sample rather than divided by uptime, which would average away
// exactly the drop we would be looking for.
static void refreshSettings(uint32_t now) {
  if (W.setLink == nullptr) {
    return;
  }
  setInfoText(W.setLink, 0, s_linkMode);
  setColourIfChanged(W.setLink,
                     (strcmp(s_linkMode, "DOWN") == 0) ? UI_CRIT : UI_PRIMARY);

  setInfoText(W.setAdapter, 1, s_linkAdapter);

  if (s_bcast != nullptr) {
    if (s_busLastMs == 0u) {
      s_busLastMs = now;
      s_busLastFrames = s_bcast->frames;
    } else if (now - s_busLastMs >= 2000u) {
      const uint32_t d = s_bcast->frames - s_busLastFrames;
      const uint32_t ms = now - s_busLastMs;
      s_busLastMs = now;
      s_busLastFrames = s_bcast->frames;
      char buf[16];
      snprintf(buf, sizeof(buf), "%u f/s", (unsigned)((d * 1000u) / (ms ? ms : 1u)));
      setInfoText(W.setBus, 2, buf);
    }
  }
}

// ---------------------------------------------------------------------------
// FAULTS -- the first drill-down, and the only screen not on the swipe axis.
//
// It is not a sixth peer. A fault list is read deliberately, from a standstill,
// after something has already told you to look -- so it costs a flick from
// nobody who is not going there. That is what the vertical axis is for, and
// this is the first thing built on it.
//
// It lives on the ROOT and covers the page area, leaving the status bar and the
// page dots showing. That is deliberate: the dots still say HEALTH, because
// that is where you are -- you are INSIDE health, not somewhere else.
// ---------------------------------------------------------------------------

static lv_obj_t* s_faultPage = nullptr;

// 40, which is two lines and a finger.
//
// It was 20 -- one line, and a target half the height of the 44 px a fingertip
// actually needs, on the one page read with a thumb. The second line is what
// the extra height is FOR: the compiled-in description, which is the thing a
// driver wants and the thing a five-character code cannot say. A separator
// rule under each row is what makes the pair read as one target rather than as
// two loose lines.
#define UI_FAULT_ROW_H 40

// The detail dialog. s_faultDlg is the SCRIM; the card is its only child, so
// deleting the scrim takes the whole thing with it.
//
// The fault it is showing is remembered as (ecu, code) as well as by index,
// for the same reason the mutes are matched that way: a RE-READ while the
// dialog is open renumbers the list, and an index alone would quietly start
// describing a different fault. It closes instead.
static lv_obj_t* s_faultDlg = nullptr;
static uint8_t   s_faultDlgIdx = 0xFFu;
static uint8_t   s_faultDlgEcu = 0xFFu;
static uint16_t  s_faultDlgCode = 0u;

// CLEAR is ARMED by the first tap and acts on the second.
//
// It is the only control in the fault pages that destroys anything: muting is
// undone by the same button, a re-read is harmless, and BACK costs nothing --
// but a code erased from a module is gone, along with the freeze frame and the
// fact that it ever happened. The TRIP reset earns a long press and a dialog
// for less than that; a dialog on top of a dialog is worse than the problem,
// so the button carries its own confirmation instead and SAYS which state it is
// in. Nothing here happens on one tap.
//
// The arming lapses on its own, so a dialog left open with CLEAR half-pressed
// cannot be finished by a bump on a rough road ten minutes later.
#define UI_CLEAR_ARM_MS  4000u
// How long to wait for the list to lose the row before saying it did not work.
// The request crosses to carTask, which transacts with the module; the module
// answering is what makes the fault disappear, and silence is a refusal.
#define UI_CLEAR_WAIT_MS 3000u
static uint32_t s_clearArmMs = 0u;    // lv_tick the first tap armed it; 0 = idle
static uint32_t s_clearSentMs = 0u;   // lv_tick the request went out; 0 = none

// ONE FINGERTIP CAN PRODUCE SEVERAL CLICKS on this panel, and a dialog that
// appears or disappears under that fingertip is where it shows.
//
// The CST328 reports slowly enough that Touch.cpp has to bridge a contact with
// a release hold, and a contact can still arrive as more than one press --
// documented in CLAUDE.md, where a flat 20 ms hold put two CLICKEDs inside the
// double-tap window on TRIP. Harmless while every control was a static widget.
// Not harmless once a tap BUILDS a widget under the finger: the trailing click
// landed on the dialog that had just appeared, and (when the scrim still
// dismissed on a tap) the one after that landed on the LIST the dialog had
// just stopped covering -- so a tap read as passing straight through the
// dialog into the row behind it, opening another.
//
// So both edges are deaf for a moment: the dialog ignores its own controls for
// SETTLE ms after it is built, and the list ignores row taps for SETTLE ms
// after a dialog closes. 250 ms is far longer than the stray click and far
// shorter than a second, deliberate tap.
#define UI_DLG_SETTLE_MS 250u
static uint32_t s_faultDlgOpenMs = 0u;     // lv_tick the dialog was built at
static uint32_t s_faultDlgClosedMs = 0u;   // lv_tick the last one was deleted

// True once the dialog has been up long enough to be acted on.
static bool faultDlgSettled(void) {
  return s_faultDlgOpenMs != 0u &&
         lv_tick_elaps(s_faultDlgOpenMs) >= UI_DLG_SETTLE_MS;
}

static void faultDlgShow(bool open, uint8_t index);
static void refreshFaultDlg(void);

// Drop every pointer into the dialog's tree WITHOUT deleting anything. Called
// both by faultDlgShow(false), which deletes it first, and by the page
// teardown, which has already deleted it as part of the page.
static void faultDlgForget(void) {
  s_faultDlg = nullptr;
  s_faultDlgIdx = 0xFFu;
  s_faultDlgEcu = 0xFFu;
  s_faultDlgCode = 0u;
  W.dlgCode = nullptr;
  W.dlgMod = nullptr;
  W.dlgStatus = nullptr;
  W.dlgDesc = nullptr;
  W.dlgMuteLbl = nullptr;
  W.dlgClearLbl = nullptr;
  s_clearArmMs = 0u;
  s_clearSentMs = 0u;
  s_faultDlgOpenMs = 0u;
}

// How far a finger may travel and still be a tap.
//
// LVGL's own suppression is not enough here, and the reason is worth knowing:
// it drops LV_EVENT_CLICKED only when the press actually became a SCROLL
// (lv_indev.c, `if(scroll_obj == NULL)`), and a scroll only starts when the
// object has somewhere to go in that direction. A list whose rows happen to
// fit -- five faults at 40 px in portrait's 206 px, which is the common case --
// has nothing to scroll, so every drag across it arrived as a click and opened
// a dialog. From the driver's side the list simply could not be dragged.
//
// 12 px, just past LV_INDEV_DEF_SCROLL_LIMIT's 10: a deliberate drag is always
// further than that, and a fingertip's roll on a 40 px row never is.
#define UI_FAULT_TAP_SLOP 12

// Where the press started, and what the list had already scrolled to.
//
// Both are needed. The travel test catches a drag on a list with nothing to
// scroll; the scroll-offset test catches the other end of it -- a press that
// landed while an inertial scroll was still settling, where the finger barely
// moves but the row under it does.
static lv_coord_t s_rowPressY = 0;
static int32_t    s_rowPressScroll = 0;
static bool       s_rowPressed = false;

static void onFaultRowPress(lv_event_t* e) {
  lv_obj_t* list = static_cast<lv_obj_t*>(lv_event_get_target(e));
  lv_indev_t* indev = lv_indev_active();
  if (list == nullptr || indev == nullptr) {
    s_rowPressed = false;
    return;
  }
  lv_point_t p;
  lv_indev_get_point(indev, &p);
  s_rowPressY = (lv_coord_t)p.y;
  s_rowPressScroll = lv_obj_get_scroll_y(list);
  s_rowPressed = true;
}

// A tap on a row OPENS that fault; it no longer mutes it.
//
// Muting from the list was a one-tap, undo-by-tapping-again toggle on a target
// that showed a code and a state and no reason -- so the decision to ignore a
// fault was being made without the description in front of the driver, and by
// the same gesture used to read one. The dialog puts the whole fault on screen
// and makes MUTE a deliberate second tap. Nothing is destroyed either way,
// which is why neither gesture needs a confirmation.
//
// ONE handler on the list, working the row out from where the finger landed,
// rather than sixteen invisible tap targets. The pool is fixed and this page
// already costs sixty-odd objects; sixteen more to answer a question the y
// coordinate already answers would be paying twice.
//
// LVGL only sends CLICKED when the press did not turn into a scroll, so this
// cannot fire while the list is being dragged -- which is exactly what makes
// tap-to-open safe on a list that also scrolls.
static void onFaultRowTap(lv_event_t* e) {
  if (s_faults == nullptr) {
    return;
  }
  lv_obj_t* list = static_cast<lv_obj_t*>(lv_event_get_target(e));
  lv_indev_t* indev = lv_indev_active();
  if (list == nullptr || indev == nullptr) {
    return;
  }
  lv_point_t p;
  lv_indev_get_point(indev, &p);

  // A DRAG, not a tap: the finger travelled, or the list moved under it. Doing
  // nothing is the whole point -- the scroll (or the attempt at one) was the
  // gesture, and opening a dialog on the way out of it is what made the list
  // feel like it could not be scrolled at all.
  // A dialog closed a moment ago: this is the rest of the contact that closed
  // it, not a new tap on the row now showing through the gap.
  if (s_faultDlgClosedMs != 0u &&
      lv_tick_elaps(s_faultDlgClosedMs) < UI_DLG_SETTLE_MS) {
    s_rowPressed = false;
    return;
  }

  const int32_t scrolled = lv_obj_get_scroll_y(list);
  if (!s_rowPressed ||
      LV_ABS((int32_t)p.y - (int32_t)s_rowPressY) > UI_FAULT_TAP_SLOP ||
      scrolled != s_rowPressScroll) {
    s_rowPressed = false;
    return;
  }
  s_rowPressed = false;

  lv_area_t a;
  lv_obj_get_coords(list, &a);
  // Into the list's own content space: screen y, less where the list starts,
  // plus however far it has been scrolled.
  const int32_t y = (int32_t)p.y - (int32_t)a.y1 + lv_obj_get_scroll_y(list);
  if (y < 0) {
    return;
  }
  const int32_t row = y / UI_FAULT_ROW_H;
  if (row >= 0 && row < (int32_t)s_faults->count) {
    faultDlgShow(true, (uint8_t)row);
  }
}

static void buildFaultsPage(lv_obj_t* root);

// The pool reading taken when the drill-down is built, PRINTED by uiTask once
// the lock is released.
//
// This is the page that grew: sixteen rows of two lines, a rule each, and a
// dialog on top, all allocated from a FIXED pool whose allocation failure is a
// bare while(1). The number that predicts that belongs in the log, and this is
// the only moment it changes. Sampled here and printed there for the same
// reason the frame-rate line is: ESP_LOGx under lvgl_port_lock blocks on the
// USB-Serial-JTAG host and stalls the renderer.
static volatile bool s_poolLogDue = false;
static uint32_t s_poolUsed = 0u;
static uint32_t s_poolTotal = 0u;
static uint32_t s_poolFrag = 0u;
static uint32_t s_poolBig = 0u;

static void faultPoolSample(void) {
  lv_mem_monitor_t m;
  lv_mem_monitor(&m);
  s_poolUsed = (uint32_t)(m.total_size - m.free_size);
  s_poolTotal = (uint32_t)m.total_size;
  s_poolFrag = (uint32_t)m.frag_pct;
  s_poolBig = (uint32_t)m.free_biggest_size;
  s_poolLogDue = true;
}

// The drill-down is built ON DEMAND and DESTROYED on the way out.
//
// It was built with the rest of the tree and merely hidden, which cost ~42
// objects resident for a page opened deliberately, from a standstill, perhaps
// once a month. The pool is a FIXED 64 KB and the five peers already sit in
// 39 KB of it; those objects pushed the permanent set high enough that the
// FIRST full-screen refresh could not allocate a draw task, and
// LV_ASSERT_MALLOC's bare while(1) hung taskLVGL with the panel still white.
// A hidden page does not draw, but it still OCCUPIES -- and what ran out was
// what render needs, not what build needs.
static void faultPageShow(bool open) {
  if (open) {
    if (s_faultPage == nullptr) {
      buildFaultsPage(s_root);
    }
    if (s_faultPage != nullptr) {
      lv_obj_remove_flag(s_faultPage, LV_OBJ_FLAG_HIDDEN);
      faultPoolSample();
    }
  } else if (s_faultPage != nullptr) {
    lv_obj_delete(s_faultPage);
    s_faultPage = nullptr;
    // These pointed INTO the tree just deleted. refresh() gates on
    // s_faultPage, but a dangling pointer that is only safe because of a
    // guard somewhere else is the next person's crash.
    W.faultTitle = nullptr;
    for (uint8_t i = 0u; i < FAULT_MAX; ++i) {
      W.faultRow[i] = nullptr;
      W.faultState[i] = nullptr;
      W.faultDesc[i] = nullptr;
      W.faultSep[i] = nullptr;
    }
    // The dialog is a CHILD of the page, so BACK with it open deletes it too.
    // Forget it without a second delete.
    faultDlgForget();
  }
}

static bool faultPageOpen(void) {
  return s_faultPage != nullptr &&
         !lv_obj_has_flag(s_faultPage, LV_OBJ_FLAG_HIDDEN);
}

// Tapping the FAULTS block on HEALTH OPENS the list. It no longer scans:
// re-reading is a button on the page itself, next to what it would replace.
static void onFaultsTap(lv_event_t* e) {
  (void)e;
  faultPageShow(true);
}

static void onFaultBack(lv_event_t* e) {
  (void)e;
  faultPageShow(false);
}

static void onFaultReread(lv_event_t* e) {
  (void)e;
  if (s_faultRescan != nullptr) {
    s_faultRescan();
  }
}

// An invisible tap target over the FAULTS caption and its line. CLICKABLE but
// NOT SCROLLABLE, so a drag that starts here still swipes the page underneath.
static void mkFaultsTap(lv_obj_t* page, lv_coord_t x, lv_coord_t y, lv_coord_t w,
                        lv_coord_t h) {
  lv_obj_t* hit = lv_obj_create(page);
  stripChrome(hit);
  lv_obj_set_size(hit, w, h);
  lv_obj_set_pos(hit, x, y);
  lv_obj_add_flag(hit, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(hit, onFaultsTap, LV_EVENT_CLICKED, nullptr);
}

static void buildHealth(lv_obj_t* page) {
  buildHeader(page, "HEALTH", nullptr);

  mkUnitKey(page, "V", UI_W - (UI_PAD - 2), 5);
  W.healthVolts = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_PAD - 4, 20);

  // Coolant and ambient moved here off DRIVE, which is now a fuel card. They
  // belong together: both are temperatures, neither changes fast enough to be
  // worth a glance while moving, and ambient had been decoded from 0x015 and
  // displayed nowhere at all.
  buildPair(page, 92, "COOLANT", "AMBIENT", &W.healthCoolant, &W.healthAmbient,
            UI_DEG "C", UI_DEG "C");

  mkRule(page, 150, 0, UI_W);
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

  mkRule(page, 214, 0, UI_W);
  mkCaption(page, "FAULTS", UI_PAD - 2, 220);
  W.faults = mkLabel(page, "not scanned", UI_FONT_LABEL, UI_DIM, UI_PAD - 2, 238);
  mkFaultsTap(page, 0, 214, UI_W, 40);
}

// Three regions rather than two: hero and temperatures share the top 156 px in
// the usual two columns, and LAMPS takes a full-width strip under both.
//
// FAULTS moves UP, into the right column above LAMPS, which is the one place
// in this band where the order genuinely changes rather than the arrangement.
// It earns it: LAMPS is the only block on the page that wants the full width,
// and putting a one-line text row in the narrow column costs nothing, while
// putting three 95 px pills there is impossible.
//
// The pills are still not driven, in either orientation. MIL stays absent:
// CCN 213F bit24 reads true in every recorded sample including at speed.
static void buildHealthLs(lv_obj_t* page) {
  const lv_coord_t topH = 156;
  buildHeader(page, "HEALTH", nullptr);
  mkVRule(page, UI_LS_DIV_X, 0, topH);

  mkUnitKey(page, "V", UI_LS_L_R, 5);
  W.healthVolts = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_LS_PAD - 2, 52);

  mkStackRow(page, 0, UI_LS_ROW_H, "COOLANT", &W.healthCoolant, UI_DEG "C");
  mkStackRow(page, 54, UI_LS_ROW_H, "AMBIENT", &W.healthAmbient, UI_DEG "C");

  // Not a value cell -- a word, in the label face. font_digits_* has no
  // letters at all, so "not scanned" could not be drawn in the condensed face
  // even if it wanted to be.
  mkCaption(page, "FAULTS", UI_LS_R_L, 109);
  W.faults = mkLabel(page, "not scanned", UI_FONT_LABEL, UI_DIM, UI_LS_R_L, 125);
  mkFaultsTap(page, UI_LS_DIV_X + 1, 104, UI_LS_RIGHT_W, 48);

  mkRule(page, topH, 0, UI_W);
  mkCaption(page, "LAMPS", UI_LS_PAD, 162);
  static const char* const kLamps[3] = {"OIL", "ABS", "CHARGE"};
  for (uint8_t i = 0; i < 3; ++i) {
    lv_obj_t* pill = mkOutline(page, static_cast<lv_coord_t>(UI_LS_PAD + i * 100), 185,
                               95, 28, 0, UI_RULE, 1);
    lv_obj_t* t = mkLabel(pill, kLamps[i], UI_FONT_LABEL, UI_FAINT, 0, 0);
    lv_obj_center(t);
  }
}

// ACCEL. Deliberately plain for now -- a table of what the last run did against
// the best ever seen, in the two bands' usual geometry, so the numbers can be
// looked at before anyone decides what the page should really be.
//
// 60-100 sits between the two it is derived from, because that is the order the
// run happens in and because it is the one that says most about the engine
// rather than about the launch.
static const char* const kAccelRowName[3] = {"0-60", "60-100", "0-100"};

static void buildAccelRows(lv_obj_t* page, lv_coord_t capX, lv_coord_t lastX,
                           lv_coord_t bestX, lv_coord_t headY, lv_coord_t row0,
                           lv_coord_t step) {
  mkCaption(page, "LAST", lastX, headY);
  mkCaption(page, "BEST", bestX, headY);
  for (uint8_t i = 0; i < 3u; ++i) {
    const lv_coord_t y = static_cast<lv_coord_t>(row0 + i * step);
    if (i != 0u) {
      mkRule(page, static_cast<lv_coord_t>(y - 8), 0, UI_W);
    }
    mkCaption(page, kAccelRowName[i], capX, static_cast<lv_coord_t>(y + 10));
    W.accLast[i] = mkLabel(page, "--", UI_FONT_MID, UI_PRIMARY, lastX, y);
    W.accBest[i] = mkLabel(page, "--", UI_FONT_MID, UI_PRIMARY, bestX, y);
  }
}

static void buildAccel(lv_obj_t* page) {
  buildHeader(page, "ACCEL", nullptr);
  mkUnitKey(page, "S", UI_W - (UI_PAD - 2), 5);
  mkRule(page, 24, 0, UI_W);
  buildAccelRows(page, UI_PAD - 2, 108, 174, 32, 52, 64);
}

static void buildAccelLs(lv_obj_t* page) {
  buildHeader(page, "ACCEL", nullptr);
  mkUnitKey(page, "S", UI_LS_R_R, 5);
  mkRule(page, 22, 0, UI_W);
  buildAccelRows(page, UI_LS_PAD, 150, 240, 30, 50, 54);
}

// Seconds to one decimal, or a dash when the run never got there.
//
// `floored` dims the whole row rather than adding a badge. A run made without
// going through the kickdown detent is a real time and worth keeping, but it
// is not the thing a 0-60 figure usually means -- and a quieter number says
// that in the space already available, which a marker would not.
static void setAccelCell(lv_obj_t* lbl, uint32_t ms, bool floored) {
  if (lbl == nullptr) {
    return;
  }
  if (ms == 0u) {
    setTextIfChanged(lbl, "--");
    setColourIfChanged(lbl, UI_FAINT);
    return;
  }
  char buf[12];
  snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(ms) / 1000.0);
  setTextIfChanged(lbl, buf);
  setColourIfChanged(lbl, floored ? UI_PRIMARY : UI_DIM);
}

// The stored-code line on HEALTH.
//
// "not scanned" and "none" are deliberately different words for deliberately
// different things. A module that never answered is not a module with nothing
// wrong, and the count is only worth reading once every module has been asked.
static void refreshFaults(void) {
  if (W.faults == nullptr || s_faults == nullptr) {
    return;
  }
  char buf[24];
  lv_color_t col = UI_DIM;
  if (faultsScanning(*s_faults)) {
    snprintf(buf, sizeof(buf), "scanning %u/%u", (unsigned)s_faults->cursor,
             (unsigned)s_faults->ecuCount);
  } else if (s_faults->doneMs == 0u) {
    snprintf(buf, sizeof(buf), "not scanned");
  } else if (s_faults->count == 0u && faultsAnsweredCount(*s_faults) == 0u) {
    // NOBODY answered. Saying "none" here would be the most confidently wrong
    // thing this screen could do: it is the difference between a clean car and
    // a scan that never reached one.
    snprintf(buf, sizeof(buf), "no reply (%u quiet)",
             (unsigned)faultsUnreachableCount(*s_faults));
    col = UI_FAINT;
  } else if (s_faults->count == 0u) {
    // Say how many modules that verdict actually covers -- "none" across four
    // modules of twelve is a different statement from "none" across all of them.
    snprintf(buf, sizeof(buf), "none (%u asked)",
             (unsigned)faultsAnsweredCount(*s_faults));
  } else {
    // The count HEALTH shows EXCLUDES muted codes. That is the whole point of
    // muting: a number that never changes is a number nobody reads, and a new
    // fault has to be able to stand out from the ones already dismissed.
    const uint8_t active = faultsActiveCount(*s_faults);
    if (active == 0u) {
      snprintf(buf, sizeof(buf), "none (%u set aside)",
               (unsigned)s_faults->count);
    } else {
      snprintf(buf, sizeof(buf), "%u stored%s", (unsigned)active,
               s_faults->full ? "+" : "");
      col = UI_CRIT;
    }
  }
  setTextIfChanged(W.faults, buf);
  setColourIfChanged(W.faults, col);
}

// The list itself. Cheap enough to run every cycle, and only while it is up.
static void refreshFaultsPage(void) {
  if (s_faultPage == nullptr || s_faults == nullptr || !faultPageOpen()) {
    return;
  }
  char buf[40];
  if (faultsScanning(*s_faults)) {
    snprintf(buf, sizeof(buf), "%u/%u", (unsigned)s_faults->cursor,
             (unsigned)s_faults->ecuCount);
  } else if (s_faults->doneMs == 0u) {
    snprintf(buf, sizeof(buf), "NOT READ");
  } else {
    snprintf(buf, sizeof(buf), "%u OF %u%s",
             (unsigned)faultsActiveCount(*s_faults), (unsigned)s_faults->count,
             s_faults->full ? "+" : "");
  }
  if (strcmp(lv_label_get_text(W.faultTitle), buf) != 0) {
    lv_label_set_text(W.faultTitle, buf);
    lv_obj_align(W.faultTitle, LV_ALIGN_TOP_RIGHT,
                 -(s_landscape ? UI_LS_PAD : (UI_PAD - 2)), 5);
  }

  for (uint8_t i = 0; i < FAULT_MAX; ++i) {
    if (i >= s_faults->count) {
      lv_obj_add_flag(W.faultRow[i], LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(W.faultState[i], LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(W.faultDesc[i], LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(W.faultSep[i], LV_OBJ_FLAG_HIDDEN);
      continue;
    }
    const Fault& f = s_faults->f[i];
    char sae[8];
    if (!kwpDtcToSae(f.code, sae, sizeof(sae))) {
      snprintf(sae, sizeof(sae), "%04X", (unsigned)f.code);
    }
    const char* mod = (f.ecu < ECU_COUNT) ? kEcus[f.ecu].name : "?";
    snprintf(buf, sizeof(buf), "%s  %s", sae, mod);
    setTextIfChanged(W.faultRow[i], buf);
    // The description the firmware already carries -- 1152 of them, compiled
    // in and binary-searched by (ecu, code). A code with no entry says so
    // rather than leaving the line blank, because an empty second line reads
    // as a rendering fault.
    const DtcDef* def = dtcLookup(f.ecu, f.code);
    setTextIfChanged(W.faultDesc[i], (def != nullptr && def->text != nullptr)
                                         ? def->text
                                         : "no description");
    setColourIfChanged(W.faultDesc[i], f.muted ? UI_FAINT : UI_LABEL);
    // The code is the thing being looked for; the module is context. Same line,
    // one colour, because two would make the list look like two lists.
    // A muted fault is still shown, still readable, and still says what it is.
    // It is only taken out of the COUNT -- so it goes quiet, not away. Anything
    // that removed it from the list would make the list a lie.
    setTextIfChanged(W.faultState[i],
                     f.muted ? "not important" : kwpDtcStatusText(f.status));
    setColourIfChanged(W.faultState[i],
                       f.muted ? UI_FAINT
                               : (((f.status & 0x20u) != 0u) ? UI_CRIT : UI_DIM));
    setColourIfChanged(W.faultRow[i], f.muted ? UI_FAINT : UI_PRIMARY);
    lv_obj_remove_flag(W.faultRow[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(W.faultState[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(W.faultDesc[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(W.faultSep[i], LV_OBJ_FLAG_HIDDEN);
  }

  if (s_faults->count == 0u) {
    setTextIfChanged(W.faultEmpty,
                     faultsScanning(*s_faults)   ? "reading..."
                     : (s_faults->doneMs == 0u)  ? "tap RE-READ to scan"
                     : (faultsAnsweredCount(*s_faults) == 0u)
                         ? "no module answered"
                         : "no faults stored");
    lv_obj_remove_flag(W.faultEmpty, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(W.faultEmpty, LV_OBJ_FLAG_HIDDEN);
  }
}

// The detail dialog's contents. Runs every cycle while it is up, which is what
// makes MUTE feel immediate: the tap only posts a request to carTask, and this
// is what notices that gFaults came back changed.
static void refreshFaultDlg(void) {
  if (s_faultDlg == nullptr || s_faults == nullptr || W.dlgCode == nullptr) {
    return;
  }
  // The fault this dialog is ABOUT, not the row it was opened from. A RE-READ
  // renumbers the list and can shorten it, and describing whatever landed on
  // that index instead is how a driver mutes the wrong fault.
  const uint8_t i = s_faultDlgIdx;
  if (i >= s_faults->count || s_faults->f[i].ecu != s_faultDlgEcu ||
      s_faults->f[i].code != s_faultDlgCode) {
    faultDlgShow(false, 0u);
    return;
  }
  const Fault& f = s_faults->f[i];

  char sae[8];
  if (!kwpDtcToSae(f.code, sae, sizeof(sae))) {
    snprintf(sae, sizeof(sae), "%04X", (unsigned)f.code);
  }
  setTextIfChanged(W.dlgCode, sae);
  setTextIfChanged(W.dlgMod, (f.ecu < ECU_COUNT) ? kEcus[f.ecu].name : "?");

  setTextIfChanged(W.dlgStatus,
                   f.muted ? "not important" : kwpDtcStatusText(f.status));
  setColourIfChanged(W.dlgStatus,
                     f.muted ? UI_FAINT
                             : (((f.status & 0x20u) != 0u) ? UI_CRIT : UI_DIM));

  const DtcDef* def = dtcLookup(f.ecu, f.code);
  setTextIfChanged(W.dlgDesc, (def != nullptr && def->text != nullptr)
                                  ? def->text
                                  : "no description for this code");
  // The one button whose WORD is the state: what it will do next, not what
  // has been done. Full-list muting is matched on (ecu, code), so a fault put
  // back here is put back everywhere it appears.
  setTextIfChanged(W.dlgMuteLbl, f.muted ? "UNMUTE" : "MUTE");

  // CLEAR says which of its four states it is in, because the only alternative
  // is a button that looks identical whether it is armed, waiting or refused.
  //
  // FAILED is not a guess: the request went to carTask, carTask transacted, and
  // the row is still here -- which for this control means the module did not
  // accept the erase. A cleared code takes the row with it and this dialog
  // closes on its own, in refreshFaultDlg's guard above.
  const char* clearWord = "CLEAR";
  if (s_clearSentMs != 0u) {
    clearWord = (lv_tick_elaps(s_clearSentMs) < UI_CLEAR_WAIT_MS) ? "CLEARING"
                                                                  : "FAILED";
  } else if (s_clearArmMs != 0u) {
    if (lv_tick_elaps(s_clearArmMs) > UI_CLEAR_ARM_MS) {
      s_clearArmMs = 0u;      // lapsed; back to needing two taps
    } else {
      clearWord = "CLEAR?";
    }
  }
  setTextIfChanged(W.dlgClearLbl, clearWord);
}

static void refreshAccel(void) {
  if (W.accLast[0] == nullptr || s_accel == nullptr) {
    return;
  }
  const AccelRun* last = accelLast(*s_accel);
  static const AccelRun kNone = {{0u, 0u}, false};
  const AccelRun& r = (last != nullptr) ? *last : kNone;
  setAccelCell(W.accLast[0], r.ms[ACCEL_SPLIT_60], r.floored);
  setAccelCell(W.accLast[1], accelMidMs(r), r.floored);
  setAccelCell(W.accLast[2], r.ms[ACCEL_SPLIT_100], r.floored);
  setAccelCell(W.accBest[0], s_accel->best.ms[ACCEL_SPLIT_60],
               s_accel->best.floored);
  // The best 60-100 is the best RUN's, not the best of each end taken
  // separately -- those two could come from different runs and the difference
  // would be a time nobody actually drove.
  setAccelCell(W.accBest[1], accelMidMs(s_accel->best), s_accel->best.floored);
  setAccelCell(W.accBest[2], s_accel->best.ms[ACCEL_SPLIT_100],
               s_accel->best.floored);
}

// The two meters share one page and one set of widgets; a tap swaps which trip
// feeds them. Two pages would have cost a swipe and a dot for what is the same
// five numbers measured over a different window.
static const char* const kTripTitle[TRIP_COUNT] = {"TRIP A", "TRIP B", "FROM START"};

// The SELECTOR's own labels, deliberately shorter than kTripTitle: three of
// them sit side by side in a 54 px cell each, and "FROM START" is 79 px in
// font_label_14 on its own. The header carries the page name now, so nothing
// in this row has to spell out which card you are looking at.
static const char* const kTripTab[TRIP_COUNT] = {"TRIP A", "TRIP B", "START"};

// Cell pitch and the underline width per tab. The widths are MEASURED, not
// estimated -- 40.5 / 40.6 / 41.4 px, from the adv_w tables in
// src/fonts/font_label_14.c -- because the artboards are CSS Barlow and render
// about 30 % narrower than the firmware face, so a coordinate taken off a board
// would have been three pixels of lie per tab.
//
// 54 x 3 = 162 px from the left pad. Portrait has 218 px of band before the KM
// key and landscape only 165, so this is sized by the NARROW band and used
// unchanged in both -- one row to keep in step instead of two.
#define UI_TRIP_SEG_W 54
#define UI_TRIP_SEG_Y 4
static const lv_coord_t kTripTabUlW[TRIP_COUNT] = {41, 41, 42};

static lv_coord_t tripSegX(uint8_t i) {
  const lv_coord_t pad = s_landscape ? UI_LS_PAD : (UI_PAD - 2);
  return static_cast<lv_coord_t>(pad + i * UI_TRIP_SEG_W);
}

// Light the active tab and slide the underline to it. Named for what it used
// to do -- set a title -- because every caller still means the same thing:
// "the shown meter changed, say so".
static void setTripTitle(void) {
  for (uint8_t i = 0u; i < TRIP_COUNT; ++i) {
    if (W.tripSeg[i] != nullptr) {
      lv_obj_set_style_text_color(W.tripSeg[i],
                                  (i == s_tripShown) ? UI_PRIMARY : UI_FAINT, 0);
    }
  }
  if (W.tripSegUl != nullptr) {
    lv_obj_set_width(W.tripSegUl, kTripTabUlW[s_tripShown]);
    lv_obj_set_pos(W.tripSegUl, tripSegX(s_tripShown),
                   static_cast<lv_coord_t>(UI_TRIP_SEG_Y + 19));
  }
}

static bool s_tripLongFired = false;

// Defined with the rest of the confirmation dialog, below.
static bool confirmOpen(void);
static void hideConfirm(void);

// A tap on the page BACKGROUND now only dismisses the dialog.
//
// It used to be the meter swap, on a double tap, with a faint line of text
// explaining that. Three named tabs replaced both: the state of the page is
// stated rather than described, getting to TRIP B from START is one tap rather
// than two-and-know-which-way-the-ring-runs, and the hint row it cost is
// exactly the row the selector now occupies.
//
// The double tap was guarding the wrong thing. Swapping which meter is on
// screen is harmless and undone by tapping the one you wanted; the destructive
// gesture is the RESET, and that is guarded by a confirmation, which is a
// better guard than making a gesture hard to perform.
//
// LVGL only sends CLICKED when the press did not turn into a scroll, so this
// cannot fire on a swipe that happens to end over the page -- which is the
// whole reason the gesture is safe to put on a page inside a scroller.
static void onTripTap(lv_event_t* e) {
  (void)e;
  // A long press ends with a release, and a release sends CLICKED. Without
  // this, letting go of a reset would be read as a fresh tap.
  if (s_tripLongFired) {
    s_tripLongFired = false;
    return;
  }
  // NOTE the latch is also cleared by onTripPress below. A long press that ends
  // in a swipe never delivers CLICKED at all -- LVGL suppresses it once the
  // press has become a scroll -- so clearing it only here left it stuck true
  // and swallowed the next real tap.
  if (confirmOpen()) {
    ESP_LOGI(TAG, "confirm: dismissed by a tap outside it");
    hideConfirm();
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

// Full width of the timeout rule, which differs between the two card sizes.
// Shared by the open (refill) and the tick (shrink), so there is one number.
static lv_coord_t s_confirmBarW = 184;

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
    lv_obj_set_width(W.confirmBar, s_confirmBarW);   // refill before it is seen
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

// Every press starts a clean gesture, whatever the last one turned into.
static void onTripPress(lv_event_t* e) {
  (void)e;
  s_tripLongFired = false;
}

// Long press asks. A trip you cannot zero is an odometer, and TRIP A in
// particular has to be clearable or it only ever grows.
static void onTripLongPress(lv_event_t* e) {
  (void)e;
  s_tripLongFired = true;
  showConfirm();
}

// A tap on a name selects that meter. One tap, not a cycle: three names are
// all on screen, so there is nothing to step through.
static void onTripSegTap(lv_event_t* e) {
  if (s_tripLongFired) {
    s_tripLongFired = false;
    return;
  }
  // The dialog is about a specific meter. Swapping out from under an open
  // "RESET TRIP A" would leave the driver confirming a reset of something
  // other than what they asked for, so a tap here dismisses and stops.
  if (confirmOpen()) {
    ESP_LOGI(TAG, "confirm: dismissed by a tap outside it");
    hideConfirm();
    return;
  }
  const uint8_t i = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (i >= TRIP_COUNT || i == s_tripShown) {
    return;
  }
  s_tripShown = i;
  setTripTitle();   // the values follow on the next refresh tick
}

// Holding a NAME asks to zero THAT meter, not whichever one happens to be on
// screen. It switches to it first, so what is being asked about is what is
// being shown -- otherwise holding TRIP B while looking at TRIP A would offer
// to clear TRIP A, which is the one mistake this page must not make.
static void onTripSegLongPress(lv_event_t* e) {
  const uint8_t i = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (i < TRIP_COUNT && i != s_tripShown) {
    s_tripShown = i;
    setTripTitle();
  }
  s_tripLongFired = true;
  showConfirm();
}

// The selector row: three names, the active one lit and underlined.
static void buildTripTabs(lv_obj_t* page) {
  for (uint8_t i = 0u; i < TRIP_COUNT; ++i) {
    const lv_coord_t x = tripSegX(i);
    W.tripSeg[i] = mkLabel(page, kTripTab[i], UI_FONT_LABEL,
                           (i == s_tripShown) ? UI_PRIMARY : UI_FAINT, x,
                           UI_TRIP_SEG_Y);

    // The TARGET is the cell, not the glyphs: 54 x 36 around a 41 x 16 label.
    // This gets used one-handed over broken tarmac, and a label-sized target
    // would be 16 px tall. Transparent, so the label still reads through it,
    // and created AFTER the label so the tap lands here rather than nowhere.
    lv_obj_t* hit = lv_obj_create(page);
    stripChrome(hit);
    lv_obj_set_size(hit, UI_TRIP_SEG_W, 30);
    lv_obj_set_pos(hit, static_cast<lv_coord_t>(x - 4), 0);
    lv_obj_add_flag(hit, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(hit, onTripSegTap, LV_EVENT_CLICKED,
                        (void*)(uintptr_t)i);
    lv_obj_add_event_cb(hit, onTripSegLongPress, LV_EVENT_LONG_PRESSED,
                        (void*)(uintptr_t)i);
    lv_obj_add_event_cb(hit, onTripPress, LV_EVENT_PRESSED, nullptr);
  }

  // stripChrome() clears CLICKABLE, so the underline cannot swallow a tap
  // meant for the cell beneath it.
  W.tripSegUl = lv_obj_create(page);
  stripChrome(W.tripSegUl);
  lv_obj_set_size(W.tripSegUl, kTripTabUlW[s_tripShown], 2);
  lv_obj_set_pos(W.tripSegUl, tripSegX(s_tripShown),
                 static_cast<lv_coord_t>(UI_TRIP_SEG_Y + 19));
  lv_obj_set_style_bg_color(W.tripSegUl, UI_PRIMARY, 0);
  lv_obj_set_style_bg_opa(W.tripSegUl, LV_OPA_COVER, 0);
}

// TRIP has NO HEADER ROW at all -- it is the only page without one.
//
// It carried the page name and a "KM" unit key, and both were dead weight
// sitting on top of the hero. The name was redundant the moment the selector
// arrived (three meters listed, one lit, says which card you are on better
// than a word does), and the unit belongs to the number, not to the page.
// Deleting the row gave the selector the top of the screen and cost nothing.
//
// The unit is drawn with mkUnit, not mkUnitKey: font_label_20 exists for unit
// KEYS parked in a header, and this is no longer one. On the baseline beside a
// value it is an ordinary inline unit, which is font_label_14 everywhere else
// on the instrument -- so it takes that face and matches buildPair's units
// rather than becoming a third size.
static void buildTrip(lv_obj_t* page) {
  buildTripTabs(page);

  W.tripKm = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_PAD - 4, 30);
  // Baselines, not boxes: font_digits_64 is line_height 73 / base_line 1, so
  // its baseline is 72 px down; font_label_14 is 16 / 3, so its own is 13 px
  // down. 30 + 72 - 13 = 89 puts the two on the same line.
  mkUnit(page, "KM", UI_W - (UI_PAD - 2), 89);

  // Time is TWO figures, and the pair is the point: IN is the whole window the
  // meter has been running, RIDE only the part spent moving. Either alone
  // misleads -- IN counts the hour in a car park, RIDE hides it -- and the gap
  // between them is the traffic.
  mkHeroSub(page, 106, UI_PAD - 2, UI_W - (UI_PAD - 2), &W.tripElapsed,
            &W.tripMoving);

  buildPair(page, 130, "USED", "AVERAGE", &W.tripUsed, &W.tripL100, "L", "L/100");
  buildPair(page, 190, "COST", "AVG SPEED", &W.tripCost, &W.tripAvgKph, UI_EUR,
            "km/h");

  // Closes the value block. It used to double as the hint row's separator; the
  // hint is gone but the rule is not, or the last pair would sit on an open
  // edge while the first two are boxed. The page dots are at page-local 278,
  // so this is the last thing that fits.
  mkRule(page, 250, 0, UI_W);

  lv_obj_add_event_cb(page, onTripTap, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(page, onTripLongPress, LV_EVENT_LONG_PRESSED, nullptr);
  lv_obj_add_event_cb(page, onTripPress, LV_EVENT_PRESSED, nullptr);
}

// TRIP, turned. Two things here are landscape-specific and both were forced by
// measurement rather than chosen:
//
// "km" MOVES INTO THE HEADER as a right-aligned key, the same move TYRES makes
// with BAR. The left column's text band is 165 px and "247.8" in
// font_digits_64 is 164 -- the number alone fills it, so a unit trailing the
// value would sit on top of the last digit. As a key it costs nothing and is
// read once rather than every glance. (Above 999.9 km the number itself
// crosses the column rule. Portrait has the same ceiling one digit further
// out; neither is fixable without a font this band is not allowed to add.)
//
// THE SELECTOR ROW is sized by THIS band, not by portrait's. Three 54 px cells
// are 162 px of the left column's 165; portrait has 218 px before the KM key
// and would have taken 56 or 60 happily. Sizing to the narrow band and using
// the result unchanged in both is what keeps the rotation a toggle instead of
// a second layout to keep in step -- the same rule the rest of this page
// follows.
static void buildTripLs(lv_obj_t* page) {
  mkVRule(page, UI_LS_DIV_X, 0, UI_PAGE_H);

  // No header row here either. The tabs start at the top of the page.
  buildTripTabs(page);

  // Hero and its sub-line are one block, centred in what the header and the
  // hint row leave: 73 of numeral, 6 of gap, 16 of line = 95 in 172.
  // The hero block -- 73 of numeral, 6 of gap, 16 of sub-line -- centred in
  // what the selector row leaves: 30..218.
  W.tripKm = mkLabel(page, "--", UI_FONT_HERO, UI_PRIMARY, UI_LS_PAD, 76);
  // 76 + 72 - 13, the same baseline arithmetic as portrait. Right-aligned to
  // the column edge rather than trailing the digits, so it holds still while
  // the number's width changes underneath it.
  mkUnit(page, "KM", UI_LS_L_R, 135);
  mkHeroSub(page, 155, UI_LS_PAD, UI_LS_L_R, &W.tripElapsed, &W.tripMoving);

  mkStackRow(page, 0, UI_LS_ROW_H, "USED", &W.tripUsed, "L");
  mkStackRow(page, 54, UI_LS_ROW_H, "AVERAGE", &W.tripL100, "L/100");
  mkStackRow(page, 108, UI_LS_ROW_H, "COST", &W.tripCost, UI_EUR);
  mkStackRow(page, 162, 56, "AVG SPEED", &W.tripAvgKph, "km/h", false);

  lv_obj_add_event_cb(page, onTripTap, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(page, onTripLongPress, LV_EVENT_LONG_PRESSED, nullptr);
  lv_obj_add_event_cb(page, onTripPress, LV_EVENT_PRESSED, nullptr);
}

// A tap target with a border and a centred legend. Deliberately large: 88 x 44
// is past the 44 px a fingertip needs, on a panel that gets used over broken
// tarmac with one hand on the wheel.
static lv_obj_t* mkButton(lv_obj_t* parent, lv_coord_t x, lv_coord_t y, lv_coord_t w,
                          lv_coord_t h, const char* text, lv_color_t colour,
                          lv_event_cb_t cb) {
  lv_obj_t* b = mkOutline(parent, x, y, w, h, 2, colour, 2);
  lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_t* t = mkLabel(b, text, UI_FONT_LABEL, colour, 0, 0);
  lv_obj_set_style_text_letter_space(t, 2, 0);
  lv_obj_center(t);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
  return b;
}

// TWO lines per fault: code and module over the description, state on the
// right, and a rule underneath so the pair reads as one 40 px target.
//
// Sixteen rows are built and the unused ones hidden, rather than built on
// demand -- allocating inside an event callback is how the fixed pool gets
// found at the worst possible moment.
static void buildFaultsPage(lv_obj_t* root) {
  const lv_coord_t pad = s_landscape ? UI_LS_PAD : (UI_PAD - 2);
  const lv_coord_t titleRule = s_landscape ? 22 : 26;
  // 1.5x the 26/40 it was, on the owner's instruction: BACK and RE-READ are
  // two words on a 16 px line and the hit targets under them were only as tall
  // as the bar. Nothing else on the page competes for the height -- the list
  // simply scrolls one row less.
  const lv_coord_t barH = s_landscape ? 39 : 60;
  const lv_coord_t botRule = static_cast<lv_coord_t>(UI_PAGE_H - barH);

  s_faultPage = lv_obj_create(root);
  stripChrome(s_faultPage);
  lv_obj_set_size(s_faultPage, UI_W, UI_PAGE_H);
  lv_obj_set_pos(s_faultPage, 0, UI_PAGE_Y);
  lv_obj_set_style_bg_color(s_faultPage, UI_BG, 0);
  lv_obj_set_style_bg_opa(s_faultPage, LV_OPA_COVER, 0);
  // CLICKABLE so it absorbs its own area -- without it a tap on the list falls
  // through to the HEALTH page underneath, which is what made the confirm
  // dialog dismiss itself wherever it was touched.
  lv_obj_add_flag(s_faultPage, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t* t = mkLabel(s_faultPage, "FAULTS", UI_FONT_LABEL, UI_PRIMARY, pad, 5);
  lv_obj_set_style_text_letter_space(t, 3, 0);
  W.faultTitle = mkLabel(s_faultPage, "", UI_FONT_LABEL, UI_DIM, 0, 0);
  lv_obj_align(W.faultTitle, LV_ALIGN_TOP_RIGHT, -pad, 5);
  mkRule(s_faultPage, titleRule, 0, UI_W);

  // The list scrolls VERTICALLY. It is on the root, not inside the page
  // scroller, so the two axes never argue over a drag.
  lv_obj_t* list = lv_obj_create(s_faultPage);
  stripChrome(list);
  lv_obj_set_size(list, UI_W, static_cast<lv_coord_t>(botRule - titleRule - 2));
  lv_obj_set_pos(list, 0, static_cast<lv_coord_t>(titleRule + 2));
  lv_obj_add_flag(list, static_cast<lv_obj_flag_t>(LV_OBJ_FLAG_SCROLLABLE |
                                                   LV_OBJ_FLAG_CLICKABLE));
  lv_obj_set_scroll_dir(list, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);
  // A list whose rows FIT cannot be dragged at all, and no flag changes that.
  //
  // Worth writing down, because the obvious fix is wrong: SCROLL_ELASTIC and
  // SCROLL_MOMENTUM are already on -- they are LVGL defaults and stripChrome()
  // only removes SCROLLABLE and CLICKABLE -- and they do nothing here anyway.
  // lv_indev_scroll.c will not even consider an object as a scroll candidate
  // unless `scroll_top > 0 || scroll_bottom > 0`, so with nothing off-screen
  // there is no scroll to be elastic ABOUT. Five faults at 40 px is 200 px of
  // portrait's 206, which is exactly that case.
  //
  // So the gesture is answered by NOT acting on it (see UI_FAULT_TAP_SLOP),
  // and the question a driver would scroll to answer -- is there more below? --
  // is answered by the header count and by the scrollbar, which appears only
  // when there is somewhere to go.
  // PRESSED as well as CLICKED: the pair is what tells a tap from a drag on a
  // list that may have nothing to scroll -- see UI_FAULT_TAP_SLOP.
  lv_obj_add_event_cb(list, onFaultRowPress, LV_EVENT_PRESSED, nullptr);
  lv_obj_add_event_cb(list, onFaultRowTap, LV_EVENT_CLICKED, nullptr);

  for (uint8_t i = 0; i < FAULT_MAX; ++i) {
    const lv_coord_t y = static_cast<lv_coord_t>(i * UI_FAULT_ROW_H);
    W.faultRow[i] = mkLabel(list, "", UI_FONT_LABEL, UI_PRIMARY, pad,
                            static_cast<lv_coord_t>(y + 3));
    W.faultState[i] = mkLabel(list, "", UI_FONT_LABEL, UI_DIM, 0, 0);
    lv_obj_align(W.faultState[i], LV_ALIGN_TOP_RIGHT, -pad,
                 static_cast<lv_coord_t>(y + 3));
    // The description gets the whole width and ONE line: 220 px holds about
    // 34 characters of font_label_14 and the median description is 40, so the
    // long ones dot rather than wrap. The whole text is in the dialog a tap
    // away, which is part of what the dialog is for.
    W.faultDesc[i] = mkLabel(list, "", UI_FONT_LABEL, UI_LABEL, pad,
                             static_cast<lv_coord_t>(y + 21));
    lv_obj_set_size(W.faultDesc[i], static_cast<lv_coord_t>(UI_W - 2 * pad), 16);
    lv_label_set_long_mode(W.faultDesc[i], LV_LABEL_LONG_MODE_DOTS);
    W.faultSep[i] = mkRule(list, static_cast<lv_coord_t>(y + UI_FAULT_ROW_H - 1),
                           0, UI_W);
    lv_obj_add_flag(W.faultRow[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(W.faultState[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(W.faultDesc[i], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(W.faultSep[i], LV_OBJ_FLAG_HIDDEN);
  }
  W.faultEmpty = mkLabel(list, "", UI_FONT_LABEL, UI_FAINT, pad, 6);

  mkRule(s_faultPage, botRule, 0, UI_W);
  const lv_coord_t barY = static_cast<lv_coord_t>(botRule + (barH - 16) / 2);

  // BACK on the left, where a thumb sweeping in from the edge lands first --
  // the same hand that reached SETTINGS reaches this.
  lv_obj_t* back = mkLabel(s_faultPage, "BACK", UI_FONT_LABEL, UI_PRIMARY, pad, barY);
  lv_obj_set_style_text_letter_space(back, 2, 0);
  lv_obj_t* reread = mkLabel(s_faultPage, "RE-READ", UI_FONT_LABEL, UI_PRIMARY, 0, 0);
  lv_obj_set_style_text_letter_space(reread, 2, 0);
  lv_obj_align(reread, LV_ALIGN_TOP_RIGHT, -pad, barY);

  // Targets far larger than the words, because this is the one page reached
  // with the truck stopped but read with a thumb.
  lv_obj_t* backHit = lv_obj_create(s_faultPage);
  stripChrome(backHit);
  lv_obj_set_size(backHit, UI_W / 2, barH);
  lv_obj_set_pos(backHit, 0, botRule);
  lv_obj_add_flag(backHit, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(backHit, onFaultBack, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* readHit = lv_obj_create(s_faultPage);
  stripChrome(readHit);
  lv_obj_set_size(readHit, UI_W / 2, barH);
  lv_obj_set_pos(readHit, UI_W / 2, botRule);
  lv_obj_add_flag(readHit, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(readHit, onFaultReread, LV_EVENT_CLICKED, nullptr);

  lv_obj_add_flag(s_faultPage, LV_OBJ_FLAG_HIDDEN);
}

// ---------------------------------------------------------------------------
// The fault detail dialog
//
// What a five-character code cannot say. The list has room for a dotted
// description and nothing else; this has the whole of it, the module, the
// status in words, and the one decision available -- mute, or put back.
//
// Built on demand and DESTROYED on close, the same discipline as the page it
// sits on: ~10 objects for a dialog opened deliberately, from a standstill, is
// not worth carrying resident in a fixed pool that the render path also takes
// its draw tasks from.
//
// It is a CHILD of the fault page, so BACK takes it with it and there is only
// ever one owner of these pointers.
// ---------------------------------------------------------------------------

static void onFaultDlgClose(lv_event_t* e) {
  (void)e;
  if (!faultDlgSettled()) {
    return;   // the click that opened this dialog, arriving twice
  }
  faultDlgShow(false, 0u);
}

static void onFaultDlgMute(lv_event_t* e) {
  (void)e;
  if (!faultDlgSettled()) {
    return;
  }
  // Posts a request; carTask owns gFaults and the NVS write. The dialog stays
  // OPEN and refreshFaultDlg() flips the button when the change lands, so the
  // driver sees what they did and can undo it with the same button. Nothing
  // here destroys anything, which is why it needs no confirmation -- unlike
  // the TRIP reset two pages over.
  if (s_faultMute != nullptr && s_faultDlgIdx < FAULT_MAX) {
    s_faultMute(s_faultDlgIdx);
  }
}

static void onFaultDlgClear(lv_event_t* e) {
  (void)e;
  if (!faultDlgSettled()) {
    return;   // and this one would have ARMED an erase on a stray click
  }
  if (s_faultClear == nullptr || s_faults == nullptr) {
    return;
  }
  if (s_clearSentMs != 0u) {
    return;   // already asked; the answer is what decides, not another tap
  }
  const uint32_t now = lv_tick_get();
  if (s_clearArmMs == 0u || lv_tick_elaps(s_clearArmMs) > UI_CLEAR_ARM_MS) {
    s_clearArmMs = (now == 0u) ? 1u : now;   // 0 is the "not armed" sentinel
    return;                                  // first tap: arm and say so
  }
  // Second tap. By (ecu, code), not by index -- see uiSetFaults().
  s_clearArmMs = 0u;
  s_clearSentMs = (now == 0u) ? 1u : now;
  s_faultClear(s_faultDlgEcu, s_faultDlgCode);
}

static void buildFaultDlg(void) {
  const bool ls = s_landscape;
  const lv_coord_t pad = 12;
  const lv_coord_t cardW = ls ? 280 : 216;
  const lv_coord_t cardH = ls ? 176 : 248;
  // THREE controls now, and the two bands cannot arrange them the same way --
  // the same thing that happened to SETTINGS.
  //
  // Landscape has the width and not the height: 218 px of page cannot take two
  // rows of 46 plus this much text, but 280 - 24 of card divides into three
  // 78 px buttons on an 11 px gap, and "UNMUTE" is 55 px at this face.
  // Portrait has the height and not the width: three 88s want 288 of 192, so
  // CLOSE and MUTE keep the row they had and CLEAR goes UNDERNEATH, full
  // width, in the critical colour. That is the better arrangement anyway --
  // the destructive control is not adjacent to the one that means "never
  // mind", and it reads as a separate decision because it is one.
  const lv_coord_t btnW = ls ? 78 : 88;
  const lv_coord_t btnH = ls ? 46 : 44;
  const lv_coord_t btnY = static_cast<lv_coord_t>(cardH - pad -
                                                  (ls ? btnH : (2 * btnH + 6)));
  const lv_coord_t descY = ls ? 58 : 62;

  // The scrim. Translucent rather than opaque: the list stays visible behind
  // the card, which is what says "still inside the fault list" instead of
  // "another page". CLICKABLE so it absorbs every tap the card does not --
  // without the flag lv_indev_search_obj() carries on to the siblings
  // underneath, which is the hole that made the TRIP confirm dialog dismiss
  // itself wherever it was touched.
  //
  // It absorbs and does NOTHING. Tap-outside-to-dismiss was tried and is what
  // made taps appear to pass through the dialog: the stray second click from
  // one contact (see UI_DLG_SETTLE_MS) landed on the list the moment the
  // dialog got out of the way, and opened the next one. The dialog closes from
  // CLOSE, which is a control the driver can see and aim at -- the same
  // arrangement the confirm dialog has always had.
  s_faultDlg = lv_obj_create(s_faultPage);
  stripChrome(s_faultDlg);
  lv_obj_set_size(s_faultDlg, UI_W, UI_PAGE_H);
  lv_obj_set_pos(s_faultDlg, 0, 0);
  lv_obj_set_style_bg_color(s_faultDlg, UI_BG, 0);
  lv_obj_set_style_bg_opa(s_faultDlg, LV_OPA_70, 0);
  lv_obj_add_flag(s_faultDlg, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t* card = lv_obj_create(s_faultDlg);
  stripChrome(card);
  lv_obj_set_size(card, cardW, cardH);
  lv_obj_set_pos(card, static_cast<lv_coord_t>((UI_W - cardW) / 2),
                 static_cast<lv_coord_t>((UI_PAGE_H - cardH) / 2));
  lv_obj_set_style_bg_color(card, UI_BG, 0);
  lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(card, UI_PRIMARY, 0);
  lv_obj_set_style_border_width(card, 2, 0);
  lv_obj_set_style_radius(card, 4, 0);
  lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);   // absorb its own area

  // The code in the 20 px LABEL face, not the digit face: font_digits_* has no
  // letters at all, and "P213A" is four of them short of renderable there.
  W.dlgCode = mkLabel(card, "", UI_FONT_UNIT, UI_PRIMARY, pad, ls ? 10 : 12);
  lv_obj_set_style_text_letter_space(W.dlgCode, 2, 0);
  W.dlgMod = mkLabel(card, "", UI_FONT_LABEL, UI_LABEL, 0, 0);
  lv_obj_align(W.dlgMod, LV_ALIGN_TOP_RIGHT, -pad, ls ? 14 : 16);
  W.dlgStatus = mkLabel(card, "", UI_FONT_LABEL, UI_DIM, pad, ls ? 36 : 40);

  // Three lines of description, wrapped and then dotted. The longest entry in
  // the table is 72 characters and three lines of this width hold ~90, so the
  // dots are a backstop rather than the normal case.
  W.dlgDesc = mkLabel(card, "", UI_FONT_LABEL, UI_PRIMARY, pad, descY);
  lv_obj_set_size(W.dlgDesc, static_cast<lv_coord_t>(cardW - 2 * pad), 51);
  lv_label_set_long_mode(W.dlgDesc, LV_LABEL_LONG_MODE_DOTS);

  // CLOSE on the left, where a thumb sweeping in from the edge lands first,
  // exactly as CANCEL sits on the confirm dialog.
  mkButton(card, pad, btnY, btnW, btnH, "CLOSE", UI_DIM, onFaultDlgClose);
  const lv_coord_t muteX = ls ? static_cast<lv_coord_t>(pad + btnW + 11)
                              : static_cast<lv_coord_t>(cardW - pad - btnW);
  lv_obj_t* mute = mkButton(card, muteX, btnY, btnW, btnH, "MUTE", UI_PRIMARY,
                            onFaultDlgMute);
  W.dlgMuteLbl = lv_obj_get_child(mute, 0);

  // CLEAR: third in the row in landscape, its own full-width row beneath in
  // portrait. UI_CRIT because it is the one control here that destroys
  // something, and it arms before it acts -- see UI_CLEAR_ARM_MS.
  const lv_coord_t clearX = ls ? static_cast<lv_coord_t>(muteX + btnW + 11) : pad;
  const lv_coord_t clearY = ls ? btnY : static_cast<lv_coord_t>(btnY + btnH + 6);
  const lv_coord_t clearW = ls ? btnW : static_cast<lv_coord_t>(cardW - 2 * pad);
  lv_obj_t* clr = mkButton(card, clearX, clearY, clearW, btnH, "CLEAR", UI_CRIT,
                           onFaultDlgClear);
  W.dlgClearLbl = lv_obj_get_child(clr, 0);
}

static void faultDlgShow(bool open, uint8_t index) {
  if (!open) {
    if (s_faultDlg != nullptr) {
      lv_obj_delete(s_faultDlg);
      // The list is about to be tappable again, under a finger that may still
      // be producing clicks from the contact that closed this.
      s_faultDlgClosedMs = lv_tick_get();
      if (s_faultDlgClosedMs == 0u) {
        s_faultDlgClosedMs = 1u;
      }
    }
    faultDlgForget();
    return;
  }
  if (s_faultPage == nullptr || s_faults == nullptr || index >= s_faults->count) {
    return;
  }
  if (s_faultDlg != nullptr) {     // re-opened on another row without closing
    lv_obj_delete(s_faultDlg);
    faultDlgForget();
  }
  s_faultDlgIdx = index;
  s_faultDlgEcu = s_faults->f[index].ecu;
  s_faultDlgCode = s_faults->f[index].code;
  // A fresh dialog is never half-way through arming a CLEAR, whatever the last
  // one was doing.
  s_clearArmMs = 0u;
  s_clearSentMs = 0u;
  buildFaultDlg();
  s_faultDlgOpenMs = lv_tick_get();
  if (s_faultDlgOpenMs == 0u) {
    s_faultDlgOpenMs = 1u;      // 0 is the "no dialog" sentinel
  }
  // Filled before the first frame it appears in, rather than showing a card of
  // empty labels until the next refresh tick.
  refreshFaultDlg();
}

static void buildConfirm(lv_obj_t* root) {
  // Landscape spends its extra width on the TARGETS, not on the text: 118 x 46
  // buttons rather than 88 x 44. A confirmation is the one place on this panel
  // where a mis-tap is expensive. A narrower card was drawn first and left a
  // slice of the hero digit standing outside it, which reads as a rendering
  // fault rather than as a page continuing underneath -- hence 272, not 232.
  const bool ls = s_landscape;
  const lv_coord_t cardW = ls ? 272 : 208;
  const lv_coord_t cardH = ls ? 152 : 156;
  const lv_coord_t btnW = ls ? 118 : 88;
  const lv_coord_t btnH = ls ? 46 : 44;
  const lv_coord_t btn2X = ls ? 140 : 108;
  const lv_coord_t barY = ls ? 136 : 140;
  s_confirmBarW = static_cast<lv_coord_t>(cardW - 2 * 12);

  // Opaque, not a translucent scrim. The page behind it is already pure black,
  // so a scrim would darken nothing; what separates the dialog from the page is
  // the border and the fact that the panel is filled at all.
  s_confirm = lv_obj_create(root);
  stripChrome(s_confirm);
  lv_obj_set_size(s_confirm, cardW, cardH);
  lv_obj_set_pos(s_confirm, static_cast<lv_coord_t>((UI_W - cardW) / 2),
                 static_cast<lv_coord_t>(ls ? 42 : 88));
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
  mkButton(s_confirm, 12, 74, btnW, btnH, "CANCEL", UI_DIM, onConfirmNo);
  mkButton(s_confirm, btn2X, 74, btnW, btnH, "RESET", UI_CRIT, onConfirmYes);

  // The timeout, drawn rather than spelled out. A rule that shortens says "this
  // is going away on its own" without asking the driver to read a number.
  W.confirmBar = mkRule(s_confirm, barY, 12, s_confirmBarW);
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
  int32_t idx = (lv_obj_get_scroll_x(s_pages) + UI_W / 2) / UI_W;
  if (idx < 0) {
    idx = 0;
  }
  if (idx >= static_cast<int32_t>(UI_SCREEN_COUNT)) {
    idx = UI_SCREEN_COUNT - 1;
  }
  return static_cast<uint8_t>(idx);
}

// One swipe moves at most one page -- and LVGL does it, not us.
//
// Left alone, a scroller with snap points predicts where the momentum would
// carry it and snaps to whichever page is nearest THAT, so a brisk flick skips
// two or three. LV_OBJ_FLAG_SCROLL_ONE (set in buildRoot) is LVGL's own answer:
// init_scroll_limits() works out the adjacent snap points when the gesture
// starts and clamps everything to them.
//
// This replaced a hand-rolled clamp on the throw animation, which was worse in
// two measurable ways. It only limited the THROW, so a long drag still slid
// visibly past two pages under the finger before being yanked back. And it
// anchored itself on a ROUNDED lv_obj_get_scroll_x() sampled at SCROLL_BEGIN --
// which is an intermediate value if a new drag starts while the previous
// settle is still animating, so a quick second flick in the same direction was
// clamped back to the page it was leaving. That was a snap-back of our own
// making, on exactly the gesture a driver makes when the first flick felt slow.
//
// The event still fires with a NULL parameter for the drag and with the
// animation for the release; only the release is interesting now.
static void onScrollBegin(lv_event_t* e) {
  lv_anim_t* a = static_cast<lv_anim_t*>(lv_event_get_param(e));
  if (a == nullptr) {
    // A swipe is an answer: the driver has decided to be somewhere else, which
    // is not a reset. Dismissed on the DRAG, not on the settle, so the dialog
    // is gone the instant the page starts moving under the finger.
    hideConfirm();
    return;
  }

  // The settle. LVGL timed this at a fixed 400 ms for any full-page move; see
  // UI_PAGE_ANIM_MS. resolve_time() runs inside lv_anim_start(), after this
  // callback returns, so a plain duration written here is what is used.
  a->duration = UI_PAGE_ANIM_MS;

  // Move the page dot NOW, to where the page is going.
  //
  // LV_EVENT_SCROLL_END fires TWICE. The first is sent from the same call that
  // merely STARTS this animation -- before scroll_x has moved a pixel -- so
  // onScrollEnd() ran against the release position and left the dot on the page
  // being left for the whole settle, then corrected. That reads exactly like a
  // swipe that did not take, which is half of what "sluggish" meant.
  //
  // The animation runs in NEGATED scroll coordinates: lv_obj_scroll_by() sets
  // its values to (-scroll_x, -scroll_x + dx) and scroll_x_anim() drives the
  // object to -v, so the settling position is -end_value.
  const int32_t pageW = UI_W;
  int32_t target = -a->end_value;
  if (target < 0) {
    target = 0;
  }
  const int32_t dest = (target + pageW / 2) / pageW;
  if (dest >= 0 && dest < static_cast<int32_t>(UI_SCREEN_COUNT)) {
    setActiveDot(static_cast<uint8_t>(dest));
  }
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

// A hero sub-line figure: a word, a number and sometimes a unit, in ONE label.
// One label rather than three because the number's width changes and separate
// neighbours would shuffle sideways every time it did.
static void setSubValue(lv_obj_t* lbl, const char* prefix, float value,
                        uint8_t decimals, const char* unit) {
  if (lbl == nullptr) {
    return;
  }
  char buf[28];
  if (value < 0.0f) {
    snprintf(buf, sizeof(buf), "%s--", prefix);
    setTextIfChanged(lbl, buf);
    setColourIfChanged(lbl, UI_FAINT);
    return;
  }
  snprintf(buf, sizeof(buf), "%s%.*f %s", prefix, static_cast<int>(decimals),
           static_cast<double>(value), unit);
  setTextIfChanged(lbl, buf);
  setColourIfChanged(lbl, UI_DIM);
}

static void setSubDuration(lv_obj_t* lbl, const char* prefix, uint32_t ms) {
  if (lbl == nullptr) {
    return;
  }
  const uint32_t mins = ms / 60000u;
  char buf[28];
  snprintf(buf, sizeof(buf), "%s%u:%02u", prefix, static_cast<unsigned>(mins / 60u),
           static_cast<unsigned>(mins % 60u));
  setTextIfChanged(lbl, buf);
  setColourIfChanged(lbl, (mins == 0u) ? UI_FAINT : UI_DIM);
}

static void refreshTrip(void) {
  if (s_trips == nullptr || s_tripShown >= TRIP_COUNT) {
    return;
  }
  const Trip& t = s_trips->t[s_tripShown];
  setOptional(W.tripKm, tripKm(t), 1);
  setOptional(W.tripUsed, tripLitres(t), 2);
  setOptional(W.tripL100, tripL100(t), 1);
  // ONE decimal, not two. The stack cell is 106 px and portrait's is 110; at
  // two decimals the numeral reaches the unit at 127.45 and runs off the panel
  // entirely past 1000. At one it is clean to 999.9, and cents on a tankful of
  // diesel are precision nobody has -- the price itself is hand-entered.
  setOptional(W.tripCost, tripCost(t, fuelPrice()), 1);
  setOptional(W.tripAvgKph, tripAvgKph(t), 0);
  // IN is the whole window the meter has been running; RIDE only the part of it
  // spent moving. The gap between them is the traffic.
  setSubDuration(W.tripElapsed, "IN ", t.elapsedMs);
  setSubDuration(W.tripMoving, "RIDE ", t.movingMs);
}

// The flow bar. Only the segments whose state actually changed are touched --
// recolouring sixteen objects ten times a second would invalidate a strip of
// the screen on every tick for a signal that moves in steps of a sixteenth.
static uint8_t s_flowLit = 0xFFu;
static bool s_flowCrit = false;

// Which unit the instantaneous readout is currently in. Latched rather than
// recomputed from speed alone, so the hysteresis band above has something to
// hold on to. Starts false: a car that has not moved yet is stopped.
static bool s_flowPerDistance = false;

static void setFlow(uint8_t lit, bool crit) {
  if (lit == s_flowLit && crit == s_flowCrit) {
    return;
  }
  const lv_color_t on = crit ? UI_CRIT : UI_PRIMARY;
  for (uint8_t i = 0; i < UI_FLOW_SEGS; ++i) {
    if (W.flowSeg[i] != nullptr) {
      lv_obj_set_style_bg_color(W.flowSeg[i], (i < lit) ? on : UI_DOT_OFF, 0);
    }
  }
  s_flowLit = lit;
  s_flowCrit = crit;
}

// How long a finished run stays up before the sub-line goes back to distance
// and cost. Long enough to read it twice at a set of lights, short enough that
// it is gone before the next thing worth looking at.
#define UI_ACCEL_HOLD_MS 20000u

static uint32_t s_accelDoneMs = 0u;    // lv_tick the last run ended at; 0 = none
static bool s_accelWasRunning = false;

// One side of the sub-line: "0-60 4.2s" once achieved, the stopwatch still
// counting while it is not, dashes when the run never got there.
//
// The dimmed running figure and the primary finished one are the whole
// readout: you can see at a glance whether the number in front of you is the
// answer or still moving.
static void setAccelSide(lv_obj_t* lbl, const char* name, uint32_t splitMs,
                         uint32_t liveMs, bool running) {
  if (lbl == nullptr) {
    return;
  }
  char buf[24];
  if (splitMs != 0u) {
    snprintf(buf, sizeof(buf), "%s %.1fs", name,
             static_cast<double>(splitMs) / 1000.0);
    setTextIfChanged(lbl, buf);
    setColourIfChanged(lbl, UI_PRIMARY);
  } else if (running) {
    snprintf(buf, sizeof(buf), "%s %.1fs", name,
             static_cast<double>(liveMs) / 1000.0);
    setTextIfChanged(lbl, buf);
    setColourIfChanged(lbl, UI_DIM);
  } else {
    snprintf(buf, sizeof(buf), "%s --", name);
    setTextIfChanged(lbl, buf);
    setColourIfChanged(lbl, UI_FAINT);
  }
}

// True while the sub-line should be showing acceleration rather than the
// journey. Also keeps the edge-detect that starts the hold timer, so it must
// be called exactly once per refresh.
static bool accelWantsSubLine(void) {
  if (s_accel == nullptr) {
    return false;
  }
  // accelLaunched(), not accelRunning(): a run is timed from the last moment
  // the car was known stopped, but it is not SHOWN until the pedal says it was
  // a launch. Otherwise the sub-line flips for every pull-away and flips back
  // a second and a half later when the run is thrown away -- which is the
  // defect this replaced, seen from the driver's seat.
  if (accelLaunched(*s_accel)) {
    s_accelWasRunning = true;
    return true;
  }
  if (s_accelWasRunning) {
    s_accelWasRunning = false;
    s_accelDoneMs = lv_tick_get();
    if (s_accelDoneMs == 0u) {
      s_accelDoneMs = 1u;      // 0 is the "never run" sentinel
    }
  }
  return s_accelDoneMs != 0u && lv_tick_elaps(s_accelDoneMs) < UI_ACCEL_HOLD_MS;
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
  // The hero's sub-line is CONTEXTUAL, and this is the one place on the
  // instrument where a readout changes meaning under the driver.
  //
  // It is allowed to, because the thing that changes it is the driver flooring
  // it from a standstill -- there is no less surprising trigger than that, and
  // as of ACCEL_LAUNCH_MS it is literally the trigger rather than nearly it. And
  // an acceleration figure is only interesting in the ten seconds around it:
  // giving it a permanent cell would have cost something useful on every other
  // drive for something read twice a year. During a launch the distance and
  // the cost of the journey are precisely what nobody is looking at.
  if (accelWantsSubLine()) {
    const bool running = accelLaunched(*s_accel);
    const AccelRun* last = accelLast(*s_accel);
    static const AccelRun kNone = {{0u, 0u}, false};
    const AccelRun& r = running ? s_accel->live : (last != nullptr ? *last : kNone);
    const uint32_t live = accelElapsedMs(*s_accel, now);

    // Reading order is 0-60, 60-100, 0-100 in both bands. Which widget carries
    // which differs, because landscape puts the pair on one line and the total
    // under it, while portrait spreads all three across one.
    lv_obj_t* const slot60 = W.driveDist;
    lv_obj_t* const slotMid = s_landscape ? W.driveCost : W.driveAccel3;
    lv_obj_t* const slot100 = s_landscape ? W.driveAccel3 : W.driveCost;

    setAccelSide(slot60, "0-60", r.ms[ACCEL_SPLIT_60], live, running);
    // 60-100 is a DIFFERENCE, so it has no meaningful live value -- it does not
    // exist until both ends of it do. Never a stopwatch.
    setAccelSide(slotMid, "60-100", accelMidMs(r), 0u, false);
    setAccelSide(slot100, "0-100", r.ms[ACCEL_SPLIT_100], live, running);
    lv_obj_remove_flag(W.driveAccel3, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(W.driveAccel3, LV_OBJ_FLAG_HIDDEN);
    // The usual: the same journey the hero measures, in distance and in money.
    // IN TANK and RANGE were the two cells TRIP A and TRIP B now hold -- the
    // tank percentage is still decoded and still polled, it is simply not
    // drawn any more.
    const Trip* start = (s_trips != nullptr) ? &s_trips->t[TRIP_START] : nullptr;
    setSubValue(W.driveDist, "", (start != nullptr) ? tripKm(*start) : -1.0f, 1,
                "km");
    setSubValue(W.driveCost, "",
                (start != nullptr) ? tripCost(*start, fuelPrice()) : -1.0f, 1,
                UI_EUR);
  }

  // WHOLE kilometres here, unlike TRIP's own hero. "km" is a 19 px unit, which
  // leaves the numeral 79 px of portrait's cell -- 999.9 only just fits and
  // 1234.5 does not. These are reference copies answering "how far on each
  // meter", and a tenth of a kilometre is not what they are for; dropping it
  // buys a ceiling of 9999 km instead of 999.
  if (s_trips != nullptr) {
    setOptional(W.tripADist, tripKm(s_trips->t[TRIP_A]), 0);
    setOptional(W.tripBDist, tripKm(s_trips->t[TRIP_B]), 0);
  }

  // Both the bar and the number come from the SMOOTHED rate, not from the last
  // raw frame. Q is a per-stroke control signal and is very spiky; an
  // unsmoothed readout at 10 Hz is a blur, and an unsmoothed bar is a flicker.
  const float lph = (s_trips != nullptr) ? tripFlowLph(*s_trips, now) : -1.0f;

  uint8_t lit = 0u;
  if (lph > 0.0f) {
    const float f = lph * UI_FLOW_SEGS / UI_FLOW_FULL_LPH + 0.5f;
    lit = (f >= (float)UI_FLOW_SEGS) ? UI_FLOW_SEGS : (uint8_t)f;
  }
  // Computed by the readout below, because the threshold is on the per-distance
  // figure and only that branch knows it.
  bool crit = false;

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
        const float l100now = lph * 100.0f / s_bcast->speedKph;
        crit = (l100now > UI_FLOW_CRIT_L100);
        snprintf(buf, sizeof(buf), "%.1f L/100", static_cast<double>(l100now));
      } else {
        snprintf(buf, sizeof(buf), "%.1f L/h", static_cast<double>(lph));
      }
      setTextIfChanged(W.flowNow, buf);
      setColourIfChanged(W.flowNow, crit ? UI_CRIT : UI_PRIMARY);
    }
  }
  setFlow(lit, crit);
}

// The low-pressure line, road wheels only.
// `now` so a corner's COLOUR can follow the same freshness its value does.
static void refreshTyreWarning(uint32_t now) {
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
  // Colour the corner ITSELF, not just the count at the foot of the page.
  // Every artboard draws the soft wheel in the critical colour; the firmware
  // only ever printed "1 LOW" and left the driver to compare four numbers
  // against a target to work out which one. On the hero feature that is the job.
  //
  // But a low corner still obeys HOLD-AND-DIM. If the WCM stops answering, the
  // reading is held and every other value on the panel visibly dims -- and a
  // held corner painted UI_CRIT would be the loudest, most confident thing on
  // the screen while being the stalest. That is exactly the frozen-but-
  // plausible failure this instrument is built to avoid, on the one page it
  // cannot afford it. Stale goes to UI_CRIT_DIM: still a warning, visibly not
  // a live one.
  //
  // ORDERING: this runs after setValScaled() has written every corner's normal
  // colour in refresh(). It only ever paints low corners, so a corner that
  // comes back up is restored by that call, not by this one -- swap the two and
  // a recovered corner stays red for good.
  for (uint8_t i = 0; i < 4; ++i) {
    if (W.tyre[i] == nullptr || !slotUsable(s_slot.tyre[i])) {
      continue;
    }
    if (s_state->slot(s_slot.tyre[i])->value * UI_PSI_TO_BAR < UI_TYRE_LOW_BAR) {
      setColourIfChanged(W.tyre[i],
                         s_state->fresh(s_slot.tyre[i], now, UI_TYRE_TTL_MS)
                             ? UI_CRIT
                             : UI_CRIT_DIM);
    }
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
    const int32_t full = s_confirmBarW;
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
  // Both against TTLs matched to their own poll rates -- see UI_TYRE_TTL_MS.
  for (uint8_t i = 0; i < 4; ++i) {
    setValScaled(W.tyre[i], s_slot.tyre[i], 2, now, UI_PSI_TO_BAR, UI_TYRE_TTL_MS);
  }
  setValScaled(W.spare, s_slot.spare, 2, now, UI_PSI_TO_BAR, UI_SPARE_TTL_MS, UI_DIM,
               UI_FAINT);
  refreshTyreWarning(now);
  refreshFaults();
  refreshFaultsPage();
  refreshFaultDlg();
  refreshAccel();
  refreshSettings(now);
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
      // Applied here rather than in the tap handler: rebuilding deletes the
      // object that was tapped, and LVGL will not survive that happening
      // inside its own event dispatch.
      const uint8_t req = s_themeReq;
      if (req != 0xFFu) {
        s_themeReq = 0xFFu;
        if (s_splashDone) {
          uiApplyTheme(req);
        }
      }
      priceCommitIfDue();
      const uint8_t oreq = s_orientReq;
      if (oreq != 0xFFu) {
        s_orientReq = 0xFFu;
        if (s_splashDone) {
          uiApplyOrientation(oreq);
        }
      }
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
    if (s_poolLogDue) {
      s_poolLogDue = false;
      ESP_LOGI(TAG, "faults page: lvgl heap %u/%u used  frag %u%%  largest free %u",
               (unsigned)s_poolUsed, (unsigned)s_poolTotal, (unsigned)s_poolFrag,
               (unsigned)s_poolBig);
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
  // The mark is drawn at a fixed 240 px width in BOTH orientations and simply
  // centred. It is a logo, not a layout: re-proportioning it would mean a
  // second set of radii and stroke widths to keep in step, for no gain on a
  // screen that is visible for under two seconds.
  const lv_coord_t dx = static_cast<lv_coord_t>((UI_W - 240) / 2);
  const lv_coord_t cy = s_landscape ? 120 : 143;
  const lv_coord_t gx = static_cast<lv_coord_t>(dx + 60);
  const lv_coord_t gy = static_cast<lv_coord_t>(cy - 15);
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
  static const lv_coord_t kLampCx[4] = {18, 41, 199, 222};
  static const lv_coord_t kLampR[4] = {16, 14, 14, 16};
  for (uint8_t i = 0; i < 4; ++i) {
    const lv_coord_t d = static_cast<lv_coord_t>(kLampR[i] * 2);
    mkOutline(s_splash, static_cast<lv_coord_t>(dx + kLampCx[i] - kLampR[i]),
              static_cast<lv_coord_t>(cy - kLampR[i]), d, d, LV_RADIUS_CIRCLE,
              UI_PRIMARY, 2);
  }

  // The identity block hangs off the bottom of the screen rather than off the
  // mark, so it stays at the same distance from the edge in either band.
  const lv_coord_t idY = s_landscape ? 172 : 250;
  mkRule(s_splash, idY, (UI_W - 48) / 2, 48);
  lv_obj_t* name = mkLabel(s_splash, "LATONITA-CARPUTER", UI_FONT_LABEL, UI_DIM, 0, 0);
  lv_obj_set_style_text_letter_space(name, 2, 0);
  lv_obj_align(name, LV_ALIGN_TOP_MID, 0, static_cast<lv_coord_t>(idY + 12));
  lv_obj_t* ver = mkLabel(s_splash, "v1.0", UI_FONT_LABEL, UI_FAINT, 0, 0);
  lv_obj_align(ver, LV_ALIGN_TOP_MID, 0, static_cast<lv_coord_t>(idY + 32));
}

void uiSetBroadcast(const BroadcastState* bcast) {
  s_bcast = bcast;
}

void uiSetAccel(const AccelSet* accel) {
  s_accel = accel;
}

void uiSetFaults(const FaultSet* faults, void (*rescanFn)(void),
                 void (*muteFn)(uint8_t),
                 void (*clearFn)(uint8_t, uint16_t)) {
  s_faults = faults;
  s_faultRescan = rescanFn;
  s_faultMute = muteFn;
  s_faultClear = clearFn;
}

void uiSetTrips(const TripSet* trips, void (*resetFn)(uint8_t)) {
  s_trips = trips;
  s_tripReset = resetFn;
}

void uiSetLinkInfo(const char* mode, const char* adapter) {
  if (mode != nullptr) {
    snprintf(s_linkMode, sizeof(s_linkMode), "%s", mode);
  }
  if (adapter != nullptr) {
    snprintf(s_linkAdapter, sizeof(s_linkAdapter), "%s", adapter);
  }
}

void uiLinkReady(bool ok) {
  // Recorded before the lock, so the state survives even if the UI is mid
  // rebuild and this call cannot reach a widget: the next buildStatusBar()
  // will pick it up.
  s_linkOk = ok;
  if (!lvgl_port_lock(200)) {
    return;
  }
  if (W.linkDot != nullptr) {
    lv_obj_set_style_bg_color(W.linkDot, linkDotColour(), 0);
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

// Everything that lives on the root screen, built from the ACTIVE palette.
//
// A widget's colour is baked in when it is created, so a theme change cannot
// repaint what already exists -- it has to be built again. Extracted here so
// uiApplyTheme() can do exactly that without duplicating a line of layout.
static void buildRoot(void) {
  memset(&W, 0, sizeof(W));
  s_confirm = nullptr;
  s_confirmStart = 0u;
  s_faultPage = nullptr;
  // Caches that describe WIDGETS THAT NO LONGER EXIST.
  //
  // setFlow() only repaints when the level changed, so leaving s_flowLit alone
  // across a rebuild leaves a freshly-built, all-dark bar agreeing with a
  // stale count -- reading zero flow beside a flowNow label saying 10.2 L/100,
  // and staying there until the engine happens to change draw. 0xFF is "no
  // level known", which forces the next refresh to paint all sixteen.
  //
  // The bus-rate window is the milder version of the same thing: it would just
  // leave the BUS field blank for a couple of seconds.
  //
  // The link dot is the third, and it is handled differently -- see s_linkOk.
  // It cannot be cleared here, because the handshake that set it may never
  // happen again while the cable stays plugged in, so the state has to be
  // REPLAYED into the new widget rather than forgotten.
  s_flowLit = 0xFFu;
  s_flowCrit = false;
  s_busLastMs = 0u;

  s_root = mkScreen();

  // The scroller: a flex row of full-width pages with centre snapping, so a
  // drag moves the pages under the finger and release settles on the nearest.
  s_pages = lv_obj_create(s_root);
  stripChrome(s_pages);
  lv_obj_set_size(s_pages, UI_W, UI_PAGE_H);
  lv_obj_set_pos(s_pages, 0, UI_PAGE_Y);
  lv_obj_set_flex_flow(s_pages, LV_FLEX_FLOW_ROW);
  lv_obj_set_style_pad_column(s_pages, 0, 0);
  // CLICKABLE as well as SCROLLABLE, and the pages below are clickable too.
  // LVGL hit-tests for a clickable object to decide what was pressed, then
  // walks UP that object's parents to find something scrollable. Strip
  // clickability from everything and there is no press target at all, so the
  // scroller never learns a finger is on it -- the drag simply vanishes.
  // SCROLL_ONE is what makes one flick move one page; see onScrollBegin.
  lv_obj_add_flag(s_pages, static_cast<lv_obj_flag_t>(LV_OBJ_FLAG_SCROLLABLE |
                                                      LV_OBJ_FLAG_CLICKABLE |
                                                      LV_OBJ_FLAG_SCROLL_ONE));
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
    lv_obj_set_size(s_page[i], UI_W, UI_PAGE_H);
    // The press target. Decorations stay non-clickable so they fall through to
    // here, and from here LVGL finds the scroller one level up.
    lv_obj_add_flag(s_page[i], LV_OBJ_FLAG_CLICKABLE);
  }
  // Same five peers, same order, same one-flick-one-page rule. Only the
  // arrangement inside each page differs -- which is the whole reason the
  // orientation can be a settings row rather than a second product.
  if (s_landscape) {
    buildTripLs(s_page[UI_TRIP]);
    buildDriveLs(s_page[UI_DRIVE]);
    buildTyresLs(s_page[UI_TYRES]);
    buildHealthLs(s_page[UI_HEALTH]);
    buildAccelLs(s_page[UI_ACCEL]);
    buildSettingsLs(s_page[UI_SETTINGS]);
  } else {
    buildTrip(s_page[UI_TRIP]);
    buildDrive(s_page[UI_DRIVE]);
    buildTyres(s_page[UI_TYRES]);
    buildHealth(s_page[UI_HEALTH]);
    buildAccel(s_page[UI_ACCEL]);
    buildSettings(s_page[UI_SETTINGS]);
  }

  // Chrome last, so it paints over the pages sliding beneath it.
  buildStatusBar(s_root);
  buildDots(s_root);
  // NOT buildFaultsPage() -- see faultPageShow(). It is built when opened.
  buildConfirm(s_root);   // last of all: nothing may paint over a dialog
}

// Half one of a screen swap: stand a bare screen up and delete the whole tree.
// Returns the placeholder, to be handed back to rootRebuild().
//
// PEAK memory is what matters here, not total. LVGL has a FIXED 64 KB pool
// (CONFIG_LV_MEM_SIZE_KILOBYTES, with CONFIG_LV_MEM_POOL_EXPAND_SIZE 0) and one
// screen's worth of widgets is a large fraction of it. An earlier version of
// this built the replacement BEFORE freeing the original, so both existed at
// once; the allocation failed inside lv_label_set_text(), and with
// CONFIG_LV_USE_ASSERT_MALLOC on, LVGL's assert handler is a bare while(1).
// The UI task span, IDLE starved, and the task watchdog fired every five
// seconds forever. The screen simply froze.
//
// So: FREE first, BUILD second. LVGL must always have an active screen, so a
// bare one stands in for the moment in between -- one object, painted in the
// palette that is active WHEN THIS IS CALLED, so the swap flashes the
// destination rather than the thing being replaced.
//
// It is split in two halves rather than written straight through because the
// gap between them is the only safe place to change the display rotation:
// lv_display_set_rotation() resizes every screen on the display and marks
// every object's layout dirty, and at that moment there is nothing to resize.
static lv_obj_t* rootTeardown(void) {
  lv_obj_t* gap = lv_obj_create(nullptr);
  stripChrome(gap);
  lv_obj_set_style_bg_color(gap, UI_BG, 0);
  lv_obj_set_style_bg_opa(gap, LV_OPA_COVER, 0);
  lv_screen_load(gap);

  if (s_root != nullptr) {
    lv_obj_delete(s_root);    // the whole previous tree, before a byte of the new
    s_root = nullptr;
  }
  // The drill-down and its dialog hang off the root too. buildRoot() clears
  // both pointers, but that is the OTHER half of the rebuild: between the
  // delete above and it running, they point into freed memory. Cleared here so
  // the gap is not a window anyone has to reason about. (Today the page
  // absorbs its own area and the swipe cannot reach SETTINGS underneath it, so
  // no flip or palette swap can start while it is open -- an argument about
  // reachability, which is not what a use-after-free should stand on.)
  s_faultPage = nullptr;
  faultDlgForget();
  return gap;
}

// Half two: build the replacement, put the driver back where they were, show it.
static void rootRebuild(lv_obj_t* gap, uint8_t page, uint8_t trip) {
  buildRoot();

  lv_obj_update_layout(s_root);
  lv_obj_scroll_to_x(s_pages, page * UI_W, LV_ANIM_OFF);
  s_current = page;
  s_tripShown = trip;
  setTripTitle();
  setActiveDot(page);

  lv_screen_load(s_root);
  lv_obj_delete(gap);

  // Headroom, every time. 64 KB is not a lot for five pages of widgets, and
  // the failure mode above is a silent freeze rather than an error -- so the
  // number that predicts it belongs in the log where it can be seen coming.
  lv_mem_monitor_t m;
  lv_mem_monitor(&m);
  ESP_LOGI(TAG, "rebuilt %s %dx%d  lvgl heap %u/%u used  frag %u%%  largest free %u",
           kUiOrientName[s_orient], (int)UI_W, (int)UI_H,
           (unsigned)(m.total_size - m.free_size), (unsigned)m.total_size,
           (unsigned)m.frag_pct, (unsigned)m.free_biggest_size);
}

// Swap the palette and rebuild the screen around it. A widget's colour is baked
// in when it is created, so a theme change cannot repaint what already exists.
static void uiApplyTheme(uint8_t theme) {
  const uint8_t page = s_current;
  const uint8_t trip = s_tripShown;

  uiThemeSet(theme);          // before the placeholder, so it flashes the new BG
  rootRebuild(rootTeardown(), page, trip);
  ESP_LOGI(TAG, "theme -> %s", kUiThemeName[theme]);
}

// Turn the instrument a quarter turn and rebuild it. Same five pages, same
// order, same page under the driver's thumb when it comes back.
//
// The rotation is ONE call. LVGL swaps the resolution it reports, which is
// what UI_W / UI_H read; esp_lvgl_port hears LV_EVENT_RESOLUTION_CHANGED and
// writes swap_xy/mirror into the ST7789; and lv_indev runs every touch point
// through lv_display_rotate_point() before any widget sees it. The panel and
// the digitiser cannot end up disagreeing, because only one of them is ever
// told -- which is why Touch.cpp is not involved here and must not be.
static void uiApplyOrientation(uint8_t orient) {
  if (orient >= UI_ORIENT_COUNT || orient == s_orient) {
    return;
  }

  // Turning the panel end for end between the two landscape ways round does
  // not move a single coordinate: same width, same height, same tree. Only a
  // change of ASPECT needs the tree thrown away, so that flip is the rotation
  // call on its own -- no allocation, no teardown, and no black frame in the
  // middle of it.
  if (s_landscape == (orient != UI_ORIENT_PORTRAIT)) {
    setOrientVars(orient);
    lv_display_set_rotation(s_disp, orientRotation(s_orient));
    ESP_LOGI(TAG, "orientation -> %s (same shape, no rebuild)",
             kUiOrientName[s_orient]);
    return;
  }

  const uint8_t page = s_current;
  const uint8_t trip = s_tripShown;

  lv_obj_t* gap = rootTeardown();

  setOrientVars(orient);
  lv_display_set_rotation(s_disp, orientRotation(s_orient));

  rootRebuild(gap, page, trip);
}

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
  // The BASE orientation, which everything else is composed against.
  //
  // Not, as an earlier comment here claimed, a no-op that "cancels out"
  // Display.cpp's mirror: lvgl_port_add_disp() calls its rotation update
  // unconditionally, so these three zeroes are written straight into MADCTL and
  // REPLACE whatever Display.cpp set. They are the panel's orientation now.
  //
  // They are also what LV_DISPLAY_ROTATION_90 is composed with -- see
  // lvgl_port_disp_rotation_update() -- so landscape resolves to
  // swap_xy(true) + mirror(false, true). If the landscape image comes up
  // upside down, the fix is LV_DISPLAY_ROTATION_270 in uiApplyOrientation(),
  // one line, and the touch follows it automatically.
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

  // Orientation before a single widget exists. Every coordinate in every
  // builder is computed from UI_W / UI_H, and the panel has to already agree
  // with them -- rotating afterwards would resize the tree without moving
  // anything inside it.
  // Clamped on the way in: a stored price outside the band would make the row
  // impossible to walk back to a sane value at a cent a tap.
  s_priceCents = settingsGet().fuelPriceCents;
  if (s_priceCents < UI_PRICE_MIN || s_priceCents > UI_PRICE_MAX) {
    s_priceCents = 201u;
  }

  setOrientVars(settingsGet().orientation);
  if (s_orient != UI_ORIENT_PORTRAIT) {
    lv_display_set_rotation(s_disp, orientRotation(s_orient));
  }

  buildRoot();

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
  lv_obj_scroll_to_x(s_pages, UI_HOME * UI_W, LV_ANIM_OFF);
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
  ESP_LOGI(TAG, "dashboard up: %u pages, %s %dx%d",
           static_cast<unsigned>(UI_SCREEN_COUNT), kUiOrientName[s_orient],
           (int)UI_W, (int)UI_H);
  return ESP_OK;
}
