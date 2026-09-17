#pragma once
// The dashboard. Four swipeable peer screens plus a welcome screen, driven
// entirely from one resolved Snapshot -- see uiSetSnapshotReader() below.
//
// Navigation is one horizontal scroller: the pages track the finger and snap.
// There is deliberately NO idle timeout -- whatever page the driver left the
// screen on is the page it is still on an hour later.
//
// This module owns the LVGL port. Do not also call uiLogInit() -- both
// initialise esp_lvgl_port and the second one will fail.
#include "esp_err.h"

#include "Snapshot.h"


// Peer screens, in swipe order, ranked by how often a driver reaches for them.
// DRIVE and TRIP are the two cards that change while the vehicle is moving and
// sit adjacent; HEALTH and TYRES are checked deliberately, usually before
// setting off, so they can afford to be further out.
//
// HEALTH BEFORE TYRES on the owner's instruction. No reason was given and none
// is invented here -- it is a preference about their own dashboard and that is
// enough. Recorded because it cuts against something else in CLAUDE.md worth
// knowing about: TPMS is called the hero feature of this instrument, and this
// puts it one flick further from DRIVE than the page that is mostly a voltage
// and two temperatures. If the order ever looks wrong to someone reading the
// rationale rather than driving the truck, this is why.
//
// DRIVE is FIRST, and is where the boot settles. An earlier order put TRIP at
// index 0 so that DRIVE sat one in from the end and could be reached from
// either direction in one swipe. That was a real property and it was given up
// deliberately: the card wanted most often should cost NO flicks rather than
// one from each side, and a hard end is its own landmark -- there is nothing to
// overshoot into off the front of the list.
//
// ACCEL sits between the deliberate-reading cards and SETTINGS: it is read
// stopped, like SETTINGS, but far more often, so it costs one flick less.
//
// TILT is deliberately absent. It is designed (design/Tilt.dc.html) and was
// built, but no IMU is wired, so it could only ever show dashes -- and a card
// that is permanently empty teaches the driver to swipe past it, which costs a
// swipe on every card that follows. It returns when there is something to put
// on it.
// COMPASS sits third, between the two trip cards and the deliberate-reading
// ones, on the owner's instruction. It is a MOVING-VEHICLE card -- it says
// nothing at a standstill that the status-bar tape does not already say -- so it
// belongs with DRIVE and TRIP rather than behind the pages read while parked.
//
// It costs HEALTH, TYRES and ACCEL one flick each, and that is the trade being
// made knowingly: three pages read deliberately, from a standstill, pay so that
// one read while moving does not.
//
// The status-bar tape STAYS. The two are not duplicates: the tape is the glance
// you get on whatever page you are already on, this is the one you go to.
enum UiScreenId {
  UI_DRIVE = 0,
  UI_TRIP,
  UI_COMPASS,
  UI_HEALTH,
  UI_TYRES,
  UI_ACCEL,
  UI_SETTINGS,
  UI_SCREEN_COUNT,
};

// SETTINGS is deliberately last, and now costs four flicks rather than three.
// It is the only page a driver reads while stopped, so it should be the most
// expensive to reach, and putting it at an end means no driving screen is ever
// behind it.
//
// Where the boot settles. It is also index 0, so the scroller's natural resting
// place and UI_HOME finally agree -- but do not let that tempt you into
// dropping the explicit scroll_to_x in uiInit(). It is what makes the two
// facts independent, and the day HOME moves again the layout has to be
// resolved before the offset means anything.
#define UI_HOME UI_DRIVE

// ---------------------------------------------------------------------------
// HOW THE RENDERER GETS ITS DATA, and why it is one call and not five.
//
// It used to be five: uiSetBroadcast, uiSetTrips, uiSetAccel, uiSetFaults and
// uiInit(state) each handed over a pointer to a LIVE structure that another
// task kept mutating. Three things were wrong with that, and all three were
// invisible until looked for.
//
//   * The renderer read structures mid-mutation. The fault list is
//     insertion-sorted at end-of-scan while the drill-down walks it, which is
//     a torn STRUCTURE rather than a torn word: rows could appear twice or not
//     at all for a frame.
//   * Nothing on the panel agreed about what "now" was. Each readout sampled
//     the clock in its own refresh function, so two numbers rendered in the
//     same frame could be judged against different instants.
//   * Every consumer decided staleness for itself, five slightly different
//     ways, in a translation unit bound to LVGL and therefore untestable.
//
// So the renderer now reads ONE structure, resolved once, in core/ where the
// laptop can test it -- and it reads its own PRIVATE COPY of it, so a frame
// cannot change under the brush. `readFn` is called from the LVGL task at the
// top of each refresh and must fill `out` atomically with respect to whoever
// publishes; main.cpp does that under a spinlock, which is a few microseconds
// for ~650 bytes.
//
// Safe to call before uiInit(). Without it the panel renders an empty
// snapshot, which is dashes everywhere -- honest, and exactly what a board
// with no link should show.
void uiSetSnapshotReader(void (*readFn)(Snapshot& out));

// Give the TRIP page a way to clear a meter. Called from the LVGL task on a
// long press with the TripId to clear; it must not block for long and should
// do the actual mutation wherever the trips are owned, so only one task ever
// writes them.
void uiSetTripReset(void (*resetFn)(uint8_t which));

// Give HEALTH a way to ask for a fresh scan, to mute a fault, and to erase
// one. `rescanFn` is called from the LVGL task on a tap; it must not block,
// and should set a flag for whichever task owns the link -- the scan itself is
// twelve ISO-TP transactions and belongs nowhere near the renderer.
// `muteFn` marks a fault unimportant; `clearFn` asks its module to ERASE it.
// Like the rescan, both must only REQUEST: the fault set is walked and written
// by the link task, and a second writer would be a race.
//
// BOTH take (ecu, code), never a list index. A scan completing between the tap
// and the link task consuming the request re-sorts the list, and the renderer
// is reading a published snapshot that can itself be a pass behind, so an
// index names whatever has since moved into that row. For the erase that is
// the one mistake that cannot be undone; for the mute it is merely the wrong
// row, undone by tapping the right one. The mute carried an index for a while
// on exactly that reasoning -- and having two request shapes where one of them
// can be wrong bought nothing, so now neither can be.
void uiSetFaultActions(void (*rescanFn)(void),
                       void (*muteFn)(uint8_t ecu, uint16_t code),
                       void (*clearFn)(uint8_t ecu, uint16_t code));

// Brings up LVGL on the panel, builds every screen, and starts the refresh
// task.
//
// It no longer takes a VehicleState. Resolving signal SLOTS was the last
// reason the renderer knew anything about the hub, and that job moved to
// main.cpp, which builds the SnapshotSlots once at startup. The renderer's
// only input is now the snapshot.
esp_err_t uiInit(void);

// Show the welcome screen, then settle on DRIVE once `holdMs` has elapsed AND
// uiLinkReady() has been called -- whichever is later, bounded by a ceiling so
// a dead gateway cannot hang the boot.
void uiSplash(uint32_t holdMs);

// Tell the UI the link handshake finished. Safe to call from any task, and
// safe to never call: the splash gives up on its own.
void uiLinkReady(bool ok);

// Adapter identity for the SETTINGS info block. `mode` is "STREAM", "ATMA" or
// "DOWN"; `adapter` is a short version string such as "v2.1". Both are copied,
// so the caller's buffers need not outlive the call. Either may be null, which
// leaves that field showing dashes.
//
// Read-only by design: this block reports what the link is doing and offers no
// way to change it. A settings page that can drop the link eventually will,
// and the link is the entire device.
void uiSetLinkInfo(const char* mode, const char* adapter);

// Move one page along the swipe axis from outside the touchscreen: -1 left,
// +1 right. Safe to call from any task -- it only accumulates a request, which
// the LVGL task consumes on its next pass.
//
// Steps ACCUMULATE rather than replace, so two presses inside one 100 ms UI
// tick move two pages. They CLAMP at both ends rather than wrapping: the ends
// of the axis are landmarks, and a control that wraps SETTINGS round to DRIVE
// takes that away.
//
// Ignored while the welcome screen or the splash replay owns the display --
// moving a scroller nobody can see would leave the driver on a page they never
// chose.
void uiPageStep(int8_t dir);

// A counter the renderer advances once per completed refresh pass, ~10 Hz.
//
// It is proof the LVGL side is ALIVE, not merely scheduled: it advances only
// after a pass has run to completion under `lvgl_port_lock`. A caller that
// watches it stall is watching the one failure the ESP task watchdog cannot
// see -- a deadlock on that lock yields the CPU, so the idle tasks keep running
// and nothing else ever notices the screen has stopped.
//
// Safe to call from any task; it is a single aligned word.
uint32_t uiHeartbeat(void);
