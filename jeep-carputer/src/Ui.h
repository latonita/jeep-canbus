#pragma once
// The dashboard. Four swipeable peer screens plus a welcome screen, driven
// entirely from VehicleState.
//
// Navigation is one horizontal scroller: the pages track the finger and snap.
// There is deliberately NO idle timeout -- whatever page the driver left the
// screen on is the page it is still on an hour later.
//
// This module owns the LVGL port. Do not also call uiLogInit() -- both
// initialise esp_lvgl_port and the second one will fail.
#include "esp_err.h"

class VehicleState;
struct BroadcastState;
struct TripSet;
struct AccelSet;
struct FaultSet;

// Peer screens, in swipe order, ranked by how often a driver reaches for them.
// DRIVE and TRIP are the two cards that change while the vehicle is moving and
// sit adjacent; TYRES and HEALTH are checked deliberately, usually before
// setting off, so they can afford to be further out.
//
// DRIVE is FIRST, and is where the boot settles. An earlier order put TRIP at
// index 0 so that DRIVE sat one in from the end and could be reached from
// either direction in one swipe. That was a real property and it was given up
// deliberately: the card wanted most often should cost NO flicks rather than
// one from each side, and a hard end is its own landmark -- there is nothing to
// overshoot into off the front of the list.
//
// ACCEL sits between HEALTH and SETTINGS: it is read stopped, like SETTINGS,
// but it is read far more often than SETTINGS is, so it costs one flick less.
//
// TILT is deliberately absent. It is designed (design/Tilt.dc.html) and was
// built, but no IMU is wired, so it could only ever show dashes -- and a card
// that is permanently empty teaches the driver to swipe past it, which costs a
// swipe on every card that follows. It returns when there is something to put
// on it.
enum UiScreenId {
  UI_DRIVE = 0,
  UI_TRIP,
  UI_TYRES,
  UI_HEALTH,
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

// Point the UI at the passively-monitored values. Optional, and safe to call
// before uiInit(). Anything available here is preferred over the polled copy:
// it arrives far more often and costs no bus traffic.
void uiSetBroadcast(const BroadcastState* bcast);

// Point the TRIP page at the trip meters, and give it a way to clear one.
// `resetFn` is called from the LVGL task on a long press with the TripId to
// clear; it must not block for long and should do the actual mutation wherever
// the trips are owned, so only one task ever writes them.
void uiSetTrips(const TripSet* trips, void (*resetFn)(uint8_t which));

// Point DRIVE at the acceleration timer. Optional: without it the hero's
// sub-line simply never switches out of its usual distance and cost.
void uiSetAccel(const AccelSet* accel);

// Point HEALTH at the stored trouble codes, and give it a way to ask for a
// fresh scan. `rescanFn` is called from the LVGL task on a tap; it must not
// block, and should set a flag for whichever task owns the link -- the scan
// itself is twelve ISO-TP transactions and belongs nowhere near the renderer.
// `muteFn` is called with a list index when the driver taps a fault to mark it
// unimportant. Like the rescan, it must only REQUEST: the fault set is walked
// and written by the link task, and a second writer would be a race.
// `clearFn` asks the module that reported a fault to ERASE it, and is given
// the fault by (ecu, code) rather than by list index. That is not symmetry with
// muteFn, it is the difference between the two: a mute is reversible with the
// same tap, while an erase is not, and a scan completing between the tap and
// the link task consuming it renumbers the list. An index would then name a
// different fault, and the wrong code would be destroyed. (ecu, code) cannot.
void uiSetFaults(const FaultSet* faults, void (*rescanFn)(void),
                 void (*muteFn)(uint8_t index),
                 void (*clearFn)(uint8_t ecu, uint16_t code));

// Brings up LVGL on the panel, builds every screen, and starts the refresh
// task. `state` must outlive the UI; it is read, never written.
esp_err_t uiInit(VehicleState* state);

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
