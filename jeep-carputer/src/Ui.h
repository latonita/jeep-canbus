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

// Peer screens, in swipe order, ranked by how often a driver reaches for them.
// TRIP and DRIVE are the two cards that change while the vehicle is moving and
// sit adjacent; TYRES and HEALTH are checked deliberately, usually before
// setting off, so they can afford to be further out.
//
// NOTE the index-0 screen is NOT the one the boot settles on -- see UI_HOME.
// TRIP is first in the list, and the splash still lands on DRIVE.
//
// TILT is deliberately absent. It is designed (design/Tilt.dc.html) and was
// built, but no IMU is wired, so it could only ever show dashes -- and a card
// that is permanently empty teaches the driver to swipe past it, which costs a
// swipe on every card that follows. It returns when there is something to put
// on it.
enum UiScreenId {
  UI_TRIP = 0,
  UI_DRIVE,
  UI_TYRES,
  UI_HEALTH,
  UI_SCREEN_COUNT,
};

// Where the boot settles, and the only screen the driver can reach from either
// direction with one swipe. Being a page IN from the end is the point: TRIP is
// one flick left, TYRES one flick right.
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
