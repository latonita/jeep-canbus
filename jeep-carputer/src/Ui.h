#pragma once
// The dashboard. Five swipeable peer screens plus a welcome screen, driven
// entirely from VehicleState.
//
// Navigation has two axes: horizontal moves between peers and wraps, which is
// the only axis a moving driver uses; vertical is for depth. After a period
// with no touch the UI returns to DRIVE on its own, so a glance down always
// lands somewhere predictable.
//
// This module owns the LVGL port. Do not also call uiLogInit() -- both
// initialise esp_lvgl_port and the second one will fail.
#include "esp_err.h"

class VehicleState;
struct BroadcastState;

// Peer screens, in swipe order. The order is the product decision, not an
// implementation detail: DRIVE is the resting screen and must be index 0.
enum UiScreenId {
  UI_DRIVE = 0,
  UI_TYRES,
  UI_HEALTH,
  UI_TRIP,
  UI_TILT,
  UI_SCREEN_COUNT,
};

// Point the UI at the passively-monitored values. Optional, and safe to call
// before uiInit(). Anything available here is preferred over the polled copy:
// it arrives far more often and costs no bus traffic.
void uiSetBroadcast(const BroadcastState* bcast);

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
