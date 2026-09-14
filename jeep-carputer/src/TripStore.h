#pragma once
// Persistence for TRIP A.
//
// TRIP A only means something if it outlives a key cycle -- otherwise it is the
// same meter as FROM START and there is no reason to have both. This box loses
// power with the ignition, so the totals go to NVS.
//
// Writes are RATE LIMITED, not immediate. NVS sits in flash with a finite erase
// budget, and a trip meter that saved on every odometer tick would write once
// every few seconds for the life of the car. Saving on a timer, and only when
// the numbers have actually moved, keeps a long drive to a handful of writes.
#include "esp_err.h"

struct Trip;

// Brings up NVS and opens the namespace. Safe to call once at boot; every other
// entry point here is a no-op if this failed, so a flash problem costs the trip
// meter and nothing else.
esp_err_t tripStoreInit(void);

// Fill `a` from flash. Leaves it untouched and returns false when there is
// nothing stored yet (the first boot, or after an erase).
bool tripStoreLoad(Trip& a);

// Write `a` back, but only if it differs from what was last written and at most
// once per `minIntervalMs`. Returns true when a write actually happened.
bool tripStoreSave(const Trip& a, unsigned nowMs, unsigned minIntervalMs);

// Write `a` back right now, ignoring the rate limit. For a reset, which the
// driver must see survive a power cycle.
void tripStoreSaveNow(const Trip& a);
