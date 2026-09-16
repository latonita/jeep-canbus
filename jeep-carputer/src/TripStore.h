#pragma once
// Persistence for the resettable trip meters, TRIP A and TRIP B.
//
// They only mean anything if they outlive a key cycle -- otherwise they are the
// same meter as FROM START and there is no reason to have three. This box loses
// power with the ignition, so the totals go to NVS.
//
// FROM START is deliberately NOT persisted: being cleared by the power cycle is
// the whole of what makes it a different meter.
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

// How many meters are persisted. TRIP_A and TRIP_B, which are TripId 0 and 1 --
// the enum puts the unpersisted FROM START last precisely so this is a prefix
// of it rather than a scatter.
#define TRIP_STORE_COUNT 2u

// Fill `t` from flash. Leaves it untouched and returns false when there is
// nothing stored yet (the first boot, or after an erase), or when `which` is
// not a persisted meter.
bool tripStoreLoad(Trip& t, uint8_t which);

// Write `t` back, but only if it differs from what was last written and at most
// once per `minIntervalMs`. Returns true when a write actually happened.
//
// The rate limit is PER METER. Sharing one clock between the two would let a
// busy TRIP A hold TRIP B's first write off indefinitely.
bool tripStoreSave(const Trip& t, uint8_t which, unsigned nowMs,
                   unsigned minIntervalMs);

// Write `t` back right now, ignoring the rate limit. For a reset, which the
// driver must see survive a power cycle.
void tripStoreSaveNow(const Trip& t, uint8_t which);
