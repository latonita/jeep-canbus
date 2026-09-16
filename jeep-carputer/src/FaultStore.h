#pragma once
// Persistence for the DTC MUTE list, and only for it.
//
// The faults themselves are not stored. They live in the modules, a re-read
// takes twelve seconds, and a stale copy on flash would be a list of problems
// the car may no longer have -- which is worse than no list.
//
// What must survive is the DECISION: which codes the driver has already looked
// at and set aside. That is not a finding, it is a judgement, and re-making it
// after every key cycle would make muting worthless. Sixteen entries of three
// bytes is nothing to store.
#include "esp_err.h"

struct FaultSet;

// Fill the mute table from flash. Leaves it empty when nothing is stored yet.
// Safe to call before any scan; the codes are matched by (ecu, code), so they
// apply to whatever the next read turns up.
bool faultStoreLoadMutes(FaultSet& s);

// Write the mute table back. Called when the driver changes it, which is rare
// enough that there is no rate limit to justify.
void faultStoreSaveMutes(const FaultSet& s);
