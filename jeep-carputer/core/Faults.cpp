#include "Faults.h"

#include <string.h>

void faultsInit(FaultSet& s) {
  memset(&s, 0, sizeof(s));
  s.cursor = 0xFFu;   // not scanning
}

void faultsBeginScan(FaultSet& s, uint8_t ecuCount) {
  const uint32_t doneMs = s.doneMs;
  // The mute table is the driver's decision, not a finding. It survives every
  // scan -- a mute you had to re-apply after each read would be worse than no
  // mute at all, because you would only notice it had gone when the count lied.
  FaultMute keep[FAULT_MUTE_MAX];
  memcpy(keep, s.mute, sizeof(keep));
  const uint8_t keepCount = s.muteCount;

  memset(&s, 0, sizeof(s));
  memcpy(s.mute, keep, sizeof(keep));
  s.muteCount = keepCount;
  // The previous scan's stamp survives until this one finishes, so the display
  // keeps saying when the numbers it is showing came from rather than blanking
  // for the twelve seconds a walk of the modules takes.
  s.doneMs = doneMs;
  s.ecuCount = (ecuCount > 16u) ? 16u : ecuCount;   // one bit per module
  s.cursor = 0u;
}

bool faultsScanning(const FaultSet& s) {
  return s.ecuCount != 0u && s.cursor < s.ecuCount;
}

uint8_t faultsNextEcu(const FaultSet& s) {
  return faultsScanning(s) ? s.cursor : 0xFFu;
}

void faultsRecord(FaultSet& s, uint8_t ecu, const DtcRecord* recs, uint8_t n,
                  bool truncated) {
  if (ecu < 16u) {
    s.answered = (uint16_t)(s.answered | (uint16_t)(1u << ecu));
  }
  if (truncated) {
    s.full = true;
  }
  for (uint8_t i = 0u; i < n && recs != 0; ++i) {
    if (s.count >= FAULT_MAX) {
      s.full = true;
      break;
    }
    // Skip a code this module has already reported. A scan re-run after a
    // module was power-cycled mid-walk can otherwise list the same fault twice,
    // and two lines for one fault reads as two faults.
    bool seen = false;
    for (uint8_t j = 0u; j < s.count; ++j) {
      if (s.f[j].ecu == ecu && s.f[j].code == recs[i].code) {
        seen = true;
        break;
      }
    }
    if (seen) {
      continue;
    }
    s.f[s.count].code = recs[i].code;
    s.f[s.count].status = recs[i].status;
    s.f[s.count].ecu = ecu;
    s.f[s.count].muted = faultsIsMuted(s, ecu, recs[i].code);
    ++s.count;
  }
  if (s.cursor < s.ecuCount) {
    ++s.cursor;
  }
}

void faultsEcuFailed(FaultSet& s, uint8_t ecu) {
  if (ecu < 16u) {
    s.unreachable = (uint16_t)(s.unreachable | (uint16_t)(1u << ecu));
  }
  if (s.cursor < s.ecuCount) {
    ++s.cursor;
  }
}

// The five characters the driver actually reads, as a sort key.
static void faultsSaeKey(uint16_t code, char* out) {
  if (!kwpDtcToSae(code, out, 6u)) {
    out[0] = 0;   // unreachable at cap 6; an empty key just sorts first
  }
}

// Order the finished list by CODE rather than by the order the modules
// happened to answer in.
//
// By the RENDERED code, not the raw one. kwpDtcToSae indexes its letter out of
// "PCBU" -- bits 15..14 -- so raw ascending puts every P before every C before
// every B, which on screen reads as no order at all. Sorting the five
// characters gives B, C, P, U, which is the order a list of text is expected
// to be in.
//
// Insertion sort, at most sixteen entries, once per completed scan. Stable, so
// two modules reporting the same code keep the scan order between them -- the
// only thing left that distinguishes those two rows.
//
// Sorting HERE and nowhere else is deliberate: the list must not reorder under
// a finger. Doing it per record would shuffle rows while the walk is still
// running, and the drill-down's open dialog tracks its fault by index as well
// as by (ecu, code).
static void faultsSort(FaultSet& s) {
  for (uint8_t i = 1u; i < s.count; ++i) {
    const Fault key = s.f[i];
    char keyStr[6];
    faultsSaeKey(key.code, keyStr);
    uint8_t j = i;
    while (j > 0u) {
      char prev[6];
      faultsSaeKey(s.f[j - 1u].code, prev);
      if (strcmp(prev, keyStr) <= 0) {
        break;
      }
      s.f[j] = s.f[j - 1u];
      --j;
    }
    s.f[j] = key;
  }
}

void faultsEndScan(FaultSet& s, uint32_t nowMs) {
  faultsSort(s);
  s.cursor = 0xFFu;
  s.doneMs = (nowMs == 0u) ? 1u : nowMs;   // 0 is the "never scanned" sentinel
}

static uint8_t popcount16(uint16_t v) {
  uint8_t n = 0u;
  while (v) {
    v = (uint16_t)(v & (uint16_t)(v - 1u));
    ++n;
  }
  return n;
}

bool faultsIsMuted(const FaultSet& s, uint8_t ecu, uint16_t code) {
  for (uint8_t i = 0u; i < s.muteCount; ++i) {
    if (s.mute[i].ecu == ecu && s.mute[i].code == code) {
      return true;
    }
  }
  return false;
}

bool faultsMuteFull(const FaultSet& s) { return s.muteCount >= FAULT_MUTE_MAX; }

uint8_t faultsActiveCount(const FaultSet& s) {
  uint8_t n = 0u;
  for (uint8_t i = 0u; i < s.count; ++i) {
    if (!s.f[i].muted) {
      ++n;
    }
  }
  return n;
}

bool faultsToggleMute(FaultSet& s, uint8_t index) {
  if (index >= s.count) {
    return false;
  }
  Fault& f = s.f[index];
  if (f.muted) {
    // Un-mute: drop it from the table, keeping the rest packed.
    for (uint8_t i = 0u; i < s.muteCount; ++i) {
      if (s.mute[i].ecu == f.ecu && s.mute[i].code == f.code) {
        for (uint8_t j = (uint8_t)(i + 1u); j < s.muteCount; ++j) {
          s.mute[j - 1u] = s.mute[j];
        }
        --s.muteCount;
        break;
      }
    }
    f.muted = false;
    return false;
  }
  if (s.muteCount >= FAULT_MUTE_MAX) {
    return false;   // table full; the caller can say so
  }
  s.mute[s.muteCount].ecu = f.ecu;
  s.mute[s.muteCount].code = f.code;
  ++s.muteCount;
  f.muted = true;
  return true;
}

uint8_t faultsAnsweredCount(const FaultSet& s) { return popcount16(s.answered); }
uint8_t faultsUnreachableCount(const FaultSet& s) { return popcount16(s.unreachable); }

void faultsRemove(FaultSet& s, uint8_t index) {
  if (index >= s.count) {
    return;   // nothing there; the caller's index went stale, not wrong
  }
  // Shift the tail down rather than swapping the last row into the hole: the
  // list is sorted by the RENDERED code and the drill-down reads it straight
  // off the touch y, so a row that jumped position under the finger would put
  // a different fault where the driver was looking.
  for (uint8_t i = (uint8_t)(index + 1u); i < s.count; ++i) {
    s.f[i - 1u] = s.f[i];
  }
  --s.count;
  s.f[s.count] = Fault();   // leave no copy of the removed row past the end
  // Everything else is left exactly as it was, deliberately: the mute table
  // (the driver's decision, which must outlive the code it was made about),
  // `full` (a fact about the scan, still true), and answered/unreachable/
  // ecuCount/doneMs (facts about the walk). See the header.
}
