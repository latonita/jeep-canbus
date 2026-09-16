#include "FaultStore.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

#include "Faults.h"

static const char* TAG = "faultstore";

// Same namespace as the trips and the settings. They share a partition anyway,
// and a third namespace would make the layout harder to reason about for no
// gain.
static const char* kNamespace = "carputer";
static const char* kKey = "dtcmute";

// Written as a version byte, a count, then the entries as three loose bytes
// each. Loose rather than the struct, because FaultMute is a uint16 behind a
// uint8 and the compiler is free to pad it -- and padding written to flash is
// a layout that depends on the compiler rather than on this file.
#define MUTE_VERSION 1u

bool faultStoreLoadMutes(FaultSet& s) {
  nvs_handle_t h = 0;
  if (nvs_open(kNamespace, NVS_READONLY, &h) != ESP_OK) {
    return false;
  }
  uint8_t buf[2u + FAULT_MUTE_MAX * 3u];
  size_t len = sizeof(buf);
  const esp_err_t err = nvs_get_blob(h, kKey, buf, &len);
  nvs_close(h);
  if (err != ESP_OK || len < 2u || buf[0] != MUTE_VERSION) {
    return false;
  }
  uint8_t n = buf[1];
  if (n > FAULT_MUTE_MAX) {
    n = FAULT_MUTE_MAX;
  }
  if (len < (size_t)(2u + n * 3u)) {
    return false;   // truncated record; take none of it rather than some
  }
  s.muteCount = n;
  for (uint8_t i = 0u; i < n; ++i) {
    const uint8_t* p = &buf[2u + i * 3u];
    s.mute[i].ecu = p[0];
    s.mute[i].code = (uint16_t)(p[1] | ((uint16_t)p[2] << 8));
  }
  ESP_LOGI(TAG, "%u muted code(s) restored", (unsigned)n);
  return true;
}

void faultStoreSaveMutes(const FaultSet& s) {
  nvs_handle_t h = 0;
  if (nvs_open(kNamespace, NVS_READWRITE, &h) != ESP_OK) {
    ESP_LOGW(TAG, "cannot open nvs to save mutes");
    return;
  }
  uint8_t buf[2u + FAULT_MUTE_MAX * 3u];
  const uint8_t n = (s.muteCount > FAULT_MUTE_MAX) ? FAULT_MUTE_MAX : s.muteCount;
  buf[0] = MUTE_VERSION;
  buf[1] = n;
  for (uint8_t i = 0u; i < n; ++i) {
    uint8_t* p = &buf[2u + i * 3u];
    p[0] = s.mute[i].ecu;
    p[1] = (uint8_t)(s.mute[i].code & 0xFFu);
    p[2] = (uint8_t)(s.mute[i].code >> 8);
  }
  const size_t len = (size_t)(2u + n * 3u);
  if (nvs_set_blob(h, kKey, buf, len) != ESP_OK || nvs_commit(h) != ESP_OK) {
    // Not fatal and not worth refusing the mute over: the driver has already
    // seen it take effect on screen, and losing it across a key cycle is a
    // smaller surprise than the tap appearing to do nothing.
    ESP_LOGW(TAG, "mute list not saved");
  }
  nvs_close(h);
}
