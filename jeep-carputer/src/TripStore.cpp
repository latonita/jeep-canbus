#include "TripStore.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "Trip.h"

static const char* TAG = "tripstore";
static const char* kNamespace = "carputer";

// One key per meter. "tripA" is the key the single-meter version used, so an
// existing TRIP A survives the upgrade to two untouched; TRIP B simply has
// nothing stored on the first boot after it, which reads as a fresh meter.
static const char* const kKeys[TRIP_STORE_COUNT] = {"tripA", "tripB"};

static nvs_handle_t s_nvs = 0;
static bool s_open = false;
static Trip s_lastWritten[TRIP_STORE_COUNT];
static unsigned s_lastWriteMs[TRIP_STORE_COUNT];

esp_err_t tripStoreInit(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // A partition from an older layout, or one that filled up. Losing the trip
    // meter is a better outcome than refusing to boot.
    ESP_LOGW(TAG, "nvs needs erase (%s)", esp_err_to_name(err));
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
    return err;
  }

  err = nvs_open(kNamespace, NVS_READWRITE, &s_nvs);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs open failed: %s", esp_err_to_name(err));
    return err;
  }
  s_open = true;
  memset(s_lastWritten, 0, sizeof(s_lastWritten));
  memset(s_lastWriteMs, 0, sizeof(s_lastWriteMs));
  return ESP_OK;
}

bool tripStoreLoad(Trip& t, uint8_t which) {
  if (!s_open || which >= TRIP_STORE_COUNT) {
    return false;
  }
  size_t len = sizeof(Trip);
  const esp_err_t err = nvs_get_blob(s_nvs, kKeys[which], &t, &len);
  if (err != ESP_OK || len != sizeof(Trip)) {
    // A size mismatch means the struct changed shape between firmware
    // versions. Discard rather than reinterpret: a misread blob would show as
    // a confident but wrong trip.
    if (err != ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGW(TAG, "stored trip %c unusable (%s, %u bytes)",
               (char)('A' + which), esp_err_to_name(err), (unsigned)len);
    }
    return false;
  }
  s_lastWritten[which] = t;
  ESP_LOGI(TAG, "restored trip %c: odo %u..%u dm, %u ms elapsed",
           (char)('A' + which), (unsigned)t.odoStartDm, (unsigned)t.odoNowDm,
           (unsigned)t.elapsedMs);
  return true;
}

static bool worthWriting(const Trip& t, uint8_t which) {
  return memcmp(&t, &s_lastWritten[which], sizeof(Trip)) != 0;
}

static void writeNow(const Trip& t, uint8_t which) {
  if (nvs_set_blob(s_nvs, kKeys[which], &t, sizeof(Trip)) == ESP_OK &&
      nvs_commit(s_nvs) == ESP_OK) {
    s_lastWritten[which] = t;
  } else {
    ESP_LOGW(TAG, "trip %c save failed", (char)('A' + which));
  }
}

bool tripStoreSave(const Trip& t, uint8_t which, unsigned nowMs,
                   unsigned minIntervalMs) {
  if (!s_open || which >= TRIP_STORE_COUNT) {
    return false;
  }
  if (nowMs - s_lastWriteMs[which] < minIntervalMs) {
    return false;
  }
  if (!worthWriting(t, which)) {
    // Nothing moved -- a parked car. Resetting the clock here as well would be
    // wrong: it would push the next real write a full interval past the first
    // moment something changed.
    return false;
  }
  s_lastWriteMs[which] = nowMs;
  writeNow(t, which);
  return true;
}

void tripStoreSaveNow(const Trip& t, uint8_t which) {
  if (!s_open || which >= TRIP_STORE_COUNT) {
    return;
  }
  writeNow(t, which);
}
