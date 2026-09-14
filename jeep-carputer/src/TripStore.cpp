#include "TripStore.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "Trip.h"

static const char* TAG = "tripstore";
static const char* kNamespace = "carputer";
static const char* kKey = "tripA";

static nvs_handle_t s_nvs = 0;
static bool s_open = false;
static Trip s_lastWritten;
static unsigned s_lastWriteMs = 0u;

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
  memset(&s_lastWritten, 0, sizeof(s_lastWritten));
  return ESP_OK;
}

bool tripStoreLoad(Trip& a) {
  if (!s_open) {
    return false;
  }
  size_t len = sizeof(Trip);
  const esp_err_t err = nvs_get_blob(s_nvs, kKey, &a, &len);
  if (err != ESP_OK || len != sizeof(Trip)) {
    // A size mismatch means the struct changed shape between firmware
    // versions. Discard rather than reinterpret: a misread blob would show as
    // a confident but wrong trip.
    if (err != ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGW(TAG, "stored trip unusable (%s, %u bytes)", esp_err_to_name(err),
               (unsigned)len);
    }
    return false;
  }
  s_lastWritten = a;
  ESP_LOGI(TAG, "restored trip A: odo %u..%u dm, %u ms elapsed",
           (unsigned)a.odoStartDm, (unsigned)a.odoNowDm, (unsigned)a.elapsedMs);
  return true;
}

static bool worthWriting(const Trip& a) {
  return memcmp(&a, &s_lastWritten, sizeof(Trip)) != 0;
}

static void writeNow(const Trip& a) {
  if (nvs_set_blob(s_nvs, kKey, &a, sizeof(Trip)) == ESP_OK &&
      nvs_commit(s_nvs) == ESP_OK) {
    s_lastWritten = a;
  } else {
    ESP_LOGW(TAG, "trip save failed");
  }
}

bool tripStoreSave(const Trip& a, unsigned nowMs, unsigned minIntervalMs) {
  if (!s_open) {
    return false;
  }
  if (nowMs - s_lastWriteMs < minIntervalMs) {
    return false;
  }
  if (!worthWriting(a)) {
    // Nothing moved -- a parked car. Resetting the clock here as well would be
    // wrong: it would push the next real write a full interval past the first
    // moment something changed.
    return false;
  }
  s_lastWriteMs = nowMs;
  writeNow(a);
  return true;
}

void tripStoreSaveNow(const Trip& a) {
  if (!s_open) {
    return;
  }
  writeNow(a);
}
