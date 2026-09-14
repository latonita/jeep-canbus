#include "Settings.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

static const char* TAG = "settings";

// Same namespace as the trip store, a different key. They share a partition
// anyway, and two namespaces would only make the layout harder to reason about.
static const char* kNamespace = "carputer";
static const char* kKey = "settings";

static Settings s_live;

const Settings& settingsGet(void) { return s_live; }

// A version byte in front of the blob. Adding a field later must not make an
// old record decode as garbage -- a stale byte here is an instrument that comes
// up in the wrong palette for no reason the driver can explain.
struct StoredSettings {
  uint8_t version;
  uint8_t theme;
};

#define SETTINGS_VERSION 2u    // 1 was a bool invertColours; see below

esp_err_t settingsLoad(void) {
  nvs_handle_t h = 0;
  // NVS_READONLY: loading must not create the namespace. nvs_flash_init() is
  // the trip store's job and may not have run or may have failed, in which case
  // this returns an error and the defaults stand -- which is the right outcome.
  esp_err_t err = nvs_open(kNamespace, NVS_READONLY, &h);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "no stored settings (%s), using defaults", esp_err_to_name(err));
    return err;
  }

  StoredSettings st;
  size_t len = sizeof(st);
  err = nvs_get_blob(h, kKey, &st, &len);
  nvs_close(h);

  if (err != ESP_OK || len != sizeof(st)) {
    ESP_LOGI(TAG, "settings not stored yet, using defaults");
    return ESP_OK;             // not an error: a first boot looks exactly like this
  }
  if (st.version != SETTINGS_VERSION) {
    // Version 1 stored a bool "invert colours" in this byte, from when the
    // daylight idea was a panel command rather than a palette. Its 0/1 happens
    // to mean NIGHT / DAY TEAL, but relying on that would be reading a
    // different field by luck. Fall back to the default and let the driver
    // choose again.
    ESP_LOGW(TAG, "settings version %u, expected %u -- using defaults",
             (unsigned)st.version, (unsigned)SETTINGS_VERSION);
    return ESP_OK;
  }

  s_live.theme = st.theme;
  ESP_LOGI(TAG, "loaded: theme=%u", (unsigned)s_live.theme);
  return ESP_OK;
}

static esp_err_t settingsSave(void) {
  nvs_handle_t h = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    return err;
  }

  StoredSettings st;
  st.version = SETTINGS_VERSION;
  st.theme = s_live.theme;

  err = nvs_set_blob(h, kKey, &st, sizeof(st));
  if (err == ESP_OK) {
    err = nvs_commit(h);
  }
  nvs_close(h);
  return err;
}

esp_err_t settingsSetTheme(uint8_t theme) {
  s_live.theme = theme;
  const esp_err_t err = settingsSave();
  if (err != ESP_OK) {
    // Not fatal and not worth refusing the change over: a driver squinting at
    // the screen in the sun cares about this afternoon, not about whether it
    // survives the next key cycle.
    ESP_LOGW(TAG, "theme %u set but not saved: %s", (unsigned)theme,
             esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "theme=%u saved", (unsigned)theme);
  }
  return err;
}
