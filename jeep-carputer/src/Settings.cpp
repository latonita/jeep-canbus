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
// Deliberately all uint8_t, including the price, which is a uint16_t
// everywhere else. A uint16_t member here would sit behind a padding byte on
// this compiler, and the padding would be written to flash and read back as
// part of a fixed-size record -- so the layout would depend on the alignment
// rules rather than on this declaration. Splitting it into two bytes makes the
// record exactly what it looks like: five bytes, no holes.
struct StoredSettings {
  uint8_t version;
  uint8_t theme;
  uint8_t orientation;
  uint8_t fuelPriceLo;
  uint8_t fuelPriceHi;
};

#define SETTINGS_VERSION 4u    // 1: bool invertColours. 2: no orientation. 3: no price

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

  // Zeroed, and the length is an IN/OUT parameter: nvs_get_blob only refuses
  // when the buffer is too SMALL, so a shorter record from an older firmware
  // reads back happily with len set to what was actually stored. That is what
  // makes the version-2 path below possible.
  StoredSettings st = {};
  size_t len = sizeof(st);
  err = nvs_get_blob(h, kKey, &st, &len);
  nvs_close(h);

  if (err != ESP_OK || len < 2u) {
    ESP_LOGI(TAG, "settings not stored yet, using defaults");
    return ESP_OK;             // not an error: a first boot looks exactly like this
  }

  if (st.version == 2u && len == 2u) {
    // Version 2 is this record without the orientation byte. Its theme means
    // exactly what it means now, so carry it forward rather than making every
    // driver who already chose a palette choose it again for a setting they
    // have not been offered yet. The orientation keeps its default, which is
    // the orientation a version-2 firmware could only have been drawing in.
    s_live.theme = st.theme;
    ESP_LOGI(TAG, "loaded v2: theme=%u, orientation defaulted",
             (unsigned)s_live.theme);
    return ESP_OK;
  }

  if (st.version == 3u && len == 3u) {
    // Version 3 is this record without the price. Same reasoning as the v2 path
    // above: carry forward what was genuinely chosen, default what did not
    // exist to be chosen.
    s_live.theme = st.theme;
    s_live.orientation = st.orientation;
    ESP_LOGI(TAG, "loaded v3: theme=%u orientation=%u, price defaulted",
             (unsigned)s_live.theme, (unsigned)s_live.orientation);
    return ESP_OK;
  }

  if (st.version != SETTINGS_VERSION || len != sizeof(st)) {
    // Version 1 stored a bool "invert colours" in this byte, from when the
    // daylight idea was a panel command rather than a palette. Its 0/1 happens
    // to mean NIGHT / DAY TEAL, but relying on that would be reading a
    // different field by luck. Fall back to the default and let the driver
    // choose again.
    ESP_LOGW(TAG, "settings version %u len %u, expected %u/%u -- using defaults",
             (unsigned)st.version, (unsigned)len, (unsigned)SETTINGS_VERSION,
             (unsigned)sizeof(st));
    return ESP_OK;
  }

  s_live.theme = st.theme;
  s_live.orientation = st.orientation;
  const uint16_t cents =
      (uint16_t)(st.fuelPriceLo | ((uint16_t)st.fuelPriceHi << 8));
  // A stored zero would make every cost readout dash forever, and there is no
  // way to reach this row and see why. Treat it as never-set.
  if (cents != 0u) {
    s_live.fuelPriceCents = cents;
  }
  ESP_LOGI(TAG, "loaded: theme=%u orientation=%u price=%u", (unsigned)s_live.theme,
           (unsigned)s_live.orientation, (unsigned)s_live.fuelPriceCents);
  return ESP_OK;
}

static esp_err_t settingsSave(void) {
  nvs_handle_t h = 0;
  esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &h);
  if (err != ESP_OK) {
    return err;
  }

  StoredSettings st = {};
  st.version = SETTINGS_VERSION;
  st.theme = s_live.theme;
  st.orientation = s_live.orientation;
  st.fuelPriceLo = (uint8_t)(s_live.fuelPriceCents & 0xFFu);
  st.fuelPriceHi = (uint8_t)(s_live.fuelPriceCents >> 8);

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

esp_err_t settingsSetFuelPrice(uint16_t cents) {
  s_live.fuelPriceCents = cents;
  const esp_err_t err = settingsSave();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "price %u set but not saved: %s", (unsigned)cents,
             esp_err_to_name(err));
  }
  return err;
}

esp_err_t settingsSetOrientation(uint8_t orientation) {
  s_live.orientation = orientation;
  const esp_err_t err = settingsSave();
  if (err != ESP_OK) {
    // Same judgement as the theme: the flip has already happened on the glass,
    // and refusing it because the record would not commit would leave the
    // driver looking at a screen that ignored them. Losing it across a key
    // cycle is the lesser failure -- and this is a setting chosen once, at the
    // bracket, so it will simply be chosen again.
    ESP_LOGW(TAG, "orientation %u set but not saved: %s", (unsigned)orientation,
             esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "orientation=%u saved", (unsigned)orientation);
  }
  return err;
}
