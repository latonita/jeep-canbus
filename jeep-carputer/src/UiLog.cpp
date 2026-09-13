#include "UiLog.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "Board.h"
#include "Display.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"

static const char* TAG = "uilog";

// 240 px at ~8 px per glyph in montserrat_14 is about 28 columns; 320 px at a
// 16 px line height is about 20 rows.
#define UILOG_ROWS 18u
#define UILOG_COLS 40u

// Rows LVGL renders per flush: 40 x 240 x 16 bpp is ~19 KB of internal
// DMA-capable RAM. PSRAM is still off and a text log needs no framebuffer.
#define UILOG_BUF_ROWS 40

// ---------------------------------------------------------------------------
// This does NOT hook esp_log_set_vprintf, and that is deliberate.
//
// An earlier version did, using a portMUX spinlock to guard the ring buffer.
// FreeRTOS spinlocks are not recursive and disable interrupts, so a log call
// arriving while the refresh task held the same lock deadlocked the core with
// interrupts off -- which killed USB-Serial-JTAG (no console, no esptool) while
// the ST7789 went on displaying its last frame out of its own GRAM. The screen
// looked alive; the chip was not.
//
// So: no interception. Our code calls uiLogLine()/uiLogPrintf() explicitly, a
// blocking mutex guards the buffer, and nothing in the logging path can re-enter
// this module. Losing automatic capture of other components' output is a cheap
// price for not being able to hang the CPU from a log statement.
// ---------------------------------------------------------------------------

static char s_rows[UILOG_ROWS][UILOG_COLS];
static uint8_t s_head;                 // next slot to write
static uint8_t s_count;                // rows used, saturates at UILOG_ROWS
static bool s_dirty;
static SemaphoreHandle_t s_mutex = nullptr;
static lv_obj_t* s_label = nullptr;

void uiLogLine(const char* text) {
  if (text == nullptr || s_mutex == nullptr) {
    return;
  }
  if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    return;                            // never block a caller for the screen
  }

  // Circular, so adding a line is O(1) -- no memmove of the whole buffer.
  char* dst = s_rows[s_head];
  size_t n = 0u;
  for (; text[n] != '\0' && n < UILOG_COLS - 1u; ++n) {
    const char c = text[n];
    dst[n] = (c == '\n' || c == '\r') ? ' ' : c;
  }
  dst[n] = '\0';

  s_head = static_cast<uint8_t>((s_head + 1u) % UILOG_ROWS);
  if (s_count < UILOG_ROWS) {
    ++s_count;
  }
  s_dirty = true;
  xSemaphoreGive(s_mutex);
}

void uiLogPrintf(const char* fmt, ...) {
  char buf[UILOG_COLS + 32u];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  uiLogLine(buf);
}

// Copies the ring out under the mutex, then touches LVGL with the mutex
// released -- holding two locks at once is what caused the last problem.
static void uiTask(void*) {
  static char joined[UILOG_ROWS * UILOG_COLS + UILOG_ROWS + 1u];

  for (;;) {
    bool dirty = false;
    size_t at = 0u;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      dirty = s_dirty;
      if (dirty) {
        s_dirty = false;
        const uint8_t rows = s_count;
        const uint8_t first =
            static_cast<uint8_t>((s_head + UILOG_ROWS - rows) % UILOG_ROWS);
        for (uint8_t r = 0u; r < rows; ++r) {
          const char* src = s_rows[(first + r) % UILOG_ROWS];
          for (size_t i = 0u; src[i] != '\0' && at + 2u < sizeof(joined); ++i) {
            joined[at++] = src[i];
          }
          if (r + 1u < rows && at + 1u < sizeof(joined)) {
            joined[at++] = '\n';
          }
        }
        joined[at] = '\0';
      }
      xSemaphoreGive(s_mutex);
    }

    if (dirty && s_label != nullptr && lvgl_port_lock(100)) {
      lv_label_set_text(s_label, joined);
      lvgl_port_unlock();
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

esp_err_t uiLogInit(void) {
  s_mutex = xSemaphoreCreateMutex();
  if (s_mutex == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
  esp_err_t err = lvgl_port_init(&port_cfg);
  if (err != ESP_OK) {
    return err;
  }

  lvgl_port_display_cfg_t disp_cfg = {};
  disp_cfg.io_handle = displayPanelIo();
  disp_cfg.panel_handle = displayPanel();
  disp_cfg.buffer_size = static_cast<uint32_t>(LCD_H_RES) * UILOG_BUF_ROWS;
  disp_cfg.double_buffer = false;
  disp_cfg.hres = LCD_H_RES;
  disp_cfg.vres = LCD_V_RES;
  disp_cfg.monochrome = false;
  // Display.cpp already mirrored the panel; doing it again here would cancel
  // it out.
  disp_cfg.rotation.swap_xy = false;
  disp_cfg.rotation.mirror_x = false;
  disp_cfg.rotation.mirror_y = false;
  disp_cfg.flags.buff_dma = true;
  disp_cfg.flags.swap_bytes = true;    // ST7789 over SPI wants big-endian

  lv_display_t* disp = lvgl_port_add_disp(&disp_cfg);
  if (disp == nullptr) {
    return ESP_FAIL;
  }

  if (!lvgl_port_lock(0)) {
    return ESP_FAIL;
  }
  lv_obj_t* scr = lv_display_get_screen_active(disp);
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);

  s_label = lv_label_create(scr);
  lv_obj_set_width(s_label, LCD_H_RES - 4);
  lv_obj_align(s_label, LV_ALIGN_TOP_LEFT, 2, 2);
  lv_label_set_long_mode(s_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(s_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_text_font(s_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_label_set_text(s_label, "jeep-carputer");
  lvgl_port_unlock();

  if (xTaskCreate(uiTask, "uilog", 4096, nullptr, 3, nullptr) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }

  ESP_LOGI(TAG, "on-screen log up: %u cols x %u rows",
           static_cast<unsigned>(UILOG_COLS - 1u),
           static_cast<unsigned>(UILOG_ROWS));
  uiLogLine("carputer: log ready");
  return ESP_OK;
}
