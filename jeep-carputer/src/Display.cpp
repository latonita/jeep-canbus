#include "Display.h"

#include <string.h>

#include "Board.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"

static const char* TAG = "display";

// Waveshare's own backlight settings: low-speed mode, timer 0, channel 0,
// 13-bit at 5 kHz.
#define BL_TIMER      LEDC_TIMER_0
#define BL_MODE       LEDC_LOW_SPEED_MODE
#define BL_CHANNEL    LEDC_CHANNEL_0
#define BL_RES        LEDC_TIMER_13_BIT
#define BL_FREQ_HZ    5000u
#define BL_DUTY_MAX   ((1u << 13) - 1u)

// Rows per DMA transfer. 40 rows of 240 px at 16 bpp is 19 KB, comfortably
// inside internal RAM and few enough transfers to be quick.
#define FILL_ROWS     40

static esp_lcd_panel_io_handle_t s_io = nullptr;
static esp_lcd_panel_handle_t s_panel = nullptr;

esp_err_t displayBacklightPermille(uint16_t permille) {
  if (permille > 1000u) {
    permille = 1000u;
  }
  const uint32_t duty = (BL_DUTY_MAX * permille) / 1000u;
  esp_err_t err = ledc_set_duty(BL_MODE, BL_CHANNEL, duty);
  if (err != ESP_OK) {
    return err;
  }
  return ledc_update_duty(BL_MODE, BL_CHANNEL);
}

esp_err_t displayBacklight(uint8_t percent) {
  if (percent > 100u) {
    percent = 100u;
  }
  const uint32_t duty = (BL_DUTY_MAX * percent) / 100u;
  esp_err_t err = ledc_set_duty(BL_MODE, BL_CHANNEL, duty);
  if (err != ESP_OK) {
    return err;
  }
  return ledc_update_duty(BL_MODE, BL_CHANNEL);
}

static esp_err_t backlightInit(void) {
  ledc_timer_config_t timer = {};
  timer.speed_mode = BL_MODE;
  timer.duty_resolution = BL_RES;
  timer.timer_num = BL_TIMER;
  timer.freq_hz = BL_FREQ_HZ;
  timer.clk_cfg = LEDC_AUTO_CLK;
  esp_err_t err = ledc_timer_config(&timer);
  if (err != ESP_OK) {
    return err;
  }

  ledc_channel_config_t ch = {};
  ch.gpio_num = LCD_PIN_BL;
  ch.speed_mode = BL_MODE;
  ch.channel = BL_CHANNEL;
  ch.timer_sel = BL_TIMER;
  ch.duty = 0;
  ch.hpoint = 0;
  err = ledc_channel_config(&ch);
  if (err != ESP_OK) {
    return err;
  }
  return displayBacklight(0u);   // stay dark until the panel is initialised
}

esp_err_t displayInit(void) {
  esp_err_t err = backlightInit();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "backlight init failed: %s", esp_err_to_name(err));
    return err;
  }

  spi_bus_config_t bus = {};
  bus.mosi_io_num = LCD_PIN_MOSI;
  bus.miso_io_num = LCD_PIN_MISO;
  bus.sclk_io_num = LCD_PIN_SCK;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  // Sized for the LVGL flush chunk, which is far larger than this file's own
  // FILL_ROWS smoke-test buffer -- see LCD_MAX_TRANSFER_SZ in Board.h.
  bus.max_transfer_sz = LCD_MAX_TRANSFER_SZ;
  err = spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
    return err;
  }

  esp_lcd_panel_io_spi_config_t io = {};
  io.cs_gpio_num = LCD_PIN_CS;
  io.dc_gpio_num = LCD_PIN_DC;
  io.spi_mode = 0;
  io.pclk_hz = LCD_PCLK_HZ;
  io.trans_queue_depth = 10;
  io.lcd_cmd_bits = 8;
  io.lcd_param_bits = 8;
  // esp_lcd_spi_bus_handle_t is an int here, not a pointer, so this is a
  // value conversion from the host enum -- not a reinterpret.
  err = esp_lcd_new_panel_io_spi(
      static_cast<esp_lcd_spi_bus_handle_t>(LCD_HOST), &io, &s_io);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "panel io failed: %s", esp_err_to_name(err));
    return err;
  }

  esp_lcd_panel_dev_config_t dev = {};
  dev.reset_gpio_num = LCD_PIN_RST;
  // RGB, not BGR.
  //
  // This was BGR "per Waveshare's config", and with it every colour reached the
  // glass with red and blue transposed. It went unnoticed through bring-up
  // because the smoke test only ever filled black and white, which are
  // symmetric under that swap. It showed up the moment the UI had a palette:
  // teal #1AFFE0 painted as yellow-green #E0FF1A, and -- the reason this is not
  // merely cosmetic -- the critical alert red #FF4426 painted as blue #2644FF.
  //
  // If a future panel revision genuinely is BGR, flip this back and invert the
  // palette instead; do not "fix" it by pre-swapping colours in UiTheme.h,
  // which would leave every future colour wrong by default.
  dev.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
  dev.bits_per_pixel = 16;
  err = esp_lcd_new_panel_st7789(s_io, &dev, &s_panel);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "st7789 create failed: %s", esp_err_to_name(err));
    return err;
  }

  ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
  ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
  // Waveshare's vendored init sends raw 0x21 (INVON), so inversion is on for
  // this panel. Their own comment mislabels that command "Display On".
  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
  // Waveshare's own value, and DEAD as soon as the UI comes up: esp_lvgl_port
  // rewrites MADCTL from disp_cfg.rotation inside lvgl_port_add_disp(), which
  // is all-false, so the panel ends up mirror(false, false). Left in place
  // because it is still what is on the glass for the smoke test between here
  // and uiInit(), and because it is the vendor's documented init -- but do not
  // change it expecting the dashboard to follow. Orientation is Ui.cpp's.
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, true, false));
  ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, 0, 0));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

  ESP_LOGI(TAG, "ST7789 up: %dx%d on SPI%d, sck=%d mosi=%d dc=%d cs=%d rst=%d bl=%d",
           LCD_H_RES, LCD_V_RES, static_cast<int>(LCD_HOST), LCD_PIN_SCK,
           LCD_PIN_MOSI, LCD_PIN_DC, LCD_PIN_CS, LCD_PIN_RST, LCD_PIN_BL);
  return ESP_OK;
}

esp_lcd_panel_handle_t displayPanel(void) { return s_panel; }
esp_lcd_panel_io_handle_t displayPanelIo(void) { return s_io; }

esp_err_t displayFill(uint16_t rgb565) {
  if (s_panel == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  const size_t pixels = static_cast<size_t>(LCD_H_RES) * FILL_ROWS;
  uint16_t* buf = static_cast<uint16_t*>(
      heap_caps_malloc(pixels * sizeof(uint16_t), MALLOC_CAP_DMA));
  if (buf == nullptr) {
    ESP_LOGE(TAG, "no DMA memory for %u pixels", static_cast<unsigned>(pixels));
    return ESP_ERR_NO_MEM;
  }

  // draw_bitmap takes the panel's byte order, which for ST7789 over SPI is
  // big-endian, so swap here rather than trusting the default.
  const uint16_t swapped = static_cast<uint16_t>((rgb565 >> 8) | (rgb565 << 8));
  for (size_t i = 0u; i < pixels; ++i) {
    buf[i] = swapped;
  }

  esp_err_t err = ESP_OK;
  for (int y = 0; y < LCD_V_RES && err == ESP_OK; y += FILL_ROWS) {
    int rows = LCD_V_RES - y;
    if (rows > FILL_ROWS) {
      rows = FILL_ROWS;
    }
    err = esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_H_RES, y + rows, buf);
  }

  heap_caps_free(buf);
  return err;
}
