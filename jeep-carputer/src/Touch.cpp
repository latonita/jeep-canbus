#include "Touch.h"

#include "Board.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_cst328.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

static const char* TAG = "touch";

// ---------------------------------------------------------------------------
// I2C driver flavour
//
// IDF 5.3 ships both driver/i2c.h (legacy) and driver/i2c_master.h (new), and
// the two cannot share a port. The vendored CST328 component reaches the chip
// only through esp_lcd_panel_io_{tx,rx}_param, so the flavour is decided by
// whichever esp_lcd_new_panel_io_i2c overload we call, not by the driver. Its
// own example (README.md and test_apps/) builds the bus with
// i2c_new_master_bus() and passes an i2c_master_bus_handle_t -- the NEW driver
// -- so that is what is used here. In C++ esp_lcd_new_panel_io_i2c() is an
// overload set rather than the C _Generic macro, so handing it the typed bus
// handle picks the _v2 (new-driver) implementation with no cast.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Coordinate transform -- the part most likely to be wrong, so: the reasoning.
//
// Display.cpp calls esp_lcd_panel_mirror(true, false), and Ui.cpp then
// deliberately leaves LVGL's own rotation flags false so the two do not cancel
// out. So exactly one horizontal flip is applied, and it happens inside the
// ST7789 (MADCTL MX): LVGL's x = 0 lands on the physical edge opposite to the
// panel's unmirrored default.
//
// What matters is WHY that mirror is there. It is not a correction we invented;
// it comes from Waveshare's own panel init, and its purpose is to make the
// LCD's coordinate space agree with the physical module as it is held -- image
// top-left at glass top-left. In other words, after mirror(true, false) the
// LVGL frame IS the module's visible frame.
//
// The touch digitiser is a separate laminated layer with its own origin, fixed
// by its flex, and module makers define that against the finished module rather
// than against the LCD controller's register defaults. The CST328 here reports
// 0..239 in x and 0..319 in y -- already the visible frame. The two spaces
// therefore already agree, and mirroring the touch as well would UNDO the panel
// mirror and put every tap on the wrong side.
//
// This is also the configuration Waveshare ships for this exact board: both
// managed_components/waveshare__esp_lcd_touch_cst328/README.md and its
// test_apps/main/test_esp_lcd_touch_cst328.c use all-zero flags against a panel
// brought up with mirror(true, false).
//
// UNVERIFIED ON GLASS. This is a layout argument, not a measurement. If a tap
// lands mirrored left-to-right, set TOUCH_MIRROR_X true; if top-to-bottom,
// TOUCH_MIRROR_Y; if x and y are transposed, TOUCH_SWAP_XY. Those three lines
// are the whole fix -- nothing in Display.cpp or Ui.cpp should be changed to
// correct it, or the image and the touch will drift apart again.
#define TOUCH_SWAP_XY   false
#define TOUCH_MIRROR_X  false
#define TOUCH_MIRROR_Y  false
// ---------------------------------------------------------------------------

// Put the controller back into normal reporting mode. The vendor driver does
// this at the end of its init, but only ~10 ms after releasing reset, which is
// tight for the CST328's boot. Re-issuing it once the chip has certainly
// settled is idempotent, and cheap insurance against a panel that answers on
// I2C but never reports a point.
#define CST328_REG_NORMAL_MODE  0xD109

// Time to let the controller boot after its reset is released.
#define CST328_BOOT_MS          60

static i2c_master_bus_handle_t   s_bus = nullptr;
static esp_lcd_panel_io_handle_t s_io = nullptr;
static esp_lcd_touch_handle_t    s_touch = nullptr;
static lv_indev_t*               s_indev = nullptr;

bool touchReady(void) { return s_indev != nullptr; }

// The vendor driver drives RST but never configures it as an output, so out of
// the box its reset pulse silently does nothing -- the pad's output driver is
// still disabled and the chip is only ever reset by power-on. Configuring it
// here makes the pulse real.
//
// Note the polarity. touch_cst328_reset() in the vendored driver writes
// !levels.reset first and levels.reset second, so levels.reset is the level it
// LEAVES the pin at -- the inverse of what esp_lcd_touch.h documents and of
// what the other esp_lcd_touch drivers do. RST on the CST328 is active low, so
// levels.reset must be 1 to get a low pulse that ends with the chip released.
// Leaving it at the zero-initialised 0 would park RST low and hold the
// controller in reset for good.
static esp_err_t resetPinInit(void) {
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << static_cast<uint32_t>(TOUCH_PIN_RST);
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  esp_err_t err = gpio_config(&cfg);
  if (err != ESP_OK) {
    return err;
  }
  return gpio_set_level(TOUCH_PIN_RST, 1);   // released; the driver pulses it
}

esp_err_t touchInit(void) {
  if (s_indev != nullptr) {
    return ESP_OK;
  }

  esp_err_t err = resetPinInit();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "reset pin config failed: %s", esp_err_to_name(err));
    return err;
  }

  // A bus of our own on I2C_NUM_1. The board's other I2C devices (IMU, RTC and
  // the external header) sit on a different pin pair and get their own port;
  // the touch lines are not shared.
  i2c_master_bus_config_t bus = {};
  bus.i2c_port = TOUCH_I2C_PORT;
  bus.sda_io_num = TOUCH_PIN_SDA;
  bus.scl_io_num = TOUCH_PIN_SCL;
  bus.clk_source = I2C_CLK_SRC_DEFAULT;
  bus.glitch_ignore_cnt = 7;
  bus.flags.enable_internal_pullup = true;   // the board pulls up too; harmless
  err = i2c_new_master_bus(&bus, &s_bus);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
    return err;
  }

  // 16-bit register addresses, no control phase. The CST328 is not a display
  // controller -- esp_lcd_panel_io is only being used as an I2C transport.
  //
  // Spelled out rather than using ESP_LCD_TOUCH_IO_I2C_CST328_CONFIG(): that
  // macro is a C designated initialiser that skips members, which C++ compiles
  // but -Wextra then warns about. These are exactly the values it sets.
  esp_lcd_panel_io_i2c_config_t io = {};
  io.dev_addr = ESP_LCD_TOUCH_IO_I2C_CST328_ADDRESS;
  io.control_phase_bytes = 1;
  io.dc_bit_offset = 0;
  io.lcd_cmd_bits = 16;                      // register address width
  io.lcd_param_bits = 0;
  io.flags.disable_control_phase = 1;
  io.scl_speed_hz = TOUCH_I2C_HZ;
  err = esp_lcd_new_panel_io_i2c(s_bus, &io, &s_io);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "touch panel io failed: %s", esp_err_to_name(err));
    return err;
  }

  esp_lcd_touch_config_t cfg = {};
  cfg.x_max = LCD_H_RES;
  cfg.y_max = LCD_V_RES;
  cfg.rst_gpio_num = TOUCH_PIN_RST;
  // POLLING, not interrupt-driven -- deliberately.
  //
  // Passing TOUCH_PIN_INT here puts esp_lvgl_port into LV_INDEV_MODE_EVENT, so
  // LVGL only reads the controller when the INT line fires. That should be the
  // better mode, but on this board it made the panel feel dead: presses landed
  // late or not at all, because a report whose interrupt is missed is a report
  // LVGL never asks for. In polling mode LVGL reads every refresh period no
  // matter what the INT line does, which trades a little CPU for a digitiser
  // that answers every time it is touched.
  //
  // Set this back to TOUCH_PIN_INT if the INT wiring is ever confirmed good and
  // the CPU cost matters.
  cfg.int_gpio_num = GPIO_NUM_NC;
  cfg.levels.reset = 1;                      // see resetPinInit()
  cfg.levels.interrupt = 0;                  // INT is active low
  cfg.flags.swap_xy = TOUCH_SWAP_XY;
  cfg.flags.mirror_x = TOUCH_MIRROR_X;
  cfg.flags.mirror_y = TOUCH_MIRROR_Y;
  err = esp_lcd_touch_new_i2c_cst328(s_io, &cfg, &s_touch);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "CST328 init failed: %s", esp_err_to_name(err));
    s_touch = nullptr;
    return err;
  }

  vTaskDelay(pdMS_TO_TICKS(CST328_BOOT_MS));
  (void)esp_lcd_panel_io_tx_param(s_io, CST328_REG_NORMAL_MODE, nullptr, 0);

  // Attach to the display the LVGL port already created in uiInit(). It is
  // the only one, so it is LVGL's default; asking LVGL for it saves Ui from
  // having to publish its handle.
  lv_display_t* disp = lv_display_get_default();
  if (disp == nullptr) {
    ESP_LOGE(TAG, "no LVGL display -- uiInit() must run first");
    return ESP_ERR_INVALID_STATE;
  }

  lvgl_port_touch_cfg_t touch_cfg = {};
  touch_cfg.disp = disp;
  touch_cfg.handle = s_touch;
  // Digitiser and panel are both 240x320, so no scaling. 0 would mean the same
  // thing to the port, but say it out loud.
  touch_cfg.scale.x = 1.0f;
  touch_cfg.scale.y = 1.0f;
  s_indev = lvgl_port_add_touch(&touch_cfg);
  if (s_indev == nullptr) {
    ESP_LOGE(TAG, "lvgl_port_add_touch failed");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG,
           "CST328 up: i2c%d sda=%d scl=%d rst=%d int=%d, %dx%d, "
           "swap=%d mirror=%d/%d",
           static_cast<int>(TOUCH_I2C_PORT), TOUCH_PIN_SDA, TOUCH_PIN_SCL,
           TOUCH_PIN_RST, TOUCH_PIN_INT, LCD_H_RES, LCD_V_RES,
           static_cast<int>(TOUCH_SWAP_XY), static_cast<int>(TOUCH_MIRROR_X),
           static_cast<int>(TOUCH_MIRROR_Y));
  return ESP_OK;
}
