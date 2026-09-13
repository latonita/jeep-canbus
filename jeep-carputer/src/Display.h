#pragma once
// Display bring-up for the ESP32-S3-Touch-LCD-2.8: backlight, SPI bus, and the
// ST7789 panel driver that is built into ESP-IDF's esp_lcd.
//
// Deliberately no LVGL and no touch here. This exists to answer the questions
// that block the UI work -- are the pins right, does the panel need colour
// inversion and mirroring, is the geometry what we think -- with as little
// else in the way as possible.
//
// Every value is cross-checked against Waveshare's own IDF demo for this
// board, not guessed: SPI3, MOSI 45, SCK 40, MISO -1, CS 42, DC 41, RST 39,
// backlight 5 via LEDC, 240x320, no offset, BGR, 16 bpp, mirror(true, false).
#include <stdint.h>

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

// RGB565. White is byte-order-invariant, so it proves the panel is alive even
// if the red/blue channels turn out to need swapping.
#define RGB565_BLACK   0x0000u
#define RGB565_WHITE   0xFFFFu
#define RGB565_RED     0xF800u
#define RGB565_GREEN   0x07E0u
#define RGB565_BLUE    0x001Fu

esp_err_t displayInit(void);

// 0..100. Backlight is PWM (LEDC) rather than a plain GPIO, so this dims.
esp_err_t displayBacklight(uint8_t percent);

// Finer-grained sibling: 0..1000 of full duty. The backlight is 13-bit, so
// whole percent cannot express the low end -- 1% is 81 counts and still looks
// lit. Use this wherever "barely visible" matters.
esp_err_t displayBacklightPermille(uint16_t permille);

// Fills the whole panel, one row at a time from a small DMA buffer -- a full
// 240x320x2 framebuffer is 150 KB and PSRAM is still switched off.
esp_err_t displayFill(uint16_t rgb565);

// Handles for the LVGL port to adopt. Valid only after displayInit().
esp_lcd_panel_handle_t    displayPanel(void);
esp_lcd_panel_io_handle_t displayPanelIo(void);
