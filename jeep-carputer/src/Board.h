#pragma once
// Waveshare ESP32-S3-Touch-LCD-2.8 (SKU 27690).
//
// Pin assignments below come from Waveshare's schematic, not from guesswork:
//   https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8/ESP32-S3-Touch-LCD-2.8.pdf
//
// This is a display-first board. Almost every GPIO is committed:
//   0            BOOT (strapping)
//   1,2,3,4      touch I2C -- SDA, RST, SCL (strapping), INT
//   5            LCD backlight, via an 8050 NPN, active high
//   6,7,8        battery button / control / ADC sense
//   9,10,11      RTC INT + the shared I2C bus (IMU, RTC, external header)
//   12,13        IMU interrupts
//   14,16,17,21  TF card (SDMMC, 1-bit mode)
//   19,20        USB D-/D+ -- native USB-Serial-JTAG, do NOT repurpose
//   26..32       SPI flash
//   33..37       octal PSRAM ("internal occupancy" on the schematic)
//   38,47,48     I2S audio
//   39..42       LCD RST/SCK/DC/CS  (also the JTAG pins, so no external probe)
//   45,46        LCD MOSI/MISO (both strapping)
//
// That leaves exactly four usable pins, all on JST-SH 1.0 mm connectors rather
// than 0.1" headers -- the supplied cables are required, jumper wires will not
// fit:
//   43, 44   UART0 TX/RX
//   15, 18   spare (J8), but also wired to SD_D1/SD_D2 with 10k pull-ups
#include "driver/gpio.h"
#include "driver/uart.h"

// ---- Link to the ATmega328 in the radio ---------------------------------
//
// UART0 on IO43/IO44. Free despite the name because this board has no
// USB-serial bridge chip: USB-C goes straight to GPIO19/20 and the console is
// native USB-Serial-JTAG, so UART0 is entirely ours and printf debugging still
// works over USB at the same time.
#define LINK_UART_PORT   UART_NUM_0
#define LINK_UART_TX     GPIO_NUM_43   // -> 328 RX (its D0)
#define LINK_UART_RX     GPIO_NUM_44   // <- 328 TX (its D1)
#define LINK_UART_BAUD   115200

// Level shifting: the 328 board runs at 5 V, this at 3.3 V.
//   ESP32 -> 328  direct. 3.3 V clears the AVR's input threshold at 5 V Vcc.
//   328 -> ESP32  MUST be divided or diode-clamped. 5 V on an S3 pin is out of
//                 spec; use a divider, or the same Schottky trick the 328 board
//                 already uses for its Bluetooth module.
//
// Known wart: the ROM bootloader prints its boot log on U0TXD (IO43) at every
// reset, regardless of the console setting, so the 328 receives a burst of
// noise whenever the ESP32 restarts. Silencing it needs an eFuse change, not a
// config option. It is harmless in practice -- the 328's parser answers "?" to
// anything it cannot read, and worst case a stray hex-looking line transmits
// one diagnostic request frame with whatever ATSH holds, which no module acts
// on. At328Link::begin() sends a bare CR and drains before handshaking.

// ---- Later: GPS ----------------------------------------------------------
// UART1 on the two J8 spares. Note they double as SD_D1/SD_D2: Waveshare's
// 1-bit SDMMC mode leaves those idle so the pins are usable, but a card in the
// slot while we drive them is a contention risk.
#define GPS_UART_PORT    UART_NUM_1
#define GPS_UART_RX      GPIO_NUM_15
#define GPS_UART_TX      GPIO_NUM_18

// ---- Not viable on this board -------------------------------------------
// LoRa needs ~5-6 pins (SCK/MOSI/MISO/CS/RST/BUSY) and there are none left
// once the link and GPS are wired. It only fits by sharing the LCD's SPI3 bus
// or the TF card's pins, neither of which is broken out -- that means
// soldering to the LCD flex or the card socket. See ARCHITECTURE.md; the
// I/O-heavy role probably wants a different board.

// ---- Display (deferred) --------------------------------------------------
// Recorded now so it is not re-researched later. V1 boards have a CST328 touch
// controller, V2 (shipping since 2026-06-05) a CST3530 -- different registry
// components with different APIs, so the board label decides which.
#define LCD_HOST         SPI3_HOST
#define LCD_PIN_SCK      GPIO_NUM_40
#define LCD_PIN_MOSI     GPIO_NUM_45
#define LCD_PIN_MISO     (-1)          // routed but unused
#define LCD_PIN_DC       GPIO_NUM_41
#define LCD_PIN_CS       GPIO_NUM_42
#define LCD_PIN_RST      GPIO_NUM_39
#define LCD_PIN_BL       GPIO_NUM_5    // LEDC, 5 kHz, 13-bit, active high
#define LCD_H_RES        240
#define LCD_V_RES        320
#define LCD_PCLK_HZ      (80 * 1000 * 1000)

// Rows per LVGL flush chunk, and the SPI transfer ceiling that must cover it.
//
// These live together because they were once apart and silently disagreed:
// Display.cpp sized the SPI bus from its own FILL_ROWS (40 -> 19200 bytes)
// while LVGL was handing it 48-row chunks (23040 bytes). esp_lcd quietly split
// every single flush into TWO DMA transactions with an extra interrupt, all
// frame, for the life of the project. Nothing failed, so nothing said so.
//
// 80 rows rather than 48: a full screen is then 4 transfers instead of 14, and
// each one costs a queue round trip plus the synchronous drain that esp_lcd
// does on every CASET/RASET. Two buffers of 240 x 80 x 2 is 77 KB of internal
// DMA RAM against ~240 KB free.
// 60, NOT 80, AND THE 20 ROWS WENT TO THE LVGL POOL.
//
// Two of these are double-buffered at 16 bpp, so the pair costs
// LCD_H_RES * LCD_DRAW_ROWS * 2 * 2 bytes -- 75 KB at 80 rows, 56 KB at 60 --
// out of the SAME internal SRAM the LVGL pool is carved from. The two are in
// direct competition and nothing in the build says so, which is how this was
// spent twice.
//
// It had to move because the seventh page arrived. The widget tree went from
// 39 KB to 67 KB of a 96 KB pool, leaving ~20 KB free -- and the FAULTS
// drill-down then builds ~42 more objects on demand and could not get a DRAW
// TASK on top of them: lv_draw_add_task -> LV_ASSERT_MALLOC -> while(1), a
// silent freeze with the watchdog naming taskLVGL. Raising the pool to 112 KB
// alone failed the other way, with esp_lvgl_port refusing buf2 at boot and no
// display at all. Freeing 18.75 KB here is what lets the pool have its 16.
//
// THE COST IS RENDER CHUNKS, NOT QUALITY. A 320 px page is 4 flushes at 80 rows
// and 6 at 60; measured render is 10-28 ms against a 100 ms budget, so there is
// room for the extra two. If that stops being true, this is the number to put
// back -- and then the dial's 24 dots are the next thing to argue about.
#define LCD_DRAW_ROWS    60
#define LCD_MAX_TRANSFER_SZ (LCD_H_RES * LCD_DRAW_ROWS * 2)
// From Waveshare's own init: colour inverted (raw 0x21 INVON),
// mirror(true, false), no gap, BGR element order.

#define TOUCH_I2C_PORT   I2C_NUM_1
#define TOUCH_PIN_SDA    GPIO_NUM_1
#define TOUCH_PIN_SCL    GPIO_NUM_3
#define TOUCH_PIN_RST    GPIO_NUM_2
#define TOUCH_PIN_INT    GPIO_NUM_4
#define TOUCH_I2C_HZ     400000
