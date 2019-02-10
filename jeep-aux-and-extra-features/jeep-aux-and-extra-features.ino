/*****************************************************************************************
 *
 * Hack for my Jeep Grand Cherokee WH 2006 (european) + RAR radio head unit
 *
 * My hardware:
 *  1. Arduino Pro Mini 5v/16Mhz
 *  2. Mcp2515_can SPI module (8Mhz)
 *  3. Generic (H166) bluetooth A2DP module based on OVC3860, UART connected to Arduino
 *
 * Features:
 *  1. Bench mode to enable radio functioning while removed from the car
 *  2. Emulating VES presense to enable AUX IN in head unit
 *  2.1. Turn on/off BT A2DP module only when AUX IN is selected
 *  3. Control BT audio source with seek buttons
 *  4. [todo] Signal comfort module - blinkers blink three times instead of one.
 *  5. [todo] Headlights - always on
 *  6. [todo] Auto wipers - require installation of rain sensor (? maybe it will work alone, w/o my software :)
 *  7. [todo] Auto parameters display
 *  7.1. gasoline cost for a trip
 *
 * Copyright (C) 2015-2017 Anton Viktorov <latonita@yandex.ru>
 *                                    https://github.com/latonita/jeep-canbus
 *
 * This is free software. You may use/redistribute it under The MIT License terms.
 *
 *****************************************************************************************/
#include <SoftwareSerial.h>
#include <SPI.h>
#include "mcp_can.h"

#define CAN_MODULE_CS_PIN 10 // PB2

#define BLUETOOTH_SWITCH A1 // 15 //PC1

// BT TX -------- RX A4(18)
// BT RX -- >|--- TX A2(19)
// My BT module is 3.3v, but arduino is 5v.
// Need to put blocking diode: do not let Arduinos 5v get to BT, but let it easily pulldown a line
// Any diode with low Vf will do. In my case I put Schottky IN5819 desoldered from some dead device
#define BLUETOOTH_TX A4
#define BLUETOOTH_RX A2
SoftwareSerial btSerial(BLUETOOTH_TX, BLUETOOTH_RX);

#define CHECK_PERIOD_MS 200
#define ANNOUNCE_PERIOD_MS 500
#define BUTTON_PRESS_DEBOUNCE_MS 350

#define CAN_DELAY_AFTER_SEND 20

//#define BENCH_MODE_ON

MCP_CAN CAN(CAN_MODULE_CS_PIN);

unsigned long lastCheck = 0;
unsigned long lastAnnounce = 0;
unsigned long lastButtonPress = 0;

#define msgVesAuxModeLen 8
unsigned char msgVesAuxMode[8] = {3, 0, 0, 0, 0, 0, 0, 0};

#ifdef BENCH_MODE_ON
#define msgPowerOnLen 6
unsigned char msgPowerOn[6] = {0x63, 0, 0, 0, 0, 0};
#endif

#define RADIOMODE_OTHER 0
#define RADIOMODE_AUX 1
unsigned char radioMode = RADIOMODE_OTHER;

const char compileDate[] = __DATE__ " " __TIME__;

#define PIN_DOWN(x)                                                                                                                                  \
  pinMode(x, OUTPUT);                                                                                                                                \
  digitalWrite(x, LOW);

void pinsSetup() {
  /*  PIN_DOWN(2);
    PIN_DOWN(3);
    PIN_DOWN(4);
    PIN_DOWN(5);
    PIN_DOWN(6);
    PIN_DOWN(7);
    PIN_DOWN(8);
    PIN_DOWN(9);
  */
  pinMode(BLUETOOTH_SWITCH, OUTPUT);
  digitalWrite(BLUETOOTH_SWITCH, LOW);
}

#define CAN_POWER 0x000
#define CAN_BLINKERS 0x006
#define CAN_RADIO_MODE 0x09f
#define CAN_RADIO_CONTROLS 0x394
#define CAN_VES_UNIT 0x3dd

//#define CAN_MULTI_SWITCH   0x11d
//#define CAN_HEADLIGHTS 0x1c8

// void setupFilters() {
//  CAN.init_Mask(0, 0, 0x3ff);
//  CAN.init_Mask(1, 0, 0x3ff);
//
//  CAN.init_Filt(0, 0, CAN_POWER);
//  CAN.init_Filt(1, 0, CAN_RADIO_MODE);
//  CAN.init_Filt(2, 0, CAN_RADIO_CONTROLS);
//  CAN.init_Filt(3, 0, CAN_BLINKERS);
//  CAN.init_Filt(4, 0, 0x08);
//  CAN.init_Filt(5, 0, 0x09);
//}

void setup() {
  //  pinsSetup();

  Serial.begin(115200);
  Serial.print("Jeep VES Enabler + Extras by latonita v.1.1, ");
  Serial.println(compileDate);

  btSerial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_83K3BPS, MCP_8MHz)) {
    Serial.println("CAN init fail");
    delay(250);
  }
  Serial.println("CAN init ok");

  // TODO: set hardware filter to minimize number messages which come through
  // setupFilters();
}

void sendAnnouncements() {
#ifdef BENCH_MODE_ON
  // when on bench - send power on command to radio to enable it
  CAN.sendMsgBuf(CAN_POWER, 0, msgPowerOnLen, msgPowerOn);
  delay(CAN_DELAY_AFTER_SEND);
#endif
  // tell them VES AUX is here
  CAN.sendMsgBuf(CAN_VES_UNIT, 0, msgVesAuxModeLen, msgVesAuxMode);
  delay(CAN_DELAY_AFTER_SEND);
}

unsigned int canId = 0;
unsigned char len = 0;
unsigned char buf[8];
unsigned char newMode = 0;

void checkIncomingMessages() {

  if (CAN_MSGAVAIL != CAN.checkReceive())
    return;

  CAN.readMsgBuf(&len, buf);
  canId = CAN.getCanId();

  switch (canId) {
  case CAN_RADIO_MODE:

    // some debug output
    Serial.print("CAN ID: ");
    Serial.print(canId, HEX);

    for (int i = 0; i < len; i++) {
      Serial.print(",");
      Serial.print(buf[i], HEX);
    }
    Serial.println();
    Serial.print("Radio mode: ");
    Serial.print(buf[0] & 0xF, HEX);
    Serial.print(":");
    Serial.print(radioMode);
    Serial.print(">");
    Serial.println(newMode);

    newMode = ((buf[0] & 0xF) == 6) ? RADIOMODE_AUX : RADIOMODE_OTHER;

    if (radioMode != newMode) {
      radioMode = newMode;
      digitalWrite(BLUETOOTH_SWITCH, radioMode == RADIOMODE_AUX ? HIGH : LOW);
      if (radioMode == RADIOMODE_AUX) {
        Serial.print("Radio Mode changed to AUX");
      } else {
        Serial.print("Radio Mode changed to something else");
      }
    }
    break;
  case CAN_RADIO_CONTROLS:
    // radio mode + buttons
    if (buf[3] > 0 && millis() > lastButtonPress + BUTTON_PRESS_DEBOUNCE_MS) { // something pressed
      lastButtonPress = millis();
      if (buf[3] & 1) { // seek right
        Serial.println("-- Seek >>");
        Serial.write("AT#MD\n\r");
        btSerial.write("AT#MD\n\r");
      } else if (buf[3] & 2) { // seek left
        Serial.println("<< Seek --");
        Serial.write("AT#ME\n\r");
        btSerial.write("AT#ME\n\r");
      } else if (buf[3] & 4) { // rw/ff right
        Serial.println("-- RW FF >>");
      } else if (buf[3] & 8) { // rw/ff left
        Serial.println("<< RW FF --");
      } else if (buf[3] & 0x20) { // RND/PTY
        Serial.println("[RND/PTY] - use as play/pause");
        Serial.write("AT#MA\n\r");
        btSerial.write("AT#MA\n\r");
      }
    }

  default:
    break;
  }
}
void loop() {
  if (millis() > lastAnnounce + ANNOUNCE_PERIOD_MS) {
    lastAnnounce = millis();
    sendAnnouncements();
  }
  checkIncomingMessages();
}
