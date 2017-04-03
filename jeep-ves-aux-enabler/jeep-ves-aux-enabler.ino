/*****************************************************************************************
*
* Hack for my Jeep Grand Cherokee WH 2006 (european) + RAR radio head unit
*
* My hardware:
*  1. Arduino Pro Mini 5v/16Mhz
*  2. Mcp2515_can SPI module (8Mhz)
*
* Features:
*  1. Bench mode to enable radio functioning while removed from the car 
*  2. Emulating VES presense to enable AUX IN in head unit
*
* Copyright (C) 2015-2017 Anton Viktorov <latonita@yandex.ru>
*                                    https://github.com/latonita/jeep-canbus
*
* This is free software. You may use/redistribute it under The MIT License terms.
*
*****************************************************************************************/
#include <SPI.h>
#include "mcp_can.h"

#define CAN_MODULE_CS_PIN 10

#define CHECK_PERIOD_MS 200
#define ANNOUNCE_PERIOD_MS 500
#define BUTTON_PRESS_DEBOUNCE_MS 350

#define BENCH_MODE_ON // When radio is removed from the car it needs to receive power-on message regularly so it thinks key is on

MCP_CAN CAN(CAN_MODULE_CS_PIN);

unsigned long lastAnnounce = 0;

unsigned char msgVesAuxMode[8] = {3,0,0,0,0,0,0,0};

#ifdef BENCH_MODE_ON
unsigned char msgPowerOn[6]= {0x63,0,0,0,0,0};
#endif

#define RADIOMODE_OTHER 0
#define RADIOMODE_AUX 1
unsigned char radioMode = RADIOMODE_OTHER;

void setup() {
  Serial.begin(115200);
  Serial.println("Jeep VES Enabler by latonita v.1.0");

  // My Jeep 2006 uses low-speed interior can bus 83.3kbps. Newer models use 125kbps for interior can-b bus and call it 'high-speed'
  // I use 8MHz mcp2515_can SPI module, had to update mcp_can lib to be able to work with lower freq. originally is works only with 16MHz modules
  while(CAN_OK != CAN.begin(CAN_83K3BPS, MCP_8MHz)) {
    Serial.println("CAN init fail");
    delay(250);
  }
  Serial.println("CAN init ok");
}

void sendAnnouncements() {
#ifdef BENCH_MODE_ON
    // when on bench - send power on command to radio to enable it
    CAN.sendMsgBuf(0x0, 0, 6, msgPowerOn);
    delay(25);
#endif
    //tell them VES AUX is here
    CAN.sendMsgBuf(0x3dd, 0, 8, msgVesAuxMode);
    delay(25);
}

void checkIncomingMessages() {
  static unsigned int canId = 0;
  static unsigned char len = 0;
  static unsigned char buf[8];
  static unsigned char oldMode = radioMode;
  if(CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);
  }
  canId = CAN.getCanId();

  switch (canId) {
    case 0x09f:
      // current radio mode
      radioMode = (buf[0] & 0xF == 6) ? RADIOMODE_AUX : RADIOMODE_OTHER;

      if (oldMode != radioMode) {
        if (radioMode == RADIOMODE_AUX) {
          Serial.print("Radio Mode changed to AUX");
        } else {
          Serial.print("Radio Mode changed to something else");
        }
      }
      break;

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
