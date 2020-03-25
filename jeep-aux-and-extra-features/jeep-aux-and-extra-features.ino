/*****************************************************************************************

   Hack for my Jeep Compass 2007 (european) + RAR radio head unit

   My hardware:
    1. ATmega8 / 12Mhz (simolar Arduino NG)
    2. MCP2515_CAN SPI module (8Mhz)
    3. Generic bluetooth A2DP module from some BT Speaker with HF and 3 buttons.

   Features:
    1. Bench mode to enable radio functioning while removed from the car.
    2. Emulating VES presense to enable AUX IN in head unit.
    3. Turn the BT module on/off when the radio is on, and switch to AUX when the mobile phone receives a call to use HF.
    4. Control BT audio source with seek buttons.
    5. Speed-depended Automatic Volume Control (AVC).
    6. Power Save sleep mode then canbus is off.

   Copyright (C) 2015-2017 Anton Viktorov <latonita@yandex.ru>
                                      https://github.com/latonita/jeep-canbus

   Copyright (C) 2020 Renemen         https://github.com/renemen64/jeep-canbus

   This is free software. You may use/redistribute it under The MIT License terms.

 *****************************************************************************************/
#include <SPI.h>
#include <avr/sleep.h>
#include "mcp_can.h"

#define CAN_MODULE_CS_PIN 10
#define CAN_MODULE_INT_PIN 3
#define BLUETOOTH_SWITCH 8
#define BLUETOOTH_PLAY 7
#define BLUETOOTH_NXT 6
#define BLUETOOTH_PRV 4
#define BLUETOOTH_RING 2

#define CHECK_PERIOD_MS 250
#define ANNOUNCE_PERIOD_MS 1000
#define SLEEP_PERIOD_MS 10000 // time to sleep after last can messege
#define BUTTON_PRESS_DEBOUNCE_MS 250
#define AVC_DIVIDER 19
#define DBG_CAN_ON  // Send all CAN data to serial port
//#define BENCH_MODE_ON // When radio is removed from the car it needs to receive power-on message regularly so it thinks key is on

unsigned long ringTime = 0;
unsigned long lastCheck = 0;
unsigned long lastAnnounce = 0;
unsigned long lastButtonPress = 0;
volatile bool incomRing = false;
//volatile bool flagRecv = false;
#ifdef DBG_CAN_ON
byte serialMsg = 0;
bool debugOn = false;
#endif
#define arrayLenght(array) sizeof(array)/sizeof(array[0]) // compile-time calculation
byte msgVesAuxMode[8] = {3, 0, 0, 0, 0, 0, 0, 0};
byte setRadioDiagMode1[8] = {2, 0x10, 0x92, 0, 0, 0, 0, 0};
byte setVesAuxMode[8] = {5, 0x30, 0x1c, 7, 0, 3, 0, 0};
byte setRadioVolUp[2] = {2, 0};
byte setRadioVolDn[2] = {4, 0};
byte setWheelBtnOk[2] = {0, 0};
#ifdef BENCH_MODE_ON
byte msgPowerOn[6] = {0x63, 0, 0, 0, 0, 0};
#endif

byte carSpeed = 0;
byte radioAVC = 0;
byte lastSpeed = 0;
bool enableVES = false;
bool auxMode = false;
bool newMode = false;
bool powerOn = false;
//#define CAN_MULTI_SWITCH 0x11d
//#define CAN_HEADLIGHTS 0x1c8
//#define CAN_BLINKERS 0x006
#define CAN_POWER 0x000
#define CAN_VEHICLE 0x002  // read speed
#define CAN_RADIO_MODE 0x09f
#define CAN_RADIO_DIAG 0x6b0
#define CAN_RADIO_INFO 0x516
#define CAN_RADIO_BUTTONS 0x394
#define CAN_WHEEL_BUTTONS 0x3a0
#define CAN_RADIO_VOLUME 0x3d0
#define CAN_VES_UNIT 0x3dd

unsigned int canId = 0;
byte len = 0;
byte buf[8];

MCP_CAN CAN(CAN_MODULE_CS_PIN);

/*
  void pinsSetup() {
  pinMode(BLUETOOTH_SWITCH, OUTPUT);
  pinMode(BLUETOOTH_PLAY, OUTPUT);
  pinMode(BLUETOOTH_NXT, OUTPUT);
  pinMode(BLUETOOTH_PRV, OUTPUT);
  pinMode(BLUETOOTH_RING, INPUT_PULLUP);
  digitalWrite(BLUETOOTH_SWITCH, HIGH);
  digitalWrite(BLUETOOTH_PLAY, LOW);
  digitalWrite(BLUETOOTH_NXT, LOW);
  digitalWrite(BLUETOOTH_PRV, LOW);
  digitalWrite(BLUETOOTH_RING, HIGH);
  }
*/

void setupFilters() {
  CAN.init_Mask(0, 0, 0x7ff);
  CAN.init_Mask(1, 0, 0x7ff);

  CAN.init_Filt(0, 0, CAN_VEHICLE);
  CAN.init_Filt(1, 0, CAN_RADIO_BUTTONS);
  CAN.init_Filt(2, 0, CAN_WHEEL_BUTTONS);
  CAN.init_Filt(3, 0, CAN_RADIO_MODE);
  CAN.init_Filt(4, 0, CAN_RADIO_VOLUME);
  CAN.init_Filt(5, 0, CAN_RADIO_INFO);
}

void setup() {
  //  pinsSetup();
  pinMode(BLUETOOTH_SWITCH, OUTPUT);
  digitalWrite(BLUETOOTH_SWITCH, HIGH);
  Serial.begin(115200);
  Serial.println("Jeep VES Enabler + Buttons v1.3b");
  while (CAN_OK != CAN.begin(CAN_83K3BPS, MCP_8MHz)) {
    Serial.println("CAN init Fail");
    delay(CHECK_PERIOD_MS);
  }
  Serial.println("CAN init OK");
#ifdef DBG_CAN_ON
  Serial.println("Press 1 to on/off CAN debug");
  Serial.println("Press 4 to enable CAN filters");
#else
  setupFilters();
#endif
  attachInterrupt(digitalPinToInterrupt(BLUETOOTH_RING), blueRing, FALLING);
  CAN.setSleepWakeup(1); // this tells the MCP2515 to wake up on incoming messages
  lastCheck = millis();
}

void pressButton(byte pin) {
  if (auxMode) {
    digitalWrite(pin, HIGH);
    delay (BUTTON_PRESS_DEBOUNCE_MS);
    digitalWrite(pin, LOW);
    lastButtonPress = millis();
  }
}

void blueRing () { // function for INT 0
  incomRing = true;
}

//EMPTY_INTERRUPT (PCINT1_vect);
void MCP2515_ISR() { // function for INT 1 mostly for wakeup with CAN
  //bool flagRecv = false;
}

void checkIncomingMessages() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    //flagRecv = false;
    lastCheck = millis();
    enableVES = true;
    CAN.readMsgBuf(&len, buf);
    canId = CAN.getCanId();
    switch (canId) {
      case CAN_RADIO_MODE:
        newMode = ((buf[0] & 0xF) == 6) ? true : false;
        powerOn = ((buf[0] & 0xF) == 7) ? false : true;
        if (auxMode != newMode) {
          if (newMode) {
            Serial.println("Radio Mode AUX");
          } else {
            Serial.println("Radio Mode OTHER");
            pressButton(BLUETOOTH_PLAY);
          }
          auxMode = newMode;
        }
        break;
      case CAN_RADIO_BUTTONS:
        // buttons decode
        if (buf[3] > 0 && millis() > lastButtonPress + BUTTON_PRESS_DEBOUNCE_MS) { // something pressed on radio
          switch (buf[3]) {
            case 0x01: // seek right
              Serial.println("Seek >>|");
              pressButton(BLUETOOTH_NXT);
              break;
            case 0x02: // seek left
              Serial.println("|<< Seek");
              pressButton(BLUETOOTH_PRV);
              break;
            case 0x04: // rw/ff right
              Serial.println("FF >>");
              pressButton(BLUETOOTH_NXT);
              break;
            case 0x08: // rw/ff left
              Serial.println("<< RW");
              pressButton(BLUETOOTH_PRV);
              break;
            case 0x20: // RND/PTY
              Serial.println("RND/PTY");
              pressButton(BLUETOOTH_PLAY);
              break;
          }
        }
        break;
      case CAN_WHEEL_BUTTONS:
        if (buf[0] > 0 && millis() > lastButtonPress + BUTTON_PRESS_DEBOUNCE_MS) { // something pressed on steering wheel
          switch (buf[0]) {
            case 0x01: // right select (mode change)
              Serial.println("R_SEL");
              break;
            case 0x02: // right up (vol+)
              Serial.println("R_UP");
              break;
            case 0x04: // right down (vol-)
              Serial.println("R_DN");
              break;
            case 0x08: // left up (seek+)
              Serial.println("L_UP");
              break;
            case 0x10: // left down (seek-)
              Serial.println("L_DN");
              break;
            case 0x20: // left select (preset/disk change)
              Serial.println("L_SEL");
              pressButton(BLUETOOTH_PLAY);
              break;
          }
        }
        break;
      case CAN_VEHICLE:
        // engineRpm = (can_MsgRx.data[0] << 8) + can_MsgRx.data[1];
        carSpeed = ((buf[2] << 8) + buf[3]) >> 7;
        if (powerOn) volumeCtrl();
        break;
    }
#ifdef DBG_CAN_ON
    if (debugOn) {
      // some debug output if pressed "1" in terminal
      Serial.print("\t");
      Serial.print(canId, HEX);
      Serial.print("\t");
      for (byte i = 0; i < len; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
#endif
  }
}

void volumeCtrl () {
  if (radioAVC < (carSpeed / AVC_DIVIDER)) {
    Serial.print ("\tAVC\t"); Serial.print (radioAVC); Serial.println ("\tVOL+");
    CAN.sendMsgBuf(CAN_WHEEL_BUTTONS, 0, 2, setRadioVolUp); //simulate a button press on the steering wheel
    delay(125);
    CAN.sendMsgBuf(CAN_WHEEL_BUTTONS, 0, 2, setWheelBtnOk); //simulate a button release
    delay (10);
  }
  else if (radioAVC > (carSpeed / AVC_DIVIDER)) {
    Serial.print ("\tAVC\t"); Serial.print (radioAVC); Serial.println ("\tVOL-");
    CAN.sendMsgBuf(CAN_WHEEL_BUTTONS, 0, 2, setRadioVolDn);
    delay(125);
    CAN.sendMsgBuf(CAN_WHEEL_BUTTONS, 0, 2, setWheelBtnOk);
    delay (10);
  }
  radioAVC = carSpeed / AVC_DIVIDER;
}

void lowPower() {
  CAN.sleep();
  Serial.println ("Sleep");
  digitalWrite(BLUETOOTH_SWITCH, HIGH);
  delay (BUTTON_PRESS_DEBOUNCE_MS);
  Serial.flush();
  cli(); // Disable interrupts
  ADCSRA = 0;
  ACSR = 0;
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_enable();
  attachInterrupt(digitalPinToInterrupt(CAN_MODULE_INT_PIN), MCP2515_ISR, LOW); // my mega8 wakeup only at LOW level
  sei(); // Enable interrupts
  sleep_mode();
  // Now the Arduino sleeps until the next message arrives...
  sleep_disable();
  detachInterrupt(digitalPinToInterrupt(CAN_MODULE_INT_PIN));
  sei();
  CAN.wake();
  Serial.println ("Woke Up");
}

void loop() {
#ifdef DBG_CAN_ON
  if (Serial.available() > 0) {
    serialMsg = Serial.read();
    if (serialMsg == 49) {
      Serial.println("\tCAN ID \tDATA ");
      debugOn = !debugOn;
      serialMsg = 0;
    }
    else if (serialMsg == 52) {
      Serial.println("4 CAN Filters");
      setupFilters();
      serialMsg = 0;
    }
    /*
      else if (serialMsg == 50) {
      Serial.println("2 Set AUX");
      CAN.sendMsgBuf(CAN_RADIO_DIAG, 0, arrayLenght(setVesAuxMode), setVesAuxMode);
      serialMsg = 0;
      }
      else if (serialMsg == 51) {
      Serial.println("3 Diag Msg1");
      CAN.sendMsgBuf(CAN_RADIO_DIAG, 0, arrayLenght(setRadioDiagMode1), setRadioDiagMode1);
      serialMsg = 0;
      }
      else if (serialMsg == 53) {
      Serial.println("5 Diag Msg3");
      CAN.sendMsgBuf(CAN_RADIO_DIAG, 0, arrayLenght(setRadioDiagMode3), setRadioDiagMode3);
      serialMsg = 0;
      }
      else if (serialMsg == 54) {
      Serial.println("6 Diag Msg4");
      CAN.sendMsgBuf(CAN_RADIO_DIAG, 0, arrayLenght(setRadioDiagMode4), setRadioDiagMode4);
      serialMsg = 0;
      }
    */
  }
#endif
  if (millis() > lastAnnounce + ANNOUNCE_PERIOD_MS) {
    lastAnnounce = millis();
    // tell them VES AUX is here then CAN bus not sleep
    if (enableVES) {
      CAN.sendMsgBuf(CAN_VES_UNIT, 0, arrayLenght(msgVesAuxMode), msgVesAuxMode);
      delay (10);
      //CAN.sendMsgBuf(CAN_WHEEL_BUTTONS, 0, 2, setWheelBtnOk);  // uncoment this if not have steering wheel control buttons
      //delay (10);
      enableVES = false;
      if (powerOn) digitalWrite(BLUETOOTH_SWITCH, LOW);
      else digitalWrite(BLUETOOTH_SWITCH, HIGH);
    }
#ifdef BENCH_MODE_ON
    CAN.sendMsgBuf(CAN_POWER, 0, arrayLenght(msgPowerOn), msgPowerOn);
    delay (10);
#endif
  }
  // Сhange audio source then bluetooth receive a call.
  // In my case, the Bluetooth module does not have a separate output for a call signal, I had to use the output for audio amplifier.
  if (incomRing) {
    ringTime = millis() - ringTime ;
    if (ringTime > 4900 && ringTime < 5020) {
      CAN.sendMsgBuf(CAN_RADIO_DIAG, 0, arrayLenght(setRadioDiagMode1), setRadioDiagMode1);
      delay(50);
      CAN.sendMsgBuf(CAN_RADIO_DIAG, 0, arrayLenght(setVesAuxMode), setVesAuxMode);
      delay (10);
      Serial.println("Incoming Call");
    }
    ringTime = millis();
    //Serial.println (ringTime);
    incomRing = false;
  }
  checkIncomingMessages();
  if (millis() > lastCheck + SLEEP_PERIOD_MS) {
    lowPower();
    lastCheck = millis();
  }
}
