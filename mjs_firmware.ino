/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman, Bas Peschier, Harmen Zijp

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   In order to compile the following libraries need to be installed:
   - SparkFunHTU21D: https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library
   - TinyGPS++: http://arduiniana.org/libraries/tinygpsplus/
   - Adafruit_SleepyDog: https://github.com/adafruit/Adafruit_SleepyDog
   - lmic: https://github.com/matthijskooijman/arduino-lmic
 *******************************************************************************/

// include external libraries
#include <SPI.h>
#include "mjs_lmic.h"
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_SleepyDog.h>

// set run mode
boolean DEBUG = true;

// setup GPS module
byte GPS_PIN = 8;
SoftwareSerial gpsSerial(GPS_PIN, GPS_PIN);
TinyGPSPlus gps;

// setup temperature and humidity sensor
HTU21D htu;
float temperature;
float humidity;

// define various pins
byte SW_GND_PIN = 20;
byte LED_PIN = 21;

// setup timing variables
long UPDATE_INTERVAL = 900000;
int GPS_TIMEOUT = 5000;
unsigned long lastUpdateTime = 0;

void setup() {
  // setup LoRa transceiver
  mjs_lmic_setup();

  // setup switched ground and power down connected peripherals (GPS module)
  pinMode(SW_GND_PIN, OUTPUT);
  digitalWrite(SW_GND_PIN, LOW);

  // blink 'hello'
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  // when in debugging mode start serial connection
  if(DEBUG) {
    Serial.begin(9600);
    Serial.println(F("Start"));
  }

  // start communication to sensors
  htu.begin();
  gpsSerial.begin(9600);
}

void loop() {
  // depending on the DEBUG variable enter either a function with regular measurements and verbose feedback over serial monitor...
  if (DEBUG) debugLoop();
  // ... or an energysaving mode of measurements alternated with deep sleep
  else sleepLoop();

  // keep the transceiver going
  updateTransceiver();
}

void debugLoop() {
  unsigned long currentTime = millis();
  boolean hasPosition = false;

  if ((currentTime - lastUpdateTime) > UPDATE_INTERVAL || lastUpdateTime == 0) {
    Serial.print(F("lng/lat: "));

    hasPosition = getPosition();

    if (hasPosition) {
      Serial.print(gps.location.lng(), 6);
      Serial.print(F(","));
      Serial.println(gps.location.lat(), 6);
    }
    else {
      Serial.println(F("No GPS found: check wiring"));
    }

    Serial.print(F("tmp/hum: "));

    temperature = htu.readTemperature();
    humidity = htu.readHumidity();

    Serial.print(temperature, 1);
    Serial.print(F(","));
    Serial.println(humidity, 1);
    Serial.flush();

    digitalWrite(LED_PIN, HIGH);
    sendData();
    digitalWrite(LED_PIN, LOW);

    lastUpdateTime = currentTime;
  }
}

void sleepLoop() {
  // We need to calculate how long we should sleep, so we need to know how long we were awake
  unsigned long startMillis = millis();

  // Activate and read our sensors
  getPosition();
  temperature = htu.readTemperature();
  humidity = htu.readHumidity();

  // We can now send the data
  sendData();

  // Schedule sleep
  unsigned long sleepDuration = UPDATE_INTERVAL - (millis() - startMillis);
  doSleep(sleepDuration);
}

void doSleep(uint32_t time) {
  while (time > 0) {
    int slept;
    if (time < 8000)
      slept = Watchdog.sleep(time);
    else
      slept = Watchdog.sleep(8000);

    if (slept >= time)
      return;
    time -= slept;
  }
}

boolean getPosition()
{
  digitalWrite(SW_GND_PIN, HIGH);
  unsigned long startTime = millis();
  while (millis() - startTime < GPS_TIMEOUT) {
    if (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isValid()) {
          digitalWrite(SW_GND_PIN, LOW);
          return true;
        }
      }
    }
  }
  digitalWrite(SW_GND_PIN, LOW);
  return false;
}

void sendData() {
  uint8_t data[14];

  // pack geoposition
  uint32_t lng24 = int32_t(gps.location.lng() * 32768) << 8;
  data[0] = lng24 >> 24 & 0xFF;
  data[1] = lng24 >> 16 & 0xFF;
  data[2] = lng24 >> 8 & 0xFF;

  int32_t lat24 = (uint32_t)(gps.location.lat() * 32768) << 8;
  data[3] = lat24 >> 24 & 0xFF;
  data[4] = lat24 >> 16 & 0xFF;
  data[5] = lat24 >> 8 & 0xFF;

  // pack temperature and humidity
  int16_t tmp16 = (uint16_t)(temperature * 16) << 4;
  data[6] = tmp16 >> 8 & 0xFF;
  data[7] = tmp16 & 0xF0;

  int16_t hum16 = (uint16_t)(humidity * 16);
  data[7] |= hum16 >> 8 & 0x0F;
  data[8] = hum16 & 0xFF;

  // pack other values: rain, light, moist
  // ...
  data[10] = 0;
  data[11] = 0;
  data[12] = 0;
  data[13] = 0;

  // send over LoRaWAN, using experiment identifier for port selection
  if (LMIC.opmode & OP_TXRXPEND) {
    if(DEBUG) Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, data, sizeof(data), 0);
    if(DEBUG) Serial.println(F("Packet queued"));
  }
}

