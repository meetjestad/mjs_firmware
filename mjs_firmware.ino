/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman, Bas Peschier, Harmen Zijp

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   In order to compile the following libraries need to be installed:
   - SparkFunHTU21D: https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library
   - NeoGPS: https://github.com/SlashDevin/NeoGPS
   - Adafruit_SleepyDog: https://github.com/adafruit/Adafruit_SleepyDog
   - lmic: https://github.com/matthijskooijman/arduino-lmic
 *******************************************************************************/

// include external libraries
#include <SPI.h>
#include "mjs_lmic.h"
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <SoftwareSerial.h>
#include <NMEAGPS.h>
#include <Adafruit_SleepyDog.h>

// set run mode
boolean const DEBUG = true;

// setup GPS module
byte const GPS_PIN = 8;
SoftwareSerial gpsSerial(GPS_PIN, GPS_PIN);
NMEAGPS gps;

// setup temperature and humidity sensor
HTU21D htu;
float temperature;
float humidity;

// define various pins
byte const SW_GND_PIN = 20;
byte const LED_PIN = 21;

// setup timing variables
long const UPDATE_INTERVAL = 900000;
int const GPS_TIMEOUT = 60000;
unsigned long lastUpdateTime = 0;
gps_fix gps_data;

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
      Serial.print(gps_data.longitudeL()/10000000.0, 6);
      Serial.print(F(","));
      Serial.println(gps_data.latitudeL()/10000000.0, 6);
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
  while (millis() - startTime < GPS_TIMEOUT && !gps_data.valid.location) {
    if (gps.available(gpsSerial))
      gps_data = gps.read();
  }
  digitalWrite(SW_GND_PIN, LOW);
  return gps_data.valid.location;
}

void sendData() {
  uint8_t data[14];

  // pack geoposition
  uint32_t lng24 = int32_t((int64_t)gps_data.longitudeL() * 32768 / 10000000);
  data[0] = lng24 >> 24 & 0xFF;
  data[1] = lng24 >> 16 & 0xFF;
  data[2] = lng24 >> 8 & 0xFF;

  uint32_t lat24 = int32_t((int64_t)gps_data.latitudeL() * 32768 / 10000000);
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

