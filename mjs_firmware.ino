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
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <SoftwareSerial.h>
#include <NMEAGPS.h>
#include <Adafruit_SleepyDog.h>

// set run mode
boolean const DEBUG = true;

#include "mjs_lmic.h"

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
// Update GPS position after transmitting this many updates
int const GPS_UPDATE_RATIO = 24*4

unsigned long lastUpdateTime = 0;
unsigned long updatesBeforeGpsUpdate = 0;
gps_fix gps_data;

int const LORA_PORT = 10;

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
  // We need to calculate how long we should sleep, so we need to know how long we were awake
  unsigned long startMillis = millis();

  // Activate GPS every now and then to update our position
  if (updatesBeforeGpsUpdate == 0) {
    getPosition();
    updatesBeforeGpsUpdate = GPS_UPDATE_RATIO;
  }
  updatesBeforeGpsUpdate--;

  // Activate and read our sensors
  temperature = htu.readTemperature();
  humidity = htu.readHumidity();

  if (DEBUG)
    dumpData();

  // We can now send the data
  queueData();

  mjs_lmic_wait_for_txcomplete();

  // Schedule sleep
  unsigned long sleepDuration = UPDATE_INTERVAL - (millis() - startMillis);
  if (DEBUG) {
    Serial.print(F("Sleeping for "));
    Serial.print(sleepDuration);
    Serial.println(F("ms..."));
    Serial.flush();
  }
  doSleep(sleepDuration);
  if (DEBUG) {
    Serial.println(F("Woke up."));
  }
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

void dumpData() {
  if (gps_data.valid.location) {
    Serial.print(F("lat/lon: "));
    Serial.print(gps_data.latitudeL()/10000000.0, 6);
    Serial.print(F(","));
    Serial.println(gps_data.longitudeL()/10000000.0, 6);
  } else {
    Serial.println(F("No GPS (fix) found: check wiring"));
  }

  Serial.print(F("tmp/hum: "));
  Serial.print(temperature, 1);
  Serial.print(F(","));
  Serial.println(humidity, 1);
  Serial.flush();
}

boolean getPosition()
{
  memset(&gps_data, 0, sizeof(gps_data));

  digitalWrite(SW_GND_PIN, HIGH);
  if (DEBUG)
    Serial.println(F("Waiting for GPS..."));

  unsigned long startTime = millis();
  while (millis() - startTime < GPS_TIMEOUT && !gps_data.valid.location) {
    if (gps.available(gpsSerial))
      gps_data = gps.read();
  }
  digitalWrite(SW_GND_PIN, LOW);
}

void queueData() {
  uint8_t data[9];

  // pack geoposition
  uint32_t lat24 = int32_t((int64_t)gps_data.latitudeL() * 32768 / 10000000);
  data[0] = lat24 >> 16 & 0xFF;
  data[1] = lat24 >> 8 & 0xFF;
  data[2] = lat24 & 0xFF;

  uint32_t lng24 = int32_t((int64_t)gps_data.longitudeL() * 32768 / 10000000);
  data[3] = lng24 >> 16 & 0xFF;
  data[4] = lng24 >> 8 & 0xFF;
  data[5] = lng24 & 0xFF;

  // pack temperature and humidity
  int16_t tmp16 = (uint16_t)(temperature * 16) << 4;
  data[6] = tmp16 >> 8 & 0xFF;
  data[7] = tmp16 & 0xF0;

  int16_t hum16 = (uint16_t)(humidity * 16);
  data[7] |= hum16 >> 8 & 0x0F;
  data[8] = hum16 & 0xFF;

  // Prepare upstream data transmission at the next possible time.
  LMIC_setTxData2(LORA_PORT, data, sizeof(data), 0);
  if(DEBUG) Serial.println(F("Packet queued"));
}

