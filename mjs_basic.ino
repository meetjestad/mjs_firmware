/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman & Bas Peschier

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

 *******************************************************************************/


#define UPDATE_INTERVAL 15 * 60 * 1000
#define TX_INTERVAL 15 * 60 * 1000  // TODO, why?
#define DEBUG

#define GPS_PIN 8
#define GPS_VCC
#define LED_PIN


#include <SPI.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <TinyGPS++.h>
#include <Adafruit_SleepyDog.h>
#include "mjs_lmic.h"

SoftwareSerial gpsSerial(GPS_PIN, GPS_PIN);
TinyGPSPlus gps;
HTU21D htu;
unsigned long lastUpdateTime;
float temperature;
float humidity;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Starting"));

  // set PB6 aka sw_gndPin to OUTPUT mode
  // to be replaced with:
  // pinMode(sw_gndPin, OUTPUT);
  DDRB |= (1 << DDB6);

  // set PB6 aka sw_gndPin LOW
  // to be replaced with
  // digitalWrite(sw_gndPin, LOW);
  PORTB &= ~(1 << PORTB6);

  // digitalWrite(ledPin, HIGH);
  PORTB |= (1 << PORTB7);
  delay(500);
  // digitalWrite(ledPin, LOW);
  PORTB &= ~(1 << PORTB7);

  gpsSerial.begin(9600);
  htu.begin();

  mjs_lmic_setup();
}

void loop() {
#ifdef DEBUG
  debugLoop();
#else
  sleepLoop();
#endif
}

void debugLoop() {
  unsigned long currentTime = millis();
  boolean hasPosition = false;

  if (currentTime - lastUpdateTime > UPDATE_INTERVAL) {
    Serial.print(F("lng/lat: "));

    hasPosition = getPosition();

    if (hasPosition) {
      Serial.print(gps.location.lng(), 6);
      Serial.print(F(","));
      Serial.println(gps.location.lat(), 6);
    }
    else {
      Serial.println(F("No GPS detected: check wiring."));
    }
    Serial.flush();

    lastUpdateTime = currentTime;

    PORTB |= (1 << PORTB7);
    Serial.print(F("tmp/hum: "));

    temperature = htu.readTemperature();
    humidity = htu.readHumidity();

    Serial.print(temperature, 1);
    Serial.print(F(","));
    Serial.println(humidity, 1);
    Serial.flush();

    sendData();

    lastUpdateTime = currentTime;

    PORTB &= ~(1 << PORTB7);
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
  // digitalWrite(sw_gndPin, HIGH);
  PORTB |= (1 << PORTB6);
  unsigned long startTime = millis();
  while (millis() - startTime < 60000) {
    if (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isValid()) {
          // digitalWrite(sw_gndPin, LOW);
          PORTB &= ~(1 << PORTB6);
          return true;
        }
      }
    }
  }
  // digitalWrite(sw_gndPin, LOW);
  PORTB &= ~(1 << PORTB6);
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
  data[8] |= hum16 >> 8 & 0x0F;
  data[9] = hum16 & 0xFF;

  // pack other values: rain, light, moist
  // ...
  data[10] = 0;
  data[11] = 0;
  data[12] = 0;
  data[13] = 0;

  // send over LoRaWAN, using experiment identifier for port selection
  if (LMIC.opmode & OP_TXRXPEND) {
#ifdef DEBUG
    Serial.println(F("OP_TXRXPEND, not sending"));
#endif
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, data, sizeof(data) - 1, 0);
#ifdef DEBUG
    Serial.println(F("Packet queued"));
#endif
  }
  os_runloop_once();
}

