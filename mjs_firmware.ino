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
   - lmic (mjs-specific fork): https://github.com/meetjestad/arduino-lmic
 *******************************************************************************/

// include external libraries
#include <SPI.h>
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <SoftwareSerial.h>
#include <NMEAGPS.h>
#include <Adafruit_SleepyDog.h>
#include "RingBufCPP.h"
#include <avr/power.h>
#include <util/atomic.h>
#include "bitstream.h"

#if !defined(GPS_FIX_HDOP)
#error "Please enable GPS_FIX_HDOP in the NeoGPS library, in src/GPSfix.h"
#endif


// set run mode
boolean const DEBUG = true;

#include "mjs_lmic.h"

// setup GPS module
uint8_t const GPS_PIN = 8;
SoftwareSerial gpsSerial(GPS_PIN, GPS_PIN);
NMEAGPS gps;

// define various pins
uint8_t const SW_GND_PIN = 20;
uint8_t const LED_PIN = 21;

uint32_t const MIN_MEASUREMENT_TIME = 15000; // ms
float const MIN_MEASUREMENT_DISTANCE = 0.050; // km

uint8_t const LORA_PORT = 21;

struct measurement {
  uint16_t time;
  int32_t lat:24;
  int32_t lon:24;
  int16_t tmp:12;
  uint16_t hum:12;
};

void setup() {
  // when in debugging mode start serial connection
  if(DEBUG) {
    Serial.begin(9600);
    Serial.println(F("Start"));
  }

  // setup LoRa transceiver
  mjs_lmic_setup();

  // setup switched ground and power down connected peripherals (GPS module)
  pinMode(SW_GND_PIN, OUTPUT);
  digitalWrite(SW_GND_PIN, LOW);

  // Power up GPS
  digitalWrite(SW_GND_PIN, HIGH);

  // blink 'hello'
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  // start communication to sensors
  gpsSerial.begin(9600);
}

enum class State {
  WAITING_FOR_GPS,
  WAITING_FOR_FIX,
  TRANSMITTING,
  IDLE,
};

State state = State::WAITING_FOR_GPS;

void setState(State newState);
void setState(State newState) {
  state = newState;
  Serial.print(millis());
  Serial.print(F(": "));
  switch (newState) {
    case State::WAITING_FOR_GPS:
      Serial.println(F("WAITING_FOR_GPS"));
      break;
    case State::WAITING_FOR_FIX:
      Serial.println(F("WAITING_FOR_FIX"));
      break;
    case State::TRANSMITTING:
      Serial.print(F("TRANSMITTING ("));
      dumpDatarate(LMIC.datarate);
      Serial.println(F(")"));
      break;
    case State::IDLE:
      Serial.println(F("IDLE"));
      break;
  }
}

void showState() {
  bool led = 0;
  switch (state) {
    case State::WAITING_FOR_GPS:
      led = true;
      break;
    case State::WAITING_FOR_FIX:
      led = (millis() / 1024) % 2;
      break;
    case State::TRANSMITTING:
      led = (millis() / 64) % 2;
      break;
    case State::IDLE:
      led = 0;
      break;
  }
  digitalWrite(LED_BUILTIN, led);
}

void dumpDatarate(uint8_t datarate) {
  if (datarate == DR_SF7B) {
    Serial.print(F("SF7B"));
  } else if (datarate == DR_FSK) {
    Serial.print(F("FSK"));
  } else if (datarate <= DR_SF7) {
    Serial.print(F("SF"));
    Serial.print(12 - (datarate));
  } else {
    Serial.print(F("Invalid datarate"));
  }
}

static NeoGPS::Location_t last_measurement_loc;
static uint32_t last_measurement_time;
static bool last_was_sent = false;

void loop() {
  showState();
  // LMIC loop
  os_runloop_once();

  if (state == State::TRANSMITTING && (LMIC.opmode & OP_TXRXPEND) == 0)
    state = State::IDLE;

  // See if GPS data is ready
  if (gps.available(gpsSerial)) {
    gps_fix gps_data = gps.read();
    bool have_fix = gps_data.valid.location;
    if (state == State::WAITING_FOR_GPS)
      setState(State::WAITING_FOR_FIX);
    if (state == State::WAITING_FOR_FIX && have_fix)
      setState(State::IDLE);

    // Take a measurement whenever the minimum and time and distance
    // have passed, or immediately when we have a fix and there are no
    // measurements yet.
    if (have_fix) {
      uint32_t now = millis();
      uint32_t dtime = now - last_measurement_time;
      float dist = gps_data.location.DistanceKm(last_measurement_loc);

      if (!last_was_sent || dtime >= MIN_MEASUREMENT_TIME && dist >= MIN_MEASUREMENT_DISTANCE) {
        last_measurement_time = now;
        last_measurement_loc = gps_data.location;
        last_was_sent = false;

        digitalWrite(LED_BUILTIN, true);

        // If there is no TX in progress, queue the packet
        if ((LMIC.opmode & OP_TXRXPEND) == 0) {
          // Try sending the data. If TX is now pending, we'll be sending.
          // If not, we're waiting for duty cycle limits, so cancel
          // transmission to prevent sending an outdated packet later
          queuePacket(gps_data);

          if ((LMIC.opmode & OP_TXRXPEND) != 0) {
            setState(State::TRANSMITTING);
            last_was_sent = true;
          } else {
            Serial.println("No airtime");
            // No airtime available yet, clear packet again
            LMIC_clrTxData();
          }
        }
        digitalWrite(LED_BUILTIN, false);
      }
    }
  }
}
/*
void dumpData() {
  for (uint8_t i = 0; i < measurements.numElements(); ++i) {
    measurement *m = measurements.peek(i);

    Serial.print(i);
    Serial.print(F(") time: "));
    Serial.print(m->time);

    Serial.print(F(", pos: "));
    Serial.print(m->lat * 1000000.0 / 32768, 6);
    Serial.print(F(" / "));
    Serial.print(m->lon * 1000000.0 / 32768, 6);

    Serial.print(F(", tmp: "));
    Serial.print(m->tmp / 16.0, 3);

    Serial.print(F(", hum: "));
    Serial.println(m->hum / 16.0, 3);
  }

  Serial.flush();
}
*/

void queuePacket(const gps_fix& gps_data) {
  uint8_t data[MAX_LEN_PAYLOAD];
  BitStream packet(data, sizeof(data));

  int32_t lat = (int64_t)gps_data.latitudeL() * 32768 / 10000000;
  int32_t lon = (int64_t)gps_data.longitudeL() * 32768 / 10000000;
  int32_t alt = gps_data.altitude_cm();
  uint16_t hdop = gps_data.hdop;
  Serial.print(F("Lat: "));
  Serial.println((float)gps_data.latitudeL() / 10000000);
  Serial.print(F("Lon: "));
  Serial.println((float)gps_data.longitudeL() / 10000000);
  Serial.print(F("Alt: "));
  Serial.println((float)gps_data.altitude());
  Serial.print(F("HDOP: "));
  Serial.println((float)gps_data.hdop / 1000);

  packet.append(lat, 24);
  packet.append(lon, 24);
  packet.append(alt, 32);
  packet.append(hdop, 16);

  // Prepare upstream data transmission at the next possible time.
  LMIC_setTxData2(LORA_PORT, packet.data(), packet.byte_size(), 0);
}
