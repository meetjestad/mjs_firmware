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

// set run mode
boolean const DEBUG = true;

// This sets the ratio of the battery voltage divider attached to A0,
// below works for 100k to ground and 470k to the battery. A setting of
// 0.0 means not to measure the voltage. On first generation boards, this
// should only be enabled when the AREF pin of the microcontroller was
// disconnected.
float const BATTERY_DIVIDER_RATIO = 0.0;
//float const BATTERY_DIVIDER_RATIO = (100.0 + 470.0) / 100.0;

#include "mjs_lmic.h"

// setup GPS module
uint8_t const GPS_PIN = 8;
SoftwareSerial gpsSerial(GPS_PIN, GPS_PIN);
NMEAGPS gps;

// setup temperature and humidity sensor
HTU21D htu;

// define various pins
uint8_t const SW_GND_PIN = 20;
uint8_t const LED_PIN = 21;

uint32_t const MIN_MEASUREMENT_TIME = 15000; // ms
float const MIN_MEASUREMENT_DISTANCE = 0.050; // km

uint8_t const LORA_PORT = 20;

struct measurement {
  uint16_t time;
  int32_t lat:24;
  int32_t lon:24;
  int16_t tmp:12;
  uint16_t hum:12;
};

RingBufCPP<measurement, 10> measurements;

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
  htu.begin();
  gpsSerial.begin(9600);

  if (DEBUG) {
    float temperature = htu.readTemperature();
    float humidity = htu.readHumidity();
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
  }
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

      if (measurements.isEmpty() || dtime >= MIN_MEASUREMENT_TIME && dist >= MIN_MEASUREMENT_DISTANCE) {
        addMeasurement(gps_data);
        last_measurement_time = now;
        last_measurement_loc = gps_data.location;
        last_was_sent = false;

        dumpData();
      }

      if (!last_was_sent) {
        digitalWrite(LED_BUILTIN, true);

        // If there is no TX in progress, queue the packet
        if ((LMIC.opmode & OP_TXRXPEND) == 0) {
          // Try sending the data. If TX is now pending, we'll be sending.
          // If not, we're waiting for duty cycle limits, so cancel
          // transmission to prevent sending an outdated packet later
          queueData();

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

void doSleep(uint32_t time) {
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();

  while (time > 0) {
    uint16_t slept;
    if (time < 8000)
      slept = Watchdog.sleep(time);
    else
      slept = Watchdog.sleep(8000);

    // Update the millis() and micros() counters, so duty cycle
    // calculations remain correct. This is a hack, fiddling with
    // Arduino's internal variables, which is needed until
    // https://github.com/arduino/Arduino/issues/5087 is fixed.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      extern volatile unsigned long timer0_millis;
      extern volatile unsigned long timer0_overflow_count;
      timer0_millis += slept;
      // timer0 uses a /64 prescaler and overflows every 256 timer ticks
      timer0_overflow_count += microsecondsToClockCycles((uint32_t)slept * 1000) / (64 * 256);
    }

    if (slept >= time)
      break;
    time -= slept;
  }

  power_adc_enable();
  ADCSRA |= (1 << ADEN);
}

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

/*
void getPosition()
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
*/

void addMeasurement(const gps_fix& gps_data) {
  measurement m;

  // Time in multiples of 1024ms. This divides by a power of 2 to
  // prevent a jump when millis() overflows.
  m.time = millis() / 1024;
  // Position in 32768ths of a degree. gps_data contains in millionths
  // of a degree, so scale using an int64 to prevent overflow.
  m.lat = int32_t((int64_t)gps_data.latitudeL() * 32768 / 10000000);
  m.lon = int32_t((int64_t)gps_data.longitudeL() * 32768 / 10000000);
  // Read sensors
  m.tmp = (uint16_t)(htu.readTemperature() * 16);
  m.hum = (uint16_t)(htu.readHumidity() * 16);

  // If the buffer is full, pop the oldest value to make room
  if (measurements.isFull())
    measurements.pop();

  // Prepend new values to the start of the buffer
  measurements.prepend(m);
}

uint8_t needed_bits_signed(int32_t v) {
  if (v >= 0)
    return needed_bits_unsigned(v) + 1;
  else
    return needed_bits_unsigned(~v) + 1;
}

uint8_t needed_bits_unsigned(uint32_t v) {
  uint8_t bits = 0;
  while (v) {
    bits++;
    v >>= 1;
  }
  return bits;
}

void queueData() {
  uint8_t data[MAX_LEN_PAYLOAD];
  BitStream packet(data, sizeof(data));

  uint8_t max_dtime_bits = 1, max_dpos_bits = 1, max_dsensor_bits = 1;

  // Using 4 bits for the diff sizes allows up to 16 bits for the diffs
  // themselves. This supports up to 18 hours, 100km distance, 2048 °C
  // and 2048% RH between measurements, which should be plenty. One step
  // down saves only 3 bits on the full packet, but limits to 400m
  // between measurements, which is easily obtainable using a car.
  const uint8_t diff_size_bits = 4;
  // How big are the full (not diffed) values. Time is never sent in
  // full, since the first measurement has an implicit age of 0.
  const uint8_t pos_bits = 24;
  const uint8_t tmp_bits = 12;
  const uint8_t hum_bits = 12;
  const uint8_t vcc_bits = 8;

  // See how much room is available for the incremental measurements
  size_t free_bits = packet.free_bits();
  free_bits -= diff_size_bits * 3; // Diff header
  free_bits -= pos_bits * 2 + tmp_bits + hum_bits + vcc_bits; // First measurement

  // Loop over each measurement and find out how many bits are needed to
  // store each diff.
  measurement *cur = measurements.peek(0);
  for (uint8_t i = 1; i < measurements.numElements(); ++i) {
    measurement *prev = cur;
    cur = measurements.peek(i);

    uint8_t dtime_bits = needed_bits_unsigned(prev->time - cur->time);
    uint8_t dlat_bits = needed_bits_signed(prev->lat - cur->lat);
    uint8_t dlon_bits = needed_bits_signed(prev->lon - cur->lon);
    uint8_t dtmp_bits = needed_bits_signed(prev->tmp - cur->tmp);
    uint8_t dhum_bits = needed_bits_signed(prev->hum - cur->hum);

    uint8_t new_max_dtime_bits = max(max_dtime_bits, dtime_bits);
    uint8_t new_max_dpos_bits = max(max_dpos_bits, max(dlat_bits, dlon_bits));
    uint8_t new_max_dsensor_bits = max(max_dsensor_bits, max(dtmp_bits, dhum_bits));

    // Number of bits for each incremental measurement
    size_t measurement_bits = new_max_dtime_bits + 2 * new_max_dpos_bits + 2 * new_max_dsensor_bits;

    // Check if adding this measurement still fits
    if ((i - 1) * measurement_bits > free_bits) {
      // When the packet is full, clear out this measurement and
      // subsequent ones, no point in keeping them around.
      while (measurements.numElements() > i)
        measurements.pop();
      break;
    }

    max_dtime_bits = new_max_dtime_bits;
    max_dpos_bits = new_max_dpos_bits;
    max_dsensor_bits = new_max_dsensor_bits;
  }

  const uintmax_t max_diff_size = (1 << diff_size_bits) - 1;
  if (max_dpos_bits > max_diff_size
      || max_dtime_bits > max_diff_size
      || max_dsensor_bits > max_diff_size) {
    // If the diff is every not representable, clear all but the last
    // measurement and send just that, rather than sending a corrupt
    // packet. It should be pretty impossible to achieve, this, though.
    while (measurements.numElements() > 1)
      measurements.pop();
  }

  // Build the packet
  // Start with the bit sizes for all the diffs in this packet
  packet.append(max_dtime_bits, diff_size_bits);
  packet.append(max_dpos_bits, diff_size_bits);
  packet.append(max_dsensor_bits, diff_size_bits);

  // Then the most recent measurement
  cur = measurements.peek(0);
  packet.append(cur->lat, pos_bits);
  packet.append(cur->lon, pos_bits);
  packet.append(cur->tmp, tmp_bits);
  packet.append(cur->hum, hum_bits);
  // Encoded in units of 10mv, starting at 1V
  packet.append((readVcc()-1000)/10, vcc_bits);

  // Finally, add all older measurements. No need to check if this fits,
  // since any extra measurements are already cleared above.
  for (uint8_t i = 1; i < measurements.numElements(); ++i) {
    measurement *prev = cur;
    cur = measurements.peek(i);

    packet.append(prev->time - cur->time, max_dtime_bits);
    packet.append(prev->lat - cur->lat, max_dpos_bits);
    packet.append(prev->lon - cur->lon, max_dpos_bits);
    packet.append(prev->tmp - cur->tmp, max_dsensor_bits);
    packet.append(prev->hum - cur->hum, max_dsensor_bits);
  }

  // Prepare upstream data transmission at the next possible time.
  LMIC_setTxData2(LORA_PORT, packet.data(), packet.byte_size(), 0);
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  // Wait a bit before measuring to stabilize the reference (or
  // something).  The datasheet suggests that the first reading after
  // changing the reference is inaccurate, but just doing a dummy read
  // still gives unstable values, but this delay helps. For some reason
  // analogRead (which can also change the reference) does not need
  // this.
  delay(2);

  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  uint16_t result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
