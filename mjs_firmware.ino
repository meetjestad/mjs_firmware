/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman, Bas Peschier, Harmen Zijp

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   In order to compile the following libraries need to be installed:
   - SparkFunHTU21D: https://github.com/sparkfun/SparkFun_HTU21D_Breakout_Arduino_Library
   - NeoGPS (mjs-specific fork): https://github.com/meetjestad/NeoGPS
   - Adafruit_SleepyDog: https://github.com/adafruit/Adafruit_SleepyDog
   - lmic (mjs-specific fork): https://github.com/meetjestad/arduino-lmic
 *******************************************************************************/

#define DEBUG true
#define WITH_LUX

// include external libraries
#include <SPI.h>
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <SoftwareSerial.h>
#include <NMEAGPS.h>
#include <Adafruit_SleepyDog.h>
#include <avr/power.h>
#include <util/atomic.h>
#include "bitstream.h"
#include "mjs_lmic.h"


// Firmware version to send. Should be incremented on release (i.e. when
// signficant changes happen, and/or a version is deployed onto
// production nodes). This value should correspond to a release tag.
const uint8_t FIRMWARE_VERSION = 1;

// This sets the ratio of the battery voltage divider attached to A0,
// below works for 100k to ground and 470k to the battery. A setting of
// 0.0 means not to measure the voltage. On first generation boards, this
// should only be enabled when the AREF pin of the microcontroller was
// disconnected.
float const BATTERY_DIVIDER_RATIO = 0.0;
//float const BATTERY_DIVIDER_RATIO = (100.0 + 470.0) / 100.0;

// These values define the sensitivity and calibration of the PAR / Lux 
// measurement.
// R12 Reference resistor for low light levels 
//  (nominal 100K in Platform Rev 2)
// R11 Reference shunt resistor for high ligh levels 
//  (nominal 10K in platform Rev 2)
// Value in Ohms
float const R12 = 100000.0;
// Value in Ohms
float const R11 = 10000.0;

// Reverse light current of the foto diode Ea at 1klx
// uA @ 1000lx  eg 8.9 nA/lx
float const light_current = 8.9;
// The Reverse dark current (max 30 nA ) is neglectable for our purpose

// Value in mV (nominal @ 25ºC, Vcc=3.3V)
float const reference_voltage = 1137.0;
// The temperature coefficient of the reference_voltage is neglected

// Global fixed point correction factor. ( now set to 1 )
float const correction_factor = 1.0;

// Latest battery voltage measurement
uint16_t vcc = 0;

// Define global lux conversion factors variables
float lx_conv_high;
float lx_conv_low;
long lux = 0;

// setup GPS module
uint8_t const GPS_PIN = 8;
SoftwareSerial gpsSerial(GPS_PIN, GPS_PIN);
NMEAGPS gps;

// setup temperature and humidity sensor
HTU21D htu;
float temperature;
float humidity;

// define various pins
uint8_t const SW_GND_PIN = 20;
uint8_t const LED_PIN = 21;
uint8_t const LUX_HIGH_PIN = 5;

// setup timing variables
uint32_t const UPDATE_INTERVAL = 900000;
uint32_t const GPS_TIMEOUT = 120000;
// Update GPS position after transmitting this many updates
uint16_t const GPS_UPDATE_RATIO = 24 * 4;

uint32_t lastUpdateTime = 0;
uint32_t updatesBeforeGpsUpdate = 0;
gps_fix gps_data;

#ifdef WITH_LUX
uint8_t const LORA_PORT = 12;
#else
uint8_t const LORA_PORT = 11;
#endif

void setup()
{
  // when in debugging mode start serial connection
  if (DEBUG)
  {
    Serial.begin(9600);
    Serial.println(F("Start"));
  }

  // setup LoRa transceiver
  mjs_lmic_setup();

  // setup switched ground and power down connected peripherals (GPS module)
  pinMode(SW_GND_PIN, OUTPUT);
  digitalWrite(SW_GND_PIN, LOW);

  // setup LUX_HIGH_PIN as input, PULL UP  Disabled
  // If the AD value of the exceed maximum range it should be set to 
  // output in low state.
  pinMode(LUX_HIGH_PIN, INPUT);

  // calculate the conversionfactors  
  setupLux();

  // blink 'hello'
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  // start communication to sensors
  htu.begin();
  gpsSerial.begin(9600);

  if (DEBUG)
  {
    vcc = readVcc();
    temperature = htu.readTemperature();
    humidity = htu.readHumidity();
    lux = readLux();
    Serial.print(F("Vcc: "));
    Serial.println(vcc);
    Serial.print(F("Temperature: "));
    Serial.println(temperature);
    Serial.print(F("Humidity: "));
    Serial.println(humidity);
    Serial.print(F("Lux [lx]: "));
    Serial.println(lux);
    Serial.flush();
  }
}

void loop()
{
  // We need to calculate how long we should sleep, so we need to know how long we were awake
  unsigned long startMillis = millis();

  // Activate GPS every now and then to update our position
  if (updatesBeforeGpsUpdate == 0)
  {
    getPosition();
    updatesBeforeGpsUpdate = GPS_UPDATE_RATIO;
    // Use the lowest datarate, to maximize range. This helps for
    // debugging, since range problems can be more easily distinguished
    // from other problems (lockups, downlink problems, etc).
    LMIC_setDrTxpow(DR_SF12, 14);
  }
  else
  {
    LMIC_setDrTxpow(DR_SF9, 14);
  }
  updatesBeforeGpsUpdate--;

  // Activate and read our sensors
  vcc = readVcc();
  temperature = htu.readTemperature();
  humidity = htu.readHumidity();
  lux = readLux();

  if (DEBUG)
  {
    dumpData();
  }

  // Work around a race condition in LMIC, that is greatly amplified
  // if we sleep without calling runloop and then queue data
  // See https://github.com/lmic-lib/lmic/issues/3
  os_runloop_once();

  // We can now send the data
  queueData();
  mjs_lmic_wait_for_txcomplete();

  // Schedule sleep
  unsigned long msPast = millis() - startMillis;
  unsigned long sleepDuration = UPDATE_INTERVAL;
  if (msPast < sleepDuration)
  {
    sleepDuration -= msPast;
  }
  else
  {
    sleepDuration = 0;
  }

  if (DEBUG)
  {
    Serial.print(F("Sleeping for "));
    Serial.print(sleepDuration);
    Serial.println(F("ms..."));
    Serial.flush();
  }
  doSleep(sleepDuration);
  if (DEBUG)
  {
    Serial.println(F("Woke up."));
  }
}

void doSleep(uint32_t time)
{
  ADCSRA &= ~(1 << ADEN);
  power_adc_disable();

  while (time > 0)
  {
    uint16_t slept;
    if (time < 8000)
    {
      slept = Watchdog.sleep(time);
    }
    else
    {
      slept = Watchdog.sleep(8000);
    }

    // Update the millis() and micros() counters, so duty cycle
    // calculations remain correct. This is a hack, fiddling with
    // Arduino's internal variables, which is needed until
    // https://github.com/arduino/Arduino/issues/5087 is fixed.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      extern volatile unsigned long timer0_millis;
      extern volatile unsigned long timer0_overflow_count;
      timer0_millis += slept;
      // timer0 uses a /64 prescaler and overflows every 256 timer ticks
      timer0_overflow_count +=
        microsecondsToClockCycles((uint32_t) slept * 1000) / (64 * 256);
    }

    if (slept >= time)
    {
      break;
    }
    time -= slept;
  }

  power_adc_enable();
  ADCSRA |= (1 << ADEN);
}

void dumpData()
{
  if (gps_data.valid.location && gps_data.valid.status
    && gps_data.status >= gps_fix::STATUS_STD)
  {
    Serial.print(F("lat/lon: "));
    Serial.print(gps_data.latitudeL() / 10000000.0, 6);
    Serial.print(F(","));
    Serial.println(gps_data.longitudeL() / 10000000.0, 6);
  }
  else
  {
    Serial.println(F("No GPS fix"));
  }

  Serial.print(F("vcc="));
  Serial.print(vcc, 1);
  Serial.print(F(", temp="));
  Serial.print(temperature, 1);
  Serial.print(F(", hum="));
  Serial.print(humidity, 1);
  Serial.print(F(", lux="));
  Serial.println(lux);
  Serial.flush();
}

void getPosition()
{
  memset(&gps_data, 0, sizeof(gps_data));
  gps.statistics.init();

  digitalWrite(SW_GND_PIN, HIGH);
  if (DEBUG)
  {
    Serial.println(F("Waiting for GPS..."));
  }

  unsigned long startTime = millis();
  int valid = 0;
  while (millis() - startTime < GPS_TIMEOUT && valid < 10)
  {
    if (gps.available(gpsSerial))
    {
      gps_data = gps.read();
      if (gps_data.valid.location && gps_data.valid.status
          && gps_data.status >= gps_fix::STATUS_STD)
      {
        valid++;
      }
      if (gps_data.valid.satellites)
      {
        Serial.print(F("Satellites: "));
        Serial.println(gps_data.satellites);
      }
    }
  }
  digitalWrite(SW_GND_PIN, LOW);

  if (gps.statistics.ok == 0)
  {
    Serial.println(F("No GPS data received, check wiring"));
  }
}

void queueData()
{
  uint8_t length = (BATTERY_DIVIDER_RATIO ? 12 : 11);
  uint8_t data[length];
  BitStream packet(data, sizeof(data));

  packet.append(FIRMWARE_VERSION, 8);
  if (gps_data.valid.location && gps_data.valid.status
    && gps_data.status >= gps_fix::STATUS_STD)
  {
    // pack geoposition
    int32_t lat24 =
      int32_t ((int64_t) gps_data.latitudeL() * 32768 / 10000000);
    packet.append(lat24, 24);

    int32_t lng24 =
      int32_t ((int64_t) gps_data.longitudeL() * 32768 / 10000000);
    packet.append(lng24, 24);
  }
  else
  {
    // Append zeroes if the location is unknown (but do not use the
    // lat/lon from gps_data, since they might still contain old
    // values).
    packet.append(0, 24);
    packet.append(0, 24);
  }

  // pack temperature and humidity
  int16_t tmp16 = temperature * 16;
  packet.append(tmp16, 12);

  int16_t hum16 = humidity * 16;
  packet.append(hum16, 12);

  // Encoded in units of 10mv, starting at 1V
  uint8_t v = (vcc - 1000) / 10;
  packet.append(v, 8);

#ifdef WITH_LUX
  packet.append(lux, 16);
#endif

  if (BATTERY_DIVIDER_RATIO)
  {
    analogReference(INTERNAL);
    uint16_t reading = analogRead(A0);
    // Encoded in units of 20mv
    uint8_t batt =
      (uint32_t) (50 * BATTERY_DIVIDER_RATIO * 1.1) * reading / 1023;
    // Shift down, zero means 1V now
    if (batt >= 50)
    {
      packet.append(batt - 50, 8);
    }
  }

  // Prepare upstream data transmission at the next possible time.
  LMIC_setTxData2(LORA_PORT, packet.data(), packet.byte_size(), 0);
  if (DEBUG)
  {
    Serial.println(F("Packet queued"));
  }
}

uint16_t readVcc()
{
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
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH;  // unlocks both

  uint16_t result = (high << 8) | low;

  result = 1125300L / result;  // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result;
}

void setupLux()
{
  // Caclulate conversion factors from constants
  // Value in Ohms
  float ref_low = R12;
  // Calculate parallel resistance
  float ref_high = (R12 * R11) / (R12 + R11);
  lx_conv_high = (reference_voltage * 1.0E6 * correction_factor)
    / (ref_high * light_current * 1024.0);
  lx_conv_low = (reference_voltage * 1.0E6 * correction_factor)
    / (ref_low * light_current * 1024.0);
}

long readLux()
{
  long result = 0;

  // Set the Reference Resistor to 100K
  pinMode(LUX_HIGH_PIN, INPUT);
  // Read the value of Analog input 2 against the internal reference 
  analogReference(INTERNAL);
  uint16_t read_low = analogRead(A2);
  // Check if read_low has an overflow
  if (read_low < 1000)
  {
    result = long(lx_conv_low * read_low);
    goto finish;
  }

  {
    // Set the Reference Resistor to 10K parallel with 100K = 9.091K
    pinMode(LUX_HIGH_PIN, OUTPUT);
    digitalWrite(LUX_HIGH_PIN, LOW);
    // Read the value of Analog input 2 against the internal reference 
    analogReference(INTERNAL);
    uint16_t read_high = analogRead(A2);
    // Check if read_high has an overflow
    if (read_high < 1000)
    {
      result = long(lx_conv_high * read_high);
      goto finish;
    }
  }

  {
    // Set the Reference Resistor to 10K parallel with 100K = 9.091K
    pinMode(LUX_HIGH_PIN, OUTPUT);
    digitalWrite(LUX_HIGH_PIN, LOW);
    // Read the value of Analog input 2 against the battery voltage
    analogReference(DEFAULT);
    uint16_t read_bat = analogRead(A2);
    float conv_bat = (lx_conv_high * vcc) / reference_voltage;
    result = long(conv_bat * read_bat);
  }
finish:
  // Set the Reference Resistor to 100K to draw the least current
  pinMode(LUX_HIGH_PIN, INPUT);
  if (DEBUG)
  {
    Serial.print(F("Lux_reading : "));
    Serial.print(result);
    Serial.println(F(" lx"));
  }
  return result;
}


