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

// include external libraries
#include <SPI.h>
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <sps30.h>
#if defined(ARDUINO_MJS_V1)
  #include <SoftwareSerial.h>
  #include <Adafruit_SleepyDog.h>
  #include <avr/power.h>
  #include <util/atomic.h>
#elif defined(ARDUINO_ARCH_STM32L0)
  #include "STM32L0.h"
#endif
#include <NMEAGPS.h>

#define DEBUG true
#include "bitstream.h"
#include "mjs_lmic.h"


// Firmware version to send. Should be incremented on release (i.e. when
// signficant changes happen, and/or a version is deployed onto
// production nodes). This value should correspond to a release tag.
// For untagged/experimental versions, use 255.
const uint8_t FIRMWARE_VERSION = 255;

#if defined(ARDUINO_MJS_V1)

// This sets the ratio of the battery voltage divider attached to A0,
// below works for 100k to ground and 470k to the battery. A setting of
// 0.0 means not to measure the voltage. On first generation boards, this
// should only be enabled when the AREF pin of the microcontroller was
// disconnected.
float const BATTERY_DIVIDER_RATIO = 0.0;
//float const BATTERY_DIVIDER_RATIO = (100.0 + 470.0) / 100.0;
uint8_t const BATTERY_DIVIDER_PIN = A0;
auto const BATTERY_DIVIDER_REF = INTERNAL;
uint16_t const BATTERY_DIVIDER_REF_MV = 1137;

// Value in mV (nominal @ 25ºC, Vcc=3.3V)
// The temperature coefficient of the reference_voltage is neglected
float const reference_voltage_internal = 1137.0;

// setup GPS module
uint8_t const GPS_PIN = 8;
uint8_t const GPS_ENABLE_PIN = 20;

// define various pins
uint8_t const LED_PIN = 21;
uint8_t const LED_ON = HIGH;
uint8_t const LUX_HIGH_PIN = 5;
uint8_t const LUX_PIN = A2;
#define GPS_USE_SOFTWARE_SERIAL

#elif defined(ARDUINO_MJS2020_PROTO2)
float const BATTERY_DIVIDER_RATIO = (1.0 + 1.0) / 1.0;

uint8_t const BATTERY_DIVIDER_PIN = A0;
auto const BATTERY_DIVIDER_REF = AR_DEFAULT;
uint16_t const BATTERY_DIVIDER_REF_MV = 3000;

#define GPS_SERIAL Serial2
uint8_t const GPS_ENABLE_PIN = PIN_ENABLE_3V_GPS;

// define various pins
uint8_t const LED_PIN = LED_BUILTIN;
uint8_t const LED_ON = LOW;
uint8_t const LUX_HIGH_PIN = 0; // TODO: PA6 not mapped?
uint8_t const LUX_PIN = 0; // TODO: PC3 not mapped?
#else
  #error "Unknown board"
#endif

// Enable this define when a light sensor is attached
// TODO: Support on MJS2020
//#define WITH_LUX

// Enable this define when an SPS30 is attached over I²C
#define WITH_SPS30_I2C

// These values define the sensitivity and calibration of the PAR / Lux
// measurement.
// R12 Reference resistor for low light levels
//  (nominal 100K in Platform Rev 2)
// R11 Reference shunt resistor for high ligh levels
//  (nominal 10K in platform Rev 2)
// Value in Ohms
float const R12 = 100000.0; // This is really R16 on MJS2020
// Value in Ohms
float const R11 = 10000.0; // This is really R15 on MJS2020

// Reverse light current of the foto diode Ea at 1klx
// uA @ 1000lx  eg 8.9 nA/lx
// The Reverse dark current (max 30 nA ) is neglectable for our purpose
float const light_current = 8.9;

// R11 and R12 in parallel
float const R11_R12 = (R12 * R11) / (R12 + R11);
float const lx_conv_high = 1.0E6 / (R11_R12 * light_current * 1024.0);
float const lx_conv_low = 1.0E6 / (R12 * light_current * 1024.0);

// Sensor object
HTU21D htu;

// Most recently read values
float temperature;
float humidity;
uint16_t vcc = 0;
#ifdef WITH_LUX
uint32_t lux = 0;
#endif
int32_t lat24 = 0;
int32_t lng24 = 0;

#if defined(WITH_SPS30_I2C)
struct sps30_measurement sps30_data;
#endif

// setup timing variables
uint32_t const UPDATE_INTERVAL = 900000;
uint32_t const GPS_TIMEOUT = 120000;
// Update GPS position after transmitting this many updates
uint16_t const GPS_UPDATE_RATIO = 24*4;

// When sending extra data, use this many bits to specify the size
// (allows up to 32-bit values)
uint8_t const EXTRA_SIZE_BITS = 5;

enum {
  FLAG_WITH_LUX = (1 << 7),
  FLAG_WITH_PM = (1 << 6),
  FLAG_WITH_BATTERY = (1 << 5),
  // bits 4:1 reserved for future additions
  FLAG_WITH_EXTRA = (1 << 0),
};

uint32_t lastUpdateTime = 0;
uint32_t updatesBeforeGpsUpdate = 0;
gps_fix gps_data;

uint8_t const LORA_PORT = 13;

void setup() {
  // when in debugging mode start serial connection
  if(DEBUG) {
    Serial.begin(9600);
    unsigned long start = millis();
    while (!Serial && millis() - start < 5000) /* wait */;
    // Wait a bit more, otherwise some clients (e.g. minicom on Linux) seem to miss the first bit of output...
    delay(250);

    if (!Serial) {
      // If serial was not opened at startup, close it again to prevent
      // locking up because writes will start blocking when the buffer
      // fills up.
      Serial.end();
    }

    Serial.println(F("Start"));
  }

  // setup LoRa transceiver
  mjs_lmic_setup();

  // setup switched ground and power down connected peripherals (GPS module)
  pinMode(GPS_ENABLE_PIN , OUTPUT);
  digitalWrite(GPS_ENABLE_PIN, LOW);

  #if defined(ARDUINO_MJS2020_PROTO2)
  pinMode(PIN_ENABLE_5V, OUTPUT);
  digitalWrite(PIN_ENABLE_5V, LOW);

  pinMode(PIN_ENABLE_3V_SENS, OUTPUT);
  digitalWrite(PIN_ENABLE_3V_SENS, LOW);
  #endif

  #ifdef WITH_LUX
  // This pin can be used in OUTPUT LOW mode to add an extra pulldown
  // resistor, or in INPUT mode to keep it disconnected
  pinMode(LUX_HIGH_PIN, INPUT);
  pinMode(LUX_PIN, INPUT);
  #endif

  // blink 'hello'
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON);
  delay(500);
  digitalWrite(LED_PIN, !LED_ON);

  // start communication to sensors
  htu.begin();

  #if defined(WITH_SPS30_I2C)
    sensirion_i2c_init();
  #endif


  if (DEBUG) {
    temperature = htu.readTemperature();
    humidity = htu.readHumidity();
    vcc = readVcc();
#ifdef WITH_LUX
    lux = readLux();
#endif
#if defined(WITH_SPS30_I2C)
    digitalWrite(PIN_ENABLE_5V, HIGH);
    delay(500);
    char sps30_serial[SPS30_MAX_SERIAL_LEN];
    int16_t ret = sps30_get_serial(sps30_serial);
    if (ret < 0) {
      Serial.print("Error reading SPS30 serial: ");
      Serial.println(ret);
    }
    // Leave 5V on, readSps30 turns it off
    sps30_data = readSps30();
#endif // defined(WITH_SPS30_I2C)
    Serial.print(F("Temperature: "));
    Serial.println(temperature);
    Serial.print(F("Humidity: "));
    Serial.println(humidity);
    Serial.print(F("Vcc: "));
    Serial.println(vcc);
#ifdef WITH_LUX
    Serial.print(F("Lux: "));
    Serial.println(lux);
#endif // WITH_LUX
#if defined(WITH_SPS30_I2C)
    Serial.print("SPS30 serial: ");
    Serial.println(sps30_serial);
    Serial.print("PM  1.0: ");
    Serial.println(sps30_data.mc_1p0);
    Serial.print("PM  2.5: ");
    Serial.println(sps30_data.mc_2p5);
    Serial.print("PM  4.0: ");
    Serial.println(sps30_data.mc_4p0);
    Serial.print("PM 10.0: ");
    Serial.println(sps30_data.mc_10p0);
    Serial.print("NC  0.5: ");
    Serial.println(sps30_data.nc_0p5);
    Serial.print("NC  1.0: ");
    Serial.println(sps30_data.nc_1p0);
    Serial.print("NC  2.5: ");
    Serial.println(sps30_data.nc_2p5);
    Serial.print("NC  4.0: ");
    Serial.println(sps30_data.nc_4p0);
    Serial.print("NC 10.0: ");
    Serial.println(sps30_data.nc_10p0);

    Serial.print("Typical partical size: ");
    Serial.println(sps30_data.typical_particle_size);
#endif // defined(WITH_SPS30_I2C)

    if (BATTERY_DIVIDER_RATIO) {
      Serial.print(F("Battery Divider Ratio: "));
      Serial.println(BATTERY_DIVIDER_RATIO);
    }
    Serial.flush();
  }

  // Start join
  LMIC_startJoining();

  // Wait for join to complete
  // TODO: Sleep between join attempts
  while ((LMIC.opmode & (OP_JOINING)))
    os_runloop_once();
}

void loop() {
  // We need to calculate how long we should sleep, so we need to know how long we were awake
  unsigned long startMillis = millis();

  // Activate GPS every now and then to update our position
  if (updatesBeforeGpsUpdate == 0) {
    getPosition();
    updatesBeforeGpsUpdate = GPS_UPDATE_RATIO;
    // Use the lowest datarate, to maximize range. This helps for
    // debugging, since range problems can be more easily distinguished
    // from other problems (lockups, downlink problems, etc).
    LMIC_setDrTxpow(DR_SF12, 14);
  } else {
    LMIC_setDrTxpow(DR_SF9, 14);
  }
  updatesBeforeGpsUpdate--;

  // Activate and read our sensors
  temperature = htu.readTemperature();
  humidity = htu.readHumidity();
  vcc = readVcc();
#ifdef WITH_LUX
  lux = readLux();
#endif // WITH_LUX
#if defined(WITH_SPS30_I2C)
  sps30_data = readSps30();
#endif // defined(WITH_SPS30_I2C)

  if (DEBUG)
    dumpData();

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
    sleepDuration -= msPast;
  else
    sleepDuration = 0;

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
  #if defined(__AVR__)
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
  #else
  // No need to update the millis counter, STM32L0 uses the RTC for
  // millis and keeps it running during sleep for wakeup.
  if (time)
    STM32L0.sleep(time);
  #endif
}

void dumpData() {
  if (gps_data.valid.location && gps_data.valid.status && gps_data.status >= gps_fix::STATUS_STD) {
    Serial.print(F("lat/lon: "));
    Serial.print(gps_data.latitudeL()/10000000.0, 6);
    Serial.print(F(","));
    Serial.println(gps_data.longitudeL()/10000000.0, 6);
  } else {
    Serial.println(F("No GPS fix"));
  }

  Serial.print(F("temp="));
  Serial.print(temperature, 1);
  Serial.print(F(", hum="));
  Serial.print(humidity, 1);
  Serial.print(F(", vcc="));
  Serial.print(vcc, 1);
#ifdef WITH_LUX
  Serial.print(F(", lux="));
  Serial.print(lux);
#endif // WITH_LUX
#if defined(WITH_SPS30_I2C)
  Serial.print(", pm1.0=");
  Serial.print(sps30_data.mc_1p0);
  Serial.print(", pm2.5=");
  Serial.print(sps30_data.mc_2p5);
  Serial.print(", pm4.0=");
  Serial.print(sps30_data.mc_4p0);
  Serial.print(", pm10.0=");
  Serial.print(sps30_data.mc_10p0);
  Serial.print(", nc0.5=");
  Serial.print(sps30_data.nc_0p5);
  Serial.print(", nc1.0=");
  Serial.print(sps30_data.nc_1p0);
  Serial.print(", nc2.5=");
  Serial.print(sps30_data.nc_2p5);
  Serial.print(", nc4.0=");
  Serial.print(sps30_data.nc_4p0);
  Serial.print(", nc10.0=");
  Serial.print(sps30_data.nc_10p0);
  Serial.print(", typ_size=");
  Serial.print(sps30_data.typical_particle_size);
#endif // defined(WITH_SPS30_I2C)
  Serial.println();
  Serial.flush();
}

void getPosition()
{
  #if defined(GPS_USE_SOFTWARE_SERIAL)
  // Setup GPS
  SoftwareSerial GPS_SERIAL(GPS_PIN, GPS_PIN);
  #endif

  NMEAGPS gps;

  GPS_SERIAL.begin(9600);
  memset(&gps_data, 0, sizeof(gps_data));
  gps.reset();
  gps.statistics.init();

  digitalWrite(GPS_ENABLE_PIN, HIGH);

  // Empty serial input buffer, so only new characters are processed
  while(Serial.read() >= 0) /* nothing */;

  if (DEBUG)
    Serial.println(F("Waiting for GPS, send 's' to skip..."));

  unsigned long startTime = millis();
  uint8_t valid = 0;
  while (millis() - startTime < GPS_TIMEOUT && valid < 10) {
    if (gps.available(GPS_SERIAL)) {
      gps_data = gps.read();
      if (gps_data.valid.location && gps_data.valid.status && gps_data.status >= gps_fix::STATUS_STD) {
        valid++;
        lat24 = int32_t((int64_t)gps_data.latitudeL() * 32768 / 10000000);
        lng24 = int32_t((int64_t)gps_data.longitudeL() * 32768 / 10000000);
      } else {
        lat24 = 0;
        lng24 = 0;
      }
      if (gps_data.valid.satellites) {
        Serial.print(F("Satellites: "));
        Serial.println(gps_data.satellites);
      }
    }
    if (DEBUG && tolower(Serial.read()) == 's')
      break;
  }
  digitalWrite(GPS_ENABLE_PIN, LOW);

  if (gps.statistics.ok == 0)
    Serial.println(F("No GPS data received, check wiring"));

  GPS_SERIAL.end();
}

void queueData() {
  uint8_t length = 12;
  uint8_t flags = 0;

  if (BATTERY_DIVIDER_RATIO) {
    flags |= FLAG_WITH_BATTERY;
    length += 1;
  }

#ifdef WITH_LUX
  flags |= FLAG_WITH_LUX;
  length += 2;
#endif
#ifdef WITH_SPS30_I2C
  flags |= FLAG_WITH_PM;
  length += 4;
#endif

#ifdef WITH_SPS30_I2C
  // Add *all* PM data as extra fields. This defines the extra size
  // needed, actual values are defined below.
  // For simplicity, use 16 bits for all extra fields.
  // This allows up to 6553.5 μg/m³ (datasheet says up to 1000), up to
  // 6553.5 #/cm³ (datasheet says up to 3000) and up to
  // 65535 nm typical particle size (datasheet suggests up to 10).
  const uint8_t EXTRA_FIELD_BITS = 15;
  const uint8_t extra_bits = 9*(EXTRA_SIZE_BITS+EXTRA_FIELD_BITS);
  length += (extra_bits + 7)/8;
  flags |= FLAG_WITH_EXTRA;
#endif // WITH_SPS30_I2C

  uint8_t data[length];
  BitStream packet(data, sizeof(data));

  packet.append(flags, 8);

  packet.append(FIRMWARE_VERSION, 8);

  packet.append(lat24, 24);
  packet.append(lng24, 24);


  // pack temperature and humidity
  int16_t tmp16 = temperature * 16;
  packet.append(tmp16, 12);

  int16_t hum16 = humidity * 16;
  packet.append(hum16, 12);

  // Encoded in units of 10mv, starting at 1V
  uint8_t vcc8 = (vcc - 1000) / 10;
  packet.append(vcc8, 8);

#ifdef WITH_LUX
  // Chop off 2 bits to allow up to 256k lux (maximum solar power should be around 128k)
  packet.append(lux >> 2, 16);
#endif
#ifdef WITH_SPS30_I2C
  packet.append(sps30_data.mc_2p5, 16);
  packet.append(sps30_data.mc_10p0, 16);
#endif

  if (BATTERY_DIVIDER_RATIO) {
    analogReference(BATTERY_DIVIDER_REF);
    uint16_t reading = analogRead(BATTERY_DIVIDER_PIN);
    // Encoded in units of 20mv
    uint8_t batt = (uint32_t)(reading*BATTERY_DIVIDER_RATIO*BATTERY_DIVIDER_REF_MV)/(20*1023);
    // Shift down, zero means 1V now
    if (batt >= 50)
      packet.append(batt - 50, 8);
    else
      packet.append(0, 8);
  }

#ifdef WITH_SPS30_I2C
  // Append extra fields. For each field, first add the size of the
  // field (minus on to allow a size of 1-32 rather than 0-31).
  packet.append(EXTRA_FIELD_BITS-1, EXTRA_SIZE_BITS);
  packet.append(sps30_data.mc_1p0 * 10 + 0.5, EXTRA_FIELD_BITS);
  packet.append(EXTRA_FIELD_BITS-1, EXTRA_SIZE_BITS);
  packet.append(sps30_data.mc_2p5 * 10 + 0.5, EXTRA_FIELD_BITS);
  packet.append(EXTRA_FIELD_BITS-1, EXTRA_SIZE_BITS);
  packet.append(sps30_data.mc_4p0 * 10 + 0.5, EXTRA_FIELD_BITS);
  packet.append(EXTRA_FIELD_BITS-1, EXTRA_SIZE_BITS);
  packet.append(sps30_data.mc_10p0 * 10 + 0.5, EXTRA_FIELD_BITS);

  packet.append(EXTRA_FIELD_BITS-1, EXTRA_SIZE_BITS);
  packet.append(sps30_data.nc_1p0 * 10 + 0.5, EXTRA_FIELD_BITS);
  packet.append(EXTRA_FIELD_BITS-1, EXTRA_SIZE_BITS);
  packet.append(sps30_data.nc_2p5 * 10 + 0.5, EXTRA_FIELD_BITS);
  packet.append(EXTRA_FIELD_BITS-1, EXTRA_SIZE_BITS);
  packet.append(sps30_data.nc_4p0 * 10 + 0.5, EXTRA_FIELD_BITS);
  packet.append(EXTRA_FIELD_BITS-1, EXTRA_SIZE_BITS);
  packet.append(sps30_data.nc_10p0 * 10 + 0.5, EXTRA_FIELD_BITS);

  packet.append(EXTRA_FIELD_BITS-1, EXTRA_SIZE_BITS);
  packet.append(sps30_data.typical_particle_size * 1000 + 0.5, EXTRA_FIELD_BITS);

  // Fill any remaining bits (from rounding up to whole bytes) with 1's,
  // so they cannot be a valid field.
  packet.append(0xff, packet.free_bits());
#endif // WITH_SPS30_I2C

  // Prepare upstream data transmission at the next possible time.
  LMIC_setTxData2(LORA_PORT, packet.data(), packet.byte_size(), 0);
  if (DEBUG)
  {
    Serial.println(F("Packet queued"));
    uint8_t *data = packet.data();
    for (int i = 0; i < packet.byte_size(); i++)
    {
      if (data[i] < 0x10)
        Serial.write('0');
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    Serial.flush();
  }
}

uint16_t readVcc()
{
  #ifdef __AVR__
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
  #elif defined(ARDUINO_ARCH_STM32L0)
  return STM32L0.getVDDA() * 1000;
  #endif
}

#ifdef WITH_LUX
uint32_t readLux()
{
  uint32_t result = 0;
  uint8_t range = 0;

  // Set the Reference Resistor to just R12
  pinMode(LUX_HIGH_PIN, INPUT);
  // Read the value of Analog input 2 against the internal reference
  analogReference(INTERNAL);
  // Throw away the first reference, in case the internal reference
  // still needs to start up and stabilize (datasheet recommendation)
  analogRead(A2);
  uint16_t raw_adc = analogRead(LUX_PIN);
  // Check if read_low has an overflow
  if (raw_adc < 1000)
  {
    result = uint32_t(lx_conv_low * reference_voltage_internal * raw_adc);
    range = 1;
  } else {
    // Set the Reference Resistor to R11 parallel with R12 for more range
    pinMode(LUX_HIGH_PIN, OUTPUT);
    digitalWrite(LUX_HIGH_PIN, LOW);

    // An external capacitor can be added to charge the ADC internal 14pF
    // without dropping significant voltage, improving read values at low
    // values. If the capacitance is 1000x as big as the internal
    // capacitance, the drop should be limited to 1 ADC value, so 10nF
    // should be fine, but in practice still shows ±50 ADC counts of
    // deviation. Using 100nF reduces this to ±15, so we're using that.
    // Possible there are more sources of noise than just the internal
    // capacitance.
    //
    // When switching from R12 to R11+R12, the capacitor has to charge
    // through R11+R12, which has an RC time of 100nF x 9k = 900μs. To be
    // sure, we wait for 3ms.
    delay(3);

    raw_adc = analogRead(A2);
    // Check if read_high has an overflow
    if (raw_adc < 1000)
    {
      result = uint32_t(lx_conv_high * reference_voltage_internal * raw_adc);
      range = 2;
    } else {
      // Read the value of Analog input 2 against VCC for more range
      analogReference(DEFAULT);
      raw_adc = analogRead(A2);
      result = uint32_t(lx_conv_high * vcc * raw_adc);
      range = 3;
    }
  }

  // Set the Reference Resistor to 100K to draw the least current
  pinMode(LUX_HIGH_PIN, INPUT);
  if (DEBUG)
  {
    Serial.print(F("Lux_reading : "));
    Serial.print(result);
    Serial.print(F(" lx, range="));
    Serial.print(range);
    Serial.print(F(", adc="));
    Serial.println(raw_adc);
  }
  return result;
}

#endif // WITH_LUX

#ifdef WITH_SPS30_I2C
struct sps30_measurement readSps30() {
  struct sps30_measurement res = {};

  // Enable power and start measurement to power up fan
  digitalWrite(PIN_ENABLE_5V, HIGH);
  delay(500);
  int16_t ret = sps30_start_measurement();
  Serial.println("Turned on SPS30, waiting to stabilize");
  delay(10000);

  uint16_t data_ready;
  do {
    ret = sps30_read_data_ready(&data_ready);
    if (ret < 0) {
      Serial.print("Error reading SPS30 data-ready flag: ");
      Serial.println(ret);
      return res;
    }
  } while (!data_ready);

  ret = sps30_read_measurement(&res);
  if (ret < 0) {
    Serial.print("Error reading SPS30 measurement: ");
    Serial.println(ret);
    return res;
  }

  Serial.println("Read data, turning off SPS30");
  digitalWrite(PIN_ENABLE_5V, LOW);

  return res;
}

#endif // WITH_SPS30_I2C
