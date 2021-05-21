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

// Baudrate for hardware serial port
const uint16_t SERIAL_BAUD = 9600;

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

float const SOLAR_DIVIDER_RATIO = 0;
uint8_t const SOLAR_DIVIDER_PIN = 0;
auto const SOLAR_DIVIDER_REF = INTERNAL;
uint16_t const SOLAR_DIVIDER_REF_MV = 1137;

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

#elif defined(ARDUINO_MJS2020)
float const BATTERY_DIVIDER_RATIO = (1.0 + 1.0) / 1.0;
uint8_t const BATTERY_DIVIDER_PIN = PIN_BATTERY;
auto const BATTERY_DIVIDER_REF = AR_DEFAULT;
uint16_t const BATTERY_DIVIDER_REF_MV = 3000;

float const SOLAR_DIVIDER_RATIO = (2.0 + 1.0) / 1.0;
uint8_t const SOLAR_DIVIDER_PIN = PIN_SOLAR;
auto const SOLAR_DIVIDER_REF = AR_DEFAULT;
uint16_t const SOLAR_DIVIDER_REF_MV = 3000;

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

// GPS reader
NMEAGPS gps;

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

#if defined(SERIAL_IS_SERIALUSB) || defined(SERIAL_IS_CONFIGURABLE)
bool wait_for_usb_configured(unsigned long timeout) {
    unsigned long start = millis();
    while (!USBDevice.connected() && millis() - start < timeout) /* wait */;

    return USBDevice.connected();
}

bool wait_for_serialusb_opened(unsigned long timeout) {
    unsigned long start = millis();
    while (!SerialUSB && millis() - start < timeout) /* wait */;

    return (bool)SerialUSB;
}

bool setup_serialusb() {
  // First, see if we are configured at the USB device level. This
  // happens automatically when USB is plugged into an USB host without
  // any user interaction, typically within a couple of hundred ms. If
  // that doesn't happen quickly, assume we're not connected to USB at
  // all and fail.
  if (wait_for_usb_configured(1000)) {
    // USB device was configured, so start serial port.
    SerialUSB.begin(0);

    // Then, see if the serial port is actually opened. Because this
    // needs action from the user, give them ample time for this.
    if (wait_for_serialusb_opened(10000)) {
      // Serial port was opened, we can use it.
      //
      // Wait a bit more, otherwise some clients (e.g. minicom on Linux) seem to miss the first bit of output...
      delay(250);

      return true;
    } else {
      // Stop the serial port again so any prints to it are ignored,
      // rather than filling up the buffer and eventually locking up
      // when the buffer is full.
      SerialUSB.end();
    }
  }

  return false;
}
#endif // defined(SERIAL_IS_SERIALUSB) || defined(SERIAL_IS_CONFIGURABLE)

bool setup_serial() {
  #if defined(ARDUINO_MJS_V1) || defined(SERIAL_IS_SERIAL1)
  // For a hardware serial port, just call begin and be done.
  // Anyone interested in the output can start listening before power-up
  // or reset and if nobody is listening, then the messages are just
  // lost without blocking.
  Serial.begin(SERIAL_BAUD);
  #elif defined(SERIAL_IS_SERIALUSB)
  // For SerialUSB, things are more tricky, since the computer can only
  // open the serial port *after* the board started. This delays
  // startup to give the computer a chance to open the port so it can
  // see all messages printed to serial.
  setup_serialusb();
  #elif defined(SERIAL_IS_CONFIGURABLE)
  if (setup_serialusb()) {
    // SerialUSB port was openend, use that
    ConfigurableSerial = &SerialUSB;
  } else {
    // SerialUSB port was not opened, use hardware serial port instead
    Serial1.begin(SERIAL_BAUD);
    ConfigurableSerial = &Serial1;
  }
  #else
  #error "Unknown serial setup"
  #endif
}

void setup() {
  writeLed(0xff0000); // red

  // when in debugging mode start serial connection
  if(DEBUG) {
    setup_serial();
    Serial.println(F("Start"));
  }

  writeLed(0xff0c00); // orange

  // setup LoRa transceiver
  mjs_lmic_setup();

  // Work around BasicMAC initializing LED_BUILTIN to OUTPUT, can be
  // removed once BasicMAC is fixed. This produces a very short blink.
  writeLed(0x800c00); // orange

  // setup switched ground and power down connected peripherals (GPS module)
  pinMode(GPS_ENABLE_PIN , OUTPUT);
  digitalWrite(GPS_ENABLE_PIN, LOW);

  #if defined(ARDUINO_MJS2020)
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
    #if defined(ARDUINO_MJS2020)
    digitalWrite(PIN_ENABLE_5V, HIGH);
    #endif // defined(ARDUINO_MJS2020)
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

    Serial.print("Typical particle size: ");
    Serial.println(sps30_data.typical_particle_size);
#endif // defined(WITH_SPS30_I2C)

    if (BATTERY_DIVIDER_RATIO) {
      Serial.print(F("Battery Divider Ratio: "));
      Serial.println(BATTERY_DIVIDER_RATIO);
    }
    if (SOLAR_DIVIDER_RATIO) {
      Serial.print(F("Solar Divider Ratio: "));
      Serial.println(SOLAR_DIVIDER_RATIO);
    }
    Serial.flush();
  }

  writeLed(0x803000); // yellow

  // Start join
  LMIC_startJoining();

  // Wait for join to complete
  // TODO: Sleep between join attempts
  while ((LMIC.opmode & (OP_JOINING)))
    os_runloop_once();

  writeLed(0x000000); // off (will be turned back on for GPS directly)
}

void loop() {
  // We need to calculate how long we should sleep, so we need to know how long we were awake
  unsigned long startMillis = millis();

  // Activate GPS every now and then to update our position
  if (updatesBeforeGpsUpdate == 0) {
    writeLed(0x408080); // cyan
    getPosition();
    writeLed(0x000000); // off
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
  writeLed(0xff00ff); // purple
  sps30_data = readSps30();
  writeLed(0x000000); // off
#endif // defined(WITH_SPS30_I2C)

  if (DEBUG)
    dumpData();

  writeLed(0x0000ff); // blue
  // Work around a race condition in LMIC, that is greatly amplified
  // if we sleep without calling runloop and then queue data
  // See https://github.com/lmic-lib/lmic/issues/3
  os_runloop_once();

  // We can now send the data
  queueData();

  mjs_lmic_wait_for_txcomplete();

  writeLed(0x000000); // off

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

  // Shut down USB. Without this, sleeping is completely prevented (and
  // the USB block also consumes power).
  // By doing this here, rather than in setup_serialusb(), firmware
  // uploads remain possible until the first sleep (and also usb is now
  // detached even when not using serialusb).
  // Note that after sleep, USB is not reattached. When USB is already
  // detached, this is just a no-op.
  USBDevice.detach();

  // No need to update the millis counter, STM32L0 uses the RTC for
  // millis and keeps it running during sleep for wakeup.
  if (time)
    STM32L0.stop(time);
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
  const uint8_t SOLAR_EXTRA_FIELD_BITS = 15;
  if (SOLAR_DIVIDER_RATIO) {
    length += SOLAR_EXTRA_FIELD_BITS;
    flags |= FLAG_WITH_EXTRA;
  }

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
    // Encoded in units of 20mv
    uint8_t batt = readVbatt() / 20;
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

  if (SOLAR_DIVIDER_RATIO) {
    // Encoded in units of 1mv
    uint16_t solar = readVsolar();
    packet.append(SOLAR_EXTRA_FIELD_BITS-1, EXTRA_SIZE_BITS);
    packet.append(solar, SOLAR_EXTRA_FIELD_BITS);
  }

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

uint16_t readVsolar() {
    analogReference(SOLAR_DIVIDER_REF);
    uint16_t reading = analogRead(SOLAR_DIVIDER_PIN);
    return (uint32_t)(reading*SOLAR_DIVIDER_RATIO*SOLAR_DIVIDER_REF_MV)/1023;
}

uint16_t readVbatt() {
    analogReference(BATTERY_DIVIDER_REF);
    uint16_t reading = analogRead(BATTERY_DIVIDER_PIN);
    return (uint32_t)(reading*BATTERY_DIVIDER_RATIO*BATTERY_DIVIDER_REF_MV)/1023;
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
  #if defined(ARDUINO_MJS2020)
  digitalWrite(PIN_ENABLE_5V, HIGH);
  #endif // defined(ARDUINO_MJS2020)
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
  #if defined(ARDUINO_MJS2020)
  digitalWrite(PIN_ENABLE_5V, LOW);
  #endif // defined(ARDUINO_MJS2020)

  return res;
}
#endif // WITH_SPS30_I2C

void writeSingleLed(uint8_t pin, uint8_t val) {
  #if defined(ARDUINO_MJS2020)
  // Work around a problem that analogWrite(255) is still
  // low for 1/256th of the time. Use HIGHZ for zero to save a
  // little power by disabling the digital pin driver completely. Also,
  // work around that analogWrite(4095) is not fully high (which is
  // problematic on PROTO2, because it means the led does not fully turn
  // off).
  // https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0/issues/104
  if (val == 0) {
    pinMode(pin, HIGHZ);
  } else {
    // Use the full 12-bits resolution (so analogWrite accepts 0-4095).
    // By not scaling val up, this effectively dims the led 4x (but
    // without rounding, so you can still produce tiny color
    // differences).
    analogWriteResolution(10);
    #if defined(ARDUINO_MJS2020_PROTO2)
    // On PROTO2, the LED is active-low
    analogWrite(pin, 1023 - val);
    #else
    analogWrite(pin, val);
    #endif
    // Reset back to the default, which other code probably expects.
    analogWriteResolution(8);
  }
  #endif
}

void writeLed(uint32_t rgb) {
  #if defined(ARDUINO_MJS2020)
  writeSingleLed(PIN_LED_RED, (rgb >> 16));
  writeSingleLed(PIN_LED_GREEN, (rgb >> 8));;
  writeSingleLed(PIN_LED_BLUE, (rgb >> 0));
  #endif
}
