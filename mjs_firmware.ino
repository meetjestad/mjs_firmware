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
#include <SoftwareSerial.h>
#include <NMEAGPS.h>
#include <Adafruit_SleepyDog.h>
#include <avr/power.h>
#include <util/atomic.h>

#define DEBUG true
#include "bitstream.h"
#include "mjs_lmic.h"


// Firmware version to send. Should be incremented on release (i.e. when
// signficant changes happen, and/or a version is deployed onto
// production nodes). This value should correspond to a release tag.
const uint8_t FIRMWARE_VERSION = 255;

// This sets the ratio of the battery voltage divider attached to A0,
// below works for 100k to ground and 470k to the battery. A setting of
// 0.0 means not to measure the voltage. On first generation boards, this
// should only be enabled when the AREF pin of the microcontroller was
// disconnected.
float const BATTERY_DIVIDER_RATIO = 0.0;
//float const BATTERY_DIVIDER_RATIO = (100.0 + 470.0) / 100.0;

// Enable this define when a light sensor is attached
//#define WITH_LUX

//#define WITH_SDS011
//#define WITH_HPMA115S0_XXX

#ifdef WITH_HPMA115S0_XXX
#include <hpma115S0.h>
#endif

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
// The Reverse dark current (max 30 nA ) is neglectable for our purpose
float const light_current = 8.9;

// R11 and R12 in parallel
float const R11_R12 = (R12 * R11) / (R12 + R11);
float const lx_conv_high = 1.0E6 / (R11_R12 * light_current * 1024.0);
float const lx_conv_low = 1.0E6 / (R12 * light_current * 1024.0);

// Value in mV (nominal @ 25ºC, Vcc=3.3V)
// The temperature coefficient of the reference_voltage is neglected
float const reference_voltage_internal = 1137.0;

// setup GPS module
uint8_t const GPS_PIN = 8;
SoftwareSerial gpsSerial(GPS_PIN, GPS_PIN);
NMEAGPS gps;

#ifdef WITH_SDS011
uint8_t const SDS_RX = A0;
uint8_t const SDS_TX = A1;
SoftwareSerial sdsSerial(SDS_RX, SDS_TX);
#endif

#ifdef WITH_HPMA115S0_XXX
uint8_t const HPMA_RX = 6;
uint8_t const HPMA_TX = 7;
SoftwareSerial hpmaSerial(HPMA_RX, HPMA_TX);
HPMA115S0 hpma115S0(hpmaSerial);
#endif

#if defined(WITH_HPMA115S0_XXX) && defined(WITH_SDS011)
#error "Both SDS and HPMA not supported"
#endif

// Sensor object
HTU21D htu;

// Most recently read values
float temperature;
float humidity;
uint16_t vcc = 0;
#ifdef WITH_LUX
uint16_t lux = 0;
#endif

#if defined(WITH_SDS011) || defined(WITH_HPMA115S0_XXX)
unsigned int Pm25 = 0;
unsigned int Pm10 = 0;
int  PmStatus = -1;
#endif

// define various pins
uint8_t const SW_GND_PIN = 20;
uint8_t const LED_PIN = 21;
uint8_t const LUX_HIGH_PIN = 5;

// setup timing variables
uint32_t const UPDATE_INTERVAL = 900000;
uint32_t const GPS_TIMEOUT = 120000;
// Update GPS position after transmitting this many updates
uint16_t const GPS_UPDATE_RATIO = 24*4;

uint32_t lastUpdateTime = 0;
uint32_t updatesBeforeGpsUpdate = 0;
gps_fix gps_data;

#ifdef WITH_LUX
uint8_t const LORA_PORT = 12;
#else
uint8_t const LORA_PORT = 11;
#endif

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

  // This pin can be used in OUTPUT LOW mode to add an extra pulldown
  // resistor, or in INPUT mode to keep it disconnected
  pinMode(LUX_HIGH_PIN, INPUT);

  // blink 'hello'
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);

  // start communication to sensors
  htu.begin();
  gpsSerial.begin(9600);
#ifdef WITH_SDS011
  sdsSerial.begin(9600);
#endif
#ifdef WITH_HPMA115S0_XXX
  hpmaSerial.begin(9600);
  // TODO: Is this delay really needed? Taken from example.
  delay(5000);
  hpma115S0.Init();
  hpma115S0.StartParticleMeasurement();
#endif

  if (DEBUG) {
    temperature = htu.readTemperature();
    humidity = htu.readHumidity();
    vcc = readVcc();
#ifdef WITH_LUX
    lux = readLux();
#endif
#ifdef WITH_SDS011
    readSds();
#endif
#ifdef WITH_HPMA115S0_XXX
    readHpma();
#endif
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
#if defined(WITH_SDS011) || defined(WITH_HPMA115S0_XXX)
    if (PmStatus == 0) {
      Serial.print(F("PM10: "));
      Serial.print(Pm10);
      Serial.print(F(" PM2.5: "));
      Serial.println(Pm25);
    } else {
      Serial.print("Error reading dust sensor: ");
      Serial.println(PmStatus);
    }
#endif

    if (BATTERY_DIVIDER_RATIO) {
      Serial.print(F("Battery Divider Ratio: "));
      Serial.print(BATTERY_DIVIDER_RATIO);
    }
    Serial.flush();
  }
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
#ifdef WITH_SDS011
  readSds();
#endif
#ifdef WITH_HPMA115S0_XXX
  readHpma();
#endif

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
#if defined(WITH_SDS011) || defined(WITH_HPMA115S0_XXX)
  if (PmStatus == 0) {
    Serial.print(F(", PM10: "));
    Serial.print(Pm10);
    Serial.print(F(", PM2.5: "));
    Serial.println(Pm25);
  } else {
    Serial.print("Error reading dust sensor: ");
    Serial.println(PmStatus);
  }
#endif
  Serial.println();
  Serial.flush();
}

void getPosition()
{
  memset(&gps_data, 0, sizeof(gps_data));
  gps.statistics.init();
  gpsSerial.listen();

  digitalWrite(SW_GND_PIN, HIGH);
  if (DEBUG)
    Serial.println(F("Waiting for GPS..."));

  unsigned long startTime = millis();
  int valid = 0;
  while (millis() - startTime < GPS_TIMEOUT && valid < 10) {
    if (gps.available(gpsSerial)) {
      gps_data = gps.read();
      if (gps_data.valid.location && gps_data.valid.status && gps_data.status >= gps_fix::STATUS_STD)
        valid++;
      if (gps_data.valid.satellites) {
        Serial.print(F("Satellites: "));
        Serial.println(gps_data.satellites);
      }
    }
  }
  digitalWrite(SW_GND_PIN, LOW);

  if (gps.statistics.ok == 0)
    Serial.println(F("No GPS data received, check wiring"));
}

void queueData() {
  uint8_t length = (BATTERY_DIVIDER_RATIO ? 12 : 11);
#ifdef WITH_LUX
  length += 2;
#endif
#if defined(WITH_SDS011) || defined(WITH_HPMA115S0_XXX)
  length += 4;
#endif
  uint8_t data[length];
  BitStream packet(data, sizeof(data));

  packet.append(FIRMWARE_VERSION, 8);
  if (gps_data.valid.location && gps_data.valid.status && gps_data.status >= gps_fix::STATUS_STD) {
    // pack geoposition
    int32_t lat24 = int32_t((int64_t)gps_data.latitudeL() * 32768 / 10000000);
    packet.append(lat24, 24);

    int32_t lng24 = int32_t((int64_t)gps_data.longitudeL() * 32768 / 10000000);
    packet.append(lng24, 24);
  } else {
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
  uint8_t vcc8 = (vcc - 1000) / 10;
  packet.append(vcc8, 8);

#ifdef WITH_LUX
  packet.append(lux, 16);
#endif
#if defined(WITH_SDS011) || defined(WITH_HPMA115S0_XXX)
  if (PmStatus != 0) {
    // Read error
    packet.append(0xffff, 16);
    packet.append(0xffff, 16);
  } else {
    packet.append(Pm25, 16);
    packet.append(Pm10, 16);
  }
#endif

  if (BATTERY_DIVIDER_RATIO) {
    analogReference(INTERNAL);
    uint16_t reading = analogRead(A0);
    // Encoded in units of 20mv
    uint8_t batt = (uint32_t)(50*BATTERY_DIVIDER_RATIO*1.1)*reading/1023;
    // Shift down, zero means 1V now
    if (batt >= 50)
      packet.append(batt - 50, 8);
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

#ifdef WITH_LUX
long readLux()
{
  long result = 0;
  bool done = false;
  int range = 0;

  // Set the Reference Resistor to 100K
  pinMode(LUX_HIGH_PIN, INPUT);
  // Read the value of Analog input 2 against the internal reference
  analogReference(INTERNAL);
  uint16_t read_low = analogRead(A2);
  // Check if read_low has an overflow
  if (read_low < 1000)
  {
    result = long(lx_conv_low * reference_voltage_internal * read_low);
    done = true;
    range = 1;
  } else {
    // Set the Reference Resistor to 10K parallel with 100K = 9.091K
    pinMode(LUX_HIGH_PIN, OUTPUT);
    digitalWrite(LUX_HIGH_PIN, LOW);
    // Read the value of Analog input 2 against the internal reference
    analogReference(INTERNAL);
    uint16_t read_high = analogRead(A2);
    // Check if read_high has an overflow
    if (read_high < 1000)
    {
      result = long(lx_conv_high * reference_voltage_internal * read_high);
      range = 2;
    } else {
      // Set the Reference Resistor to 10K parallel with 100K = 9.091K
      pinMode(LUX_HIGH_PIN, OUTPUT);
      digitalWrite(LUX_HIGH_PIN, LOW);
      // Read the value of Analog input 2 against the battery voltage
      analogReference(DEFAULT);
      uint16_t read_highest = analogRead(A2);
      result = long(lx_conv_high * vcc * read_highest);
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
    Serial.println(range);
  }
  return result;
}

#endif

#ifdef WITH_SDS011
void readSds()
{
  uint8_t mData = 0;
  uint8_t i = 0;
  uint8_t mPkt[10] = {0};
  uint8_t mCheck = 0;

  PmStatus = -1;  // Default: no data availabe ...
  //fpm10 = -99.0;
  //fpm25 = -99.0;

  // Flush any old data
  while (sdsSerial.read() >= 0) /* nothing */;
  sdsSerial.listen();

  // TODO: Timeout
  while (PmStatus < 0)
  {
    // from www.inovafitness.com
    // packet format: AA C0 PM25_Low PM25_High PM10_Low PM10_High 0 0 CRC AB

    while (!sdsSerial.available()) /* nothing */;
    mData = sdsSerial.read();

    if (mData == 0xAA) //head1 ok
    {
      mPkt[0] =  mData;
      while (!sdsSerial.available()) /* nothing */;
      mData = sdsSerial.read();
      if (mData == 0xc0) //head2 ok
      {
        mPkt[1] =  mData;
        mCheck = 0;
        for (i = 0; i < 6; i++) //data recv and crc calc
        {
          while (!sdsSerial.available()) /* nothing */;
          mPkt[i + 2] = sdsSerial.read();
          mCheck += mPkt[i + 2];
        }
        while (!sdsSerial.available()) /* nothing */;
        mPkt[8] = sdsSerial.read();
        while (!sdsSerial.available()) /* nothing */;
        mPkt[9] = sdsSerial.read();
        if (mCheck == mPkt[8]) //crc ok
        {
          Pm25 = (uint16_t)mPkt[2] | (uint16_t)(mPkt[3] << 8);
          Pm10 = (uint16_t)mPkt[4] | (uint16_t)(mPkt[5] << 8);
          // Convert from tenths of μg/m³ to integer μg/m³
          Pm25 /= 10;
          Pm10 /= 10;

          /*
          fpm25 = Pm25 / 10.0;
          fpm10 = Pm10 / 10.0;

          if (fpm25 > 999)
            fpm25 = 999;
          if (fpm10 > 999)
            fpm10 = 999;
          */
          PmStatus = 0; // Data is ok

        } // CRC ??
        else {
          Serial.println("CRC != OK,  ");
        }
      } // Head2 ??
      else {
        Serial.println("mData != 0xc0 ");
      }

    } // Head1 ??
    else {
      Serial.println("mData != 0xAA ");
    }
    // TODO: This *disables* listening on the SDS serial port, otherwise
    // messages sent from it will interrupt our sleep and cut our send
    // interval short. Since we can't *disable* SoftwareSerial, we
    // instead enable another one, which as a side effect disables all
    // others. It might be better to disable messages from the SDS
    // instead.
    gpsSerial.listen();
  }
}
#endif

#ifdef WITH_HPMA115S0_XXX
void readHpma() {
  hpmaSerial.listen();
  if (hpma115S0.ReadParticleMeasurement(&Pm25, &Pm10)) {
    PmStatus = 0;
  } else {
    PmStatus = 1;
  }
}
#endif
