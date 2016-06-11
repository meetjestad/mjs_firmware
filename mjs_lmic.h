/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman & Bas Peschier

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

 *******************************************************************************/

#define EEPROM_LAYOUT_MAGIC 0x2a60af86 // Just a random number, stored little-endian
#define EEPROM_LAYOUT_MAGIC_START 0x00 // 4 bytes
#define EEPROM_OSCCAL_START (EEPROM_LAYOUT_MAGIC_START + 4) // 1 byte
#define EEPROM_APP_EUI_START (EEPROM_OSCCAL_START + 1)
#define EEPROM_APP_EUI_LEN 8
#define EEPROM_DEV_EUI_START (EEPROM_APP_EUI_START + EEPROM_APP_EUI_LEN)
#define EEPROM_DEV_EUI_LEN 8
#define EEPROM_APP_KEY_START (EEPROM_DEV_EUI_START + EEPROM_DEV_EUI_LEN)
#define EEPROM_APP_KEY_LEN 16

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <avr/eeprom.h>

void updateTransceiver() {
  os_runloop_once();
}

void os_getArtEui (u1_t* buf) {
  for (byte i = 0; i < EEPROM_APP_EUI_LEN; i++) {
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_APP_EUI_START + EEPROM_APP_EUI_LEN - 1 - i);
  }
}

void os_getDevEui (u1_t* buf) {
  for (byte i = 0; i < EEPROM_DEV_EUI_LEN; i++) {
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_DEV_EUI_START + EEPROM_DEV_EUI_LEN - 1 - i);
  }
}

void os_getDevKey (u1_t* buf) {
  for (byte i = 0; i < EEPROM_APP_KEY_START; i++) {
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_APP_KEY_START + i);
  }
}

const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
  if (DEBUG) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
      case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
      case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
      case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
      case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
      case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
      case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        break;
      case EV_RFU1:
        Serial.println(F("EV_RFU1"));
        break;
      case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
      case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
        break;
      case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        break;
      case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
      case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
      case EV_RXCOMPLETE:
        // data received in ping slot
        Serial.println(F("EV_RXCOMPLETE"));
        break;
      case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        break;
      case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
      default:
        Serial.println(F("Unknown event"));
        break;
    }
  }
}

void mjs_lmic_setup() {
  // Check whether the layout of the EEPROM is correct
  uint32_t hash = eeprom_read_dword(0x00);
  if (hash != EEPROM_LAYOUT_MAGIC) {
    Serial.begin(9600);
    Serial.println(F("EEPROM is not correctly configured"));

    while (true) {
//      digitalWrite(LED_PIN, HIGH);
//      delay(250);
//      digitalWrite(LED_PIN, LOW);
//      delay(250);
    }
  }

  // Write OSCCAL from EEPROM
  uint8_t osccal_byte = eeprom_read_byte((uint8_t*)EEPROM_OSCCAL_START);
  if (osccal_byte != 0xff) {
    OSCCAL = osccal_byte;
  }

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // Use a fixed data rate of SF9 (not sure if tx power is actually
  // used). SF9 is the lowest datarate that (withing the TTN fair-usage-policy of 30 seconds of airtime
  // per day) allows us to send at least 4 packets every hour.
  LMIC_setDrTxpow(DR_SF9, 14);

  // Let LMIC compensate for +/- 1% clock error
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
}

