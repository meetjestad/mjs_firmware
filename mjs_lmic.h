/*******************************************************************************
   Copyright (c) 2016 Thomas Telkamp, Matthijs Kooijman & Bas Peschier

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

 *******************************************************************************/

//#define USE_EEPROM_ON_MJS2020

#define MJS_APP_EUI_LEN 8
#define MJS_DEV_EUI_LEN 8
#define MJS_APP_KEY_LEN 16

#if defined(ARDUINO_ARCH_STM32L0) && defined(USE_EEPROM_ON_MJS2020)
  // New layout
  // TODO: This actually more dynamic than this and has a CRC
  // TODO: Do not hardcode this size (but EEPROM.length() is not
  // correct, see
  // https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0/pull/166
  #define EEPROM_SIZE 6144
  #define MJS_LAYOUT_MAGIC_OLD 0x023BE0B6 // Just a random number, stored little-endian
  #define MJS_LAYOUT_MAGIC 0x023BE0B6 // Just a random number, stored little-endian
  #define EEPROM_LAYOUT_MAGIC_START (EEPROM_SIZE - EEPROM_LAYOUT_MAGIC_LEN)
  #define EEPROM_LAYOUT_MAGIC_LEN 4
  #define EEPROM_BLOCK_SEGMENT_FOOTER_LEN 6
  #define EEPROM_APP_EUI_START (EEPROM_DEV_EUI_START - MJS_APP_EUI_LEN)
  #define EEPROM_DEV_EUI_START (EEPROM_APP_KEY_START - MJS_DEV_EUI_LEN)
  #define EEPROM_APP_KEY_START (EEPROM_LAYOUT_MAGIC_START - EEPROM_BLOCK_SEGMENT_FOOTER_LEN - MJS_APP_KEY_LEN)
#elif defined(ARDUINO_ARCH_STM32L0)
  #define MJS_LAYOUT_MAGIC_OLD 0x023BE0B6 // Just a random number, stored little-endian
  #define MJS_LAYOUT_MAGIC 0x023BE0B6 // Just a random number, stored little-endian
  #define FLASH_LAYOUT_MAGIC_START (FLASH_SIZE - FLASH_LAYOUT_MAGIC_LEN)
  #define FLASH_LAYOUT_MAGIC_LEN 4
  #define FLASH_BLOCK_SEGMENT_FOOTER_LEN 6
  #define FLASH_APP_EUI_START (FLASH_DEV_EUI_START - MJS_APP_EUI_LEN)
  #define FLASH_DEV_EUI_START (FLASH_APP_KEY_START - MJS_DEV_EUI_LEN)
  #define FLASH_APP_KEY_START (FLASH_LAYOUT_MAGIC_START - FLASH_BLOCK_SEGMENT_FOOTER_LEN - MJS_APP_KEY_LEN)
#else
  // Original EEPROM layout for the AVR board
  #define MJS_LAYOUT_MAGIC_OLD 0x2a60af86 // Just a random number, stored little-endian
  #define MJS_LAYOUT_MAGIC 0x2a60af87 // Just a random number, stored little-endian
  #define EEPROM_LAYOUT_MAGIC_START 0x00 // 4 bytes
  #define EEPROM_OSCCAL_START (EEPROM_LAYOUT_MAGIC_START + 4) // 1 byte
  #define EEPROM_APP_EUI_START (EEPROM_OSCCAL_START + 1)
  #define EEPROM_DEV_EUI_START (EEPROM_APP_EUI_START + MJS_APP_EUI_LEN)
  #define EEPROM_APP_KEY_START (EEPROM_DEV_EUI_START + MJS_DEV_EUI_LEN)
#endif

// Try transmission for up to 60 seconds (this includes joining)
const uint32_t TX_TIMEOUT = 60000;

#if defined(ARDUINO_ARCH_STM32L0)
#define USE_BASICMAC
#endif

#if defined(USE_BASICMAC)
  #include <basicmac.h>
#else
  #include <lmic.h>
#endif
#include <hal/hal.h>
#include <SPI.h>
#include <avr/eeprom.h>

#include "base32.h"

#if defined(USE_BASICMAC)
  #define os_getArtEui os_getJoinEui
  #define os_getDevKey os_getNwkKey
  #define onEvent onLmicEvent
  #define os_runloop_once os_runstep
  #define OS_INIT_ARG nullptr
  #define DR_SF12 0
  #define DR_SF9 3
#else
  #define OS_INIT_ARG
#endif

void os_getArtEui (uint8_t* buf) {
  for (byte i = 0; i < MJS_APP_EUI_LEN; i++) {
    #if defined(EEPROM_APP_EUI_START)
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_APP_EUI_START + MJS_APP_EUI_LEN - 1 - i);
    #else
    buf[i] = *((uint8_t*)FLASH_APP_EUI_START + MJS_APP_EUI_LEN - 1 - i);
    #endif
  }
}

void os_getDevEui (uint8_t* buf) {
  for (byte i = 0; i < MJS_DEV_EUI_LEN; i++) {
    #if defined(EEPROM_DEV_EUI_START)
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_DEV_EUI_START + MJS_DEV_EUI_LEN - 1 - i);
    #else
    buf[i] = *((uint8_t*)FLASH_DEV_EUI_START + MJS_DEV_EUI_LEN - 1 - i);
    #endif
  }
}

void os_getDevKey (uint8_t* buf) {
  for (byte i = 0; i < MJS_APP_KEY_LEN; i++) {
    #if defined(EEPROM_APP_KEY_START)
    buf[i] = eeprom_read_byte((uint8_t*)EEPROM_APP_KEY_START + i);
    #else
    buf[i] = *((uint8_t*)FLASH_APP_KEY_START + i);
    #endif
  }
}

#if defined(USE_BASICMAC)
u1_t os_getRegion (void) {
  return REGCODE_EU868;
}
#endif

#if defined(ARDUINO_MJS_V1)
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 3, 4},
};
#elif defined(ARDUINO_MJS2020)
const lmic_pinmap lmic_pins = {
  .nss = PIN_LORA_SS,
  .tx = LMIC_CONTROLLED_BY_DIO2,
  .rx = LMIC_UNUSED_PIN,
  .rst = PIN_LORA_RST,
  .dio = {/* DIO0 */ LMIC_UNUSED_PIN, /* DIO1 */ PIN_LORA_DIO1, /* DIO2 */ LMIC_UNUSED_PIN},
  .busy = PIN_LORA_BUSY,
  .tcxo = LMIC_CONTROLLED_BY_DIO3,
};
#else
  #error "Unknown board"
#endif

ev_t waitingForEvent = (ev_t)0;

void mjs_credentials_join_success();

void onEvent (ev_t ev) {
  if (waitingForEvent == ev)
    waitingForEvent = (ev_t)0;

  if (DEBUG) {
    Serial.print((uint32_t)os_getTime());
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
        // Let join start at SF9, since that's what we'll be using for
        // transmission anyway
        LMIC_setDrTxpow(DR_SF9, 14);
        Serial.println(F("EV_JOINING"));
        break;
      case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        mjs_credentials_join_success();
        // Disable link check validation
        LMIC_setLinkCheckMode(0);
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

void printHex(const __FlashStringHelper *prefix, uint8_t *buf, size_t len, bool reverse = false) {
  Serial.print(prefix);
  for (size_t i = 0; i < len; ++i) {
    uint8_t b = reverse ? buf[len - i - 1]: buf[i];
    if (b < 0x10)
      Serial.write('0');
    Serial.print(b, HEX);
  }
  Serial.println();
}

struct Credentials {
  uint8_t appEui[MJS_APP_EUI_LEN];
  uint8_t devEui[MJS_DEV_EUI_LEN];
  uint8_t appKey[MJS_APP_KEY_LEN];
};

bool mjs_credentials_pending = false;

bool mjs_credentials_get_from_serial(Credentials *c) {
  // Clear input buffer
  while(Serial.read() != -1) /* nothing */;

  Serial.print(F("Enter credential bundle: "));

  uint8_t buf[MJS_APP_EUI_LEN + MJS_DEV_EUI_LEN + MJS_APP_KEY_LEN];
  BitStream bs(buf, sizeof(buf));

  uint8_t header[3];

  // A credential bundle is the base32 encoding of the string "lwc"
  // (LoRaWAN credentials), appEUI (big endian), devEUI (big endian) and
  // appKey, seperated by periods (after base32 encoding). Base32
  // padding can be omitted.
  int next = base32Read(Serial, header, sizeof(header));
  if (next < 0 || next != '.' || memcmp(header, "lwc", sizeof(header) != 0))
    return false;
  next = base32Read(Serial, c->appEui, sizeof(c->appEui));
  if (next < 0 || next != '.')
    return false;
  next = base32Read(Serial, c->devEui, sizeof(c->devEui));
  if (next < 0 || next != '.')
    return false;
  next = base32Read(Serial, c->appKey, sizeof(c->appKey));
  if (next < 0 || next != '\n' && next != '\r')
    return false;

  return true;
}

void mjs_credentials_save_new(Credentials *c) {
  #if defined(EEPROM_LAYOUT_MAGIC_START)
  // Invalidate layout magic to ensure these credentials are not used
  // on the next reboot
  eeprom_write_dword((uint32_t*)EEPROM_LAYOUT_MAGIC_START, 0xffffffff);

  eeprom_write_block(&c->appEui, (uint32_t*)EEPROM_APP_EUI_START, sizeof(c->appEui));
  eeprom_write_block(&c->devEui, (uint32_t*)EEPROM_DEV_EUI_START, sizeof(c->devEui));
  eeprom_write_block(&c->appKey, (uint32_t*)EEPROM_APP_KEY_START, sizeof(c->appKey));
  #else
  #error "Credential saving not implemented for flash"
  #endif
}

void mjs_credentials_join_success() {
  if (mjs_credentials_pending) {
    Serial.println(F("New credentials work, saving them permanently"));
    #if defined(EEPROM_LAYOUT_MAGIC_START)
    eeprom_write_dword((uint32_t*)EEPROM_LAYOUT_MAGIC_START, MJS_LAYOUT_MAGIC);
    #else
    #error "Credential saving not implemented for flash"
    #endif
    mjs_credentials_pending = false;
  }
}

void mjs_credentials_setup() {
  // Check whether the layout of the EEPROM is correct
  #if defined(EEPROM_LAYOUT_MAGIC_START)
  uint32_t hash = eeprom_read_dword((uint32_t*)EEPROM_LAYOUT_MAGIC_START);
  #else
  uint32_t hash = *((uint32_t*)FLASH_LAYOUT_MAGIC_START);
  #endif

  if (hash != MJS_LAYOUT_MAGIC && hash != MJS_LAYOUT_MAGIC_OLD) {
    Serial.println();
    Serial.println(F("No The Things Network credentials found."));
    Serial.println(F("Go to https://meetjestad.net/register to register your device, then enter the generated credential bundle below."));
    Serial.println();

    Credentials c;
    while (mjs_credentials_get_from_serial(&c) == false) {
      Serial.println();
      Serial.println(F("Invalid credential format. A valid credential bundle starts with \"NR3WW.\" and consists of 60 uppercase letters, numbers and dots. Please try again."));
      Serial.println();
    }
    Serial.println();

    // This saves just the credentials, the magic value is written only
    // after a successful join to prevent using potentially invalid or
    // expired credentials (without an easy way to change them again).
    mjs_credentials_save_new(&c);
    mjs_credentials_pending = true;
  }

  #if defined(EEPROM_OSCCAL_START)
  // Old magic indicates the bootloader did not handle OSCCAL yet, so we
  // need to load it from EEPROM
  if (hash == MJS_LAYOUT_MAGIC_OLD) {
    // Read OSCCAL from EEPROM
    uint8_t osccal_byte = eeprom_read_byte((uint8_t*)EEPROM_OSCCAL_START);
    if (osccal_byte != 0xff) {
      OSCCAL = osccal_byte;
    }
  }
  #endif
}

void mjs_lmic_setup() {
  mjs_credentials_setup();

  uint8_t buf[MJS_APP_KEY_LEN];
  os_getDevEui(buf);
  // For devices with low (< 64k) DevEUI values, print a more human
  // readable, decimal id too (as used in TTN).
  if (!buf[7] && !buf[6] && !buf[5] && !buf[4] && !buf[3] && !buf[2]) {
    Serial.print("meetstation-");
    Serial.println((uint16_t)buf[1] << 8 | buf[0]);
  }
  printHex(F("Dev EUI: "), buf, MJS_DEV_EUI_LEN, true);
  os_getArtEui(buf);
  printHex(F("App EUI: "), buf, MJS_APP_EUI_LEN, true);
  os_getDevKey(buf);
  printHex(F("App Key: "), buf, MJS_APP_KEY_LEN, false);

  if (mjs_credentials_pending) {
    Serial.println(F("Trying new credentials, will be stored permanently once join succeeds"));
    Serial.println();
  }

  // LMIC init
  os_init(OS_INIT_ARG);
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  #if defined(ARDUINO_MJS_V1)
  // Let LMIC compensate for +/- 2% clock error
  // Not needed on MJS2020 and not supported by BasicMAC either
  LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);
  #endif
}

void mjs_lmic_wait_for_txcomplete() {
  waitingForEvent = EV_TXCOMPLETE;
  uint32_t start = millis();
  while(waitingForEvent && millis() - start < TX_TIMEOUT)
    os_runloop_once();
  if (DEBUG) {
    if (waitingForEvent)
      Serial.println(F("Transmit timeout"));
    else
      Serial.println(F("Transmit complete"));
  }
}
