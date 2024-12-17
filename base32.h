/*******************************************************************************
   Copyright (c) 2024 Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

 *******************************************************************************/

#pragma once

#include <Arduino.h>
#include "bitstream.h"

// RFC4648 paragraph 6 base32 decoding
inline int base32Read(Stream& s, uint8_t *buf, size_t len) {
  BitStream bs(buf, len);
  const size_t BITS_PER_CHAR = 5;
  while (bs.free_bits() > 0) {
    int c = s.read();
    if (c == -1)
      continue;

    // Echo
    s.write(c);

    uint8_t data;
    if (c >= 'A' && c <= 'Z') {
      data = c - 'A';
    } else if (c >= '2' && c <= '7') {
      data = c - '2' + 26;
    } else {
      return -1;
    }

    uint8_t bits = 5;
    while (bs.free_bits() < bits) {
      if ((data & 0x1) != 0) {
        // Any extra bits should have been padded with zeroes
        return -1;
      }
      data >>= 1;
      --bits;
    }
    bs.append(data, bits);
  }

  // Officially, the encoding should be padded with '=' to a multiple of
  // 40 bits (8 encoded characteres, but since we know how much bits to
  // expect, this padding can be omitted. Just in case padding is
  // present, ignore it.
  int c;
  do {
    c = s.read();
    if (c != -1)
      s.write(c);
  } while (c == '=' || c == -1);

  // Return the next character read after the base32
  return c;
}
