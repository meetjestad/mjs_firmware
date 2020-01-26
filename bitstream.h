#pragma once

/**
 * Class that writes signed or unsigned integers with any bit size (not
 * requiring multiples of 8) to a buffer. This class in intended to be
 * used for building networking packets.
 */
class BitStream {
  public:
    BitStream(uint8_t *buf, size_t len);
    bool append(uint32_t v, size_t bits);
    void reset();

    size_t byte_size();
    size_t bit_size();
    size_t free_bits();
    uint8_t *data();

  private:
    // External buffer to write to
    uint8_t *buf;
    // Length of buffer in bytes
    size_t buflen;
    // Bit position of next write
    size_t pos = 0;
};

inline BitStream::BitStream(uint8_t *buf, size_t len)
  : buf(buf), buflen(len) { }

inline size_t BitStream::byte_size() {
  return (this->pos + 7) / 8;
}

inline size_t BitStream::bit_size() {
  return this->pos;
}

inline size_t BitStream::free_bits() {
  return buflen * 8 - bit_size();
}

inline uint8_t *BitStream::data() {
  return this->buf;
}

inline void BitStream::reset() {
  this->pos = 0;
}

/**
 * Append a new variable to the stream, with the given size in bits.
 * This works for both signed and unsigned values, since signed values
 * will get sign-extended to 32 bits, so (provided the signed value fits
 * in a bitsize-bits signed integer) all uppper bits down to and
 * including bit number (bitsize - 1) contain the sign bit.
 *
 * Returns true when the data was appended, false when there was
 * insufficient room in the buffer.
 */
inline bool BitStream::append(uint32_t v, size_t bitsize) {
  // Sanity check
  if (bitsize > sizeof(v) * 8)
    return false;

  // The value is written to buf starting from the lowest bits working
  // upward, so start from the last byte
  uint8_t *ptr = this->buf + (this->pos + bitsize - 1) / 8;

  // Do not overfow the buffer
  if (ptr >= this->buf + this->buflen) {
    return false;
  }

  this->pos += bitsize;

  // Find out how many bits there are in the last partial byte to be
  // written to buf. This can be 0 if the lsb from v lines up with bytes
  // in the buffer.
  uint8_t partial = this->pos % 8;

  // If partial is zero, v is already byte aligned
  if (partial) {
    // To bytewise align v with buf, v needs to be shifted left this much
    uint8_t shift = (8 - partial);

    // If we can shift without losing bits, do so
    if (shift + bitsize < sizeof(v) * 8) {
      v <<= shift;
      bitsize += shift;
    } else {
      // If we cannot shift so far left, handle the lowest byte separately
      // and then shift right.
      *ptr = v << shift;
      ptr--;
      // Remove the bits from v
      bitsize -= partial;
      v >>= partial;
    }
  }

  while (bitsize >= 8) {
    // Write a full byte to the buffer
    *ptr = v & 0xff;
    ptr--;
    // Remove the byte from v
    v >>= 8;
    bitsize -= 8;
  }

  if (bitsize) {
    // Write the uppermost partial byte. Mask off all but the bottom
    // bitsize bits and add them to the existing bits in buf.
    uint8_t mask = ~(0xff << bitsize);
    *ptr |= v & mask;
  }

  return true;
}

// vim: set sw=2 sts=2 et:
