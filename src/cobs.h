#pragma once
// COBS decode in place. Returns decoded length, 0 on error.
#include <stdint.h>
#include <stddef.h>

static inline size_t cobs_decode_inplace(uint8_t *buf, size_t len) {
  if (len == 0) return 0;
  size_t read = 0;
  size_t write = 0;
  while (read < len) {
    uint8_t code = buf[read++];
    if (code == 0) return 0;  // delimiter inside block
    size_t end = read + (size_t)code - 1;
    if (end > len) return 0;  // overrun
    while (read < end) buf[write++] = buf[read++];
    if (code < 0xFF && read < len) buf[write++] = 0;
  }
  return write;
}
