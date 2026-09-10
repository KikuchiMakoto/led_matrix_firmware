#pragma once
// Binary frame protocol v2: Fixed 4B header + Raw payload + COBS + 0x00.
// See led_matrix_software/protocol.py (single source of truth duplicated here).
#include <stdint.h>
#include <stddef.h>

#define PROTO_MAGIC 0x55
#define PROTO_DELIMITER 0x00
#define PROTO_MIN_BITS 1
#define PROTO_MAX_BITS 8
#define PROTO_CMD_MODE 0x00
#define PROTO_CMD_LEN 2
#define PROTO_CMD_BRIGHTNESS 0x01
#define PROTO_PANEL_LINES 8
#define PROTO_COLS 16
#define PROTO_PLANE_BYTES (PROTO_PANEL_LINES * PROTO_COLS * 2)  // 256
#define PROTO_HEADER_SIZE 4
#define PROTO_MAX_PAYLOAD (PROTO_MAX_BITS * PROTO_PLANE_BYTES)  // 2048
#define PROTO_MAX_PACKET (PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD)  // 2052

// Receive ring: holds several max-size COBS frames incl. overhead.
#define PROTO_RING_SIZE 8192

static inline size_t proto_payload_len(uint8_t bits) {
  return (size_t)bits * PROTO_PLANE_BYTES;
}

// Validate raw (post-COBS) packet. Returns payload length or 0 on error,
// sets *bits_out on success.
static inline size_t proto_validate(const uint8_t *raw, size_t len,
                                    uint8_t *bits_out) {
  if (len < PROTO_HEADER_SIZE) return 0;
  if (raw[0] != PROTO_MAGIC) return 0;
  uint8_t bits = raw[1];
  if (bits == PROTO_CMD_MODE) {
    size_t declared = (size_t)raw[2] | ((size_t)raw[3] << 8);
    if (declared != PROTO_CMD_LEN) return 0;
    if (len != PROTO_HEADER_SIZE + declared) return 0;
    if (bits_out) *bits_out = bits;
    return declared;
  }
  if (bits < PROTO_MIN_BITS || bits > PROTO_MAX_BITS) return 0;
  size_t declared = (size_t)raw[2] | ((size_t)raw[3] << 8);
  if (declared != proto_payload_len(bits)) return 0;
  if (len != PROTO_HEADER_SIZE + declared) return 0;
  if (bits_out) *bits_out = bits;
  return declared;
}
