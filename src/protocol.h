#pragma once
// Binary frame protocol v3 (COBS, no legacy).
// See led_matrix_software/protocol.py (single source of truth duplicated here).
//
// Wire: COBS(packet) + 0x00.
// Frame: [MAGIC][CLASS=0|DEPTH 1..8][LEN_L][LEN_H][FPS 0..255] + DEPTH*256B.
//   FPS 0 = swap in at next V-Sync (non-blocking); >0 = paced presentation.
// Control: [MAGIC][CLASS=1|SUB][02 00][00] + [SUB, VAL].
//   SUB 1 = brightness, SUB 2 = clear queue + blank.
#include <stdint.h>
#include <stddef.h>

#define PROTO_MAGIC 0x55
#define PROTO_DELIMITER 0x00
#define PROTO_MIN_BITS 1
#define PROTO_MAX_BITS 8
#define PROTO_FRAME_CLASS 0x0
#define PROTO_CTRL_CLASS 0x1
#define PROTO_SUB_BRIGHTNESS 0x01
#define PROTO_SUB_CLEAR 0x02
#define PROTO_PANEL_LINES 8
#define PROTO_COLS 16
#define PROTO_PLANE_BYTES (PROTO_PANEL_LINES * PROTO_COLS * 2)  // 256
#define PROTO_HEADER_SIZE 5
#define PROTO_CMD_LEN 2
#define PROTO_MAX_PAYLOAD (PROTO_MAX_BITS * PROTO_PLANE_BYTES)  // 2048

// Receive ring: holds several max-size COBS frames incl. overhead.
#define PROTO_RING_SIZE 8192

// Paced-frame packet queue (lock-free SPSC: Core0 pushes, Core1 pops
// indirectly via Core0 prefetch; head Core0-only, tail Core0-only because
// Core1 never touches the queue -- prefetch happens on Core0).
#define PROTO_QUEUE_DEPTH 8

static inline size_t proto_payload_len(uint8_t bits) {
  return (size_t)bits * PROTO_PLANE_BYTES;
}

// Validate raw (post-COBS) packet.
// Returns payload length or 0 on error. On success sets *bits_out to depth
// (1..8) or 0 for control, *fps_out to requested fps (frames) or 0,
// *sub_out to sub-command (control) or 0.
static inline size_t proto_validate(const uint8_t *raw, size_t len,
                                    uint8_t *bits_out, uint8_t *fps_out,
                                    uint8_t *sub_out) {
  if (len < PROTO_HEADER_SIZE) return 0;
  if (raw[0] != PROTO_MAGIC) return 0;
  uint8_t mode = raw[1];
  uint8_t cls = (mode >> 4) & 0xF;
  uint8_t sub = mode & 0xF;
  size_t declared = (size_t)raw[2] | ((size_t)raw[3] << 8);
  if (cls == PROTO_CTRL_CLASS) {
    if (sub != PROTO_SUB_BRIGHTNESS && sub != PROTO_SUB_CLEAR) return 0;
    if (declared != PROTO_CMD_LEN) return 0;
    if (len != PROTO_HEADER_SIZE + declared) return 0;
    if (bits_out) *bits_out = 0;
    if (fps_out) *fps_out = 0;
    if (sub_out) *sub_out = sub;
    return declared;
  }
  if (cls != PROTO_FRAME_CLASS) return 0;
  if (sub < PROTO_MIN_BITS || sub > PROTO_MAX_BITS) return 0;
  if (declared != proto_payload_len(sub)) return 0;
  if (len != PROTO_HEADER_SIZE + declared) return 0;
  if (bits_out) *bits_out = sub;
  if (fps_out) *fps_out = raw[4];
  if (sub_out) *sub_out = 0;
  return declared;
}
