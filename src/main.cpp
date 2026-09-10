/**
 * LED Matrix Firmware for RP2040 (Arduino framework, Earle Philhower core)
 *
 * Protocol v2: Fixed 4B header + Raw payload + COBS + 0x00 delimiter.
 *   header: [MAGIC 0x55][MODE N=1..8][LEN_L][LEN_H], LEN = N*256
 *   payload: N planes x 256B, plane0=LSB first, each plane is
 *            matrix_buffer[8][16] uint16LE (legacy layout).
 * No Base64. No compression.
 *
 * Core0 (setup/loop): USB CDC receive, COBS decode, header validate,
 *   then EXPAND planes into a PIO-ready word stream (ping-pong buffer).
 * Core1 (setup1/loop1): feed the word stream to PIO via DMA, STROBE
 *   blanking for BAM timing, pointer swap at V-Sync boundary.
 * LATCH pulse is generated inside the PIO program (no CPU involved).
 * UF2 upload: 1200bps touch handled by Arduino USB stack (do nothing).
 */

#include <Arduino.h>
#include <string.h>

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "shift_out.pio.h"

#include "protocol.h"
#include "cobs.h"

// Pin definitions (fixed by hardware)
#define PIN_SIN_1 0  // Row select (PIO OUT)
#define PIN_SIN_2 1  // Panel data 1 (PIO OUT)
#define PIN_SIN_3 2  // Panel data 2 (PIO OUT)
#define PIN_CLOCK 3  // Shift clock (PIO sideset)
#define PIN_LATCH 4  // Latch (PIO SET, auto-pulsed per row)
#define PIN_STROBE 5  // Strobe / OE, GPIO (LOW = display on)

// Display dimensions
#define ROWS 16
#define COLS 16
#define PANELS 4
#define WORDS_PER_ROW (PANELS * COLS)  // 64 pixels, 1 word each

// BAM timing (tunable, verify on real LED)
// LSB unit; plane p on-time = BAM_UNIT_US << p. Shift happens while
// STROBE=HIGH (blanked), so LSB is never buried in shift time.
// 8bit full cycle ~= 16 rows x (shift + 255*UNIT): UNIT=10 -> ~41ms
// (24Hz flicker), UNIT=2 -> ~8.5ms (~118Hz, flicker-free target).
#define BAM_UNIT_US 2
// STROBE toggle + loop overhead added to EVERY plane window (~3us,
// measured via rows/s counter). Subtract it so LSB weights stay linear.
// Without this, plane0 (ideal 2us) would display ~5us (2.5x too bright),
// lifting darks and flattening highlights by comparison.
#define STROBE_OH_US 3
#define BAM_MIN_ON_US 2
// 1-bit mode fixed on-time per row
#define BINARY_ON_US 400

// PIO word stream, ping-pong: [buf][plane][row][pixel-word]
// 2 x 8 x 16 x 64 x 4B = 64KB (RAM 264KB, plenty left)
static uint32_t pio_stream[2][PROTO_MAX_BITS][ROWS][WORDS_PER_ROW];
static uint8_t stream_bits[2] = {1, 1};
static volatile uint8_t stream_front = 0;
static volatile uint8_t stream_pending = 0;
static volatile bool frame_ready = false;

// Global brightness 0..255 applied to BAM on-times.
// Default 255 (100%): tonal rendering is handled entirely by the
// host-side gamma curve. Adjustable at runtime via brightness command.
#define DEFAULT_BRIGHTNESS 255
static volatile uint8_t brightness = DEFAULT_BRIGHTNESS;

// Diagnostics (Core0 writes; loop1 writes dbg_rows only)
static uint32_t dbg_ok = 0;
static uint32_t dbg_err = 0;
static uint32_t dbg_bytes = 0;
static unsigned long dbg_last_hb = 0;
static bool dbg_boot_sent = false;
static volatile uint32_t dbg_rows = 0;

// Receive ring (Core0 only)
static uint8_t ring_buf[PROTO_RING_SIZE];
static size_t ring_pos = 0;

static PIO pio_inst = pio0;
static uint pio_sm = 0;
static uint pio_offset = 0;
static int dma_ch = -1;

// Expand one validated packet into the back word-stream buffer.
// Chain order (verified on real LED): first-shifted word travels farthest,
// panel 3 is shifted first -> must carry the rightmost groups.
// With LATCH auto-pulsed right after the row, outputs follow immediately.
static void expand_packet(const uint8_t *payload, uint8_t bits, uint8_t back) {
  const uint16_t *pl = (const uint16_t *)payload;  // [plane][line][col]
  for (uint8_t p = 0; p < bits; p++) {
    for (uint8_t r = 0; r < ROWS; r++) {
      uint32_t *w = pio_stream[back][p][r];
      int k = 0;
      for (int panel = PANELS - 1; panel >= 0; panel--) {
        uint16_t sin2 = pl[p * 128 + (2 * panel + 0) * 16 + r];
        uint16_t sin3 = pl[p * 128 + (2 * panel + 1) * 16 + r];
        for (int bit = 0; bit < COLS; bit++) {
          uint32_t word = 0;
          if (r == (uint8_t)bit) word |= 1u << 0;
          if (sin2 & 1u) word |= 1u << 1;
          if (sin3 & 1u) word |= 1u << 2;
          w[k++] = word;
          sin2 >>= 1;
          sin3 >>= 1;
        }
      }
    }
  }
  stream_bits[back] = bits;
}

// Scale an on-time by global brightness (0..255).
static inline uint32_t apply_brightness(uint32_t on_us) {
  uint8_t br = brightness;
  if (br == 255) return on_us;
  if (br == 0) return 0;
  uint32_t scaled = (on_us * br) / 255;
  return (scaled > 0) ? scaled : 1;
}

static void on_packet(const uint8_t *raw, size_t len) {
  uint8_t bits = 0;
  if (proto_validate(raw, len, &bits) == 0) {
    dbg_err++;
    Serial.println("-ERR validate");
    return;
  }
  if (bits == PROTO_CMD_MODE) {
    if (raw[PROTO_HEADER_SIZE] == PROTO_CMD_BRIGHTNESS) {
      brightness = raw[PROTO_HEADER_SIZE + 1];
      Serial.print("+OK brightness ");
      Serial.println(brightness);
    }
    dbg_ok++;
    return;
  }
  uint8_t back = 1 - stream_front;  // snapshot; swap happens only at V-Sync
  expand_packet(raw + PROTO_HEADER_SIZE, bits, back);
  stream_pending = back;
  frame_ready = true;
  dbg_ok++;
  Serial.print("+OK ");
  Serial.println(bits);
}

static void drain_serial() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    uint8_t b = (uint8_t)c;
    if (b == PROTO_DELIMITER) {
      if (ring_pos > 0) {
        size_t decoded = cobs_decode_inplace(ring_buf, ring_pos);
        if (decoded > 0) on_packet(ring_buf, decoded);
        ring_pos = 0;
      }
    } else {
      if (ring_pos < sizeof(ring_buf)) {
        ring_buf[ring_pos++] = b;
        dbg_bytes++;
      } else {
        ring_pos = 0;  // overflow: drop frame, resync at next 0x00
      }
    }
  }
}

// Show one row: DMA 64 words into PIO, wait until PIO latched them,
// then STROBE on for the caller's on-time (caller manages blanking).
static inline void show_row_dma(uint8_t plane, uint8_t row) {
  digitalWrite(PIN_STROBE, HIGH);  // blank during shift
  dma_channel_set_read_addr(dma_ch, pio_stream[stream_front][plane][row], true);
  dma_channel_wait_for_finish_blocking(dma_ch);
  // DMA done = words in FIFO; wait until PIO consumed + auto-latched.
  // The SM stalls on the `pull` instruction (program index 2, see the
  // pioasm disassembly) once all 64 words are consumed and LATCH fired.
  // NOTE: poll for the pull address, NOT the wrap target: the wrap target
  // is passed transiently and is unobservable at fast PIO clocks.
  while (!pio_sm_is_tx_fifo_empty(pio_inst, pio_sm)) {
    tight_loop_contents();
  }
  while (pio_sm_get_pc(pio_inst, pio_sm) != (pio_offset + 2)) {
    tight_loop_contents();
  }
  digitalWrite(PIN_STROBE, LOW);  // display latched row
}

void setup() {
  Serial.begin(921600);
  memset(pio_stream, 0, sizeof(pio_stream));
  memset(ring_buf, 0, sizeof(ring_buf));
}

void setup1() {
  pinMode(PIN_STROBE, OUTPUT);
  digitalWrite(PIN_STROBE, HIGH);
  // NOTE: LATCH is PIO-driven (SET pin); do not pinMode it as GPIO.

  pio_offset = pio_add_program(pio_inst, &shift_out_program);
  shift_out_program_init(pio_inst, pio_sm, pio_offset, PIN_SIN_1, PIN_CLOCK,
                         PIN_LATCH);

  dma_ch = dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(dma_ch);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
  channel_config_set_read_increment(&cfg, true);
  channel_config_set_write_increment(&cfg, false);
  channel_config_set_dreq(&cfg, pio_get_dreq(pio_inst, pio_sm, true));
  dma_channel_configure(dma_ch, &cfg, &pio_inst->txf[pio_sm], NULL,
                        WORDS_PER_ROW, false);
}

void loop() {
  if (!dbg_boot_sent && millis() > 1500) {
    dbg_boot_sent = true;
    Serial.println("+BOOT cobs8dma");
  }
  drain_serial();
  unsigned long now = millis();
  if (now - dbg_last_hb >= 2000) {
    dbg_last_hb = now;
    Serial.print("+HB ok=");
    Serial.print(dbg_ok);
    Serial.print(" err=");
    Serial.print(dbg_err);
    Serial.print(" rx=");
    Serial.print(dbg_bytes);
    Serial.print(" bits=");
    Serial.print(stream_bits[stream_front]);
    Serial.print(" rows=");
    Serial.println((unsigned long)dbg_rows);
  }
}

void loop1() {
  // V-Sync swap: pointer flip only, at BAM cycle boundary
  if (frame_ready) {
    uint8_t n = stream_bits[stream_pending];
    if (n >= PROTO_MIN_BITS && n <= PROTO_MAX_BITS) {
      stream_front = stream_pending;
    }
    frame_ready = false;
  }

  uint8_t n = stream_bits[stream_front];
  if (n < PROTO_MIN_BITS || n > PROTO_MAX_BITS) n = 1;

  // 1-bit mode: always maximum brightness, bypasses brightness scaling.
  if (n == 1) {
    for (uint8_t row = 0; row < ROWS; row++) {
      show_row_dma(0, row);
      delayMicroseconds(BINARY_ON_US);
      dbg_rows++;
    }
    return;
  }

  for (uint8_t plane = 0; plane < n; plane++) {
    uint32_t ideal = (uint32_t)BAM_UNIT_US << plane;
    uint32_t on_us =
        (ideal > STROBE_OH_US) ? (ideal - STROBE_OH_US) : BAM_MIN_ON_US;
    on_us = apply_brightness(on_us);
    for (uint8_t row = 0; row < ROWS; row++) {
      show_row_dma(0 + plane, row);
      delayMicroseconds(on_us);
      dbg_rows++;
    }
  }
}
