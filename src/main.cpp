/**
 * LED Matrix Firmware for RP2040 (Arduino framework, Earle Philhower core)
 *
 * Protocol v3: Fixed 5B header + Raw payload + COBS + 0x00 delimiter.
 *   header: [MAGIC 0x55][CLASS|DEPTH][LEN_L][LEN_H][FPS 0..255]
 *   payload: DEPTH planes x 256B, plane0=LSB first, each plane is
 *            matrix_buffer[8][16] uint16LE (legacy layout).
 *   FPS 0 = swap at next V-Sync (non-blocking); >0 = paced presentation.
 *   Control: [MAGIC][0x1|SUB][02 00][00] + [SUB, VAL] (1=brightness,
 *            2=clear queue + blank). No Base64. No compression. No legacy.
 *
 * Core0 (setup/loop): USB CDC receive, COBS decode, header validate.
 *   FPS=0 frames expand immediately to the back stream buffer; FPS>0
 *   frames go through a packet queue (drop-oldest when full) and are
 *   prefetched to the back buffer whenever it is free.
 * Core1 (setup1/loop1): feed the word stream to PIO via DMA, STROBE
 *   blanking for BAM timing, pointer swap at V-Sync boundary gated by
 *   the requested presentation period. Flow state (q depth, drops) is
 *   reported in +HB for host-side detection.
 * LATCH pulse is generated inside the PIO program (no CPU involved).
 * UF2 upload: 1200bps touch handled by Arduino USB stack (do nothing).
 */

#include <Arduino.h>
#include <string.h>

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/sio.h"
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
static volatile uint8_t stream_bits[2] = {1, 1};
// Bitmask of non-empty planes per buffer (Core0 sets, Core1 reads).
// Empty planes are skipped entirely: no shift, no on-time.
static volatile uint8_t plane_mask[2] = {0xFF, 0xFF};
static volatile uint8_t stream_front = 0;
static volatile uint8_t stream_pending = 0;
static volatile bool frame_ready = false;
// Requested fps of the pending / front frames (0 = immediate).
static volatile uint8_t pending_fps = 0;
static uint8_t front_fps = 0;
static uint32_t present_start_us = 0;  // micros() at last swap (wrap-safe)

// Paced-frame packet queue. Core0 owns all indices (Core1 never touches
// the queue; it only sees prefetched stream buffers), so no lock needed.
struct queued_pkt {
  uint8_t bits;
  uint8_t fps;
  uint8_t data[PROTO_MAX_PAYLOAD];
};
static queued_pkt pkt_queue[PROTO_QUEUE_DEPTH];
static uint8_t q_head = 0;
static uint8_t q_tail = 0;
static uint32_t q_drops = 0;

static inline uint8_t q_depth() {
  return (uint8_t)((q_head - q_tail + PROTO_QUEUE_DEPTH) % PROTO_QUEUE_DEPTH);
}

// STROBE/OE via SIO single-cycle writes (digitalWrite ~1us each way).
#define STROBE_MASK (1u << PIN_STROBE)
static inline void strobe_blank() { sio_hw->gpio_set = STROBE_MASK; }  // HIGH=off
static inline void strobe_show() { sio_hw->gpio_clr = STROBE_MASK; }  // LOW=on

// Global brightness 0..255 applied to BAM on-times.
// Default 255 (100%): tonal rendering is handled entirely by the
// host-side gamma curve. Adjustable at runtime via brightness command.
#define DEFAULT_BRIGHTNESS 255
static volatile uint8_t brightness = DEFAULT_BRIGHTNESS;

// Diagnostics must NEVER block the RX core: the host may keep the port
// open without reading CDC TX, or keep it closed entirely. Unconditional
// Serial prints fill the TX buffer, println blocks, Core0 stops draining
// RX, and the host hits write timeout. Gate every print on line state
// (host has the port open, DTR asserted) AND TX buffer space.
static inline bool dbg_can_write() {
  return Serial.dtr() && Serial.availableForWrite() > 64;
}

// Setup handshake: Core1 must not touch shared state until Core0's setup
// (buffer zeroing) is fully done. Eliminates setup races by construction.
static volatile bool setup_done = false;

// Stall-point instrumentation: each counter tells where its core was last
// seen. If the firmware wedges, the frozen values pinpoint the exact wait.
static volatile uint32_t dbg_loop_iters = 0;  // Core0 loop() passes
static volatile uint32_t dbg_l1_entries = 0;  // Core1 loop1() entries
static volatile uint8_t dbg_l1_stall = 0;  // 0=running,1=dma-wait,2=fifo,3=pc

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
  uint8_t mask = 0;
  for (uint8_t p = 0; p < bits; p++) {
    uint16_t acc = 0;
    for (uint8_t r = 0; r < ROWS; r++) {
      uint32_t *w = pio_stream[back][p][r];
      int k = 0;
      for (int panel = PANELS - 1; panel >= 0; panel--) {
        uint16_t sin2 = pl[p * 128 + (2 * panel + 0) * 16 + r];
        uint16_t sin3 = pl[p * 128 + (2 * panel + 1) * 16 + r];
        acc |= (uint16_t)(sin2 | sin3);
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
    if (acc) mask |= (uint8_t)(1u << p);
  }
  stream_bits[back] = bits;
  plane_mask[back] = mask;
}

// Scale an on-time by global brightness (0..255).
static inline uint32_t apply_brightness(uint32_t on_us) {
  uint8_t br = brightness;
  if (br == 255) return on_us;
  if (br == 0) return 0;
  uint32_t scaled = (on_us * br) / 255;
  return (scaled > 0) ? scaled : 1;
}

// Prefetch one queued packet into the back stream buffer (Core0).
static void prefetch_queue() {
  if (frame_ready || q_head == q_tail) return;
  queued_pkt *slot = &pkt_queue[q_tail];
  uint8_t back = 1 - stream_front;  // snapshot; swap happens only at V-Sync
  expand_packet(slot->data, slot->bits, back);
  stream_pending = back;
  pending_fps = slot->fps;
  q_tail = (uint8_t)((q_tail + 1) % PROTO_QUEUE_DEPTH);
  frame_ready = true;
}

static void on_packet(const uint8_t *raw, size_t len) {
  uint8_t bits = 0, fps = 0, sub = 0;
  if (proto_validate(raw, len, &bits, &fps, &sub) == 0) {
    dbg_err++;
    if (dbg_can_write()) Serial.println("-ERR validate");
    return;
  }
  if (bits == 0) {  // control packet
    if (sub == PROTO_SUB_BRIGHTNESS) {
      brightness = raw[PROTO_HEADER_SIZE + 1];
      if (dbg_can_write()) {
        Serial.print("+OK brightness ");
        Serial.println(brightness);
      }
    } else if (sub == PROTO_SUB_CLEAR) {
      q_head = q_tail = 0;
      uint8_t back = 1 - stream_front;
      memset(pio_stream[back], 0, sizeof(pio_stream[back]));
      stream_bits[back] = 1;
      plane_mask[back] = 0;
      stream_pending = back;
      pending_fps = 0;
      frame_ready = true;
    }
    dbg_ok++;
    return;
  }
  const uint8_t *payload = raw + PROTO_HEADER_SIZE;
  if (fps == 0) {
    // Immediate path (text/dashboard latency): overwrite the back buffer.
    uint8_t back = 1 - stream_front;
    expand_packet(payload, bits, back);
    stream_pending = back;
    pending_fps = 0;
    frame_ready = true;
  } else {
    // Paced path: queue, drop oldest when full (keeps latency low).
    uint8_t next = (uint8_t)((q_head + 1) % PROTO_QUEUE_DEPTH);
    if (next == q_tail) {
      q_tail = (uint8_t)((q_tail + 1) % PROTO_QUEUE_DEPTH);
      q_drops++;
    }
    queued_pkt *slot = &pkt_queue[q_head];
    slot->bits = bits;
    slot->fps = fps;
    memcpy(slot->data, payload, proto_payload_len(bits));
    q_head = next;
  }
  dbg_ok++;
  // NOTE: no per-packet +OK print. At video rates it floods TX and would
  // stall Core0 when the host does not read. Watch +HB ok= instead.
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
  strobe_blank();  // blank during shift (single-cycle SIO)
  dma_channel_set_read_addr(dma_ch, pio_stream[stream_front][plane][row], true);
  dbg_l1_stall = 1;
  dma_channel_wait_for_finish_blocking(dma_ch);
  // DMA done = words in FIFO; wait until PIO consumed + auto-latched.
  // The SM stalls on the `pull` instruction (program index 2, see the
  // pioasm disassembly) once all 64 words are consumed and LATCH fired.
  // NOTE: poll for the pull address, NOT the wrap target: the wrap target
  // is passed transiently and is unobservable at fast PIO clocks.
  dbg_l1_stall = 2;
  while (!pio_sm_is_tx_fifo_empty(pio_inst, pio_sm)) {
    tight_loop_contents();
  }
  dbg_l1_stall = 3;
  while (pio_sm_get_pc(pio_inst, pio_sm) != (pio_offset + 2)) {
    tight_loop_contents();
  }
  dbg_l1_stall = 0;
  strobe_show();  // display latched row
}

void setup() {
  Serial.begin(921600);
  memset(pio_stream, 0, sizeof(pio_stream));
  memset(ring_buf, 0, sizeof(ring_buf));
  // 4s watchdog: reboots on Core0 wedge (self-heal + visible flap).
  // Fed in loop() only, so a Core0 stall always recovers by reset.
  rp2040.wdt_begin(4000);
  setup_done = true;
}

void setup1() {
  // Wait for Core0 setup (buffer zeroing) before touching shared state.
  uint32_t t0 = millis();
  while (!setup_done && (millis() - t0) < 2000) {
    tight_loop_contents();
  }
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
  rp2040.wdt_reset();
  dbg_loop_iters++;
  if (!dbg_boot_sent && millis() > 1500) {
    dbg_boot_sent = true;
    if (dbg_can_write()) Serial.println("+BOOT cobs8dma");
  }
  drain_serial();
  prefetch_queue();
  unsigned long now = millis();
  if (now - dbg_last_hb >= 2000) {
    dbg_last_hb = now;
    if (!dbg_can_write()) return;
    Serial.print("+HB ok=");
    Serial.print(dbg_ok);
    Serial.print(" err=");
    Serial.print(dbg_err);
    Serial.print(" rx=");
    Serial.print(dbg_bytes);
    Serial.print(" bits=");
    Serial.print(stream_bits[stream_front]);
    Serial.print(" rows=");
    Serial.print((unsigned long)dbg_rows);
    Serial.print(" q=");
    Serial.print(q_depth());
    Serial.print(" drop=");
    Serial.print(q_drops);
    Serial.print(" it=");
    Serial.print((unsigned long)dbg_loop_iters);
    Serial.print(" e1=");
    Serial.print((unsigned long)dbg_l1_entries);
    Serial.print(" st=");
    Serial.println((unsigned long)dbg_l1_stall);
  }
}

void loop1() {
  dbg_l1_entries++;
  // V-Sync swap: pointer flip only, at BAM cycle boundary.
  // Paced frames (fps>0) wait until the current frame's period expires;
  // immediate frames (fps=0) swap right away. A pending frame stays
  // pending (back buffer stays busy) until its turn.
  if (frame_ready) {
    uint8_t n = stream_bits[stream_pending];
    uint8_t pf = pending_fps;
    bool due = (pf == 0) ||
               ((uint32_t)(micros() - present_start_us) >= 1000000u / pf);
    if (n >= PROTO_MIN_BITS && n <= PROTO_MAX_BITS && due) {
      stream_front = stream_pending;
      front_fps = pf;
      present_start_us = micros();
      // A fully empty frame shows nothing: blank output immediately so
      // no stale latched image persists (skip paths never touch STROBE).
      if (plane_mask[stream_front] == 0) strobe_blank();
      frame_ready = false;
    } else if (n < PROTO_MIN_BITS || n > PROTO_MAX_BITS) {
      frame_ready = false;  // corrupt pending state, drop it
    }
  }

  uint8_t n = stream_bits[stream_front];
  if (n < PROTO_MIN_BITS || n > PROTO_MAX_BITS) n = 1;

  uint8_t mask = plane_mask[stream_front];

  // 1-bit mode: always maximum brightness, bypasses brightness scaling.
  if (n == 1) {
    if (!(mask & 0x01)) return;  // empty frame: stay dark, save time
    for (uint8_t row = 0; row < ROWS; row++) {
      show_row_dma(0, row);
      delayMicroseconds(BINARY_ON_US);
      dbg_rows++;
    }
    return;
  }

  for (uint8_t plane = 0; plane < n; plane++) {
    if (!(mask & (1u << plane))) continue;  // empty plane: skip entirely
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
