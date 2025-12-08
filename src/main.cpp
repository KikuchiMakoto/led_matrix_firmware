/**
 * LED Matrix Firmware for RP2040
 * Supports 8-bit brightness with Bit-Angle Modulation (BAM)
 * Multiple transfer modes for performance comparison
 *
 * Hardware: 74HC595 shift registers
 * Communication: USB CDC-ACM + Base64
 *
 * Protocol:
 * - 256 bytes (base64 encoded): 1-bit mode (compatible)
 * - 2048 bytes (base64 encoded): 8-bit brightness mode
 *
 * Transfer modes:
 * - TRANSFER_MODE 0: GPIO direct control
 * - TRANSFER_MODE 1: DMA + SPI
 * - TRANSFER_MODE 2: PIO
 */

// Select transfer mode (0: GPIO, 1: DMA, 2: PIO)
#ifndef TRANSFER_MODE
#define TRANSFER_MODE 0
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#if TRANSFER_MODE == 1
#include "hardware/spi.h"
#include "hardware/dma.h"
#endif

#if TRANSFER_MODE == 2
#include "hardware/pio.h"
#include "shift_out.pio.h"
#endif

// Pin definitions (fixed by hardware)
#define PIN_SIN_1  0  // Row select
#define PIN_SIN_2  1  // Panel data 1
#define PIN_SIN_3  2  // Panel data 2
#define PIN_CLOCK  3  // Shift clock
#define PIN_LATCH  4  // Latch
#define PIN_STROBE 5  // Strobe

// Display dimensions
#define ROWS 16
#define COLS 16
#define PANELS 4
#define PANEL_LINES 8  // 4 panels × 2 lines (sin2, sin3)

// Brightness levels
#define BRIGHTNESS_BITS 8
#define BRIGHTNESS_LEVELS (1 << BRIGHTNESS_BITS)

// Buffer sizes
#define BUFFER_1BIT_SIZE  256   // 8 panel_lines × 16 cols × 2 bytes = 256 bytes
#define BUFFER_8BIT_SIZE  2048  // 256 bytes × 8 bit planes = 2048 bytes
#define RECV_BUFFER_SIZE  4096
#define BASE64_BUFFER_SIZE 3072

// Display buffer: [panel_line][col][bit_plane]
// Compatible with original matrix_buffer[8][16] structure
// bit_plane 0 = LSB, bit_plane 7 = MSB
static uint16_t display_buffer[PANEL_LINES][COLS][BRIGHTNESS_BITS];
static uint16_t display_buffer_temp[PANEL_LINES][COLS][BRIGHTNESS_BITS];
static volatile bool buffer_ready = false;

// Reception buffer
static uint8_t recv_buffer[RECV_BUFFER_SIZE];
static uint16_t recv_pos = 0;

#if TRANSFER_MODE == 1
// DMA + SPI variables
static int dma_channel;
#endif

#if TRANSFER_MODE == 2
// PIO variables
static PIO pio = pio0;
static uint sm = 0;
#endif

// Base64 decode table
static const int8_t base64_decode_table[256] = {
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  // 0-15
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  // 16-31
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,62,-1,-1,-1,63,  // 32-47
    52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-1,-1,-1,  // 48-63
    -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,  // 64-79
    15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,-1,-1,  // 80-95
    -1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,  // 96-111
    41,42,43,44,45,46,47,48,49,50,51,-1,-1,-1,-1,-1,  // 112-127
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  // 128-143
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  // 144-159
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  // 160-175
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  // 176-191
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  // 192-207
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  // 208-223
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,  // 224-239
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1   // 240-255
};

/**
 * Base64 decoder
 * Returns: decoded length, or -1 on error
 */
static int base64_decode(const uint8_t* input, size_t input_len, uint8_t* output) {
    if (input_len % 4 != 0) return -1;

    size_t output_len = 0;

    for (size_t i = 0; i < input_len; i += 4) {
        int8_t a = base64_decode_table[input[i]];
        int8_t b = base64_decode_table[input[i + 1]];
        int8_t c = base64_decode_table[input[i + 2]];
        int8_t d = base64_decode_table[input[i + 3]];

        if (a == -1 || b == -1) return -1;

        output[output_len++] = (a << 2) | (b >> 4);

        if (input[i + 2] != '=') {
            if (c == -1) return -1;
            output[output_len++] = (b << 4) | (c >> 2);
        }

        if (input[i + 3] != '=') {
            if (d == -1) return -1;
            output[output_len++] = (c << 6) | d;
        }
    }

    return output_len;
}

#if TRANSFER_MODE == 0
/**
 * Mode 0: GPIO direct control
 * Shifts out 16 bits per panel (4 panels total)
 */
static inline void shift_out_row(uint8_t row, uint8_t bit_plane) {
    gpio_put(PIN_STROBE, 1);
    gpio_put(PIN_LATCH, 1);

    // For each panel (reversed order: panel 3, 2, 1, 0)
    for (int panel = PANELS - 1; panel >= 0; panel--) {
        // Get data for this panel and bit plane
        uint16_t sin2_data = display_buffer[2 * (3 - panel) + 0][row][bit_plane];
        uint16_t sin3_data = display_buffer[2 * (3 - panel) + 1][row][bit_plane];

        // Shift out 16 bits
        for (int bit = 0; bit < COLS; bit++) {
            gpio_put(PIN_CLOCK, 0);
            gpio_put(PIN_SIN_1, (row == bit) ? 1 : 0);
            gpio_put(PIN_SIN_2, (sin2_data & 1) ? 1 : 0);
            gpio_put(PIN_SIN_3, (sin3_data & 1) ? 1 : 0);
            gpio_put(PIN_CLOCK, 1);

            sin2_data >>= 1;
            sin3_data >>= 1;
        }
    }

    // Latch and strobe
    gpio_put(PIN_LATCH, 0);
    sleep_us(1);
    gpio_put(PIN_STROBE, 0);
    sleep_us(1);
}
#endif

#if TRANSFER_MODE == 1
/**
 * Mode 1: DMA + SPI control
 * Uses SPI with DMA for high-speed transfer
 */
static uint8_t spi_buffer[PANELS * COLS * 2];  // 2 bytes per bit (for 3 data lines)

static inline void shift_out_row(uint8_t row, uint8_t bit_plane) {
    gpio_put(PIN_STROBE, 1);
    gpio_put(PIN_LATCH, 1);

    // Prepare SPI buffer
    int buf_idx = 0;
    for (int panel = PANELS - 1; panel >= 0; panel--) {
        uint16_t sin2_data = display_buffer[2 * (3 - panel) + 0][row][bit_plane];
        uint16_t sin3_data = display_buffer[2 * (3 - panel) + 1][row][bit_plane];

        for (int bit = 0; bit < COLS; bit++) {
            uint8_t data_byte = 0;
            data_byte |= ((row == bit) ? 1 : 0) << 0;  // SIN_1
            data_byte |= ((sin2_data & 1) ? 1 : 0) << 1;  // SIN_2
            data_byte |= ((sin3_data & 1) ? 1 : 0) << 2;  // SIN_3

            spi_buffer[buf_idx++] = data_byte;

            sin2_data >>= 1;
            sin3_data >>= 1;
        }
    }

    // Transfer via SPI with DMA
    dma_channel_wait_for_finish_blocking(dma_channel);
    dma_channel_set_read_addr(dma_channel, spi_buffer, false);
    dma_channel_set_trans_count(dma_channel, buf_idx, true);
    dma_channel_wait_for_finish_blocking(dma_channel);

    // Latch and strobe
    gpio_put(PIN_LATCH, 0);
    sleep_us(1);
    gpio_put(PIN_STROBE, 0);
    sleep_us(1);
}
#endif

#if TRANSFER_MODE == 2
/**
 * Mode 2: PIO control
 * Uses PIO state machine for hardware-accelerated shift output
 */
static inline void shift_out_row(uint8_t row, uint8_t bit_plane) {
    gpio_put(PIN_STROBE, 1);
    gpio_put(PIN_LATCH, 1);

    // For each panel (reversed order: panel 3, 2, 1, 0)
    for (int panel = PANELS - 1; panel >= 0; panel--) {
        uint16_t sin2_data = display_buffer[2 * (3 - panel) + 0][row][bit_plane];
        uint16_t sin3_data = display_buffer[2 * (3 - panel) + 1][row][bit_plane];

        // Shift out 16 bits via PIO
        for (int bit = 0; bit < COLS; bit++) {
            uint32_t pio_data = 0;
            pio_data |= ((row == bit) ? 1 : 0) << 0;  // SIN_1
            pio_data |= ((sin2_data & 1) ? 1 : 0) << 1;  // SIN_2
            pio_data |= ((sin3_data & 1) ? 1 : 0) << 2;  // SIN_3

            pio_sm_put_blocking(pio, sm, pio_data);

            sin2_data >>= 1;
            sin3_data >>= 1;
        }
    }

    // Wait for PIO to finish
    while (!pio_sm_is_tx_fifo_empty(pio, sm)) {
        tight_loop_contents();
    }

    // Latch and strobe
    gpio_put(PIN_LATCH, 0);
    sleep_us(1);
    gpio_put(PIN_STROBE, 0);
    sleep_us(1);
}
#endif

/**
 * Core 1: Display update with Bit-Angle Modulation (BAM)
 */
void core1_display_task() {
#if TRANSFER_MODE == 0
    // Mode 0: GPIO direct control initialization
    gpio_init(PIN_SIN_1);
    gpio_init(PIN_SIN_2);
    gpio_init(PIN_SIN_3);
    gpio_init(PIN_CLOCK);
    gpio_init(PIN_LATCH);
    gpio_init(PIN_STROBE);

    gpio_set_dir(PIN_SIN_1, GPIO_OUT);
    gpio_set_dir(PIN_SIN_2, GPIO_OUT);
    gpio_set_dir(PIN_SIN_3, GPIO_OUT);
    gpio_set_dir(PIN_CLOCK, GPIO_OUT);
    gpio_set_dir(PIN_LATCH, GPIO_OUT);
    gpio_set_dir(PIN_STROBE, GPIO_OUT);

    gpio_put(PIN_STROBE, 1);
#endif

#if TRANSFER_MODE == 1
    // Mode 1: DMA + SPI initialization
    gpio_init(PIN_LATCH);
    gpio_init(PIN_STROBE);
    gpio_set_dir(PIN_LATCH, GPIO_OUT);
    gpio_set_dir(PIN_STROBE, GPIO_OUT);
    gpio_put(PIN_STROBE, 1);

    // Initialize SPI0 at 10 MHz
    spi_init(spi0, 10 * 1000 * 1000);
    gpio_set_function(PIN_SIN_2, GPIO_FUNC_SPI);  // MOSI (use for combined data)
    gpio_set_function(PIN_CLOCK, GPIO_FUNC_SPI);  // SCK

    // Setup DMA
    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, spi_get_dreq(spi0, true));
    dma_channel_configure(dma_channel, &c, &spi_get_hw(spi0)->dr, spi_buffer, 0, false);
#endif

#if TRANSFER_MODE == 2
    // Mode 2: PIO initialization
    gpio_init(PIN_LATCH);
    gpio_init(PIN_STROBE);
    gpio_set_dir(PIN_LATCH, GPIO_OUT);
    gpio_set_dir(PIN_STROBE, GPIO_OUT);
    gpio_put(PIN_STROBE, 1);

    // Load PIO program
    uint offset = pio_add_program(pio, &shift_out_program);
    shift_out_program_init(pio, sm, offset, PIN_SIN_1, PIN_CLOCK);
#endif

    // Main display loop with BAM
    while (true) {
        // Update display buffer if new data is ready
        if (buffer_ready) {
            memcpy(display_buffer, display_buffer_temp, sizeof(display_buffer));
            buffer_ready = false;
        }

        // Bit-Angle Modulation: display each bit plane with weighted time
        for (uint8_t bit_plane = 0; bit_plane < BRIGHTNESS_BITS; bit_plane++) {
            // Display time proportional to bit weight: 2^bit_plane
            uint32_t display_time = (1 << bit_plane);

            for (uint8_t row = 0; row < ROWS; row++) {
                shift_out_row(row, bit_plane);

                // Wait proportional to bit weight
                sleep_us(display_time);
            }
        }
    }
}

/**
 * Process received data (base64 encoded)
 */
static void process_received_data(const uint8_t* data, size_t len) {
    static uint8_t decoded_buffer[BASE64_BUFFER_SIZE];

    // Decode base64
    int decoded_len = base64_decode(data, len, decoded_buffer);

    if (decoded_len == BUFFER_1BIT_SIZE) {
        // 1-bit mode: 256 bytes (compatible mode)
        // Data format matches original: matrix_buffer[8][16] as uint16_t
        // Copy directly and expand to all bit planes
        uint16_t* src = (uint16_t*)decoded_buffer;

        for (int panel_line = 0; panel_line < PANEL_LINES; panel_line++) {
            for (int col = 0; col < COLS; col++) {
                uint16_t data_1bit = src[panel_line * COLS + col];

                // Expand 1-bit to all bit planes (0x0000 or 0xFFFF)
                for (int bit_plane = 0; bit_plane < BRIGHTNESS_BITS; bit_plane++) {
                    display_buffer_temp[panel_line][col][bit_plane] = data_1bit;
                }
            }
        }
        buffer_ready = true;
    }
    else if (decoded_len == BUFFER_8BIT_SIZE) {
        // 8-bit mode: 2048 bytes
        // Data format: 8 bit planes of matrix_buffer[8][16]
        // Each bit plane is 256 bytes (128 uint16_t)
        uint16_t* src = (uint16_t*)decoded_buffer;

        for (int bit_plane = 0; bit_plane < BRIGHTNESS_BITS; bit_plane++) {
            for (int panel_line = 0; panel_line < PANEL_LINES; panel_line++) {
                for (int col = 0; col < COLS; col++) {
                    int idx = bit_plane * (PANEL_LINES * COLS) + panel_line * COLS + col;
                    display_buffer_temp[panel_line][col][bit_plane] = src[idx];
                }
            }
        }
        buffer_ready = true;
    }
    // else: invalid length, ignore
}

/**
 * Core 0: USB reception and data processing
 */
int main() {
    // Initialize USB CDC
    stdio_init_all();

    // Wait for USB connection
    sleep_ms(3000);

    // Clear buffers
    memset(display_buffer, 0, sizeof(display_buffer));
    memset(display_buffer_temp, 0, sizeof(display_buffer_temp));
    memset(recv_buffer, 0, sizeof(recv_buffer));

    // Start display task on Core 1
    multicore_launch_core1(core1_display_task);

    // Main reception loop
    while (true) {
        int c = getchar_timeout_us(1000);

        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\r') {
                // Ignore CR
                continue;
            }
            else if (c == '\n') {
                // End of frame
                if (recv_pos > 0) {
                    process_received_data(recv_buffer, recv_pos);
                    recv_pos = 0;
                    memset(recv_buffer, 0, sizeof(recv_buffer));
                }
            }
            else {
                // Store character
                if (recv_pos < RECV_BUFFER_SIZE - 1) {
                    recv_buffer[recv_pos++] = (uint8_t)c;
                }
            }
        }
    }

    return 0;
}
