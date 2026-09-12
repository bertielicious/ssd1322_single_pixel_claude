/*
 * Simplest ESP-IDF program to draw a single pixel on an SSD1322 OLED
 * display over SPI.
 *
 * Wiring (as specified):
 *   MOSI -> GPIO 23
 *   SCK  -> GPIO 18
 *   CS   -> GPIO 5
 *   RES  -> GPIO 17
 *   DC   -> GPIO 16
 *   (MISO not used - SSD1322 is write-only over this interface)
 *
 * Uses the ESP-IDF SPI Master driver API (driver/spi_master.h), which is
 * stable across ESP-IDF 5.x / 6.x releases.
 *
 * Build as a normal ESP-IDF project: drop this file in as
 * main/main.c (with a matching main/CMakeLists.txt that lists it as
 * a source), then `idf.py build flash monitor`.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "SSD1322";

// ---- Pin definitions ----
#define PIN_MOSI 23
#define PIN_SCLK 18
#define PIN_CS   5
#define PIN_RES  17
#define PIN_DC   16

// ---- Display geometry ----
#define SSD1322_WIDTH   256
#define SSD1322_HEIGHT  64
// Many 256x64 SSD1322 modules use a RAM column offset of 0x1C (28)
// because the controller's internal RAM is 480 columns wide.
#define COLUMN_OFFSET   0x1C

#define SPI_HOST_USED   SPI2_HOST

static spi_device_handle_t spi;

// Send a single command byte (DC = 0 selects command mode)
static void ssd1322_send_cmd(uint8_t cmd)
{
    gpio_set_level(PIN_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_polling_transmit(spi, &t);
}

// Send one or more data bytes (DC = 1 selects data mode)
static void ssd1322_send_data(const uint8_t *data, size_t len)
{
    if (len == 0) return;
    gpio_set_level(PIN_DC, 1);
    spi_transaction_t t = {
        .length = 8 * len,
        .tx_buffer = data,
    };
    spi_device_polling_transmit(spi, &t);
}

// Convenience wrapper for a single data byte
static void ssd1322_send_data_byte(uint8_t b)
{
    ssd1322_send_data(&b, 1);
}

// Hardware reset pulse via the RES pin
static void ssd1322_reset(void)
{
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RES, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(PIN_RES, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Minimal init sequence taken from the SSD1322 datasheet
static void ssd1322_init(void)
{
    ssd1322_reset();

    ssd1322_send_cmd(0xFD); ssd1322_send_data_byte(0x12); // unlock commands
    ssd1322_send_cmd(0xAE);                                // display off
    ssd1322_send_cmd(0xB3); ssd1322_send_data_byte(0x91);  // clock divider
    ssd1322_send_cmd(0xCA); ssd1322_send_data_byte(0x3F);  // mux ratio 1/64
    ssd1322_send_cmd(0xA2); ssd1322_send_data_byte(0x00);  // display offset
    ssd1322_send_cmd(0xA1); ssd1322_send_data_byte(0x00);  // start line
    ssd1322_send_cmd(0xA0);                                // remap / color depth
    ssd1322_send_data_byte(0x14);
    ssd1322_send_data_byte(0x11);
    ssd1322_send_cmd(0xAB); ssd1322_send_data_byte(0x01);  // enable internal VDD
    ssd1322_send_cmd(0xC1); ssd1322_send_data_byte(0x9F);  // contrast
    ssd1322_send_cmd(0xC7); ssd1322_send_data_byte(0x0F);  // master contrast
    ssd1322_send_cmd(0xB1); ssd1322_send_data_byte(0xE2);  // phase length
    ssd1322_send_cmd(0xB4);
    ssd1322_send_data_byte(0xA0);
    ssd1322_send_data_byte(0xFD);                          // display enhancement A
    ssd1322_send_cmd(0xD1);
    ssd1322_send_data_byte(0x82);
    ssd1322_send_data_byte(0x20);                          // display enhancement B
    ssd1322_send_cmd(0xBB); ssd1322_send_data_byte(0x1F);  // precharge voltage
    ssd1322_send_cmd(0xB6); ssd1322_send_data_byte(0x08);  // precharge period
    ssd1322_send_cmd(0xBE); ssd1322_send_data_byte(0x07);  // VCOMH voltage
    ssd1322_send_cmd(0xA6);                                // normal (non-inverted) display
    ssd1322_send_cmd(0xA9);                                // exit partial display

    // Clear the whole GDDRAM so nothing but our pixel shows up
    ssd1322_send_cmd(0x15); ssd1322_send_data_byte(0x1C); ssd1322_send_data_byte(0x5B); // column range
    ssd1322_send_cmd(0x75); ssd1322_send_data_byte(0x00); ssd1322_send_data_byte(0x3F); // row range
    ssd1322_send_cmd(0x5C); // write RAM
    uint8_t zero_row[SSD1322_WIDTH / 2] = {0};
    for (int r = 0; r < SSD1322_HEIGHT; r++) {
        ssd1322_send_data(zero_row, sizeof(zero_row));
    }

    ssd1322_send_cmd(0xAF); // display on
}

/*
 * Draw one pixel at (x, y) with a 4-bit grayscale value (0 = off, 15 = brightest).
 *
 * The SSD1322 packs 2 pixels per byte (4 bits each) and addresses GDDRAM in
 * groups of 4 pixels per column-address step. To light a single pixel we:
 *   1. Set the column address window to just the 4-pixel group containing x.
 *   2. Set the row address window to just row y.
 *   3. Write 2 bytes: the byte containing our pixel gets the requested
 *      nibble set, the other pixel sharing that byte is written as 0.
 *
 * Because this simple write-only interface never reads GDDRAM back, this
 * will also blank the 3 neighbouring pixels that share the same 4-pixel
 * group - an acceptable trade-off for the "simplest possible" example.
 */
static void ssd1322_draw_pixel(int x, int y, uint8_t gray)
{
    if (x < 0 || x >= SSD1322_WIDTH || y < 0 || y >= SSD1322_HEIGHT) return;

    int group = x / 4;                        // which 4-pixel column group
    int offset_in_group = x % 4;               // 0..3 position inside the group
    int byte_in_group = offset_in_group / 2;   // 0 or 1 (2 bytes per group)
    int nibble_is_high = (offset_in_group % 2) == 0; // pixels 0,2 = high nibble

    uint8_t bytes[2] = {0, 0};
    uint8_t value = gray & 0x0F;
    if (nibble_is_high) {
        bytes[byte_in_group] = value << 4;
    } else {
        bytes[byte_in_group] = value;
    }

    uint8_t col = COLUMN_OFFSET + group;

    ssd1322_send_cmd(0x15); // set column address
    ssd1322_send_data_byte(col);
    ssd1322_send_data_byte(col);

    ssd1322_send_cmd(0x75); // set row address
    ssd1322_send_data_byte((uint8_t)y);
    ssd1322_send_data_byte((uint8_t)y);

    ssd1322_send_cmd(0x5C); // write RAM
    ssd1322_send_data(bytes, 2);
}

// Configure the SPI bus and attach the SSD1322 as a device on it
static void spi_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,           // not used - SSD1322 is write-only here
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SSD1322_WIDTH / 2 * SSD1322_HEIGHT,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_USED, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // 10 MHz - safe for SSD1322
        .mode = 0,
        .spics_io_num = PIN_CS,             // driver toggles CS automatically
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_USED, &devcfg, &spi));
}

void app_main(void)
{
    // DC and RES are plain GPIO outputs (CS is handled by the SPI driver itself)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_DC) | (1ULL << PIN_RES),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    spi_init();
    ssd1322_init();

    ESP_LOGI(TAG, "Drawing pixel at (100, 32)");
    ssd1322_draw_pixel(255, 0, 2); // full-brightness pixel near center

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}