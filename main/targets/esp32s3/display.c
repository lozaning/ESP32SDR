/* Waveform display for the LILYGO T-Dongle-S3 (0.96" 160x80 ST7735S).
 *
 * Uses LILYGO's own esp_lcd_st7735 vendor driver + the ESP-IDF esp_lcd panel
 * API, configured exactly like their reference example (examples/lcd/lcd.ino):
 * SPI2, invert on, swap_xy, gap (1,26), mirror(false,true), backlight
 * active-low on GPIO38. We render into an RGB565 framebuffer and blit the
 * whole frame. Non-fatal on any error so the RF/serial path is never affected.
 */
#include "display.h"
#include <string.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7735.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---- T-Dongle-S3 pin map (LILYGO reference) ---- */
#define LCD_HOST     SPI2_HOST
#define PIN_MOSI     3
#define PIN_CLK      5
#define PIN_CS       4
#define PIN_DC       2
#define PIN_RST      1
#define PIN_BL       38          /* backlight: active-LOW (0 = on) */

#define TFT_W        160
#define TFT_H        80

/* RGB565, byte-swapped to the panel's little-endian order (matches the
 * reference: green == 0xE007). BGR + invert are handled by the driver. */
#define RGB(r, g, b) ((uint16_t)( \
    ((uint16_t)((((r)&0xF8)<<8)|(((g)&0xFC)<<3)|((b)>>3)) >> 8) | \
    ((uint16_t)((((r)&0xF8)<<8)|(((g)&0xFC)<<3)|((b)>>3)) << 8) ))
#define C_BLACK  RGB(0,0,0)
#define C_WHITE  RGB(255,255,255)
#define C_GREEN  RGB(0,255,0)
#define C_AMBER  RGB(255,176,0)
#define C_CYAN   RGB(0,210,255)
#define C_GREY   RGB(130,140,150)
#define C_DGREY  RGB(28,34,40)

static esp_lcd_panel_handle_t s_panel;
static bool s_ok;
static uint16_t s_fb[TFT_W * TFT_H];   /* ~25 KB RGB565 framebuffer */

/* compact 5x7 font, ASCII 0x20..0x7A (space..'z'); 5 columns/char */
static const uint8_t FONT[][5] = {
    {0,0,0,0,0},{0,0,0x5F,0,0},{0,7,0,7,0},{0x14,0x7F,0x14,0x7F,0x14},
    {0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,8,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},
    {0,5,3,0,0},{0,0x1C,0x22,0x41,0},{0,0x41,0x22,0x1C,0},{0x14,8,0x3E,8,0x14},
    {8,8,0x3E,8,8},{0,0x50,0x30,0,0},{8,8,8,8,8},{0,0x60,0x60,0,0},{0x20,0x10,8,4,2},
    {0x3E,0x51,0x49,0x45,0x3E},{0,0x42,0x7F,0x40,0},{0x42,0x61,0x51,0x49,0x46},
    {0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
    {0x3C,0x4A,0x49,0x49,0x30},{1,0x71,9,5,3},{0x36,0x49,0x49,0x49,0x36},
    {6,0x49,0x49,0x29,0x1E},{0,0x36,0x36,0,0},{0,0x56,0x36,0,0},{8,0x14,0x22,0x41,0},
    {0x14,0x14,0x14,0x14,0x14},{0,0x41,0x22,0x14,8},{2,1,0x51,9,6},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},
    {0x7F,9,9,9,1},{0x3E,0x41,0x49,0x49,0x7A},{0x7F,8,8,8,0x7F},{0,0x41,0x7F,0x41,0},
    {0x20,0x40,0x41,0x3F,1},{0x7F,8,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,2,0x0C,2,0x7F},{0x7F,4,8,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,9,9,9,6},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,9,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{1,1,0x7F,1,1},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,8,0x14,0x63},
    {7,8,0x70,8,7},{0x61,0x51,0x49,0x45,0x43},{0,0x7F,0x41,0x41,0},{2,4,8,0x10,0x20},
    {0,0x41,0x41,0x7F,0},{4,2,1,2,4},{0x40,0x40,0x40,0x40,0x40},{0,1,2,4,0},
    {0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},
    {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{8,0x7E,9,1,2},
    {0x0C,0x52,0x52,0x52,0x3E},{0x7F,8,4,4,0x78},{0,0x44,0x7D,0x40,0},
    {0x20,0x40,0x44,0x3D,0},{0x7F,0x10,0x28,0x44,0},{0,0x41,0x7F,0x40,0},
    {0x7C,4,0x18,4,0x78},{0x7C,8,4,4,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,8},{8,0x14,0x14,0x18,0x7C},{0x7C,8,4,4,8},
    {0x48,0x54,0x54,0x54,0x20},{4,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},
    {0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},
    {0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},
};

/* ---- framebuffer primitives ---- */
static void fb_fill(int x, int y, int w, int h, uint16_t col) {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > TFT_W) w = TFT_W - x;
    if (y + h > TFT_H) h = TFT_H - y;
    for (int r = 0; r < h; r++)
        for (int c = 0; c < w; c++)
            s_fb[(y + r) * TFT_W + (x + c)] = col;
}
static void fb_char(int x, int y, char ch, uint16_t col) {
    if (ch < 0x20 || ch > 0x7A) ch = '?';
    const uint8_t *g = FONT[ch - 0x20];
    for (int c = 0; c < 5; c++)
        for (int row = 0; row < 8; row++)
            if (g[c] & (1 << row)) {
                int px = x + c, py = y + row;
                if ((unsigned)px < TFT_W && (unsigned)py < TFT_H)
                    s_fb[py * TFT_W + px] = col;
            }
}
static void fb_text(int x, int y, const char *s, uint16_t col) {
    while (*s && x < TFT_W - 5) { fb_char(x, y, *s++, col); x += 6; }
}
static void blit(void) {
    if (s_ok) esp_lcd_panel_draw_bitmap(s_panel, 0, 0, TFT_W, TFT_H, s_fb);
}

bool display_init(void) {
    gpio_config_t bl = {.pin_bit_mask = 1ULL << PIN_BL, .mode = GPIO_MODE_OUTPUT};
    gpio_config(&bl);
    gpio_set_level(PIN_BL, 1);          /* off (active-low) until first frame */

    spi_bus_config_t buscfg = ST7735_PANEL_BUS_SPI_CONFIG(PIN_CLK, PIN_MOSI,
                                                          TFT_W * TFT_H * sizeof(uint16_t));
    if (spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO) != ESP_OK) return false;

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = ST7735_PANEL_IO_SPI_CONFIG(PIN_CS, PIN_DC, NULL, NULL);
    if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io) != ESP_OK)
        return false;

    esp_lcd_panel_dev_config_t pcfg = {
        .reset_gpio_num = PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
    };
    if (esp_lcd_new_panel_st7735(io, &pcfg, &s_panel) != ESP_OK) return false;
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_set_gap(s_panel, 1, 26);     /* landscape spacing (reference) */
    esp_lcd_panel_swap_xy(s_panel, true);
    esp_lcd_panel_mirror(s_panel, false, true);
    esp_lcd_panel_disp_on_off(s_panel, true);

    s_ok = true;
    fb_fill(0, 0, TFT_W, TFT_H, C_BLACK);
    fb_text(4, 6, "ESP-SDR S3", C_CYAN);
    fb_text(4, 18, "TX / RX ready", C_GREY);
    blit();
    gpio_set_level(PIN_BL, 0);          /* backlight on */
    return true;
}

void display_status(const char *l1, const char *l2) {
    if (!s_ok) return;
    fb_fill(0, 0, TFT_W, TFT_H, C_BLACK);
    if (l1) fb_text(4, 6, l1, C_WHITE);
    if (l2) fb_text(4, 18, l2, C_GREY);
    blit();
}

void display_waveform(const int8_t *iq, unsigned n, const char *label, bool tx) {
    if (!s_ok || !iq || !n) return;
    const int top = 22, h = TFT_H - top, mid = top + h / 2;
    uint16_t col = tx ? C_AMBER : C_GREEN;

    fb_fill(0, 0, TFT_W, TFT_H, C_BLACK);
    fb_fill(0, mid, TFT_W, 1, C_DGREY);           /* zero axis */
    for (int x = 0; x < TFT_W; x++) {
        unsigned s0 = (unsigned)x * n / TFT_W;
        unsigned s1 = (unsigned)(x + 1) * n / TFT_W;
        if (s1 <= s0) s1 = s0 + 1;
        if (s1 > n) s1 = n;
        int mx = 0;                               /* peak |I|+|Q| in column */
        for (unsigned s = s0; s < s1; s++) {
            int a = iq[2 * s], b = iq[2 * s + 1];
            int m = (a < 0 ? -a : a) + (b < 0 ? -b : b);
            if (m > mx) mx = m;
        }
        int bar = mx * (h / 2) / 255;
        if (bar > h / 2) bar = h / 2;
        if (bar < 1) bar = 1;
        fb_fill(x, mid - bar, 1, bar * 2, col);
    }
    fb_text(4, 4, tx ? "TX" : "RX", col);
    if (label) fb_text(28, 4, label, C_WHITE);
    blit();
}
