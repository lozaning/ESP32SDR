/* Minimal ST7735S waveform display for the LILYGO T-Dongle-S3.
 *
 * Purely additive: all functions are no-ops on failure so the RF / serial
 * path is never affected. If the panel shows nothing or garbage, only the
 * geometry/inversion constants in display.c need tuning (see DISPLAY TUNING
 * comment there) -- the RF firmware is unaffected either way.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Bring up the SPI bus and ST7735S panel. Safe to ignore the return value. */
bool display_init(void);

/* Two short status lines (e.g. "RX 2437 MHz" / "16 Msps"). */
void display_status(const char *line1, const char *line2);

/* Draw a signed interleaved int8 I/Q buffer as an amplitude envelope.
 * label is shown in the header row; tx tints the trace differently. */
void display_waveform(const int8_t *iq, unsigned samples, const char *label, bool tx);
