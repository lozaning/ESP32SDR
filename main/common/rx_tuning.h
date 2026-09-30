#pragma once
#include <stdbool.h>
#define RX_FREQ_MIN 100u
#define RX_FREQ_MAX 6000u
#define RX_TUNING_RANGE_REPLY "RANGE 100 6000 1\n"
static inline bool rx_frequency_valid(unsigned mhz) {
    return mhz >= RX_FREQ_MIN && mhz <= RX_FREQ_MAX;
}
