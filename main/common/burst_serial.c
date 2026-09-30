/* The burst protocol is shared by native USB Serial/JTAG and UART0. */
#include "burst_serial.h"
#include <stdint.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "hal/usb_serial_jtag_ll.h"
#include "sdkconfig.h"

#ifndef CONFIG_ESP_SDR_UART_BAUD
#define CONFIG_ESP_SDR_UART_BAUD 2000000
#endif

#define COMMAND_SIZE 128
static burst_serial_port_t active_port = BURST_SERIAL_USB;
static unsigned next_port;
static struct {
    char line[COMMAND_SIZE];
    size_t used;
    bool overflow;
    int64_t last_byte;
} input[BURST_SERIAL_COUNT];

void burst_serial_init(void) {
#if CONFIG_ESP_SDR_UART_ENABLED
    const uart_config_t config = {
        .baud_rate = CONFIG_ESP_SDR_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, CONFIG_ESP_SDR_UART_TX_PIN,
                                CONFIG_ESP_SDR_UART_RX_PIN,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 8192, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_flush_input(UART_NUM_0));
#endif
}

burst_serial_port_t burst_serial_port(void) { return active_port; }
unsigned burst_serial_baud(void) {
    return active_port == BURST_SERIAL_UART ? CONFIG_ESP_SDR_UART_BAUD : 0;
}

static int read_port(burst_serial_port_t port, void *buffer, size_t size) {
    if (port == BURST_SERIAL_UART) {
#if CONFIG_ESP_SDR_UART_ENABLED
        return uart_read_bytes(UART_NUM_0, buffer, size, 0);
#else
        return 0;
#endif
    }
    return usb_serial_jtag_ll_read_rxfifo(buffer, size > 64 ? 64 : size);
}

bool burst_serial_receive(void *data, size_t size, unsigned timeout_ms) {
    uint8_t *p = data;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (size) {
        int got = read_port(active_port, p, size > 128 ? 128 : size);
        if (got < 0) return false;
        if (!got) {
            if (esp_timer_get_time() >= deadline) return false;
            vTaskDelay(1);
            continue;
        }
        p += got;
        size -= got;
    }
    return true;
}

int burst_serial_poll_line(char *line, size_t capacity) {
    for (unsigned j = 0; j < BURST_SERIAL_COUNT; ++j) {
        unsigned port = (next_port + j) % BURST_SERIAL_COUNT;
        int64_t now = esp_timer_get_time();
        if (now - input[port].last_byte > 3000000) {
            input[port].used = 0;
            input[port].overflow = false;
        }
        for (unsigned k = 0; k < COMMAND_SIZE; ++k) {
            char ch;
            if (read_port(port, &ch, 1) != 1) break;
            input[port].last_byte = now;
            if (ch == '\r') continue;
            if (ch != '\n') {
                if (input[port].used < COMMAND_SIZE - 1)
                    input[port].line[input[port].used++] = ch;
                else input[port].overflow = true;
                continue;
            }
            size_t used = input[port].used;
            bool overflow = input[port].overflow;
            input[port].used = 0;
            input[port].overflow = false;
            if (!used && !overflow) continue;
            active_port = port;
            next_port = (port + 1) % BURST_SERIAL_COUNT;
            if (overflow || used >= capacity) return -1;
            memcpy(line, input[port].line, used);
            line[used] = '\0';
            return 1;
        }
    }
    return 0;
}

static int64_t transfer_deadline(size_t size) {
    int64_t timeout = active_port == BURST_SERIAL_UART
        ? 2000000 + (int64_t)size * 10000000 / CONFIG_ESP_SDR_UART_BAUD
        : 3000000;
    return esp_timer_get_time() + timeout;
}

bool IRAM_ATTR burst_serial_send(const void *data, size_t size) {
    const uint8_t *p = data;
    size_t original = size;
    int64_t deadline = transfer_deadline(size);
    while (size) {
        if (esp_timer_get_time() >= deadline) return false;
        int sent;
        if (active_port == BURST_SERIAL_UART) {
            sent = uart_tx_chars(UART_NUM_0, (const char *)p, size > 128 ? 128 : size);
            if (sent < 0) return false;
            if (!sent) vTaskDelay(1);
        } else {
            if (!usb_serial_jtag_ll_txfifo_writable()) continue;
            sent = usb_serial_jtag_ll_write_txfifo(p, size > 64 ? 64 : size);
            usb_serial_jtag_ll_txfifo_flush();
        }
        p += sent;
        size -= sent;
    }
    if (active_port == BURST_SERIAL_USB && original && original % 64 == 0) {
        while (!usb_serial_jtag_ll_txfifo_writable())
            if (esp_timer_get_time() >= deadline) return false;
        usb_serial_jtag_ll_txfifo_flush();
    }
    return true;
}
