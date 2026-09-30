#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef enum { BURST_SERIAL_USB, BURST_SERIAL_UART, BURST_SERIAL_COUNT } burst_serial_port_t;

void burst_serial_init(void);
int burst_serial_poll_line(char *line, size_t capacity);
burst_serial_port_t burst_serial_port(void);
unsigned burst_serial_baud(void);
bool burst_serial_receive(void *data, size_t size, unsigned timeout_ms);
bool burst_serial_send(const void *data, size_t size);
