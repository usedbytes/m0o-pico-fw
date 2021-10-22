/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __LOG_H__
#define __LOG_H__

#include <stdint.h>

#include "pico/mutex.h"

#define LOG_BUFFER_SIZE 1024
struct log_buffer {
	uint8_t buf[LOG_BUFFER_SIZE];
	mutex_t lock;
	volatile uint16_t insert_idx;
	volatile uint16_t extract_idx;
	volatile uint16_t space;
};

void log_init(struct log_buffer *log);
void log_write(struct log_buffer *log, const char *message, uint16_t len);
int log_printf(struct log_buffer *log, const char *fmt, ...);
uint16_t log_drain(struct log_buffer *log, uint8_t *buf, uint16_t size);

#endif /* __LOG_H__ */
