/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "log.h"

struct log_message {
	uint32_t timestamp;
	uint16_t len;
	// char string[];
} __attribute__ ((packed));

void log_init(struct log_buffer *log)
{
	mutex_init(&log->lock);
	memset(log->buf, 0, sizeof(log->buf));
	log->space = sizeof(log->buf);
	log->insert_idx = 0;
	log->extract_idx = 0;
}

static inline void __log_write_bytes(struct log_buffer *log, const uint8_t *data, uint16_t len)
{
	uint16_t insert_idx = log->insert_idx;
	uint16_t space_before_wrap = sizeof(log->buf) - insert_idx;
	uint16_t first_len = space_before_wrap < len ? space_before_wrap : len;

	memcpy(log->buf + insert_idx, data, first_len);
	memcpy(log->buf, data + first_len, len - first_len);

	log->insert_idx = (insert_idx + len) % sizeof(log->buf);
	log->space -= len;
}

static inline void __log_peek_bytes(struct log_buffer *log, uint8_t *data, uint16_t len)
{
	uint16_t extract_idx = log->extract_idx;
	uint16_t space_before_wrap = sizeof(log->buf) - extract_idx;
	uint16_t first_len = space_before_wrap < len ? space_before_wrap : len;

	memcpy(data, log->buf + extract_idx, first_len);
	memcpy(data + first_len, log->buf, len - first_len);
}

static inline void __log_read_bytes(struct log_buffer *log, uint8_t *data, uint16_t len)
{
	uint16_t extract_idx = log->extract_idx;
	uint16_t space_before_wrap = sizeof(log->buf) - extract_idx;
	uint16_t first_len = space_before_wrap < len ? space_before_wrap : len;

	memcpy(data, log->buf + extract_idx, first_len);
	memcpy(data + first_len, log->buf, len - first_len);

	log->extract_idx = (extract_idx + len) % sizeof(log->buf);
	log->space += len;
}

static inline uint16_t __log_drop_message(struct log_buffer *log)
{
	if (log->space == sizeof(log->buf)) {
		return 0;
	}

	struct log_message msg;
	__log_peek_bytes(log, (uint8_t *)&msg, sizeof(msg));

	uint16_t msg_size = sizeof(msg) + msg.len;
	uint16_t extract_idx = log->extract_idx;
	log->extract_idx = (extract_idx + msg_size) % sizeof(log->buf);
	log->space += msg_size;

	return msg_size;
}

static inline void __log_make_space(struct log_buffer *log, uint32_t at_least)
{
	int space = log->space;

	// FIXME: Using <= wastes some space, but it avoids a corner case
	// where the buffer ends up exactly full meaning that drain
	// stops working.
	// We can fix this by switching the insert/extract indices to be
	// non-modulo.
	while (space <= at_least) {
		uint16_t msg_size = __log_drop_message(log);
		space += msg_size;

		if (msg_size == 0) {
			break;
		}
	}
}

void log_write(struct log_buffer *log, const char *message, uint16_t len)
{
	uint32_t space_reqd = sizeof(struct log_message) + len;

	// Will never fit, so just drop it.
	if ((space_reqd > UINT16_MAX) || space_reqd >= sizeof(log->buf)) {
		return;
	}

	mutex_enter_blocking(&log->lock);

	__log_make_space(log, space_reqd);

	struct log_message msg = {
		.timestamp = time_us_32(),
		.len = len,
	};

	__log_write_bytes(log, (const uint8_t *)&msg, sizeof(msg));
	__log_write_bytes(log, (const uint8_t *)message, len);

	mutex_exit(&log->lock);
}

int log_printf(struct log_buffer *log, const char *fmt, ...)
{
	static char buf[256];
	va_list args;
	int ret;

	va_start(args, fmt);
	ret = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	log_write(log, buf, ret);

	return ret;
}

uint16_t log_drain(struct log_buffer *log, uint8_t *buf, uint16_t size)
{
	mutex_enter_blocking(&log->lock);

	uint16_t extract_idx = log->extract_idx;
	uint16_t insert_idx = log->insert_idx;
	uint16_t used = 0;

	while (extract_idx != insert_idx) {
		struct log_message msg;
		__log_peek_bytes(log, (uint8_t *)&msg, sizeof(msg));

		uint16_t msg_size = sizeof(msg) + msg.len;
		if (used + msg_size >= size) {
			break;
		}

		__log_read_bytes(log, buf + used, msg_size);
		used += msg_size;
		extract_idx = (extract_idx + msg_size) % sizeof(log->buf);
	}

	mutex_exit(&log->lock);

	return used;
}
