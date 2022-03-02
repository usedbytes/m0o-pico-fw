/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __UTIL_H__
#define __UTIL_H__

#include "comm.h"
#include "log.h"

#define UTIL_CMD_SYNC     (('S' << 0) | ('Y' << 8) | ('N' << 16) | ('C' << 24))
#define UTIL_CMD_LOGS     (('L' << 0) | ('O' << 8) | ('G' << 16) | ('S' << 24))
#define UTIL_CMD_REBOOT   (('B' << 0) | ('O' << 8) | ('O' << 16) | ('T' << 24))
#define UTIL_CMD_READ     (('R' << 0) | ('E' << 8) | ('A' << 16) | ('D' << 24))

extern const struct comm_command util_sync_cmd;
extern const struct comm_command util_logs_cmd;
extern const struct comm_command util_reboot_cmd;
extern const struct comm_command util_read_cmd;

extern struct log_buffer util_logger;

void util_init(void);
void util_reboot(bool to_bootloader);

static inline int8_t clamp8(int16_t value) {
        if (value > 127) {
                return 127;
        } else if (value < -128) {
                return -128;
        }

        return value;
}

#endif /* __UTIL_H__ */
