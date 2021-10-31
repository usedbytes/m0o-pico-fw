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

extern const struct comm_command util_sync_cmd;
extern const struct comm_command util_logs_cmd;
extern const struct comm_command util_reboot_cmd;

extern struct log_buffer util_logger;

void util_init(void);

#endif /* __UTIL_H__ */
