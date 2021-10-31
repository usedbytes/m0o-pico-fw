/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __COMM_H__
#define __COMM_H__

#include <stdint.h>

#define COMM_MAX_NARG     5
#define COMM_MAX_DATA_LEN 1024

#define COMM_RSP_OK       (('O' << 0) | ('K' << 8) | ('O' << 16) | ('K' << 24))
#define COMM_RSP_ERR      (('E' << 0) | ('R' << 8) | ('R' << 16) | ('!' << 24))

struct comm_command {
	uint32_t opcode;
	uint32_t nargs;
	uint32_t resp_nargs;
	uint32_t (*size)(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
	uint32_t (*handle)(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);
};

void comm_init(const struct comm_command *const *cmds, int n_cmds, uint32_t sync_opcode);

#endif /* __COMM_H__ */
