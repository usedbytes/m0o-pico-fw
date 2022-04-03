/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __HOST_H__
#define __HOST_H__

#include "platform/platform.h"

#include "comm.h"

extern const struct comm_command snapget_cmd;
extern const struct comm_command camera_host_comm_cmd;

void host_init(struct platform *platform);

#endif /* __HOST_H__ */
