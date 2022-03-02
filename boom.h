/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __BOOM_H__
#define __BOOM_H__

#include <stdint.h>

int boom_extend_set(int8_t val);
bool boom_extend_at_limit();

void boom_init();

int16_t boom_update_count();
void boom_reset_count();

#endif /* __BOOM_H__ */
