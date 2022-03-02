/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __BOOM_H__
#define __BOOM_H__

#include <stdint.h>

void boom_extend_set_raw(int8_t val);
int boom_extend_set_protected(int8_t val);

void boom_init();

uint16_t boom_update_count();
void boom_reset_count();

#endif /* __BOOM_H__ */
