/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __INPUT_H__
#define __INPUT_H__

#include <stdint.h>

#define HAT_UP    (1 << 0)
#define HAT_RIGHT (1 << 1)
#define HAT_DOWN  (1 << 2)
#define HAT_LEFT  (1 << 3)

#define BTN_BIT_A       0
#define BTN_BIT_B       1
#define BTN_BIT_X       2
#define BTN_BIT_Y       3
#define BTN_BIT_L1      4
#define BTN_BIT_L2      5
#define BTN_BIT_L3      6
#define BTN_BIT_R1      7
#define BTN_BIT_R2      8
#define BTN_BIT_R3      9
#define BTN_BIT_START   10
#define BTN_BIT_SELECT  11
#define BTN_BIT_HEART   12
#define BTN_BIT_STAR    13

#define BTN_TRIANGLE (1 << BTN_BIT_X)
#define BTN_CIRCLE   (1 << BTN_BIT_A)
#define BTN_CROSS    (1 << BTN_BIT_B)
#define BTN_SQUARE   (1 << BTN_BIT_Y)
#define BTN_L1       (1 << BTN_BIT_L1)
#define BTN_L2       (1 << BTN_BIT_L2)
#define BTN_L3       (1 << BTN_BIT_L3)
#define BTN_R1       (1 << BTN_BIT_R1)
#define BTN_R2       (1 << BTN_BIT_R2)
#define BTN_R3       (1 << BTN_BIT_R3)
#define BTN_START    (1 << BTN_BIT_START)
#define BTN_SELECT   (1 << BTN_BIT_SELECT)
#define BTN_HEART    (1 << BTN_BIT_HEART)
// Note! Star doesn't work for some reason!
#define BTN_STAR     (1 << BTN_BIT_STAR)

extern const struct comm_command input_cmd;

struct input_state {
	struct {
		uint8_t lx;
		uint8_t ly;
		uint8_t rx;
		uint8_t ry;
	} axes;
	struct {
		uint8_t pressed;
		uint8_t released;
		uint8_t held;
	} hat;
	struct {
		uint16_t pressed;
		uint16_t released;
		uint16_t held;
	} buttons;
};

void input_state_print(struct input_state *input);

#endif /* __INPUT_H__ */
