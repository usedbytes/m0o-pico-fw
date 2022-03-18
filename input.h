/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __INPUT_H__
#define __INPUT_H__

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

extern const struct comm_command input_cmd;

struct input_event {
        uint16_t btn_down;
        uint16_t btn_up;

        int8_t lx;
        int8_t ly;
        int8_t rx;
        int8_t ry;

	uint8_t hat;
#define INPUT_FLAG_DUMMY (1 << 0)
	uint8_t flags;
};

#endif /* __INPUT_H__ */
