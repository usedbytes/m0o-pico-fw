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

extern const struct comm_command input_cmd;

struct input_event {
        uint16_t btn_down;
        uint16_t btn_up;

        int8_t lx;
        int8_t ly;
        int8_t rx;
        int8_t ry;

	uint8_t hat;
};

void input_init();
void input_get_event_blocking(struct input_event *event);
bool input_try_get_event(struct input_event *event);

#endif /* __INPUT_H__ */
