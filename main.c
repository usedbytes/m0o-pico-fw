/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "camera/camera.h"
#include "camera/format.h"

#include "bno055.h"
#include "camera_task.h"
#include "chassis.h"
#include "comm.h"
#include "log.h"
#include "util.h"
#include "vl53l0x.h"

#define CMD_INPUT    (('I' << 0) | ('N' << 8) | ('P' << 16) | ('T' << 24))

#define MOTOR_PIN_L_A 14
#define MOTOR_PIN_L_B 15
#define MOTOR_PIN_R_A 12
#define MOTOR_PIN_R_B 13

#define BNO055_ADDR 0x28

#define I2C_BUS      i2c0
#define I2C_PIN_SDA  0
#define I2C_PIN_SCL  1

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

#define BOOM_LIFT_A     6
#define BOOM_LIFT_B     7
#define BOOM_LIFT_LIMIT 22
#define BOOM_EXTEND_A   8
#define BOOM_EXTEND_B   9
#define BOOM_EXTEND_ENC   11  // Has to be connected to a PWM 'B' pin to count
#define BOOM_EXTEND_LIMIT 10

#define HAT_UP    (1 << 0)
#define HAT_RIGHT (1 << 1)
#define HAT_DOWN  (1 << 2)
#define HAT_LEFT  (1 << 3)
const uint8_t hat_pos_to_dirs[] = {
	[0] = HAT_UP,
	[1] = HAT_UP | HAT_RIGHT,
	[2] = HAT_RIGHT,
	[3] = HAT_RIGHT | HAT_DOWN,
	[4] = HAT_DOWN,
	[5] = HAT_DOWN | HAT_LEFT,
	[6] = HAT_LEFT,
	[7] = HAT_LEFT | HAT_UP,
};

struct chassis chassis;
volatile bool heading_mode;

const uint boom_lift_slice =   (BOOM_LIFT_A  / 2) % 8;
const uint boom_extend_slice = (BOOM_EXTEND_A  / 2) % 8;

struct bt_hid_state {
	uint16_t buttons;
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
	uint8_t hat;
	uint8_t pad;
};

static uint32_t size_input(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out);
static uint32_t handle_input(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out);

extern const struct comm_command snapget_cmd;
extern const struct comm_command camera_host_comm_cmd;

// This list is ordered to try and put the most frequent messages near the start
const struct comm_command *const cmds[] = {
	&(const struct comm_command){
		.opcode = CMD_INPUT,
		.nargs = 0,
		.resp_nargs = 0,
		.size = &size_input,
		.handle = &handle_input,
	},
	&util_sync_cmd,
	&util_logs_cmd,
	&util_reboot_cmd,
	&util_read_cmd,
	&snapget_cmd,
	&camera_host_comm_cmd,
};
const unsigned int N_CMDS = (sizeof(cmds) / sizeof(cmds[0]));

static uint32_t size_input(uint32_t *args_in, uint32_t *data_len_out, uint32_t *resp_data_len_out)
{
	*data_len_out = sizeof(struct bt_hid_state);
	*resp_data_len_out = 0;

	return COMM_RSP_OK;
}

static int8_t clamp8(int16_t value) {
	if (value > 127) {
		return 127;
	} else if (value < -128) {
		return -128;
	}

	return value;
}

static int16_t abs16(int16_t v) {
	return v < 0 ? -v : v;
}

static uint32_t handle_input(uint32_t *args_in, uint8_t *data_in, uint32_t *resp_args_out, uint8_t *resp_data_out)
{
	struct bt_hid_state state;
	int8_t lift_val = 0;
	int8_t extend_val = 0;

	memcpy(&state, data_in, sizeof(state));

	log_printf(&util_logger, "L: %2x,%2x R: %2x,%2x, Hat: %1x, Buttons: %04x",
			state.lx, state.ly, state.rx, state.ry, state.hat, state.buttons);

	if (state.buttons) {
		gpio_put(PICO_DEFAULT_LED_PIN, 1);
	} else {
		gpio_put(PICO_DEFAULT_LED_PIN, 0);
	}

	if (state.buttons & (1 << BTN_BIT_Y)) {
		heading_mode = true;
	}

	if (state.buttons & (1 << BTN_BIT_A)) {
		heading_mode = false;
	}

	if (state.hat <= 7) {
		uint8_t hat_dirs = hat_pos_to_dirs[state.hat];

		if (hat_dirs & HAT_UP) {
			lift_val = 127;
		} else if (hat_dirs & HAT_DOWN) {
			lift_val = -127;
		}

		if (hat_dirs & HAT_RIGHT) {
			extend_val = 127;
		} else if (hat_dirs & HAT_LEFT) {
			extend_val = -127;
		}
	}

	int8_t linear = clamp8(-(state.ly - 128));
	int8_t rot = clamp8(-(state.rx - 128));
	chassis_set(&chassis, linear, rot);

	slice_set(boom_lift_slice, lift_val);
	slice_set(boom_extend_slice, extend_val);

	return COMM_RSP_OK;
}

struct i2c_bus i2c;
queue_t pos_queue;
static void core1_main(void)
{
	run_camera(&pos_queue, &i2c);

	// Should hopefully never reach here.
	while (1) {
		tight_loop_contents();
	}
}

static int reinit_vl53l0x(struct vl53l0x_dev *sens)
{
	int ret = vl53l0x_init(sens);
	log_printf(&util_logger, "vl53l0x init: %d", ret);
	if (ret) {
		log_printf(&util_logger, "init fail: %x", vl53l0x_get_platform_error());
		return ret;
	}

	ret = vl53l0x_set_measurement_mode(sens, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	if (ret) {
		log_printf(&util_logger, "vl53l0x set mode: %d", ret);
		return ret;
	}

	ret = vl53l0x_set_measurement_time(sens, 66000);
	if (ret) {
		log_printf(&util_logger, "vl53l0x set time: %d", ret);
		return ret;
	}

	/*
	ret = vl53l0x_set_long_range_preset(sens);
	if (ret) {
		log_printf(&util_logger, "vl53l0x set time: %d", ret);
		return ret;
	}
	*/

	return 0;
}

struct controller {
	int16_t kp;
	int16_t scale;
};

struct controller heading_ctrl = {
	.kp = (int16_t)(1.8f * 16), // 1.0
	.scale = 256,
};

struct controller distance_ctrl = {
	.kp = (int16_t)(0.15f * 16), // 1.0
	.scale = 16,
};

int8_t controller_tick(struct controller *ctrl, int16_t diff)
{
	return clamp8(((int32_t)diff * ctrl->kp) / ctrl->scale);
}

struct plan_stage {
#define SOURCE_HEADING  1
#define SOURCE_DISTANCE 2
#define SOURCE_RED_BLOB 4
	uint8_t sources;
	uint8_t termination_source;
	int16_t target_distance;
	int16_t target_heading;
};

struct plan_stage stages[] = {
	{ // First just turn to face up the arena
		.sources = (SOURCE_HEADING),
		.termination_source = (SOURCE_HEADING),
		.target_heading = 90 * 16,
	},
	{ // Drive up to first trough level
		.sources = (SOURCE_HEADING | SOURCE_DISTANCE),
		.termination_source = (SOURCE_DISTANCE),
		.target_distance = 250,
		.target_heading = 90 * 16,
	},
	{ // Turn to face trough
		.sources = (SOURCE_HEADING),
		.termination_source = (SOURCE_HEADING),
		.target_heading = 180 * 16,
	},
	{ // Drive to first trough
		.sources = (SOURCE_HEADING | SOURCE_DISTANCE),
		.termination_source = (SOURCE_DISTANCE),
		.target_distance = 930,
		.target_heading = 180 * 16,
	},
	{ // Back up a bit
		.sources = (SOURCE_HEADING | SOURCE_DISTANCE),
		.termination_source = (SOURCE_DISTANCE),
		.target_distance = 600,
		.target_heading = 180 * 16,
	},

	{ // Face back up the arena
		.sources = (SOURCE_HEADING),
		.termination_source = (SOURCE_HEADING),
		.target_heading = 90 * 16,
	},
	{ // Drive up to second trough level
		.sources = (SOURCE_HEADING | SOURCE_DISTANCE),
		.termination_source = (SOURCE_DISTANCE),
		.target_distance = 750,
		.target_heading = 90 * 16,
	},
	{ // Turn to face trough
		.sources = (SOURCE_HEADING),
		.termination_source = (SOURCE_HEADING),
		.target_heading = 0 * 16,
	},
	{ // Drive to second trough
		.sources = (SOURCE_HEADING | SOURCE_DISTANCE),
		.termination_source = (SOURCE_DISTANCE),
		.target_distance = 930,
		.target_heading = 0 * 16,
	},
	{ // Back up a bit
		.sources = (SOURCE_HEADING | SOURCE_DISTANCE),
		.termination_source = (SOURCE_DISTANCE),
		.target_distance = 600,
		.target_heading = 0 * 16,
	},

	{ // Face back up the arena
		.sources = (SOURCE_HEADING),
		.termination_source = (SOURCE_HEADING),
		.target_heading = 90 * 16,
	},
	{ // Drive up to third trough level
		.sources = (SOURCE_HEADING | SOURCE_DISTANCE),
		.termination_source = (SOURCE_DISTANCE),
		.target_distance = 1125,
		.target_heading = 90 * 16,
	},
	{ // Turn to face trough
		.sources = (SOURCE_HEADING),
		.termination_source = (SOURCE_HEADING),
		.target_heading = 180 * 16,
	},
	{ // Drive to third trough
		.sources = (SOURCE_HEADING | SOURCE_DISTANCE),
		.termination_source = (SOURCE_DISTANCE),
		.target_distance = 930,
		.target_heading = 180 * 16,
	},
	{ // Back up a bit
		.sources = (SOURCE_HEADING | SOURCE_DISTANCE),
		.termination_source = (SOURCE_DISTANCE),
		.target_distance = 600,
		.target_heading = 180 * 16,
	},
};

extern void local_capture_cb(struct camera_buffer *buf, void *data);

int main()
{
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	const uint enc_slice = (BOOM_EXTEND_ENC / 2) % 8;
	{ /* Boom bits, needs breaking out */
		init_slice(boom_lift_slice, BOOM_LIFT_A);
		init_slice(boom_extend_slice, BOOM_EXTEND_A);

		gpio_set_function(BOOM_EXTEND_ENC, GPIO_FUNC_PWM);

		pwm_config c = pwm_get_default_config();
		pwm_config_set_clkdiv_mode(&c, PWM_DIV_B_RISING);
		pwm_init(enc_slice, &c, true);

		gpio_init(BOOM_LIFT_LIMIT);
		gpio_set_pulls(BOOM_LIFT_LIMIT, true, false);
	}

	util_init();

	comm_init(cmds, N_CMDS, UTIL_CMD_SYNC);

	i2c_bus_init(&i2c, I2C_BUS, 100000);
	gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_PIN_SDA);
	gpio_pull_up(I2C_PIN_SCL);

	chassis_init(&chassis, MOTOR_PIN_L_A, MOTOR_PIN_R_A);

	struct vl53l0x_dev sens = {
		.i2c = &i2c,
		.addr_7b = 0x29,
	};

	bool inited = false;
	int ret = reinit_vl53l0x(&sens);
	if (!ret) {
		inited = true;
	}

	VL53L0X_RangingMeasurementData_t data;
	if (inited) {
		log_printf(&util_logger, "Do measure...");
		ret = vl53l0x_do_single_measurement(&sens, &data);
		log_printf(&util_logger, "Done: %d %d mm", ret, data.RangeMilliMeter);
		if (ret) {
			inited = false;
		}
	}

	struct bno055 bno055;
	ret = bno055_init(&bno055, &i2c, BNO055_ADDR);
	log_printf(&util_logger, "bno055 init: %d", ret);

	queue_init(&pos_queue, sizeof(int), 2);

	multicore_launch_core1(core1_main);

	struct camera_buffer *buf = camera_buffer_alloc(FORMAT_YUV422, 80, 60);
	struct camera_queue_item local_capture_qitem;
	local_capture_qitem.type = CAMERA_QUEUE_ITEM_CAPTURE;
	local_capture_qitem.capture.buf = buf;
	local_capture_qitem.capture.frame_cb = local_capture_cb;

	int stage_idx = 0;
	struct plan_stage *stage = NULL;
	bool last_mode = false;
	bool stage_done = false;

	int pos;
	int16_t blob_pos;

	while (1) {
		if (!heading_mode) {
			/*
			if (inited) {
				log_printf(&util_logger, "Do measure...");
				ret = vl53l0x_do_single_measurement(&sens, &data);
				char status_str[128];
				VL53L0X_GetRangeStatusString(data.RangeStatus, status_str);
				log_printf(&util_logger, "Done: %d %d mm, timestamp %d us, status %s", ret, data.RangeMilliMeter,
						data.TimeStamp, status_str);
				if (ret != 0) {
					inited = false;
				}
			} else {
				ret = reinit_vl53l0x(&sens);
				if (ret) {
					inited = false;
				}
			}

			int16_t current_heading;
			int ret = bno055_get_heading(&bno055, &current_heading);
			if (ret != 0) {
				log_printf(&util_logger, "bno055 error: %d", ret);
				ret = bno055_init(&bno055, &i2c, BNO055_ADDR);
				log_printf(&util_logger, "re-init: %d", ret);
				continue;
			} else {
				log_printf(&util_logger, "Heading: %3.2f", (float)current_heading / 16.0);
			}
			*/

			last_mode = false;

			sleep_ms(300);

			log_printf(&util_logger, "Encoder: %d, lift endstop: %d", pwm_get_counter(enc_slice), gpio_get(BOOM_LIFT_LIMIT));
			continue;
		}

		if (last_mode != heading_mode) {
			stage_idx = 0;
			stage = &stages[stage_idx];

			log_printf(&util_logger, "restarting bno055 to start");
			// Re-init to reset heading?
			bno055_init(&bno055, &i2c, BNO055_ADDR);
		}

		last_mode = true;

		if (stage->sources & SOURCE_DISTANCE) {
			vl53l0x_start_measurement(&sens);
		}

		if (stage->sources & SOURCE_RED_BLOB) {
			// Request a camera frame
			camera_queue_add_blocking(&local_capture_qitem);

			// Wait for the result
			queue_remove_blocking(&pos_queue, &pos);
		}

		int8_t linear = 0;
		int8_t omega = 0;

		int16_t heading_diff = 0;
		int16_t distance_diff = 0;

		if (stage->sources & SOURCE_DISTANCE) {
			data.RangeStatus = -1;
			vl53l0x_get_outstanding_measurement(&sens, &data);
			if (data.RangeStatus != 0) {
				sleep_ms(10);
				continue;
			}
			distance_diff = stage->target_distance - data.RangeMilliMeter;

			linear = controller_tick(&distance_ctrl, distance_diff);
			log_printf(&util_logger, "linear diff: %d, speed: %d", distance_diff, linear);
		}

		if (stage->sources & SOURCE_HEADING) {
			const int8_t min_omega = 9;
			const int16_t small_angle = 3;
			int16_t heading;
			bno055_get_heading(&bno055, &heading);

			heading_diff = heading - stage->target_heading;
			while (heading_diff > (180 * 16)) {
				heading_diff -= (360 * 16);
			}

			omega = controller_tick(&heading_ctrl, heading_diff);

			// HAX: Need an integral term
			if (heading_diff >= small_angle && omega < min_omega) {
				omega = min_omega;
			}

			if (heading_diff <= -small_angle && omega > -min_omega) {
				omega = -min_omega;
			}

			log_printf(&util_logger, "heading diff: %3.2f, omega: %d", heading_diff / 16.0f, omega);
		}

		if ((stage->termination_source & SOURCE_DISTANCE) &&
			abs16(distance_diff) < 10) {
			log_printf(&util_logger, "distance terminated");
			stage_done = true;
		}

		if ((stage->termination_source & SOURCE_HEADING) &&
			abs16(heading_diff) < 10) {
			log_printf(&util_logger, "heading terminated");
			stage_done = true;
		}

		if (stage_done) {
			log_printf(&util_logger, "stage %d done", stage_idx);
			linear = 0;
			omega = 0;
			stage_idx++;
			if (stage_idx >= sizeof(stages) / sizeof(stages[0])) {
				heading_mode = false;
			} else {
				stage = &stages[stage_idx];
			}
			stage_done = false;

			chassis_set(&chassis, linear, omega);
			sleep_ms(300);
		}

		chassis_set(&chassis, linear, omega);

		sleep_ms(10);

		/*
		// Request a camera frame
		camera_queue_add_blocking(&local_capture_qitem);

		// Wait for the result
		queue_remove_blocking(&pos_queue, &pos);

		// Update motors
		int diff = 20 - pos;
		log_printf(&util_logger, "Middle: %d, diff: %d", pos, diff);
		chassis_set(&chassis, 0, diff * 3);
		sleep_ms(10);
		*/
	}

	/*
	TODO: Conflict on i2c bus
	i2c_init(i2c0, 100 * 1000);
	gpio_set_function(0, GPIO_FUNC_I2C);
	gpio_set_function(1, GPIO_FUNC_I2C);
	gpio_pull_up(0);
	gpio_pull_up(1);

	sleep_ms(1000);

	struct bno055 bno055;
	int ret = bno055_init(&bno055, i2c0, BNO055_ADDR);
	log_printf(&util_logger, "bno055 init: %d", ret);

	int16_t target_heading = 0;
	int16_t kp = -(1 << 6);
	int i = 0;

	struct vl53l0x_dev sens = {
		.i2c = i2c,
		.addr_7b = 0x29,
	};

	bool inited = false;
	ret = reinit_vl53l0x(&sens);
	if (!ret) {
		inited = true;
	}

	VL53L0X_RangingMeasurementData_t data;
	if (inited) {
		log_printf(&util_logger, "Do measure...");
		ret = vl53l0x_do_single_measurement(&sens, &data);
		log_printf(&util_logger, "Done: %d %d mm", ret, data.RangeMilliMeter);
		if (ret) {
			inited = false;
		}
	}

	while (1) {
		if (!heading_mode) {
			if (inited) {
				log_printf(&util_logger, "Do measure...");
				ret = vl53l0x_do_single_measurement(&sens, &data);
				char status_str[128];
				VL53L0X_GetRangeStatusString(data.RangeStatus, status_str);
				log_printf(&util_logger, "Done: %d %d mm, timestamp %d us, status %s", ret, data.RangeMilliMeter,
						data.TimeStamp, status_str);
				if (ret != 0) {
					inited = false;
				}
			} else {
				ret = reinit_vl53l0x(&sens);
				if (ret) {
					inited = false;
				}
			}
			sleep_ms(300);
			continue;
		}

		sleep_ms(10);

		int16_t current_heading;
		int ret = bno055_get_heading(&bno055, &current_heading);
		if (ret != 0) {
			heading_mode = false;
			log_printf(&util_logger, "error: %d", ret);
			ret = bno055_init(&bno055, i2c0, BNO055_ADDR);
			log_printf(&util_logger, "re-init: %d", ret);
			continue;
		}

		int16_t heading_diff = current_heading - target_heading;
		while (heading_diff > (180 * 16)) {
			heading_diff -= (360 * 16);
		}

		int16_t omega = (heading_diff * kp) >> 8;

		chassis_set(&chassis, 0, -omega);
	}
	*/
}
