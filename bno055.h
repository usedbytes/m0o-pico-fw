// C port of https://github.com/ghirlekar/bno055-python-i2c
// SPDX-License-Identifier: MIT
//
// The MIT License (MIT)
// 
// Copyright (c) 2015 Gaurav Hirlekar
// Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#ifndef __BNO055_H__
#define __BNO055_H__

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"

#define BNO055_ADDRESS_A  0x28
#define BNO055_ADDRESS_B  0x29
#define BNO055_ID         0xA0

// Power mode settings
#define BNO055_POWER_MODE_NORMAL    0x00
#define BNO055_POWER_MODE_LOWPOWER  0x01
#define BNO055_POWER_MODE_SUSPEND   0x02

// REGISTER DEFINITION START
#define BNO055_REG_PAGE_ID  0x07

#define BNO055_REG_CHIP_ID        0x00
#define BNO055_REG_ACCEL_REV_ID   0x01
#define BNO055_REG_MAG_REV_ID     0x02
#define BNO055_REG_GYRO_REV_ID    0x03
#define BNO055_REG_SW_REV_ID_LSB  0x04
#define BNO055_REG_SW_REV_ID_MSB  0x05
#define BNO055_REG_BL_REV_ID      0x06

// Accel data register
#define BNO055_REG_ACCEL_DATA_X_LSB  0x08
#define BNO055_REG_ACCEL_DATA_X_MSB  0x09
#define BNO055_REG_ACCEL_DATA_Y_LSB  0x0A
#define BNO055_REG_ACCEL_DATA_Y_MSB  0x0B
#define BNO055_REG_ACCEL_DATA_Z_LSB  0x0C
#define BNO055_REG_ACCEL_DATA_Z_MSB  0x0D

// Mag data register
#define BNO055_REG_MAG_DATA_X_LSB  0x0E
#define BNO055_REG_MAG_DATA_X_MSB  0x0F
#define BNO055_REG_MAG_DATA_Y_LSB  0x10
#define BNO055_REG_MAG_DATA_Y_MSB  0x11
#define BNO055_REG_MAG_DATA_Z_LSB  0x12
#define BNO055_REG_MAG_DATA_Z_MSB  0x13

// Gyro data registers
#define BNO055_REG_GYRO_DATA_X_LSB  0x14
#define BNO055_REG_GYRO_DATA_X_MSB  0x15
#define BNO055_REG_GYRO_DATA_Y_LSB  0x16
#define BNO055_REG_GYRO_DATA_Y_MSB  0x17
#define BNO055_REG_GYRO_DATA_Z_LSB  0x18
#define BNO055_REG_GYRO_DATA_Z_MSB  0x19

// Euler data registers
#define BNO055_REG_EULER_H_LSB  0x1A
#define BNO055_REG_EULER_H_MSB  0x1B
#define BNO055_REG_EULER_R_LSB  0x1C
#define BNO055_REG_EULER_R_MSB  0x1D
#define BNO055_REG_EULER_P_LSB  0x1E
#define BNO055_REG_EULER_P_MSB  0x1F

// Quaternion data registers
#define BNO055_REG_QUATERNION_DATA_W_LSB  0x20
#define BNO055_REG_QUATERNION_DATA_W_MSB  0x21
#define BNO055_REG_QUATERNION_DATA_X_LSB  0x22
#define BNO055_REG_QUATERNION_DATA_X_MSB  0x23
#define BNO055_REG_QUATERNION_DATA_Y_LSB  0x24
#define BNO055_REG_QUATERNION_DATA_Y_MSB  0x25
#define BNO055_REG_QUATERNION_DATA_Z_LSB  0x26
#define BNO055_REG_QUATERNION_DATA_Z_MSB  0x27

// Linear acceleration data registers
#define BNO055_REG_LINEAR_ACCEL_DATA_X_LSB  0x28
#define BNO055_REG_LINEAR_ACCEL_DATA_X_MSB  0x29
#define BNO055_REG_LINEAR_ACCEL_DATA_Y_LSB  0x2A
#define BNO055_REG_LINEAR_ACCEL_DATA_Y_MSB  0x2B
#define BNO055_REG_LINEAR_ACCEL_DATA_Z_LSB  0x2C
#define BNO055_REG_LINEAR_ACCEL_DATA_Z_MSB  0x2D

// Gravity data registers
#define BNO055_REG_GRAVITY_DATA_X_LSB  0x2E
#define BNO055_REG_GRAVITY_DATA_X_MSB  0x2F
#define BNO055_REG_GRAVITY_DATA_Y_LSB  0x30
#define BNO055_REG_GRAVITY_DATA_Y_MSB  0x31
#define BNO055_REG_GRAVITY_DATA_Z_LSB  0x32
#define BNO055_REG_GRAVITY_DATA_Z_MSB  0x33

// Temperature data register
#define BNO055_REG_TEMP  0x34

// Status registers
#define BNO055_REG_CALIB_STAT       0x35
#define BNO055_REG_SELFTEST_RESULT  0x36
#define BNO055_REG_INTR_STAT        0x37

#define BNO055_REG_SYS_CLK_STAT  0x38
#define BNO055_REG_SYS_STAT      0x39
#define BNO055_REG_SYS_ERR       0x3A

// Unit selection register
#define BNO055_REG_UNIT_SEL     0x3B
#define BNO055_REG_DATA_SELECT  0x3C

// Mode registers
#define BNO055_REG_OPR_MODE  0x3D
#define BNO055_REG_PWR_MODE  0x3E

#define BNO055_REG_SYS_TRIGGER  0x3F
#define BNO055_REG_TEMP_SOURCE  0x40

// Axis remap registers
#define BNO055_REG_AXIS_MAP_CONFIG  0x41
#define BNO055_REG_AXIS_MAP_SIGN    0x42

// SIC registers
#define BNO055_REG_SIC_MATRIX_0_LSB  0x43
#define BNO055_REG_SIC_MATRIX_0_MSB  0x44
#define BNO055_REG_SIC_MATRIX_1_LSB  0x45
#define BNO055_REG_SIC_MATRIX_1_MSB  0x46
#define BNO055_REG_SIC_MATRIX_2_LSB  0x47
#define BNO055_REG_SIC_MATRIX_2_MSB  0x48
#define BNO055_REG_SIC_MATRIX_3_LSB  0x49
#define BNO055_REG_SIC_MATRIX_3_MSB  0x4A
#define BNO055_REG_SIC_MATRIX_4_LSB  0x4B
#define BNO055_REG_SIC_MATRIX_4_MSB  0x4C
#define BNO055_REG_SIC_MATRIX_5_LSB  0x4D
#define BNO055_REG_SIC_MATRIX_5_MSB  0x4E
#define BNO055_REG_SIC_MATRIX_6_LSB  0x4F
#define BNO055_REG_SIC_MATRIX_6_MSB  0x50
#define BNO055_REG_SIC_MATRIX_7_LSB  0x51
#define BNO055_REG_SIC_MATRIX_7_MSB  0x52
#define BNO055_REG_SIC_MATRIX_8_LSB  0x53
#define BNO055_REG_SIC_MATRIX_8_MSB  0x54

// Accelerometer Offset registers
#define BNO055_REG_ACCEL_OFFSET_X_LSB  0x55
#define BNO055_REG_ACCEL_OFFSET_X_MSB  0x56
#define BNO055_REG_ACCEL_OFFSET_Y_LSB  0x57
#define BNO055_REG_ACCEL_OFFSET_Y_MSB  0x58
#define BNO055_REG_ACCEL_OFFSET_Z_LSB  0x59
#define BNO055_REG_ACCEL_OFFSET_Z_MSB  0x5A

// Magnetometer Offset registers
#define BNO055_REG_MAG_OFFSET_X_LSB  0x5B
#define BNO055_REG_MAG_OFFSET_X_MSB  0x5C
#define BNO055_REG_MAG_OFFSET_Y_LSB  0x5D
#define BNO055_REG_MAG_OFFSET_Y_MSB  0x5E
#define BNO055_REG_MAG_OFFSET_Z_LSB  0x5F
#define BNO055_REG_MAG_OFFSET_Z_MSB  0x60

// Gyroscope Offset registers
#define BNO055_REG_GYRO_OFFSET_X_LSB  0x61
#define BNO055_REG_GYRO_OFFSET_X_MSB  0x62
#define BNO055_REG_GYRO_OFFSET_Y_LSB  0x63
#define BNO055_REG_GYRO_OFFSET_Y_MSB  0x64
#define BNO055_REG_GYRO_OFFSET_Z_LSB  0x65
#define BNO055_REG_GYRO_OFFSET_Z_MSB  0x66

// Radius registers
#define BNO055_REG_ACCEL_RADIUS_LSB  0x67
#define BNO055_REG_ACCEL_RADIUS_MSB  0x68
#define BNO055_REG_MAG_RADIUS_LSB    0x69
#define BNO055_REG_MAG_RADIUS_MSB    0x6A

enum bno055_operation_mode {
	BNO055_OPERATION_MODE_CONFIG       = 0x00,
	BNO055_OPERATION_MODE_ACCONLY      = 0x01,
	BNO055_OPERATION_MODE_MAGONLY      = 0x02,
	BNO055_OPERATION_MODE_GYRONLY      = 0x03,
	BNO055_OPERATION_MODE_ACCMAG       = 0x04,
	BNO055_OPERATION_MODE_ACCGYRO      = 0x05,
	BNO055_OPERATION_MODE_MAGGYRO      = 0x06,
	BNO055_OPERATION_MODE_AMG          = 0x07,
	BNO055_OPERATION_MODE_IMUPLUS      = 0x08,
	BNO055_OPERATION_MODE_COMPASS      = 0x09,
	BNO055_OPERATION_MODE_M4G          = 0x0A,
	BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
	BNO055_OPERATION_MODE_NDOF         = 0x0C,
};

enum bno055_vector_type {
	BNO055_VECTOR_ACCELEROMETER  = 0x08,
	BNO055_VECTOR_MAGNETOMETER   = 0x0E,
	BNO055_VECTOR_GYROSCOPE      = 0x14,
	BNO055_VECTOR_EULER          = 0x1A,
	BNO055_VECTOR_LINEARACCEL    = 0x28,
	BNO055_VECTOR_GRAVITY        = 0x2E,
};

struct bno055 {
	i2c_inst_t *i2c;
	enum bno055_operation_mode mode;
	uint8_t addr;
};

int bno055_init(struct bno055 *bno055, i2c_inst_t *i2c, uint8_t addr);

int bno055_ping(struct bno055 *bno055);

int bno055_reset(struct bno055 *bno055);

int bno055_set_operation_mode(struct bno055 *bno055, enum bno055_operation_mode mode);

int bno055_set_external_crystal(struct bno055 *bno055, bool external);

int bno055_get_system_status(struct bno055 *bno055, uint8_t *stat, uint8_t *syserr, uint8_t *selftest);

int bno055_get_calibration(struct bno055 *bno055, uint8_t *calib);

int bno055_get_temp(struct bno055 *bno055, uint8_t *temp);

int bno055_get_vector(struct bno055 *bno055, enum bno055_vector_type type, float vec[3]);

#endif /* __BNO055_H__ */
