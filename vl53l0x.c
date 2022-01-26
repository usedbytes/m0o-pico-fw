/*
 * Copyright (C) 2018 Brian Starkey <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <errno.h>

#include "log.h"
#include "util.h"
#include "vl53l0x.h"

#include "pico/stdlib.h"

#include "vl53l0x/core/inc/vl53l0x_api.h"
#include "vl53l0x/core/inc/vl53l0x_def.h"
#include "vl53l0x/platform/inc/vl53l0x_platform.h"

static int vl53l0x_platform_errno;

#define VL53L0X_DEFAULT_ADDR 0x29

#define to_dev(_Dev) ((struct vl53l0x_dev *)_Dev)

int vl53l0x_init_array(struct vl53l0x_dev *devs, unsigned ndevs)
{
	int ret;
	unsigned int i, j;
	unsigned int ngpios = 0;

	for (i = 0; i < ndevs; i++) {
		if (devs[i].xshut_port) {
			ngpios++;
			gpio_set_dir(devs[i].xshut_pin, GPIO_OUT);
			gpio_put(devs[i].xshut_pin, 0);
		}
		for (j = 0; j < ndevs; j++) {
			if (j == i) {
				continue;
			}
			if (devs[i].addr_7b == devs[j].addr_7b) {
				log_printf(&util_logger, "Address %2x conflict. Devices %d and %d",
					devs[i].addr_7b, i, j);
				return -ENOMEM;
			}
		}
	}

	if (ngpios < ndevs - 1) {
		log_printf(&util_logger, "Not enough gpios.");
		return -EXDEV;
	}

	for (i = 0; i < ndevs; i++) {
		ret = vl53l0x_init(&devs[i]);
		if (ret) {
			log_printf(&util_logger, "Init failed on dev %d", i);
			return ret;
		}
	}

	return 0;
}

int vl53l0x_init(struct vl53l0x_dev *dev)
{
	VL53L0X_DEV pal_dev = &dev->pal_dev;
	VL53L0X_Error err = VL53L0X_ERROR_NONE;
	int ret;

	VL53L0X_DeviceInfo_t dev_info;
	uint32_t count;
	uint8_t vhv, phase, type;

	if (dev->xshut_port) {
		gpio_put(dev->xshut_pin, 1);
	}

	ret = vl53l0x_reset(dev);
	if (ret) {
		log_printf(&util_logger, "vl53l0x_init reset: %d",ret);
		return ret;
	}

	ret = i2c_bus_read_raw(dev->i2c, dev->addr_7b, &vhv, 1);
	if (ret) {
		log_printf(&util_logger, "vl53l0x_init initial read: %d",ret);
		return ret;
	} else if (dev->addr_7b != VL53L0X_DEFAULT_ADDR) {
		dev->addr_set = false;
		ret = vl53l0x_set_addr(dev, dev->addr_7b);
		if (ret) {
			return ret;
		}
	}
	dev->addr_set = true;

	err = VL53L0X_DataInit(pal_dev);
	if (err) {
		log_printf(&util_logger, "DataInit: %x",(uint32_t)err);
		return err;
	}

	err = VL53L0X_GetDeviceInfo(pal_dev, &dev_info);
	if (err) {
		log_printf(&util_logger, "GetDeviceInfo: %x", (uint32_t)err);
		return err;
	}

	err = VL53L0X_StaticInit(pal_dev);
	if (err) {
		log_printf(&util_logger, "StaticInit: %x", (uint32_t)err);
		return err;
	}

	ret = vl53l0x_perform_ref_cal(dev, &vhv, &phase);
	if (ret) {
		return ret;
	}
	log_printf(&util_logger, "vhv: %d phase: %d\n", (uint32_t)vhv, (uint32_t)phase);

	ret = vl53l0x_perform_ref_spad(dev, &count, &type);
	if (ret) {
		return ret;
	}
	log_printf(&util_logger, "SPAD: %d ap: %d\n", count, (uint32_t)type);

	return 0;
}

int vl53l0x_set_addr(struct vl53l0x_dev *dev, uint8_t new_addr_7b)
{
	uint8_t old_addr = dev->addr_set ? dev->addr_7b : VL53L0X_DEFAULT_ADDR;
	int ret;

	log_printf(&util_logger, "Setting addr: %x", new_addr_7b);

	ret = i2c_bus_write(dev->i2c, old_addr, (uint8_t []){ VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, new_addr_7b }, 2);
	if (ret) {
		vl53l0x_platform_errno = ret;
		log_printf(&util_logger, "SetAddress: %x",(uint32_t)ret);
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	dev->addr_7b = new_addr_7b;
	dev->addr_set = true;

	return 0;
}

int vl53l0x_perform_ref_cal(struct vl53l0x_dev *dev, uint8_t *vhv, uint8_t *phase)
{
	return VL53L0X_PerformRefCalibration(&dev->pal_dev, vhv, phase);
}

int vl53l0x_load_ref_cal(struct vl53l0x_dev *dev, uint8_t vhv, uint8_t phase)
{
	return VL53L0X_SetRefCalibration(&dev->pal_dev, vhv, phase);
}

int vl53l0x_perform_ref_spad(struct vl53l0x_dev *dev, uint32_t *count, uint8_t *type)
{
	return VL53L0X_PerformRefSpadManagement(&dev->pal_dev, count, type);
}

int vl53l0x_load_ref_spad(struct vl53l0x_dev *dev, uint32_t count, uint8_t type)
{
	return VL53L0X_SetReferenceSpads(&dev->pal_dev, count, type);
}

int vl53l0x_perform_offset_cal(struct vl53l0x_dev *dev, FixPoint1616_t distance_mm, int32_t *offset_um)
{
	return VL53L0X_PerformOffsetCalibration(&dev->pal_dev, distance_mm, offset_um);
}

int vl53l0x_load_offset_cal(struct vl53l0x_dev *dev, int32_t offset_um)
{
	return VL53L0X_SetOffsetCalibrationDataMicroMeter(&dev->pal_dev, offset_um);
}

int vl53l0x_perform_xtalk_cal(struct vl53l0x_dev *dev, FixPoint1616_t distance_mm, FixPoint1616_t *xtalk)
{
	return VL53L0X_PerformXTalkCalibration(&dev->pal_dev, distance_mm, xtalk);
}

int vl53l0x_load_xtalk_cal(struct vl53l0x_dev *dev, FixPoint1616_t xtalk)
{
	return VL53L0X_SetXTalkCompensationRateMegaCps(&dev->pal_dev, xtalk);
}

int vl53l0x_enable_xtalk_compensation(struct vl53l0x_dev *dev)
{
	return VL53L0X_SetXTalkCompensationEnable(&dev->pal_dev, 1);
}

int vl53l0x_disable_xtalk_compensation(struct vl53l0x_dev *dev)
{
	return VL53L0X_SetXTalkCompensationEnable(&dev->pal_dev, 0);
}

int vl53l0x_set_measurement_time(struct vl53l0x_dev *dev, uint32_t us)
{
	return VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&dev->pal_dev, us);
}

int vl53l0x_set_measurement_mode(struct vl53l0x_dev *dev, VL53L0X_DeviceModes mode)
{
	return VL53L0X_SetDeviceMode(&dev->pal_dev, mode);
}

int vl53l0x_start_measurement(struct vl53l0x_dev *dev)
{
	return VL53L0X_StartMeasurement(&dev->pal_dev);
}

int vl53l0x_stop_measurement(struct vl53l0x_dev *dev)
{
	return VL53L0X_StopMeasurement(&dev->pal_dev);
}

int vl53l0x_check_stop_completed(struct vl53l0x_dev *dev)
{
	VL53L0X_Error err;
	uint32_t busy;

	err = VL53L0X_GetStopCompletedStatus(&dev->pal_dev, &busy);
	if (err) {
		return err;
	}

	if (!busy) {
		return 1;
	}

	return 0;
}

int vl53l0x_check_measurement_ready(struct vl53l0x_dev *dev)
{
	VL53L0X_Error err;
	uint8_t ready;

	err = VL53L0X_GetMeasurementDataReady(&dev->pal_dev, &ready);
	if (err) {
		return err;
	}

	if (ready) {
		return 1;
	}

	return 0;
}

int vl53l0x_get_measurement(struct vl53l0x_dev *dev, VL53L0X_RangingMeasurementData_t *data)
{
	VL53L0X_Error err;

	err = VL53L0X_GetRangingMeasurementData(&dev->pal_dev, data);
	if (err) {
		return err;
	}

	/* TODO: Check error? */
	VL53L0X_ClearInterruptMask(&dev->pal_dev, 0);

	return 0;
}

int vl53l0x_do_single_measurement(struct vl53l0x_dev *dev, VL53L0X_RangingMeasurementData_t *data)
{
	int i;
	int ret = vl53l0x_set_measurement_mode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	if (ret) {
		return ret;
	}

	ret = vl53l0x_start_measurement(dev);
	if (ret) {
		return ret;
	}

	for (i = 0; i < 50; i++) {
		ret = vl53l0x_check_measurement_ready(dev);
		if (ret < 0) {
			return ret;
		} else if (ret == 1) {
			break;
		}
		sleep_ms(5);
	}

	if (i >= 50) {
		return -ETIMEDOUT;
	}

	return vl53l0x_get_measurement(dev, data);
}

int vl53l0x_get_platform_error(void)
{
	return vl53l0x_platform_errno;
}

int vl53l0x_reset(struct vl53l0x_dev *dev)
{
	return VL53L0X_ResetDevice(&dev->pal_dev);
}

int vl53l0x_set_long_range_preset(struct vl53l0x_dev *dev)
{
	VL53L0X_DEV pal_dev = &dev->pal_dev;
	int res;

	res = VL53L0X_SetLimitCheckValue(pal_dev,
				VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
				(FixPoint1616_t)(0.1*65536));
	if (res) {
		log_printf(&util_logger, "vl53l0x long range signal rate: %d", res);
		return res;
	}

	res = VL53L0X_SetLimitCheckValue(pal_dev,
				VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
				(FixPoint1616_t)(60*65536));
	if (res) {
		log_printf(&util_logger, "vl53l0x long range sigma: %d", res);
		return res;
	}

	res = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pal_dev,
			33000);
	if (res) {
		log_printf(&util_logger, "vl53l0x long range time: %d", res);
		return res;
	}

	res = VL53L0X_SetVcselPulsePeriod(pal_dev,
			VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	if (res) {
		log_printf(&util_logger, "vl53l0x long range pre range: %d", res);
		return res;
	}

	res = VL53L0X_SetVcselPulsePeriod(pal_dev,
			VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
	if (res) {
		log_printf(&util_logger, "vl53l0x long range final range: %d", res);
		return res;
	}

	return 0;
}

/******************************************************************************
 * STM API PAL Implementation
 ******************************************************************************/

uint8_t i2c_buf[32];

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
	(void)Dev;
	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
	(void)Dev;
	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	struct vl53l0x_dev *dev = to_dev(Dev);

	if (count >= sizeof(i2c_buf) - 1) {
		log_printf(&util_logger, "transfer too long: %d", count);
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	i2c_buf[0] = index;
	memcpy(i2c_buf + 1, pdata, count);

	int ret = i2c_bus_write(dev->i2c, dev->addr_7b, i2c_buf, count + 1);
	if (ret) {
		vl53l0x_platform_errno = ret;
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
	struct vl53l0x_dev *dev = to_dev(Dev);

	int ret = i2c_bus_read(dev->i2c, dev->addr_7b, index, pdata, count);
	if (ret) {
		vl53l0x_platform_errno = ret;
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
	struct vl53l0x_dev *dev = to_dev(Dev);

	int ret = i2c_bus_write(dev->i2c, dev->addr_7b, (uint8_t[]){ index, data }, 2);
	if (ret) {
		vl53l0x_platform_errno = ret;
		return VL53L0X_ERROR_CONTROL_INTERFACE;
	}

	return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
	return VL53L0X_WriteMulti(Dev, index, (uint8_t *)&data, sizeof(data));
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
	return VL53L0X_WriteMulti(Dev, index, (uint8_t *)&data, sizeof(data));
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
	return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
	return VL53L0X_ReadMulti(Dev, index, (uint8_t *)data, sizeof(*data));
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
	return VL53L0X_ReadMulti(Dev, index, (uint8_t *)data, sizeof(*data));
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
	uint8_t data;
	VL53L0X_Error err = VL53L0X_RdByte(Dev, index, &data);
	if (err != VL53L0X_ERROR_NONE) {
		log_printf(&util_logger, "vl53l0x update_byte R %d: %d", index, err);
		return err;
	}

	data = (data & AndData) | OrData;

	return VL53L0X_WrByte(Dev, index, data);
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
	(void)Dev;
	sleep_ms(5);
	return VL53L0X_ERROR_NONE;
}
