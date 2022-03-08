/*
 * Derived from Pimoroni driver:
 * Copyright (c) 2021 Pimoroni Ltd
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 */
#include "pico/time.h"

#include "ioexpander.h"

#define IOE_REG_CHIP_ID_L	0xfa
#define IOE_REG_CHIP_ID_H	0xfb
#define IOE_CHIP_ID		0xe26a

#define IOE_REG_VERSION		0xfc

// Rotary encoder
#define IOE_REG_ENC_EN		0x04
// BIT_ENC_EN_1	0
// BIT_ENC_MICROSTEP_1	1
// BIT_ENC_EN_2	2
// BIT_ENC_MICROSTEP_2	3
// BIT_ENC_EN_3	4
// BIT_ENC_MICROSTEP_3	5
// BIT_ENC_EN_4	6
// BIT_ENC_MICROSTEP_4	7

#define IOE_REG_ENC_1_CFG	0x05
#define IOE_REG_ENC_1_COUNT	0x06
#define IOE_REG_ENC_2_CFG	0x07
#define IOE_REG_ENC_2_COUNT	0x08
#define IOE_REG_ENC_3_CFG	0x09
#define IOE_REG_ENC_3_COUNT	0x0A
#define IOE_REG_ENC_4_CFG	0x0B
#define IOE_REG_ENC_4_COUNT	0x0C

// Cap touch
#define IOE_REG_CAPTOUCH_EN	0x0D
#define IOE_REG_CAPTOUCH_CFG	0x0E
#define IOE_REG_CAPTOUCH_0	0x0F  // First of 8 bytes from 15-22

// Switch counters
#define IOE_REG_SWITCH_EN_P0	0x17
#define IOE_REG_SWITCH_EN_P1	0x18
#define IOE_REG_SWITCH_P00	0x19  // First of 8 bytes from 25-40
#define IOE_REG_SWITCH_P10	0x21  // First of 8 bytes from 33-49

#define IOE_REG_USER_FLASH	0xD0
#define IOE_REG_FLASH_PAGE	0xF0
#define IOE_REG_DEBUG		0xF8

#define IOE_REG_P0		0x40  // protect_bits 2 # Bit addressing
#define IOE_REG_SP		0x41  // Read only
#define IOE_REG_DPL		0x42  // Read only
#define IOE_REG_DPH		0x43  // Read only
#define IOE_REG_RCTRIM0		0x44  // Read only
#define IOE_REG_RCTRIM1		0x45  // Read only
#define IOE_REG_RWK		0x46
#define IOE_REG_PCON		0x47  // Read only
#define IOE_REG_TCON		0x48
#define IOE_REG_TMOD		0x49
#define IOE_REG_TL0		0x4a
#define IOE_REG_TL1		0x4b
#define IOE_REG_TH0		0x4c
#define IOE_REG_TH1		0x4d
#define IOE_REG_CKCON		0x4e
#define IOE_REG_WKCON		0x4f  // Read only
#define IOE_REG_P1		0x50  // protect_bits 3 6 # Bit addressing
#define IOE_REG_SFRS		0x51  // TA protected # Read only
#define IOE_REG_CAPCON0		0x52
#define IOE_REG_CAPCON1		0x53
#define IOE_REG_CAPCON2		0x54
#define IOE_REG_CKDIV		0x55
#define IOE_REG_CKSWT		0x56  // TA protected # Read only
#define IOE_REG_CKEN		0x57  // TA protected # Read only
#define IOE_REG_SCON		0x58
#define IOE_REG_SBUF		0x59
#define IOE_REG_SBUF_1		0x5a
#define IOE_REG_EIE		0x5b  // Read only
#define IOE_REG_EIE1		0x5c  // Read only
#define IOE_REG_CHPCON		0x5f  // TA protected # Read only
#define IOE_REG_P2		0x60  // Bit addressing
#define IOE_REG_AUXR1		0x62
#define IOE_REG_BODCON0		0x63  // TA protected
#define IOE_REG_IAPTRG		0x64  // TA protected # Read only
#define IOE_REG_IAPUEN		0x65  // TA protected # Read only
#define IOE_REG_IAPAL		0x66  // Read only
#define IOE_REG_IAPAH		0x67  // Read only
#define IOE_REG_IE		0x68  // Read only
#define IOE_REG_SADDR		0x69
#define IOE_REG_WDCON		0x6a  // TA protected
#define IOE_REG_BODCON1		0x6b  // TA protected
#define IOE_REG_P3M1		0x6c
#define IOE_REG_P3S		0xc0  // Page 1 # Reassigned from 0x6c to avoid collision
#define IOE_REG_P3M2		0x6d
#define IOE_REG_P3SR		0xc1  // Page 1 # Reassigned from 0x6d to avoid collision
#define IOE_REG_IAPFD		0x6e  // Read only
#define IOE_REG_IAPCN		0x6f  // Read only
#define IOE_REG_P3		0x70  // Bit addressing
#define IOE_REG_P0M1		0x71  // protect_bits  2
#define IOE_REG_P0S		0xc2  // Page 1 # Reassigned from 0x71 to avoid collision
#define IOE_REG_P0M2		0x72  // protect_bits  2
#define IOE_REG_P0SR		0xc3  // Page 1 # Reassigned from 0x72 to avoid collision
#define IOE_REG_P1M1		0x73  // protect_bits  3 6
#define IOE_REG_P1S		0xc4  // Page 1 # Reassigned from 0x73 to avoid collision
#define IOE_REG_P1M2		0x74  // protect_bits  3 6
#define IOE_REG_P1SR		0xc5  // Page 1 # Reassigned from 0x74 to avoid collision
#define IOE_REG_P2S		0x75
#define IOE_REG_IPH		0x77  // Read only
#define IOE_REG_PWMINTC		0xc6  // Page 1 # Read only # Reassigned from 0x77 to avoid collision
#define IOE_REG_IP		0x78  // Read only
#define IOE_REG_SADEN		0x79
#define IOE_REG_SADEN_1		0x7a
#define IOE_REG_SADDR_1		0x7b
#define IOE_REG_I2DAT		0x7c  // Read only
#define IOE_REG_I2STAT		0x7d  // Read only
#define IOE_REG_I2CLK		0x7e  // Read only
#define IOE_REG_I2TOC		0x7f  // Read only
#define IOE_REG_I2CON		0x80  // Read only
#define IOE_REG_I2ADDR		0x81  // Read only
#define IOE_REG_ADCRL		0x82
#define IOE_REG_ADCRH		0x83
#define IOE_REG_T3CON		0x84
#define IOE_REG_PWM4H		0xc7  // Page 1 # Reassigned from 0x84 to avoid collision
#define IOE_REG_RL3		0x85
#define IOE_REG_PWM5H		0xc8  // Page 1 # Reassigned from 0x85 to avoid collision
#define IOE_REG_RH3		0x86
#define IOE_REG_PIOCON1		0xc9  // Page 1 # Reassigned from 0x86 to avoid collision
#define IOE_REG_TA		0x87  // Read only
#define IOE_REG_T2CON		0x88
#define IOE_REG_T2MOD		0x89
#define IOE_REG_RCMP2L		0x8a
#define IOE_REG_RCMP2H		0x8b
#define IOE_REG_TL2		0x8c
#define IOE_REG_PWM4L		0xca  // Page 1 # Reassigned from 0x8c to avoid collision
#define IOE_REG_TH2		0x8d
#define IOE_REG_PWM5L		0xcb  // Page 1 # Reassigned from 0x8d to avoid collision
#define IOE_REG_ADCMPL		0x8e
#define IOE_REG_ADCMPH		0x8f
#define IOE_REG_PSW		0x90  // Read only
#define IOE_REG_PWMPH		0x91
#define IOE_REG_PWM0H		0x92
#define IOE_REG_PWM1H		0x93
#define IOE_REG_PWM2H		0x94
#define IOE_REG_PWM3H		0x95
#define IOE_REG_PNP		0x96
#define IOE_REG_FBD		0x97
#define IOE_REG_PWMCON0		0x98
#define IOE_REG_PWMPL		0x99
#define IOE_REG_PWM0L		0x9a
#define IOE_REG_PWM1L		0x9b
#define IOE_REG_PWM2L		0x9c
#define IOE_REG_PWM3L		0x9d
#define IOE_REG_PIOCON0		0x9e
#define IOE_REG_PWMCON1		0x9f
#define IOE_REG_ACC		0xa0  // Read only
#define IOE_REG_ADCCON1		0xa1
#define IOE_REG_ADCCON2		0xa2
#define IOE_REG_ADCDLY		0xa3
#define IOE_REG_C0L		0xa4
#define IOE_REG_C0H		0xa5
#define IOE_REG_C1L		0xa6
#define IOE_REG_C1H		0xa7
#define IOE_REG_ADCCON0		0xa8
#define IOE_REG_PICON		0xa9  // Read only
#define IOE_REG_PINEN		0xaa  // Read only
#define IOE_REG_PIPEN		0xab  // Read only
#define IOE_REG_PIF		0xac  // Read only
#define IOE_REG_C2L		0xad
#define IOE_REG_C2H		0xae
#define IOE_REG_EIP		0xaf  // Read only
#define IOE_REG_B		0xb0  // Read only
#define IOE_REG_CAPCON3		0xb1
#define IOE_REG_CAPCON4		0xb2
#define IOE_REG_SPCR		0xb3
#define IOE_REG_SPCR2		0xcc  // Page 1 # Reassigned from 0xb3 to avoid collision
#define IOE_REG_SPSR		0xb4
#define IOE_REG_SPDR		0xb5
#define IOE_REG_AINDIDS		0xb6
#define IOE_REG_EIPH		0xb7  // Read only
#define IOE_REG_SCON_1		0xb8
#define IOE_REG_PDTEN		0xb9  // TA protected
#define IOE_REG_PDTCNT		0xba  // TA protected
#define IOE_REG_PMEN		0xbb
#define IOE_REG_PMD		0xbc
#define IOE_REG_EIP1		0xbe  // Read only
#define IOE_REG_EIPH1		0xbf  // Read only

#define IOE_REG_INT		0xf9
#define IOE_INT_MASK_P0		0x00
#define IOE_INT_MASK_P1		0x01
#define IOE_INT_MASK_P3		0x03

#define IOE_REG_ADDR		0xfd

#define IOE_REG_CTRL		0xfe  // 0	Sleep 1	Reset 2	Read Flash 3	Write Flash 4	Addr Unlock

static int read_regs(struct ioexpander *ioe, uint8_t reg_addr, uint8_t *buf, uint8_t n_reg)
{
	int ret = ioe->i2c_write_blocking(ioe->i2c_handle, ioe->dev_addr, (uint8_t[]){ reg_addr }, 1);
	if (ret != 1) {
		//printf("write error: %d\n", ret);
		return -1;
	}

	ret = ioe->i2c_read_blocking(ioe->i2c_handle, ioe->dev_addr, buf, n_reg);
	if (ret != n_reg) {
		//printf("read error: %d\n", ret);
		return -1;
	}

	return 0;
}

static int write_reg(struct ioexpander *ioe, uint8_t reg_addr, uint8_t val)
{
	int ret = ioe->i2c_write_blocking(ioe->i2c_handle, ioe->dev_addr, (uint8_t[]){ reg_addr, val }, 2);
	if (ret != 2) {
		//printf("write error: %d\n", ret);
		return -1;
	}

	return 0;
}

int ioe_pwm_check_loading(struct ioexpander *ioe)
{
	uint8_t buf;
	int ret = read_regs(ioe, IOE_REG_PWMCON0, &buf, 1);
	if (ret != 0) {
		return -1;
	}

	return !!(buf & (1 << 6));
}

int ioe_pwm_load(struct ioexpander *ioe, bool wait)
{
#define IOE_PWM_LOAD_TIMEOUT_MS 50
	uint8_t buf;
	int ret = read_regs(ioe, IOE_REG_PWMCON0, &buf, 1);
	if (ret != 0) {
		return -1;
	}

	buf |= (1 << 6);

	ret = write_reg(ioe, IOE_REG_PWMCON0, buf);
	if (ret != 0) {
		return -1;
	}

	if (wait) {
		int i;
		for (i = 0; i < IOE_PWM_LOAD_TIMEOUT_MS; i++) {
			ret = ioe_pwm_check_loading(ioe);
			if (ret <= 0) {
				break;
			}
			sleep_ms(1);
		}
	}

	return ret;
}

int ioe_set_pwm_period(struct ioexpander *ioe, uint16_t period)
{
	int ret = write_reg(ioe, IOE_REG_PWMPL, period & 0xff);
	if (ret) {
		return -1;
	}

	ret = write_reg(ioe, IOE_REG_PWMPH, (period >> 8) & 0xff);
	if (ret) {
		return -1;
	}

	return ioe_pwm_load(ioe, true);
}

int ioe_set_pwm_divider(struct ioexpander *ioe, enum ioe_pwm_divider divider)
{
	return write_reg(ioe, IOE_REG_PWMCON1, divider);
}

struct ioe_hw_pin {
	uint8_t port     : 2;
	uint8_t pin      : 3;
	uint8_t has_pwm  : 1;
	uint8_t has_adc  : 1;
	uint8_t adc_chan : 3;
	uint8_t pwm_chan : 3;
	uint8_t pwm_reg;
};

const struct ioe_hw_pin hw_pins[] = {
	[0] = {
		.port = 1,
		.pin = 5,
		.has_pwm = 1,
		.pwm_chan = 5,
		.pwm_reg = IOE_REG_PIOCON1,
	},
	[1] = {
		.port = 1,
		.pin = 0,
		.has_pwm = 1,
		.pwm_chan = 2,
		.pwm_reg = IOE_REG_PIOCON0,
	},
	[2] = {
		.port = 1,
		.pin = 2,
		.has_pwm = 1,
		.pwm_chan = 0,
		.pwm_reg = IOE_REG_PIOCON0,
	},
	[3] = {
		.port = 1,
		.pin = 4,
		.has_pwm = 1,
		.pwm_chan = 1,
		.pwm_reg = IOE_REG_PIOCON1,
	},
	[4] = {
		.port = 0,
		.pin = 0,
		.has_pwm = 1,
		.pwm_chan = 3,
		.pwm_reg = IOE_REG_PIOCON0,
	},
	[5] = {
		.port = 0,
		.pin = 1,
		.has_pwm = 1,
		.pwm_chan = 4,
		.pwm_reg = IOE_REG_PIOCON0,
	},
	[6] = {
		.port = 1,
		.pin = 1,
		.has_pwm = 1,
		.has_adc = 1,
		.adc_chan = 7,
		.pwm_chan = 1,
		.pwm_reg = IOE_REG_PIOCON0,
	},
	[6] = {
		.port = 0,
		.pin = 3,
		.has_pwm = 1,
		.has_adc = 1,
		.adc_chan = 6,
		.pwm_chan = 5,
		.pwm_reg = IOE_REG_PIOCON0,
	},
	[7] = {
		.port = 0,
		.pin = 4,
		.has_pwm = 1,
		.has_adc = 1,
		.adc_chan = 5,
		.pwm_chan = 3,
		.pwm_reg = IOE_REG_PIOCON1,
	},
	[8] = {
		.port = 0,
		.pin = 4,
		.has_pwm = 1,
		.has_adc = 1,
		.adc_chan = 5,
		.pwm_chan = 3,
		.pwm_reg = IOE_REG_PIOCON1,
	},
	[9] = {
		.port = 3,
		.pin = 0,
		.has_adc = 1,
		.adc_chan = 1,
	},
	[10] = {
		.port = 0,
		.pin = 6,
		.has_adc = 1,
		.adc_chan = 3,
	},
	[11] = {
		.port = 0,
		.pin = 5,
		.has_pwm = 1,
		.has_adc = 1,
		.adc_chan = 4,
		.pwm_chan = 2,
		.pwm_reg = IOE_REG_PIOCON1,
	},
	[12] = {
		.port = 0,
		.pin = 7,
		.has_adc = 1,
		.adc_chan = 2,
	},
	[13] = {
		.port = 1,
		.pin = 7,
		.has_adc = 1,
		.adc_chan = 0,
	},
};

const uint8_t reg_port_pm1[] = { IOE_REG_P0M1, IOE_REG_P1M1, 0xff, IOE_REG_P3M1 };
const uint8_t reg_port_pm2[] = { IOE_REG_P0M2, IOE_REG_P1M2, 0xff, IOE_REG_P3M2 };
const uint8_t reg_pwm_l[] =    { IOE_REG_PWM0L, IOE_REG_PWM1L, IOE_REG_PWM2L, IOE_REG_PWM3L, IOE_REG_PWM4L, IOE_REG_PWM5L };
const uint8_t reg_pwm_h[] =    { IOE_REG_PWM0H, IOE_REG_PWM1H, IOE_REG_PWM2H, IOE_REG_PWM3H, IOE_REG_PWM4H, IOE_REG_PWM5H };

int ioe_set_pwm_duty(struct ioexpander *ioe, unsigned int pin, uint16_t duty)
{
	if (pin >= sizeof(hw_pins) / sizeof(hw_pins[0])) {
		return -1;
	}

	const struct ioe_hw_pin *hw_pin = &hw_pins[pin - 1];

	if (!hw_pin->has_pwm) {
		return -1;
	}

	int ret = write_reg(ioe, reg_pwm_l[hw_pin->pwm_chan], duty & 0xff);
	if (ret) {
		return -1;
	}

	ret = write_reg(ioe, reg_pwm_h[hw_pin->pwm_chan], (duty >> 8) & 0xff);
	if (ret) {
		return -1;
	}

	return ioe_pwm_load(ioe, true);
}

static int ioe_hw_pin_cfg_pwm(struct ioexpander *ioe, const struct ioe_hw_pin *hw_pin, bool enable_pwm)
{
	// Set PWM channel and start it running
	uint8_t tmp;
	int ret = read_regs(ioe, hw_pin->pwm_reg, &tmp, 1);
	if (ret != 0) {
		return ret;
	}

	if (enable_pwm) {
		tmp |= (1 << hw_pin->pwm_chan);
	} else {
		tmp &= ~(1 << hw_pin->pwm_chan);
	}

	ret = write_reg(ioe, hw_pin->pwm_reg, tmp);
	if (ret != 0) {
		return ret;
	}

	if (enable_pwm) {
		ret = read_regs(ioe, IOE_REG_PWMCON0, &tmp, 1);
		if (ret != 0) {
			return ret;
		}

		tmp |= (1 << 7);

		ret = write_reg(ioe, IOE_REG_PWMCON0, tmp);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

int ioe_set_pin_mode(struct ioexpander *ioe, unsigned int pin, enum ioe_pin_mode mode)
{
	int ret;

	if (pin >= sizeof(hw_pins) / sizeof(hw_pins[0])) {
		return -1;
	}

	const struct ioe_hw_pin *hw_pin = &hw_pins[pin - 1];

	if (hw_pin->has_pwm) {
		ret = ioe_hw_pin_cfg_pwm(ioe, hw_pin, mode == IOE_PIN_MODE_PWM);
		if (ret) {
			return ret;
		}
	}

	const uint8_t reg_pm1 = reg_port_pm1[hw_pin->port];
	const uint8_t reg_pm2 = reg_port_pm2[hw_pin->port];
	uint8_t tmp;

	// Set up PM
	ret = read_regs(ioe, reg_pm1, &tmp, 1);
	if (ret != 0) {
		return ret;
	}

	tmp &= ~(1 << hw_pin->pin);
	tmp |= ((mode >> 1) & 0x1) << hw_pin->pin;

	ret = write_reg(ioe, reg_pm1, tmp);
	if (ret != 0) {
		return ret;
	}

	ret = read_regs(ioe, reg_pm2, &tmp, 1);
	if (ret != 0) {
		return ret;
	}

	tmp &= ~(1 << hw_pin->pin);
	tmp |= (mode & 0x1) << hw_pin->pin;

	ret = write_reg(ioe, reg_pm2, tmp);
	if (ret != 0) {
		return ret;
	}

	// TODO: Set initial output state

	return 0;
}

int ioe_init(struct ioexpander *ioe,
		int (*i2c_write)(void *i2c_handle, uint8_t addr, const uint8_t *src, size_t len),
		int (*i2c_read)(void *i2c_handle, uint8_t addr, uint8_t *src, size_t len),
		void *i2c_handle)
{
	int ret;

	ioe->i2c_write_blocking = i2c_write;
	ioe->i2c_read_blocking = i2c_read;
	ioe->i2c_handle = i2c_handle;
	// TODO: ID re-assignment
	ioe->dev_addr = IOE_DEFAULT_ADDR;

	uint8_t buf[2] = { 0, 0 };
	ret = read_regs(ioe, IOE_REG_CHIP_ID_L, buf, 2);
	if (ret) {
		return ret;
	}

	if ((buf[0] != (IOE_CHIP_ID & 0xff)) || (buf[1] != ((IOE_CHIP_ID >> 8) & 0xff))) {
		return -1;
	}

	return 0;
}
