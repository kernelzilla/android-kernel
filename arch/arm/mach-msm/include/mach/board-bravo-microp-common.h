/* arch/arm/mach-msm/board-bravo.h
 *
 * Copyright (C) 2009 HTC Corporation.
 * Author: Haley Teng <Haley_Teng@htc.com>
 * Copyright (C) 2010 Kali- <kalimero@ngi.it>
 * Copyright (C) 2010 Diogo Ferreira <diogo@underdev.org>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef _LINUX_BOARD_BRAVO_MICROP_COMMON_H
#define _LINUX_BOARD_BRAVO_MICROP_COMMON_H

#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/list.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>

#define MICROP_I2C_NAME "bravo-microp"

#define MICROP_LSENSOR_ADC_CHAN				6
#define MICROP_REMOTE_KEY_ADC_CHAN			7

#define MICROP_I2C_WCMD_MISC				0x20
#define MICROP_I2C_WCMD_SPI_EN				0x21
#define MICROP_I2C_WCMD_AUTO_BL_CTL			0x23
#define MICROP_I2C_RCMD_SPI_BL_STATUS			0x24
#define MICROP_I2C_WCMD_BUTTONS_LED_CTRL		0x25
#define MICROP_I2C_RCMD_VERSION				0x30
#define MICROP_I2C_WCMD_ADC_TABLE			0x42
#define MICROP_I2C_WCMD_LED_MODE			0x53
#define MICROP_I2C_RCMD_GREEN_LED_REMAIN_TIME		0x54
#define MICROP_I2C_RCMD_AMBER_LED_REMAIN_TIME		0x55
#define MICROP_I2C_RCMD_BLUE_LED_REMAIN_TIME		0x57
#define MICROP_I2C_WCMD_READ_ADC_VALUE_REQ		0x60
#define MICROP_I2C_RCMD_ADC_VALUE			0x62
#define MICROP_I2C_WCMD_REMOTEKEY_TABLE			0x63
#define MICROP_I2C_WCMD_LCM_REGISTER			0x70
#define MICROP_I2C_WCMD_GSENSOR_REG			0x73
#define MICROP_I2C_WCMD_GSENSOR_REG_DATA_REQ		0x74
#define MICROP_I2C_RCMD_GSENSOR_REG_DATA		0x75
#define MICROP_I2C_WCMD_GSENSOR_DATA_REQ		0x76
#define MICROP_I2C_RCMD_GSENSOR_X_DATA			0x77
#define MICROP_I2C_RCMD_GSENSOR_Y_DATA			0x78
#define MICROP_I2C_RCMD_GSENSOR_Z_DATA			0x79
#define MICROP_I2C_RCMD_GSENSOR_DATA			0x7A
#define MICROP_I2C_WCMD_OJ_REG				0x7B
#define MICROP_I2C_WCMD_OJ_REG_DATA_REQ			0x7C
#define MICROP_I2C_RCMD_OJ_REG_DATA			0x7D
#define MICROP_I2C_WCMD_OJ_POS_DATA_REQ			0x7E
#define MICROP_I2C_RCMD_OJ_POS_DATA			0x7F
#define MICROP_I2C_WCMD_GPI_INT_CTL_EN			0x80
#define MICROP_I2C_WCMD_GPI_INT_CTL_DIS			0x81
#define MICROP_I2C_RCMD_GPI_INT_STATUS			0x82
#define MICROP_I2C_RCMD_GPI_STATUS			0x83
#define MICROP_I2C_WCMD_GPI_INT_STATUS_CLR		0x84
#define MICROP_I2C_RCMD_GPI_INT_SETTING			0x85
#define MICROP_I2C_RCMD_REMOTE_KEYCODE			0x87
#define MICROP_I2C_WCMD_REMOTE_KEY_DEBN_TIME		0x88
#define MICROP_I2C_WCMD_REMOTE_PLUG_DEBN_TIME		0x89
#define MICROP_I2C_WCMD_SIMCARD_DEBN_TIME		0x8A
#define MICROP_I2C_WCMD_GPO_LED_STATUS_EN		0x90
#define MICROP_I2C_WCMD_GPO_LED_STATUS_DIS		0x91
#define MICROP_I2C_WCMD_OJ_INT_STATUS			0xA8

/* Desire - verified in 2.6.29 */
#define IRQ_OJ						(1<<12)
#define IRQ_GSENSOR					(1<<10)
#define IRQ_LSENSOR					(1<<9)
#define IRQ_REMOTEKEY					(1<<7)
#define IRQ_HEADSETIN					(1<<2)
#define IRQ_SDCARD					(1<<0)

#define SPI_GSENSOR					(1 << 0)
#define SPI_LCM						(1 << 1)
#define SPI_OJ						(1 << 2)

/* Optical Joystick callbacks */
struct microp_oj_callback {
	void (*oj_init)(void);
	void (*oj_intr)(void);
};
int microp_register_oj_callback(struct microp_oj_callback *oj);

/* I2C functions for drivers */
int microp_i2c_read(uint8_t addr, uint8_t *data, int length);
int microp_i2c_write(uint8_t addr, uint8_t *data, int length);

int microp_spi_vote_enable(int spi_device, uint8_t enable);

#endif /* _LINUX_BOARD_BRAVO_MICROP_COMMON_H */
