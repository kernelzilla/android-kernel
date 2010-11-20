/**
 *	Morrison accelerometer I2C protocol driver
 *
 *	Copyright (C) 2008  Motorola, Inc.
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation version 2 of the License.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	GNU General Public License <http://www.gnu.org/licenses/gpl-2.0.html>
 */

#ifndef ACCEL_I2C_H_
#define ACCEL_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "accel-main.h"

#define ACCEL_IRQ_INT1			0x01
#define ACCEL_IRQ_INT2			0x02

#define ACCEL_I_AM_THE_ONE		0x32	/* LIS331DLH's id */
#define ACCEL_I2C_ADDRESS		0x18

/* registers 0x00 - 0x0E are reserved */
#define ACCEL_I2C_PORT_WHO_AM_I		0x0F	/* who am i (replies with ACCEL_I_AM_THE_ONE) */

/* registers 0x10 - 0x1F are reserved */
#define ACCEL_I2C_PORT_REG1		0x20	/* ctrl register 1 */
#define ACCEL_I2C_PORT_REG2		0x21	/* ctrl register 2 */
#define ACCEL_I2C_PORT_REG3		0x22	/* ctrl register 3 */
#define ACCEL_I2C_PORT_REG4		0x23	/* ctrl register 4 */
#define ACCEL_I2C_PORT_REG5		0x24	/* ctrl register 5 */
#define ACCEL_I2C_PORT_FILTER_RESET	0x25	/* HP filter reset */
#define ACCEL_I2C_PORT_REFERENCE	0x26	/* reference */
#define ACCEL_I2C_PORT_STATUS		0x27	/* status register */
#define ACCEL_I2C_PORT_XOUT_H		0x29	/* x-axis position */
#define ACCEL_I2C_PORT_XOUT_L		0x28	/* x-axis position */
#define ACCEL_I2C_PORT_YOUT_H		0x2B	/* y-axis position */
#define ACCEL_I2C_PORT_YOUT_L		0x2A	/* y-axis position */
#define ACCEL_I2C_PORT_ZOUT_H		0x2D	/* y-axis position */
#define ACCEL_I2C_PORT_ZOUT_L		0x2C	/* y-axis position */
/* registers 0x2E - 0x2F are reserved */
#define ACCEL_I2C_PORT_INT1_CFG		0x30	/* config register for interrupt 1 source */
#define	ACCEL_I2C_PORT_INT1_SOURCE	0x31	/* interrupt 1 source */
#define ACCEL_I2C_PORT_INT1_THS		0x32	/* interrupt 1 threshold */
#define ACCEL_I2C_PORT_INT1_DUR		0x33	/* duration */
#define ACCEL_I2C_PORT_INT2_CFG		0x34	/* config register for interrupt 2 source */
#define ACCEL_I2C_PORT_INT2_SOURCE	0x35	/* interrupt 2 source */
#define ACCEL_I2C_PORT_INT2_THS		0x36	/* interrupt 2 threshold */
#define ACCEL_I2C_PORT_INT2_DUR		0x37	/* duration */
/* registers 0x38 - 0x3F are reserved */

#define PORT_MAPPER(a)			(a-ACCEL_I2C_PORT_REG1)
#define ACCEL_MAX_REGS			(PORT_MAPPER(ACCEL_I2C_PORT_INT2_DUR)+1)
#define SUB_AUTO_INCREMENTED(reg)	((1<<7) | reg)

/* Bit mask for different ctrl registers */
#define ACCEL_BM_NONE			0x00
#define ACCEL_BM_ALL			0xFF
#define ACCEL_BM_I1_CFG			0x03
#define ACCEL_BM_I2_CFG			0X18
#define ACCEL_BM_LATCH2			0x20
#define ACCEL_BM_LATCH1			0x04
#define ACCEL_BM_SCALE			0x30
#define ACCEL_BM_READY_X		0x01
#define ACCEL_BM_READY_Y		0x02
#define ACCEL_BM_READY_Z		0x04
#define ACCEL_BM_READY_XYZ		(ACCEL_BM_READY_X | ACCEL_BM_READY_Y | ACCEL_BM_READY_Z)
#define ACCEL_BM_ODR			0x18
#define ACCEL_BM_PWR			0xE0
#define ACCEL_BM_SELF			0x02

#define ACCEL_BM_ORIENT			0x0000FF00

/* Human readable interrupts config macros */
#define ACCEL_RAW_DATA_OFF		0x18
#define ACCEL_RAW_DATA_ON		0x10
#define ACCEL_PORT_LAND_OFF		0x03
#define ACCEL_PORT_LAND_ON		0x01


int accel_i2c_who_am_i(void);
#ifdef CONFIG_MACH_MOT
int accel_i2c_read_xyz(int *axis);
#if 0
int accel_i2c_read_axis(int axis);
#endif
#else
int accel_i2c_read_axis_data(int *axis);
int accel_i2c_read_axis(int axis);
#endif
int accel_i2c_enable_axis(int axis);
int accel_i2c_self_test(int mode);
int accel_i2c_set_power_mode(int mode);
int accel_i2c_get_power_mode(int *dest);
int accel_i2c_set_orient_irq(int state);
int accel_i2c_read_register(unsigned char reg, int *dest);
int accel_i2c_write_register(unsigned char reg, unsigned char val);
int accel_i2c_read_control_registers(unsigned char reg[], int number);
int accel_i2c_update_register(unsigned char reg, unsigned char mask, unsigned char newval);
int accel_i2c_arm_interrupt(int irq, unsigned char cfg, unsigned char ths, unsigned char dur);
int accel_i2c_disarm_interrupt(int irq);
int accel_i2c_set_config(int mode, int odr, int fs, int interval);

#ifdef __cplusplus
}
#endif
#endif

