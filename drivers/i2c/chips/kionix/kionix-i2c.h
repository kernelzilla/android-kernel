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

#include "kionix-main.h"

#define ACCEL_IRQ_INT1			0x01
#define ACCEL_IRQ_INT2			0x02

/* registers 0x10 - 0x1F are reserved */
#define ACCEL_I2C_PORT_XOUT_H		0x00	/* x-axis position */
#define ACCEL_I2C_PORT_XOUT_L		0x01	/* x-axis position */
#define ACCEL_I2C_PORT_YOUT_H		0x02	/* y-axis position */
#define ACCEL_I2C_PORT_YOUT_L		0x03	/* y-axis position */
#define ACCEL_I2C_PORT_ZOUT_H		0x04	/* y-axis position */
#define ACCEL_I2C_PORT_ZOUT_L		0x05	/* y-axis position */
#define ACCEL_I2C_PORT_RESET		0x06	/* HP filter reset */
#define ACCEL_I2C_PORT_FF_INT		0x08	/* ctrl register 4 */
#define ACCEL_I2C_PORT_FF_DELAY		0x09	/* ctrl register 5 */
#define ACCEL_I2C_PORT_MOT_INT		0x0A	/* ctrl register 1 */
#define ACCEL_I2C_PORT_MOT_DELAY	0x0B	/* ctrl register 1 */
#define ACCEL_I2C_PORT_REGA		0x0E	/* ctrl register 2 */
#define ACCEL_I2C_PORT_REGB		0x0D	/* ctrl register 2 */
#define ACCEL_I2C_PORT_REGC		0x0C	/* ctrl register 3 */

#define PORT_MAPPER(a)			(a-ACCEL_I2C_PORT_REG1)
#define ACCEL_MAX_REGS			(PORT_MAPPER(ACCEL_I2C_PORT_INT2_DUR)+1)
//#define SUB_AUTO_INCREMENTED(reg)	((1<<7) | reg)
#define SUB_AUTO_INCREMENTED(reg)	(reg)

#define ACCEL_I_AM_THE_ONE		0x42	/* default content of CTRL_REGB */
#define ACCEL_I2C_ADDRESS		0x18

/* registers 0x00 - 0x0E are reserved */
#define ACCEL_I2C_PORT_WHO_AM_I		ACCEL_I2C_PORT_REGB	/* who am i (replies with ACCEL_I_AM_THE_ONE) */

/* Bit mask for different ctrl registers */
#define ACCEL_BM_NONE			0x00
#define ACCEL_BM_ALL			0xFF
#define ACCEL_BM_SCALE			0x30
#define ACCEL_BM_READY_X		0x01
#define ACCEL_BM_INTS			0x03
#define ACCEL_BM_READY_Z		0x04
#define ACCEL_BM_READY_XYZ		(ACCEL_BM_READY_X | ACCEL_BM_READY_Y | ACCEL_BM_READY_Z)
#define ACCEL_BM_ODR			0x18
#define ACCEL_BM_PWR			0x40
#define ACCEL_BM_ORIENT			0x0000FF00
#define ACCEL_BM_SELF			0x02

/* Device orientation */
#define ACCEL_DEV_ORIENT_NORMAL		0x48
#define ACCEL_DEV_ORIENT_UPSIDE		0x44
#define ACCEL_DEV_ORIENT_TILTED_LEFT	0x42
#define ACCEL_DEV_ORIENT_TILTED_RIGHT	0x41


extern int accel_param_debug;

int accel_i2c_who_am_i(void);
void accel_i2c_init(void);
int accel_i2c_read_axis_data(int *axis);
int accel_i2c_read_axis(int axis);
int accel_i2c_enable_axis(int axis);
int accel_i2c_self_test(int mode);
int accel_i2c_set_power_mode(int mode);
int accel_i2c_get_power_mode(int *dest);
int accel_i2c_read_register(unsigned char reg, int *dest);
int accel_i2c_read_control_registers(unsigned char reg[], int number);
int accel_i2c_update_register(unsigned char reg, unsigned char mask, unsigned char newval);
int accel_i2c_arm_interrupt(int irq, unsigned char cfg, unsigned char ths, unsigned char dur, unsigned char ctrl_reg3);
int accel_i2c_disarm_interrupt(int irq);

#ifdef __cplusplus
}
#endif
#endif

