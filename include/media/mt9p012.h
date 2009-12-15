/*
 * mt9p012.h - Register definitions for the MT9P012 camera sensor.
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Sergio Aguirre <saaguirre@ti.com>
 * 	Martinez Leonides
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef MT9P012_H
#define MT9P012_H


#define MT9P012_I2C_ADDR		0x10

/**
 * struct mt9p012_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @default_regs: Default registers written after power-on or reset.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct mt9p012_platform_data {
	int (*power_set)(struct device* dev, enum v4l2_power power);
	u32 (*set_xclk)(u32 xclkfreq);
	int (*priv_data_set)(void *);
};

#endif /* ifndef MT9P012_H */
