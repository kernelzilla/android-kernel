/*
 * drivers/media/video/motcam.h
 *
 * Author: Elena Grimes (motorola.com)
 *
 *
 *
 */

#ifndef MOTCAM_H
#define MOTCAM_H

 /**
 * struct mt9p012_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @default_regs: Default registers written after power-on or reset.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct motcam_platform_data {
	int (*power_set)(enum v4l2_power power);
	const struct mt9p012_reg *default_regs;
	int (*ifparm)(struct v4l2_ifparm *p);
	int (*priv_data_set)(void *);
};

#endif /* ifndef MOTCAM_H */
