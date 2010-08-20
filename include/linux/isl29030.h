/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef _LINUX_ISL29030_H__
#define _LINUX_ISL29030_H__

#define	MANUAL		0
#define	AUTOMATIC	1
#define	MANUAL_SENSOR	2


#define LD_ISL29030_NAME "isl29030_als_ir"
#define FOPS_ISL29030_NAME "isl29030"

#define LD_ISL29030_ALLOWED_R_BYTES 1
#define LD_ISL29030_ALLOWED_W_BYTES 2
#define LD_ISL29030_MAX_RW_RETRIES 5
#define LD_ISL29030_I2C_RETRY_DELAY 10

#define ISL29030_CHIPID		0x00
#define ISL29030_CONFIGURE	0x01
#define ISL29030_INTERRUPT	0x02
#define ISL29030_PROX_LT	0x03
#define ISL29030_PROX_HT	0x04
#define ISL29030_ALSIR_TH1	0x05
#define ISL29030_ALSIR_TH2	0x06
#define ISL29030_ALSIR_TH3	0x07
#define ISL29030_PROX_DATA	0x08
#define ISL29030_ALSIR_DT1	0x09
#define ISL29030_ALSIR_DT2	0x0a

#define ISL29030_LOW_LUX_TRIGGER 0x007d

#define ISL29030_ALS_MASK	0x08
#define ISL29030_PROX_MASK	0x80

/* unit = millimeter */
#define PROXIMITY_NEAR	30		/* prox close threshold is 22-70mm */
#define PROXIMITY_FAR	2147483647	/* (2^31)-1 */


#ifdef __KERNEL__

struct isl29030_zone_conv {
	u16 lower_threshold;
	u16 upper_threshold;
};

struct isl29030_als_zone_data {
	u8  als_low_threshold;
	u8  als_high_low_threshold;
	u8  als_high_threshold;
};

struct isl29030_platform_data {
	u8	configure;
	u8	interrupt_cntrl;
	u8	prox_lower_threshold;
	u8	prox_higher_threshold;
	u8	als_ir_low_threshold;
	u8	als_ir_high_low_threshold;
	u8	als_ir_high_threshold;
	u32 lens_percent_t;
	struct isl29030_als_zone_data *als_low_lux;
	struct isl29030_als_zone_data *als_high_lux;
	u8  num_of_low_zones;
	u8  num_of_high_zones;
	int (*power_on)(void);
	int (*power_off)(void);
} __attribute__ ((packed));

#endif	/* __KERNEL__ */

#define ISL29030_IO			0xA3

#define ISL29030_IOCTL_GET_ENABLE	_IOR(ISL29030_IO, 0x00, char)
#define ISL29030_IOCTL_SET_ENABLE	_IOW(ISL29030_IO, 0x01, char)

#endif	/* _LINUX_ISL29030_H__ */
