/* linux/include/mach/hsusb.h
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_HSUSB_H
#define __ASM_ARCH_MSM_HSUSB_H

#include <linux/types.h>

#define PHY_TYPE_MASK		0xFF
#define PHY_MODEL_MASK		0xFF00
#define PHY_TYPE(x)		((x) & PHY_TYPE_MASK)
#define PHY_MODEL(x)		((x) & PHY_MODEL_MASK)

#define USB_PHY_MODEL_65NM	0x100
#define USB_PHY_MODEL_180NM	0x200
#define USB_PHY_UNDEFINED	0x00
#define USB_PHY_INTEGRATED	0x01
#define USB_PHY_EXTERNAL	0x02

enum hsusb_phy_type {
	UNDEFINED,
	INTEGRATED,
	EXTERNAL,
};

struct usb_function_map {
	char name[20];
	unsigned bit_pos;
};

/* platform device data for msm_hsusb driver */
struct usb_composition {
	__u16   product_id;
	unsigned long functions;
#ifdef CONFIG_MACH_MOT
	char *config;
#endif
};

struct msm_hsusb_platform_data {
	__u16   version;
#ifdef CONFIG_USB_ANDROID
	int *phy_init_seq;
	void (*phy_reset)(void);
#endif
	unsigned phy_info;
	__u16   vendor_id;
#ifdef CONFIG_MACH_MOT
	__u16   product_id;
#endif
	char   	*product_name;
	char   	*serial_number;
	char   	*manufacturer_name;
	struct usb_composition *compositions;
	int num_compositions;
	struct usb_function_map *function_map;
	int num_functions;
	/* ULPI data pins used for LPM */
	unsigned ulpi_data_1_pin;
	unsigned ulpi_data_3_pin;
	/* gpio mux function used for LPM */
	int (*config_gpio)(int config);
	/* ROC info for AHB Mode */
	unsigned int soc_version;
};

#endif
