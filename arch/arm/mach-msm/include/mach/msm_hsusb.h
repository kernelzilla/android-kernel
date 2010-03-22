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

#define PHY_TYPE_MASK		0x0F
#define PHY_TYPE_MODE		0xF0
#define PHY_MODEL_MASK		0xFF00
#define PHY_TYPE(x)		((x) & PHY_TYPE_MASK)
#define PHY_MODEL(x)		((x) & PHY_MODEL_MASK)

#define USB_PHY_MODEL_65NM	0x100
#define USB_PHY_MODEL_180NM	0x200
#define USB_PHY_MODEL_45NM	0x400
#define USB_PHY_UNDEFINED	0x00
#define USB_PHY_INTEGRATED	0x01
#define USB_PHY_EXTERNAL	0x02
#define USB_PHY_SERIAL_PMIC     0x04

#define REQUEST_STOP		0
#define REQUEST_START		1
#define REQUEST_RESUME		2

enum hsusb_phy_type {
	UNDEFINED,
	INTEGRATED,
	EXTERNAL,
};

struct usb_function_map {
	char name[20];
	unsigned bit_pos;
};

#ifndef CONFIG_USB_ANDROID
/* platform device data for msm_hsusb driver */
struct usb_composition {
	__u16   product_id;
	unsigned long functions;
};
#endif

#ifdef CONFIG_USB_GADGET_MSM_72K
enum chg_type {
	USB_CHG_TYPE__SDP,
	USB_CHG_TYPE__CARKIT,
	USB_CHG_TYPE__WALLCHARGER,
	USB_CHG_TYPE__INVALID
};
#endif

struct msm_hsusb_gadget_platform_data {
	int *phy_init_seq;
	void (*phy_reset)(void);

	u32 swfi_latency;
	int self_powered;
};

struct msm_hsusb_platform_data {
	__u16   version;
	unsigned phy_info;
	__u16   vendor_id;
	char   	*product_name;
	char   	*serial_number;
	char   	*manufacturer_name;
	struct usb_composition *compositions;
	int num_compositions;
	struct usb_function_map *function_map;
	int num_functions;
	/* gpio mux function used for LPM */
	int (*config_gpio)(int config);
	/* ROC info for AHB Mode */
	unsigned int soc_version;

	int (*phy_reset)(void __iomem *addr);

	unsigned int core_clk;

	int vreg5v_required;

	u32 swfi_latency;
};

struct msm_otg_platform_data {
	int (*rpc_connect)(int);
	int (*phy_reset)(void __iomem *);
	unsigned int core_clk;
	int pmic_vbus_irq;

	/* pmic notfications apis */
	int (*pmic_notif_init) (void);
	void (*pmic_notif_deinit) (void);
	int (*pmic_register_vbus_sn) (void (*callback)(int online));
	void (*pmic_unregister_vbus_sn) (void (*callback)(int online));
	int (*pmic_enable_ldo) (int);
};

struct msm_usb_host_platform_data {
	unsigned phy_info;
	int (*phy_reset)(void __iomem *addr);
	void (*config_gpio)(unsigned int config);
	void (*vbus_power) (unsigned phy_info, int on);
};

#endif
