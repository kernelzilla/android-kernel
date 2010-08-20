/*
 * Platform data for Android USB
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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
#ifndef	__LINUX_USB_ANDROID_H
#define	__LINUX_USB_ANDROID_H
#define ANDROID_ADB		0x0001
#define ANDROID_MSC		0x0002
#define ANDROID_ACM_MODEM	0x0003
#define ANDROID_DIAG		0x0004
#define ANDROID_ACM_NMEA	0x0005
#define ANDROID_GENERIC_MODEM	0x0006
#define ANDROID_GENERIC_NMEA	0x0007
#define ANDROID_CDC_ECM		0x0008

struct android_usb_platform_data {
	/* USB device descriptor fields */
	__u16 vendor_id;

	/* Default product ID. */
	__u16 product_id;

	/* Default function map. */
	unsigned long functions;

	/* Product ID when adb is enabled. */
	__u16 adb_product_id;

	__u16 version;
	/* Fields for composition switch support */
	struct usb_composition *compositions;
	int num_compositions;

	char *product_name;
	char *manufacturer_name;
	char *serial_number;

	/* number of LUNS for mass storage function */
	int nluns;

	/* Factory Enabled Bit set on Powerup */
	int factory_enabled;
};

/* Platform data for "usb_mass_storage" driver.
 * Contains values for the SC_INQUIRY SCSI command. */
struct usb_mass_storage_platform_data {
	char *vendor;
	char *product;
	int release;
};
/* composition support structure */
struct usb_composition {
	__u16   product_id;
	unsigned long functions;
};

extern void android_usb_set_connected(int on);

#endif	/* __LINUX_USB_ANDROID_H */
