/*
 * Gadget Driver for Android
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/device.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#include <linux/usb/android.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/io.h>

#include "f_mass_storage.h"
#include "f_adb.h"
#include "f_usbnet.h"
#include "f_acm.h"
#include "f_mtp.h"
#include "f_mot_android.h"
#include "u_serial.h"

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("Motorola Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

static DECLARE_WAIT_QUEUE_HEAD(device_mode_change_wait_q);

/* Default vendor and product IDs, overridden by platform data */
#define VENDOR_ID		0x22b8
#define PRODUCT_ID		0x41da
#define ADB_PRODUCT_ID		0x41da

struct device_pid_vid {
	char *name;
	u32 type;
	int vid;
	int pid;
	char *config_name;
	int class;
	int subclass;
	int protocol;
};

#define MAX_DEVICE_TYPE_NUM   20
#define MAX_DEVICE_NAME_SIZE  30
static struct device_pid_vid mot_android_vid_pid[MAX_DEVICE_TYPE_NUM] = {
	{"msc", MSC_TYPE_FLAG, 0x22b8, 0x41d9, "Motorola Config 14",
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE},
	{"cdrom", CDROM_TYPE_FLAG, 0x22b8, 0x41de, "Motorola CDROM Device",
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE},
	{"msc_adb", MSC_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8, 0x41db,
	 "Motorola Config 42", USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE},
	{"eth", ETH_TYPE_FLAG, 0x22b8, 0x41d4, "Motorola Config 13",
	 USB_CLASS_COMM, USB_CLASS_COMM, USB_CLASS_PER_INTERFACE},
	{"mtp", MTP_TYPE_FLAG, 0x22b8, 0x41D6, "Motorola Config 15",
	 USB_CLASS_PER_INTERFACE,
	 USB_CLASS_PER_INTERFACE, USB_CLASS_PER_INTERFACE},
	{"acm", ACM_TYPE_FLAG, 0x22b8, 0x6422, "Motorola Config 1",
	 USB_CLASS_COMM, USB_CLASS_COMM, USB_CLASS_PER_INTERFACE},
	{"eth_adb", ETH_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8, 0x41d4,
	 "Motorola Android Composite Device"},
	{"acm_eth_mtp", ACM_TYPE_FLAG | ETH_TYPE_FLAG | MTP_TYPE_FLAG, 0x22b8,
	 0x41d8, "Motorola Config 30", USB_CLASS_VENDOR_SPEC,
	 USB_CLASS_VENDOR_SPEC, USB_CLASS_VENDOR_SPEC},
	{"mtp_adb", MTP_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8, 0x41dc,
	 "Motorola Config 32", USB_CLASS_VENDOR_SPEC,
	 USB_CLASS_VENDOR_SPEC, USB_CLASS_VENDOR_SPEC},
	{"acm_eth_mtp_adb",
	 ACM_TYPE_FLAG | ETH_TYPE_FLAG | MTP_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8,
	 0x41da, "Motorola Config 31", USB_CLASS_VENDOR_SPEC,
	 USB_CLASS_VENDOR_SPEC, USB_CLASS_VENDOR_SPEC},
	{"acm_eth_adb", ACM_TYPE_FLAG | ETH_TYPE_FLAG | ADB_TYPE_FLAG, 0x22b8,
	 0x41e2, "Motorola Android Composite Device"},
	{"msc_eth", MSC_TYPE_FLAG | ETH_TYPE_FLAG, 0x22b8, 0x41d4,
	 "Motorola Android Composite Device"},
	{"msc_adb_eth", MSC_TYPE_FLAG | ADB_TYPE_FLAG | ETH_TYPE_FLAG, 0x22b8,
	 0x41d4, "Motorola Android Composite Device"},
	{}
};

static int g_device_type;
static atomic_t device_mode_change_excl;

struct android_dev {
	struct usb_gadget *gadget;
	struct usb_composite_dev *cdev;

	int product_id;
	int adb_product_id;
	int version;
	int factory_enabled;
	int nluns;
};

static atomic_t adb_enable_excl;
static struct android_dev *_android_dev;

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2
#define STRING_CONFIG_IDX		3

/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",
	[STRING_CONFIG_IDX].s = "Motorola Android Composite Device",
	{}			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language = 0x0409,	/* en-us */
	.strings = strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength = sizeof(device_desc),
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = __constant_cpu_to_le16(0x0200),
	.bDeviceClass = USB_CLASS_VENDOR_SPEC,
	.bDeviceSubClass = USB_CLASS_VENDOR_SPEC,
	.bDeviceProtocol = USB_CLASS_VENDOR_SPEC,
	.idVendor = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations = 1,
};

void android_usb_set_connected(int connected)
{
	if (_android_dev && _android_dev->cdev && _android_dev->cdev->gadget) {
		printk(KERN_INFO "USB Driver - Notification from CPCAP for %s \n",
			 connected ? "attach" : "detach");
		/*Disconnect so that USBD can control the connection */
		if (connected)
			usb_gadget_disconnect(_android_dev->cdev->gadget);
	}
}

static int __init android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;
	int ret;

	if (dev->factory_enabled)
		return usbnet_function_add(dev->cdev, c);

	/* the same sequence as force_reenumeration() */
	ret = mass_storage_function_add(dev->cdev, c, dev->nluns);
	if (ret)
		return ret;
	ret = acm_function_add(dev->cdev, c);
	if (ret)
		return ret;
	ret = usbnet_function_add(dev->cdev, c);
	if (ret)
		return ret;
	ret = mtp_function_add(dev->cdev, c);
	if (ret)
		return ret;
	ret = adb_function_add(dev->cdev, c);

	return ret;
}

static int android_setup_config(struct usb_configuration *c,
				const struct usb_ctrlrequest *ctrl);
static int usb_device_cfg_flag;
static int usb_get_desc_flag;
static int usb_data_transfer_flag;

static struct usb_configuration android_config_driver = {
	.label = "android",
	.bind = android_bind_config,
	.setup = android_setup_config,
	.bConfigurationValue = 1,
	.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower = 0xFA,	/* 500ma */
};

int get_func_thru_config(int mode)
{
	int i;
	char name[50];

	memset(name, 0, 50);
	sprintf(name, "Motorola Config %d", mode);
	for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
		if (!mot_android_vid_pid[i].config_name)
			break;
		if (!strcmp(mot_android_vid_pid[i].config_name, name))
			return i;
	}
	return -1;
}

static int pc_mode_switch_flag;

void mode_switch_cb(int mode)
{
	pc_mode_switch_flag = mode;
	wake_up_interruptible(&device_mode_change_wait_q);
}

static int android_generic_setup(struct usb_configuration *c,
				const struct usb_ctrlrequest *ctrl)
{
	int	value = -EOPNOTSUPP;
	u16     wIndex = le16_to_cpu(ctrl->wIndex);
	u16     wValue = le16_to_cpu(ctrl->wValue);
	u16     wLength = le16_to_cpu(ctrl->wLength);
	struct usb_composite_dev *cdev = c->cdev;
	struct usb_request	*req = cdev->req;

	switch (ctrl->bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_VENDOR:
		switch (ctrl->bRequest) {
		case 1:
			if ((wValue == 0) && (wLength == 0)) {
				mode_switch_cb(wIndex);
				value = 0;
				req->zero = 0;
				req->length = value;
				if (usb_ep_queue(cdev->gadget->ep0, req,
					GFP_ATOMIC))
					printk(KERN_ERR "ep0 in queue failed\n");
			}
		break;
		default:
			break;
		}
	default:
		break;
	}
	return value;
}

static int android_setup_config(struct usb_configuration *c,
				const struct usb_ctrlrequest *ctrl)
{
	int i, ret = -EOPNOTSUPP;

	ret = android_generic_setup(c, ctrl);
	if (ret >= 0)
		return ret;

	for (i = 0; i < android_config_driver.next_interface_id; i++)
		if (android_config_driver.interface[i]->setup) {
			ret =
			    android_config_driver.interface[i]->
			    setup(android_config_driver.interface[i], ctrl);
			if (ret >= 0)
				return ret;
		}
	return ret;
}

void usb_data_transfer_callback(void)
{
	if (usb_data_transfer_flag == 0) {
		usb_data_transfer_flag = 1;
		usb_get_desc_flag = 1;
		wake_up_interruptible(&device_mode_change_wait_q);
	}
}

void usb_interface_enum_cb(int flag)
{
	usb_device_cfg_flag |= flag;
	if (usb_device_cfg_flag == g_device_type)
		wake_up_interruptible(&device_mode_change_wait_q);
}

static int __init android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget *gadget = cdev->gadget;
	int gcnum;
	int id;
	int ret;

	dev->gadget = gadget;
	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_CONFIG_IDX].id = id;
	android_config_driver.iConfiguration = id;


	/* double check to move this call to f_acm.c??? */
	gserial_setup(gadget, 1);

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		printk(KERN_ERR "usb_add_config failed\n");
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			   longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;

	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name = "android_usb",
	.dev = &device_desc,
	.strings = dev_strings,
	.bind = android_bind,
};

static void get_device_pid_vid(int type, int *pid, int *vid)
{
	int i;

	*vid = 0;
	*pid = 0;
	for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
		if (mot_android_vid_pid[i].type == type) {
			*pid = mot_android_vid_pid[i].pid;
			*vid = mot_android_vid_pid[i].vid;
			break;
		}
	}
}

int get_func_thru_type(int type)
{
	int i;

	for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
		if (mot_android_vid_pid[i].type == type)
			return i;
	}
	return -1;
}

static void force_reenumeration(struct android_dev *dev, int dev_type)
{
	int vid, pid, i, temp_enabled;
	struct usb_function *f;

	/* using other namespace ??? */
	usb_device_cfg_flag = 0;
	usb_get_desc_flag   = 0;
	usb_data_transfer_flag = 0;
	pc_mode_switch_flag = 0;

	get_device_pid_vid(dev_type, &pid, &vid);
	device_desc.idProduct = __constant_cpu_to_le16(pid);
	device_desc.idVendor = __constant_cpu_to_le16(vid);

	for (i = 0; i < MAX_CONFIG_INTERFACES; i++)
		android_config_driver.interface[i] = 0;

	/* clear all intefaces */
	android_config_driver.next_interface_id = 0;

	if (dev->factory_enabled) {
		f = usbnet_function_enable(1,
				   android_config_driver.next_interface_id);
		android_config_driver.interface[android_config_driver.
						next_interface_id] = f;
		android_config_driver.next_interface_id++;
	} else {
		temp_enabled = dev_type & (MSC_TYPE_FLAG | CDROM_TYPE_FLAG);
		f = msc_function_enable(temp_enabled,
				android_config_driver.next_interface_id);
		if (temp_enabled) {
			android_config_driver.interface[android_config_driver.
						next_interface_id] = f;
			android_config_driver.next_interface_id++;
		}

		temp_enabled = dev_type & ACM_TYPE_FLAG;
		f = acm_function_enable(temp_enabled,
				android_config_driver.next_interface_id);
		if (temp_enabled) {
			android_config_driver.interface[android_config_driver.
							next_interface_id] = f;
			android_config_driver.next_interface_id++;
			android_config_driver.interface[android_config_driver.
							next_interface_id] = f;
			android_config_driver.next_interface_id++;
		}

		temp_enabled = dev_type & ETH_TYPE_FLAG;
		f = usbnet_function_enable(temp_enabled,
				   android_config_driver.next_interface_id);
		if (temp_enabled) {
			android_config_driver.interface[android_config_driver.
							next_interface_id] = f;
			android_config_driver.next_interface_id++;
		}

		temp_enabled = dev_type & MTP_TYPE_FLAG;
		f = mtp_function_enable(temp_enabled,
				android_config_driver.next_interface_id);
		if (temp_enabled) {
			android_config_driver.interface[android_config_driver.
							next_interface_id] = f;
			android_config_driver.next_interface_id++;
		}

		temp_enabled = dev_type & ADB_TYPE_FLAG;
		f = adb_function_enable_id(temp_enabled,
				   android_config_driver.next_interface_id);
		if (temp_enabled) {
			android_config_driver.interface[android_config_driver.
							next_interface_id] = f;
			android_config_driver.next_interface_id++;
		}
	}

	if (dev->cdev) {
		dev->cdev->desc.idProduct = device_desc.idProduct;
		dev->cdev->desc.idVendor = device_desc.idVendor;
		i = get_func_thru_type(dev_type);
		if (i != -1) {
			dev->cdev->desc.bDeviceClass =
				mot_android_vid_pid[i].class;
			dev->cdev->desc.bDeviceSubClass =
				mot_android_vid_pid[i].subclass;
			dev->cdev->desc.bDeviceProtocol =
				mot_android_vid_pid[i].protocol;
		}

	}

	if (dev->cdev && dev->cdev->gadget)  {
		/* dev->cdev->gadget->speed != USB_SPEED_UNKNOWN ? */
		usb_gadget_disconnect(dev->cdev->gadget);
		msleep(10);
		usb_gadget_connect(dev->cdev->gadget);
	}
}

static int adb_mode_changed_flag;

static int adb_enable_open(struct inode *ip, struct file *fp)
{
	struct android_dev *dev = _android_dev;

	if (atomic_inc_return(&adb_enable_excl) != 1) {
		atomic_dec(&adb_enable_excl);
		return -EBUSY;
	}

	if (dev->factory_enabled)
		return 0;

	adb_mode_changed_flag = 1;
	wake_up_interruptible(&device_mode_change_wait_q);

	return 0;
}

static int adb_enable_release(struct inode *ip, struct file *fp)
{
	struct android_dev *dev = _android_dev;

	atomic_dec(&adb_enable_excl);
	if (dev->factory_enabled)
		return 0;

	adb_mode_changed_flag = 1;
	wake_up_interruptible(&device_mode_change_wait_q);

	return 0;
}

static const struct file_operations adb_enable_fops = {
	.owner = THIS_MODULE,
	.open = adb_enable_open,
	.release = adb_enable_release,
};

static struct miscdevice adb_enable_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "android_adb_enable",
	.fops = &adb_enable_fops,
};

/*
 * Device is used for USB mode switch
 */
static int device_mode_change_open(struct inode *ip, struct file *fp)
{
	if (atomic_inc_return(&device_mode_change_excl) != 1) {
		atomic_dec(&device_mode_change_excl);
		return -EBUSY;
	}

	return 0;
}

static int device_mode_change_release(struct inode *ip, struct file *fp)
{
	atomic_dec(&device_mode_change_excl);
	return 0;
}

static ssize_t
device_mode_change_write(struct file *file, const char __user *buffer,
			 size_t count, loff_t *ppos)
{
	unsigned char cmd[MAX_DEVICE_NAME_SIZE + 1];
	int cnt = MAX_DEVICE_NAME_SIZE;
	int i, temp_device_type;

	if (count <= 0)
		return 0;

	if (cnt > count)
		cnt = count;

	if (copy_from_user(cmd, buffer, cnt))
		return -EFAULT;
	cmd[cnt] = 0;

	printk(KERN_DEBUG "%s cmd=%s\n", __func__, cmd);

	/* USB cable detached Command */
	if (strncmp(cmd, "usb_cable_detach", 16) == 0) {
		usb_data_transfer_flag = 0;
		g_device_type = 0;
		usb_device_cfg_flag = 0;
		usb_get_desc_flag   = 0;
		usb_gadget_disconnect(_android_dev->gadget);
		return count;
	}

	/* USB connect/disconnect Test Commands */
	if (strncmp(cmd, "usb_connect", 11) == 0) {
		usb_gadget_connect(_android_dev->gadget);
		return count;
	}
	if (strncmp(cmd, "usb_disconnect", 14) == 0) {
		usb_gadget_disconnect(_android_dev->gadget);
		return count;
	}

	for (i = 0; i < MAX_DEVICE_TYPE_NUM; i++) {
		if (mot_android_vid_pid[i].name == NULL)
			return count;
		if (strlen(mot_android_vid_pid[i].name) > cnt)
			continue;
		if (strncmp(cmd, mot_android_vid_pid[i].name, cnt - 1) == 0) {
			temp_device_type = mot_android_vid_pid[i].type;
			strings_dev[STRING_CONFIG_IDX].s =
			mot_android_vid_pid[i].config_name;
			break;
		}
	}

	if (i == MAX_DEVICE_TYPE_NUM)
		return count;

	if (temp_device_type == g_device_type)
		return count;

	g_device_type = temp_device_type;
	force_reenumeration(_android_dev, g_device_type);
	return count;
}

static int event_pending(void)
{
	if ((usb_device_cfg_flag == g_device_type) && (g_device_type != 0))
		return 1;
	else if (adb_mode_changed_flag)
		return 1;
	else if (usb_get_desc_flag)
		return 1;
	else if (pc_mode_switch_flag)
		return 1;
	else
		return 0;
}

static unsigned int device_mode_change_poll(struct file *file,
					    struct poll_table_struct *wait)
{
	poll_wait(file, &device_mode_change_wait_q, wait);

	if (event_pending())
		return POLLIN | POLLRDNORM;
	else
		return 0;
}

static ssize_t device_mode_change_read(struct file *file, char *buf,
				       size_t count, loff_t *ppos)
{
	int ret, size, cnt;
	/* double check last zero */
	unsigned char no_changed[] = "none:\0";
	unsigned char adb_en_str[] = "adb_enable:\0";
	unsigned char adb_dis_str[] = "adb_disable:\0";
	unsigned char enumerated_str[] = "enumerated\0";
	unsigned char get_desc_str[] = "get_desc\0";
	unsigned char modswitch_str[50];

	/* Message format example:
	* none:adb_enable:enumerated
	*/

	if (!event_pending())
		return 0;

	/* append PC request mode */
	if (!pc_mode_switch_flag) {
		size = strlen(no_changed);
		ret = copy_to_user(buf, no_changed, size);
	} else {
		memset(modswitch_str, 0, 50);
		ret = get_func_thru_config(pc_mode_switch_flag);
		pc_mode_switch_flag = 0;
		if (ret == -1) {
			size = strlen(no_changed);
			ret = copy_to_user(buf, no_changed, size);
		} else {
			sprintf(modswitch_str, "%s",
				 mot_android_vid_pid[ret].name);
			strcat(modswitch_str, ":");
			size = strlen(modswitch_str);
			ret = copy_to_user(buf, modswitch_str, size);
		}
	}
	cnt = size;
	buf += size;

	/* append ADB status */
	if (!adb_mode_changed_flag) {
		size = strlen(no_changed);
		ret = copy_to_user(buf, no_changed, size);
	} else {
		if (atomic_read(&adb_enable_excl)) {
			size = strlen(adb_en_str);
			ret = copy_to_user(buf, adb_en_str, size);
		} else {
			size = strlen(adb_dis_str);
			ret = copy_to_user(buf, adb_dis_str, size);
		}
		adb_mode_changed_flag = 0;
	}
	cnt += size;
	buf += size;

	/* append USB enumerated state */
	if ((usb_device_cfg_flag == g_device_type) && (g_device_type != 0)) {
		usb_device_cfg_flag = 0;
		size = strlen(enumerated_str);
		ret += copy_to_user(buf, enumerated_str, size);

	} else {
		if (usb_get_desc_flag == 1) {
			usb_get_desc_flag = 0;
			size = strlen(get_desc_str);
			ret += copy_to_user(buf, get_desc_str, size);
		} else {
			size = strlen(no_changed) - 1;
			ret += copy_to_user(buf, no_changed, size);
		}
	}
	cnt += size;

	return cnt;
}

static const struct file_operations device_mode_change_fops = {
	.owner = THIS_MODULE,
	.open = device_mode_change_open,
	.write = device_mode_change_write,
	.poll = device_mode_change_poll,
	.read = device_mode_change_read,
	.release = device_mode_change_release,
};

static struct miscdevice mode_change_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "usb_device_mode",
	.fops = &device_mode_change_fops,
};

static int __init android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;

	if (pdata) {
		if (pdata->vendor_id)
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) {
			dev->product_id = pdata->product_id;
			device_desc.idProduct =
				__constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->adb_product_id)
			dev->adb_product_id = pdata->adb_product_id;
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
				pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
		dev->nluns = pdata->nluns;
		dev->factory_enabled = pdata->factory_enabled;
	}

	return 0;
}

static struct platform_driver android_platform_driver = {
	.driver = {.name = "android_usb",},
	.probe = android_probe,
};

static int __init init(void)
{
	struct android_dev *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	_android_dev = dev;

	ret = platform_driver_register(&android_platform_driver);
	if (ret)
		return ret;
	ret = misc_register(&adb_enable_device);
	if (ret) {
		platform_driver_unregister(&android_platform_driver);
		return ret;
	}
	ret = misc_register(&mode_change_device);
	if (ret) {
		misc_deregister(&adb_enable_device);
		platform_driver_unregister(&android_platform_driver);
		return ret;
	}

	ret = usb_composite_register(&android_usb_driver);
	if (ret) {
		misc_deregister(&adb_enable_device);
		misc_deregister(&mode_change_device);
		platform_driver_unregister(&android_platform_driver);
	}

	return ret;
}

module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	misc_deregister(&adb_enable_device);
	misc_deregister(&mode_change_device);
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
}

module_exit(cleanup);
