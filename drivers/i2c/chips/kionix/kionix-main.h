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

#ifndef ACCEL_MAIN_H_
#define ACCEL_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/jiffies.h>
#include <linux/delay.h>

#include <asm/mach/irq.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/signal.h>

#include "kionix-fops.h"

#define ACCEL_POLL_INTERVAL	200	/* poll for input every 1/5s (200 ms)*/
#define ACCEL_INPUT_FUZZ	4	/* input event threshold */
#define ACCEL_INPUT_FLAT	4

#define ACCEL_GPIO_ACL_INT1	87
#define ACCEL_GPIO_ACL_INT2	108

#define ACCEL_MAGIC		0x12345678
#define MODULE_NAME		"kionix_i2c"

#undef SUCCESS
#define SUCCESS 0


/* LIS331 structures definitions */
typedef struct accel_signal {
	int irq;			/* IRQ */
	int sid;                        /* signal id */
	pid_t pid;                      /* process id */
	struct task_struct *task;	/* pointer to task structure */
} accel_signal_t;

typedef struct accel_interrupt {
	int irq;		/* IRQ */
	u8 irq_flags;		/* IRQ flags: pull-push/open drain, latch, INT pad # */
	u8 irq_cfg;		/* IRQconfig register */
	u8 irq_ths;		/* IRQ threshold register */
	u8 irq_dur;		/* IRQ duration register */
} accel_interrupt_t;

typedef struct accel_data {
	int magic;

	struct mutex mlock;		/* data modification lock */
	spinlock_t slock;		/* irq lock */
	struct i2c_client i2c;

	int x_axis;
	int y_axis;
	int z_axis;

	int mode;
	int pid;
	int screen_orient;		/* screen orienattion: portrait, landscape */
	int device_orient;		/* device orientation: normal, upside, tilted left, tilted right */

	int irq;
	struct work_struct wq;		/* work queue */

	struct workqueue_struct *myworkqueue;
	struct delayed_work	dw; 

	struct input_polled_dev *idev;
	struct cdev *cdev;

	accel_client_t *clients;	/* list of clients */
	int num_clients;		/* # of clients */

	accel_file_private_t *privates;
} accel_data_t;


#include "kionix-devconf.h"
#include "kionix-i2c.h"

#define AXIS_X		1
#define AXIS_Y		2
#define AXIS_Z		4
#define AXIS_XYZ	(AXIS_X|AXIS_Y|AXIS_Z)

enum pwrmode { PWR_OFF=0, PWR_ON, PWR_halfHZ, PWR_oneHZ, PWR_twoHZ, PWR_fiveHZ, PWR_tenHZ};

/* Extern definitions */
extern accel_data_t		accel_info;
extern unsigned long		accel_events_mask;
extern int 			accel_param_verbose;
extern int 			accel_param_debug;
extern int			accel_param_input;

extern struct attribute_group	accel_defattr_group;
extern struct file_operations	accel_fops;

extern void accel_irq_bottom_half(struct work_struct *work);
extern irqreturn_t accel_irq_handler(int irq, void *dev);

/*
 *	Useful MACROs 
 */
#define dprintk(fmt, args...)                                           	\
	do{                                                             	\
		if(accel_param_debug) printk(KERN_DEBUG MODULE_NAME ": " 	\
						fmt "\n", ## args);		\
	}while(0)

#define vprintk(fmt, args...)                                           	\
	do{                                                             	\
		if(accel_param_verbose) printk(KERN_DEBUG MODULE_NAME ": " 	\
						fmt "\n", ## args); 		\
	}while(0)

#define fprintk(fmt, args...)                                           	\
	do{                                                             	\
		printk(KERN_ERR MODULE_NAME ": " fmt "\n", ## args);    	\
	}while(0)

#define SAFE_SNPRINTF(fmt, args...)						\
	do{									\
		n = snprintf (buf+written, PAGE_SIZE-written, fmt, ## args);	\
		written += n;							\
	}while(0)

#define CUTOFF_CHECK_BREAK(pos)							\
	if (written+pos > PAGE_SIZE) {						\
		n = snprintf (buf+written, PAGE_SIZE-written, 			\
					"!!! rest of output cut off\n");	\
		written += n;							\
		break;								\
	}

#ifdef __cplusplus
}
#endif
#endif
