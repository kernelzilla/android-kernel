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

#define ACCEL_DEBUG_PRINTK
//#define ACCEL_DEBUG_IRQ

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
#include <linux/spinlock.h>
#include <linux/miscdevice.h>

#include <asm/mach/irq.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/signal.h>

#ifdef  CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "lis331dlh.h"
#include "accel-fops.h"

#define ACCEL_NO_POLL  (int)-1

#define ACCEL_POLL_INTERVAL	200	/* poll for input every 1/10s (100 ms)*/

#define ACCEL_2G_MAX		2050
#define ACCEL_4G_MAX		4100
#define ACCEL_8G_MAX		8200
#define ACCEL_INPUT_FUZZ	1	/* input event threshold */
#define ACCEL_INPUT_FLAT	1

#define SENSOR_ACCEL_HIGH_G_VAL 	1000	// highest value used in orientation detection
#define SENSOR_ACCEL_DIFF_VAL 		200	// minimum detectable difference between x-axis and y-axis
#define SENSOR_ACCEL_DIFF_VAL_SQUARE 	40000	// minimum detectable difference between x-axis and y-axis
#define SENSOR_ACCEL_AVG_NUM 		3	// number of measurements used in averaging

#define ACCEL_MAGIC		0x12345678
#define MODULE_NAME		"lis331dlh"
#define DEVICE_NAME		"accel_i2c"

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
	struct i2c_client *i2c;

	int x_axis;
	int y_axis;
	int z_axis;

    atomic_t mode;
    atomic_t odr;
    atomic_t g_range;

	int ODR;			/* effective ODR will be used to adjust interrupt configuration */
	int powermode;			/* current power mode */
	int irq2_disabled;		/* flag to indicate if DATA_READY IRQ is currently disabled */

	int screen_orient;		/* screen orienattion: portrait, landscape */
	int device_orient;		/* device orientation: normal, upside, tilted left, tilted right */

#ifdef ACCEL_DEBUG_IRQ
	int int1_isr_cnt, int1_bh_cnt;
	int int2_isr_cnt, int2_bh_cnt;
#endif
	int irq1, irq2;
	int irq1_cnt, irq2_cnt;
	struct work_struct wq1;		/* work queue */
	struct work_struct wq2;		/* work queue */
//  cvk011c: removing semaphores because they are not really needed  but cause default work queue lockup
//	struct semaphore sema1;
//	struct semaphore sema2;

	int suspended;

    atomic_t data_ready_enabled;

	struct input_dev *idev;
	struct input_polled_dev *ipdev;

	accel_client_t *clients;	/* list of clients */
	int num_clients;		/* # of clients */

	accel_file_private_t *privates;
#ifdef  CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
} accel_data_t;

struct mode_cfg {
	u8	ctrl_reg1, ctrl_reg4;
	u8	int1_cfg, int1_ths, int1_dur;
	u8	int2_cfg, int2_ths, int2_dur;
};

#include "accel-i2c.h"

#define AXIS_X		1
#define AXIS_Y		2
#define AXIS_Z		4
#define AXIS_XYZ	(AXIS_X|AXIS_Y|AXIS_Z)

#define SWAPPED_XY	(AXIS_X | AXIS_Y)
#define SWAPPED_XZ	(AXIS_X | AXIS_Z)
#define SWAPPED_YZ	(AXIS_Y | AXIS_Z)

#define ACCEL_POWER_DEFS  \
    KEYWORD(power_off,      PWR_OFF,	0,      0)	\
    KEYWORD(power_on,       PWR_ON,	50,     30)	\
    KEYWORD(power_low_half, PWR_halfHZ,	0.5,    2)	\
    KEYWORD(power_low_one,  PWR_oneHZ, 	1,      2)	\
    KEYWORD(power_low_two,  PWR_twoHZ, 	2,      2)	\
    KEYWORD(power_low_five, PWR_fiveHZ, 5,      3)	\
    KEYWORD(power_low_ten,  PWR_tenHZ, 	10,     6)

#define KEYWORD(symbol,tag,odr,multi) 	tag,
enum pwrmode {
	ACCEL_POWER_DEFS
	ACCEL_PWR_MAX
};
#undef KEYWORD

/*
 * When 2 IRQs used, recognition of some gestures can be combined under the single accelerometer setup.
 * Following is a compatibility matrix for 2 IRQs:
 *
 * RAW_DATA      combinable with - SCREEN_ORIENT or TAPPING 
 * SCREEN_ORIENT combinable with - RAW_DATA or TAPPING
 * TAPPING       combinable with - RAW_DATA or SCREEN_ORIENT
 *
 * When only 1 IRQ used, all gestures are exclusive. Meaning that the only one application can request
 * gesture recognition at the time. All others will get EBUSY, when trying to register events mask.
 */

/* since we use only 1 IRQ compatibility matrix is simple */
#define ACCEL_COMPATIBILITY_MODES \
    KEYWORD(RAW_DATA,      ACCEL_EV_SCREEN_ORIENT | ACCEL_EV_RAW_DATA) \
    KEYWORD(SCREEN_ORIENT, ACCEL_EV_SCREEN_ORIENT | ACCEL_EV_RAW_DATA) \
    KEYWORD(TAPPING,       0 ) \
    KEYWORD(SWING,         0 ) \
    KEYWORD(THROW,         0 )

#define ACCEL_CTRL_REGS_CONFIGS \
    KEYWORD(RAW_DATA,      reg1, 0xc7, reg2, 0x00, reg3, 0x10, reg4, 0xc0, reg5, 0x00) \
    KEYWORD(SCREEN_ORIENT, reg1, 0xc3, reg2, 0x00, reg3, 0x20, reg4, 0xc0, reg5, 0x00) \
    KEYWORD(TAPPING,       reg1, 0x17, reg2, 0x00, reg3, 0x00, reg4, 0xc0, reg5, 0x00) \
    KEYWORD(SWING,         reg1, 0x17, reg2, 0x00, reg3, 0x00, reg4, 0xc0, reg5, 0x00) \
    KEYWORD(THROW,         reg1, 0x17, reg2, 0x00, reg3, 0x00, reg4, 0xc0, reg5, 0x00)

#define ACCEL_IRQ_CONFIGS \
    KEYWORD(RAW_DATA,      irq, 2, cfg, 0x00, ths, 0x00, dur, 0x00) \
    KEYWORD(SCREEN_ORIENT, irq, 2, cfg, 0x4f, ths, 0x15, dur, 0x03) \
    KEYWORD(TAPPING,       irq, 1, cfg, 0x00, ths, 0x00, dur, 0x00) \
    KEYWORD(SWING,         irq, 1, cfg, 0x00, ths, 0x00, dur, 0x00) \
    KEYWORD(THROW,         irq, 1, cfg, 0x00, ths, 0x00, dur, 0x00)

#define KEYWORD(symbol,p1,p1v,p2,p2v,p3,p3v,p4,p4v) 	ACCEL_MODE_## symbol,
enum cfgmodes {
	ACCEL_IRQ_CONFIGS
	ACCEL_MODE_MAX
};
#undef KEYWORD



/* Extern definitions */
extern accel_data_t		accel_info;
extern unsigned long		accel_events_mask;
extern int 			accel_orient_by_interrupt;
extern int 			accel_param_trace_verbose;
extern int 			accel_param_trace_debug;
extern int			accel_param_input;
extern int			accel_param_swapped;
extern int			accel_param_trace_data;
extern int			accel_param_trace_orient;
extern int			accel_param_trace_irq;
extern int			accel_param_trace_poll;
extern int			accel_param_trace_bad_data;

extern struct attribute_group	accel_defattr_group;
extern struct file_operations	accel_fops;
extern struct mutex		pl;

extern void accel_input_params(struct input_dev *input, unsigned char params, int cnt, int limit);
extern void accel_irq_bottom_half(struct work_struct *work);
extern void accel_irq_bottom_half_job(int force);
extern void accel_irq_data_ready_bottom_half(struct work_struct *work);
extern int accel_configure_irqs (unsigned int enable);
extern int accel_resume(struct i2c_client *client);
extern irqreturn_t accel_irq_handler(int irq, void *dev);
extern irqreturn_t accel_irq_data_ready_handler(int irq, void *dev);
extern void accel_poll_data_ready (struct input_polled_dev *dev);
extern u8 accel_i2c_read_byte_data (u16 addr);
extern void accel_i2c_write_byte_data (u16 addr, u8 value);

/*
 *	Useful MACROs 
 */
#define fprintk(fmt, args...)                                           	\
	do{                                                             	\
		printk(KERN_INFO "%s: " fmt "\n", __FUNCTION__, ## args);    	\
	}while(0)

#define printk_bad_data(fmt, args...) \
    if (accel_param_trace_bad_data) fprintk(fmt, ## args)

#define printk_poll(fmt, args...) \
    if (accel_param_trace_poll) fprintk(fmt, ## args)

#define printk_irq(fmt, args...) \
    if (accel_param_trace_irq) fprintk(fmt, ## args)

#define printk_data(fmt, args...) \
    if (accel_param_trace_data) fprintk(fmt, ## args)

#define printk_orient(fmt, args...) \
    if (accel_param_trace_orient) fprintk(fmt, ## args)

#if defined (ACCEL_DEBUG_PRINTK)
#define vprintk(fmt, args...)                                           	\
		if(accel_param_trace_verbose)  fprintk(fmt,  ## args)
#else
#define vprintk(fmt, args...)
#endif

#if defined (ACCEL_DEBUG_PRINTK)
#define dprintk(fmt, args...)                                           	\
		if(accel_param_trace_debug) fprintk(fmt, ## args)
#else
#define dprintk(fmt, args...)
#endif

#define SAFE_SNPRINTF(fmt, args...)						\
	if (written+80 > PAGE_SIZE) {						\
		n = snprintf (buf+written, PAGE_SIZE-written, 			\
					"!!! rest of output cut off\n");	\
		written += n;							\
	}else {									\
		n = snprintf (buf+written, PAGE_SIZE-written, fmt, ## args);	\
		written += n;							\
	}

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
