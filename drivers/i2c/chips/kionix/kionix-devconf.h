/*
 * Morrison accelerometer I2C protocol driver
 *
 *	Copyright (C) 2008  Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#ifndef ACCEL_DEVCONF_H_
#define ACCEL_DEVCONF_H_

#ifdef __cplusplus
extern "C" {
#endif

#define ACCEL_DEVICE_PATH	"/dev/accelerometer"

#define ACCEL_MAJOR		251
#define ACCEL_NR_DEVS		1

#define ACCEL_IOC_MAGIC		'g'
#define ACCEL_IOC_MAXNR		19

#define IOCTL_ACCEL_READ_REVID		_IOR(ACCEL_IOC_MAGIC, 0, char) /* returns 8 bytes character string */
#define IOCTL_ACCEL_POLL_DELAY_SET	_IOR(ACCEL_IOC_MAGIC, 1, int)	/* extpects an integer delay in msec */
#define IOCTL_ACCEL_READ_AXIS_Y		_IOR(ACCEL_IOC_MAGIC, 2, int)	/* */
#define IOCTL_ACCEL_READ_AXIS_Z		_IOR(ACCEL_IOC_MAGIC, 3, int)	/* */
#define IOCTL_ACCEL_READ_AXIS_XYZ	_IOR(ACCEL_IOC_MAGIC, 4, int)	/* returns triplet of integers */
#define IOCTL_ACCEL_SET_PWR_MODE	_IOW(ACCEL_IOC_MAGIC, 5, int)	/* expects an integer */
#define IOCTL_ACCEL_GET_PWR_MODE 	_IOR(ACCEL_IOC_MAGIC, 6, int)	/* returns an integer */
#define IOCTL_ACCEL_ARM_INTERRUPT 	_IOW(ACCEL_IOC_MAGIC, 7, int)	/* expects struct accel_interrupt_t */
#define IOCTL_ACCEL_DISARM_INTERRUPT 	_IOW(ACCEL_IOC_MAGIC, 8, int)	/* expects an integer */
#define IOCTL_ACCEL_SET_SIGNAL 		_IOW(ACCEL_IOC_MAGIC, 9, int)	/* expects struct accel_signal_t */
#define IOCTL_ACCEL_RELEASE		_IO (ACCEL_IOC_MAGIC, 10)	/* takes no arguments */
#define IOCTL_ACCEL_ORIENTATION		_IOR(ACCEL_IOC_MAGIC,11, int)	/* returns pair of integers */
#define IOCTL_ACCEL_ENABLE_AXIS		_IOW(ACCEL_IOC_MAGIC,12, int)	/* expects an integer */
#define IOCTL_ACCEL_DUMP_STATUS		_IOR(ACCEL_IOC_MAGIC,13, int)	/* returns 13 unsigned char items */
#define IOCTL_ACCEL_SELF_TEST		_IOW(ACCEL_IOC_MAGIC,14, int)	/* expects an integer */
#define IOCTL_ACCEL_WAKE_UP		_IOW(ACCEL_IOC_MAGIC,15, int)	/* expects an integer */

#define IOCTL_ACCEL_EVENTS_SET		_IOW(ACCEL_IOC_MAGIC,16, int)	/* sets events mask accordingly */
#define IOCTL_ACCEL_EVENTS_GET		_IOR(ACCEL_IOC_MAGIC,17, int)	/* returns current events mask */
#define IOCTL_ACCEL_EVENTS_CLEAR	_IOW(ACCEL_IOC_MAGIC,18, int)	/* clears whole events mask */
#define IOCTL_ACCEL_FETCH_EVENT_DATA	_IOR(ACCEL_IOC_MAGIC,19, int)	/* fills out pointer with struct input_event */


#define ACCEL_ORIENT_UNKNOWN		0x00
#define ACCEL_ORIENT_NORMAL		0x08
#define ACCEL_ORIENT_UPSIDE		0x04
#define ACCEL_ORIENT_TILTED_LEFT	0x02
#define ACCEL_ORIENT_TILTED_RIGHT	0x01

#define ACCEL_SCREEN_PORTRAIT		0x01
#define ACCEL_SCREEN_LANDSCAPE		0x02

#define ACCEL_EV_SCREEN_ORIENT		0x0000001
#define ACCEL_EV_TAP			0x0000002
#define ACCEL_EV_DOUBLE_TAP		0x0000004
#define ACCEL_EV_SWING			0x0000008
#define ACCEL_EV_THROW			0x0000010
#define ACCEL_EV_RAW_DATA		0x8000000

#ifdef __cplusplus
}
#endif
#endif
