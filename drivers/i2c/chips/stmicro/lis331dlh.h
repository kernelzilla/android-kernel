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
#define ACCEL_NR_DEVS		1

#define ACCEL_IOC_MAGIC		'g'
#define ACCEL_IOC_MAXNR		10

#define IOCTL_ACCEL_GET_REVID		_IOR(ACCEL_IOC_MAGIC, 0, char)  /* returns 8 bytes character string */
#define IOCTL_ACCEL_GET_XYZ		_IOR(ACCEL_IOC_MAGIC, 1, int)	/* returns triplet of integers */
#define IOCTL_ACCEL_SET_PWR_MODE	_IOW(ACCEL_IOC_MAGIC, 2, int)	/* expects an integer */
#define IOCTL_ACCEL_GET_PWR_MODE 	_IOR(ACCEL_IOC_MAGIC, 3, int)	/* returns an integer */
#define IOCTL_ACCEL_SET_CONFIG 		_IOW(ACCEL_IOC_MAGIC, 4, int)	/* expects struct accel_config_t */
#define IOCTL_ACCEL_SET_EVENTS		_IOW(ACCEL_IOC_MAGIC, 5, int)	/* sets events mask accordingly */
#define IOCTL_ACCEL_GET_EVENTS		_IOR(ACCEL_IOC_MAGIC, 6, int)	/* returns current events mask */
#define IOCTL_ACCEL_CLR_EVENTS		_IOW(ACCEL_IOC_MAGIC, 7, int)	/* clears whole events mask */
#define IOCTL_ACCEL_GET_EVENT_DATA	_IOR(ACCEL_IOC_MAGIC, 8, int)	/* returns struct input_event */
#define IOCTL_ACCEL_IRQ_CONFIG		_IOW(ACCEL_IOC_MAGIC, 9, int)	/* enable/disable IRQ handling */

#define IOCTL_ACCEL_SET_IRQ_CFG		_IOW(ACCEL_IOC_MAGIC,10, int)	/* override default IRQ threshold and duration */

#define ACCEL_ORIENT_UNKNOWN		0x0000
#define ACCEL_ORIENT_FACE_UP		0x0020	// Z_UP
#define ACCEL_ORIENT_FACE_DOWN		0x0010	// Z_DOWN
#define ACCEL_ORIENT_NORMAL		0x0800	// Y_UP
#define ACCEL_ORIENT_UPSIDE		0x0400	// Y_DOWN
#define ACCEL_ORIENT_TILTED_LEFT	0x0200	// X_UP
#define ACCEL_ORIENT_TILTED_RIGHT	0x0100	// X_DOWN

#define ACCEL_SCREEN_PORTRAIT		0x01
#define ACCEL_SCREEN_LANDSCAPE		0x02

#define ACCEL_EV_SCREEN_ORIENT		0x00000001
#define ACCEL_EV_TAP			0x00000002
#define ACCEL_EV_DOUBLE_TAP		0x00000004
#define ACCEL_EV_TAPPING		(ACCEL_EV_TAP | ACCEL_EV_DOUBLE_TAP)
#define ACCEL_EV_SWING			0x00000008
#define ACCEL_EV_THROW			0x00000010
#define ACCEL_EV_RAW_DATA		0x80000000
#define ACCEL_EV_ALLBITS		0xFFFFFFFF

#define ACCEL_EV_ROTATION		0x0100
#define ACCEL_EV_XYZ			0x0700
#define ACCEL_EV_ROTATION_ENABLE	0x1000
#define ACCEL_EV_ROTATION_DISABLE	0x2000

#define ACCEL_ROTATION_0		ACCEL_ORIENT_NORMAL
#define ACCEL_ROTATION_90		ACCEL_ORIENT_TILTED_LEFT	
#define ACCEL_ROTATION_180		ACCEL_ORIENT_UPSIDE
#define ACCEL_ROTATION_270		ACCEL_ORIENT_TILTED_RIGHT

/*
 * This structure is used to read/write to /dev/accelerometer
 * Field description:
 *	ev_code   - event code, that defines what data to expect in ev_values
 *	ev_values - array of data
 *
 * Read is used to retrieve the latest acceleartion data:
 *	ev_code   - 0x00070003 (combo of ACCEL_EV_XYZ + EV_ABS)
 *	ev_values - triplet of X,Y and Z
 * 	
 * Write is used to inject accelerometer events:
 *
 * Variant 1:
 *	ev_code   - 0x00010005 (combo of ACCEL_EV_ROTATION + EV_SW)
 *	ev_values - rotation in [0] element of array
 *
 * Variant 2: 
 *	ev_code   - 0x00070003 (combo of ACCEL_EV_XYZ + EV_ABS)
 *	ev_values - triplet of X,Y and Z
 */
typedef struct {

 unsigned short	ev_code;
 int		ev_values[3];

} accel_event_t;

/*
 * This structure is used to deliver new values of interrupt's threshold and duration
 * Field description:
 *	orientation	- 'P' or 'L' for portrait or landscape
 *	threashold	- inclanation to horizon
 * 	duration	- duration
 */
typedef struct {

  char	orientation;
  int	threshold, 
	duration;

} accel_irq_cfg_t;

#ifdef __cplusplus
}
#endif
#endif
