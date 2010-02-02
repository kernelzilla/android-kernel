/* Header file for:
 * Cypress CY8CTMA300 Prototype touchscreen driver.
 * include/linux/cy8ctma300.h
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 * History:
 *			(C) 2010 Cypress - Update for GPL distribution
 *			(C) 2009 Cypress - Assume maintenance ownership
 *			(C) 2009 Enea - Original prototype
 *
 */


#ifndef __CY8CTMA300_H__
#define __CY8CTMA300_H__

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/workqueue.h>



/* Register addresses */
/* Only registers used will defined at this time */
#define REG_STATUS (0x02)
#define REG_POS_X1 (0x03)
#define REG_POS_Y1 (0x05)
#define REG_POS_Z1 (0x07)

/* Various defines (to be moved into cy8ctma300 header) */
#define CY8CTMA300_TEMPLATE 1
#define CONFIG_TOUCHSCREEN_CY8CTMA300_I2C 1
#define CONFIG_TOUCHSCREEN_CY8CTMA300_SPI 0
#define CONFIG_TOUCHSCREEN_CY8CTMA300_UART 0
#define	MAX_12BIT			((1<<12)-1)
#define	TOUCHSCREEN_TIMEOUT		(msecs_to_jiffies(50))
/* Various flags needed */
#define CY8F_XY_AXIS_FLIPPED (0x01)

#define GET_NUM_FINGERS(X) ((X) &0x0F)
#define IS_LARGE_AREA(X) (((X) & 0x10) >> 4)
#define FLIP_DATA(X) ((X) && 0x01)



/* CY8CTMA300 private data
 *
 * Still to be added:
 * Registers for Power management.
 */

struct cy8_platform_data {
  void (*power_on)(void);
  u32 maxx;
  u32 maxy;
  u32 flags;
};

#endif 
