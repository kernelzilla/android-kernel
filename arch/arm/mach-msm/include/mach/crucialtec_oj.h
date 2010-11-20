/* 
 *
 *
 * Copyright (c) 2009 Motorola Inc.
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


struct crucial_oj_platform_data {
   const char    * name; 
   int           gpio_motion_irq;
   int           gpio_reset;
   int           gpio_shutdown;
};
