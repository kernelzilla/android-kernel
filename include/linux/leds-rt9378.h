/* leds-rt9378.h - Platform data structure for RT9378 led controller
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

#ifndef __LINUX_RT9378_H
#define __LINUX_RT9378_H

#include <linux/leds.h>

struct rt9378_platform_data {
    const char* name;
	unsigned    gpio_en;
};

#endif /* __LINUX_RT9378_H */
