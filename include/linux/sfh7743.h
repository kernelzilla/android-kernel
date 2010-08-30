/* sfh7743.h
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

#ifndef _SFH7743_H
#define _SFH7743_H

struct sfh7743_platform_data {
	const char *name;
	unsigned    gpio_en;
	unsigned    gpio_intr;
};

#endif
