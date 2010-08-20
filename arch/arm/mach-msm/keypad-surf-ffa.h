/*
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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

#ifndef _KEYPAD_SURF_FFA_H
#define _KEYPAD_SURF_FFA_H

#include <linux/input.h>

#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD)
struct input_dev *msm_keypad_get_input_dev(void);
#else
static struct input_dev *msm_keypad_get_input_dev(void)
{
	return NULL;
}
#endif

#endif
