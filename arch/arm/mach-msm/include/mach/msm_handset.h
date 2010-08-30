/* arch/arm/mach-msm/include/mach/msm_handset.h
 *
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

#ifndef _MSM_HANDSET_H
#define _MSM_HANDSET_H

#include <linux/input.h>
#include <linux/switch.h>

#if defined(CONFIG_INPUT_MSM_HANDSET)
struct input_dev *msm_get_handset_input_dev(void);
#else
struct input_dev *msm_get_handset_input_dev(void)
{
	return NULL;
}
#endif

struct msm_handset {
	struct input_dev *ip_dev;
	struct switch_dev sdev;
};

#endif
