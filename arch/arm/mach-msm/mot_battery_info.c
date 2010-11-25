/* arch/arm/mach-msm/mot_batt_info.c
 *
 * Battery specific data passed from and to the modem processor
 *
 * Copyright (C) 2009 Motorola, Inc.
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

#include <linux/mot_battery_info.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include "smd_private.h"

void get_mot_battery_info(mot_battery_info *batt_info)
{
	memcpy(batt_info, smem_alloc(SMEM_BATTERY_INFO,
		 sizeof(mot_battery_info)), sizeof(mot_battery_info));
}

void set_mot_battery_temp(int val)
{
	static int	*temp;
	mot_battery_info *batt_info;

	if (!temp) {
		batt_info = (mot_battery_info *) smem_alloc(SMEM_BATTERY_INFO,
						 sizeof(mot_battery_info));
		temp	= &batt_info->temperature;
	}

	*temp = val;
}
