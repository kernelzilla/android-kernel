/*
 * API's for a phone battery
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

#ifndef __LINUX_PHONE_BATTERY_H__
#define __LINUX_PHONE_BATTERY_H__

#include <linux/power_supply.h>

typedef bool (*get_battery_property)(int *);

extern bool set_battery_property(enum power_supply_property psp, int val);
extern void assign_get_function(enum power_supply_property psp,
				 get_battery_property f);
extern void battery_changed(void);

#endif /* __LINUX_PHONE_BATTERY_H__ */
