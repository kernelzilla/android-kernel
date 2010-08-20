/*
 * Prototype of functions to retrieve and set Motorola battery info
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

#ifndef _MOT_BATTERY_INFO_H_
#define _MOT_BATTERY_INFO_H_

typedef struct {
	int motorola_battery;	/* is Motorola battery present */
	int factory_cable;	/* did we power up with a factory cable */
	int temperature;	/* battery temperature measured by fuel gauge */
} mot_battery_info;

extern void get_mot_battery_info(mot_battery_info *batt_info);
extern void set_mot_battery_temp(int val);

#endif
