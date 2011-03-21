/* drivers/power/mot_battery.c
 *
 * Battery driver
 *
 * Copyright (C) 2008 Motorola, Inc.
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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/mot_battery_info.h>
#include <linux/phone_battery.h>
#include <linux/io.h>

static mot_battery_info batt_info;
static bool sanity_check(enum power_supply_property psp, int val);

/* Battery properties exposed through the sysfs inteface */
static enum power_supply_property mot_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP
};

/* Local constants */
enum {
	MOT_BATTERY_NUMBER_OF_PROPERTIES = (sizeof(mot_battery_props) /
						 sizeof(mot_battery_props[0])),
	MOT_BATTERY_STATUS_INIT_VAL      =  POWER_SUPPLY_STATUS_DISCHARGING,
	MOT_BATTERY_HEALTH_INIT_VAL      =  POWER_SUPPLY_HEALTH_GOOD,
	MOT_BATTERY_PRESENT_INIT_VAL     =  true,
	MOT_BATTERY_TECHNOLOGY_INIT_VAL  =  POWER_SUPPLY_TECHNOLOGY_LION,
	MOT_BATTERY_VOLTAGE_INIT_VAL     =  3700, /* mV */
	MOT_BATTERY_CAPACITY_INIT_VAL    =  50,   /* percent */
	MOT_BATTERY_TEMP_INIT_VAL        =  250,  /* 0.1 degree Celsius */
	MOT_BATTERY_VOLTAGE_MIN          =  0,    /* mV */
	MOT_BATTERY_VOLTAGE_MAX          =  4500, /* mV */
	MOT_BATTERY_CAPACITY_MIN         =  0,    /* percent */
	MOT_BATTERY_CAPACITY_MAX         =  100,  /* percent */
	MOT_BATTERY_TEMP_MIN             = -1000, /* 0.1 degree Celsius */
	MOT_BATTERY_TEMP_MAX             =  1000, /* 0.1 degree Celsius */
	MOT_BATTERY_TEMP_NO_CHARGING     =  650,  /* 0.1 degree Celsius */
};

/* Battery property data */
typedef struct {
	enum power_supply_property	psp; /* property */
	int				val; /* value of the property */
	bool	no_update; /* can the value be updated? */

	/* function to get new value for the property */
	get_battery_property	get;

	/* name of the file in debugfs to set the value of the property */
	char			*name;
} mot_battery_property;

/* Battery properties.  All the properties from the mot_battery_props[]
 * must be listed here. The order or the properties can be arbitrary */

mot_battery_property mot_battery_data[MOT_BATTERY_NUMBER_OF_PROPERTIES] = {
	{ POWER_SUPPLY_PROP_STATUS,      MOT_BATTERY_STATUS_INIT_VAL,
					 false, NULL, "status"      },
	{ POWER_SUPPLY_PROP_HEALTH,      MOT_BATTERY_HEALTH_INIT_VAL,
					 true,  NULL, "health"      },
	{ POWER_SUPPLY_PROP_PRESENT,     MOT_BATTERY_PRESENT_INIT_VAL,
					 false, NULL, "present"     },
	{ POWER_SUPPLY_PROP_TECHNOLOGY,  MOT_BATTERY_TECHNOLOGY_INIT_VAL,
					 true,  NULL, "technology"  },
	{ POWER_SUPPLY_PROP_VOLTAGE_NOW, MOT_BATTERY_VOLTAGE_INIT_VAL,
					 false, NULL, "voltage"     },
	{ POWER_SUPPLY_PROP_CAPACITY,    MOT_BATTERY_CAPACITY_INIT_VAL,
					 false, NULL, "capacity"    },
	{ POWER_SUPPLY_PROP_TEMP,        MOT_BATTERY_TEMP_INIT_VAL,
					 false, NULL, "temperature" }
};

/* To get the index in the mot_battery_data[] */
static int psp_to_index(enum power_supply_property psp)
{
	int i;

	for (i = 0; i < MOT_BATTERY_NUMBER_OF_PROPERTIES &&
			 mot_battery_data[i].psp != psp; i++)
			;
	return i;
}

/* Update battery status.  Returns true if the value was updated */
static bool update_status(int new_status)
{
	int  *status   =
		&mot_battery_data[psp_to_index(POWER_SUPPLY_PROP_STATUS)].val;
	int *capacity =
		&mot_battery_data[psp_to_index(POWER_SUPPLY_PROP_CAPACITY)].val;
	bool  changed  = true;

	/* Stay FULL until DISCHARGING starts.
	 * Become FULL if CHARGING and CAPACITY is 100% */

	if (new_status == POWER_SUPPLY_STATUS_CHARGING &&
				*status != POWER_SUPPLY_STATUS_FULL) {
		*status = (*capacity == 100 ? POWER_SUPPLY_STATUS_FULL :
						 POWER_SUPPLY_STATUS_CHARGING);
	} else if (new_status != POWER_SUPPLY_STATUS_CHARGING) {
			*status = new_status;
	} else { /* new_status == POWER_SUPPLY_STATUS_CHARGING &&
					 *status == POWER_SUPPLY_STATUS_FULL */
		changed = false;
	}

	return changed;
}

/* Update battery capacity.  Returns true if the value was updated */
static bool update_capacity(int new_capacity)
{
	int *status =
		 &mot_battery_data[psp_to_index(POWER_SUPPLY_PROP_STATUS)].val;
	int *capacity =
		 &mot_battery_data[psp_to_index(POWER_SUPPLY_PROP_CAPACITY)].val;
	bool  changed  = true;

	/* Become FULL if CHARGING and CAPACITY is 100%.
	 * Latch 100% CAPACITY while FULL */

	if (new_capacity == 0) {
		/* Ensure that phone will perform a low battery shutdown */
		*status   = POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (new_capacity == 100 &&
				 *status == POWER_SUPPLY_STATUS_CHARGING) {
			*status   = POWER_SUPPLY_STATUS_FULL;
	} else if (new_capacity < 100 && *status == POWER_SUPPLY_STATUS_FULL) {
		/* stay at full capacity after charging was complete while
		 * charger is present */
		changed   = false;
	}

	if (changed)
		*capacity = new_capacity;

	return changed;
}

/* Update the value of the property.  Returns true if the value was updated
   i       - index in mot_battery_data[]
   value   - new value
*/
static bool set_property(int i, int val)
{
	bool changed = false;

	if (mot_battery_data[i].val != val) {
		if (i == psp_to_index(POWER_SUPPLY_PROP_STATUS)) {
			changed = update_status(val);
		} else if (i == psp_to_index(POWER_SUPPLY_PROP_CAPACITY)) {
			changed = update_capacity(val);
		} else {
			changed = true;
			mot_battery_data[i].val = val;
		}
	}

	if (i == psp_to_index(POWER_SUPPLY_PROP_TEMP))
		set_mot_battery_temp(val);

	return changed;
}

/* sysfs */
static int mot_battery_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	int	ret = 0;
	int	i = psp_to_index(psp);
	get_battery_property  get;
	int	data;

	if (i != MOT_BATTERY_NUMBER_OF_PROPERTIES) {
		if (!mot_battery_data[i].no_update) {
			get = mot_battery_data[i].get;

			if (get && get(&data) && sanity_check(psp, data)) {
				(void) set_property(i, data);
			} else if (psp == POWER_SUPPLY_PROP_TEMP) {
				/* Prevent charging when battery temperature
				 * cannot be obtained */
				pr_err("prevening battery charge since battery temp cannot be obtained\n");
				set_mot_battery_temp(MOT_BATTERY_TEMP_NO_CHARGING);
			}
		}

		val->intval = mot_battery_data[i].val;
	} else {
		ret = -EINVAL;
	}

	return ret;
}

/* Does the value for the battery property make sense? */
static bool sanity_check(enum power_supply_property psp, int val)
{
   bool ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = (val >= MOT_BATTERY_VOLTAGE_MIN &&
			 val <= MOT_BATTERY_VOLTAGE_MAX);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = (val >= MOT_BATTERY_CAPACITY_MIN &&
			 val <= MOT_BATTERY_CAPACITY_MAX);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = (val >= MOT_BATTERY_TEMP_MIN &&
			 val <= MOT_BATTERY_TEMP_MAX);
		break;
	default:
		ret = true;
		break;
	}

	return ret;
}

/* To set property values by other kernel modules. Property gets updated
 * unless updates are disabled
 */
bool set_battery_property(enum power_supply_property psp, int val)
{
	int i = psp_to_index(psp);
	bool changed = (i != MOT_BATTERY_NUMBER_OF_PROPERTIES &&
				 !mot_battery_data[i].no_update &&
				 sanity_check(psp, val) &&
				 set_property(i, val));

	if (changed)
		battery_changed();

	return changed;
}

/* To set the function used to poll new value of the property */
void assign_get_function(enum power_supply_property psp, get_battery_property f)
{
	int  i = psp_to_index(psp);

	if (i != MOT_BATTERY_NUMBER_OF_PROPERTIES)
		mot_battery_data[i].get = f;

}

/* Debugfs interface to test how system works with different values
 * of the battery properties Once the propety value is set through the
 * debugfs, updtes from the drivers will be discarded
 */
#ifdef CONFIG_DEBUG_FS
static int mot_battery_debug_set(void *data, u64 val)
{
	int i = (int) data;
	/* set the flag to prevent updates other than through debugfs */
	mot_battery_data[i].no_update = true;

	if (set_property(i, (int) val))
		battery_changed();

	return 0;
}

static int mot_battery_debug_get(void *data, u64 *val)
{
	int i = (int) data;

	*val = mot_battery_data[i].val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(mot_battery_fops, mot_battery_debug_get,
				 mot_battery_debug_set, "%llu\n");

static int __init mot_battery_debug_init(void)
{
	struct dentry *dent = debugfs_create_dir("battery", 0);
	int	ret  = 0;
	int            i;

	if (!IS_ERR(dent)) {
		for (i = 0; i < MOT_BATTERY_NUMBER_OF_PROPERTIES; i++) {
			debugfs_create_file(mot_battery_data[i].name, 0666,
					 dent, (void *) i, &mot_battery_fops);
		}
	} else {
		ret = PTR_ERR(dent);
	}

	return ret;
}

late_initcall(mot_battery_debug_init);
#endif /* CONFIG_DEBUG_FS */

static struct power_supply mot_battery = {
	.name	= "battery",
	.type	= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= mot_battery_props,
	.num_properties	= ARRAY_SIZE(mot_battery_props),
	.get_property	= mot_battery_get_property,
};

/* Notify user code that battery state has changed */
void battery_changed(void)
{
	/* Send uevent.  No need to use power_supply_changed()
	 * because LED's are not used for battery status indication
	 */
	kobject_uevent(&mot_battery.dev->kobj, KOBJ_CHANGE);
}

static int __init mot_battery_init(void)
{
	int status, health;
	int i;

	get_mot_battery_info(&batt_info);

	/* if non Motorola battery is used, set status and health to unkonwn */
	if (!batt_info.motorola_battery) {
		status = psp_to_index(POWER_SUPPLY_PROP_STATUS);
		health = psp_to_index(POWER_SUPPLY_PROP_HEALTH);

		mot_battery_data[status].val	= POWER_SUPPLY_STATUS_UNKNOWN;
		/* to discard updates from a driver */
		mot_battery_data[status].no_update = true;

		mot_battery_data[health].val	= POWER_SUPPLY_STATUS_UNKNOWN;
		mot_battery_data[health].no_update = true;
	}

	/* if a factory cable is attach, discard updates from a driver */
	if (batt_info.factory_cable) {
		for (i = 0; i < MOT_BATTERY_NUMBER_OF_PROPERTIES; i++)
			mot_battery_data[i].no_update = true;
	}

	 return power_supply_register(NULL, &mot_battery);
}

static void __exit mot_battery_exit(void)
{
	power_supply_unregister(&mot_battery);
}

/* need to be called before USB driver and after power supply driver */
fs_initcall(mot_battery_init);
module_exit(mot_battery_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Motorola Battery Driver");
