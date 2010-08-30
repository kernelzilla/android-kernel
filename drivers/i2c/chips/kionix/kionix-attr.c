/*
 * Morrison accelerometer I2C protocol driver
 *
 *	Copyright (C) 2008  Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#include "kionix-main.h"
#include "kionix-i2c.h"

/* Module attributes stuff */
enum accel_cmd
{
	ACCEL_CMD_CURRENT = 0,
	ACCEL_CMD_ORIENTATION,
	ACCEL_CMD_REGISTERS,
	ACCEL_CMD_CLIENTS,
	ACCEL_CMD_POWER_MODE,
	ACCEL_CMD_MAX
};

static char *orientation_names[] = {"Unknown", "UPSIDE-PORTRAIT", "DOWNSIDE-PORTRAIT", 
				    "TILTED_LEFT-LANDSCAPE", "TILTED_RIGHT-LANDSCAPE" };


/**
 * Attribute Show Current
 * Prints out current X,Y and Z values
 * @return pointer to character string
 */
static ssize_t accel_show_current(struct device *dev, struct device_attribute *attr, char *buf)
{
	int x, y, z;
	mutex_lock (&accel_info.mlock);
	x = accel_info.x_axis, y = accel_info.y_axis, z = accel_info.z_axis;
	mutex_unlock (&accel_info.mlock);
	return snprintf (buf, PAGE_SIZE, "%d:%d:%d\n", x, y, z);
}

/**
 * Attribute Show Orientation
 * Prints out current device orientation
 * @return pointer to character string
 */
static ssize_t accel_show_orientation(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *orient = orientation_names[0];

	mutex_lock (&accel_info.mlock);
	switch (accel_info.device_orient & ACCEL_BM_ORIENT) {
	 case ACCEL_DEV_ORIENT_NORMAL : orient = orientation_names[1]; break;
	 case ACCEL_DEV_ORIENT_UPSIDE : orient = orientation_names[2]; break;
	 case ACCEL_DEV_ORIENT_TILTED_LEFT  : orient = orientation_names[3]; break;
	 case ACCEL_DEV_ORIENT_TILTED_RIGHT : orient = orientation_names[4]; break;
	}
	mutex_unlock (&accel_info.mlock);
	return snprintf (buf, PAGE_SIZE, "%s\n", orient);
}

/**
 * Attribute Show Registers
 * Prints out current accelerometer's registers content
 * @return pointer to character string
 */
static ssize_t accel_show_registers(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t n, written=0;
	u8 regs[13];

	if (accel_i2c_read_control_registers (regs, sizeof(regs)/sizeof(regs[0])) != 0)
		return -EINVAL;
				
	SAFE_SNPRINTF ("FF_INT    0x%02x\n", regs[0]);
	SAFE_SNPRINTF ("FF_DELAY  0x%02x\n", regs[1]);
	SAFE_SNPRINTF ("MOT_INT   0x%02x\n", regs[2]);
	SAFE_SNPRINTF ("MOT_DELAY 0x%02x\n", regs[3]);
	SAFE_SNPRINTF ("CTRL_REGC 0x%02x\n", regs[4]);
	SAFE_SNPRINTF ("CTRL_REGB 0x%02x\n", regs[5]);
	SAFE_SNPRINTF ("CTRL_REGA 0x%02x\n", regs[6]);

	return written;
}


/**
 * Attribute Show Clients
 * Prints out list of registered clients
 * @return pointer to character string
 */
static ssize_t accel_show_clients(struct device *dev, struct device_attribute *attr, char *buf)
{
	accel_client_t *client;
	accel_file_private_t *priv;
	ssize_t n, written=0;

	SAFE_SNPRINTF (" Pid      | Mode     |Event mask| Status |\n");
	SAFE_SNPRINTF ("==========================================\n"); 
	
	mutex_lock (&accel_info.mlock);
	client = accel_info.clients;
	while (client != NULL) {

		priv = accel_find_private_by_pid (client->pid);
		if (priv != NULL) {
			SAFE_SNPRINTF ("%-10d|0x%08lx|0x%08lx|0x%08x|\n",
				priv->mypid, 
				priv->access_flags,
				priv->events_mask,
				priv->status);
		}else {
			SAFE_SNPRINTF ("Oops!!! Private for pid %d not found\n", client->pid);
		}
		client = client->next;
		CUTOFF_CHECK_BREAK (80);	
	}
	mutex_unlock (&accel_info.mlock);

	return written;
}


/**
 * Attribute Show power mode
 * Prints out current power mode
 * @return pointer to character string
 */
static ssize_t accel_show_power_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t n, written=0;
	int power_mode;

	/* retrieve current register value */
	accel_i2c_get_power_mode (&power_mode);
	SAFE_SNPRINTF ("[%d]- ", power_mode);
	switch(power_mode) {
	 case 0x1 : SAFE_SNPRINTF ("normal (ODR=50Hz)\n"); break;
 	 case 0x2 : SAFE_SNPRINTF ("low (ODR=0.5Hz)\n"); break;
	 case 0x3 : SAFE_SNPRINTF ("low (ODR=1Hz)\n"); break;
	 case 0x4 : SAFE_SNPRINTF ("low (ODR=2Hz)\n"); break;
	 case 0x5 : SAFE_SNPRINTF ("low (ODR=5Hz)\n"); break;
	 case 0x6 : SAFE_SNPRINTF ("low (ODR=10Hz)\n"); break;
	 case 0x0 : SAFE_SNPRINTF ("power off\n"); break;
	 default  : SAFE_SNPRINTF ("unknown\n"); break;
	}
	return written;	
}

/**
 * Attribute Set power mode
 * Sets given power mode
 * @return pointer to character string
 */
static ssize_t accel_set_power_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 power_mode = (u8)simple_strtoul(buf, NULL, 10);

	if (power_mode > 6) return -EINVAL;
	accel_i2c_set_power_mode (power_mode);
	vprintk ("power mode changed; current mode %d\n", power_mode);
	return count;	
}


/*
 *	Simple and similar set/show procedures
 *
#define ATTRIBUTES_SYS_FUNC(name)	\
static ssize_t accel_set_ ## name(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) \
{ \
	name = simple_strtoul (buf, NULL, 10); \
	return count; \
} \
static ssize_t accel_show_ ## name(struct device *dev, struct device_attribute *attr, char *buf) \
{ \
	return snprintf (buf, PAGE_SIZE, "%d\n", name); \
} \

ATTRIBUTES_SYS_FUNC(debug)
ATTRIBUTES_SYS_FUNC(verbose)
*/

static SENSOR_DEVICE_ATTR(show_current, S_IRUGO, 
		accel_show_current, NULL, ACCEL_CMD_CURRENT);
static SENSOR_DEVICE_ATTR(show_orientation, S_IRUGO, 
		accel_show_orientation, NULL, ACCEL_CMD_ORIENTATION);
static SENSOR_DEVICE_ATTR(show_registers, S_IRUGO, 
		accel_show_registers, NULL, ACCEL_CMD_REGISTERS);
static SENSOR_DEVICE_ATTR(show_clients, S_IRUGO, 
		accel_show_clients, NULL, ACCEL_CMD_CLIENTS);
static SENSOR_DEVICE_ATTR(power_mode, S_IRUGO | S_IWUSR, 
		accel_show_power_mode, accel_set_power_mode, ACCEL_CMD_POWER_MODE);
/*
static SENSOR_DEVICE_ATTR(irq_stats, S_IRUGO | S_IWUSR, 
		lis331dlh_show_irq_stats, lis331dlh_set_irq_stats, LIS331DLH_CMD_STATS);
static SENSOR_DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, 
		lis331dlh_show_debug, lis331dlh_set_debug, LIS331DLH_CMD_DBG);
static SENSOR_DEVICE_ATTR(verbose, S_IRUGO | S_IWUSR, 
		lis331dlh_show_verbose, lis331dlh_set_verbose, LIS331DLH_CMD_VERBOSE);
*/
static struct attribute *accel_attributes[] = {
	&sensor_dev_attr_show_current.dev_attr.attr,
	&sensor_dev_attr_show_orientation.dev_attr.attr,
	&sensor_dev_attr_show_registers.dev_attr.attr,
	&sensor_dev_attr_show_clients.dev_attr.attr,
	&sensor_dev_attr_power_mode.dev_attr.attr,
	NULL
};

struct attribute_group accel_defattr_group = {
	.attrs = accel_attributes,
};

