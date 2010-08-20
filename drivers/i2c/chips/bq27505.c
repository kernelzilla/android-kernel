/* drivers/i2c/chips/bq27505.c
 *
 * Driver for Texas Instruments bq27505 battery fuel gauge
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

#include "bq27505.h"
#include <linux/i2c.h>
#include <linux/phone_battery.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/rwsem.h>

MODULE_DESCRIPTION("Driver for Texas Instruments bq27505 battery fuel gauge");
MODULE_LICENSE("GPL");

/* Local constants */
enum {
	ENTER_ROM_MODE_REG           = 0x00,
	ENTER_ROM_MODE_DATA          = 0x0F00,
	DELAY_AFTER_FIRMWARE_UPGRADE = 250,	/* msec */
	DELAY_INIT_COMPLETE_CHECK    = 500,	/* msec */
	MAX_NUM_INIT_COMPLETE_CHECKS = 10,
	FW_VERSION_DELAY             = 250,	/* usec */
	/* max number of times to read batt temp */
	MAX_NUM_TRIES                = 3
};

static struct i2c_client  *bq27505_client;
static struct work_struct  work;
static struct timer_list   temperature_timer;

static DECLARE_RWSEM(rom_mode);

enum bq27505_standard_commands {
	CONTROL			= 0x00,
	AT_RATE			= 0x02,
	AT_RATE_TIME_TO_EMPTY	= 0x04,
	TEMPERATURE		= 0x06,
	VOLTAGE			= 0x08,
	FLAGS			= 0x0A,
	NOMINAL_AVAILABLE_CAPACITY	= 0x0C,
	FULL_AVAILABLE_CAPACITY		= 0x0E,
	REMAINING_CAPACITY		= 0x10,
	FULL_CHARGE_CAPACITY		= 0x12,
	AVERAGE_CURRENT			= 0x14,
	TIME_TO_EMPTY			= 0x16,
	TIME_TO_FULL			= 0x18,
	STANDBY_CURRENT			= 0x1A,
	STANDBY_TIME_TO_EMPTY		= 0x1C,
	MAX_LOAD_CURRENT		= 0x1E,
	MAX_LOAD_TIME_TO_EMPTY		= 0x20,
	AVAILABLE_ENERGY		= 0x22,
	AVERAGE_POWER			= 0x24,
	TIME_TO_EMPTY_AT_CONSTANT_POWER	= 0x26,
	STATE_OF_HEALTH			= 0x28,
	STATE_OF_CHARGE			= 0x2C,
	NORMALIZED_IMPEDANCE		= 0x2E,
	INSTANT_CURRENT			= 0x30
} ;

enum bq27505_control_subcommands {
	CONTROL_STATUS		= 0x00,
	DEVICE_TYPE		= 0x01,
	FW_VERSION		= 0x02,
	HW_VERSION		= 0x03,
	DATA_FLASH_CHECKSUM	= 0x04,
	WRITE_TEMPERATURE	= 0x06,
	PREV_COMMAND		= 0x07,
	CHEM_ID			= 0x08,
	COMPUTE_BOARD_OFFSET	= 0x09,
	COMPUTE_COULOMB_COUNTER_OFFSET	= 0x0A,
	WRITE_COULOMB_COUNTER_OFFSET	= 0x0B,
	OCV_CMD				= 0x0C,
	BAT_INSERT			= 0x0D,
	BAT_REMOVE			= 0x0E,
	SET_HIBERNATE			= 0x11,
	CLEAR_HIBERNATE			= 0x12,
	SET_SLEEP_PLUS			= 0x13,
	CLEAR_SLEEP_PLUS		= 0x14,
	SEAL				= 0x20,
	ENABLE_IMPEDANCE_TRACK		= 0x21,
	DISABLE_IMPEDANCE_TRACK		= 0x23,
	CALIBRATION_MODE		= 0x40,
	RESET				= 0x41
};

enum bq27505_extended_commands {
	DESIGN_CAPACITY,
	DATA_FLASH_CLASS 		= 0x3E,
	DATA_FLASH_BLOCK		= 0x3F,
	AUTHENTICATE,
	AUTHENTICATE_CHECKSUM,
	BLOCK_DATA			= 0x40,
	BLOCK_DATA_CHECKSUM,
	BLOCK_DATA_CONTROL		= 0x61,
	DEVICE_NAME_LENGTH,
	DEVICE_NAME,
	APPLICATION_STATUS
};

static bool bq27505_standard_command(u8 command, u16 *data)
{
	s32 read_data = i2c_smbus_read_word_data(bq27505_client, command);

	if (read_data < 0)
		return false;
	else {
		*data = (s16) read_data;
		return true;
	}
}

static bool bq27505_control_subcommand(u8 subcommand, u16 *data)
{
	if (i2c_smbus_write_word_data(bq27505_client, CONTROL, subcommand) < 0)
		return false;
	else {
		/* Delay is needed for only this subcommand */
		if (subcommand == FW_VERSION)
			udelay(FW_VERSION_DELAY);

		return bq27505_standard_command(CONTROL, data);
	}
}

static int bq27505_debug_init(void);

static void bq27505_update_battery(void)
{
	schedule_work(&work);
}

static irqreturn_t bq7505_irq_handler(int irq, void *dev_id)
{
	bq27505_update_battery();
	return IRQ_HANDLED;
}

static void temperature_timer_fn(unsigned long data)
{
	bq27505_update_battery();

	temperature_timer.expires = jiffies + 60*HZ;
	add_timer(&temperature_timer);
}

static bool bq27505_low_battery_shutdown(void)
{
	bool ret = false;
	u16  data;

	if (bq27505_standard_command(FLAGS, &data))
		ret = (data & 0x0002) ? true : false;

	return ret;
}

static bool bq27505_battery_present(void)
{
	bool ret = true;
	u16  data;

	if (bq27505_standard_command(FLAGS, &data))
		ret = (data & 0x0008) ? true : false;

	return ret;
}

static bool bq27505_get_capacity(int *capacity)
{
	bool ret = true;
	u16  data;

	if (bq27505_low_battery_shutdown()) {
		printk(KERN_INFO "Low battery shutdown - fuel_gauge\n");
		*capacity = 0;
	} else if (bq27505_standard_command(STATE_OF_CHARGE, &data)) {
			if (data == 0)
				/* keep the phone alive until
				 * threshold voltage is reached */
				*capacity = 1;
			else
				*capacity = data;
	} else
		ret = false;

	return ret;
}

static bool bq27505_get_voltage(int *voltage)
{
	bool ret = false;
	u16  data;

	if (bq27505_standard_command(VOLTAGE, &data)) {
		*voltage = data;
		ret	= true;
	}

	return ret;
}

static bool bq27505_get_temp(int *temp)
{
	bool ret = false;
	u16  data;
	int  i;

	/* try to get battery temp several times in case of errors */
	for (i = 0; i < MAX_NUM_TRIES && !ret; i++) {
		ret = bq27505_standard_command(TEMPERATURE, &data);
		if (!ret)
			pr_err("fuel gauge, cannot get battery temp,\
					 attempt %d\n", (i+1));
	}

	if (ret)
		*temp = data - 2732; /* 0.1K to 0.1C */

	return ret;
}


static bool battery_capacity(int *capacity)
{
	bool ret = false;

	if (down_read_trylock(&rom_mode)) {
		ret = bq27505_get_capacity(capacity);
		up_read(&rom_mode);
	}
	return ret;
}

static bool battery_voltage(int *voltage)
{
	bool ret = false;

	if (down_read_trylock(&rom_mode)) {
		ret = bq27505_get_voltage(voltage);
		up_read(&rom_mode);
	}
	return ret;
}

static bool battery_temp(int *temp)
{
	bool ret = false;

	if (down_read_trylock(&rom_mode)) {
		ret = bq27505_get_temp(temp);
		up_read(&rom_mode);
	}
	return ret;
}

static void bq27505_work_func(struct work_struct *work)
{
	/* int data; */

	if (!bq27505_battery_present())
		set_battery_property(POWER_SUPPLY_PROP_PRESENT, false);
	else
		battery_changed();
}

static int bq27505_open(struct inode *inode, struct file *f)
{
	return 0;
}

static int bq27505_release(struct inode *inode, struct file *f)
{
	return 0;
}

static int bq27505_enter_rom_mode(void)
{
	return i2c_smbus_write_word_data(bq27505_client,
					 ENTER_ROM_MODE_REG,
					 ENTER_ROM_MODE_DATA);
}

/* wait for initializion completed bit to be asserted */
static void wait_init_complete(void)
{
	bool complete = false;
	int  i	= 0;
	u16  data;

	do {
		msleep(DELAY_INIT_COMPLETE_CHECK);

		if (bq27505_standard_command(CONTROL, &data)
						 && (data & 0x0080))
			complete = true;

	} while (!complete && i++ < MAX_NUM_INIT_COMPLETE_CHECKS);

	if (!complete)
		pr_err("fuel gauge, initialization did not complete\n");
	else
		msleep(DELAY_AFTER_FIRMWARE_UPGRADE);
}

static int bq27505_ioctl(struct inode *inode, struct file *f,
				 unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case BQ27505_ENTER_ROM_MODE:
		disable_irq(bq27505_client->irq);
		del_timer_sync(&temperature_timer);
		flush_work(&work);
		down_write(&rom_mode);
		ret = bq27505_enter_rom_mode();
		break;

	case BQ27505_EXIT_ROM_MODE:
		wait_init_complete();
		up_write(&rom_mode);
		enable_irq(bq27505_client->irq);
		add_timer(&temperature_timer);
		break;

	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static struct file_operations bq27505_fops = {
	.owner =   THIS_MODULE,
	.open =    bq27505_open,
	.release = bq27505_release,
	.ioctl =   bq27505_ioctl
};

static struct miscdevice bq27505_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bq27505",
	.fops = &bq27505_fops,
};

static ssize_t bq27505_show_dataflash_vers(struct device *dev,
					 struct device_attribute *attr,
						 char *buf)
{
	u8 manuf_buf[32];
	memset(manuf_buf, 0, sizeof(manuf_buf));

	i2c_smbus_write_byte_data(bq27505_client, DATA_FLASH_CLASS, 0x39);
	i2c_smbus_write_byte_data(bq27505_client, DATA_FLASH_BLOCK, 0x00);
	mdelay(DELAY_AFTER_FIRMWARE_UPGRADE);
	i2c_smbus_read_i2c_block_data(bq27505_client, BLOCK_DATA,
					 sizeof(manuf_buf), manuf_buf);

	return sprintf(buf, "%d\n", manuf_buf[0]*256+manuf_buf[1]);
}
static ssize_t bq27505_show_firmware_vers(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)

{
	u8 i2c_ver[] = {0x00, 0x00};

	bq27505_control_subcommand(FW_VERSION, (u16 *) i2c_ver);
	return sprintf(buf, "%d\n", i2c_ver[0] + 256*i2c_ver[1]);
}

static ssize_t bq27505_show_battery_id(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	u8 manuf_buf[32];
	char battery[30];
	int i;

	memset(manuf_buf, 0, sizeof(manuf_buf));

	i2c_smbus_write_byte_data(bq27505_client, DATA_FLASH_CLASS, 0x39);
	i2c_smbus_write_byte_data(bq27505_client, DATA_FLASH_BLOCK, 0x00);
	mdelay(DELAY_AFTER_FIRMWARE_UPGRADE);
	i2c_smbus_read_i2c_block_data(bq27505_client, BLOCK_DATA,
					 sizeof(manuf_buf), manuf_buf);

	for (i = 0; i < sizeof(battery); i++)
		battery[i] = (char)(manuf_buf[i+2]);

	return sprintf(buf, "%s\n", battery);
}

static DEVICE_ATTR(firmware_vers, 0444, bq27505_show_firmware_vers, NULL);
static DEVICE_ATTR(dataflash_vers, 0444, bq27505_show_dataflash_vers, NULL);
static DEVICE_ATTR(battery_id, 0444, bq27505_show_battery_id, NULL);

static struct attribute *bq27505_attrs[] = {
	&dev_attr_firmware_vers.attr,
	&dev_attr_dataflash_vers.attr,
	&dev_attr_battery_id.attr,
	NULL,
};

static struct attribute_group bq27505_attr_grp = {
	.attrs = bq27505_attrs,
};

static int bq27505_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	bq27505_client = client;

	INIT_WORK(&work, bq27505_work_func);

	/* SOC_INT is asserted low for 2 msec.
	 * Rising Edge interrupt to catch the end of the pulse */
	if (client->irq) {
		(void) request_irq(client->irq, bq7505_irq_handler,
					 IRQF_TRIGGER_RISING,
						 "bq27505", &work);
	}

	init_timer(&temperature_timer);
	temperature_timer.function = temperature_timer_fn;
	temperature_timer.expires = jiffies + 60*HZ;

	add_timer(&temperature_timer);

	misc_register(&bq27505_device);

	bq27505_debug_init();

	(void) sysfs_create_group(&client->dev.kobj, &bq27505_attr_grp);

	assign_get_function(POWER_SUPPLY_PROP_TEMP,	battery_temp);
	assign_get_function(POWER_SUPPLY_PROP_VOLTAGE_NOW, battery_voltage);
	assign_get_function(POWER_SUPPLY_PROP_CAPACITY,	battery_capacity);

	return 0;
}

static int bq27505_remove(struct i2c_client *client)
{
	assign_get_function(POWER_SUPPLY_PROP_TEMP,	NULL);
	assign_get_function(POWER_SUPPLY_PROP_VOLTAGE_NOW,	NULL);
	assign_get_function(POWER_SUPPLY_PROP_CAPACITY,	NULL);

	return 0;
}

static const struct i2c_device_id bq27505_id[] = {
	{ "bq27505", 0 },
	{ }
};

static struct i2c_driver bq27505_driver = {
	.id_table = bq27505_id,
	.driver = {
		.name    = "bq27505",
		.owner   = THIS_MODULE,
	},
	.probe =  bq27505_probe,
	.remove = bq27505_remove,
};

static int __init bq27505_init(void)
{
	return i2c_add_driver(&bq27505_driver);
}

static void __exit bq27505_exit(void)
{
	i2c_del_driver(&bq27505_driver);
}

late_initcall(bq27505_init);
module_exit(bq27505_exit);

#ifdef CONFIG_DEBUG_FS

struct bq27505_debugfs_command {
	u8    command;
	bool  (*do_command) (u8 command, u16 *data);
};

struct bq27505_debugfs_file {
	const char	*name;
	struct bq27505_debugfs_command   command;
};

static struct bq27505_debugfs_file debugfs_files[] = {
	{ "control",
		 { CONTROL,	bq27505_standard_command } },
	{ "at_rate",
		 { AT_RATE,	bq27505_standard_command } },
	{ "at_rate_time_to_empty",
		 { AT_RATE_TIME_TO_EMPTY,	bq27505_standard_command } },
	{ "temperature",
		 { TEMPERATURE,	bq27505_standard_command } },
	{ "voltage",
		 { VOLTAGE,	bq27505_standard_command } },
	{ "flags",
		 { FLAGS,	bq27505_standard_command } },
	{ "nominal_available_capacity",
		 { NOMINAL_AVAILABLE_CAPACITY,	bq27505_standard_command } },
	{ "full_available_capacity",
		 { FULL_AVAILABLE_CAPACITY,	bq27505_standard_command } },
	{ "remaining_capacity",
		 { REMAINING_CAPACITY,		bq27505_standard_command } },
	{ "full_charge_capacity",
		 { FULL_CHARGE_CAPACITY,	bq27505_standard_command } },
	{ "average_current",
		 { AVERAGE_CURRENT,		bq27505_standard_command } },
	{ "time_to_empty",
		 { TIME_TO_EMPTY,		bq27505_standard_command } },
	{ "time_to_full",
		 { TIME_TO_FULL,		bq27505_standard_command } },
	{ "standby_current",
		 { STANDBY_CURRENT,		bq27505_standard_command } },
	{ "standby_time_to_empty",
		 { STANDBY_TIME_TO_EMPTY,	bq27505_standard_command } },
	{ "max_load_current",
		 { MAX_LOAD_CURRENT,		bq27505_standard_command } },
	{ "max_load_time_to_empty",
		 { MAX_LOAD_CURRENT,		bq27505_standard_command } },
	{ "available_energy",
		 { AVAILABLE_ENERGY,		bq27505_standard_command } },
	{ "average_power",
		 { AVERAGE_POWER,		bq27505_standard_command } },
	{ "time_to_empty_at_constant_power",
		 { TIME_TO_EMPTY_AT_CONSTANT_POWER, bq27505_standard_command } },
	{ "state_of_health",
		 { STATE_OF_HEALTH,		bq27505_standard_command } },
	{ "state_of_charge",
		 { STATE_OF_CHARGE,		bq27505_standard_command } },
	{ "normalized_impedance",
		 { NORMALIZED_IMPEDANCE,	bq27505_standard_command } },
	{ "instant_current",
		 { INSTANT_CURRENT,		bq27505_standard_command } },
	{ "control_status",
		 { CONTROL_STATUS,		bq27505_control_subcommand } },
	{ "device_type",
		 { DEVICE_TYPE,			bq27505_control_subcommand } },
	{ "fw_version",
		 { FW_VERSION,			bq27505_control_subcommand } },
	{ "hw_version",
		 { HW_VERSION,			bq27505_control_subcommand } },
	{ "data_flash_checksum",
		 { DATA_FLASH_CHECKSUM,		bq27505_control_subcommand } },
	{ "write_temperature",
		 { WRITE_TEMPERATURE,		bq27505_control_subcommand } },
	{ "prev_command",
		 { PREV_COMMAND,		bq27505_control_subcommand } },
	{ "chem_id",
		 { CHEM_ID,			bq27505_control_subcommand } },
	{ "compute_board_offset",
		 { COMPUTE_BOARD_OFFSET,	bq27505_control_subcommand } },
	{ "compute_coulomb_counter_offset",
		 { COMPUTE_COULOMB_COUNTER_OFFSET, bq27505_control_subcommand } },
	{ "write_coulomb_counter_offset",
		 { WRITE_COULOMB_COUNTER_OFFSET, bq27505_control_subcommand } },
	{ "ocv_cmd",
		 { OCV_CMD,			bq27505_control_subcommand } },
	{ "bat_insert",
		 { BAT_INSERT,			bq27505_control_subcommand } },
	{ "bat_remove",
		 { BAT_REMOVE,			bq27505_control_subcommand } },
	{ "set_hibernate",
		 { SET_HIBERNATE,		bq27505_control_subcommand } },
	{ "clear_hibernate",
		 { CLEAR_HIBERNATE,		bq27505_control_subcommand } },
	{ "set_sleep_plus",
		 { SET_SLEEP_PLUS,		bq27505_control_subcommand } },
	{ "clear_sleep_plus",
		 { CLEAR_SLEEP_PLUS,		bq27505_control_subcommand } },
	{ "seal",
		 { SEAL,			bq27505_control_subcommand } },
	{ "enable_impedance_track",
		 { ENABLE_IMPEDANCE_TRACK,	bq27505_control_subcommand } },
	{ "disable_impedance_track",
		 { DISABLE_IMPEDANCE_TRACK,	bq27505_control_subcommand } },
	{ "calibration_mode",
		 { CALIBRATION_MODE,		bq27505_control_subcommand } },
	{ "reset",
		 { RESET,			bq27505_control_subcommand } },
};

static int bq27505_debug_get(void *data, u64 *val)
{
	u16 value = -1;

	struct bq27505_debugfs_command *command =
				 (struct bq27505_debugfs_command *) data;

	command->do_command(command->command, &value);

	*val = value;
	return 0;
}

static int bq27505_debug_rom_mode(void *data, u64 *val)
{
	disable_irq(bq27505_client->irq);
	flush_work(&work);

	*val = bq27505_enter_rom_mode();

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(bq27505_debug_fops, bq27505_debug_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(bq27505_debug_rom_mode_fops, bq27505_debug_rom_mode,
							 NULL, "%llu\n");

static int __init bq27505_debug_init(void)
{
	struct dentry *dent = debugfs_create_dir("bq27505", 0);
	int	i;

	if (!IS_ERR(dent)) {
		for (i = 0; i < ARRAY_SIZE(debugfs_files); i++) {
			debugfs_create_file(debugfs_files[i].name, 0444, dent,
				 &debugfs_files[i].command,
				 &bq27505_debug_fops);
		}

	debugfs_create_file("rom_mode", 0444, dent, NULL,
				 &bq27505_debug_rom_mode_fops);
	}

	return 0;
}

#endif /* CONFIG_DEBUG_FS */
