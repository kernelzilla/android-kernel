/* linux/arch/arm/mach-msm/board-sapphire-rfkill.c
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

/* Control bluetooth power for sapphire platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include "gpio_chip.h"
#include "board-sapphire.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "brf6300";

extern int sapphire_bt_fastclock_power(int on);

static int bluetooth_set_power(void *data, enum rfkill_state state)
{
	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
		sapphire_bt_fastclock_power(1);
		gpio_set_value(SAPPHIRE_GPIO_BT_32K_EN, 1);
		udelay(10);
		gpio_configure(101, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_HIGH);
		break;
	case RFKILL_STATE_SOFT_BLOCKED:
		gpio_configure(101, GPIOF_DRIVE_OUTPUT | GPIOF_OUTPUT_LOW);
		gpio_set_value(SAPPHIRE_GPIO_BT_32K_EN, 0);
		sapphire_bt_fastclock_power(0);
		break;
	default:
		printk(KERN_ERR "bad bluetooth rfkill state %d\n", state);
	}
	return 0;
}

static int sapphire_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	enum rfkill_state default_state = RFKILL_STATE_SOFT_BLOCKED;  /* off */

	rfkill_set_default(RFKILL_TYPE_BLUETOOTH, default_state);
	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);
	if (!bt_rfk)
		return -ENOMEM;

	bt_rfk->name = bt_name;
	bt_rfk->state = default_state;
	/* userspace cannot take exclusive control */
	bt_rfk->user_claim_unsupported = 1;
	bt_rfk->user_claim = 0;
	bt_rfk->data = NULL;  /* user data */
	bt_rfk->toggle_radio = bluetooth_set_power;

	rc = rfkill_register(bt_rfk);

	if (rc)
		rfkill_free(bt_rfk);
	return rc;
}

static int sapphire_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_free(bt_rfk);

	return 0;
}

static struct platform_driver sapphire_rfkill_driver = {
	.probe = sapphire_rfkill_probe,
	.remove = sapphire_rfkill_remove,
	.driver = {
		.name = "sapphire_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init sapphire_rfkill_init(void)
{
	return platform_driver_register(&sapphire_rfkill_driver);
}

static void __exit sapphire_rfkill_exit(void)
{
	platform_driver_unregister(&sapphire_rfkill_driver);
}

module_init(sapphire_rfkill_init);
module_exit(sapphire_rfkill_exit);
MODULE_DESCRIPTION("sapphire rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
