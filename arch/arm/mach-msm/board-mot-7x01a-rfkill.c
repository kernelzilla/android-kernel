/*
 * Copyright (C) 2011 CyanogenMod
 * Author: Patrick Jacques <kernelzilla@kinetic-computing.com>
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

/* Control bluetooth power for mot-7x01a platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <mach/msm_serial_hs.h>

#include "board-mot.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4325";

enum {
	BT_WAKE,
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_PCM_DOUT,
	BT_PCM_DIN,
	BT_PCM_SYNC,
	BT_PCM_CLK,
	BT_HOST_WAKE,
};

static unsigned bt_config_power_on[] = {
/* GPIO_CFG(91, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), */ /* WAKE */
    GPIO_CFG(91, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),    /* WAKE */
    GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),    /* RFR */
    GPIO_CFG(44, 2, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA),    /* CTS */
    GPIO_CFG(45, 2, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA),    /* Rx */
    GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),    /* Tx */
    GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_KEEPER, GPIO_2MA),    /* PCM_DOUT */
    GPIO_CFG(69, 1, GPIO_INPUT,  GPIO_KEEPER, GPIO_2MA),    /* PCM_DIN */
    GPIO_CFG(70, 2, GPIO_OUTPUT, GPIO_KEEPER, GPIO_2MA),    /* PCM_SYNC */
    GPIO_CFG(71, 2, GPIO_OUTPUT, GPIO_KEEPER, GPIO_2MA),    /* PCM_CLK */
    GPIO_CFG(90, 0, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA),    /* HOST_WAKE */
};
static unsigned bt_config_power_off[] = {
/*    GPIO_CFG(91, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),*/  /* WAKE */
    GPIO_CFG(91, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),    /* WAKE */
    GPIO_CFG(43, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* RFR */
    GPIO_CFG(44, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* CTS */
    GPIO_CFG(45, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* Rx */
    GPIO_CFG(46, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* Tx */
    GPIO_CFG(68, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* PCM_DOUT */
    GPIO_CFG(69, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* PCM_DIN */
    GPIO_CFG(70, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* PCM_SYNC */
    GPIO_CFG(71, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* PCM_CLK */
    GPIO_CFG(90, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),    /* HOST_WAKE */
};

static int bluetooth_set_power(void *data, enum rfkill_state state)
{

	struct vreg *vreg_bt;
	int pin, rc;

	vreg_bt = vreg_get(0, "gp3");

	switch (state) {
	case RFKILL_STATE_UNBLOCKED:
	
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_request(0x3ff & (bt_config_power_on[pin] >> 4),
				"BT PIN");
			if (rc) {
				printk(KERN_ERR
				"%s.%d: gpio_request()=%d [gpio #%d, array idx=%d]\n",
				__func__, __LINE__, rc,
				0x3ff & (bt_config_power_on[pin] >> 4), pin);
			}

			gpio_tlmm_config(bt_config_power_on[pin], GPIO_ENABLE);
		}

        rc = vreg_set_level(vreg_bt, 2600);
        if (rc) {
                printk(KERN_ERR "%s.%d: vreg set level failed (%d)\n",
                        __func__, __LINE__, rc);
                return -EIO;
        }

		vreg_enable(vreg_bt);
		msleep_interruptible(100);
		gpio_request(BT_REG_ON_SIGNAL, "bt_reg_on");  // turn on internal BT VREG
		gpio_direction_output(BT_REG_ON_SIGNAL, 1);
		msleep_interruptible(100);    // BCM4325 powerup requirement
		gpio_request(BT_RESET_N_SIGNAL, "bt_reset_n"); // take BT out of reset
		gpio_direction_output(BT_RESET_N_SIGNAL, 1);
		printk(KERN_INFO "BLUETOOTH: mot_rfkill: ON\n");
		break;
	case RFKILL_STATE_SOFT_BLOCKED:
		gpio_set_value (BT_RESET_N_SIGNAL, 0); // put BT into reset
		gpio_set_value (BT_REG_ON_SIGNAL, 0); // turn off internal BT VREG
		msleep_interruptible(100);
		vreg_disable(vreg_bt);
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin],
							GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s.%d: gpio_tlmm_config(%#x)=%d\n",
					__func__, __LINE__, bt_config_power_off[pin], rc);
				return -EIO;
				}
		}
		printk(KERN_INFO "BLUETOOTH: mot_rfkill: OFF\n");
		break;
	default:
		printk(KERN_ERR "bad bluetooth rfkill state %d\n", state);
	}
	return 0;
}

static int mot_rfkill_probe(struct platform_device *pdev)
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
	bt_rfk->data = NULL;  // user data
	bt_rfk->toggle_radio = bluetooth_set_power;

	rc = rfkill_register(bt_rfk);

	if (rc)
		rfkill_free(bt_rfk);
	return rc;
}

static int mot_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_free(bt_rfk);

	return 0;
}

static struct platform_driver mot_rfkill_driver = {
	.probe = mot_rfkill_probe,
	.remove = mot_rfkill_remove,
	.driver = {
		.name = "mot_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init mot_rfkill_init(void)
{
	return platform_driver_register(&mot_rfkill_driver);
}

static void __exit mot_rfkill_exit(void)
{
	platform_driver_unregister(&mot_rfkill_driver);
}

module_init(mot_rfkill_init);
module_exit(mot_rfkill_exit);
MODULE_DESCRIPTION("mot 7x01a rfkill");
MODULE_AUTHOR("Patrick Jacques <kernelzilla@kinetic-computing.com>");
MODULE_LICENSE("GPL");
