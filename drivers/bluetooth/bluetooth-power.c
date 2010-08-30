/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Bluetooth Power Switch Module
 * controls power to external Bluetooth device
 * with interface to power management device
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>

static int bluetooth_power_state;
static int (*power_control)(int enable);

static DEFINE_SPINLOCK(bt_power_lock);

static int bluetooth_toggle_radio(void *data, enum rfkill_state state)
{
	int ret;

	spin_lock(&bt_power_lock);
	ret = (*power_control)((state == RFKILL_STATE_UNBLOCKED) ? 1 : 0);
	spin_unlock(&bt_power_lock);
	return ret;
}

static int bluetooth_power_rfkill_probe(struct platform_device *pdev)
{
	struct rfkill *rfkill;
	int ret = -ENOMEM;

	/* force Bluetooth off during init to allow for user control */
	rfkill_switch_all(RFKILL_TYPE_BLUETOOTH, RFKILL_STATE_SOFT_BLOCKED);

	rfkill = rfkill_allocate(&pdev->dev, RFKILL_TYPE_BLUETOOTH);

	if (rfkill) {
		rfkill->name = "bt_power";
		rfkill->toggle_radio = bluetooth_toggle_radio;
		rfkill->state = bluetooth_power_state ?
					RFKILL_STATE_UNBLOCKED :
					RFKILL_STATE_SOFT_BLOCKED;
		ret = rfkill_register(rfkill);
		if (ret) {
			printk(KERN_DEBUG
				"%s: rfkill register failed=%d\n", __func__,
				ret);
			rfkill_free(rfkill);
		} else
			platform_set_drvdata(pdev, rfkill);
	}
	return ret;
}

static void bluetooth_power_rfkill_remove(struct platform_device *pdev)
{
	struct rfkill *rfkill;

	rfkill = platform_get_drvdata(pdev);
	if (rfkill)
		rfkill_unregister(rfkill);

	platform_set_drvdata(pdev, NULL);
}

static int bluetooth_power_param_set(const char *val, struct kernel_param *kp)
{
	int ret;

	printk(KERN_DEBUG
		"%s: previous power_state=%d\n",
		__func__, bluetooth_power_state);

	/* lock change of state and reference */
	spin_lock(&bt_power_lock);
	ret = param_set_bool(val, kp);
	if (power_control) {
		if (!ret)
			ret = (*power_control)(bluetooth_power_state);
		else
			printk(KERN_ERR "%s param set bool failed (%d)\n",
					__func__, ret);
	} else {
		printk(KERN_INFO
			"%s: deferring power switch until probe\n",
			__func__);
	}
	spin_unlock(&bt_power_lock);
	printk(KERN_INFO
		"%s: current power_state=%d\n",
		__func__, bluetooth_power_state);
	return ret;
}

module_param_call(power, bluetooth_power_param_set, param_get_bool,
		  &bluetooth_power_state, S_IWUSR | S_IRUGO);

static int __init_or_module bt_power_probe(struct platform_device *pdev)
{
	int ret = 0;

	printk(KERN_DEBUG "%s\n", __func__);

	if (!pdev->dev.platform_data) {
		printk(KERN_ERR "%s: platform data not initialized\n",
				__func__);
		return -ENOSYS;
	}

	spin_lock(&bt_power_lock);
	power_control = pdev->dev.platform_data;

	if (bluetooth_power_state) {
		printk(KERN_INFO
			"%s: handling deferred power switch\n",
			__func__);
	}
	ret = (*power_control)(bluetooth_power_state);
	spin_unlock(&bt_power_lock);

	if (!ret && !bluetooth_power_state &&
		    bluetooth_power_rfkill_probe(pdev))
		ret = -ENOMEM;

	return ret;
}

static int __devexit bt_power_remove(struct platform_device *pdev)
{
	int ret;

	printk(KERN_DEBUG "%s\n", __func__);

	bluetooth_power_rfkill_remove(pdev);

	if (!power_control) {
		printk(KERN_ERR "%s: power_control function not initialized\n",
				__func__);
		return -ENOSYS;
	}
	spin_lock(&bt_power_lock);
	bluetooth_power_state = 0;
	ret = (*power_control)(bluetooth_power_state);
	power_control = NULL;
	spin_unlock(&bt_power_lock);

	return ret;
}

static struct platform_driver bt_power_driver = {
	.probe = bt_power_probe,
	.remove = __devexit_p(bt_power_remove),
	.driver = {
		.name = "bt_power",
		.owner = THIS_MODULE,
	},
};

static int __init_or_module bluetooth_power_init(void)
{
	int ret;

	printk(KERN_DEBUG "%s\n", __func__);
	ret = platform_driver_register(&bt_power_driver);
	return ret;
}

static void __exit bluetooth_power_exit(void)
{
	printk(KERN_DEBUG "%s\n", __func__);
	platform_driver_unregister(&bt_power_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM Bluetooth power control driver");
MODULE_VERSION("1.10");
MODULE_PARM_DESC(power, "MSM Bluetooth power switch (bool): 0,1=off,on");

module_init(bluetooth_power_init);
module_exit(bluetooth_power_exit);
