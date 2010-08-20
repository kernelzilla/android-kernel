/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
//#include <linux/workqueue.h>
#include <linux/leds-rt9378.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include <mach/gpio.h>

#define RT9378_LEVELS 16 /* RT9378 supports 16 levels brightness, valid 0~16 */
#define RT9378_STEP_VALUE ((LED_FULL-LED_OFF)/RT9378_LEVELS + 1)
#define RT9378_FIRST_DELAY (31) /* delay 31 usecs for first high */
#define RT9378_SHUTDOWN_DELAY (4) /* delay 5ms for shut down */
#define RT9378_DELAY (1)

struct rt9378_data {
	struct led_classdev cdev;
    struct mutex lock;
	unsigned gpio;
	u8 cur_level;
};

static void rt9378_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct rt9378_data *led_dat =
		container_of(led_cdev, struct rt9378_data, cdev);
	u8 level;
    u8 interval = RT9378_STEP_VALUE; /* make an integer */

    if(value > LED_FULL)
        value = LED_FULL;

    level = (value + interval - 1) / interval;

    /* equal to current level, just return*/
	if (led_dat->cur_level == level)
        return;

    mutex_lock(&led_dat->lock);
    led_dat->cur_level = level;
	gpio_set_value(led_dat->gpio, 0);
    mdelay(RT9378_SHUTDOWN_DELAY);
    if(level > 0){
        gpio_set_value(led_dat->gpio, 1);
        udelay(RT9378_FIRST_DELAY);
        /* generate pulses to proper level */
        while(level < RT9378_LEVELS){
            gpio_set_value(led_dat->gpio, 0);
            udelay(RT9378_DELAY);
            gpio_set_value(led_dat->gpio, 1);
            udelay(RT9378_DELAY);
            level++;
        }
    }
    mutex_unlock(&led_dat->lock);

    return;
}

static int rt9378_probe(struct platform_device *pdev)
{
	struct rt9378_platform_data *pdata = pdev->dev.platform_data;
	struct rt9378_data *leds_dat;
	int ret = 0;

	if (!pdata)
		return -EBUSY;

	leds_dat = kzalloc(sizeof(struct rt9378_data),	GFP_KERNEL);

	if (!leds_dat)
		return -ENOMEM;


    ret = gpio_request(pdata->gpio_en, pdata->name);
    if (ret < 0)
        goto err_gpio;

    mutex_init(&leds_dat->lock);

    leds_dat->cdev.name = pdata->name;
    leds_dat->gpio = pdata->gpio_en;
    leds_dat->cdev.brightness_set = rt9378_set;
    leds_dat->cdev.brightness = LED_OFF;
    leds_dat->cdev.flags |= LED_CORE_SUSPENDRESUME;

    ret = led_classdev_register(&pdev->dev, &leds_dat->cdev);

    if (ret < 0)
        goto err;

	platform_set_drvdata(pdev, leds_dat);

	return 0;

err:
    led_classdev_unregister(&leds_dat->cdev);
err_gpio:
    gpio_free(leds_dat->gpio);
	kfree(leds_dat);

	return ret;
}

static int __devexit rt9378_remove(struct platform_device *pdev)
{
	struct rt9378_data *leds_data;

	leds_data = platform_get_drvdata(pdev);

    led_classdev_unregister(&leds_data->cdev);
    gpio_free(leds_data->gpio);

	kfree(leds_data);

	return 0;
}

static struct platform_driver rt9378_driver = {
	.probe		= rt9378_probe,
	.remove		= __devexit_p(rt9378_remove),
	.driver		= {
		.name	= "rt9378",
		.owner	= THIS_MODULE,
	},
};

static int __init rt9378_init(void)
{
	return platform_driver_register(&rt9378_driver);
}

static void __exit rt9378_exit(void)
{
	platform_driver_unregister(&rt9378_driver);
}

module_init(rt9378_init);
module_exit(rt9378_exit);

MODULE_AUTHOR("Li Leon <leonli@motorola.com>");
MODULE_DESCRIPTION("RT9378 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:leds-rt9378");
