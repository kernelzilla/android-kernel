/* timed_vibrator.c
 *
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
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <mach/gpio.h>
#include <asm-arm/mach-types.h>

#include <linux/workqueue.h>
#include "timed_vibrator.h"


#define VIBRATOR_PULSE_WIDTH_NS(hz) (1000000000/(hz * 2))

static struct class *vibrator_class = NULL;

struct vibrator_data {
	struct device *dev;
	struct hrtimer timer_en;
	struct hrtimer timer_pwm;
	spinlock_t lock;
	unsigned 	gpio_en;
	unsigned 	gpio_pwm;
	int 		max_timeout;
	unsigned char 	active_low;
	struct work_struct   work_en;  
	int 		state_en;
	int 		state_pwm;
        int             cur_hz;
        int             end_hz;
        int             step_ns;    /* # of us before incrementing frequency */
        int             cur_step;   /* # of cycles since last frequency change */
        int             max_step;   /* # of cycles before incrementing frequency */
};

static struct vibrator_data vib_data;


static void vibrator_en_work_func( struct work_struct *work_en)
{
	gpio_direction_output(vib_data.gpio_en,  vib_data.active_low ? !vib_data.state_en : !!vib_data.state_en );
}


static enum hrtimer_restart vibrator_en_timer_func(struct hrtimer *timer_en)
{
        vib_data.state_en=0;
	schedule_work(&vib_data.work_en);
	return HRTIMER_NORESTART;
}

static void vibrator_pwm_toggle(void)
{
        if (vib_data.state_en) {
	        gpio_direction_output(vib_data.gpio_pwm,  vib_data.state_pwm);
                hrtimer_start(&vib_data.timer_pwm, ktime_set(0, VIBRATOR_PULSE_WIDTH_NS(vib_data.cur_hz)), HRTIMER_MODE_REL);
        }
        else {
	        gpio_direction_output(vib_data.gpio_pwm,  vib_data.state_pwm = 0);

        }
}

static enum hrtimer_restart vibrator_pwm_timer_func(struct hrtimer *timer_pwm)
{
        vib_data.state_pwm^=1;

        if (vib_data.end_hz > vib_data.cur_hz) {
                if (++vib_data.cur_step >= vib_data.max_step) {
                        vib_data.cur_hz++;
                        vib_data.cur_step = 0;
                        vib_data.max_step = vib_data.step_ns / VIBRATOR_PULSE_WIDTH_NS(vib_data.cur_hz);
                }
        }

        vibrator_pwm_toggle();        /* Cannot delay execution via work_queue */

	return HRTIMER_NORESTART;
}

static ssize_t vibrator_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int remaining;

	if (hrtimer_active(&vib_data.timer_en)) {
		ktime_t r = hrtimer_get_remaining(&vib_data.timer_en);
		remaining = r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		remaining = 0;

	return sprintf(buf, "%d\n", remaining);
}

static void vibrator_start(int value)
{
	/* cancel previous timer and set GPIO according to value */
	hrtimer_cancel(&vib_data.timer_en);
        hrtimer_cancel(&vib_data.timer_pwm);
	vib_data.state_en=!!value;
	schedule_work(&vib_data.work_en);

	if (value > 0) {
		if (value > vib_data.max_timeout)
			value = vib_data.max_timeout;

		vib_data.state_pwm=1;
		hrtimer_start(&vib_data.timer_en,
						ktime_set(value / 1000, (value % 1000) * 1000000),
						HRTIMER_MODE_REL);
	}
	
	vibrator_pwm_toggle();
}

static ssize_t vibrator_enable_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int value;
	unsigned long	flags;

	sscanf(buf, "%d", &value);

	spin_lock_irqsave(&vib_data.lock, flags);

        vib_data.cur_hz = STIMULUS_A_VIBRATOR_START_HZ;
        vib_data.end_hz = STIMULUS_A_VIBRATOR_END_HZ;
        vib_data.step_ns = (STIMULUS_A_VIBRATOR_SWEEP_TIME_MS * 1000 * 1000) / 
                               (STIMULUS_A_VIBRATOR_END_HZ - STIMULUS_A_VIBRATOR_START_HZ);
        vib_data.cur_step = 0;
        vib_data.max_step = vib_data.step_ns / VIBRATOR_PULSE_WIDTH_NS(vib_data.cur_hz);

        vibrator_start(value);

	spin_unlock_irqrestore(&vib_data.lock, flags);

	return size;
}

void vibrator_activate(int value)
{
	unsigned long	flags;

	if ((vib_data.dev == NULL) || (unlikely(IS_ERR(vib_data.dev))) || (vib_data.state_en))
           return;

        spin_lock_irqsave(&vib_data.lock, flags);
        vib_data.cur_hz = vib_data.end_hz = STIMULUS_B_VIBRATOR_HZ;

        vibrator_start(value);

        spin_unlock_irqrestore(&vib_data.lock, flags);
}

EXPORT_SYMBOL(vibrator_activate);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, vibrator_enable_show, vibrator_enable_store);

static int vibrator_probe(struct platform_device *pdev)
{
	struct vibrator_platform_data *pdata = pdev->dev.platform_data;
	int  ret = 0;

	if (!pdata)
		return -EBUSY;

	INIT_WORK(&vib_data.work_en, vibrator_en_work_func);
			
	hrtimer_init(&vib_data.timer_en, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib_data.timer_en.function = vibrator_en_timer_func;
        hrtimer_init(&vib_data.timer_pwm, CLOCK_REALTIME, HRTIMER_MODE_REL);
	vib_data.timer_pwm.function = vibrator_pwm_timer_func;
	spin_lock_init(&vib_data.lock);

	vib_data.gpio_en = pdata->gpio_en;
	vib_data.gpio_pwm = pdata->gpio_pwm;
	vib_data.max_timeout = pdata->max_timeout;
	vib_data.active_low = pdata->active_low;
	gpio_direction_output(vib_data.gpio_en, vib_data.active_low);
	gpio_direction_output(vib_data.gpio_pwm, 0);

	vib_data.dev = device_create(vibrator_class, &pdev->dev, 0, "%s", pdata->name);

	if (unlikely(IS_ERR(vib_data.dev)))
	  return PTR_ERR(vib_data.dev);

	dev_set_drvdata(vib_data.dev, &vib_data);
	ret = device_create_file(vib_data.dev, &dev_attr_enable);

	if (ret)
          return ret; 


	platform_set_drvdata(pdev, &vib_data);
        vib_data.state_en=0;
	schedule_work(&vib_data.work_en);

	return 0;
}

static int vibrator_remove(struct platform_device *pdev)
{
	device_remove_file(vib_data.dev, &dev_attr_enable);
	device_unregister(vib_data.dev);

	return 0;
}

static struct platform_driver vibrator_driver = {
	.probe		= vibrator_probe,
	.remove		= vibrator_remove,
	.driver		= {
		.name		= "vibrator",
		.owner		= THIS_MODULE,
	},
};

static int __init vibrator_init(void)
{
	vibrator_class = class_create(THIS_MODULE, "vibrator");
	if (IS_ERR(vibrator_class))
		return PTR_ERR(vibrator_class);
	return platform_driver_register(&vibrator_driver);
}

static void __exit vibrator_exit(void)
{
	class_destroy(vibrator_class);
	platform_driver_unregister(&vibrator_driver);
}

late_initcall(vibrator_init);
module_exit(vibrator_exit);


MODULE_AUTHOR("Vladimir Karpovich <Vladimir.Karpovich@motorola.com>");
MODULE_DESCRIPTION("Morrison Linear Vibrator driver");
MODULE_LICENSE("GPL");
