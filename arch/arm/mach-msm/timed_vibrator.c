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
#include <linux/gpio.h>
//#include <mach/mach-types.h>
#include <linux/workqueue.h>
#include "timed_vibrator.h"

static struct class *vibrator_class;

struct vibrator_data {
	struct device *dev;
	struct hrtimer timer;
	spinlock_t lock;
	unsigned 	gpio_en;
	unsigned 	gpio_pwm;
	int 		max_timeout;
	unsigned char 	active_low;
	struct work_struct   work;  
	int 		state;

};

int msm_tlmm_gp_clk_config(unsigned char  enable_disable)
{
	int rc = 0;

	if( enable_disable )
		gpio_tlmm_config(GPIO_CFG(27, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);	// GP_CLK
	else
               gpio_tlmm_config(GPIO_CFG(27, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_DISABLE); // GPIO_OUT_27
//	printk( "msm_tlmm_gp_clk_config :SET  VALUE %d\n",enable_disable);
	return rc;
}

static void vibrator_work_func( struct work_struct *work)
{
	struct vibrator_data *vib_data = container_of(work, struct vibrator_data, work);
	gpio_direction_output(vib_data->gpio_en,  vib_data->active_low ? !vib_data->state : !!vib_data->state );
        msm_tlmm_gp_clk_config(vib_data->state);
}


static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct vibrator_data *vib_data = container_of(timer, struct vibrator_data, timer);
        vib_data->state=0;
	schedule_work(&vib_data->work);
	return HRTIMER_NORESTART;
}

static ssize_t vibrator_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vibrator_data *vib_data = dev_get_drvdata(dev);
	int remaining;

	if (hrtimer_active(&vib_data->timer)) {
		ktime_t r = hrtimer_get_remaining(&vib_data->timer);
		remaining = r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		remaining = 0;

	return sprintf(buf, "%d\n", remaining);
}

static ssize_t vibrator_enable_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct vibrator_data *vib_data = dev_get_drvdata(dev);
	int value;
	unsigned long	flags;

	sscanf(buf, "%d", &value);

	spin_lock_irqsave(&vib_data->lock, flags);

	/* cancel previous timer and set GPIO according to value */
	hrtimer_cancel(&vib_data->timer);
        vib_data->state=!!value;
	schedule_work(&vib_data->work);

	if (value > 0) {
		if (value > vib_data->max_timeout)
			value = vib_data->max_timeout;

		hrtimer_start(&vib_data->timer,
						ktime_set(value / 1000, (value % 1000) * 1000000),
						HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&vib_data->lock, flags);

	return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, vibrator_enable_show, vibrator_enable_store);

static int vibrator_probe(struct platform_device *pdev)
{
	struct vibrator_platform_data *pdata = pdev->dev.platform_data;
	struct vibrator_data *vib_data;
	int  ret = 0;

	if (!pdata)
		return -EBUSY;

	vib_data = kzalloc(sizeof(struct vibrator_data), GFP_KERNEL);
	if (!vib_data)
		return -ENOMEM;

         INIT_WORK(&vib_data->work, vibrator_work_func);
		
	 hrtimer_init(&vib_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	 vib_data->timer.function = vibrator_timer_func;
	 spin_lock_init(&vib_data->lock);

	 vib_data->gpio_en = pdata->gpio_en;
	 vib_data->gpio_pwm = pdata->gpio_pwm;
	 vib_data->max_timeout = pdata->max_timeout;
	 vib_data->active_low = pdata->active_low;
	 gpio_direction_output(vib_data->gpio_en, vib_data->active_low);

	 vib_data->dev = device_create(vibrator_class, &pdev->dev, 0, "%s", pdata->name);

	 if (unlikely(IS_ERR(vib_data->dev)))
	 	 return PTR_ERR(vib_data->dev);

	 dev_set_drvdata(vib_data->dev, vib_data);
	 ret = device_create_file(vib_data->dev, &dev_attr_enable);
	 if (ret)
	 	 return ret;


	platform_set_drvdata(pdev, vib_data);
        vib_data->state=0;
	schedule_work(&vib_data->work);

	return 0;
}

static int vibrator_remove(struct platform_device *pdev)
{
	struct vibrator_data *vib_data = platform_get_drvdata(pdev);
	device_remove_file(vib_data->dev, &dev_attr_enable);
	device_unregister(vib_data->dev);
	kfree(vib_data);

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

//device_initcall(vibrator_init);
late_initcall(vibrator_init);
module_exit(vibrator_exit);


MODULE_AUTHOR("Vladimir Karpovich <Vladimir.Karpovich@motorola.com>");
MODULE_DESCRIPTION("Morrison Linear Vibrator driver");
MODULE_LICENSE("GPL");
