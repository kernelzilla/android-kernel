/*
     Copyright (C) 2009 Motorola, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License version 2 as
     published by the Free Software Foundation.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
     02111-1307  USA
*/

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/omap_mdm_ctrl.h>
#include <linux/omap_mdm_ctrl_ext.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>

/* structure to keep track of gpio, irq, and irq enabled info */
struct gpioinfo {
	unsigned gpio;
	int irq;
	int irq_enabled;	/* Mask for each client */
	int irq_fired;		/* Mask for each client */
	int irq_level_rising;	/* Mask for each client */
	int irq_level_falling;	/* Mask for each client */
	struct work_struct work;
};

struct clientinfo {
	int open;
	int mask;
	wait_queue_head_t wq;
};
#define MAX_CLIENTS 4

/* BP Interface GPIO List*/
enum bpgpio {
	BP_READY_AP = 0,
	BP_READY2_AP,
	BP_RESOUT,
	BP_PWRON,
	AP_TO_BP_PSHOLD,
	AP_TO_BP_FLASH_EN,
	GPIO_COUNT = AP_TO_BP_FLASH_EN + 1,
};

/* Internal representation of the omap_mdm_ctrl driver. */
struct omap_mdm_ctrl_info {
	struct gpioinfo gpios[GPIO_COUNT];
	struct clientinfo clients[MAX_CLIENTS];
	struct semaphore sem;
	struct workqueue_struct *working_queue;
};

/* Driver operational structure */
static struct omap_mdm_ctrl_info omap_mdm_ctrl_data;

/* The detailed wait condition used in omap_mdm_ctrl_read().  Returns 0 only
 * when an irq has fired, otherwise always returns -1. */
int wait_condition(void)
{
	int ret = -1;
	short i = 0;

	for (i = 0; i < GPIO_INTERRUPT_COUNT; i++) {
		if (omap_mdm_ctrl_data.gpios[i].irq_fired) {
			ret = 0;
			break;
		}
	}

	return ret;
}

/* Frees all active interrupts and cleans up the gpio data structure */
void clear_gpio_data(void)
{
	int i;
	for (i = 0; i < GPIO_COUNT; i++) {
		if (omap_mdm_ctrl_data.gpios[i].irq != 0)
			free_irq(omap_mdm_ctrl_data.gpios[i].irq, NULL);
		omap_mdm_ctrl_data.gpios[i].irq_enabled = 0;
		omap_mdm_ctrl_data.gpios[i].irq_fired = 0;
		if (omap_mdm_ctrl_data.gpios[i].gpio != 0)
			gpio_free(omap_mdm_ctrl_data.gpios[i].gpio);
	}
}

/* Checks to see if the client should be notified of this irq */
int should_notify_client(struct gpioinfo *gpio, int ci)
{
	int ret = 0;
	int gpio_value = gpio_get_value(gpio->gpio);
	int mask = omap_mdm_ctrl_data.clients[ci].mask;

	if (gpio->irq_enabled & mask)
		switch (gpio_value) {
		case OMAP_MDM_CTRL_GPIO_HIGH:
			if (gpio->irq_level_rising & mask)
				ret = 1;
			break;
		case OMAP_MDM_CTRL_GPIO_LOW:
			if (gpio->irq_level_falling & mask)
				ret = 1;
			break;
		}

	return ret;
}

static void irq_work(struct work_struct *work)
{
	struct gpioinfo *gpio = container_of(work, struct gpioinfo, work);
	int i;
	int mask;

	if (gpio == &omap_mdm_ctrl_data.gpios[BP_RESOUT] &&
	    gpio_get_value(gpio->gpio) == 0) {
		pr_err("%s: BP powered off!\n", __func__);
	}

	/* Search through clients, waking those that are waiting on this irq */
	for (i = 0; i < MAX_CLIENTS; i++)
		if (omap_mdm_ctrl_data.clients[i].open) {
			mask = omap_mdm_ctrl_data.clients[i].mask;
			if (!(gpio->irq_fired & mask) &&
			    (should_notify_client(gpio, i))) {
				gpio->irq_fired |= mask;
				wake_up_interruptible(
					&omap_mdm_ctrl_data.clients[i].wq);
			}
		}

	enable_irq(gpio->irq);
}

/* When an interrupt is fired the 'irq_fired' bit for the GPIO's IRQ
 * that fired is set and all listening users are told to wake up. */
irqreturn_t irq_handler(int irq, void *data)
{
	struct gpioinfo *gpio = (struct gpioinfo *)data;

	disable_irq_nosync(irq);
	queue_work(omap_mdm_ctrl_data.working_queue, &gpio->work);

	return IRQ_HANDLED;
}

/* Called when a user opens a connection to the bp interface device.
 * Sets an open flag to indicate that no further connections to this
 * device are allowed. */
static int omap_mdm_ctrl_open(struct inode *inode, struct file *filp)
{
	int ci = 0;

	/* Hold the semaphore */
	if (down_interruptible(&omap_mdm_ctrl_data.sem)) {
		pr_err("%s: Unable to open device.\n", __func__);
		return -ERESTARTSYS;
	}

	/* Check for an open client */
	for (ci = 0; ci < MAX_CLIENTS; ci++)
		if (!omap_mdm_ctrl_data.clients[ci].open) {
			omap_mdm_ctrl_data.clients[ci].open = 1;
			break;
		}

	if (ci >= MAX_CLIENTS) {
		up(&omap_mdm_ctrl_data.sem);
		pr_info("%s: Device is busy.\n", __func__);
		return -EBUSY;
	}

	/* Store client structure on filp */
	filp->private_data = &omap_mdm_ctrl_data.clients[ci];

	/* Release the semaphore and return */
	up(&omap_mdm_ctrl_data.sem);
	return 0;
}

/* Called when a user releases their connection to the bp interface
 * device. Clears the flag thus allowing another user connection to
 * this device. */
static int omap_mdm_ctrl_release(struct inode *inode, struct file *filp)
{
	struct clientinfo *cinfo = (struct clientinfo *)filp->private_data;

	/* Hold the semaphore */
	if (down_interruptible(&omap_mdm_ctrl_data.sem)) {
		pr_err("%s: Unable to release device.\n", __func__);
		return -ERESTARTSYS;
	}

	/* Release the client structure */
	cinfo->open = 0;
	filp->private_data = 0;

	/* Release the semaphore and return */
	up(&omap_mdm_ctrl_data.sem);
	return 0;
}

/* Set the user on the wait queue and tell the user that data may be
 * available soon. */
static unsigned int omap_mdm_ctrl_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct clientinfo *cinfo = (struct clientinfo *)filp->private_data;

	poll_wait(filp, &(cinfo->wq), wait);

	if (wait_condition() == 0)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

/* Returns a 1 byte bitmask containing the irq "fire status" of all
 * the BP Interface GPIOs that are capable of experiencing an interrupt.
 * Will return a value of 1 for a particular GPIO only if that GPIO's
 * IRQ is enabled and has fired.  At this point the 'fired' flag
 * for that GPIO is cleared. */
static ssize_t omap_mdm_ctrl_read(struct file *filp, char __user * buff,
				  size_t count, loff_t *f_pos)
{
	ssize_t ret = -EINVAL;
	char gpioData[GPIO_BYTE_COUNT];
	int i;
	struct clientinfo *cinfo;
	int mask;

	/* Hold the semaphore */
	if (down_interruptible(&omap_mdm_ctrl_data.sem)) {
		pr_err("%s: Unable to read device.\n", __func__);
		return -ERESTARTSYS;
	}

	cinfo = (struct clientinfo *)filp->private_data;
	if (!cinfo) {
		pr_info("%s: File pointer invalid.\n", __func__);
		return -EBADF;
	}
	mask = cinfo->mask;

	if (cinfo->open) {
		/* Inform calling process to sleep until the interrupt for
		 * any GPIO is fired */
		wait_event_interruptible(cinfo->wq, (wait_condition() == 0));

		ret = 0;
		if (*f_pos >= GPIO_BYTE_COUNT)
			goto out;

		memset(gpioData, 0, GPIO_BYTE_COUNT);

		/* Process the list of active interrupts, returning those
		 * that have fired */
		for (i = 0; i < GPIO_INTERRUPT_COUNT; i++) {
			/* GPIO interrupt was enabled and fired */
			if (omap_mdm_ctrl_data.gpios[i].irq_enabled & mask
			    && omap_mdm_ctrl_data.gpios[i].irq_fired & mask) {
				omap_mdm_ctrl_data.gpios[i].irq_fired &= ~mask;
				gpioData[0] |= 1 << i;
			}
		}

		/* Restrict the copy to the amount specified */
		if (count > GPIO_BYTE_COUNT)
			count = GPIO_BYTE_COUNT;

		/* Copy the interrupt information to the buffer */
		if (copy_to_user(buff, gpioData, count)) {
			ret = -EFAULT;
			goto out;
		}
		*f_pos += count;
		ret = count;
	}

out:
	/* Release the semaphore and return */
	up(&omap_mdm_ctrl_data.sem);
	return ret;
}

/* Modify the client information for enabling / disabling an irq */
void set_interrupt(struct gpioinfo *gpio, struct clientinfo *cinfo, int enable)
{
	int mask = cinfo->mask;
	if (enable == 0) {
		gpio->irq_enabled &= ~mask;
		gpio->irq_fired &= ~mask;
		gpio->irq_level_rising &= ~mask;
		gpio->irq_level_falling &= ~mask;
	} else {
		gpio->irq_enabled |= mask;
		if (enable == OMAP_MDM_CTRL_IRQ_RISING)
			gpio->irq_level_rising |= mask;
		else if (enable == OMAP_MDM_CTRL_IRQ_FALLING)
			gpio->irq_level_falling |= mask;
	}
}

/* Performs the appropriate action for the specified IOCTL command,
 * returning a failure code only if the IOCTL command was not recognized */
static int omap_mdm_ctrl_ioctl(struct inode *inode, struct file *filp,
			       unsigned int cmd, unsigned long data)
{
	int ret = -EINVAL;
	int *dataParam = (int *)data;
	int highEnable = -1;
	int intParam = -1;
	struct clientinfo *cinfo = (struct clientinfo *)filp->private_data;

	if (!cinfo) {
		pr_info("%s: File pointer invalid.\n", __func__);
		return -EBADF;
	}

	switch (cmd) {
	/* Command handling to read BP GPIO information */
	case OMAP_MDM_CTRL_IOCTL_GET_BP_READY_AP:
		ret =
		    put_user(gpio_get_value
			     (omap_mdm_ctrl_data.gpios[BP_READY_AP].gpio),
			     dataParam);
		break;
	case OMAP_MDM_CTRL_IOCTL_GET_BP_READY2_AP:
		ret =
		    put_user(gpio_get_value
			     (omap_mdm_ctrl_data.gpios[BP_READY2_AP].gpio),
			     dataParam);
		break;
	case OMAP_MDM_CTRL_IOCTL_GET_BP_RESOUT:
		ret =
		    put_user(gpio_get_value
			     (omap_mdm_ctrl_data.gpios[BP_RESOUT].gpio),
			     dataParam);
		break;

	/* Command handling to set BP GPIOs */
	case OMAP_MDM_CTRL_IOCTL_SET_BP_PWRON:
		ret = get_user(highEnable, dataParam);
		if (ret == 0)
			gpio_set_value(omap_mdm_ctrl_data.gpios[BP_PWRON].gpio,
				       highEnable);
		break;
	case OMAP_MDM_CTRL_IOCTL_SET_AP_TO_BP_PSHOLD:
		ret = get_user(highEnable, dataParam);
		if (ret == 0) {

			gpio_set_value(omap_mdm_ctrl_data.
				       gpios[AP_TO_BP_PSHOLD].gpio, highEnable);
		}
		break;
	case OMAP_MDM_CTRL_IOCTL_SET_AP_TO_BP_FLASH_EN:
		ret = get_user(highEnable, dataParam);
		if (ret == 0) {
			gpio_set_value(omap_mdm_ctrl_data.
				       gpios[AP_TO_BP_FLASH_EN].gpio,
				       highEnable);
		}
		break;

	case OMAP_MDM_CTRL_IOCTL_SET_INT_BP_READY_AP:
		ret = get_user(intParam, dataParam);
		if (ret == 0) {
			set_interrupt(&omap_mdm_ctrl_data.gpios[BP_READY_AP],
					cinfo, intParam);
		}
		break;
	case OMAP_MDM_CTRL_IOCTL_SET_INT_BP_READY2_AP:
		ret = get_user(intParam, dataParam);
		if (ret == 0) {
			set_interrupt(&omap_mdm_ctrl_data.gpios[BP_READY2_AP],
					cinfo, intParam);
		}
		break;
	case OMAP_MDM_CTRL_IOCTL_SET_INT_BP_RESOUT:
		ret = get_user(intParam, dataParam);
		if (ret == 0) {
			set_interrupt(&omap_mdm_ctrl_data.gpios[BP_RESOUT],
					cinfo, intParam);
		}
		break;

	default:
		pr_err("%s: Unknown IO control command received. (%d)\n",
		       __func__, cmd);
		break;
	}

	return ret;
}

/* Character device file operation function pointers */
static const struct file_operations omap_mdm_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = omap_mdm_ctrl_open,
	.release = omap_mdm_ctrl_release,
	.poll = omap_mdm_ctrl_poll,
	.read = omap_mdm_ctrl_read,
	.ioctl = omap_mdm_ctrl_ioctl,
};

static struct miscdevice omap_mdm_ctrl_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = OMAP_MDM_CTRL_MODULE_NAME,
	.fops = &omap_mdm_ctrl_fops,
};

/* Initializes the driver and creates the bp driver interface device
 * (/dev/omap_mdm_ctrl). */
static int __devinit omap_mdm_ctrl_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct omap_mdm_ctrl_platform_data *pdata = pdev->dev.platform_data;

	/* Initialize internal structures */
	init_MUTEX(&omap_mdm_ctrl_data.sem);

	/* Initialize the client information */
	memset(&omap_mdm_ctrl_data.clients, 0,
		sizeof(omap_mdm_ctrl_data.clients));
	for (i = 0; i < MAX_CLIENTS; i++) {
		init_waitqueue_head(&omap_mdm_ctrl_data.clients[i].wq);
		omap_mdm_ctrl_data.clients[i].mask = 1 << i;
	}

	/* Init working queue */
	omap_mdm_ctrl_data.working_queue =
	    create_singlethread_workqueue("omap_mdm_ctrl_wq");
	if (!omap_mdm_ctrl_data.working_queue) {
		pr_err("%s: Cannot create work queue\n", __func__);
		ret = -ENOMEM;
		goto err_destroy_wq;
	}

	/* Initialize the GPIOs */
	memset(&omap_mdm_ctrl_data.gpios, 0, sizeof(omap_mdm_ctrl_data.gpios));
	omap_mdm_ctrl_data.gpios[BP_READY_AP].gpio = pdata->bp_ready_ap_gpio;
	INIT_WORK(&omap_mdm_ctrl_data.gpios[BP_READY_AP].work, irq_work);
	omap_mdm_ctrl_data.gpios[BP_READY2_AP].gpio = pdata->bp_ready2_ap_gpio;
	INIT_WORK(&omap_mdm_ctrl_data.gpios[BP_READY2_AP].work, irq_work);
	omap_mdm_ctrl_data.gpios[BP_RESOUT].gpio = pdata->bp_resout_gpio;
	INIT_WORK(&omap_mdm_ctrl_data.gpios[BP_RESOUT].work, irq_work);
	omap_mdm_ctrl_data.gpios[BP_PWRON].gpio = pdata->bp_pwron_gpio;
	INIT_WORK(&omap_mdm_ctrl_data.gpios[BP_PWRON].work, irq_work);
	omap_mdm_ctrl_data.gpios[AP_TO_BP_PSHOLD].gpio =
	    pdata->ap_to_bp_pshold_gpio;
	INIT_WORK(&omap_mdm_ctrl_data.gpios[AP_TO_BP_PSHOLD].work, irq_work);
	omap_mdm_ctrl_data.gpios[AP_TO_BP_FLASH_EN].gpio =
	    pdata->ap_to_bp_flash_en_gpio;
	INIT_WORK(&omap_mdm_ctrl_data.gpios[AP_TO_BP_FLASH_EN].work, irq_work);

	/* Setup all interrupts */
	omap_mdm_ctrl_data.gpios[BP_RESOUT].irq =
	    gpio_to_irq(omap_mdm_ctrl_data.gpios[BP_RESOUT].gpio);

	ret =
	    request_irq(omap_mdm_ctrl_data.gpios[BP_RESOUT].irq, irq_handler,
			IRQF_DISABLED | OMAP_MDM_CTRL_IRQ_RISING |
			OMAP_MDM_CTRL_IRQ_FALLING, OMAP_MDM_CTRL_MODULE_NAME,
			&omap_mdm_ctrl_data.gpios[BP_RESOUT]);
	enable_irq_wake(omap_mdm_ctrl_data.gpios[BP_RESOUT].irq);
	if (ret < 0) {
		pr_err("%s: Can not reqeust IRQ (%d) from kernel!\n", __func__,
		       omap_mdm_ctrl_data.gpios[BP_RESOUT].irq);
		goto err_clear_gpio;
	} else {
		omap_mdm_ctrl_data.gpios[BP_RESOUT].irq_enabled = 1;
	}

	ret = misc_register(&omap_mdm_ctrl_misc_device);
	if (ret < 0) {
		pr_err("%s: omap_mdm_ctrl register failed\n", __func__);
		goto err_unregister;
	}

	pr_info("%s: Loaded omap_mdm_ctrl device driver\n", __func__);
	return ret;

	/* Handling for error conditions */
err_unregister:
	misc_deregister(&omap_mdm_ctrl_misc_device);
err_clear_gpio:
	clear_gpio_data();
err_destroy_wq:
	destroy_workqueue(omap_mdm_ctrl_data.working_queue);
	return ret;
}

/* Destroys the bp driver interface device and clears up any pending IRQs */
static int __devexit omap_mdm_ctrl_remove(struct platform_device *pdev)
{
	misc_deregister(&omap_mdm_ctrl_misc_device);

	/* Free any GPIO IRQs requested but not yet freed */
	clear_gpio_data();

	destroy_workqueue(omap_mdm_ctrl_data.working_queue);

	pr_info("%s: Unloaded omap_mdm_ctrl device driver\n", __func__);
	return 0;
}

/* Initiate modem power down */
static void __devexit omap_mdm_ctrl_shutdown(struct platform_device *pdev)
{
	int i;
	int pd_failure = 1;

	/* Check to see if the modem is already powered down */
	if (!gpio_get_value(omap_mdm_ctrl_data.gpios[BP_RESOUT].gpio)) {
		pr_info("%s: Modem already powered down.\n", __func__);
		return;
	}

	/* Disable ability to notify clients of bp reset activity */
	disable_irq(omap_mdm_ctrl_data.gpios[BP_RESOUT].irq);

	pr_info("%s: Initiate modem power down...\n", __func__);
	/* Press modem Power Button */
	gpio_set_value(omap_mdm_ctrl_data.gpios[BP_PWRON].gpio, 1);
	msleep(100);
	gpio_set_value(omap_mdm_ctrl_data.gpios[BP_PWRON].gpio, 0);
	/* Wait up to 5 seconds for the modem to properly power down */
	for (i = 0; i < 10; i++) {
		if (!gpio_get_value(omap_mdm_ctrl_data.gpios[BP_RESOUT].gpio)) {
			pr_info("%s: Modem power down success.\n", __func__);
			pd_failure = 0;
			break;
		} else
			msleep(500);
	}

	if (pd_failure) {
		/* Pull power from the modem */
		pr_info("%s: Modem pd failure.  Pull power.\n", __func__);
		gpio_set_value(
			omap_mdm_ctrl_data.gpios[AP_TO_BP_PSHOLD].gpio, 1);
		msleep(5);
		gpio_set_value(
			omap_mdm_ctrl_data.gpios[AP_TO_BP_PSHOLD].gpio, 0);
	}

	/* Re-enable ability to notify clients of bp reset activity */
	enable_irq(omap_mdm_ctrl_data.gpios[BP_RESOUT].irq);
}

static struct platform_driver omap_mdm_ctrl_driver = {
	.probe = omap_mdm_ctrl_probe,
	.remove = __devexit_p(omap_mdm_ctrl_remove),
	.shutdown = __devexit_p(omap_mdm_ctrl_shutdown),
	.driver = {
		   .name = OMAP_MDM_CTRL_MODULE_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init omap_mdm_ctrl_os_init(void)
{
	return platform_driver_register(&omap_mdm_ctrl_driver);
}

static void __exit omap_mdm_ctrl_os_exit(void)
{
	platform_driver_unregister(&omap_mdm_ctrl_driver);
}

module_init(omap_mdm_ctrl_os_init);
module_exit(omap_mdm_ctrl_os_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("OMAP Modem Control Driver");
MODULE_VERSION("1.1.3");
MODULE_LICENSE("GPL");
