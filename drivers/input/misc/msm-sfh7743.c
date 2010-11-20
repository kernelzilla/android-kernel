/*
 * Copyright (C) 2008-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 * Motorola 2008-Aug-11 - Initial Creation
 * Motorola 2009-Jan-29 - File cleanup, GPIO updates, non-error print statement
 *    removal, enhanced error handling
 * Motorola 2009-Feb-03 - Added IOCTL support
 */

/*!
 * @file sfh7743_main.c
 *
 * @brief Low-level OSRAM sfh7743 driver
 *
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/ioctl.h> /* For ioctl macros */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/hwmon-sysfs.h>
#ifndef CONFIG_MACH_MOT
#include <linux/device.h> /* For vreg.h */

#include <mach/vreg.h>
#endif

#include <asm/gpio.h>
#include <asm/uaccess.h> /* For copy_to_user function */


#include <linux/sfh7743.h>


/******************************************************************************
 * Constants
 ******************************************************************************/

/* No-error condition */
#define E_OK  0

/* Proximity near and far indicate no-presence and presence, respectively */
/* unit = millimeter */
#define SFH7743_PROXIMITY_NEAR	30	/* prox close threshold is 22-70mm */
#define SFH7743_PROXIMITY_FAR	2147483647	/* (2^31)-1 */

#define SFH7743_MODULE_NAME     "proximity"

/******************************************************************************
 * Type Definitions
 ******************************************************************************/
enum sfh7743_cmd
{
	SFH7743_CMD_CURRENT = 0,
	SFH7743_CMD_MODE,
	SFH7743_CMD_MAX
};
typedef struct sfh7743_data
{
   int gpio_en;   /* Proximity sensor enabled interrupt number */
#ifndef CONFIG_MACH_MOT
   struct vreg *vreg_en;  /* Proximity sensor vreg for enable (if gpio-en < 0) */
   char const *vreg_name;
#endif
   int gpio_intr; /* Proximity change-detect interrupt number */
   int irq;
   unsigned char state;	/* indicates weather proximity device is on(1) or off(0) */
   struct work_struct wq; /* Work queue */
   struct input_dev *idev;
} sfh7743_data_t;


/******************************************************************************
 * Global Variables
 ******************************************************************************/
/******************************************************************************
 * Local function prototypes
 ******************************************************************************/

/** See full functions for descriptions */
static int  sfh7743_fops_open (struct inode *, struct file *);
static int  sfh7743_fops_ioctl (struct inode *, struct file *, unsigned int, unsigned long);
static int  sfh7743_fops_read (struct file *, char __user *, size_t, loff_t *);
static int  sfh7743_fops_release (struct inode *, struct file *);
static int  sfh7743_fops_write (struct file *, const char __user *, size_t, loff_t *);

static int  sfh7743_config_int (struct platform_device *);
static int	sfh7743_probe(struct platform_device *);
static int	sfh7743_remove(struct platform_device *);
static int  sfh7743_register (void);
static void sfh7743_exit (void);
static irqreturn_t sfh7743_irq_handler (int, void *);
static void sfh7743_irq_bottom_half (struct work_struct *);
static void sfh7743_get_distance (long *);
static void sfh7743_post_data (long);
static int  sfh7743_init (void);
static void sfh7743_log (char *, ...);
static void sfh7743_unregister (void);
static void sfh7743_enable (unsigned char);
static ssize_t sfh7743_get_mode(struct device *, struct device_attribute *, char *);
static ssize_t sfh7743_set_mode(struct device *, struct device_attribute *, const char *, size_t );

static SENSOR_DEVICE_ATTR(proximity,S_IRUGO | S_IWUSR,
		sfh7743_get_mode, sfh7743_set_mode, SFH7743_CMD_MODE);

static struct attribute *sfh7743_attributes[] = {
	&sensor_dev_attr_proximity.dev_attr.attr,
	NULL
};
struct	attribute_group	sfh7743_attr_group = {
	.attrs = sfh7743_attributes,
};
#ifdef CONFIG_MACH_MOT
	extern unsigned mot_hw_rev;
#endif

/******************************************************************************
 * Local Variables
 ******************************************************************************/

/** \var sfh7743_fops
 *  \brief Driver file operations interface
 */
static struct file_operations sfh7743_fops =
{
   .owner   = THIS_MODULE,
   .open    = sfh7743_fops_open,
   .ioctl   = sfh7743_fops_ioctl,
   .read    = sfh7743_fops_read,
   .release = sfh7743_fops_release,
   .write   = sfh7743_fops_write,
};

/** \var sfh7743_miscdev
 *  \brief Driver miscellaneous device interface
 */
static struct miscdevice sfh7743_miscdev =
{
   .fops  = &sfh7743_fops,
   .minor = MISC_DYNAMIC_MINOR,
   .name  = SFH7743_MODULE_NAME,
};

/** \var sfh7743_platform_device
 *  \brief Driver platform device interface
 */
static struct platform_driver sfh7743_driver = {
	.probe		= sfh7743_probe,
	.remove		= sfh7743_remove,
	.driver		= {
		.name	= "sfh7743",
		.owner	= THIS_MODULE,
	},
};
/** \var sfh7743_info
 *  \brief Structure that contains SFH7743 driver information
 */
static sfh7743_data_t sfh7743_info;

/******************************************************************************
 * Local Functions
 ******************************************************************************/

/** \fn static int sfh7743_fops_open (struct inode *inode, struct file *filp)
 *  \brief System response point to fop open
 *  \return Error code where zero indicates no-error
 */
static int sfh7743_fops_open (struct inode *inode, struct file *filp)
{
   /* Currently no functionality is provided here */
   return E_OK;
} /* End sfh7743_fops_open */

/** \fn static int sfh7743_fops_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
 *  \brief System response point to fop ioctl
 *  \return Error code where zero indicates no-error, or value depending on ioctl call
 */
static int sfh7743_fops_ioctl (struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
   /** \var error
    *  \brief Return error code, initialized to no error
    */
   int error = E_OK;
   /* Temporary store */
   long tmp_var;
	uint8_t	enable;
	void __user *argp = (void __user *)arg;

   /* Process the parameter command */
   switch (cmd)
   {
      case SFH7743_IOCTL_GET_PRESENCE:
        /* Get/return distance */
		/* if devise is disable, return 0 */
		if ( sfh7743_info.state )
		{
			sfh7743_log(KERN_INFO "sfh7743_fops_ioctl: device is enabled\n");
			sfh7743_get_distance(&tmp_var);
			sfh7743_log(KERN_INFO "sfh7743_fops_ioctl: Distance value: %d\n",tmp_var);
		}
		else
		{
			sfh7743_log(KERN_INFO "sfh7743_fops_ioctl: device is disabled\n");
			tmp_var = 0;
		}
		error = tmp_var;
/*
		error = copy_to_user(&arg, &tmp_var,sizeof(tmp_var));
		if ( error != 0 )
		{
			sfh7743_log(KERN_INFO "sfh7743_fops_ioctl: Could not copy %d bytes to user space.\n",error);
		}
*/
		break;
      case SFH7743_IOCTL_PUSH_PRESENCE:
         /* Get/push-to-input distance */
         sfh7743_get_distance(&tmp_var);
         sfh7743_post_data(tmp_var);
         break;
      case SFH7743_IOCTL_SET_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			sfh7743_enable(true);
		else
			sfh7743_enable(false);
         break;
      case SFH7743_IOCTL_GET_ENABLE:
#ifdef CONFIG_MACH_MOT
		enable = gpio_get_value(sfh7743_info.gpio_en);
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;
#else
         error = sfh7743_info.state;
#endif
		   break;
      default:
         sfh7743_log(KERN_ERR "sfh7743: invalid ioctl cmd supplied\n");
         /* Indicate inappropriate ioctl for device */
         error = -ENOTTY;
         break;
   } /* End switch (cmd) */

   return error;
} /* End sfh7743_fops_ioctl */

/** \fn static int sfh7743_fops_read (struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
 *  \brief System response point to fop read - reports that the device exists
 *  \return Error code where zero indicates no-error
 */
static int sfh7743_fops_read (struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
   /** \var sfh7743_str
    *  \brief Initialized with identifier string
    */
   char * sfh7743_str = "I am the SFH7743!\n";
   int len;
   /** \var len
    *  \brief Initialized with length of identifier string (w/o null byte)
    */
#ifdef CONFIG_MACH_MOT
	int status = gpio_get_value(sfh7743_info.gpio_en);
	len = snprintf(sfh7743_str, 3, "%d\n", status);
	sfh7743_log(KERN_INFO "sfh7743_fops_read: Mode: %d\n", status);
#else
	len = snprintf(sfh7743_str,3,"%d\n",sfh7743_info.state);
	sfh7743_log(KERN_INFO "sfh7743_fops_read: Mode: %d\n", sfh7743_info.state);
#endif

   /* Report an error if the request is not at least as long as the identifier string length */
   if (count < len)
      return -EINVAL;

   /* If the current file position is non-zero (set below, but on a previous read), we assume
    *  the identifier has been read and indicate there is no more data to be read */
   if (*f_pos != 0)
      return 0;

   /* Copy the string to the user buffer, check that the user has permission to write to the
    *  buffer, that it is mapped, etc. */
   if (copy_to_user(buf, sfh7743_str, len))
      return -EINVAL;
   
   /* Updated file position with the number of bytes in the identifier */
   *f_pos = len;
   /* Return the number of bytes in the identifier */
   return len;
} /* End sfh7743_fops_read */

/** \fn static int sfh7743_fops_release (struct inode *inode, struct file *filp)
 *  \brief System response point to fop release
 *  \return Error code where zero indicates no-error
 */
static int sfh7743_fops_release (struct inode *inode, struct file *filp)
{
   /* Currently no functionality is provided here */
   return E_OK;
} /* End sfh7743_fops_release */

/** \fn static int sfh7743_fops_write (struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
 *  \brief System response point to fop write
 *  \return Error code where zero indicates no-error
 */
static char	writeBuf[10];
static int sfh7743_fops_write (struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	if ( count > 3 )
	      return -EINVAL;
	if (copy_from_user(writeBuf, buf, count))
		return -EINVAL;
	/* If disable requested */
	if ( !(strncmp(writeBuf, "0", 1 ) ) )
	{
		sfh7743_enable(false);
	}
	/* If enable requested */
	else
	{
		sfh7743_enable(true);
	}
   return E_OK;
} /* End sfh7743_fops_write */



/** \fn static int sfh7743_config_int (void)
 *  \brief Configure sfh7743 sensor interrupt GPIOs and routine
 *  \return E_OK if successful, -1 if any failure occurred
 */
static int sfh7743_config_int (struct platform_device *pdev)
{
	struct	sfh7743_platform_data	*pdata = pdev->dev.platform_data;

   /* Initialize error condition to no-error */
   int error = E_OK;

	if ( !pdata )
		return -EBUSY;

   /* Read interrupt numbers from hardware config */
   sfh7743_info.gpio_intr = pdata->gpio_intr;
   sfh7743_info.gpio_en   = pdata->gpio_en;
#ifndef CONFIG_MACH_MOT
   sfh7743_info.vreg_name = pdata->vreg_en;
#endif

   /* Ensure proper pin numbers are returned */
#ifdef CONFIG_MACH_MOT

   if ((sfh7743_info.gpio_intr < 0) || (sfh7743_info.gpio_en < 0)) {
	sfh7743_log(KERN_ERR "sfh7743_config_int: get_gpio failure\n");
#else
   if ((sfh7743_info.gpio_intr<0) || ((sfh7743_info.gpio_en<0)&&(pdata->vreg_en==NULL)))
   {
      sfh7743_log(KERN_ERR "sfh7743_config_int: get_gpio/vreg_get failure\n");
#endif
      goto sfh7743_error;
   }

   /* Configure change interrupt */
   if (gpio_request(sfh7743_info.gpio_intr, SFH7743_MODULE_NAME) == E_OK)
   {
      /* Set gpio to input direction to allow irq signals */
      if (gpio_direction_input(sfh7743_info.gpio_intr) != E_OK)
      {
         sfh7743_log(KERN_ERR "sfh7743_config_int: gpio_int direction error\n");
         goto sfh7743_error_gpio_int_requested;
      }
   }
   else
   {
      sfh7743_log(KERN_ERR "sfh7743_config_int: gpio_request error\n");
      goto sfh7743_error;
   }


   /* Record irq value to structure.  A negative value indicates an error */
   sfh7743_info.irq = gpio_to_irq(sfh7743_info.gpio_intr);
   /* Attempt setup of interrupt request handler if proper irq returned above.
    * If either fails, indicate and exit */
   if ( (sfh7743_info.irq<0) ||
        (request_irq(sfh7743_info.irq, sfh7743_irq_handler,IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_SHARED,
                      SFH7743_MODULE_NAME, &sfh7743_info) != E_OK) )
   {
      sfh7743_log(KERN_ERR "sfh7743_config_int: irq request error\n");
      goto sfh7743_error_gpio_int_requested;
   }
#ifdef CONFIG_MACH_MOT
	set_irq_wake(sfh7743_info.irq, 1);
#else
   if (sfh7743_info.gpio_en < 0) 
   {
      sfh7743_info.vreg_en = vreg_get(0, sfh7743_info.vreg_name);
      goto sfh7743_no_cleanup;
   }
#endif

   /* Configure enable interrupt */
   if (gpio_request(sfh7743_info.gpio_en, SFH7743_MODULE_NAME) == E_OK)
   {
      /* Set gpio to output direction to allow irq signals */
      if (gpio_direction_output(sfh7743_info.gpio_en,0) == E_OK)
      {
         /* This is a "total success" condition, so the function is exited */
         goto sfh7743_no_cleanup;
		}
      else
      {
         sfh7743_log(KERN_ERR "sfh7743_config_int: direction error\n");
         goto sfh7743_error_gpio_en_requested;
      }
   }
   else
   {
      sfh7743_log(KERN_ERR "sfh7743_config_int: gpio_request error\n");
      goto sfh7743_error_irq_requested;
   }

   /* Naturally flowing into this destruction is impossible */
sfh7743_error_gpio_en_requested:
   gpio_free (sfh7743_info.gpio_en);

sfh7743_error_irq_requested:
#ifdef CONFIG_MACH_MOT
	set_irq_wake(sfh7743_info.irq, 0);
#endif
	free_irq(sfh7743_info.irq, 0);

sfh7743_error_gpio_int_requested:
   gpio_free(sfh7743_info.gpio_intr);

sfh7743_error:
   sfh7743_info.gpio_intr = -1;
   sfh7743_info.gpio_en = -1;
#ifndef CONFIG_MACH_MOT
   sfh7743_info.vreg_name = NULL;
   sfh7743_info.vreg_en = NULL;
#endif
   sfh7743_info.irq = -1;
   error = -1;

sfh7743_no_cleanup:
   return error;
}

/** \fn void __exit sfh7743_exit (void)
 *  \brief sfh7743 driver cleanup function, called when the driver is destroyed
 *  \return void
 */
void __exit sfh7743_exit (void)
{
   /* Disable proximity sensor */
   sfh7743_enable(false);

   /* Free irq */
   if (sfh7743_info.irq != -1)
   {
#ifdef CONFIG_MACH_MOT
		set_irq_wake(sfh7743_info.irq, 0);
#endif
		free_irq(sfh7743_info.irq, 0);
   }
   /* Free GPIOs */
   if (sfh7743_info.gpio_intr != -1)
   {
      gpio_free(sfh7743_info.gpio_intr);
   }
   if (sfh7743_info.gpio_en != -1)
   {
      gpio_free(sfh7743_info.gpio_en);
   }

   /* prox_ir unregister */
   if (sfh7743_info.idev)
   {
      sfh7743_unregister();
   }
}

/** \fn int __init sfh7743_init (void)
 *  \brief sfh7743 driver initialization function
 *  \return E_OK if successful, or an error code
 */
int __init sfh7743_init (void)
{
	return platform_driver_register(&sfh7743_driver);
}

static int	sfh7743_remove(struct platform_device *pdev)
{
	input_unregister_device(sfh7743_info.idev);
	return 0;
}

static int	sfh7743_probe(struct platform_device *pdev)
{
   /* Initialize error condition to no-error */
   int error = E_OK;

   sfh7743_log(KERN_INFO "sfh7743_probe: Enter....\n");
   /* Initialize input device */
   sfh7743_info.idev = NULL;
   /* Initialize interrupt values */
   sfh7743_info.gpio_intr = -1;
   sfh7743_info.gpio_en = -1;
#ifndef CONFIG_MACH_MOT
   sfh7743_info.vreg_name = NULL;
   sfh7743_info.vreg_en = NULL;
#endif
   sfh7743_info.irq = -1;
	sfh7743_info.state = 0;

   /* Register IR prox device */
   error = sfh7743_register();
   if (error == E_OK)
   {
      /* Initialize the work queue */
      INIT_WORK (&sfh7743_info.wq, sfh7743_irq_bottom_half);

      /* GPIOs and IRQ config */
      error = sfh7743_config_int(pdev);
      if (error == E_OK)
      {
         /* Enable proximity sensor */
         sfh7743_enable(false);
      }
      else
      {
         /* Unregistered what was previously registered */
         sfh7743_unregister();
			return error;
      }
   }

   /* Report init status to log */
   if (error == E_OK) {
#ifdef CONFIG_MACH_MOT
	sfh7743_log(KERN_INFO "sfh7743_probe: initialized gpio_int[%d]"
		"irq[%d] gpio_en[%d]\n", sfh7743_info.gpio_intr,
			sfh7743_info.irq, sfh7743_info.gpio_en);
#else
	sfh7743_log(KERN_INFO "sfh7743_probe: initialized gpio_int[%d]"
		"irq[%d] gpio_en[%d] vreg_en[%s]\n", sfh7743_info.gpio_intr,
			sfh7743_info.irq, sfh7743_info.gpio_en,
				sfh7743_info.vreg_name);
#endif
   }
   else
   {
      sfh7743_log(KERN_ERR "sfh7743_probe: initialization failed: %d\n", error);
   }

	if ( (error = sysfs_create_group(&(pdev->dev.kobj), &sfh7743_attr_group) ))
	{
		sfh7743_log(KERN_INFO "sfh7743_probe: Exit....\n");
	}
   return error;
}

/*!
 * @brief Interrupt handler.
 *
 * This functions handle sfh7743 IRQ and schedules bottom half activity
 *
 * @param irq sfh7743 interrupt
 * @param dev unused
 *
 * @return IRQ_HANDLED
 */
irqreturn_t sfh7743_irq_handler (int irq, void *dev)
{
   disable_irq (irq);
   /* We're using latched interrupts */
   schedule_work (&sfh7743_info.wq);
   return IRQ_HANDLED;
}

/*!
 * @brief Implements the kernel thread for sfh7743 "bottom half" interrupt handling.
 *
 * The function implements the kernel queue for interrupt handling.
 *
 * @param work The parameter is unused
 *
 */
void sfh7743_irq_bottom_half (struct work_struct *work)
{
   long distance;
   sfh7743_get_distance(&distance);
   sfh7743_post_data(distance);
   enable_irq(sfh7743_info.irq);
}

/*!
 * @brief Sends data to upper layer
 *
 * This function is called when the sfh7743 driver needs to send 
 * data to the upper layer for processing.
 *
 * @param distance distance distance to forward
 *
 */
void sfh7743_post_data (long distance)
{
   sfh7743_log(KERN_INFO "sfh7743: presence [%x]\n", distance);
   input_report_abs(sfh7743_info.idev, ABS_DISTANCE, distance);
   input_sync(sfh7743_info.idev);
}

/*!
 * @brief enable/disable sfh7743 sensor
 *
 * This function is called to enable/disable the sfh7743 sensor
 *
 * @param en true ==> enable, false ==> disable
 *
 */
void sfh7743_enable (unsigned char en)
{
	/* If enable requested */
#ifdef CONFIG_MACH_MOT
	if (en == true) {
		gpio_set_value(sfh7743_info.gpio_en, 1);
#else
	if ((en == true) && (sfh7743_info.state == 0))
	{
      if (sfh7743_info.gpio_en < 0)
      {
		   vreg_enable(sfh7743_info.vreg_en);
      }
      else
      {
		   gpio_set_value(sfh7743_info.gpio_en, 1);
      }
#endif
		enable_irq (sfh7743_info.gpio_intr);
		sfh7743_info.state = 1;
	}
	/* If disable requested */
#ifdef CONFIG_MACH_MOT
	else {
		gpio_set_value(sfh7743_info.gpio_en, 0);
#else
	else if ((en == false) && (sfh7743_info.state == 1))
	{
      if (sfh7743_info.gpio_en < 0)
      {
		   vreg_disable(sfh7743_info.vreg_en);
      }
      else
      {
   		gpio_set_value(sfh7743_info.gpio_en, 0);
      }
#endif
		disable_irq (sfh7743_info.gpio_intr);
		sfh7743_info.state = 0;
	}
}

/*!
 * @brief retrieve proximity value
 *
 * This function is called to read the distance from sfh7743 sensor
 *
 * @param data pointer to the value to retrieve from sfh7743 sensor
 *
 */
void sfh7743_get_distance (long *data)
{
	/* If gpio indicates presence */
	if (gpio_get_value(sfh7743_info.gpio_intr))
	{
		*data = SFH7743_PROXIMITY_NEAR;
	}
	else
	{
		*data = SFH7743_PROXIMITY_FAR;
	}
}

/*!
 * @brief Register sfh7743 driver
 *
 * This function registers the sfh7743 driver as input device
 *
 */
int sfh7743_register (void)
{
   struct input_dev *input_dev;
   int err = -ENOMEM;

   /* Attempt to allocate an input device */
   input_dev = input_allocate_device();
   /* If allocation failed */
   if (!input_dev)
   {
      sfh7743_log(KERN_ERR "sfh7743: insufficient memory for input device\n");
      err = -ENOMEM;
      goto exit_input_dev_alloc_failed;
   }

   /* Set parameters */
   input_dev->name = SFH7743_MODULE_NAME;
   set_bit(EV_ABS, input_dev->evbit);
   set_bit(ABS_DISTANCE, input_dev->absbit);
   /* Copy to driver structure */
   sfh7743_info.idev = input_dev;

   /* Attempt to register input device */
   err = input_register_device(sfh7743_info.idev);
   if (err != E_OK)
   {
      sfh7743_log(KERN_ERR "sfh7743: failed to register input device\n");
      goto exit_input_register_device_failed;
   }

   /* Attempt to register the miscellaneous device */
   err = misc_register(&sfh7743_miscdev);
   if (err != E_OK)
   {
      sfh7743_log(KERN_ERR "sfh7743: failed to register misc device\n");
      goto exit_misc_register_device_failed;
   }   

   /* Registration succeeded */
   return E_OK;

exit_misc_register_device_failed:
   input_unregister_device(sfh7743_info.idev);

exit_input_register_device_failed:
   input_free_device(input_dev);
   sfh7743_info.idev = NULL;

exit_input_dev_alloc_failed:
   return err;
}

/*!
 * @brief UnRegister Prox IR device driver in Moto Input
 *
 * This function UnRegisters the IR proximity driver
 *
 */
void sfh7743_unregister (void)
{
   /* Deregister miscellaneous device */
   misc_deregister(&sfh7743_miscdev);

   /* Unregister input device, free device memory */
   input_unregister_device(sfh7743_info.idev);
   input_free_device(sfh7743_info.idev);
}

/*!
 * @brief Logs data
 *
 * This function is called to get the current mode (Enabled/Disabled) of
 * of the proximity device
 *
 */
static ssize_t sfh7743_get_mode(struct device *dev, struct device_attribute *att, char *buf)
{
#ifdef CONFIG_MACH_MOT
	int     status = gpio_get_value(sfh7743_info.gpio_en);
#endif
	ssize_t	rc;
#ifdef CONFIG_MACH_MOT
	rc = snprintf(buf, 3, "%d\n", status);
	sfh7743_log(KERN_INFO "sfh7743_get_mode: Mode: %d\n", status);
#else
	rc = snprintf(buf, 3, "%d\n", sfh7743_info.state);
	sfh7743_log(KERN_INFO "sfh7743_get_mode: Mode: %d\n", sfh7743_info.state);
#endif
	return	rc;
}

/*!
 * @brief Logs data
 *
 * This function is called to set the current mode (Enabled/Disabled) of
 * of the proximity device
 *
 */
static ssize_t sfh7743_set_mode(struct device *dev, struct device_attribute *att, const char *buf, size_t count)
{
	/* If disable requested */
	if ( !(strncmp(buf, "0", 1 ) ) )
	{
		sfh7743_enable(false);
	}
	/* If enable requested */
	else
	{
#ifdef CONFIG_MACH_MOT
#if 0
		if (machine_is_motus() &&  (mot_hw_rev < 0x2F)) {
			sfh7743_log(KERN_ERR "PROXIMITY SENSOR NOT"
				"ENABLED FOR MOTUS P2 BOARDS (< 2F)\n");
			return 0;
	}
#endif
#endif
		sfh7743_enable(true);
	}
	return 0;
}

/*!
 * @brief Logs data
 *
 * This function is called as a replacement to printk
 *
 * @param fmt Text of message to log
 */
void sfh7743_log (char *fmt, ...)
{
   va_list args;
   /* Print the rest of the data */
   va_start(args, fmt);
   vprintk(fmt, args);
   va_end(args);
}

/******************************************************************************
 * Global Functions
 ******************************************************************************/
/*
 * Module entry points
 */
module_init(sfh7743_init);
module_exit(sfh7743_exit);

MODULE_DESCRIPTION("IR proximity device driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
