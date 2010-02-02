/* Source for:
 * Cypress CY8CTMA300 Prototype touchscreen driver.
 * drivers/input/touchscreen/cy8ctma300.c
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Cypress Semiconductor at www.cypress.com
 *
 * History:
 *			(C) 2010 Cypress - Update for GPL distribution
 *			(C) 2009 Cypress - Assume maintenance ownership
 *			(C) 2009 Enea - Original prototype
 *
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#include "cy8ctma300.h"


MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CY8CTMA300 touch driver");
MODULE_AUTHOR("Cypress");


struct cy8 {
  struct i2c_client *client;
  struct input_dev *input;
  struct work_struct work;
  struct timer_list timer;
  struct mutex mutex;
  char phys[32];
  struct cy8_platform_data *platform_data;
  u8 prev_touch;
};


/* Structure used to read X,Y,Z, # of fingers and large area touch status
 * from device
 */
struct xydata_t {
  u8 stat;
  u16 x __attribute__ ((packed));
  u16 y __attribute__ ((packed));
  u8 z;
};



/* ****************************************************************************
 * Prototypes for static functions
 * ************************************************************************** */
static irqreturn_t cy8_irq(int irq, void *handle);
static void cy8_xy_worker(struct work_struct *work);
static int __devinit cy8_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit cy8_remove(struct i2c_client *client);
static int cy8_resume(struct i2c_client *client);
static int cy8_suspend(struct i2c_client *client, pm_message_t message);



/* *************************************************************************
 * Static variables below
 * ************************************************************************* */
static const struct i2c_device_id cy8_id[] = {
  { "cy8ctma300", 0 },  { }
};


MODULE_DEVICE_TABLE(i2c, cy8_id);

static struct i2c_driver cy8_driver = {
   .driver = {
      .name = "cy8ctma300",
      .owner = THIS_MODULE,
   },
   .probe = cy8_probe,
   .remove = __devexit_p(cy8_remove),
   .suspend = cy8_suspend,
   .resume = cy8_resume,
   .id_table = cy8_id,
};

/* ************************************************************************
 * The cy8_xy_worker function reads the XY coordinates and sends them to
 * the input layer
 * ************************************************************************ */
void cy8_xy_worker(struct work_struct *work)
{
  struct xydata_t xy_data;
  struct cy8 *ts = container_of(work,struct cy8,work);
  u16 tmp;
  s32 retval;

  retval = i2c_smbus_read_i2c_block_data(ts->client,0x02,
					 sizeof(struct xydata_t),
					 (u8 *)&xy_data);
  if (retval< 0) {
    goto exit_xy_worker;
  }

  if (FLIP_DATA(ts->platform_data->flags)) { /* We need to swap X & Y coordinates */
    tmp = xy_data.x;
    xy_data.x = xy_data.y;
    xy_data.y = tmp;
  }

  xy_data.x = be16_to_cpu(xy_data.x);
  xy_data.y = be16_to_cpu(xy_data.y);

  /* We only report data when there is exactly one finger touch to report and no
   * Large area has been detected.
   *
   * TODO:
   * 1. Add multi touch functionality
     * 2. Add press/release events if necessary
     */

  if ((GET_NUM_FINGERS(xy_data.stat) == 1)) {
    printk(KERN_ALERT "\n# of fingers : %d\n",GET_NUM_FINGERS(xy_data.stat));
    printk(KERN_ALERT "Large area   : %s\n",(IS_LARGE_AREA(xy_data.stat)? "yes": "no"));
    printk(KERN_ALERT "X            : %x\n",xy_data.x);
    printk(KERN_ALERT "Y            : %x\n",xy_data.y);
    printk(KERN_ALERT "prev_touch   : %d\n", ts->prev_touch);
    ts->prev_touch = 1;
    input_report_abs(ts->input, ABS_X, xy_data.x);
    input_report_abs(ts->input, ABS_Y, xy_data.y);
    input_report_abs(ts->input, ABS_PRESSURE, xy_data.z);
    input_report_abs(ts->input, ABS_TOOL_WIDTH,10);
    input_report_key(ts->input, BTN_TOUCH, 1);
    input_sync(ts->input);
  } else if ((GET_NUM_FINGERS(xy_data.stat) == 0) && (ts->prev_touch == 1)) {
    printk(KERN_ALERT "\n# of fingers : %d\n",GET_NUM_FINGERS(xy_data.stat));
    printk(KERN_ALERT "Large area   : %s\n",(IS_LARGE_AREA(xy_data.stat)? "yes": "no"));
    printk(KERN_ALERT "X            : %x\n",xy_data.x);
    printk(KERN_ALERT "Y            : %x\n",xy_data.y);
    printk(KERN_ALERT "prev_touch   : %d\n", ts->prev_touch);
    ts->prev_touch = 0;
    input_report_key(ts->input, BTN_TOUCH, 0);
    input_sync(ts->input);
  }

  if (IS_LARGE_AREA(xy_data.stat)==1) {
    input_report_abs(ts->input, ABS_TOOL_WIDTH,255);
    input_sync(ts->input);
  }


 exit_xy_worker:
  if (ts->client->irq == 0) {
    /* We need to restart the timer as we do not use IRQs */
    mod_timer(&ts->timer, jiffies+ TOUCHSCREEN_TIMEOUT);
  }

  return;
}



/* ************************************************************************
 * Timer function used as dummy interrupt driver
 * ************************************************************************ */
static void cy8_timer(unsigned long handle)
{
  struct cy8 *ts = (struct cy8 *) handle;
   schedule_work(&ts->work);
   return;
}



/* ************************************************************************
 * ISR function. This function is general, initialized in drivers init
 * function
 * ************************************************************************ */
static irqreturn_t cy8_irq(int irq, void *handle)
{
  struct cy8 *ts = (struct cy8 *) handle;
  printk (KERN_ALERT "CY8ctma300: Got IRQ\n");
  schedule_work(&ts->work);
  return IRQ_HANDLED;

}


/* cy8_initialize: Bus independent initialization. This function takes
 * care of the following tasks:
 * 1. Creating an input device and registering the device with the
 *    input layer
 * 2. Setting up IRQs  (TBD)
 * 3. Setting up SysFS (TBD)
 * 4. Initializing CY8CTMA300
 * 5. Starting any timers/Work queues.  */

static int cy8_initialize(struct i2c_client *client, struct cy8 *ts)
{

   struct input_dev *input_device;
   int error=0;
   int retval=0;

   /* Create the input device and register it. */
   input_device = input_allocate_device();
   if (!input_device) {
     error = -ENOMEM;
     goto error_free_device;
   }

   ts->input = input_device;
   input_device->name = "CY8CTMA300 Touchscreen";
   input_device->phys = ts->phys;
   input_device->dev.parent = &client->dev;

   set_bit(EV_SYN, input_device->evbit);
   set_bit(ABS_X, input_device->absbit);
   set_bit(ABS_Y, input_device->absbit);
   set_bit(ABS_PRESSURE, input_device->absbit);
   set_bit(BTN_TOUCH, input_device->keybit);
   set_bit(EV_KEY, input_device->evbit);
   set_bit(EV_ABS, input_device->evbit);

   input_set_abs_params(input_device, ABS_X, 0, ts->platform_data->maxx, 0, 0);
   input_set_abs_params(input_device, ABS_Y, 0, ts->platform_data->maxy, 0, 0);
   input_set_abs_params(input_device, ABS_TOOL_WIDTH, 0, 255, 0 ,0);
   input_set_abs_params(input_device, ABS_PRESSURE, 0, 255, 0, 0);

   /* Report MAXX MAXY here */

   error = input_register_device(input_device);
   if (error) {
     printk(KERN_ERR "cy8ctma300.c: Failed to register input device\n");
     retval = error;
     goto error_free_device;
   }

   /* Prepare our worker structure prior to setting up the timer/ISR */
   INIT_WORK(&ts->work,cy8_xy_worker);

   /* Power on the chip and make sure that I/Os are set as specified
    * in the platform */
   if (ts->platform_data->power_on != NULL) {
     ts->platform_data->power_on();
   }

   /* Interrupt setup: if client->irq is zero we use the polling
    * version instead */

   if (client->irq !=0) {
     printk(KERN_ALERT"Using IRQ %d\n",client->irq);
     error = request_irq (client->irq,cy8_irq,IRQF_TRIGGER_FALLING,
			  client->dev.driver->name,ts);
     if(error) {
       printk(KERN_ALERT "error: could not request irq\n");
	 retval = error;
	 goto error_free_irq;
      }
    }

   else {
   printk(KERN_ALERT "Setting up timer\n");
   setup_timer(&ts->timer, cy8_timer, (unsigned long) ts);
   mod_timer(&ts->timer,jiffies + TOUCHSCREEN_TIMEOUT);
   }


/*    Initialize the chip - nothing needed in initial version */

   goto success;

   /* error_free_sysfs: */
   printk(KERN_ALERT "error: should remove sysfs group here\n");
   /*sysfs_remove_group(&bus->dev.kobj, attr_group);*/

 error_free_irq:
   printk(KERN_ERR "Error: Failed to register IRQ handler\n");
   free_irq(client->irq,ts);

 error_free_device:
   input_free_device(input_device);

 success:
   return retval;
}


/* Function to manage power-on resume (still to be implemented in full) */
static int cy8_resume(struct i2c_client *client)
{
   /* dev_get_drvdata + device enable */
   printk(KERN_ALERT "cy8_resume\n");
   return 0;
}


/* Function to manage power-off suspend (still to be implemented in full) */
static int cy8_suspend(struct i2c_client *client, pm_message_t message)
{
  /* Dummy function for now. We need to do more... */
    printk(KERN_ALERT "cy8_suspend\n");
   return 0;
}



/* Are you sure that your driver's ids match one that is present in the
   system at the time?  That is what is needed for the probe to be called. */
static int __devinit cy8_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
   struct cy8 *ts;
   int error;

   printk(KERN_ALERT "Entering: %s\n",__FUNCTION__);

   ts = kzalloc (sizeof(struct cy8),GFP_KERNEL);
   if (!ts) {
      return -ENOMEM;
   }

   /* register driver_data */
   ts->client = client;
   ts->platform_data = client->dev.platform_data;
   i2c_set_clientdata(client,ts);

   /* bus-independent initialization of cy8ctma300 below */
   error = cy8_initialize(client, ts);
   if (error) {
     i2c_del_driver(&cy8_driver);
   }

   return 0;

}


/* registered in driver struct */
static int __devexit cy8_remove(struct i2c_client *client)
{
   struct cy8 *ts;
   int err;
   printk(KERN_ALERT "Entering: %s\n",__FUNCTION__);
   /* clientdata registered on probe */
   ts = i2c_get_clientdata(client);

   /* Start cleaning up by removing any delayed work and the timer */
   if (cancel_delayed_work((struct delayed_work *)&ts->work)<0) {
     printk(KERN_ALERT "error: could not remove work from workqueue");
   }
   err = del_timer(&ts->timer);
   if (err < 0) {
      printk(KERN_ALERT "error: failed to delete timer\n");
   }
   /* add bus independent remove */
   kfree(ts);
   printk(KERN_ALERT "Leaving: %s\n",__FUNCTION__);
   return 0;
}


static int cy8_init(void)
{
   int ret;

   printk(KERN_ALERT "CY8CTMA300 Driver (Built %s @ %s)\n",__DATE__,__TIME__);
   ret = i2c_add_driver(&cy8_driver);
   return ret;
}

static void cy8_exit(void)
{
   printk(KERN_ALERT "Goodbye cruel world\n");
   return i2c_del_driver(&cy8_driver);
}

module_init(cy8_init);
module_exit(cy8_exit);
