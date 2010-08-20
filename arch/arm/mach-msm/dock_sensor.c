/*
 * Copyright (C) 2009-2010 Motorola, Inc.
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
 * Motorola 2010-JAN-20 - Initial Creation
 */

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <mach/board.h>
#include <asm-arm/mach-types.h>
#include "board-mot.h"

#define DRIVER_NAME "dock"


#define DOCK_DETECT_DEBOUNCE_TIMER  3000000    

static void dock_work_func(struct work_struct *work);
static irqreturn_t dock_irq_handler(int irq, void *dev_id);
static enum hrtimer_restart dock_detect_event_timer_func(struct hrtimer *data);
    
struct dock_dev {
    struct switch_dev sdev;
    uint8_t           state;  /*DOCK STATE, on/off*/
    unsigned int      irq;    
    ktime_t           debounce_time;  
    struct hrtimer    timer;
    struct work_struct work;
};

static struct dock_dev *the_dock;



static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
    switch (switch_get_state(sdev)) {
        case 0:
            return sprintf(buf, "OFF\n");
        case 1:
            return sprintf(buf, "ON\n");
    }
    return -EINVAL;
}

static int __init dock_probe(struct platform_device *pdev)
{
    int     rc = 0;
    unsigned long request_flags =  IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING;
    unsigned long irq_flags;

    printk(KERN_INFO "Dock sensor:Register dock sensor driver\n");

    the_dock = kzalloc(sizeof(*the_dock), GFP_KERNEL);
    if (the_dock == NULL)
        return -ENOMEM;
    the_dock->debounce_time = ktime_set(0, DOCK_DETECT_DEBOUNCE_TIMER);
    the_dock->sdev.name = DRIVER_NAME;
    the_dock->sdev.print_name = print_switch_name;
    the_dock->state =  0;
    INIT_WORK(&the_dock->work, dock_work_func);

    rc = switch_dev_register(&the_dock->sdev);

    if (rc < 0)
        goto err_switch_dev_register;
    rc = gpio_request(DOCK_DETECT_INT, "dock");
    if (rc < 0)
        goto err_request_detect_gpio;
    rc  = gpio_direction_input(DOCK_DETECT_INT);
    if (rc < 0)
        goto err_set_detect_gpio;
    the_dock->irq = gpio_to_irq(DOCK_DETECT_INT);
    if (the_dock->irq < 0){
        rc = the_dock->irq;
        goto err_get_dock_detect_irq_num_failed;
    }
    hrtimer_init(&the_dock->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    the_dock->timer.function = dock_detect_event_timer_func;

    rc = request_irq(the_dock->irq, dock_irq_handler,request_flags, "dock_detect", the_dock);
    if (rc < 0)
        goto err_request_detect_irq;
    rc = set_irq_wake(the_dock->irq, 1);
    if (rc < 0)
        goto err_request_detect_irq;
    return 0;
err_request_detect_irq:
err_set_detect_gpio:    
err_get_dock_detect_irq_num_failed:
    gpio_free(DOCK_DETECT_INT);
err_request_detect_gpio:
    switch_dev_unregister(&the_dock->sdev);
err_switch_dev_register:
    kfree(the_dock);
    printk(KERN_ERR "DOCK: Failed to register driver\n");
    return rc;
}

static void dock_work_func(struct work_struct *work)
{
    unsigned long irq_flags;
    struct dock_dev *dock = container_of(work, struct dock_dev, work);
    int state = 1-gpio_get_value(DOCK_DETECT_INT);
    
    printk(KERN_INFO"DOCK state is %d\n",state);
    if (state != dock->state){
        dock->state = state;
        switch_set_state(&dock->sdev, state);
    }
    local_irq_save(irq_flags);
    enable_irq(dock->irq);
    local_irq_restore(irq_flags);
}

static enum hrtimer_restart dock_detect_event_timer_func(struct hrtimer *data)
{
    struct dock_dev *dock = container_of(data,struct dock_dev, timer);    
    schedule_work(&dock->work);
    return HRTIMER_NORESTART;
}


static irqreturn_t dock_irq_handler(int irq, void *dev_id)
{
    unsigned long irq_flags;
    struct dock_dev *dock = (struct dock_dev *)dev_id;
            
    local_irq_save(irq_flags);  
    disable_irq(dock->irq);
    local_irq_restore(irq_flags);
    hrtimer_start(&dock->timer, dock->debounce_time, HRTIMER_MODE_REL); 
    return IRQ_HANDLED;
}

static int dock_remove(struct platform_device *pdev)
{
    gpio_free(DOCK_DETECT_INT);
    if (the_dock) {
       free_irq(the_dock->irq, 0);
       switch_dev_unregister(&the_dock->sdev);
       kfree(the_dock);
   }
}

static struct platform_device dock_device = {
    .name       = DRIVER_NAME,
};


static struct platform_driver dock_driver = {
    .probe = dock_probe,
    .remove = dock_remove,
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
    },
};

static int __init dock_init(void)
{
    int ret = 0;

    ret = platform_driver_register(&dock_driver);
    if (ret)
    {
        printk(KERN_ERR "DOCK SENSOR:platform_driver_register error %d \n",ret);
        return ret;
    }
    return platform_device_register(&dock_device);
}

static void __exit dock_exit(void)
{
    platform_device_unregister(&dock_device);
    platform_driver_unregister(&dock_driver);
}


module_init(dock_init);
module_exit(dock_exit);

MODULE_DESCRIPTION("dock sensor  driver");
MODULE_AUTHOR("He Yi <yihe@motorola.com>");
MODULE_LICENSE("GPL");

