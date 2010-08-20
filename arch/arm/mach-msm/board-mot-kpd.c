/* linux/arch/arm/mach-msm/board-mot-kpd.c
 *
 *  Copyright (C) 2009 Motorola, Inc.
 *  Author: Adam Zajac <Adam.Zajac@motorola.com>
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
 
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <asm/gpio.h>
#include "board-mot.h"
#include <linux/workqueue.h>
#include "proc_comm.h"

/* Defined for Pittsburgh */
#define FLIP_CLOSED        1
#define FLIP_OPENED        (!FLIP_CLOSED)
#define DISP_BL_SEL_CLI    0
#define DISP_BL_SEL_MAIN   (!DISP_BL_SEL_CLI)

struct slide_status{
	struct work_struct work;		/* work struct		*/
	int irq;			/* irq issued by device		*/
	};
static struct slide_status slide_data;

static uint32_t slide_close;

static void mot_slide_gpio_call(struct work_struct *work)
{
   if (slide_close != gpio_get_value(SLIDE_DETECT))
   {
      slide_close = gpio_get_value(SLIDE_DETECT);

#ifndef CONFIG_MACH_SOCIAL /* Social product specific changes */ 
#endif 
      /* call rpc */
      msm_proc_comm(PCOM_SLIDE_STATUS, &slide_close, 0);
   }
   enable_irq(slide_data.irq);
}

static irqreturn_t mot_slide_irq_call(int irq, void *dev_id)
{
	struct slide_status *data = (struct slide_status *)dev_id;
#ifdef DEBUG_TOUCHPAD
	printk("mot_slide_irq_call\n");
#endif
	disable_irq(data->irq);
	schedule_work(&data->work);
	return IRQ_HANDLED;
}
static struct gpio_event_direct_entry mot_kpd_switch_map[] = {
#if defined(CONFIG_MACH_SOCIAL) /* Social product specific changes */	
	{ 41,       KEY_CAMERA      },
	{ 38,       KEY_HOME        },
	{ 39,       KEY_VOLUMEUP    },
	{ 40,	    KEY_VOLUMEDOWN  }
#else 
        { SLIDE_DETECT, SW_LID }
#endif /* defined(CONFIG_MACH_SOCIAL) */
};

static struct gpio_event_input_info mot_kpd_switch_info = {
	.info.func = gpio_event_input_func,
#if defined(CONFIG_MACH_CALGARY)
	.flags = 0,
#elif defined(CONFIG_MACH_PITTSBURGH)
	.flags = GPIOEDF_ACTIVE_HIGH,
#elif defined(CONFIG_MACH_SOCIAL)
	.flags = 0,
#else /* defined(CONFIG_MACH_PITTSBURGH) */
	.flags = 0,
#endif /* defined(CONFIG_MACH_PITTSBURGH) */
#if defined(CONFIG_MACH_SOCIAL) /* Social product specific changes */
        .type = EV_KEY,
#else
 	.type = EV_SW,
#endif /* defined(CONFIG_MACH_SOCIAL) */
	.keymap = mot_kpd_switch_map,
	.keymap_size = ARRAY_SIZE(mot_kpd_switch_map)
};

static struct gpio_event_direct_entry mot_slide_map[] = {
	{ SLIDE_DETECT,       SW_LID       }
};

static struct gpio_event_input_info mot_slide_info = {
	.info.func = gpio_event_input_func,
	.flags = 0,
	.type = EV_SW,
	.keymap = mot_slide_map,
	.keymap_size = ARRAY_SIZE(mot_slide_map)
};

static struct gpio_event_info *mot_kpd_info[] = {
	&mot_kpd_switch_info.info,
#if defined(CONFIG_MACH_SOCIAL)
        &mot_slide_info.info,
#endif /* defined(CONFIG_MACH_SOCIAL) */
};

static struct gpio_event_platform_data mot_kpd_data = {
#if defined(CONFIG_MACH_CALGARY)
	.name = "calgary-kpd",
#elif defined(CONFIG_MACH_PITTSBURGH)
	.name = "pittsburgh-kpd",
#elif defined(CONFIG_MACH_SOCIAL)
	.name = "social-kpd",
#else /* defined(CONFIG_MACH_PITTSBURGH) */
	.name = "mot-kpd",
#endif /* defined(CONFIG_MACH_PITTSBURGH) */
	.info = mot_kpd_info,
	.info_count = ARRAY_SIZE(mot_kpd_info)
};

static struct platform_device mot_kpd_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev		= {
		.platform_data	= &mot_kpd_data,
	},
};
#ifdef CONFIG_MACH_SOCIAL
static struct msm_gpio home_cameral_gpios[] = {
	{ GPIO_CFG(38, 0, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA), "home" },
	{ GPIO_CFG(39, 0, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA), "up" },
	{ GPIO_CFG(40, 0, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA), "down" },
	{ GPIO_CFG(41, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "camera" },
	{ GPIO_CFG(42, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "SLIDE" }
};

int slide_home_enable( uint8_t value)
{

if(value)
{
	enable_irq(slide_data.irq);
	msm_gpios_enable(home_cameral_gpios,
					ARRAY_SIZE(home_cameral_gpios));

}
else
{
	disable_irq(slide_data.irq);
	msm_gpios_disable(home_cameral_gpios,
					ARRAY_SIZE(home_cameral_gpios));
}
	return 1;
}

EXPORT_SYMBOL(slide_home_enable);
#endif /*CONFIG_MACH_SOCIAL*/

int __init mot_init_kpd()
{
   /*Initialize the slide status during power up*/
   slide_close = gpio_get_value(SLIDE_DETECT);

   msm_proc_comm(PCOM_SLIDE_STATUS, &slide_close, 0);
   INIT_WORK(&slide_data.work, mot_slide_gpio_call);
   slide_data.irq = gpio_to_irq(SLIDE_DETECT);
   request_irq(slide_data.irq, mot_slide_irq_call,
      IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "slide status", &slide_data);
   #if defined(CONFIG_MACH_SOCIAL)
   set_irq_wake(slide_data.irq, 1);
   #endif /*defined(CONFIG_MACH_SOCIAL)*/
   return platform_device_register(&mot_kpd_device);
}

