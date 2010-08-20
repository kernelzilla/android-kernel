/* linux/arch/arm/mach-msm/board-halibut-keypad.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>
#include <asm/gpio.h>
#include <mach/gpio.h>
#include "board-mot.h"
#include <mach/gpio.h>

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

      /* Pittsburgh specific */
      if (slide_close == FLIP_CLOSED)
      {
         /* set the DISP_BL_SEL GPIO to turn ON the CLI */
         gpio_set_value (DISP_BL_SEL, DISP_BL_SEL_CLI);
      }        
      else
      {
         /* set the DISP_BL_SEL GPIO to turn ON the main LCD */
         gpio_set_value (DISP_BL_SEL, DISP_BL_SEL_MAIN);
      }

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

static void __exit keypad_mot_exit_bell(void);

static unsigned int keypad_row_gpios[] = {31, 32, 33, 34, 35 };
static unsigned int keypad_col_gpios[] = {41, 37, 38, 39, 40 };

#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(keypad_col_gpios) + (col))

 static const unsigned short keypad_keymap_pitt[ARRAY_SIZE(keypad_col_gpios) * ARRAY_SIZE(keypad_row_gpios)] = 
{

    [KEYMAP_INDEX(0, 0)] = KEY_9,
	[KEYMAP_INDEX(0, 1)] = KEY_3,
	[KEYMAP_INDEX(0, 2)] = KEY_6,            
	[KEYMAP_INDEX(0, 3)] = 227, //star
	[KEYMAP_INDEX(0, 4)] = KEY_MENU, 

	[KEYMAP_INDEX(1, 0)] = KEY_1,
	[KEYMAP_INDEX(1, 1)] = KEY_4,
	[KEYMAP_INDEX(1, 2)] = KEY_7,
	[KEYMAP_INDEX(1, 3)] = KEY_0,           
	[KEYMAP_INDEX(1, 4)] = KEY_UP, 

	[KEYMAP_INDEX(2, 0)] = KEY_2,
	[KEYMAP_INDEX(2, 1)] = KEY_5,      
	[KEYMAP_INDEX(2, 2)] = KEY_8, 
	[KEYMAP_INDEX(2, 3)] = 228, //pound      
	[KEYMAP_INDEX(2, 4)] = KEY_HOME,        

	[KEYMAP_INDEX(3, 0)] = KEY_SEND,          
	[KEYMAP_INDEX(3, 1)] = KEY_DOWN,
	[KEYMAP_INDEX(3, 2)] = KEY_RIGHT,      
	[KEYMAP_INDEX(3, 3)] = 232, //center 
	[KEYMAP_INDEX(3, 4)] = KEY_LEFT,

	/*[KEYMAP_INDEX(4, 0)] = ,  */        
	/*[KEYMAP_INDEX(4, 1)] = ,  */    
	[KEYMAP_INDEX(4, 2)] = KEY_BACK,
	/*[KEYMAP_INDEX(4, 3)] = ,  */      
	[KEYMAP_INDEX(4, 4)] = KEY_SEARCH,

};

// SLIDE
static struct gpio_event_direct_entry mot_kpd_switch_map[] = {
	{ SLIDE_DETECT,       SW_LID       }
};

static struct gpio_event_input_info mot_kpd_switch_info = {
	.info.func = gpio_event_input_func,
#if defined(CONFIG_MACH_CALGARY)
   .flags = 0,
#elif defined(CONFIG_MACH_PITTSBURGH)
   .flags = GPIOEDF_ACTIVE_HIGH,
#elif defined(CONFIG_MACH_SOCIAL)
   .flags = GPIOEDF_ACTIVE_HIGH,
#else /* defined(CONFIG_MACH_PITTSBURGH) */
   .flags = 0,
#endif /* defined(CONFIG_MACH_PITTSBURGH) */
	.type = EV_SW,
	.keymap = mot_kpd_switch_map,
	.keymap_size = ARRAY_SIZE(mot_kpd_switch_map)
};

static struct gpio_event_matrix_info mot_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= keypad_keymap_pitt,
	.output_gpios	= keypad_row_gpios,
	.input_gpios	= keypad_col_gpios,    
	.noutputs	= ARRAY_SIZE(keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
    .poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 50 * NSEC_PER_MSEC,
	.flags = GPIOKPF_LEVEL_TRIGGERED_IRQ |
		 GPIOKPF_REMOVE_PHANTOM_KEYS |
		 GPIOKPF_PRINT_UNMAPPED_KEYS /*| GPIOKPF_PRINT_MAPPED_KEYS */
};

struct gpio_event_info *pitt_keypad_info[] = {
    &mot_keypad_matrix_info.info,
    &mot_kpd_switch_info.info
};

static struct gpio_event_platform_data pitt_keypad_data = {
	  .name = "pittsburgh-bell-kpd",
	  .info		= pitt_keypad_info,
	  .info_count	= ARRAY_SIZE(pitt_keypad_info)
};

struct platform_device pitt_keypad_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= 0,
	.dev	= {
		.platform_data	= &pitt_keypad_data,
	},
};

static void __exit keypad_mot_exit_bell(void)
{
    platform_device_unregister(&pitt_keypad_device);
}

int __init pitt_init_bell_kpd()
{
   if ( !machine_is_msm7627_pittsburgh() )
      return 0; 

/*Initialize the slide status during power up*/
   slide_close = gpio_get_value(SLIDE_DETECT);

   /* Pittsburgh */
   if (slide_close == FLIP_CLOSED)
   {
      /* set the DISP_BL_SEL GPIO to turn ON the CLI */
      gpio_set_value (DISP_BL_SEL, DISP_BL_SEL_CLI);
   }
   else
   {
      /* set the DISP_BL_SEL GPIO to turn ON the main LCD */
      gpio_set_value (DISP_BL_SEL, DISP_BL_SEL_MAIN);
   }

   
   msm_proc_comm(PCOM_SLIDE_STATUS, &slide_close, 0);
   INIT_WORK(&slide_data.work, mot_slide_gpio_call);
   slide_data.irq = gpio_to_irq(SLIDE_DETECT);
   request_irq(slide_data.irq, mot_slide_irq_call,
   IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "slide status", &slide_data);

   return platform_device_register(&pitt_keypad_device); 
  
}

late_initcall(pitt_init_bell_kpd);
module_exit(keypad_mot_exit_bell);
