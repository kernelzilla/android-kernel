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
 
#if defined (CONFIG_MACH_PITTSBURGH)
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#endif /* CONFIG_MACH_PITTSBURGH */
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio_event.h>
#include <asm/gpio.h>
#if defined (CONFIG_MACH_PITTSBURGH)
#include <asm/uaccess.h>
#endif /* CONFIG_MACH_PITTSBURGH */
#include "board-mot.h"
#include <linux/workqueue.h>
#include "proc_comm.h"

#define KPD_PRINTK(fmt, ...) \
   do { \
      if (kpd_debug) \
         printk(fmt, ##__VA_ARGS__); \
   } while(0)

#if defined (CONFIG_MACH_PITTSBURGH)
#define KEYMAP_INDEX(row, col)      ((row) * ARRAY_SIZE(keypad_col_gpios) + (col))
#define BACKLIGHT_SELECT_DEVICE     "backlight_select"
#define BACKLIGHT_SELECT_CMD_LEN    8
#define BACKLIGHT_SELECT_GPIO_CLI   0
#define BACKLIGHT_SELECT_GPIO_MAIN  1
#define SLIDE_CLOSED                1
#define SLIDE_OPENED                0
#endif /* CONFIG_MACH_PITTSBURGH */

struct slide_status {
	struct work_struct work;
	int irq;
};

static struct slide_status slide_data;
static uint32_t slide_close;
static bool kpd_debug = 0;

static int __init mot_init_kpd(void);
#if defined (CONFIG_MACH_PITTSBURGH)
static void backlight_select_handle_slide(int slide_close);
#endif /* CONFIG_MACH_PITTSBURGH */

static void mot_slide_gpio_call(struct work_struct *work)
{
   KPD_PRINTK("%s: enter\n", __FUNCTION__);
   if (slide_close != gpio_get_value(SLIDE_DETECT))
   {
      slide_close = gpio_get_value(SLIDE_DETECT);

      /* call rpc */
      KPD_PRINTK("%s: calling msm_proc_comm(PCOM_SLIDE_STATUS,%u,0)\n",
         __FUNCTION__, slide_close);
      msm_proc_comm(PCOM_SLIDE_STATUS, &slide_close, 0);
#if defined (CONFIG_MACH_PITTSBURGH)
      backlight_select_handle_slide(slide_close);
#endif
   }
   enable_irq(slide_data.irq);
   KPD_PRINTK("%s: exit\n", __FUNCTION__);
}

static irqreturn_t mot_slide_irq_call(int irq, void *dev_id)
{
	struct slide_status *data = (struct slide_status *)dev_id;
	disable_irq(data->irq);
	schedule_work(&data->work);
	return IRQ_HANDLED;
}

#if defined (CONFIG_MACH_PITTSBURGH)
static int backlight_select = -1;
static int backlight_select_open(struct inode *inode, struct file *filp)
{
   KPD_PRINTK("%s: enter\n", __FUNCTION__);
   KPD_PRINTK("%s: exit\n", __FUNCTION__);
   return 0;
}

static ssize_t backlight_select_read(struct file *file, char __user *buf,
    size_t count, loff_t *ppos)
{
   int len;
   int backlight_select_tmp = -1;
   unsigned char cmd[BACKLIGHT_SELECT_CMD_LEN];

   KPD_PRINTK("%s: enter\n", __FUNCTION__);

   if (file->private_data)
      return 0;

   switch(backlight_select) {
   case BACKLIGHT_SELECT_GPIO_MAIN:
      backlight_select_tmp = 0;
      break;
   case BACKLIGHT_SELECT_GPIO_CLI:
      backlight_select_tmp = 1;
      break;
   }
   sprintf (cmd, "%d\n", backlight_select_tmp);
   KPD_PRINTK("%s: returning %s", __FUNCTION__, cmd);

   len = strlen (cmd);
   if (copy_to_user (buf, cmd, len))
      return -EFAULT;

   file->private_data = (void *)1;
   KPD_PRINTK("%s: return(%d)\n", __FUNCTION__, len);
   return len;
}

static int backlight_select_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
   int backlight_select_tmp = -1;
   unsigned char cmd[BACKLIGHT_SELECT_CMD_LEN];

   KPD_PRINTK("%s: enter\n", __FUNCTION__);
   if (count < 1)
      return 0;

   if (count > BACKLIGHT_SELECT_CMD_LEN-1)
      count = BACKLIGHT_SELECT_CMD_LEN-1;
      
   if (copy_from_user (cmd, buf, count))
      return -EFAULT;

   cmd[count] = '\0';
   if (cmd[count-1] == '\n')
      cmd[--count] = '\0';

   KPD_PRINTK("%s: string received %s\n", __FUNCTION__, cmd);
   sscanf(cmd, "%d", &backlight_select_tmp);
   KPD_PRINTK("%s: processed value %d\n", __FUNCTION__, backlight_select_tmp);
   switch(backlight_select_tmp) {
   case 0:
      backlight_select_handle_slide(SLIDE_OPENED);
      break;
   case 1:
      backlight_select_handle_slide(SLIDE_CLOSED);
      break;
   default:
      return -EINVAL;
   }
   KPD_PRINTK("%s: exit\n", __FUNCTION__);
   return 0;
}

static void backlight_select_handle_slide(int slide_close)
{
   int backlight_select_tmp = -1;

   KPD_PRINTK("%s: enter\n", __FUNCTION__);
   switch(slide_close) {
   case SLIDE_CLOSED:
      backlight_select_tmp = BACKLIGHT_SELECT_GPIO_CLI;
      break;
   case SLIDE_OPENED:
      backlight_select_tmp = BACKLIGHT_SELECT_GPIO_MAIN;
      break;
   default:
      printk(KERN_ERR "%s: param value is invalid, slide_close=%d\n",
         __FUNCTION__, slide_close);
      break;
   }
   if (backlight_select_tmp >= 0 && backlight_select_tmp != backlight_select) {
      backlight_select = backlight_select_tmp;
      gpio_set_value (DISP_BL_SEL, backlight_select);
      KPD_PRINTK("%s: backlight_select set to %d\n", __FUNCTION__,
         backlight_select);
   }
   KPD_PRINTK("%s: exit\n", __FUNCTION__);
}

static unsigned int keypad_row_gpios[] = {31, 32, 33, 34, 35 };
static unsigned int keypad_col_gpios[] = {41, 37, 38, 39, 40 };
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

static struct gpio_event_matrix_info mot_keypad_matrix_info = {
	.info.func              = gpio_event_matrix_func,
	.keymap                 = keypad_keymap_pitt,
	.output_gpios           = keypad_row_gpios,
	.input_gpios            = keypad_col_gpios,    
	.noutputs               = ARRAY_SIZE(keypad_row_gpios),
	.ninputs                = ARRAY_SIZE(keypad_col_gpios),
	.settle_time.tv.nsec    = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec      = 20 * NSEC_PER_MSEC,
	.debounce_delay.tv.nsec = 50 * NSEC_PER_MSEC,
	.flags                  = (GPIOKPF_LEVEL_TRIGGERED_IRQ |
		GPIOKPF_REMOVE_PHANTOM_KEYS | GPIOKPF_PRINT_UNMAPPED_KEYS),
		/*| GPIOKPF_PRINT_MAPPED_KEYS */
};

struct file_operations backlight_select_fops =
{
	.owner   = THIS_MODULE,
	.open    = backlight_select_open,
	.write   = backlight_select_write,
	.read    = backlight_select_read,
};

static struct miscdevice backlight_select_misc_dev =
{
	.minor   = MISC_DYNAMIC_MINOR,
	.name    = BACKLIGHT_SELECT_DEVICE,
	.fops    = &backlight_select_fops,
};
#endif /* CONFIG_MACH_PITTSBURGH */

static struct gpio_event_direct_entry mot_kpd_switch_map[] = {
	{ SLIDE_DETECT, SW_LID }
};

static struct gpio_event_input_info mot_kpd_switch_info = {
	.info.func = gpio_event_input_func,
#if defined(CONFIG_MACH_CALGARY)
	.flags = 0,
#elif defined(CONFIG_MACH_PITTSBURGH)
	.flags = GPIOEDF_ACTIVE_HIGH,
#else /* defined(CONFIG_MACH_PITTSBURGH) */
	.flags = 0,
#endif /* defined(CONFIG_MACH_PITTSBURGH) */
	.type = EV_SW,
	.keymap = mot_kpd_switch_map,
	.keymap_size = ARRAY_SIZE(mot_kpd_switch_map)
};

static struct gpio_event_info *mot_kpd_info[] = {
#if defined(CONFIG_MACH_PITTSBURGH)
	&mot_keypad_matrix_info.info,
#endif /* CONFIG_MACH_PITTSBURGH */
	&mot_kpd_switch_info.info,
};

static struct gpio_event_platform_data mot_kpd_data = {
#if defined(CONFIG_MACH_CALGARY)
	.name = "calgary-kpd",
#elif defined(CONFIG_MACH_PITTSBURGH)
	.name = "pittsburgh-bell-kpd",
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

static int __init mot_init_kpd(void)
{
   int ret = 0;

   KPD_PRINTK("%s: enter\n", __FUNCTION__);

   slide_close = gpio_get_value(SLIDE_DETECT);
#if defined (CONFIG_MACH_PITTSBURGH)
   ret = misc_register(&backlight_select_misc_dev);
   if (!ret)
      backlight_select_handle_slide(slide_close);
   else
      printk(KERN_ERR "%s: failed to register %s misc device\n",
         __FUNCTION__, backlight_select_misc_dev.name);
#endif /* CONFIG_MACH_PITTSBURGH */

   if (!ret) {
      /* Initialize the slide status during power up */
      msm_proc_comm(PCOM_SLIDE_STATUS, &slide_close, 0);
      INIT_WORK(&slide_data.work, mot_slide_gpio_call);
      slide_data.irq = gpio_to_irq(SLIDE_DETECT);
      ret = request_irq(slide_data.irq, mot_slide_irq_call,
         IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "slide status", &slide_data);
      if (!ret) {
         ret = platform_device_register(&mot_kpd_device);
         if (ret)
            printk(KERN_ERR "%s: failed to register %s device\n",
               __FUNCTION__, mot_kpd_device.name);
      } else {
         printk(KERN_ERR "%s: request_irq failed, irq=%d\n", __FUNCTION__,
            slide_data.irq);
      }
   }

   KPD_PRINTK("%s: exit, ret=%d\n", __FUNCTION__, ret);
   return ret;
}

device_initcall(mot_init_kpd);

