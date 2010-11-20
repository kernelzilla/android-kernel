/* arch/arm/mach-msm/switch-mot.c
 *
 * Copyright (C) 2008 Motorola, Inc.
 * Author: Vladimir Karpovich <Vladimir.Karpovich@motorola.com>
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
#include <mach/vreg.h>

#include <asm-arm/mach-types.h>


#include "board-mot.h"

#define DRIVER_NAME "hs"
#define DRIVER_NAME_H2W "h2w"
#define HEADSET_DETECT_GPIO   	    39
#define HEADSET_SEND_END_0_GPIO     37
#define HEADSET_SEND_END_1_GPIO     38

#define HEADSET_DETECT_DEBOUNCE_TIMER   300000000    /*  500 ms */
#define HEADSET_BTN_DEBOUNCE_TIMER      100000000    /* 100 ms */
#define HEADSET_BTN_DETECT_REPEATS      20     
#define HEADSET_BTN_HOLD_TIMER          3            /* 3 sec */     

#define HEADSET_MIC_BIAS_DELAY          50           /* 50 msec*/

extern unsigned long msleep_interruptible(unsigned int);

struct hs_dev {
	struct switch_dev    sdev,sdev_h2w;
	struct input_dev    *input;
	uint8_t              state,detect_state;
	uint8_t              mic;
	uint8_t              tv_out;
	uint8_t              btn_state;
	uint8_t              btn_in_test;
	uint8_t              btn_counter;
	uint8_t              btn_off_counter;
	unsigned int         irq;
	unsigned int         irq_btn_0;
	unsigned int         irq_btn_1;
	struct hrtimer       timer;
	ktime_t              debounce_time;
	struct hrtimer       btn_timer;
	ktime_t              btn_debounce_time;	
	struct work_struct   work;    
};



static struct hs_dev	  *the_hs  ;


int convert_to_h2w( int state )
{ 
  switch( state )
    {
   case 1 : return(2);
   case 3 : return(1);
   }
return(state );
}

void hs_switch_set_state(struct hs_dev *hs, int state)
{
  switch_set_state(&hs->sdev, state);
  switch_set_state(&hs->sdev_h2w,convert_to_h2w(state));

}


/* rpc call for pm mic on */
int msm_pm_mic_en(unsigned char  enable_disable)
{
	struct vreg *vreg;
	int rc = 0;


	vreg = vreg_get(0, "gp4");
		if(enable_disable )
		{
		   if ((rc = vreg_enable(vreg)))
			printk(KERN_ERR "%s: vreg(gp4) enable failed (%d)\n", __func__, rc);
		}
		else
		{
		   if ((rc = vreg_disable(vreg)))
			printk(KERN_ERR "%s: vreg(gp4) enable failed (%d)\n", __func__, rc);
		}

	if (rc < 0) 
	{
		printk(KERN_ERR "%s: msm_pm_mic_en failed! rc = %d\n",
			__func__, rc);
	} 
/*	else
	{
		printk(KERN_INFO "msm_pm_mic_en %d \n",enable_disable);
	}
*/
	return rc;
}




static int __init hs_alloc(void)
{
	struct hs_dev		*hs;

	hs = kzalloc(sizeof *hs, GFP_KERNEL);
	if (!hs)
		return -ENOMEM;

	the_hs = hs;

	return 0;
}

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case 0:   // NO DEVICE
		return sprintf(buf, "No Device\n");
	case 1:    // Stereo HEADPHONE  
		return sprintf(buf, "Stereo HeadPhone\n");
	case 3:    // Stereo HeadSet  
		return sprintf(buf, "Stereo Headset\n");
	case 5:    // TV OUT  
		return sprintf(buf, "TV OUT\n");
	}
	return -EINVAL;
}
static ssize_t print_h2w_switch_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case 0:   // NO DEVICE
		return sprintf(buf, "No Device\n");
	case 2:    // Stereo HEADPHONE  
		return sprintf(buf, "Stereo HeadPhone\n");
	case 1:    // Stereo HeadSet  
		return sprintf(buf, "Stereo Headset\n");
	}
	return -EINVAL;
}
static int  do_check_mic(void)
{
	uint8_t   val1 , val2;
	msm_pm_mic_en(1);
	msleep_interruptible(HEADSET_MIC_BIAS_DELAY);
	val1 =gpio_get_value(HEADSET_SEND_END_0_GPIO);
	val2 =gpio_get_value(HEADSET_SEND_END_1_GPIO);
	if( (val1 == 1) && (val2 == 0 ) )	 return (1);	/* The mic is detected */

	/* No Mic , turn off mic bias */
	msm_pm_mic_en(0);	
	 return (0);
}
int  get_state_value(uint8_t state,uint8_t mic,uint8_t tv_out )
{
	return ( state |( mic <<1) | (tv_out <<2 )) ;
}
void enable_disable_btn_irq( struct hs_dev *hs,int state )
{
	unsigned long irq_flags;
	 local_irq_save(irq_flags);
	if( state )
        {
	 enable_irq(hs->irq_btn_0);
	 enable_irq(hs->irq_btn_1);
	}else { 
	 disable_irq(hs->irq_btn_0);
	 disable_irq(hs->irq_btn_1);
	}
	 local_irq_restore(irq_flags);
}
static int do_set_state(struct hs_dev *hs, uint8_t new_state)
{
	unsigned long irq_flags;
        int  mic_state;

	
        the_hs->detect_state=1;
        hrtimer_start(&hs->timer, ktime_set(HEADSET_BTN_HOLD_TIMER,0), HRTIMER_MODE_REL);

	if (the_hs->btn_in_test)
		enable_disable_btn_irq(hs, 0);

	the_hs->btn_in_test = 0;
	if (new_state != hs->state) {
		hs->state = new_state;
		if (new_state) { /* headset is ON. cheak mic */
			hs->mic = do_check_mic(); /* mic and maybe the BTN*/
		} else {
			if (hs->mic)
				msm_pm_mic_en(0);
			if (the_hs->btn_in_test)
				enable_disable_btn_irq(hs, 0);
			hs->mic = 0;
			the_hs->btn_in_test = 0;
		}
		hs_switch_set_state(hs, get_state_value(hs->state,
							hs->mic, hs->tv_out));
	} else if (hs->state) {
		/* if state did not changed and ON then retest the mic it
		   can the disconnected if HS is hlf removed ( it is very
		   noisy :) */
		if (hs->mic) {
			mic_state  = do_check_mic(); /*mic and maybe the BTN*/
			if (hs->mic != mic_state) {
				hs->mic = mic_state;
				hs_switch_set_state(hs, get_state_value(
							hs->state,
							hs->mic, hs->tv_out));
			}
		}
	}

	return 0;
}

static void do_button_event(struct hs_dev *hs, uint8_t state)
{
	if (state == hs->btn_state)	return ;
//        printk(KERN_INFO "HEADSET: HeadSet Button  %s , mic %d \n", state ? "PRESSED":"RELEASED",hs->mic);
	hs->btn_state = state ;		
	input_report_key(hs->input, KEY_MEDIA, state);
	input_sync(hs->input);
}



static void hs_work_func( struct work_struct *work)
{
	unsigned long irq_flags;
	struct hs_dev *hs = container_of(work, struct hs_dev, work);


 //       printk(KERN_INFO "HEADSET: hs_work_fun  state %s , mic %d \n", the_hs->detect_state,hs->mic);


	if (!the_hs->detect_state) /* headset detect */ {
	do_set_state(hs, !gpio_get_value(HEADSET_DETECT_GPIO));
	local_irq_save(irq_flags);
	enable_irq(hs->irq);
	local_irq_restore(irq_flags);
	} else { /* enable button irq */
		if (!hs->mic) {
			hs->mic = do_check_mic();
			if (hs->mic)    /*retest  mic*/ {
				/* Found the mic . report it */
				hs_switch_set_state(hs, get_state_value(
					hs->state, hs->mic,
					hs->tv_out));
			}
		}
        if( (hs->mic))  /* there is the mic and may be the BTN */
         {	// ENABLE button detect interrupt if headset is in and btn_int is disabled /
		if( !the_hs->btn_in_test)
		{
		enable_disable_btn_irq(hs,1);
	 	the_hs->btn_in_test=1;
	 	}	
          }	
         }
}

static enum hrtimer_restart hs_detect_event_timer_func(struct hrtimer *data)
{
	struct hs_dev *hs = container_of(data,struct hs_dev, timer);	
	schedule_work(&hs->work);
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart hs_btn_event_timer_func(struct hrtimer *data)
{
	unsigned long irq_flags;
	struct hs_dev *hs = container_of(data,struct hs_dev, btn_timer);	
	uint8_t   val1 , val2, hs_detect;
	val1 =gpio_get_value(HEADSET_SEND_END_0_GPIO);
	val2 =gpio_get_value(HEADSET_SEND_END_1_GPIO);
        hs_detect = gpio_get_value(HEADSET_DETECT_GPIO); 
        if(!hs_detect ) { /* HS is still in  check btn*/
	 if( val1 == val2  )
	 {
	  /* try  several times  to avoid false triggering */
	  if(  hs->btn_counter <HEADSET_BTN_DETECT_REPEATS )
	   {
            hs->btn_off_counter=0;
	    hs->btn_counter++;
	       return HRTIMER_RESTART;
            }else	 
	      do_button_event(hs, 1);  // Button pressed	  
	  } else {
          hs->btn_counter=0;
	  if(  hs->btn_off_counter <HEADSET_BTN_DETECT_REPEATS )
	   {
	    hs->btn_off_counter++;
	       return HRTIMER_RESTART;
            }else	 
	      do_button_event(hs, 0);  // Button released
	   }
       }else { /* HS was removed during BTN test */
        the_hs->btn_in_test =0;
	hs->btn_counter=0;
	hs->btn_off_counter=0;
	return HRTIMER_NORESTART;
       }
       /* RE ENABLE button detect interrupt  */
	hs->btn_counter=0;
	hs->btn_off_counter=0;

	if ( !the_hs->btn_in_test )
	{ 
	enable_disable_btn_irq(hs,1);
	the_hs->btn_in_test=1;
	} 
	 
	return HRTIMER_NORESTART;
}

static irqreturn_t hs_irq_handler(int irq, void *dev_id)
{
	unsigned long irq_flags;
	struct hs_dev *hs = (struct hs_dev *)dev_id;
	local_irq_save(irq_flags);	
	disable_irq(hs->irq);
       /* DISABLE button detect interrupt  because HeadSet status is changing */
//        printk(KERN_INFO "HEADSET:hs_irq_handler  state %s , mic %d \n", hs->detect_state,hs->mic);
	

	if ( the_hs->btn_in_test )
	{ 
	enable_disable_btn_irq(hs,0);
	the_hs->btn_in_test = 0;
	}
	local_irq_restore(irq_flags);
        the_hs->detect_state=0;
        hrtimer_start(&hs->timer, hs->debounce_time, HRTIMER_MODE_REL);	

	return IRQ_HANDLED;
}




static irqreturn_t hs_button_irq_handler(int irq, void *dev_id)
{
	unsigned long irq_flags;
	struct hs_dev *hs = (struct hs_dev *)dev_id;

         /* DISABLE BTN INT  and start debounce timer if headset is in */
	if ( the_hs->btn_in_test )
	{ 	
	enable_disable_btn_irq(hs,0);
	 the_hs->btn_in_test=0;
	}		
	if( the_hs->state )
	  hrtimer_start(&hs->btn_timer, hs->btn_debounce_time, HRTIMER_MODE_REL);

 
	return IRQ_HANDLED;
}



#if defined(CONFIG_DEBUG_FS)
static int  hs_debug_set(void *data, u64 val)
{
	switch_set_state(&the_hs->sdev, (int)val);
	return 0;
}

static int  hs_debug_get(void *data, u64 *val)
{
	uint8_t   val1 , val2;
	val1 =gpio_get_value(HEADSET_SEND_END_0_GPIO);
	val2 =gpio_get_value(HEADSET_SEND_END_1_GPIO);
       printk(KERN_INFO "GPIO are  %d   :  %d \n", val1,val2 );

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(hs_debug_fops, hs_debug_get, hs_debug_set, "%llu\n");

static int __init hs_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("hs", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("state", 0644, dent, NULL, &hs_debug_fops);

	return 0;
}

device_initcall(hs_debug_init);
#endif



static int __init hs_probe(struct platform_device *pdev)
{

	int		rc;
	unsigned long request_flags =  IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING;
	unsigned long irq_flags;

	printk(KERN_INFO "HEADSET: Registering headset driver\n");
	
	rc = hs_alloc();
	if (rc != 0)
		return rc;

	the_hs->debounce_time = ktime_set(0, HEADSET_DETECT_DEBOUNCE_TIMER);  /* 300 ms */	
	the_hs->btn_debounce_time = ktime_set(0, HEADSET_BTN_DEBOUNCE_TIMER); /* 10 ms */
	the_hs->sdev.name = DRIVER_NAME;
	the_hs->sdev.print_name = print_switch_name;
	the_hs->sdev_h2w.name = DRIVER_NAME_H2W;
	the_hs->sdev_h2w.print_name = print_h2w_switch_name;
        the_hs->btn_in_test =0;
	the_hs->btn_state = 0;
	the_hs->state =  0;

       INIT_WORK(&the_hs->work, hs_work_func);

	
	rc = switch_dev_register(&the_hs->sdev);
	if (rc < 0)
		goto err_switch_dev_register;

	rc = switch_dev_register(&the_hs->sdev_h2w);
	if (rc < 0)
		goto err_h2w_switch_dev_register;
	
	
	
        rc = gpio_request(HEADSET_DETECT_GPIO, "hs");
        if (rc < 0)
	      goto err_request_detect_gpio;
 
        rc = gpio_request(HEADSET_SEND_END_0_GPIO, "hs_send_end_0");
        if (rc < 0)
	      goto err_request_detect_gpio_0;

        rc = gpio_request(HEADSET_SEND_END_1_GPIO, "hs_send_end_1");
        if (rc < 0)
	      goto err_request_detect_gpio_1;

 
 
        rc  = gpio_direction_input(HEADSET_DETECT_GPIO);
        if (rc < 0)
	      goto err_set_detect_gpio;
 
       rc  = gpio_direction_input(HEADSET_SEND_END_0_GPIO);
       if (rc < 0)
	      goto err_set_detect_gpio;
 
        rc  = gpio_direction_input(HEADSET_SEND_END_1_GPIO);
        if (rc < 0)
	      goto err_set_detect_gpio;


 	the_hs->irq = gpio_to_irq(HEADSET_DETECT_GPIO);
 	if (the_hs->irq  < 0) {
		rc = the_hs->irq ;
		goto err_get_hs_detect_irq_num_failed;
	}

 	the_hs->irq_btn_0 = gpio_to_irq(HEADSET_SEND_END_0_GPIO);
 	if (the_hs->irq_btn_0  < 0) {
		rc = the_hs->irq_btn_0 ;
		goto err_get_hs_detect_irq_num_failed;
	}

 	the_hs->irq_btn_1 = gpio_to_irq(HEADSET_SEND_END_1_GPIO);
 	if (the_hs->irq_btn_1  < 0) {
		rc = the_hs->irq_btn_1 ;
		goto err_get_hs_detect_irq_num_failed;
	}


 	hrtimer_init(&the_hs->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	the_hs->timer.function = hs_detect_event_timer_func;
 
 	hrtimer_init(&the_hs->btn_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	the_hs->btn_timer.function = hs_btn_event_timer_func;

 
        local_irq_save(irq_flags); 

	rc = request_irq(the_hs->irq, hs_irq_handler,request_flags, "hs_detect", the_hs);

	if (rc < 0)
		goto err_request_detect_irq;

	rc = set_irq_wake(the_hs->irq, 1);
	if (rc < 0)
		goto err_request_detect_irq;
		

	rc = request_irq(the_hs->irq_btn_0, hs_button_irq_handler, request_flags, "hs_btn_0", the_hs);
	if (rc < 0)
		goto err_request_detect_irq;

	rc = set_irq_wake(the_hs->irq_btn_0, 1);
	if (rc < 0)
		goto err_request_detect_irq;


	rc = request_irq(the_hs->irq_btn_1, hs_button_irq_handler, request_flags, "hs_btn_1", the_hs);
	if (rc < 0)
		goto err_request_detect_irq;

	rc = set_irq_wake(the_hs->irq_btn_1, 1);
	if (rc < 0)
		goto err_request_detect_irq;

         /* DISABLE BTN INT if  Headset is not plugged in */
	 

	        the_hs->btn_in_test=0;
	        disable_irq(the_hs->irq_btn_1);
	        disable_irq(the_hs->irq_btn_0);


	
	local_irq_restore(irq_flags);
	
	the_hs->input = input_allocate_device();
	if (!the_hs->input) {
		rc = -ENOMEM;
		goto err_request_input_dev;
	}

	the_hs->input->name = "headset";
	set_bit(EV_KEY, the_hs->input->evbit);
	set_bit(EV_SW,  the_hs->input->evbit);
	set_bit(KEY_MEDIA , the_hs->input->keybit);   
	set_bit(KEY_POWER , the_hs->input->keybit);   
	set_bit(KEY_END   , the_hs->input->keybit); 


        the_hs->detect_state=0;

	rc = input_register_device(the_hs->input);
	if (rc < 0)
		goto err_register_input_dev;


	return 0;
	
err_register_input_dev:
	input_free_device(the_hs->input);		
err_request_input_dev:
err_request_detect_irq:
	local_irq_restore(irq_flags);
err_set_detect_gpio:	
err_get_hs_detect_irq_num_failed:
	gpio_free(HEADSET_SEND_END_1_GPIO);
err_request_detect_gpio_1:
	gpio_free(HEADSET_SEND_END_0_GPIO);
err_request_detect_gpio_0:
	gpio_free(HEADSET_DETECT_GPIO);	
err_request_detect_gpio:
	switch_dev_unregister(&the_hs->sdev_h2w);
err_h2w_switch_dev_register:
	switch_dev_unregister(&the_hs->sdev);
err_switch_dev_register:
	printk(KERN_ERR "HEADSET: Failed to register driver\n");

	return rc;
}
static int hs_remove(struct platform_device *pdev)
{
	input_unregister_device(the_hs->input);
	gpio_free(HEADSET_SEND_END_0_GPIO);
	gpio_free(HEADSET_SEND_END_1_GPIO);
	gpio_free(HEADSET_DETECT_GPIO);
	free_irq(the_hs->irq, 0);
	free_irq(the_hs->irq_btn_0, 0);
	free_irq(the_hs->irq_btn_1, 0);
	
	switch_dev_unregister(&the_hs->sdev);
	return 0;
}
static struct platform_device hs_device = {
	.name		= DRIVER_NAME,
};


static struct platform_driver hs_driver = {
	.probe = hs_probe,
	.remove = hs_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

struct input_dev *msm_keypad_get_input_dev(void)
{

	if (the_hs)
		return the_hs->input;
	else {
	 	printk(KERN_ERR "HEADSET:Input device is not ready yet  \n");
		return NULL;
              }
}
EXPORT_SYMBOL(msm_keypad_get_input_dev);





static void __exit hs_exit(void)
{
	platform_device_unregister(&hs_device);
	platform_driver_unregister(&hs_driver);
}

static int __init hs_init(void)
{
	int ret;

	ret = platform_driver_register(&hs_driver);
	if (ret)
	{
	 printk(KERN_ERR "HEADSET:platform_driver_register error %d \n",ret);
	 return ret;
	}
	return platform_device_register(&hs_device);
}


module_init(hs_init);
module_exit(hs_exit);

MODULE_DESCRIPTION("3.5 mm headset  driver");
MODULE_AUTHOR("Vladimir Karpovich <Vladimir.Karpovich@motorola.com>");
MODULE_LICENSE("GPL");
