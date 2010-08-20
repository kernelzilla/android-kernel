/*
 * Copyright (C) 2010 Motorola, Inc.
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
 * Motorola 2010-Jan-26 - Initial Creation
 */


#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h> /* For vreg.h */
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/hwmon-sysfs.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h> /* For ioctl macros */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/uaccess.h> /* For copy_to_user function */
#include "isl29011.h"
#include <linux/unistd.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/wakelock.h>




#ifdef  CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

static uint32_t isl29011_trace = false;
module_param(isl29011_trace, uint, 0644);
#define ISL29011_LOG(fmt,args...)  if (isl29011_trace == true) printk(KERN_INFO fmt, ##args)


#define ISL29011_PROXIMITY_NEAR 20
#define ISL29011_PROXIMITY_FAR  60
/*Typical ADC under 550 lux*/
#define ISL29011_TYP_ADC_IN_550_LUX  512

#define DARK_THRESHOLD_MIN 0
#define DARK_THRESHOLD_MAX 102
#define INDOOR_WEAK_THRESHOLD_MIN  DARK_THRESHOLD_MAX
#define INDOOR_WEAK_THRESHOLD_MAX 256
#define INDOOR_NORMAL_THRESHOLD_MIN  INDOOR_WEAK_THRESHOLD_MAX
#define INDOOR_NORMAL_THRESHOLD_MAX 614
#define INDOOR_STRONG_THRESHOLD_MIN INDOOR_NORMAL_THRESHOLD_MAX
#define INDOOR_STRONG_THRESHOLD_MAX  1331
#define OUTDOOR_NORMAL_THRESHOLD_MIN  INDOOR_STRONG_THRESHOLD_MAX
#define OUTDOOR_NORMAL_THRESHOLD_MAX  4095
#define THRESHOLD_GAP 5

typedef struct thld_range
{
     uint16_t  low_thld;  /*low threshold*/
     uint16_t  hi_thld;   /*high threshold*/
} thld_range_t;  

static thld_range_t als_zone[ALS_LEVEL_END] = {
   {DARK_THRESHOLD_MIN , DARK_THRESHOLD_MAX + THRESHOLD_GAP},
   {INDOOR_WEAK_THRESHOLD_MIN - THRESHOLD_GAP,
	   INDOOR_WEAK_THRESHOLD_MAX + THRESHOLD_GAP},
   {INDOOR_NORMAL_THRESHOLD_MIN - THRESHOLD_GAP,
	   INDOOR_NORMAL_THRESHOLD_MAX + THRESHOLD_GAP},
   {INDOOR_STRONG_THRESHOLD_MIN - THRESHOLD_GAP,
	   INDOOR_STRONG_THRESHOLD_MAX + THRESHOLD_GAP},
   {OUTDOOR_NORMAL_THRESHOLD_MIN - THRESHOLD_GAP,
	   OUTDOOR_NORMAL_THRESHOLD_MAX + THRESHOLD_GAP},
};

static DEFINE_MUTEX(isl29011_mutex);
#define ISL29011_LOCK() do {  mutex_lock (&isl29011_mutex);} while(0)
#define ISL29011_UNLOCK() do { mutex_unlock (&isl29011_mutex);} while(0)
#define ISL29011_SWITCH_MODE_TIME   500 /*500ms*/ 
#define ISL29011_PROX_START_HI_THRESHOLD 0x190 
#define ISL29011_PROX_START_LO_THRESHOLD 0

#define DARK_LUX 98
#define INDOOR_WAK_LUX  248
#define INDOOR_NORMAL_LUX  598
#define INDOOR_STRONG_LUX  1298
#define OUTDOOR_NORMAL_LUX  1698


typedef struct isl29011_data
{
    uint32_t  irq;
    uint8_t  op_mode;  /*mode from proximity to light sensor*/
    uint8_t  state;    /*current state of the device*/ 
    uint16_t fac_adc;  /*factory tuning adc in 550 lux*/
    struct input_dev *idev;  /*report proximity event to upper layer*/
    struct thld_range  proxmity_range;  /*proxmity range threshold*/
    struct thld_range  light_range;  /*light range threshold*/
    ktime_t           switch_mode_time; /*every time to switch op mode*/
    struct hrtimer    switch_mode_timer;
    struct work_struct switch_mode_wq;  /*work queue for switch mode interuppt*/
	  struct work_struct int_wq;    
    struct i2c_client *client;
    struct wake_lock wake_lock;
    uint8_t   initialized;
    uint8_t   fsr_mode;  /*Full scall range*/
    uint8_t   rsl_mode;  /*resolution .width*/
    uint8_t   timing_mode; /* timing mode, 0, internal, 1 external */
    uint8_t   int_persist; /*consecutive number ofintegration cycles */
    uint8_t   prox_schem; /* from LED or ambient */
    uint8_t   mod_freq;
    uint8_t   ir_current; /* IR driver current */
    uint8_t   switch_enabled;  /*keep switching mode or keep one mode*/
} isl29011_data_t;

/*In interuppt handle, should cancel switch mode timer first if current mode is switch mode,
 * After handle the interrupt, restart switch mode*/
static int32_t isl29011_probe(struct i2c_client *client,const struct i2c_device_id *id);
static int32_t isl29011_remove(struct i2c_client *client);
static int32_t isl29011_write_reg(unsigned reg, uint8_t value, const char *caller);
static int32_t isl29011_read_reg(unsigned reg, uint8_t *value, const char *caller);
static int32_t isl29011_device_init(isl29011_data_t *dev);
static int32_t isl29011_fops_open (struct inode *inode, struct file *file);
static int32_t isl29011_fops_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int32_t isl29011_fops_release (struct inode *inode, struct file *file);
static void isl29011_report_adc_data(isl29011_data_t *dev);
static void isl29011_update_ls_threshold(isl29011_data_t *dev);
extern unsigned long msleep_interruptible(unsigned int);
static int32_t isl29011_get_sensor_data(isl29011_data_t *dev, uint16_t  *data);
#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static void isl29011_early_suspend (struct early_suspend *h);
static void isl29011_late_resume (struct early_suspend *h);
static struct early_suspend early_suspend_data = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 5,
    .suspend = isl29011_early_suspend,
    .resume = isl29011_late_resume,
};

#endif
static int32_t isl29011_suspend (struct i2c_client *client, pm_message_t mesg);
static int32_t isl29011_resume (struct i2c_client *client);
#endif



static isl29011_data_t *the_dev = NULL;
uint8_t  isl29011_light_sensor_schem = 0;
uint8_t  isl29011_prox_sensor_chem = 0;
uint8_t  isl29011_light_sensor_fsr = 0;
uint8_t  isl29011_prox_sensor_fsr  = 0;
static uint16_t isl29011_typ_adc_in_550_lux = ISL29011_TYP_ADC_IN_550_LUX;


static const struct i2c_device_id isl29011_id[] = {
    { ISL29011_NAME, 0 },
};

static uint16_t lux_val[ALS_LEVEL_END] = {
	DARK_LUX,
	INDOOR_WAK_LUX,
	INDOOR_NORMAL_LUX,
	INDOOR_STRONG_LUX,
	OUTDOOR_NORMAL_LUX,
};

static struct i2c_driver isl29011_driver = {
    .driver    =  {
        .name  = ISL29011_NAME,

    },
    .id_table = isl29011_id,    
    .probe     =  isl29011_probe,
    .remove    =  isl29011_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = isl29011_suspend,
    .resume     = isl29011_resume,
#endif    
};


static struct file_operations isl29011_fops =
{
   .owner   = THIS_MODULE,
   .open    = isl29011_fops_open,
   .ioctl   = isl29011_fops_ioctl,
   .release = isl29011_fops_release,
};


static struct miscdevice isl29011_miscdev =
{
   .fops  = &isl29011_fops,
   .minor = MISC_DYNAMIC_MINOR,
   .name  = ISL29011_NAME,
};

static int32_t isl29011_fops_open (struct inode *inode, struct file *file)
{
    if (!the_dev->initialized)
        return -ENODEV;
    file->private_data = the_dev;
    return 0;
}

static int32_t isl29011_fops_release (struct inode *inode, struct file *file)
{
    return 0;
}



static int32_t isl29011_fops_ioctl (struct inode *inode, struct file *file, uint32_t cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = 0;
    uint8_t mode = 0;
    uint16_t sensor_data;
    thld_range_t range;
    uint32_t switch_time = 0;
    isl29011_data_t *dev = file->private_data;
    isl29011_para_t para = {0};
    

    if (!dev)
        return -EFAULT;

    switch(cmd) {
        case LGS_IOCTL_APP_SET_MODE:
            if (copy_from_user(&mode, argp, sizeof(mode)))
                return -EFAULT;
            break;
        case LGS_IOCTL_APP_SET_LIGHT_RANGE:
        case LGS_IOCTL_APP_SET_PROXIMITY_RANGE:
            if (copy_from_user(&range, argp, sizeof(range)))
                return -EFAULT;
            break;
        case LGS_IOCTL_APP_SET_SWITCH_TIME:
            if (copy_from_user(&switch_time, argp, sizeof(switch_time)))
                return -EFAULT;
            break;
        case  LGS_IOCTL_APP_SET_ALL_PARA:
            if (copy_from_user(&para, argp, sizeof(isl29011_para_t)))
                return -EFAULT;        
            break;           
        case LGS_IOCTL_APP_GET_SENSOR_DET_DATA:
        case LGS_IOCTL_APP_GET_MODE:
            break;
        case LGS_IOCTL_APP_SET_FAC_ADC:
             if (copy_from_user(&dev->fac_adc,argp, sizeof(dev->fac_adc)))
                 return -EFAULT;
             if (dev->fac_adc != 0)
                 isl29011_update_ls_threshold(dev);
             return 0;
        default:
            return -EINVAL;
    }
   

    switch(cmd) {
        case LGS_IOCTL_APP_GET_SENSOR_DET_DATA:
            ret = isl29011_get_sensor_data(dev, &sensor_data);
            ISL29011_LOG("ISL29011 ioctl get sensor data 0x%x\n", sensor_data);
            copy_to_user((void *) argp, &sensor_data, 
                        sizeof(sensor_data));
            
            
            return 0;
        case LGS_IOCTL_APP_GET_MODE:
             ISL29011_LOCK();
             if (dev->switch_enabled == true)
                 mode = ISL29011_OP_ALS_PROX;
             else {
             	   mode = dev->op_mode;
             }
             ISL29011_UNLOCK();
             copy_to_user((void *) argp, &mode, sizeof(mode));
             return 0;
        case LGS_IOCTL_APP_SET_MODE:
            switch(mode) {
                case ISL29011_OP_IR_ONCE:
                case ISL29011_OP_IR_CON:
                     return -EINVAL;                            	
                case ISL29011_OP_ALS_ONCE:
                case ISL29011_OP_POWERDOWN:
                case ISL29011_OP_PROX_ONCE:
                case ISL29011_OP_ALS_CON:
                case ISL29011_OP_PROX_CON:
                    ISL29011_LOCK();
                    dev->op_mode = mode;
                    dev->switch_enabled = false;
                    ISL29011_UNLOCK();
                    ISL29011_LOG("isl29011 ioctl set mode 0x%x\n", dev->op_mode); 
		    /*If proxmity mode is enabled, we need lock it */
			if ((mode == ISL29011_OP_PROX_CON) ||
					(mode == ISL29011_OP_PROX_ONCE))
				wake_lock(&dev->wake_lock);
			else
				wake_unlock(&dev->wake_lock);
                    break;
                case ISL29011_OP_ALS_PROX:
                    ISL29011_LOCK();
                    dev->op_mode = ISL29011_OP_ALS_CON;
		    wake_lock(&dev->wake_lock);
                    ISL29011_LOG("isl29011 ioctl set mode ALS_PROX\n");
                    dev->switch_enabled = true;
                    ISL29011_UNLOCK();
            }
            break;
        case LGS_IOCTL_APP_SET_LIGHT_RANGE:
            ISL29011_LOCK();
            dev->light_range = range;
            ISL29011_UNLOCK();
            break;
        case LGS_IOCTL_APP_SET_PROXIMITY_RANGE:
            ISL29011_LOCK();
            dev->proxmity_range = range;
            ISL29011_UNLOCK();
            break;
        case LGS_IOCTL_APP_SET_SWITCH_TIME:

            dev->switch_mode_time =  ktime_set(0, switch_time);

            break;
        case LGS_IOCTL_APP_SET_ALL_PARA:
            ISL29011_LOCK();
            dev->op_mode = para.op_mode;
            dev->timing_mode = para.timing_mode;
            dev->int_persist = para.int_persist;
            dev->prox_schem = para.prox_schema;
            dev->mod_freq = para.mod_freq;
            dev->ir_current = para.ir_curr;
            dev->rsl_mode = para.resl_width;
            dev->fsr_mode = para.fs_range;
            if (dev->op_mode == ISL29011_OP_ALS_PROX) {
                dev->op_mode = ISL29011_OP_ALS_CON;
                dev->switch_enabled = true;                
            } else if (dev->op_mode == ISL29011_OP_ALS_CON ||dev->op_mode == ISL29011_OP_ALS_ONCE){
                dev->light_range.low_thld = para.low_thres;
                dev->light_range.hi_thld = para.hi_thres;
                isl29011_light_sensor_schem = para.prox_schema;
                isl29011_light_sensor_fsr = para.fs_range;
                dev->switch_enabled = false; 
            } else if(dev->op_mode == ISL29011_OP_PROX_ONCE ||dev->op_mode == ISL29011_OP_PROX_CON){
                dev->proxmity_range.low_thld = para.low_thres;
                dev->proxmity_range.hi_thld = para.hi_thres; 
                isl29011_prox_sensor_chem = para.prox_schema;
                isl29011_prox_sensor_fsr = para.fs_range;
                dev->switch_enabled = false; 
            } else if (dev->op_mode == ISL29011_OP_ALS_PROX){
                dev->switch_enabled = true;
                dev->light_range.low_thld = para.low_thres;
                dev->light_range.hi_thld = para.hi_thres;                 
                dev->op_mode = ISL29011_OP_ALS_ONCE;                    
            }
            ISL29011_UNLOCK();
            break;           

    }
    /* re-init device since some paramters are changed */
    isl29011_device_init(dev);
    return 0;
}

static void isl29011_update_ls_threshold (isl29011_data_t *dev)
{
	int32_t als_zone_level = 0;
	int32_t thld = 0;

	for (als_zone_level= 0; als_zone_level < ALS_LEVEL_END; als_zone_level++) {
		thld = als_zone[als_zone_level].low_thld;
		thld = thld * dev->fac_adc/isl29011_typ_adc_in_550_lux;
		/*update current light range*/
		if (dev->light_range.low_thld == als_zone[als_zone_level].low_thld)
			dev->light_range.low_thld = thld;
		als_zone[als_zone_level].low_thld = thld;
		thld = als_zone[als_zone_level].hi_thld;
		thld = thld * dev->fac_adc/isl29011_typ_adc_in_550_lux;
		/*update current light range*/
		if (dev->light_range.hi_thld == als_zone[als_zone_level].hi_thld)
			dev->light_range.hi_thld = thld;
		als_zone[als_zone_level].hi_thld = thld;
		ISL29011_LOG("ISL29011 update threshold %d, hi thld %d low thld %d\n",
				als_zone_level,
				als_zone[als_zone_level].low_thld,
				als_zone[als_zone_level].hi_thld);
	}
	isl29011_typ_adc_in_550_lux = dev->fac_adc;
	if (dev->switch_enabled == false) {
	isl29011_write_reg(ISL29011_INT_LT_LSB_REG,
				*((uint8_t *)&dev->light_range.low_thld),
				__FUNCTION__);
	isl29011_write_reg(ISL29011_INT_LT_MSB_REG,
				*((uint8_t *)&dev->light_range.low_thld+1),
				__FUNCTION__);
	isl29011_write_reg(ISL29011_INT_HT_LSB_REG,
				*((uint8_t *)&dev->light_range.hi_thld),
				__FUNCTION__);
	isl29011_write_reg(ISL29011_INT_HT_MSB_REG,
				*((uint8_t *)&dev->light_range.hi_thld+1),
				__FUNCTION__);
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void isl29011_early_suspend (struct early_suspend *h)
{
    isl29011_suspend (the_dev->client, PMSG_SUSPEND);
}

static void isl29011_late_resume (struct early_suspend *h)
{
    isl29011_resume (the_dev->client);
}
#endif

static int32_t  isl29011_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int32_t  ret = 0;
    isl29011_data_t *dev = (isl29011_data_t *)i2c_get_clientdata(client);
    /*power off the device*/
    ISL29011_LOG("ISL29011 starting suspend\n");
	if ((dev->switch_enabled) == true) {
		ISL29011_LOG("ISL29011 doesn't suspend in proximity mode\n");
		return ret;
	}
    ISL29011_LOCK();
    isl29011_write_reg(ISL29011_COMMAND_I_REG, ISL29011_OP_POWERDOWN, __FUNCTION__);
    ISL29011_UNLOCK();
    return ret;
}

static int32_t  isl29011_resume (struct i2c_client *client)
{
	int32_t  ret = 0;
	isl29011_data_t *dev = (isl29011_data_t *)i2c_get_clientdata(client);
	ISL29011_LOG("ISL29011 starting resume\n");
	/*power on the device*/
	if (dev->switch_enabled == true) {
		ISL29011_LOG("ISL29011 doesn't resume in proximity mode\n");
		return ret;
	}
	isl29011_device_init(dev);
	isl29011_report_adc_data(dev);
	return ret;
}

static void isl29011_report_adc_data(isl29011_data_t *dev)
{
	uint16_t data = 0;
	int32_t level = 0;

	ISL29011_LOCK();
	isl29011_read_reg(ISL29011_DATA_LSB_REG, (uint8_t *)(&data),
			__func__);
	isl29011_read_reg(ISL29011_DATA_MSB_REG, ((uint8_t *)&data+1),
			__func__);
	if (dev->op_mode == ISL29011_OP_ALS_ONCE ||
			dev->op_mode == ISL29011_OP_ALS_CON) {
		for (level = 0; level < ALS_LEVEL_END;
				level++) {
			if (als_zone[level].low_thld <= data &&
					data <= als_zone[level].hi_thld)
				break;
		}
		if (ALS_LEVEL_END == level) {
			printk(KERN_ERR "%s:ISL29011 error als zone 0x%x\n",
					__func__, data);
		} else {
			input_event(dev->idev, EV_LED, LED_MISC,
					lux_val[level]);
			input_sync(dev->idev);
			ISL29011_LOG("Light sensor report lux level %d\n",
					level);
		}
	}
	ISL29011_UNLOCK();
	return;
}

/*write i2c reg*/
static  int32_t  isl29011_write_reg(unsigned reg, uint8_t value, const char *caller)
{
    uint8_t buf[2] = {reg, value};
    int32_t  ret = 0;
    struct i2c_client *client = the_dev->client;

    ISL29011_LOG("%s: writing 0x%x to reg 0x%x  at addr 0x%x\n",
        caller, buf[1], buf[0],client->addr);
    ret = i2c_master_send (client, buf, 2);
    if (ret < 0)
        printk(KERN_ERR "%s: i2c_master_send error %d\n",
                caller, ret);
    return ret;
}

static int32_t  isl29011_read_reg(unsigned reg, uint8_t *value, const char *caller)
{
    struct i2c_client *client = the_dev->client;
    uint8_t buf[1]={0};
    int32_t  ret =0;

    if (!value)
        return -EINVAL;
    ISL29011_LOG("%s: reading from reg 0x%X at addr 0x%X\n",
            caller, reg, client->addr);
    buf[0] = reg;
    ret = i2c_master_send(client, buf, 1);
    if (ret > 0) {
        msleep_interruptible (1);
        ret = i2c_master_recv (client, buf, 1);
        if (ret > 0){
            *value = buf[0];
            ISL29011_LOG("%s: read reg successfully. value is 0x%x\n", caller, *value);
        }
    }
    return ret;    
}
static int32_t  isl29011_get_sensor_data(isl29011_data_t *dev, uint16_t *data)
{
    ISL29011_LOCK();

    isl29011_read_reg(ISL29011_DATA_LSB_REG,(uint8_t *)(data),__FUNCTION__);
    isl29011_read_reg(ISL29011_DATA_MSB_REG,((uint8_t *)data+1), __FUNCTION__);

    ISL29011_UNLOCK();

    return 0;
}

/*register input device, init device*/
static uint8_t isl29011_register(isl29011_data_t *dev)
{
   struct input_dev *input_dev = NULL;
   int8_t ret = 0;
   /*alloc and register input device for proximity detection*/
   input_dev = input_allocate_device();
   if (!input_dev) {
   	   printk(KERN_ERR "isl29011: failed to register input device\n");
	   ret = -ENOMEM;
	   goto exit_input_dev_alloc_failed;
   }
   input_dev->name = ISL29011_MODULE_NAME;
   set_bit(EV_ABS, input_dev->evbit);
   set_bit(EV_LED, input_dev->evbit);
   set_bit(ABS_DISTANCE, input_dev->absbit);
   set_bit(LED_MISC, input_dev->ledbit);

   dev->idev = input_dev;

   ret = input_register_device(dev->idev);
   if (ret != 0) {
   	   printk(KERN_ERR "isl29011:register input device failed\n");
	   goto exit_input_dev_register_failed;
   }
   return 0;
   /*Need to done, misc driver register for isl29011, since we need control the chip*/
exit_input_dev_register_failed:
      input_free_device(dev->idev);
      dev->idev = NULL;
exit_input_dev_alloc_failed:

   return ret;

}

static void isl29011_unregister(isl29011_data_t *dev)
{
   if (dev && dev->idev) {
      input_unregister_device(dev->idev);
      input_free_device(dev->idev);
      dev->idev = NULL;   
   }
}


/*switch operation mode*/
static enum hrtimer_restart isl29011_switch_mode_handle(struct hrtimer *data)
{
   isl29011_data_t *tdev = container_of(data,isl29011_data_t, switch_mode_timer);

   schedule_work(&tdev->switch_mode_wq);    
   return 0;
}


irqreturn_t isl29011_irq_handler (int irq, void *dev)
{
    isl29011_data_t *tdev = (isl29011_data_t *)dev;
	

    /*cancel switch mode timer, we need start switch mode
     *after irq handle finished
     */
    if (tdev->switch_enabled == true) {
        ISL29011_LOG("%s cancel switch timer\n",__FUNCTION__);
        hrtimer_cancel(&tdev->switch_mode_timer);
    }
    disable_irq(tdev->client->irq);
    
    schedule_work(&tdev->int_wq);
    return IRQ_HANDLED;

}

/*handle switch mode here*/
static void switch_mode_work_func(struct work_struct *work)
{
    isl29011_data_t *dev = container_of(work, isl29011_data_t, switch_mode_wq);   

    if (dev->switch_enabled == false)
        return;    
    ISL29011_LOG("isl29011 switch mode,current mode %x\n", dev->op_mode);
    ISL29011_LOCK();
    switch(dev->op_mode) {
       case ISL29011_OP_ALS_ONCE: 
       case ISL29011_OP_ALS_CON:
        dev->op_mode = ISL29011_OP_PROX_CON;
        break;
       case ISL29011_OP_PROX_ONCE:
       case ISL29011_OP_PROX_CON:
        dev->op_mode = ISL29011_OP_ALS_CON;
        break;
       default:
        ISL29011_UNLOCK();
       return;

    }
    ISL29011_UNLOCK();
    isl29011_device_init(dev);
    return;
}
void isl29011_post_data (struct input_dev *idev,uint16_t data)
{
   int32_t  distance = 0;

   if (data > ISL29011_PROX_START_HI_THRESHOLD) {
     distance = ISL29011_PROXIMITY_NEAR;
   } else {
     distance = ISL29011_PROXIMITY_FAR;
   }
    
   ISL29011_LOG("isl29011: presence [0x%x], data is 0x%x\n", distance,data);
   input_report_abs(idev, ABS_DISTANCE, distance);
   input_sync(idev); 
}

/*we actual handle irq here*/
static void int_work_func(struct work_struct *work)
{
   unsigned long irq_flags;
   uint16_t data = 0;
   int32_t  ret = 0;
   isl29011_data_t *dev = container_of(work, isl29011_data_t, int_wq);   
   int32_t  als_zone_level = 0;

   ISL29011_LOCK();

   ret = isl29011_read_reg(ISL29011_DATA_LSB_REG,(uint8_t *)(&data),__FUNCTION__);
   ret = isl29011_read_reg(ISL29011_DATA_MSB_REG,((uint8_t *)&data+1), __FUNCTION__);
   

   ISL29011_LOG("isl29011 interrupt:%s mode 0x%x, read data 0x%x\n", __FUNCTION__, dev->op_mode, data);

   
   /*read data register*/
   switch(dev->op_mode&0xEF) {
     case ISL29011_OP_ALS_ONCE:   
     case ISL29011_OP_ALS_CON:         
        ISL29011_LOG("interrupt light:high %x, low %x\n", dev->light_range.hi_thld, dev->light_range.low_thld);
        for (als_zone_level= 0; als_zone_level < ALS_LEVEL_END; als_zone_level++) {
            if ( als_zone[als_zone_level].low_thld <= data &&
			data <= als_zone[als_zone_level].hi_thld )
                break;
        }
        if (ALS_LEVEL_END == als_zone_level) {
            printk(KERN_ERR "%s:ISL29011 detect error als zone 0x%x\n", __FUNCTION__, data);
            break;
        }
        dev->light_range.low_thld = als_zone[als_zone_level].low_thld;
        dev->light_range.hi_thld = als_zone[als_zone_level].hi_thld;
        /*Need handle light interrrupt here*/
        input_event(dev->idev,EV_LED, LED_MISC, lux_val[als_zone_level]);
        input_sync(dev->idev);
        isl29011_write_reg(ISL29011_INT_LT_LSB_REG, *((uint8_t *)&dev->light_range.low_thld), __FUNCTION__);    
        isl29011_write_reg(ISL29011_INT_LT_MSB_REG, *((uint8_t *)&dev->light_range.low_thld+1), __FUNCTION__);      
        isl29011_write_reg(ISL29011_INT_HT_LSB_REG, *((uint8_t *)&dev->light_range.hi_thld), __FUNCTION__);     
        isl29011_write_reg(ISL29011_INT_HT_MSB_REG, *((uint8_t *)&dev->light_range.hi_thld+1), __FUNCTION__); 
        
        break;
     case ISL29011_OP_PROX_ONCE:
     case ISL29011_OP_PROX_CON:
        /*report event to user space*/   
        
        if (data >= ISL29011_PROX_START_HI_THRESHOLD){
		dev->proxmity_range.hi_thld = 2047;
		dev->proxmity_range.low_thld = ISL29011_PROX_START_HI_THRESHOLD;
        } else {
		dev->proxmity_range.hi_thld = ISL29011_PROX_START_HI_THRESHOLD;
		dev->proxmity_range.low_thld = 0;
        }
        isl29011_post_data(dev->idev, data);
        isl29011_write_reg(ISL29011_INT_LT_LSB_REG,
				*((uint8_t *)&dev->proxmity_range.low_thld),
				__FUNCTION__);
        isl29011_write_reg(ISL29011_INT_LT_MSB_REG,
				*((uint8_t *)&dev->proxmity_range.low_thld+1),
				__FUNCTION__);
        isl29011_write_reg(ISL29011_INT_HT_LSB_REG,
				*((uint8_t *)&dev->proxmity_range.hi_thld),
				__FUNCTION__);
        isl29011_write_reg(ISL29011_INT_HT_MSB_REG,
				*((uint8_t *)&dev->proxmity_range.hi_thld+1),
				__FUNCTION__);
	ISL29011_LOG("Proximity interrupt:value :%d,newhigh thre:%d, new low:%d",
			data,
			dev->proxmity_range.hi_thld,
			dev->proxmity_range.low_thld);
        break;
     default:
        printk(KERN_ERR "%s:ISL29011 in error mode %d\n", __FUNCTION__, dev->op_mode);
   }
   /*clear irq flag*/
   ret = isl29011_read_reg(ISL29011_COMMAND_I_REG,(uint8_t *)(&data),__FUNCTION__);
     
   /* restart switch mode timer*/
   if (dev->switch_enabled == true) {
    hrtimer_start(&dev->switch_mode_timer, dev->switch_mode_time,HRTIMER_MODE_REL);
  }
   ISL29011_UNLOCK();  
   local_irq_save(irq_flags);
   enable_irq(dev->client->irq);
   local_irq_restore(irq_flags);

}

static int32_t  isl29011_device_init(isl29011_data_t *dev)
{
    int32_t  ret = 0;
    uint8_t data = 0;    
    struct thld_range *thld;

    ISL29011_LOCK(); 
    /*write command operation mode, other should be default*/
    isl29011_write_reg(ISL29011_COMMAND_I_REG, dev->op_mode|dev->timing_mode|dev->int_persist, __FUNCTION__);  

    if ((dev->op_mode == ISL29011_OP_PROX_ONCE) || (dev->op_mode == ISL29011_OP_PROX_CON)) {
       thld = &dev->proxmity_range;
       dev->prox_schem = isl29011_prox_sensor_chem;
       dev->fsr_mode = isl29011_prox_sensor_fsr;
    }
    else if ((dev->op_mode == ISL29011_OP_ALS_ONCE) ||(dev->op_mode == ISL29011_OP_ALS_CON)) {
       thld = &dev->light_range;
       dev->prox_schem = isl29011_light_sensor_schem;
       dev->fsr_mode = isl29011_light_sensor_fsr;
     }
    else /*power down or IR detect, we don't support IR detect now*/
        goto exit;
       
    /*set threshold value*/
    isl29011_write_reg(ISL29011_INT_LT_LSB_REG, *((uint8_t *)&thld->low_thld), __FUNCTION__);
    ISL29011_LOG("isl29011 write 0x%x to reg %x\n", *((uint8_t *)&thld->low_thld), ISL29011_INT_LT_LSB_REG);
    isl29011_write_reg(ISL29011_INT_LT_MSB_REG, *((uint8_t *)&thld->low_thld+1), __FUNCTION__);
    ISL29011_LOG("isl29011 write 0x%x to reg %x\n", *((uint8_t *)&thld->low_thld+1),ISL29011_INT_LT_MSB_REG);
    isl29011_write_reg(ISL29011_INT_HT_LSB_REG, *((uint8_t *)&thld->hi_thld), __FUNCTION__); 
    ISL29011_LOG("isl29011 write 0x%x to reg %x\n", *((uint8_t *)&thld->hi_thld), ISL29011_INT_HT_LSB_REG);    
    isl29011_write_reg(ISL29011_INT_HT_MSB_REG, *((uint8_t *)&thld->hi_thld+1), __FUNCTION__); 
    ISL29011_LOG("isl29011 write 0x%x to reg %x\n", *((uint8_t *)&thld->hi_thld+1), ISL29011_INT_HT_MSB_REG);            
    /*write command 2*/   
    data = dev->rsl_mode |dev->fsr_mode|dev->prox_schem|dev->ir_current|dev->mod_freq;
    ret = isl29011_write_reg(ISL29011_COMMAND_II_REG, data, __FUNCTION__);
    if (dev->switch_enabled == true) {
      hrtimer_start(&dev->switch_mode_timer, dev->switch_mode_time,HRTIMER_MODE_REL);
    } else {
      hrtimer_try_to_cancel(&dev->switch_mode_timer);
    }
exit:    
    ISL29011_UNLOCK();
    return ret;  
   
}


static int32_t  isl29011_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int32_t  ret = 0;
	unsigned int  request_flags =  IRQF_TRIGGER_LOW;
	int32_t  switch_time = ISL29011_SWITCH_MODE_TIME;

	printk(KERN_INFO"Light/Proximity sensor:Registering sensor driver\n");
	ISL29011_LOG ("%s:  I2C address = 0x%x, flags = 0x%x\n",
			__FUNCTION__,  client->addr, client->flags);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk (KERN_ERR "%s: I2C_FUNC_I2C not supported\n",
			__FUNCTION__);
	return -ENOTSUPP;
	}
	the_dev = kzalloc(sizeof(*the_dev),GFP_KERNEL);
	if (!the_dev)
		return -ENOMEM;
	the_dev->client = client;
	i2c_set_clientdata (client, the_dev);
	the_dev->switch_mode_time = ktime_set( switch_time/MSEC_PER_SEC,
						switch_time%MSEC_PER_SEC*NSEC_PER_MSEC);
	the_dev->light_range.hi_thld = als_zone[ALS_DARK].hi_thld;
	the_dev->light_range.low_thld = als_zone[ALS_DARK].low_thld;
	the_dev->proxmity_range.hi_thld = ISL29011_PROX_START_HI_THRESHOLD;
	the_dev->proxmity_range.low_thld = ISL29011_PROX_START_LO_THRESHOLD;
	the_dev->op_mode = ISL29011_OP_ALS_CON;
	the_dev->timing_mode = ISL29011_TM_INTERNAL;
	the_dev->int_persist = ISL29011_INT_PERSIST_4;
	the_dev->ir_current = ISL29011_IR_CUR_50;
	the_dev->rsl_mode = ISL29011_RSLT_WIDTH_12;
	the_dev->fsr_mode = ISL29011_FSR_WIDTH_1K;
	the_dev->mod_freq = ISL29011_MODULAR_FREQ_327_7;
	/*
         * hardware require use difference shem and full scale
         * range between proxmity sensor and light sensor
         */
	isl29011_light_sensor_schem = ISL29011_PROX_SCHEMA_LED_AMB;
	isl29011_prox_sensor_chem = ISL29011_PROX_SCHEMA_LED_ONLY;
	isl29011_light_sensor_fsr = ISL29011_FSR_WIDTH_1K;
	isl29011_prox_sensor_fsr  = ISL29011_FSR_WIDTH_4K;
	the_dev->switch_enabled = false;
	ret = isl29011_register(the_dev);
	wake_lock_init(&the_dev->wake_lock, WAKE_LOCK_SUSPEND, "isl29011");

	if (ret != 0)
		goto err_register_device;

	INIT_WORK (&the_dev->int_wq, int_work_func);
	INIT_WORK (&the_dev->switch_mode_wq, switch_mode_work_func);
	if (client->irq) {
		ret = request_irq(client->irq, isl29011_irq_handler,
					request_flags, "isl29011",
					the_dev);
		if (ret != 0) {
			printk(KERN_ERR "ISL29011 request irq %d failed.\n",
				client->irq);
			goto err_request_irq;
		}
	}
		
	hrtimer_init(&the_dev->switch_mode_timer, CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
	the_dev->switch_mode_timer.function = isl29011_switch_mode_handle;
    
	/*start switch mode*/
	isl29011_device_init(the_dev);
	the_dev->initialized =  1;
	ret = misc_register (&isl29011_miscdev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend (&early_suspend_data);
#endif    
	if (ret != 0)
		goto err_misc_driver_register;
	return 0;
err_misc_driver_register:
	isl29011_unregister(the_dev);
err_request_irq:
	free_irq(client->irq, the_dev);
err_register_device:
	kfree(the_dev);
	return ret;    
}

static int32_t  isl29011_remove(struct i2c_client *client)
{
   isl29011_data_t *dev = i2c_get_clientdata(client);

   if (client->irq)
      free_irq(client->irq, dev);
   isl29011_unregister(dev);  
   if (dev)
      kfree(dev); 
   misc_deregister(&isl29011_miscdev);
   return 0;
}


static int32_t  __devinit isl29011_init(void)
{
  int32_t  ret = 0;

  ret = i2c_add_driver (&isl29011_driver);  
  if (ret) {
        printk (KERN_ERR "%s: i2c_add_driver failed, error %d\n", 
            __FUNCTION__, ret);
  }
  return ret;
}

static void __exit isl29011_exit(void)
{
   i2c_del_driver(&isl29011_driver);  
}

module_init(isl29011_init);
module_exit(isl29011_exit);

MODULE_DESCRIPTION("ISL29011 LIGHT/PROXIMITY SENSOR DRIVER");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL v2");




