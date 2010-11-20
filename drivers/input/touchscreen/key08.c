/* drivers/input/keyboard/key08_touch.c
 *
 * Copyright (C) 2008 Motorola Inc.
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

#include <stdarg.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/uaccess.h>


#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "key08.h"
#ifndef CONFIG_MACH_MOT
#include <mach/board.h>
#include <../board-mot.h>
#endif


#define	IRQ_REASON_IRQ		1
#define	IRQ_REASON_SLIDER	2
#define	IRQ_REASON_SUSPEND	3
#define	IRQ_REASON_MANUAL	4

#ifndef CONFIG_MACH_MOT

/* These values come from the /system/usr/keylayout/qwerty.kl file */
#define QWERTY_KL_MENU_KEY       139
#define QWERTY_KL_HOME_KEY       102
#define QWERTY_KL_BACK_KEY       158

#define KEY08_STATUS_MSG_STATUS_BYTE 1
#define KEY08_KEY_MSG_DATA_BYTES 2

#define KEY08_KEY_MSG_TSB_0      0
#define KEY08_KEY_MSG_TSB_1      1
#define KEY08_KEY_MSG_TSB_2      2
#define KEY08_KEY_MSG_TSB_3      3
#define KEY08_KEY_MSG_TSB_4      4
#define KEY08_KEY_MSG_TSB_5      5
#define KEY08_KEY_MSG_TSB_6      6
#define KEY08_KEY_MSG_TSB_7      7
#define KEY08_KEY_MSG_TSB_8      8
#define KEY08_KEY_MSG_TSB_9      9
#define KEY08_KEY_MSG_TSB_10     10
#define KEY08_KEY_MSG_TSB_11     11
#define KEY08_KEY_MSG_TSB_12     12
#define KEY08_KEY_MSG_TSB_13     13
#define KEY08_KEY_MSG_TSB_14     14
#define KEY08_KEY_MSG_TSB_15     15
#define KEY08_KEY_MSG_TSB_16     16
#define KEY08_KEY_MSG_TSB_17     17
#define KEY08_KEY_MSG_TSB_18     18
#define KEY08_KEY_MSG_TSB_19     19
#define KEY08_KEY_MSG_TSB_20     20
#define KEY08_KEY_MSG_TSB_21     21
#define KEY08_KEY_MSG_TSB_22     22
#define KEY08_KEY_MSG_TSB_23     23
#define KEY08_KEY_MSG_TSB_24     24
#define KEY08_KEY_MSG_TSB_25     25
#define KEY08_KEY_MSG_TSB_26     26
#define KEY08_KEY_MSG_TSB_27     27
#define KEY08_KEY_MSG_TSB_28     28
#define KEY08_KEY_MSG_TSB_29     29
#define KEY08_KEY_MSG_TSB_30     30
#define KEY08_KEY_MSG_TSB_31     31

#define KEY08_TSB_UNTOUCHED      0  /* No TSBs touched */
#define KEY08_TSB_IN_DEBOUNCE    1  /* TSB not touched long enough yet */
#define KEY08_TSB_DEBOUNCED      2  /* TSB touched, sending Key Press msg */
#define KEY08_TSB_PRESSED        3  /* Key Press msg sent */

#define KEY08_KEY_MSG_TSB_ACTIVE(m, t) \
    ((*((uint8_t *)m + KEY08_KEY_MSG_DATA_BYTES + t/8)) & (0x1u << t % 8))

#define KEY08_CALIBRATE_TIMEOUT  225 /* ms */
#define KEY08_STATUS_MSG_IN_CAL(m) \
    ((*((uint8_t *)m + KEY08_STATUS_MSG_STATUS_BYTE)) & (0x1u << 4))
#endif
#define KEY08_PRINTK(args...) key08_printk(args);

extern unsigned long msleep_interruptible(unsigned int);
#ifndef CONFIG_MACH_MOT
extern void vibrator_activate (int value);
#endif

#define clk_busy_wait(time)	msleep_interruptible(time/1000)

/*#define swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0) */
#ifdef CONFIG_MACH_MOT
#define KEY08_TS_GPIO   41
/* #define gpio_to_irq( gpio)  (gpio+64) */
#else
static uint8_t KEY08_TSB_VIB_MS = 23;
static uint16_t KEY08_TSB_HOLD_MS = 150;
#define KEY08_TS_GPIO      TOUCH_INT_N
#endif

#define KEY08_I2C_NAME "key08"
// #define KEY08_TS_I2C_ADDRESS  0x54
// #define KEY08_TS_I2C_ADDRESS  0x11
#define KEY08_REG_ADDR_SIZE    2
//static unsigned short normal_i2c[] = { KEY08_TS_I2C_ADDRESS, I2C_CLIENT_END};

#define KEY08_BL_I2C_NAME "touchpad_bl"
// #define KEY08_BL_I2C_ADDRESS	0x25
// #define	KEY08_BL_I2C_ADDRESS	0x5F
// static unsigned short bootloader_i2c[] = { KEY08_BL_I2C_ADDRESS, I2C_CLIENT_END};

/* Insmod parameters */
// I2C_CLIENT_INSMOD_1(key08_touch);

enum 
{
	KEY08_FLIP_X = 1UL << 0,
	KEY08_FLIP_Y = 1UL << 1,
	KEY08_SWAP_XY = 1UL << 2,
	KEY08_SNAP_TO_INACTIVE_EDGE = 1UL << 3,
};

struct key08_i2c_platform_data {
	int (*power)(int on);
	uint32_t flags;
	uint32_t inactive_left; /* 0x10000 = screen width */
	uint32_t inactive_right; /* 0x10000 = screen width */
	uint32_t inactive_top; /* 0x10000 = screen height */
	uint32_t inactive_bottom; /* 0x10000 = screen height */
	uint32_t fuzz_x; /* 0x10000 = screen width */
	uint32_t fuzz_y; /* 0x10000 = screen height */
	int fuzz_p;
	int fuzz_w;
};

struct key08_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *sw_dev;
	struct hrtimer timer;
	struct work_struct  work;
	struct work_struct  workSlider;
	uint32_t flags;
	uint16_t max[2];
	uint16_t addr;
	uint8_t	XferStatus;
	int		irqReason;
	int use_irq;
	unsigned char	irqCount;
	int snap_down[2];
	int snap_up[2];
	char	mode;
	char	sendMode;
	char calibrate;
	bool	inCall;
	bool	suspendMode;
	bool	touch_reported[NUMBER_OF_FINGERS];
	int		gMinx;
	int		gMaxx;
	int		gMiny;
	int		gMaxy;
	int		area;
	int (*power)(int on);
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
#ifndef CONFIG_MACH_MOT
	unsigned int calibrate_ms;
	bool calibrate_msg;
	uint8_t  tsbState;
	uint16_t tsbTouched;
	struct hrtimer tsbTimer;
#endif
};

A_TOUCH_POINT	currPoints[NUMBER_OF_FINGERS];
unsigned	char	configRegs[1024];

static void key08_printk (char *, ...);
static void key08_doReset(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void key08_ts_early_suspend(struct early_suspend *h);
static void key08_ts_late_resume(struct early_suspend *h);
#endif
static int key08_ts_remove(struct i2c_client *);
/*
static int key08_ts_probe(struct i2c_adapter *adapter, int address, int kind);
*/
static int key08_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);

static const struct i2c_device_id key08_ts_id[] = {
	{ KEY08_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver key08_ts_driver = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= key08_ts_suspend,
	.resume		= key08_ts_resume,
#endif
/*
	.attach_adapter	= key08_ts_probe,
	.detach_client	= key08_ts_remove,
*/
	.probe	= key08_ts_probe,
	.remove	= key08_ts_remove,
	.id_table	= key08_ts_id,
	.driver = {
		.name	= KEY08_I2C_NAME,
	},
};

static void key08_slider_event(struct input_handle *, unsigned int, unsigned int, int );
static int key08_slider_connect(struct input_handler *, struct input_dev *, const struct input_device_id *);
static void key08_slider_disconnect(struct input_handle *);

static const struct input_device_id slider_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_SW) },
	},
	{ },
};

MODULE_DEVICE_TABLE(input,slider_ids);

static struct input_handler slider_handler = {
	.event =	key08_slider_event,
	.connect = 	key08_slider_connect,
	.disconnect = 	key08_slider_disconnect,
	.name	= "key08_slider",
	.id_table = slider_ids,
};
static	int	key08_open(struct inode *inode, struct file *filp);
static int key08_ioctl(struct inode *node, struct file *filp, unsigned int cmd, unsigned long arg);
// static int key08_release(struct inode *inode, struct file *filp);
static int key08_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos );
struct	file_operations	key08_fops =
{
	.owner		= THIS_MODULE,
	.open		= key08_open,
	.ioctl		= key08_ioctl,
//	.release	= key08_release,
	.write		= key08_write,
};
static struct miscdevice key08_pf_driver = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = KEY08_PF_NAME,
	.fops = &key08_fops,
};
typedef struct  
{
	uint16_t reg;
	uint8_t  data[4]; 
} KEY08_INIT_DATA_T;

int pos[2][2];
struct key08_ts_data *tsGl;
int	debugOn;


int	key08_normal_addr;
int	key08_bl_addr;
static	char	lpValue;
static	int	prevX;
static	int	prevY;
static	int	threshX;
static	int	threshY;
#ifdef CONFIG_MACH_MOT
int     lastKeyPressed;
#endif
#define	DBG_BUFFER_SIZE	150
char	dbgBuffer[DBG_BUFFER_SIZE];

static	char	sliderEvent = FALSE;
static	int	obj[NUMBER_OF_FINGERS];
static	int	tf[NUMBER_OF_FINGERS];
static long timsPress[NUMBER_OF_FINGERS];

void	key08_disable_irq(struct key08_ts_data *ts)
{
	if ( ts->irqCount > 0 )
	{
		ts->irqCount--;
		disable_irq(ts->client->irq);
		KEY08_PRINTK("%s: Disabling irq %d (count: %d)\n", __FUNCTION__, ts->client->irq,tsGl->irqCount);
	}
	else
		KEY08_PRINTK("%s: irq %d is already disabled (count: %d)\n", __FUNCTION__, ts->client->irq, ts->irqCount);
	return;
}

void	key08_enable_irq(struct key08_ts_data *ts)
{
	if ( ts->irqCount < 1 )
	{
		ts->irqCount++;
		enable_irq(ts->client->irq);
		KEY08_PRINTK("%s: Enabling irq %d (count: %d)\n", __FUNCTION__, ts->client->irq,ts->irqCount);
	}
	else
		KEY08_PRINTK("%s: irq %d is already enabled (count: %d)\n", __FUNCTION__, ts->client->irq, ts->irqCount);
		
	return;
}


int key08_reg_write (struct key08_ts_data *ts, unsigned int reg, unsigned int length, unsigned char *reg_value)
{
	unsigned char value[24];
	int retval = 0;
	int i = 10;
	int	j;

	KEY08_PRINTK("%s: Enter\n",__FUNCTION__);

	if ( tsGl->mode == KEY08_MODE_NORMAL )
	{
		KEY08_PRINTK("%s: call: 0x%x, %u, %u, 0x%x: %d. Slider event: %s\n",__FUNCTION__,
						ts->client, reg, length, reg_value, *reg_value,
						(sliderEvent) ? "True" : "False" );
	/*
		if ( sliderEvent )
			return retval;
	*/
		/* Copy the data into a buffer for correct format */
		value[0] = reg & 0xFF;
		value[1] = reg >> 8;
		
		// memcpy(&value[2], reg_value, length);
		for ( i = 2, j = 0; j < length; i++, j++ )
		{
			value[i] = reg_value[j];
		}

		/* Write the data to the device (retrying if necessary) */
		i = 10;
		do
		{
			KEY08_PRINTK("%s: trying to write to 0x%x\n", __FUNCTION__,ts->client->addr);
			retval = i2c_master_send (ts->client, value, KEY08_REG_ADDR_SIZE + length);

			/* On failure, output the error code and delay before trying again */
			if (retval < 0)
			{
				KEY08_PRINTK(KERN_ERR "key08_reg_write: write of reg 0x%X failed: %d\n", reg, retval);
				clk_busy_wait(3000);
			}
			else
				KEY08_PRINTK("key08_reg_write: i2c_master_send(): OK\n");
		}
		while ((retval < 0) && (i-- >= 0));

		KEY08_PRINTK("key08_reg_write: i2c_master_send(): Done\n");

		/* On success, set retval to 0 (i2c_master_send returns no. of bytes transfered) */
		if (retval == (KEY08_REG_ADDR_SIZE + length))
		{
			retval = 0;
		}

		/* Delay after every I2C access or IC will NAK */
		clk_busy_wait(3000);
	}
	else
		KEY08_PRINTK("%s: Could not write register. Current mode is not NORMAL.\n",__FUNCTION__);
	
	KEY08_PRINTK("key08_reg_write: Exit\n");
	return retval;
}

static int8_t key08_ts_read_4_regs(struct key08_ts_data *ts , A_REG *rgs)
{
	uint8_t data[6];
	int8_t rc = 0;
	int	gpioStatus;

	KEY08_PRINTK("%s: Enter\n",__FUNCTION__);
	if ( ts->mode == KEY08_MODE_NORMAL )
	{
		key08_disable_irq(ts);
		data[0] = 0x5;  //read register flag
		data[1] = 0;   // register id
		data[2] = rgs->reg>>2;  //read register flag

		gpioStatus = gpio_get_value(KEY08_TS_GPIO);
		key08_printk("%s: GPIO is %s", __FUNCTION__, (gpioStatus == 1) ? "HIGH\n":"LOW\n" );
		/* If status is low, then there is data to read */
		while ( gpioStatus == 0 )
		{
			i2c_master_recv(ts->client, data, 6 );
			gpioStatus = gpio_get_value(KEY08_TS_GPIO);
		}

		/* At this point, there is no more data in the i2c */
		if ((rc = i2c_master_send(ts->client, data, 3)) < 0)
		{
			KEY08_PRINTK(KERN_ERR "%s error  %d\n,",__FUNCTION__,rc);
		}
		clk_busy_wait(3000);
		

		gpioStatus = gpio_get_value(KEY08_TS_GPIO);
		while ( gpioStatus == 0 )
		{
			data[0] = 0;
			if ((rc = i2c_master_recv(ts->client, data, 6 )) < 0 )
			{
				key08_printk("%s: Failed reading register values\n",__FUNCTION__);
				rc = -1;
			}

			if ( data[0] == 0xFF )
			{
				rc = -1;
				break;
			}
			KEY08_PRINTK("%s: %02x %02x %02x %02x %02x %02x \n",__FUNCTION__, 
				data[0],
				data[1],
				data[2],
				data[3],
				data[4],
				data[5]
            );

			if ((data[0] == 0x80) && (data[1] == (rgs->reg>>2)))
			{
				break;
			}
			gpioStatus = gpio_get_value(KEY08_TS_GPIO);
		}
		key08_printk("%s: GPIO is %s", __FUNCTION__, (gpioStatus == 1) ? "HIGH\n":"LOW\n" );

		key08_enable_irq(ts);

		rgs->vals[0] = data[2+(rgs->reg%4)];
		rgs->vals[1] = data[2+(rgs->reg%4)+1];
		rgs->vals[2] = data[2+(rgs->reg%4)+2];
		rgs->vals[3] = data[2+(rgs->reg%4)+3];
		KEY08_PRINTK("%s: Exit(rc=%d)\n",__FUNCTION__, rc);
		return rc;
	}
	else
	{
		// in BOOTLOADER mode it will always return 0
		return rc;
	}
}
static uint8_t key08_ts_read_register(struct key08_ts_data *ts , int reg )
{
	uint8_t data[6];
	int rc = 0;
	int	i;

	KEY08_PRINTK("%s: Enter\n",__FUNCTION__);
	if ( ts->mode == KEY08_MODE_NORMAL )
	{
		data[0] = 0x5;  //read register flag
		data[1] = 0;   // register id
		data[2] = reg>>2;  //read register flag

		key08_disable_irq(ts);

		i2c_master_send(ts->client, data, 3);
		clk_busy_wait(3000);
		

		for(i=0; i<1000; i++)
		{

			data[0] = 0;
			if ((rc = i2c_master_recv(ts->client, data, 6 )) < 0 )
			{
				KEY08_PRINTK("%s: Could not read register in non-NORMAL mode\n",__FUNCTION__);
				rc = -1;
			}

			KEY08_PRINTK("%s: %02x %02x %02x %02x %02x %02x \n",__FUNCTION__, 
				data[0],
				data[1],
				data[2],
				data[3],
				data[4],
				data[5]
            );

			if ((data[0] == 0x80) && (data[1] == (reg>>2)))
			{
				break;
			}
		}

		key08_enable_irq(ts);

		KEY08_PRINTK("%s: Exit(rc=%d)\n",__FUNCTION__, rc);
		if ( rc > 0 )
			return(data[2+(reg%4)]);
		else
			return rc;
	}
	else
	{
		// in BOOTLOADER mode it will always return 0
		return rc;
	}
}

static uint16_t key08_ts_read_register16(struct  key08_ts_data *ts , int reg )
{
	uint8_t data[6];
	int   rc;
    int i;

    // make sure reg is even address
    if(reg&1) reg--;

	// Read  
	data[0] = 0x5;  //read register flag
	data[1] = 0;   // register id
	data[2] = reg>>2;  //read register flag

	KEY08_PRINTK("%s: Enter\n",__FUNCTION__);

	key08_disable_irq(ts);

	if ((rc = i2c_master_send(ts->client, data, 3)) < 0)
	{
		KEY08_PRINTK(KERN_ERR "%s error  %d\n,",__FUNCTION__,rc);
	}
	clk_busy_wait(3000);
	

    for(i=0; i<1000; i++)
    {

	data[0] = 0;
	if ((rc = i2c_master_recv(ts->client, data, 6 )) < 0 && rc >= 0 )
	{
		KEY08_PRINTK(KERN_ERR "%s error  %d\n,",__FUNCTION__,rc);
	}

        KEY08_PRINTK("%s: %02x %02x %02x %02x %02x %02x \n",__FUNCTION__, 
            data[0],
            data[1],
            data[2],
            data[3],
            data[4],
            data[5]
            );

        if ((data[0] == 0x80) && (data[1] == (reg>>2)))
        {
            break;
        }
    }

	key08_enable_irq(ts);

	KEY08_PRINTK("%s: Exit(rc=%d)\n",__FUNCTION__, rc);
	if ( rc > 0 )
        // return 16 bit value
		return((data[2+(reg%4)]<<8) | (data[2+(reg%4)+1]) );
	else
		return rc;

}

static int	key08_calibrate(struct key08_ts_data *ts)
{
	int		ret = 0;
	char	regValue = 0x01; 

	KEY08_PRINTK("%s: Calibrating touchscreen\n", __FUNCTION__);

	sliderEvent = TRUE;
	ret = key08_reg_write (ts,
			Q51001211009AK08_CALIBRATE_ADDR,
			sizeof(regValue), (char *)&regValue);
	sliderEvent = FALSE;
#ifndef CONFIG_MACH_MOT
	ts->tsbState = KEY08_TSB_UNTOUCHED;
	ts->calibrate_ms = jiffies_to_msecs(jiffies);
	ts->calibrate_msg = FALSE;
#endif
	clk_busy_wait(3000);
	clk_busy_wait(3000);

	return ret;
}

/* BEGIN Motorola, 05/07/2010, IKMORRISON-1570 */
static ssize_t key08_sysfs_calibrate(struct device *dev,
			struct device_attribute *attr,
			char *buf, size_t size)
{
	int ret = key08_calibrate(tsGl);
	return sprintf(buf, "%d\n", ret);
}


static DEVICE_ATTR(calibrate_touch, 0444, key08_sysfs_calibrate, NULL);

static struct attribute *key08_attrs[] = {
	&dev_attr_calibrate_touch.attr, NULL
};

static struct attribute_group key08_attr_grp = {
	.attrs = key08_attrs,
};
/* END Motorola, wlw038, 05/07/2010, IKMORRISON-1570 */

static int key08_init_panel(struct key08_ts_data *ts)
{
	int ret=0;
//	int	 i;
	KEY08_PRINTK("%s: Initializing Panel...\n",__FUNCTION__);
	return ret;
}

static void key08_doReset(void)
{
#ifdef CONFIG_MACH_MOT
	gpio_set_value(84, 0);
	gpio_set_value(84, 1);
#else
	gpio_set_value(TOUCH_RST_N,0);
	gpio_set_value(TOUCH_RST_N,1);
#endif
	clk_busy_wait(3000);
}

static void key08_workSlider(struct work_struct *work)
{
	key08_calibrate(tsGl);
}

static void key08_ts_work_func(struct work_struct *work)
{
	struct key08_ts_data *ts;
	int i;
	int ret;
	int bad_data = 0;
	uint8_t buf[15];
	int base;
	int	x,y,z,w;
	uint8_t	msgCode;
	ktime_t	tims[10];
	int tIndex = 0;
	int	gpioStatus;
#ifndef CONFIG_MACH_MOT
	bool saved_calibrate_msg;
#endif
	int finger=0;

	KEY08_PRINTK(KERN_ERR "%s: Enter\n",__FUNCTION__);   

	gpioStatus = gpio_get_value(KEY08_TS_GPIO);
	key08_printk("%s: GPIO is %s", __FUNCTION__, (gpioStatus == 1) ? "HIGH\n":"LOW\n" );

	ts = container_of(work, struct key08_ts_data, work);
	
	if ( ts->mode == KEY08_MODE_BOOTLOADER )
	{
		tIndex = 0;
		tims[tIndex++] = ktime_get();
		ret = i2c_master_recv(ts->client, buf,1);
		clk_busy_wait(3000);
		if (ret < 0)
		{
			/* Receive failed. Reset status  */
/* 
 * ASSUMPTION: we fail to read because there is no client on 0x5F 
 * address because IC has finished reset after the firmware.
 * So, we can switch the address to 0x11 and to NORMAL mode
 * */
			KEY08_PRINTK("%s: i2c_master_recv failed(BOOTLOADER mode)\n",__FUNCTION__);
			ts->sendMode = KEY08_BL_WAITING_FOR_NOTHING;
			ts->mode = KEY08_MODE_NORMAL;
			ts->client->addr = key08_normal_addr;
		}
		else
		{
			KEY08_PRINTK("%s: buf[0] = 0x%x\n", __FUNCTION__, buf[0]);
			/* Determine which code we got and set status appropriately */
			if ( (buf[0] & 0xF0) == 0xC0 )
			{
				ts->sendMode = KEY08_BL_WAITING_FOR_COMMAND;
			}
			else if ( (buf[0] & 0xF0) == 0x80 )
			{
				if ( ts->sendMode == KEY08_BL_GOT_BAD_CRC )
					ts->sendMode = KEY08_BL_WAITING_AFTER_BAD_CRC;
				else
					ts->sendMode = KEY08_BL_WAITING_FOR_DATA;
			}
			else if ( buf[0] == 0x02 )
			{
				ts->sendMode = KEY08_BL_WAITING_FOR_CRC;
			}
			else if ( buf[0] == 0x04 )
			{
				ts->sendMode = KEY08_BL_WAITING_AFTER_GOOD_CRC;
			}
			else if ( buf[0] == 0x03 )
			{
				/* We got bad CRC on the record */
				ts->sendMode = KEY08_BL_GOT_BAD_CRC;
			}
		}
	}
	if ( ts->mode == KEY08_MODE_NORMAL )
	{
#ifdef CONFIG_MACH_MOT
		unsigned char   complete = FALSE;
#else
		unsigned char complete = gpioStatus;
#endif
#ifndef CONFIG_MACH_MOT

		if ( ts->tsbState == KEY08_TSB_DEBOUNCED )
		{
			vibrator_activate(KEY08_TSB_VIB_MS);
			input_report_key(ts->input_dev, ts->tsbTouched, 1);
			input_sync(ts->input_dev);
			ts->tsbState = KEY08_TSB_PRESSED;
			KEY08_PRINTK("%s: key press sent, key=%u\n",__FUNCTION__,
				ts->tsbTouched);
		}	
#endif
		while ( !complete )
		{
			unsigned char	needCalibration = FALSE;
			unsigned char	need2sendRelease = FALSE;
			char	fingCal[NUMBER_OF_FINGERS];

			tIndex = 0;
			tims[tIndex++] = ktime_get();
			tims[tIndex++] = ktime_get();
			ret = i2c_master_recv(ts->client, buf, 6);

			tims[tIndex++] = ktime_get();
			if (ret < 0) 
			{
				key08_printk("%s: i2c_master_recv() failed. rc = %d\n",__FUNCTION__, ret );
				clk_busy_wait(5000);
			}
			else 
			{
				msgCode = (buf[0]>>4) & 0x07;
#ifndef CONFIG_MACH_MOT
				if (ts->calibrate_ms)
				{
					if ((jiffies_to_msecs(jiffies) - ts->calibrate_ms) >=
						KEY08_CALIBRATE_TIMEOUT)
					{
						/* Calibration timed out. */
						ts->calibrate_ms = 0;
						ts->calibrate_msg = FALSE;
						KEY08_PRINTK("%s: calibration timed out.\n", __FUNCTION__);
					}
					else if (msgCode == Q51001211009AK08_STATUS_MSG)
					{
						saved_calibrate_msg = ts->calibrate_msg;
						ts->calibrate_msg = KEY08_STATUS_MSG_IN_CAL(buf);
						KEY08_PRINTK("%s: saved_calibrate_msg=%d, ts->calibrate_msg=%u\n",
							__FUNCTION__, saved_calibrate_msg, ts->calibrate_msg);
						if (saved_calibrate_msg && !ts->calibrate_msg)
						{
							/* Calibration is complete */
							ts->calibrate_ms = 0;
							ts->calibrate_msg = FALSE;
							KEY08_PRINTK("%s: calibration completed.\n", __FUNCTION__);
						}
					}
					else
					{
						/*
						* While waiting for a calibration to complete, ignore
						* XY and Key messages. The message data isn't reliable
						* until the calibration completes.
						*/
						KEY08_PRINTK("%s: message tossed.\n", __FUNCTION__);
						continue;
					}
				}
#endif

				switch (msgCode)
				{
				case Q51001211009AK08_KEY_MSG:
					KEY08_PRINTK("%s: Processing key message - %.2X %.2X %.2X %.2X\n",
                  __FUNCTION__, buf[2], buf[3], buf[4], buf[5]);
               /*
                * Since there are only 3 keys and they are pre-defined.
                * Just check if one of this keys is pressed. Ignore
                * everything else
                */
#ifndef CONFIG_MACH_MOT
               if (ts->tsbState != KEY08_TSB_UNTOUCHED)
               {
                  if (!KEY08_KEY_MSG_TSB_ACTIVE(buf, KEY08_KEY_MSG_TSB_2) &&
                     !KEY08_KEY_MSG_TSB_ACTIVE(buf, KEY08_KEY_MSG_TSB_22) &&
                     !KEY08_KEY_MSG_TSB_ACTIVE(buf, KEY08_KEY_MSG_TSB_30))
                  {
                     if (ts->tsbState == KEY08_TSB_PRESSED)
                     {
                        input_report_key(ts->input_dev, ts->tsbTouched, 0);
                        input_sync(ts->input_dev);
                        KEY08_PRINTK("%s: key release sent, key=%u\n",
                           __FUNCTION__, ts->tsbTouched);
                     }
                     else
                     {
                        hrtimer_cancel(&ts->tsbTimer);
                     }
                     ts->tsbState = KEY08_TSB_UNTOUCHED;
                  }
               }
               else
               {
                  /* Menu key */
                  if (KEY08_KEY_MSG_TSB_ACTIVE(buf, KEY08_KEY_MSG_TSB_2))
                  {
                     ts->tsbTouched = QWERTY_KL_MENU_KEY;
                     ts->tsbState = KEY08_TSB_IN_DEBOUNCE;
                     hrtimer_start(&ts->tsbTimer,
                        ktime_set(0, KEY08_TSB_HOLD_MS *
                           (NSEC_PER_SEC/MSEC_PER_SEC)),
                        HRTIMER_MODE_REL);
                  }
                  /* Home key */
                  else if (KEY08_KEY_MSG_TSB_ACTIVE(buf, KEY08_KEY_MSG_TSB_22))
                  {
                     ts->tsbTouched = QWERTY_KL_HOME_KEY;
                     ts->tsbState = KEY08_TSB_IN_DEBOUNCE;
                     hrtimer_start(&ts->tsbTimer,
                        ktime_set(0, KEY08_TSB_HOLD_MS *
                           (NSEC_PER_SEC/MSEC_PER_SEC)),
                        HRTIMER_MODE_REL);
                  }
                  /* Back key */
                  else if (KEY08_KEY_MSG_TSB_ACTIVE(buf, KEY08_KEY_MSG_TSB_30))
                  {
                     ts->tsbTouched = QWERTY_KL_BACK_KEY;
                     ts->tsbState = KEY08_TSB_IN_DEBOUNCE;
                     hrtimer_start(&ts->tsbTimer,
                        ktime_set(0, KEY08_TSB_HOLD_MS *
                           (NSEC_PER_SEC/MSEC_PER_SEC)),
                        HRTIMER_MODE_REL);
                  }
               }
#endif
#ifdef CONFIG_MACH_MOT

					if (buf[2] & 0x80) {
						/* Back key */
						input_report_key(ts->input_dev,
							158, 1);
						lastKeyPressed = 158;
					}
					if (buf[3] & 0x08) {
						/* Home key */
						input_report_key(ts->input_dev,
							102, 1);
						lastKeyPressed = 102;
					}
					if (buf[4] & 0x80) {

						/* Menu key */
						input_report_key(ts->input_dev,
							139, 1);
						lastKeyPressed = 139;
					}
					if (lastKeyPressed != 0 && buf[2] == 0 && buf[3] == 0 && buf[4] == 0) {
						input_report_key(ts->input_dev,
							lastKeyPressed, 0);
						lastKeyPressed = 0;
					}
					input_sync(ts->input_dev);
#endif
					break;
				case Q51001211009AK08_10BIT_TSP_MSG:
						bad_data = 0;
					needCalibration = FALSE;
					for ( i = 0; i < NUMBER_OF_FINGERS; i++ )
						fingCal[i] = FALSE;
						/* Extract coordinates from the message */
/* since x and y are reversed compared to the screen, put them into the right variable to begin with */
					y = ((buf[Q51001211009AK08_MSG_PTR10_DATA_XL_BYTE] | 
						((buf[Q51001211009AK08_MSG_PTR10_DATA_XH_BYTE] &
						Q51001211009AK08_MSG_PTR10_DATA_XYH_MASK) << 8)));
					x = ((buf[Q51001211009AK08_MSG_PTR10_DATA_YL_BYTE] | 
						((buf[Q51001211009AK08_MSG_PTR10_DATA_YH_BYTE] &
						Q51001211009AK08_MSG_PTR10_DATA_XYH_MASK) << 8))); 
					
					z = buf[Q51001211009AK08_MSG_CRUNCH_FLAG_BYTE] >> Q51001211009AK08_MSG_CRUNCH_FLAG_SHIFT;
					w = buf[Q51001211009AK08_MSG_PTR10_DATA_XH_BYTE] >> 4;
					KEY08_PRINTK("%s: W = %d\n", __FUNCTION__, w);

                    // filtering touch on edge
                    switch(buf[0]&0x0f)
                    {
                    case 1:
                        finger=0;
                        break;
                    case 2:
                        finger=1;
                        break;
                    case 4:
                        finger=2;
                        break;
                    case 8:
                        finger=3;
                        break;
                    }

                    if(buf[1] && (buf[0]&0x0f))
                    {
                        // touch
                        KEY08_PRINTK("%s: touch f=%d x=%d y=%d\n", __FUNCTION__, finger, x,y);

                        if ( ((x < ts->gMinx ) || (x > (1023-ts->gMaxx)) 
							|| (y < ts->gMiny ) || (y > (1023-ts->gMaxy)))
								 && !ts->touch_reported[finger])
                        {
                            // ignore this touch
                            KEY08_PRINTK("%s: ignoring this touch\n", __FUNCTION__);
                            break;
                        }
                        else
                        {
                            ts->touch_reported[finger] = TRUE;
                        }
                    }
                    else
                    {
                        // release
                        KEY08_PRINTK("%s: release f=%d, x=%d y=%d\n", __FUNCTION__, finger, x,y);

                        if(ts->touch_reported[finger])
                        {
                            ts->touch_reported[finger] = FALSE;
                            //continue
                        }
                        else
                        {
                            // ignore this release
                            KEY08_PRINTK("%s: ignoring this release\n", __FUNCTION__);
                            break;
                        }
                    }



/* update status of each finger */
					for ( i = 0; i < NUMBER_OF_FINGERS; i++ )
					{
						obj[i] =  buf[0] & (0x01<<i);
					}
					KEY08_PRINTK("%s: Objects: %d %d %d %d\n", __FUNCTION__, obj[0],obj[1],obj[2],obj[3]);
					for ( i = 0; i < NUMBER_OF_FINGERS; i++ )
					{
						int	fing;

						fing =  buf[1] & (0x01<<i);
						if ( i > 0 )
						{
							long	msecsHT;
							/* check if finger 2,3, or 4 is lifted */
							if ( fing == 0 && tf[i] != 0 )
							{
								/* non-first finger is lifted. There is a possibility of need for calibration */
								msecsHT = jiffies - timsPress[i];
								KEY08_PRINTK("%s: Finger %d was held for %lu jiffies.\n ",__FUNCTION__, i,msecsHT);
								if ( msecsHT < 12 )
									fingCal[i] = FALSE;
								else
									fingCal[i] = TRUE;
							}
							/* Check if finger 2,3, or 4 is pressed */
							if ( fing != 0 && tf[i] == 0 )
							{
								/* non-first finger is pressed. Save jiffies */
								timsPress[i] = jiffies;
							}
							tf[i] = fing;
						}
						else
						{
							/* Check if there is a need to send a release */
							if ( fing == 0 && tf[i] != 0 )
							{
								need2sendRelease = TRUE;
								if ( tf[1] != 0 || tf[2] != 0 || tf[3] != 0 )
									needCalibration = TRUE;
							}
							tf[i] = fing;
						}
						KEY08_PRINTK("%s: Finger %d is %s.\n ",__FUNCTION__, i, (tf[i]) ? "pressed" : "lifted" );
					}
					for ( i = 0; i < NUMBER_OF_FINGERS; i++ )
					{
						if ( fingCal[i] != FALSE )
						{
							needCalibration = TRUE;
							break;
						}
					}

					if ( needCalibration )
						key08_calibrate(ts);
					needCalibration = FALSE;						
						base = 2;
#ifdef CONFIG_MACH_MOT
					x = Q51001211009AK08_TS_MAX_X - x;
					y = Q51001211009AK08_TS_MAX_Y - y;
#endif

						pos[0][0] = x;
						pos[0][1] = y;
						
					if ( need2sendRelease )
					{
						/* Finger is released */
						currPoints[0].X = 0xFFFF;
						currPoints[0].Y = 0xFFFF;
						currPoints[0].finger = 1;
						input_report_abs(ts->input_dev, ABS_PRESSURE, z);
						tims[tIndex++] = ktime_get();
						input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
						tims[tIndex++] = ktime_get();
						input_report_key(ts->input_dev, BTN_TOUCH, 0);
						tims[tIndex++] = ktime_get();
						input_sync(ts->input_dev);
						tims[tIndex++] = ktime_get();
					}
					else if (obj[0]) 
							{
								int	scX = x * SCREEN_X / KEY08_MAX_X;
								int	scY = y * SCREEN_Y / KEY08_MAX_Y;
						currPoints[0].X = scX;
						currPoints[0].Y = scY;
						currPoints[0].finger = 1;
						if ( !ts->suspendMode )
						{
								input_report_abs(ts->input_dev, ABS_X, scX);
								tims[tIndex++] = ktime_get();
								input_report_abs(ts->input_dev, ABS_Y, scY);
								tims[tIndex++] = ktime_get();
							input_report_abs(ts->input_dev, ABS_PRESSURE, z);
							tims[tIndex++] = ktime_get();
							input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
							tims[tIndex++] = ktime_get();
							input_report_key(ts->input_dev, BTN_TOUCH, obj[0]);
							tims[tIndex++] = ktime_get();
							input_sync(ts->input_dev);
							tims[tIndex++] = ktime_get();
						}
								prevX = x;
								prevY = y;
							}
					need2sendRelease = FALSE;
						snprintf(dbgBuffer,DBG_BUFFER_SIZE-1,"%s: times: ", __FUNCTION__ );
						tIndex--;
						for ( i = 0; i < tIndex; i++ )
						{
							char	tmp[50];
							sprintf(tmp, "%llu ",ktime_to_ns(ktime_sub(tims[i+1],tims[i])));
							strncat(dbgBuffer,tmp, DBG_BUFFER_SIZE - strlen(dbgBuffer) - 1);
						}
						KEY08_PRINTK("%s %llu\n",dbgBuffer, ktime_to_ns(tims[i]));
					break;
					case Q51001211009AK08_SLIDER_MSG:
					KEY08_PRINTK("%s: Processing SLIDER message\n",__FUNCTION__);
					break;
					case Q51001211009AK08_OMEGA_MSG:
					KEY08_PRINTK("%s: Processing OMEGA message\n",__FUNCTION__);
					break;
					case Q51001211009AK08_8BIT_TSP_MSG:
					KEY08_PRINTK("%s: Processing 8BIT_TSP message\n",__FUNCTION__);
					break;
					case Q51001211009AK08_STATUS_MSG:
					KEY08_PRINTK("%s: Processing STATUS message\n",__FUNCTION__);
					break;
					case Q51001211009AK08_READ_MSG:
					KEY08_PRINTK("%s: Processing READ message\n",__FUNCTION__);
					break;
				default:
					KEY08_PRINTK("%s: Processing unknown message\n",__FUNCTION__);
					break;
				}
			}
/*
 * Check if interrupt line is high. If is, then all data from i2c has been read
 */			
			gpioStatus = gpio_get_value(KEY08_TS_GPIO);
			KEY08_PRINTK("%s: GPIO is %s", __FUNCTION__, (gpioStatus == 1) ? "HIGH\n":"LOW\n" );
			if ( gpioStatus == 1 )
				complete = TRUE;
			if ( !ts->use_irq )
				complete = TRUE;
		}
	}

	key08_enable_irq(ts);
	KEY08_PRINTK("%s: Exit\n",__FUNCTION__);   
}
#ifndef CONFIG_MACH_MOT
static enum hrtimer_restart key08_ts_tsb_timer_func(struct hrtimer *timer)
{
   struct key08_ts_data *ts = container_of(timer, struct key08_ts_data,
      tsbTimer);

   if (ts->tsbState == KEY08_TSB_IN_DEBOUNCE)
   {
      ts->tsbState = KEY08_TSB_DEBOUNCED;
      schedule_work(&ts->work);
   }
   return HRTIMER_NORESTART;
}
#endif
static enum hrtimer_restart key08_ts_timer_func(struct hrtimer *timer)
{
	struct key08_ts_data *ts = container_of(timer, struct key08_ts_data, timer);
	/* KEY08_PRINTK("key08_ts_timer_func\n"); */

	schedule_work(&ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 1250000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t key08_ts_irq_handler(int irq, void *dev_id)
{
	struct key08_ts_data *ts = dev_id;
	KEY08_PRINTK("%s: Enter\n",__FUNCTION__);

	key08_disable_irq(ts);

	schedule_work(&ts->work);

	KEY08_PRINTK("%s: Exit\n",__FUNCTION__);
	return IRQ_HANDLED;
}


static int  key08_ts_read_app_version(struct key08_ts_data *ts)
{
	struct	i2c_client *client;
	uint8_t data[6];
	int	reg;
	int	rc;
	int	version = -1;
	
	KEY08_PRINTK("key08_ts_read_app_version: Enter\n");
	KEY08_PRINTK("key08_ts_read_app_version: ts: 0x%x, ts->client: 0x%x\n", (int)ts, (int)ts->client);
	client = ts->client;

	if ( ts->mode == KEY08_MODE_NORMAL )
	{
		reg = 0x01;
		KEY08_PRINTK("key08_ts_read_app_version: sending to addr 0x%x register: 0x%x\n", (int)client->addr, (int)(data[0]));
		rc = key08_ts_read_register(ts, reg);
        KEY08_PRINTK("key08_ts_read_app_version: Got: %d from reading reg 1\n", rc);

		return(rc); 
	}
	else
	{
		/* we are in bootloader mode. So, read 1 byte from bootloader client */
		version = -1;

		client->addr = key08_bl_addr;;
		if ((rc = i2c_master_recv(client, data, 1 )) < 0 )
		{
			KEY08_PRINTK("key08_ts_read_app_version: i2c_master_revc bootloader error  %d\n,",rc);
		}

	}
   	return(version);
}


struct i2c_client Key08_ts_client;

//static int (struct i2c_client *client)

int key08_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct platform_device *pdev;
	//unsigned long request_flags =  IRQ_TYPE_EDGE_FALLING;
	unsigned long request_flags =  IRQ_TYPE_LEVEL_LOW;
	int ret = 0;
	uint16_t max_x, max_y;
	int fuzz_x, fuzz_y, fuzz_p, fuzz_w;
	int inactive_area_left;
	int inactive_area_right;
	int inactive_area_top;
	int inactive_area_bottom;
	int	numNeededReads;
	char	data[10];
	int retry = 10;
	int	i;

	printk("key08_ts_probe: Enter\n");

	prevX = 0;
	prevY = 0;
	threshX = 20;
	threshY = 20;
	ret = misc_register(&key08_pf_driver);
	printk("key08_ts_init: misc_register(pf) returns %d\n", ret);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		printk(KERN_ERR "key08_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
//	client=&Key08_ts_client;
	tsGl = kzalloc(sizeof(*tsGl), GFP_KERNEL);
	if (tsGl == NULL) 
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&tsGl->work, key08_ts_work_func);
	INIT_WORK(&tsGl->workSlider, key08_workSlider);

	tsGl->client = client;
	i2c_set_clientdata(client, tsGl);
	client->flags = 0;
	tsGl->calibrate = FALSE;
	tsGl->suspendMode = FALSE;
	tsGl->XferStatus = KEY08_FM_DOWNLOAD_NOT_STARTED | KEY08_CFG_DOWNLOAD_NOT_STARTED;
#ifndef CONFIG_MACH_MOT
	tsGl->calibrate_ms = 0;
	tsGl->calibrate_msg = FALSE;
#endif
	tsGl->gMinx = 5;
	tsGl->gMaxx = 20;
	tsGl->gMiny = 12;
	tsGl->gMaxy = 0;
	tsGl->area = 0xfff;
	for ( i = 0; i < NUMBER_OF_FINGERS; i++ )
	{
		currPoints[i].X = 0xFFFF;
		currPoints[i].Y = 0xFFFF;
		currPoints[i].Z = 0xFFFF;
		currPoints[i].W = 0xFFFF;
		currPoints[i].finger = 0;
		obj[i] = 0;
		tf[i] = 0;
		tsGl->touch_reported[i] = FALSE;
	}

	key08_normal_addr = 0x11;
	key08_bl_addr = 0x5F;

	pdev = (struct platform_device *)client->dev.platform_data;
	if ( pdev && pdev->num_resources && (!strcmp(pdev->name,"key08")))
	{
		struct	resource *resource;
		printk("key08_ts_probe: paltform data for '%s', num_resources %d\n",
			pdev->name, pdev->num_resources);
		resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if ( resource)
		{
			key08_normal_addr = resource->start;
			key08_bl_addr = resource->end;
			printk("key08_ts_probe: Got resource. Normal Addr: 0x%x, Bootloader: 0x%x\n",
				key08_normal_addr, key08_bl_addr);
		}
		else
		{
			printk("key08_ts_probe: Could not optain GPIO information. Using default values\n");
		}
	} 
	else
	{
		printk("key08_ts_probe: Could not get platform reousrces. Using default values\n");
			
	}

	printk("key08_ts_probe: attached, client: 0x%x\n", (unsigned)client);
	printk("key08_ts_probe: irq: 0x%x\n", client->irq);

	inactive_area_left = 0;
	inactive_area_right = 0;
	inactive_area_top = 0;
	inactive_area_bottom = 0;
	fuzz_x = 0;
	fuzz_y = 0;
	fuzz_p = 0;
	fuzz_w = 0;

	if (tsGl->power) 
	{
		ret = tsGl->power(1);
		if (ret < 0) 
		{
			printk(KERN_ERR "key08_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}
	printk(KERN_ERR "key08_ts_probe STARTED\n");
	printk(KERN_ERR "key08_ts_probe Getting Versions\n");

/*
   This hardware requires to read all data from the buffers when it starts.
   Until all data is read, it will not assert the IRQ to high. So, we need to
   read bunch of sets of 6 bytes until there are no more.

   NOTE: if we faile to read version, we may be in the bootloader mode. In that case,
   we have to read from different address.
*/
	numNeededReads = 1;
	do
	{
		clk_busy_wait(3000);
		memset(data,0,sizeof(data));

		if ((ret = i2c_master_recv(client, data, 6 )) < 0 )
		{
			printk("key08_ts_probe: i2c_master_recv (normal client) error  %d\n",ret);
			client->addr = key08_bl_addr;
			if ((ret = i2c_master_recv(client, data, 1 )) < 0 )
			{
				printk("key08_ts_probe: i2c_master_recv (bootloader client) error  %d\n",ret);
				printk("key08_ts_probe: Cause: touch device is probably missing. \n");
				tsGl->mode = KEY08_MODE_UNKNOWN;
				return ret;
			}
			else
			{
				printk("key08_ts_probe: i2c_master_recv (bootloader client) data: 0x%x\n",data[0]);
				printk("key08_ts_probe: entering bootloader mode\n");
				client->addr = key08_bl_addr;
				tsGl->mode = KEY08_MODE_BOOTLOADER;
				/* wait until we get 0x8n */
				while ( (data[0] & 0xF0) != 0x80 )
				{
					printk("key08_ts_probe: Sending Unblock command (0xDC,0xAA)\n");
					data[0] = 0xDC;
					data[1] = 0xAA;
					ret = i2c_master_send (client, data, 2);
					if ( ret < 0 )
					{
						printk("key08_ts_probe: i2c_master_send (bootloader unlock: 0xDC,0xAA)\n");
						tsGl->mode = KEY08_MODE_UNKNOWN;
						return ret;
					}
					clk_busy_wait(3000);
					ret = i2c_master_recv(client, data, 1);
					if ( ret < 0 )
					{
						printk("key08_ts_probe: i2c_master_recv (bootloader client waiting for 0x8n) error  %d\n",ret);
						tsGl->mode = KEY08_MODE_UNKNOWN;
						return ret;
					}
					clk_busy_wait(3000);
				}
			}
		}
		else
			tsGl->mode = KEY08_MODE_NORMAL;
		if ( tsGl->mode == KEY08_MODE_NORMAL )
		{
			if ( data[0] == 0xff )
			{
				printk("key08_ts_probe: Got all data.\n");
				ret = 0;
			}
			else
			{
				printk("key08_ts_probe: (%d) receiving from 0x%x(number of bytes: %d, type: %d): 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
					numNeededReads++, client->addr, ret, ((data[0]>>4&0x07)), data[0], data[1], data[2], data[3], data[4], data[5] );
			}
		}
	} while ( ret == 6 && tsGl->mode == KEY08_MODE_NORMAL);

	while (retry-- > 0 && tsGl->mode == KEY08_MODE_NORMAL )
	{
		ret = key08_ts_read_register(tsGl, (int)0x0);
		if (ret >= 0)
			break;
		msleep(100);
	}

	if (ret < 0)
	{
		printk(KERN_ERR "key08_ts_probe: i2c_read_byte_data failed\n");
		goto err_detect_failed;
	}
	if ( tsGl->mode == KEY08_MODE_NORMAL )
	{
		printk("key08_ts_probe: Chip ID 0x%x\n", ret);
		ret = key08_ts_read_register(tsGl, (int)0x1);
		if (ret < 0)
		{
			printk(KERN_ERR "key08_ts_probe: i2c_read_byte_data failed\n");
			goto err_detect_failed;
		}

		printk("key08_ts_probe: Product Code Version 0x%x\n", ret);
#ifndef CONFIG_MACH_MOT

		ret = key08_ts_read_register(tsGl, (int)Q51001211009AK08_LP_MODE_ADDR);
		if (ret < 0)
		{
			printk(KERN_ERR "key08_ts_probe: i2c_read_byte_data failed\n");
		}
		else
		{
			printk(KERN_INFO "%s: Saving LP_MODE=%d\n", __FUNCTION__, ret);
			configRegs[Q51001211009AK08_LP_MODE_ADDR] = ret;
		}
#endif
	}
	if ( tsGl->mode != KEY08_MODE_UNKNOWN )
	{
		max_x = KEY08_MAX_X;
		max_y = KEY08_MAX_Y;

		/* tsGl->max[1] = max_y = (ret >> 8 & 0xff) | ((ret & 0x1f) << 8); */
		tsGl->max[1] = max_y;

		if (tsGl->flags & KEY08_SWAP_XY)
			swap(max_x, max_y);
		printk("key08_ts_probe Initializing Panel \n");
		ret = key08_init_panel(tsGl); /* will also switch back to page 0x04 */
		if (ret < 0) 
		{
			printk(KERN_ERR "key08_ts_proe: key08_init_panel() failed\n");
			goto err_detect_failed;
		}

		tsGl->input_dev = input_allocate_device();
		if (tsGl->input_dev == NULL) 
		{
			ret = -ENOMEM;
			printk(KERN_ERR "key08_ts_probe: Failed to allocate input device\n");
			goto err_input_dev_alloc_failed;
		}
		tsGl->input_dev->name = "touchscreen";
		set_bit(EV_SYN, tsGl->input_dev->evbit);
		set_bit(EV_KEY, tsGl->input_dev->evbit);
		set_bit(BTN_TOUCH, tsGl->input_dev->keybit);
		set_bit(KEY_HOME, tsGl->input_dev->keybit);
		set_bit(KEY_BACK, tsGl->input_dev->keybit);
		set_bit(KEY_MENU, tsGl->input_dev->keybit);
		set_bit(BTN_2, tsGl->input_dev->keybit);
		set_bit(EV_ABS, tsGl->input_dev->evbit);
		inactive_area_left = inactive_area_left * max_x / 0x10000;
		inactive_area_right = inactive_area_right * max_x / 0x10000;
		inactive_area_top = inactive_area_top * max_y / 0x10000;
		inactive_area_bottom = inactive_area_bottom * max_y / 0x10000;
		fuzz_x = fuzz_x * max_x / 0x10000;
		fuzz_y = fuzz_y * max_y / 0x10000;
		tsGl->snap_down[!!(tsGl->flags & KEY08_SWAP_XY)] = -inactive_area_left;
		tsGl->snap_up[!!(tsGl->flags & KEY08_SWAP_XY)] = max_x + inactive_area_right;
		tsGl->snap_down[!(tsGl->flags & KEY08_SWAP_XY)] = -inactive_area_top;
		tsGl->snap_up[!(tsGl->flags & KEY08_SWAP_XY)] = max_y + inactive_area_bottom;
		printk(KERN_ERR "key08_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
		printk(KERN_ERR "key08_ts_probe: inactive_x %d %d, inactive_y %d %d\n",
			inactive_area_left, inactive_area_right,
			inactive_area_top, inactive_area_bottom);
		inactive_area_left =0;
		inactive_area_top =0;
		inactive_area_right=0;
		inactive_area_bottom=0;
			
		input_set_abs_params(tsGl->input_dev, ABS_X, 2, SCREEN_X, 0, 0);
		input_set_abs_params(tsGl->input_dev, ABS_Y, 0, SCREEN_Y, 0, 0);
		
		input_set_abs_params(tsGl->input_dev, ABS_PRESSURE, 0, 0xff, 2, 0);
		input_set_abs_params(tsGl->input_dev, ABS_TOOL_WIDTH, 0, 0xf, 2, 0);
		input_set_abs_params(tsGl->input_dev, ABS_HAT0X, 0, 0xfff, 2, 0);
		input_set_abs_params(tsGl->input_dev, ABS_HAT0Y, 0, 0xfff, 2, 0);
		/* tsGl->input_dev->name = tsGl->keypad_info->name; */

		ret = input_register_handler(&slider_handler);
		if (ret) 
		{
			printk(KERN_ERR "key08_ts_probe: Unable to register slider input device\n");
			goto err_input_register_device_failed;
		}
		ret = input_register_device(tsGl->input_dev);
		if (ret) 
		{
			printk(KERN_ERR "key08_ts_probe: Unable to register %s input device\n", tsGl->input_dev->name);
			goto err_input_register_device_failed;
		}
		printk("key08_ts_probe Gpio request\n");
		ret = gpio_request(KEY08_TS_GPIO, "key08_ts_gpio");
		if (ret) 
		{
			printk(KERN_ERR " gpio_request failed for input %d\n", KEY08_TS_GPIO);
			goto err_input_register_device_failed;
		}
		
		ret = gpio_direction_input(KEY08_TS_GPIO);
		if (ret) 
		{
			printk(KERN_ERR " gpio_direction_input failed for input %d\n", KEY08_TS_GPIO);
			goto err_input_register_device_failed;
		}

		client->irq = gpio_to_irq(KEY08_TS_GPIO);
	//        printk(KERN_ERR " Input device %s , Irq %d  , dname %s\n", kp->input_dev->name,client->irq,client->driver_name);
/* try setting the interrupt low. This way if data is missed, the ISR will still be executed */
		// set_irq_type (client->irq, IRQT_LOW);
		ret = gpio_direction_input(KEY08_TS_GPIO);
		printk("key08_ts_probe: ret from gpio_direction_input = %d\n", ret);
	/*
		gpio_set_value(KEY08_TS_GPIO,1);
		msleep(500);
	*/
		printk(KERN_ERR "key08_ts_probe: gpio_request KEY08_TS_GPIO =  %d\n", gpio_get_value(KEY08_TS_GPIO));

	/* Temporary disable interrup and enable polling */
	/*
		client->irq = 0;
	*/

		if (client->irq) 
		{
			tsGl->irqCount = 1;
			tsGl->use_irq = TRUE;
			ret = request_irq(client->irq, key08_ts_irq_handler, request_flags , pdev->name, tsGl);
			if (ret != 0) 
			{
				tsGl->use_irq = FALSE;
				free_irq(client->irq, tsGl);
				dev_err(&client->dev, "request_irq failed\n");
			}
		}
		if (!tsGl->use_irq) 
		{
			hrtimer_init(&tsGl->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			tsGl->timer.function = key08_ts_timer_func;
			hrtimer_start(&tsGl->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
#ifdef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_MACH_MOT
	tsGl->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
#else
	tsGl->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 4;

#endif
		tsGl->early_suspend.suspend = key08_ts_early_suspend;
		tsGl->early_suspend.resume = key08_ts_late_resume;
		register_early_suspend(&tsGl->early_suspend);

	#if 0
		if (android_power_is_driver_suspended())
			key08_ts_early_suspend(&tsGl->early_suspend);
	#endif

#endif
		tsGl->inCall = FALSE;
			
		printk(KERN_INFO "key08_ts_probe: Start touchscreen %s in %s mode\n", tsGl->input_dev->name, tsGl->use_irq ? "interrupt" : "polling");

		if ( tsGl->mode == KEY08_MODE_NORMAL )
		{
			ret = key08_calibrate(tsGl);
		}
#ifndef CONFIG_MACH_MOT
		tsGl->tsbState = KEY08_TSB_UNTOUCHED;
		hrtimer_init(&tsGl->tsbTimer, CLOCK_REALTIME, HRTIMER_MODE_REL);
		tsGl->tsbTimer.function = key08_ts_tsb_timer_func;
#endif
#ifdef CONFIG_MACH_MOT
		lastKeyPressed = 0;
#endif

		printk("key08_ts_probe: GPIO 84: %d\n", gpio_get_value(84) );
	}
	(void) sysfs_create_group(&client->dev.kobj, &key08_attr_grp); /* Motorola, wlw038, 05/07/2010, IKMORRISON-1570 */
	printk("key08_ts_probe: Exit\n");
	return 0;


err_input_register_device_failed:
	input_free_device(tsGl->input_dev);
// exit_kfree:
err_input_dev_alloc_failed:
err_detect_failed:
err_power_failed:
	kfree(tsGl);
err_alloc_data_failed:
err_check_functionality_failed:
// exit_detach:
	i2c_detach_client(client);
	printk("key08_ts_probe: Exit after error\n");
	return ret;
}

static int key08_ts_remove(struct i2c_client *client)
{
	struct key08_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}
static int key08_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int 	ret;
	int	regValue;
	struct key08_ts_data *ts = i2c_get_clientdata(client);
	int i;
	
	KEY08_PRINTK("%s: Entering\n",__FUNCTION__);
	if ( ts->mode == KEY08_MODE_NORMAL )
	{
	lpValue = key08_ts_read_register(ts, (int)Q51001211009AK08_LP_MODE_ADDR);
	if ( lpValue != 0 )
	{
		regValue = 0x00; 
		ret = key08_reg_write (ts, Q51001211009AK08_LP_MODE_ADDR,
			sizeof(char), (char *)&regValue);
		clk_busy_wait(3000);
		clk_busy_wait(3000);
	}
	if (ts->use_irq)
	{
		key08_disable_irq(ts);
	}
	else
		hrtimer_cancel(&ts->timer);
#ifndef CONFIG_MACH_MOT
	hrtimer_cancel(&ts->tsbTimer);
	ret = cancel_work_sync(&ts->work);
	ts->tsbState = KEY08_TSB_UNTOUCHED;
#endif

	if (ts->power) 
	{
		ret = ts->power(0);
		if (ret < 0) 
		{
			KEY08_PRINTK(KERN_ERR "key08_ts_suspend: power off failed\n");
		}
	}
	i2c_set_clientdata(client,ts);
        for ( i=0; i < NUMBER_OF_FINGERS; i++ )
            ts->touch_reported[i] = FALSE;
	}
	KEY08_PRINTK("key08_ts_suspend: Exiting\n");
	return 0;
}

// static int key08_ts_resume(struct i2c_client *client)
static int key08_ts_resume(struct key08_ts_data *ts)
{
	int ret;
#ifdef CONFIG_MACH_MOT
	uint8_t buf[15];
	int numNeededReads;
	int bytes2Read;
#endif

	KEY08_PRINTK("key08_ts_resume: Entering()\n");
	if (ts->power) 
	{
		ret = ts->power(1);
		if (ret < 0) 
		{
			KEY08_PRINTK(KERN_ERR "key08_ts_resume power on failed\n");
		}
	}
	key08_init_panel(ts);

	/* wqnt78: 16119: First touch event is ignored on wakeup if there were
	 * existing touch events on suspend.
	 *
	 * The framework drops intervening touch events, causing the first gesture
	 * after wakeup to be treated as part of the gesture that was in progress
	 * during suspend. Force sending a touch up event to the framework on
	 * wakeup.
	 */
	__set_bit(BTN_TOUCH, ts->input_dev->key);       // Force a change of state
	input_report_key(ts->input_dev, BTN_TOUCH, 0);  // so event gets sent
	input_sync(ts->input_dev);
#ifndef CONFIG_MACH_MOT
	ts->tsbState = KEY08_TSB_UNTOUCHED;
#endif
#ifdef CONFIG_MACH_MOT
	bytes2Read = 6;
#endif
	if ( ts->mode == KEY08_MODE_NORMAL )
	{
		lpValue = configRegs[Q51001211009AK08_LP_MODE_ADDR];
		ret = key08_reg_write (ts, Q51001211009AK08_LP_MODE_ADDR,
			sizeof(char), (char *)&lpValue);
		clk_busy_wait(3000);
		clk_busy_wait(3000);

 		ret = key08_calibrate(ts);
	}
#ifdef CONFIG_MACH_MOT

	numNeededReads = 1;
	do {
		buf[0] = 0xff;
		ret = i2c_master_recv(ts->client, buf, bytes2Read);
		if (ret > 0) {

			clk_busy_wait(3000);
/* 			if (buf[0] == 0xff) */
			if (gpio_get_value(KEY08_TS_GPIO) > 0) {

				KEY08_PRINTK("key08_ts_resume:"
					"Got all data.\n");
				ret = 0;
			} else {
				KEY08_PRINTK("%s: reading %d.\n", __func__,
					numNeededReads++);
			}
		}
	} while (ret != 0);

	KEY08_PRINTK("%s: Finished read loop\n", __func__);
#endif

	if (ts->use_irq)
	{
		key08_enable_irq(ts);
	}
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	KEY08_PRINTK("key08_ts_resume: Exiting()\n");
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void key08_ts_early_suspend(struct early_suspend *h)
{
	struct key08_ts_data *ts;
	KEY08_PRINTK("key08_ts_early_suspend: Entering\n");
	ts = container_of(h, struct key08_ts_data, early_suspend);
	key08_ts_suspend(ts->client, PMSG_SUSPEND);
	KEY08_PRINTK("key08_ts_early_suspend: Exiting\n");
}

static void key08_ts_late_resume(struct early_suspend *h)
{
	struct key08_ts_data *ts;
	KEY08_PRINTK("key08_ts_late_resume: Enering\n");
	ts = container_of(h, struct key08_ts_data, early_suspend);
	key08_ts_resume(ts);
	KEY08_PRINTK("key08_ts_late_resume: Exiting\n");
}
#endif


static void key08_slider_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	KEY08_PRINTK("%s: enter....type: %d, code: %d\n",__FUNCTION__, type, code);
	if (type == EV_SW && (code == SW_LID || code == SW_TABLET_MODE) )
	{
		KEY08_PRINTK("%s: Event.  Type: %u, Code: %u, Value: %d\n",
				__func__, type, code, value);

 		schedule_work(&tsGl->workSlider);
    }
#ifdef CONFIG_MACH_MOT

	if (type == EV_KEY && (code == KEY_MENU || code == KEY_HOME || code == KEY_BACK)) {
		KEY08_PRINTK("%s: Event.  Type: %u, Code: %u, Value: %d\n",
						__func__, type, code, value);

		schedule_work(&tsGl->workSlider);
    }
#endif
	KEY08_PRINTK("%s: exit....\n",__FUNCTION__);
}


static int key08_slider_connect(struct input_handler *handler, struct input_dev *dev,
			const struct	input_device_id *id)
{
	struct input_handle *handle;
	int error ,sw_value;

	KEY08_PRINTK("%s: enter....\n",__FUNCTION__);
	if( tsGl->input_dev == dev )   
		return -ENODEV;

	if ( !(strncmp(dev->name,"adp5588",7)))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
   	if (!handle)
		return -ENOMEM;

	// tsGl->sliderDev = dev;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "key08_slider";
	error = input_register_handle(handle);
	if (error)
		goto err_free_handle;

   	error = input_open_device(handle);
	if (error)
		goto err_unregister_handle;
	sw_value = !!test_bit(SW_TABLET_MODE, dev->sw);

	KEY08_PRINTK("%s: Connected device: \"%s\", %s\n  sw state  %d ", __FUNCTION__, dev->name, dev->phys,sw_value);

	KEY08_PRINTK("%s: exit....\n",__FUNCTION__);
   	return 0;

err_unregister_handle:
 	input_unregister_handle(handle);
err_free_handle:
	kfree(handle);
	KEY08_PRINTK("%s: exit with error(%d)....\n",__FUNCTION__,error);
	return error;
}

static void key08_slider_disconnect(struct input_handle *handle)
{
    KEY08_PRINTK("%s: Disconnected device: %s\n",  __FUNCTION__,handle->dev->phys);

    input_close_device(handle);
    input_unregister_handle(handle);
    kfree(handle);
}

/**
 * Open device
 *
 * @param inode
 * @param filp
 * @return 0
 */
static	int	key08_open(struct inode *inode, struct file *filp)
{
	KEY08_PRINTK("key08_open: entering\n");
	KEY08_PRINTK("key08_open: exiting\n");

	return 0;
}

unsigned char	kernelBuffer[255];
/**
 * Ioctl implementation
 *
 * Device can be used only by one user process at the time
 * ioctl returns -EBUSY if device is already in use
 *
 * @param node File in /proc
 * @param f Kernel level file structure
 * @param cmd Ioctl command
 * @param arg Ioctl argument
 * @return 0 in success, or negative error code
 */
static int key08_ioctl(struct inode *node, struct file *filp, unsigned int cmd, unsigned long arg)
{
	A_TOUCH_POINT_PTR	dataPoint;
	unsigned long	bCount;
	int	regValue;
	int	rc = 0;
	char	data[10];
	uint8_t buf[15];
#ifdef CONFIG_MACH_MOT
	int     bytes2Read;
#endif
	int	numNeededReads = 1;

	KEY08_PRINTK("key08_ioctl: entering\n");

	/* struct key08_ts_data *ts = i2c_get_clientdata(client); */
	// disable_irq(tsGl->client->irq);

	switch (cmd) 
	{
	case KEY08_IOCTL_SET_IN_CALL:
		tsGl->inCall = TRUE;
		break;
	case KEY08_IOCTL_SET_NOT_IN_CALL:
		tsGl->inCall = FALSE;
		break;
	case KEY08_IOCTL_SET_BOOTLOADER_MODE:
		rc = 1;
		if ( tsGl->mode == KEY08_MODE_NORMAL )
		{
			rc = -1;
/* 
	According to the documentation:
		1) Make sure LP setting is not 0
		2) send 0xCC to the address 2
		3) sleep two cycles
		4) send reset
*/
			KEY08_PRINTK("key08_ioctl: making sure LP is not 0\n");
			regValue = key08_ts_read_register(tsGl, (int)Q51001211009AK08_LP_MODE_ADDR);
			if ( regValue == 0 )
			{
			/* Set it to whatever the config was set to */
				regValue = configRegs[Q51001211009AK08_LP_MODE_ADDR]; 
				rc = key08_reg_write (tsGl, Q51001211009AK08_LP_MODE_ADDR,
					sizeof(char), (char *)&regValue);
				clk_busy_wait(3000);
				clk_busy_wait(3000);
			}
			KEY08_PRINTK("key08_ioctl: Sending FORCE FLASH command (0xCC)\n");
			regValue = 0xCC;
			key08_reg_write (tsGl,Q51001211009AK08_BACKUP_ADDR,
				 sizeof(char), (char *)&regValue);

			clk_busy_wait(3000);
			clk_busy_wait(3000);
			KEY08_PRINTK("key08_ioctl: Sending RESET command\n");
			regValue = 0x01;
			key08_reg_write (tsGl,Q51001211009AK08_RESET_ADDR,
				 sizeof(char), (char *)&regValue);
			clk_busy_wait(3000);
			clk_busy_wait(3000);
			
			/* Confirm that we are in BOOTLOADER mode */
			regValue = 10;
			
			tsGl->mode = KEY08_MODE_BOOTLOADER;
			tsGl->client->addr = key08_bl_addr;
			data[0] = 0;
			data[1] = 0;
			data[2] = 0;
			while ( (data[0] & 0xF0) != 0xC0 &&
				(data[0] & 0xF0) != 0x80 )
			{
				rc = i2c_master_recv(tsGl->client, data, 1);
			}
			KEY08_PRINTK("key08_ioctl: Got 0x%02X\n",data[0]);
			if ( (data[0] & 0xF0 ) == 0xC0 )
			{
				KEY08_PRINTK("key08_ioctl: Got bootloader status\n");

				data[0] = 0xDC;
				data[1] = 0xAA;
				rc = i2c_master_send (tsGl->client, data, 2);
				clk_busy_wait(3000);

				data[0] = 0;
			}
			else if ( (data[0] & 0xF0 ) == 0x80 )
			{
				KEY08_PRINTK("key08_ioctl: Already in BOOTLOADER mode\n");
				rc = 1;
			}
		}
		else 
		{
			KEY08_PRINTK("key08_ioctl: Already in BOOTLOADER mode\n");
			rc = 1;
		}
		if ( rc != -1 )
			tsGl->sendMode = KEY08_BL_WAITING_FOR_DATA;
		tsGl->mode = KEY08_MODE_BOOTLOADER;
		KEY08_PRINTK("key08_ioctl: Mode is set to  %d\n", tsGl->mode);
		break;
	
	case KEY08_IOCTL_DL_GET_STATUS:
		rc = tsGl->sendMode;
		break;

	case KEY08_IOCTL_GET_MODE:
		rc = tsGl->mode;
		break;

	case KEY08_IOCTL_SET_NORMAL_MODE:
		if ( tsGl->mode != KEY08_MODE_NORMAL )
		{
			// disable_irq(tsGl->client->irq);
			KEY08_PRINTK("key08_ioctl: re-setting flags to normal mode\n");
			msleep_interruptible(200);
			tsGl->client->addr = key08_normal_addr;
			tsGl->mode = KEY08_MODE_NORMAL;
			tsGl->sendMode = KEY08_BL_WAITING_FOR_NOTHING;
			rc = tsGl->mode;

			key08_doReset();

			rc = key08_calibrate(tsGl);
		}
		KEY08_PRINTK("key08_ioctl: Mode is set to  %d\n", tsGl->mode);
		break;
	case KEY08_IOCTL_GET_VERSION:
		rc = key08_ts_read_app_version(tsGl);
		break;
	case KEY08_IOCTL_WRITE_REGISTER:
		bCount = copy_from_user(kernelBuffer, (char *)arg, (unsigned long)sizeof(A_REG)); 
		if ( bCount == 0 )
		{
			A_REG_PTR	regV;
			regV = (A_REG_PTR) kernelBuffer;
			KEY08_PRINTK("key08_ioctl: setting register %d = %d\n", (int)regV->reg, (int)regV->value);
			configRegs[regV->reg] = regV->value;
			rc = key08_reg_write (tsGl,regV->reg,
				sizeof(char), (char *)&regV->value);
			clk_busy_wait(3000);
			clk_busy_wait(3000);
		}
		break;
	case KEY08_IOCTL_READ_REGISTER:
		regValue = (int)arg;
		rc = key08_ts_read_register(tsGl,regValue);
		KEY08_PRINTK("key08_ioctl: Register %d = %d\n", (int)regValue, (int)rc);
		break;
	case KEY08_IOCTL_READ_4_REGS:
		bCount = copy_from_user(kernelBuffer, (char *)arg, (unsigned long)sizeof(A_REG)); 
		if ( bCount == 0 )
		{
			A_REG	*regV;
			regV = (A_REG *) kernelBuffer;
			rc = key08_ts_read_4_regs(tsGl,regV);
			KEY08_PRINTK("%s: Register (from %d) values %d, %d, %d, %d\n", __FUNCTION__, (int)regV->reg, regV->vals[0], regV->vals[1], regV->vals[2], regV->vals[3]);
			bCount = copy_to_user((char *)arg, (char *)regV, (unsigned long)sizeof(A_REG));
			if ( bCount != 0 )
				rc = -EFAULT;
		}
		break;
	case KEY08_IOCTL_READ_REGISTER16:
		regValue = (int)arg;
		rc = key08_ts_read_register16(tsGl,regValue);
		KEY08_PRINTK("key08_ioctl: Register16 %d = %d\n", (int)regValue, (int)rc);
		break;
	case KEY08_IOCTL_END_FIRMWARE:
		KEY08_PRINTK("key08_ioctl: Ending Firmware mode. Just switching some internal flags\n");
		tsGl->client->addr = key08_normal_addr;
		tsGl->mode = KEY08_MODE_NORMAL;
		tsGl->sendMode = KEY08_BL_WAITING_FOR_NOTHING;
		rc = tsGl->mode;
//		rc = key08_calibrate();
		break;
	case KEY08_IOCTL_DISPLAY_ONE_POINT:
		bCount = copy_from_user(kernelBuffer, (char *)arg, (unsigned long)sizeof(A_TOUCH_POINT)); 
		if ( bCount == 0 )
		{
			dataPoint = (A_TOUCH_POINT_PTR) kernelBuffer;

			KEY08_PRINTK("key08_ioctl: touching point: (%d, %d), finger: %d\n", dataPoint->X, dataPoint->Y, dataPoint->finger);
			input_report_abs(tsGl->input_dev, ABS_X, dataPoint->X);
			input_report_abs(tsGl->input_dev, ABS_Y, dataPoint->Y);
			input_report_abs(tsGl->input_dev, ABS_PRESSURE, dataPoint->Z);
			input_report_abs(tsGl->input_dev, ABS_TOOL_WIDTH, dataPoint->W);
			input_report_key(tsGl->input_dev, BTN_TOUCH, dataPoint->finger);
			input_sync(tsGl->input_dev);
			rc = KEY08_MODE_NORMAL;	
		}
		else
		{
			KEY08_PRINTK("key08_ioctl: touching point: copy_from_user() failed. Not touching anything!\n");
		}
		break;
	case KEY08_IOCTL_RESET:
		rc = KEY08_MODE_NORMAL;
		key08_doReset();
		break;
	case KEY08_IOCTL_DEBUG:
		bCount = copy_from_user(kernelBuffer, (char *)arg, (unsigned long)sizeof(char)); 
		if ( bCount == 0 )
				debugOn = kernelBuffer[0];
		rc = 0;
		break;
	case KEY08_IOCTL_CALIBRATE:
		rc = key08_calibrate(tsGl);
		break;
	case KEY08_IOCTL_SETIRQ:
		bCount = copy_from_user(kernelBuffer, (char *)arg, (unsigned long)sizeof(char)); 
		if ( bCount == 0 )
			regValue = kernelBuffer[0];
		if ( regValue )
		{
			enable_irq(tsGl->client->irq);
		}
		else
		{
			disable_irq(tsGl->client->irq);
		}	
		break;
	case KEY08_IOCTL_DISABLE:
			key08_disable_irq(tsGl);
			regValue = 0x00; 
			key08_reg_write (tsGl, Q51001211009AK08_LP_MODE_ADDR,
				sizeof(char), (char *)&regValue);
			clk_busy_wait(3000);
			clk_busy_wait(3000);
			KEY08_PRINTK("%s: Touchscreen is disabled\n", __FUNCTION__);
		break;
	case KEY08_IOCTL_ENABLE:
			KEY08_PRINTK("%s: Enabeling touchscreen\n", __FUNCTION__);
			lpValue = configRegs[Q51001211009AK08_LP_MODE_ADDR];
			key08_reg_write (tsGl, Q51001211009AK08_LP_MODE_ADDR,
				sizeof(char), (char *)&lpValue);
			clk_busy_wait(3000);
			clk_busy_wait(3000);
			key08_calibrate(tsGl);
#ifdef CONFIG_MACH_MOT
		bytes2Read = 6;
#endif
			if ( tsGl->mode == KEY08_MODE_NORMAL )
			{
				lpValue = configRegs[Q51001211009AK08_LP_MODE_ADDR];
				rc = key08_reg_write (tsGl, Q51001211009AK08_LP_MODE_ADDR,
					sizeof(char), (char *)&lpValue);
				clk_busy_wait(3000);
				clk_busy_wait(3000);

				rc = key08_calibrate(tsGl);
#ifdef CONFIG_MACH_MOT

				numNeededReads = 1;
				do {

					rc = i2c_master_recv(tsGl->client, buf, bytes2Read);
					if (rc > 0) {

						clk_busy_wait(3000);
						if (buf[0] == 0xff) {
							KEY08_PRINTK("%s: Got all data.\n", __func__);
							rc = 0;
						} else {
							KEY08_PRINTK("%s: (%d) receiving from 0x%x(number of bytes: %d, type:%d): 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", __func__,
								(int)numNeededReads++, (int)tsGl->client, (int)rc, (int)((buf[0]>>4&0x07)), (int)buf[0], (int)buf[1], (int)buf[2], (int)buf[3], (int)buf[4									], (int)buf[5]);
						}
					}
				} while (rc == bytes2Read);
#endif
			}
			if (tsGl->use_irq)
			{
				key08_enable_irq(tsGl);
			}
			KEY08_PRINTK("%s: Touchscreen is enabled\n", __FUNCTION__);
			rc = 0;
		break;
	case KEY08_IOCTL_GETIRQ:
		rc = tsGl->irqCount;
		KEY08_PRINTK("%s: IRQ State: %s\n", __FUNCTION__, (rc) ? "ON": "OFF");
		break;
	case KEY08_IOCTL_GET_CMD_STATUS:
		KEY08_PRINTK("%s: File transfer status: 0x%x\n",__FUNCTION__,
					tsGl->XferStatus);
		rc = tsGl->XferStatus;
		break;
	case KEY08_IOCTL_SET_CMD_STATUS:
		bCount = copy_from_user(&tsGl->XferStatus, (char *)arg, (unsigned long)sizeof(tsGl->XferStatus)); 
		if ( bCount != sizeof(tsGl->XferStatus) )
			rc = -EFAULT;
/* 
 * This can only be status of the configuration transfer
 * This means we have to put it into the high byte of the status
 */
		KEY08_PRINTK("%s: File transfer status: 0x%x\n",__FUNCTION__,
					tsGl->XferStatus);
		rc = tsGl->XferStatus;
		
		break;

	case KEY08_IOCTL_SUSPEND:
		switch ( arg )
		{
		case 0:
			KEY08_PRINTK("%s: Getting out from suspend mode.\n",__FUNCTION__);
/* Enable sending messages to host, Reg: 184, bit: 5 */
/*
			regValue = Q51001211009AK08_TOUCHSCREEN_LNGTH_ADDR;
			rc = key08_ts_read_register(tsGl,regValue);
			regValue = rc;
			regValue |= 0x10;
			rc = key08_reg_write (tsGl, Q51001211009AK08_TOUCHSCREEN_LNGTH_ADDR,
					sizeof(char), (char *)&regValue);
			key08_enable_irq(tsGl);
*/
			tsGl->suspendMode = FALSE;
			break;
		case 1: 
/* disable sending messages to host, Reg: 184, bit: 5 */
			KEY08_PRINTK("%s: Going into suspend mode.\n",__FUNCTION__);
/*
			regValue = Q51001211009AK08_TOUCHSCREEN_LNGTH_ADDR;
			rc = key08_ts_read_register(tsGl,regValue);
			regValue = rc;
			regValue &= 0xEF;
			rc = key08_reg_write (tsGl, Q51001211009AK08_TOUCHSCREEN_LNGTH_ADDR,
					sizeof(char), (char *)&regValue);
			key08_disable_irq(tsGl);
*/
			tsGl->suspendMode = TRUE;
			break;
		default:
			KEY08_PRINTK("%s: Quering suspend mode. (%d)\n",__FUNCTION__, tsGl->suspendMode);
			break;
		}
		rc = tsGl->suspendMode;
		break;

	case KEY08_IOCTL_GET_POINT:
		bCount = copy_to_user((char *)arg, (char *)currPoints, (unsigned long)sizeof(currPoints));
		if ( bCount != 0 )
			rc = -EFAULT;
		else
		{
			rc = 1;
			KEY08_PRINTK("%s: Getting touchpoint coordinates. (%d)\n",__FUNCTION__,rc) 
		}
		break;
#ifndef CONFIG_MACH_MOT
	case KEY08_IOCTL_SET_TSB_HOLD_MS:
		bCount = copy_from_user(&KEY08_TSB_HOLD_MS, (char *)arg,
			(unsigned long)sizeof(KEY08_TSB_HOLD_MS)); 
		if ( bCount != sizeof(KEY08_TSB_HOLD_MS) )
			rc = -EFAULT;
		break;
	case KEY08_IOCTL_SET_TSB_VIB_MS:
		bCount = copy_from_user(&KEY08_TSB_VIB_MS, (char *)arg,
			(unsigned long)sizeof(KEY08_TSB_VIB_MS)); 
		if ( bCount != sizeof(KEY08_TSB_VIB_MS) )
			rc = -EFAULT;
		break;
#endif
	case KEY08_IOCTL_SET_AREA:
		regValue = (int)arg;
		if ( regValue <= 0 )
			rc = -EFAULT;
		else
		{
			rc = 1;
			KEY08_PRINTK("%s: Setting ABS_TOOL_WIDTH to %d\n", __FUNCTION__,regValue);
			tsGl->area = regValue;
		}
		break;
	case KEY08_IOCTL_SET_GX:
		regValue = (int)arg;
		if ( regValue <= 0 || regValue > SCREEN_X )
			rc = -EFAULT;
		else
		{
			rc = 1;
			KEY08_PRINTK("%s: Setting X-Gutter to %d\n", __FUNCTION__,(int)regValue);
			tsGl->gMinx = regValue;
			tsGl->gMaxx = regValue;
		}
		break;
	case KEY08_IOCTL_SET_GY:
		regValue = (int)arg;
		if ( regValue <= 0 || regValue > SCREEN_Y )
			rc = -EFAULT;
		else
		{
			rc = 1;
			KEY08_PRINTK("%s: Setting Y-Gutter to %d\n", __FUNCTION__,(int)regValue);
			tsGl->gMiny = regValue;
			tsGl->gMaxy = regValue;
		}
		break;
		
	default:
		return -EINVAL;
		break;
	}	
	KEY08_PRINTK("%s: exiting(rc = %d)\n", __FUNCTION__, rc);
	// enable_irq(tsGl->client->irq);
	return rc;
}

/**
 * Write to device
 *
 * @param flip
 * @param buf
 * @param size
 * @param pos
 * @return number of bytes written to the device
 */


static int key08_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos )
{
	int	retval=-1;
	int	i;
	int	reg;
	unsigned long	bCount;

	KEY08_PRINTK("key08_write: entering\n");
	if ( tsGl->mode == KEY08_MODE_BOOTLOADER )
	{
		KEY08_PRINTK("key08_write: Currently in BOOTLOADER mode.\n");
		tsGl->sendMode = KEY08_BL_WAITING_FOR_COMMAND;
		/* get data from user space into kernel space */
		bCount = copy_from_user(kernelBuffer, buf, (unsigned long)count); 
		KEY08_PRINTK("key08_write: copy_from_user() returned %d\n", (int)bCount);
		tsGl->client->addr = key08_bl_addr;
		do
		{
			KEY08_PRINTK("key08_write: record to IC: size=%d\nkey08_write: record data:\n",count); 
			for ( i = 0; i < count; i++)
			{
				if ( !(i%20) )
					KEY08_PRINTK("\nkey08_write: ");
				KEY08_PRINTK("%02x ", kernelBuffer[i]);
			}
			KEY08_PRINTK("\n");
			retval = i2c_master_send (tsGl->client, kernelBuffer, count);

			/* On failure, output the error code and delay before trying again */
			if (retval < 0)
			{
				KEY08_PRINTK(KERN_ERR "key08_write: write failed: %d\n", retval);
				tsGl->sendMode = KEY08_BL_WAITING_FAILED;
			}
			else
			{
				KEY08_PRINTK("key08_write: i2c_master_send(): rc= %d\n",retval);
 				tsGl->sendMode = KEY08_BL_WAITING_FOR_COMMAND;
				retval = count;
			}
			clk_busy_wait(3000);
		}
		while (retval < 0);
	}
	else if ( tsGl->mode == KEY08_MODE_NORMAL )
	{
		KEY08_PRINTK("key08_write: Currently in NORMAL mode.\n");
		/* Get the data from the user space */
		bCount = copy_from_user(kernelBuffer, buf, (unsigned long)count); 
		KEY08_PRINTK("key08_write: copy_from_user() returned %d\n", (int)bCount);

		reg = ((kernelBuffer[0]<<8) | kernelBuffer[1]);
		KEY08_PRINTK("key08_write: config record: size: %d, reg = %d\n",count, reg);
		KEY08_PRINTK("key08_write: ");
		for ( i = 0; i < count; i++)
		{
			char	ch = kernelBuffer[i];
			KEY08_PRINTK("%02X ", ch);
		}
		KEY08_PRINTK("\n");
		tsGl->client->addr = key08_normal_addr;
		configRegs[reg] = kernelBuffer[2];
		configRegs[reg+1] = kernelBuffer[3];
		configRegs[reg+2] = kernelBuffer[4];
		configRegs[reg+3] = kernelBuffer[5];

		if ( (retval = key08_reg_write (tsGl,reg, 4, (char *)&(kernelBuffer[2]))) == 0 )
		{
			KEY08_PRINTK("key08_write: finished writing config info\n");
			retval = 6;
		}
		else
		{
			KEY08_PRINTK("key08_write: finished writing config info\n");
		}
		
	}
	else
		retval = -1;
	KEY08_PRINTK("key08_write: exiting\n");

	return retval;
}

static int __devinit key08_ts_init(void)
{
	debugOn = OFF;
	return i2c_add_driver(&key08_ts_driver);
	
}

static void __exit key08_ts_exit(void)
{
	i2c_del_driver(&key08_ts_driver);
}

/*!
 * @brief Logs data
 *
 * This function is called as a replacement to printk
 *
 * @param fmt Text of message to log
 */

static void key08_printk (char *fmt, ...)
{
	if ( debugOn )
	{
		static va_list args;
		va_start(args, fmt);
		vprintk(fmt, args);
		va_end(args);
	}
}

late_initcall(key08_ts_init);
// subsys_initcall(key08_ts_init);
module_exit(key08_ts_exit);

MODULE_DESCRIPTION("key08 Touchpad Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Boris Dubinsky <Boris.Dubinsky@motorola.com>");
MODULE_VERSION("1.0");
