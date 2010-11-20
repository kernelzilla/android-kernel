/* drivers/input/keyboard/qtm_obp_touch.c
		  }
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

#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/uaccess.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "qtm_obp_ts.h"

#ifndef FALSE
#define	FALSE	0
#define	TRUE	(!FALSE)
#endif

#define GOOGLE_MULTITOUCH 1

// Optimized for multitouch
#define KEY08_PRINTK(args...) \
				if (qtm_debugOn) qtm_obp_printk(args);

extern unsigned long msleep_interruptible(unsigned int);

#define clk_busy_wait(time)	msleep_interruptible(time/1000)

#define swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

#define KEY08_TS_RESET 84
#define KEY08_TS_GPIO 41

#define KEY08_I2C_NAME "qtm_obp"
#define KEY08_PF_NAME	"touchpad"

#define KEY08_REG_ADDR_SIZE    2

enum 
{
	KEY08_FLIP_X = 1UL << 0,
	KEY08_FLIP_Y = 1UL << 1,
	KEY08_SWAP_XY = 1UL << 2,
	KEY08_SNAP_TO_INACTIVE_EDGE = 1UL << 3,
};

// Local Functions
static int qtm_obp_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static void qtm_obp_slider_event(struct input_handle *, unsigned int, unsigned int, int );
static int qtm_obp_ts_remove(struct i2c_client *);
static int qtm_obp_slider_connect(struct input_handler *, struct input_dev *, const struct input_device_id*);
static void qtm_obp_slider_disconnect(struct input_handle *);
static  int qtm_obp_open(struct inode *inode, struct file *filp);
static int qtm_obp_ioctl(struct inode *node, struct file *filp, unsigned int cmd, unsigned long arg);
static int qtm_obp_release(struct inode *inode, struct file *filp);
static int qtm_obp_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos );
static void qtm_obp_load_obp_objects(void);
static int qtm_obp_send_message_processor_pointer(void);
static void qtm_obp_printk (char *, ...);
static void qtm_obp_doReset(void);
static void qtm_obp_hardReset(void);
static int qtm_obp_get_touch_sens_data (char * arg);
static int save_low_power_regs(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qtm_obp_ts_early_suspend(struct early_suspend *h);
static void qtm_obp_ts_late_resume(struct early_suspend *h);
#endif

// this structure maintains the current status of the touchscreen device.
// it is setup as a global in the driver

struct	coordinate_map
{
	int x_data;
	int y_data;
	int z_data;
	int w_data;
	int down;
};

#define	_NUM_FINGERS	10

struct qtm_obp_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *sw_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	struct coordinate_map	finger_data[_NUM_FINGERS];
	uint8_t XferStatus;
	int     cntBadCRC;
	uint16_t max[2];
	int snap_down[2];
	int snap_up[2];
	uint32_t flags;
	char	mode;
	bool	useData;
	bool	inCall;
        bool    suspendMode;
	char	sendMode;
	char	calibrate;
	int		irqCount;
	int (*power)(int on);
        struct mutex transaction_mutex;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static const struct i2c_device_id qtm_obp_ts_id[] = {
	{ KEY08_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver qtm_obp_ts_driver = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= qtm_obp_ts_suspend,
	.resume		= qtm_obp_ts_resume,
#endif
	.probe	= qtm_obp_ts_probe,
	.remove	= qtm_obp_ts_remove,
	.id_table	= qtm_obp_ts_id,
	.driver = {
		.name	= KEY08_I2C_NAME,
	},
};

static const struct input_device_id slider_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_SW) },
	},
	{ },
};

MODULE_DEVICE_TABLE(input,slider_ids);

static struct input_handler slider_handler = {
	.event =	qtm_obp_slider_event,
	.connect = 	qtm_obp_slider_connect,
	.disconnect = 	qtm_obp_slider_disconnect,
	.name	= "qtm_obp_slider",
	.id_table = slider_ids,
};

struct	file_operations	qtm_obp_fops =
{
	.owner		= THIS_MODULE,
	.open		= qtm_obp_open,
	.ioctl		= qtm_obp_ioctl,
	.release	= qtm_obp_release,
	.write		= qtm_obp_write,
};

static struct miscdevice qtm_obp_pf_driver = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = KEY08_PF_NAME,
	.fops = &qtm_obp_fops,
};

typedef struct  
{
	uint16_t reg;
	uint8_t  data[4]; 
} KEY08_INIT_DATA_T;

typedef struct {
   uint8_t  type;
   uint16_t start_pos;
   uint8_t  obj_size;
   uint8_t  num_of_instances;
   uint8_t  num_of_report_ids;
   uint8_t  report_id_low;
   uint8_t  report_id_high;
}QTM_OBJ_TABLE_ELEMENT_T, *QTM_OBJ_TABLE_ELEMENT_PTR;

typedef union {
    struct {
    uint8_t  family_id;
    uint8_t  variant_id;
    uint8_t  version;
    uint8_t  build;
    uint8_t  matrix_x_size;
    uint8_t  matrix_y_size;
    uint8_t  num_of_objects;
    }qtm_info_bytes;
    unsigned char        qtm_info_buf[QTM_OBP_INFO_BLOCK_SIZE];
}U_QTM_INFO_BYTES_T, *U_QTM_INFO_BYTES_PTR;

typedef struct {
    uint8_t  obj_id_cfg_size[QTM_OBP_MAX_OBJECT_NUM];
    uint8_t  obj_cfgs[QTM_OBP_MAX_OBJECT_NUM];
    uint8_t  *object[QTM_OBP_MAX_OBJECT_NUM];
}OBJ_ID_T, *OBJ_ID_PTR;


// GLOBALS

/*! @brief Variable used to define if the IC is calibrating or not */
static unsigned char calibrating = 0;
U_QTM_INFO_BYTES_T        qtm_obp_info_block;
QTM_OBJ_TABLE_ELEMENT_T   qtm_obp_obj_table_elem[QTM_OBP_MAX_OBJECT_NUM];

int qtm_pos[2][2];
struct qtm_obp_ts_data *qtmTsGl;
int	qtm_debugOn;

int	qtm_obp_normal_addr;
int	qtm_obp_bl_addr;
static	char	lpIdleValueEeprom = 14;
static	char	lpActiveValueEeprom = 14;
static	int	prevX;
static	int	prevY;
static	int	threshX;
static	int	threshY;
int cntSigerr = 0;

int	qtm_lastKeyPressed;

/*! @brief Global value to keep track of which finger id pressed or released - sajid added for multi touch*/
static int finger_press_release = 0;


// This function is used to write to a register with a given value
// The write function is common between the 2 protocols (original and OBP)
// byte[0] = LSByte of 2 byte register addr
// byte[1] = MSByte of 2 byte register addr
// byte[2] = reg_value[0]
// byte[len-1] = reg_value[len-1]
// The i2c_master_send will return the number of bytes written (reg_addr_size+len)
// but function will return 0 on success

int qtm_obp_reg_write (struct i2c_client *client, unsigned int reg, unsigned int length, unsigned char *reg_value)
{
	unsigned char value[24];
	int retval = 0;
	int i = 10;

        KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);

	/* Copy the data into a buffer for correct format */
	value[0] = reg & 0xFF;
	value[1] = reg >> 8;
	memcpy(&value[2], reg_value, length);

	KEY08_PRINTK("qtm_obp_reg_write: memcpy(): OK\n");

	/* Write the data to the device (retrying if necessary) */
	do
	{
		KEY08_PRINTK("qtm_obp_reg_write: trying to write to 0x%x\n", client->addr);
		retval = i2c_master_send (client, value, KEY08_REG_ADDR_SIZE + length);

		/* On failure, output the error code and delay before trying again */
		if (retval < 0)
		{
			KEY08_PRINTK(KERN_ERR "qtm_obp_reg_write: write of reg 0x%X failed: %d\n", reg, retval);
			clk_busy_wait(3000);
		}
		else
			KEY08_PRINTK("qtm_obp_reg_write: i2c_master_send(): OK\n");
	}
	while ((retval < 0) && (i-- >= 0));

	KEY08_PRINTK("qtm_obp_reg_write: i2c_master_send(): Done\n");

	/* On success, set retval to 0 (i2c_master_send returns no. of bytes transfered) */
	if (retval == (KEY08_REG_ADDR_SIZE + length))
	{
		retval = 0;
	}

	/* Delay after every I2C access or IC will NAK */
	clk_busy_wait(3000);

	if ( qtmTsGl->mode == KEY08_MODE_OBP )
	  {
	    qtm_obp_send_message_processor_pointer();
	  }

	/* Delay after every I2C access or IC will NAK */
	clk_busy_wait(3000);

        KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
	return retval;
}

// This function is used to read from a specified register.
// In OBP mode
//   byte[0] = LSByte of 2 byte register addr
//   byte[1] = MSByte of 2 byte register addr
//   this data is sent to the IC via I2C
//   wait 3 msec 
//   read back 1 byte of data over I2C

static uint8_t qtm_obp_ts_read_register(struct i2c_client *client , int reg )
{    
  uint8_t data[6];
  int   rc;
  uint8_t reg_value=0;

  KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
     
  // setup the register to read
  data[0] = reg & 0xFF;
  data[1] = (reg & 0xFF00)>>8;

  rc = i2c_master_send(client, data, QTM_OBP_REG_ADDR_SIZE);
  if (rc < 0)
    {
      KEY08_PRINTK(KERN_ERR "i2c_master_send error  %d\n,",rc);
    }
  else
    {
      clk_busy_wait(3000);
      
      // clear the data to be read back
      memset(data, 0, sizeof(data));

      // read 1 byte of data
      rc = i2c_master_recv(client, data, 1);

      if (rc < 0)
	{
	  KEY08_PRINTK(KERN_ERR "i2c_master_recv error  %d\n,",rc);
	}
      else 
	    {
	      reg_value = data[0];
	      KEY08_PRINTK("qtm_obp_obp_ts_read_register reg=0x%x value=0x%X\n",reg,reg_value);
	    }
    }

  /* Delay after every I2C access or IC will NAK */
  clk_busy_wait(3000);

  if ( qtmTsGl->mode == KEY08_MODE_OBP )
    {
      qtm_obp_send_message_processor_pointer();
    }

  /* Delay after every I2C access or IC will NAK */
  clk_busy_wait(3000);

  
  KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
  return reg_value;
}

// This function is used to calibrate the IC
//   write a non-zero value to the calibrate register, which is in the command processor object

static int qtm_obp_calibrate(struct i2c_client *client)
{
	char	regValue = 0x01; 
	int		ret = 0;

        KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);

        if ( qtmTsGl->mode == KEY08_MODE_OBP)
        {
	  ret = qtm_obp_reg_write (client,
                                   qtm_obp_obj_table_elem[OBJ_ID_GEN_COMMANDPROCESSOR_T6].start_pos+QTM_OBP_T6_CALIBRATE, sizeof(regValue), (char *)&regValue);
        }
	
	clk_busy_wait(6000);

        KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
	return ret;
}

static int qtm_obp_init_panel(struct qtm_obp_ts_data *ts)
{
	int ret=0;
        KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
        KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
	return ret;
}

// This function is used to reset the IC
// This needs to change to write to command processor reset register since the reset line is shared with minipad ???
static void qtm_obp_doReset(void)
{
    int     ret;
    int     regValue = 0x01;

    KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);

    ret = qtm_obp_reg_write (qtmTsGl->client, qtm_obp_obj_table_elem[OBJ_ID_GEN_COMMANDPROCESSOR_T6].start_pos,
                             sizeof(char), (char *)&regValue);

    clk_busy_wait(6000);
    KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
}

static int qtm_obp_save_config(void)
{
    int     ret;
    int     regValue;
    regValue = QTM_OBP_T6_BACKUP_CMD;

    KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
    ret = qtm_obp_reg_write (qtmTsGl->client, qtm_obp_obj_table_elem[OBJ_ID_GEN_COMMANDPROCESSOR_T6].start_pos + QTM_OBP_T6_BACKUPNV,
                           sizeof(char), (char *)&regValue);

    clk_busy_wait(6000);
    KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
    return ret;
}

static void qtm_obp_hardReset(void)
{
   	gpio_set_value(84,0);
	clk_busy_wait(3000);
	gpio_set_value(84,1);
}

// This is the work function for the driver
static void qtm_obp_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	uint8_t buf[15];
	int	x,y,z,w,finger,press_release_flag;
	ktime_t	t1,t2;
	int i2c_retry = QTM_OBP_READ_RETRIES;
	int	down;

	KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);

	qtmTsGl = container_of(work, struct qtm_obp_ts_data, work);

	KEY08_PRINTK("%s: Locking transaction mutex\n",__FUNCTION__);
	mutex_lock(&qtmTsGl->transaction_mutex);
	KEY08_PRINTK("%s: Obtained transaction mutex\n",__FUNCTION__);
	
	t1 = ktime_get();

        // Check if we are bootloader mode

	if ( qtmTsGl->mode == KEY08_MODE_BOOTLOADER )
	{
                KEY08_PRINTK("qtm_obp_work_func: trying to read to 0x%x\n", qtmTsGl->client->addr);

		ret = i2c_master_recv(qtmTsGl->client, buf,1);
		clk_busy_wait(3000);

		if (ret < 0)
		{
			/* Receive failed. Reset status  */
			printk("qtm_obp_ts_work_func: in Bootloader mode i2c_master_recv failed\n");
			qtmTsGl->sendMode = KEY08_BL_WAITING_FOR_NOTHING;
		}
		else
		{
			KEY08_PRINTK("qtm_obp_ts_work_func: buf[0] = 0x%x\n", buf[0]);

			/* Determine which code we got and set status appropriately */
			if ( (buf[0] & 0xF0) == 0xC0 )
			{
				qtmTsGl->sendMode = KEY08_BL_WAITING_FOR_COMMAND;
			}
			else if ( (buf[0] & 0xF0) == 0x80 )
			{
				if ( qtmTsGl->sendMode == KEY08_BL_GOT_BAD_CRC )
					qtmTsGl->sendMode = KEY08_BL_WAITING_AFTER_BAD_CRC;
				else
					qtmTsGl->sendMode = KEY08_BL_WAITING_FOR_DATA;
			}
			else if ( buf[0] == 0x02 )
			{
				qtmTsGl->sendMode = KEY08_BL_WAITING_FOR_CRC;
			}
			else if ( buf[0] == 0x04 )
			{
				qtmTsGl->sendMode = KEY08_BL_WAITING_AFTER_GOOD_CRC;
			}
			else if ( buf[0] == 0x03 )
			{
				/* We got bad CRC on the record */
				qtmTsGl->sendMode = KEY08_BL_GOT_BAD_CRC;
                                qtmTsGl->cntBadCRC++;

			}
                        else
                        {
			      /* We should never get here ! */ 
                              printk("qtm_obp_ts_work_func: **NOTHING** buf[0] = 0x%x\n", buf[0]);
                        }
		}
	}
	else if ( qtmTsGl->mode == KEY08_MODE_OBP )
	{
	  KEY08_PRINTK("in OBP mode\n");

	  /* Read the data from the device (retrying if necessary) */
	  do
	    {
	      memset(buf,0, QTM_OBP_READ_DATA_SIZE);
              qtm_obp_send_message_processor_pointer();  // ??? NEED TO CHECK ???
	      ret = i2c_master_recv(qtmTsGl->client, buf,  QTM_OBP_READ_DATA_SIZE); // ??? NEED TO CHECK ???
	      
	      if (ret < 0) 
		{
		  KEY08_PRINTK("qtm_obp_obp_ts_work_func: i2c_master_recv failed\n");
		  clk_busy_wait(QTM_OBP_READ_WRITE_DELAY);
		}
	    }
	  while ((ret < 0) && (i2c_retry-- >= 0));
	  
          KEY08_PRINTK("qtm_obp_obp_ts_work_func: Buffer\n");
          for (i=0;i<ret;i++)
            KEY08_PRINTK("buf[%d] = 0x%x\n", i, buf[i]);

	  if(ret < 0)
	    {
	      KEY08_PRINTK("qtm_obp_ts_obp_work_func: I2C failed to read data from the device\n");
	      goto exit_func;
	    }

	  /* Check for OBJ_ID_GEN_COMMANDPROCESSOR_T6 report IDs*/
	  if ( buf[QTM_OBP_REPORT_ID_BYTE] >= qtm_obp_obj_table_elem[OBJ_ID_GEN_COMMANDPROCESSOR_T6].report_id_low \
	       && buf[QTM_OBP_REPORT_ID_BYTE] <= qtm_obp_obj_table_elem[OBJ_ID_GEN_COMMANDPROCESSOR_T6].report_id_high )
	  {
	      if( (buf[QTM_OBP_T6_STATUS] & QTM_OBP_T6_STATUS_RESET) == QTM_OBP_T6_STATUS_RESET )
		{
		  KEY08_PRINTK("qtm_obp_obp_ts_work_func: T6-Reset\n");

                  ret =  qtm_obp_save_config();  // ??? NEED TO CHECK ???

		  /* Set touch mode */
		  /* OBP : send the message processor pointer */
                  qtm_obp_send_message_processor_pointer();	  
		}
	     
#if 0 
	      /* Check for Calibration.  If we are calibrating, indicate in a message */
	      if ((calibrating == 0) && (buf[QTM_OBP_T6_STATUS] & QTM_OBP_T6_STATUS_CAL))
		{
		  calibrating = 1;
		  KEY08_PRINTK("qtm_obp_obp_ts_work_func: The IC is calibrating.\n");
		}

#endif
	      /* Check for Calibration.  If we are calibrating, indicate in a message */
	    if ((buf[QTM_OBP_T6_STATUS] & QTM_OBP_T6_STATUS_CAL))
		{
		  calibrating = 1;
		  KEY08_PRINTK("qtm_obp_obp_ts_work_func: The IC is calibrating.\n");
		}
	    else if ((calibrating != 0) && 
		       ((buf[QTM_OBP_T6_STATUS] & QTM_OBP_T6_STATUS_CAL) == 0))
		{
		  calibrating = 0;
		  KEY08_PRINTK("qtm_obp_obp_ts_work_func: The IC is done calibrating.\n");
		  clk_busy_wait(3000);
		}
	      
	      /* Check for a possible overflow from the HW.  If there has been an overflow, log a message. */
	    if (buf[QTM_OBP_T6_STATUS] & QTM_OBP_T6_STATUS_OFL)
		{
		  KEY08_PRINTK("qtm_obp_obp_ts_work_func: An overflow has occurred.\n");
		}
	      
	      /* SIGERR: Error in acquisition */
	    if (buf[QTM_OBP_T6_STATUS] & QTM_OBP_T6_STATUS_SIGERR)
		{
		  KEY08_PRINTK("qtm_obp_obp_ts_work_func: An SIGERR has occurred.\n");
                  cntSigerr++;
                  if (cntSigerr > 1000) {
                     ret = qtm_obp_calibrate(qtmTsGl->client);    
		     cntSigerr = 0;
		  }
		}
	      
	      /* CFGERR: Error with object setup */
	    if (buf[QTM_OBP_T6_STATUS] & QTM_OBP_T6_STATUS_CFGERR)
		{
		  KEY08_PRINTK("qtm_obp_obp_ts_work_func: An CFGERR has occurred.\n");
		}
	  }

	  /* Check for OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9 report IDs*/
	  else if ( buf[QTM_OBP_REPORT_ID_BYTE] >= qtm_obp_obj_table_elem[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9].report_id_low \
		    && buf[QTM_OBP_REPORT_ID_BYTE] <= qtm_obp_obj_table_elem[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9].report_id_high )
	  {
	      /* Align X and Y LSB and MSB bits for 10-bit PTR */
	      x = buf[QTM_OBP_T9_XPOS_MSB];
	      x = (x << QTM_OBP_T9_XYPOS_LSB_POS_SHIFT) |	\
		((buf[QTM_OBP_T9_XYPOS_LSB] & QTM_OBP_T9_XYPOS_LSB_XPOS_MASK) >> QTM_OBP_T9_XYPOS_LSB_POS_SHIFT);
	      x = x >> 2;
	      
	      y = buf[QTM_OBP_T9_YPOS_MSB];
	      y = (y << QTM_OBP_T9_XYPOS_LSB_POS_SHIFT) | (buf[QTM_OBP_T9_XYPOS_LSB] & QTM_OBP_T9_XYPOS_LSB_YPOS_MASK);
	      y = y >> 2;
	      
	      /*
	      ** X and Y are flipped on the sensor panel so manipulate the data so
	      ** X data is Y and Y data is reported as X.
	      ** Swap the x y data due to sensor orientation
	      */
	      swap(x, y);
	      
	      /* Reverse the x or y value based on sensor to screen layout */
	      // for Zeppelin remove the x stuff
           // x = Q51001211009AK08_TS_MAX_X - x; /* UNCOMMENT MORRISON & MOTUS */
           // x = Q51001211009AK08_TS_MAX_X - x; /* COMMENT Zeppelin -- Need to Clean this up ??? */
	      y = Q51001211009AK08_TS_MAX_X - y;
	      
	      /* capture width/area */
	      w = buf[QTM_OBP_T9_TCH_AREA];
	      
	      /* capture pressure/amplitude */
	      z = buf[QTM_OBP_T9_TCH_AMPLI];
	      
	      /* Report touch */
	      finger = (buf[QTM_OBP_REPORT_ID_BYTE] - qtm_obp_obj_table_elem[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9].report_id_low) ; //removed the +1;
		  
	      if( ((buf[QTM_OBP_T9_STATUS] & QTM_OBP_T9_STATUS_TOUCH_MASK) == QTM_OBP_T9_STATUS_TOUCH_MASK ) \
		  || ((buf[QTM_OBP_T9_STATUS] & QTM_OBP_T9_STATUS_PRESS_MASK) == QTM_OBP_T9_STATUS_PRESS_MASK ) \
		  || ((buf[QTM_OBP_T9_STATUS] &  QTM_OBP_T9_STATUS_MOVE_MASK) == QTM_OBP_T9_STATUS_MOVE_MASK ))
		{
		  press_release_flag = 1;
		  finger_press_release |= ( 1 << finger ); 
		}
	      
	      if( (buf[QTM_OBP_T9_STATUS] & QTM_OBP_T10_STATUS_RELEASE_MASK) == QTM_OBP_T10_STATUS_RELEASE_MASK)
		{
		  press_release_flag = 0;
		  finger_press_release &= ~( 1 << finger ); 
		}
	      
	    KEY08_PRINTK("qtm_obp_obp_ts_work_func: input report : finger=%d x=%d, y=%d, z=%d, w=%d, press=0x%x, ABS_MISC=0x%x\n",
			   finger, (x*SCREEN_X/KEY08_MAX_X),(y*SCREEN_Y/KEY08_MAX_Y),z,w,press_release_flag,
			   ((finger<<16)|finger_press_release));
		down = press_release_flag;
#ifndef GOOGLE_MULTITOUCH
	    input_report_abs(qtmTsGl->input_dev, ABS_MISC,
			       ( (finger << 16) | finger_press_release )); 
	      
	    input_report_abs(qtmTsGl->input_dev, ABS_X, (x * SCREEN_X / KEY08_MAX_X));
	    input_report_abs(qtmTsGl->input_dev, ABS_Y, (y * SCREEN_Y / KEY08_MAX_Y));
	    input_report_abs(qtmTsGl->input_dev, ABS_PRESSURE, z);
	    input_report_abs(qtmTsGl->input_dev, ABS_TOOL_WIDTH, w);
	    input_report_key(qtmTsGl->input_dev, BTN_TOUCH, press_release_flag);
	      
#else
	/* The chip may report erroneous points way
	beyond what a user could possibly perform so we filter
	these out */
	if (qtmTsGl->finger_data[finger].down &&
			(abs(qtmTsGl->finger_data[finger].x_data - x) > 400 ||
			abs(qtmTsGl->finger_data[finger].y_data - y) > 250)) 
	{
		down = 0;
		KEY08_PRINTK("%s: x0 %i x1 %i y0 %i y1 %i\n",
				__func__,
				qtmTsGl->finger_data[finger].x_data, x,
						qtmTsGl->finger_data[finger].y_data, y);
	} 
	else 
	{
		qtmTsGl->finger_data[finger].x_data = x;
		qtmTsGl->finger_data[finger].y_data = y;
		qtmTsGl->finger_data[finger].w_data = w;
	}

	/* The touch IC will not give back a pressure of zero
	   so send a 0 when a liftoff is produced */
	qtmTsGl->finger_data[finger].down = down;
	if (!down) 
	{
		qtmTsGl->finger_data[finger].z_data = 0;
	} 
	else 
	{
		qtmTsGl->finger_data[finger].z_data = 1;
	}

	for (i = 0; i < 2; i++) 
	{
/*
		if (qtmTsGl->finger_data[i].down == 0)
			continue;
*/
		input_report_abs(qtmTsGl->input_dev, ABS_MT_TOUCH_MAJOR,
				 qtmTsGl->finger_data[i].z_data);
		input_report_abs(qtmTsGl->input_dev, ABS_MT_WIDTH_MAJOR,
				 qtmTsGl->finger_data[i].w_data);
		input_report_abs(qtmTsGl->input_dev, ABS_MT_POSITION_X,
				 qtmTsGl->finger_data[i].x_data);
		input_report_abs(qtmTsGl->input_dev, ABS_MT_POSITION_Y,
				 qtmTsGl->finger_data[i].y_data);
		input_mt_sync(qtmTsGl->input_dev);
	}
	input_sync(qtmTsGl->input_dev);
	
	if (!down) 
	{
		memset(&qtmTsGl->finger_data[finger], 0, sizeof(struct coordinate_map));
	}

#endif
	    input_sync(qtmTsGl->input_dev);
	      
	    } /* Check for OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9 report IDs*/

          /* OBJ_ID_TOUCH_KEYARRAY_T15 */
          else if ( buf[QTM_OBP_REPORT_ID_BYTE] >= qtm_obp_obj_table_elem[OBJ_ID_TOUCH_KEYARRAY_T15].report_id_low \
                    && buf[QTM_OBP_REPORT_ID_BYTE] <= qtm_obp_obj_table_elem[OBJ_ID_TOUCH_KEYARRAY_T15].report_id_high )
            {
                  // Check if press

                  if ( buf[QTM_OBP_T15_STATUS] & QTM_OBP_T15_KEY_PRESS_MASK)
                  {
                       if ( buf[QTM_OBP_T15_CHANNEL] == QTM_OBP_T15_BACK_KEY)
                       {
                           input_report_key(qtmTsGl->input_dev, 158, 1);
                           qtm_lastKeyPressed = 158;
                       }
                       if ( buf[QTM_OBP_T15_CHANNEL] == QTM_OBP_T15_HOME_KEY)
                       {
                           input_report_key(qtmTsGl->input_dev, 102, 1);
                           qtm_lastKeyPressed = 102;
                       }
                       if ( buf[QTM_OBP_T15_CHANNEL] == QTM_OBP_T15_MENU_KEY)
                       {
                           input_report_key(qtmTsGl->input_dev, 139, 1);
                           qtm_lastKeyPressed = 139;
                       }
                  }
                  else
                  {
                       
		    if ( qtm_lastKeyPressed != 0 &&  //  buf[QTM_OBP_T15_CHANNEL] == 0 ???
                            buf[QTM_OBP_T15_CHANNEL] != 0x02 && 
                            buf[QTM_OBP_T15_CHANNEL] != 0x04 && 
                            buf[QTM_OBP_T15_CHANNEL] != 0x10 )
                       {
                           input_report_key(qtmTsGl->input_dev, qtm_lastKeyPressed, 0 );
                           qtm_lastKeyPressed = 0;
                       }
                       input_sync(qtmTsGl->input_dev);
                  }

		  KEY08_PRINTK("qtm_obp_obp_ts_work_func: key report : press=0x%x key=0x%x qtm_lastKeyPressed=0x%x\n",buf[1],buf[2],qtm_lastKeyPressed);


            } /* OBJ_ID_TOUCH_KEYARRAY_T15 */
          
	  else
	    {
	      KEY08_PRINTK("qtm_obp_obp_ts_work_func: Message is default.  Data sent is 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n", \
			  buf[0], buf[1], buf[2],buf[3], buf[4], buf[5]);
              qtm_obp_send_message_processor_pointer();
	    }
	  
	} // if ( qtmTsGl->mode == KEY08_MODE_OBP )

exit_func:

	KEY08_PRINTK("%s: Unlocking transaction mutex\n",__FUNCTION__);
	mutex_unlock(&qtmTsGl->transaction_mutex);
	KEY08_PRINTK("%s: Released transaction mutex\n",__FUNCTION__);
	
	if (qtmTsGl->use_irq) 
	{
		KEY08_PRINTK("%s: Enabling irq %d\n", __FUNCTION__, qtmTsGl->client->irq);
		enable_irq(qtmTsGl->client->irq);
	}

	t2 = ktime_get();

	KEY08_PRINTK("qtm_obp_ts_work: final total %llu\n", ktime_to_ns(t2) - ktime_to_ns(t1));
	KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
}

// Used to poll instead of irq .. not really used
static enum hrtimer_restart qtm_obp_ts_timer_func(struct hrtimer *timer)
{
  struct qtm_obp_ts_data *ts = container_of(timer, struct qtm_obp_ts_data, timer);
  KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
  
  schedule_work(&ts->work);
  
  hrtimer_start(&ts->timer, ktime_set(0, 1250000000), HRTIMER_MODE_REL);
  KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
  return HRTIMER_NORESTART;
}

// This is the IRQ handler
static irqreturn_t qtm_obp_ts_irq_handler(int irq, void *dev_id)
{
  struct qtm_obp_ts_data *ts = dev_id;
  
  KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
  KEY08_PRINTK("%s: Disable irq %d\n",__FUNCTION__, ts->client->irq);
  disable_irq(ts->client->irq);
  
  schedule_work(&ts->work);
  KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
  return IRQ_HANDLED;
}

// This is used to read the version
// In OBP mode read the version at offset 2 in the info block
static int  qtm_obp_ts_read_app_version(void)
{
  struct	i2c_client *client;
  uint8_t data[6];
  int	reg;
  int	rc=0;
  int	version = -1;
  
  KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
  KEY08_PRINTK("qtm_obp_ts_read_app_version: qtmTsGl: 0x%x, qtmTsGl->client: 0x%x\n", (int)qtmTsGl, (int)qtmTsGl->client);
  client = qtmTsGl->client;
  
  if ( qtmTsGl->mode == KEY08_MODE_OBP )
    {
      reg = QTM_OBP_INFO_BLOCK_VERSION;
      version = qtm_obp_ts_read_register(client , reg);
      KEY08_PRINTK("qtm_obp_ts_read_app_version: Got: %d from reading reg 0x%x\n", version,reg);
      
    }
  else
    {
      /* we are in bootloader mode. So, read 1 byte from bootloader client */
      client->addr = qtm_obp_bl_addr;;
      if ((rc = i2c_master_recv(client, data, 1 )) < 0 )
	{
	  KEY08_PRINTK("qtm_obp_ts_read_app_version: i2c_master_revc bootloader error  %d\n,",rc);
	}
      else 
	{
	  version = data[0];
	  KEY08_PRINTK("qtm_obp_ts_read_app_version: In bootloader mode version 0x%x\n", version);
	}
    }
  KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
  return(version);
}

// This is the probe function that is initially called to "detect" the device
int qtm_obp_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct platform_device *pdev;
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

        printk("%s: Entering.....\n", __FUNCTION__);

	prevX = 0;
	prevY = 0;
	threshX = 20;
	threshY = 20;
	ret = misc_register(&qtm_obp_pf_driver);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	qtmTsGl = kzalloc(sizeof(*qtmTsGl), GFP_KERNEL);
	if (qtmTsGl == NULL) 
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	INIT_WORK(&qtmTsGl->work, qtm_obp_ts_work_func);

	qtmTsGl->client = client;
	i2c_set_clientdata(client, qtmTsGl);
	client->flags = 0;
	qtmTsGl->calibrate = FALSE;
	qtmTsGl->suspendMode = FALSE;
	qtmTsGl->irqCount = 0;
        qtmTsGl->XferStatus = KEY08_FM_DOWNLOAD_NOT_STARTED | KEY08_CFG_DOWNLOAD_NOT_STARTED;
        qtmTsGl->cntBadCRC = 0;

	// initialize i2c addresses 

	qtm_obp_normal_addr = 0x11;
	qtm_obp_bl_addr = 0x5F;

	// initialize transaction mutex
        mutex_init(&qtmTsGl->transaction_mutex);

	pdev = (struct platform_device *)client->dev.platform_data;
	if ( pdev && pdev->num_resources && (!strcmp(pdev->name,"qtm_obp")))
	{
		struct	resource *resource;
		printk("qtm_obp_ts_probe: paltform data for '%s', num_resources %d\n",
			pdev->name, pdev->num_resources);
		resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if ( resource)
		{
			qtm_obp_normal_addr = resource->start;
			qtm_obp_bl_addr = resource->end;
			printk("qtm_obp_ts_probe: Got resource. Normal Addr: 0x%x, Bootloader: 0x%x\n",
				qtm_obp_normal_addr, qtm_obp_bl_addr);
		}
		else
		{
			printk("qtm_obp_ts_probe: Could not optain GPIO information. Using default values\n");
		}
	} 
	else
	{
		printk("qtm_obp_ts_probe: Could not get platform reousrces. Using default values\n");
			
	}

	printk("qtm_obp_ts_probe: attached, client: 0x%x\n", (unsigned)client);
	printk("qtm_obp_ts_probe: irq: 0x%x\n", client->irq);

	inactive_area_left = 0;
	inactive_area_right = 0;
	inactive_area_top = 0;
	inactive_area_bottom = 0;
	fuzz_x = 0;
	fuzz_y = 0;
	fuzz_p = 0;
	fuzz_w = 0;

	if (qtmTsGl->power) 
	{
		ret = qtmTsGl->power(1);
		if (ret < 0) 
		{
			KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}
	KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe STARTED\n");
	KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe Getting Versions\n");

/*
   This hardware requires to read all data from the buffers when it starts.
   Until all data is read, it will not assert the IRQ to high. So, we need to
   read bunch of sets of 6 bytes until there are no more.

   NOTE: if we faile to read version, we may be in the bootloader mode. In that case,
   we have to read from different address.
*/

/* Determine the current operation mode (Normal, OBP, Bootloader)
   Read 6 bytes from i2c_normal_addr
   if read fails, we check to see if we are in bootloader mode
     we read 1 byte from i2c_bl_addr
     if read fails, we assume no flip is connected and set mode to UNKNOWN
     if read passes, we send unlock command to bootloader and set mode to BOOTLOADER.
        remianing bootloader processing will be handled by the work function
   if read passes
     set mode to NORMAL
     check if first byte read is 0xFF. this implies the buffer is empty and we are in NORMAL mode.
     check if first byte read is 0x70. this implies the chip in normal mode is sending status messages. keep reading these messages.
     if first byte read is neither 0xFF nor 0x70, we can assume we are in OBP mode.
   if we are in here for more than 20 loops, we assume something bad has happened and get out with an error message
 */

	numNeededReads = 1;
	do
	{
		clk_busy_wait(3000);
		memset(data,0,sizeof(data));

		if ((ret = i2c_master_recv(client, data, 6 )) < 0 )
		{
			printk("qtm_obp_ts_probe: i2c_master_recv (normal client) error  %d\n",ret);
			client->addr = qtm_obp_bl_addr;
			if ((ret = i2c_master_recv(client, data, 1 )) < 0 )
			{
				printk("qtm_obp_ts_probe: i2c_master_recv (bootloader client) error  %d\n",ret);
				printk("qtm_obp_ts_probe: Cause: touch device is probably missing. \n");
				qtmTsGl->mode = KEY08_MODE_UNKNOWN;
				return ret;
			}
			else
			{
				printk("qtm_obp_ts_probe: Firmware Not Present, Firmware loading will start now ");
				printk("qtm_obp_ts_probe: i2c_master_recv (bootloader client) data: 0x%x\n",data[0]);
				printk("qtm_obp_ts_probe: entering bootloader mode\n");
				client->addr = qtm_obp_bl_addr;
				qtmTsGl->mode = KEY08_MODE_BOOTLOADER;
				/* wait until we get 0x8n */
				while ( (data[0] & 0xF0) != 0x80 )
				{
					printk("qtm_obp_ts_probe: Sending Unblock command (0xDC,0xAA)\n");
					data[0] = 0xDC;
					data[1] = 0xAA;
					ret = i2c_master_send (client, data, 2);
					clk_busy_wait(3000);
					ret = i2c_master_recv(client, data, 1);
					if ( ret < 0 )
					{
						printk("qtm_obp_ts_probe: i2c_master_recv (bootloader client waiting for 0x8n) error  %d\n",ret);
						return ret;
					}
					clk_busy_wait(3000);
				}
			}
		}
		else 
                {
			printk("qtm_obp_ts_probe: (%d) receiving from 0x%x(number of bytes: %d, type: %d): 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
			numNeededReads++, client->addr, ret, ((data[0]>>4&0x07)), data[0], data[1], data[2], data[3], data[4], data[5] );

                        qtmTsGl->mode = KEY08_MODE_NORMAL;
                        
                        // if we got a 0xff it means the buffer is empty so get out of the loop, we must be in Normal mode
			if ( data[0] == 0xff )
			{
				printk("qtm_obp_ts_probe: Got all data.\n");
				ret = 0;
			}

                        // if we have the pre-OBP firmware a read from register 0 will read back the status message event (0x70), 
                        // but if we do not get 0x70 we can assume we are in OBP mode
                        else if (data[0] != 0x70)
                        {
                               qtmTsGl->mode = KEY08_MODE_OBP;
                        }
                }
	} while ( ret == 6 && qtmTsGl->mode == KEY08_MODE_NORMAL && numNeededReads < 20);

        // if we have read 20 times from the status register, something bad happened so get out.
        if (numNeededReads >= 20)
	{
		KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: stuck reading status \n");
		goto err_detect_failed;
	}
 
        // if we are in NORMAL mode, keep reading from register 0 .. empty the buffer ???
	while (retry-- > 0 && qtmTsGl->mode == KEY08_MODE_NORMAL )
	{
		ret = qtm_obp_ts_read_register(client, (int)0x0);
		if (ret >= 0)
			break;
		msleep(100);
	}

	if (ret < 0)
	{
		KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}

	if ( qtmTsGl->mode == KEY08_MODE_NORMAL )
	{
		printk("qtm_obp_ts_probe: Chip ID 0x%x\n", ret);
		ret = qtm_obp_ts_read_register(client, (int)0x1);
		if (ret < 0)
		{
			KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: i2c_smbus_read_byte_data failed\n");
			goto err_detect_failed;
		}

		printk("qtm_obp_ts_probe: Product Code Version 0x%x\n", ret);

	}

        if ( qtmTsGl->mode == KEY08_MODE_OBP)
        {
                qtm_obp_load_obp_objects();
                save_low_power_regs();
        }

	if ( qtmTsGl->mode != KEY08_MODE_UNKNOWN )
	{
		max_x = KEY08_MAX_X;
		max_y = KEY08_MAX_Y;

		qtmTsGl->max[1] = max_y = (ret >> 8 & 0xff) | ((ret & 0x1f) << 8);
		qtmTsGl->max[1] = max_y;

		if (qtmTsGl->flags & KEY08_SWAP_XY)
			swap(max_x, max_y);

		qtmTsGl->input_dev = input_allocate_device();
		if (qtmTsGl->input_dev == NULL) 
		{
			ret = -ENOMEM;
			KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: Failed to allocate input device\n");
			goto err_input_dev_alloc_failed;
		}
		qtmTsGl->input_dev->name = "touchscreen";

		inactive_area_left = inactive_area_left * max_x / 0x10000;
		inactive_area_right = inactive_area_right * max_x / 0x10000;
		inactive_area_top = inactive_area_top * max_y / 0x10000;
		inactive_area_bottom = inactive_area_bottom * max_y / 0x10000;
		fuzz_x = fuzz_x * max_x / 0x10000;
		fuzz_y = fuzz_y * max_y / 0x10000;
		qtmTsGl->snap_down[!!(qtmTsGl->flags & KEY08_SWAP_XY)] = -inactive_area_left;
		qtmTsGl->snap_up[!!(qtmTsGl->flags & KEY08_SWAP_XY)] = max_x + inactive_area_right;
		qtmTsGl->snap_down[!(qtmTsGl->flags & KEY08_SWAP_XY)] = -inactive_area_top;
		qtmTsGl->snap_up[!(qtmTsGl->flags & KEY08_SWAP_XY)] = max_y + inactive_area_bottom;
		KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
		KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: inactive_x %d %d, inactive_y %d %d\n",
			inactive_area_left, inactive_area_right,
			inactive_area_top, inactive_area_bottom);
		inactive_area_left =0;
		inactive_area_top =0;
		inactive_area_right=0;
		inactive_area_bottom=0;
		
		KEY08_PRINTK("qtm_obp_ts_probe: ABS_MISC=0xFFFFF, ABS_X=%d, ABS_Y=%d, ABS_PRESSURE=0xff, ABS_TOOL_WIDTH=0xf\n",
                SCREEN_X, SCREEN_Y);
#ifndef GOOGLE_MULTITOUCH
		input_set_abs_params(qtmTsGl->input_dev, ABS_MISC, 0,0x000FFFFF, 0, 0); //sajid - added for multitouch
		input_set_abs_params(qtmTsGl->input_dev, ABS_X, 0, SCREEN_X, 0, 0);
		input_set_abs_params(qtmTsGl->input_dev, ABS_Y, 0, SCREEN_Y, 0, 0);
		
		input_set_abs_params(qtmTsGl->input_dev, ABS_PRESSURE, 0, 0xff, 2, 0);
		input_set_abs_params(qtmTsGl->input_dev, ABS_TOOL_WIDTH, 0,0xf , 2, 0);
		input_set_abs_params(qtmTsGl->input_dev, ABS_HAT0X, 0, 0xfff, 2, 0); //sajid - not needed ??
		input_set_abs_params(qtmTsGl->input_dev, ABS_HAT0Y, 0, 0xfff, 2, 0); //sajid - not needed ??
		/* qtmTsGl->input_dev->name = qtmTsGl->keypad_info->name; */
		set_bit(EV_SYN, qtmTsGl->input_dev->evbit);
		set_bit(EV_KEY, qtmTsGl->input_dev->evbit);
		set_bit(BTN_TOUCH, qtmTsGl->input_dev->keybit);
		set_bit(KEY_HOME, qtmTsGl->input_dev->keybit);
		set_bit(KEY_BACK, qtmTsGl->input_dev->keybit);
		set_bit(KEY_MENU, qtmTsGl->input_dev->keybit);
		set_bit(BTN_2, qtmTsGl->input_dev->keybit);     
		set_bit(EV_ABS, qtmTsGl->input_dev->evbit);
#else
		/* Hard code the values. It's for Zeppelin product ONLY. Next version of the driver uses different mechanism. This will not be re-used in the future. */ 
		input_set_abs_params(qtmTsGl->input_dev, ABS_MT_POSITION_X, 0, 1023, 0, 0);
		input_set_abs_params(qtmTsGl->input_dev, ABS_MT_POSITION_Y, 0, 1023, 0, 0);
		input_set_abs_params(qtmTsGl->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(qtmTsGl->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);

		set_bit(EV_ABS, qtmTsGl->input_dev->evbit);
		set_bit(EV_KEY, qtmTsGl->input_dev->keybit);
		set_bit(EV_SYN, qtmTsGl->input_dev->keybit);
		set_bit(BTN_TOUCH, qtmTsGl->input_dev->keybit);
		set_bit(ABS_X, qtmTsGl->input_dev->keybit);
		set_bit(ABS_Y, qtmTsGl->input_dev->keybit);

#endif

		ret = input_register_handler(&slider_handler);
		if (ret) 
		{
			KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: Unable to register slider input device\n");
			goto err_input_register_device_failed;
		}
		ret = input_register_device(qtmTsGl->input_dev);
		if (ret) 
		{
			KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: Unable to register %s input device\n", qtmTsGl->input_dev->name);
			goto err_input_register_device_failed;
		}
		
		// Setup the Interrupt

		printk("qtm_obp_ts_probe Gpio request\n");

		ret = gpio_request(KEY08_TS_GPIO, "qtm_obp_ts_gpio");
		if (ret) 
		{
			KEY08_PRINTK(KERN_ERR " gpio_request failed for input %d\n", KEY08_TS_GPIO);
			goto err_input_register_device_failed;
		}
		
		ret = gpio_direction_input(KEY08_TS_GPIO);
		if (ret) 
		{
			KEY08_PRINTK(KERN_ERR " gpio_direction_input failed for input %d\n", KEY08_TS_GPIO);
			goto err_input_register_device_failed;
		}

		client->irq = gpio_to_irq(KEY08_TS_GPIO);
		ret = gpio_direction_input(KEY08_TS_GPIO);
		printk("qtm_obp_ts_probe: ret from gpio_direction_input = %d\n", ret);
		KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: gpio_request KEY08_TS_GPIO =  %d\n", gpio_get_value(KEY08_TS_GPIO));


		if (client->irq) 
		{
			
			ret = request_irq(client->irq, qtm_obp_ts_irq_handler, request_flags , pdev->name, qtmTsGl);
			if (ret == 0) 
			{
				if (ret)
					free_irq(client->irq, qtmTsGl);
			}
			if (ret == 0)
				qtmTsGl->use_irq = 1;
			else
				dev_err(&client->dev, "request_irq failed\n");
		}
		if (!qtmTsGl->use_irq) 
		{
			hrtimer_init(&qtmTsGl->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			qtmTsGl->timer.function = qtm_obp_ts_timer_func;
			hrtimer_start(&qtmTsGl->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
#ifdef CONFIG_HAS_EARLYSUSPEND
		qtmTsGl->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		qtmTsGl->early_suspend.suspend = qtm_obp_ts_early_suspend;
		qtmTsGl->early_suspend.resume = qtm_obp_ts_late_resume;
		register_early_suspend(&qtmTsGl->early_suspend);
#endif
		qtmTsGl->useData = TRUE;
		qtmTsGl->inCall = FALSE;
			
		KEY08_PRINTK(KERN_INFO "qtm_obp_ts_probe: Start touchscreen %s in %s mode\n", qtmTsGl->input_dev->name, qtmTsGl->use_irq ? "interrupt" : "polling");

		qtm_lastKeyPressed = 0;

		printk("qtm_obp_ts_probe: GPIO 84: %d\n", gpio_get_value(84) );
	}
	printk("qtm_obp_ts_probe: Exit\n");
	return 0;


err_input_register_device_failed:
	input_free_device(qtmTsGl->input_dev);
// exit_kfree:
err_input_dev_alloc_failed:
err_detect_failed:
err_power_failed:
	kfree(qtmTsGl);
err_alloc_data_failed:
err_check_functionality_failed:
// exit_detach:
	i2c_detach_client(client);
	printk("qtm_obp_ts_probe: Exit after error\n");
	return ret;
}

static int qtm_obp_ts_remove(struct i2c_client *client)
{
	struct qtm_obp_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
	if (ts->use_irq)
	  free_irq(client->irq, ts);
	else
	  hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
	return 0;
}

static int qtm_obp_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
       int ret;
       int regValue=0;

       KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);

       if (!qtmTsGl->suspendMode)
       {
         qtmTsGl->suspendMode = TRUE; 
         if (qtmTsGl->mode == KEY08_MODE_OBP)
	 {
	       ret = qtm_obp_reg_write (qtmTsGl->client, qtm_obp_obj_table_elem[OBJ_ID_GEN_POWERCONFIG_T7].start_pos+QTM_OBP_T7_IDLE,
					sizeof(char), (char *)&regValue);
	       clk_busy_wait(3000);

               if (ret)
                  printk("%s: Failed to write Idle Low Power Register\n", __FUNCTION__);

               ret = qtm_obp_reg_write (qtmTsGl->client, qtm_obp_obj_table_elem[OBJ_ID_GEN_POWERCONFIG_T7].start_pos+QTM_OBP_T7_ACTIVE,
                                   sizeof(char), (char *)&regValue);
               clk_busy_wait(3000);

               if (ret)
                  printk("%s: Failed to write Active Low Power Register\n", __FUNCTION__);
	     }
	   
	   if (qtmTsGl->use_irq)
	     {
	       KEY08_PRINTK("%s: Do Not Disable irq\n",__FUNCTION__);
               KEY08_PRINTK("%s: qtmTsGl->use_irq is true \n", __FUNCTION__);
               // disable_irq(client->irq);
	     }
	   else
	     {
               KEY08_PRINTK("%s: qtmTsGl->use_irq is false \n", __FUNCTION__);
               hrtimer_cancel(&qtmTsGl->timer);
	     }

#if 0
	   // cancel_work_sync will cancel the work if it is queued. 
	   // If the work's callback appears to be running, cancel_work_sync will block until it has completed.

	   ret = cancel_work_sync(&qtmTsGl->work);

	   if (ret && qtmTsGl->use_irq)
	     {
	       
	       KEY08_PRINTK("%s: Enable irq\n",__FUNCTION__);
               KEY08_PRINTK("%s: qtmTsGl->use_irq && cancel_work_sync ret is true \n", __FUNCTION__);
               enable_irq(client->irq);
	     }
#endif

	   if (qtmTsGl->power)
	     {
	       KEY08_PRINTK("%s: qtmTsGl->power is true \n", __FUNCTION__);
	       ret = qtmTsGl->power(0);
	       if (ret < 0)
		 {
		   KEY08_PRINTK(KERN_ERR "qtm_obp_ts_suspend: power off failed\n");
		 }
	     }
       }
       else {
          KEY08_PRINTK("%s: Already suspended\n", __FUNCTION__);
       }

       KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
       return 0;
}

static int qtm_obp_ts_resume(struct i2c_client *client)
{
        int ret;

        KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);

        if (qtmTsGl->suspendMode)
        {
           qtmTsGl->suspendMode = FALSE;
           if (qtmTsGl->mode == KEY08_MODE_OBP)
           {
				/* If we were suspended while a touch was happening
				   we need to tell the upper layers so they do not hang
				   waiting on the liftoff that will not come. */
				int i;
				for (i = 0; i < 2; i++) 
				{
					KEY08_PRINTK("%s: Finger %i down state %i\n",
							__func__, i, qtmTsGl->finger_data[i].down);
					if (qtmTsGl->finger_data[i].down == 0)
						continue;
					input_report_abs(qtmTsGl->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					input_mt_sync(qtmTsGl->input_dev);
					memset(&qtmTsGl->finger_data[i], 0, sizeof(struct coordinate_map));
				}
				input_sync(qtmTsGl->input_dev); 

               if (qtmTsGl->power)
               {
                   KEY08_PRINTK("%s: qtmTsGl->power is true \n", __FUNCTION__);
                   ret = qtmTsGl->power(1);
                   if (ret < 0)
                   {
                           KEY08_PRINTK(KERN_ERR "qtm_obp_ts_resume power on failed\n");
                   }
               }

               ret = qtm_obp_reg_write (qtmTsGl->client, qtm_obp_obj_table_elem[OBJ_ID_GEN_POWERCONFIG_T7].start_pos+QTM_OBP_T7_IDLE,
                                   sizeof(char), (char *)&lpIdleValueEeprom);
               clk_busy_wait(3000);

               if (ret)
                  printk("%s: Failed to write Idle Low Power Register\n", __FUNCTION__);

               ret = qtm_obp_reg_write (qtmTsGl->client, qtm_obp_obj_table_elem[OBJ_ID_GEN_POWERCONFIG_T7].start_pos+ QTM_OBP_T7_ACTIVE,
                                   sizeof(char), (char *)&lpActiveValueEeprom);
               clk_busy_wait(3000);

               if (ret)
                  printk("%s: Failed to write Active Low Power Register\n", __FUNCTION__);

               KEY08_PRINTK("qtm_obp_ts_resume: try to read from OBP firmware\n");

               ret = qtm_obp_calibrate(qtmTsGl->client);

               if (qtmTsGl->use_irq)
               {
	          KEY08_PRINTK("%s: Do not Enable irq\n",__FUNCTION__);
                  KEY08_PRINTK("%s: qtmTsGl->use_irq is true \n", __FUNCTION__);
                  // enable_irq(client->irq);
               }
               else
               {
                   KEY08_PRINTK("%s: qtmTsGl->use_irq is false \n", __FUNCTION__);
                   hrtimer_start(&qtmTsGl->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
               }
           }
        }
        else {
           KEY08_PRINTK("%s: Already resumed\n", __FUNCTION__);
        }

        KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
        return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qtm_obp_ts_early_suspend(struct early_suspend *h)
{
	struct qtm_obp_ts_data *ts;
	KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
	ts = container_of(h, struct qtm_obp_ts_data, early_suspend);

	KEY08_PRINTK("%s: Locking transaction mutex\n",__FUNCTION__);
	mutex_lock(&qtmTsGl->transaction_mutex);
	KEY08_PRINTK("%s: Obtained transaction mutex\n",__FUNCTION__);

	qtm_obp_ts_suspend(ts->client, PMSG_SUSPEND);

	KEY08_PRINTK("%s: Unlocking transaction mutex\n",__FUNCTION__);
	mutex_unlock(&qtmTsGl->transaction_mutex);
	KEY08_PRINTK("%s: Released transaction mutex\n",__FUNCTION__);


	KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
}

static void qtm_obp_ts_late_resume(struct early_suspend *h)
{
	struct qtm_obp_ts_data *ts;

	KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
	ts = container_of(h, struct qtm_obp_ts_data, early_suspend);

	KEY08_PRINTK("%s: Locking transaction mutex\n",__FUNCTION__);
	mutex_lock(&qtmTsGl->transaction_mutex);
	KEY08_PRINTK("%s: Obtained transaction mutex\n",__FUNCTION__);

	qtm_obp_ts_resume(ts->client);

	KEY08_PRINTK("%s: Unlocking transaction mutex\n",__FUNCTION__);
	mutex_unlock(&qtmTsGl->transaction_mutex);
	KEY08_PRINTK("%s: Released transaction mutex\n",__FUNCTION__);

	KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
}
#endif

// Is there any OBP impact here ???
// How wil this impact Zeppelin ???

static void qtm_obp_slider_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	KEY08_PRINTK("%s: enter....\n",__FUNCTION__);
	KEY08_PRINTK("%s: exit....\n",__FUNCTION__);
}

static int qtm_obp_slider_connect(struct input_handler *handler, struct input_dev *dev,
			const struct	input_device_id *id)
{
	struct input_handle *handle;
	int error ,sw_value;

     KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
	if( qtmTsGl->input_dev == dev )   
		return -ENODEV;

	if ( !(strncmp(dev->name,"adp5588",7)))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
   	if (!handle)
		return -ENOMEM;

	// qtmTsGl->sliderDev = dev;
	handle->dev = dev;
	handle->handler = handler;
	handle->name = "qtm_obp_slider";
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
     KEY08_PRINTK("%s: Exiting with error=%d.....\n", __FUNCTION__, error);
	return error;
}

static void qtm_obp_slider_disconnect(struct input_handle *handle)
{
    KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);

    input_close_device(handle);
    input_unregister_handle(handle);
    kfree(handle);
    KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
}

/**
 * Open device
 *
 * @param inode
 * @param filp
 * @return 0
 */
static	int	qtm_obp_open(struct inode *inode, struct file *filp)
{
	KEY08_PRINTK("qtm_obp_open: entering\n");
	KEY08_PRINTK("qtm_obp_open: exiting\n");

	return 0;
}

unsigned char	qtm_kernelBuffer[255];
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
static int qtm_obp_ioctl(struct inode *node, struct file *filp, unsigned int cmd, unsigned long arg)
{
	A_TOUCH_POINT_PTR	dataPoint;
	char * 	dataPtr;
	unsigned long	bCount;
	int	regValue;
	int	rc = -1;
	int	ret = 0;
	char	data[10];
        int     loopCount;
        int     cntEndFirmwareAttempts = 0;
        int     flagFirm = 1;
        int i = 0;

     KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);

     KEY08_PRINTK("%s: Locking transaction mutex\n",__FUNCTION__);
     mutex_lock(&qtmTsGl->transaction_mutex);
     KEY08_PRINTK("%s: Obtained transaction mutex\n",__FUNCTION__);


	switch (cmd) 
	{
	case KEY08_IOCTL_SET_IN_CALL:
		qtmTsGl->inCall = TRUE;
		break;
	case KEY08_IOCTL_SET_NOT_IN_CALL:
		qtmTsGl->inCall = FALSE;
		break;
	case KEY08_IOCTL_SET_BOOTLOADER_MODE:

          if ( qtmTsGl->mode == KEY08_MODE_OBP )
          {
              KEY08_PRINTK("qtm_obp_ioctl:Setting BOOTLOADER mode\n");

              rc = -1;

              // disable interrupts since we are about to set the change line
              // to an ouput to set the device into bootloader mode

              if (qtmTsGl->use_irq)
              {
                  disable_irq(qtmTsGl->client->irq);
              }

              // set Change pin to LOW

              gpio_request(KEY08_TS_GPIO, "qtm_obp_ts_gpio");
              gpio_direction_output(KEY08_TS_GPIO, 0);
              gpio_set_value(KEY08_TS_GPIO,0);

              // set RESET to Low

              gpio_request(KEY08_TS_RESET, "touch_rst_n");
              gpio_set_value(KEY08_TS_RESET,0);

              // wait 10us
              clk_busy_wait(10);

              // set RESET to high
              gpio_set_value(KEY08_TS_RESET ,1);

              // wait 100ms
              clk_busy_wait(100000);

              // set Change pin to high
              gpio_set_value(KEY08_TS_GPIO ,1);

              // set Change pin back to input for interrupt
              gpio_direction_input(KEY08_TS_GPIO);

              // re-enable irq
              if (qtmTsGl->use_irq)
              {
                  enable_irq(qtmTsGl->client->irq);
              }

              /* Confirm that we are in BOOTLOADER mode */
              qtmTsGl->client->addr = qtm_obp_bl_addr;
              data[0] = 0;
              loopCount =0;

              // Check if we are waiting for bootloader command state (data retured=0xCx)
              while ( (data[0] & 0xF0) != 0xC0 &&
                      (data[0] & 0xF0) != 0x80 &&
                      (loopCount < 100 ) )
              {
                  rc = i2c_master_recv(qtmTsGl->client, data, 1);
                  KEY08_PRINTK("qtm_obp_ioctl: reading BL status = 0x%02X\n",data[0]);
                  clk_busy_wait(3000);
                  loopCount++;
              }

              // if loopCount reached max, something is wrong.
              // exit with error and don't set mode to BOOTLOADER

              if (loopCount >=100)
              {
                  KEY08_PRINTK("qtm_obp_ioctl: unable to set to BL mode\n");
                  qtmTsGl->client->addr = qtm_obp_normal_addr;
                  rc = -1;
              }
              else
              {
                  // we are bootloader mode
                  qtmTsGl->mode = KEY08_MODE_BOOTLOADER;

                  KEY08_PRINTK("qtm_obp_ioctl: We are in BL mode, got 0x%02X\n",data[0]);

                  if ( (data[0] & 0xF0 ) == 0xC0 )
                  {
                      KEY08_PRINTK("qtm_obp_ioctl: send unlock code\n");

                      data[0] = 0xDC;
                      data[1] = 0xAA;
                      rc = i2c_master_send (qtmTsGl->client, data, 2);
                      clk_busy_wait(3000);

                      // Check if Bootloader is ready for receiving data
                      loopCount = 0; 
                      while ( (data[0] & 0xF0) != 0x80 &&
                            ( loopCount < 1000 ) )
                      {
                          rc = i2c_master_recv(qtmTsGl->client, data,1);
                          clk_busy_wait(3000);
                          loopCount++;
                      }
                
                      if (loopCount >=1000)
                      {
                          // Something wrong, Bootloader is not ready for receiving data
                          printk("qtm_obp_ioctl: Something wrong, Bootloader is not ready for receiving data \n");
                          rc = -1;
                      }
                      else
                      { 
                         rc = KEY08_MODE_BOOTLOADER;
                         // change the mode to waiting for data
                         qtmTsGl->sendMode = KEY08_BL_WAITING_FOR_DATA;
                      }
                  }
                  else if ( (data[0] & 0xF0 ) == 0x80 )
                  {
                      KEY08_PRINTK("qtm_obp_ioctl: Already in BOOTLOADER mode\n");
                      rc = KEY08_MODE_BOOTLOADER;
                  }
              
              }
          }
	  else 
	  {
			KEY08_PRINTK("qtm_obp_ioctl: Already in BOOTLOADER mode\n");
                        // change the mode to waiting for data
                        rc = KEY08_MODE_BOOTLOADER;
                        qtmTsGl->sendMode = KEY08_BL_WAITING_FOR_DATA;
          }
	  if ( rc != -1 )
          {
		// qtmTsGl->sendMode = KEY08_BL_WAITING_FOR_DATA;
	        qtmTsGl->mode = KEY08_MODE_BOOTLOADER;
                rc = KEY08_MODE_BOOTLOADER;
          }

	  KEY08_PRINTK("qtm_obp_ioctl: Mode is set to  %d\n", qtmTsGl->mode);

	  break;

	case KEY08_IOCTL_DL_GET_STATUS:
		rc = qtmTsGl->sendMode;
		break;

	case KEY08_IOCTL_CAL_STATUS:
		rc = calibrating;
		break;

	case KEY08_IOCTL_GET_MODE:
		rc = qtmTsGl->mode;
		break;

	case KEY08_IOCTL_SET_NORMAL_MODE:
          if ((qtmTsGl->mode != KEY08_MODE_NORMAL) && (qtmTsGl->mode != KEY08_MODE_OBP))
          {
              KEY08_PRINTK("qtm_obp_ioctl: NORMAL_MODE re-setting flags to normal mode\n");

              qtmTsGl->client->addr = qtm_obp_normal_addr;
              memset(data,0,sizeof(data));

              if ((ret = i2c_master_recv(qtmTsGl->client, data, 1 )) < 0 )
              {
                  KEY08_PRINTK("qtm_obp_ioctl: i2c_master_recv (normal client) error  %d\n",ret);
              }
              else
              {
                  KEY08_PRINTK("qtm_obp_ioctl: Check First Byte data[0] : %x \n", data[0]);
                  if (data[0] == 0x70 || data[0] == 0xFF)
                  {
                      qtmTsGl->mode = KEY08_MODE_NORMAL;
                  }
                  else
                  {
                      qtmTsGl->mode = KEY08_MODE_OBP;
                  }
              }
              qtmTsGl->sendMode = KEY08_BL_WAITING_FOR_NOTHING;
              rc = qtm_obp_calibrate(qtmTsGl->client);
          }
          else
          {
              KEY08_PRINTK("qtm_obp_ioctl: Mode is Already set to  %d\n", qtmTsGl->mode);
              rc = qtmTsGl->sendMode;
          }
          KEY08_PRINTK("qtm_obp_ioctl: Mode is set to  %d\n", qtmTsGl->mode);
          break;

	case KEY08_IOCTL_GET_VERSION:
		rc = qtm_obp_ts_read_app_version();
		break;

	case KEY08_IOCTL_WRITE_REGISTER:
		bCount = copy_from_user(qtm_kernelBuffer, (char *)arg, (unsigned long)sizeof(A_REG)); 
		if ( bCount == 0 )
		{
			A_REG_PTR	regV;
			regV = (A_REG_PTR) qtm_kernelBuffer;
			rc = qtm_obp_reg_write (qtmTsGl->client,
				regV->reg,
				sizeof(char), (char *)&regV->value);

			clk_busy_wait(6000);
			KEY08_PRINTK("qtm_obp_ioctl: setting register %d = %d\n", (int)regV->reg, (int)regV->value);
		}
		break;

	case KEY08_IOCTL_READ_REGISTER:
		regValue = (int)arg;
		rc = qtm_obp_ts_read_register(qtmTsGl->client,regValue);
		KEY08_PRINTK("qtm_obp_ioctl: Register %d = %d\n", (int)regValue, (int)rc);
		break;

	case KEY08_IOCTL_END_FIRMWARE:
                printk("qtm_obp_ioctl: Firmware Upgrade is Complete \n");
		printk("qtm_obp_ioctl: Config File will be Loaded \n");
		KEY08_PRINTK("qtm_obp_ioctl: END_FIRMWARE re-setting flags to normal mode\n");
                disable_irq(qtmTsGl->client->irq); // Touchpad Cleanup
		qtmTsGl->client->addr = qtm_obp_normal_addr;
		qtmTsGl->mode = KEY08_MODE_OBP;
                // Average Time to switch to normal mode is 500 ms, so wait 600 ms
		clk_busy_wait(600000);
		memset(data,0,sizeof(data));
                rc = 0;

                flagFirm = 1;
                while (flagFirm)
                {
		    if ((ret = i2c_master_recv(qtmTsGl->client, data, 6 )) < 0 )
		    {
		       printk("qtm_obp_ioctl: i2c_master_recv (normal client) error  %d\n",ret);
                       if (cntEndFirmwareAttempts > 100)
                       {
                          flagFirm = 0;
                          rc = -1;
                          cntEndFirmwareAttempts = 0;
                       }
                       cntEndFirmwareAttempts++;
		    }
		    else
		    {
                        printk("END_FIRMWARE: Buffer\n");
                        for (i=0;i<ret;i++)
                           printk("data[%d] = 0x%x\n", i, data[i]);

                        // If Doing Calibrate Wait till Calibration is Complete
                        if ((data[QTM_OBP_T6_STATUS] & QTM_OBP_T6_STATUS_CAL))
                        {
		            printk("qtm_obp_ioctl: Calibrating in END_FIRMWARE \n");
                            calibrating = 1;
		            clk_busy_wait(250000); // Allow IC to finish calibration
                        }
                        else if ((calibrating != 0) && ((data[QTM_OBP_T6_STATUS] & QTM_OBP_T6_STATUS_CAL) == 0))
                        {
		            printk("qtm_obp_ioctl: Calibration Done!! in END_FIRMWARE \n");
                            calibrating = 0;
                            flagFirm = 0;
                        }
                        else if (!calibrating)
                        {
                            printk("qtm_obp_ioctl: ** IC Resets to Normal Mode ** \n");
                            flagFirm = 0;
                        }
		    }
		    clk_busy_wait(3000);
                 }
                 if (rc < 0)
                   printk ( "qtm_obp_ioctl: Unable to Switch to NORMAL MODE");
                 else
		   qtm_obp_load_obp_objects();
		 break;

	case KEY08_IOCTL_DISPLAY_ONE_POINT:
		bCount = copy_from_user(qtm_kernelBuffer, (char *)arg, (unsigned long)sizeof(A_TOUCH_POINT)); 
		if ( bCount == 0 )
		{
			dataPoint = (A_TOUCH_POINT_PTR) qtm_kernelBuffer;

			KEY08_PRINTK("qtm_obp_ioctl: touching point: (%d, %d), finger: %d\n", dataPoint->X, dataPoint->Y, dataPoint->finger);
			input_report_abs(qtmTsGl->input_dev, ABS_X, dataPoint->X);
			input_report_abs(qtmTsGl->input_dev, ABS_Y, dataPoint->Y);
			input_report_abs(qtmTsGl->input_dev, ABS_PRESSURE, dataPoint->Z);
			input_report_abs(qtmTsGl->input_dev, ABS_TOOL_WIDTH, dataPoint->W);
			input_report_key(qtmTsGl->input_dev, BTN_TOUCH, dataPoint->finger);
			input_sync(qtmTsGl->input_dev);
			rc = qtmTsGl->mode;	
		}
		else
		{
			KEY08_PRINTK("qtm_obp_ioctl: touching point: copy_from_user() failed. Not touching anything!\n");
		}
		break;

        case KEY08_IOCTL_GET_TOUCH_SENS_DATA:
          dataPtr = (char *) arg;
          qtm_obp_get_touch_sens_data ((char *) dataPtr);
	  break;

	case KEY08_IOCTL_RESET:
	  KEY08_PRINTK("qtm_obp_ioctl: RESET !\n");
	  if (qtmTsGl->mode == KEY08_MODE_NORMAL)
		    rc = KEY08_MODE_NORMAL;
		else 
		    rc = KEY08_MODE_OBP;
	  qtm_obp_doReset();
	  break;

	case KEY08_IOCTL_DEBUG:
		bCount = copy_from_user(qtm_kernelBuffer, (char *)arg, (unsigned long)sizeof(char)); 
		if ( bCount == 0 )
				qtm_debugOn = qtm_kernelBuffer[0];
		rc = 0;
		break;

	case KEY08_IOCTL_CALIBRATE:
            if (qtmTsGl->mode == KEY08_MODE_OBP)
            {
                calibrating = 1;
                enable_irq(qtmTsGl->client->irq); // Touchpad Cleanup
		rc = qtm_obp_calibrate(qtmTsGl->client);
            }
            else
                calibrating = 0;
		break;

	case KEY08_IOCTL_SETIRQ:

                bCount = copy_from_user(qtm_kernelBuffer, (char *)arg, (unsigned long)sizeof(char));
                if ( bCount == 0 )
                        regValue = qtm_kernelBuffer[0];
                if ( regValue )
                {
                        enable_irq(qtmTsGl->client->irq);
                        KEY08_PRINTK("%s: Enabling irq %d\n", __FUNCTION__, qtmTsGl->client->irq);
                }
                else
                {
                        disable_irq(qtmTsGl->client->irq);
                        KEY08_PRINTK("%s: Disabling irq: %d\n", __FUNCTION__, qtmTsGl->client->irq);
                }
                break;

        case KEY08_IOCTL_DISABLE:
             KEY08_PRINTK("%s: Disabling touchscreen\n", __FUNCTION__);
             qtm_obp_ts_suspend (qtmTsGl->client, PMSG_SUSPEND);
             break;

        case KEY08_IOCTL_ENABLE:
             KEY08_PRINTK("%s: Enabling touchscreen\n", __FUNCTION__);
             qtm_obp_ts_resume (qtmTsGl->client); 
             rc = 0;
             break;

	case KEY08_IOCTL_SET_MESSAGE_PTR:
             rc = qtm_obp_send_message_processor_pointer();
             break;

	case KEY08_IOCTL_SAVE_CONFIG: 
             save_low_power_regs();
             // enable_irq(qtmTsGl->client->irq); // Touchpad Cleanup
             rc = qtm_obp_save_config();
             break;

     case KEY08_IOCTL_GET_CMD_STATUS:
             KEY08_PRINTK("%s: File transfer status: 0x%x\n",__FUNCTION__,
                           qtmTsGl->XferStatus);
             rc = qtmTsGl->XferStatus;
             break;

     case KEY08_IOCTL_SET_CMD_STATUS:
             bCount = copy_from_user(&qtmTsGl->XferStatus, (char *)arg, (unsigned long)sizeof(qtmTsGl->XferStatus));
             if ( bCount != sizeof(qtmTsGl->XferStatus) )
                  rc = -EFAULT;
	     /*
	      * This can only be status of the configuration transfer
	      * This means we have to put it into the high byte of the status
	      */

             KEY08_PRINTK("%s: File transfer status: 0x%x\n",__FUNCTION__,
                           qtmTsGl->XferStatus);
             rc = qtmTsGl->XferStatus;

             break;
              
      case KEY08_IOCTL_GET_BAD_CRC_COUNT:
                rc = qtmTsGl->cntBadCRC;
                break;

      case KEY08_IOCTL_READ_REGISTER16:
           rc = 0;
           break;

      case KEY08_IOCTL_GETIRQ:
           rc = 0;
           break;

      case KEY08_IOCTL_READ_4_REGS:
           rc = 0;
           break;

      case KEY08_IOCTL_SUSPEND:
           rc = 0;
           break;

      case KEY08_IOCTL_GET_POINT:
           rc = 0;
           break;
      
	default:
		return EINVAL;
		break;
	}	

     KEY08_PRINTK("%s: Unlocking transaction mutex\n",__FUNCTION__);
     mutex_unlock(&qtmTsGl->transaction_mutex);
     KEY08_PRINTK("%s: Released transaction mutex\n",__FUNCTION__);

     KEY08_PRINTK("%s: Exiting.....rc=%d\n", __FUNCTION__, rc);
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


static int qtm_obp_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos )
{
    int	retval=-1;
    int	i;
    int	reg;
    unsigned long	bCount;

    KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
    if ( qtmTsGl->mode == KEY08_MODE_BOOTLOADER )
    {
      KEY08_PRINTK("qtm_obp_write: Currently in BOOTLOADER mode.\n");
      qtmTsGl->sendMode = KEY08_BL_WAITING_FOR_DATA;
      /* get data from user space into kernel space */
      bCount = copy_from_user(qtm_kernelBuffer, buf, (unsigned long)count);
      KEY08_PRINTK("qtm_obp_write: copy_from_user() returned %d\n", (int)bCount);
      qtmTsGl->client->addr = qtm_obp_bl_addr;
      do
        {
          KEY08_PRINTK("qtm_obp_write: record to IC: size=%d\nqtm_obp_write: record data:\n",count);
          for ( i = 0; i < count; i++)
            {
              if ( !(i%20) )
                KEY08_PRINTK("\nqtm_obp_write: ");
              KEY08_PRINTK("%02x ", qtm_kernelBuffer[i]);
            }
          KEY08_PRINTK("\n");
          retval = i2c_master_send (qtmTsGl->client, qtm_kernelBuffer, count);
         
          /* On failure, output the error code and delay before trying again */
          if (retval < 0)
            {
              printk(KERN_ERR "qtm_obp_write: write failed: %d\n", retval);
              qtmTsGl->sendMode = KEY08_BL_WAITING_FAILED;
            }
          else
            {
              KEY08_PRINTK("qtm_obp_write: i2c_master_send(): rc= %d\n",retval);
              qtmTsGl->sendMode = KEY08_BL_WAITING_FOR_COMMAND;
              retval = count;
            }
          clk_busy_wait(3000);
        }
      while (retval < 0);
    }
    else if (qtmTsGl->mode == KEY08_MODE_OBP)
      {
	KEY08_PRINTK("qtm_obp_write: Currently in NORMAL/OBP mode.\n");
	/* Get the data from the user space */
	bCount = copy_from_user(qtm_kernelBuffer, buf, (unsigned long)count); 
	KEY08_PRINTK("qtm_obp_write: copy_from_user() returned %d\n", (int)bCount);
	
	reg = ((qtm_kernelBuffer[0]<<8) | qtm_kernelBuffer[1]);
	KEY08_PRINTK("qtm_obp_write: config record: size: %d, reg = %d\n",count, reg);
	KEY08_PRINTK("qtm_obp_write: ");
	for ( i = 0; i < count; i++)
	  {
	    char	ch = qtm_kernelBuffer[i];
	    KEY08_PRINTK("%02X ", ch);
	  }
	KEY08_PRINTK("\n");
	qtmTsGl->client->addr = qtm_obp_normal_addr;
	if ( (retval = qtm_obp_reg_write (qtmTsGl->client, reg, 4, (char *)&(qtm_kernelBuffer[2]))) == 0 )
	  {
	    KEY08_PRINTK("qtm_obp_write: finished writing config info\n");
	    retval = 6;
	  }
	else
	  {
	    KEY08_PRINTK("qtm_obp_write: finished writing config info\n");
	  }
      }
    else
      retval = -1;
    KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
    return retval;
}


/**
 * Release device
 *
 * @param inode
 * @param filp
 * @return 0
 */
static int qtm_obp_release(struct inode *inode, struct file *filp)
{
     KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
     KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
     return 0;
}

static int __devinit qtm_obp_ts_init(void)
{
     KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
     qtm_debugOn = OFF;
     KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
     return i2c_add_driver(&qtm_obp_ts_driver);
}

static void __exit qtm_obp_ts_exit(void)
{
     i2c_del_driver(&qtm_obp_ts_driver);
}

/*!
 * @brief Logs data
 *
 * This function is called as a replacement to printk
 *
 * @param fmt Text of message to log
 */

static void qtm_obp_printk (char *fmt, ...)
{
	if ( qtm_debugOn )
	{
		static va_list args;
		va_start(args, fmt);
		vprintk(fmt, args);
		va_end(args);
	}
}

static void qtm_obp_load_obp_objects (void)
{
    unsigned char value[QTM_OBP_REG_ADDR_SIZE];
    uint8_t qtm_obp_table_buf[QTM_OBP_INFO_BLOCK_SIZE];
    uint8_t object_type = 0;
    uint8_t report_id = 0;
    int i = 0;
    int ret = 0;
    uint16_t reg;

    KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);

    // Setup register 0 - Info Block
    memset(qtm_obp_info_block.qtm_info_buf,0,QTM_OBP_INFO_BLOCK_SIZE);

    value[0] = 0;
    value[1] = 0;
    ret = i2c_master_send(qtmTsGl->client, value, QTM_OBP_REG_ADDR_SIZE);

    if (ret < 0)
    {
        KEY08_PRINTK(KERN_ERR "qtm_obp_ts_probe: failed to read from OBP\n");
        //goto err_detect_failed;
    }

    // Read Info Block
    clk_busy_wait(3000);

    ret = i2c_master_recv(qtmTsGl->client, qtm_obp_info_block.qtm_info_buf, 7);

    // clk_busy_wait(3000);
    // clk_busy_wait(3000);
    KEY08_PRINTK( "qtm_obj: family_id :%x \n",qtm_obp_info_block.qtm_info_bytes.family_id);
    KEY08_PRINTK( "qtm_obj: variant_id:%x \n",qtm_obp_info_block.qtm_info_bytes.variant_id);
    KEY08_PRINTK( "qtm_obj: version   :%x \n",qtm_obp_info_block.qtm_info_bytes.version);
    KEY08_PRINTK( "qtm_obj: build     :%x \n",qtm_obp_info_block.qtm_info_bytes.build);
    KEY08_PRINTK( "qtm_obj: matrix_x_size :%x \n",qtm_obp_info_block.qtm_info_bytes.matrix_x_size);
    KEY08_PRINTK( "qtm_obj: matrix_y_size :%x \n",qtm_obp_info_block.qtm_info_bytes.matrix_y_size);
    KEY08_PRINTK( "qtm_obj: num_of_objects:%x \n",qtm_obp_info_block.qtm_info_bytes.num_of_objects);

    // Read the objects to setup the object table

    if(qtm_obp_info_block.qtm_info_bytes.num_of_objects != 0x0)
    {
                   
         memset(qtm_obp_table_buf,0, QTM_OBP_INFO_BLOCK_SIZE);
         reg=QTM_OBP_INFO_BLOCK_SIZE;
         report_id = 1;

         for(i=0;i<qtm_obp_info_block.qtm_info_bytes.num_of_objects;i++)
         {
              memcpy(&value[0], (unsigned char *)&reg, QTM_OBP_REG_ADDR_SIZE);
              ret = i2c_master_send(qtmTsGl->client, value, QTM_OBP_REG_ADDR_SIZE);

              /* On failure, output the error code and delay before trying again */
              if (ret < 0)
              {
                   KEY08_PRINTK( "qtm_obp_reg_write: write of reg 0x%X failed: %d\n", reg, ret);
                   clk_busy_wait(QTM_OBP_READ_WRITE_DELAY);
              }

              clk_busy_wait(QTM_OBP_READ_WRITE_DELAY);

              ret = i2c_master_recv(qtmTsGl->client, qtm_obp_table_buf, QTM_OBP_OBJ_TBL_BLK_SIZE);

              if (ret < 0)
              {
                  KEY08_PRINTK( "qtm_obp_reg_read: writeread of object_id 0x%X reg 0x%X failed: %d\n", i, reg, ret)
;
                  clk_busy_wait(QTM_OBP_READ_WRITE_DELAY);
              }

              /* Copy object table data into the struct at object type offset */
              object_type =  qtm_obp_table_buf[QTM_OBP_OBJ_TBL_TYPE];

              qtm_obp_obj_table_elem[object_type].type = qtm_obp_table_buf[QTM_OBP_OBJ_TBL_TYPE];
              qtm_obp_obj_table_elem[object_type].start_pos = (qtm_obp_table_buf[QTM_OBP_OBJ_TBL_START_POS2] << 8) | qtm_obp_table_buf[QTM_OBP_OBJ_TBL_START_POS1];
              qtm_obp_obj_table_elem[object_type].obj_size = qtm_obp_table_buf[QTM_OBP_OBJ_TBL_OBJ_SIZE] + 1;
              qtm_obp_obj_table_elem[object_type].num_of_instances = qtm_obp_table_buf[QTM_OBP_OBJ_TBL_NUM_INST] + 1;
              qtm_obp_obj_table_elem[object_type].num_of_report_ids = qtm_obp_table_buf[QTM_OBP_OBJ_TBL_NUM_REPORTID];

              if((qtm_obp_obj_table_elem[object_type].num_of_instances * qtm_obp_obj_table_elem[object_type].num_of_report_ids) != 0)
              {
                  qtm_obp_obj_table_elem[object_type].report_id_low = report_id;
                  report_id = report_id + (qtm_obp_obj_table_elem[object_type].num_of_instances * qtm_obp_obj_table_elem[object_type].num_of_report_ids);
                  qtm_obp_obj_table_elem[object_type].report_id_high = report_id - 1;
              }

              KEY08_PRINTK("qtm_obp object_index = 0x%X\n",i);
              KEY08_PRINTK("qtm_obp object_type = 0x%X\n",qtm_obp_obj_table_elem[object_type].type);
              KEY08_PRINTK("qtm_obp object_start_pos = 0x%X\n",qtm_obp_obj_table_elem[object_type].start_pos);
              KEY08_PRINTK("qtm_obp object_obj_size = 0x%X\n",qtm_obp_obj_table_elem[object_type].obj_size);
              KEY08_PRINTK("qtm_obp object_num_of_instances = 0x%X\n",qtm_obp_obj_table_elem[object_type].num_of_instances);
              KEY08_PRINTK("qtm_obp object_num_of_report_ids = 0x%X\n",qtm_obp_obj_table_elem[object_type].num_of_report_ids);

              memset(qtm_obp_table_buf,0, QTM_OBP_INFO_BLOCK_SIZE);
              reg=reg + QTM_OBP_OBJ_TBL_BLK_SIZE;

         }
     } /* if(qtm_obp_info_block.qtm_info_bytes.num_of_objects != 0x0) */
    KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
}

static int save_low_power_regs (void)
{
    lpIdleValueEeprom = qtm_obp_ts_read_register(qtmTsGl->client, qtm_obp_obj_table_elem[OBJ_ID_GEN_POWERCONFIG_T7].start_pos+QTM_OBP_T7_IDLE);
    if (!lpIdleValueEeprom) 
    { 
        /* This should not be 0, but just in case it is, set to a default value */
        lpIdleValueEeprom=14;
        printk("ERROR : qtm_obp loaded IdleValueEeprom 0");
    }
    printk("qtm_obp lowpwer = 0x%X\n",lpIdleValueEeprom);

    lpActiveValueEeprom = qtm_obp_ts_read_register(qtmTsGl->client, qtm_obp_obj_table_elem[OBJ_ID_GEN_POWERCONFIG_T7].start_pos+QTM_OBP_T7_ACTIVE);
    if (!lpActiveValueEeprom) 
    { 
       /* This should not be 0, but just in case it is, set to a default value */
       lpActiveValueEeprom=14;
       printk("ERROR : qtm_obp loaded ActiveValueEeprom 0");
    }
    printk("qtm_obp lowpwer = 0x%X\n",lpActiveValueEeprom);
	return(0);
}

static int qtm_obp_send_message_processor_pointer (void)
{
    int ret;
    unsigned char value[QTM_OBP_REG_ADDR_SIZE];
    unsigned int reg;

    KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);
    reg=qtm_obp_obj_table_elem[OBJ_ID_GEN_MESSAGEPROCESSOR_T5].start_pos;
    memcpy(&value[0], (unsigned char *)&reg, 2);
    ret = i2c_master_send(qtmTsGl->client, value, 2);
    KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
    return ret;
}

static int qtm_obp_get_touch_sens_data (char * dataPtr)
{
    unsigned char value[QTM_OBP_REG_ADDR_SIZE];
    uint8_t qtm_obp_touch_delta_buf[168];
    uint8_t qtm_obp_touch_ref_buf[168];
    int i = 0;
    int ret = 0;
    int index = 0;
    int length = 0;

    KEY08_PRINTK("%s: Entering.....\n", __FUNCTION__);

    disable_irq(qtmTsGl->client->irq);
    
    memset(qtm_obp_touch_delta_buf,0,qtm_obp_obj_table_elem[OBJ_ID_DEBUG_DELTAS_T2].obj_size);

    value[0] = qtm_obp_obj_table_elem[OBJ_ID_DEBUG_DELTAS_T2].start_pos & 0xFF;
    value[1] = (qtm_obp_obj_table_elem[OBJ_ID_DEBUG_DELTAS_T2].start_pos & 0xFF00)>>8;
    clk_busy_wait(3000);
    
    ret = i2c_master_send(qtmTsGl->client, value, QTM_OBP_REG_ADDR_SIZE);
   
    if (ret < 0)
    {
        KEY08_PRINTK(KERN_ERR "%s: i2c delta write Failed\n", __FUNCTION__);
    }
    clk_busy_wait(3000);

    ret = i2c_master_recv(qtmTsGl->client, qtm_obp_touch_delta_buf, 
                          qtm_obp_obj_table_elem[OBJ_ID_DEBUG_DELTAS_T2].obj_size);

    if (ret < 0)
    {
        KEY08_PRINTK(KERN_ERR "%s: i2c delta Read Failed\n", __FUNCTION__);
    }
    memset(qtm_obp_touch_ref_buf,0,qtm_obp_obj_table_elem[OBJ_ID_DEBUG_REFERENCES_T3].obj_size);  

    value[0] = qtm_obp_obj_table_elem[OBJ_ID_DEBUG_REFERENCES_T3].start_pos & 0xFF;
    value[1] = (qtm_obp_obj_table_elem[OBJ_ID_DEBUG_REFERENCES_T3].start_pos & 0xFF00)>>8;
    clk_busy_wait(3000);
   
    ret = i2c_master_send(qtmTsGl->client, value, QTM_OBP_REG_ADDR_SIZE);
  
    if (ret < 0)
    {
        KEY08_PRINTK(KERN_ERR "%s: i2c ref write Failed\n", __FUNCTION__);
    }

    clk_busy_wait(3000);

    // TODO
    ret = i2c_master_recv(qtmTsGl->client, qtm_obp_touch_ref_buf, 
                          qtm_obp_obj_table_elem[OBJ_ID_DEBUG_REFERENCES_T3].obj_size);

    if (ret < 0)
    {
        KEY08_PRINTK(KERN_ERR "%s: i2c ref Read Failed\n", __FUNCTION__);
    }
   
    // Process
    /* Make sure that MSByte goes into the buffer first */

    index = 0;
    for ( i = 0; i < 84; i++)
    {
       char    lsbyte;
       char    msbyte;
       msbyte = qtm_obp_touch_ref_buf [index];
       index++;
       lsbyte = qtm_obp_touch_ref_buf [index];
       index++;
       *dataPtr = msbyte;
       dataPtr++;
       length++;
       *dataPtr = lsbyte;
       dataPtr++;
       length++;
       KEY08_PRINTK("%s: Ref_Index %d = %d \n", __FUNCTION__, i, lsbyte);
       KEY08_PRINTK("%s: Ref_Index %d = %d \n", __FUNCTION__, i, msbyte);
    }

    index = 0;
    for ( i = 0; i < 84; i++)
    {
       char    lsbyte;
       char    msbyte;
       msbyte = qtm_obp_touch_delta_buf [index];
       index++; 
       lsbyte = qtm_obp_touch_delta_buf [index]; 
       index++; 
       *dataPtr = msbyte;
       dataPtr++;
       length++;
       *dataPtr = lsbyte;
       dataPtr++;
       length++;
       KEY08_PRINTK("%s: Delta_Index %d = %d \n", __FUNCTION__, i, lsbyte);
       KEY08_PRINTK("%s: Delta_Index %d = %d \n", __FUNCTION__, i, msbyte);
    }
    enable_irq(qtmTsGl->client->irq);
    KEY08_PRINTK("%s: Exiting.....\n", __FUNCTION__);
    return ret; 
}

module_init(qtm_obp_ts_init);
// subsys_initcall(qtm_obp_ts_init);
module_exit(qtm_obp_ts_exit);

MODULE_DESCRIPTION("qtm_obp Touchpad Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Boris Dubinsky <Boris.Dubinsky@motorola.com>");
MODULE_VERSION("1.0");
