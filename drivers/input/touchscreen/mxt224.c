/* drivers/input/touchscreen/mxt224.c
 *
 * Copyright (C) 2009 Motorola Inc.
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

#ifdef CONFIG_MACH_SOCIAL
#include <linux/proc_fs.h>
#endif /*CONFIG_MACH_SOCIAL*/

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "mxt224.h"
#include <mach/board.h>
#include <../board-mot.h>

/* touch keys */
#define TOUCH_KEY_NUMBER_MENU   0
#define TOUCH_KEY_NUMBER_BACK   1
#define TOUCH_KEY_NUMBER_SEARCH 3

#define TOUCH_KEYCODE_MENU      139
#define TOUCH_KEYCODE_BACK      158
#define TOUCH_KEYCODE_SEARCH    217

/* Check which key is detected */
#define ATMEL_KEY_DETECT(m, t) \
           ((*((uint8_t *)m + ATMEL_T15_KEYSTATE + t/8)) & (0x1u << t % 8))


struct atmel_ts_data *ts_data = NULL;

/*! @brief Structure used to store the touch data */
static struct atmel_touch_data touch_data;

/* brief structure used to store the ID Information Block */
U_ATMEL_INFO_BYTES_T atmel_info_block;

/* brief structure used to store the object table */
ATMEL_OBJ_TABLE_ELEMENT_T atmel_obj_table_elem[ATMEL_MAX_OBJECT_NUM];

/* brief structure used to store the object configuration */
OBJ_ID_T atmel_obj_id;

/* tracking if the log can be printed */
uint8_t MXT224_DEBUG_ON = FALSE;

/*! @brief tracking the number of process called from upper layer  */
static atomic_t  atmel_num_processes;

/*! @brief Current TouchInput Mode */
static int atmel_touchinput_mode = ATMEL_STANDBY;

/*! @brief Variable used to keep track of the previous mode when using the enable/disable fnct */
static int prev_touch_mode = ATMEL_STANDBY;

/*! @brief Variable used to define if we are currently resetting */
static unsigned char reset = 0;

/*! @brief Variable used to define if the IC is calibrating or not */
static unsigned char calibrating = 0;

/*! @brief This values allows the device to boot from the EEPROM.  This will stop the
driver from programming the IC from the HWCFG values*/
static unsigned int atmel_boot_from_eeprom = 0;

/*! @brief Global value to keep track of which finger id pressed or released*/
static int atmel_finger_press_release = 0;

/*! @brief Time in usec which the user must press virtual button to have the *         
press reported as a key down event */
static unsigned int atmel_long_press_detect_time;

/*! @brief This the time duration that the haptic pulse should be on for */
static unsigned int atmel_button_haptic_time;

/*! @brief This flag determines whether to use the haptic time in the vibe driver
or use the internal touch driver's timer*/
static unsigned int atmel_use_haptic_timer;

/*! @brief This value indicates whether the button should be sent */
static unsigned int atmel_process_button_lo = 0;

/*! @brief This value indicates whether the button press has occured */
static unsigned int atmel_press_button_sent = 0;

/*! @brief This value indicates the button that was previously processed */
static unsigned int atmel_button_reported = ATMEL_NO_BUTTON;

/*! @brief Timestamp of second when the button was initially depressed */
static unsigned int atmel_press_timestamp_s = 0;

/*! @brief Timestamp of microsecond when the button was initially depressed */
static unsigned int atmel_press_timestamp_us = 0;

/*! @brief I2C address for IC */
static int i2c_address = ATMEL_I2C_ADDR;

/*! @brief Bootloader I2C address for IC */
static uint32_t bl_i2c_address = ATMEL_BL_I2C_ADDR;

/*! @brief Pointer to an array of touch key values */
static unsigned short *atmel_touch_key_map;

/*! @brief Number of X-Y touch keys on the screen */
static unsigned int atmel_num_of_touch_keys = ATMEL_NUM_KEYS;

/*! @brief Register number for calibration */
static unsigned char atmel_cal_reg_value = 0;

/*! @brief Total sensor resolution in the x direction */
static unsigned int atmel_x_resolution = 1024;
/*! @brief Total sensor resolution in the y direction */
static unsigned int atmel_y_resolution = 1024;
/*! @brief Total sensor resolution in the z direction */
static unsigned int atmel_z_resolution = 255;
/*! @brief Total sensor resolution for area direction */
static unsigned int atmel_a_resolution = 15;

/*! @brief Display inactive area for the left side of the screen */
static unsigned int atmel_inactive_area_left = 0;
/*! @brief Display inactive area for the right side of the screen */
static unsigned int atmel_inactive_area_right = 0;
/*! @brief Display inactive area for the top side of the screen */
static unsigned int atmel_inactive_area_top = 0;
/*! @brief Display inactive area for the bottom side of the screen */
static unsigned int atmel_inactive_area_bottom = 0;

/*! @brief Number of positions to move before reporting movement in the x direction */
static unsigned int atmel_x_movement_threshold = 0;
/*! @brief Number of positions to move before reporting movement in the y direction */
static unsigned int atmel_y_movement_threshold = 0;
/*! @brief Number of positions to move before reporting movement in the z direction */
static unsigned int atmel_z_movement_threshold = 2;
/*! @brief Number of positions to move before reporting the change in area */
static unsigned int atmel_a_movement_threshold = 2;

/*! @brief Whether to swap the x y coordinate data from the sensor */
static unsigned int atmel_swap_x_y = 0;
/*! @brief Inverts the x data so that max X is now 0 coordinate and vice versa */
static unsigned int atmel_invert_x_data = 0;
/*! @brief Inverts the y data so that max Y is now 0 coordinate and vice versa */
static unsigned int atmel_invert_y_data = 0;

/******************************************************************************
* Local Functions
******************************************************************************/
static int atmel_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int atmel_ts_remove(struct i2c_client *client);
static int atmel_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int atmel_ts_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void atmel_ts_early_suspend(struct early_suspend *h);
static void atmel_ts_late_resume(struct early_suspend *h);
#endif

static int atmel_init_hwcfg(void);
static void atmel_free_hwcfg(void);
static int atmel_get_object_info(struct i2c_client *client);
static void atmel_set_msg_pointer(struct atmel_ts_data *ts);
static void atmel_mode(int mode, struct atmel_ts_data *ts);

static void atmel_reset(struct atmel_ts_data *ts);
static void atmel_gpio_reset(void);
static void atmel_calibrate(struct atmel_ts_data *ts);
static void atmel_sleep(unsigned int t);

static int atmel_i2c_read(struct i2c_client *client, char *buf, int byte_count);
static int atmel_i2c_write(struct i2c_client *client, char * buf, int byte_count);
static int atmel_reg_write(struct i2c_client *client, unsigned int reg,
		           unsigned int length, unsigned char *reg_value);

static void atmel_process_button(struct atmel_touch_data touch_data,
				   struct atmel_ts_data *ts,
				   unsigned int x, unsigned int y,
				   unsigned int finger_id,
				   unsigned int touch);

/* fops control API's */
static int atmel_open (struct inode *, struct file *);
static int atmel_ioctl (struct inode *, struct file *, unsigned int , unsigned long );
static int atmel_release (struct inode *, struct file *);
static int atmel_write (struct file *, const char __user *, size_t , loff_t * );
static void device_enable(unsigned long ,struct atmel_ts_data *);
static int atmel_ts_read_fw_version(struct atmel_ts_data *);

static void atmel_printk(char *fmt, ...);

struct	file_operations	atmel_fops =
{
	.owner		= THIS_MODULE,
	.open		= atmel_open,
	.ioctl		= atmel_ioctl,
	.release	= atmel_release,
	.write		= atmel_write,
};

static struct miscdevice atmel_pf_driver = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name  = ATMEL_I2C_NAME,
	.fops  = &atmel_fops,
};

static const struct i2c_device_id atmel_ts_id[] = {
   { ATMEL_I2C_NAME, 0 },
   { }
};

static struct i2c_driver atmel_ts_driver = {
   .probe = atmel_ts_probe,
   .remove = atmel_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
   .suspend = atmel_ts_suspend,
   .resume = atmel_ts_resume,
#endif

   .id_table = atmel_ts_id,
   .driver = {
      .name	= ATMEL_I2C_NAME,
   },
};


/*!
 * @brief Sleeps the desired amount of time
 *
 * This function is called when the qtm_obp driver needs to sleep.
 *
 * @param t Time to sleep
 */
static void atmel_sleep(unsigned int t)
{
   msleep_interruptible(t);
}


/*!
 * @brief Reads data from Captouch IC
 *
 * This function implements read data from captouch IC through I2C bus.
 * The read is accomplished by using the i2c function i2c_master_recv.
 *
 *
 * @param client     I2C client structure pointer
 * @param buf        Buffer to read to
 * @param byte_count Length of data being written
 *
 * @return This function returns no. of bytes if successful
 */
static int atmel_i2c_read(struct i2c_client *client, char *buf,
			    int byte_count)
{
   int i2c_retry = ATMEL_READ_RETRIES;
   int retval;

   if (client == NULL || buf == NULL) 
   {
      atmel_printk("atmel_i2c_read: Invalid arguments \n");
      return -EINVAL;
   }

   /* Read the data from the device (retrying if necessary) */
   do 
   {
      memset(buf, 0, byte_count);
      retval = i2c_master_recv(client, buf, byte_count);
      if (retval != byte_count) 
      {
         atmel_printk("atmel_i2c_read: i2c_master_recv failed \n");
         clk_busy_wait(ATMEL_READ_WRITE_DELAY);
      }
   } while ((retval != byte_count) && (i2c_retry-- > 0));

   return retval;
}


/*!
 * @brief Writes data to Captouch IC
 *
 * This function implements a write data to IC through I2C bus.
 * The write is accomplished by using i2c function i2c_master_send.
 *
 *
 * @param client     I2C client structure pointer
 * @param buf        Buffer to write to
 * @param byte_count Length of data being written
 *
 * @return This function returns returns no. of bytes transfered if successful
 */
static int atmel_i2c_write(struct i2c_client *client,char * buf, int byte_count)
{
   int i2c_retry = ATMEL_READ_RETRIES;
   int retval;

   if(client == NULL || buf == NULL)
   {
      atmel_printk("atmel_i2c_write: Invalid arguments \n");   
      return -EINVAL; 
   }

   do
   {
      /* Write the data to the device (retrying if necessary) */
      retval = i2c_master_send(client, buf, byte_count);
      if(retval != byte_count)
      {
         atmel_printk("atmel_i2c_write: i2c_master_send failed retval = %i \n", retval);  
	 clk_busy_wait(ATMEL_READ_WRITE_DELAY);
      }

   }while((retval != byte_count) && (i2c_retry-- >=0));

   return retval;
}


/*!
 * @brief Writes a value to a specified register
 *
 * This function implements a write to a specified register.  The write is accomplished
 * by sending the new contents to the i2c function i2c_master_send.
 *
 *
 * @param client     I2C client structure pointer
 * @param reg        Register to write to
 * @param length     Length of data being written
 * @param reg_value  Register value to write
 *
 * @return This function returns 0 if successful
 */
static int atmel_reg_write(struct i2c_client *client, unsigned int reg,
		      unsigned int length, unsigned char *reg_value)
{
   unsigned char value[ATMEL_REG_ADDR_SIZE + ATMEL_MAX_RW_SIZE];
   int retval = 0;
   int i = ATMEL_READ_RETRIES;

   /* Fail if the length is too small */
   if ((length == 0) || (reg_value == NULL)
	   || (length > (ATMEL_REG_ADDR_SIZE + ATMEL_MAX_RW_SIZE))) 
   {
      atmel_printk("atmel_reg_write: length is invalid: %d \n", length);
      return -EINVAL;
   }


   /* Fail if we weren't able to initialize (yet) */
   if (client == NULL) 
   {
      atmel_printk("atmel_reg_write: initialization failed\n");
      return -EINVAL;
   }

   /* Copy the data into a buffer for correct format */
   memcpy(&value[0], (unsigned char *) &reg, ATMEL_REG_ADDR_SIZE);
   memcpy(&value[ATMEL_REG_ADDR_SIZE], reg_value, length);

   /* Write the data to the device (retrying if necessary) */
   do 
   {
      retval = i2c_master_send(client, value,
                               ATMEL_REG_ADDR_SIZE + length);

      /* On failure, output the error code and delay before trying again */
      if ((retval < 0)
	      || (retval != (ATMEL_REG_ADDR_SIZE + length)))
      {
         atmel_printk("atmel_reg_write: write of reg 0x%X failed: %d\n",
			     reg, retval);
         clk_busy_wait(ATMEL_READ_WRITE_DELAY);
      }
   } while ((retval < 0) && (i-- > 0));

   /* On success, set retval to 0 (i2c_master_send returns no. of bytes transfered) */
   if (retval == (ATMEL_REG_ADDR_SIZE + length)) 
   {
      retval = 0;
   }

   /* Delay after every I2C access or IC will NAK */
   clk_busy_wait(ATMEL_READ_WRITE_DELAY);

   return retval;
}


/*!
 * @brief Reads firmware version from IC a value to a specified register
 *
 * This function implements a rads the the value from Chip ID register of Captouch IC
 * by i2c function i2c_master_recv.
 *
 *
 * @param ts driver private structue
 *
 * @return This function returns 0 if successful
 */
 
static int  atmel_ts_read_fw_version(struct atmel_ts_data *ts)
{
   struct i2c_client *client;
   uint8_t data[6];
   int rc;
   int version = -1;
   uint16_t reg;
   unsigned char value[ATMEL_REG_ADDR_SIZE];
   U_ATMEL_INFO_BYTES_T        local_atmel_info_block;
   int retval = 0;
  
   client = ts->client;
  
   if ( ts->mode == MOT_TOUCH_MODE_NORMAL )
   {
      memset(local_atmel_info_block.atmel_info_buf,0,ATMEL_INFO_BLOCK_SIZE);
      reg=0;

      value[0] = 0;
      value[1] = 0;

      retval = i2c_master_send(ts->client, value, ATMEL_REG_ADDR_SIZE);

      /* On failure, output the error code and delay before trying again */
      if (retval < 0)
      {
         atmel_printk( "atmel_reg_write: write of reg 0x%X failed: %d\n", reg, retval);
         clk_busy_wait(ATMEL_READ_WRITE_DELAY);
      }
      retval = i2c_master_recv(ts->client, local_atmel_info_block.atmel_info_buf, 7);
   
      atmel_printk( "qtm_obj: version   :%x \n",local_atmel_info_block.atmel_info_bytes.version);
	
      return local_atmel_info_block.atmel_info_bytes.version;
   }
   else
   {
      /* we are in bootloader mode. So, read 1 byte from bootloader client */
      version = -1;
      client->addr = bl_i2c_address;
      if ((rc = i2c_master_recv(client, data, 1 )) < 0 )
      {
         atmel_printk("atmelts_read_fw_version: i2c_master_revc bootloader error  %d\n,",rc);
      }
   }
   return(version);
}


/*!
 * @brief Initilazes Hardware config data
 *
 * This function will initialize the hardware cofiguration data
 * this data include I2C address, mode information and other adjustable
 * paramenters.
 *
 * @return int Returns error if Hardware config data is not found
 */
static int atmel_init_hwcfg(void)
{
   uint8_t obj_size = 0;
   uint8_t ret = 0;

#ifdef CONFIG_MACH_SOCIAL
   unsigned char obj_t7[] =  {0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00};
   unsigned char obj_t8[] =  {0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x05, 0x00};
   unsigned char obj_t9[] =  {0x8B, 0x00, 0x00, 0x0D, 0x09, 0x00, 0x21, 0x2D,
                              0x01, 0x03, 0x32, 0x07, 0x02, 0x00, 0x0A, 0x05,
                              0x05, 0x05, 0xDF, 0x01, 0x3F, 0x01, 0x00, 0x00,
                              0x00, 0x00, 0x9C, 0x3E, 0x00, 0x00};
#else
   unsigned char obj_t7[] =  {0x32, 0x0F, 0x32, 0x00, 0x00, 0x00};
   unsigned char obj_t8[] =  {0x08, 0x00, 0x14, 0x0A, 0x00, 0x00, 0x0A, 0x0F};
   unsigned char obj_t9[] =  {0x83, 0x00, 0x00, 0x0D, 0x09, 0x00, 0x20, 0x2D,
                              0x02, 0x03, 0x00, 0x0A, 0x00, 0x0E, 0x02, 0x0A,
                              0x0A, 0x0A, 0xDF, 0x01, 0x3F, 0x01, 0x00, 0x00,
                              0x00, 0x00, 0x9C, 0x3E, 0x00, 0x00};
#endif /* CONFIG_MACH_SOCIAL */
   unsigned char obj_t15[] = {0x83, 0x00, 0x0A, 0x04, 0x01, 0x00, 0x20, 0x1E,
                              0x02, 0x00, 0x00};
   unsigned char obj_t23[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00};
   unsigned char obj_t19[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
   unsigned char obj_t25[] = {0x00, 0x00, 0xF8, 0x2A, 0x58, 0x1B, 0x70, 0x17,
                              0xE8, 0x03, 0x00, 0x00, 0x00, 0x00};
   unsigned char obj_t28[] = {0x00, 0x00, 0x00, 0x04, 0x08, 0x0F};
   unsigned char obj_t20[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00};
   unsigned char obj_t22[] = {0x08, 0x00, 0x00, 0x19, 0x00, 0xE7, 0xFF, 0x04,
                              0x32, 0x00, 0x01, 0x0A, 0x0F, 0x14, 0xFF, 0xFF,
                              0x04};
   unsigned char obj_t24[] = {0x00, 0x0A, 0x00, 0x00, 0x00, 0x64, 0x64, 0x01,
                              0x0A, 0x14, 0x28, 0x4B, 0x00, 0x02, 0x00, 0x64,
                              0x00, 0x19, 0x00};
   unsigned char obj_t27[] = {0x00, 0x02, 0x00, 0x00, 0x03, 0x23, 0x00};
   unsigned char obj_t38[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
   unsigned char obj_t18[] = {0x00, 0x00};
   
   atmel_cal_reg_value = 0x01;

   atmel_boot_from_eeprom = 0;

   atmel_x_resolution = 320;

   atmel_y_resolution = 480;

   atmel_z_resolution = 255;

   atmel_a_resolution = 15;

   atmel_inactive_area_left = 0;

   atmel_inactive_area_right = 0;

   atmel_inactive_area_top = 0;

   atmel_inactive_area_bottom = 0;

   atmel_x_movement_threshold = 0;

   atmel_y_movement_threshold = 0;

   atmel_z_movement_threshold = 2;

   atmel_a_movement_threshold = 2;

   atmel_swap_x_y = 0;
   
   atmel_invert_x_data = 0;

   atmel_invert_y_data = 0;

   memset(&atmel_obj_id, 0, sizeof(OBJ_ID_T));
   obj_size = sizeof(obj_t7) / sizeof(obj_t7[0]);
   atmel_obj_id.object[OBJ_ID_GEN_POWERCONFIG_T7] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_GEN_POWERCONFIG_T7] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_GEN_POWERCONFIG_T7] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_GEN_POWERCONFIG_T7] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_GEN_POWERCONFIG_T7], obj_t7, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t8) / sizeof(obj_t8[0]);
   atmel_obj_id.object[OBJ_ID_GEN_ACQUIRECONFIG_T8] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_GEN_ACQUIRECONFIG_T8] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_GEN_ACQUIRECONFIG_T8] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_GEN_ACQUIRECONFIG_T8] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_GEN_ACQUIRECONFIG_T8], obj_t8, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t9) / sizeof(obj_t9[0]);
   atmel_obj_id.object[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9], obj_t9, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t15) / sizeof(obj_t15[0]);
   atmel_obj_id.object[OBJ_ID_TOUCH_KEYARRAY_T15] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_TOUCH_KEYARRAY_T15] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_TOUCH_KEYARRAY_T15] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_TOUCH_KEYARRAY_T15] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_TOUCH_KEYARRAY_T15], obj_t15, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t23) / sizeof(obj_t23[0]);
   atmel_obj_id.object[OBJ_ID_TOUCH_PROXIMITY_T23] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_TOUCH_PROXIMITY_T23] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_TOUCH_PROXIMITY_T23] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_TOUCH_PROXIMITY_T23] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_TOUCH_PROXIMITY_T23], obj_t23, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t19) / sizeof(obj_t19[0]);
   atmel_obj_id.object[OBJ_ID_SPT_GPIOPWM_T19] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_SPT_GPIOPWM_T19] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_SPT_GPIOPWM_T19] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_SPT_GPIOPWM_T19] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_SPT_GPIOPWM_T19], obj_t19, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t25) / sizeof(obj_t25[0]);
   atmel_obj_id.object[OBJ_ID_SPT_SELFTEST_T25] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_SPT_SELFTEST_T25] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_SPT_SELFTEST_T25] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_SPT_SELFTEST_T25] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_SPT_SELFTEST_T25], obj_t25, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t28) / sizeof(obj_t28[0]);
   atmel_obj_id.object[OBJ_ID_SPT_CTECONFIG_T28] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_SPT_CTECONFIG_T28] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_SPT_CTECONFIG_T28] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_SPT_CTECONFIG_T28] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_SPT_CTECONFIG_T28], obj_t28, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t20) / sizeof(obj_t20[0]);
   atmel_obj_id.object[OBJ_ID_PROCI_GRIPFACESUPPRESSION_T20] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_PROCI_GRIPFACESUPPRESSION_T20] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_PROCI_GRIPFACESUPPRESSION_T20] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_PROCI_GRIPFACESUPPRESSION_T20] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_PROCI_GRIPFACESUPPRESSION_T20], obj_t20, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t22) / sizeof(obj_t22[0]);
   atmel_obj_id.object[OBJ_ID_PROCG_NOISESUPPRESSION_T22] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_PROCG_NOISESUPPRESSION_T22] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_PROCG_NOISESUPPRESSION_T22] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_PROCG_NOISESUPPRESSION_T22] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_PROCG_NOISESUPPRESSION_T22], obj_t22, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t24) / sizeof(obj_t24[0]);
   atmel_obj_id.object[OBJ_ID_PROCI_ONETOUCHGESTUREPROCESSOR_T24] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_PROCI_ONETOUCHGESTUREPROCESSOR_T24] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_PROCI_ONETOUCHGESTUREPROCESSOR_T24] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_PROCI_ONETOUCHGESTUREPROCESSOR_T24] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_PROCI_ONETOUCHGESTUREPROCESSOR_T24], obj_t24, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t27) / sizeof(obj_t27[0]);
   atmel_obj_id.object[OBJ_ID_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_PROCI_TWOTOUCHGESTUREPROCESSOR_T27] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_PROCI_TWOTOUCHGESTUREPROCESSOR_T27], obj_t27, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   obj_size = sizeof(obj_t18) / sizeof(obj_t18[0]);
   atmel_obj_id.object[OBJ_ID_PROCI_GESTURESPROCESS_T18] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_PROCI_GESTURESPROCESS_T18] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_PROCI_GESTURESPROCESS_T18] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_PROCI_GESTURESPROCESS_T18] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_PROCI_GESTURESPROCESS_T18], obj_t18, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }


   obj_size = sizeof(obj_t38) / sizeof(obj_t38[0]);
   atmel_obj_id.object[OBJ_ID_SPT_USERDATA_T38] = kmalloc(obj_size, GFP_KERNEL);
   if (atmel_obj_id.object[OBJ_ID_SPT_USERDATA_T38] != NULL)
   {
      atmel_obj_id.obj_cfgs[OBJ_ID_SPT_USERDATA_T38] = TRUE;
      atmel_obj_id.obj_id_cfg_size[OBJ_ID_SPT_USERDATA_T38] = obj_size;
      memcpy(atmel_obj_id.object[OBJ_ID_SPT_USERDATA_T38], obj_t38, obj_size);
   }
   else
   {
      ret = 1;
      goto exit_kmalloc_failed;
   }

   return ret;

exit_kmalloc_failed:
   atmel_free_hwcfg();
   return ret;
}

static void atmel_free_hwcfg(void)
{
   uint8_t i = 0;

   for ( i=0; i < ATMEL_MAX_OBJECT_NUM; i++ )
   {
      if ( (atmel_obj_id.obj_cfgs[i] == TRUE) && (atmel_obj_id.object[i] != NULL) )
      {
         kfree(atmel_obj_id.object[i]);
         atmel_obj_id.object[i] = NULL;
      }
   }
}



/*!
 * @brief Reads object data from Captouch IC
 *
 * This function reads the object information from the IC and populates
 * the object structures.
 *
 * @param client     I2C client structure pointer
 *
 * @return This function returns a negative value if the read fails
 */
static int atmel_get_object_info(struct i2c_client *client)
{
   uint16_t reg = ATMEL_OBJ_INFO_REG;
   unsigned char reg_val = 0x00;
   int ret = 0;
   uint8_t object_type = 0;
   uint8_t report_id = 0;
   int i = 0;
   uint8_t atmel_table_buf[ATMEL_INFO_BLOCK_SIZE];

   atmel_printk("[LJ] Entering atmel_get_object_info()\n");

   /* OBP */
   ret = atmel_i2c_write(client, &reg, ATMEL_REG_ADDR_SIZE);
   /* On failure, output the error code and delay before trying again */
   if (ret < 0) 
   {
      atmel_printk("atmel_reg_write: write of reg 0x%X failed: %d\n",
		  reg, ret);
      return ret;
   }
   memset(atmel_info_block.atmel_info_buf, 0, ATMEL_INFO_BLOCK_SIZE);
   ret = atmel_i2c_read(client, atmel_info_block.atmel_info_buf,
			  ATMEL_INFO_BLOCK_SIZE);
   if (ret < 0) 
   {
      atmel_printk("%s: I2C failed to read data from the device\n",__func__);
      return ret;
   }
   atmel_printk("[LJ] atmel_id: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n", 
                                atmel_info_block.atmel_info_buf[0],
                                atmel_info_block.atmel_info_buf[1],
                                atmel_info_block.atmel_info_buf[2],
                                atmel_info_block.atmel_info_buf[3],
                                atmel_info_block.atmel_info_buf[4],
                                atmel_info_block.atmel_info_buf[5],
                                atmel_info_block.atmel_info_buf[6]);

   /* Check to see if there are any touch objects */
   if (atmel_info_block.atmel_info_bytes.num_of_objects != 0x0) 
   {  
      reg_val = ATMEL_OBJ_INFO_REG;
      reg = ATMEL_INFO_BLOCK_SIZE;
      report_id = 1;

      memset(atmel_table_buf, 0, ATMEL_INFO_BLOCK_SIZE);
      memset(atmel_obj_table_elem, 0, 
                  ATMEL_MAX_OBJECT_NUM * sizeof(ATMEL_OBJ_TABLE_ELEMENT_T));
      
      for (i = 0;
           i < atmel_info_block.atmel_info_bytes.num_of_objects;
           i++) 
      {
         /* Query the registers for the object information */
         ret = atmel_i2c_write(client, &reg, ATMEL_REG_ADDR_SIZE);
         /* On failure, output the error code */
         if (ret < 0) 
         {
            atmel_printk("%s: 1\n",__func__);
            atmel_printk("%s:write of reg 0x%X failed: %d\n",
                        __func__, reg, ret);
            return ret;
         }
         /* Read the data from the device */
         ret = atmel_i2c_read(client, atmel_table_buf,
                                ATMEL_OBJ_TBL_BLK_SIZE);
         if (ret < 0) 
         {
            atmel_printk("%s: I2C failed to read data\n",__func__);
            return ret;
         }

         /* Copy obj table data into struct at obj type offset */
         object_type = atmel_table_buf[ATMEL_OBJ_TBL_TYPE];
         atmel_obj_table_elem[object_type].type =
                atmel_table_buf[ATMEL_OBJ_TBL_TYPE];
         atmel_obj_table_elem[object_type].start_pos =
                (atmel_table_buf[ATMEL_OBJ_TBL_START_POS2] << 8) |
                atmel_table_buf[ATMEL_OBJ_TBL_START_POS1];
         atmel_obj_table_elem[object_type].obj_size =
                atmel_table_buf[ATMEL_OBJ_TBL_OBJ_SIZE] + 1;
         atmel_obj_table_elem[object_type].num_of_instances =
                atmel_table_buf[ATMEL_OBJ_TBL_NUM_INST] + 1;
         atmel_obj_table_elem[object_type].num_of_report_ids =
                atmel_table_buf[ATMEL_OBJ_TBL_NUM_REPORTID];

         atmel_printk("%s:Object type 0x%x\n", __func__, object_type);
         atmel_printk("%s:Start position 0x%x\n", __func__,
                     atmel_obj_table_elem[object_type].start_pos);
         atmel_printk("%s:Object size 0x%x\n", __func__,
                     atmel_obj_table_elem[object_type].obj_size);
         atmel_printk("%s:Num of instances 0x%x\n", __func__,
                     atmel_obj_table_elem[object_type].num_of_instances);
         atmel_printk("%s:Num of report_ids 0x%x\n", __func__,
                     atmel_obj_table_elem[object_type].num_of_report_ids);

         if ((atmel_obj_table_elem[object_type].num_of_instances *
              atmel_obj_table_elem[object_type].num_of_report_ids) != 0) 
         {
            atmel_obj_table_elem[object_type].report_id_low = report_id;
            report_id = report_id +
                        (atmel_obj_table_elem[object_type].num_of_instances *
                         atmel_obj_table_elem[object_type].num_of_report_ids);
            atmel_obj_table_elem[object_type].report_id_high = report_id - 1;
            atmel_printk("%s:Report ID 0x%x\n", __func__, report_id);
         }

         atmel_printk("%s:report_ids low 0x%x\n", __func__,
                     atmel_obj_table_elem[object_type].report_id_low);

         atmel_printk("%s:report_ids high 0x%x\n", __func__,
                     atmel_obj_table_elem[object_type].report_id_high);

         memset(atmel_table_buf, 0, ATMEL_INFO_BLOCK_SIZE);
         reg += ATMEL_OBJ_TBL_BLK_SIZE;
      }

   } /* if(atmel_info_block.atmel_info_bytes.num_of_objects != 0x0) */
   else
   {
      atmel_printk("%s:num of objects is 0\n", __func__);
   }

   atmel_printk("[LJ] Exiting atmel_get_object_info()\n");

   return 0;
}

/*!
 * @brief Sets the pointer of the IC to the message pointer.
 *
 * This function implements a write to a specified register.  The write is accomplished
 * by sending the new contents to the i2c function i2c_master_send.
 *
 *
 * @param ts     Touchscreen storage structure
 *
 */
static void atmel_set_msg_pointer(struct atmel_ts_data *ts)
{
   unsigned char reg_value = 0x0000;
   uint16_t reg;
   
   /* Set message pointer. */
   reg = atmel_obj_table_elem[OBJ_ID_GEN_MESSAGEPROCESSOR_T5].start_pos;

   atmel_i2c_write(ts->client, &reg, ATMEL_REG_ADDR_SIZE);

   atmel_printk("%s:Send Message Processor Obj Ptr:[%X]\n",__func__, reg);
}


/*!
 * @brief Changes the mode of the Qtm_Obp
 *
 * The mode of the Qtm_Obp has the ability to change dynamically by reconfiguring
 * the registers of the Qtm_Obp.
 *
 * @param  mode   New mode to be configured
 * @param  ts     Structure pointer to the touch screen data
 */
static void atmel_mode(int mode, struct atmel_ts_data *ts)
{
   int err = 0;

   atmel_printk("[LJ] Entering atmel_mode()\n");

   /* Check to see if the device is in reset */
   if (reset == 1) 
   {
      atmel_printk("atmel_mode:Condition for return reset= %i\n", reset);
      return;
   }

   /* Don't write the same mode again if it is already set or if the mode
      is set to -1 */
   if ((atmel_touchinput_mode == mode) || (mode == ATMEL_DONT_CHANGE_MODE)) 
   {
      atmel_printk("atmel_mode:Condition for return mode= %i mode_in= %i.\n",
		     atmel_touchinput_mode, mode);
      return;
   }

   /* Set new Touch Input mode and Must add 1 so that the application
      modes match HWCFG. */
   prev_touch_mode = atmel_touchinput_mode;
   atmel_touchinput_mode = mode;

   atmel_printk("atmel_mode: mode change (%d).\n", mode);

   if (OBJ_ID_GEN_POWERCONFIG_T7 == mode) 
   {
      atmel_printk("atmel_mode: T7 Initial.\n");
      err = atmel_reg_write(ts->client,
                            atmel_obj_table_elem[OBJ_ID_GEN_POWERCONFIG_T7].start_pos,
                            (atmel_obj_id.obj_id_cfg_size[OBJ_ID_GEN_POWERCONFIG_T7] / 2),
                            (atmel_obj_id.object[OBJ_ID_GEN_POWERCONFIG_T7]));

   }

   if (-OBJ_ID_GEN_POWERCONFIG_T7 == mode) 
   {
      atmel_printk("atmel_mode: T7 Standby , length: %d \n",
                   atmel_obj_id.obj_id_cfg_size[OBJ_ID_GEN_POWERCONFIG_T7]);
      err = atmel_reg_write(ts->client,
                            atmel_obj_table_elem[OBJ_ID_GEN_POWERCONFIG_T7].start_pos,
                            (atmel_obj_id.obj_id_cfg_size[OBJ_ID_GEN_POWERCONFIG_T7] / 2),
                            (atmel_obj_id.object[OBJ_ID_GEN_POWERCONFIG_T7] + 3));

   }


   /* Check to see if the device is configured to boot from the EEPROM */
   if (atmel_boot_from_eeprom == ATMEL_BOOT_FROM_EEPROM) 
   {
      atmel_printk("atmel_mode: Allowing the device to boot from EEPROM\n");
      return;
   }

   atmel_printk("atmel_mode: Programming to mode (%d).\n", mode);

   if ((mode >= OBJ_ID_GEN_ACQUIRECONFIG_T8)
	    && (mode <= OBJ_ID_SPT_USERDATA_T38)) 
   {
      atmel_printk("atmel_mode: T8,  length: %d \n", atmel_obj_id.obj_id_cfg_size[mode]);
      err = atmel_reg_write(ts->client,
                              atmel_obj_table_elem[mode].start_pos,
                              (atmel_obj_id.obj_id_cfg_size[mode]),
                              (atmel_obj_id.object[mode]));

   }

   atmel_printk("[LJ] Entering atmel_mode()\n");
}


/*!
 * @brief Callback function to control the devices
 *
 * This function is called through ioctl when it receives a
 * request from an application to enable or disable a device.
 *
 * @param en unsigned char indicating whether to enable or disable the device
 */
static void device_enable(unsigned long en ,struct atmel_ts_data *ts)
{
   /* If the device is to be enabled, then restore the previous mode. */
   if(en > 0)
   {
      atmel_mode(OBJ_ID_GEN_POWERCONFIG_T7, ts);
      atmel_mode(OBJ_ID_GEN_ACQUIRECONFIG_T8, ts);
      atmel_mode(OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9, ts);
      //atmel_mode(OBJ_ID_PROCG_SIGNALFILTER_T16, ts);
      //atmel_mode(OBJ_ID_PROCI_LINEARIZATIONTABLE_T17, ts);

      /* Reset to the message pointer */
      atmel_set_msg_pointer(ts);
   }
   /* Otherwise disable all devices by setting default mode. */
   else
   {
      atmel_mode(-OBJ_ID_GEN_POWERCONFIG_T7,ts);
      atmel_mode(-OBJ_ID_GEN_ACQUIRECONFIG_T8, ts);
      atmel_mode(-OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9, ts);
      //atmel_mode(-OBJ_ID_PROCG_SIGNALFILTER_T16, ts);
      //atmel_mode(-OBJ_ID_PROCI_LINEARIZATIONTABLE_T17, ts);
   }
}


/*!
 * @brief Resets the Qtm_Obp
 *
 * This function asserts the hardware reset line to the Qtm_Obp and waits the
 * appropriate times to allow the IC to be reset.
 */
static void atmel_reset(struct atmel_ts_data *ts)
{
   /* Identify as being in reset */
   reset = 1;	
   atmel_gpio_reset();
}


/*!
 * @brief Resets the GPIO signal to Qtm_Obp IC
 *
 * This function is called when the Qtm_Obp IC needs to be reset.
 */
static void atmel_gpio_reset(void)
{
   /* Drop the reset line to put the IC into reset */
   gpio_set_value(TOUCH_RST_N, 0);

   /* Delay to allow the reset to take effect */
   atmel_sleep(20);

   /* Bring the reset line high to take the IC out of reset */
   gpio_set_value(TOUCH_RST_N, 1);

   /* Delay to allow the IC to come out of reset before we talk */
   atmel_sleep(70);
}


/*!
 * @brief Forces the IC to calibrate the touch screen
 *
 * This function is called to force a calibration of the touch screen
 * this can be called with a flip action, upon reset or after programming.
 *
 * @param ts     Touch screen structure pointer.
 */
void atmel_calibrate(struct atmel_ts_data *ts)
{
   int err = 0;

   atmel_printk("[LJ] Entering atmel_calibrate()\n");
   /* Send in the calibrate command. Non-zero forces calibration */
   err = atmel_reg_write(ts->client,
             atmel_obj_table_elem[OBJ_ID_GEN_COMMANDPROCESSOR_T6].start_pos
				+ ATMEL_T6_CALIBRATE, 1,
                           &atmel_cal_reg_value);

   if (err < 0) 
   {
      atmel_printk("%s: Failed to send the calibrate command to the IC.\n", __func__);
   }

   atmel_printk("[LJ] Exiting atmel_calibrate()\n");
}


/**
 * Ioctl implementation
 *
 * Device can be used only by one user process at the time
 * ioctl returns -EBUSY if device is already in use
 *
 * @param node File in /proc
 * @param filp Kernel level file structure
 * @param cmd Ioctl command
 * @param arg Ioctl argument
 * @return 0 in success, or negative error code
 */
static int atmel_ioctl(struct inode *node, struct file *filp, unsigned int cmd, unsigned long arg)
{
   int	rc = -1;
   int mode_value = 0;
   /* static unsigned char FWupdateInfo[3];   */
   struct atmel_ts_data *ts;
  
   ts = ts_data;
   if(ts == NULL)
   {
      atmel_printk("atmel_ioctl : Error geting ts_data pointer\n");
      return rc;
   }

   switch (cmd) 
   {
      case MOT_TOUCH_IOCTL_GET_TOUCH_MODE:	
         atmel_printk("atmel_ioctl:Get Touch Mode\n");
         if(put_user( atmel_touchinput_mode, (int *)arg) != 0)
         {
            atmel_printk("atmel_ioctl: Error copying touch mode to user space\n");
            return -EFAULT;
         }
         rc = 1;
         break;
      case MOT_TOUCH_IOCTL_SET_TOUCH_MODE:	
         if(get_user( mode_value, (int *)arg) != 0)
         {
            atmel_printk("atmel_ioctl: Error copying touch mode from  user space\n");
            return -EFAULT;
         }
         atmel_mode(mode_value, ts);
         rc = 1;
         break;
      case MOT_TOUCH_IOCTL_GET_TOUCH_DATA:
         /* Return the Touch data back to the caller. */
         if((copy_to_user((void *)arg,&(ts->touch_data), sizeof(MOTO_TS_DATA_T))) != 0)
         {
            atmel_printk("atmel_ioctl: Error copying touch data to user space\n");
            return -EFAULT;
         }
         rc=1;
         break;
      case MOT_TOUCH_IOCTL_GET_FW_VERSION:	
         if(ts->FwUpdate_Status == ATMEL_FW_UPDATE_NOT_DONE)
         {
            if(put_user( ts->FwVersion, (int *)arg) != 0)
            {
               atmel_printk("atmel_ioctl: Failed to copy Firmware version  to user space\n");
               return -EFAULT;
            }
            rc = 1;
         }
         else
         {
            atmel_printk("atmel_ioctl : Driver will not support this operation in this mode \n"); 
         }
         break;

      case MOT_TOUCH_IOCTL_ENABLE_TOUCH:
         device_enable(ATMEL_INITIAL,ts);
         rc = 1;
         break;
      case MOT_TOUCH_IOCTL_DISABLE_TOUCH:
         device_enable(ATMEL_STANDBY,ts);
         rc = 1;
         break;
      case MOT_TOUCH_IOCTL_SET_BOOT_MODE:
         rc = -ENOTSUPP;
	 break;
      case MOT_TOUCH_IOCTL_SET_NORMAL_MODE:
         rc = 1;
         ts->client->addr = i2c_address;
         ts->mode = MOT_TOUCH_MODE_NORMAL;
         ts->driver_state = MOT_TOUCH_BL_WAITING_FOR_NOTHING;
         rc = ts->mode;
         if(ts->FwUpdate_Status == ATMEL_FW_UPDATE_NOT_DONE)
         {
            atmel_printk("atmel_ioctl: Driver state is QTM_OBP_FW_UPDATE_DONE now\n");
            ts->FwUpdate_Status = ATMEL_FW_UPDATE_DONE;
            if((int)arg == 1)
            {
               atmel_printk("atmel_ioctl: Set Normal Mode. Resetting IC..\n");
               ts->mode = MOT_TOUCH_MODE_UNKNOWN;
               atmel_gpio_reset();
            }
            break;
         }
         atmel_printk("atmel_ioctl: Mode is set to %d (NORMAL Mode)\n", ts->mode);        
         break;

      case MOT_TOUCH_IOCTL_BL_GET_STATUS:
         /* atmel_printk("ioctl: get_bl_state = %d\n",ts->driver_state); */
         if(ts->FwUpdate_Status == ATMEL_FW_UPDATE_NOT_DONE)
         {
            if(put_user( ts->driver_state, (int *)arg) != 0)
            {
               atmel_printk("atmel_ioctl: Failed to copy driver state to user space\n");
               return -EFAULT;
            }
            rc = 1;
         }
         else
         {
            atmel_printk("atmel_ioctl: Driver will not support this operation in this mode \n"); 
         }
         break;
      case MOT_TOUCH_IOCTL_GET_IC_MODE:
         if(put_user( ts->mode, (int *)arg) != 0)
         {
            atmel_printk("atmel_ioctl: Failed to copy driver state to user space\n");
            return -EFAULT;
         }
         rc = 1;
         break;
/*
      case MOT_TOUCH_IOCTL_RESET_TOUCH_IC:
         ts->driver_state = MOT_TOUCH_BL_FW_NOT_STARTED;
         ts->mode = MOT_TOUCH_MODE_UNKNOWN;
         qtm_obp_gpio_reset();
         break;
      case MOT_TOUCH_IOCTL_GET_DRIVER_ID:
         if(put_user( MOT_TOUCH_QTM_OBP, (int *)arg) != 0)
         {
            atmel_printk("atmel_ioctl: Failed to copy driver name to user space\n");
            return -EFAULT;
         }
         break;
*/
      default :
         atmel_printk("atmel_ioctl:INVALID IOCTL\n ");

   }
   return rc;
}


/**
 * Open device
 *
 * @param inode
 * @param filp
 * @return 0
 */
static int atmel_open(struct inode *inode, struct file *filp)
{
   atmel_printk("In atmel_open\n");
   atomic_dec (&atmel_num_processes);
   if (atomic_read(&atmel_num_processes) < 0)
   {
      return -1;
   }
   return 0;
}


/**
 * Release device
 *
 * @param inode
 * @param filp
 * @return 0
 */
static int atmel_release(struct inode *inode, struct file *filp)
{
   atmel_printk("In atmel_release\n");
   atomic_inc (&atmel_num_processes);
   return 0;
}/**


 * Write to device
 *
 * @param flip
 * @param buf
 * @param size
 * @param pos
 * @return number of bytes written to the device
 */
static int atmel_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos )
{
   int	retVal=-1;
   unsigned char	 kernelBuffer[400];
   struct atmel_ts_data *ts = ts_data;

   if( ts == NULL)
   {
      atmel_printk("atmel_write : Invalid pointer\n");
      return retVal;
   }
   if( buf == NULL)
   {
      atmel_printk("atmel_write : User buffer is invalid , Exiting ..... \n");
      return retVal;
   }

   if ( ts->mode == MOT_TOUCH_MODE_BOOTLOADER )
   {
      /* get data from user space into kernel space */
      if( copy_from_user(kernelBuffer, buf, (unsigned long)count))
      {
         atmel_printk("atmel_write : copy_from_user Failed \n");
         ts->driver_state = MOT_TOUCH_BL_COPY_DATA_ERROR;
         retVal = -EFAULT;
         goto atmel_write_error;
      }

      atmel_printk("atmel_write: copy_from_user() copied %i bytes \n", (int)count);

      ts->client->addr = bl_i2c_address;
      atmel_printk("atmel_write: record to IC: size=%i\n",count); 

      /* Updating Bootloader Mode Status  */
      ts->driver_state = MOT_TOUCH_BL_WAITING_FOR_STATUS;

      retVal = atmel_i2c_write(ts->client,kernelBuffer, count);

      /* On failure, output the error code and delay before trying again */
      if (retVal < 0)
      {
         atmel_printk("atmel_write: write failed: %d\n", retVal);
         ts->driver_state = MOT_TOUCH_BL_DATA_WRITE_ERROR;
         goto atmel_write_error;
      }
      else
      {
         atmel_printk("atmel_write: atmel_i2c_write(): OK,Send Size %d  \n",count);
#if 0
         atmel_printk("atmel_write: kernelBuffer[0-3] = %02x%02x%02x%02x  \n", kernelBuffer[0], kernelBuffer[1], kernelBuffer[2], kernelBuffer[3]);
#endif
         retVal = count;          
      }
   }     
   else if ( ts->mode == MOT_TOUCH_MODE_NORMAL )
   {
      atmel_printk("atmel_write : Your Write to indented device is failed, In this mode write not supported \n"); 
   }

   atmel_printk("atmel_write: exiting\n");
   return retVal;

atmel_write_error:
   atmel_printk("atmel_write: exiting with error\n");
   return retVal;
}


/*!
 * @brief This is the work func which will process virtual buttons on the touch screen
 *
 * This function implements the virtual touch buttons.
 *
 *
 * @param touch_data  touch data structure.
 * @param ts     Touch screen structure pointer.
 * @param x      Raw X data from the IC
 * @param y      Raw Y data from the IC
 * @param finger_id  Indicates the id of the finger.
 * @param touch  Indicates the status of the touch.
 *
 * @return None
 */
static void atmel_process_button(struct atmel_touch_data touch_data,
				 struct atmel_ts_data *ts,
				 unsigned int x, 
				 unsigned int y,
				 unsigned int finger_id,
				 unsigned int touch)
{
   struct timeval tv;
   int i = 0;
   int button_value = ATMEL_DO_NOT_PROCESS_LIFTOFF;
   unsigned int current_time_delta_us = 0;
   unsigned int current_time_delta_s  = 0;
   /* Loop through the key map and find the button that is associated with
	   the x values */
   for (i = 0;
        i <= (atmel_num_of_touch_keys * ATMEL_TOUCH_KEY_TOTAL_VAL);) 
   {
      if (((x < atmel_touch_key_map[ATMEL_TOUCH_KEY_X_MAX + i])
            && (x > atmel_touch_key_map[ATMEL_TOUCH_KEY_X_START + i]))
           && ((y < atmel_touch_key_map[ATMEL_TOUCH_KEY_Y_MAX + i])
            && (y > atmel_touch_key_map[ATMEL_TOUCH_KEY_Y_START + i]))) 
      {
         if (touch_data.ignore_button[finger_id] == ATMEL_IGNORE) 
         {
            /* Ignore any button touches */
            return;
         } 
         else 
         {
            /* Store the button value and allow processing */
            button_value = atmel_touch_key_map[ATMEL_TOUCH_KEY_KEYCODE + i];
            touch_data.button_area[finger_id] = ATMEL_KEY_PRESS;
         }
         break;
      } 
      else 
      {
         /* There was no match to the x coordinates so do not process anything */
         button_value = ATMEL_NO_BUTTON;
      }
      i += ATMEL_TOUCH_KEY_TOTAL_VAL;
   } /* End of the for loop */

   if((button_value != atmel_button_reported) 
       && (atmel_press_button_sent == 1)) 
   {
      /* Report key release */
      input_report_key(ts->input_dev, atmel_button_reported, ATMEL_KEY_RELEASE);
      atmel_press_button_sent = 0;
      atmel_button_reported = ATMEL_NO_BUTTON;
   }

   if (button_value == ATMEL_NO_BUTTON) 
   {
      atmel_process_button_lo = ATMEL_DO_NOT_PROCESS_LIFTOFF;
      return;
   }

   /* fire haptics and say OK to process liftoff */
   if (atmel_process_button_lo == ATMEL_DO_NOT_PROCESS_LIFTOFF) 
   {
      if (atmel_use_haptic_timer == ATMEL_USE_TIMER) 
      {
         /* Fire Haptic response and set OK to process button on liftoff */
         //VIB_fire_haptic_no_timer(ATMEL_HAPTIC_ON);
         /* Using a sleep here may cause the pulse to be longer based on
            system loading */
         atmel_sleep(atmel_button_haptic_time);
         //VIB_fire_haptic_no_timer(ATMEL_HAPTIC_OFF);

      } 
      else 
      {
         /* Fire the haptic pulse and let the vibrator driver handle
            the timer functionality */
         //VIB_fire_haptic(qtm_obp_button_haptic_time);
      }
      atmel_process_button_lo = ATMEL_PROCESS_LIFTOFF;
      atmel_button_reported = button_value;
      /* Timestamp the press of the button for future comparison */
      do_gettimeofday(&tv);
      atmel_press_timestamp_us = (int)tv.tv_usec;
      atmel_press_timestamp_s = (int)tv.tv_sec;
   }

   if((atmel_process_button_lo == ATMEL_PROCESS_LIFTOFF) 
       && (button_value ==  atmel_button_reported) 
       && ( atmel_press_button_sent == 0)) 
   {
      /* Get the current time */
      do_gettimeofday(&tv);
      current_time_delta_s = (int)tv.tv_sec - atmel_press_timestamp_s;
      current_time_delta_us = ((int)tv.tv_usec + (current_time_delta_s *  1000000))
                              - atmel_press_timestamp_us;

      if(current_time_delta_us >= atmel_long_press_detect_time) 
      {
         input_report_key(ts->input_dev, button_value, ATMEL_KEY_PRESS);
         atmel_press_button_sent = 1;
         atmel_press_timestamp_us = 0;
         atmel_press_timestamp_s = 0;
      }
   }

   if ((touch == ATMEL_KEY_RELEASE) 
       && (atmel_process_button_lo == ATMEL_PROCESS_LIFTOFF) 
       && (button_value != ATMEL_NO_BUTTON)) 
   {
      /* Press event not sent yet, report key press now */
      if(atmel_press_button_sent == 0) 
      {
         input_report_key(ts->input_dev, button_value, ATMEL_KEY_PRESS);
      }
      /* Report key release */
      input_report_key(ts->input_dev, button_value, ATMEL_KEY_RELEASE);
      atmel_process_button_lo = ATMEL_DO_NOT_PROCESS_LIFTOFF;
      atmel_press_button_sent = 0;
      atmel_button_reported = ATMEL_NO_BUTTON;
   }
}


/*!
 * @brief This is the work func which services the incoming work queue
 *
 * This function implements the work function queue handler.  This function
 * will retrieve the data from the IC, process the data and send it to the
 * framework
 *
 *
 * @param work     Work structure pointer
 *
 * @return None
 */
static void atmel_ts_work_func(struct work_struct *work)
{
   int version;
   int ret = 0;
   uint8_t buf[ATMEL_READ_DATA_SIZE];
   int x_pos = 0, y_pos = 0, z_pressure = 0, w_area = 0, finger_id =0, 
       press_release_flag = 0;
   int retval = 0;
   unsigned char value[ATMEL_REG_ADDR_SIZE];
   int current_touch = 0;
	
   unsigned char i,bit_mask;
   static unsigned short pressed_key_code=0x00;

   struct atmel_ts_data *ts = container_of(work, struct atmel_ts_data, work);

   atmel_printk("[JJ] Entering atmel_ts_work_func.\n");

   if(ts->mode == MOT_TOUCH_MODE_UNKNOWN)
   {
      clk_busy_wait(ATMEL_READ_WRITE_DELAY);
      memset(buf,0,sizeof(buf));
      ts->client->addr = i2c_address;
      if ((ret = atmel_i2c_read(ts->client, buf, ATMEL_READ_DATA_SIZE )) < 0 )
      {
         atmel_printk("atmel_ts_work_func: atmel_i2c_read (In Normal mode i2c address) error  %d\n",ret);
         ts->client->addr = bl_i2c_address;

         memset(buf,0,sizeof(buf)); 
         if ((ret = atmel_i2c_read(ts->client, buf, 1 )) < 0 )
         {
            atmel_printk("atmel_ts_work_func: atmel_i2c_read error (in Bootloader mode i2c address) %d\n",ret);
            ts->driver_state = MOT_TOUCH_BL_I2C_READ_ERROR;
            goto work_func_error;
         }
         else
         {        
            atmel_printk("atmel_ts_work_func: entering bootloader mode\n");
            /* wait until we get 0x8n ,already in waiting for command state */
            while ( (buf[0] & 0xF0) != 0x80 )
            {
               atmel_printk("Waiting for IC to send 0x80\n");
               if((buf[0] & 0xF0) == 0xC0)
               {
                  buf[0] = 0xDC;
                  buf[1] = 0xAA;
                  if((ret = atmel_i2c_write(ts->client,buf,2)) < 0)
                  {
                     ts->driver_state = MOT_TOUCH_BL_I2C_WRITE_ERROR;
                     goto work_func_error;
                  } 
                  ts->driver_state = MOT_TOUCH_BL_WAITING_FOR_STATUS;
               } 
               if((ret = atmel_i2c_read(ts->client,buf,1)) < 0)
               {
                  ts->driver_state = MOT_TOUCH_BL_I2C_READ_ERROR;
                  goto work_func_error;
               }
            }
            ts->mode = MOT_TOUCH_MODE_BOOTLOADER;
            ts->driver_state = MOT_TOUCH_BL_WAITING_FOR_DATA;
            ts->FwVersion = 0;
            goto work_func_exit;
         }
      }
      else
      {
         /*So Driver is in NORMAL mode */
         ts->mode = MOT_TOUCH_MODE_NORMAL; 
         ts->client->addr = i2c_address;
         goto normal_mode_no_read;
      }	
   }

   if ( ts->mode == MOT_TOUCH_MODE_BOOTLOADER )
   {
      /* Verifying Driver completed the firmware update successfully */
      ts->client->addr = bl_i2c_address;
      if((ret = atmel_i2c_read(ts->client,buf,ATMEL_READ_DATA_SIZE)) < 0 )
      {
         ts->client->addr = i2c_address;
         ts->mode = MOT_TOUCH_MODE_NORMAL;
         atmel_printk("Waiting before version read\n");
         version = atmel_ts_read_fw_version(ts);
         atmel_printk("atmel_ts_work_func: FW version is now %i \n", version);
	 if(version > 0 )
         {
            if(version > ts->FwVersion )
            {
               atmel_printk("atmel_ts_work_func: Firmware updated success fully \n");
            }
            else
            {
               atmel_printk("atmel_ts_work_func: Firmware updated success fully, But Not the latest\n");
            }
            ts->driver_state = MOT_TOUCH_BL_FW_UPDATE_SUCCESS;
            ts->FwVersion = version;
            ts->mode = MOT_TOUCH_MODE_UNKNOWN;
            atmel_reset(ts);
            goto work_func_exit;
         }
         else
         {
            atmel_printk("atmel_ts_work_function: atmel_ts_read_fw_version fuction returns error... \n");
            ts->driver_state = MOT_TOUCH_BL_FW_VERSION_READ_ERROR;
            goto work_func_error;
         }
      }
      else
      {
         /* Do the State management for the firmware update service */
         if ( (buf[ATMEL_FW_UPDATE_STATUS_BYTE] & 0xF0) == 0x80 )
         {
            /* 0x8n indicates that the IC is waiting for data */     
            ts->driver_state = MOT_TOUCH_BL_WAITING_FOR_DATA;
         }
         else if ( buf[ATMEL_FW_UPDATE_STATUS_BYTE] == 0x02 )
         {
            ts->driver_state  = MOT_TOUCH_BL_WAITING_FOR_CRC;
         }
         else if ( buf[ATMEL_FW_UPDATE_STATUS_BYTE] == 0x04 )
         {
            ts->driver_state = MOT_TOUCH_BL_AFTER_CRC_PASS;
         }
         else if ( buf[ATMEL_FW_UPDATE_STATUS_BYTE] == 0x03 )
         {
            /* We got bad CRC on the record */
            atmel_printk("atmel_ts_work_func: Driver is in MOT_TOUCH_BL_GOT_BAD_CRC state \n");
            ts->driver_state = MOT_TOUCH_BL_GOT_BAD_CRC;
         }
      }
      goto work_func_exit;
   }

   if ( ts->mode == MOT_TOUCH_MODE_NORMAL )
   {
      /* Read the data from the device (retrying if necessary) */
      ret = atmel_i2c_read(ts->client, buf, ATMEL_READ_DATA_SIZE);
      if (ret < 0) 
      {
         atmel_printk("%s: I2C failed to read data from the device\n",__func__);
         goto exit_func;
      }
      atmel_printk("atmel_message_T5: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n",
		     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
   }
normal_mode_no_read:
   /* Check for OBJ_ID_GEN_COMMANDPROCESSOR_T6 report IDs */
   if (buf[ATMEL_REPORT_ID_BYTE] >= 
        atmel_obj_table_elem[OBJ_ID_GEN_COMMANDPROCESSOR_T6].report_id_low
       && buf[ATMEL_REPORT_ID_BYTE] <=
        atmel_obj_table_elem[OBJ_ID_GEN_COMMANDPROCESSOR_T6].report_id_high) 
   {
      atmel_printk("atmel_command_T6\n");
      if ((buf[ATMEL_T6_STATUS] & ATMEL_T6_STATUS_RESET) ==
		ATMEL_T6_STATUS_RESET) 
      {
         atmel_printk("T6: Reset\n");

         /* Identify as being done with reset */
         reset = 0;
        if((buf[2] != 0x08) || (buf[3] != 0x1A) || (buf[4] != 0xF6))//if checksum changed, we need update configuration of firmware
       {
         /* Configure the IC for mode 7,8,9,16,17,20 */
         /* - touch screen on */
         atmel_mode(OBJ_ID_GEN_POWERCONFIG_T7, ts);
         atmel_mode(OBJ_ID_GEN_ACQUIRECONFIG_T8, ts);
         atmel_mode(OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9, ts);
         atmel_mode(OBJ_ID_TOUCH_KEYARRAY_T15, ts);
         atmel_mode(OBJ_ID_PROCI_GESTURESPROCESS_T18, ts);
         atmel_mode(OBJ_ID_TOUCH_PROXIMITY_T23, ts);
         atmel_mode(OBJ_ID_SPT_GPIOPWM_T19, ts);
         atmel_mode(OBJ_ID_SPT_SELFTEST_T25, ts);
         atmel_mode(OBJ_ID_SPT_CTECONFIG_T28, ts);
         atmel_mode(OBJ_ID_PROCI_GRIPFACESUPPRESSION_T20, ts);
         atmel_mode(OBJ_ID_PROCG_NOISESUPPRESSION_T22, ts);
         atmel_mode(OBJ_ID_PROCI_ONETOUCHGESTUREPROCESSOR_T24, ts);
         atmel_mode(OBJ_ID_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, ts);
         atmel_mode(OBJ_ID_SPT_USERDATA_T38, ts);

         /* Backup */
         value[0] = ATMEL_T6_BACKUP_CMD;
         retval = atmel_reg_write(ts->client,
                      atmel_obj_table_elem[OBJ_ID_GEN_COMMANDPROCESSOR_T6].start_pos
						   + ATMEL_T6_BACKUPNV,
                                    1, &value[0]);
         if (retval < 0)
         {
            atmel_printk("[LJ] atmel backup config failed!\n");
         }
        	}
         /* Force the IC to calibrate after the burst length is programmed */
         atmel_calibrate(ts);
      }

      /* Check for Calibration.  If we are calibrating, indicate in a message */
      if ((calibrating == 0)
          && (buf[ATMEL_T6_STATUS] & ATMEL_T6_STATUS_CAL)) 
      {
         calibrating = 1;
         atmel_printk("T6: The IC is calibrating.\n");
      } 
      else if ((calibrating != 0) 
               && ((buf[ATMEL_T6_STATUS] & ATMEL_T6_STATUS_CAL) == 0)) 
      {
         calibrating = 0;
         atmel_printk("T6: The IC is done calibrating.\n");
      }

      /* Check for a possible overflow from the HW.  If there has been an overflow, log a message. */
      if (buf[ATMEL_T6_STATUS] & ATMEL_T6_STATUS_OFL) 
      {
         atmel_printk("T6: An overflow has occurred.\n");
      }

      /* SIGERR: Error in acquisition */
      if (buf[ATMEL_T6_STATUS] & ATMEL_T6_STATUS_SIGERR) 
      {
         atmel_printk("T6: An SIGERR has occurred.\n");
      }

      /* CFGERR: Error with object setup */
      if (buf[ATMEL_T6_STATUS] & ATMEL_T6_STATUS_CFGERR) 
      {
         atmel_printk("T6: An CFGERR has occurred.\n");
      }
      /* Reset to the message pointer */
      atmel_set_msg_pointer(ts);

   }
   /* Check for OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9 report IDs */
   else if (buf[ATMEL_REPORT_ID_BYTE] >=
             atmel_obj_table_elem[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9].report_id_low
            && buf[ATMEL_REPORT_ID_BYTE] <=
             atmel_obj_table_elem[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9].report_id_high) 
   {
      atmel_printk("atmel_multitouch_T9\n");
      atmel_printk("[JJ] This is t9 obj report.\n");
      /* Align X and Y LSB and MSB bits for 10-bit PTR */
      x_pos = buf[ATMEL_T9_XPOS_MSB];
      x_pos = (x_pos << ATMEL_T9_XYPOS_LSB_POS_SHIFT) | 
               ((buf[ATMEL_T9_XYPOS_LSB] & ATMEL_T9_XYPOS_LSB_XPOS_MASK) >>
		     ATMEL_T9_XYPOS_LSB_POS_SHIFT);
      x_pos = x_pos >> 2;

      y_pos = buf[ATMEL_T9_YPOS_MSB];
      y_pos = (y_pos << ATMEL_T9_XYPOS_LSB_POS_SHIFT) |
               (buf[ATMEL_T9_XYPOS_LSB] & ATMEL_T9_XYPOS_LSB_YPOS_MASK);
      y_pos = y_pos >> 2;

      atmel_printk("[JJ] before processing x_pos=%d, y_pos=%d.\n", x_pos, y_pos);

      /*
       ** X and Y are flipped on the sensor panel so manipulate the data so
       ** X data is Y and Y data is reported as X.
       ** Swap the x y data due to sensor orientation
      */
      if (atmel_swap_x_y == ATMEL_SWAP_DATA) 
      {
         swap(x_pos, y_pos);
      }

      /* Reverse the x or y value based on sensor to screen layout */
      if (atmel_invert_x_data == ATMEL_INVERT_DATA) 
      {
         x_pos = (atmel_x_resolution - 1) - x_pos;
      }

      if (atmel_invert_y_data == ATMEL_INVERT_DATA) 
      {
         y_pos = (atmel_y_resolution - 1) - y_pos;
      }

      atmel_printk("[JJ] after processing x_pos=%d, y_pos=%d.\n", x_pos, y_pos);

      /* capture width/area */
      w_area = buf[ATMEL_T9_TCH_AREA];

      /* capture pressure/amplitude */
      z_pressure = buf[ATMEL_T9_TCH_AMPLI];

      current_touch = buf[ATMEL_REPORT_ID_BYTE];

      finger_id = (buf[ATMEL_REPORT_ID_BYTE] -
           atmel_obj_table_elem[OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9].report_id_low);

      if (((buf[ATMEL_T9_STATUS] & ATMEL_T9_STATUS_TOUCH_MASK) ==
		     ATMEL_T9_STATUS_TOUCH_MASK)
          || ((buf[ATMEL_T9_STATUS] & ATMEL_T9_STATUS_PRESS_MASK) ==
		     ATMEL_T9_STATUS_PRESS_MASK)
          || ((buf[ATMEL_T9_STATUS] & ATMEL_T9_STATUS_MOVE_MASK) == 
                     ATMEL_T9_STATUS_MOVE_MASK)) 
      {
         press_release_flag = ATMEL_KEY_PRESS;
         atmel_finger_press_release |= ( 1 << finger_id );
      }

      if ((buf[ATMEL_T9_STATUS] & ATMEL_T10_STATUS_RELEASE_MASK) ==
		    ATMEL_T10_STATUS_RELEASE_MASK) 
      {
         press_release_flag = ATMEL_KEY_RELEASE;
         atmel_finger_press_release &= ~( 1 << finger_id );
         atmel_printk("[JJ] key is released\n");
      }

      if (y_pos > atmel_y_resolution) 
      {
         /* if the user moved into the button area from the touch
            area then produce a lift off */
         if (touch_data.touch_area[finger_id] == ATMEL_IGNORE) 
         {
            /* Produce a lift off */
            atmel_finger_press_release &= (1 << finger_id);
            input_report_abs(ts->input_dev, ABS_MISC,
                             ((finger_id << 16) | atmel_finger_press_release ));
            input_report_abs(ts->input_dev, ABS_X, x_pos);
            input_report_abs(ts->input_dev, ABS_Y, y_pos);
            input_report_abs(ts->input_dev, ABS_PRESSURE, z_pressure);
            input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w_area);
            input_report_key(ts->input_dev, BTN_TOUCH, atmel_finger_press_release);
            input_sync(ts->input_dev);
            touch_data.touch_area[finger_id] = ATMEL_KEY_RELEASE;
         }
         atmel_process_button(touch_data, ts, x_pos, y_pos,
                                finger_id, press_release_flag);
      } 
      else 
      { 
         /* y < atmel_y_resolution */

         touch_data.ignore_button[finger_id] = ATMEL_IGNORE;
         touch_data.touch_area[finger_id] =  ATMEL_KEY_PRESS;
         if (touch_data.button_area[finger_id] == ATMEL_KEY_PRESS) 
         {
            touch_data.button_area[finger_id] = ATMEL_KEY_RELEASE;
            atmel_process_button_lo = ATMEL_DO_NOT_PROCESS_LIFTOFF;
         }

         input_report_abs(ts->input_dev, ABS_MISC,
                          ( (finger_id << 16) | atmel_finger_press_release ));
         input_report_abs(ts->input_dev, ABS_X, x_pos);
         input_report_abs(ts->input_dev, ABS_Y, y_pos);
         input_report_abs(ts->input_dev, ABS_PRESSURE, z_pressure);
         input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w_area);
         input_report_key(ts->input_dev, BTN_TOUCH, atmel_finger_press_release);

         atmel_printk("[LJ] atmel touch x_pos = %d\n", x_pos);
         atmel_printk("[LJ] atmel touch y_pos = %d\n", y_pos);
         atmel_printk("[LJ] atmel touch z_pressure = %d\n", z_pressure);
         atmel_printk("[LJ] atmel touch w_area = %d\n", w_area);

         input_sync(ts->input_dev);

      } /* fi else resolution check */

      /* There was a liftoff here so release the button and reset variables */
      if (press_release_flag == ATMEL_KEY_RELEASE) 
      {
         touch_data.ignore_button[finger_id] = ATMEL_ALLOW;
         touch_data.ignore_touch[finger_id] = ATMEL_ALLOW;
         touch_data.touch_area[finger_id] = ATMEL_KEY_RELEASE;
         touch_data.button_area[finger_id] = ATMEL_KEY_RELEASE;
         atmel_process_button_lo = ATMEL_DO_NOT_PROCESS_LIFTOFF;
      }
   }
   else if (buf[ATMEL_REPORT_ID_BYTE] >=
             atmel_obj_table_elem[OBJ_ID_TOUCH_KEYARRAY_T15].report_id_low
            && buf[ATMEL_REPORT_ID_BYTE] <=
             atmel_obj_table_elem[OBJ_ID_TOUCH_KEYARRAY_T15].report_id_high) 
   {
      atmel_printk("[JJ] key array t15 obj recieved\n");

      if(buf[ATMEL_T15_STATUS] & ATMEL_T15_KEY_DETECT_MASK)
      {
         /* MENU key */
         if ( ATMEL_KEY_DETECT(buf, TOUCH_KEY_NUMBER_MENU) )
         {
            pressed_key_code = TOUCH_KEYCODE_MENU;
         }
         /* BACK key */
         else if ( ATMEL_KEY_DETECT(buf, TOUCH_KEY_NUMBER_BACK) )
         {
            pressed_key_code = TOUCH_KEYCODE_BACK;
         }
         /* SEARCH key */
         else if ( ATMEL_KEY_DETECT(buf, TOUCH_KEY_NUMBER_SEARCH) )
         {
            pressed_key_code = TOUCH_KEYCODE_SEARCH;
         }
         atmel_printk("[JJ] key pressed - %d\n", pressed_key_code);
         input_report_key(ts->input_dev, pressed_key_code, ATMEL_KEY_PRESS);

      }
      else
      {
         if(pressed_key_code)
         {
            atmel_printk("[JJ] key released - %d\n", pressed_key_code);
            input_report_key(ts->input_dev, pressed_key_code, ATMEL_KEY_RELEASE);
            pressed_key_code = 0x00;
         }
      }
      input_sync(ts->input_dev);
   }
   else 
   { 
      /* Check for OBJ_ID_TOUCH_MULTITOUCHSCREEN_T9 report IDs */
      atmel_printk("Message is default.Data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n",
		     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
   }

exit_func:
work_func_exit:
work_func_error:
   enable_irq(ts->client->irq);
}


/*!
 * @brief This is the func that handles the IRQ
 *
 * This function handles the IRQ and schedules a work queue item.
 *
 *
 * @param irq     IRQ number
 * @param dev_id  Device ID of the incoming IRQ
 *
 * @return irqreturn_t - Return of IRQ status
 *
 * IRQ_NONE means we didn't handle it.
 * IRQ_HANDLED means that we did have a valid interrupt and handled it.
 * IRQ_RETVAL(x) selects on the two depending on x being non-zero (for handled)
 */
static irqreturn_t atmel_ts_irq_handler(int irq, void *dev_id)
{
   struct atmel_ts_data *ts = dev_id;
   atmel_printk("[JJ] Entering atmel_ts_irq_handler\n");
   disable_irq(ts->client->irq);
   schedule_work(&ts->work);
   return IRQ_HANDLED;
}


int atmel_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
   int ret = 0;
   int i = 0;
   int fuzz_x, fuzz_y;
   int inactive_area_left;
   int inactive_area_right;
   int inactive_area_top;
   int inactive_area_bottom;
	
   atmel_printk("atmel_mxt224 probe, hoho!\n");

   if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
   {
      atmel_printk("atmel_ts_probe: need I2C_FUNC_I2C\n");
      ret = -ENODEV;
      goto err_check_functionality_failed;
   }

   ts_data = kzalloc(sizeof(struct atmel_ts_data), GFP_KERNEL);
   if (ts_data == NULL) 
   {
      ret = -ENOMEM;
      goto exit_kzalloc;
   }


   INIT_WORK(&ts_data->work, atmel_ts_work_func);
   ts_data->client = client;
   i2c_set_clientdata(client, ts_data);

   if (ts_data->power) 
   {
      ret = ts_data->power(1);
      if (ret < 0) 
      {
         atmel_printk("%s power on failed\n",__func__);
         goto exit_kfree;
      }
   }

   ts_data->max[1] = atmel_y_resolution;

   ts_data->input_dev = input_allocate_device();
   if (ts_data->input_dev == NULL) 
   {
      ret = -ENOMEM;

      atmel_printk("%s: Failed to allocate input device\n",__func__);
      goto exit_kfree;
   }

   ts_data->input_dev->name = "atmel-touchscreen";
   set_bit(EV_SYN, ts_data->input_dev->evbit);
   set_bit(EV_ABS, ts_data->input_dev->evbit);
   set_bit(EV_KEY, ts_data->input_dev->evbit);
   set_bit(BTN_TOUCH, ts_data->input_dev->keybit);
   set_bit(BTN_2, ts_data->input_dev->keybit);

   /* Indicate which buttons are supported */
   set_bit(TOUCH_KEYCODE_MENU, ts_data->input_dev->keybit);
   set_bit(TOUCH_KEYCODE_BACK, ts_data->input_dev->keybit);
   set_bit(TOUCH_KEYCODE_SEARCH, ts_data->input_dev->keybit);

			     
   inactive_area_left = 
       atmel_inactive_area_left * atmel_x_resolution / 0x10000;      
   inactive_area_right = 
       atmel_inactive_area_right * atmel_x_resolution / 0x10000;      
   inactive_area_top = 
       atmel_inactive_area_top * atmel_y_resolution / 0x10000;      
   inactive_area_bottom = 
       atmel_inactive_area_bottom * atmel_y_resolution / 0x10000;

   fuzz_x = atmel_x_movement_threshold * atmel_x_resolution / 0x10000;	    
   fuzz_y = atmel_y_movement_threshold * atmel_y_resolution / 0x10000;

   ts_data->snap_down[!!(ts_data->flags & ATMEL_SWAP_XY)] =
	    -inactive_area_left;
   ts_data->snap_up[!!(ts_data->flags & ATMEL_SWAP_XY)] =
	    atmel_x_resolution + inactive_area_right;
   ts_data->snap_down[!(ts_data->flags & ATMEL_SWAP_XY)] =
	    -inactive_area_top;
   ts_data->snap_up[!(ts_data->flags & ATMEL_SWAP_XY)] =
	    atmel_y_resolution + inactive_area_bottom;

   inactive_area_left = 0;
   inactive_area_top = 0;
   inactive_area_right = 0;
   inactive_area_bottom = 0;

   input_set_abs_params(ts_data->input_dev, ABS_MISC, 0, 0x000FFFFF, 0, 0);
   input_set_abs_params(ts_data->input_dev, ABS_X, 0, atmel_x_resolution,
			     atmel_x_movement_threshold, 0);
   input_set_abs_params(ts_data->input_dev, ABS_Y, 0, atmel_y_resolution,
			     atmel_y_movement_threshold, 0);

   input_set_abs_params(ts_data->input_dev, ABS_PRESSURE, 0, atmel_z_resolution,
			     atmel_z_movement_threshold, 0);
   input_set_abs_params(ts_data->input_dev, ABS_TOOL_WIDTH, 0, atmel_a_resolution,
			     atmel_a_movement_threshold, 0);
	
   atmel_printk("%s: atmel_x_resolution = %d\n", __func__, atmel_x_resolution);
   atmel_printk("%s: atmel_y_resolution = %d\n", __func__, atmel_y_resolution);

   ret = input_register_device(ts_data->input_dev);
   if (ret) 
   {
      atmel_printk("%s: Unable to register %s input device\n",
			              __func__, ts_data->input_dev->name);

      goto err_input_register_device_failed;
   }

   atmel_reset(ts_data);

   ret = atmel_get_object_info(ts_data->client);
   if (ret < 0) 
   {
      atmel_printk("%s:Cannot get object information return is %i\n", __func__,ret);
   }

   /* Reset to the message pointer */
   atmel_set_msg_pointer(ts_data);

   /* Request IRQ name. */
   ret = gpio_request(TOUCH_INT_N, "atmel_int_gpio");

   if (ret < 0) 
   {
      atmel_printk(" gpio_request failed for input %d, ret %d\n",
			     TOUCH_INT_N, ret);
      goto err_input_register_device_failed;
   }
   atmel_printk(" gpio_request for input %d\n", TOUCH_INT_N);

   /* Set the direction of the Touch Interrupt line. */
   ret = gpio_direction_input(TOUCH_INT_N);
   if (ret < 0) 
   {
      atmel_printk(" gpio_direction_input failed for input %d\n",
			     TOUCH_INT_N);
      goto err_input_register_device_failed;
   }

   client->irq = gpio_to_irq(TOUCH_INT_N);
   /* set irq type Enable Falling Edge */
   set_irq_type(client->irq, IRQ_TYPE_EDGE_FALLING);

   if (client->irq) 
   {
      /* request irq */
      ret =  request_irq(client->irq, (void *) atmel_ts_irq_handler, IRQ_DISABLED, 
                            ATMEL_I2C_NAME, ts_data);
      if (ret == 0)
      {
         ts_data->use_irq = 1;
         //ret = set_irq_wake(client->irq, 1);
      }
      else
      {
         ts_data->use_irq = 0;
         dev_err(&client->dev, "request_irq failed\n");
         free_irq(client->irq, ts_data);
      }
   }

#ifdef CONFIG_HAS_EARLYSUSPEND
   ts_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
   ts_data->early_suspend.suspend = atmel_ts_early_suspend;
   ts_data->early_suspend.resume = atmel_ts_late_resume;
   register_early_suspend(&ts_data->early_suspend);
#endif

   /*initializing Firmware update status as not done yet */
   ts_data->FwUpdate_Status = ATMEL_FW_UPDATE_NOT_DONE;

   /*Initilizing Driver mode as UNKNOWN */
   ts_data->mode = MOT_TOUCH_MODE_UNKNOWN;

   if (misc_register(&atmel_pf_driver) != 0 )
   {
      atmel_printk("atmel_ts_probe: [os] misc_register failed: \n");
      goto err_misc_register;
   }

   atomic_set(&atmel_num_processes, 1);

   atmel_reset(ts_data);

	
   return 0;

err_misc_register:
   input_unregister_device(ts_data->input_dev);
err_input_register_device_failed:
   input_free_device(ts_data->input_dev);
exit_kfree:
   kfree(ts_data);
exit_kzalloc:
err_check_functionality_failed:
   i2c_detach_client(client);
   atmel_printk("atmel_ts_probe: Exit after error\n");
   return ret;
	
}


/*!
 * @brief This is the function that remove the driver from the I2C client
 *
 * This function unregisters the device to the input queue
 *
 *
 * @param clent     I2C work client structure
 *
 * @return int - always returns 0
 */
static int atmel_ts_remove(struct i2c_client *client)
{
   struct atmel_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
   unregister_early_suspend(&ts->early_suspend);
#endif
   free_irq(client->irq, ts);

   input_unregister_device(ts->input_dev);
   kfree(ts);

   /* Free the GPIOs */
   gpio_free(TOUCH_INT_N);
   gpio_free(TOUCH_RST_N);

   return 0;
}


/*!
 * @brief This is the function called during system suspend
 *
 * This function implements the suspend functionality for the Qtm_Obp
 * driver.
 *
 *
 * @param client    I2C client pointer
 * @param mesg      Power management message
 *
 * @return int      Always returns 0
 */
static int atmel_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
   struct atmel_ts_data *ts = i2c_get_clientdata(client);

   atmel_printk("atmel_ts_suspend: Entering\n");

   /* Place the device into a standby state */
   atmel_mode(-OBJ_ID_GEN_POWERCONFIG_T7, ts);
   if(ts_data->use_irq)
       disable_irq(ts->client->irq);
   atmel_printk("%s: Exiting\n",__func__);

   return 0;
}


/*!
 * @brief This is the work func which services the resume command
 *
 * This function implements the system resume functionality.
 *
 *
 * @param client     I2C work client structure
 *
 * @return int       Always returns 0
 */
static int atmel_ts_resume(struct i2c_client *client)
{
   struct atmel_ts_data *ts = i2c_get_clientdata(client);
   uint8_t buf[ATMEL_READ_DATA_SIZE];
   int ret = 0;
   /* This is a work around for Power management on the TI OMAP
      since it does not properly recover the GPIO through core retention */
   if (gpio_get_value(TOUCH_RST_N) == 0) 
   {
      atmel_printk("%s:Reset line is low resetting the IC",__func__);
      /* Reset the IC reset line back to high to pull the IC out of reset */
      atmel_reset(ts);
      return 0;
   }
   if (gpio_get_value(TOUCH_INT_N) == 0) 
   {
      atmel_printk("%s: Interrupt line is low\n",__func__);

      atmel_set_msg_pointer(ts);
      /* Loop until the interrupt line goes high and discard any
         data from the IC */
      do 
      {
         /* Read the data from the device */
         ret = atmel_i2c_read(client, buf, ATMEL_READ_DATA_SIZE);
         if (ret < 0) 
         {
            atmel_printk("%s: I2C failed to read data from the device\n", __func__);
            atmel_reset(ts);
            return 0;
         }

      } while (gpio_get_value(TOUCH_INT_N) == 0);
   }

   /* Place the device into a normal state if both Reset and interrupt
      lines where in the proper state */
   atmel_mode(OBJ_ID_GEN_POWERCONFIG_T7, ts);
   /* re-calibration after resuming */
   atmel_calibrate(ts);
   /* Reset to the message pointer */
   atmel_set_msg_pointer(ts);
   if(ts_data->use_irq)
       enable_irq(ts->client->irq);
   atmel_printk("%s: Exiting()\n",__func__);
   return 0;
}

/*!
 * @brief This is the work func which services the incoming work queue
 *
 * This function implements a write to the IC to configure the sensor
 * register to intialize the sensor to a known state
 *
 *
 * @param work     Work structure pointer
 *
 * @return None
 */
static void atmel_ts_early_suspend(struct early_suspend *h)
{
   struct atmel_ts_data *ts;

   atmel_printk("atmel_ts_early_suspend: Entering\n");

   ts = container_of(h, struct atmel_ts_data, early_suspend);
   atmel_ts_suspend(ts->client, PMSG_SUSPEND);
   
   atmel_printk("%s: Exiting\n",__func__);
}


/*!
 * @brief This is the work func which services the incoming work queue
 *
 * This function implements a write to the IC to configure the sensor
 * register to intialize the sensor to a known state
 *
 *
 * @param work     Work structure pointer
 *
 * @return None
 */
static void atmel_ts_late_resume(struct early_suspend *h)
{
   struct atmel_ts_data *ts;

   atmel_printk("%s: Entering\n",__func__);

   ts = container_of(h, struct atmel_ts_data, early_suspend);
   atmel_ts_resume(ts->client);

   atmel_printk("%s: Exiting\n",__func__);

}

#ifdef CONFIG_MACH_SOCIAL
//////////////////////////////////////////////////////////////////////
static char proc_buf[1];
// write '1' to turn on irq, '0' to turn it off
static struct proc_dir_entry *proc_entry;
extern int adp5588_enable(uint8_t value);
extern int slide_home_enable(uint8_t value);
int power_key_enable( uint8_t value);

int touch_enable_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
  struct vreg *vreg;
  int rc;

  if(copy_from_user(proc_buf, buffer, 1)) {return(-EFAULT);}
  
  if('1' == *proc_buf)
  {  
    if( (ts_data != NULL) &&  ts_data->use_irq)
	    enable_irq(ts_data->client->irq);  
  }else
  if('0' == *proc_buf)
  {  
    if( (ts_data != NULL) &&	ts_data->use_irq)
	   disable_irq(ts_data->client->irq);	
  }
  return 1;
}
int keypad_enable_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
  struct vreg *vreg;
  int rc;

  if(copy_from_user(proc_buf, buffer, 1)) {return(-EFAULT);}
  
  if('1' == *proc_buf)
  {
    adp5588_enable(1);
	slide_home_enable(1);      
	power_key_enable(1);
  }else
  if('0' == *proc_buf)
  {      
	adp5588_enable(0);
	slide_home_enable(0);
	power_key_enable(0);
  }
  return 1;
}
#endif /*CONFIG_MACH_SOCIAL*/
static int __devinit atmel_ts_init(void)
{
#ifdef CONFIG_MACH_SOCIAL
    proc_entry = create_proc_entry("touch_enable", 0660, NULL);
    if(!proc_entry)
      {
       printk(KERN_ERR "Registration of proc \"touch_enable\" file failed\n");
       return(-ENOMEM);
      }
    
    proc_entry->read_proc  =0;
    proc_entry->write_proc = touch_enable_write_proc;    
    printk(KERN_INFO "ATMEL: /proc/touch_enable created\n");
	
    proc_entry = create_proc_entry("keypad_enable", 0660, NULL);
    if(!proc_entry)
      {
       printk(KERN_ERR "Registration of proc \"keypad_enable\" file failed\n");
       return(-ENOMEM);
      }
    
    proc_entry->read_proc  =0;
    proc_entry->write_proc = keypad_enable_write_proc;    
    printk(KERN_INFO "/proc/keypad_enable created\n");
#endif /*CONFIG_MACH_SOCIAL*/

   if (atmel_init_hwcfg() != 0) 
   {
      atmel_printk("atmel_ts_init: Initializing HWCFG failed\n");
      return -ENODEV;
   }

   MXT224_DEBUG_ON = FALSE;
   
   return i2c_add_driver(&atmel_ts_driver);
}

static void __exit atmel_ts_exit(void)
{
   atmel_free_hwcfg();
   i2c_del_driver(&atmel_ts_driver);
}

/*!
 * @brief Logs data
 *
 * This function is called as a replacement to printk
 *
 * @param fmt Text of message to log
 */

static void atmel_printk (char *fmt, ...)
{
   if (MXT224_DEBUG_ON == TRUE)
   {
      static va_list args;
      va_start(args, fmt);
      vprintk(fmt, args);
      va_end(args);
   }
}

late_initcall(atmel_ts_init);
module_exit(atmel_ts_exit);

MODULE_DESCRIPTION("atmel Touchpad Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("motorola");
MODULE_VERSION("1.0");
