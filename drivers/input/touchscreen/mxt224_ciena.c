/*
 * drivers/input/touchscreen/qtouch_obp_ts.c - driver for Quantum touch IC
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 Motorola, Inc.
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
 * Derived from the Motorola OBP touch driver.
 *
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>

#include "mxt224_ciena.h"
#include <mach/vreg.h>

#define IGNORE_CHECKSUM_MISMATCH

/* KeyCode for Android buttons */
#define TOUCH_KEYCODE_MENU      139
#define TOUCH_KEYCODE_HOME      102
#define TOUCH_KEYCODE_BACK      158
#define TOUCH_KEYCODE_SEARCH    217

/* touch IC VADD - VREG_GP5 on PM7540 */
#define VREG_TOUCH_VADD         "gp5"

#define _BITMAP_LEN              BITS_TO_LONGS(QTM_OBP_MAX_OBJECT_NUM)

/* For Virtual Keys */
#define QTM_OBP_NO_BUTTON 0

struct qtouch_ts_data
{
   struct i2c_client               *client;
   struct input_dev                *input_dev;
   struct work_struct              init_work;
   struct work_struct              work;
   struct qtouch_ts_platform_data  *pdata;
   struct coordinate_map           finger_data[_NUM_FINGERS];
   struct early_suspend            early_suspend;

   struct qtm_object               obj_tbl[QTM_OBP_MAX_OBJECT_NUM];
   unsigned long                   obj_map[_BITMAP_LEN];

   /* for virtual key */
   int                             last_report_key;
   /* for key array */
   uint32_t                        last_keystate;

   /* Checksum for configurations */
   uint8_t                         eeprom_checksum[3];
   uint8_t                         checksum_cnt;

   /* Note: The message buffer is reused for reading different messages.
    * MUST enforce that there is no concurrent access to msg_buf. */
   uint8_t                         *msg_buf;
   int                             msg_size;
   char                            mode;
   MOT_TOUCH_BL_MODE_STATE_T       driver_state;
   enum QTOUCH_FW_UPDATE_STATUS_T  FwUpdate_Status;
   int                             FwVersion;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qtouch_ts_early_suspend(struct early_suspend *handler);
static void qtouch_ts_late_resume(struct early_suspend *handler);
#endif


/* fops control API's */
static int  qtouch_fops_open(struct inode *, struct file *);
static int  qtouch_fops_ioctl(struct inode *, struct file *,
                              unsigned int, unsigned long);
static int  qtouch_fops_release(struct inode *, struct file *);
static int  qtouch_fops_write(struct file *, const char __user *,
                              size_t, loff_t *);


static int  qtouch_write(struct qtouch_ts_data *ts, void *buf, int buf_sz);
static int  qtouch_set_addr(struct qtouch_ts_data *ts, uint16_t addr);
static int  qtouch_read(struct qtouch_ts_data *ts, void *buf, int buf_sz);
static int  qtouch_read_addr(struct qtouch_ts_data *ts, uint16_t addr,
                             void *buf, int buf_sz);
static struct qtm_obj_message *qtouch_read_msg(struct qtouch_ts_data *ts);
static int  qtouch_write_addr(struct qtouch_ts_data *ts, uint16_t addr,
                              void *buf, int buf_sz);
static inline struct qtm_object *find_obj(struct qtouch_ts_data *ts, int id);
static int  qtouch_gpio_reset(void);
static void qtouch_force_reset(struct qtouch_ts_data *ts, uint8_t sw_reset);
static int  qtouch_process_info_block(struct qtouch_ts_data *ts);
static int  qtouch_power_config(struct qtouch_ts_data *ts, int on);

static void ts_printk(char *fmt, ...);

/* work queue for interrupt handler */
static struct workqueue_struct *qtouch_ts_wq;

/*! @brief tracking the number of process called from upper layer  */
static atomic_t qtouch_num_processes;

/*! @brief poniter to the private structure */
static struct qtouch_ts_data *qtouch_ts_ptr;

/* tracking if the log can be printed */
uint8_t MXT224_DEBUG_ON = FALSE;

/* checksum for configuration */
uint8_t config_checksum[3];

/* VREG_TOUCH on PM7540 - VADD for touch panel IC */
struct vreg *vreg_touch;

/* Added for test command - read position */
struct coordinate_map  tcmd_touch_data;

/* starting X position for the mouse */
static unsigned int x_start;
/* starting Y position for the mouse */
static unsigned int y_start;

/* I2C addresses */
static uint32_t app_i2c_address = MXT224_I2C_ADDR;
static uint32_t bl_i2c_address = MXT224_BL_I2C_ADDR;

static struct vkey mxt224_touch_vkeys[] =
{
   {
      .code     = TOUCH_KEYCODE_MENU,
      .center_x	= 25,
      .center_y	= 345,
      .width    = 30,
      .height   = 30,
   },
   {
      .code     = TOUCH_KEYCODE_HOME,
      .center_x	= 90,
      .center_y	= 345,
      .width    = 30,
      .height   = 30,
   },
   {
      .code     = TOUCH_KEYCODE_BACK,
      .center_x	= 160,
      .center_y	= 345,
      .width    = 30,
      .height   = 30,
   },
   {
      .code     = TOUCH_KEYCODE_SEARCH,
      .center_x	= 220,
      .center_y	= 345,
      .width    = 30,
      .height   = 30,
   },
};

/* Key Arrays */
static struct qtm_touch_keyarray_cfg mxt224_key_array_data[] =
{
   {
      .ctrl        = 0x00,
      .x_origin    = 0x00,
      .y_origin    = 0x00,
      .x_size      = 0x01,
      .y_size      = 0x01,
      .aks_cfg     = 0x00,
      .burst_len   = 0x41,
      .tch_det_thr = 0x1E,
      .tch_det_int = 0x02,
      .rsvd1       = 0x00,
      .rsvd2       = 0x00,
   },
};

/* platform data */
static struct qtouch_ts_platform_data mxt224_ts_platform_data =
{
   .flags               = (QTOUCH_USE_MULTITOUCH |
                           QTOUCH_CFG_BACKUPNV),
   .irqflags            = (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW),
   .abs_min_x           = 0,
   .abs_max_x           = 240,
   .abs_min_y           = 0,
   .abs_max_y           = 347,
   .abs_min_p           = 0,
   .abs_max_p           = 255,
   .abs_min_w           = 0,
   .abs_max_w           = 15,
   .nv_checksum         = {0x88, 0xB5, 0xAE},
   .fuzz_x              = 0,
   .fuzz_y              = 0,
   .fuzz_p              = 2,
   .fuzz_w              = 2,
   .hw_reset            = qtouch_gpio_reset,
   /* object T15 */
   .key_array =
   {
      .cfg              = mxt224_key_array_data,
      .keys             = NULL,
      .num_keys	        = 0,
   },
   /* object T7 */
   .power_cfg =
   {
      .idle_acq_int     = 0xFF,
      .active_acq_int   = 0xFF,
      .active_idle_to   = 0x32,
   },
   /* object T8 */
   .acquire_cfg =
   {
      .charge_time      = 0x08,
      .atouch_drift     = 0x05,
      .touch_drift      = 0x14,
      .drift_susp       = 0x14,
      .touch_autocal    = 0x00,
      .sync             = 0x00,
      .atch_cal_suspend_time    = 0x0A,
      .atch_cal_suspend_thres   = 0x0F,
   },
   /* object T9 */
   .multi_touch_cfg =
   {
      .ctrl             = 0x83,
      .x_origin         = 0x00,
      .y_origin         = 0x00,
      .x_size           = 0x0D,
      .y_size           = 0x0A,
      .aks_cfg          = 0x00,
      .burst_len        = 0x11,
      .tch_det_thr      = 0x28,
      .tch_det_int      = 0x02,
      .orient           = 0x01,
      .mrg_to           = 0x00,
      .mov_hyst_init    = 0x01,
      .mov_hyst_next    = 0x01,
      .mov_filter       = 0x00,
      .num_touch        = 0x0A,
      .merge_hyst       = 0x0A,
      .merge_thresh     = 0x0A,
      .amp_hyst         = 0x0A,
      .x_res            = 0x015A,
      .y_res            = 0x00EF,
      .x_low_clip       = 0x00,
      .x_high_clip      = 0x00,
      .y_low_clip       = 0x00,
      .y_high_clip      = 0x00,
      .x_edge_ctrl      = 0x00,
      .x_edge_dist      = 0x00,
      .y_edge_ctrl      = 0x00,
      .y_edge_dist      = 0x00,
   },
   .multi_touch_cfg2 =
   {
      .ctrl             = 0x83,
      .x_origin         = 0x0D,
      .y_origin         = 0x0A,
      .x_size           = 0x03,
      .y_size           = 0x03,
      .aks_cfg          = 0x00,
      .burst_len        = 0x31,
      .tch_det_thr      = 0x14,
      .tch_det_int      = 0x02,
      .orient           = 0x03,
      .mrg_to           = 0x00,
      .mov_hyst_init    = 0x01,
      .mov_hyst_next    = 0x01,
      .mov_filter       = 0x00,
      .num_touch        = 0x0A,
      .merge_hyst       = 0x0A,
      .merge_thresh     = 0x0A,
      .amp_hyst         = 0x0A,
      .x_res            = 0x0080,
      .y_res            = 0x0080,
      .x_low_clip       = 0x00,
      .x_high_clip      = 0x00,
      .y_low_clip       = 0x00,
      .y_high_clip      = 0x00,
      .x_edge_ctrl      = 0x00,
      .x_edge_dist      = 0x00,
      .y_edge_ctrl      = 0x00,
      .y_edge_dist      = 0x00,
   },
   /* object T18 */
   .commsconfig_cfg =
   {
      .ctrl             = 0x00,
      .command          = 0x00,
   },
   /* object T19 */
   .gpio_pwm_cfg =
   {
      .ctrl             = 0x00,
      .report_mask      = 0x00,
      .pin_direction    = 0x00,
      .internal_pullup  = 0x00,
      .output_value     = 0x00,
      .wake_on_change   = 0x00,
      .pwm_enable       = 0x00,
      .pwm_period       = 0x00,
      .duty_cycle_0     = 0x00,
      .duty_cycle_1     = 0x00,
      .duty_cycle_2     = 0x00,
      .duty_cycle_3     = 0x00,
      .trigger_0        = 0x00,
      .trigger_1        = 0x00,
      .trigger_2        = 0x00,
      .trigger_3        = 0x00,
   },
   /* object T20 */
   .grip_suppression_cfg =
   {
      .ctrl             = 0x00,
      .xlogrip          = 0x64,
      .xhigrip          = 0x64,
      .ylogrip          = 0x64,
      .yhigrip          = 0x64,
      .maxtchs          = 0x00,
      .reserve0         = 0x00,
      .szthr1           = 0x00,
      .szthr2           = 0x00,
      .shpthr1          = 0x00,
      .shpthr2          = 0x00,
      .supextto         = 0x00,
   },
   /* object T22 */
   .noise_suppression_cfg =
   {
      .ctrl             = 0x15,
      .outlier_filter_len = 0x00,
      .reserve0         = 0x00,
      .gcaf_upper_limit	= 0x0019,
      .gcaf_lower_limit	= 0xFFE7,
      .gcaf_low_count   = 0x04,
      .noise_threshold  = 0x32,
      .reserve1         = 0x00,
      .freq_hop_scale   = 0x01,
      .burst_freq_0     = 0x0A,
      .burst_freq_1     = 0x0F,
      .burst_freq_2     = 0x14,
      .burst_freq_3     = 0x19,
      .burst_freq_4     = 0x1E,
      .idle_gcaf_valid  = 0x04
   },
   .noise_suppression_cfg2 =
   {
      .ctrl             = 0x15,
      .outlier_filter_len = 0x00,
      .reserve0         = 0x00,
      .gcaf_upper_limit	= 0x0019,
      .gcaf_lower_limit	= 0xFFE7,
      .gcaf_low_count   = 0x04,
      .noise_threshold  = 0x32,
      .reserve1         = 0x00,
      .freq_hop_scale   = 0x01,
      .burst_freq_0     = 0x0A,
      .burst_freq_1     = 0x0F,
      .burst_freq_2     = 0x14,
      .burst_freq_3     = 0x19,
      .burst_freq_4     = 0x1E,
      .idle_gcaf_valid  = 0x04
   },
   /* object T23 */
   .touch_proximity_cfg =
   {
      .ctrl             = 0x00,
      .x_origin         = 0x00,
      .y_origin         = 0x00,
      .x_size           = 0x00,
      .y_size           = 0x00,
      .reserve0         = 0x00,
      .blen             = 0x00,
      .tch_thresh       = 0x0000,
      .tch_detect_int   = 0x00,
      .average          = 0x00,
      .rate             = 0x0000,
   },
   /* object T24 */
   .one_touch_gesture_proc_cfg =
   {
      .ctrl             = 0x03,
      .max_num          = 0x04,
      .gesture_enable   = 0x03FF,
      .pres_proc        = 0x00,
      .tap_time_out     = 0x64,
      .flick_time_out   = 0x64,
      .drag_time_out    = 0x01,
      .short_press_time_out  = 0x0A,
      .long_press_time_out   = 0x14,
      .repeat_press_time_out = 0x28,
      .flick_threshold  = 0x004B,
      .drag_threshold   = 0x0002,
      .tap_threshold    = 0x0064,
      .throw_threshold  = 0x0019,
   },
   /* object T25 */
   .self_test_cfg =
   {
      .ctrl                = 0x00,
      .command             = 0x00,
      .high_signal_limit_0 = 0x2EE0,
      .low_signal_limit_0  = 0x1B58,
      .high_signal_limit_1 = 0x36B0,
      .low_signal_limit_1  = 0x01F4,
      .high_signal_limit_2 = 0x0000,
      .low_signal_limit_2  = 0x0000,
   },
   /* object T27 */
   .two_touch_gesture_proc_cfg =
   {
      .ctrl                = 0x03,
      .max_num             = 0x02,
      .reserved1           = 0x00,
      .gesture_enable      = 0xE0,
      .rotate_threshold    = 0x03,
      .zoom_threshold      = 0x0023,
   },
   /* object T28 */
   .cte_config_cfg =
   {
      .ctrl                = 0x00,
      .command             = 0x00,
      .mode                = 0x00,
      .idle_gcaf_depth     = 0x04,
      .active_gcaf_depth   = 0x08,
      .voltage             = 0xF6,
   },
   /* object T38 */
   .userdata_cfg =
   {
      .data_0              = 0x00,
      .data_1              = 0x00,
      .data_2              = 0x00,
      .data_3              = 0x00,
      .data_4              = 0x00,
      .data_5              = 0x00,
      .data_6              = 0x00,
      .data_7              = 0x00,
   },
   /* virtual keys */
   .vkeys =
   {
      .count = ARRAY_SIZE(mxt224_touch_vkeys),
      .keys  = mxt224_touch_vkeys,
   },
};

const struct file_operations qtouch_fops =
{
   .owner   = THIS_MODULE,
   .open    = qtouch_fops_open,
   .ioctl   = qtouch_fops_ioctl,
   .release = qtouch_fops_release,
   .write   = qtouch_fops_write,
};

static struct miscdevice qtouch_pf_driver =
{
   .minor = MISC_DYNAMIC_MINOR,
   .name  = QTOUCH_I2C_NAME,
   .fops  = &qtouch_fops,
};


static uint32_t qtouch_tsdebug;
module_param_named(tsdebug, qtouch_tsdebug, uint, 0664);

static irqreturn_t qtouch_ts_irq_handler(int irq, void *dev_id)
{
   struct qtouch_ts_data *ts = dev_id;

   ts_printk("[LJ] Entering qtouch_ts_irq_handler()-irq=%d\n", irq);

   disable_irq_nosync(ts->client->irq);
   queue_work(qtouch_ts_wq, &ts->work);
   ts_printk("[LJ] Exiting qtouch_ts_irq_handler()\n");
   return IRQ_HANDLED;
}


static int qtouch_fops_open(struct inode *inode, struct file *filp)
{
   ts_printk("[LJ] Entering qtouch_fops_open()\n");
   atomic_dec(&qtouch_num_processes);
   if (atomic_read(&qtouch_num_processes) < 0)
   {
      pr_info("[LJ] qtouch_fops_open() - error\n");
      return -1;
   }
   ts_printk("[LJ] Exiting qtouch_fops_open()\n");
   return 0;
}


static int qtouch_fops_release(struct inode *inode, struct file *filp)
{
   ts_printk("[LJ] Entering qtouch_fops_release()\n");
   atomic_inc(&qtouch_num_processes);
   ts_printk("[LJ] Exiting qtouch_fops_release()\n");
   return 0;
}


static int qtouch_fops_write(struct file *flip, const char __user *buf,
                             size_t count, loff_t *f_pos)
{
   int retVal = -1;
   size_t segment = 16;
   unsigned char *write_addr = NULL;
   unsigned char kernelBuffer[400];
   struct qtouch_ts_data *ts = qtouch_ts_ptr;

   ts_printk("[LJ] Entering qtouch_fops_write()\n");
   if (ts == NULL)
   {
      pr_info("[LJ] qtouch_fops_write(): Invalid pointer\n");
      return retVal;
   }
   if (buf == NULL)
   {
      pr_info("[LJ] qtouch_fops_write(): Invalid User buffer\n");
      return retVal;
   }

   if (ts->mode == MOT_TOUCH_MODE_BOOTLOADER || ts->mode == MOT_TOUCH_MODE_CRASH)
   {

      if (ts->mode == MOT_TOUCH_MODE_CRASH)
      {
         retVal = qtouch_read(ts, kernelBuffer, 1);
         if (retVal < 0)
         {
            pr_info("[LJ] qtouch_fops_write(): invalid state\n");
         }
         else
         {
            ts_printk("[LJ] qtouch_fops_write(): current ic state-%d\n",
                                               kernelBuffer[0]);
         }
      }

      /* get data from user space into kernel space */
      if (copy_from_user(kernelBuffer, buf, (unsigned long)count))
      {
         pr_info("[LJ] qtouch_fops_write(): copy_from_user Failed \n");
         ts->driver_state = MOT_TOUCH_BL_COPY_DATA_ERROR;
         retVal = -EFAULT;
         goto qtouch_write_error;
      }
      ts_printk("[LJ] qtouch_fops_write(): copy_from_user() copied %i bytes \n",
                                    (int)count);
      ts->client->addr = bl_i2c_address;
      ts_printk("[LJ] qtouch_fops_write(): record to IC: size=%i\n", count);

      /* Updating Bootloader Mode Status  */
      ts->driver_state = MOT_TOUCH_BL_WAITING_FOR_STATUS;
      for (write_addr = kernelBuffer;
           write_addr < kernelBuffer + count;
           write_addr += segment)
      {
         retVal = qtouch_write(ts, write_addr,
                               ((write_addr + segment) > (kernelBuffer + count))
                                                       ? count % segment
                                                       : segment);
         /* On failure, output the error code and
          * delay before trying again */
         if (retVal < 0)
         {
            pr_info("[LJ] qtouch_fops_write(): write failed: %d\n", retVal);
            ts->driver_state = MOT_TOUCH_BL_DATA_WRITE_ERROR;
            goto qtouch_write_error;
            break;
         }
         else
         {
            ts_printk("[LJ] qtouch_fops_write(): OK,Send Size %d\n", retVal);
            retVal = count;
         }

      }
   }
   else if (ts->mode == MOT_TOUCH_MODE_NORMAL)
   {
      pr_info("qtouch_fops_write:In this mode write not supported\n");
   }
   ts_printk("[LJ] Exiting qtouch_fops_write()\n");
   return retVal;
qtouch_write_error:
   pr_info("[LJ] Exiting qtouch_fops_write() with error=%d\n", retVal);
   return retVal;
}


static int qtouch_lock_boot_state(struct qtouch_ts_data *ts)
{
   int ret = 0;
   int retry = 4;
   unsigned char FWupdateInfo[3];

   ts_printk("[LJ] Entering qtouch_lock_boot_state()\n");

   disable_irq(ts->client->irq);
   msleep(10);

   /* back up application mode i2c address */
   app_i2c_address = ts->client->addr;
   /* set up bootloader mode i2c address */
   ts->client->addr = bl_i2c_address;

   memset(FWupdateInfo, 0, 3);

   ts_printk("[LJ] SSA %s: bl_i2c_address .. i2c addr = 0x%X ret = %i\n",
                __func__, ts->client->addr, ret);

   do
   {
      retry--;
      ret = qtouch_read(ts, FWupdateInfo, 1);

      if (ret < 0)
      {
         pr_info("[LJ] %s: I2C read Error .. i2c addr = 0x%X ret = %i\n",
			     __func__, ts->client->addr, ret);
         ts->driver_state = MOT_TOUCH_BL_I2C_READ_ERROR;
         pr_info("[LJ] Resetting the interrupt GPIO back to normal\n");

         /* Update I2c address for Firmware address */
         ts->client->addr = app_i2c_address;
         enable_irq(ts->client->irq);
         ts->mode = MOT_TOUCH_MODE_NORMAL;
         return -EFAULT;
      }

      if ((FWupdateInfo[0] & 0xF0) == 0xC0)
      {
         FWupdateInfo[0] = 0xDC;
         FWupdateInfo[1] = 0xAA;
         ret = qtouch_write(ts, FWupdateInfo, 2);
         if (ret < 0)
         {
            pr_info("[LJ] %s: I2C write Error .. ret = %i\n", __func__, ret);
            ts->driver_state = MOT_TOUCH_BL_I2C_WRITE_ERROR;
            pr_info("[LJ] reset interrupt GPIO back to normal\n");
            enable_irq(ts->client->irq);
            ts->mode = MOT_TOUCH_MODE_NORMAL;
            return -EFAULT;
         }
      }
   } while (retry > 0 && ((FWupdateInfo[0] & 0xF0) != 0x80));

   if ((FWupdateInfo[0] & 0xF0) != 0x80)
   {
      ts->mode = MOT_TOUCH_MODE_NORMAL;
      ts_printk("[LJ] %s: Failed to put the touch IC into bootloader mode\n",
                    __func__);
   }
   else
   {

      /* Currently IC ready for Firmware update.
       * So change deiver mode to BOOTLOADER
       * Driver Got Status 0s80 from Captouch IC,
       * Captouch expects the firmware data from Host.
       * So Driver goes to WAITING_FOR_DATA state.
       */
      ts->driver_state = MOT_TOUCH_BL_WAITING_FOR_DATA;
      if (ts->mode != MOT_TOUCH_MODE_CRASH)
      {
         ts->mode = MOT_TOUCH_MODE_BOOTLOADER;
      }
      ts_printk("[LJ] %s: Mode is set to  %d(BOOTLOADER Mode)\n",
                    __func__, ts->mode);
   }
   enable_irq(ts->client->irq);

   pr_info("[LJ] Entering qtouch_lock_boot_state() with ret=%d\n", ret);
   return ret;
}


static int qtouch_set_boot_mode(struct qtouch_ts_data *ts)
{
   int err = 0;
   int retry = 6;
   struct qtm_object *obj;
   unsigned char qtouch_boot_cmd_reg_value = 0xA5;

   ts_printk("[LJ] Entering qtouch_set_boot_mode()\n");

   /* there is no under-going firmware update */
   if (ts != NULL && ts->FwUpdate_Status == QTOUCH_FW_UPDATE_NOT_DONE)
   {
      while (ts->mode != MOT_TOUCH_MODE_BOOTLOADER && retry-- > 0)
      {
         /* Send sw reset command */
         obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);
         err = qtouch_write_addr(ts, obj->entry.addr,
                                 &qtouch_boot_cmd_reg_value, 1);

         msleep(20);
         if (err == 0)
         {
            err = qtouch_lock_boot_state(ts);
         }
      }
   }
   ts_printk("[LJ] Exiting qtouch_set_boot_mode() with err=%d\n", err);
   return err;
}

static int qtouch_read_fw_version(struct qtouch_ts_data *ts)
{
   struct qtm_id_info qtm_info;
   int err;
   int version = -1;
   struct qtm_object *obj;

   ts_printk("[LJ] Entering qtouch_read_fw_version()\n");

   disable_irq(ts->client->irq);
   msleep(10);

   if (ts->mode == MOT_TOUCH_MODE_NORMAL)
   {
      /* query the device and get the info block. */
      err = qtouch_read_addr(ts, QTM_OBP_ID_INFO_ADDR, &qtm_info,
					   sizeof(qtm_info));
      if (err != 0)
      {
         pr_err("[LJ] Cannot read info object block\n");
         version = -1;
      }
      else
      {
         version = qtm_info.version;
         ts_printk("[LJ] qtm_obj:version:%x\n", qtm_info.version);
         ts_printk("[LJ] qtm_obj:family ID:%x\n", qtm_info.family_id);
         ts_printk("[LJ] qtm_obj:variant ID:%x\n", qtm_info.variant_id);
         ts_printk("[LJ] qtm_obj:build:%x \n", qtm_info.build);
         ts_printk("[LJ] qtm_obj:x size:%x \n", qtm_info.matrix_x_size);
         ts_printk("[LJ] qtm_obj:y size:%x \n", qtm_info.matrix_y_size);
         ts_printk("[LJ] qtm_obj:num object:%x\n", qtm_info.num_objs);

         /* Point the address pointer to the message processor.
          * Must do this before enabling interrupts */
         obj = find_obj(ts, QTM_OBJ_GEN_MSG_PROC);
         err = qtouch_set_addr(ts, obj->entry.addr);
         if (err != 0)
         {
            pr_err("[LJ] Can't to set addr to msg processor\n");
         }
      }
   }

   enable_irq(ts->client->irq);

   ts_printk("[LJ] Exiting qtouch_read_fw_version() with version=0x%X\n",
                                                       version);
   return version;
}

static int qtouch_fops_ioctl(struct inode *node, struct file *filp,
			 unsigned int cmd, unsigned long arg)
{
   int rc = 1;
   //int mode_value = 0;
   struct qtm_object *obj;
   char kernel_buf;

   /* static unsigned char FWupdateInfo[3];   */
   struct qtouch_ts_data *ts;
   ts = qtouch_ts_ptr;

   ts_printk("[LJ] Entering qtouch_fops_ioctl()\n");
   if (ts == NULL)
   {
      pr_info("[LJ] qtouch_fops_ioctl : Error geting qtm_obp_ts_data pointer\n");
      rc = -1;
   }

   switch (cmd)
   {
      case MOT_TOUCH_IOCTL_GET_FW_VERSION:
         ts_printk("[LJ] qtouch_fops_ioctl:MOT_TOUCH_IOCTL_GET_FW_VERSION\n");
         if (ts->FwUpdate_Status == QTOUCH_FW_UPDATE_NOT_DONE &&
			ts->mode == MOT_TOUCH_MODE_NORMAL)
         {
            ts->FwVersion = qtouch_read_fw_version(ts);
            ts_printk("[LJ] qtouch_fops_ioctl:firmware version is %d\n", ts->FwVersion);
            if (put_user(ts->FwVersion, (int *)arg) != 0)
            {
               pr_info("[LJ] Failed to copy fw version to user space\n");
               rc = -EFAULT;
            }
         }
         else
         {
            pr_info("[LJ] Driver not support this operation in this mode\n");
            rc = -EFAULT;
         }
         break;
      case MOT_TOUCH_IOCTL_SET_BOOT_MODE:
         ts_printk("[LJ] qtouch_fops_ioctl:MOT_TOUCH_IOCTL_SET_BOOT_MODE\n");
         if (ts->mode == MOT_TOUCH_MODE_NORMAL)
         {
            qtouch_set_boot_mode(ts);
         }
         break;
      case MOT_TOUCH_IOCTL_BL_GET_STATUS:
         ts_printk("[LJ] ioctl: get_bl_state = %d\n", ts->driver_state);
         if (ts->FwUpdate_Status == QTOUCH_FW_UPDATE_NOT_DONE)
         {
            if (put_user(ts->driver_state, (int *)arg) != 0)
            {
               pr_info("[LJ] Failed to copy driver state to user space\n");
               rc = -EFAULT;
            }
         }
         else
         {
            rc = -EFAULT;
            pr_info("[LJ] Driver won't support this op in this mode\n");
         }
         break;
      case MOT_TOUCH_IOCTL_GET_IC_MODE:
         ts_printk("[LJ] qtouch_fops_ioctl:current ic mode is %d\n", ts->mode);
         if (put_user(ts->mode, (int *)arg) != 0)
         {
            pr_info("[LJ] Failed to copy driver state to user space\n");
            rc = -EFAULT;
         }
         break;
      case MOT_TOUCH_IOCTL_ENABLE_TOUCH:
         rc = qtouch_power_config(ts, 1);
         if (rc < 0)
         {
            rc = -EFAULT;
            pr_err("[LJ] %s: Cannot write power config\n", __func__);
         }
         obj = find_obj(ts, QTM_OBJ_GEN_MSG_PROC);
         rc = qtouch_set_addr(ts, obj->entry.addr);
         if (rc != 0)
         {
            rc = -EFAULT;
            pr_err("[LJ] Can't to set addr to msg processor\n");
         }
         rc = 1;
         break;
      case MOT_TOUCH_IOCTL_DISABLE_TOUCH:
         rc = qtouch_power_config(ts, 1);
         if (rc < 0)
         {
            rc = -EFAULT;
            pr_err("[LJ] %s: Cannot write power config\n", __func__);
         }
         rc = 1;
         break;
      case MOT_TOUCH_IOCTL_GET_TOUCH_DATA:
         ts_printk("[LJ] %s: finger x / y = %d / %d\n", __FUNCTION__,
                                tcmd_touch_data.x_data, tcmd_touch_data.y_data);

         rc = copy_to_user((void __user *)arg,
                           (void *)&tcmd_touch_data,
                           (unsigned long) sizeof(struct coordinate_map));
         ts_printk("[LJ] %s: copy_to_user = (%d)\n",__FUNCTION__,rc);

         if ( rc != 0 )
         {
            rc = -EFAULT;
         }
         else
         {
            rc = 1;
            ts_printk("[LJ] %s: coordinates x / y = %d / %d\n", __FUNCTION__,
                       ((struct coordinate_map *)arg)[0].x_data,
                       ((struct coordinate_map *)arg)[0].y_data);
            ts_printk("[LJ] %s: Getting touchpoint coordinates. (%d)\n",__FUNCTION__,rc);
         }
         break;

      case MOT_TOUCH_IOCTL_SET_IRQ:
         rc = copy_from_user(&kernel_buf, (void __user *)arg, (unsigned long)sizeof(char));
         ts_printk("[LJ] %s: irq = (%d)\n",__FUNCTION__, kernel_buf);

         if (rc == 0)
         {
            if (kernel_buf == 0)
            {
               ts_printk("[LJ] qtouch_fops_ioctl: disable irq\n ");
               disable_irq(ts->client->irq);
               rc = 1;
            }
            else if (kernel_buf == 1)
            {
               ts_printk("[LJ] qtouch_fops_ioctl: enable irq\n ");
               enable_irq(ts->client->irq);
               rc = 1;
            }
            else
            {
               ts_printk("[LJ] qtouch_fops_ioctl:INVALID irq command\n ");
               rc = -EFAULT;
            }
         }
         else
         {
            rc = -EFAULT;
         }
         break;

      default:
         ts_printk("[LJ] qtouch_fops_ioctl:INVALID IOCTL\n ");
   }
   ts_printk("[LJ] Exiting qtouch_fops_ioctl() with rc=%d\n", rc);
   return rc;
}

static int qtouch_write(struct qtouch_ts_data *ts, void *buf, int buf_sz)
{
   int retries = 10;
   int ret;

   ts_printk("[LJ] Entering qtouch_write()\n");

   do
   {
      ret = i2c_master_send(ts->client, (char *)buf, buf_sz);
   } while ((ret < buf_sz) && (--retries > 0));

   if (ret < 0)
   {
      pr_info("[LJ] %s: Error while trying to write %d bytes\n",
                                         __func__, buf_sz);
   }
   else if (ret != buf_sz)
   {
      pr_info("[LJ] %s: Write %d bytes, expected %d\n", __func__,
			ret, buf_sz);
      ret = -EIO;
   }
   ts_printk("[LJ] Exiting qtouch_write() with write_size=%d\n", ret);
   return ret;
}

static int qtouch_set_addr(struct qtouch_ts_data *ts, uint16_t addr)
{
   int ret;

   ts_printk("[LJ] Entering qtouch_set_addr()-addr=0x%X\n", addr);

   /* Note: addr on the wire is LSB first */
   ret = qtouch_write(ts, (char *)&addr, sizeof(uint16_t));
   if (ret < 0)
   {
      pr_info("[LJ] %s: Can't send obp addr 0x%4x\n", __func__, addr);
   }

   ts_printk("[LJ] Exiting qtouch_set_addr() with %d(>=0 SUC/<0 FAIL)\n", ret);
   return ret >= 0 ? 0 : ret;
}

static int qtouch_read(struct qtouch_ts_data *ts, void *buf, int buf_sz)
{
   int retries = 10;
   int ret;

   ts_printk("[LJ] Entering qtouch_read()\n");

   do
   {
      memset(buf, 0, buf_sz);
      ret = i2c_master_recv(ts->client, (char *)buf, buf_sz);
   } while ((ret < 0) && (--retries > 0));

   if (ret < 0)
   {
      pr_info("[LJ] %s: Error while trying to read %d bytes\n",
                    __func__, buf_sz);
   }
   else if (ret != buf_sz)
   {
      pr_info("[LJ] %s: Read %d bytes, expected %d\n", __func__,
                    ret, buf_sz);
      ret = -EIO;
   }

   ts_printk("[LJ] Exiting qtouch_read() with %d(>=0 SUC/<0 FAIL)\n", ret);
   return ret >= 0 ? 0 : ret;
}

static int qtouch_read_addr(struct qtouch_ts_data *ts, uint16_t addr,
			    void *buf, int buf_sz)
{
   int ret;

   ts_printk("[LJ] Entering qtouch_read_addr()-addr=0x%X\n", addr);

   ret = qtouch_set_addr(ts, addr);
   if (ret != 0)
   {
      return ret;
   }

   ts_printk("[LJ] Exiting qtouch_read_addr()\n");
   return qtouch_read(ts, buf, buf_sz);
}

static struct qtm_obj_message *qtouch_read_msg(struct qtouch_ts_data *ts)
{
   int ret;

   ts_printk("[LJ] Entering qtouch_read_msg()\n");

   ret = qtouch_read(ts, ts->msg_buf, ts->msg_size);
   if (!ret)
   {
      pr_info("[LJ] Exiting qtouch_read_msg() with report id-%d\n",
                                              ts->msg_buf[0]);
      return (struct qtm_obj_message *)ts->msg_buf;
   }

   ts_printk("[LJ] Exiting qtouch_read_msg() with NULL\n");
   return NULL;
}

static int qtouch_write_addr(struct qtouch_ts_data *ts, uint16_t addr,
			     void *buf, int buf_sz)
{
   int ret;
   uint8_t write_buf[128];

   ts_printk("[LJ] Entering qtouch_write_addr()\n");

   if (buf_sz + sizeof(uint16_t) > sizeof(write_buf))
   {
      pr_err("[LJ] %s: Buffer too large (%d)\n", __func__, buf_sz);
      return -EINVAL;
   }

   memcpy(write_buf, (void *)&addr, sizeof(addr));
   memcpy((void *)write_buf + sizeof(addr), buf, buf_sz);

   ret = qtouch_write(ts, write_buf, buf_sz + sizeof(addr));

   if (ret < 0)
   {
      pr_err("[LJ] %s: Could not write %d bytes.\n", __func__, buf_sz);
      pr_err("[LJ] Exiting qtouch_write_addr() with ret=%d\n", ret);
      return ret;
   }
   ts_printk("[LJ] Exiting qtouch_write_addr()\n");
   return 0;
}


static inline struct qtm_object *find_obj(struct qtouch_ts_data *ts, int id)
{
   ts_printk("[LJ] find_obj() id=%d\n", id);
   return &ts->obj_tbl[id];
}

static struct qtm_object *create_obj(struct qtouch_ts_data *ts,
				     struct qtm_obj_entry *entry)
{
   struct qtm_object *obj;

   ts_printk("[LJ] Entering create_obj()\n");
   obj = &ts->obj_tbl[entry->type];
   memcpy(&obj->entry, entry, sizeof(*entry));
   set_bit(entry->type, ts->obj_map);

   ts_printk("[LJ] create_obj - type=%d, addr=0x%X, size=%d, inst=%d, rids=%d\n",
                                       entry->type, entry->addr, entry->size,
                                       entry->num_inst, entry->num_rids);

   ts_printk("[LJ] Exiting create_obj()\n");
   return obj;
}

static struct qtm_object *find_object_rid(struct qtouch_ts_data *ts, int rid)
{
   int i;

   ts_printk("[LJ] Entering find_object_rid() with rid=%d\n", rid);

   for_each_bit(i, ts->obj_map, QTM_OBP_MAX_OBJECT_NUM)
   {
      struct qtm_object *obj = &ts->obj_tbl[i];

      if ((rid >= obj->report_id_min) && (rid <= obj->report_id_max))
      {
         ts_printk("[LJ] Exiting find_object_rid() with obj\n");
         return obj;
      }
   }

   ts_printk("[LJ] Exiting find_object_rid() with NULL\n");
   return NULL;
}

static int qtouch_gpio_reset(void)
{
   ts_printk("[LJ] Entering qtouch_gpio_reset()\n");

   gpio_set_value(TOUCH_RST_N, 0);
   msleep(20);
   gpio_set_value(TOUCH_RST_N, 1);
   msleep(70);

   ts_printk("[LJ] Exiting qtouch_gpio_reset()\n");

   return 0;
}

static void qtouch_force_reset(struct qtouch_ts_data *ts, uint8_t sw_reset)
{
   struct qtm_object *obj;
   uint16_t addr;
   uint8_t val;
   int ret;

   ts_printk("[LJ] Entering qtouch_force_reset()\n");

   if (ts->pdata->hw_reset && !sw_reset)
   {
      ts_printk("[LJ] %s: Forcing HW reset\n", __func__);
      ts->pdata->hw_reset();
   }
   else if (sw_reset)
   {
      ts_printk("[LJ] %s: Forcing SW reset\n", __func__);
      obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);
      addr = obj->entry.addr + offsetof(struct qtm_gen_cmd_proc, reset);
      val = 1;
      ret = qtouch_write_addr(ts, addr, &val, 1);
      if (ret)
      {
         pr_err("[LJ] %s: Unable to send the reset msg\n", __func__);
      }
   }
   ts_printk("[LJ] Exiting qtouch_force_reset()\n");
}

static int qtouch_force_calibration(struct qtouch_ts_data *ts)
{
   struct qtm_object *obj;
   uint16_t addr;
   uint8_t val;
   int ret;

   ts_printk("[LJ] Entering %s(): Forcing calibration\n", __func__);

   obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);

   addr = obj->entry.addr + offsetof(struct qtm_gen_cmd_proc, calibrate);
   val = 1;
   ret = qtouch_write_addr(ts, addr, &val, 1);
   if (ret)
   {
      pr_err("[LJ] %s: Unable to send the calibrate message\n", __func__);
   }
   ts_printk("[LJ] Exiting qtouch_force_calibration() with ret=%d\n", ret);
   return ret;
}

#undef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
static int qtouch_power_config(struct qtouch_ts_data *ts, int on)
{
   struct qtm_gen_power_cfg pwr_cfg;
   struct qtm_object *obj;

   ts_printk("[LJ] Entering qtouch_power_config()\n");

   if (!on)
   {
      /* go to standby mode */
      ts_printk("[LJ] qtouch_power_config() suspending \n");
      pwr_cfg.idle_acq_int = 0;
      pwr_cfg.active_acq_int = 0;
   }
   else
   {
      /* normal value */
      ts_printk("[LJ] qtouch_power_config() resuming \n");
      pwr_cfg.idle_acq_int = ts->pdata->power_cfg.idle_acq_int;
      pwr_cfg.active_acq_int = ts->pdata->power_cfg.active_acq_int;
   }

   pwr_cfg.active_idle_to = ts->pdata->power_cfg.active_idle_to;

   obj = find_obj(ts, QTM_OBJ_GEN_PWR_CONF);

   if (obj == NULL)
   {
      ts_printk("[LJ] Exiting qtouch_power_config() with error\n");
      return -1;
   }
   else
   {
      ts_printk("[LJ] Exiting qtouch_power_config()\n");
      return qtouch_write_addr(ts, obj->entry.addr, &pwr_cfg,
                               min(sizeof(pwr_cfg), obj->entry.size));
   }
}

/* Apply the configuration provided in the platform_data to the hardware */
static int qtouch_hw_init(struct qtouch_ts_data *ts)
{
   struct qtm_object *obj;
   int i;
   int ret;
   uint16_t adj_addr;

   ts_printk("[LJ] Entering %s(): Doing hw init\n", __func__);

   /* take the IC out of suspend */
   qtouch_power_config(ts, 1);

   /* configure the acquisition object. */
   obj = find_obj(ts, QTM_OBJ_GEN_ACQUIRE_CONF);
   ret = qtouch_write_addr(ts, obj->entry.addr, &ts->pdata->acquire_cfg,
                           min(sizeof(ts->pdata->acquire_cfg),
                           obj->entry.size));
   if (ret != 0)
   {
      pr_err("[LJ] %s: Can't write acquisition T8 config\n", __func__);
      return ret;
   }

   /* The multitouch and keyarray objects have very similar memory
    * layout, but are just different enough where we basically have to
    * repeat the same code */

   /* configure the multi-touch object. */
   obj = find_obj(ts, QTM_OBJ_TOUCH_MULTI);
   if (obj && obj->entry.num_inst > 0)
   {
      struct qtm_touch_multi_cfg cfg;
      memcpy(&cfg, &ts->pdata->multi_touch_cfg, sizeof(cfg));

      if (ts->pdata->flags & QTOUCH_USE_MULTITOUCH)
      {
         cfg.ctrl |= (1 << 1) | (1 << 0); /* reporten | enable */
      }
      else
      {
         cfg.ctrl = 0;
      }
      ret = qtouch_write_addr(ts, obj->entry.addr, &cfg,
                              min(sizeof(cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write multi-touch T9 config\n",__func__);
         return ret;
      }
   }

   if (obj && obj->entry.num_inst > 1)
   {
      struct qtm_touch_multi_cfg cfg;
      memcpy(&cfg, &ts->pdata->multi_touch_cfg2, sizeof(cfg));

      if (ts->pdata->flags & QTOUCH_USE_TINYTOUCH)
      {
         cfg.ctrl |= (1 << 1) | (1 << 0); /* reporten | enable */
      }
      else
      {
         cfg.ctrl = 0;
      }
      ret = qtouch_write_addr(ts, obj->entry.addr + obj->entry.size,
                              &cfg, min(sizeof(cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write multi-touch T9-2 config\n", __func__);
         return ret;
      }
   }

   /* configure the key-array object. */
   obj = find_obj(ts, QTM_OBJ_TOUCH_KEYARRAY);
   if (obj && obj->entry.num_inst > 0)
   {
      struct qtm_touch_keyarray_cfg cfg;

      for (i = 0; i < obj->entry.num_inst; i++)
      {
         if (i > (ts->pdata->key_array.num_keys - 1))
         {
            pr_info("[LJ] %s: No entry key instance.\n", __func__);
            memset(&cfg, 0, sizeof(cfg));
         }
         else if (ts->pdata->flags & QTOUCH_USE_KEYARRAY)
         {
            memcpy(&cfg, &ts->pdata->key_array.cfg[i], sizeof(cfg));
            cfg.ctrl |= (1 << 1) | (1 << 0); /* reporten | enable */
         }
         else
         {
            memset(&cfg, 0, sizeof(cfg));
         }

         adj_addr = obj->entry.addr + ((obj->entry.size + 1) * i);
         ret = qtouch_write_addr(ts, adj_addr, &cfg,
                                 min(sizeof(cfg), obj->entry.size));
         if (ret != 0)
         {
            pr_err("[LJ] %s: Can't write keyarray T15 config\n", __func__);
            return ret;
         }
      }
   }

   /* configure the SPT COMMSCONFIG T18 */
   obj = find_obj(ts, QTM_OBJ_SPT_COM_CONFIG);
   if (obj && obj->entry.num_inst > 0)
   {
      ret = qtouch_write_addr(ts, obj->entry.addr,
                &ts->pdata->commsconfig_cfg,
                min(sizeof(ts->pdata->commsconfig_cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write the SPT COMMSCONFIG T18 config\n",
                      __func__);
         return ret;
      }
   }

   /* configure the GPIO PWM support */
   obj = find_obj(ts, QTM_OBJ_SPT_GPIO_PWM);
   if (obj && obj->entry.num_inst > 0)
   {
      ret = qtouch_write_addr(ts, obj->entry.addr,
                &ts->pdata->gpio_pwm_cfg,
                min(sizeof(ts->pdata->gpio_pwm_cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write the GPIO PWM T19 config\n",
                      __func__);
         return ret;
      }
   }

   /* configure the grip suppression table */
   obj = find_obj(ts, QTM_OBJ_PROCI_GRIPFACESUPPRESSION);
   if (obj && obj->entry.num_inst > 0)
   {
      ret = qtouch_write_addr(ts, obj->entry.addr,
                              &ts->pdata->grip_suppression_cfg,
                min(sizeof(ts->pdata->grip_suppression_cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write the grip suppression T20 config\n",
                      __func__);
         return ret;
      }
   }

   /* configure noise suppression */
   obj = find_obj(ts, QTM_OBJ_PROCG_NOISE_SUPPRESSION);
   if (obj && obj->entry.num_inst > 0)
   {
      ret = qtouch_write_addr(ts, obj->entry.addr,
                              &ts->pdata->noise_suppression_cfg,
              min(sizeof(ts->pdata->noise_suppression_cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write the noise suppression T22 config\n",
                      __func__);
         return ret;
      }
   }

   if (obj && obj->entry.num_inst > 1)
   {
      struct qtm_procg_noise_suppression_cfg cfg;
      memcpy(&cfg, &ts->pdata->noise_suppression_cfg2, sizeof(cfg));

      ret = qtouch_write_addr(ts, obj->entry.addr + obj->entry.size,
                              &cfg, min(sizeof(cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write noise suppression T22-2 config\n", __func__);
         return ret;
      }
   }


   /* configure the touch proximity sensor */
   obj = find_obj(ts, QTM_OBJ_TOUCH_PROXIMITY);
   if (obj && obj->entry.num_inst > 0)
   {
      ret = qtouch_write_addr(ts, obj->entry.addr,
                              &ts->pdata->touch_proximity_cfg,
                min(sizeof(ts->pdata->touch_proximity_cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write the touch proximity T23 config\n",
                      __func__);
         return ret;
      }
   }

   /* configure the one touch gesture processor */
   obj = find_obj(ts, QTM_OBJ_PROCI_ONE_TOUCH_GESTURE_PROC);
   if (obj && obj->entry.num_inst > 0)
   {
      ret = qtouch_write_addr(ts, obj->entry.addr,
                              &ts->pdata->one_touch_gesture_proc_cfg,
             min(sizeof(ts->pdata->one_touch_gesture_proc_cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write the one touch gesture processor T24 config\n",
                      __func__);
         return ret;
      }
   }

   /* configure self test */
   obj = find_obj(ts, QTM_OBJ_SPT_SELF_TEST);
   if (obj && obj->entry.num_inst > 0)
   {
      ret = qtouch_write_addr(ts, obj->entry.addr,
                              &ts->pdata->self_test_cfg,
                min(sizeof(ts->pdata->self_test_cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write the self test T25 config\n", __func__);
         return ret;
      }
   }

   /* configure the two touch gesture processor */
   obj = find_obj(ts, QTM_OBJ_PROCI_TWO_TOUCH_GESTURE_PROC);
   if (obj && obj->entry.num_inst > 0)
   {
      ret = qtouch_write_addr(ts, obj->entry.addr,
                              &ts->pdata->two_touch_gesture_proc_cfg,
           min(sizeof(ts->pdata->two_touch_gesture_proc_cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write the two touch gesture processor T27 config\n",
                      __func__);
         return ret;
      }
   }

   /* configure the capacitive touch engine  */
   obj = find_obj(ts, QTM_OBJ_SPT_CTE_CONFIG);
   if (obj && obj->entry.num_inst > 0)
   {
      ret = qtouch_write_addr(ts, obj->entry.addr,
                              &ts->pdata->cte_config_cfg,
                min(sizeof(ts->pdata->cte_config_cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write the capacitive touch engine T28 config\n",
                      __func__);
         return ret;
      }
   }

   /* configure the SPT USERDATA T38 */
   obj = find_obj(ts, QTM_OBJ_SPT_USERDATA);
   if (obj && obj->entry.num_inst >0)
   {
      ret = qtouch_write_addr(ts, obj->entry.addr,
                              &ts->pdata->userdata_cfg,
                    min(sizeof(ts->pdata->userdata_cfg), obj->entry.size));
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't write the userdata T38 config\n",
                      __func__);
         return ret;
      }
   }

   ret = qtouch_force_calibration(ts);
   if (ret != 0)
   {
      pr_err("[LJ] %s: Unable to recalibrate after reset\n", __func__);
      return ret;
   }

   /* Write the settings into nvram, if needed */
   if (ts->pdata->flags & QTOUCH_CFG_BACKUPNV)
   {
      uint8_t val;
      uint16_t addr;

      ts_printk("[LJ] %s: backup configuration\n", __func__);

      obj = find_obj(ts, QTM_OBJ_GEN_CMD_PROC);
      addr = obj->entry.addr + offsetof(struct qtm_gen_cmd_proc,
                                        backupnv);
      val = 0x55;
      ret = qtouch_write_addr(ts, addr, &val, 1);
      if (ret != 0)
      {
         pr_err("[LJ] %s: Can't backup nvram settings\n", __func__);
         return ret;
      }
      /* Since the IC does not indicate that has completed the
         backup place a hard wait here.  If we communicate with the
         IC during backup the EEPROM may be corrupted */

      msleep(500);
   }

   /* If debugging, read back and print all settings */
   if (qtouch_tsdebug)
   {
      int object;
      int size;
      uint8_t *data_buff;
      int byte;
      int msg_bytes;
      int msg_location;
      char *msg;

      msg = kmalloc(1024, GFP_KERNEL);
      if (msg != NULL)
      {
         for (object = 7; object < QTM_OBP_MAX_OBJECT_NUM; object++)
         {
            size = ts->obj_tbl[object].entry.size * ts->obj_tbl[object].entry.num_inst;
            if (size != 0)
            {
               data_buff = kmalloc(size, GFP_KERNEL);
               if (data_buff == NULL)
               {
                  pr_err("[LJ] %s: Object %d: Malloc failed\n", __func__, object);
                  continue;
               }

               qtouch_read_addr(ts, ts->obj_tbl[object].entry.addr,
                                (void *)data_buff, size);

               msg_location = sprintf(msg, "[LJ] %s: Object %d:", __func__, object);
               for (byte=0; byte < size; byte++)
               {
                  msg_bytes = snprintf((msg + msg_location), (1024 - msg_location),
                                        " 0x%02x", *(data_buff + byte));
                  msg_location += msg_bytes;
                  if (msg_location >= 1024)
                  {
                     break;
                  }
               }
               if (msg_location < 1024)
               {
                  ts_printk("[LJ] %s\n", msg);
               }
               else
               {
                  ts_printk("[LJ] %s:Object %d:String overflow\n", __func__,
                                      object);
               }
               kfree (data_buff);
            }
         }
         kfree (msg);
      }

      qtouch_set_addr(ts, ts->obj_tbl[QTM_OBJ_GEN_MSG_PROC].entry.addr);
   }

   /* reset the address pointer */
   ret = qtouch_set_addr(ts, ts->obj_tbl[QTM_OBJ_GEN_MSG_PROC].entry.addr);
   if (ret != 0)
   {
      pr_err("[LJ] %s: Unable to reset address pointer after reset - ret=%d\n",
                   __func__, ret);
      return ret;
   }

   ts_printk("[LJ] Exiting qtouch_hw_init()\n");

   return 0;
}

/* Handles a message from the command processor object. */
static int do_cmd_proc_msg(struct qtouch_ts_data *ts, struct qtm_object *obj,
			   void *_msg)
{
   struct qtm_cmd_proc_msg *msg = _msg;
   int ret = 0;
   int hw_reset = 0;

   ts_printk("[LJ] Entering do_cmd_proc_msg()\n");

   ts_printk("[LJ] %s:checksum for config table = 0x%X%X%X\n", __func__,
                  msg->checksum_1, msg->checksum_2, msg->checksum_3);

   if (msg->status & QTM_CMD_PROC_STATUS_RESET)
   {
      if (qtouch_tsdebug)
      {
         pr_info("[LJ] %s:EEPROM checksum is 0x%X%X%X cnt %i\n", __func__,
             msg->checksum_1, msg->checksum_2, msg->checksum_3, ts->checksum_cnt);
      }

      ts_printk("[LJ] %s: backup checksum = 0x%X%X%X\n", __func__,
               ts->eeprom_checksum[0], ts->eeprom_checksum[1], ts->eeprom_checksum[2]);

#if 0
      if (msg->checksum != ts->eeprom_checksum)
      {
         if (ts->checksum_cnt > 2)
         {
            /* Assume the checksum is what it is, cannot
               disable the touch screen so set the checksum*/
            ts->eeprom_checksum = msg->checksum;
            ts->checksum_cnt = 0;
         }
         else
#endif
      if ( memcmp(&msg->checksum_1, ts->eeprom_checksum, (size_t)ts->checksum_cnt) )
      {
         {
            ts_printk("[LJ] %s: checksum mismatch, configure ...\n", __func__);
            ret = qtouch_hw_init(ts);
            if (ret != 0)
            {
               pr_err("[LJ] %s:Cannot init the touch IC\n", __func__);
            }
            hw_reset = 1;
            //ts->checksum_cnt++;
         }
      }
      ts_printk("[LJ] %s: Reset done.\n", __func__);
   }

   if (msg->status & QTM_CMD_PROC_STATUS_CAL)
   {
      ts_printk("[LJ] %s: Self-calibration started.\n", __func__);
   }

   if (msg->status & QTM_CMD_PROC_STATUS_OFL)
   {
      pr_err("[LJ] %s: Acquisition cycle length overflow\n", __func__);
   }

   if (msg->status & QTM_CMD_PROC_STATUS_SIGERR)
   {
      pr_err("[LJ] %s: Acquisition error\n", __func__);
   }

   if (msg->status & QTM_CMD_PROC_STATUS_CFGERR)
   {
      ret = qtouch_hw_init(ts);
      if (ret != 0)
      {
         pr_err("[LJ] %s:Cannot init the touch IC\n", __func__);
      }

      pr_err("[LJ] %s: Configuration error\n", __func__);
   }
   /* Check the EEPROM checksum.  An ESD event may cause
      the checksum to change during operation so we need to
      reprogram the EEPROM and reset the IC */
   if (ts->pdata->flags & QTOUCH_EEPROM_CHECKSUM)
   {
      //if (msg->checksum != ts->eeprom_checksum)
      if ( memcmp(&msg->checksum_1, ts->eeprom_checksum, (size_t)ts->checksum_cnt) )
      {
         if (qtouch_tsdebug)
         {
            pr_info("[LJ] %s:EEPROM checksum is 0x%X%X%X cnt %i hw_reset %i\n",
                       __func__, msg->checksum_1, msg->checksum_2, msg->checksum_3,
                       ts->checksum_cnt, hw_reset);
         }
#if 0
         if (ts->checksum_cnt > 2)
         {
            /* Assume the checksum is what it is, cannot
               disable the touch screen so set the checksum*/
            ts->eeprom_checksum = msg->checksum;
            ts->checksum_cnt = 0;
         }
         else
#endif
         {
            if (!hw_reset)
            {
               ret = qtouch_hw_init(ts);
            if (ret != 0)
            {
               pr_err("[LJ] %s:Cannot init the touch IC\n", __func__);
            }
            qtouch_force_reset(ts, 0);
            //ts->checksum_cnt++;
            }
         }
      }
   }
   ts_printk("[LJ] Exiting do_cmd_proc_msg() with ret=%d\n", ret);
   return ret;
}


/*!
 * This function implements the mouse function.  This function
 * will process mouse movements in the X,Y plane and produce the
 * corresponding trackball events. Double Tap is a timed event that must
 * occur within the same area on the touch area.
 *
 *
 * @param ts	 Touch screen structure pointer.
 * @param x 	 Raw X data from the IC
 * @param y 	 Raw Y data from the IC
 * @param touch  Indicates the status of the touch.
 *
 * @return None
 */
static void minipad_process_as_trackball(struct qtouch_ts_data *ts, unsigned int x,
                                         unsigned int y, unsigned int touch)
{

   int delta_x;
   int delta_y;

   ts_printk("[LJ] Entering minipad_process_as_trackball()\n");
   ts_printk("[LJ] minipad_process_as_trackball() x_start/y_start = %d/%d\n",
                        x_start, y_start);

   /* initial touchdown */
   if (!x_start && !y_start && touch)
   {
      x_start = x;
      y_start = y;
      if (qtouch_tsdebug & 2)
      {
         pr_info("===+===press\n");
      }
      /*ts_printk("[LJ] minipad_process_as_trackball() - initial BTN_MOUSE\n");
      input_report_key(ts->input_dev, BTN_MOUSE, QTM_OBP_KEY_PRESS);*/
      return;
   }

   if (!touch)
   {
      x_start = 0;
      y_start = 0;
      if (qtouch_tsdebug & 2)
      {
         pr_info("===+===release\n");
      }
      /*ts_printk("[LJ] minipad_process_as_trackball() - BTN_MOUSE\n");
      input_report_key(ts->input_dev, BTN_MOUSE, QTM_OBP_KEY_RELEASE);
      input_sync(ts->input_dev);*/
      return;
   }

   delta_x = (int)x-(int)x_start;
   delta_y = (int)y-(int)y_start;

   ts_printk("[LJ] minipad_process_as_trackball() - REL_X/REL_Y = %d/%d\n",
                               delta_x, delta_y);
   input_report_rel(ts->input_dev, REL_X, TINY_GET_TRACKBALL_REL_EVT(delta_x));
   input_report_rel(ts->input_dev, REL_Y, TINY_GET_TRACKBALL_REL_EVT(delta_y));
   input_sync(ts->input_dev);
   if (qtouch_tsdebug & 2)
   {
      pr_info("===+===move delta_x:%d; delta_y%d; touch:%d\n", delta_x,
			delta_y, touch);
   }

   x_start = x;
   y_start = y;

   ts_printk("[LJ] Exiting minipad_process_as_trackball()\n");

   return;
}


/* Handles a message from the minipad touch object. */
static int do_touch_tiny_msg(struct qtouch_ts_data *ts,
                             struct qtm_object *obj, void *_msg)
{
   struct qtm_touch_multi_msg *msg = _msg;
   int x;
   int y;
   int pressure;
   int width;
   int down;
   uint32_t finger;

   ts_printk("[LJ] Entering do_touch_tiny_msg()\n");

   /* just one finger is allowed at minipad */
   finger = msg->report_id - (obj->report_id_min + obj->entry.num_rids);
   if (finger != 0)
   {
      return 0;
   }
   ts_printk("[LJ] do_touch_tiny_msg() finger=%d\n", finger);

   /* x/y are 10bit values, with bottom 2 bits inside the xypos_lsb */
   x = (msg->xpos_msb << 2) | ((msg->xypos_lsb >> 6) & 0x3);
   y = (msg->ypos_msb << 2) | ((msg->xypos_lsb >> 2) & 0x3);
   width = msg->touch_area;
   pressure = msg->touch_amp;

   ts_printk("[LJ] do_touch_tiny_msg() x/y=%d/%d\n", x, y);

   /* for test command - read position */
   tcmd_touch_data.x_data = x;
   tcmd_touch_data.y_data = y;


   down = !(msg->status & QTM_TOUCH_MULTI_STATUS_RELEASE);
   ts_printk("[LJ] do_touch_tiny_msg() down=%d\n", down);

   if (qtouch_tsdebug & 2)
   {
      pr_info("%s: x=%d y=%d p=%d w=%d touch=%d\n", __func__,
                   x, y, pressure, width, down);
   }

   minipad_process_as_trackball(ts, x, y, msg->status);//down
   ts_printk("[LJ] Exiting do_touch_tiny_msg()\n");
   return 0;
}


/* Handles a message from android keys area */
static void do_touch_virtual_keys(struct qtouch_ts_data *ts, int x, int y,
                                  uint32_t finger, int touch)
{
   int i = 0;
   int button_value = QTM_OBP_NO_BUTTON;
   static unsigned int press_button_sent = 0;
   struct virt_keys v_keys = ts->pdata->vkeys;

   ts_printk("[LJ] Entering do_touch_virtual_keys() x/y=%d/%d\n", x, y);
   /* Loop through the key map and find the button that is associated with
	   the x values */

   ts_printk("[LJ] do_touch_virtual_keys() count=%d\n", v_keys.count);

   /* There was no match to the x coordinates so do not process anything */
   for (i = 0; i < v_keys.count; i++)
   {
      int x_min = v_keys.keys[i].center_x - (v_keys.keys[i].width / 2);
      int y_min = v_keys.keys[i].center_y - (v_keys.keys[i].height / 2);

      ts_printk("[LJ] do_touch_virtual_keys() x/y=%d/%d\n", x_min, y_min);

      if (((x > x_min) && (x < (x_min + v_keys.keys[i].width))) &&
          ((y > y_min) && (y < (y_min + v_keys.keys[i].height))))
      {
         button_value = v_keys.keys[i].code;
         ts_printk("[LJ] do_touch_virtual_keys() %d detect\n", button_value);
      }
   } /* End of the for loop */
   ts_printk("[LJ] do_touch_virtual_keys() previous button = %d\n", ts->last_report_key);

   if((button_value != ts->last_report_key) && (press_button_sent == 1))
   {
      /* Report key release */
      input_report_key(ts->input_dev, ts->last_report_key, QTM_OBP_KEY_RELEASE);
      ts_printk("[LJ] release previous key key_code = %d\n", ts->last_report_key);
      press_button_sent = 0;
      ts->last_report_key = QTM_OBP_NO_BUTTON;
   }

   if ((touch == QTM_OBP_KEY_PRESS) && (button_value != QTM_OBP_NO_BUTTON))
   {
      input_report_key(ts->input_dev, button_value, QTM_OBP_KEY_PRESS);
      ts_printk("[LJ] press current key key_code = %d\n", button_value);
      press_button_sent = 1;
      ts->last_report_key = button_value;
   }

   if ((touch == QTM_OBP_KEY_RELEASE) && (button_value != QTM_OBP_NO_BUTTON))
   {
      /* Press event not sent yet, report key press now */
      if(press_button_sent == 0)
      {
         input_report_key(ts->input_dev, button_value, QTM_OBP_KEY_PRESS);
         ts_printk("[LJ] press key key_code = %d\n", button_value);
      }
      /* Report key release */
      input_report_key(ts->input_dev, button_value, QTM_OBP_KEY_RELEASE);
      ts_printk("[LJ] release key key_code = %d\n", button_value);
      press_button_sent = 0;
      ts->last_report_key = QTM_OBP_NO_BUTTON;
   }
   ts_printk("[LJ] Exiting do_touch_virtual_keys()\n");
}


/* Handles a message from a multi-touch object. */
static int do_touch_multi_msg(struct qtouch_ts_data *ts, struct qtm_object *obj,
			      void *_msg)
{
   struct qtm_touch_multi_msg *msg = _msg;
   int i;
   int x;
   int y;
   int pressure;
   int width;
   uint32_t finger;
   int down;

   ts_printk("[LJ] Entering do_touch_multi_msg()\n");

   if ((obj->report_id_min <= msg->report_id) &&
       (msg->report_id <= obj->report_id_min + obj->entry.num_rids-1))
   {
      ts_printk("[LJ] do_touch_multi_msg() - main touch screen\n");

      finger = msg->report_id - obj->report_id_min;
      if (finger >= ts->pdata->multi_touch_cfg.num_touch)
      {
         pr_err("[LJ] %s: larger finger number %dd\n", __func__, finger);
         return 0;
      }

      /* x/y are 10bit values, with bottom 2 bits inside the xypos_lsb */
      x = (msg->xpos_msb << 2) | ((msg->xypos_lsb >> 6) & 0x3);
      y = (msg->ypos_msb << 2) | ((msg->xypos_lsb >> 2) & 0x3);
      width = msg->touch_area;
      pressure = msg->touch_amp;

      ts_printk("[LJ] %s: stat=%02x, f=%d x=%d y=%d p=%d w=%d\n", __func__,
                 msg->status, finger, x, y, pressure, width);

      if (qtouch_tsdebug & 2)
      {
         pr_info("[LJ] %s: stat=%02x, f=%d x=%d y=%d p=%d w=%d\n", __func__,
                 msg->status, finger, x, y, pressure, width);
      }

      if (finger >= _NUM_FINGERS)
      {
         pr_err("[LJ] %s: Invalid finger number %dd\n", __func__, finger);
         return 1;
      }

      down = !(msg->status & QTM_TOUCH_MULTI_STATUS_RELEASE);

      ts->finger_data[finger].x_data = x;
      ts->finger_data[finger].y_data = y;
      ts->finger_data[finger].w_data = width;

      /* The touch IC will not give back a pressure of zero
         so send a 0 when a liftoff is produced */
      if (!down)
      {
         ts->finger_data[finger].z_data = 0;
      }
      else
      {
         ts->finger_data[finger].z_data = pressure;
         ts->finger_data[finger].down = down;
      }

      /* for test command - read position */
      tcmd_touch_data.x_data = ts->finger_data[finger].x_data;
      tcmd_touch_data.y_data = ts->finger_data[finger].y_data;

      if (y > LCD_Y_PIXEL)
      {
         do_touch_virtual_keys(ts, x, y, finger, down);
      }
      else
      {

         for (i = 0; i < ts->pdata->multi_touch_cfg.num_touch; i++)
         {
            if (ts->finger_data[i].down == 0)
            {
               continue;
            }
            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
                             ts->finger_data[i].z_data);
            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
                             ts->finger_data[i].w_data);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
                             ts->finger_data[i].x_data);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
                             ts->finger_data[i].y_data);
            input_mt_sync(ts->input_dev);
         }

      }
      input_sync(ts->input_dev);

      if (!down)
      {
         memset(&ts->finger_data[finger], 0, sizeof(struct coordinate_map));
      }

   }
   else
   {
      /* processing a tiny touch message */
      do_touch_tiny_msg(ts, obj, msg);
   }

   ts_printk("[LJ] Exiting do_touch_multi_msg()\n");

   return 0;
}


/* Handles a message from a keyarray object. */
static int do_touch_keyarray_msg(struct qtouch_ts_data *ts,
				 struct qtm_object *obj, void *_msg)
{
   struct qtm_touch_keyarray_msg *msg = _msg;
   int i;

   ts_printk("[LJ] Entering do_touch_keyarray_msg()\n");

   /* nothing changed.. odd. */
   if (ts->last_keystate == msg->keystate)
   {
      return 0;
   }

   for (i = 0; i < ts->pdata->key_array.num_keys; ++i)
   {
      struct qtouch_key *key = &ts->pdata->key_array.keys[i];
      uint32_t bit = 1 << (key->channel & 0x1f);
      if ((msg->keystate & bit) != (ts->last_keystate & bit))
      {
         input_report_key(ts->input_dev, key->code,
                          msg->keystate & bit);
      }
   }
   input_sync(ts->input_dev);

   if (qtouch_tsdebug & 2)
   {
      pr_info("[LJ] %s: key state changed 0x%08x -> 0x%08x\n", __func__,
                    ts->last_keystate, msg->keystate);
   }

   /* update our internal state */
   ts->last_keystate = msg->keystate;

   ts_printk("[LJ] Exiting do_touch_keyarray_msg()\n");

   return 0;
}


/* Handles a message from a gpio pwm object. */
static int do_gpio_pwm_msg(struct qtouch_ts_data *ts,
                           struct qtm_object *obj, void *_msg)
{
   struct qtm_spt_gpio_pwm_msg *msg = _msg;
   static unsigned short pressed_gpio1_key_code;

   ts_printk("[LJ] Entering do_gpio_pwm_msg()\n");

   if (msg->gpio_status & QTM_OBP_T19_STATUS_GPIO1_HIGH_MASK)
   {
      if (pressed_gpio1_key_code == KEY_REPLY)
      {
         input_report_key(ts->input_dev, pressed_gpio1_key_code,
                          QTM_OBP_KEY_RELEASE);
      }
      pressed_gpio1_key_code = 0;
   }
   else
   {
      pressed_gpio1_key_code = KEY_REPLY;
      input_report_key(ts->input_dev, pressed_gpio1_key_code,
                       QTM_OBP_KEY_PRESS);
   }
   input_sync(ts->input_dev);
   ts_printk("[LJ] Exiting do_gpio_pwm_msg()\n");

   return 0;
}


static int qtouch_handle_msg(struct qtouch_ts_data *ts, struct qtm_object *obj,
			     struct qtm_obj_message *msg)
{
   int ret = 0;

   ts_printk("[LJ] Entering qtouch_handle_msg()-type=%d\n", obj->entry.type);

   /* These are all the known objects that we know how to handle. */
   switch (obj->entry.type)
   {
      case QTM_OBJ_GEN_CMD_PROC:
         ret = do_cmd_proc_msg(ts, obj, msg);
         break;

      case QTM_OBJ_TOUCH_MULTI:
         ret = do_touch_multi_msg(ts, obj, msg);
         break;

      case QTM_OBJ_TOUCH_KEYARRAY:
         ret = do_touch_keyarray_msg(ts, obj, msg);
         break;

      case QTM_OBJ_SPT_GPIO_PWM:
         ret = do_gpio_pwm_msg(ts, obj, msg);

      default:
         /* probably not fatal? */
         ret = 0;
         ts_printk("%s: No handler defined for message from object"
                       "type %d, report_id %d\n",
                 __func__, obj->entry.type, msg->report_id);
   }

   ts_printk("[LJ] Exiting qtouch_handle_msg() with %d(0 SUC/FAIL)\n", ret);

   return ret;
}

static void qtouch_ts_work_func(struct work_struct *work)
{
   struct qtouch_ts_data *ts = container_of(work, struct qtouch_ts_data, work);
   struct qtm_obj_message *msg;
   struct qtm_object *obj;
   int ret;
   uint8_t buf[QTOUCH_READ_DATA_SIZE];

   ts_printk("[LJ] Entering qtouch_ts_work_func()\n");

   if (ts->mode == MOT_TOUCH_MODE_BOOTLOADER ||
         ts->mode == MOT_TOUCH_MODE_CRASH)
   {
      memset(buf, 0, QTOUCH_READ_DATA_SIZE);
      /* Verifying Driver completed the firmware update
       * successfully */
      ts->client->addr = bl_i2c_address;
      ret = qtouch_read(ts, buf, 1);
      if (ret < 0)
      {
         ts->client->addr = app_i2c_address;
         ts->mode = MOT_TOUCH_MODE_NORMAL;
         ts->driver_state = MOT_TOUCH_BL_FW_UPDATE_SUCCESS;
         ts->FwUpdate_Status = QTOUCH_FW_UPDATE_NOT_DONE;
         ts_printk("[LJ] %s: Firmware upgrade is successful\n", __func__);

         if (qtouch_process_info_block(ts) != 0)
         {
            pr_err("[LJ] Cannot read info block\n");
         }
         else
         {
            /* Point address pointer to the msg processor.
             * Must do this before enabling interrupts */
            obj = find_obj(ts, QTM_OBJ_GEN_MSG_PROC);
            if (qtouch_set_addr(ts, obj->entry.addr) != 0)
            {
               pr_err("[LJ] Can't set addr to msg proc\n");
            }
         }

         goto done;
      }
      else
      {
         ts_printk("[LJ] %s: update status byte is %x\n", __func__, buf[0]);

         /* Do the State management
          * for the firmware update service */
         if ((buf[0] & 0xF0) == 0xC0)
         {
            /* 0x8n indicates that
             * the IC is waiting for data */
            ts->driver_state = MOT_TOUCH_BL_WAITING_FOR_DATA;
         }
         else if ((buf[0] & 0xF0) == 0x80)
         {
            /* 0x8n indicates that
             * the IC is waiting for data */
            ts->driver_state = MOT_TOUCH_BL_WAITING_FOR_DATA;
         }
         else if (buf[0] == 0x02)
         {
            ts->driver_state = MOT_TOUCH_BL_WAITING_FOR_CRC;
         }
         else if (buf[0] == 0x04)
         {
            ts->driver_state = MOT_TOUCH_BL_AFTER_CRC_PASS;
         }
         else if (buf[0] == 0x03)
         {
            /* We got bad CRC on the record */
            ts_printk("[LJ] in MOT_TOUCH_BL_GOT_BAD_CRC state\n");
            ts->driver_state = MOT_TOUCH_BL_GOT_BAD_CRC;
         }
         else if ((buf[0] & 0xF0) == 0x40)
         {

            /* 0x4n indicates that the App CRC error */
            ts->driver_state = MOT_TOUCH_BL_APP_BAD_CRC;
            ts->mode = MOT_TOUCH_MODE_NORMAL;
         }
         goto done;
      }
   }

   msg = qtouch_read_msg(ts);
   if (msg == NULL)
   {
      pr_err("[LJ] %s: Cannot read message\n", __func__);
      goto done;
   }

   obj = find_object_rid(ts, msg->report_id);
   if (!obj)
   {
      pr_err("[LJ] %s: Unknown object for report_id %d\n",
                 __func__, msg->report_id);
      goto done;
   }

   ret = qtouch_handle_msg(ts, obj, msg);
   if (ret != 0)
   {
      pr_err("[LJ] %s: Unable to process message for obj %d, report_id %d\n",
                __func__, obj->entry.type, msg->report_id);
      goto done;
   }

done:
   enable_irq(ts->client->irq);
   ts_printk("[LJ] Exiting qtouch_ts_work_func()\n");
}

static int qtouch_process_info_block(struct qtouch_ts_data *ts)
{
   struct qtm_id_info qtm_info;
   uint16_t our_csum = 0x0;
   uint8_t  their_csum[3];
   uint8_t report_id;
   uint16_t addr;
   int err;
   int i;

   ts_printk("[LJ] Entering qtouch_process_info_block()\n");

   /* query the device and get the info block. */
   err = qtouch_read_addr(ts, QTM_OBP_ID_INFO_ADDR, &qtm_info,
                          sizeof(qtm_info));
   if (err != 0)
   {
      pr_err("[LJ] %s: Cannot read info object block\n", __func__);
      goto err_read_info_block;
   }
   //our_csum = calc_csum(our_csum, &qtm_info, sizeof(qtm_info));

   /* TODO: Add a version/family/variant check? */
   ts_printk("[LJ] %s: Device (0x%x/0x%x/0x%x/0x%x) matrix x/y=%d/%d have %d objects.\n",
              __func__, qtm_info.family_id, qtm_info.variant_id,
              qtm_info.version, qtm_info.build, qtm_info.matrix_x_size,
              qtm_info.matrix_y_size, qtm_info.num_objs);

   if (qtm_info.num_objs == 0)
   {
      err = -ENODEV;
      goto err_no_objects;
   }

   /* Standard mXT224 touch panel, not dual touch chip */
   if (qtm_info.variant_id == 0x1)
   {
      ts->pdata->nv_checksum[0] = 0xE3;
      ts->pdata->nv_checksum[1] = 0x2A;
      ts->pdata->nv_checksum[2] = 0xFF;
   }

   addr = QTM_OBP_ID_INFO_ADDR + sizeof(qtm_info);
   report_id = 1;

   /* Clear the object table */
   for (i = 0; i < QTM_OBP_MAX_OBJECT_NUM; ++i)
   {
      ts->obj_tbl[i].entry.type = 0;
      ts->obj_tbl[i].entry.addr = 0;
      ts->obj_tbl[i].entry.size = 0;
      ts->obj_tbl[i].entry.num_inst = 0;
      ts->obj_tbl[i].entry.num_rids = 0;
      ts->obj_tbl[i].report_id_min = 0;
      ts->obj_tbl[i].report_id_max = 0;
   }

   /* read out the object entries table */
   for (i = 0; i < qtm_info.num_objs; ++i)
   {
      struct qtm_object *obj;
      struct qtm_obj_entry entry;

      err = qtouch_read_addr(ts, addr, &entry, sizeof(entry));
      if (err != 0)
      {
         pr_err("[LJ] %s: Can't read object (%d) entry.\n", __func__, i);
         err = -EIO;
         goto err_read_entry;
      }
      //our_csum = calc_csum(our_csum, &entry, sizeof(entry));
      addr += sizeof(entry);

      entry.size++;
      entry.num_inst++;

      ts_printk("[LJ] %s: Object %d @ 0x%04x (%d) insts %d rep_ids %d\n",
			__func__, entry.type, entry.addr, entry.size,
			entry.num_inst, entry.num_rids);

      if (entry.type >= QTM_OBP_MAX_OBJECT_NUM)
      {
         pr_warning("[LJ] %s: Unknown object type (%d) encountered\n",
                          __func__, entry.type);
         /* Not fatal */
         continue;
      }

      /* save the message_procesor msg_size for easy reference. */
      if (entry.type == QTM_OBJ_GEN_MSG_PROC)
      {
         ts->msg_size = entry.size;
      }

      obj = create_obj(ts, &entry);
      /* set the report_id range that the object is responsible for */
      if ((obj->entry.num_rids * obj->entry.num_inst) != 0)
      {
         obj->report_id_min = report_id;
         report_id += obj->entry.num_rids * obj->entry.num_inst;
         obj->report_id_max = report_id - 1;
      }
   }

   if (!ts->msg_size)
   {
      pr_err("[LJ] %s: Message processing object not found. Bailing.\n",
		       __func__);
      err = -ENODEV;
      goto err_no_msg_proc;
   }

   /* verify that some basic objects are present. These objects are
    * assumed to be present by the rest of the driver, so fail out now
    * if the firmware is busted. */
   if (!find_obj(ts, QTM_OBJ_GEN_PWR_CONF) ||
       !find_obj(ts, QTM_OBJ_GEN_ACQUIRE_CONF) ||
       !find_obj(ts, QTM_OBJ_GEN_MSG_PROC) ||
       !find_obj(ts, QTM_OBJ_GEN_CMD_PROC))
   {
      pr_err("[LJ] %s: Required objects are missing\n", __func__);
      err = -ENOENT;
      goto err_missing_objs;
   }
   else
   {
      /* Check if the instance 2 for multi object is used */
      struct qtm_object *obj = find_obj(ts, QTM_OBJ_TOUCH_MULTI);
      if (obj->entry.num_inst > 1)
      {
         ts_printk("[LJ] %s: mini pad exist\n", __func__);
         ts->pdata->flags = ts->pdata->flags | QTOUCH_USE_TINYTOUCH;

         /* checksum for dual touch configuration */
         config_checksum[0] = 0xED;
         config_checksum[1] = 0x85;
         config_checksum[2] = 0xC7;
      }
      else
      {
         /* checksum for main touch configuration */
         config_checksum[0] = 0x03;
         config_checksum[1] = 0x77;
         config_checksum[2] = 0x6D;
      }
      memcpy(ts->eeprom_checksum, config_checksum, sizeof(config_checksum));
   }


   /* Read the checksum for ID Information block */
   err = qtouch_read_addr(ts, addr, their_csum, sizeof(their_csum));
   if (err != 0)
   {
      pr_err("[LJ] %s: Unable to read remote checksum\n", __func__);
      err = -ENODEV;
      goto err_no_checksum;
   }

   /* FIXME: The algorithm described in the datasheet doesn't seem to
    * match what the touch firmware is doing on the other side. We
    * always get mismatches! */
   //if (our_csum != their_csum)
   if ( memcmp(ts->pdata->nv_checksum, their_csum, sizeof(their_csum)))
   {
      pr_warning("[LJ] %s: Checksum mismatch (0x%04x != 0x%x%x%x)\n",
			   __func__, our_csum,
			   their_csum[0], their_csum[1], their_csum[2]);
#ifndef IGNORE_CHECKSUM_MISMATCH
      err = -ENODEV;
      goto err_bad_checksum;
#endif
   }

   ts_printk("[LJ] %s: %s found. family 0x%x, variant 0x%x, ver 0x%x, "
                 "build 0x%x, matrix %dx%d, %d objects.\n",
             __func__, QTOUCH_TS_NAME, qtm_info.family_id,
             qtm_info.variant_id, qtm_info.version, qtm_info.build,
             qtm_info.matrix_x_size, qtm_info.matrix_y_size, qtm_info.num_objs);

   //ts->eeprom_checksum = ts->pdata->nv_checksum;
   //memset(ts->eeprom_checksum, ts->pdata->nv_checksum, 3);

   ts_printk("[LJ] Exiting qtouch_process_info_block()\n");
   return 0;

err_no_checksum:
err_missing_objs:
err_no_msg_proc:
err_read_entry:
err_no_objects:
err_read_info_block:
   pr_info("[LJ] Exiting qtouch_process_info_block() with err=%d\n", err);
   return err;
}

static int qtouch_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
   struct qtouch_ts_platform_data *pdata = &mxt224_ts_platform_data;
   struct qtouch_ts_data *ts;
   struct qtm_object *obj;
   int err = 0;
   int i;
   char buf;

   ts_printk("[LJ] Entering qtouch_ts_probe()\n");

   if (pdata == NULL)
   {
      pr_err("[LJ] %s: platform data required\n", __func__);
      return -ENODEV;
   }
   else if (!client->irq)
   {
      pr_err("[LJ] %s: polling mode currently not supported\n", __func__);
      return -ENODEV;
   }
   else if (!pdata->hw_reset)
   {
      pr_err("[LJ] %s: Must supply a hw reset function\n", __func__);
      return -ENODEV;
   }

   client->dev.platform_data = pdata;

   if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
   {
      pr_err("[LJ] %s: need I2C_FUNC_I2C\n", __func__);
      return -ENODEV;
   }

   ts = kzalloc(sizeof(struct qtouch_ts_data), GFP_KERNEL);
   if (ts == NULL)
   {
      err = -ENOMEM;
      goto err_alloc_data_failed;
   }
   else
   {
      qtouch_ts_ptr = ts;
      memset(qtouch_ts_ptr, 0, sizeof(struct qtouch_ts_data));
   }

   /* Set the voltage for touch IC VADD*/
   vreg_touch = vreg_get(NULL, VREG_TOUCH_VADD);
   if (IS_ERR(vreg_touch))
   {
      pr_err("[LJ] qtouch_ts_probe: vreg_get() failed\n");
      goto err_alloc_data_failed;
   }
   err = vreg_set_level(vreg_touch, 2600);
   if (err != 0)
   {
      pr_err("[LJ] qtouch_ts_probe: vreg_set_level() failed\n");
      goto err_vreg;
   }
   err = vreg_enable(vreg_touch);
   if (err != 0)
   {
      pr_err("[LJ] qtouch_ts_probe: vreg_enable() failed\n");
      goto err_vreg;
   }

   /* for test command - read position */
   memset(&tcmd_touch_data, 0, sizeof(struct coordinate_map));

   INIT_WORK(&ts->work, qtouch_ts_work_func);

   ts->pdata = pdata;
   ts->client = client;
   i2c_set_clientdata(client, ts);
   ts->checksum_cnt = 3;

   /* Request GPIO for touch reset pin */
   ts_printk("[LJ] %s: reset gpio = %d\n", __func__, TOUCH_RST_N);
   err = gpio_request(TOUCH_RST_N, "atmel_rst_gpio");
   if (err < 0)
   {
      pr_err("[LJ] gpio_request failed for input %d, err %d\n", TOUCH_RST_N, err);
      goto err_vreg;
   }
   err = gpio_direction_output(TOUCH_RST_N, 1);
   if (err < 0)
   {
      pr_err("[LJ] gpio_direction_input failed for input %d\n", TOUCH_RST_N);
      goto err_vreg;
   }

   ts->input_dev = input_allocate_device();
   if (ts->input_dev == NULL)
   {
      pr_err("[LJ] %s: failed to alloc input device\n", __func__);
      err = -ENOMEM;
      goto err_alloc_input_dev;
   }
   ts->input_dev->name = "qtouch-touchscreen";
   input_set_drvdata(ts->input_dev, ts);

   qtouch_force_reset(ts, 0);

   err = qtouch_process_info_block(ts);
   if (err != 0)
   {
      pr_info("[LJ] qtouch_ts_probe: fail to read info block\n");
      /* Check the ic is already in bootloader mode */
      app_i2c_address = ts->client->addr;
      ts->client->addr = bl_i2c_address;

      err = qtouch_read(ts, &buf, 1);
      if (err < 0)
      {
         pr_info("[LJ] %s: I2C read Error .. i2c addr = 0x%X err = %i\n",
                      __func__, ts->client->addr, err);
         ts->driver_state = MOT_TOUCH_BL_I2C_READ_ERROR;
         pr_info("[LJ] Resetting the interrupt GPIO back to normal\n");

         /* Update I2c address for Firmware address */
         ts->client->addr = app_i2c_address;
         ts->mode = MOT_TOUCH_MODE_UNKNOWN;

      }
      else
      {
         ts_printk("[LJ] already in bootloader mode\n");

         err = request_irq(ts->client->irq, qtouch_ts_irq_handler,
                           IRQ_DISABLED | pdata->irqflags, "qtouch_ts_int", ts);
         if (err != 0)
         {
            pr_err("[LJ] %s: request_irq (%d) failed\n", __func__,
                         ts->client->irq);
            goto err_request_irq;
         }

         ts->mode = MOT_TOUCH_MODE_CRASH;
         err = qtouch_lock_boot_state(ts);

         if (err < 0)
         {
            pr_info("[LJ] fail to unlock boot state\n");
            /* Update I2c address for Firmware address */
            ts->client->addr = app_i2c_address;
            ts->mode = MOT_TOUCH_MODE_UNKNOWN;
         }
         else
         {
            ts_printk("[LJ] success to unlock boot state\n");
            /* initializing Firmware update status
             *  as not done yet */
            ts->FwUpdate_Status = QTOUCH_FW_UPDATE_NOT_DONE;
            if (misc_register(&qtouch_pf_driver) != 0)
            {
               pr_info("misc_register failed\n");
               goto err_input_register_dev;
            }
            atomic_set(&qtouch_num_processes, 1);

            return 0;
         }
      }
      goto err_process_info_block;
   }
   else
   {
      ts->msg_buf = kmalloc(ts->msg_size, GFP_KERNEL);
      if (ts->msg_buf == NULL)
      {
         pr_err("[LJ] %s: Cannot allocate msg_buf\n", __func__);
         err = -ENOMEM;
         goto err_alloc_msg_buf;
      }

      /* Point the address pointer to the message processor.
       * Must do this before enabling interrupts */
      obj = find_obj(ts, QTM_OBJ_GEN_MSG_PROC);
      err = qtouch_set_addr(ts, obj->entry.addr);
      if (err != 0)
      {
         pr_err("[LJ] %s: Can't to set addr to msg processor\n", __func__);
         goto err_rst_addr_msg_proc;
      }

      set_bit(EV_SYN, ts->input_dev->evbit);

      /* register the harwdare assisted virtual keys, if any */
      obj = find_obj(ts, QTM_OBJ_TOUCH_KEYARRAY);
      if (obj && (obj->entry.num_inst > 0) &&
                 (pdata->flags & QTOUCH_USE_KEYARRAY))
      {
         for (i = 0; i < pdata->key_array.num_keys; ++i)
         {
            input_set_capability(ts->input_dev, EV_KEY,
                                 pdata->key_array.keys[i].code);
         }
      }

      /* register the software virtual keys, if any are provided */
      for (i = 0; i < pdata->vkeys.count; ++i)
      {
         input_set_capability(ts->input_dev, EV_KEY,
                              pdata->vkeys.keys[i].code);
      }

      obj = find_obj(ts, QTM_OBJ_TOUCH_MULTI);
      if (obj && obj->entry.num_inst > 0)
      {
         set_bit(EV_ABS, ts->input_dev->evbit);
         /* Legacy support for testing only */
         input_set_capability(ts->input_dev, EV_KEY, BTN_TOUCH);
         input_set_capability(ts->input_dev, EV_KEY, BTN_2);
         input_set_abs_params(ts->input_dev, ABS_X, pdata->abs_min_x,
                              pdata->abs_max_x, pdata->fuzz_x, 0);
         input_set_abs_params(ts->input_dev, ABS_HAT0X, pdata->abs_min_x,
                              pdata->abs_max_x, pdata->fuzz_x, 0);
         input_set_abs_params(ts->input_dev, ABS_Y, pdata->abs_min_y,
                              pdata->abs_max_y, pdata->fuzz_y, 0);
         input_set_abs_params(ts->input_dev, ABS_HAT0Y, pdata->abs_min_x,
                              pdata->abs_max_x, pdata->fuzz_x, 0);
         input_set_abs_params(ts->input_dev, ABS_PRESSURE, pdata->abs_min_p,
                              pdata->abs_max_p, pdata->fuzz_p, 0);
         input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, pdata->abs_min_w,
                              pdata->abs_max_w, pdata->fuzz_w, 0);

         /* multi touch */
         input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, pdata->abs_min_x,
                              pdata->abs_max_x, pdata->fuzz_x, 0);
         input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, pdata->abs_min_y,
                              pdata->abs_max_y, pdata->fuzz_y, 0);
         input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, pdata->abs_min_p,
                              pdata->abs_max_p, pdata->fuzz_p, 0);
         input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, pdata->abs_min_w,
                              pdata->abs_max_w, pdata->fuzz_w, 0);
      }

      if (obj && obj->entry.num_inst > 1)
      {
         if (pdata->flags & QTOUCH_USE_TINYTOUCH)
         {
            set_bit(EV_SYN, ts->input_dev->evbit);
            set_bit(EV_REL, ts->input_dev->evbit);
            set_bit(REL_X, ts->input_dev->relbit);
            set_bit(REL_Y, ts->input_dev->relbit);
            set_bit(BTN_MOUSE, ts->input_dev->keybit);
         }
      }

#if 0
      /* Register Touch OK Button */
      obj = find_obj(ts, QTM_OBJ_SPT_GPIO_PWM);
      if (obj && obj->entry.num_inst > 0)
      {
         set_bit(KEY_REPLY, ts->input_dev->keybit);
      }
#endif

      memset(&ts->finger_data[0], 0, (sizeof(struct coordinate_map) * _NUM_FINGERS));

      err = input_register_device(ts->input_dev);
      if (err != 0)
      {
         pr_err("[LJ] %s: Cannot register input device \"%s\"\n",
				 __func__, ts->input_dev->name);
         goto err_input_register_dev;
      }

      /* Request IRQ name. */
      ts_printk(" gpio_request for input %d\n", TOUCH_INT_N);
      err = gpio_request(TOUCH_INT_N, "atmel_int_gpio");
      if (err < 0)
      {
         ts_printk(" gpio_request failed for input %d, ret %d\n", TOUCH_INT_N, err);
         goto err_input_register_dev;
      }
      /* Set the direction of the Touch Interrupt line. */
      err = gpio_direction_input(TOUCH_INT_N);
      if (err < 0)
      {
         ts_printk(" gpio_direction_input failed for input %d\n", TOUCH_INT_N);
         goto err_input_register_dev;
      }
      ts->client->irq = gpio_to_irq(TOUCH_INT_N);
      /* set irq type Enable Falling Edge */
      set_irq_type(ts->client->irq, IRQ_TYPE_EDGE_FALLING);

      if (client->irq)
      {
         /* request irq */
         ts_printk(" qtouch_ts_probe - request_irq\n");
         err = request_irq(ts->client->irq, (void *)qtouch_ts_irq_handler,
		  IRQ_DISABLED, QTOUCH_TS_NAME, ts);
         /*err = request_irq(ts->client->irq, qtouch_ts_irq_handler,
		  IRQ_DISABLED | pdata->irqflags, "qtouch_ts_int", ts);*/
         if (err != 0)
         {
            free_irq(client->irq, ts);
            pr_err("[LJ] %s: request_irq (%d) failed\n", __func__, ts->client->irq);
            goto err_request_irq;
         }
      }

#ifdef CONFIG_HAS_EARLYSUSPEND
      ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
      ts->early_suspend.suspend = qtouch_ts_early_suspend;
      ts->early_suspend.resume = qtouch_ts_late_resume;
      register_early_suspend(&ts->early_suspend);
#endif

      /*initializing Firmware update status as not done yet */
      ts->FwUpdate_Status = QTOUCH_FW_UPDATE_NOT_DONE;
      ts->mode = MOT_TOUCH_MODE_NORMAL;
      if (misc_register(&qtouch_pf_driver) != 0)
      {
         pr_info("[LJ] qtm_obp_ts_probe: [os] misc_register failed: \n");
         goto err_input_register_dev;
      }
      atomic_set(&qtouch_num_processes, 1);

      qtouch_force_reset(ts, 0);

      ts_printk("[LJ] Exiting qtouch_ts_probe()\n");
      return 0;
   }
err_request_irq:
   input_unregister_device(ts->input_dev);

err_input_register_dev:
err_rst_addr_msg_proc:
   if (ts->msg_buf)
   kfree(ts->msg_buf);

err_alloc_msg_buf:
err_process_info_block:
   input_free_device(ts->input_dev);

err_alloc_input_dev:
   i2c_set_clientdata(client, NULL);

err_vreg:
   kfree(ts);
   err = vreg_disable(vreg_touch);

err_alloc_data_failed:
   pr_info("[LJ] Exiting qtouch_ts_probe() with err=%d\n", err);
   return err;
}

static int qtouch_ts_remove(struct i2c_client *client)
{
   struct qtouch_ts_data *ts = i2c_get_clientdata(client);

   ts_printk("[LJ] Entering qtouch_ts_remove()\n");

   unregister_early_suspend(&ts->early_suspend);
   free_irq(ts->client->irq, ts);
   input_unregister_device(ts->input_dev);
   input_free_device(ts->input_dev);
   i2c_set_clientdata(client, NULL);
   vreg_disable(vreg_touch);
   kfree(ts);
   ts_printk("[LJ] Exiting qtouch_ts_remove()\n");
   return 0;
}

static int qtouch_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
   struct qtouch_ts_data *ts = i2c_get_clientdata(client);
   int ret;

   ts_printk("[LJ] Entering qtouch_ts_suspend()\n");

   if (qtouch_tsdebug & 4)
   {
      pr_info("[LJ] %s: Suspending\n", __func__);
   }

   if (ts->mode == MOT_TOUCH_MODE_BOOTLOADER ||
           ts->mode == MOT_TOUCH_MODE_CRASH)
   {
      return -EBUSY;
   }

   disable_irq_nosync(ts->client->irq);
   ret = cancel_work_sync(&ts->work);
   if (ret)
   {
      pr_info("[LJ] %s: Not Suspending\n", __func__);
      enable_irq(ts->client->irq);
      return -EBUSY;
   }

   ret = qtouch_power_config(ts, 0);
   if (ret < 0)
   {
      pr_err("[LJ] %s: Cannot write power config\n", __func__);
   }

   ts_printk("[LJ] Exiting qtouch_ts_suspend()\n");
   return 0;
}

static int qtouch_ts_resume(struct i2c_client *client)
{
   struct qtouch_ts_data *ts = i2c_get_clientdata(client);
   int ret;
   int i;

   ts_printk("[LJ] Entering qtouch_ts_resume()\n");

   if (qtouch_tsdebug & 4)
   {
      pr_info("[LJ] %s: Resuming\n", __func__);
   }

   if (ts->mode == MOT_TOUCH_MODE_BOOTLOADER ||
		 ts->mode == MOT_TOUCH_MODE_CRASH)
   {
      return -EBUSY;
   }

   /* If we were suspended while a touch was happening
      we need to tell the upper layers so they do not hang
      waiting on the liftoff that will not come. */
   for (i = 0; i < ts->pdata->multi_touch_cfg.num_touch; i++)
   {
      if (qtouch_tsdebug & 4)
      {
         pr_info("[LJ] %s: Finger %i down state %i\n", __func__,
                     i, ts->finger_data[i].down);
      }
      if (ts->finger_data[i].down == 0)
      {
         continue;
      }
      input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
      input_mt_sync(ts->input_dev);
      memset(&ts->finger_data[i], 0,
      sizeof(struct coordinate_map));
   }
   input_sync(ts->input_dev);

   ret = qtouch_power_config(ts, 1);
   if (ret < 0)
   {
      pr_err("[LJ] %s: Cannot write power config\n", __func__);
      return -EIO;
   }
   qtouch_force_reset(ts, 0);

   //qtouch_force_calibration(ts);

   enable_irq(ts->client->irq);
   ts_printk("[LJ] Exiting qtouch_ts_resume()\n");
   return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qtouch_ts_early_suspend(struct early_suspend *handler)
{
   struct qtouch_ts_data *ts;

   ts = container_of(handler, struct qtouch_ts_data, early_suspend);
   qtouch_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void qtouch_ts_late_resume(struct early_suspend *handler)
{
   struct qtouch_ts_data *ts;

   ts = container_of(handler, struct qtouch_ts_data, early_suspend);
   qtouch_ts_resume(ts->client);
}
#endif

/******** init ********/
static const struct i2c_device_id qtouch_ts_id[] =
{
   { QTOUCH_TS_NAME, 0 },
   { }
};

static struct i2c_driver qtouch_ts_driver =
{
   .probe    = qtouch_ts_probe,
   .remove   = qtouch_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
   .suspend  = qtouch_ts_suspend,
   .resume   = qtouch_ts_resume,
#endif
   .id_table = qtouch_ts_id,
   .driver =
   {
      .name  = QTOUCH_TS_NAME,
      .owner = THIS_MODULE,
   },
};

static int __devinit qtouch_ts_init(void)
{
   qtouch_ts_wq = create_singlethread_workqueue("qtouch_obp_ts_wq");
   if (qtouch_ts_wq == NULL)
   {
      ts_printk("%s: No memory for qtouch_ts_wq\n", __func__);
      return -ENOMEM;
   }
   MXT224_DEBUG_ON = FALSE;
   return i2c_add_driver(&qtouch_ts_driver);
}

static void __exit qtouch_ts_exit(void)
{
   i2c_del_driver(&qtouch_ts_driver);
   if (qtouch_ts_wq)
   {
      destroy_workqueue(qtouch_ts_wq);
   }
}

static void ts_printk(char *fmt, ...)
{
   if (MXT224_DEBUG_ON == TRUE)
   {
      static va_list args;
      va_start(args, fmt);
      vprintk(fmt, args);
      va_end(args);
   }
}

module_init(qtouch_ts_init);
module_exit(qtouch_ts_exit);

MODULE_AUTHOR("Dima Zavin <dima@android.com>");
MODULE_DESCRIPTION("Quantum OBP Touchscreen Driver");
MODULE_LICENSE("GPL");
