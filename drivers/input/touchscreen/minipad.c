/* drivers/input/keyboard/minipad.c
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  
 * 02111-1307, USA
 *
 */

/*!
 * @file minipad.c
 * 
 * @ingroup key
 *
 * @brief Low-level minipad driver
 *
 * This driver is used to interface with the minipad IC.
 *
 */

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

#include "minipad.h"

/******************************************************************************
** Global Variables
*******************************************************************************/
/*! @brief Starting X position for the mouse */
static unsigned int x_start = 0;
/*! @brief Starting Y position for the mouse */
static unsigned int y_start = 0;
/*! @brief Indicates whether the mouse scroll was enabled */
static unsigned int tap_enabled = 0;
// first tap time in jiffies
static unsigned long touchdown_jiffies = 0;
// lasttime left pressed
static unsigned long lastleft_jiffies = 0;
// last x and y
static unsigned int x_last = 0xFFFF;
static unsigned int y_last = 0xFFFF;

/******************************************************************************
* Local Variables
******************************************************************************/
static struct minipad_data *ts;
static int    minipad_normal_addr;
static int    minipad_bl_addr;
static unsigned char    kernelBuffer[255];
static int lpValue = 32;

/******************************************************************************
* Local Functions prototypes
******************************************************************************/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void minipad_early_suspend(struct early_suspend *h);
static void minipad_late_resume(struct early_suspend *h);
#endif
static int minipad_disable(struct i2c_client *client);
static int minipad_enable(struct i2c_client *client);

static int minipad_remove(struct i2c_client *);
static int minipad_probe(struct i2c_client *client, const struct i2c_device_id *id);
static void minipad_process_mouse(struct minipad_data *ts,unsigned int x,unsigned int y,unsigned int touch);
static void minipad_process_rel_mouse(struct minipad_data *ts,unsigned int x,unsigned int y,unsigned int touch);
static int  minipad_read_app_version(void);
static int  minipad_reset(void);
static int  minipad_calibration(void);

static int minipad_reg_write (struct i2c_client *client, unsigned int reg, unsigned int length, unsigned char *reg_value);
static int minipad_reg_read(struct i2c_client *client , int reg );
static int minipad_reg_read16(struct i2c_client *client , int reg );

static int minipad_open(struct inode *inode, struct file *filp);
static int minipad_ioctl(struct inode *node, struct file *filp, unsigned int cmd, unsigned long arg);
static int minipad_release(struct inode *inode, struct file *filp);
static int minipad_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos );
static void minipad_log(char *fmt, ...);
static void minipad_log_critical(char *fmt, ...);

/******************************************************************************
* Local Macros
******************************************************************************/
#define MINIPAD_CHECK_NAME(name) ((name[0] == 'm')&&\
                                  (name[1] == 'i')&&\
                                  (name[2] == 'n')&&\
                                  (name[3] == 'i')&&\
                                  (name[4] == 'p')&&\
                                  (name[5] == 'a')&&\
                                  (name[6] == 'd'))

// 1 - zeppelin, 0 - motus/default
#define MINIPAD_GET_VERSION(name) ((!strcmp(name, "minipad_zeppelin")) ? 1 : 0)
/******************************************************************************
* Local Typedefs (STRUCTURES, UNIONS, ENUMS)
******************************************************************************/

typedef enum {
    MOUSE_UP = 0,
    MOUSE_DOWN,
    MOUSE_LEFT,
    MOUSE_RIGHT
}MINIPAD_MOUSE_MOVEMENT_T;

static const struct i2c_device_id minipad_id[] = {
	{ MINIPAD_I2C_NAME, 0 },
	{ }
};

// mouse logic parameters, these are the params that need to be different
// for different HW flavors. Other params are defined in minipad.h file
typedef struct {
    bool check_flip;
    bool swap_xy;
    bool invertx;
    bool inverty;
    int  mouse_th_x;
    int  mouse_th_y;
    bool send_relxy;
    bool use_lpValue;
}MINIPAD_LOGIC_PARAMS_T;

struct minipad_data {
    uint16_t addr;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct hrtimer timer;
    struct work_struct  work;
    char    mode;
    char    sendMode;
    int (*power)(int on);
    int     incalibration;
    int     isenabled;
    int     isresumed;
    int     isopened;
    MINIPAD_LOGIC_PARAMS_T logic_params;
    bool    isDataready;
    int     readadr;
    unsigned char readdata[6];
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
};

struct    file_operations    minipad_fops =
{
    .owner      = THIS_MODULE,
    .open       = minipad_open,
    .ioctl      = minipad_ioctl,
    .release    = minipad_release,
    .write      = minipad_write,
};
static struct miscdevice minipad_pf_driver = 
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = MINIPAD_PF_NAME,
    .fops = &minipad_fops,
};

static struct i2c_driver minipad_driver = {
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = minipad_suspend,
    .resume     = minipad_resume,
#endif
    .probe      = minipad_probe,
    .remove     = minipad_remove,
    .id_table	= minipad_id,
    .driver = {
    .name       = MINIPAD_I2C_NAME,
    },
};

/******************************************************************************
* minipad functions
******************************************************************************/

/*!
 * @brief This is the work func which creates rel xy mouse events
 *
 * This function implements the mouse function.  This function
 * will process mouse movements in the X,Y plane and produce the 
 * corresponding rel xy events.  Double Tap is a timed event that must
 * occur within the same area on the touch area.
 *
 *
 * @param ts     Touch screen structure pointer.
 * @param x      Raw X data from the IC
 * @param y      Raw Y data from the IC
 * @param touch  Indicates the status of the touch.
 *
 * @return None
 */
static void minipad_process_rel_mouse(struct minipad_data *ts, unsigned int x, 
                                      unsigned int y, unsigned int touch)
{
    int x_rel=0;
    int y_rel=0;
    unsigned long tap_jiffies = 0;
    long comp_jiffies = 0;
    int x_scaled_rel=0;
    int y_scaled_rel=0;    

    /* If we see a liftoff after a recorded mouse movement then reset the varieables */
    if((touch == 0) && (tap_enabled == 0))
    {
        minipad_log("mouse: >>> Bye\n");
        goto RESET;
    }

    x_rel = x - x_start;
    y_rel = y - y_start;

    minipad_log("mouse: x_start:%4i\t y_start:%4i\t x:%4i\t y:%4i\t REL_X:%4i\t REL_Y:%4i\n",
                        x_start, y_start, x, y, x_rel, y_rel);

    /* We see a liftoff but no recorded mouse movement so check for a tap */
    if((touch == 0) && (tap_enabled == 1))
    {
        minipad_log("mouse: >>> Liftoff with tap_enabled\n");

        // Get current jiffies
        tap_jiffies = jiffies;

        /* Check to see if the alotted time was there for a tap to be detected if the time was to long then
        reset the varieables and do not indicate a tap */
        comp_jiffies = (long)tap_jiffies - (long)touchdown_jiffies;
        minipad_log("mouse: >>> Tap time (%lu)->(%lu)=(%lu)\n", touchdown_jiffies, tap_jiffies, comp_jiffies);

        if( FALSE && //zeppelin has a physical key for select underneath the pad, tap is not needed
           (comp_jiffies >= MINIPAD_TAP_TIME_MIN_TH) && (comp_jiffies <= MINIPAD_TAP_TIME_MAX_TH) &&
           (abs(x_rel) <= MINIPAD_TAP_BOUND_X) &&
           (abs(y_rel) <= MINIPAD_TAP_BOUND_Y) &&
           (x >= (MINIPAD_MOUSE_MIN_X + MINIPAD_TAP_GUTTER_X)) && (x <= (MINIPAD_MOUSE_MAX_X - MINIPAD_TAP_GUTTER_X)) &&
           (y >= (MINIPAD_MOUSE_MIN_Y + MINIPAD_TAP_GUTTER_Y)) && (y <= (MINIPAD_MOUSE_MAX_Y - MINIPAD_TAP_GUTTER_Y))
          )
        {
            /* Touch event happened within the alotted time so send the select */
            minipad_log("mouse: Reporting MINIPAD_KEY_CENTER\n");
            /* Report key press */
            input_report_key(ts->input_dev, MINIPAD_KEY_CENTER, MINIPAD_KEY_PRESS);
            /* Report key release */
            input_report_key(ts->input_dev, MINIPAD_KEY_CENTER, MINIPAD_KEY_RELEASE);
        }
        else
        {
            minipad_log("mouse: Tap event violated constraints\n");
        }

        /* Reset the parameters */
        goto RESET;
    }
    else
    {
        if(((x_start == 0) && (y_start == 0)) && (touch == 1))
        {
            x_start = x;
            y_start = y;
            tap_enabled = 1;

            // Get touchdown jiffies
            touchdown_jiffies = jiffies;
            minipad_log("mouse: <<< Touchdown: x=%4i, y=%4i, t=(%lu)\n", x, y, touchdown_jiffies);
            return;
        }

        // Scale the values
        x_scaled_rel = ( x_rel * MINIPAD_MOUSE_ZEPPELIN_X_SCALE )/100;
        y_scaled_rel = ( y_rel * MINIPAD_MOUSE_ZEPPELIN_Y_SCALE )/100;

        // Send relative X and Y        
        if((abs(x_scaled_rel) > ts->logic_params.mouse_th_x) && (abs(y_scaled_rel) > ts->logic_params.mouse_th_y))
        {
            minipad_log("mouse: XY meet Mouse threshold met REL_X=%i REL_Y=%i REL_S_X=%i REL_S_Y=%i positions changed\n", 
                         x_rel, y_rel, x_scaled_rel, y_scaled_rel);
            input_report_rel(ts->input_dev, REL_X, x_scaled_rel);
            input_report_rel(ts->input_dev, REL_Y, y_scaled_rel);
            input_sync(ts->input_dev);

            x_start = x;
            y_start = y;
            tap_enabled = 0;
        }
        else if(abs(x_scaled_rel) > ts->logic_params.mouse_th_x)
        {
            minipad_log("mouse: X Mouse threshold met REL_X=%i REL_S_X=%i positions changed\n", x_rel, x_scaled_rel);
            input_report_rel(ts->input_dev, REL_X, x_scaled_rel);
            input_sync(ts->input_dev);

            x_start = x;
            tap_enabled = 0;
        }
        else if(abs(y_scaled_rel) > ts->logic_params.mouse_th_y)
        {
            minipad_log("mouse: Y Mouse threshold met REL_Y=%i REL_S_Y=%i positions changed\n", y_rel, y_scaled_rel);
            input_report_rel(ts->input_dev, REL_Y, y_scaled_rel);
            input_sync(ts->input_dev);

            y_start = y;
            tap_enabled = 0; 
        }
        else 
        {
             minipad_log("mouse: Changes arrived no threshold met REL_X=%i REL_Y=%i REL_S_X=%i REL_S_Y=%i\n",
                        x_rel, y_rel, x_scaled_rel, y_scaled_rel);
        }
    }/* elfi touch == 0*/

   return;

RESET:
    x_start = 0;
    y_start = 0;
    tap_enabled = 1;
    touchdown_jiffies = 0;
    lastleft_jiffies = 0;
    return;
}

/*!
 * @brief This is the work func which creates mouse events
 *
 * This function implements the mouse function.  This function
 * will process mouse movements in the X,Y plane and produce the 
 * corresponding key events.  Double Tap is a timed event that must
 * occur within the same area on the touch area.
 *
 *
 * @param ts     Touch screen structure pointer.
 * @param x      Raw X data from the IC
 * @param y      Raw Y data from the IC
 * @param touch  Indicates the status of the touch.
 *
 * @return None
 */
static void minipad_process_mouse(struct minipad_data *ts,unsigned int x,unsigned int y, unsigned int touch)
{
    int x_temp=0;
    int y_temp=0;
    unsigned int mouse_x_direction = 0;
    unsigned int mouse_y_direction = 0;
    unsigned long tap_jiffies = 0;
    long comp_jiffies = 0;

    /* If we see a liftoff after a recorded mouse movement then reset the varieables */
    if((touch == 0) && (tap_enabled == 0))
    {
        minipad_log("mouse: >>> Bye\n");
        goto RESET;
    }

    /* Check to for the math to not end up with a negative */
    if(x > x_start)
    {
        x_temp = x - x_start;
        mouse_x_direction =  MOUSE_RIGHT;
        minipad_log("mouse: R:x= %4i -> %4i = %4i\n",x_start ,x, x_temp);
    }
    else
    {
        x_temp = x_start - x;
        mouse_x_direction = MOUSE_LEFT;
        minipad_log("mouse: L:x= %4i -> %4i = %4i\n",x_start ,x, x_temp);
    }

    /* Check to for the math to not end up with a negative */
    if(y > y_start)
    {
        y_temp = y - y_start;
        mouse_y_direction = MOUSE_DOWN;
        minipad_log("mouse: D:y= %4i -> %4i = %4i\n",y_start ,y, y_temp);
    }
    else
    {
        y_temp = y_start - y;
        mouse_y_direction = MOUSE_UP;
        minipad_log("mouse: U:y= %4i -> %4i = %4i\n",y_start ,y, y_temp);
    }

    /* We see a liftoff but no recorded mouse movement so check for a tap */
    if((touch == 0) && (tap_enabled == 1))
    {
        minipad_log("mouse: >>> Liftoff with tap_enabled\n");

        /* First tap detected store the time and check for tap event*/
        //minipad_log("mouse: >>> first tap detected so take the time %i\n",taps_detected);

        // Get current jiffies
        tap_jiffies = jiffies;
        /* Check to see if the alotted time was there for a tap to be detected if the time was to long then
        reset the varieables and do not indicate a tap */
        comp_jiffies = (long)tap_jiffies - (long)touchdown_jiffies;
        minipad_log("mouse: >>> Tap time (%lu)->(%lu)=(%lu)\n", touchdown_jiffies, tap_jiffies, comp_jiffies);

        if(
           (comp_jiffies >= MINIPAD_TAP_TIME_MIN_TH) && (comp_jiffies <= MINIPAD_TAP_TIME_MAX_TH) && 
           (x_temp <= MINIPAD_TAP_BOUND_X) && 
           (y_temp <= MINIPAD_TAP_BOUND_Y) &&
           (x >= (MINIPAD_MOUSE_MIN_X + MINIPAD_TAP_GUTTER_X)) && (x <= (MINIPAD_MOUSE_MAX_X - MINIPAD_TAP_GUTTER_X)) &&
           (y >= (MINIPAD_MOUSE_MIN_Y + MINIPAD_TAP_GUTTER_Y)) && (y <= (MINIPAD_MOUSE_MAX_Y - MINIPAD_TAP_GUTTER_Y))
          )
        {
            /* Touch event happened within the alotted time so send the select */
            minipad_log("mouse: Reporting MINIPAD_KEY_CENTER\n");
            /* Report key press */
            input_report_key(ts->input_dev, MINIPAD_KEY_CENTER, MINIPAD_KEY_PRESS);
            /* Report key release */
            input_report_key(ts->input_dev, MINIPAD_KEY_CENTER, MINIPAD_KEY_RELEASE);
        }
        else
        {
            minipad_log("mouse: Tap event violated boundary\n");
        }

        /* Reset the parameters */
        goto RESET;
    }
    else
    {
        if(((x_start == 0) && (y_start == 0)) && (touch == 1))
        {
            x_start = x;
            y_start = y;
            tap_enabled = 1;

            // Get touchdown jiffies
            touchdown_jiffies = jiffies;
            minipad_log("mouse: <<< Touchdown: x=%4i, y=%4i, t=(%lu)\n", x, y, touchdown_jiffies);
            return;
        }

        /* Check the movement to see if it matches the threshold */
        if(x_temp > y_temp)
        {
            while (x_temp > ts->logic_params.mouse_th_x)
            {
                //minipad_log("mouse: X Mouse threshold met %i positions changed\n", x_temp);

                if(mouse_x_direction == MOUSE_LEFT)
                {
                    minipad_log("mouse: Reporting MINIPAD_KEY_LEFT\n");
                    /* Report key press */
                    input_report_key(ts->input_dev, MINIPAD_KEY_LEFT, MINIPAD_KEY_PRESS);
                    /* Report key release */
                    input_report_key(ts->input_dev, MINIPAD_KEY_LEFT, MINIPAD_KEY_RELEASE);
    
                    x_start = x_start - ts->logic_params.mouse_th_x;
                    if(x_start < MINIPAD_MOUSE_MIN_X)
                    {
                        x_start = MINIPAD_MOUSE_MIN_X;
                    }
                    x_temp = x_start - x;
                }
                else //if(mouse_x_direction == MOUSE_RIGHT)
                {
                    minipad_log("mouse: Reporting MINIPAD_KEY_RIGHT\n");
                    /* Report key press */
                    input_report_key(ts->input_dev, MINIPAD_KEY_RIGHT, MINIPAD_KEY_PRESS);
                    /* Report key release */
                    input_report_key(ts->input_dev, MINIPAD_KEY_RIGHT, MINIPAD_KEY_RELEASE);
    
                    x_start = x_start + ts->logic_params.mouse_th_x;
                    if(x_start > MINIPAD_MOUSE_MAX_X)
                    {
                        x_start = MINIPAD_MOUSE_MAX_X;
                    }
                    x_temp = x - x_start;

                }
                y_start = y;
                tap_enabled = 0;
            }
        }
        else // y_temp > x_temp
        {
            /* Check the movement to see if it matches the threshold */
            while (y_temp > ts->logic_params.mouse_th_y)
            {
                //minipad_log("mouse: Y Mouse threshold met %i positions changed\n", y_temp);

                if(mouse_y_direction == MOUSE_UP)
                {
                    minipad_log("mouse: Reporting MINIPAD_KEY_UP\n");
                    /* Report key press */
                    input_report_key(ts->input_dev, MINIPAD_KEY_UP, MINIPAD_KEY_PRESS);
                    /* Report key release */
                    input_report_key(ts->input_dev, MINIPAD_KEY_UP, MINIPAD_KEY_RELEASE);
    
                    y_start = y_start - ts->logic_params.mouse_th_y;
                    if(y_start < MINIPAD_MOUSE_MIN_Y)
                    {
                        y_start = MINIPAD_MOUSE_MIN_Y;
                    }
                    y_temp = y_start - y;

                    // BACK key support
                    /* Left has to be first movement event after touchdown */
                    if (tap_enabled)
                    {
                        lastleft_jiffies = jiffies;
                    }
                    // BACK key support
                }
                else //if(mouse_y_direction == MOUSE_DOWN)
                {
                    // BACK key support
                    /***
                    // Get current jiffies
                    tap_jiffies = jiffies;
                    comp_jiffies = (long)tap_jiffies - (long)lastleft_jiffies;
                    minipad_log("mouse: >>> LEFT-RIGHT time (%lu)->(%lu)=(%lu)\n", lastleft_jiffies, tap_jiffies, comp_jiffies);

                    if((lastleft_jiffies != 0) && (comp_jiffies <= 50))
                    {
                        minipad_log("mouse: Reporting KEY_BACK\n");
                        input_report_key(ts->input_dev, KEY_BACK, MINIPAD_KEY_PRESS);
                        input_report_key(ts->input_dev, KEY_BACK, MINIPAD_KEY_RELEASE);

                        x_start = x;
                        y_start = y;
                        lastleft_jiffies = 0;
                        return;
                    }
                    ***/
                    // BACK key support

                    minipad_log("mouse: Reporting MINIPAD_KEY_DOWN\n");
                    /* Report key press */
                    input_report_key(ts->input_dev, MINIPAD_KEY_DOWN, MINIPAD_KEY_PRESS);
                    /* Report key release */
                    input_report_key(ts->input_dev, MINIPAD_KEY_DOWN, MINIPAD_KEY_RELEASE);
    
                    y_start = y_start + ts->logic_params.mouse_th_y;
                    if(y_start > MINIPAD_MOUSE_MAX_Y)
                    {
                        y_start = MINIPAD_MOUSE_MAX_Y;
                    }
                    y_temp = y - y_start;
                }
                x_start = x;
                tap_enabled = 0;
            }
        }
    }/* elfi touch == 0*/

    return;

RESET:
    x_start = 0;
    y_start = 0;
    tap_enabled = 1;
    //tap_jiffies = 0;
    touchdown_jiffies = 0;
    lastleft_jiffies = 0;
    return;
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
static void minipad_work_func(struct work_struct *work)
{
    int ret;
    uint8_t buf[15];
    int    x,y,finger;
    uint8_t    msgCode;

#ifdef DEBUG_MINIPAD
    ktime_t    t1,t2;
#endif

    //minipad_log("minipad_work_func: Enter\n");   

    if ( ts->mode == MINIPAD_MODE_BOOTLOADER )
    {
        ret = i2c_master_recv(ts->client, buf,1);
        clk_busy_wait(Q5199225K03_WAIT);
        if (ret < 0)
        {
            /* Receive failed. Reset status  */
            minipad_log("minipad_work_func: i2c_master_recv failed\n");
            ts->sendMode = Q5199225K03_MODE_BL_PWR_ON;
        }
        else
        {
            minipad_log("minipad_work_func: buf[0] = 0x%x\n", buf[0]);

            /* Determine which code we got and set status appropriately */
            switch (buf[0])
            {
            case Q5199225K03_MODE_BL_PWR_ON:
                ts->sendMode = Q5199225K03_MODE_BL_PWR_ON;
                break;
            case Q5199225K03_MODE_BL_WAITING_FRAME:
                ts->sendMode = Q5199225K03_MODE_BL_WAITING_FRAME;
                break;
            case Q5199225K03_MODE_BL_FRAME_CRC_CHCK:
                ts->sendMode = Q5199225K03_MODE_BL_FRAME_CRC_CHCK;
                break;
            case Q5199225K03_MODE_BL_FRAME_CRC_FAIL:
                ts->sendMode = Q5199225K03_MODE_BL_FRAME_CRC_FAIL;
                break;
            case Q5199225K03_MODE_BL_FRAME_CRC_PASS:
                ts->sendMode = Q5199225K03_MODE_BL_FRAME_CRC_PASS;
                break;
            case Q5199225K03_MODE_BL_APP_CRC_FAIL:
                ts->sendMode = Q5199225K03_MODE_BL_APP_CRC_FAIL;
                break;
            default:
                ts->sendMode = Q5199225K03_MODE_BL_PWR_ON;
                break;
            }
        }
    }
    else
    {
        ts = container_of(work, struct minipad_data, work);

        // keep reading i2c messages untill GPIO goes back to high
        while(1)
        {

#ifdef DEBUG_MINIPAD
        t1 = ktime_get();
#endif
        /* Read the data from the device (retrying if necessary) */
        ret = i2c_master_recv(ts->client, buf, 6);

#ifdef DEBUG_MINIPAD
        t2 = ktime_get();
        //minipad_log("minipad_work: i2c_master_recv: %lld nsecs\n", ktime_to_ns(t2) - ktime_to_ns(t1));
#endif

        if (ret < 0) 
        {
            minipad_log_critical("minipad_work_func: ERROR i2c_master_recv ret=%d\n", ret);
            clk_busy_wait(Q5199225K03_WAIT);
            goto EXIT;
        }

        // debug
        minipad_log("minipad_robot: %02x %02x %02x %02x %02x %02x\n", 
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

        if(buf[0] & Q5199225K03_DATA_MSG)
        {
            minipad_log("minipad_work_func: Data message. Adr(%d)\n", buf[1]*4);
            if((buf[5] == 0xff) || (buf[5] == 0xa9))
            {
                minipad_log_critical("minipad_work_func: ERROR garbage - read\n");
                ret = i2c_master_recv(ts->client, buf, 6);
            }
            else if(ts->readadr == buf[1])
            {
                ts->readdata[0] = buf[0];
                ts->readdata[1] = buf[1];
                ts->readdata[2] = buf[2];
                ts->readdata[3] = buf[3];
                ts->readdata[4] = buf[4];
                ts->readdata[5] = buf[5];
                ts->isDataready = TRUE;
            }
            goto EXIT;
        }

        /* Find out what type of message occurred and handle it */
        msgCode = (buf[0]>>4) & 0x07;
        //minipad_log("minipad_work_func: Arrived message. type: %d\n",msgCode);
        switch (msgCode)
        {
        case Q5199225K03_10BIT_TSP_MSG:
            //minipad_log("minipad_work_func: Processing 10bit message. ");

            /* For now just register 1 touch to the framework */ 
            if (buf[Q5199225K03_MSG_OBJ_FLAG_BYTE] == 0x41)
            {
                // check garbage data
                if((buf[Q5199225K03_MSG_PTR10_DATA_XH_BYTE] & 0xfc) || (buf[Q5199225K03_MSG_PTR10_DATA_YH_BYTE] & 0xfc))
                {
                    minipad_log_critical("minipad_work_func: ERROR garbage - read\n");
                    ret = i2c_master_recv(ts->client, buf, 6);
                    goto EXIT;
                }

                // check calibration status, ignore touch event when its ongoing
                if(ts->incalibration)
                {
                    minipad_log_critical("minipad_work_func: WARNING ignoring in calibration\n");
                    goto EXIT;
                }

                // discard touch events when disabled
                if(!ts->isenabled)
                {
                    minipad_log("minipad_work_func: discarding message\n");
                    goto EXIT;
                }

                finger = (buf[Q5199225K03_MSG_OBJ_FLAG_BYTE] & Q5199225K03_MSG_OBJ_FLAG_MASK);

                /* X and Y are flipped on the sensor panel so manipulate the data so
                   X data is Y and Y data is reported as X */
                /* Swap the x y data due to sensor orientation */
                if(ts->logic_params.swap_xy)
                {
                    y = buf[Q5199225K03_MSG_PTR10_DATA_XL_BYTE];
                    y |= ((buf[Q5199225K03_MSG_PTR10_DATA_XH_BYTE] & 0x3) << 8);
                            
                    x = buf[Q5199225K03_MSG_PTR10_DATA_YL_BYTE];
                    x |= ((buf[Q5199225K03_MSG_PTR10_DATA_YH_BYTE] & 0x3) << 8);
                }
                else
                {
                    x = buf[Q5199225K03_MSG_PTR10_DATA_XL_BYTE];
                    x |= ((buf[Q5199225K03_MSG_PTR10_DATA_XH_BYTE] & 0x3) << 8);
                            
                    y = buf[Q5199225K03_MSG_PTR10_DATA_YL_BYTE];
                    y |= ((buf[Q5199225K03_MSG_PTR10_DATA_YH_BYTE] & 0x3) << 8);
                }

                /**
                z = ((buf[Q5199225K03_MSG_PTR10_DATA_XH_BYTE] & 0x0C) << 4);
                z |= ((buf[Q5199225K03_MSG_PTR10_DATA_YH_BYTE] & 0xFC) >> 2);
                
                w = ((buf[Q5199225K03_MSG_PTR10_DATA_XH_BYTE] & 0xF0)>>4);
                **/

                /* Reverse the x or y value based on sensor to screen layout */
                if(ts->logic_params.invertx)
                {
                    x = MINIPAD_MOUSE_MAX_X - x;
                }            
                if(ts->logic_params.inverty)
                {
                    y = MINIPAD_MOUSE_MAX_Y - y;
                }

                //minipad_log("minipad_work_func: x=%d y=%d touch=%d\n", x, y,(buf[Q5199225K03_MSG_TOUCH_FLAG_BYTE] & finger));
                if(ts->logic_params.send_relxy)
                {
                    minipad_process_rel_mouse(ts, x, y, (buf[Q5199225K03_MSG_TOUCH_FLAG_BYTE] & finger));
                }
                else //Send up/down/left/right key events
                {
                    minipad_process_mouse(ts, x, y, (buf[Q5199225K03_MSG_TOUCH_FLAG_BYTE] & finger));
                }
                
                // store for tcmd
                if(buf[Q5199225K03_MSG_TOUCH_FLAG_BYTE] & finger)
                {
                    x_last = x;
                    y_last = y;
                }
                else
                {
                    x_last = 0xFFFF;
                    y_last = 0xFFFF;
                }
            }
            break;

        case Q5199225K03_STATUS_MSG:
            minipad_log("minipad_work_func: Status=0x%02x\n", buf[Q5199225K03_MSG_STATUS_FLAG_BYTE]);
            // check garbage data
            if(buf[5] == 0xff)
            {
                minipad_log_critical("minipad_work_func: ERROR garbage - read\n");
                ret = i2c_master_recv(ts->client, buf, 6);
                goto EXIT;
            }
            if(buf[Q5199225K03_MSG_STATUS_FLAG_BYTE] &0x10)
            {
                minipad_log_critical("minipad_work_func: calibration = 1\n");
                ts->incalibration = 1;
            }
            else
            {
                minipad_log_critical("minipad_work_func: calibration = 0\n");
                ts->incalibration = 0;
            }
            if(buf[Q5199225K03_MSG_STATUS_FLAG_BYTE] &0x40)
            {
                minipad_log_critical("minipad_work_func: overrun error\n");
                minipad_log_critical("minipad_robot: %02x %02x %02x %02x %02x %02x\n", 
                        buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
                // this indicates a overrun event has occured and if this happens we should
                // recalibrate the IC, wait 30ms may be we were reset
                clk_busy_wait(Q5199225K03_WAIT*10);
                minipad_calibration();
            }
            break;

        case Q5199225K03_KEY_MSG:
            minipad_log("minipad_work_func: Processing key message. \n");
            break;
        case Q5199225K03_SLIDER_MSG:
            minipad_log("minipad_work_func: Processing slider message. \n");
            break;
        case Q5199225K03_OMEGA_MSG:
            minipad_log("minipad_work_func: Processing omega message. \n");
            break;
        case Q5199225K03_8BIT_TSP_MSG:
            minipad_log("minipad_work_func: Processing 8bit message. \n");
            break;
        case Q5199225K03_READ_MSG:
            minipad_log("minipad_work_func: Processing read message. \n");
            break;
        default:
            minipad_log("minipad_work_func: Processing unknown message.\n");
            break;
        }

EXIT:
        // check GPIO status
        if(gpio_get_value(MINIPAD_TS_GPIO) > 0)
            break;
        }// while(1)
    }

    enable_irq(ts->client->irq);
    //minipad_log("minipad_work_func: enable_irq = %d\n", ts->client->irq);

#ifdef DEBUG_MINIPAD
    t2 = ktime_get();
    //minipad_log("minipad_work_func: final total %lld\n", ktime_to_ns(t2) - ktime_to_ns(t1));
    //minipad_log("minipad_work_func: Exit\n");   
#endif

    return;
}

/*!
 * @brief This is the timer function for the power management
 *
 * This function implements a timer for the power management functions
 *
 *
 * @param work     Work structure pointer
 *
 * @return hrtimer_restart -     HRTIMER_NORESTART - Timer is not restarted
 *                HRTIMER_RESTART - Timer must be restarted
 */
/*
static enum hrtimer_restart minipad_timer_func(struct hrtimer *timer)
{
    struct minipad_data *ts = container_of(timer, struct minipad_data, timer);
    schedule_work(&ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, 1250000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}
*/

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
static irqreturn_t minipad_irq_handler(int irq, void *dev_id)
{
    struct minipad_data *ts = dev_id;
    disable_irq(ts->client->irq);
    minipad_log("minipad_irq_handler\n");
    schedule_work(&ts->work);
    return IRQ_HANDLED;
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
static int minipad_reg_write (struct i2c_client *client, unsigned int reg, unsigned int length, unsigned char *reg_value)
{
    unsigned char value[24];
    int retval = 0;
    int i = 10;

    //minipad_log("minipad_reg_write: Enter\n");

    /* Copy the data into a buffer for correct format */
    value[0] = reg & 0xFF;
    value[1] = reg >> 8;
    memcpy(&value[2], reg_value, length);

    //minipad_log("minipad_reg_write: memcpy(): OK\n");

    /* Write the data to the device (retrying if necessary) */
    do
    {
        minipad_log("minipad_reg_write: reg=%03d, data=0x%02x\n", reg, reg_value[0]);

        retval = i2c_master_send (client, value, Q5199225K03_REG_ADDR_SIZE + length);

        /* On failure, output the error code and delay before trying again */
        if (retval < 0)
        {
            minipad_log_critical("minipad_reg_write: ERROR i2c_master_send reg=0x%X ret=%d\n", reg, retval);
            clk_busy_wait(Q5199225K03_WAIT);
        }
        else
        {
            //minipad_log("minipad_reg_write: i2c_master_send(): OK\n");
        }
    }
    while ((retval < 0) && (i-- >= 0));

    //minipad_log("minipad_reg_write: i2c_master_send(): Done\n");

    /* On success, set retval to 0 (i2c_master_send returns no. of bytes transfered) */
    if (retval == (Q5199225K03_REG_ADDR_SIZE + length))
    {
        retval = 0;
    }

    /* Delay after every I2C access or IC will NAK */
    clk_busy_wait(Q5199225K03_WAIT);

    //minipad_log("minipad_reg_write: Exit\n");
    return retval;
}

static int minipad_reg_read(struct i2c_client *client , int reg )
{
    uint8_t data[6];
    int   rc=-1;
    int i;

    minipad_log("minipad_reg_read: Enter\n");

    ts->readadr = (reg>>2) & 0xff;
    ts->isDataready = FALSE;

    // send
    data[0] = Q5199225K03_REG_READ_ADDR;  //read register flag
    data[1] = 0;   // register id
    data[2] = (uint8_t)(ts->readadr);  //read address pointer

    for(i=0; i<10; i++)
    {
        if ((i2c_master_send(client, data, 3)) >= 0)
        {
            rc = 1;
            break;
        }
        //minipad_log("minipad_reg_read: ERROR i2c_master_send reg=%d\n,", reg);
        clk_busy_wait(Q5199225K03_WAIT);
    }

    if (rc < 0)
    {
        minipad_log_critical("minipad_reg_read: ERROR i2c_master_send timeout\n");
        goto EXIT;
    }

    // recv
    rc = -1;
    for(i=0; i<100; i++)
    {
        clk_busy_wait(Q5199225K03_WAIT);

        if(ts->isDataready)
        {
            // copy data from work_func
            data[0] = ts->readdata[0];
            data[1] = ts->readdata[1];
            data[2] = ts->readdata[2];
            data[3] = ts->readdata[3];
            data[4] = ts->readdata[4];
            data[5] = ts->readdata[5];
            // debug
            minipad_log("minipad_reg_read: %02x %02x %02x %02x %02x %02x\n", 
                data[0], data[1], data[2], data[3], data[4], data[5]);
            rc = 1;
            break;
        }
    }

    if ( rc > 0 )
    {
        minipad_log("minipad_reg_read: done\n");
        rc = data[2+(reg%4)];
    }
    else
    {
        minipad_log_critical("minipad_reg_read: ERROR i2c_master_recv timeout\n");
    }

EXIT:
    return rc;
}

static int minipad_reg_read16(struct i2c_client *client , int reg )
{
    uint8_t data[6];
    int   rc=-1;
    int i;

    minipad_log("minipad_reg_read16: Enter\n");

    // make sure reg is even address
    reg &= 0xfffe;
    ts->isenabled = 0;
    ts->readadr = (reg>>2) & 0xff;
    ts->isDataready = FALSE;

    // send
    data[0] = Q5199225K03_REG_READ_ADDR;  //read register flag
    data[1] = 0;   // register id
    data[2] = (uint8_t)(ts->readadr);  //read address pointer

    for(i=0; i<10; i++)
    {
        if ((i2c_master_send(client, data, 3)) >= 0)
        {
            rc = 1;
            break;
        }
        //minipad_log("minipad_reg_read16: ERROR i2c_master_send reg=%d\n,", reg);
        clk_busy_wait(Q5199225K03_WAIT);
    }

    if (rc < 0)
    {
        minipad_log_critical("minipad_reg_read16: ERROR i2c_master_send timeout\n");
        goto EXIT;
    }

    // recv
    rc = -1;
    for(i=0; i<100; i++)
    {
        clk_busy_wait(Q5199225K03_WAIT);

        if(ts->isDataready)
        {
            // copy data from work_func
            data[0] = ts->readdata[0];
            data[1] = ts->readdata[1];
            data[2] = ts->readdata[2];
            data[3] = ts->readdata[3];
            data[4] = ts->readdata[4];
            data[5] = ts->readdata[5];
            // debug
            minipad_log("minipad_reg_read16: %02x %02x %02x %02x %02x %02x\n", 
                data[0], data[1], data[2], data[3], data[4], data[5]);
            rc = 1;
            break;
        }
    }

    if ( rc > 0 )
    {
        // return 16 bit value
        minipad_log("minipad_reg_read16: done\n");
        rc = (data[2+(reg%4)]<<8) | (data[2+(reg%4)+1]);
    }
    else
    {
        minipad_log_critical("minipad_reg_read16: ERROR i2c_master_recv timeout\n");
    }

EXIT:
    ts->isenabled = 1;
    return rc;
}

static int  minipad_reset(void)
{
    struct    i2c_client *client;
    char data[6];
    int    rc=0;

    minipad_log_critical("minipad_reset: enter\n");
    client = ts->client;
    if ( ts->mode == MINIPAD_MODE_NORMAL )
    {
        data[0] = 0x01; 
        rc = minipad_reg_write(client , Q5199225K03_RESET_ADDR, 1, data);
    }

    minipad_log_critical("minipad_reset: done\n");
    return(rc);
}

static int  minipad_read_app_version(void)
{
    struct    i2c_client *client;
    int    rc=0;

    if (ts->isenabled < 0)
    {
        minipad_log_critical("minipad_read_app_version: ERROR not connected (err=%d)\n", ts->isenabled);
        return ts->isenabled;
    }

    client = ts->client;
    if ( ts->mode == MINIPAD_MODE_NORMAL )
    {
        rc = minipad_reg_read(client , Q5199225K03_CODE_VERSION_ADDR);
        minipad_log_critical("minipad_read_app_version: =%d (=0x%02x)\n", rc, rc);
    }

    return(rc);
}

static int  minipad_calibration(void)
{
    struct    i2c_client *client;
    char data[6];
    int rc=0;

    client = ts->client;
    if ( ts->mode == MINIPAD_MODE_NORMAL )
    {
        // do calibraton
        data[0] = 0x01; 
        rc = minipad_reg_write (ts->client,
            Q5199225K03_CALIBRATE_ADDR,
            1, data);
        clk_busy_wait(Q5199225K03_WAIT);
        clk_busy_wait(Q5199225K03_WAIT);
    }

    minipad_log_critical("minipad_calibration: done\n");
    return rc;
}

static void minipad_logic_params_init(int version)
{
    if(version == 1) //zeppelin
    {
        ts->logic_params.check_flip = false;
        ts->logic_params.swap_xy    = false;
        ts->logic_params.invertx    = false;
        ts->logic_params.inverty    = true;
        ts->logic_params.mouse_th_x = MINIPAD_MOUSE_ZEPPELIN_TH_X;
        ts->logic_params.mouse_th_y = MINIPAD_MOUSE_ZEPPELIN_TH_Y;
        ts->logic_params.send_relxy = true;
        ts->logic_params.use_lpValue= true;
    }
    else //motus/default
    {
        ts->logic_params.check_flip = true;
        ts->logic_params.swap_xy    = true;
        ts->logic_params.invertx    = false;
        ts->logic_params.inverty    = false;
        ts->logic_params.mouse_th_x = MINIPAD_MOUSE_MOTUS_TH_X;
        ts->logic_params.mouse_th_y = MINIPAD_MOUSE_MOTUS_TH_Y;
        ts->logic_params.send_relxy = false;
        ts->logic_params.use_lpValue= false;
    }
}

/*!
 * @brief This is the work func which services the incoming work queue
 *
 * This function implements a write to the IC to configure the sensor
 * register to intialize the sensor to a known state
 *
 *
 * @param adapter     I2C adapter pointer
 *
 * @return int 
 */
static int minipad_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct platform_device *pdev;
    //unsigned long request_flags =  IRQ_TYPE_EDGE_FALLING;
    unsigned long request_flags =  IRQ_TYPE_LEVEL_LOW;
    int ret = 0;

    minipad_log_critical("minipad_probe: Enter\n");

    // register the driver as a mic device
    ret = misc_register(&minipad_pf_driver);
    minipad_log_critical("minipad_probe: misc_register() returns %d\n", ret);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        minipad_log_critical("minipad_probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }

    // alloc memory
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (ts == NULL) 
    {
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }

    /* Get GPIO pin number allocation */
    INIT_WORK(&ts->work, minipad_work_func);

    ts->client = client;
    i2c_set_clientdata(client, ts);
    client->flags = 0;

    // default
    minipad_normal_addr = MINIPAD_I2C_ADDR;
    minipad_bl_addr = MINIPAD_BL_I2C_ADDR;

    // ok
    pdev = (struct platform_device *)client->dev.platform_data;
    // device name should be minipad and then "_productname" if any
    if ( pdev && pdev->num_resources && MINIPAD_CHECK_NAME(pdev->name) )
    {
        struct    resource *resource;
        minipad_log_critical("minipad_probe: paltform data for '%s', num_resources %d\n",
            pdev->name, pdev->num_resources);

        resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        if ( resource)
        {
            minipad_normal_addr = resource->start;
            minipad_bl_addr = resource->end;
            minipad_log_critical("minipad_probe: Got resource. Normal Addr: 0x%x, Bootloader: 0x%x\n",
                minipad_normal_addr, minipad_bl_addr);
        }
        else
        {
            minipad_log_critical("minipad_probe: Could not optain GPIO information. Using default values\n");
        }
    } 
    else
    {
        minipad_log_critical("minipad_probe: Could not get platform reousrces. Using default values\n");
    }

    minipad_log_critical("minipad_probe: attached, client: 0x%x\n", (unsigned)client);
    minipad_log_critical("minipad_probe: irq: 0x%x\n", client->irq);

    if (ts->power) 
    {
        ret = ts->power(1);
        if (ret < 0) 
        {
            minipad_log_critical("minipad_probe power on failed\n");
            goto err_power_failed;
        }
    }

    // config the dynamic params for the driver
    minipad_logic_params_init(MINIPAD_GET_VERSION(pdev->name));

    ts->mode = MINIPAD_MODE_NORMAL;
    ts->client->addr = minipad_normal_addr;
    ts->sendMode = Q5199225K03_MODE_BL_PWR_ON;
    ts->isresumed = 1;

    // ok
    ts->input_dev = input_allocate_device();
    if (ts->input_dev == NULL) 
    {
        ret = -ENOMEM;
        minipad_log_critical("minipad_probe: Failed to allocate input device\n");
        goto err_input_dev_alloc_failed;
    }
    ts->input_dev->name = "minipad";

    // set device capability bits
    if(ts->logic_params.send_relxy)
    {
        set_bit(EV_SYN, ts->input_dev->evbit);
        set_bit(EV_REL, ts->input_dev->evbit);
        set_bit(REL_X, ts->input_dev->relbit);
        set_bit(REL_Y, ts->input_dev->relbit);
        set_bit(BTN_MOUSE, ts->input_dev->keybit);
    }
    else
    {
        set_bit(EV_SYN, ts->input_dev->evbit);
        set_bit(EV_KEY, ts->input_dev->evbit);
        set_bit(MINIPAD_KEY_UP, ts->input_dev->keybit);
        set_bit(MINIPAD_KEY_DOWN, ts->input_dev->keybit);
        set_bit(MINIPAD_KEY_RIGHT, ts->input_dev->keybit);
        set_bit(MINIPAD_KEY_LEFT, ts->input_dev->keybit);
        set_bit(MINIPAD_KEY_CENTER, ts->input_dev->keybit);
        //set_bit(KEY_BACK, ts->input_dev->keybit);
    }
    
    // ok
    ret = input_register_device(ts->input_dev);
    if (ret) 
    {
        minipad_log_critical("minipad_probe: Unable to register %s input device\n", ts->input_dev->name);
        goto err_input_register_device_failed;
    }

    /* Request IRQ name. */     
    minipad_log_critical("minipad_probe Gpio request\n");
    ret = gpio_request(MINIPAD_TS_GPIO, "minipad_gpio");
    if (ret) 
    {
        minipad_log_critical(" gpio_request failed for input %d\n", MINIPAD_TS_GPIO);
        goto err_input_register_device_failed;
    }

     /* Set the direction of the Touch Interrupt line. */     
    ret = gpio_direction_input(MINIPAD_TS_GPIO);
    if (ret) 
    {
        minipad_log_critical(" gpio_direction_input failed for input %d\n", MINIPAD_TS_GPIO);
        goto err_input_register_device_failed;
    }

    /* set irq type Enable Falling Edge */
    client->irq = gpio_to_irq(MINIPAD_TS_GPIO);
    // minipad_log(" Input device %s , Irq %d  , dname %s\n", kp->input_dev->name,client->irq,client->driver_name);
    ret = gpio_direction_input(MINIPAD_TS_GPIO);
    minipad_log_critical("minipad_probe: ret from gpio_direction_input = %d\n", ret);
    minipad_log_critical("minipad_probe: gpio_request MINIPAD_TS_GPIO =  %d\n", gpio_get_value(MINIPAD_TS_GPIO));

    // ok
    if (client->irq) 
    {    
        /* request irq */
        ret = request_irq(client->irq, minipad_irq_handler, request_flags , pdev->name, ts);
        if (ret != 0)
        {
            minipad_log_critical("minipad_probe: ERROR request_irq failed ret=%d\n", ret);
            free_irq(client->irq, ts);
            dev_err(&client->dev, "request_irq failed\n");
        }
    }

    // ok
#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = minipad_early_suspend;
    ts->early_suspend.resume = minipad_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif
    ts->incalibration = 0;

    // sw reset IC
    ret = minipad_reset();
    if(ret)
    {
        printk("minipad: SW=%s ERROR device reset failed (err=%d)\n", MINIPAD_VERSION, ret);
        ts->isenabled = -1;
        return ret;
    }

    // wait 30ms
    clk_busy_wait(Q5199225K03_WAIT*10);

    // read chip id - this value should be 0x21
    ret = minipad_reg_read(client, Q5199225K03_CHIPID_ADDR);
    if (ret == Q5199225K03_CHIP_ID)
    {
        minipad_log_critical("minipad_probe: verified Chip ID =0x%02x\n", ret);
    }
    else if (ret == Q5199225K03_CHIP_ID_0X23)
    {
        // device reported incorrect chip ID. minipadut will automatically upgrade firmware
        minipad_log_critical("minipad_probe: WARNING - incorrect Chip ID =0x%02x\n", ret);
    }
    else if (ret == Q5199225K03_MODE_BL_APP_CRC_FAIL)
    {
        // device is in app crc fail. move to bootloader mode and wait for firware update
        minipad_log_critical("minipad_probe: ERROR - Device CRC error =0x%02x\n", ret);
    }
    else
    {   
        printk("minipad: SW=%s ERROR invalid chip ID (err=%d)\n", MINIPAD_VERSION, ret);
        ts->isenabled = -1;
        return ret;
    }

    // read product code version
    ret = minipad_read_app_version();
    printk("minipad: SW=%s FW=0x%02x\n", MINIPAD_VERSION, ret);

    // detect initial flip status
    if(!(ts->logic_params.check_flip))
    {
        // Zeppelin: no-flip
        minipad_log("minipad_probe: flip opened\n");
        ts->isenabled = 0;
        minipad_enable(ts->client);
        ts->isopened = 1;
    }
    else
    {
        // calibration
        minipad_calibration();
        
        // Motus: always power up as flip closed
        minipad_log("minipad_probe: flip closed\n");
        ts->isenabled = 1;
        minipad_disable(ts->client);
        ts->isopened = 0;
    }

    minipad_log(KERN_INFO "minipad_probe: Start (%s)\n", ts->input_dev->name);
    minipad_log_critical("minipad_probe: Exit normally\n");
    return 0;

err_input_register_device_failed:
    input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_power_failed:
    kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
    i2c_detach_client(client);
    minipad_log_critical("minipad_probe: Exit after error\n");
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
static int minipad_remove(struct i2c_client *client)
{
    struct minipad_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif
    free_irq(client->irq, ts);
    input_unregister_device(ts->input_dev);
    kfree(ts);
    return 0;
}

/*!
 * @brief This is the function called during system suspend
 *
 * This function implements the suspend functionality for the minipad
 * driver.
 *
 *
 * @param client    I2C client pointer
 * @param mesg      Power management message
 *
 * @return int      Always returns 0
 */
static int minipad_suspend(struct i2c_client *client, pm_message_t mesg)
{
    minipad_log("minipad_suspend: Entering\n");
    ts->isresumed = 0;
    minipad_disable(client);
    minipad_log("minipad_suspend: Exiting\n");
    return 0;
}

static int minipad_disable(struct i2c_client *client)
{
    int     ret;
    char    regValue;

    minipad_log("minipad_disable: Entering\n");

    // do not suspend when bootloader mode
    if ( ts->mode != MINIPAD_MODE_NORMAL ) goto EXIT;

    // immeiately exit if already turned off
    if(!ts->isenabled)
    {
        minipad_log("minipad_disable: already disabled\n");
    }
    ts->isenabled = 0;

    // disable irq
    //disable_irq(ts->client->irq);
    /*
    ret = cancel_work_sync(&ts->work);
    if (ret < 0)
        minipad_log("minipad_disable: i2c_smbus_write_byte_data failed\n");
    */

    // move to ultra low power mode
    regValue = 0x0; 
    ret = minipad_reg_write (ts->client,
        Q5199225K03_LP_MODE_ADDR,
        sizeof(char), (char *)&regValue);

    if (ts->power) 
    {
        ret = ts->power(0);
        if (ret < 0) 
        {
            minipad_log("minipad_disable: power off failed\n");
        }
    }

EXIT:
    minipad_log_critical("minipad_disable: Exiting\n");
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
static int minipad_resume(struct i2c_client *client)
{
    minipad_log("minipad_resume: Entering\n");

    ts->isresumed = 1;
    if(ts->isopened)
    {
        // enable only when flip is opened
        minipad_enable(client);
    }
    else
    {
        minipad_log("minipad_resume: ignoring since flip is closed\n");
    }

    minipad_log("minipad_resume: Exiting\n");
    return 0;
}

static int minipad_enable(struct i2c_client *client)
{
    int ret;
    char    regValue;
    int i;
    unsigned char buf[6];

    minipad_log("minipad_enable: Entering\n");

    // do not resume when bootloader mode
    if ( ts->mode != MINIPAD_MODE_NORMAL ) goto EXIT;

    // immeiately exit if already enabled
    if(ts->isenabled)
    {
        minipad_log("minipad_enable: WARNING already enabled\n");
    }

    if (ts->power) 
    {
        ret = ts->power(1);
        if (ret < 0) 
        {
            minipad_log("minipad_enable power on failed\n");
        }
    }

    // restore ultra low power mode
/*
    minipad_log("minipad_enable: restoring LP mode\n");
    regValue = 12;
    ret = minipad_reg_write (ts->client,
        Q5199225K03_LP_MODE_ADDR,
        sizeof(char), (char *)&regValue);    
    clk_busy_wait(Q5199225K03_WAIT);
    clk_busy_wait(Q5199225K03_WAIT);
    clk_busy_wait(Q5199225K03_WAIT);
    clk_busy_wait(Q5199225K03_WAIT);
*/

    // reset
    minipad_log("minipad_enable: reset ic\n");
    ret = minipad_reset();

    // wait 30ms
    clk_busy_wait(Q5199225K03_WAIT*10);

    // calibratoin
    minipad_log("minipad_enable: calibrating\n");
    minipad_calibration();

    // flush buffer and check GPIO line
    for(i=0; i<1000; i++)
    {
        ret = i2c_master_recv(ts->client, buf, 6);
        // debug
        minipad_log("minipad_enable: %02x %02x %02x %02x %02x %02x\n", 
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

        if(gpio_get_value(MINIPAD_TS_GPIO) > 0)
        {
            minipad_log("minipad_enable: GPIO going back\n");
            break;
        }

        minipad_log("minipad_enable: flushing buffer %d\n", i);
        clk_busy_wait(Q5199225K03_WAIT);
    }

    // enable irq
    //enable_irq(ts->client->irq);

    ts->isenabled = 1;

    // this is for zeppelin p1b specifically
    if( (ts->logic_params.use_lpValue == true) && (lpValue == 0) )
    {
        minipad_log_critical("minipad_enable re-write lpValue = %d\n", lpValue);
        regValue = 0;
        ret = minipad_reg_write (ts->client,
                Q5199225K03_LP_MODE_ADDR,
                sizeof(char), (char *)&regValue); 
    }

EXIT:
    minipad_log_critical("minipad_enable: Exiting\n");
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
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
static void minipad_early_suspend(struct early_suspend *h)
{
    struct minipad_data *ts;
    //minipad_log("minipad_early_suspend: Entering\n");
    ts = container_of(h, struct minipad_data, early_suspend);
    minipad_suspend(ts->client, PMSG_SUSPEND);
    //minipad_log("minipad_early_suspend: Exiting\n");
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
static void minipad_late_resume(struct early_suspend *h)
{
    struct minipad_data *ts;
    //minipad_log("minipad_late_resume: Enering\n");
    ts = container_of(h, struct minipad_data, early_suspend);
    minipad_resume(ts->client);
    //minipad_log("minipad_late_resume: Exiting\n");
}
#endif /* End Android power functions */


/**
 * Open device
 *
 * @param inode
 * @param filp
 * @return 0
 */
static int minipad_open(struct inode *inode, struct file *filp)
{
    minipad_log("minipad_open: entering\n");
    minipad_log("minipad_open: exiting\n");
    return 0;
}

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

static int minipad_ioctl(struct inode *node, struct file *filp, unsigned int cmd, unsigned long arg)
{
    unsigned long    bCount;
    int    regValue;
    int    rc = -1;
    char    data[10];

    // minipad_log("minipad_ioctl: entering cmd=(%d)\n", cmd);
    switch (cmd) 
    {
    case MINIPAD_IOCTL_SET_BOOTLOADER_MODE:
        disable_irq(ts->client->irq);
        if ( ts->mode != MINIPAD_MODE_BOOTLOADER )
        {
            // 2. reset
            minipad_log("minipad_ioctl: Sending RESET command\n");
            regValue = 0x01;
            minipad_reg_write (ts->client, Q5199225K03_RESET_ADDR,
                 1, (char *)&regValue);
            clk_busy_wait(Q5199225K03_WAIT);
            clk_busy_wait(Q5199225K03_WAIT);

            // 3. send special sequence code
            minipad_log("minipad_ioctl: Sending BL sequence\n");
            data[0] = Q5199225K03_BL_SEQ1;
            data[1] = Q5199225K03_BL_SEQ2;
            data[2] = Q5199225K03_BL_SEQ3;
            data[3] = Q5199225K03_BL_SEQ4;
            rc = i2c_master_send (ts->client, data, 4);
            clk_busy_wait(Q5199225K03_WAIT);
            clk_busy_wait(Q5199225K03_WAIT);
        }
        else 
        {
            minipad_log("minipad_ioctl: Already in BOOTLOADER mode\n");
        }

        ts->client->addr = minipad_bl_addr;

        // 4. Confirm that we are in BOOTLOADER mode */
        rc = i2c_master_recv(ts->client, data, 1);
        if (rc < 0)
        {
            minipad_log_critical("minipad_ioctl: ERROR unable to change to bootloader mode\n");
            ts->mode = MINIPAD_MODE_NORMAL;
            ts->client->addr = minipad_normal_addr;
            ts->sendMode = Q5199225K03_MODE_BL_PWR_ON;
            rc = 0;
        }
        else
        {
            minipad_log("minipad_ioctl: BL Status Received 0x%02X, ret=%d\n",data[0],rc);
            ts->mode = MINIPAD_MODE_BOOTLOADER;
            ts->sendMode = rc;
            rc = 1;
        }

        minipad_log_critical("minipad_ioctl: Mode is set to  %d\n", ts->mode);
        enable_irq(ts->client->irq);
        break;

    case MINIPAD_IOCTL_DL_GET_STATUS:
        rc = ts->sendMode;
        minipad_log_critical("minipad_ioctl: DL_GET_STATUS=(%d)\n", rc);
        break;

    case MINIPAD_IOCTL_GET_MODE:
        minipad_log_critical("minipad_ioctl: Get Mode is set to  %d\n", ts->mode);
        rc = ts->mode;
        break;

    case MINIPAD_IOCTL_SET_NORMAL_MODE:
        disable_irq(ts->client->irq);
        if ( ts->mode != MINIPAD_MODE_NORMAL)
        {
            minipad_log("minipad_ioctl: re-setting flags to normal mode\n");
        }
        else
        {
            minipad_log("minipad_ioctl: Already in NORMAL mode\n");
        }

        ts->mode = MINIPAD_MODE_NORMAL;
        ts->client->addr = minipad_normal_addr;
        ts->sendMode = Q5199225K03_MODE_BL_PWR_ON;

        // reset the chip
        minipad_reset();

        // calibration
        minipad_calibration();

        minipad_log_critical("minipad_ioctl: Mode is set to  %d\n", ts->mode);
        enable_irq(ts->client->irq);
        rc = 1;
        break;

    case MINIPAD_IOCTL_GET_VERSION:
        rc = minipad_read_app_version();
        break;

    case MINIPAD_IOCTL_WRITE_REGISTER:
        bCount = copy_from_user(kernelBuffer, (char *)arg, (unsigned long)sizeof(A_MINIPAD_REG)); 
        if ( bCount == 0 )
        {
            A_MINIPAD_REG_PTR    regV;
            regV = (A_MINIPAD_REG_PTR) kernelBuffer;
            rc = minipad_reg_write (ts->client,
                regV->reg,
                sizeof(regV->value), (char *)&regV->value);
            clk_busy_wait(Q5199225K03_WAIT);
            clk_busy_wait(Q5199225K03_WAIT);
            minipad_log("minipad_ioctl: setting register %d = %d\n", (int)regV->reg, (int)regV->value);
        }
        break;

    case MINIPAD_IOCTL_READ_REGISTER:
        rc = minipad_reg_read(ts->client,arg);
        minipad_log("minipad_ioctl: Register %d = %d\n", (int)arg, (int)rc);
        break;

    case MINIPAD_IOCTL_READ_REGISTER16:
        rc = minipad_reg_read16(ts->client,arg);
        minipad_log("minipad_ioctl: Register16 %d = %d\n", (int)arg, (int)rc);
        break;

    case MINIPAD_IOCTL_DO_RESET:
        rc = minipad_reset();
        break;
    case MINIPAD_IOCTL_DO_CALIBRATION:
        rc = minipad_calibration();
        break;

    case MINIPAD_IOCTL_GET_XANDY:
        rc = ((x_last & 0xFFFF) << 16) | (y_last & 0xFFFF);
        break;
    case MINIPAD_IOCTL_SET_ENABLE:
        minipad_log("minipad_ioctl: enable() is called\n");
        ts->isopened = 1;
        // enable always because no guarantee resume comes first
        minipad_enable(ts->client);
        rc = 0;
        break;
    case MINIPAD_IOCTL_SET_DISABLE:
        minipad_log("minipad_ioctl: disable() is called\n");
        ts->isopened = 0;
        // disable always
        minipad_disable(ts->client);
        rc = 0;
        break;

    case MINIPAD_IOCTL_START:
        // dummy
        rc = 0;
        break;

    case MINIPAD_IOCTL_SETIRQ:
        bCount = copy_from_user(kernelBuffer, (char *)arg, (unsigned long)sizeof(char)); 
        if ( bCount == 0 )
            regValue = kernelBuffer[0];
        if ( regValue )
        {
            enable_irq(ts->client->irq);
            minipad_log("minipad_ioctl: setirq enabled\n");
        }
        else
        {
            disable_irq(ts->client->irq);
            minipad_log("minipad_ioctl: setirq disabled\n");
        }
        rc = 0;
        break;

    case MINIPAD_IOCTL_GETIRQ:
        rc = ts->client->irq;
        minipad_log("minipad_ioctl: getirq =%d\n", rc);
        break;

    case MINIPAD_IOCTL_GETGPIO:
        rc = gpio_get_value(MINIPAD_TS_GPIO); //83
        minipad_log("minipad_ioctl: getgpio =%d\n", rc);
        break;
        
    case MINIPAD_IOCTL_SET_LPMODE_VALUE:
        bCount = copy_from_user(kernelBuffer, (char *)arg, (unsigned long)sizeof(A_MINIPAD_REG)); 
        if ( bCount == 0 )
        {
            A_MINIPAD_REG_PTR    regV;
            regV = (A_MINIPAD_REG_PTR) kernelBuffer;
            lpValue = regV->value;

            minipad_log("minipad_ioctl: setting lpValue = %d\n", lpValue);

            // this is for zeppelin p1b specifically
            if( lpValue == 0 )
            {
                minipad_log_critical("minipad_enable re-write lpValue = %d\n", lpValue);
                regValue = 0;
                rc = minipad_reg_write (ts->client,
                        Q5199225K03_LP_MODE_ADDR,
                        sizeof(char), (char *)&regValue); 
            }
        }
        break;

    default:
        return EINVAL;
        break;
    }

//    minipad_log("minipad_ioctl: exiting(rc = %d)\n", rc);
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
static int minipad_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos )
{
    int    retval=-1;
    //int    i;
    int    reg;
    unsigned long    bCount;

    //minipad_log("minipad_write: entering\n");

    // check size
    if (count > 255) count = 255;

    // bootloader mode
    if ( ts->mode == MINIPAD_MODE_BOOTLOADER )
    {
        minipad_log("minipad_write: Currently in BOOTLOADER mode.\n");
        //ts->sendMode = Q5199225K03_MODE_BL_WAITING_FRAME;

        /* get data from user space into kernel space */
        bCount = copy_from_user(kernelBuffer, buf, (unsigned long)count); 
        minipad_log("minipad_write: copy_from_user() returned %d\n", (int)bCount);

        ts->client->addr = minipad_bl_addr;
        do
        {
            minipad_log("minipad_write: writing to device: size=%d\n",count); 

            /* for debug
            for ( i = 0; i < count; i++)
            {
                if ( !(i%20) )
                    minipad_log("\nminipad_write: ");
                minipad_log("%02x ", kernelBuffer[i]);
            }
            minipad_log("\n");
            */

            // sending to device
            retval = i2c_master_send (ts->client, kernelBuffer, count);

            /* On failure, output the error code and delay before trying again */
            if (retval < 0)
            {
                minipad_log_critical("minipad_write: ERROR write failed: %d\n", retval);
                //ts->sendMode = Q5199225K03_MODE_BL_PWR_ON;
            }
            else
            {
                minipad_log("minipad_write: i2c_master_send(): rc= %d\n",retval);
                //ts->sendMode = Q5199225K03_MODE_BL_WAITING_FRAME;
                retval = count;
            }
            clk_busy_wait(Q5199225K03_WAIT);
            clk_busy_wait(Q5199225K03_WAIT);
            clk_busy_wait(Q5199225K03_WAIT);
            clk_busy_wait(Q5199225K03_WAIT);
        }
        while (retval < 0);
    }
    else if ( ts->mode == MINIPAD_MODE_NORMAL )
    {
        //minipad_log("minipad_write: Currently in NORMAL mode.\n");

        /* Get the data from the user space */
        bCount = copy_from_user(kernelBuffer, buf, (unsigned long)count); 
        //minipad_log("minipad_write: copy_from_user() returned %d\n", (int)bCount);

        reg = ((kernelBuffer[0]<<8) | kernelBuffer[1]);

        minipad_log("minipad_write: config record: size: %d, reg = %d\n",count, reg);
        /*
        minipad_log("minipad_write: ");
        for ( i = 0; i < count; i++)
        {
            char    ch = kernelBuffer[i];
            minipad_log("%02X ", ch);
        }
        minipad_log("\n");
        */

        ts->client->addr = minipad_normal_addr;
        if ( (retval = minipad_reg_write (ts->client, reg, 4, (char *)&(kernelBuffer[2]))) == 0 )
        {
            //minipad_log("minipad_write: finished writing config info\n");
            retval = 6;
        }
        else
        {
            //minipad_log("minipad_write: finished writing config info\n");
        }
    }

    //minipad_log("minipad_write: exiting\n");
    return retval;
}

/**
 * Release device
 *
 * @param inode
 * @param filp
 * @return 0
 */
static int minipad_release(struct inode *inode, struct file *filp)
{
    minipad_log("minipad_release: entering\n");
    minipad_log("minipad_release: exiting \n");
    return 0;
}

/*!
 * @brief Logs data with a timestamp
 *
 * This function is called as a replacement to minipad_log and includes logging of
 * a timestamp prior to the format string.
 *
 * @param fmt Text of message to log
 */
void minipad_log(char *fmt, ...)
{
#ifdef    DEBUG_MINIPAD
    va_list args;
    struct timeval tv;

    /* Get the current time */
    do_gettimeofday (&tv);

    /* Log the timestamp */
    //printk (KERN_ERR "minipad: %d.%06d: ", (int)tv.tv_sec, (int)tv.tv_usec);
    printk ("minipad: %d.%06d: ", (int)tv.tv_sec, (int)tv.tv_usec);

    /* Print the rest of the data */
    va_start(args, fmt);
    vprintk(fmt, args);
    va_end(args);
#endif
    return;
}

/*!
 * @brief Critical logs data with a timestamp
 *
 * This function is called as a replacement to minipad_log and includes logging of
 * a timestamp prior to the format string and can not be turned off with the flag.
 *
 * @param fmt Text of message to log
 */
static void minipad_log_critical(char *fmt, ...)
{
    va_list args;
    struct timeval tv;

    /* Get the current time */
    do_gettimeofday (&tv);

    /* Log the timestamp */
    //printk (KERN_ERR "minipad: %d.%06d: ", (int)tv.tv_sec, (int)tv.tv_usec);
    printk ("minipad critical: %d.%06d: ", (int)tv.tv_sec, (int)tv.tv_usec);

    /* Print the rest of the data */
    va_start(args, fmt);
    vprintk(fmt, args);
    va_end(args);
    return;
}

/*!
 * @brief This is the initialization function for the driver
 *
 * This function implements the initialization of the driver.
 *
 *
 * @param None
 *
 * @return int - reurns poi
 */
static int __devinit minipad_init(void)
{
       // This gets printed even if minipad is not present (for example on Morrison)
    // minipad_log_critical("minipad_init\n");
    return i2c_add_driver(&minipad_driver);
}

/*!
 * @brief This is the exit function of the driver
 *
 * This function is called when the driver is required to exit
 *
 * @param None
 *
 * @return None
 */
static void __exit minipad_exit(void)
{
    i2c_del_driver(&minipad_driver);
}

module_init(minipad_init);
module_exit(minipad_exit);

MODULE_DESCRIPTION("Minipad Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Katz Yamada <Katz.Yamada@motorola.com>");
MODULE_VERSION("1.0");
