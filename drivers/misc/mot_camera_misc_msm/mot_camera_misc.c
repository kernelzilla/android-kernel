//#if defined (CONFIG_MOT_KERNEL)



/*
 * Provides the interface for the Motorola Imager Scripting Engine (ISE).
 *
 * Contains camera device driver header information.
 *
 * Copyright (C) 2009 Motorola, Inc.
 * All Rights Reserved
 *
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "mot_camera_misc.h"

static int camera_dev_open (struct inode *inode, struct file *file);
static int camera_dev_release (struct inode *inode, struct file *file);
static int camera_dev_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);


static struct file_operations camera_dev_fops = {
    .open           = camera_dev_open,
    .release        = camera_dev_release,
    .ioctl          = camera_dev_ioctl
};

static struct miscdevice cam_misc_device0 = {
    .minor          = MISC_DYNAMIC_MINOR,
    .name           = "camera0",
    .fops           = &camera_dev_fops
};


#define GPIO_POWERDOWN		0
#define GPIO_RESET		1
#define GPIO_SENSOR_NAME	"camera_misc"


#define DEBUG   2

#define err_print(fmt, args...) printk(KERN_ERR "fun %s "fmt"\n", __FUNCTION__, ##args)

#if DEBUG > 0

#define dbg_print(fmt, args...) printk(KERN_INFO "fun %s "fmt"\n", __FUNCTION__, ##args)

#if DEBUG > 1
#define ddbg_print(fmt, args...) printk(KERN_INFO "fun %s "fmt"\n", __FUNCTION__, ##args)
#else
#define ddbg_print(fmt, args...) ;
#endif

#else

#define dbg_print(fmt, args...)  ;
#define ddbg_print(fmt, args...) ;

#endif

static int camera_dev_open(struct inode *inode, struct file *file)
{
// TO DO REMOVE
    ddbg_print( "camera0 opened\n");
// TO DO REMOVE
    return 0;
}


static int camera_dev_release(struct inode *inode, struct file *file)
{
// TO DO REMOVE
    ddbg_print( "camera0 closed\n");
// TO DO REMOVE
    return 0;
}


static int camera_dev_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)

{
    int rc;

    ddbg_print( "camera ioctl cmd = %u, arg = %lu\n", cmd, arg);

    switch(cmd)
    {
        case CAMERA_RESET_WRITE:
            rc = gpio_request (GPIO_RESET, GPIO_SENSOR_NAME);
            if (!rc)
            {
                gpio_direction_output (GPIO_RESET, (arg ? 1 : 0));
                dbg_print( "CAMERA_MISC set RESET line to %u\n", (arg ? 1 : 0));
            }
            gpio_free (GPIO_RESET);
            if (rc)
            {
              err_print ("CAMERA_MISC: gpio_request () failed. rc = %d; cmd = %d; arg = %lu", rc, cmd, arg); 
              return -EIO;
            }
            break;


        case CAMERA_POWERDOWN_WRITE:
            rc = gpio_request (GPIO_POWERDOWN, GPIO_SENSOR_NAME);
            if (!rc)
            {
                if (0 == arg)
                {
#if 0 // TODO: NEED TO HANDLE MCLK
                  msm_camio_camif_pad_reg_reset();
                  mdelay (50);
#endif // TODO: NEED TO HANDLE MCLK
                  rc = gpio_direction_output (GPIO_POWERDOWN, 0);
                }
                else
                {
                  rc = gpio_direction_output (GPIO_POWERDOWN, 1);
                }
                dbg_print( "CAMERA_MISC set POWERDOWN line to %u\n", (arg ? 1 : 0));
            }
            gpio_free (GPIO_POWERDOWN);
            if (rc)
            {
              err_print ("CAMERA_MISC: gpio_request () failed. rc = %d; cmd = %u; arg = %lu", rc, cmd, arg); 
              return -EIO;
            }
            break;

        case CAMERA_CLOCK_DISABLE:
            msm_camio_camif_pad_reg_reset();
            dbg_print( "CAMERA_MISC turned off MCLK done\n");
            break;

        case CAMERA_CLOCK_ENABLE:
            /* enable mclk */
            msm_camio_clk_rate_set ((int)arg);
            dbg_print( "CAMERA_MISC set MCLK to %d\n", (int) arg);
            break;

        default:
            err_print( "CAMERA_MISC received unsupported cmd; cmd = %u\n", cmd);
            return -EIO;
            break;
    }

    return 0;
}

static int mot_ise_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	CDBG("mot_ise_sensor_open_init\n");
	return 0;
}

int mot_ise_sensor_release(void)
{
	CDBG("mot_ise_sensor_release\n");
	return -EBADF;
}

int mot_ise_sensor_config(void __user *argp)
{
	CDBG("mot_ise_sensor_config\n");
	return 0;
}

static int mot_camera_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	CDBG("mot_ise_probe_init\n");
	s->s_init = mot_ise_sensor_open_init;
	s->s_release = mot_ise_sensor_release;
	s->s_config = mot_ise_sensor_config;

	return 1;
}

static int __mot_camera_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, mot_camera_sensor_probe);
}

static struct platform_driver mot_camera_driver = {
	.probe = __mot_camera_probe,
	.driver = {
		.name = "msm_camera_mot_camera",
		.owner = THIS_MODULE,
	},
};
static int __init camera_misc_init(void)
{
    ddbg_print("camera misc init\n" );

    if(misc_register( &cam_misc_device0 ))
    {
        err_print( "error in register camera misc device!\n" );
        return -EIO;
    }
	return platform_driver_register(&mot_camera_driver);
}

static void __exit camera_misc_exit(void)
{
    ddbg_print("camera misc exit\n" );
    misc_deregister( &cam_misc_device0 );
}


module_init(camera_misc_init);
module_exit(camera_misc_exit);



//#endif
