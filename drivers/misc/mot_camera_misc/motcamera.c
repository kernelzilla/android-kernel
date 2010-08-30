//#if defined (CONFIG_MOT_KERNEL)

// This file creates function pointers to dummy functions needed so
// that the msm_camera driver does not call null function pointers.

#include <mach/camera.h>
#include <../arch/arm/mach-msm/include/mach/gpio.h>
#include <linux/delay.h>


static int mot_camera_sensor_open_init(struct msm_camera_sensor_info *data)
{
	CDBG("mot_camera_sensor_open_init\n");
	return 0;
}

int mot_camera_sensor_release(void)
{
	CDBG("mot_camera_sensor_release\n");
	int rc;
          rc = gpio_request (109, "mot_camera power down");
            if (!rc)
            {
               
                  rc = gpio_direction_output (109, 1);
		    CDBG( "CAMERA_MISC set POWERDOWN line to %u\n", 1);
            }
            gpio_free (109);
            if (rc)
            {
              CDBG("CAMERA_MISC: gpio_request () failed. rc = %d; ", rc); 
              return -EIO;
            }
	     /*mdelay(10);
	     msm_camio_camif_pad_reg_reset(); // DISABLE the MCLK;
	     mdelay(5); */
		 
	return 0;
	//return -EBADF;
}

int mot_camera_sensor_config(void __user *argp)
{
	CDBG("mot_camera_sensor_config\n");
	return 0;
}

int mot_camera_probe_init(void *dev, void *ctrl)
{
	struct msm_sensor_ctrl_t* sptr;
        sptr = (struct msm_sensor_ctrl_t *)ctrl;

	CDBG("mot_camera_probe_init\n");
	
	sptr->s_init = mot_camera_sensor_open_init;
	sptr->s_release = mot_camera_sensor_release;
	sptr->s_config = mot_camera_sensor_config;

	return 1;
}

//#endif
