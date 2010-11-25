//#if defined (CONFIG_MOT_KERNEL)

// This file creates function pointers to dummy functions needed so
// that the msm_camera driver does not call null function pointers.

#include <mach/camera.h>

static int mot_ise_sensor_open_init(struct msm_camera_sensor_info *data)
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

int mot_camera_probe_init(void *dev, void *ctrl)
{
	struct msm_sensor_ctrl_t* sptr;
        sptr = (struct msm_sensor_ctrl_t *)ctrl;

	CDBG("mot_ise_probe_init\n");
	
	sptr->s_init = mot_ise_sensor_open_init;
	sptr->s_release = mot_ise_sensor_release;
	sptr->s_config = mot_ise_sensor_config;

	return 1;
}

//#endif
