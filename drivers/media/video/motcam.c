/*
 * drivers/media/video/motcam.c
 *
 * Author: Elena Grimes (motorola.com)
 *
 * motcam generic camera driver
 *
 */

#include <media/v4l2-int-device.h>
#include <linux/i2c.h>
#include "motcam.h"
#include "oldomap34xxcam.h"
#include "oldisp/isp.h"
#include <oldisp/ispreg.h>
#include <linux/io.h>
#include <linux/mm.h>

#define MOTCAM_DRIVER_NAME "motcam"
#ifdef CONFIG_VIDEO_CAM_ISE
struct motcam_capture_size {
	unsigned long width;
	unsigned long height;
};
const static struct motcam_capture_size motcam_sizes[] = {
/*	{  160, 120 }, */
	{  176, 144 }, /* QCIF for Video Recording */
	{  320, 240 }, /* QVGA for Preview and Video Recording */
	{  352, 288 }, /* CIF for Video Recording */
	{  512, 1024}, /* Capture Resolution */
/*	{  640, 480 },	 / * 4X BINNING */
/*	{ 1280, 960 },	/ * 2X BINNING */
/*	{ 1600, 1200},	/ * 2 MP */
/*	{ 2048, 1536},	/ * 3 MP */
};
#define NUM_CAPTURE_SIZES ARRAY_SIZE(motcam_sizes)

static struct omap34xxcam_sensor_config cam_hwc = {
	.sensor_isp = 1,
	.xclk = OMAP34XXCAM_XCLK_A,
	.capture_mem = PAGE_ALIGN(2048 * 1536 * 2) * 4,
};
/* list of image formats supported by motcam sensor */
const static struct v4l2_fmtdesc motcam_formats[] = {
/*
	{
		.description	= "Bayer10 (GrR/BGb)",
		.pixelformat	= V4L2_PIX_FMT_SGRBG10,
	},
	{
		.description	= "Bayer10 pattern (GrR/BGb)",
		.pixelformat	= V4L2_PIX_FMT_PATT,
	},
 */
	{
		.description	= "Walking 1's pattern",
		.pixelformat	= V4L2_PIX_FMT_W1S_PATT,
	},

	{
		.description	= "YUYV Pattern",
		.pixelformat	=  V4L2_PIX_FMT_YUYV,
	},

/*
	{
		.description = "UYVY, packed",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	}
*/

};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(motcam_formats)
static int motcam_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;
	printk("psheldo1 in private data set\n");
	hwc->u.sensor.xclk = cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = cam_hwc.capture_mem;
	hwc->dev_index = 1;
	hwc->dev_minor = CAM_DEVICE_SOC;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	hwc->interface_type = ISP_PARLL;
	return 0;
}

static struct isp_interface_config motcam_if_config = {
	.ccdc_par_ser = ISP_PARLL,
	.dataline_shift = 0x1,
	/*.dataline_shift = 0x3,*/
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.vdint0_timing = 0x0,
	.vdint1_timing = 0x0,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_AND,
	.dcsub = 0,	 /* Disabling DCSubtract function */
	/*.raw_fmt_in = ISPCCDC_INPUT_FMT_GR_BG,*/
	//.wait_hs_vs = 0x0,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_RG_GB,
	.u.par.par_bridge = 0x3,
	.u.par.par_clk_pol = 0x0,
};

static int motcam_sensor_power_set(enum v4l2_power power)
{
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	switch (power) {
	case V4L2_POWER_ON:

	  if (previous_power == V4L2_POWER_OFF) {
	    /* Power Up Sequence */
	    isp_configure_interface(&motcam_if_config);
	  }
	  break;
	default:
	  break;
	}

	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return 0;
}

static struct motcam_platform_data sdp3430_motcam_platform_data = {
	.power_set        = motcam_sensor_power_set,
	.priv_data_set    = motcam_sensor_set_prv_data,
	.default_regs     = NULL,
};

#endif


/**
 * struct motcam_sensor_id
 */
struct motcam_sensor_id {
	u16 revision ;
	u16 model ;
	u16 mfr ;
};

struct motcam_sensor {
	const struct motcam_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_rect crop_rect;
	struct motcam_sensor_id sensor_id;
};

static struct motcam_sensor motcam = {
	.sensor_id = {
	 	.revision = 0,
		.model = 0,
		.mfr = 0
	},

};

/*
static struct i2c_driver motcam_i2c_driver;
*/

static enum v4l2_power current_power_state;

/*
 * struct vcontrol - Video controls
 * @v4l2_queryctrl: V4L2 VIDIOC_QUERYCTRL ioctl structure
 * @current_value: current value of this control
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {};

/*
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array.
 *
 * Returns the index of the requested ID from the control structure array
 */
static int find_vctrl(int id)
{
	int i = 0;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/*
 * Configuring motcam.
 *
 * Returns zero if succesful, or non-zero otherwise
 *
 */
static int motcam_configure(struct v4l2_int_device *s)
{
   /* Need to setup isp*/
   isp_configure_interface(&motcam_if_config);
   return 0;
} /* end motcam_configure() */

/* To get the cropping capabilities of motcam
 * Returns zero if successful, or non-zero otherwise.
 */
static int ioctl_cropcap(struct v4l2_int_device *s,
			 struct v4l2_cropcap *cropcap)
{

	return 0;
}

/* To get the current crop window for of motcam
 * Returns zero if successful, or non-zero otherwise.
 */
static int ioctl_g_crop(struct v4l2_int_device *s, struct  v4l2_crop *crop)
{

	return 0;
}

/* To set the crop window for of motcam
 * Returns zero if successful, or non-zero otherwise.
 */
static int ioctl_s_crop(struct v4l2_int_device *s, struct  v4l2_crop *crop)
{
	return 0;
}

/*
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
						struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;
	return 0;
}

/*
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */

static int ioctl_g_ctrl(struct v4l2_int_device *s,
			struct v4l2_control *vc)
{

	return 0;
}

/*
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	return 0;
}

/*
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
 static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
				   struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
	break;
	default:
		return -EINVAL;
	}

	fmt->flags = motcam_formats[index].flags;
	strlcpy(fmt->description, motcam_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = motcam_formats[index].pixelformat;

	return 0;
}

/*
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */

static int ioctl_try_fmt_cap(struct v4l2_int_device *s,
			     struct v4l2_format *f)
{
        /* TODO: NEED TO IMPLEMENT */
	return 0;
}

/*
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
 static int ioctl_s_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
/* TODO:
	struct ov3640_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;

	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;

	sensor->pix = *pix;
*/
	return 0;
}

/*
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
/* TODO:
	struct ov3640_sensor *sensor = s->priv;
	f->fmt.pix = sensor->pix;
*/
	return 0;
}

/*
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
/* TODO
	struct ov3640_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;
*/
	return 0;
}

/*
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{

	return 0;
}

/*
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{

	struct motcam_sensor *sensor = s->priv;

	return sensor->pdata->priv_data_set(p);

}

/*
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
 static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
 /* TODO */
  /*rval = ioctl_g_priv(s, &hw_config); */

  if (current_power_state == V4L2_POWER_OFF)
    motcam_configure(s);

  current_power_state = on;
  return 0;
}

/*
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call motcam_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach.  The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.  Returns 0 if
 * ov3640 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
/* TODO:
	struct ov3640_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	int err;

	err = ov3640_detect(c);
	if (err < 0) {
		dev_err(&c->dev, "Unable to detect " OV3640_DRIVER_NAME
								" sensor\n");
		return err;
	}

	sensor->ver = err;
	pr_info(OV3640_DRIVER_NAME " chip version 0x%02x detected\n",
								sensor->ver);
*/
	return 0;
}
/**
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 **/
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
					struct v4l2_frmsizeenum *frms)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frms->pixel_format == motcam_formats[ifmt].pixelformat)
			break;
	}

	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS) {
	  return -EINVAL;
	}
	/* Do we already reached all discrete framesizes? */
	if (frms->index >= NUM_CAPTURE_SIZES) {
	  return -EINVAL;
	}
	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = motcam_sizes[frms->index].width;
	frms->discrete.height = motcam_sizes[frms->index].height;

	return 0;
}

const struct v4l2_fract motcam_frameintervals[] = {
	{  .numerator = 1, .denominator = 11 },
	{  .numerator = 1, .denominator = 15 },
	{  .numerator = 1, .denominator = 20 },
	{  .numerator = 1, .denominator = 25 },
	{  .numerator = 1, .denominator = 30 },
	{  .numerator = 1, .denominator = 120 },
};

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					struct v4l2_frmivalenum *frmi)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == motcam_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
#if 0
	if (((frmi->width == motcam_sizes[4].width) &&
				(frmi->height == motcam_sizes[4].height)) ||
				((frmi->width == motcam_sizes[3].width) &&
				(frmi->height == motcam_sizes[3].height))) {
		/* FIXME: The only frameinterval supported by 5MP and 3MP
		 * capture sizes is 1/11 fps
		 */
		if (frmi->index != 0)
			return -EINVAL;
	} else if ((frmi->width == motcam_sizes[1].width) &&
				(frmi->height == motcam_sizes[1].height)) {
		/* QVGA base size supports all framerates, including 120 fps!
		 */
		if (frmi->index >= 6)
			return -EINVAL;
	} else {
		if (frmi->index >= 5)
			return -EINVAL;
	}
#endif
	if (frmi->index >= 5)
	  return -EINVAL;
	frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmi->discrete.numerator =
				motcam_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				motcam_frameintervals[frmi->index].denominator;

	return 0;
}

static struct v4l2_int_ioctl_desc motcam_ioctl_desc[] = {

	{vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
	 (v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
	{vidioc_int_dev_init_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_exit},
	{vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_priv_num,
	  (v4l2_int_ioctl_func *)ioctl_g_priv},
	{vidioc_int_init_num,
	  (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
	{vidioc_int_try_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_try_fmt_cap},
	{vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
	{vidioc_int_s_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_s_fmt_cap},
	{vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_s_parm},
	{vidioc_int_queryctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	  { vidioc_int_g_crop_num,
	  (v4l2_int_ioctl_func *)ioctl_g_crop},
	{vidioc_int_s_crop_num,
	  (v4l2_int_ioctl_func *)ioctl_s_crop},
	  { vidioc_int_cropcap_num,
	  (v4l2_int_ioctl_func *)ioctl_cropcap},
};


static struct v4l2_int_slave motcam_slave = {
	.ioctls		= motcam_ioctl_desc,
	.num_ioctls	= ARRAY_SIZE(motcam_ioctl_desc),
};



static struct v4l2_int_device motcam_int_device = {
	.module	= THIS_MODULE,
	.name	= MOTCAM_DRIVER_NAME,
	.priv	= &motcam,
	.type	= v4l2_int_type_slave,
	.u	= {
		.slave = &motcam_slave,
	},
};


/**
 * motcam_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of mt9p012_probe().
 */
static int __exit
motcam_remove(struct i2c_client *client)
{
	struct motcam_sensor *sensor = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

/*
static const struct i2c_device_id motcam_id[] = {
	{ MOTCAM_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, motcam_id);

static struct i2c_driver motcam_i2c_driver = {
	.driver = {
		.name = MOTCAM_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = motcam_probe,
	.remove = __exit_p(motcam_remove),
	.id_table = motcam_id,
};
*/

static int __init motcamsensor_init(void)
{
	int err;
	struct motcam_sensor *sensor = &motcam;
	sensor->pdata = &sdp3430_motcam_platform_data ;


	/*
	err = i2c_add_driver(&motcam_i2c_driver);
	if (err) {
		printk(KERN_ERR "Failed to register" MOTCAM_DRIVER_NAME ".\n");
		return err;
	}
	*/
	printk("psheldo1 in motcamsensor_init\n");
	sensor->v4l2_int_device = &motcam_int_device;
	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err)
	{
		printk("unable to register motcam for v4l2\n");
		/*i2c_set_clientdata(client, NULL);*/
	}
	return 0;
}
late_initcall(motcamsensor_init);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MOTCAM camera sensor driver");



