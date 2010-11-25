/*
 * drivers/media/video/mt9p013.c
 *
 * mt9p013 sensor driver
 *
 * Copyright (C) 2008 Texas Instruments.
 * Copyright (C) 2007-2008 Motorola, Inc.
 *
 * Contributors:
 *	Sameer Venkatraman <sameerv@ti.com>
 *	Martinez Leonides
 *	Greg Hofer <greg.hofer@hp.com>
 *
 * Leverage mt9p012.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * History:
 * 13-May-2009	Hewlett Packard	Created from mt9p013.c
 * 06-Jun-2009	Hewlett Packard Allow on the fly frame rate changes
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>

#include "mt9p013_hp.h"
#include "oldisp/isp.h"

#define DRIVER_NAME  "mt9p013"
#define MOD_NAME "MT9P013: "

#define I2C_INITIAL_LIST_SIZE	7
#define I2C_CONFIG_FRAME_LIST_SIZE	23
#define FRAME_648_120fps_ADDENDUM_LIST_SIZE 5
#define FRAME_SOLID_COLOR_ADDENDUM_LIST_SIZE 12

#define I2C_M_WR 0

static unsigned char initial_list_buf[][4] = {
	/* RESET_REGISTER */
	{0x30, 0x1A, 0x10, 0xC8},
	/* GROUPED_PAR_HOLD */
	{0x01, 0x04, 0x01, 0x00},
	/* RESERVED_MFR_3064 */
	{0x30, 0x64, 0x08, 0x05},
	/* Recommended values for image quality */
	{0x30, 0x86, 0x24, 0x68},
	{0x30, 0x88, 0x6F, 0xFF},
	{0x31, 0x6C, 0xA4, 0xF0},
	/* update all at once */
	/* GROUPED_PAR_HOLD */
	{0x01, 0x04, 0x00, 0x00}, /* update */
};

static struct i2c_msg initial_list[] = {
	{MT9P013_I2C_ADDR, 0x0, 4, &initial_list_buf[0][0]},
	{MT9P013_I2C_ADDR, 0x0, 3, &initial_list_buf[1][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &initial_list_buf[2][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &initial_list_buf[3][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &initial_list_buf[4][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &initial_list_buf[5][0]},
	{MT9P013_I2C_ADDR, 0x0, 3, &initial_list_buf[6][0]},
};

#define TBD_VAL 0x00  /* value filled in by mt9p013_configure_frame */
static unsigned char  config_frame_list_buf[][4] = {
	/* GROUPED_PAR_HOLD */
	{0x01, 0x04, 0x01, 0x00},
	/* VT_PIX_CLK_DIV */
	{0x03, 0x00, TBD_VAL, TBD_VAL},   /* 1 */
	/* VT_SYS_CLK_DIV */
	{0x03, 0x02, TBD_VAL, TBD_VAL},   /* 2 */
	/* PRE_PLL_CLK_DIV */
	{0x03, 0x04, TBD_VAL, TBD_VAL},   /* 3 */
	/* PLL_MULTIPLIER */
	{0x03, 0x06, TBD_VAL, TBD_VAL},   /* 4 */
	/* OP_PIX_CLK_DIV */
	{0x03, 0x08, TBD_VAL, TBD_VAL},   /* 5 */
	/* OP_SYS_CLK_DIV */
	{0x03, 0x0A, TBD_VAL, TBD_VAL},   /* 6 */
	/* X_OUTPUT_SIZE */
	{0x03, 0x4C, TBD_VAL, TBD_VAL},   /* 7 */
	/* Y_OUTPUT_SIZE */
	{0x03, 0x4E, TBD_VAL, TBD_VAL},   /* 8 */
	/* X_ADDR_START */
	{0x03, 0x44, TBD_VAL, TBD_VAL},   /* 9 */
	/* Y_ADDR_START */
	{0x03, 0x46, TBD_VAL, TBD_VAL},   /* 10 */
	/* X_ADDR_END */
	{0x03, 0x48, TBD_VAL, TBD_VAL},   /* 11 */
	/* Y_ADDR_END */
	{0x03, 0x4A, TBD_VAL, TBD_VAL},   /* 12 */
	/* READ_MODE */
	{0x30, 0x40, TBD_VAL, TBD_VAL},   /* 13 */
	/* FINE_INT_TIME */
	{0x02, 0x00, TBD_VAL, TBD_VAL},   /* 14 */
	/* FRAME_LEN_LINES */
	{0x03, 0x40, TBD_VAL, TBD_VAL},   /* 15 */
	/* LINE_LEN_PCK */
	{0x03, 0x42, TBD_VAL, TBD_VAL},   /* 16 */
	/* SCALE_M */
	{0x04, 0x04, TBD_VAL, TBD_VAL},   /* 17 */
	/* SCALING_MODE */
	{0x04, 0x00, TBD_VAL, TBD_VAL},   /* 18 */
	/* FINE_CORRECTION */
	{0x30, 0x10, TBD_VAL, TBD_VAL},   /* 19 */
	/* COARSE_INT_TIME */
	{0x02, 0x02, TBD_VAL, TBD_VAL},   /* 20 */
	/* MANUF_GAIN_GLOBAL */
	{0x30, 0x5E, TBD_VAL, TBD_VAL},   /* 21 */
	/* GROUPED_PAR_HOLD */
	{0x01, 0x04, 0x00, 0x00}, /* update */
};

static struct i2c_msg config_frame_list[] = {
	/* hold */
	{MT9P013_I2C_ADDR, 0x0, 3, &config_frame_list_buf[0][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[1][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[2][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[3][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[4][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[5][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[6][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[7][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[8][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[9][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[10][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[11][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[12][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[13][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[14][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[15][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[16][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[17][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[18][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[19][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[20][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &config_frame_list_buf[21][0]},
	/* update */
	{MT9P013_I2C_ADDR, 0x0, 3, &config_frame_list_buf[22][0]},
};

static unsigned char frame_648_120fps_addendum_buf[][4] = {
	/* hold */
	/* GROUPED_PAR_HOLD */
	{0x01, 0x04, 0x01, 0x00},
	/* values for QVGA@120fps */
	{0x31, 0x62, 0x04, 0xCE},
	{0x30, 0x8a, 0x64, 0x24},
	{0x30, 0x92, 0x0a, 0x53},
	/* update */
	/* GROUPED_PAR_HOLD */
	{0x01, 0x04, 0x00, 0x00}, /* update */
};

static struct i2c_msg frame_648_120fps_addendum[] = {
	/* hold */
	{MT9P013_I2C_ADDR, 0x0, 3, &frame_648_120fps_addendum_buf[0][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_648_120fps_addendum_buf[1][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_648_120fps_addendum_buf[2][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_648_120fps_addendum_buf[3][0]},
	/* update */
	{MT9P013_I2C_ADDR, 0x0, 3, &frame_648_120fps_addendum_buf[4][0]},
};

static unsigned char frame_solid_color_addendum_buf[][4] = {
	/* hold */
	/* GROUPED_PAR_HOLD */
	{0x01, 0x04, 0x01, 0x00},
	/* solid color pattern */
	/* GREEN1_GAIN */
	{0x30, 0x56, 0x00, 0x00},
	/* BLUE_GAIN */
	{0x30, 0x58, 0x00, 0x00},
	/* RED_GAIN */
	{0x30, 0x5A, 0x00, 0x00},
	/* GREEN2_GAIN */
	{0x30, 0x5C, 0x00, 0x00},
	/* MANUF_GAIN_GLOBAL */
	{0x30, 0x5E, 0x00, 0x00},
	/* TEST_DATA_RED */
	{0x30, 0x72, 0x03, 0xff},
	/* TEST_DATA_GREENR */
	{0x30, 0x74, 0x03, 0xff},
	/* TEST_DATA_BLUE */
	{0x30, 0x76, 0x03, 0xff},
	/* TEST_DATA_GREENB */
	{0x30, 0x78, 0x03, 0xff},
	/* TEST_PATTERN */
	{0x30, 0x70, 0x00, 0x01},
	/* update */
	/* GROUPED_PAR_HOLD */
	{0x01, 0x04, 0x00, 0x00}, /* update */
};

static struct i2c_msg frame_solid_color_addendum[] = {
	/* hold */
	{MT9P013_I2C_ADDR, 0x0, 3, &frame_solid_color_addendum_buf[0][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_solid_color_addendum_buf[1][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_solid_color_addendum_buf[2][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_solid_color_addendum_buf[3][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_solid_color_addendum_buf[4][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_solid_color_addendum_buf[5][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_solid_color_addendum_buf[6][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_solid_color_addendum_buf[7][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_solid_color_addendum_buf[8][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_solid_color_addendum_buf[9][0]},
	{MT9P013_I2C_ADDR, 0x0, 4, &frame_solid_color_addendum_buf[10][0]},
	/* update */
	{MT9P013_I2C_ADDR, 0x0, 3, &frame_solid_color_addendum_buf[11][0]},
};

/**
 * mt9p013_write_regs - writes an array of address/data information
 * to the i2c sensor registers.
 * @client: i2c driver client structure.
 * @msg: Array of address/data to be written.
 * @num: Number of registers to be written.
 * @length: Length of address and data in bytes
 * Returns zero if successful, or negative otherwise.
 */
static int
mt9p013_write_regs(struct i2c_client *client, struct i2c_msg *msg, int num)
{
	/* Note: It is significantly faster to write an array of register
	   settings in one burst than to write each register individually.
	*/
	int err;
	int retry = 0;
again:
	err = i2c_transfer(client->adapter, msg, num);
	if (err >= 0)
		return 0;

	if (retry <= I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retry ... %d\n", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto again;
	}
	return err;
}

/**
 * struct mt9p013_sensor_id
 */
struct mt9p013_sensor_id {
	u16 revision ;
	u16 model ;
	u16 mfr ;
};

/**
 * struct mt9p013_sensor - main structure for storage of sensor information
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @i2c_client: iic client device structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @scaler:
 * @ver: mt9p013 chip version
 * @fps: frames per second value
 */
struct mt9p013_sensor {
	const struct mt9p013_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	struct mt9p013_sensor_id sensor_id;
	int scaler;
	int fps;
	int state;
	bool resuming;
};

static struct mt9p013_sensor mt9p013 = {
	.timeperframe = {
		.numerator = 1,
		.denominator = 15,
	},
	.sensor_id = {
		.revision = 0,
		.model = 0,
		.mfr = 0
	},
	.scaler = 0,
	.fps = 0,
	.state = SENSOR_NOT_DETECTED,
};

static struct i2c_driver mt9p013_i2c_driver;

/* list of image formats supported by mt9p013 sensor */
const static struct v4l2_fmtdesc mt9p013_formats[] = {
	{
		.description	= "Bayer10 (GrR/BGb)",
		.pixelformat	= V4L2_PIX_FMT_SGRBG10,
	},
	{
		.description	= "Walking 1's pattern",
		.pixelformat	= V4L2_PIX_FMT_W1S_PATT,
	}
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(mt9p013_formats)
static u32 min_exposure_time;
static u32 max_exposure_time;
static enum mt9p013_frame_type current_iframe = MT9P013_FRAME_1296_30FPS;
static enum v4l2_power current_power_state = V4L2_POWER_OFF;
static struct mt9p013_clock_freq current_clk = {
	.x_clk = MT9P013_XCLK_NOM_2
};

/**
 * struct sensor_settings - struct for storage of sensor clock, frame, and
 * exposure params.
 * @clk: clock parameters:
 *    @pre_pll_div: pre pll divider
 *    @pll_mult: pll multiplier
 *    @vt_pix_clk_div: video pixel clock divider
 *    @vt_sys_clk_div: video system clock divider
 *    @op_pix_clk_div: output pixel clock divider
 *    @op_sys_clk_div: output system clock divider
 *    @min_pll: minimum pll multiplier
 *    @max_pll: maximum pll multiplier
 * @frame: frame definition params
 *    @frame_len_lines: number of lines in frame
 *    @line_len_pck: number of pixels in line
 *    @x_addr_start: horizontal start address
 *    @x_addr_end: horizontal end address
 *    @y_addr_start: vertical start address
 *    @y_addr_end: vertical end address
 *    @x_output_size: horizontal output size
 *    @y_output_size: vertical output size
 *    @x_odd_inc: X odd increment value
 *    @y_odd_inc: Y odd increment value
 *    @x_bin: X binning enable
 *    @xy_bin: XY binning enable
 *    @scale_m: scale factor = 16/M
 *    @scale_mode: image resolution scaler mode
 * @exposure: exposure params
 *    @coarse_int_tm: coarse integration time in lines
 *    @fine_int_tm: fine integration time in pixels
 *    @fine_correction: fine correction time in pixels
 *    @analog_gain: global analog gain (smia)
*/
static struct mt9p013_sensor_settings sensor_settings[] = {

	/* FRAME_5MP */
	{
		.clk = {
			.pre_pll_div = 5,
			.pll_mult = 93,
			.vt_pix_clk_div = 4,
			.vt_sys_clk_div = 1,
			.op_pix_clk_div = 8,
			.op_sys_clk_div = 1,
		},
		.frame = {
			.frame_len_lines_min = 2077,
			.line_len_pck = 5372,
			.x_addr_start = 8,
			.x_addr_end = 2599,
			.y_addr_start = 8,
			.y_addr_end = 1951,
			.x_output_size = 2592,
			.y_output_size = 1944,
			.x_odd_inc = 1,
			.y_odd_inc = 1,
			.x_bin = 0,
			.xy_bin = 0,
			.scale_m = 0,
			.scale_mode = 0,
		},
		.exposure = {
			.coarse_int_tm = 1700,
			.fine_int_tm = 882,
			.fine_correction = 156,
			.analog_gain = 0x10C0
		}
	},

	/* FRAME_3MP */
	{
		.clk = {
			.pre_pll_div = 5,
			.pll_mult = 93,
			.vt_pix_clk_div = 4,
			.vt_sys_clk_div = 1,
			.op_pix_clk_div = 8,
			.op_sys_clk_div = 1,
		},
		.frame = {
			.frame_len_lines_min = 2702,
			.line_len_pck = 4130,
			.x_addr_start = 0,
			.x_addr_end = 2607,
			.y_addr_start = 4,
			.y_addr_end = 1955,
			.x_output_size = 2048,
			.y_output_size = 1532,
			.x_odd_inc = 1,
			.y_odd_inc = 1,
			.x_bin = 0,
			.xy_bin = 0,
			.scale_m = 20,
			.scale_mode = 2,
		},
		.exposure = {
			.coarse_int_tm = 1700,
			.fine_int_tm = 882,
			.fine_correction = 156,
			.analog_gain = 0x10C0
		}
	},

	/* FRAME_1296_30FPS */
	{
		.clk = {
			.pre_pll_div = 6,
			.pll_mult = 134,
			.vt_pix_clk_div = 5,
			.vt_sys_clk_div = 1,
			.op_pix_clk_div = 10,
			.op_sys_clk_div = 1,
		},
		.frame = {
			.frame_len_lines_min = 1061,
			.line_len_pck = 3360,
			.x_addr_start = 8,
			.x_addr_end = 2597,
			.y_addr_start = 8,
			.y_addr_end = 1949,
			.x_output_size = 1296,
			.y_output_size = 972,
			.x_odd_inc = 3,
			.y_odd_inc = 3,
			.x_bin = 0,
			.xy_bin = 1,
			.scale_m = 0,
			.scale_mode = 0,
		},
		.exposure = {
			.coarse_int_tm = 1000,
			.fine_int_tm = 1794,
			.fine_correction = 348,
			.analog_gain = 0x10C0
		}
	},

	/* FRAME_648_30FPS */
	{
		.clk = {
			.pre_pll_div = 6,
			.pll_mult = 187,
			.vt_pix_clk_div = 5,
			.vt_sys_clk_div = 2,
			.op_pix_clk_div = 10,
			.op_sys_clk_div = 2,
		},
		.frame = {
			.frame_len_lines_min = 775,
			.line_len_pck = 3200,
			.x_addr_start = 0,
			.x_addr_end = 2601,
			.y_addr_start = 4,
			.y_addr_end = 1949,
			.x_output_size = 648,
			.y_output_size = 486,
			.x_odd_inc = 7,
			.y_odd_inc = 7,
			.x_bin = 0,
			.xy_bin = 1,
			.scale_m = 0,
			.scale_mode = 0,
		},
		.exposure = {
			.coarse_int_tm = 750,
			.fine_int_tm = 1794,
			.fine_correction = 348,
			.analog_gain = 0x10C0
		}
	},

	/* FRAME_648_120FPS */
	{
		.clk = {
			.pre_pll_div = 4,
			.pll_mult = 120,
			.vt_pix_clk_div = 6,
			.vt_sys_clk_div = 1,
			.op_pix_clk_div = 10,
			.op_sys_clk_div = 1,
		},
		.frame = {
			.frame_len_lines_min = 565,
			.line_len_pck = 1832,
			.x_addr_start = 0,
			.x_addr_end = 2601,
			.y_addr_start = 8,
			.y_addr_end = 1921,
			.x_output_size = 648,
			.y_output_size = 486,
			.x_odd_inc = 7,
			.y_odd_inc = 7,
			.x_bin = 0,
			.xy_bin = 0,
			.scale_m = 32,
			.scale_mode = 0,
		},
		.exposure = {
			.coarse_int_tm = 568,
			.fine_int_tm = 1016,
			.fine_correction = 156,
			.analog_gain = 0x11FD
		}
	},

	/* FRAME_216_30FPS */
	{
		.clk = {
			.pre_pll_div = 3,
			.pll_mult = 94,
			.vt_pix_clk_div = 5,
			.vt_sys_clk_div = 2,
			.op_pix_clk_div = 10,
			.op_sys_clk_div = 2,
		},
		.frame = {
			.frame_len_lines_min = 775,
			.line_len_pck = 3200,
			.x_addr_start = 0,
			.x_addr_end = 2601,
			.y_addr_start = 4,
			.y_addr_end = 1949,
			.x_output_size = 216,
			.y_output_size = 162,
			.x_odd_inc = 7,
			.y_odd_inc = 7,
			.x_bin = 0,
			.xy_bin = 1,
			.scale_m = 48,
			.scale_mode = 2,
		},
		.exposure = {
			.coarse_int_tm = 770,
			.fine_int_tm = 1794,
			.fine_correction = 348,
			.analog_gain = 0x11FD
		}
	}
};

/**
 * struct vcontrol - Video controls
 * @v4l2_queryctrl: V4L2 VIDIOC_QUERYCTRL ioctl structure
 * @current_value: current value of this control
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = 0,
			.maximum = -1,
			.step = EXPOSURE_STEP,
			.default_value = DEF_EXPOSURE,
		},
		.current_value = DEF_EXPOSURE,
	},
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Analog Gain",
			.minimum = MT9P013_MIN_LINEAR_GAIN,
			.maximum = MT9P013_MAX_LINEAR_GAIN,
			.step = LINEAR_GAIN_STEP,
			.default_value = DEF_LINEAR_GAIN,
		},
		.current_value = DEF_LINEAR_GAIN,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_COLOR_BAR,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Color Bar",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = 0,
		},
		.current_value = 0,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_FLASH_NEXT_FRAME,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Flash On Next Frame",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = 0,
		},
		.current_value = 0,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_SENSOR_ID_REQ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Sensor ID",
			.minimum = 0,
			.maximum = -1,
			.step = 0,
			.default_value = 0,
		},
		.current_value = 0,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_ORIENTATION,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Orientation",
			.minimum = MT9P013_NO_HORZ_FLIP_OR_VERT_FLIP,
			.maximum = MT9P013_HORZ_FLIP_AND_VERT_FLIP,
			.step = 0,
			.default_value = MT9P013_NO_HORZ_FLIP_OR_VERT_FLIP,
		},
		.current_value = MT9P013_NO_HORZ_FLIP_OR_VERT_FLIP,
	}
};

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/**
 * mt9p013_read_reg - Read a value from a register in an mt9p013 sensor device
 * @client: i2c driver client structure
 * @data_length: length of data to be read
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an mt9p013 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
mt9p013_read_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 *val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;
	if (data_length != I2C_MT9P013_8BIT && data_length != I2C_MT9P013_16BIT
			&& data_length != I2C_MT9P013_32BIT)
		return -EINVAL;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		msg->len = data_length;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = 0;
		/* high byte comes first */
		if (data_length == I2C_MT9P013_8BIT)
			*val = data[0];
		else if (data_length == I2C_MT9P013_16BIT)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
				(data[1] << 16) + (data[0] << 24);
		return 0;
	}
	dev_err(&client->dev, "read from offset 0x%x error %d\n", reg, err);
	return err;
}

/**
 * Write a value to a register in mt9p013 sensor device.
 * Supports a 1, 2, or 4 byte data length
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * @length: Length of address and data in bytes
 * Returns zero if successful, or non-zero otherwise.
 */
static int mt9p013_write_reg(struct i2c_client *client, u16 length,
						u16 reg, u32 val)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[6];
	int retries = 0;

	if (!client->adapter)
		return -ENODEV;

retry:
	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = length + 2;	/* add in address bytes */
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);
	data[1] = (u8) (reg & 0xff);
	if (length == I2C_MT9P013_8BIT) {
		data[2] = val & 0xff;
	} else if (length == I2C_MT9P013_16BIT) {
		data[2] = (val >> 8) & 0xff;
		data[3] = val & 0xff;
	} else {
		data[2] = (val >> 24) & 0xff;
		data[3] = (val >> 16) & 0xff;
		data[4] = (val >> 8) & 0xff;
		data[5] = val & 0xff;
	}

	err = i2c_transfer(client->adapter, msg, 1);
	udelay(50);

	if (err >= 0)
		return 0;

	if (retries <= 5) {
		dev_dbg(&client->dev, "Retrying I2C... %d", retries);
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto retry;
	}

	return err;
}

/**
 * mt9p013_set_exposure_time - sets exposure time per input value
 * @exp_time: exposure time to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 *
 * If the requested exposure time is not within the allowed limits, the
 * exposure time is forced to the limit value. The HW
 * is configured to use the new exposure time, and the
 * video_control[] array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int mt9p013_set_exposure_time(u32 exp_time, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	int err = 0;
	struct mt9p013_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct mt9p013_sensor_settings *ss = &sensor_settings[current_iframe];
	u32 coarse_int_time = 0;

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (exp_time < min_exposure_time) {
			dev_err(&client->dev, "Exposure time %d us not within "
				"the legal range.\n", exp_time);
			dev_err(&client->dev, "Exposure time must be "
				"greater than %d us\n", min_exposure_time);
			exp_time = min_exposure_time;
		}

#ifndef ALLOW_EXPOSURE_TIME_FRAME_EXTENSION
      if (exp_time > max_exposure_time) {
			dev_err(&client->dev, "Exposure time %d us not within "
				"the legal range.\n", exp_time);
			dev_err(&client->dev, "Exposure time must be "
				"less than %d us\n", max_exposure_time);
			exp_time = max_exposure_time;
		}
#endif

		coarse_int_time = ((((exp_time / 10) * \
			(current_clk.vt_pix_clk / 1000)) / 1000) - \
			(ss->exposure.fine_int_tm / 10)) / \
			(ss->frame.line_len_pck / 10);

		dev_dbg(&client->dev, "coarse_int_time calculated = %d\n",
							coarse_int_time);

		if (coarse_int_time != ss->exposure.coarse_int_tm) {
			err = mt9p013_write_reg(client, I2C_MT9P013_16BIT,
					REG_COARSE_INT_TIME, coarse_int_time);
			ss->exposure.coarse_int_tm = coarse_int_time;
		}
	}

	if (err)
		dev_err(&client->dev, "Error setting exposure time...%d\n",
									err);
	else {
		if (lvc)
			lvc->current_value = exp_time;
	}

	return err;
}

/**
 * mt9p013_set_gain - sets sensor analog gain per input value
 * @lineargain: q8 analog gain value to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 analog gain entry in mt9p013_video_control array
 *
 * If the requested analog gain is within the allowed limits, the HW
 * is configured to use the new gain value, and the mt9p013_video_control
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int mt9p013_set_gain(u16 linear_gain_q8, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	int err = 0;
	u16 reg_gain = 0, digital_gain = 1;
	u16 shift_bits, gain_stage_2x, linear_gain_q5, digital_gain_bp;
	u16 analog_gain_code;
	struct mt9p013_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct mt9p013_sensor_settings *ss = &sensor_settings[current_iframe];

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		/* Convert gain from linear to register value
			(1<<2 = 0.5 lsb before shift) */
		linear_gain_q5 = (linear_gain_q8 + (1<<2)) >> 3;

		digital_gain_bp = 12;

		if (linear_gain_q5 >= 8*32) {	/* > 8.0 (Q5) */
			gain_stage_2x = 0x180;	/* AG2X_1 = 2X, AG2X_2 = 2X */
			/* account for gain stages */
			shift_bits = 0x2;

		} else if (linear_gain_q5 >= 2*32) { /* > 2.0 (Q5) */
			gain_stage_2x = 0x080;	/* AG2X_1 = 2X, AG2X_2 = 1X */
			shift_bits = 0x1;

		} else {
			gain_stage_2x = 0x000;	/* AG2X_1 = 1X, AG2X_2 = 1X */
			shift_bits = 0x0;
		}

		/* Combine 2X analog gains and analog PGA gain
			(compensating for gain stages) */
		analog_gain_code =
				gain_stage_2x | (linear_gain_q5 >> shift_bits);

		if (analog_gain_code < MT9P013_MIN_ANALOG_GAIN) {
			analog_gain_code = MT9P013_MIN_ANALOG_GAIN;
			dev_err(&client->dev, "Gain=%d out of legal range.\n",
				linear_gain_q8);
			dev_err(&client->dev, "Gain must be greater than %d \n",
				MT9P013_MIN_LINEAR_GAIN);
		}

		if (analog_gain_code > MT9P013_MAX_ANALOG_GAIN) {
			analog_gain_code = MT9P013_MAX_ANALOG_GAIN;
			dev_err(&client->dev, "Gain=%d out of legal range.\n",
				linear_gain_q8);
			dev_err(&client->dev, "Gain must be less than %d \n",
				MT9P013_MAX_LINEAR_GAIN);
		}

		/* Combine Digital gain and analog gains */
		reg_gain = (digital_gain << digital_gain_bp) | analog_gain_code;

		/*
		dev_info(&client->dev, "set_gain: lineargain=%d " \
			"reg_gain=0x%x sensor_rev=%d\n", \
			linear_gain_q8, reg_gain, rev);
		*/

		if (reg_gain !=	ss->exposure.analog_gain) {
			err = mt9p013_write_reg(client, I2C_MT9P013_16BIT,
					REG_MANUF_ANALOG_GAIN_GLOBAL, reg_gain);
			ss->exposure.analog_gain = reg_gain;
		}
	}

	if (err) {
		dev_err(&client->dev, "Error setting analog gain: %d\n", err);
		return err;
	} else {
		if (lvc)
			lvc->current_value = linear_gain_q8;
	}

	return err;
}

/**
 * mt9p013_update_clocks - calcs sensor clocks based on sensor settings.
 * @xclk: current sensor input clock (xclk)
 * @isize: image size enum
 */
int mt9p013_update_clocks(u32 xclk, enum mt9p013_frame_type iframe)
{
	current_clk.x_clk = xclk;

	current_clk.pll_clk =
			(xclk / sensor_settings[iframe].clk.pre_pll_div) *
			sensor_settings[iframe].clk.pll_mult;

	current_clk.vt_pix_clk = current_clk.pll_clk /
			(sensor_settings[iframe].clk.vt_pix_clk_div *
			sensor_settings[iframe].clk.vt_sys_clk_div);

	current_clk.op_pix_clk = current_clk.pll_clk /
			(sensor_settings[iframe].clk.op_pix_clk_div *
			sensor_settings[iframe].clk.op_sys_clk_div);

	printk(KERN_DEBUG "MT9P013: xclk=%u, pll_clk=%u, " \
		"vt_pix_clk=%u, op_pix_clk=%u\n", xclk, current_clk.pll_clk,
		current_clk.vt_pix_clk, current_clk.op_pix_clk);

	return 0;
}

/**
 * mt9p013_set_color_bar_mode - puts sensor in color bar test mode
 * @enable: 0 = off, 1 = on
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 * This function should be called after the resolution is setup. The sensor
 * will stay in color bar mode until the next resolution is selected.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int mt9p013_set_color_bar_mode(u16 enable, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	int err = 0;
	struct mt9p013_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (enable) {
			err = mt9p013_write_reg(client, I2C_MT9P013_16BIT,
				    REG_TEST_PATTERN, 0x2);
		} else {
			err = mt9p013_write_reg(client, I2C_MT9P013_16BIT,
				    REG_TEST_PATTERN, 0x0);
		}
	}

	if (err)
		dev_err(&client->dev, "Error setting color bar mode\n");
	else {
		if (lvc)
			lvc->current_value = enable;
	}

	return err;
}

/**
 * mt9p013_set_flash_next_frame - configures flash on for the next frame
 * @enable: 0 = off, 1 = on
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
int mt9p013_set_flash_next_frame(u16 enable, struct v4l2_int_device *s,
							struct vcontrol *lvc)
{
	int err = 0;
	struct mt9p013_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	dev_info(&client->dev, "set_flash_next_frame = %d, power = %d\n",
		enable,
		(current_power_state == V4L2_POWER_ON) || sensor->resuming);

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		if (enable) {
			err = mt9p013_write_reg(client, I2C_MT9P013_16BIT,
				    REG_FLASH, 0x0100);
		} else {
			err = mt9p013_write_reg(client, I2C_MT9P013_16BIT,
				    REG_FLASH, 0x0000);
		}

		/* Auto reset */
		enable = 0;
	}

	if (err)
		dev_err(&client->dev, "Error setting flash register\n");
	else {
		if (lvc)
			lvc->current_value = enable;
	}

	return err;
}

/**
 * mt9p013_set_framerate - Sets framerate by adjusting frame_length_lines reg.
 * @s: pointer to standard V4L2 device structure
 * @fper: frame period numerator and denominator in seconds
 * @iframe: enum value corresponding to frame type (size & fps)
 *
 * The maximum exposure time is also updated since it is affected by the
 * frame rate.
 **/
static int mt9p013_set_framerate(struct v4l2_int_device *s,
			struct v4l2_fract *fper, enum mt9p013_frame_type iframe)
{
	int err = 0, i = 0;
	u32 frame_length_lines, line_time_q8;
	struct mt9p013_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct mt9p013_sensor_settings *ss;
	struct vcontrol *lvc = NULL;

	/* fper is range checked in mt9p013_calc_xclk */

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		ss = &sensor_settings[iframe];

		line_time_q8 = (((u32)ss->frame.line_len_pck * 1000 * 256) /
			(current_clk.vt_pix_clk / 1000)); /* usec's (q8) */

		frame_length_lines = (((u32)fper->numerator * 1000000 * 256 /
			fper->denominator)) / line_time_q8;

		/* Range check frame_length_lines */
		if (frame_length_lines > MT9P013_MAX_FRAME_LENGTH_LINES)
			frame_length_lines = MT9P013_MAX_FRAME_LENGTH_LINES;
		else if (frame_length_lines < ss->frame.frame_len_lines_min)
			frame_length_lines = ss->frame.frame_len_lines_min;

		mt9p013_write_reg(client, I2C_MT9P013_16BIT,
				REG_FRAME_LEN_LINES,	frame_length_lines);

		ss[iframe].frame.frame_len_lines = frame_length_lines;

		/* Update min/max exposure times */
		min_exposure_time = (ss->exposure.fine_int_tm * 1000000 /
					(current_clk.vt_pix_clk)) + 1;
		max_exposure_time = (line_time_q8 *
					(frame_length_lines - 1)) >> 8;

		/* Update Exposure Time */
		i = find_vctrl(V4L2_CID_EXPOSURE);
		if (i >= 0) {
			lvc = &video_control[i];
			/* Update min/max for query control */
			lvc->qc.minimum = min_exposure_time;
			lvc->qc.maximum = max_exposure_time;

			mt9p013_set_exposure_time(lvc->current_value,
						sensor->v4l2_int_device, lvc);
		}

		v4l_info(client, "MT9P013 Set Framerate: fper=%d/%d, "
			"frame_len_lines=%d, max_expT=%dus\n",
			fper->numerator, fper->denominator,
			frame_length_lines, max_exposure_time);
	}

	return err;
}

/**
 * mt9p013_calc_size - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum mt9p013_image_size mt9p013_calc_size(unsigned int width,
							unsigned int height)
{
	enum mt9p013_image_size isize;
	unsigned long pixels = width * height;

	for (isize = MT9P013_BIN4XSCALE; isize <= MT9P013_FIVE_MP; isize++) {
		if (mt9p013_sizes[isize].height *
					mt9p013_sizes[isize].width >= pixels) {
#ifndef CONFIG_VIDEO_OMAP3_HP3A
			/* To improve image quality in VGA */
			if ((pixels > CIF_PIXELS) && (isize == MT9P013_BIN4X)) {
				isize = MT9P013_BIN2X;

			} else if ((pixels > QQVGA_PIXELS) &&
					(isize == MT9P013_BIN4XSCALE)) {
				isize = MT9P013_BIN4X;
			}
#endif
			return isize;
		}
	}

	return MT9P013_FIVE_MP;
}

/**
 * mt9p013_find_isize - Find the best match for a requested image capture size
 * @width: requested image width in pixels
 * @height: requested image height in pixels
 *
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum mt9p013_image_size mt9p013_find_isize(unsigned int width)
{
	enum mt9p013_image_size isize;

	for (isize = MT9P013_BIN4XSCALE; isize <= MT9P013_FIVE_MP; isize++) {
		if (mt9p013_sizes[isize].width >= width)
			break;
	}

	return isize;
}

/**
 * mt9p013_find_frame_index - Find the frame index that matches the
 *  requested frame rate and size
 * @fps: desired frame rate
 * @isize: enum value corresponding to image size
 */
static enum mt9p013_frame_type mt9p013_find_iframe(unsigned int fps,
						enum mt9p013_image_size isize)
{
	enum mt9p013_frame_type iframe = 0;

	if (isize == MT9P013_BIN4XSCALE) {
		iframe = MT9P013_FRAME_216_30FPS;

	} else if (isize == MT9P013_BIN4X) {
		if (fps <= 30)
			iframe = MT9P013_FRAME_648_30FPS;
		else
			iframe = MT9P013_FRAME_648_120FPS;

	} else if (isize == MT9P013_BIN2X) {
		iframe = MT9P013_FRAME_1296_30FPS;

	} else if (isize == MT9P013_THREE_MP) {
		iframe = MT9P013_FRAME_3MP_10FPS;

	} else {
		iframe = MT9P013_FRAME_5MP_10FPS;
	}

	return iframe;
}

/**
 * mt9p013_calc_xclk - Calculate the required xclk frequency
 * @c: i2c client driver structure
 *
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate and return the required xclk frequency
 */
static unsigned long mt9p013_calc_xclk(struct v4l2_int_device *s)
{
	int qvga_120 = 0;
	unsigned long xclk;
	struct mt9p013_sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &sensor->timeperframe;
	struct v4l2_pix_format *pix = &sensor->pix;

	if ((timeperframe->numerator == 0)
	|| (timeperframe->denominator == 0)) {
		/* supply a default nominal_timeperframe */
		timeperframe->numerator = 1;
		timeperframe->denominator = MT9P013_DEF_FPS;
	}

	sensor->fps = timeperframe->denominator/timeperframe->numerator;
	if ((sensor->fps == 120) && (pix->width <= VIDEO_WIDTH_4X_BINN) &&
				(pix->width > VIDEO_WIDTH_4X_BINN_SCALED))
		qvga_120 = 1;
	if (sensor->fps < MT9P013_MIN_FPS)
		sensor->fps = MT9P013_MIN_FPS;
	else if ((sensor->fps > MT9P013_MAX_FPS) && (!qvga_120))
		sensor->fps = MT9P013_MAX_FPS;

	timeperframe->numerator = 1;
	timeperframe->denominator = sensor->fps;

	/*
	if ((pix->width <= VIDEO_WIDTH_4X_BINN) && (sensor->fps > 15) &&
							(!qvga_120))
		xclk = MT9P013_XCLK_NOM_2;
	else
		xclk = MT9P013_XCLK_NOM_1;
	*/

	xclk = MT9P013_XCLK_NOM_2;

	return xclk;
}

/**
 * Sets the sensor orientation.
 */
static int mt9p013_set_orientation(enum mt9p013_orientation val,
			struct v4l2_int_device *s, struct vcontrol *lvc)
{
	int err = 0, i;
	u8 orient;
	struct mt9p013_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;

	if ((current_power_state == V4L2_POWER_ON) || sensor->resuming) {
		switch (val) {
		case MT9P013_NO_HORZ_FLIP_OR_VERT_FLIP:
			orient = 0x0;
			break;
		case MT9P013_HORZ_FLIP_ONLY:
			orient = 0x1;
			break;
		case MT9P013_VERT_FLIP_ONLY:
			orient = 0x2;
			break;
		case MT9P013_HORZ_FLIP_AND_VERT_FLIP:
			orient = 0x3;
			break;
		default:
			orient = 0x0;
			break;
		}
		err = mt9p013_write_reg(client, I2C_MT9P013_8BIT,
					REG_IMAGE_ORIENTATION, orient);

		v4l_info(client, "mt9p013 orientation = 0x%x\n", orient);
	}

	if (err) {
		v4l_err(client, "Error setting orientation: %d", err);
	} else {
		i = find_vctrl(V4L2_CID_PRIVATE_ORIENTATION);
		if (i >= 0) {
			lvc = &video_control[i];
			lvc->current_value = (u32)val;
		}
	}

	return err;
}

/**
 * mt9p013_configure_frame - Setup the frame, clock and exposure parmas in the
 * config_frame_list array.
 *
 * @c: i2c client driver structure
 *
 * The config_frame_list is a common list used by all frame sizes & frame
 * rates that is filled in by this routine.
 */
int mt9p013_configure_frame(enum mt9p013_frame_type iframe)
{
	u16 data;

	/* I2C_REG_VT_PIX_CLK_DIV */
	config_frame_list_buf[1][2] =
		(sensor_settings[iframe].clk.vt_pix_clk_div >> 8) & 0xff;
	config_frame_list_buf[1][3] =
		sensor_settings[iframe].clk.vt_pix_clk_div & 0xff;

	/* I2C_REG_VT_SYS_CLK_DIV */
	config_frame_list_buf[2][2] =
		(sensor_settings[iframe].clk.vt_sys_clk_div >> 8) & 0xff;
	config_frame_list_buf[2][3] =
		sensor_settings[iframe].clk.vt_sys_clk_div & 0xff;

	/* I2C_REG_PRE_PLL_CLK_DIV */
	config_frame_list_buf[3][2] =
		(sensor_settings[iframe].clk.pre_pll_div >> 8) & 0xff;
	config_frame_list_buf[3][3] =
		sensor_settings[iframe].clk.pre_pll_div & 0xff;

	/* I2C_REG_PLL_MULTIPLIER */
	config_frame_list_buf[4][2] =
		(sensor_settings[iframe].clk.pll_mult >> 8) & 0xff;
	config_frame_list_buf[4][3] =
		sensor_settings[iframe].clk.pll_mult & 0xff;

	/* I2C_REG_OP_PIX_CLK_DIV */
	config_frame_list_buf[5][2] =
		(sensor_settings[iframe].clk.op_pix_clk_div >> 8) & 0xff;
	config_frame_list_buf[5][3] =
		sensor_settings[iframe].clk.op_pix_clk_div & 0xff;

	/* I2C_REG_OP_SYS_CLK_DIV */
	config_frame_list_buf[6][2] =
		(sensor_settings[iframe].clk.op_sys_clk_div >> 8) & 0xff;
	config_frame_list_buf[6][3] =
		sensor_settings[iframe].clk.op_sys_clk_div & 0xff;

	/* I2C_REG_X_OUTPUT_SIZE */
	config_frame_list_buf[7][2] =
		(sensor_settings[iframe].frame.x_output_size >> 8) & 0xff;
	config_frame_list_buf[7][3] =
		sensor_settings[iframe].frame.x_output_size & 0xff;

	/* I2C_REG_Y_OUTPUT_SIZE */
	config_frame_list_buf[8][2] =
		(sensor_settings[iframe].frame.y_output_size >> 8) & 0xff;
	config_frame_list_buf[8][3] =
		sensor_settings[iframe].frame.y_output_size & 0xff;

	/* I2C_REG_X_ADDR_START */
	config_frame_list_buf[9][2] =
		(sensor_settings[iframe].frame.x_addr_start >> 8) & 0xff;
	config_frame_list_buf[9][3] =
		sensor_settings[iframe].frame.x_addr_start & 0xff;

	/* I2C_REG_Y_ADDR_START */
	config_frame_list_buf[10][2] =
		(sensor_settings[iframe].frame.y_addr_start >> 8) & 0xff;
	config_frame_list_buf[10][3] =
		sensor_settings[iframe].frame.y_addr_start & 0xff;

	/* I2C_REG_X_ADDR_END */
	config_frame_list_buf[11][2] =
		(sensor_settings[iframe].frame.x_addr_end >> 8) & 0xff;
	config_frame_list_buf[11][3] =
		sensor_settings[iframe].frame.x_addr_end & 0xff;

	/* I2C_REG_Y_ADDR_END */
	config_frame_list_buf[12][2] =
		(sensor_settings[iframe].frame.y_addr_end >> 8) & 0xff;
	config_frame_list_buf[12][3] =
		sensor_settings[iframe].frame.y_addr_end & 0xff;

	/* I2C_REG_READ_MODE */
	data = (sensor_settings[iframe].frame.x_bin & 1) << 11;
	data |= (sensor_settings[iframe].frame.xy_bin & 1) << 10;
	data |= (sensor_settings[iframe].frame.x_odd_inc & 7) << 6;
	data |= (sensor_settings[iframe].frame.y_odd_inc & 0x3F);

	config_frame_list_buf[13][2] = (data >> 8) & 0xff;
	config_frame_list_buf[13][3] = data & 0xff;

	/* I2C_REG_FINE_INT_TIME */
	config_frame_list_buf[14][2] =
		(sensor_settings[iframe].exposure.fine_int_tm >> 8) & 0xff;
	config_frame_list_buf[14][3] =
		sensor_settings[iframe].exposure.fine_int_tm & 0xff;

	/* I2C_REG_FRAME_LEN_LINES */
	config_frame_list_buf[15][2] =
		(sensor_settings[iframe].frame.frame_len_lines >> 8) & 0xff;
	config_frame_list_buf[15][3] =
		sensor_settings[iframe].frame.frame_len_lines & 0xff;

	/* I2C_REG_LINE_LEN_PCK */
	config_frame_list_buf[16][2] =
		(sensor_settings[iframe].frame.line_len_pck >> 8) & 0xff;
	config_frame_list_buf[16][3] =
		sensor_settings[iframe].frame.line_len_pck & 0xff;

	/* I2C_REG_SCALE_M */
	config_frame_list_buf[17][2] =
		(sensor_settings[iframe].frame.scale_m >> 8) & 0xff;
	config_frame_list_buf[17][3] =
		sensor_settings[iframe].frame.scale_m & 0xff;

	/* I2C_REG_SCALING_MODE */
	config_frame_list_buf[18][2] =
		(sensor_settings[iframe].frame.scale_mode >> 8) & 0xff;
	config_frame_list_buf[18][3] =
		sensor_settings[iframe].frame.scale_mode & 0xff;

	/* I2C_REG_FINE_CORRECTION */
	config_frame_list_buf[19][2] =
		(sensor_settings[iframe].exposure.fine_correction >> 8) & 0xff;
	config_frame_list_buf[19][3] =
		sensor_settings[iframe].exposure.fine_correction & 0xff;

	/* I2C_REG_COARSE_INT_TIME */
	config_frame_list_buf[20][2] =
		(sensor_settings[iframe].exposure.coarse_int_tm >> 8) & 0xff;
	config_frame_list_buf[20][3] =
		sensor_settings[iframe].exposure.coarse_int_tm & 0xff;

	/* I2C_REG_MANUF_ANALOG_GAIN_GLOBAL */
	config_frame_list_buf[21][2] =
		(sensor_settings[iframe].exposure.analog_gain >> 8) & 0xff;
	config_frame_list_buf[21][3] =
		sensor_settings[iframe].exposure.analog_gain & 0xff;

	current_iframe = iframe;

	return 0;
}

/**
 * mt9p013_configure - Configure the mt9p013 for the specified image mode
 * @s: pointer to standard V4L2 device structure
 *
 * Configure the mt9p013 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the mt9p013.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int mt9p013_configure(struct v4l2_int_device *s)
{
	struct mt9p013_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &sensor->pix;
	struct i2c_client *client = sensor->i2c_client;
	struct vcontrol *lvc = NULL;
	int err = 0, i = 0;
	u32 xclk;
	enum mt9p013_image_size isize = 0;
	enum mt9p013_frame_type iframe = 0;

	isize = mt9p013_find_isize(pix->width);

	/* common register initialization */
	err = mt9p013_write_reg(client, I2C_MT9P013_8BIT,
			REG_SOFTWARE_RESET, 1);
	if (err)
		return err;
	mdelay(5);

	err = mt9p013_write_regs(client, initial_list,
		I2C_INITIAL_LIST_SIZE);
	if (err)
		return err;

	/* Determine Xclk & range check Frame Rate */
	xclk = mt9p013_calc_xclk(s);

	/* configure image size, pll, and pixel format */
	if (pix->pixelformat == V4L2_PIX_FMT_SGRBG10) {
		iframe = mt9p013_find_iframe(sensor->fps, isize);
		mt9p013_configure_frame(iframe);

		err = mt9p013_write_regs(client, config_frame_list,
			I2C_CONFIG_FRAME_LIST_SIZE);

		if (sensor->fps == 120) {
			err = mt9p013_write_regs(client, \
				frame_648_120fps_addendum, \
				FRAME_648_120fps_ADDENDUM_LIST_SIZE);
		}
		if (err)
			return err;

	} else if (pix->pixelformat == V4L2_PIX_FMT_W1S_PATT) {
		iframe = MT9P013_FRAME_5MP_10FPS;
		mt9p013_configure_frame(iframe);

		err = mt9p013_write_regs(client, config_frame_list,
			I2C_CONFIG_FRAME_LIST_SIZE);

		err = mt9p013_write_reg(client, I2C_MT9P013_16BIT,
					REG_TEST_PATTERN, 0x101);
		if (err)
			return err;
	}

	v4l_info(client, "mt9p013_configure: fps=%d, isize=%d, iframe=%d\n",
		sensor->fps, isize, iframe);

	if (err)
		return err;

	mt9p013_update_clocks(xclk, iframe);

	/* configure frame rate */
	err = mt9p013_set_framerate(s, &sensor->timeperframe, iframe);
	if (err)
		return err;

	/* Set initial exposure time */
	i = find_vctrl(V4L2_CID_EXPOSURE);
	if (i >= 0) {
		lvc = &video_control[i];
		mt9p013_set_exposure_time(lvc->current_value,
			sensor->v4l2_int_device, lvc);
	}

	/* Set initial gain */
	i = find_vctrl(V4L2_CID_GAIN);
	if (i >= 0) {
		lvc = &video_control[i];
		mt9p013_set_gain(lvc->current_value,
			sensor->v4l2_int_device, lvc);
	}

	if (pix->pixelformat != V4L2_PIX_FMT_W1S_PATT) {
		/* Set initial color bars */
		i = find_vctrl(V4L2_CID_PRIVATE_COLOR_BAR);
		if (i >= 0) {
			lvc = &video_control[i];
			mt9p013_set_color_bar_mode(lvc->current_value,
				sensor->v4l2_int_device, lvc);
		}
	}

	/* Set initial flash mode */
	i = find_vctrl(V4L2_CID_PRIVATE_FLASH_NEXT_FRAME);
	if (i >= 0) {
		lvc = &video_control[i];
		mt9p013_set_flash_next_frame(lvc->current_value,
			sensor->v4l2_int_device, lvc);
	}

	/* Set initial orientation */
	i = find_vctrl(V4L2_CID_PRIVATE_ORIENTATION);
	if (i >= 0) {
		lvc = &video_control[i];
		mt9p013_set_orientation(lvc->current_value,
					sensor->v4l2_int_device, lvc);
	}

	/* start streaming */
	mt9p013_write_reg(client, I2C_MT9P013_8BIT, REG_MODE_SELECT, 1);
	mdelay(1);

	return err;
}

/**
 * mt9p013_detect - Detect if an mt9p013 is present, and if so which revision
 * @s: pointer to standard V4L2 device structure
 *
 * Detect if an mt9p013 is present, and if so which revision.
 * A device is considered to be detected if the manufacturer ID (MIDH and MIDL)
 * and the product ID (PID) registers match the expected values.
 * Any value of the version ID (VER) register is accepted.
 * Here are the version numbers we know about:
 *	mt9p013 Revision 1 thru mt9p013 Revision 1
 * Returns a negative error number if no device is detected, or the
 * non-negative value of the version ID register if a device is detected.
 */
static int
mt9p013_detect(struct v4l2_int_device *s)
{
	struct mt9p013_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct mt9p013_sensor_id *sensor_id = &(sensor->sensor_id);
	u32 data, reset_data;
	u32 model_id, mfr_id, sensor_revision;

	if (!client)
		return -ENODEV;

	if (mt9p013_read_reg(client, I2C_MT9P013_16BIT,
				REG_MODEL_ID, &model_id))
		return -ENODEV;

	if (mt9p013_read_reg(client, I2C_MT9P013_8BIT,
				REG_MANUFACTURER_ID, &mfr_id))
		return -ENODEV;

	/* Get sensor rev */
	if (mt9p013_read_reg(client, I2C_MT9P013_16BIT,
				REG_RESET_REGISTER, &reset_data))
		return -ENODEV;

	if (mt9p013_write_reg(client, I2C_MT9P013_16BIT, REG_RESET_REGISTER,
				reset_data | (1 << RESET_REG_PLL_OFF_BP)))
		return -ENODEV;

	if (mt9p013_read_reg(client, I2C_MT9P013_16BIT,
				REG_SENSOR_REVISION, &data))
		return -ENODEV;

	sensor_revision = data & 0xf;
	if (mt9p013_write_reg(client, I2C_MT9P013_16BIT,
				REG_RESET_REGISTER, reset_data))
		return -ENODEV;

	sensor_id->model = model_id;
	sensor_id->mfr = mfr_id;
	sensor_id->revision = sensor_revision;

	dev_info(&client->dev, "model id detected 0x%x mfr 0x%x rev 0x%x\n",
			model_id, mfr_id, sensor_revision);

	if ((mfr_id != MT9P013_MFR_ID) || (model_id != MT9P013_MOD_ID)) {
		/* We didn't read the values we expected, so
		 * this must not be an MT9P013.
		 */
		dev_warn(&client->dev, "model id mismatch 0x%x mfr 0x%x" \
			"rev 0x%x\n", model_id, mfr_id, sensor_revision);

		return -ENODEV;
	}

	return 0;
}

/**
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

/**
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
	struct mt9p013_sensor *sensor = s->priv;
	struct vcontrol *lvc;
	int retval = 0;
	int i;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &video_control[i];

	switch (vc->id) {
	case  V4L2_CID_EXPOSURE:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_GAIN:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_PRIVATE_COLOR_BAR:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_PRIVATE_FLASH_NEXT_FRAME:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_PRIVATE_SENSOR_ID_REQ:
		if (copy_to_user((void *)vc->value, &(sensor->sensor_id),
				sizeof(sensor->sensor_id))) {
			retval = -EINVAL;
			printk(KERN_ERR "Failed copy_to_user\n");
		}
		break;
	case V4L2_CID_PRIVATE_ORIENTATION:
		vc->value = lvc->current_value;
		retval = 0;
		break;
	}

	return retval;
}

/**
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
	int retval = -EINVAL;
	int i;
	struct vcontrol *lvc;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		retval = mt9p013_set_exposure_time(vc->value, s, lvc);
		break;
	case V4L2_CID_GAIN:
		retval = mt9p013_set_gain(vc->value, s, lvc);
		break;
	case V4L2_CID_PRIVATE_COLOR_BAR:
		retval = mt9p013_set_color_bar_mode(vc->value, s, lvc);
		break;
	case V4L2_CID_PRIVATE_FLASH_NEXT_FRAME:
		retval = mt9p013_set_flash_next_frame(vc->value, s, lvc);
		break;
	case V4L2_CID_PRIVATE_ORIENTATION:
		retval = mt9p013_set_orientation(vc->value, s, lvc);
		break;
	}

	return retval;
}


/**
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

	fmt->flags = mt9p013_formats[index].flags;
	strlcpy(fmt->description, mt9p013_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = mt9p013_formats[index].pixelformat;

	return 0;
}

/**
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
	enum mt9p013_image_size isize;
	int ifmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct mt9p013_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix2 = &sensor->pix;

	isize = mt9p013_calc_size(pix->width, pix->height);

	pix->width = mt9p013_sizes[isize].width;
	pix->height = mt9p013_sizes[isize].height;
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == mt9p013_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = mt9p013_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_RGB555X:
	default:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}
	*pix2 = *pix;
	return 0;
}

/**
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
	struct mt9p013_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;

	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;
	else
		sensor->pix = *pix;

	return rval;
}

/**
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
	struct mt9p013_sensor *sensor = s->priv;
	f->fmt.pix = sensor->pix;

	return 0;
}

/**
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct mt9p013_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;

	return 0;
}

/**
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
	u32 xclk;
	struct mt9p013_sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

	sensor->timeperframe = *timeperframe;

	xclk = mt9p013_calc_xclk(s);
	mt9p013_update_clocks(xclk, current_iframe);
	mt9p013_set_framerate(s, &sensor->timeperframe, current_iframe);

	*timeperframe = sensor->timeperframe;

	return 0;
}

/**
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct mt9p013_sensor *sensor = s->priv;

	return sensor->pdata->priv_data_set(p);
}

/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power on)
{
	struct mt9p013_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	int rval = 0;

#if defined(CONFIG_VIDEO_MT9P012_MT9P013_AUTODETECT)
	c->addr = MT9P013_I2C_ADDR;
#endif

	if ((on == V4L2_POWER_STANDBY) && (sensor->state == SENSOR_DETECTED))
		mt9p013_write_reg(c, I2C_MT9P013_8BIT, REG_MODE_SELECT, 0);

	if (on != V4L2_POWER_ON)
		isp_set_xclk(0, MT9P013_USE_XCLKA);
	else
		isp_set_xclk(current_clk.x_clk, MT9P013_USE_XCLKA);

	rval = sensor->pdata->power_set(on);
	if (rval < 0) {
		dev_err(&c->dev, "Unable to set the power state: "
			DRIVER_NAME " sensor\n");
		isp_set_xclk(0, MT9P013_USE_XCLKA);
		return rval;
	}
	if ((current_power_state == V4L2_POWER_STANDBY) &&
					(on == V4L2_POWER_ON) &&
					(sensor->state == SENSOR_DETECTED)) {
		sensor->resuming = true;
		mt9p013_configure(s);
	}

	if ((on == V4L2_POWER_ON) && (sensor->state == SENSOR_NOT_DETECTED)) {
		rval = mt9p013_detect(s);
		if (rval < 0) {
			dev_err(&c->dev, "Unable to detect "
				DRIVER_NAME " sensor\n");
			sensor->state = SENSOR_NOT_DETECTED;
			return rval;
		}
		sensor->state = SENSOR_DETECTED;
		pr_info(DRIVER_NAME " chip revision 0x%02x detected\n",
			sensor->sensor_id.revision);
	}

	sensor->resuming = false;
	current_power_state = on;
	return 0;
}

/**
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call mt9p013_configure())
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
 * mt9p013 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct mt9p013_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	int rval;

	rval = mt9p013_detect(s);
	if (rval < 0) {
		dev_err(&c->dev, "Unable to detect " DRIVER_NAME " sensor\n");
		return rval;
	}

	sensor->sensor_id.revision = rval;
	pr_info(DRIVER_NAME " chip version 0x%02x detected\n",
		sensor->sensor_id.revision);

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
		if (frms->pixel_format == mt9p013_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= 5)
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = mt9p013_sizes[frms->index].width;
	frms->discrete.height = mt9p013_sizes[frms->index].height;

	return 0;
}

const struct v4l2_fract mt9p013_frameintervals[] = {
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
		if (frmi->pixel_format == mt9p013_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */

	if (((frmi->width == mt9p013_sizes[4].width) &&
				(frmi->height == mt9p013_sizes[4].height)) ||
				((frmi->width == mt9p013_sizes[3].width) &&
				(frmi->height == mt9p013_sizes[3].height))) {
		/* FIXME: The only frameinterval supported by 5MP and 3MP
		 * capture sizes is 1/11 fps
		 */
		if (frmi->index != 0)
			return -EINVAL;
	} else if ((frmi->width == mt9p013_sizes[1].width) &&
				(frmi->height == mt9p013_sizes[1].height)) {
		/* QVGA base size supports all framerates, including 120 fps!
		 */
		if (frmi->index >= 6)
			return -EINVAL;
	} else {
		if (frmi->index >= 5)
			return -EINVAL;
	}

	frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmi->discrete.numerator =
				mt9p013_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				mt9p013_frameintervals[frmi->index].denominator;

	return 0;
}

static struct v4l2_int_ioctl_desc mt9p013_ioctl_desc[] = {
	{ .num = vidioc_int_enum_framesizes_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ .num = vidioc_int_enum_frameintervals_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ .num = vidioc_int_dev_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ .num = vidioc_int_dev_exit_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_exit },
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_priv },
	{ .num = vidioc_int_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_init },
	{ .num = vidioc_int_enum_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ .num = vidioc_int_try_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_try_fmt_cap },
	{ .num = vidioc_int_g_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ .num = vidioc_int_s_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_fmt_cap },
	{ .num = vidioc_int_g_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ .num = vidioc_int_s_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ .num = vidioc_int_g_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_ctrl },
};

static struct v4l2_int_slave mt9p013_slave = {
	.ioctls = mt9p013_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9p013_ioctl_desc),
};

static struct v4l2_int_device mt9p013_int_device = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME,
	.priv = &mt9p013,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &mt9p013_slave,
	},
};

/**
 * mt9p013_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int
mt9p013_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mt9p013_sensor *sensor = &mt9p013;
	int err;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	sensor->pdata = client->dev.platform_data;

	if (!sensor->pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	sensor->v4l2_int_device = &mt9p013_int_device;
	sensor->i2c_client = client;

	i2c_set_clientdata(client, sensor);

	/* Make the default capture format QCIF V4L2_PIX_FMT_SGRBG10 */
	sensor->pix.width = VIDEO_WIDTH_4X_BINN_SCALED;
	sensor->pix.height = VIDEO_HEIGHT_4X_BINN_SCALED;
	sensor->pix.pixelformat = V4L2_PIX_FMT_SGRBG10;

	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err)
		i2c_set_clientdata(client, NULL);

	return err;
}

/**
 * mt9p013_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of mt9p013_probe().
 */
static int __exit
mt9p013_remove(struct i2c_client *client)
{
	struct mt9p013_sensor *sensor = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id mt9p013_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mt9p013_id);

static struct i2c_driver mt9p013_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mt9p013_probe,
	.remove = __exit_p(mt9p013_remove),
	.id_table = mt9p013_id,
};

/**
 * mt9p013sensor_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init mt9p013sensor_init(void)
{
	int err;
	err = i2c_add_driver(&mt9p013_i2c_driver);
	if (err) {
		printk(KERN_ERR "Failed to register" DRIVER_NAME ".\n");
		return err;
	}
	return 0;
}
late_initcall(mt9p013sensor_init);

/**
 * mt9p013sensor_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of mt9p013sensor_init.
 */
static void __exit mt9p013sensor_cleanup(void)
{
	i2c_del_driver(&mt9p013_i2c_driver);
}
module_exit(mt9p013sensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mt9p013 camera sensor driver");
