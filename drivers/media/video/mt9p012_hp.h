/*
 * drivers/media/video/mt9p012.h
 *
 * Register definitions for the MT9P012 camera sensor.
 *
 * Author:
 *	Sameer Venkatraman <sameerv@ti.com>
 *	Martinez Leonides
 *	Greg Hofer <greg.hofer@hp.com>
 *
 *
 * Copyright (C) 2008 Texas Instruments.
 * Copyright (C) 2007-2008 Motorola, Inc.
 * Copyright (C) 2009 Hewlett Packard
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * History:
 * 11-Dec-2008  Motorola         Updated to add support for Skarda HW
 * 24-Feb-2009  Motorola         Updated for revision 7 sensor changes
 * 05-Jun-2009  Hewlett Packard  Allow on the fly frame rate changes
 */

#ifndef MT9P012_H
#define MT9P012_H

#define ALLOW_EXPOSURE_TIME_FRAME_EXTENSION
#define CONFIG_MT9P012_I2C_MIPI_BASE_ADDR

#if defined(CONFIG_MT9P012_I2C_MIPI_BASE_ADDR)
    #define MT9P012_I2C_ADDR		0x36  /* MIPI */
#else
    #define MT9P012_I2C_ADDR		0x10  /* SMIA */
#endif

/* ISP Private IOCTLs */
#define V4L2_CID_PRIVATE_SENSOR_ID_REQ		(V4L2_CID_PRIVATE_BASE + 22)
#define V4L2_CID_PRIVATE_COLOR_BAR     		(V4L2_CID_PRIVATE_BASE + 23)
#define V4L2_CID_PRIVATE_FLASH_NEXT_FRAME	(V4L2_CID_PRIVATE_BASE + 24)
#define V4L2_CID_PRIVATE_ORIENTATION     	(V4L2_CID_PRIVATE_BASE + 25)

/* The ID values we are looking for */
#define MT9P012_MOD_ID_REV1_6	0x2800
#define MT9P012_MOD_ID_REV7_8	0x2801
#define MT9P012_MFR_ID		0x0006

/* MT9P012 has 8/16/32 registers */
#define I2C_MT9P012_8BIT		1
#define I2C_MT9P012_16BIT		2
#define I2C_MT9P012_32BIT		4

/* terminating token for reg list */
#define MT9P012_TOK_TERM		0xFF

/* delay token for reg list */
#define MT9P012_TOK_DELAY		100

/* Sensor specific GPIO signals */
#define MT9P012_RESET_GPIO  	98
#define MT9P012_STANDBY_GPIO	58

#define VAUX_2_8_V		0x09
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00

#define DEBUG_BASE		0x08000000
#define REG_SDP3430_FPGA_GPIO_2 (0x50)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)

/* terminating list entry for reg */
#define MT9P012_REG_TERM		0xFF
/* terminating list entry for val */
#define MT9P012_VAL_TERM		0xFF

#define MT9P012_CLKRC			0x11

/* Used registers */
#define REG_MODEL_ID			0x0000
#define REG_REVISION_NUMBER		0x0002
#define REG_MANUFACTURER_ID		0x0003

#define REG_MODE_SELECT			0x0100
#define REG_IMAGE_ORIENTATION	0x0101
#define REG_SOFTWARE_RESET		0x0103
#define REG_GROUPED_PAR_HOLD	0x0104

#define REG_FINE_INT_TIME		0x0200
#define REG_COARSE_INT_TIME		0x0202

#define REG_ANALOG_GAIN_GLOBAL	0x0204
#define REG_ANALOG_GAIN_GREENR	0x0206
#define REG_ANALOG_GAIN_RED		0x0208
#define REG_ANALOG_GAIN_BLUE	0x020A
#define REG_ANALOG_GAIN_GREENB	0x020C
#define REG_DIGITAL_GAIN_GREENR	0x020E
#define REG_DIGITAL_GAIN_RED	0x0210
#define REG_DIGITAL_GAIN_BLUE	0x0212
#define REG_DIGITAL_GAIN_GREENB	0x0214

#define REG_VT_PIX_CLK_DIV		0x0300
#define REG_VT_SYS_CLK_DIV		0x0302
#define REG_PRE_PLL_CLK_DIV		0x0304
#define REG_PLL_MULTIPLIER		0x0306
#define REG_OP_PIX_CLK_DIV		0x0308
#define REG_OP_SYS_CLK_DIV		0x030A

#define REG_FRAME_LEN_LINES		0x0340
#define REG_LINE_LEN_PCK		0x0342

#define REG_X_ADDR_START		0x0344
#define REG_Y_ADDR_START		0x0346
#define REG_X_ADDR_END			0x0348
#define REG_Y_ADDR_END			0x034A
#define REG_X_OUTPUT_SIZE		0x034C
#define REG_Y_OUTPUT_SIZE		0x034E
#define REG_X_ODD_INC			0x0382
#define REG_Y_ODD_INC			0x0386

#define REG_SCALING_MODE		0x0400
#define REG_SCALE_M				0x0404
#define REG_SCALE_N				0x0406

#define REG_ROW_SPEED			0x3016
#define REG_RESET_REGISTER		0x301A
#define RESET_REG_PLL_OFF_BP  5
#define REG_PIXEL_ORDER			0x3024
#define REG_GPI_STATUS			0x3026
#define REG_READ_MODE			0x3040
#define REG_FLASH				0x3046
#define REG_MANUF_ANALOG_GAIN_GLOBAL	0x305E

#define REG_DATAPATH_STATUS		0x306A
#define REG_DATAPATH_SELECT		0x306E

#define REG_RESERVED_MFR_3064	0x3064
#define REG_TEST_PATTERN		0x3070
#define REG_SENSOR_REVISION		0x31FE


/*
 * The nominal xclk input frequency of the MT9P012 is 12MHz, maximum
 * frequency is 64MHz, and minimum frequency is 2MHz.
 */
#define MT9P012_XCLK_MIN   2000000
#define MT9P012_XCLK_MAX   64000000
#define MT9P012_XCLK_NOM_1 12000000
#define MT9P012_XCLK_NOM_2 24000000

#define MT9P012_USE_XCLKA	0
#define MT9P012_USE_XCLKB	1


/* FPS Capabilities */
#define MT9P012_MIN_FPS		5
#define MT9P012_DEF_FPS		15
#define MT9P012_MAX_FPS		30

#define I2C_RETRY_COUNT		5

/* Still capture 5 MP */
#define IMAGE_WIDTH_MAX		2592
#define IMAGE_HEIGHT_MAX	1944

/* Video mode, for D1 NTSC, D1 PAL */
#define VIDEO_WIDTH_2X_BINN	1296
#define VIDEO_HEIGHT_2X_BINN	972

/* Sensor Video mode size for VGA, CIF, QVGA in 4x binning mode */
#define VIDEO_WIDTH_4X_BINN		648
#define VIDEO_HEIGHT_4X_BINN	486
/* To improve image quality in VGA */
#define CIF_PIXELS			(352 * 288)
#define QQVGA_PIXELS		(160 * 120)

/* Video mode, for QCIF, SQCIF */
#define VIDEO_WIDTH_4X_BINN_SCALED	216
#define VIDEO_HEIGHT_4X_BINN_SCALED	162

/* Analog gain values */
#define MT9P012_MIN_ANALOG_GAIN	0x0034
#define MT9P012_MAX_ANALOG_GAIN	0x01FF

#define MT9P012_MIN_LINEAR_GAIN	((u16)(2.0 * 256))
#define MT9P012_MAX_LINEAR_GAIN	((u16)(15.875 * 256))

#define MT9P012_MAX_FRAME_LENGTH_LINES 0xFFFF

#define DEF_LINEAR_GAIN	((u16)(2.0 * 256))
#define LINEAR_GAIN_STEP	0x1

#define GAIN_INDEX	1

/* Exposure time values */
#define DEF_MIN_EXPOSURE	0x08
#define DEF_MAX_EXPOSURE	0x7F
#define DEF_EXPOSURE	    0x43
#define EXPOSURE_STEP	    1
#define SENSOR_DETECTED		1
#define SENSOR_NOT_DETECTED	0

/**
 * struct mt9p012_reg - mt9p012 register format
 * @length: length of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for MT9P012 register initialization values
 */
struct mt9p012_reg {
	u16	length;
	u16	reg;
	u32	val;
};

enum mt9p012_image_size {
	MT9P012_BIN4XSCALE,
	MT9P012_BIN4X,
	MT9P012_BIN2X,
	MT9P012_THREE_MP,
	MT9P012_FIVE_MP
} ;

enum mt9p012_frame_type {
	MT9P012_FRAME_5MP_10FPS = 0,
	MT9P012_FRAME_3MP_10FPS,
	MT9P012_FRAME_1296_30FPS,
	MT9P012_FRAME_648_30FPS,
	MT9P012_FRAME_648_120FPS,
	MT9P012_FRAME_216_30FPS
};

enum mt9p012_pixel_format {
	MT9P012_RAWBAYER10
};

enum mt9p012_orientation {
	MT9P012_NO_HORZ_FLIP_OR_VERT_FLIP = 0,
	MT9P012_HORZ_FLIP_ONLY,
	MT9P012_VERT_FLIP_ONLY,
	MT9P012_HORZ_FLIP_AND_VERT_FLIP
};

#define NUM_IMAGE_SIZES		5
#define NUM_PIXEL_FORMATS	1
#define NUM_FPS			2	/* 2 ranges */
#define FPS_LOW_RANGE		0
#define FPS_HIGH_RANGE		1

/**
 * struct capture_size - image capture size information
 * @width: image width in pixels
 * @height: image height in pixels
 */
struct mt9p012_capture_size {
	unsigned long width;
	unsigned long height;
};

/**
 * struct mt9p012_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @default_regs: Default registers written after power-on or reset.
 * @ifparm: Interface parameters access function
 * @priv_data_set: device private data (pointer) access function
 */
struct mt9p012_platform_data {
	int (*power_set)(enum v4l2_power power);
	const struct mt9p012_reg *default_regs;
	int (*ifparm)(struct v4l2_ifparm *p);
	int (*priv_data_set)(void *);
};

/**
 * struct struct clk_settings - struct for storage of sensor
 * clock settings
 * @pre_pll_div: pre pll divider
 * @pll_mult: pll multiplier
 * @vt_pix_clk_div: video pixel clock divider
 * @vt_sys_clk_div: video system clock divider
 * @op_pix_clk_div: output pixel clock divider
 * @op_sys_clk_div: output system clock divider
 * @min_pll: minimum pll multiplier
 * @max_pll: maximum pll multiplier
 */
struct mt9p012_clk_settings {
	u16	pre_pll_div;
	u16	pll_mult;
	u16	vt_pix_clk_div;
	u16	vt_sys_clk_div;
	u16	op_pix_clk_div;
	u16	op_sys_clk_div;
	u16	min_pll;
	u16	max_pll;
};

/**
 * struct struct frame_settings - struct for storage of sensor
 * frame settings
 * @frame_len_lines: number of lines in frame
 * @line_len_pck: number of pixels in line
 */
struct mt9p012_frame_settings {
	u16	frame_len_lines_min;
	u16	frame_len_lines;
	u16	line_len_pck;
	u16	x_addr_start;
	u16	x_addr_end;
	u16	y_addr_start;
	u16	y_addr_end;
	u16	x_output_size;
	u16	y_output_size;
	u16	x_odd_inc;
	u16	y_odd_inc;
	u16	x_bin;
	u16	xy_bin;
	u16	scale_m;
	u16	scale_mode;
};

/**
 * struct struct exposure_settings - struct for storage of sensor
 * initial exposure settings
 * @coarse_int_tm: coarse resolution interval time (line times)
 * @fine_int_tm: fine resolution interval time (pixel times)
 */
struct mt9p012_exposure_settings {
	u16	coarse_int_tm;
	u16	fine_int_tm;
	u16	fine_correction;
	u16	analog_gain;
};

/**
 * struct struct mt9p012_sensor_settings - struct for storage of
 * sensor settings.
 */
struct mt9p012_sensor_settings {
	struct mt9p012_clk_settings clk;
	struct mt9p012_frame_settings frame;
	struct mt9p012_exposure_settings exposure;
};

/**
 * struct struct mt9p012_clock_freq - struct for storage of sensor
 * clock frequencies
 */
struct mt9p012_clock_freq {
	u32 x_clk;
	u32 pll_clk;
	u32 vt_pix_clk;
	u32 op_pix_clk;
};

/*
 * Array of image sizes supported by MT9P012.  These must be ordered from
 * smallest image size to largest.
 */
const static struct mt9p012_capture_size mt9p012_sizes[] = {
	{  216, 162 },	/* 4X BINNING+SCALING */
	{  648, 486 },	/* 4X BINNING */
	{ 1296, 972 },	/* 2X BINNING */
	{ 2048, 1536},	/* 3 MP */
	{ 2592, 1944},	/* 5 MP */
};

#endif /* ifndef MT9P012_H */
