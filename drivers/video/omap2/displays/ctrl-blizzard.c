
//#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <mach/display.h>
#include <mach/dma.h>

#ifdef DEBUG
#define DBG(format, ...) printk(KERN_DEBUG "Blizzard: " format, ## __VA_ARGS__)
#else
#define DBG(format, ...)
#endif

#define BLIZZARD_REV_CODE			0x00
#define BLIZZARD_CONFIG				0x02
#define BLIZZARD_PLL_DIV			0x04
#define BLIZZARD_PLL_LOCK_RANGE			0x06
#define BLIZZARD_PLL_CLOCK_SYNTH_0		0x08
#define BLIZZARD_PLL_CLOCK_SYNTH_1		0x0a
#define BLIZZARD_PLL_MODE			0x0c
#define BLIZZARD_CLK_SRC			0x0e
#define BLIZZARD_MEM_BANK0_ACTIVATE		0x10
#define BLIZZARD_MEM_BANK0_STATUS		0x14
#define BLIZZARD_PANEL_CONFIGURATION		0x28
#define BLIZZARD_HDISP				0x2a
#define BLIZZARD_HNDP				0x2c
#define BLIZZARD_VDISP0				0x2e
#define BLIZZARD_VDISP1				0x30
#define BLIZZARD_VNDP				0x32
#define BLIZZARD_HSW				0x34
#define BLIZZARD_VSW				0x38
#define BLIZZARD_DISPLAY_MODE			0x68
#define BLIZZARD_INPUT_WIN_X_START_0		0x6c
#define BLIZZARD_DATA_SOURCE_SELECT		0x8e
#define BLIZZARD_DISP_MEM_DATA_PORT		0x90
#define BLIZZARD_DISP_MEM_READ_ADDR0		0x92
#define BLIZZARD_POWER_SAVE			0xE6
#define BLIZZARD_NDISP_CTRL_STATUS		0xE8

/* Data source select */
/* For S1D13745 */
#define BLIZZARD_SRC_WRITE_LCD_BACKGROUND	0x00
#define BLIZZARD_SRC_WRITE_LCD_DESTRUCTIVE	0x01
#define BLIZZARD_SRC_WRITE_OVERLAY_ENABLE	0x04
#define BLIZZARD_SRC_DISABLE_OVERLAY		0x05
/* For S1D13744 */
#define BLIZZARD_SRC_WRITE_LCD			0x00
#define BLIZZARD_SRC_BLT_LCD			0x06

#define BLIZZARD_COLOR_RGB565			0x01
#define BLIZZARD_COLOR_YUV420			0x09

#define BLIZZARD_VERSION_S1D13745		0x01	/* Hailstorm */
#define BLIZZARD_VERSION_S1D13744		0x02	/* Blizzard */

#define BLIZZARD_AUTO_UPDATE_TIME		(HZ / 20)



static struct {
	int			version;
} blizzard;


static inline void blizzard_cmd(u8 cmd)
{
        omap_rfbi_write_command(&cmd, 1);
}

static inline void blizzard_write(u8 cmd, const u8 *buf, int len)
{
        omap_rfbi_write_command(&cmd, 1);
        omap_rfbi_write_data(buf, len);
}

static inline void blizzard_read(u8 cmd, u8 *buf, int len)
{
        omap_rfbi_write_command(&cmd, 1);
        omap_rfbi_read_data(buf, len);
}

static u8 blizzard_read_reg(u8 cmd)
{
	u8 data;
	blizzard_read(cmd, &data, 1);
	return data;
}

static int blizzard_ctrl_init(struct omap_display *display)
{
	DBG("blizzard_ctrl_init\n");

	return 0;
}


static int blizzard_ctrl_enable(struct omap_display *display)
{
	int r = 0;
	u8 rev, conf;

	DBG("blizzard_ctrl_enable\n");

	if (display->hw_config.ctrl_enable) {
		r = display->hw_config.ctrl_enable(display);
		if (r)
			return r;
	}

	msleep(100);

	rev = blizzard_read_reg(BLIZZARD_CLK_SRC);
	printk("CLK_SRC %x\n", rev);

	rev = blizzard_read_reg(BLIZZARD_PLL_DIV);
	printk("PLLDIV %x\n", rev);

	rev = blizzard_read_reg(BLIZZARD_REV_CODE);
	conf = blizzard_read_reg(BLIZZARD_CONFIG);

	printk("rev %x, conf %x\n", rev, conf);

	switch (rev & 0xfc) {
	case 0x9c:
		blizzard.version = BLIZZARD_VERSION_S1D13744;
		pr_info("omapfb: s1d13744 LCD controller rev %d "
			"initialized (CNF pins %x)\n", rev & 0x03, conf & 0x07);
		break;
	case 0xa4:
		blizzard.version = BLIZZARD_VERSION_S1D13745;
		pr_info("omapfb: s1d13745 LCD controller rev %d "
			"initialized (CNF pins %x)\n", rev & 0x03, conf & 0x07);
		break;
	default:
		printk("invalid s1d1374x revision %02x\n",
			rev);
		r = -ENODEV;
	}

	return r;
}

static void blizzard_ctrl_disable(struct omap_display *display)
{
	DBG("blizzard_ctrl_disable\n");

	if (display->hw_config.ctrl_disable)
		display->hw_config.ctrl_disable(display);
}

int rfbi_configure(int rfbi_module, int bpp, int lines);

static void blizzard_ctrl_setup_update(struct omap_display *display,
				    u16 x, u16 y, u16 w, u16 h)
{
	u8 tmp[18];
	int x_end, y_end;

	DBG("blizzard_ctrl_setup_update\n");

	x_end = x + w - 1;
	y_end = y + h - 1;

	tmp[0] = x;
	tmp[1] = x >> 8;
	tmp[2] = y;
	tmp[3] = y >> 8;
	tmp[4] = x_end;
	tmp[5] = x_end >> 8;
	tmp[6] = y_end;
	tmp[7] = y_end >> 8;

	/* scaling? */
	tmp[8] = x;
	tmp[9] = x >> 8;
	tmp[10] = y;
	tmp[11] = y >> 8;
	tmp[12] = x_end;
	tmp[13] = x_end >> 8;
	tmp[14] = y_end;
	tmp[15] = y_end >> 8;

	tmp[16] = BLIZZARD_COLOR_RGB565; //color_mode;

	if (blizzard.version == BLIZZARD_VERSION_S1D13745)
		tmp[17] = BLIZZARD_SRC_WRITE_LCD_BACKGROUND;
	else
		tmp[17] = blizzard.version == BLIZZARD_VERSION_S1D13744 ?
				BLIZZARD_SRC_WRITE_LCD :
				BLIZZARD_SRC_WRITE_LCD_DESTRUCTIVE;

	rfbi_configure(display->hw_config.u.rfbi.channel,
		       16,
		       8);

	blizzard_write(BLIZZARD_INPUT_WIN_X_START_0, tmp, 18);

	rfbi_configure(display->hw_config.u.rfbi.channel,
		       16,
		       16);
}

static int blizzard_ctrl_enable_te(struct omap_display *display, bool enable)
{
	return 0;
}

static int blizzard_ctrl_rotate(struct omap_display *display, u8 rotate)
{
	return 0;
}

static int blizzard_ctrl_mirror(struct omap_display *display, bool enable)
{
	return 0;
}

static int blizzard_run_test(struct omap_display *display, int test_num)
{
	return 0;
}

static struct omap_ctrl blizzard_ctrl = {
	.owner = THIS_MODULE,
	.name = "ctrl-blizzard",
	.init = blizzard_ctrl_init,
	.enable = blizzard_ctrl_enable,
	.disable = blizzard_ctrl_disable,
	.setup_update = blizzard_ctrl_setup_update,
	.enable_te = blizzard_ctrl_enable_te,
	.set_rotate = blizzard_ctrl_rotate,
	.set_mirror = blizzard_ctrl_mirror,
	.run_test = blizzard_run_test,
	.pixel_size = 16,

	.timings = {
                .cs_on_time     = 0,

                .we_on_time     = 9000,
                .we_off_time    = 18000,
                .we_cycle_time  = 36000,

                .re_on_time     = 9000,
                .re_off_time    = 27000,
                .re_cycle_time  = 36000,

                .access_time    = 27000,
                .cs_off_time    = 36000,

                .cs_pulse_width = 0,
        },
};


static int __init blizzard_init(void)
{
	DBG("blizzard_init\n");
	omap_dss_register_ctrl(&blizzard_ctrl);
	return 0;
}

static void __exit blizzard_exit(void)
{
	DBG("blizzard_exit\n");

	omap_dss_unregister_ctrl(&blizzard_ctrl);
}

module_init(blizzard_init);
module_exit(blizzard_exit);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@nokia.com>");
MODULE_DESCRIPTION("Blizzard Driver");
MODULE_LICENSE("GPL");
