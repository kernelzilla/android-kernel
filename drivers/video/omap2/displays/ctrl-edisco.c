
//#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>

#include <mach/display.h>
#include <mach/dma.h>

#ifdef DEBUG
#define DBG(format, ...) (printk(KERN_DEBUG "Edisco: " format, ## __VA_ARGS__))
#else
#define DBG(format, ...)
#endif

#define EDISCO_CMD_SOFT_RESET		0x01
#define EDISCO_CMD_ENTER_SLEEP_MODE	0x10
#define EDISCO_CMD_EXIT_SLEEP_MODE	0x11
#define EDISCO_CMD_SET_DISPLAY_ON	0x29
#define EDISCO_CMD_SET_DISPLAY_OFF	0x28
#define EDISCO_CMD_SET_COLUMN_ADDRESS	0x2A
#define EDISCO_CMD_SET_PAGE_ADDRESS	0x2B

#define EDISCO_CMD_VC   0
#define EDISCO_VIDEO_VC 1

#define EDISCO_LONG_WRITE	0x29
#define EDISCO_SHORT_WRITE_1	0x23
#define EDISCO_SHORT_WRITE_0	0x13


static int edisco_ctrl_init(struct omap_display *display)
{
	return 0;
}

static int edisco_enable(struct omap_display *display)
{
	u8 data[7];
	int ret;

	printk("EDISCO CTRL ENABLE\n");
	if (display->hw_config.ctrl_enable) {
		ret = display->hw_config.ctrl_enable(display);
		if (ret)
			return ret;
	}

	/* turn of mcs register acces protection */
	data[0] = 0xb2;
	data[1] = 0x00;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1, data, 2);

	/* enable lane setting and test registers*/
	data[0] = 0xef;
	data[1] = 0x01;
	data[2] = 0x01;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 3);

	/* 2nd param 61 = 1 line; 63 = 2 lanes */
	data[0] = 0xef;
	data[1] = 0x60;
	data[2] = 0x63;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 3);

	/* 2nd param 0 = WVGA; 1 = WQVGA */
	data[0] = 0xb3;
	data[1] = 0x00;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 (auto BL control OFF);
	 * D[1]=0 (Grama correction On);
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = 0xb4;
	data[1] = 0x1f;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);

	/* set page, column address */
	data[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = (display->panel->timings.y_res - 1) >> 8;
	data[4] = (display->panel->timings.y_res - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		goto error;

	data[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = (display->panel->timings.x_res - 1) >> 8;
	data[4] = (display->panel->timings.x_res - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		goto error;

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	//ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_0, data, 1);
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);

	mdelay(200);

	data[0] = EDISCO_CMD_SET_DISPLAY_ON;
	//ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_0, data, 1);
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
	if (ret)
		goto error;

	printk("done EDISCO CTRL ENABLE\n");
	return 0;
error:
	return -EINVAL;
}

static void edisco_disable(struct omap_display *display)
{
	u8 data[1];

	DBG("edisco_ctrl_disable\n");

	data[0] = EDISCO_CMD_SET_DISPLAY_OFF;
	dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);

	data[0] = EDISCO_CMD_ENTER_SLEEP_MODE;
	dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);

	if (display->hw_config.ctrl_disable)
		display->hw_config.ctrl_disable(display);

}

static void edisco_ctrl_setup_update(struct omap_display *display,
				    u16 x, u16 y, u16 w, u16 h)
{

	u8 data[5];
	int ret;

	/* set page, column address */
	data[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	data[1] = y >> 8;
	data[2] = y & 0xff;
	data[3] = (y + h - 1) >> 8;
	data[4] = (y + h - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		return;

	data[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	data[1] = x >> 8;
	data[2] = x & 0xff;
	data[3] = (x + w - 1) >> 8;
	data[4] = (x + w - 1) & 0xff;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
	if (ret)
		return;
}

static int edisco_ctrl_enable_te(struct omap_display *display, bool enable)
{
	return 0;
}

static int edisco_ctrl_rotate(struct omap_display *display, u8 rotate)
{
	return 0;
}

static int edisco_ctrl_mirror(struct omap_display *display, bool enable)
{
	return 0;
}

static int edisco_run_test(struct omap_display *display, int test_num)
{
	return 0;
}

static struct omap_ctrl edisco_ctrl = {
	.owner = THIS_MODULE,
	.name = "ctrl-edisco",
	.init = edisco_ctrl_init,
	.enable = edisco_enable,
	.disable = edisco_disable,
	/* suspend & resume */
	.setup_update = edisco_ctrl_setup_update,
	.enable_te = edisco_ctrl_enable_te,
	.set_rotate = edisco_ctrl_rotate,
	.set_mirror = edisco_ctrl_mirror,
	.run_test = edisco_run_test,
	.pixel_size = 24,
};


static int __init edisco_init(void)
{
	DBG("edisco_init\n");
	omap_dss_register_ctrl(&edisco_ctrl);
	return 0;
}

static void __exit edisco_exit(void)
{
	DBG("edisco_exit\n");

	omap_dss_unregister_ctrl(&edisco_ctrl);
}

module_init(edisco_init);
module_exit(edisco_exit);

MODULE_AUTHOR("Rebecca Schultz Zavin <rebecca@android.com>");
MODULE_DESCRIPTION("Edisco Driver");
MODULE_LICENSE("GPL");
