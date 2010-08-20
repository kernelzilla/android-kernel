
/*#define DEBUG*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#include <mach/display.h>
#include <mach/dma.h>

#define MIPID_CMD_READ_DISP_ID		0x04
#define MIPID_CMD_READ_RED		0x06
#define MIPID_CMD_READ_GREEN		0x07
#define MIPID_CMD_READ_BLUE		0x08
#define MIPID_CMD_READ_DISP_STATUS	0x09
#define MIPID_CMD_RDDSDR		0x0F
#define MIPID_CMD_SLEEP_IN		0x10
#define MIPID_CMD_SLEEP_OUT		0x11
#define MIPID_CMD_DISP_OFF		0x28
#define MIPID_CMD_DISP_ON		0x29

#define MIPID_VER_LPH8923		3
#define MIPID_VER_LS041Y3		4

#define MIPID_ESD_CHECK_PERIOD		msecs_to_jiffies(5000)

#ifdef DEBUG
#define DBG(format, ...) printk(KERN_DEBUG "PN800: " format, ## __VA_ARGS__)
#else
#define DBG(format, ...)
#endif

struct pn800_device {
	struct backlight_device *bl_dev;
	int		enabled;
	int		model;
	int		revision;
	u8		display_id[3];
	unsigned int	saved_bklight_level;
	unsigned long	hw_guard_end;		/* next value of jiffies
						   when we can issue the
						   next sleep in/out command */
	unsigned long	hw_guard_wait;		/* max guard time in jiffies */

	struct spi_device	*spi;
	struct mutex		mutex;
	struct omap_panel	panel;
	struct omap_display	*display;
};


static void pn800_transfer(struct pn800_device *md, int cmd,
			      const u8 *wbuf, int wlen, u8 *rbuf, int rlen)
{
	struct spi_message	m;
	struct spi_transfer	*x, xfer[4];
	u16			w;
	int			r;

	BUG_ON(md->spi == NULL);

	spi_message_init(&m);

	memset(xfer, 0, sizeof(xfer));
	x = &xfer[0];

	cmd &=  0xff;
	x->tx_buf = &cmd;
	x->bits_per_word = 9;
	x->len = 2;
	spi_message_add_tail(x, &m);

	if (wlen) {
		x++;
		x->tx_buf = wbuf;
		x->len = wlen;
		x->bits_per_word = 9;
		spi_message_add_tail(x, &m);
	}

	if (rlen) {
		x++;
		x->rx_buf = &w;
		x->len = 1;
		spi_message_add_tail(x, &m);

		if (rlen > 1) {
			/* Arrange for the extra clock before the first
			 * data bit.
			 */
			x->bits_per_word = 9;
			x->len		 = 2;

			x++;
			x->rx_buf	 = &rbuf[1];
			x->len		 = rlen - 1;
			spi_message_add_tail(x, &m);
		}
	}

	r = spi_sync(md->spi, &m);
	if (r < 0)
		dev_dbg(&md->spi->dev, "spi_sync %d\n", r);

	if (rlen)
		rbuf[0] = w & 0xff;
}

static inline void pn800_cmd(struct pn800_device *md, int cmd)
{
	pn800_transfer(md, cmd, NULL, 0, NULL, 0);
}

static inline void pn800_write(struct pn800_device *md,
			       int reg, const u8 *buf, int len)
{
	pn800_transfer(md, reg, buf, len, NULL, 0);
}

static inline void pn800_read(struct pn800_device *md,
			      int reg, u8 *buf, int len)
{
	pn800_transfer(md, reg, NULL, 0, buf, len);
}

static void set_data_lines(struct pn800_device *md, int data_lines)
{
	u16 par;

	switch (data_lines) {
	case 16:
		par = 0x150;
		break;
	case 18:
		par = 0x160;
		break;
	case 24:
		par = 0x170;
		break;
	}
	pn800_write(md, 0x3a, (u8 *)&par, 2);
}

static void send_init_string(struct pn800_device *md)
{
	u16 initpar[] = { 0x0102, 0x0100, 0x0100 };
	int data_lines;

	pn800_write(md, 0xc2, (u8 *)initpar, sizeof(initpar));

	data_lines = (int)md->display->hw_config.panel_data; // XXX

	set_data_lines(md, data_lines);
}

static void hw_guard_start(struct pn800_device *md, int guard_msec)
{
	md->hw_guard_wait = msecs_to_jiffies(guard_msec);
	md->hw_guard_end = jiffies + md->hw_guard_wait;
}

static void hw_guard_wait(struct pn800_device *md)
{
	unsigned long wait = md->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= md->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static void set_sleep_mode(struct pn800_device *md, int on)
{
	int cmd, sleep_time = 50;

	if (on)
		cmd = MIPID_CMD_SLEEP_IN;
	else
		cmd = MIPID_CMD_SLEEP_OUT;
	hw_guard_wait(md);
	pn800_cmd(md, cmd);
	hw_guard_start(md, 120);
	/*
	 * When we enable the panel, it seems we _have_ to sleep
	 * 120 ms before sending the init string. When disabling the
	 * panel we'll sleep for the duration of 2 frames, so that the
	 * controller can still provide the PCLK,HS,VS signals. */
	if (!on)
		sleep_time = 120;
	msleep(sleep_time);
}

static void set_display_state(struct pn800_device *md, int enabled)
{
	int cmd = enabled ? MIPID_CMD_DISP_ON : MIPID_CMD_DISP_OFF;

	pn800_cmd(md, cmd);
}

static int panel_enabled(struct pn800_device *md)
{
	u32 disp_status;
	int enabled;

	pn800_read(md, MIPID_CMD_READ_DISP_STATUS, (u8 *)&disp_status, 4);
	disp_status = __be32_to_cpu(disp_status);
	enabled = (disp_status & (1 << 17)) && (disp_status & (1 << 10));
	dev_dbg(&md->spi->dev,
		"LCD panel %s enabled by bootloader (status 0x%04x)\n",
		enabled ? "" : "not ", disp_status);
	DBG("status %#08x\n", disp_status);
	return enabled;
}

static int panel_detect(struct pn800_device *md)
{
	pn800_read(md, MIPID_CMD_READ_DISP_ID, md->display_id, 3);
	dev_dbg(&md->spi->dev, "MIPI display ID: %02x%02x%02x\n",
		md->display_id[0], md->display_id[1], md->display_id[2]);

	switch (md->display_id[0]) {
	case 0x45:
		md->model = MIPID_VER_LPH8923;
		md->panel.name = "lph8923";
		break;
	case 0x83:
		md->model = MIPID_VER_LS041Y3;
		md->panel.name = "ls041y3";
		//md->esd_check = ls041y3_esd_check;
		break;
	default:
		md->panel.name = "unknown";
		dev_err(&md->spi->dev, "invalid display ID\n");
		return -ENODEV;
	}

	md->revision = md->display_id[1];
	pr_info("omapfb: %s rev %02x LCD detected\n",
			md->panel.name, md->revision);

	return 0;
}



static int pn800_panel_enable(struct omap_display *display)
{
	int r;
	struct pn800_device *md =
		(struct pn800_device *)display->panel->priv;

	DBG("pn800_panel_enable\n");

	mutex_lock(&md->mutex);

	if (display->hw_config.panel_enable)
		display->hw_config.panel_enable(display);

	msleep(50); // wait for power up

	r = panel_detect(md);
	if (r) {
		mutex_unlock(&md->mutex);
		return r;
	}

	md->enabled = panel_enabled(md);

	if (md->enabled) {
		DBG("panel already enabled\n");
		; /*pn800_esd_start_check(md);*/
	} else {
		; /*md->saved_bklight_level = pn800_get_bklight_level(panel);*/
	}


	if (md->enabled) {
		mutex_unlock(&md->mutex);
		return 0;
	}

	set_sleep_mode(md, 0);
	md->enabled = 1;
	send_init_string(md);
	set_display_state(md, 1);
	//mipid_set_bklight_level(panel, md->saved_bklight_level);
	//mipid_esd_start_check(md);

	mutex_unlock(&md->mutex);
	return 0;
}

static void pn800_panel_disable(struct omap_display *display)
{
	struct pn800_device *md =
		(struct pn800_device *)display->panel->priv;

	DBG("pn800_panel_disable\n");

	mutex_lock(&md->mutex);

	if (!md->enabled) {
		mutex_unlock(&md->mutex);
		return;
	}
	/*md->saved_bklight_level = pn800_get_bklight_level(panel);*/
	/*pn800_set_bklight_level(panel, 0);*/

	set_display_state(md, 0);
	set_sleep_mode(md, 1);
	md->enabled = 0;


	if (display->hw_config.panel_disable)
		display->hw_config.panel_disable(display);

	mutex_unlock(&md->mutex);
}

static int pn800_panel_init(struct omap_display *display)
{
	struct pn800_device *md =
		(struct pn800_device *)display->panel->priv;

	DBG("pn800_panel_init\n");

	mutex_init(&md->mutex);
	md->display = display;

	return 0;
}

static int pn800_run_test(struct omap_display *display, int test_num)
{
	return 0;
}

static struct omap_panel pn800_panel = {
	.owner		= THIS_MODULE,
	.name		= "panel-pn800",
	.init		= pn800_panel_init,
	/*.remove	= pn800_cleanup,*/
	.enable		= pn800_panel_enable,
	.disable	= pn800_panel_disable,
	//.set_mode	= pn800_set_mode,
	.run_test	= pn800_run_test,

	.timings = {
		.x_res = 800,
		.y_res = 480,

		.pixel_clock	= 21940,
		.hsw		= 50,
		.hfp		= 20,
		.hbp		= 15,

		.vsw		= 2,
		.vfp		= 1,
		.vbp		= 3,
	},
	.config		= OMAP_DSS_LCD_TFT,
};

static int pn800_spi_probe(struct spi_device *spi)
{
	struct pn800_device *md;

	DBG("pn800_spi_probe\n");

	md = kzalloc(sizeof(*md), GFP_KERNEL);
	if (md == NULL) {
		dev_err(&spi->dev, "out of memory\n");
		return -ENOMEM;
	}

	spi->mode = SPI_MODE_0;
	md->spi = spi;
	dev_set_drvdata(&spi->dev, md);
	md->panel = pn800_panel;
	pn800_panel.priv = md;

	omap_dss_register_panel(&pn800_panel);

	return 0;
}

static int pn800_spi_remove(struct spi_device *spi)
{
	struct pn800_device *md = dev_get_drvdata(&spi->dev);

	DBG("pn800_spi_remove\n");

	omap_dss_unregister_panel(&pn800_panel);

	/*pn800_disable(&md->panel);*/
	kfree(md);

	return 0;
}

static struct spi_driver pn800_spi_driver = {
	.driver = {
		.name	= "panel-n800",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= pn800_spi_probe,
	.remove	= __devexit_p(pn800_spi_remove),
};

static int __init pn800_init(void)
{
	DBG("pn800_init\n");
	return spi_register_driver(&pn800_spi_driver);
}

static void __exit pn800_exit(void)
{
	DBG("pn800_exit\n");
	spi_unregister_driver(&pn800_spi_driver);
}

module_init(pn800_init);
module_exit(pn800_exit);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@nokia.com>");
MODULE_DESCRIPTION("N800 LCD Driver");
MODULE_LICENSE("GPL");
