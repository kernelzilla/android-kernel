#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/led-lm3530.h>

#include <mach/display.h>
#include <mach/dma.h>
#include <asm/atomic.h>

#include <mach/dt_path.h>
#include <asm/prom.h>

#include <mach/panel.h>

#ifndef CONFIG_ARM_OF
#error CONFIG_ARM_OF must be defined for Mapphone to compile
#endif
#ifndef CONFIG_USER_PANEL_DRIVER
#error CONFIG_USER_PANEL_DRIVER must be defined for Mapphone to compile
#endif

#ifdef DEBUG
#define DBG(format, ...) (printk(KERN_DEBUG "mapphone-panel: " format, \
				## __VA_ARGS__))
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
#define EDISCO_CMD_SET_TEAR_OFF		0x34
#define EDISCO_CMD_SET_TEAR_ON		0x35
#define EDISCO_CMD_SET_TEAR_SCANLINE	0x44
#define EDISCO_CMD_READ_DDB_START	0xA1
#define EDISCO_CMD_SET_MCS		0xB2
#define EDISCO_CMD_DATA_LANE_CONFIG	0xB5

#define EDISCO_CMD_DATA_LANE_ONE	0x0
#define EDISCO_CMD_DATA_LANE_TWO	0x1

#define EDISCO_CMD_MCS_ON			0x3
#define EDISCO_CMD_MCS_OFF			0x0
#define EDISCO_CMD_PWM				0xb4

#define EDISCO_LONG_WRITE	0x29
#define EDISCO_SHORT_WRITE_1	0x23
#define EDISCO_SHORT_WRITE_0	0x13

#define EDISCO_CMD_READ_DDB_START	0xA1
#define EDISCO_CMD_VC   1
#define EDISCO_VIDEO_VC 0

#define PANEL_OFF     0x0
#define PANEL_ON      0x1

#define DCS_CMD_RETRY_MAX 5

/* DDB Supplier IDs used for run_test(1) */
#define SUPPLIER_ID_LEN	2
#define SUPPLIER_ID_AUO 0x0186
#define SUPPLIER_ID_TMD 0x0126
#define SUPPLIER_ID_INVALID 0xFFFF

/* this must be match with schema.xml section "device-id-value" */
#define MOT_DISP_MIPI_480_854_CM   	0x000a0001
#define MOT_DISP_430_MIPI_480_854_CM	0x001a0000
#define MOT_DISP_370_MIPI_480_854_CM	0x001a0001
#define MOT_DISP_248_MIPI_320_240_VM	0x00090002
#define MOT_DISP_280_MIPI_320_240_VM	0x00090003

static bool mapphone_panel_device_read_dt;
static u32 mapphone_panel_te_scan_line = 768;

static bool first_frame;

int mapphone_panel_id = 0;

/* these enum must be matched with MOT DT */
enum omap_dss_device_disp_pxl_fmt {
	OMAP_DSS_DISP_PXL_FMT_RGB565	= 1,
	OMAP_DSS_DISP_PXL_FMT_RGB888	= 5
};

static struct omap_video_timings mapphone_panel_timings = {
	.x_res          = 480,
	.y_res          = 854,
	/*.pixel_clock  = 25000,*/
	.dsi1_pll_fclk	= 100000,
	.dsi2_pll_fclk  = 100000,
	.hfp            = 0,
	.hsw            = 2,
	.hbp            = 2,
	.vfp            = 0,
	.vsw            = 1,
	.vbp            = 1,
};

struct mapphone_data {
	struct omap_dss_device *dssdev;
	atomic_t state;
	void *panel_handle;

	bool manual_te_trigger;
};

static void mapphone_panel_disable_local(struct omap_dss_device *dssdev);

static int dsi_mipi_vm_panel_on(struct omap_dss_device *dssdev)
{
	u8 data = EDISCO_CMD_SET_DISPLAY_ON;
	int ret;

	dsi_disable_vid_vc_enable_cmd_vc();
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, &data, 1);
	dsi_disable_cmd_vc_enable_vid_vc();

	return ret;
}

static int dsi_mipi_cm_panel_on(struct omap_dss_device *dssdev)
{
	u8 data = EDISCO_CMD_SET_DISPLAY_ON;
	return dsi_vc_dcs_write(EDISCO_CMD_VC, &data, 1);
}


static int mapphone_panel_display_on(struct omap_dss_device *dssdev)
{
	int ret = 0;

	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;

	if (atomic_cmpxchg(&map_data->state, PANEL_OFF, PANEL_ON) ==
						PANEL_OFF) {
		switch (dssdev->panel.panel_id) {
		case MOT_DISP_248_MIPI_320_240_VM:
		case MOT_DISP_280_MIPI_320_240_VM:
			ret = dsi_mipi_vm_panel_on(dssdev);
			break;
		case MOT_DISP_MIPI_480_854_CM:
		case MOT_DISP_370_MIPI_480_854_CM:
		case MOT_DISP_430_MIPI_480_854_CM:
			ret = dsi_mipi_cm_panel_on(dssdev);
			break;
		default:
			printk(KERN_ERR "unsupport panel =0x%lx \n",
			dssdev->panel.panel_id);
			ret = -EINVAL;
		}

		if (ret == 0)
			printk(KERN_INFO "Panel is turned on \n");
	}

	return ret;
}

static int mapphone_panel_dt_panel_probe(int *pixel_size)
{
	struct device_node *panel_node;
	const void *panel_prop;
	int panel_pixel_fmt;

	if (mapphone_panel_device_read_dt == true)
		printk("\nmapphone_panel_device_read_dt =true");

	DBG("dt_panel_probe\n");

	panel_node = of_find_node_by_path(DT_PATH_DISPLAY1);
	if (panel_node != NULL) {
		/* Retrieve the panel DSI timing */
		panel_prop = of_get_property(panel_node, "width", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.x_res = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node, "height", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.y_res = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node, "te_scan_line", NULL);
		if (panel_prop != NULL)
			mapphone_panel_te_scan_line = *(u32 *)panel_prop;

		panel_prop = of_get_property(panel_node, "dsi1_pll_fclk", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.dsi1_pll_fclk =
						*(u32 *)panel_prop / 1000;

		panel_prop = of_get_property(panel_node, "dsi2_pll_fclk", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.dsi2_pll_fclk =
						*(u32 *)panel_prop / 1000;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hfp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hfp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hsw", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hsw = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_hbp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.hbp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vfp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vfp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vsw", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vsw = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node,
						"dispc_timing_vbp", NULL);
		if (panel_prop != NULL)
			mapphone_panel_timings.vbp = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node, "pixel_fmt", NULL);
		if (panel_prop != NULL) {
			panel_pixel_fmt = *(u32 *)panel_prop;
			if (panel_pixel_fmt == OMAP_DSS_DISP_PXL_FMT_RGB888)
				*pixel_size = 24;
			else if (panel_pixel_fmt ==
					OMAP_DSS_DISP_PXL_FMT_RGB565)
				*pixel_size = 16;
			else {
				printk(KERN_ERR " Invalid panel_pxl_fmt=%d",
						panel_pixel_fmt);
				return -ENODEV;
			}
		}

		of_node_put(panel_node);

		DBG("DT:width=%d height=%d dsi1_pll_fclk=%d dsi2_pll_fclk=%d\n",
			mapphone_panel_timings.x_res,
			mapphone_panel_timings.y_res,
			mapphone_panel_timings.dsi1_pll_fclk,
			mapphone_panel_timings.dsi2_pll_fclk);

		DBG(" DT: hfp= %d hsw= %d hbp= %d vfp= %d vsw= %d vbp= %d\n",
			mapphone_panel_timings.hfp,
			mapphone_panel_timings.hsw,
			mapphone_panel_timings.hbp,
			mapphone_panel_timings.vfp,
			mapphone_panel_timings.vsw,
			mapphone_panel_timings.vbp);

		mapphone_panel_device_read_dt = true;
	}

	return panel_node ? 0 : -ENODEV;

}

void panel_print_dt(void)
{
	printk(KERN_DEBUG "DT: width= %d height= %d\n",
		mapphone_panel_timings.x_res, mapphone_panel_timings.y_res);

	printk(KERN_DEBUG "DT: hfp= %d hsw= %d hbp= %d vfp= %d vsw= %d vbp= %d\n",
		mapphone_panel_timings.hfp, mapphone_panel_timings.hsw,
		mapphone_panel_timings.hbp, mapphone_panel_timings.vfp,
		mapphone_panel_timings.vsw, mapphone_panel_timings.vbp);
}

static u16 mapphone_panel_read_supplier_id(struct omap_dss_device *dssdev)
{
	static u16 id = SUPPLIER_ID_INVALID;
	u8 data[2];

	if (id == SUPPLIER_ID_AUO || id == SUPPLIER_ID_TMD)
		goto end;

	if (dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC, 2))
		goto end;

	if (dsi_vc_dcs_read(EDISCO_CMD_VC,
			    EDISCO_CMD_READ_DDB_START, data, 2) != 2)
		goto end;

	if (dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC, 1))
		goto end;

	id = (data[0] << 8) | data[1];

	if (id != SUPPLIER_ID_AUO && id != SUPPLIER_ID_TMD)
		id = SUPPLIER_ID_INVALID;
end:
	DBG("dsi_read_supplier_id() - supplier id [%hu]\n", id);
	return id;
}

static ssize_t mapphone_panel_supplier_id_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	char *supplier_str = "INVALID";
	struct omap_dss_device *dssdev;

	dssdev = container_of(dev, struct omap_dss_device, dev);
	switch (mapphone_panel_read_supplier_id(dssdev)) {
	case SUPPLIER_ID_AUO:
		supplier_str = "AUO";
		break;
	case SUPPLIER_ID_TMD:
		supplier_str = "TMD";
		break;
	case SUPPLIER_ID_INVALID:
	default:
		break;
	}

	return sprintf(buf, "%s\n", supplier_str);
}

static DEVICE_ATTR(supplier_id, 0644, mapphone_panel_supplier_id_show, NULL);

#ifdef DEBUG
static ssize_t mapphone_panel_te_scan_line_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", mapphone_panel_te_scan_line);
}

static ssize_t mapphone_panel_te_scan_line_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int error = 0;
	unsigned long scan_line;

	error = strict_strtoul(buf, 10, &scan_line);

	if (error < 0)
		return -1;
	if (scan_line > 0xFFFF)
		return -1;

	mapphone_panel_te_scan_line = scan_line;
	return mapphone_panel_te_scan_line;
}

static DEVICE_ATTR(scan_line, 0644, mapphone_panel_te_scan_line_show,
					mapphone_panel_te_scan_line_store);
#endif /* DEBUG */

static int mapphone_panel_probe(struct omap_dss_device *dssdev)
{
	int error;
	int pixel_size = 24;
	struct mapphone_data *data;
	struct omap_panel_device panel_dev;

	DBG("probe\n");

	if (mapphone_panel_dt_panel_probe(&pixel_size))
		printk(KERN_INFO "panel: using non-dt configuration\n");

	panel_print_dt();
	dssdev->ctrl.pixel_size = pixel_size;
	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = mapphone_panel_timings;

	data = kmalloc(sizeof(struct mapphone_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memset(data, 0, sizeof(data));

	strncpy(panel_dev.name, dssdev->name, OMAP_PANEL_MAX_NAME_SIZE);
	panel_dev.fod_disable = mapphone_panel_disable_local;
	panel_dev.dssdev = dssdev;
	data->panel_handle = omap_panel_register(&panel_dev);
	if (data->panel_handle == NULL) {
		printk(KERN_ERR "Panel Register Failed\n");
		kfree(data);
		data = NULL;
		return -ENODEV;
	}

	error = device_create_file(&panel_dev.dssdev->dev,
		&dev_attr_supplier_id);
	if (error < 0) {
		pr_err("%s: device file create failed: %d\n", __func__, error);
		return -ENODEV;
	}

#ifdef DEBUG
	error = device_create_file(&panel_dev.dssdev->dev, &dev_attr_scan_line);
	if (error < 0) {
		pr_err("%s: device file create failed: %d\n", __func__, error);
		return -ENODEV;
	}
#endif /* DEBUG */

	atomic_set(&data->state, PANEL_OFF);
	data->dssdev = dssdev;
	dssdev->data = data;

	return 0;
}

static void mapphone_panel_remove(struct omap_dss_device *dssdev)
{
	void *handle;
	struct mapphone_data *data = (struct mapphone_data *) dssdev->data;

	handle = data->panel_handle;
	omap_panel_unregister(handle);

	kfree(dssdev->data);
	return;
}

static int dsi_mipi_248_vm_320_240_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7];
	int ret;

	DBG(" dsi_mipi_248_vm_320_240_panel_enable() \n");

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
	if (ret)
		goto error;

	mdelay(10);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");
	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_280_vm_320_240_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[10];
	int ret;

	DBG(" dsi_mipi_280_vm_320_240_panel_enable() \n");

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1, data, 2);

	/* Internal display set up */
	data[0] = 0xC0;
	data[1] = 0x11;
	data[2] = 0x04;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 3);

	/* Internal voltage set up */
	data[0] = 0xD3;
	data[1] = 0x1F;
	data[2] = 0x01;
	data[3] = 0x02;
	data[4] = 0x15;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 5);

	/* Internal voltage set up */
	data[0] = 0xD4;
	data[1] = 0x62;
	data[2] = 0x1E;
	data[3] = 0x00;
	data[4] = 0xB7;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 5);

	/* Internal display set up */
	data[0] = 0xC5;
	data[1] = 0x01;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1, data, 2);

	/* Load optimized red gamma (+) settings*/
	data[0] = 0xE9;
	data[1] = 0x01;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x0B;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

	/* Load optimized red gamma (-) settings*/
	data[0] = 0xEA;
	data[1] = 0x04;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x08;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

	/* Load optimized green gamma (+) settings*/
	data[0] = 0xEB;
	data[1] = 0x02;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x0B;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

	/* Load optimized green gamma (-) settings*/
	data[0] = 0xEC;
	data[1] = 0x05;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x08;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

	/* Load optimized blue gamma (+) settings*/
	data[0] = 0xED;
	data[1] = 0x04;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x0B;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

	/* Load optimized blue gamma (-) settings*/
	data[0] = 0xEE;
	data[1] = 0x07;
	data[2] = 0x0B;
	data[3] = 0x05;
	data[4] = 0x21;
	data[5] = 0x05;
	data[6] = 0x0D;
	data[7] = 0x01;
	data[8] = 0x08;
	data[9] = 0x04;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 10);

	/* turn on mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x03;
	ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1, data, 2);

	/* turn it on */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
	if (ret)
		goto error;

	mdelay(10);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");
	return 0;
error:
	return -EINVAL;
}

static int dsi_mipi_cm_480_854_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7], data_ret[7];
	int ret, i, retry_loop = DCS_CMD_RETRY_MAX;

	DBG("dsi_mipi_cm_480_854_panel_enable() \n");

	/* Check if the display we are using is actually a TMD display */
	if (dssdev->panel.panel_id == MOT_DISP_370_MIPI_480_854_CM) {
		if (mapphone_panel_read_supplier_id(dssdev)
						== SUPPLIER_ID_TMD) {
			DBG("dsi_mipi_cm_480_854_panel_enable() - TMD panel\n");
			dssdev->panel.panel_id = MOT_DISP_MIPI_480_854_CM;
			retry_loop = 1;
		}
	}

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;
	for (i = 0; i < retry_loop; i++) {
		ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1,
							data, 2);
		if (ret < 0)
			printk(KERN_ERR "failed to wake up  %d\n", i);

		data_ret[0] = 0xff;
		if (dsi_vc_dcs_read(EDISCO_CMD_VC, EDISCO_CMD_SET_MCS,
							&data_ret[0], 1) < 0)
			printk(KERN_ERR "failed to read MCS %d\n", i);
		else if (data_ret[0] != EDISCO_CMD_MCS_OFF)
			printk(KERN_INFO "Cant turn off MCS %d : %d\n",
					data_ret[0], i);
	}

	/* enable lane setting and test registers*/
	data[0] = 0xef;
	data[1] = 0x01;
	data[2] = 0x01;
	for (i = 0; i < retry_loop; i++) {
		ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_LONG_WRITE, data, 3);
		if (ret < 0)
			printk(KERN_ERR "failed to wake up  %d\n", i);
	}

	/* 2nd param 61 = 1 line; 63 = 2 lanes */
	data[0] = 0xef;
	data[1] = 0x60;
	data[2] = 0x63;
	for (i = 0; i < retry_loop; i++) {
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 3);
		if (ret < 0)
			printk(KERN_ERR "failed to wake up  %d\n", i);
	}

	/* 2nd param 0 = WVGA; 1 = WQVGA */
	data[0] = 0xb3;
	data[1] = 0x00;
	for (i = 0; i < retry_loop; i++) {
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
		if (ret < 0)
			printk(KERN_ERR "failed to wake up  %d\n", i);
	}

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 (auto BL control OFF);
	 * D[1]=0 (Grama correction On);
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = EDISCO_CMD_PWM;
	/* AUO displays require a different setting */
	if (dssdev->panel.panel_id == MOT_DISP_370_MIPI_480_854_CM)
		data[1] = 0x0f;
	else
		data[1] = 0x1f;
	for (i = 0; i < retry_loop; i++) {
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
		if (ret < 0)
			printk(KERN_ERR "failed to wake up  %d\n", i);
	}

	/* set page, column address */
	data[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = (dssdev->panel.timings.y_res - 1) >> 8;
	data[4] = (dssdev->panel.timings.y_res - 1) & 0xff;
	for (i = 0; i < retry_loop; i++) {
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
		if (ret < 0)
			printk(KERN_ERR "failed to wake up  %d\n", i);
	}

	if (ret)
		goto error;

	data[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = (dssdev->panel.timings.x_res - 1) >> 8;
	data[4] = (dssdev->panel.timings.x_res - 1) & 0xff;
	for (i = 0; i < retry_loop; i++) {
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
		if (ret < 0)
			printk(KERN_ERR "failed to wake up  %d\n", i);
	}

	if (ret)
		goto error;

	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	for (i = 0; i < retry_loop; i++) {
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
		if (ret < 0)
			printk(KERN_ERR "failed to wake up  %d\n", i);
	}

	mdelay(200);

	printk(KERN_INFO "done EDISCO CTRL ENABLE\n");

	return 0;
error:
	return -EINVAL;


}

static int dsi_mipi_430_cm_480_854_panel_enable(struct omap_dss_device *dssdev)
{
	u8 data[7], data_ret[7], i;
	int ret;
	DBG("dsi_mipi_430_cm_480_854_panel_enable() \n");


	/* Next frame will be first and has to allow PWM at the LM3530 */
	first_frame = true;

	/* Loop & repeat all commands. Cant trust AUO panel */


	/* Exit sleep mode */
	data[0] = EDISCO_CMD_EXIT_SLEEP_MODE;
	for (i = 0; i < DCS_CMD_RETRY_MAX; i++) {
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 1);
		if (ret < 0)
			printk(KERN_ERR "failed to wake up  %d\n", i);
	}

	/* 120ms delay for internal block stabilization */
	msleep(120);

	/* turn off mcs register acces protection */
	data[0] = EDISCO_CMD_SET_MCS;
	data[1] = 0x00;

	for (i = 0; i < DCS_CMD_RETRY_MAX; i++) {
		ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1,
							data, 2);
		if (ret)
			printk(KERN_ERR "failed to send SET_MCS %d\n", i);
		data_ret[0] = 0xff;
		if (dsi_vc_dcs_read(EDISCO_CMD_VC, EDISCO_CMD_SET_MCS,
							&data_ret[0], 1) < 0)
			printk(KERN_ERR "failed to read MCS %d\n", i);
		else if (data_ret[0] != EDISCO_CMD_MCS_OFF)
			printk(KERN_INFO "Cant turn off MCS %d : %d\n",
					data_ret[0], i);
	}

	/* Enable 2 data lanes */
	data[0] = EDISCO_CMD_DATA_LANE_CONFIG;
	data[1] = EDISCO_CMD_DATA_LANE_TWO;
	for (i = 0; i < DCS_CMD_RETRY_MAX; i++) {
		ret = dsi_vc_write(EDISCO_CMD_VC, EDISCO_SHORT_WRITE_1,
							data, 2);
		if (ret)
			printk(KERN_ERR "failed send DATA_LANE_CONFIG %d\n", i);

		data_ret[0] = 0xff;
		if (dsi_vc_dcs_read(EDISCO_CMD_VC, EDISCO_CMD_DATA_LANE_CONFIG,
							&data_ret[0], 1) < 0)
			printk(KERN_ERR "failed read DATA_LANE_CONFIG %d\n", i);
		else if (data_ret[0] != EDISCO_CMD_DATA_LANE_TWO)
			printk(KERN_INFO "cannot config LANE_CONFIG(%d) %d\n",
					data_ret[0], i);
	}

	msleep(10);

	/* Set dynamic backlight control and PWM; D[7:4] = PWM_DIV[3:0];*/
	/* D[3]=0 (PWM OFF);
	 * D[2]=0 (auto BL control OFF);
	 * D[1]=0 (Grama correction On);
	 * D[0]=0 (Enhanced Image Correction OFF) */
	data[0] = EDISCO_CMD_PWM;
	data[1] = 0xdf;
	for (i = 0; i < DCS_CMD_RETRY_MAX; i++) {
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
		if (ret)
			printk(KERN_ERR "failed to send PWM control %d\n", i);
	}

	/* set page, column address */
	data[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = (dssdev->panel.timings.x_res - 1) >> 8;
	data[4] = (dssdev->panel.timings.x_res - 1) & 0xff;
	for (i = 0; i < DCS_CMD_RETRY_MAX; i++) {
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
		if (ret)
			printk(KERN_ERR "failed send SET_COLUMN_ADDR %d\n", i);
	}

	if (ret)
		goto error;

	data[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = (dssdev->panel.timings.y_res - 1) >> 8;
	data[4] = (dssdev->panel.timings.y_res - 1) & 0xff;
	for (i = 0; i < DCS_CMD_RETRY_MAX; i++) {
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 5);
		if (ret)
			printk(KERN_ERR "failed send SET_PAGE_ADDRESS %d\n", i);
	}
	if (ret)
		goto error;

	mdelay(200);

	return 0;
error:
	return -EINVAL;
}

static void mapphone_panel_set_man_te_trigger(struct omap_dss_device *dssdev);
static int mapphone_panel_enable(struct omap_dss_device *dssdev)
{
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;
	int ret;
	void *handle;

	DBG("mapphone_panel_enable\n");
	if (dssdev->platform_enable) {
		ret = dssdev->platform_enable(dssdev);
		if (ret)
			return ret;
	}

	handle = map_data->panel_handle;
	if (omap_panel_fod_enabled(handle)) {
		atomic_set(&map_data->state, PANEL_OFF);
	}
	omap_panel_fod_dss_state(handle, 1);
	omap_panel_fod_panel_state(handle, 1);

	switch (dssdev->panel.panel_id) {
	case MOT_DISP_MIPI_480_854_CM:
	case MOT_DISP_370_MIPI_480_854_CM:
		ret = dsi_mipi_cm_480_854_panel_enable(dssdev);
		break;
	case MOT_DISP_430_MIPI_480_854_CM:
		ret = dsi_mipi_430_cm_480_854_panel_enable(dssdev);
		break;
	case MOT_DISP_248_MIPI_320_240_VM:
		ret = dsi_mipi_248_vm_320_240_panel_enable(dssdev) ;
		break;
	case MOT_DISP_280_MIPI_320_240_VM:
		ret = dsi_mipi_280_vm_320_240_panel_enable(dssdev) ;
		break;
	default:
		printk(KERN_ERR "unsupport panel =0x%lx \n",
			dssdev->panel.panel_id);
		goto error;
	}

	mapphone_panel_set_man_te_trigger(dssdev);

	mapphone_panel_id = dssdev->panel.panel_id;

	if (ret)
		goto error;

	return 0;
error:
	return -EINVAL;
}

static void mapphone_panel_disable_local(struct omap_dss_device *dssdev)
{
	u8 data[1];
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;

	atomic_set(&map_data->state, PANEL_OFF);

	data[0] = EDISCO_CMD_SET_DISPLAY_OFF;
	dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 1);

	data[0] = EDISCO_CMD_ENTER_SLEEP_MODE;
	dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 1);
	msleep(120);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

}

static void mapphone_panel_disable(struct omap_dss_device *dssdev)
{
	void *handle;

	DBG("mapphone_panel_disable\n");

	handle = ((struct mapphone_data *)dssdev->data)->panel_handle;
	omap_panel_fod_dss_state(handle, 0);
	if (omap_panel_fod_enabled(handle)) {
		DBG("Freezing the last frame on the display\n");
		return;
	}

	omap_panel_fod_panel_state(handle, 0);

	mapphone_panel_disable_local(dssdev);
}

static void mapphone_panel_setup_update(struct omap_dss_device *dssdev,
				      u16 x, u16 y, u16 w, u16 h)
{
	u8 data[5];
	int ret;

	/* This is needed to notify the lm3530 of an incoming pwm */
	if (dssdev->panel.panel_id == MOT_DISP_430_MIPI_480_854_CM) {
		if (first_frame) {
			first_frame = false;
			ld_lm3530_pwm_start();
		}
	}

	/* set page, column address */
	data[0] = EDISCO_CMD_SET_PAGE_ADDRESS;
	data[1] = y >> 8;
	data[2] = y & 0xff;
	data[3] = (y + h - 1) >> 8;
	data[4] = (y + h - 1) & 0xff;
	ret = dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 5);
	if (ret)
		return;

	data[0] = EDISCO_CMD_SET_COLUMN_ADDRESS;
	data[1] = x >> 8;
	data[2] = x & 0xff;
	data[3] = (x + w - 1) >> 8;
	data[4] = (x + w - 1) & 0xff;
	ret = dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 5);
	if (ret)
		return;
}

static int mapphone_panel_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	u8 data[3];
	int ret;

	if (enable == true) {
		data[0] = EDISCO_CMD_SET_TEAR_ON;
		data[1] = 0x00;
		ret = dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 2);
		if (ret)
			goto error;

		data[0] = EDISCO_CMD_SET_TEAR_SCANLINE;
		data[1] = (mapphone_panel_te_scan_line >> 8) & 0xff;
		data[2] = 0xff & mapphone_panel_te_scan_line;
		ret = dsi_vc_dcs_write_nosync(EDISCO_CMD_VC, data, 3);
		if (ret)
			goto error;

	} else {
		data[0] = EDISCO_CMD_SET_TEAR_OFF;
		data[1] = 0x00;
		ret = dsi_vc_dcs_write(EDISCO_CMD_VC, data, 2);
		if (ret)
			goto error;
	}

	DBG(" edisco_ctrl_enable_te(%d) \n", enable);
	return 0;

error:
	return -EINVAL;
}

static int mapphone_panel_get_hs_mode_timing(struct omap_dss_device *dssdev)
{
	/* The following time values are required for MIPI timing
	per OMAP spec */
	dssdev->phy.dsi.hs_timing.ths_prepare = 70;
	dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero = 175;
	dssdev->phy.dsi.hs_timing.ths_trail = 60;
	dssdev->phy.dsi.hs_timing.ths_exit = 145;
	dssdev->phy.dsi.hs_timing.tlpx_half = 25;
	dssdev->phy.dsi.hs_timing.tclk_trail = 60;
	dssdev->phy.dsi.hs_timing.tclk_prepare = 65;
	dssdev->phy.dsi.hs_timing.tclk_zero = 260;

	/* These values are required for the following spec panels */
	if ((dssdev->panel.panel_id == MOT_DISP_430_MIPI_480_854_CM) ||
		(dssdev->panel.panel_id == MOT_DISP_370_MIPI_480_854_CM)) {
		dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero = 454;
	}

	return 0;
}

static void mapphone_panel_set_man_te_trigger(struct omap_dss_device *dssdev)
{
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;

	if ((dssdev->panel.panel_id == MOT_DISP_370_MIPI_480_854_CM) ||
		(dssdev->panel.panel_id == MOT_DISP_430_MIPI_480_854_CM))
			map_data->manual_te_trigger = true;
	else
		map_data->manual_te_trigger = false;
}

static bool mapphone_panel_manual_te_trigger(struct omap_dss_device *dssdev)
{
	struct mapphone_data *map_data = (struct mapphone_data *) dssdev->data;
	return map_data->manual_te_trigger;
}

static int mapphone_panel_rotate(struct omap_dss_device *display, u8 rotate)
{
	return 0;
}

static int mapphone_panel_mirror(struct omap_dss_device *display, bool enable)
{
	return 0;
}

static int mapphone_panel_run_test(struct omap_dss_device *display,
					int test_num)
{
	int r = -1; /* Returns -1 if no dssdev or test isn't supported */
		/* Returns 0 on success, or the test_num on failure */
	u8 data[SUPPLIER_ID_LEN];
	u16 id = 0xFFFF;

	if (!display)
		return r;

	if (test_num == 1)	{
		if ((display->panel.panel_id == MOT_DISP_430_MIPI_480_854_CM) ||
		    (display->panel.panel_id == MOT_DISP_370_MIPI_480_854_CM) ||
		    (display->panel.panel_id == MOT_DISP_MIPI_480_854_CM)) {
			r = test_num;
			/* Status check to ensure communication */
			dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC,
					SUPPLIER_ID_LEN);
			if (dsi_vc_dcs_read(EDISCO_CMD_VC,
					EDISCO_CMD_READ_DDB_START,
					data, SUPPLIER_ID_LEN)
					> 0) {
				id = (data[0] << 8) | data[1];

				switch (display->panel.panel_id) {
				case MOT_DISP_430_MIPI_480_854_CM:
					/* Shadow AUO panels report either */
					if ((id == SUPPLIER_ID_AUO) ||
					    (id == SUPPLIER_ID_TMD))
						r = 0;
					break;
				case MOT_DISP_370_MIPI_480_854_CM:
					if (id == SUPPLIER_ID_AUO)
						r = 0;
					break;
				case MOT_DISP_MIPI_480_854_CM:
					if (id == SUPPLIER_ID_TMD)
						r = 0;
					break;
				default:
					r = 0;
					break;
				}
			}
			dsi_vc_set_max_rx_packet_size(EDISCO_CMD_VC, 1);
		} else {
			/* If check not supported, return success */
			r = 0;
		}
	}
	return r;
}

static int mapphone_panel_suspend(struct omap_dss_device *dssdev)
{
	mapphone_panel_disable(dssdev);
	return 0;
}

static int mapphone_panel_resume(struct omap_dss_device *dssdev)
{
	return mapphone_panel_enable(dssdev);
}

static struct omap_dss_driver mapphone_panel_driver = {
	.probe = mapphone_panel_probe,
	.remove = mapphone_panel_remove,

	.enable = mapphone_panel_enable,
	.framedone = mapphone_panel_display_on,
	.disable = mapphone_panel_disable,
	.suspend = mapphone_panel_suspend,
	.resume = mapphone_panel_resume,
	.setup_update = mapphone_panel_setup_update,
	.hs_mode_timing = mapphone_panel_get_hs_mode_timing,
	.enable_te = mapphone_panel_enable_te,
	.manual_te_trigger = mapphone_panel_manual_te_trigger,
	.set_rotate = mapphone_panel_rotate,
	.set_mirror = mapphone_panel_mirror,
	.run_test = mapphone_panel_run_test,

	.driver = {
		.name = "mapphone-panel",
		.owner = THIS_MODULE,
	},
};


static int __init mapphone_panel_init(void)
{
	DBG("mapphone_panel_init\n");

	first_frame = false;

	omap_dss_register_driver(&mapphone_panel_driver);
	mapphone_panel_device_read_dt = false;
	return 0;
}

static void __exit mapphone_panel_exit(void)
{
	DBG("mapphone_panel_exit\n");

	omap_dss_unregister_driver(&mapphone_panel_driver);
}

module_init(mapphone_panel_init);
module_exit(mapphone_panel_exit);

MODULE_AUTHOR("Rebecca Schultz Zavin <rebecca@android.com>");
MODULE_DESCRIPTION("Sholes Panel Driver");
MODULE_LICENSE("GPL");
