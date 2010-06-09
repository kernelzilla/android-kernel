/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#define DEBUG

#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include "msm_fb.h"

#define QFPROM_BASE		((uint32)hdmi_msm_state->qfprom_io)
#define HDMI_BASE		((uint32)hdmi_msm_state->hdmi_io)

#ifdef DEBUG
#define DEV_DBG(args...)	dev_dbg(&hdmi_msm_state->pdev->dev, args)
#else
#define DEV_DBG(args...)	(void)0
#endif
#define DEV_INFO(args...)	dev_info(&hdmi_msm_state->pdev->dev, args)
#define DEV_WARN(args...)	dev_warn(&hdmi_msm_state->pdev->dev, args)
#define DEV_ERR(args...)	dev_err(&hdmi_msm_state->pdev->dev, args)

/* all video formats defined by EIA CEA 861D */
#define MSM_HDMI_VFRMT_640x480p60_4_3		0
#define MSM_HDMI_VFRMT_720x480p60_4_3		1
#define MSM_HDMI_VFRMT_720x480p60_16_9 		2
#define MSM_HDMI_VFRMT_1280x720p60_16_9		3
#define MSM_HDMI_VFRMT_1920x1080i60_16_9	4
#define MSM_HDMI_VFRMT_720x480i60_4_3		5
#define MSM_HDMI_VFRMT_1440x480i60_4_3		MSM_HDMI_VFRMT_720x480i60_4_3
#define MSM_HDMI_VFRMT_720x480i60_16_9		6
#define MSM_HDMI_VFRMT_1440x480i60_16_9		MSM_HDMI_VFRMT_720x480i60_16_9
#define MSM_HDMI_VFRMT_720x240p60_4_3		7
#define MSM_HDMI_VFRMT_1440x240p60_4_3		MSM_HDMI_VFRMT_720x240p60_4_3
#define MSM_HDMI_VFRMT_720x240p60_16_9		8
#define MSM_HDMI_VFRMT_1440x240p60_16_9		MSM_HDMI_VFRMT_720x240p60_16_9
#define MSM_HDMI_VFRMT_2880x480i60_4_3		9
#define MSM_HDMI_VFRMT_2880x480i60_16_9		10
#define MSM_HDMI_VFRMT_2880x240p60_4_3		11
#define MSM_HDMI_VFRMT_2880x240p60_16_9		12
#define MSM_HDMI_VFRMT_1440x480p60_4_3		13
#define MSM_HDMI_VFRMT_1440x480p60_16_9		14
#define MSM_HDMI_VFRMT_1920x1080p60_16_9	15
#define MSM_HDMI_VFRMT_720x576p50_4_3		16
#define MSM_HDMI_VFRMT_720x576p50_16_9		17
#define MSM_HDMI_VFRMT_1280x720p50_16_9		18
#define MSM_HDMI_VFRMT_1920x1080ip50_16_9	19
#define MSM_HDMI_VFRMT_720x576i50_4_3		20
#define MSM_HDMI_VFRMT_1440x576i50_4_3 		MSM_HDMI_VFRMT_720x576i50_4_3
#define MSM_HDMI_VFRMT_720x576i50_16_9		21
#define MSM_HDMI_VFRMT_1440x576i50_16_9 	MSM_HDMI_VFRMT_720x576i50_16_9
#define MSM_HDMI_VFRMT_720x288p50_4_3		22
#define MSM_HDMI_VFRMT_1440x288p50_4_3		MSM_HDMI_VFRMT_720x288p50_4_3
#define MSM_HDMI_VFRMT_720x288p50_16_9		23
#define MSM_HDMI_VFRMT_1440x288p50_16_9		MSM_HDMI_VFRMT_720x288p50_16_9
#define MSM_HDMI_VFRMT_2880x576i50_4_3		24
#define MSM_HDMI_VFRMT_2880x576i50_16_9		25
#define MSM_HDMI_VFRMT_2880x288p50_4_3		26
#define MSM_HDMI_VFRMT_2880x288p50_16_9		27
#define MSM_HDMI_VFRMT_1440x576p50_4_3		28
#define MSM_HDMI_VFRMT_1440x576p50_16_9		29
#define MSM_HDMI_VFRMT_1920x1080p50_16_9	30
#define MSM_HDMI_VFRMT_1920x1080p24_16_9	31
#define MSM_HDMI_VFRMT_1920x1080p25_16_9	32
#define MSM_HDMI_VFRMT_1920x1080p30_16_9	33
#define MSM_HDMI_VFRMT_2880x480p60_4_3		34
#define MSM_HDMI_VFRMT_2880x480p60_16_9		35
#define MSM_HDMI_VFRMT_2880x576p50_4_3		36
#define MSM_HDMI_VFRMT_2880x576p50_16_9		37
#define MSM_HDMI_VFRMT_1920x1080i50_16_9	38
#define MSM_HDMI_VFRMT_1920x1080i100_16_9	39
#define MSM_HDMI_VFRMT_1280x720p100_16_9	40
#define MSM_HDMI_VFRMT_720x576p100_4_3		41
#define MSM_HDMI_VFRMT_720x576p100_16_9		42
#define MSM_HDMI_VFRMT_720x576i100_4_3		43
#define MSM_HDMI_VFRMT_1440x576i100_4_3		MSM_HDMI_VFRMT_720x576i100_4_3
#define MSM_HDMI_VFRMT_720x576i100_16_9		44
#define MSM_HDMI_VFRMT_1440x576i100_16_9	MSM_HDMI_VFRMT_720x576i100_16_9
#define MSM_HDMI_VFRMT_1920x1080i120_16_9	45
#define MSM_HDMI_VFRMT_1280x720p120_16_9	46
#define MSM_HDMI_VFRMT_720x480p120_4_3		47
#define MSM_HDMI_VFRMT_720x480p120_16_9		48
#define MSM_HDMI_VFRMT_720x480i120_4_3		49
#define MSM_HDMI_VFRMT_1440x480i120_4_3  	MSM_HDMI_VFRMT_720x480i120_4_3
#define MSM_HDMI_VFRMT_720x480i120_16_9		50
#define MSM_HDMI_VFRMT_1440x480i120_16_9	MSM_HDMI_VFRMT_720x480i120_16_9
#define MSM_HDMI_VFRMT_720x576p200_4_3		51
#define MSM_HDMI_VFRMT_720x576p200_16_9		52
#define MSM_HDMI_VFRMT_720x576i200_4_3		53
#define MSM_HDMI_VFRMT_1440x576i200_4_3		MSM_HDMI_VFRMT_720x576i200_4_3
#define MSM_HDMI_VFRMT_720x576i200_16_9		54
#define MSM_HDMI_VFRMT_1440x576i200_16_9	MSM_HDMI_VFRMT_720x576i200_16_9
#define MSM_HDMI_VFRMT_720x480p240_4_3		55
#define MSM_HDMI_VFRMT_720x480p240_16_9		56
#define MSM_HDMI_VFRMT_720x480i240_4_3		57
#define MSM_HDMI_VFRMT_1440x480i240_4_3		MSM_HDMI_VFRMT_720x480i240_4_3
#define MSM_HDMI_VFRMT_720x480i240_16_9		58
#define MSM_HDMI_VFRMT_1440x480i240_16_9	MSM_HDMI_VFRMT_720x480i240_16_9
#define MSM_HDMI_VFRMT_MAX			59
#define MSM_HDMI_VFRMT_FORCE_32BIT		0x7FFFFFFF

/* Supported HDMI Audio channels */
#define MSM_HDMI_AUDIO_CHANNEL_2		0
#define MSM_HDMI_AUDIO_CHANNEL_4		1
#define MSM_HDMI_AUDIO_CHANNEL_6		2
#define MSM_HDMI_AUDIO_CHANNEL_8		3
#define MSM_HDMI_AUDIO_CHANNEL_MAX		4
#define MSM_HDMI_AUDIO_CHANNEL_FORCE_32BIT	0x7FFFFFFF

/* Supported HDMI Audio sample rates */
#define MSM_HDMI_SAMPLE_RATE_32KHZ		0
#define MSM_HDMI_SAMPLE_RATE_44_1KHZ		1
#define MSM_HDMI_SAMPLE_RATE_48KHZ		2
#define MSM_HDMI_SAMPLE_RATE_88_2KHZ		3
#define MSM_HDMI_SAMPLE_RATE_96KHZ		4
#define MSM_HDMI_SAMPLE_RATE_176_4KHZ		5
#define MSM_HDMI_SAMPLE_RATE_192KHZ		6
#define MSM_HDMI_SAMPLE_RATE_MAX		7
#define MSM_HDMI_SAMPLE_RATE_FORCE_32BIT	0x7FFFFFFF

struct hdmi_msm_state_type {
	boolean disp_powered_up;

	int irq;
	uint32 video_resolution;
	struct platform_device *pdev;
	struct msm_hdmi_platform_data *pd;
	struct clk *hdmi_app_clk;
	void __iomem *qfprom_io;
	void __iomem *hdmi_io;
};

static struct hdmi_msm_state_type *hdmi_msm_state;

struct hdmi_msm_disp_mode_timing_type {
	uint32	video_format;
	uint32	active_h;
	uint32	front_porch_h;
	uint32	pulse_width_h;
	uint32	back_porch_h;
	boolean	active_low_h;
	uint32	active_v;
	uint32	front_porch_v;
	uint32	pulse_width_v;
	uint32	back_porch_v;
	boolean	active_low_v;
	/* Must divide by 1000 to get the actual frequency in MHZ */
	uint32	pixel_freq;
	/* Must divide by 1000 to get the actual frequency in HZ */
	uint32	refresh_rate;
	boolean	interlaced;
	boolean	supported;
};

#define VFRMT_NOT_SUPPORTED(VFRMT) \
	{VFRMT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, FALSE}

/* Table indicating the video format supported by the HDMI TX Core v1.0 */
/* Valid Pixel-Clock rates: 25.2MHz, 27MHz, 27.03MHz, 74.25MHz, 148.5MHz */
static struct hdmi_msm_disp_mode_timing_type
	hdmi_msm_supported_video_mode_lut[] = {

	{MSM_HDMI_VFRMT_640x480p60_4_3,      640,  16,  96,  48,  TRUE,
	 480, 10, 2, 33, TRUE, 25200, 60000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_720x480p60_4_3,      720,  16,  16,  16,  TRUE,
	 480,  9,   6,  30,  TRUE, 27030, 60000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_720x480p60_16_9,     720,  16,  16,  16,  TRUE,
	 480, 9, 6, 30,  TRUE, 27030, 60000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_1280x720p60_16_9,    1280, 110, 40,  220, FALSE,
	 720, 5, 5, 20, FALSE, 74250, 60000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_1920x1080i60_16_9,   1920, 88,  44,  148, FALSE,
	 540, 2, 5, 5, FALSE, 74250, 60000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_1440x480i60_4_3,     1440, 38,  124, 114, TRUE,
	 240, 4, 3, 15, TRUE, 27000, 60000, TRUE, TRUE},
	{MSM_HDMI_VFRMT_1440x480i60_16_9,    1440, 38,  124, 114, TRUE,
	 240, 4, 3, 15, TRUE, 27000, 60000, TRUE, TRUE},
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x240p60_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x240p60_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x480i60_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x480i60_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x240p60_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x240p60_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x480p60_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x480p60_16_9),
	{MSM_HDMI_VFRMT_1920x1080p60_16_9,   1920, 88,  44,  148,  FALSE,
	 1080, 4, 5, 36, FALSE, 148500, 60000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_720x576p50_4_3,      720,  12,  64,  68,   TRUE,
	 576,  5, 5, 39, TRUE, 27000, 50000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_720x576p50_16_9,     720,  12,  64,  68,   TRUE,
	 576,  5, 5, 39, TRUE, 27000, 50000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_1280x720p50_16_9,    1280, 440, 40,  220,  FALSE,
	 720,  5, 5, 20, FALSE, 74250, 50000, FALSE, TRUE},
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1920x1080i50_16_9),
	{MSM_HDMI_VFRMT_1440x576i50_4_3,     1440, 24,  126, 138,  TRUE,
	 288,  2, 3, 19, TRUE, 27000, 50000, TRUE, TRUE},
	{MSM_HDMI_VFRMT_1440x576i50_16_9,    1440, 24,  126, 138,  TRUE,
	 288,  2, 3, 19, TRUE, 27000, 50000, TRUE, TRUE},
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x288p50_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x288p50_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x576i50_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x576i50_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x288p50_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x288p50_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x576p50_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x576p50_16_9),
	{MSM_HDMI_VFRMT_1920x1080p50_16_9,   1920,  528,  44,  148,  FALSE,
	 1080, 4, 5, 36, FALSE, 148500, 50000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_1920x1080p24_16_9,   1920,  638,  44,  148,  FALSE,
	 1080, 4, 5, 36, FALSE, 74250, 24000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_1920x1080p25_16_9,   1920,  528,  44,  148,  FALSE,
	 1080, 4, 5, 36, FALSE, 74250, 25000, FALSE, TRUE},
	{MSM_HDMI_VFRMT_1920x1080p30_16_9,   1920,  88,   44,  148,  FALSE,
	 1080, 4, 5, 36, FALSE, 74250, 30000, FALSE, TRUE},
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x480p60_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x480p60_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x576p50_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_2880x576p50_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1920x1080i50_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1920x1080i100_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1280x720p100_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_720x576p100_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_720x576p100_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x576i100_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x576i100_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1920x1080i120_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1280x720p120_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_720x480p120_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_720x480p120_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x480i120_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x480i120_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_720x576p200_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_720x576p200_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x576i200_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x576i200_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_720x480p240_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_720x480p240_16_9),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x480i240_4_3),
	VFRMT_NOT_SUPPORTED(MSM_HDMI_VFRMT_1440x480i240_16_9),
};

#ifdef DEBUG
static const char *hdmi_msm_name(uint32 offset)
{
	switch (offset) {
	case 0x0000: return "CTRL";
	case 0x0020: return "AUDIO_PKT_CTRL1";
	case 0x0024: return "ACR_PKT_CTRL";
	case 0x0028: return "VBI_PKT_CTRL";
	case 0x002C: return "INFOFRAME_CTRL0";
	case 0x003C: return "ACP";
	case 0x0040: return "GC";
	case 0x0044: return "AUDIO_PKT_CTRL2";
	case 0x0048: return "ISRC1_0";
	case 0x004C: return "ISRC1_1";
	case 0x0050: return "ISRC1_2";
	case 0x0054: return "ISRC1_3";
	case 0x0058: return "ISRC1_4";
	case 0x005C: return "ISRC2_0";
	case 0x0060: return "ISRC2_1";
	case 0x0064: return "ISRC2_2";
	case 0x0068: return "ISRC2_3";
	case 0x006C: return "AVI_INFO0";
	case 0x0070: return "AVI_INFO1";
	case 0x0074: return "AVI_INFO2";
	case 0x0078: return "AVI_INFO3";
	case 0x00C4: return "ACR_32_0";
	case 0x00C8: return "ACR_32_1";
	case 0x00CC: return "ACR_44_0";
	case 0x00D0: return "ACR_44_1";
	case 0x00D4: return "ACR_48_0";
	case 0x00D8: return "ACR_48_1";
	case 0x00E4: return "AUDIO_INFO0";
	case 0x00E8: return "AUDIO_INFO1";
	case 0x0110: return "HDCP_CTRL";
	case 0x0114: return "HDCP_DEBUG_CTRL";
	case 0x0118: return "HDCP_DEBUG_CTRL";
	case 0x011C: return "HDCP_LINK0_STATUS";
	case 0x012C: return "HDCP_ENTROPY_CTRL0";
	case 0x0130: return "HDCP_RESET";
	case 0x0134: return "HDCP_RCVPORT_DATA0";
	case 0x0138: return "HDCP_RCVPORT_DATA1";
	case 0x013C: return "HDCP_RCVPORT_DATA2";
	case 0x0144: return "HDCP_RCVPORT_DATA3";
	case 0x0148: return "HDCP_RCVPORT_DATA4";
	case 0x014C: return "HDCP_RCVPORT_DATA5";
	case 0x0150: return "HDCP_RCVPORT_DATA6";
	case 0x0168: return "HDCP_RCVPORT_DATA12";
	case 0x01D0: return "AUDIO_CFG";
	case 0x0208: return "USEC_REFTIMER";
	case 0x020C: return "DDC_CTRL";
	case 0x0214: return "DDC_INT_CTRL";
	case 0x0218: return "DDC_STATUS";
	case 0x0220: return "DDC_SPEED";
	case 0x0224: return "DDC_SETUP";
	case 0x0228: return "DDC_TRANS0";
	case 0x022C: return "DDC_TRANS1";
	case 0x0238: return "DDC_DATA";
	case 0x0250: return "HPD_INT_STATUS";
	case 0x0254: return "HPD_INT_CTRL";
	case 0x0258: return "HPD_CTRL";
	case 0x025C: return "HDCP_ENTROPY_CTRL1";
	case 0x027C: return "DDC_REF";
	case 0x0284: return "HDCP_SW_UPPER_AKSV";
	case 0x0288: return "HDCP_SW_LOWER_AKSV";
	case 0x02B4: return "ACTIVE_H";
	case 0x02B8: return "ACTIVE_V";
	case 0x02BC: return "ACTIVE_V_F2";
	case 0x02C0: return "TOTAL";
	case 0x02C4: return "V_TOTAL_F2";
	case 0x02C8: return "FRAME_CTRL";
	case 0x02CC: return "AUD_INT";
	case 0x0300: return "PHY_REG0";
	case 0x0304: return "PHY_REG1";
	case 0x0308: return "PHY_REG2";
	case 0x030C: return "PHY_REG3";
	case 0x0310: return "PHY_REG4";
	case 0x0314: return "PHY_REG5";
	case 0x0318: return "PHY_REG6";
	case 0x031C: return "PHY_REG7";
	case 0x0320: return "PHY_REG8";
	case 0x0324: return "PHY_REG9";
	case 0x0328: return "PHY_REG10";
	case 0x032C: return "PHY_REG11";
	case 0x0330: return "PHY_REG12";
	default: return "???";
	}
}

static void __hdmi_outp(uint32 offset, uint32 value)
{
	uint32 in_val;

	outpdw(HDMI_BASE+offset, value);
	in_val = inpdw(HDMI_BASE+offset);
	DEV_DBG("HDMI[%04x] => %08x [%08x] %s\n",
		offset, value, in_val, hdmi_msm_name(offset));
}

static uint32 __hdmi_inp(uint32 offset)
{
	uint32 value = inpdw(HDMI_BASE+offset);
	DEV_DBG("HDMI[%04x] <= %08x %s\n",
		offset, value, hdmi_msm_name(offset));
	return value;
}

#define HDMI_OUTP_ND(offset, value)	outpdw(HDMI_BASE+(offset), (value))
#define HDMI_OUTP(offset, value)	__hdmi_outp((offset), (value))
#define HDMI_INP_ND(offset)		inpdw(HDMI_BASE+(offset))
#define HDMI_INP(offset)		__hdmi_inp((offset))
#else /* DEBUG */
#define HDMI_OUTP_ND(offset, value)	outpdw(HDMI_BASE+(offset), (value))
#define HDMI_OUTP(offset, value)	outpdw(HDMI_BASE+(offset), (value))
#define HDMI_INP_ND(offset)		inpdw(HDMI_BASE+(offset))
#define HDMI_INP(offset)		inpdw(HDMI_BASE+(offset))
#endif /* DEBUG */

static irqreturn_t hdmi_msm_isr(int irq, void *dev_id)
{
	/* HDMI_AUD_INT[0x02CC]
	 *   [3] AUD_SAM_DROP_MASK [R/W]
	 *   [2] AUD_SAM_DROP_ACK [W], AUD_SAM_DROP_INT [R]
	 *   [1] AUD_FIFO_URUN_MASK [R/W]
	 *   [0] AUD_FIFO_URUN_ACK [W], AUD_FIFO_URUN_INT [R] */
	uint32 audio_int_val;
	static uint32 fifo_underrun_int_occurred;
	static uint32 sample_drop_int_occurred;
	const uint32 occurrence_limit = 10;

	if (!hdmi_msm_state || !hdmi_msm_state->disp_powered_up) {
		DEV_DBG("ISR ignored, display not yet powered on\n");
		return IRQ_HANDLED;
	}

	audio_int_val = HDMI_INP_ND(0x02CC);
	/* FIFO Underrun Int is enabled */
	if (audio_int_val & (1 << 1)) {
		/* FIFO Underrun occured, clr it */
		if (audio_int_val & (1 << 0)) {
			HDMI_OUTP(0x02CC, audio_int_val | (1 << 0));
			DEV_DBG("%s: AUD_FIFO_URUN", __func__);

			++fifo_underrun_int_occurred;
			if (fifo_underrun_int_occurred >= occurrence_limit) {
				HDMI_OUTP(0x02CC, HDMI_INP(0x02CC) & ~(1 << 1));
				DEV_INFO("AUD_FIFO_URUN int has been disabled "
					"by the ISR after %d occurences...\n",
					fifo_underrun_int_occurred);
			}
		}
	}

	/* Audio Sample Drop int is enabled */
	if (audio_int_val & (1 << 3)) {
		/* Audio Sample Drop occured, clr it */
		if (audio_int_val & (1 << 2)) {
			HDMI_OUTP(0x02CC, audio_int_val | (1 << 2));
			DEV_DBG("%s: AUD_SAM_DROP", __func__);

			++sample_drop_int_occurred;
			if (sample_drop_int_occurred >= occurrence_limit) {
				HDMI_OUTP(0x02CC, HDMI_INP(0x02CC) & ~(1 << 3));
				DEV_INFO("AUD_SAM_DROP int has been disabled "
					"by the ISR after %d occurences... \n",
					sample_drop_int_occurred);
			}
		}
	}

	return IRQ_HANDLED;
}

static int check_hdmi_features(void)
{
	/* RAW_FEAT_CONFIG_ROW0_LSB */
	uint32 val = inpdw(QFPROM_BASE + 0x0238);
	/* HDMI_DISABLE */
	boolean hdmi_disabled = (val & 0x00200000) >> 21;
	/* HDCP_DISABLE */
	boolean hdcp_disabled = (val & 0x00400000) >> 22;

	DEV_DBG("Features <val:0x%08x, HDMI:%s, HDCP:%s>\n", val,
		hdmi_disabled ? "OFF" : "ON", hdcp_disabled ? "OFF" : "ON");
	if (hdmi_disabled) {
		DEV_ERR("ERROR: HDMI disabled\n");
		return -ENODEV;
	}

	if (hdcp_disabled)
		DEV_WARN("WARNING: HDCP disabled\n");

	return 0;
}

static boolean hdmi_msm_is_power_on(void)
{
	/* HDMI_CTRL, ENABLE */
	return (HDMI_INP(0x0000) & 0x00000001) ? TRUE : FALSE;
}

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL_DVI_SUPPORT
/* 1.2.1.2.1 DVI Operation
 * HDMI compliance requires the HDMI core to support DVI as well. The
 * HDMI core also supports DVI. In DVI operation there are no preambles
 * and guardbands transmitted. THe TMDS encoding of video data remains
 * the same as HDMI. There are no VBI or audio packets transmitted. In
 * order to enable DVI mode in HDMI core, HDMI_DVI_SEL field of
 * HDMI_CTRL register needs to be programmed to 0. */
static boolean hdmi_msm_is_dvi_mode(void)
{
	/* HDMI_CTRL, HDMI_DVI_SEL */
	return (HDMI_INP(0x0000) & 0x00000002) ? FALSE : TRUE;
}
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL_DVI_SUPPORT */

static void hdmi_msm_set_mode(boolean power_on)
{
	uint32 reg_val = 0;

	if (power_on) {
		/* ENABLE */
		reg_val |= 0x00000001; /* Enable the block */
		/* RB_SWITCH_EN */
		reg_val |= 0x00100000; /* Swap R&B */
#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL_DVI_SUPPORT
		/* HDMI_DVI_SEL */
		reg_val |= 0x00000002;
		/* HDMI_CTRL */
		HDMI_OUTP(0x0000, reg_val);
		/* HDMI_DVI_SEL */
		reg_val &= ~0x00000002;
#else /* CONFIG_FB_MSM_HDMI_MSM_PANEL_DVI_SUPPORT */
		reg_val |= 0x00000002;
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL_DVI_SUPPORT */
	} else {
		reg_val = 0x00000002;
	}
	/* HDMI_CTRL */
	HDMI_OUTP(0x0000, reg_val);
}

static void hdmi_msm_init_phy(int video_format)
{
	/* De-serializer delay D/C for non-lbk mode */
	/* PHY REG0 = (DESER_SEL(0) | DESER_DEL_CTRL(3) | AMUX_OUT_SEL(0)) */
	HDMI_OUTP_ND(0x0300, 0x0C); /*0b00001100*/

	if (video_format == MSM_HDMI_VFRMT_720x480p60_16_9) {
		/* PHY REG1 = DTEST_MUX_SEL(5) | PLL_GAIN_SEL(0)
			    | OUTVOL_SWING_CTRL(3) */
		HDMI_OUTP_ND(0x0304, 0x53); /*0b01010011*/
	} else {
		/* If the freq. is less than 120MHz, use low gain 0 for board
		   with termination */
		/* PHY REG1 = DTEST_MUX_SEL(5) | PLL_GAIN_SEL(0)
			    | OUTVOL_SWING_CTRL(4) */
		HDMI_OUTP_ND(0x0304, 0x54); /*0b01010100*/
	}

	/* No matter what, start from the power down mode */
	/* PHY REG2 = PD_PWRGEN | PD_PLL | PD_DRIVE_4 | PD_DRIVE_3
		    | PD_DRIVE_2 | PD_DRIVE_1 | PD_DESER */
	HDMI_OUTP_ND(0x0308, 0x7F); /*0b01111111*/

	/* Turn PowerGen on */
	/* PHY REG2 =             PD_PLL | PD_DRIVE_4 | PD_DRIVE_3
		    | PD_DRIVE_2 | PD_DRIVE_1 | PD_DESER */
	HDMI_OUTP_ND(0x0308, 0x3F); /*0b00111111*/

	/* Turn PLL power on */
	/* PHY REG2 =                      PD_DRIVE_4 | PD_DRIVE_3
		    | PD_DRIVE_2 | PD_DRIVE_1 | PD_DESER */
	HDMI_OUTP_ND(0x0308, 0x1F); /*0b00011111*/

	/* Write to HIGH after PLL power down de-assert */
	/* PHY REG3 = PLL_ENABLE */
	HDMI_OUTP_ND(0x030C, 0x01);
	/* ASIC power on */
	/* PHY REG9 = 0 */
	HDMI_OUTP_ND(0x0324, 0x00);
	/* Enable PLL lock detect, PLL lock det will go high after lock
	   Enable the re-time logic */
	/* PHY REG12 = PLL_LOCK_DETECT_EN | RETIMING_ENABLE */
	HDMI_OUTP_ND(0x0330, 0x03); /*0b00000011*/

	/* Drivers are on */
	/* PHY REG2 = PD_DESER */
	HDMI_OUTP_ND(0x0308, 0x01); /*0b00000001*/
	/* If the RX detector is needed */
	/* PHY REG2 = RCV_SENSE_EN | PD_DESER */
	HDMI_OUTP_ND(0x0308, 0x81); /*0b10000001*/

	/* PHY REG4 = 0 */
	HDMI_OUTP_ND(0x0310, 0x00);
	/* PHY REG5 = 0 */
	HDMI_OUTP_ND(0x0314, 0x00);
	/* PHY REG6 = 0 */
	HDMI_OUTP_ND(0x0318, 0x00);
	/* PHY REG7 = 0 */
	HDMI_OUTP_ND(0x031C, 0x00);
	/* PHY REG8 = 0 */
	HDMI_OUTP_ND(0x0320, 0x00);
	/* PHY REG10 = 0 */
	HDMI_OUTP_ND(0x0328, 0x00);
	/* PHY REG11 = 0 */
	HDMI_OUTP_ND(0x032C, 0x00);
	/* If we want to use lock enable based on counting */
	/* PHY REG12 = FORCE_LOCK | PLL_LOCK_DETECT_EN | RETIMING_ENABLE */
	HDMI_OUTP_ND(0x0330, 0x13); /*0b00010011*/
}

static int hdmi_msm_video_setup(int video_format)
{
	uint32 total_v   = 0;
	uint32 total_h   = 0;
	uint32 start_h   = 0;
	uint32 end_h     = 0;
	uint32 start_v   = 0;
	uint32 end_v     = 0;
	struct hdmi_msm_disp_mode_timing_type *timing =
		hdmi_msm_supported_video_mode_lut+video_format;

	/* timing register setup */
	if (video_format >= MSM_HDMI_VFRMT_MAX || !timing->supported) {
		DEV_ERR("video format not supported: %d\n", video_format);
		return -1;
	}

	/* Hsync Total and Vsync Total */
	total_h = timing->active_h + timing->front_porch_h
		+ timing->back_porch_h + timing->pulse_width_h - 1;
	total_v = timing->active_v + timing->front_porch_v
		+ timing->back_porch_v + timing->pulse_width_v - 1;
	/* 0x02C0 HDMI_TOTAL
	   [27:16] V_TOTAL Vertical Total
	   [11:0]  H_TOTAL Horizontal Total */
	HDMI_OUTP(0x02C0, ((total_v << 16) & 0x0FFF0000)
		| ((total_h << 0) & 0x00000FFF));

	/* Hsync Start and Hsync End */
	start_h = timing->back_porch_h + timing->pulse_width_h;
	end_h   = (total_h + 1) - timing->front_porch_h;
	/* 0x02B4 HDMI_ACTIVE_H
	   [27:16] END Horizontal end
	   [11:0]  START Horizontal start */
	HDMI_OUTP(0x02B4, ((end_h << 16) & 0x0FFF0000)
		| ((start_h << 0) & 0x00000FFF));

	start_v = timing->back_porch_v + timing->pulse_width_v - 1;
	end_v   = total_v - timing->front_porch_v;
	/* 0x02B8 HDMI_ACTIVE_V
	   [27:16] END Vertical end
	   [11:0]  START Vertical start */
	HDMI_OUTP(0x02B8, ((end_v << 16) & 0x0FFF0000)
		| ((start_v << 0) & 0x00000FFF));

	if (timing->interlaced) {
		/* 0x02C4 HDMI_V_TOTAL_F2
		   [11:0] V_TOTAL_F2 Vertical total for field2 */
		HDMI_OUTP(0x02C4, ((total_v + 1) << 0) & 0x00000FFF);

		/* 0x02BC HDMI_ACTIVE_V_F2
		   [27:16] END_F2 Vertical end for field2
		   [11:0]  START_F2 Vertical start for Field2 */
		HDMI_OUTP(0x02BC,
			  (((start_v + 1) << 0) & 0x00000FFF)
			| (((end_v + 1) << 16) & 0x0FFF0000));
	} else {
		/* HDMI_V_TOTAL_F2 */
		HDMI_OUTP(0x02C4, 0);
		/* HDMI_ACTIVE_V_F2 */
		HDMI_OUTP(0x02BC, 0);
	}

	/* 0x02C8 HDMI_FRAME_CTRL
	   31 INTERLACED_EN   Interlaced or progressive enable bit
	      0: Frame in progressive
	      1: Frame is interlaced
	   29 HSYNC_HDMI_POL  HSYNC polarity fed to HDMI core
	      0: Active Hi Hsync, detect the rising edge of hsync
	      1: Active lo Hsync, Detect the falling edge of Hsync
	   28 VSYNC_HDMI_POL  VSYNC polarity fed to HDMI core
	      0: Active Hi Vsync, detect the rising edge of vsync
	      1: Active Lo Vsync, Detect the falling edge of Vsync */
	HDMI_OUTP(0x02C8,
		  ((timing->interlaced << 31) & 0x80000000)
		| ((timing->active_low_h << 29) & 0x20000000)
		| ((timing->active_low_v << 28) & 0x10000000));
	return 0;
}

struct hdmi_msm_audio_acr {
	uint32 n;	/* N parameter for clock regeneration */
	uint32 cts;	/* CTS parameter for clock regeneration */
};

struct hdmi_msm_audio_arcs {
	uint32 pclk;
	struct hdmi_msm_audio_acr lut[MSM_HDMI_SAMPLE_RATE_MAX];
};

#define HDMI_MSM_AUDIO_ARCS(pclk, ...) { pclk, __VA_ARGS__ }

/* Audio constants lookup table for hdmi_msm_audio_acr_setup */
/* Valid Pixel-Clock rates: 25.2MHz, 27MHz, 27.03MHz, 74.25MHz, 148.5MHz */
static const struct hdmi_msm_audio_arcs hdmi_msm_audio_acr_lut[] = {
	/*  25.200MHz  */
	HDMI_MSM_AUDIO_ARCS(25200, {
		{4096, 25200}, {6272, 28000}, {6144, 25200}, {12544, 28000},
		{12288, 25200}, {25088, 28000}, {24576, 25200} }),
	/*  27.000MHz  */
	HDMI_MSM_AUDIO_ARCS(27000, {
		{4096, 27000}, {6272, 30000}, {6144, 27000}, {12544, 30000},
		{12288, 27000}, {25088, 30000}, {24576, 27000} }),
	/*  27.030MHz */
	HDMI_MSM_AUDIO_ARCS(27030, {
		{4096, 27030}, {6272, 30030}, {6144, 27030}, {12544, 30030},
		{12288, 27030}, {25088, 30030}, {24576, 27030} }),
	/*  74.250MHz */
	HDMI_MSM_AUDIO_ARCS(74250, {
		{4096, 74250}, {6272, 82500}, {6144, 74250}, {12544, 82500},
		{12288, 74250}, {25088, 82500}, {24576, 74250} }),
	/* 148.500MHz */
	HDMI_MSM_AUDIO_ARCS(148500, {
		{4096, 148500}, {6272, 165000}, {6144, 148500}, {12544, 165000},
		{12288, 148500}, {25088, 165000}, {24576, 148500} }),
};

static void hdmi_msm_audio_acr_setup(boolean enabled, int video_format,
	int audio_sample_rate, int num_of_channels)
{
	/* Read first before writing */
	/* HDMI_ACR_PKT_CTRL[0x0024] */
	uint32 acr_pck_ctrl_reg = HDMI_INP(0x0024);

	if (enabled) {
		const struct hdmi_msm_disp_mode_timing_type *timing =
			hdmi_msm_supported_video_mode_lut+video_format;
		const struct hdmi_msm_audio_arcs *audio_arc =
			&hdmi_msm_audio_acr_lut[0];
		const int lut_size = sizeof(hdmi_msm_audio_acr_lut)
			/sizeof(*hdmi_msm_audio_acr_lut);
		uint32 i, n, cts, layout, multiplier, aud_pck_ctrl_2_reg;

		if (video_format >= MSM_HDMI_VFRMT_MAX
			|| !timing->supported) {
			DEV_WARN("%s: video format %d not supported\n",
				__func__, video_format);
			return;
		}

		for (i = 0; i < lut_size;
			audio_arc = &hdmi_msm_audio_acr_lut[++i]) {
			if (audio_arc->pclk == timing->pixel_freq)
				break;
		}
		if (i >= lut_size) {
			DEV_WARN("%s: pixel clock %d not supported\n", __func__,
				timing->pixel_freq);
			return;
		}

		n = audio_arc->lut[audio_sample_rate].n;
		cts = audio_arc->lut[audio_sample_rate].cts;
		layout = (MSM_HDMI_AUDIO_CHANNEL_2 == num_of_channels) ? 0 : 1;

		if ((MSM_HDMI_SAMPLE_RATE_192KHZ == audio_sample_rate) ||
		    (MSM_HDMI_SAMPLE_RATE_176_4KHZ == audio_sample_rate)) {
			multiplier = 4;
			n >>= 2; /* divide N by 4 and use multiplier */
		} else if ((MSM_HDMI_SAMPLE_RATE_96KHZ == audio_sample_rate) ||
			  (MSM_HDMI_SAMPLE_RATE_88_2KHZ == audio_sample_rate)) {
			multiplier = 2;
			n >>= 1;	/* divide N by 2 and use multiplier */
		} else {
			multiplier = 1;
		}
		DEV_DBG("%s: n=%u, cts=%u, layout=%u\n", __func__, n, cts,
			layout);

		/* AUDIO_PRIORITY | SOURCE */
		acr_pck_ctrl_reg |= 0x80000100;
		/* N_MULTIPLE(multiplier) */
		acr_pck_ctrl_reg |= (multiplier & 7) << 16;

		if ((MSM_HDMI_SAMPLE_RATE_48KHZ == audio_sample_rate) ||
		    (MSM_HDMI_SAMPLE_RATE_96KHZ == audio_sample_rate) ||
		    (MSM_HDMI_SAMPLE_RATE_192KHZ == audio_sample_rate)) {
			/* SELECT(3) */
			acr_pck_ctrl_reg |= 3 << 4;
			/* CTS_48 */
			cts <<= 12;

			/* CTS: need to determine how many fractional bits */
			/* HDMI_ACR_48_0 */
			HDMI_OUTP(0x00D4, cts);
			/* N */
			/* HDMI_ACR_48_1 */
			HDMI_OUTP(0x00D8, n);
		} else if ((MSM_HDMI_SAMPLE_RATE_44_1KHZ == audio_sample_rate)
			   || (MSM_HDMI_SAMPLE_RATE_88_2KHZ ==
			       audio_sample_rate)
			   || (MSM_HDMI_SAMPLE_RATE_176_4KHZ ==
			       audio_sample_rate)) {
			/* SELECT(2) */
			acr_pck_ctrl_reg |= 2 << 4;
			/* CTS_44 */
			cts <<= 12;

			/* CTS: need to determine how many fractional bits */
			/* HDMI_ACR_44_0 */
			HDMI_OUTP(0x00CC, cts);
			/* N */
			/* HDMI_ACR_44_1 */
			HDMI_OUTP(0x00D0, n);
		} else {	/* default to 32k */
			/* SELECT(1) */
			acr_pck_ctrl_reg |= 1 << 4;
			/* CTS_32 */
			cts <<= 12;

			/* CTS: need to determine how many fractional bits */
			/* HDMI_ACR_32_0 */
			HDMI_OUTP(0x00C4, cts);
			/* N */
			/* HDMI_ACR_32_1 */
			HDMI_OUTP(0x00C8, n);
		}
		/* Payload layout depends on number of audio channels */
		/* LAYOUT_SEL(layout) */
		aud_pck_ctrl_2_reg = 1 | (layout << 1);
		/* override | layout */
		/* HDMI_AUDIO_PKT_CTRL2[0x00044] */
		HDMI_OUTP(0x00044, aud_pck_ctrl_2_reg);

		/* SEND | CONT */
		acr_pck_ctrl_reg |= 0x00000003;
	} else {
		/* ~(SEND | CONT) */
		acr_pck_ctrl_reg &= ~0x00000003;
	}
	/* HDMI_ACR_PKT_CTRL[0x0024] */
	HDMI_OUTP(0x0024, acr_pck_ctrl_reg);
}

static void hdmi_msm_outpdw_chk(uint32 offset, uint32 data)
{
	uint32 check, i = 0;

#ifdef DEBUG
	HDMI_OUTP(offset, data);
#endif
	do {
		outpdw(HDMI_BASE+offset, data);
		check = inpdw(HDMI_BASE+offset);
	} while (check != data && i++ < 10);

	if (check != data)
		DEV_ERR("%s: failed addr=%08x, data=%x, check=%x",
			__func__, offset, data, check);
}

static void hdmi_msm_rmw32or(uint32 offset, uint32 data)
{
	uint32 reg_data;
	reg_data = inpdw(HDMI_BASE+offset);
	reg_data = inpdw(HDMI_BASE+offset);
	hdmi_msm_outpdw_chk(offset, reg_data | data);
}

/* Audio info packet parameters */
static void hdmi_msm_audio_info_setup(boolean enabled, int num_of_channels,
	int level_shift, boolean down_mix)
{
	uint32 channel_allocation = 0;	/* Default to FR,FL */
	uint32 channel_count = 1;	/* Default to 2 channels
					   -> See Table 17 in CEA-D spec */
	uint32 check_sum, audio_info_0_reg, audio_info_1_reg;
	uint32 audio_info_ctrl_reg;

	/* Please see table 20 Audio InfoFrame in HDMI spec
	   FL  = front left
	   FC  = front Center
	   FR  = front right
	   FLC = front left center
	   FRC = front right center
	   RL  = rear left
	   RC  = rear center
	   RR  = rear right
	   RLC = rear left center
	   RRC = rear right center
	   LFE = low frequency effect
	 */

	/* Read first then write because it is bundled with other controls */
	/* HDMI_INFOFRAME_CTRL0[0x002C] */
	audio_info_ctrl_reg = HDMI_INP(0x002C);

	if (enabled) {
		switch (num_of_channels) {
		case MSM_HDMI_AUDIO_CHANNEL_2:
			break;
		case MSM_HDMI_AUDIO_CHANNEL_4:
			channel_count = 3;
			/* FC,LFE,FR,FL */
			channel_allocation = 0x3;
			break;
		case MSM_HDMI_AUDIO_CHANNEL_6:
			channel_count = 5;
			/* RR,RL,FC,LFE,FR,FL */
			channel_allocation = 0xB;
			break;
		case MSM_HDMI_AUDIO_CHANNEL_8:
			channel_count = 7;
			/* FRC,FLC,RR,RL,FC,LFE,FR,FL */
			channel_allocation = 0x1f;
			break;
		default:
			break;
		}

		/* Program the Channel-Speaker allocation */
		audio_info_1_reg = 0;
		/* CA(channel_allocation) */
		audio_info_1_reg |= channel_allocation & 0xff;
		/* Program the Level shifter */
		/* LSV(level_shift) */
		audio_info_1_reg |= (level_shift << 11) & 0x00007800;
		/* Program the Down-mix Inhibit Flag */
		/* DM_INH(down_mix) */
		audio_info_1_reg |= (down_mix << 15) & 0x00008000;

		/* HDMI_AUDIO_INFO1[0x00E8] */
		HDMI_OUTP(0x00E8, audio_info_1_reg);

		/* Calculate CheckSum
		   Sum of all the bytes in the Audio Info Packet bytes
		   (See table 8.4 in HDMI spec) */
		check_sum = 0;
		/* HDMI_AUDIO_INFO_FRAME_PACKET_HEADER_TYPE[0x84] */
		check_sum += 0x84;
		/* HDMI_AUDIO_INFO_FRAME_PACKET_HEADER_VERSION[0x01] */
		check_sum += 1;
		/* HDMI_AUDIO_INFO_FRAME_PACKET_LENGTH[0x0A] */
		check_sum += 0x0A;
		check_sum += channel_count;
		check_sum += channel_allocation;
		/* See Table 8.5 in HDMI spec */
		check_sum += (level_shift & 0xF) << 3 | (down_mix & 0x1) << 7;
		check_sum &= 0xFF;
		check_sum = (uint8) (256 - check_sum);

		audio_info_0_reg = 0;
		/* CHECKSUM(check_sum) */
		audio_info_0_reg |= check_sum & 0xff;
		/* CC(channel_count) */
		audio_info_0_reg |= (channel_count << 8) & 0x00000700;

		/* HDMI_AUDIO_INFO0[0x00E4] */
		HDMI_OUTP(0x00E4, audio_info_0_reg);

		/* Set these flags */
		/* AUDIO_INFO_UPDATE | AUDIO_INFO_SOURCE | AUDIO_INFO_CONT
		 | AUDIO_INFO_SEND */
		audio_info_ctrl_reg |= 0x000000F0;
	} else {
		/* Clear these flags */
		/* ~(AUDIO_INFO_UPDATE | AUDIO_INFO_SOURCE | AUDIO_INFO_CONT
		   | AUDIO_INFO_SEND) */
		audio_info_ctrl_reg &= ~0x000000F0;
	}
	/* HDMI_INFOFRAME_CTRL0[0x002C] */
	HDMI_OUTP(0x002C, audio_info_ctrl_reg);
}

static void hdmi_msm_audio_ctrl_setup(boolean enabled, int delay)
{
	uint32 audio_pkt_ctrl_reg = 0;

	/* Enable Packet Transmission */
	audio_pkt_ctrl_reg |= enabled ? 0x00000001 : 0;
	audio_pkt_ctrl_reg |= (delay << 4);

	/* HDMI_AUDIO_PKT_CTRL1[0x0020] */
	HDMI_OUTP(0x0020, audio_pkt_ctrl_reg);
}

static void hdmi_msm_en_gc_packet(boolean av_mute_is_requested)
{
	/* HDMI_GC[0x0040] */
	HDMI_OUTP(0x0040, av_mute_is_requested ? 1 : 0);

	/* GC packet enable (every frame) */
	/* HDMI_VBI_PKT_CTRL[0x0028] */
	hdmi_msm_rmw32or(0x0028, 3 << 4);
}

static void hdmi_msm_en_isrc_packet(boolean isrc_is_continued)
{
	static char isrc_psuedo_data[] = "ISRC1:0123456789isrc2=ABCDEFGHIJ";
	const uint32 * isrc_data = (const uint32 *) isrc_psuedo_data;

	/* ISRC_STATUS =0b010 | ISRC_CONTINUE | ISRC_VALID */
	/* HDMI_ISRC1_0[0x00048] */
	HDMI_OUTP(0x00048, 2 | (isrc_is_continued ? 1 : 0) << 6 | 0 << 7);

	/* HDMI_ISRC1_1[0x004C] */
	HDMI_OUTP(0x004C, *isrc_data++);
	/* HDMI_ISRC1_2[0x0050] */
	HDMI_OUTP(0x0050, *isrc_data++);
	/* HDMI_ISRC1_3[0x0054] */
	HDMI_OUTP(0x0054, *isrc_data++);
	/* HDMI_ISRC1_4[0x0058] */
	HDMI_OUTP(0x0058, *isrc_data++);

	/* HDMI_ISRC2_0[0x005C] */
	HDMI_OUTP(0x005C, *isrc_data++);
	/* HDMI_ISRC2_1[0x0060] */
	HDMI_OUTP(0x0060, *isrc_data++);
	/* HDMI_ISRC2_2[0x0064] */
	HDMI_OUTP(0x0064, *isrc_data++);
	/* HDMI_ISRC2_3[0x0068] */
	HDMI_OUTP(0x0068, *isrc_data);

	/* HDMI_VBI_PKT_CTRL[0x0028] */
	/* ISRC Send + Continuous */
	hdmi_msm_rmw32or(0x0028, 3 << 8);
}

static void hdmi_msm_en_acp_packet(uint32 byte1)
{
	/* HDMI_ACP[0x003C] */
	HDMI_OUTP(0x003C, 2 | 1 << 8 | byte1 << 16);

	/* HDMI_VBI_PKT_CTRL[0x0028] */
	/* ACP send, s/w source */
	hdmi_msm_rmw32or(0x0028, 3 << 12);
}

static void hdmi_msm_audio_setup(void)
{
	const int channels = MSM_HDMI_AUDIO_CHANNEL_2;

	/* (0) for clr_avmute, (1) for set_avmute */
	hdmi_msm_en_gc_packet(0);
	/* (0) for isrc1 only, (1) for isrc1 and isrc2 */
	hdmi_msm_en_isrc_packet(1);
	/* arbitrary bit pattern for byte1 */
	hdmi_msm_en_acp_packet(0x5a);

	hdmi_msm_audio_acr_setup(TRUE,
		hdmi_msm_state->video_resolution,
		MSM_HDMI_SAMPLE_RATE_48KHZ, channels);
	hdmi_msm_audio_info_setup(TRUE, channels, 0, FALSE);
	hdmi_msm_audio_ctrl_setup(TRUE, 1);
}

static void hdmi_msm_audio_off(void)
{
	hdmi_msm_audio_info_setup(FALSE, 0, 0, FALSE);
	hdmi_msm_audio_ctrl_setup(FALSE, 0);
	hdmi_msm_audio_acr_setup(FALSE, 0, 0, 0);
}

static int hdmi_msm_power_on(struct platform_device *pdev)
{
	int rc;

	DEV_DBG("power: ON\n");

	if (hdmi_msm_state->disp_powered_up)
		return 0;

	rc = clk_enable(hdmi_msm_state->hdmi_app_clk);
	if (rc) {
		DEV_ERR("'%s' clock enable failed, rc=%d\n",
			"hdmi_app_clk", rc);
		goto error;
	}

	hdmi_msm_init_phy(hdmi_msm_state->video_resolution);
	/* HDMI_USEC_REFTIMER[0x0208] */
	HDMI_OUTP(0x0208, 0x0001001B);
	rc = hdmi_msm_video_setup(hdmi_msm_state->video_resolution);
	if (rc)
		goto error;

	hdmi_msm_state->disp_powered_up = TRUE;
	enable_irq(hdmi_msm_state->irq);

	hdmi_msm_audio_setup();

	/* Turn on HDMI Engine */
	hdmi_msm_set_mode(TRUE);

#ifdef CONFIG_FB_MSM_HDMI_MSM_PANEL_DVI_SUPPORT
	DEV_DBG("power=%s, DVI=%s\n",
		 hdmi_msm_is_power_on() ? "ON" : "OFF",
		 hdmi_msm_is_dvi_mode() ? "ON" : "OFF");
#else /* CONFIG_FB_MSM_HDMI_MSM_PANEL_DVI_SUPPORT */
	DEV_DBG("power=%s\n", hdmi_msm_is_power_on() ? "ON" : "OFF");
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL_DVI_SUPPORT */

	return 0;

error:
	hdmi_msm_state->disp_powered_up = FALSE;
	hdmi_msm_set_mode(FALSE);

	disable_irq(hdmi_msm_state->irq);
	clk_disable(hdmi_msm_state->hdmi_app_clk);

	hdmi_msm_audio_off();

	return rc;
}

static int hdmi_msm_power_off(struct platform_device *pdev)
{
	DEV_DBG("power: OFF\n");
	if (!hdmi_msm_state->disp_powered_up)
		return 0;

	hdmi_msm_state->disp_powered_up = FALSE;
	hdmi_msm_set_mode(FALSE);

	disable_irq(hdmi_msm_state->irq);
	clk_disable(hdmi_msm_state->hdmi_app_clk);

	hdmi_msm_audio_off();

	return 0;
}

static int __init hdmi_msm_probe(struct platform_device *pdev)
{
	int rc;

	if (!hdmi_msm_state) {
		pr_err("%s: hdmi_msm_state is NULL\n", __func__);
		return -ENOMEM;
	}

	hdmi_msm_state->pdev = pdev;
	DEV_DBG("probe\n");
	if (pdev->id == 0) {
		struct resource *res;

		#define GET_RES(name, mode) do {			\
			res = platform_get_resource_byname(pdev, mode, name); \
			if (!res) {					\
				DEV_ERR("'" name "' resource not found\n"); \
				rc = -ENODEV;				\
				goto error;				\
			}						\
		} while (0)

		#define IO_REMAP(var, name) do {			\
			GET_RES(name, IORESOURCE_MEM);			\
			var = ioremap(res->start, resource_size(res));	\
			if (!var) {					\
				DEV_ERR("'" name "' ioremap failed\n");	\
				rc = -ENOMEM;				\
				goto error;				\
			}						\
		} while (0)

		#define GET_IRQ(var, name) do {				\
			GET_RES(name, IORESOURCE_IRQ);			\
			var = res->start;				\
		} while (0)

		IO_REMAP(hdmi_msm_state->qfprom_io, "hdmi_msm_qfprom_addr");
		IO_REMAP(hdmi_msm_state->hdmi_io, "hdmi_msm_hdmi_addr");
		GET_IRQ(hdmi_msm_state->irq, "hdmi_msm_irq");

		#undef GET_RES
		#undef IO_REMAP
		#undef GET_IRQ
		return 0;
	}

	hdmi_msm_state->hdmi_app_clk = clk_get(NULL, "hdmi_app_clk");
	if (IS_ERR(hdmi_msm_state->hdmi_app_clk)) {
		DEV_ERR("'hdmi_app_clk' clk not found\n");
		rc = IS_ERR(hdmi_msm_state->hdmi_app_clk);
		goto error;
	}

	rc = check_hdmi_features();
	if (rc) {
		DEV_ERR("Init FAILED: check_hdmi_features rc=%d\n", rc);
		goto error;
	}

	rc = request_irq(hdmi_msm_state->irq, &hdmi_msm_isr,
		IRQF_TRIGGER_HIGH, "hdmi_msm_isr", NULL);
	if (rc) {
		DEV_ERR("Init FAILED: IRQ request, rc=%d\n", rc);
		goto error;
	}
	disable_irq(hdmi_msm_state->irq);

	msm_fb_add_device(pdev);

	return 0;

error:
	if (hdmi_msm_state->qfprom_io)
		iounmap(hdmi_msm_state->qfprom_io);
	hdmi_msm_state->qfprom_io = NULL;

	if (hdmi_msm_state->hdmi_io)
		iounmap(hdmi_msm_state->hdmi_io);
	hdmi_msm_state->hdmi_io = NULL;

	if (hdmi_msm_state->hdmi_app_clk)
		clk_put(hdmi_msm_state->hdmi_app_clk);
	hdmi_msm_state->hdmi_app_clk = NULL;
	return rc;
}

static int __devexit hdmi_msm_remove(struct platform_device *pdev)
{
	DEV_DBG("remove\n");

	if (hdmi_msm_state->qfprom_io)
		iounmap(hdmi_msm_state->qfprom_io);
	hdmi_msm_state->qfprom_io = NULL;

	if (hdmi_msm_state->hdmi_io)
		iounmap(hdmi_msm_state->hdmi_io);
	hdmi_msm_state->hdmi_io = NULL;

	free_irq(hdmi_msm_state->irq, NULL);

	if (hdmi_msm_state->hdmi_app_clk)
		clk_put(hdmi_msm_state->hdmi_app_clk);
	hdmi_msm_state->hdmi_app_clk = NULL;

	kfree(hdmi_msm_state);
	hdmi_msm_state = NULL;

	return 0;
}

static struct platform_driver this_driver = {
	.probe = hdmi_msm_probe,
	.remove = hdmi_msm_remove,
	.driver.name = "hdmi_msm",
};

static struct msm_fb_panel_data hdmi_msm_panel_data = {
	.on = hdmi_msm_power_on,
	.off = hdmi_msm_power_off,
};

static struct platform_device this_device = {
	.name = "hdmi_msm",
	.id = 1,
	.dev.platform_data = &hdmi_msm_panel_data,
};

static void init_panel_info(int video_format)
{
	struct msm_panel_info *pinfo = &hdmi_msm_panel_data.panel_info;

	struct hdmi_msm_disp_mode_timing_type *timing =
		hdmi_msm_supported_video_mode_lut+video_format;

	if (video_format >= MSM_HDMI_VFRMT_MAX || !timing->supported)
		return;

	pinfo->xres = timing->active_h;
	pinfo->yres = timing->active_v;
	pinfo->clk_rate = timing->pixel_freq*1000;

	pinfo->lcdc.h_back_porch = timing->back_porch_h;
	pinfo->lcdc.h_front_porch = timing->front_porch_h;
	pinfo->lcdc.h_pulse_width = timing->pulse_width_h;
	pinfo->lcdc.v_back_porch = timing->back_porch_v;
	pinfo->lcdc.v_front_porch = timing->front_porch_v;
	pinfo->lcdc.v_pulse_width = timing->pulse_width_v;

	pinfo->type = DTV_PANEL;
	pinfo->pdest = DISPLAY_2;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 1;

	/* blk */
	pinfo->lcdc.border_clr = 0;
	/* blue */
	pinfo->lcdc.underflow_clr = 0xff;
	pinfo->lcdc.hsync_skew = 0;
}

static int __init hdmi_msm_init(void)
{
	int rc;

	if (msm_fb_detect_client("hdmi_msm"))
		return 0;

	hdmi_msm_state = kzalloc(sizeof(*hdmi_msm_state), GFP_KERNEL);
	if (!hdmi_msm_state) {
		pr_err("hdmi_msm_init FAILED: out of memory\n");
		rc = -ENOMEM;
		goto init_exit;
	}
	hdmi_msm_state->video_resolution = MSM_HDMI_VFRMT_1920x1080p60_16_9;

	rc = platform_driver_register(&this_driver);
	if (rc) {
		pr_err("hdmi_msm_init FAILED: platform_driver_register rc=%d\n",
		       rc);
		goto init_exit;
	}

	init_panel_info(hdmi_msm_state->video_resolution);

	rc = platform_device_register(&this_device);
	if (rc) {
		pr_err("hdmi_msm_init FAILED: platform_device_register rc=%d\n",
		       rc);
		platform_driver_unregister(&this_driver);
		goto init_exit;
	}

	pr_info("%s: success:"
#ifdef DEBUG
		" DEBUG"
#else
		" RELEASE"
#endif
		" EDID:0"
		" HPD:0"
		" HDCP:0"
		" AUDIO"
		" DVI"
#ifndef CONFIG_FB_MSM_HDMI_MSM_PANEL_DVI_SUPPORT
		":0"
#endif /* CONFIG_FB_MSM_HDMI_MSM_PANEL_DVI_SUPPORT */
		"\n", __func__);

	return 0;

init_exit:
	kfree(hdmi_msm_state);
	hdmi_msm_state = NULL;
	return rc;
}

static void __exit hdmi_msm_exit(void)
{
	platform_device_unregister(&this_device);
	platform_driver_unregister(&this_driver);
}

module_init(hdmi_msm_init);
module_exit(hdmi_msm_exit);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("HDMI MSM TX driver");
