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

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <mach/irqs.h>
#include "clock.h"
#include "clock-8x60.h"

/* Address of GSBI blocks */
#define MSM_GSBI1_PHYS	0x16000000
#define MSM_GSBI2_PHYS	0x16100000
#define MSM_GSBI3_PHYS	0x16200000
#define MSM_GSBI4_PHYS	0x16300000
#define MSM_GSBI5_PHYS	0x16400000
#define MSM_GSBI6_PHYS	0x16500000
#define MSM_GSBI7_PHYS	0x16600000
#define MSM_GSBI8_PHYS	0x19800000
#define MSM_GSBI9_PHYS	0x19900000
#define MSM_GSBI10_PHYS	0x19A00000
#define MSM_GSBI11_PHYS	0x19B00000
#define MSM_GSBI12_PHYS	0x19C00000

/* GSBI QUPe devices */
#define MSM_GSBI1_QUP_I2C_PHYS	0x16080000
#define MSM_GSBI2_QUP_I2C_PHYS	0x16180000
#define MSM_GSBI3_QUP_I2C_PHYS	0x16280000
#define MSM_GSBI4_QUP_I2C_PHYS	0x16380000
#define MSM_GSBI5_QUP_I2C_PHYS	0x16480000
#define MSM_GSBI6_QUP_I2C_PHYS	0x16580000
#define MSM_GSBI7_QUP_I2C_PHYS	0x16680000
#define MSM_GSBI8_QUP_I2C_PHYS	0x19880000
#define MSM_GSBI9_QUP_I2C_PHYS	0x19980000
#define MSM_GSBI10_QUP_I2C_PHYS	0x19A80000
#define MSM_GSBI11_QUP_I2C_PHYS	0x19B80000
#define MSM_GSBI12_QUP_I2C_PHYS	0x19C80000

static struct resource gsbi3_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI3_QUP_I2C_PHYS,
		.end	= MSM_GSBI3_QUP_I2C_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI3_PHYS,
		.end	= MSM_GSBI3_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI3_QUP_IRQ,
		.end	= GSBI3_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi4_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI4_QUP_I2C_PHYS,
		.end	= MSM_GSBI4_QUP_I2C_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI4_PHYS,
		.end	= MSM_GSBI4_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI4_QUP_IRQ,
		.end	= GSBI4_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi8_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI8_QUP_I2C_PHYS,
		.end	= MSM_GSBI8_QUP_I2C_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI8_PHYS,
		.end	= MSM_GSBI8_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI8_QUP_IRQ,
		.end	= GSBI8_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource gsbi9_qup_i2c_resources[] = {
	{
		.name	= "qup_phys_addr",
		.start	= MSM_GSBI9_QUP_I2C_PHYS,
		.end	= MSM_GSBI9_QUP_I2C_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI9_PHYS,
		.end	= MSM_GSBI9_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "qup_err_intr",
		.start	= GSBI9_QUP_IRQ,
		.end	= GSBI9_QUP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

/* Use GSBI3 QUP for /dev/i2c-0 */
struct platform_device msm_gsbi3_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(gsbi3_qup_i2c_resources),
	.resource	= gsbi3_qup_i2c_resources,
};

/* Use GSBI4 QUP for /dev/i2c-1 */
struct platform_device msm_gsbi4_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(gsbi4_qup_i2c_resources),
	.resource	= gsbi4_qup_i2c_resources,
};

/* Use GSBI8 QUP for /dev/i2c-3 */
struct platform_device msm_gsbi8_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(gsbi8_qup_i2c_resources),
	.resource	= gsbi8_qup_i2c_resources,
};

/* Use GSBI9 QUP for /dev/i2c-2 */
struct platform_device msm_gsbi9_qup_i2c_device = {
	.name		= "qup_i2c",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(gsbi9_qup_i2c_resources),
	.resource	= gsbi9_qup_i2c_resources,
};

struct clk msm_clocks_8x60[] = {
	CLK_8X60("bbrx_ssbi_clk",	BBRX_SSBI_CLK,		NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI1_UART_CLK,		NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI2_UART_CLK,		NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI3_UART_CLK,		NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI4_UART_CLK,		NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI5_UART_CLK,		NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI6_UART_CLK,		NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI7_UART_CLK,		NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI8_UART_CLK,		NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI9_UART_CLK,		NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI10_UART_CLK,	NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI11_UART_CLK,	NULL, 0),
	CLK_8X60("gsbi_uart_clk",	GSBI12_UART_CLK,	NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI1_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI2_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI3_QUP_CLK,
					&msm_gsbi3_qup_i2c_device.dev, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI4_QUP_CLK,
					&msm_gsbi4_qup_i2c_device.dev, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI5_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI6_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI7_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI8_QUP_CLK,
					&msm_gsbi8_qup_i2c_device.dev, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI9_QUP_CLK,
					&msm_gsbi9_qup_i2c_device.dev, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI10_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI11_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI12_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_src_clk",	GSBI_SIM_SRC_CLK,	NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI1_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI2_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI3_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI4_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI5_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI6_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI7_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI8_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI9_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI10_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI11_SIM_CLK,		NULL, 0),
	CLK_8X60("gsbi_sim_clk",	GSBI12_SIM_CLK,		NULL, 0),
	CLK_8X60("pdm_clk",		PDM_CLK,		NULL, 0),
	CLK_8X60("prng_clk",		PRNG_CLK,		NULL, 0),
	CLK_8X60("pmic_ssbi2_clk",	PMIC_SSBI2_CLK,		NULL, 0),
	CLK_8X60("sdc_clk",		SDC1_CLK,		NULL, 0),
	CLK_8X60("sdc_clk",		SDC2_CLK,		NULL, 0),
	CLK_8X60("sdc_clk",		SDC3_CLK,		NULL, 0),
	CLK_8X60("sdc_clk",		SDC4_CLK,		NULL, 0),
	CLK_8X60("sdc_clk",		SDC5_CLK,		NULL, 0),
	CLK_8X60("tsif_ref_clk",	TSIF_REF_CLK,		NULL, 0),
	CLK_8X60("tssc_clk",		TSSC_CLK,		NULL, 0),
	CLK_8X60("usb_hs_clk",		USB_HS_CLK,		NULL, 0),
	CLK_8X60("usb_phy_clk",		USB_PHY0_CLK,		NULL, 0),
	CLK_8X60("usb_fs_src_clk",	USB_FS1_SRC_CLK,	NULL, 0),
	CLK_8X60("usb_fs_clk",		USB_FS1_CLK,		NULL, 0),
	CLK_8X60("usb_fs_sys_clk",	USB_FS1_SYS_CLK,	NULL, 0),
	CLK_8X60("usb_fs_src_clk",	USB_FS2_SRC_CLK,	NULL, 0),
	CLK_8X60("usb_fs_clk",		USB_FS2_CLK,		NULL, 0),
	CLK_8X60("usb_fs_sys_clk",	USB_FS2_SYS_CLK,	NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI1_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI2_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI3_P_CLK,
					&msm_gsbi3_qup_i2c_device.dev, 0),
	CLK_8X60("gsbi_pclk",		GSBI4_P_CLK,
					&msm_gsbi4_qup_i2c_device.dev, 0),
	CLK_8X60("gsbi_pclk",		GSBI5_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI6_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI7_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI8_P_CLK,
					&msm_gsbi8_qup_i2c_device.dev, 0),
	CLK_8X60("gsbi_pclk",		GSBI9_P_CLK,
					&msm_gsbi9_qup_i2c_device.dev, 0),
	CLK_8X60("gsbi_pclk",		GSBI10_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI11_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI12_P_CLK,		NULL, 0),
	CLK_8X60("tsif_pclk",		TSIF_P_CLK,		NULL, 0),
	CLK_8X60("usb_fs_pclk",		USB_FS1_P_CLK,		NULL, 0),
	CLK_8X60("usb_fs_pclk",		USB_FS2_P_CLK,		NULL, 0),
	CLK_8X60("cam_clk",		CAM_CLK,		NULL, 0),
	CLK_8X60("csi_src_clk",		CSI_SRC_CLK,		NULL, 0),
	CLK_8X60("csi_clk",		CSI0_CLK,		NULL, 0),
	CLK_8X60("csi_clk",		CSI1_CLK,		NULL, 0),
	CLK_8X60("gfx2d_clk",		GFX2D0_CLK,		NULL, 0),
	CLK_8X60("gfx2d_clk",		GFX2D1_CLK,		NULL, 0),
	CLK_8X60("gfx3d_clk",		GFX3D_CLK,		NULL, 0),
	CLK_8X60("ijpeg_clk",		IJPEG_CLK,		NULL, 0),
	CLK_8X60("jpegd_clk",		JPEGD_CLK,		NULL, 0),
	CLK_8X60("mdp_clk",		MDP_CLK,		NULL, 0),
	CLK_8X60("mdp_vsync_clk",	MDP_VSYNC_CLK,		NULL, 0),
	CLK_8X60("pixel_mdp_clk",	PIXEL_MDP_CLK,		NULL, 0),
	CLK_8X60("pixel_lcdc_clk",	PIXEL_LCDC_CLK,		NULL, 0),
	CLK_8X60("rot_clk",		ROT_CLK,		NULL, 0),
	CLK_8X60("tv_src_clk",		TV_SRC_CLK,		NULL, 0),
	CLK_8X60("tv_enc_clk",		TV_ENC_CLK,		NULL, 0),
	CLK_8X60("tv_dac_clk",		TV_DAC_CLK,		NULL, 0),
	CLK_8X60("vcodec_clk",		VCODEC_CLK,		NULL, 0),
	CLK_8X60("mdp_tv_clk",		MDP_TV_CLK,		NULL, 0),
	CLK_8X60("hdmi_tv_clk",		HDMI_TV_CLK,		NULL, 0),
	CLK_8X60("dsub_tv_clk",		DSUB_TV_CLK,		NULL, 0),
	CLK_8X60("vpe_clk",		VPE_CLK,		NULL, 0),
	CLK_8X60("vfe_src_clk",		VFE_SRC_CLK,		NULL, 0),
	CLK_8X60("vfe_clk",		VFE_CLK,		NULL, 0),
	CLK_8X60("cs_vfe_clk",		CS0_VFE_CLK,		NULL, 0),
	CLK_8X60("cs_vfe_clk",		CS1_VFE_CLK,		NULL, 0),
	CLK_8X60("mi2s_src_clk",	MI2S_SRC_CLK,		NULL, 0),
	CLK_8X60("mi2s_clk",		MI2S_CLK,		NULL, 0),
	CLK_8X60("mi2s_m_clk",		MI2S_M_CLK,		NULL, 0),
	CLK_8X60("pcm_clk",		PCM_CLK,		NULL, 0),
};

unsigned msm_num_clocks_8x60 = ARRAY_SIZE(msm_clocks_8x60);
