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
#include "clock.h"
#include "clock-8x60.h"

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
	CLK_8X60("gsbi_qup_clk",	GSBI3_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI4_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI5_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI6_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI7_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI8_QUP_CLK,		NULL, 0),
	CLK_8X60("gsbi_qup_clk",	GSBI9_QUP_CLK,		NULL, 0),
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
	CLK_8X60("gsbi_pclk",		GSBI3_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI4_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI5_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI6_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI7_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI8_P_CLK,		NULL, 0),
	CLK_8X60("gsbi_pclk",		GSBI9_P_CLK,		NULL, 0),
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

