
/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <mach/hardware.h>
#include <linux/io.h>
#include <linux/debugfs.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"

void mdp4_sw_reset(ulong bits)
{
	bits &= 0x1f;	/* 5 bits */
	outpdw(MDP_BASE + 0x001c, bits);	/* MDP_SW_RESET */

	while (inpdw(MDP_BASE + 0x001c) & bits) /* self clear when complete */
		;
	MSM_FB_INFO("mdp4_sw_reset: 0x%x\n", (int)bits);
}

void mdp4_overlay_cfg(int overlayer, int blt_mode, int refresh, int direct_out)
{
	ulong bits = 0;

	if (blt_mode)
		bits |= (1 << 3);
	refresh &= 0x03;	/* 2 bites */
	bits |= (refresh << 1);
	direct_out &= 0x01;
	bits |= direct_out;

	if (overlayer == OVERLAY0_MIXER)
		outpdw(MDP_BASE + 0x10004, bits); /* MDP_OVERLAY0_CFG */
	else
		outpdw(MDP_BASE + 0x18004, bits); /* MDP_OVERLAY1_CFG */

	MSM_FB_INFO("mdp4_overlay_cfg: 0x%x\n", (int)inpdw(MDP_BASE + 0x10004));
}

void mdp4_display_intf_sel(int output, ulong intf)
{
	ulong bits, mask;

	bits = inpdw(MDP_BASE + 0x0038);	/* MDP_DISP_INTF_SEL */

	mask = 0x03;	/* 2 bits */
	intf &= 0x03;	/* 2 bits */

	switch (output) {
	case EXTERNAL_INTF_SEL:
		intf <<= 4;
		mask <<= 4;
		break;
	case SECONDARY_INTF_SEL:
		intf &= 0x02;	/* only MDDI and EBI2 support */
		intf <<= 2;
		mask <<= 2;
		break;
	default:
		break;
	}


	bits &= ~mask;
	bits |= intf;

	outpdw(MDP_BASE + 0x0038, bits);	/* MDP_DISP_INTF_SEL */

  MSM_FB_INFO("mdp4_display_intf_sel: 0x%x\n", (int)inpdw(MDP_BASE + 0x0038));
}

unsigned long mdp4_display_status(void)
{
	return inpdw(MDP_BASE + 0x0018) & 0x3ff;	/* MDP_DISPLAY_STATUS */
}

void mdp4_ebi2_lcd_setup(int lcd, ulong base, int ystride)
{
	/* always use memory map */
	ystride &= 0x01fff;	/* 13 bits */
	if (lcd == EBI2_LCD0) {
		outpdw(MDP_BASE + 0x0060, base);/* MDP_EBI2_LCD0 */
		outpdw(MDP_BASE + 0x0068, ystride);/* MDP_EBI2_LCD0_YSTRIDE */
	} else {
		outpdw(MDP_BASE + 0x0064, base);/* MDP_EBI2_LCD1 */
		outpdw(MDP_BASE + 0x006c, ystride);/* MDP_EBI2_LCD1_YSTRIDE */
	}
}

void mdp4_mddi_setup(int mddi, unsigned long id)
{
	ulong 	bits;

	if (mddi == MDDI_EXTERNAL_SET)
		bits = 0x02;
	else if (mddi == MDDI_SECONDARY_SET)
		bits = 0x01;
	else
		bits = 0;	/* PRIMARY_SET */

	id <<= 16;

	bits |= id;

	outpdw(MDP_BASE + 0x0090, bits); /* MDP_MDDI_PARAM_WR_SEL */
}

int mdp_ppp_blit(struct fb_info *info, struct mdp_blit_req *req,
	struct file **pp_src_file, struct file **pp_dst_file)
{

	/* not implemented yet */
	return -1;
}

void mdp4_hw_init(void)
{
	ulong bits;

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

#ifdef MDP4_ERROR
	/*
	 * Issue software reset on DMA_P will casue DMA_P dma engine stall
	 * on LCDC mode. However DMA_P does not stall at MDDI mode.
	 * This need further investigation.
	 */
	mdp4_sw_reset(0x17);
#endif

	mdp4_clear_lcdc();

	outp32(MDP_EBI2_PORTMAP_MODE, 0x3);

	/* system interrupts */

	bits =  mdp_intr_mask;
	outpdw(MDP_BASE + 0x0050, bits);/* enable specififed interrupts */

	/* histogram */
	MDP_OUTP(MDP_BASE + 0x95010, 1);	/* auto clear HIST */

	/* enable histogram interrupts */
	outpdw(MDP_BASE + 0x9501c, INTR_HIST_DONE);

	/* For the max read pending cmd config below, if the MDP clock     */
	/* is less than the AXI clock, then we must use 3 pending          */
	/* pending requests.  Otherwise, we should use 8 pending requests. */
	/* In the future we should do this detection automatically.	   */

	/* max read pending cmd config */
	outpdw(MDP_BASE + 0x004c, 0x02222);	/* 3 pending requests */

	/* dma_p fetch config */
	outpdw(MDP_BASE + 0x91004, 0x27);	/* burst size of 8 */

	/* both REFRESH_MODE and DIRECT_OUT are ignored at BLT mode */
	mdp4_overlay_cfg(OVERLAY0_MIXER, OVERLAY_MODE_BLT, 0, 0);

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
}


void mdp4_clear_lcdc(void)
{
	uint32 bits;

	bits = inpdw(MDP_BASE + 0xc0000);
	if (bits & 0x01) /* enabled already */
		return;

	outpdw(MDP_BASE + 0xc0004, 0);	/* vsync ctrl out */
	outpdw(MDP_BASE + 0xc0008, 0);	/* vsync period */
	outpdw(MDP_BASE + 0xc000c, 0);	/* vsync pusle width */
	outpdw(MDP_BASE + 0xc0010, 0);	/* lcdc display HCTL */
	outpdw(MDP_BASE + 0xc0014, 0);	/* lcdc display v start */
	outpdw(MDP_BASE + 0xc0018, 0);	/* lcdc display v end */
	outpdw(MDP_BASE + 0xc001c, 0);	/* lcdc active hctl */
	outpdw(MDP_BASE + 0xc0020, 0);	/* lcdc active v start */
	outpdw(MDP_BASE + 0xc0024, 0);	/* lcdc active v end */
	outpdw(MDP_BASE + 0xc0028, 0);	/* lcdc board color */
	outpdw(MDP_BASE + 0xc002c, 0);	/* lcdc underflow ctrl */
	outpdw(MDP_BASE + 0xc0030, 0);	/* lcdc hsync skew */
	outpdw(MDP_BASE + 0xc0034, 0);	/* lcdc test ctl */
	outpdw(MDP_BASE + 0xc0038, 0);	/* lcdc ctl polarity */
}

static struct mdp_dma_data dma_e_data;
static struct mdp_dma_data overlay0_data;
static struct mdp_dma_data overlay1_data;
static int intr_dma_p;
static int intr_dma_s;
static int intr_dma_e;
static int intr_overlay0;
static int intr_overlay1;

irqreturn_t mdp4_isr(int irq, void *ptr)
{
	uint32 isr, mask, lcdc;
	struct mdp_dma_data *dma;

	mdp_is_in_isr = TRUE;

	while (1) {
		isr = inpdw(MDP_INTR_STATUS);
		if (isr == 0)
			break;

		mask = inpdw(MDP_INTR_ENABLE);
		outpdw(MDP_INTR_CLEAR, isr);

		isr &= mask;

		if (unlikely(isr == 0))
			break;

		if (isr & INTR_DMA_P_DONE) {
			intr_dma_p++;
			lcdc = inpdw(MDP_BASE + 0xc0000);
			dma = &dma2_data;
			if (lcdc & 0x01) {	/* LCDC enable */
				/* disable LCDC interrupt */
				mdp_intr_mask &= ~INTR_DMA_P_DONE;
				outp32(MDP_INTR_ENABLE, mdp_intr_mask);
				dma->waiting = FALSE;
			} else {
				dma->busy = FALSE;
				mdp_pipe_ctrl(MDP_DMA2_BLOCK,
					MDP_BLOCK_POWER_OFF, TRUE);
			}
			complete(&dma->comp);
		}
		if (isr & INTR_DMA_S_DONE) {
			intr_dma_s++;
			dma = &dma_s_data;
			dma->busy = FALSE;
			mdp_pipe_ctrl(MDP_DMA_S_BLOCK,
					MDP_BLOCK_POWER_OFF, TRUE);
			complete(&dma->comp);
		}
		if (isr & INTR_DMA_E_DONE) {
			intr_dma_e++;
			dma = &dma_e_data;
			dma->busy = FALSE;
			mdp_pipe_ctrl(MDP_DMA_E_BLOCK,
					MDP_BLOCK_POWER_OFF, TRUE);
			complete(&dma->comp);
		}
		if (isr & INTR_OVERLAY0_DONE) {
			intr_overlay0++;
			dma = &overlay0_data;
			dma->busy = FALSE;
			mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK,
					MDP_BLOCK_POWER_OFF, TRUE);
			complete(&dma->comp);
		}
		if (isr & INTR_OVERLAY1_DONE) {
			intr_overlay1++;
			dma = &overlay1_data;
			dma->busy = FALSE;
			mdp_pipe_ctrl(MDP_OVERLAY1_BLOCK,
					MDP_BLOCK_POWER_OFF, TRUE);
			complete(&dma->comp);
		}
		if (isr & INTR_DMA_P_HISTOGRAM) {
			isr = inpdw(MDP_DMA_P_HIST_INTR_STATUS);
			mask = inpdw(MDP_DMA_P_HIST_INTR_ENABLE);
			outpdw(MDP_DMA_P_HIST_INTR_CLEAR, isr);
			isr &= mask;
			if (isr & INTR_HIST_DONE) {
				if (mdp_hist.r)
					memcpy(mdp_hist.r, MDP_BASE + 0x95100,
							mdp_hist.bin_cnt*4);
				if (mdp_hist.g)
					memcpy(mdp_hist.g, MDP_BASE + 0x95200,
							mdp_hist.bin_cnt*4);
				if (mdp_hist.b)
					memcpy(mdp_hist.b, MDP_BASE + 0x95300,
						mdp_hist.bin_cnt*4);
				complete(&mdp_hist_comp);
			}
		}
	}

	mdp_is_in_isr = FALSE;

	return IRQ_HANDLED;
}
