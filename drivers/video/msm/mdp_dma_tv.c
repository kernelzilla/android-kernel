/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
#include <linux/hrtimer.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <linux/fb.h>

#include "mdp.h"
#include "msm_fb.h"

extern spinlock_t mdp_spin_lock;
extern uint32 mdp_intr_mask;

int mdp_dma3_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct fb_info *fbi;
	uint8 *buf;
	int bpp;
	int ret = 0;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	fbi = mfd->fbi;
	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf +=
	    (fbi->var.xoffset + fbi->var.yoffset * fbi->var.xres_virtual) * bpp;

	/* starting address[31..8] of Video frame buffer is CS0 */
	MDP_OUTP(MDP_BASE + 0xC0008, (uint32) buf >> 3);

	mdp_pipe_ctrl(MDP_DMA3_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	MDP_OUTP(MDP_BASE + 0xC0004, 0x4c60674); /* flicker filter enabled */
	MDP_OUTP(MDP_BASE + 0xC0010, 0x20);	/* sobel treshold */

	MDP_OUTP(MDP_BASE + 0xC0018, 0xeb0010);	/* Y  Max, Y  min */
	MDP_OUTP(MDP_BASE + 0xC001C, 0xf00010);	/* Cb Max, Cb min */
	MDP_OUTP(MDP_BASE + 0xC0020, 0xf00010);	/* Cb Max, Cb min */

	MDP_OUTP(MDP_BASE + 0xC000C, 0x67686970); /* add a few chars for CC */
	MDP_OUTP(MDP_BASE + 0xC0000, 0x1);	/* MDP tv out enable */

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	ret = panel_next_on(pdev);

	return ret;
}

int mdp_dma3_off(struct platform_device *pdev)
{
	int ret = 0;

	ret = panel_next_off(pdev);
	if (ret)
		return ret;

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	MDP_OUTP(MDP_BASE + 0xC0000, 0x0);
	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	mdp_pipe_ctrl(MDP_DMA3_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	/* delay to make sure the last frame finishes */
	mdelay(100);

	return ret;
}

void mdp_dma3_update(struct msm_fb_data_type *mfd)
{
	struct fb_info *fbi = mfd->fbi;
	uint8 *buf;
	int bpp;
	unsigned long flag;

	if (!mfd->panel_power_on)
		return;

	/* no need to power on cmd block since dma3 is running */
	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf +=
	    (fbi->var.xoffset + fbi->var.yoffset * fbi->var.xres_virtual) * bpp;
	MDP_OUTP(MDP_BASE + 0xC0008, (uint32) buf >> 3);

	spin_lock_irqsave(&mdp_spin_lock, flag);
	mdp_enable_irq(MDP_DMA3_TERM);
	INIT_COMPLETION(mfd->dma->comp);
	mfd->dma->waiting = TRUE;

	outp32(MDP_INTR_CLEAR, TV_OUT_DMA3_START);
	mdp_intr_mask |= TV_OUT_DMA3_START;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	wait_for_completion_killable(&mfd->dma->comp);
	mdp_disable_irq(MDP_DMA3_TERM);
}
