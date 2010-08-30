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

#include <mach/hardware.h>
#include <asm/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <linux/fb.h>

#include "mdp.h"
#include "msm_fb.h"

static int cursor_enabled;

int mdp_hw_cursor_update(struct fb_info *info, struct fb_cursor *cursor)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct fb_image *img = &cursor->image;
	int calpha_en, transp_en;
	int alpha;
	int ret = 0;

	if ((img->width > MDP_CURSOR_WIDTH) ||
	    (img->height > MDP_CURSOR_HEIGHT) ||
	    (img->depth != 32))
		return -EINVAL;

	if (cursor->set & FB_CUR_SETPOS)
		MDP_OUTP(MDP_BASE + 0x9004c, (img->dy << 16) | img->dx);

	if (cursor->set & FB_CUR_SETIMAGE) {
		ret = copy_from_user(mfd->cursor_buf, img->data,
					img->width*img->height*4);
		if (ret)
			return ret;

		if (img->bg_color == 0xffffffff)
			transp_en = 0;
		else
			transp_en = 1;

		alpha = (img->fg_color & 0xff000000) >> 24;

		if (alpha)
			calpha_en = 0x2; /* xrgb */
		else
			calpha_en = 0x1; /* argb */

		MDP_OUTP(MDP_BASE + 0x90044, (img->height << 16) | img->width);
		MDP_OUTP(MDP_BASE + 0x90048, mfd->cursor_buf_phys);
		MDP_OUTP(MDP_BASE + 0x90060,
			 (transp_en << 3) | (calpha_en << 1) |
			 (inp32(MDP_BASE + 0x90060) & 0x1));
		MDP_OUTP(MDP_BASE + 0x90064,
			 (alpha << 24) | (0xffffff & img->bg_color));
		MDP_OUTP(MDP_BASE + 0x90068, 0);
	}

	if ((cursor->enable) && (!cursor_enabled)) {
		cursor_enabled = 1;
		MDP_OUTP(MDP_BASE + 0x90060, inp32(MDP_BASE + 0x90060) | 0x1);
	} else if ((!cursor->enable) && (cursor_enabled)) {
		cursor_enabled = 0;
		MDP_OUTP(MDP_BASE + 0x90060,
			 inp32(MDP_BASE + 0x90060) & (~0x1));
	}

	return 0;
}
