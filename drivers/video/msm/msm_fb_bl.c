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
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/backlight.h>

#include "msm_fb.h"

static int msm_fb_bl_get_brightness(struct backlight_device *pbd)
{
	return pbd->props.brightness;
}

static int msm_fb_bl_update_status(struct backlight_device *pbd)
{
	struct msm_fb_data_type *mfd = bl_get_data(pbd);
	__u32 bl_lvl;

	bl_lvl = pbd->props.brightness;
	bl_lvl = mfd->fbi->bl_curve[bl_lvl];
	msm_fb_set_backlight(mfd, bl_lvl, 1);
	return 0;
}

static struct backlight_ops msm_fb_bl_ops = {
	.get_brightness = msm_fb_bl_get_brightness,
	.update_status = msm_fb_bl_update_status,
};

void msm_fb_config_backlight(struct msm_fb_data_type *mfd)
{
	struct msm_fb_panel_data *pdata;
	struct backlight_device *pbd;
	struct fb_info *fbi;
	char name[16];

	fbi = mfd->fbi;
	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	if ((pdata) && (pdata->set_backlight)) {
		snprintf(name, sizeof(name), "msmfb_bl%d", mfd->index);
		pbd =
		    backlight_device_register(name, fbi->dev, mfd,
					      &msm_fb_bl_ops);
		if (!IS_ERR(pbd)) {
			fbi->bl_dev = pbd;
			fb_bl_default_curve(fbi,
					    0,
					    mfd->panel_info.bl_min,
					    mfd->panel_info.bl_max);
			pbd->props.max_brightness = FB_BACKLIGHT_LEVELS - 1;
			pbd->props.brightness = FB_BACKLIGHT_LEVELS - 1;
			backlight_update_status(pbd);
		} else {
			fbi->bl_dev = NULL;
			printk(KERN_ERR "msm_fb: backlight_device_register failed!\n");
		}
	}
}
