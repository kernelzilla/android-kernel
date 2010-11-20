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
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>

#include "msm_fb_panel.h"

int panel_next_on(struct platform_device *pdev)
{
	int ret = 0;
	struct msm_fb_panel_data *pdata;
	struct msm_fb_panel_data *next_pdata;
	struct platform_device *next_pdev;

	pdata = (struct msm_fb_panel_data *)pdev->dev.platform_data;

	if (pdata) {
		next_pdev = pdata->next;
		if (next_pdev) {
			next_pdata =
			    (struct msm_fb_panel_data *)next_pdev->dev.
			    platform_data;
			if ((next_pdata) && (next_pdata->on))
				ret = next_pdata->on(next_pdev);
		}
	}

	return ret;
}

int panel_next_off(struct platform_device *pdev)
{
	int ret = 0;
	struct msm_fb_panel_data *pdata;
	struct msm_fb_panel_data *next_pdata;
	struct platform_device *next_pdev;

	pdata = (struct msm_fb_panel_data *)pdev->dev.platform_data;

	if (pdata) {
		next_pdev = pdata->next;
		if (next_pdev) {
			next_pdata =
			    (struct msm_fb_panel_data *)next_pdev->dev.
			    platform_data;
			if ((next_pdata) && (next_pdata->on))
				ret = next_pdata->off(next_pdev);
		}
	}

	return ret;
}

struct platform_device *msm_fb_device_alloc(struct msm_fb_panel_data *pdata,
						u32 type, u32 id)
{
	struct platform_device *this_dev = NULL;
	char dev_name[16];

	switch (type) {
	case EBI2_PANEL:
		snprintf(dev_name, sizeof(dev_name), "ebi2_lcd");
		break;

	case MDDI_PANEL:
		snprintf(dev_name, sizeof(dev_name), "mddi");
		break;

	case EXT_MDDI_PANEL:
		snprintf(dev_name, sizeof(dev_name), "mddi_ext");
		break;

	case TV_PANEL:
		snprintf(dev_name, sizeof(dev_name), "tvenc");
		break;

	case LCDC_PANEL:
		snprintf(dev_name, sizeof(dev_name), "lcdc");
		break;

	default:
		return NULL;
	}

	if (pdata != NULL)
		pdata->next = NULL;
	else
		return NULL;

	this_dev =
	    platform_device_alloc(dev_name, ((u32) type << 16) | (u32) id);

	if (this_dev) {
		if (platform_device_add_data
		    (this_dev, pdata, sizeof(struct msm_fb_panel_data))) {
			printk
			    ("msm_fb_device_alloc: platform_device_add_data failed!\n");
			platform_device_put(this_dev);
			return NULL;
		}
	}

	return this_dev;
}
