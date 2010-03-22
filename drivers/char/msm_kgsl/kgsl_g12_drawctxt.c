/* Copyright (c) 2002,2007-2010, Code Aurora Forum. All rights reserved.
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
#include <linux/string.h>
#include <linux/types.h>
#include <linux/msm_kgsl.h>

#include "kgsl_g12_drawctxt.h"
#include "kgsl_sharedmem.h"
#include "kgsl.h"
#include "kgsl_log.h"
#include "kgsl_g12_cmdwindow.h"
#include "kgsl_g12_vgv3types.h"
#include "g12_reg.h"

struct kgsl_g12_z1xx g_z1xx = {0};

static void addmarker(struct kgsl_g12_z1xx *z1xx)
{
	if (z1xx) {
		unsigned int *p = z1xx->cmdbuf[z1xx->curr];
		/* todo: use symbolic values */
		p[z1xx->offs++] = 0x7C000176;
		p[z1xx->offs++] = (0x8000 | 5);
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
		p[z1xx->offs++] = 0x7C000176;
		p[z1xx->offs++] = 5;
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
		p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
	}
}

int
kgsl_g12_drawctxt_create(struct kgsl_device *device,
			unsigned int type,
			unsigned int *drawctxt_id,
			unsigned int flags)
{
	int i;
	int cmd;
	int result;

	(void)device;
	(void)type;
	(void)flags;

	if (g_z1xx.numcontext == 0) {
		for (i = 0; i < GSL_HAL_NUMCMDBUFFERS; i++) {
			int flags = 0;
			if (kgsl_sharedmem_alloc(flags, GSL_HAL_CMDBUFFERSIZE,
					&g_z1xx.cmdbufdesc[i]) !=  0)
				return -ENOMEM;


			g_z1xx.cmdbuf[i] = kzalloc(GSL_HAL_CMDBUFFERSIZE,
						   GFP_KERNEL);


			g_z1xx.curr = i;
			g_z1xx.offs = 0;
			addmarker(&g_z1xx);
			kgsl_sharedmem_write(&g_z1xx.cmdbufdesc[i], 0,
					 g_z1xx.cmdbuf[i],
					 (512 + 13) *
					 sizeof(unsigned int));
		}
		g_z1xx.curr = 0;
		cmd = (int)(((VGV3_NEXTCMD_JUMP) &
			VGV3_NEXTCMD_NEXTCMD_FMASK)
			<< VGV3_NEXTCMD_NEXTCMD_FSHIFT);

		/* set cmd stream buffer to hw */
		result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
					 ADDR_VGV3_MODE, 4);
		if (result != 0)
			return result;
		result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
					 ADDR_VGV3_NEXTADDR,
					 g_z1xx.cmdbufdesc[0].physaddr);
		if (result != 0)
			return result;
		result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
					 ADDR_VGV3_NEXTCMD, cmd | 5);
		if (result != 0)
			return result;

		cmd = (int)(((1) & VGV3_CONTROL_MARKADD_FMASK)
			<< VGV3_CONTROL_MARKADD_FSHIFT);
		result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
					 ADDR_VGV3_CONTROL, cmd);
		if (result != 0)
			return result;
		result = kgsl_g12_cmdwindow_write(device, KGSL_CMDWINDOW_2D,
					 ADDR_VGV3_CONTROL, 0);
		if (result != 0)
			return result;
	}
	g_z1xx.numcontext++;
	*drawctxt_id = g_z1xx.numcontext;

	return KGSL_SUCCESS;
}

int
kgsl_g12_drawctxt_destroy(struct kgsl_device *device,
			unsigned int drawctxt_id)
{

	(void)device;
	(void)drawctxt_id;

	g_z1xx.numcontext--;
	if (g_z1xx.numcontext < 0) {
		g_z1xx.numcontext = 0;
		return KGSL_FAILURE;
	}

	if (g_z1xx.numcontext == 0) {
		int i;
		for (i = 0; i < GSL_HAL_NUMCMDBUFFERS; i++) {
			kgsl_sharedmem_free(&g_z1xx.cmdbufdesc[i]);
			kfree(g_z1xx.cmdbuf[i]);
		}

		memset(&g_z1xx, 0, sizeof(struct kgsl_g12_z1xx));
	}
	return KGSL_SUCCESS;
}
