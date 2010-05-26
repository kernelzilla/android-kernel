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
#include <linux/delay.h>
#include <linux/io.h>
#include "kgsl_g12_cmdstream.h"
#include "kgsl_cmdstream.h"

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_log.h"

#include "g12_reg.h"

int kgsl_g12_cmdstream_check_timestamp(struct kgsl_device *device,
					unsigned int timestamp)
{
	int ts_diff;

	ts_diff = device->timestamp - timestamp;

	return (ts_diff >= 0) || (ts_diff < -20000);
}

static void beginpacket(struct kgsl_g12_z1xx *z1xx,
			unsigned int cmd, unsigned int nextcnt)
{
	unsigned int *p = z1xx->cmdbuf[z1xx->curr];

	p[z1xx->offs++] = 0x7C000176;
	p[z1xx->offs++] = 5;
	p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
	p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
	p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
	p[z1xx->offs++] = 0x7C000275;
	p[z1xx->offs++] = cmd;
	p[z1xx->offs++] = 0x1000 | nextcnt;
	p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
	p[z1xx->offs++] = ADDR_VGV3_LAST << 24;
}

int
kgsl_g12_cmdstream_issueibcmds(struct kgsl_device *device,
			int drawctxt_index,
			uint32_t ibaddr,
			int sizedwords,
			int *timestamp,
			unsigned int flags)
{
	unsigned int ofs        = PACKETSIZE_STATESTREAM * sizeof(unsigned int);
	unsigned int cnt        = 5;
	unsigned int nextbuf    = (g_z1xx.curr + 1) % GSL_HAL_NUMCMDBUFFERS;
	unsigned int nextaddr   = g_z1xx.cmdbufdesc[nextbuf].physaddr;
	unsigned int nextcnt    = 0x9000 | 5;
	struct kgsl_memdesc tmp = {0};
	unsigned int cmd;

	kgsl_setstate(device, device->mmu.tlb_flags);

	cmd = ibaddr;

	tmp.hostptr = (void *)*timestamp;

	device->current_timestamp++;
	*timestamp = device->current_timestamp;

	/* context switch */
	if (drawctxt_index != (int)g_z1xx.prevctx) {
		cnt = PACKETSIZE_STATESTREAM;
		ofs = 0;
	}

	g_z1xx.prevctx = drawctxt_index;

	g_z1xx.offs = 10;
	beginpacket(&g_z1xx, cmd + ofs, cnt);

	tmp.hostptr = (void *)(tmp.hostptr +
			(sizedwords * sizeof(unsigned int)));
	tmp.size = 12;

	kgsl_sharedmem_writel(&tmp, 4, nextaddr);
	kgsl_sharedmem_writel(&tmp, 8, nextcnt);

	/* sync mem */
	kgsl_sharedmem_write((const struct kgsl_memdesc *)
				 &g_z1xx.cmdbufdesc[g_z1xx.curr], 0,
				 g_z1xx.cmdbuf[g_z1xx.curr],
				 (512 + 13) * sizeof(unsigned int));

	g_z1xx.offs = 0;
	g_z1xx.curr = nextbuf;

	return KGSL_SUCCESS;
}

int kgsl_g12_cmdstream_addtimestamp(struct kgsl_device *device,
			  int *timestamp)
{
    device->current_timestamp++;
    *timestamp = device->current_timestamp;

    return KGSL_SUCCESS;
}


