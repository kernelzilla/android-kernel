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
#include <linux/sched.h>
#include <linux/msm_kgsl.h>
#include "kgsl_g12_cmdwindow.h"
#include "kgsl_g12_cmdstream.h"
#include "kgsl_g12_drawctxt.h"
#include "kgsl_cmdstream.h"

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_g12.h"
#include "kgsl_g12_drawctxt.h"
#include "kgsl_log.h"

#include "g12_reg.h"

int kgsl_g12_cmdstream_check_timestamp(struct kgsl_g12_device *g12_device,
					unsigned int timestamp)
{
	int ts_diff;

	ts_diff = g12_device->timestamp - timestamp;

	return (ts_diff >= 0) || (ts_diff < -20000);
}

static int room_in_rb(struct kgsl_g12_device *device)
{
	int ts_diff;

	ts_diff = device->current_timestamp - device->timestamp;

	return ts_diff < KGSL_G12_PACKET_COUNT;
}

static inline unsigned int rb_offset(unsigned int index)
{
	return index*sizeof(unsigned int)*KGSL_G12_PACKET_SIZE;
}

static void addcmd(struct kgsl_g12_z1xx *z1xx, unsigned int index,
			unsigned int cmd, unsigned int nextcnt)
{
	unsigned int *p = z1xx->cmdbufdesc.hostptr + rb_offset(index);

	*p++ = 0x7C000176;
	*p++ = 5;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = 0x7C000275;
	*p++ = cmd;
	*p++ = 0x1000 | nextcnt;
	*p++ = ADDR_VGV3_LAST << 24;
	*p++ = ADDR_VGV3_LAST << 24;
}

int
kgsl_g12_cmdstream_issueibcmds(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable,
			int drawctxt_index,
			uint32_t ibaddr,
			int sizedwords,
			int *timestamp,
			unsigned int ctrl)
{
	unsigned int result = 0;
	unsigned int ofs        = PACKETSIZE_STATESTREAM * sizeof(unsigned int);
	unsigned int cnt        = 5;
	unsigned int nextaddr   = 0;
	unsigned int index	= 0;
	unsigned int nextcnt    = 0x9000 | 5;
	struct kgsl_memdesc tmp = {0};
	unsigned int cmd;
	struct kgsl_g12_device *g12_device = (struct kgsl_g12_device *) device;

	cmd = ibaddr;

	tmp.hostptr = (void *)*timestamp;

	KGSL_CMD_INFO("ctxt %d ibaddr 0x%08x sizedwords %d",
			drawctxt_index, ibaddr, sizedwords);
	/* context switch */
	if (drawctxt_index != (int)g_z1xx.prevctx) {
		kgsl_mmu_setstate(device, pagetable);
		cnt = PACKETSIZE_STATESTREAM;
		ofs = 0;
	} else {
		kgsl_g12_setstate(device, device->mmu.tlb_flags);
	}

	result = wait_event_interruptible_timeout(g12_device->wait_timestamp_wq,
				  room_in_rb(g12_device),
				  msecs_to_jiffies(KGSL_TIMEOUT_DEFAULT));
	if (result < 0) {
		KGSL_CMD_ERR("failed waiting for ringbuffer. result %d",
			     result);
		goto error;
	}
	result = 0;

	index = g12_device->current_timestamp % KGSL_G12_PACKET_COUNT;
	g12_device->current_timestamp++;
	*timestamp = g12_device->current_timestamp;

	g_z1xx.prevctx = drawctxt_index;

	addcmd(&g_z1xx, index, cmd + ofs, cnt);

	nextaddr = g_z1xx.cmdbufdesc.physaddr
		 + rb_offset((index + 1) % KGSL_G12_PACKET_COUNT);

	tmp.hostptr = (void *)(tmp.hostptr +
			(sizedwords * sizeof(unsigned int)));
	tmp.size = 12;

	kgsl_sharedmem_writel(&tmp, 4, nextaddr);
	kgsl_sharedmem_writel(&tmp, 8, nextcnt);

	kgsl_g12_cmdwindow_write(device,
				KGSL_CMDWINDOW_2D, ADDR_VGV3_CONTROL, 1);
	kgsl_g12_cmdwindow_write(device,
				KGSL_CMDWINDOW_2D, ADDR_VGV3_CONTROL, 0);
error:
	return result;
}

void kgsl_g12_cmdstream_memqueue_drain(struct kgsl_g12_device *g12_device)
{
	struct kgsl_mem_entry *entry, *entry_tmp;
	uint32_t ts_processed;
	struct kgsl_device *device = &g12_device->dev;
	struct kgsl_ringbuffer *rb = &device->ringbuffer;

	/* get current EOP timestamp */
	ts_processed = g12_device->timestamp;

	list_for_each_entry_safe(entry, entry_tmp, &rb->memqueue, free_list) {
		/*NOTE: this assumes that the free list is sorted by
		 * timestamp, but I'm not yet sure that it is a valid
		 * assumption
		 */
		if (!timestamp_cmp(ts_processed, entry->free_timestamp))
			break;
		KGSL_MEM_DBG("ts_processed %d ts_free %d gpuaddr %x)\n",
			     ts_processed, entry->free_timestamp,
			     entry->memdesc.gpuaddr);
		kgsl_remove_mem_entry(entry, true);
	}
}

