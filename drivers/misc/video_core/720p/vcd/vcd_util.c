/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, and instead of the terms immediately above, this
 * software may be relicensed by the recipient at their option under the
 * terms of the GNU General Public License version 2 ("GPL") and only
 * version 2.  If the recipient chooses to relicense the software under
 * the GPL, then the recipient shall replace all of the text immediately
 * above and including this paragraph with the text immediately below
 * and between the words START OF ALTERNATE GPL TERMS and END OF
 * ALTERNATE GPL TERMS and such notices and license terms shall apply
 * INSTEAD OF the notices and licensing terms given above.
 *
 * START OF ALTERNATE GPL TERMS
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This software was originally licensed under the Code Aurora Forum
 * Inc. Dual BSD/GPL License version 1.1 and relicensed as permitted
 * under the terms thereof by a recipient under the General Public
 * License Version 2.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * END OF ALTERNATE GPL TERMS
 *
 */

#include "video_core_type.h"
#include "vcd_util.h"

u32 vcd_critical_section_create(u32 **p_cs)
{
	struct mutex *lock;
	if (!p_cs) {
		VCD_MSG_ERROR("Bad critical section ptr");
		return VCD_ERR_BAD_POINTER;
	} else {
		lock = kmalloc(sizeof(struct mutex), GFP_KERNEL);
		if (!lock) {
			VCD_MSG_ERROR("Failed: vcd_critical_section_create");
			return VCD_ERR_ALLOC_FAIL;
		}
		mutex_init(lock);
		*p_cs = (u32 *) lock;
		return VCD_S_SUCCESS;
	}
}

u32 vcd_critical_section_release(u32 *cs)
{
	struct mutex *lock = (struct mutex *)cs;
	if (!lock) {
		VCD_MSG_ERROR("Bad critical section object");
		return VCD_ERR_BAD_POINTER;
	}

	mutex_destroy(lock);
	kfree(cs);
	return VCD_S_SUCCESS;
}

u32 vcd_critical_section_enter(u32 *cs)
{
	struct mutex *lock = (struct mutex *)cs;
	if (!lock) {
		VCD_MSG_ERROR("Bad critical section object");
		return VCD_ERR_BAD_POINTER;
	} else
		mutex_lock(lock);

	return VCD_S_SUCCESS;
}

u32 vcd_critical_section_leave(u32 *cs)
{
	struct mutex *lock = (struct mutex *)cs;

	if (!lock) {
		VCD_MSG_ERROR("Bad critical section object");

		return VCD_ERR_BAD_POINTER;
	} else
		mutex_unlock(lock);

	return VCD_S_SUCCESS;
}

int vcd_pmem_alloc(u32 size, u8 **kernel_vaddr, u8 **phy_addr)
{
	*phy_addr =
	    (u8 *) pmem_kalloc(size, PMEM_MEMTYPE_EBI1 | PMEM_ALIGNMENT_4K);

	if (!IS_ERR((void *)*phy_addr)) {

		*kernel_vaddr = ioremap((unsigned long)*phy_addr, size);

		if (!*kernel_vaddr) {
			pr_err("%s: could not ioremap in kernel pmem buffers\n",
			       __func__);
			pmem_kfree((s32) *phy_addr);
			return -ENOMEM;
		}
		pr_debug("write buf: phy addr 0x%08x kernel addr 0x%08x\n",
			 (u32) *phy_addr, (u32) *kernel_vaddr);
		return 0;
	} else {
		pr_err("%s: could not allocte in kernel pmem buffers\n",
		       __func__);
		return -ENOMEM;
	}

}

int vcd_pmem_free(u8 *kernel_vaddr, u8 *phy_addr)
{
	iounmap((void *)kernel_vaddr);
	pmem_kfree((s32) phy_addr);

	return 0;
}
