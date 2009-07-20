/*
 * drivers/media/video/omap/omapvout-mem.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * Based on drivers/media/video/omap24xx/omap24xxvout.c&h
 *
 * Copyright (C) 2005-2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>

#include <asm/processor.h>
#include <asm/cacheflush.h>
#include <asm/page.h>

#include "omapvout.h"
#include "omapvout-mem.h"

int omapvout_mem_alloc(u32 size, u32 *phy_addr, u32 *virt_addr)
{
	int	order;
	u32	dss_page_addr;
	u32	dss_page_phy;
	u32	dss_page_virt;
	u32	used, alloc_end;
	struct page	*tmp_page;

	size = PAGE_ALIGN(size);
	order = get_order(size);

	dss_page_addr = __get_free_pages(GFP_KERNEL, order);
	if (!dss_page_addr) {
		printk(KERN_ERR "Failed to allocate pages !!!! \n");
		return -ENOMEM;
	}

	/*
	 *'alloc_pages' allocates pages in power of 2,
	 *so free the not needed pages
	 */
	split_page(virt_to_page(dss_page_addr), order);
	alloc_end = dss_page_addr + (PAGE_SIZE<<order);
	used = dss_page_addr + size;

	DBG("mem_alloc: dss_page_addr=0x%x, alloc_end=0x%x, used=0x%x\n"
		, dss_page_addr, alloc_end, used);
	DBG("mem_alloc: physical_start=0x%lx, order=0x%x, size=0x%x\n"
		, virt_to_phys((void *)dss_page_addr), order, size);

	while (used < alloc_end) {
		BUG_ON(!virt_addr_valid((void *)used));
		tmp_page = virt_to_page((void *)used);
		__free_page(tmp_page);
		used += PAGE_SIZE;
	}

	dss_page_phy = virt_to_phys((void *)dss_page_addr);
	dss_page_virt = (u32) ioremap_cached(dss_page_phy, size);

	*phy_addr = dss_page_phy;
	*virt_addr = dss_page_virt;

	return 0;
}

void omapvout_mem_free(u32 phy_addr, u32 virt_addr, u32 size)
{
	u32 vaddr;
	u32	end;

	size = PAGE_ALIGN(size);
	vaddr = (u32) __va((void *)phy_addr);
	end = vaddr + size;
	while (vaddr < end) {
		free_page(vaddr);
		vaddr += PAGE_SIZE;
	}
	iounmap((void *) virt_addr);
}

int omapvout_mem_map(struct vm_area_struct *vma, u32 phy_addr)
{
	struct page *cpage;
	void *pos;
	u32 start;
	u32 size;


	vma->vm_flags |= VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	pos = (void *) phy_addr;
	start = vma->vm_start;
	size = (vma->vm_end - vma->vm_start);

	while (size > 0) {
		cpage = pfn_to_page(((unsigned int)pos) >> PAGE_SHIFT);
		if (vm_insert_page(vma, start, cpage)) {
			printk(KERN_ERR "Failed to insert page to VMA \n");
			return -EAGAIN;
		}
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	vma->vm_flags &= ~VM_IO; /* using shared anonymous pages */

	return 0;
}


