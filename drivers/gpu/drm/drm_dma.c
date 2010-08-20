/**
 * \file drm_dma.c
 * DMA IOCTL and function support
 *
 * \author Rickard E. (Rik) Faith <faith@valinux.com>
 * \author Gareth Hughes <gareth@valinux.com>
 */

/*
 * Created: Fri Mar 19 14:30:16 1999 by faith@valinux.com
 *
 * Copyright 1999, 2000 Precision Insight, Inc., Cedar Park, Texas.
 * Copyright 2000 VA Linux Systems, Inc., Sunnyvale, California.
 * Copyright (c) 2009, Code Aurora Forum.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * VA LINUX SYSTEMS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "drmP.h"

/**
 * Initialize the DMA data.
 *
 * \param dev DRM device.
 * \return zero on success or a negative value on failure.
 *
 * Allocate and initialize a drm_device_dma structure.
 */
int drm_dma_setup(struct drm_device *dev)
{
	int i;

	dev->dma = drm_alloc(sizeof(*dev->dma), DRM_MEM_DRIVER);
	if (!dev->dma)
		return -ENOMEM;

	memset(dev->dma, 0, sizeof(*dev->dma));

	for (i = 0; i <= DRM_MAX_ORDER; i++)
		memset(&dev->dma->bufs[i], 0, sizeof(dev->dma->bufs[0]));

	return 0;
}

/**
 * Cleanup the DMA resources.
 *
 * \param dev DRM device.
 *
 * Free all pages associated with DMA buffers, the buffers and pages lists, and
 * finally the drm_device::dma structure itself.
 */
void drm_dma_takedown(struct drm_device *dev)
{
	struct drm_device_dma *dma = dev->dma;
	int i, j;

	if (!dma)
		return;

	/* Clear dma buffers */
	for (i = 0; i <= DRM_MAX_ORDER; i++) {
		if (dma->bufs[i].seg_count) {
			DRM_DEBUG("order %d: buf_count = %d,"
				  " seg_count = %d\n",
				  i,
				  dma->bufs[i].buf_count,
				  dma->bufs[i].seg_count);
			for (j = 0; j < dma->bufs[i].seg_count; j++) {
				if (dma->bufs[i].seglist[j]) {
					drm_dma_free(dev,
						dma->bufs[i].seglist[j]);
				}
			}
			drm_free(dma->bufs[i].seglist,
				 dma->bufs[i].seg_count
				 * sizeof(*dma->bufs[0].seglist), DRM_MEM_SEGS);
		}
		if (dma->bufs[i].buf_count) {
			for (j = 0; j < dma->bufs[i].buf_count; j++) {
				if (dma->bufs[i].buflist[j].dev_private) {
					drm_free(dma->bufs[i].buflist[j].
						 dev_private,
						 dma->bufs[i].buflist[j].
						 dev_priv_size, DRM_MEM_BUFS);
				}
			}
			drm_free(dma->bufs[i].buflist,
				 dma->bufs[i].buf_count *
				 sizeof(*dma->bufs[0].buflist), DRM_MEM_BUFS);
		}
	}

	if (dma->buflist) {
		drm_free(dma->buflist,
			 dma->buf_count * sizeof(*dma->buflist), DRM_MEM_BUFS);
	}

	if (dma->pagelist) {
		drm_free(dma->pagelist,
			 dma->page_count * sizeof(*dma->pagelist),
			 DRM_MEM_PAGES);
	}
	drm_free(dev->dma, sizeof(*dev->dma), DRM_MEM_DRIVER);
	dev->dma = NULL;
}

/**
 * Free a buffer.
 *
 * \param dev DRM device.
 * \param buf buffer to free.
 *
 * Resets the fields of \p buf.
 */
void drm_free_buffer(struct drm_device *dev, struct drm_buf * buf)
{
	if (!buf)
		return;

	buf->waiting = 0;
	buf->pending = 0;
	buf->file_priv = NULL;
	buf->used = 0;

	if (drm_core_check_feature(dev, DRIVER_DMA_QUEUE)
	    && waitqueue_active(&buf->dma_wait)) {
		wake_up_interruptible(&buf->dma_wait);
	}
}

/**
 * Reclaim the buffers.
 *
 * \param file_priv DRM file private.
 *
 * Frees each buffer associated with \p file_priv not already on the hardware.
 */
void drm_core_reclaim_buffers(struct drm_device *dev,
			      struct drm_file *file_priv)
{
	struct drm_device_dma *dma = dev->dma;
	int i;

	if (!dma)
		return;
	for (i = 0; i < dma->buf_count; i++) {
		if (dma->buflist[i]->file_priv == file_priv) {
			switch (dma->buflist[i]->list) {
			case DRM_LIST_NONE:
				drm_free_buffer(dev, dma->buflist[i]);
				break;
			case DRM_LIST_WAIT:
				dma->buflist[i]->list = DRM_LIST_RECLAIM;
				break;
			default:
				/* Buffer already on hardware. */
				break;
			}
		}
	}
}

EXPORT_SYMBOL(drm_core_reclaim_buffers);

/**
 * \brief Allocate a consistent memory block for DMA.
 */
drm_dma_handle_t *drm_dma_alloc(struct drm_device * dev, size_t size,
				size_t align, dma_addr_t maxaddr)
{
	drm_dma_handle_t *dmah;
#if 1
	unsigned long addr;
	size_t sz;
#endif
#ifdef DRM_DEBUG_MEMORY
	int area = DRM_MEM_DMA;

	spin_lock(&drm_mem_lock);
	if ((drm_ram_used >> PAGE_SHIFT)
	    > (DRM_RAM_PERCENT * drm_ram_available) / 100) {
		spin_unlock(&drm_mem_lock);
		return 0;
	}
	spin_unlock(&drm_mem_lock);
#endif

	/* pci_alloc_consistent only guarantees alignment to the smallest
	 * PAGE_SIZE order which is greater than or equal to the requested
	 * size. Return NULL here for now to make sure nobody tries for
	 * larger alignment
	 */
	if (align > size)
		return NULL;

#ifdef CONFIG_PCI
	if (!drm_core_check_feature(dev, DRIVER_USE_PLATFORM_DEVICE)) {
		if (pci_set_dma_mask(dev->pdev, maxaddr) != 0) {
			DRM_ERROR("Setting pci dma mask failed\n");
			return NULL;
		}
	}
#endif

	dmah = kmalloc(sizeof(drm_dma_handle_t), GFP_KERNEL);
	if (!dmah)
		return NULL;

	dmah->size = size;
	dmah->vaddr = dma_alloc_coherent(drm_get_device(dev), size,
		&dmah->busaddr, GFP_KERNEL | __GFP_COMP);

#ifdef DRM_DEBUG_MEMORY
	if (dmah->vaddr == NULL) {
		spin_lock(&drm_mem_lock);
		++drm_mem_stats[area].fail_count;
		spin_unlock(&drm_mem_lock);
		kfree(dmah);
		return NULL;
	}

	spin_lock(&drm_mem_lock);
	++drm_mem_stats[area].succeed_count;
	drm_mem_stats[area].bytes_allocated += size;
	drm_ram_used += size;
	spin_unlock(&drm_mem_lock);
#else
	if (dmah->vaddr == NULL) {
		kfree(dmah);
		return NULL;
	}
#endif

	memset(dmah->vaddr, 0, size);

	/* XXX - Is virt_to_page() legal for consistent mem? */
	/* Reserve */
	for (addr = (unsigned long)dmah->vaddr, sz = size;
	     sz > 0; addr += PAGE_SIZE, sz -= PAGE_SIZE) {
		SetPageReserved(virt_to_page(addr));
	}

	return dmah;
}
EXPORT_SYMBOL(drm_dma_alloc);


/**
 * \brief Free a consistent memory block without freeing its descriptor.
 *
 * This function is for internal use in the Linux-specific DRM core code.
 */
void __drm_dma_free(struct drm_device *dev, drm_dma_handle_t *dmah)
{
#if 1
	unsigned long addr;
	size_t sz;
#endif
#ifdef DRM_DEBUG_MEMORY
	int area = DRM_MEM_DMA;
	int alloc_count;
	int free_count;
#endif

	if (!dmah->vaddr) {
#ifdef DRM_DEBUG_MEMORY
		DRM_MEM_ERROR(area, "Attempt to free address 0\n");
#endif
	} else {
		/* XXX - Is virt_to_page() legal for consistent mem? */
		/* Unreserve */
		for (addr = (unsigned long)dmah->vaddr, sz = dmah->size;
		     sz > 0; addr += PAGE_SIZE, sz -= PAGE_SIZE) {
			ClearPageReserved(virt_to_page(addr));
		}
		dma_free_coherent(drm_get_device(dev), dmah->size, dmah->vaddr,
				  dmah->busaddr);
	}

#ifdef DRM_DEBUG_MEMORY
	spin_lock(&drm_mem_lock);
	free_count = ++drm_mem_stats[area].free_count;
	alloc_count = drm_mem_stats[area].succeed_count;
	drm_mem_stats[area].bytes_freed += size;
	drm_ram_used -= size;
	spin_unlock(&drm_mem_lock);
	if (free_count > alloc_count) {
		DRM_MEM_ERROR(area,
			      "Excess frees: %d frees, %d allocs\n",
			      free_count, alloc_count);
	}
#endif

}

/**
 * \brief Free a consistent memory block
 */
void drm_dma_free(struct drm_device *dev, drm_dma_handle_t *dmah)
{
	__drm_dma_free(dev, dmah);
	kfree(dmah);
}
EXPORT_SYMBOL(drm_dma_free);
