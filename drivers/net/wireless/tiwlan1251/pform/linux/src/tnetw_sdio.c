/* tnetw_sdio.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright © Texas Instruments Incorporated (Oct 2005)
 * THIS CODE/PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDED BUT NOT LIMITED TO , THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * This program has been modified from its original operation by Texas
 * Instruments Incorporated. These changes are covered under version 2
 * of the GNU General Public License, dated June 1991.
 *
 * Copyright © Google Inc (Feb 2008)
 */
/*-------------------------------------------------------------------*/
#ifdef TIWLAN_MSM7000
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/pagemap.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>

#include <linux/delay.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include "esta_drv.h"
#include "mmc_omap_api.h"
#include "osApi.h"

/*-------------------------------------------------------------------*/
extern int tiwlan_sdio_init(struct sdio_func *func);
extern int sdio_reset_comm(struct mmc_card *card);

/*-------------------------------------------------------------------*/
static struct sdio_func *tiwlan_func = NULL;
static int sdio_reset_flag = 0;

#define DMA_THRESHOLD_SIZE  64
static void *sdio_dma_ptr = NULL;
#define USE_SKETCHY_WRITES 0

/*-------------------------------------------------------------------*/
void SDIO_SetFunc( struct sdio_func *func )
{
	tiwlan_func = func;
}

struct sdio_func *SDIO_GetFunc( void )
{
	return tiwlan_func;
}

SDIO_Status SDIO_Init(SDIO_ConfigParams *ConfigParams, SDIO_Handle *Handle)
{
	if (Handle == NULL) {
		printk(KERN_ERR "Error: SDIO_Init() called with NULL!\n");
		return SDIO_FAILURE;
	}

	*Handle = (SDIO_Handle)SDIO_GetFunc();
	if ((*Handle) == NULL) {
		printk(KERN_ERR "SDIO_Init() called before init!\n");
		return SDIO_FAILURE;
	}

	if (!sdio_dma_ptr) {
		if (!(sdio_dma_ptr = kmalloc(PAGE_SIZE, GFP_KERNEL))) {
			printk(KERN_ERR "Failed to alloc DMA bounce buffer\n");
			return SDIO_FAILURE;
		}
	}
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Shutdown(SDIO_Handle Handle)
{
	if (sdio_dma_ptr) {
		kfree(sdio_dma_ptr);
		sdio_dma_ptr = NULL;
	}
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Start(SDIO_Handle Handle)
{
	struct sdio_func *func = (struct sdio_func *)Handle;

	if (func) {
		if (sdio_reset_flag) {
			sdio_reset_flag = 0;
			if (tiwlan_sdio_init(func)) {
				printk("TI: tiwlan_sdio_init Error!\n");
				return SDIO_FAILURE;
			}

		}
	}
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Reset(SDIO_Handle Handle)
{
	struct sdio_func *func = (struct sdio_func *)Handle;

	if (func && func->card) {
		sdio_release_host(func);
		sdio_reset_comm(func->card);
		sdio_claim_host(func);
	}
	return SDIO_SUCCESS;
}

SDIO_Status SDIO_Stop(SDIO_Handle Handle, unsigned long Wait_Window)
{
	sdio_reset_flag = 1;
	return SDIO_Reset(Handle);
}

static inline int spans_page(void *s, int len)
{
	if (((unsigned long) s + len) <= ((((unsigned long) s) & ~(PAGE_SIZE-1)) + PAGE_SIZE))
		return 0;
	return 1;
}

static void *vmalloc_to_unity(void *a)
{
	struct page *pg;
	unsigned long virt = (unsigned long) a;

	pg = vmalloc_to_page(a);
	virt = (unsigned long)page_address(pg) | (virt & (PAGE_SIZE -1));
	return (void *)virt;
}

SDIO_Status SDIO_SyncRead(SDIO_Handle Handle, SDIO_Request_t *Req)
{
	struct sdio_func *func = (struct sdio_func *)Handle;
	int rc;
	void *tgt = Req->buffer;

	if (Req->buffer_len >= DMA_THRESHOLD_SIZE) {
		if (is_vmalloc_addr(tgt)) {
			if (!spans_page(tgt, Req->buffer_len)) {
				tgt = vmalloc_to_unity(tgt);
				dmac_flush_range(Req->buffer,
						 Req->buffer + Req->buffer_len);
			} else
				tgt = sdio_dma_ptr;
		}
	}

	if ((rc = sdio_memcpy_fromio(func, tgt, Req->peripheral_addr,
				     Req->buffer_len))) {
		printk(KERN_ERR "%s: failed (%d)\n", __func__, rc);
		return SDIO_FAILURE;
	}

	if (tgt == sdio_dma_ptr)
		memcpy(Req->buffer, sdio_dma_ptr, Req->buffer_len);

	return SDIO_SUCCESS;
}

SDIO_Status SDIO_SyncWrite(SDIO_Handle Handle, SDIO_Request_t *Req)
{
	struct sdio_func *func = (struct sdio_func *)Handle;
	int rc;
	void *src = Req->buffer;

	if (Req->buffer_len >= DMA_THRESHOLD_SIZE) {
#if USE_SKETCHY_WRITES
		if (is_vmalloc_addr(src)) {
			if (!spans_page(src, Req->buffer_len)) {
				src = vmalloc_to_unity(src);
				dmac_clean_range(Req->buffer,
						 Req->buffer + Req->buffer_len);
			} else {
#endif
				src = sdio_dma_ptr;
				memcpy(src, Req->buffer, Req->buffer_len);
#if USE_SKETCHY_WRITES
			}
		}
#endif
	}

	rc = sdio_memcpy_toio(func, Req->peripheral_addr, src,
			      Req->buffer_len);

	if (!rc)
		return SDIO_SUCCESS;

	printk(KERN_ERR "%s: failed (%d)\n", __func__, rc);
	return SDIO_FAILURE;
}
#endif
