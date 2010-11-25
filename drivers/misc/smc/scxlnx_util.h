/*
 *Copyright (c)2006-2008 Trusted Logic S.A.
 *All Rights Reserved.
 *
 *This program is free software; you can redistribute it and/or
 *modify it under the terms of the GNU General Public License
 *version 2 as published by the Free Software Foundation.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *You should have received a copy of the GNU General Public License
 *along with this program; if not, write to the Free Software
 *Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 *MA 02111-1307 USA
 */
#ifndef __SCXLNX_UTIL_H__
#define __SCXLNX_UTIL_H__

#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/crypto.h>
#include <linux/mount.h>
#include <linux/pagemap.h>
#include <linux/vmalloc.h>
#include <asm/byteorder.h>
#include <linux/interrupt.h>

#include "scxlnx_defs.h"

/*----------------------------------------------------------------------------
 *Debug printing routines
 *---------------------------------------------------------------------------- */

#ifdef SMODULE_SMC_OMAP3430_POWER_MANAGEMENT
/*#define powerPrintk printk */
#define powerPrintk dprintk
#endif

#ifdef DEBUG

#define dprintk  printk

void SCXLNXDumpL1SharedBuffer(
		SCHANNEL_C1S_BUFFER *pBuf);

void SCXLNXDumpMessage(
		SCX_COMMAND_MESSAGE *pMessage);

void SCXLNXDumpAnswer(
		SCX_ANSWER_MESSAGE *pAnswer);

#else /*defined(DEBUG)*/

#define dprintk(args...)do { ; } while (0)
#define SCXLNXDumpL1SharedBuffer(pBuf)((void)0)
#define SCXLNXDumpMessage(pMessage)((void)0)
#define SCXLNXDumpAnswer(pAnswer)((void)0)

#endif /*defined(DEBUG)*/


/*----------------------------------------------------------------------------
 *Process identification
 *---------------------------------------------------------------------------- */

int SCXLNXConnGetCurrentProcessHash(
		void *pHash);

int SCXLNXConnGetCurrentProcessUUID(
		SCX_UUID *pUUID);


/*----------------------------------------------------------------------------
 *Statistic computation
 *---------------------------------------------------------------------------- */

void *internal_kmalloc(size_t nSize, int nPriority);
void internal_kfree(void *pMemory);
void *internal_vmalloc(size_t nSize);
void internal_vfree(void *pMemory);
unsigned long internal_get_zeroed_page(int nPriority);
void internal_free_page(unsigned long pPage);
unsigned long internal_get_free_pages(int nPriority, unsigned int order);
void internal_free_pages(unsigned long addr, unsigned int order);
int internal_get_user_pages(
		struct task_struct *tsk,
		struct mm_struct *mm,
		unsigned long start,
		int len,
		int write,
		int force,
		struct page * *pages,
		struct vm_area_struct * *vmas);
void internal_get_page(struct page *page);
void internal_page_cache_release(struct page *page);

#endif  /*__SCXLNX_UTIL_H__ */
