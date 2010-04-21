/****************************************************************************
**+-----------------------------------------------------------------------+**
**|                                                                       |**
**| Copyright(c) 1998 - 2008 Texas Instruments. All rights reserved.      |**
**| All rights reserved.                                                  |**
**|                                                                       |**
**| Redistribution and use in source and binary forms, with or without    |**
**| modification, are permitted provided that the following conditions    |**
**| are met:                                                              |**
**|                                                                       |**
**|  * Redistributions of source code must retain the above copyright     |**
**|    notice, this list of conditions and the following disclaimer.      |**
**|  * Redistributions in binary form must reproduce the above copyright  |**
**|    notice, this list of conditions and the following disclaimer in    |**
**|    the documentation and/or other materials provided with the         |**
**|    distribution.                                                      |**
**|  * Neither the name Texas Instruments nor the names of its            |**
**|    contributors may be used to endorse or promote products derived    |**
**|    from this software without specific prior written permission.      |**
**|                                                                       |**
**| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   |**
**| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     |**
**| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR |**
**| A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  |**
**| OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |**
**| SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      |**
**| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, |**
**| DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY |**
**| THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   |**
**| (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE |**
**| OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  |**
**|                                                                       |**
**+-----------------------------------------------------------------------+**
****************************************************************************/


#include "arch_ti.h"

#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/timer.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/list.h>

#include "osApi.h"
#include "osTIType.h"
#include "esta_drv.h"

typedef void (*os_free)(void *);
struct os_mem_block
{
    struct list_head blk_list;
    os_free f_free;
    __u32 size;
    __u32 signature;
};
#define MEM_BLOCK_START  (('m'<<24) | ('e'<<16) | ('m'<<8) | 's')
#define MEM_BLOCK_END    (('m'<<24) | ('e'<<16) | ('m'<<8) | 'e')

/****************************************************************************************
 *                                                                                      *
 *                      OS Memory API                                                   *
 *                                                                                      *
 ****************************************************************************************/

/****************************************************************************************
 *                        os_memoryAlloc()
 ****************************************************************************************
DESCRIPTION:    Allocates resident (nonpaged) system-space memory.

ARGUMENTS:      OsContext   - our adapter context.
                Size        - Specifies the size, in bytes, to be allocated.

RETURN:         Pointer to the allocated memory.
                NULL if there is insufficient memory available.

NOTES:          With the call to vmalloc it is assumed that this function will
                never be called in an interrupt context. vmalloc has the potential to
                sleep the caller while waiting for memory to become available.

*****************************************************************************************/
PVOID
os_memoryAlloc(
        TI_HANDLE OsContext,
        UINT32 Size
        )
{
    struct os_mem_block *blk;
    __u32 total_size = Size + sizeof(struct os_mem_block) + sizeof(__u32);

#ifdef TI_MEM_ALLOC_TRACE
    os_printf("MTT:%s:%d ::os_memoryAlloc(0x%p, %lu) : %lu\n",__FUNCTION__, __LINE__,OsContext,Size,total_size);
#endif
    if( total_size < Size ) { /* Dm: Security fix */
        return NULL;
    }
    /*
        memory optimization issue. Allocate 8 kB and less from the SLAB allocator (2^n)
        otherwise allocate from virtual pool.
    */
    /* 2 pages */
    if (Size <= 2 * 4096)
    {
        if (in_atomic())
            blk = kmalloc(total_size, GFP_ATOMIC);
        else
            blk = kmalloc(total_size, GFP_KERNEL);
        if (!blk)
            return NULL;
        blk->f_free = (os_free)kfree;
    }
    else
    {
        /* We expect that the big allocations should be made outside the interrupt,
            otherwise fail
        */
        if (in_atomic())
            return NULL;
        blk = vmalloc(total_size);
        if (!blk)
            return NULL;
        blk->f_free = (os_free)vfree;
    }

    os_profile (OsContext, 4, total_size);

    /*list_add(&blk->blk_list, &drv->mem_blocks);*/
    blk->size = Size;
    blk->signature = MEM_BLOCK_START;
    *(__u32 *)((unsigned char *)blk + total_size - sizeof(__u32)) = MEM_BLOCK_END;
    return (PVOID)((char *)blk + sizeof(struct os_mem_block));
}

/****************************************************************************************
 *                        os_memoryPreFree()
 ****************************************************************************************
DESCRIPTION:    Frees preallocated by the kernel memory.

ARGUMENTS:      ptr - pointer to memory
*****************************************************************************************/
void os_memoryPreFree( void *ptr )
{
}

/****************************************************************************************
 *                        os_memoryPreAlloc()
 ****************************************************************************************
DESCRIPTION:    Gets system-space memory preallocated by kernel.

ARGUMENTS:      OsContext   - our adapter context.
                section     - section number
                Size        - Specifies the size, in bytes, to be allocated.

RETURN:         Pointer to the allocated memory.
                NULL if there is insufficient memory available.
*****************************************************************************************/
PVOID
os_memoryPreAlloc(
        TI_HANDLE OsContext,
        int section,
        UINT32 Size
        )
{
    struct os_mem_block *blk;
    __u32 total_size = Size + sizeof(struct os_mem_block) + sizeof(__u32);

#ifdef TI_MEM_ALLOC_TRACE
    os_printf("MTT:%s:%d ::os_memoryPreAlloc(0x%p, %lu) : %lu\n",__FUNCTION__, __LINE__,OsContext,Size,total_size);
#endif
    if( total_size < Size ) { /* Dm: Security fix */
        return NULL;
    }

    blk = (struct os_mem_block *)wifi_kernel_prealloc( section, total_size );
    if( !blk ) {
        return os_memoryAlloc(OsContext, Size);
    }
    blk->f_free = (os_free)os_memoryPreFree;

    os_profile (OsContext, 4, total_size);

    /*list_add(&blk->blk_list, &drv->mem_blocks);*/
    blk->size = Size;
    blk->signature = MEM_BLOCK_START;
    *(__u32 *)((unsigned char *)blk + total_size - sizeof(__u32)) = MEM_BLOCK_END;
    return (PVOID)((char *)blk + sizeof(struct os_mem_block));
}


/****************************************************************************************
 *                        os_memoryCAlloc()
 ****************************************************************************************
DESCRIPTION:    Allocates an array in memory with elements initialized to 0.

ARGUMENTS:		OsContext	-	our adapter context.
				Number		-	Number of elements
				Size		-	Length in bytes of each element

RETURN:			None

NOTES:         	
*****************************************************************************************/
PVOID
os_memoryCAlloc(
        TI_HANDLE OsContext,
        UINT32 Number,
        UINT32 Size
        )
{
    PVOID pAllocatedMem;
    ULONG MemSize;

#ifdef TI_MEM_ALLOC_TRACE
    os_printf("MTT:%s:%d ::os_memoryCAlloc(0x%p, %lu, %lu) : %lu\n",__FUNCTION__,__LINE__,OsContext,Number,Size,Number*Size);
#endif
    MemSize = Number * Size;

    if( (Number > 0) && (Size >= (0xFFFFFFFFUL / Number)) ) { /* Dm: Security fix */
        return NULL;
    }

    pAllocatedMem = os_memoryAlloc(OsContext, MemSize);

    if(!pAllocatedMem)
        return NULL;

    memset(pAllocatedMem,0,MemSize);

    return pAllocatedMem;
}

/****************************************************************************************
 *                        os_memoryFree()                                 
 ****************************************************************************************
DESCRIPTION:    This function releases a block of memory previously allocated with the
                os_memoryAlloc function.


ARGUMENTS:      OsContext   -   our adapter context.
                pMemPtr     -   Pointer to the base virtual address of the allocated memory.
                                This address was returned by the os_memoryAlloc function.
                Size        -   Specifies the size, in bytes, of the memory block to be released.
                                This parameter must be identical to the Length that was passed to
                                os_memoryAlloc.

RETURN:         None

NOTES:          
*****************************************************************************************/
VOID
os_memoryFree(
        TI_HANDLE OsContext,
        PVOID pMemPtr,
        UINT32 Size
        )
{
    struct os_mem_block *blk =
        (struct os_mem_block *)((char *)pMemPtr - sizeof(struct os_mem_block));
   
#ifdef TI_MEM_ALLOC_TRACE
    os_printf("MTT:%s:%d ::os_memoryFree(0x%p, 0x%p, %lu) : %d\n",__FUNCTION__,__LINE__,OsContext,pMemPtr,Size,-Size);
#endif
    if (blk->signature != MEM_BLOCK_START)
    {
        printk("\n\n%s: memory block signature is incorrect - 0x%x\n\n\n",
               __FUNCTION__, blk->signature);
        return;
    }
    *(char *)(&blk->signature) = '~';
    if (*(__u32 *)((unsigned char *)blk + blk->size + sizeof(struct os_mem_block))
        != MEM_BLOCK_END)
    {
        printk("\n\n%s: memory block corruption. Size=%u\n\n\n",
               __FUNCTION__, blk->size);
    }

    os_profile (OsContext, 5, blk->size + sizeof(struct os_mem_block) + sizeof(__u32));

    blk->f_free(blk);
}


/****************************************************************************************
 *                        os_memorySet()                                 
 ****************************************************************************************
DESCRIPTION:    This function fills a block of memory with given value.

ARGUMENTS:		OsContext	- our adapter context.
				pMemPtr		- Specifies the base address of a block of memory
				Value		- Specifies the value to set
				Length		- Specifies the size, in bytes, to copy.

RETURN:			None

NOTES:       
*****************************************************************************************/
VOID
os_memorySet(
    TI_HANDLE OsContext,
    PVOID pMemPtr,
    INT32 Value,
    UINT32 Length
    )
{
   memset(pMemPtr,Value,Length);
}

/****************************************************************************************
 *                        _os_memoryAlloc4HwDma()                                 
 ****************************************************************************************
DESCRIPTION:    Allocates resident (nonpaged) system-space memory for DMA operations.

ARGUMENTS:		OsContext	- our adapter context.
				Size		- Specifies the size, in bytes, to be allocated.

RETURN:			Pointer to the allocated memory.
				NULL if there is insufficient memory available.

NOTES:         		

*****************************************************************************************/
PVOID
os_memoryAlloc4HwDma(
    TI_HANDLE pOsContext,
    UINT32 Size
    )
{
    	struct os_mem_block *blk;
    	__u32 total_size = Size + sizeof(struct os_mem_block) + sizeof(__u32);
	/* 
		if the size is greater than 2 pages then we cant allocate the memory through kmalloc so the function fails
	*/
	if (Size < 2 * OS_PAGE_SIZE)
    	{
       	blk = kmalloc(total_size, GFP_ATOMIC);
        	if (!blk)
			return NULL;
       	blk->f_free = (os_free)kfree;
	}
    	else
    	{
		printk("\n\n%s: memory cant be allocated-Size = %d\n\n\n",
               __FUNCTION__, Size);
		return NULL;
    	}

	blk->size = Size;
    	blk->signature = MEM_BLOCK_START;
    	*(__u32 *)((unsigned char *)blk + total_size - sizeof(__u32)) = MEM_BLOCK_END;
    	return (PVOID)((char *)blk + sizeof(struct os_mem_block));
}

/****************************************************************************************
 *                        _os_memory4HwDmaFree()                                 
 ****************************************************************************************
DESCRIPTION:    This function releases a block of memory previously allocated with the
				_os_memoryAlloc4HwDma function.


ARGUMENTS:		OsContext	-	our adapter context.
				pMemPtr		-	Pointer to the base virtual address of the allocated memory.
								This address was returned by the os_memoryAlloc function.
				Size		-	Specifies the size, in bytes, of the memory block to be released.
								This parameter must be identical to the Length that was passed to
								os_memoryAlloc.

RETURN:			None

NOTES:         	
*****************************************************************************************/
void
os_memory4HwDmaFree(
    TI_HANDLE pOsContext,
    PVOID pMem_ptr,
    UINT32 Size
    )
{
    struct os_mem_block *blk =
        (struct os_mem_block *)((char *)pMem_ptr - sizeof(struct os_mem_block));

	if (blk->signature != MEM_BLOCK_START)
    {
		printk("\n\n%s: memory block signature is incorrect - 0x%x\n\n\n",
               __FUNCTION__, blk->signature);
        return;
    }
    *(char *)(&blk->signature) = '~';
    if (*(__u32 *)((unsigned char *)blk + blk->size + sizeof(struct os_mem_block))
        != MEM_BLOCK_END)
    {
		printk("\n\n%s: memory block corruption. Size=%u\n\n\n",
               __FUNCTION__, blk->size);
    }

    blk->f_free(blk);
}

/****************************************************************************************
 *                        os_memoryZero()                                 
 ****************************************************************************************
DESCRIPTION:    This function fills a block of memory with 0s.

ARGUMENTS:		OsContext	- our adapter context.
				pMemPtr		- Specifies the base address of a block of memory
				Length		- Specifies how many bytes to fill with 0s.

RETURN:			None

NOTES:  
*****************************************************************************************/
VOID
os_memoryZero(
    TI_HANDLE OsContext,
    PVOID pMemPtr,
    UINT32 Length
    )
{
   memset(pMemPtr,0,Length);
}


/****************************************************************************************
 *                        os_memoryCopy()                                 
 ****************************************************************************************
DESCRIPTION:    This function copies a specified number of bytes from one caller-supplied
				location to another.

ARGUMENTS:		OsContext	- our adapter context.
				pDstPtr		- Destination buffer
				pSrcPtr		- Source buffer
				Size		- Specifies the size, in bytes, to copy.

RETURN:			None

NOTES:       
*****************************************************************************************/
VOID
os_memoryCopy(
    TI_HANDLE OsContext,
    PVOID pDstPtr,
    PVOID pSrcPtr,
    UINT32 Size
    )
{
   memcpy(pDstPtr,pSrcPtr,Size);
}

/****************************************************************************************
 *                        os_memoryCompare()                                 
 ****************************************************************************************
DESCRIPTION:    Compare characters in two buffers.

ARGUMENTS:		OsContext	- our adapter context.
				Buf1		- First buffer
				Buf2		- Second buffer
				Count		- Number of characters

RETURN:			The return value indicates the relationship between the buffers:
                < 0 Buf1 less than Buf2
                0 Buf1 identical to Buf2
                > 0 Buf1 greater than Buf2

NOTES:         	
*****************************************************************************************/
INT32
os_memoryCompare(
        TI_HANDLE OsContext,
        PUINT8 Buf1,
        PUINT8 Buf2,
        INT32 Count
        )
{
   return memcmp(Buf1, Buf2, Count);
}
