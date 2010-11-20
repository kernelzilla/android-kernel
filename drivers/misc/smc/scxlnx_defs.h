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

#ifndef __SCXLNX_DEFS_H__
#define __SCXLNX_DEFS_H__

#include <asm/atomic.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/sysdev.h>
#include <linux/sysfs.h>
#include <linux/timer.h>

#include "scx_sm_protocol.h"

/*Cache optimization to flush only the concerned buffers instead of flushing the whole cache */
/* #define SMC_CACHE_OPTIM */

/*---------------------------------------------------------------------------- */

/*
 *Shared memory max allowed descriptors
 */
#define SCXLNX_SHMEM_MAX_DESCRIPTORS	(64)

/*
 *This typedef describes the possible types of shared memories
 *
 *SCXLNX_SHMEM_TYPE_DEVICE_CONTEXT :
 *The descriptor describes a device context shared memory
 *SCXLNX_SHMEM_TYPE_PREALLOC_REGISTERED_SHMEM :
 *The descriptor describes a registered shared memory.
 *Its coarse pages are preallocated when initilizing the
 *shared memory monitor
 *SCXLNX_SHMEM_TYPE_REGISTERED_SHMEM :
 *The descriptor describes a registered shared memory.
 *Its coarse pages are not preallocated
 */
typedef enum {
	SCXLNX_SHMEM_TYPE_DEVICE_CONTEXT = 0,
	SCXLNX_SHMEM_TYPE_PREALLOC_REGISTERED_SHMEM,
	SCXLNX_SHMEM_TYPE_REGISTERED_SHMEM,
} SCXLNX_SHMEM_TYPE;

/*
 *This typedef describes the possible types of shared memories
 *
 *SCXLNX_SHMEM_TYPE_DEVICE_CONTEXT :
 *The descriptor describes a device context shared memory
 *SCXLNX_SHMEM_TYPE_PREALLOC_REGISTERED_SHMEM :
 *The descriptor describes a registered shared memory.
 *Its coarse pages are preallocated when initilizing the
 *shared memory monitor
 *SCXLNX_SHMEM_TYPE_REGISTERED_SHMEM :
 *The descriptor describes a registered shared memory.
 *Its coarse pages are not preallocated
 */
typedef enum {
	SCXLNX_SHMEM_ALLOC_TYPE_NONE = 0,
	SCXLNX_SHMEM_ALLOC_TYPE_REGISTER,
	SCXLNX_SHMEM_ALLOC_TYPE_KMALLOC,
	SCXLNX_SHMEM_ALLOC_TYPE_PAGES,
} SCXLNX_SHMEM_ALLOC_TYPE;


/*
 *Fully describes a shared memory block (device context or not)
 */
typedef struct {
	/*
	 *Identifies the shared memory descriptor in the list of free shared memory descriptors
	 */
	struct list_head list;

	/*
	 *Identifies the type of shared memory descriptor
	 */
	SCXLNX_SHMEM_TYPE nType;

	/*
	 *The identifier of the block of shared memory, as returned by the smodule
	 *This identifier can be the hBlock field of a REGISTER_SHARED_MEMORY answer
	 *or the hDeviceContext field of a CREATE_DEVICE_CONTEXT answer
	 */
	u32 hIdentifier;

	/*
	 *Buffer description (aligned on a page boundary)
	 *See SCXLNX_SHMEM_ALLOC_TYPE_xxx
	 */
	SCXLNX_SHMEM_ALLOC_TYPE nAllocType;

	/*
	 *Client buffer allocated or registered
	 */
	u8 *pBuffer;

	/*
	 *Client buffer size
	 */
	u32 nBufferSize;

	/*
	 *SCXLNX_SHMEM_ALLOC_TYPE_REGISTER: Allocated pages
	 */
	struct page * *ppPages;

	/*
	 *SCXLNX_SHMEM_ALLOC_TYPE_REGISTER: Number of pages in ppPages
	 *SCXLNX_SHMEM_ALLOC_TYPE_KMALLOC: Number of bytes allocated
	 *SCXLNX_SHMEM_ALLOC_TYPE_PAGES: Number of bytes allocated
	 */
	u32 nAllocatedBufferSize;

	/*
	 *SCXLNX_SHMEM_ALLOC_TYPE_KMALLOC: Buffer allocated
	 *SCXLNX_SHMEM_ALLOC_TYPE_PAGES: Buffer allocated
	 */
	u8 *pAllocatedBuffer;

	/*
	 *SCXLNX_SHMEM_ALLOC_TYPE_PAGES: Buffer order
	 */
	u32 nBufferOrder;

	/*
	 *Physical buffer address
	 */
	u32 nBufferPhysAddr;

}
SCXLNX_SHMEM_DESC;


/*---------------------------------------------------------------------------- */
/* *
 *The magic word.
 */
#define SCX_PUBLIC_CRYPTO_DESC_MAGIC		0x45EF683C

/*
 *Fully describes a public crypto session.
 */
typedef struct {
	/*
	 *Identifies the public crypto session in the list of the sessions.
	 */
	struct list_head list;

	u32	nMagicNumber;		  /*Must be set to {SCX_PUBLIC_CRYPTO_DESC_MAGIC} */

	u32	nReserved1;
	u32	hClientSession;
	u32	nReserved2;
	u32	nCommandID;
	u32	nReserved3;

	u32	nCryptoAlgId;		  /*PUBLIC_CRYPTO_ALG_XXX */
	u32	nCryptoDataLength;	/*Size of the pCryptoData buffer in bytes */
	void *pCryptoData;			/*Algorithm-specific crypto data */

	u32	nDataLength;
	u8 *pDataBuffer;

	u32	nResultLength;
	u8 *pResultBuffer;

	u32	nResultActualLength;
	u8 *pResultBufferUser;

	u8 *pS2CDataBuffer;

	/*
	 *A pre-allocated buffer
	 */
	u32	nInnerBufferLength;
	u8 *pInnerBuffer;

} SCX_PUBLIC_CRYPTO_DESC;


/*---------------------------------------------------------------------------- */
/*
 *Monitors the public crypto sessions associated with a SM instance.
 */
typedef struct {
	/*
	 *This is the current public crypto session (initialized to NULL).
	 *This is set to NULL to indicate that the public crypto session
	 *has been corrupted, i.e. that the key must be re-installed in secure.
	 */
	SCX_PUBLIC_CRYPTO_DESC *pCurrentSession;

	/*
	 *Lock to protect concurrent access to the public crypto accelerators.
	 */
	struct semaphore HWALock;

	/*
	 *Number of opened session using the AES HW.
	 */
	u32	nAesSession;
	/*
	 *Number of opened session using the DES HW.
	 */
	u32	nDesSession;
	/*
	 *Number of opened session using the SHA HW.
	 */
	u32	nShaSession;

	/*
	 *Lists the public crypto sessions (SCX_PUBLIC_CRYPTO_DESC *).
	 */
	struct list_head sessions;

#ifdef SMODULE_SMC_OMAP3430_POWER_MANAGEMENT
	/*
	 *The timeout timer used to power off clocks
	 */
	struct timer_list pPowerManagementTimer;
#endif /*SMODULE_SMC_OMAP3430_POWER_MANAGEMENT */
}
SCX_PUBLIC_CRYPTO_MONITOR;


/*---------------------------------------------------------------------------- */

/*
 *Monitors the communication with a SM instance.
 */
typedef struct {
	/*
	 *The spin lock protecting concurrent access to the structure.
	 */
	spinlock_t lock;

	/*
	 *Bit vector with the following possible flags:
	 *- SCXLNX_SM_COMM_FLAG_POLLING_THREAD_STARTED: If set, indicates that
	 *the polling thread has been successfully started.
	 *- SCXLNX_SM_COMM_FLAG_TERMINATING: If set, indicates that the
	 *communication with the PA is being terminated. Transmissions
	 *to the PA are not permitted
	 *
	 *This bit vector must be accessed with the kernel's atomic bitwise
	 *operations.
	 */
	unsigned long nFlags;

	/*
	 *The virtual address of the SM communication buffer.
	 */
	SCHANNEL_C1S_BUFFER *pBuffer;

	/*
	 *The wait queue the polling thread is waiting on.
	 */
	wait_queue_head_t waitQueue;

	/*
	 *Used to synchronize processes while stopping the polling thread.
	 *usage : A parent process tells the polling thread to stop
	 *and waits for pollingThreadDeath to complete.
	 *The polling thread stops and completes the pollingThreadDeath,
	 *waking up the parent process
	 */
	struct completion pollingThreadDeath;

	/*
	 *The SE SDP can be initialized only once...
	 */
	bool bSDPInitialized;

	/*
	 *The virtual address of the L0 communication buffer.
	 */
	void *pL0SharedBuffer;

	/*
	 *Temporary buffers used to load and start the PA
	 */
	void *pPAInfo;
	void *pPABuffer;

	/*
	 *Monitors the public crypto sessions with the PA SMC.
	 */
	SCX_PUBLIC_CRYPTO_MONITOR pubcrypto;

	/*
	 *The address passed to SDP_RUNTIME_INIT : used for the power management cbk
	 */
	uint32_t nSDPBkExtStoreAddr;

	/*
	 *The value in ms used for the power management timer delay.
	 */
	uint32_t nInactivityTimerExpireMs;
}
SCXLNX_SM_COMM_MONITOR;


#define SCXLNX_SM_COMM_FLAG_POLLING_THREAD_STARTED  (1)
#define SCXLNX_SM_COMM_FLAG_TERMINATING				 (2)

/*---------------------------------------------------------------------------- */

typedef struct {
	struct kobject kobj;

	struct kobj_type kobj_type;

	struct attribute kobj_stat_attribute;

	struct attribute *kobj_attribute_list[2];

	struct sysfs_ops kobj_sysfs_operations;

	atomic_t stat_pages_allocated;
	atomic_t stat_memories_allocated;
	atomic_t stat_pages_locked;
}
SCXLNX_DEVICE_STATS;

/*
 *Device monitoring descriptor.
 *This is actually the driver private data
 *describing device specific information
 */
typedef struct {
	/*
	 *field describing driver version
	 */
	u8 sDriverDescription[SCX_DESCRIPTION_BUFFER_LENGTH];

	/*
	 *Bit vector with the following flags:
	 *- SCXLNX_DEVICE_FLAG_CDEV_INITIALIZED: If set, cdev has been initialized
	 *- SCXLNX_DEVICE_FLAG_SYSDEV_CLASS_REGISTERED: If set, sysdev_class has been registered
	 *- SCXLNX_DEVICE_FLAG_SYSDEV_REGISTERED: If set, sysdev has been registered
	 *- SCXLNX_DEVICE_FLAG_CDEV_REGISTERED: If set, cdev has been registered
	 *- SCXLNX_DEVICE_FLAG_CDEV_ADDED: If set, cdev has been added and the character device
	 *										  callbacks may be called by the kernel
	 *Use the atomic bit operations to manipulate these flags
	 */
	unsigned long nFlags;

	/*
	 *The device number for the device.
	 */
	dev_t nDevNum;

	/*
	 *Interfaces the system device with the kernel.
	 */
	struct sys_device sysdev;

	/*
	 *Interfaces the char device with the kernel.
	 */
	struct cdev cdev;

	/*
	 *Monitors the communication with the PA SMC.
	 */
	SCXLNX_SM_COMM_MONITOR sm;

	/*
	 *Lists the connections attached to this device.
	 *A connection is created each time a user space
	 *application "opens" a file descriptor on the driver
	 */
	struct list_head conns;

	/*
	 *The spin lock used to protect concurrent access to the connection list.
	 */
	spinlock_t connsLock;

	SCXLNX_DEVICE_STATS sDeviceStats;
}
SCXLNX_DEVICE_MONITOR;

/*the bits of the nFlags field of the SCXLNX_DEVICE_MONITOR structure */
#define SCXLNX_DEVICE_FLAG_CDEV_INITIALIZED				  (0)
#define SCXLNX_DEVICE_FLAG_SYSDEV_CLASS_REGISTERED		 (1)
#define SCXLNX_DEVICE_FLAG_SYSDEV_REGISTERED				 (2)
#define SCXLNX_DEVICE_FLAG_CDEV_REGISTERED					(3)
#define SCXLNX_DEVICE_FLAG_CDEV_ADDED						  (4)
#define SCXLNX_DEVICE_SYSFS_REGISTERED						 (5)

/*---------------------------------------------------------------------------- */

/*
 *Monitors shared memories allocated or registered by the driver.
 */
typedef struct {
	/*
	 *handle the device context descriptor separately
	 */
	SCXLNX_SHMEM_DESC *pDeviceContextDesc;

	/*
	 *Lists the used shared memory descriptors
	 */
	struct list_head sUsedSharedMemoryList;

	/*
	 *Lists the free shared memory descriptors
	 */
	struct list_head sFreeSharedMemoryList;

	/*
	 *A mutex to use to access this structure
	 */
	struct semaphore sharedMemoriesMutex;

	/*
	 *Counts the number of shared memories registered.
	 */
	atomic_t nShmemAllocated;

}
SCXLNX_SHMEM_MONITOR;


/*---------------------------------------------------------------------------- */
/*
 *This type describes a connection state.
 *This is used to determine whether a
 *message is valid or not.
 *
 *Messages are only valid in a certain device state.
 *Messages may be invalidated between
 *the start of the ioctl call and
 *the moment the message is sent to the SM.
 *
 *SCXLNX_CONN_STATE_NO_DEVICE_CONTEXT :
 *The connection has no DEVICE_CONTEXT created and no
 *CREATE_DEVICE_CONTEXT being processed by the SM
 *SCXLNX_CONN_STATE_CREATE_DEVICE_CONTEXT_SENT :
 *The connection has a CREATE_DEVICE_CONTEXT being processed by the SM
 *SCXLNX_CONN_STATE_VALID_DEVICE_CONTEXT :
 *The connection has a DEVICE_CONTEXT created and no
 *DESTROY_DEVICE_CONTEXT is being processed by the SM
 *SCXLNX_CONN_STATE_DESTROY_DEVICE_CONTEXT_SENT :
 *The connection has a DESTROY_DEVICE_CONTEXT being processed by the SM
 */
typedef enum {
	SCXLNX_CONN_STATE_NO_DEVICE_CONTEXT = 0,
	SCXLNX_CONN_STATE_CREATE_DEVICE_CONTEXT_SENT,
	SCXLNX_CONN_STATE_VALID_DEVICE_CONTEXT,
	SCXLNX_CONN_STATE_DESTROY_DEVICE_CONTEXT_SENT
} SCXLNX_CONN_STATE;


/*
 *Monitors a connection instance.
 *A connection is created each time an application
 *opens a file descriptor on the driver
 */
typedef struct {
	/*
	 *Identifies the connection in the list of the connections attached to the
	 *same device.
	 */
	struct list_head list;

	/*
	 *State of the connection.
	 */
	SCXLNX_CONN_STATE nState;

	/*
	 *A spinlock to use to access nState
	 */
	spinlock_t stateLock;

	/*
	 *A pointer to the monitor of the device the connection is attached to.
	 */
	SCXLNX_DEVICE_MONITOR *pDevice;

	/* *
	 *The monitor for the shared memory blocks
	 * */
	SCXLNX_SHMEM_MONITOR sSharedMemoryMonitor;

	/*
	 *Counts the number of operations currently pending on the connection.
	 *(for debug only)
	 */
	atomic_t nPendingOpCounter;

}
SCXLNX_CONN_MONITOR;

/*---------------------------------------------------------------------------- */

/*
 *Driver for SMC OMAP3430
 */
#define SMODULE_SMC_OMAP3430

/* *
 *The ASCII-C string representation of the base name of the devices managed by
 *the SModule driver.
 */
#ifdef SMODULE_SMC_CRYPTO_OMAP3430
#define SCXLNX_DEVICE_BASE_NAME  "smc-crypto"
#else
#define SCXLNX_DEVICE_BASE_NAME  "smodule"
#endif

/* *
 *The major and minor numbers of the registered character device driver.
 *Only 1 SModule exists, hence only 1 instance of the driver is supported.
 */
#define SCXLNX_DEVICE_MINOR_NUMBER  (0)


/*---------------------------------------------------------------------------- */
/*
 * Kernel Differences
 */

#if defined KERNEL_2_6_29
#define SET_PAGE_LOCKED __set_page_locked
#define CLEAR_PAGE_LOCKED __clear_page_locked

#define CURRENT_UID current_uid()
#define CURRENT_EUID current_euid()
#define CURRENT_EGID current_egid()
#elif defined KERNEL_2_6_27
#define SET_PAGE_LOCKED set_page_locked
#define CLEAR_PAGE_LOCKED clear_page_locked

#define CURRENT_UID current->uid
#define CURRENT_EUID current->euid
#define CURRENT_EGID current->egid
#else /* KERNEL_2_6_24 */
#define SET_PAGE_LOCKED SetPageLocked
#define CLEAR_PAGE_LOCKED ClearPageLocked

#define CURRENT_UID current->uid
#define CURRENT_EUID current->euid
#define CURRENT_EGID current->egid
#endif


#endif  /*!defined(__SCXLNX_DEFS_H__)*/
