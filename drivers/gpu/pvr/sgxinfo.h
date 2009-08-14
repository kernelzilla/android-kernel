/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

#if !defined (__SGXINFO_H__)
#define __SGXINFO_H__

#include "sgxscript.h"
#include "servicesint.h"
#include "services.h"
#include "sgxapi_km.h"

#if defined(SGX_FEATURE_MP)
	#define SGX_REG_BANK_SHIFT 			(12)
	#define SGX_REG_BANK_SIZE 			(0x4000)
	#if defined(SGX541)
		#define SGX_REG_BANK_BASE_INDEX		(1)
		#define	SGX_REG_BANK_MASTER_INDEX	(SGX_REG_BANK_BASE_INDEX + SGX_FEATURE_MP_CORE_COUNT)
	#else
		#define SGX_REG_BANK_BASE_INDEX		(2)
		#define	SGX_REG_BANK_MASTER_INDEX	(1)
	#endif 
	#define SGX_MP_CORE_SELECT(x,i) 	(x + ((i + SGX_REG_BANK_BASE_INDEX) * SGX_REG_BANK_SIZE))
	#define SGX_MP_MASTER_SELECT(x) 	(x + (SGX_REG_BANK_MASTER_INDEX * SGX_REG_BANK_SIZE))
#else
	#define SGX_MP_CORE_SELECT(x,i) 	(x)
#endif 

#define SGX_MAX_DEV_DATA			24
#define	SGX_MAX_INIT_MEM_HANDLES	16

#if defined(SGX_FEATURE_BIF_NUM_DIRLISTS)
#define SGX_BIF_DIR_LIST_INDEX_EDM	(SGX_FEATURE_BIF_NUM_DIRLISTS - 1)
#else
#define SGX_BIF_DIR_LIST_INDEX_EDM	(0)
#endif

typedef struct _SGX_BRIDGE_INFO_FOR_SRVINIT
{
	IMG_DEV_PHYADDR sPDDevPAddr;
	PVRSRV_HEAP_INFO asHeapInfo[PVRSRV_MAX_CLIENT_HEAPS];
} SGX_BRIDGE_INFO_FOR_SRVINIT;

typedef struct _SGX_MISCINFO_STRUCT_SIZES_
{
#if defined (SGX_FEATURE_2D_HARDWARE)
	IMG_UINT32	ui32Sizeof_2DCMD;
	IMG_UINT32	ui32Sizeof_2DCMD_SHARED;
#endif
	IMG_UINT32	ui32Sizeof_CMDTA;
	IMG_UINT32	ui32Sizeof_CMDTA_SHARED;
	IMG_UINT32	ui32Sizeof_TRANSFERCMD;
	IMG_UINT32	ui32Sizeof_TRANSFERCMD_SHARED;
	IMG_UINT32	ui32Sizeof_3DREGISTERS;
	IMG_UINT32	ui32Sizeof_HWPBDESC;
	IMG_UINT32	ui32Sizeof_HWRENDERCONTEXT;
	IMG_UINT32	ui32Sizeof_HWRENDERDETAILS;
	IMG_UINT32	ui32Sizeof_HWRTDATA;
	IMG_UINT32	ui32Sizeof_HWRTDATASET;
	IMG_UINT32	ui32Sizeof_HWTRANSFERCONTEXT;
	IMG_UINT32	ui32Sizeof_HOST_CTL;
	IMG_UINT32	ui32Sizeof_COMMAND;
} SGX_MISCINFO_STRUCT_SIZES;


typedef enum _SGXMKIF_CMD_TYPE_
{
	SGXMKIF_CMD_TA				= 0,
	SGXMKIF_CMD_TRANSFER		= 1,
	SGXMKIF_CMD_2D				= 2,
	SGXMKIF_CMD_POWER			= 3,
	SGXMKIF_CMD_CLEANUP			= 4,
	SGXMKIF_CMD_GETMISCINFO		= 5,
	SGXMKIF_CMD_PROCESS_QUEUES	= 6,
	SGXMKIF_CMD_MAX				= 7,

	SGXMKIF_CMD_FORCE_I32   	= -1,

} SGXMKIF_CMD_TYPE;


typedef struct _SGX_BRIDGE_INIT_INFO_
{
	IMG_HANDLE	hKernelCCBMemInfo;
	IMG_HANDLE	hKernelCCBCtlMemInfo;
	IMG_HANDLE	hKernelCCBEventKickerMemInfo;
	IMG_HANDLE	hKernelSGXHostCtlMemInfo;
	IMG_HANDLE	hKernelSGXTA3DCtlMemInfo;
	IMG_HANDLE	hKernelSGXMiscMemInfo;

	IMG_UINT32	aui32HostKickAddr[SGXMKIF_CMD_MAX];

	SGX_INIT_SCRIPTS sScripts;

	IMG_UINT32	ui32ClientBuildOptions;
	SGX_MISCINFO_STRUCT_SIZES	sSGXStructSizes;

#if defined(SGX_SUPPORT_HWPROFILING)
	IMG_HANDLE	hKernelHWProfilingMemInfo;
#endif
#if defined(SUPPORT_SGX_HWPERF)
	IMG_HANDLE	hKernelHWPerfCBMemInfo;
#endif
#if defined(PVRSRV_USSE_EDM_STATUS_DEBUG)
	IMG_HANDLE	hKernelEDMStatusBufferMemInfo;
#endif
#if defined(SGX_FEATURE_OVERLAPPED_SPM)
	IMG_HANDLE hKernelTmpRgnHeaderMemInfo;
#endif
#if defined(SGX_FEATURE_SPM_MODE_0)
	IMG_HANDLE hKernelTmpDPMStateMemInfo;
#endif

	IMG_UINT32 ui32EDMTaskReg0;
	IMG_UINT32 ui32EDMTaskReg1;

	IMG_UINT32 ui32ClkGateStatusReg;
	IMG_UINT32 ui32ClkGateStatusMask;
#if defined(SGX_FEATURE_MP)
	IMG_UINT32 ui32MasterClkGateStatusReg;
	IMG_UINT32 ui32MasterClkGateStatusMask;
#endif 

	IMG_UINT32 ui32CacheControl;

	IMG_UINT32	asInitDevData[SGX_MAX_DEV_DATA];
	IMG_HANDLE	asInitMemHandles[SGX_MAX_INIT_MEM_HANDLES];

} SGX_BRIDGE_INIT_INFO;

typedef struct _SGXMKIF_COMMAND_
{
	IMG_UINT32				ui32ServiceAddress;		
	IMG_UINT32				ui32CacheControl;		
	IMG_UINT32				ui32Data[2];			
} SGXMKIF_COMMAND;


typedef struct _PVRSRV_SGX_KERNEL_CCB_
{
	SGXMKIF_COMMAND		asCommands[256];		
} PVRSRV_SGX_KERNEL_CCB;


typedef struct _PVRSRV_SGX_CCB_CTL_
{
	IMG_UINT32				ui32WriteOffset;		
	IMG_UINT32				ui32ReadOffset;			
} PVRSRV_SGX_CCB_CTL;


#define SGX_AUXCCBFLAGS_SHARED					0x00000001


#define	PVRSRV_CLEANUPCMD_RT		0x1
#define	PVRSRV_CLEANUPCMD_RC		0x2
#define	PVRSRV_CLEANUPCMD_TC		0x3
#define	PVRSRV_CLEANUPCMD_2DC		0x4
#define	PVRSRV_CLEANUPCMD_PB		0x5

#define PVRSRV_POWERCMD_POWEROFF	0x1
#define PVRSRV_POWERCMD_IDLE		0x2
#define PVRSRV_POWERCMD_RESUME		0x3

#define	SGX_BIF_INVALIDATE_PTCACHE	0x1
#define	SGX_BIF_INVALIDATE_PDCACHE	0x2
#define SGX_BIF_INVALIDATE_SLCACHE	0x4

typedef struct _SGXMKIF_HWDEVICE_SYNC_LIST_
{
	IMG_DEV_VIRTADDR	sAccessDevAddr;
	IMG_UINT32			ui32NumSyncObjects;
	
	PVRSRV_DEVICE_SYNC_OBJECT	asSyncData[1];
} SGXMKIF_HWDEVICE_SYNC_LIST, *PSGXMKIF_HWDEVICE_SYNC_LIST;

typedef struct _SGX_DEVICE_SYNC_LIST_
{
	PSGXMKIF_HWDEVICE_SYNC_LIST	psHWDeviceSyncList;

	IMG_HANDLE				hKernelHWSyncListMemInfo;
	PVRSRV_CLIENT_MEM_INFO	*psHWDeviceSyncListClientMemInfo;
	PVRSRV_CLIENT_MEM_INFO	*psAccessResourceClientMemInfo;

	volatile IMG_UINT32		*pui32Lock;

	struct _SGX_DEVICE_SYNC_LIST_	*psNext;

	
	IMG_UINT32			ui32NumSyncObjects;
	IMG_HANDLE			ahSyncHandles[1];
} SGX_DEVICE_SYNC_LIST, *PSGX_DEVICE_SYNC_LIST;


typedef struct _SGX_INTERNEL_STATUS_UPDATE_
{
	CTL_STATUS				sCtlStatus;
	IMG_HANDLE				hKernelMemInfo;
	
	IMG_UINT32				ui32LastStatusUpdateDumpVal;
} SGX_INTERNEL_STATUS_UPDATE;


typedef struct _SGX_CCB_KICK_
{
	SGXMKIF_COMMAND		sCommand;
	IMG_HANDLE			hCCBKernelMemInfo;

	IMG_UINT32	ui32NumDstSyncObjects;
	IMG_HANDLE	hKernelHWSyncListMemInfo;

	
	IMG_HANDLE	*pahDstSyncHandles;

	IMG_UINT32	ui32NumTAStatusVals;
	IMG_UINT32	ui32Num3DStatusVals;

#if defined(SUPPORT_SGX_NEW_STATUS_VALS)
	SGX_INTERNEL_STATUS_UPDATE	asTAStatusUpdate[SGX_MAX_TA_STATUS_VALS];
	SGX_INTERNEL_STATUS_UPDATE	as3DStatusUpdate[SGX_MAX_3D_STATUS_VALS];
#else
	IMG_HANDLE	ahTAStatusSyncInfo[SGX_MAX_TA_STATUS_VALS];
	IMG_HANDLE	ah3DStatusSyncInfo[SGX_MAX_3D_STATUS_VALS];
#endif

	IMG_BOOL	bFirstKickOrResume;
#if (defined(NO_HARDWARE) || defined(PDUMP))
	IMG_BOOL	bTerminateOrAbort;
#endif
#if defined(SUPPORT_SGX_HWPERF)
	IMG_BOOL			bKickRender;
#endif

	
	IMG_UINT32	ui32CCBOffset;

#if defined(SUPPORT_SGX_GENERALISED_SYNCOBJECTS)
	
	IMG_UINT32	ui32NumTASrcSyncs;
	IMG_HANDLE	ahTASrcKernelSyncInfo[SGX_MAX_TA_SRC_SYNCS];
	IMG_UINT32	ui32NumTADstSyncs;
	IMG_HANDLE	ahTADstKernelSyncInfo[SGX_MAX_TA_DST_SYNCS];
	IMG_UINT32	ui32Num3DSrcSyncs;
	IMG_HANDLE	ah3DSrcKernelSyncInfo[SGX_MAX_3D_SRC_SYNCS];
#else
	
	IMG_UINT32	ui32NumSrcSyncs;
	IMG_HANDLE	ahSrcKernelSyncInfo[SGX_MAX_SRC_SYNCS];
#endif

	
	IMG_BOOL	bTADependency;
	IMG_HANDLE	hTA3DSyncInfo;

	IMG_HANDLE	hTASyncInfo;
	IMG_HANDLE	h3DSyncInfo;
#if defined(PDUMP)
	IMG_UINT32	ui32CCBDumpWOff;
#endif
#if defined(NO_HARDWARE)
	IMG_UINT32	ui32WriteOpsPendingVal;
#endif
} SGX_CCB_KICK;


#define SGX_KERNEL_USE_CODE_BASE_INDEX		15

typedef struct _SGXMKIF_HOST_CTL_
{
#if defined(PVRSRV_USSE_EDM_BREAKPOINTS)
	IMG_UINT32				ui32BreakpointDisable;
	IMG_UINT32				ui32Continue;
#endif

	volatile IMG_UINT32		ui32InitStatus; 
	volatile IMG_UINT32		ui32PowerStatus; 
	volatile IMG_UINT32		ui32CleanupStatus; 
#if defined(SUPPORT_HW_RECOVERY)
	IMG_UINT32				ui32uKernelDetectedLockups;		
	IMG_UINT32				ui32HostDetectedLockups;		
	IMG_UINT32				ui32HWRecoverySampleRate;		
#endif 
	IMG_UINT32				ui32uKernelTimerClock;			
	IMG_UINT32				ui32ActivePowManSampleRate;		
	IMG_UINT32				ui32InterruptFlags; 
	IMG_UINT32				ui32InterruptClearFlags; 


	IMG_UINT32				ui32NumActivePowerEvents;	

#if defined(SUPPORT_SGX_HWPERF)
	IMG_UINT32			ui32HWPerfFlags;		
#endif

#if defined(PVRSRV_USSE_EDM_STATUS_DEBUG)
	IMG_DEV_VIRTADDR		sEDMStatusBuffer;		
#endif

	
	IMG_UINT32			ui32TimeWraps;
} SGXMKIF_HOST_CTL;


typedef struct _SGX_CLIENT_INFO_
{
	IMG_UINT32					ui32ProcessID;			
	IMG_VOID					*pvProcess;				
	PVRSRV_MISC_INFO			sMiscInfo;				

	IMG_UINT32					asDevData[SGX_MAX_DEV_DATA];

} SGX_CLIENT_INFO;

typedef struct _SGX_INTERNAL_DEVINFO_
{
	IMG_UINT32			ui32Flags;
	IMG_HANDLE			hHostCtlKernelMemInfoHandle;
	IMG_BOOL			bForcePTOff;
} SGX_INTERNAL_DEVINFO;


#if defined(TRANSFER_QUEUE)
#define SGXTQ_MAX_STATUS						SGX_MAX_TRANSFER_STATUS_VALS + 2

#define SGXMKIF_TQFLAGS_NOSYNCUPDATE			0x00000001
#define SGXMKIF_TQFLAGS_KEEPPENDING				0x00000002
#define SGXMKIF_TQFLAGS_TATQ_SYNC				0x00000004
#define SGXMKIF_TQFLAGS_3DTQ_SYNC				0x00000008
#if defined(SGX_FEATURE_FAST_RENDER_CONTEXT_SWITCH)
#define SGXMKIF_TQFLAGS_CTXSWITCH				0x00000010
#endif
typedef struct _SGXMKIF_CMDTA_SHARED_
{
	IMG_UINT32			ui32NumTAStatusVals;
	IMG_UINT32			ui32Num3DStatusVals;

	
	IMG_UINT32			ui32TATQSyncWriteOpsPendingVal;
	IMG_DEV_VIRTADDR	sTATQSyncWriteOpsCompleteDevVAddr;
	IMG_UINT32			ui32TATQSyncReadOpsPendingVal;
	IMG_DEV_VIRTADDR	sTATQSyncReadOpsCompleteDevVAddr;

	
	IMG_UINT32			ui323DTQSyncWriteOpsPendingVal;
	IMG_DEV_VIRTADDR	s3DTQSyncWriteOpsCompleteDevVAddr;
	IMG_UINT32			ui323DTQSyncReadOpsPendingVal;
	IMG_DEV_VIRTADDR	s3DTQSyncReadOpsCompleteDevVAddr;


#if defined(SUPPORT_SGX_GENERALISED_SYNCOBJECTS)
	
	IMG_UINT32					ui32NumTASrcSyncs;
	PVRSRV_DEVICE_SYNC_OBJECT	asTASrcSyncs[SGX_MAX_TA_SRC_SYNCS];
	IMG_UINT32					ui32NumTADstSyncs;
	PVRSRV_DEVICE_SYNC_OBJECT	asTADstSyncs[SGX_MAX_TA_DST_SYNCS];
	IMG_UINT32					ui32Num3DSrcSyncs;
	PVRSRV_DEVICE_SYNC_OBJECT	as3DSrcSyncs[SGX_MAX_3D_SRC_SYNCS];
#else
	
	IMG_UINT32			ui32NumSrcSyncs;
	PVRSRV_DEVICE_SYNC_OBJECT	asSrcSyncs[SGX_MAX_SRC_SYNCS];
#endif

	CTL_STATUS			sCtlTAStatusInfo[SGX_MAX_TA_STATUS_VALS];
	CTL_STATUS			sCtl3DStatusInfo[SGX_MAX_3D_STATUS_VALS];
	
	PVRSRV_DEVICE_SYNC_OBJECT	sTA3DDependency;

} SGXMKIF_CMDTA_SHARED;

typedef struct _SGXMKIF_TRANSFERCMD_SHARED_
{
	
	
	IMG_UINT32		ui32SrcReadOpPendingVal;
	IMG_DEV_VIRTADDR	sSrcReadOpsCompleteDevAddr;
	
	IMG_UINT32		ui32SrcWriteOpPendingVal;
	IMG_DEV_VIRTADDR	sSrcWriteOpsCompleteDevAddr;

	
	
	IMG_UINT32		ui32DstReadOpPendingVal;
	IMG_DEV_VIRTADDR	sDstReadOpsCompleteDevAddr;
	
	IMG_UINT32		ui32DstWriteOpPendingVal;
	IMG_DEV_VIRTADDR	sDstWriteOpsCompleteDevAddr;

	
	IMG_UINT32		ui32TASyncWriteOpsPendingVal;
	IMG_DEV_VIRTADDR	sTASyncWriteOpsCompleteDevVAddr;
	IMG_UINT32		ui32TASyncReadOpsPendingVal;
	IMG_DEV_VIRTADDR	sTASyncReadOpsCompleteDevVAddr;

	
	IMG_UINT32		ui323DSyncWriteOpsPendingVal;
	IMG_DEV_VIRTADDR	s3DSyncWriteOpsCompleteDevVAddr;
	IMG_UINT32		ui323DSyncReadOpsPendingVal;
	IMG_DEV_VIRTADDR	s3DSyncReadOpsCompleteDevVAddr;

	IMG_UINT32 		ui32NumStatusVals;
	CTL_STATUS  	sCtlStatusInfo[SGXTQ_MAX_STATUS];
} SGXMKIF_TRANSFERCMD_SHARED, *PSGXMKIF_TRANSFERCMD_SHARED;

typedef struct _PVRSRV_TRANSFER_SGX_KICK_
{
	IMG_HANDLE		hCCBMemInfo;
	IMG_UINT32		ui32SharedCmdCCBOffset;

	IMG_DEV_VIRTADDR 	sHWTransferContextDevVAddr;

	IMG_HANDLE		hTASyncInfo;
	IMG_HANDLE		h3DSyncInfo;

	IMG_UINT32		ui32NumSrcSync;
	IMG_HANDLE		ahSrcSyncInfo[SGX_MAX_TRANSFER_SYNC_OPS];

	IMG_UINT32		ui32NumDstSync;
	IMG_HANDLE		ahDstSyncInfo[SGX_MAX_TRANSFER_SYNC_OPS];

	IMG_UINT32		ui32Flags;

	IMG_UINT32		ui32PDumpFlags;
#if defined(PDUMP)
	IMG_UINT32		ui32CCBDumpWOff;
#endif
} PVRSRV_TRANSFER_SGX_KICK, *PPVRSRV_TRANSFER_SGX_KICK;

#if defined(SGX_FEATURE_2D_HARDWARE)
typedef struct _SGXMKIF_2DCMD_SHARED_ {
	
	IMG_UINT32			ui32NumSrcSync;
	PVRSRV_DEVICE_SYNC_OBJECT	sSrcSyncData[SGX_MAX_2D_SRC_SYNC_OPS];

	
	PVRSRV_DEVICE_SYNC_OBJECT	sDstSyncData;

	
	PVRSRV_DEVICE_SYNC_OBJECT	sTASyncData;

	
	PVRSRV_DEVICE_SYNC_OBJECT	s3DSyncData;
} SGXMKIF_2DCMD_SHARED, *PSGXMKIF_2DCMD_SHARED;

typedef struct _PVRSRV_2D_SGX_KICK_
{
	IMG_HANDLE		hCCBMemInfo;
	IMG_UINT32		ui32SharedCmdCCBOffset;

	IMG_DEV_VIRTADDR 	sHW2DContextDevVAddr;

	IMG_UINT32		ui32NumSrcSync;
	IMG_HANDLE		ahSrcSyncInfo[SGX_MAX_2D_SRC_SYNC_OPS];

	
	IMG_HANDLE 		hDstSyncInfo;

	
	IMG_HANDLE		hTASyncInfo;

	
	IMG_HANDLE		h3DSyncInfo;

	IMG_UINT32		ui32PDumpFlags;
#if defined(PDUMP)
	IMG_UINT32		ui32CCBDumpWOff;
#endif
} PVRSRV_2D_SGX_KICK, *PPVRSRV_2D_SGX_KICK;
#endif	
#endif	

#define PVRSRV_SGX_DIFF_NUM_COUNTERS	9

typedef struct _PVRSRV_SGXDEV_DIFF_INFO_
{
	IMG_UINT32	aui32Counters[PVRSRV_SGX_DIFF_NUM_COUNTERS];
	IMG_UINT32	ui32Time[3];
	IMG_UINT32	ui32Marker[2];
} PVRSRV_SGXDEV_DIFF_INFO, *PPVRSRV_SGXDEV_DIFF_INFO;


#define SGXMKIF_HWPERF_CB_SIZE					0x100	

#if defined(SUPPORT_SGX_HWPERF)
typedef struct _SGXMKIF_HWPERF_CB_ENTRY_
{
	IMG_UINT32	ui32FrameNo;
	IMG_UINT32	ui32Type;
	IMG_UINT32	ui32Ordinal;
	IMG_UINT32	ui32TimeWraps;
	IMG_UINT32	ui32Time;
	IMG_UINT32	ui32Counters[PVRSRV_SGX_HWPERF_NUM_COUNTERS];
} SGXMKIF_HWPERF_CB_ENTRY;

typedef struct _SGXMKIF_HWPERF_CB_
{
	IMG_UINT32				ui32Woff;
	IMG_UINT32				ui32Roff;
	IMG_UINT32				ui32OrdinalGRAPHICS;
	IMG_UINT32				ui32OrdinalMK_EXECUTION;
	SGXMKIF_HWPERF_CB_ENTRY psHWPerfCBData[SGXMKIF_HWPERF_CB_SIZE];
} SGXMKIF_HWPERF_CB;
#endif 

typedef struct _PVRSRV_SGX_MISCINFO_INFO
{
	IMG_UINT32						ui32MiscInfoFlags;
	PVRSRV_SGX_MISCINFO_FEATURES	sSGXFeatures;
	SGX_MISCINFO_STRUCT_SIZES		sSGXStructSizes;	
} PVRSRV_SGX_MISCINFO_INFO;

#endif 
