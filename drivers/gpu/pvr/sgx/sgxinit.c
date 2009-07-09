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

#include <stddef.h>

#include "sgxdefs.h"
#include "sgxmmu.h"
#include "services_headers.h"
#include "buffer_manager.h"
#include "sgxapi_km.h"
#include "sgxinfo.h"
#include "sgxinfokm.h"
#include "sgxconfig.h"
#include "sysconfig.h"
#include "pvr_bridge_km.h"

#include "pdump_km.h"
#include "ra.h"
#include "mmu.h"
#include "handle.h"
#include "perproc.h"

#include "sgxutils.h"
#include "pvrversion.h"
#include "sgx_options.h"

IMG_BOOL SGX_ISRHandler(IMG_VOID *pvData);

IMG_UINT32 gui32EventStatusServicesByISR = 0;



static
PVRSRV_ERROR SGXGetBuildInfoKM(PVRSRV_SGXDEV_INFO	*psDevInfo,
							  PVRSRV_DEVICE_NODE 	*psDeviceNode);


static IMG_VOID SGXCommandComplete(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	SGXScheduleProcessQueuesKM(psDeviceNode);
}

static IMG_UINT32 DeinitDevInfo(PVRSRV_SGXDEV_INFO *psDevInfo)
{
	if (psDevInfo->psKernelCCBInfo != IMG_NULL)
	{
		

		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP, sizeof(PVRSRV_SGX_CCB_INFO), psDevInfo->psKernelCCBInfo, IMG_NULL);
	}

	return PVRSRV_OK;
}

static PVRSRV_ERROR InitDevInfo(PVRSRV_PER_PROCESS_DATA *psPerProc,
								PVRSRV_DEVICE_NODE *psDeviceNode,
								SGX_BRIDGE_INIT_INFO *psInitInfo)
{
	PVRSRV_SGXDEV_INFO *psDevInfo = (PVRSRV_SGXDEV_INFO *)psDeviceNode->pvDevice;
	PVRSRV_ERROR		eError;

	PVRSRV_SGX_CCB_INFO	*psKernelCCBInfo = IMG_NULL;

	PVR_UNREFERENCED_PARAMETER(psPerProc);
	psDevInfo->sScripts = psInitInfo->sScripts;

	psDevInfo->psKernelCCBMemInfo = (PVRSRV_KERNEL_MEM_INFO *)psInitInfo->hKernelCCBMemInfo;
	psDevInfo->psKernelCCB = (PVRSRV_SGX_KERNEL_CCB *) psDevInfo->psKernelCCBMemInfo->pvLinAddrKM;

	psDevInfo->psKernelCCBCtlMemInfo = (PVRSRV_KERNEL_MEM_INFO *)psInitInfo->hKernelCCBCtlMemInfo;
	psDevInfo->psKernelCCBCtl = (PVRSRV_SGX_CCB_CTL *) psDevInfo->psKernelCCBCtlMemInfo->pvLinAddrKM;

	psDevInfo->psKernelCCBEventKickerMemInfo = (PVRSRV_KERNEL_MEM_INFO *)psInitInfo->hKernelCCBEventKickerMemInfo;
	psDevInfo->pui32KernelCCBEventKicker = (IMG_UINT32 *)psDevInfo->psKernelCCBEventKickerMemInfo->pvLinAddrKM;

	psDevInfo->psKernelSGXHostCtlMemInfo = (PVRSRV_KERNEL_MEM_INFO *)psInitInfo->hKernelSGXHostCtlMemInfo;
	psDevInfo->psSGXHostCtl = (SGXMKIF_HOST_CTL *)psDevInfo->psKernelSGXHostCtlMemInfo->pvLinAddrKM;

	psDevInfo->psKernelSGXTA3DCtlMemInfo = (PVRSRV_KERNEL_MEM_INFO *)psInitInfo->hKernelSGXTA3DCtlMemInfo;

 	psDevInfo->psKernelSGXMiscMemInfo = (PVRSRV_KERNEL_MEM_INFO *)psInitInfo->hKernelSGXMiscMemInfo;

#if defined(SGX_SUPPORT_HWPROFILING)
	psDevInfo->psKernelHWProfilingMemInfo = (PVRSRV_KERNEL_MEM_INFO *)psInitInfo->hKernelHWProfilingMemInfo;
#endif
#if defined(SUPPORT_SGX_HWPERF)
	psDevInfo->psKernelHWPerfCBMemInfo = (PVRSRV_KERNEL_MEM_INFO *)psInitInfo->hKernelHWPerfCBMemInfo;
#endif
#if defined(SGX_FEATURE_OVERLAPPED_SPM)
	psDevInfo->psKernelTmpRgnHeaderMemInfo = (PVRSRV_KERNEL_MEM_INFO *)psInitInfo->hKernelTmpRgnHeaderMemInfo;
#endif
#if defined(SGX_FEATURE_SPM_MODE_0)
	psDevInfo->psKernelTmpDPMStateMemInfo = (PVRSRV_KERNEL_MEM_INFO *)psInitInfo->hKernelTmpDPMStateMemInfo;
#endif
	

	eError = OSAllocMem(PVRSRV_OS_PAGEABLE_HEAP,
						sizeof(PVRSRV_SGX_CCB_INFO),
						(IMG_VOID **)&psKernelCCBInfo, 0);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"InitDevInfo: Failed to alloc memory"));
		goto failed_allockernelccb;
	}


	OSMemSet(psKernelCCBInfo, 0, sizeof(PVRSRV_SGX_CCB_INFO));
	psKernelCCBInfo->psCCBMemInfo		= psDevInfo->psKernelCCBMemInfo;
	psKernelCCBInfo->psCCBCtlMemInfo	= psDevInfo->psKernelCCBCtlMemInfo;
	psKernelCCBInfo->psCommands			= psDevInfo->psKernelCCB->asCommands;
	psKernelCCBInfo->pui32WriteOffset	= &psDevInfo->psKernelCCBCtl->ui32WriteOffset;
	psKernelCCBInfo->pui32ReadOffset	= &psDevInfo->psKernelCCBCtl->ui32ReadOffset;
	psDevInfo->psKernelCCBInfo = psKernelCCBInfo;

	

	psDevInfo->ui32HostKickAddress = psInitInfo->ui32HostKickAddress;

 	
 	psDevInfo->ui32GetMiscInfoAddress = psInitInfo->ui32GetMiscInfoAddress;

 	psDevInfo->bForcePTOff = IMG_FALSE;

	psDevInfo->ui32CacheControl = psInitInfo->ui32CacheControl;

	psDevInfo->ui32EDMTaskReg0 = psInitInfo->ui32EDMTaskReg0;
	psDevInfo->ui32EDMTaskReg1 = psInitInfo->ui32EDMTaskReg1;
	psDevInfo->ui32ClkGateStatusReg = psInitInfo->ui32ClkGateStatusReg;
	psDevInfo->ui32ClkGateStatusMask = psInitInfo->ui32ClkGateStatusMask;
#if defined(SGX_FEATURE_MP)
	psDevInfo->ui32MasterClkGateStatusReg = psInitInfo->ui32MasterClkGateStatusReg;
	psDevInfo->ui32MasterClkGateStatusMask = psInitInfo->ui32MasterClkGateStatusMask;
#endif 


	
	OSMemCopy(&psDevInfo->asSGXDevData,  &psInitInfo->asInitDevData, sizeof(psDevInfo->asSGXDevData));

	return PVRSRV_OK;

failed_allockernelccb:
	DeinitDevInfo(psDevInfo);

	return eError;
}




static PVRSRV_ERROR SGXRunScript(PVRSRV_SGXDEV_INFO *psDevInfo, SGX_INIT_COMMAND *psScript, IMG_UINT32 ui32NumInitCommands)
{
	IMG_UINT32 ui32PC;
	SGX_INIT_COMMAND *psComm;

	for (ui32PC = 0, psComm = psScript;
		ui32PC < ui32NumInitCommands;
		ui32PC++, psComm++)
	{
		switch (psComm->eOp)
		{
			case SGX_INIT_OP_WRITE_HW_REG:
			{
				OSWriteHWReg(psDevInfo->pvRegsBaseKM, psComm->sWriteHWReg.ui32Offset, psComm->sWriteHWReg.ui32Value);
				PDUMPREG(psComm->sWriteHWReg.ui32Offset, psComm->sWriteHWReg.ui32Value);
				break;
			}
#if defined(PDUMP)
			case SGX_INIT_OP_PDUMP_HW_REG:
			{
				PDUMPREG(psComm->sPDumpHWReg.ui32Offset, psComm->sPDumpHWReg.ui32Value);
				break;
			}
#endif
			case SGX_INIT_OP_HALT:
			{
				return PVRSRV_OK;
			}
			case SGX_INIT_OP_ILLEGAL:
			
			default:
			{
				PVR_DPF((PVR_DBG_ERROR,"SGXRunScript: PC %d: Illegal command: %d", ui32PC, psComm->eOp));
				return PVRSRV_ERROR_GENERIC;
			}
		}

	}

	return PVRSRV_ERROR_GENERIC;
}

PVRSRV_ERROR SGXInitialise(PVRSRV_SGXDEV_INFO	*psDevInfo,
						   IMG_BOOL				bHardwareRecovery)
{
	PVRSRV_ERROR		eError;

	

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "SGX initialisation script part 1\n");
	eError = SGXRunScript(psDevInfo, psDevInfo->sScripts.asInitCommandsPart1, SGX_MAX_INIT_COMMANDS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"SGXInitialise: SGXRunScript (part 1) failed (%d)", eError));
		return (PVRSRV_ERROR_GENERIC);
	}
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "End of SGX initialisation script part 1\n");

	
	SGXReset(psDevInfo, PDUMP_FLAGS_CONTINUOUS);

#if defined(EUR_CR_POWER)
#if defined(SGX531)
	




	OSWriteHWReg(psDevInfo->pvRegsBaseKM, EUR_CR_POWER, 1);
	PDUMPREG(EUR_CR_POWER, 1);
#else
	
	OSWriteHWReg(psDevInfo->pvRegsBaseKM, EUR_CR_POWER, 0);
	PDUMPREG(EUR_CR_POWER, 0);
#endif
#endif

	
	*psDevInfo->pui32KernelCCBEventKicker = 0;
#if defined(PDUMP)
	PDUMPMEM(IMG_NULL, psDevInfo->psKernelCCBEventKickerMemInfo, 0,
			 sizeof(*psDevInfo->pui32KernelCCBEventKicker), PDUMP_FLAGS_CONTINUOUS,
			 MAKEUNIQUETAG(psDevInfo->psKernelCCBEventKickerMemInfo));
#endif 

	

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "SGX initialisation script part 2\n");
	eError = SGXRunScript(psDevInfo, psDevInfo->sScripts.asInitCommandsPart2, SGX_MAX_INIT_COMMANDS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"SGXInitialise: SGXRunScript (part 2) failed (%d)", eError));
		return (PVRSRV_ERROR_GENERIC);
	}
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "End of SGX initialisation script part 2\n");

	SGXStartTimer(psDevInfo, (IMG_BOOL)!bHardwareRecovery);

	if (bHardwareRecovery)
	{
		SGXMKIF_HOST_CTL	*psSGXHostCtl = (SGXMKIF_HOST_CTL *)psDevInfo->psSGXHostCtl;

		
		if (PollForValueKM((volatile IMG_UINT32 *)(&psSGXHostCtl->ui32InterruptClearFlags),
						   0,
						   PVRSRV_USSE_EDM_INTERRUPT_HWR,
						   MAX_HW_TIME_US/WAIT_TRY_COUNT,
						   1000) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "SGXInitialise: Wait for uKernel HW Recovery failed"));
			PVR_DBG_BREAK;
			return PVRSRV_ERROR_RETRY;
		}
	}


	PVR_ASSERT(psDevInfo->psKernelCCBCtl->ui32ReadOffset == psDevInfo->psKernelCCBCtl->ui32WriteOffset);

	return PVRSRV_OK;
}

PVRSRV_ERROR SGXDeinitialise(IMG_HANDLE hDevCookie)

{
	PVRSRV_SGXDEV_INFO	*psDevInfo = (PVRSRV_SGXDEV_INFO *) hDevCookie;
	PVRSRV_ERROR		eError;

	
	if (psDevInfo->pvRegsBaseKM == IMG_NULL)
	{
		return PVRSRV_OK;
	}

	eError = SGXRunScript(psDevInfo, psDevInfo->sScripts.asDeinitCommands, SGX_MAX_DEINIT_COMMANDS);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"SGXDeinitialise: SGXRunScript failed (%d)", eError));
		return (PVRSRV_ERROR_GENERIC);
	}

	return PVRSRV_OK;
}


static PVRSRV_ERROR DevInitSGXPart1 (IMG_VOID *pvDeviceNode)
{
	PVRSRV_SGXDEV_INFO	*psDevInfo;
	IMG_HANDLE		hKernelDevMemContext;
	IMG_DEV_PHYADDR		sPDDevPAddr;
	IMG_UINT32		i;
	PVRSRV_DEVICE_NODE  *psDeviceNode = (PVRSRV_DEVICE_NODE *)pvDeviceNode;
	DEVICE_MEMORY_HEAP_INFO *psDeviceMemoryHeap = psDeviceNode->sDevMemoryInfo.psDeviceMemoryHeap;
	PVRSRV_ERROR		eError;

	PDUMPCOMMENT("SGX Initialisation Part 1");

	
	PDUMPCOMMENT("SGX Core Version Information: %s", SGX_CORE_FRIENDLY_NAME);
#ifdef SGX_CORE_REV
	PDUMPCOMMENT("SGX Core Revision Information: %d", SGX_CORE_REV);
#else
	PDUMPCOMMENT("SGX Core Revision Information: head rtl");
#endif

	

	if(OSAllocMem( PVRSRV_OS_NON_PAGEABLE_HEAP,
					 sizeof(PVRSRV_SGXDEV_INFO),
					 (IMG_VOID **)&psDevInfo, IMG_NULL) != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"DevInitSGXPart1 : Failed to alloc memory for DevInfo"));
		return (PVRSRV_ERROR_OUT_OF_MEMORY);
	}
	OSMemSet (psDevInfo, 0, sizeof(PVRSRV_SGXDEV_INFO));

	
	psDevInfo->eDeviceType 		= DEV_DEVICE_TYPE;
	psDevInfo->eDeviceClass 	= DEV_DEVICE_CLASS;

	
	psDeviceNode->pvDevice = (IMG_PVOID)psDevInfo;

	
	psDevInfo->pvDeviceMemoryHeap = (IMG_VOID*)psDeviceMemoryHeap;

	
	hKernelDevMemContext = BM_CreateContext(psDeviceNode,
											&sPDDevPAddr,
											IMG_NULL,
											IMG_NULL);

	psDevInfo->sKernelPDDevPAddr = sPDDevPAddr;


	
	for(i=0; i<psDeviceNode->sDevMemoryInfo.ui32HeapCount; i++)
	{
		IMG_HANDLE hDevMemHeap;

		switch(psDeviceMemoryHeap[i].DevMemHeapType)
		{
			case DEVICE_MEMORY_HEAP_KERNEL:
			case DEVICE_MEMORY_HEAP_SHARED:
			case DEVICE_MEMORY_HEAP_SHARED_EXPORTED:
			{
				hDevMemHeap = BM_CreateHeap (hKernelDevMemContext,
												&psDeviceMemoryHeap[i]);
				


				psDeviceMemoryHeap[i].hDevMemHeap = hDevMemHeap;
				break;
			}
		}
	}

	eError = MMU_BIFResetPDAlloc(psDevInfo);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"DevInitSGX : Failed to alloc memory for BIF reset"));
		return PVRSRV_ERROR_GENERIC;
	}

	return PVRSRV_OK;
}

IMG_EXPORT
PVRSRV_ERROR SGXGetInfoForSrvinitKM(IMG_HANDLE hDevHandle, SGX_BRIDGE_INFO_FOR_SRVINIT *psInitInfo)
{
	PVRSRV_DEVICE_NODE	*psDeviceNode;
	PVRSRV_SGXDEV_INFO	*psDevInfo;
	PVRSRV_ERROR		eError;

	PDUMPCOMMENT("SGXGetInfoForSrvinit");

	psDeviceNode = (PVRSRV_DEVICE_NODE *)hDevHandle;
	psDevInfo = (PVRSRV_SGXDEV_INFO *)psDeviceNode->pvDevice;

	psInitInfo->sPDDevPAddr = psDevInfo->sKernelPDDevPAddr;

	eError = PVRSRVGetDeviceMemHeapsKM(hDevHandle, &psInitInfo->asHeapInfo[0]);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"SGXGetInfoForSrvinit: PVRSRVGetDeviceMemHeapsKM failed (%d)", eError));
		return PVRSRV_ERROR_GENERIC;
	}

	return eError;
}

IMG_EXPORT
PVRSRV_ERROR DevInitSGXPart2KM (PVRSRV_PER_PROCESS_DATA *psPerProc,
                                IMG_HANDLE hDevHandle,
                                SGX_BRIDGE_INIT_INFO *psInitInfo)
{
	PVRSRV_DEVICE_NODE	*psDeviceNode;
	PVRSRV_SGXDEV_INFO	*psDevInfo;
	PVRSRV_ERROR		eError;
	SGX_DEVICE_MAP		*psSGXDeviceMap;
	PVR_POWER_STATE		eDefaultPowerState;

	PDUMPCOMMENT("SGX Initialisation Part 2");

	psDeviceNode = (PVRSRV_DEVICE_NODE *)hDevHandle;
	psDevInfo = (PVRSRV_SGXDEV_INFO *)psDeviceNode->pvDevice;

	

	eError = InitDevInfo(psPerProc, psDeviceNode, psInitInfo);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"DevInitSGXPart2KM: Failed to load EDM program"));
		goto failed_init_dev_info;
	}


	eError = SysGetDeviceMemoryMap(PVRSRV_DEVICE_TYPE_SGX,
									(IMG_VOID**)&psSGXDeviceMap);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"DevInitSGXPart2KM: Failed to get device memory map!"));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

	
	if (psSGXDeviceMap->pvRegsCpuVBase)
	{
		psDevInfo->pvRegsBaseKM = psSGXDeviceMap->pvRegsCpuVBase;
	}
	else
	{
		
		psDevInfo->pvRegsBaseKM = OSMapPhysToLin(psSGXDeviceMap->sRegsCpuPBase,
											   psSGXDeviceMap->ui32RegsSize,
											   PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
											   IMG_NULL);
		if (!psDevInfo->pvRegsBaseKM)
		{
			PVR_DPF((PVR_DBG_ERROR,"DevInitSGXPart2KM: Failed to map in regs\n"));
			return PVRSRV_ERROR_BAD_MAPPING;
		}
	}
	psDevInfo->ui32RegSize = psSGXDeviceMap->ui32RegsSize;
	psDevInfo->sRegsPhysBase = psSGXDeviceMap->sRegsSysPBase;



#if defined (SYS_USING_INTERRUPTS)

	
	psDeviceNode->pvISRData = psDeviceNode;
	
	PVR_ASSERT(psDeviceNode->pfnDeviceISR == SGX_ISRHandler);

#endif 

	
#if defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
	
	psDevInfo->psSGXHostCtl->ui32PowerStatus |= PVRSRV_USSE_EDM_POWMAN_NO_WORK;
	eDefaultPowerState = PVRSRV_POWER_STATE_D3;
#else
	eDefaultPowerState = PVRSRV_POWER_STATE_D0;
#endif 
	eError = PVRSRVRegisterPowerDevice (psDeviceNode->sDevId.ui32DeviceIndex,
										SGXPrePowerStateExt, SGXPostPowerStateExt,
										SGXPreClockSpeedChange, SGXPostClockSpeedChange,
										(IMG_HANDLE)psDeviceNode,
										PVRSRV_POWER_STATE_D3,
										eDefaultPowerState);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"DevInitSGXPart2KM: failed to register device with power manager"));
		return eError;
	}



	

	OSMemSet(psDevInfo->psKernelCCB, 0, sizeof(PVRSRV_SGX_KERNEL_CCB));
	OSMemSet(psDevInfo->psKernelCCBCtl, 0, sizeof(PVRSRV_SGX_CCB_CTL));
	OSMemSet(psDevInfo->pui32KernelCCBEventKicker, 0, sizeof(*psDevInfo->pui32KernelCCBEventKicker));
	PDUMPCOMMENT("Initialise Kernel CCB");
	PDUMPMEM(IMG_NULL, psDevInfo->psKernelCCBMemInfo, 0, sizeof(PVRSRV_SGX_KERNEL_CCB), PDUMP_FLAGS_CONTINUOUS, MAKEUNIQUETAG(psDevInfo->psKernelCCBMemInfo));
	PDUMPCOMMENT("Initialise Kernel CCB Control");
	PDUMPMEM(IMG_NULL, psDevInfo->psKernelCCBCtlMemInfo, 0, sizeof(PVRSRV_SGX_CCB_CTL), PDUMP_FLAGS_CONTINUOUS, MAKEUNIQUETAG(psDevInfo->psKernelCCBCtlMemInfo));
	PDUMPCOMMENT("Initialise Kernel CCB Event Kicker");
	PDUMPMEM(IMG_NULL, psDevInfo->psKernelCCBEventKickerMemInfo, 0, sizeof(*psDevInfo->pui32KernelCCBEventKicker), PDUMP_FLAGS_CONTINUOUS, MAKEUNIQUETAG(psDevInfo->psKernelCCBEventKickerMemInfo));

	return PVRSRV_OK;

failed_init_dev_info:
	return eError;
}

static PVRSRV_ERROR DevDeInitSGX (IMG_VOID *pvDeviceNode)
{
	PVRSRV_DEVICE_NODE			*psDeviceNode = (PVRSRV_DEVICE_NODE *)pvDeviceNode;
	PVRSRV_SGXDEV_INFO			*psDevInfo = (PVRSRV_SGXDEV_INFO*)psDeviceNode->pvDevice;
	PVRSRV_ERROR				eError;
	IMG_UINT32					ui32Heap;
	DEVICE_MEMORY_HEAP_INFO		*psDeviceMemoryHeap;
	SGX_DEVICE_MAP				*psSGXDeviceMap;

	if (!psDevInfo)
	{
		
		PVR_DPF((PVR_DBG_ERROR,"DevDeInitSGX: Null DevInfo"));
		return PVRSRV_OK;
	}

#if defined(SUPPORT_HW_RECOVERY)
	if (psDevInfo->hTimer)
	{
		eError = OSRemoveTimer(psDevInfo->hTimer);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"DevDeInitSGX: Failed to remove timer"));
			return 	eError;
		}
		psDevInfo->hTimer = IMG_NULL;
	}
#endif 


	MMU_BIFResetPDFree(psDevInfo);


	

	DeinitDevInfo(psDevInfo);

	
	psDeviceMemoryHeap = (DEVICE_MEMORY_HEAP_INFO *)psDevInfo->pvDeviceMemoryHeap;
	for(ui32Heap=0; ui32Heap<psDeviceNode->sDevMemoryInfo.ui32HeapCount; ui32Heap++)
	{
		switch(psDeviceMemoryHeap[ui32Heap].DevMemHeapType)
		{
			case DEVICE_MEMORY_HEAP_KERNEL:
			case DEVICE_MEMORY_HEAP_SHARED:
			case DEVICE_MEMORY_HEAP_SHARED_EXPORTED:
			{
				if (psDeviceMemoryHeap[ui32Heap].hDevMemHeap != IMG_NULL)
				{
					BM_DestroyHeap(psDeviceMemoryHeap[ui32Heap].hDevMemHeap);
				}
				break;
			}
		}
	}

	
	eError = BM_DestroyContext(psDeviceNode->sDevMemoryInfo.pBMKernelContext, IMG_NULL);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"DevDeInitSGX : Failed to destroy kernel context"));
		return eError;
	}

	
	eError = PVRSRVRemovePowerDevice (((PVRSRV_DEVICE_NODE*)pvDeviceNode)->sDevId.ui32DeviceIndex);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	eError = SysGetDeviceMemoryMap(PVRSRV_DEVICE_TYPE_SGX,
									(IMG_VOID**)&psSGXDeviceMap);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"DevDeInitSGX: Failed to get device memory map!"));
		return eError;
	}

	
	if (!psSGXDeviceMap->pvRegsCpuVBase)
	{
		
		if (psDevInfo->pvRegsBaseKM != IMG_NULL)
		{
			OSUnMapPhysToLin(psDevInfo->pvRegsBaseKM,
							 psDevInfo->ui32RegSize,
							 PVRSRV_HAP_KERNEL_ONLY|PVRSRV_HAP_UNCACHED,
							 IMG_NULL);
		}
	}



	
	OSFreeMem(PVRSRV_OS_NON_PAGEABLE_HEAP,
				sizeof(PVRSRV_SGXDEV_INFO),
				psDevInfo,
				0);

	psDeviceNode->pvDevice = IMG_NULL;

	if (psDeviceMemoryHeap != IMG_NULL)
	{
	
		OSFreeMem(PVRSRV_OS_NON_PAGEABLE_HEAP,
				sizeof(DEVICE_MEMORY_HEAP_INFO) * psDeviceNode->sDevMemoryInfo.ui32HeapCount,
				psDeviceMemoryHeap,
				0);
	}

	return PVRSRV_OK;
}




#if defined(SYS_USING_INTERRUPTS) || defined(SUPPORT_HW_RECOVERY)
static
IMG_VOID HWRecoveryResetSGX (PVRSRV_DEVICE_NODE *psDeviceNode,
									IMG_UINT32 			ui32Component,
									IMG_UINT32			ui32CallerID)
{
	PVRSRV_ERROR		eError;
	PVRSRV_SGXDEV_INFO	*psDevInfo = (PVRSRV_SGXDEV_INFO*)psDeviceNode->pvDevice;
	SGXMKIF_HOST_CTL	*psSGXHostCtl = (SGXMKIF_HOST_CTL *)psDevInfo->psSGXHostCtl;

	PVR_UNREFERENCED_PARAMETER(ui32Component);

	

	eError = PVRSRVPowerLock(ui32CallerID, IMG_FALSE);
	if(eError != PVRSRV_OK)
	{
		


		PVR_DPF((PVR_DBG_WARNING,"HWRecoveryResetSGX: Power transition in progress"));
		return;
	}

	psSGXHostCtl->ui32InterruptClearFlags |= PVRSRV_USSE_EDM_INTERRUPT_HWR;

	PVR_DPF((PVR_DBG_ERROR, "HWRecoveryResetSGX: SGX Hardware Recovery triggered"));


	
	PDUMPSUSPEND();

	
	do
	{
		eError = SGXInitialise(psDevInfo, IMG_TRUE);
	}
	while (eError == PVRSRV_ERROR_RETRY);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"HWRecoveryResetSGX: SGXInitialise failed (%d)", eError));
	}

	
	PDUMPRESUME();

	PVRSRVPowerUnlock(ui32CallerID);

	
	SGXScheduleProcessQueuesKM(psDeviceNode);

	
	
	PVRSRVProcessQueues(ui32CallerID, IMG_TRUE);
}
#endif 


#if defined(SUPPORT_HW_RECOVERY)
IMG_VOID SGXOSTimer(IMG_VOID *pvData)
{
	PVRSRV_DEVICE_NODE *psDeviceNode = pvData;
	PVRSRV_SGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	static IMG_UINT32	ui32EDMTasks = 0;
	static IMG_UINT32	ui32LockupCounter = 0; 
	static IMG_UINT32	ui32NumResets = 0;
	IMG_UINT32		ui32CurrentEDMTasks;
	IMG_BOOL		bLockup = IMG_FALSE;
	IMG_BOOL		bPoweredDown;

	
	psDevInfo->ui32TimeStamp++;

#if defined(NO_HARDWARE)
	bPoweredDown = IMG_TRUE;
#else
	bPoweredDown = (IMG_BOOL)!SGXIsDevicePowered(psDeviceNode);
#endif 

	
	
	if (bPoweredDown)
	{
		ui32LockupCounter = 0;
	}
	else
	{
		
		ui32CurrentEDMTasks = OSReadHWReg(psDevInfo->pvRegsBaseKM, psDevInfo->ui32EDMTaskReg0);
		if (psDevInfo->ui32EDMTaskReg1 != 0)
		{
			ui32CurrentEDMTasks ^= OSReadHWReg(psDevInfo->pvRegsBaseKM, psDevInfo->ui32EDMTaskReg1);
		}
		if ((ui32CurrentEDMTasks == ui32EDMTasks) &&
			(psDevInfo->ui32NumResets == ui32NumResets))
		{
			ui32LockupCounter++;
			if (ui32LockupCounter == 3)
			{
				ui32LockupCounter = 0;
				PVR_DPF((PVR_DBG_ERROR, "SGXOSTimer() detected SGX lockup (0x%x tasks)", ui32EDMTasks));

				bLockup = IMG_TRUE;
			}
		}
		else
		{
			ui32LockupCounter = 0;
			ui32EDMTasks = ui32CurrentEDMTasks;
			ui32NumResets = psDevInfo->ui32NumResets;
		}
	}

	if (bLockup)
	{
		SGXMKIF_HOST_CTL	*psSGXHostCtl = (SGXMKIF_HOST_CTL *)psDevInfo->psSGXHostCtl;

		
		psSGXHostCtl->ui32HostDetectedLockups ++;

		
		HWRecoveryResetSGX(psDeviceNode, 0, KERNEL_ID);
	}
}
#endif 


#if defined(SYS_USING_INTERRUPTS)


IMG_BOOL SGX_ISRHandler (IMG_VOID *pvData)
{
	IMG_BOOL bInterruptProcessed = IMG_FALSE;

	
	{
		IMG_UINT32 ui32EventStatus, ui32EventEnable;
		IMG_UINT32 ui32EventClear = 0;
		PVRSRV_DEVICE_NODE *psDeviceNode;
		PVRSRV_SGXDEV_INFO *psDevInfo;

		
		if(pvData == IMG_NULL)
		{
			PVR_DPF((PVR_DBG_ERROR, "SGX_ISRHandler: Invalid params\n"));
			return bInterruptProcessed;
		}

		psDeviceNode = (PVRSRV_DEVICE_NODE *)pvData;
		psDevInfo = (PVRSRV_SGXDEV_INFO *)psDeviceNode->pvDevice;

		ui32EventStatus = OSReadHWReg(psDevInfo->pvRegsBaseKM, EUR_CR_EVENT_STATUS);
		ui32EventEnable = OSReadHWReg(psDevInfo->pvRegsBaseKM, EUR_CR_EVENT_HOST_ENABLE);

		

		gui32EventStatusServicesByISR = ui32EventStatus;

		
		ui32EventStatus &= ui32EventEnable;

		if (ui32EventStatus & EUR_CR_EVENT_STATUS_SW_EVENT_MASK)
		{
			ui32EventClear |= EUR_CR_EVENT_HOST_CLEAR_SW_EVENT_MASK;
		}

		if (ui32EventClear)
		{
			bInterruptProcessed = IMG_TRUE;

			
			ui32EventClear |= EUR_CR_EVENT_HOST_CLEAR_MASTER_INTERRUPT_MASK;

			
			OSWriteHWReg(psDevInfo->pvRegsBaseKM, EUR_CR_EVENT_HOST_CLEAR, ui32EventClear);
		}
	}

	return bInterruptProcessed;
}


IMG_VOID SGX_MISRHandler (IMG_VOID *pvData)
{
	PVRSRV_DEVICE_NODE	*psDeviceNode = (PVRSRV_DEVICE_NODE *)pvData;
	PVRSRV_SGXDEV_INFO	*psDevInfo = (PVRSRV_SGXDEV_INFO*)psDeviceNode->pvDevice;
	SGXMKIF_HOST_CTL	*psSGXHostCtl = (SGXMKIF_HOST_CTL *)psDevInfo->psSGXHostCtl;

	if (((psSGXHostCtl->ui32InterruptFlags & PVRSRV_USSE_EDM_INTERRUPT_HWR) != 0UL) &&
		((psSGXHostCtl->ui32InterruptClearFlags & PVRSRV_USSE_EDM_INTERRUPT_HWR) == 0UL))
	{
		HWRecoveryResetSGX(psDeviceNode, 0, ISR_ID);
	}

#if defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
	SGXTestActivePowerEvent(psDeviceNode, ISR_ID);
#endif 
}
#endif 


PVRSRV_ERROR SGXRegisterDevice (PVRSRV_DEVICE_NODE *psDeviceNode)
{
	DEVICE_MEMORY_INFO *psDevMemoryInfo;
	DEVICE_MEMORY_HEAP_INFO *psDeviceMemoryHeap;

	
	psDeviceNode->sDevId.eDeviceType	= DEV_DEVICE_TYPE;
	psDeviceNode->sDevId.eDeviceClass	= DEV_DEVICE_CLASS;

	psDeviceNode->pfnInitDevice		= DevInitSGXPart1;
	psDeviceNode->pfnDeInitDevice		= DevDeInitSGX;

	psDeviceNode->pfnInitDeviceCompatCheck	= SGXDevInitCompatCheck;

	

	psDeviceNode->pfnMMUInitialise = MMU_Initialise;
	psDeviceNode->pfnMMUFinalise = MMU_Finalise;
	psDeviceNode->pfnMMUInsertHeap = MMU_InsertHeap;
	psDeviceNode->pfnMMUCreate = MMU_Create;
	psDeviceNode->pfnMMUDelete = MMU_Delete;
	psDeviceNode->pfnMMUAlloc = MMU_Alloc;
	psDeviceNode->pfnMMUFree = MMU_Free;
	psDeviceNode->pfnMMUMapPages = MMU_MapPages;
	psDeviceNode->pfnMMUMapShadow = MMU_MapShadow;
	psDeviceNode->pfnMMUUnmapPages = MMU_UnmapPages;
	psDeviceNode->pfnMMUMapScatter = MMU_MapScatter;
	psDeviceNode->pfnMMUGetPhysPageAddr = MMU_GetPhysPageAddr;
	psDeviceNode->pfnMMUGetPDDevPAddr = MMU_GetPDDevPAddr;

#if defined (SYS_USING_INTERRUPTS)
	

	psDeviceNode->pfnDeviceISR = SGX_ISRHandler;
	psDeviceNode->pfnDeviceMISR = SGX_MISRHandler;
#endif

	

	psDeviceNode->pfnDeviceCommandComplete = SGXCommandComplete;

	

	psDevMemoryInfo = &psDeviceNode->sDevMemoryInfo;
	
	psDevMemoryInfo->ui32AddressSpaceSizeLog2 = SGX_FEATURE_ADDRESS_SPACE_SIZE;

	
	psDevMemoryInfo->ui32Flags = 0;

	
	psDevMemoryInfo->ui32HeapCount = SGX_MAX_HEAP_ID;

	
	psDevMemoryInfo->ui32SyncHeapID = SGX_SYNCINFO_HEAP_ID;

	
	psDevMemoryInfo->ui32MappingHeapID = SGX_GENERAL_MAPPING_HEAP_ID;

	
	if(OSAllocMem( PVRSRV_OS_PAGEABLE_HEAP,
					 sizeof(DEVICE_MEMORY_HEAP_INFO) * psDevMemoryInfo->ui32HeapCount,
					 (IMG_VOID **)&psDevMemoryInfo->psDeviceMemoryHeap, 0) != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"SGXRegisterDevice : Failed to alloc memory for DEVICE_MEMORY_HEAP_INFO"));
		return (PVRSRV_ERROR_OUT_OF_MEMORY);
	}
	OSMemSet(psDevMemoryInfo->psDeviceMemoryHeap, 0, sizeof(DEVICE_MEMORY_HEAP_INFO) * psDevMemoryInfo->ui32HeapCount);

	psDeviceMemoryHeap = psDevMemoryInfo->psDeviceMemoryHeap;

	


	
	psDeviceMemoryHeap[SGX_GENERAL_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX , SGX_GENERAL_HEAP_ID);
	psDeviceMemoryHeap[SGX_GENERAL_HEAP_ID].sDevVAddrBase.uiAddr = SGX_GENERAL_HEAP_BASE;
	psDeviceMemoryHeap[SGX_GENERAL_HEAP_ID].ui32HeapSize = SGX_GENERAL_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_GENERAL_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
														| PVRSRV_MEM_RAM_BACKED_ALLOCATION
														| PVRSRV_HAP_SINGLE_PROCESS;
	psDeviceMemoryHeap[SGX_GENERAL_HEAP_ID].pszName = "General";
	psDeviceMemoryHeap[SGX_GENERAL_HEAP_ID].pszBSName = "General BS";
	psDeviceMemoryHeap[SGX_GENERAL_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_PERCONTEXT;
	
	psDeviceMemoryHeap[SGX_GENERAL_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

	
	psDeviceMemoryHeap[SGX_TADATA_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX , SGX_TADATA_HEAP_ID);
	psDeviceMemoryHeap[SGX_TADATA_HEAP_ID].sDevVAddrBase.uiAddr = SGX_TADATA_HEAP_BASE;
	psDeviceMemoryHeap[SGX_TADATA_HEAP_ID].ui32HeapSize = SGX_TADATA_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_TADATA_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
														| PVRSRV_MEM_RAM_BACKED_ALLOCATION
														| PVRSRV_HAP_MULTI_PROCESS;
	psDeviceMemoryHeap[SGX_TADATA_HEAP_ID].pszName = "TA Data";
	psDeviceMemoryHeap[SGX_TADATA_HEAP_ID].pszBSName = "TA Data BS";
	psDeviceMemoryHeap[SGX_TADATA_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_PERCONTEXT;
	
	psDeviceMemoryHeap[SGX_TADATA_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

	
	psDeviceMemoryHeap[SGX_KERNEL_CODE_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX ,SGX_KERNEL_CODE_HEAP_ID);
	psDeviceMemoryHeap[SGX_KERNEL_CODE_HEAP_ID].sDevVAddrBase.uiAddr = SGX_KERNEL_CODE_HEAP_BASE;
	psDeviceMemoryHeap[SGX_KERNEL_CODE_HEAP_ID].ui32HeapSize = SGX_KERNEL_CODE_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_KERNEL_CODE_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
															| PVRSRV_MEM_RAM_BACKED_ALLOCATION
															| PVRSRV_HAP_MULTI_PROCESS;
	psDeviceMemoryHeap[SGX_KERNEL_CODE_HEAP_ID].pszName = "Kernel Code";
	psDeviceMemoryHeap[SGX_KERNEL_CODE_HEAP_ID].pszBSName = "Kernel Code BS";
	psDeviceMemoryHeap[SGX_KERNEL_CODE_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_SHARED_EXPORTED;
	
	psDeviceMemoryHeap[SGX_KERNEL_CODE_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

	
	psDeviceMemoryHeap[SGX_KERNEL_DATA_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX ,SGX_KERNEL_DATA_HEAP_ID);
	psDeviceMemoryHeap[SGX_KERNEL_DATA_HEAP_ID].sDevVAddrBase.uiAddr = SGX_KERNEL_DATA_HEAP_BASE;
	psDeviceMemoryHeap[SGX_KERNEL_DATA_HEAP_ID].ui32HeapSize = SGX_KERNEL_DATA_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_KERNEL_DATA_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
																| PVRSRV_MEM_RAM_BACKED_ALLOCATION
																| PVRSRV_HAP_MULTI_PROCESS;
	psDeviceMemoryHeap[SGX_KERNEL_DATA_HEAP_ID].pszName = "KernelData";
	psDeviceMemoryHeap[SGX_KERNEL_DATA_HEAP_ID].pszBSName = "KernelData BS";
	psDeviceMemoryHeap[SGX_KERNEL_DATA_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_SHARED_EXPORTED;
	
	psDeviceMemoryHeap[SGX_KERNEL_DATA_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

	
	psDeviceMemoryHeap[SGX_PIXELSHADER_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX ,SGX_PIXELSHADER_HEAP_ID);
	psDeviceMemoryHeap[SGX_PIXELSHADER_HEAP_ID].sDevVAddrBase.uiAddr = SGX_PIXELSHADER_HEAP_BASE;
	psDeviceMemoryHeap[SGX_PIXELSHADER_HEAP_ID].ui32HeapSize = SGX_PIXELSHADER_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_PIXELSHADER_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
																| PVRSRV_MEM_RAM_BACKED_ALLOCATION
																| PVRSRV_HAP_SINGLE_PROCESS;
	psDeviceMemoryHeap[SGX_PIXELSHADER_HEAP_ID].pszName = "PixelShaderUSSE";
	psDeviceMemoryHeap[SGX_PIXELSHADER_HEAP_ID].pszBSName = "PixelShaderUSSE BS";
	psDeviceMemoryHeap[SGX_PIXELSHADER_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_PERCONTEXT;
	
	psDeviceMemoryHeap[SGX_PIXELSHADER_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

	
	psDeviceMemoryHeap[SGX_VERTEXSHADER_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX ,SGX_VERTEXSHADER_HEAP_ID);
	psDeviceMemoryHeap[SGX_VERTEXSHADER_HEAP_ID].sDevVAddrBase.uiAddr = SGX_VERTEXSHADER_HEAP_BASE;
	psDeviceMemoryHeap[SGX_VERTEXSHADER_HEAP_ID].ui32HeapSize = SGX_VERTEXSHADER_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_VERTEXSHADER_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
																| PVRSRV_MEM_RAM_BACKED_ALLOCATION
																| PVRSRV_HAP_SINGLE_PROCESS;
	psDeviceMemoryHeap[SGX_VERTEXSHADER_HEAP_ID].pszName = "VertexShaderUSSE";
	psDeviceMemoryHeap[SGX_VERTEXSHADER_HEAP_ID].pszBSName = "VertexShaderUSSE BS";
	psDeviceMemoryHeap[SGX_VERTEXSHADER_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_PERCONTEXT;
	
	psDeviceMemoryHeap[SGX_VERTEXSHADER_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

	
	psDeviceMemoryHeap[SGX_PDSPIXEL_CODEDATA_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX ,SGX_PDSPIXEL_CODEDATA_HEAP_ID);
	psDeviceMemoryHeap[SGX_PDSPIXEL_CODEDATA_HEAP_ID].sDevVAddrBase.uiAddr = SGX_PDSPIXEL_CODEDATA_HEAP_BASE;
	psDeviceMemoryHeap[SGX_PDSPIXEL_CODEDATA_HEAP_ID].ui32HeapSize = SGX_PDSPIXEL_CODEDATA_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_PDSPIXEL_CODEDATA_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
																| PVRSRV_MEM_RAM_BACKED_ALLOCATION
																| PVRSRV_HAP_SINGLE_PROCESS;
	psDeviceMemoryHeap[SGX_PDSPIXEL_CODEDATA_HEAP_ID].pszName = "PDSPixelCodeData";
	psDeviceMemoryHeap[SGX_PDSPIXEL_CODEDATA_HEAP_ID].pszBSName = "PDSPixelCodeData BS";
	psDeviceMemoryHeap[SGX_PDSPIXEL_CODEDATA_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_PERCONTEXT;
	
	psDeviceMemoryHeap[SGX_PDSPIXEL_CODEDATA_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

	
	psDeviceMemoryHeap[SGX_PDSVERTEX_CODEDATA_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX ,SGX_PDSVERTEX_CODEDATA_HEAP_ID);
	psDeviceMemoryHeap[SGX_PDSVERTEX_CODEDATA_HEAP_ID].sDevVAddrBase.uiAddr = SGX_PDSVERTEX_CODEDATA_HEAP_BASE;
	psDeviceMemoryHeap[SGX_PDSVERTEX_CODEDATA_HEAP_ID].ui32HeapSize = SGX_PDSVERTEX_CODEDATA_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_PDSVERTEX_CODEDATA_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
																| PVRSRV_MEM_RAM_BACKED_ALLOCATION
																| PVRSRV_HAP_SINGLE_PROCESS;
	psDeviceMemoryHeap[SGX_PDSVERTEX_CODEDATA_HEAP_ID].pszName = "PDSVertexCodeData";
	psDeviceMemoryHeap[SGX_PDSVERTEX_CODEDATA_HEAP_ID].pszBSName = "PDSVertexCodeData BS";
	psDeviceMemoryHeap[SGX_PDSVERTEX_CODEDATA_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_PERCONTEXT;
	
	psDeviceMemoryHeap[SGX_PDSVERTEX_CODEDATA_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

	
	psDeviceMemoryHeap[SGX_SYNCINFO_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX ,SGX_SYNCINFO_HEAP_ID);
	psDeviceMemoryHeap[SGX_SYNCINFO_HEAP_ID].sDevVAddrBase.uiAddr = SGX_SYNCINFO_HEAP_BASE;
	psDeviceMemoryHeap[SGX_SYNCINFO_HEAP_ID].ui32HeapSize = SGX_SYNCINFO_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_SYNCINFO_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
														| PVRSRV_MEM_RAM_BACKED_ALLOCATION
														| PVRSRV_HAP_MULTI_PROCESS;
	psDeviceMemoryHeap[SGX_SYNCINFO_HEAP_ID].pszName = "CacheCoherent";
	psDeviceMemoryHeap[SGX_SYNCINFO_HEAP_ID].pszBSName = "CacheCoherent BS";
	
	psDeviceMemoryHeap[SGX_SYNCINFO_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_SHARED_EXPORTED;
	
	psDeviceMemoryHeap[SGX_SYNCINFO_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

	
	psDeviceMemoryHeap[SGX_3DPARAMETERS_HEAP_ID].ui32HeapID = HEAP_ID(PVRSRV_DEVICE_TYPE_SGX, SGX_3DPARAMETERS_HEAP_ID);
	psDeviceMemoryHeap[SGX_3DPARAMETERS_HEAP_ID].sDevVAddrBase.uiAddr = SGX_3DPARAMETERS_HEAP_BASE;
	psDeviceMemoryHeap[SGX_3DPARAMETERS_HEAP_ID].ui32HeapSize = SGX_3DPARAMETERS_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_3DPARAMETERS_HEAP_ID].pszName = "3DParameters";
	psDeviceMemoryHeap[SGX_3DPARAMETERS_HEAP_ID].pszBSName = "3DParameters BS";
#if defined(SUPPORT_PERCONTEXT_PB)
	psDeviceMemoryHeap[SGX_3DPARAMETERS_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
															| PVRSRV_MEM_RAM_BACKED_ALLOCATION
															| PVRSRV_HAP_SINGLE_PROCESS;
	psDeviceMemoryHeap[SGX_3DPARAMETERS_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_PERCONTEXT;
#else
	psDeviceMemoryHeap[SGX_3DPARAMETERS_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
													| PVRSRV_MEM_RAM_BACKED_ALLOCATION
													| PVRSRV_HAP_MULTI_PROCESS;
	psDeviceMemoryHeap[SGX_3DPARAMETERS_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_SHARED_EXPORTED;
#endif
	
	psDeviceMemoryHeap[SGX_3DPARAMETERS_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

	
	psDeviceMemoryHeap[SGX_GENERAL_MAPPING_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX , SGX_GENERAL_MAPPING_HEAP_ID);
	psDeviceMemoryHeap[SGX_GENERAL_MAPPING_HEAP_ID].sDevVAddrBase.uiAddr = SGX_GENERAL_MAPPING_HEAP_BASE;
	psDeviceMemoryHeap[SGX_GENERAL_MAPPING_HEAP_ID].ui32HeapSize = SGX_GENERAL_MAPPING_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_GENERAL_MAPPING_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE | PVRSRV_HAP_MULTI_PROCESS;
	psDeviceMemoryHeap[SGX_GENERAL_MAPPING_HEAP_ID].pszName = "GeneralMapping";
	psDeviceMemoryHeap[SGX_GENERAL_MAPPING_HEAP_ID].pszBSName = "GeneralMapping BS";
#if defined(SGX_FEATURE_2D_HARDWARE) && defined(SGX_FEATURE_MULTIPLE_MEM_CONTEXTS) && defined(FIX_HW_BRN_23410)
	






	psDeviceMemoryHeap[SGX_GENERAL_MAPPING_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_SHARED_EXPORTED;
#else
	psDeviceMemoryHeap[SGX_GENERAL_MAPPING_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_PERCONTEXT;
#endif
	
	psDeviceMemoryHeap[SGX_GENERAL_MAPPING_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;

#if defined(SGX_FEATURE_2D_HARDWARE)

	
	psDeviceMemoryHeap[SGX_2D_HEAP_ID].ui32HeapID = HEAP_ID( PVRSRV_DEVICE_TYPE_SGX ,SGX_2D_HEAP_ID);
	psDeviceMemoryHeap[SGX_2D_HEAP_ID].sDevVAddrBase.uiAddr = SGX_2D_HEAP_BASE;
	psDeviceMemoryHeap[SGX_2D_HEAP_ID].ui32HeapSize = SGX_2D_HEAP_SIZE;
	psDeviceMemoryHeap[SGX_2D_HEAP_ID].ui32Attribs = PVRSRV_HAP_WRITECOMBINE
														| PVRSRV_MEM_RAM_BACKED_ALLOCATION
														| PVRSRV_HAP_SINGLE_PROCESS;
	psDeviceMemoryHeap[SGX_2D_HEAP_ID].pszName = "2D";
	psDeviceMemoryHeap[SGX_2D_HEAP_ID].pszBSName = "2D BS";
	
	psDeviceMemoryHeap[SGX_2D_HEAP_ID].DevMemHeapType = DEVICE_MEMORY_HEAP_SHARED_EXPORTED;
	
	psDeviceMemoryHeap[SGX_2D_HEAP_ID].ui32DataPageSize = SGX_MMU_PAGE_SIZE;
#endif 


	return PVRSRV_OK;
}

IMG_EXPORT
PVRSRV_ERROR SGXGetClientInfoKM(IMG_HANDLE					hDevCookie,
								SGX_CLIENT_INFO*		psClientInfo)
{
	PVRSRV_SGXDEV_INFO *psDevInfo = (PVRSRV_SGXDEV_INFO *)((PVRSRV_DEVICE_NODE *)hDevCookie)->pvDevice;

	

	psDevInfo->ui32ClientRefCount++;
#ifdef PDUMP
	if(psDevInfo->ui32ClientRefCount == 1)
	{
		psDevInfo->psKernelCCBInfo->ui32CCBDumpWOff = 0;
	}
#endif
	

	psClientInfo->ui32ProcessID = OSGetCurrentProcessIDKM();

	

	OSMemCopy(&psClientInfo->asDevData, &psDevInfo->asSGXDevData, sizeof(psClientInfo->asDevData));

	
	return PVRSRV_OK;
}

PVRSRV_ERROR SGXDevInitCompatCheck(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_SGXDEV_INFO 				*psDevInfo;
	PPVRSRV_KERNEL_MEM_INFO			psMemInfo;
	PVRSRV_ERROR	eError;
#if !defined(NO_HARDWARE)
	IMG_UINT32 			ui32BuildOptions, ui32BuildOptionsMismatch;
	PVRSRV_SGX_MISCINFO_FEATURES	*psSGXFeatures;
#endif

	
	if(psDeviceNode->sDevId.eDeviceType != PVRSRV_DEVICE_TYPE_SGX)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXDevInitCompatCheck: Device not of type SGX"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto exit;
	}
	psDevInfo = psDeviceNode->pvDevice;
	psMemInfo = psDevInfo->psKernelSGXMiscMemInfo;

#if !defined (NO_HARDWARE)
	
	eError = SGXGetBuildInfoKM(psDevInfo, psDeviceNode);
	if(eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXDevInitCompatCheck: Unable to validate device DDK version"));
		goto exit;
	}
	psSGXFeatures = &((PVRSRV_SGX_MISCINFO_INFO*)(psMemInfo->pvLinAddrKM))->sSGXFeatures;
	if( (psSGXFeatures->ui32DDKVersion !=
		((PVRVERSION_MAJ << 16) |
		 (PVRVERSION_MIN << 8) |
		  PVRVERSION_BRANCH) ) ||
		(psSGXFeatures->ui32DDKBuild != PVRVERSION_BUILD) )
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXDevInitCompatCheck: Incompatible driver DDK revision (%ld)/device DDK revision (%ld).",
				PVRVERSION_BUILD, psSGXFeatures->ui32DDKBuild));
		eError = PVRSRV_ERROR_DDK_VERSION_MISMATCH;
		goto exit;
	}
	else
	{
		PVR_DPF((PVR_DBG_WARNING, "(Success) SGXInit: driver DDK (%ld) and device DDK (%ld) match",
				PVRVERSION_BUILD, psSGXFeatures->ui32DDKBuild));
	}


	
	ui32BuildOptions = psSGXFeatures->ui32BuildOptions;
	if (ui32BuildOptions != (SGX_BUILD_OPTIONS))
	{
		ui32BuildOptionsMismatch = ui32BuildOptions ^ (SGX_BUILD_OPTIONS);
		if ( ((SGX_BUILD_OPTIONS) & ui32BuildOptionsMismatch) != 0)
		{
			PVR_DPF((PVR_DBG_ERROR, "SGXInit: Mismatch in driver and microkernel build options; "
				"extra options present in driver: (0x%lx)",
				(SGX_BUILD_OPTIONS) & ui32BuildOptionsMismatch ));
		}

		if ( (ui32BuildOptions & ui32BuildOptionsMismatch) != 0)
		{
			PVR_DPF((PVR_DBG_ERROR, "SGXInit: Mismatch in driver and microkernel build options; "
				"extra options present in microkernel: (0x%lx)",
				ui32BuildOptions & ui32BuildOptionsMismatch ));
		}
		eError = PVRSRV_ERROR_BUILD_MISMATCH;
		goto exit;
	}
	else
	{
		PVR_DPF((PVR_DBG_WARNING, "(Success) SGXInit: Driver and microkernel build options match."));
	}

#endif
	eError = PVRSRV_OK;
exit:
#if defined(IGNORE_SGX_INIT_COMPATIBILITY_CHECK)
	return PVRSRV_OK;
#else
	return eError;
#endif
}

static
PVRSRV_ERROR SGXGetBuildInfoKM(PVRSRV_SGXDEV_INFO	*psDevInfo,
							  PVRSRV_DEVICE_NODE 	*psDeviceNode)
{
	PVRSRV_ERROR		eError;
	SGXMKIF_COMMAND		sCommandData;  
	PVRSRV_SGX_MISCINFO_INFO			*psSGXMiscInfoInt; 	
	PVRSRV_SGX_MISCINFO_FEATURES		*psSGXFeatures;		

	PPVRSRV_KERNEL_MEM_INFO	psMemInfo = psDevInfo->psKernelSGXMiscMemInfo;

	if (! psMemInfo->pvLinAddrKM)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXGetMiscInfoKM: Invalid address."));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}
	psSGXMiscInfoInt = psMemInfo->pvLinAddrKM;
	psSGXMiscInfoInt->ui32MiscInfoFlags &= ~PVRSRV_USSE_MISCINFO_READY;
	psSGXFeatures = &psSGXMiscInfoInt->sSGXFeatures;

	
	OSMemSet(psMemInfo->pvLinAddrKM, 0,
			sizeof(PVRSRV_SGX_MISCINFO_INFO));

	
	sCommandData.ui32Data[1] = psMemInfo->sDevVAddr.uiAddr; 

	eError = SGXScheduleCCBCommandKM(psDeviceNode,
			SGXMKIF_COMMAND_REQUEST_SGXMISCINFO,
			&sCommandData,
			KERNEL_ID,
			0);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXGetMiscInfoKM: SGXScheduleCCBCommandKM failed."));
		return eError;
	}

	
	OSMemSet(psSGXFeatures, 0, sizeof(*psSGXFeatures));
#if !defined(NO_HARDWARE)
	{
		IMG_UINT32	ui32StartTimeMS, ui32EndTimeMS;

		ui32StartTimeMS = OSClockus();
		while ( (psSGXMiscInfoInt->ui32MiscInfoFlags & PVRSRV_USSE_MISCINFO_READY) == 0)
		{
			ui32EndTimeMS = OSClockus(); 
			if(ui32EndTimeMS - ui32StartTimeMS > MAX_HW_TIME_US)
			{
				return PVRSRV_ERROR_TIMEOUT;
			}
		}
	}
#endif 

	return PVRSRV_OK;
}

IMG_EXPORT
PVRSRV_ERROR SGXGetMiscInfoKM(PVRSRV_SGXDEV_INFO	*psDevInfo,
							  SGX_MISC_INFO			*psMiscInfo,
 							  PVRSRV_DEVICE_NODE 	*psDeviceNode)
{
	switch(psMiscInfo->eRequest)
	{
		case SGX_MISC_INFO_REQUEST_CLOCKSPEED:
		{
			psMiscInfo->uData.ui32SGXClockSpeed = psDevInfo->ui32CoreClockSpeed;
			return PVRSRV_OK;
		}

		case SGX_MISC_INFO_REQUEST_SGXREV:
		{
			PVRSRV_SGX_MISCINFO_FEATURES		*psSGXFeatures;
			PPVRSRV_KERNEL_MEM_INFO	psMemInfo = psDevInfo->psKernelSGXMiscMemInfo;

			SGXGetBuildInfoKM(psDevInfo, psDeviceNode);
			psSGXFeatures = &((PVRSRV_SGX_MISCINFO_INFO*)(psMemInfo->pvLinAddrKM))->sSGXFeatures;

			
			psMiscInfo->uData.sSGXFeatures = *psSGXFeatures;

			
			PVR_DPF((PVR_DBG_MESSAGE, "SGXGetMiscInfoKM: Core 0x%lx, sw ID 0x%lx, sw Rev 0x%lx\n",
					psSGXFeatures->ui32CoreRev,
					psSGXFeatures->ui32CoreIdSW,
					psSGXFeatures->ui32CoreRevSW));
			PVR_DPF((PVR_DBG_MESSAGE, "SGXGetMiscInfoKM: DDK version 0x%lx, DDK build 0x%lx\n",
					psSGXFeatures->ui32DDKVersion,
					psSGXFeatures->ui32DDKBuild));

			
			return PVRSRV_OK;
		}

		case SGX_MISC_INFO_REQUEST_DRIVER_SGXREV:
		{
			PPVRSRV_KERNEL_MEM_INFO	psMemInfo = psDevInfo->psKernelSGXMiscMemInfo;
			PVRSRV_SGX_MISCINFO_FEATURES		*psSGXFeatures;

			psSGXFeatures = &((PVRSRV_SGX_MISCINFO_INFO*)(psMemInfo->pvLinAddrKM))->sSGXFeatures;

			
			OSMemSet(psMemInfo->pvLinAddrKM, 0,
					sizeof(PVRSRV_SGX_MISCINFO_INFO));

			psSGXFeatures->ui32DDKVersion =
				(PVRVERSION_MAJ << 16) |
				(PVRVERSION_MIN << 8) |
				PVRVERSION_BRANCH;
			psSGXFeatures->ui32DDKBuild = PVRVERSION_BUILD;

			
			psMiscInfo->uData.sSGXFeatures = *psSGXFeatures;
			return PVRSRV_OK;
		}

#ifdef SUPPORT_SGX_HWPERF
		case SGX_MISC_INFO_REQUEST_SET_HWPERF_STATUS:
		{
			SGXMKIF_HWPERF_CB *psHWPerfCB = psDevInfo->psKernelHWPerfCBMemInfo->pvLinAddrKM;
			IMG_UINT ui32MatchingFlags;

			
			if ((psMiscInfo->uData.ui32NewHWPerfStatus & ~(PVRSRV_SGX_HWPERF_GRAPHICS_ON | PVRSRV_SGX_HWPERF_MK_EXECUTION_ON)) != 0)
			{
				return PVRSRV_ERROR_INVALID_PARAMS;
			}

			
			ui32MatchingFlags = psMiscInfo->uData.ui32NewHWPerfStatus & psDevInfo->psSGXHostCtl->ui32HWPerfFlags;
			if((ui32MatchingFlags & PVRSRV_SGX_HWPERF_GRAPHICS_ON) == 0UL)
			{
				psHWPerfCB->ui32OrdinalGRAPHICS = 0xffffffff;
			}
			if((ui32MatchingFlags & PVRSRV_SGX_HWPERF_MK_EXECUTION_ON) == 0UL)
			{
				psHWPerfCB->ui32OrdinalMK_EXECUTION = 0xffffffffUL;
			}

			
			psDevInfo->psSGXHostCtl->ui32HWPerfFlags = psMiscInfo->uData.ui32NewHWPerfStatus;
			#if defined(PDUMP)
			PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "SGX ukernel HWPerf status %lu\n",
								  psDevInfo->psSGXHostCtl->ui32HWPerfFlags);
			PDUMPMEM(IMG_NULL, psDevInfo->psKernelSGXHostCtlMemInfo,
					 offsetof(SGXMKIF_HOST_CTL, ui32HWPerfFlags),
					 sizeof(psDevInfo->psSGXHostCtl->ui32HWPerfFlags), PDUMP_FLAGS_CONTINUOUS,
					 MAKEUNIQUETAG(psDevInfo->psKernelSGXHostCtlMemInfo));
			#endif 

			return PVRSRV_OK;
		}
		case SGX_MISC_INFO_REQUEST_HWPERF_CB_ON:
		{
			
			SGXMKIF_HWPERF_CB *psHWPerfCB = psDevInfo->psKernelHWPerfCBMemInfo->pvLinAddrKM;
			psHWPerfCB->ui32OrdinalGRAPHICS = 0xffffffffUL;
			
			psDevInfo->psSGXHostCtl->ui32HWPerfFlags |= PVRSRV_SGX_HWPERF_GRAPHICS_ON;
			return PVRSRV_OK;
		}
		case SGX_MISC_INFO_REQUEST_HWPERF_CB_OFF:
		{
			
			psDevInfo->psSGXHostCtl->ui32HWPerfFlags = 0;
			return PVRSRV_OK;
		}
		case SGX_MISC_INFO_REQUEST_HWPERF_RETRIEVE_CB:
		{
			
			SGX_MISC_INFO_HWPERF_RETRIEVE_CB *psRetrieve = &psMiscInfo->uData.sRetrieveCB;
			SGXMKIF_HWPERF_CB *psHWPerfCB = psDevInfo->psKernelHWPerfCBMemInfo->pvLinAddrKM;
			IMG_UINT i;

			for (i = 0; psHWPerfCB->ui32Woff != psHWPerfCB->ui32Roff && i < psRetrieve->ui32ArraySize; i++)
			{
				SGXMKIF_HWPERF_CB_ENTRY *psData = &psHWPerfCB->psHWPerfCBData[psHWPerfCB->ui32Roff];
				


				psRetrieve->psHWPerfData[i].ui32FrameNo = psData->ui32FrameNo;
				psRetrieve->psHWPerfData[i].ui32Type = (psData->ui32Type & PVRSRV_SGX_HWPERF_TYPE_OP_MASK);
				psRetrieve->psHWPerfData[i].ui32StartTime = psData->ui32Time;
				psRetrieve->psHWPerfData[i].ui32StartTimeWraps = psData->ui32TimeWraps;
				psRetrieve->psHWPerfData[i].ui32EndTime = psData->ui32Time;
				psRetrieve->psHWPerfData[i].ui32EndTimeWraps = psData->ui32TimeWraps;
				psRetrieve->psHWPerfData[i].ui32ClockSpeed = psDevInfo->ui32CoreClockSpeed;
				psRetrieve->psHWPerfData[i].ui32TimeMax = psDevInfo->ui32uKernelTimerClock;
				psHWPerfCB->ui32Roff = (psHWPerfCB->ui32Roff + 1) & (SGXMKIF_HWPERF_CB_SIZE - 1);
			}
			psRetrieve->ui32DataCount = i;
			psRetrieve->ui32Time = OSClockus();
			return PVRSRV_OK;
		}
#endif 
		default:
		{
			
			return PVRSRV_ERROR_INVALID_PARAMS;
		}
	}
}

#if defined(SUPPORT_SGX_HWPERF)
IMG_EXPORT
PVRSRV_ERROR SGXReadDiffCountersKM(IMG_HANDLE					hDevHandle,
									 IMG_UINT32					ui32Reg,
									 IMG_UINT32					*pui32Old,
									 IMG_BOOL					bNew,
									 IMG_UINT32					ui32New,
									 IMG_UINT32					ui32NewReset,
									 IMG_UINT32					ui32CountersReg,
									 IMG_UINT32					*pui32Time,
									 IMG_BOOL					*pbActive,
 									 PVRSRV_SGXDEV_DIFF_INFO	*psDiffs)
{
	PVRSRV_ERROR    	eError;
	SYS_DATA			*psSysData;
	PVRSRV_POWER_DEV	*psPowerDevice;
	IMG_BOOL			bPowered = IMG_FALSE;
	PVRSRV_DEVICE_NODE	*psDeviceNode = hDevHandle;
	PVRSRV_SGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;

	
	if(bNew)
	{
		psDevInfo->ui32HWGroupRequested = ui32New;
	}
	psDevInfo->ui32HWReset |= ui32NewReset;

	
	eError = PVRSRVPowerLock(KERNEL_ID, IMG_FALSE);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	SysAcquireData(&psSysData);

	
	psPowerDevice = psSysData->psPowerDeviceList;
	while (psPowerDevice)
	{
		if (psPowerDevice->ui32DeviceIndex == psDeviceNode->sDevId.ui32DeviceIndex)
		{
			bPowered = (IMG_BOOL)(psPowerDevice->eCurrentPowerState == PVRSRV_POWER_STATE_D0);
			break;
		}

		psPowerDevice = psPowerDevice->psNext;
	}

	
	*pbActive = bPowered;

	

	{
		PVRSRV_SGXDEV_DIFF_INFO	sNew, *psPrev = &psDevInfo->sDiffInfo;
		IMG_UINT32					i;

		sNew.ui32Time[0] = OSClockus();

		
		*pui32Time = sNew.ui32Time[0];

		
		if(sNew.ui32Time[0] != psPrev->ui32Time[0] && bPowered)
		{
			
			*pui32Old = OSReadHWReg(psDevInfo->pvRegsBaseKM, ui32Reg);

			for (i = 0; i < PVRSRV_SGX_DIFF_NUM_COUNTERS; ++i)
			{
				sNew.aui32Counters[i] = OSReadHWReg(psDevInfo->pvRegsBaseKM, ui32CountersReg + (i * 4));
			}

			

			if (psDevInfo->ui32HWGroupRequested != *pui32Old)
			{
				
				if(psDevInfo->ui32HWReset != 0)
				{
					OSWriteHWReg(psDevInfo->pvRegsBaseKM, ui32Reg, psDevInfo->ui32HWGroupRequested | psDevInfo->ui32HWReset);
					psDevInfo->ui32HWReset = 0;
				}

				OSWriteHWReg(psDevInfo->pvRegsBaseKM, ui32Reg, psDevInfo->ui32HWGroupRequested);
			}

			sNew.ui32Marker[0] = psDevInfo->ui32KickTACounter;
			sNew.ui32Marker[1] = psDevInfo->ui32KickTARenderCounter;

			sNew.ui32Time[1] = psDevInfo->psSGXHostCtl->ui32TimeWraps;

			
			for (i = 0; i < PVRSRV_SGX_DIFF_NUM_COUNTERS; ++i)
			{
				psDiffs->aui32Counters[i] = sNew.aui32Counters[i] - psPrev->aui32Counters[i];
			}

			psDiffs->ui32Marker[0]			= sNew.ui32Marker[0] - psPrev->ui32Marker[0];
			psDiffs->ui32Marker[1]			= sNew.ui32Marker[1] - psPrev->ui32Marker[1];

			psDiffs->ui32Time[0]			= sNew.ui32Time[0] - psPrev->ui32Time[0];
			psDiffs->ui32Time[1]			= sNew.ui32Time[1] - psPrev->ui32Time[1];

			
			*psPrev = sNew;
		}
		else
		{
			
			for (i = 0; i < PVRSRV_SGX_DIFF_NUM_COUNTERS; ++i)
			{
				psDiffs->aui32Counters[i] = 0;
			}

			psDiffs->ui32Marker[0] = 0;
			psDiffs->ui32Marker[1] = 0;

			psDiffs->ui32Time[0] = 0;
			psDiffs->ui32Time[1] = 0;
		}
	}

	
	PVRSRVPowerUnlock(KERNEL_ID);

#if defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
	SGXTestActivePowerEvent(psDeviceNode, KERNEL_ID);
#endif 

	return eError;
}


IMG_EXPORT
PVRSRV_ERROR SGXReadHWPerfCBKM(IMG_HANDLE					hDevHandle,
							   IMG_UINT32					ui32ArraySize,
							   PVRSRV_SGX_HWPERF_CB_ENTRY	*psClientHWPerfEntry,
							   IMG_UINT32					*pui32DataCount,
							   IMG_UINT32					*pui32ClockSpeed,
							   IMG_UINT32					*pui32HostTimeStamp)
{
	PVRSRV_ERROR    	eError = PVRSRV_OK;
	PVRSRV_DEVICE_NODE	*psDeviceNode = hDevHandle;
	PVRSRV_SGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;
	SGXMKIF_HWPERF_CB	*psHWPerfCB = psDevInfo->psKernelHWPerfCBMemInfo->pvLinAddrKM;
	IMG_UINT			i;

	for (i = 0;
		 psHWPerfCB->ui32Woff != psHWPerfCB->ui32Roff && i < ui32ArraySize;
		 i++)
	{
		SGXMKIF_HWPERF_CB_ENTRY *psMKPerfEntry = &psHWPerfCB->psHWPerfCBData[psHWPerfCB->ui32Roff];

		psClientHWPerfEntry[i].ui32FrameNo = psMKPerfEntry->ui32FrameNo;
		psClientHWPerfEntry[i].ui32Type = psMKPerfEntry->ui32Type;
		psClientHWPerfEntry[i].ui32Ordinal	= psMKPerfEntry->ui32Ordinal;
		psClientHWPerfEntry[i].ui32Clocksx16 = SGXConvertTimeStamp(psDevInfo,
													psMKPerfEntry->ui32TimeWraps,
													psMKPerfEntry->ui32Time);
		OSMemCopy(&psClientHWPerfEntry[i].ui32Counters[0],
				  &psMKPerfEntry->ui32Counters[0],
				  sizeof(psMKPerfEntry->ui32Counters));

		psHWPerfCB->ui32Roff = (psHWPerfCB->ui32Roff + 1) & (SGXMKIF_HWPERF_CB_SIZE - 1);
	}

	*pui32DataCount = i;
	*pui32ClockSpeed = psDevInfo->ui32CoreClockSpeed;
	*pui32HostTimeStamp = OSClockus();

	return eError;
}
#else
#endif 


