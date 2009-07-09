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
#include "services_headers.h"
#include "buffer_manager.h"
#include "sgxapi_km.h"
#include "sgxinfo.h"
#include "sgxinfokm.h"
#include "sysconfig.h"
#include "pdump_km.h"
#include "mmu.h"
#include "pvr_bridge_km.h"
#include "osfunc.h"
#include "pvr_debug.h"
#include "sgxutils.h"

#ifdef __linux__
#include <linux/tty.h>			
#else
#include <stdio.h>
#endif


#if defined(SYS_CUSTOM_POWERDOWN)
PVRSRV_ERROR SysPowerDownMISR(IMG_UINT32 ui32DeviceIndex, IMG_UINT32 ui32CallerID);
#endif


#if defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
IMG_VOID SGXTestActivePowerEvent (PVRSRV_DEVICE_NODE	*psDeviceNode,
								  IMG_UINT32			ui32CallerID)
{
	PVRSRV_ERROR		eError = PVRSRV_OK;
	PVRSRV_SGXDEV_INFO	*psDevInfo = psDeviceNode->pvDevice;
	SGXMKIF_HOST_CTL	*psSGXHostCtl = psDevInfo->psSGXHostCtl;

	if (((psSGXHostCtl->ui32InterruptFlags & PVRSRV_USSE_EDM_INTERRUPT_ACTIVE_POWER) != 0) &&
		((psSGXHostCtl->ui32InterruptClearFlags & PVRSRV_USSE_EDM_INTERRUPT_ACTIVE_POWER) == 0))
	{
		
		psSGXHostCtl->ui32InterruptClearFlags |= PVRSRV_USSE_EDM_INTERRUPT_ACTIVE_POWER;
		
		
		PDUMPSUSPEND();

#if defined(SYS_CUSTOM_POWERDOWN)
		


		eError = SysPowerDownMISR(psDeviceNode->sDevId.ui32DeviceIndex, ui32CallerID);
#else
		eError = PVRSRVSetDevicePowerStateKM(psDeviceNode->sDevId.ui32DeviceIndex,
											 PVRSRV_POWER_STATE_D3,
											 ui32CallerID, IMG_FALSE);
		if (eError == PVRSRV_OK)
		{
			
			psSGXHostCtl->ui32NumActivePowerEvents++;
			
			if ((psSGXHostCtl->ui32PowerStatus & PVRSRV_USSE_EDM_POWMAN_POWEROFF_RESTART_IMMEDIATE) != 0)
			{
				


				if (ui32CallerID == ISR_ID)
				{
					psDeviceNode->bReProcessDeviceCommandComplete = IMG_TRUE;
				}
				else
				{
					SGXScheduleProcessQueuesKM(psDeviceNode);
				}
			}
		}
#endif
		if (eError == PVRSRV_ERROR_RETRY)
		{
			

			psSGXHostCtl->ui32InterruptClearFlags &= ~PVRSRV_USSE_EDM_INTERRUPT_ACTIVE_POWER;
			eError = PVRSRV_OK;
		}

		
		PDUMPRESUME();
	}

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXTestActivePowerEvent error:%lu", eError));
	}
}
#endif 


#ifdef INLINE_IS_PRAGMA
#pragma inline(SGXAcquireKernelCCBSlot)
#endif
static INLINE SGXMKIF_COMMAND * SGXAcquireKernelCCBSlot(PVRSRV_SGX_CCB_INFO *psCCB)
{
	IMG_BOOL	bStart = IMG_FALSE;
	IMG_UINT32	uiStart = 0;

	
	do
	{
		if(((*psCCB->pui32WriteOffset + 1) & 255) != *psCCB->pui32ReadOffset)
		{
			return &psCCB->psCommands[*psCCB->pui32WriteOffset];
		}

		if (bStart == IMG_FALSE)
		{
			bStart = IMG_TRUE;
			uiStart = OSClockus();
		}
		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
	} while ((OSClockus() - uiStart) < MAX_HW_TIME_US);

	
	return IMG_NULL;
}

PVRSRV_ERROR SGXScheduleCCBCommand(PVRSRV_SGXDEV_INFO 	*psDevInfo,
								   SGXMKIF_COMMAND_TYPE	eCommandType,
								   SGXMKIF_COMMAND		*psCommandData,
								   IMG_UINT32			ui32CallerID,
								   IMG_UINT32			ui32PDumpFlags)
{
	PVRSRV_SGX_CCB_INFO *psKernelCCB;
	PVRSRV_ERROR eError = PVRSRV_OK;
	SGXMKIF_COMMAND *psSGXCommand;
#if defined(PDUMP)
	IMG_VOID *pvDumpCommand;
#else
	PVR_UNREFERENCED_PARAMETER(ui32CallerID);
	PVR_UNREFERENCED_PARAMETER(ui32PDumpFlags);
#endif

	psKernelCCB = psDevInfo->psKernelCCBInfo;

	psSGXCommand = SGXAcquireKernelCCBSlot(psKernelCCB);

	
	if(!psSGXCommand)
	{
		eError = PVRSRV_ERROR_TIMEOUT;
		goto Exit;
	}

	
	psCommandData->ui32Data[2] = psDevInfo->ui32CacheControl;

#if defined(PDUMP)
	
	psDevInfo->sPDContext.ui32CacheControl |= psDevInfo->ui32CacheControl;
#endif

	
	psDevInfo->ui32CacheControl = 0;

	
	*psSGXCommand = *psCommandData;

	switch(eCommandType)	
	{
		case SGXMKIF_COMMAND_EDM_KICK:
			psSGXCommand->ui32ServiceAddress = psDevInfo->ui32HostKickAddress;
			break;
 		case SGXMKIF_COMMAND_REQUEST_SGXMISCINFO:
 			psSGXCommand->ui32ServiceAddress = psDevInfo->ui32GetMiscInfoAddress;
 			break;
		case SGXMKIF_COMMAND_VIDEO_KICK:
		default:
			PVR_DPF((PVR_DBG_ERROR,"SGXScheduleCCBCommandKM: Unknown command type: %d", eCommandType)) ;
			eError = PVRSRV_ERROR_GENERIC;
			goto Exit;
	}

#if defined(PDUMP)
	if (ui32CallerID != ISR_ID)
	{
		
		PDUMPCOMMENTWITHFLAGS(0, "Poll for space in the Kernel CCB\r\n");
		PDUMPMEMPOL(psKernelCCB->psCCBCtlMemInfo, offsetof(PVRSRV_SGX_CCB_CTL, ui32ReadOffset), (psKernelCCB->ui32CCBDumpWOff + 1) & 0xff, 0xff, PDUMP_POLL_OPERATOR_NOTEQUAL, IMG_FALSE, IMG_FALSE, MAKEUNIQUETAG(psKernelCCB->psCCBCtlMemInfo));

		PDUMPCOMMENTWITHFLAGS(0, "Kernel CCB command\r\n");
		pvDumpCommand = (IMG_VOID *)((IMG_UINT8 *)psKernelCCB->psCCBMemInfo->pvLinAddrKM + (*psKernelCCB->pui32WriteOffset * sizeof(SGXMKIF_COMMAND)));

		PDUMPMEM(pvDumpCommand,
					psKernelCCB->psCCBMemInfo,
					psKernelCCB->ui32CCBDumpWOff * sizeof(SGXMKIF_COMMAND),
					sizeof(SGXMKIF_COMMAND),
					ui32PDumpFlags,
					MAKEUNIQUETAG(psKernelCCB->psCCBMemInfo));

		
		PDUMPMEM(&psDevInfo->sPDContext.ui32CacheControl,
					psKernelCCB->psCCBMemInfo,
					psKernelCCB->ui32CCBDumpWOff * sizeof(SGXMKIF_COMMAND) +
					offsetof(SGXMKIF_COMMAND, ui32Data[2]),
					sizeof(IMG_UINT32),
					ui32PDumpFlags,
					MAKEUNIQUETAG(psKernelCCB->psCCBMemInfo));

		if (PDumpIsCaptureFrameKM()
		|| ((ui32PDumpFlags & PDUMP_FLAGS_CONTINUOUS) != 0))
		{
			
			psDevInfo->sPDContext.ui32CacheControl = 0;
		}
	}
#endif

	

	*psKernelCCB->pui32WriteOffset = (*psKernelCCB->pui32WriteOffset + 1) & 255;

#if defined(PDUMP)
	if (ui32CallerID != ISR_ID)
	{
		if (PDumpIsCaptureFrameKM()
		|| ((ui32PDumpFlags & PDUMP_FLAGS_CONTINUOUS) != 0))
		{
			psKernelCCB->ui32CCBDumpWOff = (psKernelCCB->ui32CCBDumpWOff + 1) & 0xFF;
		}

		PDUMPCOMMENTWITHFLAGS(0, "Kernel CCB write offset\r\n");
		PDUMPMEM(&psKernelCCB->ui32CCBDumpWOff,
				 psKernelCCB->psCCBCtlMemInfo,
				 offsetof(PVRSRV_SGX_CCB_CTL, ui32WriteOffset),
				 sizeof(IMG_UINT32),
				 ui32PDumpFlags,
				 MAKEUNIQUETAG(psKernelCCB->psCCBCtlMemInfo));
		PDUMPCOMMENTWITHFLAGS(0, "Kernel CCB event kicker\r\n");
		PDUMPMEM(&psKernelCCB->ui32CCBDumpWOff,
				 psDevInfo->psKernelCCBEventKickerMemInfo,
				 0,
				 sizeof(IMG_UINT32),
				 ui32PDumpFlags,
				 MAKEUNIQUETAG(psDevInfo->psKernelCCBEventKickerMemInfo));
		PDUMPCOMMENTWITHFLAGS(0, "Event kick\r\n");
		PDUMPREGWITHFLAGS(SGX_MP_CORE_SELECT(EUR_CR_EVENT_KICK, 0), EUR_CR_EVENT_KICK_NOW_MASK, 0);
	}
#endif

	*psDevInfo->pui32KernelCCBEventKicker = (*psDevInfo->pui32KernelCCBEventKicker + 1) & 0xFF;
	OSWriteHWReg(psDevInfo->pvRegsBaseKM,
				SGX_MP_CORE_SELECT(EUR_CR_EVENT_KICK, 0),
				EUR_CR_EVENT_KICK_NOW_MASK);

#if defined(NO_HARDWARE)
	
	*psKernelCCB->pui32ReadOffset = (*psKernelCCB->pui32ReadOffset + 1) & 255;
#endif

Exit:
	return eError;
}


PVRSRV_ERROR SGXScheduleCCBCommandKM(PVRSRV_DEVICE_NODE		*psDeviceNode,
									 SGXMKIF_COMMAND_TYPE	eCommandType,
									 SGXMKIF_COMMAND		*psCommandData,
									 IMG_UINT32				ui32CallerID,
									 IMG_UINT32				ui32PDumpFlags)
{
	PVRSRV_ERROR		eError;
	PVRSRV_SGXDEV_INFO 	*psDevInfo = psDeviceNode->pvDevice;

	
	PDUMPSUSPEND();

	
	eError = PVRSRVSetDevicePowerStateKM(psDeviceNode->sDevId.ui32DeviceIndex,
										 PVRSRV_POWER_STATE_D0,
										 ui32CallerID,
										 IMG_TRUE);

	PDUMPRESUME();

	if (eError == PVRSRV_OK)
	{
		psDeviceNode->bReProcessDeviceCommandComplete = IMG_FALSE;
	}
	else
	{
		if (eError == PVRSRV_ERROR_RETRY)
		{
			if (ui32CallerID == ISR_ID)
			{
				


				psDeviceNode->bReProcessDeviceCommandComplete = IMG_TRUE;
				eError = PVRSRV_OK;
			}
			else
			{
				

			}
		}
		else
		{
			PVR_DPF((PVR_DBG_ERROR,"SGXScheduleCCBCommandKM failed to acquire lock - "
					 "ui32CallerID:%ld eError:%lu", ui32CallerID, eError));
		}

		return eError;
	}

	eError = SGXScheduleCCBCommand(psDevInfo, eCommandType, psCommandData, ui32CallerID, ui32PDumpFlags);

	PVRSRVPowerUnlock(ui32CallerID);

#if defined(SUPPORT_ACTIVE_POWER_MANAGEMENT)
	if (ui32CallerID != ISR_ID)
	{
		


		SGXTestActivePowerEvent(psDeviceNode, ui32CallerID);
	}
#endif 

	return eError;
}


PVRSRV_ERROR SGXScheduleProcessQueuesKM(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_ERROR 		eError;
	PVRSRV_SGXDEV_INFO 	*psDevInfo = psDeviceNode->pvDevice;
	SGXMKIF_HOST_CTL	*psHostCtl = psDevInfo->psKernelSGXHostCtlMemInfo->pvLinAddrKM;
	IMG_UINT32			ui32PowerStatus;
	SGXMKIF_COMMAND		sCommand = {0};

	ui32PowerStatus = psHostCtl->ui32PowerStatus;
	if ((ui32PowerStatus & PVRSRV_USSE_EDM_POWMAN_NO_WORK) != 0)
	{
		
		return PVRSRV_OK;
	}

	sCommand.ui32Data[0] = PVRSRV_CCBFLAGS_PROCESS_QUEUESCMD;
	eError = SGXScheduleCCBCommandKM(psDeviceNode, SGXMKIF_COMMAND_EDM_KICK, &sCommand, ISR_ID, 0);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"SGXScheduleProcessQueuesKM failed to schedule CCB command: %lu", eError));
		return PVRSRV_ERROR_GENERIC;
	}

	return PVRSRV_OK;
}


IMG_BOOL SGXIsDevicePowered(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	return PVRSRVIsDevicePowered(psDeviceNode->sDevId.ui32DeviceIndex);
}

IMG_EXPORT
PVRSRV_ERROR SGXGetInternalDevInfoKM(IMG_HANDLE hDevCookie,
									SGX_INTERNAL_DEVINFO *psSGXInternalDevInfo)
{
	PVRSRV_SGXDEV_INFO *psDevInfo = (PVRSRV_SGXDEV_INFO *)((PVRSRV_DEVICE_NODE *)hDevCookie)->pvDevice;

	psSGXInternalDevInfo->ui32Flags = psDevInfo->ui32Flags;
	psSGXInternalDevInfo->bForcePTOff = (IMG_BOOL)psDevInfo->bForcePTOff;

	
	psSGXInternalDevInfo->hHostCtlKernelMemInfoHandle =
		(IMG_HANDLE)psDevInfo->psKernelSGXHostCtlMemInfo;

	return PVRSRV_OK;
}


#if defined (PDUMP) && !defined(EDM_USSE_HWDEBUG) 
#define PDUMP_SGX_CLEANUP
#endif

IMG_VOID SGXCleanupRequest(PVRSRV_DEVICE_NODE	*psDeviceNode,
								  IMG_DEV_VIRTADDR		*psHWDataDevVAddr,
								  IMG_UINT32			ui32ResManRequestFlag)
{
	PVRSRV_SGXDEV_INFO		*psSGXDevInfo = (PVRSRV_SGXDEV_INFO *)psDeviceNode->pvDevice;
	PVRSRV_KERNEL_MEM_INFO	*psSGXHostCtlMemInfo = psSGXDevInfo->psKernelSGXHostCtlMemInfo;
	SGXMKIF_HOST_CTL		*psSGXHostCtl = (SGXMKIF_HOST_CTL *)psSGXHostCtlMemInfo->pvLinAddrKM;
#if defined(PDUMP_SGX_CLEANUP)
	IMG_HANDLE hUniqueTag = MAKEUNIQUETAG(psSGXHostCtlMemInfo);
#endif

	if ((psSGXHostCtl->ui32PowerStatus & PVRSRV_USSE_EDM_POWMAN_NO_WORK) != 0)
	{
		
	}
	else
	{
		
		if (psSGXDevInfo->ui32CacheControl & SGX_BIF_INVALIDATE_PDCACHE)
		{
			psSGXHostCtl->ui32ResManFlags |= PVRSRV_USSE_EDM_RESMAN_CLEANUP_INVALPD;
			psSGXDevInfo->ui32CacheControl ^= SGX_BIF_INVALIDATE_PDCACHE;
		}
		if (psSGXDevInfo->ui32CacheControl & SGX_BIF_INVALIDATE_PTCACHE)
		{
			psSGXHostCtl->ui32ResManFlags |= PVRSRV_USSE_EDM_RESMAN_CLEANUP_INVALPT;
			psSGXDevInfo->ui32CacheControl ^= SGX_BIF_INVALIDATE_PTCACHE;
		}

		if(psHWDataDevVAddr == IMG_NULL)
		{
			psSGXHostCtl->sResManCleanupData.uiAddr = 0;
		}
		else
		{
			
			psSGXHostCtl->sResManCleanupData.uiAddr = psHWDataDevVAddr->uiAddr;
		}

		
		psSGXHostCtl->ui32ResManFlags |= ui32ResManRequestFlag;

#if defined(PDUMP_SGX_CLEANUP)
		
		PDUMPCOMMENTWITHFLAGS(0, "TA/3D CCB Control - Request clean-up event on uKernel...");
		PDUMPMEM(IMG_NULL, psSGXHostCtlMemInfo, offsetof(SGXMKIF_HOST_CTL, sResManCleanupData.uiAddr), sizeof(IMG_UINT32), 0, hUniqueTag);
		PDUMPMEM(IMG_NULL, psSGXHostCtlMemInfo, offsetof(SGXMKIF_HOST_CTL, ui32ResManFlags), sizeof(IMG_UINT32), 0, hUniqueTag);
#else
		PDUMPCOMMENTWITHFLAGS(0, "Clean-up event on uKernel disabled");
#endif 

		
		SGXScheduleProcessQueuesKM(psDeviceNode);

		
		#if !defined(NO_HARDWARE)
		if(PollForValueKM ((volatile IMG_UINT32 *)(&psSGXHostCtl->ui32ResManFlags),
					PVRSRV_USSE_EDM_RESMAN_CLEANUP_COMPLETE,
					PVRSRV_USSE_EDM_RESMAN_CLEANUP_COMPLETE,
					MAX_HW_TIME_US/WAIT_TRY_COUNT,
					WAIT_TRY_COUNT) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"SGXCleanupRequest: Wait for uKernel to clean up failed"));
			PVR_DBG_BREAK;
		}
		#endif

		#if defined(PDUMP_SGX_CLEANUP)
		
		PDUMPCOMMENTWITHFLAGS(0, "TA/3D CCB Control - Wait for clean-up request to complete...");
		PDUMPMEMPOL(psSGXHostCtlMemInfo,
					   offsetof(SGXMKIF_HOST_CTL, ui32ResManFlags),
					   PVRSRV_USSE_EDM_RESMAN_CLEANUP_COMPLETE,
					   PVRSRV_USSE_EDM_RESMAN_CLEANUP_COMPLETE,
					   PDUMP_POLL_OPERATOR_EQUAL,
					   IMG_FALSE, IMG_FALSE,
					   hUniqueTag);
		#endif 

		psSGXHostCtl->ui32ResManFlags &= ~(ui32ResManRequestFlag);
		psSGXHostCtl->ui32ResManFlags &= ~(PVRSRV_USSE_EDM_RESMAN_CLEANUP_COMPLETE);
#if defined(PDUMP_SGX_CLEANUP)
		PDUMPMEM(IMG_NULL, psSGXHostCtlMemInfo, offsetof(SGXMKIF_HOST_CTL, ui32ResManFlags), sizeof(IMG_UINT32), 0, hUniqueTag);
#endif 
	}
}

typedef struct _SGX_HW_RENDER_CONTEXT_CLEANUP_
{
	PVRSRV_DEVICE_NODE *psDeviceNode;
	IMG_DEV_VIRTADDR sHWRenderContextDevVAddr;
	IMG_HANDLE hBlockAlloc;
	PRESMAN_ITEM psResItem;
} SGX_HW_RENDER_CONTEXT_CLEANUP;


static PVRSRV_ERROR SGXCleanupHWRenderContextCallback(IMG_PVOID		pvParam,
													  IMG_UINT32	ui32Param)
{
	SGX_HW_RENDER_CONTEXT_CLEANUP *psCleanup = pvParam;

	PVR_UNREFERENCED_PARAMETER(ui32Param);

	SGXCleanupRequest(psCleanup->psDeviceNode,
					  &psCleanup->sHWRenderContextDevVAddr,
					  PVRSRV_USSE_EDM_RESMAN_CLEANUP_RC_REQUEST);

	OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP,
			  sizeof(SGX_HW_RENDER_CONTEXT_CLEANUP),
			  psCleanup,
			  psCleanup->hBlockAlloc);

	return PVRSRV_OK;
}

typedef struct _SGX_HW_TRANSFER_CONTEXT_CLEANUP_
{
	PVRSRV_DEVICE_NODE *psDeviceNode;
	IMG_DEV_VIRTADDR sHWTransferContextDevVAddr;
	IMG_HANDLE hBlockAlloc;
	PRESMAN_ITEM psResItem;
} SGX_HW_TRANSFER_CONTEXT_CLEANUP;


static PVRSRV_ERROR SGXCleanupHWTransferContextCallback(IMG_PVOID	pvParam,
														IMG_UINT32	ui32Param)
{
	SGX_HW_TRANSFER_CONTEXT_CLEANUP *psCleanup = (SGX_HW_TRANSFER_CONTEXT_CLEANUP *)pvParam;

	PVR_UNREFERENCED_PARAMETER(ui32Param);

	SGXCleanupRequest(psCleanup->psDeviceNode,
							&psCleanup->sHWTransferContextDevVAddr, PVRSRV_USSE_EDM_RESMAN_CLEANUP_TC_REQUEST);

	OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP,
			  sizeof(SGX_HW_TRANSFER_CONTEXT_CLEANUP),
			  psCleanup,
			  psCleanup->hBlockAlloc);

	return PVRSRV_OK;
}

IMG_EXPORT
IMG_HANDLE SGXRegisterHWRenderContextKM(IMG_HANDLE				psDeviceNode,
										IMG_DEV_VIRTADDR		*psHWRenderContextDevVAddr,
										PVRSRV_PER_PROCESS_DATA *psPerProc)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hBlockAlloc;
	SGX_HW_RENDER_CONTEXT_CLEANUP *psCleanup;
	PRESMAN_ITEM psResItem;

	eError = OSAllocMem(PVRSRV_OS_PAGEABLE_HEAP,
						sizeof(SGX_HW_RENDER_CONTEXT_CLEANUP),
						(IMG_VOID **)&psCleanup,
						&hBlockAlloc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXRegisterHWRenderContextKM: Couldn't allocate memory for SGX_HW_RENDER_CONTEXT_CLEANUP structure"));
		return IMG_NULL;
	}

	psCleanup->hBlockAlloc = hBlockAlloc;
	psCleanup->psDeviceNode = psDeviceNode;
	psCleanup->sHWRenderContextDevVAddr = *psHWRenderContextDevVAddr;

	psResItem = ResManRegisterRes(psPerProc->hResManContext,
								  RESMAN_TYPE_HW_RENDER_CONTEXT,
								  (IMG_VOID *)psCleanup,
								  0,
								  &SGXCleanupHWRenderContextCallback);

	if (psResItem == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXRegisterHWRenderContextKM: ResManRegisterRes failed"));
		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP,
				  sizeof(SGX_HW_RENDER_CONTEXT_CLEANUP),
				  psCleanup,
				  psCleanup->hBlockAlloc);

		return IMG_NULL;
	}

	psCleanup->psResItem = psResItem;

	return (IMG_HANDLE)psCleanup;
}

IMG_EXPORT
PVRSRV_ERROR SGXUnregisterHWRenderContextKM(IMG_HANDLE hHWRenderContext)
{
	PVRSRV_ERROR eError;
	SGX_HW_RENDER_CONTEXT_CLEANUP *psCleanup;

	PVR_ASSERT(hHWRenderContext != IMG_NULL);

	psCleanup = (SGX_HW_RENDER_CONTEXT_CLEANUP *)hHWRenderContext;

	if (psCleanup == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXUnregisterHWRenderContextKM: invalid parameter"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	eError = ResManFreeResByPtr(psCleanup->psResItem);

	return eError;
}


IMG_EXPORT
IMG_HANDLE SGXRegisterHWTransferContextKM(IMG_HANDLE				psDeviceNode,
										  IMG_DEV_VIRTADDR			*psHWTransferContextDevVAddr,
										  PVRSRV_PER_PROCESS_DATA	*psPerProc)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hBlockAlloc;
	SGX_HW_TRANSFER_CONTEXT_CLEANUP *psCleanup;
	PRESMAN_ITEM psResItem;

	eError = OSAllocMem(PVRSRV_OS_PAGEABLE_HEAP,
						sizeof(SGX_HW_TRANSFER_CONTEXT_CLEANUP),
						(IMG_VOID **)&psCleanup,
						&hBlockAlloc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXRegisterHWTransferContextKM: Couldn't allocate memory for SGX_HW_TRANSFER_CONTEXT_CLEANUP structure"));
		return IMG_NULL;
	}

	psCleanup->hBlockAlloc = hBlockAlloc;
	psCleanup->psDeviceNode = psDeviceNode;
	psCleanup->sHWTransferContextDevVAddr = *psHWTransferContextDevVAddr;

	psResItem = ResManRegisterRes(psPerProc->hResManContext,
								  RESMAN_TYPE_HW_TRANSFER_CONTEXT,
								  psCleanup,
								  0,
								  &SGXCleanupHWTransferContextCallback);

	if (psResItem == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXRegisterHWTransferContextKM: ResManRegisterRes failed"));
		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP,
				  sizeof(SGX_HW_TRANSFER_CONTEXT_CLEANUP),
				  psCleanup,
				  psCleanup->hBlockAlloc);

		return IMG_NULL;
	}

	psCleanup->psResItem = psResItem;

	return (IMG_HANDLE)psCleanup;
}

IMG_EXPORT
PVRSRV_ERROR SGXUnregisterHWTransferContextKM(IMG_HANDLE hHWTransferContext)
{
	PVRSRV_ERROR eError;
	SGX_HW_TRANSFER_CONTEXT_CLEANUP *psCleanup;

	PVR_ASSERT(hHWTransferContext != IMG_NULL);

	psCleanup = (SGX_HW_TRANSFER_CONTEXT_CLEANUP *)hHWTransferContext;

	if (psCleanup == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXUnregisterHWTransferContextKM: invalid parameter"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	eError = ResManFreeResByPtr(psCleanup->psResItem);

	return eError;
}

#if defined(SGX_FEATURE_2D_HARDWARE)
typedef struct _SGX_HW_2D_CONTEXT_CLEANUP_
{
	PVRSRV_DEVICE_NODE *psDeviceNode;
	IMG_DEV_VIRTADDR sHW2DContextDevVAddr;
	IMG_HANDLE hBlockAlloc;
	PRESMAN_ITEM psResItem;
} SGX_HW_2D_CONTEXT_CLEANUP;

static PVRSRV_ERROR SGXCleanupHW2DContextCallback(IMG_PVOID pvParam, IMG_UINT32 ui32Param)
{
	SGX_HW_2D_CONTEXT_CLEANUP *psCleanup = (SGX_HW_2D_CONTEXT_CLEANUP *)pvParam;

	PVR_UNREFERENCED_PARAMETER(ui32Param);

	SGXCleanupRequest(psCleanup->psDeviceNode,
							&psCleanup->sHW2DContextDevVAddr, PVRSRV_USSE_EDM_RESMAN_CLEANUP_2DC_REQUEST);

	OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP,
			  sizeof(SGX_HW_2D_CONTEXT_CLEANUP),
			  psCleanup,
			  psCleanup->hBlockAlloc);

	return PVRSRV_OK;
}

IMG_EXPORT
IMG_HANDLE SGXRegisterHW2DContextKM(IMG_HANDLE				psDeviceNode,
									IMG_DEV_VIRTADDR		*psHW2DContextDevVAddr,
									PVRSRV_PER_PROCESS_DATA *psPerProc)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hBlockAlloc;
	SGX_HW_2D_CONTEXT_CLEANUP *psCleanup;
	PRESMAN_ITEM psResItem;

	eError = OSAllocMem(PVRSRV_OS_PAGEABLE_HEAP,
						sizeof(SGX_HW_2D_CONTEXT_CLEANUP),
						(IMG_VOID **)&psCleanup,
						&hBlockAlloc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXRegisterHW2DContextKM: Couldn't allocate memory for SGX_HW_2D_CONTEXT_CLEANUP structure"));
		return IMG_NULL;
	}

	psCleanup->hBlockAlloc = hBlockAlloc;
	psCleanup->psDeviceNode = psDeviceNode;
	psCleanup->sHW2DContextDevVAddr = *psHW2DContextDevVAddr;

	psResItem = ResManRegisterRes(psPerProc->hResManContext,
								  RESMAN_TYPE_HW_2D_CONTEXT,
								  psCleanup,
								  0,
								  &SGXCleanupHW2DContextCallback);

	if (psResItem == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "SGXRegisterHW2DContextKM: ResManRegisterRes failed"));
		OSFreeMem(PVRSRV_OS_PAGEABLE_HEAP,
				  sizeof(SGX_HW_2D_CONTEXT_CLEANUP),
				  psCleanup,
				  psCleanup->hBlockAlloc);

		return IMG_NULL;
	}

	psCleanup->psResItem = psResItem;

	return (IMG_HANDLE)psCleanup;
}

IMG_EXPORT
PVRSRV_ERROR SGXUnregisterHW2DContextKM(IMG_HANDLE hHW2DContext)
{
	PVRSRV_ERROR eError;
	SGX_HW_2D_CONTEXT_CLEANUP *psCleanup;

	PVR_ASSERT(hHW2DContext != IMG_NULL);

	if (hHW2DContext == IMG_NULL)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psCleanup = (SGX_HW_2D_CONTEXT_CLEANUP *)hHW2DContext;

	eError = ResManFreeResByPtr(psCleanup->psResItem);

	return eError;
}
#endif 

#ifdef INLINE_IS_PRAGMA
#pragma inline(SGX2DQuerySyncOpsComplete)
#endif
static INLINE
IMG_BOOL SGX2DQuerySyncOpsComplete(PVRSRV_KERNEL_SYNC_INFO	*psSyncInfo,
								   IMG_UINT32				ui32ReadOpsPending,
								   IMG_UINT32				ui32WriteOpsPending)
{
	PVRSRV_SYNC_DATA *psSyncData = psSyncInfo->psSyncData;

	return (IMG_BOOL)(
					  (psSyncData->ui32ReadOpsComplete >= ui32ReadOpsPending) &&
					  (psSyncData->ui32WriteOpsComplete >= ui32WriteOpsPending)
					 );
}

IMG_EXPORT
PVRSRV_ERROR SGX2DQueryBlitsCompleteKM(PVRSRV_SGXDEV_INFO	*psDevInfo,
									   PVRSRV_KERNEL_SYNC_INFO *psSyncInfo,
									   IMG_BOOL bWaitForComplete)
{
	IMG_BOOL	bStart = IMG_FALSE;
	IMG_UINT32	uiStart = 0;
	IMG_UINT32	ui32ReadOpsPending, ui32WriteOpsPending;

	PVR_UNREFERENCED_PARAMETER(psDevInfo);

	PVR_DPF((PVR_DBG_CALLTRACE, "SGX2DQueryBlitsCompleteKM: Start"));

	ui32ReadOpsPending = psSyncInfo->psSyncData->ui32ReadOpsPending;
	ui32WriteOpsPending = psSyncInfo->psSyncData->ui32WriteOpsPending;

	if(SGX2DQuerySyncOpsComplete(psSyncInfo, ui32ReadOpsPending, ui32WriteOpsPending))
	{
		
		PVR_DPF((PVR_DBG_CALLTRACE, "SGX2DQueryBlitsCompleteKM: No wait. Blits complete."));
		return PVRSRV_OK;
	}

	
	if (!bWaitForComplete)
	{
		
		PVR_DPF((PVR_DBG_CALLTRACE, "SGX2DQueryBlitsCompleteKM: No wait. Ops pending."));
		return PVRSRV_ERROR_CMD_NOT_PROCESSED;
	}

	
	PVR_DPF((PVR_DBG_MESSAGE, "SGX2DQueryBlitsCompleteKM: Ops pending. Start polling."));
	do
	{
		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);

		if(SGX2DQuerySyncOpsComplete(psSyncInfo, ui32ReadOpsPending, ui32WriteOpsPending))
		{
			
			PVR_DPF((PVR_DBG_CALLTRACE, "SGX2DQueryBlitsCompleteKM: Wait over.  Blits complete."));
			return PVRSRV_OK;
		}

		if (bStart == IMG_FALSE)
		{
			uiStart = OSClockus();
			bStart = IMG_TRUE;
		}

		OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
	} while ((OSClockus() - uiStart) < MAX_HW_TIME_US);

	
	PVR_DPF((PVR_DBG_ERROR,"SGX2DQueryBlitsCompleteKM: Timed out. Ops pending."));

#if defined(DEBUG)
	{
		PVRSRV_SYNC_DATA *psSyncData = psSyncInfo->psSyncData;

		PVR_TRACE(("SGX2DQueryBlitsCompleteKM: Syncinfo: %p, Syncdata: %p", psSyncInfo, psSyncData));

		PVR_TRACE(("SGX2DQueryBlitsCompleteKM: Read ops complete: %d, Read ops pending: %d", psSyncData->ui32ReadOpsComplete, psSyncData->ui32ReadOpsPending));
		PVR_TRACE(("SGX2DQueryBlitsCompleteKM: Write ops complete: %d, Write ops pending: %d", psSyncData->ui32WriteOpsComplete, psSyncData->ui32WriteOpsPending));

	}
#endif

	return PVRSRV_ERROR_TIMEOUT;
}


IMG_EXPORT
IMG_VOID SGXFlushHWRenderTargetKM(IMG_HANDLE psDeviceNode, IMG_DEV_VIRTADDR sHWRTDataSetDevVAddr)
{
	PVR_ASSERT(sHWRTDataSetDevVAddr.uiAddr != IMG_NULL);

	SGXCleanupRequest((PVRSRV_DEVICE_NODE *)psDeviceNode, &sHWRTDataSetDevVAddr, PVRSRV_USSE_EDM_RESMAN_CLEANUP_RT_REQUEST);
}


IMG_UINT32 SGXConvertTimeStamp(PVRSRV_SGXDEV_INFO	*psDevInfo,
							   IMG_UINT32			ui32TimeWraps,
							   IMG_UINT32			ui32Time)
{
	IMG_UINT64	ui64Clocks;
	IMG_UINT32	ui32Clocksx16;

	ui64Clocks = ((IMG_UINT64)ui32TimeWraps * psDevInfo->ui32uKernelTimerClock) +
					(psDevInfo->ui32uKernelTimerClock - (ui32Time & EUR_CR_EVENT_TIMER_VALUE_MASK));
	ui32Clocksx16 = (IMG_UINT32)(ui64Clocks / 16);

	return ui32Clocksx16;
}



