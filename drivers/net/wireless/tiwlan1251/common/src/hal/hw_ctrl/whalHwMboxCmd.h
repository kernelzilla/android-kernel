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

/****************************************************************************
 *
 *   MODULE:  whalHwMboxCmd.h
 *   PURPOSE: wlan hardware commands handler
 * 
 ****************************************************************************/

#ifndef _WHAL_HW_MBOX_CMD_H
#define _WHAL_HW_MBOX_CMD_H


#include "whalCommon.h"
#include "whalParams.h"
#include "public_infoele.h"

typedef struct _HwMboxCmd_T
{
	TI_HANDLE		hCmdMboxQueue;
	WhalParams_T	*pWhalParams;

	TI_HANDLE		hOs;
	TI_HANDLE		hReport;
    
} HwMboxCmd_T;

extern HwMboxCmd_T *whal_hwMboxCmd_Create(TI_HANDLE hOs, WhalParams_T *pWhalParams);
extern int whal_hwMboxCmd_Destroy		(HwMboxCmd_T *pHwMboxCmd);
extern int whal_hwMboxCmd_Config		(HwMboxCmd_T *pHwMboxCmd,TI_HANDLE hCmdMboxQueue, TI_HANDLE hReport);
extern int whal_hwMboxCmd_Reset				(HwMboxCmd_T *pHwMboxCmd);
extern int whal_hwMboxCmd_RxReset			(HwMboxCmd_T *pHwMboxCmd);
extern int whal_hwMboxCmd_StartBss          (HwMboxCmd_T *pHwMboxCmd, BSS_e BssType, void *JoinCompleteCB, TI_HANDLE CB_handle);
extern int whal_hwMboxCmd_EnableRx			(HwMboxCmd_T *pHwMboxCmd);
extern int whal_hwMboxCmd_LNAControl		(HwMboxCmd_T *pHwMboxCmd, UINT8 LNAControlField);
extern int whal_hwMboxCmd_EnableTx			(HwMboxCmd_T *pHwMboxCmd,UINT8 channel);
extern int whal_hwMboxCmd_DisableRx			(HwMboxCmd_T *pHwMboxCmd);
extern int whal_hwMboxCmd_DisableTx			(HwMboxCmd_T *pHwMboxCmd);
extern int whal_hwMboxCmd_InitMemory		(HwMboxCmd_T *pHwMboxCmd);
extern int whal_hwMboxCmd_ConfigureTemplateFrame(HwMboxCmd_T *pHwMboxCmd, UINT8 *pFrame, UINT16 FrameSize,
                                                 Command_e templateType, void *CBFunc,TI_HANDLE CBObj);
extern int whal_hwMboxCmd_TimTemplate		(HwMboxCmd_T *pHwMboxCmd, char BmapControl, char *PartialBmapVec, int PartialBmapLen);
extern int whal_hwMboxCmd_StartScan         (HwMboxCmd_T *pHwMboxCmd, ScanParameters_t* pScanParams ,void* ScanCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwMboxCmd_StartSPSScan      (HwMboxCmd_T *pHwMboxCmd, ScheduledScanParameters_t* pScanParams, void* ScanCommandResponseCB, TI_HANDLE CB_handle);

extern int whal_hwMboxCmd_StopScan			(HwMboxCmd_T *pHwMboxCmd, void *ScanCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwMboxCmd_StopSPSScan       (HwMboxCmd_T *pHwMboxCmd, void* ScanCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwMboxCmd_GenCmd			(HwMboxCmd_T *pHwMboxCmd,short CmdId, char* pBuf, UINT32 Length);
extern int whal_hwMboxCmd_WriteMemory		(HwMboxCmd_T *pHwMboxCmd, UINT32 Address, UINT32 Size,PVOID  pValue);
extern int whal_hwMboxCmd_NoiseHistogramCmd (HwMboxCmd_T *pHwMboxCmd, whalCtrl_noiseHistogram_t* pNoiseHistParams);

extern int whal_hwMboxCmd_SwitchChannelCmd (HwMboxCmd_T *pHwMboxCmd, whalCtrl_switchChannelCmd_t *pSwitchChannelCmd);
extern int whal_hwMboxCmd_SwitchChannelCancelCmd (HwMboxCmd_T *pHwMboxCmd);
extern int whal_hwMboxCmd_SetKey(HwMboxCmd_T *pHwMboxCmd, int Action, char *MacAddr, int KeySize, 
				int KeyType, int KeyId, char *Key, UINT16 SecuritySeqNumLow, UINT32 SecuritySeqNumHigh,
				void *CB_Func, TI_HANDLE CB_handle);
extern int whal_hwMboxCmd_PowerMgmtConfiguration (HwMboxCmd_T *pHwMboxCmd, whalCtrl_powerSaveParams_t* powerSaveParams);
/*	whalCtrl_powerMgmtConfig_t* pPowerMgmtParams); */
extern int whal_hwMboxCmd_StartNewScan (HwMboxCmd_T *pHwMboxCmd, whalCtrl_scan_t* pScanParams);
extern int whal_hwMboxCmd_FwDisconnect(HwMboxCmd_T *pHwMboxCmd, uint32 ConfigOptions, uint32 FilterOptions);

extern int whal_hwMboxCmd_measurement (HwMboxCmd_T *pHwMboxCmd, whalCtrl_MeasurementParameters_t* pMeasurementParams,
									   void* MeasureCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwMboxCmd_measurementStop (HwMboxCmd_T *pHwMboxCmd, void* MeasureCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwMboxCmd_ApDiscovery (HwMboxCmd_T *pHwMboxCmd, whalCtrl_ApDiscoveryParameters_t* pMeasurementParams);
extern int whal_hwMboxCmd_ApDiscoveryStop (HwMboxCmd_T *pHwMboxCmd);
extern int whal_hwMboxCmd_HealthCheck(HwMboxCmd_T *pHwMboxCmd);


#endif
