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
 *   MODULE:  whalCommon.h
 *   PURPOSE: common whal definitions
 * 
 ****************************************************************************/

#ifndef WHAL_COMMON_H
#define WHAL_COMMON_H

#include "osTIType.h"
/*#include "osApi.h"*/
/*#include "paramIn.h"*/
/*#include "paramOut.h"*/
#include "report.h"
#include "utils.h"
/*#include "ti_types.h"*/

/*
 * hal reports 
 */
#define WLAN_REPORT_REPLY			WLAN_REPORT_CONSOLE
#define HAL_HW_CTRL_MODULE_LOG		HAL_CTRL_MODULE_LOG
#define HAL_HW_RX_MODULE_LOG		HAL_RX_MODULE_LOG
#define HAL_HW_TX_MODULE_LOG		HAL_TX_MODULE_LOG
#define HAL_HW_DATA_MODULE_LOG		HAL_TX_MODULE_LOG
#define HAL_TEST_MODULE_LOG			HAL_CTRL_MODULE_LOG


#if defined(HAL_ON_WIN)

#ifdef HAL_ON_DRIVER
#define os_report	DbgPrint 
#endif


/* whal utilities */
int  whalUtils_ConvertHwRate			(UINT8 HwRate, UINT8 HwModulation, rate_e *AppRate, modulationType_e *AppModulation);
int  whalUtils_ConvertAppRatesBitmap (UINT32 AppRatesBitmap, UINT32 AppModulation, UINT16 *HwRatesBitmap);
int  whalUtils_ConvertAppRate        (UINT32 AppRate, UINT32 AppModulation, UINT8 *HwRate);
int  whalUtils_ConvertHwRatesBitmap  (UINT8 HwRatesBitmap, UINT8 HwModulation, UINT32 *AppRate, UINT32 *AppModulation);
void  whalUtils_ConvertBitmapToMaxRate(UINT16 HwRatesBitmap, UINT8 *HwModulation, UINT8 *HwRate);
int  whalUtils_FindHwModulationByDrvRate (UINT32 AppRate, UINT8 *HwModu, UINT8 preamble); 
#else
int  whalUtils_ConvertAppRate        (rate_e AppRate, UINT8 *HwRate);
int  whalUtils_FindHwModulationByDrvRate (rate_e AppRate, UINT8 *HwModu, UINT8 preamble); 
int  whalUtils_ConvertAppRatesBitmap (UINT32 AppRatesBitmap, UINT32 AppModulation, UINT16 *HwRatesBitmap);
#endif /*HAL_ON_WIN*/

UINT32  whalUtils_GwsiRate2DRV_RATE (UINT32 gwsiRate);
rateMask_e  whalUtils_GwsiRate2DRV_RATE_MASK (UINT32 gwsiRate);
UINT32  whalUtils_DRV_RATE2GwsiRate (UINT32 Rate);
UINT32  whalUtils_DRV_RATE_MASK2GwsiRate (rateMask_e rateMask);


/* User Callback for Queue */
typedef void (*CmdQueue_InterrogateCB_t )(TI_HANDLE objectHandle,UINT16 MboxStatus,void *InterrogateParamsBuf);
typedef void (*CmdQueue_CB_t )(TI_HANDLE objectHandle,UINT16 MboxStatus);
typedef void (*CmdQueue_GenericCB_t )(TI_HANDLE objectHandle, UINT16 CmdType, UINT16 CmdID, UINT32 aStatus);

 
#endif
