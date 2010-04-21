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
 *   MODULE:  whalRadio.c
 *   PURPOSE: Handle all radio aspects in the Hal
 *
 *  RSSI 
 *		Set RSSI params to the firmware
 *		Conversion function From/To RxLevel/RSSI
 *	TxPowerLevel
 *		Overide the default radioTxLevel table
 *	
 ****************************************************************************/
#include "whalCommon.h"
#include "whalCtrl_api.h"
#include "whalCtrl.h"
#include "whalSecurity.h"
#include "commonTypes.h"
#include "eventMbox_api.h"
#include "whalBus_Api.h"

/* Conversion Factors */
#define RADIA_BG_FACTOR         93
#define RADIA_ABG_FACTOR        93
#define RADIA_BG_CRT_FACTOR     98

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetRadioBand
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */

int whalCtrl_SetRadioBand (TI_HANDLE hWhalCtrl, radioBand_e RadioBand)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

	pWhalCtrl->pWhalParams->WlanParams.RadioBand = RadioBand;		
	
	return OK;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_convertRSSIToRxLevel
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
INT8 whalCtrl_convertRSSIToRxLevel(TI_HANDLE hWhalCtrl, INT32 rssiVal)
{
    WHAL_CTRL   *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;	
    INT8       rxLevel;

    switch(pWhalCtrl->pWhalParams->WlanParams.radioType)
    {
    case RADIO_MAXIM2820_ID:  /* Not Supported at the moment */
        rxLevel = 0;
        break;
    case RADIO_RFMD_ID:       /* Not Supported at the moment */
        rxLevel = 0;
        break;
    case RADIO_RADIA_BG_ID:    
        rxLevel = (INT8)(rssiVal + RADIA_BG_FACTOR);
        break;
    case RADIO_RADIA_ABG_ID:    
        rxLevel = (INT8)(rssiVal + RADIA_ABG_FACTOR);
        break;
    case RADIO_RADIA_BG_CRT_ID: 
        rxLevel = (INT8)(rssiVal + RADIA_BG_CRT_FACTOR);
        break;
    case RADIO_RADIA_WBR_ID: 
        rxLevel = (UINT8)(rssiVal + RADIA_BG_CRT_FACTOR);
        break;
    case RADIO_RADIA_DCR_ID: 
	case RADIO_RADIA_DCR_1251_ID:
        rxLevel = (UINT8)(rssiVal + RADIA_BG_CRT_FACTOR);
        break;
    default: 
        rxLevel = 0;
        WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
            ("whalCtrl_convertRSSIToRxLevel: Error - Unknown radio type!\n"));
        break;
    }

	return rxLevel;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_getRadioNumber
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
TI_STATUS whalCtrl_getRadioNumber(TI_HANDLE hWhalCtrl, UINT32 *outRadioType, UINT32 *outRadioNumber)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL*)hWhalCtrl;
	WlanParams_T *pWlanParams = whal_ParamsGetWlanParams(pWhalCtrl->pWhalParams); 
	UINT32 RadioType;

	*outRadioNumber = pWlanParams->radioType;

	switch (pWlanParams->radioType)
	{
		case RADIO_RADIA_BG_ID:
			RadioType = RADIA_BG;
			WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
									 ("Radio Type is : RADIA BG\n"));
			break;

		case RADIO_RADIA_ABG_ID:
			RadioType = RADIA_ABG;
			WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
									 ("Radio Type is : RADIA ABG\n"));
			break;

		case RADIO_RADIA_BG_CRT_ID:
			RadioType = RADIA_BG;
			WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
									 ("Radio Type is : RADIA BG (crt)\n"));
			break;

        case RADIO_RADIA_WBR_ID:
            RadioType = RADIA_ABG;
            WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                 ("Radio Type is : RADIO ABG (wbr)\n"));
            break;

        case RADIO_RADIA_DCR_ID:
		case RADIO_RADIA_DCR_1251_ID:
            RadioType = RADIA_ABG;
            WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                 ("Radio Type is : RADIO ABG (dcr)\n"));
            break;

		default:
			RadioType = UNKNOWN_RADIO_TYPE;
			WLAN_OS_REPORT (("FATAL ERR: Radio Type is : 0x%x - UNKNOWN\n", pWlanParams->radioType));
			break;
	}

	*outRadioType = RadioType;
	return OK;
}

