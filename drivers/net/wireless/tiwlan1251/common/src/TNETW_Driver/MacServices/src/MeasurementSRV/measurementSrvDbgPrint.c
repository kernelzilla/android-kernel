/** \file measurementSrvDbgPrint.c
 *  \brief This file include variuos measurement SRV debug print facilities
 *  \author Ronen Kalish
 *  \date 23-December-2005
 */
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

#include "MeasurementSrvSM.h"
#include "MeasurementSrv.h"
#include "measurementSrvDbgPrint.h"
#include "report.h"

#ifdef TI_DBG    
static char bandDesc[ RADIO_BAND_NUM_OF_BANDS ][ MAX_DESC_STRING_LEN ] =
{
	"2.4 GHz",
	"5.0 GHz"
};

static char measurementTypeDesc[ MSR_TYPE_MAX_NUM_OF_MEASURE_TYPES ][ MAX_DESC_STRING_LEN ] =
{
	"basic measurement",
	"channel load measurement",
	"noise histogram measurement",
	"beacon measurement",
	"frame measurement"
};

static char measurementScanModeTypeDesc[ MSR_SCAN_MODE_MAX_NUM_OF_SCAN_MODES ][ MAX_DESC_STRING_LEN ] =
{
	"scan mode passive",
	"scan mode active",
	"scan mode beacon table"
};
#endif /* TI_DBG */

/** 
 * \author Ronen Kalish\n
 * \date 23-December-2005\n
 * \brief Prints a measurement request.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param pMsrRequest - the measurement request.\n
 */
void measurementSRVPrintRequest( TI_HANDLE hMeasurementSRV, measurement_request_t *pMsrRequest )
{
#ifdef TI_DBG    
	measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
	int i;

	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("Measurement request:\n") );
	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("band: %s, channel:%d, TX power level: %d, start time: %x-%x\n", 
							  bandDesc[ pMsrRequest->band ], pMsrRequest->channel, pMsrRequest->txPowerDbm, 
							  INT64_HIGHER( pMsrRequest->startTime ), INT64_LOWER( pMsrRequest->startTime )) );
	for ( i = 0; i < pMsrRequest->numberOfTypes; i++ )
	{
		measurementSRVPrintTypeRequest( hMeasurementSRV, &(pMsrRequest->msrTypes[ i ]) );
	}
#endif /* TI_DBG */

}

/** 
 * \author Ronen Kalish\n
 * \date 23-December-2005\n
 * \brief Prints a measurement type request.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param pMsrTypeRequest - the measurement type request.\n
 */
void measurementSRVPrintTypeRequest( TI_HANDLE hMeasurementSRV, measurement_typeRequest_t* pMsrTypeRequest )
{
#ifdef TI_DBG    
	measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;

	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("Measurement type request: type: %s, duration:%d, scan mode: %s, reserved: %d\n", 
							  measurementTypeDesc[ pMsrTypeRequest->msrType ], pMsrTypeRequest->duration,
							  measurementScanModeTypeDesc[ pMsrTypeRequest->scanMode ], pMsrTypeRequest->reserved) );
#endif /* TI_DBG */
}

