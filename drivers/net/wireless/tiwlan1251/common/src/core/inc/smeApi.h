/** \file smeMgr.h
 *  \brief SME interface
 *
 *  
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

/****************************************************************************/
/*																			*/
/*		MODULE:		smeApi.h												*/
/*		PURPOSE:	SME interface to Other core modules						*/
/*																			*/
/****************************************************************************/
#ifndef __SME_API_H__
#define __SME_API_H__

#include "osTIType.h"
#include "paramOut.h"
#include "paramIn.h"
#include "mlmeApi.h"

/* Typedefs */
typedef enum
{
	NO_MATCH = 0,
	MATCH =		1,
} match_e;


/* Prototypes */

TI_STATUS conn_reportMlmeStatus(TI_HANDLE			hConn, 
							mgmtStatus_e			status,
							UINT16					uStatusCode);

TI_STATUS conn_reportRsnStatus(TI_HANDLE			hConn, 
							mgmtStatus_e		status);

TI_STATUS siteMgr_updateSite(TI_HANDLE			hSiteMgr, 
						  macAddress_t		*bssid, 
						  mlmeFrameInfo_t	*pFrameInfo,
						  UINT8				rxChannel,
                          radioBand_e       band,
						  BOOL				measuring);

TI_STATUS siteMgr_CheckRxSignalValidity(TI_HANDLE  hSiteMgr,
                         INT8              	rxLevel,
						 UINT8				channel,
                         macAddress_t       *bssid);

TI_STATUS siteMgr_updateRxSignal(TI_HANDLE		hSiteMgr,
						 UINT8				snr, 
						 INT8				rxLevel, 
                         rate_e             rate,
						 macAddress_t		*bssid,
 						 BOOL				dataMsdu);

TI_STATUS siteMgr_saveProbeRespBuffer(TI_HANDLE hSiteMgr, macAddress_t	*bssid, UINT8 *pProbeRespBuffer, UINT32 length);

TI_STATUS siteMgr_saveBeaconBuffer(TI_HANDLE hSiteMgr, macAddress_t *bssid, UINT8 *pBeaconBuffer, UINT32 length);


#endif /* __SME_API_H__ */
