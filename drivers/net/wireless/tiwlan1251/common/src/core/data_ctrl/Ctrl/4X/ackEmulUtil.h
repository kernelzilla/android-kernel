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

/***************************************************************************/
/*																		   */
/*		MODULE:	ackEmulUtil.h											   */
/*   PURPOSE:	Ack emulation Utility 	              					   */
/*											    						   */
/***************************************************************************/
#ifndef _ACK_EMULATION_UTIL_H_
#define _ACK_EMULATION_UTIL_H_

#include "osTIType.h"
#include "ackEmulDb.h"


typedef struct
{
	TI_HANDLE	 		hWhalCtrl;
	TI_HANDLE			hOs;
	TI_HANDLE			hReport;
	TI_HANDLE			hMemMngr;

	ackEmulDB_t*		pAckEmulDB;

	int ackEmulationActive;

}ackEmul_t;

ackEmul_t* ackEmul_create(TI_HANDLE hOs);

TI_STATUS ackEmul_config(ackEmul_t*		ackEmul,
							TI_HANDLE	hWhalCtrl,
							TI_HANDLE	hOs,
							TI_HANDLE	hReport,
							TI_HANDLE	hMemMngr);

TI_STATUS ackEmul_destroy(ackEmul_t*	ackEmul);




TI_STATUS wdrv_ackEmulationRxPacket(ackEmul_t*		ackEmul, mem_MSDU_T *pMsdu);

TI_STATUS wdrv_ackEmulationTxPacket(ackEmul_t*		ackEmul, mem_MSDU_T *pMsdu,int *discardPacket);

void wdrv_aeWackReceive(ackEmul_t*		ackEmul, UINT16 station, UINT8 wackInfo);
void gener(ackEmul_t*		ackEmul, UINT16 stationIndex, UINT8 activeIndex ,UINT32 ackNumber);

void wdrv_aeSetActive(ackEmul_t*		ackEmul, int status);
int wdrv_aeGetActive(ackEmul_t*		ackEmul);

void gener(ackEmul_t*		ackEmul, UINT16 stationIndex, UINT8 activeIndex ,UINT32 ackNumber);

#endif
