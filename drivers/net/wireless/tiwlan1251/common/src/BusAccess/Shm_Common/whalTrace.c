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
 *   MODULE:  whalTrace.c
 *   PURPOSE: Trace on Rx/Tx packets
 *   
 ****************************************************************************/

#include "whalCommon.h"
#include "TNETWIF.h"
#include "whalTrace.h"


#define SEQUNCE_TRACE		

#define TRACE_INTERRUPT		0x0001	/* Trace the recieved interupt only */
#define TRACE_TX			0x0002	/* Trace all Tx path (os ->>core ->> hal) */
#define TRACE_RX			0x0004	/* Trace all Rx path (hal ->> core ->> os) */
#define TRACE_COPY_ONLY		0x0008	/* Trace the copy of data from and to the FW only  - for slave mode*/
#define TRACE_BUG_ONLY		0x0010	/* Trace the ctrls that only presents an unexpected case*/
#define TRACE_All			0xffff	/* Trace all. */


 
/****************************************************************************
 *                      OBJECT DATA BASE
 ****************************************************************************
 *
 *	TrcDb - Rx Trace buffer of 5000 packets
 *
 ****************************************************************************/
int whal_tracePrintLine(WhalTrace_T *pTrc, int EventIndex);

void local_string_copy(char *str1, char *str2, int size)
{
	int i;
	for (i=0; i<size; i++)
	{
		str1[i] = str2[i];
		if (str2[i] == 0)
			break;
	}
}
/****************************************************************************
 *                      whal_traceCreate()
 ****************************************************************************
 * DESCRIPTION:	Create the whal trace object
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	The Created object
 ****************************************************************************/
WhalTrace_T *whal_traceCreate(TI_HANDLE hOs)
{
	WhalTrace_T *pObj;

	pObj = os_memoryAlloc(hOs, sizeof(WhalTrace_T));
	if (pObj == NULL)
		return NULL;

	os_memoryZero(hOs, pObj, sizeof(WhalTrace_T));

	pObj->hOs = hOs;
	pObj->hProtect = os_protectCreate(pObj->hOs);
	if (pObj->hProtect == NULL)
	{
		whal_traceDestroy(pObj);
		return NULL;
	}

	return(pObj);
}

/****************************************************************************
 *                      whal_traceDestroy()
 ****************************************************************************
 * DESCRIPTION:	Destroy the object 
 * 
 * INPUTS:	
 *		pTrc		The object to free
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_traceDestroy(WhalTrace_T *pTrc)
{
	if (pTrc == NULL)
		return OK;

	if (pTrc->hProtect)
		os_protectDestroy(pTrc->hOs, pTrc->hProtect);
	os_memoryFree(pTrc->hOs, pTrc, sizeof(WhalTrace_T));
	return OK;
}

/****************************************************************************
 *                      whal_traceConfig()
 ****************************************************************************
 * DESCRIPTION:	Config the object 
 * 
 * INPUTS:	
 *		pTrc		The object
 *		hReport		The reports objects
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_traceConfig(WhalTrace_T *pTrc, TI_HANDLE hTNETWIF, TI_HANDLE hReport)
{
	pTrc->hReport = hReport;
	pTrc->hTNETWIF = hTNETWIF;
	pTrc->Enable = 0; /*Hardcoded*/
	pTrc->Idx = 0;
	pTrc->Num = 0;
	pTrc->MaxFreeBlks = 0;

	/* configured the trace mask flag */
	pTrc->traceMask = TRACE_BUG_ONLY;
	return OK;
}

/****************************************************************************
 *                      whal_traceAddTx()
 ****************************************************************************
 * DESCRIPTION:	Add Tx info line to the DB
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/

#ifdef TNETW_MASTER_MODE

int whal_traceAddTx(WhalTrace_T *pTrc, HwTxDesc_T *pHwTxDesc, char *Action)
{
	TRACER_EVENT *pEvt;
	UINT32 FrameAddress;
	DbTescriptor local_TxDesc;
	TRACER_DATA	*pData;

#ifdef SEQUNCE_TRACE
	return 1;
#endif
	/* Dm: os_protectLock(pTrc->hOs, pTrc->hProtect); */

	pEvt = &pTrc->Evt[pTrc->Idx];
	pData = &pEvt->Info.TrcData;
	
	/*
	 * Common
	 */
	local_string_copy(pEvt->Action, Action, TRACER_MAX_ACT_LEN-1);
	local_string_copy(pEvt->Object, "Tx", TRACER_MAX_OBJ_LEN-1);

    pEvt->TimStamp = os_timeStampUs(pTrc->hOs);

	/* 
	 * Data
	 */
	whal_hwTxDesc_Copy(pHwTxDesc, &local_TxDesc);

	pData->MpduLen	= local_TxDesc.length;
	pData->Ctl		= local_TxDesc.ctl;
	pData->Rate		= (UINT16)local_TxDesc.rate;

    pData->Status[0] = 0;
	pData->Status[1] = 0;
	pData->Status[3] = 0;

    /*
	pData->Status[0] = local_TxDesc.ctrl2;
	pData->Status[1] = local_TxDesc.ackFailures;
	pData->Status[3] = local_TxDesc.rtsFailures;
    */

	FrameAddress = 4+whal_hwTxDesc_GetAcxBufAddr(pHwTxDesc);
	TNETWIF_ReadMemSync(pTrc->hTNETWIF,
						(UINT32)((char *)FrameAddress),
						(UINT8 *)((char *)&pData->FrameHeader),
						  TRACER_HDR_LEN);

	pData->FrameHeader.fc = ENDIAN_HANDLE_WORD(pData->FrameHeader.fc);
	pData->FrameHeader.seqCtrl = ENDIAN_HANDLE_WORD(pData->FrameHeader.seqCtrl);
	pData->FrameHeader.duration = ENDIAN_HANDLE_WORD(pData->FrameHeader.duration);
	TNETWIF_ReadMemSync(pTrc->hTNETWIF,
						(UINT32)((char *)FrameAddress+TRACER_HDR_LEN),
						(UINT8 *)((char *)&pData->FrameData),
						  4);


	/* 
	 * Prepare next index
	 */
	if (++pTrc->Idx >= TRACER_MAX_EVENTS)
		pTrc->Idx = 0;

	if (pTrc->Num < TRACER_MAX_EVENTS)
		pTrc->Num++;

	/* Dm: os_protectUnlock(pTrc->hOs, pTrc->hProtect); */

	return(0);
}

#endif  /* TNETW_MASTER_MODE */


/****************************************************************************
 *                      whal_traceIsEnable()
 ****************************************************************************
 * DESCRIPTION:	return the enable value
 * 
 * INPUTS:	None
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	Enable value
 ****************************************************************************/
int whal_traceIsEnable(WhalTrace_T *pTrc)
{
	return pTrc->Enable;
}

/****************************************************************************
 *                      whal_traceEnable()
 ****************************************************************************
 * DESCRIPTION:	enable the tracing
 * 
 * INPUTS:	None
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/
void whal_traceEnable(WhalTrace_T *pTrc, int val)
{
	pTrc->Enable = val;
	
	if(val == 1) /* Enable case*/
	{
		pTrc->MinNumDescriptorFree = 0xff;
		pTrc->MacOccupiedDescriptor = 0;
		pTrc->MaxClearDescriptor = 0;
		pTrc->BugCounter = 0;
		pTrc->reBug = 0;
		pTrc->MaxFreeBlks = 0;
	}

	
}



