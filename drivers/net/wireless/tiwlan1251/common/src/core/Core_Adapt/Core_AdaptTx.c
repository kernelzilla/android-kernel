/** \file Core_AdaptTx.c
 *  \brief CORE Adaptation Ctrl layer implementation
 *
 *  \see CORE_AdaptTx.h
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
/*                                                                          */
/*    MODULE:	Core_AdaptTx.c                                            */
/*    PURPOSE:  CORE Adaptation Tx layer implementation                   */
/*                                                                          */
/***************************************************************************/
#include "report.h"
#include "osApi.h"

#include "paramOut.h"
#include "siteHash.h"
#include "configMgr.h"
#include "whalCtrl_api.h"
#include "Core_AdaptTx.h"
#include "utils.h"
#include "DataCtrl_Api.h"
#include "TNETW_Driver_api.h"

/* Globals */
TI_HANDLE		CORE_AdaptTx_handle=NULL;

/* definitions */


/* Local Macros */


/* Local  functions definitions*/


/****************************************************************************************
 *                        CORE_AdaptTx_Create                                              *
 ****************************************************************************************
DESCRIPTION:    CORE ADAPTER TX CREATION

INPUT:          TI_HANDLE hOs
OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_HANDLE CORE_AdaptTx_Create(TI_HANDLE hOs)
{
	CORE_AdaptTx_t *pCORE_AdaptTx_handle;

    pCORE_AdaptTx_handle = os_memoryAlloc (hOs, sizeof(CORE_AdaptTx_t));
    if (pCORE_AdaptTx_handle == NULL)
        return NULL;

	pCORE_AdaptTx_handle->hOs = hOs;
		
    return (TI_HANDLE)pCORE_AdaptTx_handle;
}


/****************************************************************************************
 *                        CORE_AdaptTx_Config                                              *
 ****************************************************************************************
DESCRIPTION:    CORE ADAPTER TX CONFIGURATION

INPUT:          TI_HANDLE hOs
OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
int CORE_AdaptTx_Config (TI_HANDLE CORE_AdaptTx_handle, Core_AdaptTx_config_t *pCore_AdaptTx_config,txDataInitParams_t *txDataInitParams)
{
    CORE_AdaptTx_t *pCore_AdaptTx = (CORE_AdaptTx_t*)CORE_AdaptTx_handle;

	/* 
	 * handles
	 */
	pCore_AdaptTx->hReport = pCore_AdaptTx_config->hReport;
	pCore_AdaptTx->hMemMgr = pCore_AdaptTx_config->hMemMgr;
	pCore_AdaptTx->hTnetwDrv = pCore_AdaptTx_config->hTnetwDrv;
	pCore_AdaptTx->hTxData = pCore_AdaptTx_config->hTxData;
	pCore_AdaptTx->hCtrlData = pCore_AdaptTx_config->hCtrlData;

    return OK;
}


/****************************************************************************************
 *                        CORE_AdaptTx_Destroy                                              *
 ****************************************************************************************
DESCRIPTION:    CORE ADAPTER TX DESTRUCTION

INPUT:          TI_HANDLE hCore_AdaptTx - Passed even if its is global variable

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
int CORE_AdaptTx_Destroy (TI_HANDLE hCore_AdaptTx)
{
	CORE_AdaptTx_t *pCore_AdaptTx=(CORE_AdaptTx_t*)hCore_AdaptTx;
	int status = OK;

	if(hCore_AdaptTx == NULL)
	{
		return OK;
	}

    /* free the CORE_AdaptCtrl data structure*/
    os_memoryFree (pCore_AdaptTx->hOs, pCore_AdaptTx, sizeof(CORE_AdaptTx_t));

    return status;
}


/****************************************************************************************
 *                        CORE_AdaptTx_SendPacket                                              *
 ****************************************************************************************
DESCRIPTION:    CORE ADAPTER TX SendPacket

INPUT:          TI_HANDLE hCore_AdaptTx - Passed even if its is global variable

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/


TI_STATUS CORE_AdaptTx_SendPacket (TI_HANDLE hCore_AdaptTx,
							 UINT8 TxQid,
							 mem_MSDU_T* pMsdu,
							 txData_attr_t* pTxAttr,
                             UINT32 packetId,
							 UINT32 msduTimeToExpiry )
{
	void      *aFrame;
	UINT16     aLength;
	UINT8      aQueueId;
	UINT8      aTxRateClassId;
	CORE_AdaptTx_t *pCore_AdaptTx=(CORE_AdaptTx_t*)hCore_AdaptTx;

 	/*
	 *	handling send packet parameters
	 */

    /* Pointer to the frame actual data (right after TNETWIF_WRITE_OFFSET_BYTES and descriptor)  */
    aFrame = (void *)((UINT8*)memMgr_BufData(pMsdu->firstBDPtr) + TX_TOTAL_OFFSET_BEFORE_DATA);

	/*
	 * MSDU length. Measured from the first byte of the MAC header 
	 * to the last byte of the frame body.
	 */
	aLength = memMgr_MsduDataSize(pMsdu);

	/*
	 * Transmit queue as defined in ConfigureQueue
	 */
	aQueueId = TxQid;

	/*
	 * Transmit rate class ID defined in txRatePolicy 
	 */
	aTxRateClassId = pTxAttr->txRatePolicyId;

    return TnetwDrv_txXfer_sendPacket (pCore_AdaptTx->hTnetwDrv,  
						    aFrame,
	   					    aLength,
						    aQueueId,
						    aTxRateClassId,
						    pTxAttr->HwRate,
						    FALSE, /* more indication, currently ignored by GWSI */
						    packetId,
						    0,
						    msduTimeToExpiry,
                                       NULL);
}


/* --------------------------------- Call Back ---------------------------------------------------*/

/****************************************************************************************
 *                        SendPacketComplete                                              *
 ****************************************************************************************
DESCRIPTION:    CORE ADAPTER TX Tx Complete callback

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void SendPacketComplete (TI_HANDLE hUser,
                         systemStatus_e aStatus,
                        UINT32 aPacketId,
                        UINT32 aRate,
                        UINT8 aAckFailures,
                        UINT32 actualDurationInAir,
                        UINT32 fwHandlingTime,
						UINT32 mediumDelay)
{
	CORE_AdaptTx_t *pCore_AdaptTx=(CORE_AdaptTx_t*)CORE_AdaptTx_handle;
	txCompleteAttr_t txCompleteAttr;

	WLAN_REPORT_INFORMATION(pCore_AdaptTx->hReport, CTRL_DATA_MODULE_LOG, 
						("SendPacketComplete: aStatus = %d\n\n",aStatus));

	/*
	 * Call the core  CB (TI CORE)
	 */
	txCompleteAttr.status              = aStatus;
	txCompleteAttr.packetId            = aPacketId;
	txCompleteAttr.rate                = aRate;
	txCompleteAttr.ackFailures         = aAckFailures;
	txCompleteAttr.actualDurationInAir = actualDurationInAir;
    txCompleteAttr.fwHandlingTime      = fwHandlingTime;
    txCompleteAttr.mediumDelay         = mediumDelay;

    ctrlData_txCompleteStatus (pCore_AdaptTx->hCtrlData, &txCompleteAttr);
}


/****************************************************************************************
 *                        SendPacketTransfer                                             *
 ****************************************************************************************
DESCRIPTION:    CORE ADAPTER TX SendPacketTransfer callback

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
void SendPacketTransfer (TI_HANDLE hUser, UINT32 aPacketId)
{
    CORE_AdaptTx_t *pCore_AdaptTx = (CORE_AdaptTx_t*)CORE_AdaptTx_handle;

    txData_sendPacketTransfer (pCore_AdaptTx->hTxData, aPacketId);
}


/****************************************************************************************
 *                        SendPacketDebug                                               *
 ****************************************************************************************
DESCRIPTION:    CORE ADAPTER TX SendPacketDebug callback

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
#ifdef TI_DBG
void SendPacketDebug (TI_HANDLE hUser, UINT32 uPacketId, UINT32 uDebugInfo)
{
    CORE_AdaptTx_t *pCore_AdaptTx = (CORE_AdaptTx_t *)CORE_AdaptTx_handle;

    txData_sendPacketDebug (pCore_AdaptTx->hTxData, uPacketId, uDebugInfo);
}
#endif
