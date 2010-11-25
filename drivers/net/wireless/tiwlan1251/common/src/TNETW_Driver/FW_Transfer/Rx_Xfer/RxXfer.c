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
 *   MODULE:  rxXfer.c
 *
 *   PURPOSE: Rx Xfer module implementation.Responsible for reading Rx from the FW
 *              and forward it to the upper layers.
 * 
 ****************************************************************************/

#include "RxXfer.h"
#include "utils.h"
#include "report.h"
#include "shmUtils.h"
#include "FwEvent_api.h"
#include "tnetwCommon.h"
#include "whalBus_Defs.h"

#define PLCP_HEADER_LENGTH 8

/************************ static function declaration *****************************/

static void      rxXfer_StateMachine (TI_HANDLE hRxXfer, UINT8 module_id, TI_STATUS status);
static TI_STATUS rxXfer_ReadHeader (RxXfer_t *pRxXfer);
static TI_STATUS rxXfer_ReadBody (RxXfer_t *pRxXfer);
static void      rxXfer_ForwardCB (RxXfer_t *pRxXfer);
static TI_STATUS rxXfer_AckRx (RxXfer_t *pRxXfer);
static void      rxXfer_ConvertDescFlagsToAppFlags (UINT16 descRxFlags, UINT32 *aFlags, TI_STATUS *pPacketStatus);


/****************************************************************************
 *                      RxXfer_Create()
 ****************************************************************************
 * DESCRIPTION: Create the RxXfer module object 
 * 
 * INPUTS:  None
 * 
 * OUTPUT:  None
 * 
 * RETURNS: The Created object
 ****************************************************************************/
TI_HANDLE rxXfer_Create (TI_HANDLE hOs)
{
    RxXfer_t *pRxXfer;

    pRxXfer = os_memoryAlloc (hOs, sizeof(RxXfer_t));
    if (pRxXfer == NULL)
        return NULL;

    /* For all the counters */
    os_memoryZero (hOs, pRxXfer, sizeof(RxXfer_t));

    pRxXfer->hOs = hOs;

    return (TI_HANDLE)pRxXfer;
}


/****************************************************************************
 *                      RxXfer_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the RxXfer module object 
 * 
 * INPUTS:  hRxXfer - The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/
void rxXfer_Destroy (TI_HANDLE hRxXfer)
{
    RxXfer_t *pRxXfer = (RxXfer_t *)hRxXfer;

    if (pRxXfer)
    {
        os_memoryFree (pRxXfer->hOs, pRxXfer, sizeof(RxXfer_t));
    }
} /* RxXfer_Destroy() */


/****************************************************************************
 *                      RxXfer_Config()
 ****************************************************************************
 * DESCRIPTION: Destroy the FwEvent module object 
 * 
 * INPUTS:      hRxXfer       - FwEvent handle;
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void rxXfer_Config(TI_HANDLE hRxXfer,
                   TI_HANDLE hFwEvent, 
                   TI_HANDLE hMemMgr,
                   TI_HANDLE hReport,
                   TI_HANDLE hTNETWIF)
{
    RxXfer_t  *pRxXfer      = (RxXfer_t *)hRxXfer;

    pRxXfer->hFwEvent       = hFwEvent;
    pRxXfer->hMemMgr        = hMemMgr;
    pRxXfer->hReport        = hReport;
    pRxXfer->hTNETWIF       = hTNETWIF;

    pRxXfer->state          = RX_XFER_STATE_IDLE;
    pRxXfer->currBuffer     = 0;        /* first buffer to read from */
    pRxXfer->lastPacketId   = 0;

    FwEvent_Enable (pRxXfer->hFwEvent, ACX_INTR_RX0_DATA);
    FwEvent_Enable (pRxXfer->hFwEvent, ACX_INTR_RX1_DATA);
    
#ifdef TI_DBG   
    rxXfer_ClearStats (pRxXfer);
#endif
}


/****************************************************************************
 *                      rxXfer_Register_CB()
 ****************************************************************************
 * DESCRIPTION: Register the function to be called for received Rx
 *              or the function to be called for request for buffer
 * 
 * INPUTS:      hRxXfer       - RxXfer handle;
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void rxXfer_Register_CB (TI_HANDLE hRxXfer, tiUINT32 CallBackID, void *CBFunc, TI_HANDLE CBObj)
{
    RxXfer_t* pRxXfer = (RxXfer_t *)hRxXfer;

    WLAN_REPORT_INFORMATION (pRxXfer->hReport, HAL_RX_MODULE_LOG, ("rxXfer_Register_CB (Value = 0x%x)\n", CallBackID));

    switch(CallBackID)
    {
    case HAL_INT_RECEIVE_PACKET:
        pRxXfer->ReceivePacketCB = (packetReceiveCB_t)CBFunc;
        pRxXfer->ReceivePacketCB_handle = CBObj;
        break;

    case HAL_INT_REQUEST_FOR_BUFFER:       
        pRxXfer->RequestForBufferCB = (requestForBufferCB_t)CBFunc;
        pRxXfer->RequestForBufferCB_handle = CBObj;
        break;

    default:
        WLAN_REPORT_ERROR(pRxXfer->hReport, HAL_RX_MODULE_LOG, ("rxXfer_Register_CB - Illegal value\n"));
        return;
    }
}


/****************************************************************************
 *                      rxXfer_SetDoubleBufferAddr()
 ****************************************************************************
 * DESCRIPTION: Store the addresses of the Double Buffer 
 * 
 * INPUTS:      hRxXfer       - RxXfer handle;
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void rxXfer_SetDoubleBufferAddr (TI_HANDLE hRxXfer, ACXDataPathParamsResp_t *pDataPathParams)
{
    RxXfer_t* pRxXfer = (RxXfer_t *)hRxXfer;

    pRxXfer->doubleBuffer[0] = pDataPathParams->rxPacketRingAddr;
    pRxXfer->doubleBuffer[1] = pDataPathParams->rxPacketRingAddr + pDataPathParams->rxPacketRingChunkSize;
} 


/****************************************************************************
 *                      rxXfer_RxEvent()
 ****************************************************************************
 * DESCRIPTION: Called upon Rx event from the FW.calls the SM  
 * 
 * INPUTS:      hRxXfer       - RxXfer handle;
 * 
 * OUTPUT:  None
 * 
 * RETURNS: TNETWIF_OK in case of Synch mode, or TNETWIF_PENDING in case of Asynch mode
 *          (when returning TNETWIF_PENDING, FwEvent module expects the FwEvent_EventComplete()
 *          function call to finish the Rx Client handling 
 *
 ****************************************************************************/
TI_STATUS rxXfer_RxEvent (TI_HANDLE hRxXfer)
{
    RxXfer_t* pRxXfer = (RxXfer_t *)hRxXfer;

    if (RX_XFER_STATE_IDLE != pRxXfer->state)
    {
        WLAN_REPORT_ERROR(pRxXfer->hReport,HAL_RX_MODULE_LOG,
            ("rxXfer_RxEvent called in state %d !!!\n",pRxXfer->state));
        return TNETWIF_ERROR;
    }

    WLAN_REPORT_INFORMATION (pRxXfer->hReport, HAL_RX_MODULE_LOG,
        ("rxXfer_RxEvent Calling rxXfer_StateMachine : currBuffer = %d \n",
        pRxXfer->currBuffer));

  #ifdef TI_DBG
    if (pRxXfer->currBuffer == 0)     
        pRxXfer->DbgStats.numIrq0 ++;
    else
        pRxXfer->DbgStats.numIrq1 ++;
  #endif

    /* Assume that we are in synch bus until otherwise is proven */
    pRxXfer->bSync = TRUE;
    /* The packet status is OK unless we receive error */
    pRxXfer->packetStatus = OK;

    rxXfer_StateMachine (hRxXfer, 0, OK);
    
    return pRxXfer->returnValue;
}


/****************************************************************************
 *                      rxXfer_StateMachine()
 ****************************************************************************
 * DESCRIPTION: SM for handling Synch & Asynch read of RX from the HW.
 *              The flow of the SM is by that order:
 *              IDLE -> READING_HDR -> READING_PKT -> EXIT
 *              On synch mode - each state is called in the same context in the while loop.
 *              On Asynch mode - each state returns TNETWIF_PENDING and exits the SM.The CB of
 *              each Asynch is the SM, that will continue the Rx handling.
 *
 * INPUTS:      hRxXfer       - RxXfer handle;
 * 
 * OUTPUT:      pRxXfer->returnValue is TNETWIF_OK in synch mode and TNETWIF_PENDING in Asynch mode.
 * 
 * RETURNS: 
 ****************************************************************************/
static void rxXfer_StateMachine (TI_HANDLE hRxXfer, UINT8 module_id, TI_STATUS status)
{
    RxXfer_t* pRxXfer = (RxXfer_t *)hRxXfer;

    pRxXfer->returnValue = OK;
    
    /*
     * This while loop will continue till the exit or when waiting for the CB due to
     * memory transfer operation pending for DMA to complete   
     */
    while (TNETWIF_PENDING != pRxXfer->returnValue)
    {
        WLAN_REPORT_DEBUG_RX (pRxXfer->hReport, ("Rx SM: state = %d, rc = %d\n",
            pRxXfer->state, pRxXfer->returnValue));
    
        switch(pRxXfer->state) 
        {
        case RX_XFER_STATE_IDLE:
            pRxXfer->state = RX_XFER_STATE_READING_HDR;
            pRxXfer->returnValue = rxXfer_ReadHeader (pRxXfer);
            break;

        case RX_XFER_STATE_READING_HDR:
            pRxXfer->state = RX_XFER_STATE_READING_PKT;
            pRxXfer->returnValue = rxXfer_ReadBody (pRxXfer);
            break;

        case RX_XFER_STATE_READING_PKT:
            pRxXfer->state = RX_XFER_STATE_EXITING;
            rxXfer_ForwardCB(pRxXfer);
            pRxXfer->returnValue = rxXfer_AckRx (pRxXfer);
            break;

        case RX_XFER_STATE_EXITING:
            pRxXfer->state = RX_XFER_STATE_IDLE;
            if (FALSE == pRxXfer->bSync)
            {   
                /* Async bus - call FwEvent for notifying the completion */
                FwEvent_EventComplete (pRxXfer->hFwEvent, TNETWIF_OK);
            }
            else    
            {
                /* This is the sync case - we should return TNETWIF_OK */
                pRxXfer->returnValue = TNETWIF_OK;
            }

            return;

        default:
            WLAN_REPORT_ERROR (pRxXfer->hReport, HAL_RX_MODULE_LOG,
                               ("rxXfer_StateMachine Unknown state = %d\n",
                               pRxXfer->state));
        }

        if (TNETWIF_ERROR == pRxXfer->returnValue)
        {   
            WLAN_REPORT_ERROR (pRxXfer->hReport, HAL_RX_MODULE_LOG,
                ("rxXfer_StateMachine returning TNETWIF_ERROR in state %d. Next packet will be discarded!!!\n",pRxXfer->state));

            /* Next packet will be marked as NOK and will be discarded */
            pRxXfer->packetStatus = NOK;
        }
    }

    /* If we are here - we got TNETWIF_PENDING, so we are in Async mode */
    pRxXfer->bSync = FALSE;
}


/****************************************************************************
 *                      rxXfer_ReadHeader()
 ****************************************************************************
 * DESCRIPTION: Read the packet header (descriptor)
 *
 * INPUTS:      pRxXfer       - RxXfer handle;
 * 
 * OUTPUT:      
 * 
 * RETURNS:     TNETWIF_OK in synch mode and TNETWIF_PENDING in Asynch mode.
 ****************************************************************************/
TI_STATUS rxXfer_ReadHeader(RxXfer_t *pRxXfer)
{
    WLAN_REPORT_DEBUG_RX(pRxXfer->hReport, 
                        ("rxXfer_readHeader: Before Read Memory Addr from DB No %d Addr %x !!!! \n",pRxXfer->currBuffer,pRxXfer->doubleBuffer[pRxXfer->currBuffer]));

    return TNETWIF_ReadMemOpt (pRxXfer->hTNETWIF, 
                               pRxXfer->doubleBuffer[pRxXfer->currBuffer], 
                               PADREAD (&pRxXfer->rxDescriptor),
                               RX_DESCRIPTOR_SIZE,
                               FW_EVENT_MODULE_ID,
                               rxXfer_StateMachine,
                               (TI_HANDLE)pRxXfer);
}


/****************************************************************************
 *                      rxXfer_ReadBody()
 ****************************************************************************
 * DESCRIPTION: Read the packet body
 *
 * INPUTS:      pRxXfer       - RxXfer handle;
 * 
 * OUTPUT:      
 * 
 * RETURNS:     TNETWIF_OK in synch mode and TNETWIF_PENDING in Asynch mode.
 ****************************************************************************/
TI_STATUS rxXfer_ReadBody (RxXfer_t *pRxXfer)
{
    UINT32  uCurrPacketId;             /* Current packet ID */
    UINT32  uLastPacketIdIncremented;  /* The last received packet-ID incremented with modulo */
    UINT32  uAlignToWord;              /* Used to align the length of the packet to a WORD */ 

	/* Check for correct length of Rx Descriptor */
    if (pRxXfer->rxDescriptor.length <= PLCP_HEADER_LENGTH || 
        pRxXfer->rxDescriptor.length > (MAX_DATA_BODY_LENGTH + PLCP_HEADER_LENGTH))
    {
        WLAN_REPORT_ERROR (pRxXfer->hReport, HAL_RX_MODULE_LOG,
             ("rxXfer_ReadBody: RxLength not correct! rxDescriptor.length=%d\n",         
             pRxXfer->rxDescriptor.length));
        return TNETWIF_ERROR;
    }

    pRxXfer->rxDescriptor.length = pRxXfer->rxDescriptor.length - PLCP_HEADER_LENGTH;

    uCurrPacketId = (pRxXfer->rxDescriptor.flags & RX_DESC_SEQNUM_MASK) >> RX_DESC_PACKETID_SHIFT;

    uLastPacketIdIncremented = (pRxXfer->lastPacketId + 1) % (RX_MAX_PACKET_ID + 1);

    if (uCurrPacketId == uLastPacketIdIncremented)
    {
        pRxXfer->lastPacketId = uLastPacketIdIncremented;

#ifdef GWSI_RECORDING
        WLAN_REPORT_GWSI_RECORDING(pRxXfer->hReport, ("GWSI Recording, rxXfer_ReadBody (request for buffer), Length = 0x%x\n", pRxXfer->rxDescriptor.length));
#endif /* GWSI_RECORDING */
    }
    else
    {
       WLAN_REPORT_ERROR (pRxXfer->hReport, HAL_RX_MODULE_LOG,
            ("rxXfer_ReadBody: Packet ID mismatch! CurrPacketId=%d, lastPacketId=%d\n",         
            uCurrPacketId, pRxXfer->lastPacketId));
#ifdef TI_DBG
        pRxXfer->DbgStats.numPacketsDroppedPacketIDMismatch++;
        rxXfer_PrintStats ((TI_HANDLE)pRxXfer);
#endif
        /* Reset the lastPacketId to be synchronized on the Current Packet ID read from the FW */
        pRxXfer->lastPacketId = uCurrPacketId;
    }

    /*
     * Add uAlignToWord to the body length since we have to read buffers from the FW in 4 bytes chunks.
     * NOTE: The size of the buffer is aligned to 4, but the packet itself is not.
     * Releasing the memory must be done with the actual size allocated and not the size of the packet 
     */
    uAlignToWord = 4 - (pRxXfer->rxDescriptor.length & 0x3); /* (&0x3) is equal to (% 4) */
    uAlignToWord = (uAlignToWord == 4) ? 0 : uAlignToWord;

    /* 
     * Requesting buffer from the upper layer memory manager. 
     * Add the align to word and offset for the access to the bus 
	 * Also send the encryption status of the packet. It is used only for GWSI alignment.
     */
    pRxXfer->pPacketBuffer = (void *)pRxXfer->RequestForBufferCB (
        pRxXfer->RequestForBufferCB_handle,
        pRxXfer->rxDescriptor.length + uAlignToWord + TNETWIF_READ_OFFSET_BYTES,
		((pRxXfer->rxDescriptor.flags & RX_DESC_ENCRYPTION_MASK) >> RX_DESC_FLAGS_ENCRYPTION));

    if (pRxXfer->pPacketBuffer != NULL)
    {
        WLAN_REPORT_DEBUG_RX (pRxXfer->hReport,
            (" rxXfer_ReadBody() : packetLength %d uAligntoWord = %d\n",
			  pRxXfer->rxDescriptor.length, uAlignToWord));

#ifdef TI_DBG
        pRxXfer->DbgStats.numPacketsRead++;
        pRxXfer->DbgStats.numBytesRead += pRxXfer->rxDescriptor.length;
#endif

        /* Read the packet and return TNETWIF_OK or TNETWIF_PENDING */
        return TNETWIF_ReadMemOpt (pRxXfer->hTNETWIF,
                                   pRxXfer->doubleBuffer[pRxXfer->currBuffer] + RX_DESCRIPTOR_SIZE + 20,
                                   (UINT8 *)pRxXfer->pPacketBuffer,
                                   (UINT32)pRxXfer->rxDescriptor.length + uAlignToWord,
                                   FW_EVENT_MODULE_ID,
                                   rxXfer_StateMachine,
                                   (TI_HANDLE)pRxXfer);
    
    }
    /* If no buffer could be allocated */
    else   
    {
        WLAN_REPORT_ERROR (pRxXfer->hReport, HAL_RX_MODULE_LOG,
            ("rxXfer_RecvOnePacket:  pRxXfer->rxDescriptor %x NULL !!!! \n",pRxXfer->pPacketBuffer));

#ifdef TI_DBG
        pRxXfer->DbgStats.numPacketsDroppedNoMem++;
#endif
    }
    
    return TNETWIF_ERROR;
}


/****************************************************************************
 *                      rxXfer_ForwardCB()
 ****************************************************************************
 * DESCRIPTION: Parse the Packet with the descriptor and forward the results and
 *              the packet to the registered CB
 *
 * INPUTS:      pRxXfer       - RxXfer handle;
 * 
 * OUTPUT:      
 * 
 * RETURNS:     
 ****************************************************************************/
void rxXfer_ForwardCB (RxXfer_t *pRxXfer)
{
    UINT32            aFlags = 0;
    rate_e            eRate = DRV_RATE_AUTO;
    rxXfer_Reserved_t Reserved;
    RxIfDescriptor_t *pRxParams = &pRxXfer->rxDescriptor;

    WLAN_REPORT_DEBUG_RX (pRxXfer->hReport, ("rxXfer_ForwardCB ENTERING\n"));
   
    eRate = ConvertHwRateToDrvRate(pRxParams->rate, (BOOL)(OFDM_RATE_BIT & pRxParams->modPre));

    WLAN_REPORT_DEBUG_RX (pRxXfer->hReport,
        (" rxXfer_ForwardCB() HwRate = %d, modPre = %d eRate = %d:\n",pRxParams->rate,pRxParams->modPre,eRate));

    if ( eRate == DRV_RATE_AUTO)
    {
        WLAN_REPORT_ERROR (pRxXfer->hReport, HAL_RX_MODULE_LOG,
            ("rxXfer_ForwardCB:  Received wrong rate from Hw = 0x%x, modPre = 0x%x\n",
            pRxParams->rate,pRxParams->modPre));
    }
    
    Reserved.packetType = (rxPacketType_e)pRxParams->type;
    Reserved.rssi       = pRxParams->rssi;
    Reserved.SNR        = pRxParams->snr;
    Reserved.band       = pRxParams->band;
    Reserved.TimeStamp  = pRxParams->timestamp;

    if (pRxXfer->packetStatus == OK)
    {
        /* Get the mac header from the TNETWIF_READ_OFFSET_BYTES in the packet Buffer */
        dot11_header_t *pMacHdr = (dot11_header_t *)((UINT8*)pRxXfer->pPacketBuffer + TNETWIF_READ_OFFSET_BYTES);
      #ifdef GWSI_RECORDING
        static  char TempString[(1600 * 2) + 1];
      #endif /* GWSI_RECORDING */

        /* Handle endian for the frame control fields */
        pMacHdr->fc       = ENDIAN_HANDLE_WORD(pMacHdr->fc);
        pMacHdr->duration = ENDIAN_HANDLE_WORD(pMacHdr->duration);
        pMacHdr->seqCtrl  = ENDIAN_HANDLE_WORD(pMacHdr->seqCtrl);

        rxXfer_ConvertDescFlagsToAppFlags (pRxParams->flags, &aFlags, &pRxXfer->packetStatus);

      #ifdef GWSI_RECORDING
        convert_hex_to_string ((UINT8*)pRxXfer->pPacketBuffer + TNETWIF_READ_OFFSET_BYTES, TempString, pRxParams->length);

        WLAN_REPORT_GWSI_RECORDING (pRxXfer->hReport, 
                                    ("GWSI Recording, rxXfer_RecvPacketCB, aStatus = 0x%x, aRate = 0x%x, aRCPI = 0x%x, aFlags = 0x%x\n", 
                                    pRxXfer->packetStatus, aRate, pRxParams->rcpi, aFlags));
        WLAN_REPORT_GWSI_RECORDING (pRxXfer->hReport, 
                                    ("GWSI Recording, rxXfer_RecvPacketCB, aLength = 0x%x, aFrame = %s\n", 
                                    pRxParams->length, TempString));    
      #endif /* GWSI_RECORDING */
    }

    /* Set the packet to upper layer. packet is starting after TNETWIF_READ_OFFSET_BYTES bytes */
    pRxXfer->ReceivePacketCB (pRxXfer->ReceivePacketCB_handle,
                              pRxXfer->packetStatus,
                              (const void*)pRxXfer->pPacketBuffer,
                              pRxParams->length,
                              (UINT32)eRate,
                              pRxParams->rcpi,
                              pRxParams->chanNum,
                              (void *)&Reserved,
                              aFlags);
}


/****************************************************************************
 *                      rxXfer_AckRx()
 ****************************************************************************
 * DESCRIPTION: Set Ack to the FW that the buffer was read
 *
 * INPUTS:      pRxXfer       - RxXfer handle;
 * 
 * OUTPUT:      
 * 
 * RETURNS:     TNETWIF_OK in synch mode and TNETWIF_PENDING in Asynch mode.
 ****************************************************************************/
TI_STATUS rxXfer_AckRx (RxXfer_t *pRxXfer)
{
    TI_STATUS status;

    /* Ack on the opposite buffer since we changed it in rxXfer_ForwardCB() */
    if (pRxXfer->currBuffer == 0)
    {
        WLAN_REPORT_DEBUG_RX (pRxXfer->hReport, ("Ack on Rx 0\n"));
      
      #ifdef TI_DBG
        pRxXfer->DbgStats.numAck0 ++;
      #endif

        status = TNETWIF_WriteRegOpt (pRxXfer->hTNETWIF, 
                                      ACX_REG_INTERRUPT_TRIG, 
                                      INTR_TRIG_RX_PROC0,
                                      FW_EVENT_MODULE_ID,
                                      rxXfer_StateMachine,
                                      (TI_HANDLE)pRxXfer);
    }
    else 
    {
        WLAN_REPORT_DEBUG_RX (pRxXfer->hReport, ("Ack on Rx 1\n"));

      #ifdef TI_DBG
        pRxXfer->DbgStats.numAck1 ++;
      #endif

        status = TNETWIF_WriteRegOpt (pRxXfer->hTNETWIF, 
                                      ACX_REG_INTERRUPT_TRIG_H, 
                                      INTR_TRIG_RX_PROC1,
                                      FW_EVENT_MODULE_ID,
                                      rxXfer_StateMachine,
                                      (TI_HANDLE)pRxXfer);
    }

    /* Calculate the next buffer to read from (0 or 1) */
    pRxXfer->currBuffer = 1 - pRxXfer->currBuffer;

    return status;
}


/****************************************************************************
 *                      rxXfer_ConvertDescFlagsToAppFlags()
 ****************************************************************************
 * DESCRIPTION: Add some spacing before capital letters and you'll figure it out ...
 *
 * INPUTS:      descRxFlags   - Bits as received from Fw
 * 
 * OUTPUT:      aFlags        - converted bits to our definition    
 *              pPacketStatus - changed status if an error was indicated
 * RETURNS:     
 ****************************************************************************/
static void  rxXfer_ConvertDescFlagsToAppFlags (UINT16 descRxFlags, UINT32 *aFlags, TI_STATUS *pPacketStatus)
{
    UINT32 flags = 0;

    if (descRxFlags & RX_DESC_MATCH_RXADDR1)
    {
        flags |= RX_PACKET_FLAGS_MATCH_RXADDR1;
    }

    if (descRxFlags & RX_DESC_MCAST)
    {
        flags |= RX_PACKET_FLAGS_GROUP_ADDR;
    }

    if (descRxFlags & RX_DESC_STAINTIM)
    {
        flags |= RX_PACKET_FLAGS_STAINTIM;
    }

    if (descRxFlags & RX_DESC_VIRTUAL_BM)
    {
        flags |= RX_PACKET_FLAGS_VIRTUAL_BM;
    }

    if (descRxFlags & RX_DESC_BCAST)
    {
        flags |= RX_PACKET_FLAGS_BCAST;
    }

    if (descRxFlags & RX_DESC_MATCH_SSID)
    {
        flags |= RX_PACKET_FLAGS_MATCH_SSID;
    }

    if (descRxFlags & RX_DESC_MATCH_BSSID)
    {
        flags |= RX_PACKET_FLAGS_MATCH_BSSID;
    }

    flags |= (descRxFlags & RX_DESC_ENCRYPTION_MASK) << RX_PACKET_FLAGS_ENCRYPTION_SHIFT_FROM_DESC;

    if (descRxFlags & RX_DESC_MEASURMENT)
    {
        flags |= RX_PACKET_FLAGS_MEASURMENT;
    }

    if (descRxFlags & RX_DESC_MIC_FAIL)
    {
        *pPacketStatus = RX_MIC_FAILURE_ERROR;
    }

    if (descRxFlags & RX_DESC_DECRYPT_FAIL)
    {
        *pPacketStatus = RX_DECRYPT_FAILURE;
    }

    *aFlags = flags;
}

#ifdef TI_DBG
/****************************************************************************
 *                      rxXfer_ClearStats()
 ****************************************************************************
 * DESCRIPTION: 
 *
 * INPUTS:  
 *          pRxXfer The object
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK. 
 ****************************************************************************/
void rxXfer_ClearStats (TI_HANDLE hRxXfer)
{
    RxXfer_t * pRxXfer = (RxXfer_t *)hRxXfer;

    os_memoryZero (pRxXfer->hOs, &pRxXfer->DbgStats, sizeof(RxXferStats_T));
}


/****************************************************************************
 *                      rxXfer_PrintStats()
 ****************************************************************************
 * DESCRIPTION: .
 *
 * INPUTS:  
 *          pRxXfer The object
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK. 
 ****************************************************************************/
void rxXfer_PrintStats (TI_HANDLE hRxXfer)
{
    RxXfer_t * pRxXfer = (RxXfer_t *)hRxXfer;
    
    WLAN_OS_REPORT(("Number of packets read: %d, number of bytes read:%d\n",
                    pRxXfer->DbgStats.numPacketsRead, pRxXfer->DbgStats.numBytesRead));
    WLAN_OS_REPORT(("Number of frames dropped due to no memory:%d, Number of frames dropped due to packet ID mismatch:%d\n",
                    pRxXfer->DbgStats.numPacketsDroppedNoMem, pRxXfer->DbgStats.numPacketsDroppedPacketIDMismatch));
    WLAN_OS_REPORT(("Number of irq0:%u, ack0:%d\n",
                    pRxXfer->DbgStats.numIrq0, pRxXfer->DbgStats.numAck0));
    WLAN_OS_REPORT(("Number of irq1:%u, ack1:%d\n",
                    pRxXfer->DbgStats.numIrq1, pRxXfer->DbgStats.numAck1));
}
#endif

/****************************************************************************
 *                      RxXfer_ReStart()
 ****************************************************************************
 * DESCRIPTION:	RxXfer_ReStart the RxXfer module object (called by the recovery)
 * 
 * INPUTS:	hRxXfer - The object to free
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	NONE 
 ****************************************************************************/

VOID RxXfer_ReStart(TI_HANDLE hRxXfer)
{
	RxXfer_t * pRxXfer = (RxXfer_t *)hRxXfer;

	pRxXfer->state          = RX_XFER_STATE_IDLE;
	pRxXfer->currBuffer     = 0;        /* first buffer to read from */
	pRxXfer->lastPacketId   = 0;
	
} /* RxXfer_ReStart() */

