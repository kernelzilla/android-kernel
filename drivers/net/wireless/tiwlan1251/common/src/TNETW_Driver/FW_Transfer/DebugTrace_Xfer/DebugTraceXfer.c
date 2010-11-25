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

#ifdef TI_DBG

#include "FwEvent_api.h"
#include "whalParams.h"
#include "DebugTraceXfer_api.h"
#include "report.h"
#include "whalBus_Defs.h"
#include "TNETWIF.h"
#include "utils.h"
#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
#include "memMngrEx.h"
#endif
#include "whalCommon.h"



/*****************************************************************************
 **         UDP Packet Build                                                **
 *****************************************************************************/
#define Ether_Type  0x0800  /* For IP Protocol */

 /* This is an Ethernet Version 2 frame: 14 Byte */
typedef struct {
  UINT8    ether_dst[6];
  UINT8    ether_src[6];
  UINT16   ether_Type;  
} etherHeader;


#define IP_Version    0x45   /* IP Ver - 4 Length Of IP Header 5 UINT32 (20 Byte) */
#define IP_TimeToLive 128    /* limit the number of Router Hops  */
#define IP_Protocol   0x11   /* UDP protocol */
    
 /* This is an IP frame: 20 Byte */
typedef struct 
{
    UINT8   IpVer;
    UINT8   TypeOfService;
    UINT16  Totallen;
    UINT16  Indentification;
    UINT16  FragOffset;
    UINT8   TimeToLive; 
    UINT8   protocol;
    UINT16  csum;
    UINT8   src[4];
    UINT8   dst[4];
} ipstruct; 

#define Source_Port 0
#define Destination_Port 5555

 /* This is a UDP frame: 8 Byte */
typedef struct {
  UINT16    src_port;
  UINT16    dst_port;
  UINT16    UDP_len;    
  UINT16    UDP_Csum;    
} UDPStruct;

 /* This is a Trace Frame:*/
#pragma pack(1)
typedef struct {
  etherHeader  Ether_Header;
  ipstruct     IP_Header;
  UDPStruct    UDP_Header;  
} TraceFrame;
#pragma pack()

/*****************************************************************************
 **         Structures                                                      **
 *****************************************************************************/
typedef enum
{
    DEBUG_TRACE_STATE_IDLE,
    DEBUG_TRACE_STATE_READ_BUF,
    DEBUG_TRACE_STATE_ACK_EVENT
} debugTraceState_e;
/*
 *  DebugTraceXfer: The Debug Trace Object 
 */
typedef struct 
{
    TraceFrame      FrameHeader; /* Do Not Change Place */

    /* use a struct to read buffers from the bus - used for extra bytes reserving */
    PADDING (UINT32  debugWords[TRACE_BUFFER_MAX_SIZE * 2])   /* *2 for the 2 buffers */

    BOOL            bDoPrint; 
    BOOL            bUdpSend; 
    BOOL            bSync;  /* indicate if we are in Synch bus or not */

    UINT32          TraceBufferSize;
    
    UINT32          BufferOffset[2];    /* holds the offset of Buffer A and Buffer B */
    UINT8           currBuffer;         /* switch between 0 and 1 according to the buffer used */
    
    debugTraceState_e state;
    TI_STATUS       returnValue;

    TI_HANDLE       hOs;
    TI_HANDLE       hReport;
    TI_HANDLE       hFwEvent;
    TI_HANDLE       hMemMgr;
    TI_HANDLE       hTNETWIF;
} DebugTrace_t;

/****************************************************************************
 *                      static function declaration
 ****************************************************************************/
#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
static void debugTrace_BuildFrame(DebugTrace_t *pDebugTrace, UINT8* MacAddress);
#endif
static void debugTrace_StateMachine(TI_HANDLE hDebugTrace,UINT8 module_id ,TI_STATUS status);

/****************************************************************************
 *                      debugTrace_Create()
 *****************************************************************************  
 * DESCRIPTION: Create
 * 
 * INPUTS:
 *
 * RETURNS: 
 *
 *****************************************************************************/
TI_HANDLE debugTrace_Create(TI_HANDLE hOs)
{
    DebugTrace_t *pDebugTrace;

    pDebugTrace = os_memoryAlloc(hOs, sizeof(DebugTrace_t));
    if (pDebugTrace == NULL)
    {
        WLAN_OS_REPORT(("debugTrace_Create: Error Creating Trace Buffer \n"));
        return NULL;
    }

    os_memoryZero(hOs, pDebugTrace, sizeof(DebugTrace_t));

    pDebugTrace->hOs = hOs;

    return((TI_HANDLE)pDebugTrace);
}

/****************************************************************************
 *                      debugTrace_Destroy()
 *****************************************************************************  
 * DESCRIPTION: Destroy
 * 
 * INPUTS:  
 *
 * RETURNS: 
 *
 *****************************************************************************/
void debugTrace_Destroy(TI_HANDLE hDebugTrace)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    if (pDebugTrace)
    {
        os_memoryFree(pDebugTrace->hOs, pDebugTrace, sizeof(DebugTrace_t));
    }
}

/****************************************************************************
 *                      debugTrace_Config()
 *****************************************************************************  
 * DESCRIPTION: Config
 * 
 * INPUTS:  
 *
 * RETURNS: 
 *
 *****************************************************************************/
void debugTrace_Config(TI_HANDLE hDebugTrace, TI_HANDLE hWhalParams, TI_HANDLE hReport,
                        TI_HANDLE hMemMgr, TI_HANDLE hTNETWIF, TI_HANDLE hFwEvent)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;
    
    pDebugTrace->hReport  = hReport;
    pDebugTrace->hMemMgr  = hMemMgr;
    pDebugTrace->hTNETWIF = hTNETWIF;
    pDebugTrace->hFwEvent = hFwEvent;

    /* next 2 parameters are TRUE & FALSE by default but can be changed in run time */
    pDebugTrace->bUdpSend  = FALSE;
    pDebugTrace->bDoPrint  = TRUE;
    pDebugTrace->TraceBufferSize = whal_ParamsGetTraceBufferSize(hWhalParams); 
    pDebugTrace->currBuffer = 0;

#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
    /* Build Frame to be send to the OS */
    debugTrace_BuildFrame(pDebugTrace, whal_ParamsGetSrcMac(hWhalParams));
#endif

    /* Enable Trace Interrupts  - actual interrupts will be raised only after init phase */
    FwEvent_Enable(pDebugTrace->hFwEvent, ACX_INTR_TRACE_A);
    FwEvent_Enable(pDebugTrace->hFwEvent, ACX_INTR_TRACE_B);
}

 /****************************************************************************
 *                      debugTrace_ConfigHw()
 *****************************************************************************  
 * DESCRIPTION: Initialize the address to read the buffer from the FW
 * 
 * INPUTS:  hDebugTrace - Debug Trace object
 *
 * RETURNS:  
 *
 *****************************************************************************/
void debugTrace_ConfigHw(TI_HANDLE hDebugTrace,UINT32 TraceAddA, UINT32 TraceAddB)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    pDebugTrace->BufferOffset[0] = TraceAddA;  
    pDebugTrace->BufferOffset[1] = TraceAddB;  
    
    WLAN_REPORT_INIT(pDebugTrace->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("debugTrace_ConfigHw: Buffer A offSet =0x%x, Buffer B offSet =0x%x, TraceBufferSize = %d \n"
        ,pDebugTrace->BufferOffset[0], pDebugTrace->BufferOffset[1],pDebugTrace->TraceBufferSize));

}

/****************************************************************************
 *                      debugTrace_Event()
 *****************************************************************************  
 * DESCRIPTION: Called upon an interrupt of debug trace kind.
 * 
 * INPUTS:  hDebugTrace - Debug Trace object
 *
 * RETURNS:  
 *
 *****************************************************************************/
TI_STATUS debugTrace_Event(TI_HANDLE hDebugTrace)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    WLAN_REPORT_INFORMATION(pDebugTrace->hReport, HAL_HW_CTRL_MODULE_LOG,
        (" debugTrace_Event: Reading from Buffer %d \n",pDebugTrace->currBuffer));

    /* assume that we are in synch bus until otherwise is proven */
    pDebugTrace->bSync = TRUE;

    debugTrace_StateMachine(pDebugTrace,0,(TI_STATUS)0);

    return pDebugTrace->returnValue;
}


/****************************************************************************
 *                      debugTrace_StateMachine()
 ****************************************************************************
 * DESCRIPTION: Manage the Debug trace state machine 
 *
 *              The SM is running one event at a time (buffer A or B) .
 *              The order of the states is always the same: IDLE --> READ_BUF --> ACK_EVENT
 *              The difference is whether we are using Synch or Asynch API.
 *              In the Synch case (SDIO) we are looping in the while-loop till we return to IDLE, and we return
 *              to FwEvent module a TNETWIF_OK status.
 *              In the Asynch case we use the SM CB to return to the SM after each Asynch call
 *              (In that case the return status is TNETWIF_PENDING, and we are waiting for the CB).
 *              In the Asynch case the FwEvent module gets TNETWIF_PENDING in return, and waits for 
 *              the FwEvent_EventComplete() call in order to move the FwEvent SM.
 * 
 * INPUTS:  hFwEvent - The object
 *          module_id - not used (for CB API only)
 *          status    - not used (for CB API only) 
 *      
 * OUTPUT:  None
 * 
 * RETURNS: TNETWIF_PENDING in case of Async and TNETWIF_OK on Sync 
 ****************************************************************************/
static void debugTrace_StateMachine(TI_HANDLE hDebugTrace,UINT8 module_id ,TI_STATUS status)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    pDebugTrace->returnValue = OK;

    while(pDebugTrace->returnValue != TNETWIF_PENDING)
    {
        switch(pDebugTrace->state)
        {
            case DEBUG_TRACE_STATE_IDLE:

                pDebugTrace->returnValue = TNETWIF_ReadMemOpt (pDebugTrace->hTNETWIF,
                                                               pDebugTrace->BufferOffset[pDebugTrace->currBuffer],
                                                               PADREAD (pDebugTrace->debugWords),
                                                               pDebugTrace->TraceBufferSize,
                                                               FW_EVENT_MODULE_ID,
                                                               debugTrace_StateMachine,
                                                               pDebugTrace);

                pDebugTrace->state = DEBUG_TRACE_STATE_READ_BUF;
            
                break;

            case DEBUG_TRACE_STATE_READ_BUF:
                /* Now, handle the buffer */
                /* Print on every Trace */
                if(pDebugTrace->bDoPrint)
                {
                    debugTrace_Print(pDebugTrace);
                }   

#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
                /* Handle The Trace */
                if(pDebugTrace->bUdpSend)
                {
                    debugTrace_handleBuffer(hDebugTrace);
                }
#endif

                /*Trigger the FW when finishing handle the event */
                pDebugTrace->returnValue = TNETWIF_WriteRegOpt(pDebugTrace->hTNETWIF, ACX_REG_INTERRUPT_TRIG, INTR_TRIG_DEBUG_ACK,
                                                FW_EVENT_MODULE_ID,debugTrace_StateMachine,pDebugTrace);

                pDebugTrace->state = DEBUG_TRACE_STATE_ACK_EVENT;

                break;

            case DEBUG_TRACE_STATE_ACK_EVENT:

                /* Handling of the event is done. Now switch to the next buffer for the next time */ 
                pDebugTrace->currBuffer = (++pDebugTrace->currBuffer) & 0x1; /* &0x1 is %2 */
                    
                if (FALSE  == pDebugTrace->bSync ) 
                {
                    /* Async bus - call FwEvent for notifying the completion */
                    FwEvent_EventComplete(pDebugTrace->hFwEvent, TNETWIF_OK);
                }
                else    /* This is the Sync case and we return TNETWIF_OK */
                {
                    pDebugTrace->returnValue = TNETWIF_OK;
                }
                /* Exit SM */
                pDebugTrace->state = DEBUG_TRACE_STATE_IDLE;
                return;

/*                break; */

            default:
                WLAN_REPORT_ERROR(pDebugTrace->hReport, HAL_HW_CTRL_MODULE_LOG, 
                    ("debugTrace_StateMachine: unKnown state !!!\n"));

                break;
        }
    }
    /* if we are here - we got TNETWIF_PENDING, so we are in Async mode */
    pDebugTrace->bSync = FALSE;

    if (pDebugTrace->returnValue == TNETWIF_ERROR) 
    {
        WLAN_REPORT_ERROR(pDebugTrace->hReport, HAL_HW_CTRL_MODULE_LOG, 
                ("debugTrace_StateMachine: rc = TNETWIF_ERROR in state %d  \n",pDebugTrace->state));
    }
}

 /****************************************************************************
 *                      debugTrace_Disable()
 *****************************************************************************  
 * DESCRIPTION: disable receiving interrupts of this kind (debug trace)
 * 
 * INPUTS:  hDebugTrace - Debug Trace object
 *
 * RETURNS:  
 *
 *****************************************************************************/
void debugTrace_Disable(TI_HANDLE hDebugTrace)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    FwEvent_Disable(pDebugTrace->hFwEvent, ACX_INTR_TRACE_A);
    FwEvent_Disable(pDebugTrace->hFwEvent, ACX_INTR_TRACE_B);
    
    WLAN_REPORT_INFORMATION(pDebugTrace->hReport, HAL_HW_CTRL_MODULE_LOG, 
                            ("whal_DebugTrace_t_Disable: Trace Disable \n"));

}

 /****************************************************************************
 *                      debugTrace_Enable()
 *****************************************************************************  
 * DESCRIPTION: Enables receiving interrupts of this kind (debug trace)
 * 
 * INPUTS:  hDebugTrace - Debug Trace object
 *
 * RETURNS:  
 *
 *****************************************************************************/
void debugTrace_Enable(TI_HANDLE hDebugTrace)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    FwEvent_Enable(pDebugTrace->hFwEvent, ACX_INTR_TRACE_A);
    FwEvent_Enable(pDebugTrace->hFwEvent, ACX_INTR_TRACE_B);

    WLAN_REPORT_INFORMATION(pDebugTrace->hReport, HAL_CTRL_MODULE_LOG, 
                            ("whal_DebugTrace_tEnable: Trace Enable \n"));
}

 /****************************************************************************
 *                      debugTrace_Print()
 *****************************************************************************  
 * DESCRIPTION: Print the current buffer
 * 
 * INPUTS:  hDebugTrace - Debug Trace object
 *
 * RETURNS:  
 *
 *****************************************************************************/
void debugTrace_Print(TI_HANDLE hDebugTrace)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    UINT32 index;

    WLAN_OS_REPORT(("Trace buffer size = %d\n",pDebugTrace->TraceBufferSize));

    for(index = 0; index < pDebugTrace->TraceBufferSize / 4 ; index+=2)
    {
        WLAN_OS_REPORT(("0x%08X 0x%08X\n",
                        pDebugTrace->debugWords[index],
                        pDebugTrace->debugWords[index+1]));
    }
}

 /****************************************************************************
 *                      debugTrace_EnablePrint()
 *****************************************************************************  
 * DESCRIPTION: enable printing on each interrupt
 * 
 * INPUTS:  hDebugTrace - Debug Trace object
 *
 * RETURNS:  
 *
 *****************************************************************************/
void debugTrace_EnablePrint(TI_HANDLE hDebugTrace)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    pDebugTrace->bDoPrint = TRUE;

    WLAN_OS_REPORT(("whal_DebugTrace_t_EnablePrint: Print Enabled \n"));

}

 /****************************************************************************
 *                      debugTrace_DisablePrint()
 *****************************************************************************  
 * DESCRIPTION: disable printing on each interrupt
 * 
 * INPUTS:  hDebugTrace - Debug Trace object
 *
 * RETURNS:  
 *
 *****************************************************************************/
void debugTrace_DisablePrint(TI_HANDLE hDebugTrace)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    pDebugTrace->bDoPrint = FALSE;

    WLAN_OS_REPORT(("whal_DebugTrace_t_DisablePrint: Print Disabled \n"));

}

 /****************************************************************************
 *                      debugTrace_UdpEnable()
 *****************************************************************************  
 * DESCRIPTION: enable sending udp on each interrupt
 * 
 * INPUTS:  hDebugTrace - Debug Trace object
 *
 * RETURNS:  
 *
 *****************************************************************************/
void debugTrace_UdpEnable(TI_HANDLE hDebugTrace)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    pDebugTrace->bUdpSend = TRUE;

    WLAN_OS_REPORT(("whal_DebugTrace_t_UdpEnable: Enable UDP trace send\n"));
}

 /****************************************************************************
 *                      debugTrace_UdpDisable()
 *****************************************************************************  
 * DESCRIPTION: disable sending udp on each interrupt
 * 
 * INPUTS:  hDebugTrace - Debug Trace object
 *
 * RETURNS:  
 *
 *****************************************************************************/
void debugTrace_UdpDisable(TI_HANDLE hDebugTrace)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;

    pDebugTrace->bUdpSend = FALSE;

    WLAN_OS_REPORT(("whal_DebugTrace_t_UdpEnable: Disable UDP trace send\n"));

}

#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
 /****************************************************************************
 *                      debugTrace_handleBuffer()
 *****************************************************************************  
 * DESCRIPTION: 
 * 
 * INPUTS:  hDebugTrace - Debug Trace object
 *
 * RETURNS:  
 *
 *****************************************************************************/
void debugTrace_handleBuffer(TI_HANDLE hDebugTrace)
{
    DebugTrace_t *pDebugTrace = (DebugTrace_t *)hDebugTrace;
    mem_MSDU_T *pMsdu;
    mem_BD_T *pBd;
    UINT8    *srcDataPtr;
    int Stt;
    int AcxMpduLen;

    /* The Msdu Len should be The IP total Length + the Ethernet header length */
    AcxMpduLen = wlan_ntohs(pDebugTrace->FrameHeader.IP_Header.Totallen) 
                    + sizeof(pDebugTrace->FrameHeader.Ether_Header);

    /*
     * allocate host msdu
     */
    Stt = wlan_memMngrAllocMSDU(pDebugTrace->hMemMgr, &pMsdu, AcxMpduLen, TRACE_BUFFER_MODULE);
    if (Stt != OK)
    {
        WLAN_REPORT_ERROR(pDebugTrace->hReport, HAL_HW_DATA_MODULE_LOG,  
            ("whal_DebugTrace_t_handleBuffer: wlan_memMngrAllocMSDU error\n"));
        return ;
    }

    /* set the lastBDPtr */
    pBd = pMsdu->firstBDPtr;
    while (pBd->nextBDPtr != NULL)
        pBd = pBd->nextBDPtr;
    
    pMsdu->lastBDPtr = pBd;

    /* Get the Address of the Data filed in the Msdu */
    srcDataPtr = (UINT8 *)memMgr_MsduHdrAddr(pMsdu);

    /* Copy the Frame Header */
    os_memoryCopy(pDebugTrace->hOs, srcDataPtr, &pDebugTrace->FrameHeader, sizeof(pDebugTrace->FrameHeader));
    
    /* Copy the Trace Data */
    os_memoryCopy(pDebugTrace->hOs, srcDataPtr + sizeof(pDebugTrace->FrameHeader), pDebugTrace->debugWords, pDebugTrace->TraceBufferSize);

    memMgr_BufLength(pMsdu->firstBDPtr) = AcxMpduLen;
    
    pMsdu->dataLen = AcxMpduLen;

    /* send the packet to the OS */
    os_receivePacket(pDebugTrace->hOs, pMsdu, (UINT16)pMsdu->dataLen);
}

 /****************************************************************************
 *                      debugTrace_BuildFrame()
 *****************************************************************************  
 * DESCRIPTION: Build the Trace Frame to be send to the OS
 * 
 * INPUTS:  pDebugTrace - Debug Trace object
 *          MacAddress  - for the Ethernet Header
 * RETURNS:  
 *
 *****************************************************************************/
static void debugTrace_BuildFrame(DebugTrace_t *pDebugTrace, UINT8* MacAddress)
{
    TraceFrame *Frame =  &pDebugTrace->FrameHeader;
    UINT32 i;
    UINT16 local_csum = 0;
    UINT16 *pShortData = (UINT16 *)&Frame->IP_Header;

    /*
     *  Calculate each Frame Length
     */
    /* UDP length should Be !024 of Trace Buffet Size + 8 Byte of UDP header 1032) */   
    Frame->UDP_Header.UDP_len = wlan_htons(pDebugTrace->TraceBufferSize + sizeof(Frame->UDP_Header));

    /* IP length should Be UDP length + IP header  */   
    Frame->IP_Header.Totallen = wlan_htons(wlan_htons(Frame->UDP_Header.UDP_len) + sizeof(Frame->IP_Header));

    /*
     * initialize the Ethernet Header
     */
    os_memoryCopy(pDebugTrace->hOs, (UINT8 *)Frame->Ether_Header.ether_dst, 
                  MacAddress, 6);
                     
    Frame->Ether_Header.ether_Type  = wlan_htons (Ether_Type); 
    
    /*
     *  initialize the IP Header
     */
    Frame->IP_Header.IpVer = IP_Version;

    Frame->IP_Header.TimeToLive = IP_TimeToLive;

    Frame->IP_Header.protocol = IP_Protocol;
    
    /* Set the Local host IP */
    Frame->IP_Header.dst[0] = Frame->IP_Header.src[0] = 127;
    Frame->IP_Header.dst[1] = Frame->IP_Header.src[1] = 0 ;
    Frame->IP_Header.dst[2] = Frame->IP_Header.src[2] = 0;
    Frame->IP_Header.dst[3] = Frame->IP_Header.src[3] = 1;

    /* calculate check sum on words of 16 bits */
    for (i=0; i<sizeof(Frame->IP_Header)/2; i++)
        local_csum += pShortData[i];

    Frame->IP_Header.csum = 0xFFFF - local_csum;

    /*
     *  initialize the UDP Header
     */ 
    Frame->UDP_Header.src_port = wlan_htons( Source_Port );
    Frame->UDP_Header.dst_port = wlan_htons( Destination_Port );
}
#endif /* !defined(GWSI_DRIVER) && !defined(GWSI_LIB) */

#endif /* TI_DBG */

