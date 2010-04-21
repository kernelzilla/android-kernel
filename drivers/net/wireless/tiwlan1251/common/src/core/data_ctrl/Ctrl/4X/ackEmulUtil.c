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
/*		MODULE:	ackEmulUtil.c											   */
/*   PURPOSE:	Ack emulation utility 	              					   */
/*															    		   */
/***************************************************************************/
#include "osApi.h"
#include "802_11Defs.h"
#include "ackEmulDb.h"
#include "report.h"
#include "utils.h"
#include "memMngrEx.h"
#include "ackEmulDb.h"
#include "ackEmulUtil.h"
#include "whalCtrl_api.h"


/* The minimum of segment size in byte to lock the segment size */
#define MIN_LOCK_SEGMENT_SIZE 500

typedef enum {
	OTHER    =0,
      TCP_ACK_ONLY ,
      TCP_DATA    ,
} packetInclude_T;



#define TI_SNAP_HEADER_LEN 8
#define TI_SNAP_AE_TYPE 0x02
#define TI_SNAP_AE_LEN  3

#define OFFSET_802_3_HDR	14

#define K_FACTOR 14

/* TI Snap Header */
/*  -------------------------------------------
   | dsap | SSAP | Control |    OUI   |  Type  |
   | 0xAA | 0xAA |   0x00  | 0x080028 | 0x60D0 |
    -------------------------------------------  */

static UINT8 tiSnapHeader[]={0xAA,0xAA,0x00,0x08,0x00,0x28,0x60,0xD0};

static void wdrv_ackEmulationPktType(ackEmul_t*		ackEmul, UINT8 *pWlanSnapHeader ,UINT8 *pIpHeader , packetInclude_T *packetInclude, UINT16 *tcpDataSize);
static void wdrv_ackEmulationDataStandbyState(ackEmul_t*		ackEmul, UINT8 sessionIndex, UINT8 *pIpHeader);
static void wdrv_ackEmulationAckStandbyState(ackEmul_t*		ackEmul, UINT8 sessionIndex, UINT8 *pIpHeader);
static void wdrv_ackEmulationAckCandidateActivState(ackEmul_t*		ackEmul, UINT8 sessionIndex, UINT8 *pIpHeader,
																	 UINT8 *addYTag, UINT8 *activeIndex,
																	 UINT32 *segmentSize);
static void wdrv_ackEmulationAckActivState(ackEmul_t*		ackEmul, UINT8 sessionIndex, UINT8 *pIpHeader,UINT8 *dropAck);
static void wdrv_ackEmulationAckTerminateState(ackEmul_t*		ackEmul, UINT8 sessionIndex);

static void wdrv_ackEmulationAddYTag(ackEmul_t*		ackEmul, UINT8 *pDot11Header, UINT16 firstBdLength,
												 UINT8 activeIndex, UINT32 segmentSize);

static int wdrv_aeChackSnapWithYtag(ackEmul_t*		ackEmul, UINT8 *pSnapHeader, UINT8 *tiSnapLen,
										  UINT8 *activeIndex, UINT16 *segmentSize);

static void wdrv_aeGenerateAck(ackEmul_t*		ackEmul, UINT16 stationIndex, UINT8 activeIndex ,UINT32 ackNumber);


static UINT16 wdrv_IpChecksumCalc(ackEmul_t*		ackEmul, UINT16 len_ip_header, UINT8 *buff);
static UINT16 wdrv_TcpChecksumCalc(ackEmul_t*		ackEmul, UINT16 len_tcp_header,UINT8 *IpSource, UINT8 *IpDest ,UINT8 *buff);




ackEmul_t* ackEmul_create(TI_HANDLE hOs)
{
	ackEmul_t*		ackEmul;
	ackEmulDB_t*	ackEmulDB;

	if( hOs  == NULL )
	{
	    WLAN_OS_REPORT(("FATAL ERROR: ackEmul_create(): OS handle Error - Aborting\n"));
		return NULL;
	}

	ackEmul = os_memoryAlloc(hOs, (sizeof(ackEmul_t)));

	ackEmulDB = ackEmulDb_create(hOs);

	if ( (!ackEmul) || (!ackEmulDB) )
	{
		utils_nullMemoryFree(hOs, ackEmul, sizeof(ackEmul_t));
		utils_nullMemoryFree(hOs, ackEmulDB, sizeof(ackEmulDB_t));
	    WLAN_OS_REPORT(("FATAL ERROR: ackEmul_create(): Error Creating ackEmulDB module- Aborting\n"));
		return(NULL);
	}

	/* reset control module control block */
	os_memoryZero(hOs, ackEmul, (sizeof(ackEmul_t)));

	ackEmul->pAckEmulDB = ackEmulDB;

	ackEmul->hOs = hOs;

	return(ackEmul);


}

TI_STATUS ackEmul_config(ackEmul_t*		ackEmul,
							TI_HANDLE	hWhalCtrl,
							TI_HANDLE	hOs,
							TI_HANDLE	hReport,
							TI_HANDLE	hMemMngr)
{
	/* check parameters validity */
	if( (ackEmul == NULL) || (hOs == NULL) || (hReport == NULL)|| 
		(hWhalCtrl == NULL) || (hMemMngr == NULL) )
	{
	    WLAN_OS_REPORT(("FATAL ERROR: ackEmul_config(): Parameters Error - Aborting\n"));
		return NOK;
	}

	/* set objects handles */
	ackEmul->hOs = hOs;
	ackEmul->hReport = hReport;
	ackEmul->hMemMngr = hMemMngr;
	ackEmul->hWhalCtrl = hWhalCtrl;

	ackEmul->ackEmulationActive = TRUE;

	ackEmulDb_config(ackEmul->pAckEmulDB,hOs,hReport);

	/*whalCtrl_setSend4xWackInfo(ackEmul->hWhalCtrl, 0);           */

	WLAN_REPORT_INIT(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
		(".....ackEmul configured successfully\n"));

	return OK;

}

TI_STATUS ackEmul_destroy(ackEmul_t*	ackEmul)
{
	ackEmulDb_destroy(ackEmul->pAckEmulDB);

	/* free control module controll block */
	os_memoryFree(ackEmul->hOs, ackEmul, sizeof(ackEmul_t));

	return OK;


}


/**********************************************************************************
 *                      wdrv_ackEmulationRxPacket()
 **********************************************************************************
 * DESCRIPTION:	This is the Ack emulation packet receiver main function.
 * 
 * INPUTS:   *pMsdu	 - Pointer to the packet MSDU.
 * 
 * OUTPUT:   None
 * 
 * RETURNS:		None 
 **********************************************************************************/
TI_STATUS wdrv_ackEmulationRxPacket(ackEmul_t*		ackEmul, mem_MSDU_T *pMsdu)
{
   
   UINT8 rc;
   UINT8 *dataBuf;
   dot11_header_t *pDot11Header;
   UINT8 *pWlanSnapHeader;
   UINT8 *pSnapHeader;
   UINT8 *pIpHeader;
   UINT16 tcpDataSize;
   packetInclude_T packetInclude;
   UINT8 sessionIndex, monitorState;
   UINT8 activeIndex = 0xff;
   UINT16 segmentSize = 0;
   UINT8 tiSnapLen = 0;
   UINT16 dataLen;
   UINT8 WTsessionIndex =0xff;
   UINT8 XTag =0;


   if (ackEmul->ackEmulationActive == FALSE)
      return OK;


   dataBuf = (UINT8 *)memMgr_BufData(pMsdu->firstBDPtr)+ memMgr_BufOffset(pMsdu->firstBDPtr);
   dataLen = memMgr_BufLength(pMsdu->firstBDPtr);
   pDot11Header = (dot11_header_t*) dataBuf;
   /* Find the station Index  */

   
   XTag = ((pDot11Header->fc)&0x8000)>>15;
   pSnapHeader = dataBuf + WLAN_HDR_LEN;
   
   pWlanSnapHeader = dataBuf + WLAN_HDR_LEN;
   pIpHeader = pWlanSnapHeader + WLAN_SNAP_HDR_LEN;

   /* Chack if SNAP with Y TAG */
   rc = wdrv_aeChackSnapWithYtag(ackEmul, pSnapHeader, &tiSnapLen, &activeIndex, &segmentSize);
   if(rc == OK)
   {
   		WLAN_REPORT_WARNING(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
				("Packet with Ytag........\n"));

      /* remove the TI SMAP */
      os_memoryMove(NULL, pSnapHeader,pSnapHeader + tiSnapLen, dataLen - WLAN_HDR_LEN - tiSnapLen);
      pMsdu->firstBDPtr->length -= tiSnapLen ;
      pMsdu->dataLen -= tiSnapLen;

         /* Update table and save template */
         wdrv_aeWSourceSaveAckTemplate(ackEmul->pAckEmulDB, 0,activeIndex,dataBuf,pWlanSnapHeader,pIpHeader,(UINT16)pMsdu->dataLen,segmentSize);

      
      return OK;
   }
	
   /* Packet without ack emulation TI Snap */
	
   /* Find the packet type and length */
   wdrv_ackEmulationPktType(ackEmul, pWlanSnapHeader,pIpHeader,&packetInclude,&tcpDataSize);
	
	switch (packetInclude)
   {
   case TCP_DATA:
   		WLAN_REPORT_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
				("RX packet = TCP_DATA....\n"));

      rc = wdrv_aeWTargetDbFindDataSession(ackEmul->pAckEmulDB, pIpHeader ,&sessionIndex, &monitorState);
      if(rc == OK)
      {
    		WLAN_REPORT_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
				("RX packet XTAG = %d......\n",XTag));

         wdrv_aeDbSetXTagStatus(ackEmul->pAckEmulDB, sessionIndex, XTag);
      } 
      switch (monitorState)
      {
      case AE_INACTIVE:
            rc = wdrv_aeWTargetDbAddSession(ackEmul->pAckEmulDB, pIpHeader);
         
			break;
		case AE_STANDBY:
         wdrv_ackEmulationDataStandbyState(ackEmul, sessionIndex, pIpHeader);
			break;
		case AE_CANDIDATE_ACTIVE:
			break;
		case AE_ACTIVE:
      default:
			break;
      }
      /*         printf("\n rc= %d  sessionIndex = %d monitorState = %d\n",rc, sessionIndex , monitorState);*/
      break;
      case TCP_ACK_ONLY:
     		WLAN_REPORT_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
				("RX packet = TCP_ACK_ONLY....\n"));

       wdrv_aeWSourceDbUpdateTemplate(ackEmul->pAckEmulDB, pIpHeader,0,&WTsessionIndex);
         break;
      case OTHER:
           WLAN_REPORT_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
				("RX packet = OTHER....\n"));

	  default:
         break;
   }
	/*wdrv_aeWTargetDbPrint(ackEmul->pAckEmulDB);*/
   return OK;
}


/**********************************************************************************
 *                      wdrv_ackEmulationTxPacket()
 **********************************************************************************
 * DESCRIPTION:	This is the Ack emulation packet transmission main function.
 * 
 * INPUTS:   *pMsdu	 - Pointer to the packet MSDU.
 * 
 * OUTPUT:   *discardPacket - If TRUE discard the msdu.
 * 
 * RETURNS:		None 
 **********************************************************************************/
TI_STATUS wdrv_ackEmulationTxPacket(ackEmul_t*		ackEmul, mem_MSDU_T *pMsdu,int *discardPacket)
 {
    UINT8		*pDot11Header;
    UINT8    *pWlanSnapHeader;
    UINT8		*pIpHeader;
    UINT16	tcpDataSize;
    packetInclude_T packetInclude;
    UINT8	sessionIndex = 0xff;
   UINT8	monitorState = 0xff;
   UINT8		addYTag =FALSE;
    UINT8		dropAck =FALSE;
    UINT16	firstBdLength = 0;
    UINT32	firstBddataOffset = 0;
    UINT32	secondBddataOffset = 0;
    UINT8 	activeIndex = 0xff;
    UINT32	segmentSize = 0;
    
    *discardPacket = FALSE;
    pIpHeader     = NULL;

    
    if (ackEmul->ackEmulationActive == FALSE)
       return OK;
    
    firstBdLength = memMgr_BufLength(pMsdu->firstBDPtr);
    firstBddataOffset = memMgr_BufOffset(pMsdu->firstBDPtr) ;
    pDot11Header = (UINT8 *)memMgr_BufData(pMsdu->firstBDPtr)+firstBddataOffset;
    
    if (firstBdLength == WLAN_HDR_LEN + WLAN_SNAP_HDR_LEN)
    {
       /* Packet from Ethernet, the IP heder is in the second data buffer */
       if(pMsdu->firstBDPtr->nextBDPtr != NULL)
       {
          secondBddataOffset = memMgr_BufOffset(pMsdu->firstBDPtr->nextBDPtr);
          pIpHeader = (UINT8 *)memMgr_BufData(pMsdu->firstBDPtr->nextBDPtr)+ secondBddataOffset;
       }
    }
    else
    {
       /* Packet from Wlan  */
       pIpHeader = pDot11Header + WLAN_HDR_LEN + WLAN_SNAP_HDR_LEN;
    }
    
    pWlanSnapHeader = pDot11Header + WLAN_HDR_LEN;
    
   /* Find the packet type and length */
    wdrv_ackEmulationPktType(ackEmul, pWlanSnapHeader,pIpHeader ,&packetInclude,&tcpDataSize);
    
    switch (packetInclude)
    {
    case TCP_ACK_ONLY:
		WLAN_REPORT_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
				("TX packet = TCP_ACK_ONLY....\n"));

       
       wdrv_aeWTargetDbFindAckSession(ackEmul->pAckEmulDB, pIpHeader ,&sessionIndex, &monitorState);
       
       switch (monitorState)
       {
       case AE_INACTIVE:
		   
		   break;
       case AE_STANDBY:
		   
		   wdrv_ackEmulationAckStandbyState(ackEmul, sessionIndex, pIpHeader);
		   break;
       case AE_CANDIDATE_ACTIVE:
		   wdrv_ackEmulationAckCandidateActivState(ackEmul, sessionIndex,
			   pIpHeader,
			   &addYTag,
			   &activeIndex,
			   &segmentSize);
		   if(addYTag == TRUE)
		   {
			   wdrv_ackEmulationAddYTag(ackEmul, pDot11Header, firstBdLength, activeIndex, segmentSize);
			   
			   /* update the MSDU fields */
			   pMsdu->firstBDPtr->length += TI_SNAP_HEADER_LEN+2+TI_SNAP_AE_LEN;
			   pMsdu->dataLen += TI_SNAP_HEADER_LEN+2+TI_SNAP_AE_LEN;
		   }
		   
		   break;
       case AE_ACTIVE:
		   /* Comper Template and drop packet */
		   wdrv_ackEmulationAckActivState(ackEmul, sessionIndex, pIpHeader,&dropAck);
		   
		   *discardPacket = dropAck;
		   break;
		   
       case AE_TERMINATE:
		   wdrv_ackEmulationAckTerminateState(ackEmul, sessionIndex);
		   
		   break;
       default:
		   
		   break;
       }
       
	   break;
	   case TCP_DATA:
		   WLAN_REPORT_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
			   ("TX packet = TCP_DATA....\n"));
		   
		   break;
       case OTHER:
		   WLAN_REPORT_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
			   ("TX packet = OTHER....\n"));
       default:
		   break;
    }
    return OK;
}



/**********************************************************************************
 *                      wdrv_ackEmulationPktType()
 **********************************************************************************
 * DESCRIPTION:	Find the packet type and length.
 * 
 * INPUTS:   pWlanSnapHeader	 - Pointer to the 802.11 SNAP header buffer.
 *           pIpHeader         - Pointer to the ip header header buffer.
 * 
 * OUTPUT:   packetInclude     - The packet type, TCP_DATA, TCP_ACK_ONLY or OTHER
 *           tcpDataSize       - The packet size (only for tcp packet) 
 * 
 * RETURNS:		None 
 **********************************************************************************/
static void wdrv_ackEmulationPktType(ackEmul_t*		ackEmul, UINT8 *pWlanSnapHeader ,UINT8 *pIpHeader ,
                                     packetInclude_T *packetInclude, UINT16 *tcpDataSize)
{
   UINT16 pktTotalLen;
   UINT8 ipHeaderLen, tcpHeaderLen;

   *tcpDataSize =0;



   /* Check if the packet is TCP/IP */
   if ((wlan_ntohs(((Wlan_LlcHeader_T*)pWlanSnapHeader)->Type) == IP_PROTOCOL_NUMBER) &&
      (*(unsigned char*) ((ULONG)pIpHeader+(ULONG)IP_PROTOCOL_FIELD)== TCP_PROTOCOL))
   {
      /* Check if the packet include data or Ack only */
      pktTotalLen = wlan_ntohs((*(UINT16*)(pIpHeader + IP_TOTAL_LEN_FIELD)));
      ipHeaderLen = ((*(unsigned char*)pIpHeader  & 0x0f) * 4);
      tcpHeaderLen = ((((*(unsigned char*)(pIpHeader + ipHeaderLen+TCP_OFFSET_FIELD))& 0xf0)>>4) * 4);
      *tcpDataSize = pktTotalLen - (ipHeaderLen + tcpHeaderLen);

      if(*tcpDataSize > 0)
         *packetInclude = TCP_DATA;
      else 
         *packetInclude = TCP_ACK_ONLY;

   }
   else
   {
      *packetInclude = OTHER;
   }
   
}



/**********************************************************************************
 *                      wdrv_ackEmulationDataStandbyState()
 **********************************************************************************
 * DESCRIPTION:	This function handle the tcp date packet for session in 
 *                standby state.
 * 
 * INPUTS:   sessionIndex	 - the session index
 *           *pIpHeader     - Pointer to the ip header header buffer.
 * 
 * OUTPUT:   None.
 * 
 * RETURNS:	 None 
 **********************************************************************************/
static void wdrv_ackEmulationDataStandbyState(ackEmul_t*		ackEmul, UINT8 sessionIndex, UINT8 *pIpHeader)
{
	UINT8 ipHeaderLen = 0;

	UINT32 prevSequenceNumber =0;
	UINT32 currentSequenceNumber =0;
	
	UINT32 prevSegmentSize = 0;
	UINT32 currentSegmentSize =0;

	UINT8 equalSegmentSizeCounter =0;



	/* Calculate Current Sequence Number */
	ipHeaderLen = ((*(unsigned char*)pIpHeader  & 0x0f) * 4);
	currentSequenceNumber = wlan_ntohl(*(unsigned long*)(pIpHeader+ipHeaderLen+TCP_SEQUENCE_NUMBER_FIELD));

	wdrv_aeWTargetDbGetSessionSequenceNumber(ackEmul->pAckEmulDB, sessionIndex,&prevSequenceNumber);
	/* Calclate Segment Size */
	wdrv_aeWTargetDbGetSessionSegmentSize(ackEmul->pAckEmulDB, sessionIndex,&prevSegmentSize);
	currentSegmentSize = currentSequenceNumber - prevSequenceNumber;

	if (prevSegmentSize == currentSegmentSize)
	{
		/* Increase Equal Segment Size Counter */
		wdrv_aeWTargetDbGetIncrSessionEqualSegmentSizeCounter(ackEmul->pAckEmulDB, sessionIndex,&equalSegmentSizeCounter);
		if(equalSegmentSizeCounter == 2 && currentSegmentSize > MIN_LOCK_SEGMENT_SIZE)
		{
			/* Monitor state -> AE_CANDIDATE_ACTIVE */
/*         printf("\n Session %d chabge state to AE_CANDIDATE_ACTIVE with segment size %d\n",
            sessionIndex,
            currentSegmentSize);*/
			wdrv_aeWTargetDbSetSessionMonitorState(ackEmul->pAckEmulDB, sessionIndex,AE_CANDIDATE_ACTIVE);
		}

	}
	else
	{
		wdrv_aeWTargetDbSetSessionSegmentSize(ackEmul->pAckEmulDB, sessionIndex,currentSegmentSize);
		wdrv_aeWTargetDbSetSessionEqualSegmentSizeCounter(ackEmul->pAckEmulDB, sessionIndex,0);
	}
	
	wdrv_aeWTargetDbSetSessionSequenceNumber(ackEmul->pAckEmulDB, sessionIndex,currentSequenceNumber);	
}


/**********************************************************************************
 *                      wdrv_ackEmulationAckStandbyState()
 **********************************************************************************
 * DESCRIPTION:	This function handle the tcp ack packet for session in 
 *                standby state.
 * 
 * INPUTS:   sessionIndex	 - the session index
 *           *pIpHeader     - Pointer to the ip header header buffer.
 * 
 * OUTPUT:   None.
 * 
 * RETURNS:	 None 
 **********************************************************************************/
static void wdrv_ackEmulationAckStandbyState(ackEmul_t*		ackEmul, UINT8 sessionIndex, UINT8 *pIpHeader)
{

   UINT8 ipHeaderLen = 0;
	UINT32 currentAckNumber =0;

   	/* Calculate Current Ack Number */
	ipHeaderLen = ((*(unsigned char*)pIpHeader  & 0x0f) * 4);
	currentAckNumber = wlan_ntohl(*(unsigned long*)(pIpHeader+ipHeaderLen+TCP_ACK_NUMBER_FIELD));

   	/* Set Current Ack Number */
	wdrv_aeWTargetDbSetSessionAckNumber(ackEmul->pAckEmulDB, sessionIndex,currentAckNumber);



}

/**********************************************************************************
 *                      wdrv_ackEmulationAckCandidateActivState()
 **********************************************************************************
 * DESCRIPTION:	This function handle the tcp ack packet for session in 
 *                candidate activ state.
 * 
 * INPUTS:   sessionIndex	 - the session index
 *           *pIpHeader     - Pointer to the ip header header buffer.
 * 
 * OUTPUT:   *addYTag       - If true, add Ytag to the tcp acp packet
 *           *activeIndex   - The activeIndex that assign for this session
 *           *segmentSize   - The segment size of this tcp session.
 * 
 * RETURNS:	 None 
 **********************************************************************************/
static void wdrv_ackEmulationAckCandidateActivState(ackEmul_t*		ackEmul, UINT8 sessionIndex,
																	 UINT8 *pIpHeader,
																	 UINT8 *addYTag,
																	 UINT8 *activeIndex,
																	 UINT32 *segmentSize)
{

   UINT8 ipHeaderLen = 0;
	UINT32 prevAckNumber =0;
	UINT32 currentAckNumber =0;
   UINT32 ackCounter =0;

   	/* Calculate Current Ack Number */
	ipHeaderLen = ((*(unsigned char*)pIpHeader  & 0x0f) * 4);
	currentAckNumber = wlan_ntohl(*(unsigned long*)(pIpHeader+ipHeaderLen+TCP_ACK_NUMBER_FIELD));

	wdrv_aeWTargetDbGetSessionSegmentSize(ackEmul->pAckEmulDB, sessionIndex,segmentSize);
	wdrv_aeWTargetDbGetSessionAckNumber(ackEmul->pAckEmulDB, sessionIndex,&prevAckNumber);

   if((currentAckNumber - prevAckNumber) == *segmentSize *2)
   {
       WLAN_REPORT_FATAL_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
				("Change State to active............\n"));
      /* Change state to Active and update Active index */
      if(wdrv_aeWTargetDbSetActiveState(ackEmul->pAckEmulDB, sessionIndex, activeIndex) == OK)
      {
         
 
         /* Calculate Ack Counter */
         ackCounter = currentAckNumber / (*segmentSize *2);
         wdrv_aeWTargetDbSetSessionAckCounter(ackEmul->pAckEmulDB, sessionIndex,ackCounter) ;
         /* Save template update Active index */
         wdrv_aeWTargetDbSaveAckTemplate(ackEmul->pAckEmulDB, sessionIndex,pIpHeader);
         /* add addYTag */

		 WLAN_REPORT_FATAL_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
				("ADD Y TAG.........!!!!!!!!!!!!\n"));

			*addYTag = TRUE;

      }
   }
   wdrv_aeWTargetDbSetSessionAckNumber(ackEmul->pAckEmulDB, sessionIndex,currentAckNumber);

}

/**********************************************************************************
 *                      wdrv_ackEmulationAckActivState()
 **********************************************************************************
 * DESCRIPTION:	This function handle the tcp ack packet for session in 
 *                active state.
 * 
 * INPUTS:   sessionIndex	 - the session index
 *           *pIpHeader     - Pointer to the ip header header buffer.
 * 
 * OUTPUT:   *dropAck       - If TRUE discard the packet.
 * 
 * RETURNS:	 None 
 **********************************************************************************/
static void wdrv_ackEmulationAckActivState(ackEmul_t*		ackEmul, UINT8 sessionIndex, UINT8 *pIpHeader,UINT8 *dropAck)
{
   int rc =NOK;

   UINT8 ipHeaderLen = 0;
   UINT32 currentAckNumber   =0;
   UINT32 prevAckNumber   =0;
   UINT32 segmentSize =0;
   UINT32 currentAckCounter =0;
   UINT32 prevAckCounter =0;
   UINT8  activeIndex = 0xff;
   UINT32 segmentNumber = 0;
   UINT32 prevTimeStamp;
   UINT32 currentTimeStamp;
   UINT8 newWackInfo = 0;
   UINT8 prevWackInfo;
   UINT8 prevOwnershipBit;
   UINT8 prevActvIndxBits;
	UINT8 prevAckCounterBits;

   
   currentTimeStamp =os_timeStampUs(ackEmul->hOs); 
 
   wdrv_aeWTargetDbGetSessionTimeStamp(ackEmul->pAckEmulDB, sessionIndex, &prevTimeStamp);
   if(prevTimeStamp != 0)
   {
      if((WTARGET_ACTIVE_TIME_OUT) < (currentTimeStamp - prevTimeStamp))
      {  /* If the delay between two ack packet for the same session grater */
         /* than  WTARGET_ACTIVE_TIME_OUT change session state to terminate */
         wdrv_aeWTargetDbSetSessionMonitorState(ackEmul->pAckEmulDB, sessionIndex,AE_TERMINATE);
       	 /*WLAN_OS_REPORT(("1\n"));*/
		  return;
      }
   }

   
   /* extract the information from prevWackInfo */

   whalCtrl_getSend4xWackInfo(ackEmul->hWhalCtrl, &prevWackInfo);           
   prevAckCounterBits = (prevWackInfo&0x0f);
   prevActvIndxBits = (prevWackInfo&0x10)>>4;
   prevOwnershipBit = (prevWackInfo&0x80)>>7;
   
   /* Calculate Current Ack Number */
   ipHeaderLen = ((*(unsigned char*)pIpHeader  & 0x0f) * 4);
   currentAckNumber = wlan_ntohl(*(unsigned long*)(pIpHeader+ipHeaderLen+TCP_ACK_NUMBER_FIELD));
   
   wdrv_aeWTargetDbGetSessionSegmentSize(ackEmul->pAckEmulDB, sessionIndex,&segmentSize);
   wdrv_aeWTargetDbGetSessionAckNumber(ackEmul->pAckEmulDB, sessionIndex,&prevAckNumber);
   
   wdrv_aeWTargetDbGetSessionAckCounter(ackEmul->pAckEmulDB, sessionIndex, &prevAckCounter) ;
   wdrv_aeWTargetDbGetSessionActiveIndex(ackEmul->pAckEmulDB, sessionIndex, &activeIndex);

   /* Calculate Ack Counter */
   currentAckCounter = currentAckNumber / (segmentSize*2);
   if((currentAckNumber - prevAckNumber) == segmentSize *2)
   {
       	 /*WLAN_OS_REPORT(("--2\n"));*/

	   if ((currentAckCounter > prevAckCounter) && (currentAckCounter < prevAckCounter + 5))
	   {                            
       	 /*WLAN_OS_REPORT(("----3\n"));*/
		   rc = wdrv_aeWTargetDbCmpAckTemplate(ackEmul->pAckEmulDB, sessionIndex, pIpHeader);
		   if (rc == OK)
		   {  
			   /* drop this Ack */
			   
			   UINT8 Xtag;

			   wdrv_aeDbGetXTagStatus(ackEmul->pAckEmulDB, sessionIndex,&Xtag);
		       	 /*WLAN_OS_REPORT(("------4 Xtag=%d prevOwnershipBit=%d activeIndex=%d prevActvIndxBits=%d\n",Xtag,prevOwnershipBit,activeIndex,prevActvIndxBits));*/
			   if (((prevOwnershipBit == 1)|| (activeIndex == prevActvIndxBits)) && Xtag)
			   {
				   *dropAck = TRUE;
				   
			   }
		   }
	   }
   }
   else
   {
		/*WLAN_OS_REPORT(("------99 prevAckNumber=%d,currentAckCounter=%d currentAckNumber=%d segmentSize=%d \n",*/
		/*						  prevAckNumber,   currentAckCounter,   currentAckNumber,   segmentSize));*/
   }

  WLAN_REPORT_FATAL_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
		   ("dropAck = %d\n",*dropAck));
   
   if(*dropAck == TRUE)
   {
	   newWackInfo = (((currentAckCounter % K_FACTOR)+1)&0xf); /* Add currentAckCounter MOD K_FACTOR to bit 0-3 */   
	   newWackInfo = newWackInfo | (activeIndex << 4);  /* Add activeIndex to bit 4 */
	   whalCtrl_setSend4xWackInfo(ackEmul->hWhalCtrl, newWackInfo);           
	   wdrv_aeWTargetDbSetSessionTimeStamp(ackEmul->pAckEmulDB, sessionIndex, currentTimeStamp);
   }
   else
   {
	   
	   if((prevAckNumber % (segmentSize*2))>(currentAckNumber % (segmentSize*2)))
	   {
		   /* Ack Reorder Problem */
		   newWackInfo = 0xF; 
	   }
	   else
	   {
		   if(prevAckCounterBits == 0xF)
		   {
			   /* chack if this wack info send */
			   if(prevOwnershipBit == 1)
				   newWackInfo = 0;
		   }
		   else
		   {
               newWackInfo = 0;
		   }
	   }
	   newWackInfo = newWackInfo | (activeIndex << 4);  /* Add activeIndex to bit 4 */
	   whalCtrl_setSend4xWackInfo(ackEmul->hWhalCtrl, newWackInfo);           
	   
	   wdrv_aeWTargetDbSetSessionAckCounter(ackEmul->pAckEmulDB, sessionIndex,currentAckCounter) ;
	   
   }
   wdrv_aeWTargetDbSetSessionAckNumber(ackEmul->pAckEmulDB, sessionIndex,currentAckNumber);
   segmentNumber = wlan_ntohl(*(unsigned long*)(pIpHeader+ipHeaderLen+TCP_SEQUENCE_NUMBER_FIELD));
   wdrv_aeWTargetDbUpdateAckTemplate(ackEmul->pAckEmulDB, sessionIndex,segmentNumber);
   
}


/**********************************************************************************
 *                      wdrv_ackEmulationAckTerminateState()
 **********************************************************************************
 * DESCRIPTION:	This function handle the tcp ack packet for session in 
 *                terminate state.
 * 
 * INPUTS:   sessionIndex	 - 
 * 
 * OUTPUT:   None
 * 
 * RETURNS:		None 
 **********************************************************************************/
static void wdrv_ackEmulationAckTerminateState(ackEmul_t*		ackEmul, UINT8 sessionIndex)
{
   UINT32 prevTimeStamp;
   UINT32 currentTimeStamp = os_timeStampUs(ackEmul->hOs);

   wdrv_aeWTargetDbGetSessionTimeStamp(ackEmul->pAckEmulDB, sessionIndex, &prevTimeStamp);
   if(prevTimeStamp != 0)
   {
      if((WTARGET_TERMINATE_TIME_OUT) < (currentTimeStamp - prevTimeStamp))
      {
         /* Reset Session */
         wdrv_aeWTargetDbResetTuple(ackEmul->pAckEmulDB, sessionIndex); 
      }
   }
   return;
}

/**********************************************************************************
 *                      wdrv_ackEmulationAddYTag()
 **********************************************************************************
 * DESCRIPTION: Add Ytag to the Tcp Ack PAcket
 * 
 * INPUTS:   *pDot11Header	 - Pointer to the 802.11 heder
 *           firstBdLength  - The length of the first BD
 *           activeIndex    - The active index of the monitor session. 
 *           segmentSize    - The segmentSize of the monitor session. 
 * 
 * OUTPUT:     None 
 * 
 * RETURNS:		None 
 **********************************************************************************/

/* Tcp Ack without Ytag */
/*  -------------------------------------------
   | 802.11 | SSAP |   IP   |  TCP   | Payload |
   | header |      | header | header |         |
    -------------------------------------------  */
/* Tcp Ack with Ytag */
/*  ---------------------------------------------------------
   | 802.11 |  TI  | 802.11 SNAP |   IP   |  TCP   | Payload |
   | header | SNAP | header      | header | header |         |
    ---------------------------------------------------------  */


/* TI Snap */
/*  -------------------------------------------------------------------------
   | dsap | SSAP | Control |    OUI   |  Type  |  AE  | AE  |Active |Segment |
   | 0xAA | 0xAA |   0x00  | 0x080028 | 0x60D0 | Type | Len | Index |  Size  |
    ------------------------------------------------------------------------- */


static void wdrv_ackEmulationAddYTag(ackEmul_t*		ackEmul, UINT8 *pDot11Header, UINT16 firstBdLength,
												 UINT8 activeIndex, UINT32 segmentSize)
{
	UINT8 *tempPtr;
	UINT32 moveLen;
	
	tempPtr = pDot11Header + WLAN_HDR_LEN;
	moveLen = firstBdLength - WLAN_HDR_LEN;
	os_memoryMove(NULL, tempPtr+TI_SNAP_HEADER_LEN+2+TI_SNAP_AE_LEN ,tempPtr,moveLen);
	os_memoryCopy(NULL, tempPtr,tiSnapHeader,TI_SNAP_HEADER_LEN);
	tempPtr += TI_SNAP_HEADER_LEN;
   *(UINT8*)tempPtr = TI_SNAP_AE_TYPE; 
	tempPtr ++;
	*(UINT8*)tempPtr = TI_SNAP_AE_LEN; /* Length */
	tempPtr ++;
	*(UINT8*)tempPtr = activeIndex; /* Active Index */
	tempPtr ++;
	*(UINT16*)tempPtr = wlan_ntohs((UINT16)segmentSize); /* Segment Size */
}

/**********************************************************************************
 *                      wdrv_aeChackSnapWithYtag()
 **********************************************************************************
 * DESCRIPTION: Check if the packet include Ytag 
 * 
 * INPUTS:   *pSnapHeader - Pointer to the SNAP heder
 * 
 * OUTPUT:   *tiSnapLen      - The length of the first BD
 *           *activeIndex    - The active index. 
 *           *segmentSize    - The segmentSize.       
 * 
 * RETURNS:		Ok if include Ytag, otherwise NOK
 **********************************************************************************/
static int wdrv_aeChackSnapWithYtag(ackEmul_t*		ackEmul, UINT8 *pSnapHeader, UINT8 *tiSnapLen,
                                    UINT8 *activeIndex, UINT16 *segmentSize)
{
   
	/* Chack if SNAP with Y TAG */
   if ((os_memoryCompare(ackEmul->hOs, pSnapHeader,tiSnapHeader,TI_SNAP_HEADER_LEN)== 0)&&
      (*(UINT8*)(pSnapHeader + TI_SNAP_HEADER_LEN) == TI_SNAP_AE_TYPE))
   {
      
      /* This packet include Ack with Y TAG */
		UINT8* tempPtr;
		UINT8 templen;
		tempPtr = pSnapHeader + TI_SNAP_HEADER_LEN+1;
		templen = *(UINT8*)tempPtr;
		*tiSnapLen = TI_SNAP_HEADER_LEN + 2 + templen;     
		tempPtr ++;
		*activeIndex = (UINT8)(wlan_ntohs(*(UINT16*)tempPtr));
		tempPtr +=2;
		*segmentSize = (UINT16)(wlan_ntohl(*(UINT32*)tempPtr));
		return OK;
	}
	else
	{
		return NOK;
	}
}

int genera = 0;
int modulo_off = 0;
int mod_off = 0;
int mod_on = 0;

/**********************************************************************************
 *                      wdrv_aeWackReceive()
 **********************************************************************************
 * DESCRIPTION:	Parse the wack info and generate emulated tcp ack
 * 
 * INPUTS:   station	 - The wack source station index.
 *           wackInfo - The wackInfo  
 * 
 * OUTPUT:   None    
 * 
 * RETURNS:		None 
 **********************************************************************************/
void wdrv_aeWackReceive(ackEmul_t*		ackEmul, UINT16 station, UINT8 wackInfo)
{
	UINT8 ackCounterlowBits;
	UINT8 activeIndex;
	UINT32 oldAckNumber;
	UINT32 newAckNumber;
	UINT32 oldAckCounter;
	UINT32 newAckCounter;
	UINT32 segmentSize;
	UINT32 prevWackTimeStamp;
	UINT32 currentWackTimeStamp;
	UINT32  reorderProblemStatus;
	
	static UINT8 prev_wackInfo = 0xff;
	
	if(prev_wackInfo != wackInfo)
	{
		prev_wackInfo = wackInfo;
		
		
		/* extract the information from wackInfo */
		ackCounterlowBits = (wackInfo & 0xf);
		activeIndex      = (wackInfo & 0x10) >> 4;
		
		
		if (ackCounterlowBits == 0)
			ackCounterlowBits = 0xF;
		else
			ackCounterlowBits --;
		
		
		if(ackCounterlowBits == 0xF)
			return;
		
		if(ackCounterlowBits == 0xE)
		{
			/* Reorder problem is possible */
			wdrv_aeWSourceDbSetSessionAckReorderProblem(ackEmul->pAckEmulDB, station,activeIndex,REORDER_PROBLEM_ON);
			return;
		}
		wdrv_aeWSourceDbGetSessionTimeStamp(ackEmul->pAckEmulDB, station,activeIndex,&prevWackTimeStamp);
		if(prevWackTimeStamp != 0)
		{
			currentWackTimeStamp = os_timeStampUs(ackEmul->hOs);
			
			if((WSOURCE_SESSION_TIME_OUT) < (currentWackTimeStamp - prevWackTimeStamp))
			{
				/* reset Wsource session */
				/*WLAN_OS_REPORT((" Reset Wsource session  activeIndex %\n",activeIndex));*/
				
				wdrv_aeWSourceDbResetSession(ackEmul->pAckEmulDB, station, activeIndex);
				
				return;
			}
			
			wdrv_aeWSourceDbSetSessionTimeStamp(ackEmul->pAckEmulDB, station,activeIndex,currentWackTimeStamp);
			
			wdrv_aeWSourceDbGetSessionSegmentSize(ackEmul->pAckEmulDB, station,activeIndex,&segmentSize); 
			
			if(segmentSize != 0)
			{
				wdrv_aeWSourceDbGetSessionAckNumber(ackEmul->pAckEmulDB, station,activeIndex,&oldAckNumber); 
				wdrv_aeWSourceDbGetSessionAckCounter(ackEmul->pAckEmulDB, station,activeIndex,&oldAckCounter); 
				
				/* newACK_counter = (Low_counter - (oldACK_counter mod 2^k) + 2^k + 3) mod 2^k - 3 + oldACK_counter*/
				newAckCounter = ((ackCounterlowBits - (oldAckCounter % K_FACTOR) + K_FACTOR + 3) % K_FACTOR) -3 + oldAckCounter;
				
				if(newAckCounter <= oldAckCounter)
					return;
				
				wdrv_aeWSourceDbGetSessionAckReorderProblem(ackEmul->pAckEmulDB, station,activeIndex,&reorderProblemStatus);
				
				if(reorderProblemStatus == REORDER_PROBLEM_ON)     
				{
					newAckNumber = (newAckCounter * (segmentSize*2));
				}
				else
				{
					/* newACK_number = oldACK_number mod (Segment_Size*2) + Rx_Ack_count * (Segment_Size*2)*/      
					newAckNumber = (oldAckNumber % (segmentSize*2)) + (newAckCounter * (segmentSize*2));
				}
				/* Generate ack */
				genera++;
				wdrv_aeGenerateAck(ackEmul, station,activeIndex,newAckNumber);
				
				wdrv_aeWSourceDbSetSessionAckNumber(ackEmul->pAckEmulDB, station,activeIndex,newAckNumber); 
				wdrv_aeWSourceDbSetSessionAckCounter(ackEmul->pAckEmulDB, station,activeIndex,newAckCounter); 
			}
			return;
			
		}

	}
}
/**********************************************************************************
 *                      wdrv_aeGenerateAck()
 **********************************************************************************
 * DESCRIPTION: Generate emulated TCP Ack
 * 
 * INPUTS:   sessionIndex	 - 
 *           activeIndex
 *           ackNumber
 * 
 * OUTPUT:   None     
 * 
 * RETURNS:		None 
 **********************************************************************************/
 static void wdrv_aeGenerateAck(ackEmul_t*		ackEmul, UINT16 stationIndex, UINT8 activeIndex ,UINT32 ackNumber)
 {
	 mem_MSDU_T *pMsdu;
	 UINT8 ipHeaderLen;
	 UINT8 tcpHeaderLen;
	 UINT8 *pTeplate;
	 UINT8 *pNewPkt;
	 UINT16  newPktLen;
	 UINT8 *ipSource, *ipDest;
	 UINT8 *pIpHeader, *pTcpHeader;
	 UINT16 ipChecksum =0;
	 UINT16 tcpChecksum =0;
	 
	 dot11_header_t *pDot11Header;
	 UINT8 *pDAddr,*pSAddr;

	WLAN_REPORT_ERROR(ackEmul->hReport, ACK_EMUL_MODULE_LOG, 
				("wdrv_aeGenerateAck....==============================================\n"));

	 wdrv_aeWSourceDbGetAckTemplate(ackEmul->pAckEmulDB, stationIndex, activeIndex, &pTeplate,&ipHeaderLen, &tcpHeaderLen);
	 if(ipHeaderLen > 0)
	 {
		 newPktLen = ipHeaderLen+tcpHeaderLen;
		 if(wlan_memMngrAllocMSDU(ackEmul->hMemMngr, &pMsdu,newPktLen+OFFSET_802_3_HDR,(allocatingModule_e)(ACK_EMUL_MODULE+ sizeof(DbTescriptor)))==NOK)
		 {
			 WLAN_OS_REPORT(( "WDRV_4X: GenerateAck - fail to allocate buildMsduPtr\n"));
			 return;
		 }
		 pNewPkt = (UINT8*)memMgr_BufData(pMsdu->firstBDPtr)+ sizeof(DbTescriptor);

		 os_memoryCopy(ackEmul->hOs, pNewPkt + OFFSET_802_3_HDR -2,pTeplate + WLAN_HDR_LEN+WLAN_SNAP_HDR_LEN -2,newPktLen+2);
		 
		 /* Extract Info from frame header */
		 pDot11Header = (dot11_header_t *)pTeplate;
		 
		 pDAddr = (UINT8 *)&pDot11Header->address1;
		 pSAddr = (UINT8 *)((pDot11Header->fc & DOT11_FC_FROM_DS)? &pDot11Header->address3 : &pDot11Header->address2);
		 
		 pMsdu->firstBDPtr->dataOffset =  sizeof(DbTescriptor);
		 pMsdu->firstBDPtr->length = newPktLen;
		 pMsdu->dataLen = newPktLen+OFFSET_802_3_HDR;
		 
		 /* Calclate IP Checksum */
		 pIpHeader = pNewPkt+OFFSET_802_3_HDR;
		 ipChecksum = wdrv_IpChecksumCalc(ackEmul, ipHeaderLen,pIpHeader);
		 
		 /* Calclate TCP Checksum */
		 pTcpHeader = pIpHeader + ipHeaderLen;
		 *(UINT32*)(pTcpHeader+TCP_ACK_NUMBER_FIELD) = wlan_ntohl(ackNumber);
		 ipSource = pIpHeader+IP_SRC_ADDRESS_FIELD;
		 ipDest   = pIpHeader+IP_DEST_ADDRESS_FIELD;
		 tcpChecksum = wdrv_TcpChecksumCalc(ackEmul, tcpHeaderLen, ipSource, ipDest, pTcpHeader);
		 /* Add the Checksum to the new packet */
		 *(UINT16*)(pIpHeader+IP_CHECKSUM_FIELD) = wlan_ntohs(ipChecksum);
		 *(UINT16*)(pTcpHeader+TCP_CHECKSUM_FIELD) = wlan_ntohs(tcpChecksum);
		 
		 /* Generate 802.3 Header */
		 os_memoryCopy (ackEmul->hOs, pNewPkt, pDAddr, MAC_ADDR_LEN);
		 os_memoryCopy (ackEmul->hOs, pNewPkt+MAC_ADDR_LEN, pSAddr, MAC_ADDR_LEN);
		 
		 /* Send the emulated ack to the bss Bridge*/
		 os_receivePacket(ackEmul->hOs, pMsdu, (UINT16)pMsdu->dataLen);
	 }
 }
 
 /****************************************************************************
 *                      wdrv_aeSetActive()
 ****************************************************************************
 * DESCRIPTION:	Enable/ Desable Ack Emulation
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/

void wdrv_aeSetActive(ackEmul_t*		ackEmul, int status)
{
   
/*   if (status == TRUE)
   {
      (void)whal_apiSetSend4xWackInfo(ackEmul->hWhalCtrl, 0);
      (void)whal_apiEnable4xWackFeature(ackEmul->hWhalCtrl);
   }
   if (status == FALSE)
   {
      (void)whal_apiSetSend4xWackInfo(ackEmul->hWhalCtrl, 0);
      (void)whal_apiDisable4xWackFeature(ackEmul->hWhalCtrl);
   }
*/   
      ackEmul->ackEmulationActive = status;
}



int wdrv_aeGetActive(ackEmul_t*		ackEmul )
{
   return(ackEmul->ackEmulationActive);
}



/*
**************************************************************************
Function: wdrv_IpChecksumCalc
Description: Calculate the 16 bit IP Checksum.
***************************************************************************
*/


static UINT16 wdrv_IpChecksumCalc(ackEmul_t*		ackEmul, UINT16 len_ip_header, UINT8 *buff)
{
UINT16 word16;
UINT32 sum=0;

UINT16 i;
	/* make 16 bit words out of every two adjacent 8 bit words in the packet*/
	/* and add them up*/
	for (i=0;i<len_ip_header;i=i+2){
		word16 = wlan_ntohs(*(UINT16*)(buff+i));
		sum = sum + (UINT32) word16;	
	}
	
	/* take only 16 bits out of the 32 bit sum*/
/*temp = (sum & 0xffff0000)>>16;*/
sum = (sum & 0x0000ffff)+((sum & 0xffff0000)>>16);
	/* one's complement the result*/
	sum = ~sum;
	
return ((UINT16) sum);
}

/*
**************************************************************************
Function: wdrv_TcpChecksumCalc
Description: Calculate the 16 bit TCP Checksum.
***************************************************************************
*/

static UINT16 wdrv_TcpChecksumCalc(ackEmul_t*		ackEmul, UINT16 len_tcp_header,UINT8 *IpSource, UINT8 *IpDest ,UINT8 *buff)
{
UINT16 word16;
UINT32 sum=0;
UINT16 i;


	
	/* add a padding byte = 0 at the end of packet */

		buff[len_tcp_header]=0;
	

	/* make 16 bit words out of every two adjacent 8 bit words in the packet
	   and add them up */
	for (i=0;i<len_tcp_header;i=i+2){
		word16 = wlan_ntohs(*(UINT16*)(buff+i));
		sum = sum + (UINT32) word16;	
	}

   	
   /* add the TCP pseudo header which contains:
      the IP source and destinationn addresses, TCP protocol & TCP length */

   
   word16 = wlan_ntohs(*(UINT16*)IpSource);
   sum = sum + (UINT32) word16;	
   word16 = wlan_ntohs(*(UINT16*)(IpSource+2));
   sum = sum + (UINT32) word16;	
   word16 = wlan_ntohs(*(UINT16*)IpDest);
   sum = sum + (UINT32) word16;	
   word16 = wlan_ntohs(*(UINT16*)(IpDest+2));
   sum = sum + (UINT32) word16;	
   sum = sum + (UINT32)len_tcp_header + 0x06;
   
      
      
      
      /* take only 16 bits out of the 32 bit sum */
sum = (sum & 0x0000ffff)+((sum & 0xffff0000)>>16);
	/* one's complement the result */
	sum = ~sum;
	
return ((UINT16) sum);
}


