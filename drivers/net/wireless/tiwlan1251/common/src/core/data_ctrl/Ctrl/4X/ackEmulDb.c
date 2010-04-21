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
/*                                                                         */
/*   MODULE:	ackEmulDb.c                                                */
/*   PURPOSE:	Ack emulation database                                     */
/*                                                                         */
/***************************************************************************/
#include "osApi.h"
#include "802_11Defs.h"
#include "ackEmulDb.h"
#include "report.h"
#include "utils.h"


static void wdrv_aeFindFreeActiveIndex(ackEmulDB_t*	ackEmulDB,UINT8 *freeIndex);

static void wdrv_aeWTargetDbInit(ackEmulDB_t*	ackEmulDB);
static void wdrv_aeWSourceDbInit(ackEmulDB_t*	ackEmulDB);
static void wdrv_aeWSourceDbResetstation(ackEmulDB_t*	ackEmulDB,int stationIndex);

/*
ULONG rt_osTimerGetTick(void)
{
	ULONG sysuptime;
	
#if (TIWLN_MAJOR_VERSION >= 5)  
	NdisGetSystemUpTime(&sysuptime);
#else
	LARGE_INTEGER systime;

	NdisGetCurrentSystemTime(&systime);
	sysuptime = (((UINT32)systime.LowPart >> 10) | ((UINT32)systime.HighPart << 22))/10;
#endif
	
	return sysuptime;
}

*/
ackEmulDB_t* ackEmulDb_create(TI_HANDLE hOs)
{
	ackEmulDB_t*	ackEmulDB;

	if( hOs  == NULL )
	{
	    WLAN_OS_REPORT(("FATAL ERROR: ackEmulDb_create(): OS handle Error - Aborting\n"));
		return NULL;
	}

	/* alocate concatenator block */
	ackEmulDB = os_memoryAlloc(hOs, (sizeof(ackEmulDB_t)));

	if ( (!ackEmulDB) )
	{
		utils_nullMemoryFree(hOs, ackEmulDB, sizeof(ackEmulDB_t));
	    WLAN_OS_REPORT(("FATAL ERROR: ackEmulDb_create(): Error Creating ackEmulDB module- Aborting\n"));
		return(NULL);
	}

	/* reset control module control block */
	os_memoryZero(hOs, ackEmulDB, (sizeof(ackEmulDB_t)));

	ackEmulDB->hOs = hOs;

	return(ackEmulDB);


}

TI_STATUS ackEmulDb_config(ackEmulDB_t*	ackEmulDB,
						   TI_HANDLE	hOs,
						   TI_HANDLE	hReport)
{
	/* check parameters validity */
	if( (ackEmulDB == NULL) || (hOs == NULL) || (hReport == NULL) )
	{
	    WLAN_OS_REPORT(("FATAL ERROR: ackEmulDb_config(): Parameters Error - Aborting\n"));
		return NOK;
	}

	/* set objects handles */
	ackEmulDB->hOs = hOs;
	ackEmulDB->hReport = hReport;

	wdrv_aeDbInit(ackEmulDB);

	WLAN_REPORT_INIT(ackEmulDB->hReport, ACK_EMUL_MODULE_LOG, 
		(".....ackEmulDB configured successfully\n"));

	return OK;

}

TI_STATUS ackEmulDb_destroy(ackEmulDB_t*	ackEmulDB)
{
	/* free control module controll block */
	os_memoryFree(ackEmulDB->hOs, ackEmulDB, sizeof(ackEmulDB_t));

	return OK;


}

/****************************************************************************
 *                      wdrv_aeDbInit()
 ****************************************************************************
 * DESCRIPTION:	Initialize the WTarget and WSource database
 * 
 * INPUTS:	None
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/

void wdrv_aeDbInit(ackEmulDB_t*	ackEmulDB)
{
   wdrv_aeWTargetDbInit(ackEmulDB);
   wdrv_aeWSourceDbInit(ackEmulDB);     
}	


/****************************************************************************
 *                      wdrv_aeWTargetDbInit()
 ****************************************************************************
 * DESCRIPTION:	Initialize the WTarget database
 * 
 * INPUTS:	None
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/

static void wdrv_aeWTargetDbInit(ackEmulDB_t*	ackEmulDB)
{
	int index;
	for(index=0 ; index< MAX_ACIVE_SESSION+MAX_STANDBY_SESSION; index++)
		wdrv_aeWTargetDbResetTuple(ackEmulDB,index); 
	for(index=0 ; index< MAX_ACIVE_SESSION; index++)
	{
		ackEmulDB->activeIndexTable[index].status = INDEX_FREE;
		ackEmulDB->activeIndexTable[index].monitorIndex = 0xff;
	}
	ackEmulDB->sessionsTableManager.currentActiveState =0;
	ackEmulDB->sessionsTableManager.currentStandbyState =0;
}

/****************************************************************************
 *                      wdrv_aeWTargetDbAddSession()
 ****************************************************************************
 * DESCRIPTION:	ADD new session to WTarget database
 * 
 * INPUTS:	pktBuf - Pointer to packet IP header
 *          stationIndex - The station index of the packet source 
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	Session index if added, otherwise 0xff  
 ****************************************************************************/


int wdrv_aeWTargetDbAddSession(ackEmulDB_t*	ackEmulDB,UINT8 *pktBuf)
{
   int index;
   int freeIndex = 0xff;
   UINT32 currentTimeTick;
   UINT32 oldestTimeTick;
   UINT8  oldestTimeIndex;
   
   /* find free index */
   for(index=0 ; index< MAX_ACIVE_SESSION+MAX_STANDBY_SESSION; index++)
   {
      if(ackEmulDB->wdrv_aeWTargetTable[index].sPorts	== 0)
      {
         freeIndex = index;
         break;
      }
   }
   if (freeIndex == 0xff)
   {
      /* find old session to delete */
      currentTimeTick = oldestTimeTick = os_timeStampUs(ackEmulDB->hOs);
      oldestTimeIndex = 0xff;
      for(index=0 ; index< MAX_ACIVE_SESSION+MAX_STANDBY_SESSION; index++)
      {
         if(ackEmulDB->wdrv_aeWTargetTable[index].timeStamp	< oldestTimeTick)
         {
            oldestTimeTick = ackEmulDB->wdrv_aeWTargetTable[index].timeStamp;
            oldestTimeIndex = index;
         }
      }
      
      if((WTARGET_TERMINATE_TIME_OUT) < (currentTimeTick - oldestTimeTick))
      {
         wdrv_aeWTargetDbResetTuple(ackEmulDB, oldestTimeIndex);
         freeIndex = oldestTimeIndex;
      }
      
   }
   
   if (freeIndex != 0xff)
   {
      /* Add new session */

      int ipHeaderLen = ((*(unsigned char*)pktBuf  & 0x0f) * 4);
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].sPorts = wlan_ntohs(*(UINT16*)(pktBuf +ipHeaderLen));                          
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].dPorts = wlan_ntohs(*(UINT16*)(pktBuf +ipHeaderLen+DEST_PORT_FIELD));                          
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].srcIPAddr = wlan_ntohl(*(unsigned long*)(pktBuf+IP_SRC_ADDRESS_FIELD));                      
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].destIPAddr = wlan_ntohl(*(unsigned long*)(pktBuf+IP_DEST_ADDRESS_FIELD));                     
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].segmentSize				=0;                    
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].sequenceNumber = wlan_ntohl(*(unsigned long*)(pktBuf+ipHeaderLen+TCP_SEQUENCE_NUMBER_FIELD));	                    
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].ackNumber					=0;	                    
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].timeStamp					=os_timeStampUs(ackEmulDB->hOs);                      
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].ackCounter				   =0;	                    
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].ackTemplate.ipHeaderLen			=0;			
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].ackTemplate.tcpHeaderLen			=0;			
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].monitorState				= AE_STANDBY;                     
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].equalSegmentSizeCounter	=0;          
      ackEmulDB->wdrv_aeWTargetTable[freeIndex].yTagFlag					=FALSE;  
		ackEmulDB->wdrv_aeWTargetTable[freeIndex].activeIndex				= 0xff;
        ackEmulDB->sessionsTableManager.currentStandbyState ++;
   }
   return freeIndex;
}


/****************************************************************************
 *                      wdrv_aeWTargetDbFindDataSession()
 ****************************************************************************
 * DESCRIPTION:	Find existing session index for data packet at 
 *                WTarget database.
 * 
 * INPUTS:	pktBuf - Pointer to packet IP header
 *          stationIndex - The station index of the packet source 
 * 
 * OUTPUT:	monitorState - The monitor state 
 * 
 * RETURNS:	Ok if exist  otherwise  NOK
 ****************************************************************************/


int wdrv_aeWTargetDbFindDataSession(ackEmulDB_t*	ackEmulDB, UINT8 *pktBuf,UINT8 *sessionIndex, UINT8 *monitorState)
{
   int index;
   int ipHeaderLen;

   ipHeaderLen = ((*(unsigned char*)pktBuf  & 0x0f) * 4);
   *monitorState = AE_INACTIVE;

	for(index=0 ; index< MAX_ACIVE_SESSION+MAX_STANDBY_SESSION; index++)
	{
		if (ackEmulDB->wdrv_aeWTargetTable[index].sPorts		== wlan_ntohs(*(UINT16*)(pktBuf +ipHeaderLen)) && 
          ackEmulDB->wdrv_aeWTargetTable[index].dPorts		== wlan_ntohs(*(UINT16*)(pktBuf +ipHeaderLen + DEST_PORT_FIELD)) && 
	        ackEmulDB->wdrv_aeWTargetTable[index].srcIPAddr	== wlan_ntohl(*(unsigned long*)(pktBuf+IP_SRC_ADDRESS_FIELD)) &&
	        ackEmulDB->wdrv_aeWTargetTable[index].destIPAddr	== wlan_ntohl(*(unsigned long*)(pktBuf+IP_DEST_ADDRESS_FIELD)))
		{
			*sessionIndex = index;
         *monitorState = ackEmulDB->wdrv_aeWTargetTable[index].monitorState;

         return OK;
      }
   }
   return NOK;
   
}

/****************************************************************************
 *                      wdrv_aeWTargetDbFindDataSession()
 ****************************************************************************
 * DESCRIPTION:	Find existing session index for Ack packet at 
 *                WTarget database.
 * 
 * INPUTS:	pktBuf - Pointer to packet IP header
 *          stationIndex - The station index of the packet source 
 * 
 * OUTPUT:	monitorState - The monitor state 
 * 
 * RETURNS:		Ok if exist  otherwise  NOK  
 ****************************************************************************/
int wdrv_aeWTargetDbFindAckSession(ackEmulDB_t*	ackEmulDB, UINT8 *pktBuf,UINT8 *sessionIndex, UINT8 *monitorState)
{
   int index;
   int ipHeaderLen;

   ipHeaderLen = ((*(unsigned char*)pktBuf  & 0x0f) * 4);
   *monitorState = AE_INACTIVE;

	for(index=0 ; index< MAX_ACIVE_SESSION+MAX_STANDBY_SESSION; index++)
	{

      if (ackEmulDB->wdrv_aeWTargetTable[index].sPorts		== wlan_ntohs(*(UINT16*)(pktBuf +ipHeaderLen + DEST_PORT_FIELD)) && 
          ackEmulDB->wdrv_aeWTargetTable[index].dPorts		== wlan_ntohs(*(UINT16*)(pktBuf +ipHeaderLen )) && 
	        ackEmulDB->wdrv_aeWTargetTable[index].srcIPAddr	== wlan_ntohl(*(unsigned long*)(pktBuf+IP_DEST_ADDRESS_FIELD)) &&
	        ackEmulDB->wdrv_aeWTargetTable[index].destIPAddr	== wlan_ntohl(*(unsigned long*)(pktBuf+IP_SRC_ADDRESS_FIELD)))
		{
			*sessionIndex = index;
         *monitorState = ackEmulDB->wdrv_aeWTargetTable[index].monitorState;

         return OK;
      }
   }
   return NOK;
   
}


/****************************************************************************
 *                      wdrv_aeWTargetDbResetTuple()
 ****************************************************************************
 * DESCRIPTION:	reset tuple at WTarget database 
 * 
 * INPUTS:	index - the tuple index
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/

void wdrv_aeWTargetDbResetTuple(ackEmulDB_t*	ackEmulDB, int index)
{
   UINT8 freeActiveIndex = ackEmulDB->wdrv_aeWTargetTable[index].activeIndex;
	if (ackEmulDB->wdrv_aeWTargetTable[index].monitorState == AE_ACTIVE)
	{
		ackEmulDB->sessionsTableManager.currentActiveState --;
	}
	else
	{
		ackEmulDB->sessionsTableManager.currentStandbyState --;
	}
	
   ackEmulDB->wdrv_aeWTargetTable[index].sPorts						=0;                          
   ackEmulDB->wdrv_aeWTargetTable[index].dPorts						=0;                          
   ackEmulDB->wdrv_aeWTargetTable[index].srcIPAddr					=0;                      
   ackEmulDB->wdrv_aeWTargetTable[index].destIPAddr				=0;                     
   ackEmulDB->wdrv_aeWTargetTable[index].segmentSize				=0;                    
   ackEmulDB->wdrv_aeWTargetTable[index].sequenceNumber			=0;	                    
   ackEmulDB->wdrv_aeWTargetTable[index].ackNumber					=0;	                    
   ackEmulDB->wdrv_aeWTargetTable[index].timeStamp					=0;                      
   ackEmulDB->wdrv_aeWTargetTable[index].ackCounter				=0;	                    
   ackEmulDB->wdrv_aeWTargetTable[index].ackTemplate.ipHeaderLen			=0;			
   ackEmulDB->wdrv_aeWTargetTable[index].ackTemplate.tcpHeaderLen			=0;			
   ackEmulDB->wdrv_aeWTargetTable[index].monitorState				=AE_INACTIVE;                     
   ackEmulDB->wdrv_aeWTargetTable[index].equalSegmentSizeCounter	=0;          
   ackEmulDB->wdrv_aeWTargetTable[index].yTagFlag					=FALSE; 
	ackEmulDB->wdrv_aeWTargetTable[index].activeIndex		= 0xff;
   if(freeActiveIndex != 0xff)

   {
      ackEmulDB->activeIndexTable[freeActiveIndex].status = INDEX_FREE;
      ackEmulDB->activeIndexTable[freeActiveIndex].monitorIndex = 0xff;
			
      
   }
}


/****************************************************************************
 *                      wdrv_aeWTargetDbPrint()
 ****************************************************************************
 * DESCRIPTION:	Print the WTarget database 
 * 
 * INPUTS:	None
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/
void wdrv_aeWTargetDbPrint(ackEmulDB_t*	ackEmulDB)
{
	int index;
   unsigned char* sipByte;
	 unsigned char* 	dipByte;

   WLAN_OS_REPORT(("\n current Active State = %d\n", ackEmulDB->sessionsTableManager.currentActiveState));
   WLAN_OS_REPORT((" current Standby State = %d\n", ackEmulDB->sessionsTableManager.currentStandbyState));
   WLAN_OS_REPORT((" --------------------------\n"));
   
   WLAN_OS_REPORT(("|  |   SOURCE IP   |   DEST  IP    |SPORT|DPORT|St|Seg Size |AI|AC\n"));
   WLAN_OS_REPORT(("|------------------------------------------------------------------------| \n"));
   for(index=0 ; index< MAX_ACIVE_SESSION+MAX_STANDBY_SESSION; index++)
   {
	   sipByte = (unsigned char*) &ackEmulDB->wdrv_aeWTargetTable[index].srcIPAddr;
	   dipByte = (unsigned char*) &ackEmulDB->wdrv_aeWTargetTable[index].destIPAddr;
	   
	   WLAN_OS_REPORT(("|%02.2u|%03.3u.%03.3u.%03.3u.%03.3u|%03.3u.%03.3u.%03.3u.%03.3u|%05u|%05u|%02x|%09lu|%02x|%x|\n",
		   index,
		   sipByte[3],sipByte[2],sipByte[1],sipByte[0], 
		   dipByte[3],dipByte[2],dipByte[1],dipByte[0],
		   ackEmulDB->wdrv_aeWTargetTable[index].sPorts,
		   ackEmulDB->wdrv_aeWTargetTable[index].dPorts,
		   (UINT8)ackEmulDB->wdrv_aeWTargetTable[index].monitorState,
		   ackEmulDB->wdrv_aeWTargetTable[index].segmentSize,
         (UINT8)ackEmulDB->wdrv_aeWTargetTable[index].activeIndex,
         (UINT8)ackEmulDB->wdrv_aeWTargetTable[index].ackCounter));
		   
   }
}



/****************************************************************************
 *                      wdrv_aeWTargetDbSetSessionSequenceNumber()
 ****************************************************************************
 * DESCRIPTION:	Set the Sequence Number fild at WTarget data base
 * 
 * INPUTS:	index          - session index
 *          sequenceNumber - Tcp sequence number
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/

void wdrv_aeWTargetDbSetSessionSequenceNumber(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 sequenceNumber)
{
   ackEmulDB->wdrv_aeWTargetTable[index].sequenceNumber = sequenceNumber;
}

/****************************************************************************
 *                      wdrv_aeWTargetDbGetSessionSequenceNumber()
 ****************************************************************************
 * DESCRIPTION:	Get the Sequence Number fild from WTarget data base
 * 
 * INPUTS:	index          - session index
 * 
 * OUTPUT:	*sequenceNumber - Tcp sequence number
 * 
 * RETURNS:	None
 ****************************************************************************/
void wdrv_aeWTargetDbGetSessionSequenceNumber(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 *sequenceNumber)
{
      *sequenceNumber = ackEmulDB->wdrv_aeWTargetTable[index].sequenceNumber;

}


/****************************************************************************
 *                      wdrv_aeWTargetDbSetSessionSegmentSize()
 ****************************************************************************
 * DESCRIPTION:	Set the Segmen tSize fild at WTarget data base
 * 
 * INPUTS:	index          - session index
 *          segmentSize - Tcp segment size
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/
void wdrv_aeWTargetDbSetSessionSegmentSize(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 segmentSize)
{
   ackEmulDB->wdrv_aeWTargetTable[index].segmentSize = segmentSize;

}

/****************************************************************************
 *                      wdrv_aeWTargetDbGetSessionSegmentSize()
 ****************************************************************************
 * DESCRIPTION:	Get the SegmentSize fild from WTarget data base
 * 
 * INPUTS:	index          - session index
 * 
 * OUTPUT:	*segmentSize - Tcp sequence number
 * 
 * RETURNS:	None
 ****************************************************************************/
void wdrv_aeWTargetDbGetSessionSegmentSize(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 *segmentSize)
{
      *segmentSize = ackEmulDB->wdrv_aeWTargetTable[index].segmentSize;

}
/****************************************************************************
 *               wdrv_aeWTargetDbSetSessionEqualSegmentSizeCounter()
 ****************************************************************************
 * DESCRIPTION:	Set the number of sequential packet 
 *                with the same Segment Size.
 * INPUTS:	index                    - session index
 *          equalSegmentSizeCounter - the number of sequential packet
 *                                    with the same Segment Size
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/
void wdrv_aeWTargetDbSetSessionEqualSegmentSizeCounter(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT8 equalSegmentSizeCounter)
{
	ackEmulDB->wdrv_aeWTargetTable[index].equalSegmentSizeCounter = equalSegmentSizeCounter; 
}

/****************************************************************************
 *              wdrv_aeWTargetDbGetIncrSessionEqualSegmentSizeCounter()
 ****************************************************************************
 * DESCRIPTION:	Increase and return  the number of sequential packet 
 *                with the same Segment Size.
 * 
 * INPUTS:	index          - session index
 * 
 * OUTPUT:	*equalSegmentSizeCounter - the number of sequential packet
 *          with the same Segment Size
 * 
 * RETURNS:	None
 ****************************************************************************/
void wdrv_aeWTargetDbGetIncrSessionEqualSegmentSizeCounter(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT8 *equalSegmentSizeCounter)
{
	*equalSegmentSizeCounter = (ackEmulDB->wdrv_aeWTargetTable[index].equalSegmentSizeCounter ++);
}


/****************************************************************************
 *                   wdrv_aeWTargetDbSetActiveState()
 ****************************************************************************
 * DESCRIPTION:	Find free active index and update the WTarget DB with 
 *                this index
 * 
 * INPUTS:	index          - monitor session index
 * 
 * OUTPUT:	*activeIndex - the active index
 * 
 * RETURNS:	Ok if change state to active, otherwise NOK
 ****************************************************************************/
int wdrv_aeWTargetDbSetActiveState(ackEmulDB_t*	ackEmulDB, UINT8 index , UINT8 *activeIndex)
{
   UINT8 monitorState = AE_ACTIVE ;
   UINT8 freeIndex ,aIndex;
   
   wdrv_aeFindFreeActiveIndex(ackEmulDB, &freeIndex);
   
   if(freeIndex == 0xff)
   {
      /* We don't have free active index we try find old active session to delete */
      UINT32 currentTimeTick;
      UINT32 oldestTimeTick;
      UINT8  oldestTimeIndex;
      currentTimeTick = oldestTimeTick = os_timeStampUs(ackEmulDB->hOs);
      oldestTimeIndex = 0xff;
      for(aIndex=0 ; aIndex< MAX_ACIVE_SESSION; aIndex++)
      {
         if(ackEmulDB->wdrv_aeWTargetTable[ackEmulDB->activeIndexTable[aIndex].monitorIndex].timeStamp	< 
            oldestTimeTick)
         {
            oldestTimeTick = ackEmulDB->wdrv_aeWTargetTable[ackEmulDB->activeIndexTable[aIndex].monitorIndex].timeStamp;
            oldestTimeIndex = aIndex;
         }
      }
      
      if((WTARGET_TERMINATE_TIME_OUT) < (currentTimeTick - oldestTimeTick))
      {
         wdrv_aeWTargetDbResetTuple(ackEmulDB, ackEmulDB->activeIndexTable[oldestTimeIndex].monitorIndex);
         freeIndex = oldestTimeIndex;
      }
      
   }


   if (freeIndex == 0xff)
   {
      *activeIndex = 0xff;
      return NOK;
   }

   /* we have new active index */
   ackEmulDB->activeIndexTable[freeIndex].status = INDEX_BUSY;
   ackEmulDB->activeIndexTable[freeIndex].monitorIndex = index;
   ackEmulDB->wdrv_aeWTargetTable[index].activeIndex = freeIndex;
   *activeIndex = freeIndex;

   ackEmulDB->sessionsTableManager.currentActiveState ++;
   ackEmulDB->sessionsTableManager.currentStandbyState --;
   
   /* set the monitor session state to ACTIVE */
   wdrv_aeWTargetDbSetSessionMonitorState(ackEmulDB, index, monitorState);   
   return OK;
}

/****************************************************************************
 *                   wdrv_aeWTargetDbSetSessionMonitorState()
 ****************************************************************************
 * DESCRIPTION:	Set the state of monitor session.
 * 
 * INPUTS:	index       - session index
 *          
 * 
 * OUTPUT:	monitorState - the new monitor state
 * 
 * RETURNS:	None
 ****************************************************************************/

void wdrv_aeWTargetDbSetSessionMonitorState(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT8 monitorState)
{
      ackEmulDB->wdrv_aeWTargetTable[index].monitorState = monitorState;
}


/****************************************************************************
 *                   wdrv_aeWTargetDbGetSessionAckNumber()
 ****************************************************************************
 * DESCRIPTION:	Get the monitor session ack number
 * 
 * INPUTS:	index          - monitor session index
 * 
 * OUTPUT:	*ackNumber - The ack number
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWTargetDbGetSessionAckNumber(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 *ackNumber)
{
      *ackNumber = ackEmulDB->wdrv_aeWTargetTable[index].ackNumber;
}

/****************************************************************************
 *                   wdrv_aeWTargetDbSetSessionAckNumber()
 ****************************************************************************
 * DESCRIPTION:	Set the monitor session ack number
 * 
 * INPUTS:	index          - monitor session index
 *        	ackNumber - The ack number
 * 
 * OUTPUT:	ackNumber - The ack number
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWTargetDbSetSessionAckNumber(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 ackNumber)
{
      ackEmulDB->wdrv_aeWTargetTable[index].ackNumber = ackNumber;
}


/****************************************************************************
 *                   wdrv_aeWTargetDbGetSessionAckCounter()
 ****************************************************************************
 * DESCRIPTION:	Get the monitor session ack counter
 * 
 * INPUTS:	index          - monitor session index
 * 
 * OUTPUT:	*ackCounter - The ack counter
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWTargetDbGetSessionAckCounter(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 *ackCounter)
{
      *ackCounter = ackEmulDB->wdrv_aeWTargetTable[index].ackCounter;
}

/****************************************************************************
 *                   wdrv_aeWTargetDbSetSessionAckCounter()
 ****************************************************************************
 * DESCRIPTION:	Set the monitor session ack counter
 * 
 * INPUTS:	index          - monitor session index
 *          ackCounter     - The ack counter
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWTargetDbSetSessionAckCounter(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 ackCounter)
{
      ackEmulDB->wdrv_aeWTargetTable[index].ackCounter = ackCounter;
}

/****************************************************************************
 *                   wdrv_aeWTargetDbGetSessionActiveIndex()
 ****************************************************************************
 * DESCRIPTION:	Get the monitor session active index
 * 
 * INPUTS:	index          - monitor session index
 * 
 * OUTPUT:	*activeIndex - The session active index
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWTargetDbGetSessionActiveIndex(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT8 *activeIndex)
{
      *activeIndex = ackEmulDB->wdrv_aeWTargetTable[index].activeIndex;

}

/****************************************************************************
 *                   wdrv_aeWTargetDbGetLastWackInfo()
 ****************************************************************************
 * DESCRIPTION:	Get the last wack info for this monitor session.
 * 
 * INPUTS:	index          - monitor session index
 * 
 * OUTPUT:	*lastWackInfo -  the last wack info
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWTargetDbGetSessionTimeStamp(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 *timeStamp)
{
      *timeStamp = ackEmulDB->wdrv_aeWTargetTable[index].timeStamp;

}

/****************************************************************************
 *                   wdrv_aeWTargetDbSetSessionTimeStamp()
 ****************************************************************************
 * DESCRIPTION:	Set the time stamp for this monitor session.
 * 
 * INPUTS:	index          - monitor session index
 *          timeStamp -  the time stamp info
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWTargetDbSetSessionTimeStamp(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 timeStamp)
{
      ackEmulDB->wdrv_aeWTargetTable[index].timeStamp= timeStamp;
}

/****************************************************************************
 *                   wdrv_aeFindFreeActiveIndex()
 ****************************************************************************
 * DESCRIPTION:	find if there is free index for active session.
 * 
 * INPUTS:	None.
 * 
 * OUTPUT:	*freeIndex -  index of free tuple in table activeIndexTable
 * 
 * RETURNS:None
 ****************************************************************************/
static void wdrv_aeFindFreeActiveIndex(ackEmulDB_t*	ackEmulDB, UINT8 *freeIndex)
{
	UINT8 index;
	*freeIndex = 0xff;
	for(index=0 ; index< MAX_ACIVE_SESSION; index++)
	{
		if(ackEmulDB->activeIndexTable[index].status == INDEX_FREE)
		{
			*freeIndex = index;
			return;
		}
	}	
}


/****************************************************************************
 *                   wdrv_aeWTargetDbSaveAckTemplate()
 ****************************************************************************
 * DESCRIPTION:	save the Tcp ack template for the WTarget side.
 * 
 * INPUTS:	index          - monitor session index.
 *          *pIpHeader     - Pointer to packet IP header
 * 
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWTargetDbSaveAckTemplate(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT8 *pIpHeader)
{

   UINT8 ipHeaderLen;
   UINT8 tcpHeaderLen;
   
   ipHeaderLen = ((*(unsigned char*)pIpHeader  & 0x0f) * 4);
   tcpHeaderLen = ((((*(unsigned char*)(pIpHeader + ipHeaderLen+TCP_OFFSET_FIELD))& 0xf0)>>4) * 4);
   os_memoryCopy(NULL, ackEmulDB->wdrv_aeWTargetTable[index].ackTemplate.data, pIpHeader, ipHeaderLen+tcpHeaderLen);
   ackEmulDB->wdrv_aeWTargetTable[index].ackTemplate.ipHeaderLen = ipHeaderLen;
   ackEmulDB->wdrv_aeWTargetTable[index].ackTemplate.tcpHeaderLen = tcpHeaderLen;
}




/****************************************************************************
 *                   wdrv_aeWTargetDbUpdateAckTemplate()
 ****************************************************************************
 * DESCRIPTION:	Update the Tcp ack template for the WTarget side.
 * 
 * INPUTS:	index          - monitor session index.
 *          sequenceNumber - New sequence number
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWTargetDbUpdateAckTemplate(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT32 sequenceNumber)
{
   UINT8 *pSequenceNumber;
   /* Update Template Sequence Number */
   pSequenceNumber = ackEmulDB->wdrv_aeWTargetTable[index].ackTemplate.data + 
        ackEmulDB->wdrv_aeWTargetTable[index].ackTemplate.tcpHeaderLen +TCP_SEQUENCE_NUMBER_FIELD;
   *(unsigned long*)(pSequenceNumber) = wlan_ntohl(sequenceNumber);
}



/****************************************************************************
 *                   wdrv_aeWTargetDbCmpAckTemplate()
 ****************************************************************************
 * DESCRIPTION:	comper the current tcp ack with ack template.
 * 
 * INPUTS:	index          - monitor session index.
 *          *pIpHeader     - Pointer to packet IP header
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
int wdrv_aeWTargetDbCmpAckTemplate(ackEmulDB_t*	ackEmulDB, UINT8 index, UINT8 *pIpHeader)
{
   UINT8 ipHeaderLen;
   UINT8 *pTcpHeader;
   UINT8 *pIpHeaderTemplate;
   UINT8 *pTcpHeaderTemplate;
   UINT8 ipHeaderTemplateLen;
   
   ipHeaderLen = ((*(unsigned char*)pIpHeader  & 0x0f) * 4);
   pIpHeaderTemplate = ackEmulDB->wdrv_aeWTargetTable[index].ackTemplate.data;
   ipHeaderTemplateLen = ackEmulDB->wdrv_aeWTargetTable[index].ackTemplate.ipHeaderLen;
   
   /* Comprer IP field: Version ,Header Length, Precedence, TOS , Unused, Total Length */
   if((*(UINT32*)(pIpHeaderTemplate))!=(*(UINT32*)pIpHeader))
   {
      WLAN_OS_REPORT(("\nCompare field: Version,Header Length, Precedence, TOS, Total Length fail \n"));
      return NOK;
   }

   /* Comprer IP field:Fragmentation Flags, Fragment Offset, TTL */
   if((*(UINT32*)(pIpHeaderTemplate+IP_IDENTIFIER_FIELD+2))!= 
      (*(UINT32*)(pIpHeader+IP_IDENTIFIER_FIELD+2)))
   {
   		WLAN_REPORT_WARNING(ackEmulDB->hReport, ACK_EMUL_MODULE_LOG, 
			("Compare IP field:Fragmentation Flags, Fragment Offset, TTL fail \n"));
      return NOK;
   }
   
   pTcpHeader = pIpHeader + ipHeaderLen;
   pTcpHeaderTemplate = pIpHeaderTemplate + ipHeaderTemplateLen;
   
   
      /* Comprer TCP field: Offset, Reserved, Code, Window */
      if((*(UINT32*)(pTcpHeaderTemplate+TCP_ACK_NUMBER_FIELD+4))!= 
      (*(UINT32*)(pTcpHeader+TCP_ACK_NUMBER_FIELD+4)))
   {
  		WLAN_REPORT_WARNING(ackEmulDB->hReport, ACK_EMUL_MODULE_LOG, 
			("Compare TCP field: Offset, Reserved, Code, Window fail\n"));
      return NOK;
   }
      /* Comprer TCP field: Urgent */
   
   if((*(UINT16*)(pTcpHeaderTemplate+TCP_CHECKSUM_FIELD+2))!= 
      (*(UINT16*)(pTcpHeader+TCP_CHECKSUM_FIELD+2)))
   {
   		WLAN_REPORT_WARNING(ackEmulDB->hReport, ACK_EMUL_MODULE_LOG, 
			("Compare TCP field: Urgent fail\n\n"));
      return NOK;
   }
   
      
      /* Comprer TCP field: Sequence Number */
   if((*(UINT32*)(pTcpHeaderTemplate+TCP_SEQUENCE_NUMBER_FIELD))!= 
      (*(UINT32*)(pTcpHeader+TCP_SEQUENCE_NUMBER_FIELD)))
   {
       WLAN_REPORT_WARNING(ackEmulDB->hReport, ACK_EMUL_MODULE_LOG, 
			("Comprare TCP field: Sequence Number fail\n\n"));
	/* add Ytag */
      return NOK;
   }
   

   return OK;
   
}


/****************************************************************************
 *                         wdrv_aeDbGetXTagStatus()
 ****************************************************************************
 * DESCRIPTION:	Get the Xtag status of the source station for this 
 *                session index.
 * 
 * INPUTS:	sessionIndex  - monitor session index
 * 
 * OUTPUT:	*status -  Xtag status
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeDbGetXTagStatus(ackEmulDB_t*	ackEmulDB, UINT8 sessionIndex, UINT8 *status)
{
      UINT16 stationIndex = ackEmulDB->wdrv_aeWTargetTable[sessionIndex].sourceStationIndex;
      if(stationIndex != 0xff)
         *status = ackEmulDB->ackEmulationXTagTable[stationIndex];
}

/****************************************************************************
 *                         wdrv_aeDbSetXTagStatus()
 ****************************************************************************
 * DESCRIPTION:	Set the Xtag status of the source station for this 
 *                session index.
 * 
 * INPUTS:	sessionIndex  - monitor session index
 * 
 * OUTPUT:	status -  Xtag status
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeDbSetXTagStatus(ackEmulDB_t*	ackEmulDB, UINT8 sessionIndex, UINT8 status)
{
      UINT16 stationIndex = ackEmulDB->wdrv_aeWTargetTable[sessionIndex].sourceStationIndex;
      if(stationIndex != 0xff)
         ackEmulDB->ackEmulationXTagTable[stationIndex] = status;
}







/******************************* WSource Data base *************************************/

/****************************************************************************
 *                      wdrv_aeWSourceDbInit()
 ****************************************************************************
 * DESCRIPTION:	Initialize the WSource database
 * 
 * INPUTS:	None
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/
static void wdrv_aeWSourceDbInit(ackEmulDB_t*	ackEmulDB)
{
	int stationIndex;
	for (stationIndex =0;stationIndex < MAX_AE_STATIONS;stationIndex++)
		wdrv_aeWSourceDbResetstation(ackEmulDB, stationIndex);
}

/****************************************************************************
 *                      wdrv_aeWSourceDbResetstation()
 ****************************************************************************
 * DESCRIPTION:	Reset all the WSource tuple for specific station.
 * 
 * INPUTS:	stationIndex - index of station to reset
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/
static void wdrv_aeWSourceDbResetstation(ackEmulDB_t*	ackEmulDB, int stationIndex)
{
	int activeIndex;
	for (activeIndex =0;activeIndex < MAX_ACIVE_SESSION;activeIndex++)
	{
		ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].segmentSize=0;              
      ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackNumber =0;	                        
      ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackCounter =0;	                        
      ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].timeStamp =0;	                        
      ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackReorderProblem = REORDER_PROBLEM_OFF;
		ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackTemplate.ipHeaderLen =0;	
		ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackTemplate.tcpHeaderLen =0;	
	}			
}

/****************************************************************************
 *                      wdrv_aeWSourceDbResetSession()
 ****************************************************************************
 * DESCRIPTION:	Reset specific WSource session tuple for specific station.
 * 
 * INPUTS:	stationIndex - index of station to reset
 *          activeIndex  - the index of the WSource tcp session
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/
void wdrv_aeWSourceDbResetSession(ackEmulDB_t*	ackEmulDB, int stationIndex,int activeIndex)
{
   ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].segmentSize=0;              
   ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackNumber =0;	                        
   ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackCounter =0;	                        
   ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].timeStamp =0;	                        
   ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackReorderProblem = REORDER_PROBLEM_OFF;
   ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackTemplate.ipHeaderLen =0;	
   ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackTemplate.tcpHeaderLen =0;	
}

/****************************************************************************
 *                   wdrv_aeWSourceSaveAckTemplate()
 ****************************************************************************
 * DESCRIPTION:	save the Tcp ack template for the WSource side.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 *          *dataBuf              -  Packet data
 *          dataLen               -  data len
 *          segmentSize           - segment Size
 * 
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceSaveAckTemplate(ackEmulDB_t*	ackEmulDB, UINT8 stationIndex,UINT8 activeIndex,
								   UINT8* pDot11Header, UINT8 *pWlanSnapHeader, UINT8 *pIpHeader 
								   ,UINT16 dataLen,UINT16 segmentSize)
{
	UINT8 *pTcpHeader;
	UINT8 *pTemplateData;
	UINT8 ipHeaderLen;
	UINT8 tcpHeaderLen;
	UINT32 ackNumber;
	
	
	WLAN_OS_REPORT(("wdrv_aeWSourceSaveAckTemplate datalen = %d\n",dataLen));
	
    wdrv_aeWSourceDbSetSessionTimeStamp(ackEmulDB, stationIndex,activeIndex,os_timeStampUs(ackEmulDB->hOs));   
	
	
	ipHeaderLen = ((*(unsigned char*)pIpHeader  & 0x0f) * 4);
	pTcpHeader = pIpHeader + ipHeaderLen;
	tcpHeaderLen = ((((*(unsigned char*)(pTcpHeader+TCP_OFFSET_FIELD))& 0xf0)>>4) * 4);
	pTemplateData = ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackTemplate.data;
	
	os_memoryCopy(ackEmulDB->hOs, pTemplateData,pDot11Header, WLAN_HDR_LEN);
	os_memoryCopy(ackEmulDB->hOs, pTemplateData+WLAN_HDR_LEN,pWlanSnapHeader, WLAN_SNAP_HDR_LEN);
	WLAN_OS_REPORT((" osMoveMemory 2 \n"));   
	
	/* osMoveMemory(pTemplateData+WLAN_HDR_LEN+WLAN_SNAP_HDR_LEN,pIpHeader, dataLen-WLAN_HDR_LEN-WLAN_SNAP_HDR_LEN);*/
	os_memoryCopy(ackEmulDB->hOs, pTemplateData+WLAN_HDR_LEN+WLAN_SNAP_HDR_LEN,pIpHeader, dataLen);
	ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackTemplate.ipHeaderLen = ipHeaderLen;
	ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackTemplate.tcpHeaderLen = tcpHeaderLen;
	
	ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].segmentSize = segmentSize;    
	
	ackNumber = wlan_ntohl(*(unsigned long*)(pIpHeader+ipHeaderLen+TCP_ACK_NUMBER_FIELD));
	ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackNumber = ackNumber;
	ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackCounter =	ackNumber/(segmentSize*2);	
	ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackReorderProblem = REORDER_PROBLEM_OFF;
	
	/* reset the Ip & TCP Checksum */
	*(UINT16*)(pTemplateData+WLAN_HDR_LEN+WLAN_SNAP_HDR_LEN+IP_CHECKSUM_FIELD) = 0x0000;
	*(UINT16*)(pTemplateData+WLAN_HDR_LEN+WLAN_SNAP_HDR_LEN+ipHeaderLen+TCP_CHECKSUM_FIELD) = 0x0000;
	
}


/****************************************************************************
 *                   wdrv_aeWSourceDbGetSessionAckCounter()
 ****************************************************************************
 * DESCRIPTION:	Get the ackCounter fild from WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 * 
 * OUTPUT:	*ackCounter -         -  the Ack Counter
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbGetSessionAckCounter(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex, UINT32 *ackCounter)
{
      *ackCounter = ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackCounter;
}

/****************************************************************************
 *                   wdrv_aeWSourceDbGetSessionAckCounter()
 ****************************************************************************
 * DESCRIPTION:	Set the ackCounter fild at WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 *          ackCounter -         -  the Ack Counter
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbSetSessionAckCounter(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex, UINT32 ackCounter)
{
      ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackCounter= ackCounter;
}

/****************************************************************************
 *                   wdrv_aeWSourceDbGetSessionAckNumber()
 ****************************************************************************
 * DESCRIPTION:	Get the AckNumber fild from WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 * 
 * OUTPUT:	*ackNumber -         -  the ack number
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbGetSessionAckNumber(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex, UINT32 *ackNumber)
{
      *ackNumber = ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackNumber;
}

/****************************************************************************
 *                   wdrv_aeWSourceDbSetSessionAckNumber()
 ****************************************************************************
 * DESCRIPTION:	Set the ackNumber fild at WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 *          ackNumber -           -  the ack number
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbSetSessionAckNumber(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex, UINT32 ackNumber)
{
      ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackNumber= ackNumber;
}

/****************************************************************************
 *                   wdrv_aeWSourceDbGetSessionSegmentSize()
 ****************************************************************************
 * DESCRIPTION:	Get the SegmentSize fild from WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 * 
 * OUTPUT:	*segmentSize -         -  the ack segment size
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbGetSessionSegmentSize(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex, UINT32 *segmentSize)
{
      *segmentSize = ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].segmentSize;
}

/****************************************************************************
 *                   wdrv_aeWSourceDbSetSessionSegmentSize()
 ****************************************************************************
 * DESCRIPTION:	Set the segmentSize fild at WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 *          segmentSize -         -  the segment size
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbSetSessionSegmentSize(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex, UINT32 segmentSize)
{
      ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].segmentSize= segmentSize;
}


/****************************************************************************
 *                   wdrv_aeWSourceDbGetSessionTimeStamp()
 ****************************************************************************
 * DESCRIPTION:	Get the timeStamp fild from WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 * 
 * OUTPUT:	*timeStamp -         -  the time stamp
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbGetSessionTimeStamp(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex, UINT32 *timeStamp)
{
      *timeStamp = ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].timeStamp;
}

/****************************************************************************
 *                   wdrv_aeWSourceDbSetSessionTimeStamp()
 ****************************************************************************
 * DESCRIPTION:	Set the timeStamp fild at WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 *          timeStamp -         -  the time stamp
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbSetSessionTimeStamp(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex, UINT32 timeStamp)
{
      ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].timeStamp= timeStamp;
}

/****************************************************************************
 *                   wdrv_aeWSourceDbGetSessionAckReorderProblem()
 ****************************************************************************
 * DESCRIPTION:	Get the ack reorder problem fild from WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 * 
 * OUTPUT:	*ackReorderProblem    -  the ack reorder problem
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbGetSessionAckReorderProblem(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex, UINT32 *ackReorderProblem)
{
      *ackReorderProblem = ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackReorderProblem;
}

/****************************************************************************
 *                   wdrv_aeWSourceDbSetSessionAckReorderProblem()
 ****************************************************************************
 * DESCRIPTION:	Set the timeStamp fild at WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 *          ackReorderProblem     -  the ack reorder problem
 * 
 * OUTPUT:	None
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbSetSessionAckReorderProblem(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex, UINT32 ackReorderProblem)
{
      ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackReorderProblem= ackReorderProblem;
}

/****************************************************************************
 *                   wdrv_aeWSourceDbGetAckTemplate()
 ****************************************************************************
 * DESCRIPTION:	Get the ack reorder problem fild from WSource data base.
 * 
 * INPUTS:	stationIndex          -  station index.
 *          activeIndex           -  session index
 *          
 * OUTPUT:	**pTeplate            -  pointer to the ack template buffer.
 *          *ipHeaderLen          -  IP header length of Ack template.
 *          tcpHeaderLen          -  TCP header length of Ack template.
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbGetAckTemplate(ackEmulDB_t*	ackEmulDB, UINT16 stationIndex, UINT8 activeIndex,UINT8 **pTeplate,
													 UINT8 *ipHeaderLen, UINT8 *tcpHeaderLen)
{																					
      *ipHeaderLen   = ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackTemplate.ipHeaderLen;
      *tcpHeaderLen  = ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackTemplate.tcpHeaderLen;
      *pTeplate      = ackEmulDB->wdrv_aeWSourceTable[stationIndex][activeIndex].ackTemplate.data;
}


/****************************************************************************
 *                   wdrv_aeWSourceDbUpdateTemplate()
 ****************************************************************************
 * DESCRIPTION:	.Fide  and updte the WSource monitor session 
 * 
 * INPUTS:	*pktBuf               -  receive tcp Ack buffer.
 *          stationIndex          -  station index
 *          
 * OUTPUT:	*sessionIndex         -  The session index of the received ack
 * 
 * RETURNS:None
 ****************************************************************************/
void wdrv_aeWSourceDbUpdateTemplate(ackEmulDB_t*	ackEmulDB, UINT8 *pktBuf,UINT8 stationIndex,UINT8 *sessionIndex)
{
   int index;
   int ipHeaderLen;
   UINT8 *pTmpIpHeader;
   int tmpIpHeaderLen;
   UINT32 ackNumber;
   UINT32 prevAckNumber;
   UINT32 segmentSize;
   UINT32 reorderProblemStatus;
   *sessionIndex =0xff;

   ipHeaderLen = ((*(unsigned char*)pktBuf  & 0x0f) * 4);
   /* Find WSource session */
   for(index=0 ; index< MAX_ACIVE_SESSION; index++)
	{
      pTmpIpHeader = (ackEmulDB->wdrv_aeWSourceTable[stationIndex][index].ackTemplate.data)+ WLAN_HDR_LEN+WLAN_SNAP_HDR_LEN;
      tmpIpHeaderLen = ackEmulDB->wdrv_aeWSourceTable[stationIndex][index].ackTemplate.ipHeaderLen;
      if (	*(UINT16*)(pTmpIpHeader +tmpIpHeaderLen + DEST_PORT_FIELD)== *(UINT16*)(pktBuf +ipHeaderLen + DEST_PORT_FIELD) && 
            *(UINT16*)(pTmpIpHeader +tmpIpHeaderLen )	== *(UINT16*)(pktBuf +ipHeaderLen ) && 
	         *(unsigned long*)(pTmpIpHeader+IP_DEST_ADDRESS_FIELD) 	== *(unsigned long*)(pktBuf+IP_DEST_ADDRESS_FIELD) &&
            *(unsigned long*)(pTmpIpHeader+IP_SRC_ADDRESS_FIELD)	== *(unsigned long*)(pktBuf+IP_SRC_ADDRESS_FIELD))
      {
         /* Update ackNumber , ackCounter and reorder problem flag */
         ackNumber = wlan_ntohl(*(unsigned long*)(pktBuf+ipHeaderLen+TCP_ACK_NUMBER_FIELD));
         segmentSize = ackEmulDB->wdrv_aeWSourceTable[stationIndex][index].segmentSize;
			prevAckNumber = ackEmulDB->wdrv_aeWSourceTable[stationIndex][index].ackNumber;
         
         
         if((prevAckNumber % (segmentSize*2))>(ackNumber % (segmentSize*2)))
         {
            reorderProblemStatus = ackEmulDB->wdrv_aeWSourceTable[stationIndex][index].ackReorderProblem;

            switch(reorderProblemStatus)
            {
            case REORDER_PROBLEM_OFF:
               ackEmulDB->wdrv_aeWSourceTable[stationIndex][index].ackReorderProblem = REORDER_PROBLEM_ON;
               break;
            case REORDER_PROBLEM_ON:
               ackEmulDB->wdrv_aeWSourceTable[stationIndex][index].ackReorderProblem = REORDER_PROBLEM_PRE;
               break;
            case REORDER_PROBLEM_PRE:
            default:
               break;
            }
         }
			
         ackEmulDB->wdrv_aeWSourceTable[stationIndex][index].ackNumber = ackNumber;
         ackEmulDB->wdrv_aeWSourceTable[stationIndex][index].ackCounter =	ackNumber/(segmentSize*2);	
          *(unsigned long*)(pTmpIpHeader+tmpIpHeaderLen+TCP_SEQUENCE_NUMBER_FIELD) =
          *(unsigned long*)(pktBuf+ipHeaderLen+TCP_SEQUENCE_NUMBER_FIELD);
         
         *sessionIndex = index;
         WLAN_OS_REPORT(("\nindex =  = %d ackNumber %X , indentifier %d\n", *sessionIndex,ackNumber,
            *(UINT16*)(pktBuf+IP_IDENTIFIER_FIELD)));
       
         
         return ;
      }
   }
   return ;
   
}
