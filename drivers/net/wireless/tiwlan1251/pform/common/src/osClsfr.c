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

/*
 * file osClsfr.c
 */

#include "osAdapter.h"
#include "srcApi.h"

#define ETHERNET_HEADER_SIZE    14

/************************************************************************
 *                        os_txClassifier   			                *
 ************************************************************************
DESCRIPTION: the function demonstrate qos classification from UDP port
             number.
                                                                                                   
INPUT:      pData	         -	void pointer to Msdu.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/


TI_STATUS os_txClassifier(mem_MSDU_T *pMsdu,UINT8 *pQosClassifierTable)
{
    UINT8              i;
    UINT8              *pUdpHeader;
    UINT8              *pIpHeader;
    UINT8              ipHeaderLen;
    UINT16             dstPortNum;
    UINT32             ipAddr;
    NWIF_CLSFR_ENTRY  *pCt;

    pCt = (NWIF_CLSFR_ENTRY *)pQosClassifierTable;
    /*pEthHeader = (EthernetHeader_t*)(memMgr_BufData(pMsdu->firstBDPtr)+memMgr_BufOffset(pMsdu->firstBDPtr));*/
    pIpHeader = (UINT8 *)(memMgr_BufData(pMsdu->firstBDPtr)+memMgr_BufOffset(pMsdu->firstBDPtr)) + ETHERNET_HEADER_SIZE;
    ipHeaderLen = ((*(unsigned char*)pIpHeader  & 0x0f) * 4);
    pUdpHeader = pIpHeader + ipHeaderLen;

    dstPortNum = *((UINT16 *)(pUdpHeader + 2));
    ipAddr = *((UINT32 *)(pIpHeader + 16));

	dstPortNum = ((dstPortNum >> 8) | (dstPortNum << 8));


    pMsdu->qosTag = 0;
    for(i = 0; i < NWIF_MAX_QOS_CONNS; i++ )
    {
        if ((pCt[i].ip == ipAddr) && (pCt[i].port == dstPortNum) )
        {
            pMsdu->qosTag = (UINT8)pCt[i].pri;
            /*printk(KERN_ALERT "Found classifier match for ip=0x%x, port=%d, prio=%d \n",ipAddr, dstPortNum, pCt[i].pri);*/
            break;
        }
    }
    

    return OK;

}

/************************************************************************
 *                        os_txSetClassifier   			                *
 ************************************************************************
DESCRIPTION: the function demonstrate setting qos classification table.
                                                                                                   
INPUT:      pData	         -	void pointer to Msdu.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS osSend_ConfigTxClassifier(PTIWLN_ADAPTER_T pAdapter,UINT32 bufferLength, UINT8 *pQosClassifierBuffer)
{
	NWIF_CLSFR_ENTRY *pSrc, *pDest;
	UINT8 clsfrIndex;
	UINT8 NumOfEntries;

	if(bufferLength >  NWIF_MAX_QOS_CONNS * sizeof(NWIF_CLSFR_ENTRY))
	{
		return NOK;
	}

	if((pAdapter == NULL) || (pQosClassifierBuffer == NULL))
	{
		return NOK;
	}

	pDest = (NWIF_CLSFR_ENTRY *)(pAdapter->qosClassifierTable);
	pSrc = (NWIF_CLSFR_ENTRY *)pQosClassifierBuffer;
	
	NumOfEntries = bufferLength / sizeof(NWIF_CLSFR_ENTRY);
	for(clsfrIndex = 0; clsfrIndex <NumOfEntries ; clsfrIndex++)
	{
		pDest[clsfrIndex].ip = pSrc[clsfrIndex].ip;
		pDest[clsfrIndex].port = pSrc[clsfrIndex].port;
		pDest[clsfrIndex].pri = pSrc[clsfrIndex].pri;
	}

	return OK;

}





