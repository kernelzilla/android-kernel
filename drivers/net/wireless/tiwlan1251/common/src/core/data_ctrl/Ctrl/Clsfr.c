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
 * file Clsfr.c
 */

#include "paramOut.h"
#include "Clsfr.h"
#include "utils.h"
#include "report.h"


#define ETHERNET_HEADER_SIZE    14

static TI_STATUS classifier_getIpAndUdpHeader(classifier_t* pClsfr, mem_MSDU_T *pMsdu, UINT8 **pIpHeader, UINT8 **pUdpHeader);



/*************************************************************************
*                        Classifier_create                               *
**************************************************************************
* DESCRIPTION:  This function creates the Classifier module.                 
*                                                      
* INPUT:        hOs - handle to Os Abstraction Layer
*               
* OUTPUT:       Pointer to the Classifier module
************************************************************************/

classifier_t* Classifier_create(TI_HANDLE hOs)
{
    classifier_t*   pClsfr;
    
    if( hOs  == NULL )
		return NULL;
   

    /* alocate classifier block */
    pClsfr = os_memoryAlloc(hOs, (sizeof(classifier_t)));

    if (!pClsfr) 
    {
        utils_nullMemoryFree(hOs, pClsfr, sizeof(classifier_t));
        return NULL;
    }

    /* clear the block */
    os_memoryZero(hOs, pClsfr, (sizeof(classifier_t)));

    pClsfr->hOs = hOs;

    return(pClsfr);
}



/******************************************************************************
*                        Classifier_config                                    *
*******************************************************************************
* DESCRIPTION:  This function configures the Classifier module.                 
*                                                      
* INPUT:        hOs, hReport - handle to Os Abstraction Layer and to the Report
*               
* OUTPUT:       PARAM_VALUE_NOT_VALID in case of problems with input parameters
*               and OK otherwise
********************************************************************************/


TI_STATUS Classifier_config(classifier_t* pClsfr, TI_HANDLE hOs, TI_HANDLE hReport, clsfr_Params_t* ClsfrInitParams)
{
    int i,j,actualEntryCount;
    BOOL conflictFound;
    
    /* check parameters validity */
    if (pClsfr == NULL) 
		return NOK;
    
	if ( (hOs == NULL) || (hReport == NULL) )
		return NOK;

    /* set objects handles */
    pClsfr->hOs = hOs;
    pClsfr->hReport = hReport;
    
    /* Active classification algorithm */
    pClsfr->clsfrParameters.clsfrType = ClsfrInitParams->clsfrType;

    /* the number of active entries */
    if (ClsfrInitParams->NumOfActiveEntries <= NUM_OF_CLSFR_TABLE_ENTRIES)
        pClsfr->clsfrParameters.NumOfActiveEntries = ClsfrInitParams->NumOfActiveEntries;
    else 
        pClsfr->clsfrParameters.NumOfActiveEntries = NUM_OF_CLSFR_TABLE_ENTRIES;

    /* Initialization of the classification table */
    switch (pClsfr->clsfrParameters.clsfrType)
    {
        case D_TAG_CLSFR:
			pClsfr->clsfrParameters.NumOfActiveEntries = 0;
        break;
        
        case DSCP_CLSFR:
            actualEntryCount=0;
            for (i=0; (i < pClsfr->clsfrParameters.NumOfActiveEntries ) ; i++)
            {
               conflictFound = FALSE;
                /* check conflict */
                for (j=0;j<i;j++)
                {
                   /* Detect both duplicate and conflicting entries */
                    if (pClsfr->clsfrParameters.ClsfrTable[j].Dscp.CodePoint == ClsfrInitParams->ClsfrTable[i].Dscp.CodePoint)
                    {
                        WLAN_REPORT_WARNING (pClsfr->hReport, CLSFR_MODULE_LOG,("ERROR: Classifier_config(): duplicate/conflicting classifier entries\n"));
                        conflictFound = TRUE;
                    }
                }
                if (conflictFound == FALSE)
                {
                  pClsfr->clsfrParameters.ClsfrTable[actualEntryCount].Dscp.CodePoint = ClsfrInitParams->ClsfrTable[i].Dscp.CodePoint;
                  pClsfr->clsfrParameters.ClsfrTable[actualEntryCount].DTag = ClsfrInitParams->ClsfrTable[i].DTag;
                  actualEntryCount++;
                }
            }
            pClsfr->clsfrParameters.NumOfActiveEntries = actualEntryCount;
        break;
        case PORT_CLSFR:
           actualEntryCount=0;
            for (i=0; (i < pClsfr->clsfrParameters.NumOfActiveEntries ) ; i++)
            {
				conflictFound = FALSE;
                /* check conflict */
                for (j=0;j<i;j++)
                {
                    /* Detect both duplicate and conflicting entries */
                    if (pClsfr->clsfrParameters.ClsfrTable[j].Dscp.DstPortNum == ClsfrInitParams->ClsfrTable[i].Dscp.DstPortNum)
                    {
                        WLAN_REPORT_WARNING (pClsfr->hReport, CLSFR_MODULE_LOG,("ERROR: Classifier_config(): classifier entries conflict\n"));
                        conflictFound = TRUE;
                    }
                }
                if (conflictFound == FALSE)
                {
                  pClsfr->clsfrParameters.ClsfrTable[actualEntryCount].Dscp.DstPortNum = ClsfrInitParams->ClsfrTable[i].Dscp.DstPortNum;
                  pClsfr->clsfrParameters.ClsfrTable[actualEntryCount].DTag = ClsfrInitParams->ClsfrTable[i].DTag;
                  actualEntryCount++;
                }
            }
            pClsfr->clsfrParameters.NumOfActiveEntries = actualEntryCount;
        break;    
        case IPPORT_CLSFR:
           actualEntryCount=0;
            for (i=0; (i < pClsfr->clsfrParameters.NumOfActiveEntries ) ; i++)
            {
				conflictFound = FALSE;
                /* check conflict */
                for (j=0;j<i;j++)
                {
                   /* Detect both duplicate and conflicting entries */
                    if ((pClsfr->clsfrParameters.ClsfrTable[j].Dscp.DstIPPort.DstIPAddress == ClsfrInitParams->ClsfrTable[i].Dscp.DstIPPort.DstIPAddress)&& 
                    (pClsfr->clsfrParameters.ClsfrTable[j].Dscp.DstIPPort.DstPortNum == ClsfrInitParams->ClsfrTable[i].Dscp.DstIPPort.DstPortNum))
                    {
                        WLAN_REPORT_WARNING (pClsfr->hReport, CLSFR_MODULE_LOG,("ERROR: Classifier_config(): classifier entries conflict\n"));
                        conflictFound = TRUE;
                    }
                }
                if (conflictFound == FALSE)
                {
                  pClsfr->clsfrParameters.ClsfrTable[actualEntryCount].Dscp.DstIPPort.DstIPAddress = ClsfrInitParams->ClsfrTable[i].Dscp.DstIPPort.DstIPAddress;
                  pClsfr->clsfrParameters.ClsfrTable[actualEntryCount].Dscp.DstIPPort.DstPortNum = ClsfrInitParams->ClsfrTable[i].Dscp.DstIPPort.DstPortNum;
                  pClsfr->clsfrParameters.ClsfrTable[actualEntryCount].DTag = ClsfrInitParams->ClsfrTable[i].DTag;
                  actualEntryCount++;
                }
            }
            pClsfr->clsfrParameters.NumOfActiveEntries = actualEntryCount;
        break;    
        default:
            WLAN_REPORT_WARNING (pClsfr->hReport, CLSFR_MODULE_LOG,("ERROR: Classifier_config(): Classifier type -- unknown --> set to D-Tag\n"));
            pClsfr->clsfrParameters.clsfrType = D_TAG_CLSFR;
			pClsfr->clsfrParameters.NumOfActiveEntries = 0;
    }
    
    return OK;

}


/******************************************************************************
*                        Classifier_destroy                                    *
*******************************************************************************
* DESCRIPTION:  This function destroys the Classifier module.                 
*                                                      
* INPUT:        the object
*               
* OUTPUT:       NOK in case of problems with the input parameter
*               and OK otherwise
********************************************************************************/

TI_STATUS Classifier_destroy(classifier_t* pClsfr) 
{

    /* check parameters validity */
    if( pClsfr == NULL )
        return NOK;

    /* free the classifier memory block */
    os_memoryFree(pClsfr->hOs, pClsfr, sizeof(classifier_t));
    return OK;
}



/************************************************************************
 *                        Classifier_classifyTxMSDU
 ************************************************************************
      
Input:  

* pClsfr: pointer to the classifier 
* pMsdu: pointer to the MSDU
* packet_DTag: NDIS Packet 802.1 user priority (UP)

Output:  

OK on success and PARAM_VALUE_NOT_VALID in case of input parameters problems.
If the value PARAM_VALUE_NOT_VALID is returned, the MSDU qosTag field is zero. 

Description:  

This function performs the classification algorithm for the MSDU pointed 
by pMsdu, according to the classifier parameters. 
It initializes the qosTag field of the MSDU with the classification algorithm 
returned value. Note that if the value in the field clsfrType of Clsfr_Params is 
D_TAG_CLSFR then it performs the trivial classification algorithm from 
D-tag to D-tag. That is, Msdu->qosTag is set to packet_DTag. 
For all other classification algorithms, the classification is performed 
according to the corresponding classifier table.
  
************************************************************************/


TI_STATUS Classifier_classifyTxMSDU(classifier_t* pClsfr, mem_MSDU_T *pMsdu, UINT8 packet_DTag)
{
   
    UINT8               i;
    UINT8               *pUdpHeader = NULL;
    UINT8               *pIpHeader = NULL;
    UINT8               DSCP;
    UINT16              dstPortNum;
    UINT32              dstIPAdd;

    /* Parameters validation */

	if (pClsfr == NULL)
		return NOK; 

    if (pMsdu == NULL) 
    {
		WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			(" Classifier_classifyTxMSDU() : NULL MSDU error  \n"));
        return PARAM_VALUE_NOT_VALID; 
    }

    if ((packet_DTag > MAX_NUM_OF_802_1d_TAGS) && (pClsfr->clsfrParameters.clsfrType == D_TAG_CLSFR))
    {
        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			(" Classifier_classifyTxMSDU() : packet_DTag error  \n"));
		pMsdu->qosTag = 0;
		return PARAM_VALUE_NOT_VALID;
    }
    
    /* Initialization */
    pMsdu->qosTag = 0;

    switch(pClsfr->clsfrParameters.clsfrType)
    {
        case D_TAG_CLSFR:
            /* Trivial mapping D-tag to D-tag */
            pMsdu->qosTag = packet_DTag;
            WLAN_REPORT_INFORMATION (pClsfr->hReport, CLSFR_MODULE_LOG, ("Classifier D_TAG_CLSFR. pMsdu->qosTag = %d\n",pMsdu->qosTag));
            
        break;

        case DSCP_CLSFR:
        
            if( (classifier_getIpAndUdpHeader(pClsfr, pMsdu, &pIpHeader, &pUdpHeader) != OK) || 
                (pIpHeader == NULL) )
            {
                WLAN_REPORT_INFORMATION(pClsfr->hReport, CLSFR_MODULE_LOG,
					(" Classifier_classifyTxMSDU() : DSCP clsfr, getIpAndUdpHeader error\n"));
				return PARAM_VALUE_NOT_VALID; 
            }

            /* DSCP to D-tag mapping */
            DSCP =  *((UINT8 *)(pIpHeader + 1)); /* Fetching the DSCP from the header */
            DSCP = (DSCP >> 2);
            
            /* looking for the specific DSCP, if the DSCP is found, its corresponding 
               D-tag is set to the qosTag                                           */
            for(i = 0; i<pClsfr->clsfrParameters.NumOfActiveEntries; i++ )
            {
                if (pClsfr->clsfrParameters.ClsfrTable[i].Dscp.CodePoint == DSCP)
				{
                    pMsdu->qosTag = pClsfr->clsfrParameters.ClsfrTable[i].DTag;
                    WLAN_REPORT_INFORMATION (pClsfr->hReport, CLSFR_MODULE_LOG,("Classifier DSCP_CLSFR found match - entry %d - qosTag = %d\n",i,pMsdu->qosTag));
					break;
				}
            }

        break;


        case PORT_CLSFR:
            if( (classifier_getIpAndUdpHeader(pClsfr, pMsdu, &pIpHeader, &pUdpHeader) != OK) ||
                (pUdpHeader == NULL) )
            {
                WLAN_REPORT_INFORMATION(pClsfr->hReport, CLSFR_MODULE_LOG,
					(" Classifier_classifyTxMSDU() : DstPort clsfr, getIpAndUdpHeader error\n"));
                return PARAM_VALUE_NOT_VALID; 
            }

            /* Port to D-tag mapping */
            dstPortNum = *((UINT16 *)(pUdpHeader + 2)); /* Fetching the port number from the header */
            dstPortNum = ((dstPortNum >> 8) | (dstPortNum << 8));
            
            /* looking for the specific port number, if the port number is found, its corresponding 
               D-tag is set to the qosTag                                                           */
            for(i = 0; i<pClsfr->clsfrParameters.NumOfActiveEntries; i++ )
            {
                if (pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstPortNum == dstPortNum)
				{
                    pMsdu->qosTag = pClsfr->clsfrParameters.ClsfrTable[i].DTag;
                    WLAN_REPORT_INFORMATION (pClsfr->hReport, CLSFR_MODULE_LOG,("Classifier PORT_CLSFR found match - entry %d - qosTag = %d\n",i,pMsdu->qosTag));
					break;
				}
            }
        break;

        case IPPORT_CLSFR: 
            if( (classifier_getIpAndUdpHeader(pClsfr, pMsdu, &pIpHeader, &pUdpHeader) != OK) ||
                (pIpHeader == NULL) || (pUdpHeader == NULL) )
            {
                WLAN_REPORT_INFORMATION(pClsfr->hReport, CLSFR_MODULE_LOG,
					(" Classifier_classifyTxMSDU() : Dst IP&Port clsfr, getIpAndUdpHeader error\n"));
                return PARAM_VALUE_NOT_VALID; 
            }

            /* IP&Port to D-tag mapping */
            dstPortNum = *((UINT16 *)(pUdpHeader + 2)); /* Fetching the port number from the header */
            dstPortNum = ((dstPortNum >> 8) | (dstPortNum << 8));
            {
                /* Since IP header is 2 bytes aligned we will copy IP as two 16 bits */
                /* dstIPAdd = *((UINT32 *)(pIpHeader + 16));*/
                UINT16 hiPart, loPart;
                hiPart = *((UINT16 *) pIpHeader + 8);
                loPart = *((UINT16 *) pIpHeader + 9);
                dstIPAdd = (loPart << 16) | hiPart;     // account for little endian host and network order

            }
            
            /* looking for the specific pair of dst IP address and dst port number, if it is found, its corresponding 
               D-tag is set to the qosTag                                                           */
            for(i = 0; i<pClsfr->clsfrParameters.NumOfActiveEntries; i++ )
            {
                if ((pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstIPPort.DstIPAddress == dstIPAdd)&&
                    ( pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstIPPort.DstPortNum == dstPortNum))
				{
                    pMsdu->qosTag = pClsfr->clsfrParameters.ClsfrTable[i].DTag;
                    WLAN_REPORT_INFORMATION (pClsfr->hReport, CLSFR_MODULE_LOG,("Classifier IPPORT_CLSFR found match - entry %d - qosTag = %d\n",i,pMsdu->qosTag));
					break;
				}
            }

        break;
        
        default:

            WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
					(" Classifier_classifyTxMSDU(): clsfrType error\n"));
    }

    return OK;
}



/************************************************************************
 *                        classifier_getIpAndUdpHeader
 ************************************************************************
      
Input:  

* pClsfr: pointer to the classifier 
* pMsdu: pointer to the MSDU

Output:  

* pIpHeader: pointer to the IP header
* pUdpHeader: pointer to the UDP header 

Description:  

This function fetch the addresses of the IP and UDP headers
  
************************************************************************/
static TI_STATUS classifier_getIpAndUdpHeader(classifier_t* pClsfr, mem_MSDU_T *pMsdu, UINT8 **pIpHeader, UINT8 **pUdpHeader)
{
    UINT8              ipHeaderLen=0;
    mem_BD_T*           currBD = NULL;
    UINT16              swapedTypeLength=0;

    
	/* Parameters validation */
	
	
	if (pMsdu == NULL)
    {
        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			(" classifier_getIpAndUdpHeader: NULL MSDU error \n"));
		return NOK; 
    }

    currBD = pMsdu->firstBDPtr;
    if( currBD == NULL)
    {
        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			(" classifier_getIpAndUdpHeader: first BD is NULL \n"));
		return NOK;
    }
	
    swapedTypeLength = wlan_htons(*((UINT16*)(currBD->data+currBD->dataOffset+12)) );
    
	    /* check if frame is IP according to ether type */
    if( swapedTypeLength != 0x0800)
    {
        WLAN_REPORT_INFORMATION(pClsfr->hReport, CLSFR_MODULE_LOG,
					(" classifier_getIpAndUdpHeader: swapedTypeLength is not 0x0800 \n"));
        return NOK;
    }

/*Ronnie: we could have skipped the NO_COPY_NDIS_BUFFERS ifdef (both cases are the same)*/
#ifdef NO_COPY_NDIS_BUFFERS 
    /* 
        in windows - protocols headears are in the same buffer. 
        Headears can not bu split between 2 buffers 
    */

    /* IP is in first buffer */
    *pIpHeader = (UINT8 *)(memMgr_BufData(pMsdu->firstBDPtr)+memMgr_BufOffset(pMsdu->firstBDPtr)) + ETHERNET_HEADER_SIZE;
    ipHeaderLen = ((*(unsigned char*)(*pIpHeader)  & 0x0f) * 4);


    /* UDP/TCP is in first buffer */
    *pUdpHeader = *pIpHeader + ipHeaderLen;  /* Set the pointer to the begining of the TCP/UDP header */


#else
    /* set the pointer to the beginning of the IP header  and calculate it's size*/
    *pIpHeader = (UINT8 *)(memMgr_BufData(pMsdu->firstBDPtr)+memMgr_BufOffset(pMsdu->firstBDPtr)) + ETHERNET_HEADER_SIZE;
    ipHeaderLen = ((*(unsigned char*)(*pIpHeader)  & 0x0f) * 4);
    *pUdpHeader = *pIpHeader + ipHeaderLen;  /* Set the pointer to the beggining of the TCP/UDP header */
#endif
    return OK;
}



/************************************************************************
 *                        classifier_InsertClsfrEntry                 *
 ************************************************************************
 The following API is used to configure the classifier table. Note that 
 this API provides only insert to table service (and not delete).      

Input:  

*   pClsfr: pointer to the classifier 
*   NumberOfEntries: number of entries to insert.
*   ConfigBufferPtr: pointer to the data to insert.

Output:  

OK on success and PARAM_VALUE_NOT_VALID in case of input parameters problems.
If the value PARAM_VALUE_NOT_VALID is returned, the entries insert operation 
on the classifier table is canceled. 

The value PARAM_VALUE_NOT_VALID is returned upon the following errors:
--   NumberOfEntries parameter:
    a) If it is larger than the available entries in the table.
    b) If it is smaller than 1.
    c) If an entry creates conflict with another entry. 
--   ConfigBufferPtr is pointing on NULL. 

************************************************************************/
TI_STATUS Classifier_InsertClsfrEntry(classifier_t* pClsfr, UINT8 NumberOfEntries, clsfr_tableEntry_t *ConfigBufferPtr)
{

    UINT8 avlEntries;
    clsfr_tableEntry_t *pSrc;
    int i,j;

    if(pClsfr == NULL)
		return NOK;
    
	if(ConfigBufferPtr == NULL)
    {
        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			("Classifier_InsertClsfrEntry(): NULL ConfigBuffer pointer Error - Aborting\n"));
		return PARAM_VALUE_NOT_VALID;
    }
    
    avlEntries = (NUM_OF_CLSFR_TABLE_ENTRIES - (pClsfr->clsfrParameters.NumOfActiveEntries));
    if ((NumberOfEntries < 1) || (NumberOfEntries > avlEntries))
    {
        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			("Classifier_InsertClsfrEntry(): Bad Number Of Entries - Aborting\n"));
        return PARAM_VALUE_NOT_VALID;
    }
    
    if (pClsfr->clsfrParameters.clsfrType == D_TAG_CLSFR)
    {
        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			("Classifier_InsertClsfrEntry(): D-Tag classifier - Aborting\n"));
        return PARAM_VALUE_NOT_VALID;
    }

    /* Check conflicts with classifier table entries */
    /* check all conflicts, if all entries are OK --> insert to classifier table*/
 
    pSrc = ConfigBufferPtr;
    
    switch (pClsfr->clsfrParameters.clsfrType)
    {
        case DSCP_CLSFR:
            for  (i=0; i< NumberOfEntries ; i++)  
            {   
                /* Check entry */
				if ((pSrc[i].Dscp.CodePoint > CLASSIFIER_CODE_POINT_MAX) || (pSrc[i].DTag > CLASSIFIER_DTAG_MAX)) 
                {
			        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
							("Classifier_InsertClsfrEntry(): bad parameter - Aborting\n"));
                    return PARAM_VALUE_NOT_VALID;
                }
				
				/* Check conflict*/
				for (j=0;j<pClsfr->clsfrParameters.NumOfActiveEntries;j++)
                {
                   /* Detect both duplicate and conflicting entries */
                    if (pClsfr->clsfrParameters.ClsfrTable[j].Dscp.CodePoint == pSrc[i].Dscp.CodePoint)
                        {
					        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
								("Classifier_InsertClsfrEntry(): classifier entries conflict - Aborting\n"));
                            return PARAM_VALUE_NOT_VALID;
                        }
                } 
               
            }

            /* All new entries are valid --> insert to classifier table */            
            for  (i=0; i< NumberOfEntries ; i++)  
            { 
                pClsfr->clsfrParameters.ClsfrTable[pClsfr->clsfrParameters.NumOfActiveEntries+i].Dscp.CodePoint = pSrc[i].Dscp.CodePoint;
                pClsfr->clsfrParameters.ClsfrTable[pClsfr->clsfrParameters.NumOfActiveEntries+i].DTag = pSrc[i].DTag;
            }
            
        break;
        
        case PORT_CLSFR:
            for  (i=0; i< NumberOfEntries ; i++)  
            {
				/* Check entry */
				if ((pSrc[i].DTag > CLASSIFIER_DTAG_MAX) || (pSrc[i].Dscp.DstPortNum > CLASSIFIER_PORT_MAX-1) || (pSrc[i].Dscp.DstPortNum < CLASSIFIER_PORT_MIN) )
                {
			        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
							("Classifier_InsertClsfrEntry(): bad parameter - Aborting\n"));
                    return PARAM_VALUE_NOT_VALID;
                }
				
				/* Check conflict*/
                for (j=0;j<pClsfr->clsfrParameters.NumOfActiveEntries;j++)
                {
                   /* Detect both duplicate and conflicting entries */
                    if ((pClsfr->clsfrParameters.ClsfrTable[j].Dscp.DstPortNum == pSrc[i].Dscp.DstPortNum))
                        {
					        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
								("Classifier_InsertClsfrEntry(): classifier entries conflict - Aborting\n"));
                             return PARAM_VALUE_NOT_VALID;
                        }
                }     
                                
            }    

            /* All new entries are valid --> insert to classifier table */            
            for  (i=0; i< NumberOfEntries ; i++)  
            { 
                pClsfr->clsfrParameters.ClsfrTable[pClsfr->clsfrParameters.NumOfActiveEntries+i].Dscp.DstPortNum = pSrc[i].Dscp.DstPortNum;
                pClsfr->clsfrParameters.ClsfrTable[pClsfr->clsfrParameters.NumOfActiveEntries+i].DTag = pSrc[i].DTag;
            }
                                
        break;
        
        case IPPORT_CLSFR:
            for  (i=0; i< NumberOfEntries ; i++)  
            {   
				/* Check entry */
				if ( (pSrc[i].DTag > CLASSIFIER_DTAG_MAX) || (pSrc[i].Dscp.DstIPPort.DstPortNum > CLASSIFIER_PORT_MAX-1) || 
					(pSrc[i].Dscp.DstIPPort.DstPortNum < CLASSIFIER_PORT_MIN) || (pSrc[i].Dscp.DstIPPort.DstIPAddress > CLASSIFIER_IPADDRESS_MAX-1) || 
					(pSrc[i].Dscp.DstIPPort.DstIPAddress < CLASSIFIER_IPADDRESS_MIN+1) )
                {
			        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
							("Classifier_InsertClsfrEntry(): bad parameter - Aborting\n"));
                    return PARAM_VALUE_NOT_VALID;
                }

				/* Check conflict*/
                for (j=0;j<pClsfr->clsfrParameters.NumOfActiveEntries;j++)
                {
                   /* Detect both duplicate and conflicting entries */
                    if ( (pClsfr->clsfrParameters.ClsfrTable[j].Dscp.DstIPPort.DstIPAddress == pSrc[i].Dscp.DstIPPort.DstIPAddress) && 
                         (pClsfr->clsfrParameters.ClsfrTable[j].Dscp.DstIPPort.DstPortNum == pSrc[i].Dscp.DstIPPort.DstPortNum))
                        {
					        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
								("Classifier_InsertClsfrEntry(): classifier entries conflict - Aborting\n"));
                             return PARAM_VALUE_NOT_VALID;
                        }
                }  
                
            }            

            /* All new entries are valid --> insert to classifier table */            
            for  (i=0; i< NumberOfEntries ; i++)  
            { 
                pClsfr->clsfrParameters.ClsfrTable[pClsfr->clsfrParameters.NumOfActiveEntries+i].Dscp.DstIPPort.DstIPAddress = pSrc[i].Dscp.DstIPPort.DstIPAddress;
                pClsfr->clsfrParameters.ClsfrTable[pClsfr->clsfrParameters.NumOfActiveEntries+i].Dscp.DstIPPort.DstPortNum = pSrc[i].Dscp.DstIPPort.DstPortNum;
                pClsfr->clsfrParameters.ClsfrTable[pClsfr->clsfrParameters.NumOfActiveEntries+i].DTag = pSrc[i].DTag;
            }

        break;

        default:
	        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
				("Classifier_InsertClsfrEntry(): Classifier type -- unknown - Aborting\n"));
            
    } 
    
    /* update the number of classifier active entries */
    pClsfr->clsfrParameters.NumOfActiveEntries = pClsfr->clsfrParameters.NumOfActiveEntries + NumberOfEntries; 

    return OK;
}


/************************************************************************
 *                        classifier_RemoveClsfrEntry                   *
 ************************************************************************
 The following API is used to remove an entry from the classifier table

Input:  

*   pClsfr: pointer to the classifier 
*   ConfigBufferPtr: pointer to the data to remove.

Output:  
OK on success and PARAM_VALUE_NOT_VALID in case of input parameters problems.
************************************************************************/
TI_STATUS classifier_RemoveClsfrEntry(classifier_t* pClsfr, clsfr_tableEntry_t *ConfigBufferPtr)
{
    int i,j;

    if(pClsfr == NULL)
		return NOK;
    
	if(ConfigBufferPtr == NULL)
    {
        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			("classifier_RemoveClsfrEntry(): NULL ConfigBuffer pointer Error - Aborting\n"));
		return PARAM_VALUE_NOT_VALID;
    }
    
    if (pClsfr->clsfrParameters.NumOfActiveEntries == 0)
    {
        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			("classifier_RemoveClsfrEntry(): Classifier table is empty - Aborting\n"));
		return PARAM_VALUE_NOT_VALID;
    }
   
    if (pClsfr->clsfrParameters.clsfrType == D_TAG_CLSFR)
    {
        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			("classifier_RemoveClsfrEntry(): D-Tag classifier - Aborting\n"));
        return PARAM_VALUE_NOT_VALID;
    }

    /* Check conflicts with classifier table entries */
    /* check all conflicts, if all entries are OK --> insert to classifier table*/
 
    switch (pClsfr->clsfrParameters.clsfrType)
    {
        case DSCP_CLSFR:

           /* Find the classifier entry */
           i = 0;
           while ((i < pClsfr->clsfrParameters.NumOfActiveEntries) &&
                  ((pClsfr->clsfrParameters.ClsfrTable[i].Dscp.CodePoint != ConfigBufferPtr->Dscp.CodePoint) ||
                  (pClsfr->clsfrParameters.ClsfrTable[i].DTag != ConfigBufferPtr->DTag)))
            {   
              i++;
            }

           /* If we have reached the number of active entries, it means we couldn't find the requested entry */
           if (i == pClsfr->clsfrParameters.NumOfActiveEntries)
           {
               WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			      ("classifier_RemoveClsfrEntry(): Entry not found - Aborting\n"));
               return PARAM_VALUE_NOT_VALID;
           }

           /* If there is more than 1 entry, we need to shift all the entries in the table in order to delete the requested entry */
           if (pClsfr->clsfrParameters.NumOfActiveEntries > 1)
           {
			for (j = i; j < pClsfr->clsfrParameters.NumOfActiveEntries-1; j++)
                {
                   /* Move entries */
                   pClsfr->clsfrParameters.ClsfrTable[j].Dscp.CodePoint = pClsfr->clsfrParameters.ClsfrTable[j+1].Dscp.CodePoint;
                   pClsfr->clsfrParameters.ClsfrTable[j].DTag = pClsfr->clsfrParameters.ClsfrTable[j+1].DTag;
                } 
           }
          
        break;
        
        case PORT_CLSFR:

           /* Find the classifier entry */
           i = 0;
           while ((i < pClsfr->clsfrParameters.NumOfActiveEntries) &&
                  ((pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstPortNum != ConfigBufferPtr->Dscp.DstPortNum) ||
                  (pClsfr->clsfrParameters.ClsfrTable[i].DTag != ConfigBufferPtr->DTag)))
            {   
              i++;
            }

           /* If we have reached the number of active entries, it means we couldn't find the requested entry */
           if (i == pClsfr->clsfrParameters.NumOfActiveEntries)
           {
               WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			      ("classifier_RemoveClsfrEntry(): Entry not found - Aborting\n"));
               return PARAM_VALUE_NOT_VALID;
           }

           /* If there is more than 1 entry, we need to shift all the entries in the table in order to delete the requested entry */
           if (pClsfr->clsfrParameters.NumOfActiveEntries > 1)
           {
			for (j = i; j < pClsfr->clsfrParameters.NumOfActiveEntries-1; j++)
                {
                   pClsfr->clsfrParameters.ClsfrTable[j].Dscp.DstPortNum = pClsfr->clsfrParameters.ClsfrTable[j+1].Dscp.DstPortNum;
                   pClsfr->clsfrParameters.ClsfrTable[j].DTag = pClsfr->clsfrParameters.ClsfrTable[j+1].DTag;
                } 
           }
                                
        break;
        
        case IPPORT_CLSFR:

           /* Find the classifier entry */
           i = 0;
           while ((i < pClsfr->clsfrParameters.NumOfActiveEntries) &&
                  ((pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstIPPort.DstIPAddress != ConfigBufferPtr->Dscp.DstIPPort.DstIPAddress) ||
                  (pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstIPPort.DstPortNum != ConfigBufferPtr->Dscp.DstIPPort.DstPortNum) ||
                  (pClsfr->clsfrParameters.ClsfrTable[i].DTag != ConfigBufferPtr->DTag)))
            {   
              i++;
            }

           /* If we have reached the number of active entries, it means we couldn't find the requested entry */
           if (i == pClsfr->clsfrParameters.NumOfActiveEntries)
           {
               WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			      ("classifier_RemoveClsfrEntry(): Entry not found - Aborting\n"));
               return PARAM_VALUE_NOT_VALID;
           }

           /* If there is more than 1 entry, we need to shift all the entries in the table in order to delete the requested entry */
           if (pClsfr->clsfrParameters.NumOfActiveEntries > 1)
           {
			for (j = i; j < pClsfr->clsfrParameters.NumOfActiveEntries-1; j++)
                {
                   pClsfr->clsfrParameters.ClsfrTable[j].Dscp.DstIPPort.DstIPAddress = pClsfr->clsfrParameters.ClsfrTable[j+1].Dscp.DstIPPort.DstIPAddress;
                   pClsfr->clsfrParameters.ClsfrTable[j].Dscp.DstIPPort.DstPortNum = pClsfr->clsfrParameters.ClsfrTable[j+1].Dscp.DstIPPort.DstPortNum;
                   pClsfr->clsfrParameters.ClsfrTable[j].DTag = pClsfr->clsfrParameters.ClsfrTable[j+1].DTag;
                } 
           }

        break;

        default:
	        WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
				("classifier_RemoveClsfrEntry(): Classifier type -- unknown - Aborting\n"));
            
    } 
    
    /* update the number of classifier active entries */
    pClsfr->clsfrParameters.NumOfActiveEntries--;

    return OK;
}


/************************************************************************
 *                        classifier_setClsfrType                 *
 ************************************************************************
 The following API is used to change the active classifier type. 
 In addition it empties the classifier table.

Input:  

*  pClsfr: pointer to the classifier 
*   newClsfrType: the new classifier type.

Output:  

OK on success and PARAM_VALUE_NOT_VALID in case of input parameters problems.
If the value PARAM_VALUE_NOT_VALID is returned, the classifier type and table are not updated.

************************************************************************/

TI_STATUS Classifier_setClsfrType(classifier_t* pClsfr, clsfr_type_e newClsfrType)
{
    if( pClsfr  == NULL )
        return PARAM_VALUE_NOT_VALID;

	if (newClsfrType > CLSFR_TYPE_MAX)
	{
		WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
			("Classifier_setClsfrType(): classifier type exceed its MAX \n"));
		return PARAM_VALUE_NOT_VALID;
    }
	

	if ( pClsfr->clsfrParameters.clsfrType == newClsfrType)
	{
		WLAN_REPORT_WARNING(pClsfr->hReport, CLSFR_MODULE_LOG,
			("Classifier_setClsfrType(): equal classifier type --> will empty classifier table \n"));
    }
	
	/* Update type */
    pClsfr->clsfrParameters.clsfrType = newClsfrType;
	/* Empty table */
	pClsfr->clsfrParameters.NumOfActiveEntries = 0;

    return OK;
}

TI_STATUS Classifier_getClsfrType (classifier_t* pClsfr, clsfrTypeAndSupport *newClsfrType)
{
	if (pClsfr == NULL)
		return NOK;

	newClsfrType->ClsfrType = (ULONG)pClsfr->clsfrParameters.clsfrType;
	return OK;
}


/************************************************************************
 *                        Classifier_deriveUserPriorityFromStream
 ************************************************************************
      
Input:  

* pClsfr: pointer to the classifier 
* pStream: pointer to stream properties structure

Output:  

userPriority contains the appropriate qosTag that matches the stream properties.

OK on success and PARAM_VALUE_NOT_VALID in case of input parameters problems.
If the value PARAM_VALUE_NOT_VALID is returned, the MSDU qosTag field is zero. 

Description:  
  
************************************************************************/

TI_STATUS Classifier_deriveUserPriorityFromStream (classifier_t* pClsfr, STREAM_TRAFFIC_PROPERTIES *pStream)
{
   
    UINT8               i;

  	if (pClsfr == NULL)
		return NOK; 

    /* Initialization */
    pStream->userPriority = 0;

    switch(pClsfr->clsfrParameters.clsfrType)
    {
        case DSCP_CLSFR:
            /* looking for the specific DSCP, if the DSCP is found, its corresponding D-tag is set to the qosTag */
            for(i = 0; i<pClsfr->clsfrParameters.NumOfActiveEntries; i++ )
            {
                if (pClsfr->clsfrParameters.ClsfrTable[i].Dscp.CodePoint == pStream->PktTag)
				{
                   pStream->userPriority = pClsfr->clsfrParameters.ClsfrTable[i].DTag;
					return OK;
				}
            }
        break;
        case PORT_CLSFR:
            /* looking for the specific port number, if the port number is found, its corresponding D-tag is returned */
            for(i = 0; i<pClsfr->clsfrParameters.NumOfActiveEntries; i++ )
            {
                if (pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstPortNum == pStream->dstPort)
				{
                   pStream->userPriority = pClsfr->clsfrParameters.ClsfrTable[i].DTag;
					return OK;
				}
            }
        break;
        case IPPORT_CLSFR: 
            /* looking for the specific pair of dst IP address and dst port number, if it is found, its corresponding 
               D-tag is set to the qosTag                                                           */
            for(i = 0; i<pClsfr->clsfrParameters.NumOfActiveEntries; i++ )
            {
                if ((pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstIPPort.DstIPAddress == pStream->dstIpAddress)&&
                    ( pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstIPPort.DstPortNum == pStream->dstPort))
				{
                    pStream->userPriority = pClsfr->clsfrParameters.ClsfrTable[i].DTag;
					return OK;
				}
            }
        break;
        default:
            WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
					(" Classifier_deriveUserPriorityFromStream(): clsfrType error\n"));
    }

#if 0
      WLAN_OS_REPORT (("UserPriority is %d\n",pStream->userPriority));
#endif

    return OK;
}


#ifdef TI_DBG
TI_STATUS Classifier_dbgPrintClsfrTable (classifier_t* pClsfr)
{
	int i;
	UINT32 tmpIpAddr;

    if(pClsfr == NULL)
		return NOK;
    
	if (pClsfr->clsfrParameters.clsfrType == D_TAG_CLSFR)
	{
		WLAN_OS_REPORT (("D_TAG classifier type selected...Nothing to print...\n"));
		return OK;
	}

	WLAN_OS_REPORT (("Number of active entries in classifier table : %d\n",pClsfr->clsfrParameters.NumOfActiveEntries));

	switch (pClsfr->clsfrParameters.clsfrType)
	{
		case DSCP_CLSFR:
			WLAN_OS_REPORT (("+------+-------+\n"));
			WLAN_OS_REPORT (("| Code | D-Tag |\n"));
			WLAN_OS_REPORT (("+------+-------+\n"));

			for  (i=0; i< pClsfr->clsfrParameters.NumOfActiveEntries ; i++)  
            {
				WLAN_OS_REPORT (("| %5d | %5d |\n",pClsfr->clsfrParameters.ClsfrTable[i].Dscp.CodePoint,pClsfr->clsfrParameters.ClsfrTable[i].DTag));
			}

			WLAN_OS_REPORT (("+-------+-------+\n"));
			break;
		case PORT_CLSFR:
			WLAN_OS_REPORT (("+-------+-------+\n"));
			WLAN_OS_REPORT (("| Port  | D-Tag |\n"));
			WLAN_OS_REPORT (("+-------+-------+\n"));

			for  (i=0; i< pClsfr->clsfrParameters.NumOfActiveEntries ; i++)  
            {
				WLAN_OS_REPORT (("| %5d | %5d |\n",pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstPortNum,pClsfr->clsfrParameters.ClsfrTable[i].DTag));
			}

			WLAN_OS_REPORT (("+-------+-------+\n"));
			break;
		case IPPORT_CLSFR:

			WLAN_OS_REPORT (("+-------------+-------+-------+\n"));
			WLAN_OS_REPORT (("| IP Address  | Port  | D-Tag |\n"));
			WLAN_OS_REPORT (("+-------------+-------+-------+\n"));

			for  (i=0; i< pClsfr->clsfrParameters.NumOfActiveEntries ; i++)  
            {
				tmpIpAddr = pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstIPPort.DstIPAddress;
				WLAN_OS_REPORT (("| %02x.%02x.%02x.%02x | %5d | %5d |\n",(tmpIpAddr & 0xFF),((tmpIpAddr >> 8) & (0xFF)),((tmpIpAddr >> 16) & (0xFF)),((tmpIpAddr >> 24) & (0xFF)),pClsfr->clsfrParameters.ClsfrTable[i].Dscp.DstIPPort.DstPortNum,pClsfr->clsfrParameters.ClsfrTable[i].DTag));
			}

			WLAN_OS_REPORT (("+-------------+-------+-------+\n"));
			break;
		default:
			WLAN_REPORT_ERROR(pClsfr->hReport, CLSFR_MODULE_LOG,
				("Classifier_InsertClsfrEntry(): Classifier type -- unknown - Aborting\n"));
			break;
	}

	return OK;
    

}
#endif



