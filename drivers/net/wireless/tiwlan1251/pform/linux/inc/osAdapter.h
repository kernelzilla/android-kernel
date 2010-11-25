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


#ifndef __OSADAPTER_H_
#define __OSADAPTER_H_

# include "windows_types.h"

#include "osApi.h"
#include "osDebug.h"
#include "paramOut.h"

/**/
/* This tag will appear in any crash dump of the system that occurs*/
/* subsequently. The tag will be presented in the reverse order, so*/
/* it would appear as 'TIWL' (TI Wireless Lan) if pool is dumped or*/
/* when tracking pool usage in the debugger.*/
/**/
#define TIWLN_MEM_TAG						(*(PULONG)"LWIT")

#define TI_VENDOR_ID						0x104C

#define MAX_ADAPTERS_NUM					4
#define MAX_MULTICAST_ADDRESSES				32

#define ETH_ADDR_SIZE						6
#define ETH_HEADER_SIZE                     14

#define MAX_IO_BUFFER_COMPLETE_SIZE			300



#define DISABLE_PENDING_OFF					0
#define DISABLE_PENDING_AUTO				1
#define DISABLE_PENDING_ALWAYS				2

typedef struct {
	TI_HANDLE		CoreHalCtx;            /* pointer to ConfigManager */
    NDIS_HANDLE		ConfigHandle;
    TI_HANDLE       hEvHandler;
	ULONG			LinkSpeed;
	UCHAR			CurrentAddr[ETH_ADDR_SIZE];
	BOOLEAN			bCurrentAddrFromRegistry;
	OS_802_11_WEP	DefaultWepKeys[DOT11_MAX_DEFAULT_WEP_KEYS];
	TI_HANDLE		SystemProtect;

    BOOLEAN			IntMode;
	BOOLEAN			ExtMode;

    ULONG			SlotNumber;
    ULONG			etherMaxPayloadSize;
	BOOL			EepromSupported;
	UINT8           qosClassifierTable[OS_CLSFR_TABLE_SIZE];
	PUINT8			pIoBuffer;
	PULONG			pIoCompleteBuffSize ;
	struct completion 	  *IoctlComp;
	int 				  *pCompleteReply;

	UINT8			IoCompleteBuff[MAX_IO_BUFFER_COMPLETE_SIZE];


} TIWLN_ADAPTER_T, *PTIWLN_ADAPTER_T;



#endif

