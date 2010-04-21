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

#ifndef _WHAL_SECURITY_H
#define _WHAL_SECURITY_H

#include "whalCommon.h"
#include "whalCtrl_api.h"
#include "whalWep.h"
#include "whalWpa.h"
#ifdef CKIP_ENABLED
#include "whalExc.h"
#endif /* CKIP_ENABLED*/

#define NO_OF_RECONF_SECUR_KEYS_PER_STATION		1 /* define the number of keys allocated on reconfigure 
													 data structure for each station*/
#define NO_OF_EXTRA_RECONF_SECUR_KEYS			3 

typedef struct
{
	TI_HANDLE hReport; /* handle to the reporter module*/
	TI_HANDLE hMemMgr; /* handle to the memory manager module*/ 
} whalSecur_config_t;


/* reconfigure security structure for reconfigure (FW reload) time*/
typedef struct
{
	BOOL reconfHwEncEnable;	  	/* save the last HW encryption Enable flag for reconfigure time*/
	BOOL reconfHwDecrEnable;	/* save the last HW decryption Enable flag for reconfigure time*/ 
	BOOL isHwEncDecrEnableValid; 
	
	UINT8 reconfDefaultKeyId;	/* save the last configured defualt key ID for reconfigure time*/ 
	BOOL  isDefaultKeyIdValid;	
	
	securityKeys_t* reconfKeys; /* save all the configured keys for reconfigure time, keys which
								   it's keyType are not NULL_KEY, are valid*/  
} securReconf_t;


/* CLASS WHAL_SECURITY*/
typedef struct _WHAL_SECURITY
{
	UINT32 numOfStations;
	cipherSuite_e securityMode;
	securReconf_t reconfData;      /* reconfigure security structure for reconfigure (FW reload) time*/
	
	WHAL_CTRL	*pWhalCtrl;		/* Pointer to the HL_HAL control module*/ 
	WHAL_WEP	*pWhalWep;
	WHAL_WPA	*pWhalWpa;
#ifdef CKIP_ENABLED
	privacy_t   *pWhalPrivacy;
	WHAL_EXC	*pWhalExc;
#endif /*CKIP_ENABLED*/
	TI_HANDLE hOs;
	TI_HANDLE hReport;
	TI_HANDLE hMemMgr;
} WHAL_SECURITY;

/* WHAL SECURITY Class API*/			    
TI_HANDLE whalSecur_Create (TI_HANDLE hOs, TI_HANDLE hWhalCtrl, UINT16 numOfStations);

int whalSecur_Config (TI_HANDLE hWhalSecur, whalSecur_config_t* pWhalSecurCfg);

int whalSecur_KeyAdd (TI_HANDLE hWhalSecur, securityKeys_t* apKey, BOOL reconfFlag, void *CB_Func, TI_HANDLE CB_handle);

int whalSecur_KeyRemove (TI_HANDLE hWhalSecur, securityKeys_t* apKey, BOOL reconfFlag, void *CB_Func, TI_HANDLE CB_handle);

int whalSecur_DefaultKeyIdSet (TI_HANDLE hWhalSecur, UINT8 aKeyId, void *CB_Func, TI_HANDLE CB_handle);

int whalSecur_HwEncDecrEnable (TI_HANDLE hWhalSecur, BOOL aHwEncEnable);

int whalSecur_SwEncEnable (TI_HANDLE hWhalSecur, BOOL aSwEncEnable);

int whalSecur_MicFieldEnable (TI_HANDLE hWhalSecur, BOOL aMicFieldEnable);

int whalSecur_SecurModeSet (TI_HANDLE hWhalSecur, cipherSuite_e aSecurMode);

cipherSuite_e whalSecur_SecurModeGet (TI_HANDLE hWhalSecur);

int whalSecur_KeysReconfig (TI_HANDLE hWhalSecur);

int whalSecur_Destroy (TI_HANDLE hWhalSecur, UINT16 numOfStations);

#endif /* _WHAL_SECURITY_H*/
