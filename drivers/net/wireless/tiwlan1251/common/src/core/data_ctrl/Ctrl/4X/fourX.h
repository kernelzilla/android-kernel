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
/*	  MODULE:													       */
/*    PURPOSE:		 							   */
/*																		   */
/***************************************************************************/
#ifndef _FOUR_X_H
#define _FOUR_X_H

#include "osTIType.h"
#include "paramIn.h"
#include "paramOut.h"
#include "Concatenator.h"
#include "DeConcatenator.h"
#include "ackEmulUtil.h"
#include "memMngrEx.h"
#include "802_11Defs.h"
#include "MsduList.h"
#include "DataCtrl_Api.h"


#define FOUR_X_MODULE_LOG						CTRL_DATA_MODULE_LOG

#define DEF_CW_MAX								1023
#define DEF_CW_MIN								15
#define DEF_CW_COMBO_CW_MIN						3
#define DEF_CW_COMBO_DIFS						64
#define DEF_CW_COMBO_SLOT						15

#define CONCATENATION_CAPABILITY_ID				0x0001
#define CONTENTION_WINDOW_CAPABILITY_ID			0x0002
#define CW_COMB_CAPABILITY_ID					0x0003
#define ACK ELIMINATION_CAPABILITY_ID			0x0004
#define ERP_PROTECTION_CAPABILITY_ID			0x0005

#define FOUR_X_PROTOCOL_VERSION_0			0x00
#define FOUR_X_INFO_ELEMENT_VERSION_0_LEN	4

#define FOUR_X_PROTOCOL_VERSION_1			0x01
#define FOUR_X_CONCAT_CAP_ID				0x0001
#define FOUR_X_CONCAT_CAP_LEN				0x0002


typedef struct 
{
	BOOL	enableDisable;
	UINT16	concatenationSize;
} concatenationParams_t;

typedef struct 
{
	UINT32	count1;
	UINT32	count2;
	UINT32	count3;
	UINT32	count4;
	UINT32	count5;
	UINT32	count6;
	UINT32	count7;
} concatenationDecisionCounter_t;

typedef struct 
{
	BOOL	enableDisable;
	UINT16	CWMin;
	UINT16	CWMax;
}contentionWindowParams_t;

typedef struct 
{
	BOOL	enableDisable;
	UINT16	DIFS;
	UINT16	SLOT;
	UINT16	CWMin;
}CWCombParams_t;

typedef struct 
{
	BOOL	enableDisable;
}ackEmulationParams_t;

typedef struct 
{
	BOOL	enableDisable;
}ERP_ProtectionParams_t;

typedef struct 
{
	UINT8						fourXProtocolVersion;
	concatenationParams_t		concatenationParams;
	contentionWindowParams_t	contentionWindowParams;
	CWCombParams_t				CWCombParams;
	ackEmulationParams_t		ackEmulationParams;
	ERP_ProtectionParams_t		ERP_ProtectionParams;
} fourX_Capabilities_t;


typedef struct
{
	TI_HANDLE			hOs;
	TI_HANDLE			hReport;
	TI_HANDLE			hMemMngr;
	TI_HANDLE			hWhalCtrl;
	TI_HANDLE			hTxData;

	/* current enable/disable status */
	BOOL				concatenationEnable;
	BOOL				CWMinEnable;
	BOOL				CWComboEnable;
	BOOL				ackEmulationEnable;
	BOOL				ERP_ProtectionEnable;

	/* desired enable/disable features  */
	BOOL				desiredConcatenationEnable;
	BOOL				desiredCWMinEnable;
	BOOL				desiredCWComboEnable;
	BOOL				desiredAckEmulationEnable;
	BOOL				desiredERP_ProtectionEnable;

	/* desired parameters */
	UINT32				desiredMaxConcatSize;
	UINT16				desiredCWMin;
	UINT16				desiredCWMax;

	/* AP supported features */
	fourX_Capabilities_t	ApFourX_Capabilities;

	/* 4x parameters */
	UINT32				currentMaxConcatSize;
	UINT16				currentCWMin;
	UINT16				currentCWMax;

	/* 4x sub modules */
	deConcatenator_t*	pDeConcatenator;
	concatenator_t*		pConcatenator;
	ackEmul_t*			pAckEmul;

    /* for debug */
    concatenationDecisionCounter_t counters;

} fourX_t;

fourX_t* fourX_create(TI_HANDLE hOs);

TI_STATUS fourX_config(fourX_t*				pFourX,
					   TI_HANDLE			hOs,
					   TI_HANDLE			hReport,
					   TI_HANDLE			hMemMngr,
					   TI_HANDLE			hWhalCtrl,
					   TI_HANDLE			hTxData,
					   fourXInitParams_t*	fourXInitParams);

TI_STATUS fourX_destroy(fourX_t* pFourX);

TI_STATUS fourX_rxMsdu(fourX_t*	pFourX, 
					   mem_MSDU_T**	rxMsduPtr);

TI_STATUS fourX_txMsduBeforInsertToQueue(fourX_t* pFourX, 
										 mem_MSDU_T** msduPtr);

TI_STATUS fourX_txMsduDeQueue(fourX_t*				pFourX,
							  mem_MSDU_T**			returnMsduPtr,
							  MsduList_t*			pMsduList,
							  hwTxInformation_t*	pHwTxInformation);


/* 4X manager */
TI_STATUS fourXManager_evalSite(fourX_t* pFourX, 
								dot11_4X_t* site4xParams,
								UINT32 *matchingLevel);

TI_STATUS fourXManager_setSite(fourX_t* pFourX, 
							   dot11_4X_t* site4xParams);

TI_STATUS fourXManager_get4xInfoElemnt(fourX_t* pFourX, 
									   dot11_4X_t* fourXInfoElemnt);



/* debug functions */
void fourX_printParams(fourX_t* pFourX);


#endif
