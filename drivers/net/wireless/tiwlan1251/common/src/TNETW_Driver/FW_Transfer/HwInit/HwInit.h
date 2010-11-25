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
/*	  MODULE:	HwInit.h										       */
/*    PURPOSE:	HwInit module Header file                             */
/*																		   */
/***************************************************************************/
#ifndef _HW_INIT_H_
#define _HW_INIT_H_

#include "osTIType.h"


/* State-Machine States */
typedef enum
{
	HW_INIT_STATE_IDLE,				/*  */
	HW_INIT_STATE_WAIT_BUS,	/*  */
	HW_INIT_STATE_DL_FW_CMPLT			/*  */
} hwInitSmState_e;

/* Callback function definition for EndOfRecovery */
typedef void (* EndOfHwInitCB_t)(TI_HANDLE CBObj);

/* The module object. */
typedef struct 
{
	/* Handles */
	TI_HANDLE hOs;
	TI_HANDLE hReport;
	TI_HANDLE hRecoveryCtrl;
	TI_HANDLE hWhalBus;
	TI_HANDLE hHwCtrl;
	TI_HANDLE hTNETWIF;
	TI_HANDLE hBusTxn;

	EndOfHwInitCB_t endOfHwInitCB;
	BOOL recoveryProcess;
		
	hwInitSmState_e smState;			/* The current state of the SM. */	

	PUINT8					FwBuf;		/* Firmware Image ptr */
	UINT32					FwLastAddr;	/* Firmware Image length */
	PUINT8					EEPROMBuf;	/* EEPROM Image ptr */
	UINT32 					EEPROMLen; /* EEPROM Image length */


} hwInit_t;


#endif /* _HW_INIT_H_ */
