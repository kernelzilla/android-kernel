/** \file Core_AdaptCB.h
 *  \brief Core Adapt CB API	(PALAU CB's)
 *
 */
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
/*    MODULE:	Core_AdaptCB.h                                             */
/*    PURPOSE:	Core Adapt CB API  (Palau CB's)                            */
/*									   */
/***************************************************************************/
#ifndef __CORE_ADAPT_CB_H__
#define __CORE_ADAPT_CB_H__


#include "paramOut.h"



/* External data definitions */

/* External functions definitions */
/* Shall be the definitions of PALAU CB prototypes */
/* 
 * GWSI Call-Back functions:
 */

/* Callback for Join complete */


/* Function prototypes */
void GWSI_WriteMIBComplete(TI_STATUS aStatus);

void GWSI_JoinComplete (TI_STATUS Status);				/* CB method after WLAN device completes Join operation*/
void GWSI_ConfigureQueueComplete(TI_STATUS aStatus);
void GWSI_ConfigureACComplete(TI_STATUS aStatus);
void GWSI_DisconnectComplete (TI_STATUS Status);	
void GWSI_SetBSSParametersComplete (TI_STATUS Status);	/* CB method after driver delivers the BSS parameters to the WLAN device*/
void GWSI_ReadMIBComplete (TI_STATUS Status, 
					 TMIB aMib,
					 TUINT16 aLength, 
					 const void *aData);					/* CB method deliver MIB read results*/
void GWSI_AddKeyComplete(TI_STATUS aStatus);				/* CB method after setting the Key to the WLAN device*/







#endif /* __CORE_ADAPT_CB_H__*/
