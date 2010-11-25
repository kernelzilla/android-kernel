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
#ifndef _CONCATENATOR_H_
#define _CONCATENATOR_H_

#include "osTIType.h"
#include "memMngrEx.h" 

#define CONCATENATOR_MODULE_LOG		CTRL_DATA_MODULE_LOG

#define MAX_CONCAT_SIZE			4096


typedef struct
{
	TI_HANDLE			hOs;
	TI_HANDLE			hReport;
	TI_HANDLE			hMemMngr;

	/*UINT32				concat_maxConcatSize;*/

} concatenator_t;

concatenator_t* concat_create(TI_HANDLE hOs);

TI_STATUS concat_config(concatenator_t*		pConcatenator,
						TI_HANDLE			hOs,
						TI_HANDLE			hReport,
						TI_HANDLE			hMemMngr
						/*concatInitParams_t* concatInitParams*/);

TI_STATUS concat_destroy(concatenator_t* pConcatenator);

TI_STATUS concat_concatMsduList(concatenator_t*	pConcatenator,
								mem_MSDU_T*		pFirstMsduPtr,
								mem_MSDU_T**	pReturnMsduPtr,
								UINT16			concatFlags);




#endif
