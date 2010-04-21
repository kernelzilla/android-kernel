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

/*--------------------------------------------------------------------------*/
/* Module:		CLSFRI.H*/
/**/
/* Purpose:		This module performs the classification of the MSDU.*/
/**/
/*--------------------------------------------------------------------------*/

#ifndef __CLSFR_H__
#define __CLSFR_H__

#include "paramOut.h"
#include "memMngrEx.h"

#define CLSFR_MODULE_LOG				CTRL_DATA_MODULE_LOG

typedef struct
{
	TI_HANDLE			hOs;
	TI_HANDLE			hReport;
	clsfr_Params_t		clsfrParameters;
} classifier_t;


classifier_t* Classifier_create(TI_HANDLE hOs);

TI_STATUS Classifier_config(classifier_t* pClsfr, TI_HANDLE hOs, TI_HANDLE hReport, clsfr_Params_t *ClsfrInitParams);

TI_STATUS Classifier_destroy(classifier_t* pClsfr);

TI_STATUS Classifier_classifyTxMSDU(classifier_t* pClsfr, mem_MSDU_T *pMsdu, UINT8 packet_DTag);

TI_STATUS Classifier_InsertClsfrEntry(classifier_t* pClsfr, UINT8 NumberOfEntries, clsfr_tableEntry_t *ConfigBufferPtr);

TI_STATUS classifier_RemoveClsfrEntry(classifier_t* pClsfr, clsfr_tableEntry_t *ConfigBufferPtr);

TI_STATUS Classifier_setClsfrType(classifier_t* pClsfr, clsfr_type_e newClsfrType);  

TI_STATUS Classifier_getClsfrType (classifier_t* pClsfr, clsfrTypeAndSupport *newClsfrType);

TI_STATUS Classifier_deriveUserPriorityFromStream (classifier_t* pClsfr, STREAM_TRAFFIC_PROPERTIES *pStream);

#ifdef TI_DBG
TI_STATUS Classifier_dbgPrintClsfrTable (classifier_t* pClsfr);
#endif

#endif
