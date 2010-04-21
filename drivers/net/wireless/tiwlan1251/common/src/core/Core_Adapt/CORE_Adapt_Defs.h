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

/****************************************************************************
 *
 *   MODULE:  CORE_Adapt_Defs_Defs.h
 *   PURPOSE: Core Adapt component structures definitions
 *
 ****************************************************************************/

#ifndef __CORE_ADAPT_DEFS_H__
#define __CORE_ADAPT_DEFS_H__


#include "paramOut.h"

/*
 * --------------------------------------------------------------
 *					for Core API
 * --------------------------------------------------------------
 */					 

typedef enum
{
  CORE_MANAGEMENT_FRAME = 0,
  CORE_CONTROL_FRAME    = 1,
  CORE_DATA_FRAME       = 2,
  CORE_RESERVED_FRAME   = 3,
} CoreAdapt_Tx_frameType_e;

#if 0
typedef struct
{
	CoreAdapt_Tx_frameType_e  txFrameType;
	rate_e					txRequestRate;	
	modulationType_e		txRequestModulation;
	rate_e					txActualRate;	
	modulationType_e		txActualModulation;
	int 					txStatus; /* OK/NOK of frame transmittion*/
	void					*MsduAddr;
    int						txNumWaiting;
	/* voice synch trigger for scan */
    /* disassociate packet trigger to ibssCon SM */
	UINT32					txDescFlags ;

} CoreAdapt_TxCmplt_attr_t; 

typedef enum
{
	CORE_RX_SUCCESS		= 0,
	CORE_RX_STATUS_ERROR,
	CORE_RX_MIC_FAIL
} CoreAdapt_Rx_status_e;

typedef struct
{
	CoreAdapt_Rx_status_e Status;
	UINT8       Lna;
	UINT32		TimeStamp;
	rate_e		Rate;				
	modulationType_e Modulation;
	UINT8		SNR;
	UINT8		RxLevel;	
	UINT8		channel;
} CoreAdapt_Rx_attr_t; 

/* Callback for tx compleate */
typedef void (*TxCompleteStatusCB_t)(TI_HANDLE hCtrlData, CoreAdapt_TxCmplt_attr_t* CmpltTxAttr);
/* Callback for tx compleate */
typedef void (*TxCompleteStatusCB_t)(TI_HANDLE CB_handle, whalTxCmplt_attr_t* CmpltTxAttr);
/* Callback for tx sendPacketTranfer */
typedef void (* SendPacketTranferCB_t)(TSendXferStatus_e aStatus,UINT32 aPacketId,void *reserved);


/* Callback for rx msdu */
typedef void (*msduReceiveCB_t)(TI_HANDLE hRx, mem_MSDU_T *pMsdu, CoreAdapt_Rx_attr_t* pRxAttr);

#endif


/* Scan complete Callback - This routine is called from the HAL upon TNET scan complete */
typedef void (*scanCompleteCB_t)(TI_HANDLE hScan);

/* SME scan complete CB - This function is called by the scan service when it wants to indicate scan_complete event (to the sme)*/
typedef void (*smeScanCompleteCB_t)(TI_HANDLE hSme);

/* Incoming Info Callback */
typedef void (*InfoCB_t)(TI_HANDLE handle, char* buf, UINT32 bufSize);

/* Mac status Callback */
typedef void (*MacStatusCB_t)(TI_HANDLE handle, char* str , UINT32 strLen);

/* Health Report Callback */
typedef void (*HealthReportCB_t)(TI_HANDLE handle, char* str , UINT32 strLen);

/* Aci Indication Callback */
typedef void (*AciIndicationCB_t)(TI_HANDLE handle, char* str , UINT32 strLen);

/* Failure Event Callback */
typedef void (*failureEventCB_t)(TI_HANDLE siteMgr, failureEvent_e failureEvent);

/*
 * --------------------------------------------------------------
 *					Indication definitions
 * --------------------------------------------------------------
 */					 



/*
 * --------------------------------------------------------------
 *					Internal Core_Adapt attributes
 * --------------------------------------------------------------
 */					 

typedef struct _InternalCoreAdapt_attr_t
{
	UINT8	NumFrags;
	BOOL	tkipBitEnable;
	UINT8	Frag;
	UINT16	FrameControl;
	UINT32	HwMpduAddr;
	UINT8	RtsSet;
	UINT32	MpduSize;
	UINT8   numRequiredDataBlks;
	UINT32  RtsThreshold;
} InternalCoreAdapt_attr_t;

#endif /* __CORE_ADAPT_DEFS_H__ */
