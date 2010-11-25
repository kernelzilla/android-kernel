/** \file connSelf.h
 *  \brief Self IBSS connection header file
 *
 *  \see connSelf.c
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
/*																									*/
/*	  MODULE:	connSelf.h																*/
/*    PURPOSE:	Self IBSS connection header file			 								*/
/*																									*/
/***************************************************************************/
#if 0
#ifndef __CONN_SELF_H__
#define __CONN_SELF_H__

#include "osTIType.h"
#include "paramOut.h"
#include "paramIn.h"
#include "conn.h"

/* Self IBSS connection states */
typedef enum
{
	CONN_SELF_STATE_IDLE 			= 0,
	CONN_SELF_STATE_WAIT		    = 1,
	CONN_SELF_STATE_RSN_WAIT		= 2,
	CONN_SELF_STATE_CONNECTED 		= 3,
    CONN_SELF_NUM_STATES            = 4,
} smeState_t;

typedef enum 
{
	CONN_SELF_START					= 0,
	CONN_SELF_STOP					= 1,
	CONN_SELF_RSN_SUCC				= 2,
	CONN_SELF_RSN_FAIL				= 3,
	CONN_SELF_STA_JOINED			= 4,
	CONN_SELF_TIMEOUT				= 5,
    CONN_SELF_NUM_EVENTS            = 6,
} connSelfEvent_e;

TI_STATUS conn_selfConfig(conn_t *pConn);

TI_STATUS conn_selfSMEvent(UINT8 *currentState, UINT8 event, TI_HANDLE hConn);


#endif /* __CONN_SELF_H__ */
#endif
