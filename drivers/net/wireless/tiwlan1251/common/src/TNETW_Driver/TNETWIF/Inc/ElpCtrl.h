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

/**********************************************************************************/
/*                                                                                */
/*   MODULE:  ElpCtrl.h															  */
/*   PURPOSE: ElpCtrl api file													  */
/*                                                                                */
/**********************************************************************************/
#ifndef _ELP_CONTROLLER_H_
#define _ELP_CONTROLLER_H_

/*****************************************************************************
 **         MACRO	                                                       **
 *****************************************************************************/

/* ELP register commands */
#define ELPCTRL_WAKE_UP             0x1
#define ELPCTRL_WAKE_UP_WLAN_READY  0x5
#define ELPCTRL_SLEEP               0x0
/* ELP WLAN_READY bit */
#define ELPCTRL_WLAN_READY          0x2


/*****************************************************************************
 **         Enums                                                    **
 *****************************************************************************/

/* ELP Control API return values */
typedef enum 
{
    ELPCTRL_COMPLETE,
    ELPCTRL_PENDING,
    ELPCTRL_ERROR,
    ELPCTRL_AWAKE,
    ELPCTRL_ASLEEP,
    ELPCTRL_WLAN_RDY,
    ELPCTRL_WLAN_RDY_COMPLETE

} elpCtrl_e;

 /*****************************************************************************
 **         Types	                                                       **
 *****************************************************************************/


/*****************************************************************************
 **         Structures                                                      **
 *****************************************************************************/


/*****************************************************************************
 **         External functions definitions                                  **
 *****************************************************************************/
TI_HANDLE elpCtrl_Create    (TI_HANDLE hOs);
int       elpCtrl_Destroy   (TI_HANDLE hElpCtrl);
int       elpCtrl_Configure (TI_HANDLE hElpCtrl, TI_HANDLE hTNETWIF, TNETWIF_callback_t fCb);
int       elpCtrl_Wake      (TI_HANDLE hElpCtrl, BOOL bHwAvail);
int       elpCtrl_Sleep     (TI_HANDLE hElpCtrl);
int       elpCtrl_UnMux       (TI_HANDLE hElpCtrl);
int       elpCtrl_Mode      (TI_HANDLE hElpCtrl, elpCtrl_Mode_e aMode);
int       elpCtrl_Stop      (TI_HANDLE hElpCtrl);
int       elpCtrl_Start     (TI_HANDLE hElpCtrl);
BOOL      elpCtrl_isIRQComing (TI_HANDLE hElpCtrl);
elpCtrl_e elpCtrl_exitWakeUpSeq (TI_HANDLE hElpCtrl);
void      elpCtrl_ReceivedIRQ (TI_HANDLE hElpCtrl);

void 	  elpCtrl_RegisterFailureEventCB 
                            (TI_HANDLE hElpCtrl, void *fFail, TI_HANDLE hFail);


#endif /* _ELP_CONTROLLER_H_ */

