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
/*   MODULE:  ElpCtrl.c                                                           */
/*   PURPOSE: ELP controller implementation module                                    */
/*                                                                                */
/**********************************************************************************/
#include "osApi.h"
#include "whalBus_Api.h"  
#include "fsm.h"
#include "TNETWIF.h"
#include "whalHwAccess.h"
#include "ElpCtrl.h"
/* #include <linux/timer.h> */

/* Healthy timer timeout in milliseconds */
#define ELPCTRL_TIMER_TIMEOUT 100


typedef enum 
{
    ELPS_AWAKE,
    ELPS_ASLEEP,
    ELPS_WAKING_UP_WRITE,
    ELPS_WAKING_UP_READ,
    ELPS_WAKING_UP_MUX

} elpCtrl_State_e;


typedef struct _elpCtrl_t
{       
    TI_HANDLE               hOs;
    TI_HANDLE               hTNETWIF;
    TI_HANDLE               hTNETWArb;
    
    TNETWIF_callback_t      fCb;
    elpCtrl_Mode_e          mode;
    elpCtrl_State_e         state; 
    UINT32                  uElpRegister;
    BOOL                    bExitWakeUpSeq;
    BOOL                    bMuxBackNeeded;
	BOOL					bSynch;
    TI_STATUS               eReturnValue;

    failureEventCB_t        fFail;    /* upper layer failure event callback.
                                       * called when the scan command has been timer expiry */
    TI_HANDLE               hFail;    /* object parameter passed to the fFail
                                                  * when it is called */
    TI_HANDLE               hTimer;

} elpCtrl_t;

/****************************************************************************
*                      elpCtrl_TimerTimeout()
****************************************************************************
* DESCRIPTION: 
* 
* INPUTS:  hElpCtrl - the handle to the ElpCtrl module.
*      
* OUTPUT:  
* 
* RETURNS: OK
****************************************************************************/
void elpCtrl_TimerTimeout (TI_HANDLE hElpCtrl)
{
	elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;

	/* 
	* Error Reporting and Recovery
	* This Timeout means that we failed to wake up the FW 
	* so the only way out of it is to restart the device - by recovery
	*/
	/* WLAN_OS_REPORT (("elpCtrl_TimerTimeout - ELP timeout timer expired! %lu\n", jiffies)); */
    WLAN_OS_REPORT (("elpCtrl_TimerTimeout - ELP timeout timer expired!\n"));

	if (pElpCtrl->fFail)
		pElpCtrl->fFail (pElpCtrl->hFail, HW_AWAKE_FAILURE);
}

/****************************************************************************
 *                      elpCtrl_Create()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  hOs - the handle to the OS layer
 *          hReport - the handle to the report module 
 *      
 * 
 * OUTPUT:  the context of the ElpCtrl module
 * 
 * RETURNS: the context of the ElpCtrl module (NULL if error)
 ****************************************************************************/
TI_HANDLE elpCtrl_Create (TI_HANDLE hOs)
{
    elpCtrl_t *pElpCtrl;

    /* Create context */
    pElpCtrl = os_memoryAlloc (hOs, sizeof(elpCtrl_t));
    if (pElpCtrl == NULL)
    {
        WLAN_OS_REPORT (("%s: Error allocating object\n", __FUNCTION__));
        return NULL;        
    }
    os_memoryZero (hOs, pElpCtrl, sizeof(elpCtrl_t));

    pElpCtrl->hOs = hOs;
   
    /* Create timer */
    pElpCtrl->hTimer = os_timerCreate (hOs, elpCtrl_TimerTimeout, pElpCtrl);
    if (pElpCtrl->hTimer == NULL)
    {
        WLAN_OS_REPORT (("%s: Error in creating timer\n", __FUNCTION__));
        elpCtrl_Destroy (pElpCtrl);
        return NULL;
    }

    return pElpCtrl;
}


/****************************************************************************
 *                      elpCtrl_Destroy()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module.
 *      
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK
 ****************************************************************************/
int elpCtrl_Destroy (TI_HANDLE hElpCtrl)
{           
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;

    if (pElpCtrl)
    {
        if (NULL != pElpCtrl->hTimer)
        {
            os_timerDestroy (pElpCtrl->hOs, pElpCtrl->hTimer);
        }

        os_memoryFree (pElpCtrl->hOs, pElpCtrl, sizeof(elpCtrl_t));
    }

    return OK;
}


/****************************************************************************
 *                      elpCtrl_Configure()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module
 *          hTNETWIF - the handle to the TNETWIF module
 *          fCb      - ELP asynchronous operation default callback
 *      
 * OUTPUT:  
 * 
 * RETURNS: ELPCTRL_COMPLETE - if succeeded (sync.)
 *          ELPCTRL_PENDING  - if succeeded (async.)
 *          ELPCTRL_ERROR    - if failed
 ****************************************************************************/
int elpCtrl_Configure (TI_HANDLE hElpCtrl, TI_HANDLE hTNETWIF, TNETWIF_callback_t fCb)
{
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;
  
    pElpCtrl->state = ELPS_AWAKE;
    pElpCtrl->mode = ELPCTRL_MODE_KEEP_AWAKE;
    pElpCtrl->hTNETWIF = hTNETWIF;
    pElpCtrl->hTNETWArb = ((TNETWIF_t *)hTNETWIF)->hTNETWArb;
    pElpCtrl->fCb = fCb;
    pElpCtrl->bMuxBackNeeded = FALSE;
    return ELPCTRL_COMPLETE;
}
 
/****************************************************************************
 *                      elpCtrl_wakeUpSeqSM()
 ****************************************************************************
 * DESCRIPTION: SM for handling Synch & Asynch wake up sequence.
 *              The flow of the SM is by that order:
 *              WRITE (wake up) -> READ (elp) -> if awake - exit. else WRITE (mux)
 *
 * OUTPUT:      
 * 
 * RETURNS: 
 ****************************************************************************/
static void elpCtrl_wakeUpSeqSM (TI_HANDLE hElpCtrl, UINT8 module_id, TI_STATUS status)
{
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;
    
    /* Check if Arbiter announced that interrupt occurred (i.e. no need for wake up sequence) */
    if (pElpCtrl->bExitWakeUpSeq)
    {     
        /* Fw is up - we should imitate TXN_COMPLETE to announce that we are done */
        pElpCtrl->state = ELPS_AWAKE;
        /* set TXN_COMPLETE to ArbiterSM - Note: It could only happen in Asynch IF */
        pElpCtrl->fCb (pElpCtrl->hTNETWArb, DEFAULT_MODULE_ID, OK);
        return;
    }

    pElpCtrl->eReturnValue = OK;

    /*
     * This while loop will continue till the exit or when waiting for the CB due to
     * memory transfer operation pending for DMA to complete   
     */
    while (TNETWIF_PENDING != pElpCtrl->eReturnValue)
    {
        switch(pElpCtrl->state) 
        {
        case ELPS_ASLEEP:

            pElpCtrl->state = ELPS_WAKING_UP_WRITE;
            pElpCtrl->eReturnValue = TNETWIF_WriteELPOpt (pElpCtrl->hTNETWIF, 
                                                            ELPCTRL_WAKE_UP, 
                                                            DEFAULT_MODULE_ID,
                                                            elpCtrl_wakeUpSeqSM, 
                                                            hElpCtrl,
                                                            TRUE);
            break;

        
        case ELPS_WAKING_UP_WRITE:

            pElpCtrl->state = ELPS_WAKING_UP_READ;
            pElpCtrl->eReturnValue = TNETWIF_ReadELPOpt (pElpCtrl->hTNETWIF, 
                                                            (UINT8*)&pElpCtrl->uElpRegister, 
                                                            DEFAULT_MODULE_ID,
                                                            elpCtrl_wakeUpSeqSM,
                                                            hElpCtrl,
                                                            TRUE);
            break;

        case ELPS_WAKING_UP_READ:

            /* Check whether Muxing is needed */
            if (pElpCtrl->uElpRegister & ELPCTRL_WLAN_READY)
            {
                /* Fw is up, but no WLAN_READY interrupt will occur (since we disabled the mux) */
                pElpCtrl->state = ELPS_AWAKE;
                /* 
				 * set TXN_COMPLETE to ArbiterSM only if we are not working in synchronize IF 
				 *  In synch IF we set the TXN_COMPLETE at the end of this function
				 */
				if ( !pElpCtrl->bSynch)
				{
					pElpCtrl->fCb (pElpCtrl->hTNETWArb, DEFAULT_MODULE_ID, OK);
				}
            } 
            else /* Fw is asleep - Mux to WLAN_READY and let arbiter wait for interrupt */
            {
				pElpCtrl->state = ELPS_WAKING_UP_MUX;
                pElpCtrl->bMuxBackNeeded = TRUE;
#ifdef DM_USE_WORKQUEUE
                /* printk("TI: %s:\t%lu - start timeout 1000 ms\n", __FUNCTION__, jiffies); */
                os_timerStart (pElpCtrl->hOs, pElpCtrl->hTimer, ELPCTRL_TIMER_TIMEOUT * 10, FALSE);
#else
                os_timerStart (pElpCtrl->hOs, pElpCtrl->hTimer, ELPCTRL_TIMER_TIMEOUT, FALSE);
#endif
                /* 
                 * Mux to WLAN_READY and let the Arbiter wait for Txn complete 
                 */
                pElpCtrl->eReturnValue = TNETWIF_WriteELPOpt (pElpCtrl->hTNETWIF,  
                                        ELPCTRL_WAKE_UP_WLAN_READY, 
                                        DEFAULT_MODULE_ID,
                                        pElpCtrl->fCb, 
                                        pElpCtrl->hTNETWArb,
                                        TRUE);
				if(TNETWIF_PENDING == pElpCtrl->eReturnValue)
				{
                    /* If we are here then we are not using Synch IF */
					pElpCtrl->bSynch = FALSE;
				}

				/* The previous states was async and now it sync */
				if((TNETWIF_COMPLETE == pElpCtrl->eReturnValue) && (pElpCtrl->bSynch == FALSE))
				{
					   pElpCtrl->fCb (pElpCtrl->hTNETWArb, DEFAULT_MODULE_ID, OK);
				}
             }
    
             return;

        default:
            
            WLAN_OS_REPORT(("Error: %s state = %d\n", __FUNCTION__, pElpCtrl->state));
            
        }
    }
	/* If we are here then we are not using Synch IF */
	pElpCtrl->bSynch = FALSE;
}

/****************************************************************************
 *                      elpCtrl_Wake()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module
 *          bHwAvail - TRUE if HW is available           
 *      
 * OUTPUT:  
 * 
 * RETURNS: TNETWIF_COMPLETE | TNETWIF_PENDING
 ****************************************************************************/
int elpCtrl_Wake (TI_HANDLE hElpCtrl, BOOL bHwAvail)
{   
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;
    int rc = ELPCTRL_AWAKE;

    if (pElpCtrl->state == ELPS_ASLEEP)
    {           
        if (bHwAvail == FALSE)
        {
            /* 
             * Wake up the FW without mux. The mux will be done only if the FW is asleep. This is done due to a race 
             * condition in the Fw, which causes 2 interrupts in the driver - one of them is not needed
             */  
            pElpCtrl->bExitWakeUpSeq = FALSE;
			pElpCtrl->bSynch		 = TRUE;

            elpCtrl_wakeUpSeqSM (hElpCtrl, DEFAULT_MODULE_ID, OK);

            /* 
             * In Synch IF we send won't send TXN_COMPLETE from elpCtrl so we should 
			 * indicate the arbiter to roll forward its SM. 
             */
            rc = ((pElpCtrl->bSynch) ? ELPCTRL_WLAN_RDY_COMPLETE : ELPCTRL_COMPLETE);
        }
        else
        {
            pElpCtrl->state = ELPS_AWAKE;

            if (TNETWIF_WriteELPOpt (pElpCtrl->hTNETWIF, 
                                     ELPCTRL_WAKE_UP, 
                                     DEFAULT_MODULE_ID,
                                     pElpCtrl->fCb, 
                                     pElpCtrl->hTNETWArb,
                                     TRUE) == TNETWIF_COMPLETE)
            {
                rc = ELPCTRL_WLAN_RDY_COMPLETE; 
            }
            else
            {
                rc = ELPCTRL_WLAN_RDY;
            }
        }
    }   

    return rc;
}


/****************************************************************************
 *                      elpCtrl_Sleep()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module.
 * 
 * OUTPUT:  
 * 
 * RETURNS: TNETWIF_PENDING | TNETWIF_COMPLETE
 ****************************************************************************/
int elpCtrl_Sleep (TI_HANDLE hElpCtrl)
{   
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;
    int rc = TNETWIF_COMPLETE;

    if (pElpCtrl->state == ELPS_AWAKE && pElpCtrl->mode == ELPCTRL_MODE_NORMAL)
    {
        pElpCtrl->state = ELPS_ASLEEP;

        /* IRQ_SRC(0) WLAN_WUP(0) - 'more' flag is FALSE since the HW is going to sleep */
        rc = TNETWIF_WriteELPOpt (pElpCtrl->hTNETWIF, 
                                  ELPCTRL_SLEEP, 
                                  DEFAULT_MODULE_ID,
                                  pElpCtrl->fCb, 
                                  pElpCtrl->hTNETWArb,
                                  FALSE);
    }

    return rc;
}


/****************************************************************************
 *                      elpCtrl_Unmux()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module.
 * 
 * OUTPUT:  
 * 
 * RETURNS: TNETWIF_PENDING | TNETWIF_COMPLETE
 ****************************************************************************/
int elpCtrl_UnMux (TI_HANDLE hElpCtrl)
{   
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;
    int rc = (pElpCtrl->bMuxBackNeeded ? OK : NOK);


    pElpCtrl->bMuxBackNeeded = FALSE;

    return rc;
}

/****************************************************************************
 *                      elpCtrl_ReceivedIRQ()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module.
 * 
 * OUTPUT:  
 * 
 * RETURNS: TNETWIF_PENDING | TNETWIF_COMPLETE
 ****************************************************************************/
void elpCtrl_ReceivedIRQ (TI_HANDLE hElpCtrl)
{   
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;

    if (pElpCtrl->state == ELPS_WAKING_UP_MUX)
    {
        pElpCtrl->state = ELPS_AWAKE;
        /* printk("TI: %s:\t%lu - stop timeout\n", __FUNCTION__, jiffies); */
        os_timerStop (pElpCtrl->hOs, pElpCtrl->hTimer);
    }

    return;
}

/****************************************************************************
 *                      elpCtrl_Mode()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module.
 *      
 * 
 * OUTPUT:  
 * 
 * RETURNS: ELPCTRL_COMPLETE 
 ****************************************************************************/
int elpCtrl_Mode (TI_HANDLE hElpCtrl, elpCtrl_Mode_e mode)
{   
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;

    pElpCtrl->mode = mode;

    return ELPCTRL_COMPLETE;
}

/****************************************************************************
 *                      elpCtrl_isIRQComing()
 ****************************************************************************
 * DESCRIPTION: Check if IRQ is about to come from Fw - depending on the ELP state
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module.
 *      
 * 
 * OUTPUT:  
 * 
 * RETURNS: TRUE - IRQ will arrive from WLAN_READY. FALSE - Otherwise 
 ****************************************************************************/
BOOL elpCtrl_isIRQComing (TI_HANDLE hElpCtrl)
{   
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;

    return (pElpCtrl->state == ELPS_WAKING_UP_MUX ? TRUE : FALSE);
}

/****************************************************************************
 *                      elpCtrl_exitWakeUpSeq()
 ****************************************************************************
 * DESCRIPTION: Mark that exit from wake up sequence is needed and return if 
 *                wake up is already over.
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module.
 *      
 * 
 * OUTPUT:  bExitWakeUpSeq = TRUE
 * 
 * RETURNS: TRUE - IRQ will arrive from WLAN_READY. FALSE - Otherwise 
 ****************************************************************************/
elpCtrl_e elpCtrl_exitWakeUpSeq (TI_HANDLE hElpCtrl)
{   
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;

    if (pElpCtrl->state == ELPS_AWAKE)
    {   /* We are already awake */
        return ELPCTRL_AWAKE;
    }
    else /* Still in wake up sequence */
    {
        pElpCtrl->bExitWakeUpSeq = TRUE;
        return ELPCTRL_ASLEEP;
    }
}

/****************************************************************************************
 *                       elpCtrl_RegisterFailureEventCB                                                 *
 ****************************************************************************************
DESCRIPTION: Registers a failure event callback for Hw available               
                                                                                                                   
INPUT:      - hElpCtrl  - handle to the Elp Ctrl object.        
            - fCb       - the failure event callback function.\n
            - hCb       - handle to the object passed to the failure event callback function.

OUTPUT: 

RETURN:    void.
****************************************************************************************/

void elpCtrl_RegisterFailureEventCB (TI_HANDLE hElpCtrl, void *fCb, TI_HANDLE hCb)
{
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;

    pElpCtrl->fFail = (failureEventCB_t)fCb;
    pElpCtrl->hFail  = hCb;
}

/****************************************************************************
 *                      elpCtrl_Stop()
 ****************************************************************************
 * DESCRIPTION: Stop ElpCtrl module before Recovery.
 *              Move to "open": MODE_KEEP_AWAKE + STATE_ON
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module.
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK
 ****************************************************************************/
int elpCtrl_Stop(TI_HANDLE hElpCtrl)
{
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;

    /* set the init state */
    pElpCtrl->mode =  ELPCTRL_MODE_KEEP_AWAKE;
    pElpCtrl->state = ELPS_AWAKE;

    /* printk("TI: %s:\t%lu - stop timeout\n", __FUNCTION__, jiffies); */
    os_timerStop (pElpCtrl->hOs, pElpCtrl->hTimer);
    
    return OK;
}

/****************************************************************************
 *                      elpCtrl_Start()
 ****************************************************************************
 * DESCRIPTION: Stop ElpCtrl module before Recovery.
 *              Move to "open": MODE_KEEP_AWAKE + STATE_ON
 * 
 * INPUTS:  hElpCtrl - the handle to the ElpCtrl module.
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK
 ****************************************************************************/
int elpCtrl_Start(TI_HANDLE hElpCtrl)
{
    elpCtrl_t *pElpCtrl = (elpCtrl_t*)hElpCtrl;

    /* Set: SCR = 1 and WUP = 1. The pattern is 101 */
    /* NOTE: no callback needed */
    /* IRQ_SRC(1) WLAN_WUP(1)*/
    TNETWIF_WriteELPOpt (pElpCtrl->hTNETWIF, 
                         ELPCTRL_WAKE_UP, 
                         DEFAULT_MODULE_ID,
                         NULL, 
                         NULL,
                         TRUE);

    
    return OK;
}

