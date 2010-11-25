/** \file TNETWArbvSM.h
 *  \brief This is the PowerSrv module API.
 *  \author Ruthy Zaphir
 *  \date 15-May-2005
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

/****************************************************************************
 *                                                                          *
 *   MODULE:  TNETWArbvSM                                               *
 *   PURPOSE: TNETWArbvSM Module API                                    *
 *                                                                          *
 ****************************************************************************/

#ifndef _TNETWARBSM_H_
#define _TNETWARBSM_H_

#include "commonTypes.h"
#include "whalCommon.h"

/*****************************************************************************
 **         Constants                                                       **
 *****************************************************************************/

#define TNETWARB_SM_MODULE_LOG /*POWER_MANAGER_SM_MODULE_LOG*/


/*****************************************************************************
 **         Enumerations                                                    **
 *****************************************************************************/


/** \enum TnetwArbSMEvents_e */
enum TnetwArbSMEvents_e
{
    TNETWARBSM_EV_START,             /**< move to active command */
    TNETWARBSM_EV_BUS_AVAIL,
    TNETWARBSM_EV_HW_AVAIL,
    TNETWARBSM_EV_RESTART,
    TNETWARBSM_EV_FINISH,
    TNETWARBSM_EV_TXN_CMPLT,

    TNETWARBSM_EVENT_NUM
};

/** \enum TnetwArbSMStates_e */
enum TnetwArbSMStates_e
{
    TNETWARBSM_STATE_IDLE,              /**< TNETW ARBITER is not running any process */
    TNETWARBSM_STATE_WAIT_BUS,          /**< TNETW ARBITER is waiting to receive the Bus Semaphore */
    TNETWARBSM_STATE_WAIT_HW,           /**< TNETW ARBITER is waiting for HW to awake */
    TNETWARBSM_STATE_WAIT_BUS_AFTER_HW, /**< TNETW ARBITER is waiting for bus */
    TNETWARBSM_STATE_WAIT_TXN1,         /**< TNETW ARBITER is waiting for end of bus transaction [1] */
    TNETWARBSM_STATE_RUNNING,           /**< TNETW ARBITER is running at least a process */
    TNETWARBSM_STATE_WAIT_TXN2,         /**< TNETW ARBITER is waiting for end of bus transaction [2] */
 
    TNETWARB_SM_STATE_NUM
};

/*****************************************************************************
 **         Typedefs                                                        **
 *****************************************************************************/

typedef enum TnetwArbSMEvents_e TnetwArbSMEvents_e;

typedef enum TnetwArbSMStates_e TnetwArbSMStates_e;


/*****************************************************************************
 **         Structures                                                      **
 *****************************************************************************/

/** \struct TNETWArbSM_Config_t */
/*
typedef struct
{

} TNETWArbSM_Config_t;
*/

/** \struct TNETWArbSM_t */
typedef struct
{
    TI_HANDLE           hOS;                    /**<
                                                 * Handle to the OS object.
                                                 */

    TI_HANDLE           hFSM;                   /**< Handle to the FSM object.
                                                 */

    TI_HANDLE           hTNETWArb;              /* Handle to the TNET Arbiter to call it when doing actions with the TODO Queue */

    TI_HANDLE           hReport;                /**<
                                                 * Handle to the Report module.
                                                 */
    

    TI_HANDLE           hELPCtrl;               /**< Handle to the power controller object via the WhalCtrl.
                                                 * Need for configure the desired power mode policy in the system.
                                                 */

    TI_HANDLE           hBusArbiter;            /**< Handle to the power controller object via the WhalCtrl.
                                                 * Need for configure the desired power mode policy in the system.
                                                 */

    UINT32              event;                  /**< Last event sent */

    BOOL                bHwAvail;               /**< Is set to TRUE is received HW_AVAIL event */

	TI_STATUS			SMlastOperationStatus;	/**< Used to store the status of the last SM operation,
												  *	 to allow the use of non-recursive state mahcine */

} TNETWArbSM_t;


/*****************************************************************************
 **         External data definitions                                       **
 *****************************************************************************/


/*****************************************************************************
 **         External functions definitions                                  **
 *****************************************************************************/


/*****************************************************************************
 **         Public Function prototypes                                      **
 *****************************************************************************/

/**
 * \author Yossi Peery
 * \date 15-May-2005\n
 * \brief Creates the object of the PowerSrv.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the OS.\n
 * Return Value: TI_HANDLE - handle to the PowerSrv object.\n
 */
TI_HANDLE TNETWArbSM_Create (TI_HANDLE hOs);

/**
 * \author Yossi Peery
 * \date 15-May-2005\n
 * \brief Destroy the object of the PowerSrvSM.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerSrv object.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 */
TI_STATUS TNETWArbSM_Destroy (TI_HANDLE hTNETWArbSM);

/**
 * \author Yossi Peery
 * \date 15-May-2005\n
 * \brief Initialize the PowerSrvSM module.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerSrvSM object.\n
 * 2) TI_HANDLE - handle to the Report object.
 * 3) TI_HANDLE - handle to the whalCtrl object.
 * 4) TI_HANDLE - handle to the QosMgr object.
 * 5) TI_HANDLE - handle to the Mlme object.
 * 6) TI_HANDLE - handle to the SiteMgr object.
 * 7) PowerSrvInitParams_t - the Power Server initialize parameters.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 */
TI_STATUS TNETWArbSM_Init (TI_HANDLE hTNETWArbSM, TI_HANDLE hReport, TI_HANDLE hTNETWArb, TI_HANDLE hELPCtrl, TI_HANDLE hBusArbiter);

/**
 * \author Yossi Peery
 * \date 15-May-2005\n
 * \brief trigger events from the outside of the module into the state machine.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerSrvSM object.\n
 * 2) TnetwArbSMEvents_e - the input events to the state machine.
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * this function will trigger the manager of the state macine (PowerSrvSM_SMEvent()).
 */
TI_STATUS TNETWArbSM_SMEvent(TI_HANDLE hTNETWArbSM,
                           TnetwArbSMEvents_e theSMEvent);


void TNETWArbSM_HwAvailCB(TI_HANDLE hTNETWArbSM);
void TNETWArbSM_TxnCb(TI_HANDLE hTNETWArbSM);


#endif /*  _GWSI_POWER_SRV_SM_H_  */
