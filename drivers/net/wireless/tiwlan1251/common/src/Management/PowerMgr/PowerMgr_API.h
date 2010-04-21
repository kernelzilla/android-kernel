/** \file PowerMgr_API.h
 *  \brief This is the Power Manager module API.
 *  \author Yossi Peery
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
 *   MODULE:  Power Manager                                                 *
 *   PURPOSE: Power Manager Module API                                      *
 *                                                                          *
 ****************************************************************************/

#ifndef _POWER_MGR_API_H_
#define _POWER_MGR_API_H_

#include "osTIType.h"
#include "paramOut.h"

/*****************************************************************************
 **         Constants                                                       **
 *****************************************************************************/


/*****************************************************************************
 **         Enumerations                                                    **
 *****************************************************************************/


/*****************************************************************************
 **         Typedefs                                                        **
 *****************************************************************************/


/*****************************************************************************
 **         Structures                                                      **
 *****************************************************************************/


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
 * \author Assaf Azulay
 * \date 24-Oct-2005\n
 * \brief Creates the object of the power Manager.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the OS.\n
 * Return Value: TI_HANDLE - handle to the PowerMgr object.\n
 */
TI_HANDLE PowerMgr_create(TI_HANDLE theOsHandle);

/**
 * \author Assaf Azulay
 * \date 24-Oct-2005\n
 * \brief Destroy the object of the power Manager.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 */
TI_STATUS PowerMgr_destroy(TI_HANDLE thePowerMgrHandle);

/**
 * \author Assaf Azulay
 * \date 24-Oct-2005\n
 * \brief Initialization of the PowerMgr module.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * 2) TI_HANDLE - handle to the Power Server object.\n
 * 3) TI_HANDLE - handle to the Report object.\n
 * 4) TI_HANDLE - handle to the TrafficMonitor object.\n
 * 5) PowerMgrInitParams_t - the Power Manager initialize parameters.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 */
TI_STATUS PowerMgr_init(    TI_HANDLE               hPowerMgr,
                            TI_HANDLE               hPowerSRV,
                            TI_HANDLE               hReport,
                            TI_HANDLE               hSiteMgr,
                            TI_HANDLE               hWhalCtrl,
                            TI_HANDLE               hTrafficMonitor,
                            TI_HANDLE               hSoftGemini,
                            PowerMgrInitParams_t *  pPowerMgrInitParams);

/**
 * \author Assaf Azulay
 * \date 24-Oct-2005\n
 * \brief Start the power save algorithm of the driver and also the 802.11 PS.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * PsEnable = true, and decide on the proper power mode.
 */
TI_STATUS PowerMgr_startPS(TI_HANDLE thePowerMgrHandle);

/**
 * \author Assaf Azulay
 * \date 24-Oct-2005\n
 * \brief stop the power save algorithm of the driver and also the 802.11 PS.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * PsEnable = false, and set the power mode to active.
 */
TI_STATUS PowerMgr_stopPS(TI_HANDLE thePowerMgrHandle);

/**
 * \author Assaf Azulay
 * \date 24-Oct-2005\n
 * \brief returns the 802.11 power save status (enable / disable).
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * Return Value: BOOLEAN - TRUE if enable else FALSE.\n
*/
BOOLEAN PowerMgr_getPsStatus(TI_HANDLE thePowerMgrHandle);



/**
 * \author Assaf Azulay
 * \date 24-Oct-2005\n
 * \brief Configure of the PowerMode (auto / active / short doze / long doze).
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * 2) PowerMgr_PowerMode_e - the requested power mode (auto / active / short doze / long doze).
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * desiredPowerModeProfile = PowerMode input parameter, and set the proper power mode.
*/
TI_STATUS PowerMgr_setPowerMode(TI_HANDLE thePowerMgrHandle);


/**
 * \brief Sends the current power mode configuration to the firmware.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
*/
void PowerMgr_reloadPowerMode(TI_HANDLE hPowerMgr);


/**
 * \author Assaf Azulay
 * \date 24-Oct-2005\n
 * \brief Get the current PowerMode of the PowerMgr module.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * Return Value: PowerMgr_PowerMode_e .\n
 */
PowerMgr_PowerMode_e PowerMgr_getPowerMode(TI_HANDLE thePowerMgrHandle);
    
/**
 * \author Yossi Peery
 * \date 2-August-2004\n
 * \brief reset the power manager module due to recovry event.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * simulate the stop power save function without writing to the HW. just doing the
 * logic parts of stop power save from the power manager to it state machine.
 * the power controller and it state machine are reset in the whalCtrl recovry proccess.
*/
TI_STATUS PowerMgr_swReset(TI_HANDLE thePowerMgrHandle);



TI_STATUS powerMgr_getParam(TI_HANDLE thePowerMgrHandle,
                            paramInfo_t *theParamP);
TI_STATUS powerMgr_setParam(TI_HANDLE thePowerMgrHandle,
                            paramInfo_t *theParamP);


/**
 * \author Yossi Peery
 * \date 20-July-2004\n
 * \brief print configuration of the PowerMgr object - use for debug!
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * Return Value: void.\n
 */
void PowerMgr_printObject(TI_HANDLE thePowerMgrHandle);

TI_STATUS PowerMgr_notifyFWReset(TI_HANDLE hPowerMgr);



void powerMgr_setCpuLoad(TI_HANDLE hPowerMgr, UINT32 uCpuLoad);

void SoftGemini_startPsPollFailure(TI_HANDLE hSoftGemini);

void SoftGemini_endPsPollFailure(TI_HANDLE hSoftGemini);

#endif /*  _POWER_MGR_API_H_  */

