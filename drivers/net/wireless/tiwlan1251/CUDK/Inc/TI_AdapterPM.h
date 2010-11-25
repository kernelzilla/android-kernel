/*******************************************************************************
**+--------------------------------------------------------------------------+**
**|                                                                          |**
**| Copyright 1998-2008 Texas Instruments, Inc. - http://www.ti.com/         |**
**|                                                                          |**
**| Licensed under the Apache License, Version 2.0 (the "License");          |**
**| you may not use this file except in compliance with the License.         |**
**| You may obtain a copy of the License at                                  |**
**|                                                                          |**
**|     http://www.apache.org/licenses/LICENSE-2.0                           |**
**|                                                                          |**
**| Unless required by applicable law or agreed to in writing, software      |**
**| distributed under the License is distributed on an "AS IS" BASIS,        |**
**| WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. |**
**| See the License for the specific language governing permissions and      |**
**| limitations under the License.                                           |**
**|                                                                          |**
**+--------------------------------------------------------------------------+**
*******************************************************************************/

/*--------------------------------------------------------------------------*/
/* Module:		TI_AdapterPM.h*/
/**/
/* Purpose:		This API is responsible for the power management of the STA */
/*--------------------------------------------------------------------------*/

#ifndef TI_ADAPTER_PM_H
#define TI_ADAPTER_PM_H

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************************************

    Name:	TI_SetPowerMode
	Desc:	Set the driver’s power save profile.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    thePowerMgrProfile - one of: OS_POWER_MODE_AUTO,
                                 OS_POWER_MODE_ACTIVE,
                                 OS_POWER_MODE_SHORT_DOZE,
                                 OS_POWER_MODE_LONG_DOZE
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetPowerMode          (TI_HANDLE  hAdapter, 
                                      OS_802_11_POWER_PROFILE thePowerMgrProfile);

/******************************************************************************

    Name:	TI_GetPowerMode
	Desc:	Get the driver’s power save profile (see Power Management 
            application notes).
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    thePowerMgrProfile - one of: OS_POWER_MODE_AUTO,
                                 OS_POWER_MODE_ACTIVE,
                                 OS_POWER_MODE_SHORT_DOZE,
                                 OS_POWER_MODE_LONG_DOZE
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetPowerMode          (TI_HANDLE  hAdapter, 
                                      OS_802_11_POWER_PROFILE* thePowerMgrProfile);


/******************************************************************************

    Name:	TI_ConfigPowerManagement
    Desc:	Set the driver’s power management mode.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            thePowerMgrProfile - Power management mode:
                TI_POWER_MODE_ACTIVE: The driver keeps the WiLink™ 4.0 in Active 
                                      state all the time.
                TI_POWER_MODE_SHORT_DOZE: The WiLink™ 4.0 wakes up the host on 
                                      every beacon passes the beacon to the driver 
                                      and returns to ELP Doze immediately.
                TI_POWER_MODE_LONG_DOZE: The WiLink™ 4.0 uses advanced algorithms
                                      to provide advanced system Power Save. The 
                                      features used in WiLink™ 4.0 are:
                                      Wake on DTIM 
                                      Beacon Filtering
                TI_POWER_MODE_AUTO: Uses the TI algorithm to decide the best power
                                      profile (TNET_ACTIVE, SHORT_DOZE, LONG_DOZE) 
                                      to use. The feature used in WiLink™ 4.0 is 
                                      set to ACTIVE in high Traffic and to return 
                                      to PS in low Traffic.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_ConfigPowerManagement    (TI_HANDLE  hAdapter, 
                                         OS_802_11_POWER_PROFILE thePowerMgrProfile );

    
/******************************************************************************

    Name:	TI_SetPowerLevelPS
    Desc:	This function sets the driver's power level mode.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pPowerLevel - Sets the power level mode to ELP, PD or AWAKE
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetPowerLevelPS          (TI_HANDLE  hAdapter, 
                                         OS_802_11_POWER_LEVELS uPowerLevel );

/******************************************************************************

    Name:	TI_GetPowerLevelPS
    Desc:	This function retrieves the driver's power level mode.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uPowerLevel - A pointer to the OS_802_11_POWER_LEVELS enum type 
                          (ELP, PD or AWAKE)
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetPowerLevelPS          (TI_HANDLE  hAdapter, 
                                         OS_802_11_POWER_LEVELS* pPowerLevel );

/******************************************************************************

    Name:	TI_SetPowerLevelDefault
    Desc:	
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uPowerLevel - 
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetPowerLevelDefault     (TI_HANDLE  hAdapter, 
                                         OS_802_11_POWER_LEVELS uPowerLevel );

/******************************************************************************

    Name:	TI_GetPowerLevelDefault
    Desc:	
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pPowerLevel - 
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetPowerLevelDefault     (TI_HANDLE  hAdapter, 
                                         OS_802_11_POWER_LEVELS* pPowerLevel );
    
/******************************************************************************

    Name:	TI_SetPowerMode
	Desc:	Set the driver’s doze mode when the power level is in 
			Auto mode
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    thePowerMgrProfile - one of: OS_POWER_MODE_SHORT_DOZE,
                                 OS_POWER_MODE_LONG_DOZE
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetPowerLevelDozeMode  (TI_HANDLE  hAdapter, 
                                      OS_802_11_POWER_PROFILE thePowerMgrProfile);

/******************************************************************************

    Name:	TI_GetPowerLevelDozeMode
	Desc:	Get the driver’s doze mode when the power level is in 
			Auto mode
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    thePowerMgrProfile - one of: OS_POWER_MODE_SHORT_DOZE,
                                 OS_POWER_MODE_LONG_DOZE
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetPowerLevelDozeMode  (TI_HANDLE  hAdapter, 
                                      OS_802_11_POWER_PROFILE* thePowerMgrProfile);

#ifdef __cplusplus
}
#endif

#endif /* TI_ADAPTER_PM_H*/
