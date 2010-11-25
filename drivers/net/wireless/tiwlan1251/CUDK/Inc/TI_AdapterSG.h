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
/* Module:		TI_AdapterSG.h*/
/**/
/* Purpose:		This API enables / disables the Bluetooth coexistence support, */
/*              sets the parameters of the Bluetooth coexistence feature and */
/*              retrieve its status. These functions are usually unavailable, */
/*              unless the WiLinkÂ™ 4.0 WLAN driver is specifically compiled to */
/*              support Bluetooth coexistence.*/
/**/
/*--------------------------------------------------------------------------*/

#ifndef TI_ADAPTER_SG_H
#define TI_ADAPTER_SG_H

#ifdef __cplusplus
extern "C" {
#endif
    
 #include "softGeminiTypes.h"
    

/******************************************************************************

    Name:   TI_SetBtCoeEnable	
    Desc:	This function enables and disables the Bluetooth coexistence feature
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    ModeEnable - One of the following values:
                  SG_ENABLE(0) –WLAN enabled
                  SG_DISABLE(1) – WLAN disabled
                  SG_SENSE_NO_ACTIVITY(2) –WLAN disabled, waiting for a BTH 
                                           sense interrupt
                  SG_SENSE_ACTIVE(3) - WLAN enabled, waiting for a BTH sense 
                                       interrupt.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetBtCoeEnable               (TI_HANDLE  hAdapter, 
                                             tiUINT32 ModeEnable );

/******************************************************************************

    Name:   TI_SetBtCoeRate	
    Desc:	This function sets the rate to be used when Bluetooth coexistence 
            feature is enabled.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pRate - One of the following values:
                    0 – 11 Mbps
                    1 – 5.5 and 11 Mbps
                    2 – 5.5, 11 and 22 Mbps
                    3 – 11 and 22 Mbps.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetBtCoeRate                 (TI_HANDLE  hAdapter, 
                                             tiUINT8 *pRate );

/******************************************************************************

    Name:   TI_SetBtCoeConfig	
    Desc:	This function configures Bluetooth coexistence parameters.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    pConfig - Pointer that holds the bluetooth coexistence paramaters, 
              in the following order (each parameter is 32 bits long):
              - the length inmsec of the BT period in the TDM
              - the length in msec of the WLAN period in the TDM
              - the length in msec of the BT period in the TDM in AFH mode
              - the length in msec of the BOTH period in the TDM in AFH mode
              - min time in msec between last BT activity & defining BT as 
                inactive
              - min time in msec between last WLAN activity & defining WLAN 
                as inactive
              - min time in msec between last WLAN activity & defining WLAN as
                in RxGuard mode
              - the maximum length of time the BT HP will be respected
              - the maximum length of time the WLAN HP will be respected
              - the length of time when working in SENSE mode that the BT needs
                to be inactive in order to DISABLE the SG
              - the length in msec of Bt time between every WLAN RxGuard.
              - the length in msec left for Wlan in RxGuard state
              - WLAN HW generated mode during BT period
              - specifies whether to use the AFH information from the BT
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetBtCoeConfig               (TI_HANDLE  hAdapter, 
                                             tiUINT32 *pConfig );

/******************************************************************************

    Name:   TI_SetBtCoeGetStatus	
    Desc:	This function retrieves Bluetooth coexistence feature parameters.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pStatus - Pointer that holds the Bluetooth coexistence feature 
            status. 16 4-byte words should be allocated.
            The 1st word will hold the enable status (set by TI_SetBtCoeEnable).
            The 2nd word will hold the rate (set by TI_SetBtCoeRate).
            Words 3 to 15 will hold all the parameters set by TI_SetBtCoeConfig.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetBtCoeGetStatus            (TI_HANDLE  hAdapter, 
                                             tiUINT32 *pStatus );
#ifdef __cplusplus
}
#endif

#endif /* TI_ADAPTER_SG_H*/
