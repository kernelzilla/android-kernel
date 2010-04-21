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
/* Module:		TI_AdapterDBG.h*/
/**/
/* Purpose:		*/
/**/
/*--------------------------------------------------------------------------*/

#ifndef TI_ADAPTER_DBG_H
#define TI_ADAPTER_DBG_H

#ifdef __cplusplus
extern "C" {
#endif
    
/******************************************************************************

    Name:   TI_DisplayStats	
    Desc:	This function calls the driver internal debug function with the 
            supplied information.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            puDbgBuffer - A buffer holding debug parameter. The first 4-byte 
                          word is the debug function to call, and the second 
                          word is an optional parameter, whose value depends 
                          on the debug function
            uBuffSize - The size of the buffer.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_DisplayStats                 (TI_HANDLE  hAdapter, 
                                             tiUINT8*  puDbgBuffer,    
                                             tiUINT32  uBuffSize  );

    /************** logging control in linux *****************************************/
/******************************************************************************

    Name:   TI_SetReportModule	
    Desc:	This function sets the modules which reporting is performed.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pData - A pointer to the bitmask defining the modules for which report 
                    information will be printed (see TIWlanModuleLogName_e)
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetReportModule              (TI_HANDLE  hAdapter, 
                                             tiUINT8 *pData);

/******************************************************************************

    Name:   TI_GetReportModule	
    Desc:	This function retrieves the modules which reporting is performed.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pData - A pointer to the bitmask defining the modules for which report 
                    information will be printed (see TIWlanModuleLogName_e)
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetReportModule              (TI_HANDLE  hAdapter, 
                                             tiUINT8 *pData);

/******************************************************************************

    Name:   TI_SetReportSeverity	
    Desc:	This function sets the severities for which report information will
            be available. To request the information for a severity level to be
            available, include "1 << desired severity value" in the bitmask.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pData - A pointer to a 1-byte bitmask of the severities:
                     WLAN_SEVERITY_INIT         1
                     WLAN_SEVERITY_INFORMATION  2
                     WLAN_SEVERITY_WARNING      3
                     WLAN_SEVERITY_ERROR        4
                     WLAN_SEVERITY_FATAL_ERROR  5
                     WLAN_SEVERITY_SM           6
                     WLAN_SEVERITY_CONSOLE      7
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetReportSeverity            (TI_HANDLE  hAdapter, 
                                             tiUINT8* pData);

/******************************************************************************

    Name:   TI_GetReportSeverity	
    Desc:	This function retrieves the report severities bit mask.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pData - A pointer to a 1-byte word in which severity bitmask will
            be written.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetReportSeverity            (TI_HANDLE  hAdapter, 
                                             tiUINT8* pData);


/******************************************************************************

    Name:   TI_SetOsDbgState	
    Desc:	Set OS adaptation layer debug state
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uData - .
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetOsDbgState				(TI_HANDLE  hAdapter, 
                                             tiUINT32 uData);

/******************************************************************************

    Name:   TI_GetOsDbgState	
    Desc:	Get OS adaptation layer debug state
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uData - .
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetOsDbgState				(TI_HANDLE  hAdapter, 
                                             tiUINT32* puData);


/******************************************************************************

    Name:   TI_SetReportPPMode	
    Desc:	
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uData - .
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetReportPPMode				(TI_HANDLE  hAdapter, 
                                             tiUINT32 uData);

/******************************************************************************

    Name:   TI_hwWriteRegister	
    Desc:	This debug command writes a value to a register.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uRegisterAddr - 
            dwValue - 
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_hwWriteRegister              (TI_HANDLE  hAdapter, 
                                             tiUINT32 uRegisterAddr, 
                                             tiUINT32 dwValue );

/******************************************************************************

    Name:   TI_hwReadRegister	
    Desc:	This debug command reads a value of a register.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uRegisterAddr - 
            puValue - 
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_hwReadRegister               (TI_HANDLE  hAdapter, 
                                             tiUINT32 uRegisterAddr, 
                                             tiUINT32* puValue );


#ifdef __cplusplus
}
#endif

#endif /* TI_ADAPTER_DBG_H*/
