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
/* Module:		TI_AdapterGWSI.h*/
/**/
/* Purpose:		*/
/**/
/*--------------------------------------------------------------------------*/

#ifndef TI_ADAPTER_GWSI_H
#define TI_ADAPTER_GWSI_H

#ifdef __cplusplus
extern "C" {
#endif
    
/******************************************************************************

    Name:   TI_GWSICommand	
    Desc:	
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pGWSICommand - .
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GWSICommand                  (TI_HANDLE  hAdapter, 
                                             tiUINT32* pGWSICommand );

/******************************************************************************

    Name:   TI_GWSIInitialize	
    Desc:	
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pGWSICommand - .
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GWSIInitialize               (TI_HANDLE  hAdapter, 
                                             tiUINT32* pGWSICommand );

/******************************************************************************

    Name:   TI_GWSIConfig	
    Desc:	
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pGWSICommand - .
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GWSIConfig                   (TI_HANDLE  hAdapter, 
                                             tiUINT32* pGWSICommand );

/******************************************************************************

    Name:   TI_GWSIGetInitTable	
    Desc:	
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pGWSICommand - .
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GWSIGetInitTable             (TI_HANDLE  hAdapter, 
                                             tiUINT32* pGWSICommand );

/******************************************************************************

    Name:   TI_GWSIRelease	
    Desc:	
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pGWSICommand - .
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GWSIRelease					(TI_HANDLE  hAdapter, 
                                             tiUINT32* pGWSICommand );

#ifdef __cplusplus
}
#endif

#endif /* TI_ADAPTER_GWSI_H*/
