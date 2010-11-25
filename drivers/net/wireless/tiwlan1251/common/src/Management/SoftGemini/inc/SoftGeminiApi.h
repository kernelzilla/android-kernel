/** \file SoftGeminiApi.h
 *  \brief BlueTooth-Wlan coexistence module interface header file
 *
 *  \see SoftGemini.c & SoftGemini.h
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
/*																			*/
/*	  MODULE:	SoftGeminiApi.h												*/
/*    PURPOSE:	BlueTooth-Wlan coexistence module interface header file		*/
/*																			*/
/***************************************************************************/
#ifndef __SOFT_GEMINI_API_H__
#define __SOFT_GEMINI_API_H__

#include "paramOut.h"


/************************************************************************
 *                        SoftGemini_create									*
 ************************************************************************
DESCRIPTION: SoftGemini module creation function, called by the config mgr in creation phase 
				performs the following:
				-	Allocate the SoftGemini handle
				                                                                                                   
INPUT:      hOs -			Handle to OS		


OUTPUT:		

RETURN:     Handle to the SoftGemini module on success, NULL otherwise

************************************************************************/
TI_HANDLE SoftGemini_create(TI_HANDLE hOs);


/************************************************************************
 *                        SoftGemini_config						*
 ************************************************************************
DESCRIPTION: SoftGemini module configuration function, called by the config mgr in configuration phase
				performs the following:
				-	Reset & initializes local variables
				-	Init the handles to be used by the module
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle
			List of handles to be used by the module
			pSoftGeminiInitParams	-	Init table of the module.		


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS SoftGemini_config(TI_HANDLE 	hSoftGemini,
								TI_HANDLE		hCtrlData,
					  			TI_HANDLE		hHalCtrl,
								TI_HANDLE		hReport,
								TI_HANDLE		hSCR,
								TI_HANDLE		hPowerMgr,
								TI_HANDLE		hConfigMgr,
								TI_HANDLE		hScanCncn,
								TI_HANDLE		hCurrBss,
								TI_HANDLE		hEvHandler,
								SoftGeminiInitParams_t *pSoftGeminiInitParams);


/************************************************************************
 *                        SoftGemini_destroy							*
 ************************************************************************
DESCRIPTION: SoftGemini module destroy function, called by the config mgr in the destroy phase 
				performs the following:
				-	Free all memory aloocated by the module
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle.		


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS SoftGemini_destroy(TI_HANDLE hSoftGemini);


/***********************************************************************
 *                        SoftGemini_setParam									
 ***********************************************************************
DESCRIPTION: SoftGemini set param function, called by the following:
			-	config mgr in order to set a parameter receiving from the OS abstraction layer.
			-	From inside the driver	
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle.
			pParam	-	Pointer to the parameter		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS SoftGemini_setParam(TI_HANDLE		hSoftGemini,
											paramInfo_t	*pParam);

/***********************************************************************
 *                        SoftGemini_getParam									
 ***********************************************************************
DESCRIPTION: SoftGemini get param function, called by the following:
			-	config mgr in order to get a parameter from the OS abstraction layer.
			-	From inside the dirver	
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle.
			

OUTPUT:		pParam	-	Pointer to the parameter		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS SoftGemini_getParam(TI_HANDLE		hSoftGemini,
											paramInfo_t	*pParam);

/***********************************************************************
 *                        SoftGemini_printParams									
 ***********************************************************************
DESCRIPTION: prints soft gemini params
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle.
			

OUTPUT:	

RETURN:     
************************************************************************/

void SoftGemini_printParams(TI_HANDLE hSoftGemini);

/***********************************************************************
 *                        SoftGeminiReconnect									
 ***********************************************************************
DESCRIPTION: causes driver to disconnect and reconnect to current AP
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle.
			

OUTPUT:	

RETURN:     
************************************************************************/



/***************************************************************************
*					SoftGemini_SenseIndicationCB  		    	       *
****************************************************************************
* DESCRIPTION:	This is the the function which is called for sense mode indicaton from FW
*				(i.e. we are in SENSE mode and FW detects BT activity 
*				SenseModeEnable_Bit - Indicates that FW detected BT activity
*				SenseModeDisable_Bit - Indicates that FW doesn't detect BT activity for a period of time
*
* INPUTS:		pSoftGemini - the object
* NOTE			This function is located in the API for debug purposes
***************************************************************************/
void SoftGemini_SenseIndicationCB( TI_HANDLE hSoftGemini, char* str, UINT32 strLen );


/***************************************************************************
*					SoftGemini_ProtectiveIndicationCB  		    	       *
****************************************************************************
* DESCRIPTION:	This is the the function which is called when FW starts Protective mode (i.e BT voice) 
*
*				ProtectiveModeOn_Bit - FW is activated on protective mode (BT voice is running)
*				ProtectiveModeOff_Bit - FW is not activated on protective mode
*
* INPUTS:		pSoftGemini - the object
* NOTE			This function is located in the API for debug purposes
***************************************************************************/

void SoftGemini_ProtectiveIndicationCB( TI_HANDLE hSoftGemini, char* str, UINT32 strLen );


/***************************************************************************
*					SoftGemini_AvalancheIndicationCB  		    	       *
****************************************************************************
* DESCRIPTION:	This is the the function which is called when 
*				FW detect that our current connection quality is reducing
*				(AP decrease his rates with his rate adaptation mechanism because
*				of BT activity) the solution is reconnect to the same AP
*
* INPUTS:		pSoftGemini - the object
* NOTE			This function is located in the API for debug purposes
***************************************************************************/

void SoftGemini_AvalancheIndicationCB( TI_HANDLE hSoftGemini, char* str, UINT32 strLen );


/***************************************************************************
*					SoftGemini_getSGMode  		    	       *
****************************************************************************
* DESCRIPTION:	This is the the function which is called when 
*				Recovery was issued -to read the last soft gemini mode
*
* INPUTS:		pSoftGemini - the object
* 			
***************************************************************************/
SoftGeminiEnableModes_e SoftGemini_getSGMode(TI_HANDLE hSoftGemini);

TI_STATUS SoftGemini_handleRecovery(TI_HANDLE hSoftGemini);

void SoftGemini_startPsPollFailure(TI_HANDLE hSoftGemini);

void SoftGemini_endPsPollFailure(TI_HANDLE hSoftGemini);

void SoftGemini_SetPSmode(TI_HANDLE hSoftGemini);

void SoftGemini_unSetPSmode(TI_HANDLE hSoftGemini);


#endif /* __SOFT_GEMINI_API_H__ */
