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
/*   MODULE:  PowerAuthorization_internal.h                                                */
/*   PURPOSE: PowerAuthorization internal definition file										  */
/*                                                                                */
/**********************************************************************************/
#ifndef _POWER_AUTHORIZATION_INTERNAL_H_
#define _POWER_AUTHORIZATION_INTERNAL_H_

/*****************************************************************************
 **         MACRO	                                                       **
 *****************************************************************************/

/*****************************************************************************
 **         Enums                                                    **
 *****************************************************************************/

/*****************************************************************************
 **         Types	                                                       **
 *****************************************************************************/

/*****************************************************************************
 **         Structures                                                      **
 *****************************************************************************/

typedef struct _powerAutho_t
{
	/* Internal variables and configurable parameters */
	powerAutho_PowerPolicy_e m_PowerPolicy; /* the last set PowerPolicy */
	INT32 m_AwakeRequired;
	/* 
	the power policy that was configured to the FW after call to powerAutho_CalcMinPowerLevel 
	*/
	powerAutho_PowerPolicy_e m_MinPowerLevel; 

	/* Look Up Table that sets for each MinPolicyLevel the ElpCtrl mode */
	elpCtrl_Mode_e m_ElpCtrl_Mode_LUT[POWERAUTHO_POLICY_NUM];
	
	/* Handlers of other modules used */
	TI_HANDLE hOs;
	TI_HANDLE hReport;
	TI_HANDLE hHalCtrl;

	int initComplete;
	
} powerAutho_t;


/*****************************************************************************
 **         Internal functions definitions                                  **
 *****************************************************************************/
int powerAutho_CalcMinPowerLevel(TI_HANDLE hPowerAutho);

#endif /* _POWER_AUTHORIZATION_INTERNAL_H_ */

