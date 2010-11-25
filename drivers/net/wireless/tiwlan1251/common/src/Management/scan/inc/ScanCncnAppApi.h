/** \file ScanCncnAppApi.h
 *  \brief This file include public definitions for the aplication scan requests adapter.\n
 *  \author Ronen Kalish
 *  \date 30-Jan-2005
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

#ifndef __SCANCNCNAPPAPI__
#define __SCANCNCNAPPAPI__

#include "paramOut.h"
#include "ScanCncnApi.h"

/*
 ***********************************************************************
 *	Constant definitions.
 ***********************************************************************
 */

 /*
 ***********************************************************************
 *	Enums.
 ***********************************************************************
 */

/*
 ***********************************************************************
 *	Typedefs.
 ***********************************************************************
 */

/*
 ***********************************************************************
 *	Structure definitions.
 ***********************************************************************
 */

/*
 ***********************************************************************
 *	External data definitions.
 ***********************************************************************
 */

/*
 ***********************************************************************
 *	External functions definitions
 ***********************************************************************
 */
/**
 * \author Ronen Kalish\n
 * \date 30-Jan-2005\n
 * \brief Parses and executes a set param command.\n
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param pParam - the param to set.\n
 * \return OK if the scan started successfuly, NOK otherwise.\n
 */
TI_STATUS scanConcentrator_setParam( TI_HANDLE hScanCncn, paramInfo_t *pParam );

/**
 * \author Ronen Kalish\n
 * \date 30-Jan-2005\n
 * \brief Parses and executes a get param command.\n
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param pParam - the param to get.\n
 * \return always PARAM_NOT_SUPPORTED (not supposed to be called).\n
 */
TI_STATUS scanConcentrator_getParam( TI_HANDLE hScanCncn, paramInfo_t *pParam );

/**
 * \author Ronen Kalish\n
 * \date 30-Jan-2005\n
 * \brief Scan result callback for application scan.\n
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param status - the scan result status (scan complete, result received etc.).\n
 * \param frameInfo - a pointer to the structure holding all frame related info (in case a frame was received).\n
 * \prama SPSStatus - a bitmap indicating on which channels scan was attempted (valid for SPS scan only!).\n
 */
void scanConcentrator_appScanResultCB( TI_HANDLE hScanCncn, scan_cncnResultStatus_e status,
                                       scan_frameInfo_t* frameInfo, UINT16 SPSStatus );

#endif /* __SCANCNCNAPPAPI__ */
