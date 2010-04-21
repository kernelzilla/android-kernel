/** \file roamingMngrApi.h
 *  \brief Internal Roaming Manager API
 *
 *  \see roamingMngr.c
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
 *   MODULE:  Roaming Manager                                               *
 *   PURPOSE: Roaming Manager Module API                              	    *
 *                                                                          *
 ****************************************************************************/

#ifndef _ROAMING_MNGR_API_H_
#define _ROAMING_MNGR_API_H_

/*#include "802_11Defs.h"*/
#include "osApi.h"
#include "paramOut.h"
#include "scanMngrApi.h"
#include "bssTypes.h"

/* Constants */

/* Enumerations */


/* Typedefs */

/* Structures */

/* External data definitions */

/* External functions definitions */

/* Function prototypes */

/* Creates the Roaming Manager Module */
TI_HANDLE roamingMngr_create(TI_HANDLE hOs);

/* Configures Roaming Manager Module */
TI_STATUS roamingMngr_init(TI_HANDLE hRoamingMngr,
					  TI_HANDLE hReport,
					  TI_HANDLE hScanMngr,
					  TI_HANDLE hAPConnection);
/* Unloads the Roaming Manager Module */
TI_STATUS roamingMngr_unload(TI_HANDLE hRoamingMngr);
/* IF for getting Roaming Manager Module parameters */
TI_STATUS roamingMngr_getParam(TI_HANDLE hRoamingMngr, paramInfo_t *pParam);
/* IF for setting Roaming Manager Module parameters */
TI_STATUS roamingMngr_setParam(TI_HANDLE hRoamingMngr, paramInfo_t *pParam);


#endif /*  _ROAMING_MNGR_API_H_*/

