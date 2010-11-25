/** \file roamingMngr.h
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
 *   MODULE:  Roaming Manager	    		                            *
 *   PURPOSE: Roaming Manager Module API                                    *
 *                                                                          *
 ****************************************************************************/

#ifndef _ROAMING_MNGR_H_
#define _ROAMING_MNGR_H_

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


/* called by the Scan Manager when scan for Roaming completed */
TI_STATUS roamingMngr_immediateScanComplete(TI_HANDLE hRoamingMngr, scan_mngrResultStatus_e scanCmpltStatus);

/* called by the Scan Manager when new BSSID was found */
TI_STATUS roamingMngr_updateNewBssList(TI_HANDLE hRoamingMngr, bssList_t *newBss_entry);


#endif /*  _ROAMING_MNGR_H_*/

