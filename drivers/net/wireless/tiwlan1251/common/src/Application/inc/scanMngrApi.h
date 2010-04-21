/***************************************************************************
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
/** \file scanMngrApi.h
 *  \brief This file include public definitions for the scan manager module, comprising its API.
 *  \author Ronen Kalish
 *  \date 01-Mar-2005
 */

#ifndef __SCAN_MNGR_API_H__
#define __SCAN_MNGR_API_H__

#include "scanMngrTypes.h"
#include "bssTypes.h"
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

/** \enum scan_mngrResultStatus_e 
 * \brief enumerates the different result statuses that the scan manager
 * \brief can return in response to an immediate scan request.
 */
typedef enum
{
	SCAN_MRS_SCAN_COMPLETE_OK= 0,                           /**< Scan was completed successfully */
	SCAN_MRS_SCAN_RUNNING,                                  /**< Scan was started successfully and is now running */ 
	SCAN_MRS_SCAN_NOT_ATTEMPTED_ALREADY_RUNNING,            /**< scan was not attempted because it is already running */
    SCAN_MRS_SCAN_NOT_ATTEMPTED_EMPTY_POLICY,               /**< 
                                                             * Scan was not attempted because the policy defines
                                                             * NULL scan type
                                                             */
	SCAN_MRS_SCAN_NOT_ATTEMPTED_NO_CHANNLES_AVAILABLE,      /**< 
                                                             * Scan was not attempted because no channels are 
                                                             * available for scan, according to the defined policy.
                                                             */
	SCAN_MRS_SCAN_FAILED,                                   /**< Scan failed to start */
	SCAN_MRS_SCAN_STOPPED,                                  /**< Scan was stopped by caller */
	SCAN_MRS_SCAN_ABORTED_FW_RESET,                         /**< Scan was aborted due to recovery */
	SCAN_MRS_SCAN_ABORTED_HIGHER_PRIORITY,                  /**< Scan was aborted due to a higher priority client */
} scan_mngrResultStatus_e;

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
 * \date 01-Mar-2005\n
 * \brief Creates the scan manager object.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the OS object.\n
 * \return a pointer to the scan manager object if successful, NULL otherwise.\n
 */
TI_HANDLE scanMngr_create( TI_HANDLE hOS );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief Initializes the scan manager.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param hReport - handle to the report object.\n
 * \param hRegDomain - handle to the regulatory domain object.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param hRoamMngr - handle to the roaming manager object.\n
 * \param hSiteMngr - handle to the site manager object.\n
 */
void scanMngr_init( TI_HANDLE hScanMngr, TI_HANDLE hReport, TI_HANDLE hRegDomain, 
                    TI_HANDLE hScanCncn, TI_HANDLE hRoamMngr, TI_HANDLE hSiteMngr,
					TI_HANDLE hHalCtrl);

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief unloads the scan manager object.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 */
void scanMngr_unload( TI_HANDLE hScanMngr );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief Starts an immediate scan operation.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param pNeighborAPsOnly - TRUE if scan for neighbor APs only, FALSE if scan on all channels.\n
 * \return status indication.\n
 * \retval SCAN_MRS_SCAN_RUNNING - the scan started successfully and is now running.\n
 * \retval SCAN_MRS_SCAN_NOT_ATTEMPTED_ALREADY_RUNNING - scan was not attempted because it is already running.\n
 * \retval SCAN_MRS_SCAN_NOT_ATTEMPTED_EMPTY_POLICY - scan was not attempted because NULL policy defined.\n
 * \retval SCAN_MRS_SCAN_NOT_ATTEMPTED_NO_CHANNLES_AVAILABLE - scan was not attempted because no channels were available.\n
 * \retval SCAN_MRS_SCAN_FAILED - scan failed to start at lower levels.\n
 */
scan_mngrResultStatus_e scanMngr_startImmediateScan( TI_HANDLE hScanMngr, BOOLEAN bNeighborAPsOnly);

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief Stops an immediate scan operation.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 */
void scanMngr_stopImmediateScan( TI_HANDLE hScanMngr );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief Starts the continuous scan timer.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param currentBSS - the AP we are currently connected to.\n
 * \currentBSSBand - the band of the current BSS.\n
 */
void scanMngr_startContScan( TI_HANDLE hScanMngr, macAddress_t* currentBSS, radioBand_e currentBSSBand );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief Stops the continuous scan timer.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 */
void scanMngr_stopContScan( TI_HANDLE hScanMngr );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief returns the currently available BSS list.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \return BSS list structure pointer.\n
 */
bssList_t *scanMngr_getBSSList( TI_HANDLE hScanMngr );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief Sets the neighbor APs.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param neighborAPList - the neighbor AP list.\n
 */
void scanMngr_setNeighborAPs( TI_HANDLE hScanMngr, neighborAPList_t* neighborAPList );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief change quality level (normal / deteriorating).\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param bLowQuality - TRUE if quality is deteriorating, FALSE if quality is normal.\n
 */
void scanMngr_qualityChangeTrigger( TI_HANDLE hScanMngr, BOOLEAN bLowQuality );

/**
 * \author Ronen Kalish\n
 * \date 13-Mar-2005\n
 * \brief Notifies the scan manager of a roaming event. Should be called BEFORE the SCR group 
 * \brief is changed back to connected.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param macAddress - mac address of the new AP.\n
 * \param band - the band of the new AP.\n
 */
void scanMngr_handoverDone( TI_HANDLE hScanMngr, macAddress_t* macAddress, radioBand_e band );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief Callback used by the scan concentrator for immediate scan result.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param resultStatus - reason for calling this function (frame received / scan complete).\n
 * \param frameInfo - frame related information (in case of a frame reception).\n
 * \param SPSStatus - bitmap indicating which channels were scan, in case of an SPS scan.\n
 */
void scanMngr_immedScanCB( TI_HANDLE hScanMngr, scan_cncnResultStatus_e resultStatus, 
                           scan_frameInfo_t* frameInfo, UINT16 SPSStatus );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief Callback used by the scan concentrator for continuous scan result.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param resultStatus - reason for calling this function (frame received / scan complete).\n
 * \param frameInfo - frame related info (in case of a frame reception).\n
 * \param SPSStatus - bitmap indicating which channels were scan, in case of an SPS scan.\n
 */
void scanMngr_contScanCB( TI_HANDLE hScanMngr, scan_cncnResultStatus_e resultStatus, 
                         scan_frameInfo_t* frameInfo, UINT16 SPSStatus );

/**
 * \author Ronen Kalish\n
 * \date 08-Mar-2005\n
 * \brief Parses and executes a get param command.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param pParam - the param to get.\n
 * \return OK if the parameter was get successfully, NOK otherwise.\n
 */
TI_STATUS scanMngr_getParam( TI_HANDLE hScanMngr, paramInfo_t *pParam );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief Parses and executes a set param command.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param pParam - the param to set.\n
 * \return OK if the parameter was set successfully, NOK otherwise.\n
 */
TI_STATUS scanMngr_setParam( TI_HANDLE hScanMngr, paramInfo_t *pParam );

/**
 * \author Ronen Kalish\n
 * \date 01-Mar-2005\n
 * \brief Sets the scan policy.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 * \param scanPolicy - a pointer to the policy data.\n
 */
void scanMngr_setScanPolicy( TI_HANDLE hScanMngr, scan_Policy_t* scanPolicy );

#ifdef TI_DBG
/**
 * \author Ronen Kalish\n
 * \date 09-Mar-2005\n
 * \brief Print scan policy.\n
 *
 * Function Scope \e Private.\n
 * \param scanPolicy - scan policy to print.\n
 */
void scanMngrTracePrintScanPolicy( scan_Policy_t* scanPolicy );

/**
 * \author Ronen Kalish\n
 * \date 26-May-2005\n
 * \brief Print scan manager statistics.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 */
void scanMngr_statsPrint( TI_HANDLE hScanMngr );

/**
 * \author Ronen Kalish\n
 * \date 26-May-2005\n
 * \brief Reset scan manager statistics.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 */
void scanMngr_statsReset( TI_HANDLE hScanMngr );

/**
 * \author Ronen Kalish\n
 * \date 25-July-2005\n
 * \brief Print Neighbor AP list.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - Handle to the scan manager object.\n
 */
void scanMngrDebugPrintNeighborAPList( TI_HANDLE hScanMngr );

/**
 * \author Ronen Kalish\n
 * \date 25-July-2005\n
 * \brief Prints all data in the scan manager object.\n
 *
 * Function Scope \e Public.\n
 * \param hScanMngr - handle to the scan manager object.\n
 */
void scanMngrDebugPrintObject( TI_HANDLE hScanMngr );

#endif /* TI_DBG */

#endif /* __SCAN_MNGR_API_H__ */
