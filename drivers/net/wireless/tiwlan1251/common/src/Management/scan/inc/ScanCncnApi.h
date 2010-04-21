/** \file ScanCncnApi.h
 *  \brief This file include public definitions for the scan concentrator module, comprising its API.
 *  \author Ronen Kalish
 *  \date 30-Dec-2004
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

#ifndef __SCANCNCNAPI_H__
#define __SCANCNCNAPI_H__

#include "osApi.h"
#include "scrApi.h"
#include "paramOut.h"
#include "mlmeApi.h"
#include "MacServices_api.h"


/** \enum connectionStatus_e
 * \brief enumerates the different connection statuses
 */
typedef enum
{
    STA_CONNECTED = 0,              /**< the station is connected to an infrastructure BSS */
    STA_NOT_CONNECTED,              /**< the station is not connected to an infrastructure BSS */
    STA_IBSS                        /**< the station is participating in an IBSS */
} connectionStatus_e;


/** \enum scan_CncnClient_e
 * \brief enumerates the different possible clients requesting scan from the scan concentrator
 */
typedef enum
{
    SCAN_SCC_ROAMING_IMMED = 0,     /**< immediate scan for roaming */
    SCAN_SCC_ROAMING_CONT,          /**< continuous scan for roaming */
    SCAN_SCC_DRIVER,                /**< driver (SME) scan */
    SCAN_SCC_APP,                   /**< application (user) scan */
    SCAN_SCC_NUM_OF_CLIENTS,        /**< number of clients */
    SCAN_SCC_NO_CLIENT              /**< no client */
} scan_CncnClient_e;

/** \enum scan_cncnResultStatus_e
 * \brief enumerates the different scan result statuses
 */
typedef enum 
{
    SCAN_CRS_RECEIVED_FRAME = 0,            /**< scan not yet completed, indicating a frame received during scan */
    SCAN_CRS_SCAN_COMPLETE_OK,              /**< scan completed normally */
    SCAN_CRS_SCAN_RUNNING,                  /**< scan initialized successfully and is now running */
    SCAN_CRS_SCAN_FAILED,                   /**< 
                                             * scan failed due to unexpected situation (SCR reject, no 
                                             * channels available, scan SRV returned NOK, etc)
                                             */
    SCAN_CRS_SCAN_STOPPED,                  /**< scan stopped by user */
    SCAN_CRS_TSF_ERROR,                     /**< TSF error (AP recovery) occurred (for SPS only */
    SCAN_CRS_SCAN_ABORTED_FW_RESET,         /**< scan aborted due to FW reset */
    SCAN_CRS_SCAN_ABORTED_HIGHER_PRIORITY,  /**< scan aborted due to a higher priority client */
    SCAN_CRS_NUM_OF_RES_STATUS              /**< number of possible result status */
} scan_cncnResultStatus_e;

/*
 ***********************************************************************
 *	External data definitions.
 ***********************************************************************
 */

/** \struct scan_frameInfo_t
 * \brief contains a single frame information, returned by the result CB when a frame is available
 */
typedef struct
{
    macAddress_t*       bssId;              /* MAC address of the AP from which the frame was received */
    mlmeFrameInfo_t*    parsedIEs;          /* parsed frame IEs */
    radioBand_e         band;               /* band on which the frame was received */
    UINT8               channel;            /* channel on which the frame was received */
    UINT32              staTSF;             /* TSF of the station when the frame was received */
    INT8                rssi;               /* RSSI level at which frame was received */
    rate_e              rate;               /* bitrate at which frame was received */
    UINT8*              buffer;             /* frame body */
    UINT16              bufferLength;       /* frame body length */
} scan_frameInfo_t;

/*
 ***********************************************************************
 *	Typedefs.
 ***********************************************************************
 */

 /** \typedef scan_resultCB_t
  * \brief Defines the function prototype for the scan result callback
  * (notification by the scan concentrator to a client of either a scan
  * termination or a result frame received).
  */
typedef void (*scan_resultCB_t) ( TI_HANDLE clientObj, scan_cncnResultStatus_e status,
                                 scan_frameInfo_t* frameInfo, UINT16 SPSStatus );

/*
 ***********************************************************************
 *	External functions definitions
 ***********************************************************************
 */

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Creates the scan concentrator object
 *
 * Function Scope \e Public.\n
 * \param hOS - handle to the OS object.\n
 * \return a handle to the scan SRV object, NULL if an error occurred.\n
 */
TI_HANDLE scanConcentrator_create( TI_HANDLE hOS );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Finalizes the scan concentrator object (freeing system resources)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 */
void scanConcentrator_release( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Initialize the scan concentrator object.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param hHalCtrl - handle to the Hal Ctrl object.\n
 * \param hReport - handle to the Report object.\n
 * \param hRegDomain - handle to the regulatory domain object.\n
 * \param hSiteMngr - handle to the site manager object.\n
 * \param hSCR - handle to the SCR object.\n
 * \param hMacServices - handle to the MacServices object.\n
 * \param hAPConn - handle to the AP connection object.\n
 * \param hEventSRV - handle to the event SRV object.\n
 * \param hMlme - handle to the MLME object.\n
 * \param hCtrlData - handle to the data CTRL object.\n
 * \param hHealthMonitor - handle to the health monitor object.\n
 * \param pScanConcentratorInitParams - pointer to the init parameters structure.\n 
 */
void scanConcentrator_init( TI_HANDLE hScanCncn,
						    TI_HANDLE hHalCtrl,
                            TI_HANDLE hReport,
                            TI_HANDLE hRegDomain,
                            TI_HANDLE hSiteMngr,
                            TI_HANDLE hSCR,
                            TI_HANDLE hMacServices,
                            TI_HANDLE hAPConn,
                            TI_HANDLE hEventSRV,
                            TI_HANDLE hMlme,
                            TI_HANDLE hCtrlData,
							TI_HANDLE hHealthMonitor,
                            scanConcentratorInitParams_t *pScanConcentratorInitParams );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief switch the connection mode to connected (infrastructure BSS)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 */
void scanConcentrator_switchToConnected( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief switch the connection mode to not connected
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 */
void scanConcentrator_switchToNotConnected( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief switch the connection mode to IBSS participation
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 */
void scanConcentrator_switchToIBSS( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by a client to request a scan.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param client - the client ID.\n
 * \return scan operation detailed result status.\n
 * \retval SCAN_CRS_SCAN_RUNNING - scan started successfully and is now running.\n
 * \retval SCAN_CRS_SCAN_FAILED - scan failed to start due to an unexpected error.\n
 */
scan_cncnResultStatus_e scanConcentrator_scan( TI_HANDLE hScanCncn, 
                                               scan_CncnClient_e client, 
                                               scan_Params_t* pScanParams );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by a client to stop a scan request in process. A client should ONLY stop its own request!
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param client - the client ID.\n
 */
void scanConcentrator_stopScan( TI_HANDLE hScanCncn, scan_CncnClient_e client );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Registers a scan result function for a specific client.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param client - the client ID.\n
 * \param scanResultCBFunc - the function to use.\n
 * \param scanresultCBObj - the object to pass to the scan result CB function.\n
 */
void scanConcentrator_registerScanResultCB( TI_HANDLE hScanCncn, scan_CncnClient_e client,
                                            scan_resultCB_t scanResultCBFunc, TI_HANDLE scanResultCBObj );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the scan SRV (after registration) to notify of a scan complete event.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param SPSStatus - which channels were attempted (if SPS scan).\n
 * \param bTSFError - whether a TSF error occurred (if SPS scan).\n
 * \param  ScanStatus - return the status of the scan . \n
 */
void scanConcentrator_scanCompleteNotificationCB( TI_HANDLE hScanCncn, UINT16 SPSStatus, BOOLEAN bTSFError , TI_STATUS ScanStatus , TI_STATUS PSMode);

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the MLME parser to pass information received on a beacon or probe response.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param bssid - a pointer to the address of the AP sending this frame.\n
 * \param frameInfo - the IE in the frame.\n
 * \param pRxAttr - a pointer to TNET RX attributes struct.\n
 * \param buffer - a pointer to the frame body.\n
 * \param byfferLength - the frame body length.\n
 */
void scanConcentrator_mlmeResultCB( TI_HANDLE hScanCncn, macAddress_t* bssid, mlmeFrameInfo_t* frameInfo, 
                                    Rx_attr_t* pRxAttr, UINT8* buffer, UINT16 bufferLength );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the SCR (after registration) to notify the scan concentrator of a status change
 * \brief for the immediate scan for roaming client.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param requestStatus - the continuous scan for roaming client status.\n
 * \param pendReason - the reason for pend status, if the status is pend.\n
 */
void scanConcentrator_scrRoamingImmedCB( TI_HANDLE hScanCncn, scr_clientRequestStatus_e requestStatus,
                                         scr_pendReason_e pendReason );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the SCR (after registration) to notify the scan concentrator of a status change
 * \brief for the continuous scan for roaming client.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param requestStatus - the continuous scan for roaming client status.\n
 * \param pendReason - the reason for pend status, if the status is pend.\n
 */
void scanConcentrator_scrRoamingContCB( TI_HANDLE hScanCncn, scr_clientRequestStatus_e requestStatus,
                                        scr_pendReason_e pendReason );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the SCR (after registration) to notify the scan concentrator of a status change
 * \brief for the Application scan client.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param requestStatus - the application scan status.\n
 * \param pendReason - the reason for pend status, if the status is pend.\n
 */
void scanConcentrator_scrAppCB( TI_HANDLE hScanCncn, scr_clientRequestStatus_e requestStatus,
                                scr_pendReason_e pendReason );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the SCR (after registration) to notify the scan concentrator of a status change
 * \brief for the driver scan client.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param requestStatus - the driver scan status.\n
 */
void scanConcentrator_scrDriverCB( TI_HANDLE hScanCncn, scr_clientRequestStatus_e requestStatus,
                                   scr_pendReason_e pendReason );

/**
 * \author Yuval Adler\n
 * \date 26-Jan-2006\n
 * \brief enable/disable the use of SG parameters , and update parameters of next scans 
 *		requests for minDwellTime,MaxDwellTime,numProbeReq  .\n
 *			this function is called when SG is enabled or disabled
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param bUseSGParams - whether to use the new parameters (TRUE when SG is enabled)
 * \param probeReqNumber - 
 * \param SGcompensationMaxTime - max value from which we won't increase dwelling time
 * \param SGcompensationPercent - increasing dwell time in that percentage
 */
void scanConcentrator_SGconfigureScanParams( TI_HANDLE hScanCncn, BOOL bUseSGParams ,
											 UINT8 probeReqNumber , UINT32 SGcompensationMaxTime, 
											 UINT32 SGcompensationPercent);
#endif /* __SCANCNCNAPI_H__ */
