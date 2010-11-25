/** \file ScanCncn.h
 *  \brief This file include private definitions for the scan concentrator module.
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

#ifndef __SCANCNCN_H__
#define __SCANCNCN_H__

#include "osApi.h"
#include "MacServices_api.h" 
#include "ScanCncnApi.h"
#include "fsm.h"
#include "ScanCncnDrvSM.h"



/*
 ***********************************************************************
 *	Constant definitions.
 ***********************************************************************
 */

/* allocation vector */
#define SCAN_ALLOC_OBJECT           0
#define SCAN_ALLOC_APP_SM           1
#define SCAN_ALLOC_DRV_SM           2
#define SCAN_ALLOC_CONT_SM          3
#define SCAN_ALLOC_IMMED_SM         4
#define SCAN_ALLOC_OID_SM			5  

/* Used in parameter: bAbortOrStop in scanConcentrator_t. */
#define SCAN_CNCN_ABORT         FALSE
#define SCAN_CNCN_STOP          TRUE


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

/** \struct scanConcentrator_t
 * \brief This structure contains the scan concentrator object data
 */
typedef struct
{
    /* handles to other modules */
    TI_HANDLE               hOS;                    /**< OS object handle */
	TI_HANDLE				hHalCtrl;				/**< Hal Ctrl object handle */
    TI_HANDLE               hReport;                /**< report object handle */
    TI_HANDLE               hRegulatoryDomain;      /**< regulatory domain object handle */
    TI_HANDLE               hSiteManager;           /**< site manager object handle */
    TI_HANDLE               hSCR;                   /**< SCR object handle */
    TI_HANDLE               hMacServices;           /**< Mac SRV object handle */
    TI_HANDLE               hAPConn;                /**< AP connection object handle */
    TI_HANDLE               hEventSrv;              /**< Event SRV object handle */
    TI_HANDLE               hMlme;                  /**< MLME object handle */
    TI_HANDLE               hCtrlData;              /**< Data CTRL object handle (for retrieving current BSSID) */
	TI_HANDLE				hHealthMonitor;			/**< Health Monitor , Originally at scanSrv */

    /* Scan complete callbacks */ 
    scan_resultCB_t         scanResultCB[ SCAN_SCC_NUM_OF_CLIENTS ];          /**< 
                                                                               * scan complete callback function 
                                                                               * pointers, for various clients
                                                                               */
    TI_HANDLE               scanResultCBObj[ SCAN_SCC_NUM_OF_CLIENTS ];       /**< 
                                                                               * scan complete callback objects,
                                                                               * for various clients
                                                                               */
    scan_cncnResultStatus_e scanResult[ SCAN_SCC_NUM_OF_CLIENTS ];            /**< 
                                                                               * clients' scan results, used to
                                                                               * keep information when abort or
                                                                               * stop or requested or a FW reset
                                                                               * occurs
                                                                               */
    UINT16                  SPSScanResult;                                    /**< 
                                                                               * Indicated which channels were
                                                                               * actually scanned (valid only
                                                                               * for SPS scan */

    /* state machines */
    fsm_stateMachine_t*     clientSM[ SCAN_SCC_NUM_OF_CLIENTS ];              /**< 
                                                                               * state machines for different
                                                                               * scan types
                                                                               */
    UINT8                   clientSMState[ SCAN_SCC_NUM_OF_CLIENTS ];         /**< 
                                                                               * state machine current states 
                                                                               * for different scan types
                                                                               */

    /* Scan requests */
    scan_CncnClient_e       currentRunningScanClient;                         /**< 
                                                                               * the current client for which
                                                                               * a scan is in progress
                                                                               */
    scan_Params_t           clientScanParams[ SCAN_SCC_NUM_OF_CLIENTS ];      /**<
                                                                               * scan parameters copy for 
                                                                               * different clients
                                                                               */
    scan_Type_e             drvScanRequestType;                               /**<
                                                                               * the scan type requested by the
                                                                               * driver scan. Saved if a passive
                                                                               * scan is done before the requested
                                                                               * scan type
                                                                               */
    ssid_t                  drvScanSsid;                                      /**<
                                                                               * the desired SSID requested by the 
                                                                               * driver scan. Saved to allow replacing
                                                                               * original with broadcast for passive
                                                                               * scan, to find country IE in any beacon
                                                                               */
    UINT32                  drvScanMinDwellTime[ SCAN_MAX_NUM_OF_NORMAL_CHANNELS_PER_COMMAND ];
                                                                              /**<
                                                                               * The min dwell time for all channels
                                                                               * requested by driver scan. Saved if a
                                                                               * passive scan is performed before the
                                                                               *requested scan type.
                                                                               */
    UINT32                  drvScanMaxDwellTime[ SCAN_MAX_NUM_OF_NORMAL_CHANNELS_PER_COMMAND ];
                                                                              /**<
                                                                               * The max dwell time for all channels
                                                                               * requested by driver scan. Saved if a
                                                                               * passive scan is performed before the
                                                                               *requested scan type.
                                                                               */
    BOOLEAN                 bAbortOrStop;                                     /**<
                                                                               * FLASE if the cancel reason is
                                                                               * abort (from SCR), TRUE if stop
                                                                               * (from requesting client)
                                                                               */
    BOOLEAN                 bInRequest;                                       /**<
                                                                               * Indicates whether the SM is run 
                                                                               * within the scan request context 
                                                                               * (if so, to avoid re-entrance, 
                                                                               * the complete function shouldn't
                                                                               * be called on failure, but rather 
                                                                               * an invalid status should be returned)
                                                                               */
    BOOLEAN					bUseSGParams;									  /**<
																			   * Indicates whether to use the new params
																			   * of the soft gemini module. This parameter
																			   * is TRUE when SG is enabled and FALSE otherwise
																			   */
	UINT32					SGcompensationPercent;		 					  /**<
																			   * the percentage of increasing the dwell time
																			   * on each channel when SG is enabled (that is 
																			   * bUseSGParams == TRUE)
																			   */
	UINT32					SGcompensationMaxTime;							  /**<
																			   * maximum time in microsecond from which we 
																			   * won't increase the dwelling time on the 
																			   * channel when SG is enabled
																			   */
	UINT8					SGnumOfProbeRequest;							  /**<
																			   * number of Probes to send
																			   * when SG is enabled
																			   */	
		
    /* connection status */
    connectionStatus_e      connectionStatus;   /**< the current connection status - connected / not connected / IBSS */
    scanConcentratorInitParams_t    initParams; /**< 
                                                 * parameters read from the registry (min and max dwell time
                                                 * for driver passive scan
                                                 */
/* OID scan SM */
	fsm_stateMachine_t*		hOidSM;				/**< Handle to the OID scan SM - patch for WinCE support */
	UINT8				    oidSMState;			/**< current state for the OID scan SM - patch for WinCE support */
	scan_Params_t			oidScanParams;		/**< Storage spcae for OID scan parameters */
	BOOLEAN					bOidScanRunning;	/**< Inidcates whether an OID scan is currently taking place */
	UINT32					oidScanLastTimeStamp; /**< The last time at which an OID scan was performed */
} scanConcentrator_t;

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
 * \date 05-Jan-2005\n
 * \brief Frees the scan concentrator memory, according to the init vector.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param initVec - a vector holding on bits for allocated components and off bits for non allocated components
 */
void scanConcentrator_freeMem( TI_HANDLE hScanCncn, UINT16 initVec );

/**
 * \author Ronen Kalish\n
 * \date 09-Jan-2005\n
 * \brief Verifies that specified channels are allowed for the specified scan type, and removes those that are not.\n
 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param pScanParams - a pointer to the structure holding the scan params (inc. channels and scan type)
 */
void scanConcentrator_verifyChannelsWithRegDomain( TI_HANDLE hScanCncn, scan_Params_t* pScanParams );

/**
 * \author Ronen Kalish\n
 * \date 09-Jan-2005\n
 * \brief Verifies if a certain channel is allowed according to the given bitmap.\n
 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param pScanParams - a pointer to the structure holding the scan params (inc. channels and scan type)
 * \return TRUE if the channel is allowed, FALSE otherwise.\n
 */
BOOLEAN scanConcentrator_isChannelAllowed( UINT8 channel, UINT8* pChannelBitmap );

/**
 * \author Ronen Kalish\n
 * \date 10-Jan-2005\n
 * \brief Extracts channel indexes from the bitmap to the list.
 *
 * Function Scope \e Private.\n
 * \param band - the band at which the scan is to be performed (to limit max channel number)
 * \param pChannelBitmap - the channel bitmap (input).\n
 * \param pChannelBitmap - the channel list (output).\n
 * \param pNumOfChannels - the number of channels (input - size of channel list, output - actual number of channels).\n
 */
void scanConcentrator_ExtractBitmapToList( radioBand_e band, UINT8* pChannelBitmap,
                                           UINT8* pChannelList, UINT8* pNumOfChannels );

#endif /* __SCANCNCN_H__ */
