/** \file ScanCncnOidSM.c
 *  \brief This file include the scan concentrator OID request SM module implementation
 *  \author Ronen Kalish
 *  \date 11-May-2006
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

#include "ScanCncnOidSM.h"
#include "ScanCncn.h"
#include "report.h"
#include "osApi.h"
#include "fsm.h"
#include "siteMgrApi.h"
#include "regulatoryDomainApi.h"

static TI_STATUS actionUnexpected( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 14-May-2006\n
 * \brief Initialize the scan concentrator OID scan SM.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorOidSM_init( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    fsm_actionCell_t    smMatrix[ OID_SCAN_NUM_OF_STATES ][ OID_SCAN_NUM_OF_EVENTS ] =
	{
		/* next state and actions for IDLE state */
		{	
            {OID_SCAN_STATE_SCAN_ON_G, scanConcentratorOidSM_actionStartGScan},         //"START_SCAN",
			{OID_SCAN_STATE_IDLE, actionUnexpected},                                    //"SCAN_COMPLETE",  
			{OID_SCAN_STATE_IDLE, actionUnexpected},                                    //"SCAN_FAILED",
  		},

		/* next state and actions for SCAN_ON_G state */
		{	
            {OID_SCAN_STATE_SCAN_ON_G, actionUnexpected},                               //"START_SCAN",
			{OID_SCAN_STATE_SCAN_ON_A, scanConcentratorOidSM_actionStartAScan},         //"SCAN_COMPLETE",  
			{OID_SCAN_STATE_SCAN_ON_A, scanConcentratorOidSM_actionStartAScan},         //"SCAN_FAILED",
		},

		/* next state and actions for SCAN_ON_A state */
		{	
            {OID_SCAN_STATE_SCAN_ON_A, actionUnexpected},                               //"START_SCAN",
			{OID_SCAN_STATE_IDLE, scanConcentratorOidSM_actionCleanup},                 //"SCAN_COMPLETE",  
			{OID_SCAN_STATE_IDLE, scanConcentratorOidSM_actionCleanup},					//"SCAN_FAILED",
		}
    };

    /* initialize current state */
    pScanConcentrator->oidSMState = OID_SCAN_STATE_IDLE;
	pScanConcentrator->bOidScanRunning = FALSE;
	pScanConcentrator->oidScanLastTimeStamp = 0;

    /* configure the state machine */
	return fsm_Config( pScanConcentrator->hOidSM, (fsm_Matrix_t)smMatrix, 
                       OID_SCAN_NUM_OF_STATES, OID_SCAN_NUM_OF_EVENTS, 
                       (fsm_eventActivation_t)scanConcentratorOidSM_SMEvent, pScanConcentrator->hOS );
}

#ifdef REPORT_LOG
/* state descriptions, for state machine logging */
static char stateDesc[ OID_SCAN_NUM_OF_STATES ][ MAX_DESC_STRING_LEN ] =
{
    "STATE_IDLE",
    "STATE_SCAN_ON_G",
    "STATE_SCAN_ON_A"
};

/* event descriptions, for state machine logging */
static char eventDesc[ OID_SCAN_NUM_OF_EVENTS ][ MAX_DESC_STRING_LEN ] =
{
    "EVENT_START_SCAN",
    "EVENT_SCAN_COMPLETE",
    "EVENT_SCAN_FAILED"
};
#endif

/**
 * \author Ronen Kalish\n
 * \date 11-May-2006\n
 * \brief Processes an event.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param currentState - the current OID request SM state.\n
 * \param event - the event to handle.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorOidSM_SMEvent( TI_HANDLE hScanCncn, scan_oidSMStates_e* currentState, 
                                         scan_oidSMEvents_e event )
{
    scanConcentrator_t *pScanConcentrator = (scanConcentrator_t *)hScanCncn;
	TI_STATUS status = OK;
	UINT8 nextState;

    /* obtain the next state */
	status = fsm_GetNextState( pScanConcentrator->hOidSM, (UINT8)*currentState, 
                               (UINT8)event, &nextState );
	if ( status != OK )
	{
		WLAN_REPORT_SM( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, ("Failed getting OID scan next state.\n") );
		return NOK;
	}

    /* report the move */
    WLAN_REPORT_SM( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, 
                    ("OID SCAN: <%s, %s> --> %s\n\n",
                    stateDesc[(UINT8)*currentState],
                    eventDesc[(UINT8)event],
                    stateDesc[nextState]) );

    /* move */
    return fsm_Event( pScanConcentrator->hOidSM, (UINT8*)currentState, (UINT8)event, hScanCncn );
}

/**
 * \author Ronen Kalish\n
 * \date 11-May-2006\n
 * \brief SM action - starts a scan on G band
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorOidSM_actionStartGScan( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
	paramInfo_t         param;
	int					validChannelsCount;
	BOOL				regulatoryDomainEnabled;

	/* if the STA is not configured for G band or dual band, send a scan complete event to the SM */
	param.paramType = SITE_MGR_DESIRED_DOT11_MODE_PARAM;
	siteMgr_getParam( pScanConcentrator->hSiteManager, &param );
	if ( (DOT11_G_MODE != param.content.siteMgrDot11Mode) && (DOT11_DUAL_MODE != param.content.siteMgrDot11Mode) )
	{
		WLAN_REPORT_INFORMATION( pScanConcentrator->hOS, SCAN_CNCN_MODULE_LOG,
								 ("Scan OID SM: STA does not work on 2.4 GHz, continuing to 5.0 GHz scan\n") );
		scanConcentratorOidSM_SMEvent( hScanCncn, (scan_oidSMStates_e*)&(pScanConcentrator->oidSMState), OID_SCAN_EVENT_SCAN_FAILED );
		return OK;
	}

	/* build scan command header */
	pScanConcentrator->oidScanParams.band = RADIO_BAND_2_4_GHZ;
	pScanConcentrator->oidScanParams.desiredSsid.len = 0;
	pScanConcentrator->oidScanParams.desiredSsid.ssidString[ 0 ] = '\0'; /* broadcast scan */
	pScanConcentrator->oidScanParams.Tid = 0;

	/* query the regulatory domain if 802.11d is in use */
	param.paramType = REGULATORY_DOMAIN_ENABLED_PARAM;
	regulatoryDomain_getParam( pScanConcentrator->hRegulatoryDomain,&param );
	regulatoryDomainEnabled = param.content.regulatoryDomainEnabled;

	/* Get country code status */
	param.paramType			 = REGULATORY_DOMAIN_IS_COUNTRY_FOUND;
	param.content.eRadioBand = RADIO_BAND_2_4_GHZ;
	regulatoryDomain_getParam(pScanConcentrator->hRegulatoryDomain,&param);

	/* scan type is passive if 802.11d is enabled and country IE was not yet found, active otherwise */
	if ( (TRUE == regulatoryDomainEnabled) && 
		 (FALSE == param.content.bIsCountryFound) )
	{
		pScanConcentrator->oidScanParams.scanType = SCAN_TYPE_NORMAL_PASSIVE;
	}
	else
	{
		pScanConcentrator->oidScanParams.scanType = SCAN_TYPE_NORMAL_ACTIVE;
		/* also set number and rate of probe requests */
		pScanConcentrator->oidScanParams.probeReqNumber = SCAN_OID_DEFAULT_PROBE_REQUEST_NUMBER_G;
		pScanConcentrator->oidScanParams.probeRequestRate = SCAN_OID_DEFAULT_PROBE_REQUEST_RATE_G;
	}

	/* add supported channels on G */
	if ( SCAN_TYPE_NORMAL_PASSIVE == pScanConcentrator->oidScanParams.scanType )
	{
		validChannelsCount = scanConcentratorOidSM_FillAllAvailableChannels( hScanCncn, RADIO_BAND_2_4_GHZ, SCAN_TYPE_NORMAL_PASSIVE,
                                       &(pScanConcentrator->oidScanParams.channelEntry[0]),
																			 SCAN_OID_DEFAULT_MAX_DWELL_TIME_PASSIVE_G,
																			 SCAN_OID_DEFAULT_MIN_DWELL_TIME_PASSIVE_G,
																			 SCAN_OID_DEFAULT_EARLY_TERMINATION_EVENT_PASSIVE_G,
																			 SCAN_OID_DEFAULT_EARLY_TERMINATION_COUNT_PASSIVE_G );
	}
	else
	{
		validChannelsCount = scanConcentratorOidSM_FillAllAvailableChannels( hScanCncn, RADIO_BAND_2_4_GHZ, SCAN_TYPE_NORMAL_ACTIVE,
                                       &(pScanConcentrator->oidScanParams.channelEntry[0]),
 																			 SCAN_OID_DEFAULT_MAX_DWELL_TIME_ACTIVE_G,
																			 SCAN_OID_DEFAULT_MIN_DWELL_TIME_ACTIVE_G,
																			 SCAN_OID_DEFAULT_EARLY_TERMINATION_EVENT_ACTIVE_G,
																			 SCAN_OID_DEFAULT_EARLY_TERMINATION_COUNT_ACTIVE_G );
	}
	pScanConcentrator->oidScanParams.numOfChannels = validChannelsCount;

	/* check that some channels are available */
	if ( validChannelsCount > 0 )
	{
		scan_cncnResultStatus_e res;
		
		/*
		 * Prepare scan complete's aging, by increasing the scanned sites 
		 * scan attemps counter. The counter will be checked upon scan complete,  
		 * and the sites with no update scan results will be dropped.   
		 */
		siteMgr_setNotReceivedParameter( pScanConcentrator->hSiteManager, &(pScanConcentrator->oidScanParams.desiredSsid), 
		                                 RADIO_BAND_2_4_GHZ );

		/* send command to scan concentrator APP SM */
		res = scanConcentrator_scan(hScanCncn, SCAN_SCC_APP, &(pScanConcentrator->oidScanParams) );

		/* if scan failed, send scan failed event to the SM */
		if ( SCAN_CRS_SCAN_RUNNING != res )
		{
			scanConcentratorOidSM_SMEvent( hScanCncn, (scan_oidSMStates_e*)&(pScanConcentrator->oidSMState), OID_SCAN_EVENT_SCAN_FAILED );
		}
	}
	else
	{
		/* no channels to scan, send a scan failed event */
		scanConcentratorOidSM_SMEvent( hScanCncn, (scan_oidSMStates_e*)&(pScanConcentrator->oidSMState), OID_SCAN_EVENT_SCAN_FAILED );
	}

	return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 11-May-2006\n
 * \brief SM action - starts a scan on A band
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorOidSM_actionStartAScan( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
	paramInfo_t         param;
	int					validChannelsCount;
	BOOL				regulatoryDomainEnabled;

	/* if the STA is not configured for G band or dual band, send a scan complete event to the SM */
	param.paramType = SITE_MGR_DESIRED_DOT11_MODE_PARAM;
	siteMgr_getParam( pScanConcentrator->hSiteManager, &param );
	if ( (DOT11_A_MODE != param.content.siteMgrDot11Mode) && (DOT11_DUAL_MODE != param.content.siteMgrDot11Mode) )
	{
		WLAN_REPORT_INFORMATION( pScanConcentrator->hOS, SCAN_CNCN_MODULE_LOG,
								 ("Scan OID SM: STA does not work on 5.0 GHz, quitting\n") );
		scanConcentratorOidSM_SMEvent( hScanCncn, (scan_oidSMStates_e*)&(pScanConcentrator->oidSMState), OID_SCAN_EVENT_SCAN_FAILED );
		return OK;
	}

	/* build scan command header */
	pScanConcentrator->oidScanParams.band = RADIO_BAND_5_0_GHZ;
	pScanConcentrator->oidScanParams.desiredSsid.len = 0;
	pScanConcentrator->oidScanParams.desiredSsid.ssidString[ 0 ] = '\0'; /* broadcast scan */
	pScanConcentrator->oidScanParams.Tid = 0;

	/* query the regulatory domain if 802.11d is in use */
	param.paramType = REGULATORY_DOMAIN_ENABLED_PARAM;
	regulatoryDomain_getParam( pScanConcentrator->hRegulatoryDomain,&param );
	regulatoryDomainEnabled = param.content.regulatoryDomainEnabled;

	/* Get country code status */
	param.paramType			 = REGULATORY_DOMAIN_IS_COUNTRY_FOUND;
	param.content.eRadioBand = RADIO_BAND_5_0_GHZ;
	regulatoryDomain_getParam(pScanConcentrator->hRegulatoryDomain,&param);

	/* scan type is passive if 802.11d is enabled and country IE was not yet found, active otherwise */
	if ( (TRUE == regulatoryDomainEnabled) && 
		 (FALSE == param.content.bIsCountryFound) )
	{
		pScanConcentrator->oidScanParams.scanType = SCAN_TYPE_NORMAL_PASSIVE;
	}
	else
	{
		pScanConcentrator->oidScanParams.scanType = SCAN_TYPE_NORMAL_ACTIVE;
		/* also set number and rate of probe requests */
		pScanConcentrator->oidScanParams.probeReqNumber = SCAN_OID_DEFAULT_PROBE_REQUEST_NUMBER_A;
		pScanConcentrator->oidScanParams.probeRequestRate = SCAN_OID_DEFAULT_PROBE_REQUEST_RATE_A;
	}

	/* add supported channels on G */
	if ( SCAN_TYPE_NORMAL_PASSIVE == pScanConcentrator->oidScanParams.scanType )
	{
		validChannelsCount = scanConcentratorOidSM_FillAllAvailableChannels( hScanCncn, RADIO_BAND_5_0_GHZ, SCAN_TYPE_NORMAL_PASSIVE,
                                       &(pScanConcentrator->oidScanParams.channelEntry[0]),
 																			 SCAN_OID_DEFAULT_MAX_DWELL_TIME_PASSIVE_A,
																			 SCAN_OID_DEFAULT_MIN_DWELL_TIME_PASSIVE_A,
																			 SCAN_OID_DEFAULT_EARLY_TERMINATION_EVENT_PASSIVE_A,
																			 SCAN_OID_DEFAULT_EARLY_TERMINATION_COUNT_PASSIVE_A );
	}
	else
	{
		validChannelsCount = scanConcentratorOidSM_FillAllAvailableChannels( hScanCncn, RADIO_BAND_5_0_GHZ, SCAN_TYPE_NORMAL_ACTIVE,
                                       &(pScanConcentrator->oidScanParams.channelEntry[0]),
                                       SCAN_OID_DEFAULT_MAX_DWELL_TIME_ACTIVE_A,
																			 SCAN_OID_DEFAULT_MIN_DWELL_TIME_ACTIVE_A,
																			 SCAN_OID_DEFAULT_EARLY_TERMINATION_EVENT_ACTIVE_A,
																			 SCAN_OID_DEFAULT_EARLY_TERMINATION_COUNT_ACTIVE_A );
	}
	pScanConcentrator->oidScanParams.numOfChannels = validChannelsCount;

	/* check that some channels are available */
	if ( validChannelsCount > 0 )
	{
		scan_cncnResultStatus_e res;

		/*
		 * Prepare scan complete's aging, by increasing the scanned sites 
		 * scan attemps counter. The counter will be checked upon scan complete,  
		 * and the sites with no update scan results will be dropped.   
		 */
		siteMgr_setNotReceivedParameter( pScanConcentrator->hSiteManager, &(pScanConcentrator->oidScanParams.desiredSsid), 
		                                 RADIO_BAND_5_0_GHZ );

      /* send command to scan concentrator APP SM */
		res = scanConcentrator_scan(hScanCncn, SCAN_SCC_APP, &(pScanConcentrator->oidScanParams) );

		/* if scan failed, send scan failed event to the SM */
		if ( SCAN_CRS_SCAN_RUNNING != res )
		{
			scanConcentratorOidSM_SMEvent( hScanCncn, (scan_oidSMStates_e*)&(pScanConcentrator->oidSMState), OID_SCAN_EVENT_SCAN_FAILED );
		}
	}
	else
	{
		/* no channels to scan, send a scan failed event */
		scanConcentratorOidSM_SMEvent( hScanCncn, (scan_oidSMStates_e*)&(pScanConcentrator->oidSMState), OID_SCAN_EVENT_SCAN_FAILED );
	}

	return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 14-May-2006\n
 * \brief SM action - Cleans up an OID scan operation
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorOidSM_actionCleanup( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* 	pScanConcentrator = (scanConcentrator_t*)hScanCncn;

	/* mark that OID scan process is no longer running */
	pScanConcentrator->bOidScanRunning = FALSE;


	return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 14-May-2006\n
 * \brief Fills a chhanel array with valid channels (and their params) according to band and scan type\n
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param band - band to extract channels for.\n
 * \param scanType - scan type tp ectract channels for.\n
 * \param channelArray - where to store allowed channels information.\n
 * \param maxDwellTime - maximum dwell time value to be used for each channel.\n
 * \param minDwellTime - minimum dwell time value to be used for each channel.\n
 * \param ETCondition - early termination condition value to be used for each channel.\n
 * \param ETFrameNumber - early termination frame number value to be used for each channel.\n
 * \return Number of allowed channels (that were placed in the given channels array).\n
 */
UINT32 scanConcentratorOidSM_FillAllAvailableChannels( TI_HANDLE hScanCncn, radioBand_e band, scan_Type_e scanType,
													   scan_channelEntry_u* channelArray, UINT32 maxDwellTime,
													   UINT32 minChannelTime, scan_ETCondition_e ETCondition,
													   UINT8 ETFrameNumber )
{
    scanConcentrator_t* 	pScanConcentrator = (scanConcentrator_t*)hScanCncn;
	int 					i, j, allowedChannelsCount, validChannelsCnt = 0;
	paramInfo_t 			param;
	UINT8      				tempChannelList[ SCAN_MAX_NUM_OF_NORMAL_CHANNELS_PER_COMMAND ];
 
	/* get the numnber of supported channels for this band */
	param.paramType = REGULATORY_DOMAIN_ALL_SUPPORTED_CHANNELS;
	param.content.siteMgrRadioBand = band;
	regulatoryDomain_getParam( pScanConcentrator->hRegulatoryDomain, &param );
	allowedChannelsCount = param.content.supportedChannels.sizeOfList;

	/* for the time being don't scan more channels than fit in one command */
	if ( allowedChannelsCount > SCAN_MAX_NUM_OF_NORMAL_CHANNELS_PER_COMMAND )
	{
		allowedChannelsCount = SCAN_MAX_NUM_OF_NORMAL_CHANNELS_PER_COMMAND;
	}

 	/* Copy allowed channels to reuse param var */
	os_memoryCopy( pScanConcentrator->hOS, tempChannelList,
			param.content.supportedChannels.listOfChannels, allowedChannelsCount );

	/* preapre the param var to request channel allowance for the requested scan type */
	param.paramType = REGULATORY_DOMAIN_GET_SCAN_CAPABILITIES;
	param.content.channelCapabilityReq.band = band;

	/* add default values to channels allowed for the requested scan type and band */
	for ( i = 0; i < allowedChannelsCount; i++ )
	{
		/* get specific channel allowance for scan type */
		if ( (scanType == SCAN_TYPE_NORMAL_PASSIVE) ||
			 (scanType == SCAN_TYPE_TRIGGERED_PASSIVE) ||
			 (scanType == SCAN_TYPE_SPS) )
		{
			param.content.channelCapabilityReq.scanOption = PASSIVE_SCANNING;
		}
		else
		{
			param.content.channelCapabilityReq.scanOption = ACTIVE_SCANNING;
		}
		param.content.channelCapabilityReq.channelNum = tempChannelList[ i ];

		regulatoryDomain_getParam( pScanConcentrator->hRegulatoryDomain, &param );
		if ( TRUE == param.content.channelCapabilityRet.channelValidity )
		{
			/* add the channel ID */
			channelArray[ validChannelsCnt ].normalChannelEntry.channel = tempChannelList[ i ];

			/* add other default parameters */
			channelArray[ validChannelsCnt ].normalChannelEntry.minChannelDwellTime = minChannelTime;
			channelArray[ validChannelsCnt ].normalChannelEntry.maxChannelDwellTime = maxDwellTime;
			channelArray[ validChannelsCnt ].normalChannelEntry.earlyTerminationEvent = ETCondition;
		    channelArray[ validChannelsCnt ].normalChannelEntry.ETMaxNumOfAPframes = ETFrameNumber;
			channelArray[ validChannelsCnt ].normalChannelEntry.txPowerDbm  = 
				param.content.channelCapabilityRet.maxTxPowerDbm;
			/* Fill broadcast BSSID */
			for ( j = 0; j < 6; j++ )
			{
				channelArray[ validChannelsCnt ].normalChannelEntry.bssId.addr[ j ] = 0xff;
			}
			validChannelsCnt++;
		}
	}

	/* return the number of channels that are actually allowed for the requested scan type on the requested band */
	return validChannelsCnt;
}

/**
 * \author Ronen Kalish\n
 * \date 11-Jan-2005\n
 * \brief Handles an unexpected event.\n

 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return always OK.\n
 */
TI_STATUS actionUnexpected( TI_HANDLE hScanCncn ) 
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_SM( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, ("OID scan state machine error, unexpected Event\n\n") );
	
    return OK;
}



