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

/** \file scanMngrTypes.h
 *  \brief This file include public type definitions for the scan manager application level module,
 *  \brief to be included both by driver and user-mode.
 *  \author Ronen Kalish
 *  \date 01-Mar-2005
 */

#ifndef __SCAN_MNGR_TYPES_API_H__
#define __SCAN_MNGR_TYPES_API_H__

#include "scanTypes.h"

/*
 ***********************************************************************
 *	Constant definitions.
 ***********************************************************************
 */

/* AMximal number of channels per band policy */
#define MAX_BAND_POLICY_CHANNLES    30

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

/** \struct scan_probReqParams_t
 * \brief This structure contains all information needed for probe request transmission.
 */
typedef struct
{
	UINT8						numOfProbeReqs;                     /**< number of probe requests to send */
	UINT8 						txPowerDbm;                            /**< TX power at which to transmit */
	rateMask_e					bitrate;                            /**< Bit rate at which to transmit */
} scan_probReqParams_t;

/** \struct scan_basicMethodParams_t
 * \brief This structure contains parameters for basic scan operation
 */
typedef struct
{
	UINT32						maxChannelDwellTime;                /**< Maximum time to spend on each channel */
	UINT32						minChannelDwellTime;                /**< 
                                                                     * Minimum time to spend on each channel
                                                                     * (if no activity at all was discovered
                                                                     */
	scan_ETCondition_e			earlyTerminationEvent;              /**< The cause for early termination */
	UINT8						ETMaxNumberOfApFrames;              /**< 
                                                                     * Number of frames from the above type after which
                                                                     * scan is early terminated.
                                                                     */
	scan_probReqParams_t       	probReqParams;                      /**< Parameters for probe request transmission */
} scan_basicMethodParams_t;

/** \struct scan_TidTriggeredMethodParams_t
 * \brief This structure contains parameters for Tid-Triggered scan operation
 */
typedef struct 
{
	scan_basicMethodParams_t	basicMethodParams;					/**< Basic scan parameters */
	UINT8						triggeringTid;						/**< 
																	 * Tid triggering the basic scan, 
																	 * one channel at a time.
																	 */
} scan_TidTriggeredMethodParams_t;

/** \struct scan_SPSMethodParams_t
 * \brief This structure contains parameters for SPS scan operation
 */
typedef struct 
{
	scan_ETCondition_e			earlyTerminationEvent;				/**< The cause for early termination */
	UINT8				      	ETMaxNumberOfApFrames;              /**< 
                                                                     * Number of frames from the above type after which
                                                                     * scan is early terminated.
                                                                     */
	UINT32					   	scanDuration;                       /**< Time to spend on each channel (in usec) */
} scan_SPSMethodParams_t;

/** \struct scan_Method_t
 * \brief This structure contains scan type, and accompanying parameters
 */
typedef struct 
{
	scan_Type_e					scanType;                           /**<
                                                                     * scan type (normal - active or passive),
                                                                     * Tid-Triggered (active or passive), or SPS
                                                                     */
	union
	{
		scan_basicMethodParams_t 		    basicMethodParams;      /**< scan parameters for normal scan */
        scan_TidTriggeredMethodParams_t		TidTriggerdMethodParams; /**< scan parameters for Tid-Triggered scan */
        scan_SPSMethodParams_t				spsMethodParams;        /**< scan parameters for SPS scan */
	} method;
} scan_Method_t;

/** \struct scan_bandPolicy_t
 * \brief This structure contains parameters comprising a scan policy for a single band
 */
typedef struct
{
	radioBand_e 				band;	                            /**< the band (2.4 / 5 GHz) */
	scan_Method_t               discoveryMethod;                    /**< scan method used to discover new BSS'es */
	scan_Method_t	    		trackingMethod;                     /**<
                                                                     * scan method used to track previously 
                                                                     * discovered AP's
                                                                     */
	scan_Method_t		    	immediateScanMethod;                /**< scan method used for immediate scan */
	INT8            			rxRSSIThreshold;                    /**< quality threshold for received frames */
	UINT8					    numOfChannlesForDiscovery;          /**<
                                                                     * number of channels to scan at each discovery
                                                                     * attempt
                                                                     */
	UINT8					    numOfChannles;                      /**< number of channels to use on this band */
	UINT8	           			channelList[ MAX_BAND_POLICY_CHANNLES ];
                                                                    /**< all possible channels */
} scan_bandPolicy_t;

/** \struct scan_Policy_t
 * \brief This structure contains parameters comprising scan policies on all bands
 */
typedef struct
{
    UINT32  				normalScanInterval;                     /**<
                                                                     * time interval (im msec) at which to perform
                                                                     * continuous scan, when current BSS quality is
                                                                     * considered "normal".
                                                                     */
    UINT32	    			deterioratingScanInterval;              /**<
                                                                     * time interval (in msec) at which to perform
                                                                     * continuous scab, when current BSS quality is
                                                                     * considered "deteriorating"
                                                                     */
    UINT8					maxTrackFailures;                       /**<
                                                                     * the max number of track failures to keep
                                                                     * an AP in the BSS list
                                                                     */
    UINT8					BSSListSize;                            /**< the number of APs in the BSS list */
    UINT8					BSSNumberToStartDiscovery;              /**<
                                                                     * the number of APs in the BSS list at which
                                                                     * discovery process is initiated
                                                                     */
    UINT8 		    		numOfBands;                             /**< number of bands to scan */
    scan_bandPolicy_t		bandScanPolicy[ RADIO_BAND_NUM_OF_BANDS ];/**< bands' policies */
} scan_Policy_t;

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


#endif /* __SCAN_MNGR_TYPES_API_H__ */

