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

#ifndef _SCAN_TYPES_H
#define _SCAN_TYPES_H

#include "osTIType.h"
#include "ratesTypes.h"
#include "osDot11.h"
#include "commonTypes.h"

/*****************************************************************************************
					      Scan Definitions
						  ---------------
This file is included by the whalCtrl_api.h , it should not be included apart. !!!!!!!
*****************************************************************************************/


/*
 ***********************************************************************
 *	Constant definitions.
 ***********************************************************************
 */
#define MAX_NUMBER_OF_CHANNELS_PER_SCAN  					16
#define SCAN_MAX_NUM_OF_NORMAL_CHANNELS_PER_COMMAND         MAX_NUMBER_OF_CHANNELS_PER_SCAN
#define SCAN_MAX_NUM_OF_SPS_CHANNELS_PER_COMMAND            16
#define SCAN_DEFAULT_MIN_CHANNEL_DWELL_TIME                 30000
#define SCAN_DEFAULT_MAX_CHANNEL_DWELL_TIME                 60000
#define SCAN_DEFAULT_EARLY_TERMINATION_EVENT                SCAN_ET_COND_DISABLE
#define SCAN_DEFAULT_EARLY_TERMINATION_NUM_OF_FRAMES        0

 /*
 ***********************************************************************
 *	Enums.
 ***********************************************************************
 */


/** \enum scan_Type_e
 * \brief enumerates the different scan types
 */
typedef enum
{
    SCAN_TYPE_NORMAL_PASSIVE = 0,   /**< normal passive scan */
    SCAN_TYPE_NORMAL_ACTIVE,        /**< normal active scan */
    SCAN_TYPE_SPS,                  /**< scheduled passive scan */
    SCAN_TYPE_TRIGGERED_PASSIVE,    /**< triggered passive scan */
    SCAN_TYPE_TRIGGERED_ACTIVE,     /**< triggered active scan */
    SCAN_TYPE_NO_SCAN               /**< no scan to perform */
} scan_Type_e;

/** \enum scan_ETCondition_e
 * \brief enumerates the different early termination conditions
 */
typedef enum
{
    SCAN_ET_COND_DISABLE     = 0x00,        /**< no early termination */
    SCAN_ET_COND_BEACON      = 0x10,        /**< early termination on beacons */
    SCAN_ET_COND_PROBE_RESP  = 0x20,        /**< early termination on probe responses */
    SCAN_ET_COND_ANY_FRAME   = 0x30,        /**< early termination on beacons and probe responses */
    SCAN_ET_COND_NUM_OF_CONDS= 0x4          /**< number of early termination conditions */ 
} scan_ETCondition_e;

/***********************************************************************
 *	Structure definitions.
 ***********************************************************************
 */

/** \struct scan_normalChannelEntry_t
 * \brief This structure contains single channel parameters for normal scan operation (inc. triggered)
 */
typedef struct
{
    macAddress_t        bssId;                      /**< BSS Id to filter with */
    UINT32              maxChannelDwellTime;        /**< 
                                                     * maximum time to dwell on the channel, in microseconds 
                                                     * (if something was received)
                                                     */
    UINT32              minChannelDwellTime;        /**< 
                                                     * minimum time to dwell on the channel, in microseconds
                                                     * (if nothing was received)
                                                     */
    scan_ETCondition_e  earlyTerminationEvent;      /**< the event triggering early termination */
    UINT8               ETMaxNumOfAPframes;         /**< 
                                                     * the number of frames to receive 
                                                     * to activate  early termination
                                                     */
    
    UINT8               txPowerDbm	;               /**< 
                                                      * the tx power to use for probe requests (for active scan)
                                                      */
    UINT8               channel;                    /**< the channel to scan */
} scan_normalChannelEntry_t;

/** \struct scan_SPSChannelEntry_t
 * \brief This structure contains single channel parameters for an SPS scan operation
 */
typedef struct
{
    macAddress_t         bssId;                      /**< BSS Id to filter with */
    UINT32               scanDuration;               /**< time to dwell on the channel in microseconds */
    UINT32               scanStartTime;              /**< exact time to start scan in 4 lower bytes of the TSF */
    scan_ETCondition_e   earlyTerminationEvent;      /**< the event triggering early termination */
    UINT8                ETMaxNumOfAPframes;         /**< 
                                                      * the number of frames to receive 
                                                      * to activate  early termination
                                                      */
    UINT8                channel;                    /**< the channel to scan */
} scan_SPSChannelEntry_t;

/** \union scan_channelEntry_u
 * \brief This union holds single channel parameters either for normal scan or for SPS scan
 */
typedef union
{
    scan_normalChannelEntry_t   normalChannelEntry;     /**< normal scan parameters */
    scan_SPSChannelEntry_t      SPSChannelEntry;        /**< SPS scan parameters */
} scan_channelEntry_u;

/** \struct scan_Params_t
 * \brief This structure contains parameters for a scan operation
 */
typedef struct
{
    ssid_t              desiredSsid;            /**< the SSID to search (optional) */
    scan_Type_e         scanType;               /**< 
                                                 * scan type (normal - active or passive, 
                                                 * SPS, triggered - active or passive)
                                                 */
    radioBand_e         band;                   /**< band to scan (A / BG) */
    UINT8               probeReqNumber;         /**< number of probe requests to send (for active scan) */
    rateMask_e          probeRequestRate;       /**< the rate at which to send the probe requests */
    UINT8               Tid;                    /**< the Tid triggering the scan (for triggered scan) */
    UINT64              latestTSFValue;         /**< 
                                                 * for SPS scan - the latest TSF at which a frame was 
                                                 * received. Used to detect TSF error (AP recovery).
                                                 */
    UINT32              SPSScanDuration;        /**<
                                                 * for SPS scan ONLY - the time duration of the scan (in
                                                 * milliseconds), used to set timer according to.
                                                 */
    UINT8               numOfChannels;          /**< number of channels */

	scan_channelEntry_u	channelEntry[ MAX_NUMBER_OF_CHANNELS_PER_SCAN ];      /**< channels array */
} scan_Params_t;

#endif

