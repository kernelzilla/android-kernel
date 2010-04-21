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

/** \file bssTypes.h
 *  \brief This file include public type definitions for Driver-wide BSS information
 *  \author Ronen Kalish
 *  \date 05-April-2005
 */


#ifndef __BSS_TYPES_API_H__
#define __BSS_TYPES_API_H__

#include "osDot11.h"

/*
 ***********************************************************************
 *	Constant definitions.
 ***********************************************************************
 */
#define MAX_NUM_OF_NEIGHBOR_APS     30
#define MAX_SIZE_OF_BSS_TRACK_LIST  16
 
/** \enum resultFrameType_e 
 * \brief enumerates the different types for a result frame
 */
typedef enum
{
	SCAN_RFT_BEACON = 0,                                    /**< result frame is a beacon */
	SCAN_RFT_PROBE_RESPONSE                                 /**< result frame is a probe response */
} resultFrameType_e;

/*
 ***********************************************************************
 *	Structure definitions.
 ***********************************************************************
 */

/** \struct bssEntry_t
 * \brief This structure contains a single BSS entry
 */
typedef struct
{
/* values in beacon with fixed length */
    macAddress_t        BSSID;                  /**< BSSID of this entry */
    UINT64              lastRxTSF;              /**< TSF of last received frame */
    UINT16              beaconInterval;         /**< Beacon interval of this AP */
    UINT16              capabilities;           /**< capabilities of this AP */
/* IE's in beacon */
    UINT8               DTIMPeriod;             /**< DTIm period (in beacon interval quantas */
	resultFrameType_e	resultType;             /**< The type of frame in pBuffer */
    UINT16              bufferLength;           /**< length of rest of beacon (or probe response) buffer */
    UINT8*              pBuffer;                /**< rest of beacon (or probe response) buffer */
/* Information from other sources */
    radioBand_e         band;                   /**< band on which the AP transmits */
    UINT8               channel;                /**< channel on which the AP transmits */
    UINT8               rxRate;                 /**< Rate at which last frame was received */
    UINT32              lastRxHostTimestamp;    /**< 
                                                 * the host timestamp (in milliseconds) at which last frame 
                                                 * was received
                                                 */
    INT8                RSSI;                   /**< average RSSI */
    INT8                lastRSSI;               /** last given RSSI */
	BOOLEAN             bNeighborAP;            /**< Indicates whether this is a neighbor AP */
} bssEntry_t;

/** \struct bssList_t
 * \brief This structure contains the BSS list
 */
typedef struct
{
	UINT8 					numOfEntries;                               /**< number of entries in the list */
	bssEntry_t              BSSList[ MAX_SIZE_OF_BSS_TRACK_LIST ];      /**< list of entries */
} bssList_t;

/** \struct neighborAP_t
 * \brief This structure contains information on one neighbor AP
 */
typedef struct
{
	macAddress_t	    	BSSID;                          /**< The BSSID (MAC address) of this AP */
	UINT8				    channel;                        /**< the channel on which the AP transmits */
	radioBand_e			    band;                           /**< the band used by the AP */
} neighborAP_t;

/** \struct neighborAPList_t
 * \brief This structure contains a list of Neighbor APs
 */
typedef struct
{
	UINT8				    numOfEntries;                           /**< the number of entries in the list */
	neighborAP_t            APListPtr[ MAX_NUM_OF_NEIGHBOR_APS ];   /**< a pointer to the list of APs */
} neighborAPList_t;


#endif
