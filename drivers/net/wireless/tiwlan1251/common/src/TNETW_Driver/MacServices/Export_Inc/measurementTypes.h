/** \file measurementTypes.h
 *  \brief This file include data types definitions for the measurment SRV module.
 *  \author Ronen Kalish
 *  \date 08-November-2005
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

#ifndef __MEASUREMENT_TYPES_H__
#define __MEASUREMENT_TYPES_H__

#include "osTIType.h"
#include "ratesTypes.h"
#include "osDot11.h"


/*
 ***********************************************************************
 *  Constant definitions.
 ***********************************************************************
 */

#define NOISE_HISTOGRAM_LENGTH              8
#define MAX_NUM_OF_MSR_TYPES_IN_PARALLEL    3

/* The size of the time frame in which we must start the */
/* measurement request or give up */
#define MSR_START_MAX_DELAY                 50

/* In non unicast measurement requests a random delay */
/* between 4 and 40 milliseconds */
#define MSR_ACTIVATION_DELAY_RANDOM         36
#define MSR_ACTIVATION_DELAY_OFFSET         4



 /*
 ***********************************************************************
 *  Enums.
 ***********************************************************************
 */

/** \enum measurement_type_e
 * \brief enumerates the different measurement types
 */
typedef enum 
{
    MSR_TYPE_BASIC_MEASUREMENT              = 0,
    MSR_TYPE_CCA_LOAD_MEASUREMENT           = 1,
    MSR_TYPE_NOISE_HISTOGRAM_MEASUREMENT    = 2,
    MSR_TYPE_BEACON_MEASUREMENT             = 3,
    MSR_TYPE_FRAME_MEASUREMENT              = 4,
    MSR_TYPE_MAX_NUM_OF_MEASURE_TYPES       = 5
} measurement_type_e;


/** \enum measurement_scanMode_e
 * \brief enumerates the different scan modes available for beacon measurement
 */
typedef enum
{
    MSR_SCAN_MODE_PASSIVE               = 0,
    MSR_SCAN_MODE_ACTIVE                = 1,
    MSR_SCAN_MODE_BEACON_TABLE          = 2,
    MSR_SCAN_MODE_MAX_NUM_OF_SCAN_MODES = 3,
} measurement_scanMode_e;


typedef enum 
{
    MSR_FRAME_TYPE_NO_ACTIVE                = 0,
    MSR_FRAME_TYPE_BROADCAST                = 1,
    MSR_FRAME_TYPE_MULTICAST                = 2,
    MSR_FRAME_TYPE_UNICAST                  = 3
} measurement_frameType_e;


typedef enum
{
    MSR_MODE_NONE                = 0,
    MSR_MODE_EXC                 = 1,
    MSR_MODE_SPECTRUM_MANAGEMENT = 2,
} measurement_mode_e;

typedef enum 
{
    MSR_REJECT_OTHER_REASON = 1, 
    MSR_REJECT_INVALID_MEASUREMENT_TYPE,
    MSR_REJECT_DTIM_OVERLAP,
    MSR_REJECT_DURATION_EXCEED_MAX_DURATION,
    MSR_REJECT_TRAFFIC_INTENSITY_TOO_HIGH,
    MSR_REJECT_SCR_UNAVAILABLE,
    MSR_REJECT_MAX_DELAY_PASSED,
    MSR_REJECT_INVALID_CHANNEL,
    MSR_REJECT_NOISE_HIST_FAIL,
    MSR_REJECT_CHANNEL_LOAD_FAIL,
    MSR_REJECT_EMPTY_REPORT,
} measurement_rejectReason_e;

 /*
 ***********************************************************************
 *  Unions.
 ***********************************************************************
 */

/** \union measurement_replyValue_u
 * \brief enumerates the different measurement types
 */
typedef union
{
    UINT8                                   CCABusyFraction;
    UINT8                                   RPIDensity[ NOISE_HISTOGRAM_LENGTH ];   
} measurement_replyValue_u;

/***********************************************************************
 *  Structure definitions.
 ***********************************************************************
 */

/** \struct scan_normalChannelEntry_t
 * \brief This structure contains single channel parameters for normal scan operation (inc. triggered)
 */
typedef struct
{
    measurement_type_e                      msrType;
    measurement_scanMode_e                  scanMode;
    UINT32                                  duration;
    UINT8                                   reserved;
} measurement_typeRequest_t;

typedef struct
{
    radioBand_e                             band;
    UINT8                                   channel;
    UINT64                                  startTime;
    UINT8                                   txPowerDbm;
    UINT8                                   numberOfTypes;
    measurement_typeRequest_t               msrTypes[ MAX_NUM_OF_MSR_TYPES_IN_PARALLEL ];
} measurement_request_t;


typedef struct
{
    measurement_type_e                      msrType;
    UINT8                                   status;
    measurement_replyValue_u                replyValue;
    UINT8                                   reserved;
} measurement_typeReply_t;

typedef struct 
{
    UINT8                                   numberOfTypes;
    measurement_typeReply_t                 msrTypes[ MAX_NUM_OF_MSR_TYPES_IN_PARALLEL ];
} measurement_reply_t;

PACKED_STRUCT( measurement_frameHdr_t ,

    UINT16                                  dialogToken;
    UINT8                                   activatioDelay;
    UINT8                                   measurementOffset;
);

PACKED_STRUCT( measurement_frameRequest_t ,

    measurement_frameHdr_t                  *hdr; 
    measurement_frameType_e                 frameType;
    UINT8                                   *requests;
    INT32                                   requestsLen;
);



#endif /* __MEASUREMENT_TYPES_H__ */

