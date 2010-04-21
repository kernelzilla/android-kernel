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

/** \file public_event_mbox.h
 *  \brief Public header for the Event Mailbox FW<->Driver interface.
 *
 */

#ifndef PUBLIC_EVENT_MBOX_H
#define PUBLIC_EVENT_MBOX_H

/******************************************************************************

    EVENT MBOX
     
    The event mechanism is based on a pair of event buffers (buffers "A" and "B") in fixed locations
    in the device's memory. The host processes one buffer (buffer "A") while the other buffer 
    (buffer "B") continues to collect events. When the host is finished, it begins processing the 
    other buffer ("B") while the first buffer ("A") collects, and so on.
    If the host is not processing events, an interrupt is issued to the host signaling that a 
    buffer is ready. The interrupt that the host receives indicates the appropriate event structure
    buffer. Once the host finishes processing events from one buffer, 
    it signals with an acknowledge interrupt (bit 0 in the INT_TRIG register) that the event buffer
    is free. This interrupt triggers the device to send the next event structure if there are any 
    collected events in it.

    Note: Only one instance (the last) of each type of event is collected.
       
******************************************************************************/


#include "public_types.h"
#include "public_commands.h"



/*************************************************************************

  Events Enumeration 

**************************************************************************/
typedef enum 
{
    RESERVED1_EVENT_ID                       = BIT_0,
    RESERVED2_EVENT_ID                       = BIT_1,
    MEASUREMENT_START_EVENT_ID               = BIT_2,
    SCAN_COMPLETE_EVENT_ID                   = BIT_3,
    CALIBRATION_COMPLETE_EVENT_ID            = BIT_4,
    ROAMING_TRIGGER_LOW_RSSI_EVENT_ID        = BIT_5,
    PS_REPORT_EVENT_ID                       = BIT_6,
    SYNCHRONIZATION_TIMEOUT_EVENT_ID         = BIT_7,
    HEALTH_REPORT_EVENT_ID                   = BIT_8,
    ACI_DETECTION_EVENT_ID                   = BIT_9,
    DEBUG_REPORT_EVENT_ID                    = BIT_10,
    MAC_STATUS_EVENT_ID                      = BIT_11,
    DISCONNECT_EVENT_COMPLETE_ID             = BIT_12,
    JOIN_EVENT_COMPLETE_ID                   = BIT_13,
    CHANNEL_SWITCH_COMPLETE_EVENT_ID         = BIT_14,
    BSS_LOSE_EVENT_ID                        = BIT_15,
    ROAMING_TRIGGER_MAX_TX_RETRY_EVENT_ID    = BIT_16,
    MEASUREMENT_COMPLETE_EVENT_ID            = BIT_17,
    AP_DISCOVERY_COMPLETE_EVENT_ID           = BIT_18,
    SCHEDULED_SCAN_COMPLETE_EVENT_ID         = BIT_19,
    PSPOLL_DELIVERY_FAILURE_EVENT_ID 	     = BIT_20,
    RESET_BSS_EVENT_ID                       = BIT_21, 
    REGAINED_BSS_EVENT_ID                    = BIT_22,
    ROAMING_TRIGGER_REGAINED_RSSI_EVENT_ID   = BIT_23,
    ROAMING_TRIGGER_LOW_SNR_EVENT_ID         = BIT_24,
    ROAMING_TRIGGER_REGAINED_SNR_EVENT_ID    = BIT_25,

    DBG_EVENT_ID                             = BIT_26,
    SOFT_GEMINI_SENSE_EVENT_ID               = BIT_27,
    SOFT_GEMINI_PREDICTION_EVENT_ID          = BIT_28,
    SOFT_GEMINI_AVALANCHE_EVENT_ID           = BIT_29,

    PLT_RX_CALIBRATION_COMPLETE_EVENT_ID     = BIT_30,

    EVENT_MBOX_ALL_EVENT_ID                  = MAX_POSITIVE32
} EventMBoxId_e;

/*************************************************************************

  Specific Event Parameters 

**************************************************************************/
typedef enum
{
    SCHEDULED_SCAN_COMPLETED_OK = 0,
    SCHEDULED_SCAN_TSF_ERROR    = 1
} ScheduledScanReportStatus_enum;


typedef enum
{
    CHANNEL_SWITCH_COMPLETE_OK,
    CHANNEL_SWITCH_TSF_ERROR
} ChannelSwitchReportStatus_enum;


typedef enum
{
    ENTER_POWER_SAVE_FAIL    =  0,
    ENTER_POWER_SAVE_SUCCESS =  1,
    EXIT_POWER_SAVE_FAIL     =  2,
    EXIT_POWER_SAVE_SUCCESS  =  3,
    POWER_SAVE_STATUS_NUMBER
} EventsPowerSave_enum;

typedef enum
{
    TEST1_DBG_EVENT_ID = 0,
    TEST2_DBG_EVENT_ID = 0x11,
    LAST_DBG_EVENT_ID= 0xff
}dbgEventId_enum;

#ifdef HOST_COMPILE
typedef uint8 ScheduledScanReportStatus_e;
typedef uint8 ChannelSwitchReportStatus_e;
typedef uint8 EventsPowerSave_e;
typedef uint8 dbgEventId_e;
#else
typedef ScheduledScanReportStatus_enum ScheduledScanReportStatus_e;
typedef ChannelSwitchReportStatus_enum ChannelSwitchReportStatus_e;
typedef EventsPowerSave_enum EventsPowerSave_e;
typedef dbgEventId_enum dbgEventId_e;
#endif


#define MAX_EVENT_REPORT_PARAMS 5
typedef struct
{ 
    dbgEventId_e dbgEventId;  /*uint8*/
    uint8       numberOfRelevantParams;
    uint16      reservedPad16;
    uint32      eventReportP1;
    uint32      eventReportP2;
    uint32      eventReportP3;
}dbgEventRep_t;



/*************************************************************************

  The Event Mailbox structure in memory 

**************************************************************************/
typedef struct EventMailBox_t
{
    /* Events Bit Mask */
    uint32 eventsVector;
    uint32 eventsMask;
    uint32 reserved1;
    uint32 reserved2;

    /* Events Data */
    int8   averageRssiLevel;                /* Average RSSI value.  */
                                            /* [ROAMING_TRIGGER_LOW_RSSI_EVENT_ID].*/

    uint8  psStatus;                        /* refer to EventsPowerSave_enum.*/
                                            /* [PS_REPORT_EVENT_ID].*/
    
    uint8  channelSwitchStatus;             /* Status of channel switch. Refer to*/
                                            /* ChannelSwitchReportStatus_enum.*/
                                            /* [CHANNEL_SWITCH_COMPLETE_EVENT_ID]*/
    
    uint8  scheduledScanStatus;             /* Status of scheduled scan. Refer to */
                                            /* ScheduledScanReportStatus_enum.*/
                                            /* [SCHEDULED_SCAN_COMPLETE_EVENT_ID]*/
    
    uint16 scheduledScanAttendedChannels;   /* Channels scanned by the Scheduled Scan. */
                                            /* [SCHEDULED_SCAN_COMPLETE_EVENT_ID]*/

    uint16 healthReport;                    /* Bit 0 is set in case of a fatal error in the device.*/
                                            /* [HEALTH_REPORT_EVENT_ID].*/
    
    uint16 badFFTCorrelationCounter;        /* [ACI_DETECTION_EVENT_ID]*/

    uint8  softGeminiSenseInfo;             /* Contains the type of the BT Coexistence sense event.*/
                                            /* [SOFT_GEMINI_SENSE_EVENT_ID]*/

    uint8  softGeminiProtectiveInfo;        /* Contains information from the BT activity prediction */
                                            /* machine [SOFT_GEMINI_PREDICTION_EVENT_ID]*/
    
    uint32              reserved;
    uint32 debugReport[2];                  /* [DBG_EVENT_ID]*/

    /* 16 Header + 24 data = 40 bytes till here include new fields */

    uint32 consFcsErrCnt;                   /* The number of FCS errors since the last event. */
                                            /* If this number is larger then the last recorded */
                                            /* number plus a threshold (20, by default), and no Rx*/
                                            /* packet has been received in the last 500 ms, than the*/
                                            /* MAC Rx module is reset. After 2 such resets (by */
                                            /* default), the driver performs a full recovery */
                                            /* process.  */
                                            /* [MAC_STATUS_EVENT_ID]*/

    dbgEventRep_t      dbgEventRep;         /* refer to dbgEventRep_t*/
                                            /* [DBG_EVENT_ID]*/

    uint8 averageSNRLevel;                  /* [ROAMING_TRIGGER_LOW_SNR_EVENT_ID]*/
    uint8              padding[19];          /* for alignment to 32 bits boundry*/
} EventMailBox_t;

#endif /* PUBLIC_EVENT_MBOX_H*/


