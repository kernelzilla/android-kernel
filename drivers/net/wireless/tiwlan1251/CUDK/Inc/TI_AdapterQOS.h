/*******************************************************************************
**+--------------------------------------------------------------------------+**
**|                                                                          |**
**| Copyright 1998-2008 Texas Instruments, Inc. - http://www.ti.com/         |**
**|                                                                          |**
**| Licensed under the Apache License, Version 2.0 (the "License");          |**
**| you may not use this file except in compliance with the License.         |**
**| You may obtain a copy of the License at                                  |**
**|                                                                          |**
**|     http://www.apache.org/licenses/LICENSE-2.0                           |**
**|                                                                          |**
**| Unless required by applicable law or agreed to in writing, software      |**
**| distributed under the License is distributed on an "AS IS" BASIS,        |**
**| WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. |**
**| See the License for the specific language governing permissions and      |**
**| limitations under the License.                                           |**
**|                                                                          |**
**+--------------------------------------------------------------------------+**
*******************************************************************************/

/*--------------------------------------------------------------------------*/
/* Module:		TI_AdapterQOS.h*/
/**/
/* Purpose:		*/
/**/
/*--------------------------------------------------------------------------*/

#ifndef TI_ADAPTER_QOS_H
#define TI_ADAPTER_QOS_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include "tiQosTypes.h"

/******************************************************************************

    Name:	TI_SetQosParameters
    Desc:	This function commands the driver to set AC (Tx queue) with 
            specific QOS parameters.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pQosParams - A pointer to the OS_802_11_QOS_PARAMS structure:
                acID - The number of AC to configure (0-3).
                MaxLifeTime - Maximum time to keep the frame in the HW (msec).
                VoiceDeliveryProtocol - The protocol used for voice exchange
                                        0 = No protocol is used.
                                        1 = use PS-Poll frames
                PSDeliveryProtocol - 0 = Legacy
                                     1 = U-APSD
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32 TI_SetQosParameters       (TI_HANDLE hAdapter, 
                                   OS_802_11_QOS_PARAMS* pQosParams );


/******************************************************************************

    Name:	TI_GetAPQosParameters
    Desc:	This function retrieves the associated AP’s QOS parameters.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pACQosParams - A Pointer to the OS_802_11_AP_QOS_PARAMS structure:
                uAC - Indicates the AC to which the function is addressed 
                        (range 0-3).
                uAssocAdmissionCtrlFlag - Indicates if the AC requires admission
                        before data transmission (0-1).
                uAIFS - Indicates the required AIFS by the AP for the specified
                        AC (2-15).
                uCwMin - Indicates the required CwMin by the AP for the specified
                        AC (0-15).
                uCwMax - Indicates the required CwMax by the AP for the specified
                        AC (0-15).
                uTXOPLimit - Indicates the required TXOPLimit by the AP for the 
                        specified AC (in units of 32 microseconds).
    Return:	TI_RESULT_OK indicates success.
            TI_RESULT_NOT_CONNECTED indicates that the STA is not connected to 
                                    any AP.
            TI_RESULT_NO_QOS_AP indicates that the associated AP does not 
                                    support QOS.
            TI_RESULT_NOK indicates invalid parameters. 
    Note:   This function should be called only after the STA was associated with 
            an infrastructure AP.
	
******************************************************************************/
tiINT32	TI_GetAPQosParameters     (TI_HANDLE hAdapter, 
                                   OS_802_11_AC_QOS_PARAMS* pACQosParams);


/******************************************************************************

    Name:	TI_GetAPQosCapabilitesParameters
    Desc:	This function retrieves the associated AP’s QOS capabilities.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pAPQosCapabiltiesParams - A Pointer to the  
                OS_802_11_AP_QOS_CAPABILITIES_PARAMS structure:
                    uQOSFlag - Flag that indicates whether the AP supports 
                                QOS (0-1)
                    uAPSDFlag - Flag that indicates whether the AP supports 
                                U-APSD (0-1)
    Return:	TI_RESULT_OK indicates success.
            TI_RESULT_NOT_CONNECTED indicates that the STA is not connected to any AP.
            TI_RESULT_NOK indicates invalid parameters.
    Note:   This function should be called only after the STA was associated with 
            an infrastructure AP.
            
******************************************************************************/
tiINT32	TI_GetAPQosCapabilitesParameters  (TI_HANDLE hAdapter, 
                                           OS_802_11_AP_QOS_CAPABILITIES_PARAMS* pAPQosCapabiltiesParams);


/******************************************************************************

    Name:	TI_PollApPackets
    Desc:	This function commands the driver to issue a polling frame to 
            retrieve downlink traffic from the AP. It should be activated by
            the voice application when there is no uplink traffic during a 
            voice call. This is used for a non-QoS association (so AC_BE
            is used).
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32 TI_PollApPackets         (TI_HANDLE hAdapter);

/******************************************************************************

    Name:	TI_PollApPacketsFromAC
    Desc:	This function commands the driver to issue a polling frame to 
            retrieve downlink traffic from the AP. It should be activated by 
            the voice application when there is no uplink traffic during a 
            voice call. The polling is either PS-Poll (for legacy PS) or 
            QoS-Null-Data (for UPSD). 
            Null-Data frame is sent after the PS-Poll to activate the 
            triggered-scan, which is only triggered by data frames. These
            packets are transmitted on the VO_AC, for QoS associations.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            AC - The AC to be polled (values may be 0 to 3)
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32	TI_PollApPacketsFromAC   (TI_HANDLE hAdapter, 
                                  tiUINT32 AC );

/******************************************************************************

    Name:	TI_SetShortRetry
    Desc:	This function commands the driver to set Best Effort AC with maximum
            number of transmits retries to perform for short packets.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uShortRetry - Maximum number of transmits retries to perform for 
                          short packets
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetShortRetry            (TI_HANDLE  hAdapter, 
                                         tiUINT32   uShortRetry  );

/******************************************************************************

    Name:	TI_GetShortRetry
    Desc:	This function gets the maximum number of transmits retries to perform 
            for short packets of the Best Effort AC.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            puShortRetry - Maximum number of transmits retries to perform for 
                           short packets.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetShortRetry            (TI_HANDLE  hAdapter, 
                                         tiUINT32*  puShortRetry );


/******************************************************************************

    Name:	TI_SetLongRetry
    Desc:	This function commands the driver to set Best Effort AC with maximum
            number of transmits retries to perform for long packets.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uLongRetry - Maximum number of transmits retries to perform for long
            packets.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetLongRetry             (TI_HANDLE  hAdapter, 
                                         tiUINT32   uLongRetry );

/******************************************************************************

    Name:	TI_GetLongRetry
    Desc:	This function gets the maximum number of transmits retries to perform 
            for long packets of the Best Effort AC.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            puLongRetry - Maximum number of transmits retries to perform for long 
            packets.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetLongRetry             (TI_HANDLE  hAdapter, 
                                         tiUINT32*  puLongRetry);

/******************************************************************************

    Name:	TI_SetQosRxTimeOut
    Desc:	
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pRxTimeOut - 
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32 TI_SetQosRxTimeOut        (TI_HANDLE hAdapter, 
                                   OS_802_11_QOS_RX_TIMEOUT_PARAMS*  pRxTimeOut);



/******************************************************************************

    Name:	TI_AddTspec
    Desc:	This function commands the driver to add TSPEC for the specific 
            user Priority.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pTspecParams - Pointer to OS_802_11_QOS_TSPEC_PARAMS strcuture:
            uUserPriority - Indicates the User priority for which TSPEC is 
                            requested (range 0-7)
            uNominalMSDUsize - Indicates the Nominal MSDU size in units of 
                            bytes (16 LSB only).
            uMeanDataRate - Indicates the average data rate in units of bits 
                            per sec
            uMinimumPHYRate - Indicates the desired minimum PHY rate in units 
                            of bits per sec
            uSurplusBandwidth - 
            hAllowance - Indicates the excess allocation of time (and bandwidth)
                            over and above the stated application rates 
                            (represented as fix Q13 numberand used 16 LSB only).
            uUPSDFlag - Indicates if the AC shouls support U-APSD (0-1)
            uMediumTime - Indicates the amount of time admitted to this UP. 
                            Used only for Tspec response notification (16 LSB 
                            only).
            uReasonCode - Indicates the reason code of AP response (applicable 
                            only in TSEPC response event, this field is zero 
                            in ADD_TSPEC request).
    Return:	TI_RESULT_OK - Success. Any other value indicates an error.
            TI_RESULT_TRAFIC_ADM_PENDING – Driver is still waiting for a response of previous request.
            TI_RESULT_AC_ALREADY_IN_USE – Means that other user priority from the same AC has already been used.
            TI_RESULT_NOT_CONNECTED -Indicates that the STA is not connected to any AP.
            TI_RESULT_NO_QOS_AP - indicates that the associated AP does not support QOS.
            TI_RESULT_ADM_CTRL_DISABLE – Indicates that station configured to not support admission control.
            TI_RESULT_NOK – Parameters are not valid.
    Notes:  1. Only one TSPEC per AC can be used. For example, user can’t 
                request TSPEC for UP 6 and 7 since those two UPs are mapped 
                to the same AC.
            2. User can issue Tspec request only after he gets a response of 
                the previous request.
	
******************************************************************************/
tiINT32	TI_AddTspec               (TI_HANDLE hAdapter, 
                                   OS_802_11_QOS_TSPEC_PARAMS* pTspecParams);

/******************************************************************************

    Name:	TI_GetTspecParameters
    Desc:	This function retrieves Tspec parameters for a specific user 
            Priority.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pTspecParams - Includes TSPEC parameters as indicated in 
                            TI_AddTspec().
    Return:	TI_RESULT_OK – Driver retrieves the TSPEC parameters.
            TI_RESULT_NOT_CONNECTED - Indicates that the STA is not connected
                                      to any AP.
            TI_RESULT_NO_QOS_AP - indicates that the associated AP does not 
                                  support QOS.
            TI_RESULT_TRAFIC_ADM_PENDING – Driver is waiting for a TSPEC response.
            TI_RESULT_NOK – Parameters are not valid.
    Note:   This function should be called only after TSPEC Response event 
            with valid reason code.	
******************************************************************************/
tiINT32	TI_GetTspecParameters     (TI_HANDLE hAdapter, 
                                   OS_802_11_QOS_TSPEC_PARAMS* pTspecParams);

/******************************************************************************

    Name:	TI_DeleteTspec
    Desc:	This function commands the driver to remove TSPEC request for the 
            specific user Priority.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    pDelTspecParams - A Pointer to the  OS_802_11_QOS_DELETE_TSPEC_PARAMS structure:
            uUserPriority - Indicates the User priority of which TSPEC to delete
                            (range 0-7)
            uReasonCode - Indicates the reason code for delete TSPEC request 
                            (32-39,45).
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32	TI_DeleteTspec            (TI_HANDLE hAdapter, 
                                   OS_802_11_QOS_DELETE_TSPEC_PARAMS* pDelTspecParams);

/******************************************************************************

    Name:	TI_GetCurrentACStatus
    Desc:	This function retrieves the status of the AC in terms of U-APSD
            activation and admission granted.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pAcStatusParams - Pointer to OS_802_11_QOS_AC_UPSD_STATUS_PARAMS:
                uAC - Indicates the AC to which the function is addressed 
                        (range 0-3)
                uCurrentUAPSDStatus - Current UAPSD status.
                uCurrentAdmissionStatus - TRUE if the selected AC can be used
                        for traffic transmission, i.e. either admission is not
                        required by the AP for this AC, or admission is required
                        and a successful TSPEC took place. In all other cases the
                        contents of this field should be FALSE.
    Return:	TI_RESULT_OK - Success.
            TI_RESULT_NOT_CONNECTED - Indicates that the STA is not connected to
                        any AP.
            TI_RESULT_NO_QOS_AP - indicates that the associated AP does not 
                        support QOS.
            TI_RESULT_NOK – Parameters are not valid.
	
******************************************************************************/
tiINT32	TI_GetCurrentACStatus     (TI_HANDLE hAdapter, 
                                   OS_802_11_AC_UPSD_STATUS_PARAMS *pAcStatusParams);

/******************************************************************************

    Name:	TI_SetMediumUsageThreshold
    Desc:	This function set the Medium Usage low and high threshold for a 
            specific AC.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pThresholdCrossParams - OS_802_11_THRESHOLD_CROSS_PARAMS:
                uAC - Indicates the AC to which the configuration is addressed 
                        (range 0-3)
                uHighThreshold - Indicates high threshold in percentage over 
                        the allowed medium usage (1-100).
                uLowThreshold - Indicates low threshold in percentage below the
                        allowed medium usage (1-100).
    Return:	TI_RESULT_OK – Driver set the thresholds.
            TI_RESULT_NOK – Parameters are not valid.
	
******************************************************************************/
tiINT32	TI_SetMediumUsageThreshold (TI_HANDLE hAdapter, 
                                    OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams);

/******************************************************************************

    Name:	TI_GetMediumUsageThreshold
    Desc:	This function retrieves the Medium Usage low and high threshold for 
            a specific AC.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pThresholdCrossParams - Pointer that holds the MediumUsage current 
                                    thresholds (see TI_SetMediumUsageThreshold).
    Return:	TI_RESULT_OK – Driver retrieved the thresholds.
            TI_RESULT_NOK – Parameters are not valid.
                                    
******************************************************************************/
tiINT32	TI_GetMediumUsageThreshold (TI_HANDLE hAdapter, 
                                    OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams);


/******************************************************************************

    Name:	TI_SetPhyRateThreshold
    Desc:	This function set the Phy Rate low and high thresholds for a 
            specific AC.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pThresholdCrossParams - OS_802_11_THRESHOLD_CROSS_PARAMS:
                uAC - Indicates the AC to which the configuration is addressed
                        (range 0-3)
                uHighThreshold - Indicates high Phy Rate threshold in Mbs 
                        (1,2,5,6,9,11,12,18,24,36,48,54).
                uLowThreshold - Indicates low Phy Rate threshold in Mbs
                        (1,2,5,6,9,11,12,18,24,36,48,54)
    Return:	TI_RESULT_OK – Driver set the thresholds.
            TI_RESULT_NOK – Parameters are not valid.
                        
******************************************************************************/
tiINT32	TI_SetPhyRateThreshold (TI_HANDLE hAdapter, 
                                OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams);


/******************************************************************************

    Name:	TI_GetPhyRateThreshold
    Desc:	This function retrieves the Phy Rate thresholds for a specific AC.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pThresholdCrossParams - Pointer that holds the PhyRate current 
                                    thresholds.
    Return:	TI_RESULT_OK – Driver retrieved the thresholds.
            TI_RESULT_NOK – Parameters are not valid.
                                    
******************************************************************************/
tiINT32	TI_GetPhyRateThreshold (TI_HANDLE hAdapter, 
                                OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams);

/******************************************************************************

    Name:	TI_GetDesiredPsMode
    Desc:	This function retrieves the desired Power Save mode per AC. The PS 
            mode can be either PS-Poll Legacy or UPSD.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            desiredPsMode - Pointer that holds the desired PS mode.
    Return:	TI_RESULT_OK – Driver retrieved the thresholds.
            TI_RESULT_NOK – Parameters are not valid.
            
******************************************************************************/
tiINT32 TI_GetDesiredPsMode   (TI_HANDLE hAdapter, 
                               OS_802_11_QOS_DESIRED_PS_MODE *desiredPsMode );

/******************************************************************************

    Name:	TI_ConfigTxClassifier
    Desc:	This function sends the configuration buffer to the OsSend package.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            inParamsBuffLen - Configuration buffer length.
            inParamsBuff - Configuration buffer pointer. 
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32 TI_ConfigTxClassifier             (TI_HANDLE hAdapter, 
                                           tiUINT32 inParamsBuffLen,
                                           tiUINT8  *inParamsBuff);

/******************************************************************************

    Name:	TI_RemoveClassifierEntry
    Desc:	This function removes a classifier entry.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pClsfrEntry - 
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32	TI_RemoveClassifierEntry          (TI_HANDLE hAdapter, 
                                           clsfr_tableEntry_t *pClsfrEntry);

/******************************************************************************

    Name:	TI_GetClsfrType
    Desc:	This function retrieves the classifier type.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            currClsfrType - pointer to clsfrTypeAndSupport struct:
                ClsfrType: one of
                    D_TAG_CLSFR = 0,
                    DSCP_CLSFR =1,
                    PORT_CLSFR =2,
                    IPPORT_CLSFR =3,
                oldVersionSupport:
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32 TI_GetClsfrType                   (TI_HANDLE hAdapter, 
                                           clsfrTypeAndSupport *currClsfrType );

/******************************************************************************

    Name:	TI_SetTrafficIntensityThresholds
    Desc:	This function sets the traffic intensity high and low thresholds.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pTrafficThresholds - A pointer to the 
                    OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS structure values:
                        uHighThreshold, uLowThreshold, TestInterval;
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32		TI_SetTrafficIntensityThresholds (TI_HANDLE  hAdapter, 
                OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS* pTrafficThresholds);

/******************************************************************************

    Name:   TI_GetTrafficIntensityThresholds	
    Desc:	This function retrieves the traffic intensity high and low thresholds.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pTrafficThresholds - A pointer to the 
                    OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS structure values.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32	    TI_GetTrafficIntensityThresholds (TI_HANDLE  hAdapter, 
                OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS* pTrafficThresholds);

/******************************************************************************

    Name:	TI_ToggleTrafficIntensityEvents
    Desc:	This function toggles between the traffic intensity operation.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            NewStatus - Enable or disable value (TRUE or FALSE)
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32	    TI_ToggleTrafficIntensityEvents  (TI_HANDLE  hAdapter, 
                                              tiUINT32 NewStatus );

tiINT32     TI_SetDTagToAcMappingTable(TI_HANDLE  hAdapter, acTrfcType_e* pDtagToAcTable );
tiINT32     TI_SetVAD(TI_HANDLE  hAdapter, txDataVadTimerParams_t* pVadTimer );
tiINT32     TI_GetVAD(TI_HANDLE  hAdapter, txDataVadTimerParams_t* pVadTimer );

#ifdef __cplusplus
}
#endif

#endif /* TI_ADAPTER_QOS_H*/
