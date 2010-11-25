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
/* Module:		CTI_Adapter.h*/
/**/
/* Purpose:		interface for the ITI_WLAN_Adapter class.*/
/**/
/*--------------------------------------------------------------------------*/

#if !defined(AFX_CTI_WLAN_ADAPTER_H__3B4F299B_AF52_40DE_BE67_A032DA1FB441__INCLUDED_)
#define AFX_CTI_WLAN_ADAPTER_H__3B4F299B_AF52_40DE_BE67_A032DA1FB441__INCLUDED_

#include "tiwlnif.h"

#if _MSC_VER > 1000
#pragma once
#endif /* _MSC_VER > 1000*/

#define TIWLNAPI_READ       0
#define TIWLNAPI_WRITE      1
#define TIWLNAPI_READWRITE  2

#define MAX_SSID_LENGTH     32

#ifndef _T
#define _T(x)	x
#endif




#ifdef EXC_MODULE_INCLUDED
    #define EXC_EXPANSION           CASE_EXC_SUPPORT
    #define EXC_MODULE_SUPPORT_H    ADAPTER_EXC_SUPPORT_H
#else
    #define EXC_EXPANSION
    #define EXC_MODULE_SUPPORT_H
#endif /*EXC_MODULE_INCLUDED */

#ifdef _WINDOWS
#else

class TI_IPC
{
	TI_HANDLE m_hDevice;
    public:
        TI_IPC();
       ~TI_IPC(); 
        tiINT32    IPC_DeviceIoControl  (   tiUINT32    dwIoControlCode, 
                                        tiVOID* lpInBuffer, tiUINT32 nInBufferSize,
                                        tiVOID* lpOutBuffer, tiUINT32 nOutBufferSize, 
                                        tiUINT32* lpBytesReturned);
        TI_HANDLE   IPC_DeviceOpen      ( tiCHAR* pAdapterName);
        tiVOID      IPC_DeviceClose     ();
        tiINT32     IPC_RegisterEvent   ( IPC_EVENT_PARAMS* pEventParams );
        tiINT32     IPC_UnRegisterEvent ( IPC_EVENT_PARAMS* pEventParams );
}; 
#endif /* _WINDOWS */

class TI_OSRegistry
{
    public:
        tiUINT32    PutSZ(tiCHAR* /*lpszName*/, tiCHAR* /* lpszValue*/) {return TRUE;}
        tiUINT32	PutDW(tiCHAR* /*lpszName*/, tiUINT32   /* dwValue*/)   {return TRUE;}

};

class CTI_WLAN_AdapterAPI;

struct _AdapterItem
{
        _AdapterItem()
        { 
            m_dwAdapterID   = NULL;
            m_pAdapterName  = NULL; 
            m_pNextItem     = NULL;
            m_pPrevItem     = NULL;
            m_uRef          = 1;
        }
                    
        CTI_WLAN_AdapterAPI*            m_dwAdapterID;
        tiCHAR*                         m_pAdapterName;
        tiUINT32                        m_uRef;

        _AdapterItem*                   m_pNextItem;
        _AdapterItem*                   m_pPrevItem;
       
        tiUINT32                        AddRef()    {return ++m_uRef;}
        tiUINT32                        DecRef()    {return --m_uRef;}
};

class CTI_WLAN_AdapterAPI : public TI_WLAN_AdapterAPI 
{
    protected:                                                   

        static  _AdapterItem*           m_pAdaptersList;

	                                    CTI_WLAN_AdapterAPI     ( tiCHAR* lpszAdapterName );
	    virtual                        ~CTI_WLAN_AdapterAPI     ();
                tiINT32                 PLT_RxTXCal(void* pTestCmdData, tiUINT32 Length, TestCmdID_e TestCmdID);
                tiCHAR*                 m_pszAdapterName;
                tiINT32                 PLT_RxCalStatus(TI_STATUS* pStatus);
    
    private:
#ifdef _WINDOWS
#else
                TI_IPC*                 m_pIPCmod;
#endif
#ifdef TI_EMBEDDED_SUPPLICANT
    			TI_IPC_Supplicant* m_pSupplicant;
#endif /* ifdef TI_EMBEDDED_SUPPLICANT */
                TI_OSRegistry*          m_pRegistry;
                TI_OAL*                 m_pOSLib;
#ifdef TI_EMBEDDED_SUPPLICANT
                tiBOOL                  m_bSupplicantInUse;
#endif /* ifdef TI_EMBEDDED_SUPPLICANT */

                tiINT32                 tiIoCtrl                (tiUINT32 dwIoCtrl, tiVOID* pInBuffer, tiUINT32 dwInBufferSize, 
                                                                 tiVOID* pOutBuffer = NULL, tiUINT32 dwOutBufferSize = 0, tiUINT32* dwBytesReturned = NULL);
                tiINT32                 tiMiniportSetInformation(tiUINT32 dwInfoCode,tiVOID* lpInfoBuffer, 
                                                                 tiUINT32* lpdwCbInfoBuffer);
                tiINT32                 tiMiniportQueryInformation(tiUINT32 dwInfoCode, tiVOID* lpInfoBuffer, 
                                                                 tiUINT32* lpdwCbInfoBuffer);
    public:
	    static  CTI_WLAN_AdapterAPI*    GetTIWLANAdapter        (tiCHAR* lpszAdapterName, tiBOOL bForce);
        static  tiINT32                 FreeTIWLANAdapter       ( CTI_WLAN_AdapterAPI* pAdapter, tiBOOL bForce);
                tiINT32                 CheckObjectMemory       (tiVOID* pObj, tiUINT32 uSizeObj);

        virtual tiINT32                 GetApiVersion           ( tiUINT32* pdwApiVersion );

	    virtual tiINT32                 GetBSSID                ( OS_802_11_MAC_ADDRESS* pAddrBSSID );
	    virtual tiINT32                 SetBSSID                ( OS_802_11_MAC_ADDRESS* pAddrBSSID );

        virtual tiINT32                 GetBSSIDList            ( OS_802_11_BSSID_LIST_EX** ppBSSIDlist);
        virtual tiINT32                 GetFullBSSIDList        ( OS_802_11_BSSID_LIST_EX** ppBSSIDlist);

        virtual tiINT32                 GetCurrentAddress       ( OS_802_11_MAC_ADDRESS*    pCurrentAddr);

        virtual tiINT32                 SetDesiredChannel       ( tiUINT32  uDesiredChannel  );
        virtual tiINT32                 GetDesiredChannel       ( tiUINT32* puDesiredChannel );
        virtual tiINT32                 GetCurrentChannel       ( tiUINT32* puCurrentChannel );

        virtual tiINT32                 GetDesiredRate          ( tiUINT32* puDesiredRates );
        virtual tiINT32                 GetCurrentRate          ( tiUINT32* puCurrentRates );

        virtual tiINT32                 SetFragmentThreshold    ( tiUINT32  uFragmentThreshold  );
        virtual tiINT32                 GetFragmentThreshold    ( tiUINT32* puFragmentThreshold );

        virtual tiINT32                 SetBSSType              ( OS_802_11_NETWORK_MODE  uBSSType );
        virtual tiINT32                 GetBSSType              ( OS_802_11_NETWORK_MODE* puBSSType);

        virtual tiINT32                 SetLongRetry            ( tiUINT32  uLongRetry  );
        virtual tiINT32                 GetLongRetry            ( tiUINT32* puLongRetry );

        virtual tiINT32                 SetRTSThreshold         ( tiUINT32  uRTSThreshold   );
        virtual tiINT32                 GetRTSThreshold         ( tiUINT32* puRTSThreshold  );

        virtual tiINT32                 SetShortPreamble        ( tiUINT32  uShortPreamble  );
        virtual tiINT32                 GetShortPreamble        ( tiUINT32* puShortPreamble );

        virtual tiINT32                 SetShortRetry           ( tiUINT32  uShortRetry     );
        virtual tiINT32                 GetShortRetry           ( tiUINT32* puShortRetry    );

        virtual tiINT32                 SetSSID                 ( tiUINT8*   pSSIDname );
        virtual tiINT32                 GetDesiredSSID          ( OS_802_11_SSID*   pSSID );
        virtual tiINT32                 GetCurrentSSID          ( OS_802_11_SSID*   pSSID );

        virtual tiINT32                 GetStatistics           (TIWLN_STATISTICS* tiStatistics);
        virtual tiINT32                 GetTxStatistics         (TIWLN_TX_STATISTICS* tiTxStatistics, UINT32 clearStatsFlag);
		

        virtual tiINT32                 EnableDisableRxDataFilters( tiBOOL enabled );
        virtual tiINT32                 GetRxDataFiltersStatistics( TIWLAN_DATA_FILTER_STATISTICS * pStatistics );
        virtual tiINT32                 AddRxDataFilter         ( TIWLAN_DATA_FILTER_REQUEST * pRequest );
        virtual tiINT32                 RemoveRxDataFilter      ( TIWLAN_DATA_FILTER_REQUEST * pRequest );

        virtual tiINT32                 SetSupportedRates       ( tiUINT8* pSupportedRatesLst, tiUINT32  uBufLength);
        virtual tiINT32                 GetSupportedRates       ( tiUINT8* pSupportedRatesLst, tiUINT32  uBufLength);

        virtual tiINT32                 GetDriverVersion        ( TIWLN_VERSION_EX* pdrvVersion );

        virtual tiINT32                 SetIBSSProtection       ( tiUINT32  uProtection    );
        virtual tiINT32                 GetIBSSProtection       ( tiUINT32* puProtection   );

        virtual tiINT32                 GetDriverState          ( driverState_e* puDriverState );

        virtual tiINT32                 SetShortSlot            ( tiUINT32  dwShortSlot );
        virtual tiINT32                 GetShortSlot            ( tiUINT32* pdwShortSlot);

        virtual tiINT32                 SetTxPowerDbm	        ( tiUINT8  uTxPower   );
        virtual tiINT32                 GetTxPowerLevel         ( tiCHAR* puTxPower  );
        virtual tiINT32                 GetTxPowerDbm           ( tiCHAR* puTxPower  );

        virtual tiINT32                 GetSupportedNetworkTypes( OS_802_11_NETWORK_TYPE* pNetTypeLst, tiUINT32 uMaxNetTypes    );
        virtual tiINT32                 SetNetworkTypeInUse     ( OS_802_11_NETWORK_TYPE  uNetType                              );
        virtual tiINT32                 GetNetworkTypeInUse     ( OS_802_11_NETWORK_TYPE* pdwNetType                            );  

        virtual tiINT32                 GetNumberOfAntennas     ( tiUINT32* puNumberOfAntennas    );
        virtual tiINT32                 SetAntennaDiversityParams( PTIWLAN_ANT_DIVERSITY pAntennaDiversityOptions );

        virtual tiINT32                 Start                   ( );
        virtual tiINT32                 Stop                    ( );
        virtual tiINT32                 Suspend                 ( );
        virtual tiINT32                 StartSM                 ( );
        virtual tiINT32                 StopSM                  ( );

        virtual tiINT32                 EnableDisable_802_11d   ( tiUINT8  enableDisable_802_11d );
        virtual tiINT32                 EnableDisable_802_11h   ( tiUINT8  enableDisable_802_11h );
        virtual tiINT32                 Get_802_11d             ( tiUINT8*  enableDisable_802_11d );
        virtual tiINT32                 Get_802_11h             ( tiUINT8*  enableDisable_802_11h );
        virtual tiINT32                 Set_countryIeFor2_4_Ghz ( country_t countryIe);
        virtual tiINT32                 Get_countryIeFor2_4_Ghz ( tiUINT8 **countryString);
        virtual tiINT32                 Set_countryIeFor5_Ghz    ( country_t countryIe);
        virtual tiINT32                 Get_countryIeFor5_Ghz   ( tiUINT8 **countryString);
        virtual tiINT32                 Set_minMaxDfsChannels   ( DFS_ChannelRange_t DFS_ChannelRange);
        virtual tiINT32                 Get_minMaxDfsChannels   ( DFS_ChannelRange_t *DFS_ChannelRange);

        virtual tiINT32                 GetRSSI                     ( tiINT32* pRssi );
		virtual tiINT32                 GetSNR                      ( tiUINT32* pSnr );

        virtual tiINT32                 Disassociate            ( );

        virtual tiINT32                 SetAuthenticationMode   ( OS_802_11_AUTHENTICATION_MODE  uAuthenticationMode  );                          
        virtual tiINT32                 GetAuthenticationMode   ( OS_802_11_AUTHENTICATION_MODE* puAuthenticationMode );

        virtual tiINT32                 SetEAPType              ( OS_802_11_EAP_TYPES  uEAPType );
        virtual tiINT32                 SetEAPTypeDriver        ( OS_802_11_EAP_TYPES  uEAPType );
        virtual tiINT32                 GetEAPType              ( OS_802_11_EAP_TYPES* puEAPType);

        virtual tiINT32                 SetEncryptionType       ( OS_802_11_ENCRYPTION_TYPES  uEncryptType  );
        virtual tiINT32                 GetEncryptionType       ( OS_802_11_ENCRYPTION_TYPES* puEncryptType );

        virtual tiINT32                 SetCredentials          ( tiCHAR* pszUserName, tiCHAR* pszPassword );
        
        virtual tiINT32                 SetPSK                  ( tiCHAR* pszPSK );
        virtual tiINT32                 SetKeyType              ( OS_802_11_KEY_TYPES uKeyType );
     /*   virtual tiINT32                 SetUserID               ( tiCHAR* pszUserID ); */
        virtual tiINT32                 SetMixedMode            ( tiBOOL bStatus );
        virtual tiINT32                 GetMixedMode            ( tiBOOL* pbStatus );

        virtual tiINT32                 SetCertParamsSHA1       ( TI_SHA1_HASH* pSha1Hash, tiBOOL bValidateServerCert );
        virtual tiINT32                 SetCertParamsFileName   ( tiCHAR* pszFileName,     tiBOOL bValidateServerCert );

        virtual tiINT32                 AddWEPKey               ( OS_802_11_WEP*   pWEP             );
        virtual tiINT32                 RemoveWEPKey            ( tiUINT32         uKeyIndex        ); 
        virtual tiINT32                 GetDefaultWepKey        (tiUINT32* puKeyIndex );

        virtual tiINT32                 AddKey                  ( OS_802_11_KEY*   pKey             );
        virtual tiINT32                 RemoveKey               ( OS_802_11_REMOVE_KEY* pRemoveKey  ); 

        virtual tiINT32                 SetPMKIDmap             ( OS_802_11_PMKID*          pPMKIDMap   );

		virtual tiINT32                 ConfigPowerManagement   ( OS_802_11_POWER_PROFILE thePowerMgrProfile );

        virtual tiINT32                 GetAssociationInfo      ( OS_802_11_ASSOCIATION_INFORMATION** ppInfo );

        virtual tiINT32                 RegisterEvent           ( IPC_EVENT_PARAMS*  pEventParams );
        virtual tiINT32                 UnRegisterEvent         ( IPC_EVENT_PARAMS* pEventParams );
        /*virtual tiINT32                 UnRegisterEvent         ( tiINT32 iRegisterID );*/
        
        virtual tiINT32                 StartScan               ( scan_Params_t *pScanParams );
        virtual tiINT32                 StopScan                ( );
        virtual tiINT32                 SetScanPolicy           ( UINT8* buffer, UINT16 bufferLength );
        virtual tiINT32                 GetScanBssList          ( bssList_t* bssList );

        virtual tiINT32	                PollApPackets           ( );
        virtual tiINT32	                PollApPacketsFromAC         ( tiUINT32 AC );

        virtual tiINT32	                SetDTagToAcMappingTable ( acTrfcType_e* pDtagToAcTable );
        virtual tiINT32	                SetVAD ( txDataVadTimerParams_t* pVadTimer );
        virtual tiINT32	                GetVAD ( txDataVadTimerParams_t* pVadTimer );

        virtual tiINT32	                SetQosParameters        ( OS_802_11_QOS_PARAMS* pQosParams );
		virtual tiINT32	                SetRxTimeOut	        ( OS_802_11_QOS_RX_TIMEOUT_PARAMS* pRxTimeOut );


		virtual tiINT32					GetAPQosParameters			( OS_802_11_AC_QOS_PARAMS* pACQosParams);
        virtual tiINT32					GetAPQosCapabilitesParameters ( OS_802_11_AP_QOS_CAPABILITIES_PARAMS* pAPQosCapabiltiesParams);
		virtual	tiINT32					AddTspec					( OS_802_11_QOS_TSPEC_PARAMS* pTspecParams);
		virtual	tiINT32					GetTspecParameters			( OS_802_11_QOS_TSPEC_PARAMS* pTspecParams);
		virtual	tiINT32					DeleteTspec					( OS_802_11_QOS_DELETE_TSPEC_PARAMS* pDelTspecParams);
		virtual	tiINT32					GetCurrentACStatus        	( OS_802_11_AC_UPSD_STATUS_PARAMS *pAcStatusParams);
		virtual	tiINT32					SetMediumUsageThreshold		( OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams);
		virtual	tiINT32					SetPhyRateThreshold			( OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams);
        virtual	tiINT32					GetMediumUsageThreshold		( OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams);
		virtual	tiINT32					GetPhyRateThreshold			( OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams);
		virtual	tiINT32					GetDesiredPsMode			( OS_802_11_QOS_DESIRED_PS_MODE* pDesiredPsMode);

        virtual tiINT32                 GetUserPriorityOfStream     ( STREAM_TRAFFIC_PROPERTIES* streamProperties);

        virtual tiINT32                 ConfigTxClassifier      ( tiUINT32 inParamsBuffLen, tiUINT8  *pInParamsBuff);
        virtual tiINT32	                RemoveClassifierEntry   ( clsfr_tableEntry_t *pClsfrEntry );
        virtual tiINT32                 GetClsfrType            ( clsfrTypeAndSupport *currClsfrType );

		virtual tiINT32     			GetDriverCapabilities   ( OS_802_11_DRIVER_CAPABILITIES* pDriverCapabilities );
        virtual tiINT32	                GetSelectedBSSIDInfo    ( OS_802_11_BSSID_EX  *pSelectedBSSIDInfo);
        virtual tiINT32	                GetPrimaryBSSIDInfo     ( OS_802_11_BSSID_EX  *pSelectedBSSIDInfo);

        virtual	tiINT32					SetTrafficIntensityThresholds ( OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS* pTrafficThresholds);
        virtual	tiINT32					GetTrafficIntensityThresholds ( OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS* pTrafficThresholds);
        virtual tiINT32                 ToggleTrafficIntensityEvents ( tiUINT32 NewStatus );

        virtual tiINT32                 Set4XState              ( tiBOOL    bStatus     );
        virtual tiINT32                 Get4XState              ( tiBOOL*   lpbStatus   ); 

        virtual tiINT32                 SetRSSITrigger          ( tiBOOL    bRSSItr                 );
        virtual tiINT32                 GetRSSITrigger          ( tiBOOL*   pbRSSItr                );

        virtual tiINT32                 SetWEPStatus            ( tiUINT32  dwWEPStatus     );
        virtual tiINT32                 GetWEPStatus            ( tiUINT32* pdwWEPStatus    );

        virtual tiINT32                 SetConfiguration        ( OS_802_11_CONFIGURATION*   pConfiguration );
        virtual tiINT32                 GetConfiguration        ( OS_802_11_CONFIGURATION*   pConfiguration );

        virtual tiINT32                 SetPrivacyFilter        ( tiUINT32  dwPrivacyFilter  );
        virtual tiINT32                 GetPrivacyFilter        ( tiUINT32* pdwPrivacyFilter );

        virtual tiINT32                 SetExtRatesIE           ( tiUINT32  dwExtRatesIE    );
        virtual tiINT32                 GetExtRatesIE           ( tiUINT32* pdwExtRatesIE   );

        virtual tiINT32                 SetEarlyWakeupMode      ( tiUINT8  dwEarlyWakeup);
        virtual tiINT32                 GetEarlyWakeupMode      ( tiUINT8* dwEarlyWakeup);

        virtual tiINT32                 hwReadRegister          ( tiUINT32  dwRegisterAddr, tiUINT32* pdwValue );
        virtual tiINT32                 hwWriteRegister         ( tiUINT32  dwRegisterAddr, tiUINT32  dwValue  );

        virtual tiINT32                 ReloadDefaults          ( );
        virtual tiINT32                 IsDriverLoaded          ( );

		virtual tiINT32                 SetBtCoeEnable          ( tiUINT32 uModeEnable);
		virtual tiINT32                 SetBtCoeRate            ( tiUINT8 *pRate);
		virtual tiINT32                 SetBtCoeConfig          ( tiUINT32 *pConfig);
		virtual tiINT32                 SetBtCoeGetStatus       ( tiUINT32 *pStatus);

#ifdef TI_DBG
        virtual tiINT32                 GetDebugBuffer          ( tiUINT8*  pBuffer, tiUINT32  dwLenght  );
        
        virtual tiINT32                 SetReportModule         ( tiUINT8* pData );
        virtual tiINT32                 GetReportModule         ( tiUINT8* pData );    

        virtual tiINT32                 SetReportSeverity       ( tiUINT8* pData );
        virtual tiINT32                 GetReportSeverity       ( tiUINT8* pData );

        virtual tiINT32                 SetOsDbgState           ( tiUINT32 uData );
        virtual tiINT32                 GetOsDbgState           ( tiUINT32* puData );    

		virtual tiINT32                 SetReportPPMode        ( tiUINT32  uData );

        virtual tiINT32                 DisplayStats            ( tiUINT8*  puDbgBuffer, tiUINT32 uBuffSize);

#ifdef DRIVER_PROFILING
		virtual tiINT32					ProfileReport  		 	();
		virtual tiINT32					CpuEstimatorCommand	 	(tiUINT8 uType, tiUINT32 uData);
#endif


#endif
        virtual tiINT32                 SetWPAOptions           ( tiUINT32      fWPA_options);
        virtual tiINT32                 GetWPAOptions( tiUINT32 * fWPA_options);
        
        virtual tiINT32                 GetRegDomainTable       ( TIWLN_REGDOMAINS*     pRegDomainTable );
        virtual tiINT32                 GetMediumUsage          ( TIWLN_MEDIUM_USAGE*   pMediumUsage    );

        virtual tiINT32                 SetPowerMode                ( OS_802_11_POWER_PROFILE  uPower              );
        virtual tiINT32                 GetPowerMode                ( OS_802_11_POWER_PROFILE* puPower             );
		
	    virtual tiINT32                 SetPowerLevelPS             (OS_802_11_POWER_LEVELS  uPower);
	    virtual tiINT32                 GetPowerLevelPS             (OS_802_11_POWER_LEVELS* puPower);
                                        
	    virtual tiINT32                 SetPowerLevelDefault        (OS_802_11_POWER_LEVELS  uPower);
	    virtual tiINT32                 GetPowerLevelDefault        (OS_802_11_POWER_LEVELS* puPower);
                                        
	    virtual tiINT32                 SetPowerLevelDozeMode       (OS_802_11_POWER_PROFILE  uPower);
	    virtual tiINT32                 GetPowerLevelDozeMode       (OS_802_11_POWER_PROFILE* puPower);

        
	    virtual tiINT32                 SetBeaconFilterDesiredState ( OS_802_11_BEACON_FILTER_MODE   uBeaconFilterMode);
	    virtual tiINT32                 GetBeaconFilterDesiredState ( tiUINT8*   pBeaconFilterMode);


        virtual tiINT32                 IsDriverRun                 ( tiBOOL* pbStatus );
		virtual tiINT32                 GWSICommand                 ( tiUINT32* pbStatus );
		virtual tiINT32                 GWSIInitialize              ( tiUINT32* pbStatus );
		virtual tiINT32                 GWSIConfig                  ( tiUINT32* pbStatus );
		virtual tiINT32                 GWSIRelease                 ( tiUINT32* pbStatus );
	                                                                
		virtual tiINT32					GWSIGetInitTable	        ( tiUINT32* pGWSICommand );
                                                                    
        virtual tiINT32                 Open_EAPOL_Interface        ( );
        virtual tiINT32                 Close_EAPOL_Interface       ( );
        virtual tiINT32                 Send_EAPOL_Packet           ( tiVOID* pData, tiUINT32 uSize );
                                                                    
		tiINT32                 		GetVariableLengthOID        ( tiUINT32  uOID, tiVOID** pp, tiUINT32* pnSize, tiUINT32 nNextAllocation = 0);
                                                                    
		virtual tiINT32                 Set_RoamingConfParams       ( UINT8* buffer, UINT16 bufferLength);
		virtual tiINT32                 Get_RoamingConfParams       ( UINT8* buffer, UINT16 bufferLength);
                                                                    
        virtual tiINT32                 GetPowerConsumptionStatistics(PowerConsumptionTimeStat_t * pStatistics);
                                        
		/*PLT                                                       */
		virtual tiINT32					PLT_ReadRegister		    ( UINT32 uRegisterAddr, PUINT32 uRegisterData );
		virtual tiINT32					PLT_WriteRegister		    ( UINT32 uRegisterAddr, UINT32 uRegisterData );
		virtual tiINT32					PLT_RxPerStart			    ();
		virtual tiINT32					PLT_RxPerStop			    ();
		virtual tiINT32					PLT_RxPerClear			    ();
		virtual tiINT32					PLT_RxPerGetResults		    ( PltRxPer_t* pPltRxPer );
		virtual tiINT32					PLT_TxCW				    ( TestCmdChannelBand_t* pPltTxCW);
		virtual tiINT32					PLT_TxContiues			    ( PltTxContinues_t* pPltTxContinues);
		virtual tiINT32					PLT_TxStop  			    ();
		virtual tiINT32					PLT_ReadMIB				    ( PLT_MIB_t* pMib );
		virtual tiINT32					PLT_WriteMIB			    ( PLT_MIB_t* pMib );
	    virtual tiINT32		            PLT_TxCalGainGet		    (PltGainGet_t* pPLTGainGet);
	    virtual tiINT32		            PLT_TxCalGainAdjust	        (tiINT32   uTxGainChange);
		virtual tiINT32		            PLT_TxCalStart	            (PltTxCalibrationRequest_t* pPLTTxCal);
        virtual tiINT32		            PLT_TxCalStop 	            ();
	    virtual tiINT32		            PLT_RxTxCalNVSUpdateBuffer  (PltNvsResultsBuffer_t* pPLT_NVSUpdateBuffer);
	    virtual tiINT32		            PLT_RxCal                   (PltRxCalibrationRequest_t* pPltRxCalibration_t);
        virtual tiINT32                 PLT_RadioTune               (TestCmdChannelBand_t* pChannelBand);
#ifdef _WINDOWS
#endif /* ifdef _WINDOWS */
        EXC_MODULE_SUPPORT_H
};
#endif /* !defined(AFX_CTI_WLAN_ADAPTER_H__3B4F299B_AF52_40DE_BE67_A032DA1FB441__INCLUDED_)*/
