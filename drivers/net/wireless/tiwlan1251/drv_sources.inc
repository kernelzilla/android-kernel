#start of TNETW_INCS#    do not remove or change this comment
TNETW_INCS = \
    $(COMMON)/inc \
    $(COMSRC)/inc \
    $(COMSRC)/utils \
    $(HALSRC)/hal/inc \
    $(HALSRC)/hal/TnetwServices \
    $(COMSRC)/TNETW_Driver/Export_Inc \
    $(COMSRC)/TNETW_Driver/Data_Srv/inc \
    $(COMSRC)/TNETW_Driver/FW_Transfer/inc \
    $(COMSRC)/TNETW_Driver/Ctrl/Export_Inc \
    $(COMSRC)/TNETW_Driver/MacServices/inc \
    $(COMSRC)/TNETW_Driver \
    $(COMSRC)/TNETW_Driver/MacServices/Export_Inc \
    $(COMSRC)/TNETW_Driver/MacServices/src/ScanServer \
    $(COMSRC)/TNETW_Driver/MacServices/src/PowerSaveServer \
    $(COMSRC)/TNETW_Driver/MacServices/src/MeasurementSRV \
    $(COMSRC)/TNETW_Driver/TNETWIF/Export_Inc \
    $(COMSRC)/TNETW_Driver/Data_Srv/Tx_Ctrl_Blk \
    $(COMSRC)/TNETW_Driver/TNETWIF/Inc 

ifeq ($(PLATFORM),1251)
    TNETW_INCS += \
        $(HALSRC)/hal/TnetwServices/TNETW1251
else
    ifeq ($(PLATFORM),1150)
        TNETW_INCS += \
            $(HALSRC)/hal/TnetwServices/TNETW1150 
    else
        TNETW_INCS += \
            $(HALSRC)/hal/TnetwServices/TNETW1130 
    endif
endif
    
TNETW_INCS += \
    $(HALSRC)/hal/hl_data \
    $(HALSRC)/hal/hl_ctrl \
    $(HALSRC)/hal/hw_ctrl \
    $(HALSRC)/hal/security \
    $(HALSRC)/hal/Export_Inc \
    $(HALSRC)/hal/FirmwareApi \
    $(HALSRC)/BusAccess/Shm_Common \
    $(HALSRC)/BusAccess/Export_Inc 

ifeq ($(EXC),y)
#end of TNETW_INCS_EXC#    do not remove or change this comment
endif

#end of TNETW_INCS#    do not remove or change this comment


#end of GWSI_INCS#     do not remove or change this comment


#start of CORE_INCS#     do not remove or change this comment
CORE_INCS = \
    $(COMSRC)/core/inc \
    $(COMSRC)/core/ExpInc \
    $(COMSRC)/core/data_ctrl/Tx \
    $(COMSRC)/core/data_ctrl/Ctrl \
    $(COMSRC)/core/data_ctrl/Ctrl/TrafficMonitor \
    $(COMSRC)/core/data_ctrl/Ctrl/4X \
    $(COMSRC)/core/data_ctrl/Rx \
    $(COMSRC)/core/Core_Adapt/Export_Inc \
    $(COMSRC)/core/EvHandler \
    $(COMSRC)/core/sme/Inc \
    $(COMSRC)/core/sme/smeSm/ \
    $(COMSRC)/core/sme/siteMgr \
    $(COMSRC)/core/sme/configMgr \
    $(COMSRC)/core/sme/conn \
    $(COMSRC)/core/sme/HealthMonitor \
    $(COMSRC)/core/srv/inc \
    $(COMSRC)/core/srv/scanSrv \
    $(COMSRC)/core/srv/scr \
    $(COMSRC)/core/rsn \
    $(COMSRC)/core/rsn/inc \
    $(COMSRC)/core/mlme \
    $(COMSRC)/core/mlme/Assoc \
    $(COMSRC)/core/mlme/Auth/open \
    $(COMSRC)/core/mlme/Auth/shared \
    $(COMSRC)/Management/QOS/Inc/ \
    $(COMSRC)/Management/Roaming/Inc \
    $(COMSRC)/core/NetworkCtrl/inc \
    $(COMSRC)/Management/AirLink/Measurement \
    $(COMSRC)/Management/AirLink/Measurement/dot11h \
    $(COMSRC)/Management/AirLink/inc \
    $(COMSRC)/Management/scan/inc \
    $(COMSRC)/Management/SoftGemini/inc \
    $(COMSRC)/Management/scan/scanCncn \
    $(COMSRC)/Management/SoftGemini/src \
    $(COMSRC)/core/NetworkCtrl/RegulatoryDomain \
    $(COMSRC)/Management/PowerMgr/ \
    $(COMSRC)/Management/RecoveryMgr/ \
    $(COMSRC)/core/NetworkCtrl/QOS \
    $(COMSRC)/Application/inc \
    $(COMSRC)/Application/ExpInc \
    $(COMSRC)/Application/ScanMngr \
    $(COMSRC)/Management/apConn \
    $(COMSRC)/core/currBss \
    $(COMSRC)/Management/scan/inc \
    $(DBG_MDL)/inc

#end of CORE_INCS_EXC#   do not remove or change this comment

#end of CORE_INCS#   do not remove or change this comment


#start of UTIL_SRCS#    do not remove or change this comment
UTIL_SRCS = \
    $(COMSRC)/utils/fsm.c \
    $(COMSRC)/utils/nrfsm.c \
    $(COMSRC)/utils/report.c \
    $(COMSRC)/utils/utils.c \
    $(COMSRC)/utils/bufferPool.c \
    $(COMSRC)/utils/memMngrEx.c \
#end of UTIL_SRCS#      do not remove or change this comment


#start of TNETW_SRCS#   do not remove or change this comment
TNETW_SRCS = \
    $(COMSRC)/TNETW_Driver/MacServices/src/MacServices.c \
    $(COMSRC)/TNETW_Driver/MacServices/src/ScanServer/ScanSrv.c \
    $(COMSRC)/TNETW_Driver/MacServices/src/ScanServer/ScanSrvSM.c \
    $(COMSRC)/TNETW_Driver/MacServices/src/PowerSaveServer/PowerSrv.c \
    $(COMSRC)/TNETW_Driver/MacServices/src/PowerSaveServer/PowerSrvSM.c \
    $(COMSRC)/TNETW_Driver/MacServices/src/MeasurementSRV/MeasurementSrv.c \
    $(COMSRC)/TNETW_Driver/MacServices/src/MeasurementSRV/MeasurementSrvSM.c \
    $(COMSRC)/TNETW_Driver/MacServices/src/MeasurementSRV/measurementSrvDbgPrint.c \
    $(COMSRC)/TNETW_Driver/Data_Srv/Tx_Ctrl_Blk/TNETW_Driver.c \
    $(COMSRC)/TNETW_Driver/Data_Srv/Tx_Ctrl_Blk/TNETW_Driver_Tx.c \
    $(COMSRC)/TNETW_Driver/Ctrl/Cmd_Queue/CmdQueue.c \
    $(COMSRC)/TNETW_Driver/Data_Srv/Tx_HW_Queue/txHwQueue.c \
    $(COMSRC)/TNETW_Driver/Data_Srv/Tx_HW_Queue/txHwQueueCalc.c \
    $(COMSRC)/TNETW_Driver/Data_Srv/Tx_Ctrl_Blk/txCtrlBlk.c \
    $(COMSRC)/TNETW_Driver/FW_Transfer/Cmd_MBox/CmdMBox.c \
    $(COMSRC)/TNETW_Driver/FW_Transfer/Tx_Result/txResult.c \
    $(COMSRC)/TNETW_Driver/FW_Transfer/Tx_Xfer/Slave_Dbl_Buf/txXfer.c \
    $(COMSRC)/TNETW_Driver/FW_Transfer/FwEvent/FwEvent.c \
    $(COMSRC)/TNETW_Driver/FW_Transfer/Rx_Xfer/RxXfer.c \
    $(COMSRC)/TNETW_Driver/FW_Transfer/EventMbox/eventMbox.c \
    $(COMSRC)/TNETW_Driver/MacServices/src/PowerAuthorization/PowerAuthorization.c \
    $(COMSRC)/TNETW_Driver/TNETWIF/TNETWIF.c \
    $(COMSRC)/TNETW_Driver/TNETWIF/ELP_Controller/ElpCtrl.c \
    $(COMSRC)/TNETW_Driver/TNETWIF/BusTxn/whalHwAccess.c \
    $(COMSRC)/TNETW_Driver/TNETWIF/TNETWArbiter/TNETWArb.c \
    $(COMSRC)/TNETW_Driver/TNETWIF/TNETWArbiter/TNETWArbSM.c \
    $(COMSRC)/TNETW_Driver/TNETWIF/TNETWArbiter/TNETWArb_buffer.c \
    $(HALSRC)/hal/hl_ctrl/whalCtrl.c \
    $(HALSRC)/hal/hl_ctrl/whalParams.c \
    $(HALSRC)/hal/hl_data/whalDefrag.c \
    $(HALSRC)/hal/hl_data/whalEndpntEnt.c \
    $(HALSRC)/BusAccess/Shm_Common/shmBus.c \
    $(HALSRC)/BusAccess/Shm_Common/shmFwCtrl.c \
    $(HALSRC)/BusAccess/Shm_Common/shmDebug.c \
    $(HALSRC)/BusAccess/Shm_Common/shmUtils.c \
    $(HALSRC)/hal/hw_ctrl/whalHwCtrl.c \
    $(HALSRC)/BusAccess/Shm_Common/whalHwEeprom.c \
    $(HALSRC)/hal/hw_ctrl/whalHwMboxConfig.c \
    $(HALSRC)/hal/hw_ctrl/whalHwMboxCmd.c \
    $(HALSRC)/hal/hw_ctrl/whalHwMboxCmdBit.c \
    $(HALSRC)/hal/hl_ctrl/whalRecovery.c \
    $(HALSRC)/hal/hl_ctrl/whalRadio.c \
    $(HALSRC)/hal/hl_ctrl/whalDebug.c \
    $(HALSRC)/hal/security/whalSecurity.c \
    $(HALSRC)/hal/security/whalWep.c \
    $(HALSRC)/hal/security/whalWpa.c

ifndef GWSI_SA
    TNETW_SRCS += \
        $(COMSRC)/TNETW_Driver/FW_Transfer/HwInit/HwInit.c \
        $(COMSRC)/TNETW_Driver/Ctrl/RecoverCtrl/recoveryCtrl.c
endif

ifeq ($(WSPI),y)
    TNETW_SRCS += \
        $(COMSRC)/TNETW_Driver/TNETWIF/BusTxn/wspi.c \
        $(COMSRC)/TNETW_Driver/TNETWIF/BusTxn/tiwlanspi.c
endif

ifeq ($(DEBUG),y)
    TNETW_SRCS += \
        $(HALSRC)/BusAccess/Shm_Common/whalTrace.c \
        $(COMSRC)/TNETW_Driver/FW_Transfer/DebugTrace_Xfer/DebugTraceXfer.c
endif

#end of TNETW_SRCS_EXC#   do not remove or change this comment

#end of TNETW_SRCS#    do not remove or change this comment

#end of GWSI_SRCS#    do not remove or change this comment

#start of CORE_SRCS#   do not remove or change this comment
CORE_SRCS = \
    $(COMSRC)/core/data_ctrl/Tx/MsduList.c \
    $(COMSRC)/core/data_ctrl/Tx/tx.c \
    $(COMSRC)/core/data_ctrl/Ctrl/Ctrl.c \
    $(COMSRC)/core/data_ctrl/Ctrl/Clsfr.c \
    $(COMSRC)/core/data_ctrl/Ctrl/4X/Concatenator.c \
    $(COMSRC)/core/data_ctrl/Ctrl/4X/DeConcatenator.c \
    $(COMSRC)/core/data_ctrl/Ctrl/4X/ackEmulDb.c \
    $(COMSRC)/core/data_ctrl/Ctrl/4X/ackEmulUtil.c \
    $(COMSRC)/core/data_ctrl/Ctrl/4X/fourX.c \
    $(COMSRC)/core/data_ctrl/Ctrl/4X/fourXManager.c \
    $(COMSRC)/Management/PowerMgr/PowerMgr.c \
    $(COMSRC)/Management/PowerMgr/PowerMgrDbgPrint.c \
    $(COMSRC)/Management/RecoveryMgr/recoveryMgr.c \
    $(COMSRC)/core/data_ctrl/Ctrl/RateAdaptation.c \
    $(COMSRC)/core/data_ctrl/Ctrl/TrafficMonitor/GeneralUtil.c \
    $(COMSRC)/core/data_ctrl/Ctrl/TrafficMonitor/TrafficMonitor.c \
    $(COMSRC)/core/data_ctrl/Rx/rx.c \
    $(COMSRC)/core/sme/conn/conn.c \
    $(COMSRC)/core/sme/conn/connIbss.c \
    $(COMSRC)/core/sme/conn/connInfra.c \
    $(COMSRC)/core/sme/configMgr/configMgr.c \
    $(COMSRC)/core/sme/smeSm/smeSmApi.c \
    $(COMSRC)/core/sme/smeSm/smeSm.c \
    $(COMSRC)/core/sme/HealthMonitor/healthMonitor.c \
    $(COMSRC)/core/sme/siteMgr/siteMgr.c \
    $(COMSRC)/core/sme/siteMgr/siteHash.c \
    $(COMSRC)/core/sme/siteMgr/select.c \
    $(COMSRC)/core/sme/siteMgr/templates.c \
    $(COMSRC)/core/mlme/mlmeBuilder.c \
    $(COMSRC)/core/mlme/mlmeParser.c \
    $(COMSRC)/core/mlme/mlmeSm.c \
    $(COMSRC)/core/mlme/Assoc/assocSM.c \
    $(COMSRC)/core/mlme/Auth/authSm.c \
    $(COMSRC)/core/mlme/Auth/open/openAuthSm.c \
    $(COMSRC)/core/mlme/Auth/shared/sharedKeyAuthSm.c \
    $(COMSRC)/core/rsn/rsn.c \
    $(COMSRC)/core/rsn/mainSecSm.c \
    $(COMSRC)/core/rsn/mainSecNull.c \
    $(COMSRC)/core/rsn/mainSecKeysOnly.c \
    $(COMSRC)/core/rsn/admCtrl.c \
    $(COMSRC)/core/rsn/admCtrlWep.c \
    $(COMSRC)/core/rsn/admCtrlNone.c \
    $(COMSRC)/core/rsn/admCtrlWpa.c \
    $(COMSRC)/core/rsn/admCtrlWpa2.c \
    $(COMSRC)/core/srv/scr/scr.c \
    $(COMSRC)/core/EvHandler/EvHandler.c \
    $(COMSRC)/Management/AirLink/SwitchChannel/SwitchChannel.c \
    $(COMSRC)/Management/AirLink/Measurement/measurementMgr.c \
    $(COMSRC)/Management/AirLink/Measurement/measurementMgrSM.c \
    $(COMSRC)/Management/AirLink/Measurement/dot11h/spectrumMngmntMgr.c \
    $(COMSRC)/Management/AirLink/Measurement/requestHandler.c \
    $(COMSRC)/Management/scan/scanCncn/ScanCncn.c \
    $(COMSRC)/Management/scan/scanCncn/ScanCncnAppSM.c \
    $(COMSRC)/Management/scan/scanCncn/ScanCncnDrvSM.c \
    $(COMSRC)/Management/scan/scanCncn/ScanCncnOidSM.c \
    $(COMSRC)/Management/scan/scanCncn/ScanCncnRoamingContSM.c \
    $(COMSRC)/Management/scan/scanCncn/ScanCncnRoamingImmedSM.c \
    $(COMSRC)/Management/scan/scanCncn/ScanCncnAppApi.c \
    $(COMSRC)/Management/apConn/apConn.c \
    $(COMSRC)/Management/SoftGemini/src/SoftGemini.c \
    $(COMSRC)/core/NetworkCtrl/RegulatoryDomain/regulatoryDomain.c \
    $(COMSRC)/core/rsn/keyDerive.c \
    $(COMSRC)/core/rsn/keyDeriveWep.c \
    $(COMSRC)/core/rsn/keyDeriveTkip.c \
    $(COMSRC)/core/rsn/keyDeriveAes.c \
    $(COMSRC)/core/rsn/keyParser.c \
    $(COMSRC)/core/rsn/broadcastKeySM.c \
    $(COMSRC)/core/rsn/unicastKeySM.c \
    $(COMSRC)/core/rsn/broadcastKey802_1x.c \
    $(COMSRC)/core/rsn/unicastKey802_1x.c \
    $(COMSRC)/core/rsn/broadcastKeyNone.c \
    $(COMSRC)/core/rsn/unicastKeyNone.c \
    $(COMSRC)/core/rsn/keyParserExternal.c \
    $(COMSRC)/core/rsn/keyParserWep.c \
    $(COMSRC)/core/rsn/mainKeysSm.c \
    $(COMSRC)/core/currBss/currBss.c \
    $(COMSRC)/Application/ScanMngr/scanMngr.c \
    $(COMSRC)/Application/Roaming/roamingMngr.c \
    $(COMSRC)/core/Core_Adapt/Core_AdaptTx.c \
    $(COMSRC)/Management/QOS/qosMngr.c \
    $(COMSRC)/Management/QOS/trafficAdmControl.c

ifeq ($(SUPPORT_4X),y)
    CORE_SRCS += \
        $(COMSRC)/core/data_ctrl/Ctrl/4X/Concatenator.c \
        $(COMSRC)/core/data_ctrl/Ctrl/4X/DeConcatenator.c \
        $(COMSRC)/core/data_ctrl/Ctrl/4X/ackEmulDb.c \
        $(COMSRC)/core/data_ctrl/Ctrl/4X/ackEmulUtil.c \
        $(COMSRC)/core/data_ctrl/Ctrl/4X/fourX.c \
        $(COMSRC)/core/data_ctrl/Ctrl/4X/fourXManager.c
endif

#end of CORE_SRCS_EXC#   do not remove or change this comment

#end of CORE_SRCS#   do not remove or change this comment


ifdef GWSI_SA
    DK_INCS = \
        $(TNETW_INCS) \
        $(GWSI_INCS)

    DK_SRCS = \
        $(GWSI_SRCS) \
        $(TNETW_SRCS) \
        $(UTIL_SRCS)

else
    DK_INCS = \
        $(TNETW_INCS) \
        $(CORE_INCS)

    DK_SRCS = \
        $(CORE_SRCS) \
        $(TNETW_SRCS)
endif



DK_OBJS = $(patsubst %.c, %.o, $(DK_SRCS) $(UTIL_SRCS))

