
#start of OS_DK_INCS_FILES#   do not remove or change this comment
OS_DK_INCS_FILES = \
    $(COMSRC)/inc/802_11Defs.h \
    $(COMSRC)/inc/Ethernet.h \
    $(COMSRC)/inc/paramIn.h \
    $(COMMON)/inc/srcApi.h \
    $(COMMON)/inc/TI_IPC_Api.h \
    $(COMMON)/inc/memMngrEx.h \
    $(COMMON)/inc/paramOut.h \
    $(COMMON)/inc/osDot11.h \
    $(COMMON)/inc/tiwlnif.h \
    $(COMMON)/inc/ratesTypes.h \
    $(COMMON)/inc/scanTypes.h \
    $(COMMON)/inc/bssTypes.h \
    $(COMMON)/inc/roamingMngrTypes.h \
    $(COMMON)/inc/paramMng.h \
    $(COMMON)/inc/commonTypes.h \
    $(COMMON)/inc/coreDefaultParams.h \
    $(COMMON)/inc/tiQosTypes.h \
    $(COMMON)/inc/whalDefaultParams.h \
    $(COMMON)/inc/TI_Results.h \
    $(COMMON)/inc/tiioctl.h \
    $(COMMON)/inc/wspVer.h \
    $(COMSRC)/core/inc/mlmeApi.h \
    $(COMSRC)/core/sme/configMgr/configMgr.h \
    $(COMSRC)/core/sme/HealthMonitor/healthMonitor.h \
    $(COMSRC)/utils/fsm.h \
    $(COMSRC)/utils/report.h \
    $(COMSRC)/utils/utils.h \
    $(COMSRC)/driver_bypass/driver_bypass.h \
    $(COMSRC)/TNETW_Driver/Export_Inc/TNETW_Driver_types.h \
    $(COMSRC)/TNETW_Driver/Export_Inc/TNETW_Driver_api.h \
    $(COMSRC)/TNETW_Driver/Ctrl/Export_Inc/CmdQueue_api.h \
    $(COMSRC)/TNETW_Driver/MacServices/Export_Inc/MacServices_api.h \
    $(COMSRC)/TNETW_Driver/MacServices/Export_Inc/measurementTypes.h \
    $(COMSRC)/TNETW_Driver/FW_Transfer/inc/rxXfer_api.h \
    $(COMSRC)/BusAccess/Export_Inc/whalBus_Defs.h \
    $(COMSRC)/BusAccess/Export_Inc/whalBus_Api.h \
    $(COMSRC)/hal/inc/whalCommon.h \
    $(COMSRC)/hal/FirmwareApi/public_descriptors.h \
    $(COMSRC)/hal/FirmwareApi/public_infoele.h \
    $(COMSRC)/hal/FirmwareApi/public_types.h \
    $(COMSRC)/hal/FirmwareApi/public_host_int.h \
    $(COMSRC)/hal/FirmwareApi/public_radio.h \
    $(COMSRC)/hal/FirmwareApi/public_commands.h \
    $(COMSRC)/hal/FirmwareApi/public_event_mbox.h \
    $(COMSRC)/hal/FirmwareApi/whalHwDefs.h \
    $(COMSRC)/hal/TnetwServices/tnetwCommon.h \
    $(COMSRC)/hal/TnetwServices/TNETW1251/whalHwRegs.h \
    $(COMSRC)/hal/TnetwServices/TNETW1251/TnetwRegisters.h \
    $(COMSRC)/hal/Export_Inc/whalCtrl_api.h \
    $(COMSRC)/hal/Export_Inc/whalCtrl_prm.h \
    $(COMSRC)/hal/hw_ctrl/whalHwMboxCmdBit.h \
    $(COMSRC)/hal/hw_ctrl/whalHwMboxCmd.h \
    $(COMSRC)/hal/hw_ctrl/whalHwMboxConfig.h \
    $(COMSRC)/hal/hw_ctrl/whalHwCtrl.h \
    $(COMSRC)/hal/hl_ctrl/whalCtrl.h \
    $(COMSRC)/hal/hl_ctrl/whalParams.h

#start of OS_DK_INCS_FILES_CCX#   do not remove or change this comment
ifeq ($(CCX),y)
    OS_DK_INCS_FILES_CCX = \
        $(COMMON)/inc/CCX/paramOutCcx.h \
        $(COMMON)/inc/CCX/osDot11Ccx.h

    OS_DK_INCS_FILES += $(OS_DK_INCS_FILES_CCX)
endif
#end of OS_DK_INCS_FILES_CCX#   do not remove or change this comment

#start of OS_DK_INCS_FILES#   do not remove or change this comment


#start of OS_DK_INCS#   do not remove or change this comment
OS_DK_INCS = \
    $(COMMON)/inc \
    $(COMSRC)/inc \
    $(COMSRC)/core/inc \
    $(COMSRC)/core/sme/configMgr \
    $(COMSRC)/core/sme/HealthMonitor \
    $(COMSRC)/utils \
    $(COMSRC)/driver_bypass \
    $(COMSRC)/TNETW_Driver/Export_Inc \
    $(COMSRC)/TNETW_Driver/Ctrl/Export_Inc \
    $(COMSRC)/TNETW_Driver/MacServices/Export_Inc \
    $(COMSRC)/TNETW_Driver/FW_Transfer/inc \
    $(COMSRC)/BusAccess/Export_Inc \
    $(COMSRC)/hal/inc \
    $(COMSRC)/hal/FirmwareApi \
    $(COMSRC)/hal/TnetwServices \
    $(COMSRC)/hal/TnetwServices/TNETW1251 \
    $(COMSRC)/hal/Export_Inc \
    $(COMSRC)/hal/hw_ctrl \
    $(COMSRC)/hal/hl_ctrl

#start of OS_DK_INCS_CCX#   do not remove or change this comment
ifeq ($(EXC),y)
    OS_DK_INCS_CCX = \
        $(COMMON)/inc/CCX

    OS_DK_INCS += $(OS_DK_INCS_CCX)
endif
#end of OS_DK_INCS_CCX#   do not remove or change this comment

#start of OS_DK_INCS#   do not remove or change this comment
