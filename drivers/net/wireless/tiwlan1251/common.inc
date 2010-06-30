##
##
## Directory Locations
##
##

HAL_ROOT    := $(DK_ROOT)
COMMON      := $(DK_ROOT)/common
COMSRC      := $(COMMON)/src
HALSRCR     := $(HAL_ROOT)/common
HALSRC      := $(HALSRCR)/src
DRV_BYPASS  := $(COMSRC)/driver_bypass
LINUX_PFORM := $(DK_ROOT)/pform/linux
PFORM       := $(DK_ROOT)/pform

##
##
## Make Flags
##
##

#
# Choose an interface (CRTWIPP)
#
WIFI_BOARD ?= MSM_DREAM

#
# Choose the TNET type (1251)
#
PLATFORM ?= 1251

#
# Compile with EXC support or not
#
EXC ?= n

#
# Choose the bus type (n for SDIO, y for WSPI)
#
WSPI ?= n

#
# Use periodic interrupt
#
INTR ?= y

#
# Use work queues
#
WORKQUEUE ?= y

#
# Enable or disable debugging
#
DEBUG ?= n

#
# Enable or disable OS and init messages
#
INFO ?= n

#
# Archtecture Type
#
ARCH ?= arm

#
# Cross Compiler Prefix
#
CROSS_COMPILE ?= arm-eabi-

#
# Enable driver profiling
#
DRIVER_PROFILING ?= n




##
##
## Driver Compilation Directives
##
##

ifeq ($(EXC),y)
    DK_DEFINES += -D EXC_MODULE_INCLUDED
endif

ifeq ($(INTR),n)
    DK_DEFINES += -D PRIODIC_INTERRUPT
endif    

ifeq ($(WORKQUEUE),y)
    DK_DEFINES += -D DM_USE_WORKQUEUE
endif    

ifeq ($(DEBUG),y)
    DK_DEFINES += -D TI_DBG
    DK_DEFINES += -D REPORT_LOG
else
    DK_DEFINES += -U REPORT_LOG
endif

DK_DEFINES += -D __BYTE_ORDER_LITTLE_ENDIAN
DK_DEFINES += -D CONFIGURE_BSS_TYPE_STA
DK_DEFINES += -D PERIODIC_WAKEUP

ifeq ($(DRIVER_PROFILING),y)
   DK_DEFINES += -D DRIVER_PROFILING
endif


##
##
## Platform Compilation Directives
##
##

ifeq ($(WIFI_BOARD),CRTWIPP)
	TIWLAN_OMAP1610_CRTWIPP ?= y
	PFORM_DEFINES += -D TIWLAN_OMAP1610_CRTWIPP
endif

ifeq ($(PLATFORM),1251)
    PFORM_DEFINES += -D TNETW1150=1
    PFORM_DEFINES += -D TNETW1251=1
endif

ifeq ($(WSPI),y)
	PFORM_DEFINES += -D HW_ACCESS_WSPI
else
	PFORM_DEFINES += -D HW_ACCESS_SDIO
	PFORM_DEFINES += -D USE_SYNC_API
	PFORM_DEFINES += -D USE_RECOVERY
endif

ifeq ($(INFO),y)
    PFORM_DEFINES += -D OS_INFO_MESSAGES
	PFORM_DEFINES += -D INIT_MESSAGES
endif

ifeq ($(WIFI_BOARD),CRTWIPP)
PFORM_DEFINES += -D TIWLAN_OMAP1610
endif

ifeq ($(WIFI_BOARD),MSM_DREAM)
PFORM_DEFINES += -D TIWLAN_MSM7000
endif

# PFORM_DEFINES += -D DEBUG_MESSAGES
PFORM_DEFINES += -D HOST_COMPILE
PFORM_DEFINES += -D FW_RUNNING_AS_STA
PFORM_DEFINES += -D NOT_SUPPORT_08_BIT_ACCESS_COMMAND
PFORM_DEFINES += -D FOUR_ALIGNMENT
PFORM_DEFINES += -D USE_INIT_FILE
PFORM_DEFINES += -D ENABLE_ROAMING_BY_DEFAULT

ifeq ($(FIRMWARE_DYNAMIC_LOAD),y)
PFORM_DEFINES += -D FIRMWARE_DYNAMIC_LOAD
endif

##
##
## Miscellaneous Compilation Directivcs
##
##

EXTRA_CFLAGS += -fsigned-char
EXTRA_CFLAGS += -D __int64="long long"
EXTRA_CFLAGS += -D __LINUX__
EXTRA_CFLAGS += -D TIWLN_MAJOR_VERSION=5
EXTRA_CFLAGS += -D TIWLN_MINOR_VERSION=1
EXTRA_CFLAGS += $(DK_DEFINES)
EXTRA_CFLAGS += $(PFORM_DEFINES)
