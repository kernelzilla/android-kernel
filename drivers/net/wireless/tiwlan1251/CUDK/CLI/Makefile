
STATIC_LIB ?= y

DEBUG ?= y
FIRMWARE_DYNAMIC_LOAD ?= y
BUILD_SUPPL ?= n

EXC ?= n
SG  ?= n
GWSI_SA ?= n
DRIVER_PROFILING ?= n

STA_DK_ROOT = ../..

ifeq ($(DEBUG),y)
  DEBUGFLAGS = -O0 -g -DDEBUG -DTI_DBG -fno-builtin   # "-O" is needed to expand inlines
#  DEBUGFLAGS+= -DDEBUG_MESSAGES
else
  DEBUGFLAGS = -O2
endif

DEBUGFLAGS+= -DHOST_COMPILE

DK_ROOT = $(STA_DK_ROOT)
COMMON  = $(DK_ROOT)/common
COMSRC  = $(COMMON)/src
TESTSRC = $(DK_ROOT)/Test
CUDK_ROOT ?= $(DK_ROOT)/CUDK
IPC_SRC = $(CUDK_ROOT)/IPC/Linux

OUTPUT_DIR ?= $(STA_DK_ROOT)/CUDK/CLI

DK_INCS = -I $(CUDK_ROOT)/CLI/drv_inc \
	-I $(CUDK_ROOT)/CLI/pform_inc

ifeq ($(EXC),y)
        DK_INCS += \
	-I $(CUDK_ROOT)/CLI \
	-I $(CUDK_ROOT)/Inc/EXC \
	-I $(CUDK_ROOT)/CLI/EXC
endif
ifeq ($(SG), y)
#   	DK_INCS += -I $(COMSRC)/core/NetworkCtrl/BThWlanCombo
endif

DK_DEFINES = \
	-D __BYTE_ORDER_LITTLE_ENDIAN \
	-D INCLUDE_DEFRAGMENTATION \
	-D CONFIGURE_BSS_TYPE_STA \
	-D TNETW1130=1 \
	-D DOT11_A_G=1 \
	-D ELP_NO_PDA_SCREEN_VIBRATE
ifeq ($(EXC), y)
	DK_DEFINES += -D EXC_MODULE_INCLUDED
endif    
ifeq ($(SG), y)
   	DK_DEFINES += -D BTH_COEXISTENCE
endif

ifeq ($(DRIVER_PROFILING),y)
   DK_DEFINES += -D DRIVER_PROFILING
endif

TARGET := $(OUTPUT_DIR)/wlan_cu
SRCS  := ticon.c console.c cu_cmd.c eth_utils.c wipp_ctrl.c dbg_module.c ipc.c g_tester.c  
ifeq ($(EXC), y)
    SRCS += EXC/cu_cmd_exc.c
endif
OBJS = $(SRCS:.c=.o)

DEPS = $(SRCS:%.c=%.d)

# export CROSS_COMPILE ?= arm_v4t_le-
export CROSS_COMPILE ?= arm-linux-

#ARMFLAGS  = -Wstrict-prototypes -Wno-trigraphs -Os -fno-strict-aliasing -fno-common -Uarm -fno-common -pipe
#ARMFLAGS += -mapcs -mno-sched-prolog -mapcs-32 -D__LINUX_ARM_ARCH__=4 -march=armv4
#ARMFLAGS += -mtune=arm9tdmi -mshort-load-bytes -msoft-float  -iwithprefix include

ARMFLAGS  = -fno-common -pipe

TI_UTIL_ADAPTER_LIB_DIR=$(CUDK_ROOT)/UtilityAdapter
TI_UTIL_ADAPTER_LIB = tiOsLib

#Supplicant image building
SUPP_ROOT = $(STA_DK_ROOT)/CUDK/Supplicant
TI_SUPP_LIB_DIR = $(SUPP_ROOT)/odyssey

ifeq ($(BUILD_SUPPL), y)
  SUPPL_TARGET = $(OUTPUT_DIR)/wlan_suppl
endif

INCLUDES = $(DK_INCS) -I $(STA_DK_ROOT)/CUDK/CLI/pform_inc \
			-I $(STA_DK_ROOT)/CUDK/Inc

ifeq ($(GWSI_SA), n)
ifeq ($(BUILD_SUPPL), y)
	INCLUDES +=	-I $(SUPP_ROOT)/odyssey/odSupp/linux/inc
endif
endif

CFLAGS = -Wall -Wstrict-prototypes $(DEBUGFLAGS) $(INCLUDES) -D__LINUX__ $(DK_DEFINES)
CFLAGS += -DDRV_NAME='"tiwlan"'

ifeq ($(FIRMWARE_DYNAMIC_LOAD), y)
    CFLAGS += -DFIRMWARE_DYNAMIC_LOAD
endif

ifneq "$(CROSS_COMPILE)" ""		#compile for ARM
	CFLAGS += $(ARMFLAGS)
        # strip symbols
ifneq "$(DEBUG)" "y"
    LDFLAGS = -s
endif

endif    # CROSS_COMPILE != ""

.PHONY: all $(TI_UTIL_ADAPTER_LIB) $(SUPPL_TARGET)

all: $(TARGET) $(SUPPL_TARGET)

$(TI_UTIL_ADAPTER_LIB):
	$(MAKE) -e -C $(TI_UTIL_ADAPTER_LIB_DIR) DEBUG=$(DEBUG) EXC=$(EXC) STATIC_LIB=$(STATIC_LIB) CROSS_COMPILE=$(CROSS_COMPILE) SG=$(SG) DRIVER_PROFILING=$(DRIVER_PROFILING)

ifeq ($(BUILD_SUPPL), y)

$(SUPPL_TARGET):
	$(MAKE) -e -C $(TI_SUPP_LIB_DIR) DEBUG=$(DEBUG) EXC=$(EXC) CROSS_COMPILE=$(CROSS_COMPILE) STATIC_LIB=$(STATIC_LIB)
ifeq ($(DEBUG), y)
	cp -a $(SUPPL_TARGET) $(SUPPL_TARGET)_debug
	$(CROSS_COMPILE)strip -s $(SUPPL_TARGET)
endif    #DEBUG

endif

$(TARGET): $(OBJS) $(TI_UTIL_ADAPTER_LIB)
#	echo MAKECMDGOALS: $(MAKECMDGOALS) : $(findstring $(MAKECMDGOALS), "clean cleanall")
	$(CROSS_COMPILE)gcc $(OBJS) -L $(CUDK_ROOT)/Output -l $(TI_UTIL_ADAPTER_LIB) $(LDFLAGS) -lpthread -lc -lstdc++ -o $@
ifeq ($(DEBUG), y)
	cp -a $(TARGET) $(TARGET)_debug
	$(CROSS_COMPILE)strip -s $(TARGET)
endif    #DEBUG

ifeq ("$(findstring $(MAKECMDGOALS), clean cleanall)", "")
-include .depend
endif

%.o: %.cpp
	@echo $@
	@$(CROSS_COMPILE)gcc $(CFLAGS) -c $^ -o $@

%.o: %.c
	@echo $@
	@$(CROSS_COMPILE)gcc $(CFLAGS) -c $< -o $@

%.i: %.c
	@$(CROSS_COMPILE)gcc $(CFLAGS) -E -c $< > $@

%.d: %.c
	@echo $@
	@$(CROSS_COMPILE)gcc $< -MT$(^:%.c=%.o) -M $(CFLAGS) > $@

%.d: %.cpp
	@echo $@
	@$(CROSS_COMPILE)gcc $< -MT$(^:%.cpp=%.o) -M $(CFLAGS) > $@

.depend: $(DEPS)
	rm -f $@
	for aa in $^; do cat $$aa >> $@; done

.PHONY: clean
clean:
	@rm -f .depend $(OBJS) $(DEPS) $(SRCS:.c=.i) *.ko .*.cmd .*.flags *.mod.c *~ *.~*

cleanall: clean
	$(MAKE) -e -C $(TI_UTIL_ADAPTER_LIB_DIR) CROSS_COMPILE=$(CROSS_COMPILE) clean
ifeq ($(GWSI_SA), n)
ifeq ($(BUILD_SUPPL), y)
	$(MAKE) -e -C $(TI_SUPP_LIB_DIR) CROSS_COMPILE=$(CROSS_COMPILE) clean
endif
	@rm -f $(SUPPL_OBJS) $(SUPPL_OBJS:.c=.d) $(OUTPUT_DIR)/wlan_suppl $(SUPPL_TARGET)
endif
	@rm -rf $(TARGET) $(OUTPUT_DIR)/../debug

TAGS: $(SRCS)
	{ find $(COMMON) -name '*.h' -print ; \
	  find . -name '*.h' -print ; } | etags -
	  etags -a $(SRCS)




