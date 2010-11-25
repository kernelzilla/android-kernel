DEBUG ?= y
STATIC_LIB ?= y
EXC ?= n
SG ?= n
DRIVER_PROFILING ?= n

OUTPUT_DIR ?= ../Output

LIB_NAME = libtiOsLib

ifeq ($(CUDK_ROOT), "")
	CUDK_ROOT = ..
endif

ifeq ($(DEBUG),y)
  DEBUGFLAGS = -O0 -g -fno-builtin -DDEBUG -D TI_DBG  # "-O" is needed to expand inlines
else
  DEBUGFLAGS = -O2
endif
DEBUGFLAGS+= -DHOST_COMPILE


ifeq ($(STATIC_LIB),y)
	TARGET = $(OUTPUT_DIR)/$(LIB_NAME).a
else
	TARGET = $(OUTPUT_DIR)/$(LIB_NAME).so
endif

CUDK_ROOT ?= ..
IPC_DIR = $(CUDK_ROOT)/IPC
OAL_DIR = $(CUDK_ROOT)/OAL
CROSS_COMPILE ?= arm_v4t_le-

SRCS = $(OAL_DIR)/Common/TI_OAL.cpp \
       $(OAL_DIR)/Pform/Linux/TILibLinux.cpp \
       $(CUDK_ROOT)/UtilityAdapter/Linux/IpcWithTISMLinux.cpp \
       $(CUDK_ROOT)/UtilityAdapter/TI_AdapterApi.cpp \
       $(CUDK_ROOT)/UtilityAdapter/TI_Adapter.cpp \
       $(CUDK_ROOT)/UtilityAdapter/CTI_Adapter.cpp \
       $(CUDK_ROOT)/UtilityAdapter/TI_IPC_Suppl.cpp \
       $(IPC_DIR)/Linux/ipc_event.c \
       $(IPC_DIR)/Linux/cu_ipc.c \
       $(IPC_DIR)/Linux/ipc_user.c

ifeq ($(EXC), y)
SRCS += $(CUDK_ROOT)/UtilityAdapter/EXC/CTI_AdapterEXC.cpp \
        $(CUDK_ROOT)/UtilityAdapter/EXC/TI_AdapterApiEXC.cpp

CFLAGS := -D EXC_MODULE_INCLUDED
endif

ifeq ($(SG), y)
CFLAGS += -D BTH_COEXISTENCE
endif

ifeq ($(DRIVER_PROFILING),y)
   CFLAGS += -D DRIVER_PROFILING
endif

#ARMFLAGS  = -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -Uarm -fno-common -pipe
#ARMFLAGS += -mapcs -mno-sched-prolog -mapcs-32 -D__LINUX_ARM_ARCH__=4 -march=armv4
#ARMFLAGS += -mtune=arm9tdmi -mshort-load-bytes -msoft-float  -iwithprefix include

ARMFLAGS  = -fno-common -pipe


INCLUDEDIR = -I $(CUDK_ROOT)/Inc \
	-I $(CUDK_ROOT)/OAL/Common \
	-I $(CUDK_ROOT)/UtilityAdapter \
	-I $(CUDK_ROOT)/../common/inc \
	-I $(CUDK_ROOT)/../common/src/hal/FirmwareApi \
	-I $(CUDK_ROOT)/TISMgr \
	-I $(CUDK_ROOT)/../pform/linux/inc \
	-I $(CUDK_ROOT)/../pform/common/inc \
	-I $(CUDK_ROOT)/IPC/Linux \
	-I $(CUDK_ROOT)/CLI \
	-I $(CUDK_ROOT)/CLI/drv_inc \
	-I $(CUDK_ROOT)/CLI/pform_inc

ifeq ($(EXC),y)
        INCLUDEDIR += \
        -I $(CUDK_ROOT)/../common/inc/EXC \
        -I $(CUDK_ROOT)/Inc/EXC
endif
	
	
	

CFLAGS += -Wall -D__LINUX__ -D_IPC_SUPPL -DTI_EMBEDDED_SUPPLICANT
CFLAGS += $(DEBUGFLAGS)
CFLAGS += $(INCLUDEDIR) $(ARMFLAGS)

OBJS = $(patsubst %.c,%.o,$(patsubst %.cpp,%.o,$(SRCS)))

DEPS = $(OBJS:%.o=%.d)

all: $(TARGET)

$(OUTPUT_DIR)/libtiOsLib.a: $(OBJS)
	rm -f $@
	$(CROSS_COMPILE)$(AR) -q $@ $^

$(OUTPUT_DIR)/libtiOsLib.so: $(OBJS)
	rm -f $@
	$(CROSS_COMPILE)gcc -shared $(CFLAGS) $(OBJS) -lc -o $@
	$(CROSS_COMPILE)strip $@

ifeq ($(findstring "clean", $(MAKECMDGOALS)), "")
-include .depend
endif

%.o: %.cpp
	@echo $@
	@$(CROSS_COMPILE)gcc -c $(CFLAGS) $< -o $@

%.o: %.c
	@echo $@
	@$(CROSS_COMPILE)gcc -c $(CFLAGS) $< -o $@

%.i: %.cpp %.c
	$(CROSS_COMPILE)gcc -E -v -c $(CFLAGS) $< -o $@

%.s: %.cpp %.c
	$(CROSS_COMPILE)gcc -save-temps -c $(CFLAGS) $< -o $@

%.d: %.cpp
	@echo $@
	@$(CROSS_COMPILE)gcc $< -MT$(^:%.cpp=%.o) -M $(CFLAGS) > $@

%.d: %.c
	@echo $@
	@$(CROSS_COMPILE)gcc $< -MT$(^:%.c=%.o) -M $(CFLAGS) > $@

.depend: $(DEPS)
	rm -f $@
	for aa in $^; do cat $$aa >> $@; done

clean:
	@rm -f .depend $(OBJS) $(DEPS) $(OUTPUT_DIR)/libtiOsLib.a $(OUTPUT_DIR)/libtiOsLib.so

TAGS: $(SRCS)
	{ find ${BASE_DIR}/common -name '*.h' -print ; \
	  find ${BASE_DIR}/Test -name '*.h' -print ; \
	  find . -name '*.h' -print ; } | etags -
	  etags -a $(SRCS)
