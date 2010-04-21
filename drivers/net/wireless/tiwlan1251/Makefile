##
## Flags
##
export FIRMWARE_DYNAMIC_LOAD ?= y

##
##
## File lists and locations
##
##

#
# DK_ROOT must be set prior to including common.inc
#
DK_ROOT := .
ifeq ($(M),)
    M = drivers/net/wireless/tiwlan1251
endif
DK_ROOT_INC = $(M)

#
# Includes common definitions and source file list
#
ifneq ($(KERNELRELEASE),)
    include $(M)/common.inc
    include $(M)/drv_sources.inc
    include $(M)/os_sources.inc
else
ifeq ($(KERNEL_DIR),)
    KERNEL_DIR = ../../../../
endif
    include $(M)/common.inc
    include $(M)/drv_sources.inc
    include $(M)/os_sources.inc
endif

#
# OS include paths required for compilation.
# 
OS_INCS = $(DK_ROOT)/pform/linux/inc $(DK_ROOT)/pform/common/inc
OS_INCS += $(DK_ROOT)/common/src/core/EvHandler

#
# Location and filename of the driver .lib file created by this makefile.
#
OUTPUT_DIR = $(DK_ROOT)
OUTPUT_FILE = $(OUTPUT_DIR)/libestadrv.a

OS_SRCS = \
    $(DK_ROOT)/pform/common/src/osCmd.c \
    $(DK_ROOT)/pform/common/src/osUtil.c \
    $(DK_ROOT)/pform/common/src/osClsfr.c \
    $(DK_ROOT)/pform/common/src/osRgstry.c \
    $(DK_ROOT)/pform/linux/src/esta_drv.c \
    $(DK_ROOT)/pform/linux/src/chip_stat.c \
    $(DK_ROOT)/pform/linux/src/mmc_tnetw1150_api.c \
    $(DK_ROOT)/pform/linux/src/tnetw_sdio.c \
    $(DK_ROOT)/pform/linux/src/osapi.c \
    $(DK_ROOT)/pform/linux/src/osmemapi.c \
    $(DK_ROOT)/pform/linux/src/env_tst.c \
    $(DK_ROOT)/pform/linux/src/ioctl_list.c \
    $(DK_ROOT)/pform/linux/src/ioctl_utils.c \
    $(DK_ROOT)/pform/linux/src/osRgstry_parser.c \
    $(DK_ROOT)/pform/linux/src/ipc_k.c \
    $(DK_ROOT)/pform/linux/src/proc_stat.c
    
ifeq ($(DRIVER_PROFILING),y)
    OS_SRCS += $(DK_ROOT)/pform/linux/src/tiwlan_profile.c
endif

ifeq ($(STACK_PROFILING_ON),y)
    OS_SRCS += $(DK_ROOT)/pform/linux/src/stack_profile.c
    EXTRA_CFLAGS += -D STACK_PROFILE
endif

ifeq ($(NO_ARCH_STRCMP),y)
    OS_SRCS += $(DK_ROOT)/pform/linux/src/string.c
endif
    
OS_OBJS = $(patsubst %.c, %.o, $(OS_SRCS))

#
# Remove debug info
#
EXTRA_LDFLAGS += --strip-debug 

##
##
## Build process
##
##

ifneq ($(KERNELRELEASE),)


	##
	##
	## This is the kernel build phase - set the appropriate arguments
	##
	##

	#
	# Adds the current directory as a prefix to all include directories.
	#
	EXTRA_CFLAGS += $(addprefix -I$(DK_ROOT_INC)/, $(DK_INCS) $(OS_INCS))

	#
	# Intermediate object name - this should be converted to the appropriate library file
	# after the kernel makefile finishes its work.
	#
	obj-m += wlan.o
	
	#
	# List of object files the kernel makefile needs to compile.
	#
	wlan-objs := $(DK_OBJS) $(OS_OBJS)

	
else	# ifneq ($(KERNELRELEASE),)

	#
	# Adds the current directory as a prefix to all include directories.
	#
	EXTRA_CFLAGS += $(addprefix -I$(M)/, $(DK_INCS) $(OS_INCS))


##
##
## This is the regular build phase - act according to the make actions
##
##

#
# The location of the kernel makefile
#
KERNEL_DIR ?= /vobs/wlan_linux2.6/kernel-2.6/


#
# Build the driver lib file
#
.PHONY: all
all: .depend $(OUTPUT_FILE)


#
# Prints variables
#
.PHONY: help
help:
	@echo Default Compilation:	BOARD=$(WIFI_BOARD) PLATFORM=$(PLATFORM) DEBUG=$(DEBUG) INTR=$(INTR) WSPI=$(WSPI) EXC=$(EXC) EXTRA CFLAGS: $(EXTRA_CFLAGS)


#
# Recursively cleans the driver files.
#
.PHONY: clean
clean:
	$(MAKE) -C $(KERNEL_DIR) M=`pwd` ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) clean
	@rm -f *.o *.a *~ *.~* core .depend dep $(DK_OBJS) $(OS_OBJS) $(DEPS)


#
# Causes the library file to get rebuilt.
#
.depend:
	rm -f $(OUTPUT_FILE)


#
# Recursively builds the library file.
#
$(OUTPUT_FILE):
	$(MAKE) -C $(KERNEL_DIR) M=`pwd` ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE) modules
	@$(CROSS_COMPILE)$(AR) rcs $@ $(DK_OBJS) $(OS_OBJS)
	$(CROSS_COMPILE)strip --strip-unneeded wlan.ko

endif	# ifneq ($(KERNELRELEASE),)
