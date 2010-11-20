# Copyright (C) 2009 Motorola, Inc.
#####################################################################
#
# Script creation notes by: David Ding (dding@motorola.com)
# (THIS DESCRIPTION IS NOW OBSOLETE: SEE UPDATE BELOW)
#
# The intention of creating this script is for Moto-Android platform
# common Kernel and kernel modules developer to make kernel zImage and
# driver modules .ko objects. As long as it is in your execution $PATH
# you can place this script anywhere you may preferred. A suggestion
# place can be in $HOME/bin directory, then make PATH=$PATH:$HOME/bin
#
# How to use:
# -----------
# $ cd {top-moto-android-working-dir}
# $ build_kernel
#
# if you are in the wrong place to start your kernel/module build
# script will quit and reminder you go to the RIGHT place to build
#
# UPDATE: 11/21/2009: wqnt78
#
# This makefile is now invoked from kernel/Android.mk. You may build
# the kernel and modules by using the "kernel" target:
#
# source build/envsetup.sh
# lunch
# make kernel
#
# It is also invoked automatically as part of the default target.
#
######################################################################
#set -x

PWD=$(shell pwd)

ifeq ($(DEPMOD),)
	DEPMOD := `which depmod 2> /dev/null || echo $(PWD)/out/host/linux-x86/bin/depmod`
endif

ifeq ($(KERNEL_CROSS_COMPILE),)
	KERNEL_CROSS_COMPILE=$(PWD)/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin/arm-eabi-
endif

ifeq ($(TARGET_PRODUCT),)
	TARGET_PRODUCT := generic
	MOTO_PREBUILT_DIR := $(PWD)/motorola/bsp/prebuilt/target/images
endif

ifeq ($(KERNEL_OUT_DIR),)
	KERNEL_OUT_DIR := out/target/product/$(TARGET_PRODUCT)/obj/PARTITIONS/kernel_intermediates
endif

TOPDIR=$(PWD)
KERNEL_CONF_OUT_DIR=$(PWD)/$(KERNEL_OUT_DIR)
KERNEL_BUILD_DIR=$(KERNEL_CONF_OUT_DIR)/build
KERNEL_SRC_DIR=$(PWD)/kernel

DEFCONFIGSRC        := ${KERNEL_SRC_DIR}/arch/arm/configs
LJAPDEFCONFIGSRC    := ${DEFCONFIGSRC}/ext_config
PRODUCT_SPECIFIC_DEFCONFIGS := \
    $(DEFCONFIGSRC)/mapphone_defconfig
_TARGET_DEFCONFIG := __ext_mapphone_defconfig
TARGET_DEFCONFIG := $(DEFCONFIGSRC)/$(_TARGET_DEFCONFIG)

WARN_FILTER := $(KERNEL_SRC_DIR)/scripts/gcc_warn_filter.cfg
KERNEL_ERR_LOG := $(KERNEL_CONF_OUT_DIR)/.kbld_err_log.txt
KFLAG := $(KERNEL_CONF_OUT_DIR)/.kbld_ok.txt
FFLAG := $(KERNEL_CONF_OUT_DIR)/.filter_ok.txt

all: inst_hook config kernel_and_modules modules_install ext_modules

inst_hook: $(KERNEL_SRC_DIR)/.git/hooks/pre-commit $(KERNEL_SRC_DIR)/.git/hooks/checkpatch.pl

$(KERNEL_SRC_DIR)/.git/hooks/pre-commit: $(KERNEL_SRC_DIR)/scripts/pre-commit
	@-cp -f $< $@
	@-chmod ugo+x $@

$(KERNEL_SRC_DIR)/.git/hooks/checkpatch.pl: $(KERNEL_SRC_DIR)/scripts/checkpatch.pl
	@-cp -f $< $@
	@-chmod ugo+x $@

ifneq ($(BLD_CONF),)
PRODUCT_SPECIFIC_DEFCONFIGS := $(DEFCONFIGSRC)/$(BLD_CONF)_defconfig
endif

ifneq ($(PRODUCT),)
PRODUCT_SPECIFIC_DEFCONFIGS += \
    ${LJAPDEFCONFIGSRC}/product/${PRODUCT}.config
endif

ifeq ($(ENG_BLD), 1)
PRODUCT_SPECIFIC_DEFCONFIGS += \
    ${LJAPDEFCONFIGSRC}/eng_bld.config
endif

ifeq ($(TEST_DRV_CER), 1)
        ifeq ($(TEST_COVERAGE),)
                TEST_COVERAGE=1
        endif

        ifeq ($(TEST_KMEMLEAK),)
                TEST_KMEMLEAK=1
        endif

        ifeq ($(TEST_FAULTINJECT),)
                TEST_FAULTINJECT=1
        endif
endif

# Option to enable or disable gcov
ifeq ($(TEST_COVERAGE),1)
        PRODUCT_SPECIFIC_DEFCONFIGS += \
			${LJAPDEFCONFIGSRC}/feature/coverage.config
endif

ifeq ($(TEST_KMEMLEAK),1)
        PRODUCT_SPECIFIC_DEFCONFIGS += \
			${LJAPDEFCONFIGSRC}/feature/kmemleak.config
endif

ifeq ($(TEST_FAULTINJECT),1)
        PRODUCT_SPECIFIC_DEFCONFIGS += \
			${LJAPDEFCONFIGSRC}/feature/faultinject.config
endif

ifeq ($(TEST_MUDFLAP),1)
         PRODUCT_SPECIFIC_DEFCONFIGS += \
            ${LJAPDEFCONFIGSRC}/feature/mudflap.config
endif

#
# make kernel configuration
#--------------------------
config:
	( perl -le 'print "# This file was automatically generated from:\n#\t" . join("\n#\t", @ARGV) . "\n"' $(PRODUCT_SPECIFIC_DEFCONFIGS) && cat $(PRODUCT_SPECIFIC_DEFCONFIGS) ) > $(TARGET_DEFCONFIG) || ( rm -f $@ && false )
	mkdir -p $(KERNEL_BUILD_DIR)
	make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) \
		KBUILD_DEFCONFIG=$(_TARGET_DEFCONFIG) \
		defconfig modules_prepare

$(WARN_FILTER):

$(FFLAG): $(WARN_FILTER)
	@echo "Gcc warning filter changed, clean build will be enforced\n"
	@make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
                 CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
                 O=$(KERNEL_BUILD_DIR) clean
	@touch $(FFLAG)
#
# build kernel and internal kernel modules
# ========================================
# We need to check warning no matter if build passed, failed or interuptted

kernel_and_modules: $(FFLAG)
	-rm -f $(KFLAG) 2> /dev/null
	((make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) O=$(KERNEL_BUILD_DIR) \
		DEPMOD=$(DEPMOD) INSTALL_MOD_PATH=$(KERNEL_BUILD_DIR) \
		-j4 zImage modules) \
		3>&1 1>&2 2>&3 && (touch $(KFLAG))) \
		| tee $(KERNEL_ERR_LOG)
ifneq ($(TEST_MUDFLAP),1)
	@(cat $(KERNEL_ERR_LOG) | \
		$(KERNEL_SRC_DIR)/scripts/chk_gcc_warn.pl $(KERNEL_SRC_DIR) \
			$(WARN_FILTER)) \
	 || (find $(KERNEL_BUILD_DIR) -name "*.ko" -exec rm -f {}  \; \
		&& rm -f $(KERNEL_ERR_LOG) && false)
endif
	cp -f $(KERNEL_BUILD_DIR)/arch/arm/boot/zImage $(MOTO_PREBUILT_DIR)
	rm $(KFLAG) 2> /dev/null

# To build modules (.ko) in specific folder
# It is useful for build specific module with extra options
# (e.g. TEST_DRV_CER)
dir:
	make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) $(DIR_TO_BLD)

#NOTE: "strip" MUST be done for generated .ko files!!!
modules_install:
	make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) \
		DEPMOD=$(DEPMOD) \
		INSTALL_MOD_PATH=$(KERNEL_BUILD_DIR) \
		modules_install
ifeq ($(TARGET_PRODUCT),generic)
	mkdir -p $(MOTO_PREBUILT_DIR)/system/lib/modules
	cp -f `find $(KERNEL_BUILD_DIR)/lib/modules -name "*.ko"` $(MOTO_PREBUILT_DIR)/system/lib/modules
endif

clean: ext_modules_clean kernel_clean
kernel_clean:
	make -C $(KERNEL_SRC_DIR) ARCH=arm $(KERN_FLAGS) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		O=$(KERNEL_BUILD_DIR) mrproper
	rm -f $(TARGET_DEFCONFIG)
	@rm -f $(KERNEL_CONF_OUT_DIR)/.*.txt

#
#----------------------------------------------------------------------------
# To use "make ext_modules" to buld external kernel modules
#----------------------------------------------------------------------------
# build external kernel modules
#
# NOTE: "strip" MUST be done for generated .ko files!!!
# =============================
ifeq ($(TARGET_PRODUCT),cdma_pittsburgh)
ext_modules:
else
ifeq ($(TARGET_PRODUCT),morrison)
ext_modules:
else # mapphone

ext_modules: tiwlan_drv

# TODO:
# ext_modules_clean doesn't work
# wlan, graphic, SMC drivers need to be updated to fix it
ext_modules_clean: tiwlan_drv_clean

# wlan driver module
#-------------------
#API_MAKE = env -u MAKECMDGOALS make PREFIX=$(KERNEL_BUILD_DIR) \

API_MAKE = make PREFIX=$(KERNEL_BUILD_DIR) \
		CROSS=$(KERNEL_CROSS_COMPILE) \
		CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) \
		PROCFAMILY=OMAP_3430 PROJROOT=$(PWD) \
		HOST_PLATFORM=zoom2 \
		KRNLSRC=$(KERNEL_SRC_DIR) KERNEL_DIR=$(KERNEL_BUILD_DIR)
WLAN_DRV_PATH = $(PWD)/system/wlan/ti/wilink_6_1/platforms/os/linux
tiwlan_drv:
	$(API_MAKE) -C $(WLAN_DRV_PATH)
ifeq ($(TARGET_PRODUCT),generic)
	cp $(WLAN_DRV_PATH)/tiwlan_drv.ko $(MOTO_PREBUILT_DIR)/system/lib/modules
endif

tiwlan_drv_clean:
	$(API_MAKE) -C $(WLAN_DRV_PATH) clean
endif # mapphone
endif
