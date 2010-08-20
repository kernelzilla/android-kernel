#Turn on kernel engineering build as default when TARGET_BUILD_VARIANT is eng, to disable it, add ENG_BLD=0 in build command
ifeq ($(TARGET_BUILD_VARIANT), eng)
ENG_BLD := 1
endif
ifeq ($(TARGET_BUILD_VARIANT), userdebug)
ENG_BLD := 1
endif

MOTO_MOD_INSTALL=$(TARGET_OUT)/lib/modules
KERNEL_OUT_DIR=$(PRODUCT_OUT)/obj/PARTITIONS/kernel_intermediates
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT_DIR)/build/arch/arm/boot/zImage
WLAN_DRV_PATH = $(TOPDIR)system/wlan/ti/wilink_6_1/platforms/os/linux
WLAN_AP_DRV_PATH = $(TOPDIR)system/wlan/ti/WiLink_AP/platforms/os/linux

ifneq ($(DO_NOT_REBUILD_THE_KERNEL),1)
.PHONY: $(TARGET_PREBUILT_KERNEL)
endif

ifeq ($(TARGET_PRODUCT), cdma_social)
BLD_CONF=msm7627_social
else
ifeq ($(TARGET_PRODUCT), cdma_ciena)
BLD_CONF=msm7625_ciena
else
BLD_CONF=mapphone
endif
endif

$(TARGET_PREBUILT_KERNEL) += $(MOTO_MOD_INSTALL)/dummy.ko
$(TARGET_PREBUILT_KERNEL): $(HOST_OUT_EXECUTABLES)/depmod$(HOST_EXECUTABLE_SUFFIX)
ifeq ($(TARGET_PRODUCT), cdma_social)
	make -f kernel/kernel.mk BLD_CONF=$(BLD_CONF) KERNEL_OUT_DIR=$(KERNEL_OUT_DIR) DEPMOD=$(HOST_OUT_EXECUTABLES)/depmod$(HOST_EXECUTABLE_SUFFIX) KERNEL_CROSS_COMPILE=$(shell pwd)/prebuilt/$(HOST_PREBUILT_TAG)/toolchain/arm-eabi-4.4.0/bin/arm-eabi- ENG_BLD=$(ENG_BLD) TARGET_PRODUCT=$(TARGET_PRODUCT)
else
ifeq ($(TARGET_PRODUCT), cdma_ciena)
	make -f kernel/kernel.mk BLD_CONF=$(BLD_CONF) KERNEL_OUT_DIR=$(KERNEL_OUT_DIR) DEPMOD=$(HOST_OUT_EXECUTABLES)/depmod$(HOST_EXECUTABLE_SUFFIX) KERNEL_CROSS_COMPILE=$(shell pwd)/prebuilt/$(HOST_PREBUILT_TAG)/toolchain/arm-eabi-4.4.0/bin/arm-eabi- ENG_BLD=$(ENG_BLD) TARGET_PRODUCT=$(TARGET_PRODUCT)
else
	make -f kernel/kernel.mk BLD_CONF=$(BLD_CONF) KERNEL_OUT_DIR=$(KERNEL_OUT_DIR) DEPMOD=$(HOST_OUT_EXECUTABLES)/depmod$(HOST_EXECUTABLE_SUFFIX) KERNEL_CROSS_COMPILE=$(shell pwd)/prebuilt/$(HOST_PREBUILT_TAG)/toolchain/arm-eabi-4.4.0/bin/arm-eabi- ENG_BLD=$(ENG_BLD) TARGET_PRODUCT=$(TARGET_PRODUCT)
	mkdir -p $(MOTO_MOD_INSTALL)
	rm -f $(MOTO_MOD_INSTALL)/dummy.ko
	find $(KERNEL_OUT_DIR)/build/lib/modules -name "*.ko" -exec cp -f {} \
		$(MOTO_MOD_INSTALL) \; || true
	cp $(WLAN_DRV_PATH)/tiwlan_drv.ko $(MOTO_MOD_INSTALL)
	cp $(WLAN_AP_DRV_PATH)/tiap_drv.ko $(MOTO_MOD_INSTALL)
	$(TOPDIR)prebuilt/$(HOST_OS)-$(HOST_ARCH)/toolchain/arm-eabi-4.4.0/bin/arm-eabi-strip --strip-debug $(MOTO_MOD_INSTALL)/*.ko
	touch $(MOTO_MOD_INSTALL)/dummy.ko
endif
endif

file := $(INSTALLED_KERNEL_TARGET)
ALL_PREBUILT += $(file)
$(file): $(TARGET_PREBUILT_KERNEL) | $(ACP)
	$(transform-prebuilt-to-target)

.PHONY: kernel
kernel: $(TARGET_PREBUILT_KERNEL)
