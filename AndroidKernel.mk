#Android makefile to build kernel as a part of Android Build

ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- $(KERNEL_DEFCONFIG)
ifeq ($(MOT_MAKEFILE_CHANGES), TRUE)
	@if [ "1" -lt "$$(diff --side-by-side --suppress-common-lines $(KERNEL_CONFIG) kernel/arch/*/configs/$(KERNEL_DEFCONFIG) | wc -l)" ] ; \
	then \
	    echo "ERROR: The output config file differs from the default config file. If you need to modify the config, use the \"kernelconfig\" target." ; \
	    echo "       Copying output config to $(KERNEL_CONFIG).bak." ; \
	    cp $(KERNEL_CONFIG) $(KERNEL_CONFIG).bak ; \
	    exit 1 ; \
	fi
endif #ifeq ($(MOT_MAKEFILE_CHANGES), TRUE)

$(TARGET_PREBUILT_KERNEL): $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-

kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi- menuconfig
	cp $(KERNEL_OUT)/.config kernel/arch/arm/configs/$(KERNEL_DEFCONFIG)

endif
