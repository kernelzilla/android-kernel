ifeq ($(CONFIG_MSM_AMSS_SUPPORT_256MB_EBI1),y)
zreladdr-y		:= 0x19208000
params_phys-y		:= 0x19200100
initrd_phys-y		:= 0x19A00000
else
ifeq ($(CONFIG_MSM_AMSS_RADIO2708_MEMMAP),y)
zreladdr-y          := 0x02008000
params_phys-y       := 0x02000100
initrd_phys-y       := 0x02800000
else
zreladdr-y		:= 0x10008000
params_phys-y		:= 0x10000100
initrd_phys-y		:= 0x10800000
endif
endif

# for now, override for QSD8x50
  zreladdr-$(CONFIG_ARCH_QSD8X50)		:= 0x20008000
params_phys-$(CONFIG_ARCH_QSD8X50)		:= 0x20000100
initrd_phys-$(CONFIG_ARCH_QSD8X50)		:= 0x21000000
