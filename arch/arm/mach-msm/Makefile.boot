ifeq ($(CONFIG_ARCH_QSD),y)
  zreladdr-y		:= 0x16008000
params_phys-y		:= 0x16000100
initrd_phys-y		:= 0x1A000000
else  # !CONFIG_ARCH_QSD
ifeq ($(CONFIG_MSM_STACKED_MEMORY), y)
  zreladdr-y		:= 0x10008000
params_phys-y		:= 0x10000100
initrd_phys-y		:= 0x10800000
else  # !CONFIG_MSM_STACKED_MEMORY
  zreladdr-y		:= 0x00208000
params_phys-y		:= 0x00200100
initrd_phys-y		:= 0x0A000000
endif # CONFIG_MSM_STACKED_MEMORY
endif # CONFIG_ARCH_QSD
