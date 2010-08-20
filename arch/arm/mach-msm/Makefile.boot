ifeq ($(CONFIG_ARCH_MSM_SCORPION),y)
ifeq ($(CONFIG_MSM_STACKED_MEMORY), y)
  zreladdr-y		:= 0x20008000
params_phys-y		:= 0x20000100
initrd_phys-y		:= 0x24000000
else  # !CONFIG_MSM_STACKED_MEMORY
  zreladdr-y		:= 0x00208000
params_phys-y		:= 0x00200100
initrd_phys-y		:= 0x01200000
endif # CONFIG_MSM_STACKED_MEMORY
else  # !CONFIG_ARCH_MSM_SCORPION
ifeq ($(CONFIG_MSM_STACKED_MEMORY), y)
  zreladdr-y		:= 0x10008000
params_phys-y		:= 0x10000100
initrd_phys-y		:= 0x10800000
else  # !CONFIG_MSM_STACKED_MEMORY
ifeq ($(CONFIG_MACH_CIENA),y)
  zreladdr-y		:= 0x00208000
params_phys-y		:= 0x00200100
initrd_phys-y		:= 0x0A000000
endif
ifeq ($(CONFIG_MACH_SOCIAL),y)
  zreladdr-y		:= 0x13608000
params_phys-y		:= 0x13600100
initrd_phys-y		:= 0x1D400000
endif
endif # CONFIG_MSM_STACKED_MEMORY
endif # CONFIG_ARCH_MSM_SCORPION
