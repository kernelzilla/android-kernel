  zreladdr-y		:= 0x80c08000
params_phys-y		:= 0x80c00100
initrd_phys-y		:= 0x81400000

  zreladdr-$(CONFIG_MACH_SHOLES) := 0x80c08000
params_phys-$(CONFIG_MACH_SHOLES) := 0x80c00100
initrd_phys-$(CONFIG_MACH_SHOLES) := 0x81400000

