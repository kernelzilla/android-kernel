#
# Makefile for misc devices that really don't fit anywhere else.
#

obj-$(CONFIG_IBM_ASM)		+= ibmasm/
obj-$(CONFIG_HDPU_FEATURES)	+= hdpuftrs/
obj-$(CONFIG_ATMEL_PWM)		+= atmel_pwm.o
obj-$(CONFIG_ATMEL_SSC)		+= atmel-ssc.o
obj-$(CONFIG_ATMEL_TCLIB)	+= atmel_tclib.o
obj-$(CONFIG_ICS932S401)	+= ics932s401.o
obj-$(CONFIG_LKDTM)		+= lkdtm.o
obj-$(CONFIG_TIFM_CORE)       	+= tifm_core.o
obj-$(CONFIG_TIFM_7XX1)       	+= tifm_7xx1.o
obj-$(CONFIG_PHANTOM)		+= phantom.o
obj-$(CONFIG_ANDROID_PMEM)	+= pmem.o
obj-$(CONFIG_ANDROID_PMEM_KAPI_TEST)	+= pmem_kernel_test.o
obj-$(CONFIG_SGI_IOC4)		+= ioc4.o
obj-$(CONFIG_OMAP_STI)		+= sti/
obj-$(CONFIG_ENCLOSURE_SERVICES) += enclosure.o
obj-$(CONFIG_KERNEL_DEBUGGER_CORE)	+= kernel_debugger.o
obj-$(CONFIG_KGDB_TESTS)	+= kgdbts.o
obj-$(CONFIG_SGI_XP)		+= sgi-xp/
obj-$(CONFIG_SGI_GRU)		+= sgi-gru/
obj-$(CONFIG_HP_ILO)		+= hpilo.o
obj-$(CONFIG_UID_STAT)		+= uid_stat.o
obj-$(CONFIG_C2PORT)		+= c2port/
obj-$(CONFIG_SENSORS_AKM8973)	+= akm8973.o
obj-$(CONFIG_SENSORS_AKM8973_AKMD)	+= akm8973_akmd.o
ifneq (y, $(findstring y, $(CONFIG_MACH_CALGARY) $(CONFIG_MACH_MOT)))
obj-$(CONFIG_SENSORS_LIS331DLH)	+= lis331dlh.o
endif
obj-$(CONFIG_SENSORS_KXTF9)	+= kxtf9.o
obj-$(CONFIG_SENSORS_AIRC)	+= airc.o

obj-y				+= eeprom/
obj-y				+= ts27010mux/
obj-$(CONFIG_VIB_PWM)		+= vib-pwm.o
obj-$(CONFIG_NETMUX_DRIVER)	+= netmux/
obj-$(CONFIG_NETMUX_LINKDRIVER)	+= netmux_linkdriver/
obj-$(CONFIG_UMTS_SYSPANIC)     += syspanic.o
obj-$(CONFIG_OMAP_MODEM_CONTROL)       += omap_mdm_ctrl.o
obj-$(CONFIG_VIB_GPIO)		+= vib-gpio.o
obj-$(CONFIG_TSIF) += msm_tsif.o
msm_tsif-objs := tsif.o
obj-$(CONFIG_APANIC)           += apanic.o
obj-$(CONFIG_MOT_CAMERA_MISC)   += mot_camera_misc/

obj-$(CONFIG_TSIF_CHRDEV) += tsif_chrdev.o

obj-$(CONFIG_WL127X_RFKILL)	+= wl127x-rfkill.o
obj-$(CONFIG_WL127X_TEST)	+= wl127x-test.o
obj-$(CONFIG_MODEM_PM_DRIVER)   += modem_pm_driver/
obj-$(CONFIG_SMC_DRIVER)	+= smc/
obj-$(CONFIG_SEC_DRIVER)        += sec/
obj-$(CONFIG_ARCH_OMAP3)        += clk_32k_rw.o
obj-$(CONFIG_APANIC)		+= apanic.o
obj-$(CONFIG_MOT_CAMERA_MISC)   += mot_camera_misc/
obj-$(CONFIG_MOT_CAMERA_MISC_MSM) += mot_camera_misc_msm/
