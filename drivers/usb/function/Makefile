
obj-$(CONFIG_USB_MSM_OTG)		+= msm_otg.o
obj-$(CONFIG_USB_FUNCTION_MSM_HSUSB)	+= msm_hsusb.o
obj-$(CONFIG_USB_FUNCTION_NULL)		+= null.o
obj-$(CONFIG_USB_FUNCTION_NULL)		+= zero.o
obj-$(CONFIG_USB_FUNCTION_LOOPBACK)	+= loopback.o
obj-$(CONFIG_USB_FUNCTION_ADB)		+= adb.o
obj-$(CONFIG_USB_FUNCTION_UMS)		+= ums.o
obj-$(CONFIG_USB_FUNCTION_MASS_STORAGE)	+= mass_storage.o
obj-$(CONFIG_USB_FUNCTION_DIAG)		+= diag.o
obj-$(CONFIG_USB_FUNCTION_SERIAL)       += serial.o
ifeq (y, $(findstring y, $(CONFIG_KERNEL_MOTOROLA) $(CONFIG_MACH_MOT)))
obj-$(CONFIG_USB_FUNCTION_ETHER)	+= ether.o
else #ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
obj-$(CONFIG_USB_FUNCTION_ETHER)	+= ether_cdc_ecm.o
endif #ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
