
# LED Core
obj-$(CONFIG_NEW_LEDS)			+= led-core.o
obj-$(CONFIG_LEDS_CLASS)		+= led-class.o
obj-$(CONFIG_LEDS_TRIGGERS)		+= led-triggers.o

# LED Platform Drivers
obj-$(CONFIG_LEDS_ATMEL_PWM)		+= leds-atmel-pwm.o
obj-$(CONFIG_LEDS_LOCOMO)		+= leds-locomo.o
obj-$(CONFIG_LEDS_S3C24XX)		+= leds-s3c24xx.o
obj-$(CONFIG_LEDS_AMS_DELTA)		+= leds-ams-delta.o
obj-$(CONFIG_LEDS_NET48XX)		+= leds-net48xx.o
obj-$(CONFIG_LEDS_WRAP)			+= leds-wrap.o
obj-$(CONFIG_LEDS_OMAP)			+= leds-omap.o
obj-$(CONFIG_LEDS_OMAP_PWM)		+= leds-omap-pwm.o
obj-$(CONFIG_LEDS_H1940)		+= leds-h1940.o
obj-$(CONFIG_LEDS_COBALT_QUBE)		+= leds-cobalt-qube.o
obj-$(CONFIG_LEDS_COBALT_RAQ)		+= leds-cobalt-raq.o
obj-$(CONFIG_LEDS_SUNFIRE)		+= leds-sunfire.o
obj-$(CONFIG_LEDS_PCA9532)		+= leds-pca9532.o
obj-$(CONFIG_LEDS_GPIO)			+= leds-gpio.o
obj-$(CONFIG_LEDS_LP5521)		+= leds-lp5521.o
obj-$(CONFIG_LEDS_CPLD)			+= leds-cpld.o
obj-$(CONFIG_LEDS_CLEVO_MAIL)		+= leds-clevo-mail.o
obj-$(CONFIG_LEDS_HP6XX)		+= leds-hp6xx.o
obj-$(CONFIG_LEDS_FSG)			+= leds-fsg.o
obj-$(CONFIG_LEDS_PCA955X)		+= leds-pca955x.o
obj-$(CONFIG_LEDS_MSM_PMIC)		+= leds-msm-pmic.o
obj-$(CONFIG_LEDS_DA903X)		+= leds-da903x.o
obj-$(CONFIG_LEDS_WM8350)		+= leds-wm8350.o
obj-$(CONFIG_LEDS_BD7885)		+= leds-bd7885.o
obj-$(CONFIG_LEDS_BU9847)		+= leds-bu9847.o
obj-$(CONFIG_LEDS_AF_LED)		+= leds-ld-cpcap-afled.o
obj-$(CONFIG_LEDS_SHOLES)		+= leds-ld-cpcap-disp.o \
					   leds-ld-cpcap-rgb.o \
					   leds-ld-cpcap-kpad.o
obj-$(CONFIG_LEDS_BUTTON)		+= leds-cpcap-button.o
obj-$(CONFIG_LEDS_KPAD)			+= leds-cpcap-kpad.o
obj-$(CONFIG_LEDS_DISPLAY)		+= leds-cpcap-display.o
ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
obj-$(CONFIG_ADP8860_DISP_BL)            += leds-adp8860.o
obj-$(CONFIG_ADP8870_DISP_BL)            += leds-adp8870.o
obj-$(CONFIG_LEDS_LM3535)       	+= leds-lm3535.o
obj-$(CONFIG_ADP5588_LEDS_BL)           += leds-adp5588.o
obj-$(CONFIG_LEDS_FAN5646)              += leds-fan5646.o
endif #ifeq ($(CONFIG_KERNEL_MOTOROLA),y)

ifeq (y, $(findstring y, $(CONFIG_MACH_CALGARY) $(CONFIG_MACH_MOT)))
obj-$(CONFIG_LEDS_LM3535)               += leds-lm3535.o
endif

ifeq ($(CONFIG_MACH_MOT),y)
obj-$(CONFIG_LEDS_ADP8862)      += leds-adp8862.o
obj-$(CONFIG_LEDS_PM7540_BTN)   += leds-pm7540-btn.o
obj-$(CONFIG_LEDS_PM7540_SOC)   += leds-pm7540-soc.o
obj-$(CONFIG_LEDS_PM7540_KPD)   += leds-pm7540-kpd.o
obj-$(CONFIG_LEDS_MORRISON_KPD) += leds-morrison-kpd.o
obj-$(CONFIG_LEDS_CAMERA)       += leds-camera.o
endif

ifeq (y, $(findstring y, $(CONFIG_MACH_CALGARY) $(CONFIG_MACH_MOT)))
obj-$(CONFIG_LEDS_FAN5646)              += leds-fan5646.o
endif

ifeq ($(CONFIG_MACH_CALGARY),y)
obj-$(CONFIG_ADP5588_LEDS_BL)           += leds-adp5588.o
endif

# LED Triggers
obj-$(CONFIG_LEDS_TRIGGER_TIMER)	+= ledtrig-timer.o
obj-$(CONFIG_LEDS_TRIGGER_IDE_DISK)	+= ledtrig-ide-disk.o
obj-$(CONFIG_LEDS_TRIGGER_HEARTBEAT)	+= ledtrig-heartbeat.o
obj-$(CONFIG_LEDS_TRIGGER_BACKLIGHT)	+= ledtrig-backlight.o
obj-$(CONFIG_LEDS_TRIGGER_DEFAULT_ON)	+= ledtrig-default-on.o
obj-$(CONFIG_LEDS_TRIGGER_SLEEP)	+= ledtrig-sleep.o

obj-$(CONFIG_LEDS_LM3530) += led-lm3530.o
obj-$(CONFIG_LEDS_LM3554) += led-cpcap-lm3554.o
ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
EXTRA_CFLAGS += -Iarch/arm/mach-msm
endif #ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
