#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_SENSORS_AKM8976)	+= akm8976.o
obj-$(CONFIG_DS1682)		+= ds1682.o
obj-$(CONFIG_SENSORS_MAX6875)	+= max6875.o
obj-$(CONFIG_SENSORS_PCA9539)	+= pca9539.o
obj-$(CONFIG_SENSORS_PCA963X)	+= pca963x.o
obj-$(CONFIG_SENSORS_PCF8574)	+= pcf8574.o
obj-$(CONFIG_PCF8575)		+= pcf8575.o
obj-$(CONFIG_SENSORS_PCF8591)	+= pcf8591.o
obj-$(CONFIG_SENSORS_TSL2550)	+= tsl2550.o
obj-$(CONFIG_TWL4030_POWEROFF)	+= twl4030-poweroff.o
obj-$(CONFIG_TWL4030_MADC)	+= twl4030-madc.o
obj-$(CONFIG_RTC_X1205_I2C)	+= x1205.o
obj-$(CONFIG_SENSORS_MT9T013)	+= mt9t013.o
ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
obj-$(CONFIG_ADP5588_KEYPAD)	+= adp5588_keypad.o
#obj-$(CONFIG_SENSORS_LIS331DLH)	+= stmicro/
#obj-$(CONFIG_SENSORS_LIS331DLH)	+= lis331dlh.o
obj-$(CONFIG_SENSORS_AK8973)	 += akm8973.o
obj-$(CONFIG_CRUCIALTEC_OJ)     += crucialtec_oj.o
endif #ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
obj-$(CONFIG_MOT_FEAT_LP3907)   += lp3907_i2c.o

ifeq (y, $(findstring y, $(CONFIG_MACH_CALGARY) $(CONFIG_MACH_MOT)))
obj-$(CONFIG_ADP5588_KEYPAD)    += adp5588_keypad.o
obj-$(CONFIG_SENSORS_LIS331DLH) += stmicro/
obj-$(CONFIG_SENSORS_AK8973)     += akm8973.o
endif

ifeq ($(CONFIG_MACH_MOT),y)
obj-$(CONFIG_FUEL_GAUGE_BQ27505) += bq27505.o
endif

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
EXTRA_CFLAGS += -Iarch/arm/mach-msm
endif #ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
