#
# Makefile for miscellaneous I2C chip drivers.
#
# Think twice before you add a new driver to this directory.
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_DS1682)		+= ds1682.o
obj-$(CONFIG_AT24)		+= at24.o
obj-$(CONFIG_SENSORS_EEPROM)	+= eeprom.o
obj-$(CONFIG_SENSORS_MAX6875)	+= max6875.o
obj-$(CONFIG_SENSORS_PCA9539)	+= pca9539.o
obj-$(CONFIG_SENSORS_PCA963X)	+= pca963x.o
obj-$(CONFIG_SENSORS_PCF8574)	+= pcf8574.o
obj-$(CONFIG_PCF8575)		+= pcf8575.o
obj-$(CONFIG_SENSORS_PCF8591)	+= pcf8591.o
obj-$(CONFIG_SENSORS_AKM8976)	+= akm8976.o
obj-$(CONFIG_TPS65010)		+= tps65010.o
obj-$(CONFIG_MENELAUS)		+= menelaus.o
obj-$(CONFIG_SENSORS_TSL2550)	+= tsl2550.o
obj-$(CONFIG_SENSORS_MT9T013)	+= mt9t013.o
obj-$(CONFIG_MAX7359_KEYPAD)	+= max7359_keypad.o
obj-$(CONFIG_ADP5588_KEYPAD)	+= adp5588_keypad.o
obj-$(CONFIG_SENSORS_KXPS5)	    += kionix/
obj-$(CONFIG_SENSORS_LIS331DLH)	+= stmicro/
obj-$(CONFIG_SENSORS_AK8973)	 += akm8973.o
obj-$(CONFIG_FUEL_GAUGE_BQ27505) += bq27505.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

