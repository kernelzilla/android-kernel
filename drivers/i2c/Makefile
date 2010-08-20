#
# Makefile for the i2c core.
#

ifeq ($(CONFIG_ARM),y)
# Remove the -Werror until i2c deprecated functions are fixed
CFLAGS_REMOVE_i2c-core.o = -Werror
endif

obj-$(CONFIG_I2C_BOARDINFO)	+= i2c-boardinfo.o
obj-$(CONFIG_I2C)		+= i2c-core.o
obj-$(CONFIG_I2C_CHARDEV)	+= i2c-dev.o
obj-y				+= busses/ chips/ algos/

ifeq ($(CONFIG_I2C_DEBUG_CORE),y)
EXTRA_CFLAGS += -DDEBUG
endif
