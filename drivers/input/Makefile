#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_INPUT)		+= input-core.o
input-core-objs := input.o input-compat.o ff-core.o

obj-$(CONFIG_INPUT_FF_MEMLESS)	+= ff-memless.o
obj-$(CONFIG_INPUT_POLLDEV)	+= input-polldev.o

obj-$(CONFIG_INPUT_MOUSEDEV)	+= mousedev.o
obj-$(CONFIG_INPUT_JOYDEV)	+= joydev.o
obj-$(CONFIG_INPUT_EVDEV)	+= evdev.o
obj-$(CONFIG_INPUT_EVBUG)	+= evbug.o

obj-$(CONFIG_INPUT_KEYBOARD)	+= keyboard/
obj-$(CONFIG_INPUT_MOUSE)	+= mouse/
obj-$(CONFIG_INPUT_JOYSTICK)	+= joystick/
obj-$(CONFIG_INPUT_TABLET)	+= tablet/
obj-$(CONFIG_INPUT_TOUCHSCREEN)	+= touchscreen/
obj-$(CONFIG_INPUT_MISC)	+= misc/

obj-$(CONFIG_INPUT_APMPOWER)	+= apm-power.o

ifeq (y, $(findstring y, $(CONFIG_MACH_CALGARY) $(CONFIG_MACH_MOT)))
obj-$(CONFIG_INPUT_SFH7743)     += sfh7743.o
endif


obj-$(CONFIG_INPUT_KEYRESET)	+= keyreset.o

obj-$(CONFIG_XEN_KBDDEV_FRONTEND)	+= xen-kbdfront.o
ifeq ($(CONFIG_MACH_CALGARY),y)

EXTRA_CFLAGS += -I$(srctree)/arch/arm/mach-msm
endif
