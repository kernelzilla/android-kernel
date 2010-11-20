#
# Makefile for the Linux Bluetooth HCI device drivers.
#

obj-$(CONFIG_BT_HCIVHCI)	+= hci_vhci.o
obj-$(CONFIG_BT_HCIUART)	+= hci_uart.o
obj-$(CONFIG_BT_HCIBCM203X)	+= bcm203x.o
obj-$(CONFIG_BT_HCIBPA10X)	+= bpa10x.o
obj-$(CONFIG_BT_HCIBFUSB)	+= bfusb.o
obj-$(CONFIG_BT_HCIDTL1)	+= dtl1_cs.o
obj-$(CONFIG_BT_HCIBT3C)	+= bt3c_cs.o
obj-$(CONFIG_BT_HCIBLUECARD)	+= bluecard_cs.o
obj-$(CONFIG_BT_HCIBTUART)	+= btuart_cs.o
obj-$(CONFIG_BT_HCIBRF6150)	+= brf6150.o
obj-$(CONFIG_BT_HCIH4P)		+= hci_h4p/

obj-$(CONFIG_BT_HCIBTUSB)	+= btusb.o
obj-$(CONFIG_BT_HCIBTSDIO)	+= btsdio.o

hci_uart-y				:= hci_ldisc.o
hci_uart-$(CONFIG_BT_HCIUART_H4)	+= hci_h4.o
hci_uart-$(CONFIG_BT_HCIUART_BCSP)	+= hci_bcsp.o
hci_uart-$(CONFIG_BT_HCIUART_LL)	+= hci_ll.o
hci_uart-$(CONFIG_BT_HCIUART_BRCM)	+= hci_brcm.o
hci_uart-objs				:= $(hci_uart-y)
obj-$(CONFIG_BT_MSM_SLEEP)              += msm_bt_sleep.o
msm_bt_sleep-objs                       := bluesleep.o

ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
obj-$(CONFIG_BT_HCIUART)	        += bt_power.o
else ifeq ($(CONFIG_MACH_MOT),y)
obj-$(CONFIG_BT_HCIUART)                += bt_power.o
else
obj-$(CONFIG_MSM_BT_POWER)		+= bluetooth-power.o
endif

ifeq ($(CONFIG_KERNEL_MOTOROLA),y)
EXTRA_CFLAGS += -Iarch/arm/mach-msm
endif #ifeq ($(CONFIG_KERNEL_MOTOROLA),y)

