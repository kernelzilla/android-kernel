#sms1xxx-objs := smscoreapi.o smsusb.o smsdvb.o sms-cards.o

#obj-$(CONFIG_DVB_SIANO_SMS1XXX) += sms1xxx.o

#EXTRA_CFLAGS += -Idrivers/media/dvb/dvb-core

#EXTRA_CFLAGS += $(extra-cflags-y) $(extra-cflags-m)
###############################################################################
#
# Siano Mobile Silicon, Inc.
# MDTV receiver kernel modules.
# Copyright (C) 2006-2008, Uri Shkolnik
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
###############################################################################

# enable/disable driver log
#DEBUG = y

#Local variables initailization
MODULE_NAME = smsmdtv

SMSOBJ := smscoreapi.o sms-cards.o smsendian.o smsir.o

EXTRA_CFLAGS += -DSMS_HOSTLIB_SUBSYS
SMSOBJ += smschar.o

EXTRA_CFLAGS += -DSMS_SPI_DRV
SMSOBJ += smsspilog.o smsspicommon.o smsspiphy_omap34xx.o

ifeq ($(DEBUG), y)
EXTRA_CFLAGS += -DENABLE_CONSOLE_DBG
endif

smsmdtv-objs := $(SMSOBJ)

obj-m := smsmdtv.o
