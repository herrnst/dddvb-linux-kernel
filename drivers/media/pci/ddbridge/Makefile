# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the ddbridge device driver
#

ddbridge-objs := ddbridge-main.o ddbridge-core.o ddbridge-ci.o \
		ddbridge-hw.o ddbridge-i2c.o ddbridge-io.o ddbridge-max.o \
		ddbridge-mci.o ddbridge-sx8.o

obj-$(CONFIG_DVB_DDBRIDGE) += ddbridge.o ddbridge-dummy-fe.o

ccflags-y += -I $(srctree)/drivers/media/dvb-frontends/
ccflags-y += -I $(srctree)/drivers/media/tuners/
