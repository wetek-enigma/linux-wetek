#
# Makefile for the WetekPlay driver.
#

obj-$(CONFIG_WETEK) += wetekplay.o

wetekplay-objs = nimdetect.o mxl603.o cxd2837.o avl6211.o mn88436.o

EXTRA_CFLAGS += -Idrivers/media/dvb-core -Idrivers/media/tuners
EXTRA_CFLAGS += -I.
