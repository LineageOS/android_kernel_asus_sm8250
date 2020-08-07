#
# Makefile for jedi dongle drivers.
#
obj-y		+= ec_hid_driver.o
obj-y		+= ec_hid_interface.o
#obj-m		+= ec_i2c_updatefw.o
obj-m		+= ec_i2c_interface.o
obj-m		+= station_key.o

ifeq ($(ASUS_FTM), y)
KBUILD_CFLAGS += -DASUS_FACTORY_BUILD=1
endif
