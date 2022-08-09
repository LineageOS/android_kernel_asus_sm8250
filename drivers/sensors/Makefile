
# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_SENSORS_SSC)	+= sensors_ssc.o
obj-$(CONFIG_SENSORS_ICM206XX)	+= icm206xx.o
obj-y                           += ASH/
ifeq ($(ASUS_BUILD_PROJECT),ZS661KS)
ifdef CONFIG_MACH_ASUS_ZS661KS
obj-y				+= snt8100fsr-linux/
KBUILD_CFLAGS += -DONE_PL_CHIP
KBUILD_CFLAGS += -DZS661KS_PROJECT
else #ZS670KS
KBUILD_CFLAGS += -DPSENSOR_CAL24 #selection=0 to choose 2/4 cm for psensor
KBUILD_CFLAGS += -DZS670KS_PROJECT
ifeq ($(ASUS_FTM), y)
KBUILD_CFLAGS += -DLONG_LENGTH_LOGN_IT_NON_PERF=1
else #Shipping
KBUILD_CFLAGS += -DLONG_LENGTH_LOGN_IT_PERF=1
endif #####ifdef ASUS_FTM
endif #####ifdef ZS661KS

ifeq ($(ASUS_FTM), y)
KBUILD_CFLAGS += -DFACTORY_FLAG=1
endif
endif
