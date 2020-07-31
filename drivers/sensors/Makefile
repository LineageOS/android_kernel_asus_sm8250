
# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_SENSORS_SSC)	+= sensors_ssc.o
obj-$(CONFIG_SENSORS_ICM206XX)	+= icm206xx.o
obj-y                           += ASH/
ifdef CONFIG_MACH_ASUS_ZS661KS
obj-y				+= snt8100fsr-linux/
ifeq ($(ASUS_FTM), y)
KBUILD_CFLAGS += -DFACTORY_FLAG=1
endif
KBUILD_CFLAGS += -DZS661KS_PL
endif
