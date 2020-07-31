obj-n += HALLsensor.o
obj-y += ALSPSsensor.o
obj-n += psensor.o
obj-n += lsensor.o
obj-n += ALSPS_FRGBsensor.o
ifeq ($(ASUS_BUILD_PROJECT),ZS670KS)
obj-y += ALSPSsensor_2nd.o
endif
#obj-y += SARsensor.o

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_ALGO_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)
