obj-y 	+= ALSPSsensor_Hardware.o
ifeq ($(ASUS_BUILD_PROJECT),ZS670KS)
obj-y	+= ALSPSsensor_Hardware_2nd.o
endif
obj-n	+= cm36686/
obj-n	+= ap3425/
obj-n	+= ap3045/
obj-y   += vcnl36866/
obj-y	+= vcnl36866_2nd/

DEBUG = n

ifeq ($(DEBUG),y)
	DEBFLAGS = -O -g -DASH_HW_DEBUG
else
	DEBFLAGS = -O -g
endif
ccflags-y += $(DEBFLAGS)

