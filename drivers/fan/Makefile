# on Linux 2.6 kernel
# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
	obj-m := nct7802.o
#	nct7802-objs := nct7802y.o
# Otherwise we were called directly from the command
# line; invoke the kernel build system.
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD := $(shell pwd)
2.6:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules -Wall 

clean:
	rm -rf *.o *.ko *.mod.c Module.symvers modules.order .nct7802.* .nct7802.* .tmp_versions
endif
