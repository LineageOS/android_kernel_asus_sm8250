# make sure that the environment variables ARCH and CROSS_COMPILE
# are set for your architecture and cross compiler
#
# e.g. for Beaglebone black:
# export ARCH := arm
# export CROSS_COMPILE := arm-linux-gnueabihf-
#
# e.g. Samsung Galaxy S6:
# export ARCH := arm64
# export CROSS_COMPILE := aarch64-linux-android-

# In case of out of tree build, build as a module
# (when build inside kernel we will not enter this directory and this will have no effect)

# add version number derived from Git
ifeq ($(KDIR),)
PLMA_HOSTSW_DIR=$(realpath -f $(srctree)/$(src))
else
PLMA_HOSTSW_DIR=$(realpath -f $(src))
endif
# GIT_VERSION=$(shell cd $(PLMA_HOSTSW_DIR); git describe --tags --dirty --match "v[0-9]*.[0-9]*.[0-9]*")


snd-soc-rt5683-objs += rt5683.o

obj-m := snd-soc-rt5683.o

