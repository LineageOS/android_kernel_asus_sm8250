# Check if this driver needs be built for current target
ifeq ($(call is-board-platform,kona),true)
AUDIO_SELECT  := CONFIG_SND_SOC_KONA=m
endif

ifeq ($(call is-board-platform,lito),true)
AUDIO_SELECT  := CONFIG_SND_SOC_LITO=m
endif

AUDIO_CHIPSET := audio

LOCAL_PATH := $(call my-dir)
DLKM_DIR := $(TOP)/device/qcom/common/dlkm

ifneq ($(findstring opensource,$(LOCAL_PATH)),)
#	AUDIO_BLD_DIR := $(ANDROID_BUILD_TOP)/vendor/qcom/opensource/audio-kernel
	AUDIO_BLD_DIR := $(shell pwd)/vendor/qcom/opensource/audio-kernel
endif # opensource

# KBUILD_OPTIONS := AUDIO_ROOT=$(AUDIO_BLD_DIR)

KBUILD_OPTIONS := AUDIO_ROOT=$(AUDIO_BLD_DIR) 

KBUILD_OPTIONS += MODNAME=rt5683
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)
KBUILD_OPTIONS += $(AUDIO_SELECT)

# $(warn RT5683 mk file is parsed)

include $(CLEAR_VARS)
LOCAL_MODULE              := audio_rt5683.ko
LOCAL_MODULE_KBUILD_NAME  := snd-soc-rt5683.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk


