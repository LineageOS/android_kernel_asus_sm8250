# Goodix Touchscreen Makefile
# This Makefile is only for tempory compiling use
# Use xxxproject/Makefile-release when release driver
#
ifeq ($(ASUS_BUILD_PROJECT),ZS661KS)
obj-m += station_goodix_touch.o
station_goodix_touch-objs += goodix_ts_i2c.o
station_goodix_touch-objs += goodix_ts_core.o
station_goodix_touch-objs += goodix_cfg_bin.o
station_goodix_touch-objs += goodix_gtx8_update.o
#station_goodix_touch-objs += goodix_ts_gesture.o
station_goodix_touch-objs += goodix_ts_tools.o
endif