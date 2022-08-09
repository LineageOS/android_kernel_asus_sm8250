
#ifndef _EC_I2C_INTERFACE_H_
#define _EC_I2C_INTERFACE_H_

#include <linux/i2c.h>
#include "ec_comm.h"

#define	I2C_CLASS_NAME		    "ec_i2c"
#define I2C_RETRY_NUMBER        3
#define EC_UART_GPIO            0x12

#define CMD_I2C_NOTIFY_EC_ENUMERATE 0x01
#define CMD_I2C_TO_SET_GPIO    		0x02
#define CMD_I2C_ENABLE_PWM          0x03
#define CMD_I2C_SET_FREQ			0x05
#define CMD_I2C_SET_DUTY            0x06
#define CMD_I2C_SET_FACTORY_MODE    0x08
#define CMD_I2C_DISABLE_CHARGER_SUSPEND  0x0B
//#define CMD_I2C_SET_COVER_STATE     0x0C		// Station 2 define
#define CMD_I2C_SET_ULTRA_LOW_POWER_MODE  0x0C	// Station 3 define

#define CMD_I2C_SET_BATTERY_48H_STATE    0x0E
#define CMD_I2C_SET_EC_SSN          0x0F
#define CMD_I2C_WRITE_MODEL_NAME    0x10
#define CMD_I2C_SET_DISPLAY_BL      0x11
#define CMD_I2C_CONTROL_PORTA_CC    0x13
#define CMD_I2C_SET_FPS             0x15
#define CMD_I2C_CONTROL_DISPLAY     0x16
#define CMD_I2C_SET_COLOR_TEMP      0x17
#define CMD_I2C_SET_HBM             0x18
#define CMD_I2C_SET_PHONE_PANEL_STATE     0x20	// Station 3 define, Station 2 none.
#define CMD_I2C_SET_COVER_STATE     0x21		// Station 3 define, Station 2 none.

#define CMD_I2C_ENABLE_MIPI         0x50
#define CMD_I2C_DISCONNECT_PORTA_CC	0x70
#define CMD_I2C_CONNECT_PORTA_CC	0x71
#define CMD_I2C_U0504_GET_STATE    	0x75
#define CMD_I2C_SET_DP_DISPLAY_ID	0x7F
#define CMD_I2C_SET_CHARGER_TYPE    0x80
#define CMD_I2C_STATE          		0x81
#define CMD_I2C_GET_EC_READY_STATE  0x82
#define CMD_I2C_GET_PWM_FRQ_DUTY    0x84
#define CMD_I2C_GET_EC_FW_Version 	0x85
#define CMD_I2C_GET_THERMAL_ALERT   0x87
#define CMD_I2C_GET_BATTERY_CAP     0x88
#define CMD_I2C_GET_BATTERY_VOL     0x89
#define CMD_I2C_GET_BATTERY_CUR     0x8A
#define CMD_I2C_GET_CHARGER_TYPE    0x8B
#define CMD_I2C_GET_FACTORY_MODE    0x8D
#define CMD_I2C_GET_INT_TYPE        0x90
#define CMD_I2C_GET_DP_FW           0x91
#define CMD_I2C_GET_BATTERY_48H_STATE    0x94
#define CMD_I2C_GET_FPS				0x95
#define CMD_I2C_GET_RPM             0x96
#define CMD_I2C_GET_EC_SSN_L        0x97
#define CMD_I2C_GET_EC_SSN_H        0x98
#define CMD_I2C_GET_HW_ID           0x99
#define CMD_I2C_GET_MODEL_NAME      0x9A
#define CMD_I2C_GET_PORTA_CC_STATE  0x9B

#define CMD_I2C_GET_APL6001_CHIP_ID 0x9C
#define CMD_I2C_GET_APL6001_ADC1	0x9D
#define CMD_I2C_GET_APL6001_ADAPTER_ID	0x9E
#define CMD_I2C_GET_APL6001_ADC3	0x9F

#define CMD_I2C_GET_GPIO12          0xA0
#define CMD_I2C_GET_GPIO22          0xA1
#define CMD_I2C_GET_GPIO34          0xA2
#define CMD_I2C_GET_GPIO35          0xA3
#define CMD_I2C_GET_GPIO1C          0xA4
#define CMD_I2C_GET_GPIO21          0xA5
#define CMD_I2C_GET_GPIO00          0xA6
#define CMD_I2C_GET_GPIO2A          0xA7
#define CMD_I2C_GET_PANEL_ID        0xB0
#define CMD_I2C_GET_PD_FW           0xB3

#define CMD_I2C_GET_EventLog		0xF0

// Define Interrupt
#define NotifyUSB						0x01
#define NotifyThermalAlert				0x02
#define NotifyWakeFromUltraPowerMode	0x04
#define NotifyFacTest					0x08
#define NotifyEvtLog					0x10
#define NotifyLatchState				0x20
#define NotifyBatteryZero				0x40
#define NotifyAnXInt					0x80

extern u8 gEC_init;
extern struct completion hid_state;
extern struct ec_i2c_platform_data *ec_i2c_data;
extern struct ec_check_int_interface ec_check_int;
extern struct ec_set_gpio_interface ec_set_gpio;
extern struct ec_get_gpio_interface ec_get_gpio;
extern struct ec_battery_interface ec_battery_func;
extern struct ec_set_dp_display_interface ec_set_dp_display;
extern struct ec_porta_cc_interface ec_porta_cc;
extern struct ec_fw_ver_interface ec_fw_ver;
extern bool station_shutdown;

struct class *ec_i2c_class;
static DEFINE_MUTEX(i2c_rw_access);
static int EC_FW_VER;

enum asus_station_HWID
{
	ROG_Station_none = 0,
	ROG_Station1 = 1,
	ROG_Station2 = 2,
	ROG_Station3 = 3,
	ROG_Station_other = 255,
};

#endif
