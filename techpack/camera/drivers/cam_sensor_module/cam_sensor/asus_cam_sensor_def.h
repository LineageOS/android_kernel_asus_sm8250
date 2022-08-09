#ifndef ASUS_CAM_SENSOR_DEF_H
#define ASUS_CAM_SENSOR_DEF_H

#define SENSOR_ID_IMX214  0x0214
#define SENSOR_ID_IMX298  0x0298
#define SENSOR_ID_IMX351  0x0351
#define SENSOR_ID_IMX362  0x0362
#define SENSOR_ID_IMX363  0x0363
#define SENSOR_ID_IMX563  0x0563
#define SENSOR_ID_IMX586  0x0586
#define SENSOR_ID_IMX686  0x0686

#define SENSOR_ID_OV5670  0x5670
#define SENSOR_ID_OV8856  0x885a
#define SENSOR_ID_OV13855  0xD855
#define SENSOR_ID_OV24B1Q 0x2442
#define SENSOR_ID_S5K3M3  0x30D3
#define SENSOR_ID_OV08A10 0x0841

#define PROC_MODULE_CAMERA0	"driver/CameraModule0"
#define PROC_MODULE_CAMERA1	"driver/CameraModule1"
#define PROC_MODULE_CAMERA2	"driver/CameraModule2"
#define PROC_MODULE_CAMERA3	"driver/CameraModule3"
#define PROC_MODULE_CAMERA4	"driver/CameraModule4"

#define OTP_DATA_LEN_WORD (32)
#define OTP_DATA_LEN_BYTE (OTP_DATA_LEN_WORD*2)
#define OTP_ID_LEN (12)

#define PROC_OTP_Camera0     "driver/otp0"
#define PROC_OTP_Camera1     "driver/otp1"
#define PROC_OTP_Camera2     "driver/otp2"
#define PROC_OTP_Camera3     "driver/otp3"
#define PROC_OTP_Camera4     "driver/otp4"

#define PROC_THERMAL_REAR	 "driver/rear_temp"
#define PROC_THERMAL_REAR2	 "driver/rear2_temp"
#define PROC_THERMAL_REAR3	 "driver/rear3_temp"

#define PROC_THERMAL_FRONT	 "driver/front_temp"
#define PROC_THERMAL_FRONT2  "driver/front2_temp"

#define THERMAL_TYPE_REAR	 "rear_camera"
#define THERMAL_TYPE_REAR2	 "rear_camera2"
#define THERMAL_TYPE_REAR3	 "rear_camera3"

#define THERMAL_TYPE_FRONT	 "front_camera"
#define THERMAL_TYPE_FRONT2  "front_camera2"

#define SYSFS_ROOT_DIR		 "camera_sensor"
#define SYSFS_RESOLUTION_DIR "resolution"
#define SYSFS_STATUS_DIR	 "status"

#define SYSFS_ATTR_CAMERA0	camera0	//camera
#define SYSFS_ATTR_CAMERA1	camera1	//vga
#define SYSFS_ATTR_CAMERA2	camera2	//camera_2
#define SYSFS_ATTR_CAMERA3	camera3	//camera_3
#define SYSFS_ATTR_CAMERA4	camera4	//vga_2

#define PROC_SENSOR_I2C_RW "driver/sensor_i2c_rw"

#define PROC_EEPROM_REAR	"driver/rear_eeprom"
#define PROC_EEPROM_FRONT	"driver/front_eeprom"
#define PROC_EEPROM_REAR2	"driver/rear2_eeprom"
#define PROC_EEPROM_REAR3	"driver/rear3_eeprom"
//#define PROC_EEPROM_FRONT2	"driver/front2_eeprom"

#define PROC_EEPROM_I2C_R  "driver/eeprom_i2c_r"

#define PROC_ARCSOFT_CALI "driver/dualcam_cali"

#define PROC_ARCSOFT_CALI_1x ""PROC_ARCSOFT_CALI"_1x"
#define PROC_ARCSOFT_CALI_3x ""PROC_ARCSOFT_CALI"_3x"

#define	PROC_DIT_EEPROM_REAR	"driver/dit_rear_eeprom"
#define	PROC_DIT_EEPROM_FRONT	"driver/dit_front_eeprom"
#define	PROC_DIT_EEPROM_REAR2	"driver/dit_rear2_eeprom"
#define	PROC_DIT_EEPROM_REAR3	"driver/dit_rear3_eeprom"
//#define	PROC_DIT_EEPROM_FRONT2	"driver/dit_front2_eeprom"

#define FACTORYDIR "/vendor/factory/"
#define GOLDENDIR "/vendor/lib64/camera/"
#ifdef ASUS_DXO
#define DIT_DUT_REAR "dut_rear0.bin"
#define DIT_DUT_REAR2 "dut_rear2.bin"
#define DIT_DUT_REAR3 "dut_rear4.bin"
#define DIT_DUT_REAR4 "dut_IMX686_R.bin"
#define DIT_DUT_FRONT "dut_front1.bin"
#else
#define DIT_DUT_REAR "dut_IMX686_R.bin"
#define DIT_DUT_REAR2 "dut_OV13855_R.bin"
#define DIT_DUT_REAR3 "dut_OV8856_R.bin"
#define DIT_DUT_REAR4 "dut_IMX686_R.bin"
#define DIT_DUT_FRONT "dut_OV24B1Q_R.bin"
//#define DIT_DUT_FRONT2 "dut_rear2.bin"
#endif

#define DIT_DUT_REAR_ZF7 "dut_rear0.bin"
#define DIT_DUT_REAR2_ZF7 "dut_rear2.bin"
#define DIT_DUT_REAR3_ZF7 "dut_rear4.bin"
#define DIT_DUT_FRONT_ZF7 "dut_IMX363_H.bin"

#define DUAL_CALI_BIN ""FACTORYDIR"dualcam_cali.bin"

#define PROC_SENSORS_RES "driver/camera_res"

#define PROC_MODULE_CHANGE_REAR "driver/rear_module_change"
#define PROC_MODULE_CHANGE_FRONT "driver/front_module_change"
#define PROC_MODULE_CHANGE_REAR2 "driver/rear2_module_change"
#define PROC_MODULE_CHANGE_REAR3 "driver/rear3_module_change"
#define PROC_MODULE_CHANGE_REAR4 "driver/rear4_module_change"
#define PROC_MODULE_CHANGE_FRONT2 "driver/front2_module_change"

#endif
