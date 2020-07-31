#ifndef MSP430FR2311_H
#define MSP430FR2311_H

struct MSP430FR2311_info {
	struct class *MSP430FR2311_class;
	struct device *mcu_dev;
	struct i2c_client *i2c_client; 
	struct workqueue_struct *mcu_wq;
	int mcu_polling_delay;
	uint8_t slave_addr;  
	int mcu_reset;
	int mcu_test;
	int mcu_5v_boost_enable;
	int mcu_wakeup;
	int mcu_int;
	int mcu_irq;
	struct regulator *vcc_l10a_3p3;
	struct regulator *vcc_s4a_1p8;
};

int MSP430FR2311_power_control(uint8_t enable);
int MSP430FR2311_Set_AutoMode(int mode) ;
int MSP430FR2311_Set_AutoModeWithAngle(int mode, int) ;
int MSP430FR2311_Set_ManualMode(int dir, int angle, int speed);
int MSP430FR2311_Stop(void);
int MSP430FR2311_Set_ParamMode(const uint16_t* vals);
int Zen7_MSP430FR2311_Set_ParamMode(const uint16_t* vals);
int Zen7_MSP430FR2311_DealAngle(uint16_t        *buf, uint8_t len);
int Zen7_MSP430FR2311_wrDrv(uint16_t        *buf, uint8_t len);
int Zen7_MSP430FR2311_wrAKM(uint16_t        *buf, uint8_t len);
void report_motor_event(uint8_t OpCode, uint32_t value);

extern unsigned char g_motor_status;
extern uint8_t g_motor_mode;
extern uint8_t g_motor_power_state;
extern uint8_t g_motor_camera_open;
void asus_motor_init(struct MSP430FR2311_info * ctrl);

typedef struct{
    int dir;
    int angle;
    int speed;
}motorDrvManualConfig_t;

#define ASUS_MOTOR_NAME_SIZE	32
#define ASUS_MOTOR_DATA_SIZE	4

#define ASUS_MOTOR_DRV_DEV_PATH    ("/dev/asusMotoDrv")
#define ASUS_MOTOR_DRV_IOC_MAGIC                      ('M')
#define ASUS_MOTOR_DRV_AUTO_MODE _IOW(ASUS_MOTOR_DRV_IOC_MAGIC, 1, int)
#define ASUS_MOTOR_DRV_MANUAL_MODE _IOW(ASUS_MOTOR_DRV_IOC_MAGIC, 2, motorDrvManualConfig_t)
#define ASUS_MOTOR_DRV_STOP _IOW(ASUS_MOTOR_DRV_IOC_MAGIC, 3, int)
#define ASUS_MOTOR_DRV_GET_NAME	_IOR(ASUS_MOTOR_DRV_IOC_MAGIC, 4, char[ASUS_MOTOR_NAME_SIZE])
#define ASUS_MOTOR_DRV_CLOSE	_IOR(ASUS_MOTOR_DRV_IOC_MAGIC, 5, char[ASUS_MOTOR_NAME_SIZE])

#define ASUS_MOTOR_DRV_AUTO_MODE_WITH_ANGLE _IOW(ASUS_MOTOR_DRV_IOC_MAGIC, 6, int)
#define ASUS_MOTOR_DRV_GET_STEPS	_IOR(ASUS_MOTOR_DRV_IOC_MAGIC, 7, int)
#endif
