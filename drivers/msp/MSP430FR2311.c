/* Copyright (C) 2018 Vishay MCU Microsystems Limited
 * Author: Randy Change <Randy_Change@asus.com>
 *                                    
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
//#include <asm/mach-types.h>
#include <linux/regulator/consumer.h>
//#include <asm/setup.h>
#include <linux/jiffies.h>
#include "MSP430FR2311.h"
#define MSP430FR2311_I2C_NAME "msp430fr2311"

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#define ZEN7   1

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 3
#define UPDATE_FW_RETRY_COUNT 3

#define MCU_POLLING_DELAY 5000
#define MSP430_READY_I2C 0x35

struct MSP430FR2311_info *mcu_info;
static struct mutex MSP430FR2311_control_mutex;

enum eMCUState {
	MCU_TBD,
	MCU_EMPTY,
	MCU_PROGRAMMING,
	MCU_WAIT_POWER_READY,
	MCU_CHECKING_READY,
	MCU_READY,
	MCU_LOOP_TEST
} MCUState=MCU_TBD;

static signed char iCloseCounter=0;
static signed char iOpenCounter=0;
int (*fManualMode)(int , int , int );

enum UserK_State {
	UserK_TBD = 0,
	UserK_Start,
	UserK_rMCU_Ready,
	UserK_SaveK,
	UserK_NoSaveK,
	UserK_AbortK,
	UserK_Reset,
	UserK_Finish
}UserKState=UserK_TBD;
static void mcu_cal_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(cal_work, mcu_cal_work);
static void mcu_do_work_later(struct work_struct *work);
static DECLARE_DELAYED_WORK(report_work, mcu_do_work_later);
void mStopConditionInit(void);

//const int powerUpDuration=200;
#define DEFAULT_POWERDOWNDURATION 30000
#define MCU_5V_ALWAYS_ON
int powerDownDuration=DEFAULT_POWERDOWNDURATION;
bool bShowStopInfoOnce=1;

typedef enum{
    MOTOR_ANGLE,
    MOTOR_ROTATE,
    MOTOR_FORCE,
    MOTOR_DRVINT,
    MOTOR_GET_ANGLE,
    MOTOR_COPY_MCU_FILE,
    MOTOR_K_FINISH,
}MotorOpCode;

typedef enum{
    ROTATE_FINISH,
    ROTATE_CANCEL,
    ROTATE_STOP,
}RotateStopReason;

//Backup S
#include <linux/mm.h>
#include <linux/syscalls.h>
#define MCU_BAKEUP_FILE_NAME "/motor_fw1/mcu_bak"
#define MCU_BAKEUP_RAW_FILE_NAME "/motor_fw1/mcu_raw"
#define MCU_BAKEUP_USER_FILE_NAME "/motor_fw2/mcu_bak2"
#define MCU_BAKEUP_USER_RAW_FILE_NAME "/motor_fw2/mcu_raw2"
#define CAL_DATA_OFFSET	0
#define FILE_OP_WRITE 1
#define FILE_OP_READ  0

#define CalLength	(24+80+6)
#define RawLength	40
typedef union
{
	uint8_t Cal_val[CalLength];
	struct
	{
		int16_t Cal_offSetPoint[2];		//4 bytes
		int16_t Cal_gain[2];			//4 bytes
		float 	Cal_zeroBias;			//4 bytes
		float 	Cal_rotAngle[1];		//4 bytes
		uint16_t AKMThrd[4];			//8 bytes BOP1Y_15_0/BRP1Y_15_0/BOP1Z_15_0/BRP1Z_15_0.
		float 	Cal3CorrectionA[10][2];		//80 bytes
		int16_t Cal_firstMAGdata[3]; 	//6 bytes
	} CalParam;
}CAL_TABLE;

CAL_TABLE fcal_val;
CAL_TABLE fcal_val_UserK;
CAL_TABLE fcal_val_rPhone;
CAL_TABLE fcal_val_rMCU;

static int backup_mcu_cal_thed(void);
static int read_mcu_cal_thed(void);
static int Zen7_MSP430FR2311_wI2CtoMCU(uint8_t *buf, uint8_t len);
int Zen7_MSP430FR2311_rI2CtoCPU(uint8_t cmd, uint8_t *rBuf, uint8_t len);
static void bak_raw(void);
int ManualMode_AfterAndPR2_NOSS(int dir, int angle, int speed);

//user K
static char IsFileNull(const char *filename);
static void UserCaliReset(void);
static int Backup_user_cal_thed(void);
static int read_usek_cal3threshlod(void);
static int CompareCaliData(char *filename);

#define bit0	0x01
#define bit1	0x02
#define bit2	0x04
#define bit3	0x08
#define bit4	0x10
#define bit5	0x20
#define bit6	0x40
#define bit7	0x80

/* MASK operation macros */
#define SET_MASK(reg, bit_mask) 	    ((reg)|=(bit_mask))
#define CLEAR_MASK(reg, bit_mask) 	    ((reg)&=(~(bit_mask)))
#define IS_MASK_SET(reg, bit_mask) 	    (((reg)&(bit_mask))!=0)
#define IS_MASK_CLEAR(reg, bit_mask)    (((reg)&(bit_mask))==0)
//Backup E

static int cal_cmd = 0;						//0:factory K; 1:user K.
static unsigned char FacOrUserClaFlg = 0;	//0:factory K; 1:user K.

static void waitDelayAndShowText(char * s) {
	int i=10;
	D("[MCU] (%s) command, wait for %d second.", s, i);
	return;
	
	for (i=10;i>0;i--) {
		pr_err(" %d...", i);
//			for (delayLoop=0;delayLoop<1000;delayLoop++)
//				udelay(500);
		msleep(500);
	}
	D(" issue\n");
}

int iProgrammingCounter=0;
int iProgrammingFail=0;
void dumpI2CData(char *s, uint8_t slave_addr, uint8_t* writeBuffer,  
    uint32_t numOfWriteBytes ) {
    char buf[10];
	char line[1024];
	uint8_t loop_i;
	line[0]=0;
	for (loop_i=0;loop_i<numOfWriteBytes;loop_i++) {
		sprintf(buf, " %X", writeBuffer[loop_i]);
		strcat(line, buf);
	}
	
	pr_err("[MCU] %s [0x%X] : len=%d, %s", s, slave_addr, numOfWriteBytes, line);
}




bool MSP430_I2CWriteReadA (uint8_t slave_addr, uint8_t* writeBuffer,  
    uint32_t numOfWriteBytes, uint8_t* readBuffer, uint32_t numOfReadBytes)
{
	uint8_t loop_i;

__u8* rxDMA;
		__u8* txDMA;

struct i2c_msg msgs[] = {
	{
	 .addr = slave_addr,
	 .flags = 0,
	 .len = numOfWriteBytes,
	 .buf = writeBuffer,
	},		 
	{
	 .addr = slave_addr,
	 .flags = I2C_M_RD,
	 .len = numOfReadBytes,
	 .buf = readBuffer,
	},		 
};

if (numOfWriteBytes>32||numOfReadBytes>=32) {
	rxDMA = kzalloc(sizeof(uint8_t)*numOfReadBytes, GFP_DMA | GFP_KERNEL);
	txDMA = kzalloc(sizeof(uint8_t)*numOfWriteBytes, GFP_DMA | GFP_KERNEL);
//	memcpy(rxDMA, readBuffer, numOfReadBytes);
	memcpy(txDMA, writeBuffer, numOfWriteBytes);
	msgs[0].buf=txDMA;
	msgs[1].buf=rxDMA;
}
	
		dumpI2CData("I2C_WR_w",msgs[0].addr,  writeBuffer, numOfWriteBytes);

		for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		
			if (i2c_transfer(mcu_info->i2c_client->adapter, msgs, 2) > 0)
				break;
		
			/*check intr GPIO when i2c error*/
			if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
				pr_err("[MCU][mcp error] %s, i2c err, slaveAddr 0x%x\n",
					__func__, slave_addr);
			msleep(10);
		}
		
		if (loop_i >= I2C_RETRY_COUNT) {
			pr_err(KERN_ERR "[MCU][mcp error] %s slaveAddr:0x%x retry over(i2c_err) %d\n",
				__func__, slave_addr, I2C_RETRY_COUNT);
			
			if (numOfWriteBytes>32||numOfReadBytes>=32) {
				memcpy(readBuffer, rxDMA, numOfReadBytes);
				kfree(rxDMA);
				kfree(txDMA);
				
			}
			dumpI2CData("I2C_WR_r i2c err,", msgs[1].addr, readBuffer, numOfReadBytes);
			return false;
		}
		
		if (numOfWriteBytes>32||numOfReadBytes>=32) {
			memcpy(readBuffer, rxDMA, numOfReadBytes);
			kfree(rxDMA);
			kfree(txDMA);
		}
		dumpI2CData("I2C_WR_r", msgs[1].addr, readBuffer, numOfReadBytes);

    return true;
}

bool MSP430_I2CWriteReadA_Nolog (uint8_t slave_addr, uint8_t* writeBuffer,  
	uint32_t numOfWriteBytes, uint8_t* readBuffer, uint32_t numOfReadBytes)
{
	uint8_t loop_i;

__u8* rxDMA;
		__u8* txDMA;

struct i2c_msg msgs[] = {
	{
	 .addr = slave_addr,
	 .flags = 0,
	 .len = numOfWriteBytes,
	 .buf = writeBuffer,
	},		 
	{
	 .addr = slave_addr,
	 .flags = I2C_M_RD,
	 .len = numOfReadBytes,
	 .buf = readBuffer,
	},		 
};

if (numOfWriteBytes>32||numOfReadBytes>=32) {
	rxDMA = kzalloc(sizeof(uint8_t)*numOfReadBytes, GFP_DMA | GFP_KERNEL);
	txDMA = kzalloc(sizeof(uint8_t)*numOfWriteBytes, GFP_DMA | GFP_KERNEL);
//	memcpy(rxDMA, readBuffer, numOfReadBytes);
	memcpy(txDMA, writeBuffer, numOfWriteBytes);
	msgs[0].buf=txDMA;
	msgs[1].buf=rxDMA;
}
	
		//dumpI2CData("I2C_WR_w",msgs[0].addr,  writeBuffer, numOfWriteBytes);

		for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		
			if (i2c_transfer(mcu_info->i2c_client->adapter, msgs, 2) > 0)
				break;
		
			/*check intr GPIO when i2c error*/
			if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
				pr_err("[MCU][mcp error] %s, i2c err, slaveAddr 0x%x\n",
					__func__, slave_addr);
			msleep(10);
		}
		
		if (loop_i >= I2C_RETRY_COUNT) {
			pr_err(KERN_ERR "[MCU][mcp error] %s slaveAddr:0x%x retry over(i2c_err) %d\n",
				__func__, slave_addr, I2C_RETRY_COUNT);
			
			if (numOfWriteBytes>32||numOfReadBytes>=32) {
				memcpy(readBuffer, rxDMA, numOfReadBytes);
				kfree(rxDMA);
				kfree(txDMA);
				
			}
			dumpI2CData("I2C_WR_r i2c err,", msgs[1].addr, readBuffer, numOfReadBytes);
			return false;
		}
		
		if (numOfWriteBytes>32||numOfReadBytes>=32) {
			memcpy(readBuffer, rxDMA, numOfReadBytes);
			kfree(rxDMA);
			kfree(txDMA);
		}
		//dumpI2CData("I2C_WR_r", msgs[1].addr, readBuffer, numOfReadBytes);

	return true;
}


bool MSP430_I2CWriteRead (uint8_t* writeBuffer,  
    uint32_t numOfWriteBytes, uint8_t* readBuffer, uint32_t numOfReadBytes)
{
  return MSP430_I2CWriteReadA_Nolog(mcu_info->slave_addr, writeBuffer, numOfWriteBytes, readBuffer, numOfReadBytes);
}

bool MSP430_I2CAction(struct i2c_msg* msgs, uint8_t slave_addr, uint8_t* buffer, uint32_t numberOfBytes)
{
	uint8_t loop_i;
	
	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(mcu_info->i2c_client->adapter, msgs, 1) > 0)
			break;

		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			pr_err("[MCU][mcp error] %s, i2c err, slaveAddr 0x%x\n",
				__func__, slave_addr);

		msleep(10);
	}
	
	if (loop_i >= I2C_RETRY_COUNT) {
		pr_err(KERN_ERR "[MCU][mcp error] %s slaveAddr:0x%x retry over(i2c_err) %d\n",
			__func__, slave_addr, I2C_RETRY_COUNT);
		return false;
	}

	return true;
}


bool MSP430_I2CRead(uint8_t slave_addr, uint8_t* buffer, uint32_t numberOfBytes)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = slave_addr,
		.flags = I2C_M_RD,
		 .len = numberOfBytes,
		 .buf = buffer,
		},		 
	};

	if ( MSP430_I2CAction(msgs, slave_addr, buffer, numberOfBytes)) {
		dumpI2CData("I2C_r", slave_addr, buffer, numberOfBytes);
		return true;
	};
	return false;
}

bool MSP430_I2CRead_NoLog(uint8_t slave_addr, uint8_t* buffer, uint32_t numberOfBytes)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = slave_addr,
		.flags = I2C_M_RD,
		 .len = numberOfBytes,
		 .buf = buffer,
		},		 
	};

	if ( MSP430_I2CAction(msgs, slave_addr, buffer, numberOfBytes)) {
		//dumpI2CData("I2C_r", slave_addr, buffer, numberOfBytes);
		return true;
	};
	return false;
}

bool MSP430_I2CWriteA(uint8_t slave_addr, uint8_t* buffer, uint32_t numberOfBytes)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = slave_addr,
		 .flags = 0,
		 .len = numberOfBytes,
		 .buf = buffer,
		},		 
	};

	dumpI2CData("I2C_w", slave_addr, buffer, numberOfBytes);
	return MSP430_I2CAction(msgs, slave_addr, buffer, numberOfBytes);
}

bool MSP430_I2CWriteA_NoLog(uint8_t slave_addr, uint8_t* buffer, uint32_t numberOfBytes)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = slave_addr,
		 .flags = 0,
		 .len = numberOfBytes,
		 .buf = buffer,
		},		 
	};

	//dumpI2CData("I2C_w", slave_addr, buffer, numberOfBytes);
	return MSP430_I2CAction(msgs, slave_addr, buffer, numberOfBytes);
}

bool MSP430_I2CWrite(uint8_t* buffer, uint32_t numberOfBytes)
{
	return MSP430_I2CWriteA(mcu_info->slave_addr, buffer, numberOfBytes);

}

char gFWVersion[4];
//Manual mode.
static uint16_t LEAD_DELTA = 57;
uint16_t ConstSpeedMode[]={0, 120, 120, 120, 120, 120, 120, 255, 85, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t defaultManualSpeed[22] = {0, 0, 8, 73, 10, 73, 20, 73, 120, 0, 160, 0, 4, 73, 6, 73, 28, 73, 4, 73, 4, 73};	//For convenience match to IMS speed, fill byte0 and byte1.

//Factory note.
uint16_t TightenMode[]={1, 4, 4, 4, 4, 4, 4, 100, 0, 0, 0, 0, 0, 73, 73, 73, 73, 73, 73};
uint16_t SmallAngle[]={1, 4, 4, 4, 4, 4, 4, 6, 0, 0, 0, 0, 0, 73, 73, 73, 73, 73, 73};	//for camera apk

//Auto mode(0_180/180_0).
uint16_t ConvertFRQMode[][19]={
	{0, 120, 120, 56, 28, 20, 12, 255, 47, 2, 2, 2, 40, 0, 0, 73, 73, 73, 73},
	{1, 120, 120, 56, 28, 20, 12, 255, 34, 2, 3, 3, 50, 0, 0, 73, 73, 73, 73},
};

//Free angle, total step = FS + SS.	
static uint16_t CONVERT_FRQ_FS_STEP[]={310, 300};
static uint16_t CONVERT_FRQ_SS_STEP[]={40, 50};

//Angle < 10.
uint16_t ConvertFRQModeForSmallAngle[]={0, 8, 8, 8, 8, 8, 8, 50, 0, 0, 0, 0, 0, 73, 73, 73, 73, 73, 73};

//Drop mode.
uint16_t AutoEmergencyMode[] = {0, 120, 120, 160, 160, 180, 12, 150, 0, 100, 0, 90, 10, 0, 0, 0, 0, 0, 73};
uint16_t AutoWarmUpMode[] = {0, 80, 80, 80, 80, 80, 120, 255, 55, 0, 0, 0, 40, 0, 0, 0, 0, 0, 0};
uint16_t AutoWarmUpMode2[] = {0, 120, 120, 120, 120, 120, 120, 255, 55, 0, 0, 0, 90, 0, 0, 0, 0, 0, 0};

//Main memory erase password.
uint8_t bslPassword[32] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,	\
						   0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//Motor stop condition and cali command.
uint16_t StopCondition[]={1, 179, 4, 0, 73, 4, 0, 73};
uint16_t CaliCmd[]={4, 2, 0};

static int parse_param(char* buf, const char* title, uint16_t* target, int count) {
#define PARSE_DIG_PARAM_1 "%d"
  uint16_t cali_item[25];
	char parseLine[255];
	int i=0, matched_param=0;
	snprintf(parseLine, sizeof(parseLine), "%s=%s", title, PARSE_DIG_PARAM_1);
	  if (sscanf(buf, parseLine, &cali_item[i])>0) {
			target[i]=cali_item[i];
			matched_param++;
			snprintf(parseLine, sizeof(parseLine), "%s=" PARSE_DIG_PARAM_1, title, cali_item[i]);
			buf += strlen(parseLine);
//			pr_err("[MCU] detail %s", parseLine);
			for (i=1;i<count;i++) {
//				pr_err("[MCU] detail >>>%s", buf);
				if (sscanf(buf," %d", &cali_item[i])>0) {
					target[i]=cali_item[i];
					matched_param++;
					snprintf(parseLine, sizeof(parseLine),  " " PARSE_DIG_PARAM_1, cali_item[i]);
					buf += strlen(parseLine);
//					pr_err("[MCU] detail %s", parseLine);
				} else {
					break;
				}
			}


		{//debug
			char value[50];
			i=0;
			if (matched_param) {
				snprintf(parseLine, sizeof(parseLine), "%s(%d)=" PARSE_DIG_PARAM_1, title, matched_param, cali_item[i]);
				for (i=1;i<matched_param;i++) {
					snprintf(value, sizeof(value), " " PARSE_DIG_PARAM_1, cali_item[i]);
					strncat(parseLine, value, sizeof(parseLine)-1);
				}
				pr_err("[MCU] read %s", parseLine);
			} 
		}

			
	}else {
//		pr_err("[MCU] No match %s", title);			
	}
	return matched_param;
	
}


static void process_cali_item(char* buf, ssize_t bufLen) {
/*
AUTO_DEFAULT        =100 100 100 100 100 100 0 0 0 4 4 6
AUTO_EMERGENCY=100 100 100 100 100 100 0 0 0 4 4 6
MANUAL_DEFAULT=100 100 100 100 100 100 0 0 0 4 4 6
MANUAL_LEAD_DELTA=53
MANUAL_GEAR_RATIO=166/100
*/
char* start_buf=buf;
do {
	while (buf[0]==0xd || buf[0]==0xa) buf++;
	
	{  //debug
			//D("[MCU] >>> %s", buf);
	}

	parse_param(buf, "CONST_FRQ_SPEED", &ConstSpeedMode[1], 18);
	parse_param(buf, "CONVERT_FRQ_SMALL_ANGLE", &ConvertFRQModeForSmallAngle[1], 18);
	parse_param(buf, "CONVERT_FRQ_UP", &ConvertFRQMode[0][1], 18);
	parse_param(buf, "CONVERT_FRQ_DOWN", &ConvertFRQMode[1][1], 18);
	parse_param(buf, "AUTO_EMERGENCY", &AutoEmergencyMode[1], 18);
	parse_param(buf, "WARM_UP_STEP", &AutoWarmUpMode[1], 18);
	parse_param(buf, "WARM_UP_STEP_2", &AutoWarmUpMode2[1], 18);
	parse_param(buf, "MANUAL_LEAD_DELTA", &LEAD_DELTA, 1);
	parse_param(buf, "CONVERT_FRQ_UP_SS_STEP", &CONVERT_FRQ_SS_STEP[0], 1);
	parse_param(buf, "CONVERT_FRQ_DOWN_SS_STEP", &CONVERT_FRQ_SS_STEP[1], 1);
	parse_param(buf, "CONVERT_FRQ_UP_FS_STEP", &CONVERT_FRQ_FS_STEP[0], 1);
	parse_param(buf, "CONVERT_FRQ_DOWN_FS_STEP", &CONVERT_FRQ_FS_STEP[1], 1);
	parse_param(buf, "MANUAL_SPEED", &defaultManualSpeed[2], 20);
	parse_param(buf, "TIGHTEN_STEP", &TightenMode[1], 18);
	parse_param(buf, "STOP_CONDITION", &StopCondition[0], 8);
	parse_param(buf, "CALI_CMD", &CaliCmd[0], 3);
	parse_param(buf, "S_ANGLE", &SmallAngle[1], 18);

	//parse_param(buf, "PASSWORD", (uint16_t*)(&bslPassword[0]), 32);
	
	while (*buf != 0xa) {
		buf++;
		if (buf-start_buf>bufLen) {
			buf=NULL;
			break;
		}
	}
}while (buf !=NULL);


}

extern bool read_kernel_file(const char*, void (*)(char*, ssize_t) );

void read_cali_file() {	
	#ifdef ASUS_FTM_BUILD
		const char mcu_cali[]= {"/vendor/firmware/mcu_cali_factory"};
	#else
		const char mcu_cali[]= {"/vendor/firmware/mcu_cali"};
	#endif
	read_kernel_file(mcu_cali,  process_cali_item);
}


static uint8_t invokeString[8] = {0xCA, 0xFE, 0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xBE};
static uint8_t resetVector[2];
static uint32_t resetVectorValue;

#include "i2cbsl.h"
#include "firmware_parser.h"



static int MSP43FR2311_Go_BSL_Mode(void) {
	uint8_t isAfterSR;
	
	waitDelayAndShowText("simulator bsl protocol");
if ((g_ASUS_hwID == HW_REV_EVB) || (g_ASUS_hwID == HW_REV_SR) || (g_ASUS_hwID == HW_REV_SR2) || (g_ASUS_hwID == HW_REV_SR3)) 	//EVB or SR
	isAfterSR = 1;
else
	isAfterSR = 0;
	
	//gpio_set_value(mcu_info->mcu_test, 0^isAfterSR);
	gpio_set_value(mcu_info->mcu_reset, 0);
	msleep(20);
	gpio_set_value(mcu_info->mcu_test, 0^isAfterSR);
	msleep(20);
	gpio_set_value(mcu_info->mcu_test, 1^isAfterSR);
	msleep(20);
	gpio_set_value(mcu_info->mcu_test, 0^isAfterSR);
	msleep(20);
	gpio_set_value(mcu_info->mcu_test, 1^isAfterSR);
	msleep(20);
	gpio_set_value(mcu_info->mcu_reset, 1);
	msleep(20);
	gpio_set_value(mcu_info->mcu_test, 0^isAfterSR);
	msleep(10);
	//gpio_set_value(mcu_info->mcu_test, 0);

	D("[MCU] INFO: Invoking the BSL .\n");
	msleep(50);	//Wait mcu hw go to bsl mode ready.
	return MSP430BSL_invokeBSL(invokeString, 8);			
}

static int MSP43FR2311_Update_Firmware_Load_File(bool bLoadFromFile) {
	int res=0, uu=0, ii=0;
	int delayLoop;
	tMSPMemorySegment* tTXTFile=NULL;
	MSP430FR2311_power_control(1);

	for (delayLoop=0;delayLoop<3;delayLoop++) {
		res=MSP43FR2311_Go_BSL_Mode();
		if( res== MSP430_STATUS_OPERATION_OK )
		{
			pr_err("[MCU] Go to bsl mode successful!\n");
			break;
		}
	}
	
	if( res!= MSP430_STATUS_OPERATION_OK )
	{
		pr_err("[MCU] Go to bsl mode fail!\n");
		//msleep(5000);
		goto BSLCleanUp;
	}

	D("[MCU] INFO: opening mcu_firmware.txt... ");
	
	MCUState=MCU_PROGRAMMING;
	iProgrammingCounter++;
	if (bLoadFromFile) {
		tTXTFile = read_firmware_file();	//zen7 used.
	} else {
		tTXTFile = MSP430BSL_parseTextFile();
	}
	 
	
			/* Sleeping after the invoke */
//			for (delayLoop=0;delayLoop<1000;delayLoop++)
//			udelay(2000);
	
#if 0 	
			/* Issuing a mass reset  */
			D("[MCU] INFO: Issuing a mass reset\n");
			if(MSP430BSL_massErase() != MSP430_STATUS_OPERATION_OK )
			{
					pr_err("[MCU] ERROR: Could not issue mass erase!\n");
					for (delayLoop=0;delayLoop<1000;delayLoop++)
					udelay(5000);
					goto BSLCleanUp;
			}
	
			/* Sleeping after the mass reset */
			for (delayLoop=0;delayLoop<1000;delayLoop++)
			udelay(2000);
	
			/* Unlocking the device */
#endif
#if 1	
	//memset(bslPassword, 0xFF, 32);
	for (ii=0;ii<3;ii++)  {
			D("[MCU] INFO: Unlocking the device \n");
			waitDelayAndShowText("Erase main memory");
			if(MSP430BSL_unlockDevice(bslPassword) != MSP430_STATUS_OPERATION_OK )
			{
				pr_err("[MCU] ERROR: Could not unlock device! Cnt:%d\n", ii);
//				goto BSLCleanUp;
			}
		}
#endif	
			waitDelayAndShowText("Programming all");
			/* Programming all memory segments */
			for(uu=0;uu<5;uu++)
			{
					D("[MCU] INFO: Programming attempt number %d\n", uu);
	
//					tTXTFile = firmwareImage;
					while(tTXTFile != NULL)
					{
						pr_err("MCU txtfile assign to %p", tTXTFile);
							D("[MCU] INFO: Programming @0x%x with %d bytes of data...0x%X, 0x%X, 0x%X ", 
									tTXTFile->ui32MemoryStartAddr, 
									tTXTFile->ui32MemoryLength,
									tTXTFile->ui8Buffer[0],
									tTXTFile->ui8Buffer[1],
									tTXTFile->ui8Buffer[2]
									);
	
							/* Programming the memory segment */
							ii = 0;
							while(ii < tTXTFile->ui32MemoryLength)
							{
									if((tTXTFile->ui32MemoryLength - ii) > 128)
									{
											res = MSP430BSL_sendData(tTXTFile->ui8Buffer + ii, 
																									 tTXTFile->ui32MemoryStartAddr + ii, 
																									 128);
											ii += 128;
									}
									else
									{
											res = MSP430BSL_sendData(tTXTFile->ui8Buffer + ii, 
																									 tTXTFile->ui32MemoryStartAddr + ii, 
																									 (tTXTFile->ui32MemoryLength - ii));
											ii = tTXTFile->ui32MemoryLength;
									}
	
									if(res != MSP430_STATUS_OPERATION_OK)
									{
											pr_err("[MCU] FAIL!ERROR: Programming address 0x%x (Code 0x%x).\n", 
																	tTXTFile->ui32MemoryStartAddr + ii, res);
											break;
									}
							}
	
							if(res != MSP430_STATUS_OPERATION_OK)
							{
									break;
							}
	
							D("[MCU] done!\n");
	
							tTXTFile = tTXTFile->pNextSegment;
					}
	
					if(res == MSP430_STATUS_OPERATION_OK)
					{
							D("[MCU] INFO: Programmed all memory locations successfully.\n");
							break;
					} else {
					}
			}


			if (uu>5) {
				goto BSLCleanUp;
			}
			/* Resetting the device */
			D("[MCU] INFO: Resetting the device.\n");
			res = MSP430BSL_readData(resetVector, MSP430_RESET_VECTOR_ADDR, 2);
	
			if(res != MSP430_STATUS_OPERATION_OK)
			{
					pr_err("[MCU] ERROR: Could not read reset vector address!\n");
					msleep(5000);
					goto BSLCleanUp;
			}
	
			resetVectorValue = (resetVector[1] << 8) | resetVector[0]; 
			D("[MCU] INFO: Reset vector read as 0x%x\n", resetVectorValue);
			res = MSP430BSL_setProgramCounter(resetVectorValue);
	
			if(res != MSP430_STATUS_OPERATION_OK)
			{
					pr_err("[MCU] ERROR: Could not set program counter!\n");
					msleep(5000);
					goto BSLCleanUp;
			}
	
	pr_err("[MCU] INFO: Firmware updated without issue, %d/%d.\n", iProgrammingFail, iProgrammingCounter);

	//If update FW success, clear fail count.
	iProgrammingFail = 0;
	
	D("[MCU] INFO: power off the device.\n");
	MSP430FR2311_power_control(0);
	//gpio_set_value(mcu_info->mcu_5v_boost_enable, 0);
	msleep(5);
	pr_err("[MCU] INFO: re-power on the device.\n");
	MSP430FR2311_power_control(1);
	//gpio_set_value(mcu_info->mcu_5v_boost_enable, 1);
	//copy file to asdf folder.
	//report_motor_event(MOTOR_COPY_MCU_FILE, 2);	// /factory/mcu_bak --> asdf.
	msleep(3000);	//Delay to wait MCU i2c slave init ok.

	//Re-write backup calibration and threadhold data to MCU.
	MCUState=MCU_READY; 	//Cmd 0xbf logic also will set MCU_READY when this function end, so can set MCU_READY early here.
	//read_mcu_cal_thed();
	if(IsFileNull(MCU_BAKEUP_USER_FILE_NAME) == 1){
		read_usek_cal3threshlod();	//read motor_fw2 file to mcu.
	}else{
		read_mcu_cal_thed();		//read factory file to mcu.
	}
	
	return res;
	
BSLCleanUp:
//			MSP430BSL_cleanUpPointer(firmwareImage);
	iProgrammingFail++;
	pr_err("[MCU] INFO: Programmed fail, %d/%d.\n", iProgrammingFail, iProgrammingCounter);

	//UPDATE_FW_RETRY_COUNT
	if(iProgrammingFail >= UPDATE_FW_RETRY_COUNT){
		pr_err("[MCU] Try 3 times to update new FW, the result is also fail.\n");
		MCUState = MCU_READY;	//although update fail, but need let user can control MCU.
	}
	
	//Zen7, power off/on when go to bsl mode fail.
	MSP430FR2311_power_control(0);
	//gpio_set_value(mcu_info->mcu_5v_boost_enable, 0);
	msleep(5);
	pr_err("[MCU] re-power on the device when run bsl logic fail.\n");
	MSP430FR2311_power_control(1);
	//gpio_set_value(mcu_info->mcu_5v_boost_enable, 1);
	msleep(200);
	
	return res;

}


static int MSP43FR2311_Update_Firmware(void) {
	int rc=0;
	//if (g_ASUS_hwID == HW_REV_ER ) return MSP43FR2311_Update_Firmware_Load_File(0);

	rc=MSP43FR2311_Update_Firmware_Load_File(1);
	if (rc != MSP430_STATUS_INVOKE_FAIL) {
			MSP430BSL_cleanUpPointer();
	} 
	return rc;
}


int MSP430FR2311_Get_Steps() {
	char getsteps[] = { 0xAA, 0x55, 0x10};
	char steps[] = { 0, 0};
//	int i=0;
	
//	for(i=0;i<2;i++, msleep(10)) 
	if ( !MSP430_I2CWriteA(MSP430_READY_I2C, getsteps, sizeof(getsteps)) |!MSP430_I2CRead(MSP430_READY_I2C, steps, sizeof(steps)) ) {
		pr_err("[MCU] %s I2C error!", __func__);
		return -1;
	}

	pr_err("[MCU] Current Steps=%d", steps[1]<<1);
	return steps[1];
}


void MSP430FR2311_wakeup(uint8_t enable) {
	if (enable) {
		//power up
		iOpenCounter++;
		if (iOpenCounter==1) {				
			//if (g_ASUS_hwID == HW_REV_ER  || g_ASUS_hwID == HW_REV_SR) {
			//} else {
				gpio_set_value(mcu_info->mcu_wakeup, 0);
				//msleep(1);
			//}
		} 
	}  else {
		iCloseCounter++;
		if (iCloseCounter==1) {
			queue_delayed_work(mcu_info->mcu_wq, &report_work, msecs_to_jiffies(powerDownDuration));
		} else  {
			mod_delayed_work(mcu_info->mcu_wq, &report_work, msecs_to_jiffies(powerDownDuration));
		}
	}
}

int MSP430FR2311_Get_Version(char * version) {
	char i2cfwversion[] = { 0xAA, 0x55, 0x0A};
	unsigned char i;

	MSP430FR2311_wakeup(1);
	for(i=0; i<3; i++){
		if (!MSP430_I2CWriteA(MSP430_READY_I2C, i2cfwversion, sizeof(i2cfwversion))  | !MSP430_I2CWriteReadA(MSP430_READY_I2C, i2cfwversion, sizeof(i2cfwversion), version, 4)) {
			pr_err("[MCU] %s I2C error!", __func__);
			MSP430FR2311_wakeup(0);
			return -1;
		}

		if(gFWVersion[0] == 20){	//Compare with 20 because make sure that: MCU version format in fw is 20 xx xx xx.
			i = 3;
		}else{
			pr_err("[MCU] get error fw version:0x%x %x %x %x.\n", gFWVersion[0], gFWVersion[1], gFWVersion[2], gFWVersion[3]);
		}
	}
	MSP430FR2311_wakeup(0);
	
	D("[MCU] get fw version:0x%x %x %x %x.\n", gFWVersion[0], gFWVersion[1], gFWVersion[2], gFWVersion[3]);
	return 0;
}

#define MCU_SHOW_INFO_IN_SETTING
#ifdef MCU_SHOW_INFO_IN_SETTING
//#include "../power/supply/qcom/fg-core.h"
//#include "../power/supply/qcom/fg-reg.h"
//#include "../power/supply/qcom/fg-alg.h"
#include <linux/extcon-provider.h>
extern void asus_extcon_set_fnode_name(struct extcon_dev *edev, const char *fname);
extern void asus_extcon_set_name(struct extcon_dev *edev, const char *name);

struct extcon_dev *mcu_ver_extcon;
char mcuVersion[13];
static const unsigned int asus_motor_extcon_cable[] = {
	EXTCON_NONE,
};

void registerMCUVersion() {
	int rc=0;
	mcu_ver_extcon = extcon_dev_allocate(asus_motor_extcon_cable);
	if (IS_ERR(mcu_ver_extcon)) {
		rc = PTR_ERR(mcu_ver_extcon);
		pr_err("[MCU] failed to allocate ASUS mcu_ver_extcon device rc=%d\n", rc);
	}
	asus_extcon_set_fnode_name(mcu_ver_extcon, "mcu");
	rc = extcon_dev_register(mcu_ver_extcon);
	if (rc < 0) {
		pr_err("[MCU] failed to register ASUS mcu_ver_extcon device rc=%d\n", rc);
	}
	sprintf(mcuVersion, "%d%02d%02d%02X",  gFWVersion[0],gFWVersion[1],gFWVersion[2],gFWVersion[3]);
	asus_extcon_set_name(mcu_ver_extcon, mcuVersion);
}
#endif

int MSP430FR2311_Check_Version(void) {
	memset(gFWVersion, 0x0, sizeof(gFWVersion));
	if (MSP430FR2311_Get_Version(gFWVersion)==MSP430_STATUS_OPERATION_OK){
		tMSPMemorySegment* tTXTFile = NULL;
		
		//if (g_ASUS_hwID == HW_REV_ER) {
		//	tTXTFile = MSP430BSL_parseTextFile();
		//} else {
			tTXTFile = read_firmware_file();	//zen7 used.
		//}
		if (tTXTFile==NULL) {
			pr_err("[MCU] read firmware file error, can not to check firmware version");
			
//			pr_err("[MCU] seLinux security issue, workaround update firmware manually");
//			MCUState=MCU_READY;			
//			g_motor_status = 1; //probe success
			
			return MSP430_STATUS_TXTFILE_ERROR;
		}
		pr_err("[MCU] Firmware version=%d%02d%02d%02X, new version=%d%02d%02d%02X\n", 
			gFWVersion[0], gFWVersion[1], gFWVersion[2], gFWVersion[3],
			tTXTFile->ui8Buffer[0],
			tTXTFile->ui8Buffer[2],
			tTXTFile->ui8Buffer[4],
			tTXTFile->ui8Buffer[6]	);

		if (tTXTFile->ui8Buffer[1]||
			tTXTFile->ui8Buffer[3]||
			tTXTFile->ui8Buffer[5]||
			tTXTFile->ui8Buffer[7]) {
			pr_err("[MCU] Firmware formate is incorrect\n"); 					
		}
		
		if (
			tTXTFile->ui8Buffer[0]*100000
			+tTXTFile->ui8Buffer[2]*1000
			+tTXTFile->ui8Buffer[4]*10
			+tTXTFile->ui8Buffer[6] ==
			gFWVersion[0]*100000
			+gFWVersion[1]*1000 
			+gFWVersion[2]*10 
			+gFWVersion[3]
			) {

			MCUState=MCU_READY;	//loopCounter=2 and MCUState=4, switch to MCU_READY.			
			g_motor_status = 1; //probe success
			pr_err("[MCU] fw is newest, not need to update.\n"); 
			
			//if (g_ASUS_hwID != HW_REV_ER) {
				read_cali_file();
			//}

#ifdef MCU_SHOW_INFO_IN_SETTING
			registerMCUVersion();
#endif			
		} else {
			pr_err("[MCU] Firmware need to be updated.\n"); 
		}

		//if (g_ASUS_hwID != HW_REV_ER) {
			MSP430BSL_cleanUpPointer();
		//} 

	} else {
		pr_err("[MCU] Firmware version get error.\n");
		return MSP430_STATUS_I2C_NOT_FOUND;
	}
	return 0;
}


int loopCounter=0;
int totalLoopCounter=0;



void mcu_loop_test(void) {
//	pr_err("[MCU] do loop test, loop=%d, wake up=%d", loop_i, loop_i&1);
//	gpio_set_value(mcu_info->mcu_wakeup, loop_i&1);
	int delay=1500;
	pr_err("[MCU] do loop test, loop=%d/%d", loopCounter, totalLoopCounter);
	AutoEmergencyMode[0]=0;
	MSP430FR2311_Set_ParamMode(AutoEmergencyMode);
	if (AutoEmergencyMode[1]<600) {
		delay=delay*600/AutoEmergencyMode[1];
	}
	msleep(delay);
	AutoEmergencyMode[0]=1;
	MSP430FR2311_Set_ParamMode(AutoEmergencyMode);
	msleep(delay);
}

int MSP430FR2311_Pulldown_Drv_Power() {
	/*
	char MSP430PullDownDrvMode[]={0xAA, 0x55, 0x0E, 0x00};	

	if (MCUState<MCU_READY) {
		pr_err("[MCU] Not ready!, state=%d", MCUState);
		return -MCUState;
	}
	if (!MSP430_I2CWriteA(MSP430_READY_I2C, MSP430PullDownDrvMode, sizeof(MSP430PullDownDrvMode))) {
		pr_err("[MCU] %s I2C error!", __func__);
		return -1;
	}*/		
	D("[MSP430FR2311] %s\n", __func__);
	return 0;
	
}


void mcu_do_later_power_down() {

	mutex_lock(&MSP430FR2311_control_mutex);
	if (iCloseCounter!=0) {
		MSP430FR2311_Pulldown_Drv_Power();
		gpio_set_value(mcu_info->mcu_wakeup, 1);
		pr_err("[MCU] motor power down, OpenClient=%d, CloseClient=%d", iOpenCounter, iCloseCounter);	
		iCloseCounter=0;
		iOpenCounter=0;
	} else {
		pr_err("[MCU] motor power down Ignore, OpenClient=%d, CloseClient=%d", iOpenCounter, iCloseCounter);	
	}
	mutex_unlock(&MSP430FR2311_control_mutex);		

}


static void mcu_do_work_later(struct work_struct *work)
{
	static unsigned char scinitFlg = 0; 
		
	loopCounter++;
	pr_err("[MCU] %s loopCounter=%d, state=%d",__func__, loopCounter, MCUState);	

	//Patch for EVB board, reduce redundancy log. Need remove in furture.
	if((loopCounter >= 5) && (MCUState == MCU_EMPTY)){
		pr_err("[MCU] %s quit.\n", __func__); 
		return;
	}
		
	if (MCUState!=MCU_READY) {
		if (MCUState==MCU_EMPTY) { 
			if (MSP430FR2311_Check_Version()) {
				pr_err("[MCU] %s: Fail.\n", __func__); 	
			}else{
				if(MCUState == MCU_READY){		//If first time check FW don't need update.
					cancel_delayed_work(&report_work);
					queue_delayed_work(mcu_info->mcu_wq, &report_work, 100);	//Cue to run mStopConditionInit().
					return;
				}
			}
		}
		
		if (MCUState==MCU_CHECKING_READY) {
			if (MSP430FR2311_Check_Version()!=0 && loopCounter <UPDATE_FW_RETRY_COUNT) {	//if this condition true, no any help in this handle.
				MCUState=MCU_PROGRAMMING;
				queue_delayed_work(mcu_info->mcu_wq, &report_work, mcu_info->mcu_polling_delay);		 
			} else {
				if(MCUState == MCU_READY){		//If check FW has been update success.
					cancel_delayed_work(&report_work);
					queue_delayed_work(mcu_info->mcu_wq, &report_work, 100);	//Cue to run mStopConditionInit().
					return;
				}
			}
			return;
		}
		/*
		if (MCUState==MCU_WAIT_POWER_READY) {
			D("[MCU] power ready");
			MCUState=MCU_READY;
			return;
		}

		if (MCUState==MCU_LOOP_TEST) {
			//loop i2c
			if (loopCounter < totalLoopCounter) {
				mcu_loop_test();
				queue_delayed_work(mcu_info->mcu_wq, &report_work, mcu_info->mcu_polling_delay/2); //60 secs 				
			} else {
				MCUState=MCU_READY;
			}
			return;
		}*/
		
		if (MCUState!=MCU_READY && MSP43FR2311_Update_Firmware()==0) {	//If update FW fail, will retry 3 times in function MSP43FR2311_Update_Firmware().
			MCUState=MCU_CHECKING_READY;
		}

		//finally, we do not update firmware successfully, do this again
		if (MCUState<=MCU_CHECKING_READY  && loopCounter <UPDATE_FW_RETRY_COUNT) {
			cancel_delayed_work(&report_work);
			queue_delayed_work(mcu_info->mcu_wq, &report_work, 100);	//Let MCUState change to MCU_READY in next cycle (function: MSP430FR2311_Check_Version()).
		} else {
//			pr_err("[MCU] FATAL, mcu firmware update fail !!!! loopCounter=%d, state=%d", loopCounter, MCUState); 
		}
	}  else {	
		if(scinitFlg == 0){
			scinitFlg = 1;	//Only init once when MCU_READY。
			mStopConditionInit();
		}
		mcu_do_later_power_down();

	}
}


static int mcu_open(struct inode *inode, struct file *file)
{
	D("[MCU] %s\n", __func__);

	return 0;
}

static int mcu_release(struct inode *inode, struct file *file)
{
	D("[MCU] %s\n", __func__);

	return 0;
}

static long mcu_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	int auto_mode = 0;
	int ret = 0;
	char nameMotor[ASUS_MOTOR_NAME_SIZE];
	motorDrvManualConfig_t data;
	motorCal gAngle;
	uint8_t wBuf[9] = {0xAA, 0x55, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	float gPrecision = 3.456; 
	unsigned int sbuf[2];	
	MICRO_STEP microstep;
	int special_angle = 0;
	int JudgeStopConditionFlg = 0;	
	int UserKThreshold = 0;

	pr_err("[MCU] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
		case ASUS_MOTOR_DRV_AUTO_MODE:
			ret = copy_from_user(&auto_mode, (int __user*)arg, sizeof(auto_mode));
			pr_err("[MCU] %s auto_mode:%d.\n", __func__, auto_mode);
			if(ret < 0 ){
				pr_err("%s: cmd = ASUS_MOTOR_DRV_AUTO_MODE, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}

			ret = MSP430FR2311_Set_AutoMode(auto_mode);
			if(ret < 0)
				pr_err("Set AutoMode failed\n");

			break;
			
		case ASUS_MOTOR_DRV_MANUAL_MODE:
			ret = copy_from_user(&data, (int __user*)arg, sizeof(data));
			if(ret < 0 ){
				pr_err("%s: cmd = ASUS_MOTOR_DRV_MANUAL_MODE, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}
			
			pr_err("[MCU] %s manual mode dir:%d, angle:%d, speed:%d.\n",  __func__, data.dir, data.angle, data.speed);
			ret = MSP430FR2311_Set_ManualMode(data.dir, data.angle, data.speed);
			if(ret < 0)
				pr_err("Set ManualMode failed\n");

			break;

		case ASUS_MOTOR_DRV_MANUAL_MODE_NOSS:
			ret = copy_from_user(&data, (int __user*)arg, sizeof(data));
			if(ret < 0 ){
				pr_err("%s: cmd = ASUS_MOTOR_DRV_MANUAL_MODE_NOSS, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}
			
			pr_err("[MCU] %s NOSS manual mode dir:%d, angle:%d, speed:%d.\n",  __func__, data.dir, data.angle, data.speed);
			ret = ManualMode_AfterAndPR2_NOSS(data.dir, data.angle, data.speed);
			if(ret < 0)
				pr_err("Set NOSS ManualMode failed\n");

			break;
			
		case ASUS_MOTOR_DRV_AUTO_MODE_WITH_ANGLE:
			ret = copy_from_user(&data, (int __user*)arg, sizeof(data));
			if(ret < 0 )
			{
				pr_err("%s: cmd = ASUS_MOTOR_DRV_MANUAL_MODE, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}
			ret = MSP430FR2311_Set_AutoModeWithAngle(data.dir, data.angle);
			if(ret < 0)
				pr_err("Set AutoModeWithAngle failed\n");
		
			break;
			
		case ASUS_MOTOR_DRV_STOP:
			ret = MSP430FR2311_Stop();
			if(ret < 0)
				pr_err("Stop Motor failed\n");
			break;
			
		case ASUS_MOTOR_DRV_GET_STEPS:
			ret = MSP430FR2311_Get_Steps();
			if(ret < 0) {
				pr_err("Get Motor steps failed\n");	
				goto end;
			}
			ret = copy_to_user((int __user*)arg, &ret, sizeof(ret));
			break;
				
		case ASUS_MOTOR_DRV_GET_NAME:
		    snprintf(nameMotor, sizeof(nameMotor), "%s", ASUS_MOTOR_DRV_DEV_PATH);
			D("%s: cmd = MODULE_NAME, name = %s\n", __func__, nameMotor);
			ret = copy_to_user((int __user*)arg, &nameMotor, sizeof(nameMotor));
			break;
			
		case ASUS_MOTOR_DRV_CLOSE:
			D("[MCU]ASUS_MOTOR_DRV_CLOSE+++, ask mcu power down immediately\n");			
			flush_delayed_work(&report_work);
			D("[MCU]ASUS_MOTOR_DRV_CLOSE---, ask mcu power down immediately\n");			
			break;

		case ASUS_MOTOR_DRV_SET_ANGLE:
			ret = copy_from_user(&gAngle, (int __user*)arg, sizeof(gAngle));
			if(ret < 0 ){
				pr_err("%s: cmd = ASUS_MOTOR_DRV_SET_ANGLE, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}
			
			pr_err("[MCU] %s phone set angle:%d %d.\n",  __func__, gAngle.integer, gAngle.decimals);
			wBuf[2] = 0x81;
			wBuf[3] = (gAngle.integer >> 8);
			wBuf[4] = (gAngle.integer&0x00FF);
			wBuf[5] = (gAngle.decimals >> 8);
			wBuf[6] = (gAngle.decimals&0x00FF);
			ret = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, sizeof(wBuf));
			if(ret < 0)
				pr_err("[MCU] Set angle to mcu failed\n");
			break;

		case ASUS_MOTOR_CALIBRATION:
			ret = copy_from_user(&cal_cmd, (int __user*)arg, sizeof(cal_cmd));			
			if(ret < 0 ){
				pr_err("%s: cmd = ASUS_MOTOR_CALIBRATION, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}

			if(cal_cmd == 0)
				FacOrUserClaFlg = 0;	//Fac
			else
				FacOrUserClaFlg = 1;	//User

			pr_err("[MCU] %s phone set calibration:%d %d.\n", __func__, cal_cmd, FacOrUserClaFlg);
	 
			wBuf[2] = 0x61;
			wBuf[3] = CaliCmd[0];
			wBuf[4] = CaliCmd[1];
			wBuf[5] = CaliCmd[2];
			ret = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, 6);		//2header + 4bytes.
			if(ret < 0){
				UserKState = UserK_TBD;
				FacOrUserClaFlg = 0;	//default factory K.
				pr_err("[MCU] Motor calibration failed.\n");
			}else{
				if(FacOrUserClaFlg == 1)	//Need add judge condition (&& UserKState == UserK_TBD)  ????
					UserKState = UserK_Start;
			}
			break;

		case ASUS_MOTOR_GET_MICRO_STEP:
		    //snprintf(nameMotor, sizeof(nameMotor), "%s", ASUS_MOTOR_DRV_DEV_PATH);
		    gPrecision = 0.144*4*SmallAngle[7];
			sbuf[0] = (unsigned int)gPrecision;
			sbuf[1] = ((unsigned int)(gPrecision*1000))%1000;
			pr_err("[MCU] min_angle:%d.%d\n", sbuf[0], sbuf[1]);
			ret = copy_to_user((int __user*)arg, &sbuf, sizeof(microstep));
			break;

		case ASUS_MOTOR_SPECIAL_ANGLE:
			ret = copy_from_user(&special_angle, (int __user*)arg, sizeof(special_angle));
			pr_err("[MCU] %s phone set special_angle:%d.\n", __func__, special_angle);
			if(ret < 0 ){
				pr_err("%s: cmd = ASUS_MOTOR_SPECIAL_ANGLE, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}
			
			wBuf[2] = 0xFE;
			wBuf[3] = special_angle;
			ret = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, 4);		//2header + 2bytes.
			if(ret < 0)
				pr_err("[MCU] Motor special_angle failed.\n");
			break;

		case ASUS_MOTOR_RESET_USER_CAL:
			pr_err("[MCU] %s phone reset calibration value, UserKState:%d.\n", __func__, UserKState);
			UserKState = UserK_Reset;
			queue_delayed_work(mcu_info->cal_wq, &cal_work, 20);  //20ms	

			break;

		case ASUS_MOTOR_SAVE_USER_CAL:
			pr_err("[MCU] %s phone save calibration value, UserKState:%d.\n", __func__, UserKState);

			if(UserKState == UserK_rMCU_Ready){
				UserKState = UserK_SaveK;
				queue_delayed_work(mcu_info->cal_wq, &cal_work, 20);	
			}
			break;

		case ASUS_MOTOR_NOTSAVE_USER_CAL:
			pr_err("[MCU] %s phone don't save calibration value, UserKState:%d.\n", __func__, UserKState);
			if(UserKState == UserK_rMCU_Ready){
				UserKState = UserK_NoSaveK;
				queue_delayed_work(mcu_info->cal_wq, &cal_work, 20);	
			}

			break;
		
		case ASUS_MOTOR_ABROT_USER_CAL:
			pr_err("[MCU] %s phone abrot calibration, UserKState:%d.\n", __func__, UserKState);
			if(UserKState == UserK_Start){
				UserKState = UserK_AbortK;
				queue_delayed_work(mcu_info->cal_wq, &cal_work, 20);	
			}	

			break;	
			
		case ASUS_MOTOR_JUDGEANGLECONDITION:
			ret = copy_from_user(&JudgeStopConditionFlg, (int __user*)arg, sizeof(JudgeStopConditionFlg));			
			if(ret < 0 ){
				pr_err("%s: cmd = ASUS_MOTOR_JUDGEANGLECONDITION, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}

			wBuf[2] = 0x87;
			wBuf[3] = JudgeStopConditionFlg;	//1:MCU will not use judge angle to stop motor rotate.  0:use judge angle to stop motor.
			ret = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, 4); 	//2header + 2bytes.
			if(ret < 0)
				pr_err("[MCU] ioctl disable judge angle failed.\n");
		
			pr_err("[MCU] %s phone set jasc:%d, %s jasc function.\n", __func__, JudgeStopConditionFlg, JudgeStopConditionFlg?"not use":"use");
		
			break;

		case ASUS_MOTOR_SETUSERKTHRESHOLD:
			ret = copy_from_user(&UserKThreshold, (int __user*)arg, sizeof(UserKThreshold));			
			if(ret < 0 ){
				pr_err("%s: cmd = ASUS_MOTOR_SETUSERKTHRESHOLD, copy_from_user error(%d)\n", __func__, ret);
				goto end;
			}
		
			wBuf[2] = 0x88;
			wBuf[3] = UserKThreshold;	
			ret = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, 4); 	//2header + 2bytes.
			if(ret < 0)
				pr_err("[MCU] UserKThreshold write failed.\n");
		
			pr_err("[MCU] %s phone set UserKThreshold:%d.\n", __func__, UserKThreshold);
		
			break;

		case ASUS_MOTOR_MCU_STATE:
			ret = MCUState;
			ret = copy_to_user((int __user*)arg, &ret, sizeof(ret));

			pr_err("[MCU] %s phone get mcu state:%d, ret:%d.\n", __func__, MCUState, ret);
			break;
		
		default:
			pr_err("[MCU][MSP430FR2311 error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			return -EINVAL;
	}
end:
	return ret;
}

static const struct file_operations mcu_fops = {
	.owner = THIS_MODULE,
	.open = mcu_open,
	.release = mcu_release,
	.unlocked_ioctl = mcu_ioctl
};

struct miscdevice mcu_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "asusMotoDrv",
	.fops = &mcu_fops
};


static int mcu_setup(void)
{
	int ret;


	ret = misc_register(&mcu_misc);
	if (ret < 0) {
		pr_err(
			"[MCU] %s: could not register mcu misc device\n",
			__func__);
	}

	return ret;

}

#define F_Status_AKMAngleChange			bit0
#define F_Status_MotorRoateState		bit1
#define F_Status_AKMThreadInt	 		bit2
#define F_Status_MotorKMCUCal	 	 	bit3
#define F_Status_MotorKMCUThred			bit4
#define F_Status_MotorRoateForceStop	bit5	//#define F_Status_DrvInt	 	 			bit5
#define F_Status_ReqG_Angle 			bit6
#define F_Status_ReqG_Timeout			bit7

#define extract_mcu_len	2
static void extract_mcu_data(void){
	unsigned char rBuf[6] = {0,0,0,0,0,0};
	uint8_t CmdBuf[3] = {0xAA, 0x55, 0xE0};
	uint8_t EventState = 0;

	CmdBuf[2] = 0xE0;
	if (!MSP430_I2CWriteA_NoLog(MSP430_READY_I2C, CmdBuf, sizeof(CmdBuf)) | !MSP430_I2CRead_NoLog(MSP430_READY_I2C, rBuf, extract_mcu_len)) {
		pr_err("[MCU] %s i2c read error!\n", __func__);
		return;
	}else{
		EventState = rBuf[0];
		
		if(IS_MASK_SET(EventState, F_Status_AKMAngleChange)){		//report angle.
			CmdBuf[2] = 0xE1;
			if(!MSP430_I2CWriteA_NoLog(MSP430_READY_I2C, CmdBuf, sizeof(CmdBuf)) | !MSP430_I2CRead_NoLog(MSP430_READY_I2C, rBuf, 6)){
				pr_err("[MCU] %s cmd:0xE1 i2c read error!\n", __func__);
			}else{
				report_motor_event(MOTOR_ANGLE, ((rBuf[2]<<24) | (rBuf[3]<<16) | (rBuf[4]<<8)| rBuf[5]));
			}
			//pr_err("[MCU] angle:%x.\n", ((rBuf[2]<<24) | (rBuf[3]<<16) | (rBuf[4]<<8)| rBuf[5]));
		}

		if(IS_MASK_SET(EventState, F_Status_MotorRoateState)){		//roate finish.
			report_motor_event(MOTOR_ROTATE, ROTATE_FINISH);
			pr_err("[MCU] INT:roate finish.\n");
		}	

		if(IS_MASK_SET(EventState, F_Status_AKMThreadInt)){		//akm interrupt.
			report_motor_event(MOTOR_FORCE, rBuf[2]);
			pr_err("[MCU] INT:akm interrupt trigger.\n");
		}
		
		//Notify IMS force stop cmd success. 			  
		if(IS_MASK_SET(EventState, F_Status_MotorRoateForceStop)){		//roate force stop.
			report_motor_event(MOTOR_ROTATE, ROTATE_STOP);
			pr_err("[MCU] INT:Phone force rotate stop trigger.\n");
		}

		if(FacOrUserClaFlg == 0){
			if(IS_MASK_SET(EventState, F_Status_MotorKMCUCal)){		//Calibration and threadhold data backup.
				if(!Zen7_MSP430FR2311_rI2CtoCPU(0x70, &(fcal_val.Cal_val[0]), CalLength)){				
					backup_mcu_cal_thed();
					bak_raw();
					report_motor_event(MOTOR_K_FINISH, 0);
					pr_err("[MCU] INT:backup calibration data trigger.\n");
				}else
					pr_err("[MCU] INT:%s i2c backup calibration read error!\n", __func__);			
			}

			if(IS_MASK_SET(EventState, F_Status_MotorKMCUThred)){		//Calibration and threadhold data backup.
				if(!Zen7_MSP430FR2311_rI2CtoCPU(0x70, &(fcal_val.Cal_val[0]), CalLength)){				
					backup_mcu_cal_thed();
					pr_err("[MCU] INT:backup theadhold data trigger.\n");
				}else
					pr_err("[MCU] INT:%s i2c backup theadhold read error!\n", __func__);			
			}
		}else{	//User K
			if(IS_MASK_SET(EventState, F_Status_MotorKMCUCal)){		//Calibration and threadhold data backup.
				if(UserKState == UserK_Start){
					if(!Zen7_MSP430FR2311_rI2CtoCPU(0x70, &(fcal_val_UserK.Cal_val[0]), CalLength)){				
						UserKState = UserK_rMCU_Ready;
						report_motor_event(MOTOR_K_FINISH, 0);
						pr_err("[MCU] INT:backup userK calibration data trigger.\n");
					}else
						pr_err("[MCU] INT:%s read userK calibration data error!\n", __func__);		
										
				}else{
					UserKState = UserK_TBD;
					pr_err("[MCU] INT: missing UserK interrupt.\n");
				}
				
				FacOrUserClaFlg = 0;	//Default fac K.
			}
		}
		/*
		//fault_status_reg, diag_status1_reg, diag_status2_reg.			      
		if(IS_MASK_SET(EventState, F_Status_DrvInt)){		//Drv interrupt.
			report_motor_event(MOTOR_DRVINT, ((rBuf[2]<<16) | (rBuf[3]<<8) | rBuf[4]));
			pr_err("[MCU] INT:Drv interrupt trigger.\n");
		}*/
		
		//Notify IMS that MCU need get current angle.			      
		if(IS_MASK_SET(EventState, F_Status_ReqG_Angle)){		//get g-sensor's angle.
			report_motor_event(MOTOR_GET_ANGLE, 0);
			pr_err("[MCU] INT:Get g-sensor's angle trigger.\n");
		}

		//Notify IMS that MCU get current angle timeout.			      
		if(IS_MASK_SET(EventState, F_Status_ReqG_Timeout)){		//get g-sensor's angle timeout.
			report_motor_event(MOTOR_GET_ANGLE, 1);
			pr_err("[MCU] INT:Get g-sensor's angle timeout trigger.\n");
		}
	}
}

static irqreturn_t mcu_interrupt_handler(int irq, void *dev_id)
{	
	//Only L2H level be dealt.
	if(gpio_get_value(mcu_info->mcu_int) == 1){
		//pr_err("[MCU] L2H trigger.\n");
		extract_mcu_data();		
	}//else
	//	D("[MCU] H2L trigger.\n");
		
	return IRQ_HANDLED;
}

static int init_irq(void){
	int ret = 0;

	/* GPIO to IRQ */
	mcu_info->mcu_irq = gpio_to_irq(mcu_info->mcu_int);
	
	if (mcu_info->mcu_irq < 0) {
		pr_err("[MCU] gpio_to_irq ERROR, irq=%d.\n", mcu_info->mcu_irq);
	}else {
		pr_err("[MCU] gpio_to_irq IRQ %d successed on GPIO:%d\n", mcu_info->mcu_irq, mcu_info->mcu_int);
	}

	ret = request_threaded_irq(mcu_info->mcu_irq, NULL, mcu_interrupt_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"mcu_int", mcu_info);

	if (ret < 0){
		free_irq(mcu_info->mcu_irq, mcu_info);
		pr_err("[MCU] request_irq() ERROR %d.\n", ret);
	}else {
		pr_err("[MCU] Enable irq !! \n");
		enable_irq_wake(mcu_info->mcu_irq);
	}
	
	return 0;	
}

static int initial_MSP430FR2311_gpio(void)
{
	int ret;

		ret = gpio_request(mcu_info->mcu_reset, "gpio_msp430fr423111_reset");
		if (ret < 0) {
			pr_err("[MCU] %s: gpio %d request failed (%d)\n",
				__func__, mcu_info->mcu_reset, ret);
			return ret;
		}
	
		ret = gpio_request(mcu_info->mcu_test, "gpio_msp430fr423111_test");
		if (ret < 0) {
			pr_err("[MCU] %s: gpio %d request failed (%d)\n",
				__func__, mcu_info->mcu_test, ret);
			return ret;
		}
		
		ret = gpio_direction_output(mcu_info->mcu_reset, 1);
		if (ret < 0) {
			pr_err(
				"[MCU] %s: fail to set gpio %d as input (%d)\n",
				__func__, mcu_info->mcu_reset, ret);
		gpio_free(mcu_info->mcu_reset);
		return ret;
		}
		
		ret = gpio_direction_output(mcu_info->mcu_test, 0);
		if (ret < 0) {
			pr_err(
				"[MCU] %s: fail to set gpio %d as input (%d)\n",
				__func__, mcu_info->mcu_test, ret);
			 gpio_free(mcu_info->mcu_test);
			 return ret;
		}	


	ret = gpio_request(mcu_info->mcu_wakeup, "gpio_msp430fr423111_wakeup");
	if (ret < 0) {
		pr_err("[MCU] %s: gpio %d request failed (%d)\n",
			__func__, mcu_info->mcu_wakeup, ret);
		return ret;
	}
	
	ret = gpio_direction_output(mcu_info->mcu_wakeup, 0);
	if (ret < 0) {
		pr_err(
			"[MCU] %s: fail to set gpio %d as input (%d)\n",
			__func__, mcu_info->mcu_wakeup, ret);
	gpio_free(mcu_info->mcu_wakeup);
	return ret;
	}

	ret = gpio_request(mcu_info->mcu_int, "gpio_msp430fr2155_int");
	if (ret < 0) {
		pr_err("[MCU] %s: gpio %d request failed (%d)\n",
			__func__, mcu_info->mcu_int, ret);
		return ret;
	}
	
	ret = gpio_direction_input(mcu_info->mcu_int);
	if (ret < 0) {
		pr_err(
			"[MCU] %s: fail to set gpio %d as input (%d)\n",
			__func__, mcu_info->mcu_int, ret);
		gpio_free(mcu_info->mcu_int);
		return ret;
	}


//		if (g_ASUS_hwID < HW_REV_PR)	 {
//			gpio_set_value(mcu_info->mcu_wakeup, 1);			
//		}
	return ret;
}
	
int MCU_I2C_power_control(bool enable)
{
        int ret = 0;
        pr_err("[MCU] %s Ldo10a try to :%s.\n", __func__, enable?"enable":"disable");
		/*
		mcu_info->vcc_l10a_3p3 = regulator_get(&mcu_info->i2c_client->dev, "vcc_l10a_3p3");
		if (IS_ERR(mcu_info->vcc_l10a_3p3))
		{
			ret = PTR_ERR(mcu_info->vcc_l10a_3p3);
			pr_err("[MCU] Regulator get failed vdd ret=%d", ret);
			return ret;
		}*/

		
		
        if (enable){
			ret = regulator_is_enabled(mcu_info->vcc_l10a_3p3);
			if(ret){
				pr_err("[MCU] Ldo10a has enabled, ingore this action.\n");
				return ret;
			}
			
			ret = regulator_enable(mcu_info->vcc_l10a_3p3);
			if(ret) 
				pr_err("[MCU] Ldo10a enable failed vcc_l10a_3p3 ret=%d\n", ret);        
        }else{
        	ret = regulator_is_enabled(mcu_info->vcc_l10a_3p3);
			if(ret){
				pr_err("[MCU] Ldo10a is enabled, so disable it.\n");
				
				ret = regulator_disable(mcu_info->vcc_l10a_3p3);
				if(ret) 
					pr_err("[MCU] Ldo10a disable failed vcc_l10a_3p3 ret=%d\n", ret);
			}
        }

        return ret;
}


int MSP430FR2311_power_control(uint8_t enable)
{
        int ret = 0;

//        pr_info("[MCU] %s, enable=%d +\n", __func__, enable);

		if (!enable) {
			//power down
//			gpio_set_value(mcu_info->mcu_wakeup, 1);
			gpio_set_value(mcu_info->mcu_reset, enable);
			gpio_set_value(mcu_info->mcu_test, 0);
//			msleep(powerDownDuration); //wait motor end
		}

		if (!enable)
			MCU_I2C_power_control(enable);

		g_motor_power_state = enable;
		/*
		#ifdef MCU_5V_ALWAYS_ON
		if (g_ASUS_hwID == HW_REV_ER	|| g_ASUS_hwID == HW_REV_SR ) 
		#endif
		{
			gpio_set_value(mcu_info->mcu_5v_boost_enable, enable);
			ret=gpio_get_value(mcu_info->mcu_5v_boost_enable);
			if (ret!=enable) {
				pr_err("[MCU] 3V enable failed ret=%d\n", ret);
				g_motor_power_state = 0;
			}
		}*/
		if (enable) {
			//power up
//			for ( i=0;i<1000;i++)	udelay(powerUpDuration);
			gpio_set_value(mcu_info->mcu_reset, enable);
			gpio_set_value(mcu_info->mcu_test, 0);
//			msleep(powerUpDuration);
//			gpio_set_value(mcu_info->mcu_wakeup, 0);
//			for ( i=0;i<1000;i++)	udelay(1);
			MCU_I2C_power_control(enable);
		}	

 //       pr_info("[MCU] %s -\n", __func__);
        return ret;
}

int FrqConvertMode(int dir, int angle, int speed) {
	uint16_t MotorDefault[]={0, 120, 120, 56, 28, 20, 12, 255, 47, 2, 2, 2, 40, 0, 0, 73, 73, 73, 73};
	uint16_t FS_steps = 0;
	int gearStep = LEAD_DELTA;
	
	memcpy(MotorDefault, ConvertFRQMode[dir], sizeof(MotorDefault));
	
	if (MCUState!=MCU_READY) {
		pr_err("[MCU] %s Not ready!, state=%d", __func__, MCUState);
		return -MCUState;
	}

	if (angle!=180){
		
		if(angle <= 10){	 //ASUS_MOTOR_DRV_AUTO_MODE_WITH_ANGLE, angle range is less than 10 degree.
			memcpy(MotorDefault, ConvertFRQModeForSmallAngle, sizeof(MotorDefault));	
			MotorDefault[0] = dir;
		
		}else{	//ASUS_MOTOR_DRV_AUTO_MODE_WITH_ANGLE, angle range is 10~180 degree.
			FS_steps = (CONVERT_FRQ_FS_STEP[dir])*angle/180;	
			//Step 0 + Step 1 + step5 equal total expect steps, other step don't move(steps = 0).		
			if(FS_steps <= 255){
				MotorDefault[7]  = FS_steps;
				MotorDefault[8]  = 0;
				MotorDefault[12] = (gearStep+CONVERT_FRQ_SS_STEP[dir]);	//(gearStep+CONVERT_FRQ_SS_STEP[dir])*angle/180;
			}else{
				MotorDefault[7] = 255;
				MotorDefault[8] = (FS_steps - 255);
				MotorDefault[12]= (gearStep+CONVERT_FRQ_SS_STEP[dir]);	//(gearStep+CONVERT_FRQ_SS_STEP[dir])*angle/180;
			}

			MotorDefault[9]  = 0;
			MotorDefault[10] = 0;
			MotorDefault[11] = 0;
			pr_err("[MCU] %s FS_steps:%d\n", __func__, FS_steps);
		}
	}else{
		//ASUS_MOTOR_DRV_AUTO_MODE or ASUS_MOTOR_DRV_AUTO_MODE_WITH_ANGLE(in condition:(mode==1 || mode==2) and angle==180).
	}
	
	pr_err("[MCU] PR2 auto control (Dir:%d, Angle:%d), param=%d, %d %d %d %d %d %d, %d %d %d %d %d %d, %d %d %d %d %d %d", dir, angle, MotorDefault[0],	\
		MotorDefault[1],MotorDefault[2],MotorDefault[3],MotorDefault[4],MotorDefault[5],MotorDefault[6],	\
		MotorDefault[7],MotorDefault[8],MotorDefault[9],MotorDefault[10],MotorDefault[11],MotorDefault[12],	\
		MotorDefault[13],MotorDefault[14],MotorDefault[15],MotorDefault[16],MotorDefault[17],MotorDefault[18]);	

	if(ZEN7)
		return Zen7_MSP430FR2311_Set_ParamMode(MotorDefault);
	else
		return MSP430FR2311_Set_ParamMode(MotorDefault);
 }



int MSP430FR2311_Set_AutoModeWithAngle(int mode, int angle) {

	if (mode==0xbf) {
		pr_err("[MCU] Burn firmware");
		if (MSP43FR2311_Update_Firmware_Load_File(1)==MSP430_STATUS_OPERATION_OK){
			MCUState=MCU_READY;
			MSP430BSL_cleanUpPointer();
			}
		return 0;
	}

	if (mode==0xdd) {
		MSP430FR2311_Check_Version();		
		return 0;
	}

	if (mode==222) {			 
		read_cali_file();
		return 0;
	}


	if(mode==242){ 
	 	int rc=0;
		uint16_t MotorDefault[]={1, 4, 4, 4, 4, 4, 4, 100, 0, 0, 0, 0, 0, 73, 73, 73, 73, 73, 73};
	
		memcpy(MotorDefault, TightenMode, sizeof(MotorDefault));
		if(ZEN7)
			rc= Zen7_MSP430FR2311_Set_ParamMode(MotorDefault);
		else
			rc= MSP430FR2311_Set_ParamMode(MotorDefault);
		msleep(5000);
		return rc;
	}

	//For camera app
	if(mode==243){ 	//0 to 180
	 	int rc=0;
		uint16_t MotorDefault[]={0, 4, 4, 4, 4, 4, 4, 6, 0, 0, 0, 0, 0, 73, 73, 73, 73, 73, 73};	
	
		memcpy(MotorDefault, SmallAngle, sizeof(MotorDefault));
		MotorDefault[0] = 0;
		rc= Zen7_MSP430FR2311_Set_ParamMode(MotorDefault);
		return rc;
	}

	if(mode==244){ 	//180 to 0
	 	int rc=0;
		uint16_t MotorDefault[]={1, 4, 4, 4, 4, 4, 4, 6, 0, 0, 0, 0, 0, 73, 73, 73, 73, 73, 73};	
	
		memcpy(MotorDefault, SmallAngle, sizeof(MotorDefault));
		MotorDefault[0] = 1;
		rc= Zen7_MSP430FR2311_Set_ParamMode(MotorDefault);
		return rc;
	}
	//For camera app

   	if(mode==1 || mode==2){
		//if (g_ASUS_hwID == HW_REV_ER  || g_ASUS_hwID == HW_REV_SR) {
		//	return MSP430FR2311_Set_ManualMode(--mode, angle, 6);
		//} else {
			return FrqConvertMode(--mode, angle, 6);		
		//}
   	}
	
	if(mode==3 || mode==4){ 
		AutoEmergencyMode[0]=mode-3;
		powerDownDuration=DEFAULT_POWERDOWNDURATION;
	
		if(ZEN7)
			return Zen7_MSP430FR2311_Set_ParamMode(AutoEmergencyMode);
		else
			return MSP430FR2311_Set_ParamMode(AutoEmergencyMode);
	}

	if(mode==5 || mode==6){ 
		AutoWarmUpMode[0]=mode-5;
		powerDownDuration=DEFAULT_POWERDOWNDURATION;
	
		if(ZEN7)
			return Zen7_MSP430FR2311_Set_ParamMode(AutoWarmUpMode);
		else
			return MSP430FR2311_Set_ParamMode(AutoWarmUpMode);
	}

	if(mode==7 || mode==8){ 
		AutoWarmUpMode2[0]=mode-7;
		powerDownDuration=DEFAULT_POWERDOWNDURATION;
	
		if(ZEN7)
			return Zen7_MSP430FR2311_Set_ParamMode(AutoWarmUpMode2);
		else	 
		 	return MSP430FR2311_Set_ParamMode(AutoWarmUpMode2);
	}

	pr_err("[MCU] Not supported mode for %d", mode);

	return -1;
#if 0	
	char MSP430AutoMode[]={0xaa, 0x55, 0x04, 0x01, 0, 0};
	pr_info("[MSP430FR2311] %s +\n", __func__);
	MSP430FR2311_wakeup(1);
	if (MCUState!=MCU_READY) {
		pr_err("[MCU] Not ready!, state=%d", MCUState);
		return -MCUState;
	}
	MSP430AutoMode[3]=mode;
	if (!MSP430_I2CWriteA(MSP430_READY_I2C, MSP430AutoMode, sizeof(MSP430AutoMode))) {
//		if (MCUState != MCU_LOOP_TEST)  MSP430FR2311_power_control(0);
		MSP430FR2311_wakeup(0);

		pr_err("[MCU] I2C error!");
		return -1;
	}
	
//	if (MCUState != MCU_LOOP_TEST) MSP430FR2311_power_control(0);
	MSP430FR2311_wakeup(0);

	pr_info("[MSP430FR2311] %s -\n", __func__);
	return 0;
	#endif
}

int MSP430FR2311_Set_AutoMode(int mode) {
	return MSP430FR2311_Set_AutoModeWithAngle(mode, 180);
}


int MSP430FR2311_Set_ParamMode(const uint16_t* vals) {
	char MSP430ParamMode[]={0xAA, 0x55, 0x0C, 0x00, 0x4B, 0x50, 0x57, 0x64, 0x57, 0x50, 0x0F, 0x1E, 0x32, 0x8C, 0xAF, 0xBE};

	int i=0;
	#ifdef ENABLE_LOOP_TEST
	if (vals[0]==-1 || vals[0]==65535) {
		memcpy(AutoEmergencyMode, vals, 13*sizeof (uint16_t));
		return 0;
	}

	if (vals[0]>=2 && vals[0]<=60000) {
		memcpy(AutoEmergencyMode, vals, 13*sizeof (uint16_t));
		MCUState=MCU_LOOP_TEST;		
		loopCounter=0;
		totalLoopCounter=vals[0]+1;
		queue_delayed_work(mcu_info->mcu_wq, &report_work, mcu_info->mcu_polling_delay);	
		return 0;
	}
	#endif
		pr_err("[MCU] %s +\n", __func__);
		mutex_lock(&MSP430FR2311_control_mutex);
		MSP430FR2311_wakeup(1);
		if (MCUState<MCU_READY) {
			pr_err("[MCU] Not ready!, state=%d", MCUState);
			mutex_unlock(&MSP430FR2311_control_mutex);
			return -MCUState;
		}

		powerDownDuration=0;
		for (i=1;i<7;i++) {
			uint16_t speed=vals[i]<<2;
//			const int defaultSpeedDuration=1200000;
			if (vals[i] < 50) {  //micro wave step
				switch (vals[i] ) {
					case 49:
						speed=88;
						break;
					case 39:
						speed=116;
						break;
					case 25:
						speed=165;
					break;
					case 20:
						speed=193;
					break;
					case 17:
						speed=232;
					break;
					case 8:
						speed=385;
					break;						
					case 7:
						speed=575;
					break;						
				}
				MSP430ParamMode[i+3]=vals[i]>>2;
			} else {
				MSP430ParamMode[i+3]=(vals[i]<<2)/50+50;			
			}
			powerDownDuration+=(vals[i+6]-((i==1)?0:vals[i+5]))*2400*500/speed/300;
//			pr_err("[MCU] Power duration += (%d-%d)*2400*500/%d/300 =   %d", vals[i+6], ((i==1)?0:vals[i+5]), speed, powerDownDuration);
		};
		pr_err("[MCU] Power duration=%d(ms), reference only", powerDownDuration);
		powerDownDuration=DEFAULT_POWERDOWNDURATION;
		bShowStopInfoOnce=1;
		
		MSP430ParamMode[3]=vals[0];
		MSP430ParamMode[10]=vals[7]>>1;
		MSP430ParamMode[11]=vals[8]>>1;
		MSP430ParamMode[12]=vals[9]>>1;
		MSP430ParamMode[13]=vals[10]>>1;
		MSP430ParamMode[14]=vals[11]>>1;
		MSP430ParamMode[15]=vals[12]>>1;
		
//		pr_err("[MCU] dump param=%d %d %d %d %d %d %d %d %d %d %d %d %d 254", 
//		vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7], vals[8], vals[9], vals[10], vals[11], vals[12]);
		
		if (!MSP430_I2CWriteA(MSP430_READY_I2C, MSP430ParamMode, sizeof(MSP430ParamMode))) {
	//		if (MCUState != MCU_LOOP_TEST)	MSP430FR2311_power_control(0);
			MSP430FR2311_wakeup(0);
			mutex_unlock(&MSP430FR2311_control_mutex);
	
			pr_err("[MCU] %s I2C error!", __func__);
			return -1;
		}
		
	//	if (MCUState != MCU_LOOP_TEST) MSP430FR2311_power_control(0);
		MSP430FR2311_wakeup(0);
		mutex_unlock(&MSP430FR2311_control_mutex);
	
		D("[MCU] %s -\n", __func__);
		return 0;
}

inline int MSP430FR2311_Set_ManualMode(int dir, int angle, int speed) {
	return (*fManualMode)(dir, angle, speed);
}

static char old_dir=-1;
int ManualMode_AfterAndPR2(int dir, int angle, int speed) {
	uint16_t MotorDefault[]={0, 120, 120, 120, 120, 120, 120, 255, 85, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint16_t manual_steps = 0;
	
	memcpy(MotorDefault, ConstSpeedMode, sizeof(MotorDefault));
	if (MCUState!=MCU_READY) {
		pr_err("[MCU] %s Not ready!, state=%d", __func__, MCUState);
		return -MCUState;
	}
	
	MotorDefault[0]=dir;
	if(angle!=180){
		int gearStep;
		gearStep=0;

		if(old_dir!=dir){
			gearStep+=LEAD_DELTA;
			//old_dir=dir;
		} 	
		
		//Not use 0 instead of formula. I wish that: every step can be set different speed in manual mode, but it seems IMS only set one speed.
		manual_steps = (CONVERT_FRQ_FS_STEP[dir])*angle/180;	//Maybe need modify ???
		if(manual_steps <= 255){
			MotorDefault[7]  = manual_steps;
			MotorDefault[8]  = 0;
			MotorDefault[12] = (gearStep+CONVERT_FRQ_SS_STEP[dir]);
		}else{
			MotorDefault[7] = 255;
			MotorDefault[8] = (manual_steps - 255);
			MotorDefault[12]= (gearStep+CONVERT_FRQ_SS_STEP[dir]);
		}

		MotorDefault[9] = 0;
		MotorDefault[10]= 0;
		MotorDefault[11]= 0;
		pr_err("[MCU] PR2 manual control (dir:%d, angle:%d, speed:%d, manual_steps:%d, gear_step:%d).", dir, angle, speed, manual_steps, gearStep);
	}else{
		pr_err("[MCU] PR2 manual control (dir:%d, angle:%d, speed:%d).", dir, angle, speed);
	}
	old_dir=dir;	

 
	if(speed <= 10){//micro-step process
		int i=0;

		for (i=0;i<6;i++){
			MotorDefault[i+1] = defaultManualSpeed[2*speed];
			MotorDefault[i+13]= defaultManualSpeed[(2*speed) + 1];
		}		 

	}else{
		pr_err("[MCU] %s don't support this(%d) manual speed.", __func__, speed);
	}

	if(ZEN7)
		return Zen7_MSP430FR2311_Set_ParamMode(MotorDefault);
	else
		return MSP430FR2311_Set_ParamMode(MotorDefault);
}

int ManualMode_AfterAndPR2_NOSS(int dir, int angle, int speed) {
	uint16_t MotorDefault[]={0, 120, 120, 120, 120, 120, 120, 255, 85, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint16_t manual_steps = 0;
	int gearStep = 0;
	
	memcpy(MotorDefault, ConstSpeedMode, sizeof(MotorDefault));
	if (MCUState!=MCU_READY) {
		pr_err("[MCU] %s Not ready!, state=%d", __func__, MCUState);
		return -MCUState;
	}
	
	MotorDefault[0]=dir;

	if(old_dir!=dir){
		gearStep+=LEAD_DELTA;
	} 	
	
	//Not use 0 instead of formula. I wish that: every step can be set different speed in manual mode, but it seems IMS only set one speed.
	manual_steps = (CONVERT_FRQ_FS_STEP[dir])*angle/180;	//Maybe need modify ???
	if(manual_steps <= 255){
		MotorDefault[7]  = manual_steps;
		MotorDefault[8]  = 0;
		MotorDefault[12] = (gearStep);
	}else{
		MotorDefault[7] = 255;
		MotorDefault[8] = (manual_steps - 255);
		MotorDefault[12]= (gearStep);
	}

	MotorDefault[9] = 0;
	MotorDefault[10]= 0;
	MotorDefault[11]= 0;
	pr_err("[MCU] NOSS PR2 manual control (dir:%d, angle:%d, speed:%d, manual_steps:%d, gear_step:%d).", dir, angle, speed, manual_steps, gearStep);

	old_dir=dir;	

 
	if(speed <= 10){//micro-step process
		int i=0;

		for (i=0;i<6;i++){
			MotorDefault[i+1] = defaultManualSpeed[2*speed];
			MotorDefault[i+13]= defaultManualSpeed[(2*speed) + 1];
		}		 

	}else{
		pr_err("[MCU] %s don't support this(%d) manual speed.", __func__, speed);
	}

	return Zen7_MSP430FR2311_Set_ParamMode(MotorDefault);
}


int MSP430FR2311_Stop(void) {
	char MSP430Stop[]={0xAA, 0x55, 0x08, 00, 00};
	
	D("[MCU] %s +\n", __func__);
	if (MCUState<MCU_CHECKING_READY) {
		pr_err("[MCU] %s Not ready!, state=%d", __func__, MCUState);
		return -MCUState;
	}
	
	mutex_lock(&MSP430FR2311_control_mutex);
	MSP430FR2311_wakeup(1);
	powerDownDuration=DEFAULT_POWERDOWNDURATION;
	if (!MSP430_I2CWriteA(MSP430_READY_I2C, MSP430Stop, sizeof(MSP430Stop))) {
		MSP430FR2311_wakeup(0);
		mutex_unlock(&MSP430FR2311_control_mutex);
		pr_err("[MCU] %s I2C error!", __func__);
		return -1;
	}
	if (bShowStopInfoOnce) {
		bShowStopInfoOnce=0;
		MSP430FR2311_Get_Steps();
	}
	MSP430FR2311_wakeup(0);
	mutex_unlock(&MSP430FR2311_control_mutex);
	
	pr_err("[MCU] %s.\n", __func__);
	return 0;	
}

static int MSP430FR2311_power_init(void)
{
        int ret = 0;
		pr_err("[MCU] power init, set requlator.");

		
        mcu_info->vcc_l10a_3p3 = regulator_get(&mcu_info->i2c_client->dev, "vcc_l10a_3p3");
        if (IS_ERR(mcu_info->vcc_l10a_3p3))
        {
                ret = PTR_ERR(mcu_info->vcc_l10a_3p3);
                pr_err("[MCU] Regulator get failed vdd ret=%d", ret);
                return ret;
        }

        if (regulator_count_voltages(mcu_info->vcc_l10a_3p3) > 0)
        {
                ret = regulator_set_voltage(mcu_info->vcc_l10a_3p3, 3300000, 3300000);
                if (ret)
                {
                        pr_err("[MCU] Regulator set_vtg failed vcc_l10a_3p3 ret=%d", ret);
                        goto reg_vdd_put;
                }
        }
		
	/*
	mcu_info->vcc_s4a_1p8 = regulator_get(&mcu_info->i2c_client->dev, "vcc_s4a_1p8");
	if (IS_ERR(mcu_info->vcc_s4a_1p8))
	{
					ret = PTR_ERR(mcu_info->vcc_s4a_1p8);
					pr_err("[MCU] Regulator get failed vdd ret=%d", ret);
					return ret;
	}
	
	if (regulator_count_voltages(mcu_info->vcc_s4a_1p8) > 0)
	{
					ret = regulator_set_voltage(mcu_info->vcc_s4a_1p8, 1800000, 1800000);
					if (ret)
					{
									pr_err("[MCU] Regulator set_vtg failed vcc_s4a_1p8 ret=%d", ret);
									goto reg_vdd_put;
					}
	}
	*/
	/*
	ret = gpio_request(mcu_info->mcu_5v_boost_enable, "gpio_msp430fr423111_5v_boost_enable");
	if (ret < 0) {
		pr_err("[MCU] %s: gpio %d request failed (%d)\n",
			__func__, mcu_info->mcu_5v_boost_enable, ret);
		return ret;
	}

	ret = gpio_direction_output(mcu_info->mcu_5v_boost_enable, 1);
	if (ret < 0) {
		pr_err(
			"[MCU] %s: fail to set gpio %d as input (%d)\n",
			__func__, mcu_info->mcu_5v_boost_enable, ret);
	gpio_free(mcu_info->mcu_5v_boost_enable);
	return ret;
	}*/

	return ret;

reg_vdd_put:
	regulator_put(mcu_info->vcc_l10a_3p3);
	return ret;
}


#ifdef CONFIG_OF
static int MSP430FR2311_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int rc;
	
	D("[MCU] %s\n", __func__);
	/*
	rc = of_get_named_gpio_flags(np, "MCU,mcu5V_boost_enable-gpios",
			0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read mcureset pin number\n");
		return rc;
	} 
	else
	{
		mcu_info->mcu_5v_boost_enable= rc;
 	  D("[MCU] %s GET mcu 5v enable PIN =%d\n", __func__, rc);   
	}*/

	rc = of_get_named_gpio_flags(np, "MCU,mcureset-gpios",
			0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read mcureset pin number\n");
		return rc;
	} 
	else
	{
		mcu_info->mcu_reset= rc;
 	  D("[MCU] %s GET mcu reset PIN =%d\n", __func__, rc);   
	}
	rc = of_get_named_gpio_flags(np, "MCU,mcutest-gpios",
			0, NULL);
	if (rc < 0)	{
		dev_err(dev, "Unable to read mcutest pin number\n");
		return rc;
	} 	else	{
		mcu_info->mcu_test= rc;
 	  D("[MCU] %s GET mcu test PIN=%d \n", __func__, rc);   
	}

	rc = of_get_named_gpio_flags(np, "MCU,mcuwakeup-gpios",
			0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read mcu wakeup pin number\n");
		return rc;
	} 
	else
	{
		mcu_info->mcu_wakeup= rc;
 	  D("[MCU] %s GET mcu wakeup PIN =%d\n", __func__, rc);   
	}

	rc = of_get_named_gpio_flags(np, "MCU,mcuint-gpios",
			0, NULL);
	if (rc < 0) {
		dev_err(dev, "Unable to read mcu int pin number\n");
		return rc;
	} 
	else
	{
		mcu_info->mcu_int= rc;
 	  D("[MCU] %s GET mcu int PIN =%d\n", __func__, rc);   
	}
	
	rc = of_property_read_u32(np, "MCU,slave_address", &temp_val);
	if (rc)	{
		dev_err(dev, "Unable to read slave_address\n");
		return rc;
	} 	else	{
		mcu_info->slave_addr = (uint8_t)temp_val;
	}
  
	D("[MCU] %s PARSE OK \n", __func__);

	return 0;
}
#endif

extern bool g_Charger_mode;

static int MSP430FR2311_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	pr_err("[MCU] driver probe.\n");

	if (g_Charger_mode == 1) {
		pr_err("[MCU] in charging mode, skip MCU probe\n");
		
			return -EBUSY;
		}


	mcu_info = kzalloc(sizeof(struct MSP430FR2311_info), GFP_KERNEL);
	if (!mcu_info)
		return -ENOMEM;

	/*D("[MSP430FR2311] %s: client->irq = %d\n", __func__, client->irq);*/

	mcu_info->i2c_client = client;

	mcu_info->mcu_reset=-1;
	mcu_info->mcu_test=-1;
	i2c_set_clientdata(client, mcu_info);
	mcu_info->mcu_polling_delay = msecs_to_jiffies(MCU_POLLING_DELAY);

	asus_motor_init(mcu_info);



	if( MSP430FR2311_parse_dt(&client->dev) < 0 )
	{
		ret = -EBUSY;
		goto err_platform_data_null;  
	}
	
		
        ret = MSP430FR2311_power_init();
        if (ret < 0) {
                pr_err("[MCU] %s: set regulator fail\n", __func__);
        	}

				ret= initial_MSP430FR2311_gpio();
				if (ret < 0) {
					pr_err(
						"[MCU] fail to initial MSP430FR2155 (%d)\n", ret);
					goto err_platform_data_null;	
					
				}


        ret = MSP430FR2311_power_control(1);
        if (ret < 0) {
                pr_err("[MCU] %s: enable regulator fail\n", __func__);
								goto err_platform_data_null;	
        	}

//	if (MSP430FR2311_Check_Version() && MSP43FR2311_Go_BSL_Mode()) {
		MCUState=MCU_EMPTY;
//		pr_err("[MCU][MSP430FR2311 ]%s: Fail\n", __func__);		
//		goto err_initial_MSP430FR2311_gpio;
//	}

	mutex_init(&MSP430FR2311_control_mutex);

	ret = mcu_setup();
	if (ret < 0) {
		pr_err("[MCU] %s: mcu_setup error!!\n",
			__func__);
		goto err_mcu_setup;
	}

	init_irq();  

//	if (MCUState!=MCU_READY) {
		mcu_info->mcu_wq = create_singlethread_workqueue("MSP430FR2311_wq");
		if (!mcu_info->mcu_wq) {
			pr_err("[MCU] %s: can't create workqueue\n", __func__);
			ret = -ENOMEM;
			goto err_create_singlethread_workqueue;
		}

		mcu_info->cal_wq = create_singlethread_workqueue("mcu_cal_wq");
		if (!mcu_info->cal_wq) {
			pr_err("[MCU] %s: can't create cal_wq workqueue\n", __func__);
			ret = -ENOMEM;
			goto err_create_singlethread_workqueue;
		}
		
		//Delay more time to wait vendor partion ready.
		queue_delayed_work(mcu_info->mcu_wq, &report_work, msecs_to_jiffies(20000));	//mcu_info->mcu_polling_delay
//	}

//	ret = MSP430FR2311_setup();
//	if (ret < 0) {
//		pr_err("[MCU][MSP430FR2311 error]%s: MSP430FR2311_setup error!\n", __func__);
//		goto err_MSP430FR2311_setup;
//	}

	mcu_info->MSP430FR2311_class = class_create(THIS_MODULE, "TI_mcu");
	if (IS_ERR(mcu_info->MSP430FR2311_class)) {
		ret = PTR_ERR(mcu_info->MSP430FR2311_class);
		mcu_info->MSP430FR2311_class = NULL;
		goto err_create_class;
	}

	mcu_info->mcu_dev = device_create(mcu_info->MSP430FR2311_class,
				NULL, 0, "%s", "mcu");
	if (unlikely(IS_ERR(mcu_info->mcu_dev))) {
		ret = PTR_ERR(mcu_info->mcu_dev);
		mcu_info->mcu_dev = NULL;
		goto err_create_mcu_device;
	}	

	fManualMode=ManualMode_AfterAndPR2;
	pr_err("[MCU] %s: Probe success, manual mode=PR2!\n", __func__);

	return ret;

err_create_mcu_device:
  device_destroy(mcu_info->MSP430FR2311_class, mcu_info->mcu_dev->devt);
	class_destroy(mcu_info->MSP430FR2311_class);
err_create_class:
	if (mcu_info->mcu_wq) destroy_workqueue(mcu_info->mcu_wq);
	if (mcu_info->cal_wq) destroy_workqueue(mcu_info->cal_wq);	
err_create_singlethread_workqueue:
err_mcu_setup:
	mutex_destroy(&MSP430FR2311_control_mutex);
	misc_deregister(&mcu_misc); //lightsensor_setup
//err_initial_MSP430FR2311_gpio:
		gpio_free(mcu_info->mcu_reset); 
		gpio_free(mcu_info->mcu_test); 
err_platform_data_null:
	kfree(mcu_info);
	g_motor_status = 0; //probe fail
	return ret;
}
   

static const struct i2c_device_id MSP430FR2311_i2c_id[] = {
	{MSP430FR2311_I2C_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id MSP430FR2311_match_table[] = {
	{ .compatible = "MCU,MSP430FR2311",},
	{ },
};
#else
#define MSP430FR2311_match_table NULL
#endif

#ifdef CONFIG_PM_SLEEP
static int mcu_suspend(struct device *dev)
{
	struct mcu_info *mpi;
	mpi = dev_get_drvdata(dev);
	pr_err("[MCU] go to power off");
	//MSP430FR2311_power_control(0);

	return 0;
}

static int mcu_resume(struct device *dev)
{
	struct mcu_info *mpi;
	mpi = dev_get_drvdata(dev);
	pr_err("[MCU] go to power on");
	/*
#ifdef MCU_5V_ALWAYS_ON
	if (g_ASUS_hwID == HW_REV_ER	|| g_ASUS_hwID == HW_REV_SR ) 
#endif
	{
		if (MCUState>=MCU_CHECKING_READY) {
			MCUState=MCU_WAIT_POWER_READY;
			queue_delayed_work(mcu_info->mcu_wq, &report_work, 20);  //20 for 200ms		 
		}
	}*/
	//MSP430FR2311_power_control(1);


	return 0;
}
#endif


static UNIVERSAL_DEV_PM_OPS(mcu_pm, mcu_suspend, mcu_resume, NULL);


static struct i2c_driver MSP430FR2311_driver = {
	.id_table = MSP430FR2311_i2c_id,
	.probe = MSP430FR2311_probe,
	.driver = {
		.name = MSP430FR2311_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
			.pm = &mcu_pm,	
#endif
		.of_match_table = of_match_ptr(MSP430FR2311_match_table),     
	},
};

static int __init MSP430FR2311_init(void)
{
	return i2c_add_driver(&MSP430FR2311_driver);
}

static void __exit MSP430FR2311_exit(void)
{
	i2c_del_driver(&MSP430FR2311_driver);
}

module_init(MSP430FR2311_init);
module_exit(MSP430FR2311_exit);

MODULE_AUTHOR("Randy Change <randy_change@asus.com>");
MODULE_DESCRIPTION("MCU MSP430FR2311 micro processor Driver");
MODULE_LICENSE("GPL v2");


//==========================================Zen7========================================
//=====
static int file_op(const char *filename, loff_t offset, char *buf, int length, int operation)
{
	int filep;
	mm_segment_t old_fs;
	int ret = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (FILE_OP_READ == operation)
		filep= ksys_open(filename, O_RDONLY | O_CREAT | O_SYNC, 0777);
	else if (FILE_OP_WRITE == operation)
		filep= ksys_open(filename, O_RDWR | O_CREAT | O_SYNC, 0777);
	else {
		ksys_close(filep);
		set_fs(old_fs);
		pr_err("[MCU] Unknown partition op err!\n");
		return -1;
	}
	if (filep < 0) {
		ksys_close(filep);
		set_fs(old_fs);
		pr_err("[MCU] open %s err! error code:%d\n", filename, filep);
		return -1;
	}
	else {
		pr_err("[MCU] open %s success!\n", filename);
	}
	
	//ksys_chown(filename, 1015, 1015);	//#define AID_SDCARD_RW 1015
	ksys_lseek(filep, offset, SEEK_SET);
	if (FILE_OP_READ == operation)
		ret = ksys_read(filep, buf, length);
	else if (FILE_OP_WRITE == operation) {
		ret = ksys_write(filep, buf, length);
		ksys_sync();
	}
	ksys_close(filep);
	set_fs(old_fs);
	pr_err("[MCU] %s %s length:%d ret:%d.\n", __func__, operation?"w":"r", length, ret);
	return length;
}

//Backup 110 bytes to mcu_bak.
static int backup_mcu_cal_thed(void)
{
	char buf[400]={0};
	char MCUBuf[10];
	unsigned char i;
	int rc;

	pr_err("[MCU] %s.\n", __func__);

	for(i=0; i<CalLength; i++){
		sprintf(MCUBuf, " %X", fcal_val.Cal_val[i]);
		strcat(buf, MCUBuf);		
	}
	rc = file_op(MCU_BAKEUP_FILE_NAME, CAL_DATA_OFFSET,
		(char *)&buf, 3*CalLength*sizeof(char), FILE_OP_WRITE);
	if (rc<0)
		pr_err("[MCU] %s:Write file:%s err!\n", __FUNCTION__, MCU_BAKEUP_FILE_NAME);
	
	return rc;
}

//Read 110 bytes to mcu.
static int read_mcu_cal_thed(void)
{
	char buf[400]={0};
	int rc;
	unsigned char i;
	uint8_t wBuf[CalLength+3] = {0xAA, 0x55, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	pr_err("[MCU] %s.\n", __func__);
	
	for(i=0; i<CalLength; i++)
		fcal_val.Cal_val[i] = 0;
	
	rc = file_op(MCU_BAKEUP_FILE_NAME, CAL_DATA_OFFSET,
		(char *)&buf, 3*CalLength*sizeof(char), FILE_OP_READ);
	if (rc<0){
		pr_err("%s:read file:%s err!\n", __FUNCTION__, MCU_BAKEUP_FILE_NAME);
		return rc;
	}

	//pr_err("[MCU]:%s\n", buf);

	sscanf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n", \
		&(fcal_val.Cal_val[0]), &(fcal_val.Cal_val[1]), &(fcal_val.Cal_val[2]), &(fcal_val.Cal_val[3]), &(fcal_val.Cal_val[4]),	\
		&(fcal_val.Cal_val[5]), &(fcal_val.Cal_val[6]), &(fcal_val.Cal_val[7]), &(fcal_val.Cal_val[8]), &(fcal_val.Cal_val[9]),	\
		&(fcal_val.Cal_val[10]), &(fcal_val.Cal_val[11]), &(fcal_val.Cal_val[12]), &(fcal_val.Cal_val[13]), &(fcal_val.Cal_val[14]),	\
		&(fcal_val.Cal_val[15]), &(fcal_val.Cal_val[16]), &(fcal_val.Cal_val[17]), &(fcal_val.Cal_val[18]), &(fcal_val.Cal_val[19]),	\
		&(fcal_val.Cal_val[20]), &(fcal_val.Cal_val[21]), &(fcal_val.Cal_val[22]), &(fcal_val.Cal_val[23]),	\
		&(fcal_val.Cal_val[24]), &(fcal_val.Cal_val[25]), &(fcal_val.Cal_val[26]), &(fcal_val.Cal_val[27]), &(fcal_val.Cal_val[28]), &(fcal_val.Cal_val[29]), &(fcal_val.Cal_val[30]), &(fcal_val.Cal_val[31]), &(fcal_val.Cal_val[32]), &(fcal_val.Cal_val[33]),	\
		&(fcal_val.Cal_val[34]), &(fcal_val.Cal_val[35]), &(fcal_val.Cal_val[36]), &(fcal_val.Cal_val[37]), &(fcal_val.Cal_val[38]), &(fcal_val.Cal_val[39]), &(fcal_val.Cal_val[40]), &(fcal_val.Cal_val[41]), &(fcal_val.Cal_val[42]), &(fcal_val.Cal_val[43]),	\
		&(fcal_val.Cal_val[44]), &(fcal_val.Cal_val[45]), &(fcal_val.Cal_val[46]), &(fcal_val.Cal_val[47]), &(fcal_val.Cal_val[48]), &(fcal_val.Cal_val[49]), &(fcal_val.Cal_val[50]), &(fcal_val.Cal_val[51]), &(fcal_val.Cal_val[52]), &(fcal_val.Cal_val[53]),	\
		&(fcal_val.Cal_val[54]), &(fcal_val.Cal_val[55]), &(fcal_val.Cal_val[56]), &(fcal_val.Cal_val[57]), &(fcal_val.Cal_val[58]), &(fcal_val.Cal_val[59]), &(fcal_val.Cal_val[60]), &(fcal_val.Cal_val[61]), &(fcal_val.Cal_val[62]), &(fcal_val.Cal_val[63]),	\
		&(fcal_val.Cal_val[64]), &(fcal_val.Cal_val[65]), &(fcal_val.Cal_val[66]), &(fcal_val.Cal_val[67]), &(fcal_val.Cal_val[68]), &(fcal_val.Cal_val[69]), &(fcal_val.Cal_val[70]), &(fcal_val.Cal_val[71]), &(fcal_val.Cal_val[72]), &(fcal_val.Cal_val[73]),	\
		&(fcal_val.Cal_val[74]), &(fcal_val.Cal_val[75]), &(fcal_val.Cal_val[76]), &(fcal_val.Cal_val[77]), &(fcal_val.Cal_val[78]), &(fcal_val.Cal_val[79]), &(fcal_val.Cal_val[80]), &(fcal_val.Cal_val[81]), &(fcal_val.Cal_val[82]), &(fcal_val.Cal_val[83]),	\
		&(fcal_val.Cal_val[84]), &(fcal_val.Cal_val[85]), &(fcal_val.Cal_val[86]), &(fcal_val.Cal_val[87]), &(fcal_val.Cal_val[88]), &(fcal_val.Cal_val[89]), &(fcal_val.Cal_val[90]), &(fcal_val.Cal_val[91]), &(fcal_val.Cal_val[92]), &(fcal_val.Cal_val[93]),	\
		&(fcal_val.Cal_val[94]), &(fcal_val.Cal_val[95]), &(fcal_val.Cal_val[96]), &(fcal_val.Cal_val[97]), &(fcal_val.Cal_val[98]), &(fcal_val.Cal_val[99]), &(fcal_val.Cal_val[100]), &(fcal_val.Cal_val[101]), &(fcal_val.Cal_val[102]), &(fcal_val.Cal_val[103]),	\
		&(fcal_val.Cal_val[104]), &(fcal_val.Cal_val[105]), &(fcal_val.Cal_val[106]), &(fcal_val.Cal_val[107]), &(fcal_val.Cal_val[108]), &(fcal_val.Cal_val[109]));	

	pr_err("[MCU] %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x...%x %x %x %x, %x %x %x %x %x %x.\n", \
		fcal_val.Cal_val[0], fcal_val.Cal_val[1], fcal_val.Cal_val[2], fcal_val.Cal_val[3], fcal_val.Cal_val[4],	\
		fcal_val.Cal_val[5], fcal_val.Cal_val[6], fcal_val.Cal_val[7], fcal_val.Cal_val[8], fcal_val.Cal_val[9],	\
		fcal_val.Cal_val[10], fcal_val.Cal_val[11], fcal_val.Cal_val[12], fcal_val.Cal_val[13], fcal_val.Cal_val[14],	\
		fcal_val.Cal_val[15], fcal_val.Cal_val[16], fcal_val.Cal_val[17], fcal_val.Cal_val[18], fcal_val.Cal_val[19],	\
		fcal_val.Cal_val[20], fcal_val.Cal_val[21], fcal_val.Cal_val[22], fcal_val.Cal_val[23],	\
		fcal_val.Cal_val[100], fcal_val.Cal_val[101], fcal_val.Cal_val[102], fcal_val.Cal_val[103],	\
		fcal_val.Cal_val[104], fcal_val.Cal_val[105], fcal_val.Cal_val[106], fcal_val.Cal_val[107], fcal_val.Cal_val[108], fcal_val.Cal_val[109]);

	for(i=0; i<CalLength; i++)
		wBuf[3+i] = fcal_val.Cal_val[i];
	Zen7_MSP430FR2311_wI2CtoMCU(wBuf, sizeof(wBuf));
	return rc;
}

static int backup_angle_raw(unsigned char index, unsigned char *wRawBuf)
{
	char buf[200]={0};
	int rc;
	char MCUBuf[10];
	unsigned char i;

	pr_err("[MCU] %s.\n", __func__);
	/*	
	sprintf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n", \
		wRawBuf[0], wRawBuf[1], wRawBuf[2], wRawBuf[3], wRawBuf[4],	\
		wRawBuf[5], wRawBuf[6], wRawBuf[7], wRawBuf[8], wRawBuf[9],	\
		wRawBuf[10], wRawBuf[11], wRawBuf[12], wRawBuf[13], wRawBuf[14],	\
		wRawBuf[15], wRawBuf[16], wRawBuf[17], wRawBuf[18], wRawBuf[19]);	
	*/
	for(i=0; i<RawLength; i++){
		sprintf(MCUBuf, " %X", wRawBuf[i]);
		strcat(buf, MCUBuf);		
	}
		
	rc = file_op(MCU_BAKEUP_RAW_FILE_NAME, index*3*RawLength*sizeof(char),
		(char *)&buf, 3*RawLength*sizeof(char), FILE_OP_WRITE);
	if (rc<0)
		pr_err("%s:Write file:%s err!\n", __FUNCTION__, MCU_BAKEUP_RAW_FILE_NAME);

	return rc;
}

//Backup 800+120 bytes to mcu_raw.
static void bak_raw(void){
	unsigned char i = 0;
	unsigned char rRawBuf[RawLength];
	uint8_t wBuf[4] = {0xAA, 0x55, 0x80, 0x00};
	
	for(i=0; i<23; i++){	//20 bufferMag + 3 rawAK9970Corr.
		//0xAA 0x55 0x80 index, let mcu prepare 20 bytes raw data.
		wBuf[3] = i;
		Zen7_MSP430FR2311_wI2CtoMCU(wBuf, sizeof(wBuf));

		//read and write file.
		if(MSP430_I2CRead(MSP430_READY_I2C, rRawBuf, RawLength)){
			backup_angle_raw(i, rRawBuf);
		}else
			pr_err("[MCU] %s i2c raw backup (%d) read error!\n", __func__, i);
	}

	//copy file to factory folder.
	//report_motor_event(MOTOR_COPY_MCU_FILE, 3);	// /asdf/mcu_raw --> factory.
	pr_err("[MCU] %s done.\n", __func__);
}

//==========================================================================for User K
//Check file is null. 0:null, 1:not null.
static char IsFileNull(const char *filename){
	int Isfilep;
	mm_segment_t old_fs;
	long size;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	
	Isfilep = ksys_open(filename, O_RDONLY | O_CREAT | O_SYNC, 0777);

	if (Isfilep < 0) {
		ksys_close(Isfilep);
		set_fs(old_fs);
		pr_err("[MCU] %s open %s err! error code:%d\n", __func__, filename, Isfilep);
		return 0;
	}
	else {
		pr_err("[MCU] %s open %s success!\n", __func__, filename);
	}
	
	//ksys_chown(filename, 1015, 1015);	//#define AID_SDCARD_RW 1015
	size = ksys_lseek(Isfilep, 0, SEEK_END);
	pr_err("[MCU] %s size:%d.\n", __func__,size);
	ksys_close(Isfilep);
	set_fs(old_fs);	
	
	if(size <= 0)		
		return 0;
	else
		return 1;
}

//Read motor_fw2 110 bytes to mcu.
static int read_usek_cal3threshlod(void)
{
	char buf[400]={0};
	int rc;
	unsigned char i;
	uint8_t wBuf[CalLength+3] = {0xAA, 0x55, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	
	pr_err("[MCU] %s.\n", __func__);
	
	for(i=0; i<CalLength; i++)
		fcal_val_UserK.Cal_val[i] = 0;
	
	rc = file_op(MCU_BAKEUP_USER_FILE_NAME, CAL_DATA_OFFSET,
		(char *)&buf, 3*CalLength*sizeof(char), FILE_OP_READ);
	if (rc<0){
		pr_err("%s:read file:%s err!\n", __func__, MCU_BAKEUP_USER_FILE_NAME);
		return rc;
	}

	sscanf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n", \
		&(fcal_val_UserK.Cal_val[0]), &(fcal_val_UserK.Cal_val[1]), &(fcal_val_UserK.Cal_val[2]), &(fcal_val_UserK.Cal_val[3]), &(fcal_val_UserK.Cal_val[4]),	\
		&(fcal_val_UserK.Cal_val[5]), &(fcal_val_UserK.Cal_val[6]), &(fcal_val_UserK.Cal_val[7]), &(fcal_val_UserK.Cal_val[8]), &(fcal_val_UserK.Cal_val[9]),	\
		&(fcal_val_UserK.Cal_val[10]), &(fcal_val_UserK.Cal_val[11]), &(fcal_val_UserK.Cal_val[12]), &(fcal_val_UserK.Cal_val[13]), &(fcal_val_UserK.Cal_val[14]),	\
		&(fcal_val_UserK.Cal_val[15]), &(fcal_val_UserK.Cal_val[16]), &(fcal_val_UserK.Cal_val[17]), &(fcal_val_UserK.Cal_val[18]), &(fcal_val_UserK.Cal_val[19]),	\
		&(fcal_val_UserK.Cal_val[20]), &(fcal_val_UserK.Cal_val[21]), &(fcal_val_UserK.Cal_val[22]), &(fcal_val_UserK.Cal_val[23]),	\
		&(fcal_val_UserK.Cal_val[24]), &(fcal_val_UserK.Cal_val[25]), &(fcal_val_UserK.Cal_val[26]), &(fcal_val_UserK.Cal_val[27]), &(fcal_val_UserK.Cal_val[28]), &(fcal_val_UserK.Cal_val[29]), &(fcal_val_UserK.Cal_val[30]), &(fcal_val_UserK.Cal_val[31]), &(fcal_val_UserK.Cal_val[32]), &(fcal_val_UserK.Cal_val[33]),	\
		&(fcal_val_UserK.Cal_val[34]), &(fcal_val_UserK.Cal_val[35]), &(fcal_val_UserK.Cal_val[36]), &(fcal_val_UserK.Cal_val[37]), &(fcal_val_UserK.Cal_val[38]), &(fcal_val_UserK.Cal_val[39]), &(fcal_val_UserK.Cal_val[40]), &(fcal_val_UserK.Cal_val[41]), &(fcal_val_UserK.Cal_val[42]), &(fcal_val_UserK.Cal_val[43]),	\
		&(fcal_val_UserK.Cal_val[44]), &(fcal_val_UserK.Cal_val[45]), &(fcal_val_UserK.Cal_val[46]), &(fcal_val_UserK.Cal_val[47]), &(fcal_val_UserK.Cal_val[48]), &(fcal_val_UserK.Cal_val[49]), &(fcal_val_UserK.Cal_val[50]), &(fcal_val_UserK.Cal_val[51]), &(fcal_val_UserK.Cal_val[52]), &(fcal_val_UserK.Cal_val[53]),	\
		&(fcal_val_UserK.Cal_val[54]), &(fcal_val_UserK.Cal_val[55]), &(fcal_val_UserK.Cal_val[56]), &(fcal_val_UserK.Cal_val[57]), &(fcal_val_UserK.Cal_val[58]), &(fcal_val_UserK.Cal_val[59]), &(fcal_val_UserK.Cal_val[60]), &(fcal_val_UserK.Cal_val[61]), &(fcal_val_UserK.Cal_val[62]), &(fcal_val_UserK.Cal_val[63]),	\
		&(fcal_val_UserK.Cal_val[64]), &(fcal_val_UserK.Cal_val[65]), &(fcal_val_UserK.Cal_val[66]), &(fcal_val_UserK.Cal_val[67]), &(fcal_val_UserK.Cal_val[68]), &(fcal_val_UserK.Cal_val[69]), &(fcal_val_UserK.Cal_val[70]), &(fcal_val_UserK.Cal_val[71]), &(fcal_val_UserK.Cal_val[72]), &(fcal_val_UserK.Cal_val[73]),	\
		&(fcal_val_UserK.Cal_val[74]), &(fcal_val_UserK.Cal_val[75]), &(fcal_val_UserK.Cal_val[76]), &(fcal_val_UserK.Cal_val[77]), &(fcal_val_UserK.Cal_val[78]), &(fcal_val_UserK.Cal_val[79]), &(fcal_val_UserK.Cal_val[80]), &(fcal_val_UserK.Cal_val[81]), &(fcal_val_UserK.Cal_val[82]), &(fcal_val_UserK.Cal_val[83]),	\
		&(fcal_val_UserK.Cal_val[84]), &(fcal_val_UserK.Cal_val[85]), &(fcal_val_UserK.Cal_val[86]), &(fcal_val_UserK.Cal_val[87]), &(fcal_val_UserK.Cal_val[88]), &(fcal_val_UserK.Cal_val[89]), &(fcal_val_UserK.Cal_val[90]), &(fcal_val_UserK.Cal_val[91]), &(fcal_val_UserK.Cal_val[92]), &(fcal_val_UserK.Cal_val[93]),	\
		&(fcal_val_UserK.Cal_val[94]), &(fcal_val_UserK.Cal_val[95]), &(fcal_val_UserK.Cal_val[96]), &(fcal_val_UserK.Cal_val[97]), &(fcal_val_UserK.Cal_val[98]), &(fcal_val_UserK.Cal_val[99]), &(fcal_val_UserK.Cal_val[100]), &(fcal_val_UserK.Cal_val[101]), &(fcal_val_UserK.Cal_val[102]), &(fcal_val_UserK.Cal_val[103]),	\
		&(fcal_val_UserK.Cal_val[104]), &(fcal_val_UserK.Cal_val[105]), &(fcal_val_UserK.Cal_val[106]), &(fcal_val_UserK.Cal_val[107]), &(fcal_val_UserK.Cal_val[108]), &(fcal_val_UserK.Cal_val[109]));	

	pr_err("[MCU] %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x...%x %x %x %x, %x %x %x %x %x %x.\n", \
		fcal_val_UserK.Cal_val[0], fcal_val_UserK.Cal_val[1], fcal_val_UserK.Cal_val[2], fcal_val_UserK.Cal_val[3], fcal_val_UserK.Cal_val[4],	\
		fcal_val_UserK.Cal_val[5], fcal_val_UserK.Cal_val[6], fcal_val_UserK.Cal_val[7], fcal_val_UserK.Cal_val[8], fcal_val_UserK.Cal_val[9],	\
		fcal_val_UserK.Cal_val[10], fcal_val_UserK.Cal_val[11], fcal_val_UserK.Cal_val[12], fcal_val_UserK.Cal_val[13], fcal_val_UserK.Cal_val[14],	\
		fcal_val_UserK.Cal_val[15], fcal_val_UserK.Cal_val[16], fcal_val_UserK.Cal_val[17], fcal_val_UserK.Cal_val[18], fcal_val_UserK.Cal_val[19],	\
		fcal_val_UserK.Cal_val[20], fcal_val_UserK.Cal_val[21], fcal_val_UserK.Cal_val[22], fcal_val_UserK.Cal_val[23],	\
		fcal_val_UserK.Cal_val[100], fcal_val_UserK.Cal_val[101], fcal_val_UserK.Cal_val[102], fcal_val_UserK.Cal_val[103],	\
		fcal_val_UserK.Cal_val[104], fcal_val_UserK.Cal_val[105], fcal_val_UserK.Cal_val[106], fcal_val_UserK.Cal_val[107], fcal_val_UserK.Cal_val[108], fcal_val_UserK.Cal_val[109]);

	for(i=0; i<CalLength; i++)
		wBuf[3+i] = fcal_val_UserK.Cal_val[i];
	Zen7_MSP430FR2311_wI2CtoMCU(wBuf, sizeof(wBuf));
	return rc;
}

//Read MCU calibration data to compare with phone stored.
static int CompareCaliData(char *filename)
{
	char buf[400]={0};
	int rc;
	unsigned char i;
	uint8_t wBuf[CalLength+3] = {0xAA, 0x55, 0x71, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t wFlg = 0;
		
	for(i=0; i<CalLength; i++)
		fcal_val_rPhone.Cal_val[i] = 0;
	
	rc = file_op(filename, CAL_DATA_OFFSET,
		(char *)&buf, 3*CalLength*sizeof(char), FILE_OP_READ);
	if (rc<0){
		pr_err("%s:read file:%s err!\n", __func__, filename);
		return rc;
	}

	sscanf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x\n", \
		&(fcal_val_rPhone.Cal_val[0]), &(fcal_val_rPhone.Cal_val[1]), &(fcal_val_rPhone.Cal_val[2]), &(fcal_val_rPhone.Cal_val[3]), &(fcal_val_rPhone.Cal_val[4]),	\
		&(fcal_val_rPhone.Cal_val[5]), &(fcal_val_rPhone.Cal_val[6]), &(fcal_val_rPhone.Cal_val[7]), &(fcal_val_rPhone.Cal_val[8]), &(fcal_val_rPhone.Cal_val[9]),	\
		&(fcal_val_rPhone.Cal_val[10]), &(fcal_val_rPhone.Cal_val[11]), &(fcal_val_rPhone.Cal_val[12]), &(fcal_val_rPhone.Cal_val[13]), &(fcal_val_rPhone.Cal_val[14]),	\
		&(fcal_val_rPhone.Cal_val[15]), &(fcal_val_rPhone.Cal_val[16]), &(fcal_val_rPhone.Cal_val[17]), &(fcal_val_rPhone.Cal_val[18]), &(fcal_val_rPhone.Cal_val[19]),	\
		&(fcal_val_rPhone.Cal_val[20]), &(fcal_val_rPhone.Cal_val[21]), &(fcal_val_rPhone.Cal_val[22]), &(fcal_val_rPhone.Cal_val[23]),	\
		&(fcal_val_rPhone.Cal_val[24]), &(fcal_val_rPhone.Cal_val[25]), &(fcal_val_rPhone.Cal_val[26]), &(fcal_val_rPhone.Cal_val[27]), &(fcal_val_rPhone.Cal_val[28]), &(fcal_val_rPhone.Cal_val[29]), &(fcal_val_rPhone.Cal_val[30]), &(fcal_val_rPhone.Cal_val[31]), &(fcal_val_rPhone.Cal_val[32]), &(fcal_val_rPhone.Cal_val[33]),	\
		&(fcal_val_rPhone.Cal_val[34]), &(fcal_val_rPhone.Cal_val[35]), &(fcal_val_rPhone.Cal_val[36]), &(fcal_val_rPhone.Cal_val[37]), &(fcal_val_rPhone.Cal_val[38]), &(fcal_val_rPhone.Cal_val[39]), &(fcal_val_rPhone.Cal_val[40]), &(fcal_val_rPhone.Cal_val[41]), &(fcal_val_rPhone.Cal_val[42]), &(fcal_val_rPhone.Cal_val[43]),	\
		&(fcal_val_rPhone.Cal_val[44]), &(fcal_val_rPhone.Cal_val[45]), &(fcal_val_rPhone.Cal_val[46]), &(fcal_val_rPhone.Cal_val[47]), &(fcal_val_rPhone.Cal_val[48]), &(fcal_val_rPhone.Cal_val[49]), &(fcal_val_rPhone.Cal_val[50]), &(fcal_val_rPhone.Cal_val[51]), &(fcal_val_rPhone.Cal_val[52]), &(fcal_val_rPhone.Cal_val[53]),	\
		&(fcal_val_rPhone.Cal_val[54]), &(fcal_val_rPhone.Cal_val[55]), &(fcal_val_rPhone.Cal_val[56]), &(fcal_val_rPhone.Cal_val[57]), &(fcal_val_rPhone.Cal_val[58]), &(fcal_val_rPhone.Cal_val[59]), &(fcal_val_rPhone.Cal_val[60]), &(fcal_val_rPhone.Cal_val[61]), &(fcal_val_rPhone.Cal_val[62]), &(fcal_val_rPhone.Cal_val[63]),	\
		&(fcal_val_rPhone.Cal_val[64]), &(fcal_val_rPhone.Cal_val[65]), &(fcal_val_rPhone.Cal_val[66]), &(fcal_val_rPhone.Cal_val[67]), &(fcal_val_rPhone.Cal_val[68]), &(fcal_val_rPhone.Cal_val[69]), &(fcal_val_rPhone.Cal_val[70]), &(fcal_val_rPhone.Cal_val[71]), &(fcal_val_rPhone.Cal_val[72]), &(fcal_val_rPhone.Cal_val[73]),	\
		&(fcal_val_rPhone.Cal_val[74]), &(fcal_val_rPhone.Cal_val[75]), &(fcal_val_rPhone.Cal_val[76]), &(fcal_val_rPhone.Cal_val[77]), &(fcal_val_rPhone.Cal_val[78]), &(fcal_val_rPhone.Cal_val[79]), &(fcal_val_rPhone.Cal_val[80]), &(fcal_val_rPhone.Cal_val[81]), &(fcal_val_rPhone.Cal_val[82]), &(fcal_val_rPhone.Cal_val[83]),	\
		&(fcal_val_rPhone.Cal_val[84]), &(fcal_val_rPhone.Cal_val[85]), &(fcal_val_rPhone.Cal_val[86]), &(fcal_val_rPhone.Cal_val[87]), &(fcal_val_rPhone.Cal_val[88]), &(fcal_val_rPhone.Cal_val[89]), &(fcal_val_rPhone.Cal_val[90]), &(fcal_val_rPhone.Cal_val[91]), &(fcal_val_rPhone.Cal_val[92]), &(fcal_val_rPhone.Cal_val[93]),	\
		&(fcal_val_rPhone.Cal_val[94]), &(fcal_val_rPhone.Cal_val[95]), &(fcal_val_rPhone.Cal_val[96]), &(fcal_val_rPhone.Cal_val[97]), &(fcal_val_rPhone.Cal_val[98]), &(fcal_val_rPhone.Cal_val[99]), &(fcal_val_rPhone.Cal_val[100]), &(fcal_val_rPhone.Cal_val[101]), &(fcal_val_rPhone.Cal_val[102]), &(fcal_val_rPhone.Cal_val[103]),	\
		&(fcal_val_rPhone.Cal_val[104]), &(fcal_val_rPhone.Cal_val[105]), &(fcal_val_rPhone.Cal_val[106]), &(fcal_val_rPhone.Cal_val[107]), &(fcal_val_rPhone.Cal_val[108]), &(fcal_val_rPhone.Cal_val[109]));	

	pr_err("[MCU] %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x...%x %x %x %x, %x %x %x %x %x %x.\n", \
		fcal_val_rPhone.Cal_val[0], fcal_val_rPhone.Cal_val[1], fcal_val_rPhone.Cal_val[2], fcal_val_rPhone.Cal_val[3], fcal_val_rPhone.Cal_val[4],	\
		fcal_val_rPhone.Cal_val[5], fcal_val_rPhone.Cal_val[6], fcal_val_rPhone.Cal_val[7], fcal_val_rPhone.Cal_val[8], fcal_val_rPhone.Cal_val[9],	\
		fcal_val_rPhone.Cal_val[10], fcal_val_rPhone.Cal_val[11], fcal_val_rPhone.Cal_val[12], fcal_val_rPhone.Cal_val[13], fcal_val_rPhone.Cal_val[14],	\
		fcal_val_rPhone.Cal_val[15], fcal_val_rPhone.Cal_val[16], fcal_val_rPhone.Cal_val[17], fcal_val_rPhone.Cal_val[18], fcal_val_rPhone.Cal_val[19],	\
		fcal_val_rPhone.Cal_val[20], fcal_val_rPhone.Cal_val[21], fcal_val_rPhone.Cal_val[22], fcal_val_rPhone.Cal_val[23],	\
		fcal_val_rPhone.Cal_val[100], fcal_val_rPhone.Cal_val[101], fcal_val_rPhone.Cal_val[102], fcal_val_rPhone.Cal_val[103],	\
		fcal_val_rPhone.Cal_val[104], fcal_val_rPhone.Cal_val[105], fcal_val_rPhone.Cal_val[106], fcal_val_rPhone.Cal_val[107], fcal_val_rPhone.Cal_val[108], fcal_val_rPhone.Cal_val[109]);

	
	for(i=0; i<CalLength; i++){
		wBuf[3+i] = fcal_val_rPhone.Cal_val[i];
		
		if(fcal_val_rPhone.Cal_val[i] != fcal_val_rMCU.Cal_val[i]){
			wFlg = 1;
			pr_err("[MCU] %s i:%d, rPhone:%x, rMCU:%x.\n", __func__, i, fcal_val_rPhone.Cal_val[i], fcal_val_rMCU.Cal_val[i]);
		}
	}
	
	if(wFlg == 0){
		pr_err("[MCU] %s no need update phone's cali data to mcu.\n", __func__);
	}else
		Zen7_MSP430FR2311_wI2CtoMCU(wBuf, sizeof(wBuf));

	pr_err("[MCU] %s end.\n", __func__);
	return rc;
}


//Reset user calibration data
static void UserCaliReset(void){
	int rc;
	
	rc = ksys_unlink(MCU_BAKEUP_USER_FILE_NAME);
	read_mcu_cal_thed();	//read factory file to mcu.
	pr_err("[MCU] %s done, rc:%d.\n", __func__, rc);
}

//Backup 110 bytes to mcu_bak.
static int Backup_user_cal_thed(void)
{
	char buf[400]={0};
	char MCUBuf[10];
	unsigned char i;
	int rc;

	pr_err("[MCU] %s.\n", __func__);

	for(i=0; i<CalLength; i++){
		sprintf(MCUBuf, " %X", fcal_val_UserK.Cal_val[i]);
		strcat(buf, MCUBuf);		
	}
	rc = file_op(MCU_BAKEUP_USER_FILE_NAME, CAL_DATA_OFFSET,
		(char *)&buf, 3*CalLength*sizeof(char), FILE_OP_WRITE);
	if (rc<0)
		pr_err("[MCU] %s:write file:%s err!\n", __func__, MCU_BAKEUP_USER_FILE_NAME);
	
	return rc;
}

static void mcu_cal_work(struct work_struct *work){
	unsigned char i;

	pr_err("[MCU] %s UserKState:%d.\n", __func__, UserKState);

	if(UserKState == UserK_SaveK){
		Backup_user_cal_thed();
		UserKState = UserK_TBD;
		return;
	}

	if(UserKState == UserK_NoSaveK){
		for(i=0; i<CalLength; i++)
			fcal_val_UserK.Cal_val[i] = 0;		
		
		UserKState = UserK_TBD;

		if(IsFileNull(MCU_BAKEUP_USER_FILE_NAME) == 1){
			read_usek_cal3threshlod();	//read motor_fw2 file to mcu.
		}else{
			read_mcu_cal_thed();		//read factory file to mcu.
		}
		
		return;
	}

	if(UserKState == UserK_Reset){
		UserCaliReset();
		UserKState = UserK_TBD;
		return;
	}
	
	if(UserKState == UserK_AbortK){
		for(i=0; i<CalLength; i++)
			fcal_val_UserK.Cal_val[i] = 0;	
				
		UserKState = UserK_TBD;
		
		if(IsFileNull(MCU_BAKEUP_USER_FILE_NAME) == 1){
			read_usek_cal3threshlod();	//read motor_fw2 file to mcu.
		}else{
			read_mcu_cal_thed();		//read factory file to mcu.
		}	
		
		return;
	}
}
//=====

int Zen7_MSP430FR2311_Set_ParamMode(const uint16_t* vals) {
	unsigned char i = 0;
	
	//Total len 22: 0xAA 0x55 0x0C dir freq*6 step*6 mode*6.
	unsigned char MSP430ParamMode[]={0xAA, 0x55, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	pr_err("[MCU] %s +\n", __func__);	
	//Copy dir freq*6 step*6 mode*6.
	for(i=0; i<19; i++){
		MSP430ParamMode[3+i] = (unsigned char)vals[i];
	}
	
	pr_err("[MCU] dump param=%d, %d %d %d %d %d %d, %d %d %d %d %d %d, %d %d %d %d %d %d\n", 
			vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7], vals[8], vals[9], vals[10], vals[11], vals[12],
			vals[13], vals[14], vals[15], vals[16], vals[17], vals[18]);

	pr_err("[MCU] %d, %d %d %d %d %d %d, %d %d %d %d %d %d, %d %d %d %d %d %d\n", 
			MSP430ParamMode[3], MSP430ParamMode[4], MSP430ParamMode[5], MSP430ParamMode[6], MSP430ParamMode[7], MSP430ParamMode[8], MSP430ParamMode[9], 
			MSP430ParamMode[10], MSP430ParamMode[11], MSP430ParamMode[12], MSP430ParamMode[13], MSP430ParamMode[14], MSP430ParamMode[15],
			MSP430ParamMode[16], MSP430ParamMode[17], MSP430ParamMode[18], MSP430ParamMode[19], MSP430ParamMode[20], MSP430ParamMode[21]);

	
	mutex_lock(&MSP430FR2311_control_mutex);
	MSP430FR2311_wakeup(1);
	if (MCUState<MCU_READY) {
		pr_err("[MCU] %s Not ready!, state=%d\n", __func__, MCUState);
		mutex_unlock(&MSP430FR2311_control_mutex);
		return -MCUState;
	}
	
	//Not ure this time's value.
	powerDownDuration=DEFAULT_POWERDOWNDURATION;
	bShowStopInfoOnce=1;
			
	if (!MSP430_I2CWriteA(MSP430_READY_I2C, MSP430ParamMode, sizeof(MSP430ParamMode))) {
		MSP430FR2311_wakeup(0);
		mutex_unlock(&MSP430FR2311_control_mutex);

		pr_err("[MCU] %s I2C error!\n", __func__);
		return -1;
	}
	
	MSP430FR2311_wakeup(0);
	mutex_unlock(&MSP430FR2311_control_mutex);

	pr_err("[MCU] %s -\n", __func__);
	return 0;
}

//Zen7 I2C write format: 0xAA 0x55 cmd xx....
static int Zen7_MSP430FR2311_wI2CtoMCU(uint8_t *buf, uint8_t len){

	mutex_lock(&MSP430FR2311_control_mutex);
	MSP430FR2311_wakeup(1);
	if (MCUState<MCU_READY) {
		pr_err("[MCU] %s Not ready!, state=%d\n", __func__, MCUState);
		mutex_unlock(&MSP430FR2311_control_mutex);
		return -MCUState;
	}
	
	if (!MSP430_I2CWriteA(MSP430_READY_I2C, buf, len)) {
		MSP430FR2311_wakeup(0);
		mutex_unlock(&MSP430FR2311_control_mutex);

		pr_err("[MCU] %s I2C error!\n", __func__);
		return -1;
	}

	MSP430FR2311_wakeup(0);
	mutex_unlock(&MSP430FR2311_control_mutex);

	pr_err("[MCU] %s cmd:0x%x\n", __func__, buf[2]);
	return 0;
}

//Zen7 I2C read Format. Protocol: s slave_add(w) 0xAA 0x55 cmd stop; msleep(100); s slave_add(r) xx... stop;  
uint8_t dAngle[4];
int Zen7_MSP430FR2311_rI2CtoCPU(uint8_t cmd, uint8_t *rBuf, uint8_t len) {
	uint8_t CmdBuf[3] = {0xAA, 0x55, 0x00};
	//uint8_t i =0;
	
	CmdBuf[2] = cmd;
	
	MSP430FR2311_wakeup(1);
	MSP430_I2CWriteA(MSP430_READY_I2C, CmdBuf, sizeof(CmdBuf));
	//msleep(3);	//Delay to wait MCU prepare data.
	if (!MSP430_I2CRead(MSP430_READY_I2C, rBuf, len)) {
		pr_err("[MCU] %s I2C error!", __func__);
		MSP430FR2311_wakeup(0);
		return -1;
	}
	MSP430FR2311_wakeup(0);

	pr_err("[MCU] %s", __func__);
	//for(i=0; i<len; i++){
	//	pr_err("%x", rBuf[i]);
	//}
	
	return 0;

}


//Proc note for deal with angle cal and read angle. 
int Zen7_MSP430FR2311_DealAngle(uint16_t        *buf, uint8_t len) {
	int rc = 0;
	uint8_t wBuf[7] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t i;
	uint8_t CmdBuf[3] = {0xAA, 0x55, 0x00};
	uint8_t rAngle[6];
	
	for(i=0; i<len; i++){
		wBuf[2+i] = (unsigned char)buf[i];
	}
		
	switch(wBuf[2]){			
		//Angle sensor cal.
		case 0x61:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));
			break;

		//Read angle raw data.
		case 0x62:
			//rc = Zen7_MSP430FR2311_rI2CtoCPU(0x62, dAngle, sizeof(dAngle));
			
			//In order to reduce printf cat angle log.
			MSP430FR2311_wakeup(1);
			CmdBuf[2] = 0x62;
			MSP430_I2CWriteA_NoLog(MSP430_READY_I2C, CmdBuf, sizeof(CmdBuf));
			//msleep(20);	//Delay to wait MCU prepare data.
			if (!MSP430_I2CRead_NoLog(MSP430_READY_I2C, rAngle, sizeof(rAngle))){
				pr_err("[MCU] %s I2C error!", __func__);
				MSP430FR2311_wakeup(0);
				rc = -1;
			}
			dAngle[0] = rAngle[2];
			dAngle[1] = rAngle[3];
			dAngle[2] = rAngle[4];
			dAngle[3] = rAngle[5];
			MSP430FR2311_wakeup(0);
			break;
		
		//Set MCU auto report angle.
		case 0x63:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));
			break;	

		//Set angle stop thredhold. Format: echo 0x64 Upthreshold(179) Downthreshold(1) > motor_angle.
		case 0x64:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));
			break;	

		//Dis(1)/En(0) judge angle stop condition. echo 135 disable(1) > motor_angle.
		case 0x87:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));
			break;

		//Specified angle. echo 254 x > motor_angle.
		case 0xFE:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));
			break;
		
		default:
			pr_err("%s [MCU] param fail!\n", __func__);
			rc = -1;
			break;
	}

    return rc;
}


//Proc note handle for set drv param.
uint8_t drv_state[7];
int Zen7_MSP430FR2311_wrDrv(uint16_t        *buf, uint8_t len){
	int rc = 0;
	uint8_t wBuf[7] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00};	//len 7 is cmd 0x40's length.
	uint8_t i;
	
	for(i=0; i<len; i++){
		wBuf[2+i] = (unsigned char)buf[i];
	}
	
	switch(wBuf[2]){
		//Update drv state.
		case 0x20:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));
			break;

		//Read drv state.
		case 0x21:
			rc = Zen7_MSP430FR2311_rI2CtoCPU(0x21, drv_state, sizeof(drv_state));
			break;
		
		//Write drv. 0xAA 0x55 0x40 , SPI_REG_CTRL1, SPI_REG_CTRL2, SPI_REG_CTRL3, SPI_REG_CTRL4
		case 0x40:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));	//2 is header 0xAA 0x55.
			break;
		
		//Write vref. 0xAA  0x55  0x60  0x32(Vref  duty=50), 0x64 is 100%.
		case 0x60:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));
			break;
		
		default:
			pr_err("%s [MCU] param fail!\n", __func__);
			rc = -1;
			break;
	}

	return rc;
}
	
//Proc note handle for set akm param.
uint8_t akm_temp[8];
uint8_t akm_Ktemp[20];
int Zen7_MSP430FR2311_wrAKM(uint16_t        *buf, uint8_t len){
	int rc = 0;
	uint8_t wBuf[27] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};	//len 27 is cmd 0x65's length.
	uint8_t i;
	
	for(i=0; i<len; i++){
		wBuf[2+i] = (unsigned char)buf[i];
	}
	
	switch(wBuf[2]){

		//Write akm. 0xAA 0x55 0x65 + 24bytes threadhold.
		case 0x65:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));	//2 is header 0xAA 0x55.
			break;
		
		//Update akm raw data to mcu temp buffer.
		case 0x66:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));
			break;

		//Read akm register.
		case 0x67:
			rc = Zen7_MSP430FR2311_rI2CtoCPU(0x67, akm_temp, sizeof(akm_temp));
			break;

		//Write akm threadhold.
		case 0x68:
			rc = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, (2+len));
			break;

		//Read raw_data, (BOP1Y_15_0, BOP1Z_15_0), (BOP1Y_15_0, BRP1Z_15_0), (BRP1Y_15_0, BOP1Z_15_0), (BRP1Y_15_0, BRP1Z_15_0).
		case 0x69:
			rc = Zen7_MSP430FR2311_rI2CtoCPU(0x69, akm_Ktemp, sizeof(akm_Ktemp));
			break;
		
		default:
			pr_err("%s [MCU] param fail!\n", __func__);
			rc = -1;
			break;
	}

	return rc;
}

//Proc note handle for motor_k.
uint8_t k_temp[2];
int Zen7_MSP430FR2311_wrMotorK(uint16_t *buf, uint8_t len){
	int rc = 0;
	uint8_t wBuf[10] = {0xAA, 0x55, 0x86};	//len 3 is cmd 0x86's length.
	uint8_t i;
	
	for(i=0; i<len; i++){
		wBuf[2+i] = (unsigned char)buf[i];
	}
	
	switch(wBuf[2]){

		//Read motor_k. 0xAA 0x55 0x86 xx xx.
		case 0x86:
			rc = Zen7_MSP430FR2311_rI2CtoCPU(0x86, k_temp, sizeof(k_temp));
			break;
				
		default:
			pr_err("%s [MCU] param fail!\n", __func__);
			rc = -1;
			break;
	}

	return rc;
}


//Report motor event to IMS.
#define KEY_MCU_CODE  "KEY_MCU_CODE"	/*Send SubSys UEvent+*/
#define KEY_MCU_VALUE "KEY_MCU_VALUE"	/*Send SubSys UEvent+*/

void report_motor_event(uint8_t OpCode, uint32_t value){
	char mKey_Buf[64];
	char mValue_Buf[512];
	char *envp[] = {mKey_Buf, mValue_Buf, NULL };

	snprintf(mKey_Buf, sizeof(mKey_Buf), "%s=%d", KEY_MCU_CODE, OpCode);
	snprintf(mValue_Buf, sizeof(mValue_Buf), "%s=%d", KEY_MCU_VALUE, value);
	if(kobject_uevent_env(&((mcu_misc.this_device)->kobj), KOBJ_CHANGE, envp) != 0){
		pr_err("[MCU] kobject_uevent_env fail...\n");
	} else {
		//D("[MCU] kobject_uevent_env ok.\n");
	}
}

//Init Motor stop condition
void mStopConditionInit(void){
	uint8_t wBuf[11] = {0xAA, 0x55, 0x85, 1, 179, 4, 0, 73, 4, 0, 73};	
	int ret = 0;
	
	wBuf[3] = StopCondition[0];
	wBuf[4] = StopCondition[1];
	wBuf[5] = StopCondition[2];
	wBuf[6] = StopCondition[3];
	wBuf[7] = StopCondition[4];
	wBuf[8] = StopCondition[5];
	wBuf[9] = StopCondition[6];
	wBuf[10] = StopCondition[7];	
	ret = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, sizeof(wBuf));		//2header + 6bytes + 3bytes.
	if(ret < 0)
		pr_err("[MCU] mStopConditionInit failed.\n");
	else
		pr_err("[MCU] mStopConditionInit ok.\n");

	#ifdef ASUS_FTM_BUILD
		wBuf[2] = 0x87;
		wBuf[3] = 1;	//Enable
		ret = Zen7_MSP430FR2311_wI2CtoMCU(wBuf, 4);		//2header + 2bytes.
		if(ret < 0)
			pr_err("[MCU] disable judge angle failed.\n");
		else
			pr_err("[MCU] disable judge angle ok.\n");
	#endif

	//Compare MCU's calibration data with phone backed.
	if(!Zen7_MSP430FR2311_rI2CtoCPU(0x70, &(fcal_val_rMCU.Cal_val[0]), CalLength)){				
		if(IsFileNull(MCU_BAKEUP_USER_FILE_NAME) == 1){
			CompareCaliData(MCU_BAKEUP_USER_FILE_NAME);		//read userk file to mcu.
		}else{
			if(IsFileNull(MCU_BAKEUP_FILE_NAME) == 1)
				CompareCaliData(MCU_BAKEUP_FILE_NAME);			//read factory file to mcu.
		}
	}else
		pr_err("[MCU] read calibration data from mcu error!\n");					
}

//========================================================Trigger WQ========================================
void WQ_Trigger(void){

	pr_err("[MCU] %s.\n", __func__);
	if(mcu_info->mcu_wq){
		cancel_delayed_work(&report_work);
		queue_delayed_work(mcu_info->mcu_wq, &report_work, msecs_to_jiffies(10));
		pr_err("[MCU] %s success.\n", __func__);
	}	
}

unsigned char KdState(void){
	return MCUState;
}
