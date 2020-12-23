/*
 * Driver for the NXP PCA9468 battery charger.
 *
 * Author: Clark Kim <clark.kim@nxp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/debugfs.h>
#include <linux/power/pca9468_charger.h>

#if defined (CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif /* CONFIG_OF */

#include <linux/wakelock.h>
#ifdef CONFIG_USBPD_PHY_QCOM
#include <linux/usb/usbpd.h>		// Use Qualcomm USBPD PHY
#endif
#include <linux/uaccess.h>			// NTC compensation

#include <linux/msm_drm_notify.h> //Add to get drm notifier
#include <drm/drm_panel.h>

#include "supply/qcom/smb5-lib.h"

#define BITS(_end, _start) ((BIT(_end) - BIT(_start)) + BIT(_end))
#define MASK2SHIFT(_mask)	__ffs(_mask)
#define MIN(a, b)	   ((a < b) ? (a):(b))

//
// Register Map
//
#define PCA9468_REG_DEVICE_INFO 		0x00	// Device ID, revision
#define PCA9468_BIT_DEV_REV				BITS(7,4)
#define PCA9468_BIT_DEV_ID				BITS(3,0)

#define PCA9468_REG_INT1				0x01	// Interrupt register
#define PCA9468_BIT_V_OK_INT			BIT(7)
#define PCA9468_BIT_NTC_TEMP_INT		BIT(6)
#define PCA9468_BIT_CHG_PHASE_INT		BIT(5)
#define PCA9468_BIT_CTRL_LIMIT_INT		BIT(3)
#define PCA9468_BIT_TEMP_REG_INT		BIT(2)
#define PCA9468_BIT_ADC_DONE_INT		BIT(1)
#define PCA9468_BIT_TIMER_INT			BIT(0)

#define PCA9468_REG_INT1_MSK			0x02	// INT mask for INT1 register
#define PCA9468_BIT_V_OK_M				BIT(7)
#define PCA9468_BIT_NTC_TEMP_M			BIT(6)
#define PCA9468_BIT_CHG_PHASE_M			BIT(5)
#define PCA9468_BIT_RESERVED_M			BIT(4)
#define PCA9468_BIT_CTRL_LIMIT_M		BIT(3)
#define PCA9468_BIT_TEMP_REG_M			BIT(2)
#define PCA9468_BIT_ADC_DONE_M			BIT(1)
#define PCA9468_BIT_TIMER_M				BIT(0)

#define PCA9468_REG_INT1_STS			0x03	// INT1 status regsiter
#define PCA9468_BIT_V_OK_STS			BIT(7)
#define PCA9468_BIT_NTC_TEMP_STS		BIT(6)
#define PCA9468_BIT_CHG_PHASE_STS		BIT(5)
#define PCA9468_BIT_CTRL_LIMIT_STS		BIT(3)
#define PCA9468_BIT_TEMP_REG_STS		BIT(2)
#define PCA9468_BIT_ADC_DONE_STS		BIT(1)
#define PCA9468_BIT_TIMER_STS			BIT(0)

#define PCA9468_REG_STS_A				0x04	// INT1 status register
#define PCA9468_BIT_IIN_LOOP_STS		BIT(7)
#define PCA9468_BIT_CHG_LOOP_STS		BIT(6)
#define PCA9468_BIT_VFLT_LOOP_STS		BIT(5)
#define PCA9468_BIT_CFLY_SHORT_STS		BIT(4)
#define PCA9468_BIT_VOUT_UV_STS			BIT(3)
#define PCA9468_BIT_VBAT_OV_STS			BIT(2)
#define PCA9468_BIT_VIN_OV_STS			BIT(1)
#define PCA9468_BIT_VIN_UV_STS			BIT(0)

#define PCA9468_REG_STS_B				0x05	// INT1 status register
#define PCA9468_BIT_BATT_MISS_STS		BIT(7)
#define PCA9468_BIT_OCP_FAST_STS		BIT(6)
#define PCA9468_BIT_OCP_AVG_STS			BIT(5)
#define PCA9468_BIT_ACTIVE_STATE_STS	BIT(4)
#define PCA9468_BIT_SHUTDOWN_STATE_STS	BIT(3)
#define PCA9468_BIT_STANDBY_STATE_STS	BIT(2)
#define PCA9468_BIT_CHARGE_TIMER_STS	BIT(1)
#define PCA9468_BIT_WATCHDOG_TIMER_STS	BIT(0)

#define PCA9468_REG_STS_C				0x06	// IIN status
#define PCA9468_BIT_IIN_STS				BITS(7,2)

#define PCA9468_REG_STS_D				0x07	// ICHG status
#define PCA9468_BIT_ICHG_STS			BITS(7,1)

#define PCA9468_REG_STS_ADC_1			0x08	// ADC register
#define PCA9468_BIT_ADC_IIN7_0			BITS(7,0)

#define PCA9468_REG_STS_ADC_2			0x09	// ADC register
#define PCA9468_BIT_ADC_IOUT5_0			BITS(7,2)
#define PCA9468_BIT_ADC_IIN9_8			BITS(1,0)

#define PCA9468_REG_STS_ADC_3			0x0A	// ADC register
#define PCA9468_BIT_ADC_VIN3_0			BITS(7,4)
#define PCA9468_BIT_ADC_IOUT9_6			BITS(3,0)

#define PCA9468_REG_STS_ADC_4			0x0B	// ADC register
#define PCA9468_BIT_ADC_VOUT1_0			BITS(7,6)
#define PCA9468_BIT_ADC_VIN9_4			BITS(5,0)

#define PCA9468_REG_STS_ADC_5			0x0C	// ADC register
#define PCA9468_BIT_ADC_VOUT9_2			BITS(7,0)

#define PCA9468_REG_STS_ADC_6			0x0D	// ADC register
#define PCA9468_BIT_ADC_VBAT7_0			BITS(7,0)

#define PCA9468_REG_STS_ADC_7			0x0E	// ADC register
#define PCA9468_BIT_ADC_DIETEMP5_0		BITS(7,2)
#define PCA9468_BIT_ADC_VBAT9_8			BITS(1,0)

#define PCA9468_REG_STS_ADC_8			0x0F	// ADC register
#define PCA9468_BIT_ADC_NTCV3_0			BITS(7,4)
#define PCA9468_BIT_ADC_DIETEMP9_6		BITS(3,0)

#define PCA9468_REG_STS_ADC_9			0x10	// ADC register
#define PCA9468_BIT_ADC_NTCV9_4			BITS(5,0)

#define PCA9468_REG_ICHG_CTRL			0x20	// Change current configuration
#define PCA9468_BIT_ICHG_SS				BIT(7)
#define PCA9468_BIT_ICHG_CFG			BITS(6,0)

#define PCA9468_REG_IIN_CTRL			0x21	// Input current configuration
#define PCA9468_BIT_LIMIT_INCREMENT_EN	BIT(7)
#define PCA9468_BIT_IIN_SS				BIT(6)
#define PCA9468_BIT_IIN_CFG				BITS(5,0)

#define PCA9468_REG_START_CTRL			0x22	// Device initialization configuration
#define PCA9468_BIT_SNSRES				BIT(7)
#define PCA9468_BIT_EN_CFG				BIT(6)
#define PCA9468_BIT_STANDBY_EN			BIT(5)
#define PCA9468_BIT_REV_IIN_DET			BIT(4)
#define PCA9468_BIT_FSW_CFG				BITS(3,0)

#define PCA9468_REG_ADC_CTRL			0x23	// ADC configuration
#define PCA9468_BIT_FORCE_ADC_MODE		BITS(7,6)
#define PCA9468_BIT_ADC_SHUTDOWN_CFG	BIT(5)
#define PCA9468_BIT_HIBERNATE_DELAY		BITS(4,3)

#define PCA9468_REG_ADC_CFG				0x24	// ADC channel configuration
#define PCA9468_BIT_CH7_EN				BIT(7)
#define PCA9468_BIT_CH6_EN				BIT(6)
#define PCA9468_BIT_CH5_EN				BIT(5)
#define PCA9468_BIT_CH4_EN				BIT(4)
#define PCA9468_BIT_CH3_EN				BIT(3)
#define PCA9468_BIT_CH2_EN				BIT(2)
#define PCA9468_BIT_CH1_EN				BIT(1)

#define PCA9468_REG_TEMP_CTRL			0x25	// Temperature configuration
#define PCA9468_BIT_TEMP_REG			BITS(7,6)
#define PCA9468_BIT_TEMP_DELTA			BITS(5,4)
#define PCA9468_BIT_TEMP_REG_EN			BIT(3)
#define PCA9468_BIT_NTC_PROTECTION_EN	BIT(2)
#define PCA9468_BIT_TEMP_MAX_EN			BIT(1)

#define PCA9468_REG_PWR_COLLAPSE		0x26	// Power collapse configuration
#define PCA9468_BIT_UV_DELTA			BITS(7,6)
#define PCA9468_BIT_IIN_FORCE_COUNT		BIT(4)
#define PCA9468_BIT_BAT_MISS_DET_EN		BIT(3)

#define PCA9468_REG_V_FLOAT				0x27	// Voltage regulation configuration
#define PCA9468_BIT_V_FLOAT				BITS(7,0)

#define PCA9468_REG_SAFETY_CTRL			0x28	// Safety configuration
#define PCA9468_BIT_WATCHDOG_EN			BIT(7)
#define PCA9468_BIT_WATCHDOG_CFG		BITS(6,5)
#define PCA9468_BIT_CHG_TIMER_EN		BIT(4)
#define PCA9468_BIT_CHG_TIMER_CFG		BITS(3,2)
#define PCA9468_BIT_OV_DELTA			BITS(1,0)

#define PCA9468_REG_NTC_TH_1			0x29	// Thermistor threshold configuration
#define PCA9468_BIT_NTC_THRESHOLD7_0	BITS(7,0)

#define PCA9468_REG_NTC_TH_2			0x2A	// Thermistor threshold configuration
#define PCA9468_BIT_NTC_THRESHOLD9_8	BITS(1,0)

#define PCA9468_REG_ADC_ACCESS			0x30

#define PCA9468_REG_ADC_ADJUST			0x31
#define PCA9468_BIT_ADC_GAIN			BITS(7,4)

#define PCA9468_REG_ADC_IMPROVE			0x3D
#define PCA9468_BIT_ADC_IIN_IMP			BIT(3)

#define PCA9468_REG_ADC_MODE			0x3F
#define PCA9468_BIT_ADC_MODE			BIT(4)

#define PCA9468_MAX_REGISTER			0x4F


#define PCA9468_IIN_CFG(_input_current)	(_input_current/100000)			// input current, unit - uA
#define PCA9468_ICHG_CFG(_chg_current)	(_chg_current/100000)			// charging current, uint - uA
#define PCA9468_V_FLOAT(_v_float)		((_v_float/1000 - 3725)/5)		// v_float voltage, unit - uV

#define PCA9468_SNSRES_5mOhm			0x00
#define PCA9468_SNSRES_10mOhm			PCA9468_BIT_SNSRES
extern int g_fv_setting;//Add for battery safety upgrade

//ASUS_BSP +++ Add for slow charger
extern bool smartchg_slow_flag;
#define SLOWE_CHARGING_TA_CUR  1000000 //1A
//ASUS_BSP --- Add for slow charger

/* VIN Overvoltage setting from 2*VOUT */
enum {
	OV_DELTA_10P,
	OV_DELTA_30P,
	OV_DELTA_20P,
	OV_DELTA_40P,
};

/* Switching frequency */
enum {
	FSW_CFG_833KHZ,
	FSW_CFG_893KHZ,
	FSW_CFG_935KHZ,
	FSW_CFG_980KHZ,
	FSW_CFG_1020KHZ,
	FSW_CFG_1080KHZ,
	FSW_CFG_1120KHZ,
	FSW_CFG_1160KHZ,
	FSW_CFG_440KHZ,
	FSW_CFG_490KHZ,
	FSW_CFG_540KHZ,
	FSW_CFG_590KHZ,
	FSW_CFG_630KHZ,
	FSW_CFG_680KHZ,
	FSW_CFG_730KHZ,
	FSW_CFG_780KHZ
};

/* Enable pin polarity selection */
#define PCA9468_EN_ACTIVE_H		0x00
#define PCA9468_EN_ACTIVE_L		PCA9468_BIT_EN_CFG
#define PCA9468_STANDBY_FORCED	PCA9468_BIT_STANDBY_EN
#define PCA9468_STANDBY_DONOT	0
#define PCA9468_RCP_DET_ENABLE 	PCA9468_BIT_REV_IIN_DET

/* ADC Channel */
enum {
	ADCCH_VOUT = 1,
	ADCCH_VIN,
	ADCCH_VBAT,
	ADCCH_ICHG,
	ADCCH_IIN,
	ADCCH_DIETEMP,
	ADCCH_NTC,
	ADCCH_MAX
};

/* ADC step */
#define VIN_STEP		16000	// 16mV(16000uV) LSB, Range(0V ~ 16.368V)
#define VBAT_STEP		5000	// 5mV(5000uV) LSB, Range(0V ~ 5.115V)
#define IIN_STEP		4890 	// 4.89mA(4890uA) LSB, Range(0A ~ 5A)
#define ICHG_STEP		9780 	// 9.78mA(9780uA) LSB, Range(0A ~ 10A)
#define DIETEMP_STEP  	435		// 0.435C LSB, Range(-25C ~ 160C)
#define DIETEMP_DENOM	1000	// 1000, denominator
#define DIETEMP_MIN 	-25  	// -25C
#define DIETEMP_MAX		160		// 160C
#define VOUT_STEP		5000 	// 5mV(5000uV) LSB, Range(0V ~ 5.115V)
#define NTCV_STEP		2346 	// 2.346mV(2346uV) LSB, Range(0V ~ 2.4V)
#define ADC_IIN_OFFSET	900000	// 900mA

/* adc_gain bit[7:4] of reg 0x31 - 2's complement */
static int adc_gain[16] = { 0, 1, 2, 3, 4, 5, 6, 7, -8, -7, -6, -5, -4, -3, -2, -1 };

/* Timer definition */
#define PCA9468_VBATMIN_CHECK_T	1000	// 1000ms
#define PCA9468_CCMODE_CHECK1_T	30000	// 30000ms
#define PCA9468_CCMODE_CHECK2_T	10000	// 10000ms
#define PCA9468_CCMODE_CHECK3_T	5000	// 5000ms
#define PCA9468_CVMODE_CHECK_T 	10000	// 10000ms
#define PCA9468_PDMSG_WAIT_T	300		// 200ms-->300ms
#define PCA4968_ADJCC_DELAY_T	150		// 150ms
#define PCA9468_PPS_PERIODIC_T	7000	// 10000ms-->7000ms
#define PCA9468_JEITA_CHECK_T	30000	// 30000ms
#define PCA9468_ADJCC_WAIT_T	1000	// 1000ms

/* Battery Threshold */
#define PCA9468_DC_VBAT_MIN		3500000	// 3500000uV
/* Input Current Limit default value */
#define PCA9468_IIN_CFG_DFT		2000000	// 2000000uA
/* Charging Current default value */
#define PCA9468_ICHG_CFG_DFT	6000000	// 6000000uA
/* Charging Float Voltage default value */
#define PCA9468_VFLOAT_DFT		4380000	// 4380000uV
/* Sense Resistance default value */
#define PCA9468_SENSE_R_DFT		1		// 10mOhm
/* Switching Frequency default value */
#define PCA9468_FSW_CFG_DFT		7		// 1160KHz
/* Charging Done Condition */
#define PCA9468_ICHG_DONE_DFT	1000000	// 500mA
#define PCA9468_IIN_DONE_DFT	700000	// 700mA

/* CC mode 1,2 battery threshold */
#define PCA9468_CC2_VBAT_MIN	4250000 // 4250000uV
#define PCA9468_CC3_VBAT_MIN	4330000	// 4330000uV
/* CC mode 1,2 ICHG value */
#define PCA9468_CC1_ICHG		3800000	// 3800000uA
#define PCA9468_CC2_ICHG		1800000	// 1800000uA
/* CC mode 1,2 IIN_CFG */
#define PCA9468_CC1_IIN_CFG_4A		1900000	// 1900000uA, panel Off
#define PCA9468_CC1_IIN_CFG_3A		1400000	// 1400000uA, panel On
#define PCA9468_CC2_IIN_CFG		900000	// 900000uA

/* Maximum TA voltage threshold */
#define PCA9468_TA_MAX_VOL		10000000 // 10000000uV
/* Maximum TA current threshold */
#define PCA9468_TA_MAX_CUR		2500000	// 2500000uA

/* Minimum TA voltage threshold in Preset mode */
#define PCA9468_TA_MIN_VOL_PRESET	8000000	// 7500000uV-->8000000uV
/* TA voltage threshold starting Adjust CC mode */
#define PCA9468_TA_MIN_VOL_CCADJ	8500000	// 8000000uV-->8500000uV

#define PCA9468_TA_VOL_PRE_OFFSET	400000	// 200000uV-->400000uV
/* Adjust CC mode TA voltage step */
#define PCA9468_TA_VOL_STEP_ADJ_CC	40000	// 40000uV
/* Pre CV mode TA voltage step */
#define PCA9468_TA_VOL_STEP_PRE_CV	20000	// 20000uV

/* PD Message Voltage and Current Step */
#define PD_MSG_TA_VOL_STEP			20000	// 20mV
#define PD_MSG_TA_CUR_STEP			50000	// 50mA

/* JEITA stage threshold */
#define JEITA_STAGE1_MAX			230	// 23degree
#define JEITA_STAGE2_MAX			450	// 50degree->45degree
#define JEITA_STAGE2_MIN			200	// 20degree
#define JEITA_STAGE3_MAX			550	// 55degree
#define JEITA_STAGE3_MIN			420	// 47degree->42degree
#define JEITA_STAGE4_MIN			520	// 52degree

#define JEITA_STAGE3_IIN			1400000	// 900000uA
#define JEITA_STAGE3_VFLOAT			4100000	// 4100000uV

/* Minimum charging current of Switching charger */
//Change the SWCHG_ICL_MIN from 25mA to 50mA .
//In order to fix the issue. The VBUS still keeps HIGH after unplug charger
#define SWCHG_ICL_MIN				100000	// 25000uA
#define SWCHG_ICL_MAX				3000000	// 1650000uA   //modified by power team's request
#define SWCHG_ICL_NORMAL			3000000	// 3000000uA	// switching charger ICL after charging done
#define SWCHG_VFLOAT_DFT			4380000	// 4380000uV

/* NTC Node */
#define NTC_NODE_PATH		"/sys/devices/platform/soc/c440000.qcom,spmi/spmi-0/spmi0-02/c440000.qcom,spmi:qcom,pm8150b@2:vadc@3100/iio:device1/in_temp_wp_therm_input"
/* NTC threshold */
#define NTC_STAGE1_MAX		46000	// 46degree, milidegree unit
#define NTC_STAGE2_MIN		43000	// 43degree, milidegree unit
#define NTC_STAGE2_IIN		900000	// 900mA

/* ASUS BSP : Add test variable +++ */
int g_panel_off_iin = PCA9468_CC1_IIN_CFG_4A;
int g_panel_on_iin = PCA9468_CC1_IIN_CFG_3A;
int g_high_volt_4P25_iin = PCA9468_CC2_IIN_CFG;
int g_inov_temp_max = NTC_STAGE1_MAX;
int g_inov_temp_min = NTC_STAGE2_MIN;
int g_inov_overtemp_iin = NTC_STAGE2_IIN;
int g_inov_overtemp_iin_low = NTC_STAGE2_IIN;

/* ASUS BSP : Add external variable +++ */
extern volatile enum POGO_ID ASUS_POGO_ID;
extern bool g_Charger_mode;
extern struct gpio_control *global_gpio;

//[+++]Add to update the VID when PD gets the correct PID
extern bool PE_check_asus_vid(void);
extern void PD_notify_VID(void);
//[---]Add to update the VID when PD gets the correct PID

/* NTC stage */
enum {
	NTC_STAGE1,		/* ~46degree, IIN - 1900mA */
	NTC_STAGE2,		/* 43degree~, IIN - 900mA */
	NTC_STAGE_MAX,
};


/* INT1 Register Buffer */
enum {
	REG_INT1,
	REG_INT1_MSK,
	REG_INT1_STS,
	REG_INT1_MAX
};

/* STS Register Buffer */
enum {
	REG_STS_A,
	REG_STS_B,
	REG_STS_C,
	REG_STS_D,
	REG_STS_MAX
};

/* Direct Charging State */
enum {
	DC_STATE_NO_CHARGING,		/* no charigng */
	DC_STATE_CHECK_VBAT,		/* check min battery level */
	DC_STATE_PRESET_DC, 		/* Preset Direct Charging configuration */
	DC_STATE_ADJUST_CC,			/* Adjust CC mode */
	DC_STATE_START_CC,			/* Start CC mode */
	DC_STATE_CHECK_CC,			/* Check CC mode status */
	DC_STATE_START_CC2,			/* Start CC2 mode */
	DC_STATE_CHECK_CC2,			/* Check CC2 mode status */
	DC_STATE_START_CV,			/* Start CV mode */
	DC_STATE_CHECK_CV,			/* Check CV mode status */
	DC_STATE_CHARGING_DONE,		/* Charging Done */
	DC_STATE_NO_CHARGING_JEITA,	/* Stop Charging from JEITA condition */
	DC_STATE_CHANGE_CV_JEITA,	/* Change CV level from JEITA condition */
	DC_STATE_MAX,
};


/* CC Mode Status */
enum {
	CCMODE_CHG_LOOP,
	CCMODE_VFLT_LOOP,
	CCMODE_IIN_LOOP,
	CCMODE_LOOP_INACTIVE1,
	CCMODE_LOOP_INACTIVE2,
	CCMODE_LOOP_INACTIVE3,
	CCMODE_VIN_UVLO,
};

/* CV Mode Status */
enum {
	CVMODE_CHG_LOOP,
	CVMODE_VFLT_LOOP,
	CVMODE_IIN_LOOP,
	CVMODE_LOOP_INACTIVE,
	CVMODE_CHG_DONE,
	CVMODE_VIN_UVLO,
};

/* Timer ID */
enum {
	TIMER_ID_NONE,
	TIMER_VBATMIN_CHECK,
	TIMER_ENTER_ADJ_CCMODE,
	TIMER_CCMODE_CHECK,
	TIMER_ENTER_CCMODE2,
	TIMER_CCMODE2_CHECK,
	TIMER_ENTER_CVMODE,
	TIMER_CVMODE_CHECK,
	TIMER_ENTER_JEITA,
	TIMER_JEITA_CHECK,
	TIMER_PDMSG_SEND,
};

/* PD Message Type */
enum {
	PD_MSG_REQUEST_APDO,
	PD_MSG_REQUEST_FIXED_PDO,
};

/* JEITA stage */
enum {
	JEITA_STAGE1,	/* ~20degree */
	JEITA_STAGE2,	/* 20~50degree */
	JEITA_STAGE3,	/* 50~60degree */
	JEITA_STAGE4,	/* 60degree~ */
	JEITA_STAGE_MAX,
};

struct pca9468_charger *pca9468_chg_dev;	//ASUS BSP for globle dev

int g_PanelOnOff = 0;

/**
 * struct pca9468_charger - pca9468 charger instance
 * @monitor_wake_lock: lock to enter the suspend mode
 * @lock: protects concurrent access to online variables
 * @dev: pointer to device
 * @regmap: pointer to driver regmap
 * @mains: power_supply instance for AC/DC power
 * @dc_wq: work queue for the algorithm and pps periodic timer
 * @timer_work: timer work for charging
 * @timer_id: timer id for timer_work
 * @timer_period: timer period for timer_work
 * @last_update_time: last update time after sleep
 * @pps_work: pps work for PPS periodic time
 * @pd: phandle for qualcomm PMI usbpd-phy
 * @pd_port: the pd port, PMI pd phy or standalone PD phy
 * @mains_online: is AC/DC input connected
 * @charging_state: direct charging state
 * @jeita_stage: JEITA stage
 * @ntc_stage: NTC stage
 * @cc_ta_cur: TA current of the initial CC mode
 * @ta_cur: AC/DC(TA) current, uA
 * @ta_vol: AC/DC(TA) voltage, uV
 * @ta_objpos: AC/DC(TA) PDO object position
 * @prev_ta_vol: Previous TA voltage for PS_RDY time workaround.
 * @pre_iin_adc: previous IIN ADC in CC adjust mode, uA
 * @ta_max_cur: TA maximum current for the direct charging, uA
 * @adc_comp_gain: adc gain for compensation
 * @pdata: pointer to platform data
 * @debug_root: debug entry
 * @debug_address: debug register address
 */
struct pca9468_charger {
	struct wake_lock	monitor_wake_lock;
	struct mutex		lock;
	struct device		*dev;
	struct regmap		*regmap;
	struct power_supply	*mains;
	struct power_supply *bat_psy;

	struct workqueue_struct *dc_wq;
	
	struct delayed_work timer_work;
	unsigned int		timer_id;
	unsigned long      	timer_period;
	unsigned long		last_update_time;

	struct delayed_work	pps_work;
	struct delayed_work	prop_charging_enable_work;
#ifdef CONFIG_USBPD_PHY_QCOM
	struct usbpd 		*pd;
#endif
#ifdef CONFIG_DUAL_PD_PORT
	unsigned int		pd_port;
	unsigned int		chg_enable;
#endif
	bool				mains_online;
	unsigned int 		charging_state;

	unsigned int		jeita_stage;
	int					ntc_stage;
	unsigned int		cc_ta_cur;
	
	unsigned int		ta_cur;
	unsigned int		ta_vol;
	unsigned int		ta_objpos;
	unsigned int		prev_ta_vol;

	int					pre_iin_adc;
	unsigned int		ta_max_cur;

	int					adc_comp_gain;
	
	struct pca9468_platform_data *pdata;

	/* debug */
	struct dentry		*debug_root;
	u32					debug_address;

	// Register framebuffer notify
	struct notifier_block notifier;
};


#ifdef CONFIG_USBPD_PHY_QCOM
static int pca9468_usbpd_setup(struct pca9468_charger *pca9468);
#endif


/*******************************/
/* Switching charger control function */
/*******************************/
/* This function needs some modification by a customer */
static int pca9468_set_switching_charger( bool enable, 
					unsigned int input_current, 
					unsigned int charging_current, 
					unsigned int vfloat)
{
	int ret;
	struct power_supply *psy_swcharger;
#ifdef CONFIG_DUAL_PD_PORT
	struct power_supply *psy_pca9468;
#endif
	union power_supply_propval val;

	pr_info("%s: enable=%d, iin=%d, ichg=%d, vfloat=%d\n", 
		__func__, enable, input_current, charging_current, vfloat);
	
	/* Insert Code */

	/* Get power supply name */
#ifdef CONFIG_USBPD_PHY_QCOM
#ifdef CONFIG_DUAL_PD_PORT
	/* Get the power port property */
	psy_pca9468 = power_supply_get_by_name("pca9468-mains");
	ret = power_supply_get_property(psy_pca9468, POWER_SUPPLY_PROP_PD_PORT, &val);
	if (val.intval == POWER_SUPPLY_PD_PORT_PMI) {
		psy_swcharger = power_supply_get_by_name("usb");
	} else {
		psy_swcharger = power_supply_get_by_name("main");
	}
#else
	psy_swcharger = power_supply_get_by_name("usb");
#endif
#else
	psy_swcharger = power_supply_get_by_name("sw-charger");
#endif
	if (psy_swcharger == NULL) {
		pr_err("%s: cannot get power_supply_name-usb\n", __func__);
		ret = -ENODEV;
		/* retry get the supply name as "main" */
#ifdef CONFIG_DUAL_PD_PORT	
		psy_swcharger = power_supply_get_by_name("main");
		if (psy_swcharger == NULL) {
			pr_err("%s: cannot get power_supply_name-main\n", __func__);
			ret = -ENODEV;
			goto error;
		}
#else
		goto error;
#endif
	}

	if (enable == true)	{
		/* enable charger */
#ifdef CONFIG_USBPD_PHY_QCOM
		val.intval = enable;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			goto error;
		pr_info("%s: set property Charging_Enable=%d\n", __func__, enable);
		
		//use "usb" to vote ICL instead
		psy_swcharger = power_supply_get_by_name("usb");
		if (psy_swcharger == NULL) {			
				pr_info("%s: psy_swcharger == NULL\n", __func__);
				ret = -ENODEV;
				goto error;
		}

		/* input_current */
		val.intval = input_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0) {			
			pr_info("%s: set_property FAIL\n", __func__);
			goto error;
		}
		pr_info("%s: set property Current_Max=%d\n", __func__, input_current);
#else
		/* Set Switching charger */

		/* input current */
		val.intval = input_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0)
			goto error;
		/* charging current */
		val.intval = charging_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT, &val);
		if (ret < 0)
			goto error;
		/* vfloat voltage */
		val.intval = vfloat;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &val);
		if (ret < 0)
			goto error;

		/* it depends on customer's code to enable charger */
		val.intval = enable;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			goto error;
#endif
	} else {
		/* disable charger */
#ifdef CONFIG_USBPD_PHY_QCOM
		val.intval = enable;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			goto error;
		pr_info("%s: set property Charging_Enable=%d\n", __func__, enable);

		//use "usb" to vote ICL instead
		psy_swcharger = power_supply_get_by_name("usb");
		if (psy_swcharger == NULL) {
			ret = -ENODEV;
			goto error;
		}
		
		/* input_current */
		val.intval = input_current;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0)
			goto error;
		pr_info("%s: set property Current_Max=%d\n", __func__, input_current);
#else
		/* it depends on customer's code to disable charger */
		val.intval = enable;
		ret = power_supply_set_property(psy_swcharger, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			return ret;
#endif
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

static int pca9468_get_switching_charger_property(enum power_supply_property prop,
						  union power_supply_propval *val)
{
	int ret;
	struct power_supply *psy_swcharger;
#ifdef CONFIG_DUAL_PD_PORT
	struct power_supply *psy_pca9468;
	union power_supply_propval pr_val;
#endif

	/* Get power supply name */
#ifdef CONFIG_USBPD_PHY_QCOM
#ifdef CONFIG_DUAL_PD_PORT
	/* Get the power port property */
	psy_pca9468 = power_supply_get_by_name("pca9468-mains");
	ret = power_supply_get_property(psy_pca9468, POWER_SUPPLY_PROP_PD_PORT, &pr_val);
	if (pr_val.intval == POWER_SUPPLY_PD_PORT_PMI) {
		psy_swcharger = power_supply_get_by_name("usb");
	} else {
		psy_swcharger = power_supply_get_by_name("main");
	}

	if(prop == POWER_SUPPLY_PROP_CURRENT_MAX) {
		psy_swcharger = power_supply_get_by_name("main");
	}
#else
	psy_swcharger = power_supply_get_by_name("usb");
#endif
#else
	psy_swcharger = power_supply_get_by_name("sw-charger");
#endif
	if (psy_swcharger == NULL) {
		ret = -EINVAL;
		goto error;
	}
	ret = power_supply_get_property(psy_swcharger, prop, val);

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/******************/
/* Send PD message */
/******************/
/* This function needs some modification by a customer */
static int pca9468_send_pd_message(struct pca9468_charger *pca9468, unsigned int msg_type)
{
#ifdef CONFIG_USBPD_PHY_QCOM
#ifdef CONFIG_DUAL_PD_PORT
	struct power_supply *rt_charger;
	union power_supply_propval val;
	u16	out_vol, op_cur, fixed_pdo;
#endif
#else
	u8 msg_buf[4];	/* Data Buffer for raw PD message */
	unsigned int max_cur;
	unsigned int op_cur, out_vol;
#endif
	/* Workaround for PS_RDY time */
	unsigned int new_ta_vol;

	int ret = 0;
	
	mutex_lock(&pca9468->lock);

	if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
		/* Vbus reset happened in the previous PD communication */
		goto out;
	}
	
#ifdef CONFIG_USBPD_PHY_QCOM
	/* check the phandle */
	if (pca9468->pd == NULL) {
		pr_info("%s: get phandle\n", __func__);
		ret = pca9468_usbpd_setup(pca9468);
		if (ret != 0) {
			dev_err(pca9468->dev, "Error usbpd setup!\n");
			pca9468->pd = NULL;
			goto out;
		}
	}
#endif
	
	pr_info("[PCA] %s: pd_port=%d, msg_type=%d, ta_cur=%d, ta_vol=%d, ta_objpos=%d\n", 
		__func__, pca9468->pd_port, msg_type, pca9468->ta_cur, pca9468->ta_vol, pca9468->ta_objpos);
		
	switch (msg_type) {
	case PD_MSG_REQUEST_APDO:
		/* Cancel pps request timer */
		cancel_delayed_work(&pca9468->pps_work);
		/* Workaround for PS_RDY time */
		if (pca9468->prev_ta_vol == pca9468->ta_vol) {
			new_ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
		} else {
			new_ta_vol = pca9468->ta_vol;
		}
#ifdef CONFIG_USBPD_PHY_QCOM
#ifdef CONFIG_DUAL_PD_PORT
		if (pca9468->pd_port == POWER_SUPPLY_PD_PORT_PMI) {
			pr_info("[PCA] request pdo to PMI\n");
			ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, new_ta_vol, pca9468->ta_cur);

			if (ret == -EBUSY) {
				/* wait 100ms */
				msleep(100);
				/* try again */
				ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, new_ta_vol, pca9468->ta_cur);
			}
		} else if (pca9468->pd_port == POWER_SUPPLY_PD_PORT_RT) {
			/* insert the code */
			/* call standalone PD API function */
			pr_info("[PCA] request pdo to RT-Port\n");

			rt_charger = power_supply_get_by_name("rt-chg");
			if (rt_charger == NULL) {
				dev_err(pca9468->dev, "Error getting rt-charger!\n");
				ret = -ENODEV;
				goto out;
			}
			/* Send the request message */
			out_vol = new_ta_vol/1000;	/* change unit to mV */
			op_cur = pca9468->ta_cur/1000; /* change unit to mA */
			fixed_pdo = 0;	/* not fixed pdo, apdo */
			val.intval = (out_vol<<16) | (fixed_pdo<<15) | (op_cur);
			ret = power_supply_set_property(rt_charger, POWER_SUPPLY_PROP_PD_CAP, &val);
			if (ret == -EBUSY) {
				/* wait 100ms */
				msleep(100);
				/* try again */
				ret = power_supply_set_property(rt_charger, POWER_SUPPLY_PROP_PD_CAP, &val);
			}
		} else {
			pr_err("%s: Error:No CC/PD chip\n", __func__);
			ret = -ENODEV;
		}
#else
		ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, new_ta_vol, pca9468->ta_cur);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, new_ta_vol, pca9468->ta_cur);
		}
#endif
#else
		//jonathan
		op_cur = pca9468->ta_cur/50000;		// Operating Current 50mA units
		out_vol = new_ta_vol/20000;			// Output Voltage in 20mV units
		msg_buf[0] = op_cur & 0x7F;			// Operating Current 50mA units - B6...0
		msg_buf[1] = (out_vol<<1) & 0xFE;	// Output Voltage in 20mV units - B19..(B15)..B9
		msg_buf[2] = (out_vol>>7) & 0x0F;	// Output Voltage in 20mV units - B19..(B16)..B9,
		msg_buf[3] = pca9468->ta_objpos<<4;	// Object Position - B30...B28

		/* Send the PD message to CC/PD chip */
		/* Todo - insert code */
#endif
#ifdef CONFIG_DUAL_PD_PORT
		if ((ret == 0) && (pca9468->pd_port == POWER_SUPPLY_PD_PORT_PMI)) {
			/* Start pps request timer for PMI */
			/* RT PD phy has its own PPS periodic timer */
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->pps_work,
								msecs_to_jiffies(PCA9468_PPS_PERIODIC_T));
		}
#else
		/* Start pps request timer */
		if (ret == 0) {
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->pps_work,
								msecs_to_jiffies(PCA9468_PPS_PERIODIC_T));
		}
#endif
		/* Workaround for PS_RDY time */
		pca9468->prev_ta_vol = new_ta_vol;
		break;
		
	case PD_MSG_REQUEST_FIXED_PDO:
		cancel_delayed_work(&pca9468->pps_work);
#ifdef CONFIG_USBPD_PHY_QCOM
#ifdef CONFIG_DUAL_PD_PORT
		if (pca9468->pd_port == POWER_SUPPLY_PD_PORT_PMI) {
			ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
			if (ret == -EBUSY) {
				/* wait 100ms */
				msleep(100);
				/* try again */
				ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
			}
		} else if (pca9468->pd_port == POWER_SUPPLY_PD_PORT_RT) {
			/* insert the code */
			/* call standalone PD API function */
			rt_charger = power_supply_get_by_name("rt-chg");
			if (rt_charger == NULL) {
				dev_err(pca9468->dev, "Error getting rt-charger!\n");
				ret = -ENODEV;
				goto out;
			}
			/* Send the request message */
			out_vol = pca9468->ta_vol/1000;	/* change unit to mV */
			op_cur = pca9468->ta_cur/1000; /* change unit to mA */
			fixed_pdo = 1;	/* fixed pdo */
			val.intval = (out_vol<<16) | (fixed_pdo<<15) | (op_cur);
			ret = power_supply_set_property(rt_charger, POWER_SUPPLY_PROP_PD_CAP, &val);
			if (ret < 0) {
				/* wait 100ms */
				msleep(100);
				/* try again */
				ret = power_supply_set_property(rt_charger, POWER_SUPPLY_PROP_PD_CAP, &val);
			}
		} else {
			pr_err("%s: Error:No CC/PD chip\n", __func__);
			ret = -ENODEV;
		}
#else
		ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
		if (ret == -EBUSY) {
			/* wait 100ms */
			msleep(100);
			/* try again */
			ret = usbpd_request_pdo(pca9468->pd, pca9468->ta_objpos, pca9468->ta_vol, pca9468->ta_cur);
		}
#endif
#else
		max_cur = pca9468->ta_cur/10000; 	// Maximum Operation Current 10mA units
		//jonathan
		op_cur = max_cur;					// Operating Current 10mA units
		msg_buf[0] = max_cur & 0xFF;		// Maximum Operation Current -B9..(7)..0
		msg_buf[1] = ((max_cur>>8) & 0x03) | ((op_cur<<2) & 0xFC);	// Operating Current - B19..(15)..10
		msg_buf[2] = ((op_cur>>6) & 0x0F);	// Operating Current - B19..(16)..10, Unchunked Extended Messages Supported  - not support
		msg_buf[3] = pca9468->ta_objpos<<4;	// Object Position - B30...B28

		/* Send the PD message to CC/PD chip */
		/* Todo - insert code */
#endif
		break;

	default:
		break;
	}

out:
	if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
		/* Even though PD communication success, Vbus reset might happen
		    So, check the charging state again */
		ret = -EINVAL;
	}
	
	pr_info("%s: ret=%d\n", __func__, ret);
	mutex_unlock(&pca9468->lock);
	return ret;
}

/* ADC Read function */
static int pca9468_read_adc(struct pca9468_charger *pca9468, u8 adc_ch)
{
	u8 reg_data[2];
	u16 raw_adc;	// raw ADC value
	int conv_adc;	// conversion ADC value
	int ret;
	u8 rsense; /* sense resistance */
	
	switch (adc_ch)
	{
	case ADCCH_VOUT:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_4, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VOUT9_2)<<2) | ((reg_data[0] & PCA9468_BIT_ADC_VOUT1_0)>>6);
		conv_adc = raw_adc * VOUT_STEP;	// unit - uV
		break;
	
	case ADCCH_VIN:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_3, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VIN9_4)<<4) | ((reg_data[0] & PCA9468_BIT_ADC_VIN3_0)>>4);
		conv_adc = raw_adc * VIN_STEP;	// uint - uV
		break;

	case ADCCH_VBAT:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_6, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_VBAT9_8)<<8) | ((reg_data[0] & PCA9468_BIT_ADC_VBAT7_0)>>0);
		conv_adc = raw_adc * VBAT_STEP;		// unit - uV
		break;

	case ADCCH_ICHG:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_2, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		rsense = (pca9468->pdata->snsres == 0) ? 5 : 10;	// snsres : 0 - 5mOhm, 1 - 10mOhm
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_IOUT9_6)<<6) | ((reg_data[0] & PCA9468_BIT_ADC_IOUT5_0)>>2);
		conv_adc = raw_adc * ICHG_STEP;	// unit - uA
		break;

	case ADCCH_IIN:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_1, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC - iin = rawadc*4.89 + (rawadc*4.89 - 900)*adc_comp_gain/100, unit - mA
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_IIN9_8)<<8) | ((reg_data[0] & PCA9468_BIT_ADC_IIN7_0)>>0);
		conv_adc = raw_adc * IIN_STEP + (raw_adc * IIN_STEP - ADC_IIN_OFFSET)*pca9468->adc_comp_gain/100;		// unit - uV
		if (conv_adc < 0)
			conv_adc = 0;	// If ADC raw value is 0, convert value will be minus value because of compensation gain, so in this case conv_adc is 0
		break;

	
	case ADCCH_DIETEMP:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_7, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_DIETEMP9_6)<<6) | ((reg_data[0] & PCA9468_BIT_ADC_DIETEMP5_0)>>2);
		conv_adc = (935 - raw_adc)*DIETEMP_STEP/DIETEMP_DENOM; 	// Temp = (935-rawadc)*0.435, unit - C
		if (conv_adc > DIETEMP_MAX) {
			conv_adc = DIETEMP_MAX;
		} else if (conv_adc < DIETEMP_MIN) {
			conv_adc = DIETEMP_MIN;
		}
		break;

	case ADCCH_NTC:
		// Read ADC value
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_ADC_8, reg_data, 2);
		if (ret < 0) {
			conv_adc = ret;
			goto error;
		}
		// Convert ADC
		raw_adc = ((reg_data[1] & PCA9468_BIT_ADC_NTCV9_4)<<4) | ((reg_data[0] & PCA9468_BIT_ADC_NTCV3_0)>>4);
		conv_adc = raw_adc * NTCV_STEP;		// unit - uV
		break;

	default:
		conv_adc = -EINVAL;
		break;
	}

error:
	pr_info("%s: adc_ch=%d, convert_val=%d\n", __func__, adc_ch, conv_adc);

	return conv_adc;
}


/* Read NTC */
static int pca9468_read_ntc_value(void)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8] = "";
	int l_result = -1;

	fp = filp_open(NTC_NODE_PATH, O_RDWR, 0);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("%s: open (%s) fail\n", __func__, NTC_NODE_PATH);
		return -ENOENT;     /*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_read(fp, buf, 8, &pos_lsts);

	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &l_result);
	if(l_result < 0) {
		pr_err("%s: FAIL. (%d)\n", __func__, l_result);
		return -EINVAL;       /*Invalid argument*/
	} else {
		pr_err("%s: %d\n", __func__, l_result);
	}

	return l_result;
}


static int pca9468_set_vfloat(struct pca9468_charger *pca9468, unsigned int v_float)
{
	int ret, val;

	pr_info("%s: vfloat=%d\n", __func__, v_float);

	/* v float voltage */
	val = PCA9468_V_FLOAT(v_float);
	ret = regmap_write(pca9468->regmap, PCA9468_REG_V_FLOAT, val);
	return ret;
}

static int pca9468_set_charging_current(struct pca9468_charger *pca9468, unsigned int ichg)
{
	int ret, val;

	pr_info("%s: ichg=%d\n", __func__, ichg);

	/* charging current */
	if (ichg > 8000000)
		ichg = 8000000;
	val = PCA9468_ICHG_CFG(ichg);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ICHG_CTRL, PCA9468_BIT_ICHG_CFG, val);
	return ret;
}

/*
static int pca9468_get_input_current(struct pca9468_charger *pca9468)
{
	int ret, val =9;
	unsigned reg_val;

	// Read IIN_CTRL 
	ret = regmap_read(pca9468->regmap, PCA9468_REG_IIN_CTRL, &reg_val);
	if (ret < 0)
		goto error;
	
	reg_val &= 0x3f;

	val = reg_val*100;

	pr_info("[PCA] %s: iin=%d\n", __func__, val);

error:
	return val;
}
*/

static int pca9468_set_input_current(struct pca9468_charger *pca9468, unsigned int iin)
{
	int ret, val;

	pr_info("[PCA] %s: iin=%d\n", __func__, iin);


	/* input current */
	if (iin > 5000000)
		iin = 5000000;
	val = PCA9468_IIN_CFG(iin);

	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_IIN_CTRL, PCA9468_BIT_IIN_CFG, val);
	return ret;
}

static int pca9468_set_charging(struct pca9468_charger *pca9468, bool enable)
{
	int ret, val;

	pr_info("[PCA] %s: enable=%d\n", __func__, enable);

	if (enable == true) {
		/* If PCA9468 is already enabled, we should set it to disable first */ 
		/* Disable PCA9468 */
		val = (PCA9468_EN_ACTIVE_H | PCA9468_STANDBY_FORCED);
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL, PCA9468_BIT_EN_CFG | PCA9468_BIT_STANDBY_EN, val);
		if (ret < 0)
			goto error;
		/* wait 5ms */
		mdelay(5);
		
		/* Enable Reverse current detection */
		val = PCA9468_RCP_DET_ENABLE;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL, PCA9468_BIT_REV_IIN_DET, val);
		if (ret < 0)
			goto error;
		
		/* Improve adc */
		val = 0x5B;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
		if (ret < 0)
			goto error;
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ADC_IMPROVE, PCA9468_BIT_ADC_IIN_IMP, 0);
		if (ret < 0)
			goto error;

		/* For fixing input current error */
		/* Overwrite 0x08(bit3) in 0x41 register */
		/* Disable OCP-AVG. It is the software workaround for OCP-AVG issue */
		val = 0x08;
		ret = regmap_write(pca9468->regmap, 0x41, val);
		if (ret < 0)
			goto error;
		/* Overwrite 0x01 in 0x43 register */
		val = 0x01;
		ret = regmap_write(pca9468->regmap, 0x43, val);
		if (ret < 0)
			goto error;

		/* Overwrite 0x00 in 0x4B register */
		val = 0x00;
		ret = regmap_write(pca9468->regmap, 0x4B, val);
		if (ret < 0)
			goto error;
		/* End for fixing input current error */	
	}
	
	/* Enable PCA9468 */
	val = (enable == true) ? (PCA9468_EN_ACTIVE_L | PCA9468_STANDBY_DONOT): (PCA9468_EN_ACTIVE_H | PCA9468_STANDBY_FORCED);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL, PCA9468_BIT_EN_CFG | PCA9468_BIT_STANDBY_EN, val);
	if (ret < 0)
		goto error;
	
	if (enable == true) {
		/* Wait 50ms, first to keep the start-up sequence */
		mdelay(50);
		/* Wait 150ms */
		msleep(150);
		ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_ADC_IMPROVE, PCA9468_BIT_ADC_IIN_IMP, PCA9468_BIT_ADC_IIN_IMP);
		if (ret  < 0)
			goto error;

		/* Clear bit3 to 0x41 register */
		/* Enable OCP-AVG. It is the software workaround for OCP-AVG issue */
		ret = regmap_update_bits(pca9468->regmap, 0x41, BIT(3), 0);
		if (ret  < 0)
			goto error;

		/* Lock test mode */
		val = 0x00;
		ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
	} else {
		/* Wait 5ms */
		mdelay(5);
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

void pmic_set_pca9468_charging (bool enable)
{
	static bool bSmartChargingDisable = false;
	int ret = 0;

	pr_info("[PCA] %s: enable=%d, bSmartChargingDisable=%d\n", __func__, enable, bSmartChargingDisable);

	// only when Smart Charging disables NXP first, then enable charging can be allowed
	if(enable && bSmartChargingDisable) {
		pr_info("[PCA] %s: try to enable NXP\n", __func__);
		bSmartChargingDisable = false;
		ret = pca9468_set_charging(pca9468_chg_dev, enable);
	}
	else if (!enable) { //disable NXP
		pr_info("[PCA] %s: try to disable NXP\n", __func__);
		bSmartChargingDisable = true;
		ret = pca9468_set_charging(pca9468_chg_dev, enable);
	}
	else {
		pr_info("[PCA] %s: wired, should not enable NXP, skip\n", __func__);		
	}
}

//[+++]For battery safety upgrade, need to change float voltage when the condition is reached
void pmic_set_pca9468_vfloat(unsigned int v_float)
{
	if (pca9468_chg_dev == NULL) {
		pr_info("[PCA] %s: pca9468_chg_dev is NULL", __func__);
		return;
	}
	pca9468_chg_dev->pdata->v_float = v_float;
	pr_info("[PCA] %s: Try to set PCA9468 vfloat\n", __func__);
	pca9468_set_vfloat(pca9468_chg_dev, pca9468_chg_dev->pdata->v_float);
	return;
}
//[---]For battery safety upgrade, need to change float voltage when the condition is reached

/* Stop Charging */
static int pca9468_stop_charging(struct pca9468_charger *pca9468)
{
	int ret = 0;

	/* Check the current state */
	if (pca9468->charging_state != DC_STATE_NO_CHARGING) {
		// Stop Direct charging 
		cancel_delayed_work(&pca9468->timer_work);
		cancel_delayed_work(&pca9468->pps_work);
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_ID_NONE;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		wake_unlock(&pca9468->monitor_wake_lock);

		/* Clear parameter */
		pca9468->jeita_stage = JEITA_STAGE_MAX;
		pca9468->ntc_stage = NTC_STAGE_MAX;
		pca9468->charging_state = DC_STATE_NO_CHARGING;
		/* Workaround for PS_RDY time */
		pca9468->prev_ta_vol = 0;
		
		ret = pca9468_set_charging(pca9468, false);
	}
	
	pr_info("%s: END, ret=%d\n", __func__, ret);
	
	return ret;
}

/* Check Active status */
static int pca9468_check_error(struct pca9468_charger *pca9468)
{
	int ret;
	unsigned int reg_val;
	int vbatt;
	
	/* Read STS_B */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_B, &reg_val);
	if (ret < 0)
		goto error;

	/* Read VBAT_ADC */
	vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);
	
	/* Check Active status */
	if (reg_val & PCA9468_BIT_ACTIVE_STATE_STS) {
		/* PCA9468 is active state */
		/* PCA9468 is in charging */
		/* Check whether the battery voltage is over the minimum voltage level or not */
		if (vbatt > PCA9468_DC_VBAT_MIN) {
			/* Normal charging battery level */
			/* Check temperature regulation loop */
			/* Read INT1_STS register */
			ret = regmap_read(pca9468->regmap, PCA9468_REG_INT1_STS, &reg_val);
			if (reg_val & PCA9468_BIT_TEMP_REG_STS) {				
				/* Over temperature protection */
				pr_err("%s: Device is in temperature regulation", __func__);
				ret = -EINVAL;
			} else {
				/* Normal temperature */
				ret = 0;
			}
		} else {
			/* Abnormal battery level */
			pr_err("%s: Error abnormal battery voltage=%d\n", __func__, vbatt);
			ret = -EINVAL;
		}
	} else {
		/* PCA9468 is not active state  - standby or shutdown */
		/* Stop charging in timer_work */
		
		/* Read all status register for debugging */
		u8 val[8];
		u8 test_val[16];	/* Dump for test register */
		ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
		pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
			__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
		/* Read test register for debugging */
		ret = regmap_bulk_read(pca9468->regmap, 0x40, test_val, 16);
		pr_err("%s: Error reg[0x40]=0x%x,[0x41]=0x%x,[0x42]=0x%x,[0x43]=0x%x,[0x44]=0x%x,[0x45]=0x%x,[0x46]=0x%x,[0x47]=0x%x\n",
			__func__, test_val[0], test_val[1], test_val[2], test_val[3], test_val[4], test_val[5], test_val[6], test_val[7]);
		pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4A]=0x%x,[0x4B]=0x%x,[0x4C]=0x%x,[0x4D]=0x%x,[0x4E]=0x%x,[0x4F]=0x%x\n",
			__func__, test_val[8], test_val[9], test_val[10], test_val[11], test_val[12], test_val[13], test_val[14], test_val[15]);

		/* Check INT1_STS first */
		if ((val[PCA9468_REG_INT1_STS] & PCA9468_BIT_V_OK_STS) != PCA9468_BIT_V_OK_STS) {
			/* VBUS is invalid */
			pr_err("%s: VOK is invalid", __func__);
			/* Check STS_A */
			if (val[PCA9468_REG_STS_A] & PCA9468_BIT_CFLY_SHORT_STS)
				pr_err("%s: Flying Cap is shorted to GND", __func__);	/* Flying cap is short to GND */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VOUT_UV_STS)
				pr_err("%s: VOUT UV", __func__);	/* VOUT < VOUT_OK */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VBAT_OV_STS)
				pr_err("%s: VBAT OV", __func__);	/* VBAT > VBAT_OV */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VIN_OV_STS)
				pr_err("%s: VIN OV", __func__);		/* VIN > V_OV_FIXED or V_OV_TRACKING */
			else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_VIN_UV_STS)
				pr_err("%s: VIN UV", __func__);		/* VIN < V_UVTH */
			else
				pr_err("%s: Invalid VIN or VOUT", __func__);

			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_NTC_TEMP_STS) {
			/* NTC protection */
			int ntc_adc, ntc_th;
			/* NTC threshold */
			u8 reg_data[2];
			regmap_bulk_read(pca9468->regmap, PCA9468_REG_NTC_TH_1, reg_data, 2);
			ntc_th = ((reg_data[1] & PCA9468_BIT_NTC_THRESHOLD9_8)<<8) | reg_data[0];	/* uV unit */
			/* Read NTC ADC */
			ntc_adc = pca9468_read_adc(pca9468, ADCCH_NTC);	/* uV unit */
			pr_err("%s: NTC Protection, NTC_TH=%d(uV), NTC_ADC=%d(uV)", __func__, ntc_th, ntc_adc);
			
			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_CTRL_LIMIT_STS) {
			/* OCP event happens */
			/* Check STS_B */
			if (val[PCA9468_REG_STS_B] & PCA9468_BIT_OCP_FAST_STS)
				pr_err("%s: IIN is over OCP_FAST", __func__); 	/* OCP_FAST happened */
			else if (val[PCA9468_REG_STS_B] & PCA9468_BIT_OCP_AVG_STS)
				pr_err("%s: IIN is over OCP_AVG", __func__);	/* OCP_AVG happened */
			else
				pr_err("%s: No Loop active", __func__);

			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_TEMP_REG_STS) {
			/* Over temperature protection */
			pr_err("%s: Device is in temperature regulation", __func__);
			
			ret = -EINVAL;
		} else if (val[PCA9468_REG_INT1_STS] & PCA9468_BIT_TIMER_STS) {
			/* Check STS_B */
			if (val[PCA9468_REG_STS_B] & PCA9468_BIT_CHARGE_TIMER_STS)
				pr_err("%s: Charger timer is expired", __func__);
			else if (val[PCA9468_REG_STS_B] & PCA9468_BIT_WATCHDOG_TIMER_STS)
				pr_err("%s: Watchdog timer is expired", __func__);
			else
				pr_err("%s: Timer INT, but no timer STS", __func__);

			ret = -EINVAL;
		}
		else if (val[PCA9468_REG_STS_A] & PCA9468_BIT_CFLY_SHORT_STS) {
			/* Flying cap short */
			pr_err("%s: Flying Cap is shorted to GND", __func__);	/* Flying cap is short to GND */
			ret = -EINVAL;
		} else {
			/* There is no error, but not active state */
			if (reg_val & PCA9468_BIT_STANDBY_STATE_STS) {
				/* Standby state */
				/* Check the RCP condition, T_REVI_DET is 300ms */
				/* Wait 200ms */
				msleep(200);

				/* Check the charging state */
				/* Sometimes battery driver might call set_property function to stop charging during msleep
				    At this case, charging state would change DC_STATE_NO_CHARGING.
				    PCA9468 should stop checking RCP condition and exit timer_work
				*/
				if (pca9468->charging_state == DC_STATE_NO_CHARGING) {
					pr_err("%s: other driver forced to stop direct charging\n", __func__);
					ret = -EINVAL;
				} else {
					/* Keep the current charging state */
					/* Check PCA948 state again */
					/* Read STS_B */
					ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_B, &reg_val);
					if (ret < 0)
						goto error;

					pr_info("%s:RCP check, STS_B=0x%x\n",	__func__, reg_val);

					/* Check Active status */
					if (reg_val & PCA9468_BIT_ACTIVE_STATE_STS) {
						/* RCP condition happened, but VIN is still valid */
						/* If VIN is increased, input current will increase over IIN_LOW level */
						/* Normal charging */
						pr_info("%s: RCP happened before, but VIN is valid\n", __func__);
						ret = 0;
					} else if (reg_val & PCA9468_BIT_STANDBY_STATE_STS) {
						/* It is not RCP condition */
						/* Need to retry if DC is in starting state */
						pr_err("%s: Any abnormal condition, retry\n", __func__);
						/* Dump register */
						ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
						pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
								__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
						ret = regmap_bulk_read(pca9468->regmap, 0x48, test_val, 3);
						pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4a]=0x%x\n",
								__func__, test_val[0], test_val[1], test_val[2]);
						ret = -EAGAIN;
					} else if (reg_val & PCA9468_BIT_SHUTDOWN_STATE_STS) {
						/* Shutdown State */
						pr_err("%s: Shutdown state\n", __func__);
						/* Dump register */
						ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
						pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
								__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
						ret = regmap_bulk_read(pca9468->regmap, 0x48, test_val, 3);
						pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4a]=0x%x\n",
								__func__, test_val[0], test_val[1], test_val[2]);
						ret = -EINVAL;
					} else {
						/* Power State Error - Retry */
						pr_err("%s: No Power state\n", __func__);
						/* Dump register */
						ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
						pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
								__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
						ret = regmap_bulk_read(pca9468->regmap, 0x48, test_val, 3);
						pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4a]=0x%x\n",
								__func__, test_val[0], test_val[1], test_val[2]);
						ret = -EAGAIN;
					}
				}
			} else if (reg_val & PCA9468_BIT_SHUTDOWN_STATE_STS) {
				/* PCA9468 is in shutdown state */
				pr_err("%s: PCA9468 is in shutdown state\n", __func__);
				ret = -EINVAL;
			} else {
				/* Power State Error - Retry */
				pr_err("%s: No Power state\n", __func__);
				/* Dump register */
				ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, &val[PCA9468_REG_INT1], 7);
				pr_err("%s: Error reg[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,[5]=0x%x,[6]=0x%x,[7]=0x%x\n",
						__func__, val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
				ret = regmap_bulk_read(pca9468->regmap, 0x48, test_val, 3);
				pr_err("%s: Error reg[0x48]=0x%x,[0x49]=0x%x,[0x4a]=0x%x\n",
						__func__, test_val[0], test_val[1], test_val[2]);
				ret = -EAGAIN;
			}
		}
	}

error:
	pr_info("%s: Active Status=%d\n", __func__, ret);
	return ret;
}


/* Check CC Mode status */
static int pca9468_check_ccmode_status(struct pca9468_charger *pca9468)
{
	unsigned int reg_val;
	int ret, vbat;
	
	/* Read STS_A */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &reg_val);
	if (ret < 0)
		goto error;

	/* Read VBAT ADC */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);

	/* Check STS_A */
	if (reg_val & PCA9468_BIT_VIN_UV_STS) {
		ret = CCMODE_VIN_UVLO;
	} else if (reg_val & PCA9468_BIT_CHG_LOOP_STS) {
		ret = CCMODE_CHG_LOOP;
	} else if (reg_val & PCA9468_BIT_VFLT_LOOP_STS) {
		ret = CCMODE_VFLT_LOOP;
	} else if (reg_val & PCA9468_BIT_IIN_LOOP_STS) {
		ret = CCMODE_IIN_LOOP;
	} else {
		if (vbat < PCA9468_CC2_VBAT_MIN)
			ret = CCMODE_LOOP_INACTIVE1;
		else if (vbat < PCA9468_CC3_VBAT_MIN)
			ret = CCMODE_LOOP_INACTIVE2;
		else
			ret = CCMODE_LOOP_INACTIVE3;
	}

error:
	pr_info("%s: CCMODE Status=%d\n", __func__, ret);
	return ret;
}


/* Check CVMode Status */
static int pca9468_check_cvmode_status(struct pca9468_charger *pca9468)
{
	unsigned int val, iin, ichg;
	int ret;
	union power_supply_propval value;

	if (pca9468->charging_state == DC_STATE_START_CV) {
		/* Read STS_A */
		ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &val);
		if (ret < 0)
			goto error;
		/* Check STS_A */
		if (val & PCA9468_BIT_CHG_LOOP_STS)	{
			ret = CVMODE_CHG_LOOP;
		} else if (val & PCA9468_BIT_VFLT_LOOP_STS) {
			ret = CVMODE_VFLT_LOOP;
		} else if (val & PCA9468_BIT_IIN_LOOP_STS) {
			ret = CVMODE_IIN_LOOP;
		} else if (val & PCA9468_BIT_VIN_UV_STS) {
			ret = CVMODE_VIN_UVLO;
		} else {
			/* Any LOOP is inactive */
			ret = CVMODE_LOOP_INACTIVE;
		}
	} else {
		/* Read IIN ADC */
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		if (iin < 0) {
			ret = iin;
			goto error;
		}
		
		/* Check IIN < Input Topoff current */
		if (iin < pca9468->pdata->iin_topoff) {
			pr_info("%s: iin=%d, iin_topoff=%d\n", __func__, iin, pca9468->pdata->iin_topoff);
			
			/* Check IBAT current from Fuel Gauge */
			ret = power_supply_get_property(pca9468->bat_psy, POWER_SUPPLY_PROP_CURRENT_NOW, &value);
			if (ret < 0) {
				pr_err("%s, Unable to read battery current: %d\n", __func__, ret);
				goto error;
			}
			ichg = abs(value.intval);
			pr_info("%s: ichg =%d, ichg_topoff=%d\n", __func__, ichg, pca9468->pdata->ichg_topoff);	
			if (ichg < pca9468->pdata->ichg_topoff) {
				/* Direct Charging Done */
				ret = CVMODE_CHG_DONE;
				goto error;
			}
		}

		/* It doesn't reach top-off condition yet */
				
		/* Read STS_A */
		ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_A, &val);
		if (ret < 0)
			goto error;
		/* Check STS_A */
		if (val & PCA9468_BIT_CHG_LOOP_STS) {
			ret = CVMODE_CHG_LOOP;
		} else if (val & PCA9468_BIT_VFLT_LOOP_STS) {
			ret = CVMODE_VFLT_LOOP;
		} else if (val & PCA9468_BIT_IIN_LOOP_STS) {
			ret = CVMODE_IIN_LOOP;
		} else if (val & PCA9468_BIT_VIN_UV_STS) {
			ret = CVMODE_VIN_UVLO;
		} else {
			/* Any LOOP is inactive */
			ret = CVMODE_LOOP_INACTIVE;
		}
	}
	
error:
	pr_info("%s: CVMODE Status=%d\n", __func__, ret);
	return ret;
}

/* Check NTC state */
static int pca9468_check_ntc_stage(struct pca9468_charger *pca9468)
{
	int ret;
	int ntc_value;

	/* Get the current stage */
	ntc_value = pca9468_read_ntc_value();
	if (ntc_value < 0) {
		pr_err("%s, Unable to read ntc value=%d\n", __func__, ntc_value);
		
		//workaround, return current state if reading ntc failed
		if (pca9468->ntc_stage == NTC_STAGE_MAX)
			pca9468->ntc_stage = NTC_STAGE1;
		ret = pca9468->ntc_stage;
	} else {
		/* Check the current stage */
		if (pca9468->ntc_stage == NTC_STAGE2) {
			/* Check the min threshold */
			//if (ntc_value < NTC_STAGE2_MIN)
			if (ntc_value < g_inov_temp_min)
				ret = NTC_STAGE1;
			else
				ret = NTC_STAGE2;
		} else {
			/* Assume the default stage is NTC_STAGE1 
			    Check the max threshold */
			//if (ntc_value > NTC_STAGE1_MAX)
			if (ntc_value > g_inov_temp_max)
				ret = NTC_STAGE2;
			else
				ret = NTC_STAGE1;
		}
	}
	
	pr_info("%s, new_ntc_state=%d\n", __func__, ret);
	return ret;
}

/* Check JEITA stage */
static int pca9468_check_jeita_stage(struct pca9468_charger *pca9468)
{
	int ret;
	struct power_supply *battery;
	union power_supply_propval val;
	int temp, new_stage;

	/* Get power supply name */
	battery = power_supply_get_by_name("battery");
	if (battery == NULL) {
		ret = -EINVAL;
		goto error;
	}

	ret = power_supply_get_property(battery,
				POWER_SUPPLY_PROP_TEMP, &val);
	if (ret < 0) {
		pr_err("%s, Unable to read battery temperature: %d\n", __func__, ret);
		goto error;
	}

	temp = val.intval;

	/* Check the previous jeita stage */
	switch (pca9468->jeita_stage) {
	case JEITA_STAGE1:
		/* Check the rising threshold */
		if (temp >= JEITA_STAGE1_MAX)
			new_stage = JEITA_STAGE2;
		else
			new_stage = pca9468->jeita_stage;
		break;
		
	case JEITA_STAGE2:
		/* Check the rising and falling threshold */
		if (temp >= JEITA_STAGE2_MAX)
			new_stage = JEITA_STAGE3;
		else if (temp < JEITA_STAGE2_MIN)
			new_stage = JEITA_STAGE1;
		else
			new_stage = pca9468->jeita_stage;
		break;
		
	case JEITA_STAGE3:
		/* Check the rising and falling threshold */
		if (temp >= JEITA_STAGE3_MAX)
			new_stage = JEITA_STAGE4;
		else if (temp < JEITA_STAGE3_MIN)
			new_stage = JEITA_STAGE2;
		else
			new_stage = pca9468->jeita_stage;
		break;

	case JEITA_STAGE4:
		/* Check the falling threshold */
		if (temp < JEITA_STAGE4_MIN)
			new_stage = JEITA_STAGE3;
		else
			new_stage = pca9468->jeita_stage;
		break;

	case JEITA_STAGE_MAX:
		/* Assume the temperature would increase */
		if (temp < JEITA_STAGE1_MAX)
			new_stage = JEITA_STAGE1;
		else if (temp < JEITA_STAGE2_MAX)
			new_stage = JEITA_STAGE2;
		else if (temp < JEITA_STAGE3_MAX)
			new_stage = JEITA_STAGE3;
		else
			new_stage = JEITA_STAGE4;
		break;
		
	default:
		ret = -EINVAL;
		goto error;
	}

	ret = new_stage;
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* JEITA function */
extern void pca_jeita_stop_pmic_notifier(int stage);
extern void pca_chg_done_pmic_notifier(void);
static int pca9468_charge_jeita_mode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int jeita_stage, vbat;

	pr_info("%s: ======START=======\n", __func__);

	jeita_stage = pca9468_check_jeita_stage(pca9468);
	if (jeita_stage < 0) {
		ret = jeita_stage;
		goto error;
	}

	/* compare the current jeita stage with the previous jeita stage */
	if (jeita_stage == pca9468->jeita_stage) {
		/* The current stage is same as the previous one */
		/* Check the stage 2 and battery level */
		if (jeita_stage == JEITA_STAGE2) {
			/* Check the Vbat */
			vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
			if (vbat < 0) {
				ret = vbat;
				goto error;
			}
			if (vbat >= PCA9468_CC2_VBAT_MIN) {
				/* Check the charging stage */
				if (pca9468->charging_state == DC_STATE_CHECK_CC) {
					/* go to the CC2 mode */
					pca9468->charging_state = DC_STATE_START_CC2;
				}
			}
		}
	} else {
		/* The current stage is different from the previous stage */
		switch (jeita_stage) {
		case JEITA_STAGE1:
		case JEITA_STAGE4:
			/* Save the new state */
			pca9468->jeita_stage = jeita_stage;
			/* Change the charging status */
			pca9468->charging_state = DC_STATE_NO_CHARGING_JEITA;
			/* Disable direct charging */
			ret = pca9468_set_charging(pca9468, false);
			if (ret < 0)
				goto error;
			/* Notifier PMIC PCA JEITA STOP */
			pca_jeita_stop_pmic_notifier(jeita_stage);
			break;

		case JEITA_STAGE2:
			/* Check the previous stage */
			if ((pca9468->jeita_stage == JEITA_STAGE1) ||
				(pca9468->jeita_stage == JEITA_STAGE4)) {
				/* Jeita stage changed from JEITA_STAGE1 or 4 */
				/* ICL of switching charger set to 25mA */
				ret = pca9468_set_switching_charger(true, SWCHG_ICL_MIN, SWCHG_ICL_MIN, pca9468->pdata->v_float);
				if (ret < 0)
					goto error;
				/* Notifier PMIC PCA JEITA SWITCH TO STAR */
				pca_jeita_stop_pmic_notifier(jeita_stage);
			}
			/* Save the current stage */
			pca9468->jeita_stage = jeita_stage;
			/* Start direct charging */
			/* Wait 1sec for stopping switching charger */
			mutex_lock(&pca9468->lock);
			pca9468->charging_state = DC_STATE_CHECK_VBAT;
			pca9468->timer_id = TIMER_VBATMIN_CHECK;
			pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			break;
			
		case JEITA_STAGE3:
			/* Check the previous stage */
			if ((pca9468->jeita_stage == JEITA_STAGE1) ||
				(pca9468->jeita_stage == JEITA_STAGE4)) {
				/* Jeita stage changed from JEITA_STAGE1 or 4 */
				/* ICL of switching charger set to 25mA */
				ret = pca9468_set_switching_charger(true, SWCHG_ICL_MIN, SWCHG_ICL_MIN, pca9468->pdata->v_float);
				if (ret < 0)
					goto error;
				/* Notifier PMIC PCA JEITA SWITCH TO STAR */
				pca_jeita_stop_pmic_notifier(jeita_stage);
				/* Save the current stage */
				pca9468->jeita_stage = jeita_stage;

				/* Start direct charging */
				/* Wait 1sec for stopping switching charger */
				mutex_lock(&pca9468->lock);
				pca9468->charging_state = DC_STATE_CHECK_VBAT;
				pca9468->timer_id = TIMER_VBATMIN_CHECK;
				pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
				mutex_unlock(&pca9468->lock);
				queue_delayed_work(pca9468->dc_wq,
									&pca9468->timer_work,
									msecs_to_jiffies(pca9468->timer_period));
			} else {
				/* The previous stage is JEITA_STAGE2 */
				/* Save the current stage */
				pca9468->jeita_stage = jeita_stage;
				/* Check the charging state */
				if (pca9468->charging_state == DC_STATE_CHECK_CC) {
					/* The current charging state is CC1 state */
					/* restart in DC_STATE_PRESET_DC state */
					
					/* Start direct charging */
					/* Wait 1sec for stopping switching charger */
					mutex_lock(&pca9468->lock);
					pca9468->charging_state = DC_STATE_CHECK_VBAT;
					pca9468->timer_id = TIMER_VBATMIN_CHECK;
					pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
					mutex_unlock(&pca9468->lock);
					queue_delayed_work(pca9468->dc_wq,
									&pca9468->timer_work,
									msecs_to_jiffies(pca9468->timer_period));
				} else {
					/* the current charging stage is DC_STATE_CHECK_CC2 or DC_STATE_CHECK_CV */
					/* battery voltage is over 4.25V,but the vfloat of stage3 is 4.1V */
					/* So, we should go to Pre-CV mode */
					unsigned int val;
					
					/* Change charging state */
					pca9468->charging_state = DC_STATE_CHANGE_CV_JEITA;
					/* Set IIN_CFG */
					pca9468->pdata->iin_cfg = JEITA_STAGE3_IIN;
					/* Set VFLOAT */
					pca9468->pdata->v_float = JEITA_STAGE3_VFLOAT;					
					/* Disable charging */
					ret = pca9468_set_charging(pca9468, false);
					if (ret < 0)
						goto error;
					/* Set TA current to IIN_CFG */
					pca9468->ta_cur = pca9468->pdata->iin_cfg;
					val = pca9468->ta_cur/PD_MSG_TA_CUR_STEP;	/* PPS current resolution is 50mV */
					pca9468->ta_cur = val*PD_MSG_TA_CUR_STEP;
					pca9468->ta_objpos = 0;	/* Search the proper object position of PDO */
					/* Set TA max current */
					pca9468->ta_max_cur = pca9468->pdata->iin_cfg;
					/* Send PD Message */
					ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
					if (ret < 0)
						goto error;
					pr_info("%s: Preset DC, ta_vol=%d, ta_cur=%d\n", 
						__func__, pca9468->ta_vol, pca9468->ta_cur);

					mutex_lock(&pca9468->lock);
					pca9468->timer_id = TIMER_PDMSG_SEND;
					pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
					mutex_unlock(&pca9468->lock);
					queue_delayed_work(pca9468->dc_wq,
									&pca9468->timer_work,
									msecs_to_jiffies(pca9468->timer_period));
				}
			}
			break;

		default:
			ret = -EINVAL;
			goto error;
			break;
		}
	
	}

error:
	pr_info("%s: End, charging_stage=%d, ret=%d\n", __func__,pca9468->charging_state, ret);
	return ret;
}
/* Enter JEITA check mode */
static int pca9468_charge_start_jeita(struct pca9468_charger *pca9468)
{
	int ret;

	/* The current jeita stage is stage1 or 4*/
	pr_info("%s: ======START=======\n", __func__);


	/* Set TA voltage to fixed 5V */
	pca9468->ta_vol = 5000000;
	/* Set TA current to maximum 3A */
	pca9468->ta_cur = 3000000;
	
	/* Send PD Message */
	pca9468->ta_objpos = 1;	// PDO1 - fixed 5V
	ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_FIXED_PDO);
	if (ret < 0)
		return ret;
	
	mutex_lock(&pca9468->lock);
	pca9468->timer_id = TIMER_PDMSG_SEND;
	pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
	mutex_unlock(&pca9468->lock);
	queue_delayed_work(pca9468->dc_wq,
						&pca9468->timer_work,
						msecs_to_jiffies(pca9468->timer_period));
	
	return ret;
}


/* Check the JEITA state */
static int pca9468_charge_check_jeita(struct pca9468_charger *pca9468)
{
	/* The current jeita stage is stage1 or 4 */
	/* charging state is DC_STATE_NO_CHARGING_JEITA */
	int ret;
	
	pr_info("%s: ======START=======\n", __func__);

	ret = pca9468_charge_jeita_mode(pca9468);
	if (ret < 0)
		goto error;

	pr_info("%s: charging_state=%d\n", __func__, pca9468->charging_state);

	if (pca9468->charging_state == DC_STATE_NO_CHARGING_JEITA) {
		/* Set timer */
		mutex_lock(&pca9468->lock);
		pca9468->timer_period = PCA9468_JEITA_CHECK_T;
		pca9468->timer_id = TIMER_JEITA_CHECK;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
	}
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
	
}

/* 2:1 Direct Charging Adjust CC MODE control */
static int pca9468_charge_adjust_ccmode(struct pca9468_charger *pca9468)
{
	int iin, ccmode;
	int vbatt;
	int ret = 0;

	pr_info("%s: ======START=======\n", __func__);

	if (smartchg_slow_flag){
		if (pca9468->ta_cur > SLOWE_CHARGING_TA_CUR)
			pca9468->ta_cur = SLOWE_CHARGING_TA_CUR;
	}

	pca9468->charging_state = DC_STATE_ADJUST_CC;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error;	// This is not active mode.

	ccmode = pca9468_check_ccmode_status(pca9468);
	if (ccmode < 0) {
		ret = ccmode;
		goto error;
	}
	
	switch(ccmode) {
	case CCMODE_IIN_LOOP:
	case CCMODE_CHG_LOOP:
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
		pr_info("%s: CC adjust End: ta_cur=%d\n", __func__, pca9468->ta_cur);
		/* Read VBAT ADC */
		vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);
		/* TA voltage = TA voltage + (VFLOAT - VBAT_ADC)*2 */
		pca9468->ta_vol = pca9468->ta_vol + (pca9468->pdata->v_float - vbatt)*2;
		if (pca9468->ta_vol > PCA9468_TA_MAX_VOL)
			pca9468->ta_vol = PCA9468_TA_MAX_VOL;
		pr_info("%s: CC adjust End: ta_vol=%d\n", __func__, pca9468->ta_vol);
		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
		mutex_lock(&pca9468->lock);
		/* End TA voltage and current adjustment */
		/* Save the cc initial current */
		pca9468->cc_ta_cur = pca9468->ta_cur;
		/* go to CC mode */
		pca9468->charging_state = DC_STATE_START_CC;
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;
		
	case CCMODE_VFLT_LOOP:
		/* go to CV mode */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CCMODE_LOOP_INACTIVE1:
	case CCMODE_LOOP_INACTIVE2:
	case CCMODE_LOOP_INACTIVE3:
		/* Check IIN ADC with IIN_CFG */
		iin = pca9468_read_adc(pca9468, ADCCH_IIN);
		/* IIN_ADC > IIN_CFG -40mA ? */
		if (iin > (int)(pca9468->pdata->iin_cfg - 40000)) {
			/* Input current is already over IIN_CFG */
			/* End TA voltage and current adjustment */
			/* Save the cc initial current */
			pca9468->cc_ta_cur = pca9468->ta_cur;
			/* go to CC mode */
			pca9468->charging_state = DC_STATE_START_CC;

			/* Read VBAT ADC */
			vbatt = pca9468_read_adc(pca9468, ADCCH_VBAT);
			/* TA voltage = TA voltage + (VFLOAT - VBAT_ADC)*2 */
			pca9468->ta_vol = pca9468->ta_vol + (pca9468->pdata->v_float - vbatt)*2;
			if (pca9468->ta_vol > PCA9468_TA_MAX_VOL)
				pca9468->ta_vol = PCA9468_TA_MAX_VOL;
			pr_info("%s: CC adjust End: IIN_ADC=%d, ta_vol=%d\n", __func__, iin, pca9468->ta_vol);	
		} else {
			/* IIN_ADC > pre_IIN_ADC + 20mA ? */
			if (iin > (int)(pca9468->pre_iin_adc + 20000)) {
				/* TA voltage too low to enter TA CC mode, so we should increae TA voltage */
				pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_ADJ_CC;
				if (pca9468->ta_vol > PCA9468_TA_MAX_VOL)
					pca9468->ta_vol = PCA9468_TA_MAX_VOL;
				pr_info("%s: CC adjust Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);
			} else {
				/* Check TA voltage */
				if (pca9468->ta_vol < PCA9468_TA_MIN_VOL_CCADJ) {
					/* TA voltage is too low, so increase TA voltage first */
					pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_ADJ_CC;
					if (pca9468->ta_vol > PCA9468_TA_MAX_VOL)
						pca9468->ta_vol = PCA9468_TA_MAX_VOL;
					pr_info("%s: CC adjust Cont: IIN_ADC=%d, ta_vol=%d\n", __func__, iin, pca9468->ta_vol);
				} else {				
					/* Check Max TA current */
					if (pca9468->ta_cur == pca9468->ta_max_cur) {
						/* TA current is already max value */
						/* Check TA voltage */
						if (pca9468->ta_vol == PCA9468_TA_MAX_VOL) {
							/* TA voltage is already max value */
							/* Change charging status */
							pca9468->charging_state = DC_STATE_START_CC;
							/* Save the cc initial current */
							pca9468->cc_ta_cur = pca9468->ta_cur;
							pr_info("%s: CC adjust End: MAX value, ta_vol=%d, ta_cur=%d\n", 
								__func__, pca9468->ta_vol, pca9468->ta_cur);
						} else {
							/* TA voltage too low to enter TA CC mode, so we should increae TA voltage */
							pca9468->ta_vol = pca9468->ta_vol + PCA9468_TA_VOL_STEP_ADJ_CC;
							if (pca9468->ta_vol > PCA9468_TA_MAX_VOL)
								pca9468->ta_vol = PCA9468_TA_MAX_VOL;
							pr_info("%s: CC adjust Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);
						}
					} else {
						/* TA current is low to enter IIN_LOOP, so w should increase TA current (50mA) */
						pca9468->ta_cur = pca9468->ta_cur + PD_MSG_TA_CUR_STEP;
						if (smartchg_slow_flag){
							if (pca9468->ta_cur > SLOWE_CHARGING_TA_CUR)
								pca9468->ta_cur = SLOWE_CHARGING_TA_CUR;
						}else{
							if (pca9468->ta_cur > pca9468->ta_max_cur)
								pca9468->ta_cur = pca9468->ta_max_cur;
						}
						pr_info("%s: CC adjust Cont: ta_cur=%d\n", __func__, pca9468->ta_cur);
					}
				}
			}					
		}

		/* Save the current iin adc  */
		pca9468->pre_iin_adc = iin;

		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_ADJCC_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CCMODE_VIN_UVLO:
		/* Check again */
		/* Set timer */
		mutex_lock(&pca9468->lock);
		pca9468->timer_period = 1000;	// 1000ms
		pca9468->timer_id = TIMER_ENTER_ADJ_CCMODE;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	default:
		break;
	}
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

extern int thermal_virtual_therm_get(void);
/* 2:1 Direct Charging CC MODE control */
static int pca9468_charge_ccmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int ccmode;
	int virtual_therm = 0;
	static bool virtual_therm_alert = false;
	
	pr_info("[PCA] %s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_CHECK_CC;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error;	// This is not active mode.
	
	ccmode = pca9468_check_ccmode_status(pca9468);
	if (ccmode < 0) {
		ret = ccmode;
		goto error;
	}
	
	virtual_therm = thermal_virtual_therm_get();
	printk("virtual_therm = %d\n", virtual_therm);
	
	switch(ccmode) {
	case CCMODE_LOOP_INACTIVE1:
	case CCMODE_LOOP_INACTIVE2:
	case CCMODE_LOOP_INACTIVE3:
		/* First, check JEITA function */
		/* jeita function */
		ret = pca9468_charge_jeita_mode(pca9468);
		if (ret < 0)
			goto error;
		/* Check the charging state */
		if (pca9468->charging_state == DC_STATE_CHECK_CC) {
			/* JEITA stage is  stage2 or stage3 */
			/* Check JEITA stage */
			if (pca9468->jeita_stage == JEITA_STAGE2) {
				/* Second, check NTC function */
				ret = pca9468_check_ntc_stage(pca9468);
				if (ret < 0)
					goto error;
				pca9468->ntc_stage = ret;
				/* Check NTC stage */
				if (pca9468->ntc_stage == NTC_STAGE1) {
					pr_info("[PCA] %s: NTC_STAGE1: chargeing_state= %d, LCDOnOff= %d, iin= %d, ta_cur= %d, cc_ta_cur= %d\n", 
						__func__, pca9468->charging_state, g_PanelOnOff, pca9468->pdata->iin_cfg, pca9468->ta_cur, pca9468->cc_ta_cur);

					/* NTC stage 1 */
					/* Third, check panel status */
					/* Panel on/off compensation */
					if (g_PanelOnOff && (ASUS_POGO_ID == INBOX) && (pca9468->pdata->iin_cfg >= g_panel_off_iin)) {
						/* panel on but there is INBOX to cool down, TA current back to TA CC current */
						if (pca9468->ta_cur != pca9468->cc_ta_cur) {
							/* Update TA current to TA CC current */
							pca9468->ta_cur = pca9468->cc_ta_cur;
							pr_info("[PCA] %s: Thermal trigger-2: new ta_cur=%d\n", __func__, pca9468->ta_cur);
							
							/* Send PD Message */
							ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
							if (ret < 0)
								goto error;
	
							pr_info("[PCA] %s: Thermal trigger-1, panel OFF + INBOX(4A), iin_cfg:%d(uA), iin_adc:%d(uA), ta_cur:%d(mA)\n", 
									__func__, pca9468->pdata->iin_cfg, pca9468_read_adc(pca9468, ADCCH_IIN), pca9468->ta_cur);
						}
					}
					//if (g_PanelOnOff && (pca9468->pdata->iin_cfg > PCA9468_CC1_IIN_CFG_3A)) {
					else if ((g_PanelOnOff && (pca9468->pdata->iin_cfg > g_panel_on_iin)) || (virtual_therm > 42000 || virtual_therm_alert)) {
						virtual_therm_alert = true;
						
						/* panel on, TA current  to PCA9468_CC1_IIN_CFG_3A(1400mA) */
						//if (pca9468->ta_cur != PCA9468_CC1_IIN_CFG_3A) {
						if (pca9468->ta_cur != g_panel_on_iin) {
							/* Update TA current to PCA9468_CC1_IIN_CFG_3A */
							//pca9468->ta_cur = PCA9468_CC1_IIN_CFG_3A;
							pca9468->ta_cur = g_panel_on_iin;

							/* Send PD Message */
							ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
							if (ret < 0)
								goto error;
	
							pr_info("[PCA] %s: Thermal trigger-1, panel ON (3A), iin_cfg:%d(uA), iin_adc:%d(uA), ta_cur:%d(mA)\n", 
									__func__, pca9468->pdata->iin_cfg, pca9468_read_adc(pca9468, ADCCH_IIN), pca9468->ta_cur);
	
						}
					//} else if (!g_PanelOnOff && (pca9468->pdata->iin_cfg >= PCA9468_CC1_IIN_CFG_4A)) {
					} else if (!g_PanelOnOff && (pca9468->pdata->iin_cfg >= g_panel_off_iin)) {
						/* panel off, TA current back to TA CC current */
						if (pca9468->ta_cur != pca9468->cc_ta_cur) {
							/* Update TA current to TA CC current */
							pca9468->ta_cur = pca9468->cc_ta_cur;
							pr_info("[PCA] %s: Thermal trigger-2: new ta_cur=%d\n", __func__, pca9468->ta_cur);
							
							/* Send PD Message */
							ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
							if (ret < 0)
								goto error;
	
							pr_info("[PCA] %s: Thermal trigger-1, panel OFF (4A), iin_cfg:%d(uA), iin_adc:%d(uA), ta_cur:%d(mA)\n", 
									__func__, pca9468->pdata->iin_cfg, pca9468_read_adc(pca9468, ADCCH_IIN), pca9468->ta_cur);
						}
					}
					
					if (virtual_therm < 40000) {
						virtual_therm_alert = false;
					}
				} else {
					/* NTC stage 2 */
					/* Check TA current */
					if (g_PanelOnOff && (ASUS_POGO_ID == INBOX) && (pca9468->ta_cur > g_inov_overtemp_iin)) {
						pca9468->ta_cur = g_inov_overtemp_iin;
						/* Send PD Message */
						ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
							if (ret < 0)
								goto error;
	
						pr_info("[PCA] %s: NTC stage2(screen on + INBOX), ta_cur:%d(mA)\n", __func__, pca9468->ta_cur);
					} else if ((g_PanelOnOff && (pca9468->ta_cur > g_inov_overtemp_iin_low)) || (virtual_therm > 42000 || virtual_therm_alert)) {
						virtual_therm_alert = true;
						
						pca9468->ta_cur = g_inov_overtemp_iin_low;
						/* Send PD Message */
						ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
							if (ret < 0)
								goto error;
	
						pr_info("[PCA] %s: NTC stage2(screen on), ta_cur:%d(mA)\n", __func__, pca9468->ta_cur);
					} else if (!g_PanelOnOff && (pca9468->ta_cur >= g_inov_overtemp_iin)) {
					//if (pca9468->ta_cur > NTC_STAGE2_IIN) {
					//if (pca9468->ta_cur > g_inov_overtemp_iin) {
						/* Set TA current to NTC_STAGE2_IIN */
						//pca9468->ta_cur = NTC_STAGE2_IIN;
						pca9468->ta_cur = g_inov_overtemp_iin;
						/* Send PD Message */
						ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
							if (ret < 0)
								goto error;
	
						pr_info("[PCA] %s: NTC stage2(screen off), ta_cur:%d(mA)\n", __func__, pca9468->ta_cur);
					}
					
					if (virtual_therm < 40000) {
						virtual_therm_alert = false;
					}
				}
			}

			/* Set timer */
			mutex_lock(&pca9468->lock);
			pca9468->timer_period = PCA9468_CCMODE_CHECK1_T;
			pca9468->timer_id = TIMER_CCMODE_CHECK;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));			
		} else if (pca9468->charging_state == DC_STATE_START_CC2) {
			/* go to CC2 mode */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_ENTER_CCMODE2;
			pca9468->timer_period = 0;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
		} else if (pca9468->charging_state == DC_STATE_NO_CHARGING_JEITA) {
			/* Set timer */
			mutex_lock(&pca9468->lock);
			pca9468->timer_period = 0;
			pca9468->timer_id = TIMER_ENTER_JEITA;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
		} else {
			/* wait for new state polling */
		}
		break;

	case CCMODE_VFLT_LOOP:
		/* go to CV mode */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CCMODE_IIN_LOOP:
	case CCMODE_CHG_LOOP:
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
		/* Save cc current */
		pca9468->cc_ta_cur = pca9468->ta_cur;
		/* for debugging */
		ret = pca9468_read_adc(pca9468, ADCCH_IIN);
		pr_info("%s: CC LOOP: iin_adc=%d, ta_cur=%d\n", __func__, ret, pca9468->ta_cur);
		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;			

	case CCMODE_VIN_UVLO:
		/* Check again */
		/* Set timer */
		mutex_lock(&pca9468->lock);
		pca9468->timer_period = 1000;	// 1000ms
		pca9468->timer_id = TIMER_CCMODE_CHECK;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	default:
		break;
	}

	ret = 0;
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* 2:1 Direct Charging Start CC MODE2 control - CC MODE2 */
static int pca9468_charge_start_ccmode2(struct pca9468_charger *pca9468)
{
	int ret = 0;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_START_CC2;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error;	// This is not active mode.
	
	/* Check the JEITA stage */
	if (pca9468->jeita_stage == JEITA_STAGE2) {
		/* Check the TA current > 900mA */
		//if (pca9468->ta_cur > PCA9468_CC2_IIN_CFG) {
		if (pca9468->ta_cur > g_high_volt_4P25_iin) {
			/* Decrease TA current 50mA */
			pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
			/* Save cc current */
			pca9468->cc_ta_cur = pca9468->ta_cur;
			/* Send PD Message */
			ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
			if (ret < 0)
				goto error;
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_PDMSG_SEND;
			pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
		} else {
			/* go to Check CC2 mode */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_CCMODE2_CHECK;
			pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
		}
	} else {
		/* go to Check CC2 mode */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_CCMODE2_CHECK;
		pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* 2:1 Direct Charging CC MODE2 control */
static int pca9468_charge_ccmode2(struct pca9468_charger *pca9468)
{
	int ccmode;
	int ret = 0;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_CHECK_CC2;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error;	// This is not active mode.
	
	ccmode = pca9468_check_ccmode_status(pca9468);
	if (ccmode < 0) {
		ret = ccmode;
		goto error;
	}
	
	switch(ccmode) {
	case CCMODE_LOOP_INACTIVE1:
	case CCMODE_LOOP_INACTIVE2:
	case CCMODE_LOOP_INACTIVE3:
		/* First, check JEITA function */
		/* jeita function */
		ret = pca9468_charge_jeita_mode(pca9468);
		if (ret < 0)
			goto error;
		/* Check the charging state */
		if (pca9468->charging_state == DC_STATE_CHECK_CC2) {
			/* JEITA stage is  stage2 */
			/* Second, check NTC function */
			ret = pca9468_check_ntc_stage(pca9468);
			if (ret < 0)
				goto error;
			pca9468->ntc_stage = ret;
			/* Check NTC stage */
			if (pca9468->ntc_stage == NTC_STAGE1) {
				/* NTC stage 1 */
				/* Set timer */
				if (ccmode == CCMODE_LOOP_INACTIVE1) {
					/* Set TA current to CC current */
					pca9468->ta_cur = pca9468->cc_ta_cur;
					pr_info("%s: CC2 Cont1: ta_cur=%d\n", __func__, pca9468->ta_cur);
					/* Send PD Message */
					ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
					if (ret < 0)
						goto error;
					
					/* Set 10s timer */
					/* go to Check CC2 mode */
					mutex_lock(&pca9468->lock);
					pca9468->timer_id = TIMER_CCMODE2_CHECK;
					pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
					mutex_unlock(&pca9468->lock);
					queue_delayed_work(pca9468->dc_wq,
										&pca9468->timer_work,
										msecs_to_jiffies(pca9468->timer_period));
				} else {
					/* Check CC current > 900mA */
					//if (pca9468->cc_ta_cur > PCA9468_CC2_IIN_CFG) {
					if (pca9468->cc_ta_cur > g_high_volt_4P25_iin) {
						/* Decrease TA current 50mA */
						pca9468->cc_ta_cur = pca9468->cc_ta_cur - PD_MSG_TA_CUR_STEP;
						/* Set TA current to CC current */
						pca9468->ta_cur = pca9468->cc_ta_cur;
						pr_info("%s: CC2 Cont2: ta_cur=%d\n", __func__, pca9468->ta_cur);
						/* Send PD Message */
						ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
						if (ret < 0)
							goto error;

						/* Set timer - 10sec */
						mutex_lock(&pca9468->lock);
						pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
						pca9468->timer_id = TIMER_CCMODE2_CHECK;
						mutex_unlock(&pca9468->lock);
						queue_delayed_work(pca9468->dc_wq,
											&pca9468->timer_work,
											msecs_to_jiffies(pca9468->timer_period));
					} 
					else if (pca9468->cc_ta_cur < g_high_volt_4P25_iin) {
						/* Increase TA current 50mA */
						pca9468->cc_ta_cur = g_high_volt_4P25_iin;
						/* Set TA current to CC current */
						pca9468->ta_cur = pca9468->cc_ta_cur;
						pr_info("%s: CC2 Cont2: ta_cur=%d\n", __func__, pca9468->ta_cur);
						/* Send PD Message */
						ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
						if (ret < 0)
							goto error;

						/* Set timer - 10sec */
						mutex_lock(&pca9468->lock);
						pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
						pca9468->timer_id = TIMER_CCMODE2_CHECK;
						mutex_unlock(&pca9468->lock);
						queue_delayed_work(pca9468->dc_wq,
											&pca9468->timer_work,
											msecs_to_jiffies(pca9468->timer_period));
					}
					else {
						/* go to Check CC2 mode */
						mutex_lock(&pca9468->lock);
						pca9468->timer_id = TIMER_CCMODE2_CHECK;
						if (ccmode == CCMODE_LOOP_INACTIVE2) {
							/* Set 10s timer */
							pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
						} else {
							/* Set 5s timer */
							pca9468->timer_period = PCA9468_CCMODE_CHECK3_T;
						}
						mutex_unlock(&pca9468->lock);
						queue_delayed_work(pca9468->dc_wq,
											&pca9468->timer_work,
											msecs_to_jiffies(pca9468->timer_period));
					}
				}			
			} else {
				/* NTC stage 2 */
				/* Check TA current */
				//if (pca9468->ta_cur > NTC_STAGE2_IIN) {
				//if (pca9468->ta_cur > g_inov_overtemp_iin) {
				if (pca9468->ta_cur > g_inov_overtemp_iin_low) {
					/* Set TA current to NTC_STAGE2_IIN */
					//pca9468->ta_cur = NTC_STAGE2_IIN;
					//pca9468->ta_cur = g_inov_overtemp_iin_low;
					//ASUS BSP : Decrease by 50mA each time +++
					/* Decrease TA current 50mA */
					pca9468->cc_ta_cur = pca9468->cc_ta_cur - PD_MSG_TA_CUR_STEP;
					/* Set TA current to CC current */
					pca9468->ta_cur = pca9468->cc_ta_cur;
					//ASUS BSP : Decrease by 50mA each time ---
					pr_info("%s: CC2 NTC: ta_cur=%d\n", __func__, pca9468->ta_cur);

					/* Send PD Message */
					ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
					if (ret < 0)
						goto error;
					mutex_lock(&pca9468->lock);
					pca9468->timer_id = TIMER_PDMSG_SEND;
					//pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
					pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;	// ASUS BSP : check every 10s +++
					mutex_unlock(&pca9468->lock);
					queue_delayed_work(pca9468->dc_wq,
										&pca9468->timer_work,
										msecs_to_jiffies(pca9468->timer_period));
				} else {
					/* go to Check CC2 mode */
					mutex_lock(&pca9468->lock);
					pca9468->timer_id = TIMER_CCMODE2_CHECK;
					if (ccmode == CCMODE_LOOP_INACTIVE2) {
						/* Set 10s timer */
						pca9468->timer_period = PCA9468_CCMODE_CHECK2_T;
					} else {
						/* Set 5s timer */
						pca9468->timer_period = PCA9468_CCMODE_CHECK3_T;
					}
					mutex_unlock(&pca9468->lock);
					queue_delayed_work(pca9468->dc_wq,
										&pca9468->timer_work,
										msecs_to_jiffies(pca9468->timer_period));
				}
			}
		} else if (pca9468->charging_state == DC_STATE_NO_CHARGING_JEITA) {
			/* Current Jeita stage is JEITA_STAGE1 or 4 */
			/* Set timer */
			mutex_lock(&pca9468->lock);
			pca9468->timer_period = 0;
			pca9468->timer_id = TIMER_ENTER_JEITA;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
		}
		break;

	case CCMODE_VFLT_LOOP:
		/* go to CV mode */
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_CVMODE_CHECK;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CCMODE_IIN_LOOP:
	case CCMODE_CHG_LOOP:
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
		/* Save cc current */
		pca9468->cc_ta_cur = pca9468->ta_cur;

		pr_info("%s: CC LOOP: ta_cur=%d\n", __func__, pca9468->ta_cur);
		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;			

	case CCMODE_VIN_UVLO:
		/* Check again */
		/* Set timer */
		mutex_lock(&pca9468->lock);
		pca9468->timer_period = 1000;	// 1000ms
		pca9468->timer_id = TIMER_CCMODE2_CHECK;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	default:
		break;
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* 2:1 Direct Charging Start CV MODE control - Pre CV MODE */
static int pca9468_charge_start_cvmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int cvmode;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_START_CV;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error;	// This is not active mode.
	
	/* Check TA current > CC2_IIN */
	//if (pca9468->ta_cur > PCA9468_CC2_IIN_CFG) {
	if (pca9468->ta_cur > g_high_volt_4P25_iin) {
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;

		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;

		pr_info("%s: PreCV Cont: ta_cur=%d\n", __func__, pca9468->ta_cur);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		return ret;
	}

	cvmode = pca9468_check_cvmode_status(pca9468);
	if (cvmode < 0) {
		ret = cvmode;
		goto error;
	}
	
	switch(cvmode) {
	case CVMODE_CHG_LOOP:
	case CVMODE_IIN_LOOP:
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
		pr_info("%s: PreCV Cont: ta_cur=%d\n", __func__, pca9468->ta_cur);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_VFLT_LOOP:
		/* Decrease TA voltage (20mV) */
		pca9468->ta_vol = pca9468->ta_vol - PCA9468_TA_VOL_STEP_PRE_CV;
		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
		pr_info("%s: PreCV Cont: ta_vol=%d\n", __func__, pca9468->ta_vol);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_LOOP_INACTIVE:
		/* Exit Pre CV mode */
		/* Go to CV mode */
		pr_info("%s: PreCV End: ta_cur=%d\n", __func__, pca9468->ta_cur);
		
		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_CVMODE_CHECK;
		pca9468->timer_period = 0;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_CHG_DONE:
		/* Charging Done */
		/* Disable PCA9468 */
		ret = pca9468_set_charging(pca9468, false);
		if (ret < 0)
			goto error;

		/* Change charging status */
		pca9468->charging_state = DC_STATE_CHARGING_DONE;
		
		/* Set TA voltage to fixed 5V */
		pca9468->ta_vol = 5000000;
		/* Set TA current to maximum 3A */
		pca9468->ta_cur = 3000000;
		
		/* Send PD Message */
		pca9468->ta_objpos = 1; // PDO1 - fixed 5V
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_FIXED_PDO);
		if (ret < 0)
			goto error;

		pr_info("%s: CV Done\n", __func__);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_VIN_UVLO:
		/* Check again */
		/* Set timer */
		mutex_lock(&pca9468->lock);
		pca9468->timer_period = 1000;	// 1000ms
		pca9468->timer_id = TIMER_ENTER_CVMODE;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;
		
	default:
		break;
	}
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}

/* 2:1 Direct Charging CV MODE control */
static int pca9468_charge_cvmode(struct pca9468_charger *pca9468)
{
	int ret = 0;
	int cvmode;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_CHECK_CV;

	ret = pca9468_check_error(pca9468);
	if (ret != 0)
		goto error;	// This is not active mode.
	
	cvmode = pca9468_check_cvmode_status(pca9468);
	if (cvmode < 0) {
		ret = cvmode;
		goto error;
	}

	switch(cvmode) {
	case CVMODE_CHG_DONE:
		/* Charging Done */
		/* Disable PCA9468 */
		ret = pca9468_set_charging(pca9468, false);
		if (ret < 0)
			goto error;

		/* Change charging status */
		pca9468->charging_state = DC_STATE_CHARGING_DONE;
		
		/* Set TA voltage to fixed 5V */
		pca9468->ta_vol = 5000000;
		/* Set TA current to maximum 3A */
		pca9468->ta_cur = 3000000;
		
		/* Send PD Message */
		pca9468->ta_objpos = 1;	// PDO1 - fixed 5V
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_FIXED_PDO);
		if (ret < 0)
			goto error;

		pr_info("%s: CV Done\n", __func__);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_CHG_LOOP:
	case CVMODE_IIN_LOOP:
		/* Decrease TA current (50mA) */
		pca9468->ta_cur = pca9468->ta_cur - PD_MSG_TA_CUR_STEP;
		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
		pr_info("%s: CV LOOP, Cont: ta_cur=%d\n", __func__, pca9468->ta_cur);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_VFLT_LOOP:
		/* Decrease TA voltage */
		pca9468->ta_vol = pca9468->ta_vol - PD_MSG_TA_VOL_STEP;
		/* Set TA current to CC1_IIN to change TA operation mode from CL to CV */
		pca9468->ta_cur = g_panel_off_iin;

		/* Send PD Message */
		ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
		if (ret < 0)
			goto error;
		pr_info("%s: CV VFLOAT, Cont: ta_vol=%d, ta_cur=%d\n", __func__, pca9468->ta_vol, pca9468->ta_cur);

		mutex_lock(&pca9468->lock);
		pca9468->timer_id = TIMER_PDMSG_SEND;
		pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	case CVMODE_LOOP_INACTIVE:
		/* jeita function */
		ret = pca9468_charge_jeita_mode(pca9468);
		if (ret < 0)
			goto error;
		/* Check the charging state */
		if (pca9468->charging_state == DC_STATE_CHECK_CV) {
			/* charging state is kept */
			/* Set timer */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_CVMODE_CHECK;
			pca9468->timer_period = PCA9468_CVMODE_CHECK_T;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		} else if (pca9468->charging_state == DC_STATE_NO_CHARGING_JEITA) {
			/* Current Jeita stage is JEITA_STAGE1 or 4 */
			/* Set timer */
			mutex_lock(&pca9468->lock);
			pca9468->timer_period = 0;
			pca9468->timer_id = TIMER_ENTER_JEITA;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		}
		break;

	case CVMODE_VIN_UVLO:
		/* Check again */
		/* Set timer */
		mutex_lock(&pca9468->lock);
		pca9468->timer_period = 1000; 	// 1000ms
		pca9468->timer_id = TIMER_CVMODE_CHECK;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		break;

	default:
		break;
	}

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Preset TA voltage and current for Direct Charging Mode */
static int pca9468_preset_dcmode(struct pca9468_charger *pca9468)
{
	int vbat;
	unsigned int val;
	int ret = 0;

	pr_info("%s: ======START=======\n", __func__);

	pca9468->charging_state = DC_STATE_PRESET_DC;

	/* Read VBAT ADC */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);

	/* Check the initial jeita stage */
	if (pca9468->jeita_stage == JEITA_STAGE_MAX) {
		pca9468->jeita_stage = pca9468_check_jeita_stage(pca9468);
	}
	
	/* Check the preivous JEITA stage */
	if (pca9468->jeita_stage == JEITA_STAGE3) {
		pca9468->pdata->iin_cfg = JEITA_STAGE3_IIN;
		pca9468->pdata->v_float = JEITA_STAGE3_VFLOAT;
	} else if (pca9468->jeita_stage == JEITA_STAGE2) {
		/* Check the vbat level */
		if (vbat < PCA9468_CC2_VBAT_MIN) {
			//pca9468->pdata->iin_cfg = PCA9468_CC1_IIN_CFG_4A;
			pca9468->pdata->iin_cfg = g_panel_off_iin;
		} else {
			//pca9468->pdata->iin_cfg = PCA9468_CC2_IIN_CFG;
			pca9468->pdata->iin_cfg = g_high_volt_4P25_iin;
		}
		//[+++]Modify for battery safety upgrade
		pca9468->pdata->v_float = g_fv_setting + 20000;
		//[---]Modify for battery safety upgrade
	} else {
		/* Jeita stage is JEITA_STAGE1 or JEITA_STAGE4 */
		/* Change charging state */
		pca9468->charging_state = DC_STATE_NO_CHARGING_JEITA;
		/* stop charging and check the jeita stage change */
		pr_info("%s: jeita stage=%d, polling jeita chek function\n", __func__, pca9468->jeita_stage);
		/* Change 5V/3A and start switching charging and then check the jeita stage */
		mutex_lock(&pca9468->lock);
		pca9468->timer_period = 0;
		pca9468->timer_id = TIMER_ENTER_JEITA;
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		goto error;
	}

	/* Compare the current VBAT and VFLOAT value */
	if (vbat > pca9468->pdata->v_float) {
		pr_info("%s: VBAT(%duV) is already higher than VFLOAT(%duV)\n", 
				__func__, vbat, pca9468->pdata->v_float);
		ret = -EINVAL;
		goto error;
	}
	
	/* Save Charging Configuration */

	/* clear the previous IIN ADC value */
	pca9468->pre_iin_adc = 0;

	/* Set the NTC stage to the stage max as default */
	pca9468->ntc_stage = NTC_STAGE_MAX;

	/* Set TA voltage to max[8000mV, (2*VBAT_ADC + 400 mV)] */
	pca9468->ta_vol = max(PCA9468_TA_MIN_VOL_PRESET, (2*vbat + PCA9468_TA_VOL_PRE_OFFSET));
	val = pca9468->ta_vol/PD_MSG_TA_VOL_STEP;	/* PPS voltage resolution is 20mV */
	pca9468->ta_vol = val*PD_MSG_TA_VOL_STEP;
	/* Set TA current to IIN_CFG*90% */
	pca9468->ta_cur = (pca9468->pdata->iin_cfg)*90/100;
	val = pca9468->ta_cur/PD_MSG_TA_CUR_STEP;	/* PPS current resolution is 50mV */
	pca9468->ta_cur = val*PD_MSG_TA_CUR_STEP;
	pca9468->ta_objpos = 0;	/* Search the proper object position of PDO */
	/* Set TA max current */
	val = pca9468->pdata->iin_cfg/PD_MSG_TA_CUR_STEP;
	pca9468->ta_max_cur = val*PD_MSG_TA_CUR_STEP;

	/* Send PD message */
	ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
	if (ret < 0) {
		pr_info("[PCA] PD request APDO failed\n");
		goto error;
	}

	pr_info("%s: Preset DC, ta_vol=%d, ta_cur=%d\n", 
		__func__, pca9468->ta_vol, pca9468->ta_cur);

	mutex_lock(&pca9468->lock);
	pca9468->timer_id = TIMER_PDMSG_SEND;
	pca9468->timer_period = PCA9468_PDMSG_WAIT_T;
	mutex_unlock(&pca9468->lock);
	queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));

error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Preset direct charging configuration */
static int pca9468_preset_config(struct pca9468_charger *pca9468)
{
	int ret = 0;
	
	pr_info("%s: ======START=======\n", __func__);
	
	pca9468->charging_state = DC_STATE_PRESET_DC;

	/* Set IIN_CFG */
	ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
	if (ret < 0)
		goto error;

	/* Set ICHG_CFG */
	ret = pca9468_set_charging_current(pca9468, pca9468->pdata->ichg_cfg);
	if (ret < 0)
		goto error;

	/* Set VFLOAT */
	ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
	if (ret < 0)
		goto error;

	/* Enable PCA9468 */	
	ret = pca9468_set_charging(pca9468, true);
	if (ret < 0)
		goto error;

	/* Go to Adjust CC mode after 150ms*/
	mutex_lock(&pca9468->lock);
	pca9468->timer_id = TIMER_ENTER_ADJ_CCMODE;
	pca9468->timer_period = PCA4968_ADJCC_DELAY_T;
	mutex_unlock(&pca9468->lock);
	queue_delayed_work(pca9468->dc_wq,
						&pca9468->timer_work,
						msecs_to_jiffies(pca9468->timer_period));
	ret = 0;
	
error:
	pr_info("%s: End, ret=%d\n", __func__, ret);
	return ret; 
}


/* Enter direct charging algorithm */
static int pca9468_start_direct_charging(struct pca9468_charger *pca9468)
{
	int ret;
	unsigned int val;

	pr_info("[PCA] ++%s: =========START=========\n", __func__);

	/* Set OV_DELTA to 40% */
	val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
							PCA9468_BIT_OV_DELTA, val);
	if (ret < 0)
			return ret;

	/* Set Switching Frequency */
	val = pca9468->pdata->fsw_cfg;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
							PCA9468_BIT_FSW_CFG, val);
	if (ret < 0)
		return ret;

	/* current sense resistance */
	val = pca9468->pdata->snsres << MASK2SHIFT(PCA9468_BIT_SNSRES);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
							PCA9468_BIT_SNSRES, val);
	if (ret < 0)
		return ret;

	/* Get power supply for battery */
	pca9468->bat_psy = power_supply_get_by_name("battery");
	if (pca9468->bat_psy == NULL) {
		pr_err("%s: Error:get battery psy fail\n", __func__);
		ret = -ENODEV;
		return ret;
	}
	
	/* wake lock */
	wake_lock(&pca9468->monitor_wake_lock);

	/* Preset charging configuration and TA condition */
	ret = pca9468_preset_dcmode(pca9468);

	pr_info("[PCA] --%s: End, ret=%d\n", __func__, ret);
	return ret;
}


/* Check Vbat minimum level to start direct charging */
static int pca9468_check_vbatmin(struct pca9468_charger *pca9468)
{
	unsigned int vbat;
	int ret;

	pr_info("[PCA] ++%s: =========START=========\n", __func__);

	pca9468->charging_state = DC_STATE_CHECK_VBAT;

	/* Check Vbat */
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);
	if (vbat < 0) {
		ret = vbat;
	}

	pr_info("[PCA] %s: vbat=%d\n", __func__, vbat);

	if (vbat > PCA9468_DC_VBAT_MIN) {
		/* Start Direct Charging */
		/* Read switching charger status */	
		union power_supply_propval val;

		ret = pca9468_get_switching_charger_property(POWER_SUPPLY_PROP_CURRENT_MAX, &val);

		pr_info("[PCA] %s: switching charger current max=%d\n", __func__, val.intval);

		if (ret < 0) {
			pr_info("[PCA] %s: vbat=%d\n", __func__, vbat);

			/* Start Direct Charging again after 1sec */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_VBATMIN_CHECK;
			pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			goto error;
		}
		
		if (val.intval <= SWCHG_ICL_MIN) {
			pr_info("[PCA] %s: switching charger already disabled, start direct charging\n", __func__);

			/* already disabled switching charger */
			/* enable direct charging */
			ret = pca9468_start_direct_charging(pca9468);
			if (ret < 0) {
				/* Fail to start direct charging */
				pr_info("%s: fail to start DC and return to SW charger\n", __func__);											
				goto error;
			}
			//else {	//turn on charging led if direct charging enabled successfully
			//	pca9468_enable_charging_LED(true);
			//}
		} else {
			pr_info("[PCA] %s: switching charger is enabled, disable switching charger\n", __func__);

			/* now switching charger is enabled */
			/* disable switching charger first */
			/* set the charging current of switching charger to 25mA */
			ret = pca9468_set_switching_charger(true, SWCHG_ICL_MIN, 0, 0);

			/* Wait 1sec for stopping switching charger */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_VBATMIN_CHECK;
			pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
		}
	} else {
		/* Read switching charger status */	
		union power_supply_propval val;

		pr_info("[PCA] %s: vbat < min, check if direct charging can be enabled\n", __func__);
		ret = pca9468_get_switching_charger_property(POWER_SUPPLY_PROP_CURRENT_MAX, &val);
		if (ret < 0) {
			/* Start Direct Charging again after 1sec */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_VBATMIN_CHECK;
			pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			goto error;
		}
		
		if (val.intval <= SWCHG_ICL_MIN) {
			/* already disabled switching charger */
			/* enable direct charging */
			ret = pca9468_start_direct_charging(pca9468);
			if (ret < 0) {
				/* Fail to start direct charging */
				pr_info("%s: fail to start DC and return to SW charger\n", __func__);
				goto error;
			}
		} else {
			/* Start 1sec timer for battery check */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_VBATMIN_CHECK;
			pca9468->timer_period = PCA9468_VBATMIN_CHECK_T;
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
		}
	}

error:
	pr_info("--%s: End, ret=%d\n", __func__, ret);
	return ret;
}


static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
		       __FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
		       CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
		       CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

 close_time:
	rtc_class_close(rtc);
	return rc;
}

/* delayed work function for charging timer */
static void pca9468_timer_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 = container_of(work, struct pca9468_charger,
						 timer_work.work);
	int ret = 0;
	
	get_current_time(&pca9468->last_update_time);

	pr_info("[PCA] ++%s: timer id=%d, charging_state=%d, last_update_time=%lu\n", 
		__func__, pca9468->timer_id, pca9468->charging_state, pca9468->last_update_time);
	
	switch (pca9468->timer_id) {
	case TIMER_VBATMIN_CHECK:
		ret = pca9468_check_vbatmin(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ENTER_ADJ_CCMODE:
		ret = pca9468_charge_adjust_ccmode(pca9468);
		if (ret < 0)
			goto error;
		break;			
					
	case TIMER_CCMODE_CHECK:
		ret = pca9468_charge_ccmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ENTER_CCMODE2:
		ret = pca9468_charge_start_ccmode2(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CCMODE2_CHECK:
		ret = pca9468_charge_ccmode2(pca9468);
		if (ret < 0)
			goto error;
		break;
		
	case TIMER_ENTER_CVMODE:
		/* Enter Pre-CV mode */
		ret = pca9468_charge_start_cvmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_CVMODE_CHECK:
		ret = pca9468_charge_cvmode(pca9468);
		if (ret < 0)
			goto error;
		break;

	case TIMER_ENTER_JEITA:
		ret = pca9468_charge_start_jeita(pca9468);
		if (ret < 0)
			goto error;
		break;
		
	case TIMER_JEITA_CHECK:
		ret = pca9468_charge_check_jeita(pca9468);
		if (ret < 0)
			goto error;
		break;
		
	case TIMER_PDMSG_SEND:
		/* Enter here after sending PD message */
		/* Changing TA voltage */
		
		/* check the charging status */
		if (pca9468->charging_state == DC_STATE_PRESET_DC) {
			/* preset pca9468 configuration */
			ret = pca9468_preset_config(pca9468);
		} else if (pca9468->charging_state == DC_STATE_ADJUST_CC) {
			/* Adjust CC mode */
			ret = pca9468_charge_adjust_ccmode(pca9468);
		} else if (pca9468->charging_state == DC_STATE_START_CC) {
			/* Start CC mode */
			/* interrupt enable here if we use interrupt method */
			ret = pca9468_charge_ccmode(pca9468);
		} else if (pca9468->charging_state == DC_STATE_CHECK_CC) {
			/* Check CC mode */
			ret = pca9468_charge_ccmode(pca9468);
		} else if (pca9468->charging_state == DC_STATE_START_CC2) {
			/* Start CC mode2 */
			ret = pca9468_charge_ccmode2(pca9468);
		} else if (pca9468->charging_state == DC_STATE_CHECK_CC2) {
			/* Check CC mode2 */
			ret = pca9468_charge_ccmode2(pca9468);
		} else if (pca9468->charging_state == DC_STATE_START_CV) {
			/* Start CV mode - pre CV mode */
			ret = pca9468_charge_start_cvmode(pca9468);
		} else if (pca9468->charging_state == DC_STATE_CHECK_CV) {
			/* Check CV mode */
			ret = pca9468_charge_cvmode(pca9468);
		} else if (pca9468->charging_state == DC_STATE_NO_CHARGING_JEITA) {
			/* No charging because of JEITA condition */
			/* Start the switching charger and give the master role to it */

			//[+++]Modify for battery safety upgrade
			ret = pca9468_set_switching_charger(true, SWCHG_ICL_MAX, SWCHG_ICL_MAX, g_fv_setting + 20000);
			//[---]Modify for battery safety upgrade

			if (ret < 0)
				goto error;
			/* Notifier PMIC PCA JEITA STOP */
			pca_jeita_stop_pmic_notifier(pca9468->jeita_stage);
			/* Set timer */
			pca9468->timer_id = TIMER_JEITA_CHECK;
			pca9468->timer_period = PCA9468_JEITA_CHECK_T;
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
		} else if (pca9468->charging_state == DC_STATE_CHARGING_DONE) {
			/* Timer ID is none */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_ID_NONE;
			mutex_unlock(&pca9468->lock);
			/* Enable Switching Charger */
			ret = pca9468_set_switching_charger(true, SWCHG_ICL_NORMAL, 
												pca9468->pdata->ichg_cfg, pca9468->pdata->v_float);
			/* Notifier PMIC PCA CHG DONE */
			pca_chg_done_pmic_notifier();
			/* wake unlock */
			wake_unlock(&pca9468->monitor_wake_lock);
		} else if (pca9468->charging_state == DC_STATE_CHANGE_CV_JEITA) {
			/* Set IIN_CFG and Vfloat as new value */
			ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
			if (ret < 0)
				goto error;
			ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
			if (ret < 0)
				goto error;
			/* Enable Charging */
			ret = pca9468_set_charging(pca9468, true);
			if (ret < 0)
				goto error;
			
			/* Start pre-CV mode - with 500ms delay */
			pca9468->timer_period = 500;
			pca9468->timer_id = TIMER_ENTER_CVMODE;
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
		} else {
			ret = -EINVAL;
		}
		if (ret < 0) {
			goto error;
		}
		break;
		
	default:
		break;
	}

	return;
	
error:
	pr_err("[PCA] ++%s: DC error(%d) happens and transit to switching charger\n", 
			__func__, ret);

	/* Return to the switching charger */
	/* Set TA voltage to fixed 5V */
	pca9468->ta_vol = 5000000;
	/* Set TA current to maximum 3A */
	pca9468->ta_cur = 3000000;
	
	/* Send PD Message */
	pca9468->ta_objpos = 1;	// PDO1 - fixed 5V
	ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_FIXED_PDO);
	
	/* wait 200ms */
	msleep(200);
	
	/* Enable Switching Charger */
	if (ret < 0)
		ret = pca9468_set_switching_charger(true, 500000, pca9468->pdata->ichg_cfg, pca9468->pdata->v_float);
	else
		ret = pca9468_set_switching_charger(true, SWCHG_ICL_NORMAL, pca9468->pdata->ichg_cfg, pca9468->pdata->v_float);
	
	pca9468_stop_charging(pca9468);
	pca_chg_done_pmic_notifier();
	return;
}


/* delayed work function for pps periodic timer */
static void pca9468_pps_request_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 = container_of(work, struct pca9468_charger,
						 pps_work.work);

	int ret = 0;
	
	pr_info("%s: pps_work_start\n", __func__);

	/* Send PD message */
	ret = pca9468_send_pd_message(pca9468, PD_MSG_REQUEST_APDO);
	if (ret < 0) {
		/* Stop the direct charging */
		ret = pca9468_stop_charging(pca9468);
	}
	pr_info("%s: End, ret=%d\n", __func__, ret);
}

static int pca9468_hw_init(struct pca9468_charger *pca9468)
{
	unsigned int val;
	int ret;

	pr_info("%s: =========START=========\n", __func__);

	/*
	 * Program the platform specific configuration values to the device
	 * first.
	 */

	/* Set OV_DELTA to 40% */
	val = OV_DELTA_40P << MASK2SHIFT(PCA9468_BIT_OV_DELTA);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_SAFETY_CTRL,
						 	PCA9468_BIT_OV_DELTA, val);
	if (ret < 0) {
		pr_info("[PCA] %s: set OV delta failed\n", __func__);
		return ret;
	}

	/* Set Switching Frequency */
	val = pca9468->pdata->fsw_cfg << MASK2SHIFT(PCA9468_BIT_FSW_CFG);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
						 	PCA9468_BIT_FSW_CFG, val);
	if (ret < 0) {
		pr_info("[PCA] %s: set Switching Freq failed\n", __func__);
		return ret;
	}

	/* current sense resistance */
	val = pca9468->pdata->snsres << MASK2SHIFT(PCA9468_BIT_SNSRES);
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
						 	PCA9468_BIT_SNSRES, val);
	if (ret < 0) {
		pr_info("[PCA] %s: set current sense failed\n", __func__);
		return ret;
	}

	//jonathan, WA to enable 'Reverse current detection' bit during boot
	val = 0x30;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_START_CTRL,
					 	PCA9468_BIT_REV_IIN_DET|PCA9468_BIT_STANDBY_EN|PCA9468_BIT_EN_CFG, val);
	if (ret < 0) {
		pr_info("[PCA] %s: set reverse current detection failed\n", __func__);
		return ret;
	}

	/* clear LIMIT_INCREMENT_EN */
	val = 0;
	ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_IIN_CTRL,
						 	PCA9468_BIT_LIMIT_INCREMENT_EN, val);
	if (ret < 0) {
		pr_info("[PCA] %s: set limit increment en failed\n", __func__);
		return ret;
	}
	
	/* Set the ADC channel */
	val = 0xFF;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_CFG, val);
	if (ret < 0) {
		pr_info("[PCA] %s: set ADC channel failed\n", __func__);
		return ret;
	}

	/* ADC Mode change */
	val = 0x5B;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);
	if (ret < 0)
		return ret;
	val = 0x10;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_MODE, val);
	if (ret < 0)
		return ret;
	val = 0x00;
	ret = regmap_write(pca9468->regmap, PCA9468_REG_ADC_ACCESS, val);	
	if (ret < 0)
		return ret;

	/* Read ADC compensation gain */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_ADC_ADJUST, &val);
	if (ret < 0)
		return ret;
	pca9468->adc_comp_gain = adc_gain[(val>>MASK2SHIFT(PCA9468_BIT_ADC_GAIN))];
	
	/* input current - uA*/	
	ret = pca9468_set_input_current(pca9468, pca9468->pdata->iin_cfg);
	if (ret < 0)
		return ret;

	/* charging current */
	ret = pca9468_set_charging_current(pca9468, pca9468->pdata->ichg_cfg);
	if (ret < 0)
		return ret;

	/* v float voltage */
	ret = pca9468_set_vfloat(pca9468, pca9468->pdata->v_float);
	if (ret < 0)
		return ret;
	
	pr_info("[PCA] --%s\n", __func__);
	return ret;
}


static irqreturn_t pca9468_interrupt_handler(int irq, void *data)
{
	struct pca9468_charger *pca9468 = data;
	u8 int1[REG_INT1_MAX], sts[REG_STS_MAX];	/* INT1, INT1_MSK, INT1_STS, STS_A, B, C, D */
	u8 masked_int;	/* masked int */
	bool handled = false;
	int ret;

	/* Read INT1, INT1_MSK, INT1_STS */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_INT1, int1, 3);
	if (ret < 0) {
		dev_warn(pca9468->dev, "reading INT1_X failed\n");
		return IRQ_NONE;
	}

	/* Read STS_A, B, C, D */
	ret = regmap_bulk_read(pca9468->regmap, PCA9468_REG_STS_A, sts, 4);
	if (ret < 0) {
		dev_warn(pca9468->dev, "reading STS_X failed\n");
		return IRQ_NONE;
	}

	pr_info("%s: int1=0x%2x, int1_sts=0x%2x, sts_a=0x%2x\n", __func__, 
			int1[REG_INT1], int1[REG_INT1_STS], sts[REG_STS_A]);

	/* Check Interrupt */
	masked_int = int1[REG_INT1] & !int1[REG_INT1_MSK];
	if (masked_int & PCA9468_BIT_V_OK_INT) {
		/* V_OK interrupt happened */
		mutex_lock(&pca9468->lock);
		pca9468->mains_online = (int1[REG_INT1_STS] & PCA9468_BIT_V_OK_STS) ? true : false;
		pr_info("[PCA] %s: mains_online=%d\n", __func__, pca9468->mains_online);
		mutex_unlock(&pca9468->lock);
		power_supply_changed(pca9468->mains);
		handled = true;
	}

	if (masked_int & PCA9468_BIT_NTC_TEMP_INT) {
		/* NTC_TEMP interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_NTC_TEMP_STS) {
			/* above NTC_THRESHOLD */
			dev_err(pca9468->dev, "charging stopped due to NTC threshold voltage\n");
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_CHG_PHASE_INT) {
		/* CHG_PHASE interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_CHG_PHASE_STS) {
			/* Any of loops is active*/
			if (sts[REG_STS_A] & PCA9468_BIT_VFLT_LOOP_STS)	{
				/* V_FLOAT loop is in regulation */
				pr_info("%s: V_FLOAT loop interrupt\n", __func__);
				/* Disable CHG_PHASE_M */
				ret = regmap_update_bits(pca9468->regmap, PCA9468_REG_INT1_MSK, 
										PCA9468_BIT_CHG_PHASE_M, PCA9468_BIT_CHG_PHASE_M);
				if (ret < 0) {
					handled = false;
					return handled;
				}
				/* Go to Pre CV Mode */
				pca9468->timer_id = TIMER_ENTER_CVMODE;
				pca9468->timer_period = 10;
				queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			} else if (sts[REG_STS_A] & PCA9468_BIT_IIN_LOOP_STS) {
				/* IIN loop or ICHG loop is in regulation */
				pr_info("%s: IIN loop interrupt\n", __func__);
			} else if (sts[REG_STS_A] & PCA9468_BIT_CHG_LOOP_STS) {
				/* ICHG loop is in regulation */
				pr_info("%s: ICHG loop interrupt\n", __func__);
			}
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_CTRL_LIMIT_INT) {
		/* CTRL_LIMIT interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_CTRL_LIMIT_STS) {
			/* No Loop is active or OCP */
			if (sts[REG_STS_B] & PCA9468_BIT_OCP_FAST_STS) {
				/* Input fast over current */
				dev_err(pca9468->dev, "IIN > 50A instantaneously\n");
			}
			if (sts[REG_STS_B] & PCA9468_BIT_OCP_AVG_STS) {
				/* Input average over current */
				dev_err(pca9468->dev, "IIN > IIN_CFG*150percent\n");
			}
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_TEMP_REG_INT) {
		/* TEMP_REG interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_TEMP_REG_STS) {
			/* Device is in temperature regulation */
			dev_err(pca9468->dev, "Device is in temperature regulation\n");
		}
		handled = true;
	}

	if (masked_int & PCA9468_BIT_ADC_DONE_INT) {
		/* ADC complete interrupt happened */
		dev_dbg(pca9468->dev, "ADC has been completed\n");
		handled = true;
	}

	if (masked_int & PCA9468_BIT_TIMER_INT) {
		/* Timer fault interrupt happened */
		if (int1[REG_INT1_STS] & PCA9468_BIT_TIMER_STS) {
			if (sts[REG_STS_B] & PCA9468_BIT_CHARGE_TIMER_STS) {
				/* Charger timer is expired */
				dev_err(pca9468->dev, "Charger timer is expired\n");
			}
			if (sts[REG_STS_B] & PCA9468_BIT_WATCHDOG_TIMER_STS) {
				/* Watchdog timer is expired */
				dev_err(pca9468->dev, "Watchdog timer is expired\n");
			}
		}
		handled = true;
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}


static int pca9468_irq_init(struct pca9468_charger *pca9468,
			   struct i2c_client *client)
{
	const struct pca9468_platform_data *pdata = pca9468->pdata;
	int ret, msk, irq;

	irq = gpio_to_irq(pdata->irq_gpio);

	pr_info("[PCA] ++%s++, gpio=%d, irq=%d\n", __func__, pdata->irq_gpio, irq);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, pca9468_interrupt_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   client->name, pca9468);
	if (ret < 0)
		goto fail_gpio;

	//
	// Configure the Mask Register for interrupts: disable all interrupts by default.
	//
	msk = (PCA9468_BIT_V_OK_M | 
		   PCA9468_BIT_NTC_TEMP_M | 
		   PCA9468_BIT_CHG_PHASE_M |
		   PCA9468_BIT_RESERVED_M |
		   PCA9468_BIT_CTRL_LIMIT_M |
		   PCA9468_BIT_TEMP_REG_M |
		   PCA9468_BIT_ADC_DONE_M |
		   PCA9468_BIT_TIMER_M);
	ret = regmap_write(pca9468->regmap, PCA9468_REG_INT1_MSK, msk);
	if (ret < 0) {
		pr_info("[PCA] %s: set int1_mask failed\n", __func__);
		goto fail_wirte;
	}
	
	client->irq = irq;
	pr_info("[PCA] --%s: success, client->irq==%d\n", __func__, irq);
	return 0;

fail_wirte:
	free_irq(irq, pca9468);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	client->irq = 0;
	pr_info("[PCA] --%s: irq_init failed\n", __func__);
	return ret;
}



/*
 * Returns the constant charge current programmed
 * into the charger in uA.
 */
static int get_const_charge_current(struct pca9468_charger *pca9468)
{
	int iin = 0, vbat = 0, iin_sts = 0, ichg_sts = 0;
	int ret, intval;
	unsigned int val, reg_val;

	//if (!pca9468->mains_online)
	//	return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_ICHG_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_ICHG_CFG) * 100000;

	/* Read STS_C */
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_C, &reg_val);
	if (ret < 0)
		goto error;

	iin_sts = (reg_val>>2) *100;

	// Read STS_D
	ret = regmap_read(pca9468->regmap, PCA9468_REG_STS_D, &reg_val);
	if (ret < 0)
		goto error;

	ichg_sts = (reg_val>>1) *100;
	
	iin = pca9468_read_adc(pca9468, ADCCH_IIN);
	vbat = pca9468_read_adc(pca9468, ADCCH_VBAT);

error:
	pr_info("[PCA] %s: iin_adc=%d(uA), vbat_adc=%d(v), iin_sts=%d(mA), ichg_sts=%d(mA)\n", __func__, iin, vbat, iin_sts, ichg_sts);	

	return intval;
}

/*
 * Returns the constant charge voltage programmed
 * into the charger in uV.
 */
static int get_const_charge_voltage(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;

	if (!pca9468->mains_online)
		return -ENODATA;

	ret = regmap_read(pca9468->regmap, PCA9468_REG_V_FLOAT, &val);
	if (ret < 0)
		return ret;
	
	intval = (val * 5 + 3725) * 1000;

	return intval;
}

#ifndef CONFIG_DUAL_PD_PORT
/*
 * Returns the enable or disable value.
 * into 1 or 0.
 */
static int get_charging_enabled(struct pca9468_charger *pca9468)
{
	int ret, intval;
	unsigned int val;
	
	ret = regmap_read(pca9468->regmap, PCA9468_REG_START_CTRL, &val);
	if (ret < 0)
		return ret;

	intval = (val & PCA9468_BIT_STANDBY_EN) ? 0 : 1;

	return intval;
}
#endif

int g_Port2Enable;

/* work function for charging enable timer */
static void pca9468_prop_charging_enable_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 = container_of(work, struct pca9468_charger,
						 prop_charging_enable_work.work);
	int ret =0;
	
	pr_info("%s: ======START=======\n", __func__);
	CHG_DBG("pca9468->chg_enable = %d\n", pca9468->chg_enable);
			/* Check the current charging enable status */
		if (pca9468->chg_enable == POWER_SUPPLY_CHARGING_ENABLED_PMI) {
			/* Check the new charging enable status */
			if ((g_Port2Enable == POWER_SUPPLY_CHARGING_DISABLED_RT) ||
				(g_Port2Enable == POWER_SUPPLY_CHARGING_ENABLED_RT)) {
				/* Ignore the new enable status and keep the current status */
				pca9468->chg_enable = POWER_SUPPLY_CHARGING_ENABLED_PMI;
				ret = 0;
			} else {
				/* new charging status is POWER_SUPPLY_CHARGING_DISABLED_PMI */
				/* change the change status to the new one */
				pca9468->chg_enable = g_Port2Enable;
				/* Stop Direct Charging */
				ret = pca9468_stop_charging(pca9468);
				if (ret < 0)
					goto error;
			}
		} else {
			/* change the change status to the new one */
			pca9468->chg_enable = g_Port2Enable;
			if ((g_Port2Enable == POWER_SUPPLY_CHARGING_ENABLED_PMI) ||
				(g_Port2Enable == POWER_SUPPLY_CHARGING_ENABLED_RT)) {
				/* Start Direct Charging */
				/* Start 1sec timer for battery check */
				mutex_lock(&pca9468->lock);
				pca9468->timer_id = TIMER_VBATMIN_CHECK;
				pca9468->timer_period = 5000;	/* The dealy time for PD state goes to PE_SNK_STATE */
				mutex_unlock(&pca9468->lock);
				queue_delayed_work(pca9468->dc_wq,
									&pca9468->timer_work,
									msecs_to_jiffies(pca9468->timer_period));
				ret = 0;
			} else {
				/* Stop Direct Charging */
				ret = pca9468_stop_charging(pca9468);
				if (ret < 0)
					goto error;
			}
		}
error:
	return;
}

#ifdef CONFIG_DUAL_PD_PORT
/* Prop to enable direct charging */
static void pca9468_set_property_charging_enable(struct pca9468_charger *pca9468, int intval)
{
	pr_info("[PCA] ++%s\n", __func__);

	g_Port2Enable = intval;

	queue_delayed_work(pca9468->dc_wq, &pca9468->prop_charging_enable_work, 0);

	return;
}
#endif

extern bool rt_chg_check_asus_vid(void);
extern bool smartchg_stop_flag;
static int pca9468_mains_set_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     const union power_supply_propval *val)
{
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);
	int ret = 0;

	pr_info("[PCA] ++%s: =========START=========\n", __func__);
	pr_info("%s: prop=%d, val=%d\n", __func__, prop, val->intval);

	switch (prop) {
	/* Todo - Insert code */
	/* It needs modification by a customer */
	/* The customer make a decision to start charging and stop charging property */
	
	case POWER_SUPPLY_PROP_ONLINE:	/* need to change property */
		if (val->intval == 0) {
			pr_info("[PCA] %s:Prop_Online: Stop direct charging, mains_online = false\n", __func__);
			pca9468->mains_online = false;

			// Stop Direct charging 
			ret = pca9468_stop_charging(pca9468);
			if (ret < 0)
				goto error;
		} else {
			pr_info("[PCA] %s:Prop_Online: Start direct charging, mains_online = true\n", __func__);
			// Start Direct charging
			pca9468->mains_online = true;
			/* Start 1sec timer for battery check */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_VBATMIN_CHECK;
			pca9468->timer_period = 5000;	/* The delay time for PD state goes to PE_SNK_STATE */
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			ret = 0;
		}
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
#ifdef CONFIG_DUAL_PD_PORT
		if (val->intval == POWER_SUPPLY_CHARGING_ENABLED_PMI) {
			CHG_DBG("POWER_SUPPLY_CHARGING_ENABLED_PMI\n");
			if (smartchg_stop_flag) {
				CHG_DBG("smartchg_stop_flag, don't enable pca9468\n");
				return 0;
			}
			
			if (PE_check_asus_vid())
				g_panel_off_iin = 2900000;
			else
				g_panel_off_iin = 2000000;

			pca9468_set_property_charging_enable(pca9468, val->intval);
		}
		else if (val->intval == POWER_SUPPLY_CHARGING_ENABLED_RT) {
			CHG_DBG("POWER_SUPPLY_CHARGING_ENABLED_RT\n");
			if (smartchg_stop_flag) {
				CHG_DBG("smartchg_stop_flag, don't enable pca9468\n");
				return 0;
			}
			
			if (!gpio_get_value_cansleep(global_gpio->POGO_OVP_ACOK)) {
				CHG_DBG("POGO_OVP is exist, skip POWER_SUPPLY_CHARGING_ENABLED_RT\n");
			} else {
				if (rt_chg_check_asus_vid())
					g_panel_off_iin = 2900000;
				else
					g_panel_off_iin = 2000000;

				pca9468_set_property_charging_enable(pca9468, val->intval);
			}
		}
		else if (val->intval == POWER_SUPPLY_CHARGING_DISABLED_PMI) {
			CHG_DBG("POWER_SUPPLY_CHARGING_DISABLED_PMI\n");
			pca9468_set_property_charging_enable(pca9468, val->intval);
		}
		else if (val->intval == POWER_SUPPLY_CHARGING_DISABLED_RT) {
			CHG_DBG("POWER_SUPPLY_CHARGING_DISABLED_RT\n");
			pca9468_set_property_charging_enable(pca9468, val->intval);
		}
#else
		if (val->intval == 0) 
		{
			pr_info("[PCA] %s:Prop_CHARGING_ENABLED: Stop direct charging\n", __func__);

			// Stop Direct Charging
			ret = pca9468_stop_charging(pca9468);
			if (ret < 0)
				goto error;
		} else {
			pr_info("[PCA] %s:Prop_CHARGING_ENABLED: Start direct charging, schedule timer_work\n", __func__);
			// Start Direct Charging
			/* Start 1sec timer for battery check */
			mutex_lock(&pca9468->lock);
			pca9468->timer_id = TIMER_VBATMIN_CHECK;
			pca9468->timer_period = 5000;	/* The dealy time for PD state goes to PE_SNK_STATE */
			mutex_unlock(&pca9468->lock);
			queue_delayed_work(pca9468->dc_wq,
								&pca9468->timer_work,
								msecs_to_jiffies(pca9468->timer_period));
			ret = 0;
		}
#endif
		break;

#ifdef CONFIG_DUAL_PD_PORT
	case POWER_SUPPLY_PROP_PD_PORT:
		pr_info("[PCA] %s:Prop_PD_PORT: %d\n", __func__, val->intval);

		/* Check the current PD port */
		/* QCOM PD PHY has high priority to RT PD PHY */
		if (pca9468->pd_port == POWER_SUPPLY_PD_PORT_PMI) {
			if ((val->intval == POWER_SUPPLY_PD_PORT_RT) ||
				(val->intval == POWER_SUPPLY_PD_PORT_NONE_RT)) {
				/* Ignore Port change and keep QCOM PD PHY and the current direct charging */
				pca9468->pd_port = POWER_SUPPLY_PD_PORT_PMI;
			} else {
				if (val->intval == POWER_SUPPLY_PD_PORT_NONE_PMI) {
					/* Change PD port */
					pca9468->pd_port = POWER_SUPPLY_PD_PORT_NONE;
				}
			}
		} else {
			if ((val->intval == POWER_SUPPLY_PD_PORT_NONE_PMI) ||
				(val->intval == POWER_SUPPLY_PD_PORT_NONE_RT)) {
				/* Change PD port to PORT_NONE */
				pca9468->pd_port = POWER_SUPPLY_PD_PORT_NONE;
			} else {
				/* Change PD port */
				pca9468->pd_port = val->intval;
				//[+++] Add to update the VID when PD gets the correct PID
				if (PE_check_asus_vid())
					PD_notify_VID();
				//[---] Add to update the VID when PD gets the correct PID
			}
		}
		ret = 0;
		break;
#endif

	default:
		ret = -EINVAL;
		break;
	}

error:
	pr_info("--%s: End, ret=%d\n", __func__, ret);
	return ret;
}


static int pca9468_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	int ret = 0;
	struct pca9468_charger *pca9468 = power_supply_get_drvdata(psy);

	//pr_info("[PCA] ++%s: start\n", __func__);

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		//pr_info("[PCA] %s: prop_online, mains_online= %d\n", __func__, pca9468->mains_online);
		val->intval = pca9468->mains_online;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = get_const_charge_voltage(pca9468);
		pr_info("[PCA] %s: prop_charge_voltage, ret=%d\n", __func__, ret);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = get_const_charge_current(pca9468);
		pr_info("[PCA] %s: prop_charge_current, ret=%d\n", __func__, ret);
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
#ifdef CONFIG_DUAL_PD_PORT
		//fix bug that wrongly assigns val->intval 
		ret = pca9468->chg_enable;
#else
		ret = get_charging_enabled(pca9468);
		pr_info("[PCA] %s: prop_charge_enable, ret=%d\n", __func__, ret);
#endif
		if (ret < 0)
			return ret;
		else
			val->intval = ret;
		break;

#ifdef CONFIG_DUAL_PD_PORT
	case POWER_SUPPLY_PROP_PD_PORT:
		if (&pca9468->pd_port == NULL) {
			pr_err("[PCA] %s: pca9468->pd_port = NULL\n", __func__);
			return -EINVAL;
		}

		if(pca9468->pd_port == POWER_SUPPLY_PD_PORT_NONE) {
			pr_info("[PCA] %s: prop_pd_port not yet initial\n", __func__);
			return -EAGAIN;
		}

		val->intval = pca9468->pd_port;
		pr_info("[PCA] %s: prop_pd_port, intval=%d\n", __func__, val->intval);
		break;
#endif

	default:
		pr_info("[PCA] %s: prop default\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property pca9468_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_PD_PORT,
};


static const struct regmap_config pca9468_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= PCA9468_MAX_REGISTER,
};

static const struct power_supply_desc pca9468_mains_desc = {
	.name		= "pca9468-mains",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.get_property	= pca9468_mains_get_property,
	.set_property 	= pca9468_mains_set_property,
	.properties	= pca9468_mains_properties,
	.num_properties	= ARRAY_SIZE(pca9468_mains_properties),
};

#if defined(CONFIG_OF)
static int of_pca9468_dt(struct device *dev, struct pca9468_platform_data *pdata)
{
	struct device_node *np_pca9468 = dev->of_node;
	int ret;
	if(!np_pca9468)
		return -EINVAL;

	/* irq gpio */
	pdata->irq_gpio = of_get_named_gpio(np_pca9468, "pca9468,irq-gpio", 0);
	pr_info("%s: irq-gpio: %u \n", __func__, pdata->irq_gpio);

	/* input current limit */
	ret = of_property_read_u32(np_pca9468, "pca9468,input-current-limit",
						   &pdata->iin_cfg);
	if (ret) {
		pr_info("%s: pca9468,input-current-limit is Empty\n", __func__);
		pdata->iin_cfg = PCA9468_IIN_CFG_DFT;
	}
	pr_info("%s: pca9468,iin_cfg is %d\n", __func__, pdata->iin_cfg);

	/* charging current */
	ret = of_property_read_u32(np_pca9468, "pca9468,charging-current",
							   &pdata->ichg_cfg);
	if (ret) {
		pr_info("%s: pca9468,charging-current is Empty\n", __func__);
		pdata->ichg_cfg = PCA9468_ICHG_CFG_DFT;
	}
	pr_info("%s: pca9468,ichg_cfg is %d\n", __func__, pdata->ichg_cfg);

	/* charging float voltage */
	ret = of_property_read_u32(np_pca9468, "pca9468,float-voltage",
							   &pdata->v_float);
	if (ret) {
		pr_info("%s: pca9468,float-voltage is Empty\n", __func__);
		pdata->v_float = PCA9468_VFLOAT_DFT;
	}
	pr_info("%s: pca9468,v_float is %d\n", __func__, pdata->v_float);

	/* input topoff current */
	ret = of_property_read_u32(np_pca9468, "pca9468,input-itopoff",
							   &pdata->iin_topoff);
	if (ret) {
		pr_info("%s: pca9468,input-itopoff is Empty\n", __func__);
		pdata->iin_topoff = PCA9468_IIN_DONE_DFT;
	}
	pr_info("%s: pca9468,iin_topoff is %d\n", __func__, pdata->iin_topoff);

	/* charging topoff current */
	ret = of_property_read_u32(np_pca9468, "pca9468,charging-itopoff",
							   &pdata->ichg_topoff);
	if (ret) {
		pr_info("%s: pca9468,charging-itopoff is Empty\n", __func__);
		pdata->ichg_topoff = PCA9468_ICHG_DONE_DFT;
	}
	pr_info("%s: pca9468,ichg_topoff is %d\n", __func__, pdata->ichg_topoff);

	/* sense resistance */
	ret = of_property_read_u32(np_pca9468, "pca9468,sense-resistance",
							   &pdata->snsres);
	if (ret) {
		pr_info("%s: pca9468,sense-resistance is Empty\n", __func__);
		pdata->snsres = PCA9468_SENSE_R_DFT;
	}
	pr_info("%s: pca9468,snsres is %d\n", __func__, pdata->snsres);

	/* switching frequency */
	ret = of_property_read_u32(np_pca9468, "pca9468,switching-frequency",
							   &pdata->fsw_cfg);
	if (ret) {
		pr_info("%s: pca9468,switching frequency is Empty\n", __func__);
		pdata->snsres = PCA9468_FSW_CFG_DFT;
	}
	pr_info("%s: pca9468,fsw_cfg is %d\n", __func__, pdata->fsw_cfg);

	return 0;
}
#else
static int of_pca9468_dt(struct device *dev, struct pca9468_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#ifdef CONFIG_USBPD_PHY_QCOM
static int pca9468_usbpd_setup(struct pca9468_charger *pca9468)
{
	int ret = 0;
	const char *pd_phandle = "usbpd-phy";

	pca9468->pd = devm_usbpd_get_by_phandle(pca9468->dev, pd_phandle);

	if (IS_ERR(pca9468->pd)) {
		pr_err("get_usbpd phandle failed (%ld)\n",
				PTR_ERR(pca9468->pd));
		return PTR_ERR(pca9468->pd);
	}

	return ret;
}
#endif

static int read_reg(void *data, u64 *val)
{
	struct pca9468_charger *chip = data;
	int rc;
	unsigned int temp;

	rc = regmap_read(chip->regmap, chip->debug_address, &temp); 
	if (rc) {
		pr_err("Couldn't read reg %x rc = %d\n",
			chip->debug_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int write_reg(void *data, u64 val)
{
	struct pca9468_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = regmap_write(chip->regmap, chip->debug_address, temp);
	if (rc) {
		pr_err("Couldn't write 0x%02x to 0x%02x rc= %d\n",
			temp, chip->debug_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops, read_reg, write_reg, "0x%02llx\n");

static int pca9468_create_debugfs_entries(struct pca9468_charger *chip)
{
	struct dentry *ent;
	int rc = 0;
	
	chip->debug_root = debugfs_create_dir("charger-pca9468", NULL);
	if (!chip->debug_root) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		rc = -ENOENT;
	} else {
		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->debug_address));
		if (!ent) {
			dev_err(chip->dev,
				"Couldn't create address debug file\n");
			rc = -ENOENT;
		}

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &register_debug_ops);
		if (!ent) {
			dev_err(chip->dev,
				"Couldn't create data debug file\n");
			rc = -ENOENT;
		}
	}

	return rc;
}
/*
static void pca9468_usbpd_setup_work(struct work_struct *work)
{
	struct pca9468_charger *pca9468 = container_of(work, struct pca9468_charger,
						 init_pd_work.work);
	
	pr_info("[PCA] %s start\n", __func__);

	if (pca9468_usbpd_setup(pca9468)) {
		//dev_err(dev, "Error usbpd setup!\n");
		pr_err("[PCA] %s: Error usbpd setup!\n", __func__);
		pca9468->pd = NULL;
	}
}
*/

// Register FB notifier +++
static int pca9468_fb_callback(struct notifier_block *nb, unsigned long val, void *data)
{
	struct pca9468_charger *pca9468_chg;
	struct drm_panel_notifier *evdata = data;
	unsigned int blank;

	if (!evdata)
		return 0;

	if (val != DRM_PANEL_EVENT_BLANK)
		return 0;

	printk("[PCA] go to the pca9468_fb_callback value = %d\n", (int)val);
	
	pca9468_chg = container_of(nb, struct pca9468_charger, notifier);
	if (evdata && evdata->data && val == DRM_PANEL_EVENT_BLANK && pca9468_chg) {
		blank = *(int *)(evdata->data);

	//printk("[PCA]  go to the blank value = %d\n", (int)blank);

	switch (blank) {
		case DRM_PANEL_BLANK_POWERDOWN:
			printk("[PCA] Panel Off, set IIN to 4A\n");

			g_PanelOnOff = 0;
			if (!gpio_get_value_cansleep(global_gpio->BTM_OVP_ACOK)) {
				g_inov_temp_max = 49000;//Change from 48->53
				g_inov_temp_min = 46000;//Change from 45->50
			} else {
				g_inov_temp_max = 57000;//Change from 48->53
				g_inov_temp_min = 54000;//Change from 45->50
			}
			
			/* Set IIN_CFG */
			//pca9468_set_input_current(pca9468_chg, PCA9468_CC1_IIN_CFG_4A);

			break;
		case DRM_PANEL_BLANK_UNBLANK:
			printk("[PCA] Panel On, set IIN to 3A\n");

			//[+++]This is a temporary WA for incorrect drm_notifier in COS mode
			if (g_Charger_mode) {
				printk("[PCA] COS mode, Keep Panel Off\n");
				g_PanelOnOff = 0;
			} else {
				g_PanelOnOff = 1;
			}
			//[---]This is a temporary WA for incorrect drm_notifier in COS mode
			g_inov_temp_max = 47000;
			g_inov_temp_min = 45000;

			/* Set IIN_CFG */
			//pca9468_set_input_current(pca9468_chg, PCA9468_CC1_IIN_CFG_3A);
			break;
		default:
			printk("[PCA] blank switch to default\n");
			break;
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block ec_hid_noti_block = {
	.notifier_call = pca9468_fb_callback,
};
// Register FB notifier ---

// ASUS BSP Austin_T : Add attributes +++
static ssize_t panel_on_iin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "panel_on_iin = %d\n", g_panel_on_iin);
}

static ssize_t panel_on_iin_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);
	g_panel_on_iin = tmp;
	pr_info("%s: set g_panel_on_iin = %d\n", __func__, g_panel_on_iin);

	return len;
}

static ssize_t panel_off_iin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "panel_off_iin = %d\n", g_panel_off_iin);
}

static ssize_t panel_off_iin_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);
	g_panel_off_iin = tmp;
	pr_info("%s: set g_panel_off_iin = %d\n", __func__, g_panel_off_iin);

	return len;
}

static ssize_t high_volt_4P25_iin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "high_volt_4P25_iin = %d\n", g_high_volt_4P25_iin);
}

static ssize_t high_volt_4P25_iin_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);
	g_high_volt_4P25_iin = tmp;
	pr_info("%s: set g_high_volt_4P25_iin = %d\n", __func__, g_high_volt_4P25_iin);

	return len;
}

static ssize_t inov_temp_max_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "inov_temp max = %d, min = %d\n", g_inov_temp_max, g_inov_temp_min);
}

static ssize_t inov_temp_max_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);
	g_inov_temp_max = tmp;	
	g_inov_temp_min = g_inov_temp_max - 3000;
	pr_info("%s: set g_inov_temp max = %d, min = %d\n", __func__, g_inov_temp_max, g_inov_temp_min);

	return len;
}

static ssize_t inov_overtemp_iin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "inov_overtemp_iin = %d\n", g_inov_overtemp_iin);
}

static ssize_t inov_overtemp_iin_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);
	g_inov_overtemp_iin = tmp;
	pr_info("%s: set g_inov_overtemp_iin = %d\n", __func__, g_inov_overtemp_iin);

	return len;
}

static ssize_t inov_overtemp_iin_low_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "inov_overtemp_iin_low = %d\n", g_inov_overtemp_iin_low);
}

static ssize_t inov_overtemp_iin_low_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int tmp = 0;

	tmp = simple_strtol(buf, NULL, 10);
	g_inov_overtemp_iin_low = tmp;
	pr_info("%s: set g_inov_overtemp_iin = %d\n", __func__, g_inov_overtemp_iin_low);

	return len;
}

static DEVICE_ATTR(panel_on_iin, 0664, panel_on_iin_show, panel_on_iin_store);
static DEVICE_ATTR(panel_off_iin, 0664, panel_off_iin_show, panel_off_iin_store);
static DEVICE_ATTR(high_volt_4P25_iin, 0664, high_volt_4P25_iin_show, high_volt_4P25_iin_store);
static DEVICE_ATTR(inov_temp_max, 0664, inov_temp_max_show, inov_temp_max_store);
static DEVICE_ATTR(inov_overtemp_iin, 0664, inov_overtemp_iin_show, inov_overtemp_iin_store);
static DEVICE_ATTR(inov_overtemp_iin_low, 0664, inov_overtemp_iin_low_show, inov_overtemp_iin_low_store);

static struct attribute *pca9468_attrs[] = {
	&dev_attr_panel_on_iin.attr,
	&dev_attr_panel_off_iin.attr,
	&dev_attr_high_volt_4P25_iin.attr,
	&dev_attr_inov_temp_max.attr,
	&dev_attr_inov_overtemp_iin.attr,
	&dev_attr_inov_overtemp_iin_low.attr,
	NULL
};

static const struct attribute_group pca9468_attr_group = {
	.attrs = pca9468_attrs,
};
// ASUS BSP Austin_T : Add attributes ---

void pca9468_smartchg_slow_charging(bool enable) 
{
	int ret;

	if (enable) {
		CHG_DBG("enable slow charging\n");
		g_panel_off_iin = SLOWE_CHARGING_TA_CUR;
		g_panel_on_iin = SLOWE_CHARGING_TA_CUR;
		g_high_volt_4P25_iin = SLOWE_CHARGING_TA_CUR;
		g_inov_overtemp_iin = SLOWE_CHARGING_TA_CUR;
		g_inov_overtemp_iin_low = SLOWE_CHARGING_TA_CUR;
	} else {
		CHG_DBG("disable slow charging\n");
		if (PE_check_asus_vid() || rt_chg_check_asus_vid())
			g_panel_off_iin = 2900000;
		else
			g_panel_off_iin = 2000000;
		g_panel_on_iin = 2000000;
		g_high_volt_4P25_iin = 2000000;
		g_inov_overtemp_iin = 2000000;
		g_inov_overtemp_iin_low = 1400000;
	}

	pca9468_chg_dev->ta_cur = g_panel_on_iin;
	/* Send PD Message */
	ret = pca9468_send_pd_message(pca9468_chg_dev, PD_MSG_REQUEST_APDO);
	if (ret < 0)
		CHG_DBG("pca9468_send_pd_message fail\n");
}

extern bool side_pps_flag;
void pca9468_smartchg_stop_charging(bool enable) 
{
	int ret = 0;
	struct power_supply *psy_pca9468;
	union power_supply_propval val = {0};

	if (enable) {
		CHG_DBG("enable smartchg stop charging\n");

		/* Change charging status */
		pca9468_chg_dev->charging_state = DC_STATE_CHARGING_DONE;
		
		/* Set TA voltage to fixed 5V */
		pca9468_chg_dev->ta_vol = 5000000;
		/* Set TA current to maximum 3A */
		pca9468_chg_dev->ta_cur = 3000000;
		
		/* Send PD Message */
		pca9468_chg_dev->ta_objpos = 1;	// PDO1 - fixed 5V
		ret = pca9468_send_pd_message(pca9468_chg_dev, PD_MSG_REQUEST_FIXED_PDO);
		if (ret < 0) {
			CHG_DBG("can not send fixed pdo 5V/3A\n");
			return;
		}
		
		pca9468_stop_charging(pca9468_chg_dev);
		pca9468_chg_dev->chg_enable = 0;
		
		ret = pca9468_set_switching_charger(true, SWCHG_ICL_NORMAL, 
												pca9468_chg_dev->pdata->ichg_cfg, pca9468_chg_dev->pdata->v_float);
	}
	else {
		CHG_DBG("disable smartchg stop charging\n");
		psy_pca9468 = power_supply_get_by_name("pca9468-mains");
		
		if (side_pps_flag)
			val.intval = POWER_SUPPLY_CHARGING_ENABLED_PMI;
		else
			val.intval = POWER_SUPPLY_CHARGING_ENABLED_RT;

		ret = power_supply_set_property(psy_pca9468, POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (ret < 0)
			CHG_DBG("can not enable pca9468\n");
	}
}

extern struct drm_panel *active_panel_asus;
static int pca9468_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{	
	static char *battery[] = { "pca9468-battery" };
	struct power_supply_config mains_cfg = {};
	struct pca9468_platform_data *pdata;
	struct device *dev = &client->dev;
	struct pca9468_charger *pca9468_chg;
//ASUS BSP : Request charger GPIO
	struct pinctrl *chg_pc;
	struct pinctrl_state *chg_pcs;
	int rc;

	int ret;

	pr_info("[PCA] %s: =========START=========\n", __func__);

	//msleep(1000);

	pca9468_chg = devm_kzalloc(dev, sizeof(*pca9468_chg), GFP_KERNEL);
	if (!pca9468_chg)
		return -ENOMEM;

#if defined(CONFIG_OF)
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(struct pca9468_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory \n");
			return -ENOMEM;
		}

		ret = of_pca9468_dt(&client->dev, pdata);
		if (ret < 0){
			dev_err(&client->dev, "Failed to get device of_node \n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
	} else {
		pdata = client->dev.platform_data;
	}
#else
	pdata = dev->platform_data;
#endif
	if (!pdata)
		return -EINVAL;

	i2c_set_clientdata(client, pca9468_chg);

	mutex_init(&pca9468_chg->lock);
	pca9468_chg->dev = &client->dev;
	pca9468_chg->pdata = pdata;
	pca9468_chg->charging_state = DC_STATE_NO_CHARGING;
#ifdef CONFIG_DUAL_PD_PORT
	pca9468_chg->pd_port = POWER_SUPPLY_PD_PORT_NONE;
#endif
	pca9468_chg->jeita_stage = JEITA_STAGE_MAX;
	pca9468_chg->ntc_stage = NTC_STAGE_MAX;

	/* Workaround for PS_RDY time */
	pca9468_chg->prev_ta_vol = 0;
	
#ifdef CONFIG_USBPD_PHY_QCOM

	//if (pca9468_usbpd_setup(pca9468_chg)) {
	ret = pca9468_usbpd_setup(pca9468_chg);
	if(ret < 0) {
		dev_err(dev, "Error usbpd setup!\n");
		pca9468_chg->pd = NULL;
		return ret;
	}

#endif

	pca9468_chg_dev = pca9468_chg;	// ASUS BSP Add for globle dev

//ASUS BSP : Request charger GPIO +++
	chg_pc = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(chg_pc)) {
		pr_err("%s: failed to get charger pinctrl\n", __func__);
	}
	chg_pcs = pinctrl_lookup_state(chg_pc, "pca_gpio_default");
	if (IS_ERR_OR_NULL(chg_pcs)) {
		pr_err("%s: failed to get charger input pinctrl state from dtsi\n", __func__);
	}
	rc = pinctrl_select_state(chg_pc, chg_pcs);
	if(rc < 0) {
		pr_err("%s: failed to set charger input pinctrl state\n", __func__);
	}

	pr_info("[PCA] %s: pinctrl init done\n", __func__);
//ASUS BSP : Request charger GPIO +++

	/* Create a work queue for the direct charger */
	pca9468_chg->dc_wq = alloc_ordered_workqueue("pca9468_dc_wq", WQ_MEM_RECLAIM);
	if (pca9468_chg->dc_wq == NULL) {
		dev_err(pca9468_chg->dev, "failed to create work queue\n");
		return -ENOMEM;
	}

	wake_lock_init(&pca9468_chg->monitor_wake_lock, WAKE_LOCK_SUSPEND,
		       "pca9468-charger-monitor");

	/* initialize work */
	INIT_DELAYED_WORK(&pca9468_chg->timer_work, pca9468_timer_work);
	pca9468_chg->timer_id = TIMER_ID_NONE;
	pca9468_chg->timer_period = 0;

	INIT_DELAYED_WORK(&pca9468_chg->pps_work, pca9468_pps_request_work);

	//jonathan, prop_enable
	INIT_DELAYED_WORK(&pca9468_chg->prop_charging_enable_work, pca9468_prop_charging_enable_work);

	pca9468_chg->regmap = devm_regmap_init_i2c(client, &pca9468_regmap);
	if (IS_ERR(pca9468_chg->regmap))
		return PTR_ERR(pca9468_chg->regmap);


	//++jonathan
//	INIT_DELAYED_WORK(&pca9468_chg->init_pd_work, pca9468_usbpd_setup_work);

//	schedule_delayed_work(&pca9468_chg->init_pd_work, msecs_to_jiffies(PCA9468_PPS_PERIODIC_T)); //10s
	//--

	ret = pca9468_hw_init(pca9468_chg);
	if (ret < 0)
		return ret;

	mains_cfg.supplied_to = battery;
	mains_cfg.num_supplicants = ARRAY_SIZE(battery);
	mains_cfg.drv_data = pca9468_chg;
	pca9468_chg->mains = power_supply_register(dev, &pca9468_mains_desc,
					   &mains_cfg);
	if (IS_ERR(pca9468_chg->mains))
		return PTR_ERR(pca9468_chg->mains);

	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = pca9468_irq_init(pca9468_chg, client);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
		}
		/* disable interrupt */
		disable_irq(client->irq);
	}

	ret = pca9468_create_debugfs_entries(pca9468_chg);
	if (ret < 0)
		return ret;

	pca9468_chg->notifier = ec_hid_noti_block;
	if(active_panel_asus != NULL) {
		if (drm_panel_notifier_register(active_panel_asus, &ec_hid_noti_block) < 0)
			pr_info("Failed to register fb notifier client\n");
	}
	//ASUS BSP Austin_T : PCA_ATTRs +++
	rc = sysfs_create_group(&pca9468_chg->dev->kobj, &pca9468_attr_group);
	if (rc) {
		pr_err("%s: failed to create pca9468 attr\n", __func__);
		sysfs_remove_group(&pca9468_chg->dev->kobj, &pca9468_attr_group);
	}
	//ASUS BSP Austin_T : PCA_ATTRs ---

	// ASUS BSP : Set PCA IIN value +++
	if ( g_ASUS_hwID >= HW_REV_EVB ) {
		g_panel_off_iin = 2900000;
		g_panel_on_iin = 2000000;
		g_high_volt_4P25_iin = 2000000;
		g_inov_temp_max = 50000;
		g_inov_temp_min = 48000;
		g_inov_overtemp_iin = 2000000;
		g_inov_overtemp_iin_low = 1400000;
	}
	// ASUS BSP : Set PCA IIN value +++

	pr_info("[PCA] %s: =========END=========\n", __func__);

	if(pca9468_chg->pd == NULL) {
		pr_info("[PCA] --%s: probe defer\n", __func__);
		return -EPROBE_DEFER;
	}
	else
		return 0;
}

static int pca9468_remove(struct i2c_client *client)
{
	struct pca9468_charger *pca9468_chg = i2c_get_clientdata(client);

	if (client->irq) {
		free_irq(client->irq, pca9468_chg);
		gpio_free(pca9468_chg->pdata->irq_gpio);
	}

	/* Delete the work queue */
	destroy_workqueue(pca9468_chg->dc_wq);

	wake_lock_destroy(&pca9468_chg->monitor_wake_lock);
	power_supply_unregister(pca9468_chg->mains);
	return 0;
}

static const struct i2c_device_id pca9468_id[] = {
	{ "pca9468", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pca9468_id);

#if defined(CONFIG_OF)
static struct of_device_id pca9468_i2c_dt_ids[] = {
	{ .compatible = "nxp,pca9468" },
	{ },
};
MODULE_DEVICE_TABLE(of, pca9468_i2c_dt_ids);
#endif /* CONFIG_OF */

#if defined(CONFIG_PM)
static void pca9468_check_and_update_charging_timer(struct pca9468_charger *pca9468)
{
	unsigned long current_time = 0, next_update_time, time_left;

	get_current_time(&current_time);
	
	if (pca9468->timer_id != TIMER_ID_NONE)	{
		next_update_time = pca9468->last_update_time + (pca9468->timer_period / 1000);	// unit is second

		pr_info("%s: current_time=%ld, next_update_time=%ld\n", __func__, current_time, next_update_time);

		if (next_update_time > current_time)
			time_left = next_update_time - current_time;
		else
			time_left = 0;

		mutex_lock(&pca9468->lock);
		pca9468->timer_period = time_left * 1000;	// ms unit
		mutex_unlock(&pca9468->lock);
		queue_delayed_work(pca9468->dc_wq,
							&pca9468->timer_work,
							msecs_to_jiffies(pca9468->timer_period));
		pr_info("%s: timer_id=%d, time_period=%ld\n", __func__, pca9468->timer_id, pca9468->timer_period);
	}
	pca9468->last_update_time = current_time;
}


static int pca9468_suspend(struct device *dev)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);

	pr_info("%s: cancel delayed work\n", __func__);

	/* cancel delayed_work */
	cancel_delayed_work(&pca9468->timer_work);
	return 0;
}

static int pca9468_resume(struct device *dev)
{
	struct pca9468_charger *pca9468 = dev_get_drvdata(dev);

	pr_info("%s: update_timer\n", __func__);

	/* Update the current timer */
	pca9468_check_and_update_charging_timer(pca9468);

	return 0;
}
#else
#define pca9468_suspend		NULL
#define pca9468_resume		NULL
#endif

const struct dev_pm_ops pca9468_pm_ops = {
	.suspend = pca9468_suspend,
	.resume = pca9468_resume,
};

static struct i2c_driver pca9468_driver = {
	.driver = {
		.name = "pca9468",
#if defined(CONFIG_OF)
		.of_match_table = pca9468_i2c_dt_ids,
#endif /* CONFIG_OF */
#if defined(CONFIG_PM)
		.pm = &pca9468_pm_ops,
#endif
	},
	.probe        = pca9468_probe,
	.remove       = pca9468_remove,
	.id_table     = pca9468_id,
};

module_i2c_driver(pca9468_driver);

MODULE_AUTHOR("Clark Kim <clark.kim@nxp.com>");
MODULE_DESCRIPTION("PCA9468 charger driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.4.3A");
