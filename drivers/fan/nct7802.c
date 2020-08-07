/*
    nct7802.c - Linux kernel driver for hardware monitoring
    Copyright (C) 2011 Nuvoton Technology Corp.
                  Sheng-Yuan Huang

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation - version 2.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.


    Supports following chips:

    Chip       #vin   #fanin #temp #dts wchipid  vendid  versionID  i2c  ISA
    nct7802    5        3      4     2    0xC3     0x50    0x2X     yes   no
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-vid.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include "nct7802.h"
#include <linux/of_gpio.h>

#include <linux/usb.h>
#include <linux/msm_drm_notify.h>

//#include "linux/asusdebug.h"

//ASUS_SZ_BSP Cassie  add gDongleType to diff jedi's inbox and DT+++
extern uint8_t gDongleType;
uint8_t fan_current_type=0;
//ASUS_SZ_BSP Cassie  add gDongleType to diff jedi's inbox and DT---
/* Addresses to scan */
static unsigned short normal_i2c[] = { 0x28, 0x29, 0x2A, 0x2B, 
					0x2C, 0x2D, 0x2E, 0x2F, I2C_CLIENT_END };

/* Insmod parameters */
static unsigned short force_subclients[4];
module_param_array(force_subclients, short, NULL, 0);
MODULE_PARM_DESC(force_subclients, "List of subclient addresses: "
		       "{bus, clientaddr, subclientaddr1, subclientaddr2}");

static int reset;
module_param(reset, int, 0);
MODULE_PARM_DESC(reset, "Set to 1 to reset chip, not recommended");

#define NCT7802_REG_BANKSEL    0x0
#define NCT7802_REG_VENDORID   0xFD
#define NCT7802_REG_CHIPID     0xFE
#define NCT7802_REG_DEVICEID   0xFF

//#define NCT7802_REG_I2C_ADDR        	0xXX
#define NCT7802_REG_SOFTWARE_RESET  0xFC
#define NCT7802_REG_START				0x21

/* Multi-Function Pin Ctrl Registers */
#define NCT7802_REG_MODE_SELECTION	0x22
static const u8 NCT7802_REG_MODE_SELECTION_SHFIT[] = {0, 2, 4, 6}; //RTD1, RTD2, RTD3, LTD

#define NCT7802_REG_FAN_ENABLE		0x24
#define NCT7802_REG_VOLT_MONITOR_ENABLE	0x25

/* shift bits of TR1~TR4 in NCT7802_REG_VOLT_TEMP_CTRL */
//static u16 NCT7802_TEMP_CTRL_SHIFT[] = {0,2,4,6};

/* TEMP register */
#define TEMP_READ       0
#define TEMP_CRIT       1
#define TEMP_HL      	2
#define TEMP_LL  			3
#define TEMP_LSB_MASK	0xE0
/* Feild: current, crit, high limit, low limit 
   Attention: current value has decimal fraction. */
static u16 NCT7802_REG_TEMP[][4] = {
	{0x1, 0x3A,  0x30, 0x31}, // TR1/TD1
	{0x2, 0x3B,  0x32, 0x33}, // TR2/TD2
	{0x3, 0x3C,  0x34, 0x35}, // TR3 
	{0x4, 0x3D,  0x36, 0x37}, // LTD. 
};
static u16 NCT7802_REG_TEMP_LSB = 0x5; //Notice!! LTD current value has NO decimal fraction.

/* Voltage register */
#define IN_READ				0
#define IN_MAX					1
#define IN_LOW					2

static const u16 NCT7802_REG_IN[][3] = {
	/* Current, HL, LL */
	{0x9, 0x48, 0x48},	/* VCC	*/
	{0xC, 0x47, 0x47},	/* VSEN1	*/
	{0xD, 0x47, 0x47},	/* VSEN2	*/
	{0xE, 0x48, 0x48}	/* VSEN3	*/
};
static const u16 NCT7802_REG_IN_VCORE = 0xA; //Notice: VCore seems has NO high/low limit
static const u16 IN_LSB_REG = 0xF; 
static const u16 IN_LSB_MASK = 0xC0;

/* Field:  VCC, VSEN1, VSEN2, VSEN3
   Notice: VCore has NO high/low limit */
static const u16 IN_HL_MSB_MASK[] = {0xC, 0xC0, 0xC, 0xC0};
static const u16 IN_HL_MSB_SHIFT[] = {6, 2, 6, 2}; //For shifting value to bit[9:8]
static const u16 IN_LL_MSB_MASK[] = {0x3, 0x30, 0x3, 0x30};
static const u16 IN_LL_MSB_SHIFT[] = {8, 4, 8, 4};

static const u16 IN_HL_LSB_REG[] = {0x45, 0x3F, 0x41, 0x43}; 
static const u16 IN_LL_LSB_REG[] = {0x46, 0x40, 0x42, 0x44}; 

/* VCC, VSEN1, VSEN2, VSEN3, VCORE */
static const u8 IN_MULTIPLIER[] = {4, 2, 2, 2, 2};

#define NCT7802_REG_FAN(index)    		(0x10 + index)
#define NCT7802_REG_FAN_LSB 		 		0x13
#define NCT7802_REG_FAN_MIN(index)     (0x4C + index)
#define NCT7802_REG_FAN_MIN_LSB(index) (0x49 + index)
#define NCT7802_FAN_LSB_MASK				0xF8
#define NCT7802_FAN_MIN_MSB_MASK			0xF8

#define NCT7802_REG_ALARM(index)		(0x18 + (index))

#define NCT7802_REG_PECI_ENABLE	0x23

#define NCT7802_REG_PECI_CTRL1	0x101
#define NCT7802_REG_PECI_CTRL3	0x103

#define DTS_READ        0
#define DTS_CRIT        1
#define DTS_HL        	2
#define DTS_LL   			3
#define DTS_LSB_MASK		0xC0



static u16 NCT7802_REG_DTS[][4] = {
	{0x6, 0x3E, 0x38, 0x39},
	{0x7, 0x3E, 0x38, 0x39},
};
#define NCT7802_REG_DTS_LSB	0x8
#define NCT7802_REG_DTS_MASK	0xC0
#define NCT7802_REG_DTS_SHIFT	6

static inline u16 IN_FROM_REG(u8 index, u16 val)
{
	return (val * IN_MULTIPLIER[index]);
}

static inline u16 IN_TO_REG(u8 index, u16 val)
{
	return (val / IN_MULTIPLIER[index]);
}

static inline unsigned long FAN_FROM_REG(u16 val)
{
	if ((val >= 0x1fff) || (val == 0))
		return	0;
	return (1350000UL / val);
}

static inline u16 FAN_TO_REG(long rpm)
{
	if (rpm <= 0)
		return (u16)0x1fff;
	return SENSORS_LIMIT((1350000UL + (rpm >> 1)) / rpm, 1, 0x1fffUL);
}

/*
static inline unsigned long TIME_FROM_REG(u8 reg)
{
	return (reg * 100);
}

static inline u8 TIME_TO_REG(unsigned long val)
{
	return SENSORS_LIMIT((val + 50) / 100, 0, 0xff);
}
*/

/* For 8-bit 2's complement integer portion */
static inline long TEMP_FROM_REG(s8 reg)
{
	return (reg * 1000);
}

/* For limitation */
static inline s8 TEMP_TO_REG(long val, s8 min, s8 max)
{
	return SENSORS_LIMIT(val / 1000, min, max);
}

enum chip_types {nct7802y};

static struct nct7802_data *g_data; //ASUS_BSP : set global variable
struct nct7802_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	unsigned long last_updated;	/* In jiffies */
	enum chip_types chip_type; /* For recording what the chip is */ 

	u8 bank;

	u32 has_in;    /* Enable monitor VIN or not. VCC, VSEN1~3, VCore */
	u16 in[5][3];		/* Complete format, not raw data. [VCC, VSEN1~3, VCore][read/high/low]. Completed voltage format, now raw data */

	u16 has_fan;		/* Enable fan1-3 */
	u16 fan[3];		   /* Completed FAN count format, not raw reg value */
	u16 fan_min[3];	/* Completed FAN count format, not raw reg value */

	u8 has_temp;      /* Enable monitor RTD1-3 and LTD or not */
	u8 temp[4][4];		/* Reg raw data. [RTD1-3,LTD][current, crit, high limit, low limit] */
	u8 temp_read_lsb[3]; /*  Reg raw data.  **LTD has NO lsb value.** The LSB value corresponded to temp[][TEMP_READ]. */
	u8 temp_mode;		/* 0: TR mode, 1: TD mode */

	u8 enable_dts;    /* Enable PECI and SB-TSI, 
	* bit 0: =1 enable, =0 disable , 
	* bit 1: =1 AMD SB-TSI, =0 Intel PECI */
	u8 has_dts;      /* Enable monitor DTS temp: DTS1-2 */
	u8 dts[2][4];       /*  Reg raw data. [DTS1-2][current, crit, high limit, low limit] */
	u8 dts_read_lsb[2];  /*  Reg raw data. The LSB value corresponded to dts[][DTS_READ] */

 
	u8 alarms[3];     /* Raw register value */

	unsigned int enable_pin;
	bool enable_fan;
	int pwm_reg_value;

	struct notifier_block notifier;  //register framebuffer notify
	char valid;
};

static u8 nct7802_read_value(struct i2c_client *client, u16 reg);
static int nct7802_write_value(struct i2c_client *client, u16 reg, u8 value);
static int nct7802_probe(struct i2c_client *client,
			const struct i2c_device_id *id);
static int nct7802_detect(struct i2c_client *client,
			 struct i2c_board_info *info);
static int nct7802_remove(struct i2c_client *client);

static void nct7802_init_client(struct i2c_client *client);
static struct nct7802_data *nct7802_update_device(struct device *dev);


static int reg_write_to_chip(struct i2c_client *client, u8 reg, u8 data, int len);

static ssize_t write_to_reg(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t inbox_user_fan(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t inbox_thermal_fan(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t dt_user_fan(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t dt_thermal_fan(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);

static int reg_read_from_chip(struct i2c_client *client, u8 reg, int len, char *data);
static ssize_t show_reg(struct device *dev, struct device_attribute *attr, char *buf);

static const struct i2c_device_id nct7802_id[] = {
	{ "nct7802", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, nct7802_id);



static const struct of_device_id nct7802_match_table[] = {
	{ .compatible = "nct7802",},
	{ },
};

static struct i2c_driver nct7802_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		   .name = "nct7802",
		   .owner = THIS_MODULE,
		   .of_match_table = nct7802_match_table,
	},
	.probe		= nct7802_probe,
	.remove		= nct7802_remove,
	.id_table	= nct7802_id,
	.detect		= nct7802_detect,
	.address_list	= normal_i2c,
};

#define ALARM_STATUS      0
//#define BEEP_ENABLE       1
#define ALARM_DTS_STATUS	2
#if 0
static ssize_t
show_alarm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct nct7802_data *data = nct7802_update_device(dev);
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	//int nr = sensor_attr->nr;
	int index = sensor_attr->index;
	u8 val=0;
	u8 tmp,tmp2;

	if (index<16){ //temp and peci
		tmp = (data->alarms[0]>>index) & 0x1; //below low limit
		tmp2 = (data->alarms[1]>>index) & 0x1; //over high limit
		val = (tmp | tmp2) & 0x1;
	}
	else if (index >= 16){ //FAN
		val = (data->alarms[2] >> (index-16)) & 0x1;
	}

	//nct7802 has no vin alarm

	return sprintf(buf, "%u\n", val);
}
#endif

#define FAN_INPUT     0
#define FAN_MIN       1
#if 0
static ssize_t
store_fan_min(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int index = sensor_attr->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7802_data *data = i2c_get_clientdata(client);
	
	u16 val = FAN_TO_REG(simple_strtoul(buf, NULL, 10));

	mutex_lock(&data->update_lock);
	data->fan_min[index] = val;
	nct7802_write_value(client, NCT7802_REG_FAN_MIN(index),
			   (val >> 5) & NCT7802_FAN_MIN_MSB_MASK);

	nct7802_write_value(client, NCT7802_REG_FAN_MIN_LSB(index), val & 0xFF);
	mutex_unlock(&data->update_lock);

	return count;
}
#endif

static ssize_t
show_fan(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int nr = sensor_attr->nr;
	int index = sensor_attr->index;
	struct nct7802_data *data = nct7802_update_device(dev);
	u16 val;

	if (FAN_INPUT == nr) {
		val = data->fan[index] & 0x1fff;
	} else {
		val = data->fan_min[index] & 0x1fff;
	}

	return sprintf(buf, "%lu\n", FAN_FROM_REG(val));
}

static ssize_t
show_vdd(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = 0;
	if(fan_current_type==3){
	    printk("[FAN] fan_current_type=3 , always return 1\n");
		val=1;	
	}else{
		if ( gpio_is_valid(g_data->enable_pin) ) {
			val = gpio_get_value(g_data->enable_pin);
		}
	}
	printk("[FAN] show_vdd : %d\n", val);
	return sprintf(buf,"%d\n", val);
}

/* Only for storing integer portion */
#if 0
static ssize_t
show_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int nr = sensor_attr->nr;
	int index = sensor_attr->index;
	struct nct7802_data *data = nct7802_update_device(dev);
	long temp = TEMP_FROM_REG(data->temp[index][nr]);

	if ( (TEMP_READ == nr) && (index < 3) ){ //i==3 is LTD, it has NO decimal fraction
		temp += ((data->temp_read_lsb[index] & TEMP_LSB_MASK) >> 5) * 125;
	}

	return sprintf(buf, "%ld\n", temp);
}

static ssize_t
store_temp(struct device *dev, struct device_attribute *attr,
	   const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int nr = sensor_attr->nr;
	int index = sensor_attr->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7802_data *data = i2c_get_clientdata(client);
	long tmp = simple_strtol(buf, NULL, 10);

	mutex_lock(&data->update_lock);
	data->temp[index][nr] = TEMP_TO_REG(tmp, -128, 127);
	nct7802_write_value(client, NCT7802_REG_TEMP[index][nr],
			   data->temp[index][nr]);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t
show_dts_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7802_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int index = sensor_attr->index;
	u8 tmp;

	if (data->enable_dts == 0)
		return sprintf(buf, "%d\n", 0);
	
	if ((data->has_dts >> index) & 0x01) {
		if (data->enable_dts & 2)
			tmp = 5; //TSI
		else
			tmp = 6; //PECI
	} else {
		tmp = 0;
	}

	return sprintf(buf, "%d\n", tmp);
}

static ssize_t
show_dts(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int nr = sensor_attr->nr;
	int index = sensor_attr->index;
	struct nct7802_data *data = nct7802_update_device(dev);
	long temp = TEMP_FROM_REG(data->dts[index][nr]);
	
	if (DTS_READ == nr){
		temp += ((data->dts_read_lsb[index] & DTS_LSB_MASK) >> 6) * 250;
	}

	return sprintf(buf, "%ld\n", temp);
}

/* Only for storing integer portion */
static ssize_t
store_dts(struct device *dev, struct device_attribute *attr,
	   const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int nr = sensor_attr->nr;
	int index = sensor_attr->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7802_data *data = i2c_get_clientdata(client);
	long tmp = simple_strtol(buf, NULL, 10);

	mutex_lock(&data->update_lock);
	data->dts[index][nr] = TEMP_TO_REG(tmp, -128, 127);
	nct7802_write_value(client, NCT7802_REG_DTS[index][nr],
			   data->dts[index][nr]);
	mutex_unlock(&data->update_lock);
	return count;
}

/*
	Type 3:  Thermal diode
   Type 4:  Thermistor
*/
static ssize_t
show_temp_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7802_data *data = i2c_get_clientdata(client);
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int index = sensor_attr->index;
	u8 tmp;

	if ((data->has_temp >> index) & 0x01) {
		if ((data->temp_mode >> index) & 0x01) {
			tmp = 3;	//TD			
		} else {
			tmp = 4;	//TR
		}
	} else {
		tmp = 0;
	}

	return sprintf(buf, "%d\n", tmp);
}

/* show/store VIN */
static ssize_t
show_in(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int nr = sensor_attr->nr;
	int index = sensor_attr->index;
	struct nct7802_data *data = nct7802_update_device(dev);
	u16 val = data->in[index][nr];
	

	val = IN_FROM_REG(index, val);

	return sprintf(buf, "%d\n", val);
}

/* For high/low limit. */
static ssize_t
store_in(struct device *dev, struct device_attribute *attr,
	 const char *buf, size_t count)
{
	struct sensor_device_attribute_2 *sensor_attr =
	    to_sensor_dev_attr_2(attr);
	int nr = sensor_attr->nr;
	int index = sensor_attr->index;
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7802_data *data = i2c_get_clientdata(client);
	u16 val = IN_TO_REG(index, simple_strtoul(buf, NULL, 10));
	u8 tmp;
	u8 tmp2;
	
	val = SENSORS_LIMIT(val, 0, 0x3FF);
	mutex_lock(&data->update_lock);

	
	switch (nr){
	case IN_MAX:
		// Write LSB
		nct7802_write_value(client, IN_HL_LSB_REG[index], val & 0xFF);
		// Write MSB
		tmp = ((val & 0x300) >> IN_HL_MSB_SHIFT[index]) & IN_HL_MSB_MASK[index];
		tmp2 = nct7802_read_value(client, NCT7802_REG_IN[index][nr]) & (~IN_HL_MSB_MASK[index]);
		tmp2 |= tmp;
		nct7802_write_value(client, NCT7802_REG_IN[index][nr], tmp2);

		break;
	case IN_LOW:
		// Write LSB
		nct7802_write_value(client, IN_LL_LSB_REG[index], val & 0xFF);
		// Write MSB
		tmp = ((val & 0x300) >> IN_LL_MSB_SHIFT[index]) & IN_LL_MSB_MASK[index];
		tmp2 = nct7802_read_value(client, NCT7802_REG_IN[index][nr]) & (~IN_LL_MSB_MASK[index]);
		tmp2 |= tmp;
		nct7802_write_value(client, NCT7802_REG_IN[index][nr], tmp2);
		
		break;
	default:
		break;
	}

	data->in[index][nr] = val; // Update For high/low limit

	mutex_unlock(&data->update_lock);
	return count;
}
#endif

#define NOT_USED			-1

#if 0
#define SENSOR_ATTR_IN(index)		\
	SENSOR_ATTR_2(in##index##_input, S_IRUGO, show_in, NULL,		\
		IN_READ, index), 														\
	SENSOR_ATTR_2(in##index##_max, S_IRUGO | S_IWUSR, show_in,	\
		store_in, IN_MAX, index),											\
	SENSOR_ATTR_2(in##index##_min, S_IRUGO | S_IWUSR, show_in,	\
		store_in, IN_LOW, index)											
//NCT7802 has no vin alarm
/*
	SENSOR_ATTR_2(in##index##_alarm, S_IRUGO, show_alarm,			\
		NULL, ALARM_STATUS,	index)
*/

// For NCT7802Y's VCORE
#define SENSOR_ATTR_IN_VCORE		\
	SENSOR_ATTR_2(in4_input, S_IRUGO, show_in, NULL,	\
		IN_READ, 4)
#endif

#define SENSOR_ATTR_FAN						\
	SENSOR_ATTR_2(VDD, S_IRUGO, show_vdd, NULL, NOT_USED, 0), \
	SENSOR_ATTR_2(debug_write_to_reg, S_IWUSR, NULL, write_to_reg, NOT_USED, 0), \
	SENSOR_ATTR_2(inbox_user_type, S_IWUSR, NULL, inbox_user_fan, NOT_USED, 0), \
	SENSOR_ATTR_2(inbox_thermal_type, S_IWUSR, NULL, inbox_thermal_fan, NOT_USED, 0), \
	SENSOR_ATTR_2(dt_user_type, S_IWUSR, NULL, dt_user_fan, NOT_USED, 0), \
	SENSOR_ATTR_2(dt_thermal_type, S_IWUSR, NULL, dt_thermal_fan, NOT_USED, 0), \
	SENSOR_ATTR_2(PWM, S_IRUGO , show_reg, NULL, NOT_USED, 0), \
	SENSOR_ATTR_2(RPM, S_IRUGO, show_fan, NULL, FAN_INPUT, 0)

#if 0
#define SENSOR_ATTR_FAN(index)						\
	SENSOR_ATTR_2(fan##index##_input, S_IRUGO, show_fan,		\
		NULL, FAN_INPUT, index - 1), \
	SENSOR_ATTR_2(fan##index##_min, S_IWUSR | S_IRUGO,		\
		show_fan, store_fan_min, FAN_MIN, index - 1),	\
	SENSOR_ATTR_2(fan##index##_alarm, S_IRUGO, show_alarm,	\
		NULL, ALARM_STATUS, index + 15)

#define SENSOR_ATTR_DTS(index)							\
	SENSOR_ATTR_2(temp##index##_type, S_IRUGO ,								\
		show_dts_mode, NULL, NOT_USED, index-5),								\
	SENSOR_ATTR_2(temp##index##_input, S_IRUGO, show_dts,					\
		NULL, DTS_READ, index-5),													\
	SENSOR_ATTR_2(temp##index##_crit, S_IRUGO | S_IWUSR, show_dts,		\
		store_dts, DTS_CRIT, index-5),											\
	SENSOR_ATTR_2(temp##index##_max, S_IRUGO | S_IWUSR, show_dts,		\
		store_dts, DTS_HL, index-5),												\
	SENSOR_ATTR_2(temp##index##_min, S_IRUGO | S_IWUSR, show_dts,		\
		store_dts, DTS_LL, index-5),												\
	SENSOR_ATTR_2(temp##index##_alarm, S_IRUGO,								\
		show_alarm, NULL, ALARM_DTS_STATUS, index - 1)
		
#define SENSOR_ATTR_TEMP(index)													\
	SENSOR_ATTR_2(temp##index##_type, S_IRUGO,								\
		show_temp_mode, NULL, NOT_USED, index - 1),							\
	SENSOR_ATTR_2(temp##index##_input, S_IRUGO, show_temp,				\
		NULL, TEMP_READ, index - 1),												\
	SENSOR_ATTR_2(temp##index##_crit, S_IRUGO | S_IWUSR, show_temp,	\
		store_temp, TEMP_CRIT, index - 1),										\
	SENSOR_ATTR_2(temp##index##_max, S_IRUGO | S_IWUSR, show_temp,		\
		store_temp, TEMP_HL, index - 1),											\
	SENSOR_ATTR_2(temp##index##_min, S_IRUGO | S_IWUSR, show_temp,		\
		store_temp, TEMP_LL, index - 1),											\
	SENSOR_ATTR_2(temp##index##_alarm, S_IRUGO,								\
		show_alarm, NULL, ALARM_STATUS, index - 1)
#endif

#if 0
static struct sensor_device_attribute_2 nct7802_in[] = {
	SENSOR_ATTR_IN(0),
	SENSOR_ATTR_IN(1),
	SENSOR_ATTR_IN(2),
	SENSOR_ATTR_IN(3),
};

// For NCT7802Y's VCORE
static struct sensor_device_attribute_2 nct7802_in_vcore[] = {
	SENSOR_ATTR_IN_VCORE,
};
#endif

static struct sensor_device_attribute_2 nct7802_fan[] = {
	SENSOR_ATTR_FAN,
};

#if 0
static struct sensor_device_attribute_2 nct7802_fan[] = {
	SENSOR_ATTR_FAN(1),
	SENSOR_ATTR_FAN(2),
	SENSOR_ATTR_FAN(3),
};

static struct sensor_device_attribute_2 nct7802_temp[] = {
	SENSOR_ATTR_TEMP(1),
	SENSOR_ATTR_TEMP(2),
	SENSOR_ATTR_TEMP(3),
	SENSOR_ATTR_TEMP(4),
};

static struct sensor_device_attribute_2 nct7802_dts[] = {
	SENSOR_ATTR_DTS(5),
	SENSOR_ATTR_DTS(6),
};
#endif

static void nct7802_init_client(struct i2c_client *client)
{

	u8 tmp;
	
	if (reset) {
		nct7802_write_value(client, NCT7802_REG_SOFTWARE_RESET, 0x80);
	}

	/* Start monitoring */
	tmp = nct7802_read_value(client, NCT7802_REG_START);
	tmp |= 0x1;
	nct7802_write_value(client, NCT7802_REG_SOFTWARE_RESET, tmp);
	
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int nct7802_detect(struct i2c_client *client,
			 struct i2c_board_info *info)
{
	u8 bank;
	struct i2c_adapter *adapter = client->adapter;
	//unsigned short address = client->addr;

	printk("[FAN] nct7802 detecting...\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		return -ENODEV;
	}
	bank = i2c_smbus_read_byte_data(client, NCT7802_REG_BANKSEL);

	/* Check Nuvoton vendor ID */
	if (0x50 != i2c_smbus_read_byte_data(client,
						NCT7802_REG_VENDORID)) {
		pr_debug("[FAN] Detection failed at check "
			 "vendor id\n");
		return -ENODEV;
	}


   
	if (0xC3 != i2c_smbus_read_byte_data(client,
		        NCT7802_REG_CHIPID)) {
			return -ENODEV;
	} 

	printk("[FAN] nct7802 is found.\n");

	/* Fill in the remaining client fields and put into the global list */
	strlcpy(info->type, "nct7802", I2C_NAME_SIZE);

	return 0;
}


static int nct7802_probe(struct i2c_client *client, 
			const struct i2c_device_id *id)
{
	int i, count_time;
	u8 tmp, tmp2;
	u16 u16tmp;
	struct device *dev = &client->dev;
	struct nct7802_data *data;
	struct device_node *np = dev->of_node;
	int err = 0;
	fan_current_type=gDongleType;

	printk("[FAN] nct7802_probe fan_current_type =%d +++ \n",fan_current_type);

//check i2c function
	for(count_time=1;count_time<=5;count_time++){
		if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
			if(count_time==5){
				err = -ENODEV;
				goto fan_remove;
			}
			printk("[FAN] I2C function test fail for %d time, retry\n", count_time);
			msleep(100);
		} else{
			printk("[FAN] I2C function test pass\n");
			break;
		}
}

	if (!(data = kzalloc(sizeof(struct nct7802_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}
	// +++ fan power on gpio +++ //
	//ASUS_SZ_BSP Cassie  add fan_current_type to diff jedi's inbox and DT+++
	if(fan_current_type!=3){
		data->enable_pin = of_get_named_gpio_flags(np, "inbox-fan,enable-gpio", 0, NULL);

		printk("[FAN] nct7802_probe : data->enable_pin=%d\n",data->enable_pin);
		if (data->enable_pin < 0)
			return data->enable_pin;

		if (gpio_request(data->enable_pin, "fan_enable")) {
			err = -EBUSY;
			printk("[FAN] [%s] fan_enable gpio request failed ! \n", __func__);
			goto fan_enble_gpio_failed;
		}
		printk("[FAN] nct7802_probe : config part\n");
		/* config part */
		err = gpio_direction_output(data->enable_pin, 1);
		if (err < 0) {
			printk("[FAN] [%s] gpio_direction_output enable failed ! \n", __func__);
			err = -EBUSY;
			goto fan_enble_gpio_failed;
		}
		printk("[FAN] nct7802_probe : data->enable_pin=%d, value=%d  \n",data->enable_pin,gpio_get_value(data->enable_pin));
		msleep(100); //Wait 0.1s for IC power on
		printk("[FAN] nct7802_probe : after IC power on \n");
		// --- fan power on gpio --- //
	}


	i2c_set_clientdata(client, data);
	data->bank = i2c_smbus_read_byte_data(client, NCT7802_REG_BANKSEL);
	mutex_init(&data->update_lock);

	/* Initialize the chip */
	nct7802_init_client(client);

	/* Check chip type: Only one package-48 pins. So assign it directly. */
	data->chip_type = nct7802y;

	//VOLTAGE SENSOR
	data->has_in = 0;
	tmp = nct7802_read_value(client, NCT7802_REG_VOLT_MONITOR_ENABLE);
	data->has_in |= tmp & 0x1;			 //VCC
	data->has_in |= (tmp & 0x2) << 3; //VCORE

	//VIN1~3	
	tmp = nct7802_read_value(client, NCT7802_REG_MODE_SELECTION);
	for (i=0; i<6; i+=2){
		if ( ((tmp >> i) & 0x3) == 0x3){
			data->has_in |= 1 << ((i/2)+1);
		}
	}

	//FAN
	data->has_fan = nct7802_read_value(client, NCT7802_REG_FAN_ENABLE);
	
	//TEMP1~3
	data->has_temp = 0;
	data->temp_mode = 0;
	tmp = nct7802_read_value(client, NCT7802_REG_MODE_SELECTION);

	/* Rule 1: set has_temp by TD mode for TD1 and TD2 */
	for (i=0; i<2; i++){
		tmp2 = (tmp >> (i*2)) & 0x3;
		if (tmp2== 0x1){
			data->has_temp |= 1 << i;
			
			data->temp_mode |=  1 << i; //Set as TD
		}
	}

	/* Rule 2: set has_temp by TR mode for TR3~1.
	 		 Notice: Any TR is enabled will disable TD1 
	 */
	for (i=2; i>=0; i--){
		tmp2 = (tmp >> (i*2)) & 0x3;
		if (tmp2 == 0x2){
			data->has_temp &= (~1);
			data->has_temp |= 1 << i;
			
			data->temp_mode &=  ~(1 << i); //Set as TR
		}
	}

	/* Rule 3: TD2 and TR3 are enabled at same time, it does not the reasonable setting.
	        We disable sensor's attribute because we can't know which one is correct. 
	 */
	if ( ((tmp&0xC)==0x4) && ((tmp&0x30)==20)){
		data->has_temp &= (~0x6);
	}
	
	/* LTD */
	if (tmp & 0x40){
		data->has_temp |= 0x8;
		data->temp_mode |= 0x8;
	}

	/* PECI */
	tmp = nct7802_read_value(client, NCT7802_REG_PECI_ENABLE);
	tmp2 = nct7802_read_value(client, NCT7802_REG_PECI_CTRL3);
	data->has_dts = (tmp & 0x3) & ((tmp2 & 0x30) >> 4);
	
	tmp = nct7802_read_value(client, NCT7802_REG_PECI_CTRL1); //PECI_En
	if (((tmp & 0x80) == 0) || ((tmp & 0x70) == 0) || ((tmp & 0x3) == 0)){
		data->has_dts = 0;
	}

	if (data->has_dts) {
		data->enable_dts = 1;	//bit[1:0]=1 => Enable DTS & PECI
										//bit[1:0]=3 => Enable DTS & TSI
	}
	else{
		data->enable_dts = 0;
	}

	/* First time to update the voltages measured value and limits */
	for (i = 0; i < ARRAY_SIZE(data->in); i++) {
		if (!(data->has_in & (1 << i))) {
			continue;
		}

		u16tmp = (nct7802_read_value(client, NCT7802_REG_IN[i][IN_MAX]) & IN_HL_MSB_MASK[i]) << IN_HL_MSB_SHIFT[i];
		u16tmp |= nct7802_read_value(client, IN_HL_LSB_REG[i]);
		data->in[i][IN_MAX] = u16tmp;

		u16tmp = (nct7802_read_value(client, NCT7802_REG_IN[i][IN_LOW]) & IN_LL_MSB_MASK[i]) << IN_LL_MSB_SHIFT[i];
		u16tmp |= nct7802_read_value(client, IN_LL_LSB_REG[i]);
		data->in[i][IN_LOW] = u16tmp;
		
		u16tmp = nct7802_read_value(client, NCT7802_REG_IN[i][IN_READ]) << 2;
		u16tmp |= (nct7802_read_value(client, IN_LSB_REG) & IN_LSB_MASK) >> 6;
		data->in[i][IN_READ] = u16tmp;
	}

	//VCORE Reading. (VCORE has NO high/low limit
	u16tmp = (nct7802_read_value(client, NCT7802_REG_IN_VCORE)) << 2;
	u16tmp |= (nct7802_read_value(client, IN_LSB_REG) & IN_LSB_MASK) >> 6;
	data->in[4][IN_READ] = u16tmp;

	/* First time to update fan and limits */
	for (i = 0; i < ARRAY_SIZE(data->fan); i++) {
		if (!(data->has_fan & (1 << i))) {
			continue;
		}
		data->fan_min[i] =
			(((u16)nct7802_read_value(client, NCT7802_REG_FAN_MIN(i))) & NCT7802_FAN_MIN_MSB_MASK) << 5;
		data->fan_min[i] |=
		  nct7802_read_value(client, NCT7802_REG_FAN_MIN_LSB(i));
		data->fan[i] =
			((u16)nct7802_read_value(client, NCT7802_REG_FAN(i))) << 5;
		data->fan[i] |=
		  (nct7802_read_value(client, NCT7802_REG_FAN_LSB) & NCT7802_FAN_LSB_MASK) >> 3;
	}

	/* First time to update temperature and limits */
	for (i = 0; i < ARRAY_SIZE(data->temp); i++) {
		if (!(data->has_temp & (1 << i)))
			continue;
		data->temp[i][TEMP_CRIT] = 
			nct7802_read_value(client, NCT7802_REG_TEMP[i][TEMP_CRIT]);
		data->temp[i][TEMP_HL] = 
			nct7802_read_value(client, NCT7802_REG_TEMP[i][TEMP_HL]);
		data->temp[i][TEMP_LL] = 
			nct7802_read_value(client, NCT7802_REG_TEMP[i][TEMP_LL]);
		data->temp[i][TEMP_READ] = 
			nct7802_read_value(client, NCT7802_REG_TEMP[i][TEMP_READ]);
		if (i < 3){ // Don't include LTD, it has no lsb.
			data->temp_read_lsb[i] =
				nct7802_read_value(client, NCT7802_REG_TEMP_LSB);
		}
	}

	/* dts temperature and limits */
	if (data->enable_dts != 0) {
		for (i = 0; i < ARRAY_SIZE(data->dts); i++) {
			data->dts[i][DTS_CRIT] = 
				nct7802_read_value(client, NCT7802_REG_DTS[i][DTS_CRIT]);
			data->dts[i][DTS_HL] = 
				nct7802_read_value(client, NCT7802_REG_DTS[i][DTS_HL]);
			data->dts[i][DTS_LL] = 
				nct7802_read_value(client, NCT7802_REG_DTS[i][DTS_LL]);
			
			if (!(data->has_dts & (1 << i)))
				continue;
			
			data->dts[i][DTS_READ] = 
				nct7802_read_value(client, NCT7802_REG_DTS[i][DTS_READ]);
			data->dts_read_lsb[i] =
				nct7802_read_value(client, NCT7802_REG_DTS_LSB);
			
		}
	}
	
	/* alarm */
	for (i = 0; i < ARRAY_SIZE(data->alarms); i ++) {
		data->alarms[i] = 
			nct7802_read_value(client, NCT7802_REG_ALARM(i));

	}

	printk("[FAN] nct7802_probe : start device_create_file\n");

#if 0
	/* Register sysfs hooks */
	for (i = 0; i < ARRAY_SIZE(nct7802_in); i++) {
		if (!(data->has_in & (1 << (i / 3)))) 
			continue;
		err = device_create_file(dev, &nct7802_in[i].dev_attr);
		if (err)
			goto exit_remove;
	}

	//VCORE
	if (data->has_in & 0x10){
		err = device_create_file(dev, &nct7802_in_vcore[0].dev_attr);
		if (err)
			goto exit_remove;
	}
#endif

	for (i = 0; i < ARRAY_SIZE(nct7802_fan); i++) {
		if (!(data->has_fan & (1 << (i / 8))))
			continue;
		err = device_create_file(dev, &nct7802_fan[i].dev_attr);
		if (err)
			goto exit_remove;
	}

#if 0
	for (i = 0; i < ARRAY_SIZE(nct7802_temp); i++) {
		if (!(data->has_temp & (1 << (i/6))))
			continue;
		err = device_create_file(dev, &nct7802_temp[i].dev_attr);
		if (err)
			goto exit_remove;
	}

	if (data->enable_dts != 0) {
		for (i = 0; i < ARRAY_SIZE(nct7802_dts); i++) {
			if (!(data->has_dts & (1 << (i / 6))))
				continue;
			err = device_create_file(dev, &nct7802_dts[i].dev_attr);
			if (err)
				goto exit_remove;
		}
	}
#endif

/*
	data->hwmon_dev = hwmon_device_register(dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}
*/
	data->hwmon_dev = hwmon_device_register_with_groups(dev, "Inbox_Fan", NULL, NULL);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}


	g_data = data; //ASUS_BSP : set global variable
	printk("[FAN] nct7802_probe ---\n");
	return 0;

	/* Unregister sysfs hooks */
exit_remove:
	printk("[FAN] nct7802_probe fail.\n");
#if 0

	for (i = 0; i < ARRAY_SIZE(nct7802_in); i++)
		device_remove_file(dev, &nct7802_in[i].dev_attr);

	for (i = 0; i < ARRAY_SIZE(nct7802_in_vcore); i++)
		device_remove_file(dev, &nct7802_in_vcore[i].dev_attr);
#endif


	for (i = 0; i < ARRAY_SIZE(nct7802_fan); i++)
		device_remove_file(dev, &nct7802_fan[i].dev_attr);

#if 0

	for (i = 0; i < ARRAY_SIZE(nct7802_temp); i++){
		device_remove_file(dev, &nct7802_temp[i].dev_attr);
	}

	for (i = 0; i < ARRAY_SIZE(nct7802_dts); i++)
		device_remove_file(dev, &nct7802_dts[i].dev_attr);
#endif

	kfree(data);

fan_enble_gpio_failed:
	gpio_free(data->enable_pin);
fan_remove:
	printk("[FAN] DT_Fan probe fail !!!\n");
	return err;

exit:
	return err;
}

static struct nct7802_data *nct7802_update_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct7802_data *data = i2c_get_clientdata(client);
	u16 u16tmp;
	int i;

	mutex_lock(&data->update_lock);

	if (!(time_after(jiffies, data->last_updated + HZ * 2)
	      || !data->valid))
		goto END;

	/* First time to update the voltages measured value and limits */
	for (i = 0; i < ARRAY_SIZE(data->in); i++) {
		if (!(data->has_in & (1 << i))) {
			continue;
		}

		u16tmp = (nct7802_read_value(client, NCT7802_REG_IN[i][IN_MAX]) & IN_HL_MSB_MASK[i]) << IN_HL_MSB_SHIFT[i];
		u16tmp |= nct7802_read_value(client, IN_HL_LSB_REG[i]);
		data->in[i][IN_MAX] = u16tmp;

		u16tmp = (nct7802_read_value(client, NCT7802_REG_IN[i][IN_LOW]) & IN_LL_MSB_MASK[i]) << IN_LL_MSB_SHIFT[i];
		u16tmp |= nct7802_read_value(client, IN_LL_LSB_REG[i]);
		data->in[i][IN_LOW] = u16tmp;
		
		u16tmp = nct7802_read_value(client, NCT7802_REG_IN[i][IN_READ]) << 2;
		u16tmp |= (nct7802_read_value(client, IN_LSB_REG) & IN_LSB_MASK) >> 6;
		data->in[i][IN_READ] = u16tmp;
	}

	//VCORE Reading. (VCORE has NO high/low limit
	u16tmp = (nct7802_read_value(client, NCT7802_REG_IN_VCORE)) << 2;
	u16tmp |= (nct7802_read_value(client, IN_LSB_REG) & IN_LSB_MASK) >> 6;
	data->in[4][IN_READ] = u16tmp;


	/* First time to update fan and limits */
	for (i = 0; i < ARRAY_SIZE(data->fan); i++) {
		if (!(data->has_fan & (1 << i))) {
			continue;
		}
		data->fan_min[i] =
			(((u16)nct7802_read_value(client, NCT7802_REG_FAN_MIN(i))) & NCT7802_FAN_MIN_MSB_MASK) << 5;
		data->fan_min[i] |=
		  nct7802_read_value(client, NCT7802_REG_FAN_MIN_LSB(i));
		data->fan[i] =
			((u16)nct7802_read_value(client, NCT7802_REG_FAN(i))) << 5;
		data->fan[i] |=
		  (nct7802_read_value(client, NCT7802_REG_FAN_LSB) & NCT7802_FAN_LSB_MASK) >> 3;
	}


	/* First time to update temperature and limits */
	for (i = 0; i < ARRAY_SIZE(data->temp); i++) {
		if (!(data->has_temp & (1 << i)))
			continue;
		data->temp[i][TEMP_CRIT] = 
			nct7802_read_value(client, NCT7802_REG_TEMP[i][TEMP_CRIT]);
		data->temp[i][TEMP_HL] = 
			nct7802_read_value(client, NCT7802_REG_TEMP[i][TEMP_HL]);
		data->temp[i][TEMP_LL] = 
			nct7802_read_value(client, NCT7802_REG_TEMP[i][TEMP_LL]);
		data->temp[i][TEMP_READ] = 
			nct7802_read_value(client, NCT7802_REG_TEMP[i][TEMP_READ]);
		if (i != 3){ // Not LTD
			data->temp_read_lsb[i] =
				nct7802_read_value(client, NCT7802_REG_TEMP_LSB);
		}
	}


	/* dts temperature and limits */
	if (data->enable_dts != 0) {
		for (i = 0; i < ARRAY_SIZE(data->dts); i++) {
			data->dts[i][DTS_CRIT] = 
				nct7802_read_value(client, NCT7802_REG_DTS[i][DTS_CRIT]);
			data->dts[i][DTS_HL] = 
				nct7802_read_value(client, NCT7802_REG_DTS[i][DTS_HL]);
			data->dts[i][DTS_LL] = 
				nct7802_read_value(client, NCT7802_REG_DTS[i][DTS_LL]);
			
			if (!(data->has_dts & (1 << i)))
				continue;
			
			data->dts[i][DTS_READ] = 
				nct7802_read_value(client, NCT7802_REG_DTS[i][DTS_READ]);
			data->dts_read_lsb[i] =
				nct7802_read_value(client, NCT7802_REG_DTS_LSB);
			
		}
	}

	
	/* alarm */
	for (i = 0; i < ARRAY_SIZE(data->alarms); i++) {
		data->alarms[i] = 
			nct7802_read_value(client, NCT7802_REG_ALARM(i));

	}

	data->last_updated = jiffies;
	data->valid = 1;

END:
	mutex_unlock(&data->update_lock);
	return data;
}

static int nct7802_remove(struct i2c_client *client)
{
	struct nct7802_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;
	int i,err = 0;

	printk("[FAN] nct7802_remove +++\n");
	hwmon_device_unregister(data->hwmon_dev);
#if 0
	for (i = 0; i < ARRAY_SIZE(nct7802_in); i++)
		device_remove_file(dev, &nct7802_in[i].dev_attr);

	for (i = 0; i < ARRAY_SIZE(nct7802_in_vcore); i++)
		device_remove_file(dev, &nct7802_in_vcore[i].dev_attr);
#endif

	for (i = 0; i < ARRAY_SIZE(nct7802_fan); i++)
		device_remove_file(dev, &nct7802_fan[i].dev_attr);

#if 0
	for (i = 0; i < ARRAY_SIZE(nct7802_temp); i++){
		device_remove_file(dev, &nct7802_temp[i].dev_attr);
	}

	for (i = 0; i < ARRAY_SIZE(nct7802_dts); i++)
		device_remove_file(dev, &nct7802_dts[i].dev_attr);
#endif
	//Switch to bank 0 as default, because chip ID only can be obtained by bank 0.
	//i2c_smbus_write_byte_data(client, NCT7802_REG_BANKSEL, 0);
	printk("[FAN] nct7802_remove, fan_current_type= %d \n",fan_current_type);
	if(fan_current_type!=3){ //Don't do this to dt.
		// +++ fan power off gpio +++ //
		err = gpio_direction_output(data->enable_pin, 0);
		if (err < 0)
			printk("[FAN] [%s] gpio_direction_output disable failed ! \n", __func__);

		gpio_free(data->enable_pin);
		printk("[FAN] [%s] free gpio = %d \n", __func__,data->enable_pin);
		// --- fan power off gpio --- //
	}

	kfree(data);
	printk("[FAN] nct7802_remove ---\n");
	return 0;
}

/* Ignore the possibility that somebody change bank outside the driver
   Must be called with data->update_lock held, except during initialization */
static u8 nct7802_read_value(struct i2c_client *client, u16 reg)
{
	struct nct7802_data *data = i2c_get_clientdata(client);
	u8 res = 0xff;
	u8 new_bank = reg >> 8;

	new_bank |= data->bank & 0xFE;

	if (data->bank != new_bank) {

		if (i2c_smbus_write_byte_data
		    (client, NCT7802_REG_BANKSEL, new_bank) >= 0){
			data->bank = new_bank;
		}
		else {
			dev_err(&client->dev,
				"set bank to %d failed, fall back "
				"to bank %d, read reg 0x%x error\n",
				new_bank, data->bank, reg);
			res = 0x0;	/* read 0x0 from the chip */
			goto END;
		}
	}


	res = i2c_smbus_read_byte_data(client, reg & 0xff);
END:
	return res;
}

/* Must be called with data->update_lock held, except during initialization */
static int nct7802_write_value(struct i2c_client *client, u16 reg, u8 value)
{
	struct nct7802_data *data = i2c_get_clientdata(client);
	int res;
	u8 new_bank = reg >> 8;


	new_bank |= data->bank & 0xFE;

	if (data->bank != new_bank) {

		if ((res = i2c_smbus_write_byte_data
		    (client, NCT7802_REG_BANKSEL, new_bank)) >= 0){
			data->bank = new_bank;
		}
		else {
			dev_err(&client->dev,
				"set bank to %d failed, fall back "
				"to bank %d, write reg 0x%x error\n",
				new_bank, data->bank, reg);
			goto END;
		}
	}

 

	res = i2c_smbus_write_byte_data(client, reg & 0xff, value);
END:
	return res;
}


static int reg_write_to_chip
	(struct i2c_client *client, u8 reg, u8 data, int len)
{
	int err = 0;
	//uint8_t buf[len+1];
	uint8_t buf[4];

	static struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len+1;
	msg.buf = buf;

	buf[0] = reg;
	memcpy(buf + 1, &data, sizeof(data));

	err = i2c_transfer(client->adapter, &msg, len);

	if (err < 0)
		pr_err("%s: reg=0x%x, data_=%d, err = 0x%x\n",
			__func__, reg, data, err);

	return err;
}
//+++write to reg
static ssize_t write_to_reg(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int reg_addr = 0, reg_value = 0;

	sscanf(buf, "%x %x", &reg_addr, &reg_value);
	printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
	reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);

	return size;
}
//---write to reg

//+++inbox user fan
static ssize_t inbox_user_fan(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	/*int reg_addr = 0, reg_value = 0;

	sscanf(buf, "%x %x", &reg_addr, &reg_value);
	printk("[INBOX_FAN] : reg_addr=%x, reg_value=%x\n",reg_addr,reg_value);
	reg_write_to_chip(to_i2c_client(dev), reg_addr,
				reg_value, 1);*/

	int num = 99, reg_addr = 0, reg_value = 0;

	mutex_lock(&g_data->update_lock);
	sscanf(buf, "%d", &num);
	printk("[FAN] %s : num=%d\n",__func__, num);
	//ASUSEvtlog("[FAN] %s : fan_type=%d\n",__func__, num);
	if ((num != 0) && (gpio_get_value(g_data->enable_pin) != 1)) {
		printk("[FAN] %s : enable fan\n",__func__);
		if ( gpio_is_valid(g_data->enable_pin) ) {
			gpio_set_value(g_data->enable_pin, 1);
			printk("[FAN]: %s : g_data->enable_pin=%d, value=%d  \n",__func__,g_data->enable_pin,gpio_get_value(g_data->enable_pin));
		}
		msleep(100); //Wait 0.1s for IC power on

	}
	else {
		printk("[FAN] %s : disable fan or enable_pin is already set \n",__func__);
	}
	switch (num) {
		case 0:
			printk("[FAN] %s : fan_type 0: close +++\n",__func__);

			sscanf("59 0", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			printk("[FAN] %s : fan_type 0: close ---\n",__func__);
			if ( gpio_is_valid(g_data->enable_pin) ) {
				gpio_set_value(g_data->enable_pin, 0);
				printk("[FAN]: %s : g_data->enable_pin=%d, value=%d  \n",__func__,g_data->enable_pin,gpio_get_value(g_data->enable_pin));
			}

			break;
		case 1:
			printk("[FAN] %s : fan_type 1: low +++\n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 4B", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			printk("[FAN] %s : fan_type 1: low ---\n",__func__);
			break;
		case 2:
			printk("[FAN] %s : fan_type 2: medium +++\n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 65", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			printk("[FAN] %s : fan_type 2: medium ---\n",__func__);
			break;
		case 3:
			printk("[FAN] %s : fan_type 3: high +++\n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 7F", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			printk("[FAN] %s : fan_type 3: high ---\n",__func__);
			break;
		case 4:
			printk("[FAN] %s : fan_type 4: turbo +++\n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 AC", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			printk("[FAN] %s : fan_type 4: turbo ---\n",__func__);
			break;
	}
	msleep(50); //Wait 0.05s
	mutex_unlock(&g_data->update_lock);

	return size;
}
//---inbox user fan

//+++inbox thermal fan 
static ssize_t inbox_thermal_fan(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	/*int reg_addr = 0, reg_value = 0;

	sscanf(buf, "%x %x", &reg_addr, &reg_value);
	printk("[INBOX_FAN] : reg_addr=%x, reg_value=%x\n",reg_addr,reg_value);
	reg_write_to_chip(to_i2c_client(dev), reg_addr,
				reg_value, 1);*/

	int num = 99, reg_addr = 0, reg_value = 0;

	mutex_lock(&g_data->update_lock);
	sscanf(buf, "%d", &num);
	printk("[FAN] %s : num=%d\n",__func__, num);
	//ASUSEvtlog("[FAN] %s : fan_type=%d\n",__func__, num);
	if ((num != 0) && (gpio_get_value(g_data->enable_pin) != 1)) {
		printk("[FAN] %s : enable fan\n",__func__);
		if ( gpio_is_valid(g_data->enable_pin) ) {
			gpio_set_value(g_data->enable_pin, 1);
			printk("[FAN]: %s : g_data->enable_pin=%d, value=%d  \n",__func__,g_data->enable_pin,gpio_get_value(g_data->enable_pin));
		}
		msleep(100); //Wait 0.1s for IC power on

	}
	else {
		printk("[FAN] %s : disable fan or enable_pin is already set \n",__func__);
	}
	switch (num) {
		case 0:
			printk("[FAN] %s : thermal_type 0: default 1 (low) +++\n",__func__); //thermal policy send 0 will be set 1 (low)

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 4B", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			printk("[FAN] %s : thermal_type 0: default 1 (low) ---\n",__func__);
			break;
		case 1:
			printk("[FAN] %s : thermal_type 1: low +++\n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 4B", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			printk("[FAN] %s : thermal_type 1: low ---\n",__func__);
			break;
		case 2:
			printk("[FAN] %s : thermal_type 2: medium +++\n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 65", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			printk("[FAN] %s : thermal_type 2: medium ---\n",__func__);
			break;
		case 3:
			printk("[FAN] %s : thermal_type 3: high +++\n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 7F", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			printk("[FAN] %s : thermal_type 3: high ---\n",__func__);
			break;
		case 4:
			printk("[FAN] %s : thermal_type 4: turbo +++\n",__func__); //thermal policy send 4 will be set 3 (high)

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 7F", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			printk("[FAN] %s : thermal_type 4: turbo ---\n",__func__);
			break;
	}
	msleep(50); //Wait 0.05s
	mutex_unlock(&g_data->update_lock);

	return size;
}
//---inbox thermal fan

//+++ dt user fan
static ssize_t dt_user_fan(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	/*int reg_addr = 0, reg_value = 0;

	sscanf(buf, "%x %x", &reg_addr, &reg_value);
	printk("[DT_FAN] : reg_addr=%x, reg_value=%x\n",reg_addr,reg_value);
	reg_write_to_chip(to_i2c_client(dev), reg_addr,
				reg_value, 1);*/

	int num = 99, reg_addr = 0, reg_value = 0;

	mutex_lock(&g_data->update_lock);
	sscanf(buf, "%d", &num);
	//printk("[FAN] %s : num=%d\n",__func__, num);
	//ASUSEvtlog("[FAN] %s : fan_type=%d\n",__func__, num);
/*	if ((num != 0) && (gpio_get_value(g_data->enable_pin) != 1)) {
		printk("[FAN] %s : enable fan\n",__func__);
		if ( gpio_is_valid(g_data->enable_pin) ) {
			gpio_set_value(g_data->enable_pin, 1);
			printk("[FAN]: %s : g_data->enable_pin=%d, value=%d  \n",__func__,g_data->enable_pin,gpio_get_value(g_data->enable_pin));
		}
		msleep(100); //Wait 0.1s for IC power on

	}*/
	/*else {
		printk("[FAN] %s : disable fan or enable_pin is already set \n",__func__);
	}*/
	switch (num) {
		case 0:
			printk("[FAN] %s : fan_type 0: close \n",__func__);

			sscanf("59 0", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			//printk("[FAN] %s : fan_type 0: close ---\n",__func__);
			/*if ( gpio_is_valid(g_data->enable_pin) ) {
				gpio_set_value(g_data->enable_pin, 0);
				printk("[FAN]: %s : g_data->enable_pin=%d, value=%d  \n",__func__,g_data->enable_pin,gpio_get_value(g_data->enable_pin));
			}*/
			break;
		case 1:
			printk("[FAN] %s : fan_type 1: low \n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 4B", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			//printk("[FAN] %s : fan_type 1: low ---\n",__func__);
			break;
		case 2:
			printk("[FAN] %s : fan_type 2: medium \n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 50", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			//printk("[FAN] %s : fan_type 2: medium ---\n",__func__);
			break;
		case 3:
			printk("[FAN] %s : fan_type 3: high \n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 55", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			//printk("[FAN] %s : fan_type 3: high ---\n",__func__);
			break;
		case 4:
			printk("[FAN] %s : fan_type 4: turbo \n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 FF", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			//printk("[FAN] %s : fan_type 4: turbo ---\n",__func__);
			break;
	}
	msleep(500); //Wait 0.5s
	mutex_unlock(&g_data->update_lock);

	return size;
}
//--- dt user fan

//+++ dt thermal fan
static ssize_t dt_thermal_fan(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	/*int reg_addr = 0, reg_value = 0;

	sscanf(buf, "%x %x", &reg_addr, &reg_value);
	printk("[DT_FAN] : reg_addr=%x, reg_value=%x\n",reg_addr,reg_value);
	reg_write_to_chip(to_i2c_client(dev), reg_addr,
				reg_value, 1);*/

	int num = 99, reg_addr = 0, reg_value = 0;

	mutex_lock(&g_data->update_lock);
	sscanf(buf, "%d", &num);
	//printk("[FAN] %s : num=%d\n",__func__, num);
	//ASUSEvtlog("[FAN] %s : fan_type=%d\n",__func__, num);
	/*if ((num != 0) && (gpio_get_value(g_data->enable_pin) != 1)) {
		printk("[FAN] %s : enable fan\n",__func__);
		if ( gpio_is_valid(g_data->enable_pin) ) {
			gpio_set_value(g_data->enable_pin, 1);
			printk("[FAN]: %s : g_data->enable_pin=%d, value=%d  \n",__func__,g_data->enable_pin,gpio_get_value(g_data->enable_pin));
		}
		msleep(100); //Wait 0.1s for IC power on

	}
	else {
		printk("[FAN] %s : disable fan or enable_pin is already set \n",__func__);
	//}*/
	switch (num) {
		case 0:
			printk("[FAN] %s : thermal_type 0: default 1 (low) \n",__func__); //thermal policy send 4 will be set 1 (low)

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 4B", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			//printk("[FAN] %s : thermal_type 0: default 1 (low) ---\n",__func__);
			break;
		case 1:
			printk("[FAN] %s : thermal_type 1: low\n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 4B", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			//printk("[FAN] %s : thermal_type 1: low ---\n",__func__);
			break;
		case 2:
			printk("[FAN] %s : thermal_type 2: medium\n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 50", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			//printk("[FAN] %s : thermal_type 2: medium ---\n",__func__);
			break;
		case 3:
			printk("[FAN] %s : thermal_type 3: high \n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 55", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			//printk("[FAN] %s : thermal_type 3: high ---\n",__func__);
			break;
		case 4:
			printk("[FAN] %s : thermal_type 4: turbo \n",__func__);

			sscanf("59 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 00", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("70 FF", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 14", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("EE 10", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s

			sscanf("24 01", "%x %x", &reg_addr, &reg_value);
			//printk("[FAN] %s : reg_addr=%x, reg_value=%x\n",__func__,reg_addr,reg_value);
			reg_write_to_chip(to_i2c_client(dev), reg_addr,reg_value, 1);
			msleep(3); //Wait 0.003s
			//printk("[FAN] %s : thermal_type 4: turbo ---\n",__func__);
			break;
	}
	msleep(500); //Wait 0.5s
	mutex_unlock(&g_data->update_lock);

	return size;
}
//--- dt thermal fan


static int reg_read_from_chip
	(struct i2c_client *client, u8 reg, int len, char *data)
{
	int err = 0;

	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = I2C_M_NOSTART,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		}
	};

	if (!client->adapter)
		return -ENODEV;

	memset(data, 0, sizeof(*data)*len);

	err = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));

	if (err != ARRAY_SIZE(msg))
		pr_err("%s: err %d\n", __func__, err);

	return err;
}

static ssize_t show_reg(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char *reg_value;
	int len;
	char *outtmp;
	u8 reg_addr = 0x60;
	int reg_read_len = 1;

	if (reg_read_len == 0)
		return 0;
	outtmp = kzalloc(sizeof(char)*20, GFP_KERNEL);

	reg_value = kzalloc(sizeof(char)*reg_read_len,
					GFP_KERNEL);

	reg_read_from_chip(to_i2c_client(dev),
		reg_addr, reg_read_len, reg_value);

	//sprintf(outtmp, "0x%X", (int)reg_value);
	
	for (len = 0; len < reg_read_len; len++) {
		sprintf(outtmp, "%X", (int)reg_value[len]);
	}
	//strcat(outtmp, "\n");
	printk("[FAN]show_reg : pwm=0x%s\n", outtmp);
	//ASUSEvtlog("[FAN] %s : outtmp=%s\n",__func__, outtmp);

	strcpy(buf, outtmp);
	kfree(outtmp);

	return strlen(buf);
}

static int __init sensors_nct7802_init(void)
{
	int ret;
	printk("[FAN] sensors_nct7802_init\n");

	ret=i2c_add_driver(&nct7802_driver);
	if (ret)
		printk("[FAN] nct7802 driver int failed.\n");
	else
		printk("[FAN] nct7802 driver int success.\n");

	return ret;
}

static void __exit sensors_nct7802_exit(void)
{
	printk("[FAN] sensors_nct7802_exit\n");

	i2c_del_driver(&nct7802_driver);
}

MODULE_AUTHOR("Sheng-Yuan Huang");
MODULE_DESCRIPTION("NCT7802 driver");
MODULE_LICENSE("GPL");

module_init(sensors_nct7802_init);
module_exit(sensors_nct7802_exit);
