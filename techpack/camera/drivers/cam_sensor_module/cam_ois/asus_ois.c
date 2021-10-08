#include <linux/proc_fs.h>
#include <linux/time.h>
#include "asus_ois.h"
#include "onsemi_interface.h"
#include "onsemi_i2c.h"
#include "utils.h"
#include "cam_eeprom_dev.h"
//#include "asus_cam_sensor.h"
#include <linux/delay.h>
#include "asus_actuator.h"
#undef  pr_fmt
#define pr_fmt(fmt) "OIS-ATD %s(): " fmt, __func__

//OIS PROC DRIVER NODE +++
//ASUS_BSP ASUS Factory use +++
#define	PROC_POWER	"driver/ois_power"
#define	PROC_I2C_RW	"driver/ois_i2c_rw"
#define	PROC_MODE	"driver/ois_mode"
#define	PROC_CALI	"driver/ois_cali"
#define PROC_RDATA  "driver/ois_rdata"
#define	PROC_ATD_STATUS	"driver/ois_atd_status"
#define PROC_VCM_ENABLE "driver/ois_vcm_enable" //for debug
#define	PROC_SMA_EEPROM_DUMP	"driver/ois_sma_eeprom_dump" 
//ASUS_BSP ASUS Factory use ---

#define PROC_ON     "driver/ois_on"
#define PROC_STATE "driver/ois_state"
#define	PROC_PROBE_STATUS "driver/ois_status"
#define	PROC_DEVICE	"driver/ois_device"
#define PROC_FW_UPDATE "driver/ois_fw_update"
#define	PROC_MODULE	"driver/ois_module" //ASUS_BSP Lucien +++: Add OIS SEMCO module
#define PROC_AF_ATATE  "driver/ois_af_state" //notify AF state
#define PROC_OIS_GET_LENS_INFO "driver/ois_i2c_rw_lens_info"
//OIS PROC DRIVER NODE ---
#define RDATA_OUTPUT_FILE "/sdcard/gyro.csv"
#define SMA_OFFSET_DUMP "/sdcard/sma_eeprom_dump"

#define FACTORYDIR "/vendor/factory/"

#define OIS_GYRO_K_OUTPUT_FILE_OLD ""FACTORYDIR"OIS_calibration_old"
#define OIS_GYRO_K_OUTPUT_FILE_NEW ""FACTORYDIR"OIS_calibration"

#define OIS_VCM_BACKUP ""FACTORYDIR"OIS_VCM_VERSION"
#define OIS_MODULE_SN  ""FACTORYDIR"OIS_MODULE_SN"
#define OIS_FW_UPDATE_TIMES ""FACTORYDIR"OIS_FW_UPDATE_TIMES"
#define OIS_FW_UPDATE_TIME_LIMIT 500

#define BATTERY_CAPACITY "/sys/class/power_supply/bms/capacity"
#define BATTERY_THRESHOLD 3

#define UPDATE_FW_AT_PROBE 1

typedef enum {
	CHECK_BAD_IO = -1,
	CHECK_PASS = 0,
	CHECK_BAD_ID,
	CHECK_BAD_FW,
	CHECK_BAD_FUNCTION,
	CHECK_VENDOR_MISMATCH,
	CHECK_VENDOR_INVALID,
	CHECK_CUSTOMER_INVALID,
	CHECK_VCM_INVALID,
	CHECK_FW_VERSION_INVALID,
}chip_check_result_t;

typedef enum{
	UPDATE_FAIL = -1,
	UPDATE_OK = 0,
	NO_NEED = 1,
	BATTERY_LOW,
	NO_MATCH_FW,
	VENDOR_INVALID,
	NO_VCM_BACK,
	EXCEED_LIMIT,
	BAD_IO,
	BAD_FW,
}fw_trigger_result_t;
#ifdef ZS670KS
extern uint8_t eeprom_camera_specs; //ASUS_BSP Byron take this tag for distinguish device level
#endif
static struct cam_ois_ctrl_t * ois_ctrl[OIS_CLIENT_MAX] = {0};

uint8_t g_ois_status[OIS_CLIENT_MAX] = {0};
char g_module_vendor[8] = "UNKNOWN";
uint32_t g_fw_version = 0;
uint8_t g_ois_mode = 255;//only after set mode, mode value is valid

uint8_t g_ois_power_state[OIS_CLIENT_MAX] = {0};
uint8_t g_ois_camera_open[OIS_CLIENT_MAX] = {0};

static uint8_t g_atd_status = 0;//fail

static uint16_t g_reg_addr = 0xF012;
static uint32_t g_reg_val = 0;
static uint16_t g_slave_id = 0x0024;
static enum camera_sensor_i2c_type g_data_type = CAMERA_SENSOR_I2C_TYPE_DWORD; //dword
static uint8_t g_operation = 0;//read

static char ois_subdev_string[32] = "";

static stReCalib g_calInfo;

//static struct mutex g_busy_job_mutex;

static uint32_t g_vendor_id = 0;
static uint32_t g_module_sn = 0;
static struct cam_eeprom_ctrl_t * g_ectrl = NULL;

static uint8_t g_vcm_enabled = 1;

static uint32_t g_dac_macro[OIS_CLIENT_MAX] = {0};
static uint32_t g_dac_infinity[OIS_CLIENT_MAX] = {0};
static struct timeval g_ssc_config_time_prev;
static uint32_t g_dac_macro_dit[OIS_CLIENT_MAX] = {0};
static uint32_t g_dac_infinity_dit[OIS_CLIENT_MAX] = {0};
static uint32_t g_dac_macro_base[OIS_CLIENT_MAX] = {0};
static uint32_t g_dac_infinity_base[OIS_CLIENT_MAX] = {0};
//static uint32_t g_lens_shift_10cm_to_50cm = 0;
//static uint32_t g_lens_shift_10cm = 0;
static uint8_t  g_verbose_log = 0;
static uint32_t g_lens_position_reg[2] = {0};
static void dump_sma_eeprom(struct cam_ois_ctrl_t *oisCtrl,uint32_t addr,uint8_t length);
static void set_ssc_gain_if_need(uint32_t ois_index,uint32_t val,int32_t distance_cm);
#if 0
static void onsemi_read_check(struct cam_ois_ctrl_t *o_ctrl)
{
	char buf[512];

	onsemi_dump_state(o_ctrl,buf,sizeof(buf));
	pr_info("dump state is\n%s\n",buf);

	onsemi_ois_go_on(o_ctrl);
	onsemi_ssc_go_on(o_ctrl);
	onsemi_dump_state(o_ctrl,buf,sizeof(buf));
	pr_info("dump state is\n%s\n",buf);

	onsemi_check_sequence_read(o_ctrl);
}
#endif
#if 0
static int read_vendor_id_from_eeprom(uint32_t * vendor_id)
{
	int rc;
	uint32_t reg_addr = 0x09;

	if(g_ectrl == NULL)
	{
		pr_err("eeprom ctrl is NULL!\n");
		return -1;
	}
	camera_io_init(&(g_ectrl->io_master_info));
	rc = camera_io_dev_read(&(g_ectrl->io_master_info),
	                          reg_addr,
	                          vendor_id,
	                          CAMERA_SENSOR_I2C_TYPE_WORD,//addr_type
	                          CAMERA_SENSOR_I2C_TYPE_BYTE);//data_type
	camera_io_release(&(g_ectrl->io_master_info));
	if(rc < 0)
	{
		pr_err("EEPROM read reg 0x%x failed! rc = %d\n",reg_addr,rc);
		return -2;
	}
	else
	{
		pr_info("EEPROM read reg 0x%x get val 0x%x\n",reg_addr,*vendor_id);
		if(*vendor_id == 0x01)
			*vendor_id = VENDOR_ID_LITEON;
		else if(*vendor_id == 0x06)
			*vendor_id = VENDOR_ID_PRIMAX;
	}
	return 0;
}

static int read_module_sn_from_eeprom(uint32_t * module_sn)
{
	int rc;
	uint32_t reg_addr = 0x0E;
	uint8_t  sn[4];

	if(g_ectrl == NULL)
	{
		pr_err("eeprom ctrl is NULL!\n");
		return -1;
	}
	camera_io_init(&(g_ectrl->io_master_info));
	rc = camera_io_dev_read_seq(&(g_ectrl->io_master_info),
	                              reg_addr,
	                              sn,
	                              CAMERA_SENSOR_I2C_TYPE_WORD,
	                              CAMERA_SENSOR_I2C_TYPE_BYTE,
	                              4);
	camera_io_release(&(g_ectrl->io_master_info));
	if(rc < 0)
	{
		pr_err("EEPROM read reg 0x%x failed! rc = %d\n",reg_addr,rc);
		return -2;
	}
	else
	{
		*module_sn = (sn[0]<<24 | sn[1]<<16 | sn[2]<<8 | sn[3]);
		pr_info("EEPROM seq read reg 0x%x get SN 0x%08x\n",reg_addr,*module_sn);
	}
	return 0;
}
#endif
int get_ois_status(uint32_t index) {
	if(index < OIS_CLIENT_MAX) return g_ois_status[index];
	else {
		pr_err("un-predict index(%u)\n",index);
		return -1;
	}
}
int get_ois_power_state(uint32_t index) {
	if(index < OIS_CLIENT_MAX) return g_ois_power_state[index];
	else {
		pr_err("un-predict index(%u)\n",index);
		return -1;
	}
}
static int32_t get_ois_ctrl(struct cam_ois_ctrl_t **o_ctrl) {
	uint8_t count = 0;
	while(*o_ctrl == NULL && count < 100) {
		if(ois_ctrl[OIS_CLIENT_IMX686]->ois_on == 1) {
			if(g_verbose_log)
				pr_info("Select cleint IMX686\n");
			*o_ctrl = ois_ctrl[OIS_CLIENT_IMX686];
			return OIS_CLIENT_IMX686;
		}else if(ois_ctrl[OIS_CLIENT_OV08A10]->ois_on == 1) {
			if(g_verbose_log)
				pr_info("Select cleint OV08A10\n");
			*o_ctrl = ois_ctrl[OIS_CLIENT_OV08A10];
			return OIS_CLIENT_OV08A10;
		}
		count++;
		msleep(20);
		pr_info("wait %u sec for ois ctrl ready \n",count*20);
	}

	pr_err("cannot find mapping client ois ctrl\n");
	*o_ctrl = NULL;
	return -1;
}
static fw_trigger_result_t trigger_fw_update(struct cam_ois_ctrl_t *ctrl, uint8_t update_mode, uint8_t force_update, uint32_t* updated_version)
{
	uint32_t fw_version;
	uint32_t actuator_version;

	uint8_t  module_vendor, vcm_version;
	uint8_t  backup_vcm = 0;
	uint32_t module_sn = 0;
	uint16_t fw_update_times = 0;

	uint8_t  battery_capacity;
	fw_trigger_result_t ret;

	int rc = 0;

	rc = onsemi_read_dword(ctrl,0x8000,&fw_version);
	if(rc < 0)
	{
		pr_err("read fw version failed!\n");
		return BAD_IO;
	}

	rc = onsemi_read_dword(ctrl,0x8004,&actuator_version);
	if(rc < 0)
	{
		pr_err("read actuator version failed!\n");
		return BAD_IO;
	}

	if(!ZF7_need_update_fw(fw_version,actuator_version,force_update))
	{
		return NO_NEED;//NO NEED UPDATE FW
	}

	if(sysfs_read_dword_seq(OIS_MODULE_SN,&module_sn,1) == 0 && g_module_sn == module_sn)
	{
		if(sysfs_read_word_seq(OIS_FW_UPDATE_TIMES,&fw_update_times,1) == 0 &&
			fw_update_times >= OIS_FW_UPDATE_TIME_LIMIT)
		{
			if(!force_update && fw_version != 0x0) //not force update nor bad FW
			{
				pr_err("fw has updated %d times, can not update anymore for safety\n",fw_update_times);
				return EXCEED_LIMIT;
			}
		}
	}
	else
	{
		fw_update_times = 0;
		if(fw_version == 0x0)
		{
			pr_err("Bad FW at First, not save it...\n");
			return BAD_FW;
		}
	}

	if(sysfs_read_uint8(BATTERY_CAPACITY,&battery_capacity) == 0)
	{
		pr_info("get battery capacity is %d%%\n",battery_capacity);
		if(battery_capacity<=BATTERY_THRESHOLD)
		{
			pr_err("battery is too low, not update FW\n");
			return BATTERY_LOW;
		}
	}

	if(fw_version == 0x0)
	{
		if(g_vendor_id == VENDOR_ID_LITEON || g_vendor_id == VENDOR_ID_PRIMAX)
		{
			pr_info("Saving failed module ... Vendor ID from EEPROM is 0x%x\n",g_vendor_id);
			module_vendor = g_vendor_id;
			//read backup vcm version
			if(sysfs_read_byte_seq(OIS_VCM_BACKUP,&backup_vcm,1) == 0)
			{
				pr_info("Got backup vcm %d from factory partition\n",backup_vcm);
				vcm_version = backup_vcm;
			}
			else
			{
				pr_err("Can not get backup vcm! Can not save failed module...\n");
				return NO_VCM_BACK;
			}
		}
		else
		{
			pr_err("Vendor ID 0x%x from EEPROM invalid, Can not save failed module...\n",g_vendor_id);
			return VENDOR_INVALID;
		}
	}
	else
	{
		module_vendor = fw_version >> 24;
		vcm_version = (actuator_version & 0x0000FF00)>>8;
	}

	if(update_mode == 1)
	{
		pr_info("warning: mode is 1, force change to 0\n");//0 don't erase user reserve area
		update_mode = 0;
	}

	if(fw_update_times == 0)//backup module sn & vcm for the first time
	{
		pr_info("Update FW FIRST time in this module!\n");
		if(sysfs_write_dword_seq_change_line(OIS_MODULE_SN,&g_module_sn,1,1,0) == 0)
		{
			pr_info("back up module SN 0x%08x done!\n",g_module_sn);
			if(sysfs_write_byte_seq(OIS_VCM_BACKUP,&vcm_version,1) == 0)
				pr_info("back up vcm version 0x%x done!\n",vcm_version);
		}
	}

	rc = ZF7_update_fw(ctrl, update_mode, module_vendor, vcm_version, updated_version);
	if(rc != 0xF0)//execute updating process
	{
		fw_update_times++;
		if(sysfs_write_word_seq(OIS_FW_UPDATE_TIMES,&fw_update_times,1) == 0)
		{
			pr_info("Save FW update times %d done\n",fw_update_times);
		}

		if(rc == 0)
		{
			ret = UPDATE_OK;
		}
		else
		{
			pr_err("update FW failed!\n");
			ret = UPDATE_FAIL;
		}
		get_module_name_from_fw_id(g_fw_version,g_module_vendor);
	}
	else
	{
		pr_err("Can not find correct FW to update for module 0x%x, vcm 0x%x...\n",module_vendor,vcm_version);
		ret = NO_MATCH_FW;//Not find FW to update
	}
	return ret;
}

static chip_check_result_t onsemi_f40_check_chip_info(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_ois_i2c_info_t      *i2c_info = &soc_private->i2c_info;

	uint32_t chip_id;
	uint32_t fw_id;
	uint32_t actuator_id;
	uint8_t  vendor,customer,vcm;
	//uint32_t servo_state;
	chip_check_result_t result;
	//check i2c
	rc = ZF7_IORead32A(o_ctrl,i2c_info->id_register,&chip_id);
	if(rc < 0)
	{
		pr_err("read chip id failed! rc = %d\n",rc);
		return CHECK_BAD_IO;
	}

	//check fw version
	rc = onsemi_read_dword(o_ctrl,0x8000,&fw_id);
	if(rc < 0)
	{
		pr_err("read from reg 0x%x failed! rc = %d\n",0x8000,rc);
		return CHECK_BAD_IO;
	}

	//check vcm
	rc = onsemi_read_dword(o_ctrl,0x8004,&actuator_id);
	if(rc < 0)
	{
		pr_err("read from reg 0x%x failed! rc = %d\n",0x8008,rc);
		return CHECK_BAD_IO;
	}

	if(chip_id != i2c_info->chip_id)
	{
		pr_err("read chip id 0x%x not expected\n",chip_id);
		return CHECK_BAD_ID;
	}

	g_fw_version = fw_id;
	get_module_name_from_fw_id(fw_id,g_module_vendor);//ATD

	vendor = fw_id >> 24;
	customer = (fw_id & 0x00ff0000) >> 16;
	vcm = (actuator_id & 0xff00) >> 8;

	pr_info("read fw id 0x%x, vendor 0x%x, customer 0x%x, vcm 0x%x\n",
			fw_id,vendor,customer,vcm);

	if(fw_id == 0x0)
	{
		pr_err("FW is bad!\n");
		result = CHECK_BAD_FW;
	}
#if 0
	else if(!onsemi_is_servo_on(o_ctrl))
	{
		onsemi_read_dword(o_ctrl,0xF010,&servo_state);
		pr_err("servo state is 0x%x after power up, function bad!\n",servo_state);
		result = CHECK_BAD_FUNCTION;
	}
#endif
	else if(customer != 0x13)
	{
		pr_err("This module is not for ASUS!\n");
		result = CHECK_CUSTOMER_INVALID;//error customer
	}
	else if(vendor != VENDOR_ID_LITEON && vendor != VENDOR_ID_PRIMAX && vendor != VENDOR_ID_HOLITECH)
	{
		pr_err("Module vendor 0x%x invalid!\n",vendor);
		result = CHECK_VENDOR_INVALID;//vendor not valid
	}
#if 0
	else if( (g_vendor_id != 0 && g_vendor_id != 0xFF) && g_vendor_id != vendor)
	{
		pr_err("Module vendor 0x%x mismatch that from eeprom 0x%x!\n",vendor,g_vendor_id);
		result = CHECK_VENDOR_MISMATCH;//vendor mismatch with eeprom
	}
	else
	{
		result = CHECK_PASS;
		if(vendor == VENDOR_ID_PRIMAX)
		{
			if(vcm != 0x01 && vcm != 0x02)
			{
				pr_err("PRIMAX VCM version 0x%x invalid!\n",vcm);
				result = CHECK_VCM_INVALID;
			}
			else if(fw_id < PRIMAX_VERNUM_BASE)
			{
				pr_err("PRIMAX FW version 0x%x invalid!\n",fw_id);
				result = CHECK_FW_VERSION_INVALID;//version not valid
			}
		}
		else if(vendor == VENDOR_ID_LITEON)
		{
			if(vcm != 0x00 && vcm != 0x01 && vcm != 0x02)
			{
				pr_err("LITEON VCM version 0x%x invalid\n",vcm);
				result = CHECK_VCM_INVALID;
			}
			else if(fw_id < LITEON_VERNUM_BASE)
			{
				pr_err("LITEON FW version 0x%x invalid!\n",fw_id);
				result = CHECK_FW_VERSION_INVALID;//version not valid
			}
		}
	}
#endif
    result = CHECK_PASS;
	return result;
}
static int ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc;
	struct cam_hw_soc_info         *soc_info = &o_ctrl->soc_info;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		pr_err("ois power up failed, rc %d\n", rc);
		return -1;
	}
	if (o_ctrl->io_master_info.master_type == CCI_MASTER)
	{
		rc = camera_io_init(&(o_ctrl->io_master_info));
		if (rc < 0) {
			pr_err("cci init failed!\n");
			rc = cam_sensor_util_power_down(power_info, soc_info);
			if (rc) {
				pr_err("ois power down failed, rc %d\n", rc);
			}
			return -2;
		}
	}
	return rc;
}

static int ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc;
	struct cam_hw_soc_info         *soc_info = &o_ctrl->soc_info;
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if(o_ctrl->io_master_info.master_type == CCI_MASTER)
	{
		rc = camera_io_release(&(o_ctrl->io_master_info));
		if (rc < 0)
			pr_err("cci release failed!\n");
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		pr_err("ois power down failed, rc %d\n", rc);
	}
	return rc;
}

static int ois_probe_status_proc_read(struct seq_file *buf, void *v)
{
	uint32_t k;
	for(k=0;k<OIS_CLIENT_MAX;k++) {
		if(ois_ctrl[k] == NULL) {
			pr_err("ois_ctrl[%u] is null\n",k);
		}else {
			mutex_lock(&ois_ctrl[k]->ois_mutex);
			seq_printf(buf, "%d\n", g_ois_status[k]);
			mutex_unlock(&ois_ctrl[k]->ois_mutex);
		}
	}

	return 0;
}

static int ois_probe_status_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_probe_status_proc_read, NULL);
}

static const struct file_operations ois_probe_status_fops = {
	.owner = THIS_MODULE,
	.open = ois_probe_status_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ois_atd_status_proc_read(struct seq_file *buf, void *v)
{

	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	mutex_lock(&oisCtrl->ois_mutex);

	seq_printf(buf, "%d\n", g_atd_status);
	g_atd_status = 0;//default is failure

	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_atd_status_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_atd_status_proc_read, NULL);
}

static ssize_t ois_atd_status_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;
	char messages[16]="";
	uint32_t val;
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	rc = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}
	sscanf(messages,"%d",&val);
	mutex_lock(&oisCtrl->ois_mutex);

	switch(val)
	{
		case 0:
			g_atd_status = 0;
			g_verbose_log = 0;
			break;
		case 1:
			g_atd_status = 1;
			break;
		default:
			g_atd_status = 1;
			g_verbose_log = 1;
	}
	mutex_unlock(&oisCtrl->ois_mutex);

	pr_info("ATD status changed to %d\n",g_atd_status);

	return rc;
}

static const struct file_operations ois_atd_status_fops = {
	.owner = THIS_MODULE,
	.open = ois_atd_status_proc_open,
	.read = seq_read,
	.write = ois_atd_status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ois_cali_proc_read(struct seq_file *buf, void *v)
{
	uint32_t x,y;
	struct cam_ois_ctrl_t *oisCtrl = NULL; 
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;

	mutex_lock(&oisCtrl->ois_mutex);

	if(g_ois_power_state[ois_index])
	{
		onsemi_gyro_read_xy(oisCtrl,&x,&y);
		seq_printf(buf, "Factory(0x%x,0x%x), Recal(0x%x,0x%x), Diff(0x%x 0x%x) (%d %d), ReadVal(0x%x,0x%x)\n",
					g_calInfo.SsFctryOffX,g_calInfo.SsFctryOffY,
					g_calInfo.SsRecalOffX,g_calInfo.SsRecalOffY,
					g_calInfo.SsDiffX,g_calInfo.SsDiffY,
					g_calInfo.SsDiffX,g_calInfo.SsDiffY,
					x,y
				);
	}
	else
	{
		seq_printf(buf,"POWER DOWN\n");
	}

	mutex_unlock(&oisCtrl->ois_mutex);

	return 0;
}

static int ois_cali_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_cali_proc_read, NULL);
}

static ssize_t ois_cali_proc_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	uint8_t rc;
	#define GYRO_K_REG_COUNT 2
	uint32_t gyro_data[GYRO_K_REG_COUNT];
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	pr_info("E\n");
	if(oisCtrl == NULL) return -1;
	
	mutex_lock(&g_busy_job_mutex);

	mutex_lock(&oisCtrl->ois_mutex);

	pr_info("start gyro calibration\n");
	rc = onsemi_gyro_calibration(oisCtrl); //TODO
	if(rc != 0)
	{
		pr_err("onsemi_gyro_calibration failed! rc = 0x%02x\n", rc);
		g_atd_status = 0;
	}
	else
	{
		pr_info("onsemi_gyro_calibration success!\n");
		g_atd_status = 1;
	}
    //ASUS_BSP Lucien +++: Save one Gyro data after doing OIS calibration
	rc = onsemi_gyro_read_xy(oisCtrl, &gyro_data[0], &gyro_data[1]);
	if(rc < 0) pr_err("onsemi_gyro_read_xy get fail! rc = %d\n", rc);
	rc = sysfs_write_dword_seq_change_line(OIS_GYRO_K_OUTPUT_FILE_NEW,gyro_data,GYRO_K_REG_COUNT,2,0);
	if(rc != 0) pr_err("sysfs_write_dword_seq_change_line fail! rc = %d\n", rc);
	//ASUS_BSP Lucien ---: Save one Gyro data after doing OIS calibration

	mutex_unlock(&oisCtrl->ois_mutex);

	mutex_unlock(&g_busy_job_mutex);
	pr_info("X\n" );

	return count;
}

static const struct file_operations ois_cali_fops = {
	.owner = THIS_MODULE,
	.open = ois_cali_proc_open,
	.read = seq_read,
	.write = ois_cali_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ois_mode_proc_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
        mutex_lock(&oisCtrl->ois_mutex);
	seq_printf(buf, "%d\n", g_ois_mode);
	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_mode_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_mode_proc_read, NULL);
}

static ssize_t ois_mode_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint32_t val;
	int rc;
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	ret_len = len;
	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);

	mutex_lock(&oisCtrl->ois_mutex);

	pr_info("start change ois mode\n");

	rc = onsemi_switch_mode(oisCtrl,val);

	if(rc == 0)
	{
		g_atd_status = 1;
		g_ois_mode = val;
		pr_info("OIS mode changed to %d by ATD\n",g_ois_mode);
	}
	else
	{
		g_atd_status = 0;
		pr_err("switch to mode %d failed! rc = %d\n",val,rc);
	}

	mutex_unlock(&oisCtrl->ois_mutex);

	return ret_len;
}

static const struct file_operations ois_mode_fops = {
	.owner = THIS_MODULE,
	.open = ois_mode_proc_open,
	.read = seq_read,
	.write = ois_mode_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};


static int ois_device_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%s\n",ois_subdev_string);
	return 0;
}

static int ois_device_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_device_read, NULL);
}

static ssize_t ois_device_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[64]="";

	ret_len = len;
	if (len > 64) {
		len = 64;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%s", ois_subdev_string);

	pr_info("write subdev=%s\n", ois_subdev_string);
	return ret_len;
}

static const struct file_operations ois_device_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ois_device_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ois_device_write,
};

//ASUS_BSP Lucien +++: Add OIS SEMCO module
static int ois_module_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	mutex_lock(&oisCtrl->ois_mutex);
	seq_printf(buf, "%s\n",g_module_vendor);
	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_module_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_module_read, NULL);
}

static ssize_t ois_module_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	return 0;
}

static const struct file_operations ois_module_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ois_module_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ois_module_write,
};
//ASUS_BSP Lucien ---: Add OIS SEMCO module
static int ois_i2c_debug_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val;
	int rc;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	mutex_lock(&oisCtrl->ois_mutex);

	if(g_ois_power_state[ois_index])
	{
		ZF7_WaitProcess(oisCtrl,0,__func__);
		oisCtrl->io_master_info.cci_client->sid = g_slave_id;
		switch(g_data_type)
		{
			case CAMERA_SENSOR_I2C_TYPE_BYTE:
				rc = onsemi_read_byte(oisCtrl,g_reg_addr,&reg_val);
				break;
			case CAMERA_SENSOR_I2C_TYPE_WORD:
				rc = onsemi_read_word(oisCtrl,g_reg_addr,&reg_val);
				break;
			case CAMERA_SENSOR_I2C_TYPE_DWORD:
				rc = onsemi_read_dword(oisCtrl,g_reg_addr,&reg_val);
				break;
			default:
				rc = onsemi_read_dword(oisCtrl,g_reg_addr,&reg_val);
		}

		if(g_operation == 1)//write
		{
			if(reg_val == g_reg_val)
			{
				pr_info("read back the same value as write!\n");
			}
			else
			{
				pr_err("write value 0x%x and read back value 0x%x not same!\n",
						g_reg_val,reg_val
				);
			}
		}

		if(rc == 0)
		{
			g_atd_status = 1;
		}
		else
		{
			g_atd_status = 0;
			pr_err("read from reg 0x%x failed! rc = %d\n",g_reg_addr,rc);
		}

		seq_printf(buf,"0x%x\n",reg_val);
	}
	else
	{
		seq_printf(buf,"POWER DOWN\n");
	}

	mutex_unlock(&oisCtrl->ois_mutex);

	return 0;
}

static int ois_i2c_debug_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_i2c_debug_read, NULL);
}
static ssize_t ois_i2c_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	int n;
	char messages[32]="";
	uint32_t val[4];
	int rc;
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	ret_len = len;
	if (len > 32) {
		len = 32;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages,"%x %x %x %x",&val[0],&val[1],&val[2],&val[3]);

	mutex_lock(&oisCtrl->ois_mutex);

	if(n == 1)
	{
		g_reg_addr = val[0];
		g_operation = 0;
		g_data_type = 4;//default dword
	}
	else if(n == 2)
	{
		//data type
		// 1 byte 2 word 4 dword
		g_reg_addr = val[0];
		g_data_type = val[1];
		g_operation = 0;
	}
	else if(n == 3)
	{
		g_reg_addr = val[0];
		g_data_type = val[1];
		g_reg_val = val[2];
		g_operation = 1;
	}
	else if(n == 4)
	{
		g_reg_addr = val[0];
		g_data_type = val[1];
		g_reg_val = val[2];
		g_slave_id = val[3];//slave id
		g_operation = 1;
	}

	if(g_data_type != 1 && g_data_type != 2 && g_data_type != 4 )
		g_data_type = 4;//default dword
	pr_info("gona %s SLAVE 0x%X reg 0x%04x, data type %s\n",
			g_operation ? "WRITE":"READ",
			g_slave_id,
			g_reg_addr,
			g_data_type == 1?"BYTE":(g_data_type == 2?"WORD":"DWORD")
	);

	switch(g_data_type)
	{
		case 1:
			g_data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
			break;
		case 2:
			g_data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
			break;
		case 4:
			g_data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
			break;
		default:
			g_data_type = CAMERA_SENSOR_I2C_TYPE_DWORD;
	}

	if(g_operation == 1)
	{
		ZF7_WaitProcess(oisCtrl,0,__func__);
		switch(g_data_type)
		{
			case CAMERA_SENSOR_I2C_TYPE_BYTE:
				rc = onsemi_write_byte(oisCtrl,g_reg_addr,g_reg_val);
				break;
			case CAMERA_SENSOR_I2C_TYPE_WORD:
				rc = onsemi_write_word(oisCtrl,g_reg_addr,g_reg_val);
				break;
			case CAMERA_SENSOR_I2C_TYPE_DWORD:
				rc = onsemi_write_dword(oisCtrl,g_reg_addr,g_reg_val);
				break;
			default:
				rc = onsemi_write_dword(oisCtrl,g_reg_addr,g_reg_val);
		}
		if(rc < 0)
		{
			pr_err("write 0x%x to reg 0x%04x FAIL\n",g_reg_val,g_reg_addr);
			g_atd_status = 0;
		}
		else
		{
			pr_info("write 0x%x to reg 0x%04x OK\n",g_reg_val,g_reg_addr);
			g_atd_status = 1;
			#if 0 //Temp disable //TODO
			if(g_data_type == CAMERA_SENSOR_I2C_TYPE_DWORD && g_reg_addr == 0xF01A)
			{
				set_vcm_lens_pos_from_ois_writing(g_reg_val&0x07FF);
			}
			#endif
		}
	}

	mutex_unlock(&oisCtrl->ois_mutex);

	return ret_len;
}
static const struct file_operations ois_i2c_debug_fops = {
	.owner = THIS_MODULE,
	.open = ois_i2c_debug_open,
	.read = seq_read,
	.write = ois_i2c_debug_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ois_solo_power_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; 
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
        mutex_lock(&oisCtrl->ois_mutex);
        pr_info("g_ois_power_state = %d\n", g_ois_power_state[ois_index]);
	seq_printf(buf,"%d\n",g_ois_power_state[ois_index]);
	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_solo_power_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_solo_power_read, NULL);
}

//just for ATD test
static ssize_t ois_solo_power_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	int val;
	//int rc;
	//int i;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	//struct cam_hw_soc_info         *soc_info = NULL;
	//struct cam_ois_soc_private     *soc_private = NULL;
	//struct cam_sensor_power_ctrl_t *power_info = NULL;
	int32_t ois_index  = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
		
	//soc_info = &oisCtrl->soc_info;
	//soc_private = (struct cam_ois_soc_private *)oisCtrl->soc_info.soc_private;
	//power_info = &soc_private->power_info;

	ret_len = len;
	if (len > 16) {
		len = 16;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);
	mutex_lock(&oisCtrl->ois_mutex);
	if(g_ois_camera_open[ois_index] == 0)
	{
		#if 0
		//ASUS_BSP Byron not support test ois on if camera close 
		if(val == 0)
		{
			if(g_ois_power_state[ois_index] == 1)
			{
				camera_io_release(&(oisCtrl->io_master_info));
				rc = cam_sensor_util_power_down(power_info, soc_info);
				if (rc) {
					pr_err("%s: msm_camera_power_down fail rc = %d\n", __func__, rc);
				}
				else
				{
					g_ois_power_state[ois_index] = 0;
					g_ois_mode = 255;
					pr_info("OIS POWER DOWN\n");
				}
			}
			else
			{
				pr_info("OIS already power off, do nothing\n");
			}
		}
		else
		{
			if(g_ois_power_state[ois_index] == 0)
			{
				/* Get Clock */
				for (i = 0; i < soc_info->num_clk; i++) {
					soc_info->clk[i] = clk_get(soc_info->dev,
						soc_info->clk_name[i]);
					if (!soc_info->clk[i]) {
						CAM_ERR(CAM_UTIL, "get failed for %s",
							soc_info->clk_name[i]);
					}
				}
				rc = cam_sensor_core_power_up(power_info, soc_info);
				pr_err("%s cam_sensor_core_power_up done rc %d", __func__,rc);
				if (rc) {
					pr_err("%s: msm_camera_power_up fail rc = %d\n", __func__, rc);
				}
				else
				{
					g_ois_power_state[ois_index] = 1;
					g_ois_mode = 255;
					camera_io_init(&(oisCtrl->io_master_info));
					pr_err("%s camera_io_init done", __func__);
					//delay is in power setting
					if(g_ois_status[ois_index] == 1)
					{
						ZF7_WaitProcess(oisCtrl,0,__func__);
						//onsemi_config_ssc_gain(ois_ctrl,100); //TODO
					}
					else
					{
						pr_err("OIS probe failed, not config ssc gain\n");
					}
					pr_info("OIS POWER UP\n");
				}
			}
			else
			{
				pr_info("OIS already power up, do nothing\n");
			}
		}
		#endif
		return -1;
	}
	else
	{
		pr_err("camera has been opened, can't control ois power\n");
	}
	mutex_unlock(&oisCtrl->ois_mutex);
	return ret_len;
}
static const struct file_operations ois_solo_power_fops = {
	.owner = THIS_MODULE,
	.open = ois_solo_power_open,
	.read = seq_read,
	.write = ois_solo_power_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ois_state_proc_read(struct seq_file *buf, void *v)
{
	char dump_buf[512];//must be large enough
	uint8_t battery_capacity;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	mutex_lock(&oisCtrl->ois_mutex);
	if(g_ois_power_state[ois_index] == 1)
	{
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_dump_state(oisCtrl,dump_buf,sizeof(dump_buf));
		seq_printf(buf, "%s\n", dump_buf);
		if(sysfs_read_uint8(BATTERY_CAPACITY,&battery_capacity) == 0)
		{
			pr_info("get battery capacity is %d%%\n",battery_capacity);
			seq_printf(buf, "battery: %d%%\n", battery_capacity);
		}
	}
	else
	{
		seq_printf(buf, "POWER DOWN\n");
	}
	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_state_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, ois_state_proc_read, NULL);
}

static const struct file_operations ois_state_fops = {
	.owner = THIS_MODULE,
	.open = ois_state_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ois_on_proc_read(struct seq_file *buf, void *v)
{
	int state;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	mutex_lock(&oisCtrl->ois_mutex);
	if(g_ois_power_state[ois_index])
	{
		if(onsemi_is_ois_on(oisCtrl))
			state=1;
		else
			state=0;
		seq_printf(buf, "%d\n", state);
	}
	else
		seq_printf(buf, "%s\n", "POWER DOWN\n");
	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_on_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_on_proc_read, NULL);
}

static ssize_t ois_on_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint32_t val;
	int rc;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	ret_len = len;

	
	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);
	pr_info("OIS on mode %d\n",val);

	mutex_lock(&oisCtrl->ois_mutex);
	if(g_ois_power_state[ois_index])
	{
		switch(val)
		{
			case 0:
				rc = onsemi_ois_go_off(oisCtrl);
				break;

			case 1:
				rc = onsemi_ois_go_on(oisCtrl);
				break;
			default:
				pr_err("Not supported command %d\n",val);
				rc = -1;
		}

		if(rc == 0)
		{
			g_atd_status = 1;
		}
		else
		{
			g_atd_status = 0;
			pr_err("OIS on/off failed! rc = %d\n",rc);
		}
	}
	else
	{
		pr_err("OIS POWER DOWN, power it up first!\n");
		g_atd_status = 0;
	}
	mutex_unlock(&oisCtrl->ois_mutex);

	return ret_len;
}

static const struct file_operations ois_on_fops = {
	.owner = THIS_MODULE,
	.open = ois_on_proc_open,
	.read = seq_read,
	.write = ois_on_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int process_rdata(int count,int32_t ois_index)
{
#define PAIR 3
	int rc;
	int i;
	uint32_t old_state;
	struct timeval t1,t2;

	uint32_t *pData = NULL;
	uint32_t reg_addr[PAIR*2] = {  0x0220,0x0224,//gyro raw
							     0x0450,0x047C, //acc raw
							     0x0178,0x017C //hall raw
						        };
	int j;
	
	pData = kzalloc(sizeof(uint32_t)*count*PAIR*2,GFP_KERNEL);
	if (!pData)
	{
		pr_err("no memory!\n");
		return -1;
	}

	if(ois_index == OIS_CLIENT_IMX686) {
		pr_info("OIS_CLIENT_IMX686 read data\n");
		reg_addr[4] = 0x06A0;
		reg_addr[5] = 0x06A2;
	}
	onsemi_get_ois_state(ois_ctrl[ois_index],&old_state);
	//onsemi_ois_go_on(ois_ctrl);//servo and ois must be on
	do_gettimeofday(&t1);
	for(i=0;i<count;i++)
	{
		for(j=0;j<PAIR;j++)
		{
			rc = onsemi_read_pair_sensor_data(ois_ctrl[ois_index],
											reg_addr[2*j],reg_addr[2*j+1],
											pData+PAIR*2*i+2*j,pData+PAIR*2*i+2*j+1
											);
			if(rc < 0)
			{
				pr_err("read %d times x,y failed! rc = %d\n",i,rc);
				rc = -2;
				break;
			}
		}
		if(rc<0)
		{
			break;
		}

		if(i!=0 && i%8 == 0) {
			usleep_range(50,55);
		}
	}
	do_gettimeofday(&t2);
	pr_info("read %d times x,y values, cost %lld ms, each cost %lld us\n",
		i,
		diff_time_us(&t2,&t1)/1000,
		diff_time_us(&t2,&t1)/(i)
	);

	onsemi_restore_ois_state(ois_ctrl[ois_index],old_state);

	if(i == count)
	{
		//all pass, store to /sdcard/gyro.csv
		//remove file....do it in script
		if(sysfs_write_dword_seq_change_line(RDATA_OUTPUT_FILE,pData,count*PAIR*2,PAIR*2,1) == 0)
		{
			pr_info("store gyro data to %s succeeded!\n",RDATA_OUTPUT_FILE);
			rc = 0;
		}
		else
		{
			pr_err("store gyro data to %s failed!\n",RDATA_OUTPUT_FILE);
			rc = -3;
		}
	}
	else
	{
		pr_err("read data failed, read %d data\n",i);
	}


	kfree(pData);

	return rc;
}

static int ois_rdata_proc_read(struct seq_file *buf, void *v)
{
	uint8_t* pText;
	uint64_t size;
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	mutex_lock(&oisCtrl->ois_mutex);

	if(get_file_size(RDATA_OUTPUT_FILE,&size) == 0 && size > 0)
	{
		pText = kzalloc(sizeof(uint8_t)*size,GFP_KERNEL);
		if (!pText)
		{
			pr_err("no memory!\n");
			mutex_unlock(&oisCtrl->ois_mutex);
			return 0;
		}
		read_file_into_buffer(RDATA_OUTPUT_FILE,pText,size);//Text File

		seq_printf(buf,"%s\n",pText);//ASCII

		kfree(pText);
	}
	else
	{
		seq_printf(buf,"file is empty!\n");
	}

	mutex_unlock(&oisCtrl->ois_mutex);

	return 0;
}

static int ois_rdata_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_rdata_proc_read, NULL);
}

static ssize_t ois_rdata_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint32_t val;
	int rc;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	ret_len = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);

	mutex_lock(&g_busy_job_mutex);

	mutex_lock(&oisCtrl->ois_mutex);

	if(g_ois_power_state[ois_index])
	{
		rc = process_rdata(val,ois_index); //ASUS_BSP Lucien +++: Save val numbers of Gyro X/Y and ACC X/Y data
		if(rc == 0)
		{
			g_atd_status = 1;
		}
		else
		{
			g_atd_status = 0;
			pr_err("OIS Rdata failed! rc = %d\n",rc);
		}
	}
	else
	{
		pr_err("OIS POWER DOWN, power it up first!\n");
		g_atd_status = 0;
	}

	mutex_unlock(&oisCtrl->ois_mutex);

	mutex_unlock(&g_busy_job_mutex);

	return ret_len;
}

static const struct file_operations ois_rdata_fops = {
	.owner = THIS_MODULE,
	.open = ois_rdata_proc_open,
	.read = seq_read,
	.write = ois_rdata_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ois_update_fw_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
        mutex_lock(&oisCtrl->ois_mutex);
	seq_printf(buf, "%x\n", g_fw_version);
	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_update_fw_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_update_fw_read, NULL);
}

static ssize_t ois_update_fw_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[64]="";
	int n;
	uint32_t val[2];
	uint32_t force_update, update_mode;
	struct cam_ois_ctrl_t *oisCtrl = ois_ctrl[OIS_CLIENT_OV08A10];
	int32_t ois_index = OIS_CLIENT_OV08A10;
	if(oisCtrl == NULL) return -1;
	
	ret_len = len;
	
	if (len > 64) {
		len = 64;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages, "%x %x", &val[0],&val[1]);
	if(n == 1)
	{
		force_update = val[0];
		update_mode = 0;
	}
	else if(n == 2)
	{
		force_update = val[0];
		update_mode = val[1];
	}
	else
	{
		pr_err("Invalid argument count %d!\n",n);
		return ret_len;
	}

	mutex_lock(&g_busy_job_mutex);

	mutex_lock(&oisCtrl->ois_mutex);

	if(g_ois_power_state[ois_index] == 0)
	{
		pr_err("OIS POWER DOWN, power it up first!\n");
		goto END;
	}

	pr_info("trigger fw update, force update %d, update mode %d\n",force_update,update_mode);

	trigger_fw_update(oisCtrl, update_mode, force_update, &g_fw_version);

END:
	mutex_unlock(&oisCtrl->ois_mutex);
	mutex_unlock(&g_busy_job_mutex);
	return ret_len;
}

static const struct file_operations ois_update_fw_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ois_update_fw_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ois_update_fw_write,
};


static int ois_vcm_enable_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
        mutex_lock(&oisCtrl->ois_mutex);
	seq_printf(buf, "%d\n", g_vcm_enabled);
	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_vcm_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_vcm_enable_read, NULL);
}

static ssize_t ois_vcm_enable_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[8]="";

	uint32_t val;
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	ret_len = len;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%d", &val);

	mutex_lock(&oisCtrl->ois_mutex);

	if(val == 0)
		g_vcm_enabled = 0;
	else
		g_vcm_enabled = 1;

	pr_info("ois vcm enabled set to %d\n",g_vcm_enabled);

	mutex_unlock(&oisCtrl->ois_mutex);

	return ret_len;
}

static const struct file_operations ois_vcm_enable_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ois_vcm_enable_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ois_vcm_enable_write,
};
static int ois_sma_eeprom_dump_read(struct seq_file *buf, void *v)
{
	return 0;
}
static int ois_sma_eeprom_dump_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_sma_eeprom_dump_read, NULL);
}

static ssize_t ois_sma_eeprom_dump_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
#if 1
	ssize_t ret_len;
	char messages[8]="";

	uint32_t addr;
	uint8_t length;
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) {
		pr_err("no ois ctrl to use\n");
		return -1;
	}
	
	ret_len = len;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%x %u", &addr,&length);

	pr_info("check addr(0x%x),length(%u)\n",addr,length);
	dump_sma_eeprom(oisCtrl,addr,length);

	return ret_len;
	#endif
}

static const struct file_operations ois_sma_eerpom_dump_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ois_sma_eeprom_dump_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ois_sma_eeprom_dump_write,
};

static int ois_af_state_read(struct seq_file *buf, void *v)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index __attribute__((unused))= get_ois_ctrl(&oisCtrl);
	int k;
	if(oisCtrl == NULL) return -1;

	mutex_lock(&oisCtrl->ois_mutex);
	for(k=0;k<OIS_CLIENT_MAX;k++) {
		seq_printf(buf, "ois Index(%u) DAC_macro: %d, DAC_infinity: %d\n",k,
					     g_dac_macro[k], g_dac_infinity[k]);
		seq_printf(buf, "ois Index(%u) DAC_macro_DIT: %d, DAC_infinity_DIT: %d\n",k,
					     g_dac_macro_dit[k], g_dac_infinity_dit[k]);
	}
	mutex_unlock(&oisCtrl->ois_mutex);
	return 0;
}

static int ois_af_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, ois_af_state_read, NULL);
}

static ssize_t ois_af_state_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[8]="";
	
	uint32_t af_state;
	int32_t distance_cm;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	ret_len = len;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%d %d", &af_state,&distance_cm);
	//pr_info("check state(%d) distance_cm(%d)\n",af_state,distance_cm);
	mutex_lock(&oisCtrl->ois_mutex);

	set_ssc_gain_if_need(ois_index,af_state,distance_cm);
	mutex_unlock(&oisCtrl->ois_mutex);

	return ret_len;
}

static const struct file_operations ois_af_state_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ois_af_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= ois_af_state_write,
};

void track_mode_change_from_i2c_write(struct cam_sensor_i2c_reg_setting * setting)
{
	uint32_t data[2];
	uint16_t index[2];
	bool     ret[2];

	ret[0] = i2c_setting_contain_address(setting,0xF013, &data[0], &index[0]);//movie/still mode
	ret[1] = i2c_setting_contain_address(setting,0xF012, &data[1], &index[1]);//ois on/off

	if(ret[1] && data[1] == 0x0)
	{
		g_ois_mode = 0;
		CAM_INFO(CAM_OIS, "ois mode change to %d center mode by HAL",g_ois_mode);
	}
	else if(ret[0])
	{
		if(data[0] == 0x0)
		{
			g_ois_mode = 1;
			CAM_INFO(CAM_OIS, "ois mode change to %d movie mode by HAL",g_ois_mode);
		}
		else if(data[0] == 0x1)
		{
			g_ois_mode = 2;
			CAM_INFO(CAM_OIS, "ois mode change to %d still mode by HAL",g_ois_mode);
		}
		else
		{
			CAM_INFO(CAM_OIS, "ois mode set val 0x%x not valid",data[0]);
		}
	}
}
static int ois_get_lens_info_read(struct seq_file *buf, void *v)
{
	uint32_t lens_position_val[2] = {0};
	int rc;
	struct timespec64 timeStamp;
	uint64_t recordTimeStamp = 0;
	//const uint64_t NanoSecondsPerSecond = 1000000000ULL;
	struct cam_ois_ctrl_t *oisCtrl = NULL;
	int32_t ois_index = get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	mutex_lock(&oisCtrl->ois_mutex);

	if(g_ois_power_state[ois_index])
	{
		ZF7_WaitProcess(oisCtrl,0,__func__);
		oisCtrl->io_master_info.cci_client->sid = g_slave_id;

		rc = onsemi_read_dword(oisCtrl,g_lens_position_reg[0],&lens_position_val[0]);
		rc = onsemi_read_dword(oisCtrl,g_lens_position_reg[1],&lens_position_val[1]);
			

		get_monotonic_boottime64(&timeStamp); //ASUS_BSP Byron recod boot time since 1970
		recordTimeStamp = (timeStamp.tv_sec * 1000000000) + (timeStamp.tv_nsec);
		//pr_err("Byron check LensPostion value (0x%08x 0x%08x) (%llu)\n",lens_position_val[0],lens_position_val[1],recordTimeStamp);

		seq_printf(buf,"%x %x %llu\n",lens_position_val[0],lens_position_val[1],recordTimeStamp);
	}
	else
	{
		seq_printf(buf,"POWER DOWN\n");
	}

	mutex_unlock(&oisCtrl->ois_mutex);

	return 0;
}

static int ois_get_lens_info_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_get_lens_info_read, NULL);
}
static ssize_t ois_get_lens_info_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	int n;
	char messages[32]="";
	uint32_t val[2];
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL) return -1;
	
	ret_len = len;
	if (len > 32) {
		len = 32;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages,"%x %x",&val[0],&val[1]);

	mutex_lock(&oisCtrl->ois_mutex);
	g_lens_position_reg[0] = val[0];
	g_lens_position_reg[1] = val[1];
	//pr_err("Byron check LensPostion (0x%04x,0x%04x)\n",g_lens_position_reg[0],g_lens_position_reg[1]);
	mutex_unlock(&oisCtrl->ois_mutex);

	return ret_len;
}
static const struct file_operations ois_get_lens_info_fops = {
	.owner = THIS_MODULE,
	.open = ois_get_lens_info_open,
	.read = seq_read,
	.write = ois_get_lens_info_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_proc_file(const char *PATH,const struct file_operations* f_ops)
{
	struct proc_dir_entry *pde;

	pde = proc_create(PATH, 0666, NULL, f_ops);
	if(pde)
	{
		pr_info("create(%s) done\n",PATH);
	}
	else
	{
		pr_err("create(%s) failed!\n",PATH);
	}
}

static void create_ois_proc_files_shipping(void)
{
	static uint8_t has_created = 0;
	if(!has_created)
	{
		create_proc_file(PROC_DEVICE, &ois_device_proc_fops);
		create_proc_file(PROC_AF_ATATE, &ois_af_state_proc_fops);
		has_created = 1;
	}
	else
	{
		pr_err("OIS shipping proc files have already created!\n");
	}
}

static void create_ois_proc_files_factory(void)
{
	static uint8_t has_created = 0;

	if(!has_created)
	{
		create_proc_file(PROC_POWER, &ois_solo_power_fops);
		create_proc_file(PROC_I2C_RW,&ois_i2c_debug_fops);//ATD
		create_proc_file(PROC_ON, &ois_on_fops);
		create_proc_file(PROC_STATE, &ois_state_fops);
		create_proc_file(PROC_MODE, &ois_mode_fops);//ATD
		create_proc_file(PROC_CALI, &ois_cali_fops);//ATD
		create_proc_file(PROC_RDATA, &ois_rdata_fops);//ATD
		create_proc_file(PROC_ATD_STATUS, &ois_atd_status_fops);//ATD
		create_proc_file(PROC_PROBE_STATUS, &ois_probe_status_fops);//ATD
		create_proc_file(PROC_FW_UPDATE, &ois_update_fw_proc_fops);
		create_proc_file(PROC_MODULE, &ois_module_proc_fops);//ATD ASUS_BSP Lucien +++: Add OIS SEMCO module
		create_proc_file(PROC_VCM_ENABLE, &ois_vcm_enable_proc_fops);
		create_proc_file(PROC_SMA_EEPROM_DUMP, &ois_sma_eerpom_dump_proc_fops);
		create_proc_file(PROC_OIS_GET_LENS_INFO,&ois_get_lens_info_fops);
		has_created = 1;
	}
	else
	{
		pr_err("OIS factory proc files have already created!\n");
	}
}
static void check_chip_and_update_status(struct cam_ois_ctrl_t *o_ctrl,int32_t ois_index)
{
	chip_check_result_t check_result;
	check_result = onsemi_f40_check_chip_info(o_ctrl);
	switch(check_result)
	{
		case CHECK_BAD_IO:
			 g_ois_status[ois_index] = 0;
			 break;
		case CHECK_PASS:
			 g_ois_status[ois_index] = 1;
			 break;
		case CHECK_BAD_ID:
			 g_ois_status[ois_index] = 2;//ID not match
			 break;
		case CHECK_BAD_FW:
			 g_ois_status[ois_index] = 3;//FW bad
			 break;
		case CHECK_BAD_FUNCTION:
		     g_ois_status[ois_index] = 4;//Function bad
			 break;
		case CHECK_VENDOR_MISMATCH:
		     g_ois_status[ois_index] = 5;//vendor in eeprom mismatch fw
		     break;
		case CHECK_VENDOR_INVALID:
		case CHECK_CUSTOMER_INVALID:
		case CHECK_VCM_INVALID:
		case CHECK_FW_VERSION_INVALID:
			 g_ois_status[ois_index] = 6;//module invalid
			 break;
	}
}
void ois_probe_check(uint16_t sensor_id)
{
	//int rc;
#if UPDATE_FW_AT_PROBE
	fw_trigger_result_t trigger_result;
#endif
#ifdef ZS670KS
	const uint8_t device_high_level_check = 0x6b;
#endif
	struct cam_ois_ctrl_t *oisCtrl;
	int32_t ois_index;
	
	if(sensor_id == 0x0686){
		oisCtrl = ois_ctrl[OIS_CLIENT_IMX686];
		ois_index = OIS_CLIENT_IMX686;
	}else if(sensor_id == 0x0841){
		oisCtrl = ois_ctrl[OIS_CLIENT_OV08A10];
		ois_index = OIS_CLIENT_OV08A10;
	}else {
		pr_info("no ois with sensor id(0x%x)\n",sensor_id);
		return;
	}
	#ifdef ZS670KS
	pr_info("eeprom_camera_specs = 0x%x\n",eeprom_camera_specs);
	if(eeprom_camera_specs != device_high_level_check) {
		pr_info("with low level device, no ois to probe\n");
		return;
	}
	#endif
	if(oisCtrl == NULL)
	{
		#if defined(ASUS_DXO) ||defined(ZS670KS)
		pr_err("oisCtrl is NULL!!! in sensor_id(0x%x)\n",sensor_id);
		#else
		pr_info("oisCtrl is NULL!!! in sensor_id(0x%x)\n",sensor_id);
		#endif
		return;
	}
	if(ois_power_up(oisCtrl) != 0)
	{
		pr_err("ois power up failed\n");
		return;
	}
	ZF7_WaitProcess(oisCtrl,0,__func__);//waiting after power on
#if 0
	rc = read_vendor_id_from_eeprom(&g_vendor_id);
	if(rc == 0)
	{
		pr_info("Got Vendor ID 0x%x from EEPROM Read\n",g_vendor_id);
	}
	else
		g_vendor_id = 0;

	rc = read_module_sn_from_eeprom(&g_module_sn);
	if(rc == 0)
	{
		pr_info("Got Module SN 0x%08x from EEPROM Read\n",g_module_sn);
	}
	else
		g_module_sn = 0;
#endif
	check_chip_and_update_status(oisCtrl,ois_index);
	if(g_ois_status[ois_index] == 1 || g_ois_status[ois_index] == 3)
	{
	#if UPDATE_FW_AT_PROBE
		pr_info("trigger fw update...ois status 0x%x\n",g_ois_status[ois_index]);
		trigger_result = trigger_fw_update(oisCtrl,0,0,&g_fw_version);
		if(trigger_result == UPDATE_OK || trigger_result == UPDATE_FAIL)//execute FW updating process
		{
			check_chip_and_update_status(oisCtrl,ois_index);
			pr_info("after FW update, ois status is 0x%x\n",g_ois_status[ois_index]);
		}
		else
		{
			pr_info("FW not update\n");
		}
	#endif
	}
	else
	{
		pr_err("check chip failed! ois status 0x%x\n",g_ois_status[ois_index]);
	}
	if(g_ois_status[ois_index] == 1)
		create_ois_proc_files_shipping();
	ois_power_down(oisCtrl);
}

uint8_t get_ois_probe_status(uint32_t index)
{
	#if 0
	//ASUS_BSP Byron no work
	struct cam_ois_ctrl_t *oisCtrl = ois_ctrl[OIS_CLIENT_OV08A10];
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return 0;
	}
	return g_ois_status[OIS_CLIENT_OV08A10];
	#endif
	return 0;
}

void ois_lock(void)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}
	mutex_lock(&oisCtrl->ois_mutex);
}

void ois_unlock(void)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}
	mutex_unlock(&oisCtrl->ois_mutex);
}

void ois_wait_process(void)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}
	ZF7_WaitProcess(oisCtrl,0,__func__);
}

int ois_busy_job_trylock(void)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return 0;
	}
	return mutex_trylock(&g_busy_job_mutex);
}

void ois_busy_job_unlock(void)
{
	struct cam_ois_ctrl_t *oisCtrl = NULL; get_ois_ctrl(&oisCtrl);
	if(oisCtrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}
	mutex_unlock(&g_busy_job_mutex);
}

void set_ois_module_vendor_from_eeprom(uint8_t vendor_id)
{
	if(vendor_id == 0x01)
		g_vendor_id = VENDOR_ID_LITEON;
	else if(vendor_id == 0x06)
		g_vendor_id = VENDOR_ID_PRIMAX;
	pr_info("Got Vendor ID 0x%x from EEPROM DUMP Info\n",g_vendor_id);
}

void set_ois_module_sn_from_eeprom(uint32_t sn)
{
	g_module_sn = sn;
	pr_info("Got Module SN 0x%08x from EEPROM DUMP Info\n",g_module_sn);
}

void set_rear_eeprom_ctrl(struct cam_eeprom_ctrl_t * e_ctrl)
{
	struct cam_sensor_cci_client *cci_client = NULL;

	if(e_ctrl != NULL)
	{
		if (e_ctrl->io_master_info.master_type == CCI_MASTER)
		{
			cci_client = e_ctrl->io_master_info.cci_client;
			if (!cci_client) {
				pr_err("failed: cci_client %pK",cci_client);
				return;
			}
			cci_client->cci_i2c_master = e_ctrl->cci_i2c_master;
			cci_client->sid = (0xA0) >> 1;
			cci_client->retries = 3;
			cci_client->id_map = 0;
			cci_client->i2c_freq_mode = 1;

			g_ectrl = e_ctrl;
			pr_info("config ectrl done!\n");
		}
	}
	else
	{
		pr_err("e_ctrl is NULL!\n");
	}
}

uint8_t ois_allow_vcm_move(void)
{
	return g_vcm_enabled;
}

void set_ois_afc_data_from_eeprom(uint32_t dac_macro, uint32_t dac_infinity,char* physicalSnesorModuleName)
{
	int k;
	if(physicalSnesorModuleName == NULL) {
		pr_err("get physical sensor module name is null\n");
		return;
	}
	if(strncmp(physicalSnesorModuleName,"IMX686_H",8) == 0 && g_ois_status[OIS_CLIENT_IMX686] == 1) {
		g_dac_macro[OIS_CLIENT_IMX686] = dac_macro;
		g_dac_infinity[OIS_CLIENT_IMX686] = dac_infinity;
	}else if(strncmp(physicalSnesorModuleName,"OV08A10_H",9) == 0 && g_ois_status[OIS_CLIENT_OV08A10] == 1) {
		g_dac_macro[OIS_CLIENT_OV08A10] = dac_macro;
		g_dac_infinity[OIS_CLIENT_OV08A10] = dac_infinity;
	}else {
		return;
	}

	for(k=0;k<OIS_CLIENT_MAX;k++) {
		pr_info("Get AFC data[%u], [marco - %u], [infinity - %u]\n",k,g_dac_macro[k], g_dac_infinity[k]);
	}
}

void set_ois_dit_afc_data_from_eeprom(uint32_t dac_macro, uint32_t dac_infinity,char* physicalSnesorModuleName)
{
	int k;
	if(physicalSnesorModuleName == NULL) {
		pr_err("get physical sensor module name is null\n");
		return;
	}
	if(strncmp(physicalSnesorModuleName,"IMX686_H",8) == 0 && g_ois_status[OIS_CLIENT_IMX686] == 1) {
		g_dac_macro_dit[OIS_CLIENT_IMX686] = dac_macro;
		g_dac_infinity_dit[OIS_CLIENT_IMX686] = dac_infinity;
	}else if(strncmp(physicalSnesorModuleName,"OV08A10_H",9) == 0 && g_ois_status[OIS_CLIENT_OV08A10] == 1) {
		g_dac_macro_dit[OIS_CLIENT_OV08A10] = dac_macro;
		g_dac_infinity_dit[OIS_CLIENT_OV08A10] = dac_infinity;
	}else {
		return;
	}
	for(k=0;k<OIS_CLIENT_MAX;k++) {
		pr_info("Get DIT AFC data[%u], [marco - %u], [infinity - %u]\n",k,g_dac_macro_dit[k],g_dac_infinity_dit[k]);
	}
}

static int32_t dac_to_distance_cm(uint32_t dac_value, uint32_t* distance_cm,uint32_t index)
{
	uint32_t shift_value;
	const uint32_t macro_lens_shift[OIS_CLIENT_MAX] = {330,114};// um;
	const uint32_t focal_lens[OIS_CLIENT_MAX] = {5580,7480};  // um;
	const uint32_t max_focus_distance[OIS_CLIENT_MAX] = {5000,5000};  // cm;
	uint32_t distance = 0;

	if(dac_value <= g_dac_infinity_base[index]) {
		distance = max_focus_distance[index];
		goto end;
	}
	//dac larger, shift larger
	// 1.dac -> lens_shift
	shift_value = ((dac_value - g_dac_infinity_base[index]) * macro_lens_shift[index])/(g_dac_macro_base[index] -g_dac_infinity_base[index]);

	// 2.lens_shift -> distance
	distance = (focal_lens[index] * focal_lens[index])/shift_value + focal_lens[index]; // 1/D + 1/(F + L) = 1/F
	distance = distance/1000; //mm
	if(distance % 10 >= 5) {
		*distance_cm = distance /10 +1;
	}else {
		*distance_cm = distance /10;
	}

	end:
	if(g_verbose_log)
		pr_info("DAC %d -> shift %d -> distance %d\n",dac_value,shift_value,*distance_cm);

	return 0;
}
static void set_ssc_gain_if_need(uint32_t ois_index,uint32_t af_state,int32_t distance_cm)
{
	uint32_t current_dac;
	struct timeval t1,t2;
	int rc = 0;
	struct cam_ois_ctrl_t *oisCtrl = ois_ctrl[ois_index];
	static uint32_t prev_distance = 0;

	if( g_ois_power_state[ois_index])
	{
		rc = get_current_lens_position(&current_dac,ois_index);
		if(rc == 0) 
		{
			if(distance_cm < 0) {
				rc = dac_to_distance_cm(current_dac,&distance_cm,ois_index);
				pr_err("get no distance, calculate by dac distance = %d\n",distance_cm);
			}
		}
		else
		{
			pr_err("get_current_lens_position failed ois_index(%u)\n",ois_index);
			rc = -1;
		}

		if(rc == 0) {
			if(distance_cm != prev_distance)
			{
					do_gettimeofday(&t1);
					/*
					if(diff_time_us(&t1,&g_ssc_config_time_prev) < 200*1000)
					{
						pr_info("SKIP %d cm, interval %lld us\n",distance_cm,diff_time_us(&t1,&g_ssc_config_time_prev));
						return;
					}
					*/
					onsemi_config_ssc_gain(oisCtrl,distance_cm);
					do_gettimeofday(&t2);
					pr_info("AF state(%u), CONFIG ssc dac(%u) distance(%u) cm, cost %lld us\n",af_state,current_dac,distance_cm,diff_time_us(&t2,&t1));
					g_ssc_config_time_prev = t1;
					prev_distance = distance_cm;
			}else {
				pr_info("distance_cm is same as previous current_dac(%u) distance_cm(%u) g_distance_prev(%u)\n",current_dac,distance_cm,prev_distance);
			}
		}
	}
}


void asus_ois_init_config(uint32_t ois_index)
{
	pr_info("ois_index = %d\n",ois_index);
	if(ois_ctrl[ois_index] == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}

	if(g_ois_status[ois_index] == 1)
	{
		ZF7_WaitProcess(ois_ctrl[ois_index],0,__func__);
		//onsemi_config_ssc_gain(ois_ctrl,100);//TODO
	}
	else
	{
		pr_err("OIS probe failed, not config ssc gain\n");
	}
	g_ois_power_state[ois_index] = 1;
	g_ois_camera_open[ois_index] = 1;
	g_ois_mode = 255;
	ois_ctrl[ois_index]->ois_on = true;
	memset(&g_ssc_config_time_prev,0,sizeof(struct timeval));

	if(g_dac_macro_dit[ois_index] && g_dac_infinity_dit[ois_index])
	{
		g_dac_macro_base[ois_index] = g_dac_macro_dit[ois_index];
		g_dac_infinity_base[ois_index] = g_dac_infinity_dit[ois_index];
	}
	else
	{
		g_dac_macro_base[ois_index] = g_dac_macro[ois_index];
		g_dac_infinity_base[ois_index] = g_dac_infinity[ois_index];
	}
}

void asus_ois_deinit_config(uint32_t ois_index)
{
	pr_info("ois_index = %d\n",ois_index);
	g_ois_power_state[ois_index] = 0;
	g_ois_camera_open[ois_index] = 0;
	g_ois_mode = 255;
	ois_ctrl[ois_index]->ois_on = 0;
	memset(&g_ssc_config_time_prev,0,sizeof(struct timeval));

	g_dac_macro_base[ois_index] = 0;
	g_dac_infinity_base[ois_index] = 0;
}

void asus_ois_init(struct cam_ois_ctrl_t * ctrl)
{
	if(ctrl && ctrl->soc_info.index < OIS_CLIENT_MAX) {
		ois_ctrl[ctrl->soc_info.index] = ctrl;
		pr_info("E index = %d\n",ctrl->soc_info.index);
	}else
	{
		pr_err("ois_ctrl_t NULL?(%d), index = %d\n",ctrl == NULL?1:0,ctrl->soc_info.index);
		return;
	}
	create_ois_proc_files_factory();

	mutex_init(&g_busy_job_mutex);
	//onsemi_get_50cm_to_10cm_lens_shift(&g_lens_shift_10cm_to_50cm);//TODO
	//onsemi_get_10cm_lens_shift(&g_lens_shift_10cm);//TODO
}
//ASUS_BSP Byron add for sma eeprom dump +++
static void dump_sma_eeprom(struct cam_ois_ctrl_t *oisCtrl,uint32_t addr,uint8_t length) {
	const uint32_t addr_offset = 0x01320300;
	uint32_t check_value;
	uint8_t i=0;
	uint32_t *data;
	data	= (uint32_t *)kzalloc(sizeof(uint32_t)*length,GFP_KERNEL);
	if(data == NULL) {
		pr_err("no memory to alloc\n");
	}

	for(i=0;i<length;i++) {
		addr += i;
		onsemi_write_dword(oisCtrl,0xF01E, 0x01320200);
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_write_dword(oisCtrl,0xF01E, (addr_offset+addr));
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_write_dword(oisCtrl,0xF01E, 0x01320401);
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_write_dword(oisCtrl,0xF01D, 0x01320600);
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_read_dword(oisCtrl,0xF01D, &check_value);
		ZF7_WaitProcess(oisCtrl,0,__func__);
		if(check_value != 0x40) {
			pr_err("get addr(0x%02x) failed, check value is = 0x%x\n",addr,check_value);
			break;
		}
		onsemi_write_dword(oisCtrl,0xF01D, 0x01320000);
		ZF7_WaitProcess(oisCtrl,0,__func__);
		onsemi_read_dword(oisCtrl,0xF01D, (data+i));
		ZF7_WaitProcess(oisCtrl,0,__func__);
		pr_info("get data from addr(0x%02x)= 0x%02x\n",addr,*(data+i));
	}
	if(sysfs_write_byte_from_dword_seq(SMA_OFFSET_DUMP,data,length)) {
		pr_err("write file failed\n");
	}
	kfree(data);
	return;

}
//ASUS_BSP Byron add for sma eeprom dump ---
