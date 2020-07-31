#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include "asus_cam_sensor.h"
#include "asus_actuator.h"
#include "actuator_i2c.h"

#undef  pr_fmt
#define pr_fmt(fmt) "ACTUATOR-ATD %s(): " fmt, __func__

#define	PROC_POWER	"driver/actuator_power"
#define	PROC_I2C_RW	"driver/actuator_i2c_rw"
#define	PROC_VCM_ENABLE	"driver/vcm_enable"
#define	PROC_VCM_VALUE	"driver/vcm"
#define	PROC_DSI_CHECK	"driver/cam_csi_check"  //ASUS_BSP Bryant "Add for camera csi debug"
#ifdef ASUS_DXO
#define VCM_NUMBER 3 //ASUS_BSP for jason_yeh support multi camera vcm
#else
#define VCM_NUMBER 1 //ASUS_BSP for jason_yeh support multi camera vcm
#endif
static struct cam_actuator_ctrl_t * actuator_ctrl = NULL;
static struct cam_actuator_ctrl_t * g_actuator_ctrl[3] ; //ASUS_BSP for jason_yeh support multi camera vcm

static struct mutex g_busy_job_mutex;

uint8_t g_actuator_power_state = 0;
uint8_t g_actuator_camera_open = 0;

static uint8_t g_atd_status = 0;//fail
static uint16_t vcm_dac_reg_addr[3]={0x84,0x084,0xF01A}; //imx686,imx363,ov08a //ASUS_BSP for jason_yeh support multi camera vcm

#ifdef ASUS_DXO
static uint16_t g_reg_addr = 0xF01A;
static uint16_t g_slave_id = 0x24;
#else
static uint16_t g_reg_addr = 0x84;
static uint16_t g_slave_id = 0x72;
#endif

static uint32_t g_reg_val = 0;
static uint8_t g_vcm_enabled = 1;
uint8_t g_cam_csi_check = 0;  //ASUS_BSP Bryant "Add for camera csi debug"

static enum camera_sensor_i2c_type g_data_type = CAMERA_SENSOR_I2C_TYPE_WORD; //word
static enum camera_sensor_i2c_type g_addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;

static uint8_t g_operation = 0;//read

void actuator_lock(void)
{
	if(actuator_ctrl == NULL)
	{
		pr_err("ACTUATOR not init!!!\n");
		return;
	}
	mutex_lock(&actuator_ctrl->actuator_mutex);
}

void actuator_unlock(void)
{
	if(actuator_ctrl == NULL)
	{
		pr_err("ACTUATOR not init!!!\n");
		return;
	}
	mutex_unlock(&actuator_ctrl->actuator_mutex);
}

int actuator_busy_job_trylock(void)
{
	if(actuator_ctrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return 0;
	}
	return mutex_trylock(&g_busy_job_mutex);
}

void actuator_busy_job_unlock(void)
{
	if(actuator_ctrl == NULL)
	{
		pr_err("OIS not init!!!\n");
		return;
	}
	mutex_unlock(&g_busy_job_mutex);
}

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

int32_t trans_dac_value(uint32_t input_dac)
{
	int32_t HallMax = 0x6000;
	int32_t HallMin = 0xEA80;
	int TotalRange;
	int32_t DAC;

	if(input_dac < 1 || input_dac > 4095)
	{
		CAM_INFO(CAM_ACTUATOR,"Input DAC value is wrong");
		return -1;
	}
	if(input_dac >0) input_dac -=1;

	TotalRange = (HallMax - 0x0000) + (0xFFFF - HallMin)+1;

	DAC = HallMin + ((TotalRange * input_dac)/4094);

	if(DAC > 0xFFFF ) DAC -= 0xFFFF;

	return DAC;
}
/*
int onsemi_actuator_init_setting(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc;
	uint32_t reg_addr[2] = {0xf0, 0xe0};
	uint32_t reg_data;
	uint32_t reg_input_data = 0x1;

	CAM_INFO(CAM_ACTUATOR,"onsemi_actuator_init_setting E\n");

	rc = actuator_read_byte(a_ctrl, reg_addr[0], &reg_data);
	if( rc < 0)
		CAM_INFO(CAM_ACTUATOR,"Read 0x%x failed data 0x%x\n", reg_addr[0], reg_data);

	if( reg_data == 0x42)
	{
		CAM_INFO(CAM_ACTUATOR,"0xf0 data is 0x%x! Right, do next cmd\n", reg_data);

		rc = actuator_write_byte(a_ctrl,reg_addr[1],reg_input_data);
		if(rc == 0)
		{
			CAM_INFO(CAM_ACTUATOR,"Write addr 0x%x data 0x%x success! Start delay\n",
				reg_addr[1], reg_input_data);
			mdelay(5);
			CAM_INFO(CAM_ACTUATOR,"Write addr 0x%x data 0x%x success! End delay\n",
				reg_addr[1], reg_input_data);
		}
		if( rc < 0 )
			CAM_INFO(CAM_ACTUATOR,"Write 0x%x failed\n", reg_addr[1]);
		else
		{
			reg_data = 0x5;
			rc = actuator_read_byte(a_ctrl, reg_addr[1], &reg_data);
			CAM_INFO(CAM_ACTUATOR,"Read 0x%x data 0x%x\n", reg_addr[1], reg_data);
			if(reg_data == 0x0)
				g_actuator_init_setting = 0;
		}
	}
	else
		CAM_INFO(CAM_ACTUATOR,"0xf0 data is not 0x42: 0x%x\n", reg_data);

	CAM_INFO(CAM_ACTUATOR,"onsemi_actuator_init_setting X\n");
	return rc;
}*/


static int actuator_i2c_debug_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val;
	int rc;

	mutex_lock(&actuator_ctrl->actuator_mutex);
  g_actuator_power_state = 1;
	if(g_actuator_power_state)
	{
		//F40_WaitProcess(ois_ctrl,0,__func__);
		actuator_ctrl->io_master_info.cci_client->sid = g_slave_id;
		switch(g_data_type)
		{
			case CAMERA_SENSOR_I2C_TYPE_BYTE:
				rc = actuator_read_byte(actuator_ctrl,g_reg_addr,&reg_val, g_addr_type);
				break;
			case CAMERA_SENSOR_I2C_TYPE_WORD:
				rc = actuator_read_word(actuator_ctrl,g_reg_addr,&reg_val, g_addr_type);
				break;
			case CAMERA_SENSOR_I2C_TYPE_DWORD:
				rc = actuator_read_dword(actuator_ctrl,g_reg_addr,&reg_val, g_addr_type);
				break;
			default:
				rc = actuator_read_dword(actuator_ctrl,g_reg_addr,&reg_val, g_addr_type);
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

	mutex_unlock(&actuator_ctrl->actuator_mutex);

	return 0;
}

static int actuator_i2c_debug_open(struct inode *inode, struct  file *file)
{
	return single_open(file, actuator_i2c_debug_read, NULL);
}
static ssize_t actuator_i2c_debug_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	int n;
	char messages[32]="";
	uint32_t val[4];
	int rc;
	int32_t vcm_register_value = 0;

	ret_len = len;
	if (len > 32) {
		len = 32;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	n = sscanf(messages,"%x %x %x %x",&val[0],&val[1],&val[2],&val[3]);

	mutex_lock(&actuator_ctrl->actuator_mutex);

	if(n == 1)
	{
		g_reg_addr = val[0];
		g_operation = 0;
		g_data_type = 2;//default dword
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

//ASUS_BSP for jason_yeh +++ support multi camera vcm
		g_reg_addr = val[0];
		g_data_type = val[1];
		g_reg_val = val[2];
//		g_slave_id = val[3];//slave id
		if(val[3]<VCM_NUMBER ){
			actuator_ctrl=g_actuator_ctrl[val[3]];
			g_reg_addr=vcm_dac_reg_addr[val[3]];
		}
		else {
			pr_err("camera_dir=%d not define FAIL\n",val[3]);
			mutex_unlock(&actuator_ctrl->actuator_mutex);
			return ret_len;
		}
//ASUS_BSP for jason_yeh --- support multi camera vcm
		g_operation = 1;
	}



	if(g_data_type != 1 && g_data_type != 2 && g_data_type != 4 )
		g_data_type = 4;//default dword
	pr_info("gona %s SLAVE 0x%X reg 0x%02x, data type %s\n",
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
		switch(g_data_type)
		{
			case CAMERA_SENSOR_I2C_TYPE_BYTE:

				rc = actuator_write_byte(actuator_ctrl,g_reg_addr,g_reg_val,g_addr_type);
				break;
			case CAMERA_SENSOR_I2C_TYPE_WORD:
				if(g_reg_addr == 0xFFFE)
				{
					g_reg_addr = 0xA0;
					//vcm_register_value = trans_dac_value(g_reg_val);
					rc = actuator_write_word(actuator_ctrl,g_reg_addr,vcm_register_value,g_addr_type);
					//pr_info("Dac value 0x%x register value 0x%04x\n",g_reg_val,vcm_register_value);
				}
				else
					rc = actuator_write_word(actuator_ctrl,g_reg_addr,g_reg_val,g_addr_type);
				break;
			case CAMERA_SENSOR_I2C_TYPE_DWORD:
				rc = actuator_write_dword(actuator_ctrl,g_reg_addr,g_reg_val,g_addr_type);
				break;
			default:
				rc = actuator_write_byte(actuator_ctrl,g_reg_addr,g_reg_val,g_addr_type);
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
		}
	}

	mutex_unlock(&actuator_ctrl->actuator_mutex);

	return ret_len;
}
static const struct file_operations actuator_i2c_debug_fops = {
	.owner = THIS_MODULE,
	.open = actuator_i2c_debug_open,
	.read = seq_read,
	.write = actuator_i2c_debug_write,
	.llseek = seq_lseek,
	.release = single_release,
};
static int actuator_allow_vcm_move_read(struct seq_file *buf, void *v)
{
  mutex_lock(&actuator_ctrl->actuator_mutex);
	seq_printf(buf, "%d\n", g_vcm_enabled);
	mutex_unlock(&actuator_ctrl->actuator_mutex);
	return 0;
}

static int actuator_allow_vcm_move_open(struct inode *inode, struct  file *file)
{
	return single_open(file, actuator_allow_vcm_move_read, NULL);
}
static ssize_t actuator_allow_vcm_move_write(struct file *dev, const char *buf, size_t len, loff_t *ppos)
{
	ssize_t ret_len;
	char messages[8]="";

	uint32_t val;

	ret_len = len;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buf, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages, "%d", &val);

	mutex_lock(&actuator_ctrl->actuator_mutex);

	if(val == 0)
		g_vcm_enabled = 0;
	else
		g_vcm_enabled = 1;

	pr_info("vcm enabled set to %d\n", g_vcm_enabled);

	mutex_unlock(&actuator_ctrl->actuator_mutex);

	return ret_len;
}

static const struct file_operations actuator_allow_vcm_move_fops = {
	.owner = THIS_MODULE,
	.open = actuator_allow_vcm_move_open,
	.read = seq_read,
	.write = actuator_allow_vcm_move_write,
	.llseek = seq_lseek,
	.release = single_release,
};
static int actuator_read_vcm_dac_read(struct seq_file *buf, void *v)
{
	uint32_t reg_val = 0;
	int rc;

	mutex_lock(&actuator_ctrl->actuator_mutex);
  g_actuator_power_state = 1;
	if(g_actuator_power_state)
	{
//		actuator_ctrl->io_master_info.cci_client->sid = g_slave_id;
//ASUS_BSP for jason_yeh  +++ support multi camera vcm
		pr_err("%s: g_reg_addr  = %x\n", __func__, g_reg_addr);
		if(g_reg_addr==0xF01A){ //for ov089a
			rc = actuator_read_dword(actuator_ctrl, g_reg_addr, &reg_val, CAMERA_SENSOR_I2C_TYPE_WORD);
		}
		else{
			rc = actuator_read_word(actuator_ctrl, g_reg_addr, &reg_val, CAMERA_SENSOR_I2C_TYPE_BYTE);
		}
//ASUS_BSP for jason_yeh --- support multi camera vcm
		if(rc == 0)
		{
			g_atd_status = 1;
		}
		else
		{
			g_atd_status = 0;
			pr_err("read from reg 0x%x failed! rc = %d\n",g_reg_addr,rc);
		}

		seq_printf(buf,"%d\n",(reg_val & 0x7FF)); //ASUS_Byron for (&0x7FF) only need to observer 0 ~ 10 bit for AF
	}
	else
	{
		seq_printf(buf,"POWER DOWN\n");
	}

	mutex_unlock(&actuator_ctrl->actuator_mutex);

	return 0;
}
static int actuator_read_vcm_dac_open(struct inode *inode, struct  file *file)
{
//ASUS_BSP for jason_yeh +++ support multi camera vcm
	char path[256];
	char * vcm_file_name= NULL;
	char temp[2];
	long vcm_open_number=0;
	int ret=0;
	vcm_file_name=d_path(&file->f_path, path, 256);
	temp[0]=vcm_file_name[strlen(vcm_file_name)-1];
	temp[1]='\0';
	ret=kstrtol(temp, 10, &vcm_open_number) ;
      pr_info(" actuator_read_vcm_dac_open descriptor for file %s strlen=%d result=%d test=%d\n",temp,strlen(temp),vcm_open_number,ret);
	actuator_ctrl= g_actuator_ctrl[vcm_open_number];
	g_reg_addr=vcm_dac_reg_addr[vcm_open_number];
//ASUS_BSP for jason_yeh ---support multi camera vcm

	return single_open(file, actuator_read_vcm_dac_read, NULL);
}

static const struct file_operations actuator_read_vcm_dac_fops = {
		.owner = THIS_MODULE,
		.open = actuator_read_vcm_dac_open,
		.read = seq_read,
		.llseek = seq_lseek,
		.release = single_release,
};

//ASUS_BSP Bryant +++ "Add for camera csi debug"
static ssize_t cam_csi_check_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	pr_err("camera csi check %d\n", g_cam_csi_check);

	len += sprintf(buff, "%d\n", g_cam_csi_check);
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}


static ssize_t cam_csi_check_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
		g_cam_csi_check = 0;
		pr_err("[CAM] write g_cam_csi_check is %d\n", g_cam_csi_check);
	}

	return len;
}


static struct file_operations cam_csi_check_fops = {
	.read  = cam_csi_check_read,
	.write = cam_csi_check_write,
};
//ASUS_BSP Bryant --- "Add for camera csi debug"


static int ois_actuator_power_read(struct seq_file *buf, void *v)
{
    mutex_lock(&actuator_ctrl->actuator_mutex);
    pr_info("g_actuator_power_state = %d\n", g_actuator_power_state);
	seq_printf(buf,"%d\n",g_actuator_power_state);
	mutex_unlock(&actuator_ctrl->actuator_mutex);
	return 0;
}

static int actuator_solo_power_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_actuator_power_read, NULL);
}

//just for ATD test
static ssize_t actuator_solo_power_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	int val;
	int rc;
	int i;
	struct cam_hw_soc_info         *soc_info = &actuator_ctrl->soc_info;
	struct cam_actuator_soc_private     *soc_private =
		(struct cam_actuator_soc_private *)actuator_ctrl->soc_info.soc_private;

	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	ret_len = len;
	if (len > 16) {
		len = 16;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);
	mutex_lock(&actuator_ctrl->actuator_mutex);
	if(g_actuator_camera_open == 0)
	{
		if(val == 0)
		{
			if(g_actuator_power_state == 1)
			{
				camera_io_release(&(actuator_ctrl->io_master_info));
				rc = cam_sensor_util_power_down(power_info, soc_info);
				if (rc) {
					pr_err("%s: msm_camera_power_down fail rc = %d\n", __func__, rc);
				}
				else
				{
					g_actuator_power_state = 0;
					pr_info("ACTUATOR POWER DOWN\n");
				}
			}
			else
			{
				pr_info("ACTUATOR already power off, do nothing\n");
			}
		}
		else
		{
			if(g_actuator_power_state == 0)
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
				if (rc) {
					pr_err("%s: msm_camera_power_up fail rc = %d\n", __func__, rc);
				}
				else
				{
					g_actuator_power_state = 1;
					camera_io_init(&(actuator_ctrl->io_master_info));
					//rc = onsemi_actuator_init_setting(actuator_ctrl); //tmp disable
					if(rc < 0)
					{
					  pr_err("%s: onsemi_actuator_init_setting failed rc = %d", __func__, rc);
					}
					pr_info("ACTUATOR POWER UP\n");
				}
			}
			else
			{
				pr_info("ACTUATOR already power up, do nothing\n");
			}
		}
	}
	else
	{
		pr_err("camera has been opened, can't control actuator power\n");
	}
	mutex_unlock(&actuator_ctrl->actuator_mutex);
	return ret_len;
}
static const struct file_operations actuator_solo_power_fops = {
	.owner = THIS_MODULE,
	.open = actuator_solo_power_open,
	.read = seq_read,
	.write = actuator_solo_power_write,
	.llseek = seq_lseek,
	.release = single_release,
};

int actuator_power_up(struct cam_actuator_ctrl_t *actuator_ctrl)
{

	int rc;
	struct cam_hw_soc_info         *soc_info = &actuator_ctrl->soc_info;
	struct cam_actuator_soc_private     *soc_private =
		(struct cam_actuator_soc_private *)actuator_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;


	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		pr_err("actuator power up failed, rc %d\n", rc);
		return -1;
	}
	if (actuator_ctrl->io_master_info.master_type == CCI_MASTER)
	{
		rc = camera_io_init(&(actuator_ctrl->io_master_info));
		if (rc < 0) {
			pr_err("cci init failed!\n");
			rc = cam_sensor_util_power_down(power_info, soc_info);
			if (rc) {
				pr_err("actuator power down failed, rc %d\n", rc);
			}
			return -2;
		}
	}
	pr_err("Actuator probe power up!");
	return rc;
}

int actuator_power_down(struct cam_actuator_ctrl_t *actuator_ctrl)
{

	int rc;
	struct cam_hw_soc_info         *soc_info = &actuator_ctrl->soc_info;
	struct cam_actuator_soc_private     *soc_private =
		(struct cam_actuator_soc_private *)actuator_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;


    pr_err("actuator_power_down :E");
	if(actuator_ctrl->io_master_info.master_type == CCI_MASTER)
	{
		rc = camera_io_release(&(actuator_ctrl->io_master_info));
		if (rc < 0)
			pr_err("cci release failed!\n");
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		pr_err("actuator power down failed, rc %d\n", rc);
	}
	pr_err("Actuator probe power down!");
	return rc;
}

void actuator_probe_check(void)
{
	if(actuator_ctrl == NULL)
	{
		pr_err("actuator_ctrl is NULL!!!\n");
		return;
	}
	if(actuator_power_up(actuator_ctrl) != 0)
	{
		pr_err("actuator power up failed\n");
		return;
	}

	mdelay(1000);

	actuator_power_down(actuator_ctrl);
}

uint8_t asus_allow_vcm_move(void)
{
	return g_vcm_enabled;
}

static void create_actuator_proc_files_factory(int index)
{
	static uint8_t has_created = 0;
	char actuator_proc_file[32]="";
	if(!has_created)
	{
		create_proc_file(PROC_POWER, &actuator_solo_power_fops);
		create_proc_file(PROC_I2C_RW,&actuator_i2c_debug_fops);//ATD
		create_proc_file(PROC_VCM_ENABLE, &actuator_allow_vcm_move_fops);
		create_proc_file(PROC_DSI_CHECK, &cam_csi_check_fops);  //ASUS_BSP Bryant "Add for camera csi debug"
		has_created = 1;
	}
	else//ASUS_BSP for jason_yeh +++ support multi camera vcm
	{
		pr_err("ACTUATOR factory proc files have already created!\n");
	}
//ASUS_BSP for jason_yeh +++ support multi camera vcm

	sprintf(actuator_proc_file,"%s%d",PROC_VCM_VALUE,index);
	pr_info("ACTUATOR factory proc=%s\n",actuator_proc_file);
	create_proc_file(actuator_proc_file, &actuator_read_vcm_dac_fops);
//ASUS_BSP for jason_yeh --- support multi camera vcm

}

void asus_actuator_init(struct cam_actuator_ctrl_t * ctrl)
{
	if(ctrl){
//ASUS_BSP for jason_yeh +++ support multi camera vcm
		actuator_ctrl= ctrl;
		g_actuator_ctrl[ctrl->soc_info.index]= ctrl;
//ASUS_BSP for jason_yeh --- support multi camera vcm
	}
	else
	{
		pr_err("actuator_ctrl_t passed in is NULL!\n");
		return;
	}
	pr_err("asus_actuator_init :E create_actuator_proc_files_factory =%d\n",ctrl->soc_info.index);
	create_actuator_proc_files_factory(ctrl->soc_info.index); //ASUS_BSP for jason_yeh support multi camera vcm
	mutex_init(&g_busy_job_mutex);
}

int get_current_lens_position(uint32_t *dac_value,uint32_t index)
{
	if(index == 0) {
		if(g_actuator_ctrl[CAMERA_0])
		{
			mutex_lock(&(g_actuator_ctrl[CAMERA_0]->actuator_mutex));
			*dac_value = g_actuator_ctrl[CAMERA_0]->lens_pos;
			mutex_unlock(&(g_actuator_ctrl[CAMERA_0]->actuator_mutex));
		}
		else
		{
			pr_err("actuator not init!\n");
			return -1;
		}
	}else if(index == 1) {
		if(g_actuator_ctrl[CAMERA_2])
		{
			mutex_lock(&(g_actuator_ctrl[CAMERA_2]->actuator_mutex));
			*dac_value = g_actuator_ctrl[CAMERA_2]->lens_pos;
			mutex_unlock(&(g_actuator_ctrl[CAMERA_2]->actuator_mutex));
		}
		else
		{
			pr_err("actuator not init!\n");
			return -1;
		}
	}else {
		//do nothing
		pr_err("get index is wrong index = %u\n",index);
		return -1;
	}
	//pr_info("ois index(%u) dac_value(%u)\n",index,*dac_value);
	return 0;
}

