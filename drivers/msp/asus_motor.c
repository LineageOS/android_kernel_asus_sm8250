#include <linux/proc_fs.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include "MSP430FR2311.h"
#include <linux/seq_file.h>

#define PROC_MOTOR_POWER        "driver/motor_power"
#define PROC_MOTOR_MANUAL_MODE  "driver/motor_manual"
#define PROC_MOTOR_AUTO_MODE    "driver/motor_auto"
#define PROC_MOTOR_STOP         "driver/motor_stop"
#define	PROC_MOTOR_ATD_STATUS	"driver/motor_atd_status"
#define	PROC_MOTOR_PROBE_STATUS "driver/motor_probe_status"
#define PROC_MOTOR_PARAM_MODE    "driver/motor_param"
#define PROC_MOTOR_ANGLE_MODE   "driver/motor_angle"
#define PROC_MOTOR_DRV_MODE     "driver/motor_drv"
#define PROC_MOTOR_AKM_MODE     "driver/motor_akm"

static struct MSP430FR2311_info * motor_ctrl = NULL;

unsigned char g_motor_status = 0;

uint8_t g_motor_power_state = 0;

uint8_t g_motor_mode = 255;

uint8_t g_motor_camera_open = 0;

uint16_t g_motor_dir = 255;

uint16_t g_motor_degree = 255;

uint16_t g_motor_speed = 255;

static uint8_t g_atd_status = 0;//fail

static int motor_atd_status_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", g_atd_status);
	g_atd_status = 0;//default is failure

	return 0;
}

static int motor_atd_status_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, motor_atd_status_proc_read, NULL);
}
static ssize_t motor_atd_status_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;
	char messages[16]="";
	uint32_t val;

	rc = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);

	switch(val)
	{
		case 0:
			g_atd_status = 0;
			break;
		case 1:
			g_atd_status = 1;
			break;
		default:
			g_atd_status = 1;
	}

	pr_info("ATD status changed to %d\n",g_atd_status);

	return rc;
}
static const struct file_operations motor_atd_status_fops = {
	.owner = THIS_MODULE,
	.open = motor_atd_status_proc_open,
	.read = seq_read,
	.write = motor_atd_status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int motor_solo_power_read(struct seq_file *buf, void *v)
{
    pr_info("g_motor_power_state = %d\n", g_motor_power_state);
	seq_printf(buf,"%d\n",g_motor_power_state);
	return 0;
}

static int motor_solo_power_open(struct inode *inode, struct  file *file)
{
	return single_open(file, motor_solo_power_read, NULL);
}

static ssize_t motor_solo_power_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	static int count =0;
	char messages[16]="";
	int val;
	int rc;
	ret_len = len;
	if (len > 16) {
		len = 16;
	}
	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);

	//if(g_motor_camera_open == 0)
	//{
		if(val == 0)
		{
			if(g_motor_power_state == 1)
			{
				count --;
				if(count == 0)
				{
					rc = MSP430FR2311_power_control(0);

					if (rc) {
						pr_err("%s: motor power off fail rc = %d\n", __func__, rc);
					}
					else
					{
						g_motor_power_state = 0;
						pr_info("Motor POWER DOWN\n");
					}
				}
				else
				{
					pr_err("count is %d, not call power down!\n",count);
				}
			}
			else
			{
				pr_err("Motor not power up, do nothing for powering down\n");
			}
		}
		else
		{
			count ++;
			if(count == 1)
			{
				rc = MSP430FR2311_power_control(1);

				if (rc) {
					pr_err("%s: motor power up fail rc = %d\n", __func__, rc);
				}
				else
				{
					g_motor_power_state = 1;
					pr_info("Motor POWER UP\n");
				}
			}
			else
			{
				pr_err("count is %d, not call power up!\n",count);
			}
		}
	//}
	//else
	//{
	//	count = 0;	
	//	pr_err("camera has been opened, can't control motor power\n");
	//}

	return ret_len;
}
static const struct file_operations motor_solo_power_fops = {
	.owner = THIS_MODULE,
	.open = motor_solo_power_open,
	.read = seq_read,
	.write = motor_solo_power_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int motor_manual_proc_read(struct seq_file *buf, void *v)
{
    pr_info("[Lucien] g_motor_dir %d g_motor_degree %d g_motor_speed %d\n", g_motor_dir, g_motor_degree, g_motor_speed);
	return 0;
}

static int motor_manual_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, motor_manual_proc_read, NULL);
}

static ssize_t motor_manual_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint16_t val[3];
	int rc = 0;

	ret_len = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d %d %d",&val[0], &val[1], &val[2]);

    g_motor_dir = val[0];
	g_motor_degree = val[1];
	g_motor_speed = val[2];

    switch(g_motor_dir)
    {
		case 0:
			pr_info("[Lucien] Motor toward up\n");
			break;
		case 1:
			pr_info("[Lucien] Motor toward down\n");
			break;
		default:
			pr_err("[Lucien] Not supported command %d\n",g_motor_dir);
			rc = -1;
    }

    pr_info("[Lucien] degree %d speed %d\n", g_motor_degree, g_motor_speed);

    if(rc < 0 || g_motor_degree < 0 || g_motor_degree > 180)
    {
        g_atd_status = 0;
    }
	else
	{
	    if(g_motor_speed > 10) g_motor_speed = 10;
	    //do motor manual here
	    rc = MSP430FR2311_Set_ManualMode(g_motor_dir, g_motor_degree, g_motor_speed); //speed set to 5 by default

		if(rc < 0)
		    g_atd_status = 0;
	    else
		    g_atd_status = 1;
	}

	return ret_len;
}
static const struct file_operations motor_manual_mode_fops = {
	.owner = THIS_MODULE,
	.open = motor_manual_proc_open,
	.read = seq_read,
	.write = motor_manual_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};


static int motor_auto_proc_read(struct seq_file *buf, void *v)
{
    pr_info("[Lucien] g_motor_dir %d g_motor_degree %d\n", g_motor_dir, g_motor_degree);

	return 0;
}

static int motor_auto_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, motor_auto_proc_read, NULL);
}

static ssize_t motor_auto_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint16_t val[2];
	int rc = 0;

	ret_len = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d %d",&val[0], &val[1]);

    pr_info("[Lucien] auto mode %d, Angle=%d\n", val[0], val[1]);

    //do motor auto here
    rc = MSP430FR2311_Set_AutoModeWithAngle(val[0], val[1]);

	if(rc < 0)
		g_atd_status = 0;
	else
		g_atd_status = 1;

	return ret_len;
}
static const struct file_operations motor_auto_mode_fops = {
	.owner = THIS_MODULE,
	.open = motor_auto_proc_open,
	.read = seq_read,
	.write = motor_auto_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int motor_stop_proc_read(struct seq_file *buf, void *v)
{
    pr_info("[Lucien] g_motor_dir %d g_motor_degree %d\n", g_motor_dir, g_motor_degree);
	return 0;
}

static int motor_stop_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, motor_stop_proc_read, NULL);
}

static ssize_t motor_stop_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint16_t val;
	int rc = 0;

	ret_len = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d",&val);

    //pr_info("[Lucien] degree %d\n", g_motor_degree);

    //do motor stop here
    rc = MSP430FR2311_Stop();

	if(rc < 0)
		g_atd_status = 0;
	else
		g_atd_status = 1;

	return ret_len;
}
static const struct file_operations motor_stop_mode_fops = {
	.owner = THIS_MODULE,
	.open = motor_stop_proc_open,
	.read = seq_read,
	.write = motor_stop_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int motor_probe_status_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", g_motor_status);
	return 0;
}

static int motor_probe_status_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, motor_probe_status_proc_read, NULL);
}

static const struct file_operations motor_probe_status_fops = {
	.owner = THIS_MODULE,
	.open = motor_probe_status_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


extern char gFWVersion[];

static int motor_param_proc_read(struct seq_file *buf, void *v)
{
	
	int rc = MSP430FR2311_Set_AutoMode(221);
		if (rc==0) {
			seq_printf(buf, "[MCU] Firmware version=%d%02d%02d%02X\n", 
			gFWVersion[0], gFWVersion[1], gFWVersion[2], gFWVersion[3]
			);
		} else {
			seq_printf(buf, "[MCU] Firmware version fail!!\n"); 
		}
	return rc;
}

static int motor_param_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, motor_param_proc_read, NULL);
}

static ssize_t motor_param_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
#define MAX_PARAM_MSG_LEN 100
	char messages[MAX_PARAM_MSG_LEN]="";
	char Zen7messages[MAX_PARAM_MSG_LEN]="";
	uint16_t val[14];
	uint16_t Zen7Val[20];	//Dir freq*6 step*6 mode*6 EndFlage(0xFD).
	int rc = 0;
	
	if (len>MAX_PARAM_MSG_LEN) len=MAX_PARAM_MSG_LEN;

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}else{
		//Delete will lead to Zen7Val's value error.
		memcpy(Zen7messages, messages, sizeof(messages));
	}

	sscanf(messages,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d",&val[0], &val[1], &val[2],&val[3], &val[4], &val[5],&val[6], &val[7], &val[8],&val[9], &val[10], &val[11], &val[12], &val[13]);

	pr_info("[MCU] dump param write = %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",val[0], val[1], val[2],val[3], val[4], val[5],val[6], val[7], val[8],val[9], val[10], val[11], val[12], val[13]);

	if (val[13] != 0xfe) {
		//Make sure that, zen7's 'mode value range is 0~10.
		if(1){	//(((val[13]&0x0F) >= 0) && ((val[13]&0x0F) <= 10))
			sscanf(messages,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",&Zen7Val[0], &Zen7Val[1], &Zen7Val[2], &Zen7Val[3], &Zen7Val[4], &Zen7Val[5], &Zen7Val[6], &Zen7Val[7], &Zen7Val[8], &Zen7Val[9], &Zen7Val[10], &Zen7Val[11], &Zen7Val[12], &Zen7Val[13], &Zen7Val[14], &Zen7Val[15], &Zen7Val[16], &Zen7Val[17], &Zen7Val[18], &Zen7Val[19]);

			pr_err("[MCU] long cmd = %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				Zen7Val[0], Zen7Val[1], Zen7Val[2], Zen7Val[3], Zen7Val[4], Zen7Val[5], Zen7Val[6], Zen7Val[7], Zen7Val[8], Zen7Val[9], Zen7Val[10], Zen7Val[11], Zen7Val[12], 
				Zen7Val[13], Zen7Val[14], Zen7Val[15], Zen7Val[16], Zen7Val[17], Zen7Val[18], Zen7Val[19]);

			
			if(Zen7Val[19] != 0xfe){
				pr_err("[MCU] long cmd's param error, syntax error should be 20 parameters with 254 final end");
				rc=-1;
			}else{
				//Zen7 format.
				rc = Zen7_MSP430FR2311_Set_ParamMode(Zen7Val);
			}
		}else{
			pr_err("[MCU] read param error, syntax error should be 13 parameters with 254 final end");
			rc=-1;
		}
	} else {
		rc = MSP430FR2311_Set_ParamMode(val); //speed set to 5 by default
	}

	if(rc < 0)
	    g_atd_status = 0;
    else
	    g_atd_status = 1;

	return len;
}
static const struct file_operations motor_param_mode_fops = {
	.owner = THIS_MODULE,
	.open = motor_param_proc_open,
	.read = seq_read,
	.write = motor_param_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

//===============Zen7===============
//Proc note for read angle sendor's angle raw data.
extern uint8_t dAngle[4];
static int motor_angle_proc_read(struct seq_file *buf, void *v)
{
	uint16_t cmd = 0x62;

	int rc = Zen7_MSP430FR2311_DealAngle(&cmd, 1);
	
	if (rc == 0) {
		seq_printf(buf, "[MCU] Angle raw data:%02X %02X %02X %02X\n", 
		dAngle[0], dAngle[1], dAngle[2], dAngle[3]);
	} else {
		seq_printf(buf, "[MCU] Get angle fail!\n"); 
	}

	return 0;
}

static int motor_angle_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, motor_angle_proc_read, NULL);
}

static ssize_t motor_angle_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint16_t val[3];
	int rc = 0;

	ret_len = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d %d %d",&val[0], &val[1], &val[2]);

    pr_err("%s [MCU] val[0]:%d, val[1]:%d, val[2]:%d\n", __func__, val[0], val[1], val[2]);

    rc = Zen7_MSP430FR2311_DealAngle(val, 3);

	if(rc < 0)
		g_atd_status = 0;
	else
		g_atd_status = 1;

	return ret_len;
}

static const struct file_operations motor_angle_mode_fops = {
	.owner = THIS_MODULE,
	.open = motor_angle_proc_open,
	.read = seq_read,
	.write = motor_angle_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

//Proc note for r/w re-driver IC.
extern uint8_t drv_state[7];
static int motor_drv_proc_read(struct seq_file *buf, void *v)
{
	uint16_t cmd = 0x21;
	int rc = Zen7_MSP430FR2311_wrDrv(&cmd, 1);
	
	if (rc == 0) {
		seq_printf(buf, "[MCU] drv state:%02X %02X %02X %02X %02X %02X %02X\n", 
		drv_state[0], drv_state[1], drv_state[2], drv_state[3], drv_state[4], drv_state[5], drv_state[6]);
	} else {
		seq_printf(buf, "[MCU] Get drv_state fail!\n"); 
	}

	return 0;
}

static int motor_drv_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, motor_drv_proc_read, NULL);
}

static ssize_t motor_drv_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t ret_len;
	char messages[16]="";
	uint16_t val[5];
	int rc = 0;

	ret_len = len;

	if (len > 16) {
		len = 16;
	}

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}

	sscanf(messages,"%d %d %d %d %d",&val[0], &val[1], &val[2], &val[3] ,&val[4]);

	switch(val[0]){
		case 0x20:
			pr_err("%s [MCU] val[0]:%d\n", __func__, val[0]);
			rc = Zen7_MSP430FR2311_wrDrv(val, 1);				
			break;

		case 0x40:
	    	pr_err("%s [MCU] val[0]:%d, val[1]:%d, val[2]:%d, val[3]:%d, val[4]:%d\n", __func__, val[0], val[1], val[2], val[3], val[4]);
			rc = Zen7_MSP430FR2311_wrDrv(val, 5);			
			break;
			
		case 0x60:
			pr_err("%s [MCU] val[0]:%d, val[1]:%d\n", __func__, val[0], val[1]);
			rc = Zen7_MSP430FR2311_wrDrv(val, 2);			
			break;	

		default:
			pr_err("%s [MCU] param error!",  __func__);
			rc=-1;
			break;
	}

	if(rc < 0)
		g_atd_status = 0;
	else
		g_atd_status = 1;

	return ret_len;
}

static const struct file_operations motor_drv_mode_fops = {
	.owner = THIS_MODULE,
	.open = motor_drv_proc_open,
	.read = seq_read,
	.write = motor_drv_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

//Proc note for r/w angle IC.
extern uint8_t akm_temp[8];
static int motor_akm_proc_read(struct seq_file *buf, void *v)
{	
	uint16_t cmd = 0x67;
	int rc = Zen7_MSP430FR2311_wrAKM(&cmd, 1);
	
	if (rc == 0) {
		seq_printf(buf, "[MCU] akm raw data(reg:0x17):%02X %02X %02X %02X %02X %02X %02X %02X\n", 
		akm_temp[0], akm_temp[1], akm_temp[2], akm_temp[3], akm_temp[4], akm_temp[5], akm_temp[6], akm_temp[7]);
	} else {
		seq_printf(buf, "[MCU] Get akm raw data fail!\n"); 
	}

	return 0;
}

static int motor_akm_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, motor_akm_proc_read, NULL);
}

static ssize_t motor_akm_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
#define MAX_PARAM_MSG_LEN 100
	char messages[MAX_PARAM_MSG_LEN]="";
	char Zen7messages[MAX_PARAM_MSG_LEN]="";
	uint16_t val[30];
	int rc = 0;
	
	if (len>MAX_PARAM_MSG_LEN) len=MAX_PARAM_MSG_LEN;

	if (copy_from_user(messages, buff, len)) {
		pr_err("%s command fail !!\n", __func__);
		return -EFAULT;
	}else{
		//Delete will lead to Zen7Val's value error.
		memcpy(Zen7messages, messages, sizeof(messages));
	}

	//Max length(25): cmd + 24bytes.
	sscanf(Zen7messages,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",&val[0], &val[1], &val[2], &val[3], &val[4], &val[5], &val[6], &val[7], &val[8], &val[9], &val[10], &val[11], &val[12], &val[13], &val[14], &val[15], &val[16], &val[17], &val[18], &val[19], &val[20], &val[21], &val[22], &val[23], &val[24]);
	/*
	sscanf(messages,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", &val[0],	\
									  &val[1], &val[2], &val[3], &val[4], &val[5], &val[6], &val[7], &val[8],	\
									  &val[9], &val[10], &val[11], &val[12], &val[13], &val[14], &val[15], &val[16],	\
									  &val[17], &val[18], &val[19], &val[20], &val[21], &val[22], &val[23], &val[24]);
	*/
	switch(val[0]){
		case 0x65:
	    	pr_err("%s [MCU] val[0]:%d, val[1]:%d, val[2]:%d, val[3]:%d, val[4]:%d ... val[21]:%d, val[22]:%d, val[23]:%d, val[24]:%d.\n", __func__,	\
	    	val[0], val[1], val[2], val[3], val[4], val[21], val[22], val[23], val[24]);
	
			rc = Zen7_MSP430FR2311_wrAKM(val, 25);			
			break;

		case 0x66:
			pr_err("%s [MCU] val[0]:%d\n", __func__, val[0]);
			rc = Zen7_MSP430FR2311_wrAKM(val, 1);				
			break;

		case 0x68:
			pr_err("%s [MCU] val[0]:%d\n", __func__, val[0]);
			rc = Zen7_MSP430FR2311_wrAKM(val, 1);				
			break;
			
		default:
			pr_err("%s [MCU] param error!",  __func__);
			rc=-1;
			break;
	}

	if(rc < 0)
		g_atd_status = 0;
	else
		g_atd_status = 1;

	return len;
}

static const struct file_operations motor_akm_mode_fops = {
	.owner = THIS_MODULE,
	.open = motor_akm_proc_open,
	.read = seq_read,
	.write = motor_akm_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};
	
//===============Zen7===============

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

static void create_motor_proc_files_factory(void)
{
	static uint8_t has_created = 0;

	if(!has_created)
	{
        create_proc_file(PROC_MOTOR_POWER, &motor_solo_power_fops);
		create_proc_file(PROC_MOTOR_MANUAL_MODE, &motor_manual_mode_fops);
		create_proc_file(PROC_MOTOR_AUTO_MODE, &motor_auto_mode_fops);
		create_proc_file(PROC_MOTOR_STOP, &motor_stop_mode_fops);
		create_proc_file(PROC_MOTOR_ATD_STATUS, &motor_atd_status_fops);
		create_proc_file(PROC_MOTOR_PROBE_STATUS, &motor_probe_status_fops);
		create_proc_file(PROC_MOTOR_PARAM_MODE, &motor_param_mode_fops);
		create_proc_file(PROC_MOTOR_ANGLE_MODE, &motor_angle_mode_fops);
		create_proc_file(PROC_MOTOR_DRV_MODE, &motor_drv_mode_fops);
		create_proc_file(PROC_MOTOR_AKM_MODE, &motor_akm_mode_fops);
		has_created = 1;
	}
	else
	{
		pr_err("Motor factory proc files have already created!\n");
	}
}

void asus_motor_init(struct MSP430FR2311_info * ctrl)
{
	if(ctrl)
		motor_ctrl = ctrl;
	else
	{
		pr_err("msm_ois_ctrl_t passed in is NULL!\n");
		return;
	}
	create_motor_proc_files_factory();
}


