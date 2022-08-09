#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
#include <cam_sensor_cmn_header.h>
#include <cam_sensor_i2c.h>
#include <cam_cci_dev.h>

#define EEPROM_ADDR 0x50		//7 bit address (0xa0 >> 1)
#define RYAN_SENSOR_NAME "cam_read_eeprom"

uint8_t eeprom_camera_specs = 0; 
EXPORT_SYMBOL(eeprom_camera_specs);

struct c_ctrl_t {
    struct platform_device *pdev;
    struct cam_subdev v4l2_dev_str;
    struct camera_io_master io_master_info;
	struct regulator *vdd_supply;
    struct msm_pinctrl_info pinctrl_info;
    struct kref ref;
    enum msm_camera_device_type_t device_type;
    enum cci_device_num cci_num;
    enum cci_i2c_master_t cci_master;
    char device_name[20];
};

//创建proc节点
static struct proc_dir_entry *proc_dir = NULL;
static struct proc_dir_entry *proc_file = NULL;

static ssize_t proc_file_read(struct file *file,char __user *buf,size_t count,loff_t *ppos) {
    char str[3] = {0};
    	
    if (*ppos)
    	return 0;
	sprintf(str, "%x", eeprom_camera_specs);
	str[2] = '\n';
	return simple_read_from_buffer(buf, count, ppos, str, sizeof(str));
}

static int proc_file_open(struct inode *inode, struct file *file) {  
    printk(KERN_ALERT "cam_eeprom open!\n");  
    return 0;  
}  

struct file_operations proc_fops = {
	.owner = THIS_MODULE,
    .read = proc_file_read,
    .open = proc_file_open
};

static int create_procfs_entry(void)
{
	proc_dir = proc_mkdir("cam_eeprom_specs",NULL);
	if(!proc_dir) {
		return -1;
	}
	proc_file = proc_create("cam_eeprom_specs",0644,proc_dir,&proc_fops); 
	if(!proc_file) {
		return -1;
	}
	printk("proc_file success!\n");
	return 0;
}

static int msm_c_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    int rc = 0;
    return rc;
}

static const struct v4l2_subdev_internal_ops msm_c_internal_ops = {
    .close = msm_c_close,
};

static long msm_c_subdev_ioctl(struct v4l2_subdev *sd,unsigned int cmd, void *arg)
{
    int32_t rc = 0;
    return rc;
}

static int32_t msm_c_power(struct v4l2_subdev *sd, int on)
{
    return 0;
}

static struct v4l2_subdev_core_ops msm_c_subdev_core_ops = {
    .ioctl = msm_c_subdev_ioctl,
    .s_power = msm_c_power,
};

static struct v4l2_subdev_ops msm_c_subdev_ops = {
    .core = &msm_c_subdev_core_ops,
};

static int eeprom_get_dt_info(struct device *dev, struct c_ctrl_t *c_ctrl) {
	int rc = 0;
	struct device_node *of_node = NULL;
	
	if (!dev || !c_ctrl || !dev->of_node)
        return -EINVAL;
        
    of_node = dev->of_node;
    
    rc = of_property_read_u32(of_node, "cci-master", &c_ctrl->cci_master);
    if (rc < 0) {
        printk("eeprom_get_dt_info failed to read cci master%d\n", __LINE__);
        return rc;
    }
    
	c_ctrl->vdd_supply = devm_regulator_get(dev, "cam_read_eeprom");
	if (IS_ERR(c_ctrl->vdd_supply)) {
		rc = PTR_ERR(c_ctrl->vdd_supply);
		if (rc != -EPROBE_DEFER)
			printk("Failed to get vdd regulator");
		return rc;
	}
	return rc;
}

int cam_read_eeprom_probe(struct platform_device *pdev)
{
    uint8_t rc = 0;
    uint32_t data = 0;
    struct c_ctrl_t *c_ctrl = NULL;
    struct cam_sensor_cci_client *cci_client = NULL;
	
	if (!cam_cci_get_subdev(1)) {
		printk("cam_read_eeprom_probe cam_cci_get_subdev() not ready\n");
		return -EPROBE_DEFER;	//defer the probe call.
	}
	
	c_ctrl = kzalloc(sizeof(struct c_ctrl_t), GFP_KERNEL);
    if (!c_ctrl) {
       return -ENOMEM;
    }
    
    cci_client = kzalloc(sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
    if (!cci_client) {
		kfree(c_ctrl);
        return -ENOMEM;
    }
  
    rc = create_procfs_entry();  
    if (rc != 0) {
    	printk("create_procfs_entry failed, rc=%d\n",rc);
		//goto kfree;
    }
 
	rc = eeprom_get_dt_info(&pdev->dev, c_ctrl);
	if (rc < 0) {
		printk("eeprom_get_dt_info failed, rc=%d\n",rc);
		goto kfree;
	}
	
	rc = regulator_enable(c_ctrl->vdd_supply);
	if (rc < 0) {
		printk("Enable VDD supply failed, rc=%d\n",rc);
		goto kfree;
	}
	
    c_ctrl->pdev = pdev;
    c_ctrl->device_type = MSM_CAMERA_PLATFORM_DEVICE;
    c_ctrl->io_master_info.master_type = CCI_MASTER;
    c_ctrl->io_master_info.cci_client = kzalloc(sizeof(struct cam_sensor_cci_client),GFP_KERNEL);
    if (!c_ctrl->io_master_info.cci_client) {
        kfree(c_ctrl);
        kfree(cci_client);
        return -ENOMEM;
    }
    //c_ctrl->cci_master = MASTER_0;
    c_ctrl->cci_num = CCI_DEVICE_0;
    c_ctrl->io_master_info.cci_client->cci_device = c_ctrl->cci_num;
    
    cci_client = c_ctrl->io_master_info.cci_client;
    cci_client->cci_i2c_master = c_ctrl->cci_master;
    cci_client->sid = EEPROM_ADDR;
    cci_client->retries = 3;
    cci_client->id_map = 0;
    cci_client->i2c_freq_mode = I2C_FAST_MODE;

	c_ctrl->v4l2_dev_str.internal_ops = &msm_c_internal_ops;
    c_ctrl->v4l2_dev_str.ops = &msm_c_subdev_ops;
    strlcpy(c_ctrl->device_name, RYAN_SENSOR_NAME, sizeof(c_ctrl->device_name));
    c_ctrl->v4l2_dev_str.name = c_ctrl->device_name;
    c_ctrl->v4l2_dev_str.sd_flags = (V4L2_SUBDEV_FL_HAS_EVENTS);
    c_ctrl->v4l2_dev_str.ent_function = CAM_EEPROM_DEVICE_TYPE;   
    c_ctrl->v4l2_dev_str.token = c_ctrl;
	
	rc = camera_io_init(&c_ctrl->io_master_info);
    if (rc < 0) {
    	printk("cam_read_eeprom_probe cci init failed: rc: %d", rc);
		goto kfree;
	}
    rc = cam_register_subdev(&(c_ctrl->v4l2_dev_str));
    if (rc) {
        printk("fail to create subdev\n");
        cam_unregister_subdev(&(c_ctrl->v4l2_dev_str));
        goto kfree;
    }

	cam_cci_i2c_read(c_ctrl->io_master_info.cci_client, 0x08, &data, CAMERA_SENSOR_I2C_TYPE_WORD,CAMERA_SENSOR_I2C_TYPE_BYTE);//address type is 2 byte.
	printk("cam read eeprom data is 0x%x!",data);
	eeprom_camera_specs = data;
	printk("cam read eeprom data is %x  %d!",eeprom_camera_specs,sizeof(eeprom_camera_specs));
	
	rc = camera_io_release(&c_ctrl->io_master_info);
    if (rc < 0) {
    	printk("cam_read_eeprom_probe camera_io_release failed: rc: %d", rc);
		goto kfree;
	}
	
	rc = regulator_disable(c_ctrl->vdd_supply);
	if (rc < 0) {
		printk("Disable VDD supply failed, rc=%d\n",rc);
		goto kfree;
	}
	
	cam_unregister_subdev(&(c_ctrl->v4l2_dev_str));
    return rc;

kfree:
	kfree(c_ctrl);
	kfree(cci_client);
	return rc;
}

static const struct of_device_id cam_of_match[] = {
        { .compatible = "camera,read_eeprom", },
        { },
};
MODULE_DEVICE_TABLE(of, cam_of_match);

static struct platform_driver cam_platform_driver = {
        .probe          = cam_read_eeprom_probe,
        .driver = {
                .name   = "readeeprom",
                .owner  = THIS_MODULE,
                .of_match_table = cam_of_match,
        },
};

static int __init cam_read_eeprom_init(void)
{
	int ret;
	
    ret = platform_driver_register(&cam_platform_driver);
    if (ret)
        printk("cam_read_eeprom_init error ret=%d\n", ret);	

    printk("cam_read_eeprom_init init finish!");
    return ret;
}

static void __exit cam_read_eeprom_exit()
{
	remove_proc_entry("fortune",proc_dir);
}

late_initcall(cam_read_eeprom_init);
module_exit(cam_read_eeprom_exit);

MODULE_LICENSE("GPL v2");
