 /*
  * Goodix Touchscreen Driver
  * Core layer of touchdriver architecture.
  *
  * Copyright (C) 2019 - 2020 Goodix, Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be a reference
  * to you, when you are integrating the GOODiX's CTP IC into your system,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  * General Public License for more details.
  *
  */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/of_irq.h>
#include <linux/time.h>

#include "goodix_ts_core.h"

#define GOOIDX_INPUT_PHYS	"goodix_ts/input0"
#define PINCTRL_STATE_ACTIVE    "pmx_ts_active"
#define PINCTRL_STATE_SUSPEND   "pmx_ts_suspend"

#ifdef ASUS_ZS661KS_PROJECT
extern bool g_Recovery_mode;
#endif

static int goodix_ts_remove(struct platform_device *pdev);
int goodix_start_later_init(struct goodix_ts_core *ts_core);
void goodix_ts_dev_release(void);
static int goodix_ts_resume(struct goodix_ts_core *core_data);
static int goodix_ts_suspend(struct goodix_ts_core *core_data);

// ASUS_BSP +++ Touch
static void input_switch_key(struct input_dev *dev, unsigned int code);
static int goodix_ts_switch_sample_rate(struct goodix_ts_core *core_data);
static int goodix_ts_rotation(struct goodix_ts_core *core_data, int value);
static void goodix_resume_work(struct work_struct *work);
void ATR_touch_new(struct device *dev, int id,int action, int x, int y, int random);
int goodix_change_config(struct goodix_ts_core *core_data, const char *fw_name);
void charge_mode_enable(struct goodix_ts_core *core_data, bool en);
int read_chip_cmd(struct goodix_ts_core *core_data, u16 cmd_addr, int buf_len, u8 *buffer);

#define GLOVE                  "driver/glove"

static int print_touch_count_max;
static int ts_9886_fod_position[4] = {440, 640, 1960, 2140};
//static int ts_9896_fod_position[4] = {410, 681, 1651, 1938};
static int ts_9896_fod_position[4] = {435, 641, 1679, 1868}; //CFG(47)
static int *fod_position = NULL;
static int LastATR = 0;
static int LastATL = 0;
static int SampleRateLocked = 1;
static int FPArea = 6;
static bool aod_press = false;
static int key_i = -1;
static bool key_o_sync = false;
static int fp_status = -1;
static bool process_resume = false;
static bool out_of_KEY_O = false;
static bool out_of_KEY_F = false;
bool enable_touch_debug = false;
bool enable_touch_time_debug = false;

bool GoodixTSEnTimestamp = false;
bool GoodixTSEnTimestampDebug = false;
bool GoodixTSEnInputTimestampDebug = false;
bool GoodixTSKeyMappingDebug = false;
bool GoodixTSINTDebug = false;
bool finger_press = false;
int GoodixSampleRate = 240;
int testkeycode = 0;
int touch_figer_slot[TOTAL_SLOT] = {0};

struct goodix_ts_core *gts_core_data = NULL;
// ASUS_BSP --- Touch

// ASUS_BSP +++ Touch - ATR
struct atr_queue *atr_buf_queue = NULL;

static ssize_t atr_queue_full(struct atr_queue *q);
static ssize_t atr_queue_empty(struct atr_queue *q);
static ssize_t atr_buf_write(struct atr_queue *q, u8 id, u8 active, u16 x, u16 y, u16 p, u16 m);
static ssize_t atr_buf_read(struct atr_queue *q, struct input_dev *input_dev);
static struct atr_queue* atr_buf_init(unsigned int _capacity);
// ASUS_BSP --- Touch - ATR

struct goodix_module goodix_modules;

// ASUS_BSP +++ Touch
extern char asus_var_panel_stage[3];
extern bool asus_var_regulator_always_on;
extern u8 TouchArea;
extern int data_x;
extern int data_y;

extern bool asus_display_in_normal_off(void);
extern void enable_aod_processing(bool en);
extern bool get_aod_processing(void);
extern int asus_display_global_hbm_mode(void);
extern void asus_display_report_fod_touched(void);
// ASUS_BSP --- Touch
/**
 * __do_register_ext_module - register external module
 * to register into touch core modules structure
 */
static void  __do_register_ext_module(struct work_struct *work)
{
	struct goodix_ext_module *module =
			container_of(work, struct goodix_ext_module, work);
	struct goodix_ext_module *ext_module, *next;
	struct list_head *insert_point = &goodix_modules.head;

	ts_info("__do_register_ext_module IN");
	ts_info("register external module %s",module->name);

	if (goodix_modules.core_data &&
	    !goodix_modules.core_data->initialized) {
		ts_err("core layer has exit");
		return;
	}

	if (!goodix_modules.core_data) {
		/* waitting for core layer */
		if (!wait_for_completion_timeout(&goodix_modules.core_comp,
						 30 * HZ)) {
			ts_err("Module [%s] timeout", module->name);
			return;
		}
	}

	/* driver probe failed */
	if (!goodix_modules.core_data ||
	    !goodix_modules.core_data->initialized) {
		ts_err("Can't register ext_module core error");
		return;
	}

	ts_info("start register ext_module");

	/* prority level *must* be set */
	if (module->priority == EXTMOD_PRIO_RESERVED) {
		ts_err("Priority of module [%s] needs to be set",
		       module->name);
		return;
	}

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			if (ext_module == module) {
				ts_info("Module [%s] already exists",
					module->name);
				mutex_unlock(&goodix_modules.mutex);
				return;
			}
		}

		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			/* small value of priority have
			 * higher priority level
			 */
			if (ext_module->priority >= module->priority) {
				insert_point = &ext_module->list;
				break;
			}
		}
	}

	if (module->funcs && module->funcs->init) {
		if (module->funcs->init(goodix_modules.core_data,
					module) < 0) {
			ts_err("Module [%s] init error",
			       module->name ? module->name : " ");
			mutex_unlock(&goodix_modules.mutex);
			return;
		}
	}

	list_add(&module->list, insert_point->prev);
	goodix_modules.count++;
	mutex_unlock(&goodix_modules.mutex);

	ts_info("Module [%s] registered,priority:%u",
		module->name,
		module->priority);
}

/**
 * goodix_register_ext_module - interface for external module
 * to register into touch core modules structure
 *
 * @module: pointer to external module to be register
 * return: 0 ok, <0 failed
 */
int goodix_register_ext_module(struct goodix_ext_module *module)
{
	if (!module)
		return -EINVAL;

	if (!goodix_modules.initilized) {
		ts_info("goodix_modules.initilized is false");
		goodix_modules.initilized = true;
		INIT_LIST_HEAD(&goodix_modules.head);
		mutex_init(&goodix_modules.mutex);
		init_completion(&goodix_modules.core_comp);
	}

	ts_info("goodix_register_ext_module IN");

	INIT_WORK(&module->work, __do_register_ext_module);
	schedule_work(&module->work);

	ts_info("goodix_register_ext_module OUT");

	return 0;
}
EXPORT_SYMBOL_GPL(goodix_register_ext_module);

/**
 * goodix_unregister_ext_module - interface for external module
 * to unregister external modules
 *
 * @module: pointer to external module
 * return: 0 ok, <0 failed
 */
int goodix_unregister_ext_module(struct goodix_ext_module *module)
{
	struct goodix_ext_module *ext_module, *next;
	bool found = false;

	if (!module)
		return -EINVAL;

	if (!goodix_modules.initilized)
		return -EINVAL;

	if (!goodix_modules.core_data)
		return -ENODEV;

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			if (ext_module == module) {
				found = true;
				break;
			}
		}
	} else {
		mutex_unlock(&goodix_modules.mutex);
		return -EFAULT;
	}

	if (!found) {
		ts_err("Module [%s] never registed",
				module->name);
		mutex_unlock(&goodix_modules.mutex);
		return -EFAULT;
	}

	list_del(&module->list);
	goodix_modules.count--;
	mutex_unlock(&goodix_modules.mutex);

	if (module->funcs && module->funcs->exit)
		module->funcs->exit(goodix_modules.core_data, module);

	ts_info("Moudle [%s] unregistered",
		module->name ? module->name : " ");
	return 0;
}
EXPORT_SYMBOL_GPL(goodix_unregister_ext_module);

static void goodix_remove_all_ext_modules(void)
{
	struct goodix_ext_module *ext_module, *next;

	if (!goodix_modules.initilized || !goodix_modules.core_data)
		return;

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			list_del(&ext_module->list);
			goodix_modules.count--;
			if (ext_module->funcs && ext_module->funcs->exit)
				ext_module->funcs->exit(goodix_modules.core_data,
							ext_module);
		}
	}

	mutex_unlock(&goodix_modules.mutex);
}

static void goodix_ext_sysfs_release(struct kobject *kobj)
{
	ts_info("Kobject released!");
}

#define to_ext_module(kobj)	container_of(kobj,\
				struct goodix_ext_module, kobj)
#define to_ext_attr(attr)	container_of(attr,\
				struct goodix_ext_attribute, attr)

static ssize_t goodix_ext_sysfs_show(struct kobject *kobj,
		struct attribute *attr, char *buf)
{
	struct goodix_ext_module *module = to_ext_module(kobj);
	struct goodix_ext_attribute *ext_attr = to_ext_attr(attr);

	if (ext_attr->show)
		return ext_attr->show(module, buf);

	return -EIO;
}

static ssize_t goodix_ext_sysfs_store(struct kobject *kobj,
		struct attribute *attr, const char *buf, size_t count)
{
	struct goodix_ext_module *module = to_ext_module(kobj);
	struct goodix_ext_attribute *ext_attr = to_ext_attr(attr);

	if (ext_attr->store)
		return ext_attr->store(module, buf, count);

	return -EIO;
}

static const struct sysfs_ops goodix_ext_ops = {
	.show = goodix_ext_sysfs_show,
	.store = goodix_ext_sysfs_store
};

static struct kobj_type goodix_ext_ktype = {
	.release = goodix_ext_sysfs_release,
	.sysfs_ops = &goodix_ext_ops,
};

struct kobj_type *goodix_get_default_ktype(void)
{
	return &goodix_ext_ktype;
}
EXPORT_SYMBOL_GPL(goodix_get_default_ktype);

struct kobject *goodix_get_default_kobj(void)
{
	struct kobject *kobj = NULL;

	if (goodix_modules.core_data &&
			goodix_modules.core_data->pdev)
		kobj = &goodix_modules.core_data->pdev->dev.kobj;
	return kobj;
}
EXPORT_SYMBOL_GPL(goodix_get_default_kobj);

/* debug fs */
struct debugfs_buf {
	struct debugfs_blob_wrapper buf;
	int pos;
	struct dentry *dentry;
} goodix_dbg;

void goodix_msg_printf(const char *fmt, ...)
{
	va_list args;
	int r;

	if (!goodix_dbg.dentry)
		return;
	if (goodix_dbg.pos < goodix_dbg.buf.size) {
		va_start(args, fmt);
		r = vscnprintf(goodix_dbg.buf.data + goodix_dbg.pos,
			       goodix_dbg.buf.size - 1, fmt, args);
		goodix_dbg.pos += r;
		va_end(args);
	}
}
EXPORT_SYMBOL_GPL(goodix_msg_printf);

static int goodix_debugfs_init(void)
{
	struct dentry *r_b;

	goodix_dbg.buf.size = PAGE_SIZE;
	goodix_dbg.pos = 0;
	goodix_dbg.buf.data = kzalloc(goodix_dbg.buf.size, GFP_KERNEL);
	if (goodix_dbg.buf.data == NULL) {
		pr_err("Debugfs init failed\n");
		goto exit;
	}
	r_b = debugfs_create_blob("goodix_ts", 0644, NULL, &goodix_dbg.buf);
	if (!r_b) {
		pr_err("Debugfs create failed\n");
		return -ENOENT;
	}
	goodix_dbg.dentry = r_b;

exit:
	return 0;
}

static void goodix_debugfs_exit(void)
{
	debugfs_remove(goodix_dbg.dentry);
	goodix_dbg.dentry = NULL;
	pr_info("Debugfs module exit\n");
}

/* show external module infomation */
static ssize_t goodix_ts_extmod_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ext_module *module, *next;
	size_t offset = 0;
	int r;

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(module, next,
					 &goodix_modules.head, list) {
			r = snprintf(&buf[offset], PAGE_SIZE,
				     "priority:%u module:%s\n",
				     module->priority, module->name);
			if (r < 0) {
				mutex_unlock(&goodix_modules.mutex);
				return -EINVAL;
			}
			offset += r;
		}
	}

	mutex_unlock(&goodix_modules.mutex);
	return offset;
}

/* show driver infomation */
static ssize_t goodix_ts_driver_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "DriverVersion:%s\n",
			GOODIX_DRIVER_VERSION);
}

/* show chip infoamtion */
static ssize_t goodix_ts_chip_info_show(struct device  *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	struct goodix_ts_version chip_ver;
	int r, cnt = 0;
	
	if(core_data==NULL || ts_dev==NULL)
		return cnt;
	if (core_data->initialized!=1)
		return 0;
	// For I2C power
	if(atomic_read(&core_data->suspended) == 1) {
		input_switch_key(core_data->input_dev, KEY_POWER);
		msleep(100);
	}

	cnt += snprintf(buf, PAGE_SIZE, "TouchDeviceName:%s\n", ts_dev->name);
	if (ts_dev->hw_ops->read_version) {
		r = ts_dev->hw_ops->read_version(ts_dev, &chip_ver);
		if (!r && chip_ver.valid) {
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
				"PID:%s\nVID:%02x.%02x.%02x.%02x\nSensID:%02x\n",
				chip_ver.pid, chip_ver.vid[0],
				chip_ver.vid[1], chip_ver.vid[2],
				chip_ver.vid[3], chip_ver.sensor_id);
		}
	}

	return cnt;
}

/* reset chip */
static ssize_t goodix_ts_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	if (en != 1)
		return -EINVAL;

	if (ts_dev->hw_ops->reset)
		ts_dev->hw_ops->reset(ts_dev);
	return count;

}

static ssize_t goodix_ts_read_cfg_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int ret, i, offset;
	char *cfg_buf;
	
	if(core_data==NULL || ts_dev==NULL)
		return 0;
	if(core_data->initialized != 1)
		return -EINVAL;
	// For I2C power
	if(atomic_read(&core_data->suspended) == 1) {
		input_switch_key(core_data->input_dev, KEY_POWER);
		msleep(100);
	}

	cfg_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!cfg_buf)
		return -ENOMEM;
	if (ts_dev->hw_ops->read_config) {
// ASUS_BSP +++ Touch
		disable_irq(core_data->irq);
		ret = ts_dev->hw_ops->read_config(ts_dev, cfg_buf);
		enable_irq(core_data->irq);
// ASUS_BSP --- Touch
	 } else
		ret = -EINVAL;

	if (ret > 0) {
		offset = 0;
		for (i = 0; i < ret; i++) {
			if (i != 0 && i % 20 == 0) {
				buf[offset++] = '\n';
			}
// ASUS_BSP +++ Touch
			if (offset < PAGE_SIZE ) {  
				offset += snprintf(&buf[offset], PAGE_SIZE - offset,
						"%02x ", cfg_buf[i]);
			} else {
				ts_info ("Out off PAGE_SIZE");
				break;
			}
// ASUS_BSP --- Touch 
		}
	}
	kfree(cfg_buf);
	if (ret <= 0)
		return ret;

	return offset;
}

static u8 ascii2hex(u8 a)
{
	s8 value = 0;

	if (a >= '0' && a <= '9')
		value = a - '0';
	else if (a >= 'A' && a <= 'F')
		value = a - 'A' + 0x0A;
	else if (a >= 'a' && a <= 'f')
		value = a - 'a' + 0x0A;
	else
		value = 0xff;

	return value;
}

static int goodix_ts_convert_0x_data(const u8 *buf, int buf_size,
				     unsigned char *out_buf, int *out_buf_len)
{
	int i, m_size = 0;
	int temp_index = 0;
	u8 high, low;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] == 'x' || buf[i] == 'X')
			m_size++;
	}

	if (m_size <= 1) {
		ts_err("cfg file ERROR, valid data count:%d\n", m_size);
		return -EINVAL;
	}
	*out_buf_len = m_size;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] != 'x' && buf[i] != 'X')
			continue;

		if (temp_index >= m_size) {
			ts_err("exchange cfg data error, overflow,"
			       "temp_index:%d,m_size:%d\n",
			       temp_index, m_size);
			return -EINVAL;
		}
		high = ascii2hex(buf[i + 1]);
		low = ascii2hex(buf[i + 2]);
		if (high == 0xff || low == 0xff) {
			ts_err("failed convert: 0x%x, 0x%x",
				buf[i + 1], buf[i + 2]);
			return -EINVAL;
		}
		out_buf[temp_index++] = (high << 4) + low;
	}
	return 0;
}

static ssize_t goodix_ts_send_cfg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int en, r;
	const struct firmware *cfg_img;
	struct goodix_ts_config *config = NULL;
// ASUS_BSP +++ Touch
	char cfg_name[32] = {0x00};
// ASUS_BSP --- Touch

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	if (en != 1)
		return -EINVAL;

	disable_irq(core_data->irq);

	/*request configuration*/
// ASUS_BSP +++ Touch
	if (asus_var_panel_stage[0]!='B'){
		strlcpy(cfg_name, GOODIX_YODA_CFG_NAME, sizeof(cfg_name));
	}else{
		strlcpy(cfg_name, GOODIX_DEFAULT_CFG_NAME, sizeof(cfg_name));
	}

	r = request_firmware(&cfg_img, cfg_name, dev);
	if (r < 0) {
		ts_err("cfg file [%s] not available,errno:%d",
		       cfg_name, r);
		goto exit;
	} else
		ts_info("cfg file [%s] is ready", cfg_name);
// ASUS_BSP --- Touch
	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (config == NULL)
		goto exit;

	/*parse cfg data*/
	if (goodix_ts_convert_0x_data(cfg_img->data, cfg_img->size,
				      config->data, &config->length)) {
		ts_err("convert config data FAILED");
		goto exit;
	}

	config->reg_base = ts_dev->reg.cfg_addr;
	mutex_init(&config->lock);
	config->initialized = TS_CFG_STABLE;

	if (ts_dev->hw_ops->send_config)
		ts_dev->hw_ops->send_config(ts_dev, config);

exit:
	enable_irq(core_data->irq);
	kfree(config);
	config = NULL;
	if (cfg_img) {
		release_firmware(cfg_img);
		cfg_img = NULL;
	}

	return count;
}

/* show irq infomation */
static ssize_t goodix_ts_irq_info_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct irq_desc *desc;
	size_t offset = 0;
	int r;

	r = snprintf(&buf[offset], PAGE_SIZE, "irq:%u\n", core_data->irq);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "state:%s\n",
		     atomic_read(&core_data->irq_enabled) ?
		     "enabled" : "disabled");
	if (r < 0)
		return -EINVAL;

	desc = irq_to_desc(core_data->irq);
	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "disable-depth:%d\n",
		     desc->depth);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "trigger-count:%zu\n",
		core_data->irq_trig_cnt);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset,
		     "echo 0/1 > irq_info to disable/enable irq");
	if (r < 0)
		return -EINVAL;

	offset += r;
	return offset;
}

/* enable/disable irq */
static ssize_t goodix_ts_irq_info_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	goodix_ts_irq_enable(core_data, en);
	return count;
}

/*reg read/write */
static u16 rw_addr;
static u32 rw_len;
static u8 rw_flag;
static u8 store_buf[32];
static u8 show_buf[PAGE_SIZE];
static ssize_t goodix_ts_reg_rw_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	if (!rw_addr || !rw_len) {
		ts_err("address(0x%x) and length(%d) cann't be null\n",
			rw_addr, rw_len);
		return -EINVAL;
	}

	if (rw_flag != 1) {
		ts_err("invalid rw flag %d, only support [1/2]", rw_flag);
		return -EINVAL;
	}
	
	// For I2C power
	if(atomic_read(&core_data->suspended) == 1) {
		input_switch_key(core_data->input_dev, KEY_POWER);
		msleep(100);
	}

	ret = ts_dev->hw_ops->read(ts_dev, rw_addr, show_buf, rw_len);
	if (ret) {
		ts_err("failed read addr(%x) length(%d)\n", rw_addr, rw_len);
		return snprintf(buf, PAGE_SIZE,
				"failed read addr(%x), len(%d)\n",
				rw_addr, rw_len);
	}

	return snprintf(buf, PAGE_SIZE, "0x%x,%d {%*ph}\n",
			rw_addr, rw_len, rw_len, show_buf);
}

static ssize_t goodix_ts_reg_rw_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	char *pos = NULL, *token = NULL;
	long result = 0;
	int ret, i;

	if (!buf || !count) {
		ts_err("invalid params\n");
		goto err_out;
	}

	if (buf[0] == 'r') {
		rw_flag = 1;
	} else if (buf[0] == 'w') {
		rw_flag = 2;
	} else {
		ts_err("string must start with 'r/w'\n");
		goto err_out;
	}
	
	// For I2C power
	if(atomic_read(&core_data->suspended) == 1) {
		input_switch_key(core_data->input_dev, KEY_POWER);
		msleep(100);
	}

	/* get addr */
	pos = (char *)buf;
	pos += 2;
	token = strsep(&pos, ":");
	if (!token) {
		ts_err("invalid address info\n");
		goto err_out;
	} else {
		if (kstrtol(token, 16, &result)) {
			ts_err("failed get addr info\n");
			goto err_out;
		}
		rw_addr = (u16)result;
		ts_info("rw addr is 0x%x\n", rw_addr);
	}

	/* get length */
	token = strsep(&pos, ":");
	if (!token) {
		ts_err("invalid length info\n");
		goto err_out;
	} else {
		if (kstrtol(token, 0, &result)) {
			ts_err("failed get length info\n");
			goto err_out;
		}
		rw_len = (u32)result;
		ts_info("rw length info is %d\n", rw_len);
		if (rw_len > sizeof(store_buf)) {
			ts_err("data len > %lu\n", sizeof(store_buf));
			goto err_out;
		}
	}

	if (rw_flag == 1)
		return count;

	for (i = 0; i < rw_len; i++) {
		token = strsep(&pos, ":");
		if (!token) {
			ts_err("invalid data info\n");
			goto err_out;
		} else {
			if (kstrtol(token, 16, &result)) {
				ts_err("failed get data[%d] info\n", i);
				goto err_out;
			}
			store_buf[i] = (u8)result;
			ts_info("get data[%d]=0x%x\n", i, store_buf[i]);
		}
	}
	ret = ts_dev->hw_ops->write(ts_dev, rw_addr, store_buf, rw_len);
	if (ret) {
		ts_err("failed write addr(%x) data %*ph\n", rw_addr,
			rw_len, store_buf);
		goto err_out;
	}

	ts_info("%s write to addr (%x) with data %*ph\n",
		"success", rw_addr, rw_len, store_buf);

	return count;
err_out:
	snprintf(show_buf, PAGE_SIZE, "%s\n",
		"invalid params, format{r/w:4100:length:[41:21:31]}");
	return -EINVAL;
}

// ASUS_BSP +++ Touch
static ssize_t goodix_ts_power_on_off(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
		
	int en, result;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	goodix_ts_irq_enable(core_data, en);
	
	if (core_data->power_on == 1 && en == 0) {
		result = goodix_ts_power_off(core_data);
	}
	if (core_data->power_on == 0 && en == 1)
		result = goodix_ts_power_on(core_data);
	ts_info("power operation status %d",result);
	
	return count;

}
static ssize_t goodix_ts_power_state(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
    struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n",core_data->power_on);
}

static ssize_t goodix_ts_print_count_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n",print_touch_count_max);
}

static ssize_t goodix_ts_print_count_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t count)
{
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	print_touch_count_max = en;

	ts_info("print count max =  %d", print_touch_count_max);
	
	return count;

}

static ssize_t goodix_aod_test_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	
	if(core_data == NULL)
		return snprintf(buf, PAGE_SIZE, "error\n");

	return snprintf(buf, PAGE_SIZE, "%d\n",core_data->aod_test_mode);
}

static ssize_t goodix_aod_test_mode_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
  	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
    ts_info("system enable aod test mode %d",en);

	core_data->aod_test_mode = en;

	if(((core_data->aod_test_mode == 0) || (core_data->aod_test_mode == 2)) && ((process_resume == false) || (fp_status == 2))) {
		ts_info("touch resume : vendor.asus.touch_control_fod = %d", core_data->aod_test_mode);
		process_resume = true;
		wake_up_interruptible(&core_data->fp_queue);
	}

	return count;
}

static ssize_t goodix_timestamp_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",GoodixTSEnTimestamp);
}

static ssize_t goodix_timestamp_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
	ts_info("system enable goodix timestamp %d",en);

	GoodixTSEnTimestamp = en;

	return count;
}

static ssize_t goodix_timestamp_debug_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",GoodixTSEnTimestampDebug);
}

static ssize_t goodix_timestamp_debug_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
	ts_info("system enable goodix timestamp %d",en);

	GoodixTSEnTimestampDebug = en;

	return count;
}

static ssize_t goodix_evdev_timestamp_debug_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",GoodixTSEnInputTimestampDebug);
}

static ssize_t goodix_evdev_timestamp_debug_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int en;

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
	ts_info("system enable evdev timestamp debug %d",en);

	GoodixTSEnInputTimestampDebug = en;

	return count;
}

static ssize_t test_cfg_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int en, r;
	char cfg_name[32] = {0x00};

	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;

	ts_info("update test cfg %d",en);

	if(en == 0 && atomic_read(&core_data->testcfg)!=1){
		ts_info("previous status not update test cfg");
		return count;
	}

	if (asus_var_panel_stage[0]!='B'){
		if (en == 1){
			strlcpy(cfg_name, GOODIX_TEST_YODA_CFG_NAME, sizeof(cfg_name));
		}else{
			strlcpy(cfg_name, GOODIX_YODA_CFG_NAME, sizeof(cfg_name));
		}
	}else{
		if (en == 1){
			strlcpy(cfg_name, GOODIX_TEST_CFG_NAME, sizeof(cfg_name));
		}else{
			strlcpy(cfg_name, GOODIX_DEFAULT_CFG_NAME, sizeof(cfg_name));
		}
	}
	
	r = goodix_change_config(core_data, cfg_name);
	
	return count;
}

static ssize_t goodix_sample_rate_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",GoodixSampleRate);
}

static ssize_t goodix_sample_rate_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int ret, value;

	sscanf(buf, "%d", &value);

	if ((value != 120) && (value != 240)) {
		ts_info("%d\n",value);
		return -EINVAL;
	}

	if((value == 120) && (core_data->game_mode == true)) {
		core_data->game_mode = false;
		ret = goodix_ts_switch_sample_rate(core_data);
	} else if ((value == 240) && (core_data->game_mode == false)) {
		core_data->game_mode = true;
		ret = goodix_ts_switch_sample_rate(core_data);
	} else 
		ts_info("No need to change sample rate");

	return count;
}

static ssize_t goodix_ts_rotation_type_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n",core_data->rotation);
}

static ssize_t goodix_ts_rotation_type_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int ret, value;

	sscanf(buf, "%d", &value);

	if ((value != 0) && (value != 90) && (value != 270)) {
		ts_info("%d\n",value);
		return -EINVAL;
	}
	
	ret = goodix_ts_rotation(core_data, value);

	return count;
}

static ssize_t goodix_ts_test_keycode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", testkeycode);
}

static ssize_t goodix_ts_test_keycode_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int value;

	sscanf(buf, "%d", &value);

	if ((value < 0) || (value > 1024)) {
		ts_info("test keycode%d\n",value);
		return -EINVAL;
	}
	testkeycode = value;
	input_switch_key(core_data->input_dev, value);

	return count;
}

static ssize_t game_settings_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	u16 cmd_addr = GOODIX_ADDR_READ_EXTERNAL_CMD;
	u32 buf_len = 11;
	u8 buffer[11]={0x0};
	int ret = 0, i = 0, offset = 0;

	mutex_lock(&goodix_modules.mutex);
	ret = ts_dev->hw_ops->read(ts_dev, cmd_addr, buffer, buf_len);
	if (ret) {
		ts_info("failed read addr(%x) data %*ph\n", cmd_addr,
			buf_len, buffer);
	}
	mutex_unlock(&goodix_modules.mutex);
	ts_info("%s read addr (%x) with data %*ph\n",
		"success", cmd_addr, buf_len, buffer);
	
	for (i = 0; i < buf_len; i++) {
		offset += snprintf(&buf[offset], PAGE_SIZE - offset,
						"%02X,", buffer[i]);
	}
	buf[offset++] = '\n';

	return offset;
}

static ssize_t game_settings_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{

	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	//struct goodix_ext_module *ext_module;
	char game_settings[29];
	u16 touch_level_index = 0x0,leave_level_index = 0x0, first_filter_index = 0x0, normal_filter_index = 0x0, checksum = 0x0;
	u8 Rcoef = 0x0, RcoefRight=0x0, touch_timer = 0x0;
	u8 game_cfg[13]={0x0};
	u8 apply_cfg[5]={0x42, 0x0, 0x0, 0x0, 0x42};
	u32 cmd_len = 0;
	u16 cmd_addr = 0;
	int ret = 0, i = 0;
	u32 buf_len = 11;
	u8 buffer[11]={0x0};
	
	u16 touch_level[5] = {150, 140, 130, 92, 75};
	u16 leave_level[5] = {110, 100, 100, 70, 70};
	u16 first_filter[5] = {25, 36, 48, 69, 90};
	u16 normal_filter[5] = {35, 29, 24, 19, 15};
	
	memset(game_settings, 0, sizeof(game_settings));
	sprintf(game_settings, "%s", buf);
	game_settings[count-1] = '\0';
	ts_info("game_settings %s count %d ",game_settings,count);
	
	if(count != 28){
		return -EINVAL;
	}

	touch_level_index = (u16)shex_to_u16(game_settings +0, 3);
	leave_level_index = (u16)shex_to_u16(game_settings +4, 3);
	first_filter_index = (u16)shex_to_u16(game_settings +8, 3);
	normal_filter_index = (u16)shex_to_u16(game_settings +12, 3);
	touch_timer = (u8)shex_to_u16(game_settings +24, 3);
	
	ts_info("touch_level_index 0x%04X, leave_level_index 0x%04X first_filter_index 0x%04X, normal_filter_index 0x%04X",
	touch_level_index,leave_level_index,first_filter_index,normal_filter_index);
	
	if (touch_timer == 0) {
		ts_info("touch_timer = 0");
		return -EINVAL;
	}

	cmd_addr = GOODIX_ADDR_READ_EXTERNAL_CMD;
	mutex_lock(&goodix_modules.mutex);
	ret = ts_dev->hw_ops->read(ts_dev, cmd_addr, buffer, buf_len);
	if (ret) {
		ts_info("failed read addr(%x) data %*ph\n", cmd_addr,
			buf_len, buffer);
	}
	ts_info("%s read addr (%x) with data %*ph\n",
		"success", cmd_addr, buf_len, buffer);
	for (i = 0; i < buf_len; i++) {
		if(i == 8)
			Rcoef = buffer[i];
		else if(i == 9)
			RcoefRight = buffer[i];
		else
			continue;
	}
	game_cfg[0] = (touch_level[touch_level_index]& 0xFF00) >> 8;
	game_cfg[1] = touch_level[touch_level_index]& 0x00FF;
	game_cfg[2] = (leave_level[leave_level_index]& 0xFF00) >> 8;
	game_cfg[3] = leave_level[leave_level_index]& 0x00FF;
	game_cfg[4] = (first_filter[first_filter_index]& 0xFF00) >> 8;
	game_cfg[5] = first_filter[first_filter_index]& 0x00FF;
	game_cfg[6] = (normal_filter[normal_filter_index]& 0xFF00) >> 8;
	game_cfg[7] = normal_filter[normal_filter_index]& 0x00FF;
	game_cfg[8] = Rcoef;
	game_cfg[9] = RcoefRight;
	game_cfg[10] = touch_timer;
	checksum = (u16)(game_cfg[0]+game_cfg[1]+game_cfg[2]+game_cfg[3]+game_cfg[4]+
	game_cfg[5]+game_cfg[6]+game_cfg[7]+game_cfg[8]+game_cfg[9]+game_cfg[10]);
	game_cfg[11] = (checksum & 0xFF00) >> 8;
	game_cfg[12] = checksum & 0x00FF;
	ts_info("touch_level 0x%04X, leave_level 0x%04X first_filter 0x%04X, normal_filter 0x%04X",
	touch_level[touch_level_index],leave_level[leave_level_index],first_filter[first_filter_index],normal_filter[normal_filter_index]);
	ts_info("Rcoef 0x%02X,RcoefRight 0x%02X touch_timer 0x%02X checksum 0x%04X",Rcoef,RcoefRight,touch_timer,checksum);
	// 1
	cmd_addr = GOODIX_ADDR_EXTERNAL_CMD;
	cmd_len = 13;
	ret = ts_dev->hw_ops->write(ts_dev, cmd_addr, game_cfg, cmd_len);
	if (ret) {
		ts_info("failed write addr(%x) data %*ph\n", cmd_addr,
			cmd_len, game_cfg);
	}
	
	ts_info("%s write to addr (%x) with data %*ph\n",
		"success", cmd_addr, cmd_len, game_cfg);
	// 2
	ret = ts_dev->hw_ops->write_trans(ts_dev, GOODIX_ADDR_SPECIAL_CMD,
				     apply_cfg, 5);
	if (ret < 0)
		ts_info("failed GOODIX_ADDR_SPECIAL_CMD");
	mutex_unlock(&goodix_modules.mutex);

	return count;
}

static ssize_t edge_settings_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	u16 cmd_addr = GOODIX_ADDR_READ_EXTERNAL_CMD;
	u32 buf_len = 11;
	u8 buffer[11]={0x0};
	int ret = 0, i = 0, offset = 0;

	mutex_lock(&goodix_modules.mutex);
	ret = ts_dev->hw_ops->read(ts_dev, cmd_addr, buffer, buf_len);
	if (ret) {
		ts_info("failed read addr(%x) data %*ph\n", cmd_addr,
			buf_len, buffer);
	}
	mutex_unlock(&goodix_modules.mutex);
	ts_info("%s read addr (%x) with data %*ph\n",
		"success", cmd_addr, buf_len, buffer);

	for (i = 8; i < 10; i++) {
		offset += snprintf(&buf[offset], PAGE_SIZE - offset,
						"%02X,", buffer[i]);
	}
	buf[offset++] = '\n';

	return offset;
}

static ssize_t edge_settings_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{

	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	//struct goodix_ext_module *ext_module;
	char edge_settings[9];
	u16 checksum = 0x0;
	u8 game_cfg[13]={0x0};
	u8 apply_cfg[5]={0x42, 0x0, 0x0, 0x0, 0x42};
	u32 cmd_len = 0;
	u16 cmd_addr = 0;
	int ret = 0;
	u32 buf_len = 11;
	u8 buffer[11]={0x0};
	int i = 0;

	memset(edge_settings, 0, sizeof(edge_settings));
	sprintf(edge_settings, "%s", buf);
	edge_settings[count-1] = '\0';
	ts_info("edge_settings %s count %d ",edge_settings,count);

	if(count != 8){
		return -EINVAL;
	}

	cmd_addr = GOODIX_ADDR_READ_EXTERNAL_CMD;
	mutex_lock(&goodix_modules.mutex);
	ret = ts_dev->hw_ops->read(ts_dev, cmd_addr, buffer, buf_len);
	if (ret) {
		ts_info("failed read addr(%x) data %*ph\n", cmd_addr,
			buf_len, buffer);
	}
	ts_info("%s read addr (%x) with data %*ph\n",
		"success", cmd_addr, buf_len, buffer);
	for (i = 0; i < buf_len; i++) {
        if(i == 8)
            game_cfg[i] = (u8)shex_to_u16(edge_settings +0, 3);
        else if(i == 9)
            game_cfg[i] = (u8)shex_to_u16(edge_settings +4, 3);
        else
            game_cfg[i] = buffer[i];
        checksum = (u16)(game_cfg[0]+game_cfg[1]+game_cfg[2]+game_cfg[3]+game_cfg[4]+
        game_cfg[5]+game_cfg[6]+game_cfg[7]+game_cfg[8]+game_cfg[9]+game_cfg[10]);
        game_cfg[11] = (checksum & 0xFF00) >> 8;
        game_cfg[12] = checksum & 0x00FF;
	}

	cmd_addr = GOODIX_ADDR_EXTERNAL_CMD;
	cmd_len = 13;

	ret = ts_dev->hw_ops->write(ts_dev, cmd_addr, game_cfg, cmd_len);
	if (ret) {
		ts_info("failed write addr(%x) data %*ph\n", cmd_addr,
			cmd_len, game_cfg);
	}

	ts_info("%s write to addr (%x) with data %*ph\n",
		"success", cmd_addr, cmd_len, game_cfg);

	ret = ts_dev->hw_ops->write_trans(ts_dev, GOODIX_ADDR_SPECIAL_CMD,
				     apply_cfg, 5);
	if (ret < 0)
		ts_info("failed GOODIX_ADDR_SPECIAL_CMD");
	mutex_unlock(&goodix_modules.mutex);

	return count;
}

static ssize_t game_settings_test_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{

	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	//struct goodix_ext_module *ext_module;
	char game_settings[29];
	u16 touch_level = 0x0,leave_level = 0x0, first_filter = 0x0, normal_filter = 0x0, checksum = 0x0;
	u8 Rcoef = 0x0, RcoefRight=0x0, touch_timer = 0x0;
	u8 game_cfg[13]={0x0};
	u8 apply_cfg[5]={0x42, 0x0, 0x0, 0x0, 0x42};
	u32 cmd_len = 0;
	u16 cmd_addr = 0;
	int ret = 0;
	
	memset(game_settings, 0, sizeof(game_settings));
	sprintf(game_settings, "%s", buf);
	game_settings[count-1] = '\0';
	ts_info("game_settings %s count %d ",game_settings,count);
	
	if(count != 28){
		return -EINVAL;
	}

	touch_level = (u16)shex_to_u16(game_settings +0, 3);
	leave_level = (u16)shex_to_u16(game_settings +4, 3);
	first_filter = (u16)shex_to_u16(game_settings +8, 3);
	normal_filter = (u16)shex_to_u16(game_settings +12, 3);
	Rcoef = (u8)shex_to_u16(game_settings +16, 3);
	RcoefRight = (u8)shex_to_u16(game_settings +20, 3);
	touch_timer = (u8)shex_to_u16(game_settings +24, 3);
	
	if (touch_timer == 0) {
		ts_info("touch_timer = 0");
		return -EINVAL;
	}
	
	if((core_data->rotation != 90) && (core_data->rotation != 270)) {
		ts_info("rotation = %d", core_data->rotation);
		return -EINVAL;
	}

	goodix_ts_rotation(core_data, core_data->rotation);

	game_cfg[0] = (touch_level& 0xFF00) >> 8;
	game_cfg[1] = touch_level& 0x00FF;
	game_cfg[2] = (leave_level& 0xFF00) >> 8;
	game_cfg[3] = leave_level& 0x00FF;
	game_cfg[4] = (first_filter& 0xFF00) >> 8;
	game_cfg[5] = first_filter& 0x00FF;
	game_cfg[6] = (normal_filter& 0xFF00) >> 8;
	game_cfg[7] = normal_filter& 0x00FF;
	game_cfg[8] = Rcoef;
	game_cfg[9] = RcoefRight;
	game_cfg[10] = touch_timer;
	checksum = (u16)(game_cfg[0]+game_cfg[1]+game_cfg[2]+game_cfg[3]+game_cfg[4]+
	game_cfg[5]+game_cfg[6]+game_cfg[7]+game_cfg[8]+game_cfg[9]+game_cfg[10]);
	game_cfg[11] = (checksum & 0xFF00) >> 8;
	game_cfg[12] = checksum & 0x00FF;

	ts_info("touch_level 0x%04X, leave_level 0x%04X first_filter 0x%04X, normal_filter 0x%04X",
	touch_level,leave_level,first_filter,normal_filter);
	ts_info("Rcoef 0x%02X,RcoefRight 0x%02X touch_timer 0x%02X checksum 0x%04X",Rcoef,RcoefRight,touch_timer,checksum);
	// 1
	cmd_addr = GOODIX_ADDR_EXTERNAL_CMD;
	cmd_len = 13;
	
	mutex_lock(&goodix_modules.mutex);
	ret = ts_dev->hw_ops->write(ts_dev, cmd_addr, game_cfg, cmd_len);
	if (ret) {
		ts_info("failed write addr(%x) data %*ph\n", cmd_addr,
			cmd_len, game_cfg);
	}
	
	ts_info("%s write to addr (%x) with data %*ph\n",
		"success", cmd_addr, cmd_len, game_cfg);
	// 2
	ret = ts_dev->hw_ops->write_trans(ts_dev, GOODIX_ADDR_SPECIAL_CMD,
				     apply_cfg, 5);
	if (ret < 0)
		ts_info("failed GOODIX_ADDR_SPECIAL_CMD");
	mutex_unlock(&goodix_modules.mutex);

	return count;
}

static ssize_t airtrigger_touch_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	bool stat = 0;
    struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	if (core_data->atr_enable)
		stat = true;
	else
		stat = false;

	return sprintf(buf, "%d", stat);
}

static ssize_t keymapping_touch_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int id, action, x, y, random, minus;
	//ts_info("keymapping cmd buf: %s len=%d\n", buf, count);

	if ((count != 15) && (count != 14)){
		ts_info("Invalid cmd buffer %d", count);
		return -EINVAL;
	}

	if (count == 14) {
		id = buf[0] - '0';
		action = buf[1] - '0';
		random = buf[2] - '0';
		
		minus = buf[3];
		x =  shex_to_u16(buf + 4, 4);
		if(minus == '-')
			x = -x;

		minus = buf[8];
		y =  shex_to_u16(buf + 9, 4);
		if(minus == '-')
			y = -y;
	} else if (count == 15) {
		id = shex_to_u16(buf, 2);
		action = buf[2] - '0';
		random = buf[3] - '0';

		minus = buf[4];
		x =  shex_to_u16(buf + 5, 4);
		if(minus == '-')
			x = -x;

		minus = buf[9];
		y =  shex_to_u16(buf + 10, 4);
		if(minus == '-')
			y = -y;
	}
	
	ts_info("keymapping ID=%d ACTION=%d X=%d Y=%d RANDOM=%d", id, action, x, y, random);
	ATR_touch_new(dev, id, action, x, y, random);

	return count;
}

static ssize_t airtrigger_touch_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int action, x, y, i;
	//ts_info("keymapping airtrigger cmd buf: %s len=%d\n", buf, count);

	if(count != 19)
		return -EINVAL;
	if(buf[0] == 'F')
	{
		for(i = 0; i < TOTAL_SLOT; i++)
		{
			if(touch_figer_slot[i])
			{
				input_mt_slot(core_data->input_dev, i + 10);
				input_mt_report_slot_state(core_data->input_dev, MT_TOOL_FINGER, false);
				input_sync(core_data->input_dev);
				touch_figer_slot[i] = 0;
				LastATR = LastATL = 0;
			}
		}
		core_data->atr_enable = false;
		input_report_key(core_data->input_dev, BTN_TOUCH, 0);
		input_sync(core_data->input_dev);
		ts_info("keymapping all buttons up");
		return count;
	}

	action = buf[1] - '0'; // handle R
	if(LastATR != action)
	{
		x =  shex_to_u16(buf + 10, 4);
		y =  shex_to_u16(buf + 14, 4);
		ATR_touch_new(dev, 11, action,  x,  y, 0);
		ts_info("keymapping airtrigger R %d %d, %d", action, x, y, 0);
		LastATR = action;
	}
	action = buf[0] - '0'; // handle L
	if(LastATL != action)
	{
		x =  shex_to_u16(buf + 2, 4);
		y =  shex_to_u16(buf + 6, 4);
		ATR_touch_new(dev, 12, action,  x,  y, 0);
		ts_info("keymapping airtrigger L %d %d, %d", action, x, y, 0);
		LastATL = action;
	}

	return count;
}

static ssize_t glove_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);

	if(core_data == NULL)
		return snprintf(buf, PAGE_SIZE, "error\n");

	return snprintf(buf, PAGE_SIZE, "%d\n",core_data->glove_mode);
}

static ssize_t glove_mode_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	unsigned int en;
	char cfg_name[32] = {0x00};

	if (sscanf(buf, "%u", &en) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	
	if(en == 0 && atomic_read(&core_data->glove_mode)!=1){
		ts_info("previous status not update glove cfg");
		return count;
	}
	ts_info("glove mode =%d", en);

	if (en == 1) {
		atomic_set(&core_data->glove_mode, 1);
		strlcpy(cfg_name, GOODIX_GLOVE_CFG_NAME, sizeof(cfg_name));
	} else {
		atomic_set(&core_data->glove_mode, 0);
		strlcpy(cfg_name, GOODIX_DEFAULT_CFG_NAME, sizeof(cfg_name));
	}
	
	goodix_change_config(core_data, cfg_name);

	return count;
}

static ssize_t dfps_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", core_data->dfps);
}

static ssize_t dfps_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int value;

	sscanf(buf, "%d", &value);
	if ((value != 60) && (value != 90) && (value != 120) && (value != 144)) {
		ts_info("dfps : %d\n",value);
		return -EINVAL;
	}
	core_data->dfps = value;
	goodix_ts_switch_sample_rate(gts_core_data);

	return count;
}

static ssize_t sampleratelocked_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", SampleRateLocked);
}

static ssize_t sampleratelocked_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);
	if ((value != 0) && (value != 1)) {
		ts_info("sampleratelocked : %d\n",value);
		return -EINVAL;
	}
	SampleRateLocked = value;
	goodix_ts_switch_sample_rate(gts_core_data);

	return count;
}

static ssize_t dongle_state_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int en;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
	ts_info("phone insert to %d (2:station / 3:dock)",en);

	if((en == 2) || (en == 3))
		core_data->station_insert = true;
	else
		core_data->station_insert = false;

	return count;
}

static ssize_t charge_mode_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&core_data->charge_mode));
}

static ssize_t charge_mode_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int value;

	sscanf(buf, "%d", &value);
	if ((value != 0) && (value != 1)) {
		ts_info("charge_mode : %d\n",value);
		return -EINVAL;
	}
	if(value == 0)
		charge_mode_enable(core_data, false);
	else
		charge_mode_enable(core_data, true);

	return count;
}

static ssize_t FP_area_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", FPArea);
}

static ssize_t FP_area_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	sscanf(buf, "%d", &value);
	if ((value < 0) && (value > 20)) {
		ts_info("Parameter illegal : %d (0~20)\n",value);
		return -EINVAL;
	}
	if(FPArea != value)
		FPArea = value;

	return count;
}

static ssize_t phone_state_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	int en;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
        ts_info("phone state %d",en);
	
	if(en == 1)
	  core_data->phone_call_on = true;
	else
	  core_data->phone_call_on = false;

	return count;
}

static ssize_t chip_debug_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	u8 buffer[BUF_LEN_4154]={0x0};
	int i = 0, offset = 0;

	read_chip_cmd(core_data, GOODIX_ADDR_GOODIX_DEBUG_CMD, BUF_LEN_4154, buffer);

	for (i = 0; i < BUF_LEN_4154; i++) {
		offset += snprintf(&buf[offset], PAGE_SIZE - offset,
						"%02X,", buffer[i]);
	}
	buf[offset++] = '\n';

	return offset;
}

static ssize_t enable_touch_debug_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int en;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
	ts_info("touch debug %d",en);

	if(en == 1)
		enable_touch_debug = true;
	else
		enable_touch_debug = false;

	return count;
}

static ssize_t enable_touch_debug_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	ts_info("enable_touch_debug %d", enable_touch_debug);
	return scnprintf(buf, PAGE_SIZE, "%d\n", enable_touch_debug);
}

static ssize_t enable_touch_time_debug_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int en;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
	ts_info("touch time debug %d",en);

	if(en == 1)
		enable_touch_time_debug = true;
	else
		enable_touch_time_debug = false;

	return count;
}

static ssize_t enable_touch_time_debug_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", enable_touch_time_debug);
}

static ssize_t FP_status_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int en;
	
	if (!((sscanf(buf, "%d", &en) == 0) || (sscanf(buf, "%d", &en) == 1) || (sscanf(buf, "%d", &en) == 2) || (sscanf(buf, "%d", &en) == 3) || (sscanf(buf, "%d", &en) == 4)))
		return -EINVAL;
	ts_info("FP status %d",en);

	fp_status = en;
	ts_info("touch resume : vendor.goodix.sensor.status = %d", fp_status);
	if(((fp_status == 1) || (fp_status == 3)) && (process_resume == false)) {
		process_resume = true;
		wake_up_interruptible(&gts_core_data->fp_queue);
	}
	if((fp_status == 4) && (process_resume == true)) {
		process_resume = false;
	}
	return count;
}


static ssize_t FP_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", fp_status);
}

static ssize_t keymapping_touch_debug_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int en;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
	ts_info("keymapping touch debug %d",en);

	if(en == 1)
		GoodixTSKeyMappingDebug = true;
	else
		GoodixTSKeyMappingDebug = false;

	return count;
}

static ssize_t keymapping_touch_debug_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", GoodixTSKeyMappingDebug);
}

static ssize_t touch_INT_debug_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", GoodixTSINTDebug);
}

static ssize_t touch_INT_debug_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	int en;
	
	if (sscanf(buf, "%d", &en) != 1)
		return -EINVAL;
	ts_info("touch INT debug %d",en);

	if(en == 1)
		GoodixTSINTDebug = true;
	else
		GoodixTSINTDebug = false;

	return count;
}


// ASUS_BSP --- Touch

static DEVICE_ATTR(extmod_info, S_IRUGO, goodix_ts_extmod_show, NULL);
static DEVICE_ATTR(driver_info, S_IRUGO, goodix_ts_driver_info_show, NULL);
static DEVICE_ATTR(chip_info, S_IRUGO, goodix_ts_chip_info_show, NULL);
static DEVICE_ATTR(reset, S_IWUSR | S_IWGRP, NULL, goodix_ts_reset_store);
static DEVICE_ATTR(send_cfg, S_IWUSR | S_IWGRP, NULL, goodix_ts_send_cfg_store);
static DEVICE_ATTR(read_cfg, S_IRUGO, goodix_ts_read_cfg_show, NULL);
static DEVICE_ATTR(irq_info, S_IRUGO | S_IWUSR | S_IWGRP,
		   goodix_ts_irq_info_show, goodix_ts_irq_info_store);
static DEVICE_ATTR(reg_rw, S_IRUGO | S_IWUSR | S_IWGRP,
		   goodix_ts_reg_rw_show, goodix_ts_reg_rw_store);
// ASUS_BSP +++ Touch
static DEVICE_ATTR(power_on_off, S_IWUSR | S_IWGRP, NULL, goodix_ts_power_on_off);
static DEVICE_ATTR(power_state, S_IRUGO, goodix_ts_power_state, NULL);
static DEVICE_ATTR(print_touch_count, S_IRUGO | S_IWUSR | S_IWGRP, 
			goodix_ts_print_count_show, goodix_ts_print_count_store);
static DEVICE_ATTR(enable_aod_test,S_IRUGO|S_IWUSR, goodix_aod_test_mode_show, goodix_aod_test_mode_store);
static DEVICE_ATTR(enable_timestamp,S_IRUGO|S_IWUSR, goodix_timestamp_show, goodix_timestamp_store);
static DEVICE_ATTR(enable_timestamp_debug,S_IRUGO|S_IWUSR, goodix_timestamp_debug_show, goodix_timestamp_debug_store);
static DEVICE_ATTR(enable_evdev_timestamp_debug,S_IRUGO|S_IWUSR, goodix_evdev_timestamp_debug_show, goodix_evdev_timestamp_debug_store);
static DEVICE_ATTR(test_cfg, S_IRUGO|S_IWUSR, NULL, test_cfg_store);
static DEVICE_ATTR(sample_rate, S_IRUGO|S_IWUSR, goodix_sample_rate_show, goodix_sample_rate_store);
static DEVICE_ATTR(rotation_type, S_IRUGO | S_IWUSR | S_IWGRP, goodix_ts_rotation_type_show, goodix_ts_rotation_type_store);
static DEVICE_ATTR(test_keycode, S_IRUGO | S_IWUSR | S_IWGRP, goodix_ts_test_keycode_show, goodix_ts_test_keycode_store);
static DEVICE_ATTR(game_settings, S_IRUGO | S_IWUSR | S_IWGRP, game_settings_show, game_settings_store);
static DEVICE_ATTR(edge_settings, S_IRUGO | S_IWUSR | S_IWGRP, edge_settings_show, edge_settings_store);
static DEVICE_ATTR(game_settings_test, S_IRUGO | S_IWUSR | S_IWGRP, game_settings_show, game_settings_test_store);
static DEVICE_ATTR(airtrigger_touch, S_IRUGO|S_IWUSR, airtrigger_touch_show, airtrigger_touch_store);
static DEVICE_ATTR(keymapping_touch, S_IRUGO|S_IWUSR, airtrigger_touch_show, keymapping_touch_store);
static DEVICE_ATTR(glove_mode, S_IRUGO|S_IWUSR, glove_mode_show, glove_mode_store);
static DEVICE_ATTR(dfps, S_IRUGO|S_IWUSR, dfps_show, dfps_store);
static DEVICE_ATTR(sampleratelocked, S_IRUGO|S_IWUSR, sampleratelocked_show, sampleratelocked_store);
static DEVICE_ATTR(dongle_state,S_IRUGO|S_IWUSR, NULL, dongle_state_store);
static DEVICE_ATTR(charge_mode,S_IRUGO|S_IWUSR, charge_mode_show, charge_mode_store);
static DEVICE_ATTR(FP_area,S_IRUGO|S_IWUSR, FP_area_show, FP_area_store);
static DEVICE_ATTR(phone_state_on,S_IRUGO|S_IWUSR, NULL, phone_state_store);
static DEVICE_ATTR(chip_debug,S_IRUGO|S_IWUSR, chip_debug_show, NULL);
static DEVICE_ATTR(enable_touch_debug,S_IRUGO|S_IWUSR, enable_touch_debug_show, enable_touch_debug_store);
static DEVICE_ATTR(enable_touch_time_debug,S_IRUGO|S_IWUSR, enable_touch_time_debug_show, enable_touch_time_debug_store);
static DEVICE_ATTR(FP_status,S_IRUGO|S_IWUSR, FP_status_show, FP_status_store);
static DEVICE_ATTR(keymapping_touch_debug,S_IRUGO|S_IWUSR, keymapping_touch_debug_show, keymapping_touch_debug_store);
static DEVICE_ATTR(touch_INT_debug,S_IRUGO|S_IWUSR, touch_INT_debug_show, touch_INT_debug_store);
// ASUS_BSP --- Touch

static struct attribute *sysfs_attrs[] = {
	&dev_attr_extmod_info.attr,
	&dev_attr_driver_info.attr,
	&dev_attr_chip_info.attr,
	&dev_attr_reset.attr,
	&dev_attr_send_cfg.attr,
	&dev_attr_read_cfg.attr,
	&dev_attr_irq_info.attr,
	&dev_attr_reg_rw.attr,
// ASUS_BSP +++ Touch
	&dev_attr_power_on_off.attr,
	&dev_attr_power_state.attr,
	&dev_attr_print_touch_count.attr,
	&dev_attr_enable_aod_test.attr,
	&dev_attr_enable_timestamp.attr,
	&dev_attr_enable_timestamp_debug.attr,
	&dev_attr_enable_evdev_timestamp_debug.attr,
	&dev_attr_test_cfg.attr,
	&dev_attr_sample_rate.attr,
	&dev_attr_rotation_type.attr,
	&dev_attr_test_keycode.attr,
	&dev_attr_game_settings.attr,
	&dev_attr_edge_settings.attr,
	&dev_attr_game_settings_test.attr,
	&dev_attr_keymapping_touch.attr,
	&dev_attr_airtrigger_touch.attr,
	&dev_attr_glove_mode.attr,
	&dev_attr_dfps.attr,
	&dev_attr_sampleratelocked.attr,
	&dev_attr_dongle_state.attr,
	&dev_attr_charge_mode.attr,
	&dev_attr_FP_area.attr,
	&dev_attr_phone_state_on.attr,
	&dev_attr_chip_debug.attr,
	&dev_attr_enable_touch_debug.attr,
	&dev_attr_enable_touch_time_debug.attr,
	&dev_attr_FP_status.attr,
	&dev_attr_keymapping_touch_debug.attr,
	&dev_attr_touch_INT_debug.attr,
// ASUS_BSP --- Touch
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attrs,
};

static ssize_t goodix_sysfs_config_write(struct file *file,
		struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t pos, size_t count)
{
	struct platform_device *pdev = container_of(kobj_to_dev(kobj),
				struct platform_device, dev);
	struct goodix_ts_core *ts_core = platform_get_drvdata(pdev);
	struct goodix_ts_device *ts_dev = ts_core->ts_dev;
	struct goodix_ts_config *config = NULL;
	int ret;

	if (pos != 0 || count > GOODIX_CFG_MAX_SIZE) {
		ts_info("pos(%d) != 0, cfg size %zu", (int)pos, count);
		return -EINVAL;
	}

	config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	if (config == NULL)
		return -ENOMEM;

	memcpy(config->data, buf, count);
	config->length = count;
	config->reg_base = ts_dev->reg.cfg_addr;
	mutex_init(&config->lock);
	config->initialized = true;

	ret = ts_dev->hw_ops->send_config(ts_dev, config);
	if (ret) {
		count = -EINVAL;
		ts_err("send config failed %d", ret);
	} else {
		ts_info("send config success");
	}

	kfree(config);
	return count;
}

static ssize_t goodix_sysfs_config_read(struct file *file,
		struct kobject *kobj, struct bin_attribute *attr,
		char *buf, loff_t pos, size_t size)
{
	struct platform_device *pdev = container_of(kobj_to_dev(kobj),
				struct platform_device, dev);
	struct goodix_ts_core *ts_core = platform_get_drvdata(pdev);
	struct goodix_ts_device *ts_dev = ts_core->ts_dev;
	int ret;

	ts_debug("pos = %d, size = %zu", (int)pos, size);

	if (pos != 0)
		return 0;

	if (ts_dev->hw_ops->read_config)
		ret = ts_dev->hw_ops->read_config(ts_dev, buf);
	else
		ret = -EINVAL;

	ts_debug("read config ret %d", ret);
	return ret;
}

static struct bin_attribute goodix_config_bin_attr = {
	.attr = {
		.name = "config_bin",
		.mode = S_IRUGO | S_IWUSR | S_IWGRP,
	},
	.size = GOODIX_CFG_MAX_SIZE,
	.read = goodix_sysfs_config_read,
	.write = goodix_sysfs_config_write,
};

static int goodix_ts_sysfs_init(struct goodix_ts_core *core_data)
{
	int ret;

	ret = sysfs_create_bin_file(&core_data->pdev->dev.kobj,
				    &goodix_config_bin_attr);
	if (ret) {
		ts_err("failed create config bin attr");
		return ret;
	}

	return ret;
}

static void goodix_ts_sysfs_exit(struct goodix_ts_core *core_data)
{
	sysfs_remove_bin_file(&core_data->pdev->dev.kobj,
			      &goodix_config_bin_attr);
	sysfs_remove_group(&core_data->pdev->dev.kobj, &sysfs_group);
}

/* event notifier */
static BLOCKING_NOTIFIER_HEAD(ts_notifier_list);
/**
 * goodix_ts_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *  see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&ts_notifier_list, nb);
}
EXPORT_SYMBOL(goodix_ts_register_notifier);

/**
 * goodix_ts_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *	see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&ts_notifier_list, nb);
}
EXPORT_SYMBOL(goodix_ts_unregister_notifier);

/**
 * fb_notifier_call_chain - notify clients of fb_events
 *	see enum ts_notify_event in goodix_ts_core.h
 */
int goodix_ts_blocking_notify(enum ts_notify_event evt, void *v)
{
	int ret;

	ret = blocking_notifier_call_chain(&ts_notifier_list,
			(unsigned long)evt, v);
	return ret;
}
EXPORT_SYMBOL_GPL(goodix_ts_blocking_notify);

static void goodix_ts_report_pen(struct input_dev *dev,
		struct goodix_pen_data *pen_data)
{
	int i;

	if (pen_data->coords.status == TS_TOUCH) {
		input_report_key(dev, BTN_TOUCH, 1);
		input_report_key(dev, pen_data->coords.tool_type, 1);
	} else if (pen_data->coords.status == TS_RELEASE) {
		input_report_key(dev, BTN_TOUCH, 0);
		input_report_key(dev, pen_data->coords.tool_type, 0);
	}
	if (pen_data->coords.status) {
		input_report_abs(dev, ABS_X, pen_data->coords.x);
		input_report_abs(dev, ABS_Y, pen_data->coords.y);
		input_report_abs(dev, ABS_PRESSURE, pen_data->coords.p);
	}
	/* report pen button */
	for (i = 0; i < GOODIX_MAX_PEN_KEY; i++) {
		if (!pen_data->keys[i].status)
			continue;
		if (pen_data->keys[i].status == TS_TOUCH)
			input_report_key(dev, pen_data->keys[i].code, 1);
		else if (pen_data->keys[i].status == TS_RELEASE)
			input_report_key(dev, pen_data->keys[i].code, 0);
	}
	input_sync(dev);
}

// ASUS_BSP +++ Touch
static void input_switch_key(struct input_dev *dev, unsigned int code)
{
	input_report_key(dev, code, 1);
	input_sync(dev);
	input_report_key(dev, code, 0);
	input_sync(dev);

	ts_info("keycode = %d\n", code);
}

static int goodix_ts_switch_sample_rate(struct goodix_ts_core *core_data)
{
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	u8 cmd_buf[32];
	int ret;
	u32 cmd_len = 0;
	u16 cmd_addr = 0;

	ts_info("[Start] samplerate : %d", GoodixSampleRate);
	ts_info("[1] gaming mode : %d", core_data->game_mode);
	ts_info("[2] dfps value : %d", core_data->dfps);
	
	if(SampleRateLocked == 1) {
		GoodixSampleRate = 240;
	} else {
		if((gts_core_data->game_mode == false) && (core_data->dfps <= 120))
			GoodixSampleRate = 120;
		else
			GoodixSampleRate = 240;
	}
	
	cmd_addr = GOODIX_ADDR_SPECIAL_CMD;
	cmd_len = 5;
	cmd_buf[0] = GOODIX_CMD_SAMPLE_RATE;
	cmd_buf[1] = 0;
	// CFG(25)
	if(GoodixSampleRate == 120) {
		cmd_buf[2] = 0x2;
	} else if (GoodixSampleRate == 240) {
		cmd_buf[2] = 0x1;
	}
	cmd_buf[3] = 0;
	cmd_buf[4] = cmd_buf[0] + cmd_buf[1] + cmd_buf[2];

	ret = ts_dev->hw_ops->write(ts_dev, cmd_addr, cmd_buf, cmd_len);
	if (ret) {
		ts_info("failed write addr(%x) data %*ph\n", cmd_addr,
			cmd_len, cmd_buf);
	}

	ts_info("[End] samplerate : %d", GoodixSampleRate);
	ts_info("%s write to addr (%x) with data %*ph\n",
		"success", cmd_addr, cmd_len, cmd_buf);
	
	return 0;
}

static int goodix_ts_rotation(struct goodix_ts_core *core_data, int value)
{
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	u8 cmd_buf[32];
	int ret;
	u32 cmd_len = 0;
	u16 cmd_addr = 0;

	ts_info("[Start] rotation : %d", core_data->rotation);
	
	core_data->rotation = value;
	
	cmd_addr = GOODIX_ADDR_SPECIAL_CMD;
	cmd_len = 5;
	cmd_buf[0] = GOODIX_CMD_ROTATION;
	cmd_buf[1] = 0;
	if(value == 0) {
		cmd_buf[2] = 0x0;
	} else if (value == 90) {
		cmd_buf[2] = 0x1;
	} else if (value == 270) {
		cmd_buf[2] = 0x2;
	}
	cmd_buf[3] = 0;
	cmd_buf[4] = cmd_buf[0] + cmd_buf[1] + cmd_buf[2];

	ret = ts_dev->hw_ops->write(ts_dev, cmd_addr, cmd_buf, cmd_len);
	if (ret) {
		ts_info("failed write addr(%x) data %*ph\n", cmd_addr,
			cmd_len, cmd_buf);
	}

	ts_info("[End] rotation : %d", core_data->rotation);
	ts_info("%s write to addr (%x) with data %*ph\n",
		"success", cmd_addr, cmd_len, cmd_buf);
	
	return 0;
}

void charge_mode_enable(struct goodix_ts_core *core_data, bool en)
{
	// 0x4160
	//  enable : 06 00 00 00 06
	// disable : 07 00 00 00 07
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	u8 cmd_buf[32];
	int ret;
	u32 cmd_len = 0;
	u16 cmd_addr = 0;
	

	if (core_data == NULL || core_data->initialized != 1) {
		ts_info("goodix touch not vaild or init fail\n"); 
		return;
	}
/*
	if(core_data->station_insert){
		ts_info("insert into station , not enter charge mode");
		return;
	}
*/
	cmd_addr = GOODIX_ADDR_SPECIAL_CMD;
	cmd_len = 5;
	if(en == true) {
		ts_info("usb plug , charge mode enable");
		atomic_set(&core_data->charge_mode, 1);
		cmd_buf[0] = GOODIX_CMD_CHARGE_MODE_ON;
	} else {
		ts_info("usb unplug , charge mode disable");
		atomic_set(&core_data->charge_mode, 0);
		cmd_buf[0] = GOODIX_CMD_CHARGE_MODE_OFF;
	}
	cmd_buf[1] = 0;
	cmd_buf[2] = 0x0;
	cmd_buf[3] = 0;
	cmd_buf[4] = cmd_buf[0] + cmd_buf[1] + cmd_buf[2];

	ret = core_data->ts_dev->hw_ops->write(ts_dev, cmd_addr, cmd_buf, cmd_len);
	if (ret) {
		ts_info("failed write addr(%x) data %*ph\n", cmd_addr,
			cmd_len, cmd_buf);
	}

	ts_info("[End] charge mode : %d", atomic_read(&core_data->charge_mode));
	ts_info("%s write to addr (%x) with data %*ph\n",
		"success", cmd_addr, cmd_len, cmd_buf);
}

int get_chip_int(struct goodix_ts_core *core_data)
{
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	u8 show_buf[32];
	int ret;
	u32 cmd_len = 1;
	u16 cmd_addr = GOODIX_ADDR_GOODIX_INT_DEBUG_CMD;


	if (core_data == NULL || core_data->initialized != 1) {
		ts_info("goodix touch not vaild or init fail\n"); 
		return -1;
	}

	ret = ts_dev->hw_ops->read(ts_dev, cmd_addr, show_buf, cmd_len);
	if (ret) {
		ts_err("failed read addr(%x) length(%d)\n", cmd_addr, cmd_len);
	}
	ret = show_buf[0];

	ts_info("Interrupt count = %d", ret);


	return ret;
}

void gts_usb_plugin(bool plugin)
{
	if(plugin == true) {
		charge_mode_enable(gts_core_data, true);
	} else {
		charge_mode_enable(gts_core_data, false);
	}
}
EXPORT_SYMBOL(gts_usb_plugin);

void setting_touch_print_count(int count)
{
	if (count >= 1)
		print_touch_count_max = count;
}
EXPORT_SYMBOL(setting_touch_print_count);

int read_chip_cmd(struct goodix_ts_core *core_data, u16 cmd_addr, int buf_len, u8 *buffer) {
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	int ret = 0;

	mutex_lock(&goodix_modules.mutex);
	ret = ts_dev->hw_ops->read(ts_dev, cmd_addr, buffer, buf_len);
	if (ret) {
		ts_info("failed read addr(%x) data %*ph\n", cmd_addr,
			buf_len, buffer);
	}
	mutex_unlock(&goodix_modules.mutex);
	ts_info("%s read addr (%x) with data %*ph\n",
		"success", cmd_addr, buf_len, buffer);

	return 0;
}
// ASUS_BSP --- Touch

static void goodix_resume_work(struct work_struct *work)
{
	int retval;

	ts_info("resume_work +++ AOD(%d) PanelOff(%d) FP(%d)", gts_core_data->aod_test_mode, asus_display_in_normal_off(), fp_status);

	if (gts_core_data->aod_test_mode == 1){
		if((fp_status == -1) || (fp_status == 0) || (fp_status == 2) || (fp_status == 4)) {
			ts_info("resume +++ wait sec for FP status(%d) process_resume(%d)", fp_status, process_resume);
			retval = wait_event_interruptible(gts_core_data->fp_queue, (((fp_status == 1) || (fp_status == 3)) || (process_resume == true)));
			ts_info("resume --- wait sec for FP status(%d) process_resume(%d)", fp_status, process_resume);
		}
	}
	if(get_aod_processing() == true) {
		if(asus_display_global_hbm_mode() == 0) {
			ts_info("[KEY_U] asus_display_global_hbm_mode = 0");
			enable_aod_processing(false);
		} else {
			ts_info("resume wait 0.5 sec - HBM(%d)", asus_display_global_hbm_mode());
			schedule_delayed_work(&gts_core_data->gts_resume_work, msecs_to_jiffies(500)); // 0.5 sec
			return;
		}
	} else {
		ts_info("resume ... ");
	}
	mutex_lock(&gts_core_data->gts_suspend_mutex);
	goodix_ts_resume(gts_core_data);
	goodix_ts_switch_sample_rate(gts_core_data);
	goodix_ts_rotation(gts_core_data, gts_core_data->rotation);

	mutex_unlock(&gts_core_data->gts_suspend_mutex);
	
	process_resume = false;
	ts_info("resume_work ---");
}
void phone_touch_resume(void) {
	/* driver probe failed */
	if (!goodix_modules.core_data ||
	    !goodix_modules.core_data->initialized) {
		ts_info("Skip resume (probe failed)");
		return;
	}

	if(atomic_read(&gts_core_data->suspended) == 0) {
		ts_info("Touch already resumed, ignore resume");
		return;
	}

	ts_info("resume +++ \n");
	if(gts_core_data != NULL) {
		//goodix_ts_resume(gts_core_data);
		//queue_work(gts_core_data->gts_suspend_resume_wq, &gts_core_data->gts_resume_work);
		schedule_delayed_work(&gts_core_data->gts_resume_work, msecs_to_jiffies(0)); // 0 sec
	} else {
		ts_info("gts_core_data = NULL \n");
	}
	ts_info("resume --- \n");
}
EXPORT_SYMBOL_GPL(phone_touch_resume);

void phone_touch_suspend(void) {
	/* driver probe failed */
	if (!goodix_modules.core_data ||
	    !goodix_modules.core_data->initialized) {
		ts_info("Skip suspend (probe failed)");
		return;
	}

	if(atomic_read(&gts_core_data->suspended) == 1) {
		ts_info("Touch already suspended, ignore suspend");
		return;
	}
	
	ts_info("suspend +++ \n");
	ts_info("asus_var_regulator_always_on : %d \n", asus_var_regulator_always_on);
	if(gts_core_data != NULL) {
		//gts_core_data->disable_fod = false;
		goodix_ts_suspend(gts_core_data);
	} else {
		ts_info("gts_core_data = NULL \n");
	}
	ts_info("suspend --- \n");
}
EXPORT_SYMBOL_GPL(phone_touch_suspend);
// ASUS_BSP +++ Touch

void ATR_touch_new(struct device *dev, int id,int action, int x, int y, int random)
{
	static int random_x = -5, random_y = -5, random_pressure = -20, random_major = -5;
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct input_dev *input_dev = core_data->input_dev;
	int first_empty_slot = -1;
	int i;
	static bool first_continue_flag = false;

	if (GoodixTSKeyMappingDebug == true)
		ts_info("keymapping ATR_touch_new  id=%d, action=%d, x=%d, y=%d", id, action,  x,  y);
	mutex_lock(&input_dev->mutex);
	if(action) //press, find first slot or find last slot;
	{
		for(i = TOTAL_SLOT -1; i >= 0 ; i--)
		{
			if(first_empty_slot == -1 && touch_figer_slot[i] == 0) //find first empty slot
				first_empty_slot = i;
			if(touch_figer_slot[i] == (id + 1)) //if the last id has been pressed, keep reporting same slot
				first_empty_slot = i;
		}
		if (GoodixTSKeyMappingDebug == true)
			ts_info("keymapping ATR_touch_new press found slot %d", first_empty_slot);
		if(first_empty_slot != -1) // found an available slot
		{
			if(touch_figer_slot[first_empty_slot] ==0) {
				if (GoodixTSKeyMappingDebug == true)
					ts_info("keymapping report %d down x=%d ,y=%d ",first_empty_slot,x,y);
			}
			if(!random) {
				x += random_x;
				y += random_y;
			}
			if(!finger_press) {
				input_mt_slot(input_dev, first_empty_slot + 10);
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
				input_report_abs(input_dev, ABS_MT_PRESSURE, 0x3f + random_pressure);
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0x09 + random_major);
				input_report_abs(input_dev, ABS_MT_POSITION_X, x );
				input_report_abs(input_dev, ABS_MT_POSITION_Y, y );
				ts_info("[ATR_N_R] ID %d active %d x %d y %d p %d m %d", first_empty_slot + 10, action, x, y, 0x3f + random_pressure, 0x09 + random_major);
			} else {
				atr_buf_write(atr_buf_queue, first_empty_slot + 10, action, x, y, 0x3f + random_pressure, 0x09 + random_major);
			}

			if (GoodixTSKeyMappingDebug == true)
				ts_info("slot %d", first_empty_slot+10);
			if(!finger_press){
				if(first_continue_flag == false) {
					ts_info("atr touch down(%d)", random);
					input_report_key(input_dev, BTN_TOUCH, 1);
					if (random == 1)
						first_continue_flag = true;
				}
			}
			if(!finger_press)
				input_sync(input_dev);

			touch_figer_slot[first_empty_slot] = id + 1; // save finger id in slot 
			core_data->atr_enable = true;
		}
	} 
	else //release
	{
		for(i = TOTAL_SLOT -1; i >= 0 ; i--)
		{
			if(touch_figer_slot[i] == (id + 1)) //find the released slot
			{
				first_empty_slot = i;
				break;
			}
		}
		if (GoodixTSKeyMappingDebug == true)
			ts_info("keymapping  release slot %d", first_empty_slot);
		if(first_empty_slot >= 0)
		{
			if(!finger_press) {
				input_mt_slot(input_dev, first_empty_slot + 10);
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
				ts_info("[ATR_N_R] ID %d active %d x %d y %d p %d m %d", first_empty_slot + 10, action, 0, 0, 0, 0);
				input_sync(input_dev);
			} else {
				atr_buf_write(atr_buf_queue, first_empty_slot + 10, action, 0, 0, 0, 0);
			}
			touch_figer_slot[first_empty_slot] = 0;
		}
	}

	for(i = TOTAL_SLOT -1; i >= 0 ; i--)
	{
		if(touch_figer_slot[i] != 0) //find the released slot
			break;
	}   
	if(i < 0) // all button up
	{
		core_data->atr_enable = false;
		if(first_continue_flag == true) {
			first_continue_flag = false;
		}
		if(!finger_press)
		{
			ts_info("keymapping all button up");
			input_report_key(input_dev, BTN_TOUCH, 0);
			input_sync(input_dev);
		}
	}
	random_x += 1; if(random_x > 5) random_x = -5;
	random_y += 1; if(random_y > 5) random_y = -5;
	random_pressure += 1; if(random_pressure > 20) random_pressure = -20;
	random_major += 1; if(random_major > 5) random_major = -5;

	mutex_unlock(&input_dev->mutex);
}

int goodix_change_config(struct goodix_ts_core *core_data, const char *fw_name)
{
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	const struct firmware *cfg_img;
	struct goodix_ts_config *config = NULL;
	int r;

	ts_info("******IN");

	disable_irq(core_data->irq);

	/*request configuration*/
	r = request_firmware(&cfg_img, fw_name, ts_dev->dev);
	if (r < 0) {
		ts_err("cfg file [%s] not available,errno:%d", fw_name, r);
		goto exit;
	} else
		ts_info("cfg file [%s] is ready", fw_name);

	config = kzalloc(sizeof(struct goodix_ts_config), GFP_KERNEL);
	if (config == NULL) {
		ts_err("Memory alloc err");
		goto exit;
	}

	/*parse cfg data*/
	if (goodix_ts_convert_0x_data(cfg_img->data, cfg_img->size,
				config->data, &config->length)) {
		ts_err("convert config data FAILED");
		goto exit;
	}
	
	config->reg_base = ts_dev->reg.cfg_addr;
	mutex_init(&config->lock);
	config->initialized = TS_CFG_STABLE;

	if (ts_dev->hw_ops->send_config)
		r = ts_dev->hw_ops->send_config(ts_dev, config);
	ts_info("send cfg result %d", r);
exit:
	enable_irq(core_data->irq);

	if (config) {
		kfree(config);
		config = NULL;
	}

	if (cfg_img) {
		release_firmware(cfg_img);
		cfg_img = NULL;
	}

	ts_info("******OUT");
	
	return 0;
}

static ssize_t asus_proc_glove_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	char cfg_name[32] = {0x00};
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	ts_info("setting - glove mode");
	if ((strncmp(messages, "0", 1) == 0) && (atomic_read(&gts_core_data->glove_mode) == 1)) {
		atomic_set(&gts_core_data->glove_mode, 0);
		strlcpy(cfg_name, GOODIX_DEFAULT_CFG_NAME, sizeof(cfg_name));
		goodix_change_config(gts_core_data, cfg_name);
	} else if ((strncmp(messages, "0", 1) == 1) && (atomic_read(&gts_core_data->glove_mode) == 0)) {
		atomic_set(&gts_core_data->glove_mode, 1);
		strlcpy(cfg_name, GOODIX_GLOVE_CFG_NAME, sizeof(cfg_name));
		goodix_change_config(gts_core_data, cfg_name);
	} else {
		ts_info("There is no need to update the glove config");
	}

	return len;
}

static ssize_t asus_proc_glove_read(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%d\n", atomic_read(&gts_core_data->glove_mode));
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations asus_proc_glove_ops = {
	.write = asus_proc_glove_write,
	.read  = asus_proc_glove_read,
};

static ssize_t atr_queue_full(struct atr_queue *q){
	if (q == NULL){
		return -1;
	}else if(q->buf_size == q->capacity){
		return 1;
	}else{
		return 0;
	}
}

static ssize_t atr_queue_empty(struct atr_queue *q){
	if (q == NULL){
		return -1;
	}else if(q->buf_size == 0) {
		return 1;
	}else {
		return 0;
	}
}

static ssize_t atr_buf_write(struct atr_queue *q, u8 id, u8 active, u16 x, u16 y, u16 p, u16 m)
{
	if (q == NULL){
		return -1;
	} else if (atr_queue_full(q) == 1) {
		ts_info("[ATR] buf status = full");
		return 0;
	} else {
		spin_lock(&q->buffer_lock);
		q->tail = (q->tail + 1) % q->capacity;
		q->data[q->tail].id = id;
		q->data[q->tail].active = active;
		q->data[q->tail].x = x;
		q->data[q->tail].y = y;
		q->data[q->tail].p = p;
		q->data[q->tail].m = m;
		q->buf_size++;
		spin_unlock(&q->buffer_lock);
		ts_info("[ATR_Q_W] ID %d active %d x %d y %d p %d m %d", q->data[q->tail].id, q->data[q->tail].active, q->data[q->tail].x, q->data[q->tail].y,  q->data[q->tail].p,  q->data[q->tail].m);
		return 1;
	}
}

static ssize_t atr_buf_read(struct atr_queue *q, struct input_dev *input_dev)
{
	int id_buf[TOTAL_SLOT];
	int i = 0;
	
	memset(id_buf,-1,TOTAL_SLOT);
	
	if (q == NULL){
		return -1;
	} else if (atr_queue_empty(q) == 1) {
		return 0;
	} else {
		do {
			struct goodix_atr_data* item = &(q->data[q->head]);
			// check ID status
			for(i = 0; i < TOTAL_SLOT; i++) {
				if (id_buf[i] == -1) {
					// ts_info("[ATRD_Q_R] end of id_buf[%d] = -1, skip......", i);
					break;
				}
				if (item->id == id_buf[i]) {
					// ts_info("[ATRD_Q_R] find id_buf[%d] = %d, skip......", i, item->id);
					return 0;
				}
			}
			spin_lock(&q->buffer_lock);
			q->head = (q->head + 1) % q->capacity;
			q->buf_size--;
			spin_unlock(&q->buffer_lock);
			ts_info("[ATR_Q_R] ID %d active %d x %d y %d p %d m %d", item->id, item->active, item->x, item->y, item->p, item->m);
			
			input_mt_slot(input_dev, item->id);
			if (item->active == true) {
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
				input_report_abs(input_dev, ABS_MT_PRESSURE, item->p);
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, item->m);
				input_report_abs(input_dev, ABS_MT_POSITION_X, item->x);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, item->y);
			} else {
				input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
			}
			// save ID 
			for(i = 0; i < TOTAL_SLOT; i++) {
				if (id_buf[i] == -1) {
					id_buf[i] = item->id;
					// ts_info("[ATRD_Q_R] save id_buf[%d] = %d", i, id_buf[i]);
					break;
				}
			}
		} while (atr_queue_empty(q) != 1);
		// ts_info("[ATR_Q] ==========");
		return 0;
	}
}

static struct atr_queue* atr_buf_init(unsigned int _capacity)
{
	struct atr_queue *ATRQueue = (struct atr_queue *)kmalloc(sizeof(struct atr_queue), GFP_KERNEL);

	if (ATRQueue == NULL ){
		ts_info("Malloc failed");
		return NULL;
	} else {
		ATRQueue->tail = -1;
		ATRQueue->head = 0;
		ATRQueue->buf_size = 0;
		ATRQueue->capacity = _capacity;
		ATRQueue->data = (struct goodix_atr_data *)kmalloc(_capacity * sizeof(struct goodix_atr_data), GFP_KERNEL);
		if (ATRQueue->data == NULL) {
			ts_info("Malloc failed");
			kfree(ATRQueue);
			return NULL;
		} else {
			spin_lock_init(&ATRQueue->buffer_lock);
			return ATRQueue;
		}
	}
}
// ASUS_BSP --- Touch

static void goodix_ts_report_finger(struct input_dev *dev,
		struct goodix_touch_data *touch_data)
{
	unsigned int touch_num = touch_data->touch_num;
	static u32 pre_fin;
	int i;
// ASUS_BSP +++ Touch
	struct goodix_ts_core *core_data = input_get_drvdata(dev);
	static int touch_count = 0;
	static int driver_touch_count = 0;
	static bool first_data = true;
	u8 buffer[BUF_LEN_4154]={0x0};
	static int start_count = 0;
	static int end_count = 0;
	int chip_diff_count = 0;
// ASUS_BSP --- Touch

	// gesture mode lost key_U +++
	if(get_aod_processing() == true) {
		msleep(300);
		if(get_aod_processing() == true) {
			ts_info("[KEY_U] before touch event");
			enable_aod_processing(false);
		}
	}
	// gesture mode lost key_U---

	/*first touch down and last touch up condition*/
	if (touch_num && !pre_fin) {
		if (core_data->atr_enable){
			ts_info("atr pressed, ignore touch down event"); 
		} else {
			input_report_key(dev, BTN_TOUCH, 1);
			finger_press = true;
			if (GoodixTSINTDebug == true) {
				start_count = get_chip_int(core_data);
			}
		}
	} else if (!touch_num && pre_fin) {
		if (core_data->atr_enable){
		    ts_info("game enable, ignore up event"); 
		} else {
			if (aod_press == true || out_of_KEY_F == true) {
				input_switch_key(dev, KEY_U);
				ts_info("KEY_U");
				aod_press = false;
				out_of_KEY_F = false;
				key_i = -1;
				if (key_o_sync == true || out_of_KEY_O == true) {
					key_o_sync = false;
					out_of_KEY_O = false;
					ts_info("release KEY_O");
				}
			}
			input_report_key(dev, BTN_TOUCH, 0);
			finger_press = false;
			first_data = true;
			
			if (GoodixTSINTDebug == true) {
				end_count = get_chip_int(core_data);
				chip_diff_count = ((driver_touch_count >> 8) << 8) + end_count - start_count + 1;
				if (end_count <= start_count)
					chip_diff_count += 256;

				ts_info("HW chip (%5d) | SW driver (%5d)", chip_diff_count, driver_touch_count);
				if(chip_diff_count != driver_touch_count)
					ts_info("HW chip != SW driver");
			}
			touch_count = 0;
			driver_touch_count = 1;
		}
	}

	pre_fin = touch_num;

	for (i = 0; i < GOODIX_MAX_TOUCH; i++) {
		if (!touch_data->coords[i].status)
			continue;
		if (touch_data->coords[i].status == TS_RELEASE) {
			if((aod_press == true || out_of_KEY_F == true) && (key_i == i)) {
				input_switch_key(dev, KEY_U);
				ts_info("KEY_U");
				aod_press = false;
				out_of_KEY_F = false;
				key_i = -1;
			}
			if (((key_o_sync == true) || (out_of_KEY_O == true)) && (key_i == i)) {
				ts_info("release KEY_O");
				key_o_sync = false;
				out_of_KEY_O = false;
				key_i = -1;
			}

			input_mt_slot(dev, i);
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
			if((core_data->atr_enable) && (touch_num == 0)){
				finger_press = false;
			}
			continue;
		}
		if (core_data->aod_test_mode == 0) {
			input_mt_slot(dev, i);
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);
			input_report_abs(dev, ABS_MT_POSITION_X,
					 touch_data->coords[i].x);
			input_report_abs(dev, ABS_MT_POSITION_Y,
					 touch_data->coords[i].y);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR,
					 touch_data->coords[i].w);
			input_report_abs(dev, ABS_MT_PRESSURE,
					 touch_data->coords[i].w);
// ASUS_BSP +++ Touch
			driver_touch_count ++;
			touch_count ++ ;
			if ((touch_count == 1) && (first_data == true)) {
				first_data = false;
				ts_info("[%3d][%2d]%4d|%4d|%3d|", touch_count, i, touch_data->coords[i].x, touch_data->coords[i].y, touch_data->coords[i].w);
				read_chip_cmd(core_data, GOODIX_ADDR_GOODIX_DEBUG_CMD, BUF_LEN_4154, buffer);
			}
			if (touch_count >= print_touch_count_max) {
				ts_info("[%3d][%2d]%4d|%4d|%3d|", touch_count, i, touch_data->coords[i].x, touch_data->coords[i].y, touch_data->coords[i].w);
				read_chip_cmd(core_data, GOODIX_ADDR_GOODIX_DEBUG_CMD, BUF_LEN_4154, buffer);
				touch_count = 0;
			}
// ASUS_BSP --- Touch
		} else if (core_data->aod_test_mode == 1) {
			if (!(fod_position[0] < touch_data->coords[i].x && touch_data->coords[i].x < fod_position[1] && 
				fod_position[2] < touch_data->coords[i].y && touch_data->coords[i].y < fod_position[3])) {
				if (aod_press == false) {
					input_switch_key(dev, KEY_L);
					ts_info("KEY_L");
				}
			} else {
				// KEY_F
				if (aod_press == false) {
					key_i = i;
					if(TouchArea >= FPArea) {
						if (key_o_sync == true) {
							key_o_sync = false;
							ts_info("release KEY_O");
						}
						data_x = touch_data->coords[i].x;
						data_y = touch_data->coords[i].y;
						ts_info("KEY_F X = %d, Y = %d", data_x, data_y);
						input_switch_key(dev, KEY_F);
						asus_display_report_fod_touched();
						ts_info("KEY_F");
						aod_press = true;
					} else {
						if (key_o_sync == false) {
							input_switch_key(dev, KEY_O);
							key_o_sync = true;
							ts_info("KEY_O");
						}
						ts_info("FPArea %d < %d", TouchArea, FPArea);
					}
				}
				if(!(((key_o_sync == true) || (aod_press == true)) && (key_i == i))){
					input_mt_slot(dev, i);
					input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);
					input_report_abs(dev, ABS_MT_POSITION_X,
							 touch_data->coords[i].x);
					input_report_abs(dev, ABS_MT_POSITION_Y,
							 touch_data->coords[i].y);
					input_report_abs(dev, ABS_MT_TOUCH_MAJOR,
							 touch_data->coords[i].w);
					input_report_abs(dev, ABS_MT_PRESSURE,
							 touch_data->coords[i].w);
				}
			}
		} else if (core_data->aod_test_mode == 2) {
			// KEY_F
			if (fod_position[0] < touch_data->coords[i].x && touch_data->coords[i].x < fod_position[1] && 
				fod_position[2] < touch_data->coords[i].y && touch_data->coords[i].y < fod_position[3]) {
				if ((aod_press == false) && (out_of_KEY_F == false)) {
					key_i = i;
					if(TouchArea >= FPArea) {
						if (key_o_sync == true) {
							key_o_sync = false;
							ts_info("release KEY_O");
						}
						data_x = touch_data->coords[i].x;
						data_y = touch_data->coords[i].y;
						ts_info("KEY_F X = %d, Y = %d", data_x, data_y);
						input_switch_key(dev, KEY_F);
						asus_display_report_fod_touched();
						ts_info("KEY_F");
						aod_press = true;
					} else {
						if (key_o_sync == false) {
							input_switch_key(dev, KEY_O);
							key_o_sync = true;
							ts_info("KEY_O");
						}
						ts_info("FPArea %d < %d", TouchArea, FPArea);
					}
				}
			} else {
				if (key_o_sync == true) {
					key_o_sync = false;
					out_of_KEY_O = true;
					ts_info("KEY_O out of FPArea");
				}
				if (aod_press == true) {
					aod_press = false;
					out_of_KEY_F = true;
					ts_info("KEY_F out of FPArea");
				}
			}
			if(!(((key_o_sync == true) || (aod_press == true)) && (key_i == i))){
				input_mt_slot(dev, i);
				input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);
				input_report_abs(dev, ABS_MT_POSITION_X,
						 touch_data->coords[i].x);
				input_report_abs(dev, ABS_MT_POSITION_Y,
						 touch_data->coords[i].y);
				input_report_abs(dev, ABS_MT_TOUCH_MAJOR,
						 touch_data->coords[i].w);
				input_report_abs(dev, ABS_MT_PRESSURE,
						 touch_data->coords[i].w);
				driver_touch_count ++;
				touch_count ++ ;
				if ((touch_count == 1) && (first_data == true)) {
					first_data = false;
					ts_info("[%3d][%2d]%4d|%4d|%3d|", touch_count, i, touch_data->coords[i].x, touch_data->coords[i].y, touch_data->coords[i].w);
				}
				if (touch_count >= print_touch_count_max) {
					ts_info("[%3d][%2d]%4d|%4d|%3d|", touch_count, i, touch_data->coords[i].x, touch_data->coords[i].y, touch_data->coords[i].w);
					touch_count = 0;
				}
			}
		}
		finger_press = true;
	}
	// ATR
	if (atr_queue_empty(atr_buf_queue) != 1) {
		atr_buf_read(atr_buf_queue, dev);
	}

	/* report panel key */
	for (i = 0; i < GOODIX_MAX_TP_KEY; i++) {
		if (!touch_data->keys[i].status)
			continue;
		if (touch_data->keys[i].status == TS_TOUCH)
			input_report_key(dev, touch_data->keys[i].code, 1);
		else if (touch_data->keys[i].status == TS_RELEASE)
			input_report_key(dev, touch_data->keys[i].code, 0);
	}
	input_sync(dev);
}

/**
 * goodix_ts_threadirq_func - Bottom half of interrupt
 * This functions is excuted in thread context,
 * sleep in this function is permit.
 *
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static irqreturn_t goodix_ts_threadirq_func(int irq, void *data)
{
	struct goodix_ts_core *core_data = data;
	struct goodix_ts_device *ts_dev =  core_data->ts_dev;
	struct goodix_ext_module *ext_module, *next;
	struct goodix_ts_event *ts_event = &core_data->ts_event;
	struct timeval time_in, time_out;
	suseconds_t diff;
	
	u8 irq_flag = 0;
	int r;

	if (enable_touch_time_debug == true) {
		do_gettimeofday(&time_in);
		//ts_info("Touch +++ [%d %d]", time_in.tv_sec, time_in.tv_usec);
	}
	core_data->irq_trig_cnt++;
	/* inform external module */
	mutex_lock(&goodix_modules.mutex);
	list_for_each_entry_safe(ext_module, next,
				 &goodix_modules.head, list) {
		if (!ext_module->funcs->irq_event)
			continue;
		r = ext_module->funcs->irq_event(core_data, ext_module);
		if (r == EVT_CANCEL_IRQEVT) {
			mutex_unlock(&goodix_modules.mutex);
			return IRQ_HANDLED;
		}
	}
	mutex_unlock(&goodix_modules.mutex);

	/* read touch data from touch device */
	r = ts_dev->hw_ops->event_handler(ts_dev, ts_event);
	if (likely(r >= 0)) {
		if (ts_event->event_type == EVENT_TOUCH) {
			/* report touch */
			goodix_ts_report_finger(core_data->input_dev,
					&ts_event->touch_data);
		}
		if (ts_dev->board_data.pen_enable &&
			ts_event->event_type == EVENT_PEN) {
			goodix_ts_report_pen(core_data->pen_dev,
					&ts_event->pen_data);
		}
	}

	/* clean irq flag */
	irq_flag = 0;
	ts_dev->hw_ops->write_trans(ts_dev, ts_dev->reg.coor, &irq_flag, 1);
	
	if (enable_touch_time_debug == true) {
		do_gettimeofday(&time_out);
		//ts_info("Touch --- [%d %d]", time_out.tv_sec, time_out.tv_usec);
		if (time_out.tv_sec == time_in.tv_sec)
			diff = time_out.tv_usec - time_in.tv_usec;
		else {
			diff = 1000000 + time_out.tv_usec - time_in.tv_usec;
		}
		ts_info("Delta %d", diff);
	}

	return IRQ_HANDLED;
}

/**
 * goodix_ts_init_irq - Requset interrput line from system
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_irq_setup(struct goodix_ts_core *core_data)
{
	const struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int r;

	/* if ts_bdata-> irq is invalid */
	if (ts_bdata->irq <= 0)
		core_data->irq = gpio_to_irq(ts_bdata->irq_gpio);
	else
		core_data->irq = ts_bdata->irq;

	ts_info("IRQ:%u,flags:%d", core_data->irq, (int)ts_bdata->irq_flags);
	r = devm_request_threaded_irq(&core_data->pdev->dev,
				      core_data->irq, NULL,
				      goodix_ts_threadirq_func,
				      ts_bdata->irq_flags | IRQF_ONESHOT,
				      GOODIX_CORE_DRIVER_NAME,
				      core_data);
	if (r < 0)
		ts_err("Failed to requeset threaded irq:%d", r);
	else
		atomic_set(&core_data->irq_enabled, 1);

	return r;
}

/**
 * goodix_ts_irq_enable - Enable/Disable a irq
 * @core_data: pointer to touch core data
 * enable: enable or disable irq
 * return: 0 ok, <0 failed
 */
int goodix_ts_irq_enable(struct goodix_ts_core *core_data,
			bool enable)
{
	if (enable) {
		if (!atomic_cmpxchg(&core_data->irq_enabled, 0, 1)) {
			enable_irq(core_data->irq);
			ts_debug("Irq enabled");
		}
	} else {
		if (atomic_cmpxchg(&core_data->irq_enabled, 1, 0)) {
			disable_irq(core_data->irq);
			ts_debug("Irq disabled");
		}
	}

	return 0;
}
EXPORT_SYMBOL(goodix_ts_irq_enable);

/**
 * goodix_ts_power_init - Get regulator for touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_power_init(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata;
	struct device *dev = NULL;
	int r = 0;

	ts_info("Power init");
	/* dev:i2c client device or spi slave device*/
	dev =  core_data->ts_dev->dev;
	ts_bdata = board_data(core_data);

	if (strlen(ts_bdata->avdd_name)) {
		core_data->avdd = devm_regulator_get(dev,
				 ts_bdata->avdd_name);
		if (IS_ERR_OR_NULL(core_data->avdd)) {
			r = PTR_ERR(core_data->avdd);
			ts_err("Failed to get regulator avdd:%d", r);
			core_data->avdd = NULL;
			return r;
		}
	} else {
		ts_info("Avdd name is NULL[skip]");
	}

	return r;
}

/**
 * goodix_ts_power_on - Turn on power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_power_on(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int r;

	ts_info("Device power on");
	if (core_data->power_on)
		return 0;

	if (!core_data->avdd) {
		core_data->power_on = 1;
		return 0;
	}

	r = regulator_enable(core_data->avdd);
	if (!r) {
		ts_info("regulator enable SUCCESS");
		if (ts_bdata->power_on_delay_us)
			usleep_range(ts_bdata->power_on_delay_us,
				     ts_bdata->power_on_delay_us);
	} else {
		ts_err("Failed to enable analog power:%d", r);
		return r;
	}

	core_data->power_on = 1;
	return 0;
}

/**
 * goodix_ts_power_off - Turn off power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_power_off(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int r;

	ts_info("Device power off");
	if (!core_data->power_on)
		return 0;

	if (core_data->avdd) {
		r = regulator_disable(core_data->avdd);
		if (!r) {
			ts_info("regulator disable SUCCESS");
			if (ts_bdata->power_off_delay_us)
				usleep_range(ts_bdata->power_off_delay_us,
					     ts_bdata->power_off_delay_us);
		} else {
			ts_err("Failed to disable analog power:%d", r);
			return r;
		}
	}

	core_data->power_on = 0;
	return 0;
}

#ifdef CONFIG_PINCTRL
/**
 * goodix_ts_pinctrl_init - Get pinctrl handler and pinctrl_state
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_pinctrl_init(struct goodix_ts_core *core_data)
{
	int r = 0;

	ts_info("goodix_ts_pinctrl_init start");
	/* get pinctrl handler from of node */
	core_data->pinctrl = devm_pinctrl_get(core_data->ts_dev->dev);
	if (IS_ERR_OR_NULL(core_data->pinctrl)) {
		ts_info("Failed to get pinctrl handler[need confirm]");
		core_data->pinctrl = NULL;
		return -EINVAL;
	}
	ts_debug("success get pinctrl");
	/* active state */
	core_data->pin_sta_active = pinctrl_lookup_state(core_data->pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(core_data->pin_sta_active)) {
		r = PTR_ERR(core_data->pin_sta_active);
		ts_err("Failed to get pinctrl state:%s, r:%d",
				PINCTRL_STATE_ACTIVE, r);
		core_data->pin_sta_active = NULL;
		goto exit_pinctrl_put;
	}
	ts_debug("success get avtive pinctrl state");

	/* suspend state */
	core_data->pin_sta_suspend = pinctrl_lookup_state(core_data->pinctrl,
				PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(core_data->pin_sta_suspend)) {
		r = PTR_ERR(core_data->pin_sta_suspend);
		ts_err("Failed to get pinctrl state:%s, r:%d",
				PINCTRL_STATE_SUSPEND, r);
		core_data->pin_sta_suspend = NULL;
		goto exit_pinctrl_put;
	}
	ts_debug("success get suspend pinctrl state");
	ts_info("goodix_ts_pinctrl_init complete");

	return 0;
exit_pinctrl_put:
	devm_pinctrl_put(core_data->pinctrl);
	core_data->pinctrl = NULL;
	return r;
}
#endif

/**
 * goodix_ts_gpio_setup - Request gpio resources from GPIO subsysten
 *	reset_gpio and irq_gpio number are obtained from goodix_ts_device
 *  which created in hardware layer driver. e.g.goodix_xx_i2c.c
 *	A goodix_ts_device should set those two fileds to right value
 *	before registed to touch core driver.
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_gpio_setup(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	int r = 0;

	ts_info("GPIO setup,reset-gpio:%d, irq-gpio:%d",
		ts_bdata->reset_gpio, ts_bdata->irq_gpio);
	ts_info("before setup gpio reset = %d", gpio_get_value(ts_bdata->reset_gpio));

	/*
	 * after kenerl3.13, gpio_ api is deprecated, new
	 * driver should use gpiod_ api.
	 */
	r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->reset_gpio,
				  GPIOF_OUT_INIT_HIGH, "ts_reset_gpio");
	if (r < 0) {
		ts_err("Failed to request reset gpio, r:%d", r);
		return r;
	}

	r = devm_gpio_request_one(&core_data->pdev->dev, ts_bdata->irq_gpio,
				  GPIOF_IN, "ts_irq_gpio");
	if (r < 0) {
		ts_err("Failed to request irq gpio, r:%d", r);
		return r;
	}
	
	ts_info("reset touch ic for init");
	gpio_direction_output(ts_bdata->reset_gpio, 0);
	udelay(10000);
	gpio_direction_output(ts_bdata->reset_gpio, 1);
	msleep(100);

	ts_info("gpio reset = %d", gpio_get_value(ts_bdata->reset_gpio));
	ts_info("gpio irq = %d", gpio_get_value(ts_bdata->irq_gpio));

	return 0;
}

/**
 * goodix_input_set_params - set input parameters
 */
static void goodix_ts_set_input_params(struct input_dev *input_dev,
		struct goodix_ts_board_data *ts_bdata)
{
	int i;

	if (ts_bdata->swap_axis)
		swap(ts_bdata->panel_max_x, ts_bdata->panel_max_y);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, ts_bdata->panel_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, ts_bdata->panel_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, ts_bdata->panel_max_w, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, ts_bdata->panel_max_p, 0, 0);

	if (ts_bdata->panel_max_key) {
		for (i = 0; i < ts_bdata->panel_max_key; i++)
			input_set_capability(input_dev, EV_KEY,
					     ts_bdata->panel_key_map[i]);
	}
}

/**
 * goodix_ts_input_dev_config - Requset and config a input device
 *  then register it to input sybsystem.
 *  NOTE that some hardware layer may provide a input device
 *  (ts_dev->input_dev not NULL).
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_input_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct input_dev *input_dev = NULL;
	int r;

	input_dev = input_allocate_device();
	if (!input_dev) {
		ts_err("Failed to allocated input device");
		return -ENOMEM;
	}

	core_data->input_dev = input_dev;
	input_set_drvdata(input_dev, core_data);

	input_dev->name = GOODIX_CORE_DRIVER_NAME;
	input_dev->phys = GOOIDX_INPUT_PHYS;
	input_dev->id.product = 0xDEAD;
	input_dev->id.vendor = 0xBEEF;
	input_dev->id.version = 10427;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	//__set_bit(BTN_TOOL_FINGER, input_dev->keybit);

#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

	/* set input parameters */
	goodix_ts_set_input_params(input_dev, ts_bdata);

#ifdef INPUT_TYPE_B_PROTOCOL
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0)
	input_mt_init_slots(input_dev, GOODIX_ASUS_MAX_TOUCH,
			    INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input_dev, GOODIX_ASUS_MAX_TOUCH);
#endif
#endif

	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_UP);

// ASUS_BSP +++ Touch
	input_set_capability(input_dev, EV_KEY, KEY_W);
	input_set_capability(input_dev, EV_KEY, KEY_S);
	input_set_capability(input_dev, EV_KEY, KEY_E);
	input_set_capability(input_dev, EV_KEY, KEY_Z);
	input_set_capability(input_dev, EV_KEY, KEY_V);
	input_set_capability(input_dev, EV_KEY, KEY_M);
	input_set_capability(input_dev, EV_KEY, KEY_O);

	input_set_capability(input_dev, EV_KEY, KEY_F);
	input_set_capability(input_dev, EV_KEY, KEY_U);
	input_set_capability(input_dev, EV_KEY, KEY_L);
	
	input_set_capability(input_dev, EV_KEY, KEY_PAUSE);
	input_set_capability(input_dev, EV_KEY, KEY_REWIND);
	input_set_capability(input_dev, EV_KEY, KEY_FORWARD);
// ASUS_BSP --- Touch
	r = input_register_device(input_dev);
	if (r < 0) {
		ts_err("Unable to register input device");
		input_free_device(input_dev);
		return r;
	}

	return 0;
}

static int goodix_ts_pen_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = board_data(core_data);
	struct input_dev *pen_dev = NULL;
	int r;

	pen_dev = input_allocate_device();
	if (!pen_dev) {
		ts_err("Failed to allocated pen device");
		return -ENOMEM;
	}
	core_data->pen_dev = pen_dev;
	input_set_drvdata(pen_dev, core_data);

	pen_dev->name = GOODIX_PEN_DRIVER_NAME;
	pen_dev->id.product = 0xDEAD;
	pen_dev->id.vendor = 0xBEEF;
	pen_dev->id.version = 10427;

	pen_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	__set_bit(ABS_X, pen_dev->absbit);
	__set_bit(ABS_Y, pen_dev->absbit);
	__set_bit(BTN_STYLUS, pen_dev->keybit);
	__set_bit(BTN_STYLUS2, pen_dev->keybit);
	__set_bit(BTN_TOUCH, pen_dev->keybit);
	__set_bit(BTN_TOOL_PEN, pen_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
	input_set_abs_params(pen_dev, ABS_X, 0, ts_bdata->panel_max_x, 0, 0);
	input_set_abs_params(pen_dev, ABS_Y, 0, ts_bdata->panel_max_y, 0, 0);
	input_set_abs_params(pen_dev, ABS_PRESSURE, 0,
			     GOODIX_PEN_MAX_PRESSURE, 0, 0);

	r = input_register_device(pen_dev);
	if (r < 0) {
		ts_err("Unable to register pen device");
		input_free_device(pen_dev);
		return r;
	}

	return 0;
}

void goodix_ts_input_dev_remove(struct goodix_ts_core *core_data)
{
	input_unregister_device(core_data->input_dev);
	input_free_device(core_data->input_dev);
	core_data->input_dev = NULL;
}

void goodix_ts_pen_dev_remove(struct goodix_ts_core *core_data)
{
	input_unregister_device(core_data->pen_dev);
	input_free_device(core_data->pen_dev);
	core_data->pen_dev = NULL;
}

/**
 * goodix_ts_esd_work - check hardware status and recovery
 *  the hardware if needed.
 */
static void goodix_ts_esd_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct goodix_ts_esd *ts_esd = container_of(dwork,
			struct goodix_ts_esd, esd_work);
	struct goodix_ts_core *core = container_of(ts_esd,
			struct goodix_ts_core, ts_esd);
	const struct goodix_ts_hw_ops *hw_ops = ts_hw_ops(core);
	u8 data = GOODIX_ESD_TICK_WRITE_DATA;
	int r = 0;

	if (!atomic_read(&ts_esd->esd_on))
		return;

	if (hw_ops->check_hw)
		r = hw_ops->check_hw(core->ts_dev);
	if (r < 0) {
		goodix_ts_power_off(core);
		goodix_ts_power_on(core);
		if (hw_ops->reset)
			hw_ops->reset(core->ts_dev);

		/*init dynamic esd*/
		r = hw_ops->write_trans(core->ts_dev, core->ts_dev->reg.esd,
					&data, 1);
		if (r < 0)
			ts_err("failed init dynamic esd");
	} else {
		/*init dynamic esd*/
		r = hw_ops->write_trans(core->ts_dev,
				core->ts_dev->reg.esd,
				&data, 1);
		if (r < 0)
			ts_err("failed init watch dog");
	}

	if (atomic_read(&ts_esd->esd_on))
		schedule_delayed_work(&ts_esd->esd_work, 2 * HZ);
}

/**
 * goodix_ts_esd_on - turn on esd protection
 */
static void goodix_ts_esd_on(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;

	if (core->ts_dev->reg.esd == 0)
		return;

	atomic_set(&ts_esd->esd_on, 1);
	if (!schedule_delayed_work(&ts_esd->esd_work, 2 * HZ)) {
		ts_info("esd work already in workqueue");
	}
	ts_info("esd on");
}

/**
 * goodix_ts_esd_off - turn off esd protection
 */
static void goodix_ts_esd_off(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;
	int ret;

	atomic_set(&ts_esd->esd_on, 0);
	ret = cancel_delayed_work_sync(&ts_esd->esd_work);
	ts_info("Esd off, esd work state %d", ret);
}

/**
 * goodix_esd_notifier_callback - notification callback
 *  under certain condition, we need to turn off/on the esd
 *  protector, we use kernel notify call chain to achieve this.
 *
 *  for example: before firmware update we need to turn off the
 *  esd protector and after firmware update finished, we should
 *  turn on the esd protector.
 */
static int goodix_esd_notifier_callback(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct goodix_ts_esd *ts_esd = container_of(nb,
			struct goodix_ts_esd, esd_notifier);

	switch (action) {
	case NOTIFY_FWUPDATE_START:
	case NOTIFY_SUSPEND:
	case NOTIFY_ESD_OFF:
		goodix_ts_esd_off(ts_esd->ts_core);
		break;
	case NOTIFY_FWUPDATE_FAILED:
	case NOTIFY_FWUPDATE_SUCCESS:
	case NOTIFY_RESUME:
	case NOTIFY_ESD_ON:
		goodix_ts_esd_on(ts_esd->ts_core);
		break;
	default:
		break;
	}

	return 0;
}

/**
 * goodix_ts_esd_init - initialize esd protection
 */
int goodix_ts_esd_init(struct goodix_ts_core *core)
{
	struct goodix_ts_esd *ts_esd = &core->ts_esd;
	struct goodix_ts_device *dev = core->ts_dev;
	u8 data = GOODIX_ESD_TICK_WRITE_DATA;
	int r;

	if (!dev->hw_ops->check_hw || !dev->reg.esd) {
		ts_info("missing key info for esd check");
		return 0;
	}

	INIT_DELAYED_WORK(&ts_esd->esd_work, goodix_ts_esd_work);
	ts_esd->ts_core = core;
	atomic_set(&ts_esd->esd_on, 0);
	ts_esd->esd_notifier.notifier_call = goodix_esd_notifier_callback;
	goodix_ts_register_notifier(&ts_esd->esd_notifier);

	/*init dynamic esd*/
	r = dev->hw_ops->write_trans(core->ts_dev, core->ts_dev->reg.esd,
				     &data, 1);
	if (r < 0)
		ts_err("failed init dynamic esd[ignore]");

	goodix_ts_esd_on(core);

	return 0;
}

static void goodix_ts_release_connects(struct goodix_ts_core *core_data)
{
	struct input_dev *input_dev = core_data->input_dev;
	struct input_mt *mt = input_dev->mt;
	int i;

	if (mt) {
		for (i = 0; i < mt->num_slots; i++) {
			input_mt_slot(input_dev, i);
			input_mt_report_slot_state(input_dev,
					MT_TOOL_FINGER,
					false);
		}
		input_report_key(input_dev, BTN_TOUCH, 0);
		input_mt_sync_frame(input_dev);
		input_sync(input_dev);
	}
}

/**
 * goodix_ts_suspend - Touchscreen suspend function
 * Called by PM/FB/EARLYSUSPEN module to put the device to  sleep
 */
static int goodix_ts_suspend(struct goodix_ts_core *core_data)
{
	struct goodix_ext_module *ext_module, *next;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;
	int r;

	ts_info("Suspend start");

	/*
	 * notify suspend event, inform the esd protector
	 * and charger detector to turn off the work
	 */
	goodix_ts_blocking_notify(NOTIFY_SUSPEND, NULL);

	/* inform external module */
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			if (!ext_module->funcs->before_suspend)
				continue;

			r = ext_module->funcs->before_suspend(core_data,
							      ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

	/* disable irq */
	goodix_ts_irq_enable(core_data, false);

	/* let touch ic work in sleep mode */
	if (ts_dev && ts_dev->hw_ops->suspend)
		ts_dev->hw_ops->suspend(ts_dev);
	atomic_set(&core_data->suspended, 1);

#ifdef CONFIG_PINCTRL
	if (core_data->pinctrl) {
		r = pinctrl_select_state(core_data->pinctrl,
				core_data->pin_sta_suspend);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif

	/* inform exteranl modules */
	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			if (!ext_module->funcs->after_suspend)
				continue;

			r = ext_module->funcs->after_suspend(core_data,
							     ext_module);
			if (r == EVT_CANCEL_SUSPEND) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

out:
// ASUS_BSP +++ Touch
	ts_info("keymapping SUSPEND atr_enable=%d", core_data->atr_enable);
	if(core_data->atr_enable) { // release airtrigger fingers
		input_report_key(core_data->input_dev, BTN_TOUCH, 0);
		input_sync(core_data->input_dev);
		ts_info("keymapping release all Airtrigger");
		LastATR = LastATL = 0;
		core_data->atr_enable = false;
	}
	if (aod_press == true) {
		input_switch_key(core_data->input_dev, KEY_U);
		ts_info("KEY_U");
		aod_press = false;
		key_i = -1;
		if (key_o_sync == true || out_of_KEY_O == true) {
			key_o_sync = false;
			out_of_KEY_O = false;
			ts_info("release KEY_O");
		}
		if (out_of_KEY_F == true)
			out_of_KEY_F = false;
	}
	if (finger_press == true) { // normal touch
		input_report_key(core_data->input_dev, BTN_TOUCH, 0);
		input_sync(core_data->input_dev);
		ts_info("release all touch");
		finger_press = false;
	}
// ASUS_BSP --- Touch
	ts_info("Suspend end");
	return 0;
}

/**
 * goodix_ts_resume - Touchscreen resume function
 * Called by PM/FB/EARLYSUSPEN module to wakeup device
 */
static int goodix_ts_resume(struct goodix_ts_core *core_data)
{
	struct goodix_ext_module *ext_module, *next;
	struct goodix_ts_device *ts_dev =
				core_data->ts_dev;
	int r;

	ts_info("Resume start");
	goodix_ts_release_connects(core_data);

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			if (!ext_module->funcs->before_resume)
				continue;

			r = ext_module->funcs->before_resume(core_data,
							     ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

#ifdef CONFIG_PINCTRL
	if (core_data->pinctrl) {
		r = pinctrl_select_state(core_data->pinctrl,
					 core_data->pin_sta_active);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif

	atomic_set(&core_data->suspended, 0);
	/* resume device */
	if (ts_dev && ts_dev->hw_ops->resume)
		ts_dev->hw_ops->resume(ts_dev);

	mutex_lock(&goodix_modules.mutex);
	if (!list_empty(&goodix_modules.head)) {
		list_for_each_entry_safe(ext_module, next,
					 &goodix_modules.head, list) {
			if (!ext_module->funcs->after_resume)
				continue;

			r = ext_module->funcs->after_resume(core_data,
							    ext_module);
			if (r == EVT_CANCEL_RESUME) {
				mutex_unlock(&goodix_modules.mutex);
				ts_info("Canceled by module:%s",
					ext_module->name);
				goto out;
			}
		}
	}
	mutex_unlock(&goodix_modules.mutex);

	goodix_ts_irq_enable(core_data, true);

	/*
	 * notify resume event, inform the esd protector
	 * and charger detector to turn on the work
	 */
	ts_info("try notify resume");
	goodix_ts_blocking_notify(NOTIFY_RESUME, NULL);
out:
	ts_debug("Resume end");
	return 0;
}

#ifdef CONFIG_FB
/**
 * goodix_ts_fb_notifier_callback - Framebuffer notifier callback
 * Called by kernel during framebuffer blanck/unblank phrase
 */
int goodix_ts_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct goodix_ts_core *core_data =
		container_of(self, struct goodix_ts_core, fb_notifier);
	struct fb_event *fb_event = data;

	if (fb_event && fb_event->data && core_data) {
		if (event == FB_EARLY_EVENT_BLANK) {
			/* before fb blank */
		} else if (event == FB_EVENT_BLANK) {
			int *blank = fb_event->data;
			if (*blank == FB_BLANK_UNBLANK)
				goodix_ts_resume(core_data);
			else if (*blank == FB_BLANK_POWERDOWN)
				goodix_ts_suspend(core_data);
		}
	}

	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
/**
 * goodix_ts_earlysuspend - Early suspend function
 * Called by kernel during system suspend phrase
 */
static void goodix_ts_earlysuspend(struct early_suspend *h)
{
	struct goodix_ts_core *core_data =
		container_of(h, struct goodix_ts_core,
			 early_suspend);

	goodix_ts_suspend(core_data);
}
/**
 * goodix_ts_lateresume - Late resume function
 * Called by kernel during system wakeup
 */
static void goodix_ts_lateresume(struct early_suspend *h)
{
	struct goodix_ts_core *core_data =
		container_of(h, struct goodix_ts_core,
			 early_suspend);

	goodix_ts_resume(core_data);
}
#endif

#ifdef CONFIG_PM
#if !defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND)
/**
 * goodix_ts_pm_suspend - PM suspend function
 * Called by kernel during system suspend phrase
 */
static int goodix_ts_pm_suspend(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	ts_info("goodix_ts_pm_suspend");
	return goodix_ts_suspend(core_data);
}
/**
 * goodix_ts_pm_resume - PM resume function
 * Called by kernel during system wakeup
 */
static int goodix_ts_pm_resume(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);
	ts_info("goodix_ts_pm_resume");
	return goodix_ts_resume(core_data);
}
#endif
#endif

/**
 * goodix_generic_noti_callback - generic notifier callback
 *  for goodix touch notification event.
 */
static int goodix_generic_noti_callback(struct notifier_block *self,
		unsigned long action, void *data)
{
	struct goodix_ts_core *ts_core = container_of(self,
			struct goodix_ts_core, ts_notifier);
	struct goodix_ts_device *ts_dev = ts_device(ts_core);
	const struct goodix_ts_hw_ops *hw_ops = ts_hw_ops(ts_core);
	int r;

	ts_info("notify event type 0x%x", (unsigned int)action);
	switch (action) {
	case NOTIFY_FWUPDATE_SUCCESS:
	case NOTIFY_FWUPDATE_FAILED:
		r = hw_ops->read_version(ts_dev, &ts_dev->chip_version);
		if (r < 0)
			ts_info("failed read fw version info[ignore]");
		break;
	default:
		break;
	}

	return 0;
}

int goodix_ts_stage2_init(struct goodix_ts_core *core_data)
{
	int r;
	struct goodix_ts_device *ts_dev = ts_device(core_data);

	/* send normal-cfg to firmware */
	r = ts_dev->hw_ops->send_config(ts_dev, &(ts_dev->normal_cfg));
	if (r < 0) {
		ts_info("failed send normal config[ignore]");
	}

	r = ts_dev->hw_ops->read_version(ts_dev, &ts_dev->chip_version);
	if (r < 0)
		ts_info("failed read fw version info[ignore]");

	/* alloc/config/register input device */
	r = goodix_ts_input_dev_config(core_data);
	if (r < 0) {
		ts_err("failed set input device");
		return r;
	}

	if (ts_dev->board_data.pen_enable) {
		r = goodix_ts_pen_dev_config(core_data);
		if (r < 0) {
			ts_err("failed set pen device");
			goto err_finger;
		}
	}
	/* request irq line */
	r = goodix_ts_irq_setup(core_data);
	if (r < 0) {
		ts_info("failed set irq");
		goto exit;
	}
	ts_info("success register irq");

#ifdef CONFIG_FB
	core_data->fb_notifier.notifier_call = goodix_ts_fb_notifier_callback;
	if (fb_register_client(&core_data->fb_notifier))
		ts_err("Failed to register fb notifier client:%d", r);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	core_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	core_data->early_suspend.resume = goodix_ts_lateresume;
	core_data->early_suspend.suspend = goodix_ts_earlysuspend;
	register_early_suspend(&core_data->early_suspend);
#endif
	/*create sysfs files*/
	goodix_ts_sysfs_init(core_data);

	/* esd protector */
	goodix_ts_esd_init(core_data);
	return 0;
exit:
	if (ts_dev->board_data.pen_enable) {
		goodix_ts_pen_dev_remove(core_data);
	}
err_finger:
	goodix_ts_input_dev_remove(core_data);
	return r;
}

/**
 * goodix_ts_probe - called by kernel when a Goodix touch
 *  platform driver is added.
 */
static int goodix_ts_probe(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data = NULL;
	struct goodix_ts_device *ts_device;
	int r;

	ts_info("goodix_ts_probe IN");

	ts_device = pdev->dev.platform_data;
	if (!ts_device || !ts_device->hw_ops) {
		ts_err("Invalid touch device");
		return -ENODEV;
	}

	core_data = devm_kzalloc(&pdev->dev, sizeof(struct goodix_ts_core),
				 GFP_KERNEL);
	if (!core_data) {
		ts_err("Failed to allocate memory for core data");
		return -ENOMEM;
	}

	/* touch core layer is a platform driver */
	core_data->pdev = pdev;
	core_data->ts_dev = ts_device;
	platform_set_drvdata(pdev, core_data);

	r = goodix_ts_power_init(core_data);
	if (r < 0)
		goto out;

	r = goodix_ts_power_on(core_data);
	if (r < 0)
		goto out;

#ifdef CONFIG_PINCTRL
	/* Pinctrl handle is optional. */
	r = goodix_ts_pinctrl_init(core_data);
	if (!r && core_data->pinctrl) {
		r = pinctrl_select_state(core_data->pinctrl,
					 core_data->pin_sta_active);
		if (r < 0)
			ts_err("Failed to select active pinstate, r:%d", r);
	}
#endif

	/* get GPIO resource */
	r = goodix_ts_gpio_setup(core_data);
	if (r < 0)
		goto out;

	/* confirm it's goodix touch dev or not */
	r = ts_device->hw_ops->dev_confirm(ts_device);
	if (r) {
		ts_err("goodix device confirm failed[skip]");
		goto out;
	}
	msleep(100);

	/* Try start a thread to get config-bin info */
	r = goodix_start_later_init(core_data);
	if (r) {
		ts_info("Failed start cfg_bin_proc");
		goto out;
	}

	/* generic notifier callback */
	core_data->ts_notifier.notifier_call = goodix_generic_noti_callback;
	goodix_ts_register_notifier(&core_data->ts_notifier);
out:
	if (r)
		core_data->initialized = 0;
	else
		core_data->initialized = 1;
	goodix_modules.core_data = core_data;

// ASUS_BSP +++ Touch
	print_touch_count_max = 100;
	core_data->station_insert = false;
	core_data->phone_call_on = false;
	core_data->aod_test_mode = 0;
	core_data->rotation = 0;
	core_data->dfps = 90;
	atomic_set(&core_data->charge_mode, 0);
	atomic_set(&core_data->testcfg, 0);
	atomic_set(&core_data->glove_mode, 0);
	core_data->game_mode = true;
	core_data->atr_enable = false;
	init_waitqueue_head(&core_data->fp_queue);
	if (asus_var_panel_stage[0]!='B'){
		fod_position = ts_9886_fod_position;
	}else{
		fod_position = ts_9896_fod_position;
	}
	ts_info("FOD %d %d %d %d", fod_position[0], fod_position[1], fod_position[2], fod_position[3]);
	ts_info("Touch mt max slots %d", GOODIX_ASUS_MAX_TOUCH);

	mutex_init(&core_data->gts_suspend_mutex);
	core_data->gts_suspend_resume_wq = create_singlethread_workqueue("goodix_suspend_resume_wq");
	if (NULL == core_data->gts_suspend_resume_wq) {
		ts_err("create suspend/resume workqueue failed");
		r=-1;
	}
	INIT_DELAYED_WORK(&core_data->gts_resume_work, goodix_resume_work);

	proc_create(GLOVE, 0666, NULL, &asus_proc_glove_ops);

	gts_core_data = core_data;

	r = sysfs_create_group(&core_data->pdev->dev.kobj, &sysfs_group);
	if (r) {
		ts_err("failed create core sysfs group");
	}
	
	atr_buf_queue = atr_buf_init(ATR_QUEUE_SIZE);
// ASUS_BSP --- Touch

	ts_info("goodix_ts_probe OUT, r:%d", r);
	/* wakeup ext module register work */
	complete_all(&goodix_modules.core_comp);
	return r;
}

static int goodix_ts_remove(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data = platform_get_drvdata(pdev);

	core_data->initialized = 0;
// ASUS_BSP +++ Touch
	//cancel_work_sync(&core_data->gts_resume_work);
	cancel_delayed_work_sync(&core_data->gts_resume_work);

	if (core_data->gts_suspend_resume_wq) {
		ts_info("goodix_ts_remove destroy_workqueue");
		destroy_workqueue(core_data->gts_suspend_resume_wq);
	}
// ASUS_BSP --- Touch
	if (atomic_read(&core_data->ts_esd.esd_on))
		goodix_ts_esd_off(core_data);
	goodix_remove_all_ext_modules();
	goodix_ts_power_off(core_data);
	goodix_debugfs_exit();
	goodix_ts_sysfs_exit(core_data);
	// can't free the memory for tools or gesture module
	//kfree(core_data);
	return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops dev_pm_ops = {
#if !defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = goodix_ts_pm_suspend,
	.resume = goodix_ts_pm_resume,
#endif
};
#endif

static const struct platform_device_id ts_core_ids[] = {
	{.name = GOODIX_CORE_DRIVER_NAME},
	{}
};
MODULE_DEVICE_TABLE(platform, ts_core_ids);

static struct platform_driver goodix_ts_driver = {
	.driver = {
		.name = GOODIX_CORE_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &dev_pm_ops,
#endif
	},
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = ts_core_ids,
};

int goodix_ts_core_init(void)
{
#ifdef ASUS_ZS661KS_PROJECT
	if(g_Recovery_mode) {
		ts_info("Core layer init: Set default lcd stage value for recovery");
		scnprintf(asus_var_panel_stage, sizeof(asus_var_panel_stage), "B3");
	}
#endif
	ts_info("Core layer init");
	if (!goodix_modules.initilized) {
		/* this may init by outer modules register event */
		ts_info("initilize goodix module");
		ts_info("init modules struct");
		goodix_modules.initilized = true;
		INIT_LIST_HEAD(&goodix_modules.head);
		mutex_init(&goodix_modules.mutex);
		init_completion(&goodix_modules.core_comp);
	}
	ts_info("goodix_debugfs_init");
	goodix_debugfs_init();
	return platform_driver_register(&goodix_ts_driver);
}

/* uninit module manually */
int goodix_ts_core_release(struct goodix_ts_core *core_data)
{
	ts_info("goodix core module removed");

	platform_driver_unregister(&goodix_ts_driver);
	goodix_ts_dev_release();
	return 0;
}
