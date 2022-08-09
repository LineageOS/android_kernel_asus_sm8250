/*
 * Goodix Gesture Module
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
 */
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include "goodix_ts_core.h"

#define GSX_GESTURE_CMD				0x08

#define QUERYBIT(longlong, bit) (!!(longlong[bit/8] & (1 << bit%8)))

#define GSX_MAX_KEY_DATA_LEN    64
#define GSX_KEY_DATA_LEN	37
#define GSX_KEY_DATA_LEN_YS	8
#define GSX_GESTURE_TYPE_LEN	32
// ASUS_BSP +++ Touch
#define ZENMOTION_LEN           8
#define GESTURE_TYPE             "driver/gesture_type"
#define DCLICK                   "driver/dclick"
#define SWIPEUP                  "driver/swipeup"

atomic_t aod_processing;
bool allow_report_zenmotion = true;
bool call_state = false;
int data_x = 0, data_y = 0, data_w = 200;

extern bool asus_var_regulator_always_on;
extern void asus_display_report_fod_touched(void);
extern struct goodix_ts_core *gts_core_data;
extern bool proximityStatus(void);
// ASUS_BSP --- Touch
/*
 * struct gesture_module - gesture module data
 * @registered: module register state
 * @sysfs_node_created: sysfs node state
 * @gesture_type: store valid gesture type,each bit stand for a gesture
 * @gesture_data: gesture data
 * @gesture_ts_cmd: gesture command data
 */
struct gesture_module {
	atomic_t registered;
	unsigned int kobj_initialized;
	rwlock_t rwlock;
	unsigned char gesture_type[GSX_GESTURE_TYPE_LEN];
	unsigned char gesture_data[GSX_MAX_KEY_DATA_LEN];
	struct goodix_ext_module module;
	struct goodix_ts_cmd cmd;
// ASUS_BSP +++ Touch
	unsigned int zenmotion_type[ZENMOTION_LEN];
	atomic_t zen_motion;
	atomic_t dclick;
	atomic_t swipeup;
	atomic_t aod_enable;
	atomic_t music_control;
	atomic_t fp_wakeup;
	atomic_t aod_ctrl_mode;
// ASUS_BSP --- Touch
};

static int gsx_enter_gesture_mode(struct goodix_ts_device *ts_dev);
static struct gesture_module *gsx_gesture; /*allocated in gesture init module*/

// ASUS_BSP +++ Touch
static int check_power(void);
static void zenmotion_setting(const char *buf, size_t count);
void enable_aod_processing(bool en);
bool get_aod_processing(void);
static void input_switch_key(struct input_dev *dev, unsigned int code);
// ASUS_BSP --- Touch
/**
 * gsx_gesture_type_show - show valid gesture type
 *
 * @module: pointer to goodix_ext_module struct
 * @buf: pointer to output buffer
 * Returns >=0 - succeed,< 0 - failed
 */
static ssize_t gsx_gesture_type_show(struct goodix_ext_module *module,
				char *buf)
{
	int count = 0, i, ret = 0;
	unsigned char *type;

	if (atomic_read(&gsx_gesture->registered) != 1) {
		ts_info("Gesture module not register!");
		return -EPERM;
	}
	type = kzalloc(256, GFP_KERNEL);
	if (!type)
		return -ENOMEM;
	read_lock(&gsx_gesture->rwlock);
	for (i = 0; i < 256; i++) {
		if (QUERYBIT(gsx_gesture->gesture_type, i)) {
			type[count] = i;
			count++;
		}
	}
	type[count] = '\0';
	if (count > 0)
		ret = scnprintf(buf, PAGE_SIZE, "%s", type);
	read_unlock(&gsx_gesture->rwlock);

	kfree(type);
	return ret;
}

/**
 * gsx_gesture_type_store - set vailed gesture
 *
 * @module: pointer to goodix_ext_module struct
 * @buf: pointer to valid gesture type
 * @count: length of buf
 * Returns >0 - valid gestures, < 0 - failed
 */
static ssize_t gsx_gesture_type_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	int i;

	if (count <= 0 || count > 256 || buf == NULL) {
		ts_err("Parameter error");
		return -EINVAL;
	}

	write_lock(&gsx_gesture->rwlock);
	memset(gsx_gesture->gesture_type, 0, GSX_GESTURE_TYPE_LEN);
	for (i = 0; i < count; i++)
		gsx_gesture->gesture_type[buf[i]/8] |= (0x1 << buf[i]%8);
	write_unlock(&gsx_gesture->rwlock);

	return count;
}

static ssize_t gsx_gesture_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 atomic_read(&gsx_gesture->registered));
}

static ssize_t gsx_gesture_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	unsigned int tmp;
	int ret;

	if (sscanf(buf, "%u", &tmp) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	ts_debug("Tmp value =%d", tmp);

	if (tmp == 1) {
		if (atomic_read(&gsx_gesture->registered)) {
			ts_debug("Gesture module has aready registered");
			return count;
		}
		ret = goodix_register_ext_module(&gsx_gesture->module);
		if (!ret) {
			ts_info("Gesture module registered!");
			atomic_set(&gsx_gesture->registered, 1);
		} else {
			atomic_set(&gsx_gesture->registered, 0);
			ts_err("Gesture module register failed");
		}
	} else if (tmp == 0) {
		if (!atomic_read(&gsx_gesture->registered)) {
			ts_debug("Gesture module has aready unregistered");
			return count;
		}
		ts_debug("Start unregistered gesture module");
		ret = goodix_unregister_ext_module(&gsx_gesture->module);
		if (!ret) {
			atomic_set(&gsx_gesture->registered, 0);
			ts_info("Gesture module unregistered success");
		} else {
			atomic_set(&gsx_gesture->registered, 1);
			ts_info("Gesture module unregistered failed");
		}
	} else {
		ts_err("Parameter error!");
		return -EINVAL;
	}
	return count;
}

static ssize_t gsx_gesture_data_show(struct goodix_ext_module *module,
				char *buf)
{
	ssize_t count;

	if (atomic_read(&gsx_gesture->registered) != 1) {
		ts_info("Gesture module not register!");
		return -EPERM;
	}
	if (!buf || (gsx_gesture->gesture_data == NULL)) {
		ts_info("Parameter error!");
		return -EPERM;
	}
	read_lock(&gsx_gesture->rwlock);

	count = scnprintf(buf, PAGE_SIZE, "Previous gesture type:0x%x\n",
			  gsx_gesture->gesture_data[2]);
	read_unlock(&gsx_gesture->rwlock);

	return count;
}
// ASUS_BSP +++ Touch

static ssize_t gsx_aod_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gsx_gesture->aod_enable));
}

static ssize_t gsx_aod_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	unsigned int tmp;

	if (sscanf(buf, "%u", &tmp) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	ts_debug("AOD enable =%d", tmp);

	if (tmp == 1) {
		atomic_set(&gsx_gesture->aod_enable, 1);
	} else {
		atomic_set(&gsx_gesture->aod_enable, 0);
		if(get_aod_processing()) {
			ts_info("[KEY_U] aod_enable = 0");
			enable_aod_processing(false);
		}
	}

	check_power();  

	return count;
}

static ssize_t gsx_fp_wakeup_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gsx_gesture->fp_wakeup));
}

static ssize_t gsx_fp_wakeup_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	unsigned int tmp;

	if (sscanf(buf, "%u", &tmp) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	ts_debug("fp wake enable =%d", tmp);

	if (tmp == 1) {
	  atomic_set(&gsx_gesture->fp_wakeup, 1);
	} else
	  atomic_set(&gsx_gesture->fp_wakeup, 0);

	check_power();  

	return count;
}

static ssize_t gsx_dclick_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gsx_gesture->dclick));
}

static ssize_t gsx_dclick_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
  	unsigned int tmp;

	if (sscanf(buf, "%u", &tmp) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	ts_debug("dclick =%d", tmp);

	if (tmp == 1) {
		atomic_set(&gsx_gesture->dclick, 1);
	} else
		atomic_set(&gsx_gesture->dclick, 0);

	check_power();

	return count;
  
}

static ssize_t gsx_zenmotion_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	int i, offset = 0;

	if (atomic_read(&gsx_gesture->registered) != 1) {
		ts_info("Gesture module not register!");
		return -EPERM;
	}

	write_lock(&gsx_gesture->rwlock);
	for (i = ZENMOTION_LEN - 1; i >= 0; i--) {
		offset += snprintf(&buf[offset], PAGE_SIZE - offset,
						"%c", gsx_gesture->zenmotion_type[i]);
	}
	buf[offset++] = '\n';
	write_unlock(&gsx_gesture->rwlock);

	return offset;
}

static ssize_t gsx_zenmotion_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	ts_info("%d", count);
	zenmotion_setting(buf, count);

	return count;
}

static ssize_t gsx_swipeup_enable_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gsx_gesture->swipeup));
}

static ssize_t gsx_swipeup_enable_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	unsigned int tmp;

	if (sscanf(buf, "%u", &tmp) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	ts_debug("swipeup =%d", tmp);

	if (tmp == 1)
		atomic_set(&gsx_gesture->swipeup, 1);
	else
		atomic_set(&gsx_gesture->swipeup, 0);

	check_power();

	return count;
}

static ssize_t gsx_fod_XY_data_show(struct goodix_ext_module *module,
		char *buf)
{
	ts_info("Gesture KEY_F X/Y data: %d,%d",data_x,data_y);
	return scnprintf(buf, PAGE_SIZE, "%d,%d\n", data_x, data_y);
}

static ssize_t fts_aod_ctrl_mode_show(struct goodix_ext_module *module,
		char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gsx_gesture->aod_ctrl_mode));
}

static ssize_t fts_aod_ctrl_mode_store(struct goodix_ext_module *module,
		const char *buf, size_t count)
{
	unsigned int tmp;

	if (sscanf(buf, "%u", &tmp) != 1) {
		ts_info("Parameter illegal");
		return -EINVAL;
	}
	ts_debug("aod_ctrl_mode =%d", tmp);

	if (tmp == 1)
		atomic_set(&gsx_gesture->aod_ctrl_mode, 1);
	else
		atomic_set(&gsx_gesture->aod_ctrl_mode, 0);

	check_power();

	return count;
}
// ASUS_BSP --- Touch

const struct goodix_ext_attribute gesture_attrs[] = {
	__EXTMOD_ATTR(type, 0666, gsx_gesture_type_show,
		gsx_gesture_type_store),
	__EXTMOD_ATTR(enable, 0666, gsx_gesture_enable_show,
		gsx_gesture_enable_store),
	__EXTMOD_ATTR(data, 0444, gsx_gesture_data_show, NULL),
// ASUS_BSP +++ Touch
	__EXTMOD_ATTR(zenmotion, 0666, gsx_zenmotion_enable_show,
		gsx_zenmotion_enable_store),
	__EXTMOD_ATTR(dclick, 0666, gsx_dclick_enable_show,
		gsx_dclick_enable_store),
	__EXTMOD_ATTR(aod_enable, 0666, gsx_aod_enable_show,
		gsx_aod_enable_store),
	__EXTMOD_ATTR(swipeup, 0666, gsx_swipeup_enable_show,
		gsx_swipeup_enable_store),
	__EXTMOD_ATTR(fp_wakeup, 0666, gsx_fp_wakeup_enable_show,
		gsx_fp_wakeup_enable_store),
	__EXTMOD_ATTR(XY, 0444, gsx_fod_XY_data_show, NULL),
	__EXTMOD_ATTR(fts_aod_ctrl_mode, 0666, fts_aod_ctrl_mode_show,
		fts_aod_ctrl_mode_store)
// ASUS_BSP --- Touch
};

static int gsx_enter_gesture_mode(struct goodix_ts_device *ts_dev)
{
	if (!gsx_gesture->cmd.initialized) {
		if (!ts_dev->reg.command) {
			ts_err("command reg can not be null");
			return -EINVAL;
		}
		if (ts_dev->ic_type == IC_TYPE_YELLOWSTONE) {
			gsx_gesture->cmd.cmd_reg = ts_dev->reg.command;
			gsx_gesture->cmd.length = 5;
			gsx_gesture->cmd.cmds[0] = GSX_GESTURE_CMD;
			gsx_gesture->cmd.cmds[1] = 0x0;
			gsx_gesture->cmd.cmds[2] = 0x0;
			gsx_gesture->cmd.cmds[3] = 0x0;
			gsx_gesture->cmd.cmds[4] = GSX_GESTURE_CMD;
			gsx_gesture->cmd.initialized = 1;

		} else {
			gsx_gesture->cmd.cmd_reg = ts_dev->reg.command;
			gsx_gesture->cmd.length = 3;
			gsx_gesture->cmd.cmds[0] = GSX_GESTURE_CMD;
			gsx_gesture->cmd.cmds[1] = 0x0;
			gsx_gesture->cmd.cmds[2] = 0 - GSX_GESTURE_CMD;
			gsx_gesture->cmd.initialized = 1;
		}
	}

	return ts_dev->hw_ops->send_cmd(ts_dev, &gsx_gesture->cmd);
}

// ASUS_BSP +++ Touch
static ssize_t asus_gesture_proc_type_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[ZENMOTION_LEN];
	memset(messages, 0, sizeof(messages));

	if (len > ZENMOTION_LEN)
		len = ZENMOTION_LEN;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	ts_info("%d", len);
	zenmotion_setting(messages, len);

	return len;
}

static ssize_t asus_gesture_proc_type_read(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	ssize_t ret = 0;
	char *buff = NULL;
	int i, offset = 0;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	if (atomic_read(&gsx_gesture->registered) != 1) {
		ts_info("Gesture module not register!");
		return -EPERM;
	}

	write_lock(&gsx_gesture->rwlock);
	for (i = ZENMOTION_LEN - 1; i >= 0; i--) {
		offset += snprintf(&buff[offset], PAGE_SIZE - offset,
						"%c", gsx_gesture->zenmotion_type[i]);
	}
	buff[offset++] = '\n';
	write_unlock(&gsx_gesture->rwlock);

	ret = simple_read_from_buffer(buf, count, ppos, buff, offset);
	kfree(buff);

	return ret;
}

static ssize_t asus_gesture_proc_dclick_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
		atomic_set(&gsx_gesture->dclick, 0);
	} else {
		atomic_set(&gsx_gesture->dclick, 1);
	}
	
	check_power();

	return len;
}

static ssize_t asus_gesture_proc_dclick_read(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%d\n", atomic_read(&gsx_gesture->dclick));
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static ssize_t asus_gesture_proc_swipeup_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	memset(messages, 0, sizeof(messages));

	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "0", 1) == 0) {
		atomic_set(&gsx_gesture->swipeup, 0);
	} else {
		atomic_set(&gsx_gesture->swipeup, 1);
	}
	
	check_power();

	return len;
}

static ssize_t asus_gesture_proc_swipeup_read(struct file *file, char __user *buf,
							 size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff = NULL;

	buff = kzalloc(100, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	len += sprintf(buff, "%d\n", atomic_read(&gsx_gesture->swipeup));
	ret = simple_read_from_buffer(buf, count, ppos, buff, len);
	kfree(buff);

	return ret;
}

static struct file_operations asus_gesture_proc_type_ops = {
	.write = asus_gesture_proc_type_write,
	.read  = asus_gesture_proc_type_read,
};

static struct file_operations asus_gesture_proc_dclick_ops = {
	.write = asus_gesture_proc_dclick_write,
	.read  = asus_gesture_proc_dclick_read,
};

static struct file_operations asus_gesture_proc_swipeup_ops = {
	.write = asus_gesture_proc_swipeup_write,
	.read  = asus_gesture_proc_swipeup_read,
};
// ASUS_BSP --- Touch

static int gsx_gesture_init(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module)
{
	int i, ret = -EINVAL;
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	if (!core_data || !ts_dev->hw_ops->write || !ts_dev->hw_ops->read) {
		ts_err("Register gesture module failed, ts_core unsupported");
		goto exit_gesture_init;
	}

	memset(gsx_gesture->gesture_type, 0, GSX_GESTURE_TYPE_LEN);
	memset(gsx_gesture->gesture_data, 0xff,
	       sizeof(gsx_gesture->gesture_data));

	ts_debug("Set gesture type manually");
	/* set all bit to 1 to enable all gesture wakeup */
	memset(gsx_gesture->gesture_type, 0xff, GSX_GESTURE_TYPE_LEN);

	if (gsx_gesture->kobj_initialized) {
		ret = 0;
		goto exit_gesture_init;
	}

	ret = kobject_init_and_add(&module->kobj, goodix_get_default_ktype(),
			&core_data->pdev->dev.kobj, "gesture");

	if (ret) {
		ts_err("Create gesture sysfs node error!");
		goto exit_gesture_init;
	}

	ret = 0;
	for (i = 0; i < ARRAY_SIZE(gesture_attrs) && !ret; i++)
		ret = sysfs_create_file(&module->kobj, &gesture_attrs[i].attr);
	if (ret) {
		ts_err("failed create gst sysfs files");
		while (--i >= 0)
			sysfs_remove_file(&module->kobj, &gesture_attrs[i].attr);

		kobject_put(&module->kobj);
		goto exit_gesture_init;
	}

	gsx_gesture->kobj_initialized = 1;
// ASUS_BSP +++ Touch
	atomic_set(&gsx_gesture->dclick, 0);
	atomic_set(&gsx_gesture->swipeup, 0);
	atomic_set(&gsx_gesture->aod_enable, 0);
	atomic_set(&gsx_gesture->zen_motion, 0);
	atomic_set(&gsx_gesture->fp_wakeup, 0);
	atomic_set(&gsx_gesture->music_control, 0);
	atomic_set(&gsx_gesture->aod_ctrl_mode, 0);
	check_power();
	
	proc_create(GESTURE_TYPE, 0666, NULL, &asus_gesture_proc_type_ops);
	proc_create(DCLICK, 0666, NULL, &asus_gesture_proc_dclick_ops);
	proc_create(SWIPEUP, 0666, NULL, &asus_gesture_proc_swipeup_ops);
// ASUS_BSP --- Touch

exit_gesture_init:
	return ret;
}

static int gsx_gesture_exit(struct goodix_ts_core *core_data,
		struct goodix_ext_module *module)
{
	atomic_set(&gsx_gesture->registered, 0);
// ASUS_BSP +++ Touch
	atomic_set(&gsx_gesture->dclick, 0);
	atomic_set(&gsx_gesture->swipeup, 0);
	atomic_set(&gsx_gesture->aod_enable, 0);
	atomic_set(&gsx_gesture->zen_motion, 0);
	atomic_set(&gsx_gesture->fp_wakeup, 0);
	atomic_set(&gsx_gesture->music_control, 0);
	atomic_set(&gsx_gesture->aod_ctrl_mode, 0);
	check_power();
// ASUS_BSP --- Touch
	return 0;
}

// ASUS_BSP +++ Touch
static int check_power(void)
{
	bool enable_power = false;
	
	if ((atomic_read(&gsx_gesture->zen_motion) == 0) &&
		(atomic_read(&gsx_gesture->dclick) == 0) &&
		(atomic_read(&gsx_gesture->swipeup) == 0) &&
		(atomic_read(&gsx_gesture->aod_enable) == 0) &&
		(atomic_read(&gsx_gesture->fp_wakeup) == 0) &&
		(atomic_read(&gsx_gesture->aod_ctrl_mode) == 0)) {
		enable_power = false;
	} else {
		enable_power = true;
	}
	
	if (asus_var_regulator_always_on != enable_power) {
		asus_var_regulator_always_on = enable_power;
		ts_info("Enable power %d", asus_var_regulator_always_on);
	}

	return 0;
}

static void zenmotion_setting(const char *buf, size_t count)
{
	char gesture_buf[ZENMOTION_LEN];
	char input_temp_buf[ZENMOTION_LEN];
	char cmpchar = '1';
	int tmp = 0, i = 0;
	
	// W : 11
	// all : 1111111
	ts_info("zen number %d", count);
	memset(gesture_buf, 0, sizeof(gesture_buf));
	memset(input_temp_buf, 0, sizeof(gesture_buf));
	sprintf(input_temp_buf, "%s", buf);
	ts_info("buf %s ",buf);

	for (tmp = ZENMOTION_LEN - count, i = 0;tmp < ZENMOTION_LEN ;tmp ++, i++) {
		gesture_buf[tmp] = input_temp_buf[i];
	}

	memset(gsx_gesture->zenmotion_type, 0, ZENMOTION_LEN * sizeof(unsigned int));

	if (gesture_buf[7] == cmpchar) {
		ts_info("ZenMotion enable");
		ts_info("Str L~R : music, v, z, m, e, s, w, enable");
		ts_info("Buf 0~7 : enable, w, s, e, m, z, v, music");
		if (atomic_read(&gsx_gesture->registered)) {
			atomic_set(&gsx_gesture->zen_motion, 1);
			for (tmp = 0;tmp < ZENMOTION_LEN ;tmp ++) {
				gsx_gesture->zenmotion_type[tmp] = gesture_buf[(ZENMOTION_LEN -1 )- tmp];
			}
			if(gesture_buf[0] == cmpchar)
				atomic_set(&gsx_gesture->music_control, 1);
			else
				atomic_set(&gsx_gesture->music_control, 0);
			ts_info("gsx_gesture->zenmotion_type %s ",gsx_gesture->zenmotion_type);
			ts_info("gsx_gesture->music_control %s ",gsx_gesture->music_control);
		} else {
			ts_info("gesture module not registered");
			atomic_set(&gsx_gesture->zen_motion, 0);
			atomic_set(&gsx_gesture->music_control, 0);
			memset(gsx_gesture->zenmotion_type, 0, ZENMOTION_LEN * sizeof(unsigned int));
		}
	} else {
		ts_info("ZenMotion disable");
		atomic_set(&gsx_gesture->zen_motion, 0);
		atomic_set(&gsx_gesture->music_control, 0);
		memset(gsx_gesture->zenmotion_type, 0, ZENMOTION_LEN * sizeof(unsigned int));
	}

	check_power();
}

static void input_switch_key(struct input_dev *dev, unsigned int code)
{
	input_report_key(dev, code, 1);
	input_sync(dev);
	input_report_key(dev, code, 0);
	input_sync(dev);
	
	//if(code != 116) // Power key
	ts_info("keycode = %d\n", code);
}

static int report_gesture_key(struct input_dev *dev, char keycode)
{
	if(call_state){
		if (proximityStatus() == true){
			ts_info("in call state , p sensor enable, ignore any gesture event");
			return 2;
		}
	}

	if(atomic_read(&gsx_gesture->aod_enable)==1) {
		if(keycode == 'F') {
			input_switch_key(dev, KEY_F);
			ts_info("KEY_F");
#if 0
			input_report_key(dev, BTN_TOUCH, 1);
			input_mt_slot(dev, 0);
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);
			input_report_abs(dev, ABS_MT_POSITION_X, data_x);
			input_report_abs(dev, ABS_MT_POSITION_Y, data_y);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR,data_w);
			input_report_abs(dev, ABS_MT_PRESSURE,data_w);
			input_sync(dev);
			ts_info("Gesture KEY_F -- X/Y/W data: %d,%d,%d",data_x,data_y,data_w);
#endif
			asus_display_report_fod_touched();
			enable_aod_processing(true);
#if 0
			input_mt_slot(dev, 0);
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR, 0);
			input_report_abs(dev, ABS_MT_PRESSURE, 0);
			input_report_key(dev, BTN_TOUCH, 0);
			input_sync(dev);
			ts_info("Gesture KEY_F --- release X/Y/W data: %d,%d,%d",data_x,data_y,data_w);
#endif
			return 3;
		}
		if(keycode == 'U') {
			ts_info("[KEY_U] keycode = U");
			enable_aod_processing(false);
			return 3;
		}
		if(keycode == 'L') {
			input_switch_key(dev, KEY_L);
			ts_info("KEY_L");
			return 3;
		}
		if(keycode == 0x4f) {
			input_switch_key(dev, KEY_O);
			ts_info("KEY_O");
			return 3;
		}
	} else if (atomic_read(&gsx_gesture->aod_ctrl_mode)==1) {
		if((keycode == 'F') || (keycode == 'L') || (keycode == 0x4f)) {
			input_switch_key(dev, KEY_L);
			ts_info("KEY_L");
			return 3;
		}
	}

	if (proximityStatus() == true) {
		ts_info("P-sensor near, disable gesture mode");
		return 2;
	}
	
	if (!allow_report_zenmotion) {
		return 2;
	}

	switch (keycode) {
	case 'w': // w
		if (gsx_gesture->zenmotion_type[1] == '1') {
			input_switch_key(dev, KEY_W);
			ts_info("KEY_W");
			return 1;
		}
		break;
	case 's': // S
		if(gsx_gesture->zenmotion_type[2] == '1') {
			input_switch_key(dev, KEY_S);
			ts_info("KEY_S");
			return 1;
		}
		break;
	case 'e': // e
		if(gsx_gesture->zenmotion_type[3] == '1'){
			input_switch_key(dev, KEY_E);
			ts_info("KEY_E");
			return 1;
		}
		break;
	case 'm': // M 
		if(gsx_gesture->zenmotion_type[4] == '1') {
			input_switch_key(dev, KEY_M);
			ts_info("KEY_M");
			return 1;
		}
		break;
	case 'z': // Z
		if(gsx_gesture->zenmotion_type[5] == '1'){
			input_switch_key(dev, KEY_Z);
			ts_info("KEY_Z");
			return 1;
		}
		break;
	case 'v': // V
		if(gsx_gesture->zenmotion_type[6] == '1') {
			input_switch_key(dev, KEY_V);
			ts_info("KEY_V");
			return 1;
		}
		break;
	case 0xcc: // double click
		if (atomic_read(&gsx_gesture->dclick)==1) {
			input_switch_key(dev, KEY_POWER);
			return 1;
		}
		break;
	case 0xba:
		if (atomic_read(&gsx_gesture->swipeup)==1) {
			input_switch_key(dev, KEY_UP);
			return 1;
		}
		break;
	case 0x48: // ||
		if(atomic_read(&gsx_gesture->music_control)==1) {
			input_switch_key(dev, KEY_PAUSE);
			ts_info("music_control : KEY_PAUSE");
			return 1;
		}
		break;
	case 0x63: // <
		if(atomic_read(&gsx_gesture->music_control)==1) {
			input_switch_key(dev, KEY_REWIND);
			ts_info("music_control : KEY_REWIND");
			return 1;
		}
		break;
	case 0x3e: // >
		if(atomic_read(&gsx_gesture->music_control)==1) {
			input_switch_key(dev, KEY_FORWARD);
			ts_info("music_control : KEY_FORWARD");
			return 1;
		}
		break;
	default:
		break;
	}
	return 2;
}

void enable_aod_processing(bool en)
{
	struct input_dev *input_dev = gts_core_data->input_dev;
	
	if((en == true) && (atomic_read(&aod_processing) != 1)) {
		atomic_set(&aod_processing, 1);
	} else if ((en == false) && (atomic_read(&aod_processing) != 0)) {
		atomic_set(&aod_processing, 0);
		input_switch_key(input_dev, KEY_U);
		ts_info("KEY_U");
	}
	ts_info("[G] aod_processing %d", atomic_read(&aod_processing));
}
EXPORT_SYMBOL_GPL(enable_aod_processing);

bool get_aod_processing(void)
{
	if(atomic_read(&aod_processing))
		return true;
	else
		return false;
}
EXPORT_SYMBOL_GPL(get_aod_processing);
// ASUS_BSP --- Touch

/**
 * gsx_gesture_ist - Gesture Irq handle
 * This functions is excuted when interrupt happended and
 * ic in doze mode.
 *
 * @core_data: pointer to touch core data
 * @module: pointer to goodix_ext_module struct
 * return: 0 goon execute, EVT_CANCEL_IRQEVT  stop execute
 */
static int gsx_gesture_ist(struct goodix_ts_core *core_data,
	struct goodix_ext_module *module)
{
	int ret, r;
	int key_data_len = 0;
	u8 clear_reg = 0, checksum = 0, gsx_type = 0;
	u8 temp_data[GSX_MAX_KEY_DATA_LEN];
	struct goodix_ts_device *ts_dev = core_data->ts_dev;

	if (atomic_read(&core_data->suspended) == 0)
		return EVT_CONTINUE;

	if (!ts_dev->reg.gesture) {
		ts_err("gesture reg can't be null");
		return EVT_CONTINUE;
	}
	ts_err("gesture go ++");
	/* get ic gesture state*/
	if (ts_dev->ic_type == IC_TYPE_YELLOWSTONE)
		key_data_len = GSX_KEY_DATA_LEN_YS;
	else
		key_data_len = GSX_KEY_DATA_LEN;
	ret = ts_dev->hw_ops->read_trans(ts_dev, ts_dev->reg.gesture,
					 temp_data, key_data_len);
	if (ret < 0 || ((temp_data[0] & GOODIX_GESTURE_EVENT)  == 0)) {
		ts_debug("invalid gesture event, ret=%d, temp_data[0]=0x%x",
			 ret, temp_data[0]);
		goto re_send_ges_cmd;
	}
	if (ts_dev->ic_type == IC_TYPE_YELLOWSTONE) // 9896
		checksum = checksum_u8_ys(temp_data, key_data_len);
	else // 9886
		checksum = checksum_u8(temp_data, key_data_len);
	if (checksum) {
		ts_info("Gesture data length %d", key_data_len);
		ts_err("Gesture data checksum error:0x%x", checksum);
		ts_info("Gesture data %*ph",
			(int)sizeof(temp_data), temp_data);
		goto re_send_ges_cmd;
	}

#if 0
	ts_info("Gesture data:");
	ts_info("data[0-5]0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x", temp_data[0], temp_data[1],
		 temp_data[2], temp_data[3], temp_data[4], temp_data[5]);
	ts_info("data[6-12]0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x", temp_data[6], temp_data[7],
		 temp_data[8], temp_data[9], temp_data[10], temp_data[11]);
#endif

	write_lock(&gsx_gesture->rwlock);
	memcpy(gsx_gesture->gesture_data, temp_data, key_data_len);
	write_unlock(&gsx_gesture->rwlock);

	if (ts_dev->ic_type == IC_TYPE_YELLOWSTONE)
		gsx_type = temp_data[4];
	else
		gsx_type = temp_data[2];
		
	if (core_data->station_insert){
		ts_info("inserted into station , not allow zenmotion functions");
		allow_report_zenmotion = false;
	} else {
		allow_report_zenmotion = true;
	}

	if (core_data->phone_call_on)
		call_state = true;
	else
		call_state = false;

	if (QUERYBIT(gsx_gesture->gesture_type, gsx_type)) {
		/* do resume routine */
		if((atomic_read(&gsx_gesture->aod_enable)==1) || (atomic_read(&gsx_gesture->aod_ctrl_mode)==1)) {
			if (temp_data[4] == 0x46){
				//ts_info("Get KEY_F X and Y");
				ret = ts_dev->hw_ops->read_trans(ts_dev, ts_dev->reg.gesture + key_data_len, &temp_data[key_data_len], 34);  
				if (ret < 0) {
					ts_debug("invalid get more gesture data, ret=%d", ret);
					goto re_send_ges_cmd;
				}

				if (ts_dev->ic_type == IC_TYPE_YELLOWSTONE)
					checksum = checksum_u8_ys(&temp_data[key_data_len], 34);
				else
					checksum = checksum_u8(&temp_data[key_data_len], 34);
				if (checksum) {
					ts_err("get more Gesture data checksum error:0x%x", checksum);
					goto re_send_ges_cmd;
				}
				data_x = temp_data[8] + (temp_data[9] * 256) ;
				data_y = temp_data[10] + (temp_data[11]* 256);
				//data_w = temp_data[12] + (temp_data[13]* 256);
				//ts_info("Gesture X/Y/W data : 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x", temp_data[8], temp_data[9], temp_data[10], temp_data[11], temp_data[12], temp_data[13]);
				ts_info("Gesture KEY_F ++ X/Y/W data: %d,%d,%d",data_x,data_y,data_w);
			}
		}
// ASUS_BSP +++ Touch
		ts_info("key event : %x", gsx_type);
		r = report_gesture_key(core_data->input_dev,gsx_type);
		if (r == 1) { 
			goto gesture_ist_exit; // irq handled
		} else if (r == 2) {
			goto gesture_ist_exit_without_wakeup;
		} else if (r == 3) {
			goto FOD_exit;
		}
// ASUS_BSP --- Touch

		goto gesture_ist_exit;
	} else {
		ts_info("Unsupported gesture:%x", gsx_type);
	}

re_send_ges_cmd:
	if (gsx_enter_gesture_mode(core_data->ts_dev))
		ts_info("warning: failed re_send gesture cmd\n");
gesture_ist_exit:
	ts_dev->hw_ops->write_trans(ts_dev, ts_dev->reg.gesture,
				    &clear_reg, 1);
	return EVT_CANCEL_IRQEVT;
gesture_ist_exit_without_wakeup:
	ts_info("Do not wakeup system");
	ts_dev->hw_ops->write_trans(ts_dev, ts_dev->reg.gesture,
			      &clear_reg, 1);
	r = gsx_enter_gesture_mode(ts_dev);
	if (r != 0) {
		ts_err("gesture_ist_exit_without_wakeup - failed enter gesture mode");
	}
	enable_irq_wake(core_data->irq);
	return EVT_CANCEL_RESUME; 
FOD_exit:
	ts_dev->hw_ops->write_trans(ts_dev, ts_dev->reg.gesture,
				    &clear_reg, 1);
	enable_irq_wake(core_data->irq);
	return EVT_CANCEL_RESUME;
}

/**
 * gsx_gesture_before_suspend - execute gesture suspend routine
 * This functions is excuted to set ic into doze mode
 *
 * @core_data: pointer to touch core data
 * @module: pointer to goodix_ext_module struct
 * return: 0 goon execute, EVT_IRQCANCLED  stop execute
 */
static int gsx_gesture_before_suspend(struct goodix_ts_core *core_data,
	struct goodix_ext_module *module)
{
	int ret;
	const struct goodix_ts_hw_ops *hw_ops = core_data->ts_dev->hw_ops;
	struct goodix_ts_cmd *gesture_cmd = &gsx_gesture->cmd;

// ASUS_BSP +++ Touch
	if (hw_ops == NULL) {
		ts_err("Uninitialized hw_ops");
		return 0;
	}	

	if ((atomic_read(&gsx_gesture->zen_motion) == 0) &&
		(atomic_read(&gsx_gesture->dclick) == 0) &&
		(atomic_read(&gsx_gesture->swipeup) == 0) &&
		(atomic_read(&gsx_gesture->aod_enable) == 0) &&
		(atomic_read(&gsx_gesture->fp_wakeup) == 0) &&
		(atomic_read(&gsx_gesture->aod_ctrl_mode) == 0)) {
		ts_info("Gesture not enable, going to deep sleep mode");
		return 0;
	}

	if (!gesture_cmd->initialized) {
		ts_err("Uninitialized doze command");
	}
// ASUS_BSP --- Touch

	ret = gsx_enter_gesture_mode(core_data->ts_dev);
	if (ret != 0) {
		ts_err("failed enter gesture mode");
		return 0;
	}
	ts_info("Set IC in gesture mode");
	atomic_set(&core_data->suspended, 1);
	atomic_set(&aod_processing, 0);
	enable_irq_wake(core_data->irq);
	return EVT_CANCEL_SUSPEND;
}

static struct goodix_ext_module_funcs gsx_gesture_funcs = {
	.irq_event = gsx_gesture_ist,
	.init = gsx_gesture_init,
	.exit = gsx_gesture_exit,
	.before_suspend = gsx_gesture_before_suspend
};

static int __init goodix_gsx_gesture_init(void)
{
	/* initialize core_data->ts_dev->gesture_cmd */
	int result;
	ts_info("gesture module init");
	gsx_gesture = kzalloc(sizeof(struct gesture_module), GFP_KERNEL);
	if (!gsx_gesture)
		result = -ENOMEM;
	gsx_gesture->module.funcs = &gsx_gesture_funcs;
	gsx_gesture->module.priority = EXTMOD_PRIO_GESTURE;
	gsx_gesture->module.name = "Goodix_gsx_gesture";
	gsx_gesture->module.priv_data = gsx_gesture;
	gsx_gesture->kobj_initialized = 0;
	atomic_set(&gsx_gesture->registered, 0);
	rwlock_init(&gsx_gesture->rwlock);
	ts_info("gesture module init_register_ext_module +++");
	result = goodix_register_ext_module(&(gsx_gesture->module));
	ts_info("gesture module init_register_ext_module ---");
	if (result == 0) {
		atomic_set(&gsx_gesture->registered, 1);
		ts_info("gesture module successful");
	}
	return result;
}

static void __exit goodix_gsx_gesture_exit(void)
{
	int i, ret;
	ts_info("gesture module exit");
	if (atomic_read(&gsx_gesture->registered)) {
		ret = goodix_unregister_ext_module(&gsx_gesture->module);
		atomic_set(&gsx_gesture->registered, 0);
	}
	if (gsx_gesture->kobj_initialized) {
		for (i = 0; i < ARRAY_SIZE(gesture_attrs); i++)
			sysfs_remove_file(&gsx_gesture->module.kobj,
					  &gesture_attrs[i].attr);

		kobject_put(&gsx_gesture->module.kobj);
	}

	kfree(gsx_gesture);
}

module_init(goodix_gsx_gesture_init);
module_exit(goodix_gsx_gesture_exit);

MODULE_DESCRIPTION("Goodix gsx Touchscreen Gesture Module");
MODULE_AUTHOR("Goodix, Inc.");
MODULE_LICENSE("GPL v2");
