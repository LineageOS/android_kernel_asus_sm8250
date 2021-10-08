/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 * Edit by ASUS Deeo, deeo_ho@asus.com
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include "ene_8k41.h"

static struct ene_8k41_platform_data *g_pdata;

static int i2c_read_bytes(struct i2c_client *client, char *write_buf, int writelen, char *read_buf, int readlen)
{
	struct i2c_msg msgs[2];
	int ret=-1;

	//send register address
	msgs[0].flags = !I2C_M_RD;	//write
	msgs[0].addr = client->addr;
	msgs[0].len = writelen;
	msgs[0].buf = write_buf;
	
	//read data
	msgs[1].flags = I2C_M_RD;		//read
	msgs[1].addr = client->addr;
	msgs[1].len = readlen;
	msgs[1].buf = read_buf;

	ret = i2c_transfer(client->adapter,msgs, 2);
	return ret;
}

static int i2c_write_bytes(struct i2c_client *client, char *write_buf, int writelen)
{
	struct i2c_msg msg;
	int ret=-1;
	
	msg.flags = !I2C_M_RD;		//write
	msg.addr = client->addr;
	msg.len = writelen;
	msg.buf = write_buf; 

	ret = i2c_transfer(client->adapter,&msg, 1);
	return ret;
} 

static int ene_8k41_read_bytes(struct i2c_client *client, short addr, char *data)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = 0x00;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;

//	printk("[ENE_DT] ene_8k41_read_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2]);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[ENE_DT] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x81;
	err = i2c_read_bytes(client, buf, 1, data, 1);	//send read command
	if (err != 2)
		printk("[ENE_DT] i2c_read_bytes:err %d\n", err);

	return err;
}

static int ene_8k41_write_bytes(struct i2c_client *client, short addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = 0x00;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;

//	printk("[ENE_DT] ene_8k41_write_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x, value : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2], value);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[ENE_DT] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x01;
	buf[1] = value;
	
	err = i2c_write_bytes(client, buf, 2);	//set register address
	if (err !=1)
		printk("[ENE_DT] i2c_write_bytes:err %d\n", err);
	
	return err;
}

static int ene_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	printk("[ENE_DT] ene_GetFirmwareSize.\n");
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = file_inode(pfile);
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ene_ReadFirmware(char *fw_name, unsigned char *fw_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", fw_name);
	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = file_inode(pfile);
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, fw_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

static char ene_CheckFirmwareVer(struct i2c_client *client)
{
	int err = 0;
	unsigned char data[2] = {0};

	err = ene_8k41_read_bytes(client, 0x80C0, data);
	if (err != 2){
		printk("[ENE_DT] fw_check:err %d\n", err);
		data[0] = 0xFF;
	}

	return (u8) data[0];
}

static int ene_UpdateFirmware(struct i2c_client *client, char *fw_buf)
{
	int err = 0;
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[2] = {0};
	//unsigned char buf[129] = {0};
	unsigned char *buf;
	//unsigned char tmp[129] = {0};
	//int  i = 0;
	short addr;
	int retry = 0;
	buf = kmalloc(sizeof(unsigned char)*129, GFP_DMA); //for i2c transfer --need to free
	err = ene_8k41_read_bytes(client, 0xFF8F, data);
	if (err != 2)
		printk("[ENE_DT] Check REG[0xFF8F]:err %d\n", err);

	printk("[ENE_DT] REG[0xFF8F] : 0x%x\n", data[0]);

	if( (u8)(data[0]) <= 0x40){
		platform_data->fw_version = ene_CheckFirmwareVer(client);
		printk("[ENE_DT] FW VER : 0x%x\n", platform_data->fw_version);
		//if ( platform_data->fw_version == 0x0) { // rember change condition.
		if (0) {
			printk("[ENE_DT] Don't need FW update.\n");
			kfree(buf);
			return 0;
		}
	} else
		printk("[ASUS_SYNC] FW is abnormal !!!");

	printk("[ENE_DT] Start ene_UpdateFirmware\n");

	err = ene_8k41_write_bytes(client, 0xF018, 0xCE);
	if (err !=1)
		printk("[ENE_DT] REG[F010]: write err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF018, data);
	if (err != 2)
		printk("[ENE_DT] REG[0xF018]: read err %d\n", err);

	printk("[ENE_DT] REG[0xF018]: 0x%x\n", data[0]);

// (1) Reset 8051
	printk("[ENE_DT] Reset 8051 =====\n");
	err = ene_8k41_write_bytes(client, 0xF010, 0x5);
	if (err !=1)
		printk("[ENE_DT] REG[0xF010]: write err %d\n", err);

	msleep(500);

// (2) Read & Fill OSC32M freq.
	printk("[ENE_DT] Read & Fill OSC32M freq =====\n");
	err = ene_8k41_write_bytes(client, 0xF808, 0xF0);
	if (err !=1)
		printk("[ENE_DT] REG[0xF808]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF809, 0x01);
	if (err !=1)
		printk("[ENE_DT] REG[0xF809]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF807, 0x90);
	if (err !=1)
		printk("[ENE_DT] REG[0xF807]: write err %d\n", err);

	err = ene_8k41_read_bytes(client, 0xF80B, data);
	if (err != 2)
		printk("[ENE_DT] REG[0xF80B]: read err %d\n", err);

	printk("[ENE_DT] REG[0xF80B]: 0x%x\n", data[0]);
	err = ene_8k41_write_bytes(client, 0xF806, data[0]);
	if (err != 1)
		printk("[ENE_DT] REG[0xF806]: write err %d\n", err);

// (3) Fill program & Erase timing
	printk("[ENE_DT] Fill program & Erase timing =====\n");
	err = ene_8k41_write_bytes(client, 0xF815, 0x10);
	if (err !=1)
		printk("[ENE_DT] REG[0xF815]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF816, 0x11);
	if (err !=1)
		printk("[ENE_DT] REG[0xF816]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF817, 0x06);
	if (err !=1)
		printk("[ENE_DT] REG[0xF817]: write err %d\n", err);
	err = ene_8k41_write_bytes(client, 0xF818, 0x07);
	if (err !=1)
		printk("[ENE_DT] REG[0xF818]: write err %d\n", err);

// (4) Erase
	printk("[ENE_DT] Erase page from 0x0000 to 0x3F80 =====\n");
	for (addr = 0x0000 ; addr <= 0x3F80 ; addr+= 0x80) {

		printk("[ENE_DT] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[ENE_DT] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[ENE_DT] REG[0xF809]: write err %d\n", err);

		err = ene_8k41_write_bytes(client, 0xF807, 0x20);	// erase page cmd
		if (err !=1)
			printk("[ENE_DT] REG[0xF807]: write err %d\n", err);

		while(1){	 // check cmd finished. 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[ENE_DT] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[ENE_DT] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry=0;
				break;
			}
			printk("[ENE_DT] check REG[0xF800], 0x%02x\n", data[0]);
			retry++;
			if (retry > 10) {
				printk("[ENE_DT] retry too many times: %d, Force exit!!!\n", retry);
				kfree(buf);
				return -1;
			}
		}
	}
	printk("[ENE_DT] erase page finished. =====\n");
	
// (5) Program
	printk("[ENE_DT] Program FW from 0x0000 to 0x3F80 =====\n");
	for (addr = 0x0000 ; addr <= 0x3F80 ; addr = addr + 0x80) {

		printk("[ENE_DT] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[ENE_DT] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[ENE_DT] REG[0xF809]: write err %d\n", err);

		err = ene_8k41_write_bytes(client, 0xF807, 0x80);	// 3. Clear HVPL data
		if (err !=1)
			printk("[ENE_DT] REG[0xF807]: write err %d\n", err);

		while(1){	 // 4. check cmd finished 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[ENE_DT] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[ENE_DT] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[ENE_DT] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[ENE_DT] retry too many times: %d, Force exit!!!\n", retry);
				kfree(buf);
				return -1;
			}
		}

		// 5. Set address to 0xF80A
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0A;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[ENE_DT] i2c_write_bytes:err %d\n", err);

		// 6. Send data to buffer
		buf[0] = 0x05;
		memcpy( &(buf[1]), fw_buf + addr, 128); // copy fw_buf to buf

		err = i2c_write_bytes(client, buf, 129);	 // 128 byte + 1 cmd = 129
		if (err !=1)
			printk("[ENE_DT] i2c_write_bytes:err %d\n", err);

		// 7. Set address to 0xF807
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x07;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[ENE_DT] i2c_write_bytes:err %d\n", err);

		// 8. Program page
		buf[0] = 0x05;
		buf[1] = 0x70;
		err = i2c_write_bytes(client, buf, 2);
		if (err !=1)
			printk("[ENE_DT] i2c_write_bytes:err %d\n", err);

		while(1){	 // 9. check cmd finished 0xF800[7] = 1
			msleep(50);
			data[0] = 0x0;
			err = ene_8k41_read_bytes(client, 0xF800, data);
			if (err != 2) {
				printk("[ENE_DT] REG[0xF800]: read err %d\n", err);
			}
			if ( ((data[0] >> 7) & 0x1) == 1 ){
				//printk("[ENE_DT] check REG[0xF800] pass, 0x%02x\n", (data[0]));
				retry = 0;
				break;
			}
			printk("[ENE_DT] check REG[0xF800], 0x%02x\n", (data[0]));
			retry++;
			if (retry > 10) {
				printk("[ENE_DT] retry too many times: %d, Force exit!!!\n", retry);
				kfree(buf);
				return -1;
			}
		}
	}
	
////////////////////////////
/*
	printk("[ENE_DT] read FW in 8K41 rom +++\n");
	for (addr = 0x0000 ; addr <= 0x0080 ; addr = addr + 0x80) {
		printk("[ENE_DT] set addr : 0x%04x\n", addr);
		err = ene_8k41_write_bytes(client, 0xF808, (addr & 0xFF));				//	Addr_L
		if (err !=1)
			printk("[ENE_DT] REG[0xF808]: write err %d\n", err);
		err = ene_8k41_write_bytes(client, 0xF809, (addr >> 8) & 0xFF);		//	Addr_H
		if (err !=1)
			printk("[ENE_DT] REG[0xF809]: write err %d\n", err);

		// 5. Set address to 0xF80B
		buf[0] = 0x00;
		buf[1] = 0xF8;	//Addr_H
		buf[2] = 0x0B;	//Addr_L
		err = i2c_write_bytes(client, buf, 3);	//set register address
		if (err !=1)
			printk("[ENE_DT] i2c_write_bytes:err %d\n", err);

		buf[0] = 0x06;
		err = i2c_read_bytes(client, buf, 1, tmp, 128);	//set register address
		if (err !=2)
			printk("[ENE_DT] i2c_read_bytes:err %d\n", err);

		for (i=0; i<128 ; i++)
			printk("[ENE_DT] tmp[%d]:0x%x\n", i, tmp[i]);
	}
	printk("[ENE_DT] read FW in 8K41 rom ---\n");
*/
////////////////////////////

// (9) Restart 8051
	printk("[ENE_DT] Restart 8051.\n");
	err = ene_8k41_write_bytes(client, 0xF010, 0x04);
	if (err !=1)
		printk("[ENE_DT] REG[0xF807]: write err %d\n", err);

	msleep(500);

	err = ene_8k41_write_bytes(client, 0xF018, 0x00);
	if (err !=1)
		printk("[ENE_DT] REG[F010]: write err %d\n", err);

	platform_data->FW_update_done = true;
	printk("[ENE_DT] ene_UpdateFirmware finished. %d\n", platform_data->FW_update_done);
	kfree(buf);
	return 0;
}

static ssize_t fw_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int err = 0;
	char fw_name[128];
	unsigned char *fw_buf;
	int fw_size;

	memset(fw_name, 0, sizeof(fw_name));
	sprintf(fw_name, "%s", buf);
	fw_name[count-1] = '\0';
	
	printk("[ENE_DT] fwname : %s\n", fw_name);

	// get fs_size
	fw_size = ene_GetFirmwareSize(fw_name);
	printk("[ENE_DT] fwsize %d\n", fw_size);
	
	// set fw_buf
	fw_buf = kmalloc(fw_size+1 ,GFP_ATOMIC);

	// read FW content
	if (ene_ReadFirmware(fw_name, fw_buf)) {
		printk("[ENE_DT] ERROR: request_firmware failed\n");
		kfree(fw_buf);
		return -EIO;
	}

	// Start update FW
	mutex_lock(&g_pdata->ene_mutex);
	err = ene_UpdateFirmware(client, fw_buf);
	if(err)
		printk("[ENE_DT] ene_UpdateFirmware, err %d\n", err);

	// Update FW VER
	platform_data->fw_version = ene_CheckFirmwareVer(client);
	printk("[ENE_DT] FW VER : 0x%x\n", platform_data->fw_version);

	mutex_unlock(&g_pdata->ene_mutex);
	return count;
}

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	unsigned char data[4] = {0};
	int err = 0;

	mutex_lock(&g_pdata->ene_mutex);

	err = ene_8k41_read_bytes(client, 0xF018, data);
	if (err != 2){
		printk("[ENE_DT] fw_ver_show:err %d\n", err);
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"i2c_error\n");
	}

	platform_data->fw_version = ene_CheckFirmwareVer(client);

	printk("[ENE_DT] fw_ver_show : 0x%x\n", platform_data->fw_version);
	mutex_unlock(&g_pdata->ene_mutex);

	//if (data[0] != 0x0 && !platform_data->FW_update_done){
	if (data[0] != 0x0){
		printk("[ENE_DT] FW Error, REG[0xF018] : 0x%x\n", data[0]);
		return snprintf(buf, PAGE_SIZE,"0x0\n");
	}

	if (platform_data->fw_version == 0xFF){
		return snprintf(buf, PAGE_SIZE,"i2c_error\n");
	}

	return snprintf(buf, PAGE_SIZE,"0x%x\n", platform_data->fw_version);
}

static ssize_t pause_fw_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int err = 0;
	unsigned char data[2] = {0};

	mutex_lock(&platform_data->ene_mutex);

	err = ene_8k41_read_bytes(client, 0x8070, data);
	if (err != 2) {
		printk("[ENE_DT] REG[0x8070]: read err %d\n", err);
	}

	printk("[ENE_DT] REG[0x8070] : 0x%x\n", data[0]);
	mutex_unlock(&platform_data->ene_mutex);

	return snprintf(buf, PAGE_SIZE,"0x%x\n", data[0]);
}

static ssize_t pause_fw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val;
	int err;
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&platform_data->ene_mutex);

	if(val>0) {
		err = ene_8k41_write_bytes(client, 0x8070, 0xA0);
		if (err !=1)
			printk("[ENE_DT] REG[0xF807]: write err %d\n", err);
	}else {
		err = ene_8k41_write_bytes(client, 0x8070, 0xA1);
		if (err !=1)
			printk("[ENE_DT] REG[0xF807]: write err %d\n", err);
	}

	mutex_unlock(&platform_data->ene_mutex);
	return count;
}

static ssize_t pd_switch_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int err = 0;
	unsigned char data[2] = {0};

	mutex_lock(&platform_data->ene_mutex);

	err = ene_8k41_read_bytes(client, 0x8061, data);
	if (err != 2) {
		printk("[ENE_DT] REG[0x8061]: read err %d\n", err);
	}

	printk("[ENE_DT] REG[0x8061] : 0x%x\n", data[0]);
	mutex_unlock(&platform_data->ene_mutex);

	return snprintf(buf, PAGE_SIZE,"0x%x\n", data[0]);
}

static ssize_t pd_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	u32 val, retry = 0;
	int err;
	unsigned char data[2] = {0};
	ssize_t ret;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&platform_data->ene_mutex);

	if(val>0) {
		printk("[ENE_DT] PD switch to flash mode\n");

		err = ene_8k41_write_bytes(client, 0x8060, 0xA5);
		if (err !=1)
			printk("[ENE_DT] REG[0x8060]: write err %d\n", err);

		while(retry < 5){
			err = ene_8k41_read_bytes(client, 0x8061, data);
			if (err != 2)
				printk("[ENE_DT] REG[0x8061]: read err %d\n", err);

			if (data[0] == 0x5A){
				printk("[ENE_DT] REG[0x8061] : 0x%x\n", data[0]);
				break;
			}

			printk("[ENE_DT] REG[0x8061] : 0x%x\n", data[0]);
			retry++;
			msleep(500);
		}

		data[0] = 0xFF;

		err = ene_8k41_read_bytes(client, 0x8070, data);
		if (err != 2)
			printk("[ENE_DT] REG[0x8070]: read err %d\n", err);

		printk("[ENE_DT] REG[0x8070] : 0x%x\n", data[0]);

	}else {
		printk("[ENE_DT] PD switch to normal mode\n");

		err = ene_8k41_write_bytes(client, 0x8060, 0x0);
		if (err !=1)
			printk("[ENE_DT] REG[0x8060]: write err %d\n", err);

		err = ene_8k41_write_bytes(client, 0x8070, 0x0);
		if (err !=1)
			printk("[ENE_DT] REG[0x8070]: write err %d\n", err);

		while(retry < 5){
			err = ene_8k41_read_bytes(client, 0x8061, data);
			if (err != 2)
				printk("[ENE_DT] REG[0x8061]: read err %d\n", err);

			if (data[0] == 0x0){
				printk("[ENE_DT] REG[0x8061] : 0x%x\n", data[0]);
				break;
			}

			printk("[ENE_DT] REG[0x8061] : 0x%x\n", data[0]);
			retry++;
			msleep(500);
		}

		data[0] = 0xFF;
		retry = 0;

		while(retry < 5){
			err = ene_8k41_read_bytes(client, 0x8070, data);
			if (err != 2)
				printk("[ENE_DT] REG[0x8070]: read err %d\n", err);

			if (data[0] == 0x0){
				printk("[ENE_DT] REG[0x8070] : 0x%x\n", data[0]);
				break;
			}
			printk("[ENE_DT] REG[0x8070] : 0x%x\n", data[0]);
			retry++;
			msleep(500);
		}
	}

	mutex_unlock(&platform_data->ene_mutex);
	return count;
}

static ssize_t Check_AC_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[4] = {0};
	int err = 0, result = 0;

	mutex_lock(&g_pdata->ene_mutex);

	err = ene_8k41_read_bytes(client, 0x8050, data);
	if (err != 2)
		printk("[ENE_DT] REG[0x8050]: read err %d\n", err);

	if ( (data[0] >> 4) & 0x1){
		printk("[ENE_DT] AC is more than 30W\n");
		result = 1;
	} else{
		printk("[ENE_DT] AC is less than 30W\n");
		result = 0;
	}

	err = ene_8k41_read_bytes(client, 0xF018, data);
	if (err != 2){
		printk("[ENE_DT] FW check err %d\n", err);
		result = 1;
		mutex_unlock(&g_pdata->ene_mutex);
		return snprintf(buf, PAGE_SIZE,"%d\n", result);
	}


	if (data[0] != 0x0){
		printk("[ENE_DT] FW Error, REG[0xF018] : 0x%x, force return AC is 30W, let FW can recovery\n", data[0]);
		result = 1;
	}

	mutex_unlock(&g_pdata->ene_mutex);

	return snprintf(buf, PAGE_SIZE,"%d\n", result);
}

static ssize_t mode_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);
	int val = -1;

	if ( gpio_is_valid(platform_data->switch_mode_gpio) ) {
		val = gpio_get_value(platform_data->switch_mode_gpio);
		printk("[ENE_DT] switch_mode_gpio[%d] :0x%x\n", platform_data->switch_mode_gpio, val);
		return snprintf(buf, PAGE_SIZE,"%d\n", val);
	}
	printk("[ENE_DT] switch_mode_gpio is invalid, return -1\n");
	return snprintf(buf, PAGE_SIZE,"%d\n", val);
}

static ssize_t test_pin_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[2] = {0};
	int err = 0;

	err = ene_8k41_read_bytes(client, 0xF208, data);
	if (err < 0)
		printk("[ENE_DT] ene_8k41_read_bytes:err %d\n", err);

	printk("[ENE_DT] REG[0xF208] 0x%x\n", data[0]);

	return snprintf(buf, PAGE_SIZE,"0x%x\n", ((data[0] >> 4) & 0x1));
}

static ssize_t test_pin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev->parent);
	unsigned char data[2] = {0};
	u32 val;
	ssize_t ret;
	int err = 0;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return count;

	// EC FW already set 0xF203 as 0x00 for PB4 gpio output mode enable
	//err = ene_8k41_write_bytes(client, 0xF203, 0x10);
	//if (err < 0)
	//	printk("[ENE_DT] ene_8k41_write_bytes:err %d\n", err);

	//pr_debug("[ENE_DT] %s val= %d\n", __func__,val);
	if(val>0) {
		err = ene_8k41_read_bytes(client, 0xF208, data);
		if (err < 0)
			printk("[ENE_DT] ene_8k41_read_bytes:err %d\n", err);

		data[0] = (data[0] | 0x10);

		// set 0xF208 as 0x10 for PB4 gpio HIGH
		err = ene_8k41_write_bytes(client, 0xF208, data[0]);
		if (err < 0)
			printk("[ENE_DT] ene_8k41_write_bytes:err %d\n", err);
	}else {
		err = ene_8k41_read_bytes(client, 0xF208, data);
		if (err < 0)
			printk("[ENE_DT] ene_8k41_read_bytes:err %d\n", err);

		data[0] = (data[0] & 0xEF);

		// set 0xF208 as 0x00 for PB4 gpio HIGH
		err = ene_8k41_write_bytes(client, 0xF208, data[0]);
		if (err < 0)
			printk("[ENE_DT] ene_8k41_write_bytes:err %d\n", err);
	}
	return count;
}

static DEVICE_ATTR(fw_update, 0664, NULL, fw_update_store);
static DEVICE_ATTR(fw_ver, 0664, fw_ver_show, NULL);
static DEVICE_ATTR(pause, 0664, pause_fw_show, pause_fw_store);
static DEVICE_ATTR(pd_switch, 0664, pd_switch_show, pd_switch_store);
static DEVICE_ATTR(Check_AC, 0664, Check_AC_show, NULL);
static DEVICE_ATTR(mode, 0664, mode_show, NULL);
static DEVICE_ATTR(test_pin, 0664, test_pin_show, test_pin_store);

static struct attribute *pwm_attrs[] = {
	&dev_attr_fw_update.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_pause.attr,
	&dev_attr_pd_switch.attr,
	&dev_attr_Check_AC.attr,
	&dev_attr_mode.attr,
	&dev_attr_test_pin.attr,
	NULL
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};


static void aura_sync_set(struct led_classdev *led,
			      enum led_brightness brightness)
{
	printk("[ENE_DT] aura_sync_set : %d.\n", brightness);
}

static enum led_brightness aura_sync_get(struct led_classdev *led_cdev)
{
	struct ene_8k41_platform_data *pdata;

	printk("[ENE_DT] aura_sync_get.\n");
	pdata = container_of(led_cdev, struct ene_8k41_platform_data, led);

	return pdata->led.brightness;
}

static int aura_sync_register(struct device *dev, struct ene_8k41_platform_data *pdata)
{
	pdata->led.name = "DT_power";

	pdata->led.brightness = LED_OFF;
	pdata->led.max_brightness = LED_HALF;
	pdata->led.default_trigger = "none";
	pdata->led.brightness_set = aura_sync_set;
	pdata->led.brightness_get = aura_sync_get;

	return led_classdev_register(dev, &pdata->led);
}

static void aura_sync_unregister(struct ene_8k41_platform_data *pdata)
{
	led_classdev_unregister(&pdata->led);
}

static int ene_8k41_parse_dt(struct device *dev, struct ene_8k41_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int retval = 0;

	printk("[ENE_DT] ene_8k41_parse_dt\n");

	pdata->switch_mode_gpio = of_get_named_gpio_flags(np, "ene8k41,switch-mode-gpio", 0, &pdata->switch_mode_flags);
	printk("[ENE_DT] switch_mode_gpio : %d\n", pdata->switch_mode_gpio);

	// Get the pinctrl node
	pdata->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdata->pinctrl)) {
	     dev_err(dev, "%s: Failed to get pinctrl\n", __func__);
	     return 0;
	}

	printk("[ENE_DT] Get default state\n");
	pdata->pins_default = pinctrl_lookup_state(pdata->pinctrl, "default");
	if (IS_ERR_OR_NULL(pdata->pins_default)) {
		dev_err(dev, "%s: Failed to get pinctrl state default\n", __func__);
	}
	printk("[ENE_DT] set the default state\n");
	retval = pinctrl_select_state(pdata->pinctrl, pdata->pins_default);
	if (retval)
		dev_err(dev, "%s: pinctrl_select_state retval:%d\n", __func__, retval);

	return 0;
}

static int ene_8k41_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct ene_8k41_platform_data *platform_data;
	
	printk("[ENE_DT] ene_8k41_probe.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
	goto exit_check_functionality_failed;
	} else
		printk("[ENE_DT] I2C function test pass\n");

	printk("[ENE_DT] client->addr : 0x%x\n", client->addr);
	if (client->addr != 0x40) {
		client->addr = 0x40;
		printk("[ENE_DT] Force Change client->addr : 0x%x\n", client->addr);
	}
	//printk("[ENE_DT] check data[0]: 0x%x\n", data[0]);

	platform_data = devm_kzalloc(&client->dev, sizeof(struct ene_8k41_platform_data), GFP_KERNEL);
	if (!platform_data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, platform_data);
// Parse platform data from dtsi
	err = ene_8k41_parse_dt(&client->dev, platform_data);
	if (err) {
		printk("[ENE_DT] ene_8k41_parse_dt get fail !!!\n");
		return -ENOMEM;
	}

// Check FW
	platform_data->fw_version = ene_CheckFirmwareVer(client);
	printk("[ENE_DT] FW VER : 0x%x\n", platform_data->fw_version);

	err = aura_sync_register(&client->dev, platform_data);
	if (err) {
		printk("[ENE_DT] Failed to register LED device: %d\n", err);
		goto unled;
	}
	err = sysfs_create_group(&platform_data->led.dev->kobj, &pwm_attr_group);
	if (err)
			goto unled;

	mutex_init(&platform_data->ene_mutex);

	platform_data->FW_update_done = false;
// Set global variable
	g_pdata = platform_data;

	err = ene_8k41_write_bytes(client, 0x8070, 0xA1);
	if (err !=1)
		printk("[ENE_DT] REG[0xF807]: write err %d\n", err);

	printk("[ENE_DT] ene_8k41_probe done.\n");
	return 0;

unled:
	aura_sync_unregister(platform_data);
	printk("[ENE_DT] ENE 8K41 power off.\n");

exit_check_functionality_failed:
	printk("[ENE_DT] ene_8k41_probe fail !!!\n");
	return err;
}

static int ene_8k41_remove(struct i2c_client *client)
{
	int err = 0;
	struct ene_8k41_platform_data *platform_data = i2c_get_clientdata(client);

// unregister
	printk("[ENE_DT] sysfs_remove_group\n");
	sysfs_remove_group(&platform_data->led.dev->kobj, &pwm_attr_group);

	printk("[ENE_DT] aura_sync_unregister\n");
	aura_sync_unregister(platform_data);

	mutex_destroy(&platform_data->ene_mutex);
	printk("[ENE_DT] ene_8k41_remove : err %d\n", err);
	return 0;
}

int ene_8k41_dt_suspend(struct device *dev)
{
	int err = 0;

//	printk("[ENE_DT] ene_8k41_dt_suspend.\n");

	return err;
}

int ene_8k41_dt_resume(struct device *dev)
{
	int err = 0;

//	printk("[ENE_DT] ene_8k41_dt_resume.\n");

	return err;
}

static const struct i2c_device_id ene_8k41_id[] = {
	{ "ene_8k41_i2c", 0},
	{},
};
//MODULE_DEVICE_TABLE(i2c, ene_8k41_id);

static const struct dev_pm_ops ene_8k41_pm_ops = {
	.suspend	= ene_8k41_dt_suspend,
	.resume	= ene_8k41_dt_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id ene_match_table[] = {
	{ .compatible = "ene8k41_power",},
	{ },
};
#else
#define ene_match_table NULL
#endif

static struct i2c_driver ene_8k41_driver = {
	.driver		= {
		.name		= "ene8k41_power",
		.owner = THIS_MODULE,
		.pm	= &ene_8k41_pm_ops,
		.of_match_table	= ene_match_table,
	},
	.probe		= ene_8k41_probe,
	.remove		= ene_8k41_remove,
	.id_table 	= ene_8k41_id,
};

static int __init ene_8k41_bus_init(void)
{
	int ret;

	ret = i2c_add_driver(&ene_8k41_driver);
	if (ret)
		printk("[ENE_DT] ENE 8k41 driver int failed.\n");
	else
		printk("[ENE_DT] ENE 8k41 driver int success.\n");
	
	return ret;
}
module_init(ene_8k41_bus_init);

static void __exit ene_8k41_bus_exit(void)
{
	i2c_del_driver(&ene_8k41_driver);
}
module_exit(ene_8k41_bus_exit);

MODULE_AUTHOR("ASUS Deeo Ho");
MODULE_DESCRIPTION("Aura sync LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:ene-8k41");
