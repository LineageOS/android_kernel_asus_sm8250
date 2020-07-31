#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/syscalls.h>

#include "ec_i2c_updatefw.h"

#define	I2C_CLASS_NAME		    "ec_i2c_fw"
struct class *ec_i2c_fw_class;
struct ec_i2c_platform_data *ec_i2c_fw_data;
#define I2C_RETRY_NUMBER        3
static DEFINE_MUTEX(i2c_rw_access);
int flag = 0;


static int i2c_read_bytes(struct i2c_client *client, char *write_buf, int writelen, char *read_buf, int readlen)
{
	struct i2c_msg msgs[2];
	int ret=-1;

	mutex_lock(&i2c_rw_access);

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

	mutex_unlock(&i2c_rw_access);
	return ret;
}

static int i2c_write_bytes(struct i2c_client *client, char *write_buf, int writelen)
{
	struct i2c_msg msg;
	int ret=-1;
	int i = 0;

	mutex_lock(&i2c_rw_access);	
	
	msg.flags = !I2C_M_RD;		//write
	msg.addr = client->addr;
	msg.len = writelen;
	msg.buf = write_buf; 


	for (i = 0; i < I2C_RETRY_NUMBER; i++) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			printk("[EC_I2C_FW] : %s:i2c_transfer(write) error, ret=%d", __func__, ret);
		} else
			break;
	}

	mutex_unlock(&i2c_rw_access);
	
	return ret;
} 

static int ene_8k41_read_bytes(struct i2c_client *client, short addr, char *data)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = 0x00;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;

//	printk("[AURA_INBOX] ene_8k41_read_bytes : addr: 0x%x, buf[1] : 0x%x, buf[2] : 0x%x\n", (addr & 0xFFFF), buf[1], buf[2]);
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[EC_I2C_FW] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x81;
	err = i2c_read_bytes(client, buf, 1, data, 1);	//send read command
	if (err != 2)
		printk("[EC_I2C_FW] i2c_read_bytes:err %d\n", err);

	return err;
}

static int ene_8k41_write_bytes(struct i2c_client *client, short addr, char value)
{
	int err = 0;
	unsigned char buf[16] = {0};

	buf[0] = 0x00;
	buf[1] = (addr >> 8) & 0xFF;
	buf[2] = addr & 0xFF;

	if(flag == 1)
		printk("[EC_I2C_FW] set address : buf[0] is 0x%x buf[1] is 0x%x buf[2] is 0x%x \n ",buf[0],buf[1],buf[2]);
	
	err = i2c_write_bytes(client, buf, 3);	//set register address
	if (err !=1)
		printk("[EC_I2C_FW] i2c_write_bytes:err %d\n", err);

	buf[0] = 0x01;
	buf[1] = value;

	if(flag == 1)
		printk("[EC_I2C_FW] write byte : buf[0] is 0x%x buf[1] is 0x%x \n",buf[0],buf[1]);
	
	err = i2c_write_bytes(client, buf, 2);	//set register address
	if (err !=1)
		printk("[EC_I2C_FW] i2c_write_bytes:err %d\n", err);
	
	return err;
}

static int ene_RunUserCode(struct i2c_client *client)
{
	int err = 0;
	unsigned char data = 0;


	printk("[EC_I2C_FW] : Start ene_RunUserCode \n");
	err = ene_8k41_read_bytes(client, 0xF0F9, &data);
	if (err != 2)
		printk("[EC_I2C_FW] REG[0xF0F9]: read err %d\n", err);
	
	if(0x02 == data)
	{
		flag = 1 ;

		printk("[EC_I2C_FW] : check 0xF0F9 is 0x%x\n",data);

		err = ene_8k41_write_bytes(client,0x0606,0xF7);
		if (err !=1)
			printk("[EC_I2C_FW] REG[0x0606]: write err %d\n", err);
		
		
		err = ene_8k41_write_bytes(client,0xF0F1,0x00);
		if (err !=1)
			printk("[EC_I2C_FW] REG[0xF0F1]: write err %d\n", err);

		err = ene_8k41_write_bytes(client,0xF0F2,0x00);
		if (err !=1)
			printk("[EC_I2C_FW] REG[0xF0F2]: write err %d\n", err);
		
		err = ene_8k41_write_bytes(client,0xF0F3,0x00);
		if (err !=1)
			printk("[EC_I2C_FW] REG[0xF0F3]: write err %d\n", err);

		err = ene_8k41_write_bytes(client,0xF0F4,0x00);
		if (err !=1)
			printk("[EC_I2C_FW] REG[0xF0F4]: write err %d\n", err);

		err = ene_8k41_write_bytes(client,0xF0F5,0x00);
		if (err !=1)
			printk("[EC_I2C_FW] REG[0xF0F5]: write err %d\n", err);

		err = ene_8k41_write_bytes(client,0xF0F6,0x00);
		if (err !=1)
			printk("[EC_I2C_FW] REG[0xF0F6]: write err %d\n", err);		

		err = ene_8k41_write_bytes(client,0xF0F7,0x00);
		if (err !=1)
			printk("[EC_I2C_FW] REG[0xF0F7]: write err %d\n", err); 

		err = ene_8k41_write_bytes(client,0xF0F8,0x80);
		if (err !=1)
			printk("[EC_I2C_FW] REG[0xF0F8]: write err %d\n", err); 

		err = ene_8k41_write_bytes(client,0xF0F9,0x00);
		if (err !=1)
			printk("[EC_I2C_FW] REG[0xF0F9]: write err %d\n", err);		


		printk("[EC_I2C_FW] : End run user code \n");
		
	} else {
		printk("[EC_I2C_FW] ene_RunUserCode REG[0xF0F1] is not 0x02\n");
	}

	return 0;

}

/*
static int ene_CheckFirmware(struct i2c_client *client, int fw_size , char *fw_buf)
{
	int err = 0;
	int flash_address = 0x1800;
	int addr = 0x0000;
	unsigned char *fw_buf_doing;
	int Readbuf0_flag = 0;
	int Readbuf1_flag = 0;
	unsigned char buf[32] = {0};

	// set fw_buf
	fw_buf_doing = kmalloc(fw_size,GFP_ATOMIC);

	fw_buf_doing = fw_buf;

	while((flash_address-0x1800) < fw_size)
	{
		err = ene_8k41_read_bytes(client, 0xF0F9, &data);
		if (err != 2)
			printk("[EC_I2C_FW] REG[0xF0F9]: read err %d\n", err);

		if((0x02 == data) && (Readbuf0_flag == 0))
		{
			for (addr = 0xF000 ; addr <= 0xF07F; addr = addr + 0x20) 
			{
				
			}
		}
		
	}
	
	return 0;
}*/

static int ene_FlashFirmware(struct i2c_client *client, int fw_size , char *fw_buf)
{
	int err = 0;
	int flash_address = 0x1800;
	int addr = 0x0000;
	unsigned char *buf;
	unsigned char data = 0;
	char *fw_buf_doing;
	unsigned char temp[16] = {0};

	int num = 0;

	int i = 0;

	buf = kmalloc(sizeof(unsigned char)*34, GFP_DMA);

	fw_buf_doing = &fw_buf[6144];

	printk("[EC_I2C_FW] : start ene_FlashFirmware !\n");
	while((flash_address-0x1800) < (fw_size-6144)) 
	{
		err = ene_8k41_read_bytes(client, 0xF0F9, &data);
		if (err != 2)
			printk("[EC_I2C_FW] REG[0xF0F9]: read err %d\n", err);

		if(0x02 == data)
		{
			printk("[EC_I2C_FW] ene_FlashFirmware  REG[0xF0F9] is 0x02\n");
			
			for (addr = 0xF000 ; addr <= 0xF07F; addr = addr + 0x20) 
			{

				printk("[EC_I2C_FW] : ene_FlashFirmware addr is 0x%x%x!\n",((addr >> 8)&0xFF),(addr&0xFF));
				memcpy(&buf[2],fw_buf_doing, 32);


				if(num == 298)
				{
					printk("[LOTTA] : num is %d\n",num);
					for(i=2;i<34;i++)
					{
						printk("[LOTTA] : buf[%d] is 0x%x\n",i,buf[i]);
					}
				}
			
				temp[0] = 0x00;
				temp[1] = ((addr >> 8)&0xFF);
				temp[2] = (addr&0xFF);

				err = i2c_write_bytes(client, temp, 3);
				if (err !=1)
					printk("[EC_I2C_FW] i2c_write_bytes:err %d\n", err);
				
				buf[0] = 0x03;
				buf[1] = 0x20;
				err = i2c_write_bytes(client, buf, 34);
				if (err !=1)
					printk("[EC_I2C_FW] i2c_write_bytes:err %d\n", err);
				
				fw_buf_doing = fw_buf_doing + 0x20;

				memset(buf,0,34);
				memset(temp,0,16);
				
			}
			
			err = ene_8k41_write_bytes(client,0xF0F1,(flash_address>>16)&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F1]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF0F2,(flash_address>>8)&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F2]: write err %d\n", err);
		
			err = ene_8k41_write_bytes(client,0xF0F3,flash_address&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F3]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF0F4,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F4]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF0F5,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F5]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF0F6,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F6]: write err %d\n", err);		

			err = ene_8k41_write_bytes(client,0xF0F7,0x80);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F7]: write err %d\n", err); 

			err = ene_8k41_write_bytes(client,0xF0F8,0x20);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F8]: write err %d\n", err); 

			err = ene_8k41_write_bytes(client,0xF0F9,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F9]: write err %d\n", err); 

			flash_address = flash_address + 0x80;
			num++;
		}

		data = 0;
		
		err = ene_8k41_read_bytes(client, 0xF1F9, &data);
		if (err != 2)
			printk("[EC_I2C_FW] REG[0xF1F9]: read err %d\n", err);
	
		if(0x02 == data && num < 299)
		{
			printk("[EC_I2C_FW] ene_FlashFirmware REG[0xF1F9] is 0x02\n");
		
			for (addr = 0xF100 ; addr <= 0xF17F; addr = addr + 0x20) 
			{
			    printk("[EC_I2C_FW] : ene_FlashFirmware addr is 0x%x%x!\n",((addr >> 8)&0xFF),(addr&0xFF));

				memcpy(&buf[2],fw_buf_doing, 32);

				if(num == 299)
				{
					printk("[LOTTA] : num is %d\n",num);
					for(i=2;i<34;i++)
					{
						printk("[LOTTA] : buf[%d] is 0x%x\n",i,buf[i]);
					}
				}
				temp[0] = 0x00;
				temp[1] = ((addr >> 8)&0xFF);
				temp[2] = (addr&0xFF);

				err = i2c_write_bytes(client, temp, 3);
				if (err !=1)
					printk("[EC_I2C_FW] i2c_write_bytes:err %d\n", err);

				buf[0] = 0x03;
				buf[1] = 0x20;
				err = i2c_write_bytes(client, buf, 34);
				if (err !=1)
					printk("[EC_I2C_FW] i2c_write_bytes:err %d\n", err);
				
				fw_buf_doing = fw_buf_doing + 0x20;

				memset(buf,0,34);
				memset(temp,0,16);
			}
		
			err = ene_8k41_write_bytes(client,0xF1F1,(flash_address>>16)&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F1]: write err %d\n", err);
	
			err = ene_8k41_write_bytes(client,0xF1F2,(flash_address>>8)&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F2]: write err %d\n", err);
	
			err = ene_8k41_write_bytes(client,0xF1F3,flash_address&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F3]: write err %d\n", err);
	
			err = ene_8k41_write_bytes(client,0xF1F4,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F4]: write err %d\n", err);
	
			err = ene_8k41_write_bytes(client,0xF1F5,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F5]: write err %d\n", err);
	
			err = ene_8k41_write_bytes(client,0xF1F6,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F6]: write err %d\n", err); 	
	
			err = ene_8k41_write_bytes(client,0xF1F7,0x80);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F7]: write err %d\n", err); 
	
			err = ene_8k41_write_bytes(client,0xF1F8,0x20);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F8]: write err %d\n", err); 
	
			err = ene_8k41_write_bytes(client,0xF1F9,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F9]: write err %d\n", err); 
	
			flash_address = flash_address + 0x80;
			num++;
		}

	}


	printk("[EC_I2C_FW] : num is %d\n",num);

	printk("[EC_I2C_FW] : ene_FlashFirmware sucess!\n");
	
	kfree(buf);
	
	return 0;
}

static int ene_EraseFirmware(struct i2c_client *client, int fw_size)
{
	int err = 0;
	int flash_address = 0x2000;
	unsigned char data = 0;

	while((flash_address-0x2000) < fw_size) 
	{
		err = ene_8k41_read_bytes(client, 0xF0F9, &data);
		if (err != 2)
			printk("[EC_I2C_FW] REG[0xF0F9]: read err %d\n", err);
	
		if(0x02 == data)
		{
			printk("[EC_I2C_FW] REG[0xF0F9] is 0x02\n");

			err = ene_8k41_write_bytes(client,0xF0F1,(flash_address>>16)&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F1]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF0F2,(flash_address>>8)&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F2]: write err %d\n", err);
		
			err = ene_8k41_write_bytes(client,0xF0F3,flash_address&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F3]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF0F4,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F4]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF0F5,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F5]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF0F6,0x10);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F6]: write err %d\n", err);		

			err = ene_8k41_write_bytes(client,0xF0F7,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F7]: write err %d\n", err); 

			err = ene_8k41_write_bytes(client,0xF0F8,0x12);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F8]: write err %d\n", err); 

			err = ene_8k41_write_bytes(client,0xF0F9,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF0F9]: write err %d\n", err); 

			flash_address = flash_address + 0x1000;
		}

		data = 0;
		
		err = ene_8k41_read_bytes(client, 0xF1F9, &data);
		if (err != 2)
			printk("[EC_I2C_FW] REG[0xF1F9]: read err %d\n", err);
	
		if(0x02 == data)
		{
			printk("[EC_I2C_FW] REG[0xF1F9] is 0x02\n");

			err = ene_8k41_write_bytes(client,0xF1F1,(flash_address>>16)&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F1]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF1F2,(flash_address>>8)&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F2]: write err %d\n", err);
		
			err = ene_8k41_write_bytes(client,0xF1F3,flash_address&0xFF);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F3]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF1F4,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F4]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF1F5,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F5]: write err %d\n", err);

			err = ene_8k41_write_bytes(client,0xF1F6,0x10);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F6]: write err %d\n", err);		

			err = ene_8k41_write_bytes(client,0xF1F7,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F7]: write err %d\n", err); 

			err = ene_8k41_write_bytes(client,0xF1F8,0x12);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F8]: write err %d\n", err); 

			err = ene_8k41_write_bytes(client,0xF1F9,0x00);
			if (err !=1)
				printk("[EC_I2C_FW] REG[0xF1F9]: write err %d\n", err); 

			flash_address = flash_address + 0x1000;
		}
	}

	printk("[EC_I2C_FW]: ene_EraseFirmware success !\n");

	return 0;
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
	inode = pfile->f_dentry->d_inode;
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

static int ene_GetFirmwareSize(char *firmware_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	printk("[EC_I2C_FW] ene_GetFirmwareSize.\n");
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if (NULL == pfile) {
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if (IS_ERR(pfile)) {
		pr_err("[EC_I2C_FW] : error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static ssize_t fw_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	char fw_name[128];
	unsigned char *fw_buf;
	int fw_size = 0;
	unsigned char data[2] = {0};

	memset(fw_name, 0, sizeof(fw_name));
	sprintf(fw_name, "%s", buf);
	fw_name[count-1] = '\0';

	printk("[EC_I2C_FW] fwname : %s\n", fw_name);

	// get fs_size
	fw_size = ene_GetFirmwareSize(fw_name);
	printk("[EC_I2C_FW] fwsize %d\n", fw_size);

	// set fw_buf
	fw_buf = kmalloc(fw_size,GFP_ATOMIC);


	// read FW content
	if (ene_ReadFirmware(fw_name, fw_buf)) {
		printk("[EC_I2C_FW] ERROR: request_firmware failed\n");
		kfree(fw_buf);
		return -EIO;
	}

	memset(&fw_buf[fw_size],0,172);
	
	err = ene_EraseFirmware(ec_i2c_fw_data->client,fw_size);
	if(err)
		printk("[EC_I2C_FW] ene_EraseFirmware, err %d\n", err);


	err= ene_FlashFirmware(ec_i2c_fw_data->client,fw_size,fw_buf);
	if(err)
		printk("[EC_I2C_FW] ene_EraseFirmware, err %d\n", err);

	msleep(500);
	
	ene_RunUserCode(ec_i2c_fw_data->client);


	err = ene_8k41_read_bytes(ec_i2c_fw_data->client, 0x0606, data);
	if (err != 2)
		printk("[EC_I2C_FW] REG[0x0606]: read err %d\n", err);

	printk("[EC_I2C_FW] : data is 0x%x\n",data[0]);
	
	if(!(data[0]& 0x04)) {
		printk("[EC_I2C_FW]: ec has enter into user code mode \n ");
	}

	kfree(fw_buf);
	
	return count;

}

static DEVICE_ATTR(fw_update, 0664, NULL, fw_update_store);

static struct attribute *ec_i2c_fw_attrs[] = {
	&dev_attr_fw_update.attr,
	NULL
};

const struct attribute_group ec_i2c_fw_group = {
	.attrs = ec_i2c_fw_attrs,
};

int ec_i2c_fw_create_sysfs(struct i2c_client *client)
{
    int ret = 0;
	
	ec_i2c_fw_class = class_create(THIS_MODULE, I2C_CLASS_NAME);
	if (IS_ERR(ec_i2c_fw_class)) {
		printk("[EC_I2C_FW] ec_i2c_create_sysfsis failed - unregister chrdev.\n");
	}

	device_create(ec_i2c_fw_class, &client->dev,
			    ec_i2c_fw_data->devt, ec_i2c_fw_data, "fw");

    ret = sysfs_create_group(&client->dev.kobj, &ec_i2c_fw_group);
    if (ret) {
        printk("[EC_I2C_FW]: sysfs_create_group() failed!!");
        sysfs_remove_group(&client->dev.kobj, &ec_i2c_fw_group);
        return -ENOMEM;
    } else {
        printk("[EC_I2C_FW]: sysfs_create_group() succeeded!!");
    }

    return ret;
}

static int ec_i2c_fw_remove(struct i2c_client *client)
{
	return 0;
}

static int ec_i2c_fw_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int err = 0;
	struct ec_i2c_platform_data *platform_data;
	unsigned char data[2] = {0};
	
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
       		printk("[EC_I2C_FW] i2c_check_functionality error !\n");
       		return -ENODEV;
    }

	platform_data = devm_kzalloc(&client->dev, sizeof(struct ec_i2c_platform_data), GFP_KERNEL);
	if (!platform_data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	ec_i2c_fw_data = platform_data;
	platform_data->client = client;

    err = ec_i2c_fw_create_sysfs(client);
    if (err) {
        printk("[EC_I2C_FW]create sysfs node fail");
    }

	printk("[EC_I2C_FW] start enter bootloader mode !\n");
	
	//disable WDT
	err = ene_8k41_write_bytes(client,0x0600,0x48);
	if (err !=1)
		printk("[EC_I2C_FW] REG[0x0600]: write err %d\n", err);

	//see system flag definition in 0x0606
	err = ene_8k41_write_bytes(client,0x0606,0xE1);
	if (err !=1)
		printk("[EC_I2C_FW] REG[0x0606]: write err %d\n", err);

	//disable WDT
	err = ene_8k41_write_bytes(client,0x0600,0x48);
	if (err !=1)
		printk("[EC_I2C_FW] REG[0x0600]: write err %d\n", err);

	//clear pending flag
	err = ene_8k41_write_bytes(client,0x0601,0x03);
	if (err !=1)
		printk("[EC_I2C_FW] REG[0x0601]: write err %d\n", err);

	//whole chip reset
	err = ene_8k41_write_bytes(client,0x0604,0x00);
	if (err !=1)
		printk("[EC_I2C_FW] REG[0x0604]: write err %d\n", err);

	//set WDT counter
	err = ene_8k41_write_bytes(client,0x0602,0x00);
	if (err !=1)
		printk("[EC_I2C_FW] REG[0x0602]: write err %d\n", err);

	//set WDT counter
	err = ene_8k41_write_bytes(client,0x0603,0x10);
	if (err !=1)
		printk("[EC_I2C_FW] REG[0x0603]: write err %d\n", err);

	//restart M0 with WDT reset
	err = ene_8k41_write_bytes(client,0x0600,0x03);
	if (err !=1)
		printk("[EC_I2C_FW] REG[0x0600]: write err %d\n", err);


	msleep(500);

	err = ene_8k41_read_bytes(client, 0x0606, data);
	if (err != 2)
		printk("[EC_I2C_FW] REG[0x0606]: read err %d\n", err);

	printk("[EC_I2C_FW] : data is 0x%x\n",data[0]);

	if(data[0]& 0x02) {
		printk("[EC_I2C_FW] REG[0x0606] Bootloader ready flag 0x0606(0x%x) bit has ready!\n",data[0]);
	}

	if(data[0]& 0x04) {
		printk("[EC_I2C_FW]: ec has enter into bootloader mode \n ");
	}

	printk("[EC_I2C_FW] end enter bootloader mode !\n");

	return 0;
}

int ec_i2c_fw_suspend(struct device *dev)
{
	int err = 0;

	printk("[EC_I2C_FW] ec_i2c_suspend !\n");

	return err;
}

int ec_i2c_fw_resume(struct device *dev)
{
	int err = 0;

	printk("[EC_I2C_FW] ec_i2c_resume !\n");

	return err;
}

static const struct i2c_device_id ec_i2c_fw_id[] = {
	{ "ec_i2c_ap", 0},
	{},
};

static const struct dev_pm_ops ec_i2c_fw_pm_ops = {
	.suspend	= ec_i2c_fw_suspend,
	.resume		= ec_i2c_fw_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id ec_i2c_fw_match_table[] = {
	{ .compatible = "ec_i2c_ap",},
	{ },
};
#else
#define ene_match_table NULL
#endif


static struct i2c_driver ec_i2c_fw_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver		= {
		.name		= "ec_i2c_ap",
		.owner = THIS_MODULE,
		.pm	= &ec_i2c_fw_pm_ops,
		.of_match_table	= ec_i2c_fw_match_table,
	},
	.probe		= ec_i2c_fw_probe,
	.remove		= ec_i2c_fw_remove,
	.id_table 	= ec_i2c_fw_id,
};

static int __init ec_i2c_fw_bus_init(void)
{
	int ret;

	ret = i2c_add_driver(&ec_i2c_fw_driver);
	if (ret)
		printk("[EC_I2C_FW] EC_I2C driver init failed.\n");
	else
		printk("[EC_I2C_FW] EC_I2C driver init success.\n");
	
	return ret;
}
module_init(ec_i2c_fw_bus_init);

static void __exit ec_i2c_fw_bus_exit(void)
{
	i2c_del_driver(&ec_i2c_fw_driver);
}
module_exit(ec_i2c_fw_bus_exit);


MODULE_AUTHOR("ASUS Lotta Lu");
MODULE_DESCRIPTION("EC I2C fw");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("asus:ec fw");
