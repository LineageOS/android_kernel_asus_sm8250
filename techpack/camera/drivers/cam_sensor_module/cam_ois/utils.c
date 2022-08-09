#include "utils.h"
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

#undef  pr_fmt
#define pr_fmt(fmt) "OIS-UTIL %s(): " fmt, __func__

void swap_word_data(uint16_t* register_data){
	*register_data = ((*register_data >> 8) | ((*register_data & 0xff) << 8)) ;
}

int format_hex_string(char *output_buf,int len, uint8_t* data,int count)
{
	int i;
	int offset;

	if(len<count*3+1+1)
		return -1;

	offset=0;

	for(i=0;i<count;i++)
	{
		offset+=sprintf(output_buf+offset,"0x%02x ",data[i]);
	}
	offset+=sprintf(output_buf+offset,"\n");

	return offset;
}

void delay_ms(uint32_t time)
{
	usleep_range(time*1000,time*1000+time*10);
}

void delay_us(uint32_t time)
{
	usleep_range(time,time+time/100);
}

int64_t diff_time_us(struct timeval *t1, struct timeval *t2 )
{
	return (((t1->tv_sec*1000000)+t1->tv_usec)-((t2->tv_sec*1000000)+t2->tv_usec));
}

int32_t get_file_size(const char *filename, uint64_t* size)
{
    struct kstat stat;
    mm_segment_t fs;
    int rc = 0;

	stat.size = 0;

    fs = get_fs();
    set_fs(KERNEL_DS);

	rc = vfs_stat(filename,&stat);
	if(rc < 0)
	{
		pr_err("vfs_stat(%s) failed, rc = %d\n",filename,rc);
		rc = -1;
		goto END;
	}

    *size = stat.size;
END:
	set_fs(fs);
    return rc;
}

int read_file_into_buffer(const char *filename, uint8_t* data, uint32_t size)
{
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;
	int rc;

    fp =filp_open(filename,O_RDONLY,S_IRWXU | S_IRWXG | S_IRWXO);
    if (IS_ERR(fp)){
		pr_err("open(%s) failed\n",filename);
        return -1;
    }

    fs =get_fs();
    set_fs(KERNEL_DS);

    pos =0;
	rc = vfs_read(fp,data, size, &pos);

    set_fs(fs);
    filp_close(fp,NULL);

    return rc;
}

int sysfs_write_byte_seq(char *filename, uint8_t *value, uint32_t size)
{
	struct file *fp = NULL;
	int i = 0;
	char buf[3];
	ssize_t ret;

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("open (%s) failed!\n",filename);
		return -1;
	}

	for(i = 0; i < size; i++){
		sprintf(buf, "%02x", value[i]);
		buf[2] = '\n';

		ret = __kernel_write(fp, buf, 3, &fp->f_pos);
		if(ret < 3)
		{
			pr_err("write failed!\n");
			/* Close file */
			filp_close(fp, NULL);
			return -2;
		}
	}

	/* Close file */
	filp_close(fp, NULL);
	return 0;
}

/** @brief read many word(two bytes) from file
*
*	@param filename the file to write
*	@param value the word which will store the calibration data from read file
*	@param size the size of write data
*
*/
int sysfs_read_word_seq(char *filename, uint16_t *value, uint32_t size)
{
	int i = 0;
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[5];
	ssize_t buf_size = 0;
	mm_segment_t old_fs;
	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("file (%s) not exist!\n",filename);
		return -ENOENT;	/*No such file or directory*/
	}

	pos_lsts = 0;
	for(i = 0; i < size; i++){
		old_fs = get_fs();
		set_fs(get_ds());
		buf_size = vfs_read(fp, (void __user *)buf, 5, &pos_lsts);
		set_fs(old_fs);

		if(buf_size < 5) {
			pr_err("read failed!\n");
			/* close file */
			filp_close(fp, NULL);
			return -1;
		}
		buf[4]='\0';
		sscanf(buf, "%hx", &value[i]);
	}

	/* close file */
	filp_close(fp, NULL);

	return 0;
}
/** @brief read many dword(four bytes) from file
*
*	@param filename the file to write
*	@param value the dword which will store the calibration data from read file
*	@param size the size of write data
*
*/
int sysfs_read_dword_seq(char *filename, uint32_t *value, uint32_t size)
{
	int i = 0;
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[9];
	ssize_t buf_size = 0;
	mm_segment_t old_fs;
	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("file (%s) not exist!\n",filename);
		return -ENOENT;	/*No such file or directory*/
	}

	pos_lsts = 0;
	for(i = 0; i < size; i++){
		old_fs = get_fs();
		set_fs(get_ds());
		buf_size = vfs_read(fp, (void __user *)buf, 9, &pos_lsts);
		set_fs(old_fs);

		if(buf_size < 9) {
			pr_err("read failed!\n");
			/* close file */
			filp_close(fp, NULL);
			return -1;
		}
		buf[8]='\0';
		sscanf(buf, "%x", &value[i]);
	}

	/* close file */
	filp_close(fp, NULL);

	return 0;
}

/** @brief read many byte from file
*
*	@param filename the file to write
*	@param value the byte which will store the calibration data from read file
*	@param size the size of write data
*
*/
int sysfs_read_byte_seq(char *filename, uint8_t *value, uint32_t size)
{
	int i = 0;
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[3];
	ssize_t read_size = 0;
	mm_segment_t old_fs;

	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("file (%s) not exist!\n",filename);
		return -ENOENT;	/*No such file or directory*/
	}

	pos_lsts = 0;
	for(i = 0; i < size; i++){
		old_fs = get_fs();
		set_fs(get_ds());
		read_size = vfs_read(fp, (void __user *)buf, 3, &pos_lsts);
		set_fs(old_fs);

		if(read_size < 3) {
			pr_err("read failed!\n");
			/* close file */
			filp_close(fp, NULL);
			return -1;
		}

		buf[2]='\0';

		sscanf(buf, "%02hhx", &value[i]);
	}

	/* close file */
	filp_close(fp, NULL);

	return 0;
}

int sysfs_read_uint8(char *filename, uint8_t *value)
{
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[5];
	ssize_t read_size = 0;
	mm_segment_t old_fs;

	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("file (%s) not exist!\n",filename);
		return -ENOENT;	/*No such file or directory*/
	}

	pos_lsts = 0;

	old_fs = get_fs();
	set_fs(get_ds());
	memset(buf,0,sizeof(buf));
	read_size = vfs_read(fp, (void __user *)buf, sizeof(buf), &pos_lsts);
	set_fs(old_fs);

	if(read_size < 1) {
		pr_err("read failed!\n");
		/* close file */
		filp_close(fp, NULL);
		return -1;
	}
	else
	{
		pr_info("read size is %ld\n",read_size);
	}

	sscanf(buf, "%hhu", value);

	/* close file */
	filp_close(fp, NULL);

	return 0;
}

/** @brief write many words(two bytes)  to file
*
*	@param filename the file to write
*	@param value the word which will be written to file
*	@param size the size of write data
*
*/
int sysfs_write_word_seq(char *filename, uint16_t *value, uint32_t size)
{
	struct file *fp = NULL;
	int i = 0;
	char buf[5];
	ssize_t ret;

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("open (%s) failed!\n",filename);
		return -1;
	}

	for(i = 0; i < size; i++){
		sprintf(buf, "%04x", value[i]);
		buf[4] = '\n';

		ret = __kernel_write(fp, buf, 5, &fp->f_pos);
		if(ret < 5)
		{
			pr_err("write failed!\n");
			/* Close file */
			filp_close(fp, NULL);
			return -2;
		}
	}

	/* close file */
	filp_close(fp, NULL);
	return 0;
}
int sysfs_read_char_seq(char *filename, uint8_t *value, uint32_t size)
{
	int i = 0;
	struct file *fp = NULL;
	loff_t pos_lsts = 0;
	char buf[2];
	ssize_t read_size = 0;
	mm_segment_t old_fs;

	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("file (%s) not exist!\n",filename);
		return -ENOENT;	/*No such file or directory*/
	}

	pos_lsts = 0;
	for(i = 0; i < size; i++){
		old_fs = get_fs();
		set_fs(get_ds());
		read_size = vfs_read(fp, (void __user *)buf, 1, &pos_lsts);
		set_fs(old_fs);

		if(read_size < 1) {
			pr_err("read failed!\n");
			/* close file */
			filp_close(fp, NULL);
			return -1;
		}
		buf[1]='\0';

		sscanf(buf, "%hhd", &value[i]);
	}

	/* close file */
	filp_close(fp, NULL);

	return 0;
}
int sysfs_write_word_seq_change_line(char *filename, uint16_t *value, uint32_t size,uint32_t number)
{
	struct file *fp = NULL;
	int i = 0;
	char buf[5];
	ssize_t ret;

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("open (%s) failed!\n",filename);
		return -1;
	}

	for(i = 0; i < size; i++){
		sprintf(buf, "%04x", value[i]);
		if((i + 1) % number == 0)
			buf[4] = '\n';
		else
			buf[4] = ',';

		ret = __kernel_write(fp, buf, 5, &fp->f_pos);
		if(ret < 5)
		{
			pr_err("write failed!\n");
			/* Close file */
			filp_close(fp, NULL);
			return -2;
		}
	}

	/* close file */
	filp_close(fp, NULL);
	return 0;
}

int sysfs_write_dword_seq_change_line(char *filename, uint32_t *value, uint32_t size,uint32_t number,bool shift)
{
	struct file *fp = NULL;
	int i = 0;
	char buf[9];
	ssize_t ret;

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("open (%s) failed!\n",filename);
		return -1;
	}

	for(i = 0; i < size; i++){
		if(shift)
		{
			sprintf(buf, "%04x", (value[i]>>16));
			if((i + 1) % number == 0)
				buf[4] = '\n';
			else
				buf[4] = ',';

			ret = __kernel_write(fp, buf, 5, &fp->f_pos);
			if(ret < 5)
			{
				pr_err("write failed!\n");
				/* Close file */
				filp_close(fp, NULL);
				return -2;
			}
		}
		else
		{
			sprintf(buf, "%08x", value[i]);
			if((i + 1) % number == 0)
				buf[8] = '\n';
			else
				buf[8] = ',';

			ret = __kernel_write(fp, buf, 9, &fp->f_pos);
			if(ret < 9)
			{
				pr_err("write failed!\n");
				/* Close file */
				filp_close(fp, NULL);
				return -2;
			}
		}
	}

	/* close file */
	filp_close(fp, NULL);
	return 0;
}

bool i2c_setting_contain_address(struct cam_sensor_i2c_reg_setting * setting, uint32_t addr, uint32_t* data, uint16_t* index)
{
	int i;

	for(i=setting->size-1;i>=0;i--)
	{
		if(setting->reg_setting[i].reg_addr == addr)
		{
			*data = setting->reg_setting[i].reg_data;
			*index = i;
			return true;
		}
	}
	return false;
}
int sysfs_write_byte_from_dword_seq(char *filename, uint32_t *value, uint32_t size)
{
	struct file *fp = NULL;
	int i = 0;
	char buf[9];
	ssize_t ret;

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("open (%s) failed!\n",filename);
		return -1;
	}

	for(i = 0; i < size; i++){
		sprintf(buf, "%02x", (value[i]));
		buf[2] = '\n';

		ret = __kernel_write(fp, buf, 3, &fp->f_pos);
		if(ret < 3)
		{
			pr_err("write failed!\n");
			/* Close file */
			filp_close(fp, NULL);
			return -2;
		}
	}

	/* close file */
	filp_close(fp, NULL);
	return 0;
}

