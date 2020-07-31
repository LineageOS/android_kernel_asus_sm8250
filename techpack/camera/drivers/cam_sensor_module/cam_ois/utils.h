#ifndef ASUS_OIS_UTILS_H
#define ASUS_OIS_UTILS_H
#include <linux/types.h>
#include <linux/time.h>

#include <cam_sensor_cmn_header.h>


void swap_word_data(uint16_t* register_data);
int format_hex_string(char *output_buf,int len, uint8_t* data,int count);

void delay_ms(uint32_t time);
void delay_us(uint32_t time);

int64_t diff_time_us(struct timeval *t1, struct timeval *t2);

int32_t get_file_size(const char *filename, uint64_t* size);
int read_file_into_buffer(const char *filename, uint8_t* data, uint32_t size);

int sysfs_read_uint8(char *filename, uint8_t *value);
int sysfs_read_byte_seq(char *filename, uint8_t *value, uint32_t size);
int sysfs_read_char_seq(char *filename, uint8_t *value, uint32_t size);
int sysfs_read_word_seq(char *filename, uint16_t *value, uint32_t size);
int sysfs_read_dword_seq(char *filename, uint32_t *value, uint32_t size);
int sysfs_write_byte_seq(char *filename, uint8_t *value, uint32_t size);
int sysfs_write_word_seq(char *filename, uint16_t *value, uint32_t size);
int sysfs_write_word_seq_change_line(char *filename, uint16_t *value, uint32_t size,uint32_t number);
int sysfs_write_dword_seq_change_line(char *filename, uint32_t *value, uint32_t size,uint32_t number, bool shift);

bool i2c_setting_contain_address(struct cam_sensor_i2c_reg_setting * setting, uint32_t addr, uint32_t* data, uint16_t* index);
#endif
