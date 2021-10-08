//byte0
#define GP_VIB_REPORT_ID 0x05

//byte2
#define GP_PWM_12K (0x0 << 6)
#define GP_PWM_24K (0x1 << 6)
#define GP_PWM_48K (0x2 << 6)

#define GP_UNIT_1MS 	(0x0 << 4)
#define GP_UNIT_100MS 	(0x1 << 4)
#define GP_UNIT_1S 		(0x2 << 4)
#define GP_UNIT_TIMES 	(0x3 << 4)

#define GP_RTP_MODE  0x0
#define GP_MEM_MODE  0x1
#define GP_TRIG_MODE  0x2
#define GP_A2V_MODE  0x3

#define RTP_MODE_HEADER_LEN 13

u8 g_gb_gain = 0;
extern void vib_hid_pw_normal(void);
extern void vib_hid_pw_fullon(void);
extern int vib_hid_w_report(unsigned int report_number, u8 *buf, size_t len, unsigned char report_type);
extern u8 g_gamepad;

extern int gVibDebugLog;

int aw8697_hid_rtp(u8 *data, size_t len)
{
	int ret = 0;
	u32 cnt = 0;
	unsigned int buf_len = 0;
	u8 *buf;
	printk("%s: len=%d\n",__func__,len);
	buf = kzalloc(0xFF, GFP_KERNEL);

	vib_hid_pw_fullon();
	while(len-cnt){
		memset(buf,0,0xFF);
		buf[0]=GP_VIB_REPORT_ID;
		buf[2]=GP_PWM_24K | GP_RTP_MODE;

	if ((len - cnt) > 0xff-RTP_MODE_HEADER_LEN)
		buf_len = 0xff-RTP_MODE_HEADER_LEN;
	else
		buf_len= len - cnt;

		buf[1]= buf_len+RTP_MODE_HEADER_LEN;
		buf[3]=(cnt >> 24) & 0xff;
		buf[4]=(cnt >> 16) & 0xff;
		buf[5]=(cnt >> 8) & 0xff;
		buf[6]= cnt & 0xff;
		buf[7]= g_gb_gain;
		buf[8]= g_gb_gain;
		buf[9]=(len >> 24) & 0xff;
		buf[10]=(len >> 16) & 0xff;
		buf[11]=(len >> 8) & 0xff;
		buf[12]=len & 0xff;
		memcpy(buf+RTP_MODE_HEADER_LEN, data+cnt, buf_len);

		if((gVibDebugLog & 0x0008)== 0x0008){
			printk("%s: buf_len=%d cnt=%d [0x%02x %d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x-0x%02x 0x%02x-0x%02x 0x%02x 0x%02x 0x%02x]\n",
			__func__, buf_len, cnt, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
			buf[7], buf[8], buf[9], buf[10], buf[11], buf[12]);
		}

		ret=vib_hid_w_report(GP_VIB_REPORT_ID, buf, buf_len+RTP_MODE_HEADER_LEN, 1);
		if (ret < 0){
			printk("%s : report fail (%d)\n", __func__, ret);
			break;
		}
		cnt=cnt+buf_len;
	}
	vib_hid_pw_normal();

	kfree(buf);
	return ret;
}

int aw8697_hid_mem(u8 num)
{
	int ret = 0;
	u8 *buf;
	if((gVibDebugLog & 0x0008)== 0x0008)
		printk("%s: num=%d gain=0x%02x(%d)\n", __func__, num, g_gb_gain, g_gb_gain);

	buf = kzalloc(0xFF, GFP_KERNEL);
	memset(buf,0,0xFF);
		buf[0]=GP_VIB_REPORT_ID;
		buf[2]=GP_PWM_24K | GP_MEM_MODE;

		buf[1]= 9;
		buf[3]= num;
		buf[4]= num;
		buf[5]= g_gb_gain;
		buf[7]= g_gb_gain;
		vib_hid_pw_fullon();
		ret=vib_hid_w_report(GP_VIB_REPORT_ID, buf, 9, 1);
		vib_hid_pw_normal();
		if(ret < 0)
			printk("%s : report fail (%d)\n", __func__, ret);

	kfree(buf);
	return ret;
}

int aw8697_hid_a2v(u8 num, u8 gain, u8 loop)
{
	int ret = 0;
	u8 *buf;
	//if((gVibDebugLog & 0x0008)== 0x0008)
		printk("%s: num=%d gain=%d loop=%d\n", __func__, num, gain, loop);

	buf = kzalloc(0xFF, GFP_KERNEL);
	memset(buf,0,0xFF);
		buf[0]=GP_VIB_REPORT_ID;
		buf[2]=GP_PWM_24K | GP_UNIT_TIMES | GP_A2V_MODE;

		buf[1]= 9;
		buf[3]= num;
		buf[4]= num;
		buf[5]= gain;
		buf[7]= gain;
		buf[6]= loop;
		buf[8]= loop;

		if((gVibDebugLog & 0x0008)== 0x0008){
			printk("%s: [0x%02x %d 0x%02x - 0x%02x 0x%02x - 0x%02x 0x%02x 0x%02x 0x%02x]\n",
			__func__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8] );
		}

		vib_hid_pw_fullon();
		ret=vib_hid_w_report(GP_VIB_REPORT_ID, buf, 9, 1);
		vib_hid_pw_normal();
		if(ret < 0)
			printk("%s : report fail (%d)\n", __func__, ret);

	kfree(buf);
	return ret;
}
