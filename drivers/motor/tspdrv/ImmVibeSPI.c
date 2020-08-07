/*
** =============================================================================
**
** File: ImmVibeSPI.c
**
** Description:
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
**
** Copyright (c) 2008-2017 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
**
** =============================================================================
*/

#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <dt-bindings/pinctrl/qcom,pmic-gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>

#include "dw791x_drv.h"

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** use two bus 
* 1: two bus
* 0: 1 bus
*/
//#define TWOBUS 1

/*
** i2c bus
*/

//#define DEVICE_BUS_1  5
//#define DEVICE_BUS_2  7

/*
** i2c device address for one bus, but two addresses
*/
#define DEVICE_ADDR_1 0x59
#define DEVICE_ADDR_2 0x5A

/*
** This SPI supports 2 actuators.
** Don't forget to update the code (switch statements) in this file if you change NUM_ACTUATORS.
*/
//#error Please Set NUM_ACTUATORS to the number of actuators supported by this SPI.
#define NUM_ACTUATORS       1

/*
** Include optional ImmVibeSPI_Device_GetStatus in driver.
*/
#define IMMVIBESPI_DEVICE_GETSTATUS_SUPPORT

/*
** Name displayed in TouchSense API and tools
*/
#define DEVICE_NAME "ROG3"

/*
** Manage amplifier buffer using ImmVibeSPI_ForceOut_BufferFull
*/
//#define IMMVIBESPI_USE_BUFFERFULL

/*
** Number of sample periods to prebuffer before sending
*/
//#define PREBUFFER_SAMPLE_PERIODS 3

/*
** Higher values buffer more future samples.
*/
//#define DW791X_FIFOFULL_TARGET (PREBUFFER_SAMPLE_PERIODS * VIBE_OUTPUT_SAMPLE_SIZE)
/* sample rate: 48KHz, TouchSense period=5ms, 
 * 48Khz*5ms=48k*5m=240 bytes
 * 1.5*240=360
 * FIFO sisze=DW791X_FIFOFULL_TARGET+ sample_per_period (or 360+240)=600
 */
#define DW791X_FIFOFULL_TARGET 360
#define DW791X_FIFO_LEVEL 600

/*
** Upscale immvibed data because it supports up to 12kHz (and 24kHz is less audible)
* ?????
*/
#define UPSCALE_12_TO_24khz
#define UPSCALE_12_TO_48khz

//#define NAK_RESEND_ATTEMPT 3

/*
 * waveform bin file
 */ 
#define FW_FILE "dwa_hpitic_mem_wave.bin"
#define RTP_FW_FILE "dwa_haptic_rpt_wave.bin"
#define RTP_03_FW_FILE "03_L-RTP_Layer_96000Byte_48Khz_2000.00ms.bin"
#define RTP_07_FW_FILE "07_L-RTP_Layer_26412Byte_48Khz_550.25ms.bin"
#define RTP_06_FW_FILE "06_L-RTP_Layer_95940Byte_48Khz_1998.75ms.bin"
#define RTP_08_FW_FILE "08_L-RTP_Layer_25800Byte_48Khz_537.50ms.bin"
#define RTP_01_FW_FILE "01_L-RTP_Layer_640Byte_48Khz_13.33ms.txt.bin"
#define RTP_GUN1_FW_FILE "arctic50_outdoor_01_F0_150Hz.bin"
#define RTP_GUN2_FW_FILE "type25_outdoor_loop_01_F0_150Hz.bin"
#define RTP_GUN3_FW_FILE "wpn_870mcs_outdoor_02_F0_150Hz.bin"
#define RTP_GUN4_FW_FILE "wpn_mw11_outdoor_02_F0_150Hz.bin"
#define RTP_GUN5_FW_FILE "AK47_loop_outdoor_01_F0_150Hz.bin"
#define RTP_BOT_VIB_MAX_POWER_FW_FILE "bot_vib_max_power.bin"
#define RTP_BOT_VIB_RATED_POWER_FW_FILE "bot_vib_rated_power.bin"
#define RTP_BOT_VIB_SWEEP_RATED_POWER_FW_FILE "bot_vib_sweep_rated_power.bin"
#define RTP_TOP_VIB_MAX_POWER_FW_FILE "top_vib_max_power.bin"
#define RTP_TOP_VIB_RATED_POWER_FW_FILE "top_vib_rated_power.bin"
#define RTP_TOP_VIB_SWEEP_RATED_POWER_FW_FILE "top_vib_sweep_rated_power.bin"
#define RTP_BOT_VIB_VD_TEST_500_FW_FILE "RTP_Layer_BOT_VIB_500.00ms.bin"
#define RTP_BOT_VIB_VD_TEST_1500_FW_FILE "RTP_Layer_BOT_VIB_1500.00ms.bin"
#define RTP_TOP_VIB_VD_TEST_500_FW_FILE "RTP_Layer_TOP_VIB_501.50ms.bin"
#define RTP_TOP_VIB_VD_TEST_1500_FW_FILE "RTP_Layer_TOP_VIB_1504.50ms.bin"
#define RTP_BOT_VIB_VD_TEST_6_FW_FILE "RTP_Layer_BOT_VIB_6.67ms.bin"
#define RTP_TOP_VIB_VD_TEST_4_FW_FILE "RTP_Layer_TOP_VIB_4.25ms.bin"
#define RTP_DEFAULT_POWER_FW_FILE "defalut_power.bin"
#define RTP_TEST_FW_FILE "type25_outdoor_loop_01_F0_150Hz.bin"

/*
 * Default VD_CLAMP
 */ 
#if 0
#define DEFAULT_TOP_VD_CLAMP	5500 //5.5V, 0832
#define DEFAULT_BOT_VD_CLAMP	8500 //8.5V, gamma
#else
#define DEFAULT_TOP_VD_CLAMP	4500 //4.5V, 0832
//#define DEFAULT_BOT_VD_CLAMP	3500 //3.5V, gamma
#define DEFAULT_BOT_VD_CLAMP	5500 //5.5V, gamma
#endif
#define MAX_TOP_VD_CLAMP	5000 //5.0V, 0832
#define MAX_BOT_VD_CLAMP	9000 //9.0V, gamma

struct dw791x_priv {
//Leds
	struct led_classdev	cdev;
	int activate;
	int duration;
	//int state;
	//int rtp_input_node;
	//int mode;
	//int set_sequencer;
	//int scale;
	//int ctrl_loop;
	//int lp_trigger_effect;
	struct hrtimer led_vib_timer;
//leds end
//common
	//struct work_struct work;
	struct mutex dev_lock;
	//struct hrtimer timer;
//common end
	char dev_name[8];
	u8 rtp_input;
	u8 mem_input;
	u8 play_back;
	u8 fifo_level;
	u8 fifo_addrh;
	u8 fifo_addrl;
	u8 play_mode;
//porting immersion dw7912
    unsigned char chipid;
    unsigned char i2c_addr;
    ktime_t time;
    int buffering;
    s64 position;
#if 0    
#ifdef UPSCALE_12_TO_24khz
    VibeInt8 buffer[1 + DW791X_FIFOFULL_TARGET*2];
#else
    VibeInt8 buffer[1 + DW791X_FIFOFULL_TARGET];
#endif
#endif
//end	
	int enable;
	//gpio
	struct pinctrl *pinctrl;
	struct pinctrl_state *top_vib_pin_sta_active;
	struct pinctrl_state *top_vib_pin_sta_suspend;
	struct pinctrl_state *bot_vib_pin_sta_active;
	struct pinctrl_state *bot_vib_pin_sta_suspend;
	int top_vib_irq_gpio;
	int bot_vib_irq_gpio;
	int enable_gpio;
	int trigger_gpio;
	int boot_vib_gpio;
	int top_trigger3_gpio;
	int bot_trigger3_gpio;
	u32 top_vib_irq_num;
	u32 bot_vib_irq_num;
	//end
	u32 device;
	u32 checksum;
	u32 version;
	const struct firmware *fw;
	//struct hrtimer hr_timer;
	struct i2c_client *i2c;
	struct autovib_reg autovib;
//PMIC GPIO
    struct device           *dev;
//end
	//i2c status
	bool i2c_status;
	//
	//rtp mode play control
	volatile bool bRTP_Play_Break;
	struct workqueue_struct *rtp_workqueue;
	struct work_struct rtp_work;
	int bin_num, repeat;
	char fwName[100];
	//trigger waveform
	int trigger_r, break_r, trigger_f, break_f;
	int trigger_tap_level, trigger_slider_level;
};

//PMIC PM8150 & PM8150B GPIO
//#define DW791X_PM8150_GPIO7_ENABLE	"gpio7_vib_trig1_dw7914_on"
//#define DW791X_PM8150_GPIO7_DISABLE	"gpio7_vib_trig1_dw7914_off"
//#define DW791X_PM8150B_GPIO1_ENABLE	"gpio1_en_on"
static bool gPmic_Gpio_Config=false;
static int gTrigger_gpio=-1;
//end

//I2C DMA buffer
static u8 *dma_buf_resampling=NULL;
static u8 *dma_buf_ivt=NULL;
//end

/*channel: 
 * 0x00: none
 * 0x01: right, BOT
 * 0x02: left, TOP
 * 0x03: left and right, TOP and BOT
 */
static int gChannel=0x01;
//end

//Vibrator interface creation
static bool gVibIF=false;
//end

//Vibrator debug interface
/*gVibDebugLog: 
 * 0x0000: none
 * 0x0001: irq
 * 0x0002: kernel
 * 0x0004: immersion
 * 0x0008: nForce data
 * 0x0010: store node debug
 * 0x0020: vibratorcontrol debug
 * 0x0100: open vd_clamp limitation
 */
static int gVibDebugLog=0;//0;
//end

//global i2c bus status
//static struct dw791x_priv *dw791x_devs[NUM_ACTUATORS]={NULL, NULL};
static struct dw791x_priv *dw791x_devs[NUM_ACTUATORS]={NULL};
//#if 0
//#define BOT_IS_INDEX_0	1
//#if BOT_IS_INDEX_0
//if bot_vib is index 0
//static int gActuatorIndex=2;
//#else
//if top_vib is index 0
//static int gActuatorIndex=-1;
//#endif
//#endif

//#define BOT_IS_INDEX_0	1
static int gActuatorIndex=0;

static int gBus[2]={-1, -1};
static int gAddr[2]={0x00, 0x00};
//end

/*for factory test
 * 0x01: bot_vib enabled only
 * 0x02: top_vib enabled only
 * 0x03: top_vib and bot_vib all enabled
 */
u8 bEnable_Vib=0x03;
EXPORT_SYMBOL(bEnable_Vib);
//end
#ifdef INT_EVENT_OPT
//gpio configuration
#define TOP_VIB_PINCTRL_STATE_ACTIVE    "top_vib_int_active"
#define TOP_VIB_PINCTRL_STATE_SUSPEND   "top_vib_int_suspend"
#define BOT_VIB_PINCTRL_STATE_ACTIVE    "bot_vib_int_active"
#define BOT_VIB_PINCTRL_STATE_SUSPEND   "bot_vib_int_suspend"
static int dw791x_pinctrl_init(struct i2c_client *i2c, struct dw791x_priv *pdev);
static int init_irq_rtp(struct dw791x_priv *pdev);
#endif
//end

//rtp play control
static volatile bool bRTP_Play_Break=false;
//


/*resample control
 * 1: don't resample
 * 2: 2 times
 * 4: 12KHz to 48KHz
 * 6: 8KHz to 48KHz
 */
static int gResample=4;
//end

/*pwm control
 * 0x00: 24KHz
 * 0x01: 48KHz
 * 0x02: 96KHz
 * 0x03: 12KHz
 */
static int gPWM=0x01;
//end

static bool trigger2_init = 0;
static int gTrigger_Tap_Level =2; //trigger2_tap, default value
static int gTrigger_Slider_Level =2; //trigger2_slider, default value
static int gTrigger1_3_Level =2; //trigger1 & trigger3 default level
static int getActuatorIndex(struct i2c_client* client);
static int dw791x_device_init(struct i2c_client *client);
static int i2c_recv_buf(struct i2c_client *i2c, unsigned char reg, unsigned char *buf, int count);
static int dw791x_PWM_get(struct i2c_client *client);
static void dw791x_PWM_set(struct i2c_client *client, u32 vol);
static void vibrator_set_trigger(int value);
void set_trigger1_level(int channel, int level);
void set_trigger2_level(int channel, int level);
void set_trigger3_level(int channel, int level);
void mem_play(struct dw791x_priv *dw791x, int waveform_num);
//VibeStatus I2CWrite(unsigned char address, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);
VibeStatus I2CWriteWithResendOnError_DW7914(struct dw791x_priv *dw791x, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer);
//static int dw791x_seq_read(struct i2c_client *client, u32 addr, u32 ram_addr, u32 ram_bit, u8* data, u32 size);
/*
** I2C Driver
*/
static int dw791x_probe(struct i2c_client* client, const struct i2c_device_id* id);
static void dw791x_shutdown(struct i2c_client* client);
static int dw791x_remove(struct i2c_client* client);
//===================
#define MAX_CHANNEL 1
bool check_channel(int ch)
{
	if ((ch < 0) || (ch > MAX_CHANNEL))
		return 0;
	else 
		return 1;
}
int channel_to_index(int ch)
{
	int index=-1;
	switch(ch)
	{
	case 1:
		index=0;
		break;
	default:
		DbgOut((DBL_ERROR, "not support channel=%d\n", ch));
	}
	return index;
}
//===================
/*
** DW791x Register Dump Functions
*/
#if 1
static void dw791x_dump_registers(struct dw791x_priv *dev)
{
    int i = 0;
    char buf = 0;
    char regs[] = {
        DW791x_PID,
        DW7914_STATUS0,
        DW7914_INTEN,
        DW7914_MODE,
        DW7914_PWM,
        DW7914_BST_OUTPUT,
        DW7914_BST_MODE,
        DW7914_VMH,
        DW7914_VD_CLAMP,
        DW7914_BST_OPTION, 
        DW7914_FIFO_LEVEL_SETH, 
        DW7914_FIFO_LEVEL, 
        DW7914_FIFO_ADDRH, 
        DW7914_FIFO_ADDRL,
        DW7914_TRIG_DET_EN
    };

    if (!dev || !dev->i2c) return;

    for (i = 0; i < sizeof(regs); i++) {
        i2c_recv_buf(dev->i2c, regs[i], &buf, sizeof(buf));
        DbgOut((DBL_INFO, "dw7914: reg[%02d:0x%02x]=0x%02x\n", i, regs[i], buf));
    }
}
#endif
/*#if 0
static void dw791x_dump_all_registers(struct dw791x_priv *dev)
{
    int i = 0;
    char buf = 0;
    if (!dev || !dev->i2c) return;

    for (i = 0; i < 0x5f; i++) {
        i2c_recv_buf(dev->i2c, i, &buf, sizeof(buf));
        DbgOut((DBL_INFO, "dw7914: reg[%02d:0x%02x]=0x%02x\n", i, i, buf));
    }
}
#endif*/
static struct of_device_id dw791x_i2c_dt_ids[] = {
//	{ .compatible = "dwanatech,dw791x_top"},
	{ .compatible = "dwanatech,dw791x"},
	{ }
};

static const struct i2c_device_id dw791x_drv_id[] = {
	{"dw791x", 0},
	{}
};

static struct i2c_driver dw791x_i2c_driver = {
		.probe = dw791x_probe,
		.remove = dw791x_remove,
		.id_table = dw791x_drv_id,
		.shutdown = dw791x_shutdown,
		.driver = {
			.name = "dw791x-codec",
			.owner = THIS_MODULE,

#ifdef CONFIG_OF
			.of_match_table = of_match_ptr(dw791x_i2c_dt_ids),
#endif

		},
};

/* =====================================================================================
function : define the play concept on the dw791x.
descript : auto detected devices model
version  : 1.0
release  : 2018.02.28
====================================================================================== */
int dw791x_play_mode_sel(struct i2c_client* client, int play_mode, u32 set1, u32 set2, u32 set3)
{
	int ret = 0;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];	
	//u8 rx[2];

    DbgOut((DBL_INFO, "%s: %s:%d\n, %d, %d, %d", __func__, (getActuatorIndex(client)==0)?"Bot":"Top",play_mode, set1, set2, set3));
    if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s: %d , vibrator %d i2c fail\n", __func__, play_mode, getActuatorIndex(client)));
		return -1;
	}
	dw791x_byte_write(client, DW7914_PWM, gPWM);//48Khz
	//video demo
	if(play_mode>100){
		if(gVibDebugLog&0x0010)
			DbgOut((DBL_INFO, "play_mode=%02d\n", play_mode));

		dw791x_bst_option(client, 0, 0, 0);	//default

		if((getActuatorIndex(client)==0) && (play_mode==114)){//top
				sprintf(dw791x->fwName, "14_0832.bin");
		}
		else
			sprintf(dw791x->fwName, "%d.bin", (play_mode-100));

		//stop old vibration and then play new one.
		if(dw791x->bRTP_Play_Break){
			if(gVibDebugLog&0x0010)
				DbgOut((DBL_ERROR, "%s:(2) bin_num=%d, break=%d, cancel_work_sync!\n", __func__, (play_mode-100), dw791x->bRTP_Play_Break));
			cancel_work_sync(&dw791x->rtp_work);
			dw791x->bRTP_Play_Break=false;
		}
		//end
		dw791x->bin_num=(play_mode-100);
		dw791x->repeat=1;
		if(gVibDebugLog&0x0010)//store node debug
			DbgOut((DBL_INFO, "%s:(3) bin_num=%d, break=%d\n", __func__, dw791x->bin_num, dw791x->bRTP_Play_Break));
		queue_work(dw791x->rtp_workqueue, &dw791x->rtp_work);
	}
	//end
	else{
		switch(play_mode) {
			case 0: // trig mode set
				dw7914_trigctrl_set(client, 0, 0);
				break;
			case 3:	// request firmware function prove : page1 point 2
				if(request_firmware(&dw791x->fw, FW_FILE, &dw791x->i2c->dev) == 0) {
					//DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", FW_FILE, dw791x->fw->size));
					request_transfer_mem_wave(client, 0, 1, (u8*)dw791x->fw->data, dw791x->fw->size);
				} release_firmware(dw791x->fw);

	#ifdef WAVE_PCM1 // wave1 play
				dw791x_byte_write(client, 0x0B, 0x01);
				dw791x_byte_write(client, DW7914_WAVQ1, 1);
				dw791x_byte_write(client, dw791x->play_back, DRV_PLAY);
	#else //wave2 play
				dw791x_byte_write(client, 0x0B, 0x05); //memory + auto brake
				dw791x_byte_write(client, DW7914_WAVQ1, 2);
				dw791x_byte_write(client, 0x1D, 0x03); //brake1 assign with mem3
				dw791x_byte_write(client, 0x1E, 0x28); //VD gain & cycle assign
				dw791x_byte_write(client, dw791x->play_back, DRV_PLAY);
	#endif
				break;

			case 4:	// request firmware function prove : write wave data point all
				if(request_firmware(&dw791x->fw, FW_FILE, &dw791x->i2c->dev) == 0) {
					//DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", FW_FILE, dw791x->fw->size));
					request_transfer_mem_wave(client, 1, 0, (u8*)dw791x->fw->data, dw791x->fw->size);
				} release_firmware(dw791x->fw);								
				dw791x_byte_write(client, DW7914_WAVQ1, 1);
				dw791x_byte_write(client, dw791x->play_back, DRV_PLAY);
				break;

			case 5:	// auto traking & brake funtion prove	
				dw7914_autovib_set(client);
				dw791x_byte_write(client, dw791x->play_back, DRV_PLAY);
				break;

			case 6:	// checksum prove
				dw7914_checksum(client, 0, 0);	// mode 0, page 0
				break;

			case 7: // device status read
				ret = dw791x_byte_read(client, DW791x_STATUS);
				//DbgOut((DBL_INFO, "Device status : %x\n", ret));
				break;
			case 10: // set RTP mode
				dw791x_byte_write(client, dw791x->play_mode, RTP);
				break;
			case 11: // set MEM mode
				dw791x_byte_write(client, dw791x->play_mode, MEM);
				break;
			case 12:	// long-rtp function prove: demo1
				dw791x_bst_option(client, 0, 0, 0);	//default
				if(request_firmware(&dw791x->fw, RTP_03_FW_FILE, &dw791x->i2c->dev) == 0) {
					DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", RTP_03_FW_FILE, dw791x->fw->size));
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 13:	// long-rtp function prove: demo1
				dw791x_bst_option(client, 0, 0, 0);	//default
				if(request_firmware(&dw791x->fw, RTP_07_FW_FILE, &dw791x->i2c->dev) == 0) {
					DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", RTP_07_FW_FILE, dw791x->fw->size));
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 14:	// long-rtp function prove: demo1
				dw791x_bst_option(client, 0, 0, 0);	//default
				if(request_firmware(&dw791x->fw, RTP_06_FW_FILE, &dw791x->i2c->dev) == 0) {
					DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", RTP_06_FW_FILE, dw791x->fw->size));
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 15:	// long-rtp function prove: demo1
				dw791x_bst_option(client, 0, 0, 0);	//default
				if(request_firmware(&dw791x->fw, RTP_08_FW_FILE, &dw791x->i2c->dev) == 0) {
					DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", RTP_08_FW_FILE, dw791x->fw->size));
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 16:	// factory default power
				dw791x_bst_option(client, 0, 0, 0);	//default
				if(request_firmware(&dw791x->fw, RTP_GUN1_FW_FILE, &dw791x->i2c->dev) == 0) {
					DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", RTP_GUN1_FW_FILE, dw791x->fw->size));
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 17:	// long-rtp function prove, gun2
				dw791x_bst_option(client, 0, 0, 0);	//default
				if(request_firmware(&dw791x->fw, RTP_GUN2_FW_FILE, &dw791x->i2c->dev) == 0) {
					DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", RTP_GUN2_FW_FILE, dw791x->fw->size));
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 18:	// long-rtp function prove, gun3
				dw791x_bst_option(client, 0, 0, 0);	//default
				if(request_firmware(&dw791x->fw, RTP_GUN3_FW_FILE, &dw791x->i2c->dev) == 0) {
					DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", RTP_GUN3_FW_FILE, dw791x->fw->size));
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 19:	// long-rtp function prove, gun4
				dw791x_bst_option(client, 0, 0, 0);	//default
				if(request_firmware(&dw791x->fw, RTP_GUN4_FW_FILE, &dw791x->i2c->dev) == 0) {
					DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", RTP_GUN4_FW_FILE, dw791x->fw->size));
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 20:	// long-rtp function prove, gun5
				dw791x_bst_option(client, 0, 0, 0);	//default
				if(request_firmware(&dw791x->fw, RTP_GUN5_FW_FILE, &dw791x->i2c->dev) == 0) {
					DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld\n", RTP_GUN5_FW_FILE, dw791x->fw->size));
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 21: // init for trigger, wave2, only rising
				dw791x_byte_write(client, 0x03, 0x41); // trig priority, trig2 master, trig1 as pulse trigger input
				dw791x_byte_write(client, 0x0B, 0x01); // mem mode 
				if(getActuatorIndex(client)==0)//gamma: trigger2,  trigger tap(mem1), air trigger
				{
					dw791x->trigger_r=0x01;
					dw791x->break_r=0x00;
					dw791x->trigger_f=0x01;
					dw791x->break_f=0x00;
				}
				else //0832: trigger2, trigger slider(mem1, 2, 3, 4, 5)
				{
					#if 1
					dw791x->trigger_r=0x01;
					dw791x->break_r=0x00;
					dw791x->trigger_f=0x00;
					dw791x->break_f=0x00;
					#endif
					#if 0
					dw791x->trigger_r=0x02;
					dw791x->break_r=0x00;
					dw791x->trigger_f=0x00;
					dw791x->break_f=0x00;
					#endif
					#if 0
					dw791x->trigger_r=0x03;
					dw791x->break_r=0x00;
					dw791x->trigger_f=0x00;
					dw791x->break_f=0x00;
					#endif
					#if 0
					dw791x->trigger_r=0x04;
					dw791x->break_r=0x00;
					dw791x->trigger_f=0x00;
					dw791x->break_f=0x00;
					#endif
					#if 0
					dw791x->trigger_r=0x05;
					dw791x->break_r=0x00;
					dw791x->trigger_f=0x00;
					dw791x->break_f=0x00;
					#endif
				}
				dw791x_byte_write(client, 0x26, dw791x->trigger_r); // trig2_r, mem2
				dw791x_byte_write(client, 0x28, dw791x->break_r); // brake 1 assign
				dw791x_byte_write(client, 0x29, dw791x->trigger_f); // trig2_r, mem2
				dw791x_byte_write(client, 0x2B, dw791x->break_f); // brake 1 assign
				dw791x_byte_write(client, 0x2C, 0x28); // VD gain & cycle
				dw791x_byte_write(client, 0x45, 0xdd); // TRIG2_R_EN, TRIG2_F_EN
				dw791x->trigger_tap_level=gTrigger_Tap_Level;
				dw791x->trigger_slider_level=gTrigger_Slider_Level;
				if(getActuatorIndex(client)==0)//Tap
				{
					//gamma
					if(gTrigger_Tap_Level==1)
					{
						//DbgOut((DBL_INFO, "write tap_gamma_h1_1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)tap_gamma_h1_1,5);
					}
					else if(gTrigger_Tap_Level==3){
						//DbgOut((DBL_INFO, "write tap_gamma_h1_3 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)tap_gamma_h1_3,5);
					}
					else{
						//DbgOut((DBL_INFO, "write tap_gamma_h1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)tap_gamma_h1,5);
					}
					//DbgOut((DBL_INFO, "write tap_gamma_pcm1 data\n"));
					dw791x_seq_write(client, dw791x->mem_input,0x027C,RAM_ADDR16,(u8*)tap_gamma_pcm1,sizeof(tap_gamma_pcm1));
				}

				break;
			case 23:
				dw791x_byte_write(client, 0x03, 0x41); // trig priority, trig2 master, trig1 as pulse trigger input
				dw791x_byte_write(client, 0x0B, 0x01); // mem mode + auto_brake
				//0832: trigger3, trigger slider(mem1, 2, 3, 4, 5)
				dw791x_byte_write(client, 0x2d, 0x01); // trig3_r, typing_0832_h4, mem10(0x0A) 46(0x2E)
				dw791x_byte_write(client, 0x2f, 0x00); // brake 1 assign, typing_0832_h5, mem11(00B), 51(0x33)
				dw791x_byte_write(client, 0x30, 0x00); // trig3_f, mem2
				dw791x_byte_write(client, 0x32, 0x00); // brake 1 assign
				dw791x_byte_write(client, 0x33, 0x08); // VD gain & cycle
				//use go bit to play
				dw791x_byte_write(client, 0x45, 0xdd); // TRIG_DET_EN			
				{
					//0832
					if(gTrigger1_3_Level==1){
						//DbgOut((DBL_INFO, "write slider_0832_h1_1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)slider_0832_h1_1,5);
						//DbgOut((DBL_INFO, "write slider_0832_h2_1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)slider_0832_h2_1,5);
						//DbgOut((DBL_INFO, "write slider_0832_h3_1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)slider_0832_h3_1,5);
						//DbgOut((DBL_INFO, "write slider_0832_h4_1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)slider_0832_h4_1,5);
						//DbgOut((DBL_INFO, "write slider_0832_h5_1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)slider_0832_h5_1,5);
					}
					else if(gTrigger1_3_Level==3){
						//DbgOut((DBL_INFO, "write slider_0832_h1_3 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)slider_0832_h1_3,5);
						//DbgOut((DBL_INFO, "write slider_0832_h2_3 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)slider_0832_h2_3,5);
						//DbgOut((DBL_INFO, "write slider_0832_h3_3 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)slider_0832_h3_3,5);
						//DbgOut((DBL_INFO, "write slider_0832_h4_3 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)slider_0832_h4_3,5);
						//DbgOut((DBL_INFO, "write slider_0832_h5_3 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)slider_0832_h5_3,5);
					}
					else{
						//DbgOut((DBL_INFO, "write slider_0832_h1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)slider_0832_h1,5);
						//DbgOut((DBL_INFO, "write slider_0832_h2 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)slider_0832_h2,5);
						//DbgOut((DBL_INFO, "write slider_0832_h3 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)slider_0832_h3,5);
						//DbgOut((DBL_INFO, "write slider_0832_h4 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)slider_0832_h4,5);
						//DbgOut((DBL_INFO, "write slider_0832_h5 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)slider_0832_h5,5);
					}
				}
				break;
			case 24: // init for trigger1, wave8, only rising
				//trigger1: boot vibration(no usage) and home key(gamma:mem8, 0832:mem9)
				dw791x_byte_write(client, 0x03, 0x61); // trig priority, trig2 master, trig1 as edge trigger input
				dw791x_byte_write(client, 0x0B, 0x01); // mem mode
				//use go bit to play
				dw791x_byte_write(client, 0x45, 0xdd); // TRIG_DET_EN			
				if(getActuatorIndex(client)==0)//gamma
				{
					dw791x_byte_write(client, 0x1f, 0x02); // trig1_r, mem2, 6
					dw791x_byte_write(client, 0x21, 0x00); // brake 1 assign
					dw791x_byte_write(client, 0x22, 0x00); // trig1_f, 00
					dw791x_byte_write(client, 0x24, 0x00); // brake 1 assign
					dw791x_byte_write(client, 0x25, 0x28); // VD gain & cycle
					if(set1==1)
					{
						//DbgOut((DBL_INFO, "write homekey_gamma_h1_1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)homekey_gamma_h1_1,5);
					}
					else if(set1==3){
						//DbgOut((DBL_INFO, "write homekey_gamma_h1_3 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)homekey_gamma_h1_3,5);
					}
					else if(set1==0){
						//DbgOut((DBL_INFO, "write homekey_gamma_h1_0 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)homekey_gamma_h1_0,5);
					}
					else{
						DbgOut((DBL_INFO, "write homekey_gamma_h1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)homekey_gamma_h1,5);
					}
					DbgOut((DBL_INFO, "write homekey_gamma_pcm1 data\n"));
					dw791x_seq_write(client, dw791x->mem_input,0x049F,RAM_ADDR16,(u8*)homekey_gamma_pcm1,sizeof(homekey_gamma_pcm1));			
				}
				break;
			case 25: // init for trigger3, only rising
				dw791x_byte_write(client, 0x03, 0x61); // trig priority, trig2 master, trig1 as edge trigger input
				if(getActuatorIndex(client)==0)//gamma
				{
					dw791x_byte_write(client, 0x0B, 0x01); // mem mode + auto_brake
					dw791x_byte_write(client, 0x2d, 0x05); // trig3_r, mem5, 21(0x15)
					dw791x_byte_write(client, 0x2f, 0x00); // brake 1 assign
					dw791x_byte_write(client, 0x30, 0x00); // trig3_f, mem10
					dw791x_byte_write(client, 0x32, 0x00); // brake 1 assign
					dw791x_byte_write(client, 0x33, 0x28); // VD gain & cycle
					if(set1==1)
					{
						//DbgOut((DBL_INFO, "write typing_gamma_h1_1 header\n"));
						//dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)typing_gamma_h1_1,5);
						//DbgOut((DBL_INFO, "write typing_gamma_h2_1 header\n"));
						//dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)typing_gamma_h2_1,5);
						//DbgOut((DBL_INFO, "write typing_gamma_h3_1 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)typing_gamma_h3_1,5);
					}
					else if(set1==3){
						//DbgOut((DBL_INFO, "write typing_gamma_h1_3 header\n"));
						//dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)typing_gamma_h1_3,5);
						//DbgOut((DBL_INFO, "write typing_gamma_h2_3 header\n"));
						//dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)typing_gamma_h2_3,5);
						//DbgOut((DBL_INFO, "write typing_gamma_h3_3 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)typing_gamma_h3_3,5);
					}
					else if(set1==0){
						//DbgOut((DBL_INFO, "write typing_gamma_h1_0 header\n"));
						//dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)typing_gamma_h1_0,5);
						//DbgOut((DBL_INFO, "write typing_gamma_h2_0 header\n"));
						//dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)typing_gamma_h2_0,5);
						//DbgOut((DBL_INFO, "write typing_gamma_h3_0 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)typing_gamma_h3_0,5);
					}
					else{
						//DbgOut((DBL_INFO, "write typing_gamma_h1 header\n"));
						//dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)typing_gamma_h1,5);
						//DbgOut((DBL_INFO, "write typing_gamma_h2 header\n"));
						//dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)typing_gamma_h2,5);
						DbgOut((DBL_INFO, "write typing_gamma_h3 header\n"));
						dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)typing_gamma_h3,5);
					}
					//DbgOut((DBL_INFO, "write typing_gamma_pcm1 data\n"));
					//dw791x_seq_write(client, dw791x->mem_input,0x0755,RAM_ADDR16,(u8*)typing_gamma_pcm1,sizeof(typing_gamma_pcm1));
					//DbgOut((DBL_INFO, "write typing_gamma_pcm2 data\n"));
					//dw791x_seq_write(client, dw791x->mem_input,0x0978,RAM_ADDR16,(u8*)typing_gamma_pcm2,sizeof(typing_gamma_pcm2));
					//DbgOut((DBL_INFO, "write typing_gamma_pcm3 data\n"));
					dw791x_seq_write(client, dw791x->mem_input,0x0AD8,RAM_ADDR16,(u8*)typing_gamma_pcm3,sizeof(typing_gamma_pcm3));
				}				
				//use go bit to play
				dw791x_byte_write(client, 0x45, 0xdd); // TRIG_DET_EN			
				break;
//for factorty test	
			case 30:	// BOT_VIB Max power
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2800);	//2.80V
				if(request_firmware(&dw791x->fw, RTP_BOT_VIB_MAX_POWER_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 31:	// BOT_VIB Rated power
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2800);	//2.80V
				if(request_firmware(&dw791x->fw, RTP_BOT_VIB_RATED_POWER_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 32:	// BOT_VIB Sweep rated power
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2800);	//2.80V
				if(request_firmware(&dw791x->fw, RTP_BOT_VIB_SWEEP_RATED_POWER_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 33:	// TOP_VIB Max power
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2520);	//2.52V
				if(request_firmware(&dw791x->fw, RTP_TOP_VIB_MAX_POWER_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 34:	// TOP_VIB Rated power
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2520);	//2.52V
				if(request_firmware(&dw791x->fw, RTP_TOP_VIB_RATED_POWER_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 35:	// TOP_VIB Sweep rated power
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2520);	//2.52V
				if(request_firmware(&dw791x->fw, RTP_TOP_VIB_SWEEP_RATED_POWER_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 36:	// BOT_VIB VD test 500ms
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2800);	//2.8V
				if(request_firmware(&dw791x->fw, RTP_BOT_VIB_VD_TEST_500_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 37:	// BOT_VIB VD test 1.5s
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2800);	//2.8V
				if(request_firmware(&dw791x->fw, RTP_BOT_VIB_VD_TEST_1500_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 4);
				} release_firmware(dw791x->fw);
				break;
			case 38:	// TOP_VIB VD test 500ms
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2520);	//2.52V
				if(request_firmware(&dw791x->fw, RTP_TOP_VIB_VD_TEST_500_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 39:	// TOP_VIB VD test 1.5s
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2520);	//2.52V
				if(request_firmware(&dw791x->fw, RTP_TOP_VIB_VD_TEST_1500_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 1);
				} release_firmware(dw791x->fw);
				break;
			case 40:	// BOT_VIB VD test 6.67ms
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2800);	//2.8V
				if(request_firmware(&dw791x->fw, RTP_BOT_VIB_VD_TEST_6_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 10);
				} release_firmware(dw791x->fw);
				break;
			case 41:	// TOP_VIB VD test 4.25ms
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2520);	//2.52V
				if(request_firmware(&dw791x->fw, RTP_TOP_VIB_VD_TEST_4_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 10);
				} release_firmware(dw791x->fw);
				break;
			case 42:	// BOT_VIB VD test 500msx10
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2800);	//2.8V
				if(request_firmware(&dw791x->fw, RTP_BOT_VIB_VD_TEST_500_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 10);
				} release_firmware(dw791x->fw);
				break;
			case 43:	// TOP_VIB VD test 500msx10
				dw791x_bst_option(client, 0, 0, 0);	//default
				//dw791x_vd_clamp_set(client, 2520);	//2.52V
				if(request_firmware(&dw791x->fw, RTP_TOP_VIB_VD_TEST_500_FW_FILE, &dw791x->i2c->dev) == 0) {
					request_transfer_rtp_wave(client, (u8*)dw791x->fw->data, 10);
				} release_firmware(dw791x->fw);
				break;
			case 44:	// Stop play
				//stop vibrating.
				bRTP_Play_Break=true;
				DbgOut((DBL_INFO, "%s Stop play ...\n", __func__));
				dw791x_byte_write(client, dw791x->play_back, DRV_STOP);
				break;
//end
			}
		}
	return ret;
}
//system property: vendor.asus.vibrator_control
/* =====================================================================================
function : special vibrator function for vendor.asus.vibrator_control
descript : 
version  : 1.0
release  : 2019.04.02
====================================================================================== */
static int dw791x_vibrator_control(struct i2c_client* client, int fun, int param)
{
	int ret = 0;
	int index=getActuatorIndex(client);
	struct dw791x_priv *dw791x =dw791x_devs[index];	
	u8 rx[2];

	DbgOut((DBL_INFO, "%s:fun=0x%04x, param=0x%04x\n", __func__, fun, param));
    if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s:0x%04x:0x%04x, vibrator %d i2c fail\n", __func__, fun, param, getActuatorIndex(client)));
		return -1;
	}
	switch(fun) {
		case 0:
			DbgOut((DBL_INFO, "%s:fun=0x%04x, param=0x%04x\n", __func__, fun, param));
			break;
		case 1:	// vd_clamp setting
			//check parameter range for vd_clamp
			if(index==0)//gamma
			{
				if(param<0 || param >MAX_BOT_VD_CLAMP ){
					DbgOut((DBL_INFO, "%s:fun=0x%04x, param=0x%04x, exceed the MAX(%d), set to default (%d)\n", __func__, fun, param, MAX_BOT_VD_CLAMP, MAX_BOT_VD_CLAMP));
					param=MAX_BOT_VD_CLAMP;
				}
			}
			else //0832
			{
				if(param<0 || param >MAX_TOP_VD_CLAMP ){
					DbgOut((DBL_INFO, "%s:fun=0x%04x, param=0x%04x, exceed the MAX(%d), set to default (%d)\n", __func__, fun, param, MAX_TOP_VD_CLAMP, MAX_TOP_VD_CLAMP));
					param=MAX_TOP_VD_CLAMP;
				}
			}
			dw791x_vd_clamp_set(client, param);
			break;		
		case 2:	// vib_enable
			bEnable_Vib = (param & 0x03);//0x01: bot, 0x02: top, 0x03: all ebale, 0x00: all disable
			DbgOut((DBL_INFO, "%s:Immersion:channel %d is enabled\n", __func__, bEnable_Vib));
			break;		
		case 3:	// channel
			gChannel = (param & 0x03);//0x01: bot, 0x02: top, 0x03: all , 0x00: none
			DbgOut((DBL_INFO, "%s:driver:channel %d is selected\n", __func__, gChannel));
			break;
		case 4:	// mem play
		{
			int val=0;
			val=param & 0x000000ff;
			if(val<0 || val>5) val=1;
			DbgOut((DBL_INFO, "MEM mode Play start ...\n"));
            mem_play(dw791x, val);
			rx[0] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS0);
			rx[1] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS1);
			DbgOut((DBL_INFO, "%s:status0=0x%02x, status1=0x%02x!\n", __func__, rx[0], rx[1]));		
			if(rx[0]&0x10)
				DbgOut((DBL_INFO, "%s:process done!\n", __func__));		
		}
			break;
		case 5:	// rtp play
		{
			int num=param & 0x0000ffff;
			//stop old vibration and then play new one.
			if(dw791x->bRTP_Play_Break)
			{
				if(gVibDebugLog&0x0010)//store node debug
					DbgOut((DBL_ERROR, "%s:(2) bin_num=%d, break=%d, cancel_work_sync!\n", __func__, num, dw791x->bRTP_Play_Break));
				cancel_work_sync(&dw791x->rtp_work);
				dw791x->bRTP_Play_Break=false;
			}
			//end
			dw791x->bin_num=num;
			sprintf(dw791x->fwName, "%d.bin", dw791x->bin_num);
			dw791x->repeat=1;
			if(gVibDebugLog&0x0010)//store node debug
				DbgOut((DBL_INFO, "%s:(3) bin_num=%d, break=%d, request_firmware...\n", __func__, dw791x->bin_num, dw791x->bRTP_Play_Break));
			if(request_firmware(&dw791x->fw, dw791x->fwName, &dw791x->i2c->dev) == 0) {
				if(gVibDebugLog&0x0010)//check break status
					DbgOut((DBL_INFO, "%s: (4)fw_name= %s ,bin_num=%d, repeat=%d, break=%d, queuework...\n",__func__, dw791x->fwName, dw791x->bin_num, dw791x->repeat, dw791x->bRTP_Play_Break));
				queue_work(dw791x->rtp_workqueue, &dw791x->rtp_work);
			}
			else
			{
				DbgOut((DBL_INFO, "%s: (4)fw_name= %s ,bin_num=%d, repeat=%d, break=%d, request_firmware failed!\n",__func__, dw791x->fwName, dw791x->bin_num, dw791x->repeat, dw791x->bRTP_Play_Break));
			}
		}
			break;
		case 6:	// trigger play, pm8150 gpio7
			vibrator_set_trigger(param & 0x0f);
			break;
		case 7: // stop play, go bit = 0, bRTP_Play_Break=true;
			dw791x->bRTP_Play_Break=true;
			dw791x_byte_write(client, dw791x->play_back, DRV_STOP);
			DbgOut((DBL_INFO, "%s:driver:channel %d stop playing!\n", __func__, index));
			break;
		case 8: //trigger pin setting, called by init.asus.boot_vibrate.sh
			DbgOut((DBL_INFO, "%s:fun=0x%04x:trigger1 setting\n", __func__, fun));
			dw791x_play_mode_sel(client, 24, gTrigger1_3_Level, 0, 0);//trigger1
			DbgOut((DBL_INFO, "%s:fun=0x%04x:trigger2 setting\n", __func__, fun));
			dw791x_play_mode_sel(client, 21, 0, 0, 0);//trigger2
			DbgOut((DBL_INFO, "%s:fun=0x%04x:trigger3 setting\n", __func__, fun));
			dw791x_play_mode_sel(client, 25, gTrigger1_3_Level, 0, 0);//trigger3
			//test slider trigger effect
			//DbgOut((DBL_INFO, "%s:fun=0x%04x:trigger3 setting: try slider\n", __func__, fun));
			//dw791x_play_mode_sel(client, 23, gTrigger1_3_Level, 0, 0);//trigger3
			//
			break;
		default:
			DbgOut((DBL_INFO, "%s:fun=0x%04x, param=0x%04x\n", __func__, fun, param));
			break;
		}
	return ret;
}
//end
//gpio function
static int init_gpio(struct dw791x_priv *dw791x)
{
	int ret=0;
#if 0

	//enable-pin
	if (gpio_request(dw791x->enable_gpio, "request_enable_gpio")) {
		DbgOut((DBL_INFO, "%s: enable_gpio request faiure: %d\n", __func__, dw791x->enable_gpio));
		return -1;
	}
	ret = gpio_direction_output(dw791x->enable_gpio, 1);
	if(ret < 0) {
		DbgOut((DBL_INFO, "%s: Can't set enable_gpio direction, error %i\n", __func__, ret));
    	gpio_free(dw791x->enable_gpio);
	    return -EINVAL;
	}
	else {
		DbgOut((DBL_INFO, "%s: enable_gpio direction output: %d\n", __func__, dw791x->enable_gpio));
		gpio_set_value(dw791x->enable_gpio, 1);
	}
#endif
	//top_trigger3-pin
#if 0
	if (gpio_request(dw791x->top_trigger3_gpio, "request_top_trigger3_gpio")) {
		DbgOut((DBL_INFO, "%s:top_trigger3_gpio request faiure: %d\n", __func__, dw791x->top_trigger3_gpio));
		return -1;
	}
	ret = gpio_direction_output(dw791x->top_trigger3_gpio, 0);
	if(ret < 0) {
		DbgOut((DBL_INFO, "%s:Can't set top_trigger3_gpio direction, error %i\n", __func__, ret));
    	gpio_free(dw791x->top_trigger3_gpio);
	    return -EINVAL;
	}
	else{
		DbgOut((DBL_INFO, "%s:top_trigger3_gpio direction output: %d\n", __func__, dw791x->top_trigger3_gpio));
		gpio_set_value(dw791x->top_trigger3_gpio, 0);
	 }
#endif
	//bot_trigger3-pin
#if 1
	if (gpio_request(dw791x->bot_trigger3_gpio, "dw791x_trigger_gpio")) {
		DbgOut((DBL_INFO, "%s:dw791x_trigger_gpio request faiure: %d\n", __func__, dw791x->bot_trigger3_gpio));
		return -1;
	}
	ret = gpio_direction_output(dw791x->bot_trigger3_gpio, 0);
	if(ret < 0) {
		DbgOut((DBL_INFO, "%s:Can't set dw791x_trigger_gpio direction, error %i\n", __func__, ret));
    	gpio_free(dw791x->bot_trigger3_gpio);
	    return -EINVAL;
	}
	else{
		DbgOut((DBL_INFO, "%s:dw791x_trigger_gpio direction output: %d\n", __func__, dw791x->bot_trigger3_gpio));
		gpio_set_value(dw791x->bot_trigger3_gpio, 0);
	 }
#endif
	//boot vibration trigger-pin
#if 0
	if (gpio_request(dw791x->boot_vib_gpio, "request_boot_vib_gpio")) {
		DbgOut((DBL_INFO, "%s:boot_vib_gpio request faiure: %d\n", __func__, dw791x->boot_vib_gpio));
		return -1;
	}
	ret = gpio_direction_output(dw791x->boot_vib_gpio, 0);
	if(ret < 0) {
		DbgOut((DBL_INFO, "%s:Can't set boot_vib_gpio direction, error %i\n", __func__, ret));
    	gpio_free(dw791x->boot_vib_gpio);
	    return -EINVAL;
	}
	else{
		DbgOut((DBL_INFO, "%s:boot_vib_gpio direction output: %d\n", __func__, dw791x->boot_vib_gpio));
		gpio_set_value(dw791x->boot_vib_gpio, 0);
	 }
#endif
#if 0//def INT_EVENT_OPT
	//top_vib
	if (dw791x->pinctrl) {
		ret = pinctrl_select_state(dw791x->pinctrl,
					dw791x->top_vib_pin_sta_active);
		if (ret < 0)
			DbgOut((DBL_INFO, "TOP_VIB: Failed to select active pinstate, ret:%d", ret));
	}
	//bot_vib
	if (dw791x->pinctrl) {
		ret = pinctrl_select_state(dw791x->pinctrl,
					dw791x->bot_vib_pin_sta_active);
		if (ret < 0)
			DbgOut((DBL_INFO, "BOT_VIB: Failed to select active pinstate, ret:%d", ret));
	}
#endif	
	return ret;
}
#ifdef INT_EVENT_OPT
/**
 * dw7914_pinctrl_init - Get pinctrl handler and pinctrl_state
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
 
static int dw791x_pinctrl_init(struct i2c_client *i2c, struct dw791x_priv *pdev)
{
	//struct device_node *np = dev->of_node;
	//struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(i2c)];	
	int r = 0;
#if 0	
	struct device *dev = &i2c->dev;
	
	DbgOut((DBL_INFO, "%s start\n", __func__));
	/* get pinctrl handler from of node */
	pdev->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pdev->pinctrl)) {
		DbgOut((DBL_INFO, "Failed to get pinctrl handler\n"));
		return PTR_ERR(pdev->pinctrl);
	}
	//top_vib
	/* active state */
	pdev->top_vib_pin_sta_active = pinctrl_lookup_state(pdev->pinctrl,
				TOP_VIB_PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(pdev->top_vib_pin_sta_active)) {
		r = PTR_ERR(pdev->top_vib_pin_sta_active);
		DbgOut((DBL_INFO, "Failed to get pinctrl state:%s, r:%d\n",
				TOP_VIB_PINCTRL_STATE_ACTIVE, r));
		goto top_vib_exit_pinctrl_put;
	}

	/* suspend state */
	pdev->top_vib_pin_sta_suspend = pinctrl_lookup_state(pdev->pinctrl,
				TOP_VIB_PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(pdev->top_vib_pin_sta_suspend)) {
		r = PTR_ERR(pdev->top_vib_pin_sta_suspend);
		DbgOut((DBL_INFO, "Failed to get pinctrl state:%s, r:%d\n",
				TOP_VIB_PINCTRL_STATE_SUSPEND, r));
		goto top_vib_exit_pinctrl_put;
	}
bot_vib_exit_pinctrl_get:
	//bot_vib
	/* active state */
	pdev->bot_vib_pin_sta_active = pinctrl_lookup_state(pdev->pinctrl,
				BOT_VIB_PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(pdev->bot_vib_pin_sta_active)) {
		r = PTR_ERR(pdev->bot_vib_pin_sta_active);
		DbgOut((DBL_INFO, "Failed to get pinctrl state:%s, r:%d\n",
				BOT_VIB_PINCTRL_STATE_ACTIVE, r));
		goto bot_vib_exit_pinctrl_put;
	}

	/* suspend state */
	pdev->bot_vib_pin_sta_suspend = pinctrl_lookup_state(pdev->pinctrl,
				BOT_VIB_PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(pdev->bot_vib_pin_sta_suspend)) {
		r = PTR_ERR(pdev->bot_vib_pin_sta_suspend);
		DbgOut((DBL_INFO, "Failed to get pinctrl state:%s, r:%d\n",
				BOT_VIB_PINCTRL_STATE_SUSPEND, r));
		goto bot_vib_exit_pinctrl_put;
	}
	DbgOut((DBL_INFO, "goodix_ts_pinctrl_init complete\n"));
	return 0;
top_vib_exit_pinctrl_put:
	devm_pinctrl_put(pdev->pinctrl);
	pdev->pinctrl = NULL;
	pdev->top_vib_pin_sta_active = NULL;
	pdev->top_vib_pin_sta_suspend = NULL; 
	goto bot_vib_exit_pinctrl_get;//try bot_vib
bot_vib_exit_pinctrl_put:
	devm_pinctrl_put(pdev->pinctrl);
	pdev->pinctrl = NULL;
	pdev->bot_vib_pin_sta_active = NULL;
	pdev->bot_vib_pin_sta_suspend = NULL; 
#endif
	return r;
}

static struct workqueue_struct *rtp_workqueue[2];

static void bot_vib_irq_rtp_work(struct work_struct *work)
{
    struct dw791x_priv *dw791x=dw791x_devs[0]; //BOT_VIB 

	u8 rx[2];
	
    if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
	}	
	else{
		rx[0] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS0);
		rx[1] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS1);
		//critical error can't ignore
		if(rx[0]&0x0f){
			DbgOut((DBL_INFO, "%s:VIB:status0:0x%02x, status1:0x%02x\n",__func__, rx[0], rx[1]));
			if(rx[0]&0x01)
				DbgOut((DBL_INFO, "%s:VIB:SCP (Short current is detected in VBST. The device goes into standby)\n",__func__));
			if(rx[0]&0x02)
				DbgOut((DBL_INFO, "%s:VIB:OCP (Over-current is detected in OUTP or OUTN. The device goes into standby)\n",__func__));
			if(rx[0]&0x04)
				DbgOut((DBL_INFO, "%s:VIB:TSD (Temp is above over-temperature threshold. The device goes into standby)\n",__func__));
			if(rx[0]&0x08)
				DbgOut((DBL_INFO, "%s:VIB:UVLO (A VIN droop was observed. The device goes into standby)\n",__func__));
			
		}
		//end
		if(gVibDebugLog&0x0001)//enable irq message
		{
			rx[0] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS0);
			rx[1] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS1);
			DbgOut((DBL_INFO, "%s:TOP_VIB:status0:0x%02x, status1:0x%02x\n",__func__, rx[0], rx[1]));
			if(rx[0]&0x20){
				//irq_rtp_play_func(0);
				DbgOut((DBL_INFO, "TOP_VIB: fifo empty falling edge"));
			}
		}
		//dw791x_byte_write(dw791x->i2c, dw791x->play_back, DRV_PLAY);
	}
}

DECLARE_WORK(bot_vib_work, bot_vib_irq_rtp_work);
//rtp_play workqueue
static void bot_vib_rtp_play_work_func(struct work_struct *work)
{
	struct dw791x_priv *dw791x =
                 container_of(work, struct dw791x_priv, rtp_work);
    
	//if(gVibDebugLog&0x0010)//store node debug
		//DbgOut((DBL_INFO, "%s: (1)fw_name= %s ,bin_num=%d, repeat=%d, break=%d +++\n",__func__, dw791x->fwName, dw791x->bin_num, dw791x->repeat, dw791x->bRTP_Play_Break));
	//if(request_firmware(&dw791x->fw, dw791x->fwName, &dw791x->i2c->dev) == 0) {
		if(gVibDebugLog&0x0010)//check break status
			DbgOut((DBL_INFO, "%s: (1)fw_name= %s ,bin_num=%d, repeat=%d, break=%d, call request_transfer_rtp_wave()...\n",__func__, dw791x->fwName, dw791x->bin_num, dw791x->repeat, dw791x->bRTP_Play_Break));
		request_transfer_rtp_wave(dw791x->i2c, (u8*)dw791x->fw->data, dw791x->repeat);
	//} 
	if(dw791x->fw!=NULL)
		release_firmware(dw791x->fw);
	dw791x->bRTP_Play_Break=false;
	dw791x_byte_write(dw791x->i2c, dw791x->play_back, DRV_STOP);
	dw7914_mode_set(dw791x->i2c, D14_FIFO_FLUSH, BITSET);
	if(gVibDebugLog&0x0010)//store node debug
		DbgOut((DBL_INFO, "%s: (2)fw_name= %s ,bin_num=%d, repeat=%d, break=%d ---\n",__func__, dw791x->fwName, dw791x->bin_num, dw791x->repeat, dw791x->bRTP_Play_Break));
}
//end rtp_play workqueue

static irqreturn_t bot_vib_irq_rtp_handler(int irq, void *unuse)
{	
    queue_work(rtp_workqueue[0], &bot_vib_work);

  	return IRQ_HANDLED;
}


static int init_irq_rtp(struct dw791x_priv *pdev)
{
	int ret;

	//bot_vib irq
	if (gpio_request(pdev->bot_vib_irq_gpio, "request_bot_vib__irq_gpio")) {
		DbgOut((DBL_INFO, "GPIO request faiure: %d\n", pdev->bot_vib_irq_gpio));
		return -1;
	}

	ret = gpio_direction_input(pdev->bot_vib_irq_gpio);

	if(ret < 0) {
		DbgOut((DBL_INFO, "BOT_VIB: Can't set GPIO direction, error %i\n", ret));
    	gpio_free(pdev->bot_vib_irq_gpio);
	    return -EINVAL;
	}
	else DbgOut((DBL_INFO, "BOT_VIB:GPIO direction input: %d\n", pdev->bot_vib_irq_gpio));
	

	pdev->bot_vib_irq_num = gpio_to_irq(pdev->bot_vib_irq_gpio);
	
	DbgOut((DBL_INFO, "BOT_VIB: IRQ Line: %d\n", pdev->bot_vib_irq_num));

	ret = request_irq(pdev->bot_vib_irq_num, (irq_handler_t)bot_vib_irq_rtp_handler, IRQF_TRIGGER_FALLING, "bot_vib_trig_interrupt", NULL);

	if(ret < 0) {
		irq_rtp_exit(0);
		DbgOut((DBL_INFO, "IRQ requset error: %d\n", ret));
	}

	rtp_workqueue[0] = create_singlethread_workqueue("bot_vib_rtp_work_queue");
	return ret;		
}


static int irq_rtp_exit(int index)
{
	//#if BOT_IS_INDEX_0
	//struct dw791x_priv *dw791x =dw791x_devs[1];//only top_vib hold all gpio information
	//#else
	struct dw791x_priv *dw791x =dw791x_devs[0];//only top_vib hold all gpio information
	//#endif

	if(index==0)//bot_vib
	{
		free_irq(dw791x->bot_vib_irq_num, NULL);
		cancel_work_sync(&bot_vib_work);
		//destroy_workqueue(rtp_workqueue[1]);
	}
	
	return 0;
}


#endif

static void vibrator_set_trigger(int value)
{
    int ret=-1;
    
	if(gTrigger_gpio>0){
		ret = gpio_direction_output(gTrigger_gpio, (value>0)?1:0);
		if(ret<0)
			DbgOut((DBL_INFO, "%s:Can't set trigger_gpio out failed: %i\n", __func__, ret));
		if(gVibDebugLog&0x0010)//store node debug
			DbgOut((DBL_INFO, "%s:gpio(%d)=%d\n", __func__, gTrigger_gpio,value));
	}
	else
			DbgOut((DBL_INFO, "%s:trigger_gpio=%d, error!\n", __func__));
}
//end

//for system property: vendor.asus.vibrator_control
#if 1

static ssize_t bot_vib_control_set (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct dw791x_priv *dw791x=dw791x_devs[0];  //BOT_VIB
	int mode=0;
	int bus=i2c_adapter_id(dw791x->i2c->adapter);
	
	sscanf(buf, "0x%08x", &mode);
	DbgOut((DBL_INFO, "%s cmd: bus=%d, mode=0x%08x, fun=0x%04x, param=0x%04x\n", __func__, bus, mode, (mode&0xffff0000)>>16, (mode&0x0000ffff)));
	dw791x_vibrator_control(dw791x->i2c, (mode&0xffff0000)>>16, (mode&0x0000ffff));//this function has i2c failure detection
	//DbgOut((DBL_INFO, "%s cmd: bus=%d, buf=%s\n", __func__, bus, buf));
	return count;
}

static ssize_t bot_vib_control_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct dw791x_priv *dw791x=dw791x_devs[0];  //BOT_VIB

	int ret=0;

    if(!dw791x->i2c_status){
		return snprintf(buf, PAGE_SIZE, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c));;
	}
	ret = dw791x_byte_read(dw791x->i2c, DW791x_PID);
	DbgOut((DBL_INFO, "product id : %x\n", ret));
	
	ret = dw791x_byte_read(dw791x->i2c, DW791x_STATUS);
	DbgOut((DBL_INFO, "chip status : %x\n", ret));
	return snprintf(buf, PAGE_SIZE, "[VIB] status = %x\n", ret);
}

DEVICE_ATTR(bot_vib_control, (S_IWUSR|S_IRUGO), bot_vib_control_show, bot_vib_control_set);

static int dw791x_vibrator_control_sysfs_init(void)
{
	int ret = 0;
	android_vibrator_kobj = kobject_create_and_add("asus_vib_ctrl", NULL);
	
	if (android_vibrator_kobj == NULL) {
		DbgOut((DBL_INFO, "%s:subsystem_register_failed", __func__));
	}
	ret = sysfs_create_file(android_vibrator_kobj, &dev_attr_bot_vib_control.attr);
	if (ret) {
		DbgOut((DBL_INFO, "%s: sysfs_create_file bot_vib_control failed\n", __func__));
	}
	else
		DbgOut((DBL_INFO, "attribute bot_vib_control file register Done"));
	
	return 0;
}
#endif

//end
static int dw791x_parse_dt(struct i2c_client *i2c, struct dw791x_priv *pdev)
{
	struct device *dev = &i2c->dev;
	struct device_node *np = dev->of_node;


	if (!np) return -1;
	//#ifdef INT_EVENT_OPT
	//top_vib_irq-gpio
	//DbgOut((DBL_INFO, "parse irq-gpio\n"));

	//bot_vib_irq-gpio
	DbgOut((DBL_INFO, "parse irq-gpio\n"));
	pdev->bot_vib_irq_gpio = of_get_named_gpio(np, "dw791x,irq-gpio", 0);
	
	if (pdev->bot_vib_irq_gpio < 0) {
		printk("%s: Looking up %s property in node %s failed %d\n", __func__, "dw791x,irq-gpio", dev->of_node->full_name, pdev->bot_vib_irq_gpio);
		pdev->bot_vib_irq_gpio = -1;
	}
	else{
		printk(KERN_ERR "%s: get dw791x irq pin(cpu: gpio %u)\n", __func__, pdev->bot_vib_irq_gpio);
	}
	//#endif
	//request enable pin
		
	//request top_trigger3 pin

	//request bot_trigger3 pin
#if 1
	DbgOut((DBL_INFO, "%s:parse dw791x_trigger_gpio\n",__func__));
	pdev->bot_trigger3_gpio = of_get_named_gpio(np, "dw791x_trigger_gpio", 0);
	
	if (pdev->bot_trigger3_gpio < 0) {
		printk("%s: Looking up %s property in node %s failed %d\n", __func__, "dw791x_trigger_gpio", dev->of_node->full_name, pdev->bot_trigger3_gpio);
		pdev->bot_trigger3_gpio = -1;
	}
	else{
		printk(KERN_ERR "%s: get dw791x_trigger_gpio(%u)\n", __func__, pdev->bot_trigger3_gpio);
		//gTrigger_gpio=pdev->bot_trigger3_gpio;
	}
	if(!gpio_is_valid(pdev->bot_trigger3_gpio)) {
		printk(KERN_ERR "%s: dw791x_trigger_gpio(%u) is invalid\n", __func__, pdev->bot_trigger3_gpio);
	}
	else
		printk(KERN_ERR "%s: dw791x_trigger_gpio(%u) is valid\n", __func__, pdev->bot_trigger3_gpio);
#endif	

	return 0;
}

static
enum led_brightness dw791x_brightness_get(struct led_classdev *cdev)
{
        return 0;
}

static void dw791x_brightness_set(struct led_classdev *cdev,
                enum led_brightness level)
{
}

//sysfs functions

static ssize_t channel_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	//read channel

	return snprintf(buf, PAGE_SIZE, "Channel=0x%0x\n", gChannel);
}

static ssize_t channel_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret;
	int val;
	ret=kstrtouint(buf, 0, &val);
	if(gVibDebugLog&0x0010)//store node debug
		printk("%s:set channel=%d\n", __func__,val);
	if(val >0 && val <= MAX_CHANNEL)
		gChannel=val;
	else
		gChannel=1;
	if(val!=gChannel)
		printk("%s:final channel=%d\n", __func__,gChannel);
	return count;
}

static ssize_t debuglog_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	//read channel

	return snprintf(buf, PAGE_SIZE, "gVibDebugLog=0x%0x\n", gVibDebugLog);
}

static ssize_t debuglog_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret;
	int val;
	ret=kstrtouint(buf, 0, &val);
	printk("%s:set gVibDebugLog=%d\n", __func__,val);
	if(val >0 && val <0xffffffff)
		gVibDebugLog=val;
	else
		gVibDebugLog=0;
	printk("%s:gVibDebugLog=%d\n", __func__,gVibDebugLog);
	return count;
}

//factory test: enable top_vib or bot_vib
static ssize_t vib_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(!buf) return -1;

	return snprintf(buf, PAGE_SIZE, "%s:%d\n", __func__, bEnable_Vib);
}
static ssize_t vib_enable_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int val=-1, ret=0;

	if(!buf) return -1;

	ret=kstrtouint(buf, 0, &val);
	if(ret <0) {
		printk("%s: kstrtouint failed\n", __func__);
		return ret;
	}
	DbgOut((DBL_INFO, "%s:val=%d\n", __func__, val));
	bEnable_Vib=val;
	return count;
}

//PWM
static ssize_t pwm_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    struct dw791x_priv *dw791x=NULL;
	//int ch=0;
	int index=-1;

	if(!buf) return -1;

	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];

	return snprintf(buf, PAGE_SIZE, "%s:0x%02x\n", __func__, dw791x_PWM_get(dw791x->i2c));
}
static ssize_t pwm_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int val=-1, ret=0;
	int index=-1;
    struct dw791x_priv *dw791x=NULL;
    
	if(!buf) return -1;

	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
    if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		return count;
	}

	ret=kstrtouint(buf, 0, &val);
	if(ret <0) {
		printk("%s: kstrtouint failed\n", __func__);
		return ret;
	}
	DbgOut((DBL_INFO, "%s:val=%d\n", __func__, val));
	gPWM=val&0x03;
	dw791x_PWM_set(dw791x->i2c, gPWM);
	return count;
}
//check status register
static ssize_t status_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct dw791x_priv *dw791x=NULL;
    u8 rx[4];
    
	if(!buf) return -1;
	

    dw791x=dw791x_devs[0];
    if(!dw791x->i2c_status){
		return snprintf(buf, PAGE_SIZE, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c));
	}
	rx[0] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS0);
	rx[1] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS1);
	DbgOut((DBL_INFO, "%s:BOT_VIB: status0=0x%02x, status1=0x%02x!\n", __func__, rx[0], rx[1]));		

	return snprintf(buf, PAGE_SIZE, "%s:\n[BOT_VIB:status0=0x%02x, status1=0x%02x] \n[TOP_VIB:status0=0x%02x, status1=0x%02x]\n", __func__, rx[0], rx[1], rx[2], rx[3]);
}
static ssize_t status_reg_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	return count;
}
//check register value
static int reg_addr=0;
static ssize_t register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int index=-1;
    struct dw791x_priv *dw791x=NULL;
    u8 rx, addr=0x00;

	if(!buf) return -1;

	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];

    if(!dw791x->i2c_status){
		return snprintf(buf, PAGE_SIZE, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c));
	}
	if(addr<0 || addr>0x5f) addr=0x00; //chip id
	rx = dw791x_byte_read(dw791x->i2c, reg_addr);
	DbgOut((DBL_INFO, "reg[0x%02x]=0x%02x", reg_addr, rx));		
	return	snprintf(buf, PAGE_SIZE, "reg[0x%02x]=0x%02x\n", reg_addr, rx);
}

static ssize_t register_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int index=-1;
    struct dw791x_priv *dw791x=NULL;
    u8 rx, addr=0x00, value=0, rw=0x00;//r:0x00, w:0x01
    int i=0;
    
	if(!buf) return -1;

	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
    if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		return count;
	}
	sscanf(buf, "%d 0x%2x %d", &rw, &addr, &value);
	if(rw<0x00 || rw>0x01) rw=0x00;//default: read register
	if(addr<0 || addr>0x5f) addr=0x00; //chip id

	if(rw==0x00)//read register
	{
		DbgOut((DBL_INFO, "%s:%s: dump register!\n", __func__, (index==0)?"BOT_VIB":"TOP_VIB"));
		for(i=0;i<value;i++){
			rx = dw791x_byte_read(dw791x->i2c, addr+i);
			DbgOut((DBL_INFO, "reg[0x%02x]=0x%02x", addr+i, rx));		
		}
		reg_addr=addr;
	}
	else{
		DbgOut((DBL_INFO, "%s:%s: write value(0x%02x) to register(0x%02x)!\n", __func__, (index==0)?"BOT_VIB":"TOP_VIB", value, addr));		
		dw791x_byte_write(dw791x->i2c, addr, value);
	}
	return count;
}

//trigger test
static ssize_t trigger_test_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int index=-1, ret=0;
	int val=-1;
    struct dw791x_priv *dw791x=NULL;
    u8 rx[2];
    
	if(!buf) return -1;

	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
    if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		return count;
	}

	ret=kstrtouint(buf, 0, &val);
	if(ret <0) {
		printk("%s: kstrtouint failed\n", __func__);
		return ret;
	}
	DbgOut((DBL_INFO, "%s:val=%d\n", __func__, val));
    dw791x_dump_registers(dw791x);
	DbgOut((DBL_INFO, "%s:reset PWM 48KHz\n", __func__));
    dw791x_PWM_set(dw791x->i2c, gPWM);
    dw791x_dump_registers(dw791x);
	dw791x_play_mode_sel(dw791x->i2c, 24, gTrigger1_3_Level, 0, 0); //trigger1
	//dw791x_play_mode_sel(dw791x->i2c, 25, gTrigger1_3_Level, 0, 0);//trigger3
	vibrator_set_trigger(0);
	mdelay(100);
	vibrator_set_trigger(1);
	//dw791x_byte_write(dw791x->i2c, dw791x->play_back, DRV_PLAY);
	mdelay(100);
	vibrator_set_trigger(0);
	rx[0] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS0);
	rx[1] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS1);
	DbgOut((DBL_INFO, "%s:status0=0x%02x, status1=0x%02x!\n", __func__, rx[0], rx[1]));		
	if(rx[0]&0x80)
		DbgOut((DBL_INFO, "%s:trigger in!\n", __func__));
	if(rx[0]&0x10)
		DbgOut((DBL_INFO, "%s:process done!\n", __func__));		
	if(rx[1]&0x10)
		DbgOut((DBL_INFO, "%s:Trigger 2 Rising!\n", __func__));		
	return count;
}
//trigger_waveform
static ssize_t trigger_waveform_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dw791x_priv *dw791x_1=NULL;//, *dw791x_2=NULL;
    
	if(!buf) return -1;
    dw791x_1=dw791x_devs[0];
	DbgOut((DBL_INFO, "trigger2 pattern: %s %d %d %d %d\n", "Gamma", dw791x_1->trigger_r, dw791x_1->break_r, dw791x_1->trigger_f, dw791x_1->break_f));

	return snprintf(buf, PAGE_SIZE, "[trigger2 pattern]\nGamma: %d %d %d %d\n", 
	dw791x_1->trigger_r, dw791x_1->break_r, dw791x_1->trigger_f, dw791x_1->break_f
	);
}

static ssize_t trigger_waveform_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	/* test trigger pattern
	* gamma/0832 trigger_r break_r trigger_f break_f
	* 0 2 0 2 0
	* 1 3 0 3 0
	* break inside: 1
	* no break: 2, 5
	* break：3
	* 0 1 0 1 0
	* 0 2 3 2 3
	*/
	struct dw791x_priv *dw791x=NULL;
	int actuator=0, trigger_r=0, break_r=0, trigger_f=0, break_f=0;
	
	sscanf(buf, "%d %d %d %d %d", &actuator, &trigger_r, &break_r, &trigger_f, &break_f);
	actuator=0;
	dw791x=dw791x_devs[0];

    if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		return count;
	}
	dw791x->trigger_r=trigger_r; 
	dw791x->break_r=break_r; 
	dw791x->trigger_f=trigger_f; 
	dw791x->break_f=break_f; 
	dw791x_byte_write(dw791x->i2c, 0x0B, 0x01); // mem mode 
	dw791x_byte_write(dw791x->i2c, 0x26, dw791x->trigger_r); // trig2_r
	dw791x_byte_write(dw791x->i2c, 0x28, dw791x->break_r);   // brake 1 assign
	dw791x_byte_write(dw791x->i2c, 0x29, dw791x->trigger_f); // trig2_f
	dw791x_byte_write(dw791x->i2c, 0x2B, dw791x->break_f);   // brake 1 assign
	DbgOut((DBL_INFO, "trigger2 pattern: %s %d %d %d %d\n", (actuator==0)?"Gamma":"0832", dw791x->trigger_r, dw791x->break_r, dw791x->trigger_f, dw791x->break_f));
	return count;
}

/*
 * channel: 1 for gamma, 2 for 0832, 3 for both
 */
void dw7914_trigger3_gpio(int channel, bool enable){
    struct dw791x_priv *dw791x=NULL;

	//#if BOT_IS_INDEX_0
	dw791x=dw791x_devs[0];//all gpio info in dw791x_devs[1]
	//#else
	//dw791x=dw791x_devs[1];//all gpio info in dw791x_devs[1]
	//#endif
	if(channel&0x01)//gamma
	{
		DbgOut((DBL_INFO, "%s:bot_trigger3(%d)=%d\n", __func__, dw791x->bot_trigger3_gpio, enable));
		gpio_direction_output(dw791x->bot_trigger3_gpio, enable);
	}
	if(channel&0x02)//0832
	{
		DbgOut((DBL_INFO, "%s:top_trigger3(%d)=%d\n", __func__, dw791x->top_trigger3_gpio, enable));
		gpio_direction_output(dw791x->top_trigger3_gpio, enable);
	}
}
EXPORT_SYMBOL(dw7914_trigger3_gpio);
void dw7914_trigger_gpio(int channel, int gpio, bool enable){
}
//trigger level
static ssize_t trigger_level_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int channel=1, trigger=2, level=3;
    struct dw791x_priv *dw791x=NULL;
    
	sscanf(buf, "%d %d %d", &channel, &trigger, &level);
	if(trigger==1){//trigger1
		set_trigger1_level(channel, level);
	}
	else if(trigger==2){ //trigger2
		if(channel&0x01){//gamma
			gTrigger_Tap_Level=level;
			dw791x->trigger_tap_level=level;
		}
		if(channel&0x02){//0832
			gTrigger_Slider_Level=level;
			dw791x->trigger_slider_level=level;
		}
		set_trigger2_level(channel, level);
	}
	else{//trigger3
		set_trigger1_level(channel, level);
	} 
	return count;
}
//end

//resample control
static ssize_t resample_control_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "resample_control=%d...\n", gResample);
}
static ssize_t resample_control_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret=0;
	int val=-1;

	ret=kstrtouint(buf, 0, &val);
	if(ret <0) {
		printk("%s: kstrtouint failed\n", __func__);
		return ret;
	}
	DbgOut((DBL_INFO, "%s:val=%d\n", __func__, val));
	gResample=val;
	return count;
}
//end

void set_trigger1_level(int channel, int level){
    struct dw791x_priv *dw791x=NULL;
    struct i2c_client* client=NULL;
    
	if(gVibDebugLog&0x0010)//store node debug
		DbgOut((DBL_INFO, "%s:channel=%d, level=%d\n", __func__, channel, level));
    if(channel&0x01){ //gamma
		DbgOut((DBL_INFO, "%s:gamma, level=%d\n", __func__, level));
		dw791x=dw791x_devs[0];
		client=dw791x->i2c;
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		}
		else{
			dw791x_byte_write(client, 0x03, 0x61); // trig priority, trig2 master, trig1 as edge trigger input
			dw791x_byte_write(client, 0x0B, 0x01); // mem mode + no_brake
			if(level==1)
			{
				//DbgOut((DBL_INFO, "write homekey_gamma_h1_1 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)homekey_gamma_h1_1,5);				
			}
			else if(level==3){
				//DbgOut((DBL_INFO, "write homekey_gamma_h1_3 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)homekey_gamma_h1_3,5);
			}
			else if(level==0){
				//DbgOut((DBL_INFO, "write homekey_gamma_h1_0 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)homekey_gamma_h1_0,5);
			}
			else{
				//DbgOut((DBL_INFO, "write homekey_gamma_h1 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)homekey_gamma_h1,5);
			}
		}
	}
	/*else if(channel&0x02){ //0832
		DbgOut((DBL_INFO, "%s:0832, level=%d\n", __func__, level));
		dw791x=dw791x_devs[1];
		client=dw791x->i2c;
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		}
		else{
			dw791x_byte_write(client, 0x03, 0x61); // trig priority, trig2 master, trig1 as edge trigger input
			dw791x_byte_write(client, 0x0B, 0x01); // mem mode + no_brake
			if(level==1)
			{
				//DbgOut((DBL_INFO, "write homekey_0832_h1_1 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x1A,RAM_ADDR16,(u8*)homekey_0832_h1_1,5);
			}
			else if(level==3){
				//DbgOut((DBL_INFO, "write homekey_0832_h1_3 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x1A,RAM_ADDR16,(u8*)homekey_0832_h1_3,5);
			}
			else if(level==0){
				//DbgOut((DBL_INFO, "write homekey_0832_h1_0 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x1A,RAM_ADDR16,(u8*)homekey_0832_h1_0,5);
			}
			else{
				//DbgOut((DBL_INFO, "write homekey_0832_h1 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x1A,RAM_ADDR16,(u8*)homekey_0832_h1,5);
			}
		}
	}*/
}
void set_trigger2_level(int channel, int level){
    struct dw791x_priv *dw791x=NULL;
    
    if(channel&0x01){ //gamma
		dw791x=dw791x_devs[0];
        
		if(!dw791x || !dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		}
		else{
			dw791x->trigger_tap_level=level;
			dw791x_byte_write(dw791x->i2c, 0x0B, 0x01); // mem mode 
			if(level==0){
				//DbgOut((DBL_INFO, "write tap_gamma_h1_0 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)tap_gamma_h1_0,5);
			}
			else if(level==1){
				//DbgOut((DBL_INFO, "write tap_gamma_h1_1 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)tap_gamma_h1_1,5);
			}
			else if(level==3){
				//DbgOut((DBL_INFO, "write tap_gamma_h1_3 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)tap_gamma_h1_3,5);
			}
			else{
				//DbgOut((DBL_INFO, "write tap_gamma_h1 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)tap_gamma_h1,5);
			}
		}
	}
	/*else if(channel&0x02){ //0832
		dw791x=dw791x_devs[1];
		if(!dw791x || !dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		}
		else{
			dw791x->trigger_slider_level=level;
			dw791x_byte_write(dw791x->i2c, 0x0B, 0x01); // mem mode 
			if(level==0){
				//DbgOut((DBL_INFO, "write slider_0832_h1_0 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)slider_0832_h1_0,5);
				//DbgOut((DBL_INFO, "write slider_0832_h2_0 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)slider_0832_h2_0,5);
				//DbgOut((DBL_INFO, "write slider_0832_h3_0 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)slider_0832_h3_0,5);
				//DbgOut((DBL_INFO, "write slider_0832_h4_0 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)slider_0832_h4_0,5);
				//DbgOut((DBL_INFO, "write slider_0832_h5_0 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)slider_0832_h5_0,5);
			}
			else if(level==1){
				//DbgOut((DBL_INFO, "write slider_0832_h1_1 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)slider_0832_h1_1,5);
				//DbgOut((DBL_INFO, "write slider_0832_h2_1 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)slider_0832_h2_1,5);
				//DbgOut((DBL_INFO, "write slider_0832_h3_1 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)slider_0832_h3_1,5);
				//DbgOut((DBL_INFO, "write slider_0832_h4_1 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)slider_0832_h4_1,5);
				//DbgOut((DBL_INFO, "write slider_0832_h5_1 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)slider_0832_h5_1,5);
			}
			else if(level==3){
				//DbgOut((DBL_INFO, "write slider_0832_h1_3 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)slider_0832_h1_3,5);
				//DbgOut((DBL_INFO, "write slider_0832_h2_3 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)slider_0832_h2_3,5);
				//DbgOut((DBL_INFO, "write slider_0832_h3_3 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)slider_0832_h3_3,5);
				//DbgOut((DBL_INFO, "write slider_0832_h4_3 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)slider_0832_h4_3,5);
				//DbgOut((DBL_INFO, "write slider_0832_h5_3 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)slider_0832_h5_3,5);
			}
			else{
				//DbgOut((DBL_INFO, "write slider_0832_h1 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x01,RAM_ADDR16,(u8*)slider_0832_h1,5);
				//DbgOut((DBL_INFO, "write slider_0832_h2 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x06,RAM_ADDR16,(u8*)slider_0832_h2,5);
				//DbgOut((DBL_INFO, "write slider_0832_h3 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)slider_0832_h3,5);
				//DbgOut((DBL_INFO, "write slider_0832_h4 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)slider_0832_h4,5);
				//DbgOut((DBL_INFO, "write slider_0832_h5 header\n"));
				dw791x_seq_write(dw791x->i2c, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)slider_0832_h5,5);
			}
		}
	}*/
}
void set_trigger3_level(int channel, int level){
    struct dw791x_priv *dw791x=NULL;
    struct i2c_client* client=NULL;
    
    if(channel&0x01){ //gamma
		dw791x=dw791x_devs[0];
		client=dw791x->i2c;
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		}
		else{
			dw791x_byte_write(client, 0x03, 0x61); // trig priority, trig2 master, trig1 as edge trigger input
			dw791x_byte_write(client, 0x0B, 0x01); // mem mode + no_brake
			if(level==1)
			{
				//DbgOut((DBL_INFO, "write typing_gamma_h1_1 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)typing_gamma_h1_1,5);
				//DbgOut((DBL_INFO, "write typing_gamma_h2_1 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)typing_gamma_h2_1,5);
				//DbgOut((DBL_INFO, "write typing_gamma_h3_1 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)typing_gamma_h3_1,5);
			}
			else if(level==3){
				//DbgOut((DBL_INFO, "write typing_gamma_h1_3 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)typing_gamma_h1_3,5);
				//DbgOut((DBL_INFO, "write typing_gamma_h2_3 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)typing_gamma_h2_3,5);
				//DbgOut((DBL_INFO, "write typing_gamma_h3_3 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)typing_gamma_h3_3,5);
			}
			else if(level==0){
				//DbgOut((DBL_INFO, "write typing_gamma_h1_0 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)typing_gamma_h1_0,5);
				//DbgOut((DBL_INFO, "write typing_gamma_h2_0 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)typing_gamma_h2_0,5);
				//DbgOut((DBL_INFO, "write typing_gamma_h3_0 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)typing_gamma_h3_0,5);
			}
			else{
				//DbgOut((DBL_INFO, "write typing_gamma_h1 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x0B,RAM_ADDR16,(u8*)typing_gamma_h1,5);
				//DbgOut((DBL_INFO, "write typing_gamma_h2 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x10,RAM_ADDR16,(u8*)typing_gamma_h2,5);
				//DbgOut((DBL_INFO, "write typing_gamma_h3 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x15,RAM_ADDR16,(u8*)typing_gamma_h3,5);
			}
		}
	}
	/*else if(channel&0x02){ //0832
		dw791x=dw791x_devs[1];
		client=dw791x->i2c;
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		}
		else{
			dw791x_byte_write(client, 0x03, 0x61); // trig priority, trig2 master, trig1 as edge trigger input
			dw791x_byte_write(client, 0x0B, 0x05); // mem mode + auto_brake
			if(level==1)
			{
				//DbgOut((DBL_INFO, "write typing_0832_h1_1 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x1F,RAM_ADDR16,(u8*)typing_0832_h1_1,5);
				//DbgOut((DBL_INFO, "write typing_0832_h2_1 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x24,RAM_ADDR16,(u8*)typing_0832_h2_1,5);
				//DbgOut((DBL_INFO, "write typing_0832_h3_1 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x29,RAM_ADDR16,(u8*)typing_0832_h3_1,5);
				//DbgOut((DBL_INFO, "write typing_0832_h4_1 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x2E,RAM_ADDR16,(u8*)typing_0832_h4_1,5);
				//DbgOut((DBL_INFO, "write typing_0832_h5_1 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x33,RAM_ADDR16,(u8*)typing_0832_h5_1,5);
			}
			else if(level==3){
				//DbgOut((DBL_INFO, "write typing_0832_h1_3 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x1F,RAM_ADDR16,(u8*)typing_0832_h1_3,5);
				//DbgOut((DBL_INFO, "write typing_0832_h2_3 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x24,RAM_ADDR16,(u8*)typing_0832_h2_3,5);
				//DbgOut((DBL_INFO, "write typing_0832_h3_3 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x29,RAM_ADDR16,(u8*)typing_0832_h3_3,5);
				//DbgOut((DBL_INFO, "write typing_0832_h4_3 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x2E,RAM_ADDR16,(u8*)typing_0832_h4_3,5);
				//DbgOut((DBL_INFO, "write typing_0832_h5_3 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x33,RAM_ADDR16,(u8*)typing_0832_h5_3,5);
			}
			else if(level==0){
				//DbgOut((DBL_INFO, "write typing_0832_h1_0 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x1F,RAM_ADDR16,(u8*)typing_0832_h1_0,5);
				//DbgOut((DBL_INFO, "write typing_0832_h2_0 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x24,RAM_ADDR16,(u8*)typing_0832_h2_0,5);
				//DbgOut((DBL_INFO, "write typing_0832_h3_0 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x29,RAM_ADDR16,(u8*)typing_0832_h3_0,5);
				//DbgOut((DBL_INFO, "write typing_0832_h4_0 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x2E,RAM_ADDR16,(u8*)typing_0832_h4_0,5);
				//DbgOut((DBL_INFO, "write typing_0832_h5_0 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x33,RAM_ADDR16,(u8*)typing_0832_h5_0,5);
			}
			else{
				//DbgOut((DBL_INFO, "write typing_0832_h1 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x1F,RAM_ADDR16,(u8*)typing_0832_h1,5);
				//DbgOut((DBL_INFO, "write typing_0832_h2 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x24,RAM_ADDR16,(u8*)typing_0832_h2,5);
				//DbgOut((DBL_INFO, "write typing_0832_h3 header\n"));
				//dw791x_seq_write(client, dw791x->mem_input,0x29,RAM_ADDR16,(u8*)typing_0832_h3,5);
				//DbgOut((DBL_INFO, "write typing_0832_h4 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x2E,RAM_ADDR16,(u8*)typing_0832_h4,5);
				//DbgOut((DBL_INFO, "write typing_0832_h5 header\n"));
				dw791x_seq_write(client, dw791x->mem_input,0x33,RAM_ADDR16,(u8*)typing_0832_h5,5);
			}
		}
	}*/
}
//trigger gpio
/* channel trigger1/3 enable level
 * level 1,2,3 low, middle, high
 * channel: 1 for gamma, 2 for 0832, 3 for both
 * gamma trigger1 disable, gamma trigger1 enable
 * 1 1 0 2                  1 1 1 2
 * gamma trigger3 disable, gamma trigger3 enable
 * 1 3 0 2                  1 3 1 2
 * 0832 trigger3 disable, 0832 trigger3 enable
 * 2 3 0 2                  2 3 1 2
 */
static ssize_t trigger_gpio_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int trigger=-1, channel=1;
	int val=-1, level=2;
	int old_trigger=-1;
       
    struct dw791x_priv *dw791x=NULL;
    
	sscanf(buf, "%d %d %d %d", &channel, &trigger, &val, &level);
	old_trigger=gTrigger_gpio;
	if(trigger==1){//gamma & 0832 use the same boot_vib_gpio
		//gamma
		//#if BOT_IS_INDEX_0
		//dw791x=dw791x_devs[1];
		//#else
		dw791x=dw791x_devs[0];
		//#endif
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		}
		else{
			if(gTrigger1_3_Level!=level)
			{
				//dw791x_play_mode_sel(dw791x->i2c, 24, level, 0, 0);
				if(gVibDebugLog&0x0010)//store node debug
					DbgOut((DBL_INFO, "%s:gamma reset level=%d\n", __func__, level));
				set_trigger1_level(1, level);
				//set_trigger1_level(2, 0);
			}
			
			//0832
			/*
			#if BOT_IS_INDEX_0
			dw791x=dw791x_devs[1];
			#else
			dw791x=dw791x_devs[0];
			#endif
			//since trigger1 will issue gamma and 0832, we need to check 0832 again.
			if(!dw791x->i2c_status){
				DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
			}
			else{
				if(gTrigger1_3_Level!=level)
				{
					//dw791x_play_mode_sel(dw791x->i2c, 24, level, 0, 0);
					if(gVibDebugLog&0x0010)//store node debug
						DbgOut((DBL_INFO, "%s:0832 reset level=%d\n", __func__, level));
					set_trigger1_level(2, level);
					//set_trigger1_level(2, 0);
				}
				gTrigger_gpio=dw791x->boot_vib_gpio;
				//gpio info is in 0832 i2c data structure, since 0832 i2c bus is detected first.
				if(gVibDebugLog&0x0010)//store node debug
					DbgOut((DBL_INFO, "%s:trigger=%d (gpio=%d), val=%d, level=%d\n", __func__, trigger, gTrigger_gpio, val, level));
				//vibrator_set_trigger(val);
				vibrator_set_trigger(1);
				vibrator_set_trigger(0);
			}*/
		}
	}
	else{//gamma and 0832 has different trigger3_gpio
		if(channel&0x01)//gamma
		{
			//#if BOT_IS_INDEX_0
			dw791x=dw791x_devs[0];
			//#else
			//dw791x=dw791x_devs[1];
			//#endif
			if(!dw791x->i2c_status){
				DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
			}
			else{
				if(gTrigger1_3_Level!=level)
				{
					//dw791x_play_mode_sel(dw791x->i2c, 25, level, 0, 0);
					set_trigger3_level(1, level);
				}
				//gpio info is in 0832 i2c data structure, since 0832 i2c bus is detected first.
				//#if BOT_IS_INDEX_0
				//dw791x=dw791x_devs[1];
				//#else
				dw791x=dw791x_devs[0];
				//#endif
				gTrigger_gpio=dw791x->bot_trigger3_gpio;
				if(gVibDebugLog&0x0010)//store node debug
					DbgOut((DBL_INFO, "%s:trigger=%d (gpio=%d), val=%d, level=%d\n", __func__, trigger, gTrigger_gpio, val, level));
				//vibrator_set_trigger(val);
				vibrator_set_trigger(1);
				vibrator_set_trigger(0);
			}
		}
		/*if(channel&0x02)//0832
		{
			#if BOT_IS_INDEX_0
			dw791x=dw791x_devs[1];
			#else
			dw791x=dw791x_devs[0];
			#endif
			if(!dw791x->i2c_status){
				DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
			}
			else{
				if(gTrigger1_3_Level!=level)
				{
					//dw791x_play_mode_sel(dw791x->i2c, 25, level, 0, 0);
					set_trigger3_level(2, level);
				}
				gTrigger_gpio=dw791x->top_trigger3_gpio;
				if(gVibDebugLog&0x0010)//store node debug
					DbgOut((DBL_INFO, "%s:trigger=%d (gpio=%d), val=%d, level=%d\n", __func__, trigger, gTrigger_gpio, val, level));
				//vibrator_set_trigger(val);
				vibrator_set_trigger(1);
				vibrator_set_trigger(0);
			}
		}*/
	}
	gTrigger1_3_Level=level;
	if(gVibDebugLog&0x0010)//store node debug
		DbgOut((DBL_INFO, "%s:trigger=%d (gpio=%d), val=%d, level=%d\n", __func__, trigger, gTrigger_gpio, val, level));
	gTrigger_gpio=old_trigger;
	return count;
}
//end
//memory mode waveform play function
void mem_play(struct dw791x_priv *dw791x, int waveform_num){
    
    dw791x_byte_write(dw791x->i2c, dw791x->play_mode, MEM);
    dw791x_byte_write(dw791x->i2c, DW7914_WAVQ1, waveform_num);//start
    dw791x_byte_write(dw791x->i2c, DW7914_WAVE_SEQ_LOOP0, 0x01);
    dw791x_byte_write(dw791x->i2c, DW7914_WAVQ2, 0x00);//stop
    dw791x_byte_write(dw791x->i2c, DW7914_MEM_LOOP, 0x00);
    dw791x_byte_write(dw791x->i2c, dw791x->play_back, DRV_PLAY);
}
//mem mode play test
static ssize_t mem_play_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ch=-1, waveform_num=0, rx[2], index=-1;
    struct dw791x_priv *dw791x=NULL;
    
		DbgOut((DBL_INFO, "%s: +++\n", __func__));

	sscanf(buf, "%d %d", &ch, &waveform_num);
	index=channel_to_index(ch);
	if (index == -1)
		return -1;
	
	if(waveform_num<1 || waveform_num > 127) waveform_num=1;
    dw791x=dw791x_devs[index];
    
    if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		return count;
	}
    DbgOut((DBL_INFO, "MEM mode Play start (%d)...\n", waveform_num));
    mem_play(dw791x,waveform_num);
    rx[0] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS0);
    rx[1] = dw791x_byte_read(dw791x->i2c, DW7914_STATUS1);
    DbgOut((DBL_INFO, "%s:status0=0x%02x, status1=0x%02x!\n", __func__, rx[0], rx[1]));		
    if(rx[0]&0x10)
        DbgOut((DBL_INFO, "%s:process done!\n", __func__));		
	//}
	DbgOut((DBL_INFO, "%s: ---\n", __func__));
	return count;
}

//rtp mode play test
static ssize_t rtp_play_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int channel=1,val=-1;
    struct dw791x_priv *dw791x=NULL;
    
	if(!buf) return -1;
	sscanf(buf, "%d %d", &channel, &val);
	if(gVibDebugLog&0x0010)//store node debug
		DbgOut((DBL_INFO, "%s:(1) channel=%d, num=%d +++\n", __func__, channel, val));
	if(channel&0x01)//gamma
	{
		dw791x=dw791x_devs[0];
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
			return count;
		}
		if(val<=0 || val>=65535) val=1;
		dw791x_vibrator_control(dw791x->i2c, 5, val);//this function has i2c failure detection
	}
	/*if(channel&0x02)//0832
	{
		dw791x=dw791x_devs[1];
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
			return count;
		}
		if(val<=0 || val>=65535) val=1;
		dw791x_vibrator_control(dw791x->i2c, 5, val);//this function has i2c failure detection
	}*/
	if(gVibDebugLog&0x0010)//store node debug
		DbgOut((DBL_INFO, "%s: ---\n", __func__));
	return count;
}

//rtp mode repeat play test
static ssize_t rtp_play_repeat_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int channel=1, val=-1, repeat=1;
    struct dw791x_priv *dw791x=NULL;
    
	if(!buf) return -1;
	sscanf(buf, "%d %d %d", &channel, &val, &repeat);
    
	if(gVibDebugLog&0x0010)//store node debug
		DbgOut((DBL_INFO, "%s:(1) channel=%d, num=%d, repeat=%d +++\n", __func__, channel, val, repeat));
	if(repeat<0 || repeat > 65535) repeat=1;
	if(val<=0 || val>=65535) val=1;

	if(channel&0x01)//gamma
	{
		dw791x=dw791x_devs[0];
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
			return count;
		}
		//stop old vibration and then play new one.
		if(dw791x->bRTP_Play_Break)
		{
			if(gVibDebugLog&0x0010)//store node debug
				DbgOut((DBL_ERROR, "%s:(2-1) bin_num=%d, break=%d, cancel_work_sync!\n", __func__, val, dw791x->bRTP_Play_Break));
			cancel_work_sync(&dw791x->rtp_work);
			dw791x->bRTP_Play_Break=false;
			if(gVibDebugLog&0x0010)//store node debug
				DbgOut((DBL_ERROR, "%s:(2-2) bin_num=%d, break=%d, cancel_work_sync!\n", __func__, val, dw791x->bRTP_Play_Break));
		}
		//end
		dw791x->bin_num=val;
		sprintf(dw791x->fwName, "%d.bin", dw791x->bin_num);
		dw791x->repeat=repeat;
		if(gVibDebugLog&0x0010)//store node debug
			DbgOut((DBL_INFO, "%s:(3) bin_num=%d, fwName=%s, repeat=%d, break=%d, request_firmware...\n", __func__, dw791x->bin_num, dw791x->fwName, repeat, dw791x->bRTP_Play_Break));
		if(request_firmware(&dw791x->fw, dw791x->fwName, &dw791x->i2c->dev) == 0) {
			if(gVibDebugLog&0x0010)//check break status
				DbgOut((DBL_INFO, "%s: (4)fw_name= %s ,bin_num=%d, repeat=%d, break=%d, queuework...\n",__func__, dw791x->fwName, dw791x->bin_num, dw791x->repeat, dw791x->bRTP_Play_Break));
			queue_work(dw791x->rtp_workqueue, &dw791x->rtp_work);
		}
		else
		{
			DbgOut((DBL_INFO, "%s: (4)fw_name= %s ,bin_num=%d, repeat=%d, break=%d, request_firmware failed!\n",__func__, dw791x->fwName, dw791x->bin_num, dw791x->repeat, dw791x->bRTP_Play_Break));
		}
	}
/*	if(channel&0x02)//0832
	{
		dw791x=dw791x_devs[1];
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
			return count;
		}
		//stop old vibration and then play new one.
		dw791x->bRTP_Play_Break=true;
		dw791x_byte_write(dw791x->i2c, dw791x->play_back, DRV_STOP);
		mutex_lock(&dw791x->dev_lock);
		if(request_firmware(&dw791x->fw, fwName, &dw791x->i2c->dev) == 0) {
			if(gVibDebugLog&0x0010)//store node debug
				DbgOut((DBL_INFO, "%s:fw_name= %s fw_size= %ld\n", __func__, fwName, dw791x->fw->size));
			dw791x->bRTP_Play_Break=false;
			request_transfer_rtp_wave(dw791x->i2c, (u8*)dw791x->fw->data, repeat);
		} release_firmware(dw791x->fw);
		mutex_unlock(&dw791x->dev_lock);
	}*/
	if(gVibDebugLog&0x0010)//store node debug
		DbgOut((DBL_INFO, "%s:(1) channel=%d, num=%d, repeat=%d ---\n", __func__, channel, val, repeat));
	return count;
}

//factory test
static ssize_t factory_test_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "factory_test...\n");
}

static ssize_t factory_test_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int index=-1, mode=0;
    struct dw791x_priv *dw791x=NULL;
    
	if(!buf) return -1;

	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
    if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		return count;
	}

	if(buf[0]=='0')//Default Power
	{
		DbgOut((DBL_INFO, "%s:channel=%d, command:%c: no support!\n", __func__, index, buf[0]));
	}
	if(buf[0]=='1')//Max Power
		mode=(index==1) ? 30:33;
	if(buf[0]=='2')//Rated Power
		mode=(index==1) ? 31:34;
	if(buf[0]=='3')//default, for SMMI
		mode=(index==1) ? 36:38;
	if(buf[0]=='4')//Sweep Rated Power
		mode=(index==1) ? 32:35;
	DbgOut((DBL_INFO, "%s:channel=%d, command:%c\n", __func__, index, buf[0]));
	dw791x_play_mode_sel(dw791x->i2c, mode, 0, 0, 0);
	return count;
}
static ssize_t rtp_break_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct dw791x_priv *dw791x_gamma;//, *dw791x_0832;
	dw791x_gamma=dw791x_devs[0];
	//dw791x_0832=dw791x_devs[1];
	return snprintf(buf, PAGE_SIZE, "gamma_rtp_break=%s\n", (dw791x_gamma->bRTP_Play_Break==true)?"true":"false");
}
static ssize_t rtp_break_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int channel=1, val=-1;
    struct dw791x_priv *dw791x=NULL;
    
	if(!buf) return -1;
	sscanf(buf, "%d %d", &channel, &val);
	if(channel&0x01){
		dw791x=dw791x_devs[0];
		dw791x->bRTP_Play_Break=(val!=0) ? true:false;
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		}
		else{
			if(dw791x->bRTP_Play_Break){
				dw791x_byte_write(dw791x->i2c, dw791x->play_back, DRV_STOP);
				cancel_work_sync(&dw791x->rtp_work);
			}
			if(gVibDebugLog&0x0010)//store node debug
				DbgOut((DBL_INFO, "%s:set plackbreak=%s\n", __func__, (dw791x->bRTP_Play_Break!=0) ? "true":"false"));
		}
	}

	return count;
}

//static ssize_t rtp_go_store(struct device *dev,
//			       struct device_attribute *attr, const char *buf,
//			       size_t count)
//{
//}
static ssize_t vd_clamp_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	//read vd_clamp
	int val=0, index=-1;
    struct dw791x_priv *dw791x=NULL;
    
	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
    if(!dw791x->i2c_status){
		return snprintf(buf, PAGE_SIZE, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c));
	}
	val = dw791x_byte_read(dw791x->i2c, DW7914_VD_CLAMP);
	return snprintf(buf, PAGE_SIZE, "channel=%d, vmax(vd_clamp)=%d\n", index+1, val*40);
}
static ssize_t vd_clamp_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int channel=1;
	int val=-1, val2=0;
    struct dw791x_priv *dw791x=NULL;
	
	sscanf(buf, "%d %d", &channel, &val);
	if(channel&0x01)//gamma
	{
		dw791x=dw791x_devs[0];
		if(!dw791x->i2c_status){
			DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
			return count;
		}
		if(val<=0 || val > MAX_BOT_VD_CLAMP){
			DbgOut((DBL_ERROR, "%s:val=%d, exceed the MAX(%d), set to default (%d)\n", __func__, val, MAX_BOT_VD_CLAMP, MAX_BOT_VD_CLAMP));
			val=MAX_BOT_VD_CLAMP;//MAX value
		}
		if(gVibDebugLog&0x0010)//store node debug
        {
			DbgOut((DBL_ERROR, "%s:channel=%d, vmax(vd_clamp)=%d\n", __func__, channel, val));
        }
		dw791x_vd_clamp_set(dw791x->i2c, val);
		if(gVibDebugLog&0x0010)//store node debug
        {
		//read back to check
		val2 = dw791x_byte_read(dw791x->i2c, DW7914_VD_CLAMP);
			DbgOut((DBL_ERROR, "%s:read back: channel=%d, vmax(vd_clamp)=%d\n", __func__, channel, val2*40));
        }
	}

	return count;
}
static ssize_t bst_option_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	//read vd_clamp
	int val=0, index=-1;
    struct dw791x_priv *dw791x=NULL;

	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
    if(!dw791x->i2c_status){
		return snprintf(buf, PAGE_SIZE, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c));
	}
	val = dw791x_byte_read(dw791x->i2c, DW7914_BST_OPTION);
	return snprintf(buf, PAGE_SIZE, "index=%d, bst_option=%d\n", index, val);
}
static ssize_t bst_option_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int index=-1;
	int val=-1;
	u32 lv, limit, ofs;
    struct dw791x_priv *dw791x=NULL;
    
	if(!buf) return -1;

	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
    if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s: vibrator %d i2c fail\n", __func__, getActuatorIndex(dw791x->i2c)));
		return count;
	}
    sscanf(buf, "%d %d %d", &lv, &limit, &ofs);
	DbgOut((DBL_INFO, "%s:index=%d, lv=%d limit=%d ofs=%d\n", __func__, index, lv, limit, ofs));
	dw791x_bst_option(dw791x->i2c, lv, limit, ofs);	//default
	//read back to check
	val = dw791x_byte_read(dw791x->i2c, DW7914_BST_OPTION);
	DbgOut((DBL_INFO, "%s:read index=%d, vmax(vd_clamp)=%d\n", __func__, index, val));
	return count;
}

//end


//LED vib callback
//==================================
//static void led_vib_timer_callback(struct work_struct *work)
static enum hrtimer_restart led_vib_timer_callback(struct hrtimer *timer)
{
	struct dw791x_priv *dw791x = container_of(timer, struct dw791x_priv, led_vib_timer);
	//struct dw791x_priv *dw791x =dw791x_devs[0];
	//stop vibration
	dw791x->activate=0;
	dw791x_byte_write(dw791x->i2c, dw791x->play_back, DRV_STOP);
	DbgOut((DBL_ERROR, "%s: led vib timeout to stop.\n"));


	return HRTIMER_NORESTART;
}

void led_vib_timer_start(int index)
{
	struct dw791x_priv *dw791x =dw791x_devs[index];	

DbgOut((DBL_ERROR, "%s:index=%d\n", __func__, index));

	mutex_lock(&dw791x->dev_lock);

	if (hrtimer_active(&dw791x->led_vib_timer))
		hrtimer_cancel(&dw791x->led_vib_timer);

		hrtimer_start(&dw791x->led_vib_timer,
			      ktime_set(dw791x->duration / 1000,
					(dw791x->duration % 1000) * 1000000),
			      HRTIMER_MODE_REL);

	mutex_unlock(&dw791x->dev_lock);
}
//



//for LED vibrator HAL 1.0
//vibra_led_on:
//	echo 1 > state
//	echo ms > duration
//	echo 1 > activate

//vibra_led_off:
//	echo 0 > activate

static ssize_t activate_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int index=-1;
    struct dw791x_priv *dw791x=NULL;

	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
	//read activate
	return snprintf(buf, PAGE_SIZE, "Channel=0x%0x, activate:%d\n", gChannel, dw791x->activate);
}

static ssize_t activate_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int index=-1;
    struct dw791x_priv *dw791x=NULL;
    
	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
	DbgOut((DBL_INFO, "%s:channel=0x%0x %s(%dms)\n", __func__, gChannel,buf, dw791x->duration));
	if(buf[0]=='1') {	
		DbgOut((DBL_INFO, "%s:vibrator HAL: vibrate ...\n", __func__));
		 dw791x->activate=1;
		//dw791x_play_mode_sel(dw791x->i2c, 13,0,0,0);
		dw7914_autovib_set(dw791x->i2c);
		dw791x_byte_write(dw791x->i2c, dw791x->play_back, DRV_PLAY);
		led_vib_timer_start(index);
	}
	return count;
}

static ssize_t duration_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int index=-1;
    struct dw791x_priv *dw791x=NULL;
    
	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
	//read duration
	return snprintf(buf, PAGE_SIZE, "Channel=0x%0x, %d ms\n", gChannel, dw791x->duration);
}

static ssize_t duration_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	int ret;
	int val;
	int index=-1;
    struct dw791x_priv *dw791x=NULL;

	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
	ret=kstrtouint(buf, 0, &val);
	if(ret <0) {
		DbgOut((DBL_INFO, "%s kstrtouint failed\n", __func__));
		return ret;
	}

	DbgOut((DBL_INFO, "%s:Channel=0x%0x,  %d\n", __func__,gChannel, val));
	dw791x->duration=val;
	
	return count;
}

static ssize_t state_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int index=-1;
    struct dw791x_priv *dw791x=NULL;
    
	index=channel_to_index(gChannel);
	if (index == -1)
		return -1;

    dw791x=dw791x_devs[index];
	//read state
	return snprintf(buf, PAGE_SIZE, "Channel=0x%0x, %d\n", gChannel);
}

static ssize_t state_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	return count;
}
static struct device_attribute dw791x_haptics_attrs[] = {
	__ATTR(activate, 0664, activate_show, activate_store), //for LED vibrator HAL 1.0
	__ATTR(duration, 0664, duration_show, duration_store), //for LED vibrator HAL 1.0
	__ATTR(state, 0664, state_show, state_store), //for LED vibrator HAL 1.0
//	__ATTR(rtp_input, 0664, rtp_input_show, rtp_input_store),
//	__ATTR(mode, 0664, mode_show, mode_store),
//	__ATTR(scale, 0664, scale_show, scale_store),
//	__ATTR(ctrl_loop, 0664, ctrl_loop_show, ctrl_loop_store),
//	__ATTR(set_sequencer, 0664, NULL, set_sequencer_store),
//	__ATTR(lp_trigger_effect, 0664, lp_trigger_effect_show, lp_trigger_effect_store),
	__ATTR(channel, 0664, channel_show, channel_store),//hidl, ims use
	__ATTR(vd_clamp, 0664, vd_clamp_show, vd_clamp_store),//hidl, ims use
	__ATTR(factory_test, 0664, factory_test_show, factory_test_store),
	__ATTR(bst_option, 0664, bst_option_show, bst_option_store),
	__ATTR(trigger_test, 0664, NULL, trigger_test_store),
	__ATTR(pwm, 0664, pwm_show, pwm_store),
	__ATTR(status_reg, 0664, status_reg_show, status_reg_store),
	__ATTR(mem_play, 0664, NULL, mem_play_store),//hidl
	__ATTR(trigger_gpio, 0664, NULL, trigger_gpio_store),//hidl
	__ATTR(rtp_break, 0664, rtp_break_show, rtp_break_store),//hidl, ims use
	__ATTR(register, 0664,  register_show, register_store),
	__ATTR(vib_enable, 0664, vib_enable_show, vib_enable_store),//hidl
	__ATTR(rtp_play, 0664, NULL, rtp_play_store),//hidl, ims use
	__ATTR(debuglog, 0664, debuglog_show, debuglog_store),
	__ATTR(resample_control, 0664, resample_control_show, resample_control_store),
//	__ATTR(enable_gpio, 0664, enable_gpio_show, enable_gpio_store),//hidl
	__ATTR(rtp_play_repeat, 0664, NULL, rtp_play_repeat_store),
	__ATTR(trigger_level, 0664, NULL, trigger_level_store),
	__ATTR(trigger_waveform, 0664, trigger_waveform_show, trigger_waveform_store),//hidl
//	__ATTR(rtp_play_file, 0664, NULL, rtp_play_file_store),
};

static int getActuatorIndex(struct i2c_client* client){
	int index=-1;
	if(!client) return -1;
    //DbgOut((DBL_ERROR, "%s:bus[0]=%d, bus[1]=%d\n", __func__, gBus[0], gBus[1]));
	//if(gBus[0]!=gBus[1])//two buses
	{
		int bus=i2c_adapter_id(client->adapter);
		//DbgOut((DBL_ERROR, "%s:two buses, bus=%d\n", __func__, bus));
		index=(bus==gBus[0] ? 0:1);
	}
	//else{
	//	int addr=client->addr;
		//DbgOut((DBL_ERROR, "%s:one bus, two addresses, addr=0x%02x\n", __func__, addr));
	//	index=(addr==DEVICE_ADDR_1 ? 0:1);
	//}
	//DbgOut((DBL_ERROR, "%s:index=%d\n", __func__, index));
    return index;
}

static int dw791x_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
    uint8_t status = 0;
    uint8_t result = 0;
    int i=0;
    int bus=i2c_adapter_id(client->adapter);
    int addr=client->addr;
	struct dw791x_priv *dw791x = NULL;

	//get system i2c status
	//#if BOT_IS_INDEX_0
	//if bot_vib is index 0
    //gActuatorIndex--;
    //#else
	//if top_vib is index 0
    //gActuatorIndex++;
    //#endif
	gBus[gActuatorIndex]=bus;
	gAddr[gActuatorIndex]=addr;
	//gActuatorIndex++;
    //end
    DbgOut((DBL_ERROR, "%s:bus=%d, addr=0x%0x , gActuatorIndex=%d+++\n", __func__, bus, addr, gActuatorIndex));
    dw791x = kzalloc(sizeof(struct dw791x_priv), GFP_KERNEL);
	dw791x_devs[gActuatorIndex]=dw791x;
    if (!dw791x){
        //return -ENOMEM;
		DbgOut((DBL_ERROR, "%s:dw791x allocate failed, bus=%d, addr=0x%0x ---\n", __func__, bus, addr));
        return 0;
	}
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        DbgOut((DBL_ERROR, "dw791x_probe: i2c_check_functionality failed.\n"));
        //return -ENODEV;
    }
	
	i2c_set_clientdata(client, dw791x);
	mutex_init(&dw791x->dev_lock);
    dw791x->i2c = client;
	//mdelay(100);

    if (dw791x->i2c) {
		
		//set PM8150B gpio1 to enable dw7914 and trigger gpio
		if(!gPmic_Gpio_Config){
			dw791x_parse_dt(dw791x->i2c, dw791x);
#ifdef INT_EVENT_OPT			
			dw791x_pinctrl_init(dw791x->i2c, dw791x);
#endif			
			init_gpio(dw791x);
#ifdef INT_EVENT_OPT
			init_irq_rtp(dw791x);
#endif
			gPmic_Gpio_Config=true;
		}
		//end
		
        /* get device id */
        if (2 != i2c_recv_buf(client, DW791x_PID, &dw791x->chipid, 1)) {
            DbgOut((DBL_ERROR, "dw791x_probe: failed to read chipid.\n"));
            dw791x->i2c_status=0;
            /*return -ENODEV;*/
        }
        else
			dw791x->i2c_status=1;

		dw791x_device_init(client);
			INIT_WORK(&(dw791x->rtp_work), bot_vib_rtp_play_work_func);
			dw791x->rtp_workqueue = create_singlethread_workqueue("bot_vib_rtp_play_work_queue");
		//vibrator driver interface
		if(!gVibIF){
			//dw7914 sysfile
			//dw791x_vibrator_sysfs_init();
			dw791x_vibrator_control_sysfs_init();
			gVibIF=true;
			dw791x->cdev.name="vibrator";
			dw791x->cdev.brightness_get = dw791x_brightness_get;
			dw791x->cdev.brightness_set = dw791x_brightness_set;
			dw791x->cdev.max_brightness = 100;
			result = devm_led_classdev_register(&dw791x->i2c->dev, &dw791x->cdev);
			if (result < 0) {
				DbgOut((DBL_ERROR, "%s: devm_led_classdev_register failed\n", __func__));
			}
			//dw7914 post initial
			dw791x->activate=0;
			dw791x->duration=0;
			//dw791x->state=0;
			//dw791x->mode=0;
			//dw791x->set_sequencer=0;
			//dw791x->scale=0;
			//dw791x->ctrl_loop=0;
			//dw791x->lp_trigger_effect=0;
			//=======================================
			hrtimer_init(&dw791x->led_vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			dw791x->led_vib_timer.function = led_vib_timer_callback;
			//=======================================
			//end
			
			//create sysfile
			for(i=0;i<ARRAY_SIZE(dw791x_haptics_attrs);i++) {
				result = sysfs_create_file(&dw791x->cdev.dev->kobj, &dw791x_haptics_attrs[i].attr);
				if(result < 0) {
					DbgOut((DBL_ERROR, "nrmo1210 sysfs create failed\n"));
				}
			}
		}
		//end
		
        /* read any pending error codes */
        result = i2c_recv_buf(dw791x->i2c, DW7914_STATUS0, &status, sizeof(status));
        if (result != 2) {
            DbgOut((DBL_ERROR, "dw791x_probe: i2c read status0 failure. result=%d\n", result));
        }
        result = i2c_recv_buf(dw791x->i2c, DW7914_STATUS1, &status, sizeof(status));
        if (result != 2) {
            DbgOut((DBL_ERROR, "dw791x_probe: i2c read status1 failure. result=%d\n", result));
        }

        /* diagnostic */
        //dw791x_dump_registers(dw791x);
    }
	
    //dw791x->buffer[0] = DW7914_RTP_INPUT;
    //trigger pin waveform configuration moved to dw791x_vibrator_control
    //and called by init.asus.boot_vibrate.sh
	//end
    DbgOut((DBL_ERROR, "%s:bus=%d, addr=0x%0x ---\n", __func__, bus, addr));

    return 0;
}

static void dw791x_shutdown(struct i2c_client* client)
{
    DbgOut((DBL_VERBOSE, "dw791x_shutdown.\n"));

    return;
}

static int dw791x_remove(struct i2c_client* client)
{
    DbgOut((DBL_VERBOSE, "dw791x_remove.\n"));

    return 0;
}

/*
** I2C Transfer Functions
*/

static int i2c_recv_buf(struct i2c_client *i2c, unsigned char reg, unsigned char *buf, int count)
{
    struct i2c_msg msg[2];

    msg[0].addr = i2c->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &reg;

    msg[1].addr = i2c->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = count;
    msg[1].buf = buf;

    return i2c_transfer(i2c->adapter, msg, 2);
}
static int i2c_send_buf(struct i2c_client *i2c, unsigned char *buf, int count)
{
    struct i2c_msg msg;

    msg.addr = i2c->addr;
    msg.flags = 0;
    msg.len = count;
    msg.buf = buf;

    return i2c_transfer(i2c->adapter, &msg, 1);
}
/* =====================================================================================
function : dw791x devices register write function
descript : devices id 0x59
version  : 1.0
release  : 2018.02.28
====================================================================================== */
static int dw791x_byte_write(struct i2c_client *client, u8 addr, u8 data)
{
	int ret;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];	

    ret = i2c_smbus_write_byte_data(client, addr, data);
	
    if (ret < 0) {
		DbgOut((DBL_INFO, "%s i2c byte write fail.!\n",dw791x->dev_name));
	}
	
	return ret;
}


/* =====================================================================================
function : dw791x devices register word write function
descript : devices id 0x59
version  : 1.0
release  : 2018.04.17
====================================================================================== */
static int dw791x_word_write(struct i2c_client *client, u8 addr, u32 data)
{
	int ret;
	u8 xbuf[3];
	struct i2c_msg xfer[1];
	struct i2c_client *i2c_fnc = client;

	memset(xbuf, 0, sizeof(xbuf));

	xbuf[0] = (u8)addr;			
	xbuf[1] = data >> 8;
	xbuf[2] = data;

	xfer[0].addr  = CHIP_ID;
	xfer[0].len   = 3;
	xfer[0].flags = 0;
	xfer[0].buf = xbuf;
	ret = i2c_transfer(i2c_fnc->adapter, xfer, 1);
	
	return ret;
}
/* =====================================================================================
function : dw791x devices register read function
descript : devices id 0x59
version  : 1.0
release  : 2018.02.28
====================================================================================== */
int dw791x_byte_read(struct i2c_client *client, u8 addr)
{
	int ret=-1;
	struct dw791x_priv *dw791x = NULL;
	int index=-1;
	
	if(!client) return ret;
	index=getActuatorIndex(client);
	if(index<0)
		DbgOut((DBL_INFO, "Bus error, i2c byte read fail.!\n"));
	dw791x =dw791x_devs[index];	
	ret = i2c_smbus_read_byte_data(client, addr);
	
    if (ret < 0) {
		DbgOut((DBL_INFO, "%s i2c byte read fail.!\n",dw791x->dev_name));
	}	
	
	return ret;
}

/* =====================================================================================
function : dw791x devices PWM
descript : PWM
version  : 1.0
release  : 2019.03.18
====================================================================================== */
static void dw791x_PWM_set(struct i2c_client *client, u32 val)
{
	int ret=0;
	u8 tmp=0;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];	
	
	tmp=dw791x_byte_read(client, DW7914_PWM);
	tmp=(tmp&0xfc) | (val&0x03);
	ret = dw791x_byte_write(client, DW7914_PWM, tmp);

	if(ret < 0){
		DbgOut((DBL_INFO, "%s i2c PWM set fail.!\n",dw791x->dev_name));
	}
}

/* =====================================================================================
function : dw791x devices PWM
descript : PWM
version  : 1.0
release  : 2019.03.18
====================================================================================== */
static int dw791x_PWM_get(struct i2c_client *client)
{
	int val=0;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];	
	

	val = dw791x_byte_read(client, DW7914_PWM)&0x03;

	DbgOut((DBL_INFO, "%s i2c PWM:0x%02x\n",dw791x->dev_name, val));
	return val;
}
/* =====================================================================================
function : dw791x devices vd clamp control option
descript : voltage control range from 0mV to 10200mV
version  : 1.0
release  : 2018.02.28
====================================================================================== */
static void dw791x_vd_clamp_set(struct i2c_client *client, u32 vol)
{
	u8 clamp;
	int ret=0;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];	
	
	clamp = vol / 40;

	ret = dw791x_byte_write(client, DW7914_VD_CLAMP, clamp);

	if(ret < 0){
		DbgOut((DBL_INFO, "%s i2c vd clamp set fail.!\n",dw791x->dev_name));
	}
}

/* =====================================================================================
function : dw791x devices boost control option
descript : pay attention to the settings
version  : 1.0
release  : 2018.02.28
====================================================================================== */
#ifdef BST_OPT
static void dw791x_bst_option(struct i2c_client *client, u32 lv, u32 limit, u32 ofs)
{
	u8 set=0;
	int ret=0;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];

	set = lv<<6 | limit << 4 | (ofs / 80);
	ret = dw791x_byte_write(client, DW7914_BST_OPTION, set);
	
	if(ret < 0){
		DbgOut((DBL_INFO, "%s i2c boost option set fail.!\n", dw791x->dev_name));
	}	
}
#endif

/* =====================================================================================
function : dw7914 fifo size set
descript : only DW7914 devices are supported.
		 : value(2) = mean(2KB) / value(4) = mean(4KB) / value(6) = mean(6KB)
version  : 1.0
release  : 2018.02.28
====================================================================================== */
static void dw791x_fifo_size_set(struct i2c_client *client, u32 size)
{
	int ret=0;
	u32 fifo_set;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];

	fifo_set = SRAM_BASE - (1024 * size + 4);
	DbgOut((DBL_INFO, "fifo size: %dbyte (0x%02x)\n", 1024 * size, fifo_set));
	ret = dw791x_word_write(client, dw791x->fifo_addrh, fifo_set);
	
	if(ret < 0) DbgOut((DBL_INFO, "%s i2c fifo size set fail.!\n", dw791x->dev_name));	
}

/* =====================================================================================
function : dw7914 mode setup
descript : only DW7914 devices are supported.
version  : 1.0
release  : 2018.02.28
====================================================================================== */
static void dw7914_mode_set(struct i2c_client *client, u8 type, u8 bitset)
{
	int ret, reg;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];

	reg = dw791x_byte_read(client, DW7914_MODE);

	if(bitset == BITSET) { reg = reg | type;
	}
	else { reg = reg & ~type;
	}
	
	ret = dw791x_byte_write(client, DW7914_MODE, reg);

	if(ret < 0) {
		DbgOut((DBL_INFO, "%s i2c mode register set fail.!\n", dw791x->dev_name));
	}
}


/* =====================================================================================
function : dw7914 trig ctrl setup
descript : only DW7914 devices are supported.
version  : 1.0
release  : 2018.02.28
====================================================================================== */
#ifdef TRIG_OPT
static void dw7914_trigctrl_set(struct i2c_client *client, u8 type, u8 bitset)
{
	int ret, reg;

	reg = dw791x_byte_read(client, DW7914_TRIG_CTRL);

	if(bitset == BITSET) { reg = reg | type;
	}
	else { reg = reg & (~type);
	}
	
	ret = dw791x_word_write(client, DW7914_TRIG_CTRL, reg);
	
	DbgOut((DBL_INFO, "trig ctrl register set: %x\n", reg));

	if(ret < 0) DbgOut((DBL_INFO, "i2c trig register set fail.!\n"));
}
#endif

/* =====================================================================================
function : dw7914 auto track & brake ctrl setup
descript : only DW7914 devices are supported.
version  : 1.0
release  : 2018.02.28
====================================================================================== */
static void dw7914_autovib_set(struct i2c_client *client)
{
	/*  Exmaple Design Info
	-	LRA F0 IN : 3F 00 AA 
	-	Track Pattern : 34 80 (follow setting value)
	-	Track Loop & Track0 cycle : 34 80
	-	Track_Play : 35 09 (106ms)
	-	Track_Idle : 36 00 (10.66ms)
	-	Track0 Wave : 37 00
	-	Track1 Wave : 38 01
	-	NULL=GND time : 3c 33 (100us)
	-	ZXD time : 3d 67 ( 450us)
	-	Actuator conversions range : 147Hz ~ 193Hz	
	*/

	// setp 0: ------ auto track wave transfer
	transfer_atuovib_wave(client, (u8*)auto_track_base_m1);


	// step 1: ------ control mode setting
	dw7914_mode_set(client, D14_AUTO_TRACK , BITSET);


	// step 2: ------ register setting
	dw791x_word_write(client, DW7914_LRA_F0_INH, 0xAA);
	dw791x_byte_write(client, DW7914_TRACK_CTRL0, 0x80);
	dw791x_byte_write(client, DW7914_TRACK_CTRL1, 0x09);
	dw791x_byte_write(client, DW7914_TRACK_CTRL2, 0x00);
	dw791x_byte_write(client, DW7914_TRACK0_WAVE, 0x00);
	dw791x_byte_write(client, DW7914_TRACK1_WAVE, 0x01);

	dw791x_byte_write(client, DW7914_ZXD_CTRL1, 0x33);
	dw791x_byte_write(client, DW7914_ZXD_CTRL2, 0x67);

}


/* =====================================================================================
function : dw7914 memory check sum function
descript : only DW7914 devices are supported.
		   - page : sram(12KB) memory model select
		   - return 0 (sucess)
		   - return -1 (fail)
version  : 1.0
release  : 2018.04.26
====================================================================================== */
static int dw7914_checksum(struct i2c_client *client, u32 type, u32 page)
{
	int ret=0;
	u8 *p=NULL, rx[4];
	u32 point, gsum;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];

	if(!dw791x->i2c_status) {
		DbgOut((DBL_INFO, "checksum vibrator %d i2c fail\n", getActuatorIndex(client)));
		return -1;
	}
	dw791x_fifo_size_set(client, 4);
	
	if(request_firmware(&dw791x->fw, FW_FILE, &dw791x->i2c->dev) == 0) 
	{		
		p = (u8*)dw791x->fw->data;
		point = (1 + page) * BIN_BASE;
		rx[3] = p[point-4];
		rx[2] = p[point-3];
		rx[1] = p[point-2];
		rx[0] = p[point-1];
		dw791x->checksum = (u32)rx[3]<<24|(u32)rx[2]<<16|(u32)rx[1]<<8|(u32)rx[0]<<0;
		DbgOut((DBL_INFO, "fw_name= %s fw_size= %ld fw_checksum= %x\n", FW_FILE, dw791x->fw->size, dw791x->checksum));
	} release_firmware(dw791x->fw);
	
	dw7914_mode_set(client, D14_CHKSUM, BITSET);
	msleep(20);
	rx[3] = dw791x_byte_read(client, DW7914_RAM_CHKSUM3);
	rx[2] = dw791x_byte_read(client, DW7914_RAM_CHKSUM2);
	rx[1] = dw791x_byte_read(client, DW7914_RAM_CHKSUM1);
	rx[0] = dw791x_byte_read(client, DW7914_RAM_CHKSUM0);
	gsum = (u32)rx[3]<<24|(u32)rx[2]<<16|(u32)rx[1]<<8|(u32)rx[0]<<0;

	if(gsum == dw791x->checksum) {
		ret = 0;
		DbgOut((DBL_INFO, "memory checksum passed!: %x\n", gsum));
	}
	else {
		ret = -1;
		DbgOut((DBL_INFO, "memory checksum failed!: %x, %x\n", gsum, dw791x->checksum));
	}
		
	return ret;

}


/* =====================================================================================
function : bulk i2c transfer support function
descript : transfer size can be adjusted.
version  : 1.0
release  : 2018.02.28
====================================================================================== */
static int dw791x_seq_write(struct i2c_client *client, u32 addr, u32 ram_addr, u32 ram_bit, u8* data, u32 size)
{
        
	int ret=0;
	struct i2c_msg xfer[1];
	struct i2c_client *i2c_fnc = client;

	if(size > 1027) { 
		ret = -1;
		DbgOut((DBL_INFO, "%s:The transferable size has been exceeded.\n", __func__));
		return ret;
	}

	if(ram_bit == RAM_ADDR0) { 
		dma_buf_resampling[0] = (u8)addr;	
		xfer[0].addr  = CHIP_ID;
		xfer[0].len   = size + 1;
		xfer[0].flags = 0;
		xfer[0].buf = dma_buf_resampling;
		memcpy((u8*)dma_buf_resampling + 1, (u8*)data, size);
	}
	else if(ram_bit == RAM_ADDR16) {
		dma_buf_resampling[0] = addr;
		dma_buf_resampling[1] = ram_addr >> 8;
		dma_buf_resampling[2] = (u8)ram_addr;
		
		xfer[0].addr  = CHIP_ID;
		xfer[0].len   = size + 3;
		xfer[0].flags = 0;
		xfer[0].buf = dma_buf_resampling;
		memcpy((u8*)dma_buf_resampling + 3, (u8*)data, size);
	}

	ret = i2c_transfer(i2c_fnc->adapter, xfer, 1);
	return ret;
}
#if 0
/* =====================================================================================
function : bulk i2c transfer support function
descript : transfer size can be adjusted.
version  : 1.0
release  : 2018.02.28
====================================================================================== */
static int dw791x_seq_read(struct i2c_client *client, u32 addr, u32 ram_addr, u32 ram_bit, u8* data, u32 size)
{
        
	int ret=0;
	struct i2c_msg xfer[1];
	struct i2c_client *i2c_fnc = client;

	if(size > 1027) { 
		ret = -1;
		DbgOut((DBL_INFO, "The transferable size has been exceeded.\n"));
		return ret;
	}

	if(ram_bit == RAM_ADDR0) { 
		dma_buf_resampling[0] = (u8)addr;	
		xfer[0].addr  = CHIP_ID;
		xfer[0].len   = size + 1;
		xfer[0].flags = 0;
		xfer[0].buf = dma_buf_resampling;
		memcpy((u8*)dma_buf_resampling + 1, (u8*)data, size);
	}
	else if(ram_bit == RAM_ADDR16) {
		dma_buf_resampling[0] = addr;
		dma_buf_resampling[1] = ram_addr >> 8;
		dma_buf_resampling[2] = (u8)ram_addr;
		
		xfer[0].addr  = CHIP_ID;
		xfer[0].len   = size ;
		xfer[0].flags |= I2C_M_RD;
		xfer[0].buf = dma_buf_resampling;
	}

	ret = i2c_transfer(i2c_fnc->adapter, xfer, 1);
	return ret;
}
#endif
/* =====================================================================================
function : transfer auto track brake memory wave
descript : only DW7914 devices are supported.
         : header(pure header 5byte), body(pure wave data), not include memory address.
version  : 1.0
release  : 2018.02.28
====================================================================================== */
static int transfer_atuovib_wave(struct i2c_client *client, u8* wave)
{		
	int ret, i, loop, tail, wave_size;
	int start_addr, set_addr, trans_size;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];
	
	if(!dw791x->i2c_status){
		DbgOut((DBL_INFO, "transfer_atuovib_wave vibrator %d i2c fail \n", getActuatorIndex(client)));
		return -1;
	}

	// setp 0: ---------------------------- dw7914 memory mode set
	ret = 0;
	dw7914_mode_set(client, D14_MEM_MODE, MEM);
		
 	// setp 1: ---------------------------- dw7914 header data transfer
	set_addr = (u32)wave[0]<<8 | wave[1];
	dw791x_seq_write(client, dw791x->mem_input, set_addr, RAM_ADDR16, (u8*)wave + 2, 5);
	
	// step 2: ---------------------------- dw7914 body data transfer
	trans_size = I2C_TRANS_MEM;
	set_addr = (int)wave[2] << 8 | wave[3];
	wave_size = (int)(0xF & wave[4]) << 8 | wave[5];
	
	DbgOut((DBL_INFO, "auto vib body wave size: %d\n",wave_size));
	
	loop = wave_size / trans_size;
	tail = wave_size % trans_size;
	
	for(i=0; i<loop; i++) {		
		start_addr = set_addr + i * trans_size;		
		dw791x_seq_write(client, dw791x->mem_input, start_addr, RAM_ADDR16, (u8*)wave + 7 + i * trans_size, trans_size);
	}

	if(tail > 0) {
		start_addr = set_addr + loop * trans_size;		
		dw791x_seq_write(client, dw791x->mem_input, start_addr, RAM_ADDR16, (u8*)wave + 7 + loop * trans_size, tail);
	}
	
	DbgOut((DBL_INFO, "auto vib memory data write done!\n"));
	
	return ret;
	
}


/* =====================================================================================
function : request transfer rtp wave function
descript : Only DW7914 devices are supported.
header   : 5byte[0xAE, size, size, size, size] total size 32bit
data     : rtp wave data
version  : 1.0
release  : 2018.04.17
====================================================================================== */
static int request_transfer_rtp_wave(struct i2c_client *client, u8* wave, u32 repeat)
{
	u8 rx[2];
	u32 ret, run, curcnt, trans_size, play_size;
	u32 empty_lock, fifo_level, fifo_size;
	u32 wait_t;
    int bus;
    int addr;
    int index=getActuatorIndex(client);
	struct dw791x_priv *dw791x;

	bus=i2c_adapter_id(client->adapter);
	addr=client->addr;
	dw791x=dw791x_devs[index];
	
	if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "request_transfer_rtp_wave vibrator %d i2c fail!\n", getActuatorIndex(client)));
		return -1;
	}
    if(dw791x->bRTP_Play_Break){
        DbgOut((DBL_ERROR, "(%s) (0) (%s)***An event occurred during rtp operation! bRTP_Play_Break = true***\n",(index==0)?"Bot_VIB":"Top_VIB", dw791x->fwName));
        return -1;
    }
	if(gVibDebugLog&0x0010)//store node debug
		DbgOut((DBL_ERROR, "%s: (%s) (%s) +++\n", __func__, (index==0)?"Bot_VIB":"Top_VIB", dw791x->fwName));
	// step 0: ------------------ timer set check & clear & size read
	
	//This is delay time between the current vibe and the next vibe by trigger input.
	//bit[3-1]: 010 : 10ms 
	dw791x_byte_write(client, 0x03, 0x04);	// jks ---
	
	run = curcnt = empty_lock = fifo_level = 0;
	play_size = (u32)wave[1]<<24|(u32)wave[2]<<16|(u32)wave[3]<<8|(u32)wave[4]<<0;
	rx[1] = dw791x_byte_read(client, DW7914_STATUS1);
	
	//dw7914_mode_set(client, D14_FIFO_FLUSH, BITSET);
	//dw7914_mode_set(client, D14_MEM_MODE, RTP);
	dw791x_byte_write(client, 0x0B, 0x20);//FIFO_FLUSH and RTP Mode
	dw791x_byte_read(client, DW7914_STATUS0);
	dw791x_byte_read(client, DW7914_STATUS1);
	dw791x_byte_write(client, DW7914_PWM, gPWM);//48Khz
	// step 1: ------------------ fifo set size read
	//DbgOut((DBL_INFO, "%s(VIB %d): step 1: ------------------ fifo set size read\n", __func__, index));
	rx[0] = dw791x_byte_read(client, dw791x->fifo_addrh);//0x49
	rx[1] = dw791x_byte_read(client, dw791x->fifo_addrl);//0x4a
	fifo_size = SRAM_BASE - ((u32)rx[0] << 8 | rx[1]);
	fifo_level=dw791x_byte_read(client, DW7914_FIFO_LEVEL_SETH);//0x4b fifo size??
	//DbgOut((DBL_INFO, "(%s):I2C_TRANS_RTP=%d, fifo_size=%d, fifo_level=%d(%d)bytes\n", (index==0)?"Bot_VIB":"Top_VIB", I2C_TRANS_RTP, fifo_size, fifo_level, fifo_level*256));
	DbgOut((DBL_INFO, "I2C_TRANS_RTP=%d, fifo_size=%d, fifo_level=%d(%d)bytes\n", I2C_TRANS_RTP, fifo_size, fifo_level, fifo_level*256));

	// step 2 : ----------------- short time play
	ret=0;
	if(!dw791x->bRTP_Play_Break && play_size < I2C_TRANS_RTP ) {
		if(gVibDebugLog&0x0010)//store node debug
			DbgOut((DBL_ERROR, "%s: (%s) (%s)write to fifo, play_size: %d\n", __func__, (index==0)?"Bot_VIB":"Top_VIB", dw791x->fwName,play_size));

		ret = dw791x_seq_write(client, dw791x->rtp_input, 0, RAM_ADDR0, (unsigned char*)wave + 5, play_size);
		if(ret<0) 
			DbgOut((DBL_ERROR, "(%s)write to fifo failed!! Code:%d\n", (index==0)?"Top_VIB":"Bot_VIB", ret));

		if(gVibDebugLog&0x0010)//store node debug
			DbgOut((DBL_ERROR, "%s: (%s) (%s)Go play...\n", __func__, (index==0)?"Bot_VIB":"Top_VIB", dw791x->fwName));

		ret = dw791x_byte_write(client, dw791x->play_back, DRV_PLAY);
		if(ret<0) 
			DbgOut((DBL_ERROR, "(%s)go play failed!! Code:%d\n", (index==0)?"Top_VIB":"Bot_VIB", ret));
		return ret;
	}
	//dw791x_dump_registers(dw791x);
	//DbgOut((DBL_INFO, "==============================================================\n"));
	while(!run)
	{		
        if(dw791x->bRTP_Play_Break){
            DbgOut((DBL_ERROR, "(%s) (2) (%s)***An event occurred during rtp operation! bRTP_Play_Break = true***\n",(index==0)?"Bot_VIB":"Top_VIB", dw791x->fwName));
            return -1;
        }
		// step 3: ------------------ fifo level check		
		rx[0] = dw791x_byte_read(client, 0x4D);// current fifo status
		rx[1] = dw791x_byte_read(client, 0x4E);
		fifo_level = ((u32)rx[0] << 8) | rx[1];			

		// step 4: ------------------ i2c transfer size define
		if((fifo_size - fifo_level) > I2C_TRANS_RTP) { 
			trans_size = I2C_TRANS_RTP;
		}
		else {
			trans_size = 0;//fifo_size - fifo_level;
		}
			
		// step 5: ------------------ wave data transfer loop
		if(curcnt < play_size) 
		{
			if((curcnt + trans_size) > play_size) {
				trans_size = play_size - curcnt;
			}			

			if(trans_size > 0) {
                if(dw791x->bRTP_Play_Break){
                    DbgOut((DBL_ERROR, "(%s) (3) (%s)***An event occurred during rtp operation! bRTP_Play_Break = true***\n",(index==0)?"Bot_VIB":"Top_VIB", dw791x->fwName));
                    return -1;
                }
				if(gVibDebugLog&0x0010)//store node debug
					DbgOut((DBL_ERROR, "(fifo=%d trans=%d)",fifo_level, trans_size));

				ret = dw791x_seq_write(client, dw791x->rtp_input, 0, 0, (unsigned char*)wave + 5 + curcnt, trans_size);	// fifo full write
				if(ret<0) DbgOut((DBL_ERROR, "(%s)i2c error!! Code:%d\n", (index==0)?"Top_VIB":"Bot_VIB", ret));
				curcnt += trans_size;
			}

			if(!dw791x->bRTP_Play_Break && empty_lock == 0) { 
				rx[0] = dw791x_byte_read(client, 0x4D);
				rx[1] = dw791x_byte_read(client, 0x4E);
				fifo_level = ((u32)rx[0] << 8) | rx[1];			
				if(gVibDebugLog&0x0010)//store node debug
					DbgOut((DBL_ERROR, "(%s) fifo_level=%d, fifo_size/2=%d, play_size/2=%d",__func__, fifo_level, fifo_size/2, (play_size / 3)));
				if(fifo_level > (fifo_size / 3) || (fifo_level > (play_size / 3))) 
				{
					empty_lock = 1;
					if(gVibDebugLog&0x0010)//store node debug
						DbgOut((DBL_ERROR, "(%s)Go bit...\n", __func__));
					dw791x_byte_write(client, dw791x->play_back, DRV_PLAY);

					// if not currently playing, start playing
					ret = (0x01 & dw791x_byte_read(client, DW7914_STATUS1));
					if (!ret) {
						DbgOut((DBL_ERROR, "%s: Set Go bit and Play, again ...\n", __func__));
						dw791x_PWM_set(client, gPWM);
						dw791x_byte_write(client, dw791x->play_back, DRV_PLAY);
					}
				}
			} else if (fifo_level == I2C_TRANS_RTP) {
				DbgOut((DBL_ERROR, "%s: toggle Go bit for fifo is cleared. \n", __func__));
				dw791x_byte_write(client, dw791x->play_back, DRV_PLAY);
			}


			if(dw791x->bRTP_Play_Break){//if want to stop playing
                DbgOut((DBL_ERROR, "(%s) (4) (%s)***An event occurred during rtp operation! bRTP_Play_Break = true***\n",(index==0)?"Bot_VIB":"Top_VIB", dw791x->fwName));
                ret=-1;
			}
			if(fifo_level > (fifo_size - fifo_size / 3))	// wait interrupt delay
			{
				//DbgOut((DBL_INFO, "%s(%s): step 5: ------------------ FIFO Full:wait interrupt delay 3ms\n\n", __func__, (index==0)?"Bot_VIB":"Top_VIB"));
				//DbgOut((DBL_INFO, "%s(%s): step 5: ------------------ wave data transfer loop:wait interrupt delay 5ms\n\n", __func__, (index==0)?"Bot_VIB":"Top_VIB"));
				wait_t = msecs_to_jiffies(10);				// set 10ms fixed
				schedule_timeout_interruptible(wait_t);
			}	
			
			//DbgOut((DBL_INFO, "==============================================================\n"));
		}

		// step 6: ------------------ wave data transfer done!
		else {	// play stop
			if(repeat > 1) {
				DbgOut((DBL_ERROR, "(%s) (%s)The rtp operation size: %d, repeat: %d\n", (index==0)?"Bot_VIB":"Top_VIB", dw791x->fwName, play_size, repeat));
				curcnt = 0;
				repeat--;
			}
			else {
				run = 1;
				DbgOut((DBL_ERROR, "(%s) (%s)The rtp operation is finished. size: %d, repeat: %d\n", (index==0)?"Bot_VIB":"Top_VIB", dw791x->fwName, curcnt, repeat));
				dw7914_mode_set(client, D14_FIFO_FLUSH, BITSET);
				break;	
			}
			
		}
	} // end while
	if(gVibDebugLog&0x0010)//store node debug
		DbgOut((DBL_ERROR, "%s: (%s) (%s) ---\n", __func__, (index==0)?"Bot_VIB":"Top_VIB", dw791x->fwName));
	return ret;
}
/* =====================================================================================
function : request transfer rtp mem function
descript : only use binary waves.  memory addresses not included.
page	 : 12kb bin file model start page 0 ~ n
point	 : memory header call number 0: all wirte, 1: header no.1
pw_data  : bin file data pointer
size	 : bin wave size
version  : 1.0
release  : 2018.04.28
====================================================================================== */

static int request_transfer_mem_wave(struct i2c_client *client, u32 page, u32 point, u8* fw_data, u32 size)
{		
	int ret=0;
	u32 i, loop, tail, wave_size, start_addr, set_addr, trans_size, fw_point;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];

	if(!dw791x->i2c_status){
		DbgOut((DBL_INFO, "request_transfer_mem_wave vibrator %d i2c fail \n", getActuatorIndex(client)));
		return -1;
	}
	
	// setp 0: ---------------------------- dw791x memory mode set
	trans_size = I2C_TRANS_MEM;
	ret = dw791x_byte_write(client, dw791x->play_mode, MEM);

	if(ret < 0) return ret;
		
	// step 1: ---------------------------- dw791x find header data pointer
	if(point > 0) {
		set_addr = (point * 5) - 4;
		fw_point = set_addr + BIN_BASE * page;
		dw791x_seq_write(client, dw791x->mem_input, set_addr, RAM_ADDR16, (u8*)fw_data + fw_point, 5);

		set_addr = (u32)(fw_data[fw_point + 0]) << 8 | fw_data[fw_point + 1];
		wave_size = (u32)(0xF & fw_data[fw_point + 2]) << 8 | fw_data[fw_point + 3];		
	}
	else {
		set_addr = 0;
		wave_size = BIN_BASE;
		fw_point = (1 + page) * BIN_BASE;
	}
	DbgOut((DBL_INFO, "request trans page: %d point: %d size: %d\n",page, point, wave_size));


	// step 2: ---------------------------- dw791x find body data pointer
	loop = wave_size / trans_size;
	tail = wave_size % trans_size;
	DbgOut((DBL_INFO, "set addr:%d  loop:%d  tail:%d\n", set_addr, loop, tail));
	
	for(i=0; i<loop; i++) {
		start_addr = set_addr + (i * trans_size);
		fw_point = start_addr + BIN_BASE * page;
		dw791x_seq_write(client, dw791x->mem_input, start_addr, RAM_ADDR16, (u8*)fw_data + fw_point, trans_size);
	}

	if(tail > 0) {
		start_addr = set_addr + (loop * trans_size);
		fw_point = start_addr + BIN_BASE * page;
		dw791x_seq_write(client, dw791x->mem_input, start_addr, RAM_ADDR16, (u8*)fw_data + fw_point, tail);
	}
	
	DbgOut((DBL_INFO, "requeset firmware transfer complete!\n"));
	
	return ret;
	
}

/* =====================================================================================
function : dw791x devices default setting function
descript : auto detected devices model
version  : 1.0
release  : 2018.02.28
====================================================================================== */
static int dw791x_device_init(struct i2c_client *client)
{
	int ret=0, val=0;
    int bus=i2c_adapter_id(client->adapter);
    int addr=client->addr;
	struct dw791x_priv *dw791x =dw791x_devs[getActuatorIndex(client)];
	
	if(dw791x!=NULL){
		dw791x->play_mode = DW7914_MODE;
		dw791x->play_back = DW7914_PLAYBACK;
		dw791x->rtp_input = DW7914_RTP_INPUT;
		dw791x->mem_input = DW7914_RAM_ADDR;
		dw791x->fifo_addrh = DW7914_FIFO_ADDRH;
		dw791x->fifo_addrl = DW7914_FIFO_ADDRL;
		dw791x->fifo_level = DW7914_FIFO_LEVEL;
		snprintf(dw791x->dev_name, 8, "DW7914A");
		dw791x->version = 0x7914F002;
		dw791x_fifo_size_set(client, FIFO_4KB);
		//dw791x_fifo_size_set(client, FIFO_2KB);
		//dw791x_fifo_size_set(client, FIFO_8KB);
		//dw791x_byte_write(client, 0x02, 0xff);//INTZ_EN, enable all interrupt notification
		//dw791x_byte_write(client, 0x02, 0x2f);//INTZ_EN, FIFO_EMPTY, UVLO, TSD, OCP, SCP
		dw791x_byte_write(client, 0x02, 0xcf);//INTZ_EN, FIFO_FULL, FIFO_EMPTY, UVLO, TSD, OCP, SCP
		//dw791x_byte_write(client, 0x02, 0x0f);//INTZ_EN, UVLO, TSD, OCP, SCP
		//dw791x_byte_write(client, DW7914_PWM, gPWM);//48Khz
		DbgOut((DBL_INFO, "Bus=%d, Addr=0x%0x, DWA %s driver version %x\n", bus, addr, dw791x->dev_name, dw791x->version));	
		dw791x_bst_option(dw791x->i2c, 0, 0, 0);	//default
		val=(getActuatorIndex(client)==0) ? DEFAULT_BOT_VD_CLAMP : DEFAULT_TOP_VD_CLAMP;//set default vd_clamp
        //dw791x->vd_clamp=val;
		dw791x_vd_clamp_set(dw791x->i2c, val);
		//others
		dw791x->bRTP_Play_Break=false;
	}
	else
		ret=-1;
	return ret;	
}

//=================================================================================
//Immersion function
//=================================================================================

/*
** Called to disable amp (disable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{
	struct dw791x_priv *dw791x =dw791x_devs[nActuatorIndex];

	if(dw791x==NULL){
		DbgOut((DBL_INFO, "%s:nActuatorIndex=%d , i2c fail \n", __func__, nActuatorIndex));
		return VIBE_E_FAIL;
	}
	if(!dw791x->i2c_status){
		DbgOut((DBL_INFO, "%s: nActuatorIndex:%d i2c fail.\n", __func__, nActuatorIndex));
		return VIBE_E_FAIL;
	}
	if(gVibDebugLog&0x0004)//enable immersion message
		DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_AmpDisable ActuatorIndex:%d.\n", nActuatorIndex));
	if (dw791x->buffering) {
		dw791x->buffering = -1;
		//ImmVibeSPI_ForceOut_SetSamples(nActuatorIndex, 8, VIBE_OUTPUT_SAMPLE_SIZE, 0);
		//clean driver status
		dw791x->bRTP_Play_Break=false;
		dw791x_byte_write(dw791x->i2c, dw791x->play_back, DRV_STOP);
		dw7914_mode_set(dw791x->i2c, D14_FIFO_FLUSH, BITSET);
		//end
	}
    return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
#define DW7914_MODE                    0x0B
#define DW7914_MODE_MASK               0x03
#define DW7914_MODE_RTP                0x00
#define DW7914_MODE_FIFO_FLUSH         0x20
#define DW7914_STATUS1_ACTIVE          0x01
#define DW7914_PWM_48KHZ               0x01

IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{
    int result;
    uint8_t status;
    struct dw791x_priv *dw791x=dw791x_devs[nActuatorIndex];
    int mode;
    
    /*
	if(!bEnable_Immrvib){
		DbgOut((DBL_INFO, "%s:bEnable_Immrvib=%s\n", __func__, (bEnable_Immrvib==1)?"true":"false"));
		return VIBE_S_SUCCESS;
	}
	* */
	if(dw791x==NULL){
		DbgOut((DBL_INFO, "%s:nActuatorIndex=%d , i2c fail \n", __func__, nActuatorIndex));
		return VIBE_E_FAIL;
	}
	if(!dw791x->i2c_status){
		DbgOut((DBL_INFO, "%s: ActuatorIndex:%d i2c fail.\n", __func__, nActuatorIndex));
		return VIBE_E_FAIL;
	}
	if(gVibDebugLog&0x0004)//enable immersion message
		DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_AmpEnable ActuatorIndex:%d.\n", nActuatorIndex));
    dw791x->buffering = 1;
    dw791x->position = 1;
	/* read any pending error codes */
	result = i2c_recv_buf(dw791x->i2c, DW7914_STATUS0, &status, sizeof(status));
	if (result != 2) {
		DbgOut((DBL_ERROR, "dw7914: i2c read status failure. result=%d\n", result));
	}
	if (status & 0x0F) {
		DbgOut((DBL_ERROR, "dw7914: status error! (0x%02x)\n", status));
	}		
	//check mode +++
	mode = DW7914_MODE_MASK & dw791x_byte_read(dw791x->i2c, DW7914_MODE);
	if (mode != DW7914_MODE_RTP) {
		DbgOut((DBL_ERROR, "%s: ActuatorIndex:%d, DW7914_MODE_RTP|DW7914_MODE_FIFO_FLUSH\n", __func__, nActuatorIndex));
		dw791x_byte_write(dw791x->i2c, DW7914_MODE, DW7914_MODE_RTP | DW7914_MODE_FIFO_FLUSH);
		dw791x_PWM_set(dw791x->i2c, gPWM);
    }
    //check mode ---
    dw791x->bRTP_Play_Break=true;
    return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
    int retVal = 0;

    DbgOut((DBL_ERROR, "%s:+++\n", __func__));
    
    //allocate DMA buffer
    if(!dma_buf_resampling){
		dma_buf_resampling=kmalloc(1024*8+1, GFP_DMA | GFP_KERNEL); //1028*4+1, 12k upto 48k
		if(dma_buf_resampling){
			memset(dma_buf_resampling, 0, 1024*8+1);
		}
		else{
			DbgOut((DBL_ERROR, "The buffer dma_buf_resampling kmalloc failed.\n"));
		}
	}
	if(!dma_buf_ivt){
		dma_buf_ivt=kmalloc(1024*8+1, GFP_DMA | GFP_KERNEL);
		if(dma_buf_ivt){
			memset(dma_buf_ivt, 0, 1024*8+1);
		}
		else{
			DbgOut((DBL_ERROR, "The buffer dma_buf_ivt kmalloc failed.\n"));
		}
	}
	//end
	
    retVal = i2c_add_driver(&dw791x_i2c_driver);
    if (retVal) {
        DbgOut((DBL_ERROR, "ImmVibeSPI_ForceOut_Initialize: Cannot add driver.\n"));
    }
    
    DbgOut((DBL_ERROR, "%s:---\n", __func__));
    return VIBE_S_SUCCESS;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
//#error Please review the code between the #if and #endif

    DbgOut((DBL_INFO, "ImmVibeSPI_ForceOut_Terminate.\n"));

#if 0   /* The following code is provided as sample. Please modify as required. */
    VibeStatus nActuatorIndex;  /* Initialized below. */

    /* For each actuator... */
    for (nActuatorIndex = 0; NUM_ACTUATORS > nActuatorIndex; ++nActuatorIndex)
    {
        /* Disable amp */
        ImmVibeSPI_ForceOut_AmpDisable(nActuatorIndex);
    }

    /* Set PWM frequency for actuator 0 */
    /* To be implemented with appropriate hardware access macros */

    /* Set duty cycle to 50% for actuator 0 */
    /* To be implemented with appropriate hardware access macros */

    /* Set PWM frequency for actuator 1 */
    /* To be implemented with appropriate hardware access macros */

    /* Set duty cycle to 50% for actuator 1 */
    /* To be implemented with appropriate hardware access macros */
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    struct dw791x_priv *dw791x=dw791x_devs[nActuatorIndex];
    //int val=0;
    //int mode, active;
    int active;
	if(dw791x==NULL){
		DbgOut((DBL_INFO, "%s:nActuatorIndex=%d , i2c fail \n", __func__, nActuatorIndex));
		return VIBE_E_FAIL;
	}
	if(!dw791x->i2c_status){
		DbgOut((DBL_INFO, "%s:nActuatorIndex=%d , i2c fail \n", __func__, nActuatorIndex));
		return VIBE_E_FAIL;
	}
    /* 
    ** For ERM/LRA:
    **      nBufferSizeInBytes should be equal to 1 if nOutputSignalBitDepth is equal to 8
    **      nBufferSizeInBytes should be equal to 2 if nOutputSignalBitDepth is equal to 16
    **
    ** For Piezo:
    **      nBufferSizeInBytes is typically equal to 40 if nOutputSignalBitDepth is equal to 8
    **      nBufferSizeInBytes is typically equal to 80 if nOutputSignalBitDepth is equal to 16
    */
	if(gVibDebugLog&0x0004)//enable immersion message
	{
		DbgOut((DBL_ERROR, "%s: nActuatorIndex=%d +++\n", __func__, nActuatorIndex));
	}
#ifdef IMMVIBESPI_USE_BUFFERFULL
    /* write zeros to keep hardware buffer full and in sync with driver */
    if (nBufferSizeInBytes != VIBE_OUTPUT_SAMPLE_SIZE) {
        static VibeInt8 buffer[VIBE_OUTPUT_SAMPLE_SIZE] = {0,};
        pForceOutputBuffer = buffer;
        nBufferSizeInBytes = sizeof(buffer);
        nOutputSignalBitDepth = 8;
    }
#endif
    if (VIBE_S_SUCCESS != I2CWriteWithResendOnError_DW7914(dw791x, nBufferSizeInBytes, pForceOutputBuffer)) {
		DbgOut((DBL_ERROR, "%s: nActuatorIndex=%d failed ---\n", __func__, nActuatorIndex));
        return VIBE_E_FAIL;
    }
	// if not currently playing, start playing
    active = DW7914_STATUS1_ACTIVE & dw791x_byte_read(dw791x->i2c, DW7914_STATUS1);
    if (!active) {
		DbgOut((DBL_ERROR, "%s: Set Go bit and Play ...\n", __func__));
        dw791x_PWM_set(dw791x->i2c, gPWM);
        dw791x_byte_write(dw791x->i2c, dw791x->play_back, DRV_PLAY);
    }
    if(gVibDebugLog&0x0004)//enable immersion message
		DbgOut((DBL_ERROR, "%s: nActuatorIndex=%d ---\n", __func__, nActuatorIndex));
    return VIBE_S_SUCCESS;
}
VibeStatus I2CWriteWithResendOnError_DW7914(struct dw791x_priv *dw791x, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    int nDataIndex = 0;
    char pktsize = 0;
    #if 0
    int waitDuration = 0;
    u32 datacnt = 0;
	u8 rx[2]; 
	#endif
	int i=0, j=0;
	int resample=gResample;
	
	if(!dw791x->i2c_status){
		DbgOut((DBL_ERROR, "%s i2c fail \n", __func__));
		return VIBE_E_FAIL;
	}
	if(gVibDebugLog&0x0008)//enable nForceData
	{
		DbgOut((DBL_WARNING, "%s:nBufferSizeInBytes=%d +++\n", __func__, nBufferSizeInBytes));
	}
    if (!dw791x->i2c)
        return VIBE_E_FAIL;

    if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE)
        return VIBE_E_FAIL;

	if (!pktsize) {
		/* First byte is i2c destination register address */
		dma_buf_resampling[0] = 0x0D;	// rtp input
		
		/* Copy remaining data */ // nBufferSizeInBytes ==  40byte?
		//memcpy(dma_buf_ivt + 0, pForceOutputBuffer + nDataIndex, nBufferSizeInBytes - nDataIndex);
		memcpy(dma_buf_ivt , pForceOutputBuffer , nBufferSizeInBytes);
		
		if(resample>1){
			for(i=0, j=0; i<nBufferSizeInBytes*resample; i++)
			{
				dma_buf_resampling[i+1] = dma_buf_ivt[j];
				if(i % resample == 3) {
					if(gVibDebugLog&0x0008)//enable nForceData
					{
						DbgOut((DBL_WARNING, "ivt[%04d]=0x%02x resample[%04d-%04d]=0x%02x 0x%02x 0x%02x 0x%02x\n", \
						j, dma_buf_ivt[j], j*4, i, \
						dma_buf_resampling[j*4+1],dma_buf_resampling[j*4+2],dma_buf_resampling[j*4+3],dma_buf_resampling[j*4+4]));
					}
					++j;
				}
			}
			/* Send remaining bytes */
			/* The return value of this function is unreliable so ignoring it */
            if(gVibDebugLog&0x0004)//enable immersion message
            {
                DbgOut((DBL_ERROR, "%s: read vd_clamp: %d\n", __func__, dw791x_byte_read(dw791x->i2c, DW7914_VD_CLAMP)*40));
            }
			i2c_send_buf(dw791x->i2c, dma_buf_resampling, nBufferSizeInBytes * resample - nDataIndex + 1);
		}
		else{
			if(gVibDebugLog&0x0008)//enable nForceData
			{
				for(i=0; i<nBufferSizeInBytes; i++)
						DbgOut((DBL_WARNING, "hex: ivt[%04d]=0x%02x\n", i, dma_buf_ivt[i]));
						DbgOut((DBL_WARNING, "dec: ivt[%04d]=0x%03d\n", i, dma_buf_ivt[i]));
			}
			/* Send remaining bytes */
			/* The return value of this function is unreliable so ignoring it */
			i2c_send_buf(dw791x->i2c, dma_buf_ivt, nBufferSizeInBytes - nDataIndex + 1);
		}
	}
	#if 0
	/* Check how many bytes actually received by dw7914, read FIFO level */
	datacnt = 0;
	rx[0] = dw791x_byte_read(dw791x->i2c, DW7914_FIFO_LEVEL);
	rx[1] = dw791x_byte_read(dw791x->i2c, DW7914_FIFO_LEVEL+1);
	datacnt=((u32)rx[0] << 8) | rx[1];
	if(gVibDebugLog&0x0008)//enable nForceData
	{
		DbgOut((DBL_WARNING, "fifo level=%d, original data size=%d\n", datacnt, nBufferSizeInBytes));
	}
	/* Advance data pointer */
	nDataIndex += datacnt;
	if (nDataIndex < nBufferSizeInBytes)
	{
		DbgOut((DBL_WARNING, "fifo level=%d, original data size=%d\n", datacnt, nBufferSizeInBytes));
		/* wait small amount to avoid underrun */
		waitDuration = nDataIndex > 0 ? nDataIndex * 50 : 50;
		DbgOut((DBL_WARNING, "Delay:waitDuration=%d\n", waitDuration));
		udelay(waitDuration);
	}
	else
	{
		if(gVibDebugLog&0x0008)//enable nForceData
		{
			DbgOut((DBL_WARNING, "%s : VIBE_S_SUCCESS---\n", __func__));
		}
		return VIBE_S_SUCCESS;
	}
	//DbgOut((DBL_WARNING, "%s failed ---\n", __func__));
    return VIBE_E_FAIL;
    #else
	return VIBE_S_SUCCESS;
    #endif
}

/*
** Called to set force output frequency parameters
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetFrequency(VibeUInt8 nActuatorIndex, VibeUInt16 nFrequencyParameterID, VibeUInt32 nFrequencyParameterValue)
{
//#error Please review the code between the #if and #endif

#if 0 
    /* 
    ** The following code is provided as sample. If enabled, it will allow device 
    ** frequency parameters tuning via the ImmVibeSetDeviceKernelParameter API.
    ** Please modify as required. 
    */
    switch (nActuatorIndex)
    {
        case 0:
            switch (nFrequencyParameterID)
            {
                case VIBE_KP_CFG_FREQUENCY_PARAM1:
                    /* Update frequency parameter 1 for actuator 0 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM2:
                    /* Update frequency parameter 2 for actuator 0 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM3:
                    /* Update frequency parameter 3 for actuator 0 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM4:
                    /* Update frequency parameter 4 for actuator 0 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM5:
                    /* Update frequency parameter 5 for actuator 0 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM6:
                    /* Update frequency parameter 6 for actuator 0 */
                    break;
            }
            break;

        case 1:
            switch (nFrequencyParameterID)
            {
                case VIBE_KP_CFG_FREQUENCY_PARAM1:
                    /* Update frequency parameter 1 for actuator 1 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM2:
                    /* Update frequency parameter 2 for actuator 1 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM3:
                    /* Update frequency parameter 3 for actuator 1 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM4:
                    /* Update frequency parameter 4 for actuator 1 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM5:
                    /* Update frequency parameter 5 for actuator 1 */
                    break;

                case VIBE_KP_CFG_FREQUENCY_PARAM6:
                    /* Update frequency parameter 6 for actuator 1 */
                    break;
            }
            break;

        /* Etc. if more than two actuators. */
    }
#endif

    return VIBE_S_SUCCESS;
}
#if 0
/*
** Called to save an IVT data file (pIVT) to a file (szPathName)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_IVTFile_Save(const VibeUInt8 *pIVT, VibeUInt32 nIVTSize, const char *szPathname)
{
//#error Please review the code between the #if and #endif

#if 0   /* The following code is provided as sample. Please modify as required. */
    /* To be implemented */
#endif

    return VIBE_S_SUCCESS;
}

/*
** Called to delete an IVT file
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_IVTFile_Delete(const char *szPathname)
{
//#error Please review the code between the #if and #endif

#if 0   /* The following code is provided as sample. Please modify as required. */
    /* To be implemented */
#endif

    return VIBE_S_SUCCESS;
}
#endif
/*
** Called to get the device name (device name must be returned as ANSI string)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    struct dw791x_priv *dw791x=dw791x_devs[nActuatorIndex];
    char szRevision[24];

    if ((!szDevName) || (nSize < 1)) {
        return VIBE_E_FAIL;
    }
	if(dw791x==NULL){
		sprintf(szRevision, DEVICE_NAME "-DW791x-%02x", 0x7914F002);
	}
	else{
		/* Append revision number to the device name */
		if(!dw791x->i2c_status){
			DbgOut((DBL_INFO, "%s: i2c fail, assign default dev name \n", __func__));
			sprintf(szRevision, DEVICE_NAME "-DW791x-%02x", 0x7914F002);
		}
		else
			sprintf(szRevision, DEVICE_NAME "-DW791x-%02x", dw791x->chipid);
	}
    if (strlen(szRevision) + strlen(szDevName) < nSize - 1) {
        strcat(szDevName, szRevision);
    }

    /* Make sure the string is NULL terminated */
    szDevName[nSize - 1] = '\0';
    
    return VIBE_S_SUCCESS;
}
/*
** Called by the TouchSense Player Service/Daemon when an application calls
** ImmVibeGetDeviceCapabilityInt32 with VIBE_DEVCAPTYPE_DRIVER_STATUS.
** The TouchSense Player does not interpret the returned status.
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetStatus(VibeUInt8 nActuatorIndex)
{
    return VIBE_S_SUCCESS;
}
#if 0
/*
** Called at initialization time to get the number of actuators
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetNum(void)
{
    return NUM_ACTUATORS;
}
#endif

#ifdef IMMVIBESPI_USE_BUFFERFULL
/*
** Check if the amplifier sample buffer is full (not ready for more data).
*/
IMMVIBESPIAPI int ImmVibeSPI_ForceOut_BufferFull(void)
{
	char buf[2];
	int status1=0;
	int status2=0;
	
	//read amplifier 1 buffer status
	//DbgOut((DBL_INFO, "%s: read bus:0 buffer\n", __func__));
	if(dw791x_devs[0]){
		//DbgOut((DBL_INFO, "%s: check dw791x_devs[0]->i2c \n", __func__));
		if(dw791x_devs[0]->i2c){
			if(dw791x_devs[0]->i2c_status){
				//DbgOut((DBL_INFO, "%s: read bus:0 buffer[0]\n", __func__));
				buf[0] = dw791x_byte_read(dw791x_devs[0]->i2c, 0x4D);
				//DbgOut((DBL_INFO, "%s: read bus:0 buffer[1]\n", __func__));
				buf[1] = dw791x_byte_read(dw791x_devs[0]->i2c, 0x4E);
				status1 = ((u32)buf[0] << 8) | buf[1];			
			}
			else
				DbgOut((DBL_INFO, "%s: read bus:0 buffer failed, status1=0\n", __func__));
		}
	}
	//read amplifier 2 buffer status
	//DbgOut((DBL_INFO, "%s: read bus:1 buffer\n", __func__));
	if(dw791x_devs[1]){
		//DbgOut((DBL_INFO, "%s: check dw791x_devs[1]->i2c \n", __func__));
		if(dw791x_devs[1]->i2c){
			if(dw791x_devs[1]->i2c_status){
				//DbgOut((DBL_INFO, "%s: read bus:1 buffer[0]\n", __func__));
				buf[0] = dw791x_byte_read(dw791x_devs[1]->i2c, 0x4D);
				//DbgOut((DBL_INFO, "%s: read bus:1 buffer[1]\n", __func__));
				buf[1] = dw791x_byte_read(dw791x_devs[1]->i2c, 0x4E);
				status2 = ((u32)buf[0] << 8) | buf[1];			
			}
			else
				DbgOut((DBL_INFO, "%s: read bus:1 buffer failed, status2=0.\n", __func__));
		}
	}
	//DbgOut((DBL_INFO, "%s: check status: status1=%d, status2=%d, FIFO %s\n", __func__, status1, status2, (status1 > status2 ? (status1 >= DW791X_FIFO_LEVEL) : (status2 >= DW791X_FIFO_LEVEL)) ?"Full":"Not Full"));
	if(status1 >= DW791X_FIFO_LEVEL) DbgOut((DBL_INFO, "%s: status1 >= DW791X_FIFO_LEVEL\n", __func__));
	if(status2 >= DW791X_FIFO_LEVEL) DbgOut((DBL_INFO, "%s: status2 >= DW791X_FIFO_LEVEL\n", __func__));
    return status1 > status2 ? (status1 >= DW791X_FIFO_LEVEL) : (status2 >= DW791X_FIFO_LEVEL);
}
#endif

