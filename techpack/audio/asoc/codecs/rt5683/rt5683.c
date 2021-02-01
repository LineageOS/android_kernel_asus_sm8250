/*
 * rt5683.c  --  RT5683 ALSA SoC component driver
 *
 * Copyright 2018 Realtek Semiconductor Corp.
 * Author: Jack Yu <jack.yu@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include "rt5683.h"
#include <linux/input.h>

/* ASUS_BSP +++ For headset hook key  working in system suspend */
#include <linux/ktime.h>
static struct wakeup_source hook_key_wake_lock;
/* ASUS_BSP --- */

#define FixedType

extern int audio_switch; // For get Inbox audio mode, 1:I2S, 0;USB

/**
* Below gpio number is based on realtek's dev platform, and should be
* modified according to dev platform respectivelly or parsing from DTS.
*/
#define JACK_IRQ_GPIO 1167
#define USB_I2S_MODE_SWITCH_GPIO 1182

#define RT_5683_GPIO_LOOKUP_STATE	"asustek_rt5683_usb_gpio_active"
#define RT_5683_GPIO_LOOKDOWN_STATE	"asustek_rt5683_usb_gpio_suspend"

#define MAX_KEYCODE_NUM 8

static int rt5683_devMajor = -1;
static struct class *rt5683_property_class = NULL;
static int rt5683_class_open(struct inode * inode, struct file * file)
{
    return 0;
}

static const struct file_operations rt5683_class_fops = {
    .owner      = THIS_MODULE,
    .open       = rt5683_class_open,
};

static struct device *rt5683_dev=NULL;

struct rt5683_priv {
	struct device *dev;
	struct i2c_client *i2c;
	struct snd_soc_component *component;
	struct regmap *regmap;
	struct snd_soc_jack *hs_jack;
	struct snd_soc_jack *btn_jack;
	struct delayed_work hs_btn_detect_work;
	struct delayed_work clear_4button;
	struct delayed_work read_fw_version;
	struct mutex control_lock;
	int key_code[MAX_KEYCODE_NUM];
	int sysclk;
	int sysclk_src;
	int lrck;
	int bclk;
	int master;
	int control;
	int old_control;
	int integer_control;
	int pll_src;
	int pll_in;
	int pll_out;
	int g_PlabackHPStatus;
	int jack_type;
	int jd_status;
	int is_unplug;
	int is_suspended;

	/* ASUS Port */
	u8 reg;
	u32 usb_i2s_switch_gpio;
	int rt5683_irq_gpio;
	int rt5683_irq_num;
	int rt5683_irq_status;
	int rt5683_irq_request_status;
	int rt5683_clock_state;
	int rt5683_probe_state;
	int rt5683_force_Detect_plug_flag;
	unsigned int rt5683_firmware_version;
	unsigned int rt5683_old_firmware_version;
};

static struct rt5683_priv *rt5683 = NULL;

static const struct reg_default rt5683_reg[] = {
	{ 0x0000, 0x00 },
	{ 0x0001, 0x88 },
	{ 0x0002, 0x00 },
	{ 0x0003, 0x22 },
	{ 0x0004, 0x80 },
	{ 0x0005, 0x00 },
	{ 0x0006, 0x00 },
	{ 0x000b, 0x00 },
	{ 0x000c, 0x00 },
	{ 0x000d, 0x00 },
	{ 0x000e, 0x08 },
	{ 0x000f, 0x08 },
	{ 0x0010, 0x1f },
	{ 0x0011, 0x00 },
	{ 0x0017, 0x10 },
	{ 0x0019, 0x00 },
	{ 0x001a, 0xff },
	{ 0x001b, 0x00 },
	{ 0x001c, 0x55 },
	{ 0x001d, 0x88 },
	{ 0x001e, 0xaf },
	{ 0x001f, 0xaf },
	{ 0x0020, 0xc0 },
	{ 0x0021, 0x00 },
	{ 0x0022, 0xcc },
	{ 0x0023, 0x00 },
	{ 0x0024, 0x00 },
	{ 0x0025, 0x00 },
	{ 0x0026, 0x00 },
	{ 0x0027, 0x00 },
	{ 0x0028, 0x00 },
	{ 0x0029, 0x88 },
	{ 0x002a, 0x00 },
	{ 0x002b, 0xaf },
	{ 0x002d, 0xa0 },
	{ 0x0030, 0x00 },
	{ 0x0031, 0x00 },
	{ 0x0032, 0x00 },
	{ 0x0033, 0x00 },
	{ 0x0034, 0x00 },
	{ 0x0039, 0x00 },
	{ 0x003a, 0x00 },
	{ 0x003b, 0x00 },
	{ 0x003c, 0x00 },
	{ 0x003d, 0x00 },
	{ 0x003e, 0x00 },
	{ 0x003f, 0x00 },
	{ 0x0040, 0x7f },
	{ 0x0041, 0x00 },
	{ 0x0042, 0x00 },
	{ 0x0043, 0x00 },
	{ 0x0044, 0x7f },
	{ 0x0045, 0x00 },
	{ 0x0046, 0x00 },
	{ 0x0047, 0x00 },
	{ 0x0048, 0x7f },
	{ 0x0049, 0x0c },
	{ 0x004a, 0x0c },
	{ 0x0060, 0x02 },
	{ 0x0061, 0x00 },
	{ 0x0062, 0x00 },
	{ 0x0063, 0x02 },
	{ 0x0064, 0xff },
	{ 0x0065, 0x00 },
	{ 0x0066, 0x61 },
	{ 0x0067, 0x05 },
	{ 0x0068, 0x00 },
	{ 0x0069, 0x00 },
	{ 0x006b, 0x00 },
	{ 0x008e, 0xc2 },
	{ 0x008f, 0x04 },
	{ 0x0090, 0x0c },
	{ 0x0091, 0x26 },
	{ 0x0092, 0x30 },
	{ 0x0093, 0x73 },
	{ 0x0094, 0x10 },
	{ 0x0095, 0x04 },
	{ 0x0096, 0x00 },
	{ 0x0097, 0x00 },
	{ 0x0098, 0x00 },
	{ 0x0099, 0x00 },
	{ 0x00b0, 0x00 },
	{ 0x00b1, 0x00 },
	{ 0x00b2, 0x00 },
	{ 0x00b3, 0x00 },
	{ 0x00b4, 0x00 },
	{ 0x00b5, 0x00 },
	{ 0x00b6, 0x00 },
	{ 0x00b7, 0x00 },
	{ 0x00b8, 0x00 },
	{ 0x00b9, 0x00 },
	{ 0x00ba, 0x00 },
	{ 0x00bb, 0x02 },
	{ 0x00bd, 0x00 },
	{ 0x00be, 0x00 },
	{ 0x00bf, 0x00 },
	{ 0x00c0, 0x00 },
	{ 0x00d0, 0x00 },
	{ 0x00d1, 0x22 },
	{ 0x00d2, 0x04 },
	{ 0x00d3, 0x00 },
	{ 0x00d4, 0x00 },
	{ 0x00d5, 0x33 },
	{ 0x00d6, 0x00 },
	{ 0x00d7, 0x22 },
	{ 0x00d8, 0x00 },
	{ 0x00d9, 0x09 },
	{ 0x00da, 0x00 },
	{ 0x00e0, 0x00 },
	{ 0x00f0, 0x00 },
	{ 0x00f1, 0x00 },
	{ 0x00f2, 0x00 },
	{ 0x00f3, 0x00 },
	{ 0x00f6, 0x00 },
	{ 0x00f7, 0x00 },
	{ 0x00f8, 0x00 },
	{ 0x00f9, 0x00 },
	{ 0x00fa, 0x00 },
	{ 0x00fb, 0x10 },
	{ 0x00fc, 0xec },
	{ 0x00fd, 0x01 },
	{ 0x00fe, 0x65 },
	{ 0x00ff, 0x40 },
	{ 0x0109, 0x34 },
	{ 0x013A, 0x20 },
	{ 0x013B, 0x22 },
	{ 0x0194, 0x00 },
	{ 0x01DB, 0x04 },
	{ 0x01DC, 0x04 },
	{ 0x0208, 0x00 },
	{ 0x0210, 0x00 },
	{ 0x0211, 0x00 },
	{ 0x0213, 0x00 },
	{ 0x0214, 0x00 },
	{ 0x0703, 0x00 },
	{ 0x070c, 0x00 },
	{ 0x070d, 0x00 },
	{ 0x070e, 0x40 },
	{ 0x071a, 0xaf },
	{ 0x071b, 0xaf },
	{ 0x0810, 0x00 },
	{ 0x0811, 0x80 },
	{ 0x0e03, 0x2f },
	{ 0x0e04, 0x2f },
	{ 0x1b05, 0x00 },
	{ 0x2201, 0x03 },
	{ 0x2B00, 0x42 },
	{ 0x2B01, 0x40 },
	{ 0x2B02, 0x00 },
	{ 0x2B03, 0x00 },
	{ 0x2B05, 0x04 },
	{ 0x3300, 0x40 },
	{ 0x3303, 0xa0 },
	{ 0x3312, 0x00 },
	{ 0x3316, 0x00 },
	{ 0x3317, 0x00 },
	{ 0x3a00, 0x01 },
};

static bool rt5683_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x0000:
	case 0x0029:
	case 0x00b6:
	case 0x00bd:
	case 0x00be:
	case 0x00f0:
	case 0x00f1:
	case 0x00f2:
	case 0x00f3:
	case 0x00f9:
	case 0x00fa:
	case 0x00fb:
	case 0x00fc:
	case 0x00fd:
	case 0x00fe:
	case 0x00ff:
	case 0x0194:
	case 0x070c:
	case 0x070d:

	case 0x0810:
	case 0x0811:
	case 0x2201:
	case 0x2b01:
	case 0x2b02:
	case 0x2b03:
	case 0x3303:
	case 0x3312:
	case 0x3316:
	case 0x3317:
		return true;

	default:
		return false;
	}
}

static bool rt5683_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x0000 ... 0x0006:
	case 0x000b ... 0x0011:
	case 0x0017:
	case 0x0019 ... 0x002b:
	case 0x002d:
	case 0x0030 ... 0x0034:
	case 0x0039 ... 0x004a:
	case 0x0060 ... 0x0069:
	case 0x006b:
	case 0x008e:
	case 0x008f:
	case 0x0090 ... 0x0099:
	case 0x00b0 ... 0x00bb:
	case 0x00bd ... 0x00c0:
	case 0x00d0 ... 0x00da:
	case 0x00e0:
	case 0x00f0 ... 0x00f3:
	case 0x00f6 ... 0x00ff:
	case 0x0109:
	case 0x013a:
	case 0x013b:
	case 0x0194:
	case 0x01db:
	case 0x01dc:
	case 0x0208:
	case 0x0209:
	case 0x0210:
	case 0x0211:
	case 0x0213:
	case 0x0214:
	case 0x0703:
	case 0x070c:
	case 0x070d:
	case 0x070e:
	case 0x071a:
	case 0x071b:

	case 0x0810:
	case 0x0811:

	case 0x0e03:
	case 0x0e04:
	case 0x1b05:
	case 0x2000:
	case 0x2201:
	case 0x2b00:
	case 0x2b01:
	case 0x2b02:
	case 0x2b03:
	case 0x2b05:
	case 0x3300:
	case 0x3303:
	case 0x3312:
	case 0x3316:
	case 0x3317:
	case 0x3a00:
		return true;
	default:
		return false;
	}
}

static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -65625, 375, 0);
static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -17625, 375, 0);

static void rt5683_CodecPowerBack(struct snd_soc_component *component)
{
	struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component);
	
	regmap_write(rt5683->regmap,0x0208,0x37);		      //PowerON   - sysclk

	regmap_update_bits(rt5683->regmap,0x0063,0xAE,0xAE); //PowerOn   - Fast VREF for performance + Enable MBIAS/Bandgap
	msleep(3);
	regmap_update_bits(rt5683->regmap,0x0063,0xFE,0xFE); //PowerOn   - Slow VREF for performance + Enable MBIAS/Bandgap
	regmap_update_bits(rt5683->regmap,0x0061,0x63,0x63); //PowerOn   - LDO_DACREF/DACL1/DACR1/ADCL1
	#ifdef FixedType
	regmap_update_bits(rt5683->regmap,0x0062,0xCC,0xC0); //PowerOn - BST1 Power & MICBIAS1/MICBIAS2 for CBJ          
	regmap_update_bits(rt5683->regmap,0x0065,0x61,0x61); //Keep    - LDO2/LDO_I2S 
	regmap_update_bits(rt5683->regmap,0x0214,0xFA,0xFA); //PowerOn - HPSequence/SAR_ADC/ (Here is for depop)
	#else
	regmap_update_bits(rt5683->regmap,0x0062,0x0C,0x0C); //PowerOn - MICBIAS1/MICBIAS2 for CBJ
	regmap_update_bits(rt5683->regmap,0x0065,0xE1,0xE1); //Keep    - BJ/LDO2/LDO_I2S    
	regmap_update_bits(rt5683->regmap,0x0214,0xFA,0xFA); //PowerOn - HPSequence/SAR_ADC/ComboJD (Here is for depop)
	#endif            
	regmap_update_bits(rt5683->regmap,0x0068,0x03,0x03); //PowerOn   - 1M/25M OSC 
	regmap_update_bits(rt5683->regmap,0x0069,0x80,0x80); //PowerOn   - RECMIX1L
	regmap_update_bits(rt5683->regmap,0x0210,0xA3,0xA3); //PowerOn   - ADC Filter/DAC Filter/DAC Mixer
	regmap_update_bits(rt5683->regmap,0x0211,0x01,0x01); //PowerOn   - DSP post VOL
	regmap_update_bits(rt5683->regmap,0x0213,0xC0,0xC0); //PowerOn   - Silence Detect on DA Stereo
	regmap_update_bits(rt5683->regmap,0x013A,0x10,0x10); //PowerOn   - Enable DAC Clock
	regmap_update_bits(rt5683->regmap,0x013B,0x11,0x11); //PowerOn   - Enable ADC1/ADC2 Clock       
	msleep(5);
}

static void rt5683_CodecPowerSaving()
{
	regmap_update_bits(rt5683->regmap,RT5683_HP_SIG_SRC_CTRL,Sel_hp_sig_sour1,ByRegister); //Depop
	regmap_update_bits(rt5683->regmap,0x0063,0xFE,0x00); //PowerOFF   - Slow VREF for performance + Enable MBIAS/Bandgap
	regmap_update_bits(rt5683->regmap,0x0061,0x63,0x00); //PowerOFF   - LDO_DACREF/DACL1/DACR1/ADCL1/ADCR1
	#ifdef FixedType
	regmap_update_bits(rt5683->regmap,0x0062,0xCC,0x00); //PowerOFF - BST1 Power & MICBIAS1/MICBIAS2 for CBJ
	regmap_update_bits(rt5683->regmap,0x0065,0x61,0x61); //Keep     - LDO2/LDO_I2S
	regmap_update_bits(rt5683->regmap,0x0214,0xFA,0x9A); //Keep     - InLine Detect Power               
	#else
	regmap_update_bits(rt5683->regmap,0x0062,0x0C,0x00); //PowerOFF - MICBIAS1/MICBIAS2 for CBJ
	regmap_update_bits(rt5683->regmap,0x0065,0xE1,0xE1); //Keep     - BJ/LDO2/LDO_I2S
	regmap_update_bits(rt5683->regmap,0x0214,0xFA,0x9A); //Keep     - InLine Detect Power               
	#endif                    
	regmap_update_bits(rt5683->regmap,0x0068,0x03,0x03); //Keep       - 1M/25M OSC    
	regmap_update_bits(rt5683->regmap,0x0069,0x80,0x00); //PowerOFF   - RECMIX1L
	regmap_update_bits(rt5683->regmap,0x013A,0x10,0x00); //PowerOFF   - Enable DAC Clock
	regmap_update_bits(rt5683->regmap,0x013B,0x11,0x00); //PowerOFF   - Enable ADC1/ADC2 Clock       
	regmap_write(rt5683->regmap,0x0208,0x36);		      //PowerOFF   - sysclk
	regmap_update_bits(rt5683->regmap,0x0210,0xA3,0x00); //PowerOFF   - ADC Filter/DAC Filter/DAC Mixer
	regmap_update_bits(rt5683->regmap,0x0211,0x01,0x00); //PowerOFF   - DSP post VOL
	regmap_update_bits(rt5683->regmap,0x0213,0xC0,0x00); //PowerOFF   - Silence Detect on DA Stereo
}

/*static void rt5683_reset(struct regmap *regmap)
{
	regmap_write(regmap, RT5683_RESET, 0);
}*/

static void rt5683_NoPlayback_NoRecording_Control()
{
	unsigned int silence_det;
	regmap_read(rt5683->regmap, 0x1B05, &silence_det);
	if (silence_det == 0x55){
		regmap_update_bits(rt5683->regmap,0x008E,0xFF,0x00);
		rt5683->g_PlabackHPStatus=0;
	} else {
		regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,ByRegister);
		regmap_update_bits(rt5683->regmap,0x01DC,0x04,0x04);
		regmap_update_bits(rt5683->regmap,0x008E,0xE0,0x00); //Disable EN_OUT_HP
		msleep(5);
		regmap_update_bits(rt5683->regmap,0x0061,0x03,0x00); //Disable POW_DAC
		msleep(5);
		regmap_update_bits(rt5683->regmap,0x008E,0x08,0x00); //Disable POW_CAPLESS
		msleep(5);
		regmap_update_bits(rt5683->regmap,0x008E,0x10,0x00); //Disable POW_PUMP
		msleep(5); 
		rt5683->g_PlabackHPStatus=0;
	}
	rt5683_CodecPowerSaving();
	regmap_update_bits(rt5683->regmap,0xFA34,0x01,0x01);  //Enable reg_en_ep_clkgat for power saving
	regmap_update_bits(rt5683->regmap,0x0109,0x70,0x40);  //BUCK=1.95V
	regmap_update_bits(rt5683->regmap,0x2B05,0x80,0x00);  //Disable [EN_IBUF_CBJ_BST1]  for Power Saving
	regmap_update_bits(rt5683->regmap,0x0194,0x85,0x05);   //Disable - HP Auto Mute/UnMute - On/Off by Silence Detect
	pr_err("%s: RT5683 Control No Playback +No Recording\n", __func__);
}

static void rt5683_Playback_and_Recording_Control()
{
	unsigned int silence_det;
	if(rt5683->g_PlabackHPStatus == 0)
	{
		if(silence_det == 0x55)
		{
			regmap_update_bits(rt5683->regmap,0x008E,0xFF,0x00);
			rt5683->g_PlabackHPStatus=0;
		}
		else
		{
			regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,ByRegister);
			regmap_update_bits(rt5683->regmap,0x01DC,0x04,0x04);
			regmap_update_bits(rt5683->regmap,0x008E,0xE0,0x00); //Disable EN_OUT_HP
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x0061,0x03,0x00); //Disable POW_DAC
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0x08,0x00); //Disable POW_CAPLESS
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0x10,0x00); //Disable POW_PUMP
			msleep(5);
			rt5683->g_PlabackHPStatus=0;
		}
	}
		rt5683_CodecPowerBack(rt5683->component);
		regmap_update_bits(rt5683->regmap,0x0061,0x20,0x20);    //only need ADC1L
		regmap_update_bits(rt5683->regmap,0x0210,0x80,0x80);
		regmap_update_bits(rt5683->regmap,0x0069,0x80,0x80);
		regmap_update_bits(rt5683->regmap,0x3A00,0x80,0x80);
		regmap_update_bits(rt5683->regmap,0x00F9 ,0xFF,0x84); //Toggle Clear SPKVDD Auto Recovery Error Flag during Power Saving
		msleep(1);
		regmap_update_bits(rt5683->regmap,0x00F9 ,0xFF,0x04);
		regmap_update_bits(rt5683->regmap,0x0109 ,0x70,0x40);  //BUCK=1.95V
		regmap_update_bits(rt5683->regmap,0x2B05 ,0x80,0x80);  //Recovery [EN_IBUF_CBJ_BST1]  for Power Saving
	if(rt5683->g_PlabackHPStatus == 0)
	{
		regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,ByRegister);
		regmap_update_bits(rt5683->regmap,0x01DC,0x04,0x04);
		regmap_update_bits(rt5683->regmap,0x008E,0x10,0x10); //Enable POW_PUMP
		msleep(5);
		regmap_update_bits(rt5683->regmap,0x008E,0x08,0x08); //Enable POW_CAPLESS
		msleep(5);
		regmap_update_bits(rt5683->regmap,0x0061,0x03,0x03); //Enable POW_DAC
		msleep(5);
		regmap_update_bits(rt5683->regmap,0x008E,0x20,0x20); //Enable EN_OUT_HP
		msleep(5);
		regmap_update_bits(rt5683->regmap,0x008E,0xE0,0xE0); //Enable EN_OUT_HP
		msleep(5);
		regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,SilenceDetect);
		rt5683->g_PlabackHPStatus=1;
	}
	regmap_update_bits(rt5683->regmap,0x0194,0x85,0x85);     //Enable - HP Auto Mute/UnMute - On/Off by Silence Detect
	regmap_update_bits(rt5683->regmap,0x2201,0x03,0x03);     //For RX noise issue - change 16bit to 32bit
	pr_err("%s: RT5683 Control Playback +Recording\n", __func__);
}

static const char *rt5683_ctrl_mode[] = {
	"None", "No Playback-Record","Playback+Record", "Only Playback", "Only Record",
};

static const SOC_ENUM_SINGLE_DECL(rt5683_dsp_mod_enum, 0, 0,
	rt5683_ctrl_mode);

static int rt5683_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component);
	unsigned int silence_det;

	//rt5683->control = ucontrol->value.integer.value[0];
	rt5683->control = ucontrol->value.enumerated.item[0];
	regmap_read(rt5683->regmap, 0x1B05, &silence_det);

	pr_err("%s: RT5683 Control = %d, Old Control = %d\n", __func__, rt5683->control, rt5683->old_control);

	if (rt5683->control == 3 || rt5683->control == 4) {
		pr_err("%s: RT5683 Control need mix playbak\n", __func__);
		rt5683->control = 2;
		rt5683->old_control = 2;
	} else	{
		rt5683->old_control = rt5683->control;
		pr_err("%s: Set RT5683 Control = %d, Old Control = %d\n", __func__, rt5683->control, rt5683->old_control);
	}

	/**
	* "RT5683 Control" description
	* 0: None
	* 1: No Playback +No Recording
	* 2: Playback +Recording
	* 3: Only Playback
	* 4: Only Recording
	*/
	if (rt5683->control == 0) {
		regmap_update_bits(rt5683->regmap,0x0194,0x85,0x85);     //Enable - HP Auto Mute/UnMute - On/Off by Silence Detect
		pr_err("%s: RT5683 Control None and enable Silence Detect\n", __func__);
	} else if (rt5683->control == 1) {
		if (silence_det == 0x55){
			regmap_update_bits(rt5683->regmap,0x008E,0xFF,0x00);
			rt5683->g_PlabackHPStatus=0;
		} else {
			regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,ByRegister);
			regmap_update_bits(rt5683->regmap,0x01DC,0x04,0x04);
			regmap_update_bits(rt5683->regmap,0x008E,0xE0,0x00); //Disable EN_OUT_HP
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x0061,0x03,0x00); //Disable POW_DAC
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0x08,0x00); //Disable POW_CAPLESS
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0x10,0x00); //Disable POW_PUMP
			msleep(5); 
			rt5683->g_PlabackHPStatus=0;    
		}
		rt5683_CodecPowerSaving();
		regmap_update_bits(rt5683->regmap,0xFA34,0x01,0x01);  //Enable reg_en_ep_clkgat for power saving
		regmap_update_bits(rt5683->regmap,0x0109,0x70,0x40);  //BUCK=1.95V
		regmap_update_bits(rt5683->regmap,0x2B05,0x80,0x00);  //Disable [EN_IBUF_CBJ_BST1]  for Power Saving
		regmap_update_bits(rt5683->regmap,0x0194,0x85,0x05);   //Disable - HP Auto Mute/UnMute - On/Off by Silence Detect
		pr_err("%s: RT5683 Control No Playback +No Recording\n", __func__);
	} else if (rt5683->control == 2) {
		if(rt5683->g_PlabackHPStatus == 0)
		{
			if(silence_det == 0x55)
			{
				regmap_update_bits(rt5683->regmap,0x008E,0xFF,0x00);
				rt5683->g_PlabackHPStatus=0;   
			}
			else
			{
				regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,ByRegister);
				regmap_update_bits(rt5683->regmap,0x01DC,0x04,0x04);
				regmap_update_bits(rt5683->regmap,0x008E,0xE0,0x00); //Disable EN_OUT_HP
				msleep(5);
				regmap_update_bits(rt5683->regmap,0x0061,0x03,0x00); //Disable POW_DAC
				msleep(5);
				regmap_update_bits(rt5683->regmap,0x008E,0x08,0x00); //Disable POW_CAPLESS
				msleep(5);
				regmap_update_bits(rt5683->regmap,0x008E,0x10,0x00); //Disable POW_PUMP
				msleep(5); 
				rt5683->g_PlabackHPStatus=0;
			} 
		}
			rt5683_CodecPowerBack(component);
			regmap_update_bits(rt5683->regmap,0x0061,0x20,0x20);    //only need ADC1L
			regmap_update_bits(rt5683->regmap,0x0210,0x80,0x80);
			regmap_update_bits(rt5683->regmap,0x0069,0x80,0x80); 
			regmap_update_bits(rt5683->regmap,0x3A00,0x80,0x80);
			regmap_update_bits(rt5683->regmap,0x00F9 ,0xFF,0x84); //Toggle Clear SPKVDD Auto Recovery Error Flag during Power Saving
			msleep(1); 
			regmap_update_bits(rt5683->regmap,0x00F9 ,0xFF,0x04);              
			regmap_update_bits(rt5683->regmap,0x0109 ,0x70,0x40);  //BUCK=1.95V
			regmap_update_bits(rt5683->regmap,0x2B05 ,0x80,0x80);  //Recovery [EN_IBUF_CBJ_BST1]  for Power Saving 
		if(rt5683->g_PlabackHPStatus == 0)
		{
			regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,ByRegister);
			regmap_update_bits(rt5683->regmap,0x01DC,0x04,0x04);
			regmap_update_bits(rt5683->regmap,0x008E,0x10,0x10); //Enable POW_PUMP
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0x08,0x08); //Enable POW_CAPLESS
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x0061,0x03,0x03); //Enable POW_DAC
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0x20,0x20); //Enable EN_OUT_HP
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0xE0,0xE0); //Enable EN_OUT_HP
			msleep(5); 
			regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,SilenceDetect);
			rt5683->g_PlabackHPStatus=1;
		}
		regmap_update_bits(rt5683->regmap,0x0194,0x85,0x05);     //Disable - HP Auto Mute/UnMute - On/Off by Silence Detect
		regmap_update_bits(rt5683->regmap,0x2201,0x03,0x03);     //For RX noise issue - change 16bit to 32bit
		pr_err("%s: RT5683 Control Playback +Recording\n", __func__);
	} else if (rt5683->control == 3) {
		if(rt5683->g_PlabackHPStatus == 0)
		{
			if(silence_det == 0x55)
			{
				regmap_update_bits(rt5683->regmap,0x008E,0xFF,0x00);
				rt5683->g_PlabackHPStatus=0;
			}
			else
			{
				regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,ByRegister);
				regmap_update_bits(rt5683->regmap,0x01DC,0x04,0x04);
				regmap_update_bits(rt5683->regmap,0x008E,0xE0,0x00); //Disable EN_OUT_HP
				msleep(5);
				regmap_update_bits(rt5683->regmap,0x0061,0x03,0x00); //Disable POW_DAC
				msleep(5);
				regmap_update_bits(rt5683->regmap,0x008E,0x08,0x00); //Disable POW_CAPLESS
				msleep(5);
				regmap_update_bits(rt5683->regmap,0x008E,0x10,0x00); //Disable POW_PUMP
				msleep(5); 
				rt5683->g_PlabackHPStatus=0;
			}
		}
			rt5683_CodecPowerBack(component);
			regmap_update_bits(rt5683->regmap,0xFA34,0x01,0x00); //Disable reg_en_ep_clkgat to avoid no sound issue           
			regmap_update_bits(rt5683->regmap,0x0061,0x20,0x00);   //Power Down ADC1L
			regmap_update_bits(rt5683->regmap,0x0069,0x80,0x00);   //Power Down RECMIX1_L
			
			regmap_update_bits(rt5683->regmap,0x00F9 ,0xFF,0x84);  //Toggle Clear SPKVDD Auto Recovery Error Flag during Power Saving 
			msleep(1); 
			regmap_update_bits(rt5683->regmap,0x00F9 ,0xFF,0x04);
			regmap_update_bits(rt5683->regmap,0x0109 ,0x70,0x40);  //BUCK=1.95V
			regmap_update_bits(rt5683->regmap,0x2B05 ,0x80,0x00);  //Disable [EN_IBUF_CBJ_BST1]  for Power Saving 
		if(rt5683->g_PlabackHPStatus == 0)
		{
			regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,ByRegister);
			regmap_update_bits(rt5683->regmap,0x01DC,0x04,0x04);
			regmap_update_bits(rt5683->regmap,0x008E,0x10,0x10); //Enable POW_PUMP
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0x08,0x08); //Enable POW_CAPLESS
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x0061,0x03,0x03); //Enable POW_DAC
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0x20,0x20); //Enable EN_OUT_HP
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0xE0,0xE0); //Enable EN_OUT_HP
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,SilenceDetect);
			rt5683->g_PlabackHPStatus=1;
		}
			regmap_update_bits(rt5683->regmap,0x0194,0x85,0x05);   //Disable - HP Auto Mute/UnMute - On/Off by Silence Detect
			regmap_update_bits(rt5683->regmap,0x2201,0x03,0x03);     //For RX noise issue - change 16bit to 32bit

			if (rt5683->rt5683_clock_state == RT5683_48K)	{
				pr_err("%s: RT5683 Control Only Playback and set 48K\n", __func__);
				regmap_write(rt5683->regmap, 0x0209, 0x22);
				regmap_write(rt5683->regmap, 0x2000, 0x42);			// Setting for 48k/44.1k 
			} else if (rt5683->rt5683_clock_state == RT5683_96K)	{
				pr_err("%s: RT5683 Control Only Playback and set 96K\n", __func__);
				regmap_write(rt5683->regmap, 0x0209, 0x22);
				regmap_write(rt5683->regmap, 0x2000, 0x41);
			} else	{
				pr_err("%s: RT5683 Control Only Playback and set 192K\n", __func__);
				regmap_write(rt5683->regmap, 0x0209, 0x12);
				regmap_write(rt5683->regmap, 0x2000, 0x48);
			}

			pr_err("%s: RT5683 Control Only Playback, clock state=%d\n", __func__, rt5683->rt5683_clock_state);
	} else if (rt5683->control == 4) {
		if(silence_det == 0x55)
		{
			regmap_update_bits(rt5683->regmap,0x008E,0xFF,0x00);
			rt5683->g_PlabackHPStatus=0;
		}
		else
		{
			regmap_update_bits(rt5683->regmap,0x01DB,Sel_hp_sig_sour1,ByRegister);
			regmap_update_bits(rt5683->regmap,0x01DC,0x04,0x04);
			regmap_update_bits(rt5683->regmap,0x008E,0xE0,0x00); //Disable EN_OUT_HP
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x0061,0x03,0x00); //Disable POW_DAC
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0x08,0x00); //Disable POW_CAPLESS
			msleep(5);
			regmap_update_bits(rt5683->regmap,0x008E,0x10,0x00); //Disable POW_PUMP
			msleep(5); 
			rt5683->g_PlabackHPStatus=0;
		}
		rt5683_CodecPowerBack(component);
		regmap_update_bits(rt5683->regmap,0x0061,0x20,0x20);//only need ADC1L
		regmap_update_bits(rt5683->regmap,0x0210,0x80,0x80);
		regmap_update_bits(rt5683->regmap,0x0069,0x80,0x80);
		regmap_update_bits(rt5683->regmap,0x3A00,0x80,0x80);
		regmap_update_bits(rt5683->regmap,0x0109, 0x70,0x40);//BUCK=1.95V
		regmap_update_bits(rt5683->regmap,0x2B05 ,0x80,0x80);//Recovery [EN_IBUF_CBJ_BST1]  for Power Saving 
		regmap_update_bits(rt5683->regmap,0x0194,0x85,0x05);//Disable - HP Auto Mute/UnMute - On/Off by Silence Detect
		regmap_update_bits(rt5683->regmap,0x2201,0x03,0x03);//For RX noise issue - change 16bit to 32bit

		pr_err("%s: RT5683 Control Only Record\n", __func__);
	} else if (rt5683->control == 5) {
		regmap_write(rt5683->regmap, 0x0209, 0x12);
		regmap_write(rt5683->regmap, 0x2000, 0x48);
		regmap_write(rt5683->regmap, 0x0E05, 0xC4);
		regmap_write(rt5683->regmap, 0x0E07, 0x24);
		rt5683->rt5683_clock_state = RT5683_192K;
		pr_err("%s: RT5683 Control Set 192k\n", __func__);//
	} else if (rt5683->control == 6) {
		regmap_write(rt5683->regmap, 0x0209, 0x22);
		regmap_write(rt5683->regmap, 0x2000, 0x41);
		rt5683->rt5683_clock_state = RT5683_96K;
		pr_err("%s: RT5683 Control Set 96k\n", __func__);
	} else if (rt5683->control == 7) {
		regmap_write(rt5683->regmap, 0x0209, 0x22);
		regmap_write(rt5683->regmap, 0x2000, 0x42);
		rt5683->rt5683_clock_state = RT5683_48K;
		pr_err("%s: RT5683 Control Set 48k/44.1k \n", __func__);
 	} else
 		pr_err("%s: RT5683 Error Control Put : %d\n", __func__, rt5683->control);

	return 0;
}

static int rt5683_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = rt5683->integer_control;
	ucontrol->value.enumerated.item[0] = rt5683->control;

	pr_err("%s: RT5683 Control integer = %d,  enumerated = %d\n",__func__ ,rt5683->integer_control ,rt5683->control);

	return 0;
}

static const struct snd_kcontrol_new rt5683_snd_controls[] = {
	SOC_SINGLE_TLV("DACL Playback Volume", RT5683_L_CH_VOL_DAC,
		0, 175, 0, dac_vol_tlv),
	SOC_SINGLE_TLV("DACR Playback Volume", RT5683_R_CH_VOL_DAC,
		0, 175, 0, dac_vol_tlv),
	SOC_SINGLE_TLV("ADCL Playback Volume", RT5683_L_CH_VOL_ADC,
		0, 127, 0, adc_vol_tlv),
	SOC_SINGLE_TLV("ADCR Playback Volume", RT5683_R_CH_VOL_ADC,
		0, 127, 0, adc_vol_tlv),
	SOC_ENUM_EXT("RT5683 Control", rt5683_dsp_mod_enum, rt5683_control_get,
		rt5683_control_put),
};

static const struct snd_soc_dapm_widget rt5683_dapm_widgets[] = {
	
	SND_SOC_DAPM_AIF_IN("AIF1RX", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF1TX", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIF2RX", "AIF2 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF2TX", "AIF2 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_INPUT("IN1P"),
	SND_SOC_DAPM_INPUT("IN1N"),
	SND_SOC_DAPM_INPUT("IN2P"),
	SND_SOC_DAPM_INPUT("IN2N"),
	SND_SOC_DAPM_OUTPUT("HPOL"),
	SND_SOC_DAPM_OUTPUT("HPOR"),
};

static const struct snd_soc_dapm_route rt5683_dapm_routes[] = {
	{ "AIF1TX", NULL, "IN1P" },
	{ "AIF1TX", NULL, "IN1N" },
	{ "AIF1TX", NULL, "IN2P" },
	{ "AIF1TX", NULL, "IN2N" },
	{ "AIF2TX", NULL, "IN1P" },
	{ "AIF2TX", NULL, "IN1N" },
	{ "AIF2TX", NULL, "IN2P" },
	{ "AIF2TX", NULL, "IN2N" },
	{ "HPOL", NULL, "AIF1RX" },
	{ "HPOR", NULL, "AIF1RX" },
	{ "HPOL", NULL, "AIF2RX" },
	{ "HPOR", NULL, "AIF2RX" },
};
/*
static int rt5683_set_keycode(struct rt5683_priv *rt5683)
{
	enum snd_jack_types type;
	int i, ret, result = 0;
	int *btn_key_code;

	btn_key_code = rt5683->key_code;

	for (i = 0 ; i < MAX_KEYCODE_NUM ; i++) {
		if (btn_key_code[i] != 0) {
			switch (i) {
			case 0:
				type = SND_JACK_BTN_0;
				break;
			case 1:
				type = SND_JACK_BTN_1;
				break;
			case 2:
				type = SND_JACK_BTN_2;
				break;
			case 3:
				type = SND_JACK_BTN_3;
				break;
			case 4:
				type = SND_JACK_BTN_4;
				break;
			case 5:
				type = SND_JACK_BTN_5;
				break;
			default:
				WARN_ONCE(1, "Wrong button number:%d\n", i);
				result = -1;
				return result;
			}
			ret = snd_jack_set_key(rt5683->btn_jack.jack,
							type,
							btn_key_code[i]);
			if (ret) {
				pr_err("%s: Failed to set code for %d\n",
					__func__, btn_key_code[i]);
				result = -1;
				return result;
			}
			input_set_capability(
				rt5683->btn_jack.jack->input_dev,
				EV_KEY, btn_key_code[i]);
			pr_err("%s: set btn%d key code:%d\n", __func__,
				i, btn_key_code[i]);
		}
	}
	return result;
}
*/

int rt5683_set_jack_detect(struct snd_soc_component *component,struct snd_soc_jack *hs_jack);

static struct snd_soc_jack rt5683_jack;

static int rt5683_probe(struct snd_soc_component *component)
{
	struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component);
	int ret = 0;

	dev_err(component->dev, "Begin rt5683_probe.\n");
	rt5683->component = component;
	rt5683->jack_type = 0;
	rt5683->jd_status = 0x30;

	ret = snd_soc_card_jack_new(component->card, "Headset Jack",
					SND_JACK_HEADPHONE | SND_JACK_MICROPHONE |
					SND_JACK_BTN_0 | SND_JACK_BTN_1 |
					SND_JACK_BTN_2 | SND_JACK_BTN_3,
					&rt5683_jack, NULL, 0);
	if (ret) {
		pr_err("%s: Failed to create new jack\n", __func__);
	}

	/* Set RT5683 audio jack mapping */
	ret = snd_jack_set_key(rt5683_jack.jack, SND_JACK_BTN_0, KEY_MEDIA);
	if (ret) {
		pr_err("%s: Failed to set snd_jack_set_key BTN_0 to KEY_MEDIA\n", __func__);
	}	
	ret = snd_jack_set_key(rt5683_jack.jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
	if (ret) {
		pr_err("%s: Failed to set snd_jack_set_key BTN_1 to KEY_VOICECOMMAND\n", __func__);
	}
	ret = snd_jack_set_key(rt5683_jack.jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	if (ret) {
		pr_err("%s: Failed to set snd_jack_set_key BTN_2 to KEY_VOLUMEUP\n", __func__);
	}
	ret = snd_jack_set_key(rt5683_jack.jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);
	if (ret) {
		pr_err("%s: Failed to set snd_jack_set_key BTN_3 to KEY_VOLUMEDOWN\n", __func__);
	}

	ret = rt5683_set_jack_detect(component, &rt5683_jack);
	if (ret) {
		pr_err("%s: Failed to set rt5683_set_jack_detect\n", __func__);
	}

	rt5683->rt5683_probe_state = RT5683_PROBE_FINISHED;
	dev_err(component->dev, "End rt5683_probe.\n");

	return 0;
}

#ifdef CONFIG_PM
static int rt5683_suspend(struct snd_soc_component *component)
{
	//struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component);
	pr_err("%s: Going to suspend\n", __func__);
	rt5683->is_suspended = 1;

	regcache_cache_only(rt5683->regmap, true);
	regcache_mark_dirty(rt5683->regmap);

	return 0;
}

static int rt5683_resume(struct snd_soc_component *component)
{
	//struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component)
	regcache_cache_only(rt5683->regmap, false);
	regcache_sync(rt5683->regmap);

	pr_err("%s: resume!!!\n", __func__);
	rt5683->is_suspended = 0;

	return 0;
}
#else
#define rt5683_suspend NULL
#define rt5683_resume NULL
#endif

#define RT5683_STEREO_RATES SNDRV_PCM_RATE_8000_192000
#define RT5683_FORMATS (SNDRV_PCM_FMTBIT_S8 | \
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_S24_LE)


static int rt5683_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component);

	switch (params_rate(params)) {
	case 192000:
		regmap_write(rt5683->regmap, 0x0209, 0x12);
		regmap_write(rt5683->regmap, 0x2000, 0x48);
		regmap_write(rt5683->regmap, 0x0E05, 0xC4);
		regmap_write(rt5683->regmap, 0x0E07, 0x24);
		break;

	case 96000:
		regmap_write(rt5683->regmap, 0x0209, 0x22);
		regmap_write(rt5683->regmap, 0x2000, 0x41);
		break;

	case 48000:
	case 44100:
		regmap_write(rt5683->regmap, 0x0209, 0x22);
		regmap_write(rt5683->regmap, 0x2000, 0x42);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops rt5683_aif_dai_ops = {
	.hw_params = rt5683_hw_params,
};


static struct snd_soc_dai_driver rt5683_dai[] = {
	{
		.name = "rt5683-aif1-2-1a",
		.id = RT5683_AIF1,
		.playback = {
			.stream_name = "AIF1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5683_STEREO_RATES,
			.formats = RT5683_FORMATS,
		},
		.capture = {
			.stream_name = "AIF1 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5683_STEREO_RATES,
			.formats = RT5683_FORMATS,
		},
		.ops = &rt5683_aif_dai_ops,
	},
	{
		.name = "rt5683-aif2-2-1a",
		.id = RT5683_AIF2,
		.playback = {
			.stream_name = "AIF2 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5683_STEREO_RATES,
			.formats = RT5683_FORMATS,
		},
		.capture = {
			.stream_name = "AIF2 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5683_STEREO_RATES,
			.formats = RT5683_FORMATS,
		},
		.ops = &rt5683_aif_dai_ops,
	},
};

static const struct snd_soc_component_driver soc_component_dev_rt5683 = {
	.probe = rt5683_probe,
	.suspend = rt5683_suspend,
	.resume = rt5683_resume,
	.controls = rt5683_snd_controls,
	.num_controls = ARRAY_SIZE(rt5683_snd_controls),
	.dapm_widgets = rt5683_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rt5683_dapm_widgets),
	.dapm_routes = rt5683_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(rt5683_dapm_routes),
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};

static const struct regmap_config rt5683_regmap = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = RT5683_PHY_CTRL_27,
	.volatile_reg = rt5683_volatile_register,
	.readable_reg = rt5683_readable_register,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = rt5683_reg,
	.num_reg_defaults = ARRAY_SIZE(rt5683_reg),
	.use_single_rw = true,
};

#if defined(CONFIG_OF)
static const struct of_device_id rt5683_of_match[] = {
	{ .compatible = "realtek,rt5683", },
	{},
};
MODULE_DEVICE_TABLE(of, rt5683_of_match);
#endif

#ifdef CONFIG_ACPI
static struct acpi_device_id rt5683_acpi_match[] = {
	{"10EC5683", 0,},
	{},
};
MODULE_DEVICE_TABLE(acpi, rt5683_acpi_match);
#endif

static const struct i2c_device_id rt5683_i2c_id[] = {
	{ "rt5683", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt5683_i2c_id);

static irqreturn_t rt5683_hs_btn_irq_handler(int irq, void *dev_id)
{
	struct rt5683_priv *rt5683  = dev_id;

	pr_info("%s:HS BTN IRQ detected!",__func__);

	if (rt5683->rt5683_probe_state != RT5683_PROBE_FINISHED)	{
		pr_err("%s:Inbox still not inited rt5683_probe_state = %d, ignore this irq!!\n", __func__, rt5683->rt5683_probe_state);
		return IRQ_HANDLED;
	}

	/* ASUS_BSP +++ For headset hook key  working in system suspend */
	__pm_wakeup_event(&hook_key_wake_lock, 3500);
	if (rt5683->is_suspended == 1)	{
		pr_err("%s:after Wakelock 3 sec for hook_key and delay headset delay work(0.5s)\n",__func__);
		mod_delayed_work(system_power_efficient_wq,
				&rt5683->hs_btn_detect_work, msecs_to_jiffies(1000));
	} else
	/* ASUS_BSP --- */
	mod_delayed_work(system_power_efficient_wq,
			&rt5683->hs_btn_detect_work, 0);

	return IRQ_HANDLED;
}

int rt5683_set_jack_detect(struct snd_soc_component *component,
	struct snd_soc_jack *hs_jack)
{
	struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component);

	rt5683->hs_jack = hs_jack;
	//rt5683_hs_btn_irq_handler(0, rt5683);

	return 0;
}
EXPORT_SYMBOL_GPL(rt5683_set_jack_detect);

int rt5683_button_detect(struct snd_soc_component *component)
{
	struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component);
	int btn_type = 0, val_00B6, val_070C, val3_070D;

	regmap_read(rt5683->regmap, 0x00B6, &val_00B6);
	regmap_read(rt5683->regmap, 0x070C, &val_070C);
	regmap_read(rt5683->regmap, 0x070D, &val3_070D);

	if ((val_00B6 & 0x10) == 0x00){
		if ((val_070C == 0x10) && (val3_070D == 0x00)  )       //4 Buttoms-1 (A-KEY_MEDIA Click)                   
		{
			regmap_write(rt5683->regmap, 0x070C, 0x11);           
			regmap_write(rt5683->regmap, 0x070D, 0x11);
			btn_type |= SND_JACK_BTN_0;
			pr_err("[DBG] Buttom1 media Click, 0x00b6:0x%x\n",val_00B6);
		}
		else if ((val_070C == 0x00) && (val3_070D == 0x10) )   //4 Buttoms-3 (B-KEY_VOLUMEUP Click) 
		{
			regmap_write(rt5683->regmap, 0x070C, 0x11);           
			regmap_write(rt5683->regmap, 0x070D, 0x11);
			btn_type |= SND_JACK_BTN_2;
			pr_err("[DBG] Buttom2 volume up Click, 0x00b6:0x%x\n",val_00B6);
		}              
		else if ((val_070C == 0x00) && (val3_070D == 0x01) )   //4 Buttoms-4 (C-KEY_VOLUMEDOWN Click)
		{
			regmap_write(rt5683->regmap, 0x070C, 0x11);           
			regmap_write(rt5683->regmap, 0x070D, 0x11);
			btn_type |= SND_JACK_BTN_3;
			pr_err("[DBG] Buttom3 volume down Click, 0x00b6:0x%x\n",val_00B6);
		}               
		else if ((val_070C == 0x01) && (val3_070D == 0x00) )   //4 Buttoms-2 (D-KEY_VOICECOMMAND Click)
		{
			regmap_write(rt5683->regmap, 0x070C, 0x11);
			regmap_write(rt5683->regmap, 0x070D, 0x11);
			btn_type |= SND_JACK_BTN_1;
			pr_err("[DBG] Buttom4 voice command Click, 0x00b6:0x%x\n",val_00B6);
		}
		else //Abnormal Bottom push
		{
			val_00B6 |= 0x80;
			regmap_write(rt5683->regmap, 0x00B6, val_00B6);
			regmap_write(rt5683->regmap, 0x070C, 0xFF); //Clear Flag (!!!!!! Need to clear all flag)  , Clear will become 0'b when user release press behavior           
			regmap_write(rt5683->regmap, 0x070D, 0xFF); //Clear Flag (!!!!!! Need to clear all flag)  , Clear will become 0'b when user release press behavior                                                            
			pr_err("[DBG] Abnormal Button push, val_07:0x%02X,0x%02X ,  0x00b6:0x%x\n", val_070C, val3_070D, val_00B6);
		}
	} else {
		pr_err("Unknown value,val_00B6=0x%x\n",val_00B6);
		return -EINVAL;
	}

	return btn_type;
}
EXPORT_SYMBOL(rt5683_button_detect);

void rt5683_sar_adc_button_det(struct snd_soc_component *component)
{
	//struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component);
	
	regmap_write(rt5683->regmap, 0x0011, 0xa7);
	regmap_write(rt5683->regmap, 0x3300, 0x85);
	msleep(50);
	regmap_update_bits(rt5683->regmap, 0x3300, 0x80, 0x0);
	regmap_update_bits(rt5683->regmap, 0x3300, 0x80, 0x80);
	msleep(50);
}

int rt5683_headset_detect(struct snd_soc_component *component)
{
	//struct rt5683_priv *rt5683 = snd_soc_component_get_drvdata(component);
	int jack_type, val_2b03, sleep_loop=4;
	int i = 0, sleep_time[4] = {500, 400, 300, 200};

	regmap_update_bits(rt5683->regmap, 0x0068, 0x3, 0x03);
	regmap_write(rt5683->regmap, 0x0063, 0xFE);
	regmap_update_bits(rt5683->regmap, 0x0065, 0xE0, 0xE0);
	regmap_update_bits(rt5683->regmap, 0x0090, 0xC, 0x0);
	regmap_update_bits(rt5683->regmap, 0x0214, 0x18, 0x18);
	regmap_write(rt5683->regmap, 0x2B05, 0x80);
	regmap_write(rt5683->regmap, 0x2B02, 0x0C);
	regmap_write(rt5683->regmap, 0x2B03, 0x44);
	regmap_write(rt5683->regmap, 0x2B01, 0x00);
	regmap_write(rt5683->regmap, 0x0062, 0x0C);
	regmap_write(rt5683->regmap, 0x0063, 0xAE);
	regmap_write(rt5683->regmap, 0x2B00, 0xD0);
	regmap_write(rt5683->regmap, 0x2B03, 0x44);
	regmap_write(rt5683->regmap, 0x0011, 0x80);
	regmap_write(rt5683->regmap, 0x2b01, 0x0);
	msleep(10);
	regmap_write(rt5683->regmap, 0x2b01, 0x8);

	while(i < sleep_loop){
		msleep(sleep_time[i]);
		regmap_read(rt5683->regmap, 0x2B03, &val_2b03);
		val_2b03 &= 0x3;
		pr_info("%s:val_2b03=%d sleep %d\n",
			__func__, val_2b03, sleep_time[i]);
		i++;
		if (val_2b03 == 0x1 || val_2b03 == 0x2 || val_2b03 == 0x3)
			break;
		if (val_2b03 == 0x0){
			regmap_write(rt5683->regmap, 0x2b01, 0x0);
			msleep(10);
			regmap_write(rt5683->regmap, 0x2b01, 0x8);
		}
	}

	/* Register Setting for headset insert Pop sound */
	regmap_update_bits(rt5683->regmap, 0x0065 ,0x02,0x02);          //Enable JDH2
	regmap_update_bits(rt5683->regmap, 0x2B01 ,0xC0,0xC0);          //Enable Fast Turn Off MICBIAS by JD & Set Vref/MBIAS controlled by register
	regmap_update_bits(rt5683->regmap, 0x2B08 ,0x07,0x07);          //Set Fast Turn Off MICBIAS JD = JDH2
	regmap_update_bits(rt5683->regmap, 0x2B00 ,0x01,0x01);          //Configure fast charge setting to avoid disable MICBIAS 
	regmap_update_bits(rt5683->regmap, 0x2B00 ,0x06,0x00);          //Decrease Debounce time
	regmap_update_bits(rt5683->regmap, 0x0214 ,0x04,0x04);          //Enable pow_mic_det_discharge

	if (val_2b03 == 0x0)	{
		jack_type = 0;
		pr_err("rt5683_headset_detect: detect none val_2b03:0x%x, jack_type=0\n", val_2b03);
	} else	{
	if (val_2b03 == 0x1 || val_2b03 == 0x2)
		jack_type = SND_JACK_HEADSET;
	else
		jack_type = SND_JACK_HEADPHONE;

	rt5683_sar_adc_button_det(rt5683->component);
		pr_err("rt5683_headset_detect: val_2b03:0x%x, jack_type=%s\n",
			val_2b03,(jack_type == SND_JACK_HEADSET)?"HEADSET":"HEADPHONE");
	}
	return jack_type;
}
EXPORT_SYMBOL_GPL(rt5683_headset_detect);

void rt5683_read_fw_version_event(struct work_struct *work)
{
	unsigned int val_0810 = 0;
	unsigned int val_0811 = 0;
	unsigned int fw_val = 0;

	/* Check Codec firmware version */
	regcache_cache_bypass(rt5683->regmap, true);
	regmap_read(rt5683->regmap, 0x0810, &val_0810);
	regmap_read(rt5683->regmap, 0x0811, &val_0811);
	regcache_cache_bypass(rt5683->regmap, false);
	fw_val = (val_0811*256) + val_0810;

	pr_err("%s:Read Current firmware version : %d, %d (0x%X)\n", __func__, val_0811, val_0810, fw_val);

	rt5683->rt5683_firmware_version = fw_val;
	pr_err("%s:Get Current firmware version : 0x%X, Old firmware version : 0x%X\n", __func__, 
			rt5683->rt5683_firmware_version, rt5683->rt5683_old_firmware_version);
	rt5683->rt5683_old_firmware_version = rt5683->rt5683_firmware_version;
}

void rt5683_clear_button_event(struct work_struct *work)
{
	pr_err("%s: clear 4button status again, is_unplug = %d\n",__func__ ,rt5683->is_unplug);
	if(rt5683->is_unplug){
		regmap_write(rt5683->regmap, 0x070c, 0xff);
		regmap_write(rt5683->regmap, 0x070d, 0xff);
	}
}

void rt5683_irq_interrupt_event(struct work_struct *work)
{
	//struct rt5683_priv *rt5683 = container_of(work, struct rt5683_priv, hs_btn_detect_work.work);
	unsigned int val_00bd;
	unsigned int val_00be,val_070c,val_070d;
	int report=0, i, btn_type=0, jd_is_changed=0, unplug_idx=0;
	int retry_loop = 2;

	mutex_lock(&rt5683->control_lock);

	if (rt5683->rt5683_force_Detect_plug_flag == 1)	{
		pr_err("%s: Force Detect plug status!\n", __func__);
	} else	{		
		if (rt5683->rt5683_irq_status != RT5683_IRQ_ENABLE)	{
			pr_err("%s: Unexpect irq and ignore, rt5683_irq_status = %d\n", __func__, rt5683->rt5683_irq_status);
			if (rt5683->rt5683_probe_state != RT5683_PROBE_FINISHED)
				pr_err("%s: Inbox still not finish init, rt5683_probe_state = %d\n",
					__func__, rt5683->rt5683_probe_state);
			mutex_unlock(&rt5683->control_lock);
			cancel_delayed_work(&rt5683->hs_btn_detect_work);
			return;
		}
	}

	for(i=0;i<3;i++){
		msleep(1);
		report = regmap_read(rt5683->regmap, 0x00BD, &val_00bd);
		if (report != 0)	{
			pr_err("%s: val_00bd:0x%x (ret = %d, rerty time = %d)\n", __func__, val_00bd, report, i);
			msleep(100);
		} else
			dev_dbg(rt5683->component->dev, "val_00bd:0x%x\n",val_00bd);
	}
	if (rt5683->rt5683_probe_state != RT5683_PROBE_FINISHED || report != 0)	{
		pr_err("%s: Inbox could be removed val_00bd:0x%x (ret = %d) and rt5683_irq_status = %d\n",
				__func__, val_00bd, report, rt5683->rt5683_irq_status);
		if (rt5683->rt5683_probe_state != RT5683_PROBE_FINISHED)
			pr_err("%s: Inbox still not finish init, rt5683_probe_state = %d\n",
				__func__, rt5683->rt5683_probe_state);
		mutex_unlock(&rt5683->control_lock);
		cancel_delayed_work(&rt5683->hs_btn_detect_work);
		return;
	}
	if (rt5683->jd_status != val_00bd)
		jd_is_changed = 1;
	else
		jd_is_changed = 0;

	rt5683->jd_status = val_00bd;

	/* JD Status Confirm */
	if (((val_00bd & 0x30)==0x0))	{
		rt5683->is_unplug = 0;

		if (jd_is_changed || rt5683->rt5683_force_Detect_plug_flag == 1)	{
			rt5683->jack_type = rt5683_headset_detect(rt5683->component);
			/* If detected unknow type do retry */
			if (rt5683->jack_type == 0)	{
				pr_err("Jack_type  is unkown type goto retry!\n");
				i = 0;
				while(i < retry_loop){
					i++;
					rt5683->jack_type = rt5683_headset_detect(rt5683->component);
					pr_err("Retry time=%d  Jack_type  is : %d\n", i, rt5683->jack_type);
					if (rt5683->jack_type != 0)
						break;
				}
				if (rt5683->jack_type == 0)	{
					pr_err("Finish retry(%d)  Jack_type still is : %d force report Hesdphone\n", i, rt5683->jack_type);
					rt5683->jack_type = SND_JACK_HEADPHONE;
				} else
					pr_err("Finish retry(%d)  Jack_type  is : %d\n", i, rt5683->jack_type);
			}
		}
		
		report = rt5683->jack_type;
		regmap_update_bits(rt5683->regmap, 0x0214, 0x2, 0x2);
		regmap_read(rt5683->regmap, 0x00BE, &val_00be);
		regmap_read(rt5683->regmap, 0x070C, &val_070c);
		regmap_read(rt5683->regmap, 0x070D, &val_070d);
		val_070c &= 0x77;
		val_070d &= 0x77;

		/* Status of InLine Command Trigger */
		if (((val_00be & 0x80)==0x80) && !jd_is_changed){
			btn_type = rt5683_button_detect(rt5683->component);
			if (btn_type < 0)
				pr_err("Unexpected button code.\n");
			report |= btn_type;
		}
		if (btn_type == 0 || (val_070c == 0 && val_070d == 0)){
			if (rt5683->jack_type == SND_JACK_HEADSET)	{
				pr_err("rt5683_irq_interrupt_event: SND_JACK_HEADSET Button released.\n");
				/* Enable 4button Inline command */
				regmap_update_bits(rt5683->regmap, 0x070e, 0x80, 0x80);
				/* Enable 4button Inline command IRQ */
				regmap_update_bits(rt5683->regmap, 0x00b6, 0x80, 0x80);
			} else
				pr_err("rt5683_irq_interrupt_event: Other Button op.\n");
			msleep(1);
			regmap_write(rt5683->regmap, 0x070c, 0xff);
			regmap_write(rt5683->regmap, 0x070d, 0xff);
			report = rt5683->jack_type;
		} else
			pr_err("%s(%d) Else.\n", __func__, __LINE__);
	} else{
		pr_err("rt5683_irq_interrupt_event: Unplug!\n");
		rt5683->is_unplug = 1;
		//regmap_write(rt5683->regmap, 0x070c, 0xff);
		//regmap_write(rt5683->regmap, 0x070d, 0xff);

		/* Register Setting for headset insert Pop sound */
		regmap_update_bits(rt5683->regmap, 0x0065 ,0x02,0x02);          //Enable JDH2
		regmap_update_bits(rt5683->regmap, 0x2B01 ,0xC0,0xC0);          //Enable Fast Turn Off MICBIAS by JD & Set Vref/MBIAS controlled by register
		regmap_update_bits(rt5683->regmap, 0x2B08 ,0x07,0x07);          //Set Fast Turn Off MICBIAS JD = JDH2
		regmap_update_bits(rt5683->regmap, 0x2B00 ,0x01,0x01);          //Configure fast charge setting to avoid disable MICBIAS 
		regmap_update_bits(rt5683->regmap, 0x2B00 ,0x06,0x00);          //Decrease Debounce time
		regmap_update_bits(rt5683->regmap, 0x0214 ,0x04,0x04);          //Enable pow_mic_det_discharge

		/* Disable 4button Inline command */
		regmap_update_bits(rt5683->regmap, 0x070e, 0x80, 0x0);
		/* Bypass 4button Inline command IRQ */
		regmap_update_bits(rt5683->regmap, 0x00b6, 0x80, 0x0);
		/* Disable micbias */
		regmap_update_bits(rt5683->regmap, 0x0062, 0x0c, 0x0);

		while(unplug_idx < 10)	{
			regmap_write(rt5683->regmap, 0x070c, 0xff);
			regmap_write(rt5683->regmap, 0x070d, 0xff);

			regmap_read(rt5683->regmap, 0x070c, &val_070c);
			regmap_read(rt5683->regmap, 0x070d, &val_070d);
			if ((val_070c != 0x00) || (val_070d != 0x00)){
				pr_err("0x070C,0x070D not clear!\n");
				msleep(1);
				unplug_idx++;
			} else	{
				pr_err("0x070C,0x070D clear!\n");
				break;
			}
		}
		regmap_update_bits(rt5683->regmap, 0x3300, 0x80, 0x0);
		rt5683->jack_type = 0;
		report = 0;

		/* Register Setting for headset insert Pop sound */
		rt5683_Playback_and_Recording_Control();
		msleep(50);
		rt5683_NoPlayback_NoRecording_Control();
		
		schedule_delayed_work(&rt5683->clear_4button,
			msecs_to_jiffies(500));
	}

	snd_soc_jack_report(rt5683->hs_jack, report, SND_JACK_HEADSET |
		SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2 |
		SND_JACK_BTN_3);

	/* Register Setting for headset insert Pop sound */
	if (jd_is_changed && rt5683->jack_type != 0)	{
		pr_err("%s: First time instert(0x%x)  set Playback_and_Recording_Control\n", __func__, rt5683->jack_type);
		rt5683_Playback_and_Recording_Control();
	}
	mutex_unlock(&rt5683->control_lock);
}

/***************************************************************
                  ASUS Port for ATD and Debug
****************************************************************/
static ssize_t  ATT_rt5683_headset_status_read(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int val = 0;

	switch (rt5683->jack_type) {
	case 0:
		val = 0;
		pr_info("%s report virtual Unplug event %d\n", __func__, val);
 		break;
	case SND_JACK_HEADSET:
		val = 1;
		pr_info("%s report virtual SND_JACK_HEADSET event %d\n", __func__, val);
		break;
	case SND_JACK_HEADPHONE:
		val = 2;
		pr_info("%s report virtual SND_JACK_HEADPHONE event %d\n", __func__, val);
		break;
	default:
		val = rt5683->jack_type;
		pr_err("%s other snd jack event %d\n", __func__, val);
	}

	pr_err("%s value = %d\n", __func__, val);

	return sprintf(buf, "%d\n", val);
}

static ssize_t  ATT_rt5683_headset_status_write(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int report=0, val = 0, ret = 0;
	kstrtoul(buf, 10, &data);
	val = data;

	/* Virtual report headset event */
	switch (val) {
	case 0:
		val = 0;
		pr_info("%s report virtual Unplug event %d\n", __func__, val);
		break;
	case 1:
		val = SND_JACK_HEADSET;
		pr_info("%s report virtual SND_JACK_HEADSET event %d\n", __func__, val);
		break;
	case 2:
		val = SND_JACK_HEADPHONE;
		pr_info("%s report virtual SND_JACK_HEADPHONE event %d\n", __func__, val);
		break;
	case 3:
		val = SND_JACK_LINEOUT;
		pr_info("%s report virtual SND_JACK_LINEOUT event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_0:
		val = SND_JACK_BTN_0;
		pr_info("%s report virtual SND_JACK_BTN_0 event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_1:
		val = SND_JACK_BTN_1;
		pr_info("%s report virtual SND_JACK_BTN_1 event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_2:
		val = SND_JACK_BTN_2;
		pr_info("%s report virtual SND_JACK_BTN_2 event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_3:
		val = SND_JACK_BTN_3;
		pr_info("%s report virtual SND_JACK_BTN_3 event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_4:
		val = SND_JACK_BTN_4;
		pr_info("%s report virtual SND_JACK_BTN_4 event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_5:
		val = SND_JACK_BTN_5;
		pr_info("%s report virtual SND_JACK_BTN_5 event %d\n", __func__, val);
		break;
	default:
		pr_err("%s wrong snd jack event %d\n", __func__, val);
		ret = -EINVAL;
		return ret;
	}

	report = val;
	pr_err("%s report virtual SND JACK event %d\n", __func__, report);
	snd_soc_jack_report(rt5683->hs_jack, report, SND_JACK_HEADSET |
		SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2 |
		SND_JACK_BTN_3);

	return count;
}

static struct device_attribute rt5683_property_attrs = 
	__ATTR(InboxHeadset_status, 0664, ATT_rt5683_headset_status_read, ATT_rt5683_headset_status_write);

static ssize_t ATT_rt5683_get_irq_setting_write(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	//int report=0,val = 0,ret = 0,i = 0;
	//unsigned int val_00bd;
	int val = 0,ret = 0;

	if (!audio_switch){
		pr_err("ATT_rt5683_get_irq_setting_write: audio_switch %d is USB mode, skip access irq\n", audio_switch);
		return count;
	}

	kstrtoul(buf, 10, &data);
	val = data;

	/* Get gpio67 irq control */
	if (val == 1)	{
		if (rt5683->rt5683_irq_request_status == RT5683_IRQ_REQUESTED)	{
			pr_err("ATT_rt5683_get_irq_setting_write: device already requested irq  (%d, %d)\n", val, rt5683->rt5683_irq_request_status);
		} else	{
			pr_err("ATT_rt5683_get_irq_setting_write: try to request irq  (new : %d, old : %d)\n", val, rt5683->rt5683_irq_request_status);
			rt5683->rt5683_irq_request_status = RT5683_IRQ_REQUESTED;

			ret = devm_gpio_request_one(dev, rt5683->rt5683_irq_gpio, GPIOF_DIR_IN, "RT5683_INT");
			if ( ret < 0)	{
				pr_err("ATT_rt5683_get_irq_setting_write: devm_gpio_request_one %d request failed!(%d)\n", rt5683->rt5683_irq_gpio, ret);
				rt5683->rt5683_irq_request_status = RT5683_IRQ_NOT_REQUEST;
			} else	{
				ret = request_threaded_irq(gpio_to_irq(rt5683->rt5683_irq_gpio),
					NULL, rt5683_hs_btn_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"5683_JDH", rt5683);

				if (ret < 0)
					pr_err("ATT_rt5683_get_irq_setting_write: JD IRQ %d request failed!(%d)\n", rt5683->rt5683_irq_num , ret);
				else	{
					pr_err("ATT_rt5683_get_irq_setting_write: JD IRQ %d request pass!(%d) and disable irq\n", rt5683->rt5683_irq_num, ret);
					disable_irq(rt5683->rt5683_irq_num);
					rt5683->rt5683_irq_request_status = RT5683_IRQ_REQUESTED;
					rt5683->rt5683_irq_status = RT5683_IRQ_DISABLE;

					/* Force delay 800ms for make sure codec FW init without any I2C interrupt */
					msleep(800);

					mutex_lock(&rt5683->control_lock);
					pr_err("ATT_rt5683_get_irq_setting_write: Force Detect plug status!\n");
					rt5683->rt5683_force_Detect_plug_flag = 1;
					mutex_unlock(&rt5683->control_lock);

#if 0
					/* Force detect plug status */
					mutex_lock(&rt5683->control_lock);
					pr_err("ATT_rt5683_get_irq_setting_write: Force Detect plug status!\n");
					for (i = 0; i < 3; i++)	{
						msleep(1);
						regmap_read(rt5683->regmap, 0x00BD, &val_00bd);
						dev_dbg(rt5683->component->dev, "%s: val_00bd:0x%x\n", __func__, val_00bd);
					}
					/* JD Status Confirm */
					if (((val_00bd & 0x30) == 0x0))	{
						rt5683->jack_type = rt5683_headset_detect(rt5683->component);
						report = rt5683->jack_type;
						snd_soc_jack_report(rt5683->hs_jack, report, SND_JACK_HEADSET |
								SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2 |
								SND_JACK_BTN_3);
						rt5683_Playback_and_Recording_Control();
					} else	{
						pr_err("%s: Force Detect plug status and Detect None!!(val_00bd:0x%x)\n",
								__func__, val_00bd);
						rt5683->jack_type = 0;

						rt5683_Playback_and_Recording_Control();
						msleep(50);
						rt5683_NoPlayback_NoRecording_Control();
					}
					mutex_unlock(&rt5683->control_lock);
#endif
					mod_delayed_work(system_power_efficient_wq, &rt5683->hs_btn_detect_work, 0);
				}
			}
		}
	}
	/* release gpio67 irq control */
	else	{
		rt5683->rt5683_irq_request_status = RT5683_IRQ_NOT_REQUEST;
		pr_err("ATT_rt5683_get_irq_setting_write: try to release irq(%d)\n", val);


		/* Cancel delay work */
		cancel_delayed_work_sync(&rt5683->hs_btn_detect_work);

		/* Disable irq */
		if (rt5683->rt5683_irq_status == RT5683_IRQ_ENABLE)	{
			pr_err("ATT_rt5683_get_irq_setting_write: Set irq function disable setting\n");
			disable_irq(rt5683->rt5683_irq_num);
			rt5683->rt5683_irq_status == RT5683_IRQ_DISABLE;
		}

		/* Free gpio and irq */
		devm_gpio_free(rt5683->dev, rt5683->rt5683_irq_gpio);

		if (free_irq(rt5683->rt5683_irq_num, rt5683) != NULL)
			pr_err("ATT_rt5683_get_irq_setting_write: free irq success\n");

		/* Force report Unplug headset */
		pr_err("ATT_rt5683_get_irq_setting_write: Force report Unplug!\n");
		rt5683->jack_type = 0;

		snd_soc_jack_report(rt5683->hs_jack, 0, SND_JACK_HEADSET |
			SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2 |
			SND_JACK_BTN_3);
	}

	return count;
}

static ssize_t ATT_rt5683_get_irq_setting_read(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int val = 0;

	val = rt5683->rt5683_irq_request_status;
	pr_err("ATT_rt5683_get_irq_setting_read: Get irq request status (%d)\n", val);
	
	return sprintf(buf, "%d\n", val);
}

static struct device_attribute rt5683_irq_setting_attrs = 
	__ATTR(rt5683_get_irq_setting, 0664, ATT_rt5683_get_irq_setting_read, ATT_rt5683_get_irq_setting_write);

static ssize_t ATT_rt5683_enable_irq_setting_write(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int val = 0;

	if (!audio_switch){
		pr_err("ATT_rt5683_enable_irq_setting_write: audio_switch %d is USB mode, skip access irq\n", audio_switch);
		return count;
	}

	kstrtoul(buf, 10, &data);
	val = data;

	/* Make sure device can enable/disable irq only after requested irq */
	if (rt5683->rt5683_irq_request_status == RT5683_IRQ_REQUESTED)	{
		if (val == 1)	{
			if (rt5683->rt5683_irq_status != RT5683_IRQ_ENABLE)	{
				enable_irq(rt5683->rt5683_irq_num);
				enable_irq_wake(rt5683->rt5683_irq_num);
			}
			else
				pr_err("ATT_rt5683_enable_irq_setting_write: Set irq function (%d), but device already enable irq\n", val);
			mutex_lock(&rt5683->control_lock);
			rt5683->rt5683_irq_status = RT5683_IRQ_ENABLE;
			rt5683->rt5683_force_Detect_plug_flag = 0;
			mutex_unlock(&rt5683->control_lock);
			pr_err("ATT_rt5683_enable_irq_setting_write: Set irq function (%d), enable irq\n", val);
			mod_delayed_work(system_power_efficient_wq, &rt5683->hs_btn_detect_work, 0);
		} else	{
			if (rt5683->rt5683_irq_status != RT5683_IRQ_DISABLE)	{
				disable_irq(rt5683->rt5683_irq_num);
				disable_irq_wake(rt5683->rt5683_irq_num);
			}
			else
				pr_err("ATT_rt5683_enable_irq_setting_write: Set irq function (%d), but device already disable irq\n", val);
			rt5683->rt5683_irq_status = RT5683_IRQ_DISABLE;
			if (val == 0)
				pr_err("ATT_rt5683_enable_irq_setting_write: Set irq function disable setting \n", val);
			else
				pr_err("ATT_rt5683_enable_irq_setting_write: Set irq function error(%d), use defualt disable setting \n", val);
		}
	} else	{
		pr_err("ATT_rt5683_enable_irq_setting_write: Device still not request irq(%d) ignore this enable/disable(%d) irq setting\n",
				rt5683->rt5683_irq_request_status, val);
		rt5683->rt5683_irq_status = RT5683_IRQ_DISABLE;
	}

	/* Check Codec firmware version */
	if (val == 1 && rt5683->rt5683_irq_status == RT5683_IRQ_ENABLE)
		schedule_delayed_work(&rt5683->read_fw_version, msecs_to_jiffies(500));

	return count;
}

static ssize_t ATT_rt5683_enable_irq_setting_read(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int val = 0;

	val = rt5683->rt5683_irq_status;
	pr_err("ATT_rt5683_enable_irq_setting_read: Get irq function status (%d)\n", val);

	return sprintf(buf, "%d\n", val);
}

static struct device_attribute rt5683_enable_irq_setting_attrs = 
	__ATTR(rt5683_enable_irq_setting, 0664, ATT_rt5683_enable_irq_setting_read, ATT_rt5683_enable_irq_setting_write);

static ssize_t ATT_rt5683_firmware_write(struct device *dev, 
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int val = 0;

	kstrtoul(buf, 10, &data);
	val = data;

	pr_err("%s: Command input : %d\n", __func__, val);

	return count;
}

static ssize_t ATT_rt5683_firmware_read(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	int val = 0;
	int old_firmware_version = 0;
	unsigned int val_0810 = 0;
	unsigned int val_0811 = 0;

	val = rt5683->rt5683_firmware_version;
	old_firmware_version = rt5683->rt5683_old_firmware_version;
	
	pr_err("%s:Get Current firmware version : %d, Old firmware version : %d\n", __func__, val, old_firmware_version);

	regmap_read(rt5683->regmap, 0x0810, &val_0810);
	regmap_read(rt5683->regmap, 0x0811, &val_0811);

	val = (val_0811*256) + val_0810;

	pr_err("%s:Read Current firmware version : %d, %d (0x%X)\n", __func__, val_0811, val_0810, val);

	rt5683->rt5683_firmware_version = val;
	pr_err("%s:Get Current firmware version : 0x%X, Old firmware version : 0x%X\n", __func__, 
			rt5683->rt5683_firmware_version, rt5683->rt5683_old_firmware_version);
	rt5683->rt5683_old_firmware_version = rt5683->rt5683_firmware_version;

	return sprintf(buf, "%X.%X\n", val_0811, val_0810);
}

static struct device_attribute rt5683_firmware_attrs = 
	__ATTR(rt5683_firmware, 0664, ATT_rt5683_firmware_read, ATT_rt5683_firmware_write);

/*============================= End Line of attr ==================================*/

static ssize_t rt5683_headset_status_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv *rt5683 = dev_get_drvdata(dev);
	ssize_t ret_count = 0;
	int val = 0;

	switch (rt5683->jack_type) {
	case 0:
		val = 0;
		pr_info("%s report virtual Unplug event %d\n", __func__, val);
 		break;
	case SND_JACK_HEADSET:
		val = 1;
		pr_info("%s report virtual SND_JACK_HEADSET event %d\n", __func__, val);
		break;
	case SND_JACK_HEADPHONE:
		val = 2;
		pr_info("%s report virtual SND_JACK_HEADPHONE event %d\n", __func__, val);
		break;
	default:
		val = rt5683->jack_type;
		pr_err("%s other snd jack event %d\n", __func__, val);
	}

	pr_err("%s value = %d\n", __func__, val);

	count = sprintf(buf, "%d\n", val);

	if (count > count - offset)
		count = count - offset;
	ret_count = count;

	return ret_count;
}

static ssize_t rt5683_headset_status_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv *rt5683 = dev_get_drvdata(dev);
	unsigned long data;
	int report=0, val = 0, ret = 0;
	kstrtoul(buf, 10, &data);
	val = data;

	/* Virtual report headset event */
	switch (val) {
	case 0:
		val = 0;
		pr_info("%s report virtual Unplug event %d\n", __func__, val);
		break;
	case 1:
		val = SND_JACK_HEADSET;
		pr_info("%s report virtual SND_JACK_HEADSET event %d\n", __func__, val);
		break;
	case 2:
		val = SND_JACK_HEADPHONE;
		pr_info("%s report virtual SND_JACK_HEADPHONE event %d\n", __func__, val);
		break;
	case 3:
		val = SND_JACK_LINEOUT;
		pr_info("%s report virtual SND_JACK_LINEOUT event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_0:
		val = SND_JACK_BTN_0;
		pr_info("%s report virtual SND_JACK_BTN_0 event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_1:
		val = SND_JACK_BTN_1;
		pr_info("%s report virtual SND_JACK_BTN_1 event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_2:
		val = SND_JACK_BTN_2;
		pr_info("%s report virtual SND_JACK_BTN_2 event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_3:
		val = SND_JACK_BTN_3;
		pr_info("%s report virtual SND_JACK_BTN_3 event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_4:
		val = SND_JACK_BTN_4;
		pr_info("%s report virtual SND_JACK_BTN_4 event %d\n", __func__, val);
		break;
	case SND_JACK_BTN_5:
		val = SND_JACK_BTN_5;
		pr_info("%s report virtual SND_JACK_BTN_5 event %d\n", __func__, val);
		break;
	default:
		pr_err("%s wrong snd jack event %d\n", __func__, val);
		ret = -1;
		return ret;
	}

	report = val;
	pr_err("%s report virtual SND JACK event %d\n", __func__, report);
	snd_soc_jack_report(rt5683->hs_jack, report, SND_JACK_HEADSET |
		SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2 |
		SND_JACK_BTN_3);

	return count;
}

static struct bin_attribute rt5683_dev_attr_headset_status = {
	.attr = {
		.name = "InboxHeadset_status",
		.mode = S_IRUSR | S_IWUSR,
	},
	.size = 0,
	.read = rt5683_headset_status_read,
	.write = rt5683_headset_status_write,
};


static ssize_t rt5683_usb_switch_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv *rt5683 = dev_get_drvdata(dev);
	int val = 0;

	val = gpio_get_value(rt5683->usb_i2s_switch_gpio);
	pr_err("rt5683_usb_switch_read value = %d\n", val);
	if (val < 0)	{
		count = sprintf(buf, "%s", "gpio_get_value fail");

		if (count > count - offset)
			count = count - offset;
		return count;
	}

	count = sprintf(buf, "value = %d\n", val);

	if (count > count - offset)
		count = count - offset;
	return count;
}

static ssize_t rt5683_usb_switch_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv *rt5683 = dev_get_drvdata(dev);
	unsigned long data;
	int val = 0, ret = 0;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;

	kstrtoul(buf, 10, &data);
	
	val = data;
	if ( val <= 0 )	{
		pr_err("rt5683_usb_switch_write: Set value Low (%d)\n", val);
		
		key_pinctrl = devm_pinctrl_get(&rt5683->i2c->dev);
		set_state = pinctrl_lookup_state(key_pinctrl, RT_5683_GPIO_LOOKDOWN_STATE);
		ret = pinctrl_select_state(key_pinctrl, set_state);
		if (ret)
			pr_err("rt5683_usb_switch_write: Set value Low Fail(%d)\n", ret);
	} else	{
		pr_err("rt5683_usb_switch_write: Set value High (%d)\n", val);

		key_pinctrl = devm_pinctrl_get(&rt5683->i2c->dev);
		set_state = pinctrl_lookup_state(key_pinctrl, RT_5683_GPIO_LOOKUP_STATE);
		ret = pinctrl_select_state(key_pinctrl, set_state);
		if (ret)
			pr_err("rt5683_usb_switch_write: Set value High Fail(%d)\n", ret);		
	}

	return count;
}

static struct bin_attribute rt5683_dev_attr_usb_switch = {
	.attr = {
		.name = "set_usb_switch",
		.mode = S_IRUSR | S_IWUSR,
	},
	.size = 0,
	.read = rt5683_usb_switch_read,
	.write = rt5683_usb_switch_write,
};

static ssize_t rt5683_rw_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv *rt5683 = dev_get_drvdata(dev);
	unsigned long data;
	int val = 0;

	kstrtoul(buf, 16, &data);
	val = data;
	if ( val <= 0 )	{
		rt5683->reg = 0;
		pr_err("rt5683_rw_write: Read I2C reg address error(%d), use defualt address : (0x%X)\n", val, rt5683->reg);
	} else	{
		rt5683->reg = val;
		pr_err("rt5683_rw_write: Set Read I2C reg address (0x%X)\n", rt5683->reg);
	}

	return count;
}

static ssize_t rt5683_rw_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv*rt5683 = dev_get_drvdata(dev);
	struct i2c_msg msgs[2];
	int ret, retries = 5;
	uint8_t val[2] = {0, 0};	
	uint8_t reg[2] = {0, 0};
	reg[0] = 0;
	reg[1] = rt5683->reg;

	msgs[0].addr = rt5683->i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = reg;

	msgs[1].addr = rt5683->i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 2;
	msgs[1].buf = val;

retry:
	ret = i2c_transfer(rt5683->i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		pr_err("i2c error, retries left: %d\n", retries);
		if (retries) {
			retries--;
			msleep(10);
			goto retry;
		}
		count = sprintf(buf, "%s\n", "Read I2C Fail");
		if (count > count - offset)
			count = count - offset;
		return count;
	} else
		pr_err("rt5683_rw_read: Read I2C reg 0x%X data = 0x%X, 0x%X\n", rt5683->reg, val[0], val[1]);
	/* ret contains the number of i2c transaction */
	/* return the number of bytes read */
	count = sprintf(buf, "0x%X, %X\n", val[0], val[1]);

	if (count > count - offset)
		count = count - offset;
	return count;
}

static struct bin_attribute rt5683_dev_attr_i2c_rw = {
	.attr = {
		.name = "rt5683_i2c_rw",
		.mode = S_IRUSR | S_IWUSR,
	},
	.size = 0,
	.read = rt5683_rw_read,
	.write = rt5683_rw_write,
};

/***************************************************************
                  ASUS Port for Control
****************************************************************/
static ssize_t rt5683_get_irq_setting_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv *rt5683 = dev_get_drvdata(dev);
	unsigned long data;
	//int report=0,val = 0,ret = 0,i = 0;
	//unsigned int val_00bd;
	int val = 0,ret = 0;

	if (!audio_switch){
		pr_err("rt5683_get_irq_setting_write: audio_switch %d is USB mode, skip access irq\n", audio_switch);
		return count;
	}

	kstrtoul(buf, 10, &data);
	val = data;

	/* Get gpio67 irq control */
	if (val == 1)	{
		if (rt5683->rt5683_irq_request_status == RT5683_IRQ_REQUESTED)	{
			pr_err("rt5683_get_irq_setting_write: device already requested irq  (%d, %d)\n", val, rt5683->rt5683_irq_request_status);
		} else	{
			pr_err("rt5683_get_irq_setting_write: try to request irq  (new : %d, old : %d)\n", val, rt5683->rt5683_irq_request_status);
			rt5683->rt5683_irq_request_status = RT5683_IRQ_REQUESTED;

			ret = devm_gpio_request_one(dev, rt5683->rt5683_irq_gpio, GPIOF_DIR_IN, "RT5683_INT");
			if ( ret < 0)	{
				pr_err("rt5683_get_irq_setting_write: devm_gpio_request_one %d request failed!(%d)\n", rt5683->rt5683_irq_gpio, ret);
				rt5683->rt5683_irq_request_status = RT5683_IRQ_NOT_REQUEST;
			} else	{
				ret = request_threaded_irq(gpio_to_irq(rt5683->rt5683_irq_gpio),
					NULL, rt5683_hs_btn_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						"5683_JDH", rt5683);

				if (ret < 0)
					pr_err("rt5683_get_irq_setting_write: JD IRQ %d request failed!(%d)\n", rt5683->rt5683_irq_num , ret);
				else	{
					pr_err("rt5683_get_irq_setting_write: JD IRQ %d request pass!(%d) and disable irq\n", rt5683->rt5683_irq_num, ret);
					disable_irq(rt5683->rt5683_irq_num);
					rt5683->rt5683_irq_request_status = RT5683_IRQ_REQUESTED;
					rt5683->rt5683_irq_status = RT5683_IRQ_DISABLE;

					/* Force delay 800ms for make sure codec FW init without any I2C interrupt */
					msleep(800);

					mutex_lock(&rt5683->control_lock);
					pr_err("rt5683_get_irq_setting_write: Force Detect plug status!\n");
					rt5683->rt5683_force_Detect_plug_flag = 1;
					mutex_unlock(&rt5683->control_lock);

#if 0
					/* Force detect plug status */
					mutex_lock(&rt5683->control_lock);
					pr_err("rt5683_get_irq_setting_write: Force Detect plug status!\n");
					for (i = 0; i < 3; i++)	{
						msleep(1);
						regmap_read(rt5683->regmap, 0x00BD, &val_00bd);
						dev_dbg(rt5683->component->dev, "%s: val_00bd:0x%x\n", __func__, val_00bd);
					}
					/* JD Status Confirm */
					if (((val_00bd & 0x30) == 0x0))	{
						rt5683->jack_type = rt5683_headset_detect(rt5683->component);
						report = rt5683->jack_type;
						snd_soc_jack_report(rt5683->hs_jack, report, SND_JACK_HEADSET |
								SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2 |
								SND_JACK_BTN_3);
						rt5683_Playback_and_Recording_Control();
					} else	{
						pr_err("%s: Force Detect plug status and Detect None!!(val_00bd:0x%x)\n",
								__func__, val_00bd);
						rt5683->jack_type = 0;

						rt5683_Playback_and_Recording_Control();
						msleep(50);
						rt5683_NoPlayback_NoRecording_Control();
					}
					mutex_unlock(&rt5683->control_lock);
#endif
					mod_delayed_work(system_power_efficient_wq, &rt5683->hs_btn_detect_work, 0);
				}
			}
		}
	}
	/* release gpio67 irq control */
	else	{
		rt5683->rt5683_irq_request_status = RT5683_IRQ_NOT_REQUEST;
		pr_err("rt5683_get_irq_setting_write: try to release irq(%d)\n", val);

		/* Disable irq */
		if (rt5683->rt5683_irq_status == RT5683_IRQ_ENABLE)	{
			pr_err("rt5683_get_irq_setting_write: Set irq function disable setting and cancel_delayed_work\n");
			disable_irq(rt5683->rt5683_irq_num);
			rt5683->rt5683_irq_status == RT5683_IRQ_DISABLE;

			/* Cancel delay work */
			cancel_delayed_work(&rt5683->hs_btn_detect_work);
		}

		/* Free gpio and irq */
		devm_gpio_free(dev, rt5683->rt5683_irq_gpio);

		if (free_irq(rt5683->rt5683_irq_num, rt5683) != NULL)
			pr_err("rt5683_get_irq_setting_write: free irq success\n");

		/* Force report Unplug headset */
		pr_err("rt5683_get_irq_setting_write: Force report Unplug!\n");
		rt5683->jack_type = 0;

		snd_soc_jack_report(rt5683->hs_jack, 0, SND_JACK_HEADSET |
			SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2 |
			SND_JACK_BTN_3);
	}

	return count;
}

static ssize_t rt5683_get_irq_setting_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv*rt5683 = dev_get_drvdata(dev);
	ssize_t ret_count;
	int val = 0;

	val = rt5683->rt5683_irq_request_status;
	pr_err("rt5683_get_irq_setting_read: Get irq request status (%d), count = %d/%d\n", val, (int)count, (int)offset);
	
	count = sprintf(buf, "%d\n", val);

	if (count > count - offset)
		count = count - offset;

	ret_count = count;

	return ret_count;
}

static struct bin_attribute rt5683_dev_attr_get_irq_setting = {
	.attr = {
		.name = "rt5683_get_irq_setting",
		.mode = S_IRUSR | S_IWUSR,
	},
	.size = 0,
	.read = rt5683_get_irq_setting_read,
	.write = rt5683_get_irq_setting_write,
};

static ssize_t rt5683_enable_irq_setting_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv *rt5683 = dev_get_drvdata(dev);
	unsigned long data;
	int val = 0;

	if (!audio_switch){
		pr_err("rt5683_enable_irq_setting_write: audio_switch %d is USB mode, skip access irq\n", audio_switch);
		return count;
	}

	kstrtoul(buf, 10, &data);
	val = data;

	/* Make sure device can enable/disable irq only after requested irq */
	if (rt5683->rt5683_irq_request_status == RT5683_IRQ_REQUESTED)	{
		if (val == 1)	{
			if (rt5683->rt5683_irq_status != RT5683_IRQ_ENABLE)	{
				enable_irq(rt5683->rt5683_irq_num);
				enable_irq_wake(rt5683->rt5683_irq_num);
			}
			else
				pr_err("rt5683_enable_irq_setting_write: Set irq function (%d), but device already enable irq\n", val);
			mutex_lock(&rt5683->control_lock);
			rt5683->rt5683_irq_status = RT5683_IRQ_ENABLE;
			rt5683->rt5683_force_Detect_plug_flag = 0;
			mutex_unlock(&rt5683->control_lock);
			pr_err("rt5683_enable_irq_setting_write: Set irq function (%d), enable irq\n", val);
			mod_delayed_work(system_power_efficient_wq, &rt5683->hs_btn_detect_work, 0);
		} else	{
			if (rt5683->rt5683_irq_status != RT5683_IRQ_DISABLE)	{
				disable_irq(rt5683->rt5683_irq_num);
				disable_irq_wake(rt5683->rt5683_irq_num);
			}
			else
				pr_err("rt5683_enable_irq_setting_write: Set irq function (%d), but device already disable irq\n", val);
			rt5683->rt5683_irq_status = RT5683_IRQ_DISABLE;
			if (val == 0)
				pr_err("rt5683_enable_irq_setting_write: Set irq function disable setting \n", val);
			else
				pr_err("rt5683_enable_irq_setting_write: Set irq function error(%d), use defualt disable setting \n", val);
		}
	} else	{
		pr_err("rt5683_enable_irq_setting_write: Device still not request irq(%d) ignore this enable/disable(%d) irq setting\n",
				rt5683->rt5683_irq_request_status, val);
		rt5683->rt5683_irq_status = RT5683_IRQ_DISABLE;
	}

	/* Check Codec firmware version */
	if (val == 1 && rt5683->rt5683_irq_status == RT5683_IRQ_ENABLE)
		schedule_delayed_work(&rt5683->read_fw_version, msecs_to_jiffies(500));

	return count;
}

static ssize_t rt5683_enable_irq_setting_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv*rt5683 = dev_get_drvdata(dev);
	ssize_t ret_count;
	int val = 0;

	val = rt5683->rt5683_irq_status;
	pr_err("rt5683_enable_irq_setting_read: Get irq function status (%d)\n", val);
	
	count = sprintf(buf, "%d\n", val);

	if (count > count - offset)
		count = count - offset;

	ret_count = count;

	return ret_count;
}

static struct bin_attribute rt5683_dev_attr_irq_setting = {
	.attr = {
		.name = "rt5683_enable_irq_setting",
		.mode = S_IRUSR | S_IWUSR,
	},
	.size = 0,
	.read = rt5683_enable_irq_setting_read,
	.write = rt5683_enable_irq_setting_write,
};

static ssize_t rt5683_reset_write(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv *rt5683 = dev_get_drvdata(dev);
	unsigned long data;
	int val = 0;

	kstrtoul(buf, 10, &data);
	val = data;
	if (val == 1)	{
		regmap_write(rt5683->regmap, RT5683_RESET, 0);
		pr_err("%s Reset RT5683(%d)\n", __func__, val);
	} else
		pr_err("%s Involid setting(%d)\n", __func__, val);

	return count;
}

static ssize_t rt5683_reset_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t offset, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rt5683_priv*rt5683 = dev_get_drvdata(dev);
	ssize_t ret_count;
	int val = 0;

	regmap_read(rt5683->regmap, RT5683_RESET, &val);
	pr_err("%s Get RT5683_RESET value : 0x%X \n", __func__, val);
	
	count = sprintf(buf, "0x%X\n", val);

	if (count > count - offset)
		count = count - offset;

	ret_count = count;

	return ret_count;
}

static struct bin_attribute rt5683_dev_attr_reset_setting = {
	.attr = {
		.name = "rt5683_reset",
		.mode = S_IRUSR | S_IWUSR,
	},
	.size = 0,
	.read = rt5683_reset_read,
	.write = rt5683_reset_write,
};

/***************************************************************
                      Device Probe Port
****************************************************************/
static int rt5683_i2c_probe(struct i2c_client *i2c,
		    const struct i2c_device_id *id)
{
	unsigned int irq_num;
	int ret;
	//int usb_i2s_switch_gpio = USB_I2S_MODE_SWITCH_GPIO;
	//struct pinctrl *key_pinctrl;
	//struct pinctrl_state *set_state;

	dev_t dev;

	dev_err(&i2c->dev, "Begin rt5683_i2c_probe.\n");

	rt5683 = devm_kzalloc(&i2c->dev, sizeof(struct rt5683_priv),
				GFP_KERNEL);
	if (rt5683 == NULL)
		return -ENOMEM;

	rt5683->dev = &i2c->dev;
	rt5683->i2c = i2c;
	rt5683->reg = RT5683_RESET;
	rt5683->rt5683_irq_num = 0;
	rt5683->jack_type = 0;
	rt5683->jd_status = 0x30;
	rt5683->is_unplug = 1;
	rt5683->rt5683_clock_state = RT5683_192K;
	rt5683->rt5683_irq_status = RT5683_IRQ_NONO;
	rt5683->rt5683_irq_request_status = RT5683_IRQ_NOT_REQUEST;
	rt5683->is_suspended = 0;
	rt5683->rt5683_firmware_version = 0;
	rt5683->rt5683_old_firmware_version = 0;
	rt5683->rt5683_probe_state = RT5683_PROBE_START;
	rt5683->rt5683_force_Detect_plug_flag = 0;
	i2c_set_clientdata(i2c, rt5683);

	rt5683->regmap = devm_regmap_init_i2c(i2c, &rt5683_regmap);
	if (IS_ERR(rt5683->regmap)) {
		ret = PTR_ERR(rt5683->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	/* Init Mutexlock */
	mutex_init(&rt5683->control_lock);

	/* ASUS_BSP +++ Init wake_lock for headset hook key  working in system suspend */
	wakeup_source_init(&hook_key_wake_lock, "hook_key_lock");

	/* Setting GPIO Port */
	INIT_DELAYED_WORK(&rt5683->hs_btn_detect_work, rt5683_irq_interrupt_event);
	INIT_DELAYED_WORK(&rt5683->read_fw_version, rt5683_read_fw_version_event);
	INIT_DELAYED_WORK(&rt5683->clear_4button, rt5683_clear_button_event);

	rt5683->rt5683_irq_gpio = of_get_named_gpio(i2c->dev.of_node, "irq-gpio", 0);
	if (rt5683->rt5683_irq_gpio < 0)	{
		dev_err(&i2c->dev, "No IRQ GPIO provided.: %d, use defualt setting %d\n", ret, JACK_IRQ_GPIO);
		rt5683->rt5683_irq_gpio = JACK_IRQ_GPIO;
	} else
		dev_err(&i2c->dev, "IRQ GPIO provided.: %d", rt5683->rt5683_irq_gpio);
	irq_num = devm_gpio_request_one(&rt5683->i2c->dev, rt5683->rt5683_irq_gpio, GPIOF_DIR_IN, "RT5683_INT");

	if ( irq_num < 0)	{
		dev_err(&i2c->dev, "devm_gpio_request_one %d request failed!\n", irq_num);
	} else	{
		dev_err(&i2c->dev, "devm_gpio_request_one %d request pass!\n", irq_num);
		irq_num = gpio_to_irq(rt5683->rt5683_irq_gpio);
		dev_err(&i2c->dev, "gpio_to_irq request resuilt : %d\n", irq_num);
		rt5683->rt5683_irq_num = irq_num;

		/* register irq handler */
/*		ret = devm_request_threaded_irq(&i2c->dev, gpio_to_irq(rt5683->rt5683_irq_gpio),
			NULL, rt5683_jack_type_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"5683_int", rt5683);*/

		ret = request_threaded_irq(gpio_to_irq(rt5683->rt5683_irq_gpio),
			NULL, rt5683_hs_btn_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"5683_JDH", rt5683);

		if (ret < 0)
			dev_err(&i2c->dev, "JD IRQ %d request failed!(%d)\n",irq_num , ret);
		else	{
			dev_err(&i2c->dev, "JD IRQ %d request pass!(%d) then disable and free irq\n",irq_num , ret);
			disable_irq(rt5683->rt5683_irq_num);
			rt5683->rt5683_irq_status = RT5683_IRQ_DISABLE;

			/* Free irq */
			if (free_irq(rt5683->rt5683_irq_num, rt5683) != NULL)
				dev_err(&i2c->dev, "free irq success\n");
			else
				dev_err(&i2c->dev, "free irq could be fail\n");
			rt5683->rt5683_irq_request_status = RT5683_IRQ_NOT_REQUEST;
		}
		/* Free gpio and irq */
		devm_gpio_free(&rt5683->i2c->dev, rt5683->rt5683_irq_gpio);

		/* init wake up source for irq */
		ret = device_init_wakeup(&rt5683->i2c->dev, 1);
		if (ret < 0)
			dev_err(&i2c->dev, "JD IRQ %d device_init_wakeup failed!(%d)\n",irq_num , ret);
	}

	/**************************************************************************
	 *	Setting Switch GPIO
	 *	Remove by ASUS ROG struct ec_hid driver control all of the control GPIO 
	 *	by dongle switch service and irq have to dynamic reload
	 ***************************************************************************
	usb_i2s_switch_gpio = of_get_named_gpio(i2c->dev.of_node, "switch-gpio", 0);

	ret = usb_i2s_switch_gpio;
	if (ret) {
		usb_i2s_switch_gpio = USB_I2S_MODE_SWITCH_GPIO;
		dev_err(&i2c->dev, "No Switch GPIO provided.: %d, use defualt setting %d\n", ret, usb_i2s_switch_gpio);
		rt5683->usb_i2s_switch_gpio = usb_i2s_switch_gpio;
	} else	{
		if (usb_i2s_switch_gpio != USB_I2S_MODE_SWITCH_GPIO)	{
			dev_err(&i2c->dev, "Switch GPIO provided wrong.: %d, Wrong setting %d\n", ret, usb_i2s_switch_gpio);
			usb_i2s_switch_gpio = USB_I2S_MODE_SWITCH_GPIO;
		}
	
		dev_err(&i2c->dev, "Switch GPIO provided.: %d, use setting %d\n", ret, usb_i2s_switch_gpio);
		rt5683->usb_i2s_switch_gpio = usb_i2s_switch_gpio;	
	}
	
	key_pinctrl = devm_pinctrl_get(&i2c->dev);
	set_state = pinctrl_lookup_state(key_pinctrl, RT_5683_GPIO_LOOKDOWN_STATE);
	ret = pinctrl_select_state(key_pinctrl, set_state);
	if(ret < 0)
		dev_err(&i2c->dev, "%s: pinctrl_select_state ERROR(%d).\n", __FUNCTION__, ret);
	*/

	/* Register the sysfs files for climax backdoor access */
	ret = device_create_bin_file(&i2c->dev, &rt5683_dev_attr_headset_status);
	if (ret)
		dev_err(&i2c->dev, "error creating headset_status sysfs files\n");
	
	ret = device_create_bin_file(&i2c->dev, &rt5683_dev_attr_usb_switch);
	if (ret)
		dev_err(&i2c->dev, "error creating usb_switch sysfs files\n");

	ret = device_create_bin_file(&i2c->dev, &rt5683_dev_attr_i2c_rw);
	if (ret)
		dev_err(&i2c->dev, "error creating i2c_rw sysfs files\n");

	ret = device_create_bin_file(&i2c->dev, &rt5683_dev_attr_irq_setting);
	if (ret)
		dev_err(&i2c->dev, "error creating irq_setting sysfs files\n");

	ret = device_create_bin_file(&i2c->dev, &rt5683_dev_attr_get_irq_setting);
	if (ret)
		dev_err(&i2c->dev, "error creating irq_request_setting sysfs files\n");

	ret = device_create_bin_file(&i2c->dev, &rt5683_dev_attr_reset_setting);
	if (ret)
		dev_err(&i2c->dev, "error creating reset_setting sysfs files\n");

	/*create character device*/
	rt5683_devMajor = register_chrdev(0, "rt5683", &rt5683_class_fops);
	if (rt5683_devMajor < 0) {
		dev_err(&i2c->dev, "could not get major number\n");
		return rt5683_devMajor;
	}

	/* create sys/class file node*/
	rt5683_property_class = class_create(THIS_MODULE, "rt5683");
	if (IS_ERR(rt5683_property_class)) {
		dev_err(&i2c->dev, "class_create ERROR.\n");
		return PTR_ERR(rt5683_property_class);
	}

	dev = MKDEV(rt5683_devMajor, 0);
	rt5683_dev = device_create(rt5683_property_class, NULL, dev, NULL, "%s", "rt5683_i2s_inbox");

	ret = device_create_file(rt5683_dev, &rt5683_property_attrs);
	if (ret)
		dev_err(&i2c->dev, "error creating rt5683_property_attrs sys_class files\n");
	ret = device_create_file(rt5683_dev, &rt5683_irq_setting_attrs);
	if (ret)
		dev_err(&i2c->dev, "error creating rt5683_irq_setting_attrs sys_class files\n");
	ret = device_create_file(rt5683_dev, &rt5683_enable_irq_setting_attrs);
	if (ret)
		dev_err(&i2c->dev, "error creating rt5683_enable_irq_setting_attrs sys_class files\n");	

	ret = device_create_file(rt5683_dev, &rt5683_firmware_attrs);
	if (ret)
		dev_err(&i2c->dev, "error creating rt5683_firmware_attrs sys_class files\n");	

	/* Register sound SOC DAI Port */
	ret = devm_snd_soc_register_component(&i2c->dev,
			&soc_component_dev_rt5683,
			rt5683_dai, ARRAY_SIZE(rt5683_dai));
	dev_err(&i2c->dev, "End rt5683_i2c_probe.\n");

	return ret;
}

static struct i2c_driver rt5683_i2c_driver = {
	.driver = {
		.name = "rt5683",
#if defined(CONFIG_OF)
		.of_match_table = rt5683_of_match,
#endif
#if defined(CONFIG_ACPI)
		.acpi_match_table = ACPI_PTR(rt5683_acpi_match)
#endif
	},
	.probe = rt5683_i2c_probe,
	.id_table = rt5683_i2c_id,
};
module_i2c_driver(rt5683_i2c_driver);

MODULE_DESCRIPTION("ASoC RT5683 driver");
MODULE_AUTHOR("Jack Yu <jack.yu@realtek.com>");
MODULE_LICENSE("GPL v2");
