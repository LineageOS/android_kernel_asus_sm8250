/*
 * RT5683.h  --  RT5683 ALSA SoC component driver
 *
 * Copyright 2019 Realtek Semiconductor Corp.
 * Author: Jack Yu <jack.yu@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _RT5683_H_
#define _RT5683_H_

#define RT5683_RESET				0x0000
#define RT5683_HP_AMP_CTRL1			0x0001
#define RT5683_HP_AMP_CTRL2			0x0002
#define RT5683_HP_AMP_CTRL3			0x0003
#define RT5683_HP_AMP_CTRL4			0x0004
#define RT5683_HP_AMP_L_DRE			0x0005
#define RT5683_HP_AMP_R_DRE			0x0006
#define RT5683_IN1_IN2_CTRL_1			0x000b
#define RT5683_IN1_IN2_CTRL_2			0x000c
#define RT5683_CBJ_GAIN				0x000d
#define RT5683_INL1_VOL				0x000e
#define RT5683_INR1_VOL				0x000f
#define RT5683_I2C_CTRL_IF			0x0010
#define RT5683_JD_TD_CTRL_1			0x0011
#define RT5683_DAC_STO1_MIX_1			0x001a
#define RT5683_DAC_STO1_MIX_2			0x001b
#define RT5683_DSD_ANC_DMIX_1			0x001c
#define RT5683_DSD_ANC_DMIX_2			0x001d
#define RT5683_DSD_ANC_DMIX_3			0x001e
#define RT5683_DSD_ANC_DMIX_4			0x001f
#define RT5683_MONO_ADC_MIX_1			0x0020
#define RT5683_MONO_ADC_MIX_2			0x0021
#define RT5683_ADC_STO1_MIX_1			0x0022
#define RT5683_ADC_STO1_MIX_2			0x0023
#define RT5683_STO_DAC_SRC_SEL			0x0027
#define RT5683_STO_ADC_SRC_SEL			0x0028
#define RT5683_L_CH_VOL_DAC			0x071a
#define RT5683_R_CH_VOL_DAC			0x071b
#define RT5683_L_CH_VOL_ADC			0x0e03
#define RT5683_R_CH_VOL_ADC			0x0e04
#define RT5683_HP_SIG_SRC_CTRL			0x01db
#define RT5683_SIL_DET				0x1b05
#define RT5683_PHY_CTRL_27			0x401a

/* USB Firmware Defination */
#define Sel_hp_sig_sour1              0x03
#define HP_DC_Calibration             0x00
#define HP_Impedance                  0x01
#define ByRegister                    0x02
#define SilenceDetect                 0x03




enum {
	RT5683_AIF1,
	RT5683_AIF2,
	RT5683_AIFS
};

enum {
	RT5683_IRQ_DISABLE,
	RT5683_IRQ_ENABLE,
	RT5683_IRQ_NONO
};

enum {
	RT5683_IRQ_NOT_REQUEST,
	RT5683_IRQ_REQUESTED
};

enum {
	RT5683_48K,
	RT5683_96K,
	RT5683_192K
};

enum {
	RT5683_PROBE_START,
	RT5683_PROBE_FINISHED
};


#define RT5683_JACK_MASK (SND_JACK_HEADSET | SND_JACK_OC_HPHL | \
			   SND_JACK_OC_HPHR | SND_JACK_LINEOUT | \
			   SND_JACK_MECHANICAL | SND_JACK_MICROPHONE2 | \
			   SND_JACK_UNSUPPORTED)

#define RT5683_JACK_BUTTON_MASK (SND_JACK_BTN_0 | SND_JACK_BTN_1 | \
				  SND_JACK_BTN_2 | SND_JACK_BTN_3 | \
				  SND_JACK_BTN_4 | SND_JACK_BTN_5)



#endif		/* end of _RT5683_H_ */
