/*
 * linux/sound/soc/codecs/tlv320aic325x.c
 *
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * Based on sound/soc/codecs/wm8753.c by Liam Girdwood
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 * Rev 0.1   ASoC driver support			31-04-2009
 * The AIC32 ASoC driver is ported for the codec AIC325x.
 *
 *
 * Rev 1.0   Mini DSP support				11-05-2009
 * Added mini DSP programming support
 *
 * Rev 1.1   Mixer controls				18-01-2011
 * Added all the possible mixer controls.
 *
 * Rev 1.2   Additional Codec driver support		2-02-2011
 * Support for AIC3253, AIC3206, AIC3256
 *
 * Rev 2.0   Ported the Codec driver to 2.6.39 kernel	30-03-2012
 *
 * Rev 2.1   PLL DAPM support added to the codec driver	03-04-2012
 *
 * Rev 2.2   Added event handlers for DAPM widgets	16-05-2012
 *	     Updated ENUM declerations
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/jack.h>
#include <mach/iomux.h>
#include <linux/mfd/tlv320aic3xxx-registers.h>
#include <linux/mfd/tlv320aic3256-registers.h>
#include <linux/mfd/tlv320aic3xxx-core.h>
#include "tlv320aic325x.h"
#include <linux/regulator/consumer.h>
//#include "tlv320aic3xxx-dsp.h"
//#include "aic3xxx_tiload.h"
//#include "aic3xxx_cfw_ops.h"
//#include "aic3xxx_cfw.h"
#include "tpa6130a2.h"
#include <linux/reboot.h>

#if 0
#define DBG(fmt, args...)  printk(KERN_INFO "[AIC325X] " "%s(%d): " fmt, __FUNCTION__, __LINE__, ##args)
#else
#define DBG(fmt, args...)
#endif

#define BIT_HEADSET_NO             (0 << 0)
#define BIT_HEADSET_MIC      (1 << 1) //add by yu for headset report
static int headset_insert = BIT_HEADSET_NO;
#define MIC_HS_RVAL  0x40
#define MIC_INT_RVAL 0x04
//#define MIC_HS_RVAL  0x44
//#define MIC_INT_RVAL 0x44

#define NO_PD

static bool startup_or_resume = true;
struct snd_soc_codec *aic325x_codec;
static struct snd_soc_codec *codec_inst; 


/* User defined Macros kcontrol builders */
#define SOC_SINGLE_AIC325x(xname)                                       \
	{                                                               \
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,     \
		.info = __new_control_info, .get = __new_control_get, \
		.put = __new_control_put,                       \
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,      \
	}


/*
* Function Prototype
*/
//extern int aic3xxx_driver_init(struct snd_soc_codec *codec);
static int aic325x_initalize(struct snd_soc_codec *codec);
static int aic325x_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *);
static int aic325x_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai);
static void aic325x_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai);
static int aic325x_mute(struct snd_soc_dai *dai, int mute);
static int aic325x_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
			     unsigned int freq, int dir);
static int aic325x_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt);
static int aic325x_set_bias_level(struct snd_soc_codec *codec,
					enum snd_soc_bias_level level);
static int aic325x_set_dai_pll(struct snd_soc_dai *dai, int pll_id, int source,
				unsigned int Fin, unsigned int Fout);

static unsigned int aic325x_codec_read(struct snd_soc_codec *codec,
			unsigned int reg);

static int aic325x_codec_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value);

static int aic325x_hp_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event);
static int aic3256_cp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event);
static int aic3256_get_runstate(struct snd_soc_codec *codec);
static int aic3256_dsp_pwrdwn_status(struct snd_soc_codec *codec);
static int aic3256_dsp_pwrup(struct snd_soc_codec *codec, int state);
static int aic3256_restart_dsps_sync(struct snd_soc_codec *codec, int rs);

static void aic3xxx_set_power_rec(struct snd_soc_codec *codec, int on);
static void set_spk_coeff(struct snd_soc_codec *codec, int on);

static inline unsigned int dsp_non_sync_mode(unsigned int state)
			{ return (!((state & 0x03) && (state & 0x30))); }

static void aic3256_firmware_load(const struct firmware *fw, void *context);

static int aic325x_trigger(struct snd_pcm_substream *ss, int t, struct snd_soc_dai *dai);
static int aic325x_prepare(struct snd_pcm_substream *ss, struct snd_soc_dai *dai);
static void aic3xxx_hp_en(struct snd_soc_codec *codec, int ena);

static int aic325x_wait(struct snd_soc_codec *codec, int reg, int mask, int val)
{
	int tmo = 60;
	int v;
	do{
		v = snd_soc_read(codec, reg);
		if(v >= 0)
		{
			if((v & mask) == val) return 0;
			msleep(5);
		} else {
			printk("aic325x wait read error\n");
			return -1;
		}
		tmo --;
	}while(tmo);
	printk("aic325x wait timeout\n");
	return -1;
}
static int hdmi_work_state = 0; /* 0:hdmi not insert, 1: hdmi insert*/

static bool codec_active = false;
static struct regulator *vgen3 = NULL;

static void aic3xxx_spk_en(struct snd_soc_codec *codec, int ena)
{
	//	if(hdmi_work_state)
	//	{
	//		gpio_direction_output(AIC_SPK_PIN, GPIO_LOW);
	//		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   9), 0x0c, 0x00);
	//		return ;
	//	}

	//printk("%s %s %d ena=%d,hdmi_work_state=%d,  \n",__FILE__,__func__,__LINE__,ena,hdmi_work_state);
	if(!ena || hdmi_work_state ){
		//gpio_direction_output(AIC_SPK_PIN, GPIO_LOW);
		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 63), 0x15);
		//printk("%s %s %d   \n",__FILE__,__func__,__LINE__);
		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   9), 0x0c, 0x00);
		//set_spk_coeff(codec, 0);
		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 63), 0xd5);
	}else{
		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 63), 0x15);
		//set_spk_coeff(codec, 1);
		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 63), 0xd5);//0xea
		//printk("%s %s %d   \n",__FILE__,__func__,__LINE__);
		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   9), 0x0c, 0x0c);
		//if(startup_or_resume){
		//	startup_or_resume = false;
		//}
		//else 
		//gpio_direction_output(AIC_SPK_PIN, GPIO_HIGH);
	}
}

#ifdef IPHONE_HS_SUPPORT
static void aic3xxx_pre_micdet(struct snd_soc_codec *codec)
{
	//printk("pre_micdet: %s,%d ,(gpio_is_valid(AIC_HPEN_PIN))=%d,\n",__func__,__LINE__,(gpio_is_valid(AIC_HPEN_PIN)));
	if(gpio_is_valid(AIC_HPEN_PIN))
	{
		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   51), 0x40, 0x00);
		gpio_direction_output(AIC_HPEN_PIN, GPIO_LOW);
		msleep(10);
		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   51), 0x40, 0x40);
		gpio_direction_output(AIC_HPEN_PIN, GPIO_HIGH);
	}
}
#endif
#ifdef HEADSET_DETECTION
static void pdet_function(struct work_struct *work);
static irqreturn_t pdet_handler(int irq, void *dev_id);
static DECLARE_DELAYED_WORK(pdet_work, pdet_function);
#endif
static bool pdet_irq_requested;

static int aic325x_poweroff(struct snd_soc_codec * codec)
{
//	printk("[%s]\n",__func__);
	if (!codec_active)
		return 0;

	codec_active = false;

	aic3xxx_spk_en(codec, 0);
	aic3xxx_hp_en(codec, 0);
	//snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   9), 0x0c, 0x0c);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 65), 0x81);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 66), 0x81);
	msleep(50);
	/*poweoff mute the audio*/
	snd_soc_write(codec, AIC3256_DAC_MUTE_CTRL_REG, 0x1C);

	msleep(50);
	aic3xxx_set_power_rec(codec, 0);

	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x3F), 0xc0, 0x00);
	msleep(10);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x0b), 0x80, 0x00);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x0c), 0x80, 0x00);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x13), 0x80, 0x00);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 5), 0x80, 0x00);
	msleep(10);
	snd_soc_update_bits(codec, AIC3256_REF_PWR_UP_CONF_REG,
			AIC3256_REF_PWR_UP_MASK,
			AIC3256_AUTO_REF_PWR_UP);

	msleep(50);
	snd_soc_update_bits(codec, AIC3256_LDO_CTRL,
			AIC3256_ANALOG_BLOCK_POWER_CONTROL_MASK,
			AIC3256_DISABLE_ANALOG_BLOCK);
	snd_soc_update_bits(codec, AIC3256_POW_CFG,
			AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
			AIC3256_ENABLE_AVDD_TO_DVDD);
	//snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 1, 9), 0x0c, 0x00);

	gpio_request(MX6SL_BRD_AIC325X_RESET, "tlv320-reset-pin");
	gpio_direction_output(MX6SL_BRD_AIC325X_RESET, 0);
	gpio_free(MX6SL_BRD_AIC325X_RESET);

	msleep(10);

	if (vgen3) regulator_disable(vgen3);

	return 0;
}

static struct snd_soc_codec * p_the_codec = 0;
static int aic325x_prepare_for_shutdown(struct notifier_block *this,
		unsigned long cmd, void *p)
{
	 //printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 
	if(p_the_codec)
	{
		struct snd_soc_codec * codec = p_the_codec;
		 //printk("%s %s %d   \n",__FILE__,__func__,__LINE__);
		 aic325x_poweroff(codec);

	 }
	 return NOTIFY_DONE;
}
static struct notifier_block aic325x_shutdown_notifier = {
	.notifier_call = aic325x_prepare_for_shutdown,
	.next = NULL,
	.priority = 0
};

static void aic3xxx_hp_en(struct snd_soc_codec *codec, int ena)
{
	//DBG("ena %d", ena);
//	printk("%s %s %d ena=%d ,hdmi_work_state=%d \n",__FILE__,__func__,__LINE__,ena,hdmi_work_state);

	if(!ena || hdmi_work_state){

		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   9), 0x30, 0x00);

		//DAC disable
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 0x3F), 0x00);

		int r = 0xff;
		int count = 50;
		while (((uint8_t)r & 0xEE) && --count) {
			r = snd_soc_read(codec,AIC3XXX_MAKE_REG(0, 0, 0x25));
			msleep(10);
		}

		//mute HP driver
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x10), 0x40);
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x11), 0x40);

	} else {

		//DAC enable
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 0x3F), 0xD6);
		//msleep(10);

//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x10), 0x00);
//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x11), 0x00);

		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   9), 0x30, 0x30);
		//mdelay(50);

//		int r = 0;
//		int count = 50;
//		while (!((uint8_t)r & 0x4) && --count) {
//			r = snd_soc_read(codec,AIC3XXX_MAKE_REG(0, 1, 0x2));
//			msleep(10);
//		}
	}
}

void codec_set_spk(bool on)
{	
#if 0
	if (!aic325x_codec)
		goto exit;

	struct snd_soc_codec * codec;
	hdmi_work_state = !on;
	codec = aic325x_codec;
	if (0 == headset_insert)
	{
		aic3xxx_spk_en(codec, on);
		//if(1 == on)
		//	gpio_direction_output(AIC_SPK_PIN, GPIO_HIGH);
	}else{
		aic3xxx_hp_en(codec, on);
	}

exit:
	return;
#endif
}
EXPORT_SYMBOL_GPL(codec_set_spk);

#define HS_TYPE_NONE 0
#define HS_TYPE_HP 1
#define HS_TYPE_HS 2
static int cur_headset_type = 1; //defaut HP type
static int cur_power_status = 0;
static int cur_power_status_rec = 0;
static void aic3xxx_set_power_rec(struct snd_soc_codec *codec, int on)
{
	return 0;
	// ext: in3
	// int: in1
	//printk("%s %d,cur_headset_type=%d\n", __func__,on,cur_headset_type);
	if(on){
		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 11), 0x82);
		//msleep(1);
		if(cur_headset_type == HS_TYPE_HS){
			//printk("%s %s %d   \n",__FILE__,__func__,__LINE__);
			snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 52), MIC_HS_RVAL);
			snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 54), MIC_HS_RVAL);
			snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 59), 50);//0x32
			snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 60), 50);//0x32
		} else {
			//printk("%s %s %d   \n",__FILE__,__func__,__LINE__);
			snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 52), MIC_INT_RVAL);
			snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 54), MIC_INT_RVAL);
			snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 59), 50);//0x32
			snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 60), 50);//0x32
		}
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  51), 0x40);	// Mic Bias enabled, Source = Avdd, 1.25V
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  81), 0xc0);	// Power up LADC/RADC
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  82), 0x08);	// Unmute LADC
	}else{
		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  0,  86), 0x80, 0x00);
		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  0,  87), 0x3e, 0x00);
		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  0,  94), 0x80, 0x00);
		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  0,  95), 0x3e, 0x00);
		// power off codec:
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  82), 0x88);	// Unmute LADC/RADC
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  51), 0x00);	// Mic Bias disabled, Source = Avdd, 1.25V
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  81), 0x00);	// Power up LADC/RADC
		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 11), 0x02);
		msleep(10);
	}
	cur_power_status_rec = on;
}
static void aic3xxx_set_power_play(struct snd_soc_codec *codec, int on)
{
	dev_dbg(codec->dev, "%s %d\n", __func__,on);
	//printk("%s %s %d   on=%d,cur_headset_type=%d\n",__FILE__,__func__,__LINE__,on,cur_headset_type);
	//#if 1
	if(on){
		// power on codec:
		// power on PA:
		if(cur_headset_type){
			aic3xxx_hp_en(codec, 1);
		} else {
			aic3xxx_spk_en(codec, 1);
		}
	}else{
		aic3xxx_spk_en(codec, 0);
		aic3xxx_hp_en(codec, 0);
		// power off codec:
	}
	//#else
	//	if(on){
	//		/*
	//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0,  5), 0x91);
	//		msleep(50);
	//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 11), 0x82);
	//		msleep(1);
	//		*/
	//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 63), 0xd5);
	//		aic325x_wait(codec, AIC3XXX_MAKE_REG(0, 0, 37), 0x88, 0x88);
	//		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 65), 0x81);
	//		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 66), 0x81);
	//		//msleep(8);
	//		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 16), 0x3F);
	//		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 17), 0x3F);
	//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 65), 0xfd);
	//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 66), 0xfd);
	//		msleep(8);
	//		aic3xxx_spk_en(codec, 1);
	//	}else{
	//		aic3xxx_spk_en(codec, 0);
	//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 65), 0x81);
	//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 66), 0x81);
	//		msleep(8);
	//		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 16), 0x40|0x3F);
	//		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 17), 0x40|0x3F);
	//		//msleep(8);
	//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 63), 0x15);
	//		aic325x_wait(codec, AIC3XXX_MAKE_REG(0, 0, 37), 0x88, 0x00);
	//		/*
	//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 11), 0x02);
	//		msleep(1);
	//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0,  5), 0x11);
	//		msleep(50);
	//		*/
	//	}
	//#endif
	cur_power_status = on;
}
static void aic3xxx_set_hs_type(struct snd_soc_codec *codec, int val)
{
	dev_dbg("%s %d,cur_headset_type=%d\n", __func__,val,cur_headset_type);
	if(cur_headset_type != val)
	{
		aic3xxx_spk_en(codec, 0);
		aic3xxx_hp_en(codec, 0);
		cur_headset_type = val;
		if(cur_power_status){
			if(cur_headset_type){
				aic3xxx_hp_en(codec, 1);
			} else {
				aic3xxx_spk_en(codec, 1);
			}
		}
		if(cur_power_status_rec){
			aic3xxx_set_power_rec(codec, 1);
		}	
	}
}
static void aic3xxx_on_headphone(struct snd_soc_codec *codec, int on)
{
	dev_dbg(codec->dev, "%s %d\n", __func__,on);
	if(on){
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 67), 0x8a);
	} else {
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 67), 0x0a);
	}
}
#ifdef HEADSET_DETECTION
static void aic3xxx_jack_get(struct snd_soc_codec *codec, int on)
{
	int status;
	//struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	dev_info(codec->dev, "%s %d\n", __func__,on);
#ifdef AP_GPIO_DET
	if(!on){
		aic3xxx_set_hs_type(codec, HS_TYPE_NONE);
		printk("aic325x no hp detected\n"); 
		return;
	}
#endif
	//aic3xxx_hp_en(codec, 1);
#ifdef MICDET_IO_ASSIST
	if(gpio_is_valid(AIC_HPEN_PIN))
	{
		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   51), 0x40, 0x00);
		gpio_direction_output(AIC_HPEN_PIN, GPIO_LOW);
	}
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 52), 0x08);
	msleep(10);
	printk("aic325x HS Status %d detected\n", snd_soc_read(codec, AIC3XXX_MAKE_REG(0, 0, 52)));
	if(snd_soc_read(codec, AIC3XXX_MAKE_REG(0, 0, 52)) & 0x02)
	{
		snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   51), 0x40, 0x40);
		if(gpio_is_valid(AIC_HPEN_PIN))
		{
			gpio_direction_output(AIC_HPEN_PIN, GPIO_HIGH);
		}
		aic3xxx_set_hs_type(codec, HS_TYPE_HS);
		printk("aic325x hs detected\n"); 
	}
	else
	{
		if(gpio_is_valid(AIC_HPEN_PIN))
		{
			gpio_direction_output(AIC_HPEN_PIN, GPIO_HIGH);
		}
		aic3xxx_set_hs_type(codec, HS_TYPE_HP);
		printk("aic325x hp detected\n"); 
	}
#else // MICDET_IO_ASSIST
#ifdef IPHONE_HS_SUPPORT
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 48), 0x00);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 67), 0x03); // hp det dis
	aic3xxx_pre_micdet(codec);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 67), 0x83); // hp det ena
	mdelay(30);
#endif
	status = snd_soc_read(codec, AIC3256_HEADSET_DETECT);
#ifdef AP_GPIO_DET
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 67), 0x07); // hp det dis
#else
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 67), 0x87); // hp det ena
#endif
	printk("aic325x hs status %d\n", status); 
	switch (status & AIC3256_JACK_TYPE_MASK) {
	case AIC3256_JACK_WITH_MIC:
		aic3xxx_set_hs_type(codec, HS_TYPE_HS);
		break;
	case AIC3256_JACK_WITHOUT_MIC:
		aic3xxx_set_hs_type(codec, HS_TYPE_HP);
		break;
	default:
		aic3xxx_set_hs_type(codec, HS_TYPE_NONE);
	}
	snd_soc_read(codec, AIC3XXX_MAKE_REG(0, 0, 44));
	snd_soc_read(codec, AIC3XXX_MAKE_REG(0, 0, 46));
#ifdef IPHONE_HS_SUPPORT
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 48), 0xC0);
#endif
#endif // MICDET_IO_ASSIST
}
#endif

static int cur_dsp_mode = 0;

static int aic3xxx_dsp_mode_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	//ucontrol->value.integer.value[0] = cur_dsp_mode;
	return 0;
}

static int aic3xxx_dsp_mode_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
#if 0
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(codec);

	int next_mode=0,next_cfg=0;

	next_mode = (ucontrol->value.integer.value[0]>>8);
	next_cfg = (ucontrol->value.integer.value[0] & 0x0FF);

	//aic3xxx_set_dsp_mode(aic325x->dsp_priv, next_mode, next_cfg);

	cur_dsp_mode = (next_mode << 8) + next_cfg;

#endif
	return 0;
}

/*
* Global Variable
*/

/* whenever aplay/arecord is run, aic325x_hw_params() function gets called.
 * This function reprograms the clock dividers etc. this flag can be used to
 * disable this when the clock dividers are programmed by pps config file
 */


static const char * const mute[] = { "Unmute", "Mute" };

/* DAC Volume Soft Step Control */
static const char * const dacsoftstep_control[] = { "1 step/sample",
						"1 step/2 sample",
						"disabled" };
SOC_ENUM_SINGLE_DECL(dac_vol_soft_setp_enum, AIC3256_DAC_CHN_REG, 0,
						dacsoftstep_control);

/* Volume Mode Selection Control */
static const char * const volume_extra[] = { "L&R Ind Vol", "LVol=RVol",
								"RVol=LVol" };
/* DAC Volume Mode Selection */
SOC_ENUM_SINGLE_DECL(dac_vol_extra_enum, AIC3256_DAC_MUTE_CTRL_REG, 0,
						volume_extra);

/* Beep Master Volume Control */
SOC_ENUM_SINGLE_DECL(beep_master_vol_enum, AIC3256_BEEP_CTRL_REG2, 6,
						volume_extra);

/* Headset Detection Enable/Disable Control */
static const char * const headset_detection[] = { "Enabled", "Disabled" };
SOC_ENUM_SINGLE_DECL(hs_det_ctrl_enum, AIC3256_HEADSET_DETECT, 7,
						headset_detection);

/* MIC BIAS Voltage Control */
static const char * const micbias_voltage[] = { "1.04/1.25V", "1.425/1.7V",
						"2.075/2.5V", "POWER SUPPY" };
SOC_ENUM_SINGLE_DECL(micbias_voltage_enum, AIC3256_MICBIAS_CTRL, 4,
						micbias_voltage);

/* IN1L to Left MICPGA Positive Terminal Selection */
static const char * const micpga_selection[] = { "off", "10k", "20k", "40k" };
SOC_ENUM_SINGLE_DECL(IN1L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 6,
						micpga_selection);

/* Left HP Driver Mute Control */
SOC_ENUM_SINGLE_DECL(left_hp_mute_enum, AIC3256_HPL_GAIN, 6, mute);

/* Right HP Driver Mute Control */
SOC_ENUM_SINGLE_DECL(right_hp_mute_enum, AIC3256_HPR_GAIN, 6, mute);

/* Line Out Driver Mute Control */
SOC_ENUM_SINGLE_DECL(left_lo_mute_enum, AIC3256_LOL_GAIN, 6, mute);
SOC_ENUM_SINGLE_DECL(right_lo_mute_enum, AIC3256_LOR_GAIN, 6, mute);

/* IN2L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN2L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 4,
						micpga_selection);

/* IN3L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN3L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 4,
						micpga_selection);

/* IN1R to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN1R_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 2,
						micpga_selection);

/* CM1L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(CM1L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_PIN_CFG, 0,
						micpga_selection);

/* IN2R to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN2R_LMICPGA_P_sel_enum, AIC3256_LMICPGA_NIN_CFG, 6,
						micpga_selection);

/* IN3R to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(IN3R_LMICPGA_P_sel_enum, AIC3256_LMICPGA_NIN_CFG, 4,
						micpga_selection);

/*CM2L to Left MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(CM2L_LMICPGA_P_sel_enum, AIC3256_LMICPGA_NIN_CFG, 2,
						micpga_selection);

/* IN1R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in1r_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 6,
						micpga_selection);

/* IN2R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in2r_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 4,
						micpga_selection);

/* IN3R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in3r_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 2,
						micpga_selection);

/* IN2L to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in2l_rmicpga_enum, AIC3256_RMICPGA_PIN_CFG, 0,
						micpga_selection);

/* CM1R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(cm1r_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 6,
						micpga_selection);

/* IN1L to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in1l_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 4,
						micpga_selection);

/* IN3L to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(in3l_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 2,
						micpga_selection);

/* CM2R to Right MICPGA Positive Terminal Selection */
SOC_ENUM_SINGLE_DECL(cm2r_rmicpga_enum, AIC3256_RMICPGA_NIN_CFG, 0,
						micpga_selection);

/* Power up/down */
static const char * const powerup[] = { "Power Down", "Power Up" };

/* Mic Bias Power up/down */
SOC_ENUM_SINGLE_DECL(micbias_pwr_ctrl_enum, AIC3256_MICBIAS_CTRL, 6, powerup);

/* Left DAC Power Control */
SOC_ENUM_SINGLE_DECL(ldac_power_enum, AIC3256_DAC_CHN_REG, 7, powerup);

/* Right DAC Power Control */
SOC_ENUM_SINGLE_DECL(rdac_power_enum, AIC3256_DAC_CHN_REG, 6, powerup);

/* Left ADC Power Control */
SOC_ENUM_SINGLE_DECL(ladc_pwr_ctrl_enum, AIC3256_ADC_CHN_REG, 7, powerup);
/* Right ADC Power Control */
SOC_ENUM_SINGLE_DECL(radc_pwr_ctrl_enum, AIC3256_ADC_CHN_REG, 6, powerup);

/* HeadPhone Driver Power Control */
SOC_ENUM_DOUBLE_DECL(hp_pwr_ctrl_enum, AIC3256_OUT_PWR_CTRL, 5, 4, powerup);

/*Line-Out Driver Power Control */
SOC_ENUM_DOUBLE_DECL(lineout_pwr_ctrl_enum, AIC3256_OUT_PWR_CTRL, 3, 2,
						powerup);

/* Mixer Amplifiers Power Control */
SOC_ENUM_DOUBLE_DECL(mixer_amp_pwr_ctrl_enum, AIC3256_OUT_PWR_CTRL, 1, 0,
						powerup);

/* Mic Bias Generation */
static const char * const vol_generation[] = { "AVDD", "LDOIN" };
SOC_ENUM_SINGLE_DECL(micbias_voltage_ctrl_enum, AIC3256_MICBIAS_CTRL, 3,
						vol_generation);

/* DAC Data Path Control */
static const char * const path_control[] = { "Disabled", "LDAC Data",
						"RDAC Data", "L&RDAC Data" };
/* Left DAC Data Path Control */
SOC_ENUM_SINGLE_DECL(ldac_data_path_ctrl_enum, AIC3256_DAC_CHN_REG, 4,
						path_control);

/* Right DAC Data Path Control */
SOC_ENUM_SINGLE_DECL(rdac_data_path_ctrl_enum, AIC3256_DAC_CHN_REG, 2,
						path_control);

/* Audio gain control (AGC) Enable/Disable Control */
static const char * const disable_enable[] = { "Disabled", "Enabled" };

/* Left Audio gain control (AGC) Enable/Disable Control */
SOC_ENUM_SINGLE_DECL(left_agc_enable_disable_enum, AIC3256_LEFT_AGC_REG1, 7,
						disable_enable);

/* Left/Right Audio gain control (AGC) Enable/Disable Control */
SOC_ENUM_SINGLE_DECL(right_agc_enable_disable_enum, AIC3256_RIGHT_AGC_REG1, 7,
						disable_enable);

/* Left MICPGA Gain Enabled/Disable */
SOC_ENUM_SINGLE_DECL(left_micpga_ctrl_enum, AIC3256_LMICPGA_VOL_CTRL, 7,
						disable_enable);

/* Right MICPGA Gain Enabled/Disable */
SOC_ENUM_SINGLE_DECL(right_micpga_ctrl_enum, AIC3256_RMICPGA_VOL_CTRL, 7,
						disable_enable);

/* DRC Enable/Disable Control */
SOC_ENUM_DOUBLE_DECL(drc_ctrl_enum, AIC3256_DRC_CTRL_REG1, 6, 5,
						disable_enable);

/* Beep generator Enable/Disable control */
SOC_ENUM_SINGLE_DECL(beep_gen_ctrl_enum, AIC3256_BEEP_CTRL_REG1, 7,
						disable_enable);

/* Headphone ground centered mode enable/disable control */
SOC_ENUM_SINGLE_DECL(hp_gnd_centred_mode_ctrl, AIC3256_HP_DRIVER_CONF_REG, 4,
						disable_enable);

/* Audio loopback enable/disable control */
SOC_ENUM_SINGLE_DECL(audio_loopback_enum, AIC3256_INTERFACE_SET_REG_3, 5, disable_enable);

/* DMIC intput Selection control */
static const char * const dmic_input_sel[] = { "GPIO", "SCLK", "DIN" };
SOC_ENUM_SINGLE_DECL(dmic_input_enum, AIC3256_ADC_CHN_REG, 4, dmic_input_sel);

/*charge pump Enable*/
static const char * const charge_pump_ctrl_enum[] = { "Power_Down",
							"Reserved",
							"Power_Up" };
SOC_ENUM_SINGLE_DECL(charge_pump_ctrl, AIC3256_POW_CFG, 0,
						charge_pump_ctrl_enum);

/* DAC volume DB scale */
static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -6350, 50, 0);
/* ADC volume DB scale */
static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -1200, 50, 0);
/* Output Gain in DB scale */
static const DECLARE_TLV_DB_SCALE(output_gain_tlv, -600, 100, 0);
/* MicPGA Gain in DB */
static const DECLARE_TLV_DB_SCALE(micpga_gain_tlv, 0, 50, 0);

/* Various Controls For AIC325x */
static const struct snd_kcontrol_new aic325x_snd_controls[] = {
	/* IN1L to HPL Volume Control */
	SOC_SINGLE("IN1L to HPL volume control", AIC3256_IN1L_HPL_CTRL,
						0, 0x72, 1),

	SOC_ENUM("Left HP driver mute", left_hp_mute_enum),
	SOC_ENUM("Right HP driver mute", right_hp_mute_enum),	

	SOC_ENUM("LOL driver mute", left_lo_mute_enum),
	SOC_ENUM("LOR driver mute", right_lo_mute_enum),	

	/* IN1R to HPR Volume Control */
	SOC_SINGLE("IN1R to HPR volume control", AIC3256_IN1R_HPR_CTRL,
						0, 0x72, 1),

	/* IN1L to HPL routing */
	SOC_SINGLE("IN1L to HPL Route", AIC3256_HPL_ROUTE_CTRL, 2, 1, 0),

	/* MAL output to HPL */
	SOC_SINGLE("MAL Output to HPL Route", AIC3256_HPL_ROUTE_CTRL, 1, 1, 0),

	/*MAR output to HPL */
	SOC_SINGLE("MAR Output to HPL Route", AIC3256_HPL_ROUTE_CTRL, 0, 1, 0),

	/* IN1R to HPR routing */
	SOC_SINGLE("IN1R to HPR Route", AIC3256_HPR_ROUTE_CTRL, 2, 1, 0),

	/* MAR to HPR routing */
	SOC_SINGLE("MAR Output to HPR Route", AIC3256_HPR_ROUTE_CTRL, 1, 1, 0),

	/* HPL Output to HRP routing */
	SOC_SINGLE("HPL Output to HPR Route", AIC3256_HPR_ROUTE_CTRL, 0, 1, 0),

	/* MAL Output to LOL routing*/
	SOC_SINGLE("MAL Output to LOL Route", AIC3256_LOL_ROUTE_CTRL, 1, 1, 0),

	/* LOR Output to LOL routing*/
	SOC_SINGLE("LOR Output to LOL Route", AIC3256_LOL_ROUTE_CTRL, 0, 1, 0),

	/* MAR Output to LOR routing*/
	SOC_SINGLE("MAR Outout to LOR Route", AIC3256_LOR_ROUTE_CTRL, 1, 1, 0),

	/* DRC Threshold value Control */
	SOC_SINGLE("DRC Threshold value",
					AIC3256_DRC_CTRL_REG1, 2, 0x07, 0),

	/* DRC Hysteresis value control */
	SOC_SINGLE("DRC Hysteresis value",
					AIC3256_DRC_CTRL_REG1, 0, 0x03, 0),

	/* DRC Hold time control */
	SOC_SINGLE("DRC hold time", AIC3256_DRC_CTRL_REG2, 3, 0x0F, 0),

	/* DRC Attack rate control */
	SOC_SINGLE("DRC attack rate", AIC3256_DRC_CTRL_REG3, 4, 0x0F, 0),

	/* DRC Decay rate control */
	SOC_SINGLE("DRC decay rate", AIC3256_DRC_CTRL_REG3, 0, 0x0F, 0),

	/* Beep Length MSB control */
	SOC_SINGLE("Beep Length MSB", AIC3256_BEEP_CTRL_REG3, 0, 255, 0),

	/* Beep Length MID control */
	SOC_SINGLE("Beep Length MID", AIC3256_BEEP_CTRL_REG4, 0, 255, 0),

	/* Beep Length LSB control */
	SOC_SINGLE("Beep Length LSB", AIC3256_BEEP_CTRL_REG5, 0, 255, 0),

	/* Beep Sin(x) MSB control */
	SOC_SINGLE("Beep Sin(x) MSB", AIC3256_BEEP_CTRL_REG6, 0, 255, 0),
	/* Beep Sin(x) LSB control */
	SOC_SINGLE("Beep Sin(x) LSB", AIC3256_BEEP_CTRL_REG7, 0, 255, 0),

	/* Beep Cos(x) MSB control */
	SOC_SINGLE("Beep Cos(x) MSB", AIC3256_BEEP_CTRL_REG8, 0, 255, 0),

	/* Beep Cos(x) LSB control */
	SOC_SINGLE("Beep Cos(x) LSB", AIC3256_BEEP_CTRL_REG9, 0, 255, 0),


	/* Left/Right DAC Digital Volume Control */
	//SOC_DOUBLE_R_SX_TLV("Digital Playback Volume",
	SOC_DOUBLE_R_SX_TLV("Digital",
			AIC3256_LDAC_VOL, AIC3256_RDAC_VOL, 8, 0x81, 0xe6/*0x30*/,
			dac_vol_tlv),

	/* Left/Right ADC Fine Gain Adjust */
	SOC_DOUBLE("L&R ADC Fine Gain Adjust", AIC3256_ADC_FGA, 4, 0, 0x04, 0),

	/* Left/Right ADC Volume Control */
	SOC_DOUBLE_R_SX_TLV("ADC Digital Volume Control",
		AIC3256_LADC_VOL, AIC3256_RADC_VOL, 7, 0xffffff68, 0x28 ,
						adc_vol_tlv),

	/*HP Driver Gain Control*/
	SOC_DOUBLE_R_SX_TLV("HP Driver Gain", AIC3256_HPL_GAIN,
					AIC3256_HPR_GAIN, 6, 0xfffffffa,
					0xe, output_gain_tlv),

	/*LO Driver Gain Control*/
	SOC_DOUBLE_R_SX_TLV("Line Driver Gain", AIC3256_LOL_GAIN,
					AIC3256_LOR_GAIN, 6,
					0xfffffffa, 0x1d , output_gain_tlv),


	/* Mixer Amplifier Volume Control */
	SOC_DOUBLE_R("Mixer_Amp_Vol_Ctrl",
			AIC3256_MAL_CTRL_REG, AIC3256_MAR_CTRL_REG,
			0, 0x28, 1),


	/*Left/Right MICPGA Volume Control */
	SOC_DOUBLE_R_TLV("L&R_MICPGA_Vol_Ctrl",
	AIC3256_LMICPGA_VOL_CTRL, AIC3256_RMICPGA_VOL_CTRL, 0, 0x5F,
			0, micpga_gain_tlv),

	/* Beep generator Volume Control */
	SOC_DOUBLE_R("Beep_gen_Vol_Ctrl",
			AIC3256_BEEP_CTRL_REG1, AIC3256_BEEP_CTRL_REG2,
			0, 0x3F, 1),

	/* Left/Right AGC Target level control */
	SOC_DOUBLE_R("AGC Target Level Control",
			AIC3256_LEFT_AGC_REG1, AIC3256_RIGHT_AGC_REG1,
			4, 0x07, 1),

	/* Left/Right AGC Hysteresis Control */
	SOC_DOUBLE_R("AGC Hysteresis Control",
			AIC3256_LEFT_AGC_REG1, AIC3256_RIGHT_AGC_REG1,
			0, 0x03, 0),

	/*Left/Right AGC Maximum PGA applicable */
	SOC_DOUBLE_R("AGC Maximum PGA Control",
			AIC3256_LEFT_AGC_REG3, AIC3256_RIGHT_AGC_REG3,
			0, 0x7F, 0),

	/* Left/Right AGC Noise Threshold */
	SOC_DOUBLE_R("AGC Noise Threshold",
			AIC3256_LEFT_AGC_REG2, AIC3256_RIGHT_AGC_REG2,
			1, 0x1F, 1),

	/* Left/Right AGC Attack Time control */
	SOC_DOUBLE_R("AGC Attack Time control",
			AIC3256_LEFT_AGC_REG4, AIC3256_RIGHT_AGC_REG4,
			3, 0x1F, 0),

	/* Left/Right AGC Decay Time control */
	SOC_DOUBLE_R("AGC Decay Time control",
			AIC3256_LEFT_AGC_REG5, AIC3256_RIGHT_AGC_REG5,
			3, 0x1F, 0),

	/* Left/Right AGC Noise Debounce control */
	SOC_DOUBLE_R("AGC Noice bounce control",
			AIC3256_LEFT_AGC_REG6, AIC3256_RIGHT_AGC_REG6,
			0, 0x1F, 0),

	/* Left/Right AGC Signal Debounce control */
	SOC_DOUBLE_R("AGC_Signal bounce ctrl",
		AIC3256_LEFT_AGC_REG7, AIC3256_RIGHT_AGC_REG7, 0, 0x0F, 0),

	/* DAC Signal Processing Block Control*/
	SOC_SINGLE("DAC PRB Selection(1 to 25)", AIC3256_DAC_PRB, 0, 0x19, 0),
	/* ADC Signal Processing Block Control */
	SOC_SINGLE("ADC PRB Selection(1 to 18)", AIC3256_ADC_PRB, 0, 0x12, 0),

	/*charge pump configuration for n/8 peak load current*/
	SOC_SINGLE("Charge_pump_peak_load_conf",
				AIC3256_CHRG_CTRL_REG, 4, 8, 0),

	/*charge pump clock divide control*/
	SOC_SINGLE("charge_pump_clk_divider_ctrl", AIC3256_CHRG_CTRL_REG,
			0, 16, 0),

	/*HPL, HPR master gain control in ground centerd mode */
	SOC_SINGLE("HP_gain_ctrl_gnd_centered_mode",
				AIC3256_HP_DRIVER_CONF_REG, 5, 3, 0),

	/*headphone amplifier compensation adjustment */
	SOC_SINGLE(" hp_amp_compensation_adjustment",
				AIC3256_HP_DRIVER_CONF_REG, 7, 1, 0),

	/*headphone driver power configuration*/
	SOC_SINGLE(" HP_drv_pwr_conf",
				AIC3256_HP_DRIVER_CONF_REG, 2, 4, 0),

	/*DC offset correction*/
	SOC_SINGLE("DC offset correction", AIC3256_HP_DRIVER_CONF_REG, 0, 4, 0),


	SOC_ENUM("Mic_Bias_Power_ctrl", micbias_pwr_ctrl_enum),

	SOC_SINGLE_EXT("DSP MODE", SND_SOC_NOPM, 0, 0xffff, 0, aic3xxx_dsp_mode_get, aic3xxx_dsp_mode_put),

};

/*
 *----------------------------------------------------------------------------
 * @struct  snd_soc_codec_dai_ops |
 *          It is SoC Codec DAI Operations structure
 *----------------------------------------------------------------------------
 */
struct snd_soc_dai_ops aic325x_dai_ops = {
	.hw_params = aic325x_hw_params,
	.startup = aic325x_startup,
	.shutdown = aic325x_shutdown,
	.digital_mute = aic325x_mute,
	.set_sysclk = aic325x_set_dai_sysclk,
	.set_fmt = aic325x_set_dai_fmt,
	.set_pll = aic325x_set_dai_pll,
	.prepare = aic325x_prepare,
	.trigger = aic325x_trigger,
};

/*
 * It is SoC Codec DAI structure which has DAI capabilities viz., playback
 * and capture, DAI runtime information viz. state of DAI and pop wait state,
 * and DAI private data. The aic31xx rates ranges from 8k to 192k The PCM
 * bit format supported are 16, 20, 24 and 32 bits
 */


static struct snd_soc_dai_driver tlv320aic325x_dai_driver[] = {
	{
	.name = "tlv320aic325x-MM_EXT",
	.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIC325x_RATES,
			.formats = AIC325x_FORMATS,
		},
	.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = AIC325x_RATES,
			.formats = AIC325x_FORMATS,
		},
	.ops = &aic325x_dai_ops,
},

};

/* Left HPL Mixer */
static const struct snd_kcontrol_new hpl_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", AIC3256_HPL_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("IN1_L switch", AIC3256_HPL_ROUTE_CTRL, 2, 1, 0),
	SOC_DAPM_SINGLE("MAL switch", AIC3256_HPL_ROUTE_CTRL, 1, 1, 0),
	SOC_DAPM_SINGLE("MAR switch", AIC3256_HPL_ROUTE_CTRL, 0, 1, 0),
};

static const char * const adc_mux_text[] = {
	"Analog", "Digital"
};

SOC_ENUM_SINGLE_DECL(adcl_enum, AIC3256_ADC_CHN_REG, 3, adc_mux_text);
SOC_ENUM_SINGLE_DECL(adcr_enum, AIC3256_ADC_CHN_REG, 2, adc_mux_text);

static const struct snd_kcontrol_new adcl_mux =
	SOC_DAPM_ENUM("Left ADC Route", adcl_enum);

static const struct snd_kcontrol_new adcr_mux =
	SOC_DAPM_ENUM("Right ADC Route", adcr_enum);

/* Right HPR Mixer */
static const struct snd_kcontrol_new hpr_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("L_DAC switch", AIC3256_HPR_ROUTE_CTRL, 4, 1, 0),
	SOC_DAPM_SINGLE("R_DAC switch", AIC3256_HPR_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("IN1_R switch", AIC3256_HPR_ROUTE_CTRL, 2, 1, 0),
	SOC_DAPM_SINGLE("MAR switch", AIC3256_HPR_ROUTE_CTRL, 1, 1, 0),
};

/* Left Line out mixer */
static const struct snd_kcontrol_new lol_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", AIC3256_LOL_ROUTE_CTRL, 4, 1, 0),
	SOC_DAPM_SINGLE("L_DAC switch", AIC3256_LOL_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("MAL switch", AIC3256_LOL_ROUTE_CTRL, 1, 1, 0),
	SOC_DAPM_SINGLE("LOR switch", AIC3256_LOL_ROUTE_CTRL, 0, 1, 0),
};
/* Right Line out Mixer */
static const struct snd_kcontrol_new lor_output_mixer_controls[] = {
	SOC_DAPM_SINGLE("R_DAC switch", AIC3256_LOR_ROUTE_CTRL, 3, 1, 0),
	SOC_DAPM_SINGLE("MAR switch", AIC3256_LOR_ROUTE_CTRL, 1, 1, 0),
};

/* Left Input Mixer */
static const struct snd_kcontrol_new left_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1_L switch", AIC3256_LMICPGA_PIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN2_L switch", AIC3256_LMICPGA_PIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_L switch", AIC3256_LMICPGA_PIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("IN1_R switch", AIC3256_LMICPGA_PIN_CFG, 0, 3, 0),

	SOC_DAPM_SINGLE("CM1L switch", AIC3256_LMICPGA_NIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN2_R switch", AIC3256_LMICPGA_NIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_R switch", AIC3256_LMICPGA_NIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("CM2L switch", AIC3256_LMICPGA_NIN_CFG, 0, 3, 0),
};

/* Right Input Mixer */
static const struct snd_kcontrol_new right_input_mixer_controls[] = {
	SOC_DAPM_SINGLE("IN1_R switch", AIC3256_RMICPGA_PIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN2_R switch", AIC3256_RMICPGA_PIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_R switch", AIC3256_RMICPGA_PIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("IN2_L switch", AIC3256_RMICPGA_PIN_CFG, 0, 3, 0),
	SOC_DAPM_SINGLE("CM1R switch", AIC3256_RMICPGA_NIN_CFG, 6, 3, 0),
	SOC_DAPM_SINGLE("IN1_L switch", AIC3256_RMICPGA_NIN_CFG, 4, 3, 0),
	SOC_DAPM_SINGLE("IN3_L switch", AIC3256_RMICPGA_NIN_CFG, 2, 3, 0),
	SOC_DAPM_SINGLE("CM2R switch", AIC3256_RMICPGA_NIN_CFG, 0, 3, 0),
};

static const char *asilin_text[] = {
	"Off", "ASI Left In","ASI Right In","ASI MonoMix In"
};
SOC_ENUM_SINGLE_DECL(asilin_enum, AIC3256_DAC_CHN_REG, 4, asilin_text);
static const struct snd_kcontrol_new asilin_control =
SOC_DAPM_ENUM("ASIIn Left Route", asilin_enum);      

static const char *asirin_text[] = {
	"Off", "ASI Right In","ASI Left In","ASI MonoMix In"
};
SOC_ENUM_SINGLE_DECL(asirin_enum, AIC3256_DAC_CHN_REG, 2, asirin_text);
static const struct snd_kcontrol_new asirin_control =
SOC_DAPM_ENUM("ASIIn Right Route", asirin_enum);      

/**$
 * pll_power_on_event: provide delay after widget power up
 * @w: pointer variable to dapm_widget,
 * @kcontrolr: pointer variable to sound control,
 * @event:	integer to event,
 *
 * Return value: 0 for success
 */
static int pll_power_on_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	if (event == (SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD))
		mdelay(10);
	return 0;
}

/**
 *aic325x_dac_event: Headset popup reduction and powering up dsps together
 *			when they are in sync mode
 * @w: pointer variable to dapm_widget
 * @kcontrol: pointer to sound control
 * @event: event element information
 *
 * Returns 0 for success.
 */
static int aic325x_dac_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	int reg_mask = 0;
	int ret_wbits = 0;
	int run_state_mask;
	int sync_needed = 0, non_sync_state = 0;
	int other_dsp = 0, run_state = 0;
	struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(w->codec);

	if (w->shift == 7) {
		reg_mask = AIC3256_LDAC_POWER_STATUS_MASK ;
		//run_state_mask = AIC3XXX_COPS_MDSP_D_L; // TODO: qkdang
	}
	if (w->shift == 6) {
		reg_mask = AIC3256_RDAC_POWER_STATUS_MASK ;
		//run_state_mask = AIC3XXX_COPS_MDSP_D_R ; // TODO: qkdang
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
			AIC3256_DAC_FLAG,
			reg_mask, reg_mask,
			AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER);
		sync_needed = aic3xxx_reg_read(w->codec->control_data,
			AIC3256_DAC_PRB);
		non_sync_state =
			dsp_non_sync_mode(aic325x->dsp_runstate);
		other_dsp =aic325x->dsp_runstate ;//& AIC3XXX_COPS_MDSP_A;

		if (sync_needed && non_sync_state && other_dsp) {
			run_state =
				aic3256_get_runstate(
				aic325x->codec->control_data);
			aic3256_dsp_pwrdwn_status(aic325x->codec);
			aic3256_dsp_pwrup(aic325x->codec, run_state);
		}
		aic325x->dsp_runstate |= run_state_mask;
		if (!ret_wbits) {
			dev_err(w->codec->dev, "DAC_post_pmu timed out\n");
			return -1;
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
			AIC3256_DAC_FLAG, reg_mask, 0,
			AIC3256_TIME_DELAY, AIC3256_DELAY_COUNTER);
		aic325x->dsp_runstate =
			(aic325x->dsp_runstate & ~run_state_mask);
		if (!ret_wbits) {
			dev_err(w->codec->dev, "DAC_post_pmd timed out\n");
			return -1;
		}
		break;
	default:
		BUG();
		return -EINVAL;
	}
	return 0;
				}

/**
 * aic325x_adc_event: To get DSP run state to perform synchronization
 * @w: pointer variable to dapm_widget
 * @kcontrol: pointer to sound control
 * @event: event element information
 *
 * Returns 0 for success.
 */
static int aic325x_adc_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	int run_state = 0;
	int non_sync_state = 0, sync_needed = 0;
	int other_dsp = 0;
	int run_state_mask = 0;
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(w->codec);
	int reg_mask = 0;
	int ret_wbits = 0;

	if (w->shift == 7) {
		reg_mask = AIC3256_LADC_POWER_MASK;
		//run_state_mask = AIC3XXX_COPS_MDSP_A_L; // TODO: qkdang
	}
	if (w->shift == 6) {
		reg_mask = AIC3256_RADC_POWER_MASK;
		//run_state_mask = AIC3XXX_COPS_MDSP_A_R; // TODO: qkdang
	}

	switch (event) {
			case SND_SOC_DAPM_POST_PMU:
				ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
					AIC3256_ADC_FLAG , reg_mask,
					reg_mask, AIC3256_TIME_DELAY,
					AIC3256_DELAY_COUNTER);

				sync_needed = aic3xxx_reg_read(w->codec->control_data,
					AIC3256_DAC_PRB);
				non_sync_state = dsp_non_sync_mode(aic3256->dsp_runstate);
				other_dsp = aic3256->dsp_runstate;/// & AIC3XXX_COPS_MDSP_D;
				if (sync_needed && non_sync_state && other_dsp) {
					run_state = aic3256_get_runstate(
						aic3256->codec);
					aic3256_dsp_pwrdwn_status(aic3256->codec);
					aic3256_dsp_pwrup(aic3256->codec, run_state);
				}
				aic3256->dsp_runstate |= run_state_mask;
				if (!ret_wbits) {
					dev_err(w->codec->dev, "ADC POST_PMU timedout\n");
					return -1;
				}
				break;

			case SND_SOC_DAPM_POST_PMD:
				ret_wbits = aic3xxx_wait_bits(w->codec->control_data,
					AIC3256_ADC_FLAG, reg_mask, 0,
					AIC3256_TIME_DELAY,
					AIC3256_DELAY_COUNTER);
				aic3256->dsp_runstate = (aic3256->dsp_runstate &
					~run_state_mask);
				if (!ret_wbits) {
					dev_err(w->codec->dev, "ADC POST_PMD timedout\n");
					return -1;
				}
				break;

			default:
				BUG();
				return -EINVAL;
	}
	return 0;
}

/* AIC325x Widget Structure */
static const struct snd_soc_dapm_widget aic325x_dapm_widgets[] = {

	SND_SOC_DAPM_AIF_IN("ASIIN", "ASI Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_PGA("ASILIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASIRIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASIMonoMixIN", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("ASI IN Port", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX("ASIIn Left Route",
			SND_SOC_NOPM, 0, 0, &asilin_control),       
	SND_SOC_DAPM_MUX("ASIIn Right Route",
			SND_SOC_NOPM, 0, 0, &asirin_control),       
	/* dapm widget (stream domain) for left DAC */
	SND_SOC_DAPM_DAC_E("Left DAC", NULL, AIC3256_DAC_CHN_REG,
			7, 0, aic325x_dac_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	/* dapm widget (stream domain) for right DAC */
	SND_SOC_DAPM_DAC_E("Right DAC", NULL, AIC3256_DAC_CHN_REG,
			6, 0, aic325x_dac_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	/* dapm widget (path domain) for left DAC_L Mixer */
	SND_SOC_DAPM_MIXER("HPL Output Mixer", SND_SOC_NOPM, 0, 0,
				&hpl_output_mixer_controls[0],
				ARRAY_SIZE(hpl_output_mixer_controls)),

	/* dapm widget (path domain) for right DAC_R mixer */
	SND_SOC_DAPM_MIXER("HPR Output Mixer", SND_SOC_NOPM, 0, 0,
	&hpr_output_mixer_controls[0], ARRAY_SIZE(hpr_output_mixer_controls)),

	/* dapm widget for Left Head phone Power */
	SND_SOC_DAPM_PGA_S("HPL PGA", 5,AIC3256_OUT_PWR_CTRL, 5, 0,
				aic325x_hp_event,
				0),

	/* dapm widget (path domain) for Left Line-out Output Mixer */
	SND_SOC_DAPM_MIXER("LOL Output Mixer", SND_SOC_NOPM, 0, 0,
	&lol_output_mixer_controls[0], ARRAY_SIZE(lol_output_mixer_controls)),

	/* dapm widget for Left Line-out Power */
	SND_SOC_DAPM_PGA_S("LOL PGA", 2,AIC3256_OUT_PWR_CTRL, 3, 0,
				NULL, 0),



	/* dapm widget for Right Head phone Power */
	SND_SOC_DAPM_PGA_S("HPR PGA", 5,AIC3256_OUT_PWR_CTRL, 4, 0,
				aic325x_hp_event, SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),

	/* dapm widget for (path domain) Right Line-out Output Mixer */
	SND_SOC_DAPM_MIXER("LOR Output Mixer", SND_SOC_NOPM, 0, 0,
			&lor_output_mixer_controls[0],
			ARRAY_SIZE(lor_output_mixer_controls)),

	/* dapm widget for Right Line-out Power */
	SND_SOC_DAPM_PGA_S("LOR PGA", 2,AIC3256_OUT_PWR_CTRL, 2, 0,
				NULL, 0),

	/* dapm supply widget for Charge pump */
	SND_SOC_DAPM_SUPPLY_S("Charge Pump", 4,AIC3256_POW_CFG, 1, 0, aic3256_cp_event,
						SND_SOC_DAPM_POST_PMU ),
	/* Input DAPM widget for CM */
	SND_SOC_DAPM_INPUT("CM"),
	/* Input DAPM widget for CM1L */
	SND_SOC_DAPM_INPUT("CM1L"),
	/* Input DAPM widget for CM2L */
	SND_SOC_DAPM_INPUT("CM2L"),
	/* Input DAPM widget for CM1R */
	SND_SOC_DAPM_INPUT("CM1R"),
	/* Input DAPM widget for CM2R */
	SND_SOC_DAPM_INPUT("CM2R"),

	/* Stream widget for Left ADC */
	SND_SOC_DAPM_ADC_E("Left ADC", "Left Capture", AIC3256_ADC_CHN_REG,
			7, 0, aic325x_adc_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),


	/* Stream widget for Right ADC */
	SND_SOC_DAPM_ADC_E("Right ADC", "Right Capture", AIC3256_ADC_CHN_REG,
			6, 0, aic325x_adc_event, SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_POST_PMD),

	/* Left Inputs to Left MicPGA */
	SND_SOC_DAPM_PGA_S("Left MicPGA", 0,AIC3256_LMICPGA_VOL_CTRL ,
			7, 1, NULL, 0),

	/* Right Inputs to Right MicPGA */
	SND_SOC_DAPM_PGA_S("Right MicPGA", 0,AIC3256_RMICPGA_VOL_CTRL,
			7, 1, NULL, 0),

	/* Left MicPGA to Mixer PGA Left */
	SND_SOC_DAPM_PGA_S("MAL PGA", 1,AIC3256_OUT_PWR_CTRL , 1, 0, NULL, 0),

	/* Right Inputs to Mixer PGA Right */
	SND_SOC_DAPM_PGA_S("MAR PGA", 1,AIC3256_OUT_PWR_CTRL, 0, 0, NULL, 0),

	/* dapm widget for Left Input Mixer*/
	SND_SOC_DAPM_MIXER("Left Input Mixer", SND_SOC_NOPM, 0, 0,
			&left_input_mixer_controls[0],
			ARRAY_SIZE(left_input_mixer_controls)),

	/* dapm widget for Right Input Mixer*/
	SND_SOC_DAPM_MIXER("Right Input Mixer", SND_SOC_NOPM, 0, 0,
			&right_input_mixer_controls[0],
			ARRAY_SIZE(right_input_mixer_controls)),
	/* dapm widget (platform domain) name for HPLOUT */
	SND_SOC_DAPM_OUTPUT("HPL"),

	/* dapm widget (platform domain) name for HPROUT */
	SND_SOC_DAPM_OUTPUT("HPR"),

	/* dapm widget (platform domain) name for LOLOUT */
	SND_SOC_DAPM_OUTPUT("LOL"),

	/* dapm widget (platform domain) name for LOROUT */
	SND_SOC_DAPM_OUTPUT("LOR"),

	/* dapm widget (platform domain) name for LINE1L */
	SND_SOC_DAPM_INPUT("IN1_L"),

	/* dapm widget (platform domain) name for LINE1R */
	SND_SOC_DAPM_INPUT("IN1_R"),

	/* dapm widget (platform domain) name for LINE2L */
	SND_SOC_DAPM_INPUT("IN2_L"),

	/* dapm widget (platform domain) name for LINE2R */
	SND_SOC_DAPM_INPUT("IN2_R"),

	/* dapm widget (platform domain) name for LINE3L */
	SND_SOC_DAPM_INPUT("IN3_L"),

	/* dapm widget (platform domain) name for LINE3R */
	SND_SOC_DAPM_INPUT("IN3_R"),

	/* DAPM widget for MICBIAS power control */
	SND_SOC_DAPM_MICBIAS("Mic Bias", AIC3256_MICBIAS_CTRL, 6, 0),

	/* Left DMIC Input Widget */
	SND_SOC_DAPM_INPUT("Left DMIC"),
	/* Right DMIC Input Widget */
	SND_SOC_DAPM_INPUT("Right DMIC"),

	/* Left Channel ADC Route */
	SND_SOC_DAPM_MUX("Left ADC Route", SND_SOC_NOPM, 0, 0, &adcl_mux),
	/* Right Channel ADC Route */
	SND_SOC_DAPM_MUX("Right ADC Route", SND_SOC_NOPM, 0, 0, &adcr_mux),

	/* Supply widget for PLL */
	SND_SOC_DAPM_SUPPLY_S("PLLCLK", 0,  AIC3256_CLK_REG_2, 7, 0,
			pll_power_on_event,
			SND_SOC_DAPM_POST_PMU),

	/* Supply widget for CODEC_CLK_IN */
	SND_SOC_DAPM_SUPPLY_S("CODEC_CLK_IN", 1,SND_SOC_NOPM, 0, 0, NULL, 0),
	/* Supply widget for NDAC divider */
	SND_SOC_DAPM_SUPPLY_S("NDAC_DIV", 2,AIC3256_NDAC_CLK_REG_6, 7, 0, NULL, 0),
	/* Supply widget for MDAC divider */
	SND_SOC_DAPM_SUPPLY_S("MDAC_DIV", 3,AIC3256_MDAC_CLK_REG_7, 7, 0, NULL, 0),
	/* Supply widget for NADC divider */
	SND_SOC_DAPM_SUPPLY_S("NADC_DIV", 2,AIC3256_NADC_CLK_REG_8, 7, 0, NULL, 0),
	/* Supply widget for MADC divider */
	SND_SOC_DAPM_SUPPLY_S("MADC_DIV", 3,AIC3256_MADC_CLK_REG_9, 7, 0, NULL, 0),
	/* Supply widget for Bit Clock divider */
	//SND_SOC_DAPM_SUPPLY("BCLK_N_DIV", AIC3256_CLK_REG_11, 7, 0, NULL, 0),
};

static const  struct snd_soc_dapm_route aic325x_dapm_routes[] = {

	/* PLL routing */
	{"CODEC_CLK_IN", NULL, "PLLCLK"},
	{"NDAC_DIV", NULL, "CODEC_CLK_IN"},
	{"NADC_DIV", NULL, "CODEC_CLK_IN"},
	{"MDAC_DIV", NULL, "NDAC_DIV"},
	{"MADC_DIV", NULL, "NADC_DIV"},
	//	{"BCLK_N_DIV", NULL, "MADC_DIV"},
	//	{"BCLK_N_DIV", NULL, "MDAC_DIV"},

	/* Clock routing for ADC */
	{"Left ADC", NULL, "MADC_DIV"},
	{"Right ADC", NULL, "MADC_DIV"},

	/* Clock routing for DAC */
	{"Left DAC", NULL, "MDAC_DIV" },
	{"Right DAC", NULL, "MDAC_DIV"},

	/* ASI routing */
	{"ASILIN", NULL, "ASIIN"},
	{"ASIRIN", NULL, "ASIIN"},
	{"ASIMonoMixIN", NULL, "ASIIN"},

	{"ASIIn Left Route","ASI Left In","ASILIN"},
	{"ASIIn Left Route","ASI Right In","ASIRIN"},
	{"ASIIn Left Route","ASI MonoMix In","ASIMonoMixIN"},

	{"ASIIn Right Route","ASI Left In","ASILIN"},
	{"ASIIn Right Route","ASI Right In","ASIRIN"},
	{"ASIIn Right Route","ASI MonoMix In","ASIMonoMixIN"},

	{"ASI IN Port", NULL, "ASIIn Left Route"},
	{"ASI IN Port", NULL, "ASIIn Right Route"},

	{"Left DAC", NULL, "ASI IN Port"},
	{"Right DAC", NULL, "ASI IN Port"},

	/* Left Headphone Output */
	{"HPL Output Mixer", "L_DAC switch", "Left DAC"},
	{"HPL Output Mixer", "IN1_L switch", "IN1_L"},
	{"HPL Output Mixer", "MAL switch", "MAL PGA"},
	{"HPL Output Mixer", "MAR switch", "MAR PGA"},

	/* Right Headphone Output */
	{"HPR Output Mixer", "R_DAC switch", "Right DAC"},
	{"HPR Output Mixer", "IN1_R switch", "IN1_R"},
	{"HPR Output Mixer", "MAR switch", "MAR PGA"},
	{"HPR Output Mixer", "L_DAC switch", "Left DAC"},

	/* HP output mixer to HP PGA */
	{"HPL PGA", NULL, "HPL Output Mixer"},
	{"HPR PGA", NULL, "HPR Output Mixer"},

	/* HP PGA to HP output pins */
	{"HPL", NULL, "HPL PGA"},
	{"HPR", NULL, "HPR PGA"},

	/* Charge pump to HP PGA */
	{"HPL PGA", NULL, "Charge Pump"},
	{"HPR PGA", NULL, "Charge Pump"},

	/* Left Line-out Output */
	{"LOL Output Mixer", "L_DAC switch", "Left DAC"},
	{"LOL Output Mixer", "MAL switch", "MAL PGA"},
	{"LOL Output Mixer", "R_DAC switch", "Right DAC"},
	{"LOL Output Mixer", "LOR switch", "LOR PGA"},

	/* Right Line-out Output */
	{"LOR Output Mixer", "R_DAC switch", "Right DAC"},
	{"LOR Output Mixer", "MAR switch", "MAR PGA"},

	{"LOL PGA", NULL, "LOL Output Mixer"},
	{"LOR PGA", NULL, "LOR Output Mixer"},

	{"LOL", NULL, "LOL PGA"},
	{"LOR", NULL, "LOR PGA"},

	/* ADC portions */
	/* Left Positive PGA input */
	{"Left Input Mixer", "IN1_L switch", "IN1_L"},
	{"Left Input Mixer", "IN2_L switch", "IN2_L"},
	{"Left Input Mixer", "IN3_L switch", "IN3_L"},
	{"Left Input Mixer", "IN1_R switch", "IN1_R"},
	/* Left Negative PGA input */
	{"Left Input Mixer", "IN2_R switch", "IN2_R"},
	{"Left Input Mixer", "IN3_R switch", "IN3_R"},
	{"Left Input Mixer", "CM1L switch", "CM1L"},
	{"Left Input Mixer", "CM2L switch", "CM2L"},
	/* Right Positive PGA Input */
	{"Right Input Mixer", "IN1_R switch", "IN1_R"},
	{"Right Input Mixer", "IN2_R switch", "IN2_R"},
	{"Right Input Mixer", "IN3_R switch", "IN3_R"},
	{"Right Input Mixer", "IN2_L switch", "IN2_L"},
	/* Right Negative PGA Input */
	{"Right Input Mixer", "IN1_L switch", "IN1_L"},
	{"Right Input Mixer", "IN3_L switch", "IN3_L"},
	{"Right Input Mixer", "CM1R switch", "CM1R"},
	{"Right Input Mixer", "CM2R switch", "CM2R"},

	{"CM1L", NULL, "CM"},
	{"CM2L", NULL, "CM"},
	{"CM1R", NULL, "CM"},
	{"CM1R", NULL, "CM"},

	/* Left MicPGA */
	{"Left MicPGA", NULL, "Left Input Mixer"},

	/* Right MicPGA */
	{"Right MicPGA", NULL, "Right Input Mixer"},

	{"Left ADC", NULL, "Left MicPGA"},
	{"Right ADC", NULL, "Right MicPGA"},

	{"Left ADC Route", "Analog", "Left MicPGA"},
	{"Left ADC Route", "Digital", "Left DMIC"},

	/* Selection of Digital/Analog Mic */
	{"Right ADC Route", "Analog", "Right MicPGA"},
	{"Right ADC Route", "Digital", "Right DMIC"},

	{"Left ADC", NULL, "Left ADC Route"},
	{"Right ADC", NULL, "Right ADC Route"},

	{"MAL PGA", NULL, "Left MicPGA"},
	{"MAR PGA", NULL, "Right MicPGA"},
};




/* aic3256_firmware_load: This function is called by the
 *		request_firmware_nowait function as soon
 *		as the firmware has been loaded from the file.
 *		The firmware structure contains the data and$
 *		the size of the firmware loaded.
 * @fw: pointer to firmware file to be dowloaded
 * @context: pointer variable to codec
 *
 * Returns 0 for success.
 */
void aic3256_firmware_load(const struct firmware *fw, void *context)
{
	struct snd_soc_codec *codec = context;
	struct aic325x_priv *private_ds = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

//	aic3xxx_cfw_lock(private_ds->cfw_p, 1); /* take the lock */
	if (private_ds->cur_fw != NULL)
		release_firmware(private_ds->cur_fw);
	private_ds->cur_fw = NULL;

	if (fw != NULL)	{
		dev_dbg(codec->dev, "Firmware binary load\n");
		private_ds->cur_fw = (void *)fw;
//		ret = aic3xxx_cfw_reload(private_ds->cfw_p, (void *)fw->data,
//			fw->size);
//		if (ret < 0) { /* reload failed */
//			dev_err(codec->dev, "Firmware binary load failed\n");
//			release_firmware(private_ds->cur_fw);
//			private_ds->cur_fw = NULL;
//			fw = NULL;
//		}
	} else {
		dev_err(codec->dev, "Codec Firmware failed\n");
		ret = -1;
	}
	//aic3xxx_cfw_lock(private_ds->cfw_p, 0); /* release the lock */
	if (ret >= 0) {
		/* init function for transition */
	//	aic3xxx_cfw_transition(private_ds->cfw_p, "INIT");
	//	aic3xxx_cfw_add_modes(codec, private_ds->cfw_p);
	//	aic3xxx_cfw_add_controls(codec, private_ds->cfw_p);
	//	aic3xxx_cfw_setmode_cfg(private_ds->cfw_p, 0, 0);
	}
}

/*
 * Event for headphone amplifier power changes.Special
 * power up/down sequences are required in order to maximise pop/click
 * performance.
 */
static int aic325x_hp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{

	struct snd_soc_codec *codec = w->codec;
	//struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(codec);
	int ret_wbits=0;
	unsigned int reg_mask = 0;
#if 1
	if(w->shift == 5)// HPL
	{
		reg_mask = (1 << 5);

	}
	if(w->shift == 4)// HPR
	{
		reg_mask = (1 << 1);
	}
#endif
	// Wait for HP power on 
	if (event & SND_SOC_DAPM_POST_PMU) {

		ret_wbits = aic3xxx_wait_bits(codec->control_data, AIC3256_LDO_CTRL,
			HP_DRIVER_BUSY_MASK, HP_DRIVER_BUSY_MASK,AIC3256_TIME_DELAY,AIC3256_DELAY_COUNTER);
		//		ret_wbits = aic3xxx_wait_bits(codec->control_data, AIC3256_DAC_FLAG,
		//				reg_mask, reg_mask,AIC3256_TIME_DELAY,AIC3256_DELAY_COUNTER);
		if(!ret_wbits)
			dev_err(codec->dev,"HP %s power-up timedout\n", (w->shift == 5) ? "Left":"Right");
	}
	else if (event & SND_SOC_DAPM_POST_PMD) {
		//		ret_wbits = aic3xxx_wait_bits(codec->control_data, AIC3256_PWR_CTRL_REG,
		//				HP_DRIVER_BUSY_MASK, 0x0,AIC3256_TIME_DELAY,AIC3256_DELAY_COUNTER);
		ret_wbits = aic3xxx_wait_bits(codec->control_data, AIC3256_DAC_FLAG,
			reg_mask, 0x0,AIC3256_TIME_DELAY,AIC3256_DELAY_COUNTER);
		if(!ret_wbits)
			dev_err(codec->dev,"HP %s power- down timedout \n", (w->shift == 5) ? "Left": "Right");

	}
	return 0;
}

static int aic3256_cp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	printk("cp _event\n");
	if (event & SND_SOC_DAPM_POST_PMU) {
		msleep(20);
		printk("cp _event post pmu\n");
		snd_soc_write(codec, AIC3256_HP_DRIVER_CONF_REG, 0x13);	
	}
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic325x_hw_write
 * Purpose  : i2c write function
 *----------------------------------------------------------------------------
 */
int aic325x_hw_write(struct snd_soc_codec *codec, const char *buf,
						unsigned int count)
{
	u8 data[3];
	int ret;

	data[0] = *buf;
	data[1] = *(buf+1);
	data[2] = *(buf+2);

	ret = i2c_master_send(codec->control_data, data, count);

	if (ret < count) {
		printk(KERN_ERR "#%s: I2C write Error: bytes written = %d\n\n",
				__func__, ret);
		return -EIO;
	}
	return ret;
}

/*
 *----------------------------------------------------------------------------
 * Function : aic325x_hw_read
 * Purpose  : i2c read function
 *----------------------------------------------------------------------------
 */
int aic325x_hw_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct i2c_client *client = codec->control_data;
	int data = 0;

	if (i2c_master_send(client, (char *)&reg, 1) < 0) {
		printk(KERN_ERR "%s: I2C write Error\n", __func__);
		return -EIO;
	}

	if (i2c_master_recv(client, (char *)&data, 1) < 0) {
		printk(KERN_ERR "%s: I2C read Error\n", __func__);
		return -EIO;
	}

	return data & 0x00FF;
}


/**
 * Methods for CFW Operations
 *
 * Due to incompatibilites between structures used by MFD and CFW
 * we need to transform the register format before linking to
 * CFW operations.
 */
static inline unsigned int aic3256_ops_cfw2reg(unsigned int reg)
{
	//union cfw_register *c = (union cfw_register *) &reg;
	union aic3xxx_reg_union mreg;

	//mreg.aic3xxx_register.offset = c->offset;
	//mreg.aic3xxx_register.page = c->page;
	//mreg.aic3xxx_register.reserved = 0;

	return mreg.aic3xxx_register_int;
}

static int aic3256_ops_reg_read(struct snd_soc_codec *codec, unsigned int reg)
{
	return 0;/* aic3xxx_reg_read(codec->control_data, aic3256_ops_cfw2reg(reg));*/
}

static int aic3256_ops_reg_write(struct snd_soc_codec *codec, unsigned int reg,
					unsigned char val)
{
	return 0;/*aic3xxx_reg_write(codec->control_data,
					aic3256_ops_cfw2reg(reg), val);*/
}

static int aic3256_ops_set_bits(struct snd_soc_codec *codec, unsigned int reg,
					unsigned char mask, unsigned char val)
{
	return 0;/*aic3xxx_set_bits(codec->control_data,
				aic3256_ops_cfw2reg(reg), mask, val);*/

}

static int aic3256_ops_bulk_read(struct snd_soc_codec *codec, unsigned int reg,
					int count, u8 *buf)
{
	return 0;/*aic3xxx_bulk_read(codec->control_data,
					aic3256_ops_cfw2reg(reg), count, buf);*/
}

static int aic3256_ops_bulk_write(struct snd_soc_codec *codec, unsigned int reg,
					int count, const u8 *buf)
{
	return 0;/*aic3xxx_bulk_write(codec->control_data,
					aic3256_ops_cfw2reg(reg), count, buf);*/
}


/*
********************************************************************************
Function Name : aic3256_ops_dlock_lock
Argument      : pointer argument to the codec
Return value  : Integer
Purpose	      : To Read the run state of the DAC and ADC
by reading the codec and returning the run state

Run state Bit format

------------------------------------------------------
D31|..........| D7 | D6|  D5  |  D4  | D3 | D2 | D1  |   D0  |
R               R    R   LADC   RADC    R    R   LDAC   RDAC
------------------------------------------------------

********************************************************************************
*/
int aic3256_ops_lock(struct snd_soc_codec *codec)
{
#if 0
	mutex_lock(&codec->mutex);
	/* Reading the run state of adc and dac */
	return aic3256_get_runstate(codec);
#else
    return 0;
#endif
}

/*
*******************************************************************************
Function name	: aic3256_ops_dlock_unlock
Argument	: pointer argument to the codec
Return Value	: integer returning 0
Purpose		: To unlock the mutex acqiured for reading
run state of the codec
********************************************************************************
*/
int aic3256_ops_unlock(struct snd_soc_codec *codec)
{
#if 0
	/*Releasing the lock of mutex */
	mutex_unlock(&codec->mutex);
#endif
	return 0;
}
/*
*******************************************************************************
Function Name	: aic3256_ops_dlock_stop
Argument	: pointer Argument to the codec
mask tells us the bit format of the
codec running state

Bit Format:
------------------------------------------------------
D31|..........| D7 | D6| D5 | D4 | D3 | D2 | D1 | D0 |
R               R    R   AL   AR    R    R   DL   DR
------------------------------------------------------
R  - Reserved
A  - minidsp_A
D  - minidsp_D
********************************************************************************
*/
int aic3256_ops_stop(struct snd_soc_codec *codec, int mask)
{
#if 0
	int run_state = 0;

	run_state = aic3256_get_runstate(codec);
	if (mask & AIC3XXX_COPS_MDSP_A) /* power-down ADCs */
		aic3xxx_set_bits(codec->control_data,
					AIC3256_ADC_DATAPATH_SETUP, 0xC0, 0);

	if (mask & AIC3XXX_COPS_MDSP_D) /* power-down DACs */
		aic3xxx_set_bits(codec->control_data,
					AIC3256_DAC_DATAPATH_SETUP, 0xC0, 0);

	if ((mask & AIC3XXX_COPS_MDSP_A) &&
		!aic3xxx_wait_bits(codec->control_data, AIC3256_ADC_FLAG,
					AIC3256_ADC_POWER_MASK,
					0, AIC3256_TIME_DELAY,
					AIC3256_DELAY_COUNTER))
		goto err;

	if ((mask & AIC3XXX_COPS_MDSP_D) &&
		 !aic3xxx_wait_bits(codec->control_data, AIC3256_DAC_FLAG,
					AIC3256_DAC_POWER_MASK,	0,
					AIC3256_TIME_DELAY,
					AIC3256_DELAY_COUNTER))
			goto err;
	return run_state;

err:
	dev_err(codec->dev, "Unable to turn off ADCs or DACs at [%s:%d]",
				__FILE__, __LINE__);
	return -EINVAL;
#else
   return 0;
#endif	

}

/*
****************************************************************************
Function name	: aic3256_ops_dlock_restore
Argument	: pointer argument to the codec, run_state
Return Value	: integer returning 0
Purpose		: To unlock the mutex acqiured for reading
run state of the codec and to restore the states of the dsp
******************************************************************************
*/
static int aic3256_ops_restore(struct snd_soc_codec *codec, int run_state)
{
#if 0
	int sync_state;

	/*	This is for read the sync mode register state */
	sync_state = aic3xxx_reg_read(codec->control_data, AIC3256_DAC_PRB);
	/* checking whether the sync mode has been set -
		- or not and checking the current state */
	if (((run_state & 0x30) && (run_state & 0x03)) && (sync_state & 0x80))
		aic3256_restart_dsps_sync(codec, run_state);
	else
		aic3256_dsp_pwrup(codec, run_state);

#endif
	return 0;
}
/**
 * aic3256_ops_adaptivebuffer_swap: To swap the coefficient buffers
 *                               of minidsp according to mask
 * @pv: pointer argument to the codec,
 * @mask: tells us which dsp has to be chosen for swapping
 *
 * Return Value    : returning 0 on success
 */
int aic3256_ops_adaptivebuffer_swap(struct snd_soc_codec *codec, int mask)
{
#if 0
	const int sbuf[][2] = {
		{ /*AIC3XXX_ABUF_MDSP_A,*/ AIC3256_ADC_ADAPTIVE_CRAM_REG },
		{ /*AIC3XXX_ABUF_MDSP_D1,*/ AIC3256_DAC_ADAPTIVE_CRAM_REG},
		/* { AIC3XXX_ABUF_MDSP_D2, AIC3256_DAC_ADAPTIVE_BANK2_REG }, */
	};
	int i;

	for (i = 0; i < sizeof(sbuf)/sizeof(sbuf[0]); ++i) {
		if (!(mask & sbuf[i][0]))
			continue;
		aic3xxx_set_bits(codec->control_data, sbuf[i][1], 0x1, 0x1);
		if (!aic3xxx_wait_bits(codec->control_data,
			sbuf[i][1], 0x1, 0, 15, 1))
			goto err;
	}
	return 0;
err:
	dev_err(codec->dev, "miniDSP buffer swap failure at [%s:%d]",
				__FILE__, __LINE__);
	return -EINVAL;
#else
      return 0;
#endif
}

/*****************************************************************************
Function name	: aic3256_get_runstate
Argument	: pointer argument to the codec
Return Value	: integer returning the runstate
Purpose		: To read the current state of the dac's and adc's
 ******************************************************************************/

static int aic3256_get_runstate(struct snd_soc_codec *codec)
{
	unsigned int dac, adc;
	/* Read the run state */
	dac = aic3xxx_reg_read(codec->control_data, AIC3256_DAC_FLAG);
	adc = aic3xxx_reg_read(codec->control_data, AIC3256_ADC_FLAG);

	return (((adc>>6)&1)<<5)  |
		(((adc>>2)&1)<<4) |
		(((dac>>7)&1)<<1) |
		(((dac>>3)&1)<<0);
}

/*****************************************************************************
Function name	: aic3256_dsp_pwrdwn_status
Argument	: pointer argument to the codec , cur_state of dac's and adc's
Return Value	: integer returning 0
Purpose		: To read the status of dsp's
 ******************************************************************************/

int aic3256_dsp_pwrdwn_status(
		struct snd_soc_codec *codec /* ptr to the priv data structure */
		)
{

#if 0
	aic3xxx_set_bits(codec->control_data, AIC3256_ADC_DATAPATH_SETUP,
				0XC0, 0);
	aic3xxx_set_bits(codec->control_data, AIC3256_DAC_DATAPATH_SETUP,
				0XC0, 0);

	if (!aic3xxx_wait_bits(codec->control_data, AIC3256_ADC_FLAG,
			AIC3256_ADC_POWER_MASK, 0, AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER))
		goto err;

	if (!aic3xxx_wait_bits(codec->control_data, AIC3256_DAC_FLAG,
			AIC3256_DAC_POWER_MASK, 0, AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER))
		goto err;

	return 0;

err:
	dev_err(codec->dev, "DAC/ADC Power down timedout at [%s:%d]",
				__FILE__, __LINE__);
	return -EINVAL;

#else
      return 0;
#endif
}

static int aic3256_dsp_pwrup(struct snd_soc_codec *codec, int state)
{
#if 0
	int adc_reg_mask = 0;
	int adc_power_mask = 0;
	int dac_reg_mask = 0;
	int dac_power_mask = 0;
	int ret_wbits;

	//#if 0
	//	if (state & AIC3XXX_COPS_MDSP_A_L) {
	//		adc_reg_mask	|= 0x80;
	//		adc_power_mask	|= AIC3256_LADC_POWER_MASK;
	//	}
	//	if (state & AIC3XXX_COPS_MDSP_A_R) {
	//		adc_reg_mask	|= 0x40;
	//		adc_power_mask	|= AIC3256_RADC_POWER_MASK;
	//	}
	//#endif
	if (state & AIC3XXX_COPS_MDSP_A)
		aic3xxx_set_bits(codec->control_data,
		AIC3256_ADC_DATAPATH_SETUP,
		0XC0, adc_reg_mask);

	//#if 0
	//	if (state & AIC3XXX_COPS_MDSP_D_L) {
	//		dac_reg_mask	|= 0x80;
	//		dac_power_mask	|= AIC3256_LDAC_POWER_MASK;
	//	}
	//
	//	if (state & AIC3XXX_COPS_MDSP_D_R) {
	//		dac_reg_mask	|= 0x40;
	//		dac_power_mask	|= AIC3256_RDAC_POWER_MASK;
	//	}
	//#endif

	if (state & AIC3XXX_COPS_MDSP_D)
		aic3xxx_set_bits(codec->control_data,
		AIC3256_DAC_DATAPATH_SETUP,
		0XC0, dac_reg_mask);

	if (state & AIC3XXX_COPS_MDSP_A) {
		ret_wbits = aic3xxx_wait_bits(codec->control_data,
			AIC3256_ADC_FLAG, AIC3256_ADC_POWER_MASK,
			adc_power_mask, AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER);
		if (!ret_wbits)
			dev_err(codec->dev, "ADC Power down timedout\n");
	}

	if (state & AIC3XXX_COPS_MDSP_D) {
		ret_wbits = aic3xxx_wait_bits(codec->control_data,
			AIC3256_DAC_FLAG, AIC3256_DAC_POWER_MASK,
			dac_power_mask, AIC3256_TIME_DELAY,
			AIC3256_DELAY_COUNTER);
		if (!ret_wbits)
			dev_err(codec->dev, "ADC Power down timedout\n");
	}
#endif
	return 0;
}

static int aic3256_restart_dsps_sync(struct snd_soc_codec *codec, int run_state)
{

#if 0
	aic3256_dsp_pwrdwn_status(codec);
	aic3256_dsp_pwrup(codec, run_state);
#endif	
	return 0;
}

//static const struct aic3xxx_codec_ops aic3256_cfw_codec_ops = {
//	.reg_read	=	aic3256_ops_reg_read,
//	.reg_write	=	aic3256_ops_reg_write,
//	.set_bits	=	aic3256_ops_set_bits,
//	.bulk_read	=	aic3256_ops_bulk_read,
//	.bulk_write	=	aic3256_ops_bulk_write,
//	.lock		=	aic3256_ops_lock,
//	.unlock		=	aic3256_ops_unlock,
//	.stop		=	aic3256_ops_stop,
//	.restore	=	aic3256_ops_restore,
//	.bswap		=	aic3256_ops_adaptivebuffer_swap,
//};

/**
 * aic325x_codec_read: provide read api to read aic3256 registe space
 * @codec: pointer variable to codec having codec information,
 * @reg: register address,
 *
 * Return: Return value will be value read.
 */
unsigned int aic325x_codec_read(struct snd_soc_codec *codec, unsigned int reg)
{
	u8 value;

	union aic3xxx_reg_union *aic_reg = (union aic3xxx_reg_union *)&reg;
	value = aic3xxx_reg_read(codec->control_data, reg);
	dev_dbg(codec->dev, "p %d, r 30 %x %x\n",
		aic_reg->aic3xxx_register.page,
		aic_reg->aic3xxx_register.offset, value);
	return value;
}

/**
 * aic325x_codec_write: provide write api to write at aic3256 registe space
 * @codec: Pointer variable to codec having codec information,
 * @reg: Register address,
 * @value: Value to be written to address space
 *
 * Return: Total no of byte written to address space.
 */
int aic325x_codec_write(struct snd_soc_codec *codec, unsigned int reg,
				unsigned int value)
{
	union aic3xxx_reg_union *aic_reg = (union aic3xxx_reg_union *)&reg;
	dev_dbg(codec->dev, "p %d, w 30 %x %x\n",
		aic_reg->aic3xxx_register.page,
		aic_reg->aic3xxx_register.offset, value);
	return aic3xxx_reg_write(codec->control_data, reg, value);
}

/**
 * aic325x_hw_params: This function is to set the hardware parameters
 *		for AIC3256.
 *		The functions set the sample rate and audio serial data word
 *		length.
 * @substream: pointer variable to sn_pcm_substream,
 * @params: pointer to snd_pcm_hw_params structure,
 * @dai: ponter to dai Holds runtime data for a DAI,
 *
 * Return: Return 0 on success.
 */
static int aic325x_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
					struct snd_soc_dai *dai)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
//	printk("[%s]\n",__func__);
	u8 data = 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		data |= (0x00);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		data |= (0x10);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		data |= (0x20);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data |= (0x30);
		break;
	}

	snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_1,
				INTERFACE_REG1_DATA_LEN_MASK, data);

//	aic3xxx_set_power_play(codec, 1);

	return 0;
}

static int aic325x_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
//	printk("%s %s %d,substream->stream=%d   \n",__FILE__,__func__,__LINE__,substream->stream);

	aic325x_initalize(codec);

//	aic3xxx_set_power_play(codec, 1);

//	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
//	{
//		//snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x3F), 0xc0, 0xc0);
//		aic3xxx_set_power_play(codec, 1);
//	}
//	else
//	{
//		aic3xxx_set_power_rec(codec, 1);
//	}
	return 0;
}

void an_mute_work( struct work_struct *data ) {
	struct snd_soc_codec *codec = aic325x_codec;

	//snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   9), 0x30, 0x00);
	//snd_soc_write(codec, AIC3256_DAC_MUTE_CTRL_REG, 0x1C);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x10), 0x40);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x11), 0x40);
}
DECLARE_DELAYED_WORK(analog_mute_w, an_mute_work);

void an_unmute_work( struct work_struct *data ) {
	struct snd_soc_codec *codec = aic325x_codec;

	//snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   9), 0x30, 0x30);

	//snd_soc_write(codec, AIC3256_DAC_MUTE_CTRL_REG, 0x10);

	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x10), 0x0);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x11), 0x0);

}
DECLARE_DELAYED_WORK(analog_unmute_w, an_unmute_work);

static void aic325x_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
//	printk("%s %s %d,substream->stream=%d   \n",__FILE__,__func__,__LINE__,substream->stream);

	aic3xxx_hp_en(codec,0);

	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		//snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x3F), 0xc0, 0x00);
		//aic3xxx_set_power_play(codec, 0);
		//msleep(10);
	}
	else
	{
		aic3xxx_set_power_rec(codec, 0);
	}
	//#endif
	return;
}

static int aic325x_prepare(struct snd_pcm_substream *ss, struct snd_soc_dai *dai) {
//	printk("[%s]\n",__func__);

	struct snd_soc_codec *codec = dai->codec;
	aic3xxx_hp_en(codec, 1);
}

static int aic325x_trigger(struct snd_pcm_substream *ss, int t, struct snd_soc_dai *dai) {
	struct snd_soc_codec *codec = dai->codec;

//	printk("[%s] t=%i\n",__func__,t);
	if (t == 1)
		schedule_delayed_work(&analog_unmute_w, HZ/10);
	else
		schedule_delayed_work(&analog_mute_w, 0);

}

/**
 * aic325x_mute: This function is to mute or unmute the left and right DAC
 * @dai: ponter to dai Holds runtime data for a DAI,
 * @mute: integer value one if we using mute else unmute,
 *
 * Return: return 0 on success.
 */
static int aic325x_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
//	printk("%s %s %d mute=%d\n",__FILE__,__func__,__LINE__,mute);

	//	int i;
	//#if 0	
	//	if (mute)
	//		snd_soc_write(codec, AIC3256_DAC_MUTE_CTRL_REG, 0x0C);
	//	else
	//		snd_soc_write(codec, AIC3256_DAC_MUTE_CTRL_REG, 0x00);
	//
	//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 52), MIC_INT_RVAL);
	//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 54), MIC_INT_RVAL);
	//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 55), MIC_INT_RVAL);
	//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 57), MIC_INT_RVAL);
	//#else
	if(mute)
	{
		int t = 0;
		t = work_busy(&((&analog_mute_w)->work));
		if (t) {
//			printk("[%s] wb=%i\n",__func__,t);
			flush_delayed_work_sync(&analog_mute_w);
			msleep(50);
		}

		snd_soc_write(codec, AIC3256_DAC_MUTE_CTRL_REG, 0x1C);

		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  63), 0x14);

//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x10), 0x40);
//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x11), 0x40);
//		mdelay(50);

	}else{
		//aic325x_wait(codec, AIC3XXX_MAKE_REG(0, 1, 63), 0xf0, 0xf0);

		//DAC enable
		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 0x3F), 0xD4);

//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x10), 0x0);
//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 0x11), 0x0);

		snd_soc_write(codec, AIC3256_DAC_MUTE_CTRL_REG, 0x10);

		//if (0 == cur_headset_type)
		//	gpio_direction_output(AIC_SPK_PIN, GPIO_HIGH);
		//aic3xxx_spk_en(codec, 1);
	}

	//#endif
	//#if 0
	//	for(i=0; i<0x80; i++)
	//	{
	//		printk("0x%02X: 0x%02X\n", i, snd_soc_read(codec, i));
	//	}
	//	for(i=0; i<0x80; i++)
	//	{
	//		printk("0x%02X: 0x%02X\n", 0x80|i, snd_soc_read(codec, i+256));
	//	}
	//#endif
	return 0;
}

/**
 * aic325x_set_dai_sysclk: This function is to set the DAI sysclk
 * @codec_dai: ponter to dai Holds runtime data for a DAI,
 * @freq: asi sysclk info,
 *
 * return: return 0 on success.
 */
static int aic325x_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
			     unsigned int freq, int dir)
{
	return 0;
}

/**
 * aic325x_set_dai_fmt: This function is to set the DAI format
 * @codec_dai: ponter to dai Holds runtime data for a DAI,
 * @fmt: asi format info,
 *
 * return: return 0 on success.
 */
static int aic325x_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec;
	struct aic325x_priv *aic325x;
	u8 iface_reg1 = 0;
	u8 iface_reg3 = 0;
	u8 dsp_a_val = 0;

	codec	= codec_dai->codec;
	aic325x	= snd_soc_codec_get_drvdata(codec);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aic325x->master = 1;
		iface_reg1 |= AIC3256_BIT_CLK_MASTER | AIC3256_WORD_CLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		aic325x->master = 0;
		iface_reg1 &= ~(AIC3256_BIT_CLK_MASTER |
					AIC3256_WORD_CLK_MASTER);
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		aic325x->master = 0;
		iface_reg1 |= AIC3256_BIT_CLK_MASTER;
		iface_reg1 &= ~(AIC3256_WORD_CLK_MASTER);
		break;
	default:
		printk(KERN_INFO "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
			dsp_a_val = 0x1;
		case SND_SOC_DAIFMT_DSP_B:
			switch(fmt & SND_SOC_DAIFMT_INV_MASK) {
				case SND_SOC_DAIFMT_NB_NF:
					break;
				case SND_SOC_DAIFMT_IB_NF:
					iface_reg3 |= BCLK_INV_MASK;
					break;
				default: 
					return -EINVAL;
			}
			iface_reg1 |= (AIC325x_DSP_MODE << CLK_REG_3_SHIFT);
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			iface_reg1 |= (AIC325x_RIGHT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			iface_reg1 |= (AIC325x_LEFT_JUSTIFIED_MODE << CLK_REG_3_SHIFT);
			break;
		default:
			printk(KERN_INFO "Invalid DAI interface format\n");
			return -EINVAL;
	}

	snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_1, 
			INTERFACE_REG1_DATA_TYPE_MASK |
			0, //INTERFACE_REG1_MASTER_MASK, // set master at init
			iface_reg1);
	snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_2, INTERFACE_REG2_MASK,
			dsp_a_val);
	snd_soc_update_bits(codec, AIC3256_INTERFACE_SET_REG_3, INTERFACE_REG3_MASK,
			iface_reg3);
	return 0;
}

/**
 * aic325x_set_dai_pll: This function is to Set pll for aic3256 codec dai
 * @dai: ponter to dai Holds runtime data for a DAI, $
 * @pll_id: integer pll_id
 * @fin: frequency in,
 * @fout: Frequency out,
 *
 * Return: return 0 on success
*/
static int aic325x_set_dai_pll(struct snd_soc_dai *dai, int pll_id, int source,
				unsigned int Fin, unsigned int Fout)
{
	struct snd_soc_codec *codec = dai->codec;
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	int ret;

//	aic3xxx_cfw_set_pll(aic3256->cfw_p, dai->id);
	/*P VAL */
	ret = snd_soc_read(codec,AIC3256_CLK_REG_2);
	pr_debug("p_val = %#x\n",ret);

	/* J_val */
	ret = snd_soc_read(codec,AIC3256_CLK_REG_3);
	pr_debug("j_val = %#x\n",ret);

	/* D_val msb */
	ret = snd_soc_read(codec,AIC3256_CLK_REG_4);
	pr_debug("d_val msb = %#x\n",ret);

	/* D_val lsb */
	ret = snd_soc_read(codec,AIC3256_CLK_REG_5);
	pr_debug("d_val lsb = %#x\n",ret);


	/* n_dac */
	ret = snd_soc_read(codec,AIC3256_NDAC_CLK_REG_6);
	pr_debug("n_dac = %#x\n",ret);

	/* m_dac */
	ret = snd_soc_read(codec,AIC3256_MDAC_CLK_REG_7);
	pr_debug("m_dac = %#x\n",ret);

	/* dosr */
	ret = snd_soc_read(codec,AIC3256_DAC_OSR_LSB);
	pr_debug("dosr = %#x\n",ret);
	return 0;
}

/**
 *
 * aic325x_set_bias_level: This function is to get triggered
 *			 when dapm events occurs.
 * @codec: pointer variable to codec having informaton related to codec,
 * @level: Bias level-> ON, PREPARE, STANDBY, OFF.
 *
 * Return: Return 0 on success.
 */
static int aic325x_set_bias_level(struct snd_soc_codec *codec,
					enum snd_soc_bias_level level)
{
	struct aic325x_priv *aic325x = snd_soc_codec_get_drvdata(codec);
	printk("%s %s %d level=%d  \n",__FILE__,__func__,__LINE__,level);
	return 0;
	switch (level) {

	/* full On */
	case SND_SOC_BIAS_ON:
		/* all power is driven by DAPM system */
		dev_dbg(codec->dev, "%s BIAS on\n", __func__);
		if (aic325x->master == 1) {
			snd_soc_update_bits(codec, AIC3256_CLK_REG_11, 
					BCLK_DIV_POWER_MASK, 0x80);	
		}
		aic3xxx_set_power_play(codec, 1);
		break;

	/* partial On */
	case SND_SOC_BIAS_PREPARE:
		dev_dbg(codec->dev, "%s BIAS prepare\n", __func__);
		if (codec->dapm.bias_level == SND_SOC_BIAS_ON) {
			aic3xxx_set_power_play(codec, 0);

			snd_soc_update_bits(codec, AIC3256_CLK_REG_11,
						BCLK_DIV_POWER_MASK, 0);
		}
		break;
	/* Off, with power */
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(codec->dev, "%s BIAS standby\n", __func__);
		//
		//snd_soc_update_bits(codec, AIC3256_REF_PWR_UP_CONF_REG,
		//		AIC3256_REF_PWR_UP_MASK,
		//		AIC3256_FORCED_REF_PWR_UP);
		///

		/*
		 * all power is driven by DAPM system,
		 * so output power is safe if bypass was set
		 */
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {

			snd_soc_update_bits(codec, AIC3256_POW_CFG,
				AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
				AIC3256_DISABLE_AVDD_TO_DVDD);

			//enable charge pump
			snd_soc_update_bits(codec, AIC3256_POW_CFG,	0x3, 0x2 );
			msleep(100);

			snd_soc_update_bits(codec, AIC3256_LDO_CTRL,
					AIC3256_ANALOG_BLOCK_POWER_CONTROL_MASK,
					AIC3256_ENABLE_ANALOG_BLOCK);
			msleep(50);

		}
	//	schedule_delayed_work(&pdet_work, msecs_to_jiffies(100));
		break;
	/* Off, without power */
	case SND_SOC_BIAS_OFF:
		/* force all power off */
		aic3xxx_spk_en(codec, 0);

		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
			/*
			snd_soc_update_bits(codec, AIC3256_REF_PWR_UP_CONF_REG,
					AIC3256_REF_PWR_UP_MASK,
					AIC3256_AUTO_REF_PWR_UP);
			*/

			snd_soc_update_bits(codec, AIC3256_LDO_CTRL,
					AIC3256_ANALOG_BLOCK_POWER_CONTROL_MASK,
					AIC3256_DISABLE_ANALOG_BLOCK);
			msleep(100);

			//disable charge pump
			snd_soc_update_bits(codec, AIC3256_POW_CFG,	0x3, 0x0 );
			msleep(20);

#ifndef NO_PD
			snd_soc_update_bits(codec, AIC3256_POW_CFG,
				AIC3256_AVDD_CONNECTED_TO_DVDD_MASK,
				AIC3256_ENABLE_AVDD_TO_DVDD);
#endif

		}
	/* force all power off */
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

#if defined(CONFIG_PB650_DEEP_STANDY)
extern int iDeepStandy;
#endif

/**
 *
 * aic325x_suspend; This function is to suspend the AIC3256 driver.
 * @codec: pointer variable to codec having informaton related to codec,
 *
 * Return: Return 0 on success.
 */
static int aic325x_suspend(struct snd_soc_codec *codec,
			pm_message_t state)
{
	//printk("%s: %d\n", __func__, state.event);
	//return 0;
	//aic3xxx_set_power_play(codec, 0);

	aic325x_poweroff(codec);
	return 0;

#if defined(CONFIG_PB650_DEEP_STANDY)
	if (iDeepStandy == 0) {
		aic325x_poweroff(codec);
		return 0;
	} 
#endif
	aic3xxx_set_power_rec(codec, 0);
#ifndef NO_PD
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 1, 9), 0x30, 0x00);
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 63), 0x15);
	msleep(20);
#endif
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x3F), 0xc0, 0x00);
	msleep(10);
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  12), 0x0);	// Route LDAC to HPL
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  13), 0x0);	// Route RDAC to HPR
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  14), 0x0);	// Route LDAC to LOL
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  15), 0x0);	// Route LDAC to LOR
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x0b), 0x80, 0x00);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x0c), 0x80, 0x00);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x13), 0x80, 0x00);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 5), 0x80, 0x00);
	msleep(10);
	//aic325x_set_bias_level(codec, SND_SOC_BIAS_OFF);
	//snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 1, 9), 0x0c, 0x00);
	//snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 1, 9), 0x0c, 0x00);
	//#if 0
	//	int i;
	//	for(i=0; i<0x80; i++)
	//	{
	//		printk("0x%02X: 0x%02X\n", i, snd_soc_read(codec, i));
	//	}
	//	for(i=0; i<0x80; i++)
	//	{
	//		printk("0x%02X: 0x%02X\n", 0x80|i, snd_soc_read(codec, i+256));
	//	}
	//#endif

	return 0;
}


/**
 * aic325x_resume: This function is to resume the AIC3256 driver
 *		 from off state to standby2013/9/28
 * @codec: pointer variable to codec having informaton related to codec,
 *
 * Return: Return 0 on success.
 */
static int aic325x_resume(struct snd_soc_codec *codec)
{
	return 0;

	//printk("%s\n", __func__);
#ifdef HEADSET_DETECTION	
	schedule_delayed_work(&pdet_work, msecs_to_jiffies(500));
#endif
//	msleep(200);
#if defined(CONFIG_PB650_DEEP_STANDY)
	if (iDeepStandy == 0)
		aic325x_initalize(codec);
#endif
	//aic325x_set_bias_level(codec, SND_SOC_BIAS_OFF /*SND_SOC_BIAS_ON*/);//SND_SOC_BIAS_STANDBY);
#ifndef NO_PD
	msleep(30);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 1, 9), 0x30, 0x30);
#endif
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  12), 0x08);	// Route LDAC to HPL
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  13), 0x08);	// Route RDAC to HPR
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  14), 0x08);	// Route LDAC to LOL
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  15), 0x08);	// Route LDAC to LOR
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 5), 0x80, 0x80);
	msleep(10);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x0c), 0x80, 0x80);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x13), 0x80, 0x80);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x0b), 0x80, 0x80);
	snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0, 0, 0x3F), 0xc0, 0xc0);

	return 0;
}

//#if 0
//#define DELAY_REG 45
//static const struct aic325x_configs aic325x_reg_init[] = {
//	{0x89, 0x00},
//	{DELAY_REG, 20},
//	{1,	1},
//	{DELAY_REG, 100},
//	{0x0b, 0x81}, // NDAC
//	{0x0c, 0x82}, // MDAC
//	{0x12, 0x81}, // NADC
//	{0x13, 0x82}, // MADC
//	//{0x1b, 0x00}, // I2S 16b
//	//{0x3c, 0x01}, // PRB
//	//{0x3d, 0x01}, // PRB
//
//	{0xec, 0x16}, // CP
//	{0xec, 0x06}, // CP
//	{0x81, 0x0a}, // CP on
//	{0x82, 0x00}, // power on
//	{DELAY_REG, 10},
//	{0xbd, 0x00}, // PTM
//	{0xc7, 0x32}, // analog input charging
//	{0xeb, 0x05}, // vref
//	{DELAY_REG, 50},
//
//	{0x8c, 0x08}, // hp mix
//	{0x8d, 0x08}, // hp mix
//	{0x8e, 0x08}, // lo mix
//	{0x8f, 0x08}, // lo mix
//
//	/* HPL unmute and gain -5db */
//	{HPL_GAIN, 0x40|HP_GAIN},
//	/* HPR unmute and gain -5db */
//	{HPR_GAIN, 0x40|HP_GAIN},
//
//	{0x3f, 0xd6}, // dac
//	{DELAY_REG, 20},
//	{0xed, 0x12}, // hp config, calib
//	{DELAY_REG, 5},
//	{0x89, 0x3c}, // hp
//	{DELAY_REG, 80},
//	{0x3f, 0x15}, // dac
//	{DELAY_REG, 10},
//	//{0x8c, 0x00}, // hp mix
//	//{0x8d, 0x00}, // hp mix
//	{0x8e, 0x00}, // lo mix
//	{0x8f, 0x00}, // lo mix
//
//	{65, SPK_DAC_VOL},
//	{66, SPK_DAC_VOL},
//	{DELAY_REG, 10},
//
//	{0x0b, 0x01}, // NDAC
//	{0x0c, 0x02}, // MDAC
//	{0x12, 0x01}, // NADC
//	{0x13, 0x02}, // MADC
//	
//	{0x44, 0x03}, // DRC
//
//	/* Left mic PGA unmuted */
//	{LMICPGA_VOL_CTRL, 0x00},
//	/* Right mic PGA unmuted */
//	{RMICPGA_VOL_CTRL, 0x00},
//	/* ADC volume control change by 2 gain step per ADC Word Clock */
//	{ADC_CHN_REG, 0x01},
//	/* Unmute ADC left and right channels */
//	{ADC_FGA, 0x00},
//
//	/* PLL is CODEC_CLKIN */
//	{CLK_REG_1, PLLCLK_2_CODEC_CLKIN},
//
//	/* Connect IN1_L and IN1_R to CM */
//	{INPUT_CFG_REG, 0xc0},
//	/* PLL is CODEC_CLKIN */
//	{CLK_REG_1, PLLCLK_2_CODEC_CLKIN},
//	/* DAC_MOD_CLK is BCLK source */
//	{AIS_REG_3, DAC_MOD_CLK_2_BDIV_CLKIN},
//
//	/* Setting up DAC Channel */
//
//	{MICBIAS_CTRL, 0x40},
//	/* Headphone powerup */
//	{HPHONE_STARTUP_CTRL, 0x00},
//	/* HPL unmute and gain -5db */
//	{HPL_GAIN, 0x40|HP_GAIN},
//	/* HPR unmute and gain -5db */
//	{HPR_GAIN, 0x40|HP_GAIN},
//
//	/* LOL unmute and gain 0db */
//	{LOL_GAIN, LO_GAIN},
//	/* LOR unmute and gain 0db */
//	{LOR_GAIN, LO_GAIN},
//
//	{HP_DRIVER_CONF_REG, 0x12},
//
//	{GPIO_CTRL, 0x14 },
//	/*  Headset Insertion event will generate a INT1 interrupt */
//	{INT1_CTRL, 0x80},
//	/*Enable headset detection and button press with a debounce time 64ms */
//	{HEADSET_DETECT, 0x09},//0x89
//	{128 + 0x0a, 0x00},
//	{128 + 0x0b, 0x1E},
//	// PGA
//	{128 + 59, 0x4E},
//	{128 + 60, 0x4E},
//
//	{0xB4, 0x44}, // LPGA P
//	{0xB6, 0x40}, // LPGA N
//	{0x51, 0x80},
//
//	{0x53,0x28},
//	{0x54,0x28},
//	{0x40,0x10},
//#if 0
//	// LEFT AGC
//	{86, 0x80},
//	{87, 50},
//	{88, 50},
//	{89, 22<<3},
//	{90, 4<<3},
//	//{91, 110},
//	//{92, 110},
//	// RIGHT AGC
//	{94, 0x80},
//	{95, 50},
//	{96, 50},
//	{97, 22<<3},
//	{98, 4<<3},
//	//{99, 110},
//	//{100, 110},
//#endif
//};
//#endif
//
static void rec_filter_init(struct snd_soc_codec *codec)
{
	unsigned char v[8];
	unsigned char coeff_0[] = {
		0x7E, 0x33, 0xA8, 0x00,
		0x81, 0xCC, 0x58, 0x00,
		0x7C, 0x67, 0x52, 0x00,
		};
	aic3xxx_bulk_write(codec->control_data,
					AIC3XXX_MAKE_REG(0, 8, 24), 12, coeff_0);
	aic3xxx_bulk_write(codec->control_data,
					AIC3XXX_MAKE_REG(0, 9, 32), 12, coeff_0);
}

static int aic325x_initalize(struct snd_soc_codec *codec)
{	
//	printk("[%s]\n",__func__);
	if (codec_active)
		return 0;

	codec_active = true;

	gpio_request(MX6SL_BRD_AIC325X_RESET, "tlv320-reset-pin");
	gpio_direction_output(MX6SL_BRD_AIC325X_RESET, 0);
	mdelay(1);

	//power on
	if (vgen3) regulator_enable(vgen3);
	mdelay(20);

	gpio_direction_output(MX6SL_BRD_AIC325X_RESET, 1);
	gpio_free(MX6SL_BRD_AIC325X_RESET);
	mdelay(10);

//printk("%s %s %d   \n",__FILE__,__func__,__LINE__);
//-----------------------------------------------------------------------------------
// Software Reset
//-----------------------------------------------------------------------------------
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   1), 0x01);	// Initialize the device through software reset
	//msleep(30);	// Delay 20ms

	//{0x0b, 0x81}, // NDAC
	//{0x0c, 0x82}, // MDAC
	//{0x12, 0x81}, // NADC
	//{0x13, 0x82}, // MADC
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 0x0b), 0x81);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 0x0c), 0x82);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 0x12), 0x81);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 0x13), 0x82);

//-----------------------------------------------------------------------------------
// Configure Power Supplies
//-----------------------------------------------------------------------------------
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,   1), 0x08);	// Disable weak AVDD to DVDD connection
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,   2), 0x00);	// Enable Master Analog Power Control
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  71), 0x00/*0x32*/);	// Set the input power-up time to 3.1ms
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1, 123), 0x01/*0x05*/);	// Force REF charging time to 40ms
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1, 124), 0x06);	// Charge Pump 1x Current, 333kHz clock (8MHz/(6*4))
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,   1), 0x0a);	// use OSC for CP
	msleep(50);	// Delay 50ms

	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  10), 0x0);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,   3), 0x0);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,   4), 0x0);
//-----------------------------------------------------------------------------------
// Load miniDSP Code
//-----------------------------------------------------------------------------------
//	PROGRAM_ADC		// miniDSP_A coefficients and instructions           
//	PROGRAM_DAC		// miniDSP_D coefficients and instructions

//-----------------------------------------------------------------------------------
// Signal Processing Settings
//-----------------------------------------------------------------------------------
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  60), 0x02);	// Use miniDSP_D for signal processing
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  61), 0x00);	// Use miniDSP_A for signal processing

//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  17), 0x08);	// 8x Interpolation
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  23), 0x04);	// 4x Decimation

	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  8,   1), 0x04);	// adaptive mode for ADC
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 44,   1), 0x04);	// adaptive mode for DAC

//-----------------------------------------------------------------------------------
// Clock and Interface Configuration
//-----------------------------------------------------------------------------------
// USB Audio supports 8kHz to 48kHz sample rates
// An external audio interface is required for 88.2kHz to 192kHz sample rates
//-----------------------------------------------------------------------------------
#if 1/*mclk:12Mhz*/
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   5), 0x91);	// P=1, R=1, J=8
	msleep(50);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   6), 0x08);	// P=1, R=1, J=8
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   7), 0x00);	// D=0000 (MSB)
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   8), 0x00);	// D=0000 (LSB)
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   4), 0x03);	// PLL_clkin = MCLK, codec_clkin = PLL_CLK, PLL on
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  12), 0x88);	// MDAC = 8, divider powered on
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  13), 0x00);	// DOSR = 128 (MSB)
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  14), 0x80);	// DOSR = 128 (LSB)
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  18), 0x02);	// NADC = 2, divider powered off
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  19), 0x88);	// MADC = 8, divider powered on
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  20), 0x80);	// AOSR = 128
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  11), 0x82);	// NDAC = 2, divider powered on
#else /*mclk:26Mhz*/    
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   5), 0xa1);	// P=2, R=1, J=8
	msleep(50);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   6), 0x08);	// P=2, R=1, J=8
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   7), 0x1A);	// D=0000 (MSB)
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   8), 0xBB);	// D=0000 (LSB)
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,   4), 0x03);	// PLL_clkin = MCLK, codec_clkin = PLL_CLK, PLL on
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  12), 0x8A);	// MDAC = 8, divider powered on
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  13), 0x00);	// DOSR = 128 (MSB)
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  14), 0x80);	// DOSR = 128 (LSB)
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  18), 0x02);	// NADC = 2, divider powered off
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  19), 0x8A);	// MADC = 8, divider powered on
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  20), 0x80);	// AOSR = 128
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  11), 0x82);	// NDAC = 2, divider powered on
#endif
//-----------------------------------------------------------------------------------
// ADC Channel Configuration
//-----------------------------------------------------------------------------------
       ///disable the Mic Bias.
      // snd_soc_update_bits(codec, AIC3XXX_MAKE_REG(0,  1,   51), 0x00, 0x00);
      //aic3xxx_set_power_rec(codec,0);
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  51), 0x40);	/* Mic Bias disabled, Source = Avdd, 1.25V*/
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  52), 0x40);	// Route IN1L to LEFT_P with 10K input impedance
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  54), 0x40);	// Route CM1L to LEFT_M with 10K input impedance
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  55), 0x40);	// Route IN1R to RIGHT_P with 10K input impedance
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  57), 0x40);	// Route CM1R to RIGHT_M with 10K input impedance
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  59), 0x32);	// Enable MicPGA_L Gain Control, 0dB
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  60), 0x32);	// Enable MicPGA_R Gain Control, 0dB
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  81), 0xc0);	// Power up LADC/RADC
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  82), 0x00);	// Unmute LADC/RADC
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  90), 0x06);
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  98), 0x06);

	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  83), 20);	// adc gain21
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  84), 20);	// adc gain

	///1//, 11: 0x1E // current limit
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 11), 0x1E);

//-----------------------------------------------------------------------------------
// DAC Channel Configuration
//-----------------------------------------------------------------------------------
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  12), 0x08);	// Route LDAC to HPL
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  13), 0x08);	// Route RDAC to HPR
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  14), 0x08);	// Route LDAC to LOL
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  15), 0x08);	// Route LDAC to LOR
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  63), 0xd4);	// Power up LDAC/RDAC w/ soft stepping

	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1, 125), /*0x32*/ 0x1E);	// GCHP Mode, Offset corr Enabled on present routing
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 16), 0x00);		// Unmute HPL driver, 0dB Gain
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 1, 17), 0x00);		// Unmute HPR driver, 0dB Gain change by yu
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  18), 0x01);	// Unmute LOL driver, 0dB Gain
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  19), 0x01);	// Unmute LOR driver, 0dB Gain
//	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,   9), 0x3c);	// Power up HPL/HPR and LOL/LOR drivers
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  64), 0x00);	// Unmute LDAC/RDAC

//	aic3xxx_spk_en(codec, 0);
//	aic3xxx_hp_en(codec, 0);

	//snd_soc_write(codec, AIC3256_DAC_MUTE_CTRL_REG, 0x1C);
	//snd_soc_write(codec, AIC3256_DAC_MUTE_CTRL_REG, 0x0); //unmute DAC
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  63), 0xea);
	snd_soc_write(codec, AIC3256_LDAC_VOL, -20/*0x00*/); // set to 0 max, add gain to hp/lo
	snd_soc_write(codec, AIC3256_RDAC_VOL, -20/*0x00*/);

	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  20), 0xC4); // 0xa9 //0xe9 //...dqk 0x46/0x06
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  1,  20), 0x2); //86

	//enable DAC
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  63), 0xd4);

//#endif
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 56), 0x00);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 68), 0x03); // drc disable
#ifdef AP_GPIO_DET
	// gpio as input
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 52), 0x08);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 48), 0x00);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 67), 0x00); // hp det
#else
	//0, 52: 0x14 // int1
	//0, 48: 0xc0 // hp event -> int 1
	//0, 67: 0x87 // hp det
#if 0 // disable mic det
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 52), 0x14);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 48), 0xC0);
	snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 67), 0x87); // hp det
#endif
#endif
// set as master
	snd_soc_write(codec, AIC3256_INTERFACE_SET_REG_3, 0x05);
	snd_soc_write(codec, AIC3256_CLK_REG_11, 0x82);
	snd_soc_write(codec, AIC3256_INTERFACE_SET_REG_1, 0x3c);

	rec_filter_init(codec);

	return 0;
}

/*
	true: mic
	false: no mic
*/
int aic3256_hs_jack_get(void)
{
	struct snd_soc_codec *codec = codec_inst;
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	int status, state = 0;

	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	snd_soc_write(codec, AIC3256_HEADSET_DETECT, 0x80); // hp det
	msleep(30);
	status = snd_soc_read(codec, AIC3256_HEADSET_DETECT);
	snd_soc_write(codec, AIC3256_HEADSET_DETECT, 0x00); // hp det

	/* We will check only stereo MIC and headphone */
	switch (status & AIC3256_JACK_TYPE_MASK) {
	case AIC3256_JACK_WITH_MIC:
		state |= SND_JACK_HEADSET;
		break;
	case AIC3256_JACK_WITHOUT_MIC:
		state |= SND_JACK_HEADPHONE;
	}
	return ((state & SND_JACK_HEADSET) == SND_JACK_HEADSET);
}
EXPORT_SYMBOL_GPL(aic3256_hs_jack_get);
static void aic3256_hs_jack_report(struct snd_soc_codec *codec,
					struct snd_soc_jack *jack, int report)
{
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	int status, state = 0, switch_state = BIT_NO_ACCESSORY;

	mutex_lock(&aic3256->io_lock);

	/* Sync status */
	status = snd_soc_read(codec, AIC3256_HEADSET_DETECT);
	/* We will check only stereo MIC and headphone */
	switch (status & AIC3256_JACK_TYPE_MASK) {
	case AIC3256_JACK_WITH_MIC:
		state |= SND_JACK_HEADSET;
		break;
	case AIC3256_JACK_WITHOUT_MIC:
		state |= SND_JACK_HEADPHONE;
	}

	mutex_unlock(&aic3256->io_lock);

	snd_soc_jack_report(jack, state, report);

	if ((state & SND_JACK_HEADSET) == SND_JACK_HEADSET)
		switch_state |= BIT_HEADSET;
	else if (state & SND_JACK_HEADPHONE)
		switch_state |= BIT_HEADPHONE;

}

/**
 * aic3256_hs_jack_detect: Detect headphone jack during boot time
 * @codec: pointer variable to codec having information related to codec
 * @jack: Pointer variable to snd_soc_jack having information of codec
 *	     and pin number$
 * @report: Provides informaton of whether it is headphone or microphone
 *
*/
void aic3256_hs_jack_detect(struct snd_soc_codec *codec,
			struct snd_soc_jack *jack, int report)
{
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);
	struct aic3256_jack_data *hs_jack = &aic3256->hs_jack;

	hs_jack->jack = jack;
	hs_jack->report = report;
	aic3256_hs_jack_report(codec, hs_jack->jack, hs_jack->report);
}
EXPORT_SYMBOL_GPL(aic3256_hs_jack_detect);

void aic325x_set_gpio1(struct snd_soc_codec *codec, int val)
{
//#if 0
//	dev_dbg(codec->dev, "%s %d\n", __func__,val);
//	if(val)
//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 52), 0x0D);
//	else
//		snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 52), 0x0C);
//#endif
}
EXPORT_SYMBOL_GPL(aic325x_set_gpio1);

#ifdef HEADSET_DETECTION

static void get_hs_status(struct snd_soc_codec *codec)
{
	int ret;
	gpio_direction_input(AIC_HPDET_PIN);	
	ret = gpio_get_value(AIC_HPDET_PIN);
	printk("aic3xxx %s, AIC_HPDET_PIN = %d\n", __func__, ret);
#ifdef AP_GPIO_DET
	if (ret) {
		// no hs
		//aic3xxx_on_headphone(codec, 0);
		//set_spk_coeff(codec, 1);
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  65), 0xff);
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  66), 0xff);
		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  63), 0xea);
		aic3xxx_jack_get(codec, 0);
	} else {
		//aic3xxx_on_headphone(codec, 1);
		//set_spk_coeff(codec, 0);
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  65), 0xf4);
		snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  66), 0xf4);
		//snd_soc_write(codec, AIC3XXX_MAKE_REG(0,  0,  63), 0xd6);
		aic3xxx_jack_get(codec, 1);
	}
#else
	aic3xxx_jack_get(codec, 1);
#endif
}

static void pdet_function(struct work_struct *work)
{
	///return;
	printk("%s %s %d  pdet_irq_requested=%d \n",__FILE__,__func__,__LINE__,pdet_irq_requested); 

//#if 1 // need external pull up
	struct snd_soc_codec * codec = aic325x_codec;
	int irq = gpio_to_irq(AIC_HPDET_PIN);
	unsigned long flags;
	int state = BIT_HEADSET_NO;
	int ret;

	if (!codec) {
		pr_err("aic325x_codec is null\n");
		return;
	}
	dev_info(codec->dev, "%s\n", __func__);
	if (pdet_irq_requested)
		free_irq(irq, NULL);

	gpio_direction_input(AIC_HPDET_PIN);	
	ret = gpio_get_value(AIC_HPDET_PIN);
	if(0==ret)
		state |= BIT_HEADSET_MIC;
	headset_insert = state;
	rk_headset_report(state);
	msleep(200);
	get_hs_status(codec);

	aic3xxx_set_power_play(codec, 0);
	aic3xxx_set_power_play(codec, 1);
	flags = (ret == GPIO_HIGH)?(IRQF_TRIGGER_FALLING) : (IRQF_TRIGGER_RISING);
	ret = request_irq(irq, pdet_handler, flags, "pdet", NULL);
	if (ret < 0) {
		pr_err("request_irq(%d) failed\n", irq);
	}
	pdet_irq_requested = true;
	printk("pdet done\n");
//#endif
	return;
}

static irqreturn_t pdet_handler(int irq, void *dev_id)
{
	struct snd_soc_codec * codec = aic325x_codec;
	printk("%s %s %d  pdet_irq_requested=%d \n",__FILE__,__func__,__LINE__,pdet_irq_requested); 
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 67), 0x07);
	//msleep(100);
	//snd_soc_write(codec, AIC3XXX_MAKE_REG(0, 0, 67), 0x87);
	disable_irq_nosync(irq);
	schedule_delayed_work(&pdet_work, msecs_to_jiffies(300));
	return IRQ_HANDLED;
}

static void pdet_initalize(void)
{
	int ret = 0;
	if (!gpio_is_valid(AIC_HPDET_PIN))
		return;

	printk("aic32xx det pin\n");
	//huskar	
	//rk30_mux_api_set(AIC_HPDET_MUX_NAME, AIC_HPDET_MUX_GPIO);
	ret = gpio_request(AIC_HPDET_PIN, "phone_det");
	if (ret) {
		pr_err("failed to get GPIO[AIC_HPDET_PIN]:%d\n", ret);
		gpio_free(AIC_HPDET_PIN);
		return ret;
	}
	//gpio_pull_updown(AIC_HPDET_PIN, PullDisable);
	gpio_direction_input(AIC_HPDET_PIN);	
	printk("aic32xx det pin done\n");
	pdet_irq_requested = false;
//#else
//	if (gpio_request(AIC_HPDET_PIN, "spk_ctl")) {
//		printk("spk_ctl alloc failed\n");
//	}
//	gpio_direction_output(AIC_HPDET_PIN, GPIO_HIGH);
//#endif
}
#else
static void pdet_initalize(void) {return;}
#endif

static void codec_hw_pin_init(void)
{
	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	/*if (gpio_is_valid(AIC_SPK_PIN))
	{
		printk("aic32xx spk pin\n");
		//rk30_mux_api_set(AIC_SPK_MUX_NAME, AIC_SPK_MUX_GPIO);
		if (gpio_request(AIC_SPK_PIN, "spk_pwrctl")) {
			gpio_free(AIC_SPK_PIN);
		}
		//gpio_pull_updown(AIC_SPK_PIN, PullDisable);
		gpio_direction_output(AIC_SPK_PIN, AIC_SPK_PWDN);
		printk("aic32xx spk pin done\n");
	}
	if (gpio_is_valid(AIC_RESET_PIN))
	{
		if (gpio_request(AIC_RESET_PIN, "codec_resetctl")) {
			gpio_free(AIC_RESET_PIN);
		}
		gpio_direction_output(AIC_RESET_PIN, GPIO_LOW);
		msleep(200);
		gpio_direction_output(AIC_RESET_PIN, GPIO_HIGH);
		msleep(100);
	}
	if (gpio_is_valid(AIC_HPEN_PIN))
	{
		if (gpio_request(AIC_HPEN_PIN, "codec_hp_enx")) {
			printk("codec_hp_en request failed\n");
			//gpio_free(AIC_HPEN_PIN);
		}
		gpio_direction_output(AIC_HPEN_PIN,GPIO_HIGH);
	}*/
	return;
}

static void set_spk_coeff(struct snd_soc_codec *codec, int on)
{
	unsigned char coeff_off[] = {
		0x7F,0xFF,0xFF,0x00,
		0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,
		};
	unsigned char coeff_0[] = {
		0x7B, 0x90, 0x87, 0x00,
		0x84, 0x6F, 0x79, 0x00,
		0x7B, 0x90, 0x87, 0x00,
		0x7B, 0x7C, 0xDB, 0x00,
		0x88, 0xB7, 0x98, 0x00,
		};
	unsigned char coeff_1[] = {
		0x7B, 0x90, 0x87, 0x00,
		0x84, 0x6F, 0x79, 0x00,
		0x7B, 0x90, 0x87, 0x00,
		0x7B, 0x7C, 0xDB, 0x00,
		0x88, 0xB7, 0x98, 0x00,
		};
	unsigned char coeff_2[] = {
		0x7F,0xFF,0xFF,0x00,
		0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,
		};
	
	if(on)
	{
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 44, 12), 20, coeff_0);
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 44, 32), 20, coeff_1);
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 44, 52), 20, coeff_2);
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 45, 20), 20, coeff_0);
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 45, 40), 20, coeff_1);
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 45, 60), 20, coeff_2);
	}
	else
	{
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 44, 12), 20, coeff_off);
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 44, 32), 20, coeff_off);
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 44, 52), 20, coeff_off);
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 45, 20), 20, coeff_off);
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 45, 40), 20, coeff_off);
		aic3xxx_bulk_write(codec->control_data,
						AIC3XXX_MAKE_REG(0, 45, 60), 20, coeff_off);
	}
}

static int aic325x_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	//struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct aic325x_priv *aic325x;
	struct aic3xxx *control;
	//struct aic3xxx_dsp_priv *dsp_priv = NULL;
	printk("%s %s %d   \n",__FILE__,__func__,__LINE__);

	codec->control_data = dev_get_drvdata(codec->dev->parent);
	control = codec->control_data;
	aic325x = kzalloc(sizeof(struct aic325x_priv), GFP_KERNEL);
	if (aic325x == NULL)
		return -ENOMEM;
	
	snd_soc_codec_set_drvdata(codec, aic325x);

	codec->dapm.idle_bias_off = 1;

	aic325x->codec = codec;
	aic325x->cur_fw = NULL;
	//aic325x->cfw_p = &(aic325x->cfw_ps);
	aic325x_codec = codec;
	codec_inst = codec;

	gpio_request(MX6SL_BRD_AIC325X_RESET, "tlv320-reset-pin");
	gpio_direction_output(MX6SL_BRD_AIC325X_RESET, 0);
	mdelay(1);

	vgen3 = regulator_get(aic325x->codec->dev,"VGEN3_1V8");
	printk("[%s] vgen3 = %p\n",__func__,vgen3);

	if (IS_ERR(vgen3))
		vgen3 = NULL;
//
//	if (vgen3) regulator_enable(vgen3);
//	mdelay(50);
//
//	codec_hw_pin_init();
//
//	gpio_direction_output(MX6SL_BRD_AIC325X_RESET, 1);
//	mdelay(10);
//
////	dsp_priv = kzalloc(sizeof(struct aic3xxx_dsp_priv), GFP_KERNEL);
////	if (!dsp_priv) {
////		kfree(aic325x);
////		dev_err(codec->dev, "Unable to Allocate dsp struct\n");
////		return -ENOMEM;
////	}
////	aic325x->dsp_priv = dsp_priv;
////	dsp_priv->codec = codec;
////	dsp_priv->io_lock = &codec->mutex;
////	dsp_priv->cur_mode = -1;
////	dsp_priv->cur_config = -1;
////	dsp_priv->gpio_spi_cs = 0;
////	dsp_priv->dev = aic3xxx_get_i2c_dev(codec->control_data);
//
//	printk("[%s] reset\n",__func__);
//
//	/* run the codec through software reset */
//	ret = snd_soc_write(codec,AIC3XXX_RESET,1);
//	if (ret < 0) {
//		dev_err(codec->dev, "Could not write to AIC3XXX register\n");
//	}
//	mdelay(30);
//	ret = snd_soc_read(codec,AIC3XXX_DEVICE_ID);
//	if (ret < 0) {
//		dev_err(codec->dev, "Failed to read ID register\n");
//	}
//	dev_dbg(codec->dev, "revision %d\n", ret);
//	printk("[%s] rev = %i\n",__func__,ret);
//
//	//set to reset state
//	gpio_direction_output(MX6SL_BRD_AIC325X_RESET, 0);
//	mdelay(1);
//	gpio_free(MX6SL_BRD_AIC325X_RESET);
//
//	if (vgen3) regulator_disable(vgen3);
//	mdelay(100);

//#if 0
//	snd_soc_dapm_new_controls(dapm, aic325x_dapm_widgets,
//				ARRAY_SIZE(aic325x_dapm_widgets));
//	ret = snd_soc_dapm_add_routes(dapm, aic325x_dapm_routes,
//				ARRAY_SIZE(aic325x_dapm_routes));
//	if (!ret)
//		dev_dbg(codec->dev, "#Completed adding DAPM routes = %d\n",
//			ARRAY_SIZE(aic325x_dapm_routes));
//#endif

	mutex_init(&aic325x->io_lock);
	//aic3xxx_driver_init(codec);

	//aic325x_initalize(codec);

	//set_spk_coeff(codec, 1);// add by yu 10-11

	p_the_codec = codec;
	register_reboot_notifier(&aic325x_shutdown_notifier);
	pdet_initalize();

	//aic325x_set_bias_level(codec, SND_SOC_BIAS_OFF);
	//codec->dapm.idle_bias_off = 1;

//#if 0
//	aic3xxx_cfw_init(aic325x->cfw_p, 
//		       (struct aic3xxx_codec_ops *)&aic3256_cfw_codec_ops,
//				aic325x->codec);
//#endif
	/// firmware load
	//request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
	//			"tlv320aic3254_fw_v1.bin",
	//			codec->dev, GFP_KERNEL, codec,
	//			aic3256_firmware_load);
	///

	//get_hs_status(codec);
	//schedule_delayed_work(&pdet_work, msecs_to_jiffies(100));
	printk("[%s] finish\n",__func__);
	return ret;
}

static int aic325x_remove(struct snd_soc_codec *codec)
{
	struct aic325x_priv *aic3256 = snd_soc_codec_get_drvdata(codec);

	unregister_reboot_notifier(&aic325x_shutdown_notifier);

	aic325x_set_bias_level(codec, SND_SOC_BIAS_OFF);
	if (aic3256->cur_fw != NULL)
		release_firmware(aic3256->cur_fw);

	if (vgen3) {
		if (regulator_is_enabled(vgen3))
			regulator_disable(vgen3);
		regulator_put(vgen3);
		vgen3 = NULL;
	}

	kfree(aic3256->dsp_priv);
	kfree(aic3256);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_driver_aic325x = {
	.probe = aic325x_probe,
	.remove = aic325x_remove,
	.suspend = aic325x_suspend,
	.resume = aic325x_resume,
	.read = aic325x_codec_read,
	.write = aic325x_codec_write,
	.set_bias_level = aic325x_set_bias_level,
	.controls = aic325x_snd_controls ,
	.num_controls = ARRAY_SIZE(aic325x_snd_controls),
	.reg_cache_size = 0,
	.reg_word_size = sizeof(u8),
	.reg_cache_default = NULL,
};

//int aic3256_probe(struct i2c_client *client,struct aic3xxx *aic3xxx,struct device *dev)
static int aic3256_probe(struct platform_device *pdev)
{
    int ret =-1;
	ret= snd_soc_register_codec(
					&pdev->dev,
					&soc_codec_driver_aic325x,
					tlv320aic325x_dai_driver,
					ARRAY_SIZE(tlv320aic325x_dai_driver));
	//printk("%s,%s,%d ret=%d \n",__FILE__,__func__,__LINE__,ret);
	if (ret) {
		//dev_err(&client->dev, "Failed to register codec: %d\n", ret);
		//kfree(aic3xxx);
		return ret;
	}

	return 0;
}

static int aic3256_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver aic325x_codec_driver = {
	.driver = {
		.name =AIC3256_CODEC_NAME,
		.owner = THIS_MODULE,
	},
	.probe = aic3256_probe,
	.remove = __devexit_p(aic3256_remove),
};

static int __init tlv320aic325x_init(void)
{

	platform_driver_register(&aic325x_codec_driver);
	return 0;
}

static void __exit tlv320aic325x_exit(void)
{
	return platform_driver_unregister(&aic325x_codec_driver);
}

module_init(tlv320aic325x_init);
module_exit(tlv320aic325x_exit);

MODULE_ALIAS("platform:tlv320aic325x-codec");
MODULE_DESCRIPTION("ASoC TLV320AIC325x codec driver");
MODULE_LICENSE("GPL");


#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_reg_show (struct seq_file *s, void *v)
{
	struct snd_soc_codec *codec = aic325x_codec;
	int i, j, k, offset;
	u8 val_page0[0x80] = {0,};
	u8 val_page1[0x80] = {0,};

	for (i = 0; i < 0x80; i++)
		val_page0[i] = snd_soc_read(codec, i);

	for (i = 0; i < 0x80; i++)
		val_page1[i] = snd_soc_read(codec, i+256);

	for (i = 0; i < 0x80/16; i++) {
		offset = (i * 16) % 128;
		if (offset == 0) {
			seq_printf (s, "    ");
			for (k = 0; k < 16; k++)
				seq_printf (s, "   %x", k);
			seq_printf (s, "\n");
		}
		seq_printf (s, " %3x:", i * 16);
		for (j = 0; j < 16; j++) {
			seq_printf (s, "  %02x", val_page0[i * 16 + j]);
		}
		seq_printf (s, "\n");
	}

	for (i = 0; i < 0x80/16; i++) {
		offset = (i * 16) % 128;
		seq_printf (s, "  %3x:", i * 16+256);
		for (j = 0; j < 16; j++) {
			seq_printf (s, "  %02x", val_page1[i * 16 + j]);
		}
		seq_printf (s, "\n");
	}
	seq_printf (s, "\n");	
	return 0;
}

static int proc_reg_open (struct inode *inode, struct file *file)
{
	return single_open (file, proc_reg_show, NULL);
}

static const struct file_operations proc_reg_fops = {
	.open		= proc_reg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init codec_proc_init (void)
{
	proc_create ("aic325x", 0, NULL, &proc_reg_fops);
	return 0;
}
late_initcall (codec_proc_init);
#endif /* CONFIG_PROC_FS */

