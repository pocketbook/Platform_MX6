/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mach/iomux-mx6sl.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include <linux/imx6sl_evk/imx6sl_evk_gpio_cfg.h>
#include "mxc_v4l2_capture.h"
#include "fsl_csi.h"

#define DDD(x...) printk(x)

#define OV5640_RW_ATTRIBUTE(name, mode) \
    static struct kobj_attribute name##_attribute = __ATTR(name, mode, name##_show, name##_store);

#define OV5640_REG_ATTRIBUTE(name, mode, var) \
    static ssize_t name##_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) \
    { \
        return scnprintf(buf, PAGE_SIZE, "%d\n", var); \
    } \
    static ssize_t name##_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) \
    { \
        return update_timing_value(&(var), buf, count); \
    } \
    static struct kobj_attribute name##_attribute = __ATTR(name, mode, name##_show, name##_store);


#define OV5640_VOLTAGE_ANALOG               2800000
#define OV5640_VOLTAGE_DIGITAL_CORE         1500000
#define OV5640_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV5640_XCLK_MIN 6000000
#define OV5640_XCLK_MAX 24000000

#define OV5640_CHIP_ID_HIGH_BYTE        0x300A
#define OV5640_CHIP_ID_LOW_BYTE         0x300B

#define AF_PERIOD 3000

#define AF_STOPPED    0
#define AF_SINGLESHOT 1
#define AF_CONTINUOUS 2

static int is_power_on = -1;
static int af_download = 0;
static int ov5640_current_framerate = 0;
static int ov5640_current_mode = 0;
static struct task_struct *autofocus_task;
static int af_mode = 0;
static int af_status = V4L2_AUTO_FOCUS_STATUS_IDLE;

static DECLARE_WAIT_QUEUE_HEAD(af_start_wq);

static struct delayed_work streamoff_work;
static struct delayed_work ae_on_work;

static int savemode=0;
#if defined(CONFIG_PB650_DEEP_STANDY)
extern int iDeepStandy;
#endif
#define OV5640_CMD_MAIN_Reg		0x3022
//#define OV5640_CMD_TAG_Reg			0x3023
#define OV5640_CMD_ACK_Reg 		0x3023
#define OV5640_CMD_PARA0_Reg		0x3024
#define OV5640_CMD_PARA1_Reg		0x3025
#define OV5640_CMD_PARA2_Reg		0x3026
#define OV5640_CMD_PARA3_Reg		0x3027
#define OV5640_CMD_PARA4_Reg		0x3028

//#define OV5640_STA_ZONE_Reg			0x3026
#define OV5640_STA_FOCUS_Reg		0x3029

/* ov5640 VCM Command  */

#define OV5640_ConstFocus_Cmd	  0x04
#define OV5640_StepMode_Cmd	  0x05
#define OV5640_PauseFocus_Cmd	  0x06
#define OV5640_ReturnIdle_Cmd	  0x08
#define OV5640_SetZone_Cmd 	  0x10
#define OV5640_UpdateZone_Cmd	  0x12
#define OV5640_SetMotor_Cmd	  0x20
#define OV5640_SingleFocus_Cmd 			0x03
#define OV5640_GetFocusResult_Cmd			0x07
#define OV5640_ReleaseFocus_Cmd			0x08
#define OV5640_ZoneRelaunch_Cmd			0x12
#define OV5640_DefaultZoneConfig_Cmd		0x80
#define OV5640_TouchZoneConfig_Cmd 		0x81
#define OV5640_CustomZoneConfig_Cmd		0x8f


/* ov5640 Focus State */
//#define S_FIRWRE				0xFF		/*Firmware is downloaded and not run*/
#define ov5640_S_STARTUP			0x7e		/*Firmware is initializing*/
#define ov5640_S_ERROR 			0x7f
#define ov5640_S_IDLE				0x70		/*Idle state, focus is released; lens is located at the furthest position.*/
#define ov5640_S_FOCUSING			0x00		/*Auto Focus is running.*/
#define ov5640_S_FOCUSED			0x10		/*Auto Focus is completed.*/

#define ov5640_S_CAPTURE			0x12
#define ov5640_S_STEP					0x20

/* ov5640 Zone State */
#define ov5640_Zone_Is_Focused(a, zone_val)	(zone_val&(1<<(a-3)))
#define ov5640_Zone_Get_ID(zone_val)			(zone_val&0x03)

#define ov5640_Zone_CenterMode   0x01
#define ov5640_Zone_5xMode 	  0x02
#define ov5640_Zone_5PlusMode	  0x03
#define ov5640_Zone_4fMode 	  0x04

#define ov5640_ZoneSel_Auto	  0x0b
#define ov5640_ZoneSel_SemiAuto  0x0c
#define ov5640_ZoneSel_Manual	  0x0d
#define ov5640_ZoneSel_Rotate	  0x0e

/* ov5640 Step Focus Commands */
#define ov5640_StepFocus_Near_Tag		 0x01
#define ov5640_StepFocus_Far_Tag		 0x02
#define ov5640_StepFocus_Furthest_Tag	 0x03
#define ov5640_StepFocus_Nearest_Tag	 0x04
#define ov5640_StepFocus_Spec_Tag		 0x10

#define INVMASK(v) (0xff-(v))

enum ov5640_mode {
	ov5640_mode_MIN = 0,
	ov5640_mode_VGA_640_480 = 0,
	ov5640_mode_QVGA_320_240 = 1,
	ov5640_mode_NTSC_720_480 = 2,
	ov5640_mode_PAL_720_576 = 3,
	ov5640_mode_720P_1280_720 = 4,
	ov5640_mode_1080P_1920_1080 = 5,
	ov5640_mode_QSXGA_2592_1944 = 6,
	ov5640_mode_QCIF_176_144 = 7,
	ov5640_mode_XGA_1024_768 = 8,
	ov5640_mode_MAX = 8
	//ov5640_mode_MAX = 2
};

enum ov5640_frame_rate {
	ov5640_15_fps,
	ov5640_30_fps
};
enum ov5640_process_mode {
   ov5640_preview,
   ov5640_capture
};
static int g_process_mode=ov5640_preview;

static int ov5640_framerates[] = {
	[ov5640_15_fps] = 15,
	[ov5640_30_fps] = 30,
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov5640_mode_info {
	enum ov5640_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

#define MAX_USER_SCRIPT 64

struct {
	u16 reg;
	u16 value;
} user_script [ MAX_USER_SCRIPT ];

int user_script_len = 0;

#include "ov5640_init.h"

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct sensor_data ov5640_data;
static int prev_sysclk;
static int AE_Target = 52, night_mode;
static int prev_HTS;
static int AE_high, AE_low;

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;
static struct fsl_mxc_camera_platform_data *camera_plat;

static DEFINE_MUTEX(i2c_mutex);

static int ov5640_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov5640_remove(struct i2c_client *client);
static int ov5640_suspend(struct i2c_client *client);
static int ov5640_resume(struct i2c_client *client);
static s32 ov5640_read_reg(u16 reg, u8 *val);
static s32 ov5640_write_reg(u16 reg, u8 val);
static int ov5640_init_auto_focus(void);
static int ov5640_change_mode(enum ov5640_frame_rate frame_rate,
                            enum ov5640_mode mode);

extern int mx6sl_setCameraPadVoltage(void);

static const struct i2c_device_id ov5640_id[] = {
	{"ov5640", 0},
	{"ov564x", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov5640_id);

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov5640",
		  },
	.probe  = ov5640_probe,
	.remove = ov5640_remove,
	.suspend= ov5640_suspend,
	.resume = ov5640_resume,
	.id_table = ov5640_id,
};


static s32 ov5640_write_reg(u16 reg, u8 val)
{
	u8 au8Buf[3] = {0};

	mutex_lock(&i2c_mutex);

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(ov5640_data.i2c_client, au8Buf, 3) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		mutex_unlock(&i2c_mutex);
		return -1;
	}

	mutex_unlock(&i2c_mutex);
	return 0;
}

static s32 ov5640_write_data(u16 reg, u8 *val, int count)
{
	u8 au8Buf[20] = {0};

	mutex_lock(&i2c_mutex);

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	memcpy(au8Buf+2, val, count);

	if (i2c_master_send(ov5640_data.i2c_client, au8Buf, 2+count) < 0) {
		pr_err("%s:write data error:reg=%x,count=%d\n",
			__func__, reg, count);
		mutex_unlock(&i2c_mutex);
		return -1;
	}

	mutex_unlock(&i2c_mutex);
	return 0;
}

static s32 ov5640_read_reg(u16 reg, u8 *val)
{
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	mutex_lock(&i2c_mutex);

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (2 != i2c_master_send(ov5640_data.i2c_client, au8RegBuf, 2)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		mutex_unlock(&i2c_mutex);
		return -1;
	}

	if (1 != i2c_master_recv(ov5640_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		mutex_unlock(&i2c_mutex);
		return -1;
	}

	*val = u8RdVal;

	mutex_unlock(&i2c_mutex);
	return u8RdVal;
}


static s32 ov5640_write_preg(u16 reg, u8 val, u8 mask)
{
	u8 v;

	if (ov5640_read_reg(reg, &v) == -1) return -1;
	v &= mask;
	v |= (val & ~mask);
	if (ov5640_write_reg(reg, v) == -1) return -1;
	return 0;
}

static void ov5640_soft_reset(void)
{
	/* sysclk from pad */
	ov5640_write_reg(0x3103, 0x11);

	/* software reset */
	ov5640_write_reg(0x3008, 0x82);

	/* delay at least 5ms */
	msleep(10);
}

/* set sensor driver capability
 * 0x302c[7:6] - strength
	00     - 1x
	01     - 2x
	10     - 3x
	11     - 4x
 */
static int ov5640_driver_capability(int strength)
{
	u8 temp = 0;

	if (strength > 4 || strength < 1) {
		pr_err("The valid driver capability of ov5640 is 1x~4x\n");
		return -EINVAL;
	}

	ov5640_read_reg(0x302c, &temp);

	temp &= ~0xc0;	/* clear [7:6] */
	temp |= ((strength - 1) << 6);	/* set [7:6] */

	ov5640_write_reg(0x302c, temp);

	return 0;
}
/* calculate sysclk */
static int ov5640_get_sysclk(void)
{
	int xvclk = ov5640_data.mclk / 10000;
	int sysclk;
	int temp1, temp2;
	int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv, Bit_div2x, sclk_rdiv;
	int sclk_rdiv_map[] = {1, 2, 4, 8};
	u8 regval = 0;

	temp1 = ov5640_read_reg(0x3034, &regval);
	temp2 = temp1 & 0x0f;
	if (temp2 == 8 || temp2 == 10) {
		Bit_div2x = temp2 / 2;
	} else {
		pr_err("ov5640: unsupported bit mode %d\n", temp2);
		return -1;
	}

	temp1 = ov5640_read_reg(0x3035, &regval);
	SysDiv = temp1 >> 4;
	if (SysDiv == 0){
	SysDiv = 16;}

	temp1 = ov5640_read_reg(0x3036, &regval);
	Multiplier = temp1;
	temp1 = ov5640_read_reg(0x3037, &regval);
	PreDiv = temp1 & 0x0f;
	Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;

	temp1 = ov5640_read_reg(0x3108, &regval);
	temp2 = temp1 & 0x03;

	sclk_rdiv = sclk_rdiv_map[temp2];
	VCO = xvclk * Multiplier / PreDiv;
	sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;

	return sysclk;
}

/* read HTS from register settings */
static int ov5640_get_HTS(void)
{
	int HTS;
	u8 temp = 0;

	HTS = ov5640_read_reg(0x380c, &temp);
	HTS = (HTS<<8) + ov5640_read_reg(0x380d, &temp);
	return HTS;
}

/* read VTS from register settings */
static int ov5640_get_VTS(void)
{
	int VTS;
	u8 temp = 0;

	VTS = ov5640_read_reg(0x380e, &temp);
	VTS = (VTS<<8) + ov5640_read_reg(0x380f, &temp);

	return VTS;
}

/* write VTS to registers */
static int ov5640_set_VTS(int VTS)
{
	int temp;

	temp = VTS & 0xff;
	ov5640_write_reg(0x380f, temp);

	temp = VTS>>8;
	ov5640_write_reg(0x380e, temp);
	return 0;
}

/* read shutter, in number of line period */
static int ov5640_get_shutter(void)
{
	int shutter;
	u8 regval;

	shutter = (ov5640_read_reg(0x03500, &regval) & 0x0f);

	shutter = (shutter<<8) + ov5640_read_reg(0x3501, &regval);
	shutter = (shutter<<4) + (ov5640_read_reg(0x3502, &regval)>>4);

	return shutter;
}

/* write shutter, in number of line period */
static int ov5640_set_shutter(int shutter)
{
	int temp;

	shutter = shutter & 0xffff;
	temp = shutter & 0x0f;
	temp = temp<<4;
	ov5640_write_reg(0x3502, temp);

	temp = shutter & 0xfff;
	temp = temp>>4;
	ov5640_write_reg(0x3501, temp);

	temp = shutter>>12;
	ov5640_write_reg(0x3500, temp);

	return 0;
}

/* read gain, 16 = 1x */
static int ov5640_get_gain16(void)
{
	int gain16;
	u8 regval;

	gain16 = ov5640_read_reg(0x350a, &regval) & 0x03;
	gain16 = (gain16<<8) + ov5640_read_reg(0x350b, &regval);

	return gain16;
}

/* write gain, 16 = 1x */
static int ov5640_set_gain16(int gain16)
{
	int temp;

	gain16 = gain16 & 0x3ff;
	temp = gain16 & 0xff;

	ov5640_write_reg(0x350b, temp);
	temp = gain16>>8;

	ov5640_write_reg(0x350a, temp);
	return 0;
}

/* get banding filter value */
static int ov5640_get_light_freq(void)
{
	int temp, temp1, light_frequency;
	u8 regval;

	temp = ov5640_read_reg(0x3c01, &regval);
	if (temp & 0x80) {
		/* manual */
		temp1 = ov5640_read_reg(0x3c00, &regval);
		if (temp1 & 0x04) {
			/* 50Hz */
			light_frequency = 50;
		} else {
			/* 60Hz */
			light_frequency = 60;
		}
	} else {
		/* auto */
		temp1 = ov5640_read_reg(0x3c0c, &regval);
		if (temp1 & 0x01) {
			/* 50Hz */
			light_frequency = 50;
		} else {
			/* 60Hz */
			light_frequency = 60;
		}
	}

	return light_frequency;
}

static void ov5640_set_bandingfilter(void)
{
	int prev_VTS;
	int band_step60, max_band60, band_step50, max_band50;

	/* read preview PCLK */
	prev_sysclk = ov5640_get_sysclk();

	/* read preview HTS */
	prev_HTS = ov5640_get_HTS();

	/* read preview VTS */
	prev_VTS = ov5640_get_VTS();

	/* calculate banding filter */
	/* 60Hz */
	band_step60 = prev_sysclk * 100/prev_HTS * 100/120;
	ov5640_write_reg(0x3a0a, (band_step60 >> 8));
	ov5640_write_reg(0x3a0b, (band_step60 & 0xff));

	max_band60 = (int)((prev_VTS-4)/band_step60);
	ov5640_write_reg(0x3a0d, max_band60);

	/* 50Hz */
	band_step50 = prev_sysclk * 100/prev_HTS;
	ov5640_write_reg(0x3a08, (band_step50 >> 8));
	ov5640_write_reg(0x3a09, (band_step50 & 0xff));

	max_band50 = (int)((prev_VTS-4)/band_step50);
	ov5640_write_reg(0x3a0e, max_band50);
}

/* stable in high */
static int ov5640_set_AE_target(int target)
{
	int fast_high, fast_low;

	AE_low = target * 23 / 25; /* 0.92 */
	AE_high = target * 27 / 25; /* 1.08 */
	fast_high = AE_high << 1;

	if (fast_high > 255)
		fast_high = 255;
	fast_low = AE_low >> 1;

	ov5640_write_reg(0x3a0f, AE_high);
	ov5640_write_reg(0x3a10, AE_low);
	ov5640_write_reg(0x3a1b, AE_high);
	ov5640_write_reg(0x3a1e, AE_low);
	ov5640_write_reg(0x3a11, fast_high);
	ov5640_write_reg(0x3a1f, fast_low);

	return 0;
}

/* enable = 0 to turn off night mode
   enable = 1 to turn on night mode */
static int ov5640_set_night_mode(int enable)
{
	u8 mode;

	ov5640_read_reg(0x3a00, &mode);

	if (enable) {
		/* night mode on */
		mode |= 0x04;
		ov5640_write_reg(0x3a00, mode);
	} else {
		/* night mode off */
		mode &= 0xfb;
		ov5640_write_reg(0x3a00, mode);
	}

	return 0;
}

/* enable = 0 to turn off AEC/AGC
   enable = 1 to turn on AEC/AGC */
void ov5640_turn_on_AE_AG(int enable)
{
	u8 ae_ag_ctrl;

	ov5640_read_reg(0x3503, &ae_ag_ctrl);
	if (enable) {
		/* turn on auto AE/AG */
		ae_ag_ctrl = ae_ag_ctrl & ~(0x03);
	} else {
		/* turn off AE/AG */
		ae_ag_ctrl = ae_ag_ctrl | 0x03;
	}
	ov5640_write_reg(0x3503, ae_ag_ctrl);
}

/* download ov5640 settings to sensor through i2c */
static int ov5640_download_firmware(struct reg_value *pModeSetting, s32 ArySize)
{
	static u8 data[16];
	static u16 DataAddr;
	static int n;

	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int i, retval = 0;


#if 1
	for (i = n = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;
		if (RegAddr != DataAddr+n || Mask != 0 || Delay_ms != 0 || n == 16) {
			if (n > 0) {
				retval = ov5640_write_data(DataAddr, data, n);
				if (retval < 0)
					goto err;
				n = 0;
			}
		}
		if (Mask != 0 || Delay_ms != 0) {
			retval = ov5640_read_reg(RegAddr, &RegVal);
			if (retval < 0)
                                goto err;

                        RegVal &= ~(u8)Mask;
                        Val &= Mask;
                        Val |= RegVal;

			retval = ov5640_write_reg(RegAddr, Val);
			if (retval < 0)
                        goto err;

	                if (Delay_ms)
        	                msleep(Delay_ms);
		} else {
			if (n == 0) DataAddr = RegAddr;
			data[n++] = Val;
		}
	}
	if (n > 0) {
		retval = ov5640_write_data(DataAddr, data, n);
		if (retval < 0)
			goto err;
	}
			
#else 
	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;
		//printk("i=%d,RegAddr=0x%x, Val=0x%x,Mask=0x%x,Delay_ms=%d\n",i,RegAddr,Val,Mask,Delay_ms);

		if (Mask) {
			retval = ov5640_read_reg(RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov5640_write_reg(RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
#endif 

err:
	return retval;
}
/*
After download the firmware, please check the registers below:
MCU on:0x3000 BIT6=0 BIT5=0 0x3004 BIT6=1 BIT5=1
AFC on :0x3001 BIT6=0 0x3005 BIT6=1
*/
static int ov5640_focus_check_mcu(void)
{
	int  retval = 0;
	u8 mcuValue=0,mcuValue2=0,afcValue=0;
	int bit6=0,bit5=0,bit_6=0,bit_5=0,afc_bit6=0,afc_bit=0;

	retval = ov5640_read_reg(0x3000,&mcuValue);
	if (retval >= 0 ){
		bit6 = (mcuValue >> 6)&0x1;
		bit5 = (mcuValue >> 5)&0x1;
	}

	retval = ov5640_read_reg(0x3004,&mcuValue2);
	if (retval >= 0 ){
		bit_6 = (mcuValue2 >> 6)&0x1;
		bit_5 = (mcuValue2 >> 5)&0x1;
	}

	retval = ov5640_read_reg(0x3001,&afcValue);
	if (retval >= 0 ){
		afc_bit6 = (afcValue >> 6)&0x1;
	}
	afcValue=0;
	retval = ov5640_read_reg(0x3005,&afcValue);
	if (retval >= 0 ){
		afc_bit = (afcValue >> 6)&0x1;
	}

	if((bit6==0)&&
	  (bit5==0)&&
	  (bit_6==1)&&
	  (bit_5==1)&&
	  (afc_bit6==0)&&
	  (afc_bit==1)){
	     return 0;
	}else 
	  return -1;

}
static int ov5640_init_auto_focus(void)
{
	struct reg_value *pModeSetting = NULL;
	int ArySize = 0, retval = 0;
	  u8 state;
	  int iteration = 100;

	//////ov5640_soft_reset();

	if (af_download != 0) return 0;
	
	pModeSetting = ov5640_auto_focus_setting;
	ArySize = ARRAY_SIZE(ov5640_auto_focus_setting);
	printk("%s,ArySize=%d,\n",__func__,ArySize);
	retval = ov5640_download_firmware(pModeSetting, ArySize);
	if (retval == 0) {
		printk("AF_DOWNLOAD: OK\n");
	} else {
		goto sensor_af_init_end;
	}

    do {
        ///state = (UINT8)OV5640YUV_read_cmos_sensor(0x3029);
	 retval =ov5640_read_reg(OV5640_STA_FOCUS_Reg,&state);
	mdelay(5);
        if (iteration-- == 0)
        {
            break;
        }
    } while(state!=ov5640_S_IDLE);

	retval = ov5640_focus_check_mcu();
	printk("AF_DOWNLOAD: retval=%d\n", retval);
	if (retval < 0)
		goto sensor_af_init_end;
	af_download = 1;
	return retval;

sensor_af_init_end:
	af_download = -1;
	printk("AF_DOWNLOAD: ERROR\n");
	return retval;
}

static int autofocus_thread(void *p_arg)
{
	int counter;
	u8 v;

	while (1) {
		wait_event_timeout(af_start_wq, af_mode == AF_SINGLESHOT, msecs_to_jiffies(AF_PERIOD));
		if (af_mode == AF_STOPPED || ! is_power_on) continue;
		if (af_download == 0) ov5640_init_auto_focus();
		if (af_download == -1) {
			printk("AF not available\n");
			af_status = V4L2_AUTO_FOCUS_STATUS_FAILED;
			msleep(200);
			continue;
		}
		printk("AF starting...\n");
		af_status = V4L2_AUTO_FOCUS_STATUS_BUSY;
		ov5640_write_reg(0x3022,0x08);
		msleep(100);
		ov5640_write_reg(0x3022,0x03);
		for (counter=35; counter>0; --counter) {
			if (af_mode == AF_STOPPED) break;
			ov5640_read_reg(0x3029,&v);
			if (v == 0x10) {
				printk("AF ok\n");
				break;
			}
			msleep(100);
		}
		if (af_mode != AF_STOPPED) {
			ov5640_write_reg(0x3022,0x06);
		} else {
			ov5640_write_reg(0x3022,0x08);
		}
		if (counter == 0) {
			printk("AF error\n");
		}
		if (af_mode == AF_SINGLESHOT) {
			// fix settings for image capture
			//singleshot_prepare();
			af_mode = AF_STOPPED;
		}
		af_status = (counter == 0) ? V4L2_AUTO_FOCUS_STATUS_FAILED : V4L2_AUTO_FOCUS_STATUS_REACHED;
	}
}	

static void ae_on_work_func(struct work_struct *work)
{
	if (! is_power_on) return;
	printk("AE ON\n");

	ov5640_turn_on_AE_AG(1);
	ov5640_set_bandingfilter();
	ov5640_set_AE_target(AE_Target);
	ov5640_set_night_mode(night_mode);

}


static int ov5640_init_mode(void)
{
	struct reg_value *pModeSetting = NULL;
	int ArySize = 0, retval = 0;

	ov5640_soft_reset();

	pModeSetting = ov5640_global_init_setting;
	ArySize = ARRAY_SIZE(ov5640_global_init_setting);
	retval = ov5640_download_firmware(pModeSetting, ArySize);
	if (retval < 0)
		goto err;

	pModeSetting = ov5640_init_setting_30fps_VGA;
	ArySize = ARRAY_SIZE(ov5640_init_setting_30fps_VGA);
	retval = ov5640_download_firmware(pModeSetting, ArySize);
	if (retval < 0)
		goto err;

	/* turn on overlay */
	//ov5640_write_reg(0x3022, 0x03);
	//printk("Turn on Auto Focus %s,\n",	__func__ );


	/* change driver capability to 2x according to validation board.
	 * if the image is not stable, please increase the driver strength.
	 */
	ov5640_driver_capability(2);
	ov5640_set_bandingfilter();
	ov5640_set_AE_target(AE_Target);
	ov5640_set_night_mode(night_mode);

	/* skip 9 vysnc: start capture at 10th vsync */
	msleep(300);

	/* turn off night mode */
	night_mode = 0;
	ov5640_data.pix.width = 640;
	ov5640_data.pix.height = 480;
	//ov5640_data.pix.width = 2592;///640;
	//ov5640_data.pix.height = 1944;///480;	

err:
	return retval;
}

/* change to scaling mode go through exposure calucation
 * image size above 1280 * 960 is scaling mode */
static int ov5640_change_mode(enum ov5640_frame_rate frame_rate,
			    enum ov5640_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval = 0;
	int i, count;
	u32 tgt_xclk;	/* target xclk */

	long prev_shutter, prev_gain16, average, prev_HTS, prev_VTS;
	long cap_shutter, cap_gain16;
	long cap_sysclk, cap_HTS, cap_VTS;
	long light_freq, cap_bandfilt, cap_maxband;
	long cap_gain16_shutter;
	u8 temp;

	if (frame_rate < 0 || frame_rate > 1 || mode > ov5640_mode_MAX || mode < ov5640_mode_MIN)
		return -EINVAL;

	pModeSetting = ov5640_mode_info_data[frame_rate][mode].init_data_ptr;
	ArySize = ov5640_mode_info_data[frame_rate][mode].init_data_size;
	ov5640_data.pix.width = ov5640_mode_info_data[frame_rate][mode].width;
	ov5640_data.pix.height = ov5640_mode_info_data[frame_rate][mode].height;

	if (ov5640_data.pix.width == 0 || ov5640_data.pix.height == 0 || pModeSetting == NULL || ArySize == 0)
		return -EINVAL;

	printk("OV5640: mode=%d\n", mode);

	//ov5640_init_auto_focus();

	ov5640_current_framerate = frame_rate;
	ov5640_current_mode = mode;

	if (mode == 0) {
		ov5640_set_night_mode(0);
		ov5640_turn_on_AE_AG(0);
		retval = ov5640_download_firmware(pModeSetting, ArySize);
		if (! retval) retval = ov5640_download_firmware(ov5640_awb, ARRAY_SIZE(ov5640_awb));
		if (! retval) retval = ov5640_download_firmware(ov5640_uv_adjust, ARRAY_SIZE(ov5640_uv_adjust));
		for (i=0; i<user_script_len; i++) {
			if (! retval) retval = ov5640_write_reg(user_script[i].reg, user_script[i].value);
		}
		if (retval < 0) goto err;
		return 0;
	}	

	prev_VTS = ov5640_get_VTS();
	prev_HTS = ov5640_get_HTS();
	prev_shutter = ov5640_get_shutter();
	prev_gain16 = ov5640_get_gain16();
	average = ov5640_read_reg(0x56a1, &temp);

	ov5640_set_night_mode(0);
	ov5640_turn_on_AE_AG(0);

	retval = ov5640_download_firmware(pModeSetting, ArySize);
	if (! retval) retval = ov5640_download_firmware(ov5640_awb, ARRAY_SIZE(ov5640_awb));
	if (! retval) retval = ov5640_download_firmware(ov5640_uv_adjust, ARRAY_SIZE(ov5640_uv_adjust));
	for (i=0; i<user_script_len; i++) {
		if (! retval) retval = ov5640_write_reg(user_script[i].reg, user_script[i].value);
	}
	if (retval < 0) goto err;

	cap_VTS = ov5640_get_VTS();
	cap_HTS = ov5640_get_HTS();
	cap_sysclk = ov5640_get_sysclk();

	printk("prev:  HTS=%d VTS=%d S=%d G=%d avg=%d\n", prev_HTS, prev_VTS, prev_shutter, prev_gain16, average);
	printk("cap:   HTS=%d VTS=%d sysclk=%d\n", cap_HTS, cap_VTS, cap_sysclk);

	light_freq = ov5640_get_light_freq();
	if (light_freq == 60) {
		cap_bandfilt = cap_sysclk * 100 / cap_HTS * 100 / 120;
	} else {
		cap_bandfilt = cap_sysclk * 100 / cap_HTS;
	}
	cap_maxband = (int)((cap_VTS - 4)/cap_bandfilt);

	printk("       light_freq=%d cap_bandfilt=%d cap_maxband=%d\n", light_freq, cap_bandfilt, cap_maxband);

	/* calculate capture shutter/gain16 */
	if (average > AE_low && average < AE_high) {
		/* in stable range */
		cap_gain16_shutter =
			(((((cap_sysclk * prev_gain16 * prev_shutter) / prev_sysclk) * prev_HTS) / cap_HTS) * AE_Target) /  average;
		printk("S      cap_gain16_shutter=%d\n", cap_gain16_shutter);
	} else {
		cap_gain16_shutter =
			((((cap_sysclk * prev_gain16 * prev_shutter) / prev_sysclk) * prev_HTS) / cap_HTS);
		printk("U      cap_gain16_shutter=%d\n", cap_gain16_shutter);
	}

	/* gain to shutter */
	if (cap_gain16_shutter < (cap_bandfilt * 16)) {
		/* shutter < 1/100 */
		cap_shutter = cap_gain16_shutter/16;
		if (cap_shutter < 1)
			cap_shutter = 1;
		cap_gain16 = cap_gain16_shutter/cap_shutter;
		if (cap_gain16 < 16)
			cap_gain16 = 16;
	} else {
		if (cap_gain16_shutter > (cap_bandfilt*cap_maxband*16)) {
			/* exposure reach max */
			cap_shutter = cap_bandfilt*cap_maxband;
			cap_gain16 = cap_gain16_shutter / cap_shutter;
		} else {
			/* 1/100 < cap_shutter =< max, cap_shutter = n/100 */
			cap_shutter =
				((int)(cap_gain16_shutter/16/cap_bandfilt))
				* cap_bandfilt;
			cap_gain16 = cap_gain16_shutter / cap_shutter;
		}
	}

	if (cap_shutter > cap_VTS / 4) {
		cap_gain16 = (cap_gain16 * cap_shutter) / (cap_VTS / 4);
		if (cap_gain16 > 63) cap_gain16 = 63;
		cap_shutter = cap_VTS / 4;
	}

	ov5640_set_gain16(cap_gain16);

	/* write capture shutter */
	if (cap_shutter > (cap_VTS - 4)) {
		cap_shutter = cap_VTS - 4;
	}

	ov5640_set_shutter(cap_shutter);

	printk("      gain=%d shutter=%d\n", cap_gain16, cap_shutter);

	/* skip 2 vysnc: start capture at 3rd vsync
	 * frame rate of QSXGA and 1080P is 7.5fps: 1/7.5 * 2
	 */
	//printk("ov5640: the actual frame rate of %s is 7.5fps\n",
	//mode == ov5640_mode_1080P_1920_1080 ? "1080P" : "QSXGA");
	//msleep(267);

err:
	return retval;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = ov5640_data.mclk;
	printk("   clock_curr=mclk=%d\n", ov5640_data.mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = OV5640_XCLK_MIN;
	p->u.bt656.clock_max = OV5640_XCLK_MAX;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

static void camera_power(int on)
{
	u32 tgt_xclk;
	int r;


	printk("camera_power(%d)\n", on);
	if (on && ! is_power_on) {

		mx6sl_setCameraPadVoltage();

		gpio_direction_output(MX6SL_BRD_CSI_RST, 0);
		gpio_direction_output(MX6SL_BRD_CSI_PWDN, 1);
		mdelay(1);

		//enable Vgen3
		if (io_regulator) {
			r = regulator_enable(io_regulator);
		}
		mdelay(0);

		//open key from vgen4 (1V8)
		gpio_direction_output(MX6SL_CAMERA_18V_EN, 0);
		mdelay(1);

		if (core_regulator) {
			r = regulator_enable(core_regulator);
		}

		if (analog_regulator) {
			r = regulator_enable(analog_regulator);
		}
		gpio_direction_output(MX6SL_CAMERA_28V_EN, 1);

		mdelay(10);

		gpio_set_value(MX6SL_BRD_CSI_PWDN, 0);

		mdelay(1);

#ifdef CONFIG_SOC_IMX6SL
		/* mclk */
		tgt_xclk = ov5640_data.mclk;
		tgt_xclk = min(tgt_xclk, (u32)OV5640_XCLK_MAX);
		tgt_xclk = max(tgt_xclk, (u32)OV5640_XCLK_MIN);
		ov5640_data.mclk = tgt_xclk;
		printk("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
		set_mclk_rate(&ov5640_data.mclk, ov5640_data.mclk_source);

		csi_enable_mclk(CSI_MCLK_I2C, true, true);
#endif

		gpio_set_value(MX6SL_BRD_CSI_RST, 1);
		mdelay(20);

		ov5640_download_firmware(ov5640_global_init_setting, ARRAY_SIZE(ov5640_global_init_setting));

		af_download = 0;
	}
	if ((! on) && is_power_on) {
#ifdef CONFIG_SOC_IMX6SL
		csi_enable_mclk(CSI_MCLK_I2C, false, false);
#endif
		gpio_set_value(MX6SL_BRD_CSI_RST, 0);
		gpio_set_value(MX6SL_BRD_CSI_PWDN, 0);
		mdelay(1);

		if (core_regulator) {
			while (regulator_is_enabled(core_regulator)) {
				r = regulator_disable(core_regulator);
				if (r != 0) break;
			}
		}

		if (analog_regulator) {
			while (regulator_is_enabled(analog_regulator)) {
				r = regulator_disable(analog_regulator);
				if (r != 0) break;
			}
		}

		/*Camera 2.8v low close high open*/
		gpio_set_value(MX6SL_CAMERA_28V_EN,0);
		mdelay(50);

		//disable vgen3
		if (io_regulator)
			regulator_disable(io_regulator);

		if (gpo_regulator) {
			while (regulator_is_enabled(gpo_regulator)) {
				r = regulator_disable(gpo_regulator);
				if (r != 0) break;
			}
		}

		/*Camera 1.8V: low open; high close*/
		gpio_set_value(MX6SL_CAMERA_18V_EN,1);
		mdelay(1);
		gpio_direction_input(MX6SL_CAMERA_18V_EN);

		msleep(50);

		af_download = 0;
	}
	is_power_on = on;
}
/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */

static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;
	//printk("%s %s %d on=%d, sensor->on=%d\n",__FILE__,__func__,__LINE__,on,sensor->on); 

	camera_power(on);
	sensor->on = on;
	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;
	printk("%s,a->type=%d,  \n",__func__,a->type);
	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov5640_frame_rate frame_rate;
	int ret = 0;

	/* Make sure power on */
	camera_power(1);
	printk("%s,a->type=%d,  \n",__func__,a->type);
	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;
		printk("%s,tgt_fps=%d,  \n",__func__,tgt_fps);
		if (tgt_fps == 15)
			frame_rate = ov5640_15_fps;
		else if (tgt_fps == 30)
			frame_rate = ov5640_30_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		ret = ov5640_change_mode(frame_rate,
				a->parm.capture.capturemode);
		if (ret < 0)
			return ret;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode = a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		printk("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		printk("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}


/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int ret = 0;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = ov5640_data.brightness;
		break;
	case V4L2_CID_HUE:
		vc->value = ov5640_data.hue;
		break;
	case V4L2_CID_CONTRAST:
		vc->value = ov5640_data.contrast;
		break;
	case V4L2_CID_SHARPNESS:
		vc->value = ov5640_data.sharpness;
		break;
	case V4L2_CID_SATURATION:
		vc->value = ov5640_data.saturation;
		break;
	case V4L2_CID_RED_BALANCE:
		vc->value = ov5640_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		vc->value = ov5640_data.blue;
		break;
	case V4L2_CID_EXPOSURE:
		vc->value = ov5640_data.ae_mode;
		break;
	case V4L2_CID_AUTO_FOCUS_STATUS:
		vc->value = af_status;
		printk("V4L2_CID_AUTO_FOCUS_STATUS=%d\n", af_status);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const u8 contrast_5586[9] = { 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28, 0x2c, 0x30 };
static const u8 contrast_5585[9] = { 0x10, 0x14, 0x18, 0x1c, 0x00, 0x10, 0x18, 0x1c, 0x20 };
static const u8 sharpness_5302[9] = { 0x00, 0x02, 0x04, 0x08, 0x0c, 0x10, 0x14, 0x18, 0x20 };


/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int val = vc->value;
	int retval = 0;

	pr_debug("In ov5640:ioctl_s_ctrl %d\n",
		 vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		if (val < 0) val = 0;
		if (val > 8) val = 8;
		ov5640_data.contrast = val;
		ov5640_write_preg(0x5001, 0x83, INVMASK(0x80));
		ov5640_write_preg(0x5580, 0x04, INVMASK(0x04));
		ov5640_write_reg(0x5586, contrast_5586[val]);
		ov5640_write_reg(0x5585, contrast_5585[val]);
		ov5640_write_preg(0x5588, 0x00, INVMASK(0x04));
		break;
	case V4L2_CID_SHARPNESS:
		if (val < 0) val = 0;
		if (val > 8) val = 8;
		ov5640_data.sharpness = val;
		if (val == 2) { // default
			ov5640_write_preg(0x5308, 0x00, INVMASK(0x40));
			ov5640_write_reg(0x5300, 0x08);
			ov5640_write_reg(0x5301, 0x30);
			ov5640_write_reg(0x5302, 0x10);
			ov5640_write_reg(0x5303, 0x00);
			ov5640_write_reg(0x5309, 0x08);
			ov5640_write_reg(0x530a, 0x30);
			ov5640_write_reg(0x530b, 0x04);
			ov5640_write_reg(0x530c, 0x06);
		} else {
			ov5640_write_preg(0x5308, 0x40, INVMASK(0x40));
			ov5640_write_reg(0x5302, sharpness_5302[val]);
		}
		break;
	case V4L2_CID_SATURATION:
		break;
	case V4L2_CID_HUE:
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_GAMMA:
		break;
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_AUTOGAIN:
		break;
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_HFLIP:
		break;
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_FOCUS_AUTO:
		printk("V4L2_CID_FOCUS_AUTO=%d\n", val);
		if (val) {
			// re-enable auto-exposition
			schedule_delayed_work(&ae_on_work, msecs_to_jiffies(1000));
			// continuous AF
			af_mode = AF_CONTINUOUS;
			af_status = V4L2_AUTO_FOCUS_STATUS_BUSY;
			wake_up(&af_start_wq);
		} else {
			af_mode = AF_STOPPED;
			af_status = V4L2_AUTO_FOCUS_STATUS_IDLE;
		}
		break;
	case V4L2_CID_AUTO_FOCUS_START:
		printk("V4L2_CID_AUTO_FOCUS_START\n");
		if (af_mode != AF_CONTINUOUS) af_mode = AF_SINGLESHOT;
		af_status = V4L2_AUTO_FOCUS_STATUS_BUSY;
		wake_up(&af_start_wq);
		break;
	default:
		retval = -EPERM;
		break;
	}

	return retval;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index > ov5640_mode_MAX)
		return -EINVAL;

	fsize->pixel_format = ov5640_data.pix.pixelformat;
	fsize->discrete.width =
			max(ov5640_mode_info_data[0][fsize->index].width,
			    ov5640_mode_info_data[1][fsize->index].width);
	fsize->discrete.height =
			max(ov5640_mode_info_data[0][fsize->index].height,
			    ov5640_mode_info_data[1][fsize->index].height);
	printk(KERN_INFO "%s:index=%d,fmt=%d,w=%d,h=%d\n", __func__, 
				fsize->index, fsize->pixel_format, fsize->discrete.width, fsize->discrete.height);
	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	int i, j, count;

	if (fival->index < 0 || fival->index > ov5640_mode_MAX)
		return -EINVAL;

	if (fival->pixel_format == 0 || fival->width == 0 || fival->height == 0) {
		pr_warning("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(ov5640_mode_info_data); i++) {
		for (j = 0; j < (ov5640_mode_MAX + 1); j++) {
			if (fival->pixel_format == ov5640_data.pix.pixelformat
			 && fival->width == ov5640_mode_info_data[i][j].width
			 && fival->height == ov5640_mode_info_data[i][j].height
			 && ov5640_mode_info_data[i][j].init_data_ptr != NULL) {
				count++;
			}
			if (fival->index == (count - 1)) {
				fival->discrete.denominator =
						ov5640_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int ioctl_streamon(struct v4l2_int_device *s)
{
	printk("STREAMON");
	cancel_delayed_work_sync(&streamoff_work);
	if (! is_power_on) {
		camera_power(1);
		ov5640_change_mode(ov5640_current_framerate, ov5640_current_mode);
	}
	return 0;
}

static int ioctl_streamoff(struct v4l2_int_device *s)
{
	printk("STREAMOFF");
	af_mode = AF_STOPPED;
	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "ov5640_camera");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > ov5640_mode_MAX)
		return -EINVAL;

	fmt->pixelformat = ov5640_data.pix.pixelformat;

	printk(KERN_INFO "%s:pixel_format = %d\n", __func__, fmt->pixelformat);
	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ov5640_frame_rate frame_rate;
	int ret;

	printk("ioctl_dev_init\n");

	ov5640_data.on = true;

	/* mclk */
	tgt_xclk = ov5640_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV5640_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV5640_XCLK_MIN);
	ov5640_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	set_mclk_rate(&ov5640_data.mclk, ov5640_data.mclk_source);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps == 15)
		frame_rate = ov5640_15_fps;
	else if (tgt_fps == 30)
		frame_rate = ov5640_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	ret = ov5640_init_mode();
	return ret;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

static int ioctl_release(struct v4l2_int_device *s)
{
	printk("RELEASE\n");
	camera_power(0);
	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc ov5640_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_enum_fmt_cap},
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *)ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
	{vidioc_int_streamon_num, (v4l2_int_ioctl_func *)ioctl_streamon},
	{vidioc_int_streamoff_num, (v4l2_int_ioctl_func *)ioctl_streamoff},
	{vidioc_int_release_num, (v4l2_int_ioctl_func *)ioctl_release},
};

static struct v4l2_int_slave ov5640_slave = {
	.ioctls = ov5640_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ov5640_ioctl_desc),
};

static struct v4l2_int_device ov5640_int_device = {
	.module = THIS_MODULE,
	.name = "ov5640",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ov5640_slave,
	},
};

static int reg_addr;

static ssize_t reg_addr_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return  scnprintf(buf, PAGE_SIZE, "%04x\n", reg_addr);
}
static ssize_t reg_addr_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    reg_addr = simple_strtoul(buf, NULL, 16);
    return count;
}
static ssize_t reg_value_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    u8 v;
    return scnprintf(buf, PAGE_SIZE, "%02x\n", ov5640_read_reg(reg_addr, &v) & 0xff);
}

static ssize_t reg_value_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    ov5640_write_reg(reg_addr, simple_strtoul(buf, NULL, 16));
    return count;
}

static ssize_t user_script_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    int i, len=0;
    for (i=0; i<user_script_len; i++) {
	    len += scnprintf(buf+len, PAGE_SIZE-len, "%04x %02x\n", user_script[i].reg, user_script[i].value);
    }
    return len;
}

static ssize_t user_script_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    u16 reg, val;
    char *p = buf;
    while (*p == ' ') p++;
    if (*p < ' ') {
        user_script_len = 0;
    } else {
        if (user_script_len >= MAX_USER_SCRIPT) return 0;
        reg = simple_strtoul(p, &p, 16);
        while (*p == ' ') p++;
        if (isxdigit(*p)) {
            val = simple_strtoul(p, &p, 16);
            user_script[user_script_len].reg = reg;
            user_script[user_script_len].value = val;
            user_script_len++;
        }
    }
    return count;
}

static struct kobj_attribute reg_addr_attribute = __ATTR(reg_addr, 0666, reg_addr_show, reg_addr_store);
static struct kobj_attribute reg_value_attribute = __ATTR(reg_value, 0666, reg_value_show, reg_value_store);
static struct kobj_attribute user_script_attribute = __ATTR(user_script, 0666, user_script_show, user_script_store);

static struct attribute *ov5640_attrs[] = {

    &reg_addr_attribute.attr,
    &reg_value_attribute.attr,
    &user_script_attribute.attr,
    NULL

};

static struct attribute_group ov5640_sysfs_group = {

    .attrs = ov5640_attrs,

};


/*!
 * ov5640 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov5640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int retval;
	struct fsl_mxc_camera_platform_data *plat_data = client->dev.platform_data;
	u8 chip_id_high, chip_id_low;

	/* Set initial values for the sensor struct. */
	memset(&ov5640_data, 0, sizeof(ov5640_data));
	ov5640_data.mclk = 24000000; /* 6 - 54 MHz, typical 24MHz */
	ov5640_data.mclk = plat_data->mclk;
	ov5640_data.mclk_source = plat_data->mclk_source;
	ov5640_data.csi = plat_data->csi;
	ov5640_data.io_init = plat_data->io_init;

	ov5640_data.i2c_client = client;
	ov5640_data.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	ov5640_data.pix.width =640;///2592;
	ov5640_data.pix.height = 480;///;1944
	ov5640_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov5640_data.streamcap.capturemode = 0;
	ov5640_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov5640_data.streamcap.timeperframe.numerator = 1;

	if (plat_data->io_regulator) {
		io_regulator = regulator_get(&client->dev,
					     plat_data->io_regulator);
		if (!IS_ERR(io_regulator)) {
			regulator_set_voltage(io_regulator,
					      OV5640_VOLTAGE_DIGITAL_IO,
					      OV5640_VOLTAGE_DIGITAL_IO);
			if (regulator_enable(io_regulator) != 0) {
				pr_err("%s:io set voltage error\n", __func__);
				goto err1;
			} else {
				dev_dbg(&client->dev,
					"%s:io set voltage ok\n", __func__);
			}
		} else
			io_regulator = NULL;
	}

	if (plat_data->core_regulator) {
		core_regulator = regulator_get(&client->dev,
					       plat_data->core_regulator);
		if (!IS_ERR(core_regulator)) {
			regulator_set_voltage(core_regulator,
					      OV5640_VOLTAGE_DIGITAL_CORE,
					      OV5640_VOLTAGE_DIGITAL_CORE);
			if (regulator_enable(core_regulator) != 0) {
				pr_err("%s:core set voltage error\n", __func__);
				goto err2;
			} else {
				dev_dbg(&client->dev,
					"%s:core set voltage ok\n", __func__);
			}
		} else
			core_regulator = NULL;
	}

	if (plat_data->analog_regulator) {
		analog_regulator = regulator_get(&client->dev,
						 plat_data->analog_regulator);
		if (!IS_ERR(analog_regulator)) {
			regulator_set_voltage(analog_regulator,
					      OV5640_VOLTAGE_ANALOG,
					      OV5640_VOLTAGE_ANALOG);
			if (regulator_enable(analog_regulator) != 0) {
				pr_err("%s:analog set voltage error\n",
					__func__);
				goto err3;
			} else {
				dev_dbg(&client->dev,
					"%s:analog set voltage ok\n", __func__);
			}
		} else
			analog_regulator = NULL;
	}

	if (plat_data->io_init)
		plat_data->io_init();

	camera_plat = plat_data;

	//disable after init
	camera_power(0);

	// DmitryZ: assume camera is present, don't check at startup
/*
	camera_power(1);

	retval = ov5640_read_reg(OV5640_CHIP_ID_HIGH_BYTE, &chip_id_high);
	if (retval < 0 || chip_id_high != 0x56) {
		pr_warning("camera ov5640 is not found\n");
		retval = -ENODEV;
		goto err4;
	}
	retval = ov5640_read_reg(OV5640_CHIP_ID_LOW_BYTE, &chip_id_low);
	if (retval < 0 || chip_id_low != 0x40) {
		pr_warning("camera ov5640 is not found\n");
		retval = -ENODEV;
		goto err4;
	}

	camera_power(0);
*/

	ov5640_int_device.priv = &ov5640_data;
	retval = v4l2_int_device_register(&ov5640_int_device);

	autofocus_task = kthread_create(autofocus_thread, NULL, "ov5640_af");
	wake_up_process(autofocus_task);

	INIT_DELAYED_WORK(&ae_on_work, ae_on_work_func);

	sysfs_create_group(&client->dev.kobj, &ov5640_sysfs_group);

	pr_info("camera ov5640 is found\n");
	return retval;

err4:
	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}
err3:
	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}
err2:
	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}
err1:
	return retval;
}

/*!
 * ov5640 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov5640_remove(struct i2c_client *client)
{
	v4l2_int_device_unregister(&ov5640_int_device);

	if (gpo_regulator) {
		regulator_disable(gpo_regulator);
		regulator_put(gpo_regulator);
	}

	if (analog_regulator) {
		regulator_disable(analog_regulator);
		regulator_put(analog_regulator);
	}

	if (core_regulator) {
		regulator_disable(core_regulator);
		regulator_put(core_regulator);
	}

	if (io_regulator) {
		regulator_disable(io_regulator);
		regulator_put(io_regulator);
	}

	return 0;
}


int core_enabled = 0;

/*!
 * ov5640 I2C detach function
 *  goto  suspend mode
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov5640_suspend(struct i2c_client *client)
{
	int r;

//	core_enabled = regulator_is_enabled(core_regulator);
    camera_power(0);
//	if (core_regulator) {
//		while (regulator_is_enabled(core_regulator)) {
//			r = regulator_disable(core_regulator);
//			if (r != 0) break;
//		}
//	}
	return 0;
}
/*!
 * ov5640 I2C detach function
 *  goto  suspend mode
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov5640_resume(struct i2c_client *client)
{
	//camera_plat = plat_data;
	//ov5640_int_device.priv = &ov5640_data;
	//v4l2_int_device_register(&ov5640_int_device);

//	if (core_regulator && core_enabled) {
//		regulator_enable(core_regulator);
//	}
	 return 0;
}
			
/*!
 * ov5640 init function
 * Called by insmod ov5640_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int ov5640_init(void)
{
	u8 err;

	err = i2c_add_driver(&ov5640_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

/*!
 * OV5640 cleanup function
 * Called on rmmod ov5640_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit ov5640_clean(void)
{
	i2c_del_driver(&ov5640_i2c_driver);
}

module_init(ov5640_init);
module_exit(ov5640_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("OV5640 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
