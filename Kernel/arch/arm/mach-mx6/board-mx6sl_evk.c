/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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

#define DEBUG
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
//#include <linux/wlan_plat.h>
#if defined(CONFIG_REGULATOR_TPS6518x)
#include <linux/mfd/tps6518x.h>
#endif
#if defined(CONFIG_ANATOP_THERMAL)
#include <linux/thermal.h>
#endif
#if defined(CONFIG_SND_SOC_IMX_AIC325X)
#include <linux/mfd/tlv320aic3xxx-core.h>
#endif
#include <sound/pcm.h>
#include <linux/obreey.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6sl.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/imx_rfkill.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6sl_common.h"
#include  <linux/imx6sl_evk/imx6sl_evk_gpio_cfg.h>
#include <linux/proc_fs.h>

#if defined(CONFIG_BATTERY_CW2015)
#include <linux/power/cw2015_battery.h>
#endif

#if defined(CONFIG_TOUCHSCREEN_CNT)
#include <linux/input/cntouch_i2c_ts.h>
#endif

static int tps6518x_regulator_init(struct tps6518x *tps6518x);


#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

#if defined(CONFIG_FB_MXC_SIPIX_PANEL)
static int evk_spdc_sel;
#endif

static void mx6sl_evk_suspend_enter(void);
static void mx6sl_evk_suspend_exit(void);
static int mx6sl_SetPadVoltage(void);

struct clk *extern_audio_root;
#if defined(CONFIG_PB650_DEEP_STANDY)
int iDeepStandy = 1;
#endif

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int __init mx6sl_evk_init_pfuze100(u32 int_gpio);

#if defined(CONFIG_MMC_SDHCI_PLTFM)

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};


static int mx6sl_evk_plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt;
	u32 sd_pads_100mhz_cnt;
	u32 sd_pads_50mhz_cnt;
	//bool bSd4=false;
	//if(index ==3)
	//     printk("%s,index=%d,clock=%d\n",__func__,index,clock);

	switch (index) {
	case 0:
		sd_pads_200mhz = mx6sl_sd1_200mhz;
		sd_pads_100mhz = mx6sl_sd1_100mhz;
		sd_pads_50mhz = mx6sl_sd1_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_sd1_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_sd1_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_sd1_50mhz);
		//bSd4 = false;
		break;
	case 1:
		sd_pads_200mhz = mx6sl_sd2_200mhz;
		sd_pads_100mhz = mx6sl_sd2_100mhz;
		sd_pads_50mhz = mx6sl_sd2_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_sd2_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_sd2_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_sd2_50mhz);
		//bSd4 = false;
		break;
	case 2:
		sd_pads_200mhz = mx6sl_sd3_200mhz;
		sd_pads_100mhz = mx6sl_sd3_100mhz;
		sd_pads_50mhz = mx6sl_sd3_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_sd3_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_sd3_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_sd3_50mhz);
		//bSd4 = false;
		break;
	case 3:
		sd_pads_200mhz = mx6sl_sd4_200mhz;
		sd_pads_100mhz = mx6sl_sd4_100mhz;
		sd_pads_50mhz = mx6sl_sd4_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6sl_sd4_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6sl_sd4_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6sl_sd4_50mhz);
		//if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
		//	return 0;
		//pad_mode = SD_PAD_MODE_HIGH_SPEED;
		//mxc_iomux_v3_setup_multiple_pads(mx6sl_sd4_default_mhz,
		//			ARRAY_SIZE(mx6sl_sd4_default_mhz));
		//bSd4 = true;
		break;
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	//if(index ==3)
	 //   set_wifi_pin_voltage();

	//if(bSd4){
	//	if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
	//		return 0;
	//	pad_mode = SD_PAD_MODE_HIGH_SPEED;
	//	return mxc_iomux_v3_setup_multiple_pads(mx6sl_sd4_default_mhz,
	//				ARRAY_SIZE(mx6sl_sd4_default_mhz));
	//}else{

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
	//}
}

static const struct esdhc_platform_data mx6_evk_sd1_data __initconst = {
	.cd_gpio		= MX6SL_INVALID_GPIO,
	.wp_gpio		= MX6SL_INVALID_GPIO,/*MX6_BRD_SD1_WP,*/
	.support_8bit		= 1,
	.support_18v		= 0,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.always_present = 1,
	.platform_pad_change = mx6sl_evk_plt_sd_pad_change,
};

static const struct esdhc_platform_data mx6_evk_sd2_data __initconst = {
	//.cd_gpio		= MX6_BRD_SD2_CD,
	//.wp_gpio		=MX6_BRD_SD2_WP,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.support_18v		= 1,
	.platform_pad_change = mx6sl_evk_plt_sd_pad_change,
};

static const struct esdhc_platform_data mx6_evk_sd3_data __initconst = {
	.cd_gpio		= MX6_BRD_SD3_CD,
	.wp_gpio		= MX6SL_INVALID_GPIO,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.support_18v		= 1,
	.platform_pad_change = mx6sl_evk_plt_sd_pad_change,
};

static const struct esdhc_platform_data mx6_evk_sd4_data __initconst = {
	.cd_gpio =MX6SL_INVALID_GPIO,
	.wp_gpio =MX6SL_INVALID_GPIO,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.support_18v		= 1,
	.cd_type =ESDHC_CD_CONTROLLER,/* ESDHC_CD_CONTROLLER,*/
	.platform_pad_change = mx6sl_evk_plt_sd_pad_change,
};

static struct regulator_consumer_supply evk_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
	//REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data evk_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(evk_vmmc_consumers),
	.consumer_supplies = evk_vmmc_consumers,
};

static struct fixed_voltage_config evk_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &evk_vmmc_init,
};

static struct platform_device evk_vmmc_reg_devices = {
	.name		= REGULATOR_FIXED_DRIVE_NAME,
	.id		= 0,
	.dev		= {
		.platform_data = &evk_vmmc_reg_config,
	},
};
#endif
#if defined(CONFIG_REGULATOR_TPS6518x)
 //file:1).drives/mfd/tps6518x-core.c
 //      2).drives/misc/tps6518x-utils.c
 //      3).drives/regulator/tps6518x-regulator.c
 //      4).drivers/hwmon/tps6518x-hwmon.c
 //      5).include/linux/mfd/tps6518x.h
static struct regulator_consumer_supply tps6518x_display_consumers[] = {
	{
		/* TPS6518x */
		.supply = EPDC_PMIC_TPS6518X_REG_DISPLAY_NAME,
	},
};

static struct regulator_consumer_supply tps6518x_vcom_consumers[] = {
	{
		/* TPS6518x */
		.supply = EPDC_PMIC_TPS6518X_REG_VCOM_NAME,
	},
};

static struct regulator_consumer_supply tps6518x_v3p3_consumers[] = {
	{
		/* TPS6518x */
		.supply = EPDC_PMIC_TPS6518X_REG_V3P3_NAME,
	},
};
static struct regulator_consumer_supply tps6518x_tmst_consumers[] = {
	{
		/* TPS6518x */
		.supply = EPDC_PMIC_TPS6518X_REG_TMST_NAME,
	},
};

static struct regulator_init_data tps6518x_init_data[] = {
	{
		.constraints = {
			.name = EPDC_PMIC_TPS6518X_REG_DISPLAY_NAME,
			.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps6518x_display_consumers),
		.consumer_supplies = tps6518x_display_consumers,
	},
	{
		.constraints = {
			.name = EPDC_PMIC_TPS6518X_REG_VCOM_NAME,
			.min_uV = mV_to_uV(-4325),
			.max_uV = mV_to_uV(-500),
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps6518x_vcom_consumers),
		.consumer_supplies = tps6518x_vcom_consumers,
	},
	{
		.constraints = {
			.name = EPDC_PMIC_TPS6518X_REG_V3P3_NAME,
			.valid_ops_mask =  REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps6518x_v3p3_consumers),
		.consumer_supplies = tps6518x_v3p3_consumers,
	},
	{
	.constraints = {
		.name = EPDC_PMIC_TPS6518X_REG_TMST_NAME,
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps6518x_tmst_consumers),
		.consumer_supplies = tps6518x_tmst_consumers,
	},	
};
static struct platform_device tps6518x_sensor_device = {
	.name = EPDC_PMIC_SENSOR_DRIVE_NAME,
	.id = 0,
};

static struct platform_device tps6518x_utils_device = {
	.name = EPDC_PMIC_UTILS_DRIVE_NAME,
	.id = 0,
};


static struct tps6518x_platform_data tps6518x_pdata __initdata = {
	/*
	* power sequencing for TPS65180/65181
	*/
	//.pwr_seq0 = 0xe1, /* [Vddh-seq=3 | Vpos-seq=2 | Vee-seq=0 | Vneg-seq=1] */
	.pwr_seq0 = 0xe4,/*[Vddh-up=4 | Vpos-up=2 | Vee-up=1 | Vneg-up=0] */
	.pwr_seq1 = 0x30, /* [Vneg-dly1=3 |Vee-dly0=0]  mSec */
	.pwr_seq2 = 0x33, /* [Vddh-dly3=3 | Vpos-dly2=3] mSec */
	/*+	 * power sequencing for TPS65185/65186+	 */
	//.upseq0 = 0xe1, /* power up sequencing: [Vddh-up=3 | Vpos-up=2 | Vee-up=0 | Vneg-up=1] */
	//.upseq1 = 0x40, /* power up rail delays: [vddh-dly4 = 6ms | Vpos-dly3=3 | Vneg-dly2=3 | Vee-dly1=3] mSec */

	.upseq0 = 0xe4, /* power up sequencing: [Vddh-up=4 | Vpos-up=2 | Vee-up=1 | Vneg-up=0] */
	.upseq1 = 0x55, /* power up rail delays: [vddh-dly4 = 6ms | Vpos-dly3=6 | Vneg-dly2=6 | Vee-dly1=6] mSec */

	//.dwnseq0 = 0x1e, /* power down sequencing: [Vddh-dwn=0 | Vpos-dwn=1 | Vee-dwn=3 | Vneg-dwn=2] */
	//.dwnseq1 = 0xc0, /* power down rail delays: [Vee-dly4=48 | Vneg-dly3=3 | Vpos-dly2=3 | Vddh-dly1=3] mSec */
	//.dwnseq0 = 0x1b, /* sequence : --> VDD --> VPOS --> VEE --> VNEG */
	//.dwnseq1 = 0x02, /* delays   : 6ms     6ms      6ms      6ms     */
	.dwnseq0 = 0x1e, /* sequence : --> VDD --> VPOS --> VNEG --> VEE */
	.dwnseq1 = 0xc2, /* delays   : 48ms     6ms      6ms      6ms     */
	.max_wait = (48+6+6+6), /* maximum time of upseq1 & dwnseq1 */
	.delay_3v3_highv = 1,
	/*+	 * GPIO pins+	 */
	//.gpio_pmic_pwrgood = TPS6518X_EPDC_PWRSTAT,
	//.gpio_pmic_vcom_ctrl = TPS6518X_EPDC_VCOM,
	//.gpio_pmic_wakeup = TPS6518X_EPDC_PMIC_WAKE,
	//.gpio_pmic_intr = TPS6518X_EPDC_PMIC_INT,
	//.gpio_pmic_powerup = TPS6518X_EPDC_PWRCTRL0,
	.gpio_pmic_pwrgood = MX6SL_BRD_EPDC_PWRSTAT,
	.gpio_pmic_vcom_ctrl = MX6SL_BRD_EPDC_VCOM,
	.gpio_pmic_wakeup = MX6SL_BRD_EPDC_PMIC_WAKE,
	.gpio_pmic_powerup = MX6SL_BRD_EPDC_PWRCTRL0,
	.gpio_pmic_intr = MX6SL_BRD_EPDC_PMIC_INT,
	
	/*+	 * tps6518x init+	 */
	.regulator_init = tps6518x_init_data,
	.init = tps6518x_regulator_init,
};

static int tps6518x_regulator_init(struct tps6518x *tps6518x)
{
	struct tps6518x_platform_data *pdata = &tps6518x_pdata;
	int i, ret;

	//printk("%s %s %d   \n",__FILE__,__func__,__LINE__); 

	tps6518x->pwr_seq0 = pdata->pwr_seq0;
	tps6518x->pwr_seq1 = pdata->pwr_seq1;
	tps6518x->pwr_seq2 = pdata->pwr_seq2;
	tps6518x->upseq0 = pdata->upseq0;
	tps6518x->upseq1 = pdata->upseq1;
	tps6518x->dwnseq0 = pdata->dwnseq0;
	tps6518x->dwnseq1 = pdata->dwnseq1;

	tps6518x->max_wait = (3+3+3+6); /* values from upseq1 */

	tps6518x->gpio_pmic_pwrgood = pdata->gpio_pmic_pwrgood;
	tps6518x->gpio_pmic_vcom_ctrl = pdata->gpio_pmic_vcom_ctrl;
	tps6518x->gpio_pmic_wakeup = pdata->gpio_pmic_wakeup;
	tps6518x->gpio_pmic_intr = pdata->gpio_pmic_intr;
	tps6518x->gpio_pmic_powerup = pdata->gpio_pmic_powerup; 


	gpio_request(tps6518x->gpio_pmic_wakeup, "epdc-pmic-wake");
	gpio_direction_output(tps6518x->gpio_pmic_wakeup, 1);

	gpio_request(tps6518x->gpio_pmic_vcom_ctrl, "epdc-vcom");
	gpio_direction_output(tps6518x->gpio_pmic_vcom_ctrl, 0);


	gpio_request(tps6518x->gpio_pmic_powerup, "epdc-pmic-powerup");
	gpio_direction_output(tps6518x->gpio_pmic_powerup,0);

	//gpio_request(tps6518x->gpio_pmic_v3p3, "epdc-v3p3");
	//gpio_direction_output(tps6518x->gpio_pmic_v3p3, 0);

	gpio_request(tps6518x->gpio_pmic_intr, "epdc-pmic-int");
	gpio_direction_input(tps6518x->gpio_pmic_intr);

	gpio_request(tps6518x->gpio_pmic_pwrgood, "epdc-pwrstat");
	gpio_direction_input(tps6518x->gpio_pmic_pwrgood);
      
	
	//tps6518x->vcom_uV = -2100;
	tps6518x->vcom_setup = false;
	tps6518x->init_done = false;

	for (i = 0; i < TPS6518x_NUM_REGULATORS; i++) {
		ret = tps6518x_register_regulator(tps6518x, i,
			&pdata->regulator_init[i]);
		if (ret != 0) {
			printk(KERN_ERR"TPS6518x regulator init failed: %d\n",ret);
			return ret;
		}
	}

	regulator_has_full_constraints();

	return 0;
}
#endif
/*what about this device*/
#if defined(CONFIG_ANATOP_THERMAL)
static const struct anatop_thermal_platform_data
	mx6sl_anatop_thermal_data __initconst = {
			.name = THERMAL_DRIVE_NAME,
	};
#endif

#if defined(CONFIG_SPI_IMX)
static int mx6_evk_spi_cs[] = {
	MX6_BRD_ECSPI1_CS0,
};

static const struct spi_imx_master mx6_evk_spi_data __initconst = {
	.chipselect     = mx6_evk_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6_evk_spi_cs),
};
#endif
#if (defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE))
static struct mtd_partition m25p32_partitions[] = {
	{
		.name	= "bootloader",
		.offset	= 0,
		.size	= 0x00100000,
	}, 
	{
		.name	= "kernel",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data m25p32_spi_flash_data = {
	.name		= "m25p32",
	.parts		= m25p32_partitions,
	.nr_parts	= ARRAY_SIZE(m25p32_partitions),
	.type		= "m25p32",
};

static struct spi_board_info m25p32_spi0_board_info[] __initdata = {
	{
	/* The modalias must be the same as spi device driver name */
	.modalias	= "m25p80",
	.max_speed_hz	= 20000000,
	.bus_num	= 0,
	.chip_select	= 0,
	.platform_data	= &m25p32_spi_flash_data,
	},
};

static void mx6_evk_spi_device_init(void)
{
	spi_register_board_info(m25p32_spi0_board_info,
				ARRAY_SIZE(m25p32_spi0_board_info));
}
#endif
////////////////////////////////////////////////////
///////////codec 
////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////tlv325x
////////////////////////////////////////////////////////
#if defined(CONFIG_SND_SOC_IMX_AIC325X)
static struct imx_ssi_platform_data imx6sl_evk_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data tlv325x_data;

static struct platform_device imx6sl_evk_audio_tlv325x_device = {
	.name =AIC3256_IMX_NAME,
};
static int tlv325x_init(void);
static int tlv325x_clk_enable(int enable)
{
	printk("%s %s %d enable=%d\n",__FILE__,__func__,__LINE__,enable); 
	if (IS_ERR(extern_audio_root)) {
		pr_err("audio clock source is not found!\n");
		return -1;
	}
	if (enable)
		clk_enable(extern_audio_root);
	else
		clk_disable(extern_audio_root);

	return 0;
}

static int tlv325x_init(void)
{
	struct clk *pll4;
	int rate;

	//printk("%s %s %d \n",__FILE__,__func__,__LINE__); 

	extern_audio_root = clk_get(NULL, "extern_audio_clk");
	if (IS_ERR(extern_audio_root)) {
		pr_err("can't get extern_audio_root clock.\n");
		return PTR_ERR(extern_audio_root);
	}

	pll4 = clk_get(NULL, "pll4");
	if (IS_ERR(pll4)) {
		pr_err("can't get pll4 clock.\n");
		return PTR_ERR(pll4);
	}

	clk_set_parent(extern_audio_root, pll4);

	rate = clk_round_rate(extern_audio_root,11289600/*12000000 26000000*/);
	clk_set_rate(extern_audio_root, rate);

	tlv325x_data.sysclk = rate;
	/* set AUDMUX pads to 1.8v */
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_MCLK,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_RXD,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_TXC,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_TXD,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_TXFS,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);

	return 0;
}
static struct mxc_audio_platform_data tlv325x_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = MX6SL_BRD_TLV320_HEADPHONE_DET,
	.hp_active_low = 1,
	.mic_gpio = -1,
	.mic_active_low = 1,
	.init = tlv325x_init,
	.clock_enable = tlv325x_clk_enable,
};
static struct aic3xxx_pdata aic325x_config_data={
	//.audio_mclk1=,
	//.audio_mclk2=tlv325x_clk_enable,
	/* whether AIC3262 interrupts the host AP on 
	 * a GPIO pin of AP 
	 */
	//.gpio_irq=,

	/* is the codec being reset by a gpio
	 * [host] pin, if yes provide the number. 
	*/
	.gpio_reset=MX6SL_BRD_AIC325X_RESET,
       // .gpio_spken;
	//.num_gpios=1,
	/* all gpio configuration */
	//struct aic3xxx_gpio_setup *gpio_defaults;

	//.naudint_irq,	/* audio interrupt */
	//.rq_base,
};

static int __init mx6_evk_init_tlv325x(void)
{
	mxc_register_device(&imx6sl_evk_audio_tlv325x_device,&tlv325x_data);
	imx6q_add_imx_ssi(1, &imx6sl_evk_ssi_pdata);

	return 0;
}
#endif
////////////////////////////////////////////////////////
////CW2015
/////////////////////////////////////////////////////////
#if defined(CONFIG_BATTERY_CW2015)
/*
   note the follow array must set depend on the battery that you use
   you must send the battery to cellwise-semi the contact information:
   name: chen gan; tel:13416876079; E-mail: ben.chen@cellwise-semi.com
 */

static u8 config_info[SIZE_BATINFO] = {
	/*0x15, 0x42, 0x60, 0x59, 0x52,
	0x58, 0x4D, 0x48, 0x48, 0x44,
	0x44, 0x46, 0x49, 0x48, 0x32,
	0x24, 0x20, 0x17, 0x13, 0x0F,
	0x19, 0x3E, 0x51, 0x45, 0x08,
	0x76, 0x0B, 0x85, 0x0E, 0x1C,
	0x2E, 0x3E, 0x4D, 0x52, 0x52,
	0x57, 0x3D, 0x1B, 0x6A, 0x2D,
	0x25, 0x43, 0x52, 0x87, 0x8F,
	0x91, 0x94, 0x52, 0x82, 0x8C,
	0x92, 0x96, 0xFF, 0x7B, 0xBB,
	0xCB, 0x2F, 0x7D, 0x72, 0xA5,
	0xB5, 0xC1, 0x46, 0xAE*/
	//3.4V -> 0%
	/*
	0x15,0x7E,0x62,0x68,0x63,
	0x63,0x5A,0x54,0x53,0x4F,
	0x4A,0x45,0x42,0x36,0x22,
	0x18,0x15,0x13,0x17,0x28,
	0x3D,0x69,0x7A,0x6E,0x64,
	0x56,0x0B,0x85,0x0C,0x17,
	0x37,0x45,0x46,0x41,0x37,
	0x71,0x40,0x1A,0x3A,0x2D,
	0x00,0x4C,0x52,0x87,0x8F,
	0x91,0x94,0x52,0x82,0x8C,
	0x92,0x96,0x80,0xE0,0xFF,
	0xCB,0x2F,0x7D,0x72,0xA5,
	0xB5,0xC1,0x02,0x19
	*/
	//3.6V -> 0%
	0x15,0x7E,0x58,0x61,
	0x59,0x5C,0x56,0x51,
	0x4E,0x4C,0x49,0x46,
	0x41,0x3E,0x38,0x2B,
	0x1C,0x15,0x14,0x11,
	0x16,0x17,0x24,0x36,
	0x60,0x72,0x0E,0x15,
	0x29,0x49,0x45,0x3D,
	0x3F,0x4F,0x64,0x81,
	0x3B,0x0E,0x3B,0x00,
	0x00,0x53,0x52,0x87,
	0x8F,0x91,0x94,0x52,
	0x82,0x8C,0x92,0x96,
	0x80,0xF0,0xFF,0xCB,
	0x2F,0x7D,0x72,0xA5,
	0xB5,0xC1,0x46,0xAE
};

static struct cw_bat_platform_data cw_bat_platdata = {
	.dc_det_pin      = MX6SL_INVALID_GPIO,
        .dc_det_level    = MX6SL_GPIO_LOW,

        .bat_low_pin    = MX6SL_BRD_CW2015_LOWBATTERY_N,
        .bat_low_level  = MX6SL_GPIO_LOW,   

	.chg_ok_pin   = MX6SL_INVALID_GPIO,
        .chg_ok_level = MX6SL_GPIO_HIGH,

        .is_usb_charge = 0,
        .chg_mode_sel_pin = MX6SL_INVALID_GPIO,
        .chg_mode_sel_level = MX6SL_GPIO_HIGH,

        .cw_bat_config_info     = config_info,

};
static int __init cw2015_gpio_init(void)
{

	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_cw2015_pads,
					ARRAY_SIZE(mx6sl_brd_cw2015_pads));


     return 0;
}
#endif
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
#if defined(CONFIG_LEDS_OZ8556)
static struct platform_device leds_oz8556_device = {
	.name = "oz8556-leds",
	.id = 0,
};
#endif
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
#if defined(CONFIG_IMX_USB_CHARGER)
#ifdef CONFIG_IMX_USB_CHARGER_CURMON
void charger_current_wantage_monitor_start(void);
void charger_current_wantage_monitor_stop(void);
static void switch_to_low_current_w(struct work_struct *w);
#endif //CONFIG_IMX_USB_CHARGER_CURMON
/////////////////////////////////////////////////////////
////init charger type switch gpio control
///charger type is DC-DC ,set as high
///charger type is USB(connect to PC),set as low
///default,set to low
///////////////////////////////////////////////////////
static int __init  mx6_evk_charger_type_gpio_init(void)
{
	int ret = 0;

	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_chargertype_pads,
					ARRAY_SIZE(mx6sl_brd_chargertype_pads));

	//init as gpio
	ret = gpio_request(MX6SL_CHARGE_CURRENT_SET, "charger-cur-set");
	if (ret) {
		pr_err("failed to get GPIO[MX6SL_BRD_CHARGE_TYPE]:%d\n", ret);
		return ret;
	}

	gpio_direction_output(MX6SL_CHARGE_CURRENT_SET, MX6SL_GPIO_LOW);

	return ret;
}

int mx6_evk_charger_type_gpio_control(bool bEnable)
{
	printk("%s %s %d  bEnable=%d \n",__FILE__,__func__,__LINE__,bEnable);
     /*for new hardware,when usb connect to Adapter,set to high,connect to PC,set to low*/

	if(bEnable) {
		gpio_set_value(MX6SL_CHARGE_CURRENT_SET,MX6SL_GPIO_HIGH);
#ifdef CONFIG_IMX_USB_CHARGER_CURMON
		//add wantage charging current monitoring
		charger_current_wantage_monitor_start();
#endif //CONFIG_IMX_USB_CHARGER_CURMON
	}
	else {
		gpio_set_value(MX6SL_CHARGE_CURRENT_SET,MX6SL_GPIO_LOW);
#ifdef CONFIG_IMX_USB_CHARGER_CURMON
		//disable current error monitoring
		charger_current_wantage_monitor_stop();
#endif //CONFIG_IMX_USB_CHARGER_CURMON
	}

	return 0;
}
EXPORT_SYMBOL(mx6_evk_charger_type_gpio_control);

#ifdef CONFIG_IMX_USB_CHARGER_CURMON
//monitor for detect wantage charger current
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

DEFINE_MUTEX(ccwmon_m);
DECLARE_WORK(ccwmon_w,switch_to_low_current_w);

static struct ccwmon_data_s{
	int gpio;
	unsigned long jiffies_start;
	unsigned long jiffies_bf_signal;
	int count;
	struct mutex *m;
	struct work_struct *w;
	bool active;
} ccwmon_data = {
		//init some fields of struct
		.m = &ccwmon_m,
		.w = &ccwmon_w,
		.gpio = MX6SL_FEC_REF_CLK_CHARGER_STATE,
};

irqreturn_t full_bat_pin_handler(int n, void *d) {
	struct ccwmon_data_s *cd = (struct ccwmon_data_s *)d;

	if (!cd->active) {
		pr_warn("[%s] wrong data state!\n",__func__);
		return IRQ_HANDLED;
	}

	int pv = gpio_get_value(cd->gpio);

	if (pv) {
		cd->jiffies_bf_signal = jiffies;
		if ((jiffies - cd->jiffies_start) > 10*HZ)
			cd->count = 0;

	} else {
		cd->jiffies_start = jiffies;
		//check time between bat full and start charge signals
		if ((jiffies - cd->jiffies_bf_signal) < HZ/2)
			cd->count++;
		else
			cd->count = 0;
	}

	//if we have more then 4 "charge full" signal pulses in short time period,
	//that AC charger adapter is not power
	if (cd->count > 4)
		schedule_work(cd->w);

	pr_info("[%s] n=%i; d=%p; count=%i; js=%i; jf=%i;\n",__func__,n,d,cd->count,cd->jiffies_start,cd->jiffies_bf_signal);

	return IRQ_HANDLED;
}

static void switch_to_low_current_w(struct work_struct *w) {
	mx6_evk_charger_type_gpio_control(false);
}


void charger_current_wantage_monitor_start(void) {
	int ret = 0;
	pr_info("[%s]\n",__func__);

	mutex_lock(ccwmon_data.m);

	if (ccwmon_data.active)
		goto unlock;

	//init as gpio
	ret = gpio_request(ccwmon_data.gpio, "full_bat_pin");
	if (ret) {
		pr_err("failed to get MX6SL_FEC_REF_CLK_CHARGER_STATE; %d\n", ret);
		goto unlock;
	}

	gpio_direction_input(ccwmon_data.gpio);

	//init data struct
	ccwmon_data.count = 0;
	ccwmon_data.jiffies_start = jiffies;
	ccwmon_data.jiffies_bf_signal = 0;

	ret = request_irq(gpio_to_irq(ccwmon_data.gpio),full_bat_pin_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"full_bat_irq",&ccwmon_data);
	if (ret) {
		pr_err("[%s] request irq error = %i\n",__func__,ret);
		gpio_free(ccwmon_data.gpio);
		goto unlock;
	}

	ccwmon_data.active = true;

unlock:
	mutex_unlock(ccwmon_data.m);
}

void charger_current_wantage_monitor_stop(void) {
	pr_info("[%s]\n",__func__);

	mutex_lock(ccwmon_data.m);

	if (!ccwmon_data.active)
		goto unlock;

	free_irq(gpio_to_irq(ccwmon_data.gpio),&ccwmon_data);
	gpio_free(ccwmon_data.gpio);

	ccwmon_data.active = false;

unlock:
	mutex_unlock(ccwmon_data.m);
}
#endif //CONFIG_IMX_USB_CHARGER_CURMON
#endif
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
#if defined(CONFIG_SND_SOC_MXC_SPDIF)
static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,
	.spdif_rx		= 0,
	.spdif_clk_44100	= 1,
	.spdif_clk_48000	= -1,
	.spdif_div_44100	= 23,
	.spdif_clk_set_rate	= spdif_clk_set_rate,
	.spdif_clk		= NULL,
};

#endif
/////////////////////////////////////
////////////////////////////////////
#if defined(CONFIG_FB_MXC_SII902X)

int hdmi_enabled;
static int __init hdmi_setup(char *__unused)
{
	hdmi_enabled = 1;
	return 1;
}
__setup("hdmi", hdmi_setup);

static iomux_v3_cfg_t mx6sl_sii902x_hdmi_pads_enabled[] = {
	MX6SL_PAD_LCD_RESET__GPIO_2_19,
	MX6SL_PAD_EPDC_PWRCTRL3__GPIO_2_10,
};

static int sii902x_get_pins(void)
{
	/* Sii902x HDMI controller */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_sii902x_hdmi_pads_enabled, \
		ARRAY_SIZE(mx6sl_sii902x_hdmi_pads_enabled));

	/* Reset Pin */
	gpio_request(MX6_BRD_LCD_RESET, "disp0-reset");
	gpio_direction_output(MX6_BRD_LCD_RESET, 1);

	/* Interrupter pin GPIO */
	gpio_request(MX6SL_BRD_EPDC_PWRCTRL3, "disp0-detect");
	gpio_direction_input(MX6SL_BRD_EPDC_PWRCTRL3);
       return 1;
}

static void sii902x_put_pins(void)
{
	gpio_free(MX6_BRD_LCD_RESET);
	gpio_free(MX6SL_BRD_EPDC_PWRCTRL3);
}

static void sii902x_hdmi_reset(void)
{
	gpio_set_value(MX6_BRD_LCD_RESET, 0);
	msleep(10);
	gpio_set_value(MX6_BRD_LCD_RESET, 1);
	msleep(10);
}

static struct fsl_mxc_lcd_platform_data sii902x_hdmi_data = {
       .ipu_id = 0,
       .disp_id = 0,
       .reset = sii902x_hdmi_reset,
       .get_pins = sii902x_get_pins,
       .put_pins = sii902x_put_pins,
};
#endif
#if defined(CONFIG_VIDEO_MXC_CSI_CAMERA)
static int __init csi_setup(char *__unused)
{
	csi_enabled = 1;
	return 1;
}
__setup("csi", csi_setup);

/*Set Pad to 1.8v*/
int mx6sl_setCameraPadVoltage(void)
{
	printk("%s,%s,%d \n",__FILE__,__func__,__LINE__);
       /* EPDC_D10  CAM_1.8V_EN		1.8	OUTPUT*/
	mxc_iomux_set_specialbits_register(MX6SL_PAD_EPDC_D10,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	 /*LCD_RESET	CAM_RESET 1.8 */
	mxc_iomux_set_specialbits_register(MX6SL_PAD_LCD_RESET,
				PAD_CTL_LVE, PAD_CTL_LVE_MASK);

	mxc_iomux_set_specialbits_register(MX6SL_PAD_ECSPI2_MISO,
				PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_ECSPI2_SCLK,
				PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_ECSPI2_SS0,
				PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_ECSPI2_MOSI,
				PAD_CTL_LVE, PAD_CTL_LVE_MASK);

  	mxc_iomux_set_specialbits_register(MX6SL_PAD_SD2_CLK,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK); 
  	mxc_iomux_set_specialbits_register(MX6SL_PAD_SD2_CMD,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK); 
  	mxc_iomux_set_specialbits_register(MX6SL_PAD_SD2_DAT0,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK); 
  	mxc_iomux_set_specialbits_register(MX6SL_PAD_SD2_DAT1,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK); 
  	mxc_iomux_set_specialbits_register(MX6SL_PAD_SD2_DAT2,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK); 
  	mxc_iomux_set_specialbits_register(MX6SL_PAD_SD2_DAT3,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK); 
  	mxc_iomux_set_specialbits_register(MX6SL_PAD_SD2_DAT4,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK); 
  	mxc_iomux_set_specialbits_register(MX6SL_PAD_SD2_DAT5,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK); 
  	mxc_iomux_set_specialbits_register(MX6SL_PAD_SD2_DAT6,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK); 
  	mxc_iomux_set_specialbits_register(MX6SL_PAD_SD2_DAT7,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK); 

	return 0;

}
EXPORT_SYMBOL(mx6sl_setCameraPadVoltage);

static void mx6sl_csi_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_csi_enable_pads,	\
				ARRAY_SIZE(mx6sl_brd_csi_enable_pads));

        printk("%s,%s,%d \n",__FILE__,__func__,__LINE__);
//#if defined(CONFIG_XRZ_IMX6SL_PB650)		
	mx6sl_setCameraPadVoltage();
//#endif
	/* Camera reset */
	gpio_request(MX6SL_BRD_CSI_RST, "cam-reset");
	gpio_direction_output(MX6SL_BRD_CSI_RST, 0);

	/* Camera power down */
	gpio_request(MX6SL_BRD_CSI_PWDN, "cam-pwdn");
	gpio_direction_output(MX6SL_BRD_CSI_PWDN, 0);
	msleep(5);
//	gpio_set_value(MX6SL_BRD_CSI_PWDN, 0);
//	msleep(5);
//	gpio_set_value(MX6SL_BRD_CSI_RST, 0);
//	msleep(1);
//	gpio_set_value(MX6SL_BRD_CSI_RST, 1);
//	msleep(5);
//	gpio_set_value(MX6SL_BRD_CSI_PWDN, 1);

	/*Camera 2.8v low close high open*/
	gpio_request(MX6SL_CAMERA_28V_EN, "cam-2.8v");
	gpio_direction_output(MX6SL_CAMERA_28V_EN, 0);

// not use
//	/*Camera 1.5V*/
//	gpio_request(MX6SL_CAMERA_15V_EN, "cam-1.5v");
//	gpio_direction_output(MX6SL_CAMERA_15V_EN, 1);

	/*Camera 1.8V: low open; high close*/
	gpio_request(MX6SL_CAMERA_18V_EN, "cam-1.8v");
	gpio_direction_output(MX6SL_CAMERA_18V_EN, 1);

	/*Camera  FLASH_LED_CTL*/
	//gpio_request(MX6SL_CAMERA_LED_CTL, "cam-led-ctl");
	//gpio_direction_output(MX6SL_CAMERA_LED_CTL, 0);

	/*Camera TORCH_CTL*/
	//gpio_request(MX6SL_CAMERA_TORCH_CTL, "cam-torch");
	//gpio_direction_output(MX6SL_CAMERA_TORCH_CTL, 1);

	//gpio_set_value(MX6SL_BRD_CSI_PWDN, 0);

	
}

static void mx6sl_csi_cam_powerdown(int powerdown)
{
	//printk("%s,%s,%d powerdown=%d\n",__FILE__,__func__,__LINE__,powerdown);
	printk("CAM_POWERDOWN(%d)\n", powerdown);
	if (powerdown){/*power off the camere*/
		gpio_set_value(MX6SL_BRD_CSI_PWDN, 1);
		msleep(2);
		/*Camera 2.8v low close high open*/
		gpio_direction_output(MX6SL_CAMERA_28V_EN,0);
		/*Camera 1.8V: low open; high close*/
		gpio_direction_output(MX6SL_CAMERA_18V_EN, 1);
		msleep(1);
		gpio_direction_input(MX6SL_CAMERA_18V_EN);

	    gpio_set_value(MX6SL_BRD_CSI_PWDN, 0);
	    gpio_set_value(MX6SL_BRD_CSI_RST, 0);
	}else{
		mx6sl_setCameraPadVoltage();
		gpio_set_value(MX6SL_BRD_CSI_PWDN, 1);
		mdelay(1);
		gpio_set_value(MX6SL_BRD_CSI_RST, 0);
		msleep(5);

		gpio_direction_output(MX6SL_CAMERA_18V_EN, 0);
		gpio_direction_output(MX6SL_CAMERA_28V_EN, 1);

		gpio_set_value(MX6SL_BRD_CSI_RST, 1);
		msleep(2);
		gpio_set_value(MX6SL_BRD_CSI_PWDN, 0);
		msleep(25);

	}

}

static struct fsl_mxc_camera_platform_data evk_camera_data = {
	.mclk = 24000000,
	.io_init = mx6sl_csi_io_init,
	.pwdn = mx6sl_csi_cam_powerdown,
	.core_regulator = "VGEN2_1V5",
	//.analog_regulator = "VGEN6_2V8",
	.io_regulator = "VGEN3_1V8",
};

/*! Device Definition for csi v4l2 device */
static struct platform_device csi_v4l2_devices = {
	.name = "csi_v4l2",
	.id = 0,
};
#endif
#if defined(CONFIG_VIDEO_CSI_CAMERA_FLASH_LED)
static struct platform_device  sgm3140b_device = {
	.name = "sgm3140b",
	.id = 0,
};
#endif

////////////////////////////////////////////////////////////
////////  cnt touch
///////////////////////////////////////////////////////////
#if  defined(CONFIG_TOUCHSCREEN_CNT)
static int __init imx6sl_cnt_ts_init(void)
{
	int ret = 0;
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_cnt_pads,
		ARRAY_SIZE(mx6sl_brd_cnt_pads));

	///touch power
	ret = gpio_request(MX6SL_KEY_ROW6_TOUCH_PWR_EN, "ntv-power");	 
	if (ret) {	   
		printk("failed to request cnt power gpio\n");	
		goto error;	
	}	

	gpio_direction_output(MX6SL_KEY_ROW6_TOUCH_PWR_EN, MX6SL_GPIO_LOW);




	///touch rst
	ret = gpio_request(MX6SL_KEY_COL4_TOUCH_RST, "ntv-rst");
	if (ret) {	   
		printk("failed to request cnt reset  gpio\n");	
		goto error;	
	}	


	gpio_direction_output(MX6SL_KEY_COL4_TOUCH_RST, MX6SL_GPIO_HIGH);

	return 0;

error:
	return -1;
}
static int ts_cnt_power(int enable)
{
	int ret = 0;
	//printk("%s %s %d,enable=%d \n",__FILE__,__func__,__LINE__,enable);
	if(enable) {
		gpio_direction_output(MX6SL_KEY_ROW6_TOUCH_PWR_EN, MX6SL_GPIO_HIGH);
		msleep(50); // deassert reset after powerup
		gpio_direction_output(MX6SL_KEY_COL4_TOUCH_RST, MX6SL_GPIO_HIGH);
	} else {
		// assert reset before powerdown
		gpio_direction_output(MX6SL_KEY_COL4_TOUCH_RST, MX6SL_GPIO_LOW);
		gpio_direction_output(MX6SL_KEY_ROW6_TOUCH_PWR_EN, MX6SL_GPIO_LOW);	
	}

	return 0;
}
static int ts_cnt_init(void)
{
	int ret = 0;
	///touch int
	ret = gpio_request(MX6SL_KEY_COL5_TOUCH_INT, "ntv-int");	 
	if (ret) {	   
		printk("failed to request cnt  int  gpio\n");	
		return ret;	
	}	
	gpio_direction_input(MX6SL_KEY_COL5_TOUCH_INT);

	return 0;
}
static int ts_cnt_reset(void)
{

	gpio_direction_output(MX6SL_KEY_COL4_TOUCH_RST, MX6SL_GPIO_HIGH);
	msleep(100);	
	gpio_direction_output(MX6SL_KEY_COL4_TOUCH_RST, MX6SL_GPIO_LOW);
	msleep(200);
	gpio_direction_output(MX6SL_KEY_COL4_TOUCH_RST, MX6SL_GPIO_HIGH);
	msleep(100);

	return 0;
}

struct cnt_i2c_platform_data ts_cnt_i2c_data={
	.power=ts_cnt_power,
	.init = ts_cnt_init,
	.reset=ts_cnt_reset,
};
#endif

static struct imxi2c_platform_data mx6_evk_i2c0_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_evk_i2c1_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_evk_i2c2_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
#if defined(CONFIG_REGULATOR_TPS6518x)
	 {
	     I2C_BOARD_INFO(EPDC_PMIC_DRIVE_NAME, EPDC_PMIC_I2C_ADDR),
	     .platform_data = &tps6518x_pdata,
	  },
#endif
#if defined(CONFIG_BATTERY_CW2015)
	 {
		 I2C_BOARD_INFO(CW2015_BATTERY_DRIVE_NAME, CW2015_BATTERY_SLAVE_ADD),
                .flags          = 0,
                .platform_data  = &cw_bat_platdata,
        },
#endif
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
#if defined(CONFIG_TOUCHSCREEN_CNT)
	{
                I2C_BOARD_INFO(CNT_NAME, I2C_CNT_ADDRESS),
                .platform_data = &ts_cnt_i2c_data,
                .irq =gpio_to_irq(MX6SL_KEY_COL5_TOUCH_INT),
        },

#endif	
#if defined(CONFIG_FB_MXC_SII902X)
	{
		I2C_BOARD_INFO("sii902x", 0), /*0x39*/
		.platform_data = &sii902x_hdmi_data,
		.irq = gpio_to_irq(MX6SL_BRD_EPDC_PWRCTRL3)
	},
#endif
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
#if defined(CONFIG_SND_SOC_IMX_AIC325X)
	{
		I2C_BOARD_INFO(AIC3256_DRIVER_NAME, AIC3256_DRIVER_I2C_ADDRESS),
		.platform_data = &aic325x_config_data,
	},
#endif	
#if defined(CONFIG_MXC_CAMERA_OV5640)
	{
		I2C_BOARD_INFO("ov5640", 0x3c),
		.platform_data = (void *)&evk_camera_data,
	},
#endif	
};

static struct mxc_dvfs_platform_data mx6sl_evk_dvfscore_data = {
	.reg_id			= "VDDCORE",
	.soc_id			= "VDDSOC",
	.clk1_id		= "cpu_clk",
	.clk2_id		= "gpc_dvfs_clk",
	.gpc_cntr_offset	= MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset	= MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset	= MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset	= MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask		= 0x1F800,
	.prediv_offset		= 11,
	.prediv_val		= 3,
	.div3ck_mask		= 0xE0000000,
	.div3ck_offset		= 29,
	.div3ck_val		= 2,
	.emac_val		= 0x08,
	.upthr_val		= 25,
	.dnthr_val		= 9,
	.pncthr_val		= 33,
	.upcnt_val		= 10,
	.dncnt_val		= 10,
	.delay_time		= 80,
};
#if defined(CONFIG_MXC_GPU_VIV)
static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_64M,//SZ_32M,
};
#endif
//////////////////////////////////////////////////
///
//////////////////////////////////////////////////
void __init early_console_setup(unsigned long base, struct clk *clk);
//////////////////////////////////////////////////
///
//////////////////////////////////////////////////

//////////////////////////////////////////////////
///
//////////////////////////////////////////////////
#if defined(CONFIG_SERIAL_IMX)

static inline void mx6_evk_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL); /* DEBUG UART1 */
}
#endif
//////////////////////////////////////////////////
///
//////////////////////////////////////////////////
#if defined(CONFIG_BT)
#ifdef SXSDMAN_BLUETOOTH_ENABLE
static int uart4_enabled;
static const struct imxuart_platform_data mx6sl_evk_uart4_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS,
	.dma_req_rx = MX6Q_DMA_REQ_UART4_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART4_TX,
};
static int __init uart4_setup(char * __unused)
{
	uart4_enabled = 1;
	return 1;
}
__setup("bluetooth", uart4_setup);

static void __init uart4_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_uart4_pads,
					ARRAY_SIZE(mx6sl_uart4_pads));
	imx6sl_add_imx_uart(3, &mx6sl_evk_uart4_data);
}
#else
static int uart2_enabled;
static const struct imxuart_platform_data mx6sl_evk_uart1_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS | IMXUART_SDMA,
	.dma_req_rx = MX6Q_DMA_REQ_UART2_RX,
	.dma_req_tx = MX6Q_DMA_REQ_UART2_TX,
};

static int __init uart2_setup(char * __unused)
{
	uart2_enabled = 1;
	return 1;
}
__setup("bluetooth", uart2_setup);

static void __init uart2_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_uart2_pads,
					ARRAY_SIZE(mx6sl_uart2_pads));
	imx6sl_add_imx_uart(1, &mx6sl_evk_uart1_data);
}
#endif

static void mx6sl_evk_bt_reset(void)
{
	gpio_request(MX6SL_BRD_BT_RESET, "bt-reset");
	gpio_direction_output(MX6SL_BRD_BT_RESET, 0);
	/* pull down reset pin at least >5ms */
	mdelay(6);
	/* pull up after power supply BT */
	gpio_set_value(MX6SL_BRD_BT_RESET, 1);
	gpio_free(MX6SL_BRD_BT_RESET);
}

static int mx6sl_evk_bt_power_change(int status)
{
	if (status)
		mx6sl_evk_bt_reset();
	return 0;
}

static struct platform_device mxc_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct imx_bt_rfkill_platform_data mxc_bt_rfkill_data = {
	.power_change = mx6sl_evk_bt_power_change,
};

#endif
//////////////////////////////////////////////////
///
//////////////////////////////////////////////////


//////////////////////////////////////////////////
///
//////////////////////////////////////////////////
#if defined(CONFIG_PHYLIB)
static int mx6sl_evk_fec_phy_init(struct phy_device *phydev)
{
	int val;

	/* power on FEC phy and reset phy */
	//gpio_request(MX6_BRD_FEC_PWR_EN, "fec-pwr");
	//gpio_direction_output(MX6_BRD_FEC_PWR_EN, 0);
	/* wait RC ms for hw reset */
	//msleep(1);
	//gpio_direction_output(MX6_BRD_FEC_PWR_EN, 1);

	/* check phy power */
	val = phy_read(phydev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6sl_evk_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RMII,
};
#endif
//////////////////////////////////////////////////
///
//////////////////////////////////////////////////
#if defined(CONFIG_FB_MXC_EINK_PANEL)
//static void imx6_evk_setup_epdc(void)
//{
//	/* GPR0[8]: 0:EPDC, 1:SPDC */
//	mxc_iomux_set_gpr_register(0, 8, 1, 0);
//}

static int epdc_get_pins(void)
{
	int ret = 0;

	/* Claim GPIOs for EPDC pins - used during power up/down */
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_0, "epdc_d0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_1, "epdc_d1");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_2, "epdc_d2");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_3, "epdc_d3");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_4, "epdc_d4");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_5, "epdc_d5");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_6, "epdc_d6");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_7, "epdc_d7");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDCLK, "epdc_gdclk");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDSP, "epdc_gdsp");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDOE, "epdc_gdoe");
	ret |= gpio_request(MX6SL_BRD_EPDC_GDRL, "epdc_gdrl");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCLK, "epdc_sdclk");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDOE, "epdc_sdoe");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDLE, "epdc_sdle");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDSHR, "epdc_sdshr");
	ret |= gpio_request(MX6SL_BRD_EPDC_BDR0, "epdc_bdr0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE0, "epdc_sdce0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE1, "epdc_sdce1");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE2, "epdc_sdce2");

	return ret;
}

static void epdc_put_pins(void)
{
	gpio_free(MX6SL_BRD_EPDC_SDDO_0);
	gpio_free(MX6SL_BRD_EPDC_SDDO_1);
	gpio_free(MX6SL_BRD_EPDC_SDDO_2);
	gpio_free(MX6SL_BRD_EPDC_SDDO_3);
	gpio_free(MX6SL_BRD_EPDC_SDDO_4);
	gpio_free(MX6SL_BRD_EPDC_SDDO_5);
	gpio_free(MX6SL_BRD_EPDC_SDDO_6);
	gpio_free(MX6SL_BRD_EPDC_SDDO_7);
	gpio_free(MX6SL_BRD_EPDC_GDCLK);
	gpio_free(MX6SL_BRD_EPDC_GDSP);
	gpio_free(MX6SL_BRD_EPDC_GDOE);
	gpio_free(MX6SL_BRD_EPDC_GDRL);
	gpio_free(MX6SL_BRD_EPDC_SDCLK);
	gpio_free(MX6SL_BRD_EPDC_SDOE);
	gpio_free(MX6SL_BRD_EPDC_SDLE);
	gpio_free(MX6SL_BRD_EPDC_SDSHR);
	gpio_free(MX6SL_BRD_EPDC_BDR0);
	gpio_free(MX6SL_BRD_EPDC_SDCE0);
	gpio_free(MX6SL_BRD_EPDC_SDCE1);
	gpio_free(MX6SL_BRD_EPDC_SDCE2);
}

static void epdc_enable_pins(void)
{
	/* Configure MUX settings to enable EPDC use */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_epdc_enable_pads, \
				ARRAY_SIZE(mx6sl_brd_epdc_enable_pads));

	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_2);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_3);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_4);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_5);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_6);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_7);
	gpio_direction_input(MX6SL_BRD_EPDC_GDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_GDSP);
	gpio_direction_input(MX6SL_BRD_EPDC_GDOE);
	gpio_direction_input(MX6SL_BRD_EPDC_GDRL);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_SDOE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDLE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDSHR);
	gpio_direction_input(MX6SL_BRD_EPDC_BDR0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE2);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to
	 * GPIO and drive to 0. */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_epdc_disable_pads, \
				ARRAY_SIZE(mx6sl_brd_epdc_disable_pads));

	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_2, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_3, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_4, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_5, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_6, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_7, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDSP, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDOE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDRL, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDOE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDLE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDSHR, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_BDR0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE2, 0);
}

static struct fb_videomode e60_v110_mode = {
	.name = "E60_V110",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 18604700,
	.left_margin = 8,
	.right_margin = 178,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode e60_v220_mode = {
	.name = "E60_V220",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 30000000,
	.left_margin = 8,
	.right_margin = 164,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
	.refresh = 85,
	.xres = 800,
	.yres = 600,
};
static struct fb_videomode e060scm_mode = {
	.name = "E060SCM",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock = 26666667,
	.left_margin = 8,
	.right_margin = 100,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct fb_videomode e97_v110_mode = {
	.name = "E97_V110",
	.refresh = 50,
	.xres = 1200,
	.yres = 825,
	.pixclock = 32000000,
	.left_margin = 12,
	.right_margin = 128,
	.upper_margin = 4,
	.lower_margin = 10,
	.hsync_len = 20,
	.vsync_len = 4,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
///1,edpc_E60_V220_C011_WA1001.fw
static struct fb_videomode e60_v220_c011_wa1001_mode = {
	.name = "E60_V220_C011_WA1001",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock =26666667,/*default:30000000 ,32000000==16Mhz*/
	.left_margin = 8,
	.right_margin =100,/* 164,*/
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

///1,epdc_E60_V220_E157_WA1206.fw
static struct fb_videomode e60_v220_e157_mode = {
	.name = "E60_V220_E157_WA1206",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock =26666667,/*26666667 ,32000000==16Mhz*/
	.left_margin = 8,
	.right_margin =100 ,/*100 164,*/
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

///10,epdc_E60_V220_E305_WA0501.fw
static struct fb_videomode e60_v220_e305_mode = {
	.name = "E60_V220_E305_WA0501",
	.refresh = 85,
	.xres = 800,
	.yres = 600,
	.pixclock =32000000,/*default:30000000 ,32000000==16Mhz*/
	.left_margin = 8,
	.right_margin = 164,
	.upper_margin = 4,
	.lower_margin = 8,
	.hsync_len = 4,
	.vsync_len = 1,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
///HD waveform:epdc_E60_V220_C105_WN5E21.fw
static struct fb_videomode e60_v220_hd_mode = {
	.name = "E60_V220_C105_WN5E21",
	.refresh = 85,
	.xres = 1024,
	.yres = 758,
	.pixclock = 40000000,/*56000000,40000000,32000000*/
	.left_margin = 12,/* ,8 */
	.right_margin =76 ,/*166,*/
	.upper_margin = 4 ,/* 4,*/
	.lower_margin = 5,/* ,26*/
	.hsync_len = 12,
	.vsync_len = 2,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

///pb650 waveform file.epdc_E60_pb650_ED060XF3T1
static struct fb_videomode e60_pb650_mode = {
	.name = "E60_pb650_ED060XF3T1",
	.refresh = 85,
	.xres = 1024,
	.yres = 758,
	.pixclock = 40000000,/*56000000,40000000,32000000*/
	.left_margin = 12,/* ,8 */
	.right_margin = 76 ,/*166,*/
	.upper_margin = 4 ,/* 4,*/
	.lower_margin = 5,/* ,26*/
	.hsync_len = 12,
	.vsync_len = 2,
	.sync = 0,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};
static struct imx_epdc_fb_mode evk_panel_modes[] = {
	{
		&e60_v220_hd_mode,
		4,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		428,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		10,	/* gdclk_offs */
		1,	/* num_ce */
	},	
	{
		&e60_pb650_mode,
		4,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		428,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		10,	/* gdclk_offs */
		1,	/* num_ce */
	},	
	{
		&e60_v110_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		428,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e60_v220_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		465,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		9,      /* gdclk_offs */
		1,      /* num_ce */
	},
	{
		&e060scm_mode,
		4,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		419,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		5,      /* gdclk_offs */
		1,      /* num_ce */
	},	
	{
		&e97_v110_mode,
		8,      /* vscan_holdoff */
		10,     /* sdoed_width */
		20,     /* sdoed_delay */
		10,     /* sdoez_width */
		20,     /* sdoez_delay */
		632,    /* gdclk_hp_offs */
		20,     /* gdsp_offs */
		0,      /* gdoe_offs */
		1,      /* gdclk_offs */
		3,      /* num_ce */
	},
       /*edpc_E60_V220_C011_WA1001.fw*/
	{
		&e60_v220_c011_wa1001_mode,
		4,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		419,	/*465 gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		5,	/*9 gdclk_offs */
		1,	/* num_ce */
	},	
       /*epdc_E60_V220_E157_WA1206.fw*/
	{
		&e60_v220_e157_mode,
		4,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		419,	/*465 gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		5,	/*9 gdclk_offs */
		1,	/* num_ce */
	},
      ///10,epdc_E60_V220_E305_WA0501.fw
	{
		&e60_v220_e305_mode,
		4,	/* vscan_holdoff */
		10,	/* sdoed_width */
		20,	/* sdoed_delay */
		10,	/* sdoez_width */
		20,	/* sdoez_delay */
		465,	/* gdclk_hp_offs */
		20,	/* gdsp_offs */
		0,	/* gdoe_offs */
		9,	/* gdclk_offs */
		1,	/* num_ce */
	}
};

static struct imx_epdc_fb_platform_data evk_epdc_data = {
	.epdc_mode = evk_panel_modes,
	.num_modes = ARRAY_SIZE(evk_panel_modes),
	.get_pins = epdc_get_pins,
	.put_pins = epdc_put_pins,
	.enable_pins = epdc_enable_pins,
	.disable_pins = epdc_disable_pins,
};
#endif
#if defined(CONFIG_FB_MXC_SIPIX_PANEL)
static int spdc_get_pins(void)
{
	int ret = 0;

	/* Claim GPIOs for SPDC pins - used during power up/down */
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_0, "SPDC_D0");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_1, "SPDC_D1");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_2, "SPDC_D2");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_3, "SPDC_D3");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_4, "SPDC_D4");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_5, "SPDC_D5");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_6, "SPDC_D6");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_7, "SPDC_D7");

	ret |= gpio_request(MX6SL_BRD_EPDC_GDOE, "SIPIX_YOE");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_9, "SIPIX_PWR_RDY");

	ret |= gpio_request(MX6SL_BRD_EPDC_GDSP, "SIPIX_YDIO");

	ret |= gpio_request(MX6SL_BRD_EPDC_GDCLK, "SIPIX_YCLK");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDSHR, "SIPIX_XDIO");

	ret |= gpio_request(MX6SL_BRD_EPDC_SDLE, "SIPIX_LD");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE1, "SIPIX_SOE");

	ret |= gpio_request(MX6SL_BRD_EPDC_SDCLK, "SIPIX_XCLK");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDDO_10, "SIPIX_SHD_N");
	ret |= gpio_request(MX6SL_BRD_EPDC_SDCE0, "SIPIX2_CE");

	return ret;
}

static void spdc_put_pins(void)
{
	gpio_free(MX6SL_BRD_EPDC_SDDO_0);
	gpio_free(MX6SL_BRD_EPDC_SDDO_1);
	gpio_free(MX6SL_BRD_EPDC_SDDO_2);
	gpio_free(MX6SL_BRD_EPDC_SDDO_3);
	gpio_free(MX6SL_BRD_EPDC_SDDO_4);
	gpio_free(MX6SL_BRD_EPDC_SDDO_5);
	gpio_free(MX6SL_BRD_EPDC_SDDO_6);
	gpio_free(MX6SL_BRD_EPDC_SDDO_7);

	gpio_free(MX6SL_BRD_EPDC_GDOE);
	gpio_free(MX6SL_BRD_EPDC_SDDO_9);
	gpio_free(MX6SL_BRD_EPDC_GDSP);
	gpio_free(MX6SL_BRD_EPDC_GDCLK);
	gpio_free(MX6SL_BRD_EPDC_SDSHR);
	gpio_free(MX6SL_BRD_EPDC_SDLE);
	gpio_free(MX6SL_BRD_EPDC_SDCE1);
	gpio_free(MX6SL_BRD_EPDC_SDCLK);
	gpio_free(MX6SL_BRD_EPDC_SDDO_10);
	gpio_free(MX6SL_BRD_EPDC_SDCE0);
}

static void spdc_enable_pins(void)
{
	/* Configure MUX settings to enable SPDC use */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_spdc_enable_pads, \
				ARRAY_SIZE(mx6sl_brd_spdc_enable_pads));

	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_0);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_2);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_3);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_4);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_5);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_6);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_7);
	gpio_direction_input(MX6SL_BRD_EPDC_GDOE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_9);
	gpio_direction_input(MX6SL_BRD_EPDC_GDSP);
	gpio_direction_input(MX6SL_BRD_EPDC_GDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_SDSHR);
	gpio_direction_input(MX6SL_BRD_EPDC_SDLE);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE1);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCLK);
	gpio_direction_input(MX6SL_BRD_EPDC_SDDO_10);
	gpio_direction_input(MX6SL_BRD_EPDC_SDCE0);
}

static void spdc_disable_pins(void)
{
	/* Configure MUX settings for SPDC pins to
	 * GPIO and drive to 0. */
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_spdc_disable_pads, \
				ARRAY_SIZE(mx6sl_brd_spdc_disable_pads));

	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_0, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_2, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_3, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_4, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_5, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_6, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_7, 0);

	gpio_direction_output(MX6SL_BRD_EPDC_GDOE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_9, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDSP, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_GDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDSHR, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDLE, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE1, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCLK, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDDO_10, 0);
	gpio_direction_output(MX6SL_BRD_EPDC_SDCE0, 0);
}

static struct imx_spdc_panel_init_set spdc_init_set = {
	.yoe_pol = false,
	.dual_gate = false,
	.resolution = 0,
	.ud = false,
	.rl = false,
	.data_filter_n = true,
	.power_ready = true,
	.rgbw_mode_enable = false,
	.hburst_len_en = true,
};

static struct fb_videomode erk_1_4_a01 = {
	.name = "ERK_1_4_A01",
	.refresh = 50,
	.xres = 800,
	.yres = 600,
	.pixclock = 40000000,
	.vmode = FB_VMODE_NONINTERLACED,
};

static struct imx_spdc_fb_mode spdc_panel_modes[] = {
	{
		&erk_1_4_a01,
		&spdc_init_set,
		.wave_timing = "pvi"
	},
};

static struct imx_spdc_fb_platform_data spdc_data = {
	.spdc_mode = spdc_panel_modes,
	.num_modes = ARRAY_SIZE(spdc_panel_modes),
	.get_pins = spdc_get_pins,
	.put_pins = spdc_put_pins,
	.enable_pins = spdc_enable_pins,
	.disable_pins = spdc_disable_pins,
};

static int __init early_use_spdc_sel(char *p)
{
	evk_spdc_sel = 1;
	return 0;
}
early_param("spdc", early_use_spdc_sel);

static void imx6_evk_setup_spdc(void)
{
	/* GPR0[8]: 0:EPDC, 1:SPDC */
	if (evk_spdc_sel)
		mxc_iomux_set_gpr_register(0, 8, 1, 1);
}
#endif

static void imx6_evk_usbotg_vbus(bool on)
{
	//if (on)
	//	gpio_set_value(MX6_BRD_USBOTG1_PWR, 1);
	//else
	//	gpio_set_value(MX6_BRD_USBOTG1_PWR, 0);
}

static void imx6_evk_usbh1_vbus(bool on)
{
	//if (on)
	//	gpio_set_value(MX6_BRD_USBOTG2_PWR, 1);
	//else
	//	gpio_set_value(MX6_BRD_USBOTG2_PWR, 0);
}

static int __init mx6_evk_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */

	//ret = gpio_request(MX6_BRD_USBOTG1_PWR, "usbotg-pwr");
	//if (ret) {
	//	pr_err("failed to get GPIO MX6_BRD_USBOTG1_PWR:%d\n", ret);
	//	return ret;
	//}
	//gpio_direction_output(MX6_BRD_USBOTG1_PWR, 0);

	//ret = gpio_request(MX6_BRD_USBOTG2_PWR, "usbh1-pwr");
	//if (ret) {
	//	pr_err("failed to get GPIO MX6_BRD_USBOTG2_PWR:%d\n", ret);
	//	return ret;
	//}
	//gpio_direction_output(MX6_BRD_USBOTG2_PWR, 0);

	mx6_set_otghost_vbus_func(imx6_evk_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6_evk_usbh1_vbus);
	//mx6_usb_dr_init();
#ifdef CONFIG_USB_EHCI_ARC_HSIC
	mx6_usb_h2_init();
#endif

	return ret;
}
#if defined(CONFIG_MMC_SDHCI_PLTFM)
///////////////////////////////////////////////////////////
////control the SD CARD voltage
///default is high,
///true:set to low
///false:set to high
///////////////////////////////////////////////////////////
static int __init mx6sl_evk_external_card_power_control(bool bOpen)
{
	int ret = 0;

	printk("%s %s %d bOpen=%d\n",__FILE__,__func__,__LINE__,bOpen); 

	////init cd gpio,only for test///
//	ret = gpio_request(MX6_BRD_SD1_CD, "sd1-cd");
//	if (ret) {
//		pr_err("failed to get GPIO MX6_BRD_SD1_CD:%d\n", ret);
//		return;
//	}
//	  gpio_direction_input(MX6_BRD_SD1_CD);
	 ////////

	ret = gpio_request(MX6_BRD_EXTERNAL_CARD_POWER, "external-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_BRD_EXTERNAL_CARD_POWER:%d\n", ret);
		return ret;
	}
	if(bOpen)
	   gpio_direction_output(MX6_BRD_EXTERNAL_CARD_POWER, 0);
	else
	    gpio_direction_output(MX6_BRD_EXTERNAL_CARD_POWER, 1);

	 return ret;
}
#endif
//////////////////////////////////////////
/////////backlight pwm
//////////////////////////////////////////
#if defined(CONFIG_BACKLIGHT_PWM)
static struct platform_pwm_backlight_data mx6_evk_pwm_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 128,
	.pwm_period_ns	= 1000000, //f=1k
};
#endif
//////////////////////////////////////////
///                           TFT LCD  Screen
//////////////////////////////////////////
#if (defined(CONFIG_IMX_HAVE_PLATFORM_IMX_ELCDIF) &&\
	defined(CONFIG_FB_MXC_ELCDIF_FB))
static struct fb_videomode wvga_video_modes[] = {
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 32MHz */
	 "SEIKO-WVGA", 60, 800, 480, 29850, 89, 164, 23, 10, 10, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static struct mxc_fb_platform_data wvga_fb_data[] = {
	{
	 .interface_pix_fmt = V4L2_PIX_FMT_RGB24,
	 .mode_str = "SEIKO-WVGA",
	 .mode = wvga_video_modes,
	 .num_modes = ARRAY_SIZE(wvga_video_modes),
	 },
};

static struct platform_device lcd_wvga_device = {
	.name = "lcd_seiko",
};
#endif

#if defined(CONFIG_FB_MXC_SII902X)
static struct fb_videomode hdmi_video_modes[] = {
	{
	 /* 1920x1080 @ 60 Hz , pixel clk @ 148MHz */
	 "sii9022x_1080p60", 60, 1920, 1080, 6734, 148, 88, 36, 4, 44, 5,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static struct mxc_fb_platform_data hdmi_fb_data[] = {
	{
	 .interface_pix_fmt = V4L2_PIX_FMT_RGB24,
	 .mode_str = "1920x1080M@60",
	 .mode = hdmi_video_modes,
	 .num_modes = ARRAY_SIZE(hdmi_video_modes),
	 },
};
#endif

#if defined(CONFIG_KEYBOARD_IMX)
static int mx6sl_evk_keymap[] = {
	KEY(0, 0, KEY_SELECT),
	KEY(0, 1, KEY_BACK),
	KEY(0, 2, KEY_F1),
	KEY(0, 3, KEY_F2),

	KEY(1, 0, KEY_F3),
	KEY(1, 1, KEY_F4),
	KEY(1, 2, KEY_F5),
	KEY(1, 3, KEY_MENU),

	KEY(2, 0, KEY_PREVIOUS),
	KEY(2, 1, KEY_NEXT),
	KEY(2, 2, KEY_HOME),
	KEY(2, 3, KEY_NEXT),

	KEY(3, 0, KEY_UP),
	KEY(3, 1, KEY_LEFT),
	KEY(3, 2, KEY_RIGHT),
	KEY(3, 3, KEY_DOWN),
};

static const struct matrix_keymap_data mx6sl_evk_map_data __initconst = {
	.keymap		= mx6sl_evk_keymap,
	.keymap_size	= ARRAY_SIZE(mx6sl_evk_keymap),
};
#endif
//////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

#define HW_ANADIG_USB1_VBUS_DET_STAT	(0x000001c0)
#define BM_ANADIG_USB1_VBUS_DET_STAT_VBUS_VALID 0x00000008
static int is_usb_plugged(void) {
	void __iomem *mx6_ana_base = IO_ADDRESS(ANATOP_BASE_ADDR);
	return (readl(mx6_ana_base + HW_ANADIG_USB1_VBUS_DET_STAT) \
			& BM_ANADIG_USB1_VBUS_DET_STAT_VBUS_VALID) ? 1 : 0;
}

extern void machine_restart(char *);

#include <linux/power_supply.h>
#include <linux/cpufreq.h>
#define SNVS_LPCR 0x38
static void mx6_evk_snvs_poweroff(void)
{
	u32 value;
	void __iomem *mx6_snvs_base = MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	struct power_supply *ps = power_supply_get_by_name("battery");
	struct cpufreq_policy *policy;

	//charging monitor
	int e = 0, pk = 0, cs = 0;
	int v = 0, c = 0;
//	gpio_free(MX6SL_FEC_REF_CLK_CHARGER_STATE);
	gpio_free(MX6SL_EVK_POWER_OFF);

	int usb_port_state = is_usb_plugged();

//	e = gpio_request_one(MX6SL_FEC_REF_CLK_CHARGER_STATE, GPIOF_IN, "charger_state");
//	if (e) {
//		printk("[%s] charger_state gpio_request failed, e=%i\n",__func__,e);
//		goto power_off;
//	}

	e = gpio_request_one(MX6SL_EVK_POWER_OFF, GPIOF_IN, "power_button");
	if (e) {
		printk("[%s] power_button gpio_request failed, e=%i\n",__func__,e);
		goto power_off;
	}
	if (usb_port_state) printk("USB port detected charger\n");
	printk("\n");

	// save some power
	policy = cpufreq_cpu_get(0);
	if (policy) {
		policy->max = policy->min;
		policy->user_policy.max = policy->user_policy.min;
		cpufreq_cpu_put(policy);
		cpufreq_update_policy(0);
	}

	while(usb_port_state) {
		//watch for charging state and power key
		//cs = gpio_get_value_cansleep(MX6SL_FEC_REF_CLK_CHARGER_STATE);
		pk = gpio_get_value_cansleep(MX6SL_EVK_POWER_OFF);
		usb_port_state = is_usb_plugged();
		if (ps) {
			ps->get_property(ps,POWER_SUPPLY_PROP_CAPACITY,&c);
			ps->get_property(ps,POWER_SUPPLY_PROP_VOLTAGE_NOW,&v);
		}

		printk("<c>Battery state = %s :: cap=%i voltage=%i                   \r",cs ? "full" : "charging",c,v);
		if (pk == 0) {
			printk("\nPower button was pressed\n");
			goto reset;
		}
		mdelay(200);
	}

	printk("\nDo power off!\n");

power_off:
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/* set TOP and DP_EN bit */
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
	return;

reset:
	machine_restart(NULL);
	return;
}


#if (defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE))


#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button imx6sl_buttons[] = {
	GPIO_BUTTON(MX6SL_EVK_POWER_OFF, KEY_POWER, 1, "power", 1, 1),
	GPIO_BUTTON(MX6SL_KEY_COL2_HALL_DET, KEY_SCREENLOCK, 1, "lock", 1, 1),
	GPIO_BUTTON(MX6SL_KEY_ROW0_MENU, KEY_MENU, 0, "menu", 1,1),
	GPIO_BUTTON(MX6SL_KEY_ROW1_HOME, KEY_HOME, 0, "home", 1,1),
	GPIO_BUTTON(MX6SL_KEY_ROW2_BACKWARD, KEY_LEFT, 0, "left", 1,1),
	GPIO_BUTTON(MX6SL_KEY_ROW3_FORWARD, KEY_RIGHT, 0, "right", 1,1),
	GPIO_BUTTON(MX6SL_KEY_ROW4_PAGEUP, KEY_UP, 0, "up", 1,1),
	GPIO_BUTTON(MX6SL_KEY_ROW5_PAGEDOWN, KEY_DOWN, 0, "down", 1,1),
};

static struct gpio_keys_platform_data imx6sl_button_data = {
	.buttons	= imx6sl_buttons,
	.nbuttons	= ARRAY_SIZE(imx6sl_buttons),
	.rep            = 0,
};

static struct platform_device imx6sl_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6sl_button_data,
	}
};

static void __init imx6sl_add_device_buttons(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_gpio_pads,
					ARRAY_SIZE(mx6sl_brd_gpio_pads));
	
	platform_device_register(&imx6sl_button_device);
}
#else
static void __init imx6sl_add_device_buttons(void) {}
#endif

////////////////////////////////////////////////
///////////////////////////////////////////////
///WIFI  control
////////////////////////////////////////////////
static int  __init wifi_power_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_wifi_pads,
			ARRAY_SIZE(mx6sl_brd_wifi_pads));

	gpio_request(MX6SL_BRD_WIFI_POWER, "wifi_power");
	gpio_direction_input(MX6SL_BRD_WIFI_POWER);

	//reset init 
	gpio_request(MX6SL_BRD_WIFI_RESET, "wifi_reset");
	gpio_direction_input(MX6SL_BRD_WIFI_RESET);
	msleep(1);

	return 0;
}

int  wifi_power_control(bool bEnable)
{
	printk("%s %s %d bEnable=%d\n",__FILE__,__func__,__LINE__,bEnable); 

	if(bEnable){
		//before open the gpio,we need set it as output mode
		gpio_direction_output(MX6SL_BRD_WIFI_RESET, MX6SL_GPIO_LOW);

		gpio_direction_output(MX6SL_BRD_WIFI_POWER, MX6SL_GPIO_HIGH);
		msleep(10);

		gpio_set_value(MX6SL_BRD_WIFI_RESET, MX6SL_GPIO_HIGH);

/* what for???
		//////then reset the wifi,
		gpio_set_value(MX6SL_BRD_WIFI_RESET, MX6SL_GPIO_HIGH);
		msleep(10);
		gpio_set_value(MX6SL_BRD_WIFI_RESET, MX6SL_GPIO_LOW);
		msleep(10);
		gpio_set_value(MX6SL_BRD_WIFI_RESET, MX6SL_GPIO_HIGH);
		msleep(10);
*/
	}else{
		///in order to reduce  power consumpiton,we need
		///set the gpio as input mode
		gpio_set_value(MX6SL_BRD_WIFI_RESET, MX6SL_GPIO_LOW);
		msleep(10);
		gpio_set_value(MX6SL_BRD_WIFI_POWER, MX6SL_GPIO_LOW);
		msleep(10);
		gpio_direction_input(MX6SL_BRD_WIFI_POWER);
		gpio_direction_input(MX6SL_BRD_WIFI_RESET);

	}

	return 0;
}
EXPORT_SYMBOL(wifi_power_control);

////////////////////////////////////////////////
/////PB650
////////////////////////////////////////////////
/*Set Pad to 1.8v*/
static int mx6sl_SetPadVoltage(void)
{
	printk("%s %s %d \n",__FILE__,__func__,__LINE__); 
	/*EPDC_D14	Audio_RST	1.8 */
	mxc_iomux_set_specialbits_register(MX6SL_PAD_EPDC_D14,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	/*headphone insert*/
	mxc_iomux_set_specialbits_register(MX6SL_PAD_EPDC_D13,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	/*I2C3 */
	//mxc_iomux_set_specialbits_register(MX6SL_PAD_EPDC_SDCE2,
	//				PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	//mxc_iomux_set_specialbits_register(MX6SL_PAD_EPDC_SDCE3,
	//				PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_RXFS,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);
	mxc_iomux_set_specialbits_register(MX6SL_PAD_AUD_RXC,
					PAD_CTL_LVE, PAD_CTL_LVE_MASK);

	return 0;
}

/* System LED init and control */
static int imx6sl_system_led_control(bool enable)
{
	pr_debug("[%s] Set system led %d\n", __func__, enable);

	gpio_direction_output(MX6SL_EPDC_PWRCTRL1_SYSTEM_LED, enable);
	gpio_direction_output(MX6SL_FEC_TX_EN_CHRG_OFF, enable);

	return 0;
}

/* Return current state of System LED */
static int imx6sl_system_led_state(void)
{
	pr_debug("[%s] Set system state\n", __func__);

	return gpio_get_value(MX6SL_EPDC_PWRCTRL1_SYSTEM_LED);
}

static int imx6sl_system_led_init(void)
{
	int ret;

	pr_debug("[%s] Init system led\n", __func__);

	ret = gpio_request(MX6SL_EPDC_PWRCTRL1_SYSTEM_LED, "led-int");
	ret |= gpio_request(MX6SL_FEC_TX_EN_CHRG_OFF, "led-int");
	if (ret) {
		pr_err("[%s] request led-int error!\n", __func__);
		return -1;
	}

	return 0;
}

static struct system_led_platform_data imx6sl_system_led_data = {
	.init		= imx6sl_system_led_init,
	.control	= imx6sl_system_led_control,
	.state		= imx6sl_system_led_state,
};

static struct platform_device imx6sl_system_led = {
	.name		= "system-leds",
	.id		= -1,
	.dev		= {
		.platform_data = &imx6sl_system_led_data,
	}
};

//moved directly to backlight driver (driver/video/backlight/backlight.c)
#if 0
/*fontlight enable or disable*/
static int mx6sl_fontlight_enable_init(void)
{
	 int ret;
	 
	ret = gpio_request(MX6SL_EPDC_D15_FL_EN, "fontlight-int");
	if (ret) {
		printk(KERN_ERR"request led-int error!!\n");
		return -1;
	} else {
		gpio_direction_output(MX6SL_EPDC_D15_FL_EN,MX6SL_GPIO_LOW);
	}

	 return 0;
}
int  fontlight_enable_control(bool bEnable)
{
	//printk("%s %s %d bEnable=%d\n",__FILE__,__func__,__LINE__,bEnable); 

	if(bEnable){
    	    gpio_set_value(MX6SL_EPDC_D15_FL_EN, MX6SL_GPIO_HIGH);
	}else{
	    gpio_set_value(MX6SL_EPDC_D15_FL_EN, MX6SL_GPIO_LOW);
	}

      return 0;
}
EXPORT_SYMBOL(fontlight_enable_control);
int  get_fontlight_enable_control_state(void)
{
     return gpio_get_value(MX6SL_EPDC_D15_FL_EN);
}
EXPORT_SYMBOL(get_fontlight_enable_control_state);

#endif
static int mx6sl_gpio_deep_suspend_enter(void)
{
    /// int wifi_power=0;
        //make sure the  wifi power is off
        ///wifi_power = gpio_get_value(MX6SL_BRD_WIFI_POWER);
	//printk("%s %s %d wifi_power=%d\n",__FILE__,__func__,__LINE__,wifi_power); 
	///if(wifi_power == 1)
       //      wifi_power_control(false);
	///close the wifi 32K clock,we need this pin as gpio pin.
	gpio_request(MX6SL_BRP_WIFI_CLOCK_PIN, "wifi_clock_pin");
	gpio_direction_input(MX6SL_BRP_WIFI_CLOCK_PIN);
	/////camera //we need set is as gpio pin.
	///CAM_DAT2
	gpio_request(MX6SL_CAMERA_CAM_DAT2_PIN, "cam_data_2");
	gpio_direction_input(MX6SL_CAMERA_CAM_DAT2_PIN);
	///CAM_DAT3
	gpio_request(MX6SL_CAMERA_CAM_DAT3_PIN, "cam_data_3");
	gpio_direction_input(MX6SL_CAMERA_CAM_DAT3_PIN);
	///CAM_DAT4
	gpio_request(MX6SL_CAMERA_CAM_DAT4_PIN, "cam_data_4");
	gpio_direction_input(MX6SL_CAMERA_CAM_DAT4_PIN);
	///CAM_DAT5
	gpio_request(MX6SL_CAMERA_CAM_DAT5_PIN, "cam_data_5");
	gpio_direction_input(MX6SL_CAMERA_CAM_DAT5_PIN);
	///CAM_DAT6
	gpio_request(MX6SL_CAMERA_CAM_DAT6_PIN, "cam_data_6");
	gpio_direction_input(MX6SL_CAMERA_CAM_DAT6_PIN);
	///CAM_DAT7
	gpio_request(MX6SL_CAMERA_CAM_DAT7_PIN, "cam_data_7");
	gpio_direction_input(MX6SL_CAMERA_CAM_DAT7_PIN);
	///CAM_DAT8
	gpio_request(MX6SL_CAMERA_CAM_DAT8_PIN, "cam_data_8");
	gpio_direction_input(MX6SL_CAMERA_CAM_DAT8_PIN);
	///CAM_DAT9
	gpio_request(MX6SL_CAMERA_CAM_DAT9_PIN, "cam_data_9");
	gpio_direction_input(MX6SL_CAMERA_CAM_DAT9_PIN);

//	///CAM RESET ,SET it as input mode
//	gpio_direction_input(MX6SL_BRD_CSI_RST);

	return 0;    
}
static int mx6sl_gpio_suspend_exit(void)
{

	/*set to 1.8v*/
	mx6sl_SetPadVoltage();

	//set camera's pins to operate with 1.8V
	mx6sl_setCameraPadVoltage();
	return 0;
}

static void mx6sl_evk_suspend_enter()
{
#if 0
    if(iDeepStandy == 0){
	    ///mx6sl_evk_suspend(deep_suspend_enter_pads,deep_suspend_enter_pads);	
		 //  mx6sl_evk_suspend(suspend_enter_pads,suspend_exit_pads);	
		iomux_v3_cfg_t *p = deep_suspend_enter_pads;
		int i;
		mx6sl_gpio_deep_suspend_enter();

		/* Set PADCTRL to 0 for all IOMUX. */
		for (i = 0; i < ARRAY_SIZE(deep_suspend_enter_pads); i++) {
			deep_suspend_exit_pads[i] = *p;
			*p &= ~MUX_PAD_CTRL_MASK;
			/* Enable the Pull down and the keeper
			  * Set the drive strength to 0.
			  */
			*p |= ((u64)0x3000 << MUX_PAD_CTRL_SHIFT);
			p++;
		}
		mxc_iomux_v3_get_multiple_pads(deep_suspend_exit_pads,
				ARRAY_SIZE(deep_suspend_exit_pads));
		mxc_iomux_v3_setup_multiple_pads(deep_suspend_enter_pads,
				ARRAY_SIZE(deep_suspend_enter_pads));
	    
    	}else if(iDeepStandy ==1)
    	{
		 //  mx6sl_evk_suspend(suspend_enter_pads,suspend_exit_pads);	
		iomux_v3_cfg_t *p = suspend_enter_pads;
		int i;


		/* Set PADCTRL to 0 for all IOMUX. */
		for (i = 0; i < ARRAY_SIZE(suspend_enter_pads); i++) {
			suspend_exit_pads[i] = *p;
			*p &= ~MUX_PAD_CTRL_MASK;
			/* Enable the Pull down and the keeper
			  * Set the drive strength to 0.
			  */
			*p |= ((u64)0x3000 << MUX_PAD_CTRL_SHIFT);
			p++;
		}
		mxc_iomux_v3_get_multiple_pads(suspend_exit_pads,
				ARRAY_SIZE(suspend_exit_pads));
		mxc_iomux_v3_setup_multiple_pads(suspend_enter_pads,
				ARRAY_SIZE(suspend_enter_pads));
    	}
#else
{
	iomux_v3_cfg_t *p = suspend_enter_pads;
	int i;


	/* Set PADCTRL to 0 for all IOMUX. */
	for (i = 0; i < ARRAY_SIZE(suspend_enter_pads); i++) {
		suspend_exit_pads[i] = *p;
		*p &= ~MUX_PAD_CTRL_MASK;
		/* Enable the Pull down and the keeper
		  * Set the drive strength to 0.
		  */
		*p |= ((u64)0x3000 << MUX_PAD_CTRL_SHIFT);
		p++;
	}
	mxc_iomux_v3_get_multiple_pads(suspend_exit_pads,
			ARRAY_SIZE(suspend_exit_pads));
	mxc_iomux_v3_setup_multiple_pads(suspend_enter_pads,
			ARRAY_SIZE(suspend_enter_pads));
}	
//	 mx6sl_evk_suspend(suspend_enter_pads,suspend_exit_pads);	
#endif	
}

static void mx6sl_evk_suspend_exit()
{
#if 0
	if(iDeepStandy == 0){
		mxc_iomux_v3_setup_multiple_pads(deep_suspend_exit_pads,
				ARRAY_SIZE(deep_suspend_exit_pads));
	}else if(1){
		mxc_iomux_v3_setup_multiple_pads(suspend_exit_pads,
				ARRAY_SIZE(suspend_exit_pads));
	}
#else
	mxc_iomux_v3_setup_multiple_pads(suspend_exit_pads,
			ARRAY_SIZE(suspend_exit_pads));
#endif
	mx6sl_gpio_suspend_exit();
}

static const struct pm_platform_data mx6sl_evk_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter = mx6sl_evk_suspend_enter,
	.suspend_exit = mx6sl_evk_suspend_exit,
};
////////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////
#if defined(CONFIG_MFD_PFUZE)
static int __init  mx6_evk_pfuze100_gpio_init(void)
{
	int ret;
	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_pfuze100_pads,
					ARRAY_SIZE(mx6sl_brd_pfuze100_pads));


	ret = gpio_request(MX6SL_REF_CLK_24M_PFUZE_INI, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return -1;
	} else {
		gpio_direction_input(MX6SL_REF_CLK_24M_PFUZE_INI);
		mx6sl_evk_init_pfuze100(MX6SL_REF_CLK_24M_PFUZE_INI);
	}

	 return 0;
}
#endif
/*!
 * Board specific initialization.
 */
static void __init mx6_evk_init(void)
{
	u32 i;

	pr_debug("[%s] Enter\n", __func__);

	mxc_iomux_v3_setup_multiple_pads(mx6sl_brd_pads,
					ARRAY_SIZE(mx6sl_brd_pads));


	gp_reg_id = mx6sl_evk_dvfscore_data.reg_id;
	soc_reg_id = mx6sl_evk_dvfscore_data.soc_id;

       mx6sl_SetPadVoltage();

	 #if defined(CONFIG_TOUCHSCREEN_CNT)
	 imx6sl_cnt_ts_init();
         #endif
  
#if defined(CONFIG_RTC_DRV_SNVS)
      /*about file:
         * 1). platform-imx_snvs_rtc.c
         * 2).drivers/rtc/rtc-snvs.c
         */
	imx6q_add_imx_snvs_rtc();
#endif

	imx6q_add_imx_i2c(0, &mx6_evk_i2c0_data);
	imx6q_add_imx_i2c(1, &mx6_evk_i2c1_data);
	imx6q_add_imx_i2c(2, &mx6_evk_i2c2_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));

////////////////////////////////////
#if defined(CONFIG_FB_MXC_SII902X)
	/*  setting sii902x address when hdmi enabled */
	if (hdmi_enabled) {
		for (i = 0; i < ARRAY_SIZE(mxc_i2c1_board_info); i++) {
			if (!strcmp(mxc_i2c1_board_info[i].type, "sii902x")) {
				mxc_i2c1_board_info[i].addr = 0x39;
				break;
			}
		}
	}
#endif

	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

        ///wifi power open
        wifi_power_init();
        //turn wifi power off
        wifi_power_control(false);

	////////////////////////////////////
	////////////////////////////////////
#if (defined(CONFIG_FB_MXC_SIPIX_PANEL) && defined(CONFIG_VIDEO_MXC_CSI_CAMERA))
	imx6_evk_setup_spdc();
	if (csi_enabled ==1) {
		mxc_register_device(&csi_v4l2_devices, NULL);
		imx6sl_add_fsl_csi();
	} else  {
		if (!evk_spdc_sel)
			imx6dl_add_imx_epdc(&evk_epdc_data);
		else
			imx6sl_add_imx_spdc(&spdc_data);
	}

#elif defined(CONFIG_VIDEO_MXC_CSI_CAMERA)
	/* only camera on I2C2, that's why we can do so */
	if (csi_enabled == 1) {
		mxc_register_device(&csi_v4l2_devices, NULL);
	        /*about file:
	         * 1). platform-imx-fsl-csi.c
	         * 2).drivers/media/video/mxc/capture/fsl_csi.c
	         */
		//if (csi_enabled) {
		imx6sl_add_fsl_csi();
		//imx6q_add_v4l2_output(0);
		//imx6q_add_v4l2_capture(0, &evk_capture_data[0]);
		//imx6q_add_v4l2_capture(1, &capture_data[1]);
		
	  }
#endif
#if defined(CONFIG_VIDEO_CSI_CAMERA_FLASH_LED)
        mxc_register_device(&sgm3140b_device, NULL);
#endif

	/* SPI */
#if defined(CONFIG_SPI_IMX)	
        /* drivers/spi/spi_imx.c */
	imx6q_add_ecspi(0, &mx6_evk_spi_data);
#endif
#if (defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE))
	mx6_evk_spi_device_init();
#endif
#if defined(CONFIG_MFD_PFUZE)
	//mx6_evk_pfuze100_gpio_init();
	mx6sl_evk_init_pfuze100(0);
	//mx6sl_evk_init_pfuze100(MX6SL_REF_CLK_24M_PFUZE_INI);
#endif

#if defined(CONFIG_ANATOP_THERMAL)
       /* drivers/mxc/thermal/thermal.c &&
            drivers/mxc/thermal/cooling.c  */
	imx6q_add_anatop_thermal_imx(1, &mx6sl_anatop_thermal_data);
#endif

#if defined(CONFIG_SERIAL_IMX)
       /*drivers/tty/serial/imx.c */
	mx6_evk_init_uart();
#endif
	/* get enet tx reference clk from FEC_REF_CLK pad.
	 * GPR1[14] = 0, GPR1[18:17] = 00
	 */
	mxc_iomux_set_gpr_register(1, 14, 1, 0);
	mxc_iomux_set_gpr_register(1, 17, 2, 0);

#if defined(CONFIG_PHYLIB)
        /*drivers/net/phy/phy.c*/
	imx6_init_fec(fec_data);
#endif
#if defined(CONFIG_MMC_SDHCI_PLTFM)
         ////only use for test. we need open the sd1 card voltage 
	 //mx6sl_evk_sd1_power_control(true);
	mx6sl_evk_external_card_power_control(true);

//	platform_device_register(&evk_vmmc_reg_devices);

         ////new hardware use SD1 bootup
	imx6q_add_sdhci_usdhc_imx(0, &mx6_evk_sd1_data);         
        ///SD2 bootup from this card,now use for camera.
	//imx6q_add_sdhci_usdhc_imx(1, &mx6_evk_sd2_data);
        ///externel sd card	
	imx6q_add_sdhci_usdhc_imx(2, &mx6_evk_sd3_data);
        /*sd4 wifi*/
	imx6q_add_sdhci_usdhc_imx(3, &mx6_evk_sd4_data);

	platform_device_register(&evk_vmmc_reg_devices);
#endif

	mx6_evk_init_usb();


#if defined(CONFIG_FSL_OTP)
	/*otp:CONFIG_FSL_OTP (drivers/char/fsl_otp.c)*/
	imx6q_add_otp();
#endif
#if defined(CONFIG_MXC_PWM)
	/*(arch/arm/plat-mxc/pwm.c)*/
	imx6q_add_mxc_pwm(0);
#endif
#if defined(CONFIG_BACKLIGHT_PWM)
      /*(drivers/video/backlight/pwm_bl.c)*/
	imx6q_add_mxc_pwm_backlight(0, &mx6_evk_pwm_backlight_data);
#endif

#if defined(CONFIG_FB_MXC_SII902X)
	if (hdmi_enabled) {
		imx6dl_add_imx_elcdif(&hdmi_fb_data[0]);
	} else {
		imx6dl_add_imx_elcdif(&wvga_fb_data[0]);

		gpio_request(MX6_BRD_LCD_PWR_EN, "elcdif-power-on");
		gpio_direction_output(MX6_BRD_LCD_PWR_EN, 1);
		mxc_register_device(&lcd_wvga_device, NULL);
	}
#endif
	
#if (defined(CONFIG_MXC_PXP)||\
	defined(CONFIG_MXC_PXP_V2))
	/* drivers/dma/pxp/pxp_dma_v2.c
            or  drivers/dma/pxp/pxp_dma.c */
	imx6dl_add_imx_pxp();
#endif
#if defined(CONFIG_MXC_PXP_CLIENT_DEVICE)
       /*drivers/dma/pxp/pxp_device.c*/
	imx6dl_add_imx_pxp_client();
#endif
	 //////////////////////////////
	 //TPS65185 Regulator 
#if defined(CONFIG_REGULATOR_TPS6518x)
	mxc_register_device(&tps6518x_sensor_device, NULL);
	mxc_register_device(&tps6518x_utils_device, NULL);
#endif

#if defined(CONFIG_FB_MXC_EINK_PANEL)
        //imx6_evk_setup_epdc();
	imx6dl_add_imx_epdc(&evk_epdc_data);
#endif
	
         /*about file:
         * 1). platform-imx_dvfs.c
         * 2).dvfs_core.c
         */
	imx6q_add_dvfs_core(&mx6sl_evk_dvfscore_data);

#if  defined(CONFIG_SND_SOC_IMX_AIC325X)
       mx6_evk_init_tlv325x();
#endif

#if defined(CONFIG_BT)
#ifdef SXSDMAN_BLUETOOTH_ENABLE
	if (uart4_enabled)
		uart4_init();
#else
	/* uart2 for bluetooth */
	if (uart2_enabled)
		uart2_init();
#endif

	mxc_register_device(&mxc_bt_rfkill, &mxc_bt_rfkill_data);
#endif

#if defined(CONFIG_MXS_VIIM)
        /* drivers/char/mxs_viim.c */
	imx6q_add_viim();
#endif

#if defined(CONFIG_IMX2_WDT)
       /*drivers/watchdog/imx2_wdt.c*/
	imx6q_add_imx2_wdt(0, NULL);
#endif
#if defined(CONFIG_MXC_GPU_VIV)
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
#endif

	imx6sl_add_device_buttons();

#if defined(CONFIG_KEYBOARD_IMX)
	imx6sl_add_imx_keypad(&mx6sl_evk_map_data);
#endif
	imx6q_add_busfreq();
#if defined(CONFIG_CRYPTO_DEV_DCP)
       /*about file:
         * 1). platform-imx-dcp.c
         * 2).drivers/crypto/dcp.c
         */
	imx6sl_add_dcp();
#endif

#if defined(CONFIG_HW_RANDOM_FSL_RNGC)
       /*about file:
         * 1). platform-imx-rngb.c
         * 2).drivers/char/hw_random/fsl-rngc.c
         */
	imx6sl_add_rngb();
#endif
#if defined(CONFIG_VIDEO_MXC_PXP_V4L2)
       /*about file:
         * 1). platform-imx-pxp.c
         * 2).drivers/media/video/mxc/output/mxc_pxp_v4l2.c
         */
	imx6sl_add_imx_pxp_v4l2();
#endif

#if defined(CONFIG_SND_SOC_MXC_SPDIF)
	mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0",	NULL);
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx6q_add_spdif(&mxc_spdif_data);
#endif
#if defined(CONFIG_SND_MXC_SOC_SPDIF_DAI)
	imx6q_add_spdif_dai();
#endif
#if defined(CONFIG_SND_SOC_IMX_SPDIF)
	imx6q_add_spdif_audio_device();
#endif

#if defined(CONFIG_MXS_PERFMON)
       /*about file:
         * 1). platform-imx-perfmon.c
         * 2).drivers/misc/mxs-perfmon.c	
         */

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
#endif

#if defined(CONFIG_BATTERY_CW2015)
	cw2015_gpio_init();
#endif

#if defined(CONFIG_LEDS_OZ8556)
        mxc_register_device(&leds_oz8556_device, NULL);
#endif

#if defined(CONFIG_IMX_USB_CHARGER)
	/* charger type gpio init */
	mx6_evk_charger_type_gpio_init();
#endif
	pm_power_off = mx6_evk_snvs_poweroff;

	imx6q_add_pm_imx(0, &mx6sl_evk_pm_data);

	platform_device_register(&imx6sl_system_led);
	/// mxc_register_device(&imx6sl_system_led, NULL);
}

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6sl_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = mx6_timer_init,
};

static void __init mx6_evk_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, MEMBLOCK_ALLOC_ACCESSIBLE);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
}

MACHINE_START(MX6SL_EVK, "Freescale i.MX 6SoloLite EVK Board")
	.boot_params	= MX6SL_PHYS_OFFSET + 0x100,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= mx6_evk_init,
	.timer		= &mxc_timer,
	.reserve	= mx6_evk_reserve,
MACHINE_END

////////deep standy mode

#if defined(CONFIG_PB650_DEEP_STANDY)
static int proc_keylock_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	*eof = 1;

	return snprintf(page, PAGE_SIZE, "%lu\n", iDeepStandy);
}

static int proc_keylock_write(struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char buf[16];

	if (count > sizeof(buf) -1 )
		return -EINVAL;

	if (!count)
		return 0;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	buf[count] = '\0';

	switch (buf[0]) {
		case '0':
			iDeepStandy = 0;
			break;
		default:
			iDeepStandy = 1;
			break;
	}

	///printk("%s: %d\n", __func__, iDeepStandy);

	return count;
}

int __init deepsleep_init(void)
{
	unsigned int i;
	struct proc_dir_entry *dir, *file;

	dir = proc_mkdir("keylock", NULL);
	if (!dir) {
		printk("could not create /proc/keylock\n");
		return -1;
	}

	file = create_proc_entry("lock", S_IRUGO | S_IWUGO, dir);
	if (! file) {
		printk("could not create /proc/keylock/lock\n");
		return -1;
	}

	file->data = NULL;
	file->read_proc = proc_keylock_read;
	file->write_proc = proc_keylock_write;

	return 0;
}

late_initcall(deepsleep_init);
#endif
