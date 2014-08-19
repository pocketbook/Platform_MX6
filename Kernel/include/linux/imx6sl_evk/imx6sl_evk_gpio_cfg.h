/*
 * Copyright (C) 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#if defined(CONFIG_MACH_MX6SL_EVK)
#ifndef __IMX6SL_EVK_GPIO_CFG_H__
#define __IMX6SL_EVK_GPIO_CFG_H__

#define MX6SL_GPIO_HIGH    1
#define MX6SL_GPIO_LOW      0
#define MX6SL_INVALID_GPIO  -1
//////////////////////////////////////////////////
////////   HDMI GPIO        ///////////////////////////
//////////////////////////////////////////////////
#if defined(CONFIG_FB_MXC_SII902X)
#define MX6_BRD_LCD_RESET	IMX_GPIO_NR(2, 19)	/* LCD_REST */
#endif
//////////////////////////////////////////////////
/////////LCD GPIO///////////////////////////
//////////////////////////////////////////////////
#if (defined(CONFIG_IMX_HAVE_PLATFORM_IMX_ELCDIF) && defined(CONFIG_FB_MXC_ELCDIF_FB))
#define MX6_BRD_LCD_PWR_EN	IMX_GPIO_NR(4, 3)	/* KEY_ROW5 */
#endif
//////////////////////////////////////////////////
////phylib gpio
//////////////////////////////////////////////////
#if defined(CONFIG_PHYLIB)
//#define MX6_BRD_FEC_PWR_EN	IMX_GPIO_NR(4, 21)	/* FEC_TX_CLK */
#endif
//////////////////////////////////////////////////
////spi device
//////////////////////////////////////////////////
#if defined(CONFIG_SPI_IMX)
//#define MX6_BRD_ECSPI1_CS0	IMX_GPIO_NR(4, 11)	/* ECSPI1_SS0 */
#endif
//////////////////////////////////////////////////
////SD CARD DETECT
//////////////////////////////////////////////////
#if defined(CONFIG_MMC_SDHCI_PLTFM)
////default is high,whec check SD card had been inserted,set to low//////
#define MX6_BRD_SD3_CD		IMX_GPIO_NR(3, 24)/*GPIO3_IO24*/
#define MX6_BRD_EXTERNAL_CARD_POWER		IMX_GPIO_NR(3, 26)/* GPIO3_IO26 */
#endif
//////////////////////////////////////////////////
/////pfuze100 pmic init////////
//////////////////////////////////////////////////
#if defined(CONFIG_MFD_PFUZE)
#define MX6SL_REF_CLK_24M_PFUZE_INI  IMX_GPIO_NR(3, 21)/* GPIO3_IO21*/
#endif
//////////////////////////////////////////////////
/////E-ink screen////////
//////////////////////////////////////////////////
#if defined(CONFIG_FB_MXC_EINK_PANEL)
/* EPDC GPIO pins */
#define MX6SL_BRD_EPDC_SDDO_0		IMX_GPIO_NR(1, 7)
#define MX6SL_BRD_EPDC_SDDO_1		IMX_GPIO_NR(1, 8)
#define MX6SL_BRD_EPDC_SDDO_2		IMX_GPIO_NR(1, 9)
#define MX6SL_BRD_EPDC_SDDO_3		IMX_GPIO_NR(1, 10)
#define MX6SL_BRD_EPDC_SDDO_4		IMX_GPIO_NR(1, 11)
#define MX6SL_BRD_EPDC_SDDO_5		IMX_GPIO_NR(1, 12)
#define MX6SL_BRD_EPDC_SDDO_6		IMX_GPIO_NR(1, 13)
#define MX6SL_BRD_EPDC_SDDO_7		IMX_GPIO_NR(1, 14)
#define MX6SL_BRD_EPDC_GDCLK		IMX_GPIO_NR(1, 31)
#define MX6SL_BRD_EPDC_GDSP		IMX_GPIO_NR(2, 2)
#define MX6SL_BRD_EPDC_GDOE		IMX_GPIO_NR(2, 0)
#define MX6SL_BRD_EPDC_GDRL		IMX_GPIO_NR(2, 1)
#define MX6SL_BRD_EPDC_SDCLK		IMX_GPIO_NR(1, 23)
#define MX6SL_BRD_EPDC_SDOE		IMX_GPIO_NR(1, 25)
#define MX6SL_BRD_EPDC_SDLE		IMX_GPIO_NR(1, 24)
#define MX6SL_BRD_EPDC_SDSHR		IMX_GPIO_NR(1, 26)
#define MX6SL_BRD_EPDC_PWRSTAT		IMX_GPIO_NR(2, 13)
#define MX6SL_BRD_EPDC_PWRCTRL0		IMX_GPIO_NR(2, 7)
#define MX6SL_BRD_EPDC_PWRCTRL2		IMX_GPIO_NR(2, 9)
#define MX6SL_BRD_EPDC_PWRCTRL3		IMX_GPIO_NR(2, 10)
#define MX6SL_BRD_EPDC_BDR0		IMX_GPIO_NR(2, 5)
#define MX6SL_BRD_EPDC_BDR1		IMX_GPIO_NR(2, 6)
#define MX6SL_BRD_EPDC_SDCE0		IMX_GPIO_NR(1, 27)
#define MX6SL_BRD_EPDC_SDCE1		IMX_GPIO_NR(1, 28)
#define MX6SL_BRD_EPDC_SDCE2		IMX_GPIO_NR(1, 29)
#define MX6SL_BRD_EPDC_SDCE3		IMX_GPIO_NR(1, 30)
#define MX6SL_BRD_EPDC_PMIC_WAKE 	IMX_GPIO_NR(2, 14) /* EPDC_PWRWAKEUP */
#define MX6SL_BRD_EPDC_PMIC_INT		IMX_GPIO_NR(2, 12) /* EPDC_PWRINT */
#define MX6SL_BRD_EPDC_VCOM		IMX_GPIO_NR(2, 11)
#endif
////////////////////////////////////////////////////////////
////////elan touch ic define
///////////////////////////////////////////////////////////
#if defined(CONFIG_TOUCHSCREEN_CNT)
/*KEY_ROW6	Touch_PWR_EN*/
#define MX6SL_KEY_ROW6_TOUCH_PWR_EN   IMX_GPIO_NR(4, 5)/*GPIO4_IO05*/
/*KEY_COL4	Touch_RST*/
#define MX6SL_KEY_COL4_TOUCH_RST IMX_GPIO_NR(4, 0)/*GPIO4_IO00*/
/*KEY_COL5	Touch_INT */
#define MX6SL_KEY_COL5_TOUCH_INT IMX_GPIO_NR(4, 2)/* GPIO4_IO02*/

#endif
////////////////////////////////////////////////////////////
///////////////battery///////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
#if defined(CONFIG_BATTERY_CW2015)
////////////////////////////////////////////////////////////
////////CW2015  define
/////// low battery notice, init as input ,when CW2015 check the device
////// is in low battert,it will send a interrupts to cpu,default is high.
///////////////////////////////////////////////////////////
#define MX6SL_BRD_CW2015_LOWBATTERY_N    IMX_GPIO_NR(4, 7)/*GPIO4_IO07 */

#endif
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
#if defined(CONFIG_IMX_USB_CHARGER)
///////////////////////////////////////////////////
////CHARGER TYPE switch (DC-DC or USB[connect to PC])
//////////////////////////////////////////////////
/*FEC_TXD1	Charge_current_SET */
#define  MX6SL_CHARGE_CURRENT_SET   IMX_GPIO_NR(4, 16)/*GPIO4_IO16*/

#endif
///////////////////////////////////////////////////////////////
///                             WIFI define     
//////////////////////////////////////////////////////////////
#define MX6SL_BRD_WIFI_POWER     IMX_GPIO_NR(4, 4) /*GPIO4_IO04 */
#define MX6SL_BRD_WIFI_RESET   IMX_GPIO_NR(4, 6)/*GPIO4_IO06*/
#define MX6SL_BRP_WIFI_CLOCK_PIN  IMX_GPIO_NR(3, 22)/*GPIO3_IO22*/
///////////////////////////////////////////////////////////////
/// / CSI camera
//////////////////////////////////////////////////////////////
#if defined(CONFIG_VIDEO_MXC_CSI_CAMERA) || defined(CONFIG_VIDEO_MXC_CSI_CAMERA_MODULE)
/*LCD_ENABLE 	CAM_PWDN*/
#define MX6SL_BRD_CSI_PWDN	IMX_GPIO_NR(2, 16)/*GPIO2_IO16*/
/*LCD_RESET	CAM_RESET*/
#define MX6SL_BRD_CSI_RST		IMX_GPIO_NR(2, 19)/*GPIO2_IO19*/
/*EPDC_D8	CAM_2.8V_EN*/
#define MX6SL_CAMERA_28V_EN    IMX_GPIO_NR(1, 15)/*GPIO1_IO15*/
/*EPDC_D9	CAM_1.5V_EN*/
#define MX6SL_CAMERA_15V_EN   IMX_GPIO_NR(1, 16)/*GPIO1_IO16*/
/*EPDC_D10	CAM_1.8V_EN*/
#define MX6SL_CAMERA_18V_EN  IMX_GPIO_NR(1, 17)/*GPIO1_IO17*/
/*EPDC_D11	FLASH_LED_CTL*/
#define MX6SL_CAMERA_LED_CTL  IMX_GPIO_NR(1, 18)/*GPIO1_IO18*/
/*EPDC_D12	TORCH_CTL*/
#define MX6SL_CAMERA_TORCH_CTL  IMX_GPIO_NR(1, 19)/*GPIO1_IO19*/

/*Camera date pin define,we only  used these for deep standy mode*/
#define MX6SL_CAMERA_CAM_DAT2_PIN  IMX_GPIO_NR(5, 1)/*GPIO5_IO01*/
#define MX6SL_CAMERA_CAM_DAT3_PIN  IMX_GPIO_NR(4, 30)/*GPIO4_IO30*/
#define MX6SL_CAMERA_CAM_DAT4_PIN  IMX_GPIO_NR(5, 3)/*GPIO5_IO03*/
#define MX6SL_CAMERA_CAM_DAT5_PIN  IMX_GPIO_NR(4, 28)/*GPIO4_IO28*/
#define MX6SL_CAMERA_CAM_DAT6_PIN  IMX_GPIO_NR(5, 2)/*GPIO5_IO02*/
#define MX6SL_CAMERA_CAM_DAT7_PIN  IMX_GPIO_NR(4, 31)/*GPIO4_IO31*/
#define MX6SL_CAMERA_CAM_DAT8_PIN  IMX_GPIO_NR(4, 29)/*GPIO4_IO29*/
#define MX6SL_CAMERA_CAM_DAT9_PIN  IMX_GPIO_NR(5, 0)/*GPIO5_IO00*/
#endif
//////////////////////////////////////////////////////////////
////Codec  gpio defined
//////////////////////////////////////////////////
#if defined(CONFIG_SND_SOC_IMX_AIC325X)
#define MX6SL_BRD_AIC325X_RESET   IMX_GPIO_NR(1, 21)/*GPIO1_IO21*/
#define MX6SL_BRD_TLV320_HEADPHONE_DET     IMX_GPIO_NR(1, 20)/*GPIO1_IO20*/
#endif
///////////////////////////////////////////////////////////////
/////GPIO BUTTON DEFINE
///////////////////////////////////////////////////////////////
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)

#define MX6SL_EVK_POWER_OFF	IMX_GPIO_NR(4, 23)/*GPIO4_IO23*/
#define MX6SL_KEY_ROW0_MENU  IMX_GPIO_NR(3, 25)/*GPIO3_IO25*/
#define MX6SL_KEY_ROW1_HOME  IMX_GPIO_NR(3, 27)/*GPIO3_IO27*/
#define MX6SL_KEY_ROW2_BACKWARD  IMX_GPIO_NR(3, 29)/*GPIO3_IO29*/
#define MX6SL_KEY_ROW3_FORWARD  IMX_GPIO_NR(3, 31)/*GPIO3_IO31*/
#define MX6SL_KEY_ROW4_PAGEUP  IMX_GPIO_NR(4, 1)/*GPIO4_IO01*/
#define MX6SL_KEY_ROW5_PAGEDOWN IMX_GPIO_NR(4, 3)/*GPIO4_IO03*/

/*use as input Hall sensor pin*/
#define MX6SL_KEY_COL2_HALL_DET   IMX_GPIO_NR(3, 28)/*GPIO3_IO28*/
/*System LED*/
#define MX6SL_EPDC_PWRCTRL1_SYSTEM_LED   IMX_GPIO_NR(2, 8)/*GPIO2_IO08*/
/*FEC_TX_EN	CHRG_OFF */
#define MX6SL_FEC_TX_EN_CHRG_OFF   IMX_GPIO_NR(4, 22)/*GPIO4_IO22*/
/*FEC_REF_CLK	CHARGER_STATE*/
#define MX6SL_FEC_REF_CLK_CHARGER_STATE   IMX_GPIO_NR(4, 26)/*GPIO4_IO26*/
/*EPDC_D15	FL_EN */
#define MX6SL_EPDC_D15_FL_EN    IMX_GPIO_NR(1, 22)/*GPIO1_IO22*/
//////////////////////////////////////////////////////////////////
///Software version control
//////////////////////////////////////////////////////////////////
/*ECSPI1_MISO	VC0*/
#define MX6SL_ECSPI1_MISO_VC0  IMX_GPIO_NR(4, 10)/*GPIO4_IO10*/
/*ECSPI1_SS0	VC1 */
#define MX6SL_ECSPI1_SS0_VC1 IMX_GPIO_NR(4, 11)/*GPIO4_IO11*/
//////////////////////////////////////////////////////////////////

#endif

#endif/* __MX6SL_GPIO_CFG_H__ */
#endif/*CONFIG_SOC_IMX6SL*/
