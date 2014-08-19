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
#if defined(CONFIG_SOC_IMX6SL)
#ifndef __MX6SL_GPIO_CFG_H__
#define __MX6SL_GPIO_CFG_H__

#define MX6_ARM2_USBOTG1_PWR    IMX_GPIO_NR(4, 0)       /* KEY_COL4 */
#define MX6_ARM2_USBOTG2_PWR    IMX_GPIO_NR(4, 2)       /* KEY_COL5 */
#define MX6_ARM2_LCD_PWR_EN	IMX_GPIO_NR(4, 3)	/* KEY_ROW5 */
#define MX6_ARM2_HEADPHONE_DET	IMX_GPIO_NR(4, 19)	/* FEC_RX_ER */
#define MX6_ARM2_SD2_WP		IMX_GPIO_NR(4, 29)	/* SD2_DAT6 */
#define MX6_ARM2_SD2_CD		IMX_GPIO_NR(5, 0)	/* SD2_DAT7 */
#define MX6_ARM2_SD3_CD		IMX_GPIO_NR(3, 22)	/* REF_CLK_32K */
#define MX6_ARM2_FEC_PWR_EN	IMX_GPIO_NR(4, 21)	/* FEC_TX_CLK */

//////////////////////////////////////////////////
////spi device
//////////////////////////////////////////////////
#if defined(CONFIG_SPI_IMX)
#define MX6_ARM2_ECSPI1_CS0	IMX_GPIO_NR(4, 11)	/* ECSPI1_SS0 */
#endif
//////////////////////////////////////////////////
////SD CARD DETECT
//////////////////////////////////////////////////
//#define MX6_ARM2_SD1_WP		IMX_GPIO_NR(4, 6)	/* KEY_COL7 */
//#define MX6_ARM2_SD1_CD		IMX_GPIO_NR(4, 7)	/* KEY_ROW7 */
#define MX6_ARM2_SD1_CD		IMX_GPIO_NR(4, 24)	/* FEC_TXD0 */
////default is high,whec check SD card had been inserted,set to low//////
#define MX6_ARM2_SD1_POWER		IMX_GPIO_NR(4, 23)/* FEC_MDC */
#define MX6SL_ARM2_KEY_COL7_CD     IMX_GPIO_NR(4, 6)/*GPIO4_IO06*/

//////////////////////////////////////////////////
/////E-inl screen////////
//////////////////////////////////////////////////
#if defined(CONFIG_FB_MXC_EINK_PANEL)
/* EPDC GPIO pins */
#define MX6SL_ARM2_EPDC_SDDO_0		IMX_GPIO_NR(1, 7)
#define MX6SL_ARM2_EPDC_SDDO_1		IMX_GPIO_NR(1, 8)
#define MX6SL_ARM2_EPDC_SDDO_2		IMX_GPIO_NR(1, 9)
#define MX6SL_ARM2_EPDC_SDDO_3		IMX_GPIO_NR(1, 10)
#define MX6SL_ARM2_EPDC_SDDO_4		IMX_GPIO_NR(1, 11)
#define MX6SL_ARM2_EPDC_SDDO_5		IMX_GPIO_NR(1, 12)
#define MX6SL_ARM2_EPDC_SDDO_6		IMX_GPIO_NR(1, 13)
#define MX6SL_ARM2_EPDC_SDDO_7		IMX_GPIO_NR(1, 14)
///8bit elink don't use below 8-15
//#define MX6SL_ARM2_EPDC_SDDO_8		IMX_GPIO_NR(1, 15)
//#define MX6SL_ARM2_EPDC_SDDO_9		IMX_GPIO_NR(1, 16)
//#define MX6SL_ARM2_EPDC_SDDO_10		IMX_GPIO_NR(1, 17)
//#define MX6SL_ARM2_EPDC_SDDO_11		IMX_GPIO_NR(1, 18)
//#define MX6SL_ARM2_EPDC_SDDO_12		IMX_GPIO_NR(1, 19)
//#define MX6SL_ARM2_EPDC_SDDO_13		IMX_GPIO_NR(1, 20)
//#define MX6SL_ARM2_EPDC_SDDO_14		IMX_GPIO_NR(1, 21)
//#define MX6SL_ARM2_EPDC_SDDO_15		IMX_GPIO_NR(1, 22)
#define MX6SL_ARM2_EPDC_GDCLK		IMX_GPIO_NR(1, 31)
#define MX6SL_ARM2_EPDC_GDSP		IMX_GPIO_NR(2, 2)
#define MX6SL_ARM2_EPDC_GDOE		IMX_GPIO_NR(2, 0)
#define MX6SL_ARM2_EPDC_GDRL		IMX_GPIO_NR(2, 1)
#define MX6SL_ARM2_EPDC_SDCLK		IMX_GPIO_NR(1, 23)
#define MX6SL_ARM2_EPDC_SDOE		IMX_GPIO_NR(1, 25)
#define MX6SL_ARM2_EPDC_SDLE		IMX_GPIO_NR(1, 24)
#define MX6SL_ARM2_EPDC_SDSHR		IMX_GPIO_NR(1, 26)
#define MX6SL_ARM2_EPDC_PWRCOM		IMX_GPIO_NR(2, 11)
#define MX6SL_ARM2_EPDC_PWRSTAT		IMX_GPIO_NR(2, 13)
#define MX6SL_ARM2_EPDC_PWRCTRL0	IMX_GPIO_NR(2, 7)
#define MX6SL_ARM2_EPDC_PWRCTRL1	IMX_GPIO_NR(2, 8)
#define MX6SL_ARM2_EPDC_PWRCTRL2	IMX_GPIO_NR(2, 9)
#define MX6SL_ARM2_EPDC_PWRCTRL3	IMX_GPIO_NR(2, 10)
#define MX6SL_ARM2_EPDC_BDR0		IMX_GPIO_NR(2, 5)
#define MX6SL_ARM2_EPDC_BDR1		IMX_GPIO_NR(2, 6)
#define MX6SL_ARM2_EPDC_SDCE0		IMX_GPIO_NR(1, 27)
#define MX6SL_ARM2_EPDC_SDCE1		IMX_GPIO_NR(1, 28)
#define MX6SL_ARM2_EPDC_SDCE2		IMX_GPIO_NR(1, 29)
#define MX6SL_ARM2_EPDC_SDCE3		IMX_GPIO_NR(1, 30)
#define MX6SL_ARM2_EPDC_PMIC_WAKE	IMX_GPIO_NR(2, 14) /* EPDC_PWRWAKEUP */
#define MX6SL_ARM2_EPDC_PMIC_INT	IMX_GPIO_NR(2, 12) /* EPDC_PWRINT */
#define MX6SL_ARM2_EPDC_VCOM		IMX_GPIO_NR(2, 3)
#endif
////////////////////////////////////////////////////////////
////////elan touch ic define
///////////////////////////////////////////////////////////
#if defined(CONFIG_TOUCHSCREEN_ELAN)
#define MX6SL_ARM2_ELAN_CE		IMX_GPIO_NR(2, 9)
#define MX6SL_ARM2_ELAN_INT		IMX_GPIO_NR(4, 25)/*IMX_GPIO_NR(2, 10)*/
#define MX6SL_ARM2_ELAN_RST		IMX_GPIO_NR(4, 18)/*IMX_GPIO_NR(4, 4)*/
#endif
////////////////////////////////////////////////////////////
////////gt813 touch ic define
///////////////////////////////////////////////////////////
#if defined(CONFIG_MXC_GT813)
#define MX6SL_ARM2_GT813_INT		IMX_GPIO_NR(4, 25)/*IMX_GPIO_NR(2, 10)*/
#define MX6SL_ARM2_GT813_RST		IMX_GPIO_NR(4, 18)/*IMX_GPIO_NR(4, 4)*/
#endif
////////////////////////////////////////////////////////////
////////MAX8903 define
///////////////////////////////////////////////////////////
#if defined(CONFIG_CHARGER_MAX8903)
#define MX6SL_CHARGE_DOK_B	  IMX_GPIO_NR(4, 13)/*ECSPI2_MOSI*/
#define MX6SL_CHARGE_UOK_B 	  IMX_GPIO_NR(4, 13)/*ECSPI2_MOSI*/
#define MX6SL_CHARGE_CHG_1_B	  IMX_GPIO_NR(4, 15)/*ECSPI2_SS0 */
#define MX6SL_CHARGE_FLT_1_B	  IMX_GPIO_NR(4, 14)/*ECSPI2_MISO*/
#endif
///////////////////////////////////////////////////////////////
/////GPIO BUTTON DEFINE
///////////////////////////////////////////////////////////////
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define MX6SL_ARM2_KEY_COL7_BACK     IMX_GPIO_NR(4, 6)/*GPIO4_IO06*/
#define MX6SL_ARM2_KEY_ROW0_MENU  IMX_GPIO_NR(3, 25)/*GPIO3_IO25*/
#define MX6SL_ARM2_KEY_ROW1_LEFT     IMX_GPIO_NR(3, 27)/*GPIO3_IO27*/
#define MX6SL_ARM2_KEY_ROW2_RIGHT     IMX_GPIO_NR(3, 29)/*GPIO3_IO29*/
#endif

#endif				/* __MX6SL_GPIO_CFG_H__ */
#endif	     /*CONFIG_SOC_IMX6SL*/
