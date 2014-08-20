/*
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/mx6.h>
#include <asm/arch/mx6_pins.h>
#include <asm/arch/mx6sl_pins.h>
#if defined(CONFIG_SECURE_BOOT)
#include <asm/arch/mx6_secure.h>
#endif
#include <asm/arch/iomux-v3.h>
#include <asm/arch/regs-anadig.h>
#include <asm/errno.h>
#include <imx_wdog.h>
#ifdef CONFIG_MXC_FEC
#include <miiphy.h>
#endif

#if defined(CONFIG_MXC_EPDC)
#include <lcd.h>
#endif

#ifdef CONFIG_IMX_ECSPI
#include <imx_spi.h>
#endif

#ifdef CONFIG_CMD_MMC
#include <mmc.h>
#include <fsl_esdhc.h>
#endif

#ifdef CONFIG_MXC_GPIO
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#endif

#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif

#if CONFIG_I2C_MXC
#include <i2c.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#if defined(DEBUG)
#define mdebug(fmt, ...) printf("%s(%u): " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__);
#else
#define mdebug(fmt, ...) {;};
#endif

//#ifdef CONFIG_LCD
//short lcd_cmap[256];
//#endif
short lcd_cmap[256];

static enum boot_device boot_dev;

#define CHARGE_CURRENT_SET	IMX_GPIO_NR(4, 16)
#define SYSTEM_LED		IMX_GPIO_NR(2, 8)
#define POWER_LED		IMX_GPIO_NR(4, 22)
#define USB_OTG_PWR		IMX_GPIO_NR(4, 0)
#define USB_H1_PWR		IMX_GPIO_NR(4, 2)
#define GPIO_POWER_KEY		IMX_GPIO_NR(3, 18)
#define GPIO_WIFI_EN		IMX_GPIO_NR(4, 4)
#define GPIO_TOUCH_EN		IMX_GPIO_NR(4, 5)

static int vcom_value = 0;

static void setup_i2c(unsigned int module_base);

static inline void setup_boot_device(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	uint bt_mem_ctl = (soc_sbmr & 0x000000FF) >> 4;
	uint bt_mem_type = (soc_sbmr & 0x00000008) >> 3;

	switch (bt_mem_ctl) {
	case 0x0:
		if (bt_mem_type)
			boot_dev = ONE_NAND_BOOT;
		else
			boot_dev = WEIM_NOR_BOOT;
		break;
	case 0x3:
		if (bt_mem_type)
			boot_dev = I2C_BOOT;
		else
			boot_dev = SPI_NOR_BOOT;
		break;
	case 0x4:
	case 0x5:
		boot_dev = SD_BOOT;
		break;
	case 0x6:
	case 0x7:
		boot_dev = MMC_BOOT;
		break;
	default:
		boot_dev = UNKNOWN_BOOT;
		break;
	}
}

enum boot_device get_boot_device(void)
{
	return boot_dev;
}

u32 get_board_rev(void)
{
	/*
	 * If no fuse burned for board version (i.e., 0x0),
	 * then assume latest one - RevC
	 */
	if ((fsl_system_rev & BOARD_REV_MASK) == BOARD_REV_1)
		fsl_system_rev |= BOARD_REV_4;
	return fsl_system_rev;
}

int dram_init(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;

	return 0;
}

static void setup_uart(void)
{
	/* UART1 TXD */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_UART1_TXD__UART1_TXD);

	/* UART1 RXD */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_UART1_RXD__UART1_RXD);
}

#ifdef CONFIG_NET_MULTI
int board_eth_init(bd_t *bis)
{
	int rc = -ENODEV;

	return rc;
}
#endif

#ifdef CONFIG_CMD_MMC

/* On this board, only SD3 can support 1.8V signalling
 * that is required for UHS-I mode of operation.
 * Last element in struct is used to indicate 1.8V support.
 */
struct fsl_esdhc_cfg usdhc_cfg[3] = {
	{USDHC1_BASE_ADDR, 1, 1, 1, 1},
	{USDHC2_BASE_ADDR, 1, 1, 1, 1},
	{USDHC3_BASE_ADDR, 1, 1, 1, 1},
};

iomux_v3_cfg_t usdhc1_pads[] = {
	/* 8 bit SD */
	MX6SL_PAD_SD1_CLK__USDHC1_CLK,
	MX6SL_PAD_SD1_CMD__USDHC1_CMD,
	MX6SL_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6SL_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6SL_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6SL_PAD_SD1_DAT3__USDHC1_DAT3,
	MX6SL_PAD_SD1_DAT4__USDHC1_DAT4,
	MX6SL_PAD_SD1_DAT5__USDHC1_DAT5,
	MX6SL_PAD_SD1_DAT6__USDHC1_DAT6,
	MX6SL_PAD_SD1_DAT7__USDHC1_DAT7,
};

iomux_v3_cfg_t usdhc2_pads[] = {
	/* boot SD */
	MX6SL_PAD_SD2_CLK__USDHC2_CLK,
	MX6SL_PAD_SD2_CMD__USDHC2_CMD,
	MX6SL_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6SL_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6SL_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6SL_PAD_SD2_DAT3__USDHC2_DAT3,
};

iomux_v3_cfg_t usdhc3_pads[] = {
	MX6SL_PAD_SD3_CLK__USDHC3_CLK,
	MX6SL_PAD_SD3_CMD__USDHC3_CMD,
	MX6SL_PAD_SD3_DAT0__USDHC3_DAT0,
	MX6SL_PAD_SD3_DAT1__USDHC3_DAT1,
	MX6SL_PAD_SD3_DAT2__USDHC3_DAT2,
	MX6SL_PAD_SD3_DAT3__USDHC3_DAT3,
	/* config key_col1 as GPIO */
	MX6SL_PAD_KEY_COL1__GPIO_3_26,
};

int usdhc_gpio_init(bd_t *bis)
{
	s32 status = 0;
	u32 index = 0;
	unsigned int reg;

	for (index = 0; index < CONFIG_SYS_FSL_USDHC_NUM; ++index) {
		switch (index) {
		case 0:
			mxc_iomux_v3_setup_multiple_pads(usdhc1_pads,
							ARRAY_SIZE(usdhc1_pads));
			break;
		case 1:
			mxc_iomux_v3_setup_multiple_pads(usdhc2_pads,
							ARRAY_SIZE(usdhc2_pads));
			break;
		case 2:
			mxc_iomux_v3_setup_multiple_pads(usdhc3_pads,
							ARRAY_SIZE(usdhc3_pads));
			/* output */
			reg = readl(GPIO3_BASE_ADDR + GPIO_GDIR);
			reg |= (1 << 26);
			writel(reg, GPIO3_BASE_ADDR + GPIO_GDIR);
			/* low */
			reg = readl(GPIO3_BASE_ADDR + GPIO_DR);
			reg &= ~(1 << 26);
			writel(reg, GPIO3_BASE_ADDR + GPIO_DR);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       index + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}
		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[index]);
	}

	return status;
}
#if defined(BOARD_HOLD_POWER)
/**
* power button init.
*/
int mx6sl_evk_power_button_init(void)
{
	unsigned int reg=0;
	//printf("%s %s %d \n",__FILE__,__func__,__LINE__);

#if defined(CONFIG_XRZ_IMX6SL_C037)
        mxc_iomux_v3_setup_pad(MX6SL_PAD_WDOG_B__GPIO_3_18_ONOFF);
	/* Set as input */
	reg = readl(GPIO3_BASE_ADDR + GPIO_GDIR);
	reg &= (~(1 << 18));
	//reg &= ~ONOFF_GPIO3_18_BIT_MASK;
	writel(reg, GPIO3_BASE_ADDR + GPIO_GDIR);
#elif defined(CONFIG_XRZ_IMX6SL_PB650)
        mxc_iomux_v3_setup_pad(MX6SL_PAD_FEC_MDC__GPIO_4_23);
	/* Set as input */
	reg = readl(GPIO4_BASE_ADDR + GPIO_GDIR);
	reg &= (~(1 << 23));
	writel(reg, GPIO4_BASE_ADDR + GPIO_GDIR);
#endif

     return 0;
}

/**
* charger state checked
*/
int mx6sl_evk_charger_state_init(void)
{
	unsigned int reg=0;

	printf("%s %s %d \n",__FILE__,__func__,__LINE__);

#if defined(CONFIG_XRZ_IMX6SL_C037)
        mxc_iomux_v3_setup_pad(MX6SL_PAD_ECSPI2_MOSI__GPIO_4_13);
	/* Set as input */
	reg = readl(GPIO4_BASE_ADDR + GPIO_GDIR);
	reg &= (~(1 << 13));
	writel(reg, GPIO4_BASE_ADDR + GPIO_GDIR);
#endif

     return 0;
}

int mx6sl_evk_charger_detect_check(void)
{
	unsigned int val=0;
	int b5vOrUsbInsert = 0;

	mx6sl_evk_charger_state_init();

#if defined(CONFIG_XRZ_IMX6SL_C037)
	////check whether insert Charger ////
	val = readl(GPIO4_BASE_ADDR/*+ GPIO_DR*/);
	//reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	printf("%s %s %d val=%d,(val&(1<<13))=%d\n",__FILE__,__func__,__LINE__,val,(val&(1<<13)));

	if ((val & (1 << 13)) == 0)
		b5vOrUsbInsert = 1;
	else
		b5vOrUsbInsert =0;
#endif

	return b5vOrUsbInsert;
}
#endif

static int pb650_init(void)
{
#if defined(CONFIG_XRZ_IMX6SL_PB650)
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRCTRL1__GPIO_2_8);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_FEC_TX_EN__GPIO_4_22);

	gpio_direction_output(SYSTEM_LED, 1);
	gpio_direction_output(POWER_LED, 1);
#endif

	return 0;
}

/*
********end **
 */
int board_mmc_init(bd_t *bis)
{
	if (!usdhc_gpio_init(bis))
		return 0;
	else
		return -1;
}

#ifdef CONFIG_MXC_EPDC
#ifdef CONFIG_SPLASH_SCREEN
static int setup_splash_img_addr(ulong logo_offset)
{
#ifdef CONFIG_SPLASH_IS_IN_MMC
	int mmc_dev = get_mmc_env_devno();
	ulong offset = logo_offset;
	ulong size = CONFIG_SPLASH_IMG_SIZE;
	ulong addr = 0;
	char *s = NULL;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	s = getenv("splashimage");

	if (NULL == s) {
		puts("env splashimage not found!\n");
		return -1;
	}
	addr = simple_strtoul(s, NULL, 16);

	if (!mmc) {
		printf("MMC Device %d not found\n", mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
				      blk_cnt, (u_char *) addr);
	flush_cache((ulong) addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#endif

	return 0;
}

int setup_splash_img(void)
{
	printf("Draw normal logo\n");
	return setup_splash_img_addr(CONFIG_SPLASH_IMG_OFFSET);
}

int setup_lowbatt_img(void)
{
	printf("Draw low-battery logo\n");
	return setup_splash_img_addr(CONFIG_LOWBATT_IMG_OFFSET);
}
#endif

vidinfo_t panel_info = {
	.vl_refresh = 85,
	.vl_col = 1024,
	.vl_row = 758,
	.vl_pixclock = 40000000,
	.vl_left_margin = 12,
	.vl_right_margin = 38,
	.vl_upper_margin = 6,
	.vl_lower_margin = 5,
	.vl_hsync = 20,
	.vl_vsync = 4,
	.vl_sync = 0,
	.vl_mode = 0,
	.vl_flag = 0,
	.vl_bpix = 3,
	//.vl_bpix = 4,
	//.cmap = 0,
	//.cmap : (void *)lcd_cmap,
	.cmap =(void *)lcd_cmap,
};

struct epdc_timing_params panel_timings = {
	.vscan_holdoff = 4,
	.sdoed_width = 10,
	.sdoed_delay = 20,
	.sdoez_width = 10,
	.sdoez_delay = 20,
	.gdclk_hp_offs = 428,
	.gdsp_offs = 20,
	.gdoe_offs = 0,
	.gdclk_offs = 1,
	.num_ce = 1,
};

static void setup_epdc_power(void)
{
	unsigned int reg;

	/* Setup epdc voltage */

	/* EPDC_PWRSTAT - GPIO2[13] for PWR_GOOD status */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRSTAT__GPIO_2_13);

	/* EPDC_VCOM0 - GPIO2[3] for VCOM control */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_VCOM0__GPIO_2_3);

	/* Set as output 0 */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg &= (~(1 << 3));
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);

	/* Set as output */
	reg = readl(GPIO2_BASE_ADDR + GPIO_GDIR);
	reg |= (1 << 3);
	writel(reg, GPIO2_BASE_ADDR + GPIO_GDIR);

	/* EPDC_PWRWAKEUP - GPIO2[14] for EPD PMIC WAKEUP */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRWAKEUP__GPIO_2_14);
	/* Set as output 0 */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg &= (~(1 << 14));
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);
	/* Set as output */
	reg = readl(GPIO2_BASE_ADDR + GPIO_GDIR);
	reg |= (1 << 14);
	writel(reg, GPIO2_BASE_ADDR + GPIO_GDIR);

	/* EPDC_PWRCTRL0 - GPIO2[7] for EPD PWR CTL0 */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRCTRL0__GPIO_2_7);
	/* Set as output 0 */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg &= (~(1 << 7));
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);
	/* Set as output */
	reg = readl(GPIO2_BASE_ADDR + GPIO_GDIR);
	reg |= (1 << 7);
	writel(reg, GPIO2_BASE_ADDR + GPIO_GDIR);
}


#define INF(x...) printf(x)
#define ERR(x...) printf(x)

#define MAXTEMPCOUNT 32

static int tempcount = 1;
static unsigned char temp_ranges[MAXTEMPCOUNT];

static int read3chk(unsigned char *p, int off)
{
    int v;
    unsigned char cksum;

    p += off;
    v = *p | (*(p+1)<<8) | (*(p+2)<<16);
    cksum = (*p + *(p+1) + *(p+2)) & 0xff;
    return (cksum == *(p+3)) ? v : -1;
}

static int read_waveform_data(unsigned char *wfdata, int off, unsigned char *out) {

        unsigned char *p, ec, mc, c;
	unsigned long *outl = (unsigned long *) out;
        int mode, len, n;

        ec = wfdata[0x28];
        mc = wfdata[0x29];

        p = wfdata + off;
        len = 0;
        mode = 0;

        while (1) {
                c = *p++;
                if (c == ec) break;
                if (c == mc) {
                        mode = 1-mode;
                        continue;
                }
                if (mode == 0) {
                        n = *(p++) + 1;
                } else {
                        n = 1;
                }
                len += n * 4;
                while (n-- > 0) {
			*outl++ = (c & 3) | (((c >> 2) & 3) << 8) | (((c >> 4) & 3) << 16) | (((c >> 6) & 3) << 24);
                }
        }
        return len;
}

static int unpack_waveform(unsigned char *wfdata, int inlen, unsigned char *outdata)
{
	int total[MAXTEMPCOUNT];
        unsigned char *sp;
        unsigned long *dp;
        int wf, temp, offs, offs1, offs2, len, pos, cs;
        int i, x, y, nb, n, is5bit, m0;

        // checksum
        cs = 0;
        for (i=0x08; i<0x1f; i++) cs += wfdata[i];
        if ((cs & 0xff) != wfdata[0x1f]) {
                ERR("waveform checksum error");
                goto err1;
        }

        tempcount = wfdata[0x26] + 1;
	if (tempcount > MAXTEMPCOUNT) tempcount = MAXTEMPCOUNT;
	memcpy(temp_ranges, wfdata+0x30, tempcount);

	memset(outdata, 0, 0x200);
	*((unsigned long *)(outdata+0x00)) = 0x40;
	*((unsigned long *)(outdata+0x08)) = 0x200;
	*((unsigned long *)(outdata+0x10)) = 0x200;
	*((unsigned long *)(outdata+0x18)) = 0x200;
	*((unsigned long *)(outdata+0x20)) = 0x200;
	*((unsigned long *)(outdata+0x28)) = 0x200;
	*((unsigned long *)(outdata+0x30)) = 0x200;
	*((unsigned long *)(outdata+0x38)) = 0x200;

	// empty waveform 0 - for initializing working area

	pos = 0x400;

        for (temp=0; temp<tempcount; temp++) {
		// header
		*((unsigned long *)(outdata+0x40+temp*8)) = pos;
		*((unsigned long *)(outdata+0x40+temp*8+4)) = 0;
	}
	// data
	*((unsigned long *)(outdata+pos)) = 1;
	*((unsigned long *)(outdata+pos+4)) = 0;
	memset(outdata+pos+8, 0, 0x100);

	// GC waveform

	is5bit = ((wfdata[0x24] & 0xc) == 0x4);
	pos = 0x400;
	wf = 2;

        for (temp=0; temp<tempcount; temp++) {
                offs = wfdata[0x20] + (wfdata[0x21] << 8);
                if (offs < 0x30 || offs >= inlen-4)  goto err1;
                offs1 = read3chk(wfdata, offs + wf * 4);
                if (offs1 < 0x30 || offs1 >= inlen-4) goto err1;
                offs2 = read3chk(wfdata, offs1 + temp * 4);
                if (offs2 < 0x30 || offs2 >= inlen-4) goto err1;
		sp = outdata+pos+8;
                len = read_waveform_data(wfdata, offs2, sp);
		if (len == 0) goto err1;
                if (is5bit) {
                        // 5-bit waveform
                        dp = (unsigned long *)sp;
                        nb = len;
                        while (nb >= 1024) {
                                for (y=0; y<16; y++) {
					*dp++ = sp[0]  | (sp[2] << 8)  | (sp[4] << 16)  | (sp[6] << 24);
					*dp++ = sp[8]  | (sp[10] << 8) | (sp[12] << 16) | (sp[14] << 24);
					*dp++ = sp[16] | (sp[18] << 8) | (sp[20] << 16) | (sp[22] << 24);
					*dp++ = sp[24] | (sp[26] << 8) | (sp[28] << 16) | (sp[30] << 24);
                                        sp += 64;
                                }
                                nb -= 1024;
                        }
                        len /= 4;
                }
		//
		for (nb=0; nb<len; nb+=256) {
			for (i=0; i<16; i++) {
				sp = outdata+pos+8+nb+16*i+15;
				dp = outdata+pos+8+nb+len+16*i;
				*dp = *(dp+1) = *(dp+2) = *(dp+3) = *sp | (*sp << 8) | (*sp << 16) | (*sp << 24);
			}
		}
		// prepend mode0 sequence
		// memcpy(outdata+pos+8+len, outdata+pos+8, len);
		m0 = len / 3;
		for (nb=0; nb<len; nb+=256) {
			memset(outdata+pos+8+nb, (nb<m0)?1:((nb<m0*2)?2:0), 256);
		}
		len *= 2;
		// header
		*((unsigned long *)(outdata+0x200+temp*8)) = pos;
		*((unsigned long *)(outdata+0x200+temp*8+4)) = 0;
		// data
		*((unsigned long *)(outdata+pos)) = total[temp] = len / 256;
		*((unsigned long *)(outdata+pos+4)) = 0;
		pos += (8 + len);
		pos = (pos + 7) & ~7;
        }

	// INIT waveform

        for (temp=0; temp<tempcount; temp+=2) {
		// header
		*((unsigned long *)(outdata+0x40+temp*8))    = pos;
		*((unsigned long *)(outdata+0x40+temp*8+4))  = 0;
		*((unsigned long *)(outdata+0x40+temp*8+8))  = pos;
		*((unsigned long *)(outdata+0x40+temp*8+12)) = 0;
		// data
		n =  total[temp] / 2 - 1;
		if (n < 2) n = 2;
		*((unsigned long *)(outdata+pos)) = n * 2 + 1;
		*((unsigned long *)(outdata+pos+4)) = 0;
		memset(outdata+pos+8,             0x01, 256*n);
		memset(outdata+pos+8+256*n,       0x02, 256*n);
		memset(outdata+pos+8+256*n+256*n, 0x00, 256);
		pos += (8 + 256 * (n * 2 + 1));
		pos = (pos + 7) & ~7;
	}
        return 0;

err1:
	return -1;

}

int setup_waveform_file_new(void)
{
        int mmc_dev = get_mmc_env_devno();
        ulong wf_sector = CONFIG_WAVEFORM_SECTOR + 1;
        ulong vcom_sector = CONFIG_VCOM_SECTOR;
        unsigned char *addr = (unsigned char *) CONFIG_WAVEFORM_BUF_ADDR;
        struct mmc *mmc = find_mmc_device(mmc_dev);
        uint blk_start, blk_cnt;
	int len, n, outlen, ret;
	unsigned char header[512], *data=NULL;
	unsigned char buf[4];
	char *p;

	if (!mmc) {
		printf("MMC Device %d not found\n", mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	n = mmc->block_dev.block_read(mmc_dev, wf_sector, 1, (u_char *) header);
	if (n != 1) {
		printf("Error reading waveform\n");
		return -1;
	}

        len = *((u32 *)(header+4));
        if (len < 1024 || len > 1024000) {
                ERR("wrong waveform size (%d)\n", len);
                goto err2;
        }
        INF("waveform is at %x, size %d\n", (unsigned long) addr, len);

        data = malloc(len);
        if (data == NULL) {
                ERR("could not allocate memory\n");
                goto err2;
        }

        memcpy(data, header, 512);
        n = mmc->block_dev.block_read(mmc_dev, wf_sector+1, (len+1) / 512, data+512);
        if (n != (len+1) / 512) {
                ERR("could not read waveform file (%d)\n", n);
                goto err2;
        }

        if (unpack_waveform(data, len, addr) != 0) {
                ERR("could not unpack waveform\n");
                goto err2;
        }
	free(data);

	n = mmc->block_dev.block_read(mmc_dev, vcom_sector, 1, (u_char *) header);
	if (n == 1) {
		p = (char *) header;
		if (*p == '-') p++;
		vcom_value = simple_strtoul(p, NULL, 10);
		if (vcom_value < 500 || vcom_value > 4000) vcom_value = 0;
	}

	return 0;

err2:
	if (data) free(data);
	return -1;

}

int enable_panel_3v3(int on)
{
	unsigned char buf[4];
	int ret = i2c_read(CONFIG_SYS_I2C_TPS65185_ADDR, 0x01, 1, buf, 1);
	if (on) {
		buf[0] |= (1 << 5); // enable 3v3
	} else {
		buf[0] &= ~(1 << 5); // disable 3v3
	}
	if (ret == 0) ret = i2c_write(CONFIG_SYS_I2C_TPS65185_ADDR, 0x01, 1, buf, 1);
	if (ret != 0) printf("could not enable 3V3\n");
	return ret;
}

int setup_vcom(int on)
{
	unsigned char buf[4];
	int ret = -1;

	if (on) {
		if (vcom_value != 0) {
			buf[0] = (vcom_value / 10) & 0xff;
			buf[1] = (vcom_value / 10) >> 8;
			ret = i2c_write(CONFIG_SYS_I2C_TPS65185_ADDR, 0x03, 1, buf, 1);
			if (ret == 0) ret = i2c_write(CONFIG_SYS_I2C_TPS65185_ADDR, 0x04, 1, buf+1, 1);
			if (ret == 0) {
				printf("VCOM=-%d\n", vcom_value);
			} else {
				printf("could not set VCOM\n");
			}
		} else {
			printf("VCOM value not set\n");
		}
	}
	ret = i2c_read(CONFIG_SYS_I2C_TPS65185_ADDR, 0x01, 1, buf, 1);
	if (on) {
		buf[0] |= (1 << 4); // enable vcom
	} else {
		buf[0] &= ~(1 << 4); // disable vcom
	}
	if (ret == 0) ret = i2c_write(CONFIG_SYS_I2C_TPS65185_ADDR, 0x01, 1, buf, 1);
	if (ret != 0) printf("could not enable VCOM\n");
	return ret;
}

int get_temp_index()
{
	int temp = 25;
	unsigned char buf[4];
	int i, ret;

	buf[0] = 0x80;
	ret = i2c_write(CONFIG_SYS_I2C_TPS65185_ADDR, 0x0d, 1, buf, 1);
	if (ret == 0) {
		for (i=20; i>0; i--) {
			i2c_read(CONFIG_SYS_I2C_TPS65185_ADDR, 0x0d, 1, buf, 1);
			if ((buf[0] & 0x80) == 0) break;
			udelay(1000);
		}
		if (i == 0) ret = -1;
	}
	if (ret == 0) ret = i2c_read(CONFIG_SYS_I2C_TPS65185_ADDR, 0x00, 1, buf, 1);
	if (ret == 0) {
		temp = (buf[0] <= 127) ? buf[0] : 256-buf[0];
		for (i=0; i<tempcount-1; i++) {
			if (temp < temp_ranges[i+1]) break;
		}
		printf("temperature=%d index=%d\n", temp, i);
		return i;
	} else {
		printf("could not read temperature\n");
		return tempcount / 2;
	}
}

int setup_waveform_file(void)
{
#ifdef CONFIG_WAVEFORM_FILE_IN_MMC
	int mmc_dev = get_mmc_env_devno();
	ulong offset = CONFIG_WAVEFORM_FILE_OFFSET;
	ulong size = CONFIG_WAVEFORM_FILE_SIZE;
	ulong addr = CONFIG_WAVEFORM_BUF_ADDR;
	struct mmc *mmc = find_mmc_device(mmc_dev);
	uint blk_start, blk_cnt, n;

	if (!mmc) {
		printf("MMC Device %d not found\n", mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

	blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
	blk_cnt = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;
	n = mmc->block_dev.block_read(mmc_dev, blk_start,
				      blk_cnt, (u_char *) addr);
	flush_cache((ulong) addr, blk_cnt * mmc->read_bl_len);

	return (n == blk_cnt) ? 0 : -1;
#else
	return -1;
#endif
}

static void epdc_enable_pins(void)
{
	/* epdc iomux settings */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D0__EPDC_SDDO_0);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D1__EPDC_SDDO_1);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D2__EPDC_SDDO_2);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D3__EPDC_SDDO_3);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D4__EPDC_SDDO_4);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D5__EPDC_SDDO_5);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D6__EPDC_SDDO_6);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D7__EPDC_SDDO_7);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDCLK__EPDC_GDCLK);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDSP__EPDC_GDSP);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDOE__EPDC_GDOE);
	//mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDRL__EPDC_GDRL);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCLK__EPDC_SDCLK);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDOE__EPDC_SDOE);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDLE__EPDC_SDLE);
	//mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDSHR__EPDC_SDSHR);
	//mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_BDR0__EPDC_BDR_0);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCE0__EPDC_SDCE_0);
	//mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCE1__EPDC_SDCE_1);
	//mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCE2__EPDC_SDCE_2);
}

static void epdc_disable_pins(void)
{
	/* Configure MUX settings for EPDC pins to GPIO  and drive to 0 */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D0__GPIO_1_7);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D1__GPIO_1_8);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D2__GPIO_1_9);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D3__GPIO_1_10);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D4__GPIO_1_11);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D5__GPIO_1_12);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D6__GPIO_1_13);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D7__GPIO_1_14);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDCLK__GPIO_1_31);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDSP__GPIO_2_2);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDOE__GPIO_2_0);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_GDRL__GPIO_2_1);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCLK__GPIO_1_23);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDOE__GPIO_1_25);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDLE__GPIO_1_24);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDSHR__GPIO_1_26);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_BDR0__GPIO_2_5);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCE0__GPIO_1_27);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCE1__GPIO_1_28);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_SDCE2__GPIO_1_29);
}

static void setup_epdc(void)
{
	unsigned int reg;

	/*** epdc Maxim PMIC settings ***/

	/* EPDC PWRSTAT - GPIO2[13] for PWR_GOOD status */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRSTAT__GPIO_2_13);

	/* EPDC VCOM0 - GPIO2[3] for VCOM control */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRCOM__GPIO_2_11);

	/* UART4 TXD - GPIO2[14] for EPD PMIC WAKEUP */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRWAKEUP__GPIO_2_14);

	/* EIM_A18 - GPIO2[7] for EPD PWR CTL0 */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_PWRCTRL0__GPIO_2_7);

	/*** Set pixel clock rates for EPDC ***/

	/* EPDC AXI clk from PFD_400M, set to 396/2 = 198MHz */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CHSCCDR);
	reg &= ~0x3F000;
	reg |= (0x4 << 15) | (1 << 12);
	writel(reg, CCM_BASE_ADDR + CLKCTL_CHSCCDR);

	/* EPDC AXI clk enable */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR3);
	reg |= 0x0030;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR3);

	/* EPDC PIX clk from PFD_540M, set to 540/4/5 = 27MHz */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CSCDR2);
	reg &= ~0x03F000;
	reg |= (0x5 << 15) | (4 << 12);
	writel(reg, CCM_BASE_ADDR + CLKCTL_CSCDR2);

	reg = readl(CCM_BASE_ADDR + CLKCTL_CBCMR);
	reg &= ~0x03800000;
	reg |= (0x3 << 23);
	writel(reg, CCM_BASE_ADDR + CLKCTL_CBCMR);

	/* EPDC PIX clk enable */
	reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR3);
	reg |= 0x0C00;
	writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR3);

	panel_info.epdc_data.working_buf_addr = CONFIG_WORKING_BUF_ADDR;
	panel_info.epdc_data.waveform_buf_addr = CONFIG_WAVEFORM_BUF_ADDR;

	panel_info.epdc_data.wv_modes.mode_init = 0;
	panel_info.epdc_data.wv_modes.mode_du = 1;
	panel_info.epdc_data.wv_modes.mode_gc4 = 3;
	panel_info.epdc_data.wv_modes.mode_gc8 = 2;
	panel_info.epdc_data.wv_modes.mode_gc16 = 2;
	panel_info.epdc_data.wv_modes.mode_gc32 = 2;

	panel_info.epdc_data.epdc_timings = panel_timings;

	setup_epdc_power();

	/* Assign fb_base */
	gd->fb_base = CONFIG_FB_BASE;
}

void epdc_power_on(void)
{
	unsigned int i, reg;

	/* Set PMIC Wakeup to high - enable Display power */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg |= (1 << 14);
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);
	udelay(1000);

        setup_i2c(CONFIG_SYS_I2C_PORT);
	i2c_probe(CONFIG_SYS_I2C_TPS65185_ADDR);

	enable_panel_3v3(1);
	
	/* Enable epdc signal pin */
	epdc_enable_pins();

	/* Set EPD_PWR_CTL0 to high - enable EINK_VHIGH */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg |= (1 << 7);
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);
	udelay(1000);

	/* Wait for PWRGOOD == 1 */
	for (i=5000; i>0; --i) {
		reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
		if (!(reg & (1 << 13)))
			break;

		udelay(100);
	}
	if (i == 0) printf("timeout waiting for pwrgood\n");

	setup_vcom(1);

	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);

	udelay(500);
}

void epdc_power_off(void)
{
	unsigned int reg;
	int ii;

	printf("EPDC poweroff\n");

	/* Disable VCOM */
	setup_vcom(0);

	/* Set PMIC Wakeup to low - disable Display power */
	reg = readl(GPIO2_BASE_ADDR + GPIO_DR);
	reg &= ~(1 << 7);
	writel(reg, GPIO2_BASE_ADDR + GPIO_DR);

	/* 50ms delay */
	for(ii=0; ii<100; ii++)
		udelay(500);
	epdc_disable_pins();

	enable_panel_3v3(0);
	udelay(1000);

}
#endif

/* For DDR mode operation, provide target delay parameter for each SD port.
 * Use cfg->esdhc_base to distinguish the SD port #. The delay for each port
 * is dependent on signal layout for that particular port.  If the following
 * CONFIG is not defined, then the default target delay value will be used.
 */
#ifdef CONFIG_GET_DDR_TARGET_DELAY
u32 get_ddr_delay(struct fsl_esdhc_cfg *cfg)
{
	/* No delay required on EVK board SD ports */
	return 0;
}
#endif
#endif

#ifdef CONFIG_IMX_ECSPI
s32 spi_get_cfg(struct imx_spi_dev_t *dev)
{
	switch (dev->slave.cs) {
	case 0:
		/* SPI-NOR */
		dev->base = ECSPI1_BASE_ADDR;
		dev->freq = 25000000;
		dev->ss_pol = IMX_SPI_ACTIVE_LOW;
		dev->ss = 0;
		dev->fifo_sz = 64 * 4;
		dev->us_delay = 0;
		break;
	default:
		printf("Invalid Bus ID!\n");
		break;
	}

	return 0;
}

void spi_io_init(struct imx_spi_dev_t *dev)
{
	u32 reg;

	switch (dev->base) {
	case ECSPI1_BASE_ADDR:
		/* Enable clock */
		reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR1);
		reg |= 0x3;
		writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR1);
		/* SCLK */
		mxc_iomux_v3_setup_pad(MX6SL_PAD_ECSPI1_SCLK__ECSPI1_SCLK);

		/* MISO */
		mxc_iomux_v3_setup_pad(MX6SL_PAD_ECSPI1_MISO__ECSPI1_MISO);

		/* MOSI */
		mxc_iomux_v3_setup_pad(MX6SL_PAD_ECSPI1_MOSI__ECSPI1_MOSI);

		if (dev->ss == 0)
			mxc_iomux_v3_setup_pad
			    (MX6SL_PAD_ECSPI1_SS0__ECSPI1_SS0);
		break;
	case ECSPI2_BASE_ADDR:
	case ECSPI3_BASE_ADDR:
		/* ecspi2-3 fall through */
		break;
	default:
		break;
	}
}
#endif

#ifdef CONFIG_MXC_FEC
iomux_v3_cfg_t enet_pads[] = {
	/* LAN8720A */
	MX6SL_PAD_FEC_MDIO__FEC_MDIO,
	MX6SL_PAD_FEC_MDC__FEC_MDC,
	MX6SL_PAD_FEC_RXD0__FEC_RDATA_0,
	MX6SL_PAD_FEC_RXD1__FEC_RDATA_1,
	MX6SL_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX6SL_PAD_FEC_TXD0__FEC_TDATA_0,
	MX6SL_PAD_FEC_TXD1__FEC_TDATA_1,
	MX6SL_PAD_FEC_TX_EN__FEC_TX_EN,
#ifdef CONFIG_FEC_CLOCK_FROM_ANATOP
	MX6SL_PAD_FEC_REF_CLK__FEC_REF_OUT,	/* clock from anatop */
#else
	MX6SL_PAD_FEC_REF_CLK__GPIO_4_26,	/* clock from OSC */
#endif

	/*
	 * Since FEC_RX_ER is not connected with PHY(LAN8720A), we need
	 * either configure FEC_RX_ER PAD to other mode than FEC_RX_ER,
	 * or configure FEC_RX_ER PAD to FEC_RX_ER but need pull it down,
	 * otherwise, FEC MAC will report CRC error always. We configure
	 * FEC_RX_ER PAD to GPIO mode here.
	 */

	MX6SL_PAD_FEC_RX_ER__GPIO_4_19,
	MX6SL_PAD_FEC_TX_CLK__GPIO_4_21,	/* Phy power enable */
};

void enet_board_init(void)
{
	unsigned int reg;
	mxc_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/*set GPIO4_26 input as FEC clock */
	reg = readl(GPIO4_BASE_ADDR + 0x04);
	reg &= ~(1 << 26);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	/* phy power enable and reset: gpio4_21 */
	/* DR: High Level off: Power Off */
	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg &= ~(1 << 21);
	writel(reg, GPIO4_BASE_ADDR + 0x0);

	/* DIR: output */
	reg = readl(GPIO4_BASE_ADDR + 0x4);
	reg |= (1 << 21);
	writel(reg, GPIO4_BASE_ADDR + 0x4);

	udelay(1000);

	/* DR: High Level on: Power On */
	reg = readl(GPIO4_BASE_ADDR + 0x0);
	reg |= (1 << 21);
	writel(reg, GPIO4_BASE_ADDR + 0x0);

	/* wait RC ms for hw reset */
	udelay(500);
}

#define ANATOP_PLL_LOCK                 0x80000000
#define ANATOP_PLL_PWDN_MASK            0x00001000
#define ANATOP_PLL_BYPASS_MASK          0x00010000
#define ANATOP_FEC_PLL_ENABLE_MASK      0x00002000

static int setup_fec(void)
{
	u32 reg = 0;
	s32 timeout = 100000;

	/* get enet tx reference clk from internal clock from anatop
	 * GPR1[14] = 0, GPR1[18:17] = 00
	 */
	reg = readl(IOMUXC_BASE_ADDR + 0x4);
	reg &= ~(0x3 << 17);
	reg &= ~(0x1 << 14);
	writel(reg, IOMUXC_BASE_ADDR + 0x4);

#ifdef CONFIG_FEC_CLOCK_FROM_ANATOP
	/* Enable PLLs */
	reg = readl(ANATOP_BASE_ADDR + 0xe0);	/* ENET PLL */
	if ((reg & ANATOP_PLL_PWDN_MASK) || (!(reg & ANATOP_PLL_LOCK))) {
		reg &= ~ANATOP_PLL_PWDN_MASK;
		writel(reg, ANATOP_BASE_ADDR + 0xe0);
		while (timeout--) {
			if (readl(ANATOP_BASE_ADDR + 0xe0) & ANATOP_PLL_LOCK)
				break;
		}
		if (timeout <= 0)
			return -1;
	}

	/* Enable FEC clock */
	reg |= ANATOP_FEC_PLL_ENABLE_MASK;
	reg &= ~ANATOP_PLL_BYPASS_MASK;
	writel(reg, ANATOP_BASE_ADDR + 0xe0);
#endif
	return 0;
}
#endif

#ifdef CONFIG_I2C_MXC
#define I2C1_SCL_GPIO3_12_BIT_MASK  (1 << 12)
#define I2C1_SDA_GPIO3_13_BIT_MASK  (1 << 13)
#define I2C2_SCL_GPIO3_14_BIT_MASK  (1 << 14)
#define I2C2_SDA_GPIO3_15_BIT_MASK  (1 << 15)

static void setup_i2c(unsigned int module_base)
{
	unsigned int reg;

	switch (module_base) {
	case I2C1_BASE_ADDR:
		/* i2c1 SDA */
		mxc_iomux_v3_setup_pad(MX6SL_PAD_I2C1_SDA__I2C1_SDA);
		/* i2c1 SCL */
		mxc_iomux_v3_setup_pad(MX6SL_PAD_I2C1_SCL__I2C1_SCL);

		/* Enable i2c clock */
		reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR2);
		reg |= 0xC0;
		writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR2);

		break;
	case I2C2_BASE_ADDR:
		/* i2c2 SDA */
		mxc_iomux_v3_setup_pad(MX6SL_PAD_I2C2_SDA__I2C2_SDA);

		/* i2c2 SCL */
		mxc_iomux_v3_setup_pad(MX6SL_PAD_I2C2_SCL__I2C2_SCL);

		/* Enable i2c clock */
		reg = readl(CCM_BASE_ADDR + CLKCTL_CCGR2);
		reg |= 0x300;
		writel(reg, CCM_BASE_ADDR + CLKCTL_CCGR2);

		break;
	default:
		printf("Invalid I2C base: 0x%x\n", module_base);
		break;
	}
}

/* Note: udelay() is not accurate for i2c timing */
static void __udelay(int time)
{
	int i, j;

	for (i = 0; i < time; i++) {
		for (j = 0; j < 200; j++) {
			asm("nop");
			asm("nop");
		}
	}
}

static void mx6sl_i2c_gpio_scl_direction(int bus, int output)
{
	u32 reg;

	switch (bus) {
	case 1:
		mxc_iomux_v3_setup_pad(MX6SL_PAD_I2C1_SCL__GPIO_3_12);
		reg = readl(GPIO3_BASE_ADDR + GPIO_GDIR);
		if (output)
			reg |= I2C1_SCL_GPIO3_12_BIT_MASK;
		else
			reg &= ~I2C1_SCL_GPIO3_12_BIT_MASK;
		writel(reg, GPIO3_BASE_ADDR + GPIO_GDIR);
		break;
	case 2:
		mxc_iomux_v3_setup_pad(MX6SL_PAD_I2C2_SCL__GPIO_3_14);
		reg = readl(GPIO3_BASE_ADDR + GPIO_GDIR);
		if (output)
			reg |= I2C2_SCL_GPIO3_14_BIT_MASK;
		else
			reg &= ~I2C2_SCL_GPIO3_14_BIT_MASK;
		writel(reg, GPIO3_BASE_ADDR + GPIO_GDIR);
		break;
	}
}

/* set 1 to output, sent 0 to input */
static void mx6sl_i2c_gpio_sda_direction(int bus, int output)
{
	u32 reg;

	switch (bus) {
	case 1:
		mxc_iomux_v3_setup_pad(MX6SL_PAD_I2C1_SDA__GPIO_3_13);
		reg = readl(GPIO3_BASE_ADDR + GPIO_GDIR);
		if (output)
			reg |= I2C1_SDA_GPIO3_13_BIT_MASK;
		else
			reg &= ~I2C1_SDA_GPIO3_13_BIT_MASK;
		writel(reg, GPIO3_BASE_ADDR + GPIO_GDIR);
		break;
	case 2:
		mxc_iomux_v3_setup_pad(MX6SL_PAD_I2C2_SDA__GPIO_3_15);
		reg = readl(GPIO3_BASE_ADDR + GPIO_GDIR);
		if (output)
			reg |= I2C2_SDA_GPIO3_15_BIT_MASK;
		else
			reg &= ~I2C2_SDA_GPIO3_15_BIT_MASK;
		writel(reg, GPIO3_BASE_ADDR + GPIO_GDIR);
		break;
	}
}

/* set 1 to high 0 to low */
static void mx6sl_i2c_gpio_scl_set_level(int bus, int high)
{
	u32 reg;

	switch (bus) {
	case 1:
		reg = readl(GPIO3_BASE_ADDR + GPIO_DR);
		if (high)
			reg |= I2C1_SCL_GPIO3_12_BIT_MASK;
		else
			reg &= ~I2C1_SCL_GPIO3_12_BIT_MASK;
		writel(reg, GPIO3_BASE_ADDR + GPIO_DR);
		break;
	case 2:
		reg = readl(GPIO3_BASE_ADDR + GPIO_DR);
		if (high)
			reg |= I2C2_SCL_GPIO3_14_BIT_MASK;
		else
			reg &= ~I2C2_SCL_GPIO3_14_BIT_MASK;
		writel(reg, GPIO3_BASE_ADDR + GPIO_DR);
		break;
	}
}

/* set 1 to high 0 to low */
static void mx6sl_i2c_gpio_sda_set_level(int bus, int high)
{
	u32 reg;

	switch (bus) {
	case 1:
		reg = readl(GPIO3_BASE_ADDR + GPIO_DR);
		if (high)
			reg |= I2C1_SDA_GPIO3_13_BIT_MASK;
		else
			reg &= ~I2C1_SDA_GPIO3_13_BIT_MASK;
		writel(reg, GPIO3_BASE_ADDR + GPIO_DR);
		break;
	case 2:
		reg = readl(GPIO3_BASE_ADDR + GPIO_DR);
		if (high)
			reg |= I2C2_SDA_GPIO3_15_BIT_MASK;
		else
			reg &= ~I2C2_SDA_GPIO3_15_BIT_MASK;
		writel(reg, GPIO3_BASE_ADDR + GPIO_DR);
		break;
	}
}

static int mx6sl_i2c_gpio_check_sda(int bus)
{
	u32 reg;
	int result = 0;

	switch (bus) {
	case 1:
		reg = readl(GPIO3_BASE_ADDR + GPIO_PSR);
		result = !!(reg & I2C1_SDA_GPIO3_13_BIT_MASK);
		break;
	case 2:
		reg = readl(GPIO3_BASE_ADDR + GPIO_PSR);
		result = !!(reg & I2C2_SDA_GPIO3_15_BIT_MASK);
		break;
	}

	return result;
}

 /* Random reboot cause i2c SDA low issue:
  * the i2c bus busy because some device pull down the I2C SDA
  * line. This happens when Host is reading some byte from slave, and
  * then host is reset/reboot. Since in this case, device is
  * controlling i2c SDA line, the only thing host can do this give the
  * clock on SCL and sending NAK, and STOP to finish this
  * transaction.
  *
  * How to fix this issue:
  * detect if the SDA was low on bus send 8 dummy clock, and 1
  * clock + NAK, and STOP to finish i2c transaction the pending
  * transfer.
  */
int i2c_bus_recovery(void)
{
	int i, bus, result = 0;

	for (bus = 1; bus <= 2; bus++) {
		mx6sl_i2c_gpio_sda_direction(bus, 0);

		if (mx6sl_i2c_gpio_check_sda(bus) == 0) {
			printf("i2c: I2C%d SDA is low, start i2c recovery...\n",
			       bus);
			mx6sl_i2c_gpio_scl_direction(bus, 1);
			mx6sl_i2c_gpio_scl_set_level(bus, 1);
			__udelay(10000);

			for (i = 0; i < 9; i++) {
				mx6sl_i2c_gpio_scl_set_level(bus, 1);
				__udelay(5);
				mx6sl_i2c_gpio_scl_set_level(bus, 0);
				__udelay(5);
			}

			/* 9th clock here, the slave should already
			   release the SDA, we can set SDA as high to
			   a NAK. */
			mx6sl_i2c_gpio_sda_direction(bus, 1);
			mx6sl_i2c_gpio_sda_set_level(bus, 1);
			__udelay(1);	/* Pull up SDA first */
			mx6sl_i2c_gpio_scl_set_level(bus, 1);
			__udelay(5);	/* plus pervious 1 us */
			mx6sl_i2c_gpio_scl_set_level(bus, 0);
			__udelay(5);
			mx6sl_i2c_gpio_sda_set_level(bus, 0);
			__udelay(5);
			mx6sl_i2c_gpio_scl_set_level(bus, 1);
			__udelay(5);
			/* Here: SCL is high, and SDA from low to high, it's a
			 * stop condition */
			mx6sl_i2c_gpio_sda_set_level(bus, 1);
			__udelay(5);

			mx6sl_i2c_gpio_sda_direction(bus, 0);
			if (mx6sl_i2c_gpio_check_sda(bus) == 1)
				printf("I2C%d Recovery success\n", bus);
			else {
				printf
				    ("I2C%d Recovery failed, I2C1 SDA still low!!!\n",
				     bus);
				result |= 1 << bus;
			}
		}
		/* configure back to i2c */
		switch (bus) {
		case 1:
			setup_i2c(I2C1_BASE_ADDR);
			break;
		case 2:
			setup_i2c(I2C2_BASE_ADDR);
			break;
		}
	}

	return result;
}

static int setup_pmic_voltages(void)
{
	unsigned char value, rev_id = 0;
	int r;
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	if (!i2c_probe(0x8)) {
		if (i2c_read(0x8, 0, 1, &value, 1)) {
			printf("Read device ID error!\n");
			return -1;
		}
		if (i2c_read(0x8, 3, 1, &rev_id, 1)) {
			printf("Read Rev ID error!\n");
			return -1;
		}
		printf("Found PFUZE100! deviceid=%x,revid=%x\n", value, rev_id);
/*
		r = 0;
		// SW1AB
		r |= i2c_read(0x08, 0x20, 1, &value, 1);
		r |= i2c_write(0x08, 0x21, 1, &value, 1);
		value = 0x08; // sleep=off,norm/stby=aps/aps
		r |= i2c_write(0x08, 0x23, 1, &value, 1);
		// SW1C
		r |= i2c_read(0x08, 0x2e, 1, &value, 1);
		r |= i2c_write(0x08, 0x2f, 1, &value, 1);
		value = 0x08; // sleep=off,norm/stby=aps/aps
		r |= i2c_write(0x08, 0x31, 1, &value, 1);
		// SW2
		r |= i2c_read(0x08, 0x35, 1, &value, 1);
		r |= i2c_write(0x08, 0x36, 1, &value, 1);
		value = 0x04; // sleep=off,norm/stby=aps/off
		r |= i2c_write(0x08, 0x38, 1, &value, 1);
		// SW3A
		r |= i2c_read(0x08, 0x3c, 1, &value, 1);
		r |= i2c_write(0x08, 0x3d, 1, &value, 1);
		value = 0x04; // sleep=off,norm/stby=aps/off
		r |= i2c_write(0x08, 0x3f, 1, &value, 1);
		// SW3B
		r |= i2c_read(0x08, 0x43, 1, &value, 1);
		r |= i2c_write(0x08, 0x44, 1, &value, 1);
		value = 0x04; // sleep=off,norm/stby=aps/off
		r |= i2c_write(0x08, 0x46, 1, &value, 1);
		// SW4
		r |= i2c_read(0x08, 0x4a, 1, &value, 1);
		r |= i2c_write(0x08, 0x4b, 1, &value, 1);
		value = 0x04; // sleep=off,norm/stby=aps/off
		r |= i2c_write(0x08, 0x4d, 1, &value, 1);
		// VGEN1
		r |= i2c_read(0x08, 0x6c, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x6c, 1, &value, 1);
		// VGEN2
		r |= i2c_read(0x08, 0x6d, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x6d, 1, &value, 1);
		// VGEN3
		r |= i2c_read(0x08, 0x6e, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x6e, 1, &value, 1);
		// VGEN4
		r |= i2c_read(0x08, 0x6f, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x6f, 1, &value, 1);
		// VGEN5
		r |= i2c_read(0x08, 0x70, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x70, 1, &value, 1);
		// VGEN6
		r |= i2c_read(0x08, 0x71, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x71, 1, &value, 1);

		printf("(A) r=%d\n", r);
*/
	}

	return 0;
}

static void pmic_standby(void)
{
	unsigned char value;
	int r = 0;

	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
	if (!i2c_probe(0x8)) {
		value = 0x00;
		//r |= i2c_write(0x08, 0x6d, 1, &value, 1); // VGEN2
		//r |= i2c_write(0x08, 0x6e, 1, &value, 1); // VGEN3
		//r |= i2c_write(0x08, 0x6f, 1, &value, 1); // VGEN4
		//r |= i2c_write(0x08, 0x70, 1, &value, 1); // VGEN5
		//r |= i2c_write(0x08, 0x71, 1, &value, 1); // VGEN6
		//r |= i2c_write(0x08, 0x4d, 1, &value, 1); // SW4
		// SW2
		//value = 0x04; // sleep=off,norm/stby=aps/off
		//r |= i2c_write(0x08, 0x38, 1, &value, 1);
		// SW4
		value = 0x04; // sleep=off,norm/stby=aps/off
		r |= i2c_write(0x08, 0x4d, 1, &value, 1);
		// VGEN1
		r |= i2c_read(0x08, 0x6c, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x6c, 1, &value, 1);
		// VGEN2
		r |= i2c_read(0x08, 0x6d, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x6d, 1, &value, 1);
		// VGEN3
		r |= i2c_read(0x08, 0x6e, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x6e, 1, &value, 1);
		// VGEN4
		r |= i2c_read(0x08, 0x6f, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x6f, 1, &value, 1);
		// VGEN5
		r |= i2c_read(0x08, 0x70, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x70, 1, &value, 1);
		// VGEN6
		r |= i2c_read(0x08, 0x71, 1, &value, 1);
		value = (value & 0x1f) | 0x20;
		r |= i2c_write(0x08, 0x71, 1, &value, 1);

		//value = 0x22;
		//r |= i2c_write(0x08, 0x20, 1, &value, 1); // SW3AB
		//r |= i2c_write(0x08, 0x21, 1, &value, 1); // SW3AB
		//r |= i2c_write(0x08, 0x2e, 1, &value, 1); // SW3C
		//r |= i2c_write(0x08, 0x2f, 1, &value, 1); // SW3C
		// SW2
		//value = 0x04; // sleep=off,norm/stby=aps/off
		//r |= i2c_write(0x08, 0x38, 1, &value, 1);
		//printf("(C) r=%d\n", r);
	} else {
		printf("PFUZE not found!\n");
	}
}

#endif

#ifdef CONFIG_MXC_KPD
int setup_mxc_kpd(void)
{
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_COL0__KPP_COL_0);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_COL1__KPP_COL_1);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_COL2__KPP_COL_2);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_COL3__KPP_COL_3);

	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_ROW0__KPP_ROW_0);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_ROW1__KPP_ROW_1);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_ROW2__KPP_ROW_2);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_ROW3__KPP_ROW_3);

	return 0;
}

int check_powerkey_pressed(void)
{
	mxc_iomux_v3_setup_pad(MX6SL_PAD_WDOG_B__GPIO_3_18);
	gpio_direction_input(GPIO_POWER_KEY);
	udelay(5);
	if (gpio_get_value(GPIO_POWER_KEY) == 0)
		return 1;
	return 0;
}

#endif

#define MX6SL_KEY_ROW2_BACKWARD  IMX_GPIO_NR(3, 29)/*GPIO3_IO29*/
#define MX6SL_KEY_ROW3_FORWARD  IMX_GPIO_NR(3, 31)/*GPIO3_IO31*/

int check_update_keys(void)
{
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_ROW2__GPIO_3_29);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_ROW3__GPIO_3_31);

	gpio_direction_input(MX6SL_KEY_ROW2_BACKWARD);
	gpio_direction_input(MX6SL_KEY_ROW3_FORWARD);

	udelay(5);

	if (gpio_get_value(MX6SL_KEY_ROW2_BACKWARD) == 1 && gpio_get_value(MX6SL_KEY_ROW3_FORWARD) == 1)
		return 1;

	return 0;
}

int board_init(void)
{
/*
 * need set Power Supply Glitch to 0x41736166
 * and need clear Power supply Glitch Detect bit
 * when POR or reboot or power on Otherwise system
 * could not be power off anymore;
 * need to set SNVS work at DUMP mode;
 * */
	u32 reg;
	writel(0x41736166, SNVS_BASE_ADDR + 0x64);/*set LPPGDR*/
	udelay(10);
	reg = readl(SNVS_BASE_ADDR + 0x4c);
	reg |= (1 << 3);
	writel(reg, SNVS_BASE_ADDR + 0x4c);/*clear LPSR*/

	mxc_iomux_v3_init((void *)IOMUXC_BASE_ADDR);
	setup_boot_device();
	fsl_set_system_rev();

	/* board id for linux */
	gd->bd->bi_arch_number = MACH_TYPE_MX6SL_EVK;

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	wdog_preconfig(WDOG1_BASE_ADDR);

	setup_uart();

#ifdef CONFIG_MXC_FEC
	setup_fec();
#endif

#ifdef CONFIG_MXC_EPDC
	setup_epdc();
#endif
	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_I2C_MXC
	int ret = 0;
	setup_i2c(CONFIG_SYS_I2C_PORT);
	i2c_bus_recovery();
	ret = setup_pmic_voltages();
	if (ret)
		return -1;
#endif

	pb650_init();

	return 0;
}

#if defined(BOARD_HOLD_POWER)
int checkDeviceBootReason(void)
{
        int iBootReason = -1;
	switch (__REG(SRC_BASE_ADDR + 0x8)) {
	case 0x0001:
		//printf("POR");
		iBootReason =0;
		break;
	case 0x0009:
		//printf("RST");
		iBootReason=2;
		break;
	case 0x0010:
	case 0x0011:
		iBootReason =2;
		//printf("WDOG");
		break;
	default:
		iBootReason = -1;
		break;
	}
   return iBootReason;
}
#endif

int checkboard(void)
{
	printf("Board: MX6SoloLite-EVK (0x%x): [ ", fsl_system_rev);

	switch (__REG(SRC_BASE_ADDR + 0x8)) {
	case 0x0001:
		printf("POR");
		break;
	case 0x0009:
		printf("RST");
		break;
	case 0x0010:
	case 0x0011:
		printf("WDOG");
		break;
	default:
		printf("unknown");
	}
	printf(" ]\n");

	printf("Boot Device: ");
	switch (get_boot_device()) {
	case WEIM_NOR_BOOT:
		printf("NOR\n");
		break;
	case ONE_NAND_BOOT:
		printf("ONE NAND\n");
		break;
	case I2C_BOOT:
		printf("I2C\n");
		break;
	case SPI_NOR_BOOT:
		printf("SPI NOR\n");
		break;
	case SD_BOOT:
		printf("SD\n");
		break;
	case MMC_BOOT:
		printf("MMC\n");
		break;
	case UNKNOWN_BOOT:
	default:
		printf("UNKNOWN\n");
		break;
	}

#ifdef CONFIG_SECURE_BOOT
	get_hab_status();
#endif

	return 0;
}

#ifdef CONFIG_ANDROID_RECOVERY
int check_recovery_cmd_file(void)
{
	return check_and_clean_recovery_flag();
}
#endif

#ifdef CONFIG_IMX_UDC
void udc_pins_setting(void)
{
	/* USB_OTG_PWR */
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_COL4__GPIO_4_0);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_COL5__GPIO_4_2);
	/* USB_OTG_PWR = 0 */
	gpio_direction_output(USB_OTG_PWR, 0);
	/* USB_H1_POWER = 1 */
	gpio_direction_output(USB_H1_PWR, 1);
}
#endif

#ifdef FRONTLIGHT
//configure pwm base
#define FL_PWM_BASE_ADDR ( PWM1_BASE_ADDR )

//configure frontlight switch gpio
#define FL_SWITCH_GPIO ( IMX_GPIO_NR(1, 22) )

#define PWMCR					0x0   /*Pwm Control Register*/
#define PWMSAR					0X0C  /*Pwm Sample Register*/
#define PWMPR					0X10  /*Pwm Period Register*/

#define PWMPR_PERIOD_CYCLES		32
#define PWMCR_EN				(1 << 0)
#define PWMCR_OFF				(0 << 0)
#define PWMCR_PRESCALER			(0xFFF << 4)
#define PWMCR_STOPEN			(1 << 25)
#define PWMCR_DOZEEN			(1 << 24)
#define PWMCR_WAITEN			(1 << 23)
#define PWMCR_DBGEN			    (1 << 22)
#define PWMCR_CLK_IPG_HIGH 		(2 << 16)
#define PWMCR_CLK_IPG			(3 << 16)

void frontlight_on(void)
{
	unsigned int reg;
	/*PWM Enable*/
	reg = readl(FL_PWM_BASE_ADDR + PWMCR);
	reg |= PWMCR_EN;
	writel(reg,FL_PWM_BASE_ADDR + PWMCR);

	udelay(50 * 1000);

	//switch on
	gpio_direction_output(FL_SWITCH_GPIO, 1);

}

void frontlight_off(void)
{
	unsigned int reg;

	//switch off
	gpio_direction_output(FL_SWITCH_GPIO, 0);

	/*PWM Disable*/
	reg = readl(FL_PWM_BASE_ADDR + PWMCR);
	reg &= ~PWMCR_EN;
	reg |= PWMCR_OFF;
	writel(reg,FL_PWM_BASE_ADDR + PWMCR);

}

void frontlight_config(void)
{
	unsigned int reg_cr;

	//setup frontlight switch gpio and pwm pad
	mxc_iomux_v3_setup_pad(MX6SL_PAD_EPDC_D15__GPIO_1_22);
	gpio_direction_output(FL_SWITCH_GPIO, 0);

	mxc_iomux_v3_setup_pad(MX6SL_PAD_PWM1__PWM1_PWMO);

	/*Config PWM Control Register*/
	reg_cr = readl(FL_PWM_BASE_ADDR + PWMCR);
	reg_cr |= PWMCR_STOPEN | PWMCR_DOZEEN | PWMCR_WAITEN | PWMCR_DBGEN | PWMCR_CLK_IPG;
	reg_cr &= ~PWMCR_PRESCALER;
	writel(reg_cr, FL_PWM_BASE_ADDR + PWMCR);
}

int brig_switch;
int brig_val;
int  brig_switch_val(void)
{

	int mmc_dev = 0; //Nand store
	ulong offset = 30722 * 512;
	ulong size = 512;
	ulong *addr;
	uint blk_start, blk_cnt, n;
	int h_val,l_val,m_val;
	char *fl_buffer = NULL;
	char *switch_buffer = NULL;
	int brightness;

	struct mmc *mmc = find_mmc_device(mmc_dev);

	if (!mmc) {
		printf("MMC Device %d not found\n",
			mmc_dev);
		return -1;
	}

	if (mmc_init(mmc)) {
		puts("MMC init failed\n");
		return -1;
	}

		blk_start = ALIGN(offset, mmc->read_bl_len) / mmc->read_bl_len;
		blk_cnt   = ALIGN(size, mmc->read_bl_len) / mmc->read_bl_len;

		addr = (ulong *)malloc(size);

		n = mmc->block_dev.block_read(mmc_dev, blk_start,
						blk_cnt, (u_char *)addr);
		flush_cache((ulong)addr, blk_cnt * mmc->read_bl_len);
		//printf("addr 's value is %x n is %d\n", addr, n);

		brig_val =  *((int *)addr);
		//printf("==================\n");
		brig_switch = *(((int *)(addr))+1);

		//printf("%x, %x\n", brig_val, brig_switch);
		//no value in the FL address
		if(brig_val == 0 || brig_switch == 0)/*we store it with ASCII format*/
			goto off_fl;
		//wrong value in the FL address
//		if((brig_val & 0x00ff) > 63 || ((brig_val & 0xff000000) >> 24) != 0)
//			goto off_fl;

		if(((brig_val & 0xff000000)>> 24) == 0x0a){
				l_val = (((brig_val & 0xff0000)>>16) - 0x30);
				m_val = ((brig_val & 0xff00)>>8) - 0x30;
				h_val = ((brig_val & 0x00ff) - 0x30);

				brig_val = l_val + m_val*10 + h_val*100;
//			printf("l_val %d, h_val%d\n", l_val, h_val);
		}
		else if(((brig_val & 0xff0000) >> 16) == 0x0a){
				l_val = ((brig_val & 0xff00)>>8) - 0x30;
				h_val = ((brig_val & 0x00ff) - 0x30);
				brig_val = l_val + h_val*10;
			}
			else{
				brig_val = (brig_val & 0x00ff) - 0x30;
			}

		brig_switch = (brig_switch & 0x00ff) - 0x30;
		brightness = brig_val;

		if(brig_val > 255 || brig_val < 0)
			goto off_fl;

		printf("frontlight brightness is 0x%x,brightness switch is 0x%x\n", brig_val, brig_switch);
		goto out;

off_fl:
		brig_switch = 0;
		brightness = 0;
		printf("Due to wrong value frontlight light will keep off\n");
out:
		fl_buffer = (char *)malloc(2);
		sprintf(fl_buffer, "%d", brightness);
		printf("fl_buffer: %s\n",fl_buffer);

		switch_buffer = (char *)malloc(1);
		sprintf(switch_buffer, "%d", brig_switch);
		printf("switch_buffer: %s\n",switch_buffer);

		setenv("fl_val",fl_buffer);
		setenv("fl_switch",switch_buffer);

		free(fl_buffer);
		free(switch_buffer);
		free(addr);
		return 0;

}


int frontlight_brightness_config(int duty_cycles)
{
	unsigned int reg_sar, reg_pr;
	//printf("FL duty cycles %d->", duty_cycles);
	duty_cycles = PWMPR_PERIOD_CYCLES - duty_cycles * PWMPR_PERIOD_CYCLES / 256; // 0-255
	//printf("%d\n", duty_cycles);

	/*Config PWM Sample Register*/
	reg_sar = readl(FL_PWM_BASE_ADDR + PWMSAR);
	reg_sar &= 0;
	reg_sar |= duty_cycles;
	//printf("brightness duty cycles is %d\n", duty_cycles);
	if (duty_cycles > PWMPR_PERIOD_CYCLES)
	{
		printf("Wrong value,it must be [0	%d]\n",PWMPR_PERIOD_CYCLES);
		return -1;
	}
	writel(reg_sar, FL_PWM_BASE_ADDR + PWMSAR);

	/*Setup PWM Period Register*/
	writel(PWMPR_PERIOD_CYCLES, FL_PWM_BASE_ADDR + PWMPR);
	return 0;
}
#endif

#if defined(CONFIG_I2C_MXC)

#define CW2015_I2C_ADDR 0x62
#define BATTERY_LOW_VOLTAGE 3600000
#define BATTERY_CRITICAL_VOLTAGE 3200000

#define CW2015_REG_VCELL_MSB		0x2
#define CW2015_REG_VCELL_LSB		0x3
#define CW2015_REG_MODE			0xA

/*
 * Return voltage in microvolts (uV)
 * if something went wrong, return 0
 */
int get_battery_voltage(void)
{
	unsigned char vcell_h, vcell_l, mode;

	mdebug("Enter");

        setup_i2c(CONFIG_SYS_I2C_PORT);
	i2c_init(CONFIG_SYS_I2C_SPEED, CW2015_I2C_ADDR);
	if (i2c_probe(CW2015_I2C_ADDR) != 0)
		return 0;

	/* CW2015 should be waken up before normal operation */
	if (i2c_read(CW2015_I2C_ADDR, CW2015_REG_MODE, 1, &mode, 1)) {
		printf("Read device ID error!\n");
		return 0;
	}
	/* When device is in sleep mode, 7th and 6th bits are set */
	if ((mode & 0xc0) == 0xc0) {
		printf("Detected sleep mode for cw2015\n");
		mode &= ~(0xc0);
		i2c_write(CW2015_I2C_ADDR, CW2015_REG_MODE, 1, mode, 1);
	}

	if (i2c_read(CW2015_I2C_ADDR, CW2015_REG_VCELL_MSB, 1, &vcell_h, 1)) {
		printf("Read device ID error!\n");
		return 0;
	}

	if (i2c_read(CW2015_I2C_ADDR, CW2015_REG_VCELL_LSB, 1, &vcell_l, 1)) {
		printf("Read Rev ID error!\n");
		return 0;
	}

	mdebug("VCELL %02x %02x", vcell_h, vcell_l);

	/* Convert parrots into volts */
	return (vcell_h << 8 | vcell_l) * 305;
}
#endif

static inline void mdelay(unsigned long msec)
{
	unsigned long i;

	for (i = 0; i < msec; i++)
		udelay(1000);
}

int has_voltage_to_boot(void)
{
	mdebug("Enter");

#if defined(CONFIG_I2C_MXC)
	int voltage, repeat;

	repeat = 10;
	voltage = 0;
	while (repeat--) {
		voltage = get_battery_voltage();
		if (voltage > 0)
			break;
		mdebug("Iteration %02d: voltage read failed", repeat);
		mdelay(25);
	}

	printf("Current battery voltage: %d uV\n", voltage);
#endif

	/* If read battery voltage failed due to i2c errors, consider voltage
	 * level sufficient for device boot */
	return voltage ? voltage > BATTERY_LOW_VOLTAGE : 1;
}

extern void device_power_off(void);

/* Return 1 if valid voltage is present on usb */
int usb_charger_detect(void)
{
	return (readl(ANATOP_BASE_ADDR + HW_ANADIG_USB1_VBUS_DET_STAT) \
			& BM_ANADIG_USB1_VBUS_DET_STAT_VBUS_VALID) ? 1 : 0;
}

static void charge_current_set(int value)
{
	mdebug("Set charge_current_set pin value %d\n", value);

	mxc_iomux_v3_setup_pad(MX6SL_PAD_FEC_TXD1__GPIO_4_16);
	gpio_direction_output(CHARGE_CURRENT_SET, !!value);
}

static iomux_v3_cfg_t deep_suspend_enter_pads[] = {
        /* Audio pads. */
// do not change audio pins state while is in suspend
        MX6SL_PAD_AUD_TXC__GPIO_1_3,
        MX6SL_PAD_AUD_TXD__GPIO_1_5,
        MX6SL_PAD_AUD_TXFS__GPIO_1_4,
        MX6SL_PAD_AUD_RXD__GPIO_1_2,
        MX6SL_PAD_AUD_MCLK__GPIO_1_6,
#if  defined(CONFIG_SND_SOC_IMX_AIC325X)
        MX6SL_PAD_EPDC_D13__GPIO_1_20,/*HEADPHONE_DET*/
        MX6SL_PAD_EPDC_D14__GPIO_1_21,/*reset*/
#endif
        /*I2C3*/
        MX6SL_PAD_AUD_RXC__GPIO_1_1,
        MX6SL_PAD_AUD_RXFS__GPIO_1_0,

        /*Software version control*/
        MX6SL_PAD_ECSPI1_MISO__GPIO_4_10,
        MX6SL_PAD_ECSPI1_SS0__GPIO_4_11,
        /* ECSPI pads. */
        MX6SL_PAD_ECSPI1_MOSI__GPIO_4_9,
        MX6SL_PAD_ECSPI1_SCLK__GPIO_4_8,
        /* FEC pad*/
        //MX6SL_PAD_FEC_CRS_DV__GPIO_4_25,
        MX6SL_PAD_FEC_MDC__GPIO_4_23,
//      MX6SL_PAD_FEC_MDIO__GPIO_4_20,
        MX6SL_PAD_FEC_REF_CLK__GPIO_4_26,
        MX6SL_PAD_FEC_RXD0__GPIO_4_17,
//      MX6SL_PAD_FEC_RXD1__GPIO_4_18,
//      MX6SL_PAD_FEC_TXD0__GPIO_4_24,
        MX6SL_PAD_FEC_TXD1__GPIO_4_16,
//      MX6SL_PAD_FEC_TX_CLK__GPIO_4_21,
//      MX6SL_PAD_FEC_TX_EN__GPIO_4_22,
        /* I2C pads */
        MX6SL_PAD_I2C1_SCL__GPIO_3_12,
        MX6SL_PAD_I2C1_SDA__GPIO_3_13,
        MX6SL_PAD_I2C2_SCL__GPIO_3_14,
        MX6SL_PAD_I2C2_SDA__GPIO_3_15,
        /* LCD pads*/
        MX6SL_PAD_LCD_CLK__GPIO_2_15,
        MX6SL_PAD_LCD_DAT0__GPIO_2_20,
        MX6SL_PAD_LCD_DAT1__GPIO_2_21,
        MX6SL_PAD_LCD_DAT2__GPIO_2_22,
        MX6SL_PAD_LCD_DAT3__GPIO_2_23,
        MX6SL_PAD_LCD_DAT4__GPIO_2_24,
        MX6SL_PAD_LCD_DAT5__GPIO_2_25,
        MX6SL_PAD_LCD_DAT6__GPIO_2_26,
        MX6SL_PAD_LCD_DAT7__GPIO_2_27,
        MX6SL_PAD_LCD_DAT8__GPIO_2_28,
        MX6SL_PAD_LCD_DAT9__GPIO_2_29,
        MX6SL_PAD_LCD_DAT10__GPIO_2_30,
        MX6SL_PAD_LCD_DAT11__GPIO_2_31,
        MX6SL_PAD_LCD_DAT12__GPIO_3_0,
        MX6SL_PAD_LCD_DAT13__GPIO_3_1,
        MX6SL_PAD_LCD_DAT14__GPIO_3_2,
        MX6SL_PAD_LCD_DAT15__GPIO_3_3,
        MX6SL_PAD_LCD_DAT16__GPIO_3_4,
        MX6SL_PAD_LCD_DAT17__GPIO_3_5,
        MX6SL_PAD_LCD_DAT18__GPIO_3_6,
        MX6SL_PAD_LCD_DAT19__GPIO_3_7,
        MX6SL_PAD_LCD_DAT20__GPIO_3_8,
        MX6SL_PAD_LCD_DAT21__GPIO_3_9,
        MX6SL_PAD_LCD_DAT22__GPIO_3_10,
        MX6SL_PAD_LCD_DAT23__GPIO_3_11,

        /* PWM pads */
        //do not suspend frontlight
        //MX6SL_PAD_PWM1__GPIO_3_23,

        /* SD pads. */
        MX6SL_PAD_SD1_CLK__GPIO_5_15,
        MX6SL_PAD_SD1_CMD__GPIO_5_14,
        MX6SL_PAD_SD1_DAT0__GPIO_5_11,
        MX6SL_PAD_SD1_DAT1__GPIO_5_8,
        MX6SL_PAD_SD1_DAT2__GPIO_5_13,
        MX6SL_PAD_SD1_DAT3__GPIO_5_6,
        MX6SL_PAD_SD1_DAT4__GPIO_5_12,
        MX6SL_PAD_SD1_DAT5__GPIO_5_9,
        MX6SL_PAD_SD1_DAT6__GPIO_5_7,
        MX6SL_PAD_SD1_DAT7__GPIO_5_10,

        /*Camera */
        MX6SL_PAD_SD2_CLK__GPIO_5_5,
        MX6SL_PAD_SD2_CMD__GPIO_5_4,
        MX6SL_PAD_SD2_DAT0__GPIO_5_1,
        MX6SL_PAD_SD2_DAT1__GPIO_4_30,
        MX6SL_PAD_SD2_DAT2__GPIO_5_3,
        MX6SL_PAD_SD2_DAT3__GPIO_4_28,
        MX6SL_PAD_SD2_DAT4__GPIO_5_2,
        MX6SL_PAD_SD2_DAT5__GPIO_4_31,
        MX6SL_PAD_SD2_DAT6__GPIO_4_29,
        MX6SL_PAD_SD2_DAT7__GPIO_5_0,

        MX6SL_PAD_ECSPI2_SCLK__GPIO_4_12,
        MX6SL_PAD_ECSPI2_MISO__GPIO_4_14,
        MX6SL_PAD_ECSPI2_MOSI__GPIO_4_13,
        MX6SL_PAD_ECSPI2_SS0__GPIO_4_15,

        MX6SL_PAD_LCD_RESET__GPIO_2_19,/* CMOS_RESET_B GPIO */
        //MX6SL_PAD_EPDC_SDOE__GPIO_1_25,/* CMOS_PWDN GPIO */
        MX6SL_PAD_LCD_ENABLE__GPIO_2_16,/* CMOS_PWDN GPIO */
        MX6SL_PAD_EPDC_D8__GPIO_1_15,/*CAM_2.8V_EN*/
        MX6SL_PAD_EPDC_D9__GPIO_1_16,/*CAM_1.5V_EN*/
        MX6SL_PAD_EPDC_D10__GPIO_1_17,/*CAM_1.8V_EN*/
        MX6SL_PAD_EPDC_D11__GPIO_1_18,/*FLASH_LED_CTL*/
        MX6SL_PAD_EPDC_D12__GPIO_1_19,/*TORCH_CTL*/

        /*end*/
//do not suspend ext SD pins
        MX6SL_PAD_SD3_CLK__GPIO_5_18,
        MX6SL_PAD_SD3_CMD__GPIO_5_21,
        MX6SL_PAD_SD3_DAT0__GPIO_5_19,
        MX6SL_PAD_SD3_DAT1__GPIO_5_20,
        MX6SL_PAD_SD3_DAT2__GPIO_5_16,
        MX6SL_PAD_SD3_DAT3__GPIO_5_17,

        //MX6SL_PAD_LCD_DAT11__GPIO_2_31,/*cd */
        /*32k clk*/
        MX6SL_PAD_REF_CLK_32K__GPIO_3_22,
        /*WIFI_SD4_CLK*/
        MX6SL_PAD_FEC_MDIO__GPIO_4_20,
        /*WIFI_SD4_CMD*/
        MX6SL_PAD_FEC_TX_CLK__GPIO_4_21,
        /*WIFI_SD4_DAT0*/
        MX6SL_PAD_FEC_RX_ER__GPIO_4_19,
        /*WIFI_SD4_DAT1*/
        MX6SL_PAD_FEC_CRS_DV__GPIO_4_25,
        /*WIFI_SD4_DAT2*/
        MX6SL_PAD_FEC_RXD1__GPIO_4_18,
        /*WIFI_SD4_DAT3*/
        MX6SL_PAD_FEC_TXD0__GPIO_4_24,


        /* USBOTG ID pin */
        MX6SL_PAD_HSIC_STROBE__GPIO_3_20,
        MX6SL_PAD_HSIC_DAT__GPIO_3_19,

        /* Key row/column */
//do not change state ext SD pins
        MX6SL_PAD_KEY_COL0__GPIO_3_24, //SD_CD
        MX6SL_PAD_KEY_COL1__GPIO_3_26, //SD_EN

        MX6SL_PAD_KEY_COL2__GPIO_3_28,

//do not change state eMMC0 reset pin
        MX6SL_PAD_KEY_COL3__GPIO_3_30,

        MX6SL_PAD_KEY_COL6__GPIO_4_4,
        //MX6SL_PAD_KEY_COL7__GPIO_4_6,
        MX6SL_PAD_KEY_ROW0__GPIO_3_25,
        MX6SL_PAD_KEY_ROW1__GPIO_3_27,
        MX6SL_PAD_KEY_ROW2__GPIO_3_29,
        MX6SL_PAD_KEY_ROW3__GPIO_3_31,
        MX6SL_PAD_KEY_ROW4__GPIO_4_1,
        MX6SL_PAD_KEY_ROW5__GPIO_4_3,
        //MX6SL_PAD_KEY_ROW6__GPIO_4_5,

};

static struct {
	unsigned long addr;
	unsigned long data;
} lp_config[] = {
	{ 0x020c4074, 0x1ff00000 },
	{ 0x020c4078, 0x0000f300 },
	{ 0x020c407c, 0x0f000001 },
	{ 0x020c4080, 0x00000003 },

	{ 0x020c8150, 0x04010880 },

	{ 0x020c4060, 0x000a000c },
	{ 0x020c4064, 0x0000fe62 },
	{ 0x020c4068, 0x0040000f },
	{ 0x020c406c, 0x00f00000 },
	{ 0x020c4070, 0x017f3000 },

	{ 0x020c4000, 0x040710ff },
	{ 0x020c400c, 0x0 },
	{ 0x020c4054, 0x0009077e },

	{ 0x020dc240, 0x1 },
	{ 0x020dc260, 0x1 },
	{ 0x020dc2a0, 0x1 },
};

extern void mx6_suspend(void);

void do_suspend(void)
{
	int charging, voltage, i;
	unsigned long reg;

        mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_COL6__GPIO_4_4);
        gpio_direction_output(GPIO_WIFI_EN, 0);
	mxc_iomux_v3_setup_pad(MX6SL_PAD_KEY_ROW6__GPIO_4_5);
        gpio_direction_output(GPIO_TOUCH_EN, 0);

	pmic_standby();

	// configure GPC
	reg = readl(0x020dc000);
	reg = reg | (1 << 21) | (1 << 22);
	writel(reg, 0x020dc000);
	// mask interrupts
	writel(0xffffffff, 0x020dc008);
	writel(0xffffffff, 0x020dc00c);
	writel(0xffffffff, 0x020dc010);
	writel(0xffffffff, 0x020dc014);
	//reg = readw(WDOG1_BASE_ADDR);
	//reg = (reg & 0xff) | (30 << 8) | (1 << 3);
	//writew(reg, WDOG1_BASE_ADDR);
	//reg |= (1 << 2);
	//writew(reg, WDOG1_BASE_ADDR);
	printf("wfi.");
	mdelay(200);

	for (i=0; i<ARRAY_SIZE(lp_config); i++) {
		writel(lp_config[i].data, lp_config[i].addr);
	}

	for (i=0; i<ARRAY_SIZE(deep_suspend_enter_pads); i++) {
		mxc_iomux_v3_setup_pad(deep_suspend_enter_pads[i]);
	}
	mx6_suspend();
	printf("!");
	mdelay(200);
	gpio_toggle_value(SYSTEM_LED);
}

void lowbatt_handler(void)
{
	int charging, voltage, i;
	unsigned long reg;

	mdebug("Enter");

	/* We are here because low battery voltage detected */
	/* According to current (14.04.14) schematics, charging current will be
	 * 500 mA if pin is low, and 1000 mA if pin high, we don't know usb
	 * power source (ac or pc) so choose 500 mA */
	charge_current_set(0);

	charging = 0;
	voltage = 0;
	do {
		voltage = get_battery_voltage();
		if (voltage > BATTERY_LOW_VOLTAGE) {
			mdebug("Enough voltage to boot device");
			break;
		}

		charging = usb_charger_detect();
		if (charging) {
			if (voltage > BATTERY_CRITICAL_VOLTAGE) {
				printf("device is charging ... %d uV\n", voltage);
				mdelay(1000);
			} else {
				printf("critical voltage level (%d uV), going suspend (no watchdog)...\n", voltage);
				do_suspend();
			}
			continue;
		}

		printf("Voltage level below threshold and no charger detected (%d uV)", voltage);

		device_power_off();

	} while(1);
}

