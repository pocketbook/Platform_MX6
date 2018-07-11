/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <asm/sizes.h>
#include <mach/hardware.h>
#include <mach/devices-common.h>

#define imx_spdc_data_entry_single(soc, size)	\
	{								\
		.iobase = soc ## _SIPIX_BASE_ADDR,			\
		.irq = soc ## _INT_SPDC,				\
		.iosize = size,						\
	}

#ifdef CONFIG_SOC_IMX6SL
const struct imx_epdc_data imx6sl_spdc_data __initconst =
			imx_spdc_data_entry_single(MX6SL, SZ_16K);
#endif

struct platform_device *__init imx_add_imx_spdc(
		const struct imx_epdc_data *data,
		const struct imx_spdc_fb_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device_dmamask("imx_spdc_fb", -1,
		res, ARRAY_SIZE(res), pdata, sizeof(*pdata), DMA_BIT_MASK(32));
}

