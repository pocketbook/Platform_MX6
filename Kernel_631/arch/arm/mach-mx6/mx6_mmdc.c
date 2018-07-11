/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mx6_mmdc.c
 *
 * @brief MX6 MMDC specific file.
 *
 * @ingroup PM
 */
#include <asm/io.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/iram_alloc.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/cpumask.h>
#include <linux/kernel.h>
#include <linux/smp.h>
#include <mach/hardware.h>
#include <mach/clock.h>
#include <asm/mach/map.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/tlb.h>
#include <asm/hardware/gic.h>
#include "crm_regs.h"


/* DDR settings */
unsigned long (*iram_ddr_settings)[2];
unsigned long (*normal_mmdc_settings)[2];
unsigned long (*iram_iomux_settings)[2];
void __iomem *mmdc_base;
void __iomem *iomux_base;
void __iomem *gic_dist_base;
void __iomem *gic_cpu_base;

void (*mx6_change_ddr_freq)(u32 freq, void *ddr_settings, bool dll_mode, void* iomux_offsets) = NULL;

void (*mx6sl_ddr3_change_freq)(u32 freq, void *ddr_settings, bool dll_mode, void* iomux_offsets, int low_bus_freq_mode) = NULL;

extern unsigned int ddr_low_rate;
extern unsigned int ddr_med_rate;
extern unsigned int ddr_normal_rate;
extern int low_bus_freq_mode;
extern int audio_bus_freq_mode;
extern int mmdc_med_rate;
extern void __iomem *ccm_base;
extern void mx6_ddr_freq_change(u32 freq, void *ddr_settings, bool dll_mode, void *iomux_offsets);
extern void mx6sl_ddr3_freq_change(u32 freq, void *ddr_settings, bool dll_mode, void *iomux_offsets, int low_bus_freq_mode);
extern unsigned long save_ttbr1(void);
extern void restore_ttbr1(u32 ttbr1);

extern unsigned long mx6_ddr3_iram_end asm("mx6_ddr3_iram_end");
extern unsigned long mx6_ddr3_iram_start asm("mx6_ddr3_iram_start");
extern unsigned long mx6sl_lpddr2_iram_end asm("mx6sl_lpddr2_iram_end");
extern unsigned long mx6sl_lpddr2_iram_start asm("mx6sl_lpddr2_iram_start");
extern unsigned long mx6sl_ddr3_iram_end asm("mx6sl_ddr3_iram_end");
extern unsigned long mx6sl_ddr3_iram_start asm("mx6sl_ddr3_iram_start");

unsigned long ddr_freq_change_iram_phys_addr;
static void *ddr_freq_change_iram_base;
static int ddr_settings_size;
static int iomux_settings_size;
static volatile unsigned int cpus_in_wfe;
static volatile bool wait_for_ddr_freq_update;
static int curr_ddr_rate;

#define MIN_DLL_ON_FREQ		333000000
#define MAX_DLL_OFF_FREQ		125000000

unsigned long ddr3_dll_mx6q[][2] = {
	{0x0c, 0x0},
	{0x10, 0x0},
	{0x1C, 0x04088032},
	{0x1C, 0x0408803a},
	{0x1C, 0x08408030},
	{0x1C, 0x08408038},
	{0x818, 0x0},
};

unsigned long ddr3_calibration[][2] = {
	{0x83c, 0x0},
	{0x840, 0x0},
	{0x483c, 0x0},
	{0x4840, 0x0},
	{0x848, 0x0},
	{0x4848, 0x0},
	{0x850, 0x0},
	{0x4850, 0x0},
};

unsigned long ddr3_dll_mx6dl[][2] = {
	{0x0c, 0x0},
	{0x10, 0x0},
	{0x1C, 0x04008032},
	{0x1C, 0x0400803a},
	{0x1C, 0x07208030},
	{0x1C, 0x07208038},
	{0x818, 0x0},
};

unsigned long ddr3_dll_mx6sl[][2] = {
	{0x0c, 0x0},
	{0x10, 0x0},
	{0x18, 0x0},
	{0x30, 0x0},
	{0x1C, 0x04408032},
	{0x1C, 0x00048031},
	{0x1C, 0x05208030},
	{0x1C, 0x04008040},
	{0x818, 0x0},
};

unsigned long ddr3_calibration_mx6sl[][2] = {
	{0x83c, 0x0},
	{0x840, 0x0},
	{0x848, 0x0},
	{0x850, 0x0},
};

unsigned long iomux_offsets_mx6q[][2] = {
	{0x5A8, 0x0},
	{0x5B0, 0x0},
	{0x524, 0x0},
	{0x51C, 0x0},
	{0x518, 0x0},
	{0x50C, 0x0},
	{0x5B8, 0x0},
	{0x5C0, 0x0},
};

unsigned long iomux_offsets_mx6dl[][2] = {
	{0x4BC, 0x0},
	{0x4C0, 0x0},
	{0x4C4, 0x0},
	{0x4C8, 0x0},
	{0x4CC, 0x0},
	{0x4D0, 0x0},
	{0x4D4, 0x0},
	{0x4D8, 0x0},
};

unsigned long iomux_offsets_mx6sl[][2] = {
	{0x344, 0x0},
	{0x348, 0x0},
	{0x34c, 0x0},
	{0x350, 0x0},
};

unsigned long ddr3_400[][2] = {
	{0x83c, 0x42490249},
	{0x840, 0x02470247},
	{0x483c, 0x42570257},
	{0x4840, 0x02400240},
	{0x848, 0x4039363C},
	{0x4848, 0x3A39333F},
	{0x850, 0x38414441},
	{0x4850, 0x472D4833}
};

unsigned long *irq_used;

unsigned long irqs_used_mx6q[] = {
	MXC_INT_INTERRUPT_139_NUM,
	MX6Q_INT_PERFMON1,
	MX6Q_INT_PERFMON2,
	MX6Q_INT_PERFMON3,
};

unsigned long irqs_used_mx6dl[] = {
	MXC_INT_INTERRUPT_139_NUM,
	MX6Q_INT_PERFMON1,
};

unsigned long irqs_used_mx6sl[] = {
	MXC_INT_INTERRUPT_139_NUM,
};

int can_change_ddr_freq(void)
{
	return 1;
}


/* Each active core apart from the one changing the DDR frequency will execute
  * this function. The rest of the cores have to remain in WFE state until the frequency
  * is changed.
  */
irqreturn_t wait_in_wfe_irq(int irq, void *dev_id)
{
	u32 me = smp_processor_id();

	*((char *)(&cpus_in_wfe) + (u8)me) = 0xff;

	while (wait_for_ddr_freq_update)
		wfe();

	*((char *)(&cpus_in_wfe) + (u8)me) = 0;

	return IRQ_HANDLED;
}

extern unsigned long iram_tlb_phys_addr;
/* Change the DDR frequency. */
int update_ddr_freq(int ddr_rate)
{
	int i, j;
	unsigned int reg;
	bool dll_off = false;
	unsigned int online_cpus = 0;
	int cpu = 0;
	int me;
	u32 ttbr1;

	if (!can_change_ddr_freq())
		return -1;

	if (ddr_rate == curr_ddr_rate)
		return 0;

//	printk(KERN_DEBUG"%s(%d) %s(%d, %d)\n",__FILE__,__LINE__,__FUNCTION__,ddr_rate,low_bus_freq_mode);

	if (low_bus_freq_mode || audio_bus_freq_mode)
		dll_off = true;

	if (cpu_is_mx6sl()) {
		iram_ddr_settings[0][0] = ARRAY_SIZE(ddr3_dll_mx6sl);
		iram_ddr_settings[0][1] = ARRAY_SIZE(ddr3_calibration_mx6sl);
	} else {
		iram_ddr_settings[0][0] = ddr_settings_size;
	}
	iram_iomux_settings[0][0] = iomux_settings_size;
	if (ddr_rate == ddr_med_rate && cpu_is_mx6q() && ddr_med_rate != ddr_normal_rate) {
		for (i = 0; i < ARRAY_SIZE(ddr3_dll_mx6q); i++) {
			iram_ddr_settings[i + 1][0] =
					normal_mmdc_settings[i][0];
			iram_ddr_settings[i + 1][1] =
					normal_mmdc_settings[i][1];
		}
		for (j = 0, i = ARRAY_SIZE(ddr3_dll_mx6q); i < ddr_settings_size; j++, i++) {
			iram_ddr_settings[i + 1][0] =
					ddr3_400[j][0];
			iram_ddr_settings[i + 1][1] =
					ddr3_400[j][1];
		}
	} else if (ddr_rate == ddr_normal_rate) {
		for (i = 0; i < ddr_settings_size; i++) {
			iram_ddr_settings[i + 1][0] =
					normal_mmdc_settings[i][0];
			iram_ddr_settings[i + 1][1] =
					normal_mmdc_settings[i][1];
		}
	}

	/* Ensure that all Cores are in WFE. */
	local_irq_disable();

	me = smp_processor_id();

	*((char *)(&cpus_in_wfe) + (u8)me) = 0xff;
	wait_for_ddr_freq_update = true;
	for_each_online_cpu(cpu) {
		*((char *)(&online_cpus) + (u8)cpu) = 0xff;
		if (!cpu_is_mx6sl()) {
			if (cpu != me) {
				/* Set the interrupt to be pending in the GIC. */
				reg = 1 << (irq_used[cpu] % 32);
				writel_relaxed(reg, gic_dist_base + GIC_DIST_PENDING_SET + (irq_used[cpu] / 32) * 4);
			}
		}
	}
	while (cpus_in_wfe != online_cpus)
		udelay(5);

	/* Save TTBR1 */
	ttbr1 = save_ttbr1();
	/* Now we can change the DDR frequency. */
	if (cpu_is_mx6sl())
		mx6sl_ddr3_change_freq(ddr_rate, iram_ddr_settings, dll_off, iram_iomux_settings, low_bus_freq_mode);
	else
		mx6_change_ddr_freq(ddr_rate, iram_ddr_settings, dll_off, iram_iomux_settings);
	restore_ttbr1(ttbr1);

	curr_ddr_rate = ddr_rate;

	/* DDR frequency change is done . */
	wait_for_ddr_freq_update = false;

	/* Wake up all the cores. */
	sev();

	*((char *)(&cpus_in_wfe) + (u8)me) = 0;

	local_irq_enable();

	return 0;
}

int init_mmdc_settings(void)
{
	int i, err, cpu;
	unsigned long ddr_code_size = 0;

	mmdc_base = ioremap(MMDC_P0_BASE_ADDR, SZ_32K);
	iomux_base = ioremap(MX6Q_IOMUXC_BASE_ADDR, SZ_16K);
	gic_dist_base = ioremap(IC_DISTRIBUTOR_BASE_ADDR, SZ_16K);
	gic_cpu_base = ioremap(IC_INTERFACES_BASE_ADDR, SZ_16K);

	if (cpu_is_mx6q())
		ddr_settings_size = ARRAY_SIZE(ddr3_dll_mx6q) + ARRAY_SIZE(ddr3_calibration);
	if (cpu_is_mx6dl())
		ddr_settings_size = ARRAY_SIZE(ddr3_dll_mx6dl) + ARRAY_SIZE(ddr3_calibration);
	if (cpu_is_mx6sl()) {
		ddr_settings_size = ARRAY_SIZE(ddr3_dll_mx6sl) + ARRAY_SIZE(ddr3_calibration_mx6sl);
		iomux_settings_size = ARRAY_SIZE(iomux_offsets_mx6sl);
		ddr_code_size = (&mx6sl_ddr3_iram_end -&mx6sl_ddr3_iram_start) *4;
	}

	/* Use preallocated IRAM memory */
	ddr_freq_change_iram_phys_addr = MX6_IRAM_DDR_FREQ_ADDR;
	/*
	 * Don't ioremap the address, we have fixed the IRAM address
	 * at IRAM_BASE_ADDR_VIRT
	 */
	ddr_freq_change_iram_base = (void *)IRAM_BASE_ADDR_VIRT +
		(ddr_freq_change_iram_phys_addr - IRAM_BASE_ADDR);

	normal_mmdc_settings = kmalloc((ddr_settings_size * 8), GFP_KERNEL);
	if (cpu_is_mx6sl()) {
		memcpy(normal_mmdc_settings, ddr3_dll_mx6sl, sizeof(ddr3_dll_mx6sl));
		memcpy(((char *)normal_mmdc_settings + sizeof(ddr3_dll_mx6sl)), ddr3_calibration_mx6sl, sizeof(ddr3_calibration_mx6sl));
		memcpy(ddr_freq_change_iram_base, mx6sl_ddr3_freq_change, ddr_code_size);
			mx6sl_ddr3_change_freq = (void *)ddr_freq_change_iram_base;
	}
	else {
		memcpy(ddr_freq_change_iram_base, mx6_ddr_freq_change, ddr_code_size);
		mx6_change_ddr_freq = (void *)ddr_freq_change_iram_base;
		if (cpu_is_mx6q()) {
			memcpy(normal_mmdc_settings, ddr3_dll_mx6q, sizeof(ddr3_dll_mx6q));
			memcpy(((char *)normal_mmdc_settings + sizeof(ddr3_dll_mx6q)), ddr3_calibration, sizeof(ddr3_calibration));
		}
		if (cpu_is_mx6dl()) {
			memcpy(normal_mmdc_settings, ddr3_dll_mx6dl, sizeof(ddr3_dll_mx6dl));
			memcpy(((char *)normal_mmdc_settings + sizeof(ddr3_dll_mx6dl)), ddr3_calibration, sizeof(ddr3_calibration));
		}
	}

	/* Store the original DDR settings at boot. */
	for (i = 0; i < ddr_settings_size; i++) {
		/*Writes via command mode register cannot be read back.
		 * Hence hardcode them in the initial static array.
		 * This may require modification on a per customer basis.
		 */
		if (normal_mmdc_settings[i][0] != 0x1C)
			normal_mmdc_settings[i][1] =
				__raw_readl(mmdc_base
				+ normal_mmdc_settings[i][0]);
	}

	/* Store the size of the array in iRAM also,
	 * increase the size by 8 bytes.
	 */
	iram_iomux_settings = ddr_freq_change_iram_base + ddr_code_size;
	iram_ddr_settings = iram_iomux_settings + (iomux_settings_size * 8) + 8;

	if ((ddr_code_size + (iomux_settings_size + ddr_settings_size) * 8 + 16)
		> MX6_IRAM_DDR_FREQ_CODE_SIZE) {
		printk(KERN_ERR "Not enough memory allocated for DDR Frequency change code.\n");
		return -EINVAL;
 	}

	/* Store the IOMUX settings at boot. */
	if (cpu_is_mx6q()) {
		for (i = 0; i < iomux_settings_size; i++) {
			iomux_offsets_mx6q[i][1] =
				__raw_readl(iomux_base
				+ iomux_offsets_mx6q[i][0]);
			iram_iomux_settings[i+1][0] = iomux_offsets_mx6q[i][0];
			iram_iomux_settings[i+1][1] = iomux_offsets_mx6q[i][1];
		}
		irq_used = irqs_used_mx6q;
	}

	if (cpu_is_mx6dl()) {
		for (i = 0; i < iomux_settings_size; i++) {
			iomux_offsets_mx6dl[i][1] =
				__raw_readl(iomux_base
				+ iomux_offsets_mx6dl[i][0]);
			iram_iomux_settings[i+1][0] = iomux_offsets_mx6dl[i][0];
			iram_iomux_settings[i+1][1] = iomux_offsets_mx6dl[i][1];
		}
		irq_used = irqs_used_mx6dl;
	}

	if (cpu_is_mx6sl()) {
		for (i = 0; i < iomux_settings_size; i++) {
			iomux_offsets_mx6sl[i][1] =
				__raw_readl(iomux_base
				+ iomux_offsets_mx6sl[i][0]);
			iram_iomux_settings[i+1][0] = iomux_offsets_mx6sl[i][0];
			iram_iomux_settings[i+1][1] = iomux_offsets_mx6sl[i][1];
		}
		irq_used = irqs_used_mx6sl;
	}

	curr_ddr_rate = ddr_normal_rate;

	if (!cpu_is_mx6sl()) {
		for_each_online_cpu(cpu) {
			/* Set up a reserved interrupt to get all the active cores into a WFE state
			  * before changing the DDR frequency.
			  */
			err = request_irq(irq_used[cpu], wait_in_wfe_irq, IRQF_PERCPU, "mmdc_1",
					  NULL);
			if (err) {
				printk(KERN_ERR "MMDC: Unable to attach to %ld,err = %d\n", irq_used[cpu], err);
				return err;
			}
			err = irq_set_affinity(irq_used[cpu], cpumask_of(cpu));
			if (err) {
				printk(KERN_ERR "MMDC: unable to set irq affinity irq=%ld,\n", irq_used[cpu]);
				return err;
			}
		}
	}
	return 0;
}
