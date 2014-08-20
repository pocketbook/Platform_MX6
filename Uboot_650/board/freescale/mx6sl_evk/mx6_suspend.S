#include <config.h>
#include <asm/arch/mx6.h>


        .macro  flush_disable_l1_dcache

	mov     r10, #0                         @ swith back to cache level 0
	mcr     p15, 2, r10, c0, c0, 0          @ select current cache level in cssr
	dsb
	isb
	

        /*
         * Clear the SCTLR.C bit to prevent further data cache
         * allocation. Clearing SCTLR.C would make all the data accesses
         * strongly ordered and would not hit the cache.
         */
        mrc p15, 0, r0, c1, c0, 0
        bic r0, r0, #(1 << 2)           @ Disable the C bit
        mcr p15, 0, r0, c1, c0, 0

        isb
        dsb
        dmb

        .endm


        .macro  clean_l2_cache
        /* Clean L2 cache to write the dirty data into DRAM to make
        sure the data alignment between DRAM and L2 cache.
        */

        /* Clean L2 cache here */
        ldr r1, =0x00a02000
        /* Make way to 0xFFFF 16 ways */
        mov r0, #0x10000
        sub r0, r0, #0x1
        /* 0x7BC is L2X0_CLEAN_WAY */
        mov r4, #0x700
        orr r4, #0xBC
        str r0, [r1, r4]
3:
        ldr r5, [r1, r4]
        ands    r5, r5, r0
        bne 3b
4:
        mov r5, #0x0
        /* 0x730 is L2X0_CACHE_SYNC */
        mov r4, #0x700
        orr r4, #0x30
        str r5, [r1, r4]
5:
        ldr r5, [r1, r4]
        ands    r5, r5, #0x1
        bne 5b

        .endm

        .macro  sl_ddr_io_set_lpm

        mov     r0, #0
        str     r0, [r1, #0x30c] /* DRAM_DQM0 */
        str     r0, [r1, #0x310] /* DRAM_DQM1 */
        str     r0, [r1, #0x314] /* DRAM_DQM2 */
        str     r0, [r1, #0x318] /* DRAM_DQM3 */

        str r0, [r1, #0x344] /* DRAM_SDQS0 */
        str r0, [r1, #0x348] /* DRAM_SDQS1 */
        str r0, [r1, #0x34c] /* DRAM_SDQS2 */
        str r0, [r1, #0x350] /* DRAM_SDQS3 */

        str     r0, [r1, #0x5c4] /* GPR_B0DS */
        str     r0, [r1, #0x5cc] /* GPR_B1DS */
        str     r0, [r1, #0x5d4] /* GPR_B2DS */
        str     r0, [r1, #0x5d8] /* GPR_B3DS */

        str     r0, [r1, #0x300] /* DRAM_CAS */
        str     r0, [r1, #0x31c] /* DRAM_RAS */
        str     r0, [r1, #0x338] /* DRAM_SDCLK_0 */
        str     r0, [r1, #0x5ac] /* GPR_ADDS*/

        str     r0, [r1, #0x5b0] /* DDRMODE_CTL */
        str     r0, [r1, #0x5c0] /* DDRMODE */
        str     r0, [r1, #0x33c] /* DRAM_SODT0*/
        str     r0, [r1, #0x340] /* DRAM_SODT1*/

        .endm




        .macro  mx6sl_standy_saving_set

        /* Move periph_clk to OSC_CLK. */
        ldr     r1, =CCM_BASE_ADDR

        /* Ensure the periph_clk2_sel to OSC clk. */
        ldr     r0, [r1, #0x18]
        bic     r0, r0, #0x3000
        orr     r0, r0, #0x1000
        str     r0, [r1, #0x18]

        ldr     r0, [r1, #0x14]
        orr     r0, r0, #0x2000000
        str     r0, [r1, #0x14]

periph_clk_switch:
        ldr     r0, [r1, #0x48]
        cmp     r0, #0
        bne     periph_clk_switch

        /* Set the divider to divider by 8 */
        ldr     r0, [r1, #0x14]
        orr     r0, r0, #0x70000
        orr     r0, r0, #0x1c00
        str     r0, [r1, #0x14]

ahb_podf:
        ldr     r0, [r1, #0x48]
        cmp     r0, #0
        bne     ahb_podf

        /* Move DDR clk to PLL3 clock.
          */
        /* First set the divider to 2. */
        ldr     r0, [r1, #0x14]
        orr     r0, r0, #0x1
        str     r0, [r1, #0x14]

        ldr     r0, [r1, #0x18]
        bic     r0, r0, #0x100000
        str     r0, [r1, #0x18]

        ldr     r0, [r1, #0x14]
        orr     r0, r0, #0x4000000
        str     r0, [r1, #0x14]

ddr_switch:
        ldr     r0, [r1, #0x48]
        cmp     r0, #0
        bne     ddr_switch

        /* Set DDR clock to divide by 8. */
        ldr     r0, [r1, #0x14]
        orr     r0, r0, #0x38
        str     r0, [r1, #0x14]

mmdc_div:
        ldr     r0, [r1, #0x48]
        cmp     r0, #0
        bne     mmdc_div

        /* Now Switch ARM to run from
          * step_clk sourced from OSC.
          */
        ldr     r0, [r1, #0xc]
        bic     r0, r1, #0x100
        str     r0, [r1, #0x0c]

        /* Now switch PLL1_SW_CLK to step_clk. */
        ldr     r0, [r1, #0x0c]
        orr     r0, r0, #0x4
        str     r0, [r1, #0x0c]

        ldr     r1, =ANATOP_BASE_ADDR

        /* Need to clock gate the 528 PFDs before
          * powering down PLL2.
          * Only the PLL2_PFD2_400M should be ON
          * as it feeds the MMDC/AHB
          */
        ldr     r0, [r1, #0x100]
        orr    r0, r0, #0x800000
        str     r0, [r1, #0x100]

        /* Now bypass PLL1 and PLL2. */
        ldr     r0, =0x10000
        str     r0, [r1, #0x4]
        str     r0, [r1, #0x34]

        /* Now do all the analog savings. */

        /* Disable 1p1 brown out. */
        ldr     r0, [r1, #0x110]
        bic     r0, r0, #0x2
        str     r0, [r1, #0x110]

        /* Enable the weak 2P5 */
        ldr     r0, [r1, #0x130]
        orr     r0, r0, #0x40000
        str     r0, [r1, #0x130]

        /*Disable main 2p5. */
        ldr     r0, [r1, #0x130]
        bic     r0, r0, #0x1
        str     r0, [r1, #0x130]


        /* Set the OSC bias current to -37.5%
        * to drop the power on VDDHIGH.
        */
        ldr     r0, [r1, #0x150]
        orr     r0, r0, #0xC000
        str     r0, [r1, #0x150]
        .endm



mx6_suspend_code:

#if 1
        ldr     r1, =0x021b0000

        /* set SBS_EN to block DDR access */
        ldr     r0, [r1, #0x410]
        orr     r0, r0, #0x100
        str     r0, [r1, #0x410]

        /* Put DDR explicitly into self-refresh. */
         /* Disable Automatic power savings. */
         ldr    r0, [r1, #0x404]
         orr    r0, r0, #0x01
         str    r0, [r1, #0x404]

         /* Make the DDR explicitly enter self-refresh. */
         ldr     r0, [r1, #0x404]
         orr    r0, r0, #0x200000
         str     r0, [r1, #0x404]

        /* set mmdc iomux to low power mode */
        ldr     r1, =0x020e0000

	sl_ddr_io_set_lpm
#endif

Lmx6_suspend_loop:
	wfi
	nop
	nop
	nop
	nop
	b Lmx6_suspend_loop


.globl mx6_suspend

mx6_suspend:

	cpsid i

	flush_disable_l1_dcache
	clean_l2_cache

	/* Disable L2 cache */
	ldr r2, =0x00a02000
	mov r4, #0x0
	str r4, [r2, #0x100]

// 11:
//	wfi
//	wfi
//	b 11b


	ldr r1, =mx6_suspend_code
	ldr r2, =0x00910000
	mov r3, #4096
 12:
	ldr r0, [r1],#4
	str r0, [r2],#4
	subs r3, r3, #4
	bne 12b

	mov r0, #0x00910000
	bx r0

	bx lr

