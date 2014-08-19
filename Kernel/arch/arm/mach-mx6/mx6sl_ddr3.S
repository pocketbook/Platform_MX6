/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/linkage.h>
#include <mach/hardware.h>


	  .macro	ddr_switch_400MHz

     /* Check if we are switching between
       * 400Mhz <-> 50MHz. If so, we only need to
       * update MMDC divider.
       */
      cmp    r10, #0
      beq     6f

    /* Set MMDC divider first, in case PLL3 is at 480MHz. */
      ldr     r0, [r12, #0x10]
      and    r0, r0, #0x10000
      cmp   r0, #0x10000
      beq    0f
      /* Set MMDC divder to divide by 2. */
      ldr    r0, [r6, #0x14]
      bic    r0, r0, #0x38
      orr    r0, r0, #0x8
      str    r0, [r6, #0x14]

1:
     ldr     r0, [r6, #0x48]
     cmp     r0, #0
     bne     1b

0:

     /* Ensure that MMDC is sourced from PLL2 mux first. */
     ldr     r0, [r6, #0x14]
     bic     r0, r0, #0x4000000
     str     r0, [r6, #0x14]

2:
     ldr     r0, [r6, #0x48]
     cmp     r0, #0
     bne     2b

     /* Now ensure periph2_clk2_sel mux is set to PLL3 */
     ldr     r0, [r6, #0x18]
     bic     r0, r0, #0x100000
     str     r0, [r6, #0x18]

      /* Now switch MMDC to PLL3. */
     ldr     r0, [r6, #0x14]
     orr     r0, r0, #0x4000000
     str     r0, [r6, #0x14]

3:
     ldr     r0, [r6, #0x48]
     cmp     r0, #0
     bne     3b

     /* Now power up PLL2 and unbypass it. */
     ldr    r0, [r12, #0x30]
     bic    r0, r0, #0x1000
     str     r0, [r12, #0x30]

     /* Make sure PLL2 has locked.*/
4:
     ldr    r0, [r12, #0x30]
     and   r0, r0, #0x80000000
     cmp  r0, #0x80000000
     bne   4b

      ldr    r0, [r12, #0x30]
     bic    r0, r0, #0x10000
     str     r0, [r12, #0x30]

    /* Need to enable the 528 PFDs after
      * powering up PLL2.
      * Only the PLL2_PFD2_400M should be ON
      * as it feeds the MMDC. Rest should have
      * been managed by clock code.
      */
     ldr     r0, [r12, #0x100]
     bic    r0, r0, #0x800000
     str     r0, [r12, #0x100]

    /* Now switch MMDC clk back to pll2_mux option. */
    /* Ensure pre_periph2_clk2 is set to pll2_pfd_400M */
     ldr     r0, [r6, #0x18]
     bic     r0, r0, #0x600000
     orr     r0, r0, #0x200000
     str     r0, [r6, #0x18]

     ldr     r0, [r6, #0x14]
     bic     r0, r0, #0x4000000
     str     r0, [r6, #0x14]

5:
     ldr     r0, [r6, #0x48]
     cmp     r0, #0
     bne     5b

6:
    /* Calculate the MMDC divider
     * based on the requested freq.
     */
    ldr    r0, =400000000
    ldr    r3, =0
7:
    sub    r0, r0, r4
    cmp   r0, r4
    blt     8f
    add    r3, r3, #1
    bgt     7b

     /* Shift divider into correct offset. */
     lsl    r3, r3, #3
8:
     /* Set the MMDC PODF. */
      ldr    r0, [r6, #0x14]
      bic    r0, r0, #0x38
      orr    r0, r0, r3
      str    r0, [r6, #0x14]

9:
     ldr     r0, [r6, #0x48]
     cmp     r0, #0
     bne     9b

    .endm




      .macro   mmdc_clk_lower_100MHz

      /* Prior to reducing the DDR frequency (at 528/400 MHz),
      read the Measure unit count bits (MU_UNIT_DEL_NUM) */
      ldr     r2, =0x8B8
      ldr     r0, [r5, r2]
      /* Original MU unit count */
      mov   r0, r0, LSR #16
      ldr     r1, =0x3FF
      and    r0, r0, r1
      /* Original MU unit count * 2 */
      mov    r3, r0, LSL #1
      /* Bypass the automatic measure unit when below 100 MHz
      by setting the Measure unit bypass enable bit (MU_BYP_EN) */
      ldr     r0, [r5, r2]
      orr     r0, r0, #0x400
      str     r0, [r5, r2]
      /* Double the measure count value read in step 1 and program it in the
        * measurement bypass bits (MU_BYP_VAL) of the MMDC PHY Measure Unit
        * Register for the reduced frequency operation below 100 MHz
        */
       ldr    r0, [r5, r2]
       ldr    r1, =0x3FF
       bic    r0, r0, r1
       orr    r0, r0, r3
       str     r0, [r5, r2]
      /* Now perform a Force Measurement. */
      ldr     r0, [r5, r2]
      orr    r0, r0, #0x800
      str     r0, [r5, r2]
      /* Wait for FRC_MSR to clear. */
force_measure:
      ldr     r0, [r5, r2]
      and    r0, r0, #0x800
      cmp   r0, #0x0
      bne    force_measure

       .endm

      .macro   mmdc_clk_above_100MHz

      /* Make sure that the PHY measurement unit is NOT in bypass mode */
      ldr     r2, =0x8B8
      ldr     r0, [r5, r2]
      bic    r0, r0, #0x400
      str     r0, [r5, r2]
      /* Now perform a Force Measurement. */
      ldr     r0, [r5, r2]
      orr    r0, r0, #0x800
      str     r0, [r5, r2]
      /* Wait for FRC_MSR to clear. */
11:
      ldr     r0, [r5, r2]
      and    r0, r0, #0x800
      cmp   r0, #0x0
      bne    11b
      .endm




		.macro   switch_to_24MHz

      /* Set MMDC clock to be sourced from PLL3. */
      /* Ensure first periph2_clk2 is sourced from PLL3. */
      /* Set the PERIPH2_CLK2_PODF to divide by 2. */
     ldr    r0, [r6, #0x14]
     bic    r0, r0, #0x7
     orr    r0, r0, #0x1
     str     r0, [r6, #0x14]

     /* Select PLL3 to source MMDC. */
     ldr    r0, [r6, #0x18]
     bic    r0, r0, #0x100000
     str    r0, [r6, #0x18]

     /* Swtich periph2_clk_sel to run from PLL3. */
      ldr     r0, [r6, #0x14]
      orr     r0, r0, #0x4000000
      str     r0, [r6, #0x14]

periph2_clk_switch1:
     ldr     r0, [r6, #0x48]
     cmp     r0, #0
     bne     periph2_clk_switch1

    /* Need to clock gate the 528 PFDs before
      * powering down PLL2.
      * Only the PLL2_PFD2_400M should be ON
      * as it feeds the MMDC
      */
     ldr     r0, [r12, #0x100]
     orr    r0, r0, #0x800000
     str     r0, [r12, #0x100]

    /* Set PLL2 to bypass state. We should be here
      *only if MMDC is not sourced from PLL2.*/
     ldr    r0, [r12, #0x30]
     orr    r0, r0, #0x10000
     str     r0, [r12, #0x30]

     ldr    r0, [r12, #0x30]
     orr    r0, r0, #0x1000
     str     r0, [r12, #0x30]

    /* Ensure pre_periph2_clk_mux is set to pll2 */
     ldr     r0, [r6, #0x18]
     bic     r0, r0, #0x600000
     str     r0, [r6, #0x18]

     /* Set MMDC clock to be sourced from the bypassed PLL2. */
     ldr    r0, [r6, #0x14]
     bic    r0, r0, #0x4000000
     str    r0, [r6, #0x14]

periph2_clk_switch2:
     ldr     r0, [r6, #0x48]
     cmp     r0, #0
     bne     periph2_clk_switch2

    /* Now move MMDC back to periph2_clk2 source.
      * after selecting PLL2 as the option.
      */
     /* Select PLL2 as the source. */
     ldr    r0, [r6, #0x18]
     orr    r0, r0, #0x100000
     str    r0, [r6, #0x18]

    /* set periph2_clk2_podf to divide by 1. */
     ldr    r0, [r6, #0x14]
     bic    r0, r0, #0x7
     str     r0, [r6, #0x14]

     /* Now move periph2_clk to periph2_clk2 source */
     ldr    r0, [r6, #0x14]
     orr    r0, r0, #0x4000000
     str    r0, [r6, #0x14]

periph2_clk_switch3:
     ldr     r0, [r6, #0x48]
     cmp     r0, #0
     bne     periph2_clk_switch3

     /* Now set the MMDC PODF back to 1.*/
      ldr    r0, [r6, #0x14]
      bic    r0, r0, #0x38
      str    r0, [r6, #0x14]

mmdc_podf0:
     ldr     r0, [r6, #0x48]
     cmp     r0, #0
     bne     mmdc_podf0

	.endm



/*
 *  mx6l_ddr3_freq_change
 *
 *  Idle the processor (eg, wait for interrupt).
 *  Make sure DDR is in self-refresh.
 *  IRQs are already disabled.
 */
ENTRY(mx6l_ddr3_freq_change)

	stmfd   sp!, {r4,r5,r6, r7, r8, r9, r10, r11,r12}      @ Save registers

	ldr     r6, =CCM_BASE_ADDR
	add     r6, r6, #PERIPBASE_VIRT
	ldr     r5, =MMDC_P0_BASE_ADDR
	add     r5, r5, #PERIPBASE_VIRT
	ldr     r7, =MX6Q_IOMUXC_BASE_ADDR
	add     r7, r7, #PERIPBASE_VIRT
  ldr     r12, =ANATOP_BASE_ADDR
  add   	r12, r12, #PERIPBASE_VIRT

	mov			r10, r4				@save low_bus_freq_mode
	mov     r4, r0				@save new freq requested
	mov     r8, r1				@save the ddr settings for the new rate
	mov     r9, r2				@save the mode DDR is currently in (DLL ON/OFF)
	mov     r11, r3			@save iomux offsets

ddr_freq_change:
	/* Make sure no TLB miss will occur when the DDR is in self refresh. */
	/* Invalidate TLB single entry to ensure that the address is not
	  * already in the TLB.
	  */

	adr   r3, ddr_freq_change               @Address in this function.


	mcr   p15, 0, r3, c8, c7, 1      @//@ Make sure freq code address
	                                       @//  @ is not already in TLB.
	mcr   p15, 0, r6, c8, c7, 1      @//@ Make sure CCM address
	                                         @//@ is not already in TLB.
	mcr   p15, 0, r5, c8, c7, 1      @//@ make sure MMDC address
	                                         @//@ is not already in TLB.
	mcr   p15, 0, r7, c8, c7, 1      @//@ make sure IOMUX address
	                                         @//@ is not already in TLB.
	mcr   p15, 0, r12, c8, c7, 1      @//@ make sure ANATOP address
	                                         @//@ is not already in TLB.

	mrc   p15, 0, r0, c10, c0, 0    @//@ Read the TLB lockdown register
	orr    r0, r0, #1                    @//@ Set the Preserve bit.
	mcr   p15, 0, r0, c10, c0, 0    @//@ Write to the lockdown register

	ldr    r2, [r6]                        @ TLB will miss,
	                                         @CCM address will be loaded
	ldr    r2, [r5]                        @ TLB will miss,
	                                         @MMDC address will be loaded
	ldr    r2, [r7]                        @ TLB will miss,
	                                         @IOMUX will be loaded

	ldr    r2, [r8]                        @ Get the DDR settings.

	ldr    r2, [r3]                        @ TLB will miss

	ldr    r2, [r11]				@Get the IOMUX settings
	ldr    r2, [r12]

	mrc  p15, 0, r0, c10, c0, 0      @//@ Read the lockdown register
	                                             @//@ (victim will be incremented)
	bic   r0, r0, #1                     @//@ Clear the preserve bit
	mcr   p15, 0, r0, c10, c0, 0     @//@ Write to the lockdown register

	/* Disable automatic power saving. */

	ldr    r0, [r5, #0x404]
	orr    r0, r0, #0x01
	str    r0, [r5, #0x404]

	/* Disable MMDC power down timer. */
	/*MMDC0_MDPDC disable power down timer */
	ldr    r0, [r5, #0x4]
	bic    r0, r0, #0xff00
	str    r0, [r5, #0x4]

/* Delay for a while */
	ldr      r1, =4
delay1:
	ldr      r2, =0
cont1:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont1
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay1

	/* set CON_REG */
	ldr     r0, =0x8000
	str     r0, [r5, #0x1C]
poll_conreq_set_1:
	ldr     r0, [r5, #0x1C]
	and     r0, r0, #0x4000
	cmp	    r0, #0x4000
	bne     poll_conreq_set_1

	/*setmem /32 0x021b001c = 0x00008010 //Precharge all on cs0  */
	/*setmem /32 0x021b001c = 0x00008018 //Precharge all on cs1 */
	ldr     r0, =0x00008010
	str     r0, [r5, #0x1C]
	ldr     r0, =0x00008018
	str     r0, [r5, #0x1C]


	/* if requested frequency is greater than 300MHz go to DLL on mode */
	ldr     r1, =300000000
	cmp     r4, r1
	bge     dll_on_mode

dll_off_mode:

	/* if DLL is currently on, turn it off */
	cmp    	r9, #1
	beq 	continue_dll_off_1

	/* setmem /32 0x021b001c = 0x00018031 //Rtt_NOM off + set dll off, cs 0 */
	/* setmem /32 0x021b001c = 0x00018039 //Rtt_NOM off + set dll off, cs 1 */
	ldr     r0, =0x00018031
	str     r0, [r5, #0x1C]

	ldr     r0, =0x00018039
	str     r0, [r5, #0x1C]

	ldr      r1, =10
delay1a:
	ldr      r2, =0
cont1a:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont1a
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay1a

continue_dll_off_1:

	/* set DVFS - enter self refresh mode */
	ldr     r0, [r5, #0x404]
	orr    r0, r0, #0x200000
	str     r0, [r5, #0x404]

	/* de-assert con_req */
	mov    r0, #0x0
	str    r0, [r5, #0x1C]

poll_dvfs_set_1:
	ldr     r0, [r5, #0x404]
	and     r0, r0, #0x2000000
	cmp	    r0, #0x2000000
	bne   	poll_dvfs_set_1

	ldr     r1, =100000000
	cmp     r4, r1
	bgt    set_ddr_mu_above_100
	mmdc_clk_lower_100MHz




	ldr     r1, =24000000
	cmp     r4, r1
	beq     switch_freq_24

set_ddr_mu_above_100:
	ddr_switch_400MHz

	ldr     r1,  =100000000
	cmp     r4, r1
	blt continue_dll_off_2
	mmdc_clk_above_100MHz
	b continue_dll_off_2

switch_freq_24:
	switch_to_24MHz



continue_dll_off_2:

	/* set SBS - block ddr accesses */
	ldr     r0, [r5, #0x410]
	orr    	r0, r0, #0x100
	str     r0, [r5, #0x410]

	/* clear DVFS - exit from self refresh mode */
	ldr     r0, [r5, #0x404]
	bic     r0, r0, #0x200000
	str     r0, [r5, #0x404]

poll_dvfs_clear_1:
	ldr     r0, [r5, #0x404]
	and     r0, r0, #0x2000000
	cmp     r0, #0x2000000
	beq     poll_dvfs_clear_1


	/* if DLL was previously on, continue DLL off routine */
	cmp    	r9, #1
	beq 	continue_dll_off_3

	/* setmem /32 0x021b001c = 0x00018031 //Rtt_NOM off + set dll off, cs 0 */
	/* setmem /32 0x021b001c = 0x00018039 //Rtt_NOM off + set dll off, cs 1 */
	ldr     r0, =0x00018031
	str     r0, [r5, #0x1C]

	ldr     r0, =0x00018039
	str     r0, [r5, #0x1C]

	/* setmem /32 0x021b001c = 0x04208030 //write mode reg MR0: CL=6, wr=6 ,cs 0  */
	/* setmem /32 0x021b001c = 0x04208038 //write mode reg MR0: CL=6, wr=6 ,cs 1  */
	ldr     r0, =0x04208030
	str     r0, [r5, #0x1C]

	ldr     r0, =0x04208038
	str     r0, [r5, #0x1C]

	/* setmem /32 0x021b001c = 0x00088032 //write mode reg MR2 , CWL=6 ,cs0 */
	/* setmem /32 0x021b001c = 0x0008803A //write mode reg MR2 , CWL=6 ,cs1 */
	ldr     r0, =0x00088032
	str     r0, [r5, #0x1C]

	ldr     r0, =0x0008803A
	str     r0, [r5, #0x1C]



	/* double refresh ????
	 ldr     r0, =0x00001800
	 str     r0, [r5, #0x20]*/

	/* delay for a while. */
	ldr      r1, =4
delay_1:
	ldr      r2, =0
cont_1:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont_1
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay_1

	//setmem /32       0x021b000c =     0x07082423   // MMDC0_MDCFG0 - 400MHz - 0x3F435313
	ldr     r0, =0x07082423
	str     r0, [r5, #0x0C]

	//setmem /32       0x021b0010 =     0x24628B64  // MMDC0_MDCFG1 - 400MHz - 0xB68E8B63
	ldr     r0, =0x24628B64
	str     r0, [r5, #0x10]

	/* Enable bank interleaving, Address mirror on, WALAT = 0x1, RALAT = 0x2, DDR2_EN = 0 */
	/*setmem /32 0x021b0018 = 0x00091680 */
	ldr     r0, =0x00091680
	str     r0, [r5, #0x18]

	//setmem /32       0x021b0030 =     0x00081023         // MMDC0_MDOR - 400MHz - 0x00431023
	ldr     r0, =0x00081023
	str     r0, [r5, #0x30]


	/* enable dqs pull down in the IOMUX. */
	/*
		setmem /32	0x020e0344 =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0
		setmem /32	0x020e0348 =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1
		setmem /32	0x020e034c =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2
		setmem /32	0x020e0350 =	0x00003030	// IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3
	*/
	ldr    r1, [r11]                   @size of array
	add   r11, r11, #8            @skip first eight bytes in array
update_iomux:
	ldr    r0, [r11, #0x0]           @ offset
	ldr    r3, [r7, r0]
	orr	 r3, r3, #0x3000          @ Enable PUE and PKE bits.
	str    r3, [r7, r0]
	add   r11, r11, #8
	sub   r1, r1, #1
	cmp  r1, #0
	bgt   update_iomux

	/*  ODT disabled */
	/* setmem /32 0x021b0818 = 0x0      // DDR_PHY_P0_MPODTCTRL */
	ldr    r0, =0x0
	ldr    r2, =0x818
	str    r0, [r5, r2]

	/* DQS gating disabled */
	/* setmem /32 0x021b083c = 0x607E007F */
	ldr    r2, =0x83c
	ldr    r0, [r5, r2]
	orr     r0, r0, #0x20000000
	str    r0, [r5, r2]

	/* MMDC0_MAPSR adopt power down enable */
	/* setmem /32 0x021b0404 = 0x00011006 */
	ldr    r0, [r5, #0x404]
	bic    r0, r0, #0x01
	str    r0, [r5, #0x404]

	/* frc_msr + mu bypass*/
	ldr       r0, =0x00000060
	str       r0, [r5, #0x8b8]
	ldr       r0, =0x00000460
	str       r0, [r5, #0x8b8]
	ldr       r0, =0x00000c60
	str       r0, [r5, #0x8b8]


continue_dll_off_3:

	/* clear SBS - unblock accesses to DDR */
	ldr     r0, [r5, #0x410]
	bic     r0, r0, #0x100
	str     r0, [r5, #0x410]

	mov		r0, #0x0
	str		r0, [r5, #0x1C]
poll_conreq_clear_1:
	ldr		r0, [r5, #0x1C]
	and	    r0, r0, #0x4000
	cmp	    r0, #0x4000
	beq     poll_conreq_clear_1

	b        done

dll_on_mode:
	/* assert DVFS - enter self refresh mode */
	ldr     r0, [r5, #0x404]
	orr    r0, r0, #0x200000
	str     r0, [r5, #0x404]

	/* de-assert CON_REQ */
	mov    r0, #0x0
	str    r0, [r5, #0x1C]

	/* poll DVFS ack */
poll_dvfs_set_2:
	ldr     r0, [r5, #0x404]
	and     r0, r0, #0x2000000
	cmp     r0, #0x2000000
	bne     poll_dvfs_set_2

	ddr_switch_400MHz
	mmdc_clk_above_100MHz

continue_dll_on:

	/* set SBS step-by-step mode */
	ldr     r0, [r5, #0x410]
	orr     r0, r0, #0x100
	str     r0, [r5, #0x410]

	/* clear DVFS - exit self refresh mode */
	ldr     r0, [r5, #0x404]
	bic     r0, r0, #0x200000
	str     r0, [r5, #0x404]

poll_dvfs_clear_2:
	ldr     r0, [r5, #0x404]
	and     r0, r0, #0x2000000
	cmp     r0, #0x2000000
	beq     poll_dvfs_clear_2

	/* if DLL is currently off, turn it back on */
	cmp    	r9, #0
	beq 	       update_calibration_only

	/* enable DQS gating */
	ldr    r2, =0x83c
	ldr    r0, [r5, r2]
	bic    r0, r0, #0x20000000
	str    r0, [r5, r2]

	/* force measure */
	ldr       r0, =0x00000800
	str       r0, [r5, #0x8b8]

	/* delay for while */
	ldr      r1, =4
delay5:
	ldr      r2, =0
cont5:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont5
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay5
	/* Disable dqs pull down in the IOMUX. */
	/*
	setmem /32 0x020e05a8 = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS0 - DSE=110
	setmem /32 0x020e05b0 = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS1 - DSE=110
	setmem /32 0x020e0524 = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS2 - DSE=110
	setmem /32 0x020e051c = 0x00000030 // IOMUXC_SW_PAD_CTL_PAD_DRAM_SDQS3 - DSE=110
	*/
	ldr    r1, [r11]                   @size of array
	add  r11, r11, #8             @skip first eight bytes in array
update_iomux1:
	ldr    r0, [r11, #0x0]       @ offset
	ldr    r3, [r11, #0x4]
	str    r3, [r7, r0]		  @Store the original IOMUX value read during boot
	add   r11, r11, #8
	sub   r1, r1, #1
	cmp  r1, #0
	bgt   update_iomux1

	/* config ESDCTL timings to 400MHz:
		restore the value from array
	*/
	ldr     r1, [r8, #0x0]             @size of dll array
	ldr     r9, [r8, #0x4]			@size of calibration array
	add   r8, r8, #8             @skip first eight bytes in array
update_mmdc1:
	ldr    r0, [r8, #0x0]       @ offset
	ldr    r3, [r8, #0x4]       @ value
	str    r3, [r5, r0]
	add   r8, r8, #8
	sub   r1, r1, #1
	cmp  r1, #0
	bgt   update_mmdc1

	/* delay for while */
	ldr      r1, =40
delay15:
	ldr      r2, =0
cont15:
	ldr      r0, [r5, r2]
	add     r2, r2, #4
	cmp    r2, #16
	bne     cont15
	sub     r1, r1, #1
	cmp    r1, #0
	bgt     delay15

	/* MMDC0_MAPSR adopt power down enable */
	/* setmem /32 0x021b0404 = 0x00011006 */
	ldr    r0, [r5, #0x404]
	bic    r0, r0, #0x01
	str    r0, [r5, #0x404]

	/* Enable MMDC power down timer. */
	ldr    r0, [r5, #0x4]
	orr    r0, r0, #0x5500
	str    r0, [r5, #0x4]

	b 	update_calibration

update_calibration_only:
	ldr     r1, [r8, #0x0]             @size of dll array
	ldr     r9, [r8, #0x4]			@size of calibration array
	add   r8, r8, #8             @skip first eight bytes in array

mov_arrayptr:
	add   r8, r8, #8			@skip dll array
	sub   r1, r1, #1
	cmp  r1, #0
	bgt   mov_arrayptr

update_calibration:
	/* Write the new calibration values. */
	mov   r1, r9                 @ size of calibration array

update_calib:
	ldr    r0, [r8, #0x0]       @ offset
	ldr    r3, [r8, #0x4]       @ value
	str    r3, [r5, r0]
	add   r8, r8, #8
	sub   r1, r1, #1
	cmp  r1, #0
	bgt   update_calib

	/* Perform a force measurement. */
	ldr     r0, =0x800
	str     r0, [r5, #0x8B8]

	/* clear SBS - unblock DDR accesses */
	ldr     r0, [r5, #0x410]
	bic     r0, r0, #0x100
	str     r0, [r5, #0x410]

	mov		r0, #0x0
	str		r0, [r5, #0x1C]
poll_conreq_clear_2:
	ldr		r0, [r5, #0x1C]
	and	    r0, r0, #0x4000
	cmp	    r0, #0x4000
	beq     poll_conreq_clear_2

done:

	/* Restore registers */

	ldmfd sp!, {r4,r5,r6, r7, r8, r9, r10, r11, r12}

	mov     pc, lr

	.type   mx6l_do_ddr_freq_change, #object
ENTRY(mx6l_do_ddr_freq_change)
	.word   mx6l_ddr3_freq_change
	.size    mx6l_ddr3_freq_change, . - mx6l_ddr3_freq_change
