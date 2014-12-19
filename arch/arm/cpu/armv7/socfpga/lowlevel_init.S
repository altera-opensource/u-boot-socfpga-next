/*
 *  Copyright (C) 2012 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <config.h>
#include <version.h>
#include <linux/linkage.h>
#include <asm-offsets.h>
#include <asm/arch/reset_manager.h>
#include <asm/arch/system_manager.h>

/* Save the parameter pass in by previous boot loader */
.global save_boot_params
save_boot_params:
	/* no parameter to save */

#ifdef CONFIG_SPL_BUILD
	/*
	 * If BSEL is zero, then the bootrom has not
         * handed any information to preloader.
         */
	ldr	r9,=SYSMGR_BOOTINFO
	ldr 	r9, [r9]
	and	r9, r9, #SYSMGR_BOOTINFO_BSEL_MASK
	cmp	r9, #0
	bne	bsel_non_zero
	ldr	r4, =rst_mgr_status
	ldr	r5, =RSTMGR_COLDRST_MASK
	str	r5, [r4]
	b	bsel_zero

bsel_non_zero:
	/* get the reset manager status register passed by BootROM */
	add	r5, r0,#0x38
	ldr	r4, =rst_mgr_status
	ldr	r5, [r5]
	str	r5, [r4]

bsel_zero:
	/*
	 * Initialize the early stack point on OCRAM. This is to avoid OCRAM
	 * false double bit error as OCRAM always ready in 64 bit manner
	 */
	ldr	r1, =(CONFIG_SYS_INIT_SP_ADDR)	/* start of temporary SP */
	sub	r0, r1, #GD_SIZE	/* start of mem init */
	mov	r2, #0x00000000		/* prepare zero to mem init */
clrsp:	cmp	r0, r1			/* while not at end of BSS */
	strlo	r2, [r0]		/* clear 32-bit BSS word */
	addlo	r0, r0, #4		/* move to next */
	blo	clrsp
#endif	/* #ifdef CONFIG_SPL_BUILD */
	bx	lr

#ifdef CONFIG_SPL_BUILD
ENTRY(relocate_stack_to_sdram)
	PUSH	{r4-r11, lr}		/* save registers per AAPCS */

	mov	r0, sp			/* [r0] source address */
	ldr	r2, =__stack_start	/* [r2] source end address */
	sub	r3, r2, r0
	ldr	r4, =__sdram_stack_start
	sub	r1, r4, r3		/* [r1] target address */
	mov	r5, r1			/* [r5] new sp within SDRAM */

relocate_loop1:
	ldmia	r0!, {r3}
	stmia	r1!, {r3}
	cmp	r0, r2
	blo	relocate_loop1

	/* assign SP to new address within SDRAM now */
	mov	sp, r5

	POP	{r4-r11, pc}
ENDPROC(relocate_stack_to_sdram)
#endif /* CONFIG_SPL_BUILD */
