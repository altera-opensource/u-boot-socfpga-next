/*
 * Copyright (C) 2017 Intel Corporation <www.intel.com>
 *
 * SPDX-License-Identifier:    GPL-2.0
 */

#include <asm-offsets.h>
#include <config.h>
#include <linux/linkage.h>

/*
 * Write RBF data in burst form to FPGA Manager
 * [r0] RBF binary source address
 * [r1] FPGA Manager data address
 * [r2] RBF data length
 */

ENTRY(fpgamgr_axi_write)
	PUSH    {r4-r11, lr}            /* save registers per AAPCS */

write_burst:
	cmp     r2,#32
	beq     write_burst_cont
	bls     write_word
write_burst_cont:
	ldmia   r0!, {r4-r11}
	stmia   r1, {r4-r11}
	subs    r2, r2, #32
	b       write_burst

write_word:
	cmp     r2,#4
	beq     write_word_cont
	bls     write_byte
write_word_cont:
	ldmia   r0!, {r4}
	stmia   r1, {r4}
	subs    r2, r2, #4
	b       write_word

write_byte:
	cmp     r2,#0
	beq     write_end
	ldr     r3, [r0]
	str     r3, [r1]
write_end:
	POP     {r4-r11, pc}
ENDPROC(fpgamgr_axi_write)
