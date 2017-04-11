/*
 * Specialty padding for the Altera SoCFPGA preloader image
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __BOOT0_H
#define __BOOT0_H

#ifdef CONFIG_SPL_BUILD
	.balignl 64,0xf33db33f;

	/* Start of header offset */
	.word	0x1337c0d3; /* SoCFPGA preloader validation word(4B) */
	.word	0xc01df00d; /* Header length(2B),flags(1B),version(1B) */
#ifndef CONFIG_TARGET_SOCFPGA_GEN5
	.word	0xcafec0d3; /* Program length(4B) */
	.word	0xf00dcafe; /* Program entry offset(4B),relative to  */
			    /* the start of program header */
#endif
	.word	0xfeedface; /* Simple checksum(2B),spare offset(2B) */
	nop;

	b reset;	    /* SoCFPGA jumps here */
	nop;
	nop;
	nop;
#endif

#endif /* __BOOT0_H */
