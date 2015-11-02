/*
 * Copyright (C) 2014 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	_SOCFPGA_ECC_RAM_A10_H_
#define	_SOCFPGA_ECC_RAM_A10_H_

#define IRQ_ECC_OCRAM_CORRECTED		210
#define IRQ_ECC_OCRAM_UNCORRECTED	211

extern unsigned long irq_cnt_ecc_ocram_corrected;
extern unsigned long irq_cnt_ecc_ocram_uncorrected;

#ifndef __ASSEMBLY__

struct socfpga_ecc {
	u32 ip_rev_id;
	u32 _pad_0x4_0x7;
	u32 ctrl;
	u32 initstat;
	u32 errinten;
	u32 errintens;
	u32 errintenr;
	u32 intmode;
	u32 intstat;
	u32 inttest;
	u32 modstat;
	u32 derraddra;
	u32 serraddra;
	u32 _pad_0x34_0x3b[2];
	u32 serrcntreg;
	u32 ecc_addrbus;
	u32 ecc_rdata0bus;
	u32 ecc_rdata1bus;
	u32 ecc_rdata2bus;
	u32 ecc_rdata3bus;
	u32 ecc_wdata0bus;
	u32 ecc_wdata1bus;
	u32 ecc_wdata2bus;
	u32 ecc_wdata3bus;
	u32 ecc_rdataecc0bus;
	u32 ecc_rdataecc1bus;
	u32 ecc_wdataecc0bus;
	u32 ecc_wdataecc1bus;
	u32 ecc_dbytectrl;
	u32 ecc_accctrl;
	u32 ecc_startacc;
	u32 ecc_wdctrl;
	u32 _pad_0x84_0x8f[3];
	u32 serrlkupa0;
};

void irq_handler_ecc_ram_serr(void);
void irq_handler_ecc_ram_derr(void);
void enable_ecc_ram_serr_int(void);
void clear_ecc_ocram_ecc_status(void);
#endif /* __ASSEMBLY__ */

#define ALT_ECC_INTSTAT_SERRPENA_SET_MSK	0x00000001
#define ALT_ECC_INTSTAT_DERRPENA_SET_MSK	0x00000100
#define ALT_ECC_ERRINTEN_SERRINTEN_SET_MSK	0x00000001

#endif /* _SOCFPGA_ECC_RAM_A10_H_ */
