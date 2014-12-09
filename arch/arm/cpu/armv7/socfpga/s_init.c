/*
 * Copyright (C) 2014 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <watchdog.h>
#include <asm/io.h>
#include <asm/spl.h>
#include <asm/arch/system_manager.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * First C function to initialize the critical hardware early
 */
void s_init(void)
{
#ifdef CONFIG_SPL_BUILD
	struct socfpga_system_manager *sysmgr_regs =
		(struct socfpga_system_manager *)SOCFPGA_SYSMGR_ADDRESS;
	unsigned long reg;
	/*
	 * First C code to run. Clear fake OCRAM ECC first as SBE
	 * and DBE might triggered during power on
	 */
	reg = readl(&sysmgr_regs->eccgrp_ocram);
	if (reg & SYSMGR_ECC_OCRAM_SERR)
		writel(SYSMGR_ECC_OCRAM_SERR | SYSMGR_ECC_OCRAM_EN,
			&sysmgr_regs->eccgrp_ocram);
	if (reg & SYSMGR_ECC_OCRAM_DERR)
		writel(SYSMGR_ECC_OCRAM_DERR  | SYSMGR_ECC_OCRAM_EN,
			&sysmgr_regs->eccgrp_ocram);
#else
	/*
	 * Private components security
	 * U-Boot : configure private timer, global timer and cpu
	 * component access as non secure for kernel stage (as required
	 * by kernel)
	 */
	setbits_le32(SOCFPGA_SCU_SNSAC, 0xfff);

#endif	/* CONFIG_SPL_BUILD */

	/* Configure the L2 controller to make SDRAM start at 0	*/
	writel(0x1, (SOCFPGA_MPUL2_ADDRESS + SOCFPGA_MPUL2_ADRFLTR_START));
}
