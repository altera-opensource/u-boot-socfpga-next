/*
 *  Copyright (C) 2013 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */


#include <common.h>
#include <asm/io.h>
#include <asm/arch/fpga_manager.h>
#include <asm/arch/reset_manager.h>
#include <asm/arch/system_manager.h>

DECLARE_GLOBAL_DATA_PTR;

static const struct socfpga_reset_manager *reset_manager_base =
		(void *)SOCFPGA_RSTMGR_ADDRESS;
static struct socfpga_system_manager *sysmgr_regs =
	(struct socfpga_system_manager *)SOCFPGA_SYSMGR_ADDRESS;

/*
 * Assert or de-assert SoCFPGA reset manager reset.
 */
void socfpga_per_reset(u32 reset, int set)
{
	const void *reg;

	if (RSTMGR_BANK(reset) == 0)
		reg = &reset_manager_base->mpu_mod_reset;
	else if (RSTMGR_BANK(reset) == 1)
		reg = &reset_manager_base->per_mod_reset;
	else if (RSTMGR_BANK(reset) == 2)
		reg = &reset_manager_base->per2_mod_reset;
	else if (RSTMGR_BANK(reset) == 3)
		reg = &reset_manager_base->brg_mod_reset;
	else if (RSTMGR_BANK(reset) == 4)
		reg = &reset_manager_base->misc_mod_reset;
	else	/* Invalid reset register, do nothing */
		return;

	if (set)
		setbits_le32(reg, 1 << RSTMGR_RESET(reset));
	else
		clrbits_le32(reg, 1 << RSTMGR_RESET(reset));
}

/*
 * Assert reset on every peripheral but L4WD0.
 * Watchdog must be kept intact to prevent glitches
 * and/or hangs.
 * For the Arria10, we disable all the peripherals except L4 watchdog0,
 * L4 Timer 0, and ECC.
 */
void socfpga_per_reset_all(void)
{
#if defined(CONFIG_TARGET_SOCFPGA_GEN5)
	const u32 l4wd0 = 1 << RSTMGR_RESET(SOCFPGA_RESET(L4WD0));

	writel(~l4wd0, &reset_manager_base->per_mod_reset);
	writel(0xffffffff, &reset_manager_base->per2_mod_reset);
#else
	const u32 l4wd0 = (1 << RSTMGR_RESET(SOCFPGA_RESET(L4WD0)) |
			(1 << RSTMGR_RESET(SOCFPGA_RESET(L4SYSTIMER0))));

	unsigned mask_ecc_ocp = 0x0000FF00;

	/* disable all components except ECC_OCP, L4 Timer0 and L4 WD0 */
	writel(~l4wd0, &reset_manager_base->per1_mod_reset);
	setbits_le32(&reset_manager_base->per0_mod_reset, ~mask_ecc_ocp);

	/* Finally disable the ECC_OCP */
	setbits_le32(&reset_manager_base->per0_mod_reset, mask_ecc_ocp);
#endif
}

/*
 * Write the reset manager register to cause reset
 */
void reset_cpu(ulong addr)
{
	/* request a warm reset */
	writel((1 << RSTMGR_CTRL_SWWARMRSTREQ_LSB),
		&reset_manager_base->ctrl);
	/*
	 * infinite loop here as watchdog will trigger and reset
	 * the processor
	 */
	while (1)
		;
}

#if defined(CONFIG_TARGET_SOCFPGA_GEN5)
/*
 * Release peripherals from reset based on handoff
 */
void reset_deassert_peripherals_handoff(void)
{
	writel(0, &reset_manager_base->per_mod_reset);
}
#endif

#if defined(CONFIG_SOCFPGA_VIRTUAL_TARGET)
void socfpga_bridges_reset(int enable)
{
	/* For SoCFPGA-VT, this is NOP. */
}
#else

#define L3REGS_REMAP_LWHPS2FPGA_MASK	0x10
#define L3REGS_REMAP_HPS2FPGA_MASK	0x08
#define L3REGS_REMAP_OCRAM_MASK		0x01

void socfpga_bridges_reset(int enable)
{
#if defined(CONFIG_TARGET_SOCFPGA_GEN5)
	const uint32_t l3mask = L3REGS_REMAP_LWHPS2FPGA_MASK |
				L3REGS_REMAP_HPS2FPGA_MASK |
				L3REGS_REMAP_OCRAM_MASK;

	if (enable) {
		/* brdmodrst */
		writel(0xffffffff, &reset_manager_base->brg_mod_reset);
	} else {
		writel(0, &sysmgr_regs->iswgrp_handoff[0]);
		writel(l3mask, &sysmgr_regs->iswgrp_handoff[1]);

		/* Check signal from FPGA. */
		if (!fpgamgr_test_fpga_ready()) {
			/* FPGA not ready, do nothing. */
			printf("%s: FPGA not ready, aborting.\n", __func__);
			return;
		}

		/* brdmodrst */
		writel(0, &reset_manager_base->brg_mod_reset);

		/* Remap the bridges into memory map */
		writel(l3mask, SOCFPGA_L3REGS_ADDRESS);
	}
#else
	u32 reset_mask;

	if (enable)
		;
	else {
		/* set idle request to all bridges */
		writel(ALT_SYSMGR_NOC_H2F_SET_MSK |
		       ALT_SYSMGR_NOC_LWH2F_SET_MSK |
		       ALT_SYSMGR_NOC_F2H_SET_MSK |
		       ALT_SYSMGR_NOC_F2SDR0_SET_MSK |
		       ALT_SYSMGR_NOC_F2SDR1_SET_MSK |
		       ALT_SYSMGR_NOC_F2SDR2_SET_MSK,
		       &sysmgr_regs->noc_idlereq_set);

		/* Enable the NOC timeout */
		writel(ALT_SYSMGR_NOC_TMO_EN_SET_MSK,
		       &sysmgr_regs->noc_timeout);

		/* Poll until all idleack to 1 */
		while ((readl(&sysmgr_regs->noc_idleack) ^
		       (ALT_SYSMGR_NOC_H2F_SET_MSK |
			ALT_SYSMGR_NOC_LWH2F_SET_MSK |
			ALT_SYSMGR_NOC_F2H_SET_MSK |
			ALT_SYSMGR_NOC_F2SDR0_SET_MSK |
			ALT_SYSMGR_NOC_F2SDR1_SET_MSK |
			ALT_SYSMGR_NOC_F2SDR2_SET_MSK)));

		/* Poll until all idlestatus to 1 */
		while ((readl(&sysmgr_regs->noc_idlestatus) ^
		       (ALT_SYSMGR_NOC_H2F_SET_MSK |
			ALT_SYSMGR_NOC_LWH2F_SET_MSK |
			ALT_SYSMGR_NOC_F2H_SET_MSK |
			ALT_SYSMGR_NOC_F2SDR0_SET_MSK |
			ALT_SYSMGR_NOC_F2SDR1_SET_MSK |
			ALT_SYSMGR_NOC_F2SDR2_SET_MSK)));

		/* Put all bridges (except NOR DDR scheduler) into reset state */
		socfpga_per_reset(SOCFPGA_RESET(HPS2FPGA), 1);
		socfpga_per_reset(SOCFPGA_RESET(LWHPS2FPGA), 1);
		socfpga_per_reset(SOCFPGA_RESET(FPGA2HPS), 1);
		socfpga_per_reset(SOCFPGA_RESET(F2SSDRAM0), 1);
		socfpga_per_reset(SOCFPGA_RESET(F2SSDRAM1), 1);
		socfpga_per_reset(SOCFPGA_RESET(F2SSDRAM2), 1);
	}
#endif
}
#endif
