// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016-2018 Intel Corporation <www.intel.com>
 *
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/reset_manager.h>
#include <asm/arch/system_manager.h>
#include <dt-bindings/reset/altr,rst-mgr-s10.h>

DECLARE_GLOBAL_DATA_PTR;

#define GICD_CTRL_ADDRESS	0xfffc1000

/* Assert or de-assert SoCFPGA reset manager reset. */
void socfpga_per_reset(u32 reset, int set)
{
	static const struct socfpga_reset_manager *reset_manager_base =
			(void *)SOCFPGA_RSTMGR_ADDRESS;
	const void *reg;

	if (RSTMGR_BANK(reset) == 0)
		reg = &reset_manager_base->mpumodrst;
	else if (RSTMGR_BANK(reset) == 1)
		reg = &reset_manager_base->per0modrst;
	else if (RSTMGR_BANK(reset) == 2)
		reg = &reset_manager_base->per1modrst;
	else if (RSTMGR_BANK(reset) == 3)
		reg = &reset_manager_base->brgmodrst;
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
 */
void socfpga_per_reset_all(void)
{
	static const struct socfpga_reset_manager *reset_manager_base =
			(void *)SOCFPGA_RSTMGR_ADDRESS;
	const u32 l4wd0 = 1 << RSTMGR_RESET(SOCFPGA_RESET(L4WD0));

	/* disable all except OCP and l4wd0. OCP disable later */
	writel(~(l4wd0 | RSTMGR_PER0MODRST_OCP_MASK),
	       &reset_manager_base->per0modrst);
	writel(~l4wd0, &reset_manager_base->per0modrst);
	writel(0xffffffff, &reset_manager_base->per1modrst);
}

static __always_inline void socfpga_bridges_reset_common(int enable)
{
	static const struct socfpga_reset_manager *reset_manager_base =
			(void *)SOCFPGA_RSTMGR_ADDRESS;
	static const struct socfpga_system_manager *system_manager_base =
			(void *)SOCFPGA_SYSMGR_ADDRESS;

	if (enable) {
		/* clear idle request to all bridges */
		setbits_le32(&system_manager_base->noc_idlereq_clr, ~0);

		/* Release all bridges from reset state */
		clrbits_le32(&reset_manager_base->brgmodrst, ~0);

		/* Poll until all idleack to 0 */
		while (readl(&system_manager_base->noc_idleack))
			;
	} else {
		/* set idle request to all bridges */
		writel(~0, &system_manager_base->noc_idlereq_set);

		/* Enable the NOC timeout */
		writel(1, &system_manager_base->noc_timeout);

		/* Poll until all idleack to 1 */
		while ((readl(&system_manager_base->noc_idleack) ^
			(SYSMGR_NOC_H2F_MSK | SYSMGR_NOC_LWH2F_MSK)))
			;

		/* Poll until all idlestatus to 1 */
		while ((readl(&system_manager_base->noc_idlestatus) ^
			(SYSMGR_NOC_H2F_MSK | SYSMGR_NOC_LWH2F_MSK)))
			;

		/* Reset all bridges (except NOR DDR scheduler & F2S) */
		setbits_le32(&reset_manager_base->brgmodrst,
			     ~(RSTMGR_BRGMODRST_DDRSCH_MASK |
			     RSTMGR_BRGMODRST_FPGA2SOC_MASK));

		/* Disable NOC timeout */
		writel(0, &system_manager_base->noc_timeout);
	}
}

void socfpga_bridges_reset(int enable)
{
	socfpga_bridges_reset_common(enable);
}

/*
 * This function will be copied to 'secure' memory region
 * to support FPGA programming SMC services.
 */
void __secure socfpga_bridges_reset_psci(int enable)
{
	socfpga_bridges_reset_common(enable);
}

/*
 * Release peripherals from reset based on handoff
 */
void reset_deassert_peripherals_handoff(void)
{
	static const struct socfpga_reset_manager *reset_manager_base =
			(void *)SOCFPGA_RSTMGR_ADDRESS;

	writel(0, &reset_manager_base->per1modrst);
	/* Enable OCP first */
	writel(~RSTMGR_PER0MODRST_OCP_MASK, &reset_manager_base->per0modrst);
	writel(0, &reset_manager_base->per0modrst);
}

/*
 * Return non-zero if the CPU has been warm reset
 */
int cpu_has_been_warmreset(void)
{
	static const struct socfpga_reset_manager *reset_manager_base =
			(void *)SOCFPGA_RSTMGR_ADDRESS;

	return readl(&reset_manager_base->status) &
		RSTMGR_L4WD_MPU_WARMRESET_MASK;
}

void l2_reset_cpu(void)
{
	asm volatile(
		/* Disable GIC distributor (IRQs). */
		"str    wzr, [%3]\n"
		/* Set Magic Number */
		"str	%0, [%1]\n"
		/* Increase timeout in rstmgr.hdsktimeout */
		"ldr	x2, =0xFFFFFF\n"
		"str	w2, [%2, #0x64]\n"
		"ldr	w2, [%2, #0x10]\n"
		/*
		 * Set l2flushen = 1, etrstallen = 1,
		 * fpgahsen = 1 and sdrselfrefen = 1
		 * in rstmgr.hdsken to perform handshake
		 * in certain peripherals before trigger
		 * L2 reset.
		 */
		"ldr	x3, =0x10D\n"
		"orr	x2, x2, x3\n"
		"str	w2, [%2, #0x10]\n"
		/* Trigger L2 reset in rstmgr.coldmodrst */
		"ldr	w2, [%2, #0x34]\n"
		"orr	x2, x2, #0x100\n"
		"isb\n"
		"dsb	sy\n"
		"str	w2, [%2, #0x34]\n"
		/* Put all cores into WFI mode */
		"wfi_loop:\n"
		"	wfi\n"
		"	b	wfi_loop\n"
		: : "r" (L2_RESET_DONE_STATUS),
		    "r" (L2_RESET_DONE_REG),
		    "r" (SOCFPGA_RSTMGR_ADDRESS),
		    "r" (GICD_CTRL_ADDRESS)
		: "x1", "x2", "x3");
}

void __secure l2_reset_cpu_psci(void)
{
	asm volatile(
		/* Disable GIC distributor (IRQs). */
		"str    wzr, [%3]\n"
		/* Set Magic Number */
		"str	%0, [%1]\n"
		/* Increase timeout in rstmgr.hdsktimeout */
		"ldr	x2, =0xFFFFFF\n"
		"str	w2, [%2, #0x64]\n"
		"ldr	w2, [%2, #0x10]\n"
		/*
		 * Set l2flushen = 1, etrstallen = 1,
		 * fpgahsen = 1 and sdrselfrefen = 1
		 * in rstmgr.hdsken to perform handshake
		 * in certain peripherals before trigger
		 * L2 reset.
		 */
		"ldr	x3, =0x10D\n"
		"orr	x2, x2, x3\n"
		"str	w2, [%2, #0x10]\n"
		/* Trigger L2 reset in rstmgr.coldmodrst */
		"ldr	w2, [%2, #0x34]\n"
		"orr	x2, x2, #0x100\n"
		"isb\n"
		"dsb	sy\n"
		"str	w2, [%2, #0x34]\n"
		/* Put all cores into WFI mode */
		"wfi_loop2:\n"
		"	wfi\n"
		"	b	wfi_loop2\n"
		: : "r" (L2_RESET_DONE_STATUS),
		    "r" (L2_RESET_DONE_REG),
		    "r" (SOCFPGA_RSTMGR_ADDRESS),
		    "r" (GICD_CTRL_ADDRESS)
		: "x1", "x2", "x3");
}
