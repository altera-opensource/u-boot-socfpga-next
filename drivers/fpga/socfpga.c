/*
 * Copyright (C) 2012-2017 Altera Corporation <www.altera.com>
 * All rights reserved.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 */

#include <common.h>
#include <asm/io.h>
#include <linux/errno.h>
#include <asm/arch/fpga_manager.h>
#include <asm/arch/reset_manager.h>
#include <asm/arch/system_manager.h>

DECLARE_GLOBAL_DATA_PTR;

/* Timeout count */
#define FPGA_TIMEOUT_CNT	0x1000000

static struct socfpga_fpga_manager *fpgamgr_regs =
	(struct socfpga_fpga_manager *)SOCFPGA_FPGAMGRREGS_ADDRESS;

int fpgamgr_dclkcnt_set(unsigned long cnt)
{
	unsigned long i;

	/* Clear any existing done status */
	if (readl(&fpgamgr_regs->dclkstat))
		writel(0x1, &fpgamgr_regs->dclkstat);

	/* Write the dclkcnt */
	writel(cnt, &fpgamgr_regs->dclkcnt);

	/* Wait till the dclkcnt done */
	for (i = 0; i < FPGA_TIMEOUT_CNT; i++) {
		if (!readl(&fpgamgr_regs->dclkstat))
			continue;

		writel(0x1, &fpgamgr_regs->dclkstat);
		return 0;
	}

	return -ETIMEDOUT;
}
