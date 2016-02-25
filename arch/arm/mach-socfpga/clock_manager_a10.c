/*
 * Copyright (C) 2016 Altera Corporation. All rights reserved
 *
 * SPDX-License-Identifier:	GPL-2.0
 */
#include <common.h>
#include <errno.h>
#include <clk.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/clk.h>

/* Clock Manager offsets */
#define CLK_MGR_PLL_CLK_SRC_SHIFT	8
#define CLK_MGR_PLL_CLK_SRC_MASK	0x3

/* Clock bypass bits */
#define SOCFPGA_PLL_BG_PWRDWN		0
#define SOCFPGA_PLL_PWR_DOWN		1
#define SOCFPGA_PLL_EXT_ENA		2
#define SOCFPGA_PLL_DIVF_MASK		0x00001FFF
#define SOCFPGA_PLL_DIVF_SHIFT	0
#define SOCFPGA_PLL_DIVQ_MASK		0x003F0000
#define SOCFPGA_PLL_DIVQ_SHIFT	16
#define SOCFGPA_MAX_PARENTS	5

#define SOCFPGA_MAIN_PLL_CLK		"main_pll"
#define SOCFPGA_PERIP_PLL_CLK		"periph_pll"

DECLARE_GLOBAL_DATA_PTR;

struct clk;

/**
 * struct socfpga_a10_clk_ops:
 * @set_rate:   Function pointer to set_rate() implementation
 * @get_rate:   Function pointer to get_rate() implementation
 */
struct socfpga_a10_clk_ops {
	int (*set_rate)(struct clk *clk, unsigned long rate);
	unsigned long (*get_rate)(struct clk *clk);
};

/**
 * struct clk:
 * @name:       Clock name
 * @frequency:  Currenct frequency
 * @parent:     Parent clock
 * @flags:      Clock flags
 * @reg:        Clock control register
 * @ops:        Clock operations
 */
struct clk {
	char				*name;
	unsigned long			frequency;
	enum socfpga_a10_clk		parent;
	unsigned int			flags;
	u32				*reg;
	struct socfpga_a10_clk_ops	ops;
};

static struct clk clks[clk_max];

void __iomem *clk_mgr_a10_base_addr;

static unsigned long clk_pll_recalc_rate(struct clk_hw *hwclk,
					 unsigned long parent_rate)
{
	struct socfpga_pll *socfpgaclk = to_socfpga_clk(hwclk);
	unsigned long divf, divq, reg;
	unsigned long long vco_freq;

	/* read VCO1 reg for numerator and denominator */
	reg = readl(socfpgaclk->hw.reg + 0x4);
	divf = (reg & SOCFPGA_PLL_DIVF_MASK) >> SOCFPGA_PLL_DIVF_SHIFT;
	divq = (reg & SOCFPGA_PLL_DIVQ_MASK) >> SOCFPGA_PLL_DIVQ_SHIFT;
	vco_freq = (unsigned long long)parent_rate * (divf + 1);
	do_div(vco_freq, (1 + divq));
	return (unsigned long)vco_freq;
}

static u8 clk_pll_get_parent(struct clk_hw *hwclk)
{
	struct socfpga_pll *socfpgaclk = to_socfpga_clk(hwclk);
	u32 pll_src;

	pll_src = readl(socfpgaclk->hw.reg);

	return (pll_src >> CLK_MGR_PLL_CLK_SRC_SHIFT) &
		CLK_MGR_PLL_CLK_SRC_MASK;
}

int set_cpu_clk_info(void)
{
	struct udevice *dev;

	for (uclass_first_device(UCLASS_CLK, &dev);
	

	return 0;
}
