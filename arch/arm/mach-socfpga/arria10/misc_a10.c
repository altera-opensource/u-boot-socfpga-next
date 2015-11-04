/*
 * Copyright (C) 2014 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/sections.h>
#include <asm/arch/clock_manager_a10.h>
#include <asm/arch/fpga_manager_a10.h>
#include <asm/arch/reset_manager_a10.h>
#include <asm/arch/system_manager_a10.h>
#include <asm/arch/sdram_a10.h>
#include <asm/arch/dwmmc.h>
#include <asm/pl310.h>
#include <altera.h>
#include <dwmmc.h>
#include <fdtdec.h>
#include <fpga.h>
#include <mmc.h>
#include <netdev.h>
#include <phy.h>

DECLARE_GLOBAL_DATA_PTR;

static const struct pl310_regs *pl310_regs_base = (void *)CONFIG_SYS_PL310_BASE;
static const struct socfpga_noc_fw_ocram *noc_fw_ocram_base =
		(void *)SOCFPGA_SDR_FIREWALL_OCRAM_ADDRESS;
static const struct socfpga_noc_fw_ddr_l3 *noc_fw_ddr_l3_base =
		(void *)SOCFPGA_SDR_FIREWALL_L3_ADDRESS;

/* FPGA programming support for SoC FPGA Arria 10 */
static Altera_desc altera_fpga[] = {
	{
		/* Family */
		Altera_SoCFPGA,
		/* Interface type */
		fast_passive_parallel,
		/* No limitation as additional data will be ignored */
		-1,
		/* No device function table */
		NULL,
		/* Base interface address specified in driver */
		NULL,
		/* No cookie implementation */
		0
	},
};

/* add device descriptor to FPGA device table */
static void socfpga_fpga_add(void)
{
	int i;
	fpga_init();
	for (i = 0; i < ARRAY_SIZE(altera_fpga); i++)
		fpga_add(fpga_altera, &altera_fpga[i]);
}

void v7_outer_cache_enable(void)
{
	/* disable the L2 cache */
	writel(0, &pl310_regs_base->pl310_ctrl);

	/* enable BRESP, instruction and data prefetch, full line of zeroes */
	setbits_le32(&pl310_regs_base->pl310_aux_ctrl,
		     L310_AUX_CTRL_DATA_PREFETCH_MASK |
		     L310_AUX_CTRL_INST_PREFETCH_MASK); 
}

/*
 * This function initializes security policies to be consistent across
 * all logic units in the Arria 10.
 *
 * The idea is to set all security policies to be normal, nonsecure
 * for all units.
 */
static void initialize_security_policies(void)
{
	/* Put OCRAM in non-secure */
	writel(0x003f0000, &noc_fw_ocram_base->region0);
	writel(0x1, &noc_fw_ocram_base->enable);

	/* Put DDR in non-secure */
	writel(0xffff0000, &noc_fw_ddr_l3_base->hpsregion0addr);
	writel(0x1, &noc_fw_ddr_l3_base->enable);
}

int arch_early_init_r(void)
{
	initialize_security_policies();

	/* Configure the L2 controller to make SDRAM start at 0 */
	writel(0x1, &pl310_regs_base->pl310_addr_filter_start);

	/* assert reset to all except L4WD0 and L4TIMER0 */
	reset_assert_all_peripherals_except_l4wd0_l4timer0();

	/* configuring the clock based on handoff */
	cm_basic_init(gd->fdt_blob);

	/* configure the Reset Manager */
	reset_deassert_dedicated_peripherals();

	if (is_external_fpga_config(gd->fdt_blob)) {
		while (!is_fpgamgr_user_mode()) ;

		reset_deassert_shared_connected_peripherals();
		reset_deassert_fpga_connected_peripherals();
	}

	/* Add device descriptor to FPGA device table */
	socfpga_fpga_add();
	return 0;
}

/*
 * Print CPU information
 */
#if defined(CONFIG_DISPLAY_CPUINFO)
int print_cpuinfo(void)
{
	puts("CPU   : Altera SOCFPGA Arria 10 Platform\n");
	return 0;
}
#endif

#if defined(CONFIG_SYS_CONSOLE_IS_IN_ENV) && \
defined(CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE)
int overwrite_console(void)
{
	return 0;
}
#endif

#ifdef CONFIG_DWMMC
/*
 * Initializes MMC controllers.
 * to override, implement board_mmc_init()
 */
int cpu_mmc_init(bd_t * bis)
{
	return socfpga_dwmmc_init(gd->fdt_blob);
}
#endif

/* Enable D-cache. I-cache is already enabled in start.S */
#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	dcache_enable();
}
#endif

/* Skip relocation as U-Boot cannot run on SDRAM for secure boot */
void skip_relocation(void)
{
	bd_t *bd;
	gd_t *id;
	unsigned long addr, size = 0, i;

	puts("INFO  : Skip relocation as SDRAM is non secure memory\n");

#if !(defined(CONFIG_SYS_ICACHE_OFF) && defined(CONFIG_SYS_DCACHE_OFF))
	/* reserve TLB table */
	gd->arch.tlb_size = 4096 * 4;

	/* page table is located at last 16kB of OCRAM */
	addr = CONFIG_SYS_INIT_SP_ADDR;
	gd->arch.tlb_addr = addr;
	debug("TLB table from %08lx to %08lx\n", addr,
	      addr + gd->arch.tlb_size);
#endif

	/* After page table, it would be stack then follow by malloc */
	addr -= (CONFIG_OCRAM_STACK_SIZE + CONFIG_OCRAM_MALLOC_SIZE);

	/*
	 * (permanently) allocate a Board Info struct
	 * and a permanent copy of the "global" data
	 */
	addr -= sizeof(bd_t);
	bd = (bd_t *) addr;
	gd->bd = bd;
	debug("Reserving %zu Bytes for Board Info at: %08lx\n",
	      sizeof(bd_t), addr);

#ifdef CONFIG_MACH_TYPE
	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;	/* board id for Linux */
#endif

	addr -= sizeof(gd_t);
	id = (gd_t *) addr;
	debug("Reserving %zu Bytes for Global Data at: %08lx\n",
	      sizeof(gd_t), addr);

	/* setup stackpointer for exeptions */
	gd->irq_sp = addr;
#ifdef CONFIG_USE_IRQ
	addr -= (CONFIG_STACKSIZE_IRQ + CONFIG_STACKSIZE_FIQ);
	printf("Reserving %zu Bytes for IRQ stack at: %08lx\n",
	       CONFIG_STACKSIZE_IRQ + CONFIG_STACKSIZE_FIQ, addr);
#endif
	interrupt_init();

#ifdef CONFIG_POST
	post_bootmode_init();
	post_run(NULL, POST_ROM | post_bootmode_get(0));
#endif

	/* gd->bd->bi_baudrate = gd->baudrate; */
	/* setup the dram info within bd */
	dram_init_banksize();

	/* and display it */
	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++)
		size += gd->bd->bi_dram[i].size;
	puts("DRAM  : ");
	print_size(size, "\n");

	gd->relocaddr = CONFIG_SYS_TEXT_BASE;
	gd->start_addr_sp = CONFIG_SYS_INIT_SP_ADDR;
	gd->reloc_off = 0;
	debug("relocation Offset is: %08lx\n", gd->reloc_off);

	/* copy all old global data to new allocated location */
	memcpy(id, (void *)gd, sizeof(gd_t));

	/* assign the global data to new location, no longer in stack */
	gd = id;

	/* rebase the stack pointer as we won't jump back here */
	asm volatile ("mov sp, %0"::"r" (gd->start_addr_sp)
		      :"r0", "r1", "r2", "r3", "ip", "lr", "memory", "cc");

	/* change the destination address so later malloc init will point to
	   existing allocated malloc memory space */
	board_init_r(id, (CONFIG_SYS_INIT_SP_ADDR - CONFIG_OCRAM_STACK_SIZE));
}

int is_external_fpga_config(const void *blob)
{
	int node, len;
	int rval = 0;

	node = fdt_subnode_offset(blob, 0, "chosen");
	if (node >= 0) {
		if (fdt_getprop(blob, node, "external-fpga-config", &len))
			rval = 1;
	}

	return rval;
}
