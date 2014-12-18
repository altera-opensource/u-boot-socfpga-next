/*
 *  Copyright (C) 2012 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	_RESET_MANAGER_H_
#define	_RESET_MANAGER_H_

#ifndef __ASSEMBLY__
void reset_cpu(ulong addr);
void reset_deassert_peripherals_handoff(void);

void socfpga_bridges_reset(int enable);

void socfpga_emac_reset(int enable);
void socfpga_watchdog_reset(void);
void socfpga_spim_enable(void);
void socfpga_uart0_enable(void);
void socfpga_sdram_enable(void);
void socfpga_osc1timer_enable(void);

struct socfpga_reset_manager {
	u32	status;
	u32	ctrl;
	u32	counts;
	u32	padding1;
	u32	mpu_mod_reset;
	u32	per_mod_reset;
	u32	per2_mod_reset;
	u32	brg_mod_reset;
};
#endif /* __ASSEMBLY__ */

#if defined(CONFIG_SOCFPGA_VIRTUAL_TARGET)
#define RSTMGR_CTRL_SWWARMRSTREQ_LSB 2
#else
#define RSTMGR_CTRL_SWWARMRSTREQ_LSB 1
#endif

#define RSTMGR_PERMODRST_EMAC0_LSB	0
#define RSTMGR_PERMODRST_EMAC1_LSB	1
#define RSTMGR_PERMODRST_L4WD0_LSB	6
#define RSTMGR_PERMODRST_OSC1TIMER0_LSB	8
#define RSTMGR_PERMODRST_UART0_LSB	16
#define RSTMGR_PERMODRST_SPIM0_LSB	18
#define RSTMGR_PERMODRST_SPIM1_LSB	19
#define RSTMGR_PERMODRST_SDR_LSB	29

#define RSTMGR_STAT_L4WD1RST_MASK 0x00008000
#define RSTMGR_STAT_L4WD0RST_MASK 0x00004000
#define RSTMGR_STAT_MPUWD1RST_MASK 0x00002000
#define RSTMGR_STAT_MPUWD0RST_MASK 0x00001000
#define RSTMGR_STAT_SWWARMRST_MASK 0x00000400
#define RSTMGR_STAT_FPGAWARMRST_MASK 0x00000200
#define RSTMGR_STAT_NRSTPINRST_MASK 0x00000100
#define RSTMGR_STAT_SWCOLDRST_MASK 0x00000010
#define RSTMGR_STAT_CONFIGIOCOLDRST_MASK 0x00000008
#define RSTMGR_STAT_FPGACOLDRST_MASK 0x00000004
#define RSTMGR_STAT_NPORPINRST_MASK 0x00000002
#define RSTMGR_STAT_PORVOLTRST_MASK 0x00000001

#define RSTMGR_WARMRST_MASK	(\
	RSTMGR_STAT_L4WD1RST_MASK | \
	RSTMGR_STAT_L4WD0RST_MASK | \
	RSTMGR_STAT_MPUWD1RST_MASK | \
	RSTMGR_STAT_MPUWD0RST_MASK | \
	RSTMGR_STAT_SWWARMRST_MASK | \
	RSTMGR_STAT_FPGAWARMRST_MASK | \
	RSTMGR_STAT_NRSTPINRST_MASK)
#define RSTMGR_COLDRST_MASK	(\
	RSTMGR_STAT_SWCOLDRST_MASK | \
	RSTMGR_STAT_CONFIGIOCOLDRST_MASK | \
	RSTMGR_STAT_FPGACOLDRST_MASK | \
	RSTMGR_STAT_NPORPINRST_MASK | \
	RSTMGR_STAT_PORVOLTRST_MASK)


#endif /* _RESET_MANAGER_H_ */
