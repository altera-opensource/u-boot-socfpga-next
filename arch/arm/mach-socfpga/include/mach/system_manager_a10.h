/*
 * Copyright (C) 2014 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	_SOCFPGA_SYSTEM_MANAGER_A10_H_
#define	_SOCFPGA_SYSTEM_MANAGER_A10_H_

#ifndef __ASSEMBLY__
struct socfpga_system_manager {
	u32  siliconid1;
	u32  siliconid2;
	u32  wddbg;
	u32  bootinfo;
	u32  mpu_ctrl_l2_ecc;
	u32  _pad_0x14_0x1f[3];
	u32  dma;
	u32  dma_periph;
	u32  sdmmcgrp_ctrl;
	u32  sdmmc_l3master;
	u32  nand_bootstrap;
	u32  nand_l3master;
	u32  usb0_l3master;
	u32  usb1_l3master;
	u32  emac_global;
	u32  emac0;
	u32  emac1;
	u32  emac2;
	u32  _pad_0x50_0x5f[4];
	u32  fpgaintf_en_global;
	u32  fpgaintf_en_0;
	u32  fpgaintf_en_1;
	u32  fpgaintf_en_2;
	u32  fpgaintf_en_3;
	u32  _pad_0x74_0x7f[3];
	u32  noc_addr_remap_value;
	u32  noc_addr_remap_set;
	u32  noc_addr_remap_clear;
	u32  _pad_0x8c_0x8f;
	u32  ecc_intmask_value;
	u32  ecc_intmask_set;
	u32  ecc_intmask_clr;
	u32  ecc_intstatus_serr;
	u32  ecc_intstatus_derr;
	u32  mpu_status_l2_ecc;
	u32  mpu_clear_l2_ecc;
	u32  mpu_status_l1_parity;
	u32  mpu_clear_l1_parity;
	u32  mpu_set_l1_parity;
	u32  _pad_0xb8_0xbf[2];
	u32  noc_timeout;
	u32  noc_idlereq_set;
	u32  noc_idlereq_clr;
	u32  noc_idlereq_value;
	u32  noc_idleack;
	u32  noc_idlestatus;
	u32  fpga2soc_ctrl;
	u32  _pad_0xdc_0xff[9];
	u32  tsmc_tsel_0;
	u32  tsmc_tsel_1;
	u32  tsmc_tsel_2;
	u32  tsmc_tsel_3;
	u32  _pad_0x110_0x200[60];
	u32  romhw_ctrl;
	u32  romcode_ctrl;
	u32  romcode_cpu1startaddr;
	u32  romcode_initswstate;
	u32  romcode_initswlastld;
	u32  _pad_0x214_0x217;
	u32  warmram_enable;
	u32  warmram_datastart;
	u32  warmram_length;
	u32  warmram_execution;
	u32  warmram_crc;
	u32  _pad_0x22c_0x22f;
	u32  isw_handoff[8];
	u32  romcode_bootromswstate[8];
};
#endif /* __ASSEMBLY__ */

/* FPGA interface group */
#define SYSMGR_FPGAINTF_MODULE		(SOCFPGA_SYSMGR_ADDRESS + 0x28)
/* EMAC interface selection */
#define CONFIG_SYSMGR_EMAC0_CTRL	(SOCFPGA_SYSMGR_ADDRESS + 0x44)
#define CONFIG_SYSMGR_EMAC1_CTRL	(SOCFPGA_SYSMGR_ADDRESS + 0x48)
#define CONFIG_SYSMGR_EMAC2_CTRL	(SOCFPGA_SYSMGR_ADDRESS + 0x4c)

/* Preloader handoff to bootloader register */
#define SYSMGR_ISWGRP_HANDOFF0		(SOCFPGA_SYSMGR_ADDRESS + 0x80)
#define SYSMGR_ISWGRP_HANDOFF1		(SOCFPGA_SYSMGR_ADDRESS + 0x84)
#define SYSMGR_ISWGRP_HANDOFF2		(SOCFPGA_SYSMGR_ADDRESS + 0x88)
#define SYSMGR_ISWGRP_HANDOFF3		(SOCFPGA_SYSMGR_ADDRESS + 0x8C)
#define SYSMGR_ISWGRP_HANDOFF4		(SOCFPGA_SYSMGR_ADDRESS + 0x90)
#define SYSMGR_ISWGRP_HANDOFF5		(SOCFPGA_SYSMGR_ADDRESS + 0x94)
#define SYSMGR_ISWGRP_HANDOFF6		(SOCFPGA_SYSMGR_ADDRESS + 0x98)
#define SYSMGR_ISWGRP_HANDOFF7		(SOCFPGA_SYSMGR_ADDRESS + 0x9C)

#define ISWGRP_HANDOFF_AXIBRIDGE	SYSMGR_ISWGRP_HANDOFF0
#define ISWGRP_HANDOFF_L3REMAP		SYSMGR_ISWGRP_HANDOFF1
#define ISWGRP_HANDOFF_FPGAINTF		SYSMGR_ISWGRP_HANDOFF2
#define ISWGRP_HANDOFF_FPGA2SDR		SYSMGR_ISWGRP_HANDOFF3

/* pin mux */
#define SYSMGR_PINMUXGRP		(SOCFPGA_SYSMGR_ADDRESS + 0x400)
#define SYSMGR_PINMUXGRP_NANDUSEFPGA	(SYSMGR_PINMUXGRP + 0x2F0)
#define SYSMGR_PINMUXGRP_EMAC1USEFPGA	(SYSMGR_PINMUXGRP + 0x2F8)
#define SYSMGR_PINMUXGRP_SDMMCUSEFPGA	(SYSMGR_PINMUXGRP + 0x308)
#define SYSMGR_PINMUXGRP_EMAC0USEFPGA	(SYSMGR_PINMUXGRP + 0x314)
#define SYSMGR_PINMUXGRP_SPIM1USEFPGA	(SYSMGR_PINMUXGRP + 0x330)
#define SYSMGR_PINMUXGRP_SPIM0USEFPGA	(SYSMGR_PINMUXGRP + 0x338)

/* bit fields */
#define SYSMGR_ROMCODEGRP_CTRL_WARMRSTCFGPINMUX		(1<<0)
#define SYSMGR_ROMCODEGRP_CTRL_WARMRSTCFGIO		(1<<1)
#define SYSMGR_ECC_OCRAM_EN		(1<<0)
#define SYSMGR_ECC_OCRAM_SERR		(1<<3)
#define SYSMGR_ECC_OCRAM_DERR		(1<<4)
#define SYSMGR_FPGAINTF_USEFPGA		0x1
#define SYSMGR_FPGAINTF_SPIM0		(1<<0)
#define SYSMGR_FPGAINTF_SPIM1		(1<<1)
#define SYSMGR_FPGAINTF_EMAC0		(1<<2)
#define SYSMGR_FPGAINTF_EMAC1		(1<<3)
#define SYSMGR_FPGAINTF_NAND		(1<<4)
#define SYSMGR_FPGAINTF_SDMMC		(1<<5)

/* Enumeration: sysmgr::emacgrp::ctrl::physel::enum                        */
#define SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_GMII_MII 0x0
#define SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_RGMII 0x1
#define SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_RMII 0x2
#define SYSMGR_EMACGRP_CTRL_PHYSEL0_LSB 0
#define SYSMGR_EMACGRP_CTRL_PHYSEL1_LSB 2
#define SYSMGR_EMACGRP_CTRL_PHYSEL_MASK 0x00000003

/* For dedicated IO configuration */
/* Voltage select enums */
#define VOLTAGE_SEL_3V		0x0
#define VOLTAGE_SEL_1P8V	0x1
#define VOLTAGE_SEL_2P5V	0x2

/* Input buffer enable */
#define INPUT_BUF_DISABLE	(0)
#define INPUT_BUF_1P8V		(1)
#define INPUT_BUF_2P5V3V	(2)

/* Weak pull up enable */
#define WK_PU_DISABLE		(0)
#define WK_PU_ENABLE		(1)

/* Pull up slew rate control */
#define PU_SLW_RT_SLOW		(0)
#define PU_SLW_RT_FAST		(1)
#define PU_SLW_RT_DEFAULT	PU_SLW_RT_SLOW

/* Pull down slew rate control */
#define PD_SLW_RT_SLOW		(0)
#define PD_SLW_RT_FAST		(1)
#define PD_SLW_RT_DEFAULT	PD_SLW_RT_SLOW

/* Drive strength control */
#define PU_DRV_STRG_DEFAULT	(0x10)
#define PD_DRV_STRG_DEFAULT	(0x10)

/* bit position */
#define PD_DRV_STRG_LSB		(0)
#define PD_SLW_RT_LSB		(5)
#define PU_DRV_STRG_LSB		(8)
#define PU_SLW_RT_LSB		(13)
#define WK_PU_LSB		(16)
#define INPUT_BUF_LSB		(17)
#define BIAS_TRIM_LSB		(19)
#define VOLTAGE_SEL_LSB		(0)

#define ALT_SYSMGR_NOC_H2F_SET_MSK	0x00000001
#define ALT_SYSMGR_NOC_LWH2F_SET_MSK	0x00000010
#define ALT_SYSMGR_NOC_F2H_SET_MSK	0x00000100
#define ALT_SYSMGR_NOC_F2SDR0_SET_MSK	0x00010000
#define ALT_SYSMGR_NOC_F2SDR1_SET_MSK	0x00100000
#define ALT_SYSMGR_NOC_F2SDR2_SET_MSK	0x01000000
#define ALT_SYSMGR_NOC_TMO_EN_SET_MSK	0x00000001

#define ALT_SYSMGR_ECC_INTSTAT_SERR_OCRAM_SET_MSK    0x00000002
#define ALT_SYSMGR_ECC_INTSTAT_DERR_OCRAM_SET_MSK    0x00000002

#define SYSMGR_A10_SDMMC_CTRL_SET(smplsel, drvsel)	\
	((drvsel << 0) & 0x7) | ((smplsel << 4) & 0x70)

#endif /* _SOCFPGA_SYSTEM_MANAGER_A10_H_ */
