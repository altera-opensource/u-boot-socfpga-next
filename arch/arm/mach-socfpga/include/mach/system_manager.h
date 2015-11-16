/*
 * Copyright (C) 2013 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	_SYSTEM_MANAGER_H_
#define	_SYSTEM_MANAGER_H_

#ifndef __ASSEMBLY__

void sysmgr_pinmux_init(void);
void sysmgr_config_warmrstcfgio(int enable);

void sysmgr_get_pinmux_table(const u8 **table, unsigned int *table_len);
#endif

#if defined(CONFIG_SOCFPGA_GEN5)
struct socfpga_system_manager {
	/* System Manager Module */
	u32	siliconid1;			/* 0x00 */
	u32	siliconid2;
	u32	_pad_0x8_0xf[2];
	u32	wddbg;				/* 0x10 */
	u32	bootinfo;
	u32	hpsinfo;
	u32	parityinj;
	/* FPGA Interface Group */
	u32	fpgaintfgrp_gbl;		/* 0x20 */
	u32	fpgaintfgrp_indiv;
	u32	fpgaintfgrp_module;
	u32	_pad_0x2c_0x2f;
	/* Scan Manager Group */
	u32	scanmgrgrp_ctrl;		/* 0x30 */
	u32	_pad_0x34_0x3f[3];
	/* Freeze Control Group */
	u32	frzctrl_vioctrl;		/* 0x40 */
	u32	_pad_0x44_0x4f[3];
	u32	frzctrl_hioctrl;		/* 0x50 */
	u32	frzctrl_src;
	u32	frzctrl_hwctrl;
	u32	_pad_0x5c_0x5f;
	/* EMAC Group */
	u32	emacgrp_ctrl;			/* 0x60 */
	u32	emacgrp_l3master;
	u32	_pad_0x68_0x6f[2];
	/* DMA Controller Group */
	u32	dmagrp_ctrl;			/* 0x70 */
	u32	dmagrp_persecurity;
	u32	_pad_0x78_0x7f[2];
	/* Preloader (initial software) Group */
	u32	iswgrp_handoff[8];		/* 0x80 */
	u32	_pad_0xa0_0xbf[8];		/* 0xa0 */
	/* Boot ROM Code Register Group */
	u32	romcodegrp_ctrl;		/* 0xc0 */
	u32	romcodegrp_cpu1startaddr;
	u32	romcodegrp_initswstate;
	u32	romcodegrp_initswlastld;
	u32	romcodegrp_bootromswstate;	/* 0xd0 */
	u32	__pad_0xd4_0xdf[3];
	/* Warm Boot from On-Chip RAM Group */
	u32	romcodegrp_warmramgrp_enable;	/* 0xe0 */
	u32	romcodegrp_warmramgrp_datastart;
	u32	romcodegrp_warmramgrp_length;
	u32	romcodegrp_warmramgrp_execution;
	u32	romcodegrp_warmramgrp_crc;	/* 0xf0 */
	u32	__pad_0xf4_0xff[3];
	/* Boot ROM Hardware Register Group */
	u32	romhwgrp_ctrl;			/* 0x100 */
	u32	_pad_0x104_0x107;
	/* SDMMC Controller Group */
	u32	sdmmcgrp_ctrl;
	u32	sdmmcgrp_l3master;
	/* NAND Flash Controller Register Group */
	u32	nandgrp_bootstrap;		/* 0x110 */
	u32	nandgrp_l3master;
	/* USB Controller Group */
	u32	usbgrp_l3master;
	u32	_pad_0x11c_0x13f[9];
	/* ECC Management Register Group */
	u32	eccgrp_l2;			/* 0x140 */
	u32	eccgrp_ocram;
	u32	eccgrp_usb0;
	u32	eccgrp_usb1;
	u32	eccgrp_emac0;			/* 0x150 */
	u32	eccgrp_emac1;
	u32	eccgrp_dma;
	u32	eccgrp_can0;
	u32	eccgrp_can1;			/* 0x160 */
	u32	eccgrp_nand;
	u32	eccgrp_qspi;
	u32	eccgrp_sdmmc;
	u32	_pad_0x170_0x3ff[164];
	/* Pin Mux Control Group */
	u32	emacio[20];			/* 0x400 */
	u32	flashio[12];			/* 0x450 */
	u32	generalio[28];			/* 0x480 */
	u32	_pad_0x4f0_0x4ff[4];
	u32	mixed1io[22];			/* 0x500 */
	u32	mixed2io[8];			/* 0x558 */
	u32	gplinmux[23];			/* 0x578 */
	u32	gplmux[71];			/* 0x5d4 */
	u32	nandusefpga;			/* 0x6f0 */
	u32	_pad_0x6f4;
	u32	rgmii1usefpga;			/* 0x6f8 */
	u32	_pad_0x6fc_0x700[2];
	u32	i2c0usefpga;			/* 0x704 */
	u32	sdmmcusefpga;			/* 0x708 */
	u32	_pad_0x70c_0x710[2];
	u32	rgmii0usefpga;			/* 0x714 */
	u32	_pad_0x718_0x720[3];
	u32	i2c3usefpga;			/* 0x724 */
	u32	i2c2usefpga;			/* 0x728 */
	u32	i2c1usefpga;			/* 0x72c */
	u32	spim1usefpga;			/* 0x730 */
	u32	_pad_0x734;
	u32	spim0usefpga;			/* 0x738 */
};
#else /* Arria10 System Manager */
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
#endif

#define SYSMGR_ROMCODEGRP_CTRL_WARMRSTCFGPINMUX	(1 << 0)
#define SYSMGR_ROMCODEGRP_CTRL_WARMRSTCFGIO	(1 << 1)
#define SYSMGR_ECC_OCRAM_EN	(1 << 0)
#define SYSMGR_ECC_OCRAM_SERR	(1 << 3)
#define SYSMGR_ECC_OCRAM_DERR	(1 << 4)
#define SYSMGR_FPGAINTF_USEFPGA	0x1
#define SYSMGR_FPGAINTF_SPIM0	(1 << 0)
#define SYSMGR_FPGAINTF_SPIM1	(1 << 1)
#define SYSMGR_FPGAINTF_EMAC0	(1 << 2)
#define SYSMGR_FPGAINTF_EMAC1	(1 << 3)
#define SYSMGR_FPGAINTF_NAND	(1 << 4)
#define SYSMGR_FPGAINTF_SDMMC	(1 << 5)

/* FIXME: This is questionable macro. */
#define SYSMGR_SDMMC_CTRL_SET(smplsel, drvsel)	\
	((((drvsel) << 0) & 0x7) | (((smplsel) << 3) & 0x38))

/* EMAC Group Bit definitions */
#define SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_GMII_MII	0x0
#define SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_RGMII		0x1
#define SYSMGR_EMACGRP_CTRL_PHYSEL_ENUM_RMII		0x2

#define SYSMGR_EMACGRP_CTRL_PHYSEL0_LSB			0
#define SYSMGR_EMACGRP_CTRL_PHYSEL1_LSB			2
#define SYSMGR_EMACGRP_CTRL_PHYSEL_MASK			0x3

/* For dedicated IO configuration */
/* Voltage select enums */
#define VOLTAGE_SEL_3V		0x0
#define VOLTAGE_SEL_1P8V	0x1
#define VOLTAGE_SEL_2P5V	0x2

/* Input buffer enable */
#define INPUT_BUF_DISABLE	0
#define INPUT_BUF_1P8V		1
#define INPUT_BUF_2P5V3V	2

/* Weak pull up enable */
#define WK_PU_DISABLE		0
#define WK_PU_ENABLE		1

/* Pull up slew rate control */
#define PU_SLW_RT_SLOW		0
#define PU_SLW_RT_FAST		1
#define PU_SLW_RT_DEFAULT	PU_SLW_RT_SLOW

/* Pull down slew rate control */
#define PD_SLW_RT_SLOW		0
#define PD_SLW_RT_FAST		1
#define PD_SLW_RT_DEFAULT	PD_SLW_RT_SLOW

/* Drive strength control */
#define PU_DRV_STRG_DEFAULT	0x10
#define PD_DRV_STRG_DEFAULT	0x10

/* bit position */
#define PD_DRV_STRG_LSB		0
#define PD_SLW_RT_LSB		5
#define PU_DRV_STRG_LSB		8
#define PU_SLW_RT_LSB		13
#define WK_PU_LSB		16
#define INPUT_BUF_LSB		17
#define BIAS_TRIM_LSB		19
#define VOLTAGE_SEL_LSB		0

#define ALT_SYSMGR_NOC_H2F_SET_MSK	0x00000001
#define ALT_SYSMGR_NOC_LWH2F_SET_MSK	0x00000010
#define ALT_SYSMGR_NOC_F2H_SET_MSK	0x00000100
#define ALT_SYSMGR_NOC_F2SDR0_SET_MSK	0x00010000
#define ALT_SYSMGR_NOC_F2SDR1_SET_MSK	0x00100000
#define ALT_SYSMGR_NOC_F2SDR2_SET_MSK	0x01000000
#define ALT_SYSMGR_NOC_TMO_EN_SET_MSK	0x00000001

#define ALT_SYSMGR_ECC_INTSTAT_SERR_OCRAM_SET_MSK	0x00000002
#define ALT_SYSMGR_ECC_INTSTAT_DERR_OCRAM_SET_MSK	0x00000002

#endif /* _SYSTEM_MANAGER_H_ */
