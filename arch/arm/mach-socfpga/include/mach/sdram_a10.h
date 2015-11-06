/*
 * Copyright (C) 2014 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	_SOCFPGA_SDRAM_A10_H_
#define	_SOCFPGA_SDRAM_A10_H_

extern unsigned long irq_cnt_ecc_sdram;

#ifndef __ASSEMBLY__

void irq_handler_ecc_sdram(void *arg);
int is_external_fpga_config(const void *blob);
void sdram_enable_interrupt(unsigned enable);
void sdram_mmr_init(void);
void sdram_firewall_setup(void);
int is_sdram_cal_success(void);
int ddr_calibration_sequence(void);

struct socfpga_ecc_hmc {
	u32 ip_rev_id;
	u32 _pad_0x4_0x7;
	u32 ddrioctrl;
	u32 ddrcalstat;
	u32 mpr_0beat1;
	u32 mpr_1beat1;
	u32 mpr_2beat1;
	u32 mpr_3beat1;
	u32 mpr_4beat1;
	u32 mpr_5beat1;
	u32 mpr_6beat1;
	u32 mpr_7beat1;
	u32 mpr_8beat1;
	u32 mpr_0beat2;
	u32 mpr_1beat2;
	u32 mpr_2beat2;
	u32 mpr_3beat2;
	u32 mpr_4beat2;
	u32 mpr_5beat2;
	u32 mpr_6beat2;
	u32 mpr_7beat2;
	u32 mpr_8beat2;
	u32 _pad_0x58_0x5f[2];
	u32 auto_precharge;
	u32 _pad_0x64_0xff[39];
	u32 eccctrl;
	u32 eccctrl2;
	u32 _pad_0x108_0x10f[2];
	u32 errinten;
	u32 errintens;
	u32 errintenr;
	u32 intmode;
	u32 intstat;
	u32 diaginttest;
	u32 modstat;
	u32 derraddra;
	u32 serraddra;
	u32 _pad_0x134_0x137;
	u32 autowb_corraddr;
	u32 serrcntreg;
	u32 autowb_drop_cntreg;
	u32 _pad_0x144_0x147;
	u32 ecc_reg2wreccdatabus;
	u32 ecc_rdeccdata2regbus;
	u32 ecc_reg2rdeccdatabus;
	u32 _pad_0x154_0x15f[3];
	u32 ecc_diagon;
	u32 ecc_decstat;
	u32 _pad_0x168_0x16f[2];
	u32 ecc_errgenaddr_0;
	u32 ecc_errgenaddr_1;
	u32 ecc_errgenaddr_2;
	u32 ecc_errgenaddr_3;
};

struct socfpga_noc_ddr_scheduler {
	u32 ddr_t_main_scheduler_id_coreid;
	u32 ddr_t_main_scheduler_id_revisionid;
	u32 ddr_t_main_scheduler_ddrconf;
	u32 ddr_t_main_scheduler_ddrtiming;
	u32 ddr_t_main_scheduler_ddrmode;
	u32 ddr_t_main_scheduler_readlatency;
	u32 _pad_0x20_0x34[8];
	u32 ddr_t_main_scheduler_activate;
	u32 ddr_t_main_scheduler_devtodev;
};

/*
 * OCRAM firewall
 */
struct socfpga_noc_fw_ocram {
	u32 enable;
	u32 enable_set;
	u32 enable_clear;
	u32 region0;
	u32 region1;
	u32 region2;
	u32 region3;
	u32 region4;
	u32 region5;
};

/* for master such as MPU and FPGA */
struct socfpga_noc_fw_ddr_mpu_fpga2sdram {
	u32 enable;
	u32 enable_set;
	u32 enable_clear;
	u32 _pad_0xc_0xf;
	u32 mpuregion0addr;
	u32 mpuregion1addr;
	u32 mpuregion2addr;
	u32 mpuregion3addr;
	u32 fpga2sdram0region0addr;
	u32 fpga2sdram0region1addr;
	u32 fpga2sdram0region2addr;
	u32 fpga2sdram0region3addr;
	u32 fpga2sdram1region0addr;
	u32 fpga2sdram1region1addr;
	u32 fpga2sdram1region2addr;
	u32 fpga2sdram1region3addr;
	u32 fpga2sdram2region0addr;
	u32 fpga2sdram2region1addr;
	u32 fpga2sdram2region2addr;
	u32 fpga2sdram2region3addr;
};

/* for L3 master */
struct socfpga_noc_fw_ddr_l3 {
	u32 enable;
	u32 enable_set;
	u32 enable_clear;
	u32 hpsregion0addr;
	u32 hpsregion1addr;
	u32 hpsregion2addr;
	u32 hpsregion3addr;
	u32 hpsregion4addr;
	u32 hpsregion5addr;
	u32 hpsregion6addr;
	u32 hpsregion7addr;
};

struct socfpga_io48_mmr {
	u32 dbgcfg0;
	u32 dbgcfg1;
	u32 dbgcfg2;
	u32 dbgcfg3;
	u32 dbgcfg4;
	u32 dbgcfg5;
	u32 dbgcfg6;
	u32 reserve0;
	u32 reserve1;
	u32 reserve2;
	u32 ctrlcfg0;
	u32 ctrlcfg1;
	u32 ctrlcfg2;
	u32 ctrlcfg3;
	u32 ctrlcfg4;
	u32 ctrlcfg5;
	u32 ctrlcfg6;
	u32 ctrlcfg7;
	u32 ctrlcfg8;
	u32 ctrlcfg9;
	u32 dramtiming0;
	u32 dramodt0;
	u32 dramodt1;
	u32 sbcfg0;
	u32 sbcfg1;
	u32 sbcfg2;
	u32 sbcfg3;
	u32 sbcfg4;
	u32 sbcfg5;
	u32 sbcfg6;
	u32 sbcfg7;
	u32 caltiming0;
	u32 caltiming1;
	u32 caltiming2;
	u32 caltiming3;
	u32 caltiming4;
	u32 caltiming5;
	u32 caltiming6;
	u32 caltiming7;
	u32 caltiming8;
	u32 caltiming9;
	u32 caltiming10;
	u32 dramaddrw;
	u32 sideband0;
	u32 sideband1;
	u32 sideband2;
	u32 sideband3;
	u32 sideband4;
	u32 sideband5;
	u32 sideband6;
	u32 sideband7;
	u32 sideband8;
	u32 sideband9;
	u32 sideband10;
	u32 sideband11;
	u32 sideband12;
	u32 sideband13;
	u32 sideband14;
	u32 sideband15;
	u32 dramsts;
	u32 dbgdone;
	u32 dbgsignals;
	u32 dbgreset;
	u32 dbgmatch;
	u32 counter0mask;
	u32 counter1mask;
	u32 counter0match;
	u32 counter1match;
	u32 niosreserve0;
	u32 niosreserve1;
	u32 niosreserve2;
};

union dramaddrw_reg {
	struct {
		u32 cfg_col_addr_width:5;
		u32 cfg_row_addr_width:5;
		u32 cfg_bank_addr_width:4;
		u32 cfg_bank_group_addr_width:2;
		u32 cfg_cs_addr_width:3;
		u32 reserved:13;
	};
	u32 word;
};

union ctrlcfg0_reg {
	struct {
		u32 cfg_mem_type:4;
		u32 cfg_dimm_type:3;
		u32 cfg_ac_pos:2;
		u32 cfg_ctrl_burst_len:5;
		u32 reserved:18;	/* Other fields unused */
	};
	u32 word;
};

union ctrlcfg1_reg {
	struct {
		u32 cfg_dbc3_burst_len:5;
		u32 cfg_addr_order:2;
		u32 cfg_ctrl_enable_ecc:1;
		u32 reserved:24;	/* Other fields unused */
	};
	u32 word;
};

union caltiming0_reg {
	struct {
		u32 cfg_act_to_rdwr:6;
		u32 cfg_act_to_pch:6;
		u32 cfg_act_to_act:6;
		u32 cfg_act_to_act_db:6;
		u32 reserved:8;	/* Other fields unused */
	};
	u32 word;
};

union caltiming1_reg {
	struct {
		u32 cfg_rd_to_rd:6;
		u32 cfg_rd_to_rd_dc:6;
		u32 cfg_rd_to_rd_db:6;
		u32 cfg_rd_to_wr:6;
		u32 cfg_rd_to_wr_dc:6;
		u32 reserved:2;
	};
	u32 word;
};

union caltiming2_reg {
	struct {
		u32 cfg_rd_to_wr_db:6;
		u32 cfg_rd_to_pch:6;
		u32 cfg_rd_ap_to_valid:6;
		u32 cfg_wr_to_wr:6;
		u32 cfg_wr_to_wr_dc:6;
		u32 reserved:2;
	};
	u32 word;
};

union caltiming3_reg {
	struct {
		u32 cfg_wr_to_wr_db:6;
		u32 cfg_wr_to_rd:6;
		u32 cfg_wr_to_rd_dc:6;
		u32 cfg_wr_to_rd_db:6;
		u32 cfg_wr_to_pch:6;
		u32 reserved:2;
	};
	u32 word;
};

union caltiming4_reg {
	struct {
		u32 cfg_wr_ap_to_valid:6;
		u32 cfg_pch_to_valid:6;
		u32 cfg_pch_all_to_valid:6;
		u32 cfg_arf_to_valid:8;
		u32 cfg_pdn_to_valid:6;
	};
	u32 word;
};

union caltiming9_reg {
	struct {
		u32 cfg_4_act_to_act:8;
		u32 reserved:24;
	};
	u32 word;
};

#endif /* __ASSEMBLY__ */

#define ALT_ECC_HMC_OCP_DDRIOCTRL_IO_SIZE_MSK		0x00000003

#define ALT_ECC_HMC_OCP_INTSTAT_SERRPENA_SET_MSK	0x00000001
#define ALT_ECC_HMC_OCP_INTSTAT_DERRPENA_SET_MSK	0x00000002
#define ALT_ECC_HMC_OCP_ERRINTEN_SERRINTEN_SET_MSK	0x00000001
#define ALT_ECC_HMC_OCP_ERRINTEN_DERRINTEN_SET_MSK	0x00000002
#define ALT_ECC_HMC_OCP_INTMOD_INTONCMP_SET_MSK		0x00010000
#define ALT_ECC_HMC_OCP_ECCCTL_AWB_CNT_RST_SET_MSK	0x00010000
#define ALT_ECC_HMC_OCP_ECCCTL_CNT_RST_SET_MSK		0x00000100
#define ALT_ECC_HMC_OCP_ECCCTL_ECC_EN_SET_MSK		0x00000001
#define ALT_ECC_HMC_OCP_ECCCTL2_RMW_EN_SET_MSK		0x00000100
#define ALT_ECC_HMC_OCP_ECCCTL2_AWB_EN_SET_MSK		0x00000001

#define ALT_ECC_HMC_OCP_SERRCNTREG_VALUE		8

#define ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_ACTTOACT_LSB	0
#define ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_RDTOMISS_LSB	6
#define ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_WRTOMISS_LSB	12
#define ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_BURSTLEN_LSB	18
#define ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_RDTOWR_LSB	21
#define ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_WRTORD_LSB	26
#define ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_BWRATIO_LSB	31

#define ALT_NOC_MPU_DDR_T_SCHED_DDRMOD_AUTOPRECHARGE_LSB	0
#define ALT_NOC_MPU_DDR_T_SCHED_DDRMOD_BWRATIOEXTENDED_LSB	1

#define ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_RRD_LSB	0
#define ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_FAW_LSB	4
#define ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_FAWBANK_LSB	10

#define ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSRDTORD_LSB	0
#define ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSRDTOWR_LSB	2
#define ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSWRTORD_LSB	4

#define ALT_NOC_FW_DDR_END_ADDR_LSB	16
#define ALT_NOC_FW_DDR_ADDR_MASK	0xFFFF
#define ALT_NOC_FW_DDR_SCR_EN_HPSREG0EN_SET_MSK		0x00000001
#define ALT_NOC_FW_DDR_SCR_EN_HPSREG1EN_SET_MSK		0x00000002
#define ALT_NOC_FW_DDR_SCR_EN_HPSREG2EN_SET_MSK		0x00000004
#define ALT_NOC_FW_DDR_SCR_EN_HPSREG3EN_SET_MSK		0x00000008
#define ALT_NOC_FW_DDR_SCR_EN_HPSREG4EN_SET_MSK		0x00000010
#define ALT_NOC_FW_DDR_SCR_EN_HPSREG5EN_SET_MSK		0x00000020
#define ALT_NOC_FW_DDR_SCR_EN_HPSREG6EN_SET_MSK		0x00000040
#define ALT_NOC_FW_DDR_SCR_EN_HPSREG7EN_SET_MSK		0x00000080
#define ALT_NOC_FW_DDR_SCR_EN_MPUREG0EN_SET_MSK		0x00000001
#define ALT_NOC_FW_DDR_SCR_EN_MPUREG1EN_SET_MSK		0x00000002
#define ALT_NOC_FW_DDR_SCR_EN_MPUREG2EN_SET_MSK		0x00000004
#define ALT_NOC_FW_DDR_SCR_EN_MPUREG3EN_SET_MSK		0x00000008
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR0REG0EN_SET_MSK	0x00000010
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR0REG1EN_SET_MSK	0x00000020
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR0REG2EN_SET_MSK	0x00000040
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR0REG3EN_SET_MSK	0x00000080
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR1REG0EN_SET_MSK	0x00000100
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR1REG1EN_SET_MSK	0x00000200
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR1REG2EN_SET_MSK	0x00000400
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR1REG3EN_SET_MSK	0x00000800
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR2REG0EN_SET_MSK	0x00001000
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR2REG1EN_SET_MSK	0x00002000
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR2REG2EN_SET_MSK	0x00004000
#define ALT_NOC_FW_DDR_SCR_EN_F2SDR2REG3EN_SET_MSK	0x00008000

#define ALT_IO48_DRAMTIME_MEM_READ_LATENCY_MASK		0x0000003F
#endif /* _SOCFPGA_SDRAM_A10_H_ */
