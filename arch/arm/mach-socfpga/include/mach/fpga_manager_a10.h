/*
 * Copyright (C) 2014 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	_SOCFPGA_FPGA_MANAGER_H_
#define	_SOCFPGA_FPGA_MANAGER_H_

#ifndef __ASSEMBLY__
int is_fpgamgr_fpga_ready(void);
int poll_fpgamgr_fpga_ready(void);
int fpgamgr_program_init(u32 * rbf_data, u32 rbf_size);
void fpgamgr_program_write(const unsigned long *rbf_data,
			   unsigned long rbf_size);
int fpgamgr_program_poll_cd(void);
int fpgamgr_program_poll_initphase(void);
int is_fpgamgr_user_mode(void);
int fpgamgr_program_poll_usermode(void);
int fpgamgr_program_poll_usermode(void);
int fpgamgr_program_fpga(const unsigned long *rbf_data, unsigned long rbf_size);
void fpgamgr_axi_write(const unsigned long *rbf_data,
		       const unsigned long fpgamgr_data_addr,
		       unsigned long rbf_size);
#endif

struct socfpga_fpga_manager {
	u32 _pad_0x0_0x7[2];
	u32 dclkcnt;
	u32 dclkstat;
	u32 gpo;
	u32 gpi;
	u32 misci;
	u32 _pad_0x1c_0x2f[5];
	u32 emr_data0;
	u32 emr_data1;
	u32 emr_data2;
	u32 emr_data3;
	u32 emr_data4;
	u32 emr_data5;
	u32 emr_valid;
	u32 emr_en;
	u32 jtag_config;
	u32 jtag_status;
	u32 jtag_kick;
	u32 _pad_0x5c_0x5f;
	u32 jtag_data_w;
	u32 jtag_data_r;
	u32 _pad_0x68_0x6f[2];
	u32 imgcfg_ctrl_00;
	u32 imgcfg_ctrl_01;
	u32 imgcfg_ctrl_02;
	u32 _pad_0x7c_0x7f;
	u32 imgcfg_stat;
	u32 intr_masked_status;
	u32 intr_mask;
	u32 intr_polarity;
	u32 dma_config;
	u32 imgcfg_fifo_status;
};

#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CRC_ERROR_SET_MSK		0x00000001
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_EARLY_USERMODE_SET_MSK	0x00000002
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_USERMODE_SET_MSK 		0x00000004
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_INITDONE_OE_SET_MSK 	0x00000008
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NSTATUS_PIN_SET_MSK		0x00000010
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NSTATUS_OE_SET_MSK		0x00000020
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CONDONE_PIN_SET_MSK		0x00000040
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CONDONE_OE_SET_MSK		0x00000080
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CVP_CONF_DONE_SET_MSK	0x00000100
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_READY_SET_MSK		0x00000200
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_DONE_SET_MSK		0x00000400
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_ERROR_SET_MSK		0x00000800
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NCONFIG_PIN_SET_MSK		0x00001000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NCEO_OE_SET_MSK		0x00002000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_SET_MSK    		0x00010000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL1_SET_MSK    		0x00020000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL2_SET_MSK    		0x00040000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL_SET_MSD (\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_SET_MSK |\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL1_SET_MSK |\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL2_SET_MSK)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_IMGCFG_FIFOEMPTY_SET_MSK	0x01000000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_IMGCFG_FIFOFULL_SET_MSK	0x02000000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_JTAGM_SET_MSK		0x10000000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_EMR_SET_MSK			0x20000000
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_LSB        16

#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NENABLE_NCONFIG_SET_MSK	0x00000001
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NENABLE_NSTATUS_SET_MSK	0x00000002
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NENABLE_CONDONE_SET_MSK	0x00000004
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NCONFIG_SET_MSK		0x00000100
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NSTATUS_OE_SET_MSK	0x00010000
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_CONDONE_OE_SET_MSK	0x01000000

#define ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NENABLE_CONFIG_SET_MSK	0x00000001
#define ALT_FPGAMGR_IMGCFG_CTL_01_S2F_PR_REQUEST_SET_MSK	0x00010000
#define ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NCE_SET_MSK		0x01000000

#define ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_CTRL_SET_MSK    	0x00000001
#define ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_DATA_SET_MSK    	0x00000100
#define ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_SET_MSK    		0x00030000
#define ALT_FPGAMGR_IMGCFG_CTL_02_CFGWIDTH_SET_MSK    		0x01000000
#define ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_LSB        16

/* Timeout counter */
#define FPGA_TIMEOUT_CNT		0x1000000

/* FPGA CD Ratio Value */
#define CDRATIO_x1	0x0
#define CDRATIO_x2	0x1
#define CDRATIO_x4	0x2
#define CDRATIO_x8	0x3

#define CFGWDTH_32 	1
#define CFGWDTH_16 	0
#endif /* _SOCFPGA_FPGA_MANAGER_H_ */
