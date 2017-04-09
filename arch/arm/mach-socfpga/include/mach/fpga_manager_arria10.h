/*
 * Copyright (C) 2017 Intel Corporation <www.intel.com>
 * All rights reserved.
 *
 * SPDX-License-Identifier:    BSD-3-Clause
 */

#ifndef _FPGA_MANAGER_ARRIA10_H_
#define _FPGA_MANAGER_ARRIA10_H_

#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CRC_ERROR_SET_MSK		BIT(0)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_EARLY_USERMODE_SET_MSK	BIT(1)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_USERMODE_SET_MSK 		BIT(2)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_INITDONE_OE_SET_MSK 	BIT(3)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NSTATUS_PIN_SET_MSK		BIT(4)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NSTATUS_OE_SET_MSK		BIT(5)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CONDONE_PIN_SET_MSK		BIT(6)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CONDONE_OE_SET_MSK		BIT(7)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_CVP_CONF_DONE_SET_MSK	BIT(8)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_READY_SET_MSK		BIT(9)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_DONE_SET_MSK		BIT(10)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_ERROR_SET_MSK		BIT(11)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NCONFIG_PIN_SET_MSK		BIT(12)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_NCEO_OE_SET_MSK		BIT(13)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_SET_MSK    		BIT(16)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL1_SET_MSK    		BIT(17)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL2_SET_MSK    		BIT(18)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL_SET_MSD (\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_SET_MSK |\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL1_SET_MSK |\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL2_SET_MSK)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_IMGCFG_FIFOEMPTY_SET_MSK	BIT(24)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_IMGCFG_FIFOFULL_SET_MSK	BIT(25)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_JTAGM_SET_MSK		BIT(28)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_EMR_SET_MSK			BIT(29)
#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_LSB			16

#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NENABLE_NCONFIG_SET_MSK	BIT(0)
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NENABLE_NSTATUS_SET_MSK	BIT(1)
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NENABLE_CONDONE_SET_MSK	BIT(2)
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NCONFIG_SET_MSK		BIT(8)
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NSTATUS_OE_SET_MSK	BIT(16)
#define ALT_FPGAMGR_IMGCFG_CTL_00_S2F_CONDONE_OE_SET_MSK	BIT(24)

#define ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NENABLE_CONFIG_SET_MSK	BIT(0)
#define ALT_FPGAMGR_IMGCFG_CTL_01_S2F_PR_REQUEST_SET_MSK	BIT(16)
#define ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NCE_SET_MSK		BIT(24)

#define ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_CTRL_SET_MSK    	BIT(0)
#define ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_DATA_SET_MSK    	BIT(8)
#define ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_SET_MSK    		0x00030000
#define ALT_FPGAMGR_IMGCFG_CTL_02_CFGWIDTH_SET_MSK		BIT(24)
#define ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_LSB			16

/* Timeout counter */
#define FPGA_TIMEOUT_CNT	0x1000000
#define FPGA_TIMEOUT_MSEC	1000  /* timeout in ms */

#ifndef __ASSEMBLY__

struct socfpga_fpga_manager {
	uint32_t  _pad_0x0_0x7[2];
	uint32_t  dclkcnt;
	uint32_t  dclkstat;
	uint32_t  gpo;
	uint32_t  gpi;
	uint32_t  misci;
	uint32_t  _pad_0x1c_0x2f[5];
	uint32_t  emr_data0;
	uint32_t  emr_data1;
	uint32_t  emr_data2;
	uint32_t  emr_data3;
	uint32_t  emr_data4;
	uint32_t  emr_data5;
	uint32_t  emr_valid;
	uint32_t  emr_en;
	uint32_t  jtag_config;
	uint32_t  jtag_status;
	uint32_t  jtag_kick;
	uint32_t  _pad_0x5c_0x5f;
	uint32_t  jtag_data_w;
	uint32_t  jtag_data_r;
	uint32_t  _pad_0x68_0x6f[2];
	uint32_t  imgcfg_ctrl_00;
	uint32_t  imgcfg_ctrl_01;
	uint32_t  imgcfg_ctrl_02;
	uint32_t  _pad_0x7c_0x7f;
	uint32_t  imgcfg_stat;
	uint32_t  intr_masked_status;
	uint32_t  intr_mask;
	uint32_t  intr_polarity;
	uint32_t  dma_config;
	uint32_t  imgcfg_fifo_status;
};

/* Functions */
int is_fpgamgr_fpga_ready(void);
int poll_fpgamgr_fpga_ready(void);
int fpgamgr_program_init(u32 * rbf_data, u32 rbf_size);
int fpgamgr_program_fini(void);
void fpgamgr_program_write(const unsigned long *rbf_data,
	unsigned long rbf_size);
void fpgamgr_program_sync(void);
int fpgamgr_program_poll_cd(void);
int fpgamgr_program_poll_initphase(void);
int is_fpgamgr_user_mode(void);
int fpgamgr_program_poll_usermode(void);
int fpgamgr_program_poll_usermode(void);
int fpgamgr_program_fpga(const unsigned long *rbf_data,
	unsigned long rbf_size);
void fpgamgr_axi_write(const unsigned long *rbf_data,
	const unsigned long fpgamgr_data_addr, unsigned long rbf_size);
int fpgamgr_wait_early_user_mode(void);
int is_fpgamgr_early_user_mode(void);
int fpgamgr_reset(void);
int wait_for_nconfig_pin_and_nstatus_pin(void);

#endif /* __ASSEMBLY__ */

#endif /* _FPGA_MANAGER_ARRIA10_H_ */
