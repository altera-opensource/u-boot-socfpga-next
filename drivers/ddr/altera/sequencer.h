/*
 * Copyright Altera Corporation (C) 2012-2014. All rights reserved
 *
 * SPDX-License-Identifier:    BSD-3-Clause
 */

#ifndef _SEQUENCER_H_
#define _SEQUENCER_H_

extern int32_t inst_rom_init_size;
extern uint32_t inst_rom_init[];
extern uint32_t ac_rom_init_size;
extern uint32_t ac_rom_init[];

#if RLDRAMII
#define RW_MGR_NUM_DM_PER_WRITE_GROUP (1)
#define RW_MGR_NUM_TRUE_DM_PER_WRITE_GROUP (1)
#else
#define RW_MGR_NUM_DM_PER_WRITE_GROUP (RW_MGR_MEM_DATA_MASK_WIDTH \
	/ RW_MGR_MEM_IF_WRITE_DQS_WIDTH)
#define RW_MGR_NUM_TRUE_DM_PER_WRITE_GROUP (RW_MGR_TRUE_MEM_DATA_MASK_WIDTH \
	/ RW_MGR_MEM_IF_WRITE_DQS_WIDTH)
#endif

#define RW_MGR_NUM_DQS_PER_WRITE_GROUP (RW_MGR_MEM_IF_READ_DQS_WIDTH \
	/ RW_MGR_MEM_IF_WRITE_DQS_WIDTH)
#define NUM_RANKS_PER_SHADOW_REG (RW_MGR_MEM_NUMBER_OF_RANKS / NUM_SHADOW_REGS)

#define RW_MGR_RUN_SINGLE_GROUP	(BASE_RW_MGR)
#define RW_MGR_RUN_ALL_GROUPS	(BASE_RW_MGR + 0x0400)

#define RW_MGR_DI_BASE		(BASE_RW_MGR + 0x0020)

#if DDR3
#define DDR3_MR1_ODT_MASK	0xFFFFFD99
#define DDR3_MR2_ODT_MASK	0xFFFFF9FF
#define DDR3_AC_MIRR_MASK	0x020A8
#endif /* DDR3 */

#define RW_MGR_MEM_NUMBER_OF_RANKS	1
#define NUM_SHADOW_REGS			1

struct socfpga_sdr_rw_load_manager {
	u32	load_cntr0;
	u32	load_cntr1;
	u32	load_cntr2;
	u32	load_cntr3;
};

struct socfpga_sdr_rw_load_jump_manager {
	u32	load_jump_add0;
	u32	load_jump_add1;
	u32	load_jump_add2;
	u32	load_jump_add3;
};

#define RW_MGR_RESET_READ_DATAPATH	(BASE_RW_MGR + 0x1000)
#define RW_MGR_SET_CS_AND_ODT_MASK	(BASE_RW_MGR + 0x1400)

#define RW_MGR_RANK_NONE		0xFF
#define RW_MGR_RANK_ALL			0x00

#define RW_MGR_ODT_MODE_OFF		0
#define RW_MGR_ODT_MODE_READ_WRITE	1

#define NUM_CALIB_REPEAT		1

#define NUM_READ_TESTS			7
#define NUM_READ_PB_TESTS		7
#define NUM_WRITE_TESTS			15
#define NUM_WRITE_PB_TESTS		31

#define PASS_ALL_BITS			1
#define PASS_ONE_BIT			0

/* calibration stages */

#define CAL_STAGE_NIL			0
#define CAL_STAGE_VFIFO			1
#define CAL_STAGE_WLEVEL		2
#define CAL_STAGE_LFIFO			3
#define CAL_STAGE_WRITES		4
#define CAL_STAGE_FULLTEST		5
#define CAL_STAGE_REFRESH		6
#define CAL_STAGE_CAL_SKIPPED		7
#define CAL_STAGE_CAL_ABORTED		8
#define CAL_STAGE_VFIFO_AFTER_WRITES	9

/* calibration substages */

#define CAL_SUBSTAGE_NIL		0
#define CAL_SUBSTAGE_GUARANTEED_READ	1
#define CAL_SUBSTAGE_DQS_EN_PHASE	2
#define CAL_SUBSTAGE_VFIFO_CENTER	3
#define CAL_SUBSTAGE_WORKING_DELAY	1
#define CAL_SUBSTAGE_LAST_WORKING_DELAY	2
#define CAL_SUBSTAGE_WLEVEL_COPY	3
#define CAL_SUBSTAGE_WRITES_CENTER	1
#define CAL_SUBSTAGE_READ_LATENCY	1
#define CAL_SUBSTAGE_REFRESH		1

#define MAX_RANKS			(RW_MGR_MEM_NUMBER_OF_RANKS)
#define MAX_DQS				(RW_MGR_MEM_IF_WRITE_DQS_WIDTH > \
					RW_MGR_MEM_IF_READ_DQS_WIDTH ? \
					RW_MGR_MEM_IF_WRITE_DQS_WIDTH : \
					RW_MGR_MEM_IF_READ_DQS_WIDTH)
#define MAX_DQ				(RW_MGR_MEM_DATA_WIDTH)
#define MAX_DM				(RW_MGR_MEM_DATA_MASK_WIDTH)

/* length of VFIFO, from SW_MACROS */
#define VFIFO_SIZE			(READ_VALID_FIFO_SIZE)

/* Memory for data transfer between TCL scripts and NIOS.
 *
 * - First word is a command request.
 * - The remaining words are part of the transfer.
 */

/* Define the base address of each manager. */

/* MarkW: how should these base addresses be done for A-V? */
#define BASE_PTR_MGR			(0x00040000)
#define BASE_SCC_MGR			(0x00058000)
#define BASE_REG_FILE			(0x00070000)
#define BASE_TIMER			(0x00078000)
#define BASE_PHY_MGR			(0x00088000)
#define BASE_RW_MGR			(0x00090000)
#define BASE_DATA_MGR			(0x00098000)
#define BASE_MMR                        (0x000C0000)
#define BASE_TRK_MGR			(0x000D0000)

/* Register file addresses. */

struct socfpga_sdr_reg_file {
	u32 signature;
	u32 debug_data_addr;
	u32 cur_stage;
	u32 fom;
	u32 failing_stage;
	u32 debug1;
	u32 debug2;
	u32 dtaps_per_ptap;
	u32 trk_sample_count;
	u32 trk_longidle;
	u32 delays;
	u32 trk_rw_mgr_addr;
	u32 trk_read_dqs_width;
	u32 trk_rfsh;
};

/* PHY manager configuration registers. */

#define PHY_MGR_PHY_RLAT		(BASE_PHY_MGR + 0x4000)
#define PHY_MGR_RESET_MEM_STBL		(BASE_PHY_MGR + 0x4004)
#define PHY_MGR_MUX_SEL			(BASE_PHY_MGR + 0x4008)
#define PHY_MGR_CAL_STATUS		(BASE_PHY_MGR + 0x400c)
#define PHY_MGR_CAL_DEBUG_INFO		(BASE_PHY_MGR + 0x4010)
#define PHY_MGR_VFIFO_RD_EN_OVRD	(BASE_PHY_MGR + 0x4014)
#define PHY_MGR_AFI_WLAT		(BASE_PHY_MGR + 0x4018)
#define PHY_MGR_AFI_RLAT		(BASE_PHY_MGR + 0x401c)

#define PHY_MGR_CAL_RESET		(0)
#define PHY_MGR_CAL_SUCCESS		(1)
#define PHY_MGR_CAL_FAIL		(2)

/* PHY manager command addresses. */

#define PHY_MGR_CMD_INC_VFIFO_FR	(BASE_PHY_MGR + 0x0000)
#define PHY_MGR_CMD_INC_VFIFO_HR	(BASE_PHY_MGR + 0x0004)
#define PHY_MGR_CMD_INC_VFIFO_HARD_PHY	(BASE_PHY_MGR + 0x0004)
#define PHY_MGR_CMD_FIFO_RESET		(BASE_PHY_MGR + 0x0008)
#define PHY_MGR_CMD_INC_VFIFO_FR_HR	(BASE_PHY_MGR + 0x000C)
#define PHY_MGR_CMD_INC_VFIFO_QR	(BASE_PHY_MGR + 0x0010)

#define DATA_MGR_MEM_T_WL		(BASE_DATA_MGR + 0x0004)
#define DATA_MGR_MEM_T_ADD		(BASE_DATA_MGR + 0x0008)
#define DATA_MGR_MEM_T_RL		(BASE_DATA_MGR + 0x000C)

#define MEM_T_WL_ADD			DATA_MGR_MEM_T_WL
#define MEM_T_RL_ADD			DATA_MGR_MEM_T_RL

#define CALIB_SKIP_DELAY_LOOPS		(1 << 0)
#define CALIB_SKIP_ALL_BITS_CHK		(1 << 1)
#define CALIB_SKIP_DELAY_SWEEPS		(1 << 2)
#define CALIB_SKIP_VFIFO		(1 << 3)
#define CALIB_SKIP_LFIFO		(1 << 4)
#define CALIB_SKIP_WLEVEL		(1 << 5)
#define CALIB_SKIP_WRITES		(1 << 6)
#define CALIB_SKIP_FULL_TEST		(1 << 7)
#define CALIB_SKIP_ALL			(CALIB_SKIP_VFIFO | \
				CALIB_SKIP_LFIFO | CALIB_SKIP_WLEVEL | \
				CALIB_SKIP_WRITES | CALIB_SKIP_FULL_TEST)
#define CALIB_IN_RTL_SIM			(1 << 8)

/* Scan chain manager command addresses */

#define WRITE_SCC_OCT_OUT2_DELAY(group, delay)
#define WRITE_SCC_DQS_BYPASS(group, bypass)

#define WRITE_SCC_DQ_OUT2_DELAY(pin, delay)

#define WRITE_SCC_DQ_BYPASS(pin, bypass)

#define WRITE_SCC_RFIFO_MODE(pin, mode)

#define WRITE_SCC_DQS_IO_OUT2_DELAY(delay)

#define WRITE_SCC_DM_IO_OUT2_DELAY(pin, delay)

#define WRITE_SCC_DM_BYPASS(pin, bypass)

#define READ_SCC_OCT_OUT2_DELAY(group)	0
#define READ_SCC_DQS_BYPASS(group)		0
#define READ_SCC_DQS_BYPASS(group)		0

#define READ_SCC_DQ_OUT2_DELAY(pin)		0
#define READ_SCC_DQ_BYPASS(pin)			0
#define READ_SCC_RFIFO_MODE(pin)		0

#define READ_SCC_DQS_IO_OUT2_DELAY()	0

#define READ_SCC_DM_IO_OUT2_DELAY(pin)	0
#define READ_SCC_DM_BYPASS(pin)		0

#define SCC_MGR_GROUP_COUNTER			(BASE_SCC_MGR + 0x0000)
#define SCC_MGR_DQS_IN_DELAY			(BASE_SCC_MGR + 0x0100)
#define SCC_MGR_DQS_EN_PHASE			(BASE_SCC_MGR + 0x0200)
#define SCC_MGR_DQS_EN_DELAY			(BASE_SCC_MGR + 0x0300)
#define SCC_MGR_DQDQS_OUT_PHASE			(BASE_SCC_MGR + 0x0400)
#define SCC_MGR_OCT_OUT1_DELAY			(BASE_SCC_MGR + 0x0500)
#define SCC_MGR_IO_OUT1_DELAY			(BASE_SCC_MGR + 0x0700)
#define SCC_MGR_IO_IN_DELAY			(BASE_SCC_MGR + 0x0900)

/* HHP-HPS-specific versions of some commands */
#define SCC_MGR_DQS_EN_DELAY_GATE		(BASE_SCC_MGR + 0x0600)
#define SCC_MGR_IO_OE_DELAY			(BASE_SCC_MGR + 0x0800)
#define SCC_MGR_HHP_GLOBALS			(BASE_SCC_MGR + 0x0A00)
#define SCC_MGR_HHP_RFILE			(BASE_SCC_MGR + 0x0B00)

/* HHP-HPS-specific values */
#define SCC_MGR_HHP_EXTRAS_OFFSET			0
#define SCC_MGR_HHP_DQSE_MAP_OFFSET			1

#define SCC_MGR_DQS_ENA				(BASE_SCC_MGR + 0x0E00)
#define SCC_MGR_DQS_IO_ENA			(BASE_SCC_MGR + 0x0E04)
#define SCC_MGR_DQ_ENA				(BASE_SCC_MGR + 0x0E08)
#define SCC_MGR_DM_ENA				(BASE_SCC_MGR + 0x0E0C)
#define SCC_MGR_UPD				(BASE_SCC_MGR + 0x0E20)
#define SCC_MGR_ACTIVE_RANK			(BASE_SCC_MGR + 0x0E40)
#define SCC_MGR_AFI_CAL_INIT			(BASE_SCC_MGR + 0x0D00)

/* PHY Debug mode flag constants */
#define PHY_DEBUG_IN_DEBUG_MODE 0x00000001
#define PHY_DEBUG_ENABLE_CAL_RPT 0x00000002
#define PHY_DEBUG_ENABLE_MARGIN_RPT 0x00000004
#define PHY_DEBUG_SWEEP_ALL_GROUPS 0x00000008
#define PHY_DEBUG_DISABLE_GUARANTEED_READ 0x00000010
#define PHY_DEBUG_ENABLE_NON_DESTRUCTIVE_CALIBRATION 0x00000020

/* Init and Reset delay constants - Only use if defined by sequencer_defines.h,
 * otherwise, revert to defaults
 * Default for Tinit = (0+1) * ((202+1) * (2 * 131 + 1) + 1) = 53532 =
 * 200.75us @ 266MHz
 */
#ifdef TINIT_CNTR0_VAL
#define SEQ_TINIT_CNTR0_VAL TINIT_CNTR0_VAL
#else
#define SEQ_TINIT_CNTR0_VAL 0
#endif

#ifdef TINIT_CNTR1_VAL
#define SEQ_TINIT_CNTR1_VAL TINIT_CNTR1_VAL
#else
#define SEQ_TINIT_CNTR1_VAL 202
#endif

#ifdef TINIT_CNTR2_VAL
#define SEQ_TINIT_CNTR2_VAL TINIT_CNTR2_VAL
#else
#define SEQ_TINIT_CNTR2_VAL 131
#endif


/* Default for Treset = (2+1) * ((252+1) * (2 * 131 + 1) + 1) = 133563 =
 * 500.86us @ 266MHz
 */
#ifdef TRESET_CNTR0_VAL
#define SEQ_TRESET_CNTR0_VAL TRESET_CNTR0_VAL
#else
#define SEQ_TRESET_CNTR0_VAL 2
#endif

#ifdef TRESET_CNTR1_VAL
#define SEQ_TRESET_CNTR1_VAL TRESET_CNTR1_VAL
#else
#define SEQ_TRESET_CNTR1_VAL 252
#endif

#ifdef TRESET_CNTR2_VAL
#define SEQ_TRESET_CNTR2_VAL TRESET_CNTR2_VAL
#else
#define SEQ_TRESET_CNTR2_VAL 131
#endif

#define RW_MGR_INST_ROM_WRITE BASE_RW_MGR + 0x1800
#define RW_MGR_AC_ROM_WRITE BASE_RW_MGR + 0x1C00

/* parameter variable holder */
struct param_type {
	uint32_t dm_correct_mask;
	uint32_t read_correct_mask;
	uint32_t read_correct_mask_vg;
	uint32_t write_correct_mask;
	uint32_t write_correct_mask_vg;

	/* set a particular entry to 1 if we need to skip a particular rank */

	uint32_t skip_ranks[MAX_RANKS];

	/* set a particular entry to 1 if we need to skip a particular group */

	uint32_t skip_groups;

	/* set a particular entry to 1 if the shadow register
	(which represents a set of ranks) needs to be skipped */

	uint32_t skip_shadow_regs[NUM_SHADOW_REGS];

};


/* global variable holder */
struct gbl_type {
	uint32_t phy_debug_mode_flags;

	/* current read latency */

	uint32_t curr_read_lat;

	/* current write latency */

	uint32_t curr_write_lat;

	/* error code */

	uint32_t error_substage;
	uint32_t error_stage;
	uint32_t error_group;

	/* figure-of-merit in, figure-of-merit out */

	uint32_t fom_in;
	uint32_t fom_out;

	/*USER Number of RW Mgr NOP cycles between
	write command and write data */
	uint32_t rw_wl_nop_cycles;
};
#endif /* _SEQUENCER_H_ */
