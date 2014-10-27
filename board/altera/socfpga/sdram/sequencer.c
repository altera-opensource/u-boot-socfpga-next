/*
 * Copyright Altera Corporation (C) 2012-2014. All rights reserved
 *
 * SPDX-License-Identifier:    BSD-3-Clause
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/sdram.h>
#include "sdram_io.h"
#include "sequencer.h"
#include "sequencer_auto.h"
#include "sequencer_defines.h"
#include "system.h"

static void scc_mgr_load_dqs_for_write_group(uint32_t write_group);
static void rw_mgr_mem_dll_lock_wait(void);

/******************************************************************************
 ******************************************************************************
 ** NOTE: Special Rules for Globale Variables                                **
 **                                                                          **
 ** All global variables that are explicitly initialized (including          **
 ** explicitly initialized to zero), are only initialized once, during       **
 ** configuration time, and not again on reset.  This means that they        **
 ** preserve their current contents across resets, which is needed for some  **
 ** special cases involving communication with external modules.  In         **
 ** addition, this avoids paying the price to have the memory initialized,   **
 ** even for zeroed data, provided it is explicitly set to zero in the code, **
 ** and doesn't rely on implicit initialization.                             **
 ******************************************************************************
 ******************************************************************************/

#if ARRIAV
/*
 * Temporary workaround to place the initial stack pointer at a safe
 * offset from end
 */
#define STRINGIFY(s)		STRINGIFY_STR(s)
#define STRINGIFY_STR(s)	#s
asm(".global __alt_stack_pointer");
asm("__alt_stack_pointer = " STRINGIFY(STACK_POINTER));
#endif

#if CYCLONEV
/*
 * Temporary workaround to place the initial stack pointer at a safe
 * offset from end
 */
#define STRINGIFY(s)		STRINGIFY_STR(s)
#define STRINGIFY_STR(s)	#s
asm(".global __alt_stack_pointer");
asm("__alt_stack_pointer = " STRINGIFY(STACK_POINTER));
#endif

/* Just to make the debugging code more uniform */
#ifndef RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM
#define RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM 0
#endif

#if HALF_RATE
#define HALF_RATE_MODE 1
#else
#define HALF_RATE_MODE 0
#endif

#if QUARTER_RATE
#define QUARTER_RATE_MODE 1
#else
#define QUARTER_RATE_MODE 0
#endif
#define DELTA_D 1

/*
 * case:56390
 * VFIFO_CONTROL_WIDTH_PER_DQS is the number of VFIFOs actually instantiated
 * per DQS. This is always one except:
 * AV QDRII where it is 2 for x18 and x18w2, and 4 for x36 and x36w2
 * RLDRAMII x36 and x36w2 where it is 2.
 * In 12.0sp1 we set this to 4 for all of the special cases above to
 * keep it simple.
 * In 12.0sp2 or 12.1 this should get moved to generation and unified with
 * the same constant used in the phy mgr
 */

#define VFIFO_CONTROL_WIDTH_PER_DQS 1

#if ARRIAV

#if QDRII
 #if RW_MGR_MEM_DQ_PER_READ_DQS > 9
  #undef VFIFO_CONTROL_WIDTH_PER_DQS
  #define VFIFO_CONTROL_WIDTH_PER_DQS 4
 #endif
#endif /* protocol check */

#if RLDRAMII
 #if RW_MGR_MEM_DQ_PER_READ_DQS > 9
  #undef VFIFO_CONTROL_WIDTH_PER_DQS
  #define VFIFO_CONTROL_WIDTH_PER_DQS 2
 #endif
#endif /* protocol check */

#endif /* family check */

/*
 * In order to reduce ROM size, most of the selectable calibration steps are
 * decided at compile time based on the user's calibration mode selection,
 * as captured by the STATIC_CALIB_STEPS selection below.
 *
 * However, to support simulation-time selection of fast simulation mode, where
 * we skip everything except the bare minimum, we need a few of the steps to
 * be dynamic.  In those cases, we either use the DYNAMIC_CALIB_STEPS for the
 * check, which is based on the rtl-supplied value, or we dynamically compute
 * the value to use based on the dynamically-chosen calibration mode
 */

#if QDRII
#define BTFLD_FMT "%llx"
#else
#define BTFLD_FMT "%u"
#endif

/* For HPS running on actual hardware */

#define DLEVEL 0
#ifdef HPS_HW_SERIAL_SUPPORT
/*
 * space around comma is required for varargs macro to remove comma if args
 * is empty
 */
#define DPRINT(level, fmt, args...)	if (DLEVEL >= (level)) \
					printf("SEQ.C: " fmt "\n" , ## args)
#define IPRINT(fmt, args...)	        printf("SEQ.C: " fmt "\n" , ## args)
#else
#define DPRINT(level, fmt, args...)
#define IPRINT(fmt, args...)
#endif
#define BFM_GBL_SET(field, value)
#define BFM_GBL_GET(field)		((long unsigned int)0)
#define BFM_STAGE(stage)
#define BFM_INC_VFIFO
#define COV(label)

#define TRACE_FUNC(fmt, args...) \
	DPRINT(1, "%s[%d]: " fmt, __func__, __LINE__ , ## args)

#define DYNAMIC_CALIB_STEPS (dyn_calib_steps)

#if STATIC_SIM_FILESET
#define STATIC_IN_RTL_SIM CALIB_IN_RTL_SIM
#else
#define STATIC_IN_RTL_SIM 0
#endif

#if STATIC_SKIP_MEM_INIT
#define STATIC_SKIP_DELAY_LOOPS CALIB_SKIP_DELAY_LOOPS
#else
#define STATIC_SKIP_DELAY_LOOPS 0
#endif

#define STATIC_CALIB_STEPS (STATIC_IN_RTL_SIM | CALIB_SKIP_FULL_TEST | \
	STATIC_SKIP_DELAY_LOOPS)

/* calibration steps requested by the rtl */
uint16_t dyn_calib_steps;

/*
 * To make CALIB_SKIP_DELAY_LOOPS a dynamic conditional option
 * instead of static, we use boolean logic to select between
 * non-skip and skip values
 *
 * The mask is set to include all bits when not-skipping, but is
 * zero when skipping
 */

uint16_t skip_delay_mask;	/* mask off bits when skipping/not-skipping */

#define SKIP_DELAY_LOOP_VALUE_OR_ZERO(non_skip_value) \
	((non_skip_value) & skip_delay_mask)

struct gbl_type *gbl;
struct param_type *param;
uint32_t curr_shadow_reg;

static uint32_t rw_mgr_mem_calibrate_write_test(uint32_t rank_bgn,
	uint32_t write_group, uint32_t use_dm,
	uint32_t all_correct, uint32_t *bit_chk, uint32_t all_ranks);

#if ENABLE_BRINGUP_DEBUGGING
#define DI_BUFFER_DEBUG_SIZE   64

uint8_t di_buf_gbl[DI_BUFFER_DEBUG_SIZE*4] = {0};

void load_di_buf_gbl(void)
{
	int i;
	int j;

	for (i = 0; i < DI_BUFFER_DEBUG_SIZE; i++) {
		uint32_t val = IORD_32DIRECT(RW_MGR_DI_BASE + i*4, 0);
		for (j = 0; j < 4; j++) {
			uint8_t byte = (val >> (8*j)) & 0xff;
			di_buf_gbl[i*4 + j] = byte;
		}
	}
}
#endif	/* ENABLE_BRINGUP_DEBUGGING */


#if ENABLE_DQSEN_SWEEP
void init_di_buffer(void)
{
	uint32_t i;

	debug_data->di_report.flags = 0;
	debug_data->di_report.cur_samples = 0;

	for (i = 0; i < NUM_DI_SAMPLE; i++) {
		debug_data->di_report.di_buffer[i].bit_chk = 0;
		debug_data->di_report.di_buffer[i].delay = 0;
		debug_data->di_report.di_buffer[i].d = 0;
		debug_data->di_report.di_buffer[i].v = 0;
		debug_data->di_report.di_buffer[i].p = 0;
		debug_data->di_report.di_buffer[i].di_buffer_0a = 0;
		debug_data->di_report.di_buffer[i].di_buffer_0b = 0;
		debug_data->di_report.di_buffer[i].di_buffer_1a = 0;
		debug_data->di_report.di_buffer[i].di_buffer_1b = 0;
		debug_data->di_report.di_buffer[i].di_buffer_2a = 0;
		debug_data->di_report.di_buffer[i].di_buffer_2b = 0;
		debug_data->di_report.di_buffer[i].di_buffer_3a = 0;
		debug_data->di_report.di_buffer[i].di_buffer_3b = 0;
		debug_data->di_report.di_buffer[i].di_buffer_4a = 0;
		debug_data->di_report.di_buffer[i].di_buffer_4b = 0;
	}
}

void wait_di_buffer(void)
{
	if (debug_data->di_report.cur_samples == NUM_DI_SAMPLE)	{
		debug_data->di_report.flags |= DI_REPORT_FLAGS_READY;
		while (debug_data->di_report.cur_samples != 0)
			;
		debug_data->di_report.flags = 0;
	}
}

void sample_di_data(uint32_t bit_chk, uint32_t delay, uint32_t d,
	uint32_t v, uint32_t p)
{
	uint32_t k;
	uint32_t di_status_word;
	uint32_t di_word_avail;
	uint32_t di_write_to_read_ratio;
	uint32_t di_write_to_read_ratio_2_exp;

	wait_di_buffer();

	k = debug_data->di_report.cur_samples;

	debug_data->di_report.di_buffer[k].bit_chk = bit_chk;
	debug_data->di_report.di_buffer[k].delay = delay;
	debug_data->di_report.di_buffer[k].d = d;
	debug_data->di_report.di_buffer[k].v = v;
	debug_data->di_report.di_buffer[k].p = p;

	di_status_word = IORD_32DIRECT(BASE_RW_MGR + 8, 0);
	di_word_avail = di_status_word & 0x0000FFFF;
	di_write_to_read_ratio = (di_status_word & 0x00FF0000) >> 16;
	di_write_to_read_ratio_2_exp = (di_status_word & 0xFF000000) >> 24;

	debug_data->di_report.di_buffer[k].di_buffer_0a =
		IORD_32DIRECT(BASE_RW_MGR + 16 + 0*4, 0);
	debug_data->di_report.di_buffer[k].di_buffer_0b =
		IORD_32DIRECT(BASE_RW_MGR + 16 + 1*4, 0);
	debug_data->di_report.di_buffer[k].di_buffer_1a =
		IORD_32DIRECT(BASE_RW_MGR + 16 + 2*4, 0);
	debug_data->di_report.di_buffer[k].di_buffer_1b =
		IORD_32DIRECT(BASE_RW_MGR + 16 + 3*4, 0);
	debug_data->di_report.di_buffer[k].di_buffer_2a =
		IORD_32DIRECT(BASE_RW_MGR + 16 + 4*4, 0);
	debug_data->di_report.di_buffer[k].di_buffer_2b =
		IORD_32DIRECT(BASE_RW_MGR + 16 + 5*4, 0);
	debug_data->di_report.di_buffer[k].di_buffer_3a =
		IORD_32DIRECT(BASE_RW_MGR + 16 + 6*4, 0);
	debug_data->di_report.di_buffer[k].di_buffer_3b =
		IORD_32DIRECT(BASE_RW_MGR + 16 + 7*4, 0);
	debug_data->di_report.di_buffer[k].di_buffer_4a =
		IORD_32DIRECT(BASE_RW_MGR + 16 + 8*4, 0);
	debug_data->di_report.di_buffer[k].di_buffer_4b =
		IORD_32DIRECT(BASE_RW_MGR + 16 + 9*4, 0);

	debug_data->di_report.cur_samples =
		debug_data->di_report.cur_samples + 1;
}
#endif

/*
 * This (TEST_SIZE) is used to test handling of large roms, to make
 * sure we are sizing things correctly
 * Note, the initialized data takes up twice the space in rom, since
 *there needs to be a copy with the initial value and a copy that is
 * written too, since on soft-reset, it needs to have the initial values
 * without reloading the memory from external sources
 */


#ifdef TEST_SIZE

#define PRE_POST_TEST_SIZE 3

unsigned int pre_test_size_mem[PRE_POST_TEST_SIZE] = { 1, 2, 3};
unsigned int test_size_mem[TEST_SIZE/sizeof(unsigned int)] = { 100, 200, 300 };
unsigned int post_test_size_mem[PRE_POST_TEST_SIZE] = {10, 20, 30};
void write_test_mem(void)
{
	int i;

	for (i = 0; i < PRE_POST_TEST_SIZE; i++) {
		pre_test_size_mem[i] = (i+1)*10;
		post_test_size_mem[i] = (i+1);
	}

	for (i = 0; i < sizeof(test_size_mem)/sizeof(unsigned int); i++)
		test_size_mem[i] = i;
}

int check_test_mem(int start)
{
	int i;

	for (i = 0; i < PRE_POST_TEST_SIZE; i++) {
		if (start) {
			if (pre_test_size_mem[i] != (i+1))
				return 0;
			if (post_test_size_mem[i] != (i+1)*10)
				return 0;
		} else {
			if (pre_test_size_mem[i] != (i+1)*10)
				return 0;
			if (post_test_size_mem[i] != (i+1))
				return 0;
		}
	}

	for (i = 0; i < sizeof(test_size_mem)/sizeof(unsigned int); i++) {
		if (start) {
			if (i < 3) {
				if (test_size_mem[i] != (i+1)*100)
					return 0;
			} else {
				if (test_size_mem[i] != 0)
					return 0;
			}
		} else {
			if (test_size_mem[i] != i)
				return 0;
		}
	}

	return 1;
}

#endif /* TEST_SIZE */

static void set_failing_group_stage(uint32_t group, uint32_t stage,
	uint32_t substage)
{
	ALTERA_ASSERT(group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);

	/*
	 * Only set the global stage if there was not been any other
	 * failing group
	 */
	if (gbl->error_stage == CAL_STAGE_NIL)	{
		gbl->error_substage = substage;
		gbl->error_stage = stage;
		gbl->error_group = group;
	}
}

static inline void reg_file_set_group(uint32_t set_group)
{
	/* Read the current group and stage */
	uint32_t cur_stage_group = IORD_32DIRECT(REG_FILE_CUR_STAGE, 0);

	/* Clear the group */
	cur_stage_group &= 0x0000FFFF;

	/* Set the group */
	cur_stage_group |= (set_group << 16);

	/* Write the data back */
	IOWR_32DIRECT(REG_FILE_CUR_STAGE, 0, cur_stage_group);
}

static inline void reg_file_set_stage(uint32_t set_stage)
{
	/* Read the current group and stage */
	uint32_t cur_stage_group = IORD_32DIRECT(REG_FILE_CUR_STAGE, 0);

	/* Clear the stage and substage */
	cur_stage_group &= 0xFFFF0000;

	/* Set the stage */
	cur_stage_group |= (set_stage & 0x000000FF);

	/* Write the data back */
	IOWR_32DIRECT(REG_FILE_CUR_STAGE, 0, cur_stage_group);
}

static inline void reg_file_set_sub_stage(uint32_t set_sub_stage)
{
	/* Read the current group and stage */
	uint32_t cur_stage_group = IORD_32DIRECT(REG_FILE_CUR_STAGE, 0);

	/* Clear the substage */
	cur_stage_group &= 0xFFFF00FF;

	/* Set the sub stage */
	cur_stage_group |= ((set_sub_stage << 8) & 0x0000FF00);

	/* Write the data back */
	IOWR_32DIRECT(REG_FILE_CUR_STAGE, 0, cur_stage_group);
}

static inline uint32_t is_write_group_enabled_for_dm(uint32_t write_group)
{
#if DM_PINS_ENABLED
 #if RLDRAMII
	int32_t decrement_counter = write_group + 1;

	while (decrement_counter > 0)
		decrement_counter -= RW_MGR_MEM_IF_WRITE_DQS_WIDTH /
			RW_MGR_MEM_DATA_MASK_WIDTH;

	if (decrement_counter == 0)
		return 1;
	return 0;
 #else
	return 1;
 #endif
#else
	return 0;
#endif
}

static inline void select_curr_shadow_reg_using_rank(uint32_t rank)
{
#if USE_SHADOW_REGS
	/* USER Map the rank to its shadow reg and set the global variable  */
	curr_shadow_reg = (rank >> (NUM_RANKS_PER_SHADOW_REG - 1));
#endif
}

void initialize(void)
{
	TRACE_FUNC();

	/* USER calibration has control over path to memory */
#if HARD_PHY
	/*
	 * In Hard PHY this is a 2-bit control:
	 * 0: AFI Mux Select
	 * 1: DDIO Mux Select
	 */
	IOWR_32DIRECT(PHY_MGR_MUX_SEL, 0, 0x3);
#else
	IOWR_32DIRECT(PHY_MGR_MUX_SEL, 0, 1);
#endif

	/* USER memory clock is not stable we begin initialization  */

	IOWR_32DIRECT(PHY_MGR_RESET_MEM_STBL, 0, 0);

	/* USER calibration status all set to zero */

	IOWR_32DIRECT(PHY_MGR_CAL_STATUS, 0, 0);
	IOWR_32DIRECT(PHY_MGR_CAL_DEBUG_INFO, 0, 0);

	if (((DYNAMIC_CALIB_STEPS) & CALIB_SKIP_ALL) != CALIB_SKIP_ALL) {
		param->read_correct_mask_vg  = ((uint32_t)1 <<
			(RW_MGR_MEM_DQ_PER_READ_DQS /
			RW_MGR_MEM_VIRTUAL_GROUPS_PER_READ_DQS)) - 1;
		param->write_correct_mask_vg = ((uint32_t)1 <<
			(RW_MGR_MEM_DQ_PER_READ_DQS /
			RW_MGR_MEM_VIRTUAL_GROUPS_PER_READ_DQS)) - 1;
		param->read_correct_mask     = ((uint32_t)1 <<
			RW_MGR_MEM_DQ_PER_READ_DQS) - 1;
		param->write_correct_mask    = ((uint32_t)1 <<
			RW_MGR_MEM_DQ_PER_WRITE_DQS) - 1;
		param->dm_correct_mask       = ((uint32_t)1 <<
			(RW_MGR_MEM_DATA_WIDTH / RW_MGR_MEM_DATA_MASK_WIDTH))
			- 1;
	}
}


#if MRS_MIRROR_PING_PONG_ATSO
/*
 * This code is specific to the ATSO setup.  There are two ways to set
 * the cs/odt mask:
 * 1. the normal way (set_rank_and_odt_mask)
 * This method will be used in general.  The behavior will be to unmask
 * BOTH CS (i.e. broadcast to both sides as if calibrating one large interface).
 * 2. this function
 * This method will be used for MRS settings only.  This allows us to do
 * settings on a per-side basis.  This is needed because Slot 1 Rank 1 needs
 * a mirrored MRS. This function is specific to our setup ONLY.
 */
void set_rank_and_odt_mask_for_ping_pong_atso(uint32_t side, uint32_t odt_mode)
{
	uint32_t odt_mask_0 = 0;
	uint32_t odt_mask_1 = 0;
	uint32_t cs_and_odt_mask;

	if (odt_mode == RW_MGR_ODT_MODE_READ_WRITE) {
		/*
		 * USER 1 Rank
		 * USER Read: ODT = 0
		 * USER Write: ODT = 1
		 */
		odt_mask_0 = 0x0;
		odt_mask_1 = 0x1;
	} else	{
		odt_mask_0 = 0x0;
		odt_mask_1 = 0x0;
	}

	cs_and_odt_mask =
		(0xFF & ~(1 << side)) |
		((0xFF & odt_mask_0) << 8) |
		((0xFF & odt_mask_1) << 16);

	IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, cs_and_odt_mask);
}
#endif

#if DDR3
void set_rank_and_odt_mask(uint32_t rank, uint32_t odt_mode)
{
	uint32_t odt_mask_0 = 0;
	uint32_t odt_mask_1 = 0;
	uint32_t cs_and_odt_mask;

	if (odt_mode == RW_MGR_ODT_MODE_READ_WRITE) {
#if USE_SHADOW_REGS
		uint32_t rank_one_hot = (0xFF & (1 << rank));
		select_curr_shadow_reg_using_rank(rank);

		/*
		 * Assert afi_rrank and afi_wrank. These signals ultimately
		 * drive the read/write rank select signals which select the
		 * shadow register.
		 */
		IOWR_32DIRECT(RW_MGR_SET_ACTIVE_RANK, 0, rank_one_hot);
#endif

		if (LRDIMM) {
			/*
			 * LRDIMMs have two cases to consider: single-slot and
			 * dual-slot. In single-slot, assert ODT for write only.
			 * In dual-slot, assert ODT for both slots for write,
			 * and on the opposite slot only for reads.
			 *
			 * Further complicating this is that both DIMMs have
			 * either 1 or 2 ODT inputs, which do the same thing
			 * (only one is actually required).
			 */
			if ((RW_MGR_MEM_CHIP_SELECT_WIDTH /
			     RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM) == 1) {
				/* USER Single-slot case */
				if (RW_MGR_MEM_ODT_WIDTH == 1) {
					/* USER Read = 0, Write = 1*/
					odt_mask_0 = 0x0;
					odt_mask_1 = 0x1;
				} else if (RW_MGR_MEM_ODT_WIDTH == 2) {
					/* USER Read = 00, Write = 11*/
					odt_mask_0 = 0x0;
					odt_mask_1 = 0x3;
				}
			} else if ((RW_MGR_MEM_CHIP_SELECT_WIDTH /
				    RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM) == 2) {
				/* USER Dual-slot case*/
				if (RW_MGR_MEM_ODT_WIDTH == 2) {
					/*
					 * USER Read: asserted for opposite
					 * slot, Write: asserted for both
					 */
					odt_mask_0 = (rank < 2) ? 0x2 : 0x1;
					odt_mask_1 = 0x3;
				} else if (RW_MGR_MEM_ODT_WIDTH == 4) {
					/*
					 * USER Read: asserted for opposite
					 * slot, Write: asserted for both
					 */
					odt_mask_0 = (rank < 2) ? 0xC : 0x3;
					odt_mask_1 = 0xF;
				}
			}
		} else if (RW_MGR_MEM_NUMBER_OF_RANKS == 1) {
			/*
			 * 1 Rank
			 * Read: ODT = 0
			 * Write: ODT = 1
			 */
			odt_mask_0 = 0x0;
			odt_mask_1 = 0x1;
		} else if (RW_MGR_MEM_NUMBER_OF_RANKS == 2) {
			/* 2 Ranks */
			if (RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM == 1 ||
			    (RDIMM && RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM == 2 &&
			    RW_MGR_MEM_CHIP_SELECT_WIDTH == 4)) {
				/* - Dual-Slot , Single-Rank
				 * (1 chip-select per DIMM)
				 * OR
				 * - RDIMM, 4 total CS (2 CS per DIMM)
				 * means 2 DIMM
				 * Since MEM_NUMBER_OF_RANKS is 2 they are
				 * both single rank
				 * with 2 CS each (special for RDIMM)
				 * Read: Turn on ODT on the opposite rank
				 * Write: Turn on ODT on all ranks
				 */
				odt_mask_0 = 0x3 & ~(1 << rank);
				odt_mask_1 = 0x3;
			} else {
				/*
				 * USER - Single-Slot , Dual-rank DIMMs
				 * (2 chip-selects per DIMM)
				 * USER Read: Turn on ODT off on all ranks
				 * USER Write: Turn on ODT on active rank
				 */
				odt_mask_0 = 0x0;
				odt_mask_1 = 0x3 & (1 << rank);
			}
				} else {
			/* 4 Ranks
			 * Read:
			 * ----------+-----------------------+
			 *           |                       |
			 *           |         ODT           |
			 * Read From +-----------------------+
			 *   Rank    |  3  |  2  |  1  |  0  |
			 * ----------+-----+-----+-----+-----+
			 *     0     |  0  |  1  |  0  |  0  |
			 *     1     |  1  |  0  |  0  |  0  |
			 *     2     |  0  |  0  |  0  |  1  |
			 *     3     |  0  |  0  |  1  |  0  |
			 * ----------+-----+-----+-----+-----+
			 *
			 * Write:
			 * ----------+-----------------------+
			 *           |                       |
			 *           |         ODT           |
			 * Write To  +-----------------------+
			 *   Rank    |  3  |  2  |  1  |  0  |
			 * ----------+-----+-----+-----+-----+
			 *     0     |  0  |  1  |  0  |  1  |
			 *     1     |  1  |  0  |  1  |  0  |
			 *     2     |  0  |  1  |  0  |  1  |
			 *     3     |  1  |  0  |  1  |  0  |
			 * ----------+-----+-----+-----+-----+
			 */
			switch (rank) {
			case 0:
				odt_mask_0 = 0x4;
				odt_mask_1 = 0x5;
				break;
			case 1:
				odt_mask_0 = 0x8;
				odt_mask_1 = 0xA;
				break;
			case 2:
				odt_mask_0 = 0x1;
				odt_mask_1 = 0x5;
				break;
			case 3:
				odt_mask_0 = 0x2;
				odt_mask_1 = 0xA;
				break;
			}
		}
	} else {
		odt_mask_0 = 0x0;
		odt_mask_1 = 0x0;
	}

#if MRS_MIRROR_PING_PONG_ATSO
	/* See set_cs_and_odt_mask_for_ping_pong_atso*/
	cs_and_odt_mask =
			(0xFC) |
			((0xFF & odt_mask_0) << 8) |
			((0xFF & odt_mask_1) << 16);
#else
	if (RDIMM && RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM == 2 &&
	    RW_MGR_MEM_CHIP_SELECT_WIDTH == 4 &&
	    RW_MGR_MEM_NUMBER_OF_RANKS == 2) {
		/* See RDIMM special case above */
		cs_and_odt_mask =
			(0xFF & ~(1 << (2*rank))) |
			((0xFF & odt_mask_0) << 8) |
			((0xFF & odt_mask_1) << 16);
	} else if (LRDIMM) {
#if LRDIMM
		/*
		 * USER LRDIMM special cases - When RM=2, CS[2] is remapped
		 * to A[16] so skip it, and when RM=4, CS[3:2] are remapped
		 * to A[17:16] so skip them both.
		 */
		uint32_t lrdimm_rank = 0;
		uint32_t lrdimm_rank_mask = 0;

		/*
		 * USER When rank multiplication is active, the remapped CS
		 * pins must be forced low instead of high for proper
		 * targetted RTT_NOM programming.
		 */
		if (LRDIMM_RANK_MULTIPLICATION_FACTOR == 2)
			/* USER Mask = CS[5:0] = 011011 */
			lrdimm_rank_mask = (0x3 | (0x3 << 3));
		else if (LRDIMM_RANK_MULTIPLICATION_FACTOR == 4)
			/* USER Mask = CS[7:0] = 00110011 */
			lrdimm_rank_mask = (0x3 | (0x3 << 4));

		/*
		 * USER Handle LRDIMM cases where Rank multiplication
		 * may be active
		 */
		if (((RW_MGR_MEM_CHIP_SELECT_WIDTH /
		      RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM) == 1)) {
			/* USER Single-DIMM case */
			lrdimm_rank = ~(1 << rank);
		} else if ((RW_MGR_MEM_CHIP_SELECT_WIDTH /
			    RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM) == 2) {
			if (rank < (RW_MGR_MEM_NUMBER_OF_RANKS >> 1)) {
			/* USER Dual-DIMM case, accessing first slot */
				lrdimm_rank = ~(1 << rank);
			} else {
			/* USER Dual-DIMM case, accessing second slot */
				lrdimm_rank = ~(1 << (rank +
				RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM -
				(RW_MGR_MEM_NUMBER_OF_RANKS>>1)));
			}
		}
		cs_and_odt_mask =
			(lrdimm_rank_mask & lrdimm_rank) |
			((0xFF & odt_mask_0) << 8) |
			((0xFF & odt_mask_1) << 16);
#endif /* LRDIMM */
	} else {
		cs_and_odt_mask =
			(0xFF & ~(1 << rank)) |
			((0xFF & odt_mask_0) << 8) |
			((0xFF & odt_mask_1) << 16);
	}
#endif
	IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, cs_and_odt_mask);
}
#else
#if DDR2
void set_rank_and_odt_mask(uint32_t rank, uint32_t odt_mode)
{
	uint32_t odt_mask_0 = 0;
	uint32_t odt_mask_1 = 0;
	uint32_t cs_and_odt_mask;

	if (odt_mode == RW_MGR_ODT_MODE_READ_WRITE) {
		if (RW_MGR_MEM_NUMBER_OF_RANKS == 1) {
			/*
			 * 1 Rank
			 * Read: ODT = 0
			 * Write: ODT = 1
			 */
			odt_mask_0 = 0x0;
			odt_mask_1 = 0x1;
		} else if (RW_MGR_MEM_NUMBER_OF_RANKS == 2) {
			/* 2 Ranks */
			if (RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM == 1 ||
			    (RDIMM && RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM == 2 &&
			    RW_MGR_MEM_CHIP_SELECT_WIDTH == 4)) {
				/*
				 * USER - Dual-Slot ,
				 * Single-Rank (1 chip-select per DIMM)
				 * OR
				 * - RDIMM, 4 total CS (2 CS per DIMM) means
				 * 2 DIMM
				 * Since MEM_NUMBER_OF_RANKS is 2 they are both
				 * single rank with 2 CS each (special for
				 * RDIMM)
				 * Read/Write: Turn on ODT on the opposite rank
				 */
				odt_mask_0 = 0x3 & ~(1 << rank);
				odt_mask_1 = 0x3 & ~(1 << rank);
			} else {
				/*
				 * USER - Single-Slot , Dual-rank DIMMs
				 * (2 chip-selects per DIMM)
				 * Read: Turn on ODT off on all ranks
				 * Write: Turn on ODT on active rank
				 */
				odt_mask_0 = 0x0;
				odt_mask_1 = 0x3 & (1 << rank);
			}
		} else {
			/*
			 * 4 Ranks
			 * Read/Write:
			 * -----------+-----------------------+
			 *            |                       |
			 *            |         ODT           |
			 * Read/Write |                       |
			 *   From     +-----------------------+
			 *   Rank     |  3  |  2  |  1  |  0  |
			 * -----------+-----+-----+-----+-----+
			 *     0      |  0  |  1  |  0  |  0  |
			 *     1      |  1  |  0  |  0  |  0  |
			 *     2      |  0  |  0  |  0  |  1  |
			 *     3      |  0  |  0  |  1  |  0  |
			 * -----------+-----+-----+-----+-----+
			 */
			switch (rank) {
			case 0:
				odt_mask_0 = 0x4;
				odt_mask_1 = 0x4;
				break;
			case 1:
				odt_mask_0 = 0x8;
				odt_mask_1 = 0x8;
				break;
			case 2:
				odt_mask_0 = 0x1;
				odt_mask_1 = 0x1;
				break;
			case 3:
				odt_mask_0 = 0x2;
				odt_mask_1 = 0x2;
				break;
			}
		}
	} else {
		odt_mask_0 = 0x0;
		odt_mask_1 = 0x0;
	}

	if (RDIMM && RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM == 2 &&
	    RW_MGR_MEM_CHIP_SELECT_WIDTH == 4 &&
	    RW_MGR_MEM_NUMBER_OF_RANKS == 2) {
		/* See RDIMM/LRDIMM special case above */
		cs_and_odt_mask =
			(0xFF & ~(1 << (2*rank))) |
			((0xFF & odt_mask_0) << 8) |
			((0xFF & odt_mask_1) << 16);
	} else {
		cs_and_odt_mask =
			(0xFF & ~(1 << rank)) |
			((0xFF & odt_mask_0) << 8) |
			((0xFF & odt_mask_1) << 16);
	}

	IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, cs_and_odt_mask);
}
#else /* QDRII and RLDRAMx */
void set_rank_and_odt_mask(uint32_t rank, uint32_t odt_mode)
{
	uint32_t cs_and_odt_mask =
		(0xFF & ~(1 << rank));

	IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, cs_and_odt_mask);
}
#endif
#endif

/*
 * Given a rank, select the set of shadow registers that is responsible
 * for the delays of such rank, so that subsequent SCC updates will
 * go to those shadow registers.
 */
void select_shadow_regs_for_update(uint32_t rank, uint32_t group,
	uint32_t update_scan_chains)
{
#if USE_SHADOW_REGS
	uint32_t rank_one_hot = (0xFF & (1 << rank));

	/*
	 * USER Assert afi_rrank and afi_wrank. These signals ultimately drive
	 * the read/write rank select signals which select the shadow register.
	 */
	IOWR_32DIRECT(RW_MGR_SET_ACTIVE_RANK, 0, rank_one_hot);

	/*
	 * USER Cause the SCC manager to switch its register file, which is used
	 * as local cache of the various dtap/ptap settings. There's one
	 * register file per shadow register set.
	 */
	IOWR_32DIRECT(SCC_MGR_ACTIVE_RANK, 0, rank_one_hot);

	if (update_scan_chains) {
		uint32_t i;

		/*
		 * USER On the read side, a memory read is required because the
		 * read rank select signal (as well as the postamble delay
		 * chain settings) is clocked into the periphery by the
		 * postamble signal. Simply asserting afi_rrank is not enough.
		 * If update_scc_regfile is not set, we assume there'll be a
		 * subsequent read that'll handle this.
		 */
		for (i = 0; i < RW_MGR_MEM_NUMBER_OF_RANKS; ++i) {
			/*
			 * USER The dummy read can go to any non-skipped rank.
			 * Skipped ranks are uninitialized and their banks are
			 * un-activated. Accessing skipped ranks can lead to
			 * bad behavior.
			 */
			if (!param->skip_ranks[i]) {
				set_rank_and_odt_mask(i,
						      RW_MGR_ODT_MODE_READ_WRITE);
				/*
				 * must re-assert afi_wrank/afi_rrank prior to
				 * issuing read because set_rank_and_odt_mask
				 * may have changed the signals.
				 */
				IOWR_32DIRECT(RW_MGR_SET_ACTIVE_RANK, 0,
					      rank_one_hot);

				IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, 0x10);
				IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
					      __RW_MGR_READ_B2B_WAIT1);

				IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, 0x10);
				IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
					      __RW_MGR_READ_B2B_WAIT2);

				IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x0);
				IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0,
					      __RW_MGR_READ_B2B);

				IOWR_32DIRECT(RW_MGR_LOAD_CNTR_3, 0, 0x0);
				IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_3, 0,
					      __RW_MGR_READ_B2B);

				IOWR_32DIRECT(RW_MGR_RUN_ALL_GROUPS, 0,
					      __RW_MGR_READ_B2B);

				/*
				 * USER The dummy read above may cause the DQS
				 * enable signal to be stuck high. The
				 * following corrects this.
				 */
				IOWR_32DIRECT(RW_MGR_RUN_ALL_GROUPS, 0,
					      __RW_MGR_CLEAR_DQS_ENABLE);

				set_rank_and_odt_mask(i, RW_MGR_ODT_MODE_OFF);

				break;
			}
		}

		/* Reset the fifos to get pointers to known state */
		IOWR_32DIRECT(PHY_MGR_CMD_FIFO_RESET, 0, 0);
		IOWR_32DIRECT(RW_MGR_RESET_READ_DATAPATH, 0, 0);

		/*
		 * USER On the write side the afi_wrank signal eventually
		 * propagates to the I/O
		 * through the write datapath. We need to make sure we wait
		 * long enough for this to happen. The operations above should
		 * be enough, hence no extra delay
		 * inserted here.
		 */

		/*
		 * USER Make sure the data in the I/O scan chains are in-sync
		 * with the register file inside the SCC manager. If we don't
		 * do this, a subsequent SCC_UPDATE may cause stale data for
		 * the other shadow register to be loaded. This must
		 * be done for every scan chain of the current group.
		 * that in shadow register mode, the SCC_UPDATE signal is
		 * per-group.
		 */
		IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0, group);
		IOWR_32DIRECT(SCC_MGR_DQS_ENA, 0, group);
		IOWR_32DIRECT(SCC_MGR_DQS_IO_ENA, 0, 0);

		for (i = 0; i < RW_MGR_MEM_DQ_PER_WRITE_DQS; i++)
			IOWR_32DIRECT(SCC_MGR_DQ_ENA, 0, i);
		for (i = 0; i < RW_MGR_NUM_DM_PER_WRITE_GROUP; i++)
			IOWR_32DIRECT(SCC_MGR_DM_ENA, 0, i);
	}

	/* Map the rank to its shadow reg */
	select_curr_shadow_reg_using_rank(rank);
#endif
}

void scc_mgr_initialize(void)
{
	/*
	 * Clear register file for HPS
	 * 16 (2^4) is the size of the full register file in the scc mgr:
	 *	RFILE_DEPTH = log2(MEM_DQ_PER_DQS + 1 + MEM_DM_PER_DQS +
	 * MEM_IF_READ_DQS_WIDTH - 1) + 1;
	 */
	uint32_t i;
	for (i = 0; i < 16; i++) {
		DPRINT(1, "Clearing SCC RFILE index %u", i);
		IOWR_32DIRECT(SCC_MGR_HHP_RFILE, i << 2, 0);
	}
}

inline void scc_mgr_set_dqs_bus_in_delay(uint32_t read_group, uint32_t delay)
{
	ALTERA_ASSERT(read_group < RW_MGR_MEM_IF_READ_DQS_WIDTH);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DQS_IN_DELAY(read_group, delay);
}

static inline void scc_mgr_set_dqs_io_in_delay(uint32_t write_group,
	uint32_t delay)
{
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DQS_IO_IN_DELAY(delay);
}

static inline void scc_mgr_set_dqs_en_phase(uint32_t read_group, uint32_t phase)
{
	ALTERA_ASSERT(read_group < RW_MGR_MEM_IF_READ_DQS_WIDTH);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DQS_EN_PHASE(read_group, phase);
}

void scc_mgr_set_dqs_en_phase_all_ranks(uint32_t read_group, uint32_t phase)
{
	uint32_t r;
	uint32_t update_scan_chains;

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS;
	     r += NUM_RANKS_PER_SHADOW_REG) {
		/*
		 * USER although the h/w doesn't support different phases per
		 * shadow register, for simplicity our scc manager modeling
		 * keeps different phase settings per shadow reg, and it's
		 * important for us to keep them in sync to match h/w.
		 * for efficiency, the scan chain update should occur only
		 * once to sr0.
		 */
		update_scan_chains = (r == 0) ? 1 : 0;

		select_shadow_regs_for_update(r, read_group,
					      update_scan_chains);
		scc_mgr_set_dqs_en_phase(read_group, phase);

		if (update_scan_chains) {
			IOWR_32DIRECT(SCC_MGR_DQS_ENA, 0, read_group);
			IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
		}
	}
}

static inline void scc_mgr_set_dqdqs_output_phase(uint32_t write_group,
	uint32_t phase)
{
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);

	#if CALIBRATE_BIT_SLIPS
	uint32_t num_fr_slips = 0;
	while (phase > IO_DQDQS_OUT_PHASE_MAX) {
		phase -= IO_DLL_CHAIN_LENGTH;
		num_fr_slips++;
	}
	IOWR_32DIRECT(PHY_MGR_FR_SHIFT, write_group*4, num_fr_slips);
#endif

	/* Load the setting in the SCC manager */
	WRITE_SCC_DQDQS_OUT_PHASE(write_group, phase);
}

static void scc_mgr_set_dqdqs_output_phase_all_ranks(uint32_t write_group,
	uint32_t phase)
{
	uint32_t r;
	uint32_t update_scan_chains;

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS;
	     r += NUM_RANKS_PER_SHADOW_REG) {
		/*
		 * USER although the h/w doesn't support different phases per
		 * shadow register, for simplicity our scc manager modeling
		 * keeps different phase settings per shadow reg, and it's
		 * important for us to keep them in sync to match h/w.
		 * for efficiency, the scan chain update should occur only
		 * once to sr0.
		 */
		update_scan_chains = (r == 0) ? 1 : 0;

		select_shadow_regs_for_update(r, write_group,
					      update_scan_chains);
		scc_mgr_set_dqdqs_output_phase(write_group, phase);

		if (update_scan_chains) {
			IOWR_32DIRECT(SCC_MGR_DQS_ENA, 0, write_group);
			IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
		}
	}
}

static inline void scc_mgr_set_dqs_en_delay(uint32_t read_group, uint32_t delay)
{
	ALTERA_ASSERT(read_group < RW_MGR_MEM_IF_READ_DQS_WIDTH);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DQS_EN_DELAY(read_group, delay);
}

static void scc_mgr_set_dqs_en_delay_all_ranks(uint32_t read_group,
	uint32_t delay)
{
	uint32_t r;

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS;
		r += NUM_RANKS_PER_SHADOW_REG) {
		select_shadow_regs_for_update(r, read_group, 0);

		scc_mgr_set_dqs_en_delay(read_group, delay);

		IOWR_32DIRECT(SCC_MGR_DQS_ENA, 0, read_group);

#if !USE_SHADOW_REGS
		/*
		 * In shadow register mode, the T11 settings are stored in
		 * registers in the core, which are updated by the DQS_ENA
		 * signals. Not issuing the SCC_MGR_UPD command allows us to
		 * save lots of rank switching overhead, by calling
		 * select_shadow_regs_for_update with update_scan_chains
		 * set to 0.
		 */
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
#endif
	}
}

static void scc_mgr_set_oct_out1_delay(uint32_t write_group, uint32_t delay)
{
	uint32_t read_group;

	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);

	/*
	 * Load the setting in the SCC manager
	 * Although OCT affects only write data, the OCT delay is controlled
	 * by the DQS logic block which is instantiated once per read group.
	 * For protocols where a write group consists of multiple read groups,
	 * the setting must be set multiple times.
	 */
	for (read_group = write_group * RW_MGR_MEM_IF_READ_DQS_WIDTH /
	     RW_MGR_MEM_IF_WRITE_DQS_WIDTH;
	     read_group < (write_group + 1) * RW_MGR_MEM_IF_READ_DQS_WIDTH /
	     RW_MGR_MEM_IF_WRITE_DQS_WIDTH; ++read_group)
		WRITE_SCC_OCT_OUT1_DELAY(read_group, delay);
}

static void scc_mgr_set_oct_out2_delay(uint32_t write_group, uint32_t delay)
{
	uint32_t read_group;

	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);

	/*
	 * Load the setting in the SCC manager
	 * Although OCT affects only write data, the OCT delay is controlled
	 * by the DQS logic block which is instantiated once per read group.
	 * For protocols where a write group consists
	 * of multiple read groups, the setting must be set multiple times.
	 */
	for (read_group = write_group * RW_MGR_MEM_IF_READ_DQS_WIDTH /
	     RW_MGR_MEM_IF_WRITE_DQS_WIDTH;
	     read_group < (write_group + 1) * RW_MGR_MEM_IF_READ_DQS_WIDTH /
	     RW_MGR_MEM_IF_WRITE_DQS_WIDTH; ++read_group)
		WRITE_SCC_OCT_OUT2_DELAY(read_group, delay);
}

static inline void scc_mgr_set_dqs_bypass(uint32_t write_group, uint32_t bypass)
{
	/* Load the setting in the SCC manager */
	WRITE_SCC_DQS_BYPASS(write_group, bypass);
}

inline void scc_mgr_set_dq_out1_delay(uint32_t write_group,
	uint32_t dq_in_group, uint32_t delay)
{
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);
	ALTERA_ASSERT(dq < RW_MGR_MEM_DATA_WIDTH);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DQ_OUT1_DELAY(dq_in_group, delay);
}

inline void scc_mgr_set_dq_out2_delay(uint32_t write_group,
	uint32_t dq_in_group, uint32_t delay)
{
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);
	ALTERA_ASSERT(dq < RW_MGR_MEM_DATA_WIDTH);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DQ_OUT2_DELAY(dq_in_group, delay);
}

inline void scc_mgr_set_dq_in_delay(uint32_t write_group,
	uint32_t dq_in_group, uint32_t delay)
{
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);
	ALTERA_ASSERT(dq < RW_MGR_MEM_DATA_WIDTH);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DQ_IN_DELAY(dq_in_group, delay);
}

static inline void scc_mgr_set_dq_bypass(uint32_t write_group,
	uint32_t dq_in_group, uint32_t bypass)
{
	/* Load the setting in the SCC manager */
	WRITE_SCC_DQ_BYPASS(dq_in_group, bypass);
}

static inline void scc_mgr_set_rfifo_mode(uint32_t write_group,
	uint32_t dq_in_group, uint32_t mode)
{
	/* Load the setting in the SCC manager */
	WRITE_SCC_RFIFO_MODE(dq_in_group, mode);
}

static inline void scc_mgr_set_hhp_extras(void)
{
	/*
	 * Load the fixed setting in the SCC manager
	 * bits: 0:0 = 1'b1   - dqs bypass
	 * bits: 1:1 = 1'b1   - dq bypass
	 * bits: 4:2 = 3'b001   - rfifo_mode
	 * bits: 6:5 = 2'b01  - rfifo clock_select
	 * bits: 7:7 = 1'b0  - separate gating from ungating setting
	 * bits: 8:8 = 1'b0  - separate OE from Output delay setting
	 */
	uint32_t value = (0<<8) | (0<<7) | (1<<5) | (1<<2) | (1<<1) | (1<<0);
	WRITE_SCC_HHP_EXTRAS(value);
}

static inline void scc_mgr_set_hhp_dqse_map(void)
{
	/* Load the fixed setting in the SCC manager */
	WRITE_SCC_HHP_DQSE_MAP(0);
}

static inline void scc_mgr_set_dqs_out1_delay(uint32_t write_group,
	uint32_t delay)
{
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DQS_IO_OUT1_DELAY(delay);
}

static inline void scc_mgr_set_dqs_out2_delay(uint32_t write_group,
	uint32_t delay)
{
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DQS_IO_OUT2_DELAY(delay);
}

inline void scc_mgr_set_dm_out1_delay(uint32_t write_group,
	uint32_t dm, uint32_t delay)
{
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);
	ALTERA_ASSERT(dm < RW_MGR_NUM_DM_PER_WRITE_GROUP);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DM_IO_OUT1_DELAY(dm, delay);
}

inline void scc_mgr_set_dm_out2_delay(uint32_t write_group, uint32_t dm,
	uint32_t delay)
{
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);
	ALTERA_ASSERT(dm < RW_MGR_NUM_DM_PER_WRITE_GROUP);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DM_IO_OUT2_DELAY(dm, delay);
}

static inline void scc_mgr_set_dm_in_delay(uint32_t write_group,
	uint32_t dm, uint32_t delay)
{
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);
	ALTERA_ASSERT(dm < RW_MGR_NUM_DM_PER_WRITE_GROUP);

	/* Load the setting in the SCC manager */
	WRITE_SCC_DM_IO_IN_DELAY(dm, delay);
}

static inline void scc_mgr_set_dm_bypass(uint32_t write_group, uint32_t dm,
	uint32_t bypass)
{
	/* Load the setting in the SCC manager */
	WRITE_SCC_DM_BYPASS(dm, bypass);
}

/*
 * USER Zero all DQS config
 * TODO: maybe rename to scc_mgr_zero_dqs_config (or something)
 */
void scc_mgr_zero_all(void)
{
	uint32_t i, r;

	/*
	 * USER Zero all DQS config settings, across all groups and all
	 * shadow registers
	 */
	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r +=
	     NUM_RANKS_PER_SHADOW_REG) {
		/*
		 * Strictly speaking this should be called once per group to
		 * make sure each group's delay chain is refreshed from the
		 * SCC register file, but since we're resetting all delay
		 * chains anyway, we can save some runtime by calling
		 * select_shadow_regs_for_update just once to switch rank.
		 */
		select_shadow_regs_for_update(r, 0, 1);

		for (i = 0; i < RW_MGR_MEM_IF_READ_DQS_WIDTH; i++) {
			/*
			 * The phases actually don't exist on a per-rank basis,
			 * but there's no harm updating them several times, so
			 * let's keep the code simple.
			 */
			scc_mgr_set_dqs_bus_in_delay(i, IO_DQS_IN_RESERVE);
			scc_mgr_set_dqs_en_phase(i, 0);
			scc_mgr_set_dqs_en_delay(i, 0);
		}

		for (i = 0; i < RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++) {
			scc_mgr_set_dqdqs_output_phase(i, 0);
#if ARRIAV || CYCLONEV
			/* av/cv don't have out2 */
			scc_mgr_set_oct_out1_delay(i, IO_DQS_OUT_RESERVE);
#else
			scc_mgr_set_oct_out1_delay(i, 0);
			scc_mgr_set_oct_out2_delay(i, IO_DQS_OUT_RESERVE);
#endif
		}

		/* multicast to all DQS group enables */
		IOWR_32DIRECT(SCC_MGR_DQS_ENA, 0, 0xff);
#if USE_SHADOW_REGS
		/*
		 * USER in shadow-register mode, SCC_UPDATE is done on a
		 * per-group basis unless we explicitly ask for a multicast
		 * via the group counter
		 */
		IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0, 0xFF);
#endif
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	}
}

void scc_set_bypass_mode(uint32_t write_group, uint32_t mode)
{
	/* mode = 0 : Do NOT bypass - Half Rate Mode */
	/* mode = 1 : Bypass - Full Rate Mode */

	/* only need to set once for all groups, pins, dq, dqs, dm */
	if (write_group == 0) {
		DPRINT(1, "Setting HHP Extras");
		scc_mgr_set_hhp_extras();
		DPRINT(1, "Done Setting HHP Extras");
	}
	/* multicast to all DQ enables */
	IOWR_32DIRECT(SCC_MGR_DQ_ENA, 0, 0xff);
	IOWR_32DIRECT(SCC_MGR_DM_ENA, 0, 0xff);

	/* update current DQS IO enable */
	IOWR_32DIRECT(SCC_MGR_DQS_IO_ENA, 0, 0);

	/* update the DQS logic */
	IOWR_32DIRECT(SCC_MGR_DQS_ENA, 0, write_group);

	/* hit update */
	IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
}

void scc_mgr_zero_group(uint32_t write_group, uint32_t test_begin,
	int32_t out_only)
{
	uint32_t i, r;

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r +=
		NUM_RANKS_PER_SHADOW_REG) {
		select_shadow_regs_for_update(r, write_group, 1);

		/* Zero all DQ config settings */
		for (i = 0; i < RW_MGR_MEM_DQ_PER_WRITE_DQS; i++) {
			scc_mgr_set_dq_out1_delay(write_group, i, 0);
			scc_mgr_set_dq_out2_delay(write_group, i,
						  IO_DQ_OUT_RESERVE);
			if (!out_only)
				scc_mgr_set_dq_in_delay(write_group, i, 0);
		}

		/* multicast to all DQ enables */
		IOWR_32DIRECT(SCC_MGR_DQ_ENA, 0, 0xff);

		/* Zero all DM config settings */
		for (i = 0; i < RW_MGR_NUM_DM_PER_WRITE_GROUP; i++) {
			scc_mgr_set_dm_out1_delay(write_group, i, 0);
			scc_mgr_set_dm_out2_delay(write_group, i,
						  IO_DM_OUT_RESERVE);
		}

		/* multicast to all DM enables */
		IOWR_32DIRECT(SCC_MGR_DM_ENA, 0, 0xff);

		/* zero all DQS io settings */
		if (!out_only)
			scc_mgr_set_dqs_io_in_delay(write_group, 0);
#if ARRIAV || CYCLONEV
		/* av/cv don't have out2 */
		scc_mgr_set_dqs_out1_delay(write_group, IO_DQS_OUT_RESERVE);
		scc_mgr_set_oct_out1_delay(write_group, IO_DQS_OUT_RESERVE);
		scc_mgr_load_dqs_for_write_group(write_group);
#else
		scc_mgr_set_dqs_out1_delay(write_group, 0);
		scc_mgr_set_dqs_out2_delay(write_group, IO_DQS_OUT_RESERVE);
		scc_mgr_set_oct_out1_delay(write_group, 0);
		scc_mgr_set_oct_out2_delay(write_group, IO_DQS_OUT_RESERVE);
		scc_mgr_load_dqs_for_write_group(write_group);
#endif
		/* multicast to all DQS IO enables (only 1) */
		IOWR_32DIRECT(SCC_MGR_DQS_IO_ENA, 0, 0);
#if USE_SHADOW_REGS
		/*
		 * in shadow-register mode, SCC_UPDATE is done on a per-group
		 * basis unless we explicitly ask for a multicast via the group
		 * counter
		 */
		IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0, 0xFF);
#endif
		/* hit update to zero everything */
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	}
}

/* load up dqs config settings */
static void scc_mgr_load_dqs(uint32_t dqs)
{
	IOWR_32DIRECT(SCC_MGR_DQS_ENA, 0, dqs);
}

static void scc_mgr_load_dqs_for_write_group(uint32_t write_group)
{
	uint32_t read_group;

	/*
	 * Although OCT affects only write data, the OCT delay is controlled
	 * by the DQS logic block which is instantiated once per read group.
	 * For protocols where a write group consists of multiple read groups,
	 * the setting must be scanned multiple times.
	 */
	for (read_group = write_group * RW_MGR_MEM_IF_READ_DQS_WIDTH /
	     RW_MGR_MEM_IF_WRITE_DQS_WIDTH;
	     read_group < (write_group + 1) * RW_MGR_MEM_IF_READ_DQS_WIDTH /
	     RW_MGR_MEM_IF_WRITE_DQS_WIDTH; ++read_group)
		IOWR_32DIRECT(SCC_MGR_DQS_ENA, 0, read_group);
}

/* load up dqs io config settings */
static void scc_mgr_load_dqs_io(void)
{
	IOWR_32DIRECT(SCC_MGR_DQS_IO_ENA, 0, 0);
}

/* load up dq config settings */
static void scc_mgr_load_dq(uint32_t dq_in_group)
{
	IOWR_32DIRECT(SCC_MGR_DQ_ENA, 0, dq_in_group);
}

/* load up dm config settings */
static void scc_mgr_load_dm(uint32_t dm)
{
	IOWR_32DIRECT(SCC_MGR_DM_ENA, 0, dm);
}

/*
 * apply and load a particular input delay for the DQ pins in a group
 * group_bgn is the index of the first dq pin (in the write group)
 */
static void scc_mgr_apply_group_dq_in_delay(uint32_t write_group,
	uint32_t group_bgn, uint32_t delay)
{
	uint32_t i, p;

	for (i = 0, p = group_bgn; i < RW_MGR_MEM_DQ_PER_READ_DQS; i++, p++) {
		scc_mgr_set_dq_in_delay(write_group, p, delay);
		scc_mgr_load_dq(p);
	}
}

/* apply and load a particular output delay for the DQ pins in a group */
static void scc_mgr_apply_group_dq_out1_delay(uint32_t write_group,
	uint32_t group_bgn, uint32_t delay1)
{
	uint32_t i, p;

	for (i = 0, p = group_bgn; i < RW_MGR_MEM_DQ_PER_WRITE_DQS; i++, p++) {
		scc_mgr_set_dq_out1_delay(write_group, i, delay1);
		scc_mgr_load_dq(i);
	}
}

/* apply and load a particular output delay for the DM pins in a group */
static void scc_mgr_apply_group_dm_out1_delay(uint32_t write_group,
	uint32_t delay1)
{
	uint32_t i;

	for (i = 0; i < RW_MGR_NUM_DM_PER_WRITE_GROUP; i++) {
		scc_mgr_set_dm_out1_delay(write_group, i, delay1);
		scc_mgr_load_dm(i);
	}
}


/* apply and load delay on both DQS and OCT out1 */
static void scc_mgr_apply_group_dqs_io_and_oct_out1(uint32_t write_group,
	uint32_t delay)
{
	scc_mgr_set_dqs_out1_delay(write_group, delay);
	scc_mgr_load_dqs_io();

	scc_mgr_set_oct_out1_delay(write_group, delay);
	scc_mgr_load_dqs_for_write_group(write_group);
}

/*
 * set delay on both DQS and OCT out1 by incrementally changing
 * the settings one dtap at a time towards the target value, to avoid
 * breaking the lock of the DLL/PLL on the memory device.
 */
static void scc_mgr_set_group_dqs_io_and_oct_out1_gradual(uint32_t write_group,
	uint32_t delay)
{
	uint32_t d = READ_SCC_DQS_IO_OUT1_DELAY();

	while (d > delay) {
		--d;
		scc_mgr_apply_group_dqs_io_and_oct_out1(write_group, d);
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
		if (QDRII)
			rw_mgr_mem_dll_lock_wait();
	}
	while (d < delay) {
		++d;
		scc_mgr_apply_group_dqs_io_and_oct_out1(write_group, d);
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
		if (QDRII)
			rw_mgr_mem_dll_lock_wait();
	}
}

/* apply a delay to the entire output side: DQ, DM, DQS, OCT */
static void scc_mgr_apply_group_all_out_delay(uint32_t write_group,
	uint32_t group_bgn, uint32_t delay)
{
	/* dq shift */
	scc_mgr_apply_group_dq_out1_delay(write_group, group_bgn, delay);

	/* dm shift */
	scc_mgr_apply_group_dm_out1_delay(write_group, delay);

	/* dqs and oct shift */
	scc_mgr_apply_group_dqs_io_and_oct_out1(write_group, delay);
}

/*
 * USER apply a delay to the entire output side (DQ, DM, DQS, OCT)
 * and to all ranks
 */
static void scc_mgr_apply_group_all_out_delay_all_ranks(uint32_t write_group,
	uint32_t group_bgn, uint32_t delay)
{
	uint32_t r;

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS;
	     r += NUM_RANKS_PER_SHADOW_REG) {
		select_shadow_regs_for_update(r, write_group, 1);
		scc_mgr_apply_group_all_out_delay(write_group, group_bgn,
						  delay);
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	}
}

/* apply a delay to the entire output side: DQ, DM, DQS, OCT */
static void scc_mgr_apply_group_all_out_delay_add(uint32_t write_group,
	uint32_t group_bgn, uint32_t delay)
{
	uint32_t i, p, new_delay;

	/* dq shift */
	for (i = 0, p = group_bgn; i < RW_MGR_MEM_DQ_PER_WRITE_DQS; i++, p++) {
		new_delay = READ_SCC_DQ_OUT2_DELAY(i);
		new_delay += delay;

		if (new_delay > IO_IO_OUT2_DELAY_MAX) {
			DPRINT(1, "%s(%u, %u, %u) DQ[%u,%u]: %u > %lu => %lu",
			       __func__, write_group, group_bgn, delay, i, p,
			       new_delay,
			       (long unsigned int)IO_IO_OUT2_DELAY_MAX,
			       (long unsigned int)IO_IO_OUT2_DELAY_MAX);
			new_delay = IO_IO_OUT2_DELAY_MAX;
		}

		scc_mgr_set_dq_out2_delay(write_group, i, new_delay);
		scc_mgr_load_dq(i);
	}

	/* dm shift */
	for (i = 0; i < RW_MGR_NUM_DM_PER_WRITE_GROUP; i++) {
		new_delay = READ_SCC_DM_IO_OUT2_DELAY(i);
		new_delay += delay;

		if (new_delay > IO_IO_OUT2_DELAY_MAX) {
			DPRINT(1, "%s(%u, %u, %u) DM[%u]: %u > %lu => %lu",
			       __func__, write_group, group_bgn, delay, i,
			       new_delay,
			       (long unsigned int)IO_IO_OUT2_DELAY_MAX,
			       (long unsigned int)IO_IO_OUT2_DELAY_MAX);
			new_delay = IO_IO_OUT2_DELAY_MAX;
		}

		scc_mgr_set_dm_out2_delay(write_group, i, new_delay);
		scc_mgr_load_dm(i);
	}

	/* dqs shift */
	new_delay = READ_SCC_DQS_IO_OUT2_DELAY();
	new_delay += delay;

	if (new_delay > IO_IO_OUT2_DELAY_MAX) {
		DPRINT(1, "%s(%u, %u, %u) DQS: %u > %d => %d;"
			" adding %u to OUT1",
			__func__, write_group, group_bgn, delay,
			new_delay, IO_IO_OUT2_DELAY_MAX, IO_IO_OUT2_DELAY_MAX,
			new_delay - IO_IO_OUT2_DELAY_MAX);
		scc_mgr_set_dqs_out1_delay(write_group, new_delay -
					   IO_IO_OUT2_DELAY_MAX);
		new_delay = IO_IO_OUT2_DELAY_MAX;
	}

	scc_mgr_set_dqs_out2_delay(write_group, new_delay);
	scc_mgr_load_dqs_io();

	/* oct shift */
	new_delay = READ_SCC_OCT_OUT2_DELAY(write_group);
	new_delay += delay;

	if (new_delay > IO_IO_OUT2_DELAY_MAX) {
		DPRINT(1, "%s(%u, %u, %u) DQS: %u > %d => %d;"
		       " adding %u to OUT1",
		       __func__, write_group, group_bgn, delay,
		       new_delay, IO_IO_OUT2_DELAY_MAX, IO_IO_OUT2_DELAY_MAX,
		       new_delay - IO_IO_OUT2_DELAY_MAX);
		scc_mgr_set_oct_out1_delay(write_group, new_delay -
					   IO_IO_OUT2_DELAY_MAX);
		new_delay = IO_IO_OUT2_DELAY_MAX;
	}

	scc_mgr_set_oct_out2_delay(write_group, new_delay);
	scc_mgr_load_dqs_for_write_group(write_group);
}

/*
 * USER apply a delay to the entire output side (DQ, DM, DQS, OCT)
 * and to all ranks
 */
static void scc_mgr_apply_group_all_out_delay_add_all_ranks(
	uint32_t write_group, uint32_t group_bgn, uint32_t delay)
{
	uint32_t r;

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS;
		r += NUM_RANKS_PER_SHADOW_REG) {
		select_shadow_regs_for_update(r, write_group, 1);
		scc_mgr_apply_group_all_out_delay_add(write_group,
						      group_bgn, delay);
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	}
}

static inline void scc_mgr_spread_out2_delay_all_ranks(uint32_t write_group,
	uint32_t test_bgn)
{
#if STRATIXV || ARRIAVGZ
	uint32_t found;
	uint32_t i;
	uint32_t p;
	uint32_t d;
	uint32_t r;

	const uint32_t delay_step = IO_IO_OUT2_DELAY_MAX /
		(RW_MGR_MEM_DQ_PER_WRITE_DQS-1);
		/* we start at zero, so have one less dq to devide among */

	TRACE_FUNC("(%lu,%lu)", write_group, test_bgn);

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS;
	     r += NUM_RANKS_PER_SHADOW_REG) {
		select_shadow_regs_for_update(r, write_group, 1);
		for (i = 0, p = test_bgn, d = 0;
		     i < RW_MGR_MEM_DQ_PER_WRITE_DQS;
		     i++, p++, d += delay_step) {
			scc_mgr_set_dq_out2_delay(write_group, i, d);
			scc_mgr_load_dq(i);
		}
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	}
#endif
}

#if DDR3
/* optimization used to recover some slots in ddr3 inst_rom */
/* could be applied to other protocols if we wanted to */
static void set_jump_as_return(void)
{
	/*
	 * to save space, we replace return with jump to special shared
	 * RETURN instruction so we set the counter to large value so that
	 * we always jump
	 */
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0xFF);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_RETURN);
}
#endif

/*
 * should always use constants as argument to ensure all computations are
 * performed at compile time
 */
static inline void delay_for_n_mem_clocks(const uint32_t clocks)
{
	uint32_t afi_clocks;
	uint8_t inner = 0;
	uint8_t outer = 0;
	uint16_t c_loop = 0;

	TRACE_FUNC("clocks=%u ... start", clocks);

	afi_clocks = (clocks + AFI_RATE_RATIO-1) / AFI_RATE_RATIO;
	/* scale (rounding up) to get afi clocks */

	/*
	 * Note, we don't bother accounting for being off a little bit
	 * because of a few extra instructions in outer loops
	 * Note, the loops have a test at the end, and do the test before
	 * the decrement, and so always perform the loop
	 * 1 time more than the counter value
	 */
	if (afi_clocks == 0) {
		;
	} else if (afi_clocks <= 0x100) {
		inner = afi_clocks-1;
		outer = 0;
		c_loop = 0;
	} else if (afi_clocks <= 0x10000) {
		inner = 0xff;
		outer = (afi_clocks-1) >> 8;
		c_loop = 0;
	} else {
		inner = 0xff;
		outer = 0xff;
		c_loop = (afi_clocks-1) >> 16;
	}

	/*
	 * rom instructions are structured as follows:
	 *
	 *    IDLE_LOOP2: jnz cntr0, TARGET_A
	 *    IDLE_LOOP1: jnz cntr1, TARGET_B
	 *                return
	 *
	 * so, when doing nested loops, TARGET_A is set to IDLE_LOOP2, and
	 * TARGET_B is set to IDLE_LOOP2 as well
	 *
	 * if we have no outer loop, though, then we can use IDLE_LOOP1 only,
	 * and set TARGET_B to IDLE_LOOP1 and we skip IDLE_LOOP2 entirely
	 *
	 * a little confusing, but it helps save precious space in the inst_rom
	 * and sequencer rom and keeps the delays more accurate and reduces
	 * overhead
	 */
	if (afi_clocks <= 0x100) {
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0,
			      SKIP_DELAY_LOOP_VALUE_OR_ZERO(inner));
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0, __RW_MGR_IDLE_LOOP1);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_IDLE_LOOP1);
	} else {
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0,
			      SKIP_DELAY_LOOP_VALUE_OR_ZERO(inner));
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0,
			      SKIP_DELAY_LOOP_VALUE_OR_ZERO(outer));

		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_IDLE_LOOP2);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0, __RW_MGR_IDLE_LOOP2);

		/* hack to get around compiler not being smart enough */
		if (afi_clocks <= 0x10000) {
			/* only need to run once */
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_IDLE_LOOP2);
		} else {
			do {
				IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
					      __RW_MGR_IDLE_LOOP2);
			} while (c_loop-- != 0);
		}
	}
	TRACE_FUNC("clocks=%u ... end", clocks);
}

/*
 * should always use constants as argument to ensure all computations are
 * performed at compile time
 */
static inline void delay_for_n_ns(const uint32_t nanoseconds)
{
	TRACE_FUNC("nanoseconds=%u ... end", nanoseconds);
	delay_for_n_mem_clocks((1000*nanoseconds) /
			       (1000000/AFI_CLK_FREQ) * AFI_RATE_RATIO);
}

#if RLDRAM3
/*
 * Special routine to recover memory device from illegal state after
 * ck/dk relationship is potentially violated.
 */
static inline void recover_mem_device_after_ck_dqs_violation(void)
{
	/*
	 * Issue MRS0 command. For some reason this is required once we
	 * violate tCKDK. Without this all subsequent write tests will
	 * fail even with known good delays.
	 */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS0);

	/* Wait MRSC */
	delay_for_n_mem_clocks(12);
}
#else
/*
 * Special routine to recover memory device from illegal state after
 * ck/dqs relationship is violated.
 */
static inline void recover_mem_device_after_ck_dqs_violation(void)
{
	/* Current protocol doesn't require any special recovery */
}
#endif

#if (LRDIMM && DDR3)
/* Routine to program specific LRDIMM control words. */
static void rw_mgr_lrdimm_rc_program(uint32_t fscw, uint32_t rc_addr,
	uint32_t rc_val)
{
	uint32_t i;
	const uint32_t AC_BASE_CONTENT = __RW_MGR_CONTENT_ac_rdimm;
	/* These values should be dynamically loaded instead of hard-coded */
	const uint32_t AC_ADDRESS_POSITION = 0x0;
	const uint32_t AC_BANK_ADDRESS_POSITION = 0xD;
	uint32_t ac_content;
	uint32_t lrdimm_cs_msk = RW_MGR_RANK_NONE;

	TRACE_FUNC();

	/* Turn on only CS0 and CS1 for each DIMM. */
	for (i = 0; i < RW_MGR_MEM_CHIP_SELECT_WIDTH;
	     i += RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM)
		lrdimm_cs_msk &= (~(3 << i));

	IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, lrdimm_cs_msk);

	/* Program the fscw first (RC7), followed by the actual value */
	for (i = 0; i < 2; i++)	{
		uint32_t addr;
		uint32_t val;

		addr = (i == 0) ? 7 : rc_addr;
		val = (i == 0) ? fscw : rc_val;

		ac_content =
			AC_BASE_CONTENT |
			/* Word address */
			((addr & 0x7) << AC_ADDRESS_POSITION) |
			(((addr >> 3) & 0x1) <<
			(AC_BANK_ADDRESS_POSITION + 2)) |
			/* Configuration Word */
			(((val >> 2) & 0x3) << (AC_BANK_ADDRESS_POSITION)) |
			((val & 0x3) << (AC_ADDRESS_POSITION + 3));

		/* Override the AC row with the RDIMM command */
		IOWR_32DIRECT(BASE_RW_MGR, 0x1C00 + (__RW_MGR_ac_rdimm << 2),
			      ac_content);

		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_RDIMM_CMD);
	}

	/*
	 * USER The following registers require a delay of tSTAB (6us)
	 * USER for proper functionality.
	 * USER F0RC2, F0RC10, F0RC11, F1RC8, F1RC11-F1RC15
	 * USER Note that it is only necessary to wait tSTAB after all of these
	 * USER control words have been written, not after each one. Only
	 * USER F0RC0-F0RC15
	 * USER are guaranteed to be written (and in order), but F1* are not so
	 * USER wait after each.
	 */
	if (((fscw == 0) && (rc_addr == 11)) ||
	    ((fscw == 1) && (rc_addr >= 8)))
		delay_for_n_ns(6000);
}
#endif
#if (RDIMM || LRDIMM) && DDR3
void rw_mgr_rdimm_initialize(void)
{
	uint32_t i;
	uint32_t conf_word;
#if RDIMM
	const uint32_t AC_BASE_CONTENT = __RW_MGR_CONTENT_ac_rdimm;
	/* These values should be dynamically loaded instead of hard-coded */
	const uint32_t AC_ADDRESS_POSITION = 0x0;
	const uint32_t AC_BANK_ADDRESS_POSITION = 0xD;
	uint32_t ac_content;
#endif

	TRACE_FUNC();

	/*
	 * RDIMM registers are programmed by writing 16 configuration words
	 * 1. An RDIMM command is a NOP with all CS asserted
	 * 2. The 4-bit address of the configuration words is
	 *    * { mem_ba[2] , mem_a[2] , mem_a[1] , mem_a[0] }
	 * 3. The 4-bit configuration word is
	 *    * { mem_ba[1] , mem_ba[0] , mem_a[4] , mem_a[3] }
	 */

#if RDIMM
	/* Turn on all ranks */
	IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, RW_MGR_RANK_ALL);
#endif

	for (i = 0; i < 16; i++)	{
		if (i < 8)
			conf_word = (RDIMM_CONFIG_WORD_LOW >> (i * 4)) & 0xF;
		else
			conf_word = (RDIMM_CONFIG_WORD_HIGH >> ((i - 8) * 4))
				& 0xF;

#if RDIMM
		ac_content =
			AC_BASE_CONTENT |
			/* Word address */
			((i & 0x7) << AC_ADDRESS_POSITION) |
			(((i >> 3) & 0x1) << (AC_BANK_ADDRESS_POSITION + 2)) |
			/* Configuration Word */
			(((conf_word >> 2) & 0x3) <<
			(AC_BANK_ADDRESS_POSITION)) |
			((conf_word & 0x3) << (AC_ADDRESS_POSITION + 3));

		/* Override the AC row with the RDIMM command */
		IOWR_32DIRECT(BASE_RW_MGR, 0x1C00 +
			      (__RW_MGR_ac_rdimm << 2), ac_content);

		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_RDIMM_CMD);
		/*
		 * When sending the RC2 or RC10 word, tSTAB time must
		 * elapse before the next command
		 * is sent out. tSTAB is currently hard-coded to 6us.
		 */
		if ((i == 2) || (i == 10))
			/* tSTAB = 6 us */
			delay_for_n_ns(6000);
#endif
#if LRDIMM
		/* USER Program configuration word with FSCW set to zero. */
		rw_mgr_lrdimm_rc_program(0, i, conf_word);
#endif
	}
}
#else
void rw_mgr_rdimm_initialize(void) { }
#endif

#if DDR3

#if LRDIMM
uint32_t ddr3_mirror_mrs_cmd(uint32_t bit_vector)
{
	/*
	 * This function performs address mirroring of an AC ROM command, which
	 * requires swapping the following DDR3 bits:
	 *     A[3] <=> A[4]
	 *     A[5] <=> A[6]
	 *     A[7] <=> A[8]
	 *    BA[0] <=>BA[1]
	 * We assume AC_ROM_ENTRY = {BA[2:0], A[15:0]}.
	 */
	uint32_t unchanged_bits;
	uint32_t mask_a;
	uint32_t mask_b;
	uint32_t retval;

	unchanged_bits = (~(DDR3_AC_MIRR_MASK | (DDR3_AC_MIRR_MASK << 1)))
			  & bit_vector;
	mask_a = DDR3_AC_MIRR_MASK & bit_vector;
	mask_b = (DDR3_AC_MIRR_MASK << 1) & bit_vector;

	retval = unchanged_bits | (mask_a << 1) | (mask_b >> 1);

	return retval;
}

void rtt_change_MRS1_MRS2_NOM_WR(uint32_t prev_ac_mr , uint32_t odt_ac_mr,
	uint32_t mirr_on, uint32_t mr_cmd) {
	/*
	 * This function updates the ODT-specific Mode Register bits
	 * (MRS1 or MRS2) in the AC ROM.
	 * Parameters:  prev_ac_mr - Original, *un-mirrored* AC ROM Entry
	 *              odt_ac_mr  - ODT bits to update (un-mirrored)
	 *              mirr_on    - boolean flag indicating if the regular
	 * 			     or mirrored entry is updated
	 *              mr_cmd     - Mode register command (only MR1 and MR2
	 *			     are supported for DDR3)
	 */
	uint32_t new_ac_mr;
	uint32_t ac_rom_entry = 0;
	uint32_t ac_rom_mask;

	switch (mr_cmd) {
		case 1: {
			/* USER MRS1 = RTT_NOM, RTT_DRV */
			ac_rom_mask = DDR3_MR1_ODT_MASK;
			ac_rom_entry = mirr_on ? (0x1C00 |
				(__RW_MGR_ac_mrs1_mirr << 2))
				: (0x1C00 | (__RW_MGR_ac_mrs1 << 2));
		} break;
		case 2: {
			/* USER MRS2 = RTT_WR */
			ac_rom_mask = DDR3_MR2_ODT_MASK;
			ac_rom_entry = mirr_on ? (0x1C00 |
				(__RW_MGR_ac_mrs2_mirr << 2))
				: (0x1C00 | (__RW_MGR_ac_mrs2 << 2));
		} break;
	}

	/* USER calculate new AC values and update ROM */
	new_ac_mr  = odt_ac_mr;
	new_ac_mr |= (prev_ac_mr & ac_rom_mask);
	if (mirr_on)
		new_ac_mr = ddr3_mirror_mrs_cmd(new_ac_mr);
	IOWR_32DIRECT(BASE_RW_MGR, ac_rom_entry, new_ac_mr);
}
#endif /*LRDIMM */

void rw_mgr_mem_initialize(void)
{
	uint32_t r;

#if LRDIMM
	uint32_t rtt_nom;
	uint32_t rtt_drv;
	uint32_t rtt_wr;
#endif /* LRDIMM */

	TRACE_FUNC();

	/* The reset / cke part of initialization is broadcasted to all ranks */
	IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, RW_MGR_RANK_ALL);

	/*
	 * Here's how you load register for a loop
	 * Counters are located @ 0x800
	 * Jump address are located @ 0xC00
	 * For both, registers 0 to 3 are selected using bits 3 and 2, like
	 * in 0x800, 0x804, 0x808, 0x80C and 0xC00, 0xC04, 0xC08, 0xC0C
	 * I know this ain't pretty, but Avalon bus throws away the 2 least
	 * significant bits
	 */

	/* start with memory RESET activated */

	/* tINIT = 200us */

	/*
	 * 200us @ 266MHz (3.75 ns) ~ 54000 clock cycles
	 * If a and b are the number of iteration in 2 nested loops
	 * it takes the following number of cycles to complete the operation:
	 * number_of_cycles = ((2 + n) * a + 2) * b
	 * where n is the number of instruction in the inner loop
	 * One possible solution is n = 0 , a = 256 , b = 106 => a = FF,
	 * b = 6A
	 */

	/* Load counters */
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(SEQ_TINIT_CNTR0_VAL));
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(SEQ_TINIT_CNTR1_VAL));
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(SEQ_TINIT_CNTR2_VAL));

	/* Load jump address */
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_INIT_RESET_0_CKE_0);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0, __RW_MGR_INIT_RESET_0_CKE_0);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0, __RW_MGR_INIT_RESET_0_CKE_0);

	/* Execute count instruction */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_INIT_RESET_0_CKE_0);

	/* indicate that memory is stable */
	IOWR_32DIRECT(PHY_MGR_RESET_MEM_STBL, 0, 1);

	/*
	 * transition the RESET to high
	 * Wait for 500us
	 */

	/*
	 * 500us @ 266MHz (3.75 ns) ~ 134000 clock cycles
	 * If a and b are the number of iteration in 2 nested loops
	 * it takes the following number of cycles to complete the operation
	 * number_of_cycles = ((2 + n) * a + 2) * b
	 * where n is the number of instruction in the inner loop
	 * One possible solution is n = 2 , a = 131 , b = 256 => a = 83,
	 * b = FF
	 */

	/* Load counters */
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(SEQ_TRESET_CNTR0_VAL));
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(SEQ_TRESET_CNTR1_VAL));
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(SEQ_TRESET_CNTR2_VAL));

	/* Load jump address */
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_INIT_RESET_1_CKE_0);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0, __RW_MGR_INIT_RESET_1_CKE_0);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0, __RW_MGR_INIT_RESET_1_CKE_0);

	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_INIT_RESET_1_CKE_0);

	/* bring up clock enable */

	/* tXRP < 250 ck cycles */
	delay_for_n_mem_clocks(250);

#ifdef RDIMM
	/*
	 * USER initialize RDIMM buffer so MRS and RZQ Calibrate commands will
	 * USER be propagated to discrete memory devices
	 */
	rw_mgr_rdimm_initialize();
#endif

#if LRDIMM
	/*
	 * USER initialize LRDIMM MB so MRS and RZQ Calibrate commands will be
	 * USER propagated to all sub-ranks.  Per LRDIMM spec, all LRDIMM ranks
	 * USER must have
	 * USER RTT_WR set, but only physical ranks 0 and 1 should have RTT_NOM
	 * USER set.
	 * USER Therefore RTT_NOM=0 is broadcast to all ranks, and the non-zero
	 * USER value is
	 * USER programmed directly into Ranks 0 and 1 using physical MRS
	 * targetting.
	 */
	rw_mgr_rdimm_initialize();

	rtt_nom = LRDIMM_SPD_MR_RTT_NOM(LRDIMM_SPD_MR);
	rtt_drv = LRDIMM_SPD_MR_RTT_DRV(LRDIMM_SPD_MR);
	rtt_wr  = LRDIMM_SPD_MR_RTT_WR(LRDIMM_SPD_MR);

	/* Configure LRDIMM to broadcast LRDIMM MRS commands to all ranks */
	rw_mgr_lrdimm_rc_program(0, 14, 0x9); /*broadcast mode */

	/*
	 * USER Update contents of AC ROM with new RTT WR, DRV values only
	 * (NOM = Off)
	 */
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs1, rtt_drv, 0, 1);
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs1, rtt_drv, 1, 1);
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs2, rtt_wr,  0, 2);
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs2, rtt_wr,  1, 2);
#endif
#if RDIMM
	/*
	 * USER initialize RDIMM buffer so MRS and RZQ Calibrate commands
	 * USER will be
	 * USER propagated to discrete memory devices
	 */
	rw_mgr_rdimm_initialize();
#endif

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r]) {
			/* request to skip the rank */

			continue;
		}

		/* set rank */
#if MRS_MIRROR_PING_PONG_ATSO
		/* Special case */
		/* SIDE 0 */
		set_rank_and_odt_mask_for_ping_pong_atso(0,
							 RW_MGR_ODT_MODE_OFF);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS2);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS3);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS1);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_MRS0_DLL_RESET);

		/* SIDE 1 */
		set_rank_and_odt_mask_for_ping_pong_atso(1,
							 RW_MGR_ODT_MODE_OFF);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS2_MIRR);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS3_MIRR);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS1_MIRR);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_MRS0_DLL_RESET_MIRR);

		/* Unmask all CS */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);
#else
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/*
		 * USER Use Mirror-ed commands for odd ranks if address
		 * mirrorring is on
		 */
		if ((RW_MGR_MEM_ADDRESS_MIRRORING >> r) & 0x1) {
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS2_MIRR);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS3_MIRR);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS1_MIRR);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS0_DLL_RESET_MIRR);
		} else {
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS2);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS3);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS1);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS0_DLL_RESET);
		}
#endif
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_ZQCL);

		/* tZQinit = tDLLK = 512 ck cycles */
		delay_for_n_mem_clocks(512);
	}
#if LRDIMM
	/*
	 * USER Configure LRDIMM to target physical ranks decoded by
	 * RM bits only (ranks 0,1 only)
	 */
	rw_mgr_lrdimm_rc_program(0, 14, 0xD);
	/* USER update AC ROM MR1 entry to include RTT_NOM */
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs1,
				    (rtt_drv|rtt_nom), 0, 1);
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs1,
				    (rtt_drv|rtt_nom), 1, 1);
	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r]) {
			/* USER request to skip the rank */
			continue;
		}

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/*
		 * Use Mirror-ed commands for odd ranks if address
		 * mirrorring is on
		 */
		if ((RW_MGR_MEM_ADDRESS_MIRRORING >> r) & 0x1) {
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS1_MIRR);
			delay_for_n_mem_clocks(4);
		} else {
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS1);
			delay_for_n_mem_clocks(4);
		}
	}

	/*
	 * USER Initiate LRDIMM MB->Physical Rank training here
	 * USER -> Set minimum skew mode for levelling - F3RC6 = 0001
	 */
	rw_mgr_lrdimm_rc_program(3, 6, 0x1);
	/*
	 * USER -> Set error status output in register F2RC3 for debugging
	 * purposes
	 */
	rw_mgr_lrdimm_rc_program(2, 3, 0x8);

#ifdef LRDIMM_EXT_CONFIG_ARRAY
	/* USER Configure LRDIMM ODT/Drive parameters using SPD information */
	{
		static const uint8_t lrdimm_cfg_array[][3] =
			LRDIMM_EXT_CONFIG_ARRAY;
		uint32_t cfg_reg_ctr;

		for (cfg_reg_ctr = 0; cfg_reg_ctr < (sizeof(lrdimm_cfg_array) /
			sizeof(lrdimm_cfg_array[0])); cfg_reg_ctr++) {
			uint32_t lrdimm_fp  = (uint32_t)lrdimm_cfg_array
				[cfg_reg_ctr][0];
			uint32_t lrdimm_rc  = (uint32_t)lrdimm_cfg_array
				[cfg_reg_ctr][1];
			uint32_t lrdimm_val = (uint32_t)lrdimm_cfg_array
				[cfg_reg_ctr][2];

			rw_mgr_lrdimm_rc_program(lrdimm_fp, lrdimm_rc,
						 lrdimm_val);
		}
	}
#endif /* LRDIMM_EXT_CONFIG_ARRAY */

	/* USER -> Initiate MB->DIMM training on the LRDIMM */
	rw_mgr_lrdimm_rc_program(0, 12, 0x2);
#if (!STATIC_SKIP_DELAY_LOOPS)
	/*
	 * USER Wait for max(tcal) * number of physical ranks. Tcal is approx.
	 * 10ms.
	 */
	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS *
	     RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM; r++)
		delay_for_n_ns(40000000UL);
#endif /* !STATIC_SKIP_DELAY_LOOPS */
	/* USER Place MB back in normal operating mode */
	rw_mgr_lrdimm_rc_program(0, 12, 0x0);
#endif /* LRDIMM */
}
#endif /* DDR3 */

#if DDR2
void rw_mgr_mem_initialize(void)
{
	uint32_t r;

	TRACE_FUNC();

	/* *** NOTE *** */
	/* The following STAGE (n) notation refers to the corresponding
	stage in the Micron datasheet */

	/*
	 *Here's how you load register for a loop
	 * Counters are located @ 0x800
	 * Jump address are located @ 0xC00
	 * For both, registers 0 to 3 are selected using bits 3 and 2,
	 like in
	 * 0x800, 0x804, 0x808, 0x80C and 0xC00, 0xC04, 0xC08, 0xC0C
	 * I know this ain't pretty, but Avalon bus throws away the 2 least
	 significant bits
	 */

	/* *** STAGE (1, 2, 3) *** */

	/* start with CKE low */

	/* tINIT = 200us */

	/* 200us @ 300MHz (3.33 ns) ~ 60000 clock cycles
	* If a and b are the number of iteration in 2 nested loops
	* it takes the following number of cycles to complete the operation:
	* number_of_cycles = ((2 + n) * b + 2) * a
	* where n is the number of instruction in the inner loop
	* One possible solution is n = 0 , a = 256 , b = 118 => a = FF,
	* b = 76
	*/

	/*TODO: Need to manage multi-rank */

	/* Load counters */
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(SEQ_TINIT_CNTR0_VAL));
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(SEQ_TINIT_CNTR1_VAL));
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(SEQ_TINIT_CNTR2_VAL));

	/* Load jump address */
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_INIT_CKE_0);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0, __RW_MGR_INIT_CKE_0);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0, __RW_MGR_INIT_CKE_0);

	/* Execute count instruction */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_INIT_CKE_0);

	/* indicate that memory is stable */
	IOWR_32DIRECT(PHY_MGR_RESET_MEM_STBL, 0, 1);

	/* Bring up CKE */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_NOP);

	/* *** STAGE (4) */
	delay_for_n_ns(400);

	/* Multi-rank section begins here */
	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/*
		 * * **** *
		 * * NOTE *
		 * * **** *
		 * The following commands must be spaced by tMRD or tRPA
		 * which are in the order
		 * of 2 to 4 full rate cycles. This is peanuts in the
		 * NIOS domain, so for now
		 * we can avoid redundant wait loops
		 */

		/*
		 * Possible FIXME BEN: for HHP, we need to add delay loops
		 * to be sure although, the sequencer write interface by itself
		 * likely has enough delay
		 */

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_PRECHARGE_ALL);

		/* *** STAGE (5) */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_EMR2);

		/* *** STAGE (6) */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_EMR3);

		/* *** STAGE (7) */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_EMR);

		/* *** STAGE (8) */
		/* DLL reset */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_MR_DLL_RESET);

		/* *** STAGE (9) */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_PRECHARGE_ALL);

		/* *** STAGE (10) */

		/* Issue 2 refresh commands spaced by tREF */

		/* First REFRESH */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_REFRESH);

		/* tREF = 200ns */
		delay_for_n_ns(200);

		/* Second REFRESH */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_REFRESH);

		/* Second idle loop */
		delay_for_n_ns(200);

		/* *** STAGE (11) */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_MR_CALIB);

		/* *** STAGE (12) */
		/* OCD defaults */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_EMR_OCD_ENABLE);

		/* *** STAGE (13) */
		/* OCD exit */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_EMR);

		/* *** STAGE (14) */

		/*
		 * The memory is now initialized. Before being able to
		 * use it, we must still
		 * wait for the DLL to lock, 200 clock cycles after it
		 * was reset @ STAGE (8).
		 * Since we cannot keep track of time in any other way,
		 * let's start counting from now
		 */
		delay_for_n_mem_clocks(200);
	}
}
#endif /* DDR2 */

#if LPDDR2
void rw_mgr_mem_initialize(void)
{
	uint32_t r;

	/*
	 * The following STAGE (n) notation refers to the corresponding
	 * stage in the Micron datasheet
	 */

	/*
	 * Here's how you load register for a loop
	 * Counters are located @ 0x800
	 * Jump address are located @ 0xC00
	 * For both, registers 0 to 3 are selected using bits 3 and 2,
	 * like in
	 * 0x800, 0x804, 0x808, 0x80C and 0xC00, 0xC04, 0xC08, 0xC0C
	 * I know this ain't pretty, but Avalon bus throws away the 2 least
	 * significant bits
	 */


	/* *** STAGE (1, 2, 3) *** */

	/* start with CKE low */

	/* tINIT1 = 100ns */

	/*
	 * 100ns @ 300MHz (3.333 ns) ~ 30 cycles
	 * If a is the number of iteration in a loop
	 * it takes the following number of cycles to complete the operation
	 * number_of_cycles = (2 + n) * a
	 * where n is the number of instruction in the inner loop
	 * One possible solution is n = 0 , a = 15 => a = 0x10
	 */

	/* Load counter */
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(0x10));

	/* Load jump address */
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_INIT_CKE_0);

	/* Execute count instruction */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_INIT_CKE_0);

	/* tINIT3 = 200us */
	delay_for_n_ns(200000);

	/* indicate that memory is stable */
	IOWR_32DIRECT(PHY_MGR_RESET_MEM_STBL, 0, 1);

	/* Multi-rank section begins here */
	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/* MRW RESET */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR63_RESET);
	}

	/* tINIT5 = 10us */
	delay_for_n_ns(10000);

	/* Multi-rank section begins here */
	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/*
		 * MRW ZQC
		 * Note: We cannot calibrate other ranks when the current rank
		 * is calibrating for tZQINIT
		 */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR10_ZQC);

		/* tZQINIT = 1us */
		delay_for_n_ns(1000);

		/*
		 * * **** *
		 * * NOTE *
		 * * **** *
		 * The following commands must be spaced by tMRW which is
		 * in the order
		 * of 3 to 5 full rate cycles. This is peanuts in the NIOS
		 * domain, so for now
		 * we can avoid redundant wait loops
		 */

		/* MRW MR1 */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR1_CALIB);

		/* MRW MR2 */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR2);

		/* MRW MR3 */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR3);
	}
}
#endif /* LPDDR2 */

#if LPDDR1
void rw_mgr_mem_initialize(void)
{
	uint32_t r;

	TRACE_FUNC();

	/* *** NOTE *** */
	/*
	 * The following STAGE (n) notation refers to the corresponding
	 * stage in the Micron datasheet
	 */

	/*
	 * Here's how you load register for a loop
	 * Counters are located @ 0x800
	 * Jump address are located @ 0xC00
	 * For both, registers 0 to 3 are selected using bits 3 and 2, like
	 * in 0x800, 0x804, 0x808, 0x80C and 0xC00, 0xC04, 0xC08, 0xC0C
	 * I know this ain't pretty, but Avalon bus throws away the 2 least
	 * significant bits
	 */

	/* *** STAGE (1, 2, 3) *** */

	/* start with CKE high */

	/* tINIT = 200us */

	/*
	 * 200us @ 300MHz (3.33 ns) ~ 60000 clock cycles
	 * If a and b are the number of iteration in 2 nested loops
	 * it takes the following number of cycles to complete the operation
	 * number_of_cycles = ((2 + n) * b + 2) * a
	 * where n is the number of instruction in the inner loop
	 * One possible solution is n = 0 , a = 256 , b = 118 => a = FF,
	 * b = 76
	 */

	/* TODO: Need to manage multi-rank */

	/* Load counters */
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(0xFF));
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(0x76));

	/* Load jump address */
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_INIT_CKE_1);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0, __RW_MGR_INIT_CKE_1_inloop);

	/*Execute count instruction and bring up CKE */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_INIT_CKE_1);

	/* indicate that memory is stable */
	IOWR_32DIRECT(PHY_MGR_RESET_MEM_STBL, 0, 1);

	/* Multi-rank section begins here */
	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/*
		 * * **** *
		 * * NOTE *
		 * * **** *
		 * The following commands must be spaced by tMRD or tRPA
		 * which are in the order
		 * of 2 to 4 full rate cycles. This is peanuts in the
		 * NIOS domain, so for now
		 * we can avoid redundant wait loops
		 */

		/*
		 * Possible FIXME BEN: for HHP, we need to add delay loops
		 * to be sure although, the sequencer write interface by
		 * itself likely has enough delay
		 */

		/* *** STAGE (9) */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_PRECHARGE_ALL);

		/* *** STAGE (10) */

		/* Issue 2 refresh commands spaced by tREF */

		/* First REFRESH */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_REFRESH);

		/* tREF = 200ns */
		delay_for_n_ns(200);

		/* Second REFRESH */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_REFRESH);

		/* Second idle loop */
		delay_for_n_ns(200);

		/* *** STAGE (11) */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR_CALIB);

		/* *** STAGE (13) */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_EMR);
	}
}
#endif /* LPDDR1 */

#if QDRII
void rw_mgr_mem_initialize(void)
{
	TRACE_FUNC();

	/* Turn off QDRII DLL to reset it */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_IDLE);

	/* Turn on QDRII DLL and wait 25us for it to lock */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_NOP);
	delay_for_n_ns(25000);

	/* indicate that memory is stable */
	IOWR_32DIRECT(PHY_MGR_RESET_MEM_STBL, 0, 1);
}
#endif

#if QDRII
void rw_mgr_mem_dll_lock_wait(void)
{
	/* The DLL in QDR requires 25us to lock */
	delay_for_n_ns(25000);
}
#else
void rw_mgr_mem_dll_lock_wait(void) { }
#endif

#if RLDRAMII
void rw_mgr_mem_initialize(void)
{
	TRACE_FUNC();

	/* start with memory RESET activated */

	/* tINIT = 200us */
	delay_for_n_ns(200000);

	/* indicate that memory is stable */
	IOWR_32DIRECT(PHY_MGR_RESET_MEM_STBL, 0, 1);

	/*
	 * Dummy MRS, followed by valid MRS commands to reset the DLL on
	 * memory device
	 */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS_INIT);

	/*
	 * 8192 memory cycles for DLL to lock.
	 * 8192 cycles are required by Renesas LLDRAM-II, though we don't
	 * officially support it.
	 */
	delay_for_n_mem_clocks(8192);

	/* Refresh all banks */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_REF_X8);

	/* 1024 memory cycles */
	delay_for_n_mem_clocks(1024);
}
#endif

#if RLDRAM3
void rw_mgr_mem_initialize(void)
{
	TRACE_FUNC();
	uint32_t r;

	/*
	 * Here's how you load register for a loop
	 * Counters are located @ 0x800
	 * Jump address are located @ 0xC00
	 * For both, registers 0 to 3 are selected using bits 3 and 2, like in
	 * 0x800, 0x804, 0x808, 0x80C and 0xC00, 0xC04, 0xC08, 0xC0C
	 * I know this ain't pretty, but Avalon bus throws away the 2
	 * least significant bits
	 */

	/* start with memory RESET activated */

	/* tINIT = 200us */

	/*
	 * 200us @ 266MHz (3.75 ns) ~ 54000 clock cycles
	 * If a and b are the number of iteration in 2 nested loops
	 * it takes the following number of cycles to complete the operation:
	 * number_of_cycles = ((2 + n) * a + 2) * b
	 * where n is the number of instruction in the inner loop
	 * One possible solution is n = 0 , a = 256 , b = 106 => a = FF,
	 * b = 6A
	 */

	/* Load counters */
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(0xFF));
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0,
		      SKIP_DELAY_LOOP_VALUE_OR_ZERO(0x6A));

	/* Load jump address */
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_INIT_RESET_0);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0, __RW_MGR_INIT_RESET_0_inloop);

	/* Execute count instruction */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_INIT_RESET_0);

	/* indicate that memory is stable */
	IOWR_32DIRECT(PHY_MGR_RESET_MEM_STBL, 0, 1);

	/* transition the RESET to high */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_NOP);

	/* Wait for 10000 cycles */
	delay_for_n_mem_clocks(10000);

	/* Load MR0 */
	if (RW_MGR_MEM_NUMBER_OF_RANKS == 1) {
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xFE);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS0);
	} else if (RW_MGR_MEM_NUMBER_OF_RANKS == 2) {
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xFC);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS0);
	} else if (RW_MGR_MEM_NUMBER_OF_RANKS == 4) {
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xFC);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS0);
		/* Wait MRSC */
		delay_for_n_mem_clocks(12);
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xF3);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_MRS0_QUAD_RANK);
	} else {
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xFE);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS0);
	}

	/* Wait MRSC */
	delay_for_n_mem_clocks(12);

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);
		/* Load MR1 (reset DLL reset & kick off long ZQ calibration) */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS1_CALIB);
	}

	/*
	 * Wait 512 cycles for DLL to reset and for ZQ
	 * calibration to complete
	 */
	delay_for_n_mem_clocks(512);

	/* Load MR2 (set write protocol to Single Bank) */
	if (RW_MGR_MEM_NUMBER_OF_RANKS == 1)
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xFE);
	else if (RW_MGR_MEM_NUMBER_OF_RANKS == 2)
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xFC);
	else if (RW_MGR_MEM_NUMBER_OF_RANKS == 4)
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xF0);
	else
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xFE);
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS2_CALIB);

	/* Wait MRSC and a bit more */
	delay_for_n_mem_clocks(64);
}
#endif

/*
 * At the end of calibration we have to program the user settings in, and
 * USER  hand off the memory to the user.
 */
#if DDR3
void rw_mgr_mem_handoff(void)
{
	uint32_t r;
#if LRDIMM
	uint32_t rtt_nom;
	uint32_t rtt_drv;
	uint32_t rtt_wr;
#endif /* LRDIMM */

	TRACE_FUNC();

#if LRDIMM
	rtt_nom = LRDIMM_SPD_MR_RTT_NOM(LRDIMM_SPD_MR);
	rtt_drv = LRDIMM_SPD_MR_RTT_DRV(LRDIMM_SPD_MR);
	rtt_wr  = LRDIMM_SPD_MR_RTT_WR(LRDIMM_SPD_MR);

	/* Configure LRDIMM to broadcast LRDIMM MRS commands to all ranks */
	rw_mgr_lrdimm_rc_program(0, 14, 0x9); /*broadcast mode */

	/* USER Update contents of AC ROM with new RTT WR, DRV values */
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs1, rtt_drv, 0, 1);
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs1, rtt_drv, 1, 1);
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs2, rtt_wr,  0, 2);
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs2, rtt_wr,  1, 2);
#endif /* LRDIMM */

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;
#if MRS_MIRROR_PING_PONG_ATSO
		/* Side 0 */
		set_rank_and_odt_mask_for_ping_pong_atso(0,
							 RW_MGR_ODT_MODE_OFF);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_PRECHARGE_ALL);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS2);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS3);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS1);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS0_USER);

		/* Side 1 */
		set_rank_and_odt_mask_for_ping_pong_atso(1,
							 RW_MGR_ODT_MODE_OFF);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_PRECHARGE_ALL);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS2_MIRR);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS3_MIRR);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS1_MIRR);
		delay_for_n_mem_clocks(4);
		set_jump_as_return();
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_MRS0_USER_MIRR);

		/* Unmask all CS */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);
#else
		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/* precharge all banks ... */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_PRECHARGE_ALL);

		/* load up MR settings specified by user */

		/*
		 * Use Mirror-ed commands for odd ranks if address
		 * mirrorring is on
		 */
		if ((RW_MGR_MEM_ADDRESS_MIRRORING >> r) & 0x1) {
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS2_MIRR);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS3_MIRR);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS1_MIRR);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS0_USER_MIRR);
		} else {
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS2);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS3);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS1);
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS0_USER);
		}
#endif
		/*
		 * USER  need to wait tMOD (12CK or 15ns) time before issuing
		 * other commands, but we will have plenty of NIOS cycles before
		 * actual handoff so its okay.
		 */
	}


#if LRDIMM
	delay_for_n_mem_clocks(12);
	/* USER Set up targetted MRS commands */
	rw_mgr_lrdimm_rc_program(0, 14, 0xD);
	/*
	 * USER update AC ROM MR1 entry to include RTT_NOM for physical
	 * ranks 0,1 only
	 */
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs1,
				    (rtt_drv|rtt_nom), 0, 1);
	rtt_change_MRS1_MRS2_NOM_WR(__RW_MGR_CONTENT_ac_mrs1,
				    (rtt_drv|rtt_nom), 1, 1);

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/*
		 * Use Mirror-ed commands for odd ranks if address
		 * mirrorring is on
		 */
		if ((RW_MGR_MEM_ADDRESS_MIRRORING >> r) & 0x1) {
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS1_MIRR);
			delay_for_n_mem_clocks(4);
		} else {
			delay_for_n_mem_clocks(4);
			set_jump_as_return();
			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
				      __RW_MGR_MRS1);
			delay_for_n_mem_clocks(4);
		}
	}
#endif /* LRDIMM */
}
#endif /* DDR3 */

#if DDR2
void rw_mgr_mem_handoff(void)
{
	uint32_t r;

	TRACE_FUNC();

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/* precharge all banks ... */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_PRECHARGE_ALL);

		/* load up MR settings specified by user */

		/*
		 * FIXME BEN: for HHP, we need to add delay loops to be sure
		 * We can check this with BFM perhaps
		 * Likely enough delay in RW_MGR though
		 */

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_EMR2);

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_EMR3);

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_EMR);

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR_USER);

		/*
		 * USER need to wait tMOD (12CK or 15ns) time before issuing
		 * other commands,
		 * USER but we will have plenty of NIOS cycles before actual
		 * handoff so its okay.
		 */
	}
}
#endif /* DDR2 */

#if LPDDR2
void rw_mgr_mem_handoff(void)
{
	uint32_t r;

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/* precharge all banks... */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_PRECHARGE_ALL);

		/* load up MR settings specified by user */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR1_USER);

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR2);

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR3);
	}
}
#endif /* LPDDR2 */

#if LPDDR1
void rw_mgr_mem_handoff(void)
{
	uint32_t r;

	TRACE_FUNC();

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/* precharge all banks ... */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_PRECHARGE_ALL);

		/* load up MR settings specified by user */

		/*
		 * FIXME BEN: for HHP, we need to add delay loops to be sure
		 * We can check this with BFM perhaps
		 * Likely enough delay in RW_MGR though
		 */

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_EMR);

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MR_USER);

		/*
		 * need to wait tMOD (12CK or 15ns) time before issuing
		 * other commands,
		 * but we will have plenty of NIOS cycles before actual
		 * handoff so its okay.
		 */
	}
}
#endif /* LPDDR1 */

#if RLDRAMII
void rw_mgr_mem_handoff(void)
{
	TRACE_FUNC();

	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS);
}
#endif

#if RLDRAM3
void rw_mgr_mem_handoff(void)
{
	TRACE_FUNC();

	uint32_t r;
	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/* Load user requested MR1 */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS1);
	}

	if (RW_MGR_MEM_NUMBER_OF_RANKS == 1)
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xFE);
	else if (RW_MGR_MEM_NUMBER_OF_RANKS == 2)
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xFC);
	else if (RW_MGR_MEM_NUMBER_OF_RANKS == 4)
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xF0);
	else
		IOWR_32DIRECT(RW_MGR_SET_CS_AND_ODT_MASK, 0, 0xFE);
	/* Load user requested MR2 */
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0, __RW_MGR_MRS2);

	/* Wait MRSC and a bit more */
	delay_for_n_mem_clocks(64);
}
#endif

#if QDRII
void rw_mgr_mem_handoff(void)
{
	TRACE_FUNC();
}
#endif

#if DDRX
/*
 * performs a guaranteed read on the patterns we are going to use during a
 * read test to ensure memory works
 */
uint32_t rw_mgr_mem_calibrate_read_test_patterns(uint32_t rank_bgn,
	uint32_t group, uint32_t num_tries, uint32_t *bit_chk,
	uint32_t all_ranks)
{
	uint32_t r, vg;
	uint32_t correct_mask_vg;
	uint32_t tmp_bit_chk;
	uint32_t rank_end = all_ranks ? RW_MGR_MEM_NUMBER_OF_RANKS :
		(rank_bgn + NUM_RANKS_PER_SHADOW_REG);

	*bit_chk = param->read_correct_mask;
	correct_mask_vg = param->read_correct_mask_vg;

	for (r = rank_bgn; r < rank_end; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_READ_WRITE);

		/* Load up a constant bursts of read commands */

		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x20);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0,
			      __RW_MGR_GUARANTEED_READ);

		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, 0x20);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
			      __RW_MGR_GUARANTEED_READ_CONT);

		tmp_bit_chk = 0;
		for (vg = RW_MGR_MEM_VIRTUAL_GROUPS_PER_READ_DQS-1; ; vg--) {
			/* reset the fifos to get pointers to known state */

			IOWR_32DIRECT(PHY_MGR_CMD_FIFO_RESET, 0, 0);
			IOWR_32DIRECT(RW_MGR_RESET_READ_DATAPATH, 0, 0);

			tmp_bit_chk = tmp_bit_chk << (RW_MGR_MEM_DQ_PER_READ_DQS
				/ RW_MGR_MEM_VIRTUAL_GROUPS_PER_READ_DQS);

			IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP,
				      ((group *
				      RW_MGR_MEM_VIRTUAL_GROUPS_PER_READ_DQS +
				      vg) << 2), __RW_MGR_GUARANTEED_READ);
			tmp_bit_chk = tmp_bit_chk | (correct_mask_vg &
				~(IORD_32DIRECT(BASE_RW_MGR, 0)));

			if (vg == 0)
				break;
		}
		*bit_chk &= tmp_bit_chk;
	}

	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, (group << 2),
		      __RW_MGR_CLEAR_DQS_ENABLE);

	set_rank_and_odt_mask(0, RW_MGR_ODT_MODE_OFF);
	DPRINT(2, "test_load_patterns(%u,ALL) => (%u == %u) => %lu",
	       group, *bit_chk, param->read_correct_mask,
	       (long unsigned int)(*bit_chk == param->read_correct_mask));
	return *bit_chk == param->read_correct_mask;
}

static inline uint32_t rw_mgr_mem_calibrate_read_test_patterns_all_ranks
	(uint32_t group, uint32_t num_tries, uint32_t *bit_chk)
{
	return rw_mgr_mem_calibrate_read_test_patterns(0, group,
		num_tries, bit_chk, 1);
}
#endif

/* load up the patterns we are going to use during a read test */
#if DDRX
void rw_mgr_mem_calibrate_read_load_patterns(uint32_t rank_bgn,
	uint32_t all_ranks)
{
	uint32_t r;
	uint32_t rank_end = all_ranks ? RW_MGR_MEM_NUMBER_OF_RANKS :
		(rank_bgn + NUM_RANKS_PER_SHADOW_REG);

	TRACE_FUNC();

	for (r = rank_bgn; r < rank_end; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_READ_WRITE);

		/* Load up a constant bursts */
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x20);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0,
			      __RW_MGR_GUARANTEED_WRITE_WAIT0);

		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, 0x20);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
			      __RW_MGR_GUARANTEED_WRITE_WAIT1);

#if QUARTER_RATE
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, 0x01);
#endif
#if HALF_RATE
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, 0x02);
#endif
#if FULL_RATE
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, 0x04);
#endif
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
			      __RW_MGR_GUARANTEED_WRITE_WAIT2);

#if QUARTER_RATE
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_3, 0, 0x01);
#endif
#if HALF_RATE
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_3, 0, 0x02);
#endif
#if FULL_RATE
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_3, 0, 0x04);
#endif
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_3, 0,
			      __RW_MGR_GUARANTEED_WRITE_WAIT3);

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_GUARANTEED_WRITE);
	}

	set_rank_and_odt_mask(0, RW_MGR_ODT_MODE_OFF);
}
#endif

#if QDRII
void rw_mgr_mem_calibrate_read_load_patterns(uint32_t rank_bgn,
	uint32_t all_ranks)
{
	TRACE_FUNC();

	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x20);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0,
		      __RW_MGR_GUARANTEED_WRITE_WAIT0);
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, 0x20);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
		      __RW_MGR_GUARANTEED_WRITE_WAIT1);
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
		      __RW_MGR_GUARANTEED_WRITE);
}
#endif

#if RLDRAMX
void rw_mgr_mem_calibrate_read_load_patterns(uint32_t rank_bgn,
	uint32_t all_ranks)
{
	TRACE_FUNC();
	uint32_t r;
	uint32_t rank_end = RW_MGR_MEM_NUMBER_OF_RANKS;
#if QUARTER_RATE
	uint32_t write_data_cycles = 0x10;
#else
	uint32_t write_data_cycles = 0x20;
#endif

	for (r = rank_bgn; r < rank_end; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;
		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_READ_WRITE);

		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, write_data_cycles);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0,
			      __RW_MGR_GUARANTEED_WRITE_WAIT0);
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, write_data_cycles);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
			      __RW_MGR_GUARANTEED_WRITE_WAIT1);
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, write_data_cycles);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
			      __RW_MGR_GUARANTEED_WRITE_WAIT2);
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_3, 0, write_data_cycles);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_3, 0,
			      __RW_MGR_GUARANTEED_WRITE_WAIT3);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_GUARANTEED_WRITE);
	}
	set_rank_and_odt_mask(0, RW_MGR_ODT_MODE_OFF);
}
#endif

/*
 * try a read and see if it returns correct data back. has dummy reads
 * inserted into the mix used to align dqs enable. has more thorough checks
 * than the regular read test.
 */

uint32_t rw_mgr_mem_calibrate_read_test(uint32_t rank_bgn, uint32_t group,
	uint32_t num_tries, uint32_t all_correct, uint32_t *bit_chk,
	uint32_t all_groups, uint32_t all_ranks)
{
	uint32_t r, vg;
	uint32_t correct_mask_vg;
	uint32_t tmp_bit_chk;
	uint32_t rank_end = all_ranks ? RW_MGR_MEM_NUMBER_OF_RANKS :
		(rank_bgn + NUM_RANKS_PER_SHADOW_REG);

#if LRDIMM
	/* USER Disable MB Write-levelling mode and enter normal operation */
	rw_mgr_lrdimm_rc_program(0, 12, 0x0);
#endif

	*bit_chk = param->read_correct_mask;
	correct_mask_vg = param->read_correct_mask_vg;

	uint32_t quick_read_mode = (((STATIC_CALIB_STEPS) &
		CALIB_SKIP_DELAY_SWEEPS) && ENABLE_SUPER_QUICK_CALIBRATION);

	for (r = rank_bgn; r < rank_end; r++) {
		if (param->skip_ranks[r])
			/* request to skip the rank */
			continue;

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_READ_WRITE);

		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, 0x10);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
			      __RW_MGR_READ_B2B_WAIT1);
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, 0x10);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
			      __RW_MGR_READ_B2B_WAIT2);

		if (quick_read_mode)
			IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x1);
			/* need at least two (1+1) reads to capture failures */
		else if (all_groups)
			IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x06);
		else
			IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x32);

		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_READ_B2B);
		if (all_groups)
			IOWR_32DIRECT(RW_MGR_LOAD_CNTR_3, 0,
				      RW_MGR_MEM_IF_READ_DQS_WIDTH *
				      RW_MGR_MEM_VIRTUAL_GROUPS_PER_READ_DQS - 1);
		else
			IOWR_32DIRECT(RW_MGR_LOAD_CNTR_3, 0, 0x0);

		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_3, 0, __RW_MGR_READ_B2B);

		tmp_bit_chk = 0;
		for (vg = RW_MGR_MEM_VIRTUAL_GROUPS_PER_READ_DQS-1; ; vg--) {
			/* reset the fifos to get pointers to known state */
			IOWR_32DIRECT(PHY_MGR_CMD_FIFO_RESET, 0, 0);
			IOWR_32DIRECT(RW_MGR_RESET_READ_DATAPATH, 0, 0);

			tmp_bit_chk = tmp_bit_chk << (RW_MGR_MEM_DQ_PER_READ_DQS
				/ RW_MGR_MEM_VIRTUAL_GROUPS_PER_READ_DQS);

			IOWR_32DIRECT(all_groups ? RW_MGR_RUN_ALL_GROUPS :
				      RW_MGR_RUN_SINGLE_GROUP, ((group *
				      RW_MGR_MEM_VIRTUAL_GROUPS_PER_READ_DQS
				      + vg) << 2), __RW_MGR_READ_B2B);
			tmp_bit_chk = tmp_bit_chk | (correct_mask_vg &
				~(IORD_32DIRECT(BASE_RW_MGR, 0)));

			if (vg == 0)
				break;
		}
		*bit_chk &= tmp_bit_chk;
	}

#if ENABLE_BRINGUP_DEBUGGING
	load_di_buf_gbl();
#endif

	#if DDRX
	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, (group << 2),
		      __RW_MGR_CLEAR_DQS_ENABLE);
	#endif

	if (all_correct) {
		set_rank_and_odt_mask(0, RW_MGR_ODT_MODE_OFF);
		DPRINT(2, "read_test(%u,ALL,%u) => (%u == %u) => %lu",
		       group, all_groups, *bit_chk, param->read_correct_mask,
		       (long unsigned int)(*bit_chk ==
		       param->read_correct_mask));
		return *bit_chk == param->read_correct_mask;
	} else	{
		set_rank_and_odt_mask(0, RW_MGR_ODT_MODE_OFF);
		DPRINT(2, "read_test(%u,ONE,%u) => (%u != %lu) => %lu",
		       group, all_groups, *bit_chk, (long unsigned int)0,
		       (long unsigned int)(*bit_chk != 0x00));
		return *bit_chk != 0x00;
	}
}

static inline uint32_t rw_mgr_mem_calibrate_read_test_all_ranks(uint32_t group,
	uint32_t num_tries, uint32_t all_correct, uint32_t *bit_chk,
	uint32_t all_groups)
{
	return rw_mgr_mem_calibrate_read_test(0, group, num_tries, all_correct,
					      bit_chk, all_groups, 1);
}

void rw_mgr_incr_vfifo(uint32_t grp, uint32_t *v)
{
	/* fiddle with FIFO */
	if (HARD_PHY) {
		IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_HARD_PHY, 0, grp);
	} else if (QUARTER_RATE_MODE && !HARD_VFIFO) {
		if ((*v & 3) == 3)
			IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_QR, 0, grp);
		else if ((*v & 2) == 2)
			IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_FR_HR, 0, grp);
		else if ((*v & 1) == 1)
			IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_HR, 0, grp);
		else
			IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_FR, 0, grp);
	} else if (HARD_VFIFO) {
		/* Arria V & Cyclone V have a hard full-rate VFIFO that only
		has a single incr signal */
		IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_FR, 0, grp);
	} else {
		if (!HALF_RATE_MODE || (*v & 1) == 1)
			IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_HR, 0, grp);
		else
			IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_FR, 0, grp);
	}

	(*v)++;
	BFM_INC_VFIFO;
}

/*Used in quick cal to properly loop through the duplicated VFIFOs in
AV QDRII/RLDRAM */
static inline void rw_mgr_incr_vfifo_all(uint32_t grp, uint32_t *v)
{
#if VFIFO_CONTROL_WIDTH_PER_DQS == 1
	rw_mgr_incr_vfifo(grp, v);
#else
	uint32_t i;
	for (i = 0; i < VFIFO_CONTROL_WIDTH_PER_DQS; i++) {
		rw_mgr_incr_vfifo(grp*VFIFO_CONTROL_WIDTH_PER_DQS+i, v);
		if (i != 0)
			(*v)--;
	}
#endif
}

void rw_mgr_decr_vfifo(uint32_t grp, uint32_t *v)
{
	uint32_t i;

	for (i = 0; i < VFIFO_SIZE-1; i++)
		rw_mgr_incr_vfifo(grp, v);
}

/* find a good dqs enable to use */
#if QDRII || RLDRAMX
uint32_t rw_mgr_mem_calibrate_vfifo_find_dqs_en_phase(uint32_t grp)
{
	uint32_t v;
	uint32_t found;
	uint32_t dtaps_per_ptap, tmp_delay;
	uint32_t bit_chk;

	TRACE_FUNC("%lu", grp);

	reg_file_set_sub_stage(CAL_SUBSTAGE_DQS_EN_PHASE);

	found = 0;

	/* first push vfifo until we get a passing read */
	for (v = 0; v < VFIFO_SIZE && found == 0;) {
		DPRINT(2, "find_dqs_en_phase: vfifo %lu",
		       BFM_GBL_GET(vfifo_idx));
		if (rw_mgr_mem_calibrate_read_test_all_ranks(grp, 1,
							     PASS_ONE_BIT,
							     &bit_chk, 0)) {
			found = 1;
		}

		if (!found) {
			/* fiddle with FIFO */
#if (VFIFO_CONTROL_WIDTH_PER_DQS != 1)
			uint32_t i;
			for (i = 0; i < VFIFO_CONTROL_WIDTH_PER_DQS; i++) {
				rw_mgr_incr_vfifo(grp *
					VFIFO_CONTROL_WIDTH_PER_DQS+i, &v);
				v--;
				/* undo increment of v in rw_mgr_incr_vfifo */
			}
			v++;	/* add back single increment */
#else
			rw_mgr_incr_vfifo(grp, &v);
#endif
		}
	}

#if (VFIFO_CONTROL_WIDTH_PER_DQS != 1)
	if (found) {
		/*
		 * we found a vfifo setting that works for at least one vfifo
		 * "group"
		 * Some groups may need next vfifo setting, so check each one to
		 * see if we get new bits passing by increment the vfifo
		 */
		uint32_t i;
		uint32_t best_bit_chk_inv;
		uint8_t found_on_first_check = (v == 1);

		best_bit_chk_inv = ~bit_chk;

		for (i = 0; i < VFIFO_CONTROL_WIDTH_PER_DQS; i++) {
			rw_mgr_incr_vfifo(grp * VFIFO_CONTROL_WIDTH_PER_DQS+i,
					  &v);
			v--;
			/*
			 * undo increment of v in rw_mgr_incr_vfifo,
			 * just in case it matters for next check
			 */
			rw_mgr_mem_calibrate_read_test_all_ranks(grp, 1,
								 PASS_ONE_BIT,
								 &bit_chk, 0);
			if ((bit_chk & best_bit_chk_inv) != 0) {
				/* found some new bits */
				best_bit_chk_inv = ~bit_chk;
			} else {
				/* no improvement, so put back */
				rw_mgr_decr_vfifo(grp *
					VFIFO_CONTROL_WIDTH_PER_DQS+i, &v);
				v++;
				if (found_on_first_check) {
					/* found on first vfifo check, so we
					also need to check earlier vfifo
					values */
					rw_mgr_decr_vfifo(grp *
						VFIFO_CONTROL_WIDTH_PER_DQS +
						i, &v);
					v++; /* undo decrement of v in
					rw_mgr_incr_vfifo, just in case it
					matters for next check */
					rw_mgr_mem_calibrate_read_test_all_ranks
					(grp, 1, PASS_ONE_BIT, &bit_chk, 0);
					if ((bit_chk & best_bit_chk_inv) != 0) {
						/* found some new bits */
						best_bit_chk_inv = ~bit_chk;
					} else {
						/* no improvement,
						so put back */
						rw_mgr_incr_vfifo(grp *
						VFIFO_CONTROL_WIDTH_PER_DQS +
						i, &v);
						v--;
					}
				} /* found_on_first_check */
			} /* check for new bits */
		} /* loop over all vfifo control bits */
	}
#endif

	if (found) {
		DPRINT(2, "find_dqs_en_phase: found vfifo=%lu",
		       BFM_GBL_GET(vfifo_idx));
		/*
		 * Not really dqs_enable left/right edge, but close enough
		 * for testing purposes.
		 */
		BFM_GBL_SET(dqs_enable_left_edge[grp].v,
			    BFM_GBL_GET(vfifo_idx));
		BFM_GBL_SET(dqs_enable_right_edge[grp].v,
			    BFM_GBL_GET(vfifo_idx));
		BFM_GBL_SET(dqs_enable_mid[grp].v,
			    BFM_GBL_GET(vfifo_idx));
	} else {
		DPRINT(2, "find_dqs_en_phase: no valid vfifo found");
	}

	return found;
}
#endif

#if DDRX
uint32_t rw_mgr_mem_calibrate_vfifo_find_dqs_en_phase(uint32_t grp)
{
	uint32_t i, d, v, p;
	uint32_t max_working_cnt;
	uint32_t fail_cnt;
	uint32_t bit_chk;
	uint32_t dtaps_per_ptap;
	uint32_t found_begin, found_end;
	uint32_t work_bgn, work_mid, work_end, tmp_delay;
	uint32_t test_status;
	uint32_t found_passing_read, found_failing_read, initial_failing_dtap;

	TRACE_FUNC("%u", grp);
	BFM_STAGE("find_dqs_en_phase");
	ALTERA_ASSERT(grp < RW_MGR_MEM_IF_READ_DQS_WIDTH);

	reg_file_set_sub_stage(CAL_SUBSTAGE_VFIFO_CENTER);

	scc_mgr_set_dqs_en_delay_all_ranks(grp, 0);
	scc_mgr_set_dqs_en_phase_all_ranks(grp, 0);

	fail_cnt = 0;

	/* ************************************************************** */
	/* * Step 0 : Determine number of delay taps for each phase tap * */

	dtaps_per_ptap = 0;
	tmp_delay = 0;
	while (tmp_delay < IO_DELAY_PER_OPA_TAP) {
		dtaps_per_ptap++;
		tmp_delay += IO_DELAY_PER_DQS_EN_DCHAIN_TAP;
	}
	dtaps_per_ptap--;
	ALTERA_ASSERT(dtaps_per_ptap <= IO_DQS_EN_DELAY_MAX);
	tmp_delay = 0;

	/* VFIFO sweep */
#if ENABLE_DQSEN_SWEEP
	init_di_buffer();
	work_bgn = 0;
	for (d = 0; d <= dtaps_per_ptap; d++, tmp_delay +=
		IO_DELAY_PER_DQS_EN_DCHAIN_TAP) {
		work_bgn = tmp_delay;
		scc_mgr_set_dqs_en_delay_all_ranks(grp, d);

		for (i = 0; i < VFIFO_SIZE; i++) {
			for (p = 0; p <= IO_DQS_EN_PHASE_MAX; p++, work_bgn +=
				IO_DELAY_PER_OPA_TAP) {
				scc_mgr_set_dqs_en_phase_all_ranks(grp, p);

				test_status =
				rw_mgr_mem_calibrate_read_test_all_ranks(grp,
									 1,
									 PASS_ONE_BIT,
									 &bit_chk, 0);

				/*if (p ==0 && d == 0) */
				sample_di_data(bit_chk, work_bgn, d, i, p);
			}
			/*Increment FIFO */
			rw_mgr_incr_vfifo(grp, &v);
		}

		work_bgn++;
	}
	debug_data->di_report.flags |= DI_REPORT_FLAGS_READY;
	debug_data->di_report.flags |= DI_REPORT_FLAGS_DONE;
#endif

	/* ********************************************************* */
	/* * Step 1 : First push vfifo until we get a failing read * */
	for (v = 0; v < VFIFO_SIZE; ) {
		DPRINT(2, "find_dqs_en_phase: vfifo %lu", BFM_GBL_GET
		       (vfifo_idx));
		test_status = rw_mgr_mem_calibrate_read_test_all_ranks
			(grp, 1, PASS_ONE_BIT, &bit_chk, 0);
		if (!test_status) {
			fail_cnt++;

			if (fail_cnt == 2)
				break;
		}

		/* fiddle with FIFO */
		rw_mgr_incr_vfifo(grp, &v);
	}

	if (v >= VFIFO_SIZE) {
		/* no failing read found!! Something must have gone wrong */
		DPRINT(2, "find_dqs_en_phase: vfifo failed");
		return 0;
	}

	max_working_cnt = 0;

	/* ******************************************************** */
	/* * step 2: find first working phase, increment in ptaps * */
	found_begin = 0;
	work_bgn = 0;
	for (d = 0; d <= dtaps_per_ptap; d++, tmp_delay +=
		IO_DELAY_PER_DQS_EN_DCHAIN_TAP) {
		work_bgn = tmp_delay;
		scc_mgr_set_dqs_en_delay_all_ranks(grp, d);

		for (i = 0; i < VFIFO_SIZE; i++) {
			for (p = 0; p <= IO_DQS_EN_PHASE_MAX; p++, work_bgn +=
				IO_DELAY_PER_OPA_TAP) {
				scc_mgr_set_dqs_en_phase_all_ranks(grp, p);

				test_status =
				rw_mgr_mem_calibrate_read_test_all_ranks
				(grp, 1, PASS_ONE_BIT, &bit_chk, 0);

				if (test_status) {
					max_working_cnt = 1;
					found_begin = 1;
					break;
				}
			}

			if (found_begin)
				break;

			if (p > IO_DQS_EN_PHASE_MAX)
				/* fiddle with FIFO */
				rw_mgr_incr_vfifo(grp, &v);
		}

		if (found_begin)
			break;
	}

	if (i >= VFIFO_SIZE) {
		/* cannot find working solution */
		DPRINT(2, "find_dqs_en_phase: no vfifo/ptap/dtap");
		return 0;
	}

	work_end = work_bgn;

	/*  If d is 0 then the working window covers a phase tap and
	we can follow the old procedure otherwise, we've found the beginning,
	and we need to increment the dtaps until we find the end */
	if (d == 0) {
		/* ********************************************************* */
		/* * step 3a: if we have room, back off by one and
		increment in dtaps * */
		COV(EN_PHASE_PTAP_OVERLAP);

		/* Special case code for backing up a phase */
		if (p == 0) {
			p = IO_DQS_EN_PHASE_MAX ;
			rw_mgr_decr_vfifo(grp, &v);
		} else {
			p = p - 1;
		}
		tmp_delay = work_bgn - IO_DELAY_PER_OPA_TAP;
		scc_mgr_set_dqs_en_phase_all_ranks(grp, p);

		found_begin = 0;
		for (d = 0; d <= IO_DQS_EN_DELAY_MAX && tmp_delay < work_bgn;
			d++, tmp_delay += IO_DELAY_PER_DQS_EN_DCHAIN_TAP) {
			scc_mgr_set_dqs_en_delay_all_ranks(grp, d);

			if (rw_mgr_mem_calibrate_read_test_all_ranks(grp, 1,
								     PASS_ONE_BIT,
								     &bit_chk, 0)) {
				found_begin = 1;
				work_bgn = tmp_delay;
				break;
			}
		}

		/* We have found a working dtap before the ptap found above */
		if (found_begin == 1)
			max_working_cnt++;

		/* Restore VFIFO to old state before we decremented it
		(if needed) */
		p = p + 1;
		if (p > IO_DQS_EN_PHASE_MAX) {
			p = 0;
			rw_mgr_incr_vfifo(grp, &v);
		}

		scc_mgr_set_dqs_en_delay_all_ranks(grp, 0);

		/* ********************************************************* */
		/* * step 4a: go forward from working phase to non working
		phase, increment in ptaps * */
		p = p + 1;
		work_end += IO_DELAY_PER_OPA_TAP;
		if (p > IO_DQS_EN_PHASE_MAX) {
			/* fiddle with FIFO */
			p = 0;
			rw_mgr_incr_vfifo(grp, &v);
		}

		found_end = 0;
		for (; i < VFIFO_SIZE + 1; i++) {
			for (; p <= IO_DQS_EN_PHASE_MAX; p++, work_end
				+= IO_DELAY_PER_OPA_TAP) {
				scc_mgr_set_dqs_en_phase_all_ranks(grp, p);

				if (!rw_mgr_mem_calibrate_read_test_all_ranks
					(grp, 1, PASS_ONE_BIT, &bit_chk, 0)) {
					found_end = 1;
					break;
				} else {
					max_working_cnt++;
				}
			}

			if (found_end)
				break;

			if (p > IO_DQS_EN_PHASE_MAX) {
				/* fiddle with FIFO */
				rw_mgr_incr_vfifo(grp, &v);
				p = 0;
			}
		}

		if (i >= VFIFO_SIZE + 1) {
			/* cannot see edge of failing read */
			DPRINT(2, "find_dqs_en_phase: end: failed");
			return 0;
		}

		/* ********************************************************* */
		/* * step 5a:  back off one from last, increment in dtaps  * */

		/* Special case code for backing up a phase */
		if (p == 0) {
			p = IO_DQS_EN_PHASE_MAX;
			rw_mgr_decr_vfifo(grp, &v);
		} else {
			p = p - 1;
		}

		work_end -= IO_DELAY_PER_OPA_TAP;
		scc_mgr_set_dqs_en_phase_all_ranks(grp, p);

		/* * The actual increment of dtaps is done outside of
		the if/else loop to share code */
		d = 0;

		DPRINT(2, "find_dqs_en_phase: v/p: vfifo=%lu ptap=%u",
		       BFM_GBL_GET(vfifo_idx), p);
	} else {
		/* ******************************************************* */
		/* * step 3-5b:  Find the right edge of the window using
		delay taps   * */
		COV(EN_PHASE_PTAP_NO_OVERLAP);

		DPRINT(2, "find_dqs_en_phase:vfifo=%lu ptap=%u dtap=%u bgn=%u",
		       BFM_GBL_GET(vfifo_idx), p, d, work_bgn);
		BFM_GBL_SET(dqs_enable_left_edge[grp].v,
			    BFM_GBL_GET(vfifo_idx));
		BFM_GBL_SET(dqs_enable_left_edge[grp].p, p);
		BFM_GBL_SET(dqs_enable_left_edge[grp].d, d);
		BFM_GBL_SET(dqs_enable_left_edge[grp].ps, work_bgn);

		work_end = work_bgn;

		/* * The actual increment of dtaps is done outside of the
		if/else loop to share code */

		/* Only here to counterbalance a subtract later on which is
		not needed if this branch of the algorithm is taken */
		max_working_cnt++;
	}

	/* The dtap increment to find the failing edge is done here */
	for (; d <= IO_DQS_EN_DELAY_MAX; d++, work_end +=
		IO_DELAY_PER_DQS_EN_DCHAIN_TAP) {
			DPRINT(2, "find_dqs_en_phase: end-2: dtap=%u", d);
			scc_mgr_set_dqs_en_delay_all_ranks(grp, d);

			if (!rw_mgr_mem_calibrate_read_test_all_ranks(grp, 1,
								      PASS_ONE_BIT,
								      &bit_chk, 0)) {
				break;
			}
		}

	/* Go back to working dtap */
	if (d != 0)
		work_end -= IO_DELAY_PER_DQS_EN_DCHAIN_TAP;

	DPRINT(2, "find_dqs_en_phase: v/p/d: vfifo=%lu ptap=%u dtap=%u end=%u",
	       BFM_GBL_GET(vfifo_idx), p, d-1, work_end);
	BFM_GBL_SET(dqs_enable_right_edge[grp].v, BFM_GBL_GET(vfifo_idx));
	BFM_GBL_SET(dqs_enable_right_edge[grp].p, p);
	BFM_GBL_SET(dqs_enable_right_edge[grp].d, d-1);
	BFM_GBL_SET(dqs_enable_right_edge[grp].ps, work_end);

	if (work_end >= work_bgn) {
		/* we have a working range */
	} else {
		/* nil range */
		DPRINT(2, "find_dqs_en_phase: end-2: failed");
		return 0;
	}

	DPRINT(2, "find_dqs_en_phase: found range [%u,%u]",
	       work_bgn, work_end);

#if USE_DQS_TRACKING
	/* *************************************************************** */
	/*
	 * * We need to calculate the number of dtaps that equal a ptap
	 * * To do that we'll back up a ptap and re-find the edge of the
	 * * window using dtaps
	 */

	DPRINT(2, "find_dqs_en_phase: calculate dtaps_per_ptap for tracking");

	/* Special case code for backing up a phase */
	if (p == 0) {
		p = IO_DQS_EN_PHASE_MAX;
		rw_mgr_decr_vfifo(grp, &v);
		DPRINT(2, "find_dqs_en_phase: backedup cycle/phase: v=%lu p=%u",
		       BFM_GBL_GET(vfifo_idx), p);
	} else {
		p = p - 1;
		DPRINT(2, "find_dqs_en_phase: backedup phase only: v=%lu p=%u",
		       BFM_GBL_GET(vfifo_idx), p);
	}

	scc_mgr_set_dqs_en_phase_all_ranks(grp, p);

	/*
	 * Increase dtap until we first see a passing read (in case the
	 * window is smaller than a ptap),
	 * and then a failing read to mark the edge of the window again
	 */

	/* Find a passing read */
	DPRINT(2, "find_dqs_en_phase: find passing read");
	found_passing_read = 0;
	found_failing_read = 0;
	initial_failing_dtap = d;
	for (; d <= IO_DQS_EN_DELAY_MAX; d++) {
		DPRINT(2, "find_dqs_en_phase: testing read d=%u", d);
		scc_mgr_set_dqs_en_delay_all_ranks(grp, d);

		if (rw_mgr_mem_calibrate_read_test_all_ranks(grp, 1,
							     PASS_ONE_BIT,
							     &bit_chk, 0)) {
			found_passing_read = 1;
			break;
		}
	}

	if (found_passing_read) {
		/* Find a failing read */
		DPRINT(2, "find_dqs_en_phase: find failing read");
		for (d = d + 1; d <= IO_DQS_EN_DELAY_MAX; d++) {
			DPRINT(2, "find_dqs_en_phase: testing read d=%u", d);
			scc_mgr_set_dqs_en_delay_all_ranks(grp, d);

			if (!rw_mgr_mem_calibrate_read_test_all_ranks
				(grp, 1, PASS_ONE_BIT, &bit_chk, 0)) {
				found_failing_read = 1;
				break;
			}
		}
	} else {
		DPRINT(1, "find_dqs_en_phase: failed to calculate dtaps ");
		DPRINT(1, "per ptap. Fall back on static value");
	}

	/*
	 * The dynamically calculated dtaps_per_ptap is only valid if we
	 * found a passing/failing read. If we didn't, it means d hit the max
	 * (IO_DQS_EN_DELAY_MAX). Otherwise, dtaps_per_ptap retains its
	 * statically calculated value.
	 */
	if (found_passing_read && found_failing_read)
		dtaps_per_ptap = d - initial_failing_dtap;

	ALTERA_ASSERT(dtaps_per_ptap <= IO_DQS_EN_DELAY_MAX);
	IOWR_32DIRECT(REG_FILE_DTAPS_PER_PTAP, 0, dtaps_per_ptap);
	DPRINT(2, "find_dqs_en_phase: dtaps_per_ptap=%u - %u = %u", d,
	       initial_failing_dtap, dtaps_per_ptap);
#endif

	/* ******************************************** */
	/* * step 6:  Find the centre of the window   * */

	work_mid = (work_bgn + work_end) / 2;
	tmp_delay = 0;

	DPRINT(2, "work_bgn=%d work_end=%d work_mid=%d", work_bgn,
	       work_end, work_mid);
	/* Get the middle delay to be less than a VFIFO delay */
	for (p = 0; p <= IO_DQS_EN_PHASE_MAX;
		p++, tmp_delay += IO_DELAY_PER_OPA_TAP)
		;
	DPRINT(2, "vfifo ptap delay %d", tmp_delay);
	while (work_mid > tmp_delay)
		work_mid -= tmp_delay;
	DPRINT(2, "new work_mid %d", work_mid);
	tmp_delay = 0;
	for (p = 0; p <= IO_DQS_EN_PHASE_MAX && tmp_delay < work_mid;
		p++, tmp_delay += IO_DELAY_PER_OPA_TAP)
		;
	tmp_delay -= IO_DELAY_PER_OPA_TAP;
	DPRINT(2, "new p %d, tmp_delay=%d", p-1, tmp_delay);
	for (d = 0; d <= IO_DQS_EN_DELAY_MAX && tmp_delay < work_mid; d++,
		tmp_delay += IO_DELAY_PER_DQS_EN_DCHAIN_TAP)
		;
	DPRINT(2, "new d %d, tmp_delay=%d", d, tmp_delay);

	scc_mgr_set_dqs_en_phase_all_ranks(grp, p-1);
	scc_mgr_set_dqs_en_delay_all_ranks(grp, d);

	/*
	 * push vfifo until we can successfully calibrate. We can do this
	 * because the largest possible margin in 1 VFIFO cycle.
	 */
	for (i = 0; i < VFIFO_SIZE; i++) {
		DPRINT(2, "find_dqs_en_phase: center: vfifo=%lu",
		       BFM_GBL_GET(vfifo_idx));
		if (rw_mgr_mem_calibrate_read_test_all_ranks(grp, 1,
							     PASS_ONE_BIT,
							     &bit_chk, 0)) {
			break;
		}

		/* fiddle with FIFO */
		rw_mgr_incr_vfifo(grp, &v);
	}

	if (i >= VFIFO_SIZE) {
		DPRINT(2, "find_dqs_en_phase: center: failed");
		return 0;
	}
	DPRINT(2, "find_dqs_en_phase: center found: vfifo=%lu ptap=%u dtap=%u",
	       BFM_GBL_GET(vfifo_idx), p-1, d);
	BFM_GBL_SET(dqs_enable_mid[grp].v, BFM_GBL_GET(vfifo_idx));
	BFM_GBL_SET(dqs_enable_mid[grp].p, p-1);
	BFM_GBL_SET(dqs_enable_mid[grp].d, d);
	BFM_GBL_SET(dqs_enable_mid[grp].ps, work_mid);
	return 1;
}
#endif


/* Try rw_mgr_mem_calibrate_vfifo_find_dqs_en_phase across different
dq_in_delay values */
static inline uint32_t
rw_mgr_mem_calibrate_vfifo_find_dqs_en_phase_sweep_dq_in_delay
(uint32_t write_group, uint32_t read_group, uint32_t test_bgn)
{
#if STRATIXV || ARRIAV || CYCLONEV || ARRIAVGZ
	uint32_t found;
	uint32_t i;
	uint32_t p;
	uint32_t d;
	uint32_t r;

	const uint32_t delay_step = IO_IO_IN_DELAY_MAX /
		(RW_MGR_MEM_DQ_PER_READ_DQS-1);
		/* we start at zero, so have one less dq to devide among */

	TRACE_FUNC("(%u,%u,%u)", write_group, read_group, test_bgn);

	/* try different dq_in_delays since the dq path is shorter than dqs */

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS;
		r += NUM_RANKS_PER_SHADOW_REG) {
		select_shadow_regs_for_update(r, write_group, 1);
		for (i = 0, p = test_bgn, d = 0; i < RW_MGR_MEM_DQ_PER_READ_DQS;
			i++, p++, d += delay_step) {
			DPRINT(1, "rw_mgr_mem_calibrate_vfifo_find_dqs_");
			DPRINT(1, "en_phase_sweep_dq_in_delay: g=%u/%u ",
			       write_group, read_group);
			DPRINT(1, "r=%u, i=%u p=%u d=%u", r, i , p, d);
			scc_mgr_set_dq_in_delay(write_group, p, d);
			scc_mgr_load_dq(p);
		}
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	}

	found = rw_mgr_mem_calibrate_vfifo_find_dqs_en_phase(read_group);

	DPRINT(1, "rw_mgr_mem_calibrate_vfifo_find_dqs_en_phase_sweep_dq");
	DPRINT(1, "_in_delay: g=%u/%u found=%u; Reseting delay chain to zero",
	       write_group, read_group, found);

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS;
		r += NUM_RANKS_PER_SHADOW_REG) {
		select_shadow_regs_for_update(r, write_group, 1);
		for (i = 0, p = test_bgn; i < RW_MGR_MEM_DQ_PER_READ_DQS;
			i++, p++) {
			scc_mgr_set_dq_in_delay(write_group, p, 0);
			scc_mgr_load_dq(p);
		}
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	}

	return found;
#else
	return rw_mgr_mem_calibrate_vfifo_find_dqs_en_phase(read_group);
#endif
}

/* per-bit deskew DQ and center */
uint32_t rw_mgr_mem_calibrate_vfifo_center(uint32_t rank_bgn,
	uint32_t write_group, uint32_t read_group, uint32_t test_bgn,
	uint32_t use_read_test, uint32_t update_fom)
{
	uint32_t i, p, d, min_index;
	/* Store these as signed since there are comparisons with
	signed numbers */
	uint32_t bit_chk;
	uint32_t sticky_bit_chk;
	int32_t left_edge[RW_MGR_MEM_DQ_PER_READ_DQS];
	int32_t right_edge[RW_MGR_MEM_DQ_PER_READ_DQS];
	int32_t final_dq[RW_MGR_MEM_DQ_PER_READ_DQS];
	int32_t mid;
	int32_t orig_mid_min, mid_min;
	int32_t new_dqs, start_dqs, start_dqs_en, shift_dq, final_dqs,
		final_dqs_en;
	int32_t dq_margin, dqs_margin;
	uint32_t stop;

	TRACE_FUNC("%u %u", read_group, test_bgn);

	ALTERA_ASSERT(read_group < RW_MGR_MEM_IF_READ_DQS_WIDTH);
	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);

	start_dqs = READ_SCC_DQS_IN_DELAY(read_group);
	if (IO_SHIFT_DQS_EN_WHEN_SHIFT_DQS)
		start_dqs_en = READ_SCC_DQS_EN_DELAY(read_group);

	select_curr_shadow_reg_using_rank(rank_bgn);

	/* per-bit deskew */

	/* set the left and right edge of each bit to an illegal value */
	/* use (IO_IO_IN_DELAY_MAX + 1) as an illegal value */
	sticky_bit_chk = 0;
	for (i = 0; i < RW_MGR_MEM_DQ_PER_READ_DQS; i++) {
		left_edge[i]  = IO_IO_IN_DELAY_MAX + 1;
		right_edge[i] = IO_IO_IN_DELAY_MAX + 1;
	}

	/* Search for the left edge of the window for each bit */
	for (d = 0; d <= IO_IO_IN_DELAY_MAX; d++) {
		scc_mgr_apply_group_dq_in_delay(write_group, test_bgn, d);

		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);

		/*
		 * Stop searching when the read test doesn't pass AND when
		 * we've seen a passing read on every bit.
		 */
		if (use_read_test) {
			stop = !rw_mgr_mem_calibrate_read_test(rank_bgn,
				read_group, NUM_READ_PB_TESTS, PASS_ONE_BIT,
				&bit_chk, 0, 0);
		} else {
			rw_mgr_mem_calibrate_write_test(rank_bgn, write_group,
							0, PASS_ONE_BIT,
							&bit_chk, 0);
			bit_chk = bit_chk >> (RW_MGR_MEM_DQ_PER_READ_DQS *
				(read_group - (write_group *
					RW_MGR_MEM_IF_READ_DQS_WIDTH /
					RW_MGR_MEM_IF_WRITE_DQS_WIDTH)));
			stop = (bit_chk == 0);
		}
		sticky_bit_chk = sticky_bit_chk | bit_chk;
		stop = stop && (sticky_bit_chk == param->read_correct_mask);
		DPRINT(2, "vfifo_center(left): dtap=%u => " BTFLD_FMT " == "
			BTFLD_FMT " && %u", d, sticky_bit_chk,
			param->read_correct_mask, stop);

		if (stop == 1) {
			break;
		} else {
			for (i = 0; i < RW_MGR_MEM_DQ_PER_READ_DQS; i++) {
				if (bit_chk & 1) {
					/* Remember a passing test as the
					left_edge */
					left_edge[i] = d;
				} else {
					/* If a left edge has not been seen yet,
					then a future passing test will mark
					this edge as the right edge */
					if (left_edge[i] ==
						IO_IO_IN_DELAY_MAX + 1) {
						right_edge[i] = -(d + 1);
					}
				}
				bit_chk = bit_chk >> 1;
			}
		}
	}

	/* Reset DQ delay chains to 0 */
	scc_mgr_apply_group_dq_in_delay(write_group, test_bgn, 0);
	sticky_bit_chk = 0;
	for (i = RW_MGR_MEM_DQ_PER_READ_DQS - 1;; i--) {
		DPRINT(2, "vfifo_center: left_edge[%u]: %d right_edge[%u]: %d",
		       i, left_edge[i], i, right_edge[i]);

		/*
		 * Check for cases where we haven't found the left edge,
		 * which makes our assignment of the the right edge invalid.
		 * Reset it to the illegal value.
		 */
		if ((left_edge[i] == IO_IO_IN_DELAY_MAX + 1) && (
			right_edge[i] != IO_IO_IN_DELAY_MAX + 1)) {
			right_edge[i] = IO_IO_IN_DELAY_MAX + 1;
			DPRINT(2, "vfifo_center: reset right_edge[%u]: %d",
			       i, right_edge[i]);
		}

		/*
		 * Reset sticky bit (except for bits where we have seen
		 * both the left and right edge).
		 */
		sticky_bit_chk = sticky_bit_chk << 1;
		if ((left_edge[i] != IO_IO_IN_DELAY_MAX + 1) &&
		    (right_edge[i] != IO_IO_IN_DELAY_MAX + 1)) {
			sticky_bit_chk = sticky_bit_chk | 1;
		}

		if (i == 0)
			break;
	}

	/* Search for the right edge of the window for each bit */
	for (d = 0; d <= IO_DQS_IN_DELAY_MAX - start_dqs; d++) {
		scc_mgr_set_dqs_bus_in_delay(read_group, d + start_dqs);
		if (IO_SHIFT_DQS_EN_WHEN_SHIFT_DQS) {
			uint32_t delay = d + start_dqs_en;
			if (delay > IO_DQS_EN_DELAY_MAX)
				delay = IO_DQS_EN_DELAY_MAX;
			scc_mgr_set_dqs_en_delay(read_group, delay);
		}
		scc_mgr_load_dqs(read_group);

		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);

		/*
		 * Stop searching when the read test doesn't pass AND when
		 * we've seen a passing read on every bit.
		 */
		if (use_read_test) {
			stop = !rw_mgr_mem_calibrate_read_test(rank_bgn,
				read_group, NUM_READ_PB_TESTS, PASS_ONE_BIT,
				&bit_chk, 0, 0);
		} else {
			rw_mgr_mem_calibrate_write_test(rank_bgn, write_group,
							0, PASS_ONE_BIT,
							&bit_chk, 0);
			bit_chk = bit_chk >> (RW_MGR_MEM_DQ_PER_READ_DQS *
				(read_group - (write_group *
					RW_MGR_MEM_IF_READ_DQS_WIDTH /
					RW_MGR_MEM_IF_WRITE_DQS_WIDTH)));
			stop = (bit_chk == 0);
		}
		sticky_bit_chk = sticky_bit_chk | bit_chk;
		stop = stop && (sticky_bit_chk == param->read_correct_mask);

		DPRINT(2, "vfifo_center(right): dtap=%u => " BTFLD_FMT " == "
			BTFLD_FMT " && %u", d, sticky_bit_chk,
			param->read_correct_mask, stop);

		if (stop == 1) {
			break;
		} else {
			for (i = 0; i < RW_MGR_MEM_DQ_PER_READ_DQS; i++) {
				if (bit_chk & 1) {
					/* Remember a passing test as
					the right_edge */
					right_edge[i] = d;
				} else {
					if (d != 0) {
						/* If a right edge has not been
						seen yet, then a future passing
						test will mark this edge as the
						left edge */
						if (right_edge[i] ==
						IO_IO_IN_DELAY_MAX + 1) {
							left_edge[i] = -(d + 1);
						}
					} else {
						/* d = 0 failed, but it passed
						when testing the left edge,
						so it must be marginal,
						set it to -1 */
						if (right_edge[i] ==
							IO_IO_IN_DELAY_MAX + 1 &&
							left_edge[i] !=
							IO_IO_IN_DELAY_MAX
							+ 1) {
							right_edge[i] = -1;
						}
						/* If a right edge has not been
						seen yet, then a future passing
						test will mark this edge as the
						left edge */
						else if (right_edge[i] ==
							IO_IO_IN_DELAY_MAX +
							1) {
							left_edge[i] = -(d + 1);
						}
					}
				}

				DPRINT(2, "vfifo_center[r,d=%u]: ", d);
				DPRINT(2, "bit_chk_test=%d left_edge[%u]: %d ",
				       (int)(bit_chk & 1), i, left_edge[i]);
				DPRINT(2, "right_edge[%u]: %d", i,
				       right_edge[i]);
				bit_chk = bit_chk >> 1;
			}
		}
	}

	/* Check that all bits have a window */
	for (i = 0; i < RW_MGR_MEM_DQ_PER_READ_DQS; i++) {
		DPRINT(2, "vfifo_center: left_edge[%u]: %d right_edge[%u]: %d",
		       i, left_edge[i], i, right_edge[i]);
		BFM_GBL_SET(dq_read_left_edge[read_group][i], left_edge[i]);
		BFM_GBL_SET(dq_read_right_edge[read_group][i], right_edge[i]);
		if ((left_edge[i] == IO_IO_IN_DELAY_MAX + 1) || (right_edge[i]
			== IO_IO_IN_DELAY_MAX + 1)) {
			/*
			 * Restore delay chain settings before letting the loop
			 * in rw_mgr_mem_calibrate_vfifo to retry different
			 * dqs/ck relationships.
			 */
			scc_mgr_set_dqs_bus_in_delay(read_group, start_dqs);
			if (IO_SHIFT_DQS_EN_WHEN_SHIFT_DQS) {
				scc_mgr_set_dqs_en_delay(read_group,
							 start_dqs_en);
			}
			scc_mgr_load_dqs(read_group);
			IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);

			DPRINT(1, "vfifo_center: failed to find edge [%u]: %d %d",
			       i, left_edge[i], right_edge[i]);
			if (use_read_test) {
				set_failing_group_stage(read_group *
					RW_MGR_MEM_DQ_PER_READ_DQS + i,
					CAL_STAGE_VFIFO,
					CAL_SUBSTAGE_VFIFO_CENTER);
			} else {
				set_failing_group_stage(read_group *
					RW_MGR_MEM_DQ_PER_READ_DQS + i,
					CAL_STAGE_VFIFO_AFTER_WRITES,
					CAL_SUBSTAGE_VFIFO_CENTER);
			}
			return 0;
		}
	}

	/* Find middle of window for each DQ bit */
	mid_min = left_edge[0] - right_edge[0];
	min_index = 0;
	for (i = 1; i < RW_MGR_MEM_DQ_PER_READ_DQS; i++) {
		mid = left_edge[i] - right_edge[i];
		if (mid < mid_min) {
			mid_min = mid;
			min_index = i;
		}
	}

	/*
	 * -mid_min/2 represents the amount that we need to move DQS.
	 * If mid_min is odd and positive we'll need to add one to
	 * make sure the rounding in further calculations is correct
	 * (always bias to the right), so just add 1 for all positive values.
	 */
	if (mid_min > 0)
		mid_min++;

	mid_min = mid_min / 2;

	DPRINT(1, "vfifo_center: mid_min=%d (index=%u)", mid_min, min_index);

	/* Determine the amount we can change DQS (which is -mid_min) */
	orig_mid_min = mid_min;
#if ENABLE_DQS_IN_CENTERING
	new_dqs = start_dqs - mid_min;
	if (new_dqs > IO_DQS_IN_DELAY_MAX)
		new_dqs = IO_DQS_IN_DELAY_MAX;
	else if (new_dqs < 0)
		new_dqs = 0;

	mid_min = start_dqs - new_dqs;
	DPRINT(1, "vfifo_center: new mid_min=%d new_dqs=%d",
	       mid_min, new_dqs);

	if (IO_SHIFT_DQS_EN_WHEN_SHIFT_DQS) {
		if (start_dqs_en - mid_min > IO_DQS_EN_DELAY_MAX)
			mid_min += start_dqs_en - mid_min - IO_DQS_EN_DELAY_MAX;
		else if (start_dqs_en - mid_min < 0)
			mid_min += start_dqs_en - mid_min;
	}
	new_dqs = start_dqs - mid_min;
#else
	new_dqs = start_dqs;
	mid_min = 0;
#endif

	DPRINT(1, "vfifo_center: start_dqs=%d start_dqs_en=%d new_dqs=%d mid_min=%d",
	       start_dqs, IO_SHIFT_DQS_EN_WHEN_SHIFT_DQS ? start_dqs_en : -1,
	       new_dqs, mid_min);

	/* Initialize data for export structures */
	dqs_margin = IO_IO_IN_DELAY_MAX + 1;
	dq_margin  = IO_IO_IN_DELAY_MAX + 1;

	/* add delay to bring centre of all DQ windows to the same "level" */
	for (i = 0, p = test_bgn; i < RW_MGR_MEM_DQ_PER_READ_DQS; i++, p++) {
		/* Use values before divide by 2 to reduce round off error */
		shift_dq = (left_edge[i] - right_edge[i] -
			(left_edge[min_index] - right_edge[min_index]))/2  +
			(orig_mid_min - mid_min);

		DPRINT(2, "vfifo_center: before: shift_dq[%u]=%d", i,
		       shift_dq);

		if (shift_dq + (int32_t)READ_SCC_DQ_IN_DELAY(p) >
			(int32_t)IO_IO_IN_DELAY_MAX) {
			shift_dq = (int32_t)IO_IO_IN_DELAY_MAX -
				READ_SCC_DQ_IN_DELAY(i);
		} else if (shift_dq + (int32_t)READ_SCC_DQ_IN_DELAY(p) < 0) {
			shift_dq = -(int32_t)READ_SCC_DQ_IN_DELAY(p);
		}
		DPRINT(2, "vfifo_center: after: shift_dq[%u]=%d", i,
		       shift_dq);
		final_dq[i] = READ_SCC_DQ_IN_DELAY(p) + shift_dq;
		scc_mgr_set_dq_in_delay(write_group, p, final_dq[i]);
		scc_mgr_load_dq(p);

		DPRINT(2, "vfifo_center: margin[%u]=[%d,%d]", i,
		       left_edge[i] - shift_dq + (-mid_min),
		       right_edge[i] + shift_dq - (-mid_min));
		/* To determine values for export structures */
		if (left_edge[i] - shift_dq + (-mid_min) < dq_margin)
			dq_margin = left_edge[i] - shift_dq + (-mid_min);

		if (right_edge[i] + shift_dq - (-mid_min) < dqs_margin)
			dqs_margin = right_edge[i] + shift_dq - (-mid_min);
	}

#if ENABLE_DQS_IN_CENTERING
	final_dqs = new_dqs;
	if (IO_SHIFT_DQS_EN_WHEN_SHIFT_DQS)
		final_dqs_en = start_dqs_en - mid_min;
#else
	final_dqs = start_dqs;
	if (IO_SHIFT_DQS_EN_WHEN_SHIFT_DQS)
		final_dqs_en = start_dqs_en;
#endif

	/* Move DQS-en */
	if (IO_SHIFT_DQS_EN_WHEN_SHIFT_DQS) {
		scc_mgr_set_dqs_en_delay(read_group, final_dqs_en);
		scc_mgr_load_dqs(read_group);
	}

#if QDRII || RLDRAMX
	/*
	 * Move DQS. Do it gradually to minimize the chance of causing a timing
	 * failure in core FPGA logic driven by an input-strobe-derived clock.
	 */
	d = READ_SCC_DQS_IN_DELAY(read_group);
	while (d != final_dqs) {
		if (d > final_dqs)
			--d;
		else
			++d;

		scc_mgr_set_dqs_bus_in_delay(read_group, d);
		scc_mgr_load_dqs(read_group);
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	}
#else
	/* Move DQS */
	scc_mgr_set_dqs_bus_in_delay(read_group, final_dqs);
	scc_mgr_load_dqs(read_group);
#endif
	DPRINT(2, "vfifo_center: dq_margin=%d dqs_margin=%d",
	       dq_margin, dqs_margin);

	/* Do not remove this line as it makes sure all of our decisions
	have been applied */
	IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	return (dq_margin >= 0) && (dqs_margin >= 0);
}

/*
 * calibrate the read valid prediction FIFO.
 *
 *  - read valid prediction will consist of finding a good DQS enable phase,
 * DQS enable delay, DQS input phase, and DQS input delay.
 *  - we also do a per-bit deskew on the DQ lines.
 */
uint32_t rw_mgr_mem_calibrate_vfifo(uint32_t read_group, uint32_t test_bgn)
{
	uint32_t p, d, rank_bgn, sr;
	uint32_t dtaps_per_ptap;
	uint32_t tmp_delay;
	uint32_t bit_chk;
	uint32_t grp_calibrated;
	uint32_t write_group, write_test_bgn;
	uint32_t failed_substage;

	TRACE_FUNC("%u %u", read_group, test_bgn);

	/* update info for sims */
	reg_file_set_stage(CAL_STAGE_VFIFO);

	if (DDRX) {
		write_group = read_group;
		write_test_bgn = test_bgn;
	} else {
		write_group = read_group / (RW_MGR_MEM_IF_READ_DQS_WIDTH /
			RW_MGR_MEM_IF_WRITE_DQS_WIDTH);
		write_test_bgn = read_group * RW_MGR_MEM_DQ_PER_READ_DQS;
	}

	/* USER Determine number of delay taps for each phase tap */
	dtaps_per_ptap = 0;
	tmp_delay = 0;
	if (!QDRII) {
		while (tmp_delay < IO_DELAY_PER_OPA_TAP) {
			dtaps_per_ptap++;
			tmp_delay += IO_DELAY_PER_DQS_EN_DCHAIN_TAP;
		}
		dtaps_per_ptap--;
		tmp_delay = 0;
	}

	/* update info for sims */
	reg_file_set_group(read_group);

	grp_calibrated = 0;

	reg_file_set_sub_stage(CAL_SUBSTAGE_GUARANTEED_READ);
	failed_substage = CAL_SUBSTAGE_GUARANTEED_READ;

	for (d = 0; d <= dtaps_per_ptap && grp_calibrated == 0; d += 2) {
		if (DDRX || RLDRAMX) {
			/*
			 * In RLDRAMX we may be messing the delay of pins in
			 * the same write group but outside of the current read
			 * the group, but that's ok because we haven't
			 * calibrated output side yet.
			 */
			if (d > 0) {
				scc_mgr_apply_group_all_out_delay_add_all_ranks
				(write_group, write_test_bgn, d);
			}
		}

		for (p = 0; p <= IO_DQDQS_OUT_PHASE_MAX && grp_calibrated == 0;
			p++) {
			/* set a particular dqdqs phase */
			if (DDRX) {
				scc_mgr_set_dqdqs_output_phase_all_ranks(
					read_group, p);
			}

			/*
			 * Previous iteration may have failed as a result of
			 * ck/dqs or ck/dk violation, in which case the device
			 * may require special recovery.
			 */
			if (DDRX || RLDRAMX) {
				if (d != 0 || p != 0)
					recover_mem_device_after_ck_dqs_violation();
			}

			DPRINT(1, "calibrate_vfifo: g=%u p=%u d=%u",
			       read_group, p, d);
			BFM_GBL_SET(gwrite_pos[read_group].p, p);
			BFM_GBL_SET(gwrite_pos[read_group].d, d);

			/*
			 * Load up the patterns used by read calibration
			 * using current DQDQS phase.
			 */
			rw_mgr_mem_calibrate_read_load_patterns(0, 1);
#if DDRX
			if (!(gbl->phy_debug_mode_flags &
				PHY_DEBUG_DISABLE_GUARANTEED_READ)) {
				if (!rw_mgr_mem_calibrate_read_test_patterns_all_ranks
				    (read_group, 1, &bit_chk)) {
					DPRINT(1, "Guaranteed read test failed:");
					DPRINT(1, " g=%u p=%u d=%u", read_group,
					       p, d);
					break;
				}
			}
#endif

/* case:56390 */
			grp_calibrated = 1;
	if (rw_mgr_mem_calibrate_vfifo_find_dqs_en_phase_sweep_dq_in_delay
		(write_group, read_group, test_bgn)) {
				/*
				 * USER Read per-bit deskew can be done on a
				 * per shadow register basis.
				 */
				for (rank_bgn = 0, sr = 0;
					rank_bgn < RW_MGR_MEM_NUMBER_OF_RANKS;
					rank_bgn += NUM_RANKS_PER_SHADOW_REG,
					++sr) {
					/*
					 * Determine if this set of ranks
					 * should be skipped entirely.
					 */
					if (!param->skip_shadow_regs[sr]) {
						/* Select shadow register set */
						select_shadow_regs_for_update
						(rank_bgn, read_group, 1);

						/*
						 * If doing read after write
						 * calibration, do not update
						 * FOM, now - do it then.
						 */
#if READ_AFTER_WRITE_CALIBRATION
					if (!rw_mgr_mem_calibrate_vfifo_center
						(rank_bgn, write_group,
						read_group, test_bgn, 1, 0)) {
#else
					if (!rw_mgr_mem_calibrate_vfifo_center
						(rank_bgn, write_group,
						read_group, test_bgn, 1, 1)) {
#endif
							grp_calibrated = 0;
							failed_substage =
						CAL_SUBSTAGE_VFIFO_CENTER;
						}
					}
				}
			} else {
				grp_calibrated = 0;
				failed_substage = CAL_SUBSTAGE_DQS_EN_PHASE;
			}
		}
	}

	if (grp_calibrated == 0) {
		set_failing_group_stage(write_group, CAL_STAGE_VFIFO,
					failed_substage);
		return 0;
	}

	/*
	 * Reset the delay chains back to zero if they have moved > 1
	 * (check for > 1 because loop will increase d even when pass in
	 * first case).
	 */
	if (DDRX || RLDRAMII) {
		if (d > 2)
			scc_mgr_zero_group(write_group, write_test_bgn, 1);
	}


	return 1;
}

#if READ_AFTER_WRITE_CALIBRATION
/* VFIFO Calibration -- Read Deskew Calibration after write deskew */
uint32_t rw_mgr_mem_calibrate_vfifo_end(uint32_t read_group, uint32_t test_bgn)
{
	uint32_t rank_bgn, sr;
	uint32_t grp_calibrated;
	uint32_t write_group;

	TRACE_FUNC("%u %u", read_group, test_bgn);

	/* update info for sims */

	reg_file_set_stage(CAL_STAGE_VFIFO_AFTER_WRITES);
	reg_file_set_sub_stage(CAL_SUBSTAGE_VFIFO_CENTER);

	if (DDRX) {
		write_group = read_group;
	} else {
		write_group = read_group / (RW_MGR_MEM_IF_READ_DQS_WIDTH /
			RW_MGR_MEM_IF_WRITE_DQS_WIDTH);
	}

	/* update info for sims */
	reg_file_set_group(read_group);

	grp_calibrated = 1;
	/* Read per-bit deskew can be done on a per shadow register basis */
	for (rank_bgn = 0, sr = 0; rank_bgn < RW_MGR_MEM_NUMBER_OF_RANKS;
		rank_bgn += NUM_RANKS_PER_SHADOW_REG, ++sr) {
		/* Determine if this set of ranks should be skipped entirely */
		if (!param->skip_shadow_regs[sr]) {
			/* Select shadow register set */
			select_shadow_regs_for_update(rank_bgn, read_group, 1);

		/* This is the last calibration round, update FOM here */
			if (!rw_mgr_mem_calibrate_vfifo_center(rank_bgn,
								write_group,
								read_group,
								test_bgn, 0,
								1)) {
				grp_calibrated = 0;
			}
		}
	}


	if (grp_calibrated == 0) {
		set_failing_group_stage(write_group,
					CAL_STAGE_VFIFO_AFTER_WRITES,
					CAL_SUBSTAGE_VFIFO_CENTER);
		return 0;
	}

	return 1;
}
#endif


/* Calibrate LFIFO to find smallest read latency */
uint32_t rw_mgr_mem_calibrate_lfifo(void)
{
	uint32_t found_one;
	uint32_t bit_chk;

	TRACE_FUNC();
	BFM_STAGE("lfifo");

	/* update info for sims */
	reg_file_set_stage(CAL_STAGE_LFIFO);
	reg_file_set_sub_stage(CAL_SUBSTAGE_READ_LATENCY);

	/* Load up the patterns used by read calibration for all ranks */
	rw_mgr_mem_calibrate_read_load_patterns(0, 1);
	found_one = 0;

	do {
		IOWR_32DIRECT(PHY_MGR_PHY_RLAT, 0, gbl->curr_read_lat);
		DPRINT(2, "lfifo: read_lat=%u", gbl->curr_read_lat);

		if (!rw_mgr_mem_calibrate_read_test_all_ranks(0,
							      NUM_READ_TESTS,
							      PASS_ALL_BITS,
							      &bit_chk, 1)) {
			break;
		}

		found_one = 1;
		/* reduce read latency and see if things are working */
		/* correctly */
		gbl->curr_read_lat--;
	} while (gbl->curr_read_lat > 0);

	/* reset the fifos to get pointers to known state */

	IOWR_32DIRECT(PHY_MGR_CMD_FIFO_RESET, 0, 0);

	if (found_one) {
		/* add a fudge factor to the read latency that was determined */
		gbl->curr_read_lat += 2;
		IOWR_32DIRECT(PHY_MGR_PHY_RLAT, 0, gbl->curr_read_lat);
		DPRINT(2, "lfifo: success: using read_lat=%u",
		       gbl->curr_read_lat);

		return 1;
	} else {
		set_failing_group_stage(0xff, CAL_STAGE_LFIFO,
					CAL_SUBSTAGE_READ_LATENCY);

		DPRINT(2, "lfifo: failed at initial read_lat=%u",
		       gbl->curr_read_lat);

		return 0;
	}
}

/*
 * issue write test command.
 * two variants are provided. one that just tests a write pattern and
 * another that tests datamask functionality.
 */

#if QDRII
void rw_mgr_mem_calibrate_write_test_issue(uint32_t group, uint32_t test_dm)
{
	uint32_t quick_write_mode = (((STATIC_CALIB_STEPS) & CALIB_SKIP_WRITES) &&
		ENABLE_SUPER_QUICK_CALIBRATION);
	/*
	 * CNTR 1 - This is used to ensure enough time elapses for
	 * read data to come back.
	 */
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, 0x30);

	if (test_dm) {
		IOWR_32DIRECT(RW_MGR_RESET_READ_DATAPATH, 0, 0);
		if (quick_write_mode)
			IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x08);
		else
			IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x40);

		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0,
			      __RW_MGR_LFSR_WR_RD_DM_BANK_0);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
			      __RW_MGR_LFSR_WR_RD_DM_BANK_0_WAIT);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, (group) << 2,
			      __RW_MGR_LFSR_WR_RD_DM_BANK_0);
	} else {
		IOWR_32DIRECT(RW_MGR_RESET_READ_DATAPATH, 0, 0);
		if (quick_write_mode)
			IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x08);
		else
			IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x40);

		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0,
			      __RW_MGR_LFSR_WR_RD_BANK_0);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
			      __RW_MGR_LFSR_WR_RD_BANK_0_WAIT);
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, (group) << 2,
			      __RW_MGR_LFSR_WR_RD_BANK_0);
	}
}
#else
void rw_mgr_mem_calibrate_write_test_issue(uint32_t group, uint32_t test_dm)
{
	uint32_t mcc_instruction;
	uint32_t quick_write_mode = (((STATIC_CALIB_STEPS) & CALIB_SKIP_WRITES) &&
		ENABLE_SUPER_QUICK_CALIBRATION);
	uint32_t rw_wl_nop_cycles;

	/*
	 * Set counter and jump addresses for the right
	 * number of NOP cycles.
	 * The number of supported NOP cycles can range from -1 to infinity
	 * Three different cases are handled:
	 *
	 * 1. For a number of NOP cycles greater than 0, the RW Mgr looping
	 *    mechanism will be used to insert the right number of NOPs
	 *
	 * 2. For a number of NOP cycles equals to 0, the micro-instruction
	 *    issuing the write command will jump straight to the
	 *    micro-instruction that turns on DQS (for DDRx), or outputs write
	 *    data (for RLD), skipping
	 *    the NOP micro-instruction all together
	 *
	 * 3. A number of NOP cycles equal to -1 indicates that DQS must be
	 *    turned on in the same micro-instruction that issues the write
	 *    command. Then we need
	 *    to directly jump to the micro-instruction that sends out the data
	 *
	 * NOTE: Implementing this mechanism uses 2 RW Mgr jump-counters
	 *       (2 and 3). One jump-counter (0) is used to perform multiple
	 *       write-read operations.
	 *       one counter left to issue this command in "multiple-group" mode
	 */

#if MULTIPLE_AFI_WLAT
	rw_wl_nop_cycles = gbl->rw_wl_nop_cycles_per_group[group];
#else
	rw_wl_nop_cycles = gbl->rw_wl_nop_cycles;
#endif

	if (rw_wl_nop_cycles == -1) {
		#if DDRX
		/*
		 * CNTR 2 - We want to execute the special write operation that
		 * turns on DQS right away and then skip directly to the
		 * instruction that sends out the data. We set the counter to a
		 * large number so that the jump is always taken.
		 */
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, 0xFF);

		/* CNTR 3 - Not used */
		if (test_dm) {
			mcc_instruction = __RW_MGR_LFSR_WR_RD_DM_BANK_0_WL_1;
			IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
				      __RW_MGR_LFSR_WR_RD_DM_BANK_0_DATA);
			IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_3, 0,
				      __RW_MGR_LFSR_WR_RD_DM_BANK_0_NOP);
		} else {
			mcc_instruction = __RW_MGR_LFSR_WR_RD_BANK_0_WL_1;
			IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
				      __RW_MGR_LFSR_WR_RD_BANK_0_DATA);
			IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_3, 0,
				      __RW_MGR_LFSR_WR_RD_BANK_0_NOP);
		}

		#endif
	} else if (rw_wl_nop_cycles == 0) {
		#if DDRX
		/*
		 * CNTR 2 - We want to skip the NOP operation and go straight
		 * to the DQS enable instruction. We set the counter to a large
		 * number so that the jump is always taken.
		 */
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, 0xFF);

		/* CNTR 3 - Not used */
		if (test_dm) {
			mcc_instruction = __RW_MGR_LFSR_WR_RD_DM_BANK_0;
			IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
				      __RW_MGR_LFSR_WR_RD_DM_BANK_0_DQS);
		} else {
			mcc_instruction = __RW_MGR_LFSR_WR_RD_BANK_0;
			IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
				      __RW_MGR_LFSR_WR_RD_BANK_0_DQS);
		}
		#endif

		#if RLDRAMX
		/*
		 * CNTR 2 - We want to skip the NOP operation and go straight
		 * to the write data instruction. We set the counter to a large
		 * number so that the jump is always taken.
		 */
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, 0xFF);

		/* CNTR 3 - Not used */
		if (test_dm) {
			mcc_instruction = __RW_MGR_LFSR_WR_RD_DM_BANK_0;
			IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
				      __RW_MGR_LFSR_WR_RD_DM_BANK_0_DATA);
		} else {
			mcc_instruction = __RW_MGR_LFSR_WR_RD_BANK_0;
			IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
				      __RW_MGR_LFSR_WR_RD_BANK_0_DATA);
		}
		#endif
	} else {
		/* CNTR 2 - In this case we want to execute the next instruction
		and NOT take the jump. So we set the counter to 0. The jump
		address doesn't count */
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, 0x0);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0, 0x0);

		/* CNTR 3 - Set the nop counter to the number of cycles we
		need to loop for, minus 1 */
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_3, 0, rw_wl_nop_cycles - 1);
		if (test_dm) {
			mcc_instruction = __RW_MGR_LFSR_WR_RD_DM_BANK_0;
			IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_3, 0,
				      __RW_MGR_LFSR_WR_RD_DM_BANK_0_NOP);
		} else {
			mcc_instruction = __RW_MGR_LFSR_WR_RD_BANK_0;
			IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_3, 0,
				      __RW_MGR_LFSR_WR_RD_BANK_0_NOP);
		}
	}

	IOWR_32DIRECT(RW_MGR_RESET_READ_DATAPATH, 0, 0);

	if (quick_write_mode)
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x08);
	else
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x40);

	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, mcc_instruction);

	/*
	 * CNTR 1 - This is used to ensure enough time elapses
	 * for read data to come back.
	 */
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, 0x30);

	if (test_dm) {
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
			      __RW_MGR_LFSR_WR_RD_DM_BANK_0_WAIT);
	} else {
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
			      __RW_MGR_LFSR_WR_RD_BANK_0_WAIT);
	}

	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, (group << 2), mcc_instruction);
}
#endif

/* Test writes, can check for a single bit pass or multiple bit pass */
uint32_t rw_mgr_mem_calibrate_write_test(uint32_t rank_bgn,
	uint32_t write_group, uint32_t use_dm, uint32_t all_correct,
	uint32_t *bit_chk, uint32_t all_ranks)
{
	uint32_t r;
	uint32_t correct_mask_vg;
	uint32_t tmp_bit_chk;
	uint32_t vg;
	uint32_t rank_end = all_ranks ? RW_MGR_MEM_NUMBER_OF_RANKS :
		(rank_bgn + NUM_RANKS_PER_SHADOW_REG);

	*bit_chk = param->write_correct_mask;
	correct_mask_vg = param->write_correct_mask_vg;

	for (r = rank_bgn; r < rank_end; r++) {
		if (param->skip_ranks[r]) {
			/* request to skip the rank */

			continue;
		}

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_READ_WRITE);

		tmp_bit_chk = 0;
		for (vg = RW_MGR_MEM_VIRTUAL_GROUPS_PER_WRITE_DQS-1; ; vg--) {
			/* reset the fifos to get pointers to known state */
			IOWR_32DIRECT(PHY_MGR_CMD_FIFO_RESET, 0, 0);

			tmp_bit_chk = tmp_bit_chk <<
				(RW_MGR_MEM_DQ_PER_WRITE_DQS /
				RW_MGR_MEM_VIRTUAL_GROUPS_PER_WRITE_DQS);
			rw_mgr_mem_calibrate_write_test_issue(write_group *
				RW_MGR_MEM_VIRTUAL_GROUPS_PER_WRITE_DQS+vg,
				use_dm);

			tmp_bit_chk = tmp_bit_chk | (correct_mask_vg &
				~(IORD_32DIRECT(BASE_RW_MGR, 0)));
			DPRINT(2, "write_test(%u,%u,%u) :[%u,%u] "
				BTFLD_FMT " & ~%x => " BTFLD_FMT " => "
				BTFLD_FMT, write_group, use_dm, all_correct,
				r, vg, correct_mask_vg,
				IORD_32DIRECT(BASE_RW_MGR, 0), correct_mask_vg
				& ~IORD_32DIRECT(BASE_RW_MGR, 0),
				tmp_bit_chk);

			if (vg == 0)
				break;
		}
		*bit_chk &= tmp_bit_chk;
	}

	if (all_correct) {
		set_rank_and_odt_mask(0, RW_MGR_ODT_MODE_OFF);
		DPRINT(2, "write_test(%u,%u,ALL) : " BTFLD_FMT " == "
			BTFLD_FMT " => %lu", write_group, use_dm,
			*bit_chk, param->write_correct_mask,
			(long unsigned int)(*bit_chk ==
			param->write_correct_mask));
		return *bit_chk == param->write_correct_mask;
	} else {
		set_rank_and_odt_mask(0, RW_MGR_ODT_MODE_OFF);
		DPRINT(2, "write_test(%u,%u,ONE) : " BTFLD_FMT " != ",
		       write_group, use_dm, *bit_chk);
		DPRINT(2, "%lu" " => %lu", (long unsigned int)0,
			(long unsigned int)(*bit_chk != 0));
		return *bit_chk != 0x00;
	}
}

static inline uint32_t rw_mgr_mem_calibrate_write_test_all_ranks
(uint32_t write_group, uint32_t use_dm, uint32_t all_correct, uint32_t *bit_chk)
{
	return rw_mgr_mem_calibrate_write_test(0, write_group,
		use_dm, all_correct, bit_chk, 1);
}


/* level the write operations */

#if DYNAMIC_CALIBRATION_MODE || STATIC_QUICK_CALIBRATION

#if QDRII

/* Write Levelling -- Quick Calibration */
uint32_t rw_mgr_mem_calibrate_wlevel(uint32_t g, uint32_t test_bgn)
{
	TRACE_FUNC("%lu %lu", g, test_bgn);

	return 0;
}

#endif

#if RLDRAMX
#if !ENABLE_SUPER_QUICK_CALIBRATION

/* Write Levelling -- Quick Calibration */
uint32_t rw_mgr_mem_calibrate_wlevel(uint32_t g, uint32_t test_bgn)
{
	uint32_t d;
	uint32_t bit_chk;

	TRACE_FUNC("%lu %lu", g, test_bgn);

	/* update info for sims */

	reg_file_set_stage(CAL_STAGE_WLEVEL);
	reg_file_set_sub_stage(CAL_SUBSTAGE_WORKING_DELAY);
	reg_file_set_group(g);

	for (d = 0; d <= IO_IO_OUT1_DELAY_MAX; d++) {
		scc_mgr_apply_group_all_out_delay_all_ranks(g, test_bgn, d);

		if (rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
							      PASS_ONE_BIT,
							      &bit_chk)) {
			break;
		}
	}

	if (d > IO_IO_OUT1_DELAY_MAX) {
		set_failing_group_stage(g, CAL_STAGE_WLEVEL,
					CAL_SUBSTAGE_WORKING_DELAY);

		return 0;
	}

	return 1;
}

#else

/* Write Levelling -- Super Quick Calibration */
uint32_t rw_mgr_mem_calibrate_wlevel(uint32_t g, uint32_t test_bgn)
{
	uint32_t d;
	uint32_t bit_chk;

	TRACE_FUNC("%lu %lu", g, test_bgn);

	/* The first call to this function will calibrate all groups */
	if (g != 0)
		return 1;

	/* update info for sims */
	reg_file_set_stage(CAL_STAGE_WLEVEL);
	reg_file_set_sub_stage(CAL_SUBSTAGE_WORKING_DELAY);
	reg_file_set_group(g);

	for (d = 0; d <= IO_IO_OUT1_DELAY_MAX; d++) {
		scc_mgr_apply_group_all_out_delay_all_ranks(g, test_bgn, d);

		if (rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
							      PASS_ONE_BIT,
							      &bit_chk)) {
			break;
		}
	}

	if (d > IO_IO_OUT1_DELAY_MAX) {
		set_failing_group_stage(g, CAL_STAGE_WLEVEL,
					CAL_SUBSTAGE_WORKING_DELAY);

		return 0;
	}

	reg_file_set_sub_stage(CAL_SUBSTAGE_WLEVEL_COPY);

	/* Now copy the calibration settings to all other groups */
	for (g = 1, test_bgn = RW_MGR_MEM_DQ_PER_WRITE_DQS;
		g < RW_MGR_MEM_IF_WRITE_DQS_WIDTH; g++,
		test_bgn += RW_MGR_MEM_DQ_PER_WRITE_DQS) {
		scc_mgr_apply_group_all_out_delay_all_ranks(g, test_bgn, d);

		/* Verify that things worked as expected */
		if (!rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
							       PASS_ONE_BIT,
							       &bit_chk)) {
			set_failing_group_stage(g, CAL_STAGE_WLEVEL,
						CAL_SUBSTAGE_WLEVEL_COPY);
			return 0;
		}
	}

	return 1;
}

#endif
#endif

#if DDRX
#if !ENABLE_SUPER_QUICK_CALIBRATION

/* Write Levelling -- Quick Calibration */
uint32_t rw_mgr_mem_calibrate_wlevel(uint32_t g, uint32_t test_bgn)
{
	uint32_t p;
	uint32_t bit_chk;

	TRACE_FUNC("%lu %lu", g, test_bgn);

	/* update info for sims */

	reg_file_set_stage(CAL_STAGE_WLEVEL);
	reg_file_set_sub_stage(CAL_SUBSTAGE_WORKING_DELAY);

	/* maximum phases for the sweep */

	/* starting phases */

	/* update info for sims */
	reg_file_set_group(g);

	for (p = 0; p <= IO_DQDQS_OUT_PHASE_MAX; p++) {
		scc_mgr_set_dqdqs_output_phase_all_ranks(g, p);

		if (rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
							      PASS_ONE_BIT,
							      &bit_chk)) {
			break;
		}
	}

	if (p > IO_DQDQS_OUT_PHASE_MAX) {
		set_failing_group_stage(g, CAL_STAGE_WLEVEL,
					CAL_SUBSTAGE_WORKING_DELAY);

		return 0;
	}

	return 1;
}

#else

/* Write Levelling -- Super Quick Calibration */
uint32_t rw_mgr_mem_calibrate_wlevel(uint32_t g, uint32_t test_bgn)
{
	uint32_t p;
	uint32_t bit_chk;

	TRACE_FUNC("%lu %lu", g, test_bgn);

	/* The first call to this function will calibrate all groups */
	if (g != 0)
		return 1;

	/* update info for sims */

	reg_file_set_stage(CAL_STAGE_WLEVEL);
	reg_file_set_sub_stage(CAL_SUBSTAGE_WORKING_DELAY);

	/* maximum phases for the sweep */

	/* starting phases */

	/* update info for sims */
	reg_file_set_group(g);

	for (p = 0; p <= IO_DQDQS_OUT_PHASE_MAX; p++) {
		scc_mgr_set_dqdqs_output_phase_all_ranks(g, p);

		if (rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
							      PASS_ONE_BIT,
							      &bit_chk)) {
			break;
		}
	}

	if (p > IO_DQDQS_OUT_PHASE_MAX) {
		set_failing_group_stage(g, CAL_STAGE_WLEVEL,
					CAL_SUBSTAGE_WORKING_DELAY);

		return 0;
	}

	reg_file_set_sub_stage(CAL_SUBSTAGE_WLEVEL_COPY);

	/* Now copy the calibration settings to all other groups */
	for (g = 1, test_bgn = RW_MGR_MEM_DQ_PER_READ_DQS;
		(g < RW_MGR_MEM_IF_READ_DQS_WIDTH); g++,
		test_bgn += RW_MGR_MEM_DQ_PER_READ_DQS) {
		IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0, g);
		scc_mgr_set_dqdqs_output_phase_all_ranks(g, p);

		/* Verify that things worked as expected */
		if (!rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
							       PASS_ONE_BIT,
							       &bit_chk)) {
			set_failing_group_stage(g, CAL_STAGE_WLEVEL,
						CAL_SUBSTAGE_WLEVEL_COPY);
			IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0, 0);
			return 0;
		}
	}

	IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0, 0);
	return 1;
}

#endif
#endif

#endif

#if QDRII
/* Write Levelling -- Full Calibration */
uint32_t rw_mgr_mem_calibrate_wlevel(uint32_t g, uint32_t test_bgn)
{
	TRACE_FUNC("%lu %lu", g, test_bgn);

	return 0;
}
#endif

#if RLDRAMX
/* Write Levelling -- Full Calibration */
uint32_t rw_mgr_mem_calibrate_wlevel(uint32_t g, uint32_t test_bgn)
{
	uint32_t d;
	uint32_t bit_chk;
	uint32_t work_bgn, work_end;
	uint32_t d_bgn, d_end;
	uint32_t found_begin;

	TRACE_FUNC("%lu %lu", g, test_bgn);
	BFM_STAGE("wlevel");

	ALTERA_ASSERT(g < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);

	/* update info for sims */
	reg_file_set_stage(CAL_STAGE_WLEVEL);
	reg_file_set_sub_stage(CAL_SUBSTAGE_WORKING_DELAY);

	/* maximum delays for the sweep */

	/* update info for sims */
	reg_file_set_group(g);

	/* starting and end range where writes work */
	scc_mgr_spread_out2_delay_all_ranks(g, test_bgn);

	work_bgn = 0;
	work_end = 0;

	/* step 1: find first working dtap, increment in dtaps */
	found_begin = 0;
	for (d = 0; d <= IO_IO_OUT1_DELAY_MAX; d++,
	     work_bgn += IO_DELAY_PER_DCHAIN_TAP) {
		DPRINT(2, "wlevel: begin: d=%lu", d);
		scc_mgr_apply_group_all_out_delay_all_ranks(g, test_bgn, d);

		if (rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
							      PASS_ONE_BIT,
							      &bit_chk)) {
			found_begin = 1;
			d_bgn = d;
			break;
		} else {
			recover_mem_device_after_ck_dqs_violation();
		}
	}

	if (!found_begin) {
		/* fail, cannot find first working delay */
		DPRINT(2, "wlevel: failed to find first working delay", d);

		set_failing_group_stage(g, CAL_STAGE_WLEVEL,
					CAL_SUBSTAGE_WORKING_DELAY);

		return 0;
	}

	DPRINT(2, "wlevel: found begin d=%lu work_bgn=%lu", d_bgn, work_bgn);
	BFM_GBL_SET(dqs_wlevel_left_edge[g].d, d_bgn);
	BFM_GBL_SET(dqs_wlevel_left_edge[g].ps, work_bgn);

	reg_file_set_sub_stage(CAL_SUBSTAGE_LAST_WORKING_DELAY);

	/* step 2 : find first non-working dtap, increment in dtaps */
	work_end = work_bgn;
	d = d + 1;
	for (; d <= IO_IO_OUT1_DELAY_MAX; d++,
		work_end += IO_DELAY_PER_DCHAIN_TAP) {
		DPRINT(2, "wlevel: end: d=%lu", d);
		scc_mgr_apply_group_all_out_delay_all_ranks(g, test_bgn, d);

		if (!rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
							       PASS_ONE_BIT,
							       &bit_chk)) {
			recover_mem_device_after_ck_dqs_violation();
			break;
		}
	}
	d_end = d - 1;

	if (d_end >= d_bgn) {
		/* we have a working range */
	} else {
		/* nil range */
		/*Note: don't think this is possible */
		set_failing_group_stage(g, CAL_STAGE_WLEVEL,
					CAL_SUBSTAGE_LAST_WORKING_DELAY);

		return 0;
	}

	DPRINT(2, "wlevel: found end: d=%lu work_end=%lu", d_end, work_end);
	BFM_GBL_SET(dqs_wlevel_right_edge[g].d, d_end);
	BFM_GBL_SET(dqs_wlevel_right_edge[g].ps, work_end);

	/* center */
	d = (d_end + d_bgn) / 2;

	DPRINT(2, "wlevel: found middle: d=%lu work_mid=%lu", d,
	       (work_end + work_bgn)/2);
	BFM_GBL_SET(dqs_wlevel_mid[g].d, d);
	BFM_GBL_SET(dqs_wlevel_mid[g].ps, (work_end + work_bgn)/2);

	scc_mgr_zero_group(g, test_bgn, 1);
	scc_mgr_apply_group_all_out_delay_add_all_ranks(g, test_bgn, d);

	return 1;
}
#endif


#if DDRX
/* Write Levelling -- Full Calibration */
uint32_t rw_mgr_mem_calibrate_wlevel(uint32_t g, uint32_t test_bgn)
{
	uint32_t p, d;

#if CALIBRATE_BIT_SLIPS
#if QUARTER_RATE_MODE
	int32_t num_additional_fr_cycles = 3;
#elif HALF_RATE_MODE
	int32_t num_additional_fr_cycles = 1;
#else
	int32_t num_additional_fr_cycles = 0;
#endif
#if MULTIPLE_AFI_WLAT
	num_additional_fr_cycles++;
#endif
#else
	uint32_t num_additional_fr_cycles = 0;
#endif
	uint32_t bit_chk;
	uint32_t work_bgn, work_end, work_mid;
	uint32_t tmp_delay;
	uint32_t found_begin;
	uint32_t dtaps_per_ptap;

	TRACE_FUNC("%u %u", g, test_bgn);
	BFM_STAGE("wlevel");


	/* update info for sims */
	reg_file_set_stage(CAL_STAGE_WLEVEL);
	reg_file_set_sub_stage(CAL_SUBSTAGE_WORKING_DELAY);

	/* maximum phases for the sweep */
#if USE_DQS_TRACKING
	dtaps_per_ptap = IORD_32DIRECT(REG_FILE_DTAPS_PER_PTAP, 0);
#else
	dtaps_per_ptap = 0;
	tmp_delay = 0;
	while (tmp_delay < IO_DELAY_PER_OPA_TAP) {
		dtaps_per_ptap++;
		tmp_delay += IO_DELAY_PER_DCHAIN_TAP;
	}
	dtaps_per_ptap--;
	tmp_delay = 0;
#endif

	/* starting phases */

	/* update info for sims */
	reg_file_set_group(g);

	/* starting and end range where writes work */
	scc_mgr_spread_out2_delay_all_ranks(g, test_bgn);

	work_bgn = 0;
	work_end = 0;

	/* step 1: find first working phase, increment in ptaps, and then in
	dtaps if ptaps doesn't find a working phase */
	found_begin = 0;
	tmp_delay = 0;
	for (d = 0; d <= dtaps_per_ptap; d++, tmp_delay +=
		IO_DELAY_PER_DCHAIN_TAP) {
		scc_mgr_apply_group_all_out_delay_all_ranks(g, test_bgn, d);

		work_bgn = tmp_delay;

		for (p = 0; p <= IO_DQDQS_OUT_PHASE_MAX +
			num_additional_fr_cycles*IO_DLL_CHAIN_LENGTH;
			p++, work_bgn += IO_DELAY_PER_OPA_TAP) {
			DPRINT(2, "wlevel: begin-1: p=%u d=%u", p, d);
			scc_mgr_set_dqdqs_output_phase_all_ranks(g, p);

			if (rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
								      PASS_ONE_BIT,
								      &bit_chk)) {
				found_begin = 1;
				break;
			}
		}

		if (found_begin)
			break;
	}

	if (p > IO_DQDQS_OUT_PHASE_MAX + num_additional_fr_cycles *
		IO_DLL_CHAIN_LENGTH) {
		/* fail, cannot find first working phase */
		set_failing_group_stage(g, CAL_STAGE_WLEVEL,
					CAL_SUBSTAGE_WORKING_DELAY);

		return 0;
	}

	DPRINT(2, "wlevel: first valid p=%u d=%u", p, d);

	reg_file_set_sub_stage(CAL_SUBSTAGE_LAST_WORKING_DELAY);

	/*
	 * If d is 0 then the working window covers a phase tap and we can
	 * follow the old procedure otherwise, we've found the beginning, and
	 * we need to increment the dtaps until we find the end.
	 */
	if (d == 0) {
		COV(WLEVEL_PHASE_PTAP_OVERLAP);
		work_end = work_bgn + IO_DELAY_PER_OPA_TAP;

		/* step 2: if we have room, back off by one and increment
		in dtaps */

		if (p > 0) {
			scc_mgr_set_dqdqs_output_phase_all_ranks(g, p - 1);

			tmp_delay = work_bgn - IO_DELAY_PER_OPA_TAP;

			for (d = 0; d <= IO_IO_OUT1_DELAY_MAX &&
				tmp_delay < work_bgn; d++,
				tmp_delay += IO_DELAY_PER_DCHAIN_TAP) {
				DPRINT(2, "wlevel: begin-2: p=%u d=%u",
				       (p-1), d);
				scc_mgr_apply_group_all_out_delay_all_ranks
				(g, test_bgn, d);

				if (rw_mgr_mem_calibrate_write_test_all_ranks
					(g, 0, PASS_ONE_BIT, &bit_chk)) {
					work_bgn = tmp_delay;
					break;
				}
			}

			scc_mgr_apply_group_all_out_delay_all_ranks(g,
								    test_bgn,
								    0);
		} else {
			DPRINT(2, "wlevel: found begin-B: p=%u d=%u ps=%u",
				p, d, work_bgn);

			BFM_GBL_SET(dqs_wlevel_left_edge[g].p, p);
			BFM_GBL_SET(dqs_wlevel_left_edge[g].d, d);
			BFM_GBL_SET(dqs_wlevel_left_edge[g].ps, work_bgn);
		}

		/* step 3: go forward from working phase to non working phase,
		increment in ptaps */

		for (p = p + 1; p <= IO_DQDQS_OUT_PHASE_MAX +
			num_additional_fr_cycles * IO_DLL_CHAIN_LENGTH; p++,
			work_end += IO_DELAY_PER_OPA_TAP) {
			DPRINT(2, "wlevel: end-0: p=%u d=0", p);
			scc_mgr_set_dqdqs_output_phase_all_ranks(g, p);

			if (!rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
								       PASS_ONE_BIT,
								       &bit_chk)) {
				break;
			}
		}

		/*
		 * step 4: back off one from last, increment in dtaps.
		 * The actual increment is done outside the if/else statement
		 * since it is shared with other code.
		 */
		p -= 1;

		scc_mgr_set_dqdqs_output_phase_all_ranks(g, p);

		work_end -= IO_DELAY_PER_OPA_TAP;
		d = 0;
	} else {
		/*
		 * step 5: Window doesn't cover phase tap, just increment
		 * dtaps until failure.
		 * The actual increment is done outside the if/else statement
		 * since it is shared with other code.
		 */
		COV(WLEVEL_PHASE_PTAP_NO_OVERLAP);
		work_end = work_bgn;
		DPRINT(2, "wlevel: found begin-C: p=%u d=%u ps=%u", p,
		       d, work_bgn);
		BFM_GBL_SET(dqs_wlevel_left_edge[g].p, p);
		BFM_GBL_SET(dqs_wlevel_left_edge[g].d, d);
		BFM_GBL_SET(dqs_wlevel_left_edge[g].ps, work_bgn);
	}

	/* The actual increment until failure */
	for (; d <= IO_IO_OUT1_DELAY_MAX; d++, work_end +=
	     IO_DELAY_PER_DCHAIN_TAP) {
		DPRINT(2, "wlevel: end: p=%u d=%u", p, d);
		scc_mgr_apply_group_all_out_delay_all_ranks(g, test_bgn, d);

		if (!rw_mgr_mem_calibrate_write_test_all_ranks(g, 0,
							       PASS_ONE_BIT,
							       &bit_chk))
			break;
	}
	scc_mgr_zero_group(g, test_bgn, 1);

	work_end -= IO_DELAY_PER_DCHAIN_TAP;

	if (work_end >= work_bgn) {
		/* we have a working range */
	} else {
		/* nil range */
		set_failing_group_stage(g, CAL_STAGE_WLEVEL,
					CAL_SUBSTAGE_LAST_WORKING_DELAY);
		return 0;
	}

	DPRINT(2, "wlevel: found end: p=%u d=%u; range: [%u,%u]", p,
	       d-1, work_bgn, work_end);
	BFM_GBL_SET(dqs_wlevel_right_edge[g].p, p);
	BFM_GBL_SET(dqs_wlevel_right_edge[g].d, d-1);
	BFM_GBL_SET(dqs_wlevel_right_edge[g].ps, work_end);

	/* center */
	work_mid = (work_bgn + work_end) / 2;

	DPRINT(2, "wlevel: work_mid=%d", work_mid);

	tmp_delay = 0;

	for (p = 0; p <= IO_DQDQS_OUT_PHASE_MAX  +
		num_additional_fr_cycles * IO_DLL_CHAIN_LENGTH &&
		tmp_delay < work_mid; p++, tmp_delay += IO_DELAY_PER_OPA_TAP)
		;

	if (tmp_delay > work_mid) {
		tmp_delay -= IO_DELAY_PER_OPA_TAP;
		p--;
	}

	while (p > IO_DQDQS_OUT_PHASE_MAX) {
		tmp_delay -= IO_DELAY_PER_OPA_TAP;
		p--;
	}

	scc_mgr_set_dqdqs_output_phase_all_ranks(g, p);

	DPRINT(2, "wlevel: p=%u tmp_delay=%u left=%u", p, tmp_delay,
	       work_mid - tmp_delay);

	for (d = 0; d <= IO_IO_OUT1_DELAY_MAX && tmp_delay < work_mid; d++,
		tmp_delay += IO_DELAY_PER_DCHAIN_TAP)
		;

	if (tmp_delay > work_mid) {
		tmp_delay -= IO_DELAY_PER_DCHAIN_TAP;
		d--;
	}

	DPRINT(2, "wlevel: p=%u d=%u tmp_delay=%u left=%u", p, d,
	       tmp_delay, work_mid - tmp_delay);

	scc_mgr_apply_group_all_out_delay_add_all_ranks(g, test_bgn, d);

	DPRINT(2, "wlevel: found middle: p=%u d=%u", p, d);
	BFM_GBL_SET(dqs_wlevel_mid[g].p, p);
	BFM_GBL_SET(dqs_wlevel_mid[g].d, d);
	BFM_GBL_SET(dqs_wlevel_mid[g].ps, work_mid);

	return 1;
}
#endif

/*
 * center all windows. do per-bit-deskew to possibly increase size of
 * certain windows.
 */
uint32_t rw_mgr_mem_calibrate_writes_center(uint32_t rank_bgn,
	uint32_t write_group, uint32_t test_bgn)
{
	uint32_t i, p, min_index;
	int32_t d;
	/*
	 * Store these as signed since there are comparisons with
	 * signed numbers.
	 */
	uint32_t bit_chk;
#if QDRII
	uint32_t tmp_bit_chk;
	uint32_t tmp_mask;
	uint32_t mask;
#endif
	uint32_t sticky_bit_chk;
	int32_t left_edge[RW_MGR_MEM_DQ_PER_WRITE_DQS];
	int32_t right_edge[RW_MGR_MEM_DQ_PER_WRITE_DQS];
	int32_t mid;
	int32_t mid_min, orig_mid_min;
	int32_t new_dqs, start_dqs, shift_dq;
	int32_t dq_margin, dqs_margin, dm_margin;
	uint32_t stop;

	TRACE_FUNC("%u %u", write_group, test_bgn);
	BFM_STAGE("writes_center");

	ALTERA_ASSERT(write_group < RW_MGR_MEM_IF_WRITE_DQS_WIDTH);

	dm_margin = 0;

	start_dqs = READ_SCC_DQS_IO_OUT1_DELAY();

	select_curr_shadow_reg_using_rank(rank_bgn);

	/* per-bit deskew */

	/*
	 * set the left and right edge of each bit to an illegal value
	 * use (IO_IO_OUT1_DELAY_MAX + 1) as an illegal value.
	 */
	sticky_bit_chk = 0;
	for (i = 0; i < RW_MGR_MEM_DQ_PER_WRITE_DQS; i++) {
		left_edge[i]  = IO_IO_OUT1_DELAY_MAX + 1;
		right_edge[i] = IO_IO_OUT1_DELAY_MAX + 1;
	}

	/* Search for the left edge of the window for each bit */
	for (d = 0; d <= IO_IO_OUT1_DELAY_MAX; d++) {
		scc_mgr_apply_group_dq_out1_delay(write_group, test_bgn, d);

		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);

		/*
		 * Stop searching when the read test doesn't pass AND when
		 * we've seen a passing read on every bit.
		 */
		stop = !rw_mgr_mem_calibrate_write_test(rank_bgn, write_group,
			0, PASS_ONE_BIT, &bit_chk, 0);
		sticky_bit_chk = sticky_bit_chk | bit_chk;
		stop = stop && (sticky_bit_chk == param->write_correct_mask);
		DPRINT(2, "write_center(left): dtap=%d => " BTFLD_FMT
			" == " BTFLD_FMT " && %u [bit_chk=" BTFLD_FMT "]",
			d, sticky_bit_chk, param->write_correct_mask,
			stop, bit_chk);

		if (stop == 1) {
			break;
		} else {
			for (i = 0; i < RW_MGR_MEM_DQ_PER_WRITE_DQS; i++) {
				if (bit_chk & 1) {
					/*
					 * Remember a passing test as the
					 * left_edge.
					 */
					left_edge[i] = d;
				} else {
					/*
					 * If a left edge has not been seen
					 * yet, then a future passing test will
					 * mark this edge as the right edge.
					 */
					if (left_edge[i] ==
						IO_IO_OUT1_DELAY_MAX + 1) {
						right_edge[i] = -(d + 1);
					}
				}
				DPRINT(2, "write_center[l,d=%d):", d);
				DPRINT(2, "bit_chk_test=%d left_edge[%u]: %d",
				       (int)(bit_chk & 1), i, left_edge[i]);
				DPRINT(2, "right_edge[%u]: %d", i,
				       right_edge[i]);
				bit_chk = bit_chk >> 1;
			}
		}
	}

	/* Reset DQ delay chains to 0 */
	scc_mgr_apply_group_dq_out1_delay(write_group, test_bgn, 0);
	sticky_bit_chk = 0;
	for (i = RW_MGR_MEM_DQ_PER_WRITE_DQS - 1;; i--) {
		DPRINT(2, "write_center: left_edge[%u]: %d right_edge[%u]: %d",
		       i, left_edge[i], i, right_edge[i]);

		/*
		 * Check for cases where we haven't found the left edge,
		 * which makes our assignment of the the right edge invalid.
		 * Reset it to the illegal value.
		 */
		if ((left_edge[i] == IO_IO_OUT1_DELAY_MAX + 1) &&
		    (right_edge[i] != IO_IO_OUT1_DELAY_MAX + 1)) {
			right_edge[i] = IO_IO_OUT1_DELAY_MAX + 1;
			DPRINT(2, "write_center: reset right_edge[%u]: %d",
			       i, right_edge[i]);
		}

		/*
		 * Reset sticky bit (except for bits where we have
		 * seen the left edge).
		 */
		sticky_bit_chk = sticky_bit_chk << 1;
		if ((left_edge[i] != IO_IO_OUT1_DELAY_MAX + 1))
			sticky_bit_chk = sticky_bit_chk | 1;

		if (i == 0)
			break;
	}

	/* Search for the right edge of the window for each bit */
	for (d = 0; d <= IO_IO_OUT1_DELAY_MAX - start_dqs; d++) {
		scc_mgr_apply_group_dqs_io_and_oct_out1(write_group,
							d + start_dqs);

		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
		if (QDRII)
			rw_mgr_mem_dll_lock_wait();

		/*
		 * Stop searching when the read test doesn't pass AND when
		 * we've seen a passing read on every bit.
		 */
		stop = !rw_mgr_mem_calibrate_write_test(rank_bgn, write_group,
			0, PASS_ONE_BIT, &bit_chk, 0);
		if (stop)
			recover_mem_device_after_ck_dqs_violation();

		sticky_bit_chk = sticky_bit_chk | bit_chk;
		stop = stop && (sticky_bit_chk == param->write_correct_mask);

		DPRINT(2, "write_center (right): dtap=%u => " BTFLD_FMT " == "
			BTFLD_FMT " && %u", d, sticky_bit_chk,
			param->write_correct_mask, stop);

		if (stop == 1) {
			if (d == 0) {
				for (i = 0; i < RW_MGR_MEM_DQ_PER_WRITE_DQS;
					i++) {
					/* d = 0 failed, but it passed when
					testing the left edge, so it must be
					marginal, set it to -1 */
					if (right_edge[i] ==
						IO_IO_OUT1_DELAY_MAX + 1 &&
						left_edge[i] !=
						IO_IO_OUT1_DELAY_MAX + 1) {
						right_edge[i] = -1;
					}
				}
			}
			break;
		} else {
			for (i = 0; i < RW_MGR_MEM_DQ_PER_WRITE_DQS; i++) {
				if (bit_chk & 1) {
					/*
					 * Remember a passing test as
					 * the right_edge.
					 */
					right_edge[i] = d;
				} else {
					if (d != 0) {
						/*
						 * If a right edge has not
						 * been seen yet, then a future
						 * passing test will mark this
						 * edge as the left edge.
						 */
						if (right_edge[i] ==
						    IO_IO_OUT1_DELAY_MAX + 1)
							left_edge[i] = -(d + 1);
					} else {
						/*
						 * d = 0 failed, but it passed
						 * when testing the left edge,
						 * so it must be marginal, set
						 * it to -1.
						 */
						if (right_edge[i] ==
						    IO_IO_OUT1_DELAY_MAX + 1 &&
						    left_edge[i] !=
						    IO_IO_OUT1_DELAY_MAX + 1)
							right_edge[i] = -1;
						/*
						 * If a right edge has not been
						 * seen yet, then a future
						 * passing test will mark this
						 * edge as the left edge.
						 */
						else if (right_edge[i] ==
							IO_IO_OUT1_DELAY_MAX +
							1)
							left_edge[i] = -(d + 1);
					}
				}
				DPRINT(2, "write_center[r,d=%d):", d);
				DPRINT(2, "bit_chk_test=%d left_edge[%u]: %d",
				       (int)(bit_chk & 1), i, left_edge[i]);
				DPRINT(2, "right_edge[%u]: %d", i,
				       right_edge[i]);
				bit_chk = bit_chk >> 1;
			}
		}
	}

	/* Check that all bits have a window */
	for (i = 0; i < RW_MGR_MEM_DQ_PER_WRITE_DQS; i++) {
		DPRINT(2, "write_center: left_edge[%u]: %d right_edge[%u]: %d",
		       i, left_edge[i], i, right_edge[i]);
		BFM_GBL_SET(dq_write_left_edge[write_group][i], left_edge[i]);
		BFM_GBL_SET(dq_write_right_edge[write_group][i], right_edge[i]);
		if ((left_edge[i] == IO_IO_OUT1_DELAY_MAX + 1) ||
		    (right_edge[i] == IO_IO_OUT1_DELAY_MAX + 1)) {
			set_failing_group_stage(test_bgn + i,
						CAL_STAGE_WRITES,
						CAL_SUBSTAGE_WRITES_CENTER);
			return 0;
		}
	}

	/* Find middle of window for each DQ bit */
	mid_min = left_edge[0] - right_edge[0];
	min_index = 0;
	for (i = 1; i < RW_MGR_MEM_DQ_PER_WRITE_DQS; i++) {
		mid = left_edge[i] - right_edge[i];
		if (mid < mid_min) {
			mid_min = mid;
			min_index = i;
		}
	}

	/*
	 * -mid_min/2 represents the amount that we need to move DQS.
	 * If mid_min is odd and positive we'll need to add one to
	 * make sure the rounding in further calculations is correct
	 * (always bias to the right), so just add 1 for all positive values.
	 */
	if (mid_min > 0)
		mid_min++;
	mid_min = mid_min / 2;
	DPRINT(1, "write_center: mid_min=%d", mid_min);

	/* Determine the amount we can change DQS (which is -mid_min) */
	orig_mid_min = mid_min;
	new_dqs = start_dqs;
	mid_min = 0;
	DPRINT(1, "write_center: start_dqs=%d new_dqs=%d mid_min=%d",
	       start_dqs, new_dqs, mid_min);

	/* Initialize data for export structures */
	dqs_margin = IO_IO_OUT1_DELAY_MAX + 1;
	dq_margin  = IO_IO_OUT1_DELAY_MAX + 1;

	/* add delay to bring centre of all DQ windows to the same "level" */
	for (i = 0, p = test_bgn; i < RW_MGR_MEM_DQ_PER_WRITE_DQS; i++, p++) {
		/* Use values before divide by 2 to reduce round off error */
		shift_dq = (left_edge[i] - right_edge[i] -
			(left_edge[min_index] - right_edge[min_index]))/2  +
		(orig_mid_min - mid_min);

		DPRINT(2, "write_center: before: shift_dq[%u]=%d", i,
		       shift_dq);

		if (shift_dq + (int32_t)READ_SCC_DQ_OUT1_DELAY(i) >
			(int32_t)IO_IO_OUT1_DELAY_MAX) {
			shift_dq = (int32_t)IO_IO_OUT1_DELAY_MAX -
			READ_SCC_DQ_OUT1_DELAY(i);
		} else if (shift_dq + (int32_t)READ_SCC_DQ_OUT1_DELAY(i) < 0) {
			shift_dq = -(int32_t)READ_SCC_DQ_OUT1_DELAY(i);
		}
		DPRINT(2, "write_center: after: shift_dq[%u]=%d",
		       i, shift_dq);
		scc_mgr_set_dq_out1_delay(write_group, i,
					  READ_SCC_DQ_OUT1_DELAY(i) +
					  shift_dq);
		scc_mgr_load_dq(i);

		DPRINT(2, "write_center: margin[%u]=[%d,%d]", i,
		       left_edge[i] - shift_dq + (-mid_min),
		       right_edge[i] + shift_dq - (-mid_min));
		/* To determine values for export structures */
		if (left_edge[i] - shift_dq + (-mid_min) < dq_margin)
			dq_margin = left_edge[i] - shift_dq + (-mid_min);

		if (right_edge[i] + shift_dq - (-mid_min) < dqs_margin)
			dqs_margin = right_edge[i] + shift_dq - (-mid_min);
	}

	/* Move DQS */
	if (QDRII) {
		scc_mgr_set_group_dqs_io_and_oct_out1_gradual(write_group,
							      new_dqs);
	} else {
		scc_mgr_apply_group_dqs_io_and_oct_out1(write_group, new_dqs);
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	}

	/* Centre DM */
	BFM_STAGE("dm_center");
	DPRINT(2, "write_center: DM");
#if RLDRAMX

	/*
	 * this is essentially the same as DDR with the exception of
	 * the dm_ global accounting.
	 * Determine if first group in device to initialize left and
	 * right edges.
	 */
	if (!is_write_group_enabled_for_dm(write_group)) {
		DPRINT(2, "dm_calib: skipping since not last in group");
	} else {
		/* last in the group, so we need to do DM */
		DPRINT(2, "dm_calib: calibrating DM since last in group");

		/*
		 * set the left and right edge of each bit to illegal value,
		 * use (IO_IO_OUT1_DELAY_MAX + 1) as an illegal value.
		 */
		left_edge[0]  = IO_IO_OUT1_DELAY_MAX + 1;
		right_edge[0] = IO_IO_OUT1_DELAY_MAX + 1;

		sticky_bit_chk = 0;
		/* Search for the left edge of the window for the DM bit */
		for (d = 0; d <= IO_IO_OUT1_DELAY_MAX; d++) {
			scc_mgr_apply_group_dm_out1_delay(write_group, d);
			IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);

			/*
			 * Stop searching when the write test doesn't pass AND
			 * when we've seen a passing write before.
			 */
			if (rw_mgr_mem_calibrate_write_test(rank_bgn,
							    write_group, 1,
							    PASS_ALL_BITS,
							    &bit_chk, 0)) {
				DPRINT(2, "dm_calib: left=%lu passed", d);
				left_edge[0] = d;
			} else {
				DPRINT(2, "dm_calib: left=%lu failed", d);
				/*
				 * If a left edge has not been seen yet, then
				 * a future passing test will mark this edge as
				 * the right edge.
				 */
				if (left_edge[0] == IO_IO_OUT1_DELAY_MAX + 1) {
					right_edge[0] = -(d + 1);
				} else {
					/*
					 * left edge has been seen, so this
					 * failure marks the left edge, and
					 * we are done.
					 */
					break;
				}
			}
			DPRINT(2, "dm_calib[l,d=%lu]: left_edge: %ld", d,
			       left_edge[0]);
			DPRINT(2, "right_edge: %ld", right_edge[0]);
		}

		DPRINT(2, "dm_calib left done: left_edge: %ld right_edge: %ld",
		       left_edge[0], right_edge[0]);

		/* Reset DM delay chains to 0 */
		scc_mgr_apply_group_dm_out1_delay(write_group, 0);

		/*
		 * Check for cases where we haven't found the left edge,
		 * which makes our assignment of the the right edge invalid.
		 * Reset it to the illegal value.
		 */
		if ((left_edge[0] == IO_IO_OUT1_DELAY_MAX + 1) &&
		    (right_edge[0] != IO_IO_OUT1_DELAY_MAX + 1)) {
			right_edge[0] = IO_IO_OUT1_DELAY_MAX + 1;
			DPRINT(2, "dm_calib: reset right_edge: %ld",
			       right_edge[0]);
		}

		/* Search for the right edge of the window for the DM bit */
		for (d = 0; d <= IO_IO_OUT1_DELAY_MAX - new_dqs; d++) {
			/*
			 * Note: This only shifts DQS, so are we limiting
			 * ourselve to width of DQ unnecessarily.
			 */
			scc_mgr_apply_group_dqs_io_and_oct_out1(write_group,
								d + new_dqs);

			IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);

			/*
			 * Stop searching when the test fails and we've seen
			 * passing test already.
			 */
			if (rw_mgr_mem_calibrate_write_test(rank_bgn,
							    write_group, 1,
							    PASS_ALL_BITS,
							    &bit_chk, 0)) {
				DPRINT(2, "dm_calib: right=%lu passed", d);
				right_edge[0] = d;
			} else {
				recover_mem_device_after_ck_dqs_violation();

				DPRINT(2, "dm_calib: right=%lu failed", d);
				if (d != 0) {
					/*
					 * If a right edge has not been seen
					 * yet, then a future passing test will
					 * mark this edge as the left edge.
					 */
					if (right_edge[0] ==
					IO_IO_OUT1_DELAY_MAX + 1) {
						left_edge[0] = -(d + 1);
					} else {
						break;
					}
				} else {
					/*
					 * d = 0 failed, but it passed when
					 * testing the left edge, so it must be
					 * marginal, set it to -1.
					 */
					if (right_edge[0] ==
					IO_IO_OUT1_DELAY_MAX + 1 && left_edge[0]
					!= IO_IO_OUT1_DELAY_MAX + 1) {
						right_edge[0] = -1;
						/* we're done */
						break;
					}
					/*
					 * If a right edge has not been seen
					 * yet, then a future passing test will
					 * mark this edge as the left edge.
					 */
					else if (right_edge[0] ==
					IO_IO_OUT1_DELAY_MAX + 1) {
						left_edge[0] = -(d + 1);
					}
				}
			}
			DPRINT(2, "dm_calib[l,d=%lu]: left_edge: %ld", d,
			       left_edge[0]);
			DPRINT(2, "right_edge: %ld", right_edge[0]);
		}

		DPRINT(2, "dm_calib: left=%ld right=%ld", left_edge[0],
		       right_edge[0]);

		/* Move DQS (back to orig) */
		scc_mgr_apply_group_dqs_io_and_oct_out1(write_group, new_dqs);

		/*
		 * move DM
		 * Find middle of window for the DM bit.
		 */
		mid = (left_edge[0] - right_edge[0]) / 2;
		if (mid < 0)
			mid = 0;
		scc_mgr_apply_group_dm_out1_delay(write_group, mid);

		dm_margin = left_edge[0];
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
		DPRINT(2, "dm_calib: left=%ld right=%ld mid=%ld dm_margin=%ld",
		       left_edge[0], right_edge[0], mid, dm_margin);
	} /* end of DM calibration */
#endif


#if DDRX
	/*
	 * set the left and right edge of each bit to an illegal value,
	 * use (IO_IO_OUT1_DELAY_MAX + 1) as an illegal value,
	 */
	left_edge[0]  = IO_IO_OUT1_DELAY_MAX + 1;
	right_edge[0] = IO_IO_OUT1_DELAY_MAX + 1;
	int32_t bgn_curr = IO_IO_OUT1_DELAY_MAX + 1;
	int32_t end_curr = IO_IO_OUT1_DELAY_MAX + 1;
	int32_t bgn_best = IO_IO_OUT1_DELAY_MAX + 1;
	int32_t end_best = IO_IO_OUT1_DELAY_MAX + 1;
	int32_t win_best = 0;

	/* Search for the/part of the window with DM shift */
	for (d = IO_IO_OUT1_DELAY_MAX; d >= 0; d -= DELTA_D) {
		scc_mgr_apply_group_dm_out1_delay(write_group, d);
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);

		if (rw_mgr_mem_calibrate_write_test(rank_bgn, write_group, 1,
						    PASS_ALL_BITS, &bit_chk,
						    0)) {
			/* USE Set current end of the window */
			end_curr = -d;
			/*
			 * If a starting edge of our window has not been seen
			 * this is our current start of the DM window.
			 */
			if (bgn_curr == IO_IO_OUT1_DELAY_MAX + 1)
				bgn_curr = -d;

			/*
			 * If current window is bigger than best seen.
			 * Set best seen to be current window.
			 */
			if ((end_curr-bgn_curr+1) > win_best) {
				win_best = end_curr-bgn_curr+1;
				bgn_best = bgn_curr;
				end_best = end_curr;
			}
		} else {
			/* We just saw a failing test. Reset temp edge */
			bgn_curr = IO_IO_OUT1_DELAY_MAX + 1;
			end_curr = IO_IO_OUT1_DELAY_MAX + 1;
			}
		}


	/* Reset DM delay chains to 0 */
	scc_mgr_apply_group_dm_out1_delay(write_group, 0);

	/*
	 * Check to see if the current window nudges up aganist 0 delay.
	 * If so we need to continue the search by shifting DQS otherwise DQS
	 * search begins as a new search. */
	if (end_curr != 0) {
		bgn_curr = IO_IO_OUT1_DELAY_MAX + 1;
		end_curr = IO_IO_OUT1_DELAY_MAX + 1;
	}

	/* Search for the/part of the window with DQS shifts */
	for (d = 0; d <= IO_IO_OUT1_DELAY_MAX - new_dqs; d += DELTA_D) {
		/*
		 * Note: This only shifts DQS, so are we limiting ourselve to
		 * width of DQ unnecessarily.
		 */
		scc_mgr_apply_group_dqs_io_and_oct_out1(write_group,
							d + new_dqs);

		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
		if (rw_mgr_mem_calibrate_write_test(rank_bgn, write_group, 1,
						    PASS_ALL_BITS, &bit_chk,
						    0)) {
			/* USE Set current end of the window */
			end_curr = d;
			/*
			 * If a beginning edge of our window has not been seen
			 * this is our current begin of the DM window.
			 */
			if (bgn_curr == IO_IO_OUT1_DELAY_MAX + 1)
				bgn_curr = d;

			/*
			 * If current window is bigger than best seen. Set best
			 * seen to be current window.
			 */
			if ((end_curr-bgn_curr+1) > win_best) {
				win_best = end_curr-bgn_curr+1;
				bgn_best = bgn_curr;
				end_best = end_curr;
			}
		} else {
			/* We just saw a failing test. Reset temp edge */
			recover_mem_device_after_ck_dqs_violation();
			bgn_curr = IO_IO_OUT1_DELAY_MAX + 1;
			end_curr = IO_IO_OUT1_DELAY_MAX + 1;

			/* Early exit optimization: if ther remaining delay
			chain space is less than already seen largest window
			we can exit */
			if ((win_best-1) >
				(IO_IO_OUT1_DELAY_MAX - new_dqs - d)) {
					break;
				}
			}
		}

	/* assign left and right edge for cal and reporting; */
	left_edge[0] = -1*bgn_best;
	right_edge[0] = end_best;

	DPRINT(2, "dm_calib: left=%d right=%d", left_edge[0], right_edge[0]);
	BFM_GBL_SET(dm_left_edge[write_group][0], left_edge[0]);
	BFM_GBL_SET(dm_right_edge[write_group][0], right_edge[0]);

	/* Move DQS (back to orig) */
	scc_mgr_apply_group_dqs_io_and_oct_out1(write_group, new_dqs);

	/* Move DM */

	/* Find middle of window for the DM bit */
	mid = (left_edge[0] - right_edge[0]) / 2;

	/* only move right, since we are not moving DQS/DQ */
	if (mid < 0)
		mid = 0;

	/* dm_marign should fail if we never find a window */
	if (win_best == 0)
		dm_margin = -1;
	else
		dm_margin = left_edge[0] - mid;

	scc_mgr_apply_group_dm_out1_delay(write_group, mid);
	IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);

	DPRINT(2, "dm_calib: left=%d right=%d mid=%d dm_margin=%d",
	       left_edge[0], right_edge[0], mid, dm_margin);
#endif

#if QDRII
	sticky_bit_chk = 0;

	/*
	 * set the left and right edge of each bit to an illegal value
	 * use (IO_IO_OUT1_DELAY_MAX + 1) as an illegal value.
	 */
	for (i = 0; i < RW_MGR_MEM_DATA_MASK_WIDTH /
		RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++) {
		left_edge[i] = IO_IO_OUT1_DELAY_MAX + 1;
		right_edge[i] = IO_IO_OUT1_DELAY_MAX + 1;
	}

	mask = param->dm_correct_mask;
	/* Search for the left edge of the window for the DM bit */
	for (d = 0; d <= IO_IO_OUT1_DELAY_MAX; d++) {
		scc_mgr_apply_group_dm_out1_delay(write_group, d);
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);

		/*
		 * Stop searching when the read test doesn't pass for all bits
		 * (as they've already been calibrated).
		 */
		stop = !rw_mgr_mem_calibrate_write_test(rank_bgn, write_group,
			1, PASS_ONE_BIT, &bit_chk, 0);
		DPRINT(2, "dm_calib[l,d=%lu] stop=%ld bit_chk=%llx ",
		       d, stop, bit_chk);
		DPRINT(2, "sticky_bit_chk=%llx mask=%llx", sticky_bit_chk,
		       param->write_correct_mask);
		tmp_bit_chk = bit_chk;
		tmp_mask = mask;
		for (i = 0; i < RW_MGR_MEM_DATA_MASK_WIDTH /
			RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++) {
			if ((tmp_bit_chk & mask) == mask)
				sticky_bit_chk = sticky_bit_chk | tmp_mask;
			tmp_bit_chk = tmp_bit_chk >> (RW_MGR_MEM_DATA_WIDTH /
				RW_MGR_MEM_DATA_MASK_WIDTH);
			tmp_mask = tmp_mask << (RW_MGR_MEM_DATA_WIDTH /
				RW_MGR_MEM_DATA_MASK_WIDTH);
		}
		stop = stop && (sticky_bit_chk == param->write_correct_mask);

		if (stop == 1) {
			break;
		} else {
			for (i = 0; i < RW_MGR_MEM_DATA_MASK_WIDTH /
				RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++) {
				DPRINT(2, "dm_calib[l,i=%lu] d=%lu ",
				       BTFLD_FMT, i, d);
				DPRINT(2, "bit_chk&dm_mask=" BTFLD_FMT " == "
					bit_chk & mask, mask);
				if ((bit_chk & mask) == mask) {
					DPRINT(2, "dm_calib: left[%lu]=%lu",
					       i, d);
					left_edge[i] = d;
				} else {
					/* If a left edge has not been seen yet,
					then a future passing test will mark
					this edge as the right edge */
					if (left_edge[i] ==
						IO_IO_OUT1_DELAY_MAX + 1) {
						right_edge[i] = -(d + 1);
					}
				}
				bit_chk = bit_chk >> (RW_MGR_MEM_DATA_WIDTH /
					RW_MGR_MEM_DATA_MASK_WIDTH);
			}
		}
	}

	/* Reset DM delay chains to 0 */
	scc_mgr_apply_group_dm_out1_delay(write_group, 0);

	/*
	 * Check for cases where we haven't found the left edge, which makes
	 * our assignment of the the right edge invalid.  Reset it to the
	 * illegal value.
	 */
	for (i = 0; i < RW_MGR_MEM_DATA_MASK_WIDTH /
		RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++) {
		if ((left_edge[i] == IO_IO_OUT1_DELAY_MAX + 1) &&
		    (right_edge[i] != IO_IO_OUT1_DELAY_MAX + 1)) {
			right_edge[i] = IO_IO_OUT1_DELAY_MAX + 1;
			DPRINT(2, "dm_calib: reset right_edge: %d",
			       right_edge[i]);
		}
	}

	/* Search for the right edge of the window for the DM bit */
	for (d = 0; d <= IO_IO_OUT1_DELAY_MAX - new_dqs; d++) {
		scc_mgr_apply_group_dqs_io_and_oct_out1(write_group,
							d + new_dqs);

		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
		rw_mgr_mem_dll_lock_wait();

		/*
		 * Stop searching when the read test doesn't pass for all bits
		 * (as they've already been calibrated).
		 */
		stop = !rw_mgr_mem_calibrate_write_test(rank_bgn, write_group,
			1, PASS_ONE_BIT, &bit_chk, 0);
		DPRINT(2, "dm_calib[l,d=%lu] stop=%ld bit_chk=%llx ",
		       d, stop, bit_chk);
		DPRINT(2, "sticky_bit_chk=%llx mask=%llx", sticky_bit_chk,
		       param->write_correct_mask);
		tmp_bit_chk = bit_chk;
		tmp_mask = mask;
		for (i = 0; i < RW_MGR_MEM_DATA_MASK_WIDTH /
			RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++) {
			if ((tmp_bit_chk & mask) == mask)
				sticky_bit_chk = sticky_bit_chk | tmp_mask;

			tmp_bit_chk = tmp_bit_chk >> (RW_MGR_MEM_DATA_WIDTH /
				RW_MGR_MEM_DATA_MASK_WIDTH);
			tmp_mask = tmp_mask << (RW_MGR_MEM_DATA_WIDTH /
				RW_MGR_MEM_DATA_MASK_WIDTH);
		}
		stop = stop && (sticky_bit_chk == param->write_correct_mask);

		if (stop == 1) {
			break;
		} else {
			for (i = 0; i < RW_MGR_MEM_DATA_MASK_WIDTH /
				RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++) {
				DPRINT(2, "dm_calib[r,i=%lu] d=%lu",
				       BTFLD_FMT, i, d);
				DPRINT(2, "bit_chk&dm_mask=" BTFLD_FMT " == ",
					bit_chk & mask, mask);
				if ((bit_chk & mask) == mask) {
					right_edge[i] = d;
				} else {
					/* d = 0 failed, but it passed when
					testing the left edge, so it must be
					marginal, set it to -1 */
					if (right_edge[i] ==
						IO_IO_OUT1_DELAY_MAX + 1 &&
						left_edge[i] !=
						IO_IO_OUT1_DELAY_MAX + 1) {
						right_edge[i] = -1;
						/* we're done */
						break;
					}
					/* If a right edge has not been seen
					yet, then a future passing test will
					mark this edge as the left edge */
					else if (right_edge[i] ==
					IO_IO_OUT1_DELAY_MAX + 1) {
						left_edge[i] = -(d + 1);
					}
				}
				bit_chk = bit_chk >> (RW_MGR_MEM_DATA_WIDTH /
					RW_MGR_MEM_DATA_MASK_WIDTH);
			}
		}
	}

	/* Move DQS (back to orig) */
	scc_mgr_set_group_dqs_io_and_oct_out1_gradual(write_group, new_dqs);

	/* Move DM */
	dm_margin = IO_IO_OUT1_DELAY_MAX;
	for (i = 0; i < RW_MGR_MEM_DATA_MASK_WIDTH /
		RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++) {
		/* Find middle of window for the DM bit */
		mid = (left_edge[i] - right_edge[i]) / 2;
		DPRINT(2, "dm_calib[mid,i=%lu] left=%ld right=%ld mid=%ld", i,
		       left_edge[i], right_edge[i], mid);
		BFM_GBL_SET(dm_left_edge[write_group][i], left_edge[i]);
		BFM_GBL_SET(dm_right_edge[write_group][i], right_edge[i]);

		if (mid < 0)
			mid = 0;
		scc_mgr_set_dm_out1_delay(write_group, i, mid);
		scc_mgr_load_dm(i);
		if ((left_edge[i] - mid) < dm_margin)
			dm_margin = left_edge[i] - mid;
	}
#endif
	/* Export values */
	gbl->fom_out += dq_margin + dqs_margin;

	DPRINT(2, "write_center: dq_margin=%d dqs_margin=%d dm_margin=%d",
	       dq_margin, dqs_margin, dm_margin);

	/*
	 * Do not remove this line as it makes sure all of our
	 * decisions have been applied.
	 */
	IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	return (dq_margin >= 0) && (dqs_margin >= 0) && (dm_margin >= 0);
}

/* calibrate the write operations */
uint32_t rw_mgr_mem_calibrate_writes(uint32_t rank_bgn, uint32_t g,
	uint32_t test_bgn)
{
	/* update info for sims */
	TRACE_FUNC("%u %u", g, test_bgn);

	reg_file_set_stage(CAL_STAGE_WRITES);
	reg_file_set_sub_stage(CAL_SUBSTAGE_WRITES_CENTER);

	reg_file_set_group(g);

	if (!rw_mgr_mem_calibrate_writes_center(rank_bgn, g, test_bgn)) {
		set_failing_group_stage(g, CAL_STAGE_WRITES,
					CAL_SUBSTAGE_WRITES_CENTER);
		return 0;
	}

	return 1;
}

/* precharge all banks and activate row 0 in bank "000..." and bank "111..." */
#if DDRX
void mem_precharge_and_activate(void)
{
	uint32_t r;

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r]) {
			/* request to skip the rank */

			continue;
		}

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_OFF);

		/* precharge all banks ... */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_PRECHARGE_ALL);

		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x0F);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0,
			      __RW_MGR_ACTIVATE_0_AND_1_WAIT1);

		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, 0x0F);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
			      __RW_MGR_ACTIVATE_0_AND_1_WAIT2);

		/* activate rows */
		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_ACTIVATE_0_AND_1);
	}
}
#endif

#if QDRII || RLDRAMX
void mem_precharge_and_activate(void) {}
#endif

/* Configure various memory related parameters. */
#if DDRX
void mem_config(void)
{
	uint32_t rlat, wlat;
	uint32_t rw_wl_nop_cycles;
	uint32_t max_latency;
#if CALIBRATE_BIT_SLIPS
	uint32_t i;
#endif

	TRACE_FUNC();

	/* read in write and read latency */
	wlat = IORD_32DIRECT(MEM_T_WL_ADD, 0);
#if HARD_PHY
	wlat += IORD_32DIRECT(DATA_MGR_MEM_T_ADD, 0);
	/* WL for hard phy does not include additive latency */

#if DDR3 || DDR2
	/*
	 * add addtional write latency to offset the address/command extra
	 * clock cycle. We change the AC mux setting causing AC to be delayed
	 * by one mem clock cycle. Only do this for DDR3
	 */
	wlat = wlat + 1;
#endif

#endif

	rlat = IORD_32DIRECT(MEM_T_RL_ADD, 0);

	if (QUARTER_RATE_MODE) {
		/*
		 * In Quarter-Rate the WL-to-nop-cycles works like this:
		 * 0,1     -> 0
		 * 2,3,4,5 -> 1
		 * 6,7,8,9 -> 2
		 * etc...
		 */
		rw_wl_nop_cycles = (wlat + 6) / 4 - 1;
	} else if (HALF_RATE_MODE)	{
		/*
		 * In Half-Rate the WL-to-nop-cycles works like this:
		 * 0,1 -> -1
		 * 2,3 -> 0
		 * 4,5 -> 1
		 * etc...
		 */
		if (wlat % 2)
			rw_wl_nop_cycles = ((wlat - 1) / 2) - 1;
		else
			rw_wl_nop_cycles = (wlat / 2) - 1;
	} else {
		rw_wl_nop_cycles = wlat - 2;
#if LPDDR2
		rw_wl_nop_cycles = rw_wl_nop_cycles + 1;
#endif
	}
#if MULTIPLE_AFI_WLAT
	for (i = 0; i < RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++)
		gbl->rw_wl_nop_cycles_per_group[i] = rw_wl_nop_cycles;
#endif
	gbl->rw_wl_nop_cycles = rw_wl_nop_cycles;

#if ARRIAV || CYCLONEV
	/*
	 * For AV/CV, lfifo is hardened and always runs at full rate so
	 * max latency in AFI clocks, used here, is correspondingly smaller.
	 */
	if (QUARTER_RATE_MODE)
		max_latency = (1<<MAX_LATENCY_COUNT_WIDTH)/4 - 1;
	else if (HALF_RATE_MODE)
		max_latency = (1<<MAX_LATENCY_COUNT_WIDTH)/2 - 1;
	else
		max_latency = (1<<MAX_LATENCY_COUNT_WIDTH)/1 - 1;
#else
	max_latency = (1<<MAX_LATENCY_COUNT_WIDTH) - 1;
#endif
	/* configure for a burst length of 8 */

	if (QUARTER_RATE_MODE) {
		/* write latency */
		wlat = (wlat + 5) / 4 + 1;

		/* set a pretty high read latency initially */
		gbl->curr_read_lat = (rlat + 1) / 4 + 8;
	} else if (HALF_RATE_MODE) {
		/* write latency */
		wlat = (wlat - 1) / 2 + 1;

		/* set a pretty high read latency initially */
		gbl->curr_read_lat = (rlat + 1) / 2 + 8;
	} else {
		/* write latency */
#if HARD_PHY
		/* Adjust Write Latency for Hard PHY */
		wlat = wlat + 1;
#if LPDDR2
		/*
		 * Add another one in hard for LPDDR2 since this value is raw
		 * from controller assume tdqss is one.
		 */
		wlat = wlat + 1;
#endif
#endif
		/* set a pretty high read latency initially */
		gbl->curr_read_lat = rlat + 16;
	}

	if (gbl->curr_read_lat > max_latency)
		gbl->curr_read_lat = max_latency;

	IOWR_32DIRECT(PHY_MGR_PHY_RLAT, 0, gbl->curr_read_lat);

	/* advertise write latency */
	gbl->curr_write_lat = wlat;
#if MULTIPLE_AFI_WLAT
	for (i = 0; i < RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++) {
#if HARD_PHY
		IOWR_32DIRECT(PHY_MGR_AFI_WLAT, i*4, wlat - 2);
#else
		IOWR_32DIRECT(PHY_MGR_AFI_WLAT, i*4, wlat - 1);
#endif
	}
#else
#if HARD_PHY
	IOWR_32DIRECT(PHY_MGR_AFI_WLAT, 0, wlat - 2);
#else
	IOWR_32DIRECT(PHY_MGR_AFI_WLAT, 0, wlat - 1);
#endif
#endif
	/* initialize bit slips */
#if CALIBRATE_BIT_SLIPS
	for (i = 0; i < RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++)
		IOWR_32DIRECT(PHY_MGR_FR_SHIFT, i*4, 0);
#endif
	mem_precharge_and_activate();
}
#endif

#if QDRII || RLDRAMX
void mem_config(void)
{
	uint32_t wlat, nop_cycles, max_latency;

	TRACE_FUNC();

	max_latency = (1<<MAX_LATENCY_COUNT_WIDTH) - 1;

	if (QUARTER_RATE_MODE)
		/* TODO_JCHOI: verify confirm */
		gbl->curr_read_lat =
			(IORD_32DIRECT(MEM_T_RL_ADD, 0) + 1) / 4 + 8;
	else if (HALF_RATE_MODE)
		gbl->curr_read_lat =
			(IORD_32DIRECT(MEM_T_RL_ADD, 0) + 1) / 2 + 8;
	else
		gbl->curr_read_lat =
			IORD_32DIRECT(MEM_T_RL_ADD, 0) + 16;

	if (gbl->curr_read_lat > max_latency)
		gbl->curr_read_lat = max_latency;

	IOWR_32DIRECT(PHY_MGR_PHY_RLAT, 0, gbl->curr_read_lat);

	if (RLDRAMX) {
		/* read in write and read latency */
		wlat = IORD_32DIRECT(MEM_T_WL_ADD, 0);

		if (QUARTER_RATE_MODE) {
			/* TODO_JCHOI Verify */
			nop_cycles = ((wlat - 1) / 4) - 1;
		} else if (HALF_RATE_MODE) {
#if HR_DDIO_OUT_HAS_THREE_REGS
			nop_cycles = (wlat / 2) - 2;
#else
	#if RLDRAM3
			/* RLDRAM3 uses all AFI phases to issue commands */
			nop_cycles = (wlat / 2) - 2;
	#else
			nop_cycles = ((wlat + 1) / 2) - 2;
	#endif
#endif
		} else	{
			nop_cycles = wlat - 1;
		}
		gbl->rw_wl_nop_cycles = nop_cycles;
	}
}
#endif

/* Set VFIFO and LFIFO to instant-on settings in skip calibration mode */
void mem_skip_calibrate(void)
{
	uint32_t vfifo_offset;
	uint32_t i, j, r;
#if HCX_COMPAT_MODE && DDR3
	uint32_t v;
#if (RDIMM || LRDIMM)
	uint32_t increment = 2;
#else
	uint32_t wlat = IORD_32DIRECT(PHY_MGR_MEM_T_WL, 0);
	uint32_t rlat = IORD_32DIRECT(PHY_MGR_MEM_T_RL, 0);
	uint32_t increment = rlat - wlat*2 + 1;
#endif
#endif

	TRACE_FUNC();

	/* Need to update every shadow register set used by the interface */
	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS;
		r += NUM_RANKS_PER_SHADOW_REG) {
		/*
		 * Strictly speaking this should be called once per group to
		 * make sure each group's delay chains are refreshed from the
		 * SCC register file, but since we're resetting all delay chains
		 * anyway, we can save some runtime by calling
		 * select_shadow_regs_for_update just once to switch rank.
		 */
		 select_shadow_regs_for_update(r, 0, 1);

		/*
		 * Set output phase alignment settings appropriate for
		 * skip calibration.
		 */
		for (i = 0; i < RW_MGR_MEM_IF_READ_DQS_WIDTH; i++) {
#if STRATIXV || ARRIAV || CYCLONEV || ARRIAVGZ
			scc_mgr_set_dqs_en_phase(i, 0);
#else
#if IO_DLL_CHAIN_LENGTH == 6
			scc_mgr_set_dqs_en_phase(i, (IO_DLL_CHAIN_LENGTH >> 1)
						 - 1);
#else
			scc_mgr_set_dqs_en_phase(i, (IO_DLL_CHAIN_LENGTH >> 1));
#endif
#endif
#if HCX_COMPAT_MODE && DDR3
			v = 0;
			for (j = 0; j < increment; j++)
				rw_mgr_incr_vfifo(i, &v);
#if IO_DLL_CHAIN_LENGTH == 6
			scc_mgr_set_dqdqs_output_phase(i, 6);
#else
			scc_mgr_set_dqdqs_output_phase(i, 7);
#endif
#else
	#if HCX_COMPAT_MODE
			/*
			 * in this mode, write_clk doesn't always lead mem_ck
			 * by 90 deg, and so the enhancement in case:33398 can't
			 * be applied.
			 */
			scc_mgr_set_dqdqs_output_phase(i, (IO_DLL_CHAIN_LENGTH
				- IO_DLL_CHAIN_LENGTH / 3));
	#else
			/*
			 * Case:33398
			 *
			 * Write data arrives to the I/O two cycles before write
			 * latency is reached (720 deg).
			 *   -> due to bit-slip in a/c bus
			 *   -> to allow board skew where dqs is longer than ck
			 *      -> how often can this happen!?
			 *      -> can claim back some ptaps for high freq
			 *       support if we can relax this, but i digress...
			 *
			 * The write_clk leads mem_ck by 90 deg
			 * The minimum ptap of the OPA is 180 deg
			 * Each ptap has (360 / IO_DLL_CHAIN_LENGH) deg of delay
			 * The write_clk is always delayed by 2 ptaps
			 *
			 * Hence, to make DQS aligned to CK, we need to delay
			 * DQS by:
			 *    (720 - 90 - 180 - 2 * (360 / IO_DLL_CHAIN_LENGTH))
			 *
			 * Dividing the above by (360 / IO_DLL_CHAIN_LENGTH)
			 * gives us the number of ptaps, which simplies to:
			 *
			 *    (1.25 * IO_DLL_CHAIN_LENGTH - 2)
			 */
			scc_mgr_set_dqdqs_output_phase(i, (1.25 *
				IO_DLL_CHAIN_LENGTH - 2));
	#endif
#endif
		}
		IOWR_32DIRECT(SCC_MGR_DQS_ENA, 0, 0xff);
		IOWR_32DIRECT(SCC_MGR_DQS_IO_ENA, 0, 0xff);

		for (i = 0; i < RW_MGR_MEM_IF_WRITE_DQS_WIDTH; i++) {
			IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0, i);
			IOWR_32DIRECT(SCC_MGR_DQ_ENA, 0, 0xff);
			IOWR_32DIRECT(SCC_MGR_DM_ENA, 0, 0xff);
		}
#if USE_SHADOW_REGS
		/*
		 * in shadow-register mode, SCC_UPDATE is done on a per-group
		 * basis unless we explicitly ask for a multicast via the
		 * group counter.
		 */
		IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0, 0xFF);
#endif
		IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	}

#if ARRIAV || CYCLONEV
	/* Compensate for simulation model behaviour */
	for (i = 0; i < RW_MGR_MEM_IF_READ_DQS_WIDTH; i++) {
		scc_mgr_set_dqs_bus_in_delay(i, 10);
		scc_mgr_load_dqs(i);
	}
	IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
#endif

#if ARRIAV || CYCLONEV
	/*
	 * ArriaV has hard FIFOs that can only be initialized by incrementing
	 * in sequencer.
	 */
	vfifo_offset = CALIB_VFIFO_OFFSET;
	for (j = 0; j < vfifo_offset; j++) {
		if (HARD_PHY)
			IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_HARD_PHY, 0, 0xff);
		else
			IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_FR, 0, 0xff);
	}
#else
/*
 * Note, this is not currently supported; changing this might significantly
 * increase the size of the ROM.
 */
#if SUPPORT_DYNAMIC_SKIP_CALIBRATE_ACTIONS
	if ((DYNAMIC_CALIB_STEPS) & CALIB_IN_RTL_SIM) {
		/* VFIFO is reset to the correct settings in RTL simulation */
	} else {
		vfifo_offset = IORD_32DIRECT(PHY_MGR_CALIB_VFIFO_OFFSET, 0);

		if (QUARTER_RATE_MODE) {
			while (vfifo_offset > 3) {
				IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_QR, 0,
					      0xff);
				vfifo_offset -= 4;
			}

			if (vfifo_offset == 3) {
				IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_FR_HR,
					      0, 0xff);
			} else if (vfifo_offset == 2) {
				IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_HR, 0,
					      0xff);
			} else if (vfifo_offset == 1) {
				IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_FR, 0,
					      0xff);
			}
		} else {
			while (vfifo_offset > 1) {
				IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_HR, 0,
					      0xff);
				vfifo_offset -= 2;
			}

			if (vfifo_offset == 1) {
				IOWR_32DIRECT(PHY_MGR_CMD_INC_VFIFO_FR, 0,
					      0xff);
			}
		}
	}
#endif
#endif
	IOWR_32DIRECT(PHY_MGR_CMD_FIFO_RESET, 0, 0);

#if ARRIAV || CYCLONEV
	/*
	 * For ACV with hard lfifo, we get the skip-cal setting from
	 * generation-time constant.
	 */
	gbl->curr_read_lat = CALIB_LFIFO_OFFSET;
#else
	gbl->curr_read_lat = IORD_32DIRECT(PHY_MGR_CALIB_LFIFO_OFFSET, 0);
#endif
	IOWR_32DIRECT(PHY_MGR_PHY_RLAT, 0, gbl->curr_read_lat);
}

/* Memory calibration entry point */
uint32_t mem_calibrate(void)
{
	uint32_t i;
	uint32_t rank_bgn, sr;
	uint32_t write_group, write_test_bgn;
	uint32_t read_group, read_test_bgn;
	uint32_t run_groups, current_run;
	uint32_t failing_groups = 0;
	uint32_t group_failed = 0;
	uint32_t sr_failed = 0;

	TRACE_FUNC();

	/* Initialize the data settings */
	DPRINT(1, "Preparing to init data");
	DPRINT(1, "Init complete");

	gbl->error_substage = CAL_SUBSTAGE_NIL;
	gbl->error_stage = CAL_STAGE_NIL;
	gbl->error_group = 0xff;
	gbl->fom_in = 0;
	gbl->fom_out = 0;

	mem_config();

	if (ARRIAV || CYCLONEV) {
		uint32_t bypass_mode = (HARD_PHY) ? 0x1 : 0x0;
		for (i = 0; i < RW_MGR_MEM_IF_READ_DQS_WIDTH; i++) {
			IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0, i);
			scc_set_bypass_mode(i, bypass_mode);
		}
	}

	if (((DYNAMIC_CALIB_STEPS) & CALIB_SKIP_ALL) == CALIB_SKIP_ALL) {
		/*
		 * Set VFIFO and LFIFO to instant-on settings in skip
		 * calibration mode.
		 */
		mem_skip_calibrate();
	} else {
		for (i = 0; i < NUM_CALIB_REPEAT; i++) {
			/*
			 * Zero all delay chain/phase settings for all
			 * groups and all shadow register sets.
			 */
			scc_mgr_zero_all();

#if ENABLE_SUPER_QUICK_CALIBRATION
			for (write_group = 0, write_test_bgn = 0; write_group
				< RW_MGR_MEM_IF_WRITE_DQS_WIDTH; write_group++,
				write_test_bgn += RW_MGR_MEM_DQ_PER_WRITE_DQS) {
				IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0,
					      write_group);
				scc_mgr_zero_group(write_group, write_test_bgn,
						   0);
			}
#endif

			run_groups = ~param->skip_groups;

			for (write_group = 0, write_test_bgn = 0; write_group
				< RW_MGR_MEM_IF_WRITE_DQS_WIDTH; write_group++,
				write_test_bgn += RW_MGR_MEM_DQ_PER_WRITE_DQS) {
				/* Initialized the group failure */
				group_failed = 0;

#if RLDRAMX || QDRII
				/*
				 *  It seems that with rldram and qdr vfifo
				 *  starts at max (not sure for ddr)
				 *  also not sure if max is really vfifo_size-1
				 *  or vfifo_size
				 */
				BFM_GBL_SET(vfifo_idx, VFIFO_SIZE-1);
#else
				BFM_GBL_SET(vfifo_idx, 0);
#endif
				current_run = run_groups & ((1 <<
					RW_MGR_NUM_DQS_PER_WRITE_GROUP) - 1);
				run_groups = run_groups >>
					RW_MGR_NUM_DQS_PER_WRITE_GROUP;

				if (current_run == 0)
					continue;

				IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0,
					      write_group);
#if !ENABLE_SUPER_QUICK_CALIBRATION
				scc_mgr_zero_group(write_group, write_test_bgn,
						   0);
#endif

				for (read_group = write_group *
					RW_MGR_MEM_IF_READ_DQS_WIDTH /
					RW_MGR_MEM_IF_WRITE_DQS_WIDTH,
					read_test_bgn = 0;
					read_group < (write_group + 1) *
					RW_MGR_MEM_IF_READ_DQS_WIDTH /
					RW_MGR_MEM_IF_WRITE_DQS_WIDTH &&
					group_failed == 0;
					read_group++, read_test_bgn +=
					RW_MGR_MEM_DQ_PER_READ_DQS) {
					/* Calibrate the VFIFO */
					if (!((STATIC_CALIB_STEPS) &
						CALIB_SKIP_VFIFO)) {
						if (!rw_mgr_mem_calibrate_vfifo
							(read_group,
							read_test_bgn)) {
							group_failed = 1;

							if (!(gbl->
							phy_debug_mode_flags &
						PHY_DEBUG_SWEEP_ALL_GROUPS)) {
								return 0;
							}
						}
					}
				}

				/*
				 * level writes (or align DK with CK
				 * for RLDRAMX).
				 */
				if (group_failed == 0) {
					if ((DDRX || RLDRAMII) && !(ARRIAV ||
					     CYCLONEV)) {
						if (!((STATIC_CALIB_STEPS) &
							CALIB_SKIP_WLEVEL)) {
						if (!rw_mgr_mem_calibrate_wlevel
						(write_group, write_test_bgn)) {
							group_failed = 1;

						if (!(gbl->phy_debug_mode_flags
						& PHY_DEBUG_SWEEP_ALL_GROUPS)) {
								return 0;
								}
							}
						}
					}
				}

				/* Calibrate the output side */
				if (group_failed == 0)	{
					for (rank_bgn = 0, sr = 0; rank_bgn
						< RW_MGR_MEM_NUMBER_OF_RANKS;
						rank_bgn +=
						NUM_RANKS_PER_SHADOW_REG,
						++sr) {
						sr_failed = 0;
						if (!((STATIC_CALIB_STEPS) &
						CALIB_SKIP_WRITES)) {
							if ((STATIC_CALIB_STEPS)
						& CALIB_SKIP_DELAY_SWEEPS) {
						/* not needed in quick mode! */
							} else {
						/*
						 * Determine if this set of
						 * ranks should be skipped
						 * entirely.
						 */
					if (!param->skip_shadow_regs[sr]) {
						/* Select shadow register set */
						select_shadow_regs_for_update
						(rank_bgn, write_group, 1);

						if (!rw_mgr_mem_calibrate_writes
						(rank_bgn, write_group,
						write_test_bgn)) {
							sr_failed = 1;
							if (!(gbl->
							phy_debug_mode_flags &
						PHY_DEBUG_SWEEP_ALL_GROUPS)) {
								return 0;
									}
									}
								}
							}
						}
						if (sr_failed != 0)
							group_failed = 1;
					}
				}

#if READ_AFTER_WRITE_CALIBRATION
				if (group_failed == 0) {
					for (read_group = write_group *
					RW_MGR_MEM_IF_READ_DQS_WIDTH /
					RW_MGR_MEM_IF_WRITE_DQS_WIDTH,
					read_test_bgn = 0;
						read_group < (write_group + 1)
						* RW_MGR_MEM_IF_READ_DQS_WIDTH
						/ RW_MGR_MEM_IF_WRITE_DQS_WIDTH &&
						group_failed == 0;
						read_group++, read_test_bgn +=
						RW_MGR_MEM_DQ_PER_READ_DQS) {
						if (!((STATIC_CALIB_STEPS) &
							CALIB_SKIP_WRITES)) {
					if (!rw_mgr_mem_calibrate_vfifo_end
						(read_group, read_test_bgn)) {
							group_failed = 1;

						if (!(gbl->phy_debug_mode_flags
						& PHY_DEBUG_SWEEP_ALL_GROUPS)) {
								return 0;
								}
							}
						}
					}
				}
#endif

				if (group_failed != 0)
					failing_groups++;
			}

			/*
			 * USER If there are any failing groups then report
			 * the failure.
			 */
			if (failing_groups != 0)
				return 0;

			/* Calibrate the LFIFO */
			if (!((STATIC_CALIB_STEPS) & CALIB_SKIP_LFIFO)) {
				/*
				 * If we're skipping groups as part of debug,
				 * don't calibrate LFIFO.
				 */
				if (param->skip_groups == 0) {
					if (!rw_mgr_mem_calibrate_lfifo())
						return 0;
				}
			}
		}
	}

	/*
	 * Do not remove this line as it makes sure all of our decisions
	 * have been applied.
	 */
	IOWR_32DIRECT(SCC_MGR_UPD, 0, 0);
	return 1;
}

uint32_t run_mem_calibrate(void)
{
	uint32_t pass;
	uint32_t debug_info;

	/* Reset pass/fail status shown on afi_cal_success/fail */
	IOWR_32DIRECT(PHY_MGR_CAL_STATUS, 0, PHY_MGR_CAL_RESET);

	TRACE_FUNC();

	BFM_STAGE("calibrate");

#if USE_DQS_TRACKING
	/* stop tracking manger */
	uint32_t ctrlcfg = IORD_32DIRECT(CTRL_CONFIG_REG, 0);

	IOWR_32DIRECT(CTRL_CONFIG_REG, 0, ctrlcfg & 0xFFBFFFFF);
#endif

	initialize();
	rw_mgr_mem_initialize();

#if ENABLE_BRINGUP_DEBUGGING
	do_bringup_test();
#endif
	pass = mem_calibrate();

	mem_precharge_and_activate();
	IOWR_32DIRECT(PHY_MGR_CMD_FIFO_RESET, 0, 0);

	if (pass) {
		BFM_STAGE("handoff");

#ifdef TEST_SIZE
		if (!check_test_mem(0)) {
			gbl->error_stage = 0x92;
			gbl->error_group = 0x92;
		}
#endif
	}

	/*
	 * Handoff:
	 * Don't return control of the PHY back to AFI when in debug mode.
	 */
	if ((gbl->phy_debug_mode_flags & PHY_DEBUG_IN_DEBUG_MODE) == 0) {
		rw_mgr_mem_handoff();
#if HARD_PHY
		/*
		 * In Hard PHY this is a 2-bit control:
		 * 0: AFI Mux Select
		 * 1: DDIO Mux Select
		 */
		IOWR_32DIRECT(PHY_MGR_MUX_SEL, 0, 0x2);
#else
		IOWR_32DIRECT(PHY_MGR_MUX_SEL, 0, 0);
#endif
	}

#if USE_DQS_TRACKING
	IOWR_32DIRECT(CTRL_CONFIG_REG, 0, ctrlcfg);
#endif

	if (pass) {
		IPRINT("CALIBRATION PASSED");

		gbl->fom_in /= 2;
		gbl->fom_out /= 2;

		if (gbl->fom_in > 0xff)
			gbl->fom_in = 0xff;

		if (gbl->fom_out > 0xff)
			gbl->fom_out = 0xff;

		/* Update the FOM in the register file */
		debug_info = gbl->fom_in;
		debug_info |= gbl->fom_out << 8;
		IOWR_32DIRECT(REG_FILE_FOM, 0, debug_info);

		IOWR_32DIRECT(PHY_MGR_CAL_DEBUG_INFO, 0, debug_info);
		IOWR_32DIRECT(PHY_MGR_CAL_STATUS, 0, PHY_MGR_CAL_SUCCESS);
	} else {
		IPRINT("CALIBRATION FAILED");

		debug_info = gbl->error_stage;
		debug_info |= gbl->error_substage << 8;
		debug_info |= gbl->error_group << 16;

		IOWR_32DIRECT(REG_FILE_FAILING_STAGE, 0, debug_info);
		IOWR_32DIRECT(PHY_MGR_CAL_DEBUG_INFO, 0, debug_info);
		IOWR_32DIRECT(PHY_MGR_CAL_STATUS, 0, PHY_MGR_CAL_FAIL);

		/* Update the failing group/stage in the register file */
		debug_info = gbl->error_stage;
		debug_info |= gbl->error_substage << 8;
		debug_info |= gbl->error_group << 16;
		IOWR_32DIRECT(REG_FILE_FAILING_STAGE, 0, debug_info);
	}

	return pass;
}

#if HCX_COMPAT_MODE || ENABLE_INST_ROM_WRITE
void hc_initialize_rom_data(void)
{
	uint32_t i;

	for (i = 0; i < inst_rom_init_size; i++) {
		uint32_t data = inst_rom_init[i];
		IOWR_32DIRECT(RW_MGR_INST_ROM_WRITE, (i << 2), data);
	}

	for (i = 0; i < ac_rom_init_size; i++) {
		uint32_t data = ac_rom_init[i];
		IOWR_32DIRECT(RW_MGR_AC_ROM_WRITE, (i << 2), data);
	}
}
#endif

void initialize_reg_file(void)
{
	/* Initialize the register file with the correct data */
	IOWR_32DIRECT(REG_FILE_SIGNATURE, 0, REG_FILE_INIT_SEQ_SIGNATURE);
	IOWR_32DIRECT(REG_FILE_DEBUG_DATA_ADDR, 0, 0);
	IOWR_32DIRECT(REG_FILE_CUR_STAGE, 0, 0);
	IOWR_32DIRECT(REG_FILE_FOM, 0, 0);
	IOWR_32DIRECT(REG_FILE_FAILING_STAGE, 0, 0);
	IOWR_32DIRECT(REG_FILE_DEBUG1, 0, 0);
	IOWR_32DIRECT(REG_FILE_DEBUG2, 0, 0);
}

void initialize_hps_phy(void)
{
	/* These may need to be included also: */
	/* wrap_back_en (false) */
	/* atpg_en (false) */
	/* pipelineglobalenable (true) */

	uint32_t reg;
	/*
	 * Tracking also gets configured here because it's in the
	 * same register.
	 */
	uint32_t trk_sample_count = 7500;
	uint32_t trk_long_idle_sample_count = (10 << 16) | 100;
	/*
	 * Format is number of outer loops in the 16 MSB, sample
	 * count in 16 LSB.
	*/

	reg = 0;
#if DDR3 || DDR2
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_ACDELAYEN_SET(2);
#else
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_ACDELAYEN_SET(1);
#endif
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_DQDELAYEN_SET(1);
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_DQSDELAYEN_SET(1);
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_DQSLOGICDELAYEN_SET(1);
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_RESETDELAYEN_SET(0);
#if LPDDR2
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_LPDDRDIS_SET(0);
#else
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_LPDDRDIS_SET(1);
#endif
	/*
	 * This field selects the intrinsic latency to RDATA_EN/FULL path.
	 * 00-bypass, 01- add 5 cycles, 10- add 10 cycles, 11- add 15 cycles.
	 */
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_ADDLATSEL_SET(0);
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_SAMPLECOUNT_19_0_SET(
		trk_sample_count);
	IOWR_32DIRECT(BASE_MMR, SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_OFFSET, reg);

	reg = 0;
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_1_SAMPLECOUNT_31_20_SET(
		trk_sample_count >>
		SDR_CTRLGRP_PHYCTRL_PHYCTRL_0_SAMPLECOUNT_19_0_WIDTH);
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_1_LONGIDLESAMPLECOUNT_19_0_SET(
		trk_long_idle_sample_count);
	IOWR_32DIRECT(BASE_MMR, SDR_CTRLGRP_PHYCTRL_PHYCTRL_1_OFFSET, reg);

	reg = 0;
	reg |= SDR_CTRLGRP_PHYCTRL_PHYCTRL_2_LONGIDLESAMPLECOUNT_31_20_SET(
		trk_long_idle_sample_count >>
		SDR_CTRLGRP_PHYCTRL_PHYCTRL_1_LONGIDLESAMPLECOUNT_19_0_WIDTH);
	IOWR_32DIRECT(BASE_MMR, SDR_CTRLGRP_PHYCTRL_PHYCTRL_2_OFFSET, reg);
}

#if USE_DQS_TRACKING
void initialize_tracking(void)
{
	uint32_t concatenated_longidle = 0x0;
	uint32_t concatenated_delays = 0x0;
	uint32_t concatenated_rw_addr = 0x0;
	uint32_t concatenated_refresh = 0x0;
	uint32_t dtaps_per_ptap;
	uint32_t tmp_delay;

	/*
	 * compute usable version of value in case we skip full
	 * computation later
	 */
	dtaps_per_ptap = 0;
	tmp_delay = 0;
	while (tmp_delay < IO_DELAY_PER_OPA_TAP) {
		dtaps_per_ptap++;
		tmp_delay += IO_DELAY_PER_DCHAIN_TAP;
	}
	dtaps_per_ptap--;

	concatenated_longidle = concatenated_longidle ^ 10;
		/*longidle outer loop */
	concatenated_longidle = concatenated_longidle << 16;
	concatenated_longidle = concatenated_longidle ^ 100;
		/*longidle sample count */
	concatenated_delays = concatenated_delays ^ 243;
		/* trfc, worst case of 933Mhz 4Gb */
	concatenated_delays = concatenated_delays << 8;
	concatenated_delays = concatenated_delays ^ 14;
		/* trcd, worst case */
	concatenated_delays = concatenated_delays << 8;
	concatenated_delays = concatenated_delays ^ 10;
		/* vfifo wait */
	concatenated_delays = concatenated_delays << 8;
	concatenated_delays = concatenated_delays ^ 4;
		/* mux delay */

#if DDR3 || LPDDR2
	concatenated_rw_addr = concatenated_rw_addr ^ __RW_MGR_IDLE;
	concatenated_rw_addr = concatenated_rw_addr << 8;
	concatenated_rw_addr = concatenated_rw_addr ^ __RW_MGR_ACTIVATE_1;
	concatenated_rw_addr = concatenated_rw_addr << 8;
	concatenated_rw_addr = concatenated_rw_addr ^ __RW_MGR_SGLE_READ;
	concatenated_rw_addr = concatenated_rw_addr << 8;
	concatenated_rw_addr = concatenated_rw_addr ^ __RW_MGR_PRECHARGE_ALL;
#endif

#if DDR3 || LPDDR2
	concatenated_refresh = concatenated_refresh ^ __RW_MGR_REFRESH_ALL;
#else
	concatenated_refresh = concatenated_refresh ^ 0;
#endif
	concatenated_refresh = concatenated_refresh << 24;
	concatenated_refresh = concatenated_refresh ^ 1000; /* trefi */

	/* Initialize the register file with the correct data */
	IOWR_32DIRECT(REG_FILE_DTAPS_PER_PTAP, 0, dtaps_per_ptap);
	IOWR_32DIRECT(REG_FILE_TRK_SAMPLE_COUNT, 0, 7500);
	IOWR_32DIRECT(REG_FILE_TRK_LONGIDLE, 0, concatenated_longidle);
	IOWR_32DIRECT(REG_FILE_DELAYS, 0, concatenated_delays);
	IOWR_32DIRECT(REG_FILE_TRK_RW_MGR_ADDR, 0, concatenated_rw_addr);
	IOWR_32DIRECT(REG_FILE_TRK_READ_DQS_WIDTH, 0,
		      RW_MGR_MEM_IF_READ_DQS_WIDTH);
	IOWR_32DIRECT(REG_FILE_TRK_RFSH, 0, concatenated_refresh);
}

#else

void initialize_tracking(void)
{
	uint32_t concatenated_longidle = 0x0;
	uint32_t concatenated_delays = 0x0;
	uint32_t concatenated_rw_addr = 0x0;
	uint32_t concatenated_refresh = 0x0;
	uint32_t dtaps_per_ptap;
	uint32_t tmp_delay;

	/* compute usable version of value in case we skip full
	computation later */
	dtaps_per_ptap = 0;
	tmp_delay = 0;
	while (tmp_delay < IO_DELAY_PER_OPA_TAP) {
		dtaps_per_ptap++;
		tmp_delay += IO_DELAY_PER_DCHAIN_TAP;
	}
	dtaps_per_ptap--;

	concatenated_longidle = concatenated_longidle ^ 10;
		/*longidle outer loop */
	concatenated_longidle = concatenated_longidle << 16;
	concatenated_longidle = concatenated_longidle ^ 100;
		/*longidle sample count */
#if FULL_RATE
	concatenated_delays = concatenated_delays ^ 60; /* trfc */
#endif
#if HALF_RATE
	concatenated_delays = concatenated_delays ^ 30; /* trfc */
#endif
#if QUARTER_RATE
	concatenated_delays = concatenated_delays ^ 15; /* trfc */
#endif
	concatenated_delays = concatenated_delays << 8;
#if FULL_RATE
	concatenated_delays = concatenated_delays ^ 4; /* trcd */
#endif
#if HALF_RATE
	concatenated_delays = concatenated_delays ^ 2; /* trcd */
#endif
#if QUARTER_RATE
	concatenated_delays = concatenated_delays ^ 0; /* trcd */
#endif
	concatenated_delays = concatenated_delays << 8;
#if FULL_RATE
	concatenated_delays = concatenated_delays ^ 5; /* vfifo wait */
#endif
#if HALF_RATE
	concatenated_delays = concatenated_delays ^ 3; /* vfifo wait */
#endif
#if QUARTER_RATE
	concatenated_delays = concatenated_delays ^ 1; /* vfifo wait */
#endif
	concatenated_delays = concatenated_delays << 8;
#if FULL_RATE
	concatenated_delays = concatenated_delays ^ 4; /* mux delay */
#endif
#if HALF_RATE
	concatenated_delays = concatenated_delays ^ 2; /* mux delay */
#endif
#if QUARTER_RATE
	concatenated_delays = concatenated_delays ^ 0; /* mux delay */
#endif

#if DDR3 || LPDDR2
	concatenated_rw_addr = concatenated_rw_addr ^ __RW_MGR_IDLE;
	concatenated_rw_addr = concatenated_rw_addr << 8;
	concatenated_rw_addr = concatenated_rw_addr ^ __RW_MGR_ACTIVATE_1;
	concatenated_rw_addr = concatenated_rw_addr << 8;
	concatenated_rw_addr = concatenated_rw_addr ^ __RW_MGR_SGLE_READ;
	concatenated_rw_addr = concatenated_rw_addr << 8;
	concatenated_rw_addr = concatenated_rw_addr ^ __RW_MGR_PRECHARGE_ALL;
#endif

#if DDR3 || LPDDR2
	concatenated_refresh = concatenated_refresh ^ __RW_MGR_REFRESH_ALL;
#else
	concatenated_refresh = concatenated_refresh ^ 0;
#endif
	concatenated_refresh = concatenated_refresh << 24;
	concatenated_refresh = concatenated_refresh ^ 546; /* trefi */

	IOWR_32DIRECT(TRK_DTAPS_PER_PTAP, 0, dtaps_per_ptap);
	IOWR_32DIRECT(TRK_SAMPLE_COUNT, 0, 7500);
	IOWR_32DIRECT(TRK_LONGIDLE, 0, concatenated_longidle);
	IOWR_32DIRECT(TRK_DELAYS, 0, concatenated_delays);
	IOWR_32DIRECT(TRK_RW_MGR_ADDR, 0, concatenated_rw_addr);
	IOWR_32DIRECT(TRK_READ_DQS_WIDTH, 0, RW_MGR_MEM_IF_READ_DQS_WIDTH);
	IOWR_32DIRECT(TRK_RFSH, 0, concatenated_refresh);
}
#endif	/* USE_DQS_TRACKING */

void user_init_cal_req(void)
{
	uint32_t scc_afi_reg;

	scc_afi_reg = IORD_32DIRECT(SCC_MGR_AFI_CAL_INIT, 0);

	if (scc_afi_reg == 1) {	/* 1 is initialization request */
		initialize();
		rw_mgr_mem_initialize();
		rw_mgr_mem_handoff();
		IOWR_32DIRECT(PHY_MGR_MUX_SEL, 0, 0);
		IOWR_32DIRECT(PHY_MGR_CAL_STATUS, 0, PHY_MGR_CAL_SUCCESS);
	} else if (scc_afi_reg == 2) {
		run_mem_calibrate();
	}
}

int sdram_calibration(void)
{
	struct param_type my_param;
	struct gbl_type my_gbl;
	uint32_t pass;
	uint32_t i;

	param = &my_param;
	gbl = &my_gbl;

	/* Initialize the debug mode flags */
	gbl->phy_debug_mode_flags = 0;
	/* Set the calibration enabled by default */
	gbl->phy_debug_mode_flags |= PHY_DEBUG_ENABLE_CAL_RPT;
	/* Only enable margining by default if requested */
#if ENABLE_MARGIN_REPORT_GEN
	gbl->phy_debug_mode_flags |= PHY_DEBUG_ENABLE_MARGIN_RPT;
#endif
	/* Only sweep all groups (regardless of fail state) by default
	if requested */
#if ENABLE_SWEEP_ALL_GROUPS
	gbl->phy_debug_mode_flags |= PHY_DEBUG_SWEEP_ALL_GROUPS;
#endif
	/*Set enabled read test by default */
#if DISABLE_GUARANTEED_READ
	gbl->phy_debug_mode_flags |= PHY_DEBUG_DISABLE_GUARANTEED_READ;
#endif
	/* Initialize the register file */
	initialize_reg_file();

	/* Initialize any PHY CSR */
	initialize_hps_phy();

	scc_mgr_initialize();

#if USE_DQS_TRACKING
	initialize_tracking();
#endif

	/* USER Enable all ranks, groups */
	for (i = 0; i < RW_MGR_MEM_NUMBER_OF_RANKS; i++)
		param->skip_ranks[i] = 0;
	for (i = 0; i < NUM_SHADOW_REGS; ++i)
		param->skip_shadow_regs[i] = 0;
	param->skip_groups = 0;

	IPRINT("Preparing to start memory calibration");

	TRACE_FUNC();
	DPRINT(1, "%s%s %s ranks=%lu cs/dimm=%lu dq/dqs=%lu,%lu vg/dqs=%lu,%lu",
	       RDIMM ? "r" : (LRDIMM ? "l" : ""),
	       DDR2 ? "DDR2" : (DDR3 ? "DDR3" : (QDRII ? "QDRII" : (RLDRAMII ?
	       "RLDRAMII" : (RLDRAM3 ? "RLDRAM3" : "??PROTO??")))),
	       FULL_RATE ? "FR" : (HALF_RATE ? "HR" : (QUARTER_RATE ?
	       "QR" : "??RATE??")),
	       (long unsigned int)RW_MGR_MEM_NUMBER_OF_RANKS,
	       (long unsigned int)RW_MGR_MEM_NUMBER_OF_CS_PER_DIMM,
	       (long unsigned int)RW_MGR_MEM_DQ_PER_READ_DQS,
	       (long unsigned int)RW_MGR_MEM_DQ_PER_WRITE_DQS,
	       (long unsigned int)RW_MGR_MEM_VIRTUAL_GROUPS_PER_READ_DQS,
	       (long unsigned int)RW_MGR_MEM_VIRTUAL_GROUPS_PER_WRITE_DQS);
	DPRINT(1, "dqs=%lu,%lu dq=%lu dm=%lu ptap_delay=%lu dtap_delay=%lu",
	       (long unsigned int)RW_MGR_MEM_IF_READ_DQS_WIDTH,
	       (long unsigned int)RW_MGR_MEM_IF_WRITE_DQS_WIDTH,
	       (long unsigned int)RW_MGR_MEM_DATA_WIDTH,
	       (long unsigned int)RW_MGR_MEM_DATA_MASK_WIDTH,
	       (long unsigned int)IO_DELAY_PER_OPA_TAP,
	       (long unsigned int)IO_DELAY_PER_DCHAIN_TAP);
	DPRINT(1, "dtap_dqsen_delay=%lu, dll=%lu",
	       (long unsigned int)IO_DELAY_PER_DQS_EN_DCHAIN_TAP,
	       (long unsigned int)IO_DLL_CHAIN_LENGTH);
	DPRINT(1, "max values: en_p=%lu dqdqs_p=%lu en_d=%lu dqs_in_d=%lu",
	       (long unsigned int)IO_DQS_EN_PHASE_MAX,
	       (long unsigned int)IO_DQDQS_OUT_PHASE_MAX,
	       (long unsigned int)IO_DQS_EN_DELAY_MAX,
	       (long unsigned int)IO_DQS_IN_DELAY_MAX);
	DPRINT(1, "io_in_d=%lu io_out1_d=%lu io_out2_d=%lu",
	       (long unsigned int)IO_IO_IN_DELAY_MAX,
	       (long unsigned int)IO_IO_OUT1_DELAY_MAX,
	       (long unsigned int)IO_IO_OUT2_DELAY_MAX);
	DPRINT(1, "dqs_in_reserve=%lu dqs_out_reserve=%lu",
	       (long unsigned int)IO_DQS_IN_RESERVE,
	       (long unsigned int)IO_DQS_OUT_RESERVE);

#if HCX_COMPAT_MODE || ENABLE_INST_ROM_WRITE
	hc_initialize_rom_data();
#endif

#if !HARD_PHY
	/* Hard PHY does not support soft reset */
	IOWR_32DIRECT(RW_MGR_SOFT_RESET, 0, 0);
#endif

	/* update info for sims */
	reg_file_set_stage(CAL_STAGE_NIL);
	reg_file_set_group(0);

	/*
	 * Load global needed for those actions that require
	 * some dynamic calibration support.
	 */
#if HARD_PHY
	dyn_calib_steps = STATIC_CALIB_STEPS;
#else
	dyn_calib_steps = IORD_32DIRECT(PHY_MGR_CALIB_SKIP_STEPS, 0);
#endif
	/*
	 * Load global to allow dynamic selection of delay loop settings
	 * based on calibration mode.
	 */
	if (!((DYNAMIC_CALIB_STEPS) & CALIB_SKIP_DELAY_LOOPS))
		skip_delay_mask = 0xff;
	else
		skip_delay_mask = 0x0;

#ifdef TEST_SIZE
	if (!check_test_mem(1)) {
		IOWR_32DIRECT(PHY_MGR_CAL_DEBUG_INFO, 0, 0x9090);
		IOWR_32DIRECT(PHY_MGR_CAL_STATUS, 0, PHY_MGR_CAL_FAIL);
	}
	write_test_mem();
	if (!check_test_mem(0)) {
		IOWR_32DIRECT(PHY_MGR_CAL_DEBUG_INFO, 0, 0x9191);
		IOWR_32DIRECT(PHY_MGR_CAL_STATUS, 0, PHY_MGR_CAL_FAIL);
	}
#endif
	pass = run_mem_calibrate();

	IPRINT("Calibration complete");
	/* Send the end of transmission character */
	IPRINT("%c", 0x4);
	return pass;
}


#if ENABLE_BRINGUP_DEBUGGING
/* Bring-Up test Support */
void do_bringup_test_guaranteed_write(void)
{
	uint32_t r;

	TRACE_FUNC();

	for (r = 0; r < RW_MGR_MEM_NUMBER_OF_RANKS; r++) {
		if (param->skip_ranks[r]) {
			/* request to skip the rank */
			continue;
		}

		/* set rank */
		set_rank_and_odt_mask(r, RW_MGR_ODT_MODE_READ_WRITE);

		/* Load up a constant bursts */
		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 0x20);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0,
			      __RW_MGR_GUARANTEED_WRITE_0_1_A_5_WAIT0);

		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, 0x20);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
			      __RW_MGR_GUARANTEED_WRITE_0_1_A_5_WAIT1);

		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_2, 0, 0x20);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_2, 0,
			      __RW_MGR_GUARANTEED_WRITE_0_1_A_5_WAIT2);

		IOWR_32DIRECT(RW_MGR_LOAD_CNTR_3, 0, 0x20);
		IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_3, 0,
			      __RW_MGR_GUARANTEED_WRITE_0_1_A_5_WAIT3);

		IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, 0,
			      __RW_MGR_GUARANTEED_WRITE_0_1_A_5);
	}

	set_rank_and_odt_mask(0, RW_MGR_ODT_MODE_OFF);
}

void do_bringup_test_clear_di_buf(uint32_t group)
{
	IOWR_32DIRECT(PHY_MGR_CMD_FIFO_RESET, 0, 0);
	IOWR_32DIRECT(RW_MGR_RESET_READ_DATAPATH, 0, 0);

	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 128);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_DO_CLEAR_DI_BUF);

	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, group << 2,
		      __RW_MGR_DO_CLEAR_DI_BUF);
}

void do_bringup_test_guaranteed_read(uint32_t group)
{
	IOWR_32DIRECT(PHY_MGR_CMD_FIFO_RESET, 0, 0);
	IOWR_32DIRECT(RW_MGR_RESET_READ_DATAPATH, 0, 0);

	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_0, 0, 16);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_0, 0, __RW_MGR_DO_TEST_READ);
	IOWR_32DIRECT(RW_MGR_LOAD_CNTR_1, 0, 16);
	IOWR_32DIRECT(RW_MGR_LOAD_JUMP_ADD_1, 0,
		      __RW_MGR_DO_TEST_READ_POST_WAIT);

	IOWR_32DIRECT(RW_MGR_RUN_SINGLE_GROUP, group << 2,
		      __RW_MGR_DO_TEST_READ);
}

void do_bringup_test(void)
{
	int i;
	uint32_t group;
	uint32_t v = 0;

	group = 0;

	mem_config();

	/* 15 is the maximum latency (should make dependent on actual design */
	IOWR_32DIRECT(PHY_MGR_PHY_RLAT, 0, 15); /* lfifo setting */

#if ARRIAV || CYCLONEV
	for (i = 0; i < RW_MGR_MEM_IF_READ_DQS_WIDTH; i++) {
		IOWR_32DIRECT(SCC_MGR_GROUP_COUNTER, 0, i);
		scc_set_bypass_mode(i, 0);
	}
#endif

	/* initialize global buffer to something known */
	for (i = 0; i < sizeof(di_buf_gbl); i++)
		di_buf_gbl[i] = 0xee;

	/* pre-increment vfifo to ensure not at max value */
	rw_mgr_incr_vfifo(group, &v);
	rw_mgr_incr_vfifo(group, &v);

	do_bringup_test_clear_di_buf(group);

	while (1) {
		do_bringup_test_guaranteed_write();
		do_bringup_test_guaranteed_read(group);
		load_di_buf_gbl();
		rw_mgr_incr_vfifo(group, &v);
	}
}


#endif /* ENABLE_BRINGUP_DEBUGGING */

#if ENABLE_ASSERT
void err_report_internal_error(
	const char *description,
	const char *module,
	const char *file,
	int line
)
{
	void *array[10];
	size_t size;
	char **strings;
	size_t i;

	fprintf(stderr, ERR_IE_TEXT, module, file, line, description, "\n");

	size = backtrace(array, 10);
	strings = backtrace_symbols(array, size);

	fprintf(stderr, "Obtained %zd stack frames.\n", size);

	for (i = 0; i < size; i++)
		fprintf(stderr, "%s\n", strings[i]);

	free(strings);
}
#endif
