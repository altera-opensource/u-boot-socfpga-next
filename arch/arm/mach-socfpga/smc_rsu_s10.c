/*
 * Copyright (C) 2018 Intel Corporation. All rights reserved
 *
 * SPDX-License-Identifier:    GPL-2.0
 */

#include <common.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/mailbox_s10.h>
#include <asm/arch/smc_s10.h>
#include <linux/intel-smc.h>

DECLARE_GLOBAL_DATA_PTR;

u32 smc_rsu_update_address __secure_data = 0;

static void __secure smc_socfpga_rsu_status_psci(unsigned long function_id)
{
	SMC_ALLOC_REG_MEM(r);
	u64 rsu_status[4];

	SMC_INIT_REG_MEM(r);

	if (mbox_rsu_status_psci((u32 *)rsu_status, sizeof(rsu_status) / 4)) {
		SMC_ASSIGN_REG_MEM(r, SMC_ARG0, INTEL_SIP_SMC_RSU_ERROR);
		SMC_RET_REG_MEM(r);
		return;
	}

	SMC_ASSIGN_REG_MEM(r, SMC_ARG0, rsu_status[0]);
	SMC_ASSIGN_REG_MEM(r, SMC_ARG1, rsu_status[1]);
	SMC_ASSIGN_REG_MEM(r, SMC_ARG2, rsu_status[2]);
	SMC_ASSIGN_REG_MEM(r, SMC_ARG3, rsu_status[3]);

	SMC_RET_REG_MEM(r);
}

static void __secure smc_socfpga_rsu_update_psci(unsigned long function_id,
					    unsigned long update_address)
{
	SMC_ALLOC_REG_MEM(r);

	SMC_INIT_REG_MEM(r);

	smc_rsu_update_address = update_address;

	SMC_ASSIGN_REG_MEM(r, SMC_ARG0, INTEL_SIP_SMC_STATUS_OK);

	SMC_RET_REG_MEM(r);
}

static void __secure smc_socfpga_rsu_notify_psci(unsigned long function_id,
					    unsigned long execution_stage)
{
	SMC_ALLOC_REG_MEM(r);
	SMC_INIT_REG_MEM(r);

	if (mbox_hps_stage_notify_psci(execution_stage))
		SMC_ASSIGN_REG_MEM(r, SMC_ARG0, INTEL_SIP_SMC_RSU_ERROR);
	else
		SMC_ASSIGN_REG_MEM(r, SMC_ARG0, INTEL_SIP_SMC_STATUS_OK);

	SMC_RET_REG_MEM(r);
}

DECLARE_SECURE_SVC(rsu_status_psci, INTEL_SIP_SMC_RSU_STATUS,
		   smc_socfpga_rsu_status_psci);
DECLARE_SECURE_SVC(rsu_update_psci, INTEL_SIP_SMC_RSU_UPDATE,
		   smc_socfpga_rsu_update_psci);
DECLARE_SECURE_SVC(rsu_notify_psci, INTEL_SIP_SMC_RSU_NOTIFY,
		   smc_socfpga_rsu_notify_psci);
