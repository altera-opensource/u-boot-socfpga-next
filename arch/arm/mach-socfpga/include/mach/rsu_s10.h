/*
 * Copyright (C) 2018 Intel Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0
 */
#ifndef _RSU_S10_H_
#define _RSU_S10_H_

extern u32 smc_rsu_update_address;

#define RSU_S10_CPB_MAGIC_NUMBER	0x57789609
#define RSU_S10_SPT_MAGIC_NUMBER	0x57713427

#define SPT0_INDEX	1
#define SPT1_INDEX	3

/* CMF pointer block */
struct socfpga_rsu_s10_cpb {
	u32 magic_number;
	u32 header_size;
	u32 total_size;
	u32 reserved1;
	u32 iptab_offset;
	u32 nslots;
	u32 reserved2;
	u64 pointer_slot[508];
};

/* sub partition slot */
struct socfpga_rsu_s10_spt_slot {
	char name[16];
	u32 offset[2];
	u32 length;
	u32 flag;
};

/* sub partition table */
struct socfpga_rsu_s10_spt {
	u32 magic_number;
	u32 version;
	u32 entries;
	u32 reserved[5];
	struct socfpga_rsu_s10_spt_slot spt_slot[127];
};

#endif /* _RSU_S10_H_ */
