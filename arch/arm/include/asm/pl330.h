/* linux/include/linux/amba/pl330.h
 *
 * Copyright (C) 2010 Samsung Electronics Co. Ltd.
 *	Jaswinder Singh <jassi.brar@samsung.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	__PL330_H_
#define	__PL330_H_

struct dma_pl330_platdata {
	/*
	 * Number of valid peripherals connected to DMAC.
	 * This may be different from the value read from
	 * CR0, as the PL330 implementation might have 'holes'
	 * in the peri list or the peri could also be reached
	 * from another DMAC which the platform prefers.
	 */
	u8 nr_valid_peri;
	/* Array of valid peripherals */
	u8 *peri_id;
	/* Operational capabilities */
	dma_cap_mask_t cap_mask;
	/* Bytes to allocate for MC buffer */
	unsigned mcbuf_sz;
};

/* structure to be passed in for pl330_transfer_x */
struct pl330_transfer_struct {
	u32 channel_num;
	u32 src_addr;
	u32 dst_addr;
	u32 size_byte;
	u32 brst_size;
	u32 single_brst_size;
	u32 brst_len;
	u32 peripheral_id;
	u32 transfer_type;
	u32 enable_cache1;
	u32 buf_size;
	u8 *buf;
};/* structure to be passed in for pl330_transfer_x */
#endif	/* __PL330_H_ */
