/* linux/include/linux/amba/pl330.h
 *
 * Copyright (C) 2010 Samsung Electronics Co. Ltd.
 *	Jaswinder Singh <jassi.brar@samsung.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	__PL330_H_
#define	__PL330_H_

#define PL330_DMA_MAX_BURST_SIZE	3

/* structure to be passed in for pl330_transfer_x */
struct pl330_transfer_struct {
	u32 reg_base;
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
};

/*
 * Request Configuration.
 * The PL330 core does not modify this and uses the last
 * working configuration if the request doesn't provide any.
 *
 * The Client may want to provide this info only for the
 * first request and a request with new settings.
 */
struct pl330_reqcfg {
	/* Address Incrementing */
	unsigned dst_inc:1;
	unsigned src_inc:1;

	/*
	 * For now, the SRC & DST protection levels
	 * and burst size/length are assumed same.
	 */
	int nonsecure;
	int privileged;
	int insnaccess;
	unsigned brst_len:5;
	unsigned brst_size:3; /* in power of 2 */

	enum pl330_dstcachectrl dcctl;
	enum pl330_srccachectrl scctl;
	enum pl330_byteswap swap;
};
#endif	/* __PL330_H_ */
