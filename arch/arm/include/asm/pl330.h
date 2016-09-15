/* linux/include/linux/amba/pl330.h
 *
 * Copyright (C) 2010 Samsung Electronics Co. Ltd.
 *	Jaswinder Singh <jassi.brar@samsung.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	__PL330_H_
#define	__PL330_H_

#define PL330_STATE_STOPPED		(1 << 0)
#define PL330_STATE_EXECUTING		(1 << 1)
#define PL330_STATE_WFE			(1 << 2)
#define PL330_STATE_FAULTING		(1 << 3)
#define PL330_STATE_COMPLETING		(1 << 4)
#define PL330_STATE_WFP			(1 << 5)
#define PL330_STATE_KILLING		(1 << 6)
#define PL330_STATE_FAULT_COMPLETING	(1 << 7)
#define PL330_STATE_CACHEMISS		(1 << 8)
#define PL330_STATE_UPDTPC		(1 << 9)
#define PL330_STATE_ATBARRIER		(1 << 10)
#define PL330_STATE_QUEUEBUSY		(1 << 11)
#define PL330_STATE_INVALID		(1 << 15)

#define PL330_DMA_MAX_BURST_SIZE	3

/* structure to be passed in for pl330_transfer_x */
struct pl330_transfer_struct {
	void __iomem *reg_base;
	u32 channel_num;
	u32 src_addr;
	u32 dst_addr;
	u32 len;
	u32 brst_size;
	u32 single_brst_size;
	u32 brst_len;
	u32 peripheral_id;
	u32 transfer_type;
	u32 enable_cache1;
	u32 buf_size;
	u8 *buf;
};

enum pl330_srccachectrl {
	SCCTRL0 = 0,	/* Noncacheable and nonbufferable */
	SCCTRL1,	/* Bufferable only */
	SCCTRL2,	/* Cacheable, but do not allocate */
	SCCTRL3,	/* Cacheable and bufferable, but do not allocate */
	SINVALID1,
	SINVALID2,
	SCCTRL6,	/* Cacheable write-through, allocate on reads only */
	SCCTRL7,	/* Cacheable write-back, allocate on reads only */
};

enum pl330_dstcachectrl {
	DCCTRL0 = 0,	/* Noncacheable and nonbufferable */
	DCCTRL1,	/* Bufferable only */
	DCCTRL2,	/* Cacheable, but do not allocate */
	DCCTRL3,	/* Cacheable and bufferable, but do not allocate */
	DINVALID1 = 8,
	DINVALID2,
	DCCTRL6,	/* Cacheable write-through, allocate on writes only */
	DCCTRL7,	/* Cacheable write-back, allocate on writes only */
};

enum pl330_byteswap {
	SWAP_NO = 0,
	SWAP_2,
	SWAP_4,
	SWAP_8,
	SWAP_16,
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

void arm_pl330_transfer(struct pl330_transfer_struct *pl330);
#endif	/* __PL330_H_ */
