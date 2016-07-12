/* linux/include/linux/amba/pl330.h
 *
 * Copyright (C) 2010 Samsung Electronics Co. Ltd.
 *	Jaswinder Singh <jassi.brar@samsung.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef	__PL330_H_
#define	__PL330_H_

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
