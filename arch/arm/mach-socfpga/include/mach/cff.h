/*
 * Copyright (C) 2017 Intel Corporation <www.intel.com>
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#ifndef	_SOCFPGA_CFF_H_
#define	_SOCFPGA_CFF_H_

#include <fpga.h>
#include <nand.h>

#ifndef __ASSEMBLY_

struct flash_info {
	char *filename;
	int dev_part;
};

enum rbf_type {unknown, periph_section, core_section};
enum rbf_security {invalid, unencrypted, encrypted};

struct rbf_info {
	enum rbf_type section ;
	enum rbf_security security;
};

struct cff_flash_info {
	struct flash_info sdmmc_flashinfo;
	u32 buffer[4096] __aligned(ARCH_DMA_MINALIGN);
	u32 remaining;
	u32 flash_offset;
	struct rbf_info rbfinfo;
	struct image_header header;
};

#ifdef CONFIG_SPL_BUILD
int cff_from_sdmmc_env(void);
#endif
int cff_from_flash(fpga_fs_info *fpga_fsinfo);
const char *get_cff_filename(const void *fdt, int *len);
#endif /* __ASSEMBLY__ */

#endif /* _SOCFPGA_CFF_H_ */
