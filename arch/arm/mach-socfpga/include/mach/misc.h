/*
 * Copyright (C) 2016-2017 Intel Corporation
 *
 * SPDX-License-Identifier:    GPL-2.0
 */

#ifndef _MISC_H_
#define _MISC_H_

void dwmac_deassert_reset(const unsigned int of_reset_id, const u32 phymode);

struct bsel {
	const char	*mode;
	const char	*name;
};

enum {
	BOOT_DEVICE_RAM,
	BOOT_DEVICE_MMC1,
	BOOT_DEVICE_MMC2,
	BOOT_DEVICE_MMC2_2,
	BOOT_DEVICE_NAND,
	BOOT_DEVICE_ONENAND,
	BOOT_DEVICE_NOR,
	BOOT_DEVICE_UART,
	BOOT_DEVICE_SPI,
	BOOT_DEVICE_USB,
	BOOT_DEVICE_SATA,
	BOOT_DEVICE_I2C,
	BOOT_DEVICE_BOARD,
	BOOT_DEVICE_DFU,
	BOOT_DEVICE_NONE
};

extern struct bsel bsel_str[];

#ifdef CONFIG_FPGA
void socfpga_fpga_add(void);
#else
static inline void socfpga_fpga_add(void) {}
#endif

#if defined(CONFIG_TARGET_SOCFPGA_ARRIA10)
unsigned int dedicated_uart_com_port(const void *blob);
unsigned int shared_uart_com_port(const void *blob);
unsigned int uart_com_port(const void *blob);
int is_early_release_fpga_config(const void *blob);
int is_chosen_boolean_true(const void *blob, const char *name);
u32 boot_device(void);
#endif

#endif /* _MISC_H_ */
