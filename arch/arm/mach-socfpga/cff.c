/*
 * COPYRIGHT (C) 2017 Intel Corporation <www.intel.com>
 *
 * SPDX-License-Identifier:    GPL-2.0
 */

#include <altera.h>
#include <common.h>
#include <asm/io.h>
#include <asm/arch/cff.h>
#include <asm/arch/fpga_manager.h>
#include <asm/arch/reset_manager.h>
#include <asm/arch/system_manager.h>
#include <asm/arch/misc.h>
#include <fat.h>
#include <fs.h>
#include <mmc.h>
#include <malloc.h>
#include <watchdog.h>
#include <fdtdec.h>
#include <spi_flash.h>

#define RBF_UNENCRYPTED 0xa65c
#define RBF_ENCRYPTED 0xa65d
#define ARRIA10RBF_PERIPH 0x0001
#define ARRIA10RBF_CORE 0x8001

DECLARE_GLOBAL_DATA_PTR;

static int flash_type = -1;

struct mmc *mmc;

/* Local functions */
static int cff_flash_read(struct cff_flash_info *cff_flashinfo, u32 *buffer,
	u32 *buffer_sizebytes);
static int cff_flash_preinit(struct cff_flash_info *cff_flashinfo,
	fpga_fs_info *fpga_fsinfo, u32 *buffer, u32 *buffer_sizebytes);
static const char *get_cff_filename(const void *fdt, int *len,
	unsigned int core);
#ifndef CONFIG_RBF_SDMMC_FAT_SUPPORT
static int get_cff_offset(const void *fdt, unsigned int core);
#endif

static struct mmc *init_mmc_device(int dev, bool force_init)
{
	struct mmc *mmc;

	mmc = find_mmc_device(dev);
	if (!mmc) {
		printf("no mmc device at slot %x\n", dev);
		return NULL;
	}

	if (force_init)
		mmc->has_init = 0;
	if (mmc_init(mmc))
		return NULL;
	return mmc;
}

static int cff_flash_probe(struct cff_flash_info *cff_flashinfo)
{
	int dev = 0;

	if(BOOT_DEVICE_MMC1 == flash_type)
	{
		mmc = init_mmc_device(dev, true);

		if (!mmc)
			return -1;

#ifdef CONFIG_RBF_SDMMC_FAT_SUPPORT
		/* we are looking at the FAT partition */
		if (fat_register_device(mmc_get_blk_desc(mmc),
			cff_flashinfo->sdmmc_flashinfo.dev_part)) {
			printf("Failed to set filesystem to FAT.\n");
			return -1;
		}
#endif
	}

	return flash_type;
}

static int flash_read(struct cff_flash_info *cff_flashinfo,
	u32 size_read,
	u32 *buffer_ptr)
{
	size_t ret = 1;
#ifdef CONFIG_RBF_SDMMC_FAT_SUPPORT
	loff_t actread;
#endif

	if(BOOT_DEVICE_MMC1 == flash_type) {
#ifdef CONFIG_RBF_SDMMC_FAT_SUPPORT
		ret = fat_read_file(cff_flashinfo->sdmmc_flashinfo.filename,
				buffer_ptr, cff_flashinfo->flash_offset,
				 size_read, &actread);

		if (ret || actread != size_read) {
			printf("Failed to read %s from FAT %d ",
				cff_flashinfo->sdmmc_flashinfo.filename,
				 ret);
			printf("!= %d.\n", size_read);
			return -3;
		} else {
			ret = actread;
		}
#else
		u32 blk = cff_flashinfo->flash_offset/mmc->read_bl_len;
		u32 cnt = size_read / mmc->read_bl_len;

		if (!cnt)
			cnt = 1;

		if((size_read % mmc->read_bl_len) &&
				(size_read >= mmc->read_bl_len))
			cnt++;

		ret = blk_dread(mmc_get_blk_desc(mmc), blk, cnt,
			 buffer_ptr);

		if (cnt != ret)
			return -EIO;
#endif
	}

		return ret;
}

const char *get_cff_filename(const void *fdt, int *len, unsigned int core)
{
	const char *cff_filename = NULL;
	const char *cell;
	int nodeoffset;
	nodeoffset = fdt_subnode_offset(fdt, 0, "chosen");

	if (nodeoffset >= 0) {
		if (core) {
			cell = fdt_getprop(fdt,
					nodeoffset,
					"cffcore-file",
					len);
		} else {
			cell = fdt_getprop(fdt, nodeoffset, "cff-file", len);
		}

		if (cell) {
			cff_filename = cell;
		}
	}
	return cff_filename;
}

#ifndef CONFIG_RBF_SDMMC_FAT_SUPPORT
static int get_cff_offset(const void *fdt, unsigned int core)
{
	int nodeoffset = 0;

	nodeoffset = fdt_subnode_offset(fdt, 0, "chosen");

	if (nodeoffset >= 0) {
		if (core) {
			return fdtdec_get_int(fdt,
					nodeoffset,
					"cffcore-offset",
					-1);
		} else {
			return fdtdec_get_int(fdt,
					nodeoffset,
					"cff-offset",
					-1);
		}
	}
	return -1;
}
#endif

int cff_from_sdmmc_env(unsigned int core)
{
	int rval = -ENOENT;
	fpga_fs_info fpga_fsinfo;

#ifdef CONFIG_RBF_SDMMC_FAT_SUPPORT
	int len = 0;
	const char *cff = NULL;
#else
	char addrToString[32] = {0};

	int sdmmc_rbf_rawaddr = -ENOENT;
#endif

	flash_type = boot_device();

	fpga_fsinfo.interface = "sdmmc";

#ifdef CONFIG_RBF_SDMMC_FAT_SUPPORT
	cff = get_cff_filename(gd->fdt_blob, &len, core);

	/* FAT periph RBF file reading */
	if (cff && (len > 0)) {
		mmc_initialize(gd->bd);

		fpga_fsinfo.filename = (char *)cff;

		fpga_fsinfo.dev_part = getenv("cff_devsel_partition");

		if (NULL == fpga_fsinfo.dev_part) {
			/* FAT partition */
			fpga_fsinfo.dev_part = "1";

			printf("No SD/MMC partition found in environment. ");
			printf("Assuming partition 1.\n");
		}

		rval = cff_from_flash(&fpga_fsinfo);
	}
#else
	sdmmc_rbf_rawaddr = get_cff_offset(gd->fdt_blob);

	 /* RAW periph RBF reading */
	if (sdmmc_rbf_rawaddr >= 0) {
		sprintf(addrToString, "%x", sdmmc_rbf_rawaddr);

		fpga_fsinfo.filename = addrToString;

		rval = cff_from_flash(&fpga_fsinfo);
	}
#endif
	return rval;
}

void get_rbf_image_info(struct rbf_info *rbf, u16 *buffer)
{
	/*
	  Magic ID starting at:
	   -> 1st dword in periph.rbf
	   -> 2nd dword in core.rbf
	*/
	u32 word_reading_max = 2;
	u32 i;

	for(i = 0; i < word_reading_max; i++)
	{
		if(RBF_UNENCRYPTED == *(buffer + i)) /* PERIPH RBF */
			rbf->security = unencrypted;
		else if (RBF_ENCRYPTED == *(buffer + i))
			rbf->security = encrypted;
		else if (RBF_UNENCRYPTED == *(buffer + i + 1)) /* CORE RBF */
					rbf->security = unencrypted;
		else if (RBF_ENCRYPTED == *(buffer + i + 1))
					rbf->security = encrypted;
		else {
			rbf->security = invalid;
			continue;
		}

		/* PERIPH RBF */
		if (ARRIA10RBF_PERIPH == *(buffer + i + 1)) {
			rbf->section = periph_section;
			break;
		}
		else if (ARRIA10RBF_CORE == *(buffer + i + 1)) {
			rbf->section = core_section;
			break;
		} /* CORE RBF */
		else if (ARRIA10RBF_PERIPH == *(buffer + i + 2)) {
			rbf->section = periph_section;
			break;
		}
		else if (ARRIA10RBF_CORE == *(buffer + i + 2)) {
			rbf->section = core_section;
			break;
		}
		else {
			rbf->section = unknown;
			break;
		}
	}

	return;
}

static int cff_flash_preinit(struct cff_flash_info *cff_flashinfo,
	fpga_fs_info *fpga_fsinfo, u32 *buffer, u32 *buffer_sizebytes)
{
	u32 *bufferptr_after_header = NULL;
	u32 buffersize_after_header = 0;
	u32 rbf_header_data_size = 0;
	int ret = 0;
	/* To avoid from keeping re-read the contents */
	struct image_header *header = &(cff_flashinfo->header);
	size_t buffer_size = *buffer_sizebytes;
	u32 *buffer_ptr = (u32 *)*buffer;

#ifdef CONFIG_RBF_SDMMC_FAT_SUPPORT
	cff_flashinfo->sdmmc_flashinfo.filename = fpga_fsinfo->filename;
	cff_flashinfo->flash_offset = 0;
#else
	cff_flashinfo->flash_offset =
		simple_strtoul(fpga_fsinfo->filename, NULL, 16);
#endif

	 /* Load mkimage header into buffer */
	ret = flash_read(cff_flashinfo,
			sizeof(struct image_header), buffer_ptr);

	if (0 >= ret) {
		printf(" Failed to read mkimage header from flash.\n");
		return -4;
	}

	WATCHDOG_RESET();

	memcpy(header, (u_char *)buffer_ptr, sizeof(*header));

	if (!image_check_magic(header)) {
		printf("FPGA: Bad Magic Number.\n");
		return -6;
	}

	if (!image_check_hcrc(header)) {
		printf("FPGA: Bad Header Checksum.\n");
		return -7;
	}

	/* Getting rbf data size */
	cff_flashinfo->remaining =
		image_get_data_size(header);

	/* Calculate total size of both rbf data with mkimage header */
	rbf_header_data_size = cff_flashinfo->remaining +
				sizeof(struct image_header);

	/* Loading to buffer chunk by chunk, normally for OCRAM buffer */
	if (rbf_header_data_size > buffer_size) {
		/* Calculate size of rbf data in the buffer */
		buffersize_after_header =
			buffer_size - sizeof(struct image_header);
		cff_flashinfo->remaining -= buffersize_after_header;
	} else {
	/* Loading whole rbf image into buffer, normally for DDR buffer */
		buffer_size = rbf_header_data_size;
		/* Calculate size of rbf data in the buffer */
		buffersize_after_header =
			buffer_size - sizeof(struct image_header);
		cff_flashinfo->remaining = 0;
	}

	/* Loading mkimage header and rbf data into buffer */
	ret = flash_read(cff_flashinfo, buffer_size, buffer_ptr);

	if (0 >= ret) {
		printf(" Failed to read mkimage header and rbf data ");
		printf("from flash.\n");
		return -5;
	}

	/* Getting pointer of rbf data starting address where is it
	   right after mkimage header */
	bufferptr_after_header =
		(u32 *)((u_char *)buffer_ptr + sizeof(struct image_header));

	/* Update next reading rbf data flash offset */
	cff_flashinfo->flash_offset += buffer_size;

	/* Update the starting addr of rbf data to init FPGA & programming
	   into FPGA */
	*buffer = (u32)bufferptr_after_header;

	get_rbf_image_info(&cff_flashinfo->rbfinfo, (u16 *)bufferptr_after_header);

	/* Update the size of rbf data to be programmed into FPGA */
	*buffer_sizebytes = buffersize_after_header;

#ifdef CONFIG_CHECK_FPGA_DATA_CRC
	cff_flashinfo->datacrc =
		crc32(cff_flashinfo->datacrc,
		(u_char *)bufferptr_after_header,
		buffersize_after_header);
#endif
if (0 == cff_flashinfo->remaining) {
#ifdef CONFIG_CHECK_FPGA_DATA_CRC
	if (cff_flashinfo->datacrc !=
		image_get_dcrc(&(cff_flashinfo->header))) {
		printf("FPGA: Bad Data Checksum.\n");
		return -8;
	}
#endif
}
	return 0;
}


static int cff_flash_read(struct cff_flash_info *cff_flashinfo, u32 *buffer,
	u32 *buffer_sizebytes)
{
	int ret = 0;
	/* To avoid from keeping re-read the contents */
	size_t buffer_size = *buffer_sizebytes;
	u32 *buffer_ptr = (u32 *)*buffer;
	u32 flash_addr = cff_flashinfo->flash_offset;

	/* Buffer allocated in OCRAM */
	/* Read the data by small chunk by chunk. */
	if (cff_flashinfo->remaining > buffer_size)
		cff_flashinfo->remaining -= buffer_size;
	else {
		/* Buffer allocated in DDR, larger than rbf data most
		  of the time */
		buffer_size = cff_flashinfo->remaining;
		cff_flashinfo->remaining = 0;
	}

	ret = flash_read(cff_flashinfo, buffer_size, buffer_ptr);

	if (0 >= ret) {
		printf(" Failed to read rbf data from flash.\n");
		return -9;
	}

#ifdef CONFIG_CHECK_FPGA_DATA_CRC
	cff_flashinfo->datacrc =
		crc32(cff_flashinfo->datacrc,
			(unsigned char *)buffer_ptr, buffer_size);
#endif

if (0 == cff_flashinfo->remaining) {
#ifdef CONFIG_CHECK_FPGA_DATA_CRC
	if (cff_flashinfo->datacrc !=
		image_get_dcrc(&(cff_flashinfo->header))) {
		printf("FPGA: Bad Data Checksum.\n");
		return -8;
	}
#endif
}
	/* Update next reading rbf data flash offset */
	flash_addr += buffer_size;

	cff_flashinfo->flash_offset = flash_addr;

	/* Update the size of rbf data to be programmed into FPGA */
	*buffer_sizebytes = buffer_size;

	return 0;
}

int cff_from_flash(fpga_fs_info *fpga_fsinfo)
{
	u32 buffer = 0;
	u32 buffer_ori = 0;
	u32 buffer_sizebytes = 0;
	u32 buffer_sizebytes_ori = 0;
	struct cff_flash_info cff_flashinfo;
	u32 status;
	int ret = 0;

	memset(&cff_flashinfo, 0, sizeof(cff_flashinfo));

	if (fpga_fsinfo->filename == NULL) {
		printf("no [periph/core] rbf [filename/offset] specified.\n");
		return 0;
	}

	WATCHDOG_RESET();

#ifdef CONFIG_RBF_SDMMC_FAT_SUPPORT
	cff_flashinfo.sdmmc_flashinfo.dev_part =
		simple_strtol(fpga_fsinfo->dev_part, NULL, 10);
#endif

	ret = cff_flash_probe(&cff_flashinfo);

	if (0 >= ret) {
		puts("Flash probe failed.\n");
		return ret;
	}

#ifdef CONFIG_RBF_SDMMC_FAT_SUPPORT
	/* Loading rbf data with DDR, faster than OCRAM,
	   only for core rbf */
	if (gd->ram_size != 0) {
		ret = fat_size(fpga_fsinfo->filename, (loff_t *)&buffer_sizebytes);

		if(ret){
			puts("Failed to read file size.\n");
			return ret;
		}

		buffer_ori = (u32)memalign(ARCH_DMA_MINALIGN, buffer_sizebytes);

		if (!buffer_ori) {
			error("RBF calloc failed!\n");
			return -ENOMEM;
		}

		/* Loading mkimage header and rbf data into
		   DDR instead of OCRAM */
		buffer = buffer_ori;

		buffer_sizebytes_ori = buffer_sizebytes;
	} else {
		buffer = buffer_ori = (u32)cff_flashinfo.buffer;

		buffer_sizebytes =
			buffer_sizebytes_ori =
				sizeof(cff_flashinfo.buffer);
	}
#else
	/* Adjust to adjacent block */
	buffer_sizebytes = buffer_sizebytes_ori =
				(sizeof(cff_flashinfo.buffer)/ mmc->read_bl_len) *
					mmc->read_bl_len;
#endif

	/* Note: Both buffer and buffer_sizebytes values can be altered by
	   function below. */
	ret = cff_flash_preinit(&cff_flashinfo, fpga_fsinfo, &buffer,
		&buffer_sizebytes);

	if (ret)
		return ret;

	if (periph_section == cff_flashinfo.rbfinfo.section) {
		/* initialize the FPGA Manager */
		status = fpgamgr_program_init((u32 *)buffer, buffer_sizebytes);
		if (status) {
			printf("FPGA: Init with periph rbf failed with error. ");
			printf("code %d\n", status);
			return -1;
		}
	}

	WATCHDOG_RESET();

	/* transfer data to FPGA Manager */
	fpgamgr_program_write((const long unsigned int *)buffer,
		buffer_sizebytes);

	WATCHDOG_RESET();

	while (cff_flashinfo.remaining) {
		ret = cff_flash_read(&cff_flashinfo, &buffer_ori,
			&buffer_sizebytes_ori);

		if (ret)
			return ret;

		/* transfer data to FPGA Manager */
		fpgamgr_program_write((const long unsigned int *)buffer_ori,
			buffer_sizebytes_ori);

		WATCHDOG_RESET();
	}

	if ((periph_section == cff_flashinfo.rbfinfo.section) &&
	 is_early_release_fpga_config(gd->fdt_blob)) {
		if (-ETIMEDOUT != fpgamgr_wait_early_user_mode()) {
			printf("FPGA: Early Release Succeeded.\n");
		} else {
			printf("FPGA: Failed to see Early Release.\n");
			return -2;
		}
	} else if ((core_section == cff_flashinfo.rbfinfo.section) ||
	 ((periph_section == cff_flashinfo.rbfinfo.section) &&
	 !is_early_release_fpga_config(gd->fdt_blob))) {
		/* Ensure the FPGA entering config done */
		status = fpgamgr_program_fini();
		if (status)
			return status;
		else
			printf("FPGA: Enter user mode.\n");

	} else {
		printf("Config Error: Unsupported FGPA raw binary type.\n");
		return -3;
	}

	WATCHDOG_RESET();
	return 1;
}
