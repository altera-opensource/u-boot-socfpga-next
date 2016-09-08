/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Copyright (C) 2010 Samsung Electronics Co. Ltd.
 *	Jaswinder Singh <jassi.brar@samsung.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <asm/io.h>
#include <common.h>
#include <dma.h>
#include <dm/device.h>
#include <asm/pl330.h>

struct dma_pl330_platdata {
	u32 base;
};

/* Register and Bit field Definitions */
#define DS			0x0
#define DS_ST_STOP		0x0
#define DS_ST_EXEC		0x1
#define DS_ST_CMISS		0x2
#define DS_ST_UPDTPC		0x3
#define DS_ST_WFE		0x4
#define DS_ST_ATBRR		0x5
#define DS_ST_QBUSY		0x6
#define DS_ST_WFP		0x7
#define DS_ST_KILL		0x8
#define DS_ST_CMPLT		0x9
#define DS_ST_FLTCMP		0xe
#define DS_ST_FAULT		0xf

#define DPC			0x4
#define INTEN			0x20
#define ES			0x24
#define INTSTATUS		0x28
#define INTCLR			0x2c
#define FSM			0x30
#define FSC			0x34
#define FTM			0x38

#define _FTC			0x40
#define FTC(n)			(_FTC + (n)*0x4)

#define _CS			0x100
#define CS(n)			(_CS + (n)*0x8)
#define CS_CNS			(1 << 21)

#define _CPC			0x104
#define CPC(n)			(_CPC + (n)*0x8)

#define _SA			0x400
#define SA(n)			(_SA + (n)*0x20)

#define _DA			0x404
#define DA(n)			(_DA + (n)*0x20)

#define _CC			0x408
#define CC(n)			(_CC + (n)*0x20)

#define CC_SRCINC		(1 << 0)
#define CC_DSTINC		(1 << 14)
#define CC_SRCPRI		(1 << 8)
#define CC_DSTPRI		(1 << 22)
#define CC_SRCNS		(1 << 9)
#define CC_DSTNS		(1 << 23)
#define CC_SRCIA		(1 << 10)
#define CC_DSTIA		(1 << 24)
#define CC_SRCBRSTLEN_SHFT	4
#define CC_DSTBRSTLEN_SHFT	18
#define CC_SRCBRSTSIZE_SHFT	1
#define CC_DSTBRSTSIZE_SHFT	15
#define CC_SRCCCTRL_SHFT	11
#define CC_SRCCCTRL_MASK	0x7
#define CC_DSTCCTRL_SHFT	25
#define CC_DRCCCTRL_MASK	0x7
#define CC_SWAP_SHFT		28

#define _LC0			0x40c
#define LC0(n)			(_LC0 + (n)*0x20)

#define _LC1			0x410
#define LC1(n)			(_LC1 + (n)*0x20)

#define DBGSTATUS		0xd00
#define DBG_BUSY		(1 << 0)

#define DBGCMD			0xd04
#define DBGINST0		0xd08
#define DBGINST1		0xd0c

#define CR0			0xe00
#define CR1			0xe04
#define CR2			0xe08
#define CR3			0xe0c
#define CR4			0xe10
#define CRD			0xe14

#define PERIPH_ID		0xfe0
#define PERIPH_REV_SHIFT	20
#define PERIPH_REV_MASK		0xf
#define PERIPH_REV_R0P0		0
#define PERIPH_REV_R1P0		1
#define PERIPH_REV_R1P1		2

#define CR0_PERIPH_REQ_SET	(1 << 0)
#define CR0_BOOT_EN_SET		(1 << 1)
#define CR0_BOOT_MAN_NS		(1 << 2)
#define CR0_NUM_CHANS_SHIFT	4
#define CR0_NUM_CHANS_MASK	0x7
#define CR0_NUM_PERIPH_SHIFT	12
#define CR0_NUM_PERIPH_MASK	0x1f
#define CR0_NUM_EVENTS_SHIFT	17
#define CR0_NUM_EVENTS_MASK	0x1f

#define CR1_ICACHE_LEN_SHIFT	0
#define CR1_ICACHE_LEN_MASK	0x7
#define CR1_NUM_ICACHELINES_SHIFT	4
#define CR1_NUM_ICACHELINES_MASK	0xf

/* Configuration register value */
#define CRD_DATA_WIDTH_SHIFT	0
#define CRD_DATA_WIDTH_MASK	0x7
#define CRD_WR_CAP_SHIFT	4
#define CRD_WR_CAP_MASK		0x7
#define CRD_WR_Q_DEP_SHIFT	8
#define CRD_WR_Q_DEP_MASK	0xf
#define CRD_RD_CAP_SHIFT	12
#define CRD_RD_CAP_MASK		0x7
#define CRD_RD_Q_DEP_SHIFT	16
#define CRD_RD_Q_DEP_MASK	0xf
#define CRD_DATA_BUFF_SHIFT	20
#define CRD_DATA_BUFF_MASK	0x3ff

/* Microcode opcode value */
#define CMD_DMAADDH		0x54
#define CMD_DMAEND		0x00
#define CMD_DMAFLUSHP		0x35
#define CMD_DMAGO		0xa0
#define CMD_DMALD		0x04
#define CMD_DMALDP		0x25
#define CMD_DMALP		0x20
#define CMD_DMALPEND		0x28
#define CMD_DMAKILL		0x01
#define CMD_DMAMOV		0xbc
#define CMD_DMANOP		0x18
#define CMD_DMARMB		0x12
#define CMD_DMASEV		0x34
#define CMD_DMAST		0x08
#define CMD_DMASTP		0x29
#define CMD_DMASTZ		0x0c
#define CMD_DMAWFE		0x36
#define CMD_DMAWFP		0x30
#define CMD_DMAWMB		0x13

/* the size of opcode plus opcode required settings */
#define SZ_DMAADDH		3
#define SZ_DMAEND		1
#define SZ_DMAFLUSHP		2
#define SZ_DMALD		1
#define SZ_DMALDP		2
#define SZ_DMALP		2
#define SZ_DMALPEND		2
#define SZ_DMAKILL		1
#define SZ_DMAMOV		6
#define SZ_DMANOP		1
#define SZ_DMARMB		1
#define SZ_DMASEV		2
#define SZ_DMAST		1
#define SZ_DMASTP		2
#define SZ_DMASTZ		1
#define SZ_DMAWFE		2
#define SZ_DMAWFP		2
#define SZ_DMAWMB		1
#define SZ_DMAGO		6

/* Use this _only_ to wait on transient states */
#define UNTIL(t, s)		while (!(_state(t) & (s)))
static unsigned cmd_line;

#ifdef PL330_DEBUG_MCGEN
#define PL330_DBGCMD_DUMP(off, x...)	do { \
						printf("%x:", cmd_line); \
						printf(x); \
						cmd_line += off; \
					} while (0)
#define PL330_DBGMC_START(addr)		(cmd_line = addr)
#else
#define PL330_DBGCMD_DUMP(off, x...)	do {} while (0)
#define PL330_DBGMC_START(addr)		do {} while (0)
#endif

/* Enum declaration */
enum dmamov_dst {
	SAR = 0,
	CCR,
	DAR,
};

enum pl330_dst {
	SRC = 0,
	DST,
};

enum pl330_cond {
	SINGLE,
	BURST,
	ALWAYS,
};

/* Structure will be used by _emit_LPEND function */
struct _arg_LPEND {
	enum pl330_cond cond;
	int forever;
	unsigned loop;
	u8 bjump;
};

/* Structure will be used by _emit_GO function */
struct _arg_GO {
	u8 chan;
	u32 addr;
	unsigned ns;
};

/*
 * Function:	add opcode DMAEND into microcode (end)
 * Return:	size of opcode
 * Parameter:	buf -> the buffer which stored the microcode program
 */
static inline u32 _emit_END(u8 buf[])
{
	buf[0] = CMD_DMAEND;

	PL330_DBGCMD_DUMP(SZ_DMAEND, "\tDMAEND\n");

	return SZ_DMAEND;
}

static inline u32 _emit_FLUSHP(u8 buf[], u8 peri)
{
	buf[0] = CMD_DMAFLUSHP;

	peri &= 0x1f;
	peri <<= 3;
	buf[1] = peri;

	PL330_DBGCMD_DUMP(SZ_DMAFLUSHP, "\tDMAFLUSHP %u\n", peri >> 3);

	return SZ_DMAFLUSHP;
}

static inline u32 _emit_LD(u8 buf[],	enum pl330_cond cond)
{
	buf[0] = CMD_DMALD;

	if (cond == SINGLE)
		buf[0] |= (0 << 1) | (1 << 0);
	else if (cond == BURST)
		buf[0] |= (1 << 1) | (1 << 0);

	PL330_DBGCMD_DUMP(SZ_DMALD, "\tDMALD%c\n",
		cond == SINGLE ? 'S' : (cond == BURST ? 'B' : 'A'));

	return SZ_DMALD;
}

static inline u32 _emit_LDP(u8 buf[], enum pl330_cond cond, u8 peri)
{
	buf[0] = CMD_DMALDP;

	if (cond == BURST)
		buf[0] |= (1 << 1);

	peri &= 0x1f;
	peri <<= 3;
	buf[1] = peri;

	PL330_DBGCMD_DUMP(SZ_DMALDP, "\tDMALDP%c %u\n",
		cond == SINGLE ? 'S' : 'B', peri >> 3);

	return SZ_DMALDP;
}

static inline u32 _emit_LP(u8 buf[], unsigned loop, u8 cnt)
{
	buf[0] = CMD_DMALP;

	if (loop)
		buf[0] |= (1 << 1);

	cnt--; /* DMAC increments by 1 internally */
	buf[1] = cnt;

	PL330_DBGCMD_DUMP(SZ_DMALP, "\tDMALP_%c %u\n", loop ? '1' : '0', cnt);

	return SZ_DMALP;
}

static inline u32 _emit_LPEND(u8 buf[], const struct _arg_LPEND *arg)
{
	enum pl330_cond cond = arg->cond;
	bool forever = arg->forever;
	unsigned loop = arg->loop;
	u8 bjump = arg->bjump;

	buf[0] = CMD_DMALPEND;

	if (loop)
		buf[0] |= (1 << 2);

	if (!forever)
		buf[0] |= (1 << 4);

	if (cond == SINGLE)
		buf[0] |= (0 << 1) | (1 << 0);
	else if (cond == BURST)
		buf[0] |= (1 << 1) | (1 << 0);

	buf[1] = bjump;

	PL330_DBGCMD_DUMP(SZ_DMALPEND, "\tDMALP%s%c_%c bjmpto_%x\n",
			forever ? "FE" : "END",
			cond == SINGLE ? 'S' : (cond == BURST ? 'B' : 'A'),
			loop ? '1' : '0',
			bjump);

	return SZ_DMALPEND;
}

static inline u32 _emit_KILL(u8 buf[])
{
	buf[0] = CMD_DMAKILL;

	return SZ_DMAKILL;
}

static inline u32 _emit_MOV(u8 buf[], enum dmamov_dst dst, u32 val)
{
	buf[0] = CMD_DMAMOV;
	buf[1] = dst;

	buf[2] = val & 0xFF;
	buf[3] = (val >> 8) & 0xFF;
	buf[4] = (val >> 16) & 0xFF;
	buf[5] = (val >> 24) & 0xFF;

	PL330_DBGCMD_DUMP(SZ_DMAMOV, "\tDMAMOV %s 0x%x\n",
		dst == SAR ? "SAR" : (dst == DAR ? "DAR" : "CCR"), val);

	return SZ_DMAMOV;
}

static inline u32 _emit_NOP(u8 buf[])
{
	buf[0] = CMD_DMANOP;

	PL330_DBGCMD_DUMP(SZ_DMANOP, "\tDMANOP\n");

	return SZ_DMANOP;
}

static inline u32 _emit_RMB(u8 buf[])
{
	buf[0] = CMD_DMARMB;

	PL330_DBGCMD_DUMP(SZ_DMARMB, "\tDMARMB\n");

	return SZ_DMARMB;
}

static inline u32 _emit_SEV(u8 buf[], u8 ev)
{
	buf[0] = CMD_DMASEV;

	ev &= 0x1f;
	ev <<= 3;
	buf[1] = ev;

	PL330_DBGCMD_DUMP(SZ_DMASEV, "\tDMASEV %u\n", ev >> 3);

	return SZ_DMASEV;
}

static inline u32 _emit_ST(u8 buf[], enum pl330_cond cond)
{
	buf[0] = CMD_DMAST;

	if (cond == SINGLE)
		buf[0] |= (0 << 1) | (1 << 0);
	else if (cond == BURST)
		buf[0] |= (1 << 1) | (1 << 0);

	PL330_DBGCMD_DUMP(SZ_DMAST, "\tDMAST%c\n",
		cond == SINGLE ? 'S' : (cond == BURST ? 'B' : 'A'));

	return SZ_DMAST;
}

static inline u32 _emit_STP(u8 buf[], enum pl330_cond cond, u8 peri)
{
	buf[0] = CMD_DMASTP;

	if (cond == BURST)
		buf[0] |= (1 << 1);

	peri &= 0x1f;
	peri <<= 3;
	buf[1] = peri;

	PL330_DBGCMD_DUMP(SZ_DMASTP, "\tDMASTP%c %u\n",
		cond == SINGLE ? 'S' : 'B', peri >> 3);

	return SZ_DMASTP;
}

static inline u32 _emit_STZ(u8 buf[])
{
	buf[0] = CMD_DMASTZ;

	PL330_DBGCMD_DUMP(SZ_DMASTZ, "\tDMASTZ\n");

	return SZ_DMASTZ;
}

static inline u32 _emit_WFE(u8 buf[], u8 ev, unsigned invalidate)
{
	buf[0] = CMD_DMAWFE;

	ev &= 0x1f;
	ev <<= 3;
	buf[1] = ev;

	if (invalidate)
		buf[1] |= (1 << 1);

	PL330_DBGCMD_DUMP(SZ_DMAWFE, "\tDMAWFE %u%s\n",
		ev >> 3, invalidate ? ", I" : "");

	return SZ_DMAWFE;
}

static inline u32 _emit_WFP(u8 buf[], enum pl330_cond cond, u8 peri)
{
	buf[0] = CMD_DMAWFP;

	if (cond == SINGLE)
		buf[0] |= (0 << 1) | (0 << 0);
	else if (cond == BURST)
		buf[0] |= (1 << 1) | (0 << 0);
	else
		buf[0] |= (0 << 1) | (1 << 0);

	peri &= 0x1f;
	peri <<= 3;
	buf[1] = peri;

	PL330_DBGCMD_DUMP(SZ_DMAWFP, "\tDMAWFP%c %u\n",
		cond == SINGLE ? 'S' : (cond == BURST ? 'B' : 'P'), peri >> 3);

	return SZ_DMAWFP;
}

static inline u32 _emit_WMB(u8 buf[])
{
	buf[0] = CMD_DMAWMB;

	PL330_DBGCMD_DUMP(SZ_DMAWMB, "\tDMAWMB\n");

	return SZ_DMAWMB;
}

static inline u32 _emit_GO(u8 buf[],
		const struct _arg_GO *arg)
{
	u8 chan = arg->chan;
	u32 addr = arg->addr;
	unsigned ns = arg->ns;

	buf[0] = CMD_DMAGO;
	buf[0] |= (ns << 1);

	buf[1] = chan & 0x7;
	buf[2] = addr & 0xFF;
	buf[3] = (addr >> 8) & 0xFF;
	buf[4] = (addr >> 16) & 0xFF;
	buf[5] = (addr >> 24) & 0xFF;
	return SZ_DMAGO;
}

/*
 * Function:	Populate the CCR register
 * Parameter:	rqc -> Request Configuration.
 */
static inline u32 _prepare_ccr(const struct pl330_reqcfg *rqc)
{
	u32 ccr = 0;

	if (rqc->src_inc)
		ccr |= CC_SRCINC;
	if (rqc->dst_inc)
		ccr |= CC_DSTINC;

	/* We set same protection levels for Src and DST for now */
	if (rqc->privileged)
		ccr |= CC_SRCPRI | CC_DSTPRI;
	if (rqc->nonsecure)
		ccr |= CC_SRCNS | CC_DSTNS;
	if (rqc->insnaccess)
		ccr |= CC_SRCIA | CC_DSTIA;

	ccr |= (((rqc->brst_len - 1) & 0xf) << CC_SRCBRSTLEN_SHFT);
	ccr |= (((rqc->brst_len - 1) & 0xf) << CC_DSTBRSTLEN_SHFT);

	ccr |= (rqc->brst_size << CC_SRCBRSTSIZE_SHFT);
	ccr |= (rqc->brst_size << CC_DSTBRSTSIZE_SHFT);

	ccr |= (rqc->scctl << CC_SRCCCTRL_SHFT);
	ccr |= (rqc->dcctl << CC_DSTCCTRL_SHFT);

	ccr |= (rqc->swap << CC_SWAP_SHFT);
	return ccr;
}

/*
 * Function:	wait until DMA Manager is idle
 * Return:	1 = error / timeout ocurred before idle
 * Parameter:	loop -> number of loop before timeout ocurred
 */
static int _until_dmac_idle(struct pl330_transfer_struct *pl330, int loops)
{
	u32 regs = pl330->reg_base;

	do {
		/* Until Manager is Idle */
		if (!(readl(regs + DBGSTATUS) & DBG_BUSY))
			break;
	} while (--loops);

	if (!loops)
		return true;

	return false;
}

static inline void _execute_DBGINSN(struct pl330_transfer_struct *pl330,
				    u8 insn[], bool as_manager, int timeout_loops)
{
	u32 regs = pl330->reg_base;
	u32 val;

	val = (insn[0] << 16) | (insn[1] << 24);
	if (!as_manager)
		val |= (1 << 0);
	val |= (pl330->channel_num << 8); /* Channel Number */
	writel(val, regs + DBGINST0);
	val = insn[2];
	val = val | (insn[3] << 8);
	val = val | (insn[4] << 16);
	val = val | (insn[5] << 24);
	writel(val, regs + DBGINST1);

	/* If timed out due to halted state-machine */
	if (_until_dmac_idle(pl330, timeout_loops)) {
		printf("DMAC halted!\n");
		return;
	}

	/* Get going */
	writel(0, regs + DBGCMD);
}

static inline u32 _state(struct pl330_transfer_struct *pl330)
{
	u32 regs = pl330->reg_base;
	u32 val;

	val = readl(regs + CS(pl330->channel_num)) & 0xf;

	udelay(1);

	switch (val) {
	case DS_ST_STOP:
		return PL330_STATE_STOPPED;
	case DS_ST_EXEC:
		return PL330_STATE_EXECUTING;
	case DS_ST_CMISS:
		return PL330_STATE_CACHEMISS;
	case DS_ST_UPDTPC:
		return PL330_STATE_UPDTPC;
	case DS_ST_WFE:
		return PL330_STATE_WFE;
	case DS_ST_FAULT:
		return PL330_STATE_FAULTING;
	case DS_ST_ATBRR:
		return PL330_STATE_ATBARRIER;
	case DS_ST_QBUSY:
		return PL330_STATE_QUEUEBUSY;
	case DS_ST_WFP:
		return PL330_STATE_WFP;
	case DS_ST_KILL:
		return PL330_STATE_KILLING;
	case DS_ST_CMPLT:
		return PL330_STATE_COMPLETING;
	case DS_ST_FLTCMP:
		return PL330_STATE_FAULT_COMPLETING;
	default:
		return PL330_STATE_INVALID;
	}
}

static void _stop(struct pl330_transfer_struct *pl330, int timeout_loops)
{
	u8 insn[6] = {0, 0, 0, 0, 0, 0};

	if (_state(pl330) == PL330_STATE_FAULT_COMPLETING)
		UNTIL(pl330, PL330_STATE_FAULTING | PL330_STATE_KILLING);

	/* Return if nothing needs to be done */
	if (_state(pl330) == PL330_STATE_COMPLETING
		  || _state(pl330) == PL330_STATE_KILLING
		  || _state(pl330) == PL330_STATE_STOPPED)
		return;

	_emit_KILL(insn);

	_execute_DBGINSN(pl330, insn, 0, timeout_loops);
}

static bool _trigger(struct pl330_transfer_struct *pl330, u8 *buffer,
		     int timeout_loops)
{
	struct _arg_GO go;
	u8 insn[6] = {0, 0, 0, 0, 0, 0};

	/* Return if already ACTIVE */

	go.chan = pl330->channel_num;
	go.addr = (u32)buffer;

	/* TODO: determine security. Assume secure */
	go.ns = 0;
	_emit_GO(insn, &go);

	/* Only manager can execute GO */
	_execute_DBGINSN(pl330, insn, true, timeout_loops);

	return false;
}

static bool _start(struct pl330_transfer_struct *pl330, int timeout_loops)
{
	switch (_state(pl330)) {
	case PL330_STATE_FAULT_COMPLETING:
		UNTIL(pl330, PL330_STATE_FAULTING | PL330_STATE_KILLING);

		if (_state(pl330) == PL330_STATE_KILLING)
			UNTIL(pl330, PL330_STATE_STOPPED)

	case PL330_STATE_FAULTING:
		_stop(pl330, timeout_loops);

	case PL330_STATE_KILLING:
	case PL330_STATE_COMPLETING:
		UNTIL(pl330, PL330_STATE_STOPPED)

	case PL330_STATE_STOPPED:
		return _trigger(pl330, pl330->buf, timeout_loops);

	case PL330_STATE_WFP:
	case PL330_STATE_QUEUEBUSY:
	case PL330_STATE_ATBARRIER:
	case PL330_STATE_UPDTPC:
	case PL330_STATE_CACHEMISS:
	case PL330_STATE_EXECUTING:
	case PL330_STATE_WFE: /* For RESUME, nothing yet */
	default:
		return false;
	}
}

/******************************************************************************
DMA run or start
Return:		1 for error or not successful

channel_num	-	channel number assigned, valid from 0 to 7
buf		-	buffer handler which will point to the memory
			allocated for dma microcode
******************************************************************************/
static int pl330_transfer_start(struct pl330_transfer_struct *pl330)
{
	/* Timeout loop */
	int timeout_loops = 10000;

	/* Execute the command list */
	return _start(pl330, timeout_loops);
}

/******************************************************************************
DMA poll until finish or error
Return:		1 for error or not successful

channel_num	-	channel number assigned, valid from 0 to 7
******************************************************************************/
static int pl330_transfer_finish(struct pl330_transfer_struct *pl330)
{
	/* Wait until finish execution to ensure we compared correct result*/
	UNTIL(pl330, PL330_STATE_STOPPED | PL330_STATE_FAULTING);

	/* check the state */
	if (_state(pl330) == PL330_STATE_FAULTING) {
		printf("FAULT Mode: Channel %u Faulting, FTR = 0x%08x, "
			"CPC = 0x%08x\n", pl330->channel_num,
			readl(pl330->reg_base + FTC(pl330->channel_num)),
			((u32)readl(pl330->reg_base + CPC(pl330->channel_num))
				- (u32)pl330->buf));
		return 1;
	}
	return 0;
}

/******************************************************************************
DMA transfer setup (DMA_SUPPORTS_MEM_TO_MEM, DMA_SUPPORTS_MEM_TO_DEV or
		    DMA_SUPPORTS_DEV_TO_MEM)
For Peripheral transfer, the FIFO threshold value is expected at
2 ^ pl330->brst_size * pl330->brst_len.
Return:		1 for error or not successful

channel_num	-	channel number assigned, valid from 0 to 7
src_addr	-	address to transfer from / source
dst_addr	-	address to transfer to / destination
size_byte	-	number of bytes to be transferred
brst_size	-	valid from 0 - 3
			where 0 = 1 (2 ^ 0) bytes and 3 = 8 bytes (2 ^ 3)
single_brst_size -	single transfer size (from 0 - 3)
brst_len	-	valid from 1 - 16 where each burst can trasfer 1 - 16
			data chunk (each chunk size equivalent to brst_size)
peripheral_id	-	assigned peripheral_id, valid from 0 to 31
transfer_type	-	DMA_SUPPORTS_MEM_TO_MEM, DMA_SUPPORTS_MEM_TO_DEV or
			DMA_SUPPORTS_DEV_TO_MEM
enable_cache1	-	1 for cache enabled for memory
			(cacheable and bufferable, but do not allocate)
buf_size	-	sizeof(buf)
buf		-	buffer handler which will point to the memory
			allocated for dma microcode
******************************************************************************/
static int pl330_transfer_setup(struct pl330_transfer_struct *pl330)
{
	/* Variable declaration */
	int off = 0;			/* buffer offset clear to 0 */
	int ret = 0;
	unsigned loopjmp0, loopjmp1;	/* for DMALPEND */
	unsigned lcnt0 = 0;		/* loop count 0 */
	unsigned lcnt1 = 0;		/* loop count 1 */
	unsigned burst_size = 0;
	unsigned data_size_byte = pl330->size_byte;
	u32 ccr = 0;			/* Channel Control Register */
	struct pl330_reqcfg reqcfg;
	cmd_line = 0;

#ifdef PL330_DEBUG_MCGEN
	if (pl330->transfer_type == DMA_SUPPORTS_MEM_TO_DEV)
		puts("INFO: mem2perip");
	else if (pl330->transfer_type == DMA_SUPPORTS_DEV_TO_MEM)
		puts("INFO: perip2mem");
	else
		puts("INFO: mem2mem");

	printf(" - 0x%08lx -> 0x%08lx\nsize=%08x brst_size=2^%li "
		"brst_len=%li singles_brst_size=2^%li\n",
		pl330->src_addr, pl330->dst_addr, data_size_byte,
		pl330->brst_size, pl330->brst_len,
		pl330->single_brst_size);
#endif

	/* for burst, always use the maximum burst size and length */
	pl330->brst_size = PL330_DMA_MAX_BURST_SIZE;
	pl330->brst_len = 16;
	pl330->single_brst_size = 1;

	/* burst_size = 2 ^ brst_size */
	burst_size = 1 << pl330->brst_size;

	pl330->src_addr	= &pl330->buf;
	if (pl330->dst_addr & (burst_size - 1)) {
		puts("ERROR PL330 : destination address unaligned\n");
		return 1;
	}

	/* DMAMOV DAR, x->dst_addr */
	off += _emit_MOV(&pl330->buf[off], DAR, pl330->dst_addr);
	/* DMAFLUSHP P(periheral_id) */
	if (pl330->transfer_type != DMA_SUPPORTS_MEM_TO_MEM)
		off += _emit_FLUSHP(&pl330->buf[off], pl330->peripheral_id);

	/* Preparing the CCR value */
	if (pl330->transfer_type == DMA_SUPPORTS_MEM_TO_DEV) {
		reqcfg.dst_inc = 0;	/* disable auto increment */
		reqcfg.src_inc = 1;	/* enable auto increment */
	} else if (pl330->transfer_type == DMA_SUPPORTS_DEV_TO_MEM) {
		reqcfg.dst_inc = 1;
		reqcfg.src_inc = 0;
	} else {
		/* DMA_SUPPORTS_MEM_TO_MEM */
		reqcfg.dst_inc = 1;
		reqcfg.src_inc = 1;
	}

	reqcfg.nonsecure = 0;	/* Secure mode */
	reqcfg.dcctl = 0x1;	/* noncacheable but bufferable */
	reqcfg.scctl = 0x1;
	reqcfg.privileged = 1;		/* 1 - Priviledge  */
	reqcfg.insnaccess = 0;		/* 0 - data access */
	reqcfg.swap = 0;		/* 0 - no endian swap */
	reqcfg.brst_len = pl330->brst_len;	/* DMA burst length */
	reqcfg.brst_size = pl330->brst_size;	/* DMA burst size */
	/* Preparing the CCR value */
	ccr = _prepare_ccr(&reqcfg);
	/* DMAMOV CCR, ccr */
	off += _emit_MOV(&pl330->buf[off], CCR, ccr);

	/* BURST */
	/* Can initiate a burst? */
	while (data_size_byte >= burst_size * pl330->brst_len) {
		lcnt0 = data_size_byte / (burst_size * pl330->brst_len);
		lcnt1 = 0;
		if (lcnt0 >= 256 * 256)
			lcnt0 = lcnt1 = 256;
		else if (lcnt0 >= 256) {
			lcnt1 = lcnt0 / 256;
			lcnt0 = 256;
		}
		data_size_byte = data_size_byte -
			(burst_size * pl330->brst_len * lcnt0 * lcnt1);

		if (lcnt1) {
			/* DMALP1 */
			off += _emit_LP(&pl330->buf[off], 1, lcnt1);
			loopjmp1 = off;
		}
		/* DMALP0 */
		off += _emit_LP(&pl330->buf[off], 0, lcnt0);
		loopjmp0 = off;

		off += _emit_STZ(&pl330->buf[off]);
		/* DMALP0END */
		struct _arg_LPEND lpend;
		lpend.cond = ALWAYS;
		lpend.forever = 0;
		lpend.loop = 0;		/* loop cnt 0 */
		lpend.bjump = off - loopjmp0;
		off += _emit_LPEND(&pl330->buf[off], &lpend);
		/* DMALP1END */
		if (lcnt1) {
			struct _arg_LPEND lpend;
			lpend.cond = ALWAYS;
			lpend.forever = 0;
			lpend.loop = 1;		/* loop cnt 1*/
			lpend.bjump = off - loopjmp1;
			off += _emit_LPEND(&pl330->buf[off], &lpend);
		}
		/* ensure the microcode don't exceed buffer size */
		if (off > pl330->buf_size) {
			puts("ERROR PL330 : Exceed buffer size\n");
			return 1;
		}
	}

	/* SINGLE */
	pl330->brst_size = pl330->single_brst_size;
	pl330->brst_len = 1;
	/* burst_size = 2 ^ brst_size */
	burst_size = (1 << pl330->brst_size);
	lcnt0 = data_size_byte / (burst_size * pl330->brst_len);

	/* ensure all data will be transfered */
	data_size_byte = data_size_byte -
		(burst_size * pl330->brst_len * lcnt0);
	if (data_size_byte)
		puts("ERROR PL330 : Detected the possibility of untransfered"
			"data. Please ensure correct single burst size\n");

	if (lcnt0) {
		/* Preparing the CCR value */
		reqcfg.brst_len = pl330->brst_len;	/* DMA burst length */
		reqcfg.brst_size = pl330->brst_size;	/* DMA burst size */
		ccr = _prepare_ccr(&reqcfg);
		/* DMAMOV CCR, ccr */
		off += _emit_MOV(&pl330->buf[off], CCR, ccr);

		/* DMALP0 */
		off += _emit_LP(&pl330->buf[off], 0, lcnt0);
		loopjmp0 = off;

		off += _emit_STZ(&pl330->buf[off]);
		struct _arg_LPEND lpend1;
		lpend1.cond = ALWAYS;
		lpend1.forever = 0;
		lpend1.loop = 0;	/* loop cnt 0 */
		lpend1.bjump = off - loopjmp0;
		off += _emit_LPEND(&pl330->buf[off], &lpend1);
		/* ensure the microcode don't exceed buffer size */
		if (off > pl330->buf_size) {
			puts("ERROR PL330 : Exceed buffer size\n");
			return 1;
		}
	}

	/* DMAEND */
	off += _emit_END(&pl330->buf[off]);

	ret = pl330_transfer_start(pl330);
	if (ret)
		return ret;

	ret = pl330_transfer_finish(pl330);
	if (ret)
		return ret;

	return 0;
}

#ifndef CONFIG_DMA
void arm_pl330_transfer(struct pl330_transfer_struct *pl330)
{
	pl330_transfer_setup(pl330);
}

#else
static int pl330_transfer(struct udevice *dev, int direction, void *dst,
			  void *src, size_t len)
{
	int ret = 0;
	struct dma_pl330_platdata *priv = dev_get_priv(dev);
	struct pl330_transfer_struct *pl330;

	/* Allocate a new DMAC and its Channels */
	pl330 = devm_kzalloc(dev, sizeof(*pl330), GFP_KERNEL);
	if (!pl330)
		return -ENOMEM;

	pl330->reg_base = priv->base;

	pl330->dst_addr = (unsigned int) (dst);
	pl330->src_addr = (unsigned int) (src);
	pl330->size_byte = len;

	/* channel 1 */
	pl330->channel_num = 1;

	switch(direction) {
	case DMA_MEM_TO_MEM:
		pl330->transfer_type = DMA_SUPPORTS_MEM_TO_MEM;
		break;
	case DMA_MEM_TO_DEV:
		pl330->transfer_type = DMA_SUPPORTS_MEM_TO_DEV;
		break;
	case DMA_DEV_TO_MEM:
		pl330->transfer_type = DMA_SUPPORTS_DEV_TO_MEM;
		break;
	}

	ret = pl330_transfer_setup(pl330);

	return ret;
}

static int pl330_ofdata_to_platdata(struct udevice *dev)
{
	struct dma_pl330_platdata *priv = dev_get_priv(dev);

	priv->base = dev_get_addr(dev);

	return 0;
}

static int pl330_probe(struct udevice *adev)
{
	struct dma_dev_priv *uc_priv = dev_get_uclass_priv(adev);

	uc_priv->supported = (DMA_SUPPORTS_MEM_TO_MEM |
			      DMA_SUPPORTS_MEM_TO_DEV |
			      DMA_SUPPORTS_DEV_TO_MEM);
	return 0;
}

static const struct dma_ops pl330_ops = {
        .transfer	= pl330_transfer,
};

static const struct udevice_id pl330_ids[] = {
	{ .compatible = "arm,pl330" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(dma_pl330) = {
	.name	= "dma_pl330",
	.id 	= UCLASS_DMA,
	.of_match = pl330_ids,
	.ops	= &pl330_ops,
	.ofdata_to_platdata = pl330_ofdata_to_platdata,
	.probe = pl330_probe,
	.priv_auto_alloc_size = sizeof(struct dma_pl330_platdata),
};
#endif /* CONFIG_DMA */
