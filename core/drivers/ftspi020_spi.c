// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (c) 2016, Linaro Limited
 *
 */

#include <assert.h>
#include <initcall.h>
#include <io.h>
#include <keep.h>
#include <kernel/delay.h>
#include <kernel/panic.h>
#include <kernel/tee_time.h>
#include <platform_config.h>
#include <trace.h>
#include <util.h>

#include <drivers/ftspi020_spi.h>

/******************************************************************************
 * FTSPI020 Registers
 *****************************************************************************/
#define FTSPI020_REG_CMD0                   0x00	/* Flash address */
#define FTSPI020_REG_CMD1                   0x04
#define FTSPI020_REG_CMD2                   0x08
#define FTSPI020_REG_CMD3                   0x0c
#define FTSPI020_REG_CTRL                   0x10	/* Control */
#define FTSPI020_REG_AC_TIME                0x14
#define FTSPI020_REG_STS                    0x18	/* Status */
#define FTSPI020_REG_ICR                    0x20	/* Interrupt Enable */
#define FTSPI020_REG_ISR                    0x24	/* Interrupt Status */
#define FTSPI020_REG_READ_STS               0x28
#define FTSPI020_REG_REVISION               0x50
#define FTSPI020_REG_FEATURE                0x54
#define FTSPI020_REG_DATA_PORT              0x100

/*
 * Control Register offset 0x10
 */
#define FTSPI020_CTRL_READY_LOC_MASK        (~(0x7 << 16))
#define FTSPI020_CTRL_READY_LOC(x)          (((x) & 0x7) << 16)

#define FTSPI020_CTRL_DAMR_PORT             (1 << 20)

#define FTSPI020_CTRL_ABORT                 (1 << 8)

#define FTSPI020_CTRL_CLK_MODE_MASK         (~(0x1 << 4))
#define FTSPI020_CTRL_CLK_MODE_0            (0 << 4)
#define FTSPI020_CTRL_CLK_MODE_3            (1 << 4)

#define FTSPI020_CTRL_CLK_DIVIDER_MASK      (~(0x3 << 0))
#define FTSPI020_CTRL_CLK_DIVIDER_2         (0 << 0)
#define FTSPI020_CTRL_CLK_DIVIDER_4         (1 << 0)
#define FTSPI020_CTRL_CLK_DIVIDER_6         (2 << 0)
#define FTSPI020_CTRL_CLK_DIVIDER_8         (3 << 0)

/*
 * Status Register offset 0x18
 */
#define FTSPI020_STS_RFR                    (1 << 1)	/* RX FIFO ready */
#define FTSPI020_STS_TFR                    (1 << 0)	/* TX FIFO ready */

/*
 * Interrupt Control Register
 */
#ifdef CFG_FTSPI020_V1_7_0
#define FTSPI020_ICR_DMA_RX_THOD(x)         (((x) >>  20) & 0x7)	/* DMA RX FIFO threshold */
#define FTSPI020_ICR_DMA_TX_THOD(x)         (((x) >>  16) & 0x7)	/* DMA TX FIFO threshold */
#define FTSPI020_ICR_RX_XFR_SIZE(x)         (((x) >>  12) & 0x3)	/* RX Transfer size */
#define FTSPI020_ICR_TX_XFR_SIZE(x)         (((x) >>   8) & 0x3)	/* TX Transfer size */
#else
#define FTSPI020_ICR_DMA_RX_THOD(x)         (((x) >>  12) & 0x3)	/* DMA RX FIFO threshold */
#define FTSPI020_ICR_DMA_TX_THOD(x)         (((x) >>   8) & 0x3)	/* DMA TX FIFO threshold */
#endif

/*
 * Interrupt Status Register
 */
#define FTSPI020_ISR_CMD_CMPL	(1 << 0) /* Command complete status */

/*
 * Feature Register
 */
#define FTSPI020_FEATURE_RXFIFO_DEPTH(x)    (((x) >>  8) & 0xff)
#define FTSPI020_FEATURE_TXFIFO_DEPTH(x)    (((x) >>  0) & 0xff)

/*
 * CMD1 Register offset 4: Command queue Second Word
 */
#define FTSPI020_CMD1_CONT_READ_MODE_EN     (1 << 28)
#define FTSPI020_CMD1_CONT_READ_MODE_DIS    (0 << 28)

#define FTSPI020_CMD1_OP_CODE_0_BYTE        (0 << 24)
#define FTSPI020_CMD1_OP_CODE_1_BYTE        (1 << 24)
#define FTSPI020_CMD1_OP_CODE_2_BYTE        (2 << 24)

#define FTSPI020_CMD1_DUMMY_CYCLE(x)        (((x) & 0xff) << 16)

#define FTSPI020_CMD1_ADDR_BYTES(x)         ((x & 0x7) << 0)

/*
 * CMD3 Register offset 0xc: Command queue Fourth Word
 */
#define FTSPI020_CMD3_INSTR_CODE(x)         (((x) & 0xff) << 24)
#define FTSPI020_CMD3_CONT_READ_CODE(x)     (((x) & 0xff) << 16)
#define FTSPI020_CMD3_CE(x)                 (((x) & 0x3) << 8)
#define FTSPI020_CMD3_SERIAL_MODE           (0 << 5)
#define FTSPI020_CMD3_DUAL_MODE             (1 << 5)
#define FTSPI020_CMD3_QUAD_MODE             (2 << 5)
#define FTSPI020_CMD3_DUAL_IO_MODE          (3 << 5)
#define FTSPI020_CMD3_QUAD_IO_MODE          (4 << 5)
#define FTSPI020_CMD3_DTR_MODE_EN           (1 << 4)
#define FTSPI020_CMD3_DTR_MODE_DIS          (0 << 4)
#define FTSPI020_CMD3_STS_SW_READ           (1 << 3)
#define FTSPI020_CMD3_STS_HW_READ           (0 << 3)
#define FTSPI020_CMD3_RD_STS_EN             (1 << 2)
#define FTSPI020_CMD3_RD_STS_DIS            (0 << 2)
#define FTSPI020_CMD3_WRITE                 (1 << 1)
#define FTSPI020_CMD3_READ                  (0 << 1)
//A380 must NOT define SPI_FTSPI020_NOR_V1_1_0
#ifndef CFG_FTSPI020_V1_1_0
#define FTSPI020_CMD3_INTR_EN               (1 << 0)
#else
#define FTSPI020_CMD3_INTR_EN               (0 << 0)
#endif

/* Flash opcodes */
#define SPINOR_OP_READ                      0x03	/* Read data bytes (low frequency) */
#define SPINOR_OP_READ_FAST                 0x0b	/* Read data bytes (high frequency) */
#define SPINOR_OP_PP                        0x02	/* Page program (up to 256 bytes) */
#define SPINOR_OP_SE                        0xd8	/* Sector erase (usually 64KiB) */

#define min(a, b)	(a) < (b) ? a : b

static void ftspi020_write_word(vaddr_t base, void *data, int wsize)
{
	if (data) {
		switch (wsize) {
		case 1:
			io_write8(base + FTSPI020_REG_DATA_PORT, *(uint8_t *)data);
			break;

		case 2:
			io_write16(base + FTSPI020_REG_DATA_PORT, *(uint16_t *)data);
			break;

		default:
			io_write32(base + FTSPI020_REG_DATA_PORT, *(uint32_t *)data);
			break;
		}
	}
}

static void ftspi020_read_word(vaddr_t base, void *buf, int wsize)
{
	if (buf) {
		switch (wsize) {
		case 1:
			*(uint8_t *)buf = io_read8(base + FTSPI020_REG_DATA_PORT);
			break;

		case 2:
			*(uint16_t *)buf = io_read16(base + FTSPI020_REG_DATA_PORT);
			break;

		default:
			*(uint32_t *)buf = io_read32(base + FTSPI020_REG_DATA_PORT);
			break;
		}
	}
}

static void ftspi020_configure(struct spi_chip *chip)
{
	struct ftspi020_data *hw = container_of(chip, struct ftspi020_data, chip);
	unsigned int val;

	/* Set SPI mode */
	val = io_read32(hw->base + FTSPI020_REG_CTRL);
	val &= FTSPI020_CTRL_CLK_MODE_MASK;
	switch (hw->mode) {
	case SPI_MODE0:
		DMSG("SPI mode 0");
		val |= FTSPI020_CTRL_CLK_MODE_0;
		break;
	case SPI_MODE3:
		DMSG("SPI mode 3");
		val |= FTSPI020_CTRL_CLK_MODE_3;
		break;
	default:
		EMSG("Invalid SPI mode: %u", hw->mode);
		panic();
	}
	io_write32(hw->base + FTSPI020_REG_CTRL, val);

	/* Set SPI clock divider and DAMR port selection to Command based */
	val = io_read32(hw->base + FTSPI020_REG_CTRL);
	val &= ~FTSPI020_CTRL_DAMR_PORT;
	val &= FTSPI020_CTRL_CLK_DIVIDER_MASK;
	val |= FTSPI020_CTRL_CLK_DIVIDER_8;
	io_write32(hw->base + FTSPI020_REG_CTRL, val);

	while ((io_read32(hw->base + FTSPI020_REG_CTRL) &
		FTSPI020_CTRL_ABORT)) {;}

	/* busy bit location */
	val = io_read32(hw->base + FTSPI020_REG_CTRL);
	val &= FTSPI020_CTRL_READY_LOC_MASK;
	val |= FTSPI020_CTRL_READY_LOC(0);
	io_write32(hw->base + FTSPI020_REG_CTRL, val);
}

static enum spi_result ftspi020_txrx(struct spi_chip *chip, const uint8_t *wdat,
		uint8_t *rdat, size_t num_pkts, uint32_t wsize)
{
	void (*access_fifo)(vaddr_t base, void *buf, int wsize);
	struct ftspi020_data *hw = container_of(chip, struct ftspi020_data, chip);
	uint32_t transfer_size, timeout;
	uint8_t fifo_ready_mask;
	uint8_t *buf;
	int ret = SPI_OK;

	if (wdat) {
		buf = (uint8_t *)wdat;
		transfer_size = hw->tx_transfer_size;
		fifo_ready_mask = FTSPI020_STS_TFR;
		access_fifo = ftspi020_write_word;
	} else {
		buf = rdat;
		transfer_size = hw->rx_transfer_size;
		fifo_ready_mask = FTSPI020_STS_RFR;
		access_fifo = ftspi020_read_word;
	}

	while (num_pkts > 0) {
		int access_byte;

		access_byte = min(num_pkts, transfer_size);

		/* Wait for txfifo or rxfifo ready */
		timeout = timeout_init_us(1000);
		while (!(io_read32(hw->base + FTSPI020_REG_STS) & fifo_ready_mask)) {
			if (timeout_elapsed(timeout)) {
				EMSG("wait fifo timeout(num_pkts:%ld sts:%#x(%#x))\n",
				     num_pkts, io_read32(hw->base + FTSPI020_REG_STS), fifo_ready_mask);
				ret = SPI_ERR_PKTCNT;
				goto out;
			}
		}

		num_pkts -= access_byte;
		while (access_byte) {
			access_fifo(hw->base, buf, wsize);
			buf += wsize;
			access_byte -= wsize;
		}
	}

out:
	return ret;
}

static enum spi_result ftspi020_txrx8(struct spi_chip *chip, uint8_t *wdat,
		uint8_t *rdat, size_t num_pkts)
{
	return ftspi020_txrx(chip, wdat, rdat, num_pkts, 1);
}

static enum spi_result ftspi020_txrx16(struct spi_chip *chip, uint16_t *wdat,
		uint16_t *rdat, size_t num_pkts)
{
	return ftspi020_txrx(chip, (uint8_t *)wdat, (uint8_t *)rdat, num_pkts, 2);
}

static const struct spi_ops ftspi020_ops = {
	.configure  = ftspi020_configure,
	.txrx8      = ftspi020_txrx8,
	.txrx16     = ftspi020_txrx16,
};
KEEP_PAGER(ftspi020_ops);

static int ftspi020_wait_complete(struct ftspi020_data *hw, signed long timeout)
{
	unsigned long expire;
	uint32_t isr;

	expire = timeout_init_us(timeout);

	do {
		isr = io_read32(hw->base + FTSPI020_REG_ISR);
		io_write32(hw->base + FTSPI020_REG_ISR, isr);

		if (isr & FTSPI020_ISR_CMD_CMPL)
			return 1;

	} while (!(timeout_elapsed(expire)));

	return 0;
}

static void ftspi020_reset_ctrl(struct ftspi020_data *hw)
{
	int val;

	/* Set DAMR port selection to Command based */
	val = io_read32(hw->base + FTSPI020_REG_CTRL);
	val |= FTSPI020_CTRL_ABORT;
	io_write32(hw->base + FTSPI020_REG_CTRL, val);

	while (io_read32(hw->base + FTSPI020_REG_CTRL) & FTSPI020_CTRL_ABORT);
}

static int ftspi020_firecmd(struct ftspi020_data *hw, uint32_t cmd3, const uint8_t *tx_buf, uint8_t *rx_buf, size_t len, int bpw)
{
	int ret;

	io_write32(hw->base + FTSPI020_REG_CMD3, cmd3 | FTSPI020_CMD3_INTR_EN);

	if (tx_buf || rx_buf) {
		ret = ftspi020_txrx(&hw->chip, tx_buf, rx_buf, len, bpw);
		if (ret)
			goto out;
	}

	ret = ftspi020_wait_complete(hw, 1000000);
	if (!ret) {
		EMSG("timeout: cmd0 0x%08x cmd1 0x%08x " \
		     "cmd2 0x%08x cmd3 0x%08x\n",
		     io_read32(hw->base + FTSPI020_REG_CMD0),
		     io_read32(hw->base + FTSPI020_REG_CMD1),
		     io_read32(hw->base + FTSPI020_REG_CMD2),
		     io_read32(hw->base + FTSPI020_REG_CMD3));
		ret = -110;
		goto out;
	}

	return 0;
out:
	ftspi020_reset_ctrl(hw);
	return ret;
}

void ftspi020_read_reg(struct ftspi020_data *hw, uint8_t opcode, uint8_t *buf,
                       size_t len)
{
	uint32_t cmd3;

	io_write32(hw->base + FTSPI020_REG_CMD0, 0);

	io_write32(hw->base + FTSPI020_REG_CMD1,
	           FTSPI020_CMD1_OP_CODE_1_BYTE);

	io_write32(hw->base + FTSPI020_REG_CMD2, len);

	cmd3 = (FTSPI020_CMD3_INSTR_CODE(opcode) |
	        FTSPI020_CMD3_CE(0) | FTSPI020_CMD3_SERIAL_MODE |
	        FTSPI020_CMD3_READ);

	ftspi020_firecmd(hw, cmd3, NULL, buf, len, 1);
}

void ftspi020_write_reg(struct ftspi020_data *hw, uint8_t opcode, uint8_t *buf,
                        size_t len)
{
	uint32_t cmd3;

	io_write32(hw->base + FTSPI020_REG_CMD0, 0);

	io_write32(hw->base + FTSPI020_REG_CMD1,
	           FTSPI020_CMD1_OP_CODE_1_BYTE);

	io_write32(hw->base + FTSPI020_REG_CMD2, len);

	cmd3 = (FTSPI020_CMD3_INSTR_CODE(opcode) |
	        FTSPI020_CMD3_CE(0) | FTSPI020_CMD3_SERIAL_MODE |
	        FTSPI020_CMD3_WRITE);

	ftspi020_firecmd(hw, cmd3, buf, NULL, len, 1);
}

void ftspi020_read(struct ftspi020_data *hw, uint32_t from, size_t len,
                   uint8_t *buf)
{
	uint32_t cmd3;

	io_write32(hw->base + FTSPI020_REG_CMD0, from);

	io_write32(hw->base + FTSPI020_REG_CMD1,
	           FTSPI020_CMD1_OP_CODE_1_BYTE |
	           FTSPI020_CMD1_DUMMY_CYCLE(8) |
	           FTSPI020_CMD1_ADDR_BYTES(3));

	io_write32(hw->base + FTSPI020_REG_CMD2, len);

	cmd3 = (FTSPI020_CMD3_INSTR_CODE(SPINOR_OP_READ_FAST) |
	        FTSPI020_CMD3_CE(0) | FTSPI020_CMD3_SERIAL_MODE |
	        FTSPI020_CMD3_READ);

	ftspi020_firecmd(hw, cmd3, NULL, buf, len, 1);
}

void ftspi020_write(struct ftspi020_data *hw, uint32_t to, size_t len,
                    const uint8_t *buf)

{
	uint32_t cmd3;

	io_write32(hw->base + FTSPI020_REG_CMD0, to);

	io_write32(hw->base + FTSPI020_REG_CMD1,
	           FTSPI020_CMD1_OP_CODE_1_BYTE |
	           FTSPI020_CMD1_ADDR_BYTES(3));

	io_write32(hw->base + FTSPI020_REG_CMD2, len);

	cmd3 = (FTSPI020_CMD3_INSTR_CODE(SPINOR_OP_PP) |
	        FTSPI020_CMD3_CE(0) | FTSPI020_CMD3_SERIAL_MODE |
	        FTSPI020_CMD3_WRITE);

	ftspi020_firecmd(hw, cmd3, buf, NULL, len, 1);
}

void ftspi020_erase(struct ftspi020_data *hw, uint32_t offset)
{
	uint32_t cmd3;

	io_write32(hw->base + FTSPI020_REG_CMD0, offset);

	io_write32(hw->base + FTSPI020_REG_CMD1, 
	           FTSPI020_CMD1_OP_CODE_1_BYTE |
	           FTSPI020_CMD1_ADDR_BYTES(3));

	io_write32(hw->base + FTSPI020_REG_CMD2, 0);

	cmd3 = (FTSPI020_CMD3_INSTR_CODE(SPINOR_OP_SE) |
	        FTSPI020_CMD3_CE(0) |
	        FTSPI020_CMD3_WRITE);

	ftspi020_firecmd(hw, cmd3, NULL, NULL, 0, 1);
}

void ftspi020_init(struct ftspi020_data *hw)
{
	uint32_t val;

	assert(hw);

	hw->chip.ops = &ftspi020_ops;

	val = io_read32(hw->base + FTSPI020_REG_FEATURE);
	hw->rxfifo_depth = FTSPI020_FEATURE_RXFIFO_DEPTH(val);
	hw->txfifo_depth = FTSPI020_FEATURE_TXFIFO_DEPTH(val);
	DMSG("tx fifo %d, rx fifo %d\n",
	     hw->txfifo_depth, hw->rxfifo_depth);

#ifdef CFG_FTSPI020_V1_7_0
	val = io_read32(hw->base + FTSPI020_REG_ICR);
	hw->rx_transfer_size = (1 << FTSPI020_ICR_RX_XFR_SIZE(val)) * 32;
	hw->tx_transfer_size = (1 << FTSPI020_ICR_TX_XFR_SIZE(val)) * 32;
#else
	hw->rx_transfer_size = hw->rxfifo_depth;
	hw->tx_transfer_size = hw->txfifo_depth;
#endif
	DMSG("tx transfer %d, rx transfer %d\n",
	     hw->tx_transfer_size, hw->rx_transfer_size);
}