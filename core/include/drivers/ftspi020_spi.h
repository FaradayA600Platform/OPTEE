/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (c) 2016, Linaro Limited
 *
 */

#ifndef __FTSPI020_SPI_H__
#define __FTSPI020_SPI_H__

#include <spi.h>

struct ftspi020_data {
	struct spi_chip     chip;
	vaddr_t             base;
	enum spi_mode       mode;
	unsigned int        tx_transfer_size;
	unsigned int        rx_transfer_size;
	unsigned char       txfifo_depth;
	unsigned char       rxfifo_depth;
};

void ftspi020_init(struct ftspi020_data *hw);
void ftspi020_read_reg(struct ftspi020_data *hw, uint8_t opcode, uint8_t *buf,
                       size_t len);
void ftspi020_write_reg(struct ftspi020_data *hw, uint8_t opcode, uint8_t *buf,
                        size_t len);
void ftspi020_read(struct ftspi020_data *hw, uint32_t from, size_t len,
                   uint8_t *buf);
void ftspi020_write(struct ftspi020_data *hw, uint32_t to, size_t len,
                    const uint8_t *buf);
void ftspi020_erase(struct ftspi020_data *hw, uint32_t offset);

#endif	/* __FTSPI020_SPI_H__ */

