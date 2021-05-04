/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020-2021 Alexander Sverdlin
 */

#ifndef _SPI_H_
#define _SPI_H_

#include <stddef.h>
#include <stdint.h>

/* Combine SPI2X:SPR1:SPR0 to achive SPI clock not higher than clk */
#define SPI_CLK_CFG(clk) (F_CPU / 2 <= (clk) ? 4 : \
			  F_CPU / 4 <= (clk) ? 0 : \
			  F_CPU / 8 <= (clk) ? 5 : \
			  F_CPU / 16 <= (clk) ? 1 : \
			  F_CPU / 32 <= (clk) ? 6 : \
			  F_CPU / 64 <= (clk) ? 2 : 3)

void spi_config(uint8_t cs, uint8_t clkcfg);

uint8_t __spi_command(uint8_t cs, uint8_t cmd, const uint8_t *txbuf, const __flash uint8_t *txrom, uint8_t *rxbuf, uint8_t data_cnt);

/*
 * @txbuf points to transmit buffer of size cnt or NULL
 * @rxbuf points to receive buffer of size cnt or NULL
 * @data_cnt doesn't account for cmd to be transmitted, and is minimum 0
 * function returns the first byte received during the shifting of the command
 */
static uint8_t __maybe_unused spi_command(uint8_t cs, uint8_t cmd, const uint8_t *txbuf, uint8_t *rxbuf, uint8_t data_cnt)
{
	return __spi_command(cs, cmd, txbuf, NULL, rxbuf, data_cnt);
}

static uint8_t __maybe_unused spi_command_rom(uint8_t cs, uint8_t cmd, const __flash uint8_t *txbuf, uint8_t *rxbuf, uint8_t data_cnt)
{
	return __spi_command(cs, cmd, NULL, txbuf, rxbuf, data_cnt);
}

#endif /* _SPI_H_ */
