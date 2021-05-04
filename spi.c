/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020-2021 Alexander Sverdlin
 */

#include <avr/io.h>
#include <avr/power.h>
#include "misc.h"
#include "gpio.h"

static void spi_cs_activate(uint8_t pin)
{
	gpio_set_output(pin, 0);
}

static void spi_cs_deactivate(uint8_t pin)
{
	gpio_set_output(pin, 1);
}

static uint8_t spi_byte(uint8_t txbyte)
{
	SPDR = txbyte;
	while (!(SPSR & BIT(SPIF)));
	return SPDR;
}

uint8_t __spi_command(uint8_t cs, uint8_t cmd, const uint8_t *txbuf, const __flash uint8_t *txrom, uint8_t *rxbuf, uint8_t data_cnt)
{
	uint8_t ret;

	spi_cs_activate(cs);

	ret = spi_byte(cmd);

	while (data_cnt--) {
		uint8_t tmp = spi_byte(txbuf ? *txbuf++ : txrom ? *txrom++ : 0);

		if (likely(rxbuf))
			*rxbuf++ = tmp;
	}

	spi_cs_deactivate(cs);

	return ret;
}

void spi_config(uint8_t cs, uint8_t clkcfg)
{
	power_spi_enable();
	gpio_cfg_input(GPIO_PIN_B3, true);
	gpio_cfg_output(GPIO_PIN_B2, 0);
	gpio_cfg_output(GPIO_PIN_B1, 0);
	gpio_cfg_output(cs, 1);

	/* Enable SPI Master, configure clock */
	SPCR = BIT(SPE) | BIT(MSTR) | (clkcfg & (BIT(SPR0) | BIT(SPR1)));
	SPSR = clkcfg >> 2;
}
