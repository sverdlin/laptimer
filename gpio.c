/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020 Alexander Sverdlin
 */

#include <avr/io.h>
#include "misc.h"
#include "gpio.h"

static volatile uint8_t *const ports[] = {
	&PORTB,
	&PORTC,
	&PORTD,
};
static volatile uint8_t *const ddrs[] = {
	&DDRB,
	&DDRC,
	&DDRD,
};
static volatile uint8_t *const pins[] = {
	&PINB,
	&PINC,
	&PIND,
};

#define PINBANK(pin)	((pin) / 8)
#define PORTx(pin)	(*ports[PINBANK(pin)])
#define DDRx(pin)	(*ddrs[PINBANK(pin)])
#define PINx(pin)	(*pins[PINBANK(pin)])
#define PINBIT(pin)	((pin) & 7)

void gpio_cfg_output(uint8_t pin, bool initial_state)
{
	uint8_t mask = BIT(PINBIT(pin));
	uint8_t nmask = ~mask;

	if (initial_state)
		PORTx(pin) |= mask;
	else
		PORTx(pin) &= nmask;

	DDRx(pin) |= mask;
}

void gpio_cfg_input(uint8_t pin, bool pullup)
{
	uint8_t mask = BIT(PINBIT(pin));
	uint8_t nmask = ~mask;

	DDRx(pin) &= nmask;

	if (pullup)
		PORTx(pin) |= mask;
	else
		PORTx(pin) &= nmask;
}

void gpio_set_output(uint8_t pin, bool state)
{
	if (state)
		PORTx(pin) |= BIT(PINBIT(pin));
	else
		PORTx(pin) &= ~BIT(PINBIT(pin));
}

bool gpio_get_input(uint8_t pin)
{
	return PINx(pin) & BIT(PINBIT(pin));
}

void gpio_unused(const __flash uint8_t *pins, uint8_t count)
{
	while (count--)
		gpio_cfg_input(*pins++, true);
}
