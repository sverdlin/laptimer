/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020-2021 Alexander Sverdlin
 */

#ifndef _GPIO_H_
#define _GPIO_H_

#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include "misc.h"

#define GPIO_PIN_B0	0x00
#define GPIO_PIN_B1	0x01
#define GPIO_PIN_B2	0x02
#define GPIO_PIN_B3	0x03
#define GPIO_PIN_B4	0x04
#define GPIO_PIN_B5	0x05
#define GPIO_PIN_B6	0x06
#define GPIO_PIN_B7	0x07
#define GPIO_PIN_C0	0x08
#define GPIO_PIN_C1	0x09
#define GPIO_PIN_C2	0x0A
#define GPIO_PIN_C3	0x0B
#define GPIO_PIN_C4	0x0C
#define GPIO_PIN_C5	0x0D
#define GPIO_PIN_C6	0x0E
#define GPIO_PIN_C7	0x0F
#define GPIO_PIN_D0	0x10
#define GPIO_PIN_D1	0x11
#define GPIO_PIN_D2	0x12
#define GPIO_PIN_D3	0x13
#define GPIO_PIN_D4	0x14
#define GPIO_PIN_D5	0x15
#define GPIO_PIN_D6	0x16
#define GPIO_PIN_D7	0x17

static volatile uint8_t *const gpio_ports[] = {
	&PORTB,
	&PORTC,
	&PORTD,
};
static volatile uint8_t *const gpio_ddrs[] = {
	&DDRB,
	&DDRC,
	&DDRD,
};
static volatile uint8_t *const gpio_pins[] = {
	&PINB,
	&PINC,
	&PIND,
};

#define PINBANK(pin)	((pin) / 8)
#define PORTx(pin)	(*gpio_ports[PINBANK(pin)])
#define DDRx(pin)	(*gpio_ddrs[PINBANK(pin)])
#define PINx(pin)	(*gpio_pins[PINBANK(pin)])
#define PINBIT(pin)	((pin) & 7)

static __maybe_unused void gpio_cfg_output(uint8_t pin, bool initial_state)
{
	uint8_t mask = BIT(PINBIT(pin));
	uint8_t nmask = ~mask;

	if (initial_state)
		PORTx(pin) |= mask;
	else
		PORTx(pin) &= nmask;

	DDRx(pin) |= mask;
}

static __maybe_unused void gpio_cfg_input(uint8_t pin, bool pullup)
{
	uint8_t mask = BIT(PINBIT(pin));
	uint8_t nmask = ~mask;

	DDRx(pin) &= nmask;

	if (pullup)
		PORTx(pin) |= mask;
	else
		PORTx(pin) &= nmask;
}

static __maybe_unused void gpio_set_output(uint8_t pin, bool state)
{
	if (state)
		PORTx(pin) |= BIT(PINBIT(pin));
	else
		PORTx(pin) &= ~BIT(PINBIT(pin));
}

static __maybe_unused bool gpio_get_input(uint8_t pin)
{
	return PINx(pin) & BIT(PINBIT(pin));
}

static __maybe_unused void gpio_unused(const __flash uint8_t *pins, uint8_t count)
{
	while (count--)
		gpio_cfg_input(*pins++, true);
}

#endif /* _GPIO_H_ */
