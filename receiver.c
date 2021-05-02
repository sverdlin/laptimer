/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020 Alexander Sverdlin
 *
 * LED Indication:
 *
 * Yellow blinks 8Hz:		Failed to configure/detect Radio Module
 * Yellow constant:		Radio Module malfunction (interrupt flood)
 */

#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "hw_receiver.h"
#include <util/delay.h>
#include "misc.h"
#include "gpio.h"
#include "power.h"
#include "spi.h"
#include "rfm75.h"
#include "record.h"
#include "timer.h"
#include "usb.h"

FUSES = {
	/* No CLK division, no CLK output on PC7/CLKO, startup delay 14CK + 65 ms, Full Swing Crystal Oscillator */
	.low		= FUSE_CKSEL3,
	/* Reset vector @0x0000, no forced WDT, enable SPI flashing, Reset pin enabled */
	.high		= (HFUSE_DEFAULT & FUSE_SPIEN) | ~FUSE_BOOTRST | ~FUSE_WDTON | ~FUSE_RSTDISBL,
	/* BOR at 3.0v, PD7/HWB pin can not be used to force Boot Loader execution after reset */
	.extended	= (EFUSE_DEFAULT & FUSE_BODLEVEL1 & FUSE_BODLEVEL0) | ~FUSE_BODLEVEL2 | ~FUSE_HWBE,
};

static void hw_error(uint8_t led) __attribute__((noreturn));
static void hw_error(uint8_t led)
{
	bool s = 0;

	for (;;) {
		s ^= 1;
		gpio_set_output(led, s);
		_delay_ms(64);
	}
}

static void radio_interrupt_enable(void)
{
	/* The low level of INT6 generates an interrupt request */
	EIMSK |= BIT(INT6);
}

static void radio_interrupt_disable(void)
{
	EIMSK &= ~BIT(INT6);
}

/* Used only to wake up MCU from sleep */
ISR(INT6_vect)
{
	radio_interrupt_disable();
}

static void power_cfg(void)
{
	power_ac_disable();
	power_timer0_disable();
	power_timer1_disable();
	power_usart1_disable();
}

static void gpio_cfg(void)
{
	static const __flash uint8_t gpio_unused_tab[] = {
		GPIO_PIN_B7,
		GPIO_PIN_C2,
		GPIO_PIN_C4,
		GPIO_PIN_C5,
		GPIO_PIN_C6,
		GPIO_PIN_C7,
		GPIO_PIN_D4,
		GPIO_PIN_D7,
		GPIO_PIN_D0,
		GPIO_PIN_D1,
		GPIO_PIN_D2,
		GPIO_PIN_D3,
		GPIO_PIN_B4,
	};

	gpio_unused(gpio_unused_tab, sizeof(gpio_unused_tab));

	gpio_cfg_output(GPIO_PIN_LEDG, 0);
	gpio_cfg_output(GPIO_PIN_LEDY, 0);
}

static void radio_cfg(void)
{
	if (rfm75_config(GPIO_PIN_CS) ||
	    rfm75_set_rx_addr(RFM75_ADDR4, RFM75_ADDR3, RFM75_ADDR2, RFM75_ADDR1, RFM75_ADDR0) ||
	    rfm75_enable_rx((1 << RFM75_NUM_PIPES) - 1) ||
	    rfm75_power_up(true))	/* Power up as PRX */
		hw_error(GPIO_PIN_LEDY);

	_delay_ms(2);			/* Refer to state diagrams in RFM75 datasheet */

	gpio_cfg_output(GPIO_PIN_CE, 1);
}

static void process_lap(uint8_t addr, __uint24 ts)
{
	uint16_t mask = BIT(addr);
	static uint16_t received;
	static __uint24 tss[ID_COUNT];

	if (received & mask && tss[addr] != ts) {
		printf_P(PSTR("LAP %u %lu\n"), addr, (unsigned long)(ts - tss[addr]));
	}
	received |= mask;
	tss[addr] = ts;
}

static void process_incoming_pkt(void)
{
	struct lap newlap;
	uint8_t ret;

	ret = rfm75_read_payload((uint8_t *)&newlap, sizeof(newlap));
	if (ret != sizeof(newlap))
		return;

	ret = newlap.id % ID_COUNT;

	switch (newlap.id / ID_COUNT) {
	case 0:
		process_lap(ret, newlap.ts);
		break;
	case 1:
		/* Retries in last transmission */
		printf_P(PSTR("RET %u %u\n"), ret, (uint8_t)newlap.ts);
		break;
	case 2:
		/* Power up event */
		printf_P(PSTR("PWR %u\n"), ret);
		break;
	}
}

static void radio_interrupt_poll(void)
{
	bool ostate;

	if (unlikely(gpio_get_input(GPIO_PIN_RIRQ)))
		return;

	ostate = gpio_get_input(GPIO_PIN_LEDY);
	gpio_set_output(GPIO_PIN_LEDY, 1);

	while (!gpio_get_input(GPIO_PIN_RIRQ)) {
		uint8_t status = rfm75_get_status();

		if (likely(rfm75_data_ready(status)))
			process_incoming_pkt();

		if (rfm75_irq_ack(status))
			hw_error(GPIO_PIN_LEDY);
	}

	gpio_set_output(GPIO_PIN_LEDY, ostate);
}

static void sleep_in_main_loop(void)
{
	static bool printed;

	if (!printed)
		printed = printf_P(PSTR("VER %x.%02x\n"), (uint8_t)(USB_PRODUCT_RELEASE >> 8), (uint8_t)USB_PRODUCT_RELEASE) > 0;

	/*
	 * It's important that we go to sleep before INT6 is served
	 * otherwise we miss it and will not be woken up.
	 */
	cli();
	radio_interrupt_enable();
	sei();
	sleep_cpu();
}

static int usb_putchar(char c, FILE *stream)
{
	if (usb_cdc_putchar(c))
		return -1;

	if (c >= 0x20)
		return 0;

	return usb_cdc_drain() ? -1 : 0;
}

static void stdio_cfg(void)
{
	static FILE usbstdout = FDEV_SETUP_STREAM(usb_putchar, NULL, _FDEV_SETUP_WRITE);

	stdout = &usbstdout;
}

int main(void) __attribute__((noreturn));
int main(void)
{
	power_cfg();
	gpio_cfg();

	spi_config(GPIO_PIN_CS, SPI_CLK_CFG(RFM75_MAX_SPI_CLK));

	sei();
	sleep_enable();

	usb_cdc_init(USB_PLL_CFG, 0);
	radio_cfg();
	stdio_cfg();

	for (;;) {
		radio_interrupt_poll();
		sleep_in_main_loop();
	}
}
