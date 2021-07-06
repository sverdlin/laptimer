/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020-2021 Alexander Sverdlin
 *
 * LED Indication:
 *
 * Green single blinks:		packet transmitted
 *
 * Yellow blinks 1Hz:		FIFO overflow (no ACK from receiver)
 * Yellow blinks 8Hz:		Failed to configure/detect Radio Module
 * Yellow constant:		Radio Module malfunction (interrupt flood)
 */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include "hw_sender.h"
#include <util/delay.h>
#include "misc.h"
#include "gpio.h"
#include "power.h"
#include "spi.h"
#include "rfm75.h"
#include "record.h"
#include "timer.h"

#define LAP_TIME_MIN	10000	/* ms */
#define LAP_FIFO_SIZE	32
#define LAP_TIMER_FREQ	1000
#define MAX_SW_RETR	16

FUSES = {
	/* No CLK division, no CLK output on PC7/CLKO, startup delay 14CK + 65 ms, Full Swing Crystal Oscillator */
	.low		= FUSE_CKSEL3,
	/* Reset vector @0x0000, no forced WDT, enable SPI flashing, Reset pin enabled */
	.high		= (HFUSE_DEFAULT & FUSE_SPIEN) | ~FUSE_BOOTRST | ~FUSE_WDTON | ~FUSE_RSTDISBL,
	/* BOR at 3.0v, PD7/HWB pin can not be used to force Boot Loader execution after reset */
	.extended	= (EFUSE_DEFAULT & FUSE_BODLEVEL1 & FUSE_BODLEVEL0) | ~FUSE_BODLEVEL2 | ~FUSE_HWBE,
};

volatile static uint8_t sw_retr_cnt;
static struct lap lap_fifo[LAP_FIFO_SIZE + BUILD_BUG_ON_ZERO(sizeof(struct lap) > 32)];
volatile static uint8_t fifo_head, fifo_tail;
volatile static __uint24 tmr1ms;
static bool last_packet_lap;
static bool packet_inflight;
static uint8_t last_retries;
volatile static bool fifo_overflow;

static bool is_packet_inflight(void)
{
	return packet_inflight;
}

static void set_packet_inflight(bool state)
{
	packet_inflight = state;
	gpio_set_output(GPIO_PIN_CE, state);
	gpio_set_output(GPIO_PIN_LEDG, state);
}

/* Shall be called with interrupts disabled */
static __uint24 get_timestamp(void)
{
	return tmr1ms;
}

static void led_processing(void)
{
	gpio_set_output(GPIO_PIN_LEDY, fifo_overflow ? (tmr1ms & BIT(9)) : 0);
}

static void hw_error(uint8_t led) __attribute__((noreturn));
static void hw_error(uint8_t led)
{
	for (;;) {
		gpio_set_output(led, tmr1ms & BIT(9 - 3));
		sleep_cpu();
	}
}

static uint8_t get_address(void)
{
	static const uint8_t pins[] = {GPIO_PIN_AD3, GPIO_PIN_AD2,
				       GPIO_PIN_AD1, GPIO_PIN_AD0};
	uint8_t a = 0xFF;
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(pins); ++i) {
		a <<= 1;
		a |= gpio_get_input(pins[i]);
	}

	return ~a;
}

#define IMPULSE_US	6538
#define CARRIER_PERIOD	(F_CPU / 1000000 * IMPULSE_US)
static void signal_tmr_cfg(void)
{
	/* Timer period */
	OCR1A = TMR1OCR(CARRIER_PERIOD);
	/* Clear Timer on Compare Match */
	TCCR1B = BIT(WGM12);
	/* Compare Match A Interrupt Enable */
	TIMSK1 |= BIT(OCIE1A);
}

static void signal_tmr_enable(void)
{
	TCCR1B |= TMR1PREBITS(CARRIER_PERIOD);
}

static void signal_tmr_disable(void)
{
	TCCR1B &= ~(BIT(CS12) | BIT(CS11) | BIT(CS10));
}

static uint16_t last_signal_tmr;

static void signal_tmr_clr(void)
{
	TCNT1 = 0;
	last_signal_tmr = 0;
}

static uint16_t signal_tmr_val(void)
{
	return TCNT1;
}

/* Accumulators for Fourier transform */
static int32_t acc_a, acc_b, prev_acc_a, prev_acc_b;
/* Walsh function values */
static uint8_t v_a, v_b;
/* Rest time before above function value changes */
static uint16_t rest_a, rest_b;

static void signal_acc_clr(void)
{
	acc_a = 0;
	acc_b = 0;
	prev_acc_a = 0;
	prev_acc_b = 0;
	v_a = 0;
	v_b = 0;
	rest_a = CARRIER_PERIOD / 2;
	rest_b = CARRIER_PERIOD / 4;
}

static __always_inline void signal_fourier_loop(int32_t *acc, uint8_t *v, uint8_t s, uint16_t *rest, uint16_t time)
{
	while (time) {
		if (time >= *rest) {
			time -= *rest;
			if (s ^ *v)
				*acc -= *rest;
			else
				*acc += *rest;
			*v ^= 1;
			*rest = CARRIER_PERIOD / 2;
		} else {
			*rest -= time;
			if (s ^ *v)
				*acc -= time;
			else
				*acc += time;
			time = 0;
		}
	}
}

static void signal_fourier_run(uint8_t signal)
{
	uint16_t ts = signal_tmr_val();
	__int24 tmp = ts - last_signal_tmr;

	last_signal_tmr = ts;
	if (tmp < 0)
		tmp += CARRIER_PERIOD;

	signal_fourier_loop(&acc_a, &v_a, signal, &rest_a, tmp);
	signal_fourier_loop(&acc_b, &v_b, signal, &rest_b, tmp);
}

#define SIGNAL_THRESHOLD	1673658368
static bool signal_detect(uint8_t signal)
{
	signal_fourier_run(signal);

	prev_acc_a += acc_a;
	prev_acc_b += acc_b;

	if ((int64_t)prev_acc_a * prev_acc_a + (int64_t)prev_acc_b * prev_acc_b >= SIGNAL_THRESHOLD)
		return true;

	prev_acc_a = acc_a;
	prev_acc_b = acc_b;
	return false;
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

static __uint24 last_lap_ts;
static bool last_lap_valid;
static __uint24 lap_ts;
/*
 * If there was a pin state change within 153Hz period, first
 * captured timestamp is still valid.
 */
static bool lap_ts_valid;
static bool pin_change;

/* Light signal, PCINT4 */
ISR(PCINT0_vect)
{
	signal_tmr_enable();
	signal_fourier_run(!gpio_get_input(GPIO_PIN_SIGNAL));

	pin_change = true;
	if (lap_ts_valid)
		return;

	lap_ts = get_timestamp();
}

ISR(TIMER1_COMPA_vect)
{
	struct lap *newlap;

	if (!pin_change) {
		lap_ts_valid = false;
		signal_acc_clr();
		signal_tmr_disable();
		signal_tmr_clr();
		return;
	}
	pin_change = false;

	if (!signal_detect(gpio_get_input(GPIO_PIN_SIGNAL)))
		return;

	if (last_lap_valid && (lap_ts - last_lap_ts < LAP_TIME_MIN))
		return;

	newlap = &lap_fifo[fifo_tail];
	newlap->ts = lap_ts;
	newlap->id = get_address();

	/* Ensure that % LAP_FIFO_SIZE can be done with bitmasking */
	fifo_tail = (fifo_tail + 1) % LAP_FIFO_SIZE +
		    BUILD_BUG_ON_NOT_POWER_OF_2(LAP_FIFO_SIZE);
	if (unlikely(fifo_tail == fifo_head))
		fifo_overflow = true;
	sw_retr_cnt = 0;
	last_lap_ts = lap_ts;
	last_lap_valid = true;
}
ISR(TIMER1_OVF_vect, ISR_ALIASOF(TIMER1_COMPA_vect));

ISR(TIMER0_COMPA_vect)
{
	tmr1ms++;
}
ISR(TIMER0_OVF_vect, ISR_ALIASOF(TIMER0_COMPA_vect));

static void lap_timer_cfg(void)
{
	/* Timer period */
	OCR0A = TMR0OCR(LAP_TIMER_FREQ);
	/* Clear Timer on Compare Match */
	TCCR0A = BIT(WGM01);
	/* Prescaler */
	TCCR0B = TMR0PREBITS(LAP_TIMER_FREQ);
	/* Compare Match A Interrupt Enable */
	TIMSK0 |= BIT(OCIE0A);
}

static void power_cfg(void)
{
	power_ac_disable();
	power_timer1_enable();
	power_usart1_disable();
	power_usb_disable();
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
	};

	gpio_unused(gpio_unused_tab, sizeof(gpio_unused_tab));

	gpio_cfg_input(GPIO_PIN_AD0, true);
	gpio_cfg_input(GPIO_PIN_AD1, true);
	gpio_cfg_input(GPIO_PIN_AD2, true);
	gpio_cfg_input(GPIO_PIN_AD3, true);
	gpio_cfg_output(GPIO_PIN_LEDG, 0);
	gpio_cfg_output(GPIO_PIN_LEDY, 0);

	/* Enable PCINT4 (PCI0) */
	PCICR |= BIT(PCIE0);
	PCMSK0 |= BIT(PCINT4);
}

static void radio_cfg(void)
{
	gpio_cfg_output(GPIO_PIN_CE, 0);

	if (rfm75_config(GPIO_PIN_CS) ||
	    rfm75_set_tx_addr(RFM75_ADDR4, RFM75_ADDR3, RFM75_ADDR2, RFM75_ADDR1, RFM75_ADDR0 + get_address() % RFM75_NUM_PIPES) ||
	    rfm75_enable_rx(1) ||	/* For ACK reception enable pipe 0 only */
	    rfm75_power_up(false))	/* Power up as PTX */
		hw_error(GPIO_PIN_LEDY);

	_delay_ms(2);			/* Refer to state diagrams in RFM75 datasheet */
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

		if (likely(rfm75_data_sent(status))) {
			if (likely(last_packet_lap)) {
				if (likely(fifo_head != fifo_tail)) {
					/* Ensure that % LAP_FIFO_SIZE can be done with bitmasking */
					fifo_head = (fifo_head + 1) % LAP_FIFO_SIZE +
						    BUILD_BUG_ON_NOT_POWER_OF_2(LAP_FIFO_SIZE);
					fifo_overflow = false;
				} else {
					/* This could happen in case of FIFO overflow */
				}
				sw_retr_cnt = 0;
				last_retries = rfm75_arc_cnt();
			}
			set_packet_inflight(false);
		}

		if (unlikely(rfm75_max_rt(status))) {
			set_packet_inflight(false);
			sw_retr_cnt++;
			last_retries = 255;
			rfm75_flush_tx();
		}

		if (unlikely(rfm75_data_ready(status)))
			rfm75_flush_rx();

		if (rfm75_irq_ack(status))
			hw_error(GPIO_PIN_LEDY);
	}

	gpio_set_output(GPIO_PIN_LEDY, ostate);
}

static void radio_transmit(const uint8_t *buf, uint8_t count)
{
	set_packet_inflight(true);
	/* Differentiate Lap Data and diagnostic info */
	last_packet_lap = ((struct lap *)buf)->id < ID_COUNT;
	rfm75_write_payload(buf, count, true);
}

static void try_transmit(void)
{
	if (is_packet_inflight() || sw_retr_cnt > MAX_SW_RETR)
		return;

	if (fifo_head != fifo_tail) {
		radio_transmit((uint8_t *)&lap_fifo[fifo_head], sizeof(lap_fifo[fifo_head]));
		return;
	}

	if (unlikely(last_packet_lap && last_retries)) {
		struct lap buf;

		buf.id = get_address() + ID_COUNT;
		buf.ts = last_retries;

		radio_transmit((uint8_t *)&buf, sizeof(buf));

		return;
	}
}

static void event_power_up(void)
{
	struct lap buf;

	buf.id = get_address() + ID_COUNT * 2;
	buf.ts = 0;
	radio_transmit((uint8_t *)&buf, sizeof(buf));
}

static void sleep_in_main_loop(void)
{
	/*
	 * It's important that we go to sleep before INT6 is served
	 * otherwise we miss it and will not be woken up.
	 */
	cli();
	radio_interrupt_enable();
	sei();
	sleep_cpu();
}

int main(void) __attribute__((noreturn));
int main(void)
{
	power_cfg();
	gpio_cfg();
	lap_timer_cfg();
	signal_acc_clr();
	signal_tmr_cfg();

	spi_config(GPIO_PIN_CS, SPI_CLK_CFG(RFM75_MAX_SPI_CLK));

	sei();
	sleep_enable();

	radio_cfg();
	event_power_up();

	for (;;) {
		radio_interrupt_poll();
		try_transmit();
		led_processing();
		sleep_in_main_loop();
	}
}
