/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020-2021 Alexander Sverdlin
 */

#ifndef _RFM75_H_
#define _RFM75_H_

#include <stdbool.h>
#include <stdint.h>
#include "misc.h"

/* Transceiver configuration */
#define RFM75_CRC_BYTES		2
#define RFM75_ADDR_WIDTH	3
#define RFM75_ARC		15
#define RFM75_RF_CH		((72 + 84) / 2)	/* See WLAN freq map */
#define RFM75_DATA_RATE		RFM75_DATA_RATE_250KBPS

#define RFM75_ADDR4		'S'
#define RFM75_ADDR3		'V'
#define RFM75_ADDR2		'S'
#define RFM75_ADDR1		'U'
#define RFM75_ADDR0		'L'

#define RFM75_NUM_PIPES		6
#define RFM75_MAX_CH		127
#define RFM75_MAX_SPI_CLK	8000000

/* STATUS */
#define RFM75_RBANK		BIT(7)
#define RFM75_RX_DR		BIT(6)
#define RFM75_TX_DS		BIT(5)
#define RFM75_MAX_RT		BIT(4)
#define RFM75_RX_P_NO_MASK	(BIT(3) | BIT(2) | BIT(1))
#define RFM75_RX_P_NO_SHIFT	1
#define RFM75_TX_FULL		BIT(0)

uint8_t rfm75_get_status(void);
int8_t rfm75_config(uint8_t cs);
int8_t rfm75_set_rx_addr(uint8_t addr4,
			 uint8_t addr3,
			 uint8_t addr2,
			 uint8_t addr1,
			 uint8_t addr0);
int8_t rfm75_set_tx_addr(uint8_t addr4,
			 uint8_t addr3,
			 uint8_t addr2,
			 uint8_t addr1,
			 uint8_t addr0);
int8_t rfm75_power_up(bool prx);
int8_t rfm75_power_down(void);
int8_t rfm75_enable_rx(uint8_t pipes_mask);
int8_t rfm75_set_ard(uint8_t ard);
uint8_t rfm75_read_payload(uint8_t *buf, uint8_t size);
void rfm75_write_payload(const uint8_t *buf, uint8_t len, bool ack);
void rfm75_reuse_payload(void);
int8_t rfm75_irq_ack(uint8_t mask);
uint8_t rfm75_arc_cnt(void);
void rfm75_flush_tx(void);
void rfm75_flush_rx(void);

static bool __maybe_unused rfm75_data_ready(uint8_t status)
{
	return status & RFM75_RX_DR;
}

static bool __maybe_unused rfm75_data_sent(uint8_t status)
{
	return status & RFM75_TX_DS;
}

static bool __maybe_unused rfm75_max_rt(uint8_t status)
{
	return status & RFM75_MAX_RT;
}

#endif /* _RFM75_H_ */
