/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2020 Alexander Sverdlin
 */

#include <stddef.h>
#include "gpio.h"
#include "misc.h"
#include "spi.h"
#include "rfm75.h"

#define RFM75_CMD_R_REGISTER		0x00
#define RFM75_CMD_W_REGISTER		0x20
#define RFM75_CMD_R_RX_PAYLOAD		0x61
#define RFM75_CMD_W_TX_PAYLOAD		0xA0
#define RFM75_CMD_FLUSH_TX		0xE1
#define RFM75_CMD_FLUSH_RX		0xE2
#define RFM75_CMD_REUSE_TX_PL		0xE3
#define RFM75_CMD_ACTIVATE		0x50
#define RFM75_CMD_R_RX_PL_WID		0x60
#define RFM75_CMD_W_ACK_PAYLOAD		0xA8
#define RFM75_CMD_W_TX_PAYLOAD_NOACK	0xB0
#define RFM75_CMD_NOP			0xFF

#define RFM75_REG_CONFIG	0x00
#define RFM75_REG_EN_AA		0x01
#define RFM75_REG_EN_RXADDR	0x02
#define RFM75_REG_SETUP_AW	0x03
#define RFM75_REG_SETUP_RETR	0x04
#define RFM75_REG_RF_CH		0x05
#define RFM75_REG_RF_SETUP	0x06
#define RFM75_REG_STATUS	0x07
#define RFM75_REG_OBSERVE_TX	0x08
#define RFM75_REG_CD		0x09
#define RFM75_REG_RX_ADDR_P0	0x0A
#define RFM75_REG_RX_ADDR_P1	0x0B
#define RFM75_REG_RX_ADDR_P2	0x0C
#define RFM75_REG_RX_ADDR_P3	0x0D
#define RFM75_REG_RX_ADDR_P4	0x0E
#define RFM75_REG_RX_ADDR_P5	0x0F
#define RFM75_REG_TX_ADDR	0x10
#define RFM75_REG_RX_PW_P0	0x11
#define RFM75_REG_RX_PW_P1	0x12
#define RFM75_REG_RX_PW_P2	0x13
#define RFM75_REG_RX_PW_P3	0x14
#define RFM75_REG_RX_PW_P4	0x15
#define RFM75_REG_RX_PW_P5	0x16
#define RFM75_REG_FIFO_STATUS	0x17
#define RFM75_REG_DYNPD		0x1C
#define RFM75_REG_FEATURE	0x1D
#define RFM75_REG_CHIP_ID	0x88

/* CONFIG */
#define RFM75_EN_CRC		BIT(3)
#define RFM75_CRCO		BIT(2)
#define RFM75_PWR_UP		BIT(1)
#define RFM75_PRIM_RX		BIT(0)
#define RFM75_DFLT_CONFIG	RFM75_EN_CRC

/* SETUP_RETR */
#define RFM75_ARD_SHIFT		4

/* RF_SETUP */
#define RFM75_RF_DR_LOW		BIT(5)
#define RFM75_RF_DR_HIGH	BIT(3)
#define RFM75_RF_PWR1		BIT(2)
#define RFM75_RF_PWR0		BIT(1)
#define RFM75_LNA_HCURR		BIT(0)
#define RFM75_DFLT_RF_SETUP	(RFM75_RF_PWR1 | RFM75_RF_PWR0 | RFM75_LNA_HCURR | \
				 RFM75_RF_DR_HIGH)
#define RFM75_DATA_RATE_MASK	(RFM75_RF_DR_HIGH | RFM75_RF_DR_LOW)
#define RFM75_DATA_RATE_250KBPS	RFM75_RF_DR_LOW
#define RFM75_DATA_RATE_1MBPS	0
#define RFM75_DATA_RATE_2MBPS	RFM75_RF_DR_HIGH

/* OBSERVE_TX */
#define RFM75_PLOS_CNT_MASK	0xF0
#define RFM75_PLOS_CNT_SHIFT	4
#define RFM75_ARC_CNT_MASK	0x0F

/* CD */
#define RFM75_CD		BIT(0)

/* FIFO_STATUS */
#define RFM75_RX_FULL		BIT(1)
#define RFM75_RX_EMPTY		BIT(0)

/* FEATURE */
#define RFM75_EN_DPL		BIT(2)
#define RFM75_EN_DYN_ACK	BIT(0)

/* CHIP_ID */
#define RFM75_CHIP_ID		0x00000063

static uint8_t gpio_cs;

static void rfm75_activate(uint8_t param)
{
	spi_command(gpio_cs, RFM75_CMD_ACTIVATE, &param, NULL, sizeof(param));
}

uint8_t rfm75_get_status(void)
{
	return spi_command(gpio_cs, RFM75_CMD_NOP, NULL, NULL, 0);
}

static int8_t rfm75_set_bank(uint8_t reg)
{
	uint8_t retries = 3;
	static bool cached;
	static uint8_t status;

	while (retries--) {
		if (cached && !((status ^ reg) & RFM75_RBANK)) {
			return 0;
		}
		rfm75_activate(0x53);
		status = rfm75_get_status();
		cached = true;
	}

	return -1;
}

static int8_t rfm75_reg_write_long(uint8_t addr, const uint8_t *buf, uint8_t count)
{
	int8_t ret;

	ret = rfm75_set_bank(addr);
	if (unlikely(ret))
		return ret;

	spi_command(gpio_cs, RFM75_CMD_W_REGISTER + (addr & 0x1F), buf, NULL, count);

	return 0;
}

static int8_t rfm75_reg_write_long_rom(uint8_t addr, const __flash uint8_t *buf, uint8_t count)
{
	int8_t ret;

	ret = rfm75_set_bank(addr);
	if (unlikely(ret))
		return ret;

	spi_command_rom(gpio_cs, RFM75_CMD_W_REGISTER + (addr & 0x1F), buf, NULL, count);

	return 0;
}

static int8_t rfm75_reg_read_long(uint8_t addr, uint8_t *buf, uint8_t count)
{
	int8_t ret;

	ret = rfm75_set_bank(addr);
	if (unlikely(ret))
		return ret;

	spi_command(gpio_cs, RFM75_CMD_R_REGISTER + (addr & 0x1F), NULL, buf, count);

	return 0;
}

static int8_t rfm75_reg_write(uint8_t addr, uint8_t val)
{
	return rfm75_reg_write_long(addr, &val, sizeof(val));
}

static uint8_t rfm75_reg_read(uint8_t addr)
{
	uint8_t buf;

	rfm75_reg_read_long(addr, &buf, sizeof(buf));
	return buf;
}

/* There is no way to read out if the "features" are enabled, one has to (re)try */
static int8_t rfm75_activate_features(const uint8_t mask)
{
	int8_t ret;
	uint8_t i;

	for (i = 2; ; i--) {
		if (!i)
			return -1;

		/* This one toggles the "enabled" state, not enables it */
		rfm75_activate(0x73);

		ret = rfm75_reg_write(RFM75_REG_FEATURE, mask);
		if (ret)
			return ret;

		if (rfm75_reg_read(RFM75_REG_FEATURE) == mask)
			break;
	}

	return 0;
}

/*
 * Refer to "AN0008- RF-2423 Communication In 250Kbps Air Rate"
 * This voodoo definitely reduces retransmission occurrences
 */
static int8_t rfm75_voodoo_reg4(uint8_t reg, const __flash uint8_t *p)
{
	uint8_t buf[4];

	if ((reg & 0x1F) != 4)
		return 0;

	buf[0] = *p++ | 6;
	buf[1] = *p++;
	buf[2] = *p++;
	buf[3] = *p;

	return rfm75_reg_write_long(reg, buf, sizeof(buf));
}

int8_t rfm75_config(uint8_t cs)
{
	/* Attack of the Clones */
	static const __flash uint8_t reg_blob[] = {
		/* reg, count, values, ... */
		0x80,  4, 0x40, 0x4B, 0x01, 0xE2,
		0x81,  4, 0xC0, 0x4B, 0x00, 0x00,
		0x82,  4, 0xD0, 0xFC, 0x8C, 0x02,
		0x83,  4, 0x99, 0x00, 0x39, 0x21,
		0x84,  4, 0xF9, 0x96, (RFM75_DATA_RATE == RFM75_DATA_RATE_250KBPS) ? 0x8A : 0x82,
				      (RFM75_DATA_RATE == RFM75_DATA_RATE_1MBPS) ? 0x1B : 0xDB,
		0x85,  4, 0x24, 0x06, 0x0F, (RFM75_DATA_RATE == RFM75_DATA_RATE_1MBPS) ? 0xA6 : 0xB6,
		0x8C,  4, 0x00, 0x12, 0x73, 0x00,	/* Last voodoo byte can be 0x05 as well */
		0x8D,  4, 0x36, 0xB4, 0x80, 0x00,
		0x8E, 11, 0x41, 0x20, 0x08, 0x04, 0x81, 0x20, 0xCF, 0xF7, 0xFE, 0xFF, 0xFF,
		/* Sentinel */
		0x00,
	};
	const __flash uint8_t *p = reg_blob;
	int8_t ret;
	uint32_t id;

	gpio_cs = cs;

	ret = rfm75_reg_read_long(RFM75_REG_CHIP_ID, (uint8_t *)&id, sizeof(id));
	if (ret)
		return ret;
	if (id != RFM75_CHIP_ID)
		return -1;

	while (*p) {
		uint8_t reg = *p++;
		uint8_t cnt = *p++;

		ret = rfm75_voodoo_reg4(reg, p);
		if (ret)
			return ret;

		ret = rfm75_reg_write_long_rom(reg, p, cnt);
		if (ret)
			return ret;
		p += cnt;
	}

	ret = rfm75_reg_write(RFM75_REG_SETUP_AW, RFM75_ADDR_WIDTH - 2);
	if (ret)
		return ret;
	ret = rfm75_reg_write(RFM75_REG_RF_CH, RFM75_RF_CH);
	if (ret)
		return ret;
	ret = rfm75_reg_write(RFM75_REG_RF_SETUP, (RFM75_DFLT_RF_SETUP &
						   ~RFM75_DATA_RATE_MASK) |
						  RFM75_DATA_RATE);
	if (ret)
		return ret;

	ret = rfm75_activate_features(RFM75_EN_DPL | RFM75_EN_DYN_ACK);
	if (ret)
		return ret;

	ret = rfm75_reg_write(RFM75_REG_DYNPD, (1 << RFM75_NUM_PIPES) - 1);
	if (ret)
		return ret;

	return 0;
}

static int8_t rfm75_set_addr(uint8_t reg,
			     uint8_t addr4,
			     uint8_t addr3,
			     uint8_t addr2,
			     uint8_t addr1,
			     uint8_t addr0)
{
	switch (reg) {
	case RFM75_REG_RX_ADDR_P2 ... RFM75_REG_RX_ADDR_P5:
		return rfm75_reg_write(reg, addr0);
	}

	{
		uint8_t buf[5];

		buf[0] = addr0;
		buf[1] = addr1;
		buf[2] = addr2;
		buf[3] = addr3;
		buf[4] = addr4;

		return rfm75_reg_write_long(reg, buf, RFM75_ADDR_WIDTH);
	}
}

int8_t rfm75_set_rx_addr(uint8_t addr4,
			 uint8_t addr3,
			 uint8_t addr2,
			 uint8_t addr1,
			 uint8_t addr0)
{
	uint8_t i;
	int8_t ret;

	for (i = 0; i < RFM75_NUM_PIPES; ++i) {
		ret = rfm75_set_addr(RFM75_REG_RX_ADDR_P0 + i,
				     addr4, addr3, addr2, addr1, addr0 + i);
		if (ret)
			return ret;
	}
	return 0;
}

int8_t rfm75_set_tx_addr(uint8_t addr4,
			 uint8_t addr3,
			 uint8_t addr2,
			 uint8_t addr1,
			 uint8_t addr0)
{
	int8_t ret;

	ret = rfm75_set_addr(RFM75_REG_RX_ADDR_P0, addr4, addr3, addr2, addr1, addr0);
	if (ret)
		return ret;
	return rfm75_set_addr(RFM75_REG_TX_ADDR, addr4, addr3, addr2, addr1, addr0);
}

int8_t rfm75_power_up(bool prx)
{
	return rfm75_reg_write(RFM75_REG_CONFIG, RFM75_DFLT_CONFIG |
						 (RFM75_CRC_BYTES == 2 ? RFM75_CRCO : 0) |
						 RFM75_PWR_UP |
						 (prx ? RFM75_PRIM_RX : 0));
}

int8_t rfm75_power_down(void)
{
	return rfm75_reg_write(RFM75_REG_CONFIG, RFM75_DFLT_CONFIG);
}

int8_t rfm75_enable_rx(uint8_t pipes_mask)
{
	return rfm75_reg_write(RFM75_REG_EN_RXADDR, pipes_mask);
}

int8_t rfm75_set_ard(uint8_t ard)
{
	return rfm75_reg_write(RFM75_REG_SETUP_RETR, (ard << RFM75_ARD_SHIFT) + RFM75_ARC);
}

uint8_t rfm75_read_payload(uint8_t *buf, uint8_t size)
{
	uint8_t width;

	spi_command(gpio_cs, RFM75_CMD_R_RX_PL_WID, NULL, &width, sizeof(width));
	if (unlikely(!width))
		return 0;

	spi_command(gpio_cs, RFM75_CMD_R_RX_PAYLOAD, NULL, buf, min(width, size));

	return width;
}

void rfm75_write_payload(const uint8_t *buf, uint8_t len, bool ack)
{
	spi_command(gpio_cs, ack ? RFM75_CMD_W_TX_PAYLOAD : RFM75_CMD_W_TX_PAYLOAD_NOACK,
		    buf, NULL, len);
}

void rfm75_reuse_payload(void)
{
	spi_command(gpio_cs, RFM75_CMD_REUSE_TX_PL, NULL, NULL, 0);
}

int8_t rfm75_irq_ack(uint8_t mask)
{
	return rfm75_reg_write(RFM75_REG_STATUS, mask & (RFM75_RX_DR | RFM75_TX_DS | RFM75_MAX_RT));
}

uint8_t rfm75_arc_cnt(void)
{
	return rfm75_reg_read(RFM75_REG_OBSERVE_TX) & RFM75_ARC_CNT_MASK;
}

void rfm75_flush_tx(void)
{
	spi_command(gpio_cs, RFM75_CMD_FLUSH_TX, NULL, NULL, 0);
}

void rfm75_flush_rx(void)
{
	spi_command(gpio_cs, RFM75_CMD_FLUSH_RX, NULL, NULL, 0);
}
