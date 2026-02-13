/**
 * Copyright (c) 2026 William Fish (Manulytica ltd)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT analog_ad74x_uart
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/mfd/ad74x.h>

/**
 * @brief Read a character from the HART RX FIFO.
 *
 * @param dev UART child device instance.
 * @param c Pointer to store the received character.
 * @return 0 on success, -1 if the RX FIFO is empty.
 */
static int uart_ad74x_poll_in(const struct device *dev, unsigned char *c)
{
	const struct device *mfd = dev->parent;
	const struct ad74x_mfd_api *api = mfd->api;
	uint8_t ch = DT_INST_REG_ADDR(0);

	/*
	 * HART Register Mapping Logic:
	 * AD74115H (Single): RFC=0x69, RX=0x6A (Ref: Table 109)
	 * AD74416H (Quad):   RFC=0x85, RX=0x81 (Ref: Table 40, Stride 0x10)
	 */
	uint8_t type = api->get_chip_type(mfd);
	uint8_t rfc_reg = (type == CHIP_AD74115H) ? 0x69 : (0x85 + (ch * 0x10));
	uint8_t rx_reg = (type == CHIP_AD74115H) ? 0x6A : (0x81 + (ch * 0x10));

	uint16_t count;
	/* Check Receive FIFO Count (RFC) register bits [5:0] */
	api->transfer(mfd, rfc_reg, 0, &count, true);
	if ((count & 0x3F) > 0) {
		uint16_t val;
		/* Read data from RX register bits [7:0] */
		api->transfer(mfd, rx_reg, 0, &val, true);
		*c = (unsigned char)(val & 0xFF);
		return 0;
	}
	return -1;
}

/**
 * @brief Write a character to the HART TX FIFO.
 *
 * @param dev UART child device instance.
 * @param c Character to transmit.
 */
static void uart_ad74x_poll_out(const struct device *dev, unsigned char c)
{
	const struct device *mfd = dev->parent;
	const struct ad74x_mfd_api *api = mfd->api;
	uint8_t ch = DT_INST_REG_ADDR(0);

	/*
	 * HART Register Mapping Logic:
	 * AD74115H (Single): MCR=0x6C, TX=0x6B (Ref: Table 112)
	 * AD74416H (Quad):   MCR=0x84, TX=0x82 (Ref: Table 84/86, Stride 0x10)
	 */
	uint8_t type = api->get_chip_type(mfd);
	uint8_t mcr_reg = (type == CHIP_AD74115H) ? 0x6C : (0x84 + (ch * 0x10));
	uint8_t tx_reg = (type == CHIP_AD74115H) ? 0x6B : (0x82 + (ch * 0x10));

	/*
	 * HART is half-duplex. We must set the RTS (Request to Send) bit
	 * in the Modem Control Register (MCR) to enable the carrier for transmission.
	 * Bit 0 = RTS.
	 */
	api->transfer(mfd, mcr_reg, 0x0001, NULL, false);
	/* Write character to the Transmit Data Register (TDR) bits [7:0] */
	api->transfer(mfd, tx_reg, (uint16_t)c, NULL, false);
}

static const struct uart_driver_api uart_api = {
	.poll_in = uart_ad74x_poll_in,
	.poll_out = uart_ad74x_poll_out,
};

/* Standard Zephyr instance iteration pattern */
#define AD74X_UART_DEFINE(n)                                                                       \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL, 60, &uart_api);
DT_INST_FOREACH_STATUS_OKAY(AD74X_UART_DEFINE)
