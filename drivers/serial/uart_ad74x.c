/**
 * Copyright (c) 2026 William Fish (Manulytica ltd)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT analog_ad74x_uart
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/mfd/ad74x.h>

static int uart_ad74x_poll_in(const struct device *dev, unsigned char *c)
{
	const struct device *mfd = dev->parent;
	const struct ad74x_mfd_api *api = mfd->api;
	uint8_t ch = DT_INST_REG_ADDR(0);
	uint8_t rfc_reg = (api->get_chip_type(mfd) == CHIP_AD74115H) ? 0x69 : (0x85 + (ch * 0x10));
	uint8_t rx_reg = (api->get_chip_type(mfd) == CHIP_AD74115H) ? 0x6A : (0x81 + (ch * 0x10));

	uint16_t count;
	api->transfer(mfd, rfc_reg, 0, &count, true);
	if ((count & 0x3F) > 0) {
		uint16_t val;
		api->transfer(mfd, rx_reg, 0, &val, true);
		*c = (unsigned char)(val & 0xFF);
		return 0;
	}
	return -1;
}

static void uart_ad74x_poll_out(const struct device *dev, unsigned char c)
{
	const struct device *mfd = dev->parent;
	const struct ad74x_mfd_api *api = mfd->api;
	uint8_t ch = DT_INST_REG_ADDR(0);
	uint8_t mcr_reg = (api->get_chip_type(mfd) == CHIP_AD74115H) ? 0x6C : (0x84 + (ch * 0x10));
	uint8_t tx_reg = (api->get_chip_type(mfd) == CHIP_AD74115H) ? 0x6B : (0x82 + (ch * 0x10));

	api->transfer(mfd, mcr_reg, 0x0001, NULL, false); // RTS High
	api->transfer(mfd, tx_reg, (uint16_t)c, NULL, false);
}

static const struct uart_driver_api uart_api = {.poll_in = uart_ad74x_poll_in,
						.poll_out = uart_ad74x_poll_out};
#define AD74X_UART_DEFINE(n)                                                                       \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL, 60, &uart_api);
DT_INST_FOREACH_STATUS_OKAY(AD74X_UART_DEFINE)
