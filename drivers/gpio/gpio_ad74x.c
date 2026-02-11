/**
 * Copyright (c) 2026 William Fish (Manulytica ltd)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT analog_ad74x_gpio
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mfd/ad74x.h>

static int gpio_ad74x_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct device *mfd = dev->parent;
	const struct ad74x_mfd_api *api = mfd->api;
	uint8_t setup_reg =
		AD74X_REG_CH_FUNC_SETUP(api->get_chip_type(mfd) == CHIP_AD74416H ? pin : 0);

	if (flags & GPIO_OUTPUT) {
		api->transfer(mfd, setup_reg, 0x0006, NULL, false); // Mode 0x06: DO
		uint8_t do_reg = (api->get_chip_type(mfd) == CHIP_AD74115H)
					 ? 0x09
					 : (0x08 + (pin * AD74X_CH_STRIDE));
		return api->transfer(mfd, do_reg, 0x0001, NULL, false);
	}
	return api->transfer(mfd, setup_reg, 0x0005, NULL, false); // Mode 0x05: DI
}

static int gpio_ad74x_port_set_masked(const struct device *dev, uint32_t mask, uint32_t value)
{
	const struct device *mfd = dev->parent;
	const struct ad74x_mfd_api *api = mfd->api;
	uint8_t type = api->get_chip_type(mfd);

	for (int i = 0; i < (type == CHIP_AD74416H ? 4 : 1); i++) {
		if (mask & BIT(i)) {
			uint16_t reg_val;
			uint8_t data_reg =
				(type == CHIP_AD74115H) ? 0x09 : (0x08 + (i * AD74X_CH_STRIDE));
			api->transfer(mfd, data_reg, 0, &reg_val, true);
			if (value & BIT(i)) {
				reg_val |= BIT(0);
			} else {
				reg_val &= ~BIT(0); // BIT(0) is Data
			}
			api->transfer(mfd, data_reg, reg_val, NULL, false);
		}
	}
	return 0;
}

static const struct gpio_driver_api gpio_api = {.pin_configure = gpio_ad74x_config,
						.port_set_masked_raw = gpio_ad74x_port_set_masked};
#define AD74X_GPIO_DEFINE(n)                                                                       \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL, 50, &gpio_api);
DT_INST_FOREACH_STATUS_OKAY(AD74X_GPIO_DEFINE)
