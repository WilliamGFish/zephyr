/**
 * Copyright (c) 2026 William Fish (Manulytica ltd)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MFD_AD74X_H_
#define ZEPHYR_DRIVERS_MFD_AD74X_H_

#include <zephyr/device.h>

enum ad74x_chip_type {
	CHIP_AD74416H,
	CHIP_AD74115H
};

struct ad74x_mfd_api {
	int (*transfer)(const struct device *dev, uint8_t reg, uint16_t val_in, uint16_t *val_out,
			bool is_read);
	uint8_t (*get_chip_type)(const struct device *dev);
	struct k_sem *(*get_adc_sem)(const struct device *dev);
};

#define AD74X_CH_STRIDE            0x0C
#define AD74X_REG_CH_FUNC_SETUP(n) (0x01 + (n * AD74X_CH_STRIDE))
#define AD74X_REG_DIN_COMP_OUT     0x3E

#endif
