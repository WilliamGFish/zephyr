/**
 * Copyright (c) 2026 William Fish (Manulytica ltd)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_MFD_AD74X_H_
#define ZEPHYR_DRIVERS_MFD_AD74X_H_

#include <zephyr/device.h>

enum ad74x_chip_type { CHIP_AD74416H, CHIP_AD74115H };

/* Signal Type Definitions (Datasheet Table 40/42) */
#define AD74X_FUNC_HIGH_Z          0x00
#define AD74X_FUNC_VOLTAGE_OUT     0x01
#define AD74X_FUNC_CURRENT_OUT     0x02
#define AD74X_FUNC_VOLTAGE_IN      0x03
#define AD74X_FUNC_CURR_IN_EXT     0x04
#define AD74X_FUNC_CURR_IN_LOOP    0x05
#define AD74X_FUNC_DIGITAL_OUT     0x06
#define AD74X_FUNC_RTD_34WIRE      0x07
#define AD74X_FUNC_DIGITAL_IN      0x08

struct ad74x_cal_data {
	float gain;
	float offset;
};

struct ad74x_mfd_api {
	int (*transfer)(const struct device *dev, uint8_t reg, uint16_t val_in, uint16_t *val_out, bool is_read);
	uint8_t (*get_chip_type)(const struct device *dev);
	struct k_sem *(*get_adc_sem)(const struct device *dev);
	int (*save_calibration)(const struct device *dev, uint8_t chan, struct ad74x_cal_data *cal);
};

#define AD74X_CH_STRIDE              0x0C
#define AD74X_REG_CH_FUNC_SETUP(n)   (0x01 + (n * AD74X_CH_STRIDE))
#define AD74X_REG_ADC_CONV_CTRL      0x39
#define AD74X_REG_ALERT_STATUS       0x3F
#define AD74X_REG_CH_ALERT_STATUS(n) (0x58 + n)

#endif
