/*
 * Copyright (c) 2026 William Fish (Manulytica ltd)
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_DRIVERS_MFD_AD74X_H_
#define ZEPHYR_DRIVERS_MFD_AD74X_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

enum ad74x_chip_type {
	CHIP_AD74416H,
	CHIP_AD74115H
};

/* Datasheet CH_FUNC_SETUP modes (Table 40/42) */
#define AD74X_MODE_HIGH_Z       0x00
#define AD74X_MODE_VOLTAGE_OUT  0x01
#define AD74X_MODE_CURRENT_OUT  0x02
#define AD74X_MODE_VOLTAGE_IN   0x03
#define AD74X_MODE_CURR_IN_EXT  0x04
#define AD74X_MODE_CURR_IN_LOOP 0x05
#define AD74X_MODE_DIGITAL_OUT  0x06
#define AD74X_MODE_RTD_34WIRE   0x07
#define AD74X_MODE_DIGITAL_IN   0x08

/* Alert Status Bitmask (Datasheet P. 100) */
#define AD74X_ALERT_HI_TEMP BIT(4)
#define AD74X_ALERT_CH_SC   BIT(13)
#define AD74X_ALERT_CH_OC   BIT(14)

struct ad74x_cal_data {
	float gain;
	float offset;
};

struct ad74x_mfd_api {
	int (*transfer)(const struct device *dev, uint8_t reg, uint16_t val_in, uint16_t *val_out,
			bool is_read);
	uint8_t (*get_chip_type)(const struct device *dev);
	struct k_sem *(*get_adc_sem)(const struct device *dev);
	int (*save_calibration)(const struct device *dev, uint8_t chan, struct ad74x_cal_data *cal);
	int (*get_calibration)(const struct device *dev, uint8_t chan, struct ad74x_cal_data *cal);
};

#define AD74X_CH_STRIDE            0x0C
#define AD74X_REG_CH_FUNC_SETUP(n) (0x01 + (n * AD74X_CH_STRIDE))
#define AD74X_REG_ADC_CONV_CTRL    0x39
#define AD74X_REG_DIN_COMP_OUT     0x3E

/* HART Register Offsets (Stride 0x10) */
#define AD74X_HART_STRIDE     0x10
#define AD74X_HART_REG_RX(n)  (0x81 + (n * AD74X_HART_STRIDE))
#define AD74X_HART_REG_TX(n)  (0x82 + (n * AD74X_HART_STRIDE))
#define AD74X_HART_REG_MCR(n) (0x84 + (n * AD74X_HART_STRIDE))
#define AD74X_HART_REG_RFC(n) (0x85 + (n * AD74X_HART_STRIDE))

#endif /* ZEPHYR_DRIVERS_MFD_AD74X_H_ */
