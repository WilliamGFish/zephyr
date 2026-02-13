/**
 * Copyright (c) 2026 William Fish (Manulytica ltd)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT analog_ad74x_adc
#include <math.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/mfd/ad74x.h>

/* Callendar-Van Dusen for Pt1000 (Datasheet P. 47) */
static float ad74x_rtd_math(uint32_t raw_adc)
{
	float r_ref = 2100.0f; // Internal reference + sense
	float resistance = ((float)raw_adc / 16777216.0f) * r_ref;
	// Standard Pt1000: t = (R - 1000) / 3.85
	return (resistance - 1000.0f) / 3.85f;
}

static int adc_ad74x_channel_setup(const struct device *dev, const struct adc_channel_cfg *cfg)
{
	const struct device *mfd = dev->parent;
	const struct ad74x_mfd_api *api = mfd->api;

	/* Map standard acquisition time to hardware modes */
	uint16_t mode = (cfg->acquisition_time == ADC_ACQ_TIME(1, ADC_ACQ_TIME_TICKS))
				? AD74X_MODE_CURR_IN_LOOP
				: AD74X_MODE_VOLTAGE_IN;
	if (cfg->acquisition_time == ADC_ACQ_TIME(2, ADC_ACQ_TIME_TICKS)) {
		mode = AD74X_MODE_RTD_34WIRE;
	}

	return api->transfer(mfd, AD74X_REG_CH_FUNC_SETUP(cfg->channel_id), mode, NULL, false);
}

static int adc_ad74x_read(const struct device *dev, const struct adc_sequence *seq)
{
	const struct device *mfd = dev->parent;
	const struct ad74x_mfd_api *api = mfd->api;
	uint8_t ch = find_lsb_set(seq->channels) - 1;
	uint8_t type = api->get_chip_type(mfd);
	struct ad74x_cal_data cal;

	/* Trigger conversion: Set Start bit (8) and Enable specific Channel bit.
	 * AD74115H Register: 0x3B (Table 41)
	 * AD74416H Register: 0x39 (Table 40)
	 */
	uint8_t ctrl_reg = (type == CHIP_AD74115H) ? 0x3B : 0x39;
	api->transfer(mfd, ctrl_reg, BIT(8) | BIT(ch), NULL, false);

	/* Wait for physical ADC_RDY pin to trigger the MFD parent's ISR.
	 * This prevents blocking the SPI bus with busy-wait polling.
	 */
	if (k_sem_take(api->get_adc_sem(mfd), K_MSEC(200)) != 0) {
		return -EIO;
	}

	if (type == CHIP_AD74416H) {
		/* AD74416H has a 24-bit ADC. Results are split across two 16-bit registers.
		 * UPR (Upper) contains bits [23:16]. LWR (Lower) contains [15:0].
		 * Ref: AD74416H Datasheet Table 61.
		 */
		uint16_t u, l;
		api->transfer(mfd, 0x41 + (ch * 2), 0, &u, true);
		api->transfer(mfd, 0x42 + (ch * 2), 0, &l, true);
		raw = ((u & 0xFF) << 16) | l;
	} else {
		/* AD74115H has a 16-bit ADC in a single register (0x44). */
		uint16_t val;
		api->transfer(mfd, 0x44, 0, &val, true);
		raw = val;
	}

	/* Apply Calibration */
	api->get_calibration(mfd, ch, &cal);
	float val_f = (float)raw * cal.gain + cal.offset;

	/* RTD Math if needed */
	// Note: In production, check channel mode before applying RTD math
	// float R = (val_f / 16777216.0f) * 2100.0f;
	// val_f = (R - 1000.0f) / 3.85f;

	*((uint32_t *)seq->buffer) = (uint32_t)val_f;

	return 0;
}

static const struct adc_driver_api adc_api = {.read = adc_ad74x_read,
					      .channel_setup = adc_ad74x_channel_setup};
#define AD74X_ADC_DEFINE(n)                                                                        \
	DEVICE_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL, 50, &adc_api);
DT_INST_FOREACH_STATUS_OKAY(AD74X_ADC_DEFINE)
