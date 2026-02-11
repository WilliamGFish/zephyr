/**
 * Copyright (c) 2026 William Fish (Manulytica ltd)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT analog_ad74x_mfd
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mfd/ad74x.h>

struct ad74x_mfd_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec adc_rdy_gpio;
	enum ad74x_chip_type type;
};

struct ad74x_mfd_data {
	struct k_mutex bus_lock;
	struct k_sem adc_sync_sem;
	struct gpio_callback adc_rdy_cb;
	uint8_t frame_size;
};

/**
 * @brief CRC-8 calculation for AD74x series.
 * Polynomial: x^8 + x^2 + x^1 + 1 (0x07).
 * Reference: AD74416H Datasheet Page 72, SPI CRC section.
 */
static uint8_t ad74x_crc8(const uint8_t *p, size_t len)
{
	uint8_t crc = 0;
	for (size_t i = 0; i < len; i++) {
		crc ^= p[i];
		for (int j = 0; j < 8; j++) {
			crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
		}
	}
	return crc;
}

static void ad74x_adc_rdy_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct ad74x_mfd_data *data = CONTAINER_OF(cb, struct ad74x_mfd_data, adc_rdy_cb);
	k_sem_give(&data->adc_sync_sem);
}

/**
 * @brief Central transfer engine for all child drivers.
 * Handles the 2-stage SPI transaction required for reads.
 */
static int ad74x_mfd_transfer(const struct device *dev, uint8_t reg, uint16_t val_in,
			      uint16_t *val_out, bool is_read)
{
	const struct ad74x_mfd_config *config = dev->config;
	struct ad74x_mfd_data *data = dev->data;
	uint8_t tx[5] = {0}, rx[5] = {0};
	uint8_t read_cmd = (config->type == CHIP_AD74416H) ? 0x6E : 0x64;
	int ret;

	/* Stage 1: Write data or Send Read-Request.
	 * AD74416H (40-bit) uses a leading zero byte.
	 * AD74115H (32-bit) starts immediately with the address.
	 */    
	k_mutex_lock(&data->bus_lock, K_FOREVER);
	if (config->type == CHIP_AD74416H) {
		tx[1] = is_read ? read_cmd : reg;
		tx[2] = is_read ? reg : (val_in >> 8);
		tx[3] = is_read ? 0 : (val_in & 0xFF);
	} else {
		tx[0] = is_read ? read_cmd : reg;
		tx[1] = is_read ? reg : (val_in >> 8);
		tx[2] = is_read ? 0 : (val_in & 0xFF);
	}
	tx[data->frame_size - 1] = ad74x_crc8(tx, data->frame_size - 1);

	struct spi_buf tx_b = {.buf = tx, .len = data->frame_size};
	struct spi_buf rx_b = {.buf = rx, .len = data->frame_size};
	const struct spi_buf_set tx_s = {.buffers = &tx_buf, .count = 1},
				 rx_s = {.buffers = &rx_buf, .count = 1};

	ret = spi_transceive_dt(&config->spi, &tx_s, &rx_s);

	/* Stage 2: Data Extraction.
	 * Per Datasheet P. 71 (74115H) / P. 70 (4416H), read data is clocked out 
	 * in the transaction following the read request. We send a NOP (0x00) here.
	 */
	if (ret == 0 && is_read && val_out) {
		memset(tx, 0, 5);
		tx[frame_sz - 1] = ad74x_crc8(tx, frame_sz - 1);
		ret = spi_transceive_dt(&config->spi, &tx_s, &rx_s);
		
		/* Verify CRC of the incoming data frame */
		if (rx[frame_sz - 1] != ad74x_crc8(rx, frame_sz - 1)) {
			ret = -EIO;
		} else {
			/* Data is located in the middle bytes of the frame */
			*val_out = (rx[frame_sz - 3] << 8) | rx[frame_sz - 2];
		}
	}
	k_mutex_unlock(&data->bus_lock);
	return ret;
}

static uint8_t ad74x_get_type(const struct device *dev)
{
	return ((struct ad74x_mfd_config *)dev->config)->type;
}
static struct k_sem *ad74x_get_sem(const struct device *dev)
{
	return &((struct ad74x_mfd_data *)dev->data)->adc_sync_sem;
}

static const struct ad74x_mfd_api mfd_api = {.transfer = ad74x_mfd_transfer,
					     .get_chip_type = ad74x_get_type,
					     .get_adc_sem = ad74x_get_sem};

static int ad74x_mfd_init(const struct device *dev)
{
	const struct ad74x_mfd_config *config = dev->config;
	struct ad74x_mfd_data *data = dev->data;
	k_mutex_init(&data->bus_lock);
	k_sem_init(&data->adc_sync_sem, 0, 1);
	data->frame_size = (config->type == CHIP_AD74416H) ? 5 : 4;

	if (config->adc_rdy_gpio.port) {
		gpio_pin_configure_dt(&config->adc_rdy_gpio, GPIO_INPUT);
		gpio_init_callback(&data->adc_rdy_cb, ad74x_adc_rdy_isr,
				   BIT(config->adc_rdy_gpio.pin));
		gpio_add_callback(config->adc_rdy_gpio.port, &data->adc_rdy_cb);
		gpio_pin_interrupt_configure_dt(&config->adc_rdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	}
	return 0;
}

#define AD74X_MFD_DEFINE(n)                                                                        \
	static struct ad74x_mfd_data data_##n;                                                     \
	static const struct ad74x_mfd_config config_##n = {                                        \
		.spi = SPI_DT_SPEC_INST_GET(n, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),           \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                       \
		.adc_rdy_gpio = GPIO_DT_SPEC_INST_GET_OR(n, adc_rdy_gpios, {0}),                   \
		.type = DT_INST_ENUM_IDX(n, chip_type),                                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, ad74x_mfd_init, NULL, &data_##n, &config_##n, POST_KERNEL, 40,    \
			      &mfd_api);

DT_INST_FOREACH_STATUS_OKAY(AD74X_MFD_DEFINE)
