/**
 * Copyright (c) 2026 William Fish (Manulytica ltd)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT analog_ad74x_mfd
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mfd/ad74x.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(AD74X_MFD, CONFIG_MFD_LOG_LEVEL);

struct ad74x_mfd_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec adc_rdy_gpio;
	struct gpio_dt_spec alert_gpio;
	enum ad74x_chip_type type;
	const struct flash_area *nvs_flash;
	uint32_t partition_id;
	uint8_t wdt_val;
};

struct ad74x_mfd_data {
	struct k_mutex bus_lock;
	struct k_sem adc_sync_sem;
	struct gpio_callback adc_rdy_cb;
	struct gpio_callback alert_cb;
	struct k_work alert_work;
	const struct device *dev;
	struct nvs_fs fs;
	struct ad74x_cal_data cal[4];
	bool nvs_ready;
};

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

static void ad74x_alert_worker(struct k_work *work)
{
	struct ad74x_mfd_data *data = CONTAINER_OF(work, struct ad74x_mfd_data, alert_work);
	uint16_t status;
	uint8_t reg = (data->dev->config->type == CHIP_AD74115H) ? 0x41 : 0x3F;

	const struct ad74x_mfd_api *api = data->dev->api;
	api->transfer(data->dev, reg, 0, &status, true);

	if (status & AD74X_ALERT_HI_TEMP) {
		LOG_ERR("AD74x: Over-Temperature Fault (>115C)");
	}
	if (status & AD74X_ALERT_CH_SC) {
		LOG_ERR("AD74x: Short Circuit Fault on Screw Terminal");
	}

	/* Clear W1C */
	api->transfer(data->dev, reg, status, NULL, false);
}

static void ad74x_adc_rdy_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct ad74x_mfd_data *data = CONTAINER_OF(cb, struct ad74x_mfd_data, adc_rdy_cb);
	k_sem_give(&data->adc_sync_sem);
}

static void ad74x_alert_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct ad74x_mfd_data *data = CONTAINER_OF(cb, struct ad74x_mfd_data, alert_cb);
	k_work_submit(&data->alert_work);
}

int ad74x_mfd_transfer(const struct device *dev, uint8_t reg, uint16_t val_in, uint16_t *val_out,
		       bool is_read)
{
	const struct ad74x_mfd_config *config = dev->config;
	struct ad74x_mfd_data *data = dev->data;
	uint8_t frame_sz = (config->type == CHIP_AD74416H) ? 5 : 4;
	uint8_t read_cmd = (config->type == CHIP_AD74416H) ? 0x6E : 0x64;
	uint8_t tx[5] = {0}, rx[5] = {0};

	k_mutex_lock(&data->bus_lock, K_FOREVER);

	/* Frame Format:
	 * AD74416H (Quad): [CMD][REG][DATA_H][DATA_L][CRC]
	 * AD74115H (Single): [CMD][DATA_H][DATA_L][CRC]
	 * CMD: 0x6E for Read, 0x00-0x3F for Write (Reg Addr in bits [5:0])
	 */
	if (config->type == CHIP_AD74416H) {
		tx[1] = is_read ? read_cmd : reg;
		tx[2] = is_read ? reg : (val_in >> 8);
		tx[3] = is_read ? 0 : (val_in & 0xFF);
	} else {
		tx[0] = is_read ? read_cmd : reg;
		tx[1] = is_read ? reg : (val_in >> 8);
		tx[2] = is_read ? 0 : (val_in & 0xFF);
	}
	tx[frame_sz - 1] = ad74x_crc8(tx, frame_sz - 1);

	struct spi_buf tx_buf = {.buf = tx, .len = frame_sz};
	struct spi_buf rx_buf = {.buf = rx, .len = frame_sz};
	const struct spi_buf_set tx_s = {.buffers = &tx_buf, .count = 1},
				 rx_s = {.buffers = &rx_buf, .count = 1};

	int ret = spi_transceive_dt(&config->spi, &tx_s, &rx_s);
	if (ret == 0 && is_read && val_out) {
		memset(tx, 0, 5);
		tx[frame_sz - 1] = ad74x_crc8(tx, frame_sz - 1);
		spi_transceive_dt(&config->spi, &tx_s, &rx_s);
		*val_out = (rx[frame_sz - 3] << 8) | rx[frame_sz - 2];
	}
	k_mutex_unlock(&data->bus_lock);
	return ret;
}

static int ad74x_mfd_save_calibration(const struct device *dev, uint8_t chan,
				      struct ad74x_cal_data *cal)
{
	struct ad74x_mfd_data *data = dev->data;
	if (!data->nvs_ready) {
		return -ENOTSUP;
	}
	data->cal[chan] = *cal;
	return nvs_write(&data->fs, chan, cal, sizeof(struct ad74x_cal_data));
}

static int ad74x_mfd_get_calibration(const struct device *dev, uint8_t chan,
				     struct ad74x_cal_data *cal)
{
	struct ad74x_mfd_data *data = dev->data;
	*cal = data->cal[chan];
	return 0;
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
					     .get_adc_sem = ad74x_get_sem,
					     .save_calibration = ad74x_mfd_save_calibration,
					     .get_calibration = ad74x_mfd_get_calibration};

static int ad74x_mfd_init(const struct device *dev)
{
	const struct ad74x_mfd_config *config = dev->config;
	struct ad74x_mfd_data *data = dev->data;
	data->dev = dev;
	k_mutex_init(&data->bus_lock);
	k_sem_init(&data->adc_sync_sem, 0, 1);
	k_work_init(&data->alert_work, ad74x_alert_worker);

	/* Initialize NVS */
	if (config->partition_id) {
		data->fs.flash_device = FLASH_AREA_DEVICE(config->partition_id);
		if (device_is_ready(data->fs.flash_device)) {
			data->fs.offset = FLASH_AREA_OFFSET(config->partition_id);
			struct flash_pages_info info;
			flash_get_page_info_by_offs(data->fs.flash_device, data->fs.offset, &info);
			data->fs.sector_size = info.size;
			data->fs.sector_count = 1;
			if (nvs_mount(&data->fs) == 0) {
				data->nvs_ready = true;
				for (int i = 0; i < 4; i++) {
					/* Load or set defaults */
					if (nvs_read(&data->fs, i, &data->cal[i],
						     sizeof(struct ad74x_cal_data)) <= 0) {
						data->cal[i].gain = 1.0f;
						data->cal[i].offset = 0.0f;
					}
				}
			}
		}
	}

	/* Watchdog Config */
	if (config->wdt_val > 0) {
		ad74x_mfd_transfer(dev, 0x3B, (0x01 << 4) | config->wdt_val, NULL, false);
	}

	if (config->adc_rdy_gpio.port) {
		gpio_pin_configure_dt(&config->adc_rdy_gpio, GPIO_INPUT);
		gpio_init_callback(&data->adc_rdy_cb, ad74x_adc_rdy_isr,
				   BIT(config->adc_rdy_gpio.pin));
		gpio_add_callback(config->adc_rdy_gpio.port, &data->adc_rdy_cb);
		gpio_pin_interrupt_configure_dt(&config->adc_rdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	}
	if (config->alert_gpio.port) {
		gpio_pin_configure_dt(&config->alert_gpio, GPIO_INPUT);
		gpio_init_callback(&data->alert_cb, ad74x_alert_isr, BIT(config->alert_gpio.pin));
		gpio_add_callback(config->alert_gpio.port, &data->alert_cb);
		gpio_pin_interrupt_configure_dt(&config->alert_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	}
	return 0;
}

#define AD74X_MFD_DEFINE(n)                                                                        \
	static struct ad74x_mfd_data data_##n;                                                     \
	static const struct ad74x_mfd_config config_##n = {                                        \
		.spi = SPI_DT_SPEC_INST_GET(n, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),           \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),                       \
		.adc_rdy_gpio = GPIO_DT_SPEC_INST_GET_OR(n, adc_rdy_gpios, {0}),                   \
		.alert_gpio = GPIO_DT_SPEC_INST_GET_OR(n, alert_gpios, {0}),                       \
		.type = DT_INST_ENUM_IDX(n, chip_type),                                            \
		.partition_id = DT_FIXED_PARTITION_ID(DT_INST_PHANDLE(n, partition)),              \
		.wdt_val = DT_INST_PROP_OR(n, wdt_timeout, 0),                                     \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, ad74x_mfd_init, NULL, &data_##n, &config_##n, POST_KERNEL, 40,    \
			      &mfd_api);

DT_INST_FOREACH_STATUS_OKAY(AD74X_MFD_DEFINE)
