/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MANULYTICA_BME280_C_
#define MANULYTICA_BME280_C_


#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>


// #ifdef CONFIG_BME280_TRIGGER
static void bme280_trigger_handler(const struct device *bmedev, struct sensor_trigger *trig)
{
	struct sensor_value temp;

	sensor_sample_fetch(bmedev);
	sensor_channel_get(bmedev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

	printf("BME280 trigger fired, temp %d.%06d\n", temp.val1, temp.val2);
}
// #endif



// void bme_init(struct k_work *work)
void bme_init()
{
	const struct device *bmedev = device_get_binding("BME280");


    // If not found return nothing
	if (bmedev == NULL) {
		printf("ERROR: Device not found - BME280 \n");
		return;
	}

	printf("bmedev %p name %s\n", bmedev, bmedev->name);



    // #ifdef CONFIG_BME280_TRIGGER -- not sure if this exists
        struct sensor_value val;
        struct sensor_trigger trig;

        //Set trigger Upper Temp in C
        val.val1 = 26;
        val.val2 = 0;

        sensor_attr_set(bmedev, SENSOR_CHAN_AMBIENT_TEMP,
                SENSOR_ATTR_UPPER_THRESH, &val);

        trig.type = SENSOR_TRIG_THRESHOLD;
        trig.chan = SENSOR_CHAN_AMBIENT_TEMP;

        if (sensor_trigger_set(bmedev, &trig, bme280_trigger_handler)) {
            printf("ERROR: BME280 Could not set trigger. \n");
            return;
        }
    // #endif




	// while (1) {
		struct sensor_value temp, press, humidity;

		sensor_sample_fetch(bmedev);
		sensor_channel_get(bmedev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(bmedev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(bmedev, SENSOR_CHAN_HUMIDITY, &humidity);

		printf("temp: %d.%06d; press: %d.%06d; humidity: %d.%06d\n",
		      temp.val1, temp.val2, press.val1, press.val2,
		      humidity.val1, humidity.val2);

	// 	k_sleep(1000);
	// }
}

/* End of Define trap */
#endif