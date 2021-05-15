/*
 * Copyright (c) 2020 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MANULYTICA_HTS221_C_
#define _MANULYTICA_HTS221_C_

// // #include <zephyr.h>
// #include <device.h>
// #include <drivers/sensor.h>
// #include <stdio.h>
// #include <sys/util.h>

#include "hts221.h"

/* Semaphor starts "not available" */
K_SEM_DEFINE(st_hts221_sem, 0, 1);	

/* Define Global Device */
const struct device *st_hts221;

void st_hts221_get_temp(struct sensor_value* temp)
{
	/* FUDGE TO NOT READING */
	// st_hts221 = device_get_binding(DT_LABEL(DT_INST(0, st_hts221)));
    st_hts221 = device_get_binding("HTS221");
	if (st_hts221 == NULL) {
		printf("ERROR: Could not read Accelometer device\n");
		return;
	}

	// /* Always fetch the sample to clear the data ready interrupt in the sensor. */
	// if (sensor_sample_fetch(st_hts221)) {
	// 	printf("ERROR: hss221 sensor_sample_fetch failed (hss221_get_accel)\n");
	// 	return;
	// }

	/* Take a semaphor flag */
	// k_sem_take(&hss221_sem, K_FOREVER);
	k_sem_take(&hss221_sem, K_NO_WAIT);

	if(sensor_channel_get(st_hts221, SENSOR_CHAN_AMBIENT_TEMP, temp)){
		printf("ERROR: st_hts221 temperature channel failed\n");
		return;
	} else {
		printf("st_hts221 Temperature Accessed: %3.3f C \n",
				sensor_value_to_double(temp));
	}

	return;
}

void st_hts221_get_hum(struct sensor_value* hum)
{
	/* FUDGE TO NOT READING */
    st_hts221 = device_get_binding("HTS221");
	if (st_hts221 == NULL) {
		printf("ERROR: Could not read Accelometer device\n");
		return;
	}

	/* Take a semaphor flag */
	// k_sem_take(&hss221_sem, K_FOREVER);
	k_sem_take(&hss221_sem, K_NO_WAIT);

	if(sensor_channel_get(st_hts221, SENSOR_CHAN_AMBIENT_TEMP, hum)){
		printf("ERROR: st_hts221 humidity channel failed\n");
		return;
	} else {
		printf("st_hts221 Humidity Accessed: %3.3f C \n",
				sensor_value_to_double(hum));
	}

	return;
}


static void st_hts221_handler(const struct device *st_hts221,
			   struct sensor_trigger *trig)
{
	/* TODO: */
}

/* st_hts221 Initilisation */
void st_hts221_init(void)
{
    struct sensor_value temp, hum;
	st_hts221 = device_get_binding("HTS221");

	if (st_hts221 == NULL) {
		printf("Could not get st_hts221 device\n");
		return;
	}

	if (IS_ENABLED(CONFIG_st_hts221_TRIGGER)) {
		struct sensor_trigger trig = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
		};
		if (sensor_trigger_set(st_hts221, &trig, st_hts221_handler) < 0) {
			printf("Cannot configure trigger\n");
			return;
		};
	}

	if (sensor_channel_get(st_hts221, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printf("Cannot read st_hts221 temperature channel\n");
		return;
	}

	if (sensor_channel_get(st_hts221, SENSOR_CHAN_HUMIDITY, &hum) < 0) {
		printf("Cannot read st_hts221 humidity channel\n");
		return;
	}

	printf("Observation:\n");

	/* display temperature */
	printf("Temperature:%.1f C\n", sensor_value_to_double(&temp));

	/* display humidity */
	printf("Relative Humidity:%.1f%%\n",
	       sensor_value_to_double(&hum));

}

#endif