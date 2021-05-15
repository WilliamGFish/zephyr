/*
 * Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2018 Phytec Messtechnik GmbH
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MANULYTICA_ADPS9960_C_
#define MANULYTICA_ADPS9960_C_

#include <zephyr.h>
#include <drivers/sensor.h>
#include <stdio.h>

// #include "apds9960.h"

/* Define Global Device */
const struct device *apdsdev;

/* struct sensor_value */
void apds_get_value(uint8_t channel, struct sensor_value* light_value)
{

	if (apdsdev == NULL) {
		printk("ERROR: Could read get Light Sensor device\n");
		return;
	}

	/* Always fetch the sample to clear the data ready interrupt in the sensor. */
	if (sensor_sample_fetch(apdsdev)) {
		printk("ERROR: APDS9960 sensor_sample_fetch failed\n");
		return;
	}


sensor_channel_get(apdsdev, SENSOR_CHAN_LIGHT, light_value);
printk("ambient light intensity %f\n", sensor_value_to_double(light_value));

sensor_channel_get(apdsdev, SENSOR_CHAN_PROX, light_value);
printk("proximity %f\n", sensor_value_to_double(light_value));



	/* Take a semaphor flag */
	k_sem_take(&sem, K_NO_WAIT);

	// /* GET the readings from sensor based on passed value */
	// if 		(channel == LIGHT_ALL ){ 	/* Get X value */
	// 	sensor_channel_get(apdsdev, SENSOR_CHAN_LIGHT, light_value);
	// }
	// else if (channel == LIGHT_RED ){ 	/* Get Y value */
	// 	sensor_channel_get(apdsdev, SENSOR_CHAN_RED, light_value);
	// }
	// else if (channel == LIGHT_GREEN ){	/* Get Z value */
	// 	sensor_channel_get(apdsdev, SENSOR_CHAN_GREEN, light_value);
	// }
	// else if (channel == LIGHT_BLUE ){ /* Get channel value */
	// 	sensor_channel_get(apdsdev, SENSOR_CHAN_BLUE, light_value);
	// }
	// else if (channel == LIGHT_PROX ){ /* Get channel value */
	// 	sensor_channel_get(apdsdev, SENSOR_CHAN_PROX, light_value);
	// }
	// else {
	// 	/* Default with LIGHT_ALL Value */
	// 	sensor_channel_get(apdsdev, SENSOR_CHAN_LIGHT, light_value);
	// }

	return;
}


#ifdef CONFIG_APDS9960_TRIGGER
/* Semaphor starts "not available" */
K_SEM_DEFINE(sem, 0, 1);

static void trigger_handler(struct device *apdsdev, struct sensor_trigger *trigger)
{
	ARG_UNUSED(trigger);

	/* Always fetch the sample to clear the data ready interrupt in the sensor. */
	if (sensor_sample_fetch(apdsdev)) {
		printk("ERROR: APDS9960 Sensor_sample_fetch failed\n");
		return;
	}

	/* Show the out on console for monitoring */	
	// sensor_channel_get(apdsdev, SENSOR_CHAN_LIGHT, &intensity);
	// sensor_channel_get(apdsdev, SENSOR_CHAN_PROX, &pdata);

	// printk("APDS9960: ambient light intensity %d, proximity %d\n",
	// 		intensity.val1, pdata.val1);

	// }



	struct sensor_value accel_x, accel_y, accel_z;

	apds_get_value( ACCEL_X , &accel_x);

	printk("X: %3.2f  ", sensor_value_to_double(&accel_x));	
	printk("Y: %3.2f  ", sensor_value_to_double(&accel_y));	
	printk("Z: %3.2f \n", sensor_value_to_double(&accel_z));	

	/*  Send unsolicited request to get sensor readings
	*  Send 1 message containing all axis as a value array */
	send_sensor_data(SENS_PROP_ID_ACCEL_channel); /* Accel channel Values (m/s) */

	/* Take a thread semaphor */
	k_sem_give(&sem);
}
#endif


/* APDS-9960 Init */
void apds_init()
{
	struct sensor_value intensity, pdata;

	// apdsdev = device_get_binding(DT_AVAGO_APDS9960_0_LABEL);
	apdsdev = device_get_binding("APDS9960");

	if (apdsdev == NULL) {
		printk("ERROR: Device not found - APDS-9960 \n");
		return;
	}


#ifdef CONFIG_APDS9960_TRIGGER
	struct sensor_value attr = {
		.val1 = 127,
		.val2 = 0,
	};

	if (sensor_attr_set(apdsdev, SENSOR_CHAN_PROX,
			    SENSOR_ATTR_UPPER_THRESH, &attr)) {
		printk("ERROR: APDS9960 Could not set threshold\n");
		return;
	}

	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_THRESHOLD,
		.chan = SENSOR_CHAN_PROX,
	};

	if (sensor_trigger_set(apdsdev, &trig, trigger_handler)) {
		printk("ERROR: APDS9960 Could not set trigger\n");
		return;
	}
#endif


	if (sensor_sample_fetch(apdsdev)) {
		printk("ERROR: APDS9960 sensor_sample fetch failed\n");
	}

	sensor_channel_get(apdsdev, SENSOR_CHAN_LIGHT, &intensity);
	sensor_channel_get(apdsdev, SENSOR_CHAN_PROX, &pdata);

	printk("APDS9960: ambient light intensity %d, proximity %d\n",
			intensity.val1, pdata.val1);

}

/* End of Define trap */
#endif