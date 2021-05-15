/*
 * Copyright (c) 2020 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MANULYTICA_LIS2DH_C_
#define _MANULYTICA_LIS2DH_C_

#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>

#include "lis2dh.h"

/* Semaphor starts "not available" */
K_SEM_DEFINE(lis2dh_sem, 0, 1);	

/* Define Global Device */
const struct device *st_lis2dh;

void lis2dh_get_accel(uint8_t xyz, struct sensor_value* accel_value)
{
	// /* Stuff values for 'error trapping?' */	
	// accel_value[0].val1 = 0;
	// accel_value[1].val1 = 0;
	// accel_value[2].val1 = 0;

	/* FUDGE TO NOT READING */
	st_lis2dh = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dh)));
	if (st_lis2dh == NULL) {
		printf("ERROR: Could not read Accelometer device\n");
		return;
	}

	/* Take a semaphor flag */
	// k_sem_take(&lis2dh_sem, K_FOREVER);
	k_sem_take(&lis2dh_sem, K_NO_WAIT);
	
	if(sensor_channel_get(st_lis2dh,
			SENSOR_CHAN_ACCEL_XYZ, 
			accel_value)){
		printf("ERROR: LIS2DH Read Sensor failed (lis2dh_get_accel)\n");
		return;
	} else {
		printf("LIS Accessed: %d\nAX:%3.3f AY:%3.3f AZ:%3.3f \n\n",
				xyz,
				sensor_value_to_double(&accel_value[0]),
				sensor_value_to_double(&accel_value[1]),
				sensor_value_to_double(&accel_value[2]));
	}

	// /* GET the readings from sensor based on passed value */
	// if 		(xyz == ACCEL_X ){ 	/* Get X value */
	// 	sensor_channel_get(st_lis2dh, SENSOR_CHAN_ACCEL_X, accel_value);
	// }
	// else if (xyz == ACCEL_Y ){ 	/* Get Y value */
	// 	sensor_channel_get(st_lis2dh, SENSOR_CHAN_ACCEL_Y, accel_value);
	// }
	// else if (xyz == ACCEL_Z ){	/* Get Z value */
	// 	sensor_channel_get(st_lis2dh, SENSOR_CHAN_ACCEL_Z, accel_value);
	// }
	// else if (xyz == ACCEL_XYZ ){ /* Get XYZ value */
	// 	sensor_channel_get(st_lis2dh, SENSOR_CHAN_ACCEL_XYZ, accel_value);
	// }
	// else {
	// 	/* Default with X Value (MAYBE XYZ???) */
	// 	sensor_channel_get(st_lis2dh, SENSOR_CHAN_ACCEL_X;, accel_value);
	// }

	return;
}


static void trigger_handler(const struct device *lis_dev,
			    struct sensor_trigger *trigger)
{
	/*TODO: */
	// ARG_UNUSED(trigger);

	switch (trigger->type) {
	case SENSOR_TRIG_DATA_READY:
		if (sensor_sample_fetch(lis_dev)) {
			printf("Sample fetch error\n");
			return;
		}
		k_sem_give(&lis2dh_sem);
		break;
	case SENSOR_TRIG_DELTA:
		printf("Trigger Delta trigger\n");
		break;
	default:
		printf("Unknown trigger\n");
	}


	// printk(" *******************************TRIGGERED LIS2DH****************************\n\n\n\n\n");
	// /* Always fetch the sample to clear the data ready interrupt in the sensor. */
	// if (sensor_sample_fetch(lis_dev)) {
	// 	printf("ERROR: st_lis2dh Sensor_sample_fetch failed (trigger_handler)\n");
	// 	return;
	// }

	/* Show the out on console for monitoring */	
	struct sensor_value accel[3];

	if(sensor_channel_get(st_lis2dh,
			SENSOR_CHAN_ACCEL_XYZ, 
			accel)){
		printf("ERROR: LIS2DH Sensor_sample_fetch failed (sensor_channel_get)\n");
		return;
	} else {
		printf("LIS Triggered\nAX:%3.3f AY:%3.3f AZ:%3.3f \n\n",
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]));
	}
	/*  Send unsolicited request to get sensor readings
	*  Send 1 message containing all axis as a value array */
	// send_sensor_data(SENS_PROP_ID_ACCEL_XYZ); /* Accel XYZ Values (m/s) */

	/* Take a thread semaphor */
	k_sem_give(&lis2dh_sem);	
}

void lis2_init(void)
{
	struct sensor_value accel[3];
	st_lis2dh = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dh)));

	if (st_lis2dh == NULL) {
		printf("ERROR:  Device not found - %s device\n",
		       DT_LABEL(DT_INST(0, st_lis2dh)));
		return;
	}

if (IS_ENABLED(CONFIG_LIS2DH_TRIGGER)){
		struct sensor_trigger trig;
		int rc;

		/* Set MOTION Trigger value */
		trig.type = SENSOR_TRIG_DELTA;
		trig.chan = SENSOR_CHAN_ACCEL_XYZ;

		if (sensor_attr_set(st_lis2dh, trig.chan,
					SENSOR_ATTR_SLOPE_TH, &lis2dh_attr_trigger)) {
			printk("ERROR: LIS2DH Could not set slope threshold\n");
			return;
		}	

		printf("Slope threshold at %u \n", lis2dh_attr_trigger.val1);	

		if (sensor_attr_set(st_lis2dh, trig.chan,
					SENSOR_ATTR_SLOPE_DUR, &lis2dh_attr_trig_dur)) {
			printk("ERROR: LIS2DH Could not set slope threshold\n");
			return;
		}	

		printf("Duration threshold at %u \n", lis2dh_attr_trig_dur.val1);	


		rc = sensor_trigger_set(st_lis2dh, &trig, trigger_handler);
		if (rc != 0) {
		printf("Failed to set trigger: %d\n", rc);
		}

		if (IS_ENABLED(CONFIG_LIS2DH_ODR_RUNTIME)) {

			// trig.type = SENSOR_TRIG_DATA_READY;
			// trig.chan = SENSOR_CHAN_ACCEL_XYZ;

			// rc = sensor_attr_set(st_lis2dh, trig.chan,
			// 		     SENSOR_ATTR_SAMPLING_FREQUENCY,
			// 		     &lis2dh_odr);
			// if (rc != 0) {
			// 	printf("Failed to set lis2dh_odr: %d\n", rc);
			// 	return;
			// }
			// printf("Sampling at %u Hz\n", lis2dh_odr.val1);	

			// rc = sensor_trigger_set(st_lis2dh, &trig, trigger_handler);
			// if (rc != 0) {
			// 	printf("Failed to set trigger: %d\n", rc);
			// 	return;
			// }				
		}
		

	}
// #endif /* CONFIG_LIS2DH_TRIGGER */

	/* Always fetch the sample to clear the data ready interrupt in the sensor. */
	if (sensor_sample_fetch(st_lis2dh)) {
		printf("ERROR: LIS2DH Sensor_sample_fetch failed (sensor_sample_fetch)\n");
		return;
	}

	if(sensor_channel_get(st_lis2dh,
			SENSOR_CHAN_ACCEL_XYZ, 
			accel)){
		printf("ERROR: LIS2DH Read Sensor failed (sensor_channel_get)\n");
		return;
	} else {
		printf("AX=%10.6f AY=%10.6f AZ=%10.6f ",
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]));
	}

	printf("\n");

	// /* Always fetch the sample to clear the data ready interrupt in the sensor. */
	// if (sensor_sample_fetch(st_lis2dh)) {
	// 	printf("ERROR: LIS2DH Sensor_sample_fetch failed\n");
	// }
}

/* End of Define trap */
#endif
