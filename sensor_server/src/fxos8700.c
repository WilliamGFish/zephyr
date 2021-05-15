/*
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright (c) 2018 Phytec Messtechnik GmbH
 * Copyright (c) 2019 Manulytica Ltd 
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MANULYTICA_FXOS8700_C_
#define MANULYTICA_FXOS8700_C_

#include <zephyr.h>
#include <drivers/sensor.h>
#include <stdio.h>

#include "fxos8700.h"

/* Semaphor starts "not available" */
K_SEM_DEFINE(sem, 0, 1);	

/* Define Global Device */
// static struct device *dev;
const struct device *fxdev;

/* struct sensor_value */
void fxos_get_accel(uint8_t xyz, struct sensor_value* accel_value)
{

/* FUDGE TO NOT READING */
// fxdev = device_get_binding(DT_INST_0_NXP_FXOS8700_LABEL);	
// fxdev = device_get_binding("MMA8653FC");

	if (fxdev == NULL) {
		printf("ERROR: Could not read Accelometer device\n");
		return;
	}

	/* Always fetch the sample to clear the data ready interrupt in the sensor. */
	if (sensor_sample_fetch(fxdev)) {
		printf("ERROR: FXOS8700 sensor_sample_fetch failed\n");
		return;
	}

	/* Take a semaphor flag */
	// k_sem_take(&sem, K_FOREVER);
	k_sem_take(&sem, K_NO_WAIT);

	/* GET the readings from sensor based on passed value */
	if 		(xyz == ACCEL_X ){ 	/* Get X value */
		sensor_channel_get(fxdev, SENSOR_CHAN_ACCEL_X, accel_value);
	}
	else if (xyz == ACCEL_Y ){ 	/* Get Y value */
		sensor_channel_get(fxdev, SENSOR_CHAN_ACCEL_Y, accel_value);
	}
	else if (xyz == ACCEL_Z ){	/* Get Z value */
		sensor_channel_get(fxdev, SENSOR_CHAN_ACCEL_Z, accel_value);
	}
	else if (xyz == ACCEL_XYZ ){ /* Get XYZ value */
		sensor_channel_get(fxdev, SENSOR_CHAN_ACCEL_XYZ, accel_value);
	}
	else {
		/* Default with X Value (MAYBE XYZ???) */
		sensor_channel_get(fxdev, SENSOR_CHAN_ACCEL_X, accel_value);
	}

	return;
}


// struct sensor_value fxos_get_mag(uint8_t xyz)
void fxos_get_mag(uint8_t xyz, struct sensor_value* magn_value)
{
	/* Use Global Device (fxdev) */
	if (fxdev == NULL) {
		printf("ERROR: Could not get magnometer device\n");
		return;
	}

	#if defined(CONFIG_FXOS8700_MODE_MAGN) || defined(CONFIG_FXOS8700_MODE_HYBRID)
		/* Take a semaphor flag */
		k_sem_take(&sem, K_NO_WAIT);

		sensor_channel_get(fxdev, SENSOR_CHAN_MAGN_XYZ, magn);
		/* Print mag x,y,z data */
		printf("MX=%10.6f MY=%10.6f MZ=%10.6f ",
			sensor_value_to_double(&magn_value[0]),
			sensor_value_to_double(&magn_value[1]),
			sensor_value_to_double(&magn_value[2]));


		/* Use the data selection passed */
		switch (xyz) {
		case 1: /* Get X value */
			sensor_channel_get(fxdev, SENSOR_CHAN_MAGN_X, &magn_value);
		break;
		case 2:  /* Get Y value */
			sensor_channel_get(fxdev, SENSOR_CHAN_MAGN_Y, &magn_value);
		break;
		case 3: /* Get Z value */
			sensor_channel_get(fxdev, SENSOR_CHAN_MAGN_Z, &magn_value);
		break;
		default: /* Default with X Value (MAYBE XYZ???) */
			sensor_channel_get(fxdev, SENSOR_CHAN_ACCEL_X, &magn_value);
		}
	#else
		printf("ERROR: NO Magnometer device defined\n");
	#endif

	return;
}


#if !defined(CONFIG_FXOS8700_TRIGGER_NONE)
static void fxos_trigger_handler(const struct device *fxdev, struct sensor_trigger *trigger)
{
	ARG_UNUSED(trigger);

	/* Always fetch the sample to clear the data ready interrupt in the sensor. */
	if (sensor_sample_fetch(fxdev)) {
		printf("ERROR: Sensor_sample_fetch failed\n");
		return;
	}

	/* Show the out on console for monitoring */	
	struct sensor_value accel_x, accel_y, accel_z;

	fxos_get_accel( ACCEL_X , &accel_x);
	fxos_get_accel( ACCEL_Y , &accel_y);
	fxos_get_accel( ACCEL_Z , &accel_z);

	printf("X: %3.3f  ", sensor_value_to_double(&accel_x));	
	printf("Y: %3.3f  ", sensor_value_to_double(&accel_y));	
	printf("Z: %3.3f \n", sensor_value_to_double(&accel_z));	

	/*  Send unsolicited request to get sensor readings
	*  Send 1 message containing all axis as a value array */
	send_sensor_data(SENS_PROP_ID_ACCEL_XYZ); /* Accel XYZ Values (m/s) */

	/* Take a thread semaphor */
	k_sem_give(&sem);
}
#endif /* !CONFIG_FXOS8700_TRIGGER_NONE */

/* FXOS8700 Init */
// void fxos_init(struct k_work *work)
void fxos_init()
{
	struct sensor_value accel[3];
	// struct device *fxdev = device_get_binding(CONFIG_FXOS8700_NAME);
	
	// fxdev = device_get_binding(DT_LABEL(DT_INST(0, nxp_fxos8700)));
	fxdev = device_get_binding("MMA8653FC");

	if (fxdev == NULL) {
		printf("ERROR: Device not found - MMA8653FC/FXOS8700 \n");
		return;
	}
	


#ifdef CONFIG_FXOS8700_MOTION
	// /* Trigger value */
	// attr.val1 = 10;
	// attr.val2 = 800000;
	if (sensor_attr_set(fxdev, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SLOPE_TH, &attr_trigger)) {
		printk("ERROR: FXOS8700 Could not set slope threshold\n");
		return;
	}
#endif

#if !defined(CONFIG_FXOS8700_TRIGGER_NONE)
	struct sensor_trigger trig = {
#ifdef CONFIG_FXOS8700_MOTION
		.type = SENSOR_TRIG_DELTA,
#else
		.type = SENSOR_TRIG_DATA_READY,
#endif
		.chan = SENSOR_CHAN_ACCEL_XYZ,
	};

	if (sensor_trigger_set(fxdev, &trig, fxos_trigger_handler)) {
		printf("ERROR: FXOS8700 Could not set trigger\n");
		return;
	}
#endif /* !CONFIG_FXOS8700_TRIGGER_NONE */

	/* Always fetch the sample to clear the data ready interrupt in the sensor. */
	if (sensor_sample_fetch(fxdev)) {
		printf("ERROR: FXOS8700 Sensor_sample_fetch failed\n");
		return;
	}

	sensor_channel_get(fxdev, SENSOR_CHAN_ACCEL_XYZ, accel);
	/* Print accel x,y,z data */
	printf("AX=%10.6f AY=%10.6f AZ=%10.6f ",
			sensor_value_to_double(&accel[0]),
			sensor_value_to_double(&accel[1]),
			sensor_value_to_double(&accel[2]));
	#if defined(CONFIG_FXOS8700_MODE_MAGN) || defined(CONFIG_FXOS8700_MODE_HYBRID)
			struct sensor_value magn[3];

			sensor_channel_get(fxdev, SENSOR_CHAN_MAGN_XYZ, magn);
			/* Print mag x,y,z data */
			printf("MX=%10.6f MY=%10.6f MZ=%10.6f ",
				sensor_value_to_double(&magn[0]),
				sensor_value_to_double(&magn[1]),
				sensor_value_to_double(&magn[2]));
	#endif
	#ifdef CONFIG_FXOS8700_TEMP
			struct sensor_value temp;

			sensor_channel_get(fxdev, SENSOR_CHAN_DIE_TEMP, &temp);
			/* Print accel x,y,z and mag x,y,z data */
			printf("T=%10.6f", sensor_value_to_double(&temp));
	#endif

	printf("\n");

	/* Always fetch the sample to clear the data ready interrupt in the sensor. */
	if (sensor_sample_fetch(fxdev)) {
		printf("ERROR: FXOS8700 sensor_sample_fetch failed\n");
		return;
	}
		
}

/* End of Define trap */
#endif