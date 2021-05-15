/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 */
#ifndef MANULYTICA_NRFTEMP_C_
#define MANULYTICA_NRFTEMP_C_

#include <zephyr.h>
#include <string.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <bluetooth/hci.h>

#include <drivers/sensor.h>


// LOG_MODULE_REGISTER(mesh_nrftemp);
// LOG_MODULE_DECLARE(nrftemp);

/* Temperature trigger for MPU die temp */
#ifdef CONFIG_NRFTEMP_TRIGGER
static void trigger_handler(struct device *dev, struct sensor_trigger *trig)
{
	struct sensor_value temp;

	sensor_sample_fetch(dev);
	sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);

	printk("trigger fired, temp %d.%06d\n", temp.val1, temp.val2);

	send_sensor_data(SENS_PROP_ID_TEMP); /* Temperature Value (C) */
}
#endif


/* Get Temperature Sensor Reading */ 
struct sensor_value mpu_temp_init(void)
{
	struct sensor_value temp;
	const struct device *temp_dev = device_get_binding("TEMP_0");
	int err;

    // If not found return nothing
	if (temp_dev == NULL) {
		printk("ERROR: Device not found - Temperature Sensor (TEMP_0) \n");
		return temp;
	}

#ifdef CONFIG_NRFTEMP_TRIGGER
	struct sensor_value val;
	struct sensor_trigger trig;

	val.val1 = 26;
	val.val2 = 0;

	sensor_attr_set(dev, SENSOR_CHAN_DIE_TEMP,
			SENSOR_ATTR_UPPER_THRESH, &val);

	trig.type = SENSOR_TRIG_THRESHOLD;
	trig.chan = SENSOR_CHAN_DIE_TEMP;

	err = (sensor_trigger_set(dev, &trig, trigger_handler)) ;
	if (err != 0) {
		printk("Could not set trigger. %d\n", err);
		return temp;
	}
#endif

	// Test to ensure I/O connection working
	// int err;
	err = sensor_sample_fetch(temp_dev);
	if (err != 0) {
		printk("ERROR: Temperature (MPU) sensor_sample_fetch error: %d\n", err);
		return temp;
	}

	err = sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &temp);
	if (err != 0) {
		printk("ERROR: Temperature (MPU) sensor_channel_get failed return: %d\n", err);
		return temp;
	}

	printk("Local Temperature (MPU): %d.%d \n",	temp.val1, temp.val2);

	return temp;
}

/* End of Define trap */
#endif