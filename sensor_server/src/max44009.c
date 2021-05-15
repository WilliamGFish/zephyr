/*
*  Copyright (c) 2012-2014 Wind River Systems, Inc.
*
*  Copyright (c) 2019 Manulytica Ltd
*  
*  SPDX-License-Identifier Apache-2.0
* MAX44009 light sensor through I2C. 
*/
#ifndef MANULYTICA_MAX44009_C_
#define MANULYTICA_MAX44009_C_

 
#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>


// #ifdef CONFIG_MAX44009_TRIGGER
static void max44009_trigger_handler(const struct device *maxdev, struct sensor_trigger *trig)
{
	struct sensor_value lum;

	sensor_sample_fetch(maxdev);
	sensor_channel_get(maxdev, SENSOR_CHAN_LIGHT, &lum);

	printk("MAX44009 trigger fired, lum %d.%06d\n", lum.val1, lum.val2);
}
// #endif



// void bme_init(struct k_work *work)
void max_init()
{
	const struct device *maxdev = device_get_binding("max44009");

    // If not found return nothing
	if (maxdev == NULL) {
		printk("ERROR: Device not found - MAX44009 \n");
		return;
	}

	printk("maxdev %p name %s\n", maxdev, maxdev->name);

    // #ifdef CONFIG_MAX44009_TRIGGER -- not sure if this exists
        struct sensor_value val;
        struct sensor_trigger trig;

        //Set trigger Upper Temp in C
        val.val1 = 26;
        val.val2 = 0;

        sensor_attr_set(maxdev, SENSOR_CHAN_LIGHT,
                SENSOR_ATTR_UPPER_THRESH, &val);

        trig.type = SENSOR_TRIG_THRESHOLD;
        trig.chan = SENSOR_CHAN_LIGHT;

        if (sensor_trigger_set(maxdev, &trig, max44009_trigger_handler)) {
            printk("ERROR: max44009 Could not set trigger. \n");
            return;
        }
    // #endif

    struct sensor_value lum ;

    sensor_sample_fetch(maxdev);

    if (sensor_sample_fetch_chan(maxdev, SENSOR_CHAN_LIGHT) != 0) {
        printk("ERROR: max44009 sensor sample fetch failed \n");
        return;
    }

    if (sensor_channel_get(maxdev, SENSOR_CHAN_LIGHT, &lum) != 0) {
        printk("ERROR: max44009 sensor channel get failed \n");
        return;
    }

	printk("sensor lum reading %d.%06d /n", lum.val1, lum.val2);            
}

/* End of Define trap */
#endif