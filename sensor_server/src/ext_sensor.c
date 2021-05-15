/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MANULYTICA_EXT_COUNTER_C_
#define _MANULYTICA_EXT_COUNTER_C_


#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <stdio.h>

#include <string.h>
#include <misc/printk.h>
#include <misc/byteorder.h>

// Application Files
#include "ext_sensor.h"


/* Define Global Device */
static struct device *adc_dev;


/* TODO:

Read propoerty ID 

check Device active/connected
stop/disable all external devices
cfg external device
cfg external sensor
initialise external sensor

take initial ready
respornd to Sensor Setup Message
	
	*/




/**
 * strange mapping type function 
 * takes double (decimal) and recalibates to min and max range.
 * 
 * double map(double value, double in_min, double istop, double ostart, double ostop) 
 * 
 * 
 *  All values double (int32_t)
 * 
 * double value_map(x, in_min, in_max, out_min, out_max):
    return ((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

**/






int ext_sensor_cfg(struct device *dev, uint16_t property_id)
{

}


int ext_sensor_init(void)
{
	int ret = 0U;
	adc_dev = device_get_binding(ADC_DEVICE_NAME);

	if (!adc_dev) {
		printk("ERROR: Device not found - ADC DEVICE %s \n", ADC_DEVICE_NAME);
		return ret;
	}

	ret = adc_channel_setup(adc_dev, &adc_1st_channel_cfg);

	if (ret != 0) {
		printk("Setting up of the first channel failed with code %d", ret);
		return ret;
	}

#if defined(ADC_2ND_CHANNEL_ID)
	ret = adc_channel_setup(adc_dev, &adc_2nd_channel_cfg);
	if (ret != 0) {
		printk("Setting up of the second channel failed with code %d", ret);
		return ret;
	}		
#endif /* defined(ADC_2ND_CHANNEL_ID) */

	(void)memset(adc_sample_buffer, 0, sizeof(adc_sample_buffer));

	return ret;

}





static void check_samples(int expected_count)
{
	int i;

	// printk("expected_count: %d \n", expected_count);
	// printk("Samples read: \n");
	for (i = 0; i < ADC_BUFFER_SIZE; i++) {
		int16_t sample_value = adc_sample_buffer;

		printk("0x%04x \t", sample_value);
		printk("%d \t ", sample_value & 0xffff);
		if (i < expected_count) {
            	if (sample_value != 0) {
                    // printk("[%u] should be non-zero \n", i);
                }	

		} else {
            	if (sample_value == 0) {
                    // printk("[%u] should be zero \n", i);
                }	

		}
	}
	printk("\n");
}



static enum adc_action adc_with_interval_callback(
				struct device *dev,
				const struct adc_sequence *sequence,
				uint16_t sampling_index)
{
	// printk("%s: sampling %d\n", __func__, sampling_index);
	// return ADC_ACTION_CONTINUE;
	return ADC_ACTION_REPEAT;
	
}



int read_adc_with_interval(void)
{
	int ret = 0U;
	const struct adc_sequence_options options = {
		// .interval_us     = 10 * 1000UL,
		.callback        = adc_with_interval_callback,
		// .extra_samplings = 5,
	};
	const struct adc_sequence sequence = {
		.options     = &options,
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	if (!adc_dev) {
		return -1;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret != 0) {
		printk("adc_read() failed with code %d", ret);
		return ret;
	}	

	check_samples(1 + options.extra_samplings);

	return ret;
}








/* End of Define trap */
#endif