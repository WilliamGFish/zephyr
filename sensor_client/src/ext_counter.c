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
#include <sys/printk.h>
#include <sys/byteorder.h>

// Application Files
#include "ext_counter.h"

struct device *ext_counter_device;
static struct k_work ext_counter_work;

    /* TODO: create a ext counter function */
    /* needs to:
    // send a 'item sensed' SENSOR_STATUS message
    // increment counter total
    write to memory
    check total is not over max (reset)

    // Maybe button 3 & 4 send total
    // Add property ID logic to return count totals SENSOR_GET_STATUS

    */


// static uint32_t ext_counter_read(struct device *port, uint32_t pin)
// {
// 	uint32_t val = 0U;

// 	gpio_pin_read(port, pin, &val);
// 	return val;
// }

static void ext_counter_trigger_handler(struct k_work *work)
{

	uint32_t val, pin = 0U;	

	/* TODO: Define pin and port */	
	/* Send unsolicited message of sensor readings */
	val = gpio_pin_get(ext_counter_device, pin);

	//** if (ext_counter_read(ext_counter_device, DT_ALIAS_SW0_GPIOS_PIN) == 0) { **//
	// 	/* Send or Update count total */
	//     // send_sensor_data(SENS_PROP_ID_EXT_CNT_TOTAL); // Ext Item Count Total
    // }
}


static void ext_counter_activated(struct device *dev,
			   struct gpio_callback *cb, uint32_t pins)
{
	k_work_submit(&ext_counter_work);

}



void ext_counter_init(void)
{
	static struct gpio_callback ext_counter_cb;

	/* Buttons configuration & setting */
	k_work_init(&ext_counter_work, ext_counter_trigger_handler);

    ext_counter_device = device_get_binding(DT_ALIAS_SW0_GPIOS_CONTROLLER); /* TODO: WHAT GPIO CONTROL NEEDED */
	// gpio_pin_configure(ext_counter_device, DT_ALIAS_SW0_GPIOS_PIN,
	// 		   (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
	// 		    GPIO_PUD_PULL_UP |
	// 		    GPIO_INT_DEBOUNCE | GPIO_INT_ACTIVE_LOW));

	gpio_pin_configure(ext_counter_device, DT_ALIAS_SW0_GPIOS_PIN,
			   GPIO_INPUT | DT_ALIAS_SW1_GPIOS_FLAGS);
	gpio_pin_interrupt_configure(ext_counter_device, DT_ALIAS_SW0_GPIOS_PIN,
				     GPIO_PULL_UP | GPIO_INT_DEBOUNCE |
					     GPIO_INT_LOW_0 |
					     GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&ext_counter_cb, ext_counter_activated, BIT(DT_ALIAS_SW0_GPIOS_PIN));
	gpio_add_callback(ext_counter_device, &ext_counter_cb);
	// gpio_pin_enable_callback(ext_counter_device, DT_ALIAS_SW0_GPIOS_PIN);

}

/* End of Define trap */
#endif