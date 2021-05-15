/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MANULYTICA_NRF_BUTTONS_C_
#define _MANULYTICA_NRF_BUTTONS_C_


#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <stdio.h>

#include <string.h>
#include <sys/printk.h>
#include <sys/byteorder.h>

// Application Files
#include "nrf_buttons.h"

struct device *led_device[4];
struct device *button_device[4];

static struct k_work button_work;

    /* TODO: create a ext counter function */
    /* needs to:
    // send a 'item sensed' SENSOR_STATUS message
    // increment counter total
    write to memory
    check total is not over max (reset)

    // Maybe button 3 & 4 send total
    // Add property ID logic to return count totals SENSOR_GET_STATUS

    */


static uint32_t button_read(struct device *port, uint32_t pin)
{
	// uint32_t val = 0U;
	// gpio_pin_read(port, pin, &val);
	// return val;

	return gpio_pin_get_raw(port, pin);
}

static void button_trigger_handler(struct k_work *work)
{
	/* Send unsolicited message of sensor readings */
	if (button_read(button_device[0], DT_ALIAS_SW0_GPIOS_PIN) == 0) {
	    send_sensor_data(SENS_PROP_ID_TEMP); // CPU Die Temperature (C)
    }
	else if (button_read(button_device[1], DT_ALIAS_SW1_GPIOS_PIN) == 0) {
	    cmd_health_fault_add(); // Add fault to register
    }
	else if (button_read(button_device[2], DT_ALIAS_SW2_GPIOS_PIN) == 0) {
	    send_sensor_data(SENS_PROP_ID_COUNTER); // Ext Item Detected (Increment count total)

	    set_sensor_data();  // Set Sensor Data function 
    }
	else if (button_read(button_device[3], DT_ALIAS_SW3_GPIOS_PIN) == 0) {
	    cmd_health_del_fault();
	    // send_sensor_data(SENS_PROP_ID_EXT_CNT_TOTAL); // Ext Item Count Total
    }
}


static void button_pressed(struct device *dev,
			   struct gpio_callback *cb, uint32_t pins)
{
	k_work_submit(&button_work);
}



void nrf_buttons_init(void)
{
	static struct gpio_callback button_cb[4];

	/* Buttons configuration & setting */
	k_work_init(&button_work, button_trigger_handler);

    button_device[0] = device_get_binding(DT_ALIAS_SW0_GPIOS_CONTROLLER);
	// gpio_pin_configure(button_device[0], DT_ALIAS_SW0_GPIOS_PIN,
	// 		   (GPIO_INPUT | GPIO_INT | GPIO_INT_EDGE |
	// 		    GPIO_PULL_UP |
	// 		    GPIO_INT_DEBOUNCE | GPIO_INT_LOW_0));

	gpio_pin_configure(button_device[0], DT_ALIAS_SW0_GPIOS_PIN,
			   GPIO_INPUT | DT_ALIAS_SW0_GPIOS_FLAGS);
	gpio_pin_interrupt_configure(button_device[0], DT_ALIAS_SW0_GPIOS_PIN,
				     GPIO_PULL_UP | GPIO_INT_DEBOUNCE |
					     GPIO_INT_LOW_0 |
					     GPIO_INT_EDGE_TO_ACTIVE);

	gpio_init_callback(&button_cb[0], button_pressed, BIT(DT_ALIAS_SW0_GPIOS_PIN));
	gpio_add_callback(button_device[0], &button_cb[0]);
	// gpio_pin_enable_callback(button_device[0], DT_ALIAS_SW0_GPIOS_PIN);

	button_device[1] = device_get_binding(DT_ALIAS_SW1_GPIOS_CONTROLLER);
	gpio_pin_configure(button_device[1], DT_ALIAS_SW1_GPIOS_PIN,
			   GPIO_INPUT | DT_ALIAS_SW1_GPIOS_FLAGS);
	gpio_pin_interrupt_configure(button_device[1], DT_ALIAS_SW1_GPIOS_PIN,
				     GPIO_PULL_UP | GPIO_INT_DEBOUNCE |
					     GPIO_INT_LOW_0 |
					     GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb[1], button_pressed, BIT(DT_ALIAS_SW1_GPIOS_PIN));
	gpio_add_callback(button_device[1], &button_cb[1]);

	button_device[2] = device_get_binding(DT_ALIAS_SW2_GPIOS_CONTROLLER);
	gpio_pin_configure(button_device[2], DT_ALIAS_SW2_GPIOS_PIN,
			   GPIO_INPUT | DT_ALIAS_SW1_GPIOS_FLAGS);
	gpio_pin_interrupt_configure(button_device[2], DT_ALIAS_SW2_GPIOS_PIN,
				     GPIO_PULL_UP | GPIO_INT_DEBOUNCE |
					     GPIO_INT_LOW_0 |
					     GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb[2], button_pressed, BIT(DT_ALIAS_SW2_GPIOS_PIN));
	gpio_add_callback(button_device[2], &button_cb[2]);

	button_device[3] = device_get_binding(DT_ALIAS_SW3_GPIOS_CONTROLLER);
	gpio_pin_configure(button_device[3], DT_ALIAS_SW3_GPIOS_PIN,
			   GPIO_INPUT | DT_ALIAS_SW1_GPIOS_FLAGS);
	gpio_pin_interrupt_configure(button_device[3], DT_ALIAS_SW3_GPIOS_PIN,
				     GPIO_PULL_UP | GPIO_INT_DEBOUNCE |
					     GPIO_INT_LOW_0 |
					     GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb[3], button_pressed, BIT(DT_ALIAS_SW3_GPIOS_PIN));
	gpio_add_callback(button_device[3], &button_cb[3]);
}

/* End of Define trap */
#endif