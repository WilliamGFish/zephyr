/* microbit.c - BBC micro:bit specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MANULYTICA_MICROBIT_C_
#define MANULYTICA_MICROBIT_C_

#include <board.h> //Not in v1.14
#include <soc.h>
#include <sys/printk.h>
#include <ctype.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>

#include <display/mb_display.h>

#include <bluetooth/mesh.h>


// Include Application Files
#include "board.h"
// #include "mesh.h"
// #include "mesh_sensor.c"


#define SCROLL_SPEED   K_MSEC(300)

#define BUZZER_PIN     EXT_P0_GPIO_PIN
#define BEEP_DURATION  K_MSEC(60)

#define SEQ_PER_BIT  976
#define SEQ_PAGE     (NRF_FICR->CODEPAGESIZE * (NRF_FICR->CODESIZE - 1))
#define SEQ_MAX      (NRF_FICR->CODEPAGESIZE * 8 * SEQ_PER_BIT)

static struct device *gpio;
static struct device *nvm;

static struct k_work button_work;

static void button_send_pressed(struct k_work *work)
{
	// printk("button_send_pressed()\n");
	board_button_1_pressed();
}

static void button_pressed(struct device *dev, struct gpio_callback *cb,
			   uint32_t pins)
{
	struct mb_display *disp = mb_display_get();

	if (pins & BIT(DT_ALIAS_SW0_GPIOS_PIN)) {
		k_work_submit(&button_work);
	} else {
		uint16_t target = board_set_target();

		if (target > 0x0009) {
			mb_display_print(disp, MB_DISPLAY_MODE_SINGLE,
					 2 * MSEC_PER_SEC, "A");
		} else {
			mb_display_print(disp, MB_DISPLAY_MODE_SINGLE,
					 2 * MSEC_PER_SEC, "%X", (target & 0xf));
		}
	}
}

void board_heartbeat(uint8_t hops, uint16_t feat)
{
	struct mb_display *disp = mb_display_get();
	const struct mb_image hops_img[] = {
		MB_IMAGE({ 1, 1, 1, 1, 1 },
			 { 1, 1, 1, 1, 1 },
			 { 1, 1, 1, 1, 1 },
			 { 1, 1, 1, 1, 1 },
			 { 1, 1, 1, 1, 1 }),
		MB_IMAGE({ 1, 1, 1, 1, 1 },
			 { 1, 1, 1, 1, 1 },
			 { 1, 1, 0, 1, 1 },
			 { 1, 1, 1, 1, 1 },
			 { 1, 1, 1, 1, 1 }),
		MB_IMAGE({ 1, 1, 1, 1, 1 },
			 { 1, 0, 0, 0, 1 },
			 { 1, 0, 0, 0, 1 },
			 { 1, 0, 0, 0, 1 },
			 { 1, 1, 1, 1, 1 }),
		MB_IMAGE({ 1, 1, 1, 1, 1 },
			 { 1, 0, 0, 0, 1 },
			 { 1, 0, 0, 0, 1 },
			 { 1, 0, 0, 0, 1 },
			 { 1, 1, 1, 1, 1 }),
		MB_IMAGE({ 1, 0, 1, 0, 1 },
			 { 0, 0, 0, 0, 0 },
			 { 1, 0, 0, 0, 1 },
			 { 0, 0, 0, 0, 0 },
			 { 1, 0, 1, 0, 1 })
	};

	// printk("%u hops\n", hops);

	if (hops) {
		hops = MIN(hops, ARRAY_SIZE(hops_img));
		mb_display_image(disp, MB_DISPLAY_MODE_SINGLE, 2 * MSEC_PER_SEC,
				 &hops_img[hops - 1], 1);
	}
}

void board_other_dev_pressed(uint16_t addr)
{
	struct mb_display *disp = mb_display_get();

	printk("board_other_dev_pressed(0x%04x)\n", addr);

	mb_display_print(disp, MB_DISPLAY_MODE_SINGLE, 2 * MSEC_PER_SEC,
			 "%X", (addr & 0xf));
}

void board_attention(bool attention)
{
	struct mb_display *disp = mb_display_get();
	static const struct mb_image attn_img[] = {
		MB_IMAGE({ 0, 0, 0, 0, 0 },
			 { 0, 0, 0, 0, 0 },
			 { 0, 0, 1, 0, 0 },
			 { 0, 0, 0, 0, 0 },
			 { 0, 0, 0, 0, 0 }),
		MB_IMAGE({ 0, 0, 0, 0, 0 },
			 { 0, 1, 1, 1, 0 },
			 { 0, 1, 1, 1, 0 },
			 { 0, 1, 1, 1, 0 },
			 { 0, 0, 0, 0, 0 }),
		MB_IMAGE({ 1, 1, 1, 1, 1 },
			 { 1, 1, 1, 1, 1 },
			 { 1, 1, 0, 1, 1 },
			 { 1, 1, 1, 1, 1 },
			 { 1, 1, 1, 1, 1 }),
		MB_IMAGE({ 1, 1, 1, 1, 1 },
			 { 1, 0, 0, 0, 1 },
			 { 1, 0, 0, 0, 1 },
			 { 1, 0, 0, 0, 1 },
			 { 1, 1, 1, 1, 1 }),
	};

	if (attention) {
		mb_display_image(disp,
				 MB_DISPLAY_MODE_DEFAULT | MB_DISPLAY_FLAG_LOOP,
				 150, attn_img, ARRAY_SIZE(attn_img));
	} else {
		mb_display_stop(disp);
	}
}

static void configure_button(void)
{
	static struct gpio_callback button_cb;

	k_work_init(&button_work, button_send_pressed);

	gpio = device_get_binding(DT_ALIAS_SW0_GPIOS_CONTROLLER);

	gpio_pin_configure(gpio, DT_ALIAS_SW0_GPIOS_PIN,
			   GPIO_INPUT | DT_ALIAS_SW0_GPIOS_FLAGS);
	gpio_pin_interrupt_configure(gpio, DT_ALIAS_SW0_GPIOS_PIN,
				     GPIO_INT_EDGE_TO_ACTIVE);

	gpio_pin_configure(gpio, DT_ALIAS_SW1_GPIOS_PIN,
			   GPIO_INPUT | DT_ALIAS_SW1_GPIOS_FLAGS);
	gpio_pin_interrupt_configure(gpio, DT_ALIAS_SW1_GPIOS_PIN,
				     GPIO_INT_EDGE_TO_ACTIVE);

	// gpio_pin_configure(gpio, DT_ALIAS_SW0_GPIOS_PIN,
	// 		   (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
	// 		    GPIO_INT_ACTIVE_LOW));
	// gpio_pin_configure(gpio, DT_ALIAS_SW1_GPIOS_PIN,
	// 		   (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
	// 		    GPIO_INT_ACTIVE_LOW));

	gpio_init_callback(&button_cb, button_pressed,
			   BIT(DT_ALIAS_SW0_GPIOS_PIN) | BIT(DT_ALIAS_SW1_GPIOS_PIN));
	gpio_add_callback(gpio, &button_cb);

	// gpio_pin_enable_callback(gpio, DT_ALIAS_SW0_GPIOS_PIN);
	// gpio_pin_enable_callback(gpio, DT_ALIAS_SW1_GPIOS_PIN);
}

void board_init(uint16_t *addr)
{
	struct mb_display *disp = mb_display_get();

	// nvm = device_get_binding(DT_FLASH_DEV_NAME);
	nvm = device_get_binding(DT_CHOSEN_ZEPHYR_FLASH_CONTROLLER_LABEL);

	*addr = NRF_UICR->CUSTOMER[0];
	if (!*addr || *addr == 0xffff) {
#if defined(NODE_ADDR)
		*addr = NODE_ADDR;
#else
		*addr = 0x0b0c;
#endif
	}

	mb_display_print(disp, MB_DISPLAY_MODE_DEFAULT, SCROLL_SPEED,
			 "0x%04x", *addr);

	configure_button();
}


/* End of Define trap */
#endif 
