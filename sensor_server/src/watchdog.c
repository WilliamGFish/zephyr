/*
 * Copyright (c) 2015 Intel Corporation
 * Copyright (c) 2018 Nordic Semiconductor
 * 
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/watchdog.h>
#include <sys/printk.h>

#define WDT_FEED_TRIES 5


// #ifdef CONFIG_WDT_0_NAME
// #define WDT_DEV_NAME CONFIG_WDT_0_NAME
// #else
// #define WDT_DEV_NAME DT_WDT_0_NAME
// #endif

/*
 * If the devicetree has a watchdog node, get its label property.
 */
#ifdef WDT_NODE
#define WDT_DEV_NAME DT_LABEL(WDT_NODE)
#else
#define WDT_DEV_NAME ""
/* error "Unsupported SoC and no watchdog0 alias in zephyr.dts" */
#endif

static void wdt_callback(const struct device *wdt_dev, int channel_id)
{
	/* Watchdog timer expired. Handle it here.
    * 
    * This will use the channel ID to define action
    * The callback will be used to send messages
	 * Remember that SoC reset will be done soon.
	 */
	static bool handled_event;

	if (handled_event) {
		return;
	}

	wdt_feed(wdt_dev, channel_id);

	printk("Handled things..ready to reset\n");
	handled_event = true;
}


void watchdog_init(void)
{
	int err;
	int wdt_channel_id;
	const struct device *wdt;
	struct wdt_timeout_cfg wdt_config;

	printk("Watchdog Sample\n");
	
	wdt = device_get_binding(WDT_DEV_NAME);
	if (!wdt) {
		printk("Cannot get WDT device\n");
		return;
	}

	/* Reset SoC when watchdog timer expires. */
	wdt_config.flags = WDT_FLAG_RESET_SOC;

	/* Expire watchdog after 5000 milliseconds. */
	wdt_config.window.min = 0U;
	wdt_config.window.max = 5000U;

	/* Set up watchdog callback. Jump into it when watchdog expired. */
	wdt_config.callback = wdt_callback;

	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	if (wdt_channel_id == -ENOTSUP) {
		/* IWDG driver for STM32 doesn't support callback */
		wdt_config.callback = NULL;
		wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	}
	if (wdt_channel_id < 0) {
		printk("Watchdog install error\n");
		return;
	}

	err = wdt_setup(wdt, 0);
	if (err < 0) {
		printk("Watchdog setup error\n");
		return;
	}

	/* Feeding watchdog. */
	printk("Feeding watchdog %d times\n", WDT_FEED_TRIES);
	for (int i = 0; i < WDT_FEED_TRIES; ++i) {
		printk("Feeding watchdog...\n");
		wdt_feed(wdt, wdt_channel_id);
		k_sleep(K_MSEC(1000));
	}

	/* Waiting for the SoC reset. */
	printk("Waiting for reset...\n");
	while (1) {
		k_yield();
	};
}
