/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * This is a function to manually provision node
 */
#ifndef _MANULYTICA_MESH_CONF_C_
#define _MANULYTICA_MESH_CONF_C_


#include <zephyr.h>
#include <sys/printk.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <drivers/sensor.h>

#include <sys/byteorder.h>

#include "mesh_conf.h"

static void mesh_configure(void)
{
	printk("Configuring...\n");

	#if NODE_ADDR == GATEWAY_ADDR
	{
		struct bt_mesh_cfg_hb_pub pub = {
			.dst = GROUP_ADDR,
			.count = 0xff,
			.period = 0x05,
			.ttl = 0x07,
			.feat = 0,
			.net_idx = net_idx,
		};

		bt_mesh_cfg_hb_pub_set(net_idx, addr, &pub, NULL);
		printk("Publishing heartbeat messages\n");
	}
	#endif
	printk("Configuration complete\n");
}

/* End of Define trap */
#endif