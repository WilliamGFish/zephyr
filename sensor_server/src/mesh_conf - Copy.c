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

	/* Default: publish periodicaly to a remote address */
	struct bt_mesh_cfg_mod_pub pub = {
		.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
		.app_idx = app_idx,
		.ttl = 0x07,
		.period = BT_MESH_PUB_PERIOD_SEC(5), 
		.transmit=BT_MESH_TRANSMIT(1, 20),
	};

	/* Health srv: publish periodicaly to a remote address */
	struct bt_mesh_cfg_mod_pub health_srv_pub = {
		.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
		.app_idx = app_idx,
		.ttl = 0x07,
		// .period = HEALTH_SRV_UPDATE_PERIOD, 
		.transmit=BT_MESH_TRANSMIT(1, 20),
	};

	/* Sensor CLI: Enviroment: publish periodicaly to a remote address */
	struct bt_mesh_cfg_mod_pub sensor_cli_pub = {
		.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
		.app_idx = app_idx,
		.ttl = 0x07,
		.period = SENSOR_CLI_UPDATE_PERIOD,
		.transmit=BT_MESH_TRANSMIT(1, 20),
	};

	/*  Sensor srv: Count: publish periodicaly to a remote address */
	struct bt_mesh_cfg_mod_pub count_pub = {
		.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
		.app_idx = app_idx,
		.ttl = 0x07,
		// .period = sensor_srv_period, 
		.period = SENSOR_SRV_CNT_UPD_PERIOD, 	
		// .transmit=BT_MESH_TRANSMIT(1, 20),
	};

	// /* Sensor srv: Enviroment: publish periodicaly to a remote address */
	// struct bt_mesh_cfg_mod_pub sensor_srv_pub = {
	// 	.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
	// 	.app_idx = app_idx,
	// 	.ttl = 0x07,
	// 	.period = SENSOR_SRV_ENV_UPD_PERIOD,
	// 	.transmit=BT_MESH_TRANSMIT(1, 20),
	// };

	// uint16_t key_app_idx = 1;

/*
int bt_mesh_cfg_app_key_add(uint16_t net_idx, uint16_t addr, uint16_t key_net_idx, uint16_t key_app_idx, const uint8_t app_key[16], uint8_t *status)
int bt_mesh_cfg_mod_app_bind(uint16_t net_idx, uint16_t addr, uint16_t elem_addr, uint16_t mod_app_idx, uint16_t mod_id, uint8_t *status)
int bt_mesh_cfg_mod_sub_add(uint16_t net_idx, uint16_t addr, uint16_t elem_addr, uint16_t sub_addr, uint16_t mod_id, uint8_t *status)
int bt_mesh_cfg_mod_pub_set(uint16_t net_idx, uint16_t addr, uint16_t elem_addr, uint16_t mod_id, struct bt_mesh_cfg_mod_pub *pub, uint8_t *status)
*/

	/* Add model keys, subscriptions and publishers */
	int err;

	/* Add Application Key */
	bt_mesh_cfg_app_key_add(net_idx, addr, net_idx, app_idx, app_key, NULL);


	/* Bind to Health Server model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, addr, elem_addr_r, app_idx,
				       BT_MESH_MODEL_ID_HEALTH_SRV, NULL);
	if (err) {
		printk("ERROR: Health Server Binding App Key failed (err %d)\n", err);
	}
	/* Add Health Server model subscription */
	err = bt_mesh_cfg_mod_sub_add(net_idx, addr, elem_addr_r, GROUP_ADDR,
				      BT_MESH_MODEL_ID_HEALTH_SRV, NULL);
	if (err) {
		printk("ERROR: Health Server subscription failed (err %d)\n", err);
	}
	/* Add Health Server model Publisher */
	err = bt_mesh_cfg_mod_pub_set(net_idx, addr, elem_addr_r,
				      BT_MESH_MODEL_ID_HEALTH_SRV, &health_srv_pub, NULL);
	if (err) {
		printk("ERROR: Health Server Publisher failed (err %d)\n", err);
	}


	/* Bind to Health Client model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, addr, elem_addr_r, app_idx,
				       BT_MESH_MODEL_ID_HEALTH_CLI, NULL);
	if (err) {
		printk("ERROR: Health Client Binding App Key failed (err %d)\n", err);
	}
	/* Add Health Client model subscription */
	err = bt_mesh_cfg_mod_sub_add(net_idx, addr, elem_addr_r, GROUP_ADDR,
				      BT_MESH_MODEL_ID_HEALTH_CLI, NULL);
	if (err) {
		printk("ERROR: Health Client subscription failed (err %d)\n", err);
	}

#if !defined(CONFIG_BOARD_BBC_MICROBIT)
	/* Bind to Vendor model */
	bt_mesh_cfg_mod_app_bind_vnd(net_idx, addr, elem_addr_r, app_idx,
				     MOD_LF, BT_COMP_ID_LF, NULL);

	/* Add Vendor model subscription */
	err = bt_mesh_cfg_mod_sub_add_vnd(net_idx, addr, elem_addr_r, GROUP_ADDR,
					  MOD_LF, BT_COMP_ID_LF, NULL);
	if (err) {
		printk("ERROR: Vendor Server model subscription (err %d)\n", err);
	}
	/* Add Vendor Server model Publisher */
	err = bt_mesh_cfg_mod_pub_set_vnd(net_idx, addr, elem_addr_r,
					  MOD_LF, BT_COMP_ID_LF, &pub, NULL);
	if (err) {
		printk("ERROR: Vendor Server model Publisher failed  (err %d)\n", err);
	}
#endif



	/* Bind to Sensor Server model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, addr, elem_addr_r, app_idx,
				       BT_MESH_MODEL_ID_SENSOR_SRV, NULL);
	if (err) {
		printk("ERROR: Sensor Binding App Key failed (err %d)\n", err);
	}
	/* Add Sensor Server model subscription */
	err = bt_mesh_cfg_mod_sub_add(net_idx, addr, elem_addr_r, GROUP_ADDR,
				      BT_MESH_MODEL_ID_SENSOR_SRV, NULL);
	if (err) {
		printk("ERROR: Sensor Server model subscription failed (err %d)\n", err);
	}
	/* Add Sensor Server model Publisher: Enviroment Info */
	err = bt_mesh_cfg_mod_pub_set(net_idx, addr, elem_addr_r,
				      BT_MESH_MODEL_ID_SENSOR_SRV, &count_pub, NULL);
	if (err) {
		printk("ERROR: Sensor Server model Publisher failed (err %d)\n", err);
	}


	/* Bind to Sensor Client model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, addr, elem_addr_r, app_idx,
				       BT_MESH_MODEL_ID_SENSOR_CLI, NULL);
	if (err) {
		printk("ERROR: Sensor Client Binding App Key failed (err %d)\n", err);
	}

	/* Add Sensor Client model subscription */
	err = bt_mesh_cfg_mod_sub_add(net_idx, addr, elem_addr_r, GROUP_ADDR,
				      BT_MESH_MODEL_ID_SENSOR_CLI, NULL);
	if (err) {
		printk("ERROR: Sensor Client model subscription failed (err %d)\n", err);
	}
	/* Add Sensor Client model Publisher */
	err = bt_mesh_cfg_mod_pub_set(net_idx, addr, elem_addr_r,
				      BT_MESH_MODEL_ID_SENSOR_CLI, &sensor_cli_pub, NULL);
	if (err) {
		printk("ERROR: Sensor Client model Publisher failed (err %d)\n", err);
	}
#if !defined(CONFIG_BOARD_BBC_MICROBIT)
#endif

	/****************************************************************************************/
	/*                      		*/
    /* 		Second Model Elements 	*/
	/* Bind to Sensor Server model 	*/
	// err = bt_mesh_cfg_mod_app_bind(net_idx, addr, elem_addr_1, app_idx,
	// 			       BT_MESH_MODEL_ID_SENSOR_SRV, NULL);
	// if (err) {
	// 	printk("ERROR: Sensor Model 1 Binding App Key failed (err %d)\n", err);
	// }
	// /* Add Sensor Server model subscription */
	// err = bt_mesh_cfg_mod_sub_add(net_idx, GROUP_ADDR, elem_addr_1,
	// 			      addr, BT_MESH_MODEL_ID_SENSOR_SRV, NULL);
	// if (err) {
	// 	printk("ERROR: Sensor Server Model 1 subscription failed (err %d)\n", err);
	// }
	// /* Add Sensor Server model Publisher */
	// err = bt_mesh_cfg_mod_pub_set(net_idx, addr, elem_addr_1,
	// 			      BT_MESH_MODEL_ID_SENSOR_SRV, &sensor_srv_pub, NULL);
	// if (err) {
	// 	printk("ERROR: Sensor Server Model 1 Publisher failed (err %d)\n", err);
	// }


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