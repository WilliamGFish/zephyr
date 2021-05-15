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

	/* Sensor srv: Enviroment: publish periodicaly to a remote address */
	struct bt_mesh_cfg_mod_pub sensor_srv_pub = {
		.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
		.app_idx = app_idx,
		.ttl = 0x07,
		.period = SENSOR_SRV_ENV_UPD_PERIOD,
		.transmit=BT_MESH_TRANSMIT(1, 20),
	};


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
	// /* Add Sensor Server model Publisher: Enviroment Info */
	// err = bt_mesh_cfg_mod_pub_set(net_idx, addr, elem_addr_r,
	// 			      BT_MESH_MODEL_ID_SENSOR_SRV, &count_pub, NULL);
	// if (err) {
	// 	printk("ERROR: Sensor Server model Publisher failed (err %d)\n", err);
	// }


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
	// /* Add Sensor Client model Publisher */
	// err = bt_mesh_cfg_mod_pub_set(net_idx, addr, elem_addr_r,
	// 			      BT_MESH_MODEL_ID_SENSOR_CLI, &sensor_cli_pub, NULL);
	// if (err) {
	// 	printk("ERROR: Sensor Client model Publisher failed (err %d)\n", err);
	// }
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



/*****************************************************************************************/

static void configure_self(struct bt_mesh_cdb_node *self)
{
	struct bt_mesh_cdb_app_key *key;

	printk("Configuring self...\n");

	key = bt_mesh_cdb_app_key_get(app_idx);
	if (key == NULL) {
		printk("No app-key 0x%04x\n", app_idx);
		return;
	}


	// /* Add Application Key */
	// int err;
	// err = bt_mesh_cfg_app_key_add(self->net_idx, self->addr, self->net_idx,
	// 			      app_idx, key->keys[0].app_key, NULL);
	// if (err < 0) {
	// 	printk("Failed to add app-key (err %d)\n", err);
	// 	return;
	// }

	// err = bt_mesh_cfg_mod_app_bind(self->net_idx, self->addr, self->addr,
	// 			       app_idx, BT_MESH_MODEL_ID_HEALTH_CLI,
	// 			       NULL);
	// if (err < 0) {
	// 	printk("Failed to bind app-key (err %d)\n", err);
	// 	return;
	// }


	memcpy(key->keys[0].app_key, app_key, 16);
	mesh_configure();

	atomic_set_bit(self->flags, BT_MESH_CDB_NODE_CONFIGURED);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_node_store(self);
	}

	printk("Configuration complete\n");
}


int config_node_sig_model(uint16_t node_addr, uint16_t mod_id)
{
	/* Code */
	int err = 0U;
	printk("\t\tSIG Models: ");
	printk("\t 0x%04x\n", mod_id);

	/* Default: publish periodicaly to a remote address */
	struct bt_mesh_cfg_mod_pub pub = {
		.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
		.app_idx = app_idx,
		.ttl = 0x07,
		.period = BT_MESH_PUB_PERIOD_SEC(5), 
		.transmit=BT_MESH_TRANSMIT(1, 20),
	};

	/* Use the model ID for selection */
	switch (mod_id) {
	case BT_MESH_MODEL_ID_SENSOR_SRV: 
		pub.period = SENSOR_SRV_CNT_UPD_PERIOD;
		break;
	case 2:  /* Get Y value */
		
	break;
	case 3: /* Get Z value */
	
	break;
	default: /* Default */
		pub.period = BT_MESH_PUB_PERIOD_SEC(5);
	}


	/* Bind to Health Server model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, node_addr, node_addr, app_idx,
					mod_id, NULL);
	if (err) {
		printk("ERROR: Health Server Binding App Key failed (err %d)\n", err);
	}
	/* Add Health Server model subscription */
	err = bt_mesh_cfg_mod_sub_add(net_idx, node_addr, node_addr, GROUP_ADDR,
					mod_id, NULL);
	if (err) {
		printk("ERROR: Health Server subscription failed (err %d)\n", err);
	}
	/* Add Health Server model Publisher */
	err = bt_mesh_cfg_mod_pub_set(net_idx, node_addr, node_addr, mod_id,
				      &pub, NULL);
	if (err) {
		printk("ERROR: Health Server Publisher failed (err %d)\n", err);
	}



	return 0;
}




int config_node_vnd_model(uint16_t addr, uint16_t cid, uint16_t mod_id)
{
	/* Code */
	int err = 0U;
	printk("\t\tVendor Models: ");
	printk("\tCompany 0x%04x:   0x%04x\n", cid, mod_id);


	/* Default: publish periodicaly to a remote address */
	struct bt_mesh_cfg_mod_pub pub = {
		.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
		.app_idx = app_idx,
		.ttl = 0x07,
		.period = BT_MESH_PUB_PERIOD_SEC(5), 
		.transmit=BT_MESH_TRANSMIT(1, 20),
	};

	/* Bind to Vendor model */
	bt_mesh_cfg_mod_app_bind_vnd(net_idx, addr, elem_addr_r, app_idx,
				     mod_id, cid, NULL);

	/* Add Vendor model subscription */
	err = bt_mesh_cfg_mod_sub_add_vnd(net_idx, addr, elem_addr_r, GROUP_ADDR,
					  mod_id, cid, NULL);
	if (err) {
		printk("ERROR: Vendor Server model subscription (err %d)\n", err);
	}
	/* Add Vendor Server model Publisher */
	err = bt_mesh_cfg_mod_pub_set_vnd(net_idx, addr, elem_addr_r,
					  mod_id, cid, &pub, NULL);
	if (err) {
		printk("ERROR: Vendor Server model Publisher failed  (err %d)\n", err);
	}

	return 0;	
}




static void configure_node(struct bt_mesh_cdb_node *node)
{	
	// struct bt_mesh_cdb_app_key *key;
	// struct bt_mesh_cfg_mod_pub pub;
	int err = 0U;

	printk("Getting config info.....\n");

	int comp_buffer_size = 128;
	NET_BUF_SIMPLE_DEFINE(remote_comp, comp_buffer_size);
	uint8_t status, page = 0xff;

	
	err = bt_mesh_cfg_comp_data_get(net_idx, node->addr, page,
					&status, &remote_comp);

	if (err == -EAGAIN) {
		if (IS_ENABLED(CONFIG_BT_MESH_CDB_ADDR_REISSUE)) {
			// LOG_DBG("Cleaning Replay Buffers for Addr %d (err %d)", addr, err);
			// printk("ERROR: CLEARING REPLAY BUFFER (err %d)\n", err);	

			/* Clear Replay buffer to allow reconnection */
			// mesh_rpl_addr_clear(node->addr);
		}
		return;			
	}

	if (err) {
		LOG_DBG("ERROR: Getting composition failed for Addr %d  (err %d)", node->addr, err);
		// printk("ERROR: Getting composition failed (err %d)\n", err);		
		return;
	}

	if (remote_comp.len < 7 || remote_comp.len > comp_buffer_size ){
		err = -ENOEXEC;
		LOG_DBG("ERROR: Composition data length invalid: %d - (err %d)", remote_comp.len, err);
		// printk("ERROR: Composition data length invalid: %d - (err %d)\n", remote_comp.len, err);
		
		if (IS_ENABLED(CONFIG_BT_MESH_CDB_ADDR_REISSUE)) {
			/* Clear Replay buffer to allow reconnection */
			// mesh_rpl_addr_clear(node->addr);
		}
		
		return;			
	}

	if (status != 0x00) {
		LOG_DBG("ERROR: Got non-success status 0x%02x\n", status);
		return;;
	}

	printk("\n\n\nGetting composition info, net_idx 0x%04x address 0x%04x\n", net_idx,
	       node->addr);

    net_buf_simple_pull_le16(&remote_comp);
    net_buf_simple_pull_le16(&remote_comp);
    net_buf_simple_pull_le16(&remote_comp);
    net_buf_simple_pull_le16(&remote_comp);

	uint16_t features = net_buf_simple_pull_le16(&remote_comp);

	printk("\tFeatures:\n\tLPN\tFnd\tPxy\tRly\n\t");
	int c, k;
	for (c = 3; c >= 0; c--){
		k = features >> c;
		printk(" %d\t", k & 1);
	}


	/* Add Application Key */
	err = bt_mesh_cfg_app_key_add(net_idx, node->addr, net_idx, app_idx, app_key, NULL);
	if (err) {
		printk("ERROR: Adding App Key failed on %0x04x (err %d)\n",node->addr, err);
	}


	while (remote_comp.len > 4 && remote_comp.len < comp_buffer_size) {
		uint8_t sig, vnd;
		uint16_t loc;
		int i;

		loc = net_buf_simple_pull_le16(&remote_comp);
		sig = net_buf_simple_pull_u8(&remote_comp);
		vnd = net_buf_simple_pull_u8(&remote_comp);

		printk("\tElement @ 0x%04x:\n", loc);

		if (remote_comp.len < ((sig * 2U) + (vnd * 4U))) {
			LOG_DBG("\t\t...truncated data!");
			break;
		}

		for (i = 0; i < sig; i++) {
			uint16_t mod_id = net_buf_simple_pull_le16(&remote_comp);

			err = config_node_sig_model(node->addr + loc, mod_id);
		}

		for (i = 0; i < vnd; i++) {
			uint16_t cid = net_buf_simple_pull_le16(&remote_comp);
			uint16_t mod_id = net_buf_simple_pull_le16(&remote_comp);

			err = config_node_vnd_model(node->addr  + loc, cid, mod_id);
		}
	}


	/* Add model keys, subscriptions and publishers */
	printk("Configuring...0x%04x\n", node->addr);
	
	// /* Add Application Key */
	// err = bt_mesh_cfg_app_key_add(net_idx, node->addr, net_idx, app_idx, app_key, NULL);
	// if (err) {
	// 	printk("ERROR: Adding App Key failed on %0x04x (err %d)\n",node->addr, err);
	// }

	// /* Default: publish periodicaly to a remote address */
	// struct bt_mesh_cfg_mod_pub pub = {
	// 	.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
	// 	.app_idx = app_idx,
	// 	.ttl = 0x07,
	// 	.period = BT_MESH_PUB_PERIOD_SEC(5), 
	// 	.transmit=BT_MESH_TRANSMIT(1, 20),
	// };

	// /* Health srv: publish periodicaly to a remote address */
	// struct bt_mesh_cfg_mod_pub health_srv_pub = {
	// 	.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
	// 	.app_idx = app_idx,
	// 	.ttl = 0x07,
	// 	// .period = HEALTH_SRV_UPDATE_PERIOD, 
	// 	.transmit=BT_MESH_TRANSMIT(1, 20),
	// };

	// /* Sensor CLI: Enviroment: publish periodicaly to a remote address */
	// struct bt_mesh_cfg_mod_pub sensor_cli_pub = {
	// 	.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
	// 	.app_idx = app_idx,
	// 	.ttl = 0x07,
	// 	.period = SENSOR_CLI_UPDATE_PERIOD,
	// 	.transmit=BT_MESH_TRANSMIT(1, 20),
	// };

	// /*  Sensor srv: Count: publish periodicaly to a remote address */
	// struct bt_mesh_cfg_mod_pub count_pub = {
	// 	.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
	// 	.app_idx = app_idx,
	// 	.ttl = 0x07,
	// 	// .period = sensor_srv_period, 
	// 	.period = SENSOR_SRV_CNT_UPD_PERIOD, 	
	// 	// .transmit=BT_MESH_TRANSMIT(1, 20),
	// };

	// /* Sensor srv: Enviroment: publish periodicaly to a remote address */
	// struct bt_mesh_cfg_mod_pub sensor_srv_pub = {
	// 	.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
	// 	.app_idx = app_idx,
	// 	.ttl = 0x07,
	// 	.period = SENSOR_SRV_ENV_UPD_PERIOD,
	// 	.transmit=BT_MESH_TRANSMIT(1, 20),
	// };


	// /* Bind to Health Server model */
	// err = bt_mesh_cfg_mod_app_bind(net_idx, node->addr, node->addr, app_idx,
	// 				BT_MESH_MODEL_ID_HEALTH_SRV, NULL);
	// if (err) {
	// 	printk("ERROR: Health Server Binding App Key failed (err %d)\n", err);
	// }
	// /* Add Health Server model subscription */
	// err = bt_mesh_cfg_mod_sub_add(net_idx, node->addr, node->addr, GROUP_ADDR,
	// 				BT_MESH_MODEL_ID_HEALTH_SRV, NULL);
	// if (err) {
	// 	printk("ERROR: Health Server subscription failed (err %d)\n", err);
	// }
	// /* Add Health Server model Publisher */
	// err = bt_mesh_cfg_mod_pub_set(net_idx, node->addr, node->addr,
	// 				BT_MESH_MODEL_ID_HEALTH_SRV, &health_srv_pub, NULL);
	// if (err) {
	// 	printk("ERROR: Health Server Publisher failed (err %d)\n", err);
	// }


	// /* Bind to Health Client model */
	// err = bt_mesh_cfg_mod_app_bind(net_idx, node->addr, node->addr, app_idx,
	// 				BT_MESH_MODEL_ID_HEALTH_CLI, NULL);
	// if (err) {
	// 	printk("ERROR: Health Client Binding App Key failed (err %d)\n", err);
	// }
	// /* Add Health Client model subscription */
	// err = bt_mesh_cfg_mod_sub_add(net_idx, node->addr, node->addr, GROUP_ADDR,
	// 				BT_MESH_MODEL_ID_HEALTH_CLI, NULL);
	// if (err) {
	// 	printk("ERROR: Health Client subscription failed (err %d)\n", err);
	// }



	// /* Bind to Vendor model */
	// bt_mesh_cfg_mod_app_bind_vnd(net_idx, addr, elem_addr_r, app_idx,
	// 			     MOD_LF, BT_COMP_ID_LF, NULL);

	// /* Add Vendor model subscription */
	// err = bt_mesh_cfg_mod_sub_add_vnd(net_idx, addr, elem_addr_r, GROUP_ADDR,
	// 				  MOD_LF, BT_COMP_ID_LF, NULL);
	// if (err) {
	// 	printk("ERROR: Vendor Server model subscription (err %d)\n", err);
	// }
	// /* Add Vendor Server model Publisher */
	// err = bt_mesh_cfg_mod_pub_set_vnd(net_idx, addr, elem_addr_r,
	// 				  MOD_LF, BT_COMP_ID_LF, &pub, NULL);
	// if (err) {
	// 	printk("ERROR: Vendor Server model Publisher failed  (err %d)\n", err);
	// }

	
	// /* Bind to Sensor Server model */
	// err = bt_mesh_cfg_mod_app_bind(net_idx, node->addr, node->addr, app_idx,
	// 				BT_MESH_MODEL_ID_SENSOR_SRV, NULL);
	// if (err) {
	// 	printk("ERROR: Sensor Binding App Key failed (err %d)\n", err);
	// }
	// /* Add Sensor Server model subscription */
	// err = bt_mesh_cfg_mod_sub_add(net_idx, node->addr, node->addr, GROUP_ADDR,
	// 				BT_MESH_MODEL_ID_SENSOR_SRV, NULL);
	// if (err) {
	// 	printk("ERROR: Sensor Server model subscription failed (err %d)\n", err);
	// }
	// /* Add Sensor Server model Publisher: Enviroment Info */
	// err = bt_mesh_cfg_mod_pub_set(net_idx, node->addr, node->addr,
	// 				BT_MESH_MODEL_ID_SENSOR_SRV, &count_pub, NULL);
	// if (err) {
	// 	printk("ERROR: Sensor Server model Publisher failed (err %d)\n", err);
	// }

	// /* CDB for ading node to config DB */
	atomic_set_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_node_store(node);
	}

	printk("Node Configuration complete\n");
}



/* End of Define trap */
#endif