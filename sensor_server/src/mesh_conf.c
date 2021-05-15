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



#define HB_ADDR 0xfab0
#define HEALTH_ADDR 0xfab1
#define SENSOR_ROOT_ADDR 0xfac0
#define SENSOR_2ND_ADDR 0xfac1
#define SENSOR_3RD_ADDR 0xfac2

/* For replay buffer clearing */
// #include <mesh/rpl.h>


/*********************************************************************************************************************/

// int mesh_rpl_addr_clear(uint16_t src)
// {
// 	int i;

// 	if (!src) {
// 		LOG_DBG("ERROR: Invalid source Addr: 0x%04x", src);
// 		// printk("ERROR: Invalid source Addr: 0x%04x\n", src);
// 		return -ENOENT;
// 	}

// // // Changed after rewrite of replay buffer (Bluetooth: Mesh: Move Replay Protect to seperate module)
// // 	for (i = 0; i < ARRAY_SIZE(bt_mesh.rpl); i++) {
// // 		if (bt_mesh.rpl[i].src == src) {

// // 			LOG_DBG("Cleaning Replay Buffers for Addr %d", src);
// // 			// printk("\n\tCLEARING RPL for source Addr: 0x%04x\n", src);

// // 			(void)memset(&bt_mesh.rpl[i], 0, sizeof(*&bt_mesh.rpl[i]));
// // 			return 0;			
// // 		}
// // 	}

// 	for (i = 0; i < ARRAY_SIZE(replay_list); i++) {
// 		struct bt_mesh_rpl *rpl = &replay_list[i];

// 		if (rpl->src == src) {
// 				(void)memset(rpl, 0, sizeof(*rpl));
		
// 			if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
// 				bt_mesh_store_rpl(rpl);
// 			}
// 			return 0;
// 		}
// 	}	

// 	LOG_DBG("Error: No Replay Buffers Found for Addr %d", src);
// 	// printk("ERROR: NOT FOUND!! source Addr: 0x%04x\n", src);
// 	return -ENOENT;
// }

/*********************************************************************************************************************/

static void mesh_configure(void)
{
	printk("Configuring GATEWAY started ...\n");

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
		.addr = HEALTH_ADDR, /*GROUP_ADDR or Node addr */
		.app_idx = app_idx,
		.ttl = 0x07,
		// .period = HEALTH_SRV_UPDATE_PERIOD, 
		.transmit=BT_MESH_TRANSMIT(1, 20),
	};

	/* Sensor CLI: Enviroment: publish periodicaly to a remote address */
	// struct bt_mesh_cfg_mod_pub sensor_cli_pub = {
	// 	.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
	// 	.app_idx = app_idx,
	// 	.ttl = 0x07,
	// 	.period = SENSOR_CLI_UPDATE_PERIOD,
	// 	.transmit=BT_MESH_TRANSMIT(1, 20),
	// };

	/*  Sensor srv: Count: publish periodicaly to a remote address */
	// struct bt_mesh_cfg_mod_pub count_pub = {
	// 	.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
	// 	.app_idx = app_idx,
	// 	.ttl = 0x07,
	// 	// .period = sensor_srv_period, 
	// 	.period = SENSOR_SRV_CNT_UPD_PERIOD, 	
	// 	// .transmit=BT_MESH_TRANSMIT(1, 20),
	// };

	/* Sensor srv: Enviroment: publish periodicaly to a remote address */
	// struct bt_mesh_cfg_mod_pub sensor_srv_pub = {
	// 	.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
	// 	.app_idx = app_idx,
	// 	.ttl = 0x07,
	// 	.period = SENSOR_SRV_ENV_UPD_PERIOD,
	// 	.transmit=BT_MESH_TRANSMIT(1, 20),
	// };



	/* Add model keys, subscriptions and publishers */
	int err = 0U;

	/* Add Application Key */
	bt_mesh_cfg_app_key_add(net_idx, addr, net_idx, app_idx, app_key, NULL);


	/* Bind to Health Server model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, addr, elem_addr_r, app_idx,
				       BT_MESH_MODEL_ID_HEALTH_SRV, NULL);
	if (err) {
		printk("ERROR: Health Server Binding App Key failed (err %d)\n", err);
	}
	/* Add Health Server model subscription */
	err = bt_mesh_cfg_mod_sub_add(net_idx, addr, elem_addr_r, HEALTH_ADDR,
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
	err = bt_mesh_cfg_mod_sub_add(net_idx, addr, elem_addr_r, HEALTH_ADDR,
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
	err = bt_mesh_cfg_mod_sub_add(net_idx, addr, elem_addr_r, SENSOR_ROOT_ADDR,
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
	err = bt_mesh_cfg_mod_sub_add(net_idx, addr, elem_addr_r, SENSOR_ROOT_ADDR,
				      BT_MESH_MODEL_ID_SENSOR_CLI, NULL);
	if (err) {
		printk("ERROR: Sensor Client model subscription failed (err %d)\n", err);
	}


	/* Add Sensor Client model subscription ROOT */
	err = bt_mesh_cfg_mod_sub_add(net_idx, addr, elem_addr_r, SENSOR_2ND_ADDR,
				      BT_MESH_MODEL_ID_SENSOR_CLI, NULL);
	if (err) {
		printk("ERROR: Sensor Client model subscription failed (err %d)\n", err);
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
		// int err;
		// struct bt_mesh_cfg_hb_sub sub = {
		// 	// .src = node_addr,
		// 	.dst = GROUP_ADDR,
		// 	.period = 0x10,
		// };

		// err = bt_mesh_cfg_hb_sub_set(net_idx, addr, &sub, NULL);
		// if (err) {
		// 	printk("\nERROR: Subscribing heartbeat messages failed NODE:  0x%04x\n", addr);
		// 	return err;
		// }else{
		// 	printk("Subscribing to NODE heartbeat messages:  0x%04x\n", addr);
		// }


		// struct bt_mesh_cfg_hb_pub pub = {
		// 	.dst = GROUP_ADDR,
		// 	.count = 0xff,
		// 	.period = 0x05,
		// 	.ttl = 0x07,
		// 	.feat = 0,
		// 	.net_idx = net_idx,
		// };

		// bt_mesh_cfg_hb_pub_set(net_idx, addr, &pub, NULL);
		// printk("Publishing heartbeat messages\n");
	}
	#endif

	printk("GATEWAY Configuration complete\n");
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

	// bt_rand(key->keys[0].app_key, 16);
	memcpy(key->keys[0].app_key, app_key, 16);
	mesh_configure();

	atomic_set_bit(self->flags, BT_MESH_CDB_NODE_CONFIGURED);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_node_store(self);
	}

	printk("Configuration complete\n");
}


int config_node_sig_model(uint16_t node_addr, uint16_t elem_addr, uint16_t mod_id)
{
	/* Code */
	int err = 0U;
	bool add_pub, add_sub;
	uint16_t sub_addr = GROUP_ADDR;
	uint8_t status;

	printk("\t\tSIG Models: ");
	printk("\t 0x%04x\t", mod_id);

	/* Default: publish periodicaly to a remote address */
	struct bt_mesh_cfg_mod_pub sig_pub = {
		.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
		.app_idx = app_idx,
		.ttl = 0x07,
		.period = BT_MESH_PUB_PERIOD_SEC(1), 
		// .transmit=BT_MESH_TRANSMIT(1, 20),
	};

	/* Use the model ID for selection */
	switch (mod_id) {
		case BT_MESH_MODEL_ID_CFG_CLI:
		case BT_MESH_MODEL_ID_CFG_SRV:		
			add_pub = false;
			add_sub = false;
			break;

		case BT_MESH_MODEL_ID_HEALTH_CLI:
			add_pub = false;
			add_sub = true;
			sig_pub.period = 0U;
			sig_pub.addr = GROUP_ADDR; 
			break;

		case BT_MESH_MODEL_ID_HEALTH_SRV:
			add_pub = true;
			add_sub = true;
			sig_pub.period = 0U;
			sig_pub.addr = HEALTH_ADDR;

			if (node_addr != GATEWAY_ADDR)
				/* Heartbeat subcscription is a temporary state (due to there
				* not being an "indefinite" value for the period, so it never
				* gets stored persistently. Therefore, we always have to configure
				* it explicitly.
				*/
				{
					// struct bt_mesh_cfg_hb_sub gw_hb_sub = {
					// 	.src = node_addr,
					// 	.dst = GROUP_ADDR,
					// 	.period = 0x10,
					// };

					// err = bt_mesh_cfg_hb_sub_set(net_idx, addr, &gw_hb_sub, NULL);
					// if (err) {
					// 	printk("\nERROR: Subscribing heartbeat messages failed NODE:  0x%04x\n", node_addr);
					// 	return err;
					// }else{
					// 	printk("\nSubscribing to NODE heartbeat messages:  0x%04x\n", node_addr);
					// }




/*****************************************************************/
// 					struct bt_mesh_cfg_hb_pub node_hb_pub = {
// 						.dst = HB_ADDR,
// 						.count = 0xff,
// 						.period = 0x05,
// 						.ttl = 0x07,
// 						.feat = BT_MESH_FEAT_RELAY,
// 						.net_idx = net_idx,
// 					};

// 					err = bt_mesh_cfg_hb_pub_set(net_idx, node_addr, &node_hb_pub, NULL);
// 					if (err) {
// 						printk("\nERROR: Publishing heartbeat messages failed NODE:  0x%04x\n", node_addr);
// 						return err;
// 					}else{
// 						LOG_DBG("\nPublishing NODE heartbeat messages:  0x%04x\n", node_addr);
// 					}
/*****************************************************************/


					// struct bt_mesh_cfg_hb_sub sub = {
					// 	.src = GATEWAY_ADDR,
					// 	.dst = GROUP_ADDR,
					// 	.period = 0x10,
					// };

					// err = bt_mesh_cfg_hb_sub_set(net_idx, node_addr, &sub, NULL);
					// printk("Subscribing to heartbeat messages\n");
			}
						
			break;

		case BT_MESH_MODEL_ID_SENSOR_CLI:
			add_pub = false;
			add_sub = false;
			break;

		case BT_MESH_MODEL_ID_SENSOR_SRV:
			add_pub = true;
			add_sub = true;
			if (node_addr == elem_addr){
				sig_pub.period = SENSOR_SRV_ENV_UPD_PERIOD;
				sig_pub.addr = SENSOR_ROOT_ADDR;
			}else
			{
				sig_pub.period = SENSOR_SRV_CNT_UPD_PERIOD;
				sig_pub.addr = SENSOR_2ND_ADDR;
			}
						
			
			break;

		case BT_MESH_MODEL_ID_SENSOR_SETUP_SRV:
			add_pub = false;
			add_sub = false;
			sig_pub.period = 0U;
			break;

		default: /* Default */
			add_pub = false;
			add_sub = false;
			sig_pub.period = BT_MESH_PUB_PERIOD_SEC(5);
	}


printk("pub.period: %d", sig_pub.period);
	printk("\n");


	if(add_pub || add_sub){
		/* Bind to Model */
		err = bt_mesh_cfg_mod_app_bind(net_idx, node_addr, node_addr, app_idx,
						mod_id, NULL);
		if (err) {
			printk("ERROR: Binding App Key failed 0x%04x : 0x%04x (err %d)\n", node_addr, mod_id, err);
		}

		if (add_sub){
			/* Add Model subscription */
			err = bt_mesh_cfg_mod_sub_add(net_idx, node_addr, elem_addr, sub_addr,
							mod_id, NULL);
			if (err) {
				printk("ERROR: Subscription failed 0x%04x : 0x%04x (err %d)\n", node_addr, mod_id, err);
			}
		}
		if (add_pub){
			/* Add Model Publisher */
			err = bt_mesh_cfg_mod_pub_set(net_idx, node_addr, elem_addr, mod_id,
							&sig_pub, &status);
			if (err) {
				printk("ERROR: Publisher failed 0x%04x : 0x%04x (err %d)\n", node_addr, mod_id, err);
				return err;
			}
			
			if (status) {
				printk("ERROR: Model Publication Set failed (status 0x%02x)\n", status);
				err = -status;
			} else {
				LOG_DBG("Model Publication successfully set (status 0x%02x)\n", status);
			}
		}
	}

	return err;
}

int config_node_vnd_model(uint16_t addr, uint16_t elem_addr, uint16_t cid, uint16_t mod_id)
{
	/* Code */
	int err = 0U;
	printk("\t\tVendor Models: ");
	printk("\tCompany 0x%04x:   0x%04x\n", cid, mod_id);


	/* Default: publish periodicaly to a remote address */
	struct bt_mesh_cfg_mod_pub vnd_pub = {
		.addr = GROUP_ADDR, /*GROUP_ADDR or Node addr */
		.app_idx = app_idx,
		.ttl = 0x07,
		.period = BT_MESH_PUB_PERIOD_SEC(5), 
		// .transmit=BT_MESH_TRANSMIT(1, 20),
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
					  mod_id, cid, &vnd_pub, NULL);
	if (err) {
		printk("ERROR: Vendor Server model Publisher failed  (err %d)\n", err);
	}

	return err;	
}

static void configure_node(struct bt_mesh_cdb_node *node)
{	
	// struct bt_mesh_cdb_app_key *key;
	// struct bt_mesh_cfg_mod_pub pub;
	int err = 0U;
	int cnt = 0U;

	printk("Configuring node 0x%04x...\n", node->addr);

	int comp_buffer_size = 128;
	NET_BUF_SIMPLE_DEFINE(remote_comp, comp_buffer_size);
	uint8_t status, page = 0xff;

	err = bt_mesh_cfg_comp_data_get(net_idx, node->addr, page, &status,
					&remote_comp);

	if (err == -EAGAIN) {
		while (err == -EAGAIN && cnt++ < 2)
		{		
			if (IS_ENABLED(CONFIG_BT_MESH_CDB_ADDR_REISSUE)) {
				printk("Cleaning Replay Buffers for Addr 0x%04x (err %d)\n",  node->addr, err);
				/* Clear Replay buffer to allow reconnection */
				// mesh_rpl_addr_clear(node->addr);
				bt_mesh_rpl_addr_clear(node->addr);
			}
			
			k_sleep(K_MSEC(1000));
			err = bt_mesh_cfg_comp_data_get(net_idx, node->addr, page, &status,
				&remote_comp);
		}
		if(err){
			return;			
		}
	}


	if (err) {
		printk("ERROR: Getting composition failed for Addr 0x%04x  (err %d)", node->addr, err);
		return;
	}

	if (remote_comp.len < 7 || remote_comp.len > comp_buffer_size ){
		err = -ENOEXEC;
		LOG_DBG("ERROR: Composition data length invalid: %d - (err %d)", remote_comp.len, err);
		
		if (IS_ENABLED(CONFIG_BT_MESH_CDB_ADDR_REISSUE)) {
			/* Clear Replay buffer to allow reconnection */
			// mesh_rpl_addr_clear(node->addr);
			// bt_mesh_rpl_addr_clear(node->addr);
		}
		
		return;			
	}

	if (status != 0x00) {
		LOG_DBG("ERROR: Got non-success status 0x%02x\n", status);
		return;;
	}

	printk("\n\n\nGetting composition info, net_idx 0x%04x address 0x%04x\n",
	       net_idx, node->addr);

	net_buf_simple_pull_le16(&remote_comp);
	net_buf_simple_pull_le16(&remote_comp);
	net_buf_simple_pull_le16(&remote_comp);
	net_buf_simple_pull_le16(&remote_comp);
	// net_buf_simple_pull_le16(&remote_comp);

	uint16_t features = net_buf_simple_pull_le16(&remote_comp);

	printk("\tFeatures:\n\tLPN\tFnd\tPxy\tRly %04x \n\t",features);
	int c, k;
	for (c = 3; c >= 0; c--){
		k = features >> c;
		printk(" %d\t", k & 1);
	}

	/* Add Application Key */
	printk("Adding App Key on 0x%04x \n",node->addr);
	err = bt_mesh_cfg_app_key_add(net_idx, node->addr, net_idx, app_idx, app_key, NULL);
	if (err) {
		printk("ERROR: Adding App Key failed on 0x%04x (err %d)\n",node->addr, err);
	}

	while (remote_comp.len > 4 && remote_comp.len < comp_buffer_size) {
		uint8_t sig, vnd;
		uint16_t loc;
		int i;

		loc = net_buf_simple_pull_le16(&remote_comp);
		sig = net_buf_simple_pull_u8(&remote_comp);
		vnd = net_buf_simple_pull_u8(&remote_comp);

		printk("\tElement @ 0x%04x:  - Addr  0x%04x: \n", loc, node->addr  + loc);

		if (remote_comp.len < ((sig * 2U) + (vnd * 4U))) {
			LOG_DBG("\t\t...truncated data!");
			break;
		}

			
		for (i = 0; i < sig; i++) {
			uint16_t mod_id = net_buf_simple_pull_le16(&remote_comp);
			
			LOG_DBG("Config SIG Model 0x%04x (no: %d of %d)\n", mod_id, i, sig);
			err = config_node_sig_model(node->addr,
						    node->addr + loc, mod_id);
			if (err){
				printk("ERROR: Failed to Config SIG Model 0x%04x (err %d)\n", mod_id, err);
				return;
			}
		}

		for (i = 0; i < vnd; i++) {
			uint16_t cid = net_buf_simple_pull_le16(&remote_comp);
			uint16_t mod_id = net_buf_simple_pull_le16(&remote_comp);

			LOG_DBG("Config VND Model 0x%04x (no:  %d of %d)\n", mod_id, i, vnd);
			err = config_node_vnd_model(
				node->addr, node->addr + loc, cid, mod_id);
			if (err){
				printk("ERROR: Failed to Config VND Model 0x%04x (err %d)\n", mod_id, err);
				return;
			}
		}
	}
	
	// /* CDB for adding node to config DB */
	atomic_set_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_node_store(node);
	}

	printk("Node Configuration complete\n");
	
	/* Add sub to gateway for heartbeat messages */
	struct bt_mesh_cfg_hb_sub gw_hb_sub = {
		.src = node->addr,
		.dst = HB_ADDR, /* GROUP_ADDR, */
		.period = 0x10,
	};

	err = bt_mesh_cfg_hb_sub_set(net_idx, addr, &gw_hb_sub, NULL);
	if (err) {
		printk("\nERROR: Subscribing heartbeat messages failed NODE:  0x%04x\n", node->addr);
		return;
	}else{
		printk("\nSubscribing to NODE heartbeat messages:  0x%04x\n", node->addr);
	}
}



/* End of Define trap */
#endif