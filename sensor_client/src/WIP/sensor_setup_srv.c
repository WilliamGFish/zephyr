/** @file
 *  @brief Bluetooth Mesh Sensor Setup Server Model
 */

/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <stdlib.h>
#include <errno.h>

#include "sensor_setup_srv.h"

int sensor_setting_attr_add(char *dev_name, uint16_t sensor_property_id,
			    uint16_t sensor_setting_property_id,
			    uint8_t sensor_setting_access,
			    struct sensor_value sensor_setting_val){

/** Check array position
 * resize array
 * add new element 
 */
printk("Prop ID: 0x%04x \t",sensor_setting_property_id);



	int added = -1;
	// int attr_cnt = sizeof(sensor_attr_list)/sizeof(sensor_attr_list[0]);
	int attr_cnt = ARRAY_SIZE(sensor_attr_list);


	for (int i = 0; i < attr_cnt; i++) {
		/* Add to the first empty slot in array or update to avoid duplicates */
		if (sensor_attr_list[i].dev_name == 0 ||
			    (sensor_attr_list[i].dev_name == 
					dev_name &&
			     sensor_attr_list[i]
					     .sensor_settings
					     .sensor_property_id ==
				    sensor_property_id &&
			     sensor_attr_list[i]
					     .sensor_settings
					     .sensor_setting_property_id ==
				    sensor_setting_property_id)
			) {
			sensor_attr_list[i].dev_name = dev_name;
			sensor_attr_list[i].sensor_settings.sensor_property_id =
				sensor_property_id;
			sensor_attr_list[i]
				.sensor_settings.sensor_setting_property_id =
				sensor_setting_property_id;
			sensor_attr_list[i]
				.sensor_settings.sensor_setting_access =
				sensor_setting_access;
			sensor_attr_list[i].sensor_settings.sensor_setting_val =
				sensor_setting_val;
			added = 1;
			printk("Slot: %d \t", i);
			printk("Prop ID: 0x%04x \t", sensor_attr_list[i].sensor_settings.sensor_property_id);
			printk("Attr ID: 0x%04x \t", sensor_attr_list[i].sensor_settings.sensor_setting_property_id);
			printk("Val1: %d \n", sensor_attr_list[i].sensor_settings.sensor_setting_val.val1);		
			return added;
		}
	}

printk("Slot: %d \n", attr_cnt);
	/** Resize the array if neccassary **/
	// if (added < 0){

	// 	// attr_cnt++;
	// 	sensor_attr_list[attr_cnt].dev_name = dev_name;
	// 	sensor_attr_list[attr_cnt].sensor_settings.sensor_property_id =
	// 		sensor_property_id;
	// 	sensor_attr_list[attr_cnt]
	// 		.sensor_settings.sensor_setting_property_id =
	// 		sensor_setting_property_id;
	// 	sensor_attr_list[attr_cnt]
	// 		.sensor_settings.sensor_setting_access =
	// 		sensor_setting_access;
	// 	sensor_attr_list[attr_cnt].sensor_settings.sensor_setting_val =
	// 		sensor_setting_val;
	// 	added = true;		
	// 	printk("Slot: %d \t", attr_cnt);
	// 	printk("Prop ID: 0x%04x \t", sensor_attr_list[attr_cnt].sensor_settings.sensor_property_id);
	// 	printk("Attr ID: 0x%04x \t", sensor_attr_list[attr_cnt].sensor_settings.sensor_setting_property_id);
	// 	printk("Val1: %d \n", sensor_attr_list[attr_cnt].sensor_settings.sensor_setting_val.val1);
	// }

	return added;
};


int sensor_setting_attr_update(){
	int err = 0;

	// int attr_cnt = sizeof(sensor_attr_list)/sizeof(sensor_attr_list[0]);
	int attr_cnt = ARRAY_SIZE(sensor_attr_list);
	
	for (int i = 0; i < attr_cnt; i++) {
		/* If no entry return */

		if (sensor_attr_list[i].dev_name){
     		printk("UPDATING...");
			printk("Slot: %d \t", i);
			printk("Device: %s\t", sensor_attr_list[i].dev_name);
			printk("Prop ID: 0x%04x \t", sensor_attr_list[i].sensor_settings.sensor_property_id);
			printk("Attr ID: 0x%04x \t", sensor_attr_list[i].sensor_settings.sensor_setting_property_id);
			printk("Val1: %d \n", sensor_attr_list[i].sensor_settings.sensor_setting_val.val1);	

			const struct device *sensor_dev =
				device_get_binding(sensor_attr_list[i].dev_name);

			err = sensor_attr_set(
				sensor_dev, SENSOR_CHAN_ALL,
				sensor_attr_list[i].sensor_settings.sensor_setting_property_id,
				&sensor_attr_list[i].sensor_settings.sensor_setting_val);
			if (err < 0) {
				printk("ERROR: Could not set sensor attribute.\n");
			}
		}
	}
	return err;
}


static void sensor_cadence_get(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/* Trap bad buffers */
	if (buf->len <= 0) {
		printk("ERROR: No Buffer Data (sensor_cadence_get).");
		return;
	}

	struct bt_mesh_sensor_setup_srv *srv = model->user_data;
	struct bt_mesh_sensor_cadence_data cadence_data;	

	/* Pull the Sensor ID from Message Buffer */
	cadence_data.property_id = net_buf_simple_pull_le16(buf);

	/* Pass the bt_mesh_sensor_cadence_data struct to cb function 
	* retunrs Sensor Cadence Status updated stucture */
	if (srv->cb && srv->cb->sensor_get_cadence) {
		if (srv->cb->sensor_get_cadence(&cadence_data)) {
			printk("Failed to get sensor cadance data.");			
		}
	} else {
		printk("No Current sensor_cadence_get callback available");
	}	

printk("sensor_cadence_get setup srv....\n");

	/* Size Messagese */
	// NET_BUF_SIMPLE_DEFINE(msg, 2 + 24 + 4);
	NET_BUF_SIMPLE_DEFINE(msg, MAX_SENS_STATUS_LEN);


	/* Initialise Model message */
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_SENS_CADENCE_STATUS);
	net_buf_simple_add_mem(&msg, &cadence_data, sizeof(cadence_data));

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("ERROR: Unable to send Sensor Cadence Status response\n");
	}
}










static void sensor_cadence_set(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/* TODO */
	/* Trap bad buffers */
	if (buf->len <= 0) {
		printk("ERROR: No Buffer Data (sensor_cadence_set).");
		return;
	}

	struct bt_mesh_sensor_setup_srv *srv = model->user_data;
	struct bt_mesh_sensor_cadence_data cadence_data;

	cadence_data.property_id = net_buf_simple_pull_le16(buf);

	/* TODO: Seporate the 2 fields */
	cadence_data.fast_cadence_period_divisor = net_buf_simple_pull_u8(buf);
	// cadence_data.status_trigger_type
	
	cadence_data.status_trigger_delta_down.val1 = net_buf_simple_pull_le16(buf);
	cadence_data.status_trigger_delta_down.val2 = net_buf_simple_pull_le16(buf);

	cadence_data.status_trigger_delta_up.val1 = net_buf_simple_pull_le16(buf);
	cadence_data.status_trigger_delta_up.val2 = net_buf_simple_pull_le16(buf);

	cadence_data.status_min_interval = net_buf_simple_pull_u8(buf);

	cadence_data.fast_cadence_low.val1 = net_buf_simple_pull_le16(buf);
	cadence_data.fast_cadence_low.val2 = net_buf_simple_pull_le16(buf);

	cadence_data.fast_cadence_high.val1 = net_buf_simple_pull_le16(buf);
	cadence_data.fast_cadence_high.val2 = net_buf_simple_pull_le16(buf);


	/* Pass the sensor_data structure to cb function */
	if (!srv->cb->sensor_set_cadence) {
		printk("No Current sensor_data callback available");
	}else
	{
		srv->cb->sensor_set_cadence(model, cadence_data);
	}	
}






static void sensor_settings_get(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
/**
Field				Size (octets)	Notes
Sensor Property ID		2			Property ID identifying a sensor.
**/

	/* Trap bad buffers */
	if (buf->len <= 0) {
		printk("ERROR: No Buffer Data (sensor_setting_get).");
		return;
	}

	struct bt_mesh_sensor_setup_srv *srv = model->user_data;
	struct bt_mesh_sensor_settings_status sensor_settings_status;	

	/* Pull the Sensor ID from Message Buffer */
	sensor_settings_status.sensor_property_id = net_buf_simple_pull_le16(buf);


	/* Pass the sensor_data structure to cb function */
	if (!srv->cb->sensor_get_settings) {
		printk("No Current sensor_settings_get callback available");
	}else
	{
		srv->cb->sensor_get_settings(&sensor_settings_status);
	}

	/* TODO */
	/* Size Messsage */
	// NET_BUF_SIMPLE_DEFINE(msg, 2 + 24 + 4);
	NET_BUF_SIMPLE_DEFINE(msg, MAX_SENS_STATUS_LEN);	

/** reply with sensor setting status message **/
	/* Initialise Model message */
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_SENS_SETTINGS_STATUS);
	net_buf_simple_add_mem(&msg, &sensor_settings_status, sizeof(sensor_settings_status));

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("ERROR: Unable to send Sensor Settings Status response\n");
	}	

/** Returns
 * Field							Size (octets)	Notes		
 * Sensor Property ID				2				Property ID identifying a sensor.
 * Sensor Setting Property IDs		2*N				
 * A sequence of N Sensor Setting Property IDs identifying settings within a sensor, 
 * where N is the number of property IDs included in the message. (Optional)
**/

}


static void sensor_setting_get(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
/**
 * Field							Size (octets)	Notes		
 * Sensor Property ID				2				Property ID identifying a sensor.
 * Sensor Setting Property IDs		2				Setting Property ID identifying a setting within a sensor.
**/


	/* Trap bad buffers */
	if (buf->len <= 0) {
		printk("ERROR: No Buffer Data (sensor_setting_get).");
		return;
	}

	struct bt_mesh_sensor_setup_srv *srv = model->user_data;
	struct bt_mesh_sensor_setting_status setting_status;	

	/* Pull the Sensor ID from Message Buffer */
	setting_status.sensor_property_id = net_buf_simple_pull_le16(buf);


	/* TODO */
	/* Pass the bt_mesh_sensor_cadence_data struct to cb function 
	* retunrs Sensor Setting Status updated stucture */
	if (srv->cb && srv->cb->sensor_get_setting) {
		if (srv->cb->sensor_get_setting(&setting_status)) {
			printk("ERROR: Failed to get sensor setting IDs (sensor_setting_get).");			
		}
	} else {
		printk("ERROR: No Current sensor_cadence_get callback available");
	}	

	/** TODO:
	* Check if setting_status invalid/empty 
	* Size Messsage 
	**/
	// NET_BUF_SIMPLE_DEFINE(msg, 2 + 24 + 4);
	NET_BUF_SIMPLE_DEFINE(msg, MAX_SENS_STATUS_LEN);

/** reply with sensor setting status message **/
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_SENS_SETTING_STATUS);
	net_buf_simple_add_mem(&msg, &setting_status, sizeof(setting_status));

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("ERROR: Unable to send Sensor Cadence Status response\n");
	}	
}


static void sensor_setting_set_unack(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/* Trap bad buffers */
	if (buf->len <= 0) {
		printk("ERROR: No Buffer Data (sensor_setting_set).");
		return;
	}

	int value_cnt = 0U;
	struct bt_mesh_sensor_setup_srv *srv = model->user_data;
	struct bt_mesh_sensor_setting_data setup_data;

	setup_data.sensor_property_id = net_buf_simple_pull_le16(buf);			// Pull Property ID identifying a sensor 
	setup_data.sensor_setting_property_id = net_buf_simple_pull_le16(buf);	// Pull Setting ID identifying a setting within a sensor

	value_cnt = buf->len / 4; /* Assess how many pairs of uint16_t elements exist (sensor_structure) */
	
	/* Loop through the buffer to extract values */
	for (int i = 0; i < value_cnt; i++)
	{
		if (buf->len >= 4){
			setup_data.sensor_setting_val.val1 = net_buf_simple_pull_le16(buf);		// Pull Sensor value1 for the setting
			setup_data.sensor_setting_val.val2 = net_buf_simple_pull_le16(buf);		// Pull Sensor value2 for the setting
		}
	}

	/* Pass the sensor_data structure to cb function */
	if (!srv->cb->sensor_set_setting) {
		printk("No Current sensor_data callback available");
	}else
	{
		srv->cb->sensor_set_setting(&setup_data);
	}	
}


static void sensor_setting_set(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/* Trap bad buffers */
	if (buf->len <= 0) {
		printk("ERROR: No Buffer Data (sensor_setting_set).");
		return;
	}

	/* Sensor Settting Set via UnAck Set fuction */
	sensor_setting_set_unack(model, ctx, buf);

	/* Send Sensor Seting Status Message */
	sensor_setting_get(model, ctx, buf);
}



/** Tramsition Messages functons **/


int bt_mesh_sensor_setting_status(struct bt_mesh_elem *elem,
			      struct bt_mesh_msg_ctx *ctx,
			      struct bt_mesh_sensor_setting_status *sensor_setting_status)
{
	/* TODO NOWHERE NEAR COMPLETE*/
	struct bt_mesh_model *mod;

	mod = bt_mesh_model_find(elem, BT_MESH_MODEL_ID_SENSOR_SETUP_SRV);
	if (!mod) {
		return -EINVAL;
	}

	// NET_BUF_SIMPLE_DEFINE(msg, 2 + MAX_SENS_STATUS_LEN + 4);
	NET_BUF_SIMPLE_DEFINE(msg, BT_MESH_SENSOR_SETTINGS_STATUS_MSG_LEN);

	/* Build message */
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_SENS_SETTING_STATUS);	

	// printk("ctx size: %d ", sizeof(&ctx));
	// printk(" -- msg size: %d \n", sizeof(&msg));

	int err = (bt_mesh_model_send(mod, ctx, &msg, NULL, NULL));
	if (err != 0) {
		printf("ERROR: Unable to send Sensor Setting Set: 0x%04x\n",
			sensor_setting_status->sensor_property_id);
		printf("ERROR: bt_mesh_sensor_setting_set Error ID: %d\n",
			err);
	} else {
		printf("MESSAGE: bt_mesh_sensor_setting_set message sent: 0x%04x",
			sensor_setting_status->sensor_property_id);
	}

	return err;	
}


/* Mapping of message handlers for Sensor Setup Server (0x1101) */
/* TODO update the function calls */
const struct bt_mesh_model_op sensor_setup_srv_op[] = {
	{ BT_MESH_MODEL_OP_SENS_CADENCE_GET, 0, sensor_cadence_get }, // Rx
	{ BT_MESH_MODEL_OP_SENS_CADENCE_SET, 0, sensor_cadence_set }, // Rx
	{ BT_MESH_MODEL_OP_SENS_CADENCE_SET_UNACK, 0, sensor_cadence_set }, // Rx
	// { BT_MESH_MODEL_OP_SENS_CADENCE_STATUS, 0, sensor_cadence_status }, // Tx
	{ BT_MESH_MODEL_OP_SENS_SETTINGS_GET, 0, sensor_settings_get }, // Rx
	// { BT_MESH_MODEL_OP_SENS_SETTINGS_STATUS, 0, sensor_settings_status }, // Tx
	{ BT_MESH_MODEL_OP_SENS_SETTING_GET, 0, sensor_setting_get }, // Rx
	{ BT_MESH_MODEL_OP_SENS_SETTING_SET, 0, sensor_setting_set }, // Rx
	{ BT_MESH_MODEL_OP_SENS_SETTING_SET_UNACK, 0, sensor_setting_set_unack}, // Rx
	// { BT_MESH_MODEL_OP_SENS_SETTING_STATUS, 0, sensor_setting_status }, // Tx
	BT_MESH_MODEL_OP_END,
};



