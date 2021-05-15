/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 *  @brief Bluetooth Mesh Sensor Server Model
 */

#include "sensor_srv.h"



static void sensor_col_get(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/** TODO: **/
}

static void sensor_series_get(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/** TODO: **/
}



static void sensor_desc_get(struct bt_mesh_model *model,
		       struct bt_mesh_msg_ctx *ctx,
		       struct net_buf_simple *buf)
{
	/** TODO: There is nowhere to get this info @ present (see Sensors Project on GIT)**/
	/* Size Messagese */
	// NET_BUF_SIMPLE_DEFINE(msg, 2 + MAX_SENS_STATUS_LEN + 4);
	NET_BUF_SIMPLE_DEFINE(msg, BT_MESH_SENSOR_DESCRIPTOR_GET_MSG_LEN);

	printk("sensor_get....\n");
	/* Pull the Sensor ID from Message Buffer */
	uint16_t sensor_id;
	sensor_id = net_buf_simple_pull_le16(buf);

	/* Call Status Message Creator in Mesh_Sensor.c */
	sensor_status_msg(sensor_id, &msg);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("ERROR: Unable to send Sensor get status response\n");
	}
}


static void sensor_get(struct bt_mesh_model *model,
		       struct bt_mesh_msg_ctx *ctx,
		       struct net_buf_simple *buf)
{

	struct bt_mesh_sensor_srv *srv = model->user_data;	

	printk("sensor_get recieved ....\n");

	/* Size Messagese */
	// NET_BUF_SIMPLE_DEFINE(msg, 2 + MAX_SENS_STATUS_LEN + 4);
	NET_BUF_SIMPLE_DEFINE(msg, BT_MESH_SENSOR_STATUS_MSG_LEN);

	/* Pull the Sensor ID from Message Buffer */
	uint16_t sensor_id = 0U;
	sensor_id = net_buf_simple_pull_le16(buf);

	/** TODO: Build the Status message inc HDR A & B */
	/* Using Callback Function */
	if (srv->cb && srv->cb->sensor_get_status) {
		srv->cb->sensor_get_status(sensor_id, &msg);
	} else {
		printk("No callback for getting sensor data");
	}	

	// /* Call Status Message Creator in Mesh_Sensor.c */
	// sensor_status_msg(sensor_id, &msg);

	if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("ERROR: Unable to send Sensor get status response\n");
	}
}



/* Mapping of message handlers for Sensor Server (0x1100) */
const struct bt_mesh_model_op sensor_srv_op[] = {
	{ BT_MESH_MODEL_OP_SENS_DESC_GET, 0, sensor_desc_get }, // Rx
	// { BT_MESH_MODEL_OP_SENS_DESC_STATUS, 2, sensor_desc_status }, // Tx
	{ BT_MESH_MODEL_OP_SENS_GET, 0, sensor_get }, // Rx
	// { BT_MESH_MODEL_OP_SENS_STATUS, 0, sensor_status_t }, // Tx
	{ BT_MESH_MODEL_OP_SENS_COL_GET, 0, sensor_col_get }, // Rx
	// { BT_MESH_MODEL_OP_SENS_COL_STATUS, 2, sensor_col_status }, // Tx
	{ BT_MESH_MODEL_OP_SENS_SERIES_GET, 0, sensor_series_get }, // Rx
	// { BT_MESH_MODEL_OP_SENS_SERIES_STATUS, 2, sensor_series_status }, // Tx
	BT_MESH_MODEL_OP_END,
};