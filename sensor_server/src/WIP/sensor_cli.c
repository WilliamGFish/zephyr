/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/** @file
 *  @brief Bluetooth Mesh Sensor Client Model
 */

#include "sensor_cli.h"




/** ----------  STUBS as most of these are unable to return anything as the data is not available --------------- **/
static void sensor_settings_status(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/* TODO */
	/** Message Format 
	Field							Size (octets)		Notes
	Sensor Property ID					2				Property ID identifying a sensor.
	Sensor Setting Property IDs			2*N				A sequence of N Sensor Setting Property IDs identifying settings within a sensor, where N is the number of property IDs included in the message. (Optional)

	The Sensor Setting Property IDs field contains a sequence of all Sensor Setting Property ID states of a sensor
	**/

}


static void sensor_setting_status(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/* TODO */
	/** Message Format 
	Field							Size (octets)		Notes
	Sensor Property ID					2				Property ID identifying a sensor.
	Sensor Setting Property ID			2				Setting ID identifying a setting within a sensor
	Sensor Setting Access				1				Read / Write access rights for the setting. (Optional)
	Sensor Setting Raw				variable			Raw value for the setting.

	The Sensor Property ID field identifies a Sensor Property ID state of an element (see Section 4.1.2.1).
	The Sensor Setting Property ID field identifies a Sensor Setting Property ID state of a sensor (see Section 4.1.2.2).
	If present, the Sensor Setting Access field identifies a Sensor Setting Access state of a sensor (see Section 4.1.2.3).
	If present, the Sensor Setting Raw field identifies a Sensor Setting Raw state of a sensor

	**/

}

static void sensor_desc_status(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/* TODO */
}

static void sensor_col_status(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/* TODO */
}


static void sensor_series_status(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/* TODO */
}

/*----------------------------------*/

/** Sensor Cadence Status Message (Rx):
 * 
 **/
static void sensor_cadence_status(struct bt_mesh_model *model,
			      struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	/* TODO 
	need to understand how the varible length data will be handled.
	
	*/
	/* Trap bad buffers */
	if (buf->len <= 0) {
		printk("ERROR: No Buffer Data (sensor_cadence_status).");
		return;
	}

	uint8_t xformat = 0U;
	struct bt_mesh_sensor_cli *cli = model->user_data;
	struct bt_mesh_sensor_cadence_data cadence_data;

	cadence_data.property_id = net_buf_simple_pull_le16(buf);

	/* TODO: Seporate the 2 fields: fast_cadence_period_divisor (7bit) & status_trigger_type (bit) */
	xformat = net_buf_simple_pull_u8(buf);  

	// cadence_data.fast_cadence_period_divisor = (xformat >> 0) & 0x7F; 	/* Break out 7 bits for divisor */
	cadence_data.fast_cadence_period_divisor = (xformat >> 3) & 0x0F; 	/* Break out 4 bits for divisor therefore stays between 16-1 limits */
	cadence_data.status_trigger_type = (xformat >> 7) & 0x01; 				/* Break out 1 bit for Trigger Type */

	cadence_data.status_trigger_delta_down.val1 = net_buf_simple_pull_le16(buf);
	cadence_data.status_trigger_delta_down.val2 = net_buf_simple_pull_le16(buf);

	cadence_data.status_trigger_delta_up.val1 = net_buf_simple_pull_le16(buf);
	cadence_data.status_trigger_delta_up.val2 = net_buf_simple_pull_le16(buf);

	cadence_data.status_min_interval = net_buf_simple_pull_u8(buf);

	cadence_data.fast_cadence_low.val1 = net_buf_simple_pull_le16(buf);
	cadence_data.fast_cadence_low.val2 = net_buf_simple_pull_le16(buf);

	cadence_data.fast_cadence_high.val1 = net_buf_simple_pull_le16(buf);
	cadence_data.fast_cadence_high.val2 = net_buf_simple_pull_le16(buf);
   
	/* Pass the Cadence Settings structure to formatter */
	if (!cli->cb->sensor_cadence_update) {
		printk("No Cadence Settings Update  callback available");
	} else {
		cli->cb->sensor_cadence_update(model, cadence_data);
	}
}
 


/**
 * Deconstructs marshalled sensor data from status message
 **/
static void sensor_status(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	struct sensor_value val;
	uint16_t node_id = ctx->addr - model->elem_idx; // Get calling address

	/* Trap bad buffers */
	if (buf->len <= 0) {
		return;
	}

	struct bt_mesh_sensor_cli *cli = model->user_data;

	// uint8_t format_bit = (buf->data[0] >> 0) & 1;  // Get 1st bit of message buffer
	uint16_t xformat, xproperty_id = 0, xlength;
	int valuecnt = 1;
	// float lon, lat, ele = 0;

    /* Create Datum Array structure */
	struct bt_mesh_sensor_property_value datum;
	int property_cnt = 0;


	/* Set MQTT string buffer snprint pointer to 0 */
	int mqttcx = 0U;
	// /* if MQTT Defined write to FIFO buffer */
	// struct mqtt_data_t msg_data ;

	/* This loop covers the mutliple property sent in the same message. */
	while (buf->len > 0U) {

		uint8_t format_bit = (buf->data[0] >> 0) & 1;  // Get 1st bit of message buffer
		if (format_bit == 0U) {
			/* Header Format A
			 * size_t format = 1;
			 * size_t length = 4;
			 * size_t property_id = 11;
			 */
			xformat = net_buf_simple_pull_le16(buf);        // Pull header format and length data

			/* Break out 4 bits for Data Length */
			xlength = (xformat >> 1) & 0xf;

			/* Break out 11 bits for Propery ID */
			xproperty_id = (xformat >> 5) & 0x7ff;

			/* Define how many uint16_t elements exist */
			valuecnt = xlength / 2;
		} else {
			/* Header Format B
			 * size_t format = 1;
			 * size_t length = 7;
			 * size_t property_id = 16;
			 */
			xformat = net_buf_simple_pull_u8(buf);          // Pull header format and length data
			xproperty_id = net_buf_simple_pull_le16(buf);       // Pull Property ID

			/* Break out 7 bits for Data Length */
			xlength = (xformat >> 1) & 0x7f;

			/* Define how many pairs of uint16_t elements exist (sensor_structure) */
			valuecnt = xlength / 4;
		}

        /* Build datum array */
        datum.node_id = node_id;
		datum.property_id = xproperty_id;

		/* Create JSON Output header */
		// mqttcx += snprintf(mqttbuf + mqttcx, 150, "sensor:-nodeID:%x  propID:%04x", node_id, xproperty_id );

		/* Build MQTT Topic */
		// msg_data.deviceID = node_id;
		// msg_data.propertyID = xproperty_id;

		int cnt = 0U;
		while (cnt < valuecnt) {

			// TODO: Get sensor value name
			// string sensorname[valuecnt];
			// namestring[] = get_sensor_name(xproperty_id);

			if (format_bit == 0U) {
				// Using Sensor Value Structure
				val.val1 = net_buf_simple_pull_le16(buf);
				val.val2 = 0U;

				/* Build datum array */
				datum.val[cnt] = val;

				// Build JSON Output
				// mqttcx += snprintf(mqttbuf + mqttcx, 25,"  value: %d", val.val1);

			} else 
			{
				// Using Sensor Value Structure
				val.val1 = net_buf_signed_pull_le16(buf);
				val.val2 = net_buf_signed_pull_le16(buf) * 1000; // 3 decimal places

                /* Build datum array */         
                datum.val[cnt] = val;

				// float sensor_float = sensor_value_to_float(&val);

				// Build JSON Output
				if (cnt > 0U){
					// mqttcx += snprintf(mqttbuf + mqttcx, 20,"  value%d:%.3f", cnt, sensor_float);
				} 
				else
				{
					// mqttcx += snprintf(mqttbuf + mqttcx, 20, "  value:%.3f", sensor_float);
				}
			}

			cnt++;
		}
		property_cnt++;

		/* Pass the DATUM structure to formatter */
		if (!cli->cb->sensor_data_format) {
			printk("No Current Status callback available");
		}else
		{
			cli->cb->sensor_data_format(datum, valuecnt);
		}
		
		// sensor_data_format(datum, cnt);

		// Close JSON Output
		// printk("%s : %d \n", mqttbuf, mqttcx);
// #if defined(CONFIG_MODEM)
// 		/* if MQTT Defined write to FIFO buffer */
// 		strcpy(msg_data.message, mqttbuf);
// 		k_fifo_put(&mqtt_fifo, &msg_data);
// #endif		
		
        /* Reset snprint buffer pointer to 0 */
		mqttcx = 0U;

	} // End of buffer While loop
}



/** Message Transmision Functions **/

/** Sensor Setting Set (UnAck) Message:
 * @param bt_mesh_elem
 * @param bt_mesh_msg_ctx
 * @param bt_mesh_sensor_setting_data
 * 
 **/
int bt_mesh_sensor_setting_set(struct bt_mesh_elem *elem,
			      struct bt_mesh_msg_ctx *ctx,
			      struct bt_mesh_sensor_setting_data *setting_data)
{
	struct bt_mesh_model *mod;

	if (!setting_data) {
		printk("ERROR: bt_mesh_sensor_setting_set Error ID: %d\n",
		       EINVAL);
		return -EINVAL;
	}

	mod = bt_mesh_model_find(elem, BT_MESH_MODEL_ID_SENSOR_CLI);
	if (!mod) {
		return -EINVAL;
	}

	NET_BUF_SIMPLE_DEFINE(msg, BT_MESH_SENSOR_SETTING_SET_MSG_LEN);

	/* Build message */
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_SENS_SETTING_SET);	

	net_buf_simple_add_le16(&msg, setting_data->sensor_property_id );
	net_buf_simple_add_le16(&msg, setting_data->sensor_setting_property_id ); 

	net_buf_simple_add_le16(&msg, setting_data->sensor_setting_val.val1 ); 
	net_buf_simple_add_le16(&msg, setting_data->sensor_setting_val.val2 ); 

	// printk("ctx size: %d ", sizeof(&ctx));
	// printk(" -- msg size: %d \n", sizeof(&msg));

	int err = (bt_mesh_model_send(mod, ctx, &msg, NULL, NULL));
	if (err != 0) {
		printk("ERROR: Unable to send Sensor Setting Set: 0x%04x\n",
			setting_data->sensor_property_id);
		printk("ERROR: bt_mesh_sensor_setting_set Error ID: %d\n",
			err);
	} else {
		printk("MESSAGE: bt_mesh_sensor_setting_set message sent: 0x%04x \n",
			setting_data->sensor_property_id);
	}

	return err;	
}

int bt_mesh_sensor_setting_set_unack(struct bt_mesh_elem *elem,
			      struct bt_mesh_msg_ctx *ctx,
			      struct bt_mesh_sensor_setting_data *setting_data)
{
	/* TODO */
	struct bt_mesh_model *mod;

	mod = bt_mesh_model_find(elem, BT_MESH_MODEL_ID_SENSOR_CLI);
	if (!mod) {
		return -EINVAL;
	}

	NET_BUF_SIMPLE_DEFINE(msg, BT_MESH_SENSOR_SETTING_SET_MSG_LEN);

	/* Build message */
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_SENS_SETTING_SET_UNACK);	

	net_buf_simple_add_le16(&msg, setting_data->sensor_property_id );
	net_buf_simple_add_le16(&msg, setting_data->sensor_setting_property_id ); 

	net_buf_simple_add_le16(&msg, setting_data->sensor_setting_val.val1 ); 
	net_buf_simple_add_le16(&msg, setting_data->sensor_setting_val.val2 ); 

	// printk("ctx size: %d ", sizeof(&ctx));
	// printk(" -- msg size: %d \n", sizeof(&msg));

	int err = (bt_mesh_model_send(mod, ctx, &msg, NULL, NULL));
	if (err != 0) {
		printk("ERROR: Unable to send Sensor Setting Set: 0x%04x\n",
			setting_data->sensor_property_id);
		printk("ERROR: bt_mesh_sensor_setting_set Error ID: %d\n",
			err);
	} else {
		printk("MESSAGE: bt_mesh_sensor_setting_set message sent: 0x%04x",
			setting_data->sensor_property_id);
	}

	return err;	
}





/** Sensor Cadence Set (UnAck) Message:
 * @param bt_mesh_elem
 * @param bt_mesh_msg_ctx
 * @param bt_mesh_sensor_cadence_data
 * 
 **/
int bt_mesh_sensor_cadence_set(struct bt_mesh_elem *elem,
			      struct bt_mesh_msg_ctx *ctx,
			      struct bt_mesh_sensor_cadence_data *cadence_data)
{
	struct bt_mesh_model *mod;

	mod = bt_mesh_model_find(elem, BT_MESH_MODEL_ID_SENSOR_CLI);
	if (!mod) {
		return -EINVAL;
	}

	NET_BUF_SIMPLE_DEFINE(msg, BT_MESH_SENSOR_CADENCE_SET_MSG_LEN);

	/* Build message */
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_SENS_CADENCE_SET);	

	net_buf_simple_add_le16(&msg, cadence_data->property_id);

	/* TODO: Compine the 2 fields */
	net_buf_simple_add_u8(&msg, cadence_data->fast_cadence_period_divisor);
	// cadence_data->status_trigger_type
	
	net_buf_simple_add_le16(&msg, cadence_data->status_trigger_delta_down.val1);
	net_buf_simple_add_le16(&msg, cadence_data->status_trigger_delta_down.val2);
	net_buf_simple_add_le16(&msg, cadence_data->status_trigger_delta_up.val1);
	net_buf_simple_add_le16(&msg, cadence_data->status_trigger_delta_up.val2);
	net_buf_simple_add_u8(&msg, cadence_data->status_min_interval);
	net_buf_simple_add_le16(&msg, cadence_data->fast_cadence_low.val1);
	net_buf_simple_add_le16(&msg, cadence_data->fast_cadence_low.val2);
	net_buf_simple_add_le16(&msg, cadence_data->fast_cadence_high.val1);
	net_buf_simple_add_le16(&msg, cadence_data->fast_cadence_high.val2);


	// printk("ctx size: %d ", sizeof(&ctx));
	// printk(" -- msg size: %d \n", sizeof(&msg));

	int err = (bt_mesh_model_send(mod, ctx, &msg, NULL, NULL));
	if (err != 0) {
		printk("ERROR: Unable to send Sensor Cadence Set: 0x%04x\n",
			cadence_data->property_id);
		printk("ERROR: bt_mesh_sensor_cadence_set Error ID: %d\n",
			err);
	} else {
		printk("MESSAGE: bt_mesh_sensor_cadence_set message sent: 0x%04x",
			cadence_data->property_id);
	}
	
	return err;	
}

int bt_mesh_sensor_cadence_set_unack(struct bt_mesh_elem *elem,
			      struct bt_mesh_msg_ctx *ctx,
			      struct bt_mesh_sensor_cadence_data *cadence_data)
{
	/* TODO */
	struct bt_mesh_model *mod;

	mod = bt_mesh_model_find(elem, BT_MESH_MODEL_ID_SENSOR_CLI);
	if (!mod) {
		return -EINVAL;
	}

	NET_BUF_SIMPLE_DEFINE(msg, BT_MESH_SENSOR_CADENCE_SET_MSG_LEN);

	/* Build message */
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_SENS_CADENCE_SET_UNACK);	

	// printk("ctx size: %d ", sizeof(&ctx));
	// printk(" -- msg size: %d \n", sizeof(&msg));

	int err = (bt_mesh_model_send(mod, ctx, &msg, NULL, NULL));
	if (err != 0) {
		printk("ERROR: Unable to send Sensor Setting Set: 0x%04x\n",
			cadence_data->property_id);
		printk("ERROR: bt_mesh_sensor_setting_set Error ID: %d\n",
			err);
	} else {
		printk("MESSAGE: bt_mesh_sensor_setting_set message sent: 0x%04x",
			cadence_data->property_id);
	}

	return err;	
}





/* Mapping of message handlers for Sensor Client (0x1102) */
const struct bt_mesh_model_op sensor_cli_op[] = {
	// { BT_MESH_MODEL_OP_SENS_DESC_GET, 0, sensor_desc_get }, // Tx
	// { BT_MESH_MODEL_OP_SENS_GET, 0, sensor_get }, // Tx
	// { BT_MESH_MODEL_OP_SENS_COL_GET, 0, sensor_col_get }, // Tx
	// { BT_MESH_MODEL_OP_SENS_SERIES_GET, 2, sensor_series_get }, // Tx

	// { BT_MESH_MODEL_OP_SENS_CADENCE_GET, 0, sensor_cadence_get_t }, // Tx
	// { BT_MESH_MODEL_OP_SENS_SETTING_GET, 0, sensor_setting_get }, // Tx
	// { BT_MESH_MODEL_OP_SENS_SETTINGS_GET, 0, sensor_settings_get }, // Tx

	{ BT_MESH_MODEL_OP_SENS_DESC_STATUS, 1, sensor_desc_status }, // Rx
	{ BT_MESH_MODEL_OP_SENS_STATUS, 0, sensor_status }, // Rx
	{ BT_MESH_MODEL_OP_SENS_COL_STATUS, 1, sensor_col_status }, // Rx
	{ BT_MESH_MODEL_OP_SENS_SERIES_STATUS, 1, sensor_series_status }, // Rx

	// { BT_MESH_MODEL_OP_SENS_CADENCE_SET, 0, sensor_cadence_set }, // Tx
	// { BT_MESH_MODEL_OP_SENS_CADENCE_SET_UNACK, 0, sensor_cadence_set }, // Tx
	{ BT_MESH_MODEL_OP_SENS_CADENCE_STATUS, 0, sensor_cadence_status }, // Rx
	{ BT_MESH_MODEL_OP_SENS_SETTINGS_STATUS, 0, sensor_settings_status }, // Rx
	// { BT_MESH_MODEL_OP_SENS_SETTING_SET, 1, bt_mesh_sensor_setting_set }, // Tx
	// { BT_MESH_MODEL_OP_SENS_SETTING_SET_UNACK, 1, bt_mesh_sensor_setting_set_unack }, // Tx
	{ BT_MESH_MODEL_OP_SENS_SETTING_STATUS, 0, sensor_setting_status }, // Rx
	BT_MESH_MODEL_OP_END,
};