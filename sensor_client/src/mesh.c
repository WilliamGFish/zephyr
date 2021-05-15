/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 */
#ifndef _MANULYTICA_MESH_C_
#define _MANULYTICA_MESH_C_


#include <zephyr.h>
#include <sys/printk.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <bluetooth/uuid.h>
#include <sys/__assert.h>


/* Application Files */
#include "mesh.h"
#include "mesh_conf.c"
#include "misc.c"

/* Sensor Config Files */
#include "mesh_sensor.c"
#include "WIP/sensor_cli.c"
#include "WIP/sensor_srv.c"
#include "WIP/sensor_setup_srv.c"


/* VENDOR BUTTON OPCODE DEFINE */
#define OP_VENDOR_BUTTON BT_MESH_MODEL_OP_3(0x00, BT_COMP_ID_LF)

static void heartbeat(uint8_t hops, uint16_t feat)
{
	board_heartbeat(hops, feat);
	printk("%u hops\n", hops);
}

/* Configure Mesh Config Server  */
// static struct bt_mesh_cfg_srv cfg_srv = {
// #if defined(CONFIG_BOARD_BBC_MICROBIT)
// 	// .relay = BT_MESH_RELAY_DISABLED,
// 	.beacon = BT_MESH_BEACON_ENABLED,	
// 	// .beacon = BT_MESH_BEACON_DISABLED,

// 	/* 3 transmissions with 20ms interval */
// 	.net_transmit = BT_MESH_TRANSMIT(1, 20),
// 	// .relay_retransmit = BT_MESH_TRANSMIT(1, 20),

// #else
// 	// .relay = BT_MESH_RELAY_ENABLED,
// 	// .relay = BT_MESH_RELAY_DISABLED,
// 	.beacon = BT_MESH_BEACON_ENABLED,

// 	/* 3 transmissions with 20ms interval */
// 	.net_transmit = BT_MESH_TRANSMIT(1, 20),
// 	// .relay_retransmit = BT_MESH_TRANSMIT(1, 20),

// #endif
// 	.frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
// 	.default_ttl = 7,

// 	.hb_sub.func = heartbeat,
// };

/* Configure Mesh Config Client  */
static struct bt_mesh_cfg_cli cfg_cli = {
};


/* =========================== Health Server Calback Functions ================================================= */
static uint8_t cur_srv_faults[CUR_FAULTS_MAX];
static uint8_t reg_srv_faults[CUR_FAULTS_MAX * 2];

static void attention_on(struct bt_mesh_model *model)
{
	printk("attention_on()\n");
	board_attention(true);
}

static void attention_off(struct bt_mesh_model *model)
{
	printk("attention_off()\n");
	board_attention(false);
}

static void get_health_faults(uint8_t *faults, uint8_t faults_size, uint8_t *dst, uint8_t *count)
{
	uint8_t i, limit = *count;

	for (i = 0U, *count = 0U; i < faults_size && *count < limit; i++) {
		if (faults[i]) {
			*dst++ = faults[i];
			(*count)++;
		}
	}
}

static int health_fault_get_cur(struct bt_mesh_model *model, uint8_t *test_id,
				uint16_t *company_id, uint8_t *faults, uint8_t *fault_count)
{
	get_health_faults(cur_srv_faults, sizeof(cur_srv_faults), faults, fault_count);
	return 0;
}

static int health_fault_get_reg(struct bt_mesh_model *model, uint16_t cid,
				uint8_t *test_id, uint8_t *faults, uint8_t *fault_count)
{
	if (cid != BT_COMP_ID_LF) {
		printk("Faults requested for unknown Company ID"
		       " 0x%04x", cid);
		return -EINVAL;
	}

	printk("Sending registered faults \n");

	*test_id = 0x00;
	get_health_faults(reg_srv_faults, sizeof(reg_srv_faults), faults, fault_count);
	return 0;
}

static int health_fault_clear(struct bt_mesh_model *model, uint16_t cid)
{
	if (cid != BT_COMP_ID_LF) {
		return -EINVAL;
	}

	(void)memset(reg_srv_faults, 0, sizeof(reg_srv_faults));
	return 0;
}

static int health_fault_test(struct bt_mesh_model *model, uint8_t test_id,
			     uint16_t cid)
{
	if (cid != BT_COMP_ID_LF) {
		return -EINVAL;
	}

	if (test_id != 0x00) {
		return -EINVAL;
	}

	return 0;
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
	.fault_test = health_fault_test,
	.fault_clear = health_fault_clear,
	.fault_get_cur = health_fault_get_cur,
	.fault_get_reg = health_fault_get_reg,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};
/* ===================================Health Server Callbacks END ================================= */


/* =========================     HEALTH CLIENT (APP SIDE) ========================================== */
void show_faults(uint8_t test_id, uint16_t cid, uint8_t *faults, size_t fault_count)
{
	int i;

	if (!fault_count) {
		printk("Health Test ID 0x%02x Company ID "
		       "0x%04x: No Faults\n", test_id, cid);
		return;
	}

	printk("Health Test ID 0x%02x Company ID 0x%04x Fault "
	       "Count %u:\n", test_id, cid, fault_count);

	for (i = 0; i < fault_count; i++) {
		printk("\t0x%02x", faults[i]);
	}
	printk("\n\n");
}


static void health_status_current(struct bt_mesh_health_cli *cli, uint16_t addr,
				  uint8_t test_id, uint16_t cid, uint8_t *faults,
				  size_t fault_count)
{
	printk("Health Current Status from 0x%04x \t", addr);
	show_faults(test_id, cid, faults, fault_count);
}


static struct bt_mesh_health_cli health_cli = {
	.current_status = health_status_current,
};
/* =========================     HEALTH CLIENT (APP SIDE) end ========================================== */


/*---------------------------- SENSOR CLI Callback Functions (APP SIDE) -------------------------------------*/

/* Sensor Client CB Function to Format Marshalled Data */
int format_marshalled_data(struct bt_mesh_sensor_property_value sensor_data, int property_count)
{
	/* Data message buffer */
	char str_buf[100];
	int i = 0U;

	/* code */
	// printk("COUNT: %d \t", property_count);
	printk("NodeID: %04x ", sensor_data.node_id);
	printk("PropertyID: %04x ", sensor_data.property_id);

	while (i < property_count) {
		/* code */
		float sensor_float = sensor_value_to_float(&sensor_data.val[i]);

		snprintf(str_buf, 50, " Value%d: %.3f \t", i,
			 sensor_float);
		printk("%s", str_buf);

		i++;
	}

	printk("\n");
	return 0;
}


/* Sensor Client CB Function to Update Sensor Cadence Setttings */
int model_divisor_update(struct bt_mesh_model *mod, uint16_t period_divisor)
{
	/* Update publication period with divisor if fast cadence set */
	int period;
	int err = 0U;

	/* Trap no publish attached */
	if (!mod->pub) {
		return 0U;
	}

	// Convert period but To work divisor has to be 4bit (see sensor_cli)
	switch (mod->pub->period >> 6) {
	case 0x00:
		// 1 step is 100 ms
		// period = K_MSEC((mod->pub->period & BIT_MASK(6)) * 100U);
		period = ((mod->pub->period & BIT_MASK(6)) * 100U);
		break;
	case 0x01:
		// 1 step is 1 second
		// period = K_SECONDS((mod->pub->period & BIT_MASK(6)));
		period = ((mod->pub->period & BIT_MASK(6)) * MSEC_PER_SEC);
		break;
	case 0x02:
		// 1 step is 10 seconds
		// period = K_SECONDS((mod->pub->period & BIT_MASK(6)) * 10U);
		period = ((mod->pub->period & BIT_MASK(6)) * 10U) * MSEC_PER_SEC;
		break;
	case 0x03:
		// 1 step is 10 minutes
		// period = K_MINUTES((mod->pub->period & BIT_MASK(6)) * 10U);
		period = ((mod->pub->period & BIT_MASK(6)) * 60U * 10U) * MSEC_PER_SEC;
		break;
	default:	
		CODE_UNREACHABLE;
		return -ENXIO;
	}

	if (mod->pub->fast_period) {
		// return period >> mod->pub->period_div;
		mod->pub->period = period >> period_divisor;
	}

	return err;
}
/*---------------------------- SENSOR CLI Callback Functions (APP SIDE) END ---------------------------------*/



/* =================================== Sensor srv Callback UPDATE Functions ================================= */
/* Sensor srv Update Callback */
static int sensor_env_update(struct bt_mesh_model *model)
{
	struct net_buf_simple *msg = model->pub->msg;

	/* Update publshing frequncy */
	// if (model->pub->period != sensor_srv_period) {
	//      model->pub->period = sensor_srv_period;
	// }

	/* Build message for SENSOR SRV MODEL CB get sensor readings */
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS);

	uint16_t sensor_id = 0;

	/* TODO: Add a comparison to reduce meessage, count to N identical messages before sending another */

	/* Get ENVIROMENT Values includes Temp and Accel XYZ */

	// sensor_id = SENS_PROP_ID_ENVIRO; // MPU Temp Data
	// sensor_id = SENS_PROP_ID_EXT_PRESSURE; // External ADC Sensor Data
	// sensor_status_msg(sensor_id, msg);

	/* Get Accelerometer Values */
	// sensor_id = SENS_PROP_ID_ACCEL_XYZ; // Accel X Value (m/s)
	// sensor_status_msg(sensor_id, msg);

	/* Get COUNT Values */
	// sensor_id = SENS_PROP_ID_EXT_CNT_TOTAL; // Total Count Value (int)
	// sensor_status_msg(sensor_id, msg);

	// // /* Get HUMIDITY Values */
	sensor_id = SENS_PROP_ID_HUMIDITY; // Humidity Level
	sensor_status_msg(sensor_id, msg);


	/* TODO: Error trapping stuff, return errno from sensor_status_msg */
	int rc = 0;
	// int rc = sensor_status_msg(sensor_id, msg);
	// if (rc != 0){
	//      printk("ERROR: bt_mesh_srv_update Error ID: %d\n", rc);
	// }
	// else
	// {
	//      printk("Sensor message updated with OpCode: 0x%04x \n", sensor_id);
	// }

	return rc;
}

/* Sensor srv Update Callback */
static int sensor_tmp_update(struct bt_mesh_model *model)
{
     struct net_buf_simple *msg = model->pub->msg;

     /* Update publshing frequncy */
     // if (model->pub->period != sensor_srv_period) {
     //      model->pub->period = sensor_srv_period;
     // }

     /* Build message for SENSOR SRV MODEL CB get sensor readings */
     bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS);

     /* Get Temperature Value */
     uint16_t sensor_id = SENS_PROP_ID_TEMP;
     sensor_status_msg(sensor_id, msg);

     /* TODO: Error trapping stuff, return errno from sensor_status_msg */
     int rc = 0;

     return rc;
}

/* Sensor srv Update Callback */
static int sensor_cnt_update(struct bt_mesh_model *model)
{
	struct net_buf_simple *msg = model->pub->msg;

	/* Update publshing frequncy */
	// if (model->pub->period != sensor_srv_period) {
	//      model->pub->period = sensor_srv_period;
	// }

	/* Build message for SENSOR SRV MODEL CB get sensor readings */
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS);

	/* Get Total Count Value */
	uint16_t sensor_id = SENS_PROP_ID_EXT_CNT_TOTAL;
	sensor_status_msg(sensor_id, msg);

	/* TODO: Error trapping stuff, return errno from sensor_status_msg */
	int rc = 0;

	return rc;
}


/* =================================== Sensor srv Callback Functions ================================= */
int sensor_get_status_msg(uint16_t id, struct net_buf_simple *msg)
{
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS);

	switch (id) {
	/* Send Enviroment Information */
	case SENS_PROP_ID_ENVIRO:
		// printk("Getting status for Sensor ID: 0x%04x \n", SENS_PROP_ID_ENVIRO);
		sensor_enviro_msg(msg);
		break;
	/* Ext Count Sensor */
	case SENS_PROP_ID_COUNTER:
		// printk("Getting status for Sensor ID: 0x%04x \n", SENS_PROP_ID_COUNTER);
		sensor_counter_msg(msg);
		break;
	case SENS_PROP_ID_EXT_CNT_TOTAL:
		// printk("Getting status for Sensor ID: 0x%04x \n", SENS_PROP_ID_EXT_CNT_TOTAL);
		sensor_counter_total_msg(msg);
		break;
	/* Ambient Temp Sensor */
	case SENS_PROP_ID_TEMP:
		// printk("Getting status for Sensor ID: 0x%04x \n", SENS_PROP_ID_TEMP);
		sensor_temp_msg(msg);
		break;
	/* Accelorometer Sensor */
	case SENS_PROP_ID_ACCEL_X:
		// printk("Getting status for Sensor ID: 0x%04x \n", SENS_PROP_ID_ACCEL_X);
		sensor_accel_msg(msg, ACCEL_X);
		break;
	case SENS_PROP_ID_ACCEL_Y:
		// printk("Getting status for Sensor ID: 0x%04x \n", SENS_PROP_ID_ACCEL_Y);
		sensor_accel_msg(msg, ACCEL_Y);
		break;
	case SENS_PROP_ID_ACCEL_Z:
		// printk("Getting status for Sensor ID: 0x%04x \n", SENS_PROP_ID_ACCEL_Z);
		sensor_accel_msg(msg, ACCEL_Z);
		break;
	case SENS_PROP_ID_ACCEL_XYZ:
		// printk("Getting status for Sensor ID: 0x%04x \n", SENS_PROP_ID_ACCEL_Z);
		sensor_accel_msg(msg, ACCEL_XYZ);
		break;

	/* Send Bad Call Unknown Message */
	default:
		sens_unknown_msg(id, msg);
		printk("ERROR: Unable to find Sensor status 0x%04x \n", id);
		break;
		// return -EINVAL;
	}
	/*
	 * When the message is a response to a Sensor Get message that
	 * identifies a sensor property that does not exist on the element, the
	 * Length field shall represent the value of zero and the Raw Value for
	 * that property shall be omitted. (Mesh model spec 1.0, 4.2.14)
	 */
	return 0;
}
/* =================================== Sensor srv Callback Functions END ================================= */




/* =================================== Sensor Setup srv Callback Functions ================================= */

/* Get sensor cadence status */
int get_sensor_cadence(struct bt_mesh_sensor_cadence_data *settings_cadence_data)
{
	/** TODO: **/
	/* Get the Sensor_cadence_data */
	return 0U;
};

int set_sensor_cadence(struct bt_mesh_model *mod, struct bt_mesh_sensor_cadence_data cadence_data)
{

#if !defined(CONFIG_BOARD_BBC_MICROBIT)	&& !defined(CONFIG_BOARD_ESP32) && !defined(CONFIG_BOARD_ARDUINO_NANO_33_BLE)
	/* TODO:
	   Need a method to look up BT UUID in Zephyr to fix many issues
	   If redefining external sensor type then need to:
	                force reconfiguration of sensor (ADC or GPIO)
	                re-initialise sensor
	                set-config as default/per message
	                check readings
	        if the property ID is counter then change to GPIO
	 */

	/* Set adc_cadence_data values to those passed in message */
	uint16_t id = cadence_data.property_id;

	switch (id) {
	/* BT_UUID_TEMPERATURE */
	case SENS_PROP_ID_EXT_PRESSURE:
		printk("Updating (BT_UUID_PRESSURE) Sensor Propeerty ID: 0x%04x \n", id);
		adc_cadence_data = cadence_data;
		break;
	/* Ext Count Sensor */
	case SENS_PROP_ID_EXT_TEMP:
		printk("Updating (BT_UUID_TEMPERATURE) Sensor Propeerty ID: 0x%04x \n", id);
		adc_cadence_data = cadence_data;
		break;
	case SENS_PROP_ID_COUNTER:
	case SENS_PROP_ID_EXT_CNT_TOTAL:
		printk("Updating (SENS_PROP_ID_COUNTER) Sensor Propeerty ID: 0x%04x \n", id);

		/* Set input to GPIO NOT ADC !! */
		adc_cadence_data = cadence_data;
		break;
	/* Send Bad Call Unknown Message */
	default:
		printk("ERROR: Unable to find Sensor Property ID 0x%04x \n", id);
		return -ENODEV;
		/* TODO: May need to send BAD ID message */
	}

	if (!adc_reset()) {
		printk("ERROR: Unable to Reset ADC for Sensor Property ID 0x%04x \n", id);
		return -EBUSY;
	}
#endif /* End of CONFIG_BOARD_BBC_MICROBIT */

	return 0U;
};



/* Get_sensor_settings status */
int get_sensor_settings(struct bt_mesh_sensor_settings_status *sensor_settings_ids)
{
	/** TODO: **/
	/** Build Function to return Sensor Server Property IDs
	 * This should be build as part of the initilisation process and amended during settings updates !!!!
	 *
	 * Field						Size (octets)	Notes
	 * Sensor Property ID				2			Property ID identifying a sensor.
	 * Sensor Setting Property IDs		2*N			A sequence of N Sensor Setting Property IDs identifying settings within a sensor, where N is the number of property
	 */

	/* Loop through list of Sensor Property List */

	return 0U;
};



/* Get_sensor_setting status */
int get_sensor_setting(struct bt_mesh_sensor_setting_status *sensor_setting_status)
{
	int err = 0U;

	/** TODO: **/
	/* Read the Sensor_Setting_status
	   struct bt_mesh_sensor_setting_status {
	   uint16_t sensor_property_id;         // Property ID identifying a sensor
	   uint16_t sensor_setting_property_id; // Setting ID identifying a setting within a sensor
	   uint8_t  sensor_setting_access;      // Read/Write access rights for the setting (optional set to 0x0)
	   struct sensor_value sensor_setting_val; // Sensor value for the setting
	 */

	// int attr_cnt = ARRAY_SIZE(sensor_attr_list);
	int attr_cnt = sizeof(*sensor_attr_list);

	for (int i = 0; i < attr_cnt; i++) {
		/* If no entry return */
		if (sensor_attr_list[i].sensor_settings.sensor_property_id
		    == sensor_setting_status->sensor_property_id
		    && sensor_attr_list[i].sensor_settings.sensor_setting_property_id
		    == sensor_setting_status->sensor_setting_property_id) {

			/* Return the stored Sensor Server Attr Value */
			sensor_setting_status->sensor_setting_val =
				sensor_attr_list[i]
				.sensor_settings.sensor_setting_val;

			sensor_setting_status->sensor_setting_access =
				sensor_attr_list[i]
				.sensor_settings.sensor_setting_access;

			printk("UPDATED...");
			printk("Prop ID: 0x%04x \t",
			       sensor_attr_list[i]
			       .sensor_settings.sensor_property_id);
			printk("Attr ID: 0x%04x \t",
			       sensor_attr_list[i]
			       .sensor_settings
			       .sensor_setting_property_id);
			printk("Val1: %d \n",
			       sensor_attr_list[i]
			       .sensor_settings.sensor_setting_val.val1);

			/* Once updated no need to carry on */
			return err;
		}
	}

	return -EINVAL;
};



/* Set sensor setting status */
int set_sensor_setting(struct bt_mesh_sensor_setting_data *setup_data)
{
	int err = 0U;

	/** TODO: **/
	/**
	 * This could use sensor_attr_set for sensor devices
	 *
	 * For ADC a Atribute List will be created ::: TODO
	 *
	 *     uint16_t sensor_property_id;                        Property ID identifying a sensor
	 *     uint16_t sensor_setting_property_id;                Setting ID identifying a setting within a sensor
	 *     struct sensor_value sensor_setting_val;  Sensor value for the setting
	 *
	 **/

	// int attr_cnt = ARRAY_SIZE(sensor_attr_list);
	int attr_cnt = sizeof(*sensor_attr_list);

	for (int i = 0; i < attr_cnt; i++) {
		/* If no entry return */
		if (sensor_attr_list[i].sensor_settings.sensor_property_id
		    == setup_data->sensor_property_id
		    && sensor_attr_list[i].sensor_settings.sensor_setting_property_id
		    == setup_data->sensor_setting_property_id) {

			printk("UPDATING...");
			printk("Slot: %d \t", i);
			printk("Device: %s\t", sensor_attr_list[i].dev_name);
			printk("Prop ID: 0x%04x \t",
			       sensor_attr_list[i]
			       .sensor_settings.sensor_property_id);
			printk("Attr ID: 0x%04x \t",
			       sensor_attr_list[i]
			       .sensor_settings
			       .sensor_setting_property_id);
			printk("Val1: %d \n",
			       sensor_attr_list[i]
			       .sensor_settings.sensor_setting_val.val1);

			/* Update the Sensor Server Attr List */
			sensor_attr_list[i].sensor_settings.sensor_setting_val =
				setup_data->sensor_setting_val;

			printk("UPDATED...");
			printk("Prop ID: 0x%04x \t",
			       sensor_attr_list[i]
			       .sensor_settings.sensor_property_id);
			printk("Attr ID: 0x%04x \t",
			       sensor_attr_list[i]
			       .sensor_settings
			       .sensor_setting_property_id);
			printk("Val1: %d \n",
			       sensor_attr_list[i]
			       .sensor_settings.sensor_setting_val.val1);


			/* Look up sensor device */
			const struct device *sensor_dev =
				device_get_binding(sensor_attr_list[i].dev_name);

			err = sensor_attr_set(
				sensor_dev, SENSOR_CHAN_ALL,
				sensor_attr_list[i].sensor_settings.sensor_setting_property_id,
				&sensor_attr_list[i].sensor_settings.sensor_setting_val);
			if (err < 0) {
				printk("ERROR: Could not set sensor attribute.\n");
				return err;
			}

			/* Once updated no need to carry on */
			return err;
		}
	}

	return err;
};

/* =================================== Sensor Setup srv Callback Functions END ================================= */


/* Definitions of models publication context (Start) */

/* Define Health Server publication context  */
BT_MESH_HEALTH_PUB_DEFINE(health_pub, CUR_FAULTS_MAX);

/* Define Sensor Server publication context  */
#if defined(CONFIG_BOARD_BBC_MICROBIT) || defined(CONFIG_BOARD_ESP32)// || defined(CONFIG_BOARD_ARDUINO_NANO_33_BLE)
BT_MESH_MODEL_PUB_DEFINE(sensor_srv_cnt_pub, sensor_cnt_update, MAX_SENS_STATUS_LEN);
// BT_MESH_MODEL_PUB_DEFINE(sensor_srv_tmp_pub, sensor_tmp_update, MAX_SENS_STATUS_LEN);
// BT_MESH_MODEL_PUB_DEFINE(sensor_srv_env_pub, sensor_env_update, MAX_SENS_STATUS_LEN);
#else
BT_MESH_MODEL_PUB_DEFINE(sensor_srv_cnt_pub, sensor_cnt_update, MAX_SENS_STATUS_LEN);
BT_MESH_MODEL_PUB_DEFINE(sensor_srv_tmp_pub, sensor_tmp_update, MAX_SENS_STATUS_LEN);
BT_MESH_MODEL_PUB_DEFINE(sensor_srv_env_pub, sensor_env_update, MAX_SENS_STATUS_LEN);
#endif

/* Define Sensor Setup Server publication context  */
BT_MESH_MODEL_PUB_DEFINE(sensor_setup_srv_pub, NULL, MAX_SENS_STATUS_LEN); /* TODO create callback for sensor setup srv */

#if defined(CONFIG_BOARD_BBC_MICROBIT) || defined(CONFIG_BOARD_ESP32) || defined(CONFIG_BOARD_ARDUINO_NANO_33_BLE)
#endif


/*---------------------------- SENSOR SRV CONTEXT (APP SIDE) -------------------------------------*/
static const struct bt_mesh_sensor_srv_cb sensor_srv_cb = {
/* TODO: Update all functions */
	.sensor_get_status =  sensor_status_msg,

	/* TODO: Resolve callbacks */
	// sensor_desc_get = get_sensor_desc,
	// sensor_get = ,
	// sensor_col_get = ,
	// sensor_series_get = ,
};

static struct bt_mesh_sensor_srv sensor_srv = {
	.cb = &sensor_srv_cb,
};

static struct bt_mesh_sensor_srv sensor_srv_2 = {
	.cb = &sensor_srv_cb,
};
/*---------------------------- SENSOR SRV CONTEXT (APP SIDE) END -------------------------------------*/



/*---------------------------- SENSOR SETUP SRV CONTEXT (APP SIDE) -------------------------------------*/
static const struct bt_mesh_sensor_setup_srv_cb sensor_setup_srv_cb = {
/* TODO: Update all functions */
	.sensor_get_cadence = get_sensor_cadence,
	.sensor_set_cadence = set_sensor_cadence,
	.sensor_get_settings = get_sensor_settings,
	.sensor_get_setting = get_sensor_setting,
	.sensor_set_setting = set_sensor_setting,
};

static struct bt_mesh_sensor_setup_srv sensor_setup_srv = {
	.cb = &sensor_setup_srv_cb,
};
/*---------------------------- SENSOR SETUP SRV CONTEXT (APP SIDE) END -------------------------------------*/


/*---------------------------- SENSOR CLI CONTEXT (APP SIDE)  -------------------------------------*/
static const struct bt_mesh_sensor_cli_cb sensor_cli_cb = {
/* TODO: Update all functions */
	.sensor_data_format = format_marshalled_data,
	// TODO: been set to SRV function to stop errors
	.sensor_cadence_update = set_sensor_cadence,
};

static struct bt_mesh_sensor_cli sensor_cli = {
	.cb = &sensor_cli_cb,

};
/*---------------------------- SENSOR CLI CONTEXT (APP SIDE) END ---------------------------------*/




/*   ------------ ROOT MODELS  -----------  */

#if defined(CONFIG_BOARD_BBC_MICROBIT) || defined(CONFIG_BOARD_ESP32) 
static struct bt_mesh_model root_models[] = {
	/* Sensor Model Invokation */
	BT_MESH_MODEL_SENSOR_SRV(&sensor_srv, &sensor_srv_cnt_pub),
	BT_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_srv, &sensor_setup_srv_pub),
	/* Not Really neeeded for a node/client*/
	// BT_MESH_MODEL_SENSOR_CLI(&sensor_cli),

	/* Standard Besh Models */
	// BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	/* Not Really neeeded for a node/client*/
	// BT_MESH_MODEL_HEALTH_CLI(&health_cli), 
};
#else
static struct bt_mesh_model root_models[] = {
	/* Sensor Model Invokation */
	BT_MESH_MODEL_SENSOR_SRV(&sensor_srv, &sensor_srv_env_pub),
	// BT_MESH_MODEL_SENSOR_SRV(&sensor_srv, &sensor_srv_tmp_pub),	
	BT_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_srv, &sensor_setup_srv_pub),
	/* Not Really neeeded for a node/client*/
	// BT_MESH_MODEL_SENSOR_CLI(&sensor_cli),

	/* Standard Besh Models */
	// BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	/* Not Really neeeded for a node/client*/
	BT_MESH_MODEL_HEALTH_CLI(&health_cli),
};

static struct bt_mesh_model secondary_models[] = {
	/* Sensor Model Invokation */
	BT_MESH_MODEL_SENSOR_SRV(&sensor_srv_2, &sensor_srv_cnt_pub),
	// BT_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_srv, &sensor_setup_srv_pub),
	BT_MESH_MODEL_SENSOR_CLI(&sensor_cli),
};
#endif

/*   ------------ DEFINE MODELS ROOT & VENDOR -----------  */
#if defined(CONFIG_BOARD_BBC_MICROBIT) || defined(CONFIG_BOARD_ESP32) || defined(CONFIG_BOARD_ARDUINO_NANO_33_BLE)
static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

#else
/*   ------------ VENDOR MODEL  -----------  */
static void vnd_button_pressed(struct bt_mesh_model *model,
			       struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{
	printk("src 0x%04x\n", ctx->addr);

	if (ctx->addr == bt_mesh_model_elem(model)->addr) {
		return;
	}

	board_other_dev_pressed(ctx->addr);
}

static const struct bt_mesh_model_op vnd_ops[] = {
	{ OP_VENDOR_BUTTON, 0, vnd_button_pressed },
	BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model vnd_models[] = {
	BT_MESH_MODEL_VND(BT_COMP_ID_LF, MOD_LF, vnd_ops, NULL, NULL),
};

// static struct bt_mesh_elem elements[] = {
//      BT_MESH_ELEM(0, root_models, vnd_models),
// };

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, vnd_models),
	BT_MESH_ELEM(1, secondary_models, BT_MESH_MODEL_NONE),
};
#endif

static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	printk("Local node provisioned, net_idx 0x%04x address 0x%04x\n", net_idx,
	       addr);
	board_addr_display(&addr);
}

static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.reset = prov_reset,
	.complete = prov_complete,
};


/** Function to find model in the elements list
 * @para uint16_t model_id
 * @para struct bt_mesh_model *mod
 * **/
int bt_mesh_get_model(uint16_t model_id, struct bt_mesh_model *mod)
{
	/* Trap not model_id */
	if (!model_id) {
		return -EINVAL;
	}

	for (int i = 0; i < sizeof(elements); i++) {
		mod = bt_mesh_model_find(&elements[i], model_id);
		printk("Element: %d", i);
	}

	if (!mod) {
		return -EINVAL;
	}

	return 0U;
}

/* End of Define trap */
#endif