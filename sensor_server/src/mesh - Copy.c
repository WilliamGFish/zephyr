/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 */
#ifndef _MANULYTICA_MESH_C_
#define _MANULYTICA_MESH_C_


#include <zephyr.h>
#include <string.h>
#include <sys/printk.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/mesh.h>
#include <sys/__assert.h>

#include "common/log.h"

// #include "net.h"
// #include "settings.h"

/* Private includes for raw Network & Transport layer access */
#include <mesh/net.h>
#include <mesh/transport.h>
#include <mesh/rpl.h>

/* Application Files */
#include "mesh.h"
#include "mesh_conf.c"
#include "misc.c"
#include "include/time.c"
#include "include/location.c"

/* Sensor Config Files */
#include "mesh_sensor.c"
#include "WIP/sensor_cli.c"
#include "WIP/sensor_srv.c"
#include "WIP/sensor_setup_srv.c"


/* Mesh & CDB Remote Node UUID */
// static const uint16_t net_idx;
// static const uint16_t app_idx;
static uint16_t node_addr;
static const uint8_t dev_uuid[16] = { 0xdd, 0xdd, 0x32, 0xfa };
static uint8_t node_uuid[16];

/* Set Up Threads for Provisoning */
K_SEM_DEFINE(sem_unprov_beacon, 0, 1);
K_SEM_DEFINE(sem_node_added, 0, 1);


/* VENDOR BUTTON OPCODE DEFINE */
#define OP_VENDOR_BUTTON BT_MESH_MODEL_OP_3(0x00, BT_COMP_ID_LF)

// Set dummy app_idx
// uint16_t key_app_idx = 1;

/* MQTT message buffer */
char mqttbuf[MQTT_MSG_SIZE];

static void heartbeat(uint8_t hops, uint16_t feat)
{
	board_heartbeat(hops, feat);
	printk("heartbeat....................");
	printk("Hops: %u \n", hops);

	printk("Features Changed:\nLPN\tFnd\tPxy\tRly\n");
	int c, k;
	for (c = 3; c >= 0; c--){
		k = feat >> c;
		printk(" %d\t", k & 1);
	}	
}

static void heartbeat_sub(uint16_t src, uint8_t hops, uint16_t feat)
{
	board_heartbeat(hops, feat);
	printk("**************************************************\n");
	printk("Source: 0x%04x - %u hops\n", src, hops);

	printk("Features Changed:\nLPN\tFnd\tPxy\tRly\n");
	int c, k;
	for (c = 3; c >= 0; c--){
		k = feat >> c;
		printk(" %d\t", k & 1);
	}
	printk("\n**************************************************\n\n");
}


/* Configure Mesh Config Server  */
static struct bt_mesh_cfg_srv cfg_srv = {
#if defined(CONFIG_BOARD_BBC_MICROBIT)
	// .relay = BT_MESH_RELAY_ENABLED,
	.relay = BT_MESH_RELAY_DISABLED,
	.beacon = BT_MESH_BEACON_DISABLED,

	/* 3 transmissions with 20ms interval */
	.net_transmit = BT_MESH_TRANSMIT(1, 20),
	// .relay_retransmit = BT_MESH_TRANSMIT(1, 20),

#else
	// .relay = BT_MESH_RELAY_ENABLED,
	.relay = BT_MESH_RELAY_DISABLED,
	.beacon = BT_MESH_BEACON_ENABLED,

	/* 3 transmissions with 20ms interval */
	.net_transmit = BT_MESH_TRANSMIT(1, 20),
	// .relay_retransmit = BT_MESH_TRANSMIT(1, 20),

#if defined(CONFIG_BT_MESH_GATT_PROXY)
	.gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
#else
	.gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif



#endif
	.frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
	.default_ttl = 7,

	// .hb_sub.func = heartbeat,
	.hb_sub.heartbeat = heartbeat_sub,
};

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
	// LOG_DBG("\nSending current faults ");

	// *test_id = 0x00;
	// *company_id = BT_COMP_ID_LF;

	get_health_faults(cur_srv_faults, sizeof(cur_srv_faults), faults, fault_count);

	// LOG_DBG("test_id 0x%04x \t", *test_id);
	// LOG_DBG("company_id 0x%04x \t", *company_id);
	// LOG_DBG("faults 0x%04x \t", *faults);
	// LOG_DBG("fault_count %d \n", *fault_count);

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
	int i = 0U;
	char lat[12] = "0.0", lon[12] = "0.0", alt[12] = "0.0";
	char time_str[sizeof("1970-01-01T00:00:00")];

	/* Shows the data recieved from node */
	// printk("NodeID: %04x ", sensor_data.node_id);
	// printk("PropertyID: %04x ", sensor_data.property_id);
	// printk("Count: %d ", property_count);


	/* Create JSON Ouput */
	#if (CONFIG_MQTT_LIB)
	/* Set MQTT string buffer snprint pointer to 0 */
	int mqttcx = 0U;
	/* if MQTT Defined write to FIFO buffer */
	struct mqtt_data_t msg_data;

	/* Create JSON Output header */

	/* ADAFRUIT FORMAT */
	// mqttcx += snprintf(
	// 	mqttbuf + mqttcx, 50,
	// 	"{\"nodeid\": %x, \"'propid\":%04x,",sensor_data.node_id, sensor_data.property_id);

	// /*  BLUEMIX */
	mqttcx += snprintf(mqttbuf + mqttcx, 150, "{\"d\":{\"sensor\":{[\"nodeID\":%x,\"ID\":%04x", sensor_data.node_id, sensor_data.property_id );
	// mqttcx += snprintf(mqttbuf + mqttcx, 150, "{'d':{'sensor':{['nodeID':%x,'ID':%04x", sensor_data.node_id, sensor_data.property_id );

	// mqttcx +=
	//      snprintf(mqttbuf + mqttcx, 150,
	//               "sensor:-nodeID:%x  propID:%04x",
	//               sensor_data.node_id, sensor_data.property_id);
	#endif


	while (i < property_count) {
		/* code */
		float sensor_float = sensor_value_to_float(&sensor_data.val[i]);

		/* Shows the data recieved from node */
		// snprintf(str_buf, 50, " Value%d: %.3f \t", i,
		// 	 sensor_float);
		// printk("%s", str_buf);


		/* Create JSON Ouput */
		#if (CONFIG_MQTT_LIB)
		/* Build JSON Output */
		if (i > 0U) {

			/* ADAFRUIT */
			mqttcx += snprintf(mqttbuf + mqttcx, 50,
					   "{\"value%d\":%.3f}", i,
					   sensor_float);
		} else {
			/* ADAFRUIT */
			mqttcx += snprintf(mqttbuf + mqttcx, 50,
					   "{\"value\":%.3f}",
					   sensor_float);
		}
		#endif

		i++;
	}

// if (!get_node_location(&lat, &lon, alt)){
// 	printk("ERROR: Couldn't get location..");
// }

	/* Convert time to make sure. */
	time_t now = node_k_time(NULL);
	struct tm now_tm;

	gmtime_r(&now, &now_tm);
	strftime(time_str, sizeof(time_str), "%FT%T", &now_tm);




#if (CONFIG_MQTT_LIB)
	/* Close and Send JSON Output */
	/* ADAFRUIT FORMAT */
	// mqttcx += snprintf(mqttbuf + mqttcx, 5,"]}");

	mqttcx += snprintf(
		mqttbuf + mqttcx, 150,
		"],\"lat\":%s,\"lon\":%s,\"ele\":%s,\"tp\":\"%s\"}}",
		lat, lon, alt, time_str);

	/* if MQTT Defined write to FIFO buffer */
	if ((mqttcx) > MIN_LENGTH){
		// printf("\n %s : %d \n", mqttbuf, mqttcx);
		// memcpy(msg_data.message, mqttbuf, mqttcx);
		strcpy(msg_data.message, mqttbuf);
		// printf("\n %s:%d = %s:%d \n", mqttbuf, mqttcx, msg_data.message, strlen(msg_data.message));
		k_fifo_put(&mqtt_fifo, &msg_data);
		// k_fifo_alloc_put(&mqtt_fifo, &msg_data);
	}
#endif

	return 0U;
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
		err = 1;
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
	sensor_id = SENS_PROP_ID_ACCEL_XYZ; // Accel X Value (m/s)
	sensor_status_msg(sensor_id, msg);

	/* Get COUNT Values */
	// sensor_id = SENS_PROP_ID_EXT_CNT_TOTAL; // Total Count Value (int)
	// sensor_status_msg(sensor_id, msg);


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

// /* Sensor srv Update Callback */
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

#if !defined(CONFIG_BOARD_BBC_MICROBIT)
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
#if defined(CONFIG_BOARD_BBC_MICROBIT)
BT_MESH_MODEL_PUB_DEFINE(sensor_srv_cnt_pub, sensor_cnt_update, MAX_SENS_STATUS_LEN);
#else
// BT_MESH_MODEL_PUB_DEFINE(sensor_srv_tmp_pub, sensor_tmp_update, 2 + MAX_SENS_STATUS_LEN + 4);
BT_MESH_MODEL_PUB_DEFINE(sensor_srv_cnt_pub, sensor_cnt_update, MAX_SENS_STATUS_LEN);
BT_MESH_MODEL_PUB_DEFINE(sensor_srv_env_pub, sensor_env_update, MAX_SENS_STATUS_LEN);
#endif

/* Define Sensor Setup Server publication context  */
BT_MESH_MODEL_PUB_DEFINE(sensor_setup_srv_pub, NULL, MAX_SENS_STATUS_LEN); /* TODO create callback for sensor setup srv */



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
static struct bt_mesh_model root_models[] = {
	/* Sensor Model Invokation */
	BT_MESH_MODEL_SENSOR_SRV(&sensor_srv, &sensor_srv_cnt_pub),
	BT_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_srv, &sensor_setup_srv_pub),
	BT_MESH_MODEL_SENSOR_CLI(&sensor_cli),

	/* Standard Besh Models */
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL_HEALTH_CLI(&health_cli),


};

static struct bt_mesh_model secondary_models[] = {
	// BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV, sensor_srv_op, &sensor_srv_cnt_pub, NULL),
};


/*   ------------ DEFINE MODELS ROOT & VENDOR -----------  */


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

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, vnd_models),
	BT_MESH_ELEM(0, secondary_models, BT_MESH_MODEL_NONE),
};


static const struct bt_mesh_comp comp = {
	.cid = BT_COMP_ID_LF,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};



/** @brief Unprovisioned beacon has been received.
 *
 * This callback notifies the application that an unprovisioned
 * beacon has been received.
 *
 * @param rem_uuid UUID
 * @param oob_info OOB Information
 * @param uri_hash Pointer to URI Hash value. NULL if no hash was
 *                 present in the beacon.
 */
static void prov_unprovisioned_beacon(uint8_t rem_uuid[16],
				 bt_mesh_prov_oob_info_t oob_info,
				 uint32_t *uri_hash) 
{
	// /* Add code to automatically provision node */
	// char uuid_hex_str[32 + 1];
	// uint8_t attention_duration = 5U ;
	// uint16_t net_idx = 0U;
	// uint16_t addr = 0U;
	// int err;

	// bin2hex(rem_uuid, 16, uuid_hex_str, sizeof(uuid_hex_str));

	// printk("Provisioning %s\n", uuid_hex_str);
	// err = bt_mesh_provision_adv(rem_uuid, net_idx, addr, attention_duration);
	// if (err) {
	// 	printk("ERROR: Provisioning remote node failed (err %d)\n", err);
	// }

	memcpy(node_uuid, rem_uuid, 16);
	k_sem_give(&sem_unprov_beacon);	
}





// /*********************************************************************************************************************/


// int mesh_rpl_addr_clear(uint16_t src)
// {
// 	int i;

// 	if (!src) {
// 		LOG_DBG("ERROR: Invalid source Addr: 0x%04x", src);
// 		// printk("ERROR: Invalid source Addr: 0x%04x\n", src);
// 		return -ENOENT;
// 	}

// 	for (i = 0; i < ARRAY_SIZE(bt_mesh.rpl); i++) {
// 		if (bt_mesh.rpl[i].src == src) {

// 			LOG_DBG("Cleaning Replay Buffers for Addr %d", src);
// 			// printk("\n\tCLEARING RPL for source Addr: 0x%04x\n", src);

// 			(void)memset(&bt_mesh.rpl[i], 0, sizeof(*&bt_mesh.rpl[i]));
// 			return 0;			
// 		}
// 	}

// 	LOG_DBG("Error: No Replay Buffers Found for Addr %d", src);
// 	// printk("ERROR: NOT FOUND!! source Addr: 0x%04x\n", src);
// 	return -ENOENT;
// }

// /*********************************************************************************************************************/


int get_node_conf_data(uint16_t net_idx, uint16_t addr)
{
	int comp_buffer_size = 256;
	NET_BUF_SIMPLE_DEFINE(remote_comp, comp_buffer_size);
	uint8_t status, page = 0xff;
	int err = 0U;
	
	err = bt_mesh_cfg_comp_data_get(net_idx, addr, page,
					&status, &remote_comp);

	if (err == -EAGAIN) {
		if (IS_ENABLED(CONFIG_BT_MESH_CDB_ADDR_REISSUE)) {
			LOG_DBG("ERROR: Cleaning Replay Buffers for Addr 0x%04x (err %d)", addr, err);
			printk("ERROR: Cleaning Replay Buffers for Addr 0x%04x (err %d) \n", addr, err);

			/* Clear Replay buffer to allow reconnection */
			// bt_mesh_rpl_clear();
			// mesh_rpl_addr_clear(addr);
			bt_mesh_rpl_addr_clear(addr);
		}
		
		err = bt_mesh_cfg_comp_data_get(net_idx, addr, page,
					&status, &remote_comp);		
		return err;			
	}

	if (err) {
		LOG_DBG("ERROR: Getting composition failed for Addr 0x%04x  (err %d)", addr, err);
		printk("ERROR: Getting composition failed for Addr 0x%04x (err %d)\n", addr, err);		
		return err;
	}

	if (remote_comp.len < 7 || remote_comp.len > comp_buffer_size ){
		err = -ENOEXEC;
		LOG_DBG("ERROR: Composition data length invalid: %d - (err %d)", remote_comp.len, err);
		printk("ERROR: Composition data length invalid: %d - (err %d)\n", remote_comp.len, err);
		
		if (IS_ENABLED(CONFIG_BT_MESH_CDB_ADDR_REISSUE)) {
			/* Clear Replay buffer to allow reconnection */
			// bt_mesh_rpl_clear();
			// mesh_rpl_addr_clear(addr);
			bt_mesh_rpl_addr_clear(addr);
		}
		
		return err;			
	}

	if (status != 0x00) {
		LOG_DBG("ERROR: Got non-success status 0x%02x\n", status);
		return -EINVAL;;
	}

	printk("\n\n\nGetting composition info, net_idx 0x%04x address 0x%04x\n", net_idx,
	       addr);

	printk("Got Composition Data for 0x%04x: \n", addr);
	printk("\tCID      0x%04x\n",
		    net_buf_simple_pull_le16(&remote_comp));
	printk("\tPID      0x%04x\n",
		    net_buf_simple_pull_le16(&remote_comp));
	printk("\tVID      0x%04x\n",
		    net_buf_simple_pull_le16(&remote_comp));
	printk("\tCRPL     0x%04x\n",
		    net_buf_simple_pull_le16(&remote_comp));

	uint16_t features = net_buf_simple_pull_le16(&remote_comp);

	printk("\tFeatures:\n\tLPN\tFnd\tPxy\tRly\n\t");
	int c, k;
	for (c = 3; c >= 0; c--){
		k = features >> c;
		printk(" %d\t", k & 1);
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
			printk("\t\t...truncated data!\n");
			break;
		}

		if (sig) {
			printk("\t\tSIG Models: %d", sig);
		} else {
			printk("\t\tNo SIG Models\n");
		}

		for (i = 0; i < sig; i++) {
			uint16_t mod_id = net_buf_simple_pull_le16(&remote_comp);

			if ( i % 3 == 0 ){
				printk("\n");
			}
			printk("\t\t\t0x%04x", mod_id);
		}
		printk("\n");

		if (vnd) {
			printk("\t\tVendor Models: %d\n", vnd);
		} else {
			printk("\t\tNo Vendor Models\n");
		}

		for (i = 0; i < vnd; i++) {
			uint16_t cid = net_buf_simple_pull_le16(&remote_comp);
			uint16_t mod_id = net_buf_simple_pull_le16(&remote_comp);

			printk("\t\t\tCompany 0x%04x: 0x%04x\n", cid,
				    mod_id);
		}
		printk("\n");
	}
	printk("\n\n");
	return err;
}


static void prov_node_added(uint16_t net_idx, uint8_t uuid[16], uint16_t addr, uint8_t num_elem)
{

	node_addr = addr;
	k_sem_give(&sem_node_added);

	LOG_DBG("Remote node provisioned, net_idx 0x%04x address 0x%04x elements %d",
	       net_idx, addr, num_elem);
}

static void prov_complete(uint16_t net_idx, uint16_t addr)
{
	LOG_DBG("Local node provisioned, net_idx 0x%04x address 0x%04x", net_idx,
	       addr);
}


static void prov_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}


static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.unprovisioned_beacon = prov_unprovisioned_beacon,
	.node_added = prov_node_added,
	.reset = prov_reset,
	.complete = prov_complete,	
};


/** Function to find model in the elements list
 * @para uint16_t model_id
 * @para struct bt_mesh_model *mod
 * **/
int bt_mesh_get_model(uint16_t model_id, struct bt_mesh_model *mod)
{
	// struct bt_mesh_model *mod;

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
	// return mod;
}


/* Mesh CDB Functions */
static void cdb_print_nodes(void)
{
	char key_hex_str[32 + 1], uuid_hex_str[32 + 1];
	struct bt_mesh_cdb_node *node;
	int i, total = 0;
	bool configured;

	printk("\n\nAddress  \tElements  \tFlags  \tUUID  \t\t\t\t\t\tDevKey \n");

	for (i = 0; i < ARRAY_SIZE(bt_mesh_cdb.nodes); ++i) {
		node = &bt_mesh_cdb.nodes[i];
		if (node->addr == BT_MESH_ADDR_UNASSIGNED) {
			continue;
		}

		configured = atomic_test_bit(node->flags,
					     BT_MESH_CDB_NODE_CONFIGURED);

		total++;
		bin2hex(node->uuid, 16, uuid_hex_str, sizeof(uuid_hex_str));
		bin2hex(node->dev_key, 16, key_hex_str, sizeof(key_hex_str));
		printk("0x%04x   \t%-8d  \t%-5s  \t%s  \t\t%s \n", node->addr,
			    node->num_elem, configured ? "C" : "-",
			    uuid_hex_str, key_hex_str);
	}

	printk("> Total nodes: %d\n\n", total);
}



static uint8_t check_unconfigured(struct bt_mesh_cdb_node *node, void *data)
{
	if (!atomic_test_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED)) {

		int err;

		if (node->addr == addr) {
			// /* This is a stub for defining config of node based on models running */
			// err = get_node_conf_data(node->net_idx, node->addr);
			// if (err != 0){
			// 	printk("ERROR: Getting Config Data failed 0x%04x (err %d)\n", node->addr, err);

			// 	return BT_MESH_CDB_ITER_CONTINUE;
			// }
				LOG_DBG("Unconfigured Provisioner found: 0x%04x\n", node->addr);
				configure_self(node);
		} else {
			LOG_DBG("Unconfigured Node found: 0x%04x\n", node->addr);
			configure_node(node);
		}
	}

	return BT_MESH_CDB_ITER_CONTINUE;
}


static void setup_cdb(void)
{
	struct bt_mesh_cdb_app_key *key;

	key = bt_mesh_cdb_app_key_alloc(net_idx, app_idx);
	if (key == NULL) {
		LOG_DBG("Failed to allocate app-key 0x%04x\n", app_idx);
		return;
	}

	bt_rand(key->keys[0].app_key, 16);
	memcpy(key->keys[0].app_key, app_key, 16);
	// bt_rand(key->keys[0].app_key, 16);

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		bt_mesh_cdb_app_key_store(key);
	}
}

void provisioner_init(void)
{
	int err;
	char uuid_hex_str[32 + 1];	
	uint8_t attention_duration = 5U ;
	uint8_t prov_wait_duration = 10U ;

	printk("Provisioner initializing...\n");

	bt_mesh_cdb_node_foreach(check_unconfigured, NULL);

	while (1) {
		k_sem_reset(&sem_unprov_beacon);
		k_sem_reset(&sem_node_added);
		// // bt_mesh_cdb_node_foreach(check_unconfigured, NULL);

		cdb_print_nodes();

		printk("Waiting for unprovisioned beacon...\n");
		err = k_sem_take(&sem_unprov_beacon, K_SECONDS(prov_wait_duration));
		if (err == -EAGAIN) {
			/* Check for unprovisioned CDB nodes */
			bt_mesh_cdb_node_foreach(check_unconfigured, NULL);
			continue;
		}

		bin2hex(node_uuid, 16, uuid_hex_str, sizeof(uuid_hex_str));

		printk("Provisioning %s\n", uuid_hex_str);

		err = bt_mesh_provision_adv(node_uuid, net_idx, 0U, attention_duration);
		if (err == -EBUSY){
			printk("ERROR: Provisioning failed radio busy: (err %d)\n", err);
			/* Wait for connection to reset */
			k_sleep(K_SECONDS(prov_wait_duration / 2));	
			continue;					
		}else if (err)
		{
			printk("ERROR: Provisioning failed (err %d)\n", err);
			continue;
		}else{
			err = k_sem_take(&sem_node_added, K_SECONDS(prov_wait_duration));
			if (err == -EAGAIN) {
				printk("ERROR: Timeout waiting for node to be added\n");
				continue;
			}
			else if (err == 0){
				LOG_DBG("Configuring unconfigured beacon............. 0x%04x", net_idx);
				bt_mesh_cdb_node_foreach(check_unconfigured, NULL);
			}
		}

		/* Clean up for next loop */
		// bt_mesh_cdb_node_foreach(check_unconfigured, NULL);
	}
}

/* End of Define trap */
#endif