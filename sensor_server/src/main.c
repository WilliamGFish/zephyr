/* main.c - Application main entry point */

/*
 * Copyright (c) 2019-2021 Manulytica Ltd
 *
 */
#include <logging/log.h>
// #if !defined(LOG_LEVEL)
// #define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
// #endif

#define LOG_MODULE_NAME manulytica_app
#include "common/log.h"

#include "include/location.c"
#include "board.h"

#include <zephyr.h>
#include <settings/settings.h>

#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <sys/byteorder.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>

// Networking Files
#include <net/net_if.h>
#include <net/wifi_mgmt.h>
#include <net/net_event.h>

/* Application Files */


/* Includes for GSM PPP Modem */
#include <drivers/uart.h>
#include <net/net_mgmt.h>
#include <net/net_event.h>
#include <net/net_conn_mgr.h>
#include <drivers/gsm_ppp.h>



/*FINALLY MQTT Client*/
#if defined(CONFIG_MQTT_LIB)
	// MQTT Files
	// #include "mqtt/mqtt_pub.h"
	// #include "mqtt_iot/mqtt_pub.c"

	#ifdef CONFIG_CLOUD_AZURE_PLATFORM
		#include "mqtt_iot/mqtt_pub.h"
		#include "mqtt_iot/azure/main.c"
	#elif CONFIG_CLOUD_GOOGLE_PLATFORM
		#include "mqtt_iot/mqtt_pub.h"
		#include "mqtt_iot/google/main.c"
	#else
		#include "mqtt/mqtt_pub.h"
		#include "mqtt/mqtt_pub.c"
	#endif
#endif


/* Bluetooth inc Mesh */
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>

/* Application Files */
#include "include/time.c"
#include "watchdog.c"
#include "mesh.c"
#include "health_faults.c"
#include "battery_srv.c"

/* Sensor Set-up Server Sensor Atrr */
#if defined(CONFIG_FXOS8700)	
	#include "sensor_settings.c"
#endif

#include <device.h>


#include <drivers/flash.h>
#include <devicetree.h>

// #include "sensor_cli.c"

/* added for modem context info */
// #include "../drivers/modem/modem_receiver.h"
#include "../drivers/modem/modem_context.h"

/* Configure work threads */
// static struct k_work sensor_fxos_work;
// static struct k_work sensor_bme280_work;
// static struct k_work sensor_nrftemp_work;
// static struct k_work mesh_work;

static const struct device *gsm_dev;
static struct net_mgmt_event_callback mgmt_cb;
static bool connected = false;

static void net_if_event_handler(struct net_mgmt_event_callback * cb,
				uint32_t mgmt_event, struct net_if * iface)
{
	if ((mgmt_event & (NET_EVENT_L4_CONNECTED |
				NET_EVENT_L4_DISCONNECTED | NET_EVENT_IPV4_ADDR_ADD)) != mgmt_event) {
		return;
	}

	if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
		printk("Network address assigned");
		connected = true;
		return;
	}
	if (mgmt_event == NET_EVENT_L4_CONNECTED) {
		printk("Network connected");
		connected = true;
		return;
	}

	if (mgmt_event == NET_EVENT_L4_DISCONNECTED) {
		LOG_INF("Network disconnected");
		connected = false;
		return;
	}
}





// static void bt_ready(int err)
static int bt_ready(void)
{
	int err = 0;
	printk("Bluetooth initialized\n");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("ERROR: Initializing mesh failed (err %d)\n", err);
		return -1;
	}

	printk("Mesh initialized\n");

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		printk("Loading stored settings\n");
		if (!settings_load()){
			printk("Using preconfigured settings\n");
			// mesh_configure(); // CHNAGED TO FORCE TO USE THE CDB COMMISSIONSING 
		}
	}

	err = bt_mesh_cdb_create(net_key);
	if (err == -EALREADY) {
		printk("Using stored CDB\n");
		mesh_configure(); // CHANGED TO FORCE TO USE THE MANUAL/CDB COMMISSIONSING GATWAY
	} else if (err) {
		printk("Failed to create CDB (err %d)\n", err);
		return -1;
	} else {
		printk("Created CDB\n");
		setup_cdb();
	}

	err = bt_mesh_provision(net_key, net_idx, flags, iv_index, addr,
				dev_key);
	if (err == -EALREADY) {
		printk("Using stored settings\n");
	} else if (err) {
		printk("Provisioning failed (err %d)\n", err);
		return -1;
	} else {
		// mesh_configure(); // CHANGED TO FORCE TO USE THE MANUAL/CDB COMMISSIONSING GATWAY
		
		printk("Provisioning completed!\n");
	}

#if NODE_ADDR == GATEWAY_ADDR && (IS_ENABLED(CONFIG_BT_MESH_PROVISIONER))
	/* Heartbeat subcscription is a temporary state (due to there
	 * not being an "indefinite" value for the period, so it never
	 * gets stored persistently. Therefore, we always have to configure
	 * it explicitly.
	 */
	{
		// struct bt_mesh_cfg_hb_sub sub = {
		// 	.src = GROUP_ADDR,
		// 	.dst = GATEWAY_ADDR,
		// 	.period = 0x10,
		// };

		// bt_mesh_cfg_hb_sub_set(net_idx, addr, &sub, NULL);
		// printk("Subscribing to heartbeat messages\n");
	}
#endif
	return 0;
}


static uint16_t target = GROUP_ADDR;

uint16_t board_set_target(void)
{
	switch (target) {
	case GROUP_ADDR:
		target = 1U;
		break;
	case 9:
		target = GROUP_ADDR;
		break;
	default:
		target++;
		break;
	}

	return target;
}


/** Strawman function to send bt_mesh_sensor_setting_set message **/
void set_sensor_data(){  // Set Sensor Data function 

	/* send unsolicited request to get sensor readings */
	struct bt_mesh_msg_ctx ctx = {
		.net_idx = net_idx,
		.app_idx = app_idx,
		.addr = target,
		.send_rel = 0U,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	struct bt_mesh_sensor_setting_data setting_data;

	setting_data.sensor_property_id = SENS_PROP_ID_ACCEL_XYZ;
	setting_data.sensor_setting_property_id = 0x002;  /* Trigger value */

	setting_data.sensor_setting_val.val1 = 11;
	setting_data.sensor_setting_val.val2 = 500000;

	bt_mesh_sensor_setting_set(&elements[0], &ctx, &setting_data);
}



void send_sensor_data(uint16_t sensor_id)
{
	/* send unsolicited request to get sensor readings */
	struct bt_mesh_msg_ctx ctx = {
		.net_idx = net_idx,
		.app_idx = app_idx,
		.addr = target,
		.send_rel = 0U,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	NET_BUF_SIMPLE_DEFINE(msg, 2 + MAX_SENS_STATUS_LEN + 4);
	/* Get sensor data */
	sensor_status_msg(sensor_id, &msg);

	// printk("ctx size: %d ", sizeof(&ctx));
	// printk(" -- msg size: %d \n", sizeof(&msg));

	/* Send message via SENSOR_SVR */
	/* TODO: Mesh model lookup */
	int err = (bt_mesh_model_send(&root_models[0], &ctx, &msg, NULL, NULL));
	if (err != 0) {
		LOG_DBG("ERROR: Unable to send status for Sensor ID: 0x%04x\n", sensor_id);
		LOG_DBG("ERROR: bt_mesh_model_send Error ID: %d\n", err);
	} else {
		LOG_DBG("MESSAGE: Sensor message sent OpCode: 0x%04x", sensor_id);
	}

	/* Reset message buffer */
	net_buf_simple_reset(&msg);
}

/** Button 1 unsolicitaed messages sneding **/
void board_button_1_pressed(void)
{
	/* send unsolicited request to get sensor readings */
	uint16_t sensor_id = 0;

	sensor_id = SENS_PROP_ID_TEMP; // MPU Die Temp Data

	struct bt_mesh_msg_ctx ctx = {
		.net_idx = net_idx,
		.app_idx = app_idx,
		.addr = target,
		.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	NET_BUF_SIMPLE_DEFINE(msg, 2 + MAX_SENS_STATUS_LEN + 4);
	// NET_BUF_SIMPLE_DEFINE(msg, 1 + 4 + 8);
	sensor_status_msg(sensor_id, &msg);

	/* Send message via SENSOR_SVR */
	/* TODO: Mesh model lookup */
	int rc = (bt_mesh_model_send(&root_models[0], &ctx, &msg, NULL, NULL));

	if (rc != 0) {
		LOG_DBG("ERROR: Unable to send status for Sensor ID: 0x%04x\n", sensor_id);
		LOG_DBG("ERROR: bt_mesh_model_send Error ID: %d\n", rc);
	} else {
		/* success */
		LOG_DBG("MESSAGE: Sensor message sent OpCode: 0x%04x \n", sensor_id);
	}

	/* Reset message buffer */
	net_buf_simple_reset(&msg);
}


/** --------- Start of Health SRV Faults --------- **/
void cmd_health_fault_add(){
	uint8_t err = 0;

	uint8_t fault_code = HEALTH_FAULT_ID_POWER_SUPPLY_INTERRUPTED_ERROR;
	// HEALTH_FAULT_ID_SUPPLY_VOLTAGE_LOW_WARNING   0x03
	// HEALTH_FAULT_ID_SUPPLY_VOLTAGE_LOW_ERROR   0x04
	// HEALTH_FAULT_ID_SUPPLY_VOLTAGE_HIGH_WARNING   0x05
	// HEALTH_FAULT_ID_SUPPLY_VOLTAGE_HIGH_ERROR   0x06
	// HEALTH_FAULT_ID_POWER_SUPPLY_INTERRUPTED_WARNING   0x07
	// HEALTH_FAULT_ID_POWER_SUPPLY_INTERRUPTED_ERROR   0x08

	err = health_fault_add(root_models, fault_code);
	if (err < 0) {
		LOG_DBG("ERROR: Fault add failed !");
	}
}


void cmd_health_del_fault(){

	uint8_t err = 0;

	uint8_t fault_code = HEALTH_FAULT_ID_POWER_SUPPLY_INTERRUPTED_ERROR;
	// HEALTH_FAULT_ID_SUPPLY_VOLTAGE_LOW_WARNING   0x03
	// HEALTH_FAULT_ID_SUPPLY_VOLTAGE_LOW_ERROR   0x04
	// HEALTH_FAULT_ID_SUPPLY_VOLTAGE_HIGH_WARNING   0x05
	// HEALTH_FAULT_ID_SUPPLY_VOLTAGE_HIGH_ERROR   0x06
	// HEALTH_FAULT_ID_POWER_SUPPLY_INTERRUPTED_WARNING   0x07
	// HEALTH_FAULT_ID_POWER_SUPPLY_INTERRUPTED_ERROR   0x08

	err = health_del_fault(root_models, fault_code);
	if (err < 0) {
		LOG_DBG("ERROR: Fault deletion failed !");
	}
}



/** ---- External ADC Sensor Functions ----- **/
#if !defined(CONFIG_BOARD_BBC_MICROBIT)	
enum adc_action adc_sampling_callback(
				const struct device *dev,
				const struct adc_sequence *sequence,
				uint16_t sampling_index)
{
	struct sensor_value sample_value;
	float cadence_high, cadence_low, value_sample = 0.0, value_delta = 0.0;

	struct bt_mesh_model *mod;
	mod = bt_mesh_model_find(&elements[0], BT_MESH_MODEL_ID_SENSOR_SRV);

	/* Set period divisor */
	// mod->pub->period_div = adc_cadence_data.fast_cadence_period_divisor;
	mod->pub->period_div = 0x04;

	/* Set the Last Sample Value if none exist */
	if (last_sample == 0)
	{
		// last_sample = adc_sample_buffer[0];
		ext_value_convert(&sample_value, adc_sample_buffer[0]);
		last_sample = sensor_value_to_float(&sample_value);
	}
	
	cadence_high=
		sensor_value_to_double(&adc_cadence_data.fast_cadence_high);
	
	cadence_low =
		sensor_value_to_double(&adc_cadence_data.fast_cadence_low);

	int i;	
	for (i = 1; i < ADC_BUFFER_SIZE; i++) {
		ext_value_convert(&sample_value, adc_sample_buffer[i]);
		value_sample = sensor_value_to_float(&sample_value);
		value_delta = last_sample - value_sample;

		
		/** Check fast cadence boundries **/
		/* If Higher than High and Lower than Low */
		if ( cadence_high < cadence_low && value_sample >= cadence_high && value_sample <= cadence_low) 
		{
			/* Increase reporting frequnecy */
			mod->pub->fast_period = 1U;
			// printf("------ AND:  %d\n\n", sensor_srv_period);
		} else if (value_sample >= cadence_high || value_sample <= cadence_low) {
			mod->pub->fast_period = 1U;
			// printf("------ OR:  %d\n\n", sensor_srv_period);
        } else {
			mod->pub->fast_period = 0U;
			// printf("OKay: reset  %d\n\n", sensor_srv_period);
		}

		/* TODO: */
		/* force model to refresh/send publish messages */
		if (mod->pub->period != sensor_srv_period)
		{
			// mod->pub->period = sensor_srv_period;
			// printf("UPDATED: Period %d\n", sensor_srv_period);
			// bt_mesh_model_publish(mod);
		}


		/** Check if Excessive Trigger Delta occured **/
		/* Status Trigger Type 0b1 percentage change */
		if (adc_cadence_data.status_trigger_type == 0b1)
		{
			if (value_delta < 0 && value_delta <= sensor_value_to_float(&adc_cadence_data.status_trigger_delta_down)/100)
			{
				printf("0b1 Value has gone DOWN to fast:%f : %d\n\n", value_delta, i);
				bt_mesh_model_publish(mod);

				printf("SWITCH BACK TO COUNTER !!! \n");
				mod->pub->fast_period = 0U;
				adc_cadence_data.property_id =
					SENS_PROP_ID_COUNTER;
				adc_reset();
				return ADC_ACTION_FINISH; /* Stop Scanning */

			}else if (value_delta > 0 && value_delta >= sensor_value_to_float(&adc_cadence_data.status_trigger_delta_up)/100)
			{
					printf("0b1 Value has gone UP to fast:%f : %d\n\n",  value_delta, i);
					bt_mesh_model_publish(mod);
			}
		/* Status Trigger Type 0b0 defined by UUID Format Type (sec 4.1.1.1) */
		} else if (adc_cadence_data.status_trigger_type == 0b0)
		{
			/* TODO:  */
			if (value_delta < 0 && value_delta <= sensor_value_to_float(&adc_cadence_data.status_trigger_delta_down))
			{
				printf("0b0 Value has gone DOWN to fast:%f : %d\n\n", value_delta, i);
				bt_mesh_model_publish(mod);

			}else if (value_delta > 0 && value_delta >= sensor_value_to_float(&adc_cadence_data.status_trigger_delta_up))
			{
				printf("0b0 Value has gone UP to fast:%f : %d\n\n",  value_delta, i);
				bt_mesh_model_publish(mod);
			}
		}	

		last_sample = value_sample;
	}

	return ADC_ACTION_CONTINUE;
}
#endif

static uint8_t check_node_config(struct bt_mesh_cdb_node *node, void *data)
{
	// printk("check_node_config.........\n\n");
	// get_node_conf_data(node->net_idx, node->addr);

	return BT_MESH_CDB_ITER_CONTINUE;
}


/* External Sensor Timer Stuff (ADC) */
// void r_node_comp_work_handler(struct k_work *work)
// {
// 	// get_node_conf_data(net_idx, node_addr);
// 	bt_mesh_cdb_node_foreach(check_node_config, NULL);
// }

/* Some time that kicks off a CDB read and print */
// K_WORK_DEFINE(r_node_comp_work, r_node_comp_work_handler);

// void r_node_comp_timer_handler(struct k_timer *r_node_comp_dummy)
// {
//     k_work_submit(&r_node_comp_work);
// }

// K_TIMER_DEFINE(r_node_comp_timer, r_node_comp_timer_handler, NULL);



/* DUMMY COUNT HANDLER FOR External Sensor */
void count_work_handler(struct k_work *count_work)
{
		// NET_BUF_SIMPLE_DEFINE(msg, 2 + MAX_SENS_STATUS_LEN + 4);
		// /* Get counter data to increase by 1 */
		// sensor_status_msg(SENS_PROP_ID_COUNTER, &msg);	

		// Increment the item counter
		ext_item_count++;

		/* Reset counter at 65000 dec */
		if (ext_item_count > 0xfde8) { // 0xfde8
			ext_item_count = 1; 
		}		
}

K_WORK_DEFINE(count_work, count_work_handler);


/* Counter Thread Definitions */
void count_timer_handler(struct k_timer *my_count)
{
    k_work_submit(&count_work);
}

K_TIMER_DEFINE(count_timer, count_timer_handler, NULL);






/*/   ------------ MAIN -----------  /*/
void main(void)
{
	int err;

	printk("Initializing...\n");


/* Do networking stuff first */
#if defined(CONFIG_WIFI)
	struct net_if *iface = net_if_get_default();
	/* Make sure modem has booted before continuing, add delay to allow modem to wakeup */
	printk("Checking modem has booted...\n");
	k_sleep(K_SECONDS(5));
	// net_if_up(iface);
	int tries = 0U;
	bool is_up = net_if_is_up(iface);
	while (!is_up)
	{
		/* Just a delay */
		tries++;
		is_up = net_if_is_up(iface);
		printk("Waiting..%d \n", is_up);
		k_sleep(K_SECONDS(1));

		/* if the modem hasn't responded force restart and wait for restart */
		if (tries > 3){
			printk("RESTARTING MODEM.....%d\n",tries);
			net_if_up(iface);
			tries = 0U;
			k_sleep(K_SECONDS(10));
		}
	}




	static struct wifi_connect_req_params params;
	/* SSID */
	// params.ssid = "BillyBobJoe2G";
	params.ssid = CONFIG_WIFI_SSID;

	/* SSID length */
	params.ssid_length = strlen(params.ssid);
	
	/* Channel (optional) */
	params.channel = WIFI_CHANNEL_ANY;

	/* PSK (optional) */
	// params.psk = "Noyoucantbu5ter";
	params.psk = CONFIG_WIFI_PASSWORD;

	params.psk_length = strlen(params.psk);
	params.security = WIFI_SECURITY_TYPE_PSK;

	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
		     &params, sizeof(struct wifi_connect_req_params))) {
		printk("ERROR: Connection request failed\n");
	} else {
		printk("***************************************** WiFi Connection Requested*****************************************\n");
	}

	net_mgmt_init_event_callback(&mgmt_cb, net_if_event_handler,
						NET_EVENT_IPV4_ADDR_ADD);
	net_mgmt_add_event_callback(&mgmt_cb);

	while (!connected) {
		k_sleep(K_SECONDS(2));
		printk(".");
	}
	printk("\n");	
	/*Allow modem interface to stablise */
	k_sleep(K_SECONDS(5));
#endif //if CONFIG_WIFI

#if defined(CONFIG_MODEM_GSM_PPP)
	printk("TRYING TO CONNECT\n");
	k_sleep(K_SECONDS(10));


	const struct device *uart_dev =
				device_get_binding(CONFIG_MODEM_GSM_UART_NAME);

	gsm_dev = device_get_binding(GSM_MODEM_DEVICE_NAME);

	LOG_INF("Board '%s' APN '%s' UART '%s' device %p (%s)",
		CONFIG_BOARD, CONFIG_MODEM_GSM_APN,
		CONFIG_MODEM_GSM_UART_NAME, uart_dev, GSM_MODEM_DEVICE_NAME);

	net_mgmt_init_event_callback(&mgmt_cb, net_if_event_handler,
						NET_EVENT_L4_CONNECTED |
							NET_EVENT_L4_DISCONNECTED);
	net_mgmt_add_event_callback(&mgmt_cb);

	while (!connected) {
		k_sleep(K_SECONDS(1));
		printk(".");
	}
	printk("\n");
#endif


	// k_sleep(K_SECONDS(5));
	printk("*****************************************SNTP Connection Requested*****************************************\n");
	/*Use SNTP to set time_base to convert UTF time*/
	if (IS_ENABLED(CONFIG_WIFI) || IS_ENABLED(CONFIG_MODEM)){
		set_time_base();
	}

	k_sleep(K_SECONDS(1));
printk("***************************************** Start MQTT Client *****************************************\n");
	
	/* Last thing start the MQTT Publisher */
#if defined(CONFIG_MQTT_LIB)
	/* Initialize the MQTT Client Subsystem */
	printk("Waiting for MQTT client...\n");

	if (IS_ENABLED(CONFIG_CLOUD_AZURE_PLATFORM)) {
		printk("Starting Azure MQTT client.\n");		
		
	} else if (IS_ENABLED(CONFIG_CLOUD_GOOGLE_PLATFORM)) {
		printk("Starting Google MQTT client.\n");
	} else {
		printk("Starting Default MQTT client.\n");
		// mqtt_init();
	}

	mqtt_init();

	k_sleep(K_SECONDS(2));
#endif // CONFIG_MQTT_LIB


printk("*****************************************SNTP Connection Requested*****************************************\n");
	/*Use SNTP to set time_base to convert UTF time*/
	if (IS_ENABLED(CONFIG_WIFI) || IS_ENABLED(CONFIG_MODEM)){
		set_time_base();
	}



	/* Initialize the Board Subsystem */
	board_init(&addr);
	k_sleep(K_SECONDS(2));

	mpu_temp_init();
	fxos_init();
	apds_init();
	bme_init();
	max_init();
#if !defined(CONFIG_BOARD_BBC_MICROBIT)	
	init_adc();
#endif

	/* Add the sensor settings to senser set-up server attr list */
#if defined(CONFIG_FXOS8700)	
	add_sensor_attr();
#endif


	printk("Unicast address: 0x%04x\n", addr);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");
	bt_ready();


#if defined(CONFIG_BOARD_NRF52840_PCA10056) || defined(CONFIG_BOARD_NRF52_PCA10040)
		nrf_buttons_init();
#endif

	// k_sleep(K_SECONDS(2));

	printk("\nInitialisation Completed \n ");
	printk("\n\n");


	/* Update publishing frequncy */
// #if defined(CONFIG_BOARD_BBC_MICROBIT)
// 	sensor_srv_period = BT_MESH_PUB_PERIOD_100MS(20);
// #else
// 	sensor_srv_period = BT_MESH_PUB_PERIOD_100MS(10);
// #endif

	/* start periodic timer that expires once every second */
	// k_timer_start(&r_node_comp_timer, K_SECONDS(150), K_SECONDS(150));






	// /* Turn on GATT Proxy */
	// uint8_t proxy;
	// printk("\nSetting PROXY Node Identity\n");
	// err = bt_mesh_cfg_gatt_proxy_set(net_idx, addr, BT_MESH_GATT_PROXY_ENABLED, &proxy);	
	// if (err) {
	// 	printk("\n\nFailed set PROXY Node Identity (err %d)\n\n", err);
	// }

	// if (proxy == BT_MESH_GATT_PROXY_ENABLED ) {
	// 	printk("\n\nPROXY Node status (BT_MESH_GATT_PROXY_ENABLED %d)\n\n", proxy);
	// }else if (proxy == BT_MESH_GATT_PROXY_DISABLED )
	// {
	// 	printk("\n\nPROXY Node status (BT_MESH_GATT_PROXY_DISABLED %d)\n\n", proxy);
	// }else if (proxy == BT_MESH_GATT_PROXY_NOT_SUPPORTED  )
	// {
	// 	printk("\n\nPROXY Node status (BT_MESH_GATT_PROXY_NOT_SUPPORTED  %d)\n\n", proxy);
	// }else
	// {
	// 	printk("\n\nERROR: Failed set PROXY Node Identity (proxy statsu %d)\n\n", proxy);
	// }
	
	// err = bt_mesh_proxy_identity_enable();
	// if (err) {
	// 	printk("\n\nFailed advertise using PROXY Node Identity (err %d)\n\n", err);
	// }


#if (CONFIG_SPI_NOR - 0) ||				\
	DT_NODE_HAS_STATUS(DT_INST(0, jedec_spi_nor), okay)
#define FLASH_DEVICE DT_LABEL(DT_INST(0, jedec_spi_nor))
#define FLASH_NAME "JEDEC SPI-NOR"
#else
#error Unsupported flash driver
#endif
#define FLASH_SECTOR_SIZE        4096

	const struct device *flash_dev;
	int rc;

	printf("\n" FLASH_NAME " SPI flash testing\n");
	printf("==========================\n");

	flash_dev = device_get_binding(FLASH_DEVICE);

	if (!flash_dev) {
		printf("SPI flash driver %s was not found!\n",
		       FLASH_DEVICE);
	}



/* SHOULD MOVE THE MODEM CONTEXT INIT TO BOARD INIT FUNCTION */
#if defined(CONFIG_MODEM)

	// printf("getting modem info\n\n");

	// int ii;
	// static struct modem_context *mdm_ctx;
	
	// // for (ii = 0; ii < CONFIG_MODEM_RECEIVER_MAX_CONTEXTS; ii++) {
	// for (ii = 0; ii < 5; ii++) {
	// 	mdm_ctx = mdm_receiver_context_from_id(ii);
	// 	if (mdm_ctx) {
	// 		break;
	// 	}
	// }
#endif //if CONFIG_MODEM

#if defined(CONFIG_MODEM)
	// int ii = 0U;
	// while(ii<1){
	// 	// k_sleep(K_SECONDS(30));
	// 	printf("getting modem info\n\n");


	// 	struct modem_context *mdm_ctx;

	// 	/* Display Modem context information - struct in modem_receiver.h */
	// 	printk("Modem receiver:\n");

	// 	for (ii = 0; ii < CONFIG_MODEM_RECEIVER_MAX_CONTEXTS; ii++) {
	// 	// for (ii = 0; ii < 5; ii++) {
	// 		mdm_ctx = modem_context_from_id(ii);
	// 		if (mdm_ctx) {
	// 			printk(
	// 				// "%d:\tUART Name:    %s\n"
	// 				"%d:"
	// 				"\tAlt:          %s\n"
	// 				"\tVari:         %d\n"
	// 				"\tDate:         %s\n"
	// 				"\tTime:         %s\n"		
	// 				"\tLAT:          %s\n"
	// 				"\tLON:          %s\n"					
	// 				"\tManufacturer: %s\n"
	// 				"\tModel:        %s\n"
	// 				"\tRevision:     %s\n"
	// 				"\tIMEI:         %s\n"
	// 				"\tRSSI:         %d\n"
	// 				"\tAlt:          %s\n"
	// 				"\tVari:         %d\n"
	// 				"\tDate:         %s\n"
	// 				"\tTime:         %s\n"		
	// 				"\tLAT:          %s\n"
	// 				"\tLON:          %s\n",				
	// 				ii,
	// 				// mdm_ctx->uart_dev->config->name,
	// 				mdm_ctx->data_localization.alt,
	// 				mdm_ctx->data_localization.uncertainty,
	// 				mdm_ctx->data_localization.date,
	// 				mdm_ctx->data_localization.time,
	// 				mdm_ctx->data_localization.lat,
	// 				mdm_ctx->data_localization.lon,
	// 				mdm_ctx->data_manufacturer,
	// 				mdm_ctx->data_model,
	// 				mdm_ctx->data_revision,
	// 				mdm_ctx->data_imei,
	// 				mdm_ctx->data_rssi,

	// 				// mdm_ctx->data_alt
	// 				mdm_ctx->data_localization.alt,
	// 				mdm_ctx->data_localization.uncertainty,

	// 				mdm_ctx->data_localization.date,
	// 				mdm_ctx->data_localization.time,
	// 				mdm_ctx->data_localization.lat,
	// 				mdm_ctx->data_localization.lon
	// 			);
	// 			k_sleep(K_SECONDS(2));
	// 		}
	// 	}
	// 	// k_sleep(K_SECONDS(10));

	// } /* End of big while loop */
#endif //if CONFIG_MODEM	



printk("***************************************** Start Mesh Provisioner *****************************************\n");
	/* Get remote note configuration info CDC Message */
	get_node_conf_data(0x00, addr);
	provisioner_init();

	k_sleep(K_SECONDS(5));
}
