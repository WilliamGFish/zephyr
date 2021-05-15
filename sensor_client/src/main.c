/* main.c - Application main entry point */
/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 */
#include <zephyr.h>
#include <settings/settings.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>

#include <drivers/sensor.h>
#include <sys/byteorder.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <string.h>


#include <common/log.h>

// #if defined(CONFIG_WIFI)
// #if defined(CONFIG_MODEM)
#if defined(CONFIG_MQTT_LIB)
   	// Networking Files
	#include "mqtt/mqtt_pub.c"
#endif
// #endif
// #endif

// Application Files
#include "board.h"
#include "mesh.c"
#include "health_faults.c"
#include "battery_srv.c"
#include "watchdog.c"

/* Sensor Set-up Server Sensor Atrr */
#if defined(CONFIG_FXOS8700)	
	#include "sensor_settings.c"
#endif

#include <device.h>

// #include "sensor_cli.c"

/* added for modem context info */
// #include <drivers/modem/modem_receiver.h>

/* Configure work threads */
// static struct k_work sensor_fxos_work;
// static struct k_work sensor_bme280_work;
// static struct k_work sensor_nrftemp_work;
// static struct k_work mesh_work;

static void bt_ready(int err)
{
	if (err) {
		printk("ERROR: Bluetooth init failed (err %d)\n", err);
		return;
	}

	struct bt_le_oob oob;
	
	printk("Bluetooth initialized\n");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("ERROR: Initializing mesh failed (err %d)\n", err);
		return;
	}

 	// /* The unprovisioned beacon uses the device address set by Nordic
 	//  * in the FICR as its UUID and is presumed unique.
 	//  * Use identity address as device UUID */
	// if (bt_le_oob_get_local(BT_ID_DEFAULT, &oob)) {
	// 	printk("Identity Address unavailable\n");
	// } else {
	// 	memcpy(dev_uuid, oob.addr.a.val, 6);
	// }

	/* Use identity address as device UUID */
	if (bt_le_oob_get_local(BT_ID_DEFAULT, &oob)) {
		printk("Identity Address unavailable\n");
	} else {
		memcpy(dev_uuid, oob.addr.a.val, 6);
	}


	printk("Mesh initialized\n");

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		printk("Loading stored settings\n");
		// settings_load();

		if( settings_load() ){
			printk("Loading stored settings !!!!!!!!!!!!!!!!!\n");
			err = bt_mesh_provision(net_key, net_idx, flags, iv_index, addr,
						dev_key);
			if (err == -EALREADY) {
				printk("Using stored settings\n");
			} else if (err) {
				printk("ERROR: Provisioning failed (err %d)\n", err);
				// return;
			} else {
				printk("Provisioning completed!\n");
				mesh_configure();
			}
			return;
		}
		printk("ERROR: Loading stored settings\n");
	} 
	
	printk("Awating to be provisioned !\n");
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

	/* This will be a no-op if settings_load() loaded provisioning info */
	mesh_configure();


// #if NODE_ADDR != GATEWAY_ADDR
// 	/* Heartbeat subcscription is a temporary state (due to there
// 	 * not being an "indefinite" value for the period, so it never
// 	 * gets stored persistently. Therefore, we always have to configure
// 	 * it explicitly.
// 	 */
// 	{
// 		struct bt_mesh_cfg_hb_sub sub = {
// 			.src = GATEWAY_ADDR,
// 			.dst = GROUP_ADDR,
// 			.period = 0x10,
// 		};

// 		bt_mesh_cfg_hb_sub_set(net_idx, addr, &sub, NULL);
// 		printk("Subscribing to heartbeat messages\n");
// 	}
// #endif

	return;
}


// static uint16_t target = SENSOR_ADDR;

// uint16_t board_set_target(void)
// {
// 	switch (target) {
// 	case SENSOR_ADDR:
// 		target = 1U;
// 		break;
// 	case 9:
// 		target = SENSOR_ADDR;
// 		break;
// 	default:
// 		target++;
// 		break;
// 	}

// 	return target;
// }

static uint16_t target = GATEWAY_ADDR;

uint16_t board_set_target(void)
{
	switch (target) {
	case GATEWAY_ADDR:
		target = 1U;
		break;
	case 9:
		target = GATEWAY_ADDR;
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
	// /* Error Trapping */
	// if ((!app_idx) || (!net_idx)){
	// 	printk("ERROR: Could send sensor data (send_sensor_data)\n");
	// 	return;
	// }

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
	LOG_DBG("Getting status for Sensor ID: 0x%04x\n", sensor_id);
	sensor_status_msg(sensor_id, &msg);

	// printk("ctx size: %d ", sizeof(&ctx));
	// printk(" -- msg size: %d \n", sizeof(&msg));

	/* Send message via SENSOR_SVR */
	/* TODO: Mesh model lookup */
	struct bt_mesh_model *sensor_mod =
		bt_mesh_model_find(&elements[0], BT_MESH_MODEL_ID_SENSOR_SRV);

	int err = (bt_mesh_model_send(sensor_mod, &ctx, &msg, NULL, NULL));

	// int err = (bt_mesh_model_send(&root_models[0], &ctx, &msg, NULL, NULL));
	if (err != 0) {
		LOG_DBG("ERROR: Unable to send status for Sensor ID: 0x%04x\n", sensor_id);
		LOG_DBG("ERROR: bt_mesh_model_send Error ID: %d\n", err);
	} else {
		LOG_DBG("MESSAGE: Sensor message sent OpCode: 0x%04x", sensor_id);
	}

	/* Reset message buffer */
	net_buf_simple_reset(&msg);
}

/** Button 1 unsolicitaed messages sending **/
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
#if !defined(CONFIG_BOARD_BBC_MICROBIT)	&& !defined(CONFIG_BOARD_ESP32) && !defined(CONFIG_BOARD_ARDUINO_NANO_33_BLE)
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

/* External Sensor Timer Stuff (ADC) */
void ext_sensor_work_handler(struct k_work *work)
{
	if (bt_mesh_is_provisioned()){

	#if !defined(CONFIG_BOARD_BBC_MICROBIT)	&& !defined(CONFIG_BOARD_ESP32) && !defined(CONFIG_BOARD_ARDUINO_NANO_33_BLE)
		switch (adc_cadence_data.property_id)
		{
		case SENS_PROP_ID_COUNTER :
		case SENS_PROP_ID_EXT_CNT_TOTAL :
			/* Let the GPIO Callback handle things */
			break;
		default:
			// removed as not using ADC
			// adc_sensor_sampling();
			break;
		}

		/* Just to push timed standard readings */
		send_sensor_data(SENS_PROP_ID_TEMP);
		// send_sensor_data(SENS_PROP_ID_HUMIDITY);
		send_sensor_data(SENS_PROP_ID_ACCEL_XYZ);
		// send_sensor_data(SENS_PROP_ID_EXT_CNT_TOTAL);
		
		// send_sensor_data(SENS_PROP_ID_EXT_PRESSURE);
	#else
		// send_sensor_data(SENS_PROP_ID_TEMP);
		// send_sensor_data(SENS_PROP_ID_EXT_CNT_TOTAL);
		// send_sensor_data(SENS_PROP_ID_EXT_PRESSURE);
	#endif	

	}
}

K_WORK_DEFINE(ext_sensor_work, ext_sensor_work_handler);

void ext_sensor_timer_handler(struct k_timer *ext_sensor_dummy){
    k_work_submit(&ext_sensor_work);
}

K_TIMER_DEFINE(ext_sensor_timer, ext_sensor_timer_handler, NULL);

/* DUMMY COUNT HANDLER FOR External Sensor */
void count_work_handler(struct k_work *count_work)
{
	// Increment the item counter
	ext_item_count++;
	/* Reset counter at 65000 dec */
	if (ext_item_count > 0xfde8) { // 0xfde8
		ext_item_count = 1; 
	}		
}

K_WORK_DEFINE(count_work, count_work_handler);

/* Counter Thread Definitions */
void count_timer_handler(struct k_timer *my_count){
    k_work_submit(&count_work);
}

K_TIMER_DEFINE(count_timer, count_timer_handler, NULL);



/*   ------------ MAIN -----------  */
void main(void){
	int err;

	printk("Initializing...\n");

	/* Initialize the Board Subsystem */
	board_init(&addr);

k_sleep(K_SECONDS(2));

	mpu_temp_init();

	fxos_init();
	apds_init();
	bme_init();
	max_init();
#if !defined(CONFIG_BOARD_BBC_MICROBIT)	&& !defined(CONFIG_BOARD_ESP32) && !defined(CONFIG_BOARD_ARDUINO_NANO_33_BLE)
	init_adc();
#endif

	st_hts221_init();


	
	/* Add the sensor settings to senser set-up server attr list */
#if defined(CONFIG_FXOS8700)	
	add_sensor_attr();
#endif


	printk("Unicast address: 0x%04x\n", addr);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}

	/* Flash the lghts to show 'alive' */
	if (!bt_mesh_is_provisioned()){
		// k_sleep(K_SECONDS(10));
		board_init_complete();
	}


#if defined(CONFIG_BOARD_NRF52840_PCA10056) || defined(CONFIG_BOARD_NRF52_PCA10040)
		nrf_buttons_init();
#endif

	printk("Initialisation Completed \n ");
	printk("\n\n");

	/* Update publishing frequncy */
// #if defined(CONFIG_BOARD_BBC_MICROBIT)
// 	sensor_srv_period = BT_MESH_PUB_PERIOD_100MS(20);
// #else
// 	sensor_srv_period = BT_MESH_PUB_PERIOD_100MS(10);
// #endif

#if defined(CONFIG_LIS2DH)
	printk("888888888888888888888888888888888888888888888888888888888888888888888888\n");
	k_sleep(K_SECONDS(1));	
	lis2_init();
	printk("\n888888888888888888888888888888888888888888888888888888888888888888888888\n");
#endif

	/* External Sensor Timer Stuff ?????? */
	/* start periodic timer that for random count */
	k_timer_start(&count_timer,  _COUNT_TIMER_DELAY, _COUNT_TIMER_DELAY);


	/* start periodic timer that expires once every second */
	k_timer_start(&ext_sensor_timer, _EXT_SENSOR_TIMER_DELAY, _EXT_SENSOR_TIMER_DELAY);


	// while(1){
		/* Main Loop for stuff.... */

	// }
}
