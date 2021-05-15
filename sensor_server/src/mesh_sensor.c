/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 */
#ifndef _MANULYTICA_MESH_SENSOR_C_
#define _MANULYTICA_MESH_SENSOR_C_

#include <zephyr.h>
#include <string.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <bluetooth/hci.h>

#include <drivers/sensor.h>

// Application Files
#include "misc.c"
#include "mesh_sensor.h"
#include "fxos8700.c"
#include "bme280.c"
#include "apds9960.c"
#include "max44009.c"
#include "nrftemp.c"

#if !defined(CONFIG_BOARD_BBC_MICROBIT)	
	#include "adc_sensor.c"
#endif


/* Include External Sensor Code if a Dev Board */
#if defined(CONFIG_BOARD_NRF52840_PCA10056) || defined(CONFIG_BOARD_NRF52_PCA10040)
	#include "nrf_buttons.c"
	#include "ext_counter.c"
#endif


/* To define decimal places to help convert sensor data to float */
#define SENSOR_DEVISOR  1000

/* Define global variables */
uint16_t ext_item_count = 0;

/** Build Enviromental sensor message **/
/** TODO: Rewrite for different length values and floating point numners **/
static void sensor_external_msg(struct net_buf_simple *msg)
{
	// Should use HDR A as small amount of data sent
	struct sensor_msg_hdr_a hdr;
	struct sensor_value val; 



#if !defined(CONFIG_BOARD_BBC_MICROBIT)
	hdr.format = SENSOR_MSG_HDR_A;
	// TODO: FIX Length of data sent
	hdr.length = sizeof(int16_t);
	hdr.property_id = SENS_PROP_ID_EXT_PRESSURE;
#else
	hdr.format = SENSOR_MSG_HDR_B;
	// TODO: FIX Length of data sent
	hdr.length = sizeof(int16_t) + sizeof(int16_t);
	hdr.property_id = SENS_PROP_ID_TEMP;
#endif

	/* Initialise Model message */
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS );

	// Add Header to Sensor Message
	net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));

#if defined(CONFIG_BOARD_BBC_MICROBIT)
	// Get Temperature Value in C (struct sensor_value)
	val = mpu_temp_init();
#else
	// Increment the item counter
	external_get_value(SENS_PROP_ID_EXT_PRESSURE, &val);
#endif

	/* Just use the first part for now */
	int16_t a16val1 = val.val1;

	net_buf_simple_add_le16(msg, a16val1 );
	// net_buf_simple_add_le16(msg, a16val2 ); 
}


/* Build Enviromental sensor message */
static void sensor_enviro_msg(struct net_buf_simple *msg)
{
	struct sensor_msg_hdr_a hdr_a;
	struct sensor_msg_hdr_b hdr_b;
	struct sensor_value val[3]; 

	/* Initialise Model message */
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS );

	/*									 */
	/* Add Temperature values to message */
	/*									 */
	hdr_b.format = SENSOR_MSG_HDR_B;
	// TODO: FIX Length of data sent
	hdr_b.length = sizeof(int16_t) + sizeof(int16_t);
	hdr_b.property_id = SENS_PROP_ID_TEMP;

	// Get Temperature Value in C (struct sensor_value)
	val[0] = mpu_temp_init();

	// Add Header to Sensor Message
	net_buf_simple_add_mem(msg, &hdr_b, sizeof(hdr_b));

	// int16_t aval1 = val.val1;
	// int16_t aval2 = val.val2  / SENSOR_DEVISOR;	
	int16_t aval1 = val[0].val1 ;
	int16_t aval2 = val[0].val2  / SENSOR_DEVISOR;	
	
	net_buf_simple_add_le16(msg, aval1);
	net_buf_simple_add_le16(msg, aval2); 	



	/*	TODO: Need to change to Pressure, Humidity etc... */
	/* Add Count values to message just to show something */
	/*									 				  */
	hdr_a.format = SENSOR_MSG_HDR_A;
	// TODO: FIX Length of data sent
	hdr_a.length = sizeof(int16_t);
	hdr_a.property_id = SENS_PROP_ID_EXT_CNT_TOTAL;


	/* Add Header to Sensor Message */
	net_buf_simple_add_mem(msg, &hdr_a, sizeof(hdr_a));

	aval1 = 0;
	if(ext_item_count > 0){
		aval1 = ext_item_count;
	}
	net_buf_simple_add_le16(msg, aval1);


	/*	TODO: Need to change to Pressure, Humidity etc... */
	/* Add Accceleromter values to message */
	/*									   */
	// // Create Header
	// hdr_b.format = SENSOR_MSG_HDR_B;
	// // TODO: FIX Length of data sent
	// hdr_b.length = (sizeof(int16_t)+sizeof(int16_t)) * sizeof(val)/sizeof(val[0]);

	// /* the correct property ID */
	// hdr_b.property_id = SENS_PROP_ID_ACCEL_XYZ;

	// // Add Header to Sensor Message
	// net_buf_simple_add_mem(msg, &hdr_b, sizeof(hdr_b));

	// // Get Acceleromtere Values in m/s (struct sensor_value)
	// fxos_get_accel(ACCEL_XYZ, val);

	// int count = 0 ;
	// int datalength = sizeof(val)/sizeof(val[0]);

	// while (count < datalength) {
	// 	int16_t aval1 = val[count].val1 ;
	// 	int16_t aval2 = val[count].val2  / SENSOR_DEVISOR;

	// 	net_buf_simple_add_le16(msg, aval1);
	// 	net_buf_simple_add_le16(msg, aval2); 

	// count++;
	// }
}


// Build Accelometer Sensor Message
static void sensor_accel_msg(struct net_buf_simple *msg, uint8_t xyz)
{
	struct sensor_msg_hdr_b hdr;
	// struct sensor_value val[1];

	/** Integer part of the value. */
	int32_t aval1;
	/** Fractional part of the value (in one-millionth parts). */
	int32_t aval2;

	/* Initialise Model message */
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS );
	
	if (xyz == ACCEL_XYZ){
		struct sensor_value val[3]; 	// Size array for XYZ values

		// Create Header
		hdr.format = SENSOR_MSG_HDR_B;
		// TODO: FIX Length of data sent
		hdr.length = (sizeof(int16_t)+sizeof(int16_t)) * sizeof(val)/sizeof(val[0]);	// Get the length value multiplied by array length
		// hdr.length = sizeof(val);	// Get the length of val array 

		/* the correct property ID */
		 hdr.property_id = SENS_PROP_ID_ACCEL_XYZ;

		// Add Header to Sensor Message
		net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));

		// Get Acceleromtere Values in m/s (struct sensor_value)
		fxos_get_accel(ACCEL_XYZ, val);

		int count = 0 ;
		int datalength = sizeof(val)/sizeof(val[0]);

		while (count < datalength) {
			aval1 = val[count].val1 ;
			aval2 = val[count].val2 ;
			// printk("int32_t Value: %d.%d \n", aval1, aval2 / SENSOR_DEVISOR); // 3 decimal places

			int16_t a16val1 = aval1;
			int16_t a16val2 = aval2 / SENSOR_DEVISOR;	
			// printk("ARRAY Values: %d.%d \n", count, datalength );
			// printk("int16_t Value: %d.%d \n\n", a16val1, a16val2 );

			net_buf_simple_add_le16(msg, a16val1 );
			net_buf_simple_add_le16(msg, a16val2 ); 

		count++;
		}

	}
	else{		// Handler for single direction get call
		struct sensor_value val[1];
		// Get Acceleromtere Value in C (struct sensor_value)
		fxos_get_accel(xyz, val);
		// fxos_get_accel(ACCEL_XYZ, val);		

		int count = 0 ;
		int datalength = sizeof(val)/sizeof(val[0]);

		while (count < datalength) {

			// Create Header
			hdr.format = SENSOR_MSG_HDR_B;
			// TODO: FIX Length of data sent
			// hdr.length = (sizeof((val[count].val1)+sizeof(val[count].val1)) /2);
			// hdr.length = (sizeof((int16_t)+sizeof(int16_t))) * sizeof(val)/sizeof(val[0]);	// Get the length value multiplied by array length
			hdr.length = sizeof(val);	// Get the length of val array 

			/* TODO: need to add the correct property ID */
			 hdr.property_id = SENS_PROP_ID_ACCEL_X;

			// printk("ARRAY Values: %d.%d \n\n", count, datalength );
			// printk("Header Length:  %d \n", sizeof(hdr));			
			// printk("Hdr.length Field: %d \n\n", hdr.length);

			// aval1 = val[count].val1 ;
			// aval2 = val[count].val2 ;
			// printk("Value: %d.%d \n\n", aval1, aval2 / SENSOR_DEVISOR); // 3 decimal places

			int16_t a16val1 = val[count].val1;
			int16_t a16val2 = val[count].val2  / SENSOR_DEVISOR;	
			// printk("Value: %d.%d \n", a16val1, a16val2 );

			// Add Header to Sensor Message
			net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));

			net_buf_simple_add_le16(msg, a16val1 );
			net_buf_simple_add_le16(msg, a16val2 ); 

		count++;
		}
	}
}


static void sensor_temp_msg(struct net_buf_simple *msg)
{
	struct sensor_msg_hdr_b hdr;
	struct sensor_value val;

	hdr.format = SENSOR_MSG_HDR_B;
	// TODO: FIX Length of data sent
	// hdr.length = (sizeof(val));	
	hdr.length = sizeof(int16_t) + sizeof(int16_t);
	hdr.property_id = SENS_PROP_ID_TEMP;

	// Get Temperature Value in C (struct sensor_value)
	val = mpu_temp_init();

	/* Initialise Model message */
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS );

	// Add Header to Sensor Message
	net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));

	int16_t a16val1 = val.val1;
	int16_t a16val2 = val.val2  / SENSOR_DEVISOR;	
	
	net_buf_simple_add_le16(msg, a16val1 );
	net_buf_simple_add_le16(msg, a16val2 ); 	
}



static void sensor_counter_msg(struct net_buf_simple *msg)
{
	// Should use HDR A as small amount of data sent
	struct sensor_msg_hdr_a hdr;

	hdr.format = SENSOR_MSG_HDR_A;
	// TODO: FIX Length of data sent
	hdr.length = sizeof(int16_t);
	hdr.property_id = SENS_PROP_ID_COUNTER;


	/* Initialise Model message */
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS );

	// Add Header to Sensor Message
	net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));

	// Increment the item counter
	ext_item_count++;

    /* Reset counter at 65000 dec */
    if (ext_item_count > 0xfde8) { // 0xfde8
        ext_item_count = 1; 
    }

	// Send 1 item detected (count=1)
	uint16_t a16val1 = 1 ;
	
	net_buf_simple_add_le16(msg, a16val1 );
	// net_buf_simple_add_le16(msg, a16val2 ); 
}


static void sensor_counter_total_msg(struct net_buf_simple *msg)
{
	// Should use HDR A as small amount of data sent
	struct sensor_msg_hdr_a hdr;

	hdr.format = SENSOR_MSG_HDR_A;
	// TODO: FIX Length of data sent
	hdr.length = sizeof(int16_t);
	hdr.property_id = SENS_PROP_ID_EXT_CNT_TOTAL;

	/* Initialise Model message */
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS );

	/* Add Header to Sensor Message */
	net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));

	int16_t a16val1 = 0;
	if(ext_item_count > 0){
		a16val1 = ext_item_count;
	}
	net_buf_simple_add_le16(msg, a16val1 );
}


static void sens_unknown_msg(uint16_t id, struct net_buf_simple *msg)
{
	struct sensor_msg_hdr_a hdr;

	/*
	 * When the message is a response to a Sensor Get message that
	 * identifies a sensor property that does not exist on the element, the
	 * Length field shall represent the value of zero and the Raw Value for
	 * that property shall be omitted. (Mesh model spec 1.0, 4.2.14)
	 */
	hdr.format = SENSOR_MSG_HDR_A;
	hdr.length = 0U;
	hdr.property_id = id;

	/* Initialise Model message */
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS );
	
	net_buf_simple_add_mem(msg, &hdr, sizeof(hdr));
}


// static void sensor_ext_sensor_msg(struct net_buf_simple *msg){

// };

/* Function that builds sensor_status_msg based on sensor property id */
static void sensor_status_msg(uint16_t id, struct net_buf_simple *msg)
{
	bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENS_STATUS);

	// printk("Looking for Sensor ID: 0x%04x \n", id);

	switch (id) {

	/* Send External Sensor Reading */
	case SENS_PROP_ID_EXT_PRESSURE:
		// printk("Getting status for Sensor ID: 0x%04x \n", SENS_PROP_ID_EXT_PRESSURE);
		sensor_external_msg(msg);
		break;		

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
	}
	/*
	* When the message is a response to a Sensor Get message that
	* identifies a sensor property that does not exist on the element, the
	* Length field shall represent the value of zero and the Raw Value for
	* that property shall be omitted. (Mesh model spec 1.0, 4.2.14)
	*/
}

/* End of Define trap */
#endif