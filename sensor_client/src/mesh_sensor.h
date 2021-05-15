/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 */
#ifndef MANULYTICA_MESH_SENSOR_H_
#define MANULYTICA_MESH_SENSOR_H_

/**
* DEFINE MANULYTICA SENSOR PROPERTY IDs //
*
**/
/** DEFINE SENSOR PROPERTY IDs **/
#define SENS_PROP_ID_ENVIRO		    0x0999

#define SENS_PROP_ID_TEMP  		    0x0700
#define SENS_PROP_ID_HUMIDITY  	    0x0701
#define SENS_PROP_ID_PRESSURE  	    0x0702

#define SENS_PROP_ID_ACCEL_XYZ  	0xF710
#define SENS_PROP_ID_ACCEL_X  		0x0711
#define SENS_PROP_ID_ACCEL_Y  		0x0712
#define SENS_PROP_ID_ACCEL_Z  		0x0713

#define SENS_PROP_ID_MAGN_XYZ  	    0xF720
#define SENS_PROP_ID_MAGN_X  		0x0721
#define SENS_PROP_ID_MAGN_Y  		0x0722
#define SENS_PROP_ID_MAGN_Z  		0x0723

#define SENS_PROP_ID_LIGHT			0xF730
#define SENS_PROP_ID_RED			0x0731
#define SENS_PROP_ID_GREEN			0x0732
#define SENS_PROP_ID_BLUE			0x0734
#define SENS_PROP_ID_IR				0x0735

#define SENS_PROP_ID_PROX			0x0736

// #define SENS_PROP_ID_ALTITUDE
// #define BT_UUID_BAS_BATTERY_LEVEL         BT_UUID_DECLARE_16(0x2a19)


/** Extrenal Sensor Properties **/ 
#define SENS_PROP_ID_EXT_CNT_TOTAL  	0x07A0
#define SENS_PROP_ID_COUNTER  		0x07A1

#define SENS_PROP_ID_EXT_PRESSURE	0x7B1
#define SENS_PROP_ID_EXT_TEMP		0x7B2
#define SENS_PROP_ID_EXT_READING	0x7B3

/* Enumerate the Sensor Headers */
enum {
	SENSOR_MSG_HDR_A = 0U,
	SENSOR_MSG_HDR_B = 1U,
};

/** Define the header structures **/

/** Sensor Marshalled Data Haeder Type
 * Format A is defined as a 
	1-bit Format field (0b0)
	4-bit Length field 
	11-bit Property ID field
This format may be used for Property Values that are 
	not longer than 16 octets
	Property IDs less than 0x0800.
*/
struct sensor_msg_hdr_a {
	uint16_t format:1;
	uint16_t length:4;
	uint16_t property_id:11;
} __packed;

/** Sensor Marshalled Data Haeder Type
Format B is defined as a 
	1-bit Format field (0b1)
	7-bit Length field (uint7 value) (valid range 0x0–0x7F, representing range of 1–127). The value 0x7F represents a length of zero.
	16-bit Property ID field
This format may be used for Property Values that are 
	not longer than 128 octets ( Size not supported by the Sensor Status message)
	Property IDs less than 0x0800.
*/
struct sensor_msg_hdr_b {
	uint16_t format:1;
	uint16_t length:7;
	uint16_t property_id:16;
} __packed;


/** Define Sensor Discriptor Structures **/

/** The Sensor Descriptor state represents the attributes describing the sensor data. 
	This state does not change throughout the lifetime of an element.
	16-bit Sensor Property ID 		 (valid range 0x0001–0xFFFF) Defined in Section 4.1.1.1.
	12-bit Sensor Positive Tolerance (valid range 0x001–0xFFF)   Defined in Section 4.1.1.2.
	12-bit Sensor Negative Tolerance (valid range 0x001–0xFFF)   Defined in Section 4.1.1.3.
	8-bit Sensor Sampling Function   See below					 Defined in Section 4.1.1.4.
	8-Bit Sensor Measurement Period  Time period in seconds		 Defined in Section 4.1.1.5.
	8-Bit Sensor Update Interval 	 Update interval in seconds  Defined in Section 4.1.1.6. 
*/
struct sensor_desc
{
	uint16_t property_id:16;
	uint16_t pos_tolerance:12;
	uint16_t neg_tolerance:12;	
	uint8_t  sampling_func:8;
	uint8_t  measurement_period:8;	
	uint8_t  update_interval:8;	
};

/*
Sensor Sampling Function Definitions
	Value 	Description
	0x00 	Unspecified
	0x01 	Instantaneous
	0x02 	Arithmetic Mean
	0x03 	RMS
	0x04 	Maximum
	0x05 	Minimum
	0x06 	Accumulated. 	(See note Section 4.1.1.4)
	0x07 	Count. 		(See note Section 4.1.1.4)
	0x08–0xFF Reserved for Future Use
*/
#define SENS_SMP_INSTANT 	= 0X01
#define SENS_SMP_MEAN 		= 0X02
#define SENS_SMP_RMS		= 0X03
#define SENS_SMP_MAX		= 0X04
#define SENS_SMP_MIN 		= 0X05
#define SENS_SMP_ACCUM		= 0X06
#define SENS_SMP_COUNT		= 0X07


/* End of Define trap */
#endif