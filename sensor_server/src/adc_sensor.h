/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/** @file
 *  @brief ADC Sensor with Mesh Sensor Setup Model Functions
 */

#include "WIP/sensor_model.h"

#include <math.h>


// #define ADC_DEVICE_NAME		DT_ADC_0_NAME
// #define ADC_DEVICE_NAME		DT_LABEL(DT_INST(0, adc))
#define ADC_DEVICE_NAME		DT_LABEL(DT_ALIAS(adc_0))
#define ADC_RESOLUTION		12
#define ADC_GAIN		    ADC_GAIN_1 // ADC_GAIN_1_6
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID	0
#define ADC_1ST_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN1
// #define ADC_2ND_CHANNEL_ID	2
#define ADC_2ND_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN2


#define ADC_BUFFER_SIZE  2
static int16_t adc_sample_buffer[ADC_BUFFER_SIZE];

/* Define Global Device */
// static struct device *adc_dev;
const struct device *adc_dev;

/* Define External Sensor Cadence Data */
struct bt_mesh_sensor_cadence_data adc_cadence_data = {
    /* Set some defaults */
    .property_id = SENS_PROP_ID_COUNTER,
    // .property_id = SENS_PROP_ID_EXT_TEMP,
};

/* GPIO Callback struct */
static struct gpio_callback adc_gpio_cb;
// const struct gpio_callback adc_gpio_cb;

static const struct adc_channel_cfg adc_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_1ST_CHANNEL_INPUT,
#endif
};
#if defined(ADC_2ND_CHANNEL_ID)
static const struct adc_channel_cfg adc_2nd_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_2ND_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_2ND_CHANNEL_INPUT,
#endif
};
#endif /* defined(ADC_2ND_CHANNEL_ID) */

static const struct adc_channel_cfg adc_empty_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
};

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS  	3300  //!< Reference voltage (in milli volts) used by ADC while doing conversion.
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS 	270  //!< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com.
#define ADC_RES_12BIT                  	4096.0 //!< Maximum digital value for 12-bit ADC conversion.
#define ADC_PRE_SCALING_COMPENSATION   	1    //!< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.
#define ADC_INPUT_OFFSET				0 //Gain in mVolts
#define ADC_THERMISTOR_B_CONST			1

#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)                                   \
	(((((ADC_VALUE)*ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_12BIT) +      \
	  ADC_INPUT_OFFSET) *                                                  \
	 ADC_PRE_SCALING_COMPENSATION)


/* Calcuating Temp in C from temp in K */
#define ADC_RESULT_IN_UNITS(ADC_VALUE)                                         \
	(1 / (log(ADC_RES_12BIT / ADC_VALUE - 1.0) / ADC_THERMISTOR_B_CONST +  \
	      1 / 298.15) -                                                    \
	 273.15)


/** Function Stubs **/
/* Initiasation Funtion */
static int init_adc(void);
 
/* Initiasation Funtion */
static int adc_reset(void);


/* ADC Read Callback */
enum adc_action adc_sampling_callback(const struct device *dev,
				      const struct adc_sequence *sequence,
				      uint16_t sampling_index);


/*  Create Sensor Setting Data/Status */
struct bt_mesh_sensor_setting_status *sensor_status_list[30];

/*  Create Sensor Setting Data/Status */
struct bt_mesh_sensor_settings_status sensor_property_ids;
