/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/** @file
 *  @brief ADC Sensor with Mesh Sensor Setup Model Functions
 */
#include <zephyr.h>
#include <drivers/adc.h>
#include <math.h>

#include <hal/nrf_saadc.h>

#include "mesh.c"
#include "adc_sensor.h"


/* see PR #21606 */
struct io_channel_config {
	const char *label;
	uint8_t channel;
};

struct gpio_channel_config {
	const char *label;
	uint8_t pin;
	uint8_t flags;
};

struct divider_config {
	const struct io_channel_config io_channel;
	const struct gpio_channel_config power_gpios;
	const uint32_t output_ohm;
	const uint32_t full_ohm;
};

static const struct divider_config divider_config = {
#if DT_NODE_HAS_STATUS(EXTSENSOR, okay)
	.io_channel = {
		DT_IO_CHANNELS_LABEL(EXTSENSOR),
		DT_IO_CHANNELS_INPUT(EXTSENSOR),
	},
#if DT_NODE_HAS_PROP(VBATT, power_gpios)
	.power_gpios = {
		DT_GPIO_LABEL(EXTSENSOR, power_gpios),
		DT_GPIO_PIN(EXTSENSOR, power_gpios),
		DT_GPIO_FLAGS(EXTSENSOR, power_gpios),
	},
#endif
	.output_ohm = DT_PROP(EXTSENSOR, output_ohms),
	.full_ohm = DT_PROP(EXTSENSOR, full_ohms),
#else /* EXTSENSOR dont exists */
	.io_channel = {
		DT_LABEL(DT_ALIAS(adc_0)),
	},
#endif /* /EXTSENSOR exists */
};

/* create sample holder */
float last_sample = 0.0;
int ii = 0;


static void ext_value_convert(struct sensor_value *val, int16_t raw)
{
	int32_t value;

	/* Convert units to micro Gauss. Raw magnetic data always has a
	 * resolution of 0.1 uT/LSB, which is equivalent to 0.001 G/LSB. */
	value = raw;

	val->val1 = value ;
	val->val2 = value % value;
}


/** Configure ADC thread worker  **/
static struct k_work adc_gpio_work;


/** Configure the ADC PIN for GPIO (button) reading **/
static uint32_t adc_gpio_read(const struct device *port, unsigned int pin)
{
	// uint32_t val = 0U;

	// gpio_pin_get_raw(port, pin, &val);
	// return val;

	return gpio_pin_get_raw(port, pin);
}

static void gpio_trigger_handler(struct k_work *work)
{
	/* Send unsolicited message of sensor readings */
	if (adc_gpio_read(adc_dev, 0x03) == 0) {
		/* TODO: Increase counter value.... */
		printk("PUSHED THE BUTTON ---- : %d \n", ii);
		send_sensor_data(SENS_PROP_ID_TEMP); // CPU Die Temperature (C)
		send_sensor_data(
			SENS_PROP_ID_COUNTER); // Ext Item Detected (Increment count total)

		// // Increment the item counter
		// ext_item_count++;

		// /* Reset counter at 65000 dec */
		// if (ext_item_count > 0xfde8) { // 0xfde8
		// 	ext_item_count = 1; 
		// }	

		if (ii++ > 10)
		{
			adc_cadence_data.property_id = SENS_PROP_ID_EXT_TEMP;
			// init_adc();
			adc_reset();
		}		
    }

}

static void gpio_triggered(const struct device *dev,
			   struct gpio_callback *cb, unsigned int pins)
{
	k_work_submit(&adc_gpio_work);
}




/*******************************************************************************
 * Calibrate ADC Inputs 
 */
int adc_calibrate(void)
{
	int ret = 0U;
	const struct adc_sequence sequence = {
		.channels = BIT(ADC_1ST_CHANNEL_ID),
		// .buffer = adc_sample_buffer,
		// .buffer_size = sizeof(adc_sample_buffer),
		// .resolution = ADC_RESOLUTION,
		.calibrate = 0b1,
	};

	if (!adc_dev) {
		printk("ERROR: Device not found - ADC DEVICE %s \n", ADC_DEVICE_NAME);
        return ret;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret != 0) {
		printk("adc_read() failed with code %d", ret);
		return ret;
	}	

	return ret;
}


/*******************************************************************************
 * Initaise the ADC Device based on candence Property ID ie counter or not
 */
static int init_adc(void)
{
	int ret = 0U;

	adc_dev = device_get_binding("NULL");

	if (!adc_dev) {
		printk("ERROR: ADC Device not found EMPTY \n");
	}	


	/* Add function to update the Sensor sensor_property_ids */
	struct net_buf_simple buf;
	net_buf_simple_init(&buf, 0);
	// net_buf_simple_add_le16(&buf, SENS_PROP_ID_COUNTER);

	sensor_property_ids.sensor_property_id = adc_cadence_data.property_id;
	sensor_property_ids.sensor_setting_property_ids =  &buf;



	switch (adc_cadence_data.property_id)
	{
	case SENS_PROP_ID_COUNTER :
	case SENS_PROP_ID_EXT_CNT_TOTAL :
		/* code */
		printf("GPIO CONTROLLER \n");
		
		k_work_init(&adc_gpio_work, gpio_trigger_handler);

		// static struct gpio_callback adc_gpio_cb;

		adc_dev = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(sw0), gpios));

		if (!adc_dev) {
			printk("ERROR: Device not found - GPIOS_CONTROLLER DEVICE %s \n", (DT_GPIO_LABEL(DT_ALIAS(sw0), gpios)));
			return ret;
		}

		gpio_pin_configure(adc_dev, 0x03,
				GPIO_INPUT | DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios));
		gpio_pin_interrupt_configure(adc_dev, 0x03,
						GPIO_PULL_UP | GPIO_INT_DEBOUNCE |
							GPIO_INT_LOW_0 |
							GPIO_INT_EDGE_TO_ACTIVE);

		gpio_init_callback(&adc_gpio_cb, gpio_triggered,BIT(0x03));
		gpio_add_callback(adc_dev, &adc_gpio_cb);

		/* Add function to update the Sensor sensor_property_ids */
		// net_buf_simple_add_le16(&buf, SENS_PROP_ID_COUNTER);
		sensor_property_ids.sensor_setting_property_ids =  &buf;

		break;
	
	default: /* Use ADC to read input */
		printf("ADC_DEVICE \n");

		adc_dev = device_get_binding(ADC_DEVICE_NAME);

		if (!adc_dev) {
			printk("ERROR: Device not found - ADC DEVICE %s \n", ADC_DEVICE_NAME);
			return ret;
		}

		ret = adc_channel_setup(adc_dev, &adc_1st_channel_cfg);

		if (ret != 0) {
			printk("Setting up of the first channel failed with code %d", ret);
			return ret;
		}

#if defined(ADC_2ND_CHANNEL_ID)
		ret = adc_channel_setup(adc_dev, &adc_2nd_channel_cfg);
		if (ret != 0) {
			printk("Setting up of the second channel failed with code %d", ret);
			return ret;
		}		
#endif /* defined(ADC_2ND_CHANNEL_ID) */

		adc_calibrate();

		(void)memset(adc_sample_buffer, 0, sizeof(adc_sample_buffer));

		/* Update the Sensor sensor_property_ids */
		// net_buf_simple_add_le16(&buf, SENS_PROP_ID_COUNTER);
		sensor_property_ids.sensor_setting_property_ids =  &buf;

		/* If no errors define/initialise adc Cadence data */
		printk("Setting Cadence Data....\n\n");

		/* TODO: Set the defaults in Header File */
		// adc_cadence_data.property_id = SENS_PROP_ID_EXT_TEMP;

		adc_cadence_data.fast_cadence_low.val1 = -14.0;
		adc_cadence_data.fast_cadence_low.val2 = 00;
		adc_cadence_data.fast_cadence_high.val1 = 100.0;
		adc_cadence_data.fast_cadence_high.val2 = 00;
		adc_cadence_data.status_trigger_delta_down.val1 = -5000;
		adc_cadence_data.status_trigger_delta_down.val2 = 0;
		adc_cadence_data.status_trigger_delta_up.val1 = 5000;
		adc_cadence_data.status_trigger_delta_up.val2 = 0;
		adc_cadence_data.status_trigger_type = 0b1;

		break;
	}

	return ret;
}



/** The rest function will change the ADC sensor cadence settings and 
 * the sensor TYPE, ie Counter, Pressure, Temperature, etc..
 * 
 **/
static int adc_reset(void)
{
	int ret = 0U;

	// adc_dev = device_get_binding("NULL");

	// if (!adc_dev) {
	// 	printk("ERROR: ADC Device not found EMPTY \n");
	// }	

	switch (adc_cadence_data.property_id)
	{
	case SENS_PROP_ID_COUNTER :
	case SENS_PROP_ID_EXT_CNT_TOTAL :

		printf("GPIO CONTROLLER \n");
		/* RESET COUNTER */
		ii = 0;

		/* unconfigure ADC Channels */
		ret = adc_channel_setup(adc_dev, &adc_empty_channel_cfg);
		if (ret != 0) {
			printk("No channel set-up %d", ret);
		}		

		adc_dev = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(sw0), gpios));

		if (!adc_dev) {
			printk("ERROR: Device not found - GPIOS_CONTROLLER DEVICE %s \n", (DT_GPIO_LABEL(DT_ALIAS(sw0), gpios)));
			return ret;
		}
		
		gpio_pin_configure(adc_dev, 0x03,
				GPIO_INPUT | DT_GPIO_FLAGS(DT_ALIAS(sw0), gpios));
		gpio_pin_interrupt_configure(adc_dev, 0x03,
						GPIO_PULL_UP | GPIO_INT_DEBOUNCE |
							GPIO_INT_LOW_0 |
							GPIO_INT_EDGE_TO_ACTIVE);

		gpio_init_callback(&adc_gpio_cb, gpio_triggered,BIT(0x03));
		gpio_add_callback(adc_dev, &adc_gpio_cb);	
		break;
	
	default: /* Use ADC to read input */
		printf("ADC_DEVICE \n");

		/* Disable GPIO Callback */
		// gpio_pin_disable_callback(adc_dev, 0x03);
		gpio_pin_interrupt_configure(adc_dev, 0x03, GPIO_INT_DISABLE);


		adc_dev = device_get_binding(ADC_DEVICE_NAME);
		if (!adc_dev) {
			printk("ERROR: Device not found - ADC DEVICE %s \n", ADC_DEVICE_NAME);
			return ret;
		}

		ret = adc_channel_setup(adc_dev, &adc_1st_channel_cfg);
		if (ret != 0) {
			printk("Setting up of the first channel failed with code %d", ret);
			return ret;
		}

	#if defined(ADC_2ND_CHANNEL_ID)
		ret = adc_channel_setup(adc_dev, &adc_2nd_channel_cfg);
		if (ret != 0) {
			printk("Setting up of the second channel failed with code %d", ret);
			return ret;
		}		
	#endif /* defined(ADC_2ND_CHANNEL_ID) */

		(void)memset(adc_sample_buffer, 0, sizeof(adc_sample_buffer));

		/* If no errors define/initialise adc Cadence data */
		printk("Setting Cadence Data....\n\n");

		/* TODO: Set the defaults in Header File */
		// adc_cadence_data.property_id = SENS_PROP_ID_EXT_TEMP;

		adc_cadence_data.fast_cadence_low.val1 = -14.0;
		adc_cadence_data.fast_cadence_low.val2 = 00;

		adc_cadence_data.fast_cadence_high.val1 = 100.0;
		adc_cadence_data.fast_cadence_high.val2 = 00;

		adc_cadence_data.status_trigger_delta_down.val1 = -5000;
		adc_cadence_data.status_trigger_delta_down.val2 = 0;

		adc_cadence_data.status_trigger_delta_up.val1 = 5000;
		adc_cadence_data.status_trigger_delta_up.val2 = 0;

		adc_cadence_data.status_trigger_type = 0b1;

		break;
	}

	return ret;
}





static void check_samples(int expected_count)
{

}




/*******************************************************************************
 * read adc_sample_one_channel
 */
int read_channel_one(void)
{

	printf("read_channel_one \n");

	int ret = 0U;
	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	if (!adc_dev) {
		printk("ERROR: Device not found - ADC DEVICE %s \n", ADC_DEVICE_NAME);
        return ret;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret != 0) {
		printk("adc_read() failed with code %d", ret);
		return ret;
	}	

	check_samples(1);

	return ret;
}


/*******************************************************************************
 * read adc_sample_two_channels
 */
int read_channel_two(void)
{
	int ret = 0U;

	printf("read_channel_two \n");


#if !defined(ADC_2ND_CHANNEL_ID)
    printk("adc_read() ADC_2ND_CHANNEL_ID not set");
    return -EINVAL;

#else
	const struct adc_sequence sequence = {
		// .channels    = BIT(ADC_1ST_CHANNEL_ID) |
		// 	       BIT(ADC_2ND_CHANNEL_ID),
		.channels    = BIT(ADC_2ND_CHANNEL_ID),				   
		.buffer      = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};




	if (!adc_dev) {
		return ret;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret != 0) {
		printk("adc_read() failed with code %d", ret);
		return ret;
	}	

	// check_samples(2);
	check_samples(1);
#endif

	return ret;
}




/*******************************************************************************
 * test_adc_sample_with_interval
 */
// static enum adc_action adc_sampling_callback(
// 				struct device *dev,
// 				const struct adc_sequence *sequence,
// 				uint16_t sampling_index)
// {
// 	// printk("%s: sampling %d\n", __func__, sampling_index);
// 	return ADC_ACTION_CONTINUE;
// }

int adc_sensor_sampling(void)
{
	int ret = 0U;
	struct adc_sequence_options options = {
		.interval_us     = 300 * 1000UL,
		.callback        = adc_sampling_callback,
		.extra_samplings = ADC_BUFFER_SIZE - 1,
	};


	if (!adc_dev) {
		return -1;
	}

#if !defined(ADC_2ND_CHANNEL_ID) 
	const struct adc_sequence sequence = {
		.options     = &options,
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	ret = adc_read(adc_dev, &sequence);
	int channels = 1;
#else
	const struct adc_sequence sequence = {
		.options     = &options,	
		.channels    = BIT(ADC_2ND_CHANNEL_ID),
		.buffer      = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};
	int channels = 1;
	ret = adc_read(adc_dev, &sequence);
#endif
	if (ret != 0) {
		printk("adc_read() failed with code %d \n", ret);
		return ret;
	}	

	check_samples(channels + options.extra_samplings);

	return ret;
}


/**  External function to return sensor value  **/
void external_get_value(uint16_t property_id,struct sensor_value *sensor_value)
{
	/* Use Global Device (dev) */
	if (adc_dev == NULL) {
		printf("ERROR: Could not get external sensor device\n");
		return;
	}

	if (adc_cadence_data.property_id == SENS_PROP_ID_COUNTER)
	{
		sensor_value->val1 = ext_item_count;
		printk("Counter ID : %d\n", sensor_value->val1);
		return;
	}
	/* Take a semaphor flag */
	// k_sem_take(&sem, K_NO_WAIT);

#if !defined(ADC_2ND_CHANNEL_ID) 
	if (read_channel_one()){
		printf("ERROR: Could not read external sensor device\n");
		return;
	}
#else
	if (read_channel_two()){
		printf("ERROR: Could not read external sensor device\n");
		return;
	}

#endif

	int reading_val;
	float in_min, in_max, out_min, out_max;
	reading_val = adc_sample_buffer[0];
	in_min = 0U;
	out_min = -10;
	in_max = 4095;
	out_max = 1000;

	/* Create divisor structure */
	const struct divider_config *dcp = &divider_config;
	int32_t val = reading_val;
	
	printk("Converting to mV \n");

	adc_raw_to_millivolts(adc_ref_internal(adc_dev),
				ADC_GAIN,
				ADC_RESOLUTION,
				&val);
	int32_t reading_volts = val * (uint64_t)dcp->full_ohm / dcp->output_ohm;
	




	float step_coeficient = (float)(out_max - out_min) / ADC_REF_VOLTAGE_IN_MILLIVOLTS ;
	// float reading_volts = (((reading_val * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / (ADC_RES_12BIT -1)) * 1);
	// float reading_volts = ADC_RESULT_IN_MILLI_VOLTS(reading_val);
	float reading_pressure = reading_volts * step_coeficient;
	float reading_temp = ADC_RESULT_IN_UNITS(reading_val);

	// sensor_value->val1 = adc_sample_buffer[0];
	sensor_value->val1 = (((reading_val - in_min) * ((out_max - out_min)) /
				      (in_max - in_min)) +
			      out_min);

	if (sensor_value->val1 <0 ){
		sensor_value->val1 = 0;
	}

	printf("Tcf %.3f\t", step_coeficient);
	// printf("mVolts %.3f\t", reading_volts);
	printf("mVolts %d\t", reading_volts);
	printf("Adjusted %.3f\t", reading_pressure);
	printf("Tempp (c) %.3f\t", reading_temp);	
	printk("Raw %d \t", reading_val);
	printk("Mapped %d \t", sensor_value->val1);

	printk("\n ");


	/**
 * strange mapping type function 
 * takes double (decimal) and recalibates to min and max range.
 * 
 * double map(double value, double in_min, double istop, double ostart, double ostop) 
 * 
 * 
 *  All values double (int32_t)
 * 
 * double value_map(x, in_min, in_max, out_min, out_max):
    return ((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

**/



/**
 * 
 * 
   Inputs ADC Value from Thermistor and outputs Temperature in Celsius
    requires: include <math.h>
   Utilizes the Steinhart-Hart Thermistor Equation:
      Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]3}
      where A = 0.001129148, B = 0.000234125 and C = 8.76741E-08

   These coefficients seem to work fairly universally, which is a bit of a
   surprise.

   Schematic:
     [Ground] -- [10k-pad-resistor] -- | -- [thermistor] --[Vcc (5 or 3.3v)]
                                                 |
                                            Analog Pin 0

   In case it isn't obvious (as it wasn't to me until I thought about it), the analog ports
   measure the voltage between 0v -> Vcc which for an Arduino is a nominal 5v, but for (say)
   a JeeNode, is a nominal 3.3v.

   The resistance calculation uses the ratio of the two resistors, so the voltage
   specified above is really only required for the debugging that is commented out below

   Resistance = PadResistor * (1024/ADC -1)

   I have used this successfully with some CH Pipe Sensors (http://www.atcsemitec.co.uk/pdfdocs/ch.pdf)
   which be obtained from http://www.rapidonline.co.uk.



#include <math.h>

#define ThermistorPIN A0                 // Analog Pin 0

float vcc = 4.91;                       // only used for display purposes, if used
// set to the measured Vcc.
float pad = 9850;                       // balance/pad resistor value, set this to
// the measured resistance of your pad resistor
float thermr = 10000;                   // thermistor nominal resistance

float Thermistor(int RawADC) {
  long Resistance;
  float Temp;  // Dual-Purpose variable to save space.

  Resistance = pad * ((1024.0 / RawADC) - 1);
  Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius

  // BEGIN- Remove these lines for the function not to display anything
  //Serial.print("ADC: ");
  //Serial.print(RawADC);
  //Serial.print("/1024");                           // Print out RAW ADC Number
  //Serial.print(", vcc: ");
  //Serial.print(vcc,2);
  //Serial.print(", pad: ");
  //Serial.print(pad/1000,3);
  //Serial.print(" Kohms, Volts: ");
  //Serial.print(((RawADC*vcc)/1024.0),3);
  //Serial.print(", Resistance: ");
  //Serial.print(Resistance);
  //Serial.print(" ohms, ");
  // END- Remove these lines for the function not to display anything

  // Uncomment this line for the function to return Fahrenheit instead.
  //temp = (Temp * 9.0)/ 5.0 + 32.0;                  // Convert to Fahrenheit
  return Temp;                                      // Return the Temperature
}

void setup() {
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  Serial.begin(9600);
}

void loop() {
  float temp;
  temp = Thermistor(1023 - analogRead(ThermistorPIN));     // read ADC and  convert it to Celsius
  Serial.print("Celsius: ");
  Serial.print(temp, 2);                            // display Celsius
  //temp = (temp * 9.0)/ 5.0 + 32.0;                  // converts to  Fahrenheit
  //Serial.print(", Fahrenheit: ");
  //Serial.print(temp,1);                             // display  Fahrenheit
  Serial.println("");
  delay(1000);                                      // Delay a bit...
}
 * 
 **/
	return;
}
