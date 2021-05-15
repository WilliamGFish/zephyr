#include <zephyr.h>
#include <adc.h>


#include <hal/nrf_saadc.h>

#define ADC_DEVICE_NAME		DT_ADC_0_NAME
#define ADC_RESOLUTION		10
#define ADC_GAIN		    ADC_GAIN_1_6
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID	0
#define ADC_1ST_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN1
#define ADC_2ND_CHANNEL_ID	2
#define ADC_2ND_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN2


#define BUFFER_SIZE  6
static int16_t adc_sample_buffer[BUFFER_SIZE];

/* Define Global Device */
static struct device *adc_dev;


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



static int init_adc()
{
	int ret = 0U;
	adc_dev = device_get_binding(ADC_DEVICE_NAME);

	if (!adc_dev) {
		printf("ERROR: Device not found - ADC DEVICE %s \n", ADC_DEVICE_NAME);
		return ret;
	}

	ret = adc_channel_setup(adc_dev, &adc_1st_channel_cfg);

	if (ret != 0) {
		printf("Setting up of the first channel failed with code %d", ret);
		return ret;
	}

#if defined(ADC_2ND_CHANNEL_ID)
	ret = adc_channel_setup(adc_dev, &adc_2nd_channel_cfg);
	if (ret != 0) {
		printf("Setting up of the second channel failed with code %d", ret);
		return ret;
	}		
#endif /* defined(ADC_2ND_CHANNEL_ID) */

	(void)memset(adc_sample_buffer, 0, sizeof(adc_sample_buffer));

	return ret;
}



/*******************************************************************************
 * read adc_sample_one_channel
 */
static int test_task_one_channel(void)
{
	int ret = 0U;
	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	if (!adc_dev) {
		return ret;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret != 0) {
		printf("adc_read() failed with code %d", ret);
		return ret;
	}	

	// check_samples(1);

	return ret;
}


/*******************************************************************************
 * read adc_sample_two_channels
 */
static int test_task_two_channels(void)
{
	int ret = 0U;

#if !defined(ADC_2ND_CHANNEL_ID)
    return -EINVAL;
#endif

	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID) |
			       BIT(ADC_2ND_CHANNEL_ID),
		.buffer      = adc_sample_buffer,
		.buffer_size = sizeof(adc_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	if (!adc_dev) {
		return ret;
	}

	ret = adc_read(adc_dev, &sequence);
	if (ret != 0) {
		printf("adc_read() failed with code %d", ret);
		return ret;
	}	

	// check_samples(2);

	return ret;
}









