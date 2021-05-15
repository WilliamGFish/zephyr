/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/** @file
 *  @brief Bluetooth Mesh Sensor Set-up Server Model attributes configuration
 */


int add_sensor_attr(){
	int err = 0;

	/** Add FXOS8700 Sensor Attr to Mesh Settings SVR Model **/
	struct sensor_value sensor_setting_val;

	/* Add Sampling Frequncy */
	sensor_setting_val.val1 = 6;
	sensor_setting_val.val2 = 250000;

	/* It does return an error */
	sensor_setting_attr_add(DT_LABEL(DT_INST(0, nxp_fxos8700)),
				SENS_PROP_ID_ACCEL_XYZ,
				SENSOR_ATTR_SAMPLING_FREQUENCY, 0x0,
				sensor_setting_val);
	// sensor_setting_attr_add(DT_INST_0_NXP_FXOS8700_LABEL,
	// 			SENS_PROP_ID_ACCEL_XYZ,
	// 			SENSOR_ATTR_SAMPLING_FREQUENCY, 0x0,
	// 			sensor_setting_val);

    #ifdef CONFIG_FXOS8700_MOTION
    /* Add Motion threshold in G */
    sensor_setting_val.val1 = 10;
    sensor_setting_val.val2 = 800000;
    sensor_setting_attr_add(DT_LABEL(DT_INST(0, nxp_fxos8700)), SENS_PROP_ID_ACCEL_XYZ,
                SENSOR_ATTR_SLOPE_TH, 0x0, sensor_setting_val);
    
    // sensor_setting_attr_add(DT_INST_0_NXP_FXOS8700_LABEL, SENS_PROP_ID_ACCEL_XYZ,
    //             SENSOR_ATTR_SLOPE_TH, 0x0, sensor_setting_val);
    #endif

    sensor_setting_attr_update();
    k_sleep(K_SECONDS(1));


    /* Try to Add DUPLICATE Sampling Frequncy */
    sensor_setting_val.val1 = 6;
    sensor_setting_val.val2 = 250000;
    sensor_setting_attr_add(DT_LABEL(DT_INST(0, nxp_fxos8700)),
                    SENS_PROP_ID_ACCEL_XYZ,
                    SENSOR_ATTR_SAMPLING_FREQUENCY, 0x0,
                    sensor_setting_val);
    // sensor_setting_attr_add(DT_INST_0_NXP_FXOS8700_LABEL,
    //                 SENS_PROP_ID_ACCEL_XYZ,
    //                 SENSOR_ATTR_SAMPLING_FREQUENCY, 0x0,
    //                 sensor_setting_val);


    #ifdef CONFIG_FXOS8700_MOTION
    /* Add Motion threshold in G */
    sensor_setting_val.val1 = 10;
    sensor_setting_val.val2 = 600000;
    sensor_setting_attr_add(DT_LABEL(DT_INST(0, nxp_fxos8700)), SENS_PROP_ID_ACCEL_XYZ,
                SENSOR_ATTR_SLOPE_TH, 0x0, sensor_setting_val);
    // sensor_setting_attr_add(DT_INST_0_NXP_FXOS8700_LABEL, SENS_PROP_ID_ACCEL_XYZ,
    //             SENSOR_ATTR_SLOPE_TH, 0x0, sensor_setting_val);                
    #endif

    sensor_setting_attr_update();

    return err;
}
