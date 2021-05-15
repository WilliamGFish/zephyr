/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MANULYTICA_LIS2DH_H_
#define MANULYTICA_LIS2DH_H_

/* Define constants for Accelorometer */
#define ACCEL_X 1
#define ACCEL_Y 2
#define ACCEL_Z 3
#define ACCEL_XYZ 5

/* Sampling frequency */
struct sensor_value lis2dh_odr = {
    .val1 = 9,
};

/* Trigger value */
struct sensor_value lis2dh_attr_trigger = {
    .val1 = 8,
    .val2 = 2000,

    // .val1 = (int32_t) SENSOR_G / 1000000,
    // .val2 = (int32_t) (SENSOR_G + 0.5) % 1000000,
};

/* Trigger duration value */
struct sensor_value lis2dh_attr_trig_dur = {
    .val1 = 100,
    .val2 = 0,

    // .val1 = (int32_t) SENSOR_G / 1000000,
    // .val2 = (int32_t) (SENSOR_G + 0.5) % 1000000,
};
/* End of Define trap */
#endif