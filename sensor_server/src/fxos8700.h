/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MANULYTICA_FXOS8700_H_
#define MANULYTICA_FXOS8700_H_

/* Define constants for Accelorometer */
#define ACCEL_X 1
#define ACCEL_Y 2
#define ACCEL_Z 3
#define ACCEL_XYZ 5

/* Sampling frequency */
struct sensor_value attr_splrate = {
    .val1 = 6,
    .val2 = 250000,
};


/* Trigger value */
struct sensor_value attr_trigger = {
    .val1 = 11,
    .val2 = 500000,

    // .val1 = (int32_t) SENSOR_G / 1000000,
    // .val2 = (int32_t) (SENSOR_G + 0.5) % 1000000,
};

/* End of Define trap */
#endif