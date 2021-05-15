/*
 * Copyright (c) 2020 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MANULYTICA_HTS221_h_
#define _MANULYTICA_HTS221_H_

/* Sampling frequency */
struct sensor_value hss221_odr = {
    .val1 = 9,
};

/* Trigger value */
struct sensor_value hss221_attr_trigger = {
    .val1 = 8,
    .val2 = 2000,

    // .val1 = (int32_t) SENSOR_G / 1000000,
    // .val2 = (int32_t) (SENSOR_G + 0.5) % 1000000,
};

/* Trigger duration value */
struct sensor_value hss221_attr_trig_dur = {
    .val1 = 100,
    .val2 = 0,

    // .val1 = (int32_t) SENSOR_G / 1000000,
    // .val2 = (int32_t) (SENSOR_G + 0.5) % 1000000,
};
/* End of Define trap */


#endif