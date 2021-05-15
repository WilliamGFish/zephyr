/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MANULYTICA_EXT_EXT_TEMPERATURE_C_
#define MANULYTICA_EXT_EXT_TEMPERATURE_C_

/** @file External Temperature Sensor configuration and readings
 * 
 **/


/**
 * Convert raw ADC value to equivalent temperature with 5v as ADC reference (should obtain true ref value)
 * Step size of AdC= (5v/1023)=4.887mv = 5mv.  (ref/bitsize)
 * for every degree celcius an Lm35 provides 10mv voltage change.
 * 1 step of ADC=5mv=0.5'c, hence the Raw ADC value can be divided by 2 to get equivalent temp
 * 
**/


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

/* End of Define MANULYTICA_EXT_EXT_TEMPERATURE_C_ trap */
#endif 




