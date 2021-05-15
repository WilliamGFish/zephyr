/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MANULYTICA_EXT_EXT_PRESSURE_C_
#define MANULYTICA_EXT_EXT_PRESSURE_C_

/** @file External Pressure Sensor configuration and readings
 * 
 **/

/*
If you use a 0-16 bar sensor. Assuming you want the readout in bar.
(This needs to part of configuation messaging)

offset is calibration setting.


    pressure = (sensorValue - offset) * 16.0 / (fullScale - offset);

*/



/* End of Define MANULYTICA_EXT_EXT_PRESSURE_C_ trap */
#endif 
