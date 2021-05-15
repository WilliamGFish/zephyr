/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/** @file
 *  @brief Bluetooth Mesh Sensor Client Model
 */

#ifndef ZEPHYR_INCLUDE_BT_MESH_SENSOR_CLI_H
#define ZEPHYR_INCLUDE_BT_MESH_SENSOR_CLI_H

#include "sensor_model.c"


/* Sensor Client Model OpCode Context */
extern const struct bt_mesh_model_op sensor_cli_op[];

/** @def BT_MESH_MODEL_SENSOR_CLI
 *
 *  Define a new sensor client model. Note that this API needs to
 *  be repeated for each element which the application wants to
 *  have a sensor client model on.
 *  @param cli_pub  Pointer to a unique struct bt_mesh_model_pub.
 *  @param cli_data Pointer to a unique struct bt_mesh_sensor_cli.
 *
 *  @return New sensor client model instance.
 */
#define BT_MESH_MODEL_SENSOR_CLI(cli_data) \
        BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_CLI,  \
                    sensor_cli_op, NULL, cli_data)



/** Mesh Sensor Client Model Callback */
struct bt_mesh_sensor_cli_cb {
	/* TODO: Add callback funcions */

	/**
	 * Sensor Descriptor Status
	 * Sensor Cadence Status
	 * Sensor Settings Status
	 * Sensor Setting Status
	 * Sensor Status
	 * Sensor Column Status
	 * Sensor Series Status
	 * 
	 * C.1: If the Sensor Descriptor Get message is supported, the Sensor Descriptor Status message shall also be supported; otherwise support is optional.
	 * C.2: If any of the messages: Sensor Cadence Get, Sensor Cadence Set are supported, the Sensor Cadence Status message shall also be supported; otherwise support for the Sensor Cadence Status message is optional.
	 * C.3: If the Sensor Settings Get message is supported, the Sensor Settings Status message shall also be supported; otherwise support for the Sensor Settings Status message is optional.
	 * C.4: If any of the messages: Sensor Setting Get, Sensor Setting Set are supported, the Sensor Setting Status message shall also be supported; otherwise support for the Sensor Setting Status message is optional.
	 * C.5: If the Sensor Get message is supported, the Sensor Status message shall also be supported; otherwise support for the Sensor Status message is optional.
	 * C.6: If the Sensor Column Get message is supported, the Sensor Column Status message shall also be supported; otherwise support for the Sensor Column Status message is optional.
	 * C.7: If the Sensor Series Get message is supported, the Sensor Series Status message shall also be supported; otherwise support for the Sensor Series Status message is optional.
	 * 
	 **/ 



	/* Optional mashelled sensor data formating callback struct */
	int (*sensor_data_format)(struct bt_mesh_sensor_property_value sensor_data, int property_count);

	/* Optional cadence data formating callback struct */
	int (*sensor_cadence_update)(struct bt_mesh_model *model, struct bt_mesh_sensor_cadence_data sensor_cadence_data);	

};

/** Mesh Sensor Client Model Context */
struct bt_mesh_sensor_cli {
	struct bt_mesh_model *model;

	/* Optional callback struct */
	const struct bt_mesh_sensor_cli_cb *cb;

	/* TODO: Add other sensor setup bits */
};


/**
 * @brief This function is called to Format the Marshelled Data.
 *
 * @param[value] bt_mesh_sensor_property_value:   Pointer to sensor property value
  *
 * @return Zero-success, other-fail
 */
int sensor_data_format(struct bt_mesh_sensor_property_value sensor_data, int property_count);


/**
 * @brief This function is called to Update the Sensor Cadence Settings.
 *
 * @param[value] bt_mesh_sensor_cadence_data:   Pointer to Sensor Cadence Settings
  *
 * @return Zero-success, other-fail
 */
int sensor_cadence_update(struct bt_mesh_model *model, struct bt_mesh_sensor_cadence_data sensor_cadence_data);


#endif /* ZEPHYR_INCLUDE_BT_MESH_SENSOR_CLI_H */