/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 *  @brief Bluetooth Mesh Sensor Server Model
 */

#ifndef ZEPHYR_INCLUDE_BT_MESH_SENSOR_SRV_H
#define ZEPHYR_INCLUDE_BT_MESH_SENSOR_SRV_H

/* INclude Sensor Model Common Functions */
#include "sensor_model.c"

struct bt_mesh_sensor_srv_cb {
	/* TODO: Add callback funcions */

	// /* Get Sensor Data */
	void (*sensor_get_status)(uint16_t sensor_id, struct net_buf_simple *msg);

/** TODO: Resolve callbacks **/
// sensor_desc_get
// sensor_get
// sensor_col_get
// sensor_series_get



	// 	/* Get sensor cadence status */
	// int (*sensor_get_cadence)(
	// 	struct bt_mesh_sensor_cadence_data *cadence_data);

	// /* Set sensor cadence status */
	// int (*sensor_set_cadence)(
	// 	struct bt_mesh_sensor_cadence_data *cadence_data);

	// /* Get sensor setting Property IDs */
	// int (*sensor_get_settings)(
	// 	struct bt_mesh_sensor_settings_status *sensor_settings_status);

	// /* Get sensor setting Property IDs */
	// int (*sensor_get_setting)(
	// 	struct bt_mesh_sensor_setting_status *setting_status);

	// /* Set sensor setting status */
	// int (*sensor_set_setting)(
	// 	struct bt_mesh_sensor_setting_data *setup_data);


};


/** Mesh Health Server Model Context */
struct bt_mesh_sensor_srv {
	struct bt_mesh_model *model;

	/* Optional callback struct */
	const struct bt_mesh_sensor_srv_cb *cb;

	/* TODO: Add other sensor setup bits */
};


/* Sensor Client Model Context */
extern const struct bt_mesh_model_op sensor_srv_op[];

/** @def BT_MESH_MODEL_SENSOR_SRV
 *
 *  Define a new sensor client model. Note that this API needs to
 *  be repeated for each element which the application wants to
 *  have a sensor client model on.
 *  @param srv_pub  Pointer to a unique struct bt_mesh_model_pub.
 *  @param srv_data Pointer to a unique struct bt_mesh_sensor_srv.
 *
 *  @return New sensor client model instance.
 */
#define BT_MESH_MODEL_SENSOR_SRV(srv_data, srv_pub) \
        BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV,  \
                    sensor_srv_op, srv_pub, srv_data)

#endif /* ZEPHYR_INCLUDE_BT_MESH_SENSOR_SRV_H */