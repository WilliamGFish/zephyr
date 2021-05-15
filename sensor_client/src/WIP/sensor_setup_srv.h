/** @file
 *  @brief Bluetooth Mesh Sensor Setup Server Model
 */

/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_MESH_SENSOR_SETUP_SRV_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_MESH_SENSOR_SETUP_SRV_H_

/**
 * @brief Bluetooth Mesh Sensor Setup Server Model
 * @defgroup bt_mesh_sensor_setup_srv Bluetooth Mesh Sensor Setup Server Model
 * @ingroup bt_mesh
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Include Sensor Model Common Functions */
#include "sensor_model.c"


/* Global Sensor Set-up SVR Setting Structure */
/* to hold sensor set-up/configuration attributes */
#if defined(CONFIG_BOARD_BBC_MICROBIT)	
	struct bt_mesh_sensor_setting_attr sensor_attr_list[3];
#else
	struct bt_mesh_sensor_setting_attr sensor_attr_list[20];	
#endif




/** sensor_data structure to cb function */
struct bt_mesh_sensor_setup_srv_cb {

	/* Get sensor cadence status */
	int (*sensor_get_cadence)(
		struct bt_mesh_sensor_cadence_data *cadence_data);

	/* Set sensor cadence status */
	int (*sensor_set_cadence)(
		struct bt_mesh_model *model,
		struct bt_mesh_sensor_cadence_data sensor_cadence_data);

	/* Get sensor setting Property IDs */
	int (*sensor_get_settings)(
		struct bt_mesh_sensor_settings_status *sensor_settings_ids);

	/* Get sensor setting Property IDs */
	int (*sensor_get_setting)(
		struct bt_mesh_sensor_setting_status *setting_status);

	/* Set sensor setting status */
	int (*sensor_set_setting)(
		struct bt_mesh_sensor_setting_data *setup_data);


/** ODD STUBS to DELETE **/	
	/* Get sensor property IDs */
	int (*sensor_get_ids)(struct bt_mesh_model *model, uint16_t company_id);

};


/** Mesh Sensor Server Model Context */
struct bt_mesh_sensor_setup_srv {
	struct bt_mesh_model *model;

	/* Optional callback struct */
	const struct bt_mesh_sensor_setup_srv_cb *cb;

	/* TODO: Add other sensor setup bits */
};

extern const struct bt_mesh_model_op sensor_setup_srv_op[];


/** @def BT_MESH_MODEL_SENSOR_SETUP_SRV 
 *
 *  Define a new Sensor Setup Server model. Note that this API needs to be
 *  repeated for each element that the application wants to have a
 *  Sensor Setup Server model on. Each instance also needs a unique
 *  bt_mesh_SENSOR_SETUP_SRV and bt_mesh_model_pub context.
 *
 *  @param srv Pointer to a unique struct bt_mesh_sensor_setup_srv.
 *  @param pub Pointer to a unique struct bt_mesh_model_pub.
 *
 *  @return New mesh model instance.
 */
#define BT_MESH_MODEL_SENSOR_SETUP_SRV(srv, pub)                                   \
		BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SETUP_SRV,                   \
			      sensor_setup_srv_op, pub, srv)

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_MESH_SENSOR_SETUP_SRV_H_ */
