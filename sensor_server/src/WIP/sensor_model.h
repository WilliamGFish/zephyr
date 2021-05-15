/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


/** @file
 *  @brief Bluetooth Mesh Sensor Model Common Functions
 */

#ifndef ZEPHYR_INCLUDE_BT_MESH_SENSOR_MODEL_H
#define ZEPHYR_INCLUDE_BT_MESH_SENSOR_MODEL_H


/** The following are the macro definitions of sensor model messages length, 
 *  and a message is composed of three parts: Opcode + msg_value + MIC
 **/
/* Sensor client messages length */
#define MAX_SENS_STATUS_LEN                     (2 + 8 + 4) /* Maximum single unsegmented message size */
#define BT_MESH_SENSOR_DESCRIPTOR_GET_MSG_LEN   (2 + 2 + 4)
#define BT_MESH_SENSOR_CADENCE_GET_MSG_LEN      (2 + 2 + 4)
#define BT_MESH_SENSOR_CADENCE_SET_MSG_LEN      MAX_SENS_STATUS_LEN /* variable */
#define BT_MESH_SENSOR_CADENCE_STATUS_MSG_LEN   MAX_SENS_STATUS_LEN /* variable  (2 + 8 + 4) */
#define BT_MESH_SENSOR_SETTINGS_GET_MSG_LEN     (2 + 2 + 4)
#define BT_MESH_SENSOR_SETTINGS_STATUS_MSG_LEN  MAX_SENS_STATUS_LEN /* variable  (2 + 4 + 4) */
#define BT_MESH_SENSOR_SETTING_GET_MSG_LEN      (2 + 4 + 4)
#define BT_MESH_SENSOR_SETTING_SET_MSG_LEN      MAX_SENS_STATUS_LEN /* variable */
#define BT_MESH_SENSOR_SETTING_STATUS_MSG_LEN   (2 + 4 + 4)
#define BT_MESH_SENSOR_GET_MSG_LEN              (2 + 2 + 4)
#define BT_MESH_SENSOR_STATUS_MSG_LEN           MAX_SENS_STATUS_LEN /* variable (2 + 8 + 4) */
#define BT_MESH_SENSOR_COLUMN_GET_MSG_LEN       MAX_SENS_STATUS_LEN /* variable */
#define BT_MESH_SENSOR_SERIES_GET_MSG_LEN       MAX_SENS_STATUS_LEN /* variable */



/* Define the data structures required for the BT Mesh Sensor Server Model (Client and Server inc Setup Srv) */

/** @brief Mesh Sensor Model Property Marshelled Data.
 *
 * [uint16_t]  node_id:   Node ID of sending Sensor SRV
 * [uint16_t]  property_id:   Property ID of Sensor Data Type
 * [struct]  sensor_value val[3]:   Array of Sensor Data values (3 Elements)
 */
struct bt_mesh_sensor_property_value {
	uint16_t node_id;
	uint16_t property_id;
    /* TODO: Maybe chnage to a pointer rather than 3 empty items */
    struct sensor_value val[3];
};


struct bt_mesh_sensor_setting_data {
    uint16_t sensor_property_id;         /* Property ID identifying a sensor                 */
    uint16_t sensor_setting_property_id; /* Setting ID identifying a setting within a sensor */
    // struct net_buf_simple *sensor_setting_raw; /* Raw value for the setting */
    struct sensor_value sensor_setting_val; /* Snesor value for the setting */
};

struct bt_mesh_sensor_setting_status {
    uint16_t sensor_property_id;         /* Property ID identifying a sensor                    */
    uint16_t sensor_setting_property_id; /* Setting ID identifying a setting within a sensor    */
    uint8_t  sensor_setting_access;      /* Read/Write access rights for the setting (optional set to 0x0) */
    // struct net_buf_simple *sensor_setting_raw; /* Raw value for the setting */
    struct sensor_value sensor_setting_val; /* Snesor value for the setting */
};

/** special structure to hold sensor set-up/configuration attributes */
struct bt_mesh_sensor_setting_attr{
	char *dev_name; /* Name of device binding */
	// uint8_t type; /* Is this a trigger atribute of just a sensor attr */
	struct bt_mesh_sensor_setting_status sensor_settings;  
};

enum sensor_setting_attr_type {
    SENSOR_SETTING_ATTR = 0,
    SENSOR_SETTING_TRIG,
    SENSOR_SETTING_ADC,
};


struct bt_mesh_sensor_settings_status {
    uint16_t sensor_property_id;              /* Property ID identifying a sensor                        */
    struct net_buf_simple *sensor_setting_property_ids; /* A sequence of N sensor setting property IDs (optional) */
};

 struct bt_mesh_sensor_cadence_data {
    uint16_t property_id;                          /* Property ID for the sensor                               */
    uint8_t  fast_cadence_period_divisor : 7,      /* Divisor for the publish period                           */
          status_trigger_type : 1;              /* The unit and format of the Status Trigger Delta fields   */
    struct sensor_value status_trigger_delta_down; /* Delta down value that triggers a status message      */
    struct sensor_value status_trigger_delta_up; /* Delta up value that triggers a status message          */
    uint8_t  status_min_interval;                  /* Minimum interval between two consecutive Status messages */
    struct sensor_value fast_cadence_low;      /* Low value for the fast cadence range                     */
    struct sensor_value fast_cadence_high;     /* High value for the fast cadence range                    */    
};



struct bt_mesh_sensor_descriptor_status {
    struct net_buf_simple *descriptor; /* Sequence of 8-octet sensor descriptors (optional) */
};

// struct bt_mesh_sensor_cadence_status {
//     uint16_t property_id;              /* Property for the sensor                       */
//     struct net_buf_simple *sensor_cadence_value; /* Value of sensor cadence state */
// };



struct bt_mesh_sensor_status {
    struct net_buf_simple *marshalled_sensor_data; /* Value of sensor data state (optional) */
};

struct bt_mesh_sensor_column_status {
    uint16_t property_id;             /* Property identifying a sensor and the Y axis  */
    struct net_buf_simple *sensor_column_value; /* Left values of sensor column status */
};

struct bt_mesh_sensor_series_status {
    uint16_t property_id;             /* Property identifying a sensor and the Y axis  */
    struct net_buf_simple *sensor_series_value; /* Left values of sensor series status */
};

struct bt_mesh_sensor_descriptor_get {
    bool  op_en;       /* Indicate whether optional parameters included */
    uint16_t property_id; /* Property ID for the sensor (optional)         */
};

struct bt_mesh_sensor_cadence_get {
    uint16_t property_id; /* Property ID for the sensor */
};

struct bt_mesh_sensor_cadence_set {
    uint16_t property_id;                     /* Property ID for the sensor                                */
    uint8_t  fast_cadence_period_divisor : 7, /* Divisor for the publish period                            */
          status_trigger_type : 1;         /* The unit and format of the Status Trigger Delta fields    */
    struct net_buf_simple *status_trigger_delta_down; /* Delta down value that triggers a status message */
    struct net_buf_simple *status_trigger_delta_up; /* Delta up value that triggers a status message */
    uint8_t  status_min_interval;             /* Minimum interval between two consecutive Status messages  */
    struct net_buf_simple *fast_cadence_low; /* Low value for the fast cadence range */
    struct net_buf_simple *fast_cadence_high; /* Fast value for the fast cadence range */
};

struct bt_mesh_sensor_settings_get {
    uint16_t sensor_property_id; /* Property ID for the sensor */
};



struct bt_mesh_sensor_setting_get {
    uint16_t sensor_property_id;         /* Property ID identifying a sensor                 */
    uint16_t sensor_setting_property_id; /* Setting ID identifying a setting within a sensor */
};


struct bt_mesh_sensor_get {
    bool  op_en;       /* Indicate whether optional parameters included */
    uint16_t property_id; /* Property ID for the sensor (optional)         */
};

struct bt_mesh_sensor_column_get {
    uint16_t property_id;     /* Property identifying a sensor                */
    struct net_buf_simple *raw_value_x; /* Raw value identifying a column */
};

struct bt_mesh_sensor_series_get {
    bool  op_en;            /* Indicate whether optional parameters included         */
    uint16_t property_id;      /* Property identifying a sensor                         */
    struct net_buf_simple *raw_value_x1; /* Raw value identifying a starting column (optional) */
    struct net_buf_simple *raw_value_x2; /* Raw value identifying a ending column (C.1) */
};


// /****************************************************************************/


#endif /* ZEPHYR_INCLUDE_BT_MESH_SENSOR_MODEL_H */