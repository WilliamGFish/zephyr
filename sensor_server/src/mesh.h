
/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Copyright (c) 2019 Manulytica Ltd
 *
 */
#ifndef MANULYTICA_MESH_H_
#define MANULYTICA_MESH_H_


/* External Sensor Reporting Delays */
#if defined(CONFIG_BOARD_BBC_MICROBIT)
#define _COUNT_TIMER_DELAY 		K_MSEC(31)	/* pseudo random numebr */
#define _EXT_SENSOR_TIMER_DELAY K_SECONDS(1)
#else
#define _COUNT_TIMER_DELAY 		K_MSEC(28)	/* pseudo random numebr */
#define _EXT_SENSOR_TIMER_DELAY K_SECONDS(1)

#endif

/* Health Server Fault count Maximum size hisorical register */
#define CUR_FAULTS_MAX 10

/* Emun On/Off states */
#define STATE_OFF		0x00
#define STATE_ON		0x01
#define STATE_DEFAULT	0x01
#define STATE_RESTORE	0x02


/* Gen On-OFf Model Operation Codes */
#define BT_MESH_MODEL_OP_GEN_ONOFF_GET			BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET			BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS		BT_MESH_MODEL_OP_2(0x82, 0x04)

/* Sensor Model (CLI & SRV) Operation Codes */
#define BT_MESH_MODEL_OP_SENS_DESC_GET			BT_MESH_MODEL_OP_2(0x82, 0x30)
#define BT_MESH_MODEL_OP_SENS_GET				BT_MESH_MODEL_OP_2(0x82, 0x31)
#define BT_MESH_MODEL_OP_SENS_COL_GET			BT_MESH_MODEL_OP_2(0x82, 0x32)
#define BT_MESH_MODEL_OP_SENS_SERIES_GET		BT_MESH_MODEL_OP_2(0x82, 0x33)

#define BT_MESH_MODEL_OP_SENS_DESC_STATUS		BT_MESH_MODEL_OP_1(0x51)
#define BT_MESH_MODEL_OP_SENS_STATUS			BT_MESH_MODEL_OP_1(0x52)
#define BT_MESH_MODEL_OP_SENS_COL_STATUS		BT_MESH_MODEL_OP_1(0x53)
#define BT_MESH_MODEL_OP_SENS_SERIES_STATUS		BT_MESH_MODEL_OP_1(0x54)

/* Sensor Model Setup Operation Codes */
#define BT_MESH_MODEL_OP_SENS_CADENCE_GET 		BT_MESH_MODEL_OP_2(0x82, 0x34)
#define BT_MESH_MODEL_OP_SENS_SETTING_GET 		BT_MESH_MODEL_OP_2(0x82, 0x36)
#define BT_MESH_MODEL_OP_SENS_SETTINGS_GET 		BT_MESH_MODEL_OP_2(0x82, 0x35)

#define BT_MESH_MODEL_OP_SENS_CADENCE_SET 		BT_MESH_MODEL_OP_1(0x55)
#define BT_MESH_MODEL_OP_SENS_CADENCE_SET_UNACK BT_MESH_MODEL_OP_1(0x56)
#define BT_MESH_MODEL_OP_SENS_CADENCE_STATUS 	BT_MESH_MODEL_OP_1(0x57)
#define BT_MESH_MODEL_OP_SENS_SETTINGS_STATUS 	BT_MESH_MODEL_OP_1(0x58)
#define BT_MESH_MODEL_OP_SENS_SETTING_SET 		BT_MESH_MODEL_OP_1(0x59)
#define BT_MESH_MODEL_OP_SENS_SETTING_SET_UNACK BT_MESH_MODEL_OP_1(0x5A)
#define BT_MESH_MODEL_OP_SENS_SETTING_STATUS 	BT_MESH_MODEL_OP_1(0x5B)


struct led_onoff_state {
	uint8_t current;
	uint8_t previous;
	uint8_t dev_id;

	uint8_t last_tid;
	uint16_t last_tx_addr;
	int64_t last_msg_timestamp;
};

void mesh_send_hello(void);

uint16_t mesh_get_addr(void);
bool mesh_is_initialized(void);
void mesh_start(void);
int mesh_init(void);



/* End of Define trap */
#endif