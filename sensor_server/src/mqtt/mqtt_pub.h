/*
 *
 * Copyright (c) 2019 Manulytica Ltd
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __MQTT_PUB_H__
#define __MQTT_PUB_H__

#define MQTT_MSG_SIZE 300

/* Define special variables */
struct mqtt_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	char message[MQTT_MSG_SIZE];
	uint16_t deviceID;
	uint16_t propertyID;
};


/* Define function stubs */
void mqtt_init(void);

/* End of Define trap */
#endif
