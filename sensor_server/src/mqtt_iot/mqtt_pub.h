/*
 *
 * Copyright (c) 2019 Manulytica Ltd
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __MQTT_IOT_PUB_H__
#define __MQTT_IOT_PUB_H__

// /* size of stack area used for thread */
// #define STACKSIZE 2048

// /* scheduling priority for thread */
// #define PRIORITY 7

// /* FIFO thread for MQTT messages*/
// K_FIFO_DEFINE(mqtt_fifo);

// /* Wait delay for modem reboot */
// #define MODEM_REBOOT_DELAY 30

// /*MQTT Data structure */
// struct mqtt_data_t {
// 	void *fifo_reserved; /* 1st word reserved for use by fifo */
// 	char message[1024];
// 	u16_t deviceID;
// 	u16_t propertyID;
// };


#define MQTT_MSG_SIZE 300

/* Define special variables */
/*MQTT Data structure */
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
