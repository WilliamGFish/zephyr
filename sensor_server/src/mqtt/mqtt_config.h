/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Copyright (c) 2019 Manulytica Ltd
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __MQTT_CONFIG_H__
#define __MQTT_CONFIG_H__


// #ifdef CONFIG_NET_CONFIG_SETTINGS
//     #ifdef CONFIG_NET_IPV6
//     #define ZEPHYR_ADDR		CONFIG_NET_CONFIG_MY_IPV6_ADDR
//     #define SERVER_ADDR		CONFIG_NET_CONFIG_PEER_IPV6_ADDR
//     #else
//     #define ZEPHYR_ADDR		CONFIG_NET_CONFIG_MY_IPV4_ADDR
//     #define SERVER_ADDR		CONFIG_NET_CONFIG_PEER_IPV4_ADDR
//     #endif
// #else
//     #ifdef CONFIG_NET_IPV6
//     #define ZEPHYR_ADDR		"2001:db8::1"
//     #define SERVER_ADDR		"2001:db8::2"
//     #else
//     #define ZEPHYR_ADDR		"192.168.0.200"
//     #define SERVER_ADDR		"192.168.0.35"
//     #endif
// #endif

    // #define CONFIG_MQTT_BROKER_USERNAME		""
    // #define CONFIG_MQTT_BROKER_PASSWORD		""
    
    /* Hive HQ Connection */
    // #define SERVER_ADDR		"18.185.216.165"   // broker.hivemq.com
    #define SERVER_ADDR		"35.158.43.238"   // broker.hivemq.com 35.157.242.32 35.158.43.238
    #define CONFIG_MQTT_BROKER_USERNAME		""
    #define CONFIG_MQTT_BROKER_PASSWORD		""    

    // /* ADAFruit Connection */
    // #define SERVER_ADDR		"52.7.124.212"   // io.adafruit.com
    // #define CONFIG_MQTT_BROKER_USERNAME		"BillyTheFish"
    // #define CONFIG_MQTT_BROKER_PASSWORD		"6c991061a6a847ad964432f3cb11f6a7"




#if defined(CONFIG_MQTT_LIB_SOCKS)
#define SOCKS5_PROXY_ADDR	SERVER_ADDR
#define SOCKS5_PROXY_PORT	1080
#endif

#ifdef CONFIG_MQTT_LIB_TLS
#define SERVER_PORT		8883
#else
#define SERVER_PORT		1883
#endif

#define APP_SLEEP_MSECS		1000
#define APP_TX_RX_TIMEOUT       300
#define APP_NET_INIT_TIMEOUT    10000

#define APP_CONNECT_TRIES	2

#define APP_MAX_ITERATIONS	5000

#define APP_MQTT_BUFFER_SIZE	4096
// #define APP_MQTT_BUFFER_SIZE	512

// #define MQTT_CLIENTID		"d:quickstart:sensor:manulytica"
// #define MQTT_CLIENTID		"d:v93xbs:sensor:manulytica"
#define MQTT_CLIENTID		"manulytica"

// // #define CONFIG_MQTT_BROKER_USERNAME		""
// #define CONFIG_MQTT_BROKER_USERNAME		"use-token-auth"
// #define CONFIG_MQTT_BROKER_PASSWORD		"noyoucantbu5ter"


// #define SERVER_ADDR		"52.7.124.212"   // io.adafruit.com
// #define CONFIG_MQTT_BROKER_USERNAME		"BillyTheFish"
// #define CONFIG_MQTT_BROKER_PASSWORD		"6c991061a6a847ad964432f3cb11f6a7"

// #define CONFIG_MQTT_BROKER_USERNAME		""
// #define CONFIG_MQTT_BROKER_PASSWORD		""


/* Set the following to 1 to enable the Bluemix topic format */
#define APP_BLUEMIX_TOPIC	0

/* These are the parameters for the Bluemix topic format */
#if APP_BLUEMIX_TOPIC
#define BLUEMIX_DEVTYPE		"sensor"
#define BLUEMIX_DEVID		"manulytica"
#define BLUEMIX_EVENT		"status"
#define BLUEMIX_FORMAT		"json"
#endif

#endif
