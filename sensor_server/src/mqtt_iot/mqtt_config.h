/*
 * Copyright (c) 2017 Intel Corporation
 *
 * Copyright (c) 2019 Manulytica Ltd
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __MQTT_CONFIG_H__
#define __MQTT_CONFIG_H__

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

#define APP_CONNECT_TRIES	5

#define APP_MAX_ITERATIONS	5000

#define APP_MQTT_BUFFER_SIZE	2048


#endif
