/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#define SERVER_ADDR		CONFIG_CLOUD_SERVER_ADDR
#define SERVER_PORT		CONFIG_CLOUD_SERVER_PORT

#if defined(CONFIG_SOCKS)
#define SOCKS5_PROXY_ADDR	CONFIG_PROXY_SOCKS_ADDR
#define SOCKS5_PROXY_PORT	CONFIG_PROXY_SOCKS_PORT
#endif

#define MQTT_CLIENTID		CONFIG_CLOUD_CLIENT_ID

#if defined (CONFIG_MQTT_LIB_TLS)
#define APP_SLEEP_MSECS		1000
#else
#define APP_SLEEP_MSECS		1000
#endif

#define APP_MQTT_BUFFER_SIZE	4096

#endif /* __CONFIG_H__ */
