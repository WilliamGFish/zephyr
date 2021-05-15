/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <net/socket.h>
#include <net/net_mgmt.h>
#include <net/mqtt.h>
#include <random/rand32.h>

#include <sys/printk.h>
#include <string.h>
#include <errno.h>

#include "mqtt_config.h"

/* size of stack area used for thread */
#define STACKSIZE 8192

/* scheduling priority for thread */
#define PRIORITY 7

/* scheduling priority for thread */
#define THREAD_DELAY 000

/* Minimum message length */
#define MIN_LENGTH 30

/* FIFO thread for MQTT messages*/
K_FIFO_DEFINE(mqtt_fifo);


/* Wait delay for modem reboot */
#define MODEM_REBOOT_DELAY 30

/* The mqtt client struct */
static struct mqtt_client client_ctx;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

// #if defined(CONFIG_MQTT_LIB_SOCKS)
// static struct sockaddr_storage socks5_proxy;
// #endif
#if defined(CONFIG_SOCKS)
static struct sockaddr socks5_proxy;
#endif

/* MQTT Login details */
static struct mqtt_utf8 password;
static struct mqtt_utf8 user_name;

static struct pollfd fds[1];
static int nfds;

static bool connected;

#if defined(CONFIG_MQTT_LIB_TLS)

#include "test_certs.h"

#define TLS_SNI_HOSTNAME "localhost"
#define APP_CA_CERT_TAG 1
#define APP_PSK_TAG 2

static sec_tag_t m_sec_tags[] = {
#if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
		APP_CA_CERT_TAG,
#endif
#if defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED)
		APP_PSK_TAG,
#endif
};

static int tls_init(void)
{
	int err = -EINVAL;

#if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
	err = tls_credential_add(APP_CA_CERT_TAG, TLS_CREDENTIAL_CA_CERTIFICATE,
				 ca_certificate, sizeof(ca_certificate));
	if (err < 0) {
		LOG_ERR("Failed to register public certificate: %d", err);
		return err;
	}
#endif

#if defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED)
	err = tls_credential_add(APP_PSK_TAG, TLS_CREDENTIAL_PSK,
				 client_psk, sizeof(client_psk));
	if (err < 0) {
		LOG_ERR("Failed to register PSK: %d", err);
		return err;
	}

	err = tls_credential_add(APP_PSK_TAG, TLS_CREDENTIAL_PSK_ID,
				 client_psk_id, sizeof(client_psk_id) - 1);
	if (err < 0) {
		LOG_ERR("Failed to register PSK ID: %d", err);
	}
#endif

	return err;
}

#endif /* CONFIG_MQTT_LIB_TLS */

static void prepare_fds(struct mqtt_client *client)
{
	if (client->transport.type == MQTT_TRANSPORT_NON_SECURE) {
		fds[0].fd = client->transport.tcp.sock;
	}
#if defined(CONFIG_MQTT_LIB_TLS)
	else if (client->transport.type == MQTT_TRANSPORT_SECURE) {
		fds[0].fd = client->transport.tls.sock;
	}
#endif

	fds[0].events = ZSOCK_POLLIN;
	nfds = 1;
}

static void clear_fds(void)
{
	nfds = 0;
}

static int wait(int timeout)
{
	int ret = 0;

	if (nfds > 0) {
		ret = poll(fds, nfds, timeout);
		if (ret < 0) {
			printk("poll error: %d", errno);
		}
	}

	return ret;
}

void mqtt_evt_handler(struct mqtt_client *const client,
		      const struct mqtt_evt *evt)
{
	int err;

	switch (evt->type) {
	case MQTT_EVT_CONNACK:
		if (evt->result != 0) {
			printk("MQTT connect failed %d\n", evt->result);
			break;
		}

		connected = true;
		printk("[%s:%d] MQTT client connected!\n", __func__, __LINE__);

		break;

	case MQTT_EVT_DISCONNECT:
		printk("[%s:%d] MQTT client disconnected %d\n", __func__,
		       __LINE__, evt->result);

		connected = false;
		clear_fds();

		break;

	case MQTT_EVT_PUBACK:
		if (evt->result != 0) {
			printk("MQTT PUBACK error %d\n", evt->result);
			break;
		}

		printk("[%s:%d] PUBACK packet id: %u\n", __func__, __LINE__,
				evt->param.puback.message_id);

		break;

	case MQTT_EVT_PUBREC:
		if (evt->result != 0) {
			printk("MQTT PUBREC error %d\n", evt->result);
			break;
		}

		printk("[%s:%d] PUBREC packet id: %u\n", __func__, __LINE__,
		       evt->param.pubrec.message_id);

		const struct mqtt_pubrel_param rel_param = {
			.message_id = evt->param.pubrec.message_id
		};

		err = mqtt_publish_qos2_release(client, &rel_param);
		if (err != 0) {
			printk("Failed to send MQTT PUBREL: %d\n", err);
		}

		break;

	case MQTT_EVT_PUBCOMP:
		if (evt->result != 0) {
			printk("MQTT PUBCOMP error %d\n", evt->result);
			break;
		}

		printk("[%s:%d] PUBCOMP packet id: %u\n", __func__, __LINE__,
		       evt->param.pubcomp.message_id);

		break;
		
	case MQTT_EVT_PINGRESP:
		// LOG_INF("PINGRESP packet");
		printk("PINGRESP packet\n");
		break;

	default:
		break;
	}
	if (!connected){
		printk("DISCONNECTED \n");
	}
}

static char *get_mqtt_payload(enum mqtt_qos qos)
{
#if APP_BLUEMIX_TOPIC
	static char payload[100];

	snprintk(payload, sizeof(payload), "{\"d\":{\"temperature\":%d}}",
		 (uint8_t)sys_rand32_get());

#else
	static char payload[] = "DOORS:OPEN_QoSx";

	payload[strlen(payload) - 1] = '0' + qos;
#endif

printk("payload: %s\n", payload);

	return payload;
}

static char *get_mqtt_topic(void)
{
#if APP_BLUEMIX_TOPIC

	return "iot-2/evt/status/fmt/json";
	// return "iot-2/evt/"BLUEMIX_EVENT"/fmt/"BLUEMIX_FORMAT;
#else
	return "BillyTheFish/feeds/manulytica/json";
#endif
}


static int publish(struct mqtt_client *client, char *mqttbuf, enum mqtt_qos qos)
{
	struct mqtt_publish_param param;

// printk("topic: %s \n",get_mqtt_topic());	

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = (uint8_t *)get_mqtt_topic();
	param.message.topic.topic.size =
			strlen(param.message.topic.topic.utf8);
	// param.message.payload.data = get_mqtt_payload(qos);
	param.message.payload.data = mqttbuf;
	param.message.payload.len =
			strlen(param.message.payload.data);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0U;
	param.retain_flag = 0U;

	return mqtt_publish(client, &param);
}

#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")

#define PRINT_RESULT(func, rc) \
	printk("[%s:%d] %s: %d <%s>\n", __func__, __LINE__, \
	       (func), rc, RC_STR(rc))

static void broker_init(void)
{
#if defined(CONFIG_NET_IPV6)
	struct sockaddr_in6 *broker6 = (struct sockaddr_in6 *)&broker;

	broker6->sin6_family = AF_INET6;
	broker6->sin6_port = htons(SERVER_PORT);
	inet_pton(AF_INET6, SERVER_ADDR, &broker6->sin6_addr);

// #if defined(CONFIG_MQTT_LIB_SOCKS)
#if defined(CONFIG_SOCKS)
	struct sockaddr_in6 *proxy6 = (struct sockaddr_in6 *)&socks5_proxy;

	proxy6->sin6_family = AF_INET6;
	proxy6->sin6_port = htons(SOCKS5_PROXY_PORT);
	inet_pton(AF_INET6, SOCKS5_PROXY_ADDR, &proxy6->sin6_addr);
#endif

#else
	struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;

	broker4->sin_family = AF_INET;
	broker4->sin_port = htons(SERVER_PORT);
	inet_pton(AF_INET, SERVER_ADDR, &broker4->sin_addr);

// #if defined(CONFIG_MQTT_LIB_SOCKS)
#if defined(CONFIG_SOCKS)
	struct sockaddr_in *proxy4 = (struct sockaddr_in *)&socks5_proxy;

	proxy4->sin_family = AF_INET;
	proxy4->sin_port = htons(SOCKS5_PROXY_PORT);
	inet_pton(AF_INET, SOCKS5_PROXY_ADDR, &proxy4->sin_addr);
#endif
#endif
}

static void client_init(struct mqtt_client *client)
{
	mqtt_client_init(client);

	broker_init();

	/* Setup MQTT Log-In Details */
	password.utf8 = (uint8_t*)CONFIG_MQTT_BROKER_PASSWORD;
	password.size = strlen(CONFIG_MQTT_BROKER_PASSWORD);
	user_name.utf8 = (uint8_t*)CONFIG_MQTT_BROKER_USERNAME;
	user_name.size = strlen(CONFIG_MQTT_BROKER_USERNAME);


	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_evt_handler;
	client->client_id.utf8 = (uint8_t *)MQTT_CLIENTID;
	client->client_id.size = strlen(MQTT_CLIENTID);
	client->password = &password; // NULL ;
	client->user_name = &user_name; // NULL ;

	client->protocol_version = MQTT_VERSION_3_1_1;

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	/* MQTT transport configuration */
#if defined(CONFIG_MQTT_LIB_TLS)
#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
	client->transport.type = MQTT_TRANSPORT_SECURE_WEBSOCKET;
#else
	client->transport.type = MQTT_TRANSPORT_SECURE;
#endif	

	struct mqtt_sec_config *tls_config = &client->transport.tls.config;

	tls_config->peer_verify = 2;
	// tls_config->peer_verify = TLS_PEER_VERIFY_REQUIRED;
	tls_config->cipher_list = NULL;
	tls_config->sec_tag_list = m_sec_tags;
	tls_config->sec_tag_count = ARRAY_SIZE(m_sec_tags);
#if defined(MBEDTLS_X509_CRT_PARSE_C) || defined(CONFIG_NET_SOCKETS_OFFLOAD)
	tls_config->hostname = TLS_SNI_HOSTNAME;
#else
	tls_config->hostname = NULL;
#endif

#else
#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
	client->transport.type = MQTT_TRANSPORT_NON_SECURE_WEBSOCKET;
// #if defined(CONFIG_MQTT_LIB_SOCKS)
// 	client->transport.type = MQTT_TRANSPORT_SOCKS;
	client->transport.socks5.proxy = &socks5_proxy;
#else
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
#endif
#endif

#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
	client->transport.websocket.config.host = SERVER_ADDR;
	client->transport.websocket.config.url = "/mqtt";
	client->transport.websocket.config.tmp_buf = temp_ws_rx_buf;
	client->transport.websocket.config.tmp_buf_len =
						sizeof(temp_ws_rx_buf);
	client->transport.websocket.timeout = 5 * MSEC_PER_SEC;
#endif

#if defined(CONFIG_SOCKS)
	mqtt_client_set_proxy(client, &socks5_proxy,
			      socks5_proxy.sa_family == AF_INET ?
			      sizeof(struct sockaddr_in) :
			      sizeof(struct sockaddr_in6));
#endif
}

/* In this routine we block until the connected variable is 1 */
static int try_to_connect(struct mqtt_client *client)
{
	int rc, i = 0;

	while (i++ < APP_CONNECT_TRIES && !connected) {

// k_sleep(K_MSEC(2000));
// printk("client_init(client) Attempt:%d    \n\n", i);

		client_init(client);

// k_sleep(K_MSEC(2000));
// printk("mqtt_connect(client) Attempt:%d    \n\n", i);

		rc = mqtt_connect(client);
		if (rc != 0) {
			PRINT_RESULT("mqtt_connect", rc);
			k_sleep(K_MSEC(APP_SLEEP_MSECS));
			continue;
		}

		prepare_fds(client);

		if (wait(APP_SLEEP_MSECS)) {
			mqtt_input(client);
		}

		if (!connected) {
			printk("*************************** ABORTED MQTT CLIENT ***************************  \n\n\n\n");
			mqtt_abort(client);
		}
	}

	if (connected) {
		return 0;
	}

	return -ENETUNREACH;
}

static int process_mqtt_and_sleep(struct mqtt_client *client, int timeout)
{
	int64_t remaining = timeout;
	int64_t start_time = k_uptime_get();
	int rc;

	while (remaining > 0 && connected) {
		if (wait(remaining)) {
			rc = mqtt_input(client);
			if (rc != 0) {
				PRINT_RESULT("mqtt_input", rc);
				return rc;
			}
		}

		rc = mqtt_live(client);
		if (rc != 0 && rc != -EAGAIN) {
			PRINT_RESULT("mqtt_live", rc);
			return rc;
		} else if (rc == 0) {
			rc = mqtt_input(client);
			if (rc != 0) {
				PRINT_RESULT("mqtt_input", rc);
				return rc;
			}
		}

		remaining = timeout + start_time - k_uptime_get();
	}

	return 0;	
}

#define SUCCESS_OR_EXIT(rc) { if (rc != 0) { return; } }
#define SUCCESS_OR_BREAK(rc) { if (rc != 0) { break; } }

// static void publisher(void)
void publisher(char *mqttbuf)
{
	int i, rc;

	printk("attempting to connect: \n");
	rc = try_to_connect(&client_ctx);
	PRINT_RESULT("try_to_connect", rc);
	SUCCESS_OR_EXIT(rc);

	rc = mqtt_ping(&client_ctx);
	PRINT_RESULT("mqtt_ping", rc);
	rc = process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);

	i = 0;
	while (i++ < APP_MAX_ITERATIONS && connected) {

		rc = publish(&client_ctx, mqttbuf, MQTT_QOS_0_AT_MOST_ONCE);
		PRINT_RESULT("mqtt_publish", rc);
		SUCCESS_OR_BREAK(rc);

		rc = process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);
		SUCCESS_OR_BREAK(rc);		
	}

	// rc = mqtt_disconnect(&client_ctx);
	// PRINT_RESULT("mqtt_disconnect", rc);
	// wait(APP_SLEEP_MSECS);

	rc = mqtt_input(&client_ctx);
	PRINT_RESULT("mqtt_input", rc);

	printk("\nLooped!\n");
}


void mqtt_init(void)
{
#if defined(CONFIG_MQTT_LIB_TLS)
	int rc;

	rc = tls_init();
	PRINT_RESULT("tls_init", rc);
#endif

}




/* MQTT FIFO Handler */
int fifo_send(char *mqttbuf)
{
	int rc;

restart:
	/* Check to ensure there is more than an EOL */
	if (strlen(mqttbuf) < 3)	
	{
		return -EAGAIN;
	}

	// printk("attempting to connect: \n");
	while (!connected){
		rc = try_to_connect(&client_ctx);
		PRINT_RESULT("try_to_connect", rc);
		
		/* If the network unreachable wait to let modem reset */
		int counter = 0U;
		if (rc == -ENETUNREACH){
			printk("Going to wait for modem...");
			while (counter++ < 2 && !connected) {
				printk("..");
				k_sleep(K_SECONDS(MODEM_REBOOT_DELAY));
			}
		}

		rc = process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);
	}

	// TODO: stop attempted send if not connected and retry
	rc = publish(&client_ctx, mqttbuf, MQTT_QOS_0_AT_MOST_ONCE);
	// rc = publish(&client_ctx, mqttbuf, MQTT_QOS_1_AT_LEAST_ONCE);
	if (rc != 0){
		printk("ERROR: Publish Failed !\n");
		goto restart;
	}
	return rc;
}


void fifo_loop(void){
	int rc, count = 0U;
	char sent_message[MQTT_MSG_SIZE];

	// printk("MQTT fifo_loop..............................\n");

	printk("attempting to connect: \n");
	rc = try_to_connect(&client_ctx);
	PRINT_RESULT("try_to_connect", rc);

	rc = mqtt_ping(&client_ctx);
	PRINT_RESULT("mqtt_ping", rc);

	// rc = mqtt_live(&client_ctx);
	if (!mqtt_live(&client_ctx)) {
		PRINT_RESULT("mqtt_live", rc);
	}

	k_sleep(K_MSEC(200));
	printk(" -------------- STARTING MQTT BUFFER LOOP -------------- \n");

	while (1) {
		rc = 0u;

		struct mqtt_data_t *tx_data = k_fifo_get(&mqtt_fifo, K_FOREVER); //K_NO_WAIT K_FOREVER

		rc = memcmp(&sent_message, &tx_data->message,
			    strlen(tx_data->message));
// 			printk("Messsages compare: %d \n\n", rc);


		if (rc != 0){
			rc = fifo_send(tx_data->message);
			memcpy(sent_message, tx_data->message, strlen(tx_data->message));	
// 			printk("Messsages sent: %d \n\n", rc);

			if (rc){
				printk("Messsages NOT Sent: %d \n", rc);
				// k_free(tx_data);
			}else{
				count++;
				if (count % 50 == 0){
					printk("Messsages Sent: %d  - %s \n", count, tx_data->message);
				}
				// k_free(tx_data);
			}

		}else{
			// printk("Messsages the same: %d \n", rc);
		}
	}
}

K_THREAD_DEFINE(mqtt_out_id, STACKSIZE, fifo_loop, NULL, NULL, NULL,
		PRIORITY, 0, 0);		