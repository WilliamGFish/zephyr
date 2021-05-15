/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>

#include <zephyr.h>
#include <net/socket.h>
#include <net/mqtt.h>

#include <sys/printk.h>
#include <random/rand32.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "config.h"
#include "test_certs.h"

/***********************************************/
/* size of stack area used for thread */
#define THREAD_STACKSIZE 10240

/* scheduling priority for thread */
#define PRIORITY 7

/* scheduling priority for thread */
#define THREAD_DELAY 2000

/* Minimum message length */
#define MIN_LENGTH 10

/* FIFO thread for MQTT messages*/
K_FIFO_DEFINE(mqtt_fifo);

/* Wait delay for modem reboot */
#define MODEM_REBOOT_DELAY 3

/* Catch all error */
int rc = 0U;

#define RC_STR(rc) ((rc) == 0 ? "OK" : "ERROR")

#define PRINT_RESULT(func, rc) \
	printk("[%s:%d] %s: %d <%s>\n", __func__, __LINE__, \
	       (func), rc, RC_STR(rc))


// // Moved 
// static struct mqtt_utf8 password;
// static struct mqtt_utf8 username;		   
/***********************************************/

/* Buffers for MQTT client. */
static uint8_t rx_buffer[APP_MQTT_BUFFER_SIZE];
static uint8_t tx_buffer[APP_MQTT_BUFFER_SIZE];

/* The mqtt client struct */
static struct mqtt_client client_ctx;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

#if defined(CONFIG_SOCKS)
static struct sockaddr socks5_proxy;
#endif

/* Socket Poll */
static struct zsock_pollfd fds[1];
static int nfds;

static bool mqtt_connected;

static struct k_work_delayable pub_message;
#if defined(CONFIG_NET_DHCPV4)
static struct k_work_delayable check_network_conn;

/* Network Management events */
#define L4_EVENT_MASK (NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED)

static struct net_mgmt_event_callback l4_mgmt_cb;
#endif

#if defined(CONFIG_DNS_RESOLVER)
static struct zsock_addrinfo hints;
static struct zsock_addrinfo *haddr;
#endif

static K_SEM_DEFINE(mqtt_start, 0, 1);

/* Application TLS configuration details */
#define TLS_SNI_HOSTNAME CONFIG_CLOUD_HOSTNAME
#define APP_CA_CERT_TAG 1
#define APP_PRIVATE_SERVER_KEY_TAG 1

static sec_tag_t m_sec_tags[] = {
	APP_CA_CERT_TAG,
	APP_PRIVATE_SERVER_KEY_TAG,
};

static uint8_t topic[] = "devices/" MQTT_CLIENTID "/messages/devicebound/#";
static struct mqtt_topic subs_topic;
static struct mqtt_subscription_list subs_list;

static void mqtt_event_handler(struct mqtt_client *const client,
			       const struct mqtt_evt *evt);

static int tls_init(void)
{
	int err = 0U;
#if defined (CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	if(CONFIG_CLOUD_HOSTNAME == "mqtt.tago.io"){
		printk("Registering certificate...\n");
		err = tls_credential_add(APP_CA_CERT_TAG, TLS_CREDENTIAL_CA_CERTIFICATE,
					ca_certificate, sizeof(ca_certificate));
		if (err < 0) {
			
			printk("ERROR:Failed to register public certificate: %d \n", err);
			return err;
		}

		err = tls_credential_add(APP_PRIVATE_SERVER_KEY_TAG, TLS_CREDENTIAL_PRIVATE_KEY,
								private_key, sizeof(private_key));
		if (err < 0) {
				printk("ERROR:Failed to register private key: %d \n", err);
				return err;
		}	
	}
#endif
	return err;
}

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
	int rc = -EINVAL;
	printk("waiting..........nfds:%d\n", nfds);

	if (nfds <= 0) {
		return rc;
	}

	rc = zsock_poll(fds, nfds, timeout);
	if (rc < 0) {
		printk("ERROR:zsock_poll error: %d", errno);
		return -errno;
	}

	return 0;
}

static void broker_init(void)
{
	struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;

	broker4->sin_family = AF_INET;
	broker4->sin_port = htons(SERVER_PORT);

#if defined(CONFIG_DNS_RESOLVER)
	printk("\nGetting broker adrress DHCP\n");
	if (haddr->ai_addr) {
		net_ipaddr_copy(&broker4->sin_addr,
				&net_sin(haddr->ai_addr)->sin_addr);
}
#else
	zsock_inet_pton(AF_INET, SERVER_ADDR, &broker4->sin_addr);
#endif


#if defined(CONFIG_SOCKS)
	struct sockaddr_in *proxy4 = (struct sockaddr_in *)&socks5_proxy;

	proxy4->sin_family = AF_INET;
	proxy4->sin_port = htons(SOCKS5_PROXY_PORT);
	zsock_inet_pton(AF_INET, SOCKS5_PROXY_ADDR, &proxy4->sin_addr);
#endif
}

static void client_init(struct mqtt_client *client)
{
	static struct mqtt_utf8 password;
	static struct mqtt_utf8 username;
	struct mqtt_sec_config *tls_config;

	mqtt_client_init(client);

	broker_init();

	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_event_handler;

	client->client_id.utf8 = (uint8_t *)MQTT_CLIENTID;
	client->client_id.size = strlen(MQTT_CLIENTID);

	password.utf8 = (uint8_t *)CONFIG_CLOUD_PASSWORD;
	password.size = strlen(CONFIG_CLOUD_PASSWORD);

	client->password = &password;

	username.utf8 = (uint8_t *)CONFIG_CLOUD_USERNAME;
	username.size = strlen(CONFIG_CLOUD_USERNAME);

	client->user_name = &username;

	client->protocol_version = MQTT_VERSION_3_1_1;

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	/* MQTT transport configuration */
// 	#if defined (CONFIG_MQTT_LIB_TLS)
// 	client->transport.type = MQTT_TRANSPORT_SECURE;
	
// 	tls_config = &client->transport.tls.config;

// // tls_config->peer_verify = 2;
// 	tls_config->peer_verify = TLS_PEER_VERIFY_REQUIRED;
// 	tls_config->cipher_list = NULL;
// 	tls_config->sec_tag_list = m_sec_tags;
// 	tls_config->sec_tag_count = ARRAY_SIZE(m_sec_tags);
// 	tls_config->hostname = TLS_SNI_HOSTNAME;	
// 	#else
// 	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
// 	// tls_config->hostname = NULL;
// 	#endif

	/* MQTT transport configuration */
#if defined(CONFIG_MQTT_LIB_TLS)
#if defined(CONFIG_MQTT_LIB_WEBSOCKET)
	client->transport.type = MQTT_TRANSPORT_SECURE_WEBSOCKET;
#else
	client->transport.type = MQTT_TRANSPORT_SECURE;
#endif	

	tls_config = &client->transport.tls.config;

	tls_config->peer_verify = TLS_PEER_VERIFY_REQUIRED;
	tls_config->cipher_list = NULL;
	tls_config->sec_tag_list = m_sec_tags;
	tls_config->sec_tag_count = ARRAY_SIZE(m_sec_tags);	
	tls_config->hostname = TLS_SNI_HOSTNAME;

	if(TLS_SNI_HOSTNAME == "mqtt.tago.io"){
		tls_config->peer_verify = TLS_PEER_VERIFY_NONE;
	}
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

#if defined(CONFIG_SOCKS)
	mqtt_client_set_proxy(client, &socks5_proxy,
			      socks5_proxy.sa_family == AF_INET ?
			      sizeof(struct sockaddr_in) :
			      sizeof(struct sockaddr_in6));
#endif
}

static void mqtt_event_handler(struct mqtt_client *const client,
			       const struct mqtt_evt *evt)
{
	struct mqtt_puback_param puback;
	uint8_t data[33];
	int len;
	int bytes_read;

	switch (evt->type) {
	case MQTT_EVT_SUBACK:
		LOG_INF("SUBACK packet id: %u", evt->param.suback.message_id);
		break;

	case MQTT_EVT_UNSUBACK:
		LOG_INF("UNSUBACK packet id: %u", evt->param.suback.message_id);
		break;

	case MQTT_EVT_CONNACK:
		if (evt->result) {
			printk("\n\n\n\nMQTT connect failed %d\n\n\n\n", evt->result);
			break;
		}

		mqtt_connected = true;
		printk("MQTT client connected!");
		break;

	case MQTT_EVT_DISCONNECT:
		printk("MQTT client disconnected %d", evt->result);

		mqtt_connected = false;
		clear_fds();
		break;

	case MQTT_EVT_PUBACK:
		if (evt->result) {
			LOG_ERR("MQTT PUBACK error %d", evt->result);
			break;
		}

		printk("PUBACK packet id: %u\n", evt->param.puback.message_id);
		break;

	case MQTT_EVT_PUBLISH:
		len = evt->param.publish.message.payload.len;

		LOG_INF("MQTT publish received %d, %d bytes", evt->result, len);
		LOG_INF(" id: %d, qos: %d", evt->param.publish.message_id,
			evt->param.publish.message.topic.qos);

		while (len) {
			bytes_read = mqtt_read_publish_payload(&client_ctx,
					data,
					len >= sizeof(data) - 1 ?
					sizeof(data) - 1 : len);
			if (bytes_read < 0 && bytes_read != -EAGAIN) {
				LOG_ERR("failure to read payload");
				break;
			}

			data[bytes_read] = '\0';
			LOG_INF("   payload: %s", log_strdup(data));
			len -= bytes_read;
		}

		puback.message_id = evt->param.publish.message_id;
		mqtt_publish_qos1_ack(&client_ctx, &puback);
		break;

	default:
		printk("Unhandled MQTT event %d", evt->type);
		break;
	}
}

static void subscribe(struct mqtt_client *client)
{
	int err;

	/* subscribe */
	subs_topic.topic.utf8 = topic;
	subs_topic.topic.size = strlen(topic);
	subs_list.list = &subs_topic;
	subs_list.list_count = 1U;
	subs_list.message_id = 1U;

	err = mqtt_subscribe(client, &subs_list);
	if (err) {
		LOG_ERR("Failed on topic %s", topic);
	}
}

// static int publish(struct mqtt_client *client, enum mqtt_qos qos)
static int publish(struct mqtt_client *client, char *mqttbuf, enum mqtt_qos qos)
{
	// char payload[] = "{id=123}";
	char payload[] = "";

	/* Standard mqtt topic */
	// char topic[] = "devices/" MQTT_CLIENTID "/messages/events/";

	/* kaa connection JSON message - kp1/{app uuid}/dcx/{device token}/json */
	// char topic[] = "kp1/bv2fb15bhnjfabjee9r0-v1/dcx/teddy/json";
	// char topic[] = "kp1/bv2fb15bhnjfabjee9r0-v1/dcx/wifi/json";
	char topic[] = "kp1/bvga0llbhnjfabjf4mb0-v1/dcx/wifi/json";


	uint8_t len = strlen(topic);
	struct mqtt_publish_param param;

	param.message.topic.qos = qos;
	param.message.topic.topic.utf8 = (uint8_t *)topic;
	param.message.topic.topic.size = len;
	// param.message.payload.data = payload;
	// param.message.payload.len = strlen(payload);

	// (void)snprintf(param.message.payload.data, strlen(mqttbuf), mqttbuf);
	param.message.payload.data = mqttbuf;
	param.message.payload.len = strlen(mqttbuf);

	param.message_id = sys_rand32_get();
	param.dup_flag = 0U;
	param.retain_flag = 0U;

	return mqtt_publish(client, &param);
}

static void poll_mqtt(void)
{
	int rc;

	while (mqtt_connected) {
		rc = wait(SYS_FOREVER_MS);
		if (rc > 0) {
			mqtt_input(&client_ctx);
		}
	}
}

/* Random time between 10 - 15 seconds
 * If you prefer to have this value more than CONFIG_MQTT_KEEPALIVE,
 * then keep the application connection live by calling mqtt_live()
 * in regular intervals.
 */
static uint8_t timeout_for_publish(void)
{
	return (10 + sys_rand32_get() % 5);
}

static void publish_timeout(struct k_work *work)
{
	int rc;

	if (!mqtt_connected) {
		return;
	}

	// rc = publish(&client_ctx, MQTT_QOS_1_AT_LEAST_ONCE);
	if (rc) {
		LOG_ERR("mqtt_publish ERROR");
		goto end;
	}

	printk("mqtt_publish OK");
end:
	k_work_reschedule(&pub_message, K_SECONDS(timeout_for_publish()));
}

static int try_to_connect(struct mqtt_client *client)
{
	uint8_t retries = 3U;
	int rc;

	if (mqtt_connected) {
		printk("Already connected...\n");
		return 0;
	}	

	LOG_DBG("attempting to connect...");

	while (retries-- && !mqtt_connected) {
		client_init(client);

	printk("MQTT client initalised.......\n");
		rc = mqtt_connect(client);
		if (rc != 0) {
			printk("ERROR: mqtt_connect failed %d\n", rc);
			continue;
		}

		LOG_INF("preparing fds");
		prepare_fds(client);

		LOG_INF("waiting for mqtt responce..");
		rc = wait(APP_SLEEP_MSECS);
		if (rc < 0) {
			printk("ERROR:*************************** ABORTED MQTT CLIENT ***************************  \n\n\n\n");
			mqtt_abort(client);
			return rc;
		}

		rc = mqtt_input(client);
		if (rc < 0) {
			printk("ERROR:*************************** mqtt_input %d ***************************  \n\n\n\n",rc);
		}

		// if (mqtt_connected) {
		// 	// subscribe(client);
		// 	// k_work_reschedule(&pub_message, K_SECONDS(timeout_for_publish()));
		// 	return 0;
		// }

		
		// if (!mqtt_connected) {
		// 	mqtt_abort(client);
		// }

		// wait(10 * MSEC_PER_SEC);

		if (mqtt_connected) {
			printk("....MQTT client connected.......");
			LOG_INF("....MQTT client connected.......");
			// mqtt_input(client);
			return 0;
		}
	}

	printk("\nERROR:....MQTT client NOT connected.......\n");

	return -ENETUNREACH;
}

#if defined(CONFIG_DNS_RESOLVER)
static int get_mqtt_broker_addrinfo(void)
{
	int retries = 3;
	int rc = -EINVAL;

	while (retries--) {
		LOG_INF("Get mqtt broker addr (attempt:%d)", retries);
		hints.ai_family = AF_INET;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = 0;

		// rc = zsock_getaddrinfo(CONFIG_CLOUD_HOSTNAME, "8883",
		// 		       &hints, &haddr);

		rc = zsock_getaddrinfo(CONFIG_CLOUD_HOSTNAME, "1883",
				       &hints, &haddr);


		if (rc == 0) {
			printk("DNS resolved for %s:%d\n",
			CONFIG_CLOUD_HOSTNAME,
			CONFIG_CLOUD_SERVER_PORT);

			return rc;
		}

		printk("ERROR:DNS not resolved for %s:%d, retrying",
			CONFIG_CLOUD_HOSTNAME,
			CONFIG_CLOUD_SERVER_PORT);
	}

	return rc;
}
#endif

static void connect_to_cloud_and_publish(void)
{
	int rc = -EINVAL;

#if defined(CONFIG_NET_DHCPV4)
	while (true) {
		k_sem_take(&mqtt_start, K_FOREVER);
#endif
#if defined(CONFIG_DNS_RESOLVER)
		rc = get_mqtt_broker_addrinfo();
		if (rc) {
			return;
		}
#endif
		rc = try_to_connect(&client_ctx);
		if (rc) {
			return;
		}

		poll_mqtt();
#if defined(CONFIG_NET_DHCPV4)
	}
#endif
}

/* DHCP tries to renew the address after interface is down and up.
 * If DHCPv4 address renewal is success, then it doesn't generate
 * any event. We have to monitor this way.
 * If DHCPv4 attempts exceeds maximum number, it will delete iface
 * address and attempts for new request. In this case we can rely
 * on IPV4_ADDR_ADD event.
 */
#if defined(CONFIG_NET_DHCPV4)
static void check_network_connection(struct k_work *work)
{
	struct net_if *iface;

	if (mqtt_connected) {
		return;
	}

	iface = net_if_get_default();
	if (!iface) {
		goto end;
	}

	if (iface->config.dhcpv4.state == NET_DHCPV4_BOUND) {
		k_sem_give(&mqtt_start);
		return;
	}

	LOG_INF("waiting for DHCP to acquire addr");

end:
	k_work_reschedule(&check_network_conn, K_SECONDS(3));
}
#endif

#if defined(CONFIG_NET_DHCPV4)
static void abort_mqtt_connection(void)
{
	if (mqtt_connected) {
		mqtt_connected = false;
		mqtt_abort(&client_ctx);
		k_work_cancel_delayable(&pub_message);
	}
}

static void l4_event_handler(struct net_mgmt_event_callback *cb,
			     uint32_t mgmt_event, struct net_if *iface)
{
	if ((mgmt_event & L4_EVENT_MASK) != mgmt_event) {
		return;
	}

	if (mgmt_event == NET_EVENT_L4_CONNECTED) {
		/* Wait for DHCP to be back in BOUND state */
		k_work_reschedule(&check_network_conn, K_SECONDS(3));

		return;
	}

	if (mgmt_event == NET_EVENT_L4_DISCONNECTED) {
		abort_mqtt_connection();
		k_work_cancel_delayable(&check_network_conn);

		return;
	}
}
#endif

// void azure_iot_init(void)
void mqtt_init(void)
{
	int rc;

	printk("Waiting for network to setup...");

	rc = tls_init();
	if (rc) {
		return;
	}

#if defined(CONFIG_DNS_RESOLVER)
		rc = get_mqtt_broker_addrinfo();
		if (rc) {
			LOG_ERR("Error: DNS Lookup Failed");
			return;
		}
#endif
	// k_work_init_delayable(&pub_message, publish_timeout);

// #if defined(CONFIG_NET_DHCPV4)
// 	k_work_init_delayable(&check_network_conn, check_network_connection);

// 	net_mgmt_init_event_callback(&l4_mgmt_cb, l4_event_handler,
// 				     L4_EVENT_MASK);
// 	net_mgmt_add_event_callback(&l4_mgmt_cb);
// #endif

	// connect_to_cloud_and_publish();
}


/**********************************************************************************/
static int process_mqtt_and_sleep(struct mqtt_client *client, int timeout)
{
	int64_t remaining = timeout;
	int64_t start_time = k_uptime_get();
	int rc;

	while (remaining > 0 && mqtt_connected) {
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

/* MQTT FIFO Handler */
int fifo_send(char *mqttbuf)
{
	int rc;
	LOG_INF("fifo_send");

	/* Check to ensure there is more than an EOL */
	if (strlen(mqttbuf) < MIN_LENGTH){
		// printk("ERROR: Tried to Published: %d:%s\n", strlen(mqttbuf), mqttbuf);
		// LOG_INF("ERROR: Tried to Published: %d:%s", strlen(mqttbuf), mqttbuf);
		LOG_INF("ERROR: Tried to Publish a SHORT message.. !!!!");
		return -EAGAIN;
	}

restart:
	while (!mqtt_connected){
#if defined(CONFIG_DNS_RESOLVER)
		LOG_INF("getting broker ip address (DNS)");
		rc = get_mqtt_broker_addrinfo();
		if (rc) {
			LOG_INF("ERROR: DNS Lookup");
			return rc;
		}
#endif

		LOG_INF("Caliing try_to_connect()");

		rc = try_to_connect(&client_ctx);
		PRINT_RESULT("try_to_connect", rc);
		
		/* If the network unreachable wait to let modem reset */
		int counter = 0U;
		if (rc == -ENETUNREACH){
			printk("Going to wait for modem...");
			while (counter++ < 2 && !mqtt_connected) {
				printk("..");
				k_sleep(K_SECONDS(MODEM_REBOOT_DELAY));
			}
		}
		// process_mqtt_and_sleep(&client_ctx, APP_SLEEP_MSECS);
	}

	/* Send a keep-alive if needed */
	if(mqtt_keepalive_time_left(&client_ctx) < 40) {
		if (!mqtt_live(&client_ctx)) {
			PRINT_RESULT("mqtt_live", rc);
		}

	}

	/* Check to ensure there is more than an EOL */
	if (strlen(mqttbuf) < MIN_LENGTH){
		// printk("ERROR: Tried to Published: %d:%s\n", strlen(mqttbuf), mqttbuf);
		// LOG_INF("ERROR: Tried to Published: %d:%s", strlen(mqttbuf), mqttbuf);
		LOG_INF("ERROR: Tried to Publish a SHORT message.. !!!!");
		return -EAGAIN;
	}

	rc = publish(&client_ctx, mqttbuf, MQTT_QOS_0_AT_MOST_ONCE);
	// rc = publish(&client_ctx, mqttbuf, MQTT_QOS_1_AT_LEAST_ONCE);
	if (rc != 0){
		printk("ERROR: Publish Failed !\n");
		goto restart;
	}else{
		// LOG_DBG("Published: %d:%s\n", strlen(mqttbuf), mqttbuf);
		LOG_DBG("Published: message...");
	}

	return rc;
}


void fifo_loop(void){
	int rc, err, count = 0U, badcount =0U, failcount = 0U;
	char sent_message[MQTT_MSG_SIZE] = " "; // ORIG 300
	char new_message[MQTT_MSG_SIZE] = " ";
	char empty_message[MQTT_MSG_SIZE] = " ";
	
	struct mqtt_data_t *tx_data;


	// printk("attempting to connect: \n");
	// rc = try_to_connect(&client_ctx);
	// PRINT_RESULT("try_to_connect", rc);

	// rc = mqtt_ping(&client_ctx);
	// PRINT_RESULT("mqtt_ping", rc);

	// // rc = mqtt_live(&client_ctx);
	// if (!mqtt_live(&client_ctx)) {
	// 	PRINT_RESULT("mqtt_live", rc);
	// }

	k_sleep(K_MSEC(20));

	printk(" -------------- STARTING AZURE CLOUD MQTT BUFFER LOOP -------------- \n");
	memcpy(sent_message, empty_message, MQTT_MSG_SIZE);
	memcpy(new_message, empty_message, MQTT_MSG_SIZE);

	while (1) {
		err = 0u;
		rc = 0U;
		tx_data = NULL;
		
		// struct mqtt_data_t *
		tx_data = k_fifo_get(&mqtt_fifo, K_FOREVER); //K_NO_WAIT K_FOREVER

		if (tx_data == NULL){
			printk("n");
			continue;
		}else{
			printk("-");
		}

		// if (strlen(tx_data->message) > MIN_LENGTH){
			memcpy(new_message, empty_message, MQTT_MSG_SIZE);
			memcpy(new_message, tx_data->message, strlen(tx_data->message));
			rc = strlen(new_message);

			err = memcmp(&sent_message, &new_message,
					strlen(new_message));
			
			if(strlen(new_message) != strlen(sent_message)){
				err = 1;
			}

		// }
		
		// if (strlen(tx_data->message) > MIN_LENGTH){
		// 	err = memcmp(&sent_message, &tx_data->message, strlen(tx_data->message));
		// }
		// rc = strlen(tx_data->message);


		if (err != 0 && strlen(new_message) > MIN_LENGTH){
		// if (err != 0){
			// printk("\nMesssage to send (%d): %s \n", strlen(new_message), new_message);
			memcpy(sent_message, empty_message, MQTT_MSG_SIZE);
			memcpy(sent_message, new_message, strlen(new_message));

			// memcpy(sent_message, tx_data->message, strlen(tx_data->message));

			// printk("\nMesssages sent buffer: (%d) %s ::: sent: (%d) %s  \n", strlen(sent_message), sent_message, strlen(tx_data->message), tx_data->message);
				   
			err = fifo_send(new_message);
			// err = fifo_send(tx_data->message);

			// memcpy(sent_message, empty_message, MQTT_MSG_SIZE);
			// memcpy(sent_message, tx_data->message, strlen(tx_data->message));
		
			if (err){
				failcount++;				
				// printk("%d Messsages NOT Sent (%d): %s \n\n",failcount, strlen(tx_data->message), tx_data->message);
				printk("\n%d Messsages NOT Sent \n\n",
				       failcount);
				// k_free(tx_data);
			}else{
				// printk("Messsage SENT (%d): %s \n\n", strlen(tx_data->message), tx_data->message);

				count++;
				if (count % 50 == 0){
					if(IS_ENABLED(CONFIG_MQTT_LIB_TLS)){
						printk("%s SECURE ", CONFIG_CLOUD_HOSTNAME);
					}
					printk("\n%d Messsages Sent: %s \n", count, sent_message);
				}
				// k_free(tx_data);
			}

		}else{
			badcount++;
			// printk("%d Messsages the same or too short: (%d)(%d) %s =  %s  \n",badcount, strlen(sent_message), strlen(tx_data->message), sent_message, tx_data->message);
			// printk("%d Messsages the same or too short: (%d)(%d) \n",badcount, strlen(sent_message), strlen(tx_data->message));
			// printk("%d Messsages the same or too short: (%d) %s \n", badcount, strlen(new_message), new_message);
			printk("x");
			if (badcount % 50 == 0) {
				// printk("\n%d Messsages the same or too short: (%d)(%d) %s =  %s  \n",
				//        badcount, strlen(sent_message),
				//        strlen(new_message), sent_message,
				//        new_message);
				}
			// memcpy(new_message, empty_message, MQTT_MSG_SIZE);
			// memcpy(sent_message, empty_message, MQTT_MSG_SIZE);
			// k_fifo_get(&mqtt_fifo, K_NO_WAIT); //
			// k_free(tx_data);
		}
		
	}
}

K_THREAD_DEFINE(mqtt_out_id, THREAD_STACKSIZE, fifo_loop, NULL, NULL, NULL,
		PRIORITY, 0, 5000);		