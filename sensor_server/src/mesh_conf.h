/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 */
#ifndef MANULYTICA_MESH_CONF_H_
#define MANULYTICA_MESH_CONF_H_


/* Set DEFINES */
#define MOD_LF 0x0000
#define GROUP_ADDR 0xc000

#define HB_ADDR 0xfab0
#define HEALTH_ADDR 0xfab1
#define SENSOR_ROOT_ADDR 0xfac0
#define SENSOR_2ND_ADDR 0xfac1
#define SENSOR_3RD_ADDR 0xfac2

#define GATEWAY_ADDR  0x00ff // Should be f or future ff

#define SENSOR_SRV_CNT_UPD_PERIOD		BT_MESH_PUB_PERIOD_SEC(5) 
#define SENSOR_SRV_ENV_UPD_PERIOD		BT_MESH_PUB_PERIOD_SEC(5) 
#define SENSOR_CLI_UPDATE_PERIOD		BT_MESH_PUB_PERIOD_SEC(5)
#define HEALTH_SRV_UPDATE_PERIOD		BT_MESH_PUB_PERIOD_SEC(10)
#define BT_MESH_PUB_UPDATE_PERIOD		BT_MESH_PUB_PERIOD_100MS(500)


// ----------  MESH KEYS ------------- //
static const uint8_t net_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const uint8_t dev_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const uint8_t app_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const uint16_t net_idx;
static const uint16_t app_idx;
static const uint32_t iv_index;
static uint8_t flags;
static uint16_t addr = NODE_ADDR;
static uint16_t elem_addr_r = NODE_ADDR;
// Added when concidering multiple sensor server pubs
// static uint16_t elem_addr_1 = NODE_ADDR + 1;

/* Sensor SRV Pulishing period */
uint16_t sensor_srv_period = BT_MESH_PUB_PERIOD_SEC(5);


/* End of Define trap */
#endif