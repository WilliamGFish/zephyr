
/* board.h - Board-specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _MANULYTICA_BOARD_H_
#define _MANULYTICA_BOARD_H_

// 0x0001
// 0x0002
// 0x00ff
// 0x000f
// 0x00db
// 0x0a0c
// 0x0bbb
// 0x0b1e

#define NODE_ADDR 0x00ff

#if !defined(NODE_ADDR)
	#if defined(CONFIG_BOARD_BBC_MICROBIT)
		#define NODE_ADDR 0x0005
	#elif defined(CONFIG_BOARD_PARTICLE_ARGON)
		#define NODE_ADDR 0x00AA
	#elif defined(CONFIG_BOARD_PARTICLE_BORON)
		#define NODE_ADDR 0x00BB
	#elif defined(CONFIG_BOARD_NRF52840_PCA10056)
		#define NODE_ADDR 0x0840
	#elif defined(CONFIG_BOARD_NRF52_PCA10040)
		#define NODE_ADDR 0x0052
	#else
		#define NODE_ADDR 0x0111
	#endif
#endif

/* added for modem context (Cell Location) info */
#if defined(CONFIG_MODEM)
	// struct mdm_receiver_context *mdm_ctx;
	// struct modem_context mdm_ctx;
#endif

/* Added for model period divisor updates sensor_setup model */
	int32_t bt_mesh_model_pub_period_get(struct bt_mesh_model *mod);

	void board_button_1_pressed(void);
	uint16_t board_set_target(void);
	void board_play(const char *str);

	// Sensor Additions
	void set_sensor_data();
	void send_sensor_data(uint16_t sensor_id);
	void cmd_health_fault_add();
	void cmd_health_del_fault();

#if defined(CONFIG_BOARD_BBC_MICROBIT)
void board_init(uint16_t *addr);
void board_play_tune(const char *str);
void board_heartbeat(uint8_t hops, uint16_t feat);
void board_other_dev_pressed(uint16_t addr);
void board_attention(bool attention);

#else
static inline void board_init(uint16_t *addr)
{
	*addr = NODE_ADDR;
}

static inline void board_play_tune(const char *str)
{
}

void board_heartbeat(uint8_t hops, uint16_t feat)
{
}

void board_other_dev_pressed(uint16_t addr)
{
}

void board_attention(bool attention)
{
}
#endif

/* End of Define trap */
#endif 
