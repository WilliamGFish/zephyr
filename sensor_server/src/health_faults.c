/** @file
 *  @brief Bluetooth Mesh shell
 *
 */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _MANULYTICA_FAULTS_C_
#define _MANULYTICA_FAULTS_C_


#include <stdlib.h>
#include <ctype.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <shell/shell.h>
#include <settings/settings.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>

// Application Files
#include "health_faults.h"

// void health_server_fault_register(health_server_t * p_server, uint8_t fault_code);
// void health_server_fault_clear(health_server_t * p_server, uint8_t fault_code);
// bool health_server_fault_is_set(health_server_t * p_server, uint8_t fault_code);

static int health_fault_add(struct bt_mesh_model *model, uint8_t fault_code)
{
	uint8_t i;

	if (!fault_code) {
		// LOG_DBG("The Fault ID must be non-zero!\n");
		return -EINVAL;
	}

	for (i = 0U; i < sizeof(cur_srv_faults); i++) {
		if (!cur_srv_faults[i]) {
			cur_srv_faults[i] = fault_code;
			break;
		}
	}

	if (i == sizeof(cur_srv_faults)) {
		printk("Fault array is full. Use \"del-fault\" to "
			    "clear it \n");
		return 0;
	}

	for (i = 0U; i < sizeof(reg_srv_faults); i++) {
		if (!reg_srv_faults[i]) {
			reg_srv_faults[i] = fault_code;
			break;
		}
	}

	if (i == sizeof(reg_srv_faults)) {
		printk("No space to store more registered faults \n");
	}

	bt_mesh_fault_update(&elements[0]);

	return 0;
}



static int health_del_fault(struct bt_mesh_model *model, uint8_t fault_code)
{
	uint8_t i;
	// printk("health_del_fault function \n\n");

	if (!model) {
		(void)memset(cur_srv_faults, 0, sizeof(cur_srv_faults));
		printk("All current faults cleared \n");
		bt_mesh_fault_update(&elements[0]);
		return 0;
	}

	if (!fault_code) {
		printk("The Fault ID must be non-zero! \n");
		return -EINVAL;
	}

	for (i = 0U; i < sizeof(cur_srv_faults); i++) {
		if (cur_srv_faults[i] == fault_code) {
			cur_srv_faults[i] = 0U;
			printk("Fault cleared 0x%02x \n", fault_code);
		}
	}

	bt_mesh_fault_update(&elements[0]);

	return 0;
}


/* End of Define trap */
#endif