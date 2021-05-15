/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MANULYTICA_NRF_BUTTONS_H_
#define MANULYTICA_NRF_BUTTONS_H_

/* GPIO */
extern struct device *button_device[4];
extern struct device *led_device[4];

// void app_gpio_init(void);



/* STUFF TO LOOK AT RE PUBLISHING MESAGES */

/*
static uint32_t button_read(struct device *port, uint32_t pin)
{
	uint32_t val = 0;

	gpio_pin_read(port, pin, &val);
	return val;
}

void publish(struct k_work *work)
{
	int err = 0;

	if (is_randomization_of_TIDs_done == false) {
		return;
	}

	if (button_read(button_device[0], SW0_GPIO_PIN) == 0) {
#if defined(ONOFF)
		bt_mesh_model_msg_init(root_models[3].pub->msg,
				       BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK);
		net_buf_simple_add_u8(root_models[3].pub->msg, 0x01);
		net_buf_simple_add_u8(root_models[3].pub->msg, tid_onoff++);
		err = bt_mesh_model_publish(&root_models[3]);
#elif defined(ONOFF_TT)
		bt_mesh_model_msg_init(root_models[3].pub->msg,
				       BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK);
		net_buf_simple_add_u8(root_models[3].pub->msg, 0x01);
		net_buf_simple_add_u8(root_models[3].pub->msg, tid_onoff++);
		net_buf_simple_add_u8(root_models[3].pub->msg, 0x45);
		net_buf_simple_add_u8(root_models[3].pub->msg, 0x28);
		err = bt_mesh_model_publish(&root_models[3]);
#endif
	}

	if (err) {
		printk("bt_mesh_model_publish: err: %d\n", err);
	}
}
*/




/* End of Define trap */
#endif 
