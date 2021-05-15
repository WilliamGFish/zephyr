/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * This is a function to manually provision node
 */
#ifndef _MANULYTICA_NODE_LOCATION_C_
#define _MANULYTICA_NODE_LOCATION_C_

// int get_cell_location(void);

// int get_node_location(void){

int get_node_location(char *latitude, char *longitude, char *altitude)){
	return 0;
}

	// /* added for modem context (Cell Location) info */
	// // #if defined(CONFIG_MODEM)
	// // 	struct mdm_receiver_context *mdm_ctx;
	// // 	lat = mdm_ctx->data_locaization.lat;
	// // 	lon = mdm_ctx->data_locaization.lon;
	// // 	alt = mdm_ctx->data_locaization.alt;
	// // #endif

	//     // if (!get_cell_location()){
	//     //     return -EINVAL;
	//     // }

	// 	return 0;
	// }

#endif