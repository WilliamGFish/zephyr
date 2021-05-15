/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * This is a function to manually provision node
 */
#ifndef _MANULYTICA_NODE_LOCATION_C_
#define _MANULYTICA_NODE_LOCATION_C_

int get_cell_location(void);

// int get_node_location(void){

int get_node_location(char *latitude, char *longitude, char *altitude){

	/* added for modem context (Cell Location) info */
	#if defined(CONFIG_MODEM)
		// struct mdm_receiver_context *mdm_ctx;
		// latitude = mdm_ctx->data_localization.lat;
		// longitude = mdm_ctx->data_localization.lon;
		// altitude = mdm_ctx->data_localization.alt;


	// mctx.data_localization.date = mdata.mdm_date;
	// mctx.data_localization.time = mdata.mdm_time;
	// mctx.data_localization.lat = mdata.mdm_lat;
	// mctx.data_localization.lon = mdata.mdm_lon;
	// mctx.data_localization.alt = mdata.mdm_alt;

	#endif

    if (!get_cell_location()){
        return -EINVAL;
    }

    return 0;
}

#endif