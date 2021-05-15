/*
 * Copyright (c) 2019 Manulytica Ltd
 *
 * This is a function set node time based on SNTP call
 */
#ifndef _MANULYTICA_TIME_C_
#define _MANULYTICA_TIME_C_

#include <zephyr.h>

#include <net/sntp.h>
#include <net/net_config.h>
#include <net/net_event.h>

/* This comes from newlib. */
#include <time.h>
#include <inttypes.h>

int64_t time_base;

int set_time_base(void)
{
	int rc;
	struct sntp_time sntp_time;
	char time_str[sizeof("1970-01-01T00:00:00")];

	rc = sntp_simple("time.google.com", SYS_FOREVER_MS, &sntp_time);
	if (rc == 0) {
		time_base = sntp_time.seconds * MSEC_PER_SEC - k_uptime_get();

		/* Convert time to make sure. */
		time_t now = sntp_time.seconds;
		struct tm now_tm;

		gmtime_r(&now, &now_tm);
		strftime(time_str, sizeof(time_str), "%FT%T", &now_tm);
		// LOG_INF("  Acquired time: %s", log_strdup(time_str));
		printk("  Acquired time: %s \n", time_str);

	} else {
		printk("ERROR: Failed to acquire SNTP, code %d\n", rc);
	}
	return rc;
}

time_t node_k_time(time_t *ptr)
{
	int64_t stamp;
	time_t now;

	stamp = k_uptime_get();
	now = (time_t)((stamp + time_base) / 1000);

	if (ptr) {
		*ptr = now;
	}

	return now;
}

#endif