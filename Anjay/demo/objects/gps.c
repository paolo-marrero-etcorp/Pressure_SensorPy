#include "http.h"
#include <stdbool.h>
#include <stdio.h>
#include <err.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include "gps.h"

int read_gps_stats(GpsStats* gps_stats)
{
	unsigned char* payload;
	int return_code;
    int size_pl;
	int ret_val = http_get("READ_GPS_STATS", "", &return_code, &payload);
	
	if (ret_val == 0)
	{
        size_pl = strlen(payload);
        gps_stats->gps_stats = malloc(size_pl);
        strncpy(gps_stats->gps_stats, payload, size_pl);
        ret_val = return_code;
		free(payload);
	}
	/* free(query_buffer); */
	return ret_val;
}

