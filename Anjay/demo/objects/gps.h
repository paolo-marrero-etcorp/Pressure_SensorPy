#ifndef GPS_
#define GPS_

/*
 * These functions are for accessing the GPS stats.
 **/

typedef struct
{
    unsigned char* gps_stats;
} GpsStats;
	
int read_gps_stats(GpsStats* gps_stats);

/*
 * This function will populate the GpsStats structure with the latest stats.
 * Memory will be allocated. Caller should free gps_status after use.
 * 
 * Parameter:
 *   
 *	Return:
 *  
 *  Code sample:
 *  int ret_val;
 *  GpsStats gps_stats;
 *  ret_val = read_gps_stats(&gps_stats);
 *  -- Parse GPS info as needed -- 
 *  free(gps_stats.gps_stats);
 **/

#endif
