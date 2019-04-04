#ifndef CONN_
#define CONN_

/*
 * These functions are for accessing the GPS stats.
 **/

typedef struct
{
    unsigned char* connection_status;
} ConnectionStatus;
	
int read_connection_status(ConnectionStatus* connection_status);

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
 *  ConnectionStatus connection_status;
 *  ret_val = read_connection_status(&connection_status);
 *  -- Parse Connection Status info as needed --
 *  free(connection_status.connection_status);
 **/

#endif
