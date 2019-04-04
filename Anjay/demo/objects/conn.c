#include "http.h"
#include <stdbool.h>
#include <stdio.h>
#include <err.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include "conn.h"

int read_connection_status(ConnectionStatus* connection_status)
{
	unsigned char* payload;
	int return_code;
    int size_pl;
	int ret_val = http_get("READ_CONNECTION_STATUS", "", &return_code, &payload);
	
	if (ret_val == 0)
	{
        size_pl = strlen(payload);
        connection_status->connection_status = malloc(size_pl);
        strncpy(connection_status->connection_status, payload, size_pl);
        ret_val = return_code;
		free(payload);
	}
	/* free(query_buffer); */
	return ret_val;
}

