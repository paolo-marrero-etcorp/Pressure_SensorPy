#include "http.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "utilities.h"
#include "lwm2m.h"

bool get_json_dict_string_key(const char* string, char* key, char* value);

int get_lwm2m_connection_data(Lwm2mConnectionInfoStruct* InfoStruct)
{
	unsigned char* payload;
	int return_code;
	
	InfoStruct->host_name[0] = 0;
	InfoStruct->identity[0] = 0;
	InfoStruct->secret_key[0] = 0;
	InfoStruct->endpoint[0] = 0;
	
	int ret_val = http_get("GET_LWM2M_SECURITY_INFO", "", &return_code, &payload);

	if (ret_val == 0)
	{
		bool success = false;
		if (get_json_dict_string_key((char*)payload, "LWM2M_IDENTITY", InfoStruct->identity))
			if (get_json_dict_string_key((char*)payload, "LWM2M_HOST_NAME", InfoStruct->host_name))
				if (get_json_dict_string_key((char*)payload, "LWM2M_SECRET_KEY", InfoStruct->secret_key))
					if (get_json_dict_string_key((char*)payload, "LWM2M_ENDPOINT", InfoStruct->endpoint))
						success = true;
					
		free(payload);
		if (success)
			ret_val = return_code;
		else
			ret_val = -1;
	}
	return ret_val;	
}

