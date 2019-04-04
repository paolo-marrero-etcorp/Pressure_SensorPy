#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "http.h"
#include "base_64.h"
#include <stdio.h>
#include <stdarg.h>
#include "utilities.h"
#include "modbus.h"

#define JSON_FORMAT_MODBUS_WRITE "{\"FRIENDLY_NAME\":\"%s\",\"WRITE_DATA\":\"%s\",\"STATION_ADDRESS\":%i,\"REGISTER_ADDRESS\":%i}"
#define QUERY_FORMAT_MODBUS_READ "?FRIENDLY_NAME=%s&STATION_ADDRESS=%i&REGISTER_ADDRESS=%i&NUM_VALUES=%i"
#define MODBUS_SETTINGS_FORMAT "{\"baud_rate\":%i,\"parity\":\"%c\",\"data_bits\":%i,\"stop_bits\":%i}"
#define READ_DATA_FORMAT_STRING " {\"%[^\n]s\""

static int modbus_write(char* function, char* friendly_name, int station_id, int register_address, const unsigned char *data, int num_val)
{	
	char *b64Data;
	unsigned char* payload;
	int return_code;			
	b64Data = base64_encode((const unsigned char*)data, num_val);

	char* content = get_formated_string(JSON_FORMAT_MODBUS_WRITE, friendly_name, b64Data, station_id, register_address);
	int ret_val = http_put(function, content, &return_code, &payload);
	free(content);
	
	if (ret_val == 0)
	{
		free(payload);
		ret_val = return_code;
	}	
	
	free(b64Data);
	return ret_val;
}

static int modbus_read(char* function, char* friendly_name, int station_id, int register_address, int num_val, unsigned char** register_vals)
{
	unsigned char* payload;	
	int return_code;
	
	*register_vals = 0;
	char* query_buffer = get_formated_string(QUERY_FORMAT_MODBUS_READ, friendly_name, station_id, register_address, num_val);
	
	int ret_val = http_get(function, query_buffer, &return_code, &payload);
	free(query_buffer);
	
	if (ret_val == 0)
	{
		ret_val = return_code;
		if (return_code == 0)
		{
			unsigned char* encoded_buf = payload;
			int length = strlen((const char*)payload);
			// The payload should be a string delimited withy ".
			if(payload[length - 1] == '\"')
			{
				payload[length - 1] = '\0';
			}
			if (payload[0] == '\"')
			{
				encoded_buf = &payload[1];
			}
			base64_decode((char*)encoded_buf, register_vals, strlen((const char*)encoded_buf));				
		}		
		free(payload);
	}

	return (ret_val);
}

static int read_registers(char* function, char* friendly_name, int station_id, int register_address, int num_register, unsigned short* register_vals)
{
	int i;
	unsigned char* register_bytes;
	int ret_val = modbus_read(function, friendly_name, station_id, register_address, num_register, &register_bytes);
	
	if (ret_val == 0)
	{
		unsigned char temp;
		for (i = 0; i < num_register; i++)
		{
			register_vals[i] = ((unsigned short)register_bytes[2*i]) << 8;
			register_vals[i] += (unsigned short)register_bytes[2*i + 1];
		}
		free(register_bytes);
	}
	return ret_val;
}

static int read_bytes(char* function, char* friendly_name, int station_id, int register_address, int num_bytes, unsigned char* register_vals)
{
	int i;
	unsigned char* register_bytes;
	int ret_val = modbus_read(function, friendly_name, station_id, register_address, num_bytes, &register_bytes);
	
	if (ret_val == 0)
	{
		memcpy(register_vals, register_bytes, num_bytes);
		free(register_bytes);
	}
	return ret_val;
}

int read_input(char* friendly_name, int station_id, int register_address, int num_val, unsigned short* register_vals)
{ 
	return read_registers("READ_MODBUS_INPUTS", friendly_name, station_id, register_address, num_val, register_vals) ;
}

int read_holding(char* friendly_name, int station_id, int register_address, int num_register, unsigned short* register_vals)
{	
	return read_registers("READ_MODBUS_HOLDING", friendly_name, station_id, register_address, num_register, register_vals) ;
}

int read_discretes(char* friendly_name, int station_id, int register_address, int num_val, unsigned char* register_vals)
{
	return read_bytes("READ_MODBUS_DISCRETES", friendly_name, station_id, register_address, num_val, register_vals) ;
}

int read_coils(char* friendly_name, int station_id, int register_address, int num_val, unsigned char* register_vals)
{
	return read_bytes("READ_MODBUS_COILS", friendly_name, station_id, register_address, num_val, register_vals) ;
}

int write_holding(char* friendly_name, int station_id, int register_address, const unsigned short *values, int num_val)
{	
	int i = 0;
	int num_bytes = num_val * 2;
	unsigned char* write_buf = malloc(num_bytes);
	
	for (i = 0; i < num_val; i++)
	{
		write_buf[2*i] = (unsigned char)(values[i] >> 8);
		write_buf[(2*i) + 1] = (unsigned char)(0xFF & values[i]);
	}
	int ret_val = modbus_write("WRITE_MODBUS_HOLDING", friendly_name, station_id, register_address, write_buf, num_bytes);
	free(write_buf);
	return ret_val;
}

int write_coil(char* friendly_name, int station_id, int register_address, const unsigned char *values, int num_val)
{
	return modbus_write("WRITE_MODBUS_COILS", friendly_name, station_id, register_address, values, num_val) ;
}

int read_modbus_settings(char* friendly_name, ModbusPortSettings* port_settings)
{
	unsigned char* payload;
	int return_code;
	char* query_buffer = get_formated_string("?FRIENDLY_NAME=%s", friendly_name);
	
	int ret_val = http_get("GET_MODBUS_PORT_SETTINGS", query_buffer, &return_code, &payload);
	free(query_buffer);	
	
	if (ret_val == 0)
	{
		int return_code;
		int num_scanned = scan_json((char*)payload, MODBUS_SETTINGS_FORMAT, &port_settings->baudrate, &port_settings->parity, &port_settings->data_bits, &port_settings->stopbits);
		if (num_scanned == 4)
		{
			ret_val = return_code;
		}
		else
		{
			ret_val = -1;
		}
		free(payload);				
	}
	
	return (ret_val);	
}




