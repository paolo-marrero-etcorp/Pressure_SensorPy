#include "http.h"
#include <stdbool.h>
#include "utilities.h"
#include <stdio.h>
#include <err.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include "analog.h"
#define ADC_PATH "/home/hw/"

#define ANALOG_PROCESS_FORMAT "{ \"min_voltage\" : %f ,\"max_voltage\" : %f ,\"max_sensor\" : %f , \"min_sensor\" : %f }"

int read_analog_process_config(AnalogSettings* analog_settings, char friendly_name[])
{
	unsigned char* payload;
	int return_code;

	char* query_buffer = get_formated_string("?FRIENDLY_NAME=%s", friendly_name);
	int ret_val = http_get("READ_ANALOG_CONFIG", query_buffer, &return_code, &payload);
	
	if (ret_val == 0)
	{
		int num_scanned = scan_json((char*)payload,
			ANALOG_PROCESS_FORMAT, 
			&analog_settings->min_voltage,
			&analog_settings->max_voltage,
			&analog_settings->max_sensor,
			&analog_settings->min_sensor);

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
	free(query_buffer);
	return ret_val;
}

int analog_open(char friendly_name[])
{
	char* full_path = get_full_path(ADC_PATH, friendly_name, "");
	int adc_handle = (int)fopen(full_path, "r");
	if (adc_handle == -1)
	{
		printf("errno = %s", strerror(errno));
	}
	free((void*)full_path);
	return (adc_handle);
}

int read_analog_counts(int handle)
{
	int counts = 0;
	char value[10] = { 0 };
	fseek((FILE*)handle, 0, SEEK_SET);
	fflush((FILE*)handle);
	char* gets_ret = fgets(value, sizeof(value), (FILE*)handle);	
	if (gets_ret == 0)
	{
		return (-1);
	}
	sscanf(value, "%d", &counts);
	return counts;
}

float read_analog_millivolts(int handle)
{
	float RESISTOR_DIVIDER_RATIO = 81.4 / 12.4;
	float V_REF = 1.8;
	float MAX_ADC_COUNTS =  4096;    
	float counts = (float)read_analog_counts(handle);

    if (counts == -1)
		return -1;
    float milli_volts = counts * (V_REF/MAX_ADC_COUNTS) * RESISTOR_DIVIDER_RATIO * 1000.0;
	return milli_volts;
}

void analog_close(int handle)
{
	close(handle);
}
