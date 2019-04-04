#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>

#include <unistd.h>
#include <linux/serial.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <err.h>
#include "utilities.h"
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "analog.h"
#include "digital.h"
#include "modbus.h"
#include "serial_port.h"
#include <stdbool.h>
#include <time.h>
#include "api_client.h"
#include "lwm2m.h"

#define MAX_BUFFER_SIZE (3)

// Digital test parameters
#define TEST_LWM2M ("lwm2m\n")
#define TEST_DIGITAL ("digital\n")
#define DO_NAME ("do")
#define DI1_NAME ("di1")
#define DI2_NAME ("di2")

#define DIGITAL_HIGH (1)
#define DIGITAL_LOW (0)

#define TEST_ANALOG ("analog\n")
#define SENPWR_NAME ("senpwr")
#define AI1_NAME ("ai1")
#define AI2_NAME ("ai2")
#define AI_HIGH_COUNTS_TEST_VAL (1950)
#define AI_HIGH_MILLIVOLTS_TEST_VAL (4500)

#define AI_LOW_TEST_VAL (100)

#define TEST_MODBUS_COM1 ("modbus1\n")

#define MODBUS_COM1_NAME ("alien2")
#define MODBUS_COM2_NAME ("modbus2")

#define TEST_ALL ("all\n")

#define GET_SB_EVENT_HUB ("get_event\n")

// test results
#define PASS ("PASS")
#define FAIL ("FAIL")

void print_usage(void)
{
	printf("Commands available:\n\n");
	printf("\t*%s\t\tPerforms the digital loop back test. DO must be connected to DI1, and DI2.\n", TEST_DIGITAL);
	printf("\t*%s\t\tPerforms the analog loop back test. SENPWR must be connected to AI1, and AI2.\n", TEST_ANALOG);
	printf("\t*all\n\t\tPerforms all tests.\n");
	printf("\tq to quit.\n\n");
}

bool test_digital_loopback(void)
{
	bool do_high_result = true;
	bool do_low_result = true;
	int do_handle = digital_out_open(DO_NAME);
	int di1_handle = digital_in_open(DI1_NAME);
	int di2_handle = digital_in_open(DI2_NAME);
	
	if ((do_handle && di1_handle && di2_handle) == false)
	{
		printf("*Digital Loopback Result : %s. \n\tInit failed, please confirm you have the right names for your io. Names should be: %s, %s, and %s. "
			"\n\tHandles returned from open functions were : do handle = %i, di1 handle = %i, di2_handle = %i\n",
			FAIL, 
			DO_NAME,
			DI1_NAME, 
			DI2_NAME, 
			do_handle,
			di1_handle,
			di2_handle);
		if (di1_handle)
			digital_in_close(di1_handle);
		if (di2_handle)
			digital_in_close(di2_handle);
		if (do_handle)
			digital_out_close(do_handle);
		return false;
	}
	
	// do high test
	digital_out_set(do_handle);
	int di1_val = digital_in_read(di1_handle);
	int di2_val = digital_in_read(di2_handle);
	if (di1_val != DIGITAL_HIGH || di2_val != DIGITAL_HIGH)
	{
		do_high_result = false;		
	}
	printf("\tdo high test %s : ", do_high_result ? PASS : FAIL);
	printf("do set to high. di1 = %i, do2 = %i\n", di1_val, di2_val);

	// do low test
	digital_out_clear(do_handle);
	di1_val = digital_in_read(di1_handle);
	di2_val = digital_in_read(di2_handle);
	if (di1_val != DIGITAL_LOW || di2_val != DIGITAL_LOW)
	{
		do_low_result = false;		
	}
	printf("\tdo high test %s : ", do_low_result ? PASS : FAIL);
	printf("do set to low. di1 = %i, do2 = %i\n", di1_val, di2_val);
	
	printf("\t*Digital Loopback Result : %s\n\n", (do_low_result && do_high_result) ? PASS : FAIL);
	
	digital_in_close(di1_handle);
	digital_in_close(di2_handle);
	digital_out_close(do_handle);	
	
	return do_low_result && do_high_result;
}

bool test_analog_loopback(void)
{
	bool senpwr_high_result = true;
	bool senpwr_low_result = true;
	
	int senpwr_handle = sen_power_open(SENPWR_NAME);
	int ai1_handle = analog_open(AI1_NAME);
	int ai2_handle = analog_open(AI2_NAME);
	
	if ((senpwr_handle && ai1_handle && ai2_handle) == false)
	{
		printf("*Digital Loopback Result : %s. \n\tInit failed, please confirm you have the right names for your io. Names should be: %s, %s, and %s. "
			"\n\tHandles returned from open functions were : Sensor Power handle = %i, ai1 handle = %i, ai2_handle = %i\n",
			FAIL, 
			SENPWR_NAME,
			AI1_NAME, 
			AI2_NAME, 
			senpwr_handle,
			ai1_handle,
			ai2_handle);
		if (senpwr_handle)
			sen_power_close(senpwr_handle);
		if (ai1_handle)
			analog_close(ai1_handle);
		if (ai2_handle)
			analog_close(ai2_handle);
		return false;
	}
	
	
	// sensor power high test
	sen_power_set(senpwr_handle);
	// AIs always need a little time to settle
	sleep(1);
	int ai1_counts = read_analog_counts(ai1_handle);
	int ai2_counts = read_analog_counts(ai2_handle);
	int ai1_millivolts = read_analog_millivolts(ai1_handle);
	int ai2_millivolts = read_analog_millivolts(ai2_handle);	
	if (ai1_counts < AI_HIGH_COUNTS_TEST_VAL || 
		ai2_counts < AI_HIGH_COUNTS_TEST_VAL || 
		ai1_millivolts < AI_HIGH_MILLIVOLTS_TEST_VAL || 
		ai2_millivolts < AI_HIGH_MILLIVOLTS_TEST_VAL)
	{
		senpwr_high_result = false;		
	}
	printf("\n\tSensor power high test %s : \n", senpwr_high_result ? PASS : FAIL);
	printf("\t\tSensor power set to high. ai1 counts = %i, ai2 counts = %i. ai1 milli-volts = %i, ai2 milli-volts = %i.\n"
		   "\t\tValues are expected to be above %i counts and %i milli-volts.",
		ai1_counts,
		ai2_counts,
		ai1_millivolts,
		ai2_millivolts,
		AI_HIGH_COUNTS_TEST_VAL,
		AI_HIGH_MILLIVOLTS_TEST_VAL);	
	
	// sensor power low test
	sen_power_clear(senpwr_handle);
	// AIs always need a little time to settle
	sleep(1);
	ai1_counts = read_analog_counts(ai1_handle);
	ai2_counts = read_analog_counts(ai2_handle);
	ai1_millivolts = read_analog_millivolts(ai1_handle);
	ai2_millivolts = read_analog_millivolts(ai2_handle);	
	if (ai1_counts > AI_LOW_TEST_VAL || 
		ai2_counts > AI_LOW_TEST_VAL || 
		ai1_millivolts > AI_LOW_TEST_VAL || 
		ai2_millivolts > AI_LOW_TEST_VAL)
	{
		senpwr_low_result = false;		
	}
	printf("\n\tSensor power low test %s : \n", senpwr_high_result ? PASS : FAIL);
	printf("\t\tSensor power set to low. ai1 counts = %i, ai2 counts = %i. ai1 milli-volts = %i, ai2 milli-volts = %i.\n"
		   "\t\tValues are expected to be below %i counts and %i milli-volts.",
		ai1_counts,
		ai2_counts,
		ai1_millivolts,
		ai2_millivolts,
		AI_LOW_TEST_VAL,
		AI_LOW_TEST_VAL);		
	
	printf("\n\t*Analog Loopback Result : %s\n\n", (senpwr_high_result && senpwr_low_result) ? PASS : FAIL);
	sen_power_close(senpwr_handle);
	analog_close(ai1_handle);
	analog_close(ai2_handle);	
	return senpwr_high_result && senpwr_low_result;
}


#define HOLDING_BUFFER_LENGTH (50)
#define MAX_HOLDING_VAL (65535)
#define NUM_MODBUS_TEST_RUNS (50)

bool modbus_test(char* name)
{
	int i;
	int test_num;
	int buf_length;
	int address = 0;
	bool holding_test = false;
	
	static unsigned short holding_write_buffer[HOLDING_BUFFER_LENGTH];
	static unsigned short holding_read_buffer[HOLDING_BUFFER_LENGTH];
	
	srand(time(NULL));
	for (test_num = 0; test_num < NUM_MODBUS_TEST_RUNS; test_num++)
	{
		buf_length = rand() % HOLDING_BUFFER_LENGTH;
		if (buf_length == 0)
			buf_length++;
		
		for (i = 0; i < buf_length; i++)
		{
			holding_write_buffer[i] = rand() % MAX_HOLDING_VAL;
		}
		address = rand() % 1000;
		int ret_val = write_holding(name, 1, address, holding_write_buffer, buf_length);
		if (ret_val != 0)
		{
			printf("write failed. ret val = %i\n", ret_val);
		}

		ret_val = read_holding(name, 1, address, buf_length, holding_read_buffer);
		if (ret_val  != 0)
		{
			printf("read failed. ret val = %i\n", ret_val);
		}
	
		if (memcmp(holding_write_buffer, holding_read_buffer, buf_length) != 0)
		{
			printf("FAIL!!!!!!!!!!!!!!!!!\n");
		}
	}
	printf("Done modbus test\n");
	return false;
}

int main(int argc, char *argv[])
{
	int result;
	ApiInit();
	size_t length = MAX_BUFFER_SIZE;
	char* test_cmd = malloc(MAX_BUFFER_SIZE);
	print_usage();
	while (true)
	{
		ssize_t num_char = getline(&test_cmd, &length, stdin);
		
		if (num_char != -1)
		{
			if (strcmp(test_cmd, TEST_DIGITAL) == 0)
			{
				test_digital_loopback();
			}
			else if (strcmp(test_cmd, TEST_ANALOG) == 0)
			{
				test_analog_loopback();
			}			
			else if (strcmp(test_cmd, "q\n") == 0)
			{
				printf("Quitting...");
				break;
			}
			else if (strcmp(test_cmd, TEST_MODBUS_COM1) == 0)
			{
				modbus_test(MODBUS_COM1_NAME);
			}		
			else if(strcmp(test_cmd, TEST_LWM2M) == 0)
			{
				static Lwm2mConnectionInfoStruct InfoStruct;
				int ret_val = get_lwm2m_connection_data(&InfoStruct);
				printf("Lwm2m connection test return value = %i\n", ret_val);
			}
			else if(strcmp(test_cmd, TEST_ALL) == 0)
			{
				test_digital_loopback();
				test_analog_loopback();
			}
			else
			{
				printf("Command not recognized.\n\n");
				print_usage();
			}
		}
	}
	return 1;

}