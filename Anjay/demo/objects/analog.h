#ifndef ANALOG_
#define ANALOG_

/*
 * These functions are for accessing the Analog Inputs on the Morpheus, and reading the analog
 * installer settings.
 **/

typedef struct
{
	float min_voltage;
	float max_voltage;
	float max_sensor;
	float min_sensor;
} AnalogSettings;
	
typedef struct
{
	AnalogSettings AI1;
	AnalogSettings AI2;
} AnalogProcessConfig;

int read_analog_process_config(AnalogSettings* analog_settings, char friendly_name[]);

/*
 * This function will open a handle to the analog input with the specified friendly name.
 * 
 * Parameter:
 *  friendly_name: The string name associated with the Alalog Input. Set at build time,
 *                 friendly names are what's used to reference the hardware.
 *   
 *	Return:
 *	 A handle to the analog input. -1 will be returned if the operation failed.
 **/
int analog_open(char friendly_name[]);

/*
 * This function will read the raw ADC (analog to digital converter) counts for the 
 * analog input. The ADC is a 12-bit converter which will give a range of 0 to 4096
 * counts.
 * 
 * Parameter:
 *  handle: The handle to the analog input given by the analog_open function.
 *  
 * Return:
 *  The raw count of the ADC, ranging from 0 to 4096 or -1 if an error occurred.
 **/
int read_analog_counts(int handle);

/*
 * Reads the the voltage in millivolts on the analog input pin. The analog input voltage
 * range is 0 to 11.2 Volts
 * 
 * Parameter:
 *  handle: The handle to the analog input given by the analog_open function.
 *  
 * Return:
 *  The voltage in millivolts on the analog input or -1 if an error occurred.
 **/
float read_analog_millivolts(int handle);

/*
 * Closes the handle to the analog input resource given by the analog_open function.
 **/
void analog_close(int handle);

#endif
