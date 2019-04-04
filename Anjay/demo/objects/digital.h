#ifndef DIGITAL_
#define DIGITAL_


/*
 * A function pointer type for the edge detection callback function passed to the enable_edge_detect function
 */
typedef void(*EdgeCallback)(void);

/*
 * An enum type to define the kinds of detection that can be performed.
 */
typedef enum
{
	RISING,
	FALLING,
	BOTH
} EdgeDetectEnum;

/*
 * This function will open a handle to the digital input with the specified friendly name.
 *
 * Parameter:
 * -friendly_name: The string name associated with the Digital Input. Set at build time, friendly names are what's used to
 *  reference the hardware.
 *
 *  Return:
 *  int - A handle to the digital input. -1 will be returned if the operation failed.
 **/
int digital_in_open(char friendly_name[]);

/*
 * Reads the the voltage polarity on the digital input
 *
 * Parameter:
 * -handle: The handle to the digital input given by the digital_in_open function.
 *
 * Return:
 * int - The polarity as an integer 0 or 1
 **/
int digital_in_read(int handle);

/*
 * Disables edge detection on a digital input
 *
 * Parameter:
 * -int handle: The handle to the edge detection pipe. The handle was the value returned from the enable_edge_detect function.
 */

void disabled_edge_detect(int pipe_handle);

/*
 * Enables edge detection a the digital input
 *
 * Parameter:
 * -EdgeDetectEnum edge: The type of edge detection to perform
 * -EdgeCallback callback: Callback function that will be called when the edge is detected
 * -char* name: The friendly name assigned to the digital input
 *
 * Return:
 * int - A handle to a pipe needed by the disabled_edge_detect function to stop edge detection.
 **/
int enable_edge_detect(EdgeDetectEnum edge, EdgeCallback callback, char* name);

/*
 * Closes the handle to the digital input
 *
 * Parameter:
 * - int handle: The handle returned from the digital_in_open function
 */
void digital_in_close(int handle);

/*
 * Puts the digital output with the given friendly name into a failsafe state. The failsafe state is set by the installer
 * using the hardware setup GUI. The installer will set the failsafe state to the appropriate high or low state for given
 * hardware setup on site.
 *
 * Parameter:
 * - char friendly_name[]: The string name associated with the Digital output. Set at build time, friendly names are what's used
 *  to reference the hardware.
 */
int enter_failsafe_state(char* friendly_name);

/*
 * Opens a handle to the digital output with the specified friendly name.
 *
 * Parameter:
 * - char friendly_name[]: The string name associated with the Digital output. Set at build time, friendly names are what's used
 *  to reference the hardware.
 *
 *  Return:
*   int - A handle to the digital output. -1 will be returned if the operation failed.
 */
int digital_out_open(char friendly_name[]);

/*
 * Sets the digital output high
 *
 * Parameter:
 * - int handle: The handle to the digital output, given by the digital_out_open function
 *
 * Return:
 * int - 0 on Success
 */
int digital_out_set(int handle);

/*
 * Clears the digital output low
 *
 * Parameter:
 * - int handle: The handle to the digital output, given by the digital_out_open function
 *
 * Return:
 * int - 0 on Success
 */
int digital_out_clear(int handle);

/*
 * Closes the handle to the digital output
 *
 * Parameter:
 * - int handle: The handle to the digital output, given by the digital_out_open function
 *
 */
void digital_out_close(int handle);

/*
 * This function will open a handle to the sensor power output with the specified friendly name.
 *
 * Parameter:
 * -friendly_name: The string name associated with the sensor power output. Set at build time, friendly names are what's used
 *  to reference the hardware.
 *  Return:
 *   int - A handle to the sensor power output. -1 will be returned if the operation failed.*
 */
int sen_power_open(char friendly_name[]);

/*
 * Sets the sensor power high
 *
 * Parameter:
 * - int handle: The handle to the sensor power output, given by the sen_power_open function
 *
 * Return:
 * int - 0 on Success
 */
int sen_power_set(int handle);

/*
 * Sets the sensor power low
 *
 * Parameter:
 * - int handle: The handle to the sensor power output, given by the sen_power_open function
 *
 * Return:
 * int - 0 on Success
 */
int sen_power_clear(int handle);

/*
 * Closes the handle to the sensor power output
 *
 * Parameter:
 * - int handle: The handle to the sensor power output, given by the sen_power_open function
 *
 */
void sen_power_close(int handle);

#endif
