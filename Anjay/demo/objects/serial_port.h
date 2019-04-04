#ifndef  SERIAL_PORT_
#define SERIAL_PORT_

typedef enum
{
	PARITY_NONE,
	PARITY_ODD,
	PARITY_EVEN
} ParityEnum;

typedef enum
{
	STOP_1BIT,
	STOP_2BIT
} StopBitsEnum;

typedef enum
{
	BAUD_4800,
	BAUD_9600,
	BAUD_19200,
	BAUD_38400,
	BAUD_57600,
	BAUD_115200
} BaudRateEnum;

typedef struct
{
	int baudrate;
	int data_bits;
	int stopbits;
	char parity;
} SerialPortSettings;

/*
 * Retrieves the port settings for the serial port with the given friendly name
 *
 * Parameters:
 * - char* friendly_name: The string name associated with the modbus port. Set at build time,
 *   friendly names are what's used to reference the hardware
 * - SerialPortSettings* port_settings: A pointer to the structure that the settings will be
 *   written to.
 *
 * Return:
 * int - 0 if the port settings were successfully retrieved. The values in port_settings are not valid
 * if a non-zero value is returned.
 */
int get_port_settings(char* friendly_name, SerialPortSettings* port_settings);

/*
 * Opens a handle to a 485 serial port. Once the handle has been acquired, the port can be used using
 * the standard functions available in stdio.h such as fputs, fwrite, fread, etc.
 *
 * Parameters:
 * - char* friendly_name: The string name associated with the modbus port. Set at build time,
 *   friendly names are what's used to reference the hardware.
 * - BaudRateEnum baud_rate: The baud rate
 * - ParityEnum parity: Type of parity to use
 * - StopBitsEnum stop_bits: The number of stop bits per packet
 * - int data_bits: Number of data bits in a character. Options are 5,6,7,8, or 9. 8 is the typical value used.
 *
 * Return:
 * int - A handle to the serial port or -1 if the open failed.
 */
int serial485_open(char friendly_name[], 
	BaudRateEnum baud_rate, 
	ParityEnum parity, 
	StopBitsEnum stop_bits, 
	int data_bits);


#endif
