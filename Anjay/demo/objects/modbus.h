#ifndef MODBUS_
#define MODBUS_

/*
 * Thus structure is used to return the setting for the serial port used by the modbus. It is passed as a parameter to
 * the read_modbus_settings function.
 */
typedef struct
{
	int baudrate;
	int data_bits;
	int stopbits;
	char parity;
} ModbusPortSettings;

/*
 * This will perform a modbus input register read
 *
 * Parameter:
 * - char* friendly_name: The string name associated with the modbus port. Set at build time,
 *   friendly names are what's used to reference the hardware.
 * - int station_id: The id of the modbus slave
 * - int register_address: The base modbus address to read from
 * - int num_val: The number of registers to read, starting from the base address
 * - unsigned short* register_vals: A pointer to a buffer to put the read values in.
 *
 *  Return:
 *  int - 0 on Success
 *      - 15 for Modbus CRC error
 *      - 16 for Modbus timeout while waiting for response
 **/
int read_input(char* friendly_name, int station_id, int register_address, int num_val, unsigned short* register_vals);

/*
 * This will perform a modbus holding register read
 *
 * Parameter:
 * - char* friendly_name: The string name associated with the modbus port. Set at build time,
 *   friendly names are what's used to reference the hardware.
 * - int station_id: The id of the modbus slave
 * - int register_address: The base modbus address to read from
 * - int num_val: The number of registers to read, starting from the base address
 * - unsigned short* register_vals: A pointer to a buffer to put the read values in.
 *
 *  Return:
 *  int - 0 on Success
 *      - 15 for Modbus CRC error
 *      - 16 for Modbus timeout while waiting for response
 **/
int read_holding(char* friendly_name, int station_id, int register_address, int num_register, unsigned short* register_vals);

/*
 * This will perform a modbus discrete register read.
 * Modbus discretes are bit values that are bit packed in the modbus response message, however this function
 * will unpack these values and return each discrete as an entry in a byte array. Each array entry will be
 * either 0 or 1. The size of the return buffer should be equal to the number of discretes you are reading
 *
 * Parameter:
 * - char* friendly_name: The string name associated with the modbus port. Set at build time,
 *   friendly names are what's used to reference the hardware.
 * - int station_id: The id of the modbus slave
 * - int register_address: The base modbus address to read from
 * - int num_val: The number of registers to read, starting from the base address
 * - unsigned char*  register_vals: A pointer to a buffer to put the discrete values in. Each entry in the array
 *   represents one discrete, and will have a value of either 0 or 1.
 *
 *  Return:
 *  int - 0 on Success
 *      - 15 for Modbus CRC error
 *      - 16 for Modbus timeout while waiting for response
 **/
int read_discretes(char* friendly_name, int station_id, int register_address, int num_val, unsigned char* register_vals);

/*
 * This will perform a modbus coil register read.
 * Modbus coils are bit values that are bit packed in the modbus response message, however this function
 * will unpack these values and return each coil as an entry in a byte array. Each array entry will be
 * either 0 or 1. The size of the return buffer should be equal to the number of coils you are reading
 *
 * Parameter:
 * - char* friendly_name: The string name associated with the modbus port. Set at build time,
 *   friendly names are what's used to reference the hardware.
 * - int station_id: The id of the modbus slave
 * - int register_address: The base modbus address to read from
 * - int num_val: The number of registers to read, starting from the base address
 * - unsigned char*  register_vals: A pointer to a buffer to put the coil values in. Each entry in the array
 *   represents one coil, and will have a value of either 0 or 1.
 *
 *  Return:
 *  int - 0 on Success
 *      - 15 for Modbus CRC error
 *      - 16 for Modbus timeout while waiting for response
 **/
int read_coils(char* friendly_name, int station_id, int register_address, int num_val, unsigned char* register_vals);

/*
 * This will perform a modbus holding register write
 *
 * Parameter:
 * - char* friendly_name: The string name associated with the modbus port. Set at build time,
 *   friendly names are what's used to reference the hardware.
 * - int station_id: The id of the modbus slave
 * - int register_address: The base modbus address to write to
 * - const unsigned short *values: A buffer of holding register values to write. Each entry represents a register
 *   and will be written starting from the base modbus address given by the register_address parameter.
 * - int num_val: The number of entries in the write buffer
 *
 *  Return:
 *  int - 0 on Success
 *      - 15 for Modbus CRC error
 *      - 16 for Modbus timeout while waiting for response
 **/
int write_holding(char* friendly_name, int station_id, int register_address, const unsigned short *values, int num_val);

/*
 * This will perform a modbus coil register write
 *
 * Parameter:
 * - char* friendly_name: The string name associated with the modbus port. Set at build time,
 *   friendly names are what's used to reference the hardware.
 * - int station_id: The id of the modbus slave
 * - int register_address: The base modbus address to write to
 * - const unsigned char *values: A buffer of coil register values to write. Each entry in the buffer represents
 *   a coil, and should have a value of either 0 or 1.
 * - int num_val: The number of entries in the write buffer
 *
 *  Return:
 *  int - 0 on Success
 *      - 15 for Modbus CRC error
 *      - 16 for Modbus timeout while waiting for response
 **/
int write_coil(char* friendly_name, int station_id, int register_address, const unsigned char *values, int num_val);

/*
 * Gets the serial port setting of the modbus port with the given friendly name
 *
 * Parameters:
 * - char* friendly_name: The string name associated with the modbus port. Set at build time,
 *   friendly names are what's used to reference the hardware.
 * - ModbusPortSettings* port_settings: A pointer to the structure to output the results to.
 *
 * Return:
 * int - 0 on success. The values in port_settings are not valid for any other return value.
 *
 */
int read_modbus_settings(char* friendly_name, ModbusPortSettings* port_settings);

#endif
