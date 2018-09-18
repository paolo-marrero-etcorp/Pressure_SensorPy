#pragma once

#include "azure_c_shared_utility/macro_utils.h"

#define HTTP_PORT_NUM 8002
#define API_SERVER_URL "api_server"
#define DEVICE_ID_LENGTH 36

#define PAYLOAD_DELIMITER "\r\n\r\n"
#define INIT_DATA_BITS  "data_bits"
#define INIT_STOP_BITS  "stop_bits"
#define INIT_PARITY  "parity"
#define ADC_CHANNEL "ADC_CHANNEL"


#define ATTR_SERVER_ID_STR       "ep="
#define ATTR_SERVER_ID_LEN       3
#define ATTR_MIN_PERIOD_STR      "pmin="
#define ATTR_MIN_PERIOD_LEN      5
#define ATTR_MAX_PERIOD_STR      "pmax="
#define ATTR_MAX_PERIOD_LEN      5
#define ATTR_GREATER_THAN_STR    "gt="
#define ATTR_GREATER_THAN_LEN    3
#define ATTR_LESS_THAN_STR       "lt="
#define ATTR_LESS_THAN_LEN       3
#define ATTR_STEP_STR            "st="
#define ATTR_STEP_LEN            3
#define ATTR_DIMENSION_STR       "dim="
#define ATTR_DIMENSION_LEN       4

#define API_FUNCTION_VALUES \
        INIT_FUNCTION_FRIENDLY_NAME, \
        FUNCTION_READ_MODBUS_INPUTS, \
        FUNCTION_READ_MODBUS_HOLDING, \
	    FUNCTION_READ_MODBUS_DISCRETES, \
	    FUNCTION_READ_MODBUS_COILS, \
	    FUNCTION_WRITE_MODBUS_COILS, \
	    FUNCTION_WRITE_MODBUS_HOLDING, \
	    FUNCTION_READ_RS232, \
	    FUNCTION_WRITE_RS232, \
	    FUNCTION_INIT_RS232, \
	    FUNCTION_READ_RS485, \
	    FUNCTION_WRITE_RS485, \
	    FUNCTION_INIT_MODBUS, \
	    FUNCTION_INIT_RS485, \
	    FUNCTION_INIT_AI, \
	    FUNCTION_INIT_DI, \
	    FUNCTION_INIT_DO, \
	    FUNCTION_READ_ADC, \
	    FUNCTION_READ_DI, \
	    FUNCTION_WRITE_DO, \
	    FUNCTION_GET_EVENTHUB_SAS_TOKEN_HTTP, \
	    FUNCTION_GET_EVENTHUB_SAS_TOKEN_SB, \
	    FUNCTION_GET_IOTHUB_SAS_TOKEN, \
	    FUNCTION_GET_CONNECTION_STRING, \
	    FUNCTION_GET_EVENTHUB_ENDPOINT
DEFINE_ENUM(API_FUNCTION, API_FUNCTION_VALUES)

#define API_FUNCTION_STR(func)  \
    (INIT_FUNCTION_FRIENDLY_NAME == func ? "FRIENDLY_NAME" : \
	(FUNCTION_INIT_RS232 == func ? "INIT_RS232" : \
	(FUNCTION_INIT_MODBUS == func ? "INIT_MODBUS" : \
	(FUNCTION_INIT_RS485 == func ? "INIT_RS485" : \
	(FUNCTION_INIT_AI == func ? "INIT_AI" : \
	(FUNCTION_INIT_DI == func ? "INIT_DI" : \
	(FUNCTION_INIT_DO == func ? "INIT_DO" : \
	(FUNCTION_READ_MODBUS_INPUTS == func ? "READ_MODBUS_INPUTS" : \
	(FUNCTION_READ_MODBUS_HOLDING == func ? "READ_MODBUS_HOLDING" : \
	(FUNCTION_READ_MODBUS_DISCRETES == func ? "READ_MODBUS_DISCRETES" : \
	(FUNCTION_READ_MODBUS_COILS == func ? "READ_MODBUS_COILS" : \
	(FUNCTION_WRITE_MODBUS_COILS == func ? "WRITE_MODBUS_COILS" : \
	(FUNCTION_WRITE_MODBUS_HOLDING == func ? "WRITE_MODBUS_HOLDING" : \
	(FUNCTION_READ_RS232 == func ? "READ_RS232" : \
	(FUNCTION_WRITE_RS232 == func ? "WRITE_RS232" : \
	(FUNCTION_READ_RS485 == func ? "READ_RS485" : \
	(FUNCTION_WRITE_RS485 == func ? "WRITE_RS485" : \
	(FUNCTION_READ_ADC == func ? "READ_ADC" : \
	(FUNCTION_READ_DI == func ? "READ_DI" : \
	(FUNCTION_WRITE_DO == func ? "WRITE_DO" : \
	(FUNCTION_GET_EVENTHUB_SAS_TOKEN_HTTP == func ? "GET_EVENTHUB_SAS_TOKEN_HTTP" : \
	(FUNCTION_GET_EVENTHUB_SAS_TOKEN_SB == func ? "GET_EVENTHUB_SAS_TOKEN_SB" : \
	(FUNCTION_GET_IOTHUB_SAS_TOKEN == func ? "GET_IOTHUB_SAS_TOKEN" : \
	(FUNCTION_GET_CONNECTION_STRING == func ? "GET_CONNECTION_STRING" : \
	(FUNCTION_GET_EVENTHUB_ENDPOINT == func ? "GET_EVENTHUB_ENDPOINT" : "" )))))))))))))))))))))))))
	
#define BAUDRATE_VALUES \
        BAUD_9600 = 9600, \
        BAUD_14400 = 14400, \
	    BAUD_19200 = 19200, \
	    BAUD_38400 = 38400, \
	    BAUD_57600 = 57600, \
	    BAUD_115200 = 115200
DEFINE_ENUM(BAUDRATE, BAUDRATE_VALUES)

#define PARITY_VALUES \
        NONE, \
        EVEN, \
	    ODD, \
	    MARK, \
	    SPACE
DEFINE_ENUM(PARITY, PARITY_VALUES)

#define PARITY_STR(parity)  \
    (NONE  == parity ? "N" : \
    (EVEN  == parity ? "E" : \
    (ODD   == parity ? "O" : \
	(MARK  == parity ? "M" : \
    (SPACE == parity ? "S" : "N")))))  	

#define DATABITS_VALUES \
        FIVEBITS = 5, \
        SIXBITS = 6, \
	    SEVENBITS = 7, \
	    EIGHTBITS = 8
DEFINE_ENUM(DATABITS, DATABITS_VALUES)	

	
#define STOPBITS_VALUES \
        ONE, \
        ONE_POINT_FIVE, \
	    TWO
DEFINE_ENUM(STOPBITS, STOPBITS_VALUES)

#define STOPBITS_STR(stopbits)  \
    (ONE  == stopbits ? "1" : \
    (ONE_POINT_FIVE  == stopbits ? "1.5" : \
    (TWO   == stopbits ? "2" : "1" )))  	
 
	
#define STRING_EMPTY ""


#define EVENTHUB_SAS_TOKEN_TYPE_VALUES \
        servicebus, \
        http
DEFINE_ENUM(EVENTHUB_SAS_TOKEN_TYPE, EVENTHUB_SAS_TOKEN_TYPE_VALUES)

#ifndef API_CLIENT_H
#define API_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif
	
	int base64_encode(const unsigned char* buffer, size_t length, char** b64text);
	int api_base64_decode(char* b64message, unsigned char** buffer, int length);
	int modbus_write(API_FUNCTION function, int station_id, int register_address, const char *data);
	int modbus_read(API_FUNCTION function, int station_id, int register_address, int num_val, unsigned char** data);
	int read_input(int station_id, int register_address, int num_register, unsigned char** data);
	int read_holding(int station_id, int register_address, int num_register, unsigned char** data);
	int read_discretes(int station_id, int register_address, int num_val, unsigned char** data);
	int read_coils(int station_id, int register_address, int num_val, unsigned char** data);
	int rs232_write(const char* data);
	int rs232_read(unsigned char** data);
	int adc_read();
	int init_adc(const char *friendly_name);
	int init_modbus(const char *friendly_name, BAUDRATE baud_rate);
	int init_rs232(const char *friendly_name, BAUDRATE baud_rate, DATABITS databits, STOPBITS stopbits, PARITY parity);
	int init_rs485(const char *friendly_name, BAUDRATE baud_rate);
	int init_d0(const char *friendly_name);
	int write_holding(int station_id, int register_address, const char *values); 
	int write_coil(int station_id, int register_address, const char *values); 
	int getTelemetryEndPoint(unsigned char ** dest_telemetry_endpoint);
	//int getManagementSASToken(unsigned char ** dest_sas_token, unsigned char ** deviceId);
	int getManagementConnectionString(unsigned char ** connStrP);
	int getTelemetrySASToken(unsigned char ** dest_sas_token, EVENTHUB_SAS_TOKEN_TYPE sas_token_type);

#ifdef __cplusplus
}
#endif

#endif /* API_CLIENT_H */