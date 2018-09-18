#include <stdio.h> 
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <openssl/bio.h>
#include <openssl/evp.h>
#include <openssl/buffer.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "ApiClient.h"
#include "Utilities.h"

#define MAX_RETRY 20
#define API_RESULT "\"RESULT\": "
#define API_PAYLOAD "\"PAYLOAD\": \""

#define JSON_FORMAT_MODBUS_WRITE "{\"WRITE_DATA\":\"%s\",\"STATION_ADDRESS\":%i,\"REGISTER_ADDRESS\":%i}"
#define JSON_FORMAT_RS232_WRITE "{\"FRIENDLY_NAME\":\"%s\", \"WRITE_DATA\":\"%s\"}"
#define JSON_FORMAT_INIT "{\"FRIENDLY_NAME\":\"%s\",\"baud_rate\":%i}"
#define JSON_FORMAT_INIT_AI "{\"FRIENDLY_NAME\":\"%s\"}"
#define JSON_FORMAT_INIT_RS232 "{\"FRIENDLY_NAME\":\"%s\",\"baud_rate\":%i,\"data_bits\":%i,\"stop_bits\":%s,\"parity\":\"%s\"}"
#define QUERY_FORMAT_MODBUS_READ "?FRIENDLY_NAME=%s&STATION_ADDRESS=%i&REGISTER_ADDRESS=%i&NUM_VALUES=%i"
#define QUERY_FORMAT_RS232_READ "?FRIENDLY_NAME=%s"
#define QUERY_FORMAT_ADC_READ "?FRIENDLY_NAME=%s"
#define HEADER_FORMAT_WRITE "User-Agent: etc/1.0\r\nContent-Type: application/json\r\nETC_FUNCTION: %s"
#define POST_FORMAT "POST %s HTTP/1.1\r\nHost: http://%s\r\n%s\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n%s"
#define GET_FORMAT "GET %s%s HTTP/1.1\r\nHost: http://%s\r\n%s\r\n\r\n"

#define	IOT_CONN_FORMAT "HostName=%s;DeviceId=%s;SharedAccessSignature=%s"

const char* _friendly_name;
 
int base64_encode(const unsigned char* buffer, size_t length, char** b64text) {
	//Encodes a binary safe base 64 string
   BIO *bio, *b64;
	BUF_MEM *bufferPtr;

	b64 = BIO_new(BIO_f_base64());
	bio = BIO_new(BIO_s_mem());
	bio = BIO_push(b64, bio);

	BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);   //Ignore newlines - write everything in one line
	BIO_write(bio, buffer, length);
	BIO_flush(bio);
	BIO_get_mem_ptr(bio, &bufferPtr);
	BIO_set_close(bio, BIO_NOCLOSE);
	BIO_free_all(bio);

	*b64text = (*bufferPtr).data;

	return (0); //success
}


size_t calcDecodeLength(const char* b64input) {
	//Calculates the length of a decoded string
   size_t len = strlen(b64input),
   	padding = 0;

	if (b64input[len - 1] == '=' && b64input[len - 2] == '=') //last two chars are =
		padding = 2;
	else if (b64input[len - 1] == '=') //last char is =
		padding = 1;

	return (len * 3) / 4 - padding;
}

int api_base64_decode(char* b64message, unsigned char** buffer, int length) {
	BIO *bio, *b64;
	int decodeLen = calcDecodeLength(b64message);
	*buffer = (unsigned char*)malloc(decodeLen + 1);
	(*buffer)[decodeLen] = '\0';

	bio = BIO_new_mem_buf(b64message, -1);
	b64 = BIO_new(BIO_f_base64());
	bio = BIO_push(b64, bio);

	BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);   //Do not use newlines to flush buffer
	BIO_read(bio, *buffer, length);	
	BIO_free_all(bio);
	(*buffer)[decodeLen] = '\0';
	return (0); //success
}


static int http_request_once(unsigned char *message, unsigned char *dest, int len_dest)
{ 
	int retry = 0;
	struct hostent *server;
	struct sockaddr_in serv_addr;
	int sockfd, bytes, sent, received, total;
	
	int message_size = strlen(message);
	 
	(void)printf("Request:\n%s\n", message) ;

	/* create the socket */
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) 
	{	
		(void)printf("ERROR opening socket\n");
		return -1;
	}

	/* lookup the ip address */
	server = gethostbyname(API_SERVER_URL);
	if (server == NULL) 
	{	
		(void)printf("ERROR, no such host\n");
		return -1;
	}

	/* fill in the structure */
	memset(&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(HTTP_PORT_NUM);
	memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);

	/* connect the socket */
	while(connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{	
		(void)printf("ERROR connecting\n");
		
		retry++;
		if(retry >= MAX_RETRY)		
			return -1;
	}

	/* send the request */
	total = strlen(message);
	sent = 0;
	do {
		bytes = write(sockfd, message + sent, total - sent);
		if (bytes < 0)
			(void)printf("ERROR writing message to socket\n");
		if (bytes == 0)
			break;
		sent += bytes;
	} while (sent < total);

	/* receive the response */
	memset(dest, 0, len_dest);
	total = len_dest - 1;
	received = 0;
	do {
		bytes = read(sockfd, dest + received, total - received);
		if (bytes < 0)
			(void)printf("ERROR reading response from socket\n");
		if (bytes == 0)
			break;
		received += bytes;
	} while (received < total);

	 

	/* close the socket */
	close(sockfd);

	/* process response */
	(void)printf("Response:\n%s\n", dest);
	
	len_dest = received;
	
	if (received == 0)
		return -1;
	
	return 0;
}


static int http_request(unsigned char *message, unsigned char *dest, int len_dest)
{
	
	int retry = 0 ;
	
	while (retry < 20 && http_request_once(message, dest, len_dest) != 0)
	{
		retry++;
	}
	
	if (dest == NULL)
	{
		return -1 ;	
	}
	
	return 0;
}

static int get_response(unsigned char* response, int *return_code, unsigned char** payload, bool decode)
{
	if (response == NULL)
		return -1;
	
	int resp_len = strlen(response);	
	char *result_start = strstr(response, API_RESULT) + strlen(API_RESULT );
	if (result_start == NULL)
		return -1;
	
	sscanf(result_start, "%d", return_code);
	
	char *payload_start = strstr(response, API_PAYLOAD) + strlen(API_PAYLOAD);
	
	if (payload_start == NULL)
		return resp_len;
	
	char *payload_end = strstr(payload_start, "\"");
	if (payload_end == NULL)
		return resp_len;
	
	int payload_length = payload_end - payload_start;

	if (decode)
        return api_base64_decode(payload_start, payload, payload_length);
	
	if (payload != NULL)
	{	
		*payload = calloc(payload_length + 1, sizeof(char));
		memcpy(*payload, payload_start, payload_length);		
	}
	return resp_len;
}


static int get_response_payload(unsigned char* response, unsigned char** payload)
{
	if (response == NULL)
		return -1;
	
	int resp_len = strlen(response);	
	char *payload_start = memchr(response, '{', resp_len);	
	payload_start = memchr(payload_start, ':', resp_len);
	payload_start = memchr(payload_start, '"', resp_len) + 1;
	
	char *payload_end = memchr(payload_start, '"', resp_len);
	int payload_size = payload_end - payload_start;
    return api_base64_decode(payload_start, payload, payload_size);
}

int modbus_write(API_FUNCTION function, int station_id, int register_address, const char *data)
{	
	int result = -1;
	
	char *headers = malloc(10 + strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(function)));
	sprintf(headers, HEADER_FORMAT_WRITE, API_FUNCTION_STR(function));
	
	char *b64Data;
		
	base64_encode(data, sizeof(data), &b64Data);
	
	char *content = malloc(10 + strlen(JSON_FORMAT_MODBUS_WRITE) + strlen(b64Data));
	sprintf(content, JSON_FORMAT_MODBUS_WRITE, b64Data, station_id, register_address);	
	
	char *message = malloc(10 + strlen(POST_FORMAT) + strlen(API_SERVER_URL) + strlen(content) + strlen(headers));
	
	sprintf(message,
		POST_FORMAT,
		"/",
		API_SERVER_URL,
		headers,
		strlen(content),
		content);
	
	int sizeResponse = 1024;
	
	char *response =  calloc(sizeResponse, sizeof(char));
	
	result = http_request(message, response, sizeResponse);
	
	free(b64Data);
	free(message);
	free(headers);
	free(response);
	
	return result;
}

int modbus_read(API_FUNCTION function, int station_id, int register_address, int num_val, unsigned char** data)
{
	
	char *headers = malloc(10 + strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(function)));
	sprintf(headers, HEADER_FORMAT_WRITE, API_FUNCTION_STR(function));
	
	char *query = malloc(10 + strlen(QUERY_FORMAT_MODBUS_READ) + strlen(API_SERVER_URL) + strlen(_friendly_name));
	sprintf(query, QUERY_FORMAT_MODBUS_READ, _friendly_name, station_id, register_address, num_val);	
	 	
	char *buffer = malloc(1024); 
	
	sprintf(buffer,
		GET_FORMAT,
		"/",
		query,
		API_SERVER_URL,
		headers);
	
	free(query);
	free(headers);
	
	char *response = malloc(1024); 
	memset(response, 0, 1024);
	
	if (http_request(buffer, response, 1024) != 0) 
	{	
		free(response);
		free(buffer);
		return -1;
	}
	
	
	int result;
	
	get_response(response, &result, data, true);
	
	free(response); 
	
	if (result != 0)
	{
		return -1;
	}
	
	return 0;
	 
}

int read_input(int station_id, int register_address, int num_val,  unsigned char** data)
{ 
	return modbus_read(FUNCTION_READ_MODBUS_INPUTS, station_id, register_address, num_val, data) ;
}

int read_holding(int station_id, int register_address, int num_register, unsigned char** data)
{	
	return modbus_read(FUNCTION_READ_MODBUS_HOLDING, station_id, register_address, num_register, data) ;
}

int read_discretes(int station_id, int register_address, int num_val, unsigned char** data)
{
	return modbus_read(FUNCTION_READ_MODBUS_DISCRETES, station_id, register_address, num_val, data) ;
}

int read_coils(int station_id, int register_address, int num_val, unsigned char** data)
{
	return modbus_read(FUNCTION_READ_MODBUS_COILS, station_id, register_address, num_val, data) ;
}

int init_mode(const char *friendly_name, BAUDRATE baud_rate, API_FUNCTION function)
{
	_friendly_name = friendly_name;
	
	int result = -1;
	
	char *headers = malloc(10 + strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(function)));
	sprintf(headers, HEADER_FORMAT_WRITE, API_FUNCTION_STR(function));
	 
	char *content = malloc(10 + strlen(JSON_FORMAT_INIT) + strlen(_friendly_name));
	
	sprintf(content, JSON_FORMAT_INIT, _friendly_name, baud_rate);	
	
	char *message = malloc(10 + strlen(POST_FORMAT) + strlen(API_SERVER_URL) + strlen(content) + strlen(headers));
	
	sprintf(message,
		POST_FORMAT,
		"/",
		API_SERVER_URL,
		headers,
		strlen(content),
		content);
	
	int sizeResponse = 1024;
	
	char *response =  calloc(sizeResponse, sizeof(char));
	
	http_request(message, response, sizeResponse);
	
	if (response != NULL && strstr(response, "HTTP/1.0 200") != NULL)
	{
		result = 0;	
		char *payload;	
		get_response(response, &result, &payload, false);		
		free(payload);
	}
	else 
	{
		result = -1;
	}
	
	free(message);
	free(headers);
	free(response);
	
	return result;
}

int write_holding(int station_id, int register_address, const char *values)
{
	return modbus_write(FUNCTION_WRITE_MODBUS_HOLDING, station_id, register_address, values) ;
}

int write_coil(int station_id, int register_address, const char *values)
{
	return modbus_write(FUNCTION_WRITE_MODBUS_COILS, station_id, register_address, values) ;
}

int rs232_write(const char *data)
{	
	int result = -1;
	
	char *headers = malloc(10 + strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(FUNCTION_WRITE_RS232))) ;
	sprintf(headers,
		HEADER_FORMAT_WRITE,
		API_FUNCTION_STR(FUNCTION_WRITE_RS232));
	
	char *b64Data;
		
	base64_encode(data, strlen(data), &b64Data);
	
	char *content = malloc(10 + strlen(JSON_FORMAT_RS232_WRITE) + strlen(b64Data));
	sprintf(content, JSON_FORMAT_RS232_WRITE, _friendly_name, b64Data);	
	
	char *message = malloc(10 + strlen(POST_FORMAT) + strlen(API_SERVER_URL) + strlen(content) + strlen(headers));
	
	sprintf(message,
			POST_FORMAT,
			"/",
			API_SERVER_URL,
			headers,
			strlen(content),
			content);
	
		int sizeResponse = 1024;
	
		char *response =  calloc(sizeResponse, sizeof(char));
	
		result = http_request(message, response, sizeResponse);
	
		free(b64Data);
		free(message);
		free(headers);
		free(response);
	
		return result;}

int rs232_read(unsigned char** data)
{
	
	char *headers = malloc(10 + strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(FUNCTION_READ_RS232))) ;
	sprintf(headers,
		HEADER_FORMAT_WRITE,
		API_FUNCTION_STR(FUNCTION_READ_RS232));
	
	char *query = malloc(10 + strlen(QUERY_FORMAT_RS232_READ) + strlen(API_SERVER_URL) + strlen(_friendly_name));
	sprintf(query, QUERY_FORMAT_RS232_READ, _friendly_name);	
	 	
	char *message = malloc(10 + strlen(GET_FORMAT) + strlen(API_SERVER_URL) + strlen(query) + strlen(headers));
	
	sprintf(message,
			GET_FORMAT,
			"/",
			query,
			API_SERVER_URL,
			headers);
	
		int sizeResponse = 1024;
	
		char *response =  calloc(sizeResponse, sizeof(char));
	 
		if(http_request(message, response, sizeResponse) != 0)
		{
			return -1 ;
		}
		
		if(get_response_payload(response, data) != 0)
		{
			return -1 ;
		}
	
		free(headers);
		free(message);
		free(query);
		free(response);
	
	return 0;
}


int init_rs232(const char *friendly_name, BAUDRATE baud_rate, DATABITS databits, STOPBITS stopbits, PARITY parity)
{	
	_friendly_name = friendly_name;
	
	int result = -1;
	
	char *headers = malloc(10 + strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(FUNCTION_INIT_RS232))) ;
	sprintf(headers,
		HEADER_FORMAT_WRITE,
		API_FUNCTION_STR(FUNCTION_INIT_RS232));
	 
	char *content = malloc(20 + strlen(JSON_FORMAT_INIT_RS232) + strlen(_friendly_name));

	sprintf(content, JSON_FORMAT_INIT_RS232, _friendly_name, baud_rate, databits, STOPBITS_STR(stopbits), PARITY_STR(parity));	
	
	char *message = malloc(10 + strlen(POST_FORMAT) + strlen(API_SERVER_URL) + strlen(content) + strlen(headers));
	
	sprintf(message,
			POST_FORMAT,
			"/",
			API_SERVER_URL,
			headers,
			strlen(content),
			content);
	
		int sizeResponse = 1024;
	
		char *response =  calloc(sizeResponse, sizeof(char));
	
		http_request(message, response, sizeResponse);
	
		if(strstr(response, "HTTP/1.0 200") != NULL)
		{
			result = 0 ;
		}
	
		free(message);
		free(headers);
		free(response);
	
		return result;
}


int adc_read()
{
	
	unsigned char *headers = malloc(10 + strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(FUNCTION_READ_ADC))) ;
	sprintf(headers,
		HEADER_FORMAT_WRITE,
		API_FUNCTION_STR(FUNCTION_READ_ADC));
	
	unsigned char *query = malloc(10 + strlen(QUERY_FORMAT_ADC_READ) + strlen(API_SERVER_URL) + strlen(_friendly_name));
	sprintf(query, QUERY_FORMAT_ADC_READ, _friendly_name);	
	 	
	unsigned char *message = malloc(10 + strlen(GET_FORMAT) + strlen(API_SERVER_URL) + strlen(query) + strlen(headers));
	
	sprintf(message,
			GET_FORMAT,
			"/",
			query,
			API_SERVER_URL,
			headers);
	
		int sizeResponse = 1024;
	
		unsigned char *response =  calloc(sizeResponse, sizeof(char));
	 
		if(http_request(message, response, sizeResponse) != 0)
		{
			return -1 ;
		}
	
		unsigned char* payload;
	
		int result = 0;
	
		if(get_response_payload(response, &payload) == 0)
		{
			result = (payload[0] << 24) + (payload[1] << 16) + (payload[2] << 8) + payload[3] ;	
		}
	
		free(headers);
		free(message);
		free(query);
		free(response);
		free(payload);
		
		return result;
}


int init_adc(const char *friendly_name)
{	
	_friendly_name = friendly_name;
	
	int result = -1;
	
	char *headers = malloc(10 + strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(FUNCTION_INIT_AI))) ;
	sprintf(headers,
		HEADER_FORMAT_WRITE,
		API_FUNCTION_STR(FUNCTION_INIT_AI));
	 
	char *content = malloc(20 + strlen(JSON_FORMAT_INIT_AI) + strlen(_friendly_name));

	sprintf(content, JSON_FORMAT_INIT_AI, _friendly_name);	
	
	char *message = malloc(10 + strlen(POST_FORMAT) + strlen(API_SERVER_URL) + strlen(content) + strlen(headers));
	
	sprintf(message,
			POST_FORMAT,
			"/",
			API_SERVER_URL,
			headers,
			strlen(content),
			content);
	
		int sizeResponse = 1024;
	
		char *response =  calloc(sizeResponse, sizeof(char));
	
		http_request(message, response, sizeResponse);
	
		if(strstr(response, "HTTP/1.0 200") != NULL)
		{
			result = 0 ;
		}
	
		free(message);
		free(headers);
		free(response);
	
		return result;
}



int init_rs485(const char *friendly_name, BAUDRATE baud_rate)
{
	return init_mode(friendly_name,
		baud_rate,
		FUNCTION_INIT_RS485);
}

int init_modbus(const char *friendly_name, BAUDRATE baud_rate)
{
	return init_mode(friendly_name,
		baud_rate,
		FUNCTION_INIT_MODBUS);	
}

int init_do(const char *friendly_name)
{
	return init_mode(friendly_name,
		0,
		FUNCTION_INIT_DO);	
}

int getTelemetrySASToken(unsigned char **destSasToken, EVENTHUB_SAS_TOKEN_TYPE sasType)
{
	
	char *function;
	
	if (sasType == http)
		function = API_FUNCTION_STR(FUNCTION_GET_EVENTHUB_SAS_TOKEN_HTTP);
	else
		function = API_FUNCTION_STR(FUNCTION_GET_EVENTHUB_SAS_TOKEN_SB);
	
	char *headers = malloc(strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(FUNCTION_GET_EVENTHUB_SAS_TOKEN_HTTP)));
	sprintf(headers,
		HEADER_FORMAT_WRITE,
		API_FUNCTION_STR(FUNCTION_GET_EVENTHUB_SAS_TOKEN_HTTP));
	 	
	char *message = malloc(strlen(GET_FORMAT) + strlen(API_SERVER_URL) + strlen(headers));
	
	sprintf(message,
		GET_FORMAT,
		"/",
		STRING_EMPTY,
		API_SERVER_URL,
		headers);
	
	int sizeResponse = 512;
	
	char *response =  calloc(sizeResponse, sizeof(char));
	 
	int result = http_request(message, response, sizeResponse);
	
	if (result == 0)
	{	
	
		get_response(response, &result, destSasToken, false);
	
		if (*destSasToken == NULL)
		{
			result = -1;
		}
	} 
	
	free(headers);
	free(message);
	free(response);
	
	return result;
}


int getTelemetryEndPoint(unsigned char **destTelemetryEndPoint)
{
	
	char *headers = malloc(strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(FUNCTION_GET_EVENTHUB_ENDPOINT)));
	sprintf(headers,
		HEADER_FORMAT_WRITE,
		API_FUNCTION_STR(FUNCTION_GET_EVENTHUB_SAS_TOKEN_HTTP));
	 	
	char *message = malloc(strlen(GET_FORMAT) + strlen(API_SERVER_URL) + strlen(headers));
	
	sprintf(message,
		GET_FORMAT,
		"/",
		STRING_EMPTY,
		API_SERVER_URL,
		headers);
	
	int sizeResponse = 512;
	
	char *response =  calloc(sizeResponse, sizeof(char));
	 
	int result = http_request(message, response, sizeResponse);
	
	if (result == 0)
	{	
	
		get_response(response, &result, destTelemetryEndPoint, false);
	
		if (*destTelemetryEndPoint == NULL)
		{
			result = -1;
		}
	}
	
	free(headers);
	free(message);
	free(response);
	
	return result;
}

int getManagementConnectionString(unsigned char ** connStrP)
{
	
	unsigned char * destSasTokenP;
	unsigned char * deviceIdP;
	unsigned char * iotHostP;
	
	char *headers = malloc(10 + strlen(HEADER_FORMAT_WRITE) + strlen(API_FUNCTION_STR(FUNCTION_GET_CONNECTION_STRING)));
	
	sprintf(headers,
		HEADER_FORMAT_WRITE,
		API_FUNCTION_STR(FUNCTION_GET_CONNECTION_STRING));
	 	
	char *message = malloc(10 + strlen(GET_FORMAT) + strlen(API_SERVER_URL) + strlen(headers));
	
	sprintf(message,
		GET_FORMAT,
		"/",
		STRING_EMPTY,
		API_SERVER_URL,
		headers);
	
	int sizeResponse = 512;
	
	char *response =  calloc(sizeResponse, sizeof(char));
	 
	int result = http_request(message, response, sizeResponse);
	
	if (result == 0)
	{	
	
		get_response(response, &result, connStrP, false);
	
		if (connStrP != NULL)
		{
			result = 0;
		}
		else 
		{
			result = -1;
		}
	}
	
	free(headers);
	free(message);
	free(response);
	
	return result;
}

