#include <stdbool.h>
#include <stdio.h> 
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <curl/curl.h>
#include <curl/easy.h>
#include "base_64.h"
#include "utilities.h"
#include <stdbool.h>

#define API_SERVER_URL_WITH_PORT "http://api_server:8002"
#define FUNC_ID_HEADER_STRING "ETC_FUNCTION:"

#define API_SERVER_URL "api_server"

#define API_PAYLOAD "\"PAYLOAD\": "

#define RX_DATA_FORMAT_STRING " { \"RESULT\" : %i, \"PAYLOAD\" : %[^\n]s"

#define API_SERVER_URL "api_server"

typedef struct 
{
	char *ptr;
	size_t len;
} string;

static int get_response(char* response, int *return_code, unsigned char** payload)
{	
	int ret_val = -1;
	char *payload_start = strstr(response, API_PAYLOAD);
	
	if (payload_start)
	{
		payload_start += strlen(API_PAYLOAD);
		int length_of_payload = strlen(payload_start);
		
		if (length_of_payload > 0)
		{
			*payload = malloc(length_of_payload + 1);
			
			if (*payload != 0)
			{
				int num_scanned = sscanf(response, RX_DATA_FORMAT_STRING, return_code, *payload);
			
				if (num_scanned == 2)
				{
					int payload_length = strlen((const char*)*payload);
					// sscanf isn't able to take off the trailing } of the json payload so it needs to be done manually
					// so we only return the payload
					if ((*payload)[payload_length - 1] == '}')
					{
						(*payload)[payload_length - 1] = '\0';
					}
					ret_val = 0;
				}
				else
				{
					free(*payload);
				}
			}
		}
	}
	return ret_val;
}

static int init_string(string *s) 
{
	s->len = 0;
	s->ptr = malloc(s->len + 1);
	if (s->ptr == NULL) 
	{
		return false;
	}
	s->ptr[0] = '\0';
	return true;
}

size_t writefunc(void *ptr, size_t size, size_t nmemb, string *s)
{
	size_t buf_length = size*nmemb;
	size_t new_len = s->len + buf_length;
	s->ptr = realloc(s->ptr, new_len + 1);
	if (s->ptr == NULL)
	{
		return 0;
	}
	memcpy(s->ptr + s->len, ptr, buf_length);
	s->ptr[new_len] = '\0';
	s->len = new_len;

	return buf_length;
}


void http_init(void)
{
	curl_global_init(CURL_GLOBAL_ALL);
}


int http_put(char function_id[], char payload[], int* return_code, unsigned char** returned_payload)
{
	int ret_val = -1;
	string s;
	
	if (!init_string(&s))
	{
		return ret_val;
	}

	char* header_string = malloc(strlen(FUNC_ID_HEADER_STRING) + strlen(function_id) + 1);
	if (header_string == 0)
	{
		free(s.ptr);
		return ret_val;
	}
	strcpy(header_string, FUNC_ID_HEADER_STRING);
	strcat(header_string, function_id);	
		
	CURL *put_handle = curl_easy_init();
	
	if (put_handle)
	{
		struct curl_slist *header_list = NULL;
		header_list = curl_slist_append(header_list, header_string);		
	
		if (header_list != 0)
		{	
			CURLcode curl_code = curl_easy_setopt(put_handle, CURLOPT_HTTPHEADER, header_list);		

			/* no progress meter*/
			if (curl_code == CURLE_OK)
				curl_code = curl_easy_setopt(put_handle, CURLOPT_NOPROGRESS, 1L);
			/* send all data to this function  */
			if (curl_code == CURLE_OK)
				curl_code = curl_easy_setopt(put_handle, CURLOPT_WRITEFUNCTION, writefunc);	
			if (curl_code == CURLE_OK)
				curl_code = curl_easy_setopt(put_handle, CURLOPT_URL, API_SERVER_URL_WITH_PORT);	
			/* Now specify the POST data */ 
			if (curl_code == CURLE_OK)
				curl_code = curl_easy_setopt(put_handle, CURLOPT_POSTFIELDS, payload);
			/* we want the headers to this file handle */
			if (curl_code == CURLE_OK)
				curl_code = curl_easy_setopt(put_handle, CURLOPT_WRITEDATA, &s); 
			/* Perform the request, res will get the return code */ 
			if (curl_code == CURLE_OK)
				curl_code = curl_easy_perform(put_handle);
			if (curl_code == CURLE_OK)
				ret_val = get_response(s.ptr, return_code, returned_payload);

			curl_slist_free_all(header_list);		
		}
		curl_easy_cleanup(put_handle);
	}
	
	free(header_string);
	free(s.ptr);

	return ret_val;	
}


int http_get(char function_id[], char query_string[], int* return_code, unsigned char** returned_payload)
{
	int ret_val = -1;

	string s;

	if (!init_string(&s))
		return ret_val;

	
	char* header_string = malloc(strlen(FUNC_ID_HEADER_STRING) + strlen(function_id) + 1);
	if (header_string == 0)
	{
		free(s.ptr);
		return ret_val;
	}
	strcpy(header_string, FUNC_ID_HEADER_STRING);
	strcat(header_string, function_id);		
		
	char* url_string = malloc(strlen(API_SERVER_URL_WITH_PORT) + strlen(query_string) + 1);
	if (url_string == 0)
	{
		free(s.ptr);
		free(header_string);
		return ret_val;	
	}
	strcpy(url_string, API_SERVER_URL_WITH_PORT);
	strcat(url_string, query_string);		
	
	
	CURL *get_handle = curl_easy_init();
	
	if (get_handle)
	{
		struct curl_slist *header_list = NULL;
		header_list = curl_slist_append(header_list, header_string);
	
		if (header_list != 0)
		{
			/* no progress meter*/
			CURLcode curl_code = curl_easy_setopt(get_handle, CURLOPT_NOPROGRESS, 1L);
			/* send all data to this function  */
			if (curl_code == CURLE_OK)
				curl_code = curl_easy_setopt(get_handle, CURLOPT_WRITEFUNCTION, writefunc);			
			
			if (curl_code == CURLE_OK)			
				curl_code = curl_easy_setopt(get_handle, CURLOPT_HTTPHEADER, header_list);
			
			/* set URL to get */
			if (curl_code == CURLE_OK)
				curl_code = curl_easy_setopt(get_handle, CURLOPT_URL, url_string);	
			
			/* we want the headers to this file handle */
			if (curl_code == CURLE_OK)
				curl_code = curl_easy_setopt(get_handle, CURLOPT_WRITEDATA, &s);

			/* get it! */
			if (curl_code == CURLE_OK)
				curl_code = curl_easy_perform(get_handle);
		
			if (curl_code == CURLE_OK)
				ret_val = get_response(s.ptr, return_code, returned_payload);
			
			curl_slist_free_all(header_list);
		}
		curl_easy_cleanup(get_handle);
	}
	

	free(url_string);
	free(header_string);
	free(s.ptr);
	
	return ret_val;	
}