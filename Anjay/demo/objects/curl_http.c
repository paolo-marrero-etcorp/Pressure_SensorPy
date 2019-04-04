#if 0
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <curl/curl.h>
#include <curl/easy.h>

#define API_SERVER_URL "http://api_server:8002"
#define FUNC_ID_HEADER_STRING "ETC_FUNCTION:"

struct string {
	char *ptr;
	size_t len;
};

void init_string(struct string *s) {
	s->len = 0;
	s->ptr = malloc(s->len + 1);
	if (s->ptr == NULL) {
		fprintf(stderr, "malloc() failed\n");
		exit(EXIT_FAILURE);
	}
	s->ptr[0] = '\0';
}

size_t writefunc(void *ptr, size_t size, size_t nmemb, struct string *s)
{
	size_t new_len = s->len + size*nmemb;
	s->ptr = realloc(s->ptr, new_len + 1);
	if (s->ptr == NULL) {
		fprintf(stderr, "realloc() failed\n");
		exit(EXIT_FAILURE);
	}
	memcpy(s->ptr + s->len, ptr, size*nmemb);
	s->ptr[new_len] = '\0';
	s->len = new_len;

	return size*nmemb;
}

int http_get(char function_id[], char query_string[])
{
	CURLcode res;
	struct curl_slist *chunk = NULL;	
	CURL *curl_handle;
	struct string s;
	init_string(&s);	
	
	curl_global_init(CURL_GLOBAL_ALL);

	curl_handle = curl_easy_init();
	
	// Add the function id header entry
	char* header_string = malloc(strlen(FUNC_ID_HEADER_STRING) + strlen(function_id) + 1);
	strcpy(header_string, FUNC_ID_HEADER_STRING);
	strcat(header_string, function_id);
	chunk = curl_slist_append(chunk, header_string);
	res = curl_easy_setopt(curl_handle, CURLOPT_HTTPHEADER, chunk);

	// Concatenate the query string to the url string
	char* url_buffer = malloc(strlen(query_string) + strlen(API_SERVER_URL) + 1) ;
	strcpy(url_buffer, API_SERVER_URL);
	strcat(url_buffer, query_string);

	/* set URL to get */
	curl_easy_setopt(curl_handle, CURLOPT_URL, url_buffer);

	/* no progress meter please */
	curl_easy_setopt(curl_handle, CURLOPT_NOPROGRESS, 1L);

	/* send all data to this function  */
	curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, writefunc);

	/* we want the headers to this file handle */
	curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA, &s);

	/*
	 * Notice here that if you want the actual data sent anywhere else but
	 * stdout, you should consider using the CURLOPT_WRITEDATA option.  */

	/* get it! */
	curl_easy_perform(curl_handle);

	printf("%s\n", s.ptr);
	free(s.ptr);	
	
	/* cleanup curl stuff */
	curl_easy_cleanup(curl_handle);
	
	free(header_string);
	free(url_buffer);

	return 0;	
}

int main(void)
{
	

	return 0;
}

#endif