#ifndef HTTP_
#define HTTP_

void http_init(void);
int http_put(char function_id[], char payload[], int* return_code, unsigned char** returned_payload);
int http_get(char function_id[], char query_string[], int* return_code, unsigned char** returned_payload);

#endif