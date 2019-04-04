#ifndef BASE_64_
#define BASE_64_

#include <stdlib.h>

char* base64_encode(const unsigned char* buffer, size_t length);
size_t calcDecodeLength(const char* b64input);
int base64_decode(char* b64message, unsigned char** buffer, int length);

#endif