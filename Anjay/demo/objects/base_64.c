#include <stdbool.h>
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

static const char basis_64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int Base64encode_len(int len)
{
	return ((len + 2) / 3 * 4) + 1;
}

char* base64_encode(const char *string, int len)
{
	char* encoded = malloc(Base64encode_len(len));
	int i;
	char *p;

	p = encoded;
	for (i = 0; i < len - 2; i += 3) {
		*p++ = basis_64[(string[i] >> 2) & 0x3F];
		*p++ = basis_64[((string[i] & 0x3) << 4) |
		                ((int)(string[i + 1] & 0xF0) >> 4)];
		*p++ = basis_64[((string[i + 1] & 0xF) << 2) |
		                ((int)(string[i + 2] & 0xC0) >> 6)];
		*p++ = basis_64[string[i + 2] & 0x3F];
	}
	if (i < len) {
		*p++ = basis_64[(string[i] >> 2) & 0x3F];
		if (i == (len - 1)) {
			*p++ = basis_64[((string[i] & 0x3) << 4)];
			*p++ = '=';
		}
		else {
			*p++ = basis_64[((string[i] & 0x3) << 4) |
			                ((int)(string[i + 1] & 0xF0) >> 4)];
			*p++ = basis_64[((string[i + 1] & 0xF) << 2)];
		}
		*p++ = '=';
	}

	*p++ = '\0';
	return encoded;
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

int base64_decode(char* b64message, unsigned char** buffer, int length) 
{
	BIO *bio, *b64;
	int decodeLen = calcDecodeLength(b64message);
	*buffer = (unsigned char*)malloc(decodeLen + 1);
	(*buffer)[decodeLen] = '\0';

	bio = BIO_new_mem_buf(b64message, -1);
	b64 = BIO_new(BIO_f_base64());
	bio = BIO_push(b64, bio);

	BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);    //Do not use newlines to flush buffer
	BIO_read(bio, *buffer, length);	
	BIO_free_all(bio);
	(*buffer)[decodeLen] = '\0';
	return (0); //success
}