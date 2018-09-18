// Copyright (c) Extreme Telematics. All rights reserved.
#include "Utilities.h"
// #include "azure_c_shared_utility/string_tokenizer.h" 
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <float.h>

static const char * hexMap = "0123456789ABCDEF";


unsigned char *BinToHexStr(const char *data, int length)
{
	if (data == NULL)
		return NULL;
	 
	unsigned char* result = calloc(length * 4, sizeof(char));

	for (int i = 0; i < length; ++i) {
		result[2 * i] = hexMap[(data[i] & 0xF0) >> 4];
		result[2 * i + 1] = hexMap[data[i] & 0x0F];
	}

	return result;
}

unsigned char *HexStrToBin(const char * str)
{
	
	int blen = strlen(str) / 2;
	
	unsigned char *bytes = malloc(blen);
	
   uint8_t  pos;
   uint8_t  idx0;
   uint8_t  idx1;

   // mapping of ASCII characters to hex values
   const uint8_t hashmap[] =
   {
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  !"#$%&'
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ()*+,-./
     0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // 01234567
     0x08, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 89:;<=>?
     0x00, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, // @ABCDEFG
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // HIJKLMNO
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // PQRSTUVW
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // XYZ[\]^_
     0x00, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, // `abcdefg
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // hijklmno
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // pqrstuvw
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // xyz{|}~.
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // ........
   };

   bzero(bytes, blen);
   for (pos = 0; ((pos < (blen*2)) && (pos < strlen(str))); pos += 2)
   {
      idx0 = (uint8_t)str[pos+0];
      idx1 = (uint8_t)str[pos+1];
      bytes[pos/2] = (uint8_t)(hashmap[idx0] << 4) | hashmap[idx1];
   };

	return bytes;
}

/* 
JSON_DECODER_RESULT JSON_To_MultiTree(const char* source, MULTITREE_HANDLE* multiTreeHandle)
{
	size_t size = strlen(source);
	char* json = (char*)malloc(size + 1);

	(void)memcpy(json, source, size);
	json[size] = '\0';

	
	return JSONDecoder_JSON_To_MultiTree(json, multiTreeHandle);
}


MULTITREE_RESULT JSON_DeleteChild(MULTITREE_HANDLE* treeHandle, const char* childName)
{
	if (treeHandle == NULL)
	{
		return MULTITREE_INVALID_ARG;
	}
	
	return MultiTree_DeleteChild(treeHandle, childName);   
}

unsigned char *JSON_GetValue(const char* key, MULTITREE_HANDLE* multiTreeHandle)
{
	unsigned char *value;
	MULTITREE_HANDLE nameTreeNode;
	JSON_GetNode(key, multiTreeHandle, &nameTreeNode);
	JSON_GetNodeStringValue(&nameTreeNode, &value);
	return value;
}

MULTITREE_RESULT JSON_GetNodeStringValue(MULTITREE_HANDLE* multiTreeHandle, const void **destination)
{	
	unsigned char *value;
	MULTITREE_RESULT result = MultiTree_GetValue(multiTreeHandle, &value);
	if (result == MULTITREE_OK)
	{
		int length = strlen(value);
		*destination = calloc(length, sizeof(unsigned char));

		memcpy(*destination, value + 1, length - 2);
	}

	return result;
}

MULTITREE_RESULT JSON_GetNodeIntValue(MULTITREE_HANDLE* multiTreeHandle, const void **destination)
{
	unsigned char *value;
	MULTITREE_RESULT result = MultiTree_GetValue(multiTreeHandle, &value);
	if (result == MULTITREE_OK)
		*destination = (int *)atoi(value);

	return result;
}

MULTITREE_RESULT JSON_GetNode(const char* key, MULTITREE_HANDLE* srcNodeHandle, MULTITREE_HANDLE* childTreeHandle)
{
	return MultiTree_GetChildByName(srcNodeHandle, key, childTreeHandle);
}

static JSON_ENCODER_TOSTRING_RESULT toStringFunc(STRING_HANDLE destination, const void* value)
{

	if (STRING_concat(destination, value) != 0)
	{
		return JSON_ENCODER_ERROR;
	}

	return JSON_ENCODER_TOSTRING_OK;
}

JSON_ENCODER_RESULT JSONEncodeTree(MULTITREE_HANDLE treeHandle, STRING_HANDLE destination)
{ 
	return JSONEncoder_EncodeTree(treeHandle, destination, toStringFunc); 
}
*/
/* contains tailing NULL char */
unsigned char* ReadFile(const char *fileName)
{
	unsigned char *retValue;
	FILE *fd = fopen(fileName, "r");

	/*
	if (fd == NULL)
	{
		// retValue = NULL;
		CreateConfig();
	}*/
	
	fd = fopen(fileName, "r"); 
	
	
	if (fseek(fd, 0L, SEEK_END) != 0)
	{
		retValue = NULL;
	}
	else
	{
		long size = ftell(fd) + 1;
		if ((size <= 0) || (fseek(fd, 0L, SEEK_SET) != 0))
		{
			retValue = NULL;
		}
		else
		{
			retValue = calloc((size + 1), sizeof(char));
			if (retValue == NULL)
			{
			}
			else
			{
				unsigned char *result = fgets(retValue, size, fd);
				if (result == NULL)
				{
					free(retValue);
					retValue = NULL;
				}
				else
				{
					 
					retValue = result;
				}
			} 
		}


	}
	fclose(fd);

	return retValue;
}

/* Overwrites the file */
bool WriteFile(const char *fileName, const char* buff)
{
	FILE *fd = fopen(fileName, "w");

	if (fd == NULL)
	{
		return false;
	}
	else if (fwrite(buff, 1, strlen(buff), fd) <= 0)
	{
		printf("\n fwrite() failed\n");
		return false;
	}

	fclose(fd);

	return true;
}


unsigned char *StringReplace(const char* source, char target, char replacement)
{
	int len = strlen(source);
	char* destination = calloc(len, sizeof(char));
	int j = 0;
	for (int i = 0; i < len; i++)
	{
		if (source[i] != target)
		{
			destination[j] = source[i];
			j += 1;
		}
		else if (replacement != '\0')
		{
			destination[j] = replacement;
			j += 1;
		}

		if (i == len - 1)
		{
			destination[j] = '\0';
		}
	}

	return destination;
}


void StrReverse(char* begin, char* end) 
{	
	char aux;	
	while (end > begin)	
		aux = *end, *end --= *begin, *begin ++= aux;	
}
	
/*
STRING_HANDLE STRING_GetTokenValue(const char *input, const char *token)
{
	STRING_TOKENIZER_HANDLE tokenizer1 = NULL;
	STRING_HANDLE inputString = NULL;
	STRING_HANDLE tokenString = NULL;
	STRING_HANDLE valueString = NULL;
	STRING_HANDLE currentTokenString = NULL;
	
	if ((inputString = STRING_construct(input)) == NULL)
	{
		return NULL;
	}
	else if ((tokenizer1 = STRING_TOKENIZER_create(inputString)) == NULL)
	{
		return NULL;
	}
	else if ((tokenString = STRING_construct(token)) == NULL)
	{
		return NULL;
	}
	else if ((valueString = STRING_new()) == NULL)
	{
		return NULL;
	}
	else if ((currentTokenString = STRING_new()) == NULL)
	{
		return NULL;
	}
	else		
	{
		while ((STRING_TOKENIZER_get_next_token(tokenizer1, currentTokenString, "=") == 0))
		{
			if (STRING_TOKENIZER_get_next_token(tokenizer1, valueString, ";") != 0)
			{
				return NULL;
			}
			else if(STRING_compare(currentTokenString, tokenString) == 0)
			{
				return valueString;
			}
		}
		
	}
	
	return NULL;
}
*/

int TextToInt(uint8_t * buffer,
	int length,
	int64_t * dataP)
{
	uint64_t result = 0;
	int sign = 1;
	int i = 0;

	if (0 == length) return 0;

	if (buffer[0] == '-')
	{
		sign = -1;
		i = 1;
	}

	while (i < length)
	{
		if ('0' <= buffer[i] && buffer[i] <= '9')
		{
			if (result > (UINT64_MAX / 10)) return 0;
			result *= 10;
			result += buffer[i] - '0';
		}
		else
		{
			return 0;
		}
		i++;
	}

	if (result > INT64_MAX) return 0;

	if (sign == -1)
	{
		*dataP = 0 - result;
	}
	else
	{
		*dataP = result;
	}

	return 1;
}

int TextToFloat(uint8_t * buffer,
	int length,
	double * dataP)
{
	double result;
	int sign;
	int i;

	if (0 == length) return 0;

	if (buffer[0] == '-')
	{
		sign = -1;
		i = 1;
	}
	else
	{
		sign = 1;
		i = 0;
	}

	result = 0;
	while (i < length && buffer[i] != '.')
	{
		if ('0' <= buffer[i] && buffer[i] <= '9')
		{
			if (result > (DBL_MAX / 10)) return 0;
			result *= 10;
			result += (buffer[i] - '0');
		}
		else
		{
			break;
		}
		i++;
	}
	if (buffer[i] == '.')
	{
		double dec;

		i++;
		if (i == length) return 0;

		dec = 0.1;
		while (i < length)
		{
			if ('0' <= buffer[i] && buffer[i] <= '9')
			{
				if (result > (DBL_MAX - 1)) return 0;
				result += (buffer[i] - '0') * dec;
				dec /= 10;
			}
			else
			{
				return 0;
			}
			i++;
		}
	}

	*dataP = result * sign;
	return 1;
}

size_t IntToText(int64_t data,
	uint8_t * string,
	size_t length)
{
	int index;
	bool minus;
	size_t result;

	if (data < 0)
	{
		minus = true;
		data = 0 - data;
	}
	else
	{
		minus = false;
	}

	index = length - 1;
	do
	{
		string[index] = '0' + data % 10;
		data /= 10;
		index--;
	} while (index >= 0 && data > 0);

	if (data > 0) return 0;

	if (minus == true)
	{
		if (index == 0) return 0;
		string[index] = '-';
	}
	else
	{
		index++;
	}

	result = length - index;

	if (result < length)
	{
		memmove(string, string + index, result);
	}

	return result;
}

size_t FloatToText(double data,
	uint8_t * string,
	size_t length)
{
	size_t intLength;
	size_t decLength;
	int64_t intPart;
	double decPart;

	if (data <= (double)INT64_MIN || data >= (double)INT64_MAX) return 0;

	intPart = (int64_t)data;
	decPart = data - intPart;
	if (decPart < 0)
	{
		decPart = 1 - decPart;
	}
	else
	{
		decPart = 1 + decPart;
	}

	if (decPart <= 1 + FLT_EPSILON)
	{
		decPart = 0;
	}

	if (intPart == 0 && data < 0)
	{
		// deal with numbers between -1 and 0
		if(length < 4) return 0;    // "-0.n"
		string[0] = '-';
		string[1] = '0';
		intLength = 2;
	}
	else
	{
		intLength = utils_intToText(intPart, string, length);
		if (intLength == 0) return 0;
	}
	decLength = 0;
	if (decPart >= FLT_EPSILON)
	{
		double noiseFloor;

		if (intLength >= length - 1) return 0;

		noiseFloor = FLT_EPSILON;
		do
		{
			decPart *= 10;
			noiseFloor *= 10;
		} while (decPart - (int64_t)decPart > noiseFloor);

		decLength = utils_intToText(decPart, string + intLength, length - intLength);
		if (decLength <= 1) return 0;

		// replace the leading 1 with a dot
		string[intLength] = '.';
	}

	return intLength + decLength;
}