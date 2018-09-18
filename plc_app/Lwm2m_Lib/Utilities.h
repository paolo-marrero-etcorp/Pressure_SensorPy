#pragma once
// Copyright (c) Extreme Telematics. All rights reserved.
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <stddef.h>

// #include "multitree.h"
// #include "jsondecoder.h"
// #include "jsonencoder.h"
// #include "azure_c_shared_utility/strings.h"
// #include "azure_c_shared_utility/crt_abstractions.h"


#ifndef UTILITIES_H
#define UTILITIES_H

#ifdef __cplusplus
extern "C" {
#endif

	//typedef unsigned char	uint8_t;
	//typedef unsigned long long int	uint64_t;
	
	unsigned char *BinToHexStr(const char *data, int len);
	unsigned char *HexStrToBin(const char * str);
/*	
	JSON_DECODER_RESULT JSON_To_MultiTree(const char* json, MULTITREE_HANDLE* multiTreeHandle);
	unsigned char *JSON_GetValue(const char* key, MULTITREE_HANDLE* multiTreeHandle);
	MULTITREE_RESULT JSON_GetNode(const char* key, MULTITREE_HANDLE* srcNodeHandle, MULTITREE_HANDLE* childTreeHandle);
	MULTITREE_RESULT JSON_GetNodeStringValue(MULTITREE_HANDLE* multiTreeHandle, const void **destination);
	MULTITREE_RESULT JSON_GetNodeIntValue(MULTITREE_HANDLE* multiTreeHandle, const void **destination);
	JSON_ENCODER_RESULT JSONEncodeTree(MULTITREE_HANDLE treeHandle, STRING_HANDLE destination);
	MULTITREE_RESULT JSON_DeleteChild(MULTITREE_HANDLE* treeHandle, const char* childName);
*/
	unsigned char* ReadFile(const char *fileName);
	bool WriteFile(const char *fileName, const char* buff);
	unsigned char *StringReplace(const char* source, char target, char replacement);
	void StrReverse(char* begin, char* end);
	// STRING_HANDLE STRING_GetTokenValue(const char *input, const char *token);
	
	int TextToInt(uint8_t * buffer, int length, int64_t * dataP);
	int TextToFloat(uint8_t * buffer, int length, double * dataP);
	size_t IntToText(int64_t data, uint8_t * string, size_t length);
	size_t FloatToText(double data, uint8_t * string, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* UTILITIES_H */
