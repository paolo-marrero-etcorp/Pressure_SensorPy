#include <stdarg.h>
#include <stddef.h>
#include "utilities.h"
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
void remove_white_space(char* string, char* result)
{
	int i;

	for (i = 0; i < strlen(string); i++)
	{
		if (!isspace(string[i]))
		{
			*result = string[i];
			result++;
		}
	}
	*result = '\0';
}

bool get_json_dict_string_key(const char* string, char* key, char* value)
{
	bool match = false;
	char* key_pointer = strstr(string, key);
	if (key_pointer)
	{
		char* colon = strstr(key_pointer, ":");
		if (colon)
		{
			char* first_parentheses = strstr(colon, "\"");
			if (first_parentheses)
			{
				char* value_pointer = first_parentheses + 1;
				char* second_parentheses = strstr(value_pointer, "\"");
				if (second_parentheses)
				{					
					int length = second_parentheses - value_pointer;
					if (length > 0)
					{
						strncpy(value, value_pointer, length);
					}
					value[length] = '\0';
					match = true;
				}
			}		
		}
	}
	return match;
}

int scan_json(char* input_string, char* format_string, ...)
{
	int num_var_filled = 0;
	va_list valist;
	va_start(valist, format_string);
	
	char* no_white_in_string = malloc(strlen(input_string) + 1);
	char* no_white_format_string = malloc(strlen(format_string) + 1);
	
	remove_white_space(input_string, no_white_in_string);
	remove_white_space(format_string, no_white_format_string);
	
	num_var_filled = vsscanf(no_white_in_string, no_white_format_string, valist);
	
	free(no_white_in_string);
	free(no_white_format_string);
	return num_var_filled;
}

char* get_formated_string(char* format_string, ...)
{
	va_list valist;
	va_start(valist, format_string);
	size_t  buffer_size = vsnprintf(NULL, 0, format_string, valist);
	char* content = malloc(buffer_size + 1);
	int ret_val = vsnprintf(content, buffer_size + 1, format_string, valist);	
	return content;
}

char* get_full_path(char hw_base_path[], char friendly_name[], char file_name[])
{
	size_t str_len = strlen(hw_base_path) + strlen(friendly_name) + strlen(file_name) + 1;
	char* full_path = (char*)malloc(str_len);

	if (full_path == 0)
	{
		return 0;
	}
	strcpy(full_path, hw_base_path);
	strcat(full_path, friendly_name);
	strcat(full_path, file_name);
	return full_path;	
}
