#ifndef UTILITIES_
#define UTILITIES_
#include <stdbool.h>

char* get_formated_string(char* format_string, ...);
char* get_full_path(char hw_base_path[], char friendly_name[], char file_name[]);
int scan_json(char* input_string, char* format_string, ...);
bool get_json_dict_string_key(const char* string, char* key, char* value);

#endif