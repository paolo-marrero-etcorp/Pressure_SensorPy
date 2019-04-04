#include "utilities.h"
#include "http.h"
#include <stdarg.h>
#include <sys/ioctl.h>
#include <string.h>
#include "utilities.h"
#include <stdio.h>
#include <sys/epoll.h>
#include "digital.h"
#include <fcntl.h>
#include <pthread.h>
#include "stdlib.h"
#include <unistd.h>
#define VALUE "/value"
#define EDGE "/edge"
#define HW_PATH "/home/hw/"
#define JSON_FORMAT_FAILSAFE_STATE "{\"FRIENDLY_NAME\":\"%s\"}"



typedef struct
{
	EdgeCallback callback;
	char* path;
	int pipe_fd;
} ThreadParamsStruct;

static char* get_full_edge_path(char friendly_name[])
{
	return get_full_path(HW_PATH, friendly_name, EDGE);
}

static char* get_full_input_path(char friendly_name[])
{
	return get_full_path(HW_PATH, friendly_name, VALUE);
}

int enter_failsafe_state(char* friendly_name)
{
	unsigned char* payload;
	int return_code;	
	char* json_buffer = get_formated_string(JSON_FORMAT_FAILSAFE_STATE, friendly_name);
	
	int ret_val = http_put("PUT_DO_IN_FAILSAFE", json_buffer, &return_code, &payload);	
	free(json_buffer);

	if (ret_val == 0)
	{
		free(payload);
		ret_val = return_code;
	}
	return ret_val;
}

int digital_in_open(char friendly_name[])
{
	FILE * fp;
	char* full_path = get_full_input_path(friendly_name);
	fp = fopen(full_path, "r");
	free((void*)full_path);
	return (int)fp;
}

int digital_in_read(int handle)
{
	if (handle != 0)
	{
		char value[3];
		fseek((FILE*)handle, 0, SEEK_SET);
		fflush((FILE*)handle);
		fgets(value, sizeof(value), (FILE*)handle);
		
		if (strcmp("0\n", value) == 0)
		{
			return 0;
		}
		else if (strcmp("1\n", value) == 0)
		{
			return 1;
		}
		else
		{
			return (-1);	
		}
		return (0);
	}
	return (-1);	
}

void disabled_edge_detect(int pipe_handle)
{
	// Write a value to the pipe to stop the edge detect thread. The value of the string doesn't matter.
	write(pipe_handle, "0", 2);
	close(pipe_handle);
}

static void* edge_detect_thread(void* ptr)
{
	ThreadParamsStruct* params = (ThreadParamsStruct*)ptr;
	EdgeCallback callback = params->callback;

	int gpio_handle = (int)open(params->path, O_RDONLY | O_NONBLOCK);  
	int pipe_handle = params->pipe_fd;
	free(params->path);
	free(params);
	
	int epfd = epoll_create(1);
	
	struct epoll_event events_out[2];
	
	struct epoll_event events_in[2];
	
	events_in[0].events = EPOLLPRI | EPOLLIN | EPOLLET;
	events_in[0].data.fd = gpio_handle;	
	int epoll_gpio_ret = epoll_ctl(epfd, EPOLL_CTL_ADD, gpio_handle, &events_in[0]);
		
	events_in[1].events = EPOLLPRI | EPOLLIN | EPOLLET;
	events_in[1].data.fd = pipe_handle;		
	int epoll_pipe_ret = epoll_ctl(epfd, EPOLL_CTL_ADD, pipe_handle, &events_in[1]);
	
	if (epoll_gpio_ret == 0 && epoll_pipe_ret == 0)
	{
		int exit_loop = 0;
		while (!exit_loop) 
		{
			int n;
			int num_fd = epoll_wait(epfd, events_out, 2, -1);
			
			for (n = 0; n < num_fd; n++)
			{
				if (events_out[n].data.fd == pipe_handle)
				{
					exit_loop = 1;
					printf("exiting fd = %i\n", pipe_handle);
				}
				else if (events_out[n].data.fd == gpio_handle)
				{
					callback();	
				}
				else
				{
					exit_loop = 1;
				}
			}
		}		
	}

	close(pipe_handle);
	close(gpio_handle);
	close(epfd);
	return 0;	
}

int enable_edge_detect(EdgeDetectEnum edge, EdgeCallback callback, char* name)
{
	pthread_t thread1;
	int pipe_fd[2];
	if (pipe(pipe_fd) == -1)
	{
		return (-1);
	}
	
	int edge_handle = (int)open(get_full_edge_path(name), O_RDWR); 	
	if (edge_handle == -1)
	{
		return (-1);
	}
	
	char* edge_string;
	if (edge == RISING)
	{
		edge_string = "rising";		
	}
	else if (edge == FALLING)
	{
		edge_string = "falling";		
	}
	else if (edge == BOTH)
	{
		edge_string = "both";		
	}
	else
	{
		return (-1);	
	}
	write(edge_handle, edge_string, strlen(edge_string));
	close(edge_handle);
	
	ThreadParamsStruct* param_struct = (ThreadParamsStruct*)malloc(sizeof(ThreadParamsStruct));
	param_struct->callback = callback;
	param_struct->path = get_full_input_path(name);
	param_struct->pipe_fd = pipe_fd[0];
	int pthread_return = pthread_create(&thread1, NULL, edge_detect_thread, (void*)param_struct);
	if (pthread_return != 0)
	{
		return (-1);
	}
	return pipe_fd[1];
}

void digital_in_close(int handle)
{
	fclose((FILE*)handle);
}

int digital_out_open(char friendly_name[])
{
	FILE * fp;
	char* full_path;
	size_t str_len = strlen(HW_PATH) + strlen(friendly_name) + 1;
	full_path = (char*)malloc(str_len);

	if (full_path == 0)
	{
		return 0;
	}
	strcpy(full_path, HW_PATH);
	strcat(full_path, friendly_name);
	
	fp = fopen(full_path, "r+");
	free((void*)full_path);
	return (int)fp;
}

int digital_out_set(int handle)
{
	if (handle != 0)
	{
		int bytes_written = fprintf((FILE*)handle, "1");
		int seek_success = fseek((FILE*)handle, 0, SEEK_SET);
		if (bytes_written == 1  && seek_success == 0)
		{
			return (0);
		}
		else
		{
			return (1);
		}
	}
	return (1);
}

int digital_out_clear(int handle)
{
	if (handle != 0)
	{
		int bytes_written = fprintf((FILE*)handle, "0");
		int seek_success = fseek((FILE*)handle, 0, SEEK_SET);
		if (bytes_written == 1 && seek_success == 0)
		{
			return (0);
		}
		else
		{
			return (1);
		}
	}
	return (1);
}

void digital_out_close(int handle)
{
	fclose((FILE*)handle);
}

int sen_power_open(char friendly_name[])
{
	return digital_out_open(friendly_name);
}

int sen_power_set(int handle)
{
	return digital_out_set(handle);
}

int sen_power_clear(int handle)
{
	return digital_out_clear(handle);
}

void sen_power_close(int handle)
{
	digital_out_close(handle);
}
