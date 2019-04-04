#include <linux/serial.h>
#include <termios.h>
// #include <linux/termios.h>
#include <sys/ioctl.h>
#include "utilities.h"
#include <unistd.h>
#include <fcntl.h>
#include "serial_port.h"
#include "http.h"
#include <stdbool.h>
#include "utilities.h"
#include <stdlib.h>

#define SERIAL_PATH "/home/hw/"
#define CRTSCTS  020000000000
#define PORT_SETTINGS_FORMAT "{\"baud_rate\":\"%i\",\"parity\":\"%c\",\"data_bits\":%i,\"stop_bits\":%i}"

int get_port_settings(char* friendly_name, SerialPortSettings* port_settings)
{
	unsigned char* payload;
	int return_code;
	char* query_buffer = get_formated_string("?FRIENDLY_NAME=%s", friendly_name);
	
	int ret_val = http_get("GET_485_PORT_SETTINGS", query_buffer, &return_code, &payload);
	
	if (ret_val == 0)
	{
		if (return_code == 0)
		{
			int num_scanned = scan_json((char*)payload, PORT_SETTINGS_FORMAT, &port_settings->baudrate, &port_settings->parity, &port_settings->data_bits, &port_settings->stopbits);
			if (num_scanned == 4)
			{
				ret_val = return_code;
			}
			else
			{
				ret_val = -1;
			}	
		}
		else
		{
			ret_val = return_code;
		}
		
		free(payload);
	}
	
	free(query_buffer);
	return (ret_val);		

}

int serial485_open(char friendly_name[], 
	BaudRateEnum baud_rate, 
	ParityEnum parity, 
	StopBitsEnum stop_bits, 
	int data_bits)
{
	int serial_fd;
	struct termios tio;
	struct serial_rs485 rs485conf;
	unsigned int speed = B4800;
	char* full_path = get_full_path(SERIAL_PATH, friendly_name, "");
	/* open the serial port */
	serial_fd = open(full_path, O_RDWR  | O_NOCTTY | O_SYNC);
	if (serial_fd == -1)
	{
		return (-1);
	}

	/* get the current attributes of the serial port */
	if (tcgetattr(serial_fd, &tio) == -1)
	{
		return (-1);
	}

	switch (baud_rate)
	{
	case BAUD_4800:
		{
			speed = B4800;
			break;
		}		
	case BAUD_9600:
		{
			speed = B9600;
			break;
		}
	case BAUD_19200:
		{
			speed = B19200;
			break;
		}
	case BAUD_38400:
		{
			speed = B38400;
			break;
		}
	case BAUD_57600:
		{
			speed = B57600;
			break;
		}
	default:// BAUD_115200
		{
			speed = B115200;
			break;
		}	
	}	
	
	/* write the speed of the serial line */
	if (cfsetospeed(&tio, speed) < 0 || cfsetispeed(&tio, speed) < 0)
	{
		return (-1);
	}

	switch (parity)
	{
	case PARITY_ODD:
		{
			tio.c_cflag |= (PARENB | PARODD);
			break;
		}
	case PARITY_EVEN:
		{
			tio.c_cflag |= PARENB;
			tio.c_cflag &= ~PARODD;
			break;
		}		
	default:/// PARITY_NONE
		{
			tio.c_cflag &= ~PARENB;
			break;
		}
	}
	
	switch (stop_bits)
	{
	case STOP_2BIT:
		{
			tio.c_cflag |= CSTOPB;
			break;
		}		
	default:// STOP_1BIT
		{
			tio.c_cflag &= ~CSTOPB;
			break;
		}
	}
	
	// Clear the size bits
	tio.c_cflag &= ~CSIZE;
	
	switch (data_bits)
	{
	case 5:
		{
			tio.c_cflag |=  CS5;
			break;
		}
	case 6:
		{
			tio.c_cflag |=  CS6;
			break;
		}
	case 7:
		{
			tio.c_cflag |=  CS7;
			break;
		}
	default:/// 8
		{
			tio.c_cflag |=  CS8;
			break;
		}
	}
	// There are no flow pins so turn flow control off.
	tio.c_cflag &= ~CRTSCTS;
	// Enable receiver and ignore modem control lines.
	tio.c_cflag |= CREAD | CLOCAL;
	// Turn off software flow control
	tio.c_iflag &= ~(IXON | IXOFF | IXANY);
	
	tio.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	
	tcsetattr(serial_fd, TCSANOW, &tio);

	/* Set RS485 mode: */
	rs485conf.delay_rts_after_send = 0;
	rs485conf.delay_rts_before_send = 0;
	rs485conf.flags = 0;

	/* Set RTS on send and reset it after send */
	rs485conf.flags |= (SER_RS485_ENABLED | SER_RS485_RTS_AFTER_SEND | SER_RS485_RX_DURING_TX);

	if (ioctl(serial_fd, TIOCSRS485, &rs485conf) < 0) 
	{
		return (-1);
	}
	return (serial_fd);
}
