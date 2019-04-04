#ifndef LWM2M_H_
#define LWM2M_H_

/*
 * The morpheus uses the LWM2M (Lightweight machine to machine) protocol for external 
 * communication. LwM2M is a secure, efficient and deployable client-server protocol 
 * for managing resource constrained devices on a variety of networks.
 * The function in this file will give the connection information needed to connect 
 * to the ETC LWM2M server.
 * Applications will come with a pre-installed lwm2m library. The lib file is located 
 * at /usr/lib/liblwm2m.a and the header is /usr/include/lwm2m/liblwm2m.h. This library
 * file was built from code in the eclipse/wakaama github project located at 
 * https://github.com/eclipse/wakaama.
 **/

/* This structure is used as an input parameter to the get_lwm2m_connection_data
 * function for retrieving LWM2M connection information.
 **/ 
typedef struct
{
	char identity[128];
	char endpoint[128];
	// host_name is the url of the LWM2M server
	char host_name[256];
	// secret_key is the pre-shared secrect key
	char secret_key[64];	
} Lwm2mConnectionInfoStruct;

/*
 * This function will retrieve the information needed to make a connection
 * to the LWM2M server.
 * 
 * Parameters:
 *- Lwm2mConnectionInfoStruct* ConnectInfo: A pointer to an Lwm2mConnectionInfoStruct
 *   structure. The buffers in the struct will filled with the string information for
 *   connecting to the server.
 *	 
 * Return:
 *  0 if the connection information was successfully retrieved, -1 on failure. If
 *  the operation fails the strings in the Lwm2mConnectionInfoStruct structure are
 *  not valid.
 **/
int get_lwm2m_connection_data(Lwm2mConnectionInfoStruct* ConnectInfo);

#endif
