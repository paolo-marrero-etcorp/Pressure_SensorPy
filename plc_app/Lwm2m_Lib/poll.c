/*******************************************************************************
 *
 * Copyright (c) 2018 Extreme Telematics Corp.
 *
 *******************************************************************************/

#include "lwm2mclient.h"
#include "liblwm2m.h"
#include "commandline.h"
#include "api_client.h"
#include "analog.h"
#include "digital.h"
#include "lwm2m.h"
#ifdef WITH_TINYDTLS
#include "dtlsconnection.h"
#else
#include "connection.h"
#endif

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/stat.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include "modbus.h"

lwm2m_context_t * lwm2mH = NULL;

#define LOW_PRI_LOG_INST_START 2

void waitFor(unsigned int secs) {
    unsigned int retTime = time(0) + secs;    // Get finishing time.
    while(time(0) < retTime);                // Loop until it arrives.
}

void *poll_plc(void *arg)
{
    thread_data_t *data = (thread_data_t *)arg;
    int result;
    int i;
    int poll_time;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];
    int poll_state = 0;
    uint16_t obj_30005_id = 0;
    uint16_t obj_30008_id = 0;
    uint16_t obj_30011_id = 0;
    uint16_t obj_30012_id = 0;
    poll_timer_low_pri = POLL_INTERVAL_LOW_PRIORITY;
    uint16_t obj_30008_low_pri_id = LOW_PRI_LOG_INST_START;
    uint16_t obj_30011_low_pri_id = LOW_PRI_LOG_INST_START;
    

#ifdef WITH_LOGS
        fprintf(stderr, "read all registers, t=0 ...\n");
#endif
    /* 
    poll_30000_object(data->objArray[PLC_30000_0BJ], data->lwm2mH);
    poll_30001_object(data->objArray[PLC_30001_0BJ], data->lwm2mH);
    poll_30002_object(data->objArray[PLC_30002_0BJ], data->lwm2mH);
    poll_30003_object(data->objArray[PLC_30003_0BJ], data->lwm2mH);
    poll_30004_object(data->objArray[PLC_30004_0BJ], data->lwm2mH);
    for (i = 0; i < 5; i++)
    {
        poll_30005_object(data->objArray[PLC_30005_0BJ], data->lwm2mH, i);
    }
    poll_30006_object(data->objArray[PLC_30006_0BJ], data->lwm2mH);
    poll_30007_object(data->objArray[PLC_30007_0BJ], data->lwm2mH);
    for (i = 0; i < 25; i++)
    {
        poll_30008_object(data->objArray[PLC_30008_0BJ], data->lwm2mH, i);
    }
    poll_30009_object(data->objArray[PLC_30009_0BJ], data->lwm2mH);
    poll_30010_object(data->objArray[PLC_30010_0BJ], data->lwm2mH);
    for (i = 0; i < 9; i++)
    {
        poll_30011_object(data->objArray[PLC_30011_0BJ], data->lwm2mH, i);
    } 
    */
    
    /*
     * We now enter in a polling thread while loop 
     */
    
    while (0 == poll_quit)
    {
        // waitFor(5);
        sleep(7);
        /* plc_app specific */
        read_discretes(FRIENDLY_NAME, modbus_stn_id, 1, 1, char_buf);
        if (char_buf[0] == 1)
        {
            operator_present = true;
        }
        else
        {
            if (operator_present == true)
            {
                operator_configured = true; /* operator was here but left */
                operator_present = false;
            }
        }
        switch (poll_state)
        {
        case 0:
            poll_30000_object(data->objArray[PLC_30000_0BJ], data->lwm2mH);
            break;
        case 1:
            poll_30001_object(data->objArray[PLC_30001_0BJ], data->lwm2mH);
            break;
        case 2:
            poll_30002_object(data->objArray[PLC_30002_0BJ], data->lwm2mH);
            break;
        case 3:
            poll_30003_object(data->objArray[PLC_30003_0BJ], data->lwm2mH);
            break;
        case 4:
            poll_30004_object(data->objArray[PLC_30004_0BJ], data->lwm2mH);
            break;
        case 5:
            switch (obj_30005_id)
            {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
                poll_30005_object(data->objArray[PLC_30005_0BJ], data->lwm2mH, obj_30005_id);
                obj_30005_id++;
                if (obj_30005_id > 4)
                    obj_30005_id = 0;
                break;  
            default:
                obj_30005_id = 0;
                break;
            }
            break;
        case 6:
            poll_30006_object(data->objArray[PLC_30006_0BJ], data->lwm2mH);
            break;
        case 7:
            poll_30007_object(data->objArray[PLC_30007_0BJ], data->lwm2mH);
            break;
        case 8:
            switch (obj_30008_id)
            {
            case 0:
            case 1:
                poll_30008_object(data->objArray[PLC_30008_0BJ], data->lwm2mH, obj_30008_id);
                obj_30008_id++;
                if (obj_30008_id > 1)
                    obj_30008_id = 0;
                break;  
            default:
                obj_30008_id = 0;
                break;
            }
            break;
        case 9:
            poll_30009_object(data->objArray[PLC_30009_0BJ], data->lwm2mH);
            break;
        case 10:
            poll_30010_object(data->objArray[PLC_30010_0BJ], data->lwm2mH);
            break;
        case 11: 
            switch (obj_30011_id)
            {
            case 0:
            case 1:
                poll_30011_object(data->objArray[PLC_30011_0BJ], data->lwm2mH, obj_30011_id);
                obj_30011_id++;
                if (obj_30011_id > 1)
                    obj_30011_id = 0;
                break;  
            default:
                obj_30011_id = 0;
                break;
            }
            break;
        case 12: 
            switch (obj_30012_id)
            {
            case 0:
            case 1:
                poll_30012_object(data->objArray[PLC_30012_0BJ], data->lwm2mH, obj_30012_id);
                obj_30012_id++;
                if (obj_30012_id > 1)
                    obj_30012_id = 0;
                break;  
            default:
                obj_30012_id = 0;
                break;
            }
            break;
        default:
            poll_state = 0;
            break;
        }
        poll_state++;
        if (poll_state > 11) // 30012 is not a polling item as per conversation with Valens 
            poll_state = 0;
        
        if (poll_timer_low_pri != 0)
        {
            poll_timer_low_pri--;
        }
        else
        {
            if (obj_30008_low_pri_id < 25)
            {
                poll_30008_object(data->objArray[PLC_30008_0BJ], data->lwm2mH, obj_30008_low_pri_id);
                obj_30008_low_pri_id++;
                if (obj_30011_low_pri_id < 9)
                {
                    poll_30011_object(data->objArray[PLC_30011_0BJ], data->lwm2mH, obj_30011_low_pri_id);
                    obj_30011_low_pri_id++;
                }
            }
            else
            {
                obj_30008_low_pri_id = LOW_PRI_LOG_INST_START;
                obj_30011_low_pri_id = LOW_PRI_LOG_INST_START;
                poll_timer_low_pri = POLL_INTERVAL_LOW_PRIORITY;                
            }
        }
    }
#ifdef WITH_LOGS
    fprintf(stdout, "Poller thread exiting ...\r\n");
#endif
    pthread_exit(NULL);
}
