/*******************************************************************************
 *
 * Copyright (c) 2013, 2014 Intel Corporation and others.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution.
 *
 * The Eclipse Public License is available at
 *    http://www.eclipse.org/legal/epl-v10.html
 * The Eclipse Distribution License is available at
 *    http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    David Navarro, Intel Corporation - initial API and implementation
 *    domedambrosio - Please refer to git log
 *    Fabien Fleutot - Please refer to git log
 *    Axel Lorente - Please refer to git log
 *    Achim Kraus, Bosch Software Innovations GmbH - Please refer to git log
 *    Pascal Rieux - Please refer to git log
 *    Ville Skyttä - Please refer to git log
 *    
 *******************************************************************************/

/*
 Copyright (c) 2013, 2014 Intel Corporation

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 THE POSSIBILITY OF SUCH DAMAGE.

 David Navarro <david.navarro@intel.com>

*/

/* September 13, 2018:
 * Adapted by Extreme Telematics Corp.
 * 
 * Implements an object for Alien2 Plunger Lift Controller Plunger Production Log Record
 *
 *                                           Multiple
 *  Object                         |  ID   | Instances | Mandatory |
 *  Plunger Lift Controller        | 30011 |    Yes    |    No     |
 *  Time and Velocity Settings
 *  Resources:
 *                                          Supported    Multiple
 *  Name                             | ID | Operations | Instances | Mandatory |  Type   | Range   |  Units         |           Description                                     |
 *  Log Save Time                    |  1 |     R      |    No     |    Yes    | Time    |         |                | App must convert time datum                               |
 *  Log Open Time                    |  2 |     R      |    No     |    No     | Integer |         | Seconds        |                                                           |
 *  Log Close Time                   |  3 |     R      |    No     |    No     | Integer |         | Seconds        |                                                           | 
 *  Log Vent Time                    |  4 |     R      |    No     |    No     | Integer |         | Seconds        |                                                           |
 *  Log Volume                       |  5 |     R      |    No     |    No     | Float   |         |                | See resource 17                                           |
 *  Log Number of Cycles             |  6 |     R      |    No     |    No     | Integer |         |                |                                                           |
 *  Log Normal Arrival Count         |  7 |     R      |    No     |    No     | Integer | 0-65535 |                |                                                           |
 *  Log Non-arrival Count            |  8 |     R      |    No     |    No     | Integer | 0-65535 |                |                                                           |
 *  Log Fast Trip Count              |  9 |     R      |    No     |    No     | Integer | 0-65535 |                |                                                           |
 *  Log Max Open Count               | 10 |     R      |    No     |    No     | Integer | 0-65535 |                |                                                           |
 *  Log Low Battery Count            | 11 |     R      |    No     |    No     | Integer | 0-65535 |                |                                                           |
 *  Log Operator Change Count        | 12 |     R      |    No     |    No     | Integer | 0-65535 |                |                                                           |
 *  Log Startup Count                | 13 |     R      |    No     |    No     | Integer | 0-65535 |                |                                                           |
 *  Log Minimum Average Velocity     | 14 |     R      |    No     |    No     | Float   |         |                | See resource 18                                           |
 *  Log Maximum Average Velocity     | 15 |     R      |    No     |    No     | Float   |         |                | See resource 18                                           |
 *  Log Minimum Surface Velocity     | 16 |     R      |    No     |    No     | Float   |         |                | See resource 18                                           |
 *  Log Maximun Surface Velocity     | 17 |     R      |    No     |    No     | Float   |         |                | See resource 18                                           |
 *  Log Volume Units                 | 18 |     R      |    No     |    No     | String  |         |                | Read from 0:0010, send "Mcf/d" or "e3m3/d" accordingly    |
 *  Log Velocity Units               | 19 |     R      |    No     |    No     | String  |         |                | Read from 0:0010, send "ft/min" or "m/min" accordingly    |
 *  Application Type                 | 5750 |   RW     |    No     |    No     | String  |         |                | Description of Production Log e.g. Total Production,      |
 *                                   |      |          |           |           |         |         |                | Daily Production 1, Daily Production 2, .... Daily        |
 *                                   |      |          |           |           |         |         |                | Production 8
 */

#include "liblwm2m.h"
#include "lwm2mclient.h"
#include "modbus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define PRV_TLV_BUFFER_SIZE 64
#define EPOCH_TIME_ADJ 946684800
/*
 * Multiple instance objects can use userdata to store data that will be shared between the different instances.
 * The lwm2m_object_t object structure - which represent every object of the liblwm2m as seen in the single instance
 * object - contain a chained list called instanceList with the object specific structure prv_instance_t:
 */
typedef struct _prv_instance_
{
    /*
     * The first two are mandatories and represent the pointer to the next instance and the ID of this one. The rest
     * is the instance scope user data (uint8_t test in this case)
     */
    struct _prv_instance_ * next;   // matches lwm2m_list_t::next
    uint16_t    shortID;               // matches lwm2m_list_t::id
    uint64_t    logSaveTime;
    uint64_t    logOpenTime;
    uint64_t    logCloseTime;
    uint64_t    logVentTime;
    double      logVolume;
    uint64_t    logNumberOfCycles;
    uint64_t    logNormalArrivalCount;
    uint64_t    logNonArrivalCount;
    uint64_t    logFastTripCount;
    uint64_t    logMaxOpenCount;
    uint64_t    logLowBatteryCount;
    uint64_t    logOperatorChangeCount;
    uint64_t    logStartupCount;
    double      logMinAverageVelocity;
    double      logMaxAverageVelocity;
    double      logMinSurfaceVelocity;
    double      logMaxSurfaceVelocity;
    char        logVolumeUnits[7];             // Resource 18: Read from 0:0010, send "Mcf/d" or "e3m3/d" = 1 accordingly
    char        logVelocityUnits[7];           // Resource 19: Read from 0:0010, send "ft/min" = 0 or "m/min" = 1 accordingly
    char        *applicationType;              // resource #5750
} prv_instance_t;

static void prv_output_buffer(uint8_t * buffer,
                              int length)
{
    int i;
    uint8_t array[16];

    i = 0;
    while (i < length)
    {
        int j;
        fprintf(stderr, "  ");

        memcpy(array, buffer+i, 16);

        for (j = 0 ; j < 16 && i+j < length; j++)
        {
            fprintf(stderr, "%02X ", array[j]);
        }
        while (j < 16)
        {
            fprintf(stderr, "   ");
            j++;
        }
        fprintf(stderr, "  ");
        for (j = 0 ; j < 16 && i+j < length; j++)
        {
            if (isprint(array[j]))
                fprintf(stderr, "%c ", array[j]);
            else
                fprintf(stderr, ". ");
        }
        fprintf(stderr, "\n");

        i += 16;
    }
}
uint8_t poll_30011_object(lwm2m_object_t *obj, lwm2m_context_t * lwm2mH, uint16_t instanceId)
{
    prv_instance_t * targetP;
    lwm2m_uri_t uri;
    int i;
    int result;
    int modbusReg;
    char char_uri[32];
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];   

    targetP = (prv_instance_t *)lwm2m_list_find(obj->instanceList, instanceId);
    /* This part is needed to trigger the watcher of the observable */
    sprintf(char_uri, "/30011/%d/1", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 901;    
        }
        else
        {
            modbusReg = 102 + (6 * (instanceId - 1)); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->logSaveTime = reg_buf[0];
            targetP->logSaveTime <<= 16;
            targetP->logSaveTime += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logSaveTime: %d\n", targetP->logSaveTime);
#endif    
    }
    sprintf(char_uri, "/30011/%d/2", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 907;    
        }
        else
        {
            modbusReg = 150 + (3 * (instanceId - 1)); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->logOpenTime = reg_buf[0];
            targetP->logOpenTime <<= 16;
            targetP->logOpenTime += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logOpenTime: %d\n", targetP->logOpenTime);
#endif    
    }
    sprintf(char_uri, "/30011/%d/3", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 910;    
        }
        else
        {
            modbusReg = 174 + (3 * (instanceId - 1)); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->logCloseTime = reg_buf[0];
            targetP->logCloseTime <<= 16;
            targetP->logCloseTime += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logCloseTime: %d\n", targetP->logCloseTime);
#endif    
    }
    sprintf(char_uri, "/30011/%d/4", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 925;    
        }
        else
        {
            modbusReg = 294 + (3 * (instanceId - 1)); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->logVentTime = reg_buf[0];
            targetP->logVentTime <<= 16;
            targetP->logVentTime += reg_buf[1];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }              
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logVentTime: %d\n", targetP->logVentTime);
#endif    
    }
    sprintf(char_uri, "/30011/%d/5", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 913;    
        }
        else
        {
            modbusReg = 198 + (2 * (instanceId - 1)); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
        if (result == 0)
        {
            targetP->logVolume = reg_buf[0];
            targetP->logVolume *= 65536;
            targetP->logVolume += reg_buf[1];
            targetP->logVolume /= 10;
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }              
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logVolume: %f\n", targetP->logVolume);
#endif    
    }
    sprintf(char_uri, "/30011/%d/6", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 915;    
        }
        else
        {
            modbusReg = 214 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logNumberOfCycles = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }               
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logNumberOfCycles: %d\n", targetP->logNumberOfCycles);
#endif    
    }
    sprintf(char_uri, "/30011/%d/7", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 916;    
        }
        else
        {
            modbusReg = 222 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logNormalArrivalCount = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }              
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logNormalArrivalCount: %d\n", targetP->logNormalArrivalCount);
#endif    
    }
    sprintf(char_uri, "/30011/%d/8", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 917;    
        }
        else
        {
            modbusReg = 230 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logNonArrivalCount = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }              
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logNonArrivalCount: %d\n", targetP->logNonArrivalCount);
#endif    
    }
    sprintf(char_uri, "/30011/%d/9", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 918;    
        }
        else
        {
            modbusReg = 238 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logFastTripCount = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }             
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logFastTripCount: %d\n", targetP->logFastTripCount);
#endif    
    }
    sprintf(char_uri, "/30011/%d/10", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 920;    
        }
        else
        {
            modbusReg = 254 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logMaxOpenCount = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }             
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logMaxOpenCount: %d\n", targetP->logMaxOpenCount);
#endif    
    }
    sprintf(char_uri, "/30011/%d/11", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 921;    
        }
        else
        {
            modbusReg = 262 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logLowBatteryCount = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }             
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logLowBatteryCount: %d\n", targetP->logLowBatteryCount);
#endif    
    }
    sprintf(char_uri, "/30011/%d/12", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 922;    
        }
        else
        {
            modbusReg = 270 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logOperatorChangeCount = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }             
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logOperatorChangeCount: %d\n", targetP->logOperatorChangeCount);
#endif    
    }
    sprintf(char_uri, "/30011/%d/13", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 923;    
        }
        else
        {
            modbusReg = 278 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logStartupCount = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }            
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logStartupCount: %d\n", targetP->logStartupCount);
#endif    
    }
    sprintf(char_uri, "/30011/%d/14", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 928;    
        }
        else
        {
            modbusReg = 318 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logMinAverageVelocity = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }           
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logMinAverageVelocity: %f\n", targetP->logMinAverageVelocity);
#endif    
    }
    sprintf(char_uri, "/30011/%d/15", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 929;    
        }
        else
        {
            modbusReg = 326 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logMaxAverageVelocity = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }         
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logMaxAverageVelocity: %f\n", targetP->logMaxAverageVelocity);
#endif    
    }
    sprintf(char_uri, "/30011/%d/16", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 930;    
        }
        else
        {
            modbusReg = 334 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logMinSurfaceVelocity = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }          
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logMinSurfaceVelocity: %f\n", targetP->logMinSurfaceVelocity);
#endif    
    }
    sprintf(char_uri, "/30011/%d/17", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        if (instanceId == 0)
        {
            modbusReg = 931;    
        }
        else
        {
            modbusReg = 342 + (instanceId - 1); 
        }
        result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
        if (result == 0)
        {
            targetP->logMaxSurfaceVelocity = reg_buf[0];
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }         
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logMaxSurfaceVelocity: %f\n", targetP->logMaxSurfaceVelocity);
#endif    
    }
    sprintf(char_uri, "/30011/%d/18", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
        if (result == 0)
        {
            memset(targetP->logVolumeUnits, 0x00, 7);
            if (char_buf[0] == 0x00)
            {
                strcpy(targetP->logVolumeUnits, "Mcf/d");
            }
            else
            {
                strcpy(targetP->logVolumeUnits, "e3m3/d");
            }                        
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }         
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logVolumeUnits: %s\n", targetP->logVolumeUnits);
#endif    
    }
    sprintf(char_uri, "/30011/%d/19", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
        if (result == 0)
        {
            memset(targetP->logVelocityUnits, 0x00, 7);
            if (char_buf[0] == 0x00)
            {
                strcpy(targetP->logVelocityUnits, "ft/min");
            }
            else
            {
                strcpy(targetP->logVelocityUnits, "m/min");
            }                        
        }
        else
        {
            return COAP_400_BAD_REQUEST;
        }          
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "logVelocityUnits: %s\n", targetP->logVelocityUnits);
#endif    
    }
    sprintf(char_uri, "/30011/%d/5750", instanceId);
    if (lwm2m_stringToUri(char_uri, (strlen(char_uri)), &uri))
    {
        lwm2m_resource_value_changed(lwm2mH, &uri);
#ifdef WITH_LOGS
        fprintf(stderr, "applicationType: %s\n", targetP->applicationType);
#endif    
    }
}
static uint8_t prv_read(uint16_t instanceId,
                        int * numDataP,
                        lwm2m_data_t ** dataArrayP,
                        lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;
    int i;
    int result;
    int modbusReg;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];
    
    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(20);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 20;
        (*dataArrayP)[0].id = 1;
        lwm2m_data_encode_int(targetP->logSaveTime + EPOCH_TIME_ADJ, *dataArrayP + 0);
        (*dataArrayP)[1].id = 2;
        lwm2m_data_encode_int(targetP->logOpenTime, *dataArrayP + 1);
        (*dataArrayP)[2].id = 3;
        lwm2m_data_encode_int(targetP->logCloseTime, *dataArrayP + 2);
        (*dataArrayP)[3].id = 4;
        lwm2m_data_encode_int(targetP->logVentTime, *dataArrayP + 3);
        (*dataArrayP)[4].id = 5;
        lwm2m_data_encode_float(targetP->logVolume, *dataArrayP + 4);
        (*dataArrayP)[5].id = 6;
        lwm2m_data_encode_int(targetP->logNumberOfCycles, *dataArrayP + 5);
        (*dataArrayP)[6].id = 7;
        lwm2m_data_encode_int(targetP->logNormalArrivalCount, *dataArrayP + 6);
        (*dataArrayP)[7].id = 8;
        lwm2m_data_encode_int(targetP->logNonArrivalCount, *dataArrayP + 7);
        (*dataArrayP)[8].id = 9;
        lwm2m_data_encode_int(targetP->logFastTripCount, *dataArrayP + 8);
        (*dataArrayP)[9].id = 10;
        lwm2m_data_encode_int(targetP->logMaxOpenCount, *dataArrayP + 9);
        (*dataArrayP)[10].id = 11;
        lwm2m_data_encode_int(targetP->logLowBatteryCount, *dataArrayP + 10);
        (*dataArrayP)[11].id = 12;
        lwm2m_data_encode_int(targetP->logOperatorChangeCount, *dataArrayP + 11);
        (*dataArrayP)[12].id = 13;
        lwm2m_data_encode_int(targetP->logStartupCount, *dataArrayP + 12);
        (*dataArrayP)[13].id = 14;
        lwm2m_data_encode_float(targetP->logMinAverageVelocity, *dataArrayP + 13);
        (*dataArrayP)[14].id = 15;
        lwm2m_data_encode_float(targetP->logMaxAverageVelocity, *dataArrayP + 14);
        (*dataArrayP)[15].id = 16;
        lwm2m_data_encode_float(targetP->logMinSurfaceVelocity, *dataArrayP + 15);
        (*dataArrayP)[16].id = 17;
        lwm2m_data_encode_float(targetP->logMaxSurfaceVelocity, *dataArrayP + 16);
        (*dataArrayP)[17].id = 18;  
        lwm2m_data_encode_string(targetP->logVolumeUnits, *dataArrayP + 17);
        (*dataArrayP)[18].id = 19; 
        lwm2m_data_encode_string(targetP->logVelocityUnits, *dataArrayP + 18);
        (*dataArrayP)[19].id = 5750;  //applicationType = 5750
        lwm2m_data_encode_string(targetP->applicationType, *dataArrayP + 19);
    }
    for (i = 0 ; i < *numDataP ; i++)
    {
        switch ((*dataArrayP)[i].id)
        {
        case 1:
            if (instanceId == 0)
            {
                modbusReg = 901;    
            }
            else
            {
                modbusReg = 102 + (6 * (instanceId - 1)); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
            if (result == 0)
            {
                targetP->logSaveTime = reg_buf[0];
                targetP->logSaveTime <<= 16;
                targetP->logSaveTime += reg_buf[1];
                lwm2m_data_encode_int(targetP->logSaveTime + EPOCH_TIME_ADJ, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 2:
            if (instanceId == 0)
            {
                modbusReg = 907;    
            }
            else
            {
                modbusReg = 150 + (3 * (instanceId - 1)); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
            if (result == 0)
            {
                targetP->logOpenTime = reg_buf[0];
                targetP->logOpenTime <<= 16;
                targetP->logOpenTime += reg_buf[1];
                lwm2m_data_encode_int(targetP->logOpenTime, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 3:
            if (instanceId == 0)
            {
                modbusReg = 910;    
            }
            else
            {
                modbusReg = 174 + (3 * (instanceId - 1)); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
            if (result == 0)
            {
                targetP->logCloseTime = reg_buf[0];
                targetP->logCloseTime <<= 16;
                targetP->logCloseTime += reg_buf[1];
                lwm2m_data_encode_int(targetP->logCloseTime, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 4:
            if (instanceId == 0)
            {
                modbusReg = 925;    
            }
            else
            {
                modbusReg = 294 + (3 * (instanceId - 1)); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
            if (result == 0)
            {
                targetP->logVentTime = reg_buf[0];
                targetP->logVentTime <<= 16;
                targetP->logVentTime += reg_buf[1];
                lwm2m_data_encode_int(targetP->logVentTime, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 5:
            if (instanceId == 0)
            {
                modbusReg = 913;    
            }
            else
            {
                modbusReg = 198 + (2 * (instanceId - 1)); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 2, reg_buf);
            if (result == 0)
            {
                targetP->logVolume = reg_buf[0];
                targetP->logVolume *= 65536;
                targetP->logVolume += reg_buf[1];
                targetP->logVolume /= 10;
                lwm2m_data_encode_float(targetP->logVolume, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 6:
            if (instanceId == 0)
            {
                modbusReg = 915;    
            }
            else
            {
                modbusReg = 214 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logNumberOfCycles = reg_buf[0];
                lwm2m_data_encode_int(targetP->logNumberOfCycles, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 7:
            if (instanceId == 0)
            {
                modbusReg = 916;    
            }
            else
            {
                modbusReg = 222 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logNormalArrivalCount = reg_buf[0];
                lwm2m_data_encode_int(targetP->logNormalArrivalCount, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 8:
            if (instanceId == 0)
            {
                modbusReg = 917;    
            }
            else
            {
                modbusReg = 230 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logNonArrivalCount = reg_buf[0];
                lwm2m_data_encode_int(targetP->logNonArrivalCount, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 9:
            if (instanceId == 0)
            {
                modbusReg = 918;    
            }
            else
            {
                modbusReg = 238 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logFastTripCount = reg_buf[0];
                lwm2m_data_encode_int(targetP->logFastTripCount, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 10:
            if (instanceId == 0)
            {
                modbusReg = 920;    
            }
            else
            {
                modbusReg = 254 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logMaxOpenCount = reg_buf[0];
                lwm2m_data_encode_int(targetP->logMaxOpenCount, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 11:
            if (instanceId == 0)
            {
                modbusReg = 921;    
            }
            else
            {
                modbusReg = 262 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logLowBatteryCount = reg_buf[0];
                lwm2m_data_encode_int(targetP->logLowBatteryCount, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 12:
            if (instanceId == 0)
            {
                modbusReg = 922;    
            }
            else
            {
                modbusReg = 270 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logOperatorChangeCount = reg_buf[0];
                lwm2m_data_encode_int(targetP->logOperatorChangeCount, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 13:
            if (instanceId == 0)
            {
                modbusReg = 923;    
            }
            else
            {
                modbusReg = 278 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logStartupCount = reg_buf[0];
                lwm2m_data_encode_int(targetP->logStartupCount, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 14:
            if (instanceId == 0)
            {
                modbusReg = 928;    
            }
            else
            {
                modbusReg = 318 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logMinAverageVelocity = reg_buf[0];
                lwm2m_data_encode_float(targetP->logMinAverageVelocity, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 15:
            if (instanceId == 0)
            {
                modbusReg = 929;    
            }
            else
            {
                modbusReg = 326 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logMaxAverageVelocity = reg_buf[0];
                lwm2m_data_encode_float(targetP->logMaxAverageVelocity, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 16:
            if (instanceId == 0)
            {
                modbusReg = 930;    
            }
            else
            {
                modbusReg = 334 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logMinSurfaceVelocity = reg_buf[0];
                lwm2m_data_encode_float(targetP->logMinSurfaceVelocity, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 17:
            if (instanceId == 0)
            {
                modbusReg = 931;    
            }
            else
            {
                modbusReg = 342 + (instanceId - 1); 
            }
            result = read_input(FRIENDLY_NAME, modbus_stn_id, modbusReg, 1, reg_buf);
            if (result == 0)
            {
                targetP->logMaxSurfaceVelocity = reg_buf[0];
                lwm2m_data_encode_float(targetP->logMaxSurfaceVelocity, *dataArrayP + i);    
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 18:
            result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
            if (result == 0)
            {
                memset(targetP->logVolumeUnits, 0x00, 7);
                if (char_buf[0] == 0x00)
                {
                    strcpy(targetP->logVolumeUnits, "Mcf/d");
                }
                else
                {
                    strcpy(targetP->logVolumeUnits, "e3m3/d");
                }                        
                lwm2m_data_encode_string(targetP->logVolumeUnits, *dataArrayP + i);
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 19:
            result = read_coils(FRIENDLY_NAME, modbus_stn_id, 10, 1, char_buf);
            if (result == 0)
            {
                memset(targetP->logVelocityUnits, 0x00, 7);
                if (char_buf[0] == 0x00)
                {
                    strcpy(targetP->logVelocityUnits, "ft/min");
                }
                else
                {
                    strcpy(targetP->logVelocityUnits, "m/min");
                }                        
                lwm2m_data_encode_string(targetP->logVelocityUnits, *dataArrayP + i);
            }
            else
            {
                return COAP_400_BAD_REQUEST;
            }                
            break;
        case 5750:
            lwm2m_data_encode_string(targetP->applicationType, *dataArrayP + i);
            break;
        default:
            return COAP_404_NOT_FOUND;
        }
    }
    return COAP_205_CONTENT;
}

static uint8_t prv_discover(uint16_t instanceId,
                            int * numDataP,
                            lwm2m_data_t ** dataArrayP,
                            lwm2m_object_t * objectP)
{
    int i;

    // is the server asking for the full object ?
    if (*numDataP == 0)
    {
        *dataArrayP = lwm2m_data_new(20);
        if (*dataArrayP == NULL) return COAP_500_INTERNAL_SERVER_ERROR;
        *numDataP = 20;
        (*dataArrayP)[0].id = 1;
        (*dataArrayP)[1].id = 2;
        (*dataArrayP)[2].id = 3;
        (*dataArrayP)[3].id = 4;
        (*dataArrayP)[4].id = 5;
        (*dataArrayP)[5].id = 6;
        (*dataArrayP)[6].id = 7;
        (*dataArrayP)[7].id = 8;
        (*dataArrayP)[8].id = 9;
        (*dataArrayP)[9].id = 10;
        (*dataArrayP)[10].id = 11;
        (*dataArrayP)[11].id = 12;
        (*dataArrayP)[12].id = 13;
        (*dataArrayP)[13].id = 14;
        (*dataArrayP)[14].id = 15;
        (*dataArrayP)[15].id = 16;
        (*dataArrayP)[16].id = 17;
        (*dataArrayP)[17].id = 18;
        (*dataArrayP)[18].id = 19;
        (*dataArrayP)[19].id = 5750;
    }
    else
    {
        for (i = 0; i < *numDataP; i++)
        {
            switch ((*dataArrayP)[i].id)
            {
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
            case 16:
            case 17:
            case 18:
            case 19:
            case 5750:
                break;
            default:
                return COAP_404_NOT_FOUND;
            }
        }
    }

    return COAP_205_CONTENT;
}

static uint8_t prv_write(uint16_t instanceId,
                         int numData,
                         lwm2m_data_t * dataArray,
                         lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;
    int i;
    int result;
    unsigned short reg_buf[MAX_REGS_TO_RES];
    unsigned char char_buf[MAX_REGS_TO_RES];
    
    targetP = (prv_instance_t *)lwm2m_list_find(objectP->instanceList, instanceId);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    for (i = 0 ; i < numData ; i++)
    {
        switch (dataArray[i].id)
        {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
        case 19:
            return COAP_405_METHOD_NOT_ALLOWED;
        case 5750:  // res == 5750
            {
                int16_t len;
                len = (dataArray)->value.asBuffer.length;
                if (len == 0)
                {
                    return COAP_400_BAD_REQUEST;
                }
                if (targetP->applicationType != NULL)
                {
                    lwm2m_free(targetP->applicationType);
                }
                targetP->applicationType = (char *)lwm2m_malloc(len + 1);
                memcpy(targetP->applicationType, (dataArray)->value.asBuffer.buffer, len);
                targetP->applicationType[len] = 0x00;
            }
            break;
        default:
            return COAP_404_NOT_FOUND;
        }
    }
    return COAP_204_CHANGED;
}

static uint8_t prv_delete(uint16_t id,
                          lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;

    objectP->instanceList = lwm2m_list_remove(objectP->instanceList, id, (lwm2m_list_t **)&targetP);
    if (NULL == targetP) return COAP_404_NOT_FOUND;

    lwm2m_free(targetP);

    return COAP_202_DELETED;
}

static uint8_t prv_create(uint16_t instanceId,
                          int numData,
                          lwm2m_data_t * dataArray,
                          lwm2m_object_t * objectP)
{
    prv_instance_t * targetP;
    uint8_t result;


    targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
    if (NULL == targetP) return COAP_500_INTERNAL_SERVER_ERROR;
    memset(targetP, 0, sizeof(prv_instance_t));

    targetP->shortID = instanceId;
    objectP->instanceList = LWM2M_LIST_ADD(objectP->instanceList, targetP);

    result = prv_write(instanceId, numData, dataArray, objectP);

    if (result != COAP_204_CHANGED)
    {
        (void)prv_delete(instanceId, objectP);
    }
    else
    {
        result = COAP_201_CREATED;
    }

    return result;
}

static uint8_t prv_exec(uint16_t instanceId,
                        uint16_t resourceId,
                        uint8_t * buffer,
                        int length,
                        lwm2m_object_t * objectP)
{
    int result;
    unsigned char buf[] = { 1, 0 };
    if (NULL == lwm2m_list_find(objectP->instanceList, instanceId)) return COAP_404_NOT_FOUND;

    switch (resourceId)
    {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 5750:
        return COAP_405_METHOD_NOT_ALLOWED;
    default:
        return COAP_404_NOT_FOUND;
    }
}

void display_30011_object(lwm2m_object_t * object)
{
#ifdef WITH_LOGS
    fprintf(stdout, "  /%u: Test object, instances:\r\n", object->objID);
    prv_instance_t * instance = (prv_instance_t *)object->instanceList;
    while (instance != NULL)
    {
        //fprintf(stdout, "    /%u/%u: shortId: %u, test: %u\r\n",
                //object->objID, instance->shortID,
                //instance->shortID, instance->test);
        fprintf(stdout, "    /%u/%u:", object->objID, instance->shortID);
        instance = (prv_instance_t *)instance->next;
    }
#endif
}

lwm2m_object_t * get_30011_object(void)
{
    lwm2m_object_t * obj30011;

    obj30011 = (lwm2m_object_t *)lwm2m_malloc(sizeof(lwm2m_object_t));

    if (NULL != obj30011)
    {
        int i;
        prv_instance_t * targetP;
        obj30011->objID = PLC_PRODUCTION_LOG_RECORD;
        for (i = 0; i < 9; i++)
        {
            targetP = (prv_instance_t *)lwm2m_malloc(sizeof(prv_instance_t));
            if (NULL == targetP) return NULL;
            memset(targetP, 0, sizeof(prv_instance_t));
            targetP->shortID                        = i;
            targetP->logSaveTime                    = 0;
            targetP->logOpenTime                    = 0;
            targetP->logCloseTime                   = 0;
            targetP->logVentTime                    = 0;
            targetP->logVolume                      = 0;
            targetP->logNumberOfCycles              = 0;
            targetP->logNormalArrivalCount          = 0;
            targetP->logNonArrivalCount             = 0;
            targetP->logFastTripCount               = 0;
            targetP->logMaxOpenCount                = 0;
            targetP->logLowBatteryCount             = 0;
            targetP->logOperatorChangeCount         = 0;
            targetP->logStartupCount                = 0;   
            targetP->logMinAverageVelocity          = 0;
            targetP->logMaxAverageVelocity          = 0;
            targetP->logMinSurfaceVelocity          = 0;
            targetP->logMaxSurfaceVelocity          = 0;
            targetP->applicationType                = NULL;
            memset(targetP->logVolumeUnits, 0x00, 7);
            memset(targetP->logVelocityUnits, 0x00, 7);
            obj30011->instanceList = LWM2M_LIST_ADD(obj30011->instanceList, targetP);
        }
        /*
         * From a single instance object, two more functions are available.
         * - The first one (createFunc) create a new instance and filled it with the provided informations. If an ID is
         *   provided a check is done for verifying his disponibility, or a new one is generated.
         * - The other one (deleteFunc) delete an instance by removing it from the instance list (and freeing the memory
         *   allocated to it)
         */
        obj30011->readFunc = prv_read;
        obj30011->discoverFunc = prv_discover;
        obj30011->writeFunc = prv_write;
        obj30011->executeFunc = prv_exec;
        obj30011->createFunc = prv_create;
        obj30011->deleteFunc = prv_delete;
        obj30011->userData = NULL;    // added by Phil after finding spurious data here. Explicitly NULL the pointer
    }

    return obj30011;
}

void free_30011_object(lwm2m_object_t * object)
{
    LWM2M_LIST_FREE(object->instanceList);
    if (object->userData != NULL)
    {
        lwm2m_free(object->userData);
        object->userData = NULL;
    }
    lwm2m_free(object);
}

